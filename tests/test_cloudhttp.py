"""HTTP / S3 兼容云通道（``HTTPCloudStore`` / ``SigV4Signer`` / ``CloudStoreServer``）的回归测试。

这一层要守住三件事：

* **签名要真的对**——不是"我们自己签、自己验"就完事。测试里带上了 **AWS 官方文档
  给出的 4 组示例向量**（规范请求哈希 + 已签头 + 最终签名三处逐字符比对），
  它们来自 Amazon S3 文档的"签名计算示例"一节，是唯一能证明本实现符合 SigV4
  规则的外部证据。
* **两类失败语义必须分开**——连不上 / 超时 / 服务端 5xx 是**传输级故障**（抛错
  并把 store 标记为不可达）；``404 对象不存在``是**正常回答**（返回 ``None``）；
  ``401/403`` 是**配置问题**（抛专门的鉴权异常）。把断网渲染成"云端是空的"是
  现场最难查的一类故障，所以这里逐条钉死。
* **多工位共用真的成立**——两个 ``CloudLink`` 连同一个参考服务端，一个部署程序
  另一个能列出并取回；换个命名空间又互不干扰。

刻意不用 mock 顶掉 HTTP：这一层的价值恰恰在"网络上会不会出岔子"，所以全部走真
socket + 真 ``http.server``。只有需要构造**服务端不该有的行为**（列举返回非 JSON、
返回结构不对、键不是字符串）时，才用文件末尾那个最简 :class:`RawServer` 逐条脚本化。
"""

import calendar
import hashlib
import json
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

import orolp as orp  # noqa: E402
from orolp import Robot  # noqa: E402
from orolp.cloudhttp import (  # noqa: E402
    CLOUD_HTTP_API_VERSION,
    SIGV4_ALGORITHM,
    SIGV4_EMPTY_PAYLOAD_SHA256,
    CloudAuthError,
    CloudHTTPError,
    CloudStoreServer,
    HTTPCloudStore,
    SigV4Signer,
    canonical_query_string,
    main as cloudhttp_main,
    payload_hash,
    uri_encode,
    verify_sigv4,
)
from orolp.twinsync import (  # noqa: E402
    CloudLink,
    LocalCloudStore,
    MemoryCloudStore,
    TwinError,
    TwinSnapshot,
    TwinTrace,
    compare_traces,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
URDF = REPO_ROOT / "models/ur/ur5e/ur5e.urdf"
Q0 = [0.0, -30.0, 30.0, 0.0, 60.0, 0.0]
PROGRAM_CODE = "// 抓取程序（中文注释）\nmovej([0,-30,30,0,60,0])\nmovej([10,-45,45,0,90,0])\n"

# --- AWS 官方示例向量（Amazon S3 文档「簽章計算示例」一节） ------------------- #
AWS_ACCESS_KEY = "AKIAIOSFODNN7EXAMPLE"
AWS_SECRET_KEY = "wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY"
AWS_TS = calendar.timegm(time.strptime("20130524T000000Z", "%Y%m%dT%H%M%SZ"))

AWS_VECTORS = {
    # 取对象前 10 字节（无载荷）
    "get_object": dict(
        method="GET",
        url="https://examplebucket.s3.amazonaws.com/test.txt",
        headers={"Range": "bytes=0-9"},
        body=None,
        canonical_hash="7344ae5b7ee6c3e7e6b0fe0640412a37625d1fbfff95c48bbb2dc43964946972",
        signed="host;range;x-amz-content-sha256;x-amz-date",
        signature="f0e8bdb87c964420e857bd35b5d6ed310bd44f0170aba48dd91039c6036bdb41",
    ),
    # 上传对象（带载荷，头里含 Date 与 x-amz-storage-class）
    "put_object": dict(
        method="PUT",
        url="https://examplebucket.s3.amazonaws.com/test%24file.text",
        headers={
            "Date": "Fri, 24 May 2013 00:00:00 GMT",
            "x-amz-storage-class": "REDUCED_REDUNDANCY",
        },
        body=b"Welcome to Amazon S3.",
        canonical_hash="9e0e90d9c76de8fa5b200d8c849cd5b8dc7a3be3951ddb7f6a76b4158342019d",
        signed="date;host;x-amz-content-sha256;x-amz-date;x-amz-storage-class",
        signature="98ad721746da40c64f1a55b78f14c238d841ea1380cd77a1b5971af0ece108bd",
    ),
    # 取桶生命周期（只带子资源名、没有值）
    "get_lifecycle": dict(
        method="GET",
        url="https://examplebucket.s3.amazonaws.com/?lifecycle",
        headers={},
        body=None,
        canonical_hash="9766c798316ff2757b517bc739a67f6213b4ab36dd5da2f94eaebf79c77395ca",
        signed="host;x-amz-content-sha256;x-amz-date",
        signature="fea454ca298b7da1c68078a5d1bdbfbbe0d65c699e0f91ac7a200a0136783543",
    ),
    # 列举对象（两个查询参数，必须按字母序重排）
    "list_objects": dict(
        method="GET",
        url="https://examplebucket.s3.amazonaws.com/?max-keys=2&prefix=J",
        headers={},
        body=None,
        canonical_hash="df57d21db20da04d7fa30298dd4488ba3a2b47ca3a489c74750e0f1e7df1b9b7",
        signed="host;x-amz-content-sha256;x-amz-date",
        signature="34b48302e7b5fa45bde8084f4b7868a86f0a534bc59db6670ed5711ef69dc6f7",
    ),
}


# --------------------------------------------------------------------------- #
# 夹具与工具
# --------------------------------------------------------------------------- #


def _free_port() -> int:
    """拿一个当前没人监听的端口（用于验收"连不上"这一路）。"""
    import socket as _socket

    with _socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


@pytest.fixture
def robot():
    return Robot(str(URDF), "UR5")


@pytest.fixture
def cloud_root(tmp_path):
    return tmp_path / "cloud"


@pytest.fixture
def server(cloud_root):
    """默认参考服务端（无鉴权、单机回环）。"""
    srv = CloudStoreServer(cloud_root)
    srv.start()
    yield srv
    srv.stop()


@pytest.fixture
def make_server(cloud_root):
    """按需构造额外服务端（鉴权 / 只读 / 故障注入各用各的）。"""
    made = []

    def _make(**kwargs):
        srv = CloudStoreServer(cloud_root, **kwargs)
        srv.start()
        made.append(srv)
        return srv

    yield _make
    for srv in made:
        srv.stop()


@pytest.fixture
def store(server):
    return HTTPCloudStore(f"http://{server.address}", "robotworld")


@pytest.fixture
def link(server, robot):
    client = CloudLink(
        HTTPCloudStore(f"http://{server.address}", "robotworld"), robot, namespace="cell1"
    )
    client.connect()
    return client


def _synthetic_trace(robot, program_name="TWIN_HTTP", source="plant"):
    """造一条小轨迹（规划/实测各 4 点，偏差递增）。"""
    snaps = []
    for i, t in enumerate([0.0, 0.5, 1.0, 1.5]):
        planned = np.asarray(Q0, dtype=np.float64) + i * 0.5
        actual = planned + 0.2
        snaps.append(
            TwinSnapshot(
                t=t,
                planned_q=planned,
                actual_q=actual,
                planned_pose=robot.forward_kinematics(planned),
                actual_pose=robot.forward_kinematics(actual),
                pos_dev_mm=1.0 + i * 0.1,
                rot_dev_deg=0.05,
                joint_dev_deg=0.2,
            )
        )
    return TwinTrace(
        snapshots=snaps,
        source=source,
        robot_name="UR5",
        program_name=program_name,
        notes="合成（测试用，非真机实测）",
    )


# --------------------------------------------------------------------------- #
# ① SigV4：与 AWS 官方向量逐字符比对
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("key", sorted(AWS_VECTORS))
def test_sigv4_matches_official_aws_vector(key):
    """规范请求哈希、已签头、最终签名三处都要与 AWS 文档一致。"""
    v = AWS_VECTORS[key]
    signer = SigV4Signer(AWS_ACCESS_KEY, AWS_SECRET_KEY, timestamp=AWS_TS)

    text, signed = signer.canonical_request_for(
        v["method"], v["url"], v["body"], v["headers"]
    )
    got_hash = hashlib.sha256(text.encode("utf-8")).hexdigest()
    assert got_hash == v["canonical_hash"], f"规范请求哈希不一致：\n{text}"

    headers = signer.sign(v["method"], v["url"], v["body"], v["headers"])
    assert headers["X-Amz-Date"] == "20130524T000000Z"
    assert headers["X-Amz-Content-Sha256"] == payload_hash(v["body"])
    assert headers["Authorization"] == (
        f"{SIGV4_ALGORITHM} "
        f"Credential={AWS_ACCESS_KEY}/20130524/us-east-1/s3/aws4_request,"
        f"SignedHeaders={v['signed']},"
        f"Signature={v['signature']}"
    )


def test_sigv4_empty_body_hash_matches_aws_constant():
    """无载荷时用的哈希就是 AWS 文档里那个常量。"""
    assert payload_hash(None) == SIGV4_EMPTY_PAYLOAD_SHA256
    assert payload_hash(b"") == SIGV4_EMPTY_PAYLOAD_SHA256


def test_sigv4_unsigned_payload_skips_hashing():
    signer = SigV4Signer(AWS_ACCESS_KEY, AWS_SECRET_KEY, unsigned_payload=True)
    headers = signer.sign("PUT", "https://examplebucket.s3.amazonaws.com/a.txt", b"x")
    assert headers["X-Amz-Content-Sha256"] == "UNSIGNED-PAYLOAD"


def test_sigv4_signature_changes_with_secret():
    a = SigV4Signer("AK", "secret-a", timestamp=AWS_TS).sign(
        "GET", "https://h.s3.amazonaws.com/o"
    )
    b = SigV4Signer("AK", "secret-b", timestamp=AWS_TS).sign(
        "GET", "https://h.s3.amazonaws.com/o"
    )
    assert a["Authorization"] != b["Authorization"]


def test_sigv4_signature_changes_with_query_but_not_header_order():
    """查询串参与签名；同内容的头换个书写顺序不应改变签名。"""
    signer = SigV4Signer("AK", "SK", timestamp=AWS_TS)
    a = signer.sign("GET", "https://h.s3.amazonaws.com/o?prefix=a")
    b = signer.sign("GET", "https://h.s3.amazonaws.com/o?prefix=b")
    assert a["Authorization"] != b["Authorization"]

    c = signer.sign("GET", "https://h.s3.amazonaws.com/o", None, {"X-B": 2, "X-A": 1})
    d = signer.sign("GET", "https://h.s3.amazonaws.com/o", None, {"X-A": 1, "X-B": 2})
    assert c["Authorization"] == d["Authorization"]


def test_sigv4_uses_current_time_when_timestamp_is_none():
    headers = SigV4Signer("AK", "SK").sign("GET", "https://h.s3.amazonaws.com/o")
    parsed = time.strptime(headers["X-Amz-Date"], "%Y%m%dT%H%M%SZ")
    assert abs(calendar.timegm(parsed) - time.time()) < 30


def test_sigv4_session_token_is_signed():
    """临时凭证：令牌必须进已签头，否则服务端看不到它。"""
    signer = SigV4Signer("AK", "SK", session_token="TOKEN123", timestamp=AWS_TS)
    headers = signer.sign("GET", "https://h.s3.amazonaws.com/o")
    assert "x-amz-security-token" in headers["Authorization"]
    assert headers["X-Amz-Security-Token"] == "TOKEN123"


def test_sigv4_requires_both_credentials():
    with pytest.raises(TwinError):
        SigV4Signer("", "SK")
    with pytest.raises(TwinError):
        SigV4Signer("AK", "")


def test_uri_encode_follows_aws_rules():
    """AWS 明确要求：只留 A-Za-z0-9-._~，空格编成 %20，斜杠按需。"""
    assert uri_encode("a b") == "a%20b"
    assert uri_encode("test$file.text") == "test%24file.text"
    assert uri_encode("photos/Jan/sample.jpg") == "photos%2FJan%2Fsample.jpg"
    assert uri_encode("photos/Jan/sample.jpg", encode_slash=False) == "photos/Jan/sample.jpg"
    assert uri_encode("a~b_c-d.e") == "a~b_c-d.e"


def test_canonical_query_string_sorts_and_encodes():
    assert canonical_query_string({"prefix": "a b", "max-keys": "2"}) == "max-keys=2&prefix=a%20b"
    assert canonical_query_string([("b", 2), ("a", 1)]) == "a=1&b=2"


def test_canonical_query_string_accepts_raw_query_text():
    """从 URL 解析出来的 query 是字符串——直接当键值对序列解包会炸。"""
    assert canonical_query_string("max-keys=2&prefix=J") == "max-keys=2&prefix=J"
    assert canonical_query_string("lifecycle=") == "lifecycle="
    assert canonical_query_string("lifecycle") == "lifecycle="
    assert canonical_query_string("") == ""
    assert canonical_query_string("?") == ""
    assert canonical_query_string(None) == ""


def test_canonical_headers_trim_and_squeeze_whitespace():
    signer = SigV4Signer(AWS_ACCESS_KEY, AWS_SECRET_KEY, timestamp=AWS_TS)
    text, signed = signer.canonical_request_for(
        "PUT",
        "https://examplebucket.s3.amazonaws.com/test%24file.text",
        b"Welcome to Amazon S3.",
        {"Date": "  Fri,  24 May 2013 00:00:00 GMT  "},
    )
    assert "date:Fri, 24 May 2013 00:00:00 GMT" in text
    assert signed.startswith("date;host;")


# --------------------------------------------------------------------------- #
# ② 服务端复算：证明签名真的覆盖了它宣称覆盖的东西
# --------------------------------------------------------------------------- #


def _verify(method, path, query, headers, body, **kw):
    return verify_sigv4(
        method, path, query, headers, body, access_key="AK", secret_key="SK", **kw
    )


def test_verify_sigv4_accepts_freshly_signed_request():
    signer = SigV4Signer("AK", "SK")
    url = "http://127.0.0.1:9/bucket/key.txt"
    headers = signer.sign("PUT", url, b"payload")
    ok, reason = _verify("PUT", "/bucket/key.txt", "", headers, b"payload")
    assert ok, reason


def test_verify_sigv4_rejects_tampered_body():
    """签名必须真的覆盖载荷——否则"签了名"就没有任何完整性含义。"""
    signer = SigV4Signer("AK", "SK")
    url = "http://127.0.0.1:9/bucket/key.txt"
    headers = signer.sign("PUT", url, b"payload")
    ok, reason = _verify("PUT", "/bucket/key.txt", "", headers, b"payload-tampered")
    assert not ok
    assert "载荷哈希" in reason


def test_verify_sigv4_rejects_tampered_path():
    signer = SigV4Signer("AK", "SK")
    headers = signer.sign("GET", "http://127.0.0.1:9/bucket/a.txt")
    ok, _ = _verify("GET", "/bucket/b.txt", "", headers, b"")
    assert not ok


def test_verify_sigv4_rejects_wrong_access_key():
    signer = SigV4Signer("AK-OTHER", "SK")
    headers = signer.sign("GET", "http://127.0.0.1:9/bucket/a.txt")
    ok, reason = _verify("GET", "/bucket/a.txt", "", headers, b"")
    assert not ok and "access_key" in reason


def test_verify_sigv4_rejects_wrong_region_scope():
    signer = SigV4Signer("AK", "SK", region="us-west-2")
    headers = signer.sign("GET", "http://127.0.0.1:9/bucket/a.txt")
    ok, reason = _verify("GET", "/bucket/a.txt", "", headers, b"")
    assert not ok and "作用域" in reason


def test_verify_sigv4_rejects_wrong_secret():
    signer = SigV4Signer("AK", "SK-WRONG")
    headers = signer.sign("GET", "http://127.0.0.1:9/bucket/a.txt")
    ok, reason = _verify("GET", "/bucket/a.txt", "", headers, b"")
    assert not ok and "签名不匹配" in reason


def test_verify_sigv4_reports_missing_authorization():
    ok, reason = _verify("GET", "/b/k", "", {}, b"")
    assert not ok and "缺少 Authorization" in reason


def test_verify_sigv4_rejects_other_algorithm():
    ok, reason = _verify("GET", "/b/k", "", {"Authorization": "AWS4-HMAC-SHA1 x=1"}, b"")
    assert not ok and "算法" in reason


def test_verify_sigv4_rejects_malformed_credential():
    ok, reason = _verify(
        "GET",
        "/b/k",
        "",
        {"Authorization": "AWS4-HMAC-SHA256 Credential=noslash,SignedHeaders=host,Signature=x"},
        b"",
    )
    assert not ok and "Credential 格式" in reason


def test_verify_sigv4_rejects_declared_header_that_is_missing():
    """声明签了某个头、实际却没发——服务端必须拒绝，不能"按缺省值凑"。"""
    ok, reason = _verify(
        "GET",
        "/b/k",
        "",
        {
            "Authorization": (
                "AWS4-HMAC-SHA256 Credential=AK/20130524/us-east-1/s3/aws4_request,"
                "SignedHeaders=host;x-amz-date;x-amz-nope,Signature=deadbeef"
            ),
            "host": "h",
            "x-amz-date": "20130524T000000Z",
        },
        b"",
    )
    assert not ok and "实际缺失" in reason


def test_verify_sigv4_rejects_payload_hash_mismatch():
    ok, reason = _verify(
        "PUT",
        "/b/k",
        "",
        {
            "Authorization": (
                "AWS4-HMAC-SHA256 Credential=AK/20130524/us-east-1/s3/aws4_request,"
                "SignedHeaders=host;x-amz-content-sha256;x-amz-date,Signature=deadbeef"
            ),
            "host": "h",
            "x-amz-date": "20130524T000000Z",
            "x-amz-content-sha256": "0" * 64,
        },
        b"real-body",
    )
    assert not ok and "载荷哈希" in reason


# --------------------------------------------------------------------------- #
# ③ 客户端最基本的四个动作
# --------------------------------------------------------------------------- #


def test_put_get_roundtrip(store):
    handle = store.put("a/b.txt", b"hello")
    assert handle == f"{store.base_url}/robotworld/a/b.txt"
    assert store.get("a/b.txt") == b"hello"
    assert store.exists("a/b.txt")
    assert store.get("a/none.txt") is None
    assert not store.exists("a/none.txt")


def test_empty_object_is_not_missing(store):
    """``b""`` 与"对象不存在"是两件事——把空程序当成缺失是典型事故。"""
    store.put("empty.src", b"")
    got = store.get("empty.src")
    assert got == b""
    assert got is not None


def test_delete_reports_whether_it_really_deleted(store):
    store.put("x.txt", b"1")
    assert store.delete("x.txt") is True
    assert store.delete("x.txt") is False
    assert store.get("x.txt") is None


def test_overwrite_replaces_content(store):
    store.put("p.src", b"v1")
    store.put("p.src", b"v2-longer")
    assert store.get("p.src") == b"v2-longer"


def test_keys_lists_and_sorts_by_prefix(store):
    store.put("progs/b.src", b"1")
    store.put("progs/a.src", b"2")
    store.put("progs/sub/c.src", b"3")
    store.put("traces/t.jsonl", b"4")
    assert store.keys("progs") == ["progs/a.src", "progs/b.src", "progs/sub/c.src"]
    assert store.keys() == [
        "progs/a.src",
        "progs/b.src",
        "progs/sub/c.src",
        "traces/t.jsonl",
    ]
    assert store.keys("nope") == []


def test_unicode_and_spaced_keys_roundtrip_through_urls(store):
    key = "程序 库/抓取 v2.src"
    handle = store.put(key, "中文内容".encode("utf-8"))
    assert " " not in handle and "%20" in handle
    assert store.get(key) == "中文内容".encode("utf-8")
    assert store.keys("程序 库") == ["程序 库/抓取 v2.src"]
    assert store.delete(key) is True


def test_traversal_key_is_rejected_before_any_request(store):
    """非法键根本不该上网——在客户端就拦掉，别把风险丢给服务端。"""
    before = store.requests
    for bad in ("../evil.txt", "a/../../evil.txt", ""):
        with pytest.raises(TwinError):
            store.put(bad, b"x")
        with pytest.raises(TwinError):
            store.get(bad)
    assert store.requests == before


def test_store_rejects_bad_base_url():
    with pytest.raises(TwinError):
        HTTPCloudStore("127.0.0.1:8080")  # 少了协议
    with pytest.raises(TwinError):
        HTTPCloudStore("http://")  # 少了主机
    with pytest.raises(TwinError):
        HTTPCloudStore("")


def test_from_url_extracts_bucket():
    store = HTTPCloudStore.from_url("http://10.0.0.5:9000/cell-a")
    assert store.base_url == "http://10.0.0.5:9000"
    assert store.bucket == "cell-a"
    assert HTTPCloudStore.from_url("http://h:1").bucket == "robotworld"
    with pytest.raises(TwinError):
        HTTPCloudStore.from_url("http://h:1/a/b")


def test_describe_and_repr(store):
    store.put("k", b"v")
    info = store.describe()
    assert info["protocol"] == "http"
    assert info["api_version"] == CLOUD_HTTP_API_VERSION
    assert info["bucket"] == "robotworld"
    assert info["auth"] == "none"
    assert info["reachable"] is True and info["requests"] >= 1
    assert info["last_error"] is None
    assert "robotworld" in repr(store)


# --------------------------------------------------------------------------- #
# ④ 契约一致性：三种传输对同一套断言给同样的回答
# --------------------------------------------------------------------------- #


def _assert_transport_contract(store):
    """CloudTransport 的契约——每种实现都必须过这一套。"""
    assert store.get("nope.txt") is None
    assert store.delete("nope.txt") is False
    assert store.keys() == []

    store.put("b/2.txt", b"two")
    store.put("b/1.txt", b"one")
    assert store.keys() == ["b/1.txt", "b/2.txt"]
    assert store.keys("b/1") == ["b/1.txt"]
    assert store.get("b/1.txt") == b"one"
    assert store.exists("b/2.txt")
    assert store.delete("b/2.txt") is True
    assert store.keys() == ["b/1.txt"]
    with pytest.raises(TwinError):
        store.put("../escape.txt", b"x")


def test_contract_memory_store():
    _assert_transport_contract(MemoryCloudStore())


def test_contract_local_store(tmp_path):
    _assert_transport_contract(LocalCloudStore(tmp_path / "local"))


def test_contract_http_store(store):
    _assert_transport_contract(store)


def test_three_stores_agree_on_the_same_sequence(tmp_path, server):
    """同一串操作喂三种实现，最终应当得到同一个键集合——接口一致性的硬证据。"""
    stores = [
        MemoryCloudStore(),
        LocalCloudStore(tmp_path / "local2"),
        HTTPCloudStore(f"http://{server.address}", "robotworld"),
    ]
    ops = ["put", "put", "put", "delete", "put"]
    for i, op in enumerate(ops):
        key = f"{i % 2}/item{i}.txt"
        for s in stores:
            if op == "put":
                s.put(key, f"v{i}".encode())
            else:
                s.delete(key)
    snapshots = [s.keys() for s in stores]
    assert snapshots[0] == snapshots[1] == snapshots[2]


# --------------------------------------------------------------------------- #
# ⑤ 鉴权：401 / 403 是配置问题，必须响亮
# --------------------------------------------------------------------------- #


def test_missing_token_is_auth_error(make_server):
    srv = make_server(token="s3cr3t")
    store = HTTPCloudStore(f"http://{srv.address}", "robotworld")
    with pytest.raises(CloudAuthError) as exc:
        store.get("x.txt")
    assert exc.value.status == 401
    assert isinstance(exc.value, CloudHTTPError)  # 也是传输级异常，便于统一兜底


def test_correct_token_is_accepted(make_server):
    srv = make_server(token="s3cr3t")
    store = HTTPCloudStore(f"http://{srv.address}", "robotworld", token="s3cr3t")
    store.put("x.txt", b"ok")
    assert store.get("x.txt") == b"ok"
    assert store.describe()["auth"] == "bearer"


def test_read_only_store_refuses_writes(make_server):
    srv = make_server(read_only=True)
    store = HTTPCloudStore(f"http://{srv.address}", "robotworld")
    with pytest.raises(CloudAuthError) as exc:
        store.put("x.txt", b"nope")
    assert exc.value.status == 403
    assert "只读" not in str(exc.value)  # 服务端的拒绝先于只读检查：这里只验状态码与类型


def test_sigv4_end_to_end_against_verifying_server(make_server):
    """服务端按真实 S3 的方式复算签名——这是"签名可用"的运行时证据。"""
    srv = make_server(
        s3={"access_key": AWS_ACCESS_KEY, "secret_key": AWS_SECRET_KEY, "region": "us-east-1"}
    )
    signer = SigV4Signer(AWS_ACCESS_KEY, AWS_SECRET_KEY, region="us-east-1")
    store = HTTPCloudStore(f"http://{srv.address}", "robotworld", signer=signer)
    store.put("sig/p.txt", b"signed-body")
    assert store.get("sig/p.txt") == b"signed-body"
    assert store.keys("sig") == ["sig/p.txt"]
    assert store.delete("sig/p.txt") is True
    assert store.describe()["auth"] == "sigv4"


def test_sigv4_wrong_secret_is_rejected_by_server(make_server):
    srv = make_server(
        s3={"access_key": AWS_ACCESS_KEY, "secret_key": AWS_SECRET_KEY}
    )
    store = HTTPCloudStore(
        f"http://{srv.address}",
        "robotworld",
        signer=SigV4Signer(AWS_ACCESS_KEY, "WRONG-SECRET"),
    )
    with pytest.raises(CloudAuthError) as exc:
        store.get("x.txt")
    assert exc.value.status == 403
    assert "签名" in str(exc.value)


# --------------------------------------------------------------------------- #
# ⑥ 两类失败语义
# --------------------------------------------------------------------------- #


def test_unreachable_cloud_is_a_transport_error_with_cli_hint():
    """连不上 ≠ 云端是空的。这一条是现场最难查的故障形态。"""
    store = HTTPCloudStore(f"http://127.0.0.1:{_free_port()}", "robotworld", timeout=1.0)
    with pytest.raises(CloudHTTPError) as exc:
        store.keys()
    assert not isinstance(exc.value, CloudAuthError)
    assert "不可达" in str(exc.value)
    assert "orolp.cloudhttp" in str(exc.value)  # 直接告诉用户怎么在本机起一个
    assert store.reachable is False
    assert store.last_error is not None


def test_server_5xx_is_a_transport_error(make_server):
    srv = make_server(fail_ops={"PUT"})
    store = HTTPCloudStore(f"http://{srv.address}", "robotworld")
    with pytest.raises(CloudHTTPError) as exc:
        store.put("x.txt", b"v")
    assert exc.value.status == 503
    assert store.reachable is False


def test_missing_object_keeps_store_reachable(store):
    """404 是**正常回答**：对象不存在。别把它渲染成"云端离线"。"""
    assert store.get("ghost.txt") is None
    assert store.reachable is True
    assert store.last_error is None


def test_success_after_failure_clears_last_error(make_server):
    srv = make_server(fail_ops={"GET": 1})
    store = HTTPCloudStore(f"http://{srv.address}", "robotworld")
    with pytest.raises(CloudHTTPError):
        store.keys()
    assert store.reachable is False
    assert store.keys() == []
    assert store.reachable is True and store.last_error is None


def test_garbage_list_body_is_an_error_not_an_empty_result(make_server):
    """把"解析不了"当成"一个对象都没有"是最危险的静默降级。"""
    srv = make_server(garbage_list=True)
    store = HTTPCloudStore(f"http://{srv.address}", "robotworld")
    with pytest.raises(CloudHTTPError) as exc:
        store.keys()
    assert "不是合法 JSON" in str(exc.value)


def test_list_body_with_wrong_structure_is_rejected():
    with RawServer({"/robotworld": (200, b'{"objects": []}')}) as srv:
        store = HTTPCloudStore(srv.base_url, "robotworld")
        with pytest.raises(CloudHTTPError) as exc:
            store.keys()
        assert "结构不对" in str(exc.value)


def test_list_body_with_non_string_keys_is_rejected():
    with RawServer({"/robotworld": (200, json.dumps({"keys": [1, 2]}).encode())}) as srv:
        store = HTTPCloudStore(srv.base_url, "robotworld")
        with pytest.raises(CloudHTTPError) as exc:
            store.keys()
        assert "非字符串键" in str(exc.value)


def test_object_too_large_is_rejected(make_server):
    srv = make_server(max_put_bytes=16)
    store = HTTPCloudStore(f"http://{srv.address}", "robotworld")
    store.put("small.txt", b"x" * 8)
    with pytest.raises(CloudHTTPError) as exc:
        store.put("big.txt", b"x" * 64)
    assert exc.value.status == 413
    assert store.get("big.txt") is None


def test_slow_server_hits_client_timeout(make_server):
    srv = make_server(latency=0.5)
    store = HTTPCloudStore(f"http://{srv.address}", "robotworld", timeout=0.12)
    with pytest.raises(CloudHTTPError) as exc:
        store.keys()
    assert "不可达" in str(exc.value)
    assert store.reachable is False


# --------------------------------------------------------------------------- #
# ⑦ 重试语义
# --------------------------------------------------------------------------- #


def test_retries_recover_from_transient_failure(make_server):
    """服务端偶发 503（滚动重启/限流）时，幂等请求应当自己挺过去。

    契约：``retries=N`` = 失败后最多再试 N 次（总尝试 1+N）。失败次数 ≤ N 就要
    恢复而不是抛错——原断言把"恢复"写成"抛错"，与本文件顶部及模块文档描述的
    重试语义自相矛盾，按契约修正。
    """
    srv = make_server(fail_ops={"GET": 2})
    store = HTTPCloudStore(f"http://{srv.address}", "robotworld", retries=2)
    assert store.keys() == []  # 前两次 503 被重试吃掉，第三次成功
    assert store.requests == 3  # 1 次首发 + 2 次重试
    assert store.keys() == []  # 故障注入耗尽后一切正常
    assert store.requests == 4


def test_retries_exhausted_still_raises(make_server):
    srv = make_server(fail_ops={"GET": -1})
    store = HTTPCloudStore(f"http://{srv.address}", "robotworld", retries=2)
    with pytest.raises(CloudHTTPError):
        store.keys()
    assert store.requests == 3


def test_4xx_is_never_retried(make_server):
    """鉴权错重试一万次也还是错，只会拖慢界面。"""
    srv = make_server(read_only=True)
    store = HTTPCloudStore(f"http://{srv.address}", "robotworld", retries=3)
    with pytest.raises(CloudAuthError):
        store.put("x.txt", b"v")
    assert store.requests == 1


# --------------------------------------------------------------------------- #
# ⑧ 并发 + 多工位共用（HTTP 传输真正的价值所在）
# --------------------------------------------------------------------------- #


def test_concurrent_puts_all_land(server):
    store = HTTPCloudStore(f"http://{server.address}", "robotworld")
    errors: list = []

    def worker(tag):
        try:
            for i in range(15):
                store.put(f"conc/{tag}-{i:02d}.txt", f"{tag}{i}".encode())
        except Exception as exc:  # pragma: no cover - 有错就报
            errors.append(exc)

    threads = [threading.Thread(target=worker, args=(t,)) for t in ("a", "b", "c")]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=20)
    assert errors == []
    assert len(store.keys("conc")) == 45
    assert store.get("conc/b-07.txt") == b"b7"


def test_two_workstations_share_one_program_library(server, robot):
    """现场真正要的：A 工位部署的程序，B 工位能列出来并取回同一份代码。"""
    a = CloudLink(HTTPCloudStore(f"http://{server.address}", "robotworld"), robot, namespace="cellA")
    b = CloudLink(HTTPCloudStore(f"http://{server.address}", "robotworld"), robot, namespace="cellA")
    a.connect()
    b.connect()

    a.upload_program("PICK", PROGRAM_CODE, {"ext": "urp", "brand": "UR", "post_processor": "ur_polyscope"})
    assert b.list_programs() == ["PICK"]
    assert b.fetch_program("PICK", "urp") == PROGRAM_CODE
    manifest = b.program_manifest("PICK")
    assert manifest["brand"] == "UR"
    assert manifest["lines"] == 3
    assert manifest["bytes"] == len(PROGRAM_CODE.encode("utf-8"))


def test_different_namespaces_do_not_see_each_other(server, robot):
    a = CloudLink(HTTPCloudStore(f"http://{server.address}", "robotworld"), namespace="cellA")
    b = CloudLink(HTTPCloudStore(f"http://{server.address}", "robotworld"), namespace="cellB")
    a.connect()
    b.connect()
    a.upload_program("ONLY_A", PROGRAM_CODE, {"ext": "src"})
    assert a.list_programs() == ["ONLY_A"]
    assert b.list_programs() == []
    assert b.fetch_program("ONLY_A", "src") is None


# --------------------------------------------------------------------------- #
# ⑨ CloudLink 端到端：程序库 / 轨迹库 / 状态回显
# --------------------------------------------------------------------------- #


def test_program_library_over_http(link):
    handle = link.upload_program(
        "GRASP v2", PROGRAM_CODE, {"ext": "src", "brand": "FANUC", "robot_name": "UR5"}
    )
    assert handle.startswith("http://")
    assert link.list_programs() == ["GRASP_v2"]  # 服务端键做过去路径语义化
    assert link.fetch_program("GRASP v2", "src") == PROGRAM_CODE
    assert link.fetch_program("GRASP v2", "urp") is None


def test_trace_library_over_http(link, robot):
    trace = _synthetic_trace(robot, "TWIN_HTTP")
    handle = link.publish_trace(trace)
    assert handle.endswith("/traces/TWIN_HTTP.jsonl")

    assert link.list_traces() == ["TWIN_HTTP"]
    meta = link.trace_meta("TWIN_HTTP")
    assert meta["source"] == "plant"
    assert meta["samples"] == 4
    assert meta["max_pos_mm"] == pytest.approx(1.3)
    assert meta["duration_s"] == pytest.approx(1.5)
    assert "非真机实测" in meta["notes"]

    back = link.fetch_trace("TWIN_HTTP")
    assert len(back) == 4
    assert back.max_pos_dev() == pytest.approx(1.3)
    # 取回不改变物理来源（是不是真机测的比存在哪重要），
    # 「经过云端」只追加到 notes，避免链路痕迹被抹掉
    assert back.source == "plant"
    assert "从云端取回" in back.notes
    assert "非真机实测" in back.notes

    report = compare_traces(_synthetic_trace(robot, "SIM"), back)
    assert report.unmatched == 0
    assert len(report.samples) == 4


def test_fetch_missing_trace_raises(link):
    with pytest.raises(TwinError):
        link.fetch_trace("nope")
    assert link.trace_meta("nope") is None


def test_status_feedback_over_http(link, robot):
    assert link.is_actual_feedback is False
    assert link.read_joints() is None
    assert link.get_pose() is None

    link.push_status([10.0, -20.0, 30.0, 0.0, 60.0, 0.0], t=time.time(), source="real")
    assert link.is_actual_feedback is True
    assert link.is_fresh(max_age=60.0)
    q = link.read_joints()
    assert q is not None and q.shape == (6,)
    pose = link.get_pose()
    assert pose is not None

    # 时刻是"上报的时刻"，不是"服务器收到请求的时刻"——机时偏差必须能表达出来
    assert link.status_age(now=time.time()) is not None
    assert link.status()["source"] == "real"
    assert link.status()["joints"][0] == 10.0


def test_status_freshness_is_questionable(link):
    """新鲜度必须能被质疑：远过去的时刻不该被判成"实时"。"""
    link.push_status([0.0] * 6, t=time.time() - 3600.0)
    assert link.is_fresh(max_age=5.0) is False
    assert link.is_fresh(max_age=7200.0) is True


def test_cloudlink_summary_over_http(link):
    link.upload_program("P", PROGRAM_CODE, {"ext": "src"})
    link.publish_trace(_synthetic_trace(link.robot, "T"))
    link.push_status([0.0] * 6)
    summary = link.summary()
    assert summary["namespace"] == "cell1"
    assert summary["protocol"] == "cloud"
    assert summary["programs"] == ["P"]
    assert summary["traces"] == ["T"]
    assert summary["has_status"] is True
    assert summary["status_age_s"] is not None


def test_cloudlink_requires_transport():
    with pytest.raises(TwinError):
        CloudLink(None)


# --------------------------------------------------------------------------- #
# ⑩ 服务端自省与 CLI
# --------------------------------------------------------------------------- #


def test_server_describe_and_request_log(server):
    store = HTTPCloudStore(f"http://{server.address}", "robotworld")
    store.put("k.txt", b"v")
    store.get("k.txt")
    info = server.describe()
    assert info["bucket"] == "robotworld"
    assert info["auth"] == "none"
    assert info["objects"] == 1
    assert info["requests"] >= 2
    assert any(entry[0] == "PUT" for entry in server.log)
    assert any(entry[0] == "GET" for entry in server.log)


def test_server_dialog_log_is_capped(cloud_root):
    srv = CloudStoreServer(cloud_root, dialog=3)
    srv.start()
    try:
        store = HTTPCloudStore(f"http://{srv.address}", "robotworld")
        for i in range(8):
            store.put(f"k{i}.txt", b"v")
        assert len(srv.log) == 3
        assert srv.log[-1][1].endswith("k7.txt")
    finally:
        srv.stop()


def test_server_stop_is_idempotent_and_really_closes(cloud_root):
    srv = CloudStoreServer(cloud_root)
    port = srv.start()
    store = HTTPCloudStore(f"http://127.0.0.1:{port}", "robotworld", timeout=1.0)
    store.put("k.txt", b"v")
    srv.stop()
    srv.stop()  # 幂等
    with pytest.raises(CloudHTTPError):
        store.get("k.txt")


def test_server_start_twice_is_rejected(server):
    with pytest.raises(TwinError):
        server.start()


def test_server_context_manager(cloud_root):
    with CloudStoreServer(cloud_root) as srv:
        store = HTTPCloudStore(f"http://{srv.address}", "robotworld")
        store.put("k.txt", b"v")
        assert store.get("k.txt") == b"v"


def test_server_storage_root_layout(server, cloud_root):
    """落盘布局要可读可备份：<root>/<bucket>/<key>。"""
    store = HTTPCloudStore(f"http://{server.address}", "robotworld")
    store.put("progs/p.src", b"code")
    assert (cloud_root / "robotworld" / "progs" / "p.src").read_bytes() == b"code"
    assert server.read("progs/p.src") == b"code"


def test_server_rejects_unknown_bucket(server):
    store = HTTPCloudStore(f"http://{server.address}", "other-bucket")
    with pytest.raises(CloudHTTPError) as exc:
        store.get("k.txt")
    assert exc.value.status == 404


def test_cli_serves_requests_then_exits(tmp_path):
    """CLI 真实子进程验收：起服务、抓地址、发请求、按 --max-requests 退出。"""
    root = tmp_path / "clicloud"
    proc = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "orolp.cloudhttp",
            "--root",
            str(root),
            "--port",
            "0",
            "--print-address",
            "--max-requests",
            "2",
        ],
        cwd=str(REPO_ROOT),
        env={"PYTHONPATH": str(REPO_ROOT / "python"), "PATH": "/usr/bin:/bin"},
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        # ``python -m orolp.cloudhttp`` 会先带出一行 runpy 的 RuntimeWarning
        # （orolp/__init__ 已经 import 过本模块），ADDRESS 行在其后——扫到为止
        line = ""
        for _ in range(10):
            line = proc.stdout.readline().strip()
            if line.startswith("ADDRESS ") or not line:
                break
        assert line.startswith("ADDRESS "), f"没拿到地址行：{line!r}"
        port = int(line.split()[1].rsplit(":", 1)[1])
        store = HTTPCloudStore(f"http://127.0.0.1:{port}", "robotworld", timeout=5.0)
        store.put("cli.txt", b"from-subprocess")
        assert store.get("cli.txt") == b"from-subprocess"
        assert proc.wait(timeout=15) == 0
    finally:
        if proc.poll() is None:  # pragma: no cover - 失败时别留孤儿进程
            proc.kill()
            proc.wait(timeout=5)


def test_cli_requires_both_s3_credentials(tmp_path):
    rc = cloudhttp_main(["--root", str(tmp_path / "x"), "--s3-access-key", "AK"])
    assert rc == 2


# --------------------------------------------------------------------------- #
# ⑪ 导出与示例脚本
# --------------------------------------------------------------------------- #


def test_exports_are_reachable_from_package_root():
    assert orp.HTTPCloudStore is HTTPCloudStore
    assert orp.SigV4Signer is SigV4Signer
    assert orp.CloudStoreServer is CloudStoreServer
    assert orp.CloudHTTPError is CloudHTTPError
    assert orp.CloudAuthError is CloudAuthError
    assert orp.cloudhttp.HTTPCloudStore is HTTPCloudStore


def test_example_script_runs_end_to_end():
    """示例脚本是文档的一部分——必须真的能跑完（走真 HTTP）。"""
    script = REPO_ROOT / "examples/cloud_http_demo.py"
    result = subprocess.run(
        [sys.executable, str(script)],
        cwd=str(REPO_ROOT),
        env={"PYTHONPATH": str(REPO_ROOT / "python"), "PATH": "/usr/bin:/bin"},
        capture_output=True,
        text=True,
        timeout=180,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "全部通过" in result.stdout
    assert "真机实测" in result.stdout or "未对真实云" in result.stdout


# --------------------------------------------------------------------------- #
# ⑫ 最简脚本化服务端：验收"服务端不该有的行为"
# --------------------------------------------------------------------------- #


class RawServer:
    """按路径返回固定响应的最简服务端（只用于构造坏服务端行为）。

    :class:`CloudStoreServer` 是"正确的参考实现"——它不该返回非 JSON 的列举结果，
    也不该回结构不对的 JSON。要验收客户端在这些情况下的表现，只能自己造一个。
    """

    def __init__(self, routes: dict):
        import http.server as _http

        self.routes = routes
        self._http = _http
        outer = self

        class _Handler(_http.BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *args):  # noqa: D102
                pass

            def _reply(self) -> None:
                status, body = outer.routes.get(
                    self.path.split("?")[0], (404, b"no such route")
                )
                self.send_response(status)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                if body and self.command != "HEAD":
                    self.wfile.write(body)

            do_GET = _reply
            do_PUT = _reply
            do_DELETE = _reply

        self._server = _http.ThreadingHTTPServer(("127.0.0.1", 0), _Handler)
        self._server.daemon_threads = True
        self.port = int(self._server.server_address[1])
        self.base_url = f"http://127.0.0.1:{self.port}"

    def __enter__(self) -> "RawServer":
        thread = threading.Thread(
            target=self._server.serve_forever, kwargs={"poll_interval": 0.02}, daemon=True
        )
        thread.start()
        self._thread = thread
        return self

    def __exit__(self, *exc) -> None:
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2.0)
