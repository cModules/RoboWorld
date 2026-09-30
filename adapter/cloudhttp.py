"""HTTP / S3 兼容云通道：把 :class:`~service.twinsync.CloudTransport` 从"本机目录"接到"真的云上"。

v1.0.3 的云协作已经有 :class:`~service.twinsync.MemoryCloudStore`（进程内）与
:class:`~service.twinsync.LocalCloudStore`（本机目录）两个实现——`CloudLink` 的程序库、
轨迹库、状态回显全都跑通了，但它们都只在**一台机器**上。现场真正想要的是"多工位共用
同一套程序库与轨迹库"，那就需要一个跨进程 / 跨机器的传输。本模块补上第三个实现。

三个层次，各自可以单独用::

    HTTPCloudStore   客户端：把 CloudTransport 的 put/get/keys/delete 映射成 HTTP 请求
    SigV4Signer      可选签名：让同一个客户端直接对接 S3 兼容对象存储（零第三方依赖）
    CloudStoreServer 参考服务端：纯 http.server 的文件式对象存储，用于本机验收与自建

REST 契约（刻意做得很窄，写一个兼容服务端只要几十行）::

    PUT    /<bucket>/<key>                  请求体 = 对象内容          → 200
    GET    /<bucket>/<key>                  响应体 = 对象内容          → 200 / 404
    DELETE /<bucket>/<key>                                             → 200（删除）/ 404（不存在）
    GET    /<bucket>?prefix=<p>             响应体 {"keys": [...]}     → 200（列举）

**两类失败语义刻意分开**（与 :class:`~domain.link.SocketLink` 同一套路数）：

* 传输 / 协议级故障（连不上、超时、响应不是合法 JSON、服务端 5xx）→ 抛
  :class:`CloudHTTPError`，**并把 store 标记为不可达**（``reachable`` 变 ``False``）；
  GUI 据此显示"云端离线"，而不是让上层把它当成"对象不存在"继续往下走。
* 业务级回答：``404`` **不是错误**（"对象不存在"是正常回答）→ ``get`` 返回 ``None``、
  ``delete`` 返回 ``False``；``401/403`` 是配置问题（token 错 / 只读）→ 抛
  :class:`CloudAuthError`，必须响亮。

诚实说明（本模块最需要说清的一点）：:class:`SigV4Signer` 的签名结果已对过 **AWS 官方
文档给出的 4 组示例向量**（规范请求哈希与最终签名两处都能对上，见
``tests/test_cloudhttp.py`` 里的已知答案用例），因此"我们算的签名"与"AWS 说的签名"
是一致的。但**没有对任何真实云端点做过端到端验证**——参考服务端只能证明"我们签的、
我们验的"自洽；接真云之前，请先拿一家 S3 兼容存储做一次冒烟。

用法::

    # 本地验收：先起参考服务端，再用 HTTP 传输跑通整条云链路
    python -m adapter.cloudhttp --root /tmp/rwcloud --port 8080 --print-address
    >>> from adapter.cloudhttp import HTTPCloudStore
    >>> from service.twinsync import CloudLink
    >>> link = CloudLink(HTTPCloudStore("http://127.0.0.1:8080", "robotworld"))
    >>> link.connect(); link.upload_program("PICK", code, {"ext": "urp"})

    # 对接 S3 兼容对象存储（此处只需一个签名器）
    >>> signer = SigV4Signer("AKIA...", "secret...", region="cn-north-1")
    >>> store = HTTPCloudStore("https://s3.example.com", "my-bucket", signer=signer)
"""

from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import socket
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from service.twinsync import CloudTransport, TwinError, normalize_key

__all__ = [
    "CloudHTTPError",
    "CloudAuthError",
    "HTTPCloudStore",
    "SigV4Signer",
    "verify_sigv4",
    "CloudStoreServer",
    "DEFAULT_HTTP_TIMEOUT",
    "CLOUD_HTTP_API_VERSION",
    "SIGV4_ALGORITHM",
    "SIGV4_EMPTY_PAYLOAD_SHA256",
    "uri_encode",
    "canonical_query_string",
    "canonical_request",
    "string_to_sign",
    "signing_key",
]


#: 云 HTTP 通道的协议版本（与 ``core-socket`` 的 ``SOCKET_API_VERSION`` 同理，
#: 用于在自建服务端与客户端之间表达"我按哪一版契约说话"）。
CLOUD_HTTP_API_VERSION = 1

#: 默认请求超时（秒）。云端慢是常态，但离线编程不该被一个卡住的 GET 挂死。
DEFAULT_HTTP_TIMEOUT = 10.0

#: 默认 User-Agent。服务端日志里能一眼看出请求来自哪个版本的客户端。
_USER_AGENT = "core-cloud/1"

#: SigV4 算法标识（出现在 ``Authorization`` 头与待签字符串第一行）。
SIGV4_ALGORITHM = "AWS4-HMAC-SHA256"

#: 空请求体的 SHA256——AWS 文档里反复出现的那个常量。
SIGV4_EMPTY_PAYLOAD_SHA256 = "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"

#: 单次响应的读取上限，防止对端返回无穷流把内存吃光。
_MAX_RESPONSE_BYTES = 64 * 1024 * 1024


# --------------------------------------------------------------------------- #
# 异常
# --------------------------------------------------------------------------- #


class CloudHTTPError(TwinError):
    """传输 / 协议级故障（连不上、超时、响应不合法、服务端 5xx）。

    Attributes:
        status: HTTP 状态码；纯传输层故障（连不上/超时）为 ``None``。
        detail: 服务端响应体的片段（便于把错误带进日志）。
    """

    def __init__(self, message: str, *, status: Optional[int] = None, detail: str = ""):
        text = message if not detail else f"{message}（服务端返回：{detail}）"
        super().__init__(text)
        self.status = status
        self.detail = detail


class CloudAuthError(CloudHTTPError):
    """鉴权 / 授权失败（401 token 缺失或错误、403 无权限或只读）。

    单独一类，因为它是**配置问题**而不是"云端离线"：上层应该提示用户去改配置，
    而不是重试或降级。
    """


# --------------------------------------------------------------------------- #
# SigV4：签名核心（拆成独立函数，便于逐个环节与官方向量对齐）
# --------------------------------------------------------------------------- #


def uri_encode(value: Any, *, encode_slash: bool = True) -> str:
    """SigV4 要求的 URI 编码。

    AWS 文档特意提醒"别用平台自带的 UriEncode"：只保留
    ``A-Za-z0-9-._~``，空格必须编成 ``%20``（不是 ``+``），十六进制用大写，
    斜杠按需编码。这里不依赖 :func:`urllib.parse.quote` 的安全字符集默认值，
    而是显式指定 ``safe``，避免不同 Python 版本默认值差异带来的漂移。
    """
    text = str(value)
    safe = "" if encode_slash else "/"
    return urllib.parse.quote(text, safe=safe + "-._~")


def canonical_query_string(params: Any) -> str:
    """规范查询串：键名排序、逐对编码、``key=value`` 用 ``&`` 连接。

    接受三种输入：``None`` / 空串、``Mapping``、``(key, value)`` 序列，以及
    **已经拼好的查询串**（``"max-keys=2&prefix=J"``）——最后一种是签名时最容易
    踩的坑：从 URL 解析出来的 ``query`` 是字符串，直接按"键值对序列"去解包会
    在第一个字符上报 "not enough values to unpack"。字符串输入会先按 URL 语义
    解成键值对，再统一重排重编码（``keep_blank_values`` 保证 ``?lifecycle``
    这种"只有子资源名"的形式解成 ``lifecycle=``，与 AWS 示例一致）。
    """
    if params is None:
        return ""
    if isinstance(params, str):
        if not params.strip("?&"):
            return ""
        params = urllib.parse.parse_qsl(params.lstrip("?"), keep_blank_values=True)
    if isinstance(params, Mapping):
        pairs: Sequence[Tuple[str, Any]] = list(params.items())
    else:
        pairs = [(k, v) for k, v in params]
    encoded = [(uri_encode(k), uri_encode("" if v is None else v)) for k, v in pairs]
    encoded.sort(key=lambda kv: (kv[0], kv[1]))
    return "&".join(f"{k}={v}" for k, v in encoded)


def _canonical_headers(headers: Mapping[str, Any]) -> Tuple[str, str]:
    """规范头 + 已签头列表。

    头名转小写后按字母序排列；值做 ``Trim``（去首尾空白、连续空白折成一个空格）。
    """
    items = []
    for name, value in headers.items():
        low = str(name).strip().lower()
        text = " ".join(str(value).split())
        items.append((low, text))
    items.sort(key=lambda kv: kv[0])
    canonical = "".join(f"{k}:{v}\n" for k, v in items)
    signed = ";".join(k for k, _ in items)
    return canonical, signed


def canonical_request(
    method: str,
    canonical_uri: str,
    query: Any,
    headers: Mapping[str, Any],
    payload_hash: str,
) -> Tuple[str, str]:
    """构造规范请求，返回 ``(规范请求文本, 已签头列表)``。

    注意：``canonical_uri`` 必须是**已经编码**的路径，且其中的 ``/`` 不编码
    （S3 不对对象键里的斜杠做编码，也不做路径规范化）。
    """
    canonical_headers, signed_headers = _canonical_headers(headers)
    text = "\n".join(
        [
            str(method).upper(),
            canonical_uri,
            canonical_query_string(query),
            canonical_headers,
            signed_headers,
            payload_hash,
        ]
    )
    return text, signed_headers


def credential_scope(amz_date: str, region: str, service: str = "s3") -> str:
    """凭证作用域 ``YYYYMMDD/region/service/aws4_request``。"""
    return f"{str(amz_date)[:8]}/{region}/{service}/aws4_request"


def string_to_sign(
    amz_date: str,
    scope: str,
    canonical_request_text: str,
    *,
    algorithm: str = SIGV4_ALGORITHM,
) -> str:
    """待签字符串：算法、时刻、作用域、规范请求的哈希。"""
    digest = hashlib.sha256(canonical_request_text.encode("utf-8")).hexdigest()
    return "\n".join([algorithm, str(amz_date), scope, digest])


def signing_key(secret_key: str, date_stamp: str, region: str, service: str = "s3") -> bytes:
    """逐级 HMAC 派生签名密钥（不直接用长期密钥签请求，泄一个日期只泄一天）。"""

    def _hmac(key: bytes, msg: str) -> bytes:
        return hmac.new(key, msg.encode("utf-8"), hashlib.sha256).digest()

    k_date = _hmac(("AWS4" + str(secret_key)).encode("utf-8"), date_stamp)
    k_region = _hmac(k_date, region)
    k_service = _hmac(k_region, service)
    return _hmac(k_service, "aws4_request")


def payload_hash(data: Any, *, unsigned: bool = False) -> str:
    """请求体哈希；``unsigned=True`` 时用 ``UNSIGNED-PAYLOAD`` 字面量。"""
    if unsigned:
        return "UNSIGNED-PAYLOAD"
    if data is None:
        raw = b""
    elif isinstance(data, (bytes, bytearray, memoryview)):
        raw = bytes(data)
    else:
        raw = str(data).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _format_amz_date(now: Optional[float] = None) -> str:
    """把时刻格式化成 ``YYYYMMDDTHHMMSSZ``（UTC）。"""
    stamp = time.time() if now is None else float(now)
    return time.strftime("%Y%m%dT%H%M%SZ", time.gmtime(stamp))


class SigV4Signer:
    """AWS Signature Version 4 签名器（header 形式，单一载荷块）。

    只做离线编程真正需要的那部分：给一次请求算出 ``Authorization`` /
    ``X-Amz-Date`` / ``X-Amz-Content-Sha256``。分块上传、预签名 URL、
    临时凭证（STS）没有实现——需要时再加，不在没需求的时候造。

    Args:
        access_key: 访问密钥 ID
        secret_key: 访问密钥（长期密钥本身不参与签名，只用于派生签名密钥）
        region: 区域，例如 ``us-east-1``；中国区常用 ``cn-north-1``
        service: 服务名，对象存储固定 ``s3``
        session_token: 临时凭证的会话令牌（有则加进已签头）
        unsigned_payload: 载荷用 ``UNSIGNED-PAYLOAD`` 跳过哈希核对
        timestamp: 固定时刻（测试用）；``None`` 表示用当前时间

    Note:
        签名覆盖 ``method``、路径、查询串、已签头与载荷哈希——**任何一处变了
        就必须重签**，所以 :class:`HTTPCloudStore` 是"每次请求签一次"，不做
        缓存（缓存 Authorization 会出现"headers 复用导致签名不匹配"的典型故障）。
    """

    algorithm = SIGV4_ALGORITHM

    def __init__(
        self,
        access_key: str,
        secret_key: str,
        region: str = "us-east-1",
        service: str = "s3",
        *,
        session_token: Optional[str] = None,
        unsigned_payload: bool = False,
        timestamp: Optional[float] = None,
    ):
        if not access_key or not secret_key:
            raise TwinError("SigV4Signer 需要 access_key 与 secret_key")
        self.access_key = str(access_key)
        self._secret_key = str(secret_key)
        self.region = str(region)
        self.service = str(service)
        self.session_token = session_token
        self.unsigned_payload = bool(unsigned_payload)
        self.timestamp = timestamp

    # ------------------------------------------------------------------ #

    def sign(
        self,
        method: str,
        url: str,
        body: Any = None,
        headers: Optional[Mapping[str, Any]] = None,
        *,
        now: Optional[float] = None,
    ) -> Dict[str, str]:
        """算出这次请求要附带的签名头。

        Args:
            method: HTTP 方法
            url: 完整 URL（签名用其中的 path + query）
            body: 请求体（bytes 或 str）
            headers: 额外参与签名的头；``Host`` 会自动按 URL 补齐
            now: 固定时刻（测试用）

        Returns:
            需要合并进请求的头部字典。
        """
        parsed = urllib.parse.urlsplit(str(url))
        uri = parsed.path or "/"
        # 已签头至少要含 host；调用方给的头一并参与（x-amz-* 必须参与）
        to_sign: Dict[str, Any] = {"host": parsed.netloc}
        for name, value in (headers or {}).items():
            low = str(name).strip().lower()
            if low in ("authorization", "x-amz-date", "host"):
                continue
            to_sign[low] = value
        stamp = self.timestamp if now is None else now
        amz_date = _format_amz_date(stamp)
        to_sign["x-amz-date"] = amz_date
        digest = payload_hash(body, unsigned=self.unsigned_payload)
        to_sign["x-amz-content-sha256"] = digest
        if self.session_token:
            to_sign["x-amz-security-token"] = self.session_token

        text, signed_headers = canonical_request(method, uri, parsed.query, to_sign, digest)
        scope = credential_scope(amz_date, self.region, self.service)
        sts = string_to_sign(amz_date, scope, text)
        key = signing_key(self._secret_key, amz_date[:8], self.region, self.service)
        signature = hmac.new(key, sts.encode("utf-8"), hashlib.sha256).hexdigest()

        auth = (
            f"{self.algorithm} "
            f"Credential={self.access_key}/{scope},"
            f"SignedHeaders={signed_headers},"
            f"Signature={signature}"
        )
        out = {
            # host 参与了签名，就必须由调用方原样发出去——让传输层"自己猜"
            # 一个 Host 可能与签名不一致（代理/自定义 Host 场景），且服务端
            # 复算时也要求已签头必须真的在场
            "Host": parsed.netloc,
            "Authorization": auth,
            "X-Amz-Date": amz_date,
            "X-Amz-Content-Sha256": digest,
        }
        if self.session_token:
            out["X-Amz-Security-Token"] = str(self.session_token)
        return out

    # 逐个环节暴露，方便把中间值拿去和官方示例向量对照（也便于排查"到底哪一步错了"）
    def canonical_request_for(
        self,
        method: str,
        url: str,
        body: Any = None,
        headers: Optional[Mapping[str, Any]] = None,
        *,
        now: Optional[float] = None,
    ) -> Tuple[str, str]:
        """返回 ``(规范请求文本, 已签头列表)``。"""
        parsed = urllib.parse.urlsplit(str(url))
        to_sign: Dict[str, Any] = {"host": parsed.netloc}
        for name, value in (headers or {}).items():
            low = str(name).strip().lower()
            if low in ("authorization", "x-amz-date", "host"):
                continue
            to_sign[low] = value
        stamp = self.timestamp if now is None else now
        amz_date = _format_amz_date(stamp)
        to_sign["x-amz-date"] = amz_date
        digest = payload_hash(body, unsigned=self.unsigned_payload)
        to_sign["x-amz-content-sha256"] = digest
        if self.session_token:
            to_sign["x-amz-security-token"] = self.session_token
        return canonical_request(method, parsed.path or "/", parsed.query, to_sign, digest)

    def __repr__(self) -> str:  # pragma: no cover - 展示用
        return (
            f"<SigV4Signer {self.access_key[:4]}… region={self.region} "
            f"service={self.service} token={'有' if self.session_token else '无'}>"
        )


def verify_sigv4(
    method: str,
    path: str,
    query: str,
    headers: Mapping[str, Any],
    body: Any,
    *,
    access_key: str,
    secret_key: str,
    region: str = "us-east-1",
    service: str = "s3",
) -> Tuple[bool, str]:
    """服务端复算签名（真实 S3 就是这么校验的）。

    参考服务端在 ``s3`` 模式下用它来验收客户端签名——**这是本模块唯一能证明
    "我们签的确实符合 SigV4 规则"的运行时手段**（之外的证据就是那份官方向量
    测试）。返回 ``(是否通过, 原因)``，原因用于日志与 403 响应体。
    """
    auth = ""
    for name, value in headers.items():
        if str(name).strip().lower() == "authorization":
            auth = str(value)
            break
    if not auth:
        return False, "缺少 Authorization 头"
    if not auth.startswith(SIGV4_ALGORITHM + " "):
        return False, f"算法不是 {SIGV4_ALGORITHM}"

    fields: Dict[str, str] = {}
    for chunk in auth[len(SIGV4_ALGORITHM) + 1 :].split(","):
        if "=" in chunk:
            k, v = chunk.split("=", 1)
            fields[k.strip()] = v.strip()
    for required in ("Credential", "SignedHeaders", "Signature"):
        if required not in fields:
            return False, f"Authorization 缺少 {required}"
    cred = fields["Credential"]
    if "/" not in cred:
        return False, "Credential 格式不对"
    got_key, scope = cred.split("/", 1)
    if got_key != access_key:
        return False, "access_key 不匹配"

    # 只按客户端**声明**的已签头复算：签名覆盖范围由 SignedHeaders 决定，
    # 服务端必须照单全收（这也是"签了什么就只保证什么"的诚实含义）
    declared = [h.strip().lower() for h in fields["SignedHeaders"].split(";") if h.strip()]
    lower = {str(k).strip().lower(): v for k, v in headers.items()}
    to_sign: Dict[str, Any] = {}
    for name in declared:
        if name not in lower:
            return False, f"已签头 {name} 实际缺失"
        to_sign[name] = lower[name]
    digest = payload_hash(body)
    if "x-amz-content-sha256" in lower and str(lower["x-amz-content-sha256"]) != digest:
        return False, "载荷哈希与 x-amz-content-sha256 不一致"
    to_sign.setdefault("x-amz-content-sha256", digest)
    if "x-amz-date" not in to_sign:
        return False, "缺少 x-amz-date"

    text, signed_headers = canonical_request(method, path, query, to_sign, digest)
    if signed_headers != ";".join(sorted(declared)):
        return False, "SignedHeaders 未按字母序排列"
    amz_date = str(to_sign["x-amz-date"])
    expected_scope = credential_scope(amz_date, region, service)
    if scope != expected_scope:
        return False, f"凭证作用域不匹配（应为 {expected_scope}）"
    sts = string_to_sign(amz_date, scope, text)
    key = signing_key(secret_key, amz_date[:8], region, service)
    expect = hmac.new(key, sts.encode("utf-8"), hashlib.sha256).hexdigest()
    if not hmac.compare_digest(expect, fields["Signature"]):
        return False, "签名不匹配"
    return True, "ok"


# --------------------------------------------------------------------------- #
# 客户端
# --------------------------------------------------------------------------- #


class HTTPCloudStore(CloudTransport):
    """:class:`~service.twinsync.CloudTransport` 的 HTTP 实现。

    把四个方法映射成四个请求（见模块文档的 REST 契约）。既然键就是路径，
    键的规范化与穿越防护仍然由 :func:`~service.twinsync.normalize_key` 在**发请求之前**
    做掉——不合法键根本不上网。

    Args:
        base_url: 服务根，例如 ``http://127.0.0.1:8080`` 或 ``https://s3.example.com``
        bucket: 桶名（会成为路径的第一段）
        token: 可选 Bearer 令牌（自建服务的简易鉴权）
        timeout: 单请求超时（秒）
        signer: 可选的 :class:`SigV4Signer`（对接 S3 兼容存储时给）
        retries: 传输层失败后的重试次数（只重试幂等的 GET/PUT/DELETE 与 5xx）
        opener: 自定义 ``urlopen``（测试/代理注入用）
    """

    api_version = CLOUD_HTTP_API_VERSION

    def __init__(
        self,
        base_url: str,
        bucket: str = "robotworld",
        *,
        token: Optional[str] = None,
        timeout: float = DEFAULT_HTTP_TIMEOUT,
        signer: Optional[SigV4Signer] = None,
        retries: int = 0,
        opener: Any = None,
    ):
        text = str(base_url or "").strip()
        if not text:
            raise TwinError("HTTPCloudStore 需要 base_url")
        parsed = urllib.parse.urlsplit(text)
        if parsed.scheme not in ("http", "https"):
            raise TwinError(f"base_url 必须以 http:// 或 https:// 开头：{base_url!r}")
        if not parsed.netloc:
            raise TwinError(f"base_url 缺少主机名：{base_url!r}")
        self.base_url = text.rstrip("/")
        self.bucket = normalize_key(bucket)
        self.token = token
        self.timeout = float(timeout)
        self.signer = signer
        self.retries = max(0, int(retries))
        self._opener = opener or urllib.request.urlopen
        self._reachable = True
        self._last_error: Optional[CloudHTTPError] = None
        self._requests = 0
        self._lock = threading.Lock()

    # ------------------------------ 工具 ------------------------------ #

    @classmethod
    def from_url(cls, url: str, **kwargs: Any) -> "HTTPCloudStore":
        """从 ``http://host:port/bucket`` 形式的 URL 构造。

        配置里写一个 URL 比写"根地址 + 桶名"两处更不容易配错，所以给这个入口：
        路径只有一段时当桶名，多余部分报错而不是悄悄丢掉。
        """
        parsed = urllib.parse.urlsplit(str(url))
        parts = [p for p in parsed.path.split("/") if p]
        if len(parts) > 1:
            raise TwinError(f"URL 里只能带一个桶名，收到多段路径：{parsed.path!r}")
        bucket = kwargs.pop("bucket", parts[0] if parts else "robotworld")
        base = f"{parsed.scheme}://{parsed.netloc}"
        return cls(base, bucket, **kwargs)

    @property
    def reachable(self) -> bool:
        """上一次请求后是否认为云端可达。

        "连不上"与"对象不存在"是两件事——这个标记让 GUI 能显示"云端离线"，
        而不是把一次断网渲染成"云端是空的"。
        """
        return self._reachable

    @property
    def last_error(self) -> Optional[CloudHTTPError]:
        """最近一次传输级失败（成功一次后清空）。"""
        return self._last_error

    @property
    def requests(self) -> int:
        """已发出的 HTTP 请求数（含重试）。"""
        return self._requests

    def _key_path(self, key: str) -> str:
        """把云端键编成 URL 路径（逐段编码，斜杠保留为层级分隔）。"""
        k = normalize_key(key)
        return "/".join(uri_encode(part) for part in k.split("/"))

    def _url(self, key: Optional[str] = None, query: Any = None) -> str:
        url = f"{self.base_url}/{uri_encode(self.bucket)}"
        if key is not None:
            url += "/" + self._key_path(key)
        if query:
            qs = canonical_query_string(query)
            if qs:
                url += "?" + qs
        return url

    def _headers(self, method: str, url: str, data: Optional[bytes]) -> Dict[str, str]:
        headers: Dict[str, str] = {"User-Agent": _USER_AGENT}
        if self.token:
            headers["Authorization"] = f"Bearer {self.token}"
        if self.signer is not None:
            # 每次请求重新签：签名覆盖 path/query/载荷，复用会不匹配
            extra = {"user-agent": headers["User-Agent"]}
            if data is not None:
                extra["content-length"] = str(len(data))
            headers.update(self.signer.sign(method, url, data, extra))
        return headers

    def _send(
        self, method: str, url: str, data: Optional[bytes]
    ) -> Tuple[int, bytes, Dict[str, str]]:
        """真正发一次请求；把"连不上/超时/协议错误"翻译成 :class:`CloudHTTPError`。

        响应头一并带回：404 到底是"对象不存在"还是"桶不存在"，客户端要靠
        服务端的标记头来区分（见 :meth:`get`）。
        """
        req = urllib.request.Request(url, data=data, method=method)
        for name, value in self._headers(method, url, data).items():
            req.add_header(name, value)
        with self._lock:
            self._requests += 1
        try:
            with self._opener(req, timeout=self.timeout) as resp:
                body = resp.read(_MAX_RESPONSE_BYTES)
                heads = {str(k).lower(): str(v) for k, v in resp.headers.items()}
                return int(getattr(resp, "status", resp.getcode())), body, heads
        except urllib.error.HTTPError as exc:  # 4xx / 5xx 也是"有回答"
            try:
                body = exc.read(_MAX_RESPONSE_BYTES)
            except Exception:  # pragma: no cover - 读不到就算了
                body = b""
            heads = {str(k).lower(): str(v) for k, v in exc.headers.items()}
            return int(exc.code), body, heads
        except urllib.error.URLError as exc:
            raise self._transport_error(url, exc.reason) from exc
        except socket.timeout as exc:  # pragma: no cover - 版本差异下的兜底
            raise self._transport_error(url, f"超时（>{self.timeout}s）") from exc
        except OSError as exc:
            raise self._transport_error(url, exc) from exc

    def _transport_error(self, url: str, reason: Any) -> CloudHTTPError:
        err = CloudHTTPError(
            f"云端不可达：{url}（{reason}）；若在本地验收，可先起参考服务端："
            f"python -m adapter.cloudhttp --root <目录> --port <端口>"
        )
        self._reachable = False
        self._last_error = err
        return err

    def _brief(self, body: bytes, limit: int = 200) -> str:
        try:
            text = body.decode("utf-8", "replace")
        except Exception:  # pragma: no cover - decode 带 replace 不会抛
            text = repr(body)
        text = " ".join(text.split())
        return text[:limit] + ("…" if len(text) > limit else "")

    def _request(
        self,
        method: str,
        key: Optional[str] = None,
        data: Optional[bytes] = None,
        query: Any = None,
        *,
        allow_missing: bool = False,
        allow_auth: bool = False,
    ) -> Tuple[int, bytes, Dict[str, str]]:
        """带重试的请求；统一把 5xx 当传输故障、4xx 按语义分开处理。"""
        url = self._url(key, query)
        attempt = 0
        while True:
            status, body, heads = self._send(method, url, data)
            if status >= 500:
                err = CloudHTTPError(
                    f"云服务端错误 {status}：{method} {url}",
                    status=status,
                    detail=self._brief(body),
                )
                self._reachable = False
                self._last_error = err
            elif status in (401, 403) and not allow_auth:
                raise CloudAuthError(
                    f"云端拒绝访问（{status}）：{method} {url}——请检查 token / 密钥 / 权限配置",
                    status=status,
                    detail=self._brief(body),
                )
            elif status == 404 and allow_missing:
                self._reachable = True
                self._last_error = None
                return status, body, heads
            elif 400 <= status < 500:
                raise CloudHTTPError(
                    f"云端请求被拒（{status}）：{method} {url}",
                    status=status,
                    detail=self._brief(body),
                )
            else:
                self._reachable = True
                self._last_error = None
                return status, body, heads
            attempt += 1
            if attempt > self.retries:
                raise err
            time.sleep(min(0.2, 0.05 * (2 ** (attempt - 1))))

    # ------------------------------ CloudTransport 契约 ------------------------------ #

    def put(self, key: str, data: bytes) -> str:
        """写入对象，返回该对象的 HTTP 地址（比 ``cloud://`` 句柄更有用：能直接点开）。"""
        url = self._url(key)
        status, body, _ = self._request("PUT", key, bytes(data))
        if status not in (200, 201, 204):
            raise CloudHTTPError(
                f"上传失败（{status}）：{url}", status=status, detail=self._brief(body)
            )
        return url

    def get(self, key: str) -> Optional[bytes]:
        """读取对象；不存在返回 ``None``（404 是正常回答，不是错误）。

        但**桶不存在是配置错误**，不能伪装成"对象不存在"——那会让断错桶的人
        以为云端是空的。服务端用标记头区分，真实 S3 用响应体里的
        ``NoSuchBucket`` 表达，两种都识别。空对象与不存在也分得清：
        前者 ``b""``，后者 ``None``。
        """
        status, body, heads = self._request("GET", key, allow_missing=True)
        if status == 404:
            if heads.get("x-core-error") == "no-such-bucket" or b"NoSuchBucket" in body:
                raise CloudHTTPError(
                    f"云端没有桶 {self.bucket!r}（配置错误，不是对象缺失）", status=404
                )
            return None
        return body

    def keys(self, prefix: str = "") -> List[str]:
        """列出前缀下的全部键（升序）。"""
        p = prefix.strip("/")
        query = {"prefix": p} if p else {"list": "1"}
        status, body, _ = self._request("GET", None, query=query)
        try:
            payload = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise CloudHTTPError(f"列举响应不是合法 JSON：{self._brief(body)}") from exc
        if not isinstance(payload, dict) or not isinstance(payload.get("keys"), list):
            raise CloudHTTPError(f'列举响应结构不对（应为 {{"keys": [...]}}）：{self._brief(body)}')
        out: List[str] = []
        for item in payload["keys"]:
            if not isinstance(item, str):
                raise CloudHTTPError(f"列举响应里出现了非字符串键：{item!r}")
            out.append(item)
        return sorted(out)

    def delete(self, key: str) -> bool:
        """删除对象；返回是否真的删掉了。"""
        status, _, _ = self._request("DELETE", key, allow_missing=True)
        return status != 404

    # ------------------------------ 自省 ------------------------------ #

    def describe(self) -> Dict[str, Any]:
        """现状摘要（给 GUI / CLI 显示）。"""
        return {
            "protocol": "http",
            "api_version": self.api_version,
            "base_url": self.base_url,
            "bucket": self.bucket,
            "auth": "bearer" if self.token else ("sigv4" if self.signer else "none"),
            "signer": repr(self.signer) if self.signer else None,
            "timeout_s": self.timeout,
            "retries": self.retries,
            "reachable": self._reachable,
            "requests": self._requests,
            "last_error": str(self._last_error) if self._last_error else None,
        }

    def __repr__(self) -> str:  # pragma: no cover - 展示用
        return (
            f"<HTTPCloudStore {self.base_url}/{self.bucket} "
            f"auth={'token' if self.token else ('sigv4' if self.signer else 'none')}>"
        )


# --------------------------------------------------------------------------- #
# 参考服务端：本机验收 / 自建对象存储
# --------------------------------------------------------------------------- #


def _parse_fail_ops(spec: Any) -> Dict[str, int]:
    """把 ``fail_ops`` 规范化成 ``{方法: 剩余失败次数}``（负数=永远失败）。"""
    out: Dict[str, int] = {}
    if spec is None:
        return out
    if isinstance(spec, Mapping):
        for k, v in spec.items():
            out[str(k).upper()] = int(v)
    else:
        for item in spec:
            out[str(item).upper()] = -1
    return out


class _Handled(Exception):
    """内部哨兵：错误响应已发出，处理函数必须立刻返回（防止二次响应）。"""


class _StoreHandler(BaseHTTPRequestHandler):
    """参考对象存储的请求处理（协议 HTTP/1.1，始终带 Content-Length）。"""

    protocol_version = "HTTP/1.1"
    server_version = "core-cloudstore/1"
    sys_version = ""

    # ------------------------------ 基础 ------------------------------ #

    def log_message(self, fmt: str, *args: Any) -> None:  # noqa: A003
        """静音默认的 stderr 日志（测试输出要干净；要日志请用 server.log）。"""

    @property
    def store(self) -> "CloudStoreServer":
        return self.server.store  # type: ignore[attr-defined]

    def _respond(
        self,
        status: int,
        body: bytes = b"",
        content_type: str = "text/plain",
        extra_headers: Optional[Dict[str, str]] = None,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        for name, value in (extra_headers or {}).items():
            self.send_header(name, value)
        self.end_headers()
        if body:
            self.wfile.write(body)

    def _json(self, status: int, payload: Any) -> None:
        self._respond(
            status,
            json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            "application/json",
        )

    # ------------------------------ 路由 ------------------------------ #

    def _split(self) -> Tuple[str, str, Dict[str, List[str]]]:
        parsed = urllib.parse.urlsplit(self.path)
        parts = [urllib.parse.unquote(p) for p in parsed.path.split("/") if p]
        query = urllib.parse.parse_qs(parsed.query, keep_blank_values=True)
        return (parts[0] if parts else ""), "/".join(parts[1:]), query

    def _guard(self, method: str, body: bytes) -> Optional[bytes]:
        """公共前置检查：故障注入 → 延迟 → 鉴权 → 只读。返回要直接发回去的响应体。"""
        store = self.store
        store._note(method, self.path)
        if store.latency:
            time.sleep(store.latency)
        plan = store.fail_ops.get(method)
        if plan is not None:
            if plan < 0:
                self._respond(503, b"injected failure")
                return b"stop"
            if plan > 0:
                store.fail_ops[method] = plan - 1
                self._respond(503, b"injected failure")
                return b"stop"
            store.fail_ops[method] = None  # type: ignore[assignment]
        if store.s3:
            ok, reason = verify_sigv4(
                method,
                urllib.parse.urlsplit(self.path).path,
                urllib.parse.urlsplit(self.path).query,
                {k: v for k, v in self.headers.items()},
                body,
                access_key=store.s3["access_key"],
                secret_key=store.s3["secret_key"],
                region=store.s3.get("region", "us-east-1"),
                service=store.s3.get("service", "s3"),
            )
            if not ok:
                self._respond(403, f"signature rejected: {reason}".encode("utf-8"))
                return b"stop"
        elif store.token is not None:
            got = str(self.headers.get("Authorization", ""))
            if got != f"Bearer {store.token}":
                self._respond(401, b"bad or missing token")
                return b"stop"
        if store.read_only and method in ("PUT", "DELETE"):
            self._respond(403, b"read-only store")
            return b"stop"
        return None

    def _bucket_and_key(self) -> Optional[Tuple[str, str]]:
        bucket, raw_key, _ = self._split()
        if bucket != self.store.bucket:
            self._respond(
                404,
                f"no such bucket: {bucket}".encode("utf-8"),
                # 404 同时意味着"对象不存在"与"桶不存在"——后者是配置错误，
                # 必须让客户端能区分出来（真实 S3 用 XML body 的 NoSuchBucket
                # 表达同一件事，客户端两种标记都认）
                extra_headers={"X-Core-Error": "no-such-bucket"},
            )
            raise _Handled()  # 已响应，请求到此为止——不能继续往下走再发第二个响应
        if not raw_key:
            return None  # 列举请求
        try:
            return bucket, normalize_key(raw_key)
        except TwinError as exc:
            self._respond(400, str(exc).encode("utf-8"))
            raise _Handled() from exc

    # ------------------------------ 方法 ------------------------------ #

    def do_GET(self) -> None:  # noqa: N802
        if self._guard("GET", b"") is not None:
            return
        try:
            target = self._bucket_and_key()
        except _Handled:
            return
        if target is None:
            if self.store.garbage_list:
                self._respond(200, b"<html>not json</html>", "text/html")
                return
            _, _, query = self._split()
            prefix = (query.get("prefix") or [""])[0]
            self._json(200, {"keys": self.store.list_keys(prefix)})
            return
        _, key = target
        data = self.store.read(key)
        if data is None:
            self._respond(404, b"no such key")
            return
        self._respond(200, data, "application/octet-stream")

    def do_PUT(self) -> None:  # noqa: N802
        length = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(length) if length else b""
        if self._guard("PUT", body) is not None:
            return
        try:
            target = self._bucket_and_key()
        except _Handled:
            return
        if target is None:
            return
        if self.store.max_put_bytes is not None and len(body) > self.store.max_put_bytes:
            self._respond(
                413,
                f"object too large: {len(body)} > {self.store.max_put_bytes}".encode("utf-8"),
            )
            return
        _, key = target
        self.store.write(key, body)
        self._respond(200, b"")

    def do_DELETE(self) -> None:  # noqa: N802
        if self._guard("DELETE", b"") is not None:
            return
        try:
            target = self._bucket_and_key()
        except _Handled:
            return
        if target is None:
            return
        _, key = target
        if self.store.remove(key):
            self._respond(200, b"")
        else:
            self._respond(404, b"no such key")


class _StoreServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, addr: Tuple[str, int], store: "CloudStoreServer"):
        self.store = store  # type: ignore[assignment]
        super().__init__(addr, _StoreHandler)


class CloudStoreServer:
    """参考对象存储服务端（纯标准库，文件式）。

    存在的意义有两个：①让 :class:`HTTPCloudStore` **不需要任何云账号**就能被
    验收（跑真 HTTP、真 socket、真并发）；②给自建对象存储的人一份可读的参考
    实现与 REST 契约。

    故障注入（验收"客户端遇到坏服务端会怎样"）：
    ``fail_ops`` 让指定方法返回 503、``garbage_list`` 让列举返回非 JSON、
    ``max_put_bytes`` 让大对象 413、``read_only`` 让写操作 403、``latency``
    压出超时。

    Args:
        root: 存储根目录（桶会作为第一层子目录）
        host / port: 监听地址；``port=0`` 由系统分配
        bucket: 桶名
        token: 简易 Bearer 鉴权；``None`` 表示不校验
        s3: 打开 SigV4 校验，形如 ``{"access_key":…, "secret_key":…, "region":…}``
        max_put_bytes: 单对象上限；``None`` 不限
        read_only: 只读模式（PUT/DELETE 一律 403）
        fail_ops: ``{方法: 失败次数}`` 或方法名序列（序列＝永远失败）
        latency: 每个请求的固定延迟（秒），用来压超时路径
        garbage_list: 列举接口返回非 JSON（验收客户端的解析防御）
        dialog: ``log`` 保留的最近请求条数（0 表示不记）
    """

    def __init__(
        self,
        root: str | Path,
        host: str = "127.0.0.1",
        port: int = 0,
        *,
        bucket: str = "robotworld",
        token: Optional[str] = None,
        s3: Optional[Mapping[str, Any]] = None,
        max_put_bytes: Optional[int] = None,
        read_only: bool = False,
        fail_ops: Any = (),
        latency: float = 0.0,
        garbage_list: bool = False,
        dialog: int = 32,
    ):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.host = host
        self.port = int(port)
        self.bucket = normalize_key(bucket)
        self.token = token
        self.s3 = dict(s3) if s3 else None
        self.max_put_bytes = max_put_bytes
        self.read_only = bool(read_only)
        self.fail_ops = _parse_fail_ops(fail_ops)
        self.latency = float(latency)
        self.garbage_list = bool(garbage_list)
        self._dialog_limit = int(dialog)
        self.log: List[Tuple[str, str, str]] = []
        self.requests = 0
        self.address = f"{host}:{self.port}"
        self._server: Optional[_StoreServer] = None
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    # ------------------------------ 存储 ------------------------------ #

    def _path(self, key: str) -> Path:
        return self.root / self.bucket / normalize_key(key)

    def read(self, key: str) -> Optional[bytes]:
        """直接读文件（服务端内部用，也方便测试断言落盘内容）。"""
        p = self._path(key)
        return p.read_bytes() if p.is_file() else None

    def write(self, key: str, data: bytes) -> None:
        p = self._path(key)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(bytes(data))

    def remove(self, key: str) -> bool:
        p = self._path(key)
        if p.is_file():
            p.unlink()
            return True
        return False

    def list_keys(self, prefix: str = "") -> List[str]:
        base = self.root / self.bucket
        if not base.exists():
            return []
        p = prefix.strip("/")
        out: List[str] = []
        for f in base.rglob("*"):
            if f.is_file():
                rel = f.relative_to(base).as_posix()
                if rel.startswith(p):
                    out.append(rel)
        return sorted(out)

    def _note(self, method: str, path: str) -> None:
        with self._lock:
            self.requests += 1
            if self._dialog_limit:
                self.log.append((method, path, time.strftime("%H:%M:%S")))
                if len(self.log) > self._dialog_limit:
                    del self.log[: len(self.log) - self._dialog_limit]

    # ------------------------------ 生命周期 ------------------------------ #

    def start(self) -> int:
        """启动服务，返回实际监听端口（``port=0`` 时由系统分配）。"""
        if self._server is not None:
            raise TwinError("CloudStoreServer 已经启动")
        server = _StoreServer((self.host, self.port), self)
        self.port = int(server.server_address[1])
        self.address = f"{self.host}:{self.port}"
        self._server = server
        thread = threading.Thread(
            target=server.serve_forever,
            kwargs={"poll_interval": 0.02},
            name="core-cloudstore",
            daemon=True,
        )
        thread.start()
        self._thread = thread
        return self.port

    def stop(self) -> None:
        """停止服务（幂等）。HTTP/1.1 的连接是短的，这里只需关监听口并等线程退出。"""
        server, thread = self._server, self._thread
        self._server = None
        self._thread = None
        if server is not None:
            server.shutdown()
            server.server_close()
        if thread is not None:
            thread.join(timeout=2.0)

    def describe(self) -> Dict[str, Any]:
        """现状摘要。"""
        return {
            "address": self.address,
            "bucket": self.bucket,
            "root": str(self.root),
            "auth": "sigv4" if self.s3 else ("bearer" if self.token else "none"),
            "read_only": self.read_only,
            "max_put_bytes": self.max_put_bytes,
            "fail_ops": {k: v for k, v in self.fail_ops.items() if v is not None},
            "requests": self.requests,
            "objects": len(self.list_keys()),
        }

    def __enter__(self) -> "CloudStoreServer":
        self.start()
        return self

    def __exit__(self, *exc: Any) -> None:
        self.stop()

    def __repr__(self) -> str:  # pragma: no cover - 展示用
        state = "运行中" if self._server is not None else "已停止"
        return f"<CloudStoreServer {self.address} bucket={self.bucket} {state}>"


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #


def main(argv: Optional[Sequence[str]] = None) -> int:
    """起一个参考对象存储服务端（Ctrl-C 退出）。

    打印 ``ADDRESS <host>:<port>`` 一行，方便脚本/测试抓地址。
    """
    ap = argparse.ArgumentParser(
        prog="python -m adapter.cloudhttp",
        description="RobotWorld 参考对象存储服务端（本机验收 / 自建自用）",
    )
    ap.add_argument("--root", required=True, help="存储根目录")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080, help="0 表示由系统分配")
    ap.add_argument("--bucket", default="robotworld")
    ap.add_argument("--token", default=None, help="启用 Bearer 鉴权并指定令牌")
    ap.add_argument("--s3-access-key", default=None, help="启用 SigV4 校验：访问密钥 ID")
    ap.add_argument("--s3-secret-key", default=None, help="启用 SigV4 校验：访问密钥")
    ap.add_argument("--s3-region", default="us-east-1")
    ap.add_argument("--max-put-bytes", type=int, default=None)
    ap.add_argument("--read-only", action="store_true")
    ap.add_argument(
        "--fail-op",
        action="append",
        default=[],
        help="让指定方法永远返回 503（可重复，例如 --fail-op PUT）",
    )
    ap.add_argument("--latency", type=float, default=0.0)
    ap.add_argument("--print-address", action="store_true", help="启动后打印 ADDRESS 行")
    ap.add_argument(
        "--max-requests",
        type=int,
        default=0,
        help="处理完这么多请求后退出（0＝一直服务），供脚本化验收用",
    )
    args = ap.parse_args(list(argv) if argv is not None else None)

    s3 = None
    if args.s3_access_key or args.s3_secret_key:
        if not (args.s3_access_key and args.s3_secret_key):
            print("启用 SigV4 需要同时给 --s3-access-key 与 --s3-secret-key", file=sys.stderr)
            return 2
        s3 = {
            "access_key": args.s3_access_key,
            "secret_key": args.s3_secret_key,
            "region": args.s3_region,
        }
    server = CloudStoreServer(
        args.root,
        args.host,
        args.port,
        bucket=args.bucket,
        token=args.token,
        s3=s3,
        max_put_bytes=args.max_put_bytes,
        read_only=args.read_only,
        fail_ops=args.fail_op,
        latency=args.latency,
    )
    port = server.start()
    print(f"ADDRESS {args.host}:{port}", flush=True)
    if args.print_address:
        print(
            f"参考对象存储已启动：http://{args.host}:{port}/{server.bucket} "
            f"（根目录 {server.root}）",
            flush=True,
        )
    try:
        while True:
            time.sleep(0.05)
            if args.max_requests and server.requests >= args.max_requests:
                break
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C，停止服务", flush=True)
    finally:
        server.stop()
    return 0


if __name__ == "__main__":  # pragma: no cover - 手动运行
    raise SystemExit(main())
