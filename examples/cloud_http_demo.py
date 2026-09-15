#!/usr/bin/env python3
"""RobotWorld 示例：HTTP / S3 云通道全流程（v1.0.6）。

v1.0.3 的云协作给了两种传输：`MemoryCloudStore`（进程内）与 `LocalCloudStore`
（本机目录）。两者都能把"程序库 / 轨迹库 / 状态回显"跑通，但都只在**一台机器**上
——多工位共用一套程序库这件事，它们做不到。本示例走的是**真 HTTP**：

    起参考对象存储服务端 → 签名核对（AWS 官方向量）
      → 三种传输的契约一致性 → 两个工位共用程序库与轨迹库
      → 失败语义演练（连不上 / 5xx / 404 / 401 / 413 / 超时）
      → 服务端请求日志与落盘目录

整条链路真跑一遍，全程只用标准库。

关于"云"的诚实说明
------------------

**本示例没有云账号，也没有连过任何真实云端点**。服务端是仓库自带、跑在本机回环上的
参考实现 :class:`~orolp.cloudhttp.CloudStoreServer`，所以"能通"这件事只说明
**客户端与服务端彼此吻合**，**不代表对真实 S3 / 对象存储做过端到端验证**。

签名部分比"自洽"强一档：`SigV4Signer` 的结果对着 **AWS 官方文档给出的 4 组示例向量**
逐字符核对（规范请求哈希 + 已签头 + 最终签名），见下面的第 2 节。也就是说
"我们算的签名 == AWS 说的签名"这一点是**有外部证据**的；剩下没有外部证据的是
"真实端点会不会接受它、以及对象存储端的权限/加密/多段上传等行为"。

接真云时要做的只有一件事：把 base_url / bucket / 凭证换成真的，其余（`CloudLink`、
孪生层、调试会话、后处理）一行都不用改。用 HTTP + Bearer 就走自建服务；用 S3 兼容
存储就多给一个 :class:`~orolp.cloudhttp.SigV4Signer`。

运行::

    python examples/cloud_http_demo.py
    # 或未安装 orolp 时：PYTHONPATH=python python examples/cloud_http_demo.py
"""

from __future__ import annotations

import calendar
import hashlib
import json
import socket
import sys
import tempfile
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
for p in (str(REPO_ROOT / "python"), str(REPO_ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)

from orolp import Pose, ProgramBuilder, Robot  # noqa: E402
from orolp.cloudhttp import (  # noqa: E402
    CloudAuthError,
    CloudHTTPError,
    CloudStoreServer,
    HTTPCloudStore,
    SigV4Signer,
    payload_hash,
)
from orolp.planner import TrajectoryPlanner  # noqa: E402
from orolp.post import get_post_processor  # noqa: E402
from orolp.twinsync import (  # noqa: E402
    CloudLink,
    LocalCloudStore,
    MemoryCloudStore,
    TwinError,
    capture_plant_trace,
    compare_traces,
)

URDF = REPO_ROOT / "models/ur/ur5e/ur5e.urdf"
Q0 = [0.0, -30.0, 30.0, 0.0, 60.0, 0.0]

# 合成"真机"模型（零位偏置 / 传动比误差 / 编码器噪声 / 一阶伺服滞后）
PLANT_KWARGS = dict(joint_bias_deg=0.25, scale_error=8e-4, noise_deg=0.01, lag_tau=0.02)

# AWS 官方文档示例向量（Amazon S3「簽章計算示例」一节）
AWS_ACCESS_KEY = "AKIAIOSFODNN7EXAMPLE"
AWS_SECRET_KEY = "wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY"
AWS_TS = calendar.timegm(time.strptime("20130524T000000Z", "%Y%m%dT%H%M%SZ"))
AWS_VECTORS = [
    (
        "取对象前 10 字节（无载荷）",
        "GET",
        "https://examplebucket.s3.amazonaws.com/test.txt",
        {"Range": "bytes=0-9"},
        None,
        "7344ae5b7ee6c3e7e6b0fe0640412a37625d1fbfff95c48bbb2dc43964946972",
        "f0e8bdb87c964420e857bd35b5d6ed310bd44f0170aba48dd91039c6036bdb41",
    ),
    (
        "上传对象（带载荷）",
        "PUT",
        "https://examplebucket.s3.amazonaws.com/test%24file.text",
        {"Date": "Fri, 24 May 2013 00:00:00 GMT", "x-amz-storage-class": "REDUCED_REDUNDANCY"},
        b"Welcome to Amazon S3.",
        "9e0e90d9c76de8fa5b200d8c849cd5b8dc7a3be3951ddb7f6a76b4158342019d",
        "98ad721746da40c64f1a55b78f14c238d841ea1380cd77a1b5971af0ece108bd",
    ),
    (
        "取桶生命周期（子资源无值）",
        "GET",
        "https://examplebucket.s3.amazonaws.com/?lifecycle",
        {},
        None,
        "9766c798316ff2757b517bc739a67f6213b4ab36dd5da2f94eaebf79c77395ca",
        "fea454ca298b7da1c68078a5d1bdbfbbe0d65c699e0f91ac7a200a0136783543",
    ),
    (
        "列举对象（两个查询参数）",
        "GET",
        "https://examplebucket.s3.amazonaws.com/?max-keys=2&prefix=J",
        {},
        None,
        "df57d21db20da04d7fa30298dd4488ba3a2b47ca3a489c74750e0f1e7df1b9b7",
        "34b48302e7b5fa45bde8084f4b7868a86f0a534bc59db6670ed5711ef69dc6f7",
    ),
]

FAILURES: list = []


def _hr(title: str) -> None:
    print()
    print("=" * 72)
    print(title)
    print("=" * 72)


def _ok(label: str, passed: bool, extra: str = "") -> None:
    mark = "✓" if passed else "✗"
    if not passed:
        FAILURES.append(label)
    print(f"  [{mark}] {label}{('：' + extra) if extra else ''}")


def _free_port() -> int:
    """拿一个刚关掉、确定没人监听的端口（用于演示"连不上"）。"""
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _build_program(robot: Robot):
    return (
        ProgramBuilder(robot)
        .moveJ(Q0, speed=40)
        .moveL(Pose.from_xyz_rpy(500, 0, 300, 0, 0, 0), speed=50, blend=8)
        .moveL(Pose.from_xyz_rpy(500, 200, 300, 0, 0, 0), speed=50, blend=8)
        .build()
    )


# --------------------------------------------------------------------------- #
# 第 1 节：起服务端
# --------------------------------------------------------------------------- #


def section_server(root: Path) -> CloudStoreServer:
    _hr("［1］起一个参考对象存储服务端（本机回环，纯标准库）")
    srv = CloudStoreServer(root, bucket="robotworld")
    port = srv.start()
    print(f"  服务地址：http://127.0.0.1:{port}/robotworld")
    print(f"  存储根目录：{root}")
    print("  描述：", json.dumps(srv.describe(), ensure_ascii=False))
    print()
    print("  REST 契约（自己实现一个兼容服务端只要几十行）：")
    print("    PUT    /<bucket>/<key>              请求体 = 对象内容")
    print("    GET    /<bucket>/<key>              响应体 = 对象内容（不存在 → 404）")
    print("    DELETE /<bucket>/<key>              200 删除 / 404 不存在")
    print('    GET    /<bucket>?prefix=<p>         响应体 {"keys": [...]}')
    return srv


# --------------------------------------------------------------------------- #
# 第 2 节：签名对着官方向量核对
# --------------------------------------------------------------------------- #


def section_sigv4() -> None:
    _hr("［2］SigV4 签名：与 AWS 官方文档示例向量逐字符核对")
    print("  向量来源：Amazon S3 文档「簽章計算示例」一节（GET 对象 / PUT 对象 /")
    print("  取桶生命周期 / 列举对象 四组），核对规范请求哈希与最终签名两处。")
    print()
    for name, method, url, headers, body, exp_hash, exp_sig in AWS_VECTORS:
        signer = SigV4Signer(AWS_ACCESS_KEY, AWS_SECRET_KEY, timestamp=AWS_TS)
        text, signed = signer.canonical_request_for(method, url, body, headers)
        got_hash = hashlib.sha256(text.encode("utf-8")).hexdigest()
        auth = signer.sign(method, url, body, headers)["Authorization"]
        got_sig = auth.split("Signature=")[1]
        _ok(f"{name} · 规范请求哈希", got_hash == exp_hash)
        _ok(f"{name} · 最终签名", got_sig == exp_sig)
        print(f"        已签头：{signed}")
    print()
    print(f"  无载荷时的哈希常量：{payload_hash(None)}")
    print("  （这点比「自洽」强一档：签名是否符合 SigV4 规则有**外部证据**。）")


# --------------------------------------------------------------------------- #
# 第 3 节：三种传输的契约一致性
# --------------------------------------------------------------------------- #


def section_contract(http_store: HTTPCloudStore, tmp: Path) -> None:
    _hr("［3］契约一致性：三种传输对同一串操作给出同一个结果")
    stores = [
        ("MemoryCloudStore（进程内）", MemoryCloudStore()),
        ("LocalCloudStore（本机目录）", LocalCloudStore(tmp / "local")),
        ("HTTPCloudStore（真 HTTP）", http_store),
    ]
    ops = [("put", "b/2.txt", b"two"), ("put", "b/1.txt", b"one"), ("delete", "b/2.txt", None),
           ("put", "b/3.txt", b"three")]
    for label, store in stores:
        for op, key, data in ops:
            if op == "put":
                store.put(key, data)
            else:
                store.delete(key)
    keysets = [store.keys() for _, store in stores]
    for (label, store), keys in zip(stores, keysets):
        print(f"  · {label:<28} keys = {keys}  请求数 = {getattr(store, 'requests', '—')}")
    _ok("三者键集合完全一致", keysets[0] == keysets[1] == keysets[2])
    http_store.put("empty.src", b"")
    _ok(
        "空对象与「不存在」分得清",
        http_store.get("empty.src") == b"" and http_store.get("ghost") is None,
    )
    _ok("穿越键在客户端就被拦掉（不上网）", _traversal_blocked(http_store))


def _traversal_blocked(store: HTTPCloudStore) -> bool:
    before = store.requests
    try:
        store.put("../evil.txt", b"x")
        return False
    except TwinError:
        return store.requests == before


# --------------------------------------------------------------------------- #
# 第 4 节：两个工位共用程序库与轨迹库
# --------------------------------------------------------------------------- #


def section_shared_library(server: CloudStoreServer, robot: Robot) -> None:
    _hr("［4］两个工位共用一套程序库与轨迹库（HTTP 传输真正的价值）")
    base = f"http://{server.address}"
    station_a = CloudLink(HTTPCloudStore(base, "robotworld"), robot, namespace="cellA")
    station_b = CloudLink(HTTPCloudStore(base, "robotworld"), robot, namespace="cellA")
    station_a.connect()
    station_b.connect()

    # --- 程序库：A 出码并部署，B 列出来并取回核对 ---
    program = _build_program(robot)
    planner = TrajectoryPlanner(robot)
    traj = planner.plan(program)
    report = planner.validate(program)
    post = get_post_processor("ur_polyscope")
    code = post.generate(program.items, metadata={"program_name": "PICK_HTTP"})
    print(f"  程序：{len(traj.waypoints)} 路点 / 节拍 {report.cycle_time:.2f}s / "
          f"{'有错误' if report.errors else '无错误'}")
    handle = station_a.upload_program(
        "PICK_HTTP", code, {"ext": "script", "brand": "UR", "post_processor": "ur_polyscope",
                            "robot_name": robot.name}
    )
    print(f"  工位 A 部署 → {handle}")
    print(f"  工位 B 看到的程序：{station_b.list_programs()}")
    back = station_b.fetch_program("PICK_HTTP", "script")
    _ok("B 取回的代码与 A 部署的逐字节一致", back == code, f"{len(code)} 字符")
    manifest = station_b.program_manifest("PICK_HTTP")
    print(f"  部署清单：品牌 {manifest['brand']} / {manifest['lines']} 行 / "
          f"{manifest['bytes']} 字节 / {time.strftime('%H:%M:%S', time.localtime(manifest['uploaded_at']))}")

    # --- 轨迹库：合成真机采一条，B 取回做跨源比对 ---
    plant_trace = capture_plant_trace(
        traj, robot, notes="合成传动误差模型（非真机实测，仅用于演示链路）"
    )
    station_a.publish_trace(plant_trace, "PICK_HTTP")
    meta = station_b.trace_meta("PICK_HTTP")
    print()
    print(f"  轨迹索引（不下载本体）：来源 {meta['source']} / {meta['samples']} 快照 / "
          f"{meta['duration_s']:.2f}s / 峰值 {meta['max_pos_mm']:.2f}mm")
    fetched = station_b.fetch_trace("PICK_HTTP")
    print(f"  工位 B 取回：{len(fetched)} 快照，来源仍标 {fetched.source!r}（存储位置不改变来源）")
    print(f"  附注：{fetched.notes}")

    sim_trace = capture_plant_trace(traj, robot, notes="纯仿真回读（无误差模型）")
    report_x = compare_traces(sim_trace, fetched, tol_pos_mm=1.0, notes="工位 B 取回的云上轨迹 vs 本机仿真")
    peak = report_x.samples and max(s.pos_dev_mm for s in report_x.samples)
    print()
    print(f"  跨源差分：{len(report_x.samples)} 对齐样本，未匹配 {report_x.unmatched}，"
          f"峰值偏差 {peak:.2f}mm，超容差 {len(report_x.over_tol)} 点")
    print(report_x.format_causes())

    # --- 命名空间隔离 ---
    other = CloudLink(HTTPCloudStore(base, "robotworld"), robot, namespace="cellB")
    other.connect()
    _ok("换命名空间就看不见（cellA 的程序不会串到 cellB）", other.list_programs() == [])
    print(f"  cellB 看到的程序：{other.list_programs()}")
    _ok("轨迹也在命名空间内隔离", other.list_traces() == [])

    # --- 状态回显 ---
    station_a.push_status([12.0, -25.0, 33.0, 0.0, 61.0, 1.5], source="real")
    age = station_b.status_age()
    print()
    print(f"  工位 A 上报状态 → 工位 B 回读：{'新鲜' if station_b.is_fresh(max_age=5.0) else '过期'}"
          f"（{age:.2f}s 前）")
    _ok("回读到的就是上报值", station_b.read_joints()[0] == 12.0)
    _ok("新鲜度可被质疑（3600s 前的上报不算实时）",
        (station_a.push_status([0.0] * 6, t=time.time() - 3600.0)
         or not station_a.is_fresh(max_age=5.0)))


# --------------------------------------------------------------------------- #
# 第 5 节：失败语义演练
# --------------------------------------------------------------------------- #


def _drill(fault: str, server_kwargs: dict | None, act, *, base: str | None = None,
           store_kwargs: dict | None = None) -> None:
    """故障演练：把"链路上出岔子"变成可复现的几行输出。"""
    root = Path(tempfile.mkdtemp(prefix="rwcloud-drill-"))
    srv = None
    if base is None:
        srv = CloudStoreServer(root, **(server_kwargs or {}))
        srv.start()
        url = f"http://{srv.address}"
    else:
        url = base
    store = HTTPCloudStore(url, "robotworld", **(store_kwargs or {}))
    try:
        try:
            act(store)
            detail = "（未按预期报错）"
        except (CloudHTTPError, CloudAuthError, TwinError) as exc:
            kind = type(exc).__name__
            detail = f"{kind}：{str(exc).split('（服务端') [0][:110]}"
        print(f"  · {fault}")
        print(f"      报错：{detail}")
        print(f"      仍认为云端可达：{'是' if store.reachable else '否 ✓'}"
              f"   请求次数：{store.requests}")
    finally:
        if srv is not None:
            srv.stop()


def section_failures() -> None:
    _hr("［5］失败语义演练：传输级故障 vs 业务级回答")
    print("  设计要点：连不上 / 5xx 是**传输级**（抛错并标记不可达）；404 是**业务级**")
    print("  正常回答（对象不存在，返回 None）；401/403 是**配置问题**（专门的鉴权异常）。")
    print()
    _drill("连不上（端口没人听）", None, lambda s: s.keys(), base=f"http://127.0.0.1:{_free_port()}",
           store_kwargs={"timeout": 0.5})
    _drill("服务端 5xx（注入 503）", {"fail_ops": {"PUT": -1}},
           lambda s: s.put("x.txt", b"v"))
    _drill("对象过大（413）", {"max_put_bytes": 16}, lambda s: s.put("big.txt", b"x" * 64))
    _drill("鉴权失败（401，未带 token）", {"token": "s3cr3t"}, lambda s: s.get("x.txt"))
    _drill("只读存储（403）", {"read_only": True}, lambda s: s.put("x.txt", b"v"))
    _drill("服务端很慢（压出客户端超时）", {"latency": 0.5}, lambda s: s.keys(),
           store_kwargs={"timeout": 0.12})
    _drill("SigV4 密钥错误（服务端复算不通过）",
           {"s3": {"access_key": AWS_ACCESS_KEY, "secret_key": AWS_SECRET_KEY}},
           lambda s: s.get("x.txt"),
           store_kwargs={"signer": SigV4Signer(AWS_ACCESS_KEY, "WRONG")})
    print()
    print("  对比：这类「正常回答」不该被当成故障——")
    root = Path(tempfile.mkdtemp(prefix="rwcloud-404-"))
    srv = CloudStoreServer(root)
    srv.start()
    try:
        store = HTTPCloudStore(f"http://{srv.address}", "robotworld")
        got = store.get("从来没有过的.txt")
        print(f"  · 取一个不存在的对象 → 返回 {got!r}   仍认为云端可达："
              f"{'是 ✓' if store.reachable else '否'}")
        _ok("404 不改变可达性判定", store.reachable and store.last_error is None)
    finally:
        srv.stop()


# --------------------------------------------------------------------------- #
# 第 6 节：服务端日志与落盘
# --------------------------------------------------------------------------- #


def section_dialog(server: CloudStoreServer, root: Path) -> None:
    _hr("［6］服务端请求日志与落盘布局")
    print(f"  共处理 {server.requests} 个请求，最近 {len(server.log)} 条：")
    for method, path, at in server.log[-12:]:
        print(f"    {at}  {method:<6} {path}")
    print()
    print("  落盘布局（<根>/<桶>/<键>，可直接用文件工具备份/迁移）：")
    files = sorted(p for p in root.rglob("*") if p.is_file())
    for f in files[:12]:
        print(f"    {f.relative_to(root).as_posix():<42} {f.stat().st_size} 字节")
    if len(files) > 12:
        print(f"    … 共 {len(files)} 个文件")


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #


def main() -> int:
    print("RobotWorld · HTTP / S3 兼容云通道演示（v1.0.6）")

    robot = Robot(str(URDF), "UR5e")
    tmp = Path(tempfile.mkdtemp(prefix="rwcloud-"))
    root = tmp / "cloud"

    server = section_server(root)
    try:
        http_store = HTTPCloudStore(f"http://{server.address}", "robotworld")
        section_sigv4()
        section_contract(http_store, tmp)
        section_shared_library(server, robot)
        section_failures()
        section_dialog(server, root)
    finally:
        print()
        print("  停止参考服务端…")
        server.stop()

    _hr("结果")
    if FAILURES:
        print(f"  有 {len(FAILURES)} 项未通过：")
        for f in FAILURES:
            print(f"    ✗ {f}")
        return 1
    print("  全部通过。")

    _hr("诚实说明（请勿略过）")
    print("  1. 本示例**没有云账号、也没有连过任何真实云端点**。服务端是仓库自带的")
    print("     参考实现，跑在本机回环上——「能通」只说明客户端与服务端彼此吻合，")
    print("     **不代表对真实 S3 / 对象存储做过端到端验证**（权限、加密、分块上传、")
    print("     一致性语义这些都没验过）。")
    print("  2. 签名有**外部证据**：SigV4Signer 的结果对着 AWS 官方文档的 4 组示例向量")
    print("     逐字符核对过（规范请求哈希 + 已签头 + 最终签名）。")
    print("  3. 第 4 节的偏差曲线来自**合成传动误差模型**（source=\"plant\"），不是现场")
    print("     实测数据，数字不能当验收依据；示例里已把它写在轨迹附注里。")
    print("  4. 接真云只需换 base_url / bucket / 凭证（S3 兼容存储再给一个 SigV4Signer），")
    print("     CloudLink、孪生层、调试会话、后处理一行都不用改。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
