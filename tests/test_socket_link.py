"""真机连接（``SocketLink``）与打桩控制器（``ControllerStubServer``）的回归测试。

守住离线编程的"最后一公里"里的真机形态：

* 一轮真 TCP 往返——握手/能力协商 → 下发程序 → 列清单/取回 → 关节下发与回读 →
  位姿回读；
* **两种失败语义必须分开**——网络/协议级故障（连不上、超时、乱码、对端撂断）
  要立刻把连接标记为未连接；服务端业务性拒绝（``ok:false``）只抛异常、不断链；
* 能力要如实——控制器不支持回读时返回 ``None``，**不用本机运动学伪造"实际位置"**；
* 无真机也能整条链路验收——打桩服务端 + 合成真机模型把"部署 → 播放 → 回读 →
  偏差 → 孪生采集"跑通一遍，且走的是真 socket。

刻意不用 ``unittest.mock`` 顶掉 socket：这一层的价值恰恰在"网络上会不会出岔子"。
只有需要构造**打桩服务端自己都不该有的行为**（例如谎报关节数）时，才用文件末尾
那个最简 :class:`RawStub` 逐条脚本化应答。
"""

import base64
import json
import os
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

import orolp as orp  # noqa: E402
from orolp import Pose, Robot  # noqa: E402
from orolp.debug import DebugSession, _accepts_kwarg  # noqa: E402
from orolp.link import (  # noqa: E402
    DEFAULT_SOCKET_PORT,
    DEFAULT_SOCKET_TIMEOUT,
    SOCKET_API_VERSION,
    LinkError,
    SocketLink,
)
from orolp.linkstub import ControllerStubServer, StubState  # noqa: E402
from orolp.simulation import Trajectory  # noqa: E402
from orolp.twinsync import PlantModel, capture_from_session, compare_traces  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
URDF = REPO_ROOT / "models/ur/ur5e/ur5e.urdf"
Q0 = [0.0, -30.0, 30.0, 0.0, 60.0, 0.0]
PROGRAM_CODE = "// 抓取程序（中文注释）\nmovej([0,-30,30,0,60,0])\nmovej([10,-45,45,0,90,0])\n"


# --------------------------------------------------------------------------- #
# 夹具
# --------------------------------------------------------------------------- #


@pytest.fixture
def robot():
    return Robot(str(URDF), "UR5")


@pytest.fixture
def stub_factory():
    """造打桩服务端并在用例结束后统一收掉（端口是系统分配的，必须回收）。"""
    made = []

    def _make(**kwargs):
        srv = ControllerStubServer(port=0, **kwargs)
        srv.start()
        made.append(srv)
        return srv

    yield _make
    for srv in made:
        srv.stop()


@pytest.fixture
def stub(stub_factory):
    return stub_factory(dof=6, bias=0.05, scale=5e-4)


@pytest.fixture
def link(stub):
    lk = SocketLink("127.0.0.1", stub.port, timeout=5.0)
    lk.connect()
    yield lk
    lk.disconnect()


def _free_port() -> int:
    """拿一个"刚关掉、确定没人监听"的端口，用于验收连接失败路径。"""
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _demo_program(robot):
    return (
        orp.ProgramBuilder(robot)
        .moveJ(Q0, speed=40)
        .moveL(Pose.from_xyz_rpy(500, 0, 300, 0, 0, 0), speed=50, blend=8)
        .moveL(Pose.from_xyz_rpy(500, 200, 300, 0, 0, 0), speed=50, blend=8)
        .build()
    )


def _shorten(traj, stride: int = 16) -> Trajectory:
    """抽稀轨迹：真机通路是**逐点一次 RPC**，验收语义不必跑满两千个点。

    时间戳沿用原规划结果（仍然单调），所以伺服滞后模型照常生效。
    """
    wps = list(traj.waypoints)
    if len(wps) <= 64:
        return traj
    picked = wps[::stride]
    if picked[-1] is not wps[-1]:
        picked.append(wps[-1])
    return Trajectory(picked)


# --------------------------------------------------------------------------- #
# 一、握手与连接管理
# --------------------------------------------------------------------------- #


def test_default_address_and_protocol():
    lk = SocketLink()
    assert lk.protocol == "socket"
    assert lk.host == "127.0.0.1"
    assert lk.port == DEFAULT_SOCKET_PORT
    assert lk.timeout == DEFAULT_SOCKET_TIMEOUT
    assert lk.name == f"127.0.0.1:{DEFAULT_SOCKET_PORT}"
    assert lk.connected is False
    # 没连上之前不能声称有回读能力（能力来自握手，不是类常量）
    assert lk.is_actual_feedback is False
    assert lk.dof is None
    assert lk.caps == {}


def test_handshake_negotiates_caps_dof_and_server(link):
    assert link.connected is True
    assert link.caps == {"programs": True, "feedback": True, "pose": True}
    assert link.dof == 6
    assert link.is_actual_feedback is True
    assert link.server_info["server"] == "orolp.ControllerStubServer"
    # 握手本身算一次请求
    assert link.requests == 1


def test_handshake_reports_no_feedback_capability(stub_factory):
    srv = stub_factory(dof=6, caps={"feedback": False})
    lk = SocketLink("127.0.0.1", srv.port)
    lk.connect()
    try:
        assert lk.caps["feedback"] is False
        assert lk.is_actual_feedback is False
        # 不支持回读就如实返回 None，而不是拿本机运动学伪造"实际位置"
        assert lk.read_joints() is None
    finally:
        lk.disconnect()


def test_handle_carries_real_address(stub_factory):
    srv = stub_factory()
    lk = SocketLink("127.0.0.1", srv.port)
    lk.connect()
    try:
        handle = lk.upload_program("pick", "code", {"ext": "script"})
        assert handle == f"sock://127.0.0.1:{srv.port}/programs/pick.script"
    finally:
        lk.disconnect()


def test_connect_failure_explains_where_and_how():
    port = _free_port()
    lk = SocketLink("127.0.0.1", port, timeout=1.0)
    with pytest.raises(LinkError) as exc:
        lk.connect()
    msg = str(exc.value)
    assert f"127.0.0.1:{port}" in msg
    assert "连接失败" in msg
    # 报错要给出可操作的下一步，而不是只有 errno
    assert "orolp.linkstub" in msg
    assert lk.connected is False
    assert lk._sock is None


def test_api_version_mismatch_is_rejected_at_handshake(stub_factory):
    srv = stub_factory(api=SOCKET_API_VERSION + 1)
    lk = SocketLink("127.0.0.1", srv.port, timeout=2.0)
    with pytest.raises(LinkError) as exc:
        lk.connect()
    assert "协议版本不匹配" in str(exc.value)
    assert lk.connected is False
    assert lk._sock is None  # 握手失败不能留下半开的连接


def test_reconnect_replaces_previous_connection(stub_factory):
    srv = stub_factory()
    lk = SocketLink("127.0.0.1", srv.port)
    lk.connect()
    try:
        lk.connect()  # 重复 connect ＝ 重连
        assert lk.connected is True
        assert lk.list_programs() == []
        deadline = time.time() + 2.0
        while srv.state.clients < 2 and time.time() < deadline:
            time.sleep(0.01)
        assert srv.state.clients >= 2
    finally:
        lk.disconnect()


def test_context_manager_connects_and_disconnects(stub):
    with SocketLink("127.0.0.1", stub.port) as lk:
        assert lk.connected is True
    assert lk.connected is False
    assert lk._sock is None


def test_disconnect_is_idempotent(stub):
    lk = SocketLink("127.0.0.1", stub.port)
    lk.connect()
    lk.disconnect()
    lk.disconnect()  # 第二次不该抛
    assert lk.connected is False


def test_operations_without_connect_raise(stub):
    lk = SocketLink("127.0.0.1", stub.port)
    with pytest.raises(LinkError):
        lk.upload_program("p", "code")
    with pytest.raises(LinkError):
        lk.list_programs()
    with pytest.raises(LinkError):
        lk.read_joints()
    with pytest.raises(LinkError):
        lk.set_joints(Q0)
    with pytest.raises(LinkError):
        lk.get_pose()
    with pytest.raises(LinkError):
        lk.fetch_program("p")


def test_stopped_server_does_not_leave_link_lying(stub_factory):
    srv = stub_factory()
    lk = SocketLink("127.0.0.1", srv.port, timeout=2.0)
    lk.connect()
    srv.stop()
    with pytest.raises(LinkError):
        lk.list_programs()
    # 关键：断了就要承认断了，不能把坏掉的连接继续当"已连接"用
    assert lk.connected is False
    assert lk._sock is None


# --------------------------------------------------------------------------- #
# 二、程序下发 / 清单 / 取回
# --------------------------------------------------------------------------- #


def test_upload_stores_code_and_manifest(link, stub):
    handle = link.upload_program(
        "pick_place", PROGRAM_CODE, {"ext": "script", "brand": "ur", "post_processor": "ur_polyscope"}
    )
    assert handle == f"sock://127.0.0.1:{stub.port}/programs/pick_place.script"
    assert stub.state.programs["pick_place"] == PROGRAM_CODE
    manifest = stub.state.manifests["pick_place"]
    assert manifest["ext"] == "script"
    assert manifest["lines"] == 3
    assert manifest["bytes"] == len(PROGRAM_CODE.encode("utf-8"))
    assert manifest["meta"] == {"ext": "script", "brand": "ur", "post_processor": "ur_polyscope"}
    assert link.list_programs() == ["pick_place"]
    assert link.uploaded == ["pick_place"]


def test_upload_default_ext_is_src(link):
    handle = link.upload_program("weld", "PTP x")
    assert handle.endswith("/programs/weld.src")


def test_upload_sanitizes_weird_program_name(link, stub):
    handle = link.upload_program("抓取/程序 v2", "code")
    # 中文属 isalnum → 保留；斜杠与空格替换成下划线
    assert handle.endswith("/programs/抓取_程序_v2.src")
    assert link.list_programs() == ["抓取/程序 v2"]  # 服务器侧保留原名


def test_upload_sanitizes_leading_dots(link):
    assert link.upload_program("..", "code").endswith("/programs/program.src")
    assert link.upload_program("", "code").endswith("/programs/program.src")


def test_upload_metadata_is_made_json_safe(link, stub):
    link.upload_program(
        "p", "code", {"robot_name": "UR5", "path": Path("/tmp/x.urdf"), "nested": {"a": 1}}
    )
    meta = stub.state.manifests["p"]["meta"]
    assert meta["robot_name"] == "UR5"
    assert meta["path"] == "/tmp/x.urdf"  # Path → str，不能到部署那一刻才炸
    assert isinstance(meta["nested"], str)


def test_upload_empty_code_is_allowed(link, stub):
    """空程序是合法内容：清单记 0 行，取回也应当回 "" 而不是"没有这个程序"。"""
    link.upload_program("empty", "")
    assert stub.state.manifests["empty"]["lines"] == 0
    assert stub.state.manifests["empty"]["bytes"] == 0
    assert link.fetch_program("empty") == ""
    assert link.fetch_program("never_uploaded") is None


def test_upload_same_name_overwrites_and_stays_single_entry(link, stub):
    link.upload_program("p", "v1")
    link.upload_program("p", "v2")
    assert link.list_programs() == ["p"]
    assert link.fetch_program("p") == "v2"
    assert link.uploaded == ["p"]  # 本机去重，不重复记


def test_fetch_program_round_trip_and_missing(link):
    link.upload_program("p", PROGRAM_CODE, {"ext": "script"})
    assert link.fetch_program("p", "script") == PROGRAM_CODE
    assert link.fetch_program("nope") is None


def test_list_programs_sorted(link):
    for name in ("zeta", "alpha", "mid"):
        link.upload_program(name, "code")
    assert link.list_programs() == ["alpha", "mid", "zeta"]


def test_full_program_deploy_over_socket(robot, stub):
    """真机通路验收：`Program.deploy` 产出的代码必须与控制器上的逐字节一致。"""
    program = _demo_program(robot)
    with SocketLink("127.0.0.1", stub.port) as lk:
        dep = program.deploy(lk, "ur_polyscope", name="pick_place")
        assert dep.ok is True
        assert dep.brand == "ur"
        assert dep.post_processor == "ur_polyscope"
        assert dep.handle.startswith("sock://")
        assert dep.code == stub.state.programs["pick_place"]
        assert lk.fetch_program("pick_place", "script") == dep.code
        assert stub.state.manifests["pick_place"]["meta"]["brand"] == "ur"


# --------------------------------------------------------------------------- #
# 三、关节下发与回读
# --------------------------------------------------------------------------- #


def test_set_joints_then_read_round_trip_without_error_model(stub_factory):
    srv = stub_factory(dof=6)  # 无偏差无缩放：应该一模一样读回来
    with SocketLink("127.0.0.1", srv.port) as lk:
        lk.set_joints(Q0)
        back = lk.read_joints()
    assert back is not None
    assert np.allclose(back, Q0, atol=1e-12)


def test_read_joints_reflects_error_model(link):
    link.set_joints(Q0)
    back = link.read_joints()
    assert back is not None
    assert not np.allclose(back, Q0)  # 打桩服务端带偏移+缩放，回读不该"刚好等于指令"
    assert np.allclose(back, np.asarray(Q0) * (1.0 + 5e-4) + 0.05, atol=1e-9)


def test_set_joints_forwards_at_time(link, stub):
    link.set_joints(Q0, at_time=0.42)
    assert stub.state.commands == [{"joints": [float(v) for v in Q0], "at_time": 0.42}]
    # DebugSession 用签名探测决定要不要传时刻，所以签名里必须有 at_time
    assert _accepts_kwarg(link.set_joints, "at_time") is True


def test_set_joints_without_at_time_sends_none(link, stub):
    link.set_joints(Q0)
    assert stub.state.commands[0]["at_time"] is None


def test_set_joints_wrong_dof_is_business_reject_and_keeps_connection(link):
    with pytest.raises(LinkError) as exc:
        link.set_joints([0.0, 0.0, 0.0])
    assert "关节数不符" in str(exc.value)
    assert link.connected is True  # 一次笔误不该废掉整台机器的会话
    link.set_joints(Q0)  # 立刻还能继续用


def test_set_joints_empty_array_locally_rejected(link):
    with pytest.raises(LinkError) as exc:
        link.set_joints([])
    assert "空关节数组" in str(exc.value)


def test_read_joints_returns_none_when_server_says_none(stub_factory):
    """服务端声明支持回读但这次给了 null（例如示教器占用）→ 如实返回 None。"""
    raw = RawStub(
        [
            _HELLO_OK,
            {"ok": True, "joints": None},
        ]
    ).start()
    lk = SocketLink("127.0.0.1", raw.port, timeout=2.0)
    lk.connect()
    try:
        assert lk.read_joints() is None
    finally:
        lk.disconnect()
        raw.stop()


def test_read_joints_rejects_joint_count_mismatch():
    """服务端谎报关节数（握手 6 轴、回读 7 个）→ 必须报错，不能悄悄传下去。"""
    raw = RawStub([_HELLO_OK, {"ok": True, "joints": [0.0] * 7}]).start()
    lk = SocketLink("127.0.0.1", raw.port, timeout=2.0)
    lk.connect()
    try:
        with pytest.raises(LinkError) as exc:
            lk.read_joints()
        assert "关节数不符" in str(exc.value)
    finally:
        lk.disconnect()
        raw.stop()


def test_read_joints_rejects_non_numeric_payload():
    raw = RawStub([_HELLO_OK, {"ok": True, "joints": ["a", "b"]}]).start()
    lk = SocketLink("127.0.0.1", raw.port, timeout=2.0)
    lk.connect()
    try:
        with pytest.raises(LinkError) as exc:
            lk.read_joints()
        assert "不是数值数组" in str(exc.value)
    finally:
        lk.disconnect()
        raw.stop()


# --------------------------------------------------------------------------- #
# 四、位姿回读
# --------------------------------------------------------------------------- #


def test_get_pose_from_controller(robot, stub_factory):
    srv = stub_factory(robot=robot, plant=PlantModel())
    with SocketLink("127.0.0.1", srv.port, robot=robot) as lk:
        lk.set_joints(Q0)
        pose = lk.get_pose()
        joints = lk.read_joints()
    assert pose is not None
    # 控制器给的位姿应当就是它自己那份关节状态的正解
    expected = robot.forward_kinematics(joints)
    assert np.allclose(pose.to_xyz_rpy()[:3], expected.to_xyz_rpy()[:3], atol=1e-6)


def test_get_pose_falls_back_to_local_fk(robot, stub_factory):
    srv = stub_factory(robot=robot, caps={"pose": False})
    with SocketLink("127.0.0.1", srv.port, robot=robot) as lk:
        lk.set_joints(Q0)
        pose = lk.get_pose()
        joints = lk.read_joints()
    assert pose is not None
    assert np.allclose(pose.to_xyz_rpy()[:3], robot.forward_kinematics(joints).to_xyz_rpy()[:3], atol=1e-6)
    assert "get_pose" not in srv.state.requests  # 既然不支持位姿就别去问


def test_get_pose_none_without_controller_pose_or_robot(stub_factory):
    srv = stub_factory(caps={"pose": False})
    with SocketLink("127.0.0.1", srv.port) as lk:
        lk.set_joints(Q0)
        assert lk.get_pose() is None


def test_get_pose_rejects_malformed_payload():
    raw = RawStub([_HELLO_OK, {"ok": True, "pose": [1.0, 2.0, 3.0]}]).start()
    lk = SocketLink("127.0.0.1", raw.port, timeout=2.0)
    lk.connect()
    try:
        with pytest.raises(LinkError) as exc:
            lk.get_pose()
        assert "期望 6" in str(exc.value)
    finally:
        lk.disconnect()
        raw.stop()


# --------------------------------------------------------------------------- #
# 五、两种失败语义：网络级故障 vs 业务性拒绝
# --------------------------------------------------------------------------- #


def test_business_reject_keeps_connection_alive(stub_factory):
    srv = stub_factory(dof=6, reject_ops={"upload"})
    with SocketLink("127.0.0.1", srv.port) as lk:
        with pytest.raises(LinkError) as exc:
            lk.upload_program("p", "code")
        assert "服务端拒绝 upload" in str(exc.value)
        assert lk.connected is True
        assert lk.list_programs() == []  # 会话还能继续用


def test_garbage_response_marks_link_disconnected(stub_factory):
    srv = stub_factory(dof=6, garbage=True)
    lk = SocketLink("127.0.0.1", srv.port, timeout=2.0)
    with pytest.raises(LinkError):
        lk.connect()
    assert lk.connected is False
    assert lk._sock is None


def test_garbage_after_connect_marks_disconnected(stub_factory):
    """连接建立后再变乱码（典型场景：连错端口连到了别的服务/固件）。"""
    srv = stub_factory(dof=6)
    lk = SocketLink("127.0.0.1", srv.port, timeout=2.0)
    lk.connect()
    srv.garbage = True  # 握手之后才发疯
    with pytest.raises(LinkError) as exc:
        lk.list_programs()
    assert "合法 JSON" in str(exc.value)
    assert lk.connected is False


def test_dropped_connection_is_reported_and_marks_disconnected(stub_factory):
    # 握手 1 条 + upload 1 条答完就撂断（模拟现场网线被拔/控制器重启）
    srv = stub_factory(dof=6, drop_after=2)
    lk = SocketLink("127.0.0.1", srv.port, timeout=2.0)
    lk.connect()
    lk.upload_program("p", "code")
    with pytest.raises(LinkError) as exc:
        lk.list_programs()
    assert "对端关闭" in str(exc.value)
    assert lk.connected is False


def test_silent_server_times_out(stub_factory):
    srv = stub_factory(dof=6, silent=True)
    lk = SocketLink("127.0.0.1", srv.port, timeout=0.3)
    with pytest.raises(LinkError) as exc:
        lk.connect()
    assert "超时" in str(exc.value)
    assert lk.connected is False


def test_response_without_ok_field_marks_disconnected():
    raw = RawStub([{"api": 1}]).start()
    lk = SocketLink("127.0.0.1", raw.port, timeout=2.0)
    with pytest.raises(LinkError) as exc:
        lk.connect()
    assert "缺少 ok 字段" in str(exc.value)
    assert lk.connected is False
    raw.stop()


def test_server_message_is_surfaced_to_caller(stub_factory):
    srv = stub_factory(dof=6, reject_ops={"list"})
    with SocketLink("127.0.0.1", srv.port) as lk:
        with pytest.raises(LinkError) as exc:
            lk.list_programs()
    assert "打桩服务端拒绝 list" in str(exc.value)


# --------------------------------------------------------------------------- #
# 六、端到端：部署 → 播放 → 回读 → 偏差 → 孪生
# --------------------------------------------------------------------------- #


def test_debug_session_over_socket_reports_feedback(robot, stub_factory):
    srv = stub_factory(robot=robot, plant=PlantModel())
    program = _demo_program(robot)
    traj = _shorten(program.plan())
    with SocketLink("127.0.0.1", srv.port, robot=robot) as lk:
        program.deploy(lk, "ur_polyscope", name="pick_place")
        session = DebugSession(traj, lk, robot=robot)
        assert session.feedback_available is True
        summary = session.run(sample_dt=0.05)
    assert summary["num_waypoints"] == len(traj.waypoints)
    assert summary["feedback_available"] is True
    assert summary["max_pos_deviation_mm"] >= 0.0


def test_debug_session_sends_timestamps_over_the_wire(robot, stub_factory):
    """时刻要真的过线：服务端收到的 at_time 必须是轨迹时间戳，不是常量 0。"""
    srv = stub_factory(robot=robot, plant=PlantModel(lag_tau=0.02))
    program = _demo_program(robot)
    traj = program.plan()
    with SocketLink("127.0.0.1", srv.port, robot=robot) as lk:
        session = DebugSession(traj, lk, robot=robot)
        assert session._link_takes_time is True
        session.goto_waypoint(0)
        session.goto_waypoint(1)
    times = [c["at_time"] for c in srv.state.commands]
    assert times[0] == pytest.approx(float(traj.waypoints[0].timestamp), abs=1e-9)
    assert times[1] == pytest.approx(float(traj.waypoints[1].timestamp), abs=1e-9)
    assert times[1] > times[0]  # 时间轴确实在推进


def test_twin_capture_over_real_socket_and_compare_with_sim(robot, stub_factory):
    """孪生链路的真机形态：偏差来自**线上回读**，而不是本机仿真自说自话。"""
    program = _demo_program(robot)
    traj = _shorten(program.plan())
    plant = PlantModel(joint_bias_deg=0.3, scale_error=1e-3, lag_tau=0.02)

    srv = stub_factory(robot=robot, plant=plant)
    with SocketLink("127.0.0.1", srv.port, robot=robot) as lk:
        trace_socket = capture_from_session(
            DebugSession(traj, lk, robot=robot),
            source="real",
            robot_name="UR5",
            program_name="pick_place",
        )

    from orolp.link import SimLink

    sim = SimLink(robot)
    sim.connect()
    trace_sim = capture_from_session(
        DebugSession(traj, sim, robot=robot), source="sim", robot_name="UR5"
    )

    assert trace_socket.source == "real"
    assert len(trace_socket) == len(traj.waypoints)
    assert trace_socket.feedback_available is True
    # 仿真侧只该有数值噪声（控制器＝本机运动学本身），线上侧必须是真偏差
    sim_dev = trace_sim.max_pos_dev()
    sock_dev = trace_socket.max_pos_dev()
    assert sim_dev < 0.05, f"仿真侧偏差应是数值噪声，实测 {sim_dev}mm"
    assert sock_dev > 1.0, f"线上回读的偏差应显著非零，实测 {sock_dev}mm"
    assert sock_dev > 10 * max(sim_dev, 1e-9)
    assert compare_traces(trace_sim, trace_socket) is not None


def test_two_clients_share_one_stub(stub_factory):
    """打桩服务端是线程化的，且**如实模拟"一台控制器"**：两个工位可以同时上线，
    但机器人状态是共享的（真机也是如此），不是每个客户端各有一台虚拟机器。"""
    srv = stub_factory(dof=6)
    a = SocketLink("127.0.0.1", srv.port)
    b = SocketLink("127.0.0.1", srv.port)
    a.connect()
    b.connect()
    try:
        a.upload_program("fa", "code-a")
        b.upload_program("fb", "code-b")
        assert a.list_programs() == ["fa", "fb"]
        assert b.list_programs() == ["fa", "fb"]
        a.set_joints([1.0] * 6)
        assert np.allclose(a.read_joints(), [1.0] * 6)
        b.set_joints([2.0] * 6)
        assert np.allclose(a.read_joints(), [2.0] * 6)  # 同一台机器，同一个状态
        assert np.allclose(b.read_joints(), [2.0] * 6)
        assert srv.state.clients == 2
    finally:
        a.disconnect()
        b.disconnect()


def test_report_and_describe_are_honest(stub):
    lk = SocketLink("127.0.0.1", stub.port)
    assert "未连接" in lk.describe()
    lk.connect()
    try:
        text = lk.describe()
        assert "已连接" in text
        assert "orolp.ControllerStubServer" in text
        assert f"api=v{SOCKET_API_VERSION}" in text
        assert "feedback" in text
    finally:
        lk.disconnect()
    assert "disconnected" in repr(lk)


# --------------------------------------------------------------------------- #
# 七、打桩控制器自身（不走网络，直接驱动状态机）
# --------------------------------------------------------------------------- #


def test_state_handle_hello_and_unknown_op():
    st = StubState(dof=6)
    hello = st.handle({"op": "hello"})
    assert hello["ok"] is True
    assert hello["api"] == SOCKET_API_VERSION
    assert hello["dof"] == 6
    assert hello["caps"] == {"programs": True, "feedback": True, "pose": True}
    bad = st.handle({"op": "launch_missile"})
    assert bad["ok"] is False
    assert "不支持的 op" in bad["error"]
    assert st.handle({"op": "bye"})["closing"] is True
    assert st.requests[-3:] == ["hello", "launch_missile", "bye"]


def test_state_rejects_bad_base64_and_bad_joints():
    st = StubState(dof=6)
    bad_b64 = st.handle({"op": "upload", "name": "p", "code_b64": "not base64!!"})
    assert bad_b64["ok"] is False and "code_b64" in bad_b64["error"]
    assert st.handle({"op": "set_joints"})["ok"] is False
    assert st.handle({"op": "set_joints", "joints": ["x"] * 6})["ok"] is False
    assert st.handle({"op": "set_joints", "joints": [0.0] * 5})["ok"] is False
    good = st.handle(
        {"op": "upload", "name": "p", "code_b64": base64.b64encode(b"hi").decode()}
    )
    assert good["ok"] is True
    assert st.programs["p"] == "hi"


def test_state_can_disable_program_capability():
    st = StubState(dof=6, caps={"programs": False})
    assert st.handle({"op": "upload", "name": "p", "code_b64": ""})["ok"] is False
    assert st.handle({"op": "list"})["ok"] is False
    assert st.handle({"op": "fetch", "name": "p"})["ok"] is False
    # 回读仍然可用：能力是逐项协商的，不是"要么全能要么全不能"
    assert st.handle({"op": "read_joints"})["ok"] is True


def test_state_pose_is_none_without_robot():
    st = StubState(dof=6)
    assert st.pose() is None
    assert st.handle({"op": "get_pose"})["pose"] is None


def test_stub_rejects_bad_bias_length():
    with pytest.raises(ValueError):
        StubState(dof=6, bias=[0.1, 0.2])


def test_stub_describe_mentions_state(stub):
    text = stub.describe()
    assert "ControllerStubServer[127.0.0.1:" in text
    assert "dof=6" in text


def test_state_dialog_records_abbreviated_round_trips():
    """往返报文要能"出示证据"：op 与标量留全，程序正文只留规模，数组只留前几项。"""
    st = StubState(dof=6, dialog_capacity=4)
    st.handle({"op": "hello"})
    st.handle(
        {
            "op": "upload",
            "name": "p",
            "ext": "src",
            "code_b64": base64.b64encode(b"x" * 500).decode(),
        }
    )
    st.handle({"op": "set_joints", "joints": [1.0] * 6, "at_time": 0.5})
    assert [d["op"] for d in st.dialog] == ["hello", "upload", "set_joints"]
    upload = st.dialog[1]
    assert upload["request"]["code_b64"].startswith("<")  # 正文只留规模
    assert upload["request"]["name"] == "p"
    assert upload["response"]["handle"].endswith("/programs/p.src")
    sent = st.dialog[2]["request"]
    assert sent["at_time"] == 0.5
    assert len(sent["joints"]) == 4  # 6 项 → 前 3 项 + "…共 6 项"
    for _ in range(5):
        st.handle({"op": "list"})
    assert len(st.dialog) == 4  # 容量上限，不会无限长
    assert st.dialog[-1]["op"] == "list"
    assert st.requests.count("list") == 5  # 但完整 op 序列仍全程留档


def test_state_dialog_can_be_disabled():
    st = StubState(dof=6, dialog_capacity=0)
    st.handle({"op": "hello"})
    assert st.dialog == []


def test_exports_are_reachable_from_package_root():
    """包根要能拿到这两个名字——否则用户得先知道模块布局才用得起来。"""
    import orolp

    assert orolp.SocketLink is SocketLink
    assert orolp.linkstub.ControllerStubServer is ControllerStubServer
    assert "SocketLink" in orolp.__all__


# --------------------------------------------------------------------------- #
# 八、命令行入口（真实子进程）
# --------------------------------------------------------------------------- #


def test_cli_serves_and_exits_by_itself():
    env = dict(os.environ)
    env["PYTHONPATH"] = str(REPO_ROOT / "python") + os.pathsep + env.get("PYTHONPATH", "")
    proc = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "orolp.linkstub",
            "--port",
            "0",
            "--print-address",
            "--max-clients",
            "1",
            "--bias-all",
            "0.1",
        ],
        cwd=str(REPO_ROOT),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        address = None
        deadline = time.time() + 20.0
        while time.time() < deadline:
            line = proc.stdout.readline()
            if not line:
                break
            if line.startswith("ADDRESS "):
                address = line.split(None, 1)[1].strip()
                break
        assert address, "命令行没有输出 ADDRESS 行"
        host, _, port = address.rpartition(":")
        with SocketLink(host, int(port), timeout=3.0) as lk:
            assert lk.connected is True
            assert lk.caps["feedback"] is True
            lk.set_joints(Q0)
            assert lk.read_joints()[0] == pytest.approx(Q0[0] * 1.0 + 0.1, abs=1e-9)
        # --max-clients 1：客户端断开后服务端应当自己退出
        assert proc.wait(timeout=15) == 0
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait(timeout=5)


def test_example_script_runs_end_to_end():
    """示例脚本必须真能跑完（README/网页都指着它说"一条命令可复现"）。"""
    env = dict(os.environ)
    env["PYTHONPATH"] = str(REPO_ROOT / "python") + os.pathsep + env.get("PYTHONPATH", "")
    env.setdefault("PYTHONIOENCODING", "utf-8")
    proc = subprocess.run(
        [sys.executable, str(REPO_ROOT / "examples/socket_link_demo.py")],
        cwd=str(REPO_ROOT),
        env=env,
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert proc.returncode == 0, (proc.stdout[-3000:] + proc.stderr[-3000:])
    out = proc.stdout
    assert "SocketLink[127.0.0.1:" in out
    assert "逐字节一致" in out
    assert "已断开" in out
    assert "没有真机" in out  # 诚实披露必须在输出里
    assert "socket_deviation.svg" in out


# --------------------------------------------------------------------------- #
# 九、最简脚本化应答器（构造"打桩服务端自己都不该有的行为"）
# --------------------------------------------------------------------------- #

_HELLO_OK = {
    "ok": True,
    "api": SOCKET_API_VERSION,
    "server": "RawStub",
    "dof": 6,
    "caps": {"programs": True, "feedback": True, "pose": True},
}


class RawStub:
    """逐条脚本化应答的 TCP 服务端（测试专用）。

    ``replies`` 按请求顺序取用；元素可以是 dict（转成 JSON 行）、bytes（原样发）、
    ``None``（撂断连接）或 ``callable(request_dict) -> 同上``。超出脚本长度的请求
    不再应答（客户端会看到 EOF）。
    """

    def __init__(self, replies):
        self.replies = list(replies)
        self.requests = []
        self._sock = socket.socket()
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.listen(4)
        self.port = self._sock.getsockname()[1]
        self._stopped = False
        self._thread = threading.Thread(target=self._serve, name="raw-stub", daemon=True)

    def start(self) -> "RawStub":
        self._thread.start()
        return self

    def _serve(self) -> None:
        while not self._stopped:
            try:
                conn, _ = self._sock.accept()
            except OSError:
                return
            try:
                self._serve_one(conn)
            finally:
                try:
                    conn.close()
                except OSError:
                    pass

    def _serve_one(self, conn) -> None:
        reader = conn.makefile("rb")
        index = 0
        while not self._stopped:
            line = reader.readline()
            if not line:
                return
            self.requests.append(line)
            if index >= len(self.replies):
                return
            reply = self.replies[index]
            index += 1
            if callable(reply):
                reply = reply(line)
            if reply is None:
                return
            payload = reply if isinstance(reply, bytes) else (
                json.dumps(reply, ensure_ascii=False).encode("utf-8") + b"\n"
            )
            try:
                conn.sendall(payload)
            except OSError:
                return

    def stop(self) -> None:
        self._stopped = True
        try:
            self._sock.close()
        except OSError:
            pass
