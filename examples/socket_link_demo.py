#!/usr/bin/env python3
"""RobotWorld 示例：真机连接（Socket）全流程（v1.0.5）。

离线编程的最后一公里此前只有两种**离线**形态（`FileLink` 落盘、`SimLink` 进程内
仿真）。本示例走的是**真 TCP socket**：客户端 :class:`orolp.link.SocketLink` 连到
一个跑在本机的 :class:`orolp.linkstub.ControllerStubServer`（打桩控制器），把

    起打桩控制器 → 握手（版本/能力/dof） → 下发程序 → 列清单/取回核对
      → 逐点下发（时刻过线） + 回读实际位置 → 偏差 → 孪生轨迹采集 → 跨源比对
      → 故障演练（乱码/掉线/拒答/静默）

整条链路真跑一遍。

关于"真机"的诚实说明
--------------------

**本示例没有真机，也没有连过真机**。打桩控制器的关节回读用
:class:`~orolp.twinsync.PlantModel` 的**合成传动误差模型**（零位偏置 / 传动比误差 /
编码器噪声 / 一阶伺服滞后）产生，所以偏差曲线的形状是可解释的、可复现的，但
**它的数字不是现场实测，不能当验收依据引用**（代码里这条轨迹带 ``source="real"``
只表示"数据是经网络从控制器侧回读的"，抬头另附合成模型说明）。

接真机时要做的**只有一件事**：在控制器侧准备一段同协议转发（UR 可用 URScript 的
``socket_open``/``read``/``write`` 写一个小桩，或由 PC 侧代理进程把厂商 SDK/原生
协议翻译成这个协议），然后把 :class:`~orolp.link.SocketLink` 指向它即可——孪生层、
调试会话、后处理一行都不用改。协议本身只有一问一答的单行 JSON，见
``link.SocketLink`` 的类文档。

运行::

    python examples/socket_link_demo.py
    # 或未安装 orolp 时：PYTHONPATH=python python examples/socket_link_demo.py
"""

from __future__ import annotations

import json
import socket
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
for p in (str(REPO_ROOT / "python"), str(REPO_ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402

from orolp import Pose, ProgramBuilder, Robot  # noqa: E402
from orolp.debug import DebugSession  # noqa: E402
from orolp.link import LinkError, SimLink, SocketLink  # noqa: E402
from orolp.linkstub import ControllerStubServer  # noqa: E402
from orolp.simulation import Trajectory  # noqa: E402
from orolp.twinsync import (  # noqa: E402
    PlantModel,
    capture_from_session,
    compare_traces,
    deviation_chart_svg,
)

URDF = REPO_ROOT / "models/ur/ur5e/ur5e.urdf"
PROGRAM_NAME = "socket_pick"
Q0 = [0.0, -30.0, 30.0, 0.0, 60.0, 0.0]

#: 打桩控制器的"真机"参数（合成模型，不是现场实测）
PLANT = PlantModel(joint_bias_deg=0.25, scale_error=8e-4, noise_deg=0.01, lag_tau=0.02)


def _hr(title: str) -> None:
    print()
    print("=" * 72)
    print(title)
    print("=" * 72)


def _thin(traj: Trajectory, stride: int = 8) -> Trajectory:
    """抽稀轨迹：真机通路是**逐点一次 RPC**（下发 + 回读 + 位姿），示例不必跑满两千点。

    时间戳沿用原规划结果（仍单调），伺服滞后模型照常生效。
    """
    wps = list(traj.waypoints)
    if len(wps) <= 64:
        return traj
    picked = wps[::stride]
    if picked[-1] is not wps[-1]:
        picked.append(wps[-1])
    return Trajectory(picked)


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


def _drill(fault: str, stub_kwargs: dict, act, *, listen: bool = True) -> None:
    """故障演练：把"链路上出岔子"变成可复现的几行输出。

    ``listen=False`` 表示**故意不起服务端**（端口没人听），用来演示连接失败。
    """
    srv = None
    if listen:
        srv = ControllerStubServer(port=0, **stub_kwargs)
        srv.start()
        port = srv.port
    else:
        port = _free_port()
    link = SocketLink("127.0.0.1", port, timeout=0.4)
    try:
        try:
            link.connect()
            act(link)
            detail = "（未按预期报错）"
        except LinkError as exc:
            detail = str(exc).split("——")[0]
        print(f"  · {fault}")
        print(f"      报错：{detail}")
        print(f"      连接状态：{'已连接' if link.connected else '已断开 ✓'}")
    finally:
        link.disconnect()
        if srv is not None:
            srv.stop()


def main() -> int:
    robot = Robot(str(URDF), "UR5e")
    program = _build_program(robot)
    traj_full = program.plan()
    traj = _thin(traj_full)
    out_dir = Path(tempfile.mkdtemp(prefix="orolp_socket_"))

    # 打桩控制器：一台"带传动误差的 UR5e"，回读由 PlantLink 产生
    stub = ControllerStubServer(port=0, robot=robot, plant=PLANT)
    stub.start()

    try:
        # ---------------------------------------------------------------- #
        _hr("[1] 握手：协议版本 + 能力协商（真 TCP，不是 mock）")
        link = SocketLink("127.0.0.1", stub.port, robot=robot, name="127.0.0.1:%d" % stub.port)
        link.connect()
        print(f"  {link.describe()}")
        print(f"  服务端：{link.server_info.get('server')}（打桩，非真机）")
        print(f"  能力表：{link.caps}   dof={link.dof}")
        print("  说明：能力表由服务端声明——没有 feedback 时 read_joints() 会如实返回")
        print("        None，而**不会**拿本机运动学伪造一份'实际位置'。")

        # ---------------------------------------------------------------- #
        _hr("[2] 部署程序：Program.deploy(...) → 真机通路")
        dep = program.deploy(link, "ur_polyscope", name=PROGRAM_NAME)
        print(f"  句柄      ：{dep.handle}")
        print(f"  后处理器  ：{dep.post_processor}（品牌 {dep.brand}）")
        print(f"  代码规模  ：{len(dep.code.splitlines())} 行 / {len(dep.code.encode('utf-8'))} 字节")
        print(f"  控制器清单：{link.list_programs()}")

        # ---------------------------------------------------------------- #
        _hr("[3] 线上核对：把程序原文取回来逐字节比对")
        back = link.fetch_program(PROGRAM_NAME, "script")
        print(f"  fetch_program() → {'逐字节一致 ✓' if back == dep.code else '不一致 ✗'}")
        print(f"  控制器侧清单（打桩服务端记录）：{stub.state.manifests[PROGRAM_NAME]['meta']}")

        # ---------------------------------------------------------------- #
        _hr("[4] 逐点下发 + 回读：DebugSession 接在真机连接上")
        session = DebugSession(traj, link, robot=robot)
        print(f"  反馈可用（feedback_available）：{session.feedback_available}")
        print(f"  下发时携带程序时刻（set_joints 签名含 at_time）：{session._link_takes_time}")
        for i in range(0, min(len(traj.waypoints), 6)):
            session.goto_waypoint(i)
        summary = session.run()
        first_cmd = stub.state.commands[0]
        print(f"  已下发指令数：{len(stub.state.commands)}（每条都带 at_time）")
        print(
            f"  第一条指令：t={first_cmd['at_time']:.4f}s  关节 q1={first_cmd['joints'][0]:+.4f}°"
        )
        print(
            f"  回读峰值偏差（规划 vs 实际，单一来源）：位置 "
            f"{summary['max_pos_deviation_mm']:.3f} mm / 姿态 {summary['max_rot_deviation_deg']:.4f}°"
        )
        print("  注意：偏差全部来自打桩控制器的**合成**传动误差模型，非现场实测。")

        # ---------------------------------------------------------------- #
        _hr("[5] 孪生采集：偏差数据经网络回读，与仿真轨迹跨源比对")
        trace_real = capture_from_session(
            DebugSession(traj, link, robot=robot),
            source="real",
            robot_name=robot.name,
            program_name=PROGRAM_NAME,
            notes="经 SocketLink 从打桩控制器回读（合成传动误差模型，非现场实测）",
        )
        sim = SimLink(robot)
        sim.connect()
        trace_sim = capture_from_session(
            DebugSession(traj, sim, robot=robot), source="sim", robot_name=robot.name
        )
        report = compare_traces(trace_sim, trace_real, tol_pos_mm=1.0, tol_rot_deg=0.5)
        print(f"  采样点：真机 {len(trace_real)} / 仿真 {len(trace_sim)}")
        print(f"  抽稀：每 8 个规划点取 1（真机是逐点一次 RPC，示例不必跑满两千点）——")
        print("        起停段的低速样本因此被抽掉，下面的速度分解会如实报告")
        print("        \"证据不足、无法分离\"，而不是编一个数字出来。")
        print()
        print(report.format(max_rows=6))
        print()
        print(report.format_causes())
        chart = out_dir / "socket_deviation.svg"
        chart.write_text(deviation_chart_svg(report, theme="light"), encoding="utf-8")
        print(f"\n  偏差曲线 SVG：{chart}")

        # ---------------------------------------------------------------- #
        _hr("[6] 故障演练：网络级故障 vs 业务性拒绝（两类语义刻意不同）")
        print("  网络/协议级故障 → 抛错 + 立刻把连接标记为断开（承认断了）")
        _drill("连不上（端口没人听）", {}, lambda lk: None, listen=False)
        _drill("对端不是本协议", {"garbage": True}, lambda lk: lk.list_programs())
        _drill(
            "对端撂断（网线被拔）",
            {"drop_after": 2},
            lambda lk: (lk.upload_program("p", "code"), lk.list_programs()),
        )
        _drill("对端卡死（不回复）", {"silent": True}, lambda lk: lk.list_programs())
        print()
        print("  业务性拒绝（ok:false）→ 只抛异常、**不断开会话**（一次笔误不该废掉会话）")
        rej = ControllerStubServer(port=0, reject_ops={"upload"})
        rej.start()
        try:
            lk = SocketLink("127.0.0.1", rej.port, timeout=2.0)
            lk.connect()
            try:
                lk.upload_program("p", "code")
            except LinkError as exc:
                print(f"  · upload 被拒 → {str(exc).split('：', 1)[-1]}")
            print(f"  · 连接状态：{'已连接 ✓' if lk.connected else '已断开'}")
            print(f"  · 同一会话继续可用：list_programs() → {lk.list_programs()}")
            lk.disconnect()
        finally:
            rej.stop()

        # ---------------------------------------------------------------- #
        _hr("[7] 线上对话原文（打桩服务端留档，代码正文只留规模）")
        shown = stub.state.dialog[:4] + stub.state.dialog[-2:]
        for record in shown:
            print(f"  → {json.dumps(record['request'], ensure_ascii=False)}")
            print(f"  ← {json.dumps(record['response'], ensure_ascii=False)}")
        print(
            f"  共 {len(stub.state.dialog)} 条往返留档（容量上限 32），"
            f"op 全序列：{stub.state.requests[:6]} … {stub.state.requests[-3:]}"
        )

        link.disconnect()
    finally:
        stub.stop()

    _hr("闭环完成")
    print("起打桩控制器 → 握手 → 下发 → 取回核对 → 逐点下发(时刻过线) → 回读 →")
    print("偏差 → 孪生采集 → 跨源比对 → 故障演练 → 线上对话留档")
    print()
    print("诚实披露：")
    print("  · 本例**没有真机**，回读值来自 PlantModel 合成传动误差模型（source=real 只表示")
    print("    '数据经网络从控制器侧回读'，不代表现场实测）。")
    print("  · Socket 通路**未在真机上验证过**。接真机只需在控制器侧准备一段同协议转发")
    print("    （UR 用 URScript 的 socket_open/read/write，或 PC 侧代理翻译厂商 SDK），")
    print("    孪生层/调试会话/后处理无需改动。")
    print("  · 想自己动手：`python -m orolp.linkstub --port 30002 --urdf models/ur/ur5e/ur5e.urdf`")
    print("    起一个打桩控制器，然后把 SocketLink 指向 127.0.0.1:30002。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
