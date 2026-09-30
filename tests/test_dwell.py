"""waitTime 驻留段时间轴（v1.1.3）验收。

v1.1.2 之前 ``WaitTime`` 对规划器完全不可见：节拍少算全部驻留时间，多机
时间轴也看不见"驻留占用"——"A 在交接点驻留、B 扫过来"这类最经典的机器人间
冲突在仿真里根本不存在。本轮把驻留接进时间轴，语义变化与影响面全部在此验证。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

# core 在 <repo>/python 下；自插路径保证单文件可跑（与 test_packaging_spec 同法）
_SRC = str(Path(__file__).resolve().parents[1] / "python")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

import numpy as np  # noqa: E402

from domain.robot import Robot
from service.planner import TrajectoryPlanner
from service.program import ProgramBuilder  # noqa: E402


@pytest.fixture()
def robot():
    return Robot(
        str(Path(__file__).resolve().parents[1] / "assets/models/ur/ur5e/ur5e.urdf"), "UR5e"
    )


Q0 = [0.0, -90.0, 90.0, -90.0, 0.0, 0.0]
Q1 = [30.0, -70.0, 80.0, -90.0, 25.0, 0.0]


# --------------------------- 节拍：驻留必须计入 --------------------------- #


def test_cycle_time_includes_dwell(robot):
    """同一条程序，加了 waitTime(0.8) 节拍必须精确多出 0.8s。"""
    base = ProgramBuilder(robot).moveJ(Q0, speed=40).moveJ(Q1, speed=40).build().plan().duration()
    with_dwell = (
        ProgramBuilder(robot)
        .moveJ(Q0, speed=40)
        .waitTime(0.8)
        .moveJ(Q1, speed=40)
        .build()
        .plan()
        .duration()
    )
    assert with_dwell == pytest.approx(base + 0.8, abs=1e-6)


def test_dwell_waypoints_hold_pose(robot):
    """驻留段的路点关节值恒定（时间在走、机器人没动）。"""
    traj = ProgramBuilder(robot).moveJ(Q0, speed=40).waitTime(0.5).build().plan()
    wps = traj.waypoints
    assert len(wps) >= 3
    tail = wps[-2:]
    # 驻留末端路点与前一路点（运动末端）关节完全一致
    assert np.allclose(tail[0].joints, tail[1].joints, atol=1e-12)
    # 时间戳确实推进了 0.5s
    assert tail[1].timestamp - tail[0].timestamp == pytest.approx(0.5, abs=1e-9)
    # 驻留段速度为零（差分动力学填充）
    assert np.allclose(tail[1].velocity, 0.0, atol=1e-9)


def test_multiple_dwells_sum(robot):
    prog = (
        ProgramBuilder(robot)
        .waitTime(0.2)
        .moveJ(Q0, speed=40)
        .waitTime(0.3)
        .moveJ(Q1, speed=40)
        .waitTime(0.1)
        .build()
    )
    traj = prog.plan()
    base = ProgramBuilder(robot).moveJ(Q0, speed=40).moveJ(Q1, speed=40).build().plan().duration()
    assert traj.duration() == pytest.approx(base + 0.6, abs=1e-6)


def test_zero_and_negative_wait_are_ignored(robot):
    """seconds<=0 不产生路点也不占时间（驻留零长度没有意义）。"""
    n_ref = len(ProgramBuilder(robot).moveJ(Q0, speed=40).build().plan().waypoints)
    traj = ProgramBuilder(robot).moveJ(Q0, speed=40).waitTime(0.0).waitTime(-1.0).build().plan()
    assert len(traj.waypoints) == n_ref


# --------------------------- 报告：驻留如实列出 --------------------------- #


def test_report_shows_dwell_total(robot):
    report = TrajectoryPlanner(robot).validate(
        ProgramBuilder(robot)
        .moveJ(Q0, speed=40)
        .waitTime(0.4)
        .moveJ(Q1, speed=40)
        .waitTime(0.2)
        .build()
    )
    assert report.dwell_total == pytest.approx(0.6, abs=1e-9)
    assert report.num_dwells == 2
    text = report.format()
    assert "0.60" in text and "waitTime" in text
    # cycle_time 必须含驻留（motion_times 之和 + 驻留 = 节拍）
    assert report.cycle_time == pytest.approx(sum(report.motion_times) + 0.6, abs=1e-6)


def test_report_without_dwell_has_no_line(robot):
    report = TrajectoryPlanner(robot).validate(ProgramBuilder(robot).moveJ(Q0, speed=40).build())
    assert report.dwell_total == 0.0 and report.num_dwells == 0
    assert "waitTime" not in report.format()


def test_dwell_excluded_from_motion_times(robot):
    """节拍明细仍按 Motion 口径（与示教点/夹爪事件对齐），驻留不混进明细。"""
    planner = TrajectoryPlanner(robot)
    report = planner.validate(
        ProgramBuilder(robot).moveJ(Q0, speed=40).waitTime(0.5).moveJ(Q1, speed=40).build()
    )
    assert len(report.motion_times) == 2  # 只有两条 Motion
    assert len(planner._motion_spans) == 2
    spans = planner._motion_spans
    # 驻留发生在两条运动之间的空隙里：span1 起点 = span0 终点 + 0.5s
    assert spans[1][0] - spans[0][1] == pytest.approx(0.5, abs=1e-9)


# --------------------------- 下游语义：夹爪事件不漂移 --------------------------- #


def test_gripper_events_not_shifted_by_dwell(robot):
    """夹爪事件时刻＝上一条运动的边界；驻留插在中间不挪动事件。

    顺序：moveJ → close → waitTime(0.7) → moveJ → open
    close 事件 = 第 0 条运动结束时刻（与无驻留版完全相同）；
    open 事件 = 第 1 条运动结束时刻（比无驻留版晚 0.7s——那是真实的）。
    """
    from domain.world.animation import gripper_events

    with_dwell = (
        ProgramBuilder(robot)
        .moveJ(Q0, speed=40)
        .gripper("close")
        .waitTime(0.7)
        .moveJ(Q1, speed=40)
        .gripper("open")
        .build()
    )
    without = (
        ProgramBuilder(robot)
        .moveJ(Q0, speed=40)
        .gripper("close")
        .moveJ(Q1, speed=40)
        .gripper("open")
        .build()
    )
    p1, p2 = TrajectoryPlanner(robot), TrajectoryPlanner(robot)
    t1, t2 = p1.plan(with_dwell), p2.plan(without)
    ev1 = gripper_events(with_dwell, p1)
    ev2 = gripper_events(without, p2)
    assert [e.action for e in ev1] == ["close", "open"] == [e.action for e in ev2]
    # close：都在第 0 条运动结束——时刻一致
    assert ev1[0].time == pytest.approx(ev2[0].time, abs=1e-9)
    # open：在第 1 条运动结束，驻留版晚 0.7s
    assert ev1[1].time == pytest.approx(ev2[1].time + 0.7, abs=1e-6)


# --------------------------- 多机：驻留占用必须可见 --------------------------- #


def test_dwell_occupies_timeline_for_multirobot(robot):
    """A 在交接位姿驻留、B 在驻留窗内到达同一点——必须看到机器人-机器人冲突。

    两台 UR5e（B 基座沿 +X 偏移 300mm）经 IK 去往**同一个世界点**：

    * 冲突组：B 的到达时刻落在 A 的驻留窗内 → 必有冲突（此前 WaitTime
      对规划器不可见，A 在时间轴上根本不会停在交接点，这类冲突不存在）；
    * 无冲突组：B 等到 A 驻留结束、退回远离交接点的零位之后才到 → 零冲突。
    """
    from domain.pose import Pose
    from service.multirobot import RobotJob, scan_timeline
    from domain.world.scene import Scene

    urdf = str(Path(__file__).resolve().parents[1] / "assets/models/ur/ur5e/ur5e.urdf")
    scene = Scene()
    scene.add_robot(urdf, list(Q0), name="A")
    scene.add_robot(
        urdf, list(Q0), name="B", pose=Pose.from_xyz_rpy(300.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    )
    robot_b = Robot(urdf, "UR5e")  # B 的程序在 B 自身基座系里规划

    # 交接点：A 的 TCP 世界位姿；B 用 IK 去同一个世界点
    Qh = [40.0, -60.0, 80.0, -90.0, 30.0, 0.0]
    p_world = robot.forward_kinematics(Qh)
    x, y, z, rx, ry, rz = p_world.to_xyz_rpy()
    p_b = Pose.from_xyz_rpy(x - 300.0, y, z, rx, ry, rz)
    qb = [float(v) for v in robot_b.inverse_kinematics(p_b, guess=Qh)]

    # A：到交接点 → 驻留 2.0s → 快速退回零位（远离交接区）
    prog_a = ProgramBuilder(robot).moveJ(Qh, speed=80).waitTime(2.0).moveJ(Q0, speed=160).build()
    planner_a = TrajectoryPlanner(robot)
    traj_a = planner_a.plan(prog_a)
    t_arrive = planner_a._motion_spans[0][1]
    t_leave = planner_a._motion_spans[1][0]
    assert t_leave - t_arrive == pytest.approx(2.0, abs=1e-6)  # 驻留窗

    traj_b = ProgramBuilder(robot_b).moveJ(qb, speed=60).build().plan()
    t_b = traj_b.duration()

    def _jobs(b_start: float):
        return [
            RobotJob(robot="A", trajectory=traj_a, start=0.0, label="A"),
            RobotJob(robot="B", trajectory=traj_b, start=b_start, label="B"),
        ]

    # 冲突组：B 恰好在驻留结束前 0.4s 到达同一世界点
    b_hit = max(0.0, t_leave - 0.4 - t_b)
    rr_hit, _env = scan_timeline(scene, _jobs(b_hit), dt=0.05, clearance=60.0)
    assert len(rr_hit) > 0
    assert any(i.kind == "robot-robot" for i in rr_hit)
    # 冲突时刻必须落在 A 的驻留窗（含 B 随后短暂同驻）内
    t_conflict = rr_hit[0].time
    assert t_arrive - 0.2 <= t_conflict <= t_leave + 0.5

    # 无冲突组：B 等到 A 退回零位、远离交接点之后才到
    b_safe = t_leave + t_b + 1.0
    rr_safe, _ = scan_timeline(scene, _jobs(b_safe), dt=0.05, clearance=60.0)
    assert len(rr_safe) == 0


# --------------------------- 出码与序列化不受影响 --------------------------- #


def test_waittime_still_reaches_post(robot):
    """驻留接进时间轴后，出码仍输出品牌等待语法（FANUC WAIT n(sec)）。"""
    code = (
        ProgramBuilder(robot)
        .moveJ(Q0, speed=40)
        .waitTime(0.5)
        .build()
        .to_post("fanuc_r30ib", metadata={"program_name": "DWELL"})
    )
    assert "WAIT 0.50(sec)" in code


def test_planner_accepts_programbuilder_and_program_alike(robot):
    """ProgramBuilder 直传与 build() 后的 Program 行为一致（duck typing）。"""
    b = ProgramBuilder(robot).moveJ(Q0, speed=40).waitTime(0.3)
    t_builder = TrajectoryPlanner(robot).plan(b)  # 直传 builder
    t_program = TrajectoryPlanner(robot).plan(b.build())
    assert t_builder.duration() == pytest.approx(t_program.duration(), abs=1e-9)
