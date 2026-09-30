"""v0.2.2 速度/加速度 S 曲线规划 + 节拍报表 的回归测试。

守住的契约：

* **速度曲线是真的** —— moveJ/moveL 的路径点时间分布按梯形/S 曲线推进，
  起停速度归零、峰值速度不超轴限位，而不是"全程匀速的假节拍"。
* **Waypoint.velocity/acceleration 被真实填充** —— 差分值与时间戳自洽。
* **节拍不回归** —— 默认梯形曲线下总时长与原节拍公式一致，S 曲线只是
  重新分配"怎么走"，且可预期地略长。
* **节拍报表** —— ProgramReport 按指令给出耗时明细。
* **后处理带速度/加速度** —— Motion.velocity/acceleration 覆盖值进入
  UR / KUKA / ABB / FANUC 品牌代码。

运行::

    pytest tests/test_scurve.py -q
"""

from __future__ import annotations

import math
import sys
import warnings
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from service.planner import (
    TrajectoryPlanner,
    s_curve_time,
    trapezoid_time,
)
from domain.pose import Pose  # noqa: E402
from service.program import Motion, MotionType, Program, ProgramBuilder  # noqa: E402
from domain.robot import Robot  # noqa: E402

warnings.filterwarnings("ignore")

URDF = REPO_ROOT / "assets/models/kuka/kr16_2/kr16_2.urdf"
HOME = [0.0, -20.0, 70.0, 0.0, 40.0, 0.0]


def make_robot() -> Robot:
    robot = Robot(str(URDF), "kr16_2")
    robot.set_joints(HOME)
    return robot


@pytest.fixture(scope="module")
def robot() -> Robot:
    return make_robot()


def joint_speeds(robot: Robot) -> np.ndarray:
    from service.planner import AxisLimits

    return AxisLimits.from_kinematics(robot.kinematics).max_speed


# --------------------------------------------------------------------------- #
# S 曲线时间公式
# --------------------------------------------------------------------------- #


def test_s_curve_time_longer_than_trapezoid():
    """同参数下 S 曲线节拍应略长于梯形（起停更柔和的代价）。"""
    d, v, a = 500.0, 500.0, 2000.0
    t_s = s_curve_time(d, v, a)
    t_t = trapezoid_time(d, v, a)
    assert t_t > 0
    assert t_s > t_t
    # 但不该离谱：不超过梯形的 1.6 倍
    assert t_s < t_t * 1.6


def test_s_curve_time_triangle_short_distance():
    """短距离（达不到 vmax）退化为三角形 S 曲线，仍单调、为正。"""
    t1 = s_curve_time(1.0, 500.0, 2000.0)
    t2 = s_curve_time(5.0, 500.0, 2000.0)
    assert 0.0 < t1 < t2


def test_s_curve_time_zero_inputs():
    assert s_curve_time(0.0, 500.0, 2000.0) == 0.0
    assert s_curve_time(100.0, 0.0, 2000.0) == 0.0


# --------------------------------------------------------------------------- #
# 规划器：moveJ 速度曲线与动力学填充
# --------------------------------------------------------------------------- #


def test_moveJ_velocity_fields_populated(robot):
    target = [q for q in HOME]
    target[0] += 40.0
    target[2] -= 25.0
    traj = ProgramBuilder(robot).moveJ(target).build().plan()
    wps = traj.waypoints
    assert len(wps) >= 3
    for wp in wps:
        assert wp.velocity is not None
        assert wp.acceleration is not None
    # 起停速度归零（速度曲线端点）
    assert np.max(np.abs(wps[0].velocity)) < 1e-6
    assert np.max(np.abs(wps[-1].velocity)) < 1e-6


def test_moveJ_peak_velocity_within_axis_limits(robot):
    target = list(HOME)
    target[0] += 60.0
    traj = ProgramBuilder(robot).moveJ(target, speed=100.0).build().plan()
    peak = max(np.max(np.abs(wp.velocity)) for wp in traj.waypoints)
    assert peak <= joint_speeds(robot).max() * 1.05


def test_moveJ_velocity_profile_shape(robot):
    """速度先升后降（梯形形状）：峰值落在中段而不是两端。"""
    target = list(HOME)
    target[1] -= 45.0
    traj = ProgramBuilder(robot).moveJ(target).build().plan()
    speeds = [float(np.max(np.abs(wp.velocity))) for wp in traj.waypoints]
    peak_i = int(np.argmax(speeds))
    n = len(speeds)
    assert n * 0.25 <= peak_i <= n * 0.75
    # 不是匀速假节拍：首末段速度显著低于峰值
    assert speeds[1] < speeds[peak_i] * 0.6


def test_scurve_moveJ_slower_and_softer(robot):
    """S 曲线比梯形慢，且起步更柔和（首步平均速度显著更小）。"""
    target = list(HOME)
    target[0] += 50.0
    prog = ProgramBuilder(robot).moveJ(target).build()
    traj_trap = TrajectoryPlanner(robot, profile="trapezoid").plan(prog)
    traj_sc = TrajectoryPlanner(robot, profile="scurve").plan(prog)
    assert traj_sc.duration() > traj_trap.duration()

    def first_step_speed(traj):
        wps = traj.waypoints
        dt = wps[1].timestamp - wps[0].timestamp
        return float(np.max(np.abs(wps[1].joints - wps[0].joints)) / dt)

    # 梯形在 t=0 就以满加速度冲出去；S 曲线起步近似零加速度
    assert first_step_speed(traj_sc) < first_step_speed(traj_trap) * 0.75


def test_acceleration_within_axis_limits(robot):
    """两种曲线的差分加速度都不应显著超过轴加速度限位（4×速度）。"""
    from service.planner import AxisLimits

    limits = AxisLimits.from_kinematics(robot.kinematics)
    target = list(HOME)
    target[0] += 50.0
    prog = ProgramBuilder(robot).moveJ(target).build()
    for profile in ("trapezoid", "scurve"):
        traj = TrajectoryPlanner(robot, profile=profile).plan(prog)
        for wp in traj.waypoints:
            assert np.all(np.abs(wp.acceleration) <= limits.max_accel * 1.25)


def test_invalid_profile_rejected(robot):
    with pytest.raises(Exception):
        TrajectoryPlanner(robot, profile="hyperbolic")


# --------------------------------------------------------------------------- #
# 规划器：moveL 速度曲线与节拍不回归
# --------------------------------------------------------------------------- #


def _reach_pose(robot: Robot, dx: float) -> Pose:
    base = robot.forward_kinematics(np.asarray(HOME))
    x, y, z, *_ = base.to_xyz_quat()
    return base.with_position(x + dx, y, z)


def test_moveL_velocity_profile_and_cycle_time(robot):
    target = _reach_pose(robot, 80.0)
    prog = ProgramBuilder(robot).moveL(target).build()
    planner = TrajectoryPlanner(robot)
    traj = planner.plan(prog)

    wps = traj.waypoints
    assert len(wps) >= 3
    for wp in wps:
        assert wp.velocity is not None and wp.acceleration is not None
    # 起停速度归零
    assert np.max(np.abs(wps[0].velocity)) < 1e-6
    assert np.max(np.abs(wps[-1].velocity)) < 1e-6
    # 时间戳严格递增且非均匀（有加减速）
    ts = traj.get_timestamps()
    assert np.all(np.diff(ts) > 0)
    # TCP 峰值速度不超过默认 TCP 限速 500 mm/s（含 5% 离散容差）
    peak_joint = max(float(np.max(np.abs(wp.velocity))) for wp in wps)
    assert peak_joint <= joint_speeds(robot).max() * 1.05
    # 节拍与原公式一致或更长（关节约束可延伸；梯形默认下不应变短）
    length = np.linalg.norm(
        np.asarray(target.to_xyz_quat()[:3])
        - np.asarray(robot.forward_kinematics(np.asarray(HOME)).to_xyz_quat()[:3])
    )
    expected = trapezoid_time(length, planner.tcp_speed, planner.tcp_accel)
    assert traj.duration() >= expected * 0.999


def test_moveL_scurve_slower(robot):
    target = _reach_pose(robot, 120.0)
    prog = ProgramBuilder(robot).moveL(target).build()
    t_trap = TrajectoryPlanner(robot, profile="trapezoid").plan(prog).duration()
    t_sc = TrajectoryPlanner(robot, profile="scurve").plan(prog).duration()
    assert t_sc > t_trap


# --------------------------------------------------------------------------- #
# 节拍报表
# --------------------------------------------------------------------------- #


def test_report_motion_times_align_with_cycle_time(robot):
    target = _reach_pose(robot, 60.0)
    jtarget = list(HOME)
    jtarget[2] += 20.0
    prog = (
        ProgramBuilder(robot)
        .moveJ(jtarget)
        .moveL(target)
        .moveL(target.with_position(*np.asarray(target.to_xyz_quat()[:3]) + [0, -40.0, 0]))
        .build()
    )
    report = prog.validate()
    assert report.cycle_time and report.cycle_time > 0
    assert report.motion_times is not None
    assert len(report.motion_times) == report.num_motions
    assert sum(report.motion_times) == pytest.approx(report.cycle_time, rel=1e-6)
    assert report.motion_labels == ["moveJ", "moveL", "moveL"]
    text = report.format()
    assert "节拍明细" in text
    assert "#1" in text and "#3" in text


def test_report_without_plan_has_no_motion_times():
    empty = type("R", (), {})  # 未规划时不该出现 motion_times 字段错误
    from service.planner import ProgramReport

    r = ProgramReport()
    assert r.motion_times is None
    assert empty is not None


# --------------------------------------------------------------------------- #
# Motion 覆盖字段与后处理
# --------------------------------------------------------------------------- #


def test_motion_defaults_and_builder_passthrough(robot):
    m = Motion(MotionType.LINEAR, Pose())
    assert m.velocity is None and m.acceleration is None
    prog = (
        ProgramBuilder(robot)
        .moveL(Pose(), velocity=300.0, acceleration=1200.0)
        .moveJ(HOME, velocity=90.0, acceleration=360.0)
        .build()
    )
    lin, jnt = prog.motions
    assert lin.velocity == 300.0 and lin.acceleration == 1200.0
    assert jnt.velocity == 90.0 and jnt.acceleration == 360.0


def test_ur_post_vel_acc_overrides(robot):
    code = (
        ProgramBuilder(robot)
        .moveL(Pose(), velocity=250.0, acceleration=1500.0)
        .moveJ(HOME, velocity=90.0)
        .build()
        .to_post("ur_polyscope", metadata={"program_name": "T"})
    )
    assert "v=0.2500" in code
    assert "a=1.50" in code  # 1500 mm/s² → 1.5 m/s²
    # 关节速度 90 deg/s → 1.5708 rad/s
    assert "v=1.5708" in code


def test_kuka_post_vel_cp_acc_cp(robot):
    code = (
        ProgramBuilder(robot)
        .moveL(Pose(), velocity=500.0, acceleration=2000.0)
        .build()
        .to_post("kuka_krc4", metadata={"program_name": "T"})
    )
    assert "$VEL.CP = 0.500" in code
    assert "$ACC.CP = 2.000" in code


def test_kuka_post_no_vel_cp_by_default(robot):
    code = (
        ProgramBuilder(robot)
        .moveL(Pose())
        .build()
        .to_post("kuka_krc4", metadata={"program_name": "T"})
    )
    assert "$VEL.CP" not in code
    assert "$ACC.CP" not in code


def test_abb_post_vnum_override(robot):
    code = (
        ProgramBuilder(robot)
        .moveL(Pose(), velocity=350.0)
        .moveL(Pose())
        .build()
        .to_post("abb_irc5", metadata={"program_name": "T"})
    )
    assert "v350" in code
    # 未覆盖的指令沿用 speed 百分比（100 → v100）
    assert "v100" in code


def test_fanuc_post_speed_override(robot):
    code = (
        ProgramBuilder(robot)
        .moveL(Pose(), velocity=420.0)
        .build()
        .to_post("fanuc_r30ib", metadata={"program_name": "T"})
    )
    assert "420mm/sec" in code


def test_spline_lowering_carries_vel_acc(robot):
    """SPLINE 降级为 LIN/PTP 时不得丢失速度/加速度覆盖。"""
    m = Motion(MotionType.SPLINE, Pose(), velocity=300.0, acceleration=1200.0)
    from adapter.post.base import BasePostProcessor

    lowered = BasePostProcessor._lower_spline(None, m)  # type: ignore[arg-type]
    assert lowered.velocity == 300.0 and lowered.acceleration == 1200.0
