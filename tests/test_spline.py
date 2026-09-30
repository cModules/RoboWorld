"""SPLINE 插补的回归测试。

守住规划层补齐的运动类型：关节目标 / 位姿目标都可达、不越限、节拍合理，
且具备 moveJ 没有的"端点平滑"（加减速连续）。
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

from service.program import ProgramBuilder  # noqa: E402
from domain.pose import Pose
from domain.robot import Robot  # noqa: E402
from service.program import MotionType  # noqa: E402

URDF = Path(__file__).resolve().parent.parent / "assets/models/ur/ur5e/ur5e.urdf"
Q0 = [0.0, -30.0, 30.0, 0.0, 60.0, 0.0]
QT = [10.0, -60.0, 40.0, 0.0, 90.0, 0.0]


@pytest.fixture
def robot():
    return Robot(str(URDF), "UR5")


def _max_step(q):
    return float(np.max(np.abs(np.diff(q, axis=0))))


def test_spline_joints_reaches_target(robot):
    prog = ProgramBuilder(robot).moveJ(Q0, speed=40).moveSpline(QT, speed=50).build()
    traj = prog.simulate()
    q = traj.get_joint_trajectory()
    # 末端点（去掉起点）必须等于目标关节角
    assert np.allclose(q[-1], QT, atol=1e-6)
    rep = prog.validate()
    assert rep.ok, rep.format()


def test_spline_pose_ik_target(robot):
    target = Pose.from_xyz_rpy(500, 0, 300, 0, 0, 0)
    prog = ProgramBuilder(robot).moveJ(Q0, speed=40).moveSpline(target, speed=50).build()
    traj = prog.simulate()
    rep = prog.validate()
    assert rep.ok, rep.format()
    # 末端 TCP 应落在目标附近
    end = traj.waypoints[-1].pose.to_xyz_rpy()
    assert np.linalg.norm(np.array(end[:3]) - np.array([500, 0, 300])) < 5.0


def test_spline_cycle_time_positive(robot):
    p1 = ProgramBuilder(robot).moveJ(Q0, speed=40).moveSpline(QT, speed=50).build()
    t1 = p1.simulate().duration()
    assert t1 > 0 and math.isfinite(t1)
    # 速度越低耗时越长
    p2 = ProgramBuilder(robot).moveJ(Q0, speed=40).moveSpline(QT, speed=10).build()
    assert p2.simulate().duration() > t1


def test_spline_endpoint_smoothness(robot):
    """SPLINE 与 moveJ 的本质区别：端点加减速连续（首/末步增量趋近 0）。

    把机器人初始关节设到 Q0，使整条轨迹就是「从 Q0 到 QT 的这一个运动」，
    从而把样条段从拼接轨迹里隔离出来测量。
    """
    robot.set_joints(np.array(Q0, dtype=float))
    sp = ProgramBuilder(robot).moveSpline(QT, speed=50).build()
    qs = sp.simulate().get_joint_trajectory()
    incr_s = np.abs(np.diff(qs, axis=0))
    max_s = float(np.max(incr_s))

    robot.set_joints(np.array(Q0, dtype=float))
    jn = ProgramBuilder(robot).moveJ(QT, speed=50).build()
    qj = jn.simulate().get_joint_trajectory()
    incr_j = np.abs(np.diff(qj, axis=0))
    max_j = float(np.max(incr_j))

    # 线性 moveJ：v0.2.2 起按梯形速度曲线推进，端点增量同样趋近 0，
    # 平台期（匀速段）增量才接近最大值
    assert incr_j[0].max() < 0.3 * max_j
    assert incr_j[-1].max() < 0.3 * max_j
    assert incr_j[len(incr_j) // 2].max() > 0.7 * max_j
    # 最小抖动 SPLINE：首/末步增量远小于最大增量（端点平滑）
    assert incr_s[0].max() < 0.3 * max_s
    assert incr_s[-1].max() < 0.3 * max_s


def test_spline_no_limit_violation_reported(robot):
    # 一个会越限的目标应被规划器报错，而不是静默夹回
    bad = [0, 0, 0, 0, 0, 9999]  # 末轴远超限位
    prog = ProgramBuilder(robot).moveJ(Q0, speed=40).moveSpline(bad, speed=50).build()
    rep = prog.validate()
    assert not rep.ok
    assert any("越限" in i.message or "限位" in i.message for i in rep.errors)


def test_spline_in_full_program_to_post_all_brands(robot):
    prog = (
        ProgramBuilder(robot)
        .moveJ(Q0, speed=40)
        .moveL(Pose.from_xyz_rpy(500, 0, 300, 0, 0, 0), speed=50)
        .moveSpline([10, -60, 40, 0, 90, 0], speed=50)
        .moveSpline(Pose.from_xyz_rpy(500, 200, 300, 0, 0, 0), speed=50)
        .build()
    )
    from adapter.post import list_post_processors

    for pp in list_post_processors():
        code = prog.to_post(pp)
        assert code.strip(), f"{pp} 未产出代码"
        # 降级为 PTP/LIN，不应再出现未支持的运动类型报错
