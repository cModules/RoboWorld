"""在线调试会话的回归测试。

守住：SimLink 下播放/单步/跳转/示教都能跑，且实际位姿与规划位姿偏差近零；
FileLink 下正确标注"无回读"但仍可播放。
"""

import sys
import tempfile
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

from service.program import ProgramBuilder  # noqa: E402
from domain.pose import Pose
from domain.robot import Robot  # noqa: E402
from service.debug import DebugSession, Deviation  # noqa: E402
from domain.link import FileLink, SimLink  # noqa: E402

URDF = Path(__file__).resolve().parent.parent / "assets/models/ur/ur5e/ur5e.urdf"
Q0 = [0.0, -30.0, 30.0, 0.0, 60.0, 0.0]


@pytest.fixture
def robot():
    return Robot(str(URDF), "UR5")


@pytest.fixture
def traj(robot):
    prog = (
        ProgramBuilder(robot)
        .moveJ(Q0, speed=40)
        .moveL(Pose.from_xyz_rpy(500, 0, 300, 0, 0, 0), speed=50, blend=8)
        .moveL(Pose.from_xyz_rpy(500, 200, 300, 0, 0, 0), speed=50, blend=8)
        .build()
    )
    return prog.simulate()


def test_sim_session_feedback_available(traj, robot):
    sl = SimLink(robot)
    sl.connect()
    sess = DebugSession(traj, sl, robot=robot)
    assert sess.feedback_available is True


def test_single_step_advances_and_reads_back(traj, robot):
    sl = SimLink(robot)
    sl.connect()
    sess = DebugSession(traj, sl, robot=robot)
    wp = sess.step()
    assert wp is not None
    assert sess.current_index == 1
    actual = sess.read_actual()
    assert actual is not None
    # 实际关节角 == 该路点规划关节角
    assert np.allclose(actual.joints, wp.joints, atol=1e-9)


def test_run_deviation_near_zero(traj, robot):
    sl = SimLink(robot)
    sl.connect()
    sess = DebugSession(traj, sl, robot=robot)
    summary = sess.run()
    assert summary["executed"] == len(traj.waypoints) - 1
    # 完美仿真下，实际位姿与规划位姿偏差应在数值容差内
    assert summary["max_pos_deviation_mm"] < 0.1
    assert summary["max_rot_deviation_deg"] < 0.1
    assert sess.current_index == len(traj.waypoints) - 1


def test_goto_time_and_jog(traj, robot):
    sl = SimLink(robot)
    sl.connect()
    sess = DebugSession(traj, sl, robot=robot)
    sess.goto_time(traj.duration() / 2)
    before = sl.read_joints().copy()
    jogged = sess.jog(1, 5.0)
    assert jogged is not None
    assert abs(jogged[1] - (before[1] + 5.0)) < 1e-9


def test_file_session_no_feedback(traj):
    fl = FileLink(tempfile.mkdtemp())
    fl.connect()
    sess = DebugSession(traj, fl)
    assert sess.feedback_available is False
    dev = sess.deviation()
    assert isinstance(dev, Deviation)
    assert dev.available is False
    # 仍能播放（执行计数推进）
    sess.run()
    assert sess.summary()["executed"] == len(traj.waypoints) - 1


def test_reset_returns_to_start(traj, robot):
    sl = SimLink(robot)
    sl.connect()
    sess = DebugSession(traj, sl, robot=robot)
    sess.run()
    assert sess.current_index > 0
    sess.reset()
    assert sess.current_index == 0
    assert sess.summary()["max_pos_deviation_mm"] == 0.0
