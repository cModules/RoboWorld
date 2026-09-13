"""程序级抓取动画（S3）的测试：夹爪指令 → 工件吸附/释放。

守的东西：

* **事件映射正确**——``gripper()`` 指令落在轨迹时间轴的哪个时刻，
  必须恰好是「上一条运动结束」的边界；复位信号、连续重复动作不产生事件
* **规划器区间对齐**——某条指令规划失败（不可达等）也要在时间轴上占位，
  否则后续 IO 事件的时刻会整体错位
* **随动语义**——吸附瞬间工件世界位姿不变；机器人动，工件跟着走；
  释放后工件原地留下，不再随动
* **端到端 sync**——按播放头推进即可驱动整个抓取循环

运行::

    pytest tests/test_grasp_animation.py -q
"""

from __future__ import annotations

import sys
import warnings
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

from orolp.planner import TrajectoryPlanner  # noqa: E402
from orolp.pickplace import PointType, gripper_action_for  # noqa: E402
from orolp.program import Motion, MotionType, Program, SetDO  # noqa: E402
from orolp.robot import Robot  # noqa: E402
from orolp.world import Scene  # noqa: E402
from orolp.world.animation import GraspAnimator, GripperEvent, gripper_events  # noqa: E402
from orolp.world.scene import ROBOT_NAME  # noqa: E402

warnings.filterwarnings("ignore")

URDF = REPO_ROOT / "models" / "kuka" / "kr16_2" / "kr16_2.urdf"

WORKPIECE = "工件1"


@pytest.fixture
def robot() -> Robot:
    if not URDF.exists():
        pytest.skip(f"缺少测试用 URDF: {URDF}")
    return Robot(str(URDF), "kr16_2")


@pytest.fixture
def scene() -> Scene:
    s = Scene()
    s.set_robot(str(URDF))
    s.add("workpiece_box", name=WORKPIECE)
    return s


def _plan(robot: Robot, program: Program) -> TrajectoryPlanner:
    planner = TrajectoryPlanner(robot)
    planner.plan(program)
    return planner


# --------------------------------------------------------------------------- #
# 事件映射：gripper_events
# --------------------------------------------------------------------------- #


def test_events_between_motions(robot):
    """夹爪事件落在两条运动的边界时刻。"""
    prog = (
        robot.moveJ([0, -20, 30, 0, 40, 0])
        .gripper("close")
        .moveJ([10, -20, 30, 0, 40, 0])
        .gripper("open")
        .build()
    )
    planner = _plan(robot, prog)
    events = gripper_events(prog, planner)

    assert [e.action for e in events] == ["close", "open"]
    # close 在第 1 条运动结束时、第 2 条开始前；open 在第 2 条结束时
    assert events[0].time == pytest.approx(planner._motion_spans[0][1])
    assert events[1].time == pytest.approx(planner._motion_spans[1][1])
    assert 0.0 < events[0].time < events[1].time
    assert events[1].time <= planner.plan(prog).duration() + 1e-9


def test_event_before_first_motion_at_time_zero(robot):
    """没有任何运动前置位的夹紧事件落在轨迹起点 t=0。"""
    prog = Program(
        robot,
        [SetDO("gripper_close", True), Motion(MotionType.JOINTS, [0, 0, 0, 0, 0, 0])],
    )
    planner = _plan(robot, prog)
    events = gripper_events(prog, planner)
    assert len(events) == 1
    assert events[0].action == "close"
    assert events[0].time == 0.0


def test_reset_signal_is_not_an_event(robot):
    """互锁复位（gripper_close=False / gripper_open=False）不构成事件。"""
    prog = (
        robot.moveJ([0, 0, 0, 0, 0, 0])
        .gripper("close")
        .setDO("gripper_close", False)
        .build()
    )
    planner = _plan(robot, prog)
    assert [e.action for e in gripper_events(prog, planner)] == ["close"]


def test_consecutive_duplicate_actions_deduped(robot):
    """连续两次同向夹爪动作只保留首次——重复置位没有动画语义。"""
    prog = (
        robot.moveJ([0, 0, 0, 0, 0, 0])
        .gripper("close")
        .gripper("close", confirm=False)
        .build()
    )
    planner = _plan(robot, prog)
    assert [e.action for e in gripper_events(prog, planner)] == ["close"]


def test_program_without_io_has_no_events(robot):
    prog = robot.moveJ([0, 0, 0, 0, 0, 0]).moveJ([10, 0, 0, 0, 0, 0]).build()
    planner = _plan(robot, prog)
    assert gripper_events(prog, planner) == []


# --------------------------------------------------------------------------- #
# 规划器区间对齐：失败指令也要占位
# --------------------------------------------------------------------------- #


def test_failed_motion_keeps_span_alignment(robot):
    """不可达指令规划失败后，spans 仍与 motions 一一对齐。

    早期实现失败即跳过不占位，导致后续 IO 事件的时刻被整体前移——
    工件会在机器人还没走到抓取点时就被吸附。
    """
    prog = (
        robot.moveJ([0, 0, 0, 0, 0, 0])
        .gripper("close")
        .moveJ([9999, 0, 0, 0, 0, 0])  # 远超限位，规划必然失败
        .gripper("open")
        .moveJ([5, -10, 15, 0, 20, 0])
        .build()
    )
    planner = _plan(robot, prog)

    n_motions = len(prog.motions)
    assert len(planner._motion_spans) == n_motions
    # 失败指令占的是零长度区间（时刻不变），事件时刻不被它挤偏
    assert planner._motion_spans[1][0] == pytest.approx(planner._motion_spans[1][1])

    events = gripper_events(prog, planner)
    assert [e.action for e in events] == ["close", "open"]
    assert events[0].time == pytest.approx(planner._motion_spans[0][1])
    assert events[1].time == pytest.approx(planner._motion_spans[1][1])


# --------------------------------------------------------------------------- #
# 状态查询
# --------------------------------------------------------------------------- #


def test_state_at_timeline():
    anim = GraspAnimator(
        events=[
            GripperEvent(time=1.0, action="close"),
            GripperEvent(time=3.0, action="open"),
        ]
    )
    assert anim.state_at(0.5) is None
    assert anim.state_at(1.0) == "close"
    assert anim.state_at(2.9) == "close"
    assert anim.state_at(3.0) == "open"
    assert anim.state_at(100.0) == "open"


def test_bad_action_rejected(scene):
    with pytest.raises(ValueError):
        GraspAnimator.apply(scene, WORKPIECE, "toggle")


def test_gripper_action_for_point_type():
    assert gripper_action_for(PointType.PICK) == "close"
    assert gripper_action_for(PointType.PLACE) == "open"
    for t in (PointType.APPROACH, PointType.LIFT, PointType.VIA, None):
        assert gripper_action_for(t) is None


# --------------------------------------------------------------------------- #
# 场景随动语义
# --------------------------------------------------------------------------- #


def test_attach_keeps_world_then_follows_robot(scene):
    """吸附瞬间工件停在原地；机器人动，工件随 TCP 走。"""
    wp_before = scene.world_transform(WORKPIECE).copy()

    GraspAnimator.apply(scene, WORKPIECE, "close")
    assert scene.objects[WORKPIECE].parent == ROBOT_NAME
    # 被夹住的瞬间：世界位姿不变
    np.testing.assert_allclose(
        scene.world_transform(WORKPIECE), wp_before, atol=1e-6
    )

    # 机器人换位形 → 工件世界位姿跟着变（随动）
    scene.set_robot_joints([0, -45, 45, 0, 45, 0])
    assert not np.allclose(scene.world_transform(WORKPIECE), wp_before, atol=1e-3)


def test_detach_leaves_workpiece_in_place(scene):
    """释放后工件原地留下，机器人再动它也不跟。"""
    GraspAnimator.apply(scene, WORKPIECE, "close")
    scene.set_robot_joints([0, -45, 45, 0, 45, 0])
    GraspAnimator.apply(scene, WORKPIECE, "open")
    assert scene.objects[WORKPIECE].parent is None

    p_released = scene.world_transform(WORKPIECE).copy()
    scene.set_robot_joints([10, -30, 30, 0, 30, 0])
    np.testing.assert_allclose(
        scene.world_transform(WORKPIECE), p_released, atol=1e-9
    )


def test_attach_to_missing_workpiece_raises(scene):
    with pytest.raises(KeyError):
        GraspAnimator.apply(scene, "不存在的工件", "close")


# --------------------------------------------------------------------------- #
# 端到端：sync 按播放头驱动完整抓取循环
# --------------------------------------------------------------------------- #


def test_sync_drives_full_pick_place_cycle(robot, scene):
    prog = (
        robot.moveJ([0, -20, 30, 0, 40, 0])
        .gripper("close")
        .moveJ([10, -20, 30, 0, 40, 0])
        .gripper("open")
        .build()
    )
    planner = _plan(robot, prog)
    traj = planner.plan(prog)
    anim = GraspAnimator(prog, planner)
    assert len(anim.events) == 2

    t_close, t_open = anim.events[0].time, anim.events[1].time
    assert t_close < t_open <= traj.duration()

    # 起点：未夹持
    assert anim.sync(scene, WORKPIECE, 0.0) is None
    assert scene.objects[WORKPIECE].parent is None

    # 越过抓取事件 → 吸附；同一时刻重复 sync 不重复动作
    applied = anim.sync(scene, WORKPIECE, t_close)
    assert applied == "close"
    assert scene.objects[WORKPIECE].parent == ROBOT_NAME

    # 夹持状态下机器人走到 place 位姿，工件随动
    wp_at_close = scene.world_transform(WORKPIECE).copy()
    scene.set_robot_joints([10, -20, 30, 0, 40, 0])
    assert not np.allclose(scene.world_transform(WORKPIECE), wp_at_close, atol=1e-3)

    # 越过释放事件 → 原地留下
    applied = anim.sync(scene, WORKPIECE, traj.duration() + 1.0)
    assert applied == "open"
    assert scene.objects[WORKPIECE].parent is None
    assert anim.done


def test_reset_allows_replay(robot, scene):
    """回放重来：reset 后事件可再次驱动（上一次释放的工件重新被抓起）。"""
    prog = (
        robot.moveJ([0, 0, 0, 0, 0, 0])
        .gripper("close")
        .moveJ([10, -20, 30, 0, 40, 0])
        .gripper("open")
        .build()
    )
    planner = _plan(robot, prog)
    anim = GraspAnimator(prog, planner)
    t_close = anim.events[0].time

    anim.sync(scene, WORKPIECE, t_close)
    anim.sync(scene, WORKPIECE, anim.events[1].time)
    assert scene.objects[WORKPIECE].parent is None

    anim.reset()
    assert not anim.done
    anim.sync(scene, WORKPIECE, t_close)
    assert scene.objects[WORKPIECE].parent == ROBOT_NAME
