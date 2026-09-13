"""抓取工艺（pick & place）的回归测试。

守的是「示教点 → 机器人程序 + 仿真动画」这条链路的一致性：

* IO 必须插在**正确的点位之后**（顺序错了就是撞机或掉件）
* 夹紧与松开必须**互锁**，不能同时为真
* 吸附事件索引必须指向真正的抓取点/放置点

运行::

    pytest tests/test_pickplace.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

from orolp.pickplace import (  # noqa: E402
    DEFAULT_SEQUENCE,
    PickPlacePlan,
    PointType,
    build_pick_place,
    default_types_for_count,
)
from orolp.program import Motion, Program, SetDO, WaitDI, WaitTime  # noqa: E402
from orolp.robot import Robot  # noqa: E402

URDF = REPO_ROOT / "models" / "kuka" / "kr16_2" / "kr16_2.urdf"

#: 五个示教点（度），纯数值无含义，只用于跑通流程
POINTS = [
    [0.0, -20.0, 60.0, 0.0, 30.0, 0.0],
    [0.0, -10.0, 70.0, 0.0, 30.0, 0.0],
    [0.0, -30.0, 50.0, 0.0, 30.0, 0.0],
    [45.0, -10.0, 70.0, 0.0, 30.0, 0.0],
    [45.0, -30.0, 50.0, 0.0, 30.0, 0.0],
]


@pytest.fixture
def robot() -> Robot:
    if not URDF.exists():
        pytest.skip(f"缺少测试用 URDF: {URDF}")
    return Robot(str(URDF), "kr16_2")


# --------------------------------------------------------------------------- #
# 默认语义推导
# --------------------------------------------------------------------------- #


def test_default_types_five_points():
    assert default_types_for_count(5) == list(DEFAULT_SEQUENCE)


def test_default_types_less_than_five():
    assert default_types_for_count(3) == [
        PointType.APPROACH, PointType.PICK, PointType.LIFT,
    ]


def test_default_types_more_than_five_uses_via():
    """多出来的点宁可标成途经点，也不瞎猜哪个是抓取点。"""
    types = default_types_for_count(7)
    assert len(types) == 7
    assert types[0] is PointType.APPROACH
    assert types[-1] is PointType.RETRACT
    assert all(t is PointType.VIA for t in types[1:-1])


def test_default_types_zero():
    assert default_types_for_count(0) == []


# --------------------------------------------------------------------------- #
# 程序生成
# --------------------------------------------------------------------------- #


def test_plan_structure():
    plan = build_pick_place(POINTS)
    assert isinstance(plan, PickPlacePlan)

    # 5 个运动 + PICK(4 条 IO) + PLACE(3 条 IO) = 12
    n_motion = sum(1 for i in plan.items if isinstance(i, Motion))
    assert n_motion == 5
    assert len(plan.items) == 12


def test_events_point_at_correct_indices():
    """抓取点在索引 1、放置点在索引 3（五点式循环）。"""
    plan = build_pick_place(POINTS)
    assert plan.pick_indices == [1]
    assert plan.place_indices == [3]
    assert plan.events == [(1, "pick"), (3, "place")]


def test_pick_io_order_and_interlock():
    """夹紧前必须先复位松开信号——两个输出不能同时为真。"""
    plan = build_pick_place(POINTS)
    setdos = [i for i in plan.items if isinstance(i, SetDO)]

    # 顺序：松开复位 → 夹紧 → 松开 → 夹紧复位
    assert setdos[0] == SetDO("gripper_open", False)
    assert setdos[1] == SetDO("gripper_close", True)
    assert setdos[2] == SetDO("gripper_close", False)
    assert setdos[3] == SetDO("gripper_open", True)

    # 任意时刻两者不得同时为真
    close = open_ = False
    for i in plan.items:
        if isinstance(i, SetDO):
            if i.signal == "gripper_close":
                close = i.value
            elif i.signal == "gripper_open":
                open_ = i.value
        assert not (close and open_), "夹紧与松开同时为真——互锁失效"


def test_pick_waits_for_clamped_feedback():
    plan = build_pick_place(POINTS)
    waits = [i for i in plan.items if isinstance(i, WaitDI)]
    assert waits and waits[0].signal == "clamped"
    assert waits[0].value is True


def test_dwell_present_twice():
    plan = build_pick_place(POINTS)
    times = [i for i in plan.items if isinstance(i, WaitTime)]
    assert len(times) == 2


def test_items_feed_program_and_plan(robot):
    """items 能直接喂给 Program，且轨迹规划只看运动部分。"""
    plan = build_pick_place(POINTS)
    prog = Program(robot, plan.items)
    assert len(prog.motions) == 5
    assert prog.plan() is not None


def test_explicit_types_respected():
    """显式指定的语义优先于默认推导。"""
    types = [PointType.VIA, PointType.PICK, PointType.VIA,
             PointType.PLACE, PointType.VIA]
    plan = build_pick_place(POINTS, types)
    assert plan.pick_indices == [1]
    assert plan.place_indices == [3]


def test_no_pick_or_place_yields_no_events():
    plan = build_pick_place(POINTS, [PointType.VIA] * 5)
    assert plan.events == []
    assert len(plan.items) == 5  # 只有运动


# --------------------------------------------------------------------------- #
# 参数校验
# --------------------------------------------------------------------------- #


def test_empty_points_rejected():
    with pytest.raises(ValueError, match="示教点"):
        build_pick_place([])


def test_mismatched_types_rejected():
    with pytest.raises(ValueError, match="不一致"):
        build_pick_place(POINTS, [PointType.VIA] * 3)


# --------------------------------------------------------------------------- #
# 端到端：生成的代码里真的有抓取动作
# --------------------------------------------------------------------------- #


def test_generated_kuka_code_has_gripper_actions(robot):
    plan = build_pick_place(POINTS)
    code = Program(robot, plan.items).to_post(
        "kuka_krc4",
        metadata={
            "program_name": "PICK",
            "io_map": {
                "gripper_close": "Q0.1",
                "gripper_open": "Q0.0",
                "clamped": "I0.3",
            },
        },
    )
    assert "$OUT[1] = TRUE" in code        # 夹紧
    assert "$OUT[0] = TRUE" in code        # 松开
    assert "WAIT FOR $IN[3]" in code       # 等夹紧反馈
    # 夹紧指令必须出现在第二个示教点之后
    assert code.index("PTP") < code.index("$OUT[1] = TRUE")
