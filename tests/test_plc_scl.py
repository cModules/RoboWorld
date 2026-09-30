"""西门子 SCL 生成器的回归测试。

重点是 **PLC 步序必须由机器人程序的真实 IO 时序派生**：

* 写死的默认骨架（``default_steps``）与实际示教动作无关，联调时必然对不上
* 保压延时要有对应的定时器，不能假装存在
* 自定义信号必须被声明，否则 TIA 编译不过

运行::

    pytest tests/test_plc_scl.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from service.cell import CellConfig, Device  # noqa: E402
from service.pickplace import build_pick_place, default_types_for_count  # noqa: E402
from service.plc_scl import (
    Step,
    _format_pt,
    default_steps,
    generate_scl,
    steps_from_program,
)
from service.program import Motion, MotionType, SetDO, WaitDI, WaitTime  # noqa: E402

POINTS = [
    [0, -20, 60, 0, 30, 0],
    [0, -10, 70, 0, 30, 0],
    [0, -30, 50, 0, 30, 0],
    [45, -10, 70, 0, 30, 0],
    [45, -30, 50, 0, 30, 0],
]


@pytest.fixture
def cell() -> CellConfig:
    c = CellConfig()
    c.add(
        Device(
            name="夹具",
            type="夹具",
            io_map={
                "gripper_open": "Q0.1",
                "gripper_close": "Q0.2",
                "clamped": "I0.3",
            },
        )
    )
    return c


# --------------------------------------------------------------------------- #
# 步序派生
# --------------------------------------------------------------------------- #


def test_steps_derived_from_real_io():
    plan = build_pick_place(POINTS, default_types_for_count(5))
    steps = steps_from_program(plan.items)

    names = [s.name for s in steps]
    # 起始步 + 等待夹紧 + 夹紧保压 + 松开保压
    assert names[0] == "等待工件到位"
    assert any("clamped" in n for n in names)
    assert sum(1 for s in steps if s.delay > 0) == 2


def test_clamp_step_carries_outputs():
    plan = build_pick_place(POINTS, default_types_for_count(5))
    steps = steps_from_program(plan.items)

    clamp = next(s for s in steps if s.wait == "clamped")
    assert clamp.outputs["gripper_close"] is True
    assert clamp.outputs["gripper_open"] is False


def test_release_step_opens_gripper():
    plan = build_pick_place(POINTS, default_types_for_count(5))
    steps = steps_from_program(plan.items)

    release = [s for s in steps if s.outputs.get("gripper_open") is True]
    assert release, "没有生成松开步"


def test_no_io_yields_empty_and_falls_back(cell):
    """程序里没有 IO 时不能硬凑工步——应回退默认骨架。"""
    steps = steps_from_program([Motion(MotionType.JOINTS, [0] * 6)])
    assert steps == []

    code = generate_scl(cell, steps=steps)
    assert "夹紧工件" in code  # 默认骨架里的步


def test_custom_signal_is_declared(cell):
    """只按默认集合声明的话，自定义信号会"用了没声明"，TIA 编译不过。"""
    steps = [Step("自定义", outputs={"conveyor_run": True}, wait="box_in_place")]
    code = generate_scl(cell, steps=steps)
    assert "conveyor_run" in code.split("BEGIN")[0], "输出未声明"
    assert "box_in_place" in code.split("BEGIN")[0], "输入未声明"


# --------------------------------------------------------------------------- #
# 保压定时器
# --------------------------------------------------------------------------- #


def test_delay_emits_ton_timer(cell):
    steps = [Step("保压", outputs={}, delay=0.4)]
    code = generate_scl(cell, steps=steps)
    assert "dwellTimer : TON" in code
    assert "T#400MS" in code


def test_no_delay_declares_no_timer(cell):
    steps = [Step("直接过", wait="clamped")]
    code = generate_scl(cell, steps=steps)
    assert "TON" not in code


def test_estop_resets_timer(cell):
    """急停时定时器必须一并复位，否则恢复后会立刻误判为"已保压完成"。"""
    steps = [Step("保压", delay=0.4)]
    code = generate_scl(cell, steps=steps)
    head, body = code.split("BEGIN")
    assert "dwellTimer(IN := FALSE)" in body.split("CASE")[0]


@pytest.mark.parametrize(
    "sec,expected",
    [(0.4, "T#400MS"), (1.5, "T#1S500MS"), (2.0, "T#2S"), (0.0, "T#0MS")],
)
def test_format_pt(sec, expected):
    assert _format_pt(sec) == expected


# --------------------------------------------------------------------------- #
# 安全联锁（既有行为不能被改坏）
# --------------------------------------------------------------------------- #


def test_safety_interlock_present(cell):
    code = generate_scl(
        cell, steps=steps_from_program(build_pick_place(POINTS, default_types_for_count(5)).items)
    )
    assert "IF NOT #safety_ok OR #estop THEN" in code
    assert "RETURN;" in code


def test_io_address_comments(cell):
    code = generate_scl(cell, steps=[Step("夹紧", outputs={"gripper_close": True})])
    assert "// 夹具: Q0.2" in code


def test_default_steps_unchanged():
    assert len(default_steps()) == 5
