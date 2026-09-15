"""工艺 IO 链式指令（S2）的测试：``setDO`` / ``waitDI`` / ``gripper``。

守的是「用户在程序里写抓取动作，出码后动作真的在、且在正确的位置」：

* 链式 API 与运动指令**混排有序**
* ``gripper("close"/"open")`` 展开出的信号互锁语义与
  :func:`orolp.pickplace.build_pick_place` 一致
* ``then()`` 合并 Program 时**不得丢 IO**（早期实现只取 motions，
  抓取动作静默消失）
* FANUC TP 输出 ``DO[n]=ON`` / ``WAIT DI[n]`` 品牌语法

运行::

    pytest tests/test_io_instructions.py -q
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

from orolp.pose import Pose  # noqa: E402
from orolp.post import get_post_processor  # noqa: E402
from orolp.program import (  # noqa: E402
    Motion,
    MotionType,
    Program,
    ProgramBuilder,
    SetDO,
    WaitDI,
    WaitTime,
)
from orolp.robot import Robot  # noqa: E402

URDF = REPO_ROOT / "models" / "kuka" / "kr16_2" / "kr16_2.urdf"

IO_MAP = {
    "gripper_close": "Q0.1",
    "gripper_open": "Q0.0",
    "clamped": "I0.3",
}


@pytest.fixture
def robot() -> Robot:
    if not URDF.exists():
        pytest.skip(f"缺少测试用 URDF: {URDF}")
    return Robot(str(URDF), "kr16_2")


# --------------------------------------------------------------------------- #
# Builder 链式 IO 指令
# --------------------------------------------------------------------------- #


def test_builder_setdo_waitdi_waittime_chainable(robot):
    """setDO/waitDI/waitTime 可链式书写且保序。"""
    prog = (
        robot.moveJ([0, -30, 60, 0, 30, 0])
        .setDO("gripper_close", True)
        .waitDI("clamped", True, timeout=2.0)
        .waitTime(0.5)
        .build()
    )
    assert isinstance(prog.items[1], SetDO)
    assert prog.items[1].signal == "gripper_close"
    assert isinstance(prog.items[2], WaitDI)
    assert prog.items[2].timeout == 2.0
    assert isinstance(prog.items[3], WaitTime)
    assert prog.items[3].seconds == 0.5


def test_builder_setdo_false_resets(robot):
    prog = robot.moveJ([0, 0, 0, 0, 0, 0]).setDO("gripper_close", False).build()
    assert isinstance(prog.items[1], SetDO)
    assert prog.items[1].value is False


def test_gripper_close_expands_interlocked(robot):
    """close = 复位 open + 置位 close + 等 clamped 反馈。"""
    prog = robot.moveL(Pose.from_xyz_rpy(600, 0, 800, 0, 180, 0)).gripper("close").build()
    tail = prog.items[1:]
    assert [type(i) for i in tail] == [SetDO, SetDO, WaitDI]
    assert (tail[0].signal, tail[0].value) == ("gripper_open", False)
    assert (tail[1].signal, tail[1].value) == ("gripper_close", True)
    assert (tail[2].signal, tail[2].value) == ("clamped", True)
    assert tail[2].timeout == pytest.approx(2.0)


def test_gripper_open_expands_interlocked_no_feedback(robot):
    """open = 复位 close + 置位 open；无反馈信号，confirm 不生效。"""
    prog = (
        robot.moveL(Pose.from_xyz_rpy(600, 0, 800, 0, 180, 0))
        .gripper("open", confirm=True)
        .build()
    )
    tail = prog.items[1:]
    assert [type(i) for i in tail] == [SetDO, SetDO]
    assert (tail[0].signal, tail[0].value) == ("gripper_close", False)
    assert (tail[1].signal, tail[1].value) == ("gripper_open", True)


def test_gripper_close_without_confirm_skips_waitdi(robot):
    prog = robot.moveJ([0, 0, 0, 0, 0, 0]).gripper("close", confirm=False).build()
    tail = prog.items[1:]
    assert all(isinstance(i, SetDO) for i in tail)


def test_gripper_invalid_action_raises(robot):
    with pytest.raises(ValueError):
        robot.moveJ([0, 0, 0, 0, 0, 0]).gripper("toggle")


def test_then_merges_program_with_io(robot):
    """合并 Program 必须保留 IO 指令——抓取动作不能静默消失。"""
    sub = (
        Program(robot)
        .setDO("gripper_close", True)
        .waitDI("clamped", True, timeout=1.0)
    )
    prog = robot.moveJ([0, -30, 60, 0, 30, 0]).then(sub).build()
    kinds = [type(i) for i in prog.items]
    assert kinds == [Motion, SetDO, WaitDI]


def test_io_program_plans_transparently(robot):
    """IO 混排不影响轨迹规划（SetDO/WaitDI 不占时间；v1.1.3 起 waitTime
    作为驻留段占时间，见 tests/test_dwell.py）。"""
    prog = (
        robot.moveJ([0, -30, 60, 0, 30, 0])
        .setDO("gripper_close", True)
        .waitTime(0.3)
        .build()
    )
    traj = prog.plan()
    assert len(traj.waypoints) > 0


def test_validate_counts_only_motions_with_io_present(robot):
    """回归：``validate()`` 曾把 IO 指令当运动指令。

    ``ProgramBuilder.motions`` 是"全部指令"的历史别名，而校验要的是
    ``Program.motions``（只有 :class:`Motion`）。混排时旧实现会抛
    ``'SetDO' object has no attribute 'type'``，即便侥幸不抛，节拍明细也会
    与夹爪事件错位（事件时刻被算到错误的下标上）。
    """
    prog = (
        robot.moveJ([0, -30, 60, 0, 30, 0])
        .setDO("gripper_close", True)
        .waitTime(0.3)
        .moveJ([0, -45, 90, 0, 45, 0])
        .setDO("gripper_open", True)
        .build()
    )
    report = prog.validate(check_dynamics=False)
    assert report.num_motions == 2, "IO 指令不能计入运动指令条数"
    assert report.motion_labels == ["moveJ", "moveJ"]
    assert len(report.motion_times or []) == 2
    assert report.cycle_time and report.cycle_time > 0.0


def test_io_program_to_post_keeps_order(robot):
    code = (
        robot.moveL(Pose.from_xyz_rpy(700, 0, 900, 0, 180, 0))
        .gripper("close")
        .build()
        .to_post("kuka_krc4", metadata={"program_name": "PICK", "io_map": IO_MAP})
    )
    assert code.index("LIN") < code.index("$OUT[0] = FALSE")
    assert code.index("$OUT[0] = FALSE") < code.index("$OUT[1] = TRUE")
    assert code.index("$OUT[1] = TRUE") < code.index("WAIT FOR $IN[3]")


# --------------------------------------------------------------------------- #
# FANUC TP 品牌语法
# --------------------------------------------------------------------------- #


def test_fanuc_setdo_syntax(robot):
    code = (
        robot.moveJ([0, -30, 60, 0, 30, 0])
        .setDO("gripper_close", True)
        .setDO("gripper_open", False)
        .build()
        .to_post("fanuc_r30ib", metadata={"program_name": "PICK", "io_map": IO_MAP})
    )
    assert "DO[1]=ON" in code
    assert "DO[0]=OFF" in code


def test_fanuc_waitdi_and_waittime_syntax(robot):
    code = (
        robot.moveJ([0, -30, 60, 0, 30, 0])
        .waitDI("clamped", True, timeout=2.0)
        .waitTime(0.5)
        .build()
        .to_post("fanuc_r30ib", metadata={"program_name": "PICK", "io_map": IO_MAP})
    )
    assert "WAIT DI[3]=ON" in code
    assert "超时" in code          # TP 不支持超时，必须提示而不是静默丢弃
    assert "WAIT 0.50(sec)" in code


def test_fanuc_unmapped_signal_yields_todo(robot):
    pp = get_post_processor("fanuc_r30ib")
    text = pp.format_setdo("gripper_close", True, {})
    assert "TODO" in text


def test_program_level_io_methods(robot):
    """Program 上也有同名方法（add 场景，不带构建器）。"""
    prog = Program(robot)
    prog.setDO("gripper_close", True).gripper("open").waitTime(0.2)
    kinds = [type(i) for i in prog.items]
    assert kinds == [SetDO, SetDO, SetDO, WaitTime]


# --------------------------------------------------------------------------- #
# pick_and_place 示例：占位注释已换成真实 IO
# --------------------------------------------------------------------------- #


def test_pick_and_place_example_runs_with_io():
    """示例脚本可完整跑通，且不再包含 IO 占位注释。"""
    result = subprocess.run(
        [sys.executable, str(REPO_ROOT / "examples" / "pick_and_place.py")],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        timeout=180,
        env={
            "PYTHONPATH": str(REPO_ROOT / "python"),
            "PATH": "/usr/bin:/bin:/usr/local/bin",
        },
    )
    assert result.returncode == 0, result.stderr[-2000:]
    assert "would go here" not in result.stdout
    assert "would go here" not in (result.stderr or "")
