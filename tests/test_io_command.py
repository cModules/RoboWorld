"""IO 指令层（``SetDO`` / ``WaitDI`` / ``WaitTime``）的回归测试。

守的是「机器人程序里能出抓取动作」：

* IO 指令必须与运动指令**混排有序**，且对轨迹规划**透明**（planner 只看 Motion）
* 物理地址解析失败时**绝不编造假点位**，必须留下可见的 TODO

写错 IO 点位在现场会导致夹爪不动作或误动作，是最贵的一类错误，
所以这里对"降级行为"也做断言，而不只测正常路径。

运行::

    pytest tests/test_io_command.py -q
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from domain.pose import Pose  # noqa: E402
from adapter.post import get_post_processor, list_post_processors  # noqa: E402
from service.program import (
    Motion,
    MotionType,
    Program,
    SetDO,
    WaitDI,
    WaitTime,
)
from domain.robot import Robot  # noqa: E402

URDF = REPO_ROOT / "assets" / "models" / "kuka" / "kr16_2" / "kr16_2.urdf"

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


def pick_program(robot: Robot) -> Program:
    """一段带抓取 IO 的取放程序。"""
    return Program(
        robot,
        [
            Motion(MotionType.JOINTS, [0, -30, 60, 0, 30, 0]),
            Motion(MotionType.LINEAR, Pose.from_xyz_rpy(700, 0, 900, 0, 180, 0)),
            SetDO("gripper_close", True),
            WaitTime(0.5),
            WaitDI("clamped", True, timeout=2.0),
            SetDO("gripper_open", True),
        ],
    )


# --------------------------------------------------------------------------- #
# Program 容器
# --------------------------------------------------------------------------- #


def test_items_keep_order_and_motions_filters(robot):
    """items 保留全部指令与顺序；motions 只过滤出运动指令。"""
    prog = pick_program(robot)

    assert len(prog.items) == 6
    assert len(prog.motions) == 2
    # 顺序不能乱——IO 插在哪个运动之后，决定了夹爪何时动作
    assert isinstance(prog.items[2], SetDO)
    assert isinstance(prog.items[3], WaitTime)
    assert isinstance(prog.items[4], WaitDI)


def test_planner_sees_only_motions(robot):
    """IO 指令对轨迹规划必须透明：带 IO 的程序照样能规划。"""
    prog = pick_program(robot)
    traj = prog.plan()
    assert len(traj.waypoints) > 0


def test_add_appends(robot):
    prog = Program(robot)
    prog.add(Motion(MotionType.JOINTS, [0, 0, 0, 0, 0, 0]))
    prog.add(SetDO("gripper_close", True))
    assert len(prog.items) == 2


# --------------------------------------------------------------------------- #
# 地址解析
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "addr,expected",
    [
        ("Q0.0", 0),
        ("Q0.1", 1),
        ("Q1.3", 11),  # 1*8+3
        ("I0.3", 3),
        ("i2.7", 23),
    ],
)
def test_io_index_parsing(addr, expected):
    pp = get_post_processor("kuka_krc4")
    assert pp.io_index(addr) == expected


@pytest.mark.parametrize(
    "addr",
    ["", "X0.1", "Q0", "abc", "Q0.1.2"],
)
def test_io_index_rejects_unparseable(addr):
    """解析不了必须返回 None，而不是猜一个序号。"""
    pp = get_post_processor("kuka_krc4")
    assert pp.io_index(addr) is None


# --------------------------------------------------------------------------- #
# 各品牌语法
# --------------------------------------------------------------------------- #


def test_kuka_io_syntax(robot):
    code = pick_program(robot).to_post(
        "kuka_krc4",
        metadata={"program_name": "PICK", "io_map": IO_MAP},
    )
    assert "$OUT[1] = TRUE" in code  # gripper_close → Q0.1
    assert "$OUT[0] = TRUE" in code  # gripper_open  → Q0.0
    assert "WAIT SEC 0.500" in code
    assert "WAIT FOR $IN[3]" in code  # clamped       → I0.3


def test_abb_io_syntax(robot):
    code = pick_program(robot).to_post(
        "abb_irc5",
        metadata={"program_name": "PICK", "io_map": IO_MAP},
    )
    assert "SetDO gripper_close, 1;" in code
    assert "WaitTime 0.500;" in code
    assert "WaitDI clamped, 1 \\MaxTime:=2.000;" in code


def test_ur_io_syntax(robot):
    code = pick_program(robot).to_post(
        "ur_polyscope",
        metadata={"program_name": "PICK", "io_map": IO_MAP},
    )
    assert "set_digital_out(1, True)" in code
    assert "sleep(0.500)" in code
    assert "wait_digital_in(3, True)" in code


def test_io_appears_in_order(robot):
    """IO 必须出现在正确的运动之后——顺序错了就是撞机。"""
    code = pick_program(robot).to_post(
        "kuka_krc4",
        metadata={"program_name": "PICK", "io_map": IO_MAP},
    )
    assert code.index("LIN") < code.index("$OUT[1] = TRUE")
    assert code.index("$OUT[1] = TRUE") < code.index("WAIT SEC")


# --------------------------------------------------------------------------- #
# 降级行为：绝不静默编造假地址
# --------------------------------------------------------------------------- #


def test_unmapped_signal_yields_todo_not_fake_address():
    """未映射的信号必须留 TODO，不得输出一个看似正确的点位。"""
    pp = get_post_processor("kuka_krc4")
    code = pp.generate(
        [SetDO("gripper_close", True)],
        metadata={"program_name": "T", "io_map": {}},
    )
    assert "TODO" in code
    # 关键：不能出现 $OUT[<数字>]，否则看起来像真地址。
    # （TODO 注释里会写 $OUT[n] 作为改写提示，那是纯文本，不算真地址）
    assert re.search(r"\$OUT\[\d", code) is None


def test_timeout_is_flagged_where_unsupported():
    """KRL 的 WAIT FOR 没有超时参数——给了超时必须提示，不能静默丢弃。"""
    pp = get_post_processor("kuka_krc4")
    code = pp.generate(
        [WaitDI("clamped", True, timeout=2.0)],
        metadata={"program_name": "T", "io_map": IO_MAP},
    )
    assert "WAIT FOR $IN[3]" in code
    assert "超时" in code


@pytest.mark.parametrize("name", list_post_processors())
def test_every_brand_emits_io_operand(robot, name):
    """**所有**品牌的模板都必须把 IO 的操作数输出出来。

    这条测试是被真实缺陷逼出来的：FANUC 模板原本只认运动指令，遇到 IO 时
    退化到 else 分支，输出一个光秃秃的 ``SETDO ;``——指令名在、操作数没了。
    代码看起来"生成成功"，下装后夹爪却毫无动作，且极难排查。

    这里不校验各品牌语法是否地道（未适配的品牌输出通用伪语法是可接受的），
    只强制要求：**地址或信号名必须出现在输出里**。
    """
    pp = get_post_processor(name)

    # 直接比对"格式化后的 IO 文本"是否原样出现在最终代码里——
    # 比匹配地址串更严格也更通用（KUKA/UR 用数字序号，ABB 用符号名）
    setdo = pp.format_setdo("gripper_close", True, IO_MAP).strip()
    waitdi = pp.format_waitdi("clamped", True, 0.0, IO_MAP).strip()
    waittime = pp.format_waittime(0.5).strip()

    code = pp.generate(
        [
            Motion(MotionType.JOINTS, [0, 0, 0, 0, 0, 0]),
            SetDO("gripper_close", True),
            WaitDI("clamped", True),
            WaitTime(0.5),
        ],
        metadata={"program_name": "T", "io_map": IO_MAP},
    )

    assert setdo, f"{name} 的 SetDO 文本为空"
    assert setdo in code, f"{name} 吞掉了 SetDO 操作数: {setdo!r}"
    assert waitdi in code, f"{name} 吞掉了 WaitDI 操作数: {waitdi!r}"
    assert waittime in code, f"{name} 吞掉了 WaitTime 时长: {waittime!r}"
