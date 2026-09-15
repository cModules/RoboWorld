#!/usr/bin/env python3
"""v0.3.3 剩余部分测试：焊接/喷涂工艺模板库 + waitDI 完整语义 + 工艺指令出码。

覆盖：
1. ``orolp.process``：模板数据类校验、库的内置默认/YAML 读写/名字解析；
2. ``Program``/``ProgramBuilder``：``arc_start/arc_end/spray_on/spray_off``
   链式指令（实例/模板名/默认三路解析）与 ``waitDI`` 边沿触发语义；
3. 校验透明性：工艺指令混排不影响规划与节拍；
4. 四个主力品牌后处理出码：FANUC 原生 ``ARCON/ARCOFF``，UR/KUKA/ABB 降级
   为 SetDO 语法 + 工艺参数注释；waitDI 边沿触发在无原生指令处必须留
   TODO 注释（含信号地址解析失败的分支，不静默吞语义）。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

from orolp.pose import Pose  # noqa: E402
from orolp.post import get_post_processor  # noqa: E402
from orolp.process import (  # noqa: E402
    ProcessLibrary,
    SprayProcess,
    WeldProcess,
    resolve_spray,
    resolve_weld,
)
from orolp.program import (  # noqa: E402
    ArcEnd,
    ArcStart,
    Motion,
    SprayOff,
    SprayOn,
    WaitDI,
)
from orolp.robot import Robot  # noqa: E402

URDF = REPO_ROOT / "models" / "kuka" / "kr16_2" / "kr16_2.urdf"

IO_MAP = {
    "arc_start": "Q0.3",
    "arc_end": "Q0.4",
    "arc_established": "I0.5",
    "spray_on": "Q0.5",
    "spray_off": "Q0.6",
    "part_ok": "I0.0",
}


@pytest.fixture
def robot() -> Robot:
    if not URDF.exists():
        pytest.skip(f"缺少测试用 URDF: {URDF}")
    return Robot(str(URDF), "kr16_2")


# --------------------------------------------------------------------------- #
# 工艺模板数据类
# --------------------------------------------------------------------------- #
def test_weld_process_defaults():
    w = WeldProcess()
    assert w.name == "default_weld"
    assert w.current == 220.0 and w.voltage == 24.0
    assert w.arc_start_signal == "arc_start"
    assert "电流" in w.summary() and "送丝" in w.summary()


def test_weld_process_validation():
    with pytest.raises(ValueError):
        WeldProcess(current=-1)
    with pytest.raises(ValueError):
        WeldProcess(voltage=0)
    with pytest.raises(ValueError):
        WeldProcess(arc_ok_timeout=-0.1)


def test_spray_process_defaults_and_validation():
    s = SprayProcess()
    assert s.spray_on_signal == "spray_on"
    assert "流量" in s.summary()
    with pytest.raises(ValueError):
        SprayProcess(flow=0)
    with pytest.raises(ValueError):
        SprayProcess(atomizing=-1)


# --------------------------------------------------------------------------- #
# 工艺模板库
# --------------------------------------------------------------------------- #
def test_library_builtin_defaults():
    lib = ProcessLibrary.builtin()
    assert isinstance(lib.require("default_weld"), WeldProcess)
    assert isinstance(lib.require("default_spray"), SprayProcess)
    with pytest.raises(KeyError) as ei:
        lib.require("nope")
    assert "default_weld" in str(ei.value)  # 候选清单进错误信息


def test_library_yaml_roundtrip(tmp_path):
    lib = ProcessLibrary.builtin()
    lib.add(WeldProcess(name="thin", current=140, voltage=18.5, wire_feed=5.5))
    p = tmp_path / "proc.yaml"
    lib.save_yaml(p)

    loaded = ProcessLibrary.load_yaml(p)
    thin = loaded.require("thin")
    assert thin.current == 140 and thin.voltage == 18.5
    assert isinstance(thin, WeldProcess)
    # 内置默认仍在（叠加而非替换）
    assert isinstance(loaded.require("default_spray"), SprayProcess)


def test_library_yaml_kind_guessing(tmp_path):
    """历史手写 YAML 可能没有 kind 字段——按字段名猜。"""
    p = tmp_path / "guess.yaml"
    p.write_text(
        "w1:\n  current: 180\n  voltage: 22\n"
        "s1:\n  flow: 200\n  fan_width: 60\n",
        encoding="utf-8",
    )
    lib = ProcessLibrary.load_yaml(p)
    assert isinstance(lib.require("w1"), WeldProcess)
    assert isinstance(lib.require("s1"), SprayProcess)


def test_library_yaml_unknown_kind_raises(tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text("x:\n  kind: glue\n", encoding="utf-8")
    with pytest.raises(ValueError):
        ProcessLibrary.load_yaml(p)


def test_resolve_helpers():
    assert resolve_weld(None).name == "default_weld"
    w = WeldProcess(name="w2")
    assert resolve_weld(w) is w
    assert resolve_weld("default_weld").name == "default_weld"
    with pytest.raises(TypeError):
        resolve_weld(123)
    with pytest.raises(TypeError):
        resolve_spray("default_weld")  # 类型不匹配
    with pytest.raises(KeyError):
        resolve_weld("missing_weld")


# --------------------------------------------------------------------------- #
# Builder 链式工艺指令
# --------------------------------------------------------------------------- #
def test_builder_arc_chain_with_motions(robot):
    w = WeldProcess(name="w3", current=150)
    prog = (
        robot.moveJ([0, -30, 60, 0, 30, 0])
        .arc_start(w)
        .moveL(Pose.from_xyz_rpy(700, 0, 500, 0, 180, 0))
        .arc_end(w)
        .build()
    )
    kinds = [type(i) for i in prog.items]
    assert kinds == [Motion, ArcStart, Motion, ArcEnd]
    assert prog.items[1].process.current == 150
    assert prog.items[3].process is w  # 熄弧带同一模板（收弧参数）


def test_builder_arc_by_name_and_default(robot):
    prog = robot.arc_start().arc_end("default_weld").build()
    assert prog.items[0].process.name == "default_weld"  # None → 内置默认
    assert isinstance(prog.items[1].process, WeldProcess)


def test_builder_spray_chain(robot):
    prog = robot.spray_on("default_spray").spray_off().build()
    assert isinstance(prog.items[0], SprayOn)
    assert isinstance(prog.items[1], SprayOff)
    assert prog.items[0].process.flow == 300.0
    assert prog.items[1].process is None


def test_builder_waitdi_trigger(robot):
    prog = robot.waitDI("part_ok", True, 5.0, trigger="rising").build()
    assert prog.items[0].trigger == "rising"
    # 电平默认不变，向后兼容
    prog2 = robot.waitDI("part_ok").build()
    assert prog2.items[0].trigger == "level"


def test_waitdi_invalid_trigger_raises(robot):
    with pytest.raises(ValueError):
        robot.waitDI("part_ok", trigger="edge").build()
    with pytest.raises(ValueError):
        WaitDI("x", True, 0.0, "pulse")


def test_program_level_waitdi_transparent(robot):
    """IO 混排不影响规划——工艺指令对 planner 完全透明。"""
    prog = (
        robot.moveJ([0, -30, 60, 0, 30, 0])
        .arc_start()
        .moveL(Pose.from_xyz_rpy(700, 0, 500, 0, 180, 0))
        .arc_end()
        .spray_on()
        .moveL(Pose.from_xyz_rpy(700, 100, 500, 0, 180, 0))
        .spray_off()
        .build()
    )
    assert len(prog.motions) == 3  # 只有 Motion 进规划
    report = prog.validate()
    assert report.cycle_time > 0
    assert [i for i in report.issues if i.severity == "error"] == []


# --------------------------------------------------------------------------- #
# 品牌出码
# --------------------------------------------------------------------------- #
def _weld_program(robot):
    w = WeldProcess(name="wq", current=150, voltage=19.0, wire_feed=6.0)
    return (
        robot.moveJ([0, -30, 60, 0, 30, 0])
        .moveL(Pose.from_xyz_rpy(700, 0, 500, 0, 180, 0))
        .arc_start(w)
        .moveL(Pose.from_xyz_rpy(800, 0, 500, 0, 180, 0))
        .arc_end(w)
        .build()
    ), w


def test_fanuc_native_arcon_arcoff(robot):
    prog, _ = _weld_program(robot)
    code = prog.to_post("fanuc_r30ib", metadata={"io_map": IO_MAP})
    assert "ARCON AC=150 V=19.0" in code
    assert "送丝 6.0m/min" in code
    assert "ARCOFF" in code
    assert "熄弧后保护气滞流" in code


def test_ur_degrades_setdo_with_params(robot):
    prog, _ = _weld_program(robot)
    code = prog.to_post("ur_polyscope", metadata={"io_map": IO_MAP})
    assert "set_digital_out(3, True)" in code  # Q0.3 → 序号 3
    assert "焊接 电流150A" in code  # 工艺参数进注释


def test_kuka_degrades_setdo_with_params(robot):
    prog, _ = _weld_program(robot)
    code = prog.to_post("kuka_krc4", metadata={"io_map": IO_MAP})
    assert "$OUT[3] = TRUE" in code
    assert "电流150A" in code


def test_abb_degrades_setdo_with_params(robot):
    prog, _ = _weld_program(robot)
    code = prog.to_post("abb_irc5", metadata={"io_map": IO_MAP})
    assert "SetDO arc_start, 1" in code
    assert "! 焊接 电流150A" in code


def test_spray_degrades_all_brands(robot):
    prog = robot.spray_on().spray_off().build()
    for name in ("ur_polyscope", "kuka_krc4", "abb_irc5", "fanuc_r30ib"):
        code = prog.to_post(name, metadata={"io_map": IO_MAP})
        # UR/KUKA/FANUC 把 Q0.5/Q0.6 解析成数字序号，ABB 带符号名
        on_ok = (
            "spray_on" in code
            or "set_digital_out(5, True)" in code
            or "$OUT[5] = TRUE" in code
            or "DO[5]=ON" in code
        )
        off_ok = (
            "spray_off" in code
            or "set_digital_out(6, False)" in code
            or "$OUT[6] = FALSE" in code
            or "DO[6]=OFF" in code
        )
        assert on_ok and off_ok, f"{name} 缺喷涂开/关指令:\n{code}"
        assert "喷涂 流量300ml/min" in code


def test_waitdi_edge_todo_in_all_brands(robot):
    """边沿触发无原生指令时必须留 TODO，不能静默按电平输出。"""
    for name in ("ur_polyscope", "kuka_krc4", "abb_irc5", "fanuc_r30ib"):
        prog = robot.waitDI("part_ok", True, 2.0, trigger="falling").build()
        code = prog.to_post(name, metadata={"io_map": IO_MAP})
        assert "边沿触发" in code, f"{name} 缺少边沿语义 TODO"
        assert "falling" in code


def test_waitdi_edge_todo_survives_unresolved_signal(robot):
    """信号地址解析失败的 TODO 分支同样不能吞掉边沿语义。"""
    for name in ("ur_polyscope", "kuka_krc4", "fanuc_r30ib"):
        prog = robot.waitDI("mystery", True, 0.0, trigger="rising").build()
        code = prog.to_post(name, metadata={})  # 空 io_map → 解析失败分支
        assert "边沿触发" in code, f"{name} 未解析地址分支丢失边沿语义"


def test_waitdi_level_no_edge_todo(robot):
    prog = robot.waitDI("part_ok", True, 2.0).build()
    code = prog.to_post("abb_irc5", metadata={"io_map": IO_MAP})
    assert "边沿" not in code
    assert "\\MaxTime:=2.000" in code


def test_plc_scl_ignores_process_commands():
    from orolp.plc_scl import steps_from_program

    class FakeProgram:  # 避免依赖 Robot，只测指令遍历
        items = [
            ArcStart(WeldProcess()),
            SprayOn(SprayProcess()),
        ]

    steps = steps_from_program(FakeProgram())
    # 纯工艺指令不产生 PLC 步序（工艺由机器人侧执行）
    assert steps == []
