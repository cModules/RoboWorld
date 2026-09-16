"""工艺时序（v1.3.1）：``orolp.sequence`` 模型/校验/双出口 + 设备树里的时序编辑器。

PRD §4.3 P1 的「动态时序」在本轮落地：工序触发（SignalStep）、信号等待
（WaitStep）、子程序调用（CallStep）、驻留（DwellStep）编成有序序列，翻译到
机器人侧（``to_program`` → 既有 SetDO/WaitDI/WaitTime）与 PLC 侧
（``to_plc_steps`` → ``plc_scl.Step``，可直接出 SCL）。诚实性口径由测试钉住：
只有驻留是可计算时间，summary 绝不编"预计节拍"。

跑法::

    QT_QPA_PLATFORM=offscreen pytest tests/test_sequence.py -q
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
for _p in (REPO_ROOT, REPO_ROOT / "python"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from orolp.planner import SEVERITY_ERROR, SEVERITY_WARNING
from orolp.program import ProgramBuilder, SetDO, WaitDI, WaitTime
from orolp.sequence import (  # noqa: E402
    CallStep,
    DwellStep,
    ProcessSequence,
    SeqIssue,
    SignalStep,
    WaitStep,
)


def _sample() -> ProcessSequence:
    """参考序列：**零告警**的那种（边沿触发的告警有自己的专测，别混进来）。"""
    return ProcessSequence([
        SignalStep("夹紧", "机器人", "gripper_close", value=True),
        DwellStep("保压", "机器人", 0.5),
        CallStep("抓取", "机器人", "PickPart"),
        WaitStep("等到位", "输送带", "part_ready", trigger="level", timeout=5.0),
        SignalStep("放行", "输送带", "conveyor_run", value=False),
    ])


#: 内置 UR5e——注意真实路径是 ``models/ur/ur5e/``，写错会静默 skip 成"绿"
UR5E_URDF = REPO_ROOT / "models" / "ur" / "ur5e" / "ur5e.urdf"


def _make_builder() -> ProgramBuilder:
    """内置 UR5e 的 :class:`ProgramBuilder`；模型真缺失才跳过。"""
    if not UR5E_URDF.exists():
        pytest.skip(f"内置 UR5e 模型缺失：{UR5E_URDF}")
    from orolp import Robot

    return ProgramBuilder(Robot(str(UR5E_URDF), "UR5e"))


def _sig(step):
    """工步的语义字段（不含步名——两侧命名口径有意不同）。"""
    return (step.outputs, step.wait, step.wait_value, step.delay)


def _codes(issues):
    return {i.code for i in issues}


# --------------------------------------------------------------------------- #
# 模型
# --------------------------------------------------------------------------- #


def test_add_and_lanes_keep_first_appearance_order():
    seq = _sample()
    assert seq.lanes() == ["机器人", "输送带"]
    assert len(seq) == 5


def test_move_swaps_and_reports_bounds():
    seq = _sample()
    assert seq.move(0, 1) is True
    assert seq.steps[1].name == "夹紧"
    assert seq.move(0, -1) is False      # 越界不抛错、返回 False
    assert seq.move(len(seq), 1) is False


def test_insert_clamps_and_remove_returns_step():
    seq = ProcessSequence()
    seq.insert(99, DwellStep("d", "机器人", 1.0))
    assert len(seq) == 1                  # 越界插入夹到末尾
    removed = seq.remove(0)
    assert isinstance(removed, DwellStep) and len(seq) == 0


def test_wait_trigger_whitelist():
    with pytest.raises(ValueError):
        WaitStep("w", "机器人", "s", trigger="both")


def test_dwell_seconds_non_negative():
    with pytest.raises(ValueError):
        DwellStep("d", "机器人", -0.1)


# --------------------------------------------------------------------------- #
# 校验
# --------------------------------------------------------------------------- #


def test_empty_sequence_warns():
    issues = ProcessSequence().validate()
    assert _codes(issues) == {"empty"}
    assert issues[0].severity == SEVERITY_WARNING


def test_clean_sequence_passes():
    assert _sample().validate() == []


def test_unknown_signal_is_error():
    issues = _sample().validate(signal_names={"gripper_close", "conveyor_run"})
    codes = _codes(issues)
    assert "unknown_signal" in codes
    bad = [i for i in issues if i.code == "unknown_signal"]
    assert all(i.severity == SEVERITY_ERROR for i in bad)
    # 报的是点表里没有的那一个，而不是全部
    assert any("part_ready" in i.message for i in bad)


def test_no_timeout_is_warning():
    seq = ProcessSequence([WaitStep("w", "机器人", "part_ready")])
    issues = seq.validate()
    assert _codes(issues) == {"no_timeout"}
    assert issues[0].severity == SEVERITY_WARNING


def test_edge_trigger_ignores_value():
    seq = ProcessSequence([WaitStep("w", "机器人", "s", trigger="falling", value=True, timeout=1.0)])
    assert _codes(seq.validate()) == {"edge_ignores_value"}


def test_unknown_device_and_subprogram_are_errors():
    seq = ProcessSequence([
        SignalStep("s", "不存在的设备", "gripper_close"),
        CallStep("c", "机器人", "NoSuchSub"),
    ])
    issues = seq.validate(devices=["机器人"], subprograms=["PickPart"])
    codes = _codes(issues)
    assert {"unknown_device", "unknown_subprogram"} <= codes
    assert all(i.severity == SEVERITY_ERROR for i in issues)


def test_duplicate_names_warn_with_index():
    seq = ProcessSequence([
        DwellStep("同名", "机器人", 0.1),
        DwellStep("同名", "机器人", 0.2),
    ])
    issues = seq.validate()
    assert _codes(issues) == {"duplicate_name"}
    assert issues[0].index == 1            # 指向后出现的那一个


def test_issue_carries_index():
    seq = ProcessSequence([
        SignalStep("a", "机器人", "gripper_close"),
        SignalStep("b", "机器人", "nope"),
    ])
    issues = seq.validate(signal_names={"gripper_close"})
    assert len(issues) == 1 and issues[0].index == 1


# --------------------------------------------------------------------------- #
# 出口：机器人侧 / PLC 侧
# --------------------------------------------------------------------------- #


def test_to_program_translates_io_steps():
    builder = _make_builder()
    notes = _sample().to_program(builder)
    items = builder.build().items          # builder 本身只有 .motions，指令在 Program 上
    kinds = [type(i) for i in items]
    # 5 步里 CallStep 不是程序指令（见下一条测试），落地 4 条 IO 指令
    assert kinds == [SetDO, WaitTime, WaitDI, SetDO]
    wait = items[2]
    assert isinstance(wait, WaitDI)
    assert wait.signal == "part_ready"
    assert wait.trigger == "level" and wait.value is True and wait.timeout == 5.0
    assert len(notes) == 1 and "PickPart" in notes[0]


def test_to_program_passes_edge_trigger_through():
    """rising/falling 原样落到 WaitDI——边沿语义不在翻译层被悄悄改成电平。"""
    builder = _make_builder()
    seq = ProcessSequence([
        WaitStep("等边沿", "输送带", "part_ready", trigger="rising", timeout=2.0),
    ])
    seq.to_program(builder)
    wait = builder.build().items[0]
    assert isinstance(wait, WaitDI) and wait.trigger == "rising"
    assert _codes(seq.validate()) == {"edge_ignores_value"}   # 边沿忽略电平，如实告警


def test_to_program_records_call_in_notes():
    builder = _make_builder()
    seq = ProcessSequence([CallStep("抓取", "机器人", "PickPart")])
    notes = seq.to_program(builder)
    assert builder.build().items == []     # 不臆造调用语法
    assert len(notes) == 1 and "PickPart" in notes[0]


def test_to_plc_steps_semantics_match_steps_from_program():
    """序列侧 PLC 出口与程序侧 ``steps_from_program`` 逐条同义。

    两处**刻意**的差异，别当成漂移：
    * ``steps_from_program`` 会在头部补一步「等待工件到位」当循环起点；序列
      自己就编排等待，再补一步会与序列里的等待打架，所以序列侧不补；
    * 步名不同（驻留 0.5s vs 保压 0.50s、末步输出 vs 输出保持），语义字段必须一致。
    """
    from orolp.plc_scl import steps_from_program

    builder = _make_builder()
    _sample().to_program(builder)
    from_seq = _sample().to_plc_steps()
    from_prog = steps_from_program(builder.build().items)

    assert from_prog[0].wait == "part_present"          # 程序侧多一步循环起点
    assert [_sig(s) for s in from_seq] == [_sig(s) for s in from_prog[1:]]

    # 别只断言"两边一样"——把翻译规则本身也钉死
    assert [s.delay for s in from_seq] == [0.5, 0.0, 0.0]
    assert from_seq[0].outputs == {"gripper_close": True}   # 触发被下一次收束带走
    assert from_seq[1].wait == "part_ready" and from_seq[1].wait_value is True
    assert from_seq[1].outputs == {}                        # 等待步把前置输出收走了
    assert from_seq[-1].outputs == {"conveyor_run": False}   # 末步输出


def test_to_plc_steps_call_flushes_pending_and_is_skipped():
    seq = ProcessSequence([
        SignalStep("s", "机器人", "gripper_close"),
        CallStep("c", "机器人", "PickPart"),
    ])
    steps = seq.to_plc_steps()
    assert len(steps) == 1                 # 调用不进 PLC 步序
    assert steps[0].outputs == {"gripper_close": True}


def test_summary_reports_dwell_only_as_time():
    s = _sample().summary()
    assert s["steps"] == 5
    assert s["counts"] == {"call": 1, "signal": 2, "wait": 1, "dwell": 1}
    assert s["dwell_total_s"] == 0.5
    assert "取决于现场" in s["note"]       # 诚实口径：别的时间不编


def test_to_plc_steps_feed_generate_scl_end_to_end():
    """序列的 PLC 出口能直接出 SCL——docstring 里那句"可直接喂给 generate_scl"要被钉住。"""
    from orolp.cell import CellConfig, Device
    from orolp.plc_scl import generate_scl

    cell = CellConfig()
    cell.add(Device(name="夹具", type="夹具", io_map={"gripper_close": "Q0.2"}))
    cell.add(Device(
        name="输送带", type="输送带",
        io_map={"part_ready": "I0.4", "conveyor_run": "Q0.4"},
    ))

    code = generate_scl(cell, program_name="CellControl", steps=_sample().to_plc_steps())
    head = code.split("BEGIN")[0]
    # 序列用到的信号都必须先声明，否则 TIA 编译不过
    for sig in ("gripper_close", "part_ready", "conveyor_run"):
        assert sig in head, f"{sig} 未声明"
    assert "dwellTimer : TON" in code and "T#500MS" in code   # 0.5s 驻留 → 定时器


# --------------------------------------------------------------------------- #
# JSON 往返
# --------------------------------------------------------------------------- #


def test_json_roundtrip(tmp_path):
    seq = _sample()
    path = tmp_path / "seq.json"
    seq.save_json(path)
    data = json.loads(path.read_text(encoding="utf-8"))
    assert data["schema"] == "orolp.sequence/1"
    back = ProcessSequence.load_json(path)
    assert [type(a) for a in back.steps] == [type(b) for b in seq.steps]
    assert back.summary() == seq.summary()


def test_load_rejects_bad_schema(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text(json.dumps({"schema": "orolp.sequence/9", "steps": []}), encoding="utf-8")
    with pytest.raises(ValueError):
        ProcessSequence.load_json(path)


def test_load_rejects_unknown_kind(tmp_path):
    path = tmp_path / "bad2.json"
    path.write_text(
        json.dumps({"schema": "orolp.sequence/1", "steps": [{"kind": "teleport"}]}),
        encoding="utf-8",
    )
    with pytest.raises(ValueError):
        ProcessSequence.load_json(path)


def test_package_exports():
    import orolp

    for name in ("ProcessSequence", "SeqIssue", "SeqStep",
                 "SignalStep", "WaitStep", "CallStep", "DwellStep"):
        assert hasattr(orolp, name), name


# --------------------------------------------------------------------------- #
# 设备树里的时序编辑器（offscreen）
# --------------------------------------------------------------------------- #


@pytest.fixture(scope="module")
def qapp():
    from PySide6 import QtWidgets

    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture()
def editor(qapp):
    from orolp.world.panel import SequenceEditor

    w = SequenceEditor()
    yield w
    w.deleteLater()


def test_editor_add_updates_table_and_canvas(editor):
    idx = editor.add_step("signal", device="机器人", signal="gripper_close", value=True)
    assert idx == 0
    assert editor.table.rowCount() == 1
    assert editor.table.item(0, 2).text() == "触发"
    assert editor.table.item(0, 3).text() == "gripper_close = True"
    assert len(editor.canvas._blocks) == 1


def test_editor_move_and_remove(editor):
    editor.add_step("dwell", seconds=0.5)
    editor.add_step("dwell", seconds=1.0)
    assert editor.move_step(0, 1) is True
    assert float(editor.sequence.steps[1].seconds) == 0.5
    assert editor.move_step(1, 1) is False
    editor.remove_step(0)
    assert len(editor.sequence) == 1


def test_editor_validate_reports_unknown_signal(editor):
    editor.set_available_signals(["gripper_close"])
    editor.add_step("signal", device="机器人", signal="ghost_signal")
    issues = editor.validate_now()
    assert any(i.code == "unknown_signal" and i.severity == SEVERITY_ERROR for i in issues)
    assert "错误" in editor.validate_label.text()


def test_editor_validate_clean_shows_pass(editor):
    editor.add_step("dwell", seconds=0.2)
    assert editor.validate_now() == []
    assert "校验通过" in editor.validate_label.text()


def test_editor_wait_dialog_values(qapp):
    from orolp.world.panel import _StepDialog

    dialog = _StepDialog("wait", ["机器人", "输送带"], ["part_ready"], [])
    dialog.name_edit.setText("等到位")
    dialog.device_combo.setCurrentText("输送带")
    dialog.signal_combo.setCurrentText("part_ready")
    dialog.trigger_combo.setCurrentIndex(1)          # 上升沿
    dialog.timeout_spin.setValue(3.0)
    values = dialog.values()
    assert values == {
        "name": "等到位", "device": "输送带", "signal": "part_ready",
        "trigger": "rising", "value": True, "timeout": 3.0,
    }


def test_editor_save_load_roundtrip(editor, qapp, tmp_path):
    editor.add_step("signal", device="机器人", signal="gripper_close")
    editor.add_step("dwell", device="机器人", seconds=0.8)
    path = tmp_path / "seq.json"
    editor.save_to(path)
    other = type(editor)()
    other.load_from(path)
    assert other.summary_text() == editor.summary_text()
    assert float(other.sequence.steps[1].seconds) == 0.8
    other.deleteLater()


def test_editor_in_scene_panel_wiring(qapp):
    from orolp.world.panel import ScenePanel

    panel = ScenePanel()
    got = []
    # 面板对外的信号是 sceneChanged（时序改动＝场景改动），编辑器内部的
    # sequenceChanged 被转发到这里——别去连面板上不存在的序列信号
    panel.sceneChanged.connect(lambda: got.append(1))
    n_before = len(panel.sequence_editor.sequence)
    panel.sequence_editor.add_step("dwell", seconds=0.1)
    assert len(got) >= 1                   # 编辑器的改动转发到了面板
    assert len(panel.sequence_editor.sequence) == n_before + 1
    panel.deleteLater()


def test_editor_default_names_increment(editor):
    editor.add_step("dwell", seconds=0.1)
    editor.add_step("dwell", seconds=0.2)
    assert editor.sequence.steps[0].name != editor.sequence.steps[1].name


def test_new_strings_translated(monkeypatch):
    from orolp import i18n

    monkeypatch.setattr(i18n, "_LANGUAGE", "en", raising=False)
    for text in (
        "工艺时序（泳道图）",
        "添加工步",
        "校验时序",
        "保存时序…",
        "载入时序…",
        "未校验",
        "添加工步：等待信号",
        "先在步骤表里选中一行",
        "等待触发方式必须是 {opts} 之一，收到 {got}",
        "驻留时长不能为负，收到 {s}s",
        "未知工步类型：{kind}",
        "末步输出",
        "上升沿",
        "下降沿",
        "{where} 无限等待 {signal}——现场必须配看门狗，否则一步卡死整条线",
        "工步 {name}（调用 {sub}）不是程序指令，未写入——调用语法由品牌主程序层生成",
        "步骤序号 →（驻留按秒加宽；非实测节拍）",
        "{n} 个工步 ｜ 触发 {signal} ｜ 等待 {wait} ｜ 调用 {call} ｜ 驻留合计 {dwell}s",
    ):
        assert i18n.tr(text) != text, f"「{text}」缺英文译文"
