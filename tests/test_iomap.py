"""PLC 侧信号 ↔ 机器人 IO 点表（v1.1.6）的测试。

现场返工最常见的原因不是算法，而是**点表对不上**——机器人程序里
``SetDO("gripper_close")`` 而 PLC 侧这个词接在别处、两个信号塞进同一个
物理地址、两侧同时驱动同一个信号、机器人在等一个没人给的输入。
这些在各自一侧都"看着没问题"，只有同一张表上才看得出来。本文件覆盖：

* 地址解析/格式化/别名归并（``E``→``I``、``A``→``Q``，``X``/``Y`` **刻意不归并**）
* 从真实程序指令里提取信号用途（``SetDO`` / ``WaitDI`` / 工艺输出与反馈）
* 点表的行来源合并（握手 + 设备 ``io_map`` + 程序实际用法 + 额外信号）
* 编辑：``assign`` / ``unassign`` / ``auto_assign``（跳过已占用地址、输入输出分前缀）
* 校验：六类问题代码的 **severity 必须正确**（错误阻塞下装、警告不阻塞但要看见）
* JSON / YAML 往返的三种结果计数（``applied`` / ``unchanged`` / ``skipped``）
* SCL 出码随附 IO 点表注释块 + 未映射声明在 TIA 里就看得见
* GUI 通路（offscreen）：表格 → 点表回写、状态栏提示不再 AttributeError

运行::

    pytest tests/test_iomap.py -q
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))
sys.path.insert(0, str(REPO_ROOT))

from orolp.cell import HANDSHAKE_SIGNALS, CellConfig, Device  # noqa: E402
from orolp.i18n import tr  # noqa: E402
from orolp.iomap import (  # noqa: E402
    DEFAULT_DIRECTIONS,
    DIRECTIONS,
    DIRECTION_PLC_TO_ROBOT,
    DIRECTION_ROBOT_TO_PLC,
    DIRECTION_SENSOR_TO_PLC,
    PLC_INPUT_SIGNALS,
    PLC_OUTPUT_SIGNALS,
    SignalMap,
    address_problem,
    format_address,
    parse_address,
    used_signals,
    validate_cell_io,
)
from orolp.plc_scl import Step, default_steps, generate_scl  # noqa: E402
from orolp.process import WeldProcess  # noqa: E402
from orolp.program import ArcStart, Program, SprayOn, SetDO, WaitDI  # noqa: E402


def _cell(**devices) -> CellConfig:
    """按 ``名字=io_map`` 快速造单元；``cell`` 关键字额外给一个 PLC。"""
    c = CellConfig()
    for name, io_map in devices.items():
        type_ = "PLC" if name.lower().startswith("plc") else "夹具"
        c.add(Device(name=name, type=type_, io_map=dict(io_map)))
    return c


@pytest.fixture
def cell() -> CellConfig:
    return _cell(PLC1={"robot_start": "Q0.0", "robot_done": "I0.0"})


@pytest.fixture
def smap(cell) -> SignalMap:
    return SignalMap(cell)


# --------------------------------------------------------------------------- #
# 地址解析
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "text,area,byte,bit,db",
    [
        ("Q0.0", "Q", 0, 0, ""),
        ("I1.3", "I", 1, 3, ""),
        ("M10", "M", 10, None, ""),
        ("%I1.3", "I", 1, 3, ""),
        ("  Q2.7  ", "Q", 2, 7, ""),
        ("DB10.DBX0.0", "DBX", 0, 0, "DB10"),
        ("IB2", "IB", 2, None, ""),
        ("MW2", "MW", 2, None, ""),
    ],
)
def test_parse_address_recognizes_iec_style(text, area, byte, bit, db):
    parsed = parse_address(text)
    assert parsed is not None, text
    assert (parsed.area, parsed.byte, parsed.bit, parsed.db) == (area, byte, bit, db)


@pytest.mark.parametrize("text", ["", "   ", "socket:1", "open_gripper", "输出1"])
def test_parse_address_rejects_unknown_without_guessing(text):
    # 不认识的写法一律 None：宁可不说，也不假装认识厂商私有格式
    assert parse_address(text) is None


def test_parse_address_german_aliases_merge_into_iec_areas():
    # E/A 是西门子的德文写法，归一化后判重才准
    assert parse_address("E0.0").area == "I"
    assert parse_address("A0.0").area == "Q"
    assert parse_address("E0.0").key == parse_address("I0.0").key
    assert parse_address("A1.2").key == parse_address("%Q1.2").key


def test_parse_address_x_y_deliberately_not_aliased():
    """X/Y 是三菱等厂商的**位序号**写法，硬当成 I/Q 会造出假重复。

    判重宁可漏报也不误报：``X7`` 与 ``I7`` 必须是两个不同的点。
    """
    assert parse_address("X7").area == "X"
    assert parse_address("Y7").area == "Y"
    assert parse_address("X7").key != parse_address("I7").key
    assert parse_address("Y0.0").key != parse_address("Q0.0").key


def test_parse_address_key_is_case_insensitive():
    assert parse_address("q0.0").key == parse_address("Q0.0").key


@pytest.mark.parametrize(
    "index,expected",
    [(0, "Q0.0"), (7, "Q0.7"), (8, "Q1.0"), (15, "Q1.7"), (65, "Q8.1"), (-3, "Q0.0")],
)
def test_format_address_siemens_bit_layout(index, expected):
    assert format_address("Q", index) == expected


def test_format_address_input_prefix():
    assert format_address("I", 0) == "I0.0"
    assert format_address("I", 9) == "I1.1"


@pytest.mark.parametrize(
    "address,why",
    [
        ("", "地址为空"),
        ("   ", "地址为空"),
        ("Q0.0 Q0.1", "地址里有空白字符"),
        ("Q0.0,Q0.1", "地址里有逗号——一行只能写一个地址"),
        ("Q0.0，Q0.1", "地址里有逗号——一行只能写一个地址"),
        ("open_gripper", "地址里没有任何数字"),
    ],
)
def test_address_problem_only_flags_obviously_broken(address, why):
    assert address_problem(address) == tr(why)


@pytest.mark.parametrize("address", ["Q0.0", "DB10.DBX0.0", "%I1.3", "Y7", "socket:1"])
def test_address_problem_accepts_vendor_specific(address):
    # 本模块没能力判断厂商私有写法对不对——放过它们，用户才敢用
    assert address_problem(address) is None


# --------------------------------------------------------------------------- #
# 程序里的信号用途
# --------------------------------------------------------------------------- #


def test_used_signals_reads_setdo_and_waitdi():
    program = Program("P")
    program.setDO("gripper_close", True)
    program.waitDI("part_present")
    usage = used_signals(program)

    assert usage["gripper_close"].written and not usage["gripper_close"].read
    assert usage["part_present"].read and not usage["part_present"].written
    assert usage["gripper_close"].commands == ["SetDO"]
    assert usage["part_present"].commands == ["WaitDI"]


def test_used_signals_accepts_plain_item_list():
    # duck typing：ProgramBuilder 直传 / 尚未包成 Program 的 items 都要能收
    usage = used_signals([SetDO("a", True), WaitDI("b")])
    assert set(usage) == {"a", "b"}


def test_used_signals_extracts_process_outputs_and_feedback():
    program = Program("W")
    program.arc_start(WeldProcess(arc_start_signal="arc_on", arc_ok_signal="arc_ok"))
    usage = used_signals(program)

    assert usage["arc_on"].written
    assert usage["arc_ok"].read and not usage["arc_ok"].written
    assert usage["arc_on"].commands == ["ArcStart"]


def test_used_signals_merges_same_signal_across_commands():
    program = Program("P")
    program.setDO("sig", True)
    program.waitDI("sig")
    usage = used_signals(program)

    assert usage["sig"].written and usage["sig"].read
    assert set(usage["sig"].commands) == {"SetDO", "WaitDI"}


def test_used_signals_ignores_motion_commands():
    program = Program("P")
    program.setDO("io_sig", True)
    program.waitTime(0.2)
    assert set(used_signals(program)) == {"io_sig"}


def test_used_signals_extracts_spray_outputs():
    from orolp.process import SprayProcess

    program = Program("S")
    program.spray_on(SprayProcess(spray_on_signal="sp_on", spray_off_signal="sp_off"))
    usage = used_signals(program)

    assert usage["sp_on"].written and usage["sp_off"].written
    assert usage["sp_on"].commands == [SprayOn.__name__]


# --------------------------------------------------------------------------- #
# 点表视图
# --------------------------------------------------------------------------- #


def test_points_merge_handshake_device_and_program_signals(cell, smap):
    program = Program("P")
    program.setDO("ghost", True)
    points = smap.points(program, extra_signals=("manual_sig",))
    names = [p.signal for p in points]

    # 握手信号在最前（顺序稳定才好比对），随后设备键、程序用法、额外信号
    assert names[: len(HANDSHAKE_SIGNALS)] == list(HANDSHAKE_SIGNALS)
    assert "ghost" in names and "manual_sig" in names
    for name in ("robot_start", "robot_done", "ghost", "manual_sig"):
        assert names.count(name) == 1


def test_point_reports_owner_address_usage_and_direction(cell, smap):
    program = Program("P")
    program.setDO("robot_start", True)
    p = next(x for x in smap.points(program) if x.signal == "robot_start")

    assert p.mapped and p.address == "Q0.0"
    assert p.device == "PLC1"
    assert p.direction == DIRECTION_PLC_TO_ROBOT
    assert p.direction_text() == "PLC→机器人"
    assert p.usage_text() == tr("机器人写")
    assert p.to_row()["mapped"] == "1" and p.to_row()["written"] == "1"


def test_point_usage_text_covers_all_four_states(cell, smap):
    program = Program("P")
    program.setDO("both", True)
    program.waitDI("both")
    program.setDO("only_write", True)
    program.waitDI("only_read")
    by_signal = {p.signal: p for p in smap.points(program)}

    assert by_signal["both"].usage_text() == tr("机器人读写")
    assert by_signal["only_write"].usage_text() == tr("机器人写")
    assert by_signal["only_read"].usage_text() == tr("机器人读")
    assert by_signal["safety_ok"].usage_text() == tr("机器人未用")


def test_direction_of_known_and_unknown():
    cell = CellConfig()
    smap = SignalMap(cell)
    assert smap.direction_of("estop") == DIRECTION_SENSOR_TO_PLC
    assert smap.direction_of("at_home") == DIRECTION_ROBOT_TO_PLC
    assert smap.direction_of("从未见过的信号") == ""
    # 全部默认方向都必须是合法方向值
    assert set(DEFAULT_DIRECTIONS.values()) <= set(DIRECTIONS)


def test_address_of_and_multiple_owners(cell, smap):
    assert smap.address_of("robot_done") == "I0.0"
    assert smap.address_of("没有的信号") == ""

    cell.add(Device(name="夹具1", type="夹具", io_map={"robot_done": "I9.9"}))
    p = next(x for x in smap.points() if x.signal == "robot_done")
    assert p.devices == ["PLC1", "夹具1"]
    assert p.device == "PLC1"          # 多归属时取第一个，但 devices 全留
    assert p.address == "I0.0"          # 取第一个有值的


def test_signal_map_requires_cell():
    with pytest.raises(ValueError):
        SignalMap(None)


# --------------------------------------------------------------------------- #
# 编辑
# --------------------------------------------------------------------------- #


def test_assign_prefers_device_already_declaring_signal(cell, smap):
    cell.add(Device(name="夹具2", type="夹具", io_map={"clamped": "I0.4"}))
    smap.assign("clamped", "I0.5")
    assert cell.find("夹具2").io_map["clamped"] == "I0.5"
    assert "clamped" not in cell.find("PLC1").io_map


def test_assign_falls_back_to_plc_then_first_device(smap):
    assert smap.assign("brand_new", "Q1.0") is True
    assert smap.cell.plc().io_map["brand_new"] == "Q1.0"

    empty = SignalMap(CellConfig())
    assert empty.assign("x", "Q0.0") is False      # 没有任何设备可挂


def test_unassign_removes_only_targeted_device(smap):
    smap.cell.add(Device(name="夹具3", type="夹具", io_map={"robot_start": "Q7.7"}))
    assert smap.unassign("robot_start", "夹具3") is True
    assert smap.address_of("robot_start") == "Q0.0"   # PLC1 上的还在
    assert smap.unassign("从来没映射过") is False


def test_auto_assign_splits_input_and_output_prefixes(smap):
    # Q0.0 / I0.0 已被 robot_start / robot_done 占用 → 从下一位接着编
    got = smap.auto_assign("PLC1", ["gripper_close", "part_present", "safety_ok"])

    assert got["gripper_close"] == "Q0.1"    # PLC 驱动 → 输出侧
    assert got["part_present"] == "I0.1"     # 传感器 → 输入侧
    assert got["safety_ok"] == "I0.2"
    assert not any(addr.startswith("Q0.0") or addr == "I0.0" for addr in got.values())


def test_auto_assign_skips_addresses_already_taken(smap):
    # Q0.0 已被 robot_start 占用；自动编号绝不能把两个信号编到同一个点上
    got = smap.auto_assign("PLC1", ["gripper_close"])
    assert got["gripper_close"] == "Q0.1"
    assert len({p.address for p in smap.points() if p.address}) == len(
        [p for p in smap.points() if p.address]
    )


def test_auto_assign_leaves_mapped_signals_alone(smap):
    got = smap.auto_assign("PLC1", ["robot_start", "gripper_close"])
    assert "robot_start" not in got
    assert smap.address_of("robot_start") == "Q0.0"
    assert got["gripper_close"] == "Q0.1"


def test_auto_assign_uses_input_prefix_for_read_only_signal(cell):
    """程序里机器人只读的信号（既不是传感器方向也不是 PLC 输出）→ 输入侧。"""
    smap = SignalMap(cell)
    program = Program("P")
    program.waitDI("mystery_read")
    got = smap.auto_assign("PLC1", ["mystery_read"], program=program)
    assert got["mystery_read"] == "I0.1"     # I0.0 被 robot_done 占着，从下一位起


def test_auto_assign_default_when_signals_omitted_fills_unmapped(smap):
    got = smap.auto_assign("PLC1")
    # 握手信号里除 robot_start/robot_done 外都未映射
    assert set(got) == {"gripper_open", "gripper_close", "part_present", "safety_ok"}
    assert smap.validate().mapped_count >= 6


# --------------------------------------------------------------------------- #
# 校验
# --------------------------------------------------------------------------- #


def test_validate_flags_used_but_unmapped_as_error(cell):
    smap = SignalMap(cell)
    report = smap.validate([SetDO("ghost_close", True)])
    hit = next(i for i in report.errors if i.signal == "ghost_close")
    assert hit.code == "unmapped_signal"
    assert report.ok is False


def test_validate_unused_unmapped_is_only_warning(cell):
    smap = SignalMap(cell)
    report = smap.validate()
    codes = {i.code for i in report.warnings}
    assert "unmapped_signal" in codes
    assert report.errors == []
    assert report.ok is True


def test_validate_duplicate_address_is_error_and_mentions_the_other_signal(cell):
    cell.add(Device(name="夹具4", type="夹具", io_map={"gripper_close": "Q0.0"}))
    report = SignalMap(cell).validate()
    hit = next(i for i in report.errors if i.code == "duplicate_address")
    assert hit.signal == "gripper_close"
    assert "robot_start" in hit.message
    assert report.ok is False


def test_validate_duplicate_detects_german_alias(cell):
    cell.add(Device(name="夹具5", type="夹具", io_map={"gripper_close": "E0.0"}))
    # robot_done 是 I0.0；E0.0 就是 I0.0
    report = SignalMap(cell).validate()
    assert any(
        i.code == "duplicate_address" and i.signal == "gripper_close"
        for i in report.errors
    )


def test_validate_does_not_invent_duplicates_across_vendor_numbering(cell):
    # X/Y 是位序号写法，与 I/Q 不是同一套编号——不许报假重复
    cell.add(Device(name="夹具6", type="夹具", io_map={"gripper_close": "Y7"}))
    report = SignalMap(cell).validate()
    assert not any(i.code == "duplicate_address" for i in report.issues)


def test_validate_ambiguous_owner_is_warning(cell):
    cell.add(Device(name="夹具7", type="夹具", io_map={"robot_start": "Q5.0"}))
    report = SignalMap(cell).validate()
    hit = next(i for i in report.warnings if i.code == "ambiguous_owner")
    assert "PLC1" in hit.message and "夹具7" in hit.message
    assert report.ok is True   # 歧义是警告：不阻塞，但必须人工确认


def test_validate_address_format_warning(cell):
    cell.add(Device(name="夹具8", type="夹具", io_map={"clamped": "接在夹爪上"}))
    report = SignalMap(cell).validate()
    hit = next(i for i in report.warnings if i.code == "address_format")
    assert hit.signal == "clamped"
    assert report.ok is True


def test_validate_double_drive_warning(cell):
    """机器人 SetDO 与 PLC 输出同时驱动 → 谁后写谁生效（现场表现为偶发动作）。

    这不是臆造：``gripper_close`` 既在 ``PLC_OUTPUT_SIGNALS`` 里，抓取程序
    也会 ``SetDO`` 它——正是本模块要抓的那类真实冲突。
    """
    smap = SignalMap(cell)
    smap.assign("gripper_close", "Q0.2", "PLC1")
    report = smap.validate([SetDO("gripper_close", True)])
    hit = next(i for i in report.warnings if i.code == "double_drive")
    assert hit.signal == "gripper_close"


def test_validate_wait_without_source_warning(cell):
    cell.add(Device(name="夹具9", type="夹具", io_map={"mystery_read": "Q3.0"}))
    report = SignalMap(cell).validate([WaitDI("mystery_read")])
    hit = next(i for i in report.warnings if i.code == "wait_without_source")
    assert hit.signal == "mystery_read"


def test_validate_wait_with_sensor_source_is_quiet(cell):
    cell.add(Device(name="传感器1", type="传感器", io_map={"part_present": "I0.2"}))
    report = SignalMap(cell).validate([WaitDI("part_present")])
    assert not any(i.code == "wait_without_source" for i in report.issues)


def test_validate_steps_count_as_plc_side_drivers(cell):
    """工步里 PLC 驱动的信号不该再被说成"没人给"。"""
    cell.add(Device(name="夹具10", type="夹具", io_map={"mystery_read": "Q3.0"}))
    steps = [Step(name="给信号", outputs={"mystery_read": True}, wait="mystery_read")]
    report = SignalMap(cell).validate([WaitDI("mystery_read")], steps=steps)
    assert not any(i.code == "wait_without_source" for i in report.issues)


def test_report_summary_format_and_dict(cell):
    smap = SignalMap(cell)
    report = smap.validate([SetDO("ghost", True)])

    assert f"点表 {len(report.points)} 点" in report.summary()
    text = report.format()
    assert " | " in text          # 用竖线分隔而不是假对齐（中文双宽补不出对齐）
    assert "ghost" in text and "✗" in text
    data = report.to_dict()
    assert data["ok"] is False
    assert data["total"] == len(report.points)
    assert data["errors"][0]["code"] == "unmapped_signal"
    assert len(data["points"]) == len(report.points)


def test_report_format_without_issues():
    # 全部握手信号都映射到、且程序没用任何一个 → 无错误也无警告
    cell = _cell(PLC1={
        "robot_start": "Q0.0", "robot_done": "I0.0", "gripper_open": "Q0.1",
        "gripper_close": "Q0.2", "part_present": "I0.1", "safety_ok": "I0.2",
    })
    report = SignalMap(cell).validate()
    assert report.issues == []
    assert tr("点表未发现问题") in report.format()
    assert "✓" in report.format()


def test_validate_cell_io_is_a_shortcut(cell):
    report = validate_cell_io(cell, [SetDO("ghost", True)])
    assert [i.code for i in report.errors] == ["unmapped_signal"]


def test_number_of_handshake_defaults_matches_plc_declaration_lists():
    # 谁驱动谁只有一处定义：plc_scl 反向引用本模块
    from orolp import plc_scl

    assert tuple(plc_scl._OUTPUT_SIGNALS) == PLC_OUTPUT_SIGNALS
    assert tuple(plc_scl._INPUT_SIGNALS) == PLC_INPUT_SIGNALS


def test_default_steps_still_round_trip_through_validation():
    cell = _cell(PLC1={"robot_start": "Q0.0", "robot_done": "I0.0"})
    report = validate_cell_io(cell, None, default_steps())
    assert report.ok is True


# --------------------------------------------------------------------------- #
# 往返
# --------------------------------------------------------------------------- #


def test_json_round_trip_is_lossless(cell):
    smap = SignalMap(cell)
    smap.assign("gripper_close", "Q0.2", "PLC1")
    text = smap.to_json()

    target = _cell(PLC1={"robot_start": "Q0.0", "robot_done": "I0.0"})
    stats = SignalMap.from_json(target, text)
    assert stats["skipped"] == 0
    assert SignalMap(target).address_of("gripper_close") == "Q0.2"
    # 空地址 + 本来就不在任何设备上 → unchanged，不是失败
    assert stats["unchanged"] >= 1


def test_load_rows_distinguishes_applied_unchanged_and_skipped(smap):
    """三种结果必须分开报——静默丢点是最难查的一类问题。"""
    stats = smap.load_rows([
        {"signal": "gripper_close", "address": "Q0.2", "device": "PLC1"},   # applied
        {"signal": "part_present", "address": ""},                          # unchanged
        {"signal": "幽灵信号", "address": "Q9.9", "device": "不存在的设备"},   # skipped
        {"signal": "", "address": "Q1.1"},                                  # skipped
    ])
    assert stats["applied"] == 1
    assert stats["unchanged"] == 1
    assert stats["skipped"] == 2
    assert "幽灵信号" in stats["skipped_signals"]


def test_load_rows_can_clear_address(tmp_path, smap):
    stats = smap.load_rows([{"signal": "robot_start", "address": "", "device": "PLC1"}])
    assert stats["applied"] == 1
    assert smap.address_of("robot_start") == ""


def test_yaml_round_trip(tmp_path, cell):
    smap = SignalMap(cell)
    smap.assign("clamped", "I0.3", "PLC1")
    path = tmp_path / "io_map.yaml"
    smap.save_yaml(path, program=[WaitDI("clamped")])

    target = _cell(PLC1={"robot_start": "Q0.0", "robot_done": "I0.0"})
    SignalMap.load_yaml(target, path)
    assert SignalMap(target).address_of("clamped") == "I0.3"
    body = path.read_text(encoding="utf-8")
    assert "点表" in body or "points" in body   # allow_unicode：中文不转义
    assert "\\u" not in body


# --------------------------------------------------------------------------- #
# SCL 出码联动
# --------------------------------------------------------------------------- #


def test_scl_carries_io_point_table_comment_block(cell):
    code = generate_scl(cell, program=[SetDO("gripper_close", True)])
    assert "IO 点表（自动生成，下装前必须核对）" in code
    assert "信号 | 地址 | 方向 | 用途 | 归属" in code
    assert "点表 " in code and " 点：" in code


def test_scl_point_table_can_be_disabled(cell):
    code = generate_scl(cell, include_point_table=False)
    assert "IO 点表（自动生成" not in code
    assert 'FUNCTION_BLOCK "CellControl"' in code


def test_scl_point_table_reports_errors_inline(cell):
    code = generate_scl(cell, program=[SetDO("ghost_close", True)])
    # 未映射的点直接写在文件里，而不是只在某个界面里红一下
    assert ">>> 未映射" in code
    assert "!!! [ghost_close]" in code


def test_scl_unmapped_declaration_is_visible_in_var_block():
    cell = _cell(PLC1={"robot_start": "Q0.0"})
    code = generate_scl(cell)
    assert "// PLC1: Q0.0" in code
    assert "// >>> 未映射（下装前必须补地址）" in code


def test_scl_post_processor_passes_program_through(cell):
    from orolp.plc_scl import SCLPostProcessor

    code = SCLPostProcessor().generate(
        cell, metadata={"program_name": "CellA"}, items=[SetDO("ghost", True)]
    )
    assert 'FUNCTION_BLOCK "CellA"' in code
    assert "ghost" in code


# --------------------------------------------------------------------------- #
# GUI 通路（offscreen）
# --------------------------------------------------------------------------- #


@pytest.fixture(scope="module")
def qapp():
    from PySide6 import QtWidgets

    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture(scope="module")
def window(qapp):
    """整个模块共用一个主窗口：MainWindow 构造成本不低，而各用例都自建 cell。"""
    import app as rw

    win = rw.MainWindow()
    yield win
    win.close()
    win.deleteLater()


def test_path_code_editor_is_a_qwidget_without_status_bar(window):
    """回归：编辑器是普通 QWidget，**没有** statusBar。

    早期代码直接 ``self.statusBar()``，一点「生成 PLC」就 AttributeError。
    现在必须走 ``_notify``（有主窗口用主窗口的，没有则发信号）。
    """
    import app as rw

    assert not hasattr(rw.PathCodeEditor, "statusBar")
    assert hasattr(window.code_editor, "statusMessage")
    # 主窗口自己是有状态栏的，否则 _notify 就没有落脚处
    assert window.statusBar() is not None


def test_notify_falls_back_to_signal_without_main_window(qapp):
    import app as rw

    ed = rw.PathCodeEditor()
    seen = []
    ed.statusMessage.connect(seen.append)
    ed._notify("孤零零的提示")
    assert seen == ["孤零零的提示"]


def test_notify_uses_main_window_status_bar(window):
    window.code_editor._notify("来自编辑器的提示", 100)
    assert window.statusBar().currentMessage() == "来自编辑器的提示"


def test_hardware_tab_has_io_map_buttons(window):
    ed = window.code_editor
    assert ed.btn_io_map.text() == tr("IO 映射…")
    assert ed.btn_io_check.text() == tr("IO 校验")


def test_write_io_map_from_table_writes_and_clears(window):
    from PySide6 import QtWidgets

    ed = window.code_editor
    ed.cell = _cell(PLC1={"robot_start": "Q0.0"})
    smap = ed._signal_map()

    table = QtWidgets.QTableWidget(4, 4)
    for r, (sig, addr) in enumerate([
        ("gripper_close", "Q0.2"),   # 新映射
        ("robot_start", ""),         # 摘掉地址
        ("clamped", " I0.5 "),       # 两边带空白 → 必须 strip
    ]):
        table.setItem(r, 0, QtWidgets.QTableWidgetItem(sig))
        table.setItem(r, 1, QtWidgets.QTableWidgetItem(addr))
    table.setItem(3, 0, QtWidgets.QTableWidgetItem("  "))   # 空信号行

    handled = ed._write_io_map_from_table(table, "PLC1", smap)
    assert handled == 3                       # 第三行是空信号，不计
    assert smap.address_of("gripper_close") == "Q0.2"
    assert smap.address_of("robot_start") == ""
    assert smap.address_of("clamped") == "I0.5"


def test_check_io_map_silent_shows_summary_in_status_bar(window):
    ed = window.code_editor
    ed.cell = _cell(PLC1={"robot_start": "Q0.0"})
    report = ed._check_io_map(silent=True)

    assert report is not None and report.points
    assert window.statusBar().currentMessage() == report.summary()


def test_check_io_map_reports_used_but_unmapped_error(window, monkeypatch):
    ed = window.code_editor
    ed.cell = _cell(PLC1={"robot_start": "Q0.0"})
    monkeypatch.setattr(ed, "_current_program_items", lambda: [SetDO("ghost_close", True)])

    report = ed._check_io_map(silent=True)
    assert [i.signal for i in report.errors] == ["ghost_close"]


def test_generate_plc_emits_code_with_point_table(window, monkeypatch):
    ed = window.code_editor
    ed.cell = _cell(PLC1={"robot_start": "Q0.0"})
    monkeypatch.setattr(ed, "_current_program_items", lambda: [SetDO("ghost_close", True)])

    ed._generate_plc()
    text = ed.plc_preview.toPlainText()
    assert 'FUNCTION_BLOCK "CellControl"' in text
    assert "IO 点表（自动生成" in text
    assert "ghost_close" in text
    # 点表有错时状态栏必须当场说出来（不只在 SCL 注释里）
    assert "处错误" in window.statusBar().currentMessage()


def test_generate_plc_without_cell_says_so(qapp):
    import app as rw

    ed = rw.PathCodeEditor()
    ed.cell = None
    ed._generate_plc()
    assert ed.plc_preview.toPlainText() == tr("单元模型不可用")


def test_selected_device_tracks_table_selection(window):
    ed = window.code_editor
    ed.cell = _cell(PLC1={"robot_start": "Q0.0"})
    ed._refresh_device_table()

    ed.device_table.clearSelection()
    ed.device_table.setCurrentCell(-1, -1)
    assert ed._selected_device() is None

    ed.device_table.selectRow(0)
    assert ed._selected_device().name == "PLC1"


def test_refresh_device_table_shows_io_map_owner(window):
    ed = window.code_editor
    ed.cell = _cell(PLC1={"robot_start": "Q0.0"})
    ed._refresh_device_table()
    assert ed.device_table.item(0, 0).text() == "PLC1"
    assert ed.device_table.item(0, 1).text() == "PLC"
