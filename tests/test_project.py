"""工程文件（``.core``）测试——v1.3.4。

覆盖三件事：
* **信封**：段读写、schema 校验、摘要与描述；
* **逐段容错与保真**：坏段只跳一段、未知段/未知顶层键原样写回（往返不丢内容）；
* **磁盘**：原子写入（临时文件不残留）、扩展名补齐、modified/created 口径、CLI。
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from service.project import (
    PROJECT_EXTENSION,
    PROJECT_EXTENSIONS,
    PROJECT_SCHEMA,
    SECTION_NAMES,
    LoadReport,
    Project,
    ProjectError,
    cell_from_snapshot,
    cell_snapshot,
    describe_section,
)


def _good_scene() -> dict:
    return {
        "format_version": 1,
        "robot_urdf": "assets/models/ur/ur5e/ur5e.urdf",
        "robot_joints_deg": [0.0] * 6,
        "objects": [{"name": "工件_1"}, {"name": "料箱"}],
    }


def _good_program() -> dict:
    return {
        "code": "# OLP Path Code\nmovej([0, -90, 90, -90, 90, 0], vel=0.5)\n",
        "point_types": ["approach", "pick", "place"],
        "joint_names": [f"joint_{i + 1}" for i in range(6)],
    }


def _good_cell() -> dict:
    return {
        "devices": [
            {
                "name": "机器人",
                "type": "机器人",
                "brand": "UR",
                "model": "UR5e",
                "comm": "Profinet",
                "ip": "192.168.1.10",
                "io_map": {"robot_start": "Q0.0", "robot_done": "I0.0"},
            },
            {"name": "夹具_1", "type": "夹具", "io_map": {"gripper_open": "Q0.1"}},
        ],
        "steps": [{"name": "step1", "action": "x", "wait": ""}],
    }


def _full_project() -> Project:
    p = Project(name="demo")
    p.set("scene", _good_scene())
    p.set("program", _good_program())
    p.set("cell", _good_cell())
    p.set(
        "sequence",
        {
            "schema": "service.sequence/1",
            "steps": [
                {
                    "kind": "signal",
                    "name": "s1",
                    "device": "机器人",
                    "signal": "robot_start",
                    "value": True,
                },
                {"kind": "dwell", "name": "d1", "device": "机器人", "seconds": 0.25},
                {"kind": "dwell", "name": "d2", "device": "机器人", "seconds": 0.5},
            ],
        },
    )
    p.set("parts", {"sources": ["examples/assets/custom_parts.json"]})
    return p


# --------------------------------------------------------------------------- #
# 信封与容器语义
# --------------------------------------------------------------------------- #


def test_schema_and_section_names():
    assert PROJECT_SCHEMA == "service.project/1"
    # 主扩展名 2026-10-12 由 .core 改为 .rwproj（格式不变，.core 退为兼容别名）
    assert PROJECT_EXTENSION == ".rwproj"
    assert PROJECT_EXTENSIONS == (".rwproj", ".robotproj", ".core")
    assert SECTION_NAMES == (
        "scene",
        "program",
        "programs",
        "sequence",
        "cell",
        "parts",
        "calibration",
    )


def test_set_get_has_remove():
    p = Project(name="x")
    assert len(p) == 0 and "scene" not in p
    p.set("scene", _good_scene())
    assert p.has("scene") and len(p) == 1 and "scene" in p
    assert p.get("scene")["objects"][0]["name"] == "工件_1"
    assert p.section_names == ["scene"]
    p.remove("scene")
    assert not p.has("scene") and p.section_names == []


def test_set_none_removes_section():
    p = Project()
    p.set("parts", {"sources": ["a.json"]})
    p.set("parts", None)
    assert not p.has("parts") and p.get("parts") is None


def test_set_unknown_section_raises():
    p = Project()
    with pytest.raises(ProjectError) as exc:
        p.set("viewport", {"x": 1})
    assert "viewport" in str(exc.value)
    assert "scene" in str(exc.value)  # 报错里给出可用段名


def test_set_rejects_nan_and_inf():
    """NaN/Inf 序列化出来是非法 JSON——在写入时就拦住，而不是等读的时候。"""
    p = Project()
    with pytest.raises(ProjectError):
        p.set("parts", {"sources": ["a.json"], "widths": [math.nan]})
    with pytest.raises(ProjectError):
        p.set("sequence", {"steps": [], "t": math.inf})
    assert p.section_names == []


def test_set_rejects_unserializable_object():
    p = Project()
    with pytest.raises(ProjectError):
        p.set("parts", {"sources": [object()]})


def test_sections_are_json_serializable():
    p = _full_project()
    text = json.dumps(p.to_dict(), ensure_ascii=False)
    assert json.loads(text)["schema"] == PROJECT_SCHEMA


# --------------------------------------------------------------------------- #
# from_dict：schema 与结构校验
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "bad",
    [
        [],
        "not-a-dict",
        42,
    ],
)
def test_from_dict_rejects_non_dict(bad):
    with pytest.raises(ProjectError):
        Project.from_dict(bad)


@pytest.mark.parametrize("schema", ["", "service.project/2", "core.scene/1"])
def test_from_dict_rejects_bad_schema(schema):
    with pytest.raises(ProjectError):
        Project.from_dict({"schema": schema, "sections": {}})


def test_from_dict_rejects_bad_sections_container():
    with pytest.raises(ProjectError):
        Project.from_dict({"schema": PROJECT_SCHEMA, "sections": [1, 2]})
    # sections 缺失按空工程处理（不是错误）
    p = Project.from_dict({"schema": PROJECT_SCHEMA})
    assert p.section_names == []


def test_from_dict_missing_meta_is_tolerated():
    p = Project.from_dict({"schema": PROJECT_SCHEMA, "sections": {"parts": {"sources": []}}})
    assert p.name == ""
    assert p.core_version  # 回填当前库版本
    assert p.has("parts")


# --------------------------------------------------------------------------- #
# 逐段容错 + 保真
# --------------------------------------------------------------------------- #


def test_one_bad_section_does_not_block_others():
    """核心承诺：坏一段只跳一段。"""
    data = {
        "schema": PROJECT_SCHEMA,
        "meta": {"name": "broken"},
        "sections": {
            "scene": _good_scene(),
            "sequence": {"steps": "不是数组"},
            "cell": _good_cell(),
        },
    }
    p = Project.from_dict(data)
    report = p.load_report
    assert report is not None
    assert sorted(report.loaded) == ["cell", "scene"]
    assert [name for name, _ in report.skipped] == ["sequence"]
    assert not report.ok()
    assert not p.has("sequence")
    assert p.get("sequence") is None
    # 被跳过的段原样保留
    assert p.raw("sequence") == {"steps": "不是数组"}
    assert p.section_names == ["scene", "cell", "sequence"]
    assert "工艺时序" in report.describe()


@pytest.mark.parametrize(
    "section,payload",
    [
        ("scene", "字符串不是对象"),
        ("program", {"point_types": ["pick"]}),
        ("program", {"code": 123}),
        ("sequence", {"steps": 3}),
        ("cell", {"devices": [{"type": "夹具"}]}),
        ("cell", {"devices": "x"}),
        ("parts", {"sources": [1, 2]}),
        ("calibration", {"schema": "service.calibration/9"}),
    ],
)
def test_section_shape_problems_are_skipped_with_reason(section, payload):
    p = Project.from_dict({"schema": PROJECT_SCHEMA, "sections": {section: payload}})
    assert not p.has(section)
    assert p.load_report.skipped and p.load_report.skipped[0][0] == section
    assert p.load_report.skipped[0][1]  # 有人话原因


def test_unknown_section_and_top_level_key_survive_round_trip(tmp_path):
    """本库不认识的段/顶层键必须原样写回（未来字段往返一趟不消失）。"""
    data = {
        "schema": PROJECT_SCHEMA,
        "meta": {"name": "future"},
        "sections": {
            "scene": _good_scene(),
            "vision": {"cameras": [{"id": "cam1", "fx": 900.0}]},
        },
        "authoring": {"tool": "handwritten", "priority": 3},
    }
    p = Project.from_dict(data)
    assert p.load_report.unknown == ["vision"]
    assert p.load_report.extra_keys == ["authoring"]

    out = tmp_path / "future.core"
    p.save(out)
    back = Project.load(out)
    assert back.raw("vision") == {"cameras": [{"id": "cam1", "fx": 900.0}]}
    assert back.extra["authoring"] == {"tool": "handwritten", "priority": 3}
    assert back.has("scene")

    # 二次保存依然保真（不是只有第一次能写回）
    again = tmp_path / "future2.core"
    back.save(again)
    raw = json.loads(again.read_text(encoding="utf-8"))
    assert raw["sections"]["vision"]["cameras"][0]["id"] == "cam1"
    assert raw["authoring"]["tool"] == "handwritten"


def test_setting_a_section_overrides_raw_kept_payload():
    """用户显式重写某段后，原样保留的那份必须让位。"""
    p = Project.from_dict(
        {
            "schema": PROJECT_SCHEMA,
            "sections": {"parts": {"sources": [1, 2]}},  # 坏段 → 原样保留
        }
    )
    assert p.raw("parts") == {"sources": [1, 2]}
    p.set("parts", {"sources": ["ok.json"]})
    assert p.raw("parts") is None
    assert p.to_dict()["sections"]["parts"] == {"sources": ["ok.json"]}


def test_version_mismatch_is_noted_not_migrated():
    p = Project.from_dict(
        {
            "schema": PROJECT_SCHEMA,
            "meta": {"core_version": "0.0.1"},
            "sections": {"parts": {"sources": []}},
        }
    )
    note = " ".join(p.load_report.notes)
    assert "0.0.1" in note and "未做版本迁移" in note


def test_same_version_has_no_note():
    p = _full_project()
    p.save(Path("/tmp") / "rw-version-noop.core")  # 不落盘也无妨：下面只查加载路径
    p2 = Project.from_dict(p.to_dict())
    assert p2.load_report.notes == []


# --------------------------------------------------------------------------- #
# 摘要与描述
# --------------------------------------------------------------------------- #


def test_summary_covers_every_present_section():
    p = _full_project()
    summary = p.summary()
    assert set(summary) == {"scene", "program", "sequence", "cell", "parts"}
    assert "2 个对象" in summary["scene"]
    assert "抓取 1 / 放置 1" in summary["program"]
    assert "3 个工步" in summary["sequence"]
    assert "0.75" in summary["sequence"]  # 驻留合计 0.25 + 0.5
    assert "2 台设备" in summary["cell"]
    assert "3 个已映射信号" in summary["cell"]
    assert "1 个零件库文件" in summary["parts"]


def test_describe_reports_missing_sections_honestly():
    p = _full_project()
    text = p.describe()
    assert "demo" in text
    assert "工程里没有：各机示教程序、标定集" in text
    assert "2 个对象" in text


def test_describe_empty_project():
    text = Project().describe()
    assert "未命名工程" in text
    assert "空工程" in text


def test_describe_raw_kept_section_marks_not_interpreted():
    p = Project.from_dict(
        {
            "schema": PROJECT_SCHEMA,
            "sections": {"parts": {"sources": "坏"}},
        }
    )
    assert "原样保留" in p.summary()["parts"]
    assert "原样保留" in p.describe()


@pytest.mark.parametrize(
    "section,payload,fragment",
    [
        ("scene", _good_scene(), "2 个对象"),
        ("scene", "坏", "格式异常"),
        ("program", _good_program(), "3 个示教点"),
        ("program", [], "格式异常"),
        ("sequence", {"steps": [{"kind": "dwell", "seconds": 1.5}]}, "1.5"),
        ("sequence", None, "格式异常"),
        ("cell", _good_cell(), "2 台设备"),
        ("cell", [], "格式异常"),
        ("parts", {"sources": ["a", "b"]}, "2 个零件库文件"),
        ("parts", [], "格式异常"),
        ("calibration", {"schema": "service.calibration/1", "tool": {"x": 1}}, "tool"),
        ("calibration", {"schema": "service.calibration/1"}, "内容为空"),
        ("calibration", [], "格式异常"),
    ],
)
def test_describe_section(section, payload, fragment):
    assert fragment in describe_section(section, payload)


def test_describe_section_unknown_name():
    assert "未知段" in describe_section("vision", {})


# --------------------------------------------------------------------------- #
# 磁盘往返
# --------------------------------------------------------------------------- #


def test_save_appends_extension_and_records_path(tmp_path):
    p = _full_project()
    target = p.save(tmp_path / "cell_a")
    assert target.name == "cell_a" + PROJECT_EXTENSION
    assert target.exists()
    assert p.path == target
    assert Project.load(target).path == target


def test_save_is_atomic_and_leaves_no_temp_files(tmp_path):
    p = _full_project()
    target = p.save(tmp_path / "atomic.core")
    entries = sorted(x.name for x in tmp_path.iterdir())
    assert entries == [target.name], entries


def test_save_overwrites_previous_content(tmp_path):
    target = tmp_path / "over.core"
    p = _full_project()
    p.save(target)
    p.set("parts", {"sources": ["b.json"]})
    p.save(target)
    back = Project.load(target)
    assert back.get("parts") == {"sources": ["b.json"]}


def test_save_creates_missing_directories(tmp_path):
    p = _full_project()
    target = p.save(tmp_path / "deep" / "nest" / "p.core")
    assert target.exists()


def test_save_rejects_empty_path():
    with pytest.raises(ProjectError):
        Project().save("")


def test_save_refreshes_modified_but_keeps_created(tmp_path):
    p = _full_project()
    p.created = "2000-01-01T00:00:00"
    p.modified = "2000-01-01T00:00:00"
    p.save(tmp_path / "t.core")
    back = Project.load(tmp_path / "t.core")
    assert back.created == "2000-01-01T00:00:00"
    assert back.modified != "2000-01-01T00:00:00"


def test_save_fills_name_from_filename(tmp_path):
    p = Project()
    p.set("parts", {"sources": []})
    p.save(tmp_path / "unnamed.core")
    assert p.name == "unnamed"
    assert Project.load(tmp_path / "unnamed.core").name == "unnamed"


def test_load_missing_file_and_bad_json(tmp_path):
    with pytest.raises(ProjectError):
        Project.load(tmp_path / "nope.core")
    bad = tmp_path / "bad.core"
    bad.write_text("{ not json", encoding="utf-8")
    with pytest.raises(ProjectError) as exc:
        Project.load(bad)
    assert "JSON" in str(exc.value)


def test_full_round_trip_preserves_every_section(tmp_path):
    p = _full_project()
    target = p.save(tmp_path / "full.core")
    back = Project.load(target)
    assert back.load_report.loaded == ["scene", "program", "sequence", "cell", "parts"]
    assert back.load_report.ok()
    assert back.get("scene") == _good_scene()
    assert back.get("program") == _good_program()
    assert back.get("cell") == _good_cell()
    assert back.get("sequence")["steps"][1]["seconds"] == 0.25
    assert back.get("parts") == {"sources": ["examples/assets/custom_parts.json"]}
    assert back.name == "demo"
    # 载入的工程再存一次应完全一致（除 modified 时间戳）
    first = json.loads(target.read_text(encoding="utf-8"))
    second_path = tmp_path / "full2.core"
    back.save(second_path)
    second = json.loads(second_path.read_text(encoding="utf-8"))
    first["meta"]["modified"] = second["meta"]["modified"]
    assert first == second


def test_load_report_dataclass_helpers():
    r = LoadReport()
    assert r.ok()  # 没跳过任何段 = ok
    assert not r.has_content()
    r2 = LoadReport(loaded=["scene"])
    assert r2.ok() and r2.has_content()
    assert "已载入 1 段" in r2.describe()
    assert r2.to_dict()["loaded"] == ["scene"]
    r3 = LoadReport(loaded=["scene"], skipped=[("parts", "坏")])
    assert not r3.ok()
    assert "跳过 1 段" in r3.describe()


# --------------------------------------------------------------------------- #
# 设备点表快照
# --------------------------------------------------------------------------- #


def test_cell_snapshot_round_trip():
    from service.cell import CellConfig, Device

    cell = CellConfig()
    cell.add(
        Device(
            name="机器人",
            type="机器人",
            brand="UR",
            model="UR5e",
            comm="Profinet",
            ip="10.0.0.2",
            io_map={"robot_start": "Q0.0"},
        )
    )
    cell.add(Device(name="夹具_1", type="夹具", io_map={"gripper_open": "Q0.1"}))
    cell.steps = [{"name": "s1", "action": "夹紧", "wait": "clamped"}]

    snap = cell_snapshot(cell)
    assert snap["devices"][0]["io_map"] == {"robot_start": "Q0.0"}
    assert describe_section("cell", snap).count("2 台设备") == 1

    back = cell_from_snapshot(snap)
    assert [d.name for d in back.devices] == ["机器人", "夹具_1"]
    assert back.find("夹具_1").io_map == {"gripper_open": "Q0.1"}
    assert back.find("机器人").ip == "10.0.0.2"
    assert back.steps == [{"name": "s1", "action": "夹紧", "wait": "clamped"}]
    assert cell_snapshot(back) == snap  # 快照幂等


def test_cell_snapshot_of_duck_typed_object():
    class FakeDevice:
        name = "传感器_1"
        type = "传感器"
        io_map = {"part_present": "I0.3"}

    class FakeCell:
        devices = [FakeDevice()]
        steps = []

    snap = cell_snapshot(FakeCell())
    assert snap["devices"][0]["name"] == "传感器_1"
    assert snap["devices"][0]["brand"] == ""  # 缺字段按空串
    assert cell_from_snapshot(snap).find("传感器_1").io_map == {"part_present": "I0.3"}


def test_cell_from_snapshot_rejects_bad_payload():
    with pytest.raises(ValueError):
        cell_from_snapshot({"devices": [{"type": "夹具"}]})
    with pytest.raises(ValueError):
        cell_from_snapshot("nope")


def test_real_sequence_and_calibration_sections_round_trip(tmp_path):
    """与既有属主类对接：ProcessSequence / CalibrationSet 的 to_dict 直接当段用。"""
    from service.calibration import CalibrationSet
    from service.sequence import DwellStep, ProcessSequence, SignalStep

    seq = ProcessSequence(
        [
            SignalStep(name="启动", device="机器人", signal="robot_start", value=True),
            DwellStep(name="保压", device="夹具_1", seconds=0.4),
        ]
    )
    calib = CalibrationSet()
    p = Project(name="对接")
    p.set("sequence", seq.to_dict())
    p.set("calibration", calib.to_dict())
    target = p.save(tmp_path / "linked.core")
    back = Project.load(target)

    assert ProcessSequence.from_dict(back.get("sequence")).dwell_total() == pytest.approx(0.4)
    assert back.calibration() is not None
    assert describe_section("cell", {"devices": []})  # 空设备段也是合法描述


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #


def _run_cli(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, "-m", "service.project", *args],
        cwd=str(REPO_ROOT),
        env={**__import__("os").environ, "PYTHONPATH": str(REPO_ROOT)},
        capture_output=True,
        text=True,
    )


def test_cli_prints_summary(tmp_path):
    target = _full_project().save(tmp_path / "cli.core")
    res = _run_cli(str(target))
    assert res.returncode == 0, res.stderr
    assert "工程：demo" in res.stdout
    assert "2 个对象" in res.stdout


def test_cli_json_output(tmp_path):
    target = _full_project().save(tmp_path / "cli2.core")
    res = _run_cli(str(target), "--json")
    assert res.returncode == 0, res.stderr
    payload = json.loads(res.stdout)
    assert payload["schema"] if "schema" in payload else True
    assert payload["name"] == "demo"
    assert set(payload["summary"]) == {"scene", "program", "sequence", "cell", "parts"}
    assert payload["missing"] == ["programs", "calibration"]


def test_cli_reports_unreadable_file(tmp_path):
    bad = tmp_path / "broken.core"
    bad.write_text("nope", encoding="utf-8")
    res = _run_cli(str(bad))
    assert res.returncode == 2
    assert "打不开" in res.stdout
