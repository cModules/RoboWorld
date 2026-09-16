"""工程文件在 GUI 里的往返（v1.3.4）。

库层的 ``tests/test_project.py`` 只管信封；这里管**界面这一侧真的存得住、
真的读得回**：

* 菜单里有「保存工程…/打开工程…」，窗口标题显示工程名与未保存标记；
* ``_collect_into`` 只收界面里真有的东西（空场景不写场景段）；
* 「保存 → 清空 → 打开」之后：场景对象、示教程序正文、**代码文本里没有的**
  抓取/放置语义、工艺时序、设备点表都回来了；
* 「打开—保存」一趟之后，本库不认识的段与字段**没有被抹掉**（保真承诺）；
* 坏段/找不到的零件库文件如实报出来（不静默丢）。

跑法::

    QT_QPA_PLATFORM=offscreen pytest tests/test_project_gui.py -q
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest
from PySide6 import QtWidgets

REPO_ROOT = Path(__file__).resolve().parents[1]
for _p in (REPO_ROOT, REPO_ROOT / "python"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

UR5E = REPO_ROOT / "models" / "ur" / "ur5e" / "ur5e.urdf"

#: 六个关节都能解析的示教点（度）
_POINTS = [
    [0, -90, 90, -90, 90, 0],
    [10, -80, 80, -90, 90, 0],
    [20, -70, 70, -90, 90, 0],
    [30, -60, 60, -90, 90, 0],
]


@pytest.fixture(scope="module")
def qapp():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture()
def window(qapp):
    import app as rw

    win = rw.MainWindow()
    yield win
    try:
        if hasattr(win, "program_panel"):
            win.program_panel.stop_playback()
    finally:
        win.close()
        win.deleteLater()


@pytest.fixture()
def silent_dialogs(monkeypatch):
    """把模态对话框换成静默替身，同时收集它们展示过的文本。"""
    shown: list = []
    monkeypatch.setattr(
        QtWidgets.QMessageBox, "exec",
        lambda self: (shown.append(self.text()), QtWidgets.QMessageBox.Ok)[1],
    )
    for name in ("information", "warning", "critical"):
        monkeypatch.setattr(
            QtWidgets.QMessageBox, name,
            staticmethod(lambda *a, **k: shown.append(" ".join(str(x) for x in a[2:]))),
        )
    return shown


def _teach(window, points=None, types=None):
    """在路径编辑器里灌入示教点（与手工示教同一条数据通路）。"""
    points = _POINTS if points is None else points
    code = "# OLP Path Code\n" + "".join(
        f"movej([{', '.join(str(v) for v in pt)}], vel=0.5)\n" for pt in points
    )
    window.code_editor.setPlainText(code)
    window.code_editor._parse_code_to_positions()
    window.code_editor.sync_point_types()
    if types is not None:
        from orolp.pickplace import PointType

        window.code_editor.point_types = [PointType(t) for t in types]
    window._sync_program_points()


# --------------------------------------------------------------------------- #
# 菜单 / 标题 / 置脏
# --------------------------------------------------------------------------- #


def _file_menu_actions(window) -> list:
    for action in window.menuBar().actions():
        if action.menu() is not None and action.text().replace("&", "") == "File":
            return [a.text().replace("&", "") for a in action.menu().actions()]
    raise AssertionError("没有 File 菜单")


def test_project_actions_present_with_tooltips(window):
    texts = " | ".join(_file_menu_actions(window))
    assert "打开工程" in texts
    assert "保存工程" in texts
    for action in _find_project_actions(window):
        assert action.toolTip()  # 每个入口都要说清"存/读的是哪几段"


def _find_project_actions(window):
    out = []
    for action in window.menuBar().actions():
        if action.menu() is None:
            continue
        for sub in action.menu().actions():
            if "工程" in sub.text():
                out.append(sub)
    return out


def test_title_shows_project_name_and_dirty_mark(window):
    assert "未命名工程" in window.windowTitle()
    window._set_project_dirty(True)
    assert "未保存" in window.windowTitle()
    window._set_project_dirty(False)
    assert "未保存" not in window.windowTitle()


def test_editing_marks_dirty_then_save_clears_it(window, tmp_path, silent_dialogs, monkeypatch):
    window._set_project_dirty(False)
    # 改场景 → 置脏（sceneChanged 通路）
    window.scene_panel.add_component("worktable")
    assert window._project_dirty
    window._set_project_dirty(False)

    # 改示教程序 → 置脏（textChanged 通路）
    _teach(window)
    assert window._project_dirty

    target = tmp_path / "dirty.orolp"
    monkeypatch.setattr(
        QtWidgets.QFileDialog, "getSaveFileName",
        staticmethod(lambda *a, **k: (str(target), "")),
    )
    window.save_project()
    assert target.exists()
    assert not window._project_dirty
    assert "未保存" not in window.windowTitle()


def test_project_name_from_saved_file(window, tmp_path, silent_dialogs, monkeypatch):
    target = tmp_path / "my_cell.orolp"
    monkeypatch.setattr(
        QtWidgets.QFileDialog, "getSaveFileName",
        staticmethod(lambda *a, **k: (str(target), "")),
    )
    window.save_project()
    assert window._project_name() == "my_cell"
    assert "my_cell" in window.windowTitle()


# --------------------------------------------------------------------------- #
# collect：只收界面里真有的东西
# --------------------------------------------------------------------------- #


def test_collect_only_includes_present_sections(window):
    import orolp

    project = orolp.Project()
    window._collect_into(project)
    # 空场景不写场景段——"工程里没有：场景"比"场景：0 个对象"诚实
    assert not project.has("scene")
    assert project.has("program")
    assert project.has("sequence")
    assert project.has("cell")
    assert project.get("parts") == {"sources": []}

    window.scene_panel.add_component("worktable")
    project2 = orolp.Project()
    window._collect_into(project2)
    assert project2.has("scene")
    assert "工作台" in json.dumps(project2.get("scene"), ensure_ascii=False)


# --------------------------------------------------------------------------- #
# 保存 → 打开：六段都回来了
# --------------------------------------------------------------------------- #


def _run_round_trip(window, path, silent_dialogs, monkeypatch):
    monkeypatch.setattr(
        QtWidgets.QFileDialog, "getSaveFileName",
        staticmethod(lambda *a, **k: (str(path), "")),
    )
    window.save_project()
    monkeypatch.setattr(
        QtWidgets.QFileDialog, "getOpenFileName",
        staticmethod(lambda *a, **k: (str(path), "")),
    )
    window.open_project()


def test_round_trip_restores_scene_program_sequence_cell(window, tmp_path, silent_dialogs, monkeypatch):
    win = window
    if not UR5E.exists():
        pytest.skip("内置 UR5e 模型缺失")
    win.load_robot(str(UR5E))
    win.scene_panel.add_component("worktable")
    win.scene_panel.add_component("workpiece_box")
    _teach(win, types=["pick", "place", "via", "place"])

    # 工艺时序：两个工步
    editor = win.scene_panel.sequence_editor
    editor.add_step("signal", signal="robot_start", value=True, device="机器人", name="启动")
    editor.add_step("dwell", seconds=0.4, device="机器人", name="保压")

    # 设备点表：给机器人映射一个信号
    win.code_editor.cell.robot().io_map["robot_start"] = "Q0.0"
    win.code_editor._refresh_device_table()

    target = tmp_path / "cell.orolp"
    _run_round_trip(win, target, silent_dialogs, monkeypatch)

    # 打开一个新的窗口（模拟"关掉再开"），载入同一工程
    import app as rw

    fresh = rw.MainWindow()
    try:
        monkeypatch.setattr(
            QtWidgets.QFileDialog, "getOpenFileName",
            staticmethod(lambda *a, **k: (str(target), "")),
        )
        fresh.open_project()
        # 场景
        names = set(fresh.scene_panel.scene.objects)
        assert len(names) >= 3
        # 示教程序正文 + 逐点关节值
        assert fresh.code_editor.get_recorded_positions()
        assert len(fresh.code_editor.get_recorded_positions()) == len(_POINTS)
        # **代码文本里没有的**工艺语义
        values = [getattr(t, "value", t) for t in fresh.code_editor.get_point_types()]
        assert values == ["pick", "place", "via", "place"]
        # 工艺时序
        steps = list(fresh.scene_panel.sequence_editor.sequence)
        assert [s.name for s in steps] == ["启动", "保压"]
        assert fresh.scene_panel.sequence_editor.sequence.dwell_total() == pytest.approx(0.4)
        # 设备点表
        assert fresh.code_editor.cell.robot().io_map.get("robot_start") == "Q0.0"
        assert not fresh._project_dirty
        assert "cell" in fresh.windowTitle()
    finally:
        fresh.program_panel.stop_playback()
        fresh.close()
        fresh.deleteLater()


def test_workpiece_semantics_survive_without_code_text(window, tmp_path, silent_dialogs, monkeypatch):
    """落盘的内容里真的带着 point_types（不是靠"没变过"糊过去）。"""
    _teach(window, types=["approach", "pick", "lift", "place"])
    target = tmp_path / "types.orolp"
    _run_round_trip(window, target, silent_dialogs, monkeypatch)
    raw = json.loads(target.read_text(encoding="utf-8"))
    assert raw["sections"]["program"]["point_types"] == ["approach", "pick", "lift", "place"]
    assert "pick" not in raw["sections"]["program"]["code"]  # 正文里确实没有语义信息


# --------------------------------------------------------------------------- #
# 保真：未知段 / 未知字段在 GUI 里也不能被抹掉
# --------------------------------------------------------------------------- #


def test_open_then_save_keeps_unknown_sections(window, tmp_path, silent_dialogs, monkeypatch):
    import orolp

    source = tmp_path / "future.orolp"
    data = {
        "schema": orolp.PROJECT_SCHEMA,
        "meta": {"name": "future-work", "orolp_version": orolp.__version__},
        "sections": {
            "scene": {"format_version": 1, "robot_urdf": "", "robot_joints_deg": [], "objects": []},
            "vision": {"cameras": [{"id": "cam1"}]},
        },
        "authoring": {"tool": "other-cad"},
    }
    source.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")

    monkeypatch.setattr(
        QtWidgets.QFileDialog, "getOpenFileName",
        staticmethod(lambda *a, **k: (str(source), "")),
    )
    window.open_project()
    assert window._project_name() == "future-work"

    # 在工程里改点东西（加个工作台）再保存
    window.scene_panel.add_component("worktable")
    target = tmp_path / "future-out.orolp"
    monkeypatch.setattr(
        QtWidgets.QFileDialog, "getSaveFileName",
        staticmethod(lambda *a, **k: (str(target), "")),
    )
    window.save_project()

    back = orolp.Project.load(target)
    assert back.raw("vision") == {"cameras": [{"id": "cam1"}]}
    assert back.extra["authoring"] == {"tool": "other-cad"}
    assert "工作台" in json.dumps(back.get("scene"), ensure_ascii=False)


def test_open_reports_skipped_and_missing_content(window, tmp_path, silent_dialogs, monkeypatch):
    import orolp

    source = tmp_path / "broken.orolp"
    data = {
        "schema": orolp.PROJECT_SCHEMA,
        "meta": {"name": "broken"},
        "sections": {
            "sequence": {"steps": "不是数组"},          # 坏段 → 应被跳过
            "parts": {"sources": ["/definitely/not/here.json"]},  # 找不到 → 应报出
        },
    }
    source.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
    monkeypatch.setattr(
        QtWidgets.QFileDialog, "getOpenFileName",
        staticmethod(lambda *a, **k: (str(source), "")),
    )
    window.open_project()

    text = "\n".join(silent_dialogs)
    assert "跳过 1 段" in text
    assert "工艺时序" in text
    assert "零件库文件找不到" in text
    assert "/definitely/not/here.json" in text
    # 坏段不阻塞其余部分：工程对象仍然可用，且原样保留了坏段
    assert window.project.raw("sequence") == {"steps": "不是数组"}


def test_open_reports_unreadable_file(window, tmp_path, silent_dialogs, monkeypatch):
    bad = tmp_path / "nope.orolp"
    bad.write_text("{", encoding="utf-8")
    monkeypatch.setattr(
        QtWidgets.QFileDialog, "getOpenFileName",
        staticmethod(lambda *a, **k: (str(bad), "")),
    )
    window.open_project()
    assert any("打开工程失败" in t or "JSON" in t for t in silent_dialogs)
    # 打开失败不应把当前工程置脏或改名
    assert "未命名工程" in window.windowTitle()


def test_save_failure_keeps_session_usable(window, tmp_path, silent_dialogs, monkeypatch):
    _teach(window)
    blocker = tmp_path / "blocker.txt"
    blocker.write_text("x", encoding="utf-8")
    # 目标父路径是个**文件** → 建目录必失败
    monkeypatch.setattr(
        QtWidgets.QFileDialog, "getSaveFileName",
        staticmethod(lambda *a, **k: (str(blocker / "x.orolp"), "")),
    )
    window.save_project()
    assert silent_dialogs, "写失败必须弹窗说明，不能静默"
    assert window._project_dirty  # 没存成，仍然是脏的


# --------------------------------------------------------------------------- #
# 零件库来源：可解析的相对路径
# --------------------------------------------------------------------------- #


def test_parts_sources_restore_from_project_dir(window, tmp_path, silent_dialogs, monkeypatch):
    import orolp

    parts = tmp_path / "parts.json"
    parts.write_text(json.dumps({
        "schema": "orolp.parts/1",
        "parts": [{
            "id": "flange", "name": "法兰盘", "category": "自定义",
            "description": "测试件",
            "primitives": [{"kind": "cylinder", "radius": 30.0, "height": 10.0,
                            "offset": [0, 0, 5], "rpy_deg": [0, 0, 0],
                            "color": [0.3, 0.5, 0.7]}],
        }],
    }, ensure_ascii=False), encoding="utf-8")

    if not (orolp.world.__dict__.get("components") or True):  # pragma: no cover
        pytest.skip("组件库不可用")

    project = orolp.Project(name="parts-demo")
    project.set("parts", {"sources": ["parts.json"]})  # 相对工程目录的路径
    source = tmp_path / "parts.orolp"
    project.save(source)

    monkeypatch.setattr(
        QtWidgets.QFileDialog, "getOpenFileName",
        staticmethod(lambda *a, **k: (str(source), "")),
    )
    window.open_project()

    from orolp.world.parts_library import CUSTOM_ID_PREFIX, COMPONENTS

    try:
        assert any(cid.startswith(CUSTOM_ID_PREFIX) for cid in COMPONENTS)
        # 来源被记进面板（下次保存工程时还在）
        assert str(parts) in window.library_panel.parts_sources
    finally:
        from orolp.world.parts_library import unregister_custom_parts

        unregister_custom_parts(
            [cid for cid in list(COMPONENTS) if cid.startswith(CUSTOM_ID_PREFIX)]
        )
