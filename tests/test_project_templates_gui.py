"""工程模板的 GUI 侧测试（v1.3.6，offscreen 真开窗口）。

* 文件菜单有「从模板新建工程…」；
* 直传模板 id（跳过对话框）→ 场景/示教程序/语义真的进界面，
  新工程按未保存对待、不进最近打开；
* 当前工程有未保存改动 → 先询问，拒绝则什么都不动；
* 选择对话框：取消返回 None，确定返回选中 id。

跑法::

    QT_QPA_PLATFORM=offscreen pytest tests/test_project_templates_gui.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
from PySide6 import QtWidgets

REPO_ROOT = Path(__file__).resolve().parents[1]
for _p in (REPO_ROOT, REPO_ROOT):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

UR5E = REPO_ROOT / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"

_POINTS = [
    [0, -90, 90, -90, 90, 0],
    [10, -80, 80, -90, 90, 0],
]


@pytest.fixture(scope="module")
def qapp():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture()
def window(qapp, tmp_path, monkeypatch):
    """带独立配置目录的主窗口（最近打开等用户级状态不串场）。"""
    import main as rw

    monkeypatch.setenv("CORE_CONFIG_DIR", str(tmp_path / "cfg"))
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
    monkeypatch.setattr(QtWidgets.QMessageBox, "exec", lambda self: QtWidgets.QMessageBox.Ok)
    for name in ("information", "warning", "critical"):
        monkeypatch.setattr(QtWidgets.QMessageBox, name, staticmethod(lambda *a, **k: None))
    return monkeypatch


def _menu_texts(window) -> list:
    """「文件」菜单的条目——标题已中文化，按当前语言的译文找。"""
    from support.i18n import tr

    wanted = tr("文件(&F)").replace("&", "")
    for action in window.menuBar().actions():
        if action.menu() is not None and action.text().replace("&", "") == wanted:
            return [a.text().replace("&", "") for a in action.menu().actions()]
    raise AssertionError(f"没有「{wanted}」菜单")


def test_menu_action_present_with_tooltip(window):
    texts = _menu_texts(window)
    assert any("从模板新建工程" in t for t in texts)
    for action in window.menuBar().actions():
        menu = action.menu()
        if menu is None:
            continue
        for sub in menu.actions():
            if "从模板新建工程" in sub.text():
                assert sub.toolTip()


def test_new_from_template_applies_scene_and_program(window):
    if not UR5E.exists():
        pytest.skip("内置 UR5e 模型缺失")
    assert window.new_project_from_template("grasping_workstation") is True

    # 场景真的进来了
    assert len(set(window.scene_panel.scene.objects)) >= 4
    # 示教程序与工艺语义真的进来了（与一键演示同一数据源）
    positions = window.code_editor.get_recorded_positions()
    assert len(positions) == 8
    values = [getattr(t, "value", t) for t in window.code_editor.get_point_types()]
    assert values.count("pick") == 1
    assert values.count("place") == 1
    # 新建的工程：未落盘 → 按未保存对待；不进最近打开
    assert window._project_dirty
    assert "未保存" in window.windowTitle()
    assert window.project.path is None
    from app.session import RecentProjects

    assert RecentProjects.load().paths == []


def test_dirty_project_asks_before_discarding(window, monkeypatch):
    # 先弄脏当前工程
    window.scene_panel.add_component("worktable")
    assert window._project_dirty

    answers = []

    def _fake_question(*a, **k):
        answers.append(1)
        return QtWidgets.QMessageBox.No

    monkeypatch.setattr(QtWidgets.QMessageBox, "question", staticmethod(_fake_question))
    assert window.new_project_from_template("sorting_station") is False
    assert answers == [1]  # 问过了，用户拒绝
    # 当前工程原样保留（没被模板覆盖）
    assert window.project.name == ""


def test_dirty_project_yes_discards_and_applies(window, monkeypatch):
    if not UR5E.exists():
        pytest.skip("内置 UR5e 模型缺失")
    window.scene_panel.add_component("worktable")
    monkeypatch.setattr(
        QtWidgets.QMessageBox,
        "question",
        staticmethod(lambda *a, **k: QtWidgets.QMessageBox.Yes),
    )
    assert window.new_project_from_template("grasping_workstation") is True
    assert window._project_dirty
    assert window.project.name == "抓取工作站"


def test_unknown_template_id_fails_honestly(window, silent_dialogs):
    assert window.new_project_from_template("nope") is False
    # 当前工程没有被破坏
    assert "未命名工程" in window.windowTitle()


def test_template_dialog_cancel_returns_none(window, monkeypatch):
    monkeypatch.setattr(
        QtWidgets.QDialog,
        "exec",
        lambda self: QtWidgets.QDialog.Rejected,
    )
    assert window._choose_project_template() is None


def test_template_dialog_ok_returns_selected_id(window, monkeypatch):
    def _accept(self):
        listw = self.findChildren(QtWidgets.QListWidget)[0]
        listw.setCurrentRow(1)  # 分拣工位
        return QtWidgets.QDialog.Accepted

    monkeypatch.setattr(QtWidgets.QDialog, "exec", _accept)
    assert window._choose_project_template() == "sorting_station"
