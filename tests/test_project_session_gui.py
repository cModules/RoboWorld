"""会话支撑的 GUI 侧（v1.3.5）：最近打开菜单 + 自动保存 + 恢复询问。

机制层见 ``tests/test_project_session.py``；这里验证界面真的接上了：

* 文件菜单有「最近打开」子菜单，保存工程后出现该条目；
* 点最近打开条目 = 打开工程（与对话框同一条路）；
* 文件已不存在的条目**置灰**而不是悄悄藏掉；
* 脏会话定时落自动保存快照（含原工程的未知段），干净时不写；
* 启动时存在快照 → 询问恢复：同意 → 会话回来且按未保存对待、
  快照路径不进最近打开、保存建议路径指回原工程；
* 干净退出清快照，脏退出保留。

跑法::

    QT_QPA_PLATFORM=offscreen pytest tests/test_project_session_gui.py -q
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

import orolp
from orolp.session import (
    RecentProjects,
    clear_autosave,
    has_autosave,
    read_autosave,
    write_autosave,
)


@pytest.fixture(scope="module")
def qapp():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture()
def cfg_dir(tmp_path, monkeypatch):
    base = tmp_path / "orolp-config"
    monkeypatch.setenv("OROLP_CONFIG_DIR", str(base))
    return base


@pytest.fixture()
def window(qapp, cfg_dir):
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
    monkeypatch.setattr(
        QtWidgets.QMessageBox, "exec",
        lambda self: QtWidgets.QMessageBox.Ok,
    )
    for name in ("information", "warning", "critical"):
        monkeypatch.setattr(
            QtWidgets.QMessageBox, name,
            staticmethod(lambda *a, **k: None),
        )
    return monkeypatch


def _recent_menu(window):
    for action in window.menuBar().actions():
        if action.menu() is None:
            continue
        for sub in action.menu().actions():
            if "最近打开" in sub.text():
                return sub.menu()
    return None


def _save_to(window, path, monkeypatch):
    monkeypatch.setattr(
        QtWidgets.QFileDialog, "getSaveFileName",
        staticmethod(lambda *a, **k: (str(path), "")),
    )
    window.save_project()


# --------------------------------------------------------------------------- #
# 最近打开菜单
# --------------------------------------------------------------------------- #


def test_recent_menu_exists_and_empty_state(window, cfg_dir):
    menu = _recent_menu(window)
    assert menu is not None, "文件菜单里没有「最近打开」子菜单"
    actions = [a.text() for a in menu.actions()]
    assert any("暂无记录" in t for t in actions)


def test_save_project_records_recent_and_menu_updates(window, cfg_dir, tmp_path, monkeypatch):
    target = tmp_path / "saved_cell.orolp"
    _save_to(window, target, monkeypatch)

    assert RecentProjects.load().paths == [str(target)]
    menu = _recent_menu(window)
    texts = [a.text() for a in menu.actions()]
    assert any(str(target) in t for t in texts)
    assert not any("暂无记录" in t for t in texts)


def test_click_recent_entry_opens_project(window, cfg_dir, tmp_path, monkeypatch):
    target = tmp_path / "via_menu.orolp"
    _save_to(window, target, monkeypatch)
    window.scene_panel.add_component("worktable")  # 让会话与文件不同，验证真载入

    menu = _recent_menu(window)
    entry = next(
        a for a in menu.actions() if str(target) in a.text() and a.isEnabled()
    )
    entry.trigger()
    assert window._project_name() == "via_menu"
    assert not window._project_dirty


def test_missing_recent_entry_is_disabled_not_hidden(window, cfg_dir, tmp_path, monkeypatch):
    target = tmp_path / "will_vanish.orolp"
    _save_to(window, target, monkeypatch)
    target.unlink()

    menu = _recent_menu(window)
    menu.aboutToShow.emit()  # 模拟用户展开菜单（此时才重建、置灰失效条目）
    entry = next(a for a in menu.actions() if str(target) in a.text())
    assert entry.isEnabled() is False  # 置灰，不悄悄藏


def test_clear_recent_entries(window, cfg_dir, tmp_path, monkeypatch):
    target = tmp_path / "to_clear.orolp"
    _save_to(window, target, monkeypatch)
    menu = _recent_menu(window)
    clear = next(a for a in menu.actions() if "清除最近打开记录" in a.text())
    clear.trigger()
    assert RecentProjects.load().paths == []
    assert any("暂无记录" in a.text() for a in _recent_menu(window).actions())


# --------------------------------------------------------------------------- #
# 自动保存
# --------------------------------------------------------------------------- #


def test_autosave_timer_exists_and_interval(window):
    assert window._autosave_timer is not None
    assert window._autosave_timer.isActive()
    assert window.AUTOSAVE_INTERVAL_MS == 60_000


def test_autosave_tick_only_when_dirty(window, cfg_dir):
    window._set_project_dirty(False)
    window._autosave_tick()
    assert has_autosave() is False  # 干净会话没什么可救

    window.scene_panel.add_component("worktable")  # sceneChanged → 置脏
    assert window._project_dirty
    window._autosave_tick()
    assert has_autosave() is True
    info = read_autosave()
    assert info.source_path == ""  # 未保存过的会话没有"源工程"

    # 快照本身就是合法工程：场景真的收进去了
    loaded = orolp.Project.load(info.snapshot)
    assert loaded.has("scene")
    assert "工作台" in json.dumps(loaded.get("scene"), ensure_ascii=False)


def test_autosave_snapshot_keeps_unknown_sections(window, cfg_dir, tmp_path, monkeypatch):
    import json as _json

    source = tmp_path / "with_future.orolp"
    data = {
        "schema": orolp.PROJECT_SCHEMA,
        "meta": {"name": "with-future"},
        "sections": {"vision": {"cameras": [{"id": "cam1"}]}},
    }
    source.write_text(_json.dumps(data, ensure_ascii=False), encoding="utf-8")
    monkeypatch.setattr(
        QtWidgets.QFileDialog, "getOpenFileName",
        staticmethod(lambda *a, **k: (str(source), "")),
    )
    window.open_project()
    window.scene_panel.add_component("worktable")  # 置脏
    window._autosave_tick()

    info = read_autosave()
    assert info.source_path == str(source)
    snapshot = orolp.Project.load(info.snapshot)
    assert snapshot.raw("vision") == {"cameras": [{"id": "cam1"}]}  # 未知段跟着走


def test_clean_close_clears_autosave_dirty_close_keeps(window, cfg_dir, qapp):
    import app as rw

    # 脏会话 → 快照在；关闭这个脏窗口 → 快照必须留着（这正是要恢复的东西）
    win = rw.MainWindow()
    win.scene_panel.add_component("worktable")
    win._autosave_tick()
    assert has_autosave() is True
    win.close()
    win.deleteLater()
    assert has_autosave() is True

    # 干净窗口关闭 → 快照清掉（没有未保存改动，留着只会误导下次启动）
    clean = rw.MainWindow()
    clean.close()
    clean.deleteLater()
    assert has_autosave() is False


def test_open_project_clears_autosave(window, cfg_dir, tmp_path, monkeypatch):
    target = tmp_path / "fresh.orolp"
    _save_to(window, target, monkeypatch)
    window.scene_panel.add_component("worktable")
    window._autosave_tick()
    assert has_autosave() is True

    monkeypatch.setattr(
        QtWidgets.QFileDialog, "getOpenFileName",
        staticmethod(lambda *a, **k: (str(target), "")),
    )
    window.open_project()
    assert has_autosave() is False  # 磁盘上有工程了，旧快照只会误导


# --------------------------------------------------------------------------- #
# 恢复询问
# --------------------------------------------------------------------------- #


def _make_autosave(tmp_path):
    project = orolp.Project(name="crashed-work")
    project.set("program", {
        "code": "# OLP Path Code\nmovej([5,-90,90,-90,90,0])\n",
        "point_types": ["via"],
        "joint_names": [],
    })
    source = tmp_path / "crashed-work.orolp"
    snapshot = write_autosave(project, source_path=str(source))
    return snapshot, source


def test_recovery_prompt_yes_restores_session(window, cfg_dir, tmp_path, monkeypatch):
    snapshot, source = _make_autosave(tmp_path)
    monkeypatch.setattr(
        QtWidgets.QMessageBox, "question",
        staticmethod(lambda *a, **k: QtWidgets.QMessageBox.Yes),
    )
    assert window.maybe_prompt_recovery() is True

    # 会话回来了
    assert "movej([5,-90,90,-90,90,0])" in window.code_editor.toPlainText()
    # 按未保存改动对待：标题有标记、自动保存继续守护
    assert window._project_dirty
    assert "未保存" in window.windowTitle()
    # 保存建议路径指回原工程，快照路径只当内部槽位
    assert window.project.path == source
    # 快照路径不进最近打开
    assert RecentProjects.load().paths == []
    # 快照仍保留（再次崩溃前用户可能还没保存）
    assert has_autosave() is True


def test_recovery_prompt_no_does_nothing(window, cfg_dir, tmp_path, monkeypatch):
    _make_autosave(tmp_path)
    monkeypatch.setattr(
        QtWidgets.QMessageBox, "question",
        staticmethod(lambda *a, **k: QtWidgets.QMessageBox.No),
    )
    assert window.maybe_prompt_recovery() is False
    assert "未命名工程" in window.windowTitle()
    assert not window._project_dirty
    assert has_autosave() is True  # 不恢复也不销毁证据，可手动打开快照


def test_recovery_prompt_only_once(window, cfg_dir, tmp_path, monkeypatch):
    _make_autosave(tmp_path)
    calls = []
    monkeypatch.setattr(
        QtWidgets.QMessageBox, "question",
        staticmethod(lambda *a, **k: calls.append(1) or QtWidgets.QMessageBox.No),
    )
    window.maybe_prompt_recovery()
    window.maybe_prompt_recovery()
    assert len(calls) == 1  # showEvent 可能触发多次，询问只弹一次


def test_no_snapshot_no_prompt(window, cfg_dir):
    clear_autosave()
    assert window.maybe_prompt_recovery() is False
