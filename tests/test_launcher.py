"""启动编排（:mod:`app.launcher`）接线测试。

这里不跑 ``run_app`` 的事件循环（那会一直阻塞），而是验证它**依赖的每一个决策**：

* 工程分支为什么不能走构造参数——``showEvent`` 会载入默认机器人并询问崩溃
  恢复，工程若在之前载入会被整个盖掉（症状："双击打开了工程，看到的是空场景"）；
* URDF 分支保持历史路径（构造参数）；
* 后续进程转交过来的路径要**重新分类**，而不是无条件当工程打开。

跑法::

    QT_QPA_PLATFORM=offscreen pytest tests/test_launcher.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
from PySide6 import QtWidgets

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from app.launcher import (  # noqa: E402
    _build_window,
    _on_path_received,
    _open_request,
)
from app.startup_args import StartupKind, StartupRequest  # noqa: E402
from service.project import Project  # noqa: E402


@pytest.fixture(scope="module")
def qapp():
    """**任何会构造 QWidget 的用例都必须请求本 fixture**。

    没有 QApplication 就建 QWidget 不是抛 Python 异常，而是直接 abort——
    pytest 只报 ``Fatal Python error: Aborted``，栈停在构造那一行，看不出原因。
    """
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture()
def cfg_dir(tmp_path, monkeypatch):
    base = tmp_path / "core-config"
    monkeypatch.setenv("CORE_CONFIG_DIR", str(base))
    return base


@pytest.fixture()
def window(qapp, cfg_dir):
    import main as rw

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


def _make_project(tmp_path: Path, name: str = "cell") -> str:
    project = Project()
    project.name = name
    return str(project.save(tmp_path / f"{name}.rwproj"))


# --------------------------------------------------------------------------- #
# 建窗口：哪条分支走构造参数
# --------------------------------------------------------------------------- #


def test_project_request_does_not_use_the_urdf_constructor(qapp, tmp_path):
    """工程**不能**走构造参数：那会在 showEvent 载入默认机器人之前就被覆盖。"""
    request = StartupRequest(kind=StartupKind.PROJECT, path=Path(_make_project(tmp_path)))
    win = _build_window(request)
    try:
        assert win.urdf_path is None
    finally:
        win.close()
        win.deleteLater()


def test_urdf_request_keeps_the_historical_constructor(qapp, tmp_path):
    """URDF 保持历史行为：走 ``MainWindow(urdf_path)``。"""
    urdf = tmp_path / "ur5e.urdf"
    urdf.write_text("<robot/>", encoding="utf-8")
    win = _build_window(StartupRequest(kind=StartupKind.URDF, path=urdf))
    try:
        assert win.urdf_path == str(urdf)
    finally:
        win.close()
        win.deleteLater()


# --------------------------------------------------------------------------- #
# 窗口就绪后执行意图
# --------------------------------------------------------------------------- #


def test_open_request_loads_the_project(window, tmp_path, silent_dialogs):
    """工程分支落到 ``open_project_path``——工程载入的唯一入口。"""
    path = _make_project(tmp_path)
    _open_request(window, StartupRequest(kind=StartupKind.PROJECT, path=Path(path)))
    assert window.project is not None
    assert str(window.project.path) == path


def test_open_request_sends_urdf_to_load_robot(window, tmp_path, monkeypatch):
    """URDF 分支落到 ``load_robot``（不在这里真解析，只验调用）。"""
    urdf = tmp_path / "ur5e.urdf"
    urdf.write_text("<robot/>", encoding="utf-8")
    calls = []
    monkeypatch.setattr(window, "load_robot", lambda p: calls.append(p))
    _open_request(window, StartupRequest(kind=StartupKind.URDF, path=urdf))
    assert calls == [str(urdf)]


def test_open_request_ignores_an_empty_request(window, monkeypatch):
    """没有路径就是没有意图：不做任何事（正常双击 exe 启动走这条）。"""
    opened = []
    monkeypatch.setattr(window, "open_project_path", lambda *a, **k: opened.append(a))
    _open_request(window, StartupRequest(kind=StartupKind.NONE))
    assert opened == []


# --------------------------------------------------------------------------- #
# 后续进程转交过来的路径
# --------------------------------------------------------------------------- #


def test_forwarded_project_path_is_opened(window, tmp_path, silent_dialogs, monkeypatch):
    """第二个实例转交来的工程路径，主实例要真的打开它。"""
    path = _make_project(tmp_path)
    _on_path_received(window, path)
    assert window.project is not None
    assert str(window.project.path) == path


def test_forwarded_empty_payload_only_focuses(window, monkeypatch):
    """空载荷 = 对方只是又双击了 exe：把窗口提前台，不去打开 "" 这个工程。"""
    opened = []
    focused = []
    monkeypatch.setattr(window, "open_project_path", lambda *a, **k: opened.append(a))
    monkeypatch.setattr(window, "raise_", lambda: focused.append(True))
    monkeypatch.setattr(window, "activateWindow", lambda: None)

    _on_path_received(window, "")
    assert opened == []
    assert focused == [True]


def test_forwarded_urdf_path_is_reclassified(window, tmp_path, monkeypatch):
    """转交来的路径要重新分类：URDF 不能被当成工程打开。"""
    urdf = tmp_path / "ur5e.urdf"
    urdf.write_text("<robot/>", encoding="utf-8")
    calls = []
    monkeypatch.setattr(window, "load_robot", lambda p: calls.append(p))
    monkeypatch.setattr(window, "open_project_path", lambda *a, **k: calls.append("project"))

    _on_path_received(window, str(urdf))
    assert calls == [str(urdf)]
