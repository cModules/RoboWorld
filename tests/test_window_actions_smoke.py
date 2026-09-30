"""窗口级菜单动作的冒烟测试（补 D1 覆盖缺口）。

背景：``MainWindow`` 的菜单动作（校验程序 / 生成代码 / 存取路径代码 / 干涉
检查 / 抓取向导 / 清示教点 / 复位视图）此前**没有任何直接测试**——
``tests/test_trajectory_editor*.py`` 里的 ``panel.validate_program()`` 测的是
``ProgramPanel`` **自己的**同名方法，与窗口那层无关。搬移本身有结构测试兜底
（归属模块 / MRO / 门禁登记），但"点这个菜单会不会炸"没人守。

策略：**不构造完整 MainWindow**——它要建 ``GLWidget``、吃 OpenGL 上下文，在
headless CI 上脆弱。改为用**轻量宿主**调用 mixin 的 unbound 方法：``self`` 只
提供被测方法真正碰到的那几个属性。这样可覆盖两条最易腐的路径：

* **降级分支**：面板不存在时必须"弹个提示然后安全返回"，而不是抛 ``AttributeError``；
* **按名归属**：多机下每个动作都必须带**机器人名**，否则就是"动 A 的数据动到 B"。

对话框一律拦截 ``exec`` / ``QMessageBox``——测试不进事件循环，也不真的弹窗。
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

from PySide6 import QtWidgets  # noqa: E402

from app.window.misc_actions import MiscActionsMixin  # noqa: E402
from app.window.program_ops import ProgramOpsMixin  # noqa: E402


@pytest.fixture(scope="module")
def qt_app():
    """offscreen 应用：构造 QWidget / QDialog 需要它，但不进事件循环。"""
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield app


class _Host(QtWidgets.QWidget):
    """轻量宿主：真 QWidget（对话框要 parent）+ 按需挂 mock 属性。"""


def _info_calls(monkeypatch):
    """拦截信息弹窗，返回记录列表（顺便避免真的弹窗阻塞）。"""
    calls: list = []
    monkeypatch.setattr(
        QtWidgets.QMessageBox,
        "information",
        staticmethod(lambda *a, **k: calls.append(a[1:] if len(a) > 1 else a)),
    )
    return calls


class _FakeReport:
    def __init__(self, ok, errors=(), warnings=(), cycle_time=None):
        self.ok = ok
        self.errors = list(errors)
        self.warnings = list(warnings)
        self.cycle_time = cycle_time


# --------------------------------------------------------------------------- #
# 降级分支：面板不存在时"弹提示 + 安全返回"，绝不抛 AttributeError
# --------------------------------------------------------------------------- #
def test_validate_program_without_panel_prompts_instead_of_crashing(qt_app, monkeypatch):
    """F7：离线编程面板不可用时要给一句话，而不是炸在属性访问上。"""
    calls = _info_calls(monkeypatch)
    ProgramOpsMixin.validate_program(_Host())  # 宿主没有 program_panel
    assert len(calls) == 1, "应提示一次「离线编程模块不可用」"


def test_open_grasp_wizard_without_panel_prompts_instead_of_crashing(qt_app, monkeypatch):
    calls = _info_calls(monkeypatch)
    MiscActionsMixin.open_grasp_wizard(_Host())
    assert len(calls) == 1, "应提示一次「离线编程模块不可用」"


def test_check_collisions_without_scene_panel_prompts_instead_of_crashing(qt_app, monkeypatch):
    """F6：环境建模不可用时的降级（历史写法是 `_HAS_WORLD and hasattr(...)`）。"""
    calls = _info_calls(monkeypatch)
    MiscActionsMixin.check_collisions(_Host())
    assert len(calls) == 1, "应提示一次「环境建模模块不可用」"


def test_generate_robot_code_without_taught_points_prompts(qt_app, monkeypatch):
    """出码：没有示教点时必须明确提示，不做静默降级（AGENTS.md #11）。"""
    calls = _info_calls(monkeypatch)
    host = _Host()
    host.code_editor = type("E", (), {"get_recorded_positions": lambda self: []})()
    ProgramOpsMixin.generate_robot_code(host)
    assert len(calls) == 1, "无示教点应弹一次提示"


# --------------------------------------------------------------------------- #
# 按名归属：多机下动谁的数据必须指名道姓
# --------------------------------------------------------------------------- #
def test_clear_requested_clears_only_the_current_robot(qt_app):
    """清示教点：传的是**当前这台**的名字——整机清空会抹掉别人教的点。"""
    host = _Host()
    cleared = {}
    host.gl_widget = type(
        "G",
        (),
        {
            "current_robot": "B",
            "clear_recorded_positions": lambda self, key: cleared.setdefault("gl", key),
            "update": lambda self: None,
        },
    )()
    editor_calls = {}
    host.code_editor = type(
        "E",
        (),
        {
            "set_recorded_positions": lambda self, pts, key: editor_calls.setdefault(
                "ed", (pts, key)
            ),
            "pos_count_label": type("L", (), {"setText": lambda self, s: None})(),
            "_update_code_with_positions": lambda self: None,
        },
    )()
    host._sync_program_points = lambda key=None: editor_calls.setdefault("sync", key)
    host.statusBar = lambda: type("S", (), {"showMessage": lambda self, s: None})()

    MiscActionsMixin.on_clear_requested(host)

    assert cleared["gl"] == "B", "视口必须按当前机器人名清，不能整机清空"
    assert editor_calls["ed"] == ([], "B"), "编辑器也要按名回写空表"
    assert editor_calls["sync"] == "B", "同步给编程面板的也应是同一台"


def test_reset_view_restores_the_default_camera(qt_app):
    """复位视图：三个相机参数回到固定默认值（历史上一改就是几处不一致）。"""
    host = _Host()
    host.gl_widget = type("G", (), {"camera_dist": 99, "camera_az": 0, "camera_el": 0})()
    host.gl_widget.update = lambda: None
    MiscActionsMixin.reset_view(host)
    assert (host.gl_widget.camera_dist, host.gl_widget.camera_az, host.gl_widget.camera_el) == (
        1.2,
        45.0,
        30.0,
    )


# --------------------------------------------------------------------------- #
# 汇总对话框：三种报告分支（通过 / 不通过 / 没程序）都要渲染成一行
# --------------------------------------------------------------------------- #
def test_validation_summary_renders_one_row_per_robot(qt_app, monkeypatch):
    """Shift+F7 汇总：三台机器人三种结论，表格要 3 行 × 5 列且不抛异常。

    这条同时是"对话框别把测试卡住"的护栏——``dlg.exec()`` 被拦成了 no-op。
    """
    monkeypatch.setattr(QtWidgets.QDialog, "exec", lambda self: 0)
    cells: list = []
    monkeypatch.setattr(
        QtWidgets.QTableWidget,
        "setItem",
        lambda self, row, col, item: cells.append((row, col)),
    )
    monkeypatch.setattr(QtWidgets.QTableWidget, "resizeColumnsToContents", lambda self: None)

    host = _Host()
    reports = {
        "R1": _FakeReport(ok=True, cycle_time=6.5),
        "R2": _FakeReport(ok=False, errors=["e"], warnings=["w"]),
        "R3": None,  # 没程序（未示教）
    }
    ProgramOpsMixin._show_validation_summary(host, reports)

    assert len(cells) == 15, f"3 台 × 5 列 = 15 个单元格，实际 {len(cells)}"
    assert {r for r, _ in cells} == {0, 1, 2}, "每台机器人都应占一行"


def test_validation_summary_with_no_reports_returns_without_a_dialog(qt_app, monkeypatch):
    """空输入直接返回：不该为一个空汇总弹个空对话框。"""
    dialogs = []
    monkeypatch.setattr(QtWidgets.QDialog, "__init__", lambda self, *a, **k: dialogs.append(self))
    ProgramOpsMixin._show_validation_summary(_Host(), {})
    assert not dialogs, "没有报告时不该构造对话框"
