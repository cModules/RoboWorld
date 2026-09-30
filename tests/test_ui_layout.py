"""界面架构改版（``docs/prd/ui-architecture.md``）的验收标准守卫。

PRD §6 的六条验收标准在这里变成可执行的断言——它此前只写在文档里，没有任何
测试盯着，于是"左侧双 Tab / 右侧三标签 / 小窗口不裁切"这些结构性约束一旦被
后续改动破坏（例如某个面板把内容直接挂在标签页上、不再套滚动容器），全量测试
依然是绿的。

覆盖：
* 左侧双 Tab「场景设备库 / 代码预览」、右侧三标签「属性 / 设备树 / 导出打包」
  （2026-09-19 换序：输入区集中到左手——挑设备与改代码都紧贴视口；
  结构（设备树）跟属性同侧，树上选中、当场改位姿）
* 旧标签（「场景」「动作与从属」「资源库」）不再存在
* 代码预览内部的四个子标签仍在（改版只搬位置，不删能力）
* 任一标签页在窗口缩到最小时内容不被裁切（内容更高的部分由滚动容器承接）
* 一键抓取工作站演示之后关键按钮可用（不存在"永远点不亮"的置灰按钮）
* 标签名与窗口标题在 en 下的表现（顺带守住版本口径）

跑法::

    QT_QPA_PLATFORM=offscreen pytest tests/test_ui_layout.py -q
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import pytest
from PySide6 import QtWidgets

REPO_ROOT = Path(__file__).resolve().parents[1]
for _p in (REPO_ROOT, REPO_ROOT):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

#: 改版后左侧标签（顺序即界面顺序）：设备库挑设备 + 代码预览改点位，都贴着视口
LEFT_TABS = ["场景设备库", "代码预览"]
#: 改版后右侧标签（顺序即界面顺序）：属性与设备树同侧，编排层级与改参数不分家
RIGHT_TABS = ["属性", "设备树", "导出打包"]
#: 改版前的旧标签名，任何一个再出现都说明回退了
LEGACY_TABS = ["场景", "动作与从属", "资源库", "机器人库"]
#: 代码预览内部的子标签（搬位置时最容易丢的东西）。
#: 这里跑在全功能档下（features 对 pytest 的默认）——MVP 档会隐藏「硬件列表」，
#: 那是 features 的事；这里守的是"能力还在，只是可能被藏起来"。
CODE_SUBTABS = ["路径示教", "硬件列表", "PLC 程序逻辑", "机器人子程序"]


@pytest.fixture(scope="module")
def qapp():
    from PySide6 import QtWidgets

    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture()
def window(qapp):
    import main as rw

    win = rw.MainWindow()
    yield win
    try:
        if hasattr(win, "program_panel"):
            win.program_panel.stop_playback()
    finally:
        win.close()
        win.deleteLater()


def _titles(tab_widget) -> list:
    return [tab_widget.tabText(i) for i in range(tab_widget.count())]


# --------------------------------------------------------------------------- #
# 标签结构
# --------------------------------------------------------------------------- #


def test_left_tabs_are_device_library_and_device_tree(window):
    assert _titles(window.left_tabs) == LEFT_TABS


def test_right_tabs_are_property_code_and_export(window):
    assert _titles(window.right_tabs) == RIGHT_TABS


def test_legacy_tabs_are_gone(window):
    present = set(_titles(window.left_tabs)) | set(_titles(window.right_tabs))
    assert not (present & set(LEGACY_TABS)), f"旧标签又出现了: {present & set(LEGACY_TABS)}"


def test_code_preview_subtabs_intact(window):
    """改版只把「路径代码」搬进左侧「代码预览」，子能力一个都不能少。"""
    assert _titles(window.code_editor.sub_tabs) == CODE_SUBTABS


def test_side_panels_do_not_squeeze_the_viewport(window):
    """左右两侧都必须有宽度上限，否则窄窗口下 3D 视口被压没。"""
    assert window.left_tabs.maximumWidth() <= 400
    assert window.right_tabs.maximumWidth() <= 360
    assert window.gl_widget.minimumSize().width() >= 300


# --------------------------------------------------------------------------- #
# 小窗口不裁切（PRD §6 第 5 条）
# --------------------------------------------------------------------------- #


def _pages(window):
    for label, tabs in (("左", window.left_tabs), ("右", window.right_tabs)):
        for i in range(tabs.count()):
            yield f"{label}/{tabs.tabText(i)}", tabs.widget(i)


def test_no_tab_page_is_clipped_at_minimum_window(window):
    """把窗口缩到它能到达的最小高度，逐页断言"内容不需要比可视区更高"。

    修复前实测：`RobotLibraryPanel`（场景设备库）最小内容高度 **286px**，而最小
    窗口下标签页可视高度只有 **267px** ⇒ 底部的「刷新」与拖拽提示被裁掉；套上
    滚动容器后降到 **88px**。这条断言就是那次修复的守卫——把滚动容器摘掉即红。
    """
    window.show()
    window.resize(900, window.minimumSizeHint().height())
    QtWidgets.QApplication.processEvents()

    clipped = []
    too_tall = []
    for name, page in _pages(window):
        need = page.minimumSizeHint().height()
        have = page.height()
        if need > have:
            clipped.append(f"{name}: 需要 {need}px > 可视 {have}px")
        # 内容本身也不该太"硬"：真硬到 300px 以上就该改成滚动容器了
        if need > 300:
            too_tall.append(f"{name}: {need}px")

    assert not clipped, "标签页内容在小窗口下被裁切: " + "; ".join(clipped)
    assert not too_tall, "标签页最小内容高度过大（应改用滚动容器）: " + "; ".join(too_tall)


def test_library_panel_has_scroll_container(window):
    from PySide6.QtWidgets import QScrollArea

    panel = window.library_panel
    scrolls = panel.findChildren(QScrollArea)
    assert len(scrolls) == 1, "场景设备库应且只应有一层滚动容器"
    assert panel.minimumSizeHint().height() <= 160
    assert panel.list_widget.minimumHeight() >= 120  # 列表有下限，筛选行才不会被压扁


def test_scene_panel_has_scroll_container(window):
    from PySide6.QtWidgets import QScrollArea

    assert window.scene_panel.findChildren(QScrollArea)


def test_program_panel_has_scroll_container(window):
    from PySide6.QtWidgets import QScrollArea

    assert window.program_panel.findChildren(QScrollArea)


# --------------------------------------------------------------------------- #
# 按钮可达性（PRD §6 第 6 条）
# --------------------------------------------------------------------------- #


def test_one_click_demo_keeps_buttons_live(window, monkeypatch):
    """一键展示后关键按钮必须全部可用。

    PRD 第 6 条要的是"无置灰不可用的按钮"——它们的**正确形态**是随状态启用，
    而不是永远灰着。这里用最重的路径（一键抓取工作站：建场景 → IK 示教 → 校验
    → 回放）验证按钮真的会亮，而不是只看初始态。
    """
    from PySide6 import QtWidgets

    # 失败路径会弹模态框，离屏测试里必须挡住，否则整轮挂住
    monkeypatch.setattr(QtWidgets.QMessageBox, "exec", lambda self: QtWidgets.QMessageBox.Ok)
    monkeypatch.setattr(QtWidgets.QMessageBox, "information", staticmethod(lambda *a, **k: None))
    monkeypatch.setattr(QtWidgets.QMessageBox, "warning", staticmethod(lambda *a, **k: None))
    monkeypatch.setattr(QtWidgets.QMessageBox, "critical", staticmethod(lambda *a, **k: None))

    window.load_grasping_workstation_demo()

    assert len(window.scene_panel.scene) >= 5, "抓取工作站模板没进场景"
    panel = window.program_panel
    assert panel.point_list.count() >= 5, "示例示教点没进列表"
    for btn, label in (
        (panel.btn_validate_all, "校验全部"),
        (panel.btn_generate, "生成代码"),
        (panel.btn_export_all, "导出全部"),
    ):
        assert btn.isEnabled(), f"「{label}」按钮在演示就绪后仍不可用"
    panel.stop_playback()


# --------------------------------------------------------------------------- #
# i18n / 版本口径
# --------------------------------------------------------------------------- #


def test_tab_titles_have_english_translations(monkeypatch):
    from support import i18n

    monkeypatch.setattr(i18n, "_LANGUAGE", "en", raising=False)
    for text in LEFT_TABS + RIGHT_TABS:
        assert i18n.tr(text) != text, f"「{text}」缺英文译文"


def test_window_title_carries_library_version(window):
    from support import __version__

    assert __version__ in window.windowTitle()


# --------------------------------------------------------------------------- #
# 界面文案：全中文（品牌/型号/技术词除外）+ 能塞进侧栏
# --------------------------------------------------------------------------- #

#: 允许留在中文界面上的"技术词/品牌型号"：机器人型号名、指令名、单位、快捷键
_TECH_OK = re.compile(
    r"^(move[jlJC]?|F\d+|Shift\+F\d+|Ctrl\+[A-Z]|"
    r"[A-Za-z0-9_.]+\.(script|src|mod|tp|py|txt|urdf|json)|"
    r"(mm|kg|s|%|N·m)|(UR|KUKA|ABB|FANUC|JAKA|URScript|TCP|DOF|IO|IP|PLC|SCL)|"
    r"[A-Z]{2,}[-_/][A-Z0-9./-]*|(Zu|KR|IRB|ER|M-|R-|LBR|iiwa)\b)"
)

#: 文件路径、版本号这类"数据"不是界面文案
_DATA_OK = re.compile(r"^(/|[A-Za-z]:\\\\|\d+\s+/|v?\d+(\.\d+)*$)")

#: 型号代号：字母开头、带数字、无汉字（Zu 3 / KR 6 R900 / ER20-1780 / IRB 4600-20/250…）
_MODEL_OK = re.compile(r"^[A-Za-z][A-Za-z0-9 ./\-_]*\d[A-Za-z0-9 ./\-_]*$")


def test_ui_text_is_chinese(window):
    """界面上不该再有"纯英文"的可见文案（中英文混用，2026-09-21 用户提出）。

    产品型号（Zu 3 / KR 6 R900…）、指令名（movej）、单位（mm）、快捷键、文件
    路径这些是**数据或专有名词**，不算混用，放行。
    """
    offenders = []
    for kid in window.findChildren(QtWidgets.QWidget):
        getter = getattr(kid, "text", None)
        if getter is None:
            continue
        try:
            text = (getter() or "").replace("&", "").strip()
        except Exception:
            continue
        if not text or _DATA_OK.match(text) or _TECH_OK.match(text):
            continue
        if _MODEL_OK.match(text):  # 机器人型号代号
            continue
        if re.search(r"[一-鿿]", text):
            continue
        if re.search(r"[A-Za-z]", text):
            offenders.append((type(kid).__name__, text))

    assert offenders == [], f"界面文案漏中文化：{offenders[:6]}"


def test_playback_button_rows_fit_the_side_bar(window):
    """代码预览的播放按钮按行摆得下（左侧栏只有 280–400px）。

    回归：一行塞 6 个按钮时 Qt 不会换行，只会把每个压成色块（2026-09-21 体检）。
    """
    ed = window.code_editor
    window.layout().activate()

    grid = None
    for lay in ed.findChildren(QtWidgets.QGridLayout):
        if lay.count() >= 6:
            grid = lay
            break
    assert grid is not None, "播放按钮的网格布局不见了"

    rows = {}
    for i in range(grid.count()):
        w = grid.itemAt(i).widget()
        if w is not None:
            rows.setdefault(grid.getItemPosition(i)[0], []).append(w)

    limit = window.left_tabs.maximumWidth()
    for row, widgets in rows.items():
        need = sum(w.minimumSizeHint().width() for w in widgets)
        need += grid.horizontalSpacing() * (len(widgets) - 1)
        assert need <= limit, f"第 {row} 行按钮需要 {need}px，左栏只有 {limit}px——会被压扁"
