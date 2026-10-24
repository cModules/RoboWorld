"""v0.1.40 构建档位开关（``app/features.py``）的回归测试。

这里守三条底线，缺一条都不能发 MVP：

1. **MVP 档必须真的藏干净**——没做完 / 没上真机的入口不许出现在界面上；
2. **全功能档必须全回来**——内部开发与后续 2.0 靠 ``CORE_FEATURES=full``，
   隐藏是暂时的，不能变成"删过了所以再也回不来"；
3. **隐藏入口 ≠ 丢数据**——工程里已经有 sequence 段时，没有编辑器也必须原样
   带回去，不能因为节省 Entropy 让用户的数据静默蒸发。
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest  # noqa: E402

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_ROOT))

from app import features  # noqa: E402
from support.i18n import tr  # noqa: E402


# --------------------------------------------------------------------------- #
# 测试用例都跑在全功能档下（features 对 pytest 的默认）——只有显式指定 mvp
# 时才切到 MVP，这样被隐藏的那些入口在测试里仍然全量回归。
# --------------------------------------------------------------------------- #
def test_unspecified_level_is_full_under_pytest(monkeypatch):
    """不显式设置时，测试进程按全功能档跑（参见 features._under_pytest）。"""
    monkeypatch.delenv("CORE_FEATURES", raising=False)
    assert features.read_level() == features.FULL_LEVEL
    assert features.is_full()
    assert features.disabled_features() == frozenset()


def test_mvp_level_hides_unfinished_entries(monkeypatch):
    monkeypatch.setenv("CORE_FEATURES", "mvp")
    assert features.read_level() == features.MVP_LEVEL
    for name in (
        "sequence_editor",
        "plc_logic",
        "subprogram",
        "controller_link",
        "plugins",
        "dev_tools",
    ):
        assert not features.enabled(name), f"{name} 不该出现在 MVP 档的界面上"
    assert features.disabled_features() == features.HIDDEN_IN_MVP


def test_hardware_list_is_hidden_in_mvp(monkeypatch):
    """代码预览下的「硬件列表」属于未完成项，MVP 档不许出现。"""
    monkeypatch.setenv("CORE_FEATURES", "mvp")
    assert not features.enabled("hardware_list")


def test_multirobot_stays_enabled_in_mvp(monkeypatch):
    """多机是 MVP 承诺的一部分（多台独立编程与回放），不许被隐藏。"""
    for level in ("mvp", "full"):
        monkeypatch.setenv("CORE_FEATURES", level)
        assert features.enabled("multirobot")


# --------------------------------------------------------------------------- #
# 隐藏清单来自 pyproject.toml：改口径只改文件，不用碰代码
# --------------------------------------------------------------------------- #
def _write_config(tmp_path, text):
    cfg = tmp_path / "features.toml"
    cfg.write_text(text, encoding="utf-8")
    return cfg


def test_hidden_list_comes_from_pyproject(monkeypatch):
    """默认读到仓库根的 pyproject.toml，且与内置兜底清单一致。

    两边漂移说明有人只改了一边（改了文件却忘了同步默认清单 / 反之），
    打包后读不到文件时口径会突然变化——这必须当场暴露。
    """
    monkeypatch.delenv(features.CONFIG_FILE_ENV, raising=False)
    monkeypatch.setenv("CORE_FEATURES", "mvp")
    assert features.config_file() == _ROOT / "pyproject.toml"
    assert features.hidden_in_mvp() == frozenset(features.DEFAULT_HIDDEN_IN_MVP)


def test_custom_config_file_wins(tmp_path, monkeypatch):
    """``CORE_FEATURES_FILE`` 指向别处时以它为准（打包版 / 多套口径 / 测试）。"""
    monkeypatch.setenv(
        features.CONFIG_FILE_ENV,
        str(_write_config(tmp_path, '[tool.core.features]\nhidden-in-mvp = ["plc_logic"]\n')),
    )
    monkeypatch.setenv("CORE_FEATURES", "mvp")
    assert features.hidden_in_mvp() == frozenset({"plc_logic"})
    assert not features.enabled("plc_logic")
    assert features.enabled("controller_link")  # 没写进清单的一律放开


def test_empty_hidden_list_means_nothing_hidden(tmp_path, monkeypatch):
    """清单清空 = 全放开：这是「逐个完善」的终点，不能退化成兜底清单。"""
    monkeypatch.setenv(
        features.CONFIG_FILE_ENV,
        str(_write_config(tmp_path, "[tool.core.features]\nhidden-in-mvp = []\n")),
    )
    monkeypatch.setenv("CORE_FEATURES", "mvp")
    assert features.hidden_in_mvp() == frozenset()
    assert features.disabled_features() == frozenset()


def test_missing_config_falls_back_to_default(tmp_path, monkeypatch):
    """文件不存在（打包时忘了带）→ 回退内置清单，不能让程序因此起不来。"""
    monkeypatch.setenv(features.CONFIG_FILE_ENV, str(tmp_path / "不存在的文件.toml"))
    monkeypatch.setenv("CORE_FEATURES", "mvp")
    assert features.hidden_in_mvp() == frozenset(features.DEFAULT_HIDDEN_IN_MVP)


def test_broken_config_falls_back_to_default(tmp_path, monkeypatch):
    """TOML 写坏了同样回退：清单只是口径，读错不该等于打不开软件。"""
    broken = _write_config(tmp_path, '[tool.core.features\nhidden-in-mvp = ["plc_logic"\n')
    monkeypatch.setenv(features.CONFIG_FILE_ENV, str(broken))
    monkeypatch.setenv("CORE_FEATURES", "mvp")
    assert features.hidden_in_mvp() == frozenset(features.DEFAULT_HIDDEN_IN_MVP)


def test_full_level_enables_everything(monkeypatch):
    monkeypatch.setenv("CORE_FEATURES", "full")
    assert features.is_full()
    for name in sorted(features.ALL_FEATURES):
        assert features.enabled(name)


@pytest.mark.parametrize("level", ["all", "on", "1", "true", "dev"])
def test_full_aliases(level, monkeypatch):
    monkeypatch.setenv("CORE_FEATURES", level)
    assert features.is_full()


def test_unknown_level_falls_back_to_mvp(monkeypatch):
    """写错的档位按 MVP 处理：宁可少暴露，也不要把没验证的东西放出去。"""
    monkeypatch.setenv("CORE_FEATURES", "beta ???")
    assert features.read_level() == features.MVP_LEVEL
    assert not features.enabled("controller_link")


def test_unknown_feature_is_enabled(monkeypatch):
    """新功能没登记到隐藏集合也不该悄悄消失——未知 ≠ 隐藏。"""
    monkeypatch.setenv("CORE_FEATURES", "mvp")
    assert features.enabled("还不知道叫什么的未来功能")


def test_describe_mentions_level(monkeypatch):
    monkeypatch.setenv("CORE_FEATURES", "mvp")
    assert features.MVP_LEVEL in features.describe()
    monkeypatch.setenv("CORE_FEATURES", "full")
    assert features.FULL_LEVEL in features.describe()


# --------------------------------------------------------------------------- #
# GUI 侧：入口到底有没有真的消失
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def qapp():
    from PySide6 import QtWidgets

    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    yield application


def test_scene_panel_hides_sequence_editor_in_mvp(qapp, monkeypatch):
    """MVP 档不构建工艺时序（泳道图）分组——整组消失，不是留个空壳。"""
    monkeypatch.setenv("CORE_FEATURES", "mvp")
    from domain.world.panel import ScenePanel

    panel = ScenePanel()
    assert not hasattr(panel, "sequence_editor")
    assert not panel.findChildren(ScenePanel)


def test_scene_panel_keeps_sequence_editor_in_full(qapp, monkeypatch):
    """全功能档照旧要有：这里的入口只是被发版口径关掉，不是被删掉。"""
    monkeypatch.setenv("CORE_FEATURES", "full")
    from domain.world.panel import ScenePanel

    panel = ScenePanel()
    assert hasattr(panel, "sequence_editor")
    assert panel.sequence_editor is not None


def _tab_names(tabs) -> set:
    return {tabs.tabText(i).replace("&", "") for i in range(tabs.count())}


def test_code_editor_hides_hardware_list_in_mvp(qapp, monkeypatch):
    """MVP 档「代码预览」下面没有「硬件列表」这一页（路径示教必须还在）。"""
    monkeypatch.setenv("CORE_FEATURES", "mvp")
    from main import PathCodeEditor

    editor = PathCodeEditor()
    names = _tab_names(editor.sub_tabs)
    assert tr("路径示教") in names
    assert tr("硬件列表") not in names


def test_code_editor_keeps_hardware_list_in_full(qapp, monkeypatch):
    monkeypatch.setenv("CORE_FEATURES", "full")
    from main import PathCodeEditor

    editor = PathCodeEditor()
    assert tr("硬件列表") in _tab_names(editor.sub_tabs)


def test_sync_devices_keeps_data_without_hardware_table(qapp, monkeypatch):
    """没有硬件列表也得把设备数据攒着。

    ``cell`` 是真源（PLC、点表读的都是它），表格只是看它的窗口。MVP 档表格
    不存在时 ``sync_devices`` 必须静默通过，不能崩在一个 AttributeError 上。
    """
    monkeypatch.setenv("CORE_FEATURES", "mvp")
    from main import PathCodeEditor

    editor = PathCodeEditor()
    assert editor.cell is not None, "工作单元模型不可用，这条用例没有意义"

    class _Scene:
        objects = {"robot_1": object()}

        def is_robot(self, _name):
            return True

    editor.sync_devices(_Scene())
    assert [d.name for d in editor.cell.devices] == ["robot_1"]


# --------------------------------------------------------------------------- #
# 布局约定：设备树在右、代码预览在左
# --------------------------------------------------------------------------- #
def test_device_tree_is_on_the_right(window):
    """「设备树」搬到右侧：层级编排与属性（选中即改位姿）在同一只手下完成。"""
    assert tr("设备树") in _tab_names(window.right_tabs)
    assert tr("设备树") not in _tab_names(window.left_tabs)


def test_code_preview_is_on_the_left(window):
    """「代码预览」搬到左侧：紧贴视口，改一行 movej 当场看到效果。"""
    assert tr("代码预览") in _tab_names(window.left_tabs)
    assert tr("代码预览") not in _tab_names(window.right_tabs)


def test_focus_tab_finds_the_panel_whichever_side_it_is_on(window):
    """面板左右能挪，但"点按钮就该跳过去"不能坏：按所在组找，不写死是哪一侧。"""
    window.right_tabs.setCurrentIndex(0)
    window._focus_tab(window.scene_panel)
    assert window.right_tabs.currentWidget() is window.scene_panel

    window._focus_tab(window.code_editor)
    assert window.left_tabs.currentWidget() is window.code_editor


@pytest.fixture()
def window(qapp, monkeypatch, tmp_path):
    """一个真实的主窗口。

    两个刻意的设定：① 隔离 ``CORE_CONFIG_DIR``——否则会读到开发机上残留的
    自动保存快照，``showEvent`` 里的恢复询问会弹出模态框等人点；② 不 ``show()``，
    这条用例只关心数据保真，不需要真的走到 ``showEvent``。
    """
    import main

    monkeypatch.setenv("CORE_CONFIG_DIR", str(tmp_path / "cfg"))
    win = main.MainWindow()
    qapp.processEvents()
    yield win
    win.program_panel.stop_playback()
    win.close()


def test_collect_into_preserves_sequence_without_editor(window):
    """隐藏入口 ≠ 丢数据。

    时序编辑器不在时，``_collect_into`` 跳过写入——但工程里**已经有的**
    sequence 段必须原样留在 Project 里（Unknown 段保留是 Project 的既有语义），
    否则客户打开旧工程再保存，时序数据就静默没了。
    """
    from service.project import Project

    seq = {
        "schema": "service.sequence/1",
        "steps": [{"name": "等待取件信号", "kind": "wait_signal"}],
    }
    project = Project()
    project.set("sequence", seq)

    editor = getattr(window.scene_panel, "sequence_editor", None)
    try:
        if editor is not None:
            # 模拟 MVP 档：入口没建起来
            del window.scene_panel.sequence_editor
        window._collect_into(project)
    finally:
        if editor is not None:
            window.scene_panel.sequence_editor = editor

    assert project.get("sequence") == seq
