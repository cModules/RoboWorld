"""i18n（v1.0.10）的无头测试。

覆盖轻量国际化基础设施与 GUI 接线：

* ``tr()`` 语言语义——zh 恒等（默认零回归）、en 查表、未收录原样返回
* 语言解析优先级——环境变量 ``CORE_LANG`` > 配置文件 > 默认 zh
* ``set_language`` 持久化与非法语言拒绝
* GUI 接线——en 模式下 ProgramPanel 按钮文案为英文；zh 模式保持中文
* 语言菜单——主窗口存在语言动作且互斥选中
* 目录卫生——所有英文译文非空、目录可完整往返

全部跑在 Qt offscreen 平台上，不需要真实显示服务器。
"""

import ast
import importlib.util
import json
import os
import re
import sys
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

import support.i18n as i18n  # noqa: E402
from domain.robot import Robot  # noqa: E402
from app.views.program_panel import ProgramPanel  # noqa: E402

URDF = REPO_ROOT / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"
JOINTS = [f"joint_{i}" for i in range(1, 7)]


@pytest.fixture(autouse=True)
def _isolated_i18n(monkeypatch, tmp_path):
    """每个用例独享语言状态与配置目录，结束后回到 zh。"""
    monkeypatch.delenv("CORE_LANG", raising=False)
    monkeypatch.setenv("CORE_CONFIG_DIR", str(tmp_path / "cfg"))
    i18n.set_language(i18n.LANG_ZH, save=False)
    yield
    i18n.set_language(i18n.LANG_ZH, save=False)


@pytest.fixture(scope="module")
def qapp():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


from PySide6 import QtWidgets  # noqa: E402

# ---------------------------------------------------------------------------
# tr() 语义
# ---------------------------------------------------------------------------


def test_zh_is_identity():
    i18n.set_language(i18n.LANG_ZH, save=False)
    assert i18n.tr("删除") == "删除"
    assert i18n.tr("不存在的文案") == "不存在的文案"


def test_en_translates_known_string():
    i18n.set_language(i18n.LANG_EN, save=False)
    assert i18n.tr("删除") == "Delete"
    assert i18n.tr("校验程序 (F7)") == "Validate (F7)"


def test_en_falls_back_for_unknown():
    i18n.set_language(i18n.LANG_EN, save=False)
    assert i18n.tr("绝不存在的文案 XYZ") == "绝不存在的文案 XYZ"


def test_en_template_lookup():
    i18n.set_language(i18n.LANG_EN, save=False)
    tpl = "已载入 {name}（{n} 个对象）"
    assert i18n.tr(tpl).format(name="s1", n=3) == "Loaded s1 (3 objects)"


# ---------------------------------------------------------------------------
# 语言解析与持久化
# ---------------------------------------------------------------------------


def test_default_is_zh():
    assert i18n.current_language() == i18n.LANG_ZH


def test_env_var_wins(monkeypatch):
    monkeypatch.setenv("CORE_LANG", "en")
    # 重置进程内缓存
    i18n._LANGUAGE = None
    assert i18n.current_language() == i18n.LANG_EN
    i18n._LANGUAGE = None


def test_env_var_invalid_falls_back(monkeypatch):
    monkeypatch.setenv("CORE_LANG", "fr")
    i18n._LANGUAGE = None
    assert i18n.current_language() == i18n.LANG_ZH
    i18n._LANGUAGE = None


def test_config_file_roundtrip(tmp_path):
    i18n.set_language(i18n.LANG_EN)  # save=True 默认
    cfg = tmp_path / "cfg" / "language.json"
    assert cfg.exists()
    assert json.loads(cfg.read_text(encoding="utf-8"))["language"] == "en"
    # 新"进程"（清缓存）读回
    i18n._LANGUAGE = None
    assert i18n.current_language() == i18n.LANG_EN
    i18n._LANGUAGE = None


def test_env_beats_config_file(monkeypatch, tmp_path):
    i18n.set_language(i18n.LANG_EN)  # 写入配置文件
    i18n._LANGUAGE = None
    monkeypatch.setenv("CORE_LANG", "zh")
    assert i18n.current_language() == i18n.LANG_ZH
    i18n._LANGUAGE = None


def test_set_language_rejects_unknown():
    with pytest.raises(ValueError):
        i18n.set_language("klingon", save=False)


def test_available_languages():
    langs = dict(i18n.available_languages())
    assert langs == {"zh": "中文", "en": "English"}


# ---------------------------------------------------------------------------
# 目录卫生
# ---------------------------------------------------------------------------


def test_catalog_values_non_empty():
    for key, value in i18n._CATALOG_EN.items():
        assert isinstance(value, str) and value.strip(), f"empty translation for {key!r}"


def test_catalog_keys_unique_nonempty():
    keys = list(i18n._CATALOG_EN)
    assert len(keys) == len(set(keys))
    assert all(k.strip() for k in keys)


# ---------------------------------------------------------------------------
# GUI 接线（offscreen）
# ---------------------------------------------------------------------------


@pytest.fixture()
def make_panel(qapp):
    """面板工厂：tr() 在构造期求值，语言切换后须重新构造才能看到新文案。"""
    created = []

    def _make():
        robot = Robot(str(URDF), "UR5e")
        p = ProgramPanel()
        p.set_robot(robot, JOINTS)
        created.append(p)
        return p

    yield _make
    for p in created:
        p.deleteLater()


def test_panel_text_zh_by_default(make_panel):
    i18n.set_language(i18n.LANG_ZH, save=False)
    panel = make_panel()
    assert panel.btn_validate_all.text() == "校验全部"
    assert panel.btn_delete_point.text() == "删除"


def test_panel_text_en(make_panel):
    i18n.set_language(i18n.LANG_EN, save=False)
    panel = make_panel()
    assert panel.btn_validate_all.text() == "Validate all"
    assert panel.btn_delete_point.text() == "Delete"
    assert panel.combo_post.itemText(0)  # 冒烟：组合框可访问


def test_main_window_has_language_menu_en(qapp, monkeypatch):
    """en 模式下 MainWindow 构造成功且语言菜单动作互斥选中。"""
    monkeypatch.setenv("CORE_LANG", "en")
    i18n._LANGUAGE = None
    try:
        spec = importlib.util.spec_from_file_location("rw_app_i18n", REPO_ROOT / "main.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        win = mod.MainWindow()
        try:
            assert hasattr(win, "_lang_actions")
            assert set(win._lang_actions) == {"zh", "en"}
            assert win._lang_actions["en"].isChecked()
            assert not win._lang_actions["zh"].isChecked()
            # 英文模式下按钮文案已是英文
            assert win.program_panel.btn_validate_all.text() == "Validate all"
        finally:
            win.deleteLater()
    finally:
        i18n._LANGUAGE = None


# --------------------------------------------------------------------------- #
# 语言切换的"假全英文"守卫
# --------------------------------------------------------------------------- #

#: 英文词典里漏掉的键会**原样显示中文**——切到 en 后界面中英夹杂，正是
#: "提供了语言切换却切不干净"的样子（2026-09-21 用户提出要语言切换菜单）
_CJK = re.compile(r"[一-鿿]")


def _tr_keys() -> set:
    """扫源码里所有 ``tr("...")`` 的字面量键（ast 取值，转义与运行时一致）。"""
    # 必须连**顶层 GUI 包**一起扫：tr() 键可能只出现在 app/ 里，
    # 漏掉它 = 这类键缺双语词条也不会被发现
    files = [REPO_ROOT / "main.py"]
    for _root_dir in (REPO_ROOT / "app", REPO_ROOT / "core"):
        files += sorted(_root_dir.rglob("*.py"))
    keys = set()
    for f in files:
        try:
            tree = ast.parse(f.read_text(encoding="utf-8"))
        except SyntaxError:  # pragma: no cover - 源码都合法
            continue
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if not (isinstance(node.func, ast.Name) and node.func.id == "tr"):
                continue
            if node.args and isinstance(node.args[0], ast.Constant):
                value = node.args[0].value
                if isinstance(value, str):
                    keys.add(value)
    return keys


def test_every_tr_key_has_english():
    """每个 tr() 中文键都得有英文译文——否则 en 模式漏出中文。"""
    from support import i18n

    missing = sorted(k for k in _tr_keys() if _CJK.search(k) and k not in i18n._CATALOG_EN)
    assert missing == [], f"这些键缺英文译文：{missing[:5]}"


def test_catalog_has_no_duplicate_keys():
    """词典里的重复键只有后者生效——漏译时极难看出来。

    用 ast 读字面量而不是正则扫文本：键里有 ``\\"`` 转义时正则会在转义引号处
    截断，把一条正常条目误判成两条"重复"。
    """
    from support import i18n_catalog

    # _CATALOG_EN 自 2026-10-18 起定义在 i18n_catalog.py（数据表外置，Phase 1），
    # i18n.py 仅 re-export——重复键检测要读数据真身。
    tree = ast.parse(Path(i18n_catalog.__file__).read_text(encoding="utf-8"))
    keys = []
    for node in ast.walk(tree):
        value = getattr(node, "value", None)
        name = getattr(node, "target", None)
        if not isinstance(node, ast.AnnAssign) or not isinstance(value, ast.Dict):
            continue
        if not (isinstance(name, ast.Name) and name.id == "_CATALOG_EN"):
            continue
        keys = [k.value for k in value.keys if isinstance(k, ast.Constant)]
    assert keys, "没解析到词典内容"
    dups = sorted({k for k in keys if keys.count(k) > 1})
    assert dups == [], f"词典里有重复键：{dups}"
