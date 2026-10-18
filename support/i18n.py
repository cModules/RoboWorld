"""轻量国际化（v1.0.10）：`tr()` 运行时翻译 + 语言持久化。

设计取舍
========

* **zh 是源语言**：代码里的中文字面量就是 key，``tr()`` 在 zh（默认）下
  恒等返回——既有测试与文档零回归；en 通过目录查表，查不到就原样返回
  （宁可漏翻一句中文，也不让界面出现占位符）。
* **显式配置优先**，不做 locale 自动探测：语言影响的是"用户看到什么"，
  必须可预期。优先级：环境变量 ``CORE_LANG`` > 配置文件（默认
  ``~/.core/language.json``，可用 ``CORE_CONFIG_DIR`` 重定向）> zh。
* **f-string 一律转成模板**：``tr("已载入 {name}（{n} 个对象）").format(...)``
  ——目录里的 key 是带占位符的模板本身，占位符用命名参数，英文语序可调整。
* **GUI 切换语言需要重启**（面板在构造期写入文案，做全量 retranslate
  的复杂度远大于价值）；库层消息（planner Issue / 报告 / 后处理注释）
  自 v1.1 起同样走 ``tr()``——消息在**生成时刻**按当前语言定稿，与 GUI
  一致（切语言需重启后重建面板/重跑校验）。

用法::

    from support.i18n import tr, set_language, current_language

    set_language("en")
    tr("删除")                  # -> "Delete"
    tr("已载入 {name}").format(name="scene1")
"""

from __future__ import annotations

import json
import os
from pathlib import Path

__all__ = [
    "LANG_ZH",
    "LANG_EN",
    "LANGUAGE_NAMES",
    "available_languages",
    "current_language",
    "set_language",
    "tr",
]

LANG_ZH = "zh"
LANG_EN = "en"

#: 语言代码 -> 显示名（菜单里展示用）
LANGUAGE_NAMES = {LANG_ZH: "中文", LANG_EN: "English"}

_LANGUAGE: str | None = None


def _config_path() -> Path:
    """语言配置文件路径。``CORE_CONFIG_DIR`` 供测试/便携部署重定向。"""
    base = os.environ.get("CORE_CONFIG_DIR")
    if base:
        return Path(base) / "language.json"
    return Path.home() / ".core" / "language.json"


def _detect() -> str:
    """启动时解析语言：环境变量 > 配置文件 > 默认 zh。"""
    env = os.environ.get("CORE_LANG", "").strip().lower()
    if env in LANGUAGE_NAMES:
        return env
    try:
        data = json.loads(_config_path().read_text(encoding="utf-8"))
        lang = str(data.get("language", "")).strip().lower()
        if lang in LANGUAGE_NAMES:
            return lang
    except Exception:
        pass
    return LANG_ZH


def current_language() -> str:
    """当前语言代码（惰性解析，进程内缓存）。"""
    global _LANGUAGE
    if _LANGUAGE is None:
        _LANGUAGE = _detect()
    return _LANGUAGE


def set_language(lang: str, *, save: bool = True) -> str:
    """设置当前语言；``save=True`` 时写入配置文件供下次启动沿用。"""
    global _LANGUAGE
    if lang not in LANGUAGE_NAMES:
        raise ValueError(f"unsupported language {lang!r}; available: {sorted(LANGUAGE_NAMES)}")
    _LANGUAGE = lang
    if save:
        path = _config_path()
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(
                json.dumps({"language": lang}, ensure_ascii=False, indent=1),
                encoding="utf-8",
            )
        except Exception:
            pass  # 配置盘写失败不该挡住切换
    return lang


def available_languages() -> list[tuple[str, str]]:
    """(代码, 显示名) 列表，菜单/设置界面直接用。"""
    return [(code, LANGUAGE_NAMES[code]) for code in (LANG_ZH, LANG_EN)]


def tr(text: str) -> str:
    """翻译一条 UI 文案。zh 恒等返回；en 查目录，未收录原样返回。"""
    if current_language() == LANG_ZH:
        return text
    return _CATALOG_EN.get(text, text)


# re-export：词条数据已外置至 support/i18n_catalog.py（2026-10-18，Phase 1 完成）。
# 守护 H9（tools/check_arch_rules.py）与 tools/build_web_site.py 经本模块取词条，
# 调用面保持不变。
from support.i18n_catalog import _CATALOG_EN  # noqa: F401,E402
