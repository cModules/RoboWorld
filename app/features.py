"""构建档位（feature gate）：MVP 只暴露已跑通的离线编程闭环。

为什么用开关而不是删代码
------------------------

v1.4.0 是 MVP 发版口径。下面这条链路已经跑通、有示例（``S3/S4/S5``），可以对外承诺：

    搭场景 → 路径示教 → 校验与节拍 → 干涉检查 → 仿真回放（含抓取随动）
    → 生成多品牌代码 → 存 .core 工程

而工艺时序编辑器、PLC 程序逻辑、机器人子程序、真机连接、插件生态、开发者项，
**要么没做完、要么从未上过真机验证**。它们只要出现在菜单上，就会被当成产品能力。
所以这里隐藏的是**入口**，不是实现：

* 库层代码、测试用例、``.core`` 工程格式一行不动，回归照跑、互不污染；
* ``CORE_FEATURES=full`` 立即恢复全部入口（内部开发 / 后续 2.0 全功能档）；
* 隐藏项的工程数据（时序段、点表段等）继续走「载入并跳过」——**保真**，不静默丢客户数据。

放开某项 = 从配置文件里删一个元素——**不用改这里的 Python 代码**：

* ``pyproject.toml`` 的 ``[tool.core.features] hidden-in-mvp`` 是清单唯一真源；
* 文件读不到 / 段被删掉 / TOML 写错，就退回内置清单 :data:`DEFAULT_HIDDEN_IN_MVP`
  ——宁可按 MVP 保守处理，也不能让程序因为读配置起不来；
* 临时要看全部入口：``CORE_FEATURES=full``（不动文件）；
* 想指着别的文件（打包版、多套发版口径、测试）：``CORE_FEATURES_FILE=/path/to.toml``。

用法::

    from app.features import enabled

    if enabled("plc_logic"):
        file_menu.addAction(link_action)

环境变量 ``CORE_FEATURES``：
    缺省 / ``mvp`` → MVP 档；``full`` / ``all`` / ``on`` / ``1`` / ``dev`` → 全功能档。

测试进程（pytest）在未显式设置该变量时按**全功能档**处理：被测对象是被隐藏掉的那些
入口本身（时序编辑器、PLC 出码、真机连接……），它们必须从入口到实现继续被 test 覆盖，
不能因为发版口径调整而失去回归保护。
"""

from __future__ import annotations

import os
import re
import sys
from pathlib import Path

try:  # Python 3.11+ 自带；老解释器（本项目 supports >=3.9）走下面的简易解析
    import tomllib
except ImportError:  # pragma: no cover - 取决于解释器版本
    tomllib = None  # type: ignore[assignment]

__all__ = [
    "LEVEL_ENV",
    "MVP_LEVEL",
    "FULL_LEVEL",
    "CONFIG_FILE_ENV",
    "DEFAULT_HIDDEN_IN_MVP",
    "HIDDEN_IN_MVP",
    "ALL_FEATURES",
    "configured_hidden",
    "describa",
    "describe",
    "config_file",
    "disabled_features",
    "enabled",
    "hidden_in_mvp",
    "is_full",
    "read_level",
]

#: 档位开关的环境变量名
LEVEL_ENV = "CORE_FEATURES"

#: 显式指定功能清单所在文件（打包版 / 多套发版口径 / 测试）
CONFIG_FILE_ENV = "CORE_FEATURES_FILE"

#: MVP 档（默认）
MVP_LEVEL = "mvp"

#: 全功能档
FULL_LEVEL = "full"

#: 被认作全功能档的写法
_FULL_ALIASES = frozenset({"full", "all", "on", "1", "true", "yes", "dev"})

#: v1.4.0 MVP 隐藏的功能入口。
#:
#: 这是**兜底清单**：只有配置文件读不到时才生效。真正的发版口径写在
#: ``pyproject.toml`` 的 ``[tool.core.features] hidden-in-mvp``，两边保持一致即可，
#: 改口径只动文件。
DEFAULT_HIDDEN_IN_MVP = frozenset(
    {
        # 工艺时序（泳道图）编辑器：交互还在改，队列里排 P5
        "sequence_editor",
        # PLC 程序逻辑：未完善（队列 P2）
        "plc_logic",
        # 机器人子程序：未完善（队列 P2）
        "subprogram",
        # 真机连接（Socket/SSH 传程序 + 回读）：从未上真机验证过，风险最大
        "controller_link",
        # 上传真机 / 真机验证按钮：同上，等 MVP 跑通后再放开
        # （2026-09-21 用户反馈：MVP 档藏掉而不是置灰）
        "real_machine_upload",
        # 插件生态（覆盖层 / 重扫描 / 插件状态）：生态未成型，属开发者项
        "plugins",
        # 开发者项：缓存状态 / 清空缓存
        "dev_tools",
        # 代码生成 → 硬件列表：设备台账还没和出码打通（队列 P2）
        "hardware_list",
    }
)

#: 旧名保留：外部/文档可能还在引用，语义是「默认清单」（同上）
HIDDEN_IN_MVP = DEFAULT_HIDDEN_IN_MVP

#: 受开关管辖的全部功能名（多机不在隐藏集合内：MVP 只做多台独立编程与回放）
ALL_FEATURES = frozenset(DEFAULT_HIDDEN_IN_MVP | {"multirobot"})


def _under_pytest() -> bool:
    """当前进程是否在 pytest 下（测试档默认全功能）。"""
    return "pytest" in sys.modules or "PYTEST_CURRENT_TEST" in os.environ


# --------------------------------------------------------------------------- #
# 配置：``pyproject.toml`` 里的隐藏清单
# --------------------------------------------------------------------------- #
#: 清单所在的 TOML 路径（tool → core → features）
_CONFIG_SECTION = ("tool", "core", "features")

#: 键名两种写法都认（TOML 惯例用连字符，Python 读者爱下划线）
_CONFIG_KEYS = ("hidden-in-mvp", "hidden_in_mvp")

#: 文件内容缓存：键 =(路径, mtime_ns, 大小)，改完文件重启即生效、不用清缓存
_FILE_CACHE: dict[tuple[str, int, int], tuple[str, ...] | None] = {}


def _repo_root() -> Path:
    """仓库根：``<root>/app/features.py`` 往上一级。"""
    return Path(__file__).resolve().parents[1]


def config_candidates() -> list[Path]:
    """按优先级列出可能的功能清单文件（第一个读到的说了算）。"""
    paths: list[Path] = []
    env = (os.environ.get(CONFIG_FILE_ENV) or "").strip()
    if env:
        paths.append(Path(env))
    # PyInstaller onefile 的解压目录：打包时把 pyproject.toml 塞进 datas 就能认到
    meipass = getattr(sys, "_MEIPASS", "")
    if meipass:
        paths.append(Path(meipass) / "pyproject.toml")
    paths.append(_repo_root() / "pyproject.toml")
    paths.append(_repo_root() / "features.toml")
    return paths


def config_file() -> Path | None:
    """实际生效的清单文件；一个都没读到就是 None（此时用内置默认清单）。"""
    for path in config_candidates():
        if _read_list(path) is not None:
            return path
    return None


def _scan_table(text: str) -> tuple[str, ...] | None:
    """没有 tomllib 时的兜底：手撕 ``[tool.core.features]`` 段里的字符串数组。

    只认这一种形状（当前就用这一种），其余一律交给调用方回退默认清单。
    """
    head = "[" + ".".join(_CONFIG_SECTION) + "]"
    lines = text.splitlines()
    try:
        start = next(i for i, ln in enumerate(lines) if ln.strip().replace(" ", "") == head)
    except StopIteration:
        return None
    body: list[str] = []
    for line in lines[start + 1 :]:
        stripped = line.strip()
        if stripped.startswith("["):  # 下一个段开始了
            break
        body.append(line.split("#", 1)[0])
    joined = "\n".join(body)
    for key in _CONFIG_KEYS:
        match = re.search(re.escape(key) + r"\s*=\s*\[([^\]]*)\]", joined, re.S)
        if match:
            return tuple(re.findall(r"['\"]([^'\"]+)['\"]", match.group(1)))
    return None


def _read_list(path: Path) -> tuple[str, ...] | None:
    """读出某文件里的隐藏清单；读不到/没写这个键/写坏了 → None。"""
    try:
        stat = path.stat()
    except OSError:
        return None
    key = (str(path), stat.st_mtime_ns, stat.st_size)
    if key in _FILE_CACHE:
        return _FILE_CACHE[key]
    value: tuple[str, ...] | None = None
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        _FILE_CACHE[key] = None
        return None
    if tomllib is not None:
        try:
            node: object = tomllib.loads(text)
            for part in _CONFIG_SECTION:
                node = node[part]  # type: ignore[index]
            for name in _CONFIG_KEYS:
                found = node.get(name) if isinstance(node, dict) else None  # type: ignore[attr-defined]
                if isinstance(found, list):
                    value = tuple(str(item).strip() for item in found if str(item).strip())
                    break
        except Exception:  # noqa: BLE001 - TOML 写坏不能让 GUI 起不来，回退默认
            value = None
    if value is None:
        value = _scan_table(text)
    _FILE_CACHE[key] = value
    return value


def configured_hidden() -> tuple[str, ...] | None:
    """配置文件里写明的隐藏清单；没有可用配置 → None（交给内置默认清单）。"""
    for path in config_candidates():
        found = _read_list(path)
        if found is not None:
            return found
    return None


def hidden_in_mvp() -> frozenset:
    """当前生效的 MVP 隐藏集合。

    配置文件说什么就是什么——包括**空列表**（等于全放开）。只有读不到配置
    时才用 :data:`DEFAULT_HIDDEN_IN_MVP` 兜底。
    """
    configured = configured_hidden()
    if configured is None:
        return frozenset(DEFAULT_HIDDEN_IN_MVP)
    return frozenset(configured)


def read_level() -> str:
    """读取当前构建档位（每次都读环境变量，便于测试用 monkeypatch 切换）。"""
    raw = (os.environ.get(LEVEL_ENV) or "").strip().lower()
    if not raw:
        return FULL_LEVEL if _under_pytest() else MVP_LEVEL
    if raw == MVP_LEVEL:
        return MVP_LEVEL
    if raw in _FULL_ALIASES:
        return FULL_LEVEL
    # 未知写法按 MVP 处理：宁可少暴露，也不要把没验证的东西放出去
    return MVP_LEVEL


def is_full() -> bool:
    """是否全功能档。"""
    return read_level() == FULL_LEVEL


def enabled(feature: str) -> bool:
    """该功能的入口是否应该构建。

    未知功能名一律开：未知不应该等于隐藏，避免新功能悄悄消失。
    """
    if is_full():
        return True
    return feature not in hidden_in_mvp()


def disabled_features() -> frozenset:
    """当前档位下被隐藏的功能集合（全功能档为空）。"""
    return frozenset() if is_full() else hidden_in_mvp()


def describe() -> str:
    """一句人话描述当前档位（「关于」对话框/日志可以用）。"""
    if is_full():
        return f"{FULL_LEVEL}: 全部功能入口可用"
    source = config_file()
    origin = f"，清单来自 {source.name}" if source is not None else "，清单用内置默认"
    hidden = hidden_in_mvp()
    return f"{MVP_LEVEL}: 仅暴露 MVP 闭环（隐藏 {len(hidden)} 项未完成入口{origin}）"


# 历史拼写兼容：``describa`` 是早期草稿里的笔误，别让外部调用炸掉
describa = describe
