"""RoboWorld 命令行入口（ADR-003）：``core doctor / post / world``。

用法::

    core doctor               # 环境自检：依赖 / 档位 / 资源 / 加速层
    core post --list          # 列出内置与插件后处理器
    core post --check kuka    # 取一个后处理器并回显元信息
    core world --list         # 列出预置工作站（assets/cells/*.json）
    core world --show NAME    # 载入一个预置工作站并打印摘要

**为什么必须存在**：``pyproject.toml`` 的 ``[project.scripts]`` 长期指向这个不存在
的文件——``pip install core`` 之后敲 ``core`` 直接 ModuleNotFoundError，而 ADR-003
把"打包即代码"定为原则：入口点与包结构必须真能跑通（CI 里由 wheel 冒烟守住）。

定位：**只读诊断**。不写文件、不改场景、不连真机——出问题时它是最先能跑起来的
那条路，所以这里刻意不引入任何 GUI 依赖（PySide6 缺失也照常工作）。
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path
from typing import List, Sequence

from support import __version__
from support.i18n import tr

#: ``pyproject.toml`` 的 requires-python 是 3.9，doctor 按同一口径判
_REQUIRED_PYTHON = (3, 9)

#: 库层硬依赖（缺一个核心就不可用）
_CORE_DEPS = ("numpy", "scipy", "jinja2", "yaml", "pydantic")

#: GUI 可选依赖（缺失只是没有界面，库层与 CLI 照常）
_GUI_DEPS = ("PySide6", "OpenGL")


def repo_root() -> Path:
    """资源根：源码树是仓库根，冻结包（PyInstaller）里是 ``sys._MEIPASS``。

    与 :func:`domain.world.templates.repo_root` 同口径——``assets/models/``、``assets/``
    都挂在这个根下；冻结后 ``__file__`` 会跑到临时解压目录里，不能再往上数级。
    """
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        return Path(meipass)
    return Path(__file__).resolve().parents[1]


def _cells_dir() -> Path:
    return repo_root() / "assets" / "cells"


def _installed(name: str) -> bool:
    """模块能否被找到（不真正导入，避免拖进 OCP 这类重依赖）。"""
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ValueError):  # 命名空间包异常/父包缺失
        return False


def _missing(names: Sequence[str]) -> List[str]:
    return [n for n in names if not _installed(n)]


def _ok(flag: bool) -> str:
    return tr("可用") if flag else tr("缺失")


# --------------------------------------------------------------------------- #
# doctor
# --------------------------------------------------------------------------- #
def cmd_doctor(args: argparse.Namespace) -> int:
    """环境自检：把"为什么跑不起来"一次说清。"""
    hard_fail = 0

    print(f"RoboWorld {__version__} — {tr('环境自检')}")

    py_ok = sys.version_info[:2] >= _REQUIRED_PYTHON
    print(
        tr("Python {ver}（要求 {req}+）: {state}").format(
            ver=".".join(str(v) for v in sys.version_info[:3]),
            req=".".join(str(v) for v in _REQUIRED_PYTHON),
            state=_ok(py_ok),
        )
    )
    if not py_ok:
        hard_fail += 1

    missing_core = _missing(_CORE_DEPS)
    print(
        tr("核心依赖: {state}{detail}").format(
            state=_ok(not missing_core),
            detail="" if not missing_core else f"（{tr('缺少')}: {', '.join(missing_core)}）",
        )
    )
    if missing_core:
        hard_fail += 1

    missing_gui = _missing(_GUI_DEPS)
    print(
        tr("GUI 依赖: {state}{detail}").format(
            state=_ok(not missing_gui),
            detail=(
                ""
                if not missing_gui
                else f"（{tr('未安装')}: {', '.join(missing_gui)}；{tr('界面不可用，库层不受影响')}）"
            ),
        )
    )

    try:
        from app import features

        level = features.read_level()
        source = features.config_file()
        origin = source.name if source is not None else tr("内置默认清单")
        print(tr("构建档位: {level}（隐藏清单来自 {origin}）").format(level=level, origin=origin))
    except Exception as exc:  # pragma: no cover - features 模块极度稳定
        print(tr("构建档位: 读取失败（{exc}）").format(exc=exc))

    try:
        from accel import has_accel

        print(tr("C++ 加速层: {state}").format(state=_ok(has_accel)))
    except Exception:  # pragma: no cover
        print(tr("C++ 加速层: 缺失"))

    models = repo_root() / "assets" / "models"
    urdfs = sorted(models.rglob("*.urdf")) if models.is_dir() else []
    cells = sorted(_cells_dir().glob("*.json")) if _cells_dir().is_dir() else []
    print(tr("模型库: {n} 个 URDF（{root}）").format(n=len(urdfs), root=models))
    print(tr("预置工作站: {n} 个（{root}）").format(n=len(cells), root=_cells_dir()))

    if hard_fail:
        print(tr("结论: 有 {n} 项硬缺失，请先安装依赖").format(n=hard_fail))
        return 1
    print(tr("结论: 环境可用"))
    return 0


# --------------------------------------------------------------------------- #
# post
# --------------------------------------------------------------------------- #
def cmd_post(args: argparse.Namespace) -> int:
    """后处理器：列清单 / 回显元信息。"""
    from adapter.post import PostProcessError, get_post_processor, list_post_processors

    if args.check:
        try:
            pp = get_post_processor(args.check)
        except PostProcessError as exc:
            print(tr("未知后处理器: {name}（{exc}）").format(name=args.check, exc=exc))
            return 1
        print(
            tr("{name}: 品牌 {brand} / 控制器 {controller} / 扩展名 .{ext}").format(
                name=pp.full_name,
                brand=pp.brand,
                controller=pp.controller,
                ext=pp.file_extension,
            )
        )
        return 0

    names = list_post_processors()
    print(tr("后处理器 {n} 个:").format(n=len(names)))
    for name in names:
        try:
            pp = get_post_processor(name)
        except Exception:  # 单个品牌构造失败不该带崩整张清单
            print(f"  {name}  ({tr('构造失败')})")
            continue
        print(f"  {name:<20} .{pp.file_extension}")
    return 0


# --------------------------------------------------------------------------- #
# world
# --------------------------------------------------------------------------- #
def cmd_world(args: argparse.Namespace) -> int:
    """预置工作站：列清单 / 载入并打印摘要。"""
    cells_dir = _cells_dir()
    if not cells_dir.is_dir():
        print(tr("未找到预置工作站目录: {path}").format(path=cells_dir))
        return 1

    files = sorted(cells_dir.glob("*.json"))

    if args.show:
        match = next((f for f in files if f.stem == args.show), None)
        if match is None:
            print(tr("未找到工作站: {name}").format(name=args.show))
            return 1
        from domain.world.scene import Scene

        scene = Scene.load(match)
        robots = ", ".join(scene.robot_names) or tr("无")
        print(
            tr("工作站 {name}: 机器人 {r} 台（{names}），对象 {o} 个").format(
                name=match.stem,
                r=len(scene.robot_names),
                names=robots,
                o=len(scene.objects),
            )
        )
        return 0

    if not files:
        print(tr("未找到预置工作站目录: {path}").format(path=cells_dir))
        return 1
    print(tr("预置工作站 {n} 个:").format(n=len(files)))
    for path in files:
        print(f"  {path.stem}")
    return 0


# --------------------------------------------------------------------------- #
# 入口
# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    """构造参数解析器（独立出来便于测试直接复用）。"""
    parser = argparse.ArgumentParser(
        prog="core", description=tr("RoboWorld 命令行：环境自检与资源查询")
    )
    parser.add_argument("--version", action="version", version=f"core {__version__}")
    sub = parser.add_subparsers(dest="command")

    p_doctor = sub.add_parser("doctor", help=tr("环境自检"))
    p_doctor.set_defaults(func=cmd_doctor)

    p_post = sub.add_parser("post", help=tr("后处理器清单与查询"))
    p_post.add_argument("--check", metavar="NAME", help=tr("回显该后处理器的元信息"))
    p_post.add_argument("--list", action="store_true", help=tr("列出全部后处理器"))
    p_post.set_defaults(func=cmd_post)

    p_world = sub.add_parser("world", help=tr("预置工作站清单与查询"))
    p_world.add_argument("--show", metavar="NAME", help=tr("载入该工作站并打印摘要"))
    p_world.add_argument("--list", action="store_true", help=tr("列出全部预置工作站"))
    p_world.set_defaults(func=cmd_world)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """``core`` 入口点。无子命令时打帮助并返回 0（不是错误用法）。"""
    parser = build_parser()
    args = parser.parse_args(argv)
    if not getattr(args, "func", None):
        parser.print_help()
        return 0
    return int(args.func(args))


if __name__ == "__main__":  # pragma: no cover - 便于 python -m app.cli
    raise SystemExit(main())
