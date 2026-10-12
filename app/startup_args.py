"""启动参数解析：把命令行/文件关联传进来的一个路径，判定成一次启动意图。

双击一个 ``.rwproj``、把 URDF 拖到 exe 上、或者什么都不给——三种情况必须走
不同的启动分支，而它们的差别只体现在 ``sys.argv`` 里那一个字符串上。

为什么单独成文件（而不是塞进 ``main.py``）：

* ``main.py`` 是**启动壳**（``sys.path`` 前导 + 崩溃处理 + 拉窗口）。塞进判定
  规则会把它从 160 行推到 300 行，还混进"需要单测的分支逻辑"；
* 判定规则**不依赖 Qt**——放在这里可以在没有 GUI 的环境（Linux 无头、CI）
  直接跑测试，不需要 ``QT_QPA_PLATFORM=offscreen`` 也能验证。

后缀真源在 :mod:`service.project`（``PROJECT_EXTENSIONS``）：工程扩展名是工程
格式的一部分，那里是唯一定义处。本模块只做"字符串 → 意图"的翻译，
**不再抄一份后缀表**——两处漂移的典型症状是"菜单能打开、双击没反应"。

目录分支为什么单列：用户报告 ``.rwproj/.robotproj`` 在 Windows 上显示成
**文件夹图标**，说明它有可能是目录形态的 bundle（尚未确认）。这里先把目录判成
:attr:`StartupKind.DIRECTORY` 单独一支，形态确认后**只收口这一处**。
"""

from __future__ import annotations

import enum
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

try:
    from service.project import PROJECT_EXTENSIONS
except Exception:  # pragma: no cover - 工程层不可用时退化成内置一份

    PROJECT_EXTENSIONS = (".rwproj", ".robotproj", ".core")


__all__ = [
    "StartupKind",
    "StartupRequest",
    "URDF_EXTENSIONS",
    "parse_startup_args",
]


#: 机器人模型扩展名（``main.py`` 的历史行为：``argv[1]`` 一律当 URDF 载入）
URDF_EXTENSIONS = (".urdf", ".xacro")


class StartupKind(enum.Enum):
    """一次启动要做什么。

    用枚举而不是裸字符串/布尔：启动分支是"看得见的语义"，散落成 ``if suffix``
    或 ``has_project`` 这类开关后，新增一种入口就要翻遍全文件找分支。
    """

    #: 没有可识别的参数（正常双击 exe / 命令行裸启动）
    NONE = "none"
    #: 参数是机器人模型 → 走原来的 ``MainWindow(urdf_path)`` 分支
    URDF = "urdf"
    #: 参数是工程文件 → 窗口起来后走 ``open_project_path``（工程载入唯一入口）
    PROJECT = "project"
    #: 参数是目录：工程形态未定（bundle?），先单独一支，确认后收口
    DIRECTORY = "directory"


@dataclass(frozen=True)
class StartupRequest:
    """启动意图（DTO）。

    :attr:`kind` 为 :attr:`StartupKind.NONE` 时 :attr:`path` 一定是 ``None``。
    """

    kind: StartupKind
    path: Optional[Path] = None


def _resolve(raw: str) -> Optional[Path]:
    """把一个命令行参数变成绝对路径；给不出有效路径时返回 ``None``。

    Windows 的文件关联可能传 8.3 短路径（``C:\\PROGRA~1\\…``）或带引号的
    ``"%1"``，``Path`` + ``resolve()`` 会还原成普通路径；不存在的一律当没给，
    绝不带着一个幽灵路径去启动（那会在窗口构造期弹出难懂的错误）。
    """
    text = (raw or "").strip().strip('"').strip("'")
    if not text:
        return None
    try:
        candidate = Path(os.path.expanduser(text))
        if not candidate.exists():
            return None
        return candidate.resolve()
    except OSError:  # pragma: no cover - 路径过长/非法字符：按"没给参数"处理
        return None


def _classify(path: Path) -> StartupKind:
    """按扩展名与形态判定意图。

    后缀比较一律 ``lower()``：Windows 资源管理器会给 ``A.RWPROJ`` 这种大写后缀，
    不归一化的话"双击大写文件"就成了随机失败。
    """
    if path.is_dir():
        return StartupKind.DIRECTORY
    suffix = path.suffix.lower()
    if suffix in PROJECT_EXTENSIONS:
        return StartupKind.PROJECT
    if suffix in URDF_EXTENSIONS:
        return StartupKind.URDF
    return StartupKind.NONE


def parse_startup_args(argv: Optional[Sequence[str]] = None) -> StartupRequest:
    """解析启动参数，返回 :class:`StartupRequest`。

    只取**第一个**能识别的参数：多选几个文件后回车、或打包器塞了额外开关时，
    后面的参数一律忽略。同时开多个工程是另一件事（要动场景生命周期），
    这里宁可少做，也不要静默开错一个。

    以 ``-`` 开头的参数直接跳过：Qt 与 PyInstaller 都会往 argv 里塞自己的开关，
    把它们当路径会误判（而且它们多半不存在，会被 :func:`_resolve` 挡掉）。

    任何异常都退化成 :attr:`StartupKind.NONE`——参数解析失败绝不能挡住启动，
    用户至少还能从菜单打开工程。
    """
    try:
        args = list(sys.argv if argv is None else argv)
        for raw in args[1:]:
            if str(raw).startswith("-"):
                continue
            resolved = _resolve(str(raw))
            if resolved is None:
                continue
            kind = _classify(resolved)
            if kind is StartupKind.NONE:
                continue
            return StartupRequest(kind=kind, path=resolved)
    except Exception:  # pragma: no cover - 解析绝不能成为启动失败的理由
        pass
    return StartupRequest(kind=StartupKind.NONE)
