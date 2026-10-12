"""启动编排：串起「启动壳」与「窗口」之间那几步有先后依赖的动作。

``main.py`` 只做三件事（``sys.path`` 前导、崩溃处理、拉窗口）。而"双击一个
工程文件把软件叫起来"实际要做的，是一串**顺序不能换**的事：

1. 建 ``QApplication``——任何 Qt 动作的前提；
2. 声明 Windows 任务栏身份（AppUserModelID）——**必须在任何窗口出现之前**，
   否则 Windows 已经替我们猜过一个身份，Jump List 的"最近"会挂到别处；
3. 解析 argv → 这次启动到底要开什么（:mod:`app.startup_args`）；
4. 单实例裁决：已经有一个实例在跑 → 把路径交给它，自己**立刻退出**；
5. 建窗口（URDF 走构造参数，工程**不能**走——见 :func:`_open_request`）；
6. 窗口真正显示之后，再打开工程。

为什么单独成文件：这四件事（身份 / 解析 / 单实例 / 延后开工程）加起来上百行，
塞回 ``main.py`` 会让它从"启动壳"变成"第二个窗口模块"；放在这里，
``main.py`` 只多两行，且编排逻辑可以被测试直接调用。
"""

from __future__ import annotations

import sys
from typing import Optional, Sequence

from PySide6 import QtCore, QtWidgets

from app.app_identity import APPLICATION_ID
from app.platform_win import set_app_user_model_id
from app.single_instance import SingleInstance
from app.startup_args import StartupKind, StartupRequest, parse_startup_args
from app.window.main_window import MainWindow

__all__ = ["run_app"]


def _declare_shell_identity() -> None:
    """向系统声明"我是谁"（非 Windows 上是 no-op）。

    失败就当没发生过：任务栏分组与 Jump List 是增强能力，不是启动前提。
    """
    set_app_user_model_id(APPLICATION_ID)


def _build_window(request: StartupRequest) -> MainWindow:
    """按启动意图建窗口。

    只有 URDF 走构造参数（``MainWindow(urdf_path)`` 的历史路径）。工程**不能**
    走构造参数：``showEvent`` 里会载入默认机器人并询问崩溃恢复，工程若在那之前
    载入会被默认机器人整个盖掉——表现为"双击打开了工程，但看到的是空场景"。
    """
    if request.kind is StartupKind.URDF and request.path is not None:
        return MainWindow(str(request.path))
    return MainWindow()


def _open_request(window: MainWindow, request: StartupRequest) -> None:
    """窗口就绪后执行启动意图。"""
    if request.path is None:
        return
    if request.kind is StartupKind.URDF:
        window.load_robot(str(request.path))
        return
    if request.kind in (StartupKind.PROJECT, StartupKind.DIRECTORY):
        # 目录形态（bundle?）尚未确认：暂按单文件工程处理。Project.load 读到
        # 目录会给出明确的 ProjectError，不会静默——确认形态后在此收口
        window.open_project_path(str(request.path))


def _focus_window(window: MainWindow) -> None:
    """把已运行的窗口提到前台（第二个实例空手启动时的归宿）。"""
    window.raise_()
    window.activateWindow()


def _on_path_received(window: MainWindow, payload: str) -> None:
    """主实例侧：处理后续进程转交过来的路径。

    空载荷 = 对方只是又双击了一次 exe（没有文件）→ 把窗口提前台即可。
    非空则**重新走一遍分类**：转交来的可能是工程也可能是 URDF，判定规则
    只有 :func:`parse_startup_args` 一份，这里不复制分支。
    """
    if not payload:
        _focus_window(window)
        return
    _open_request(window, parse_startup_args(["roboworld", payload]))


def run_app(argv: Optional[Sequence[str]] = None) -> int:
    """跑完一次启动，返回进程退出码。

    :param argv: 命令行参数（含 ``argv[0]``），缺省取 ``sys.argv``
    """
    args = list(sys.argv if argv is None else argv)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(args)

    _declare_shell_identity()

    request = parse_startup_args(args)
    guard = SingleInstance()

    if not guard.try_become_primary():
        # 已经有人在跑了：把意图交出去就走。hand_off 失败（连不上那个其实
        # 已经死了的旧实例）时继续正常启动，不把用户挡在门外
        guard.hand_off(str(request.path) if request.path is not None else "")
        return 0

    window = _build_window(request)
    window.show()

    if request.path is not None:
        # 排到事件循环里：等 showEvent 把默认机器人/崩溃恢复处理完再开工程
        QtCore.QTimer.singleShot(0, lambda: _open_request(window, request))

    guard.pathReceived.connect(lambda payload: _on_path_received(window, payload))

    return app.exec()
