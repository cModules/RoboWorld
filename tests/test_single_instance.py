"""单实例守卫测试。

本模块只用 ``QLocalServer`` / ``QLocalSocket``，不需要 GUI——但下面那个 fixture
**创建的是 QApplication，不是 QCoreApplication**，原因见它的注释（进程里第一个
Qt 应用对象的类型决定了后面能不能造 QWidget，挑错了会直接 abort）。本模块自己
仍然不创建任何窗口，所以"无头环境能跑"这个性质没变。

最要紧的一条是 :func:`test_second_launch_yields_to_the_running_instance`：
注册文件关联之后，每双击一个工程都会**启动一个新进程**；若第二个进程也认为
自己是主实例，用户就会看到 N 个窗口各开各的，最近打开与自动保存互相覆盖。
这个 bug 只在"第二次启动"时暴露——单次冒烟永远发现不了，所以必须有这条测试。
"""

import sys
import time
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from PySide6 import QtCore, QtNetwork, QtWidgets  # noqa: E402

from app.single_instance import SingleInstance  # noqa: E402


@pytest.fixture(scope="module")
def qcore_app():
    """模块级 Qt 应用对象：socket 握手要靠事件循环把数据送到主实例。

    **没有实例时建 QApplication，而不是 QCoreApplication**（2026-10-12 实测）：
    Qt 进程里第一个应用对象的类型就是"这个进程有没有 GUI"的答案，一旦先建了
    QCoreApplication，之后任何模块再建 QApplication 都会在 macOS 上直接
    ``Fatal Python error: Aborted``，栈停在 QWidget 构造那一行，看不出原因。
    症状还与**跑测试的顺序**有关——全量跑（字母序 single_instance 在 launcher
    之后）恰好不触发，单独/分组跑就崩，是典型的"只在分片 CI 上出现"的坑。

    本模块不建窗口，所以借用 QApplication 不会额外要求显示环境；有 Qt 的机器
    上都可用（无头的 CI 由 ``QT_QPA_PLATFORM=offscreen`` 覆盖，与 GUI 测试同）。
    """
    app = QtCore.QCoreApplication.instance()
    if app is None:
        app = QtWidgets.QApplication([])
    return app


@pytest.fixture
def guards(qcore_app):
    """产出守卫并在用例结束时**彻底**拆掉监听。

    不清理的话，上一个用例留下的 socket 会让下一个用例的"第二次启动"判定
    变成"已经有主实例"——测试之间互相污染，表现为红绿随机。
    """
    made = []

    def _make(app_id: str = "com.openrobot.work.robotworld") -> SingleInstance:
        guard = SingleInstance(app_id=app_id)
        made.append(guard)
        return guard

    yield _make

    for guard in made:
        try:
            if guard._server is not None:
                guard._server.close()
        except Exception:
            pass
        try:
            QtNetwork.QLocalServer.removeServer(guard.server_name)
        except Exception:
            pass


def _wait_until(predicate, qcore_app, timeout: float = 3.0) -> bool:
    """把事件循环转起来，等到 predicate 成立或超时。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        qcore_app.processEvents()
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_first_launch_becomes_primary(guards, qcore_app):
    assert guards().try_become_primary() is True


def test_second_launch_yields_to_the_running_instance(guards, qcore_app):
    """第二次启动必须让位——这是文件关联能用起来的前提。"""
    primary = guards()
    assert primary.try_become_primary() is True

    secondary = guards()
    assert secondary.try_become_primary() is False


def test_hand_off_delivers_path_to_primary(guards, qcore_app):
    """第二个进程把路径交给主实例，自己退出。"""
    primary = guards()
    assert primary.try_become_primary() is True
    received = []
    primary.pathReceived.connect(received.append)

    assert guards().hand_off("/tmp/demo.rwproj") is True
    assert _wait_until(lambda: bool(received), qcore_app)
    assert received == ["/tmp/demo.rwproj"]


def test_hand_off_without_primary_returns_false(qcore_app):
    """没有主实例时转交失败 → 调用方应当自己正常启动，而不是退出。"""
    orphan = SingleInstance(app_id="com.openrobot.work.robotworld-nobody-listening")
    try:
        assert orphan.hand_off("/tmp/demo.rwproj") is False
    finally:
        try:
            QtNetwork.QLocalServer.removeServer(orphan.server_name)
        except Exception:
            pass


def test_empty_payload_means_focus_request(guards, qcore_app):
    """空载荷 = "只是又双击了 exe"：主实例收到空串，据此把窗口提到前台。

    ``QLocalSocket`` 写 0 字节不可靠，所以协议上用不可见字符当替身；读回时
    必须还原成空串——还原错了就会去打开一个叫 "" 的工程。
    """
    primary = guards()
    assert primary.try_become_primary() is True
    received = []
    primary.pathReceived.connect(received.append)

    assert guards().hand_off("") is True
    assert _wait_until(lambda: bool(received), qcore_app)
    assert received == [""]


def test_server_name_is_scoped_by_user():
    """socket 名按用户隔离：多用户/远程桌面上不该串台。"""
    first = SingleInstance()
    second = SingleInstance(app_id="com.openrobot.work.robotworld")
    assert first.server_name == second.server_name
    assert first.server_name != SingleInstance(app_id="other-app").server_name
    # 用户名本身不出现在 socket 名里（可能含不能用作文件名的字符）
    assert first.server_name.endswith(tuple("0123456789abcdef"))
