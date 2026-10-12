"""Windows 外壳集成测试。

这里断言两件事：

1. **纯函数**：``shell\\open\\command`` 的命令行拼接（``"%1"`` 必须带引号，否则
   路径里有空格时 Explorer 会把路径切成两段，症状是"有的文件双击能开、
   有的打不开"）。这部分在所有平台上都跑；
2. **no-op 契约**：非 Windows 平台上所有函数必须返回 ``False`` 且不抛异常。
   :mod:`app.launcher` 是**无条件调用**它们的（平台分支只留在 platform_win
   内部），一旦在非 Windows 上炸了，macOS/Linux 就起不来软件。

真正的注册表读写与 Jump List 需要 Windows 环境，本机（macOS）不假装验证过。
"""

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from app import platform_win as pw  # noqa: E402


def test_open_command_quotes_both_program_and_argument():
    """程序路径与 ``%1`` 都要带引号。

    ``C:\\Program Files\\…`` 这种路径不带引号会被切成两段参数。
    """
    line = pw._command_line(r"C:\Program Files\RoboWorld\RoboWorld.exe")
    assert line == r'"C:\Program Files\RoboWorld\RoboWorld.exe" "%1"'


def test_open_command_handles_paths_without_spaces():
    assert pw._command_line(r"C:\RW\RoboWorld.exe") == r'"C:\RW\RoboWorld.exe" "%1"'


@pytest.mark.skipif(pw.IS_WINDOWS, reason="真实外壳调用需要 Windows 环境")
def test_everything_is_a_noop_off_windows():
    """非 Windows：一律 False，绝不抛异常（launcher 无条件调用）。"""
    assert pw.set_app_user_model_id() is False
    assert pw.add_recent_document("/tmp/demo.rwproj") is False
    assert pw.notify_shell_associations_changed() is False
    assert pw.register_file_association("RoboWorld.exe") is False
    assert pw.unregister_file_association() is False


@pytest.mark.skipif(pw.IS_WINDOWS, reason="真实外壳调用需要 Windows 环境")
def test_association_is_never_registered_off_windows():
    assert pw.is_association_registered() is False


def test_module_imports_without_windows_dlls():
    """``ctypes.windll`` 只能在惰性路径里碰：模块级碰会让非 Windows 导入失败。

    这条在 Windows 上也成立（导入本身不触发任何外壳调用）。
    """
    assert pw.IS_WINDOWS == (sys.platform == "win32")
    assert pw._shell32() is None or pw.IS_WINDOWS
