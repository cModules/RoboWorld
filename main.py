#!/usr/bin/env python3
"""RoboWorld - OLP Terminal（启动壳）。

ADR-002 拆解完成态：窗口与视图已全部落地 ``app/``，本文件**不再定义任何类**，
只做三件事——``sys.path`` 前导、崩溃处理、拉起主窗口。

保留的那些导入是**历史调用面**：测试与工具仍按 ``main.<name>`` 取符号
（``main.MainWindow`` / ``main.GLWidget`` / ``main.plugin_registry`` …），
删掉会静默断链——调用方不会报错，只会拿到 ``AttributeError`` 或错误的空值。
故此处只转发、不实现；真正的实现与降级逻辑都在各自的归属模块里。
"""

import os
import sys
from pathlib import Path

# 本文件既可作为脚本直接运行，也可被打包；确保仓库根下的包可被导入
_REPO_ROOT = Path(__file__).resolve().parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from PySide6 import QtGui, QtWidgets

# 版本号（崩溃日志落盘用；窗口标题的那份在 app/window/main_window.py 里派生）
try:
    from support import __version__ as _CORE_VERSION
except Exception:  # pragma: no cover - 仅用于包不可用的降级
    _CORE_VERSION = "0.0.0-dev"

try:
    from support.i18n import tr
except Exception:  # pragma: no cover - i18n 缺失时退化为恒等

    def tr(s):
        return s


try:
    from app.features import describe as _features_describe
except Exception:  # pragma: no cover - features 缺失时按全功能处理

    def _features_describe() -> str:
        return "full（features 模块缺失，按全功能降级）"


# --------------------------------------------------------------------------- #
# 历史调用面：只转发、不实现（真正的所有者是 app/ 下各模块）
# --------------------------------------------------------------------------- #
from app.views.gl_widget import GLWidget  # noqa: E402
from app.views.path_editor import PathCodeEditor  # noqa: E402
from app.views.property_panel import DevicePropertyPanel  # noqa: E402
from app.views.robot_renderer import (  # noqa: E402
    HOME_POSE_MARGIN,
    HOME_POSE_PATTERN_DEG,
    RobotRenderer,
)
from app.launcher import run_app  # noqa: E402
from app.window.main_window import MainWindow  # noqa: E402
from domain.pose import matrix_to_rpy as _matrix_to_rpy  # noqa: E402

# 插件注册表：tests/test_app_degraded_paths.py 断言
# ``main.plugin_registry.add_error`` 可调用（"default_registry 未绑定"的回归）
try:
    from adapter.plugins import default_registry as plugin_registry

    _HAS_PLUGINS = True
except Exception:  # pragma: no cover - 插件层缺失时安全降级
    _HAS_PLUGINS = False
    plugin_registry = None


# --------------------------------------------------------------------------- #
# 崩溃处理：打包出去的是 console=False 的 GUI，Windows 上没有终端，未捕获异常的
# 对话框一闪而过（macOS 双击 dist 里的 app 同样看不到任何输出），现场问题最后只能
# 靠口头猜。这里把 traceback 落到与自动保存同一处的日志目录，配合崩溃自动恢复，
# 至少保证「现场有据可查」。
# --------------------------------------------------------------------------- #
def crash_log_dir() -> "Path":
    """崩溃日志目录（``$CORE_CONFIG_DIR/logs`` > ``~/.core/logs``）。

    与 :mod:`app.session` 的用户级配置根目录同口径：找自动保存的文件在一个
    地方，找崩溃日志也在同一个地方。
    """
    base = os.environ.get("CORE_CONFIG_DIR")
    return (Path(base) if base else Path.home() / ".core") / "logs"


def write_crash_log(exc_type, exc_value, exc_tb) -> "Path | None":
    """把一次未捕获异常写成崩溃日志；写不了返回 None（绝不二次抛错）。"""
    import datetime as _dt
    import platform as _platform
    import traceback as _tb

    try:
        log_dir = crash_log_dir()
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = _dt.datetime.now().strftime("%Y%m%d-%H%M%S")
        path = log_dir / f"crash-{stamp}.log"
        lines = [
            f"RoboWorld v{_CORE_VERSION} 崩溃日志",
            f"时间: {_dt.datetime.now().isoformat(timespec='seconds')}",
            f"平台: {_platform.platform()}",
            f"Python: {_platform.python_version()}",
            f"构建档位: {_features_describe()}",
            "",
            *_tb.format_exception(exc_type, exc_value, exc_tb),
        ]
        path.write_text("\n".join(lines), encoding="utf-8")
        return path
    except Exception:
        return None


def _install_crash_handler() -> None:
    """安装 ``sys.excepthook``：先落盘并给一句人话，再交回默认钩子。"""
    default_hook = sys.excepthook

    def _hook(exc_type, exc_value, exc_tb):
        path = write_crash_log(exc_type, exc_value, exc_tb)
        if path is None:
            default_hook(exc_type, exc_value, exc_tb)
            return
        try:
            # 事件循环可能已经瘫了：弹窗失败不影响日志（日志已经写完）
            if QtWidgets.QApplication.instance() is not None:
                QtWidgets.QMessageBox.critical(
                    None,
                    tr("程序异常"),
                    tr(
                        "程序遇到未处理的异常，当前操作已停止。\n\n"
                        "崩溃日志已写入：\n{p}\n\n"
                        "自动保存的那份工程还在，重新打开会提示恢复。"
                    ).format(p=path),
                )
        except Exception:
            pass
        default_hook(exc_type, exc_value, exc_tb)

    sys.excepthook = _hook


def main():
    fmt = QtGui.QSurfaceFormat()
    fmt.setVersion(2, 1)
    fmt.setProfile(QtGui.QSurfaceFormat.NoProfile)
    QtGui.QSurfaceFormat.setDefaultFormat(fmt)

    # 必须在构造任何窗口之前挂好：窗口构造期抛异常也算
    _install_crash_handler()

    # 启动编排在 app/launcher.py（系统身份 / 单实例转交 / 双击打开工程）。
    # 这里保持"启动壳"的口径：argv[1] 不再无条件当 URDF——双击 .rwproj 时
    # 它是工程文件，按老写法会被当成机器人模型去解析
    sys.exit(run_app(sys.argv))


if __name__ == "__main__":
    main()
