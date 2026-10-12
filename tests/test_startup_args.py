"""启动参数解析测试——"双击能不能打开工程"这条链的第一环。

系统（Windows 资源管理器 / macOS LaunchServices / Linux 桌面）只做一件事：
把被双击的文件路径塞进 ``argv``。它**不告诉**我们这是什么，判定全在这里。
判错了的表现很隐蔽：软件能起来、窗口正常，就是没打开用户点的那个文件。

为什么不依赖 Qt：判定规则与界面无关，挪进 :mod:`app.startup_args` 之后
在无头环境（CI / Linux 无头）也能直接跑，不需要 offscreen 插件。
"""

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from app.startup_args import (  # noqa: E402
    URDF_EXTENSIONS,
    StartupKind,
    parse_startup_args,
)
from service.project import PROJECT_EXTENSIONS  # noqa: E402


def _argv(*args: str):
    """造一份 argv：``argv[0]`` 是程序名，解析时被跳过。"""
    return ["roboworld", *args]


def _touch(tmp_path: Path, name: str) -> str:
    """在临时目录里造一个文件，返回它的路径字符串。"""
    target = tmp_path / name
    target.write_text("{}", encoding="utf-8")
    return str(target)


# --------------------------------------------------------------------------- #
# 无参数 / 无效参数
# --------------------------------------------------------------------------- #


def test_no_args_is_none():
    request = parse_startup_args(_argv())
    assert request.kind is StartupKind.NONE
    assert request.path is None


def test_empty_argv_does_not_raise():
    """连 argv[0] 都没有：解析失败绝不能挡住启动。"""
    assert parse_startup_args([]).kind is StartupKind.NONE


def test_empty_string_arg_is_none():
    assert parse_startup_args(_argv("")).kind is StartupKind.NONE


def test_missing_path_is_none(tmp_path):
    """文件不存在 → 当没给参数。

    带着一个幽灵路径去启动，会在窗口构造期弹出用户看不懂的错误。
    """
    request = parse_startup_args(_argv(str(tmp_path / "nope.rwproj")))
    assert request.kind is StartupKind.NONE
    assert request.path is None


def test_unknown_suffix_is_none(tmp_path):
    """未知后缀一律忽略：宁可不开，也不要拿它去猜。"""
    path = _touch(tmp_path, "readme.txt")
    assert parse_startup_args(_argv(path)).kind is StartupKind.NONE


# --------------------------------------------------------------------------- #
# 工程 / URDF / 目录分流
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("suffix", PROJECT_EXTENSIONS)
def test_all_project_extensions_open_as_project(tmp_path, suffix):
    """主扩展名与两个兼容别名都要能被识别成"打开工程"。"""
    path = _touch(tmp_path, f"cell{suffix}")
    request = parse_startup_args(_argv(path))
    assert request.kind is StartupKind.PROJECT
    assert request.path == Path(path).resolve()


def test_uppercase_extension_is_recognized(tmp_path):
    """大写后缀也要认：Windows 资源管理器会给 ``A.RWPROJ``。

    不归一化的话"双击大写文件"就成了随机失败——同一份工程，改个名就打不开。
    """
    path = _touch(tmp_path, "CELL.RWPROJ")
    assert parse_startup_args(_argv(path)).kind is StartupKind.PROJECT


def test_urdf_argument_stays_on_the_urdf_branch(tmp_path):
    """URDF 保持历史行为：走 ``MainWindow(urdf_path)`` 那条路。"""
    path = _touch(tmp_path, "ur5e.urdf")
    request = parse_startup_args(_argv(path))
    assert request.kind is StartupKind.URDF
    assert request.path == Path(path).resolve()
    assert ".urdf" in URDF_EXTENSIONS


def test_directory_is_directory_not_project(tmp_path):
    """目录单独判一支：工程形态（bundle?）未确认前不直接当工程打开。"""
    request = parse_startup_args(_argv(str(tmp_path)))
    assert request.kind is StartupKind.DIRECTORY
    assert request.path == tmp_path.resolve()


def test_home_directory_argument_is_resolved():
    """``~`` 要展开：shell 传进来的可能是缩写形式。"""
    request = parse_startup_args(_argv("~"))
    assert request.kind is StartupKind.DIRECTORY
    assert request.path == Path.home()


# --------------------------------------------------------------------------- #
# 参数清洗
# --------------------------------------------------------------------------- #


def test_option_like_arguments_are_skipped(tmp_path):
    """``--xxx`` 开关不当路径：Qt 与 PyInstaller 都会往 argv 里塞自己的开关。"""
    path = _touch(tmp_path, "cell.rwproj")
    request = parse_startup_args(_argv("--platform", "offscreen", path))
    assert request.kind is StartupKind.PROJECT
    assert request.path == Path(path).resolve()


def test_quoted_path_is_unwrapped(tmp_path):
    """带引号的参数要脱壳：文件关联的 ``"%1"`` 就是带引号传进来的。"""
    path = _touch(tmp_path, "cell.rwproj")
    request = parse_startup_args(_argv(f'"{path}"'))
    assert request.kind is StartupKind.PROJECT
    assert request.path == Path(path).resolve()


def test_first_recognized_argument_wins(tmp_path):
    """多选几个文件后回车：只认第一个能识别的，其余忽略。

    同时开多个工程是另一件事（要动场景生命周期），宁可少做也不静默开错。
    """
    good = _touch(tmp_path, "first.rwproj")
    other = _touch(tmp_path, "second.rwproj")
    request = parse_startup_args(_argv(str(tmp_path / "missing.rwproj"), good, other))
    assert request.path == Path(good).resolve()


def test_unrecognized_first_argument_does_not_stop_the_scan(tmp_path):
    """第一个参数不认识（比如 readme.txt）时，继续往后找，不要直接放弃。"""
    path = _touch(tmp_path, "cell.rwproj")
    stray = _touch(tmp_path, "notes.txt")
    request = parse_startup_args(_argv(stray, path))
    assert request.kind is StartupKind.PROJECT
    assert request.path == Path(path).resolve()


# --------------------------------------------------------------------------- #
# 单一真源
# --------------------------------------------------------------------------- #


def test_project_extensions_come_from_service_not_a_second_copy():
    """后缀表必须是 service.project 那一份，不是这里另抄的。

    两处漂移的典型症状：文件对话框能打开、双击却没反应。
    """
    from app import startup_args

    assert startup_args.PROJECT_EXTENSIONS == PROJECT_EXTENSIONS
