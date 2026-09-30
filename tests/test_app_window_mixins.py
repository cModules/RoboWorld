"""`MainWindow` 拆解（ADR-002 Step 3 起）的守门测试。

拆解本身没有功能行为可测（方法原文一字未改），会退化的是**结构纪律**。这里钉住
五条——缺一条，搬迁就会在两三次提交内慢慢烂回去：

1. **真实发生**：这些方法必须解析到 ``app/window/*``，且**不再留在** ``main.py``
   的 ``MainWindow`` 里——否则"搬了个空"（方法还在原地）测试不会变红；
2. **宿主接线**：``MainWindow`` 以 mixin 继承它们，且 **MRO 排在 QMainWindow 之前**
   （混入放后面会被 Qt 的同名方法遮住，且必须是显式接线、不靠巧合）；
3. **方向**：``app/window/*.py`` 不得 import ``main``（方向单向：入口 import 本包）；
4. **自带依赖**：可选依赖与降级标志必须在各自模块的命名空间里，**不能靠 main 提供**
   ——否则模块在 main 之外（单测、未来 CLI）就不可用；
5. **门禁同步**：新文件必须在守护的 CJK 基线里有条目（搬迁最容易漏的一步：中文串
   跟着代码进了新文件，而守卫按文件记账，漏登记立刻是一片无关硬违规——实测过）。
"""

from __future__ import annotations

import ast
import os
import sys
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
pytest.importorskip("PySide6")

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

_PKG = _ROOT / "app" / "window"

#: 模块自带的 dunder：任何模块里都成立，不算"靠入口脚本提供"。
#: （不排除它们，⑤ 那条测试会把 ``__file__`` 误报成"搬迁漏了"。）
_MODULE_DUNDERS = {
    "__file__",
    "__name__",
    "__doc__",
    "__package__",
    "__spec__",
    "__loader__",
    "__builtins__",
}

#: 第 3 步搬走的两组方法 → 目标模块
_MOVED = {
    "app.window.plugin_cache": (
        "_refresh_plugin_overlays",
        "toggle_plugin_overlays",
        "reload_plugins",
        "show_plugin_status",
        "_cache_status_text",
        "show_cache_status",
        "clear_caches",
    ),
    "app.window.controller_link": (
        "show_controller_dialog",
        "_controller_program_payload",
        "on_controller_joints_read",
    ),
    "app.window.program_ops": (
        "validate_program",
        "validate_all_programs",
        "_show_validation_summary",
        "save_path_code",
        "load_path_code",
        "generate_robot_code",
        "_show_generated_code",
        "_save_generated_code",
    ),
    "app.window.project_session": (
        "_project_name",
        "_update_window_title",
        "_set_project_dirty",
        "_collect_into",
        "_resolve_project_path",
        "_apply_project",
        "save_project",
        "open_project",
        "open_project_path",
        "_record_recent",
        "_rebuild_recent_menu",
        "_clear_recent_menu_records",
        "_write_autosave_snapshot",
        "_autosave_tick",
        "_clear_autosave_snapshot",
        "maybe_prompt_recovery",
        "new_project_from_template",
        "_choose_project_template",
    ),
    "app.window.misc_actions": (
        "open_grasp_wizard",
        "_on_grasp_cycle_applied",
        "_focus_tab",
        "check_collisions",
        "_robot_for_selection",
        "_robot_usable",
        "on_positions_parsed",
        "on_clear_requested",
        "reset_view",
    ),
    "app.window.multi_robot_sync": (
        "_register_robot_for_programming",
        "_restore_robot_programs",
        "_sync_program_points",
        "_editor_points_for",
        "_sync_viewport_paths_from_editor",
        "_on_robot_removed",
        "_workpiece_for",
        "_on_workpiece_combo_changed",
        "_on_program_robot_changed",
        "on_taught_points_changed",
        "_on_program_sync_requested",
    ),
    "app.window.playback": (
        "on_playback_requested",
        "on_playback_frame",
        "on_playback_stopped",
        "_halt_playback",
        "_apply_pick_event",
        "on_gripper_state_changed",
        "_update_io_status",
        "on_joint_changed",
        "on_joint_changed_for_record",
        "_sync_scene_joints",
        "on_path_point_move_requested",
        "_highlight_point_line",
    ),
    "app.window.robot_scene": (
        "_load_default_robot",
        "show_robot_library",
        "reload_robot_library",
        "_add_component_from_library",
        "_on_robot_dropped",
        "_adopt_scene",
        "load_preset_cell",
        "load_grasping_workstation_demo",
        "open_model",
        "_register_robot_in_scene",
        "load_robot",
    ),
}


def _imported_names(path: Path) -> set[str]:
    """模块顶层（含缩进的 try/except 内）import 到的模块名。"""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    out: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            out |= {a.name for a in node.names}
        elif isinstance(node, ast.ImportFrom) and not node.level and node.module:
            out.add(node.module)
    return out


def _defined_names(path: Path) -> set[str]:
    """模块里**自己**绑定过的名字（import / 赋值 / 函数 / 类 / except 绑定）。"""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    defined = {"self"}
    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            defined |= {a.asname or a.name.split(".")[0] for a in node.names}
        elif isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
            defined.add(node.id)
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            defined.add(node.name)
        elif isinstance(node, ast.ExceptHandler) and node.name:
            defined.add(node.name)
    return defined


def test_methods_really_moved_out_of_the_entry_shell():
    """① 搬迁真实发生：方法解析到新模块，且 main.py 里已**不再定义** MainWindow。

    第 10 步把整个类落地 ``app/window/main_window.py`` 后，启动壳只剩一条
    ``from app.window.main_window import MainWindow`` 的历史调用面——所以这里
    的判据从"类里不该有这些方法"升级为**"类不该在这里定义"**（更强的断言）。
    """
    import main

    for module_name, methods in _MOVED.items():
        for name in methods:
            owner = getattr(main.MainWindow, name)
            assert (
                owner.__module__ == module_name
            ), f"{name} 仍归属 {owner.__module__}——应已搬进 {module_name}"

    tree = ast.parse((_ROOT / "main.py").read_text(encoding="utf-8"))
    defined = {n.name for n in tree.body if isinstance(n, ast.ClassDef)}
    assert (
        "MainWindow" not in defined
    ), "MainWindow 应已整体落地 app/window/main_window.py；main.py 只能保留导入形式的调用面"
    assert main.MainWindow.__module__ == "app.window.main_window"


def test_window_shell_lives_in_the_window_package():
    """①-b 窗口骨架的**定义**在 app/window/，启动壳只是转发。

    与 ① 互补：那里证明"不在 main.py"，这里证明"确实在新家"（两个方向都钉住，
    否则删掉定义又忘了建新家也会让 ① 通过）。
    """
    from app.window import main_window

    assert isinstance(getattr(main_window, "MainWindow", None), type)
    assert main_window.MainWindow.__module__ == "app.window.main_window"
    for name in ("__init__", "setup_menu", "setup_ui", "_set_window_icon", "showEvent"):
        assert name in vars(
            main_window.MainWindow
        ), f"{name} 应随窗口骨架一起落在 app/window/main_window.py"


def test_main_window_inherits_mixins_before_qt_base():
    """② 宿主接线：mixin 必须排在同名方法可能被遮蔽之前（MRO）。"""
    import main
    from PySide6 import QtWidgets

    from app.window.controller_link import ControllerLinkMixin
    from app.window.plugin_cache import PluginCacheMixin

    mro = main.MainWindow.__mro__
    assert (
        PluginCacheMixin in mro and ControllerLinkMixin in mro
    ), "MainWindow 没有以 mixin 方式接线拆出去的组"
    qt_at = mro.index(QtWidgets.QMainWindow)
    assert mro.index(PluginCacheMixin) < qt_at, "mixin 排在 QMainWindow 之后，可能被遮蔽"
    assert mro.index(ControllerLinkMixin) < qt_at, "mixin 排在 QMainWindow 之后，可能被遮蔽"


def test_mixins_do_not_import_the_entry_shell():
    """③ 依赖方向：本包不得反向 import 入口脚本。"""
    for path in sorted(_PKG.glob("*.py")):
        bad = {m for m in _imported_names(path) if m == "main" or m.startswith("main.")}
        assert not bad, f"{path.relative_to(_ROOT).as_posix()} 反向依赖了入口脚本：{bad}"


def test_mixins_own_their_optional_dependencies():
    """④ 降级块跟着方法走：可选依赖与标志由各模块自己提供。"""
    from app.window import controller_link, plugin_cache

    for flag in ("_HAS_PLUGINS", "_HAS_INCREMENTAL"):
        assert hasattr(plugin_cache, flag), f"{flag} 应由 plugin_cache 自己定义"
    for name in (
        "OverlayContext",
        "plugin_registry",
        "load_entry_points",
        "cache_overview",
        "clear_all_caches",
        "reset_all_cache_stats",
        "tr",
    ):
        assert hasattr(plugin_cache, name), f"{name} 未带进 plugin_cache 命名空间"

    assert hasattr(controller_link, "_HAS_CONTROLLER_PANEL")
    for name in ("ControllerPanel", "math", "tr"):
        assert hasattr(controller_link, name), f"{name} 未带进 controller_link 命名空间"

    # 程序组没有可选依赖，但它的 Qt/GUI 名字必须自备（QFont 只在 QtGui 里）
    from app.window import playback, program_ops

    for name in ("QtWidgets", "QtGui", "QFont", "Path", "math", "tr"):
        assert hasattr(program_ops, name), f"{name} 未带进 program_ops 命名空间"

    # 回放组：真标志 + 语义枚举都要自带（PointType 缺失时"工件吸附"只能整体降级）
    for flag in ("_HAS_WORLD", "_HAS_PICKPLACE"):
        assert hasattr(playback, flag), f"{flag} 应由 playback 自己定义"
    for name in ("ROBOT_NAME", "PointType", "Optional", "math", "tr"):
        assert hasattr(playback, name), f"{name} 未带进 playback 命名空间"

    from app.window import multi_robot_sync

    for flag in ("_HAS_PROGRAM", "_HAS_WORLD", "_HAS_PICKPLACE"):
        assert hasattr(multi_robot_sync, flag), f"{flag} 应由 multi_robot_sync 自己定义"
    for name in ("Path", "Optional", "_REPO_ROOT", "tr"):
        assert hasattr(multi_robot_sync, name), f"{name} 未带进 multi_robot_sync 命名空间"

    # 工程/会话组：工程文件层与会话层都是可选依赖，标志与符号都要自带
    from app.window import project_session

    for flag in (
        "_HAS_PROJECT",
        "_HAS_SESSION_SUPPORT",
        "_HAS_PROJECT_TEMPLATES",
        "_HAS_SEQUENCE",
    ):
        assert hasattr(project_session, flag), f"{flag} 应由 project_session 自己定义"
    for name in (
        "Project",
        "ProjectError",
        "PROJECT_FILTER",
        "PROJECT_EXTENSION",
        "RecentProjects",
        "_CORE_VERSION",
        "math",
        "Path",
        "tr",
    ):
        assert hasattr(project_session, name), f"{name} 未带进 project_session 命名空间"

    # 零散动作组：QtWidgets + 抓取语义真标志（PointType 缺失时点类型回灌整体跳过）
    from app.window import misc_actions

    assert hasattr(misc_actions, "_HAS_PICKPLACE"), "_HAS_PICKPLACE 应由 misc_actions 自己定义"
    for name in ("QtWidgets", "Optional", "tr"):
        assert hasattr(misc_actions, name), f"{name} 未带进 misc_actions 命名空间"


def test_mixins_need_no_names_from_the_entry_shell():
    """⑤ 搬迁完整性：本模块用到的名字必须能自己解析，不得指望入口脚本提供。

    与 ``test_app_views_gl_widget.py`` / ``test_app_views_path_editor.py`` 同一条
    纪律——那里踩过真事故（模块级函数漏搬，测试却全绿）。
    """
    import main

    for path in sorted(_PKG.glob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        used = {
            n.id for n in ast.walk(tree) if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)
        }
        missing = sorted((used & set(dir(main))) - _defined_names(path) - _MODULE_DUNDERS)
        assert not missing, (
            f"{path.relative_to(_ROOT).as_posix()} 这些名字自己解析不了、"
            f"只能靠入口脚本提供（搬迁漏了）：{missing}"
        )


def test_preset_cell_dir_points_at_the_real_repo_cells():
    """搬迁专有陷阱：``__file__`` 相对路径在新模块里会**静默**指错目录。

    ``main.py`` 在仓库根，``Path(__file__).resolve().parent / "assets" / "cells"`` 是对的；
    同一个表达式搬进 ``app/window/`` 就变成 ``app/window/assets/cells``——一条预置场景都
    找不到，而界面只会说"未找到预置场景"，**不报错**（与 Step 1 漏搬模块级函数同型）。

    所以这里不查源码文本（注释里本来就会提到 ``__file__``，会被误伤），而是**真去问一次
    目录在哪**：``_REPO_ROOT`` 拼出来的路径必须真的存在、且里面有 .json。
    """
    from app.window import robot_scene

    cells = Path(robot_scene._REPO_ROOT) / "assets" / "cells"
    assert cells.is_dir(), f"预置场景目录解析错了（应指向仓库根 assets/cells）：{cells}"
    assert list(cells.glob("*.json")), f"{cells} 里没有预置场景 .json"


def test_repo_root_comes_from_the_shared_helper():
    """``_REPO_ROOT`` 的真源是 ``domain.world.templates.REPO_ROOT``（处理了 ``_MEIPASS``）。

    C2 收敛后各 mixin 改从 ``app.window.capabilities`` 取，而那里也只是转发——
    所以这里**不再查 robot_scene 的文本**（转发路径一变就误报），改为问**值**：
    沿途每一处 ``_REPO_ROOT`` 都必须等于 ``domain.world.templates.REPO_ROOT``。
    """
    from domain.world.templates import REPO_ROOT

    from app.window import capabilities, robot_scene

    assert robot_scene._REPO_ROOT == REPO_ROOT, "robot_scene 的仓库根与真源不一致"
    assert capabilities._REPO_ROOT == REPO_ROOT, "capabilities 的仓库根与真源不一致"
    text = (_PKG / "capabilities.py").read_text(encoding="utf-8")
    assert (
        "from domain.world.templates import REPO_ROOT as _REPO_ROOT" in text
    ), "真源入口应直接取 domain.world.templates.REPO_ROOT（不要自己拼 __file__）"


def test_window_icon_resolves_packaging_logo():
    """窗口图标必须使用 packaging/logo.png，不能回退到不存在的仓库根 logo。"""
    from app.window import main_window

    logo = Path(main_window._REPO_ROOT) / "packaging" / "logo.png"
    assert logo.is_file(), f"状态栏/窗口图标资源不存在：{logo}"
    assert logo.parent == _ROOT / "packaging"
    assert logo != _PKG / "logo.png", "图标路径落到了 app/window 包目录"


def test_constants_moved_with_their_group_keep_a_single_source():
    """随组搬走的常量不能留双份真源（两份早晚漂移）。"""
    import main

    from app.window import robot_scene

    assert robot_scene.DEFAULT_ROBOT_KEYS, "DEFAULT_ROBOT_KEYS 应随「机器人/场景加载」组一起搬来"
    assert not hasattr(
        main, "DEFAULT_ROBOT_KEYS"
    ), "main.py 里不该再留 DEFAULT_ROBOT_KEYS——唯二使用点都搬走了，留着就是第二份真源"


def test_new_files_registered_in_guard_baseline():
    """⑤ 结构门禁同步：新文件必须在守护的 CJK 基线里有条目。"""
    text = (_ROOT / "tools" / "check_arch_rules.py").read_text(encoding="utf-8")
    for module_name in _MOVED:
        rel = module_name.replace(".", "/") + ".py"
        assert f'"{rel}"' in text, (
            f"{rel} 未登记进 CJK_BASELINE —— 代码搬迁必须同步守护脚本"
            "（债随文件走：源文件下调、目标文件立键，总数守恒）"
        )
