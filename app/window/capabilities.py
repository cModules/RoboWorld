"""窗口层可选依赖的**单一探测点**（C2：消除降级块的 N 份拷贝）。

背景：ADR-002 拆解时，每个 mixin 都把自己用到的可选依赖连同 ``try/except``
降级块一起搬了过去——结果同一个依赖被探测了 5～6 遍（``_HAS_PICKPLACE`` 6 个
模块、``_HAS_WORLD`` 5 个），全仓 33 个 try 块。三处代价：

1. **降级口径会漂移**：某天有人只在一个模块里改了 fallback 值，同一个依赖就
   有了两种"不可用时的样子"；
2. **刷屏**：缺依赖时同一句 ``[warning] … unavailable`` 被打 6 遍；
3. **探测本身被重复执行**（import 是有成本的）。

集中到本模块后：每个依赖**只探测一次**，各 mixin 只做 ``from … import``。
降级语义（失败时置什么、打印什么）只有一份真源。

放在 ``app/window/`` 而不是 ``support/``：本模块要探测 domain / service /
adapter 三层，而 H8 要求 ``support/`` 不依赖任何层。

注意：被搬走的 mixin 曾各自持有这些名字，**测试按
``hasattr(mod, "_HAS_WORLD")`` 断言**（见 tests/test_app_window_mixins.py
④「自带依赖」）——改为 ``from … import`` 后该断言依然成立，语义未变。
"""

from __future__ import annotations

from app.logging import logger

# --------------------------------------------------------------------------- #
# i18n（中文即 key；缺失时退化为恒等）
# --------------------------------------------------------------------------- #
try:
    from support.i18n import available_languages, current_language, set_language, tr
except Exception:  # pragma: no cover - i18n 缺失时退化为恒等

    def tr(s):
        return s

    def available_languages():
        return []

    def current_language():
        return "zh"

    def set_language(lang, **kw):
        return lang


# 版本号（崩溃日志与窗口标题都从库版本派生，不手写）
try:
    from support import __version__ as _CORE_VERSION
except Exception:  # pragma: no cover - 仅用于包不可用的降级
    _CORE_VERSION = "0.0.0-dev"

# 构建档位开关（``app/features.py``）。缺失时按**全功能**处理：宁可多暴露，
# 也不要因为依赖问题把已经能用的功能锁死。
# 只导出 ``enabled``：``describe`` 仅启动壳（main.py）写崩溃日志时用，它自己有
# 一份降级——这里给 window 层多导一个没人用的符号，会凭空多一条需登记的中文串。
try:
    from app.features import enabled as _feature_enabled
except Exception:  # pragma: no cover - features 模块缺失时按全功能处理

    def _feature_enabled(name: str) -> bool:
        return True


# --------------------------------------------------------------------------- #
# 机器人库（组件库面板）。缺失时降级为纯手工加载 URDF，不影响主程序启动
# --------------------------------------------------------------------------- #
try:
    from adapter.robot_library.panel import RobotLibraryPanel

    _HAS_ROBOT_LIBRARY = True
except Exception as _exc:  # pragma: no cover
    logger.warning("robot_library_unavailable: %s", _exc)
    _HAS_ROBOT_LIBRARY = False
    RobotLibraryPanel = None

# --------------------------------------------------------------------------- #
# 环境建模（工作单元场景 + 参数化组件库 + 干涉检查）
# --------------------------------------------------------------------------- #
try:
    from domain.world.panel import ScenePanel
    from domain.world.scene import ROBOT_NAME, Scene

    _HAS_WORLD = True
except Exception as _exc:  # pragma: no cover
    logger.warning("world modeling unavailable: %s", _exc)
    _HAS_WORLD = False
    ScenePanel = None
    Scene = None
    ROBOT_NAME = ""

# --------------------------------------------------------------------------- #
# 工作单元模型 + PLC（西门子 SCL）代码生成
# --------------------------------------------------------------------------- #
try:
    from service.cell import HANDSHAKE_SIGNALS

    _HAS_CELL = True
except Exception as _exc:  # pragma: no cover
    logger.warning("cell model unavailable: %s", _exc)
    _HAS_CELL = False
    HANDSHAKE_SIGNALS = ()

# --------------------------------------------------------------------------- #
# 抓取工艺（示教点语义 → IO 指令 + 吸附事件）
# --------------------------------------------------------------------------- #
try:
    from service.pickplace import PointType

    _HAS_PICKPLACE = True
except Exception as _exc:  # pragma: no cover
    logger.warning("pickplace unavailable: %s", _exc)
    _HAS_PICKPLACE = False
    PointType = None

# --------------------------------------------------------------------------- #
# 工作单元模板 + 一键演示示教点（抓取工作站演示入口）
# --------------------------------------------------------------------------- #
try:
    from domain.world.demo import demo_taught_points
    from domain.world.templates import build_grasping_workstation

    _HAS_TEMPLATES = True
except Exception as _exc:  # pragma: no cover
    logger.warning("workcell templates unavailable: %s", _exc)
    _HAS_TEMPLATES = False
    build_grasping_workstation = None
    demo_taught_points = None

# 位姿数学（纯 Domain：URDF 变换与 RPY 互转）。无外部依赖，若它失败说明 Domain
# 层整个不可用——故不设 ``_HAS_*`` 标志（设了也没人看，反而是个僵尸开关）。
from domain.pose import Pose

# 仓库根定位器（处理 PyInstaller 的 ``_MEIPASS``，比自己拼 ``__file__`` 稳）
from domain.world.templates import REPO_ROOT as _REPO_ROOT

# --------------------------------------------------------------------------- #
# 离线编程面板 / 真机连接面板（app 自己的视图）
# --------------------------------------------------------------------------- #
try:
    from app.views.program_panel import ProgramPanel

    _HAS_PROGRAM = True
except Exception as _exc:  # pragma: no cover
    logger.warning("program panel unavailable: %s", _exc)
    _HAS_PROGRAM = False
    ProgramPanel = None

try:
    from app.views.controller_panel import ControllerPanel

    _HAS_CONTROLLER_PANEL = True
except Exception as _exc:  # pragma: no cover
    logger.warning("controller panel unavailable: %s", _exc)
    _HAS_CONTROLLER_PANEL = False
    ControllerPanel = None

# --------------------------------------------------------------------------- #
# 插件接口（渲染覆盖层 + 第三方后处理/运动学）
# --------------------------------------------------------------------------- #
try:
    from adapter.plugins import (
        OverlayContext,
        default_registry as plugin_registry,
        load_entry_points,
    )

    _HAS_PLUGINS = True
except Exception as _exc:  # pragma: no cover
    logger.warning("plugin layer unavailable: %s", _exc)
    _HAS_PLUGINS = False
    OverlayContext = None
    plugin_registry = None
    load_entry_points = None

# --------------------------------------------------------------------------- #
# 缓存可观测性：三类结果缓存的统一快照（规划分段 / 干涉检查 / 力矩校验）
# --------------------------------------------------------------------------- #
try:
    from service.incremental import (
        cache_overview,
        clear_all_caches,
        reset_all_cache_stats,
    )

    _HAS_INCREMENTAL = True
except Exception as _exc:  # pragma: no cover
    logger.warning("cache overview unavailable: %s", _exc)
    _HAS_INCREMENTAL = False
    cache_overview = None
    clear_all_caches = None
    reset_all_cache_stats = None

# --------------------------------------------------------------------------- #
# 工程文件（.rwproj，兼容 .robotproj / .core）与工艺时序
# --------------------------------------------------------------------------- #
try:
    from service.project import (
        PROJECT_EXTENSION,
        PROJECT_EXTENSIONS,
        PROJECT_FILTER,
        Project,
        ProjectError,
        cell_from_snapshot,
        cell_snapshot,
    )

    _HAS_PROJECT = True
except Exception as _exc:  # pragma: no cover
    logger.warning("project file support unavailable: %s", _exc)
    _HAS_PROJECT = False
    Project = None
    ProjectError = ValueError
    PROJECT_FILTER = ""
    # 降级值必须与 service.project 同口径：这里若仍写 .core，工程层不可用时会
    # 把"保存"默认存成已被取代的后缀，用户拿着文件去问"为什么打不开"
    PROJECT_EXTENSION = ".rwproj"
    PROJECT_EXTENSIONS = (".rwproj", ".robotproj", ".core")
    cell_snapshot = None
    cell_from_snapshot = None

try:
    from service.sequence import ProcessSequence

    _HAS_SEQUENCE = True
except Exception as _exc:  # pragma: no cover
    logger.warning("process sequence unavailable: %s", _exc)
    _HAS_SEQUENCE = False
    ProcessSequence = None

# --------------------------------------------------------------------------- #
# 会话支撑（最近打开 + 崩溃自动恢复）与工程模板：都依赖工程文件层
# --------------------------------------------------------------------------- #
if _HAS_PROJECT:
    try:
        from app.session import (
            RecentProjects,
            clear_autosave,
            read_autosave,
            write_autosave,
        )

        _HAS_SESSION_SUPPORT = True
    except Exception as _exc:  # pragma: no cover
        logger.warning("session support unavailable: %s", _exc)
        _HAS_SESSION_SUPPORT = False
        RecentProjects = None
        clear_autosave = None
        read_autosave = None
        write_autosave = None
else:
    _HAS_SESSION_SUPPORT = False
    RecentProjects = None
    clear_autosave = None
    read_autosave = None
    write_autosave = None

if _HAS_PROJECT:
    try:
        from service.project_templates import (
            build_project_template,
            list_project_templates,
        )

        _HAS_PROJECT_TEMPLATES = True
    except Exception as _exc:  # pragma: no cover
        logger.warning("project templates unavailable: %s", _exc)
        _HAS_PROJECT_TEMPLATES = False
        build_project_template = None
        list_project_templates = None
else:
    _HAS_PROJECT_TEMPLATES = False
    build_project_template = None
    list_project_templates = None
