#!/usr/bin/env python3
"""
RobotWorld - OLP Terminal
"""

import sys
import math
import re
import json
from typing import Dict, List, Optional, Tuple  # noqa: F401

import numpy as np
from pathlib import Path

# 本文件既可作为脚本直接运行，也可被打包；确保 python/ 下的 orolp 包可被导入
_REPO_ROOT = Path(__file__).resolve().parent
_PYTHON_DIR = _REPO_ROOT / "python"
if str(_PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(_PYTHON_DIR))

try:
    from orolp.i18n import available_languages, current_language, set_language, tr
except Exception:  # pragma: no cover - i18n 缺失时退化为恒等
    def tr(s):
        return s

    def available_languages():
        return []

    def current_language():
        return "zh"

    def set_language(lang, **kw):
        return lang

try:
    from PySide6 import QtWidgets, QtCore, QtOpenGLWidgets, QtGui
    from PySide6.QtCore import Qt, Signal
    from PySide6.QtGui import QFont, QAction, QKeySequence
except ImportError:
    print("PySide6 not installed. Install with: pip install PySide6")
    sys.exit(1)

try:
    import OpenGL.GL as gl
except ImportError:
    print("PyOpenGL not installed. Install with: pip install PyOpenGL")
    sys.exit(1)

# 机器人库（组件库面板）。缺失时降级为纯手工加载 URDF，不影响主程序启动
try:
    from orolp.robot_library.catalog import RobotCatalog
    from orolp.robot_library.panel import RobotLibraryPanel

    _HAS_ROBOT_LIBRARY = True
except Exception as _exc:  # pragma: no cover - 仅用于缺失依赖时的降级
    print(f"[warning] robot library unavailable: {_exc}")
    _HAS_ROBOT_LIBRARY = False
    RobotCatalog = None
    RobotLibraryPanel = None

# 组件库拖拽到视口的 MIME 标识（与 orolp.robot_library.panel.MIME_TYPE 保持一致）
MIME_TYPE = "application/x-machine-item"

# 环境建模（工作单元场景 + 参数化组件库 + 干涉检查）
try:
    from orolp.world.panel import ScenePanel
    from orolp.world.scene import Scene, ROBOT_NAME, GEOMETRY_COLLISION
    from orolp.world.geometry import prim_aabb
    from orolp.world.components import COMPONENTS, ParamSpec
    from orolp.world import placement as _placement
    from orolp.pose import Pose

    _HAS_WORLD = True
except Exception as _exc:  # pragma: no cover
    print(f"[warning] world modeling unavailable: {_exc}")
    _HAS_WORLD = False
    ScenePanel = None
    Scene = None
    COMPONENTS = None
    ParamSpec = None
    _placement = None

# 工作单元模型 + PLC（西门子 SCL）代码生成
try:
    from orolp.cell import HANDSHAKE_SIGNALS, CellConfig, Device, DEVICE_TYPES
    from orolp.plc_scl import generate_scl, steps_from_program
    from orolp.iomap import DIRECTION_LABELS, SignalMap

    _HAS_CELL = True
except Exception as _exc:  # pragma: no cover
    print(f"[warning] cell model unavailable: {_exc}")
    _HAS_CELL = False
    CellConfig = None
    Device = None
    DEVICE_TYPES = ()
    SignalMap = None
    DIRECTION_LABELS = {}
    generate_scl = None
    steps_from_program = None

# 抓取工艺（示教点语义 → IO 指令 + 吸附事件）
try:
    from orolp.pickplace import (
        DEFAULT_SEQUENCE,
        PointType,
        build_pick_place,
        default_types_for_count,
    )

    _HAS_PICKPLACE = True
except Exception as _exc:  # pragma: no cover
    print(f"[warning] pickplace unavailable: {_exc}")
    _HAS_PICKPLACE = False
    PointType = None
    build_pick_place = None
    default_types_for_count = None

# 工作单元模板 + 一键演示示教点（抓取工作站演示入口）
try:
    from orolp.world.demo import demo_taught_points
    from orolp.world.templates import build_grasping_workstation

    _HAS_TEMPLATES = True
except Exception as _exc:  # pragma: no cover
    print(f"[warning] workcell templates unavailable: {_exc}")
    _HAS_TEMPLATES = False
    build_grasping_workstation = None
    demo_taught_points = None

# 版本号：窗口标题**从库版本派生**，不再手写。历史上标题、pyproject、app.spec
# 与 orolp.__version__ 四处各写一份，改版本时漏掉一处就只能靠肉眼比对
# （v1.0.1 里程碑就漏过 pyproject）。标题这一份从此不可能漂移。
try:
    from orolp import __version__ as _OROLP_VERSION
except Exception:  # pragma: no cover - 仅用于包不可用的降级
    _OROLP_VERSION = "0.0.0-dev"

# 离线编程（轨迹展开 + 节拍估算 + 程序校验面板）
try:
    from orolp.program_panel import ProgramPanel

    _HAS_PROGRAM = True
except Exception as _exc:  # pragma: no cover
    print(f"[warning] program panel unavailable: {_exc}")
    _HAS_PROGRAM = False
    ProgramPanel = None

# 真机连接面板（v1.1.1）：SocketLink 的界面化——连接 / 部署 / 回读
try:
    from orolp.controller_panel import ControllerPanel

    _HAS_CONTROLLER_PANEL = True
except Exception as _exc:  # pragma: no cover
    print(f"[warning] controller panel unavailable: {_exc}")
    _HAS_CONTROLLER_PANEL = False
    ControllerPanel = None

# 渲染模型（URDF 解析 + STL/OBJ 网格）。支持多 visual、mesh、材质引用，
# 并按 URDF 规范使用 T·R 变换；缺失时回退到内置的最小解析器。
try:
    from orolp.render import MeshData, RobotModel, UrdfParseError

    _HAS_RENDER_MODEL = True
except Exception as _exc:  # pragma: no cover
    print(f"[warning] render model unavailable: {_exc}")
    _HAS_RENDER_MODEL = False
    RobotModel = None
    UrdfParseError = None
    MeshData = None

# 插件接口（v1.0.4）：渲染覆盖层 + 第三方后处理/运动学
try:
    from orolp.plugins import (
        FrameMarker,
        LineSegments,
        OverlayContext,
        PointCloud,
        collect_overlays,
        default_registry as plugin_registry,
        load_entry_points,
    )

    _HAS_PLUGINS = True
except Exception as _exc:  # pragma: no cover
    print(f"[warning] plugin layer unavailable: {_exc}")
    _HAS_PLUGINS = False
    OverlayContext = None
    collect_overlays = None
    plugin_registry = None
    load_entry_points = None
    LineSegments = PointCloud = FrameMarker = ()

# 缓存可观测性（v1.2.4）：三类结果缓存的统一快照（规划与分段/干涉检查/力矩校验）
try:
    from orolp.incremental import (
        cache_overview,
        clear_all_caches,
        reset_all_cache_stats,
    )

    _HAS_INCREMENTAL = True
except Exception as _exc:  # pragma: no cover
    print(f"[warning] cache overview unavailable: {_exc}")
    _HAS_INCREMENTAL = False
    cache_overview = None
    clear_all_caches = None
    reset_all_cache_stats = None

# 工程文件（v1.3.4）：把场景/示教程序/工艺时序/设备点表/零件来源收进单一 .orolp
try:
    from orolp.project import (
        PROJECT_EXTENSION,
        PROJECT_FILTER,
        Project,
        ProjectError,
        cell_from_snapshot,
        cell_snapshot,
    )

    _HAS_PROJECT = True
except Exception as _exc:  # pragma: no cover
    print(f"[warning] project file support unavailable: {_exc}")
    _HAS_PROJECT = False
    Project = None
    ProjectError = ValueError
    PROJECT_FILTER = ""
    PROJECT_EXTENSION = ".orolp"
    cell_snapshot = None
    cell_from_snapshot = None

# 工艺时序（v1.3.1）：工程文件载入时需要把段还原成 ProcessSequence
try:
    from orolp.sequence import ProcessSequence

    _HAS_SEQUENCE = True
except Exception as _exc:  # pragma: no cover
    print(f"[warning] process sequence unavailable: {_exc}")
    _HAS_SEQUENCE = False
    ProcessSequence = None

# 会话支撑（v1.3.5）：最近打开列表 + 崩溃自动恢复（依赖工程文件层）
if _HAS_PROJECT:
    try:
        from orolp.session import (
            RecentProjects,
            clear_autosave,
            has_autosave,
            read_autosave,
            write_autosave,
        )

        _HAS_SESSION_SUPPORT = True
    except Exception as _exc:  # pragma: no cover
        print(f"[warning] session support unavailable: {_exc}")
        _HAS_SESSION_SUPPORT = False
else:
    _HAS_SESSION_SUPPORT = False

# 工程模板（v1.3.6）：「从模板新建工程」——新活不从空白开始
if _HAS_PROJECT:
    try:
        from orolp.project_templates import (
            build_project_template,
            list_project_templates,
        )

        _HAS_PROJECT_TEMPLATES = True
    except Exception as _exc:  # pragma: no cover
        print(f"[warning] project templates unavailable: {_exc}")
        _HAS_PROJECT_TEMPLATES = False
else:
    _HAS_PROJECT_TEMPLATES = False


GL_COLOR_BUFFER_BIT = 0x00004000
GL_DEPTH_BUFFER_BIT = 0x00000100
GL_TRIANGLES = 0x00000004
GL_QUADS = 0x00000007
GL_QUAD_STRIP = 0x00000008
GL_TRIANGLE_FAN = 0x00000006
GL_LINES = 0x00000001
GL_FRONT_AND_BACK = 0x00000408
GL_LINE = 0x00001B01
GL_FILL = 0x00001B02
GL_LINE_STRIP = 0x00000003
GL_POINTS = 0x00000000
GL_DEPTH_TEST = 0x00000B71
GL_MODELVIEW = 0x00001700
GL_PROJECTION = 0x00001701

#: 首屏默认载入的示例机器人（按顺序回退）
DEFAULT_ROBOT_KEYS = ("kuka/kr16_2", "kuka/kr6_r900", "ur/ur5e")

#: 曲面细分参数：把圆周按目标弦长切段，再钳到 [16, 72] 段。
#: 视觉上 72 段的圆柱在 4K 屏上已看不出棱角；再往上只是浪费顶点。
#: 目标弦长 6 mm——视口单位是米，因此工业臂最粗的底座（半径约 0.13 m）用满 72 段，
#: 而法兰上的 M6 螺栓（半径约 3 mm）只用 16 段，不会为看不见的细节付费。
_TESSELLATION_CHORD_M = 0.006
_TESSELLATION_MIN_SEGMENTS = 16
_TESSELLATION_MAX_SEGMENTS = 72


def _segments_for_radius(radius: float) -> int:
    """按半径推算圆柱/球需要的周向分段数。

    Args:
        radius: 半径（米）。

    Returns:
        分段数，钳制在 ``[16, 72]`` 区间内。
    """
    if radius <= 0.0:
        return _TESSELLATION_MIN_SEGMENTS
    circumference = 2.0 * math.pi * radius
    segments = int(math.ceil(circumference / _TESSELLATION_CHORD_M))
    return max(_TESSELLATION_MIN_SEGMENTS, min(_TESSELLATION_MAX_SEGMENTS, segments))


#: 各轴"待机位姿"的目标角（度）。工业臂的经典 HOME 是「大臂后仰至水平、
#: 小臂竖起、法兰朝前」，形如举手；全零位姿会让整条臂竖直朝天，既不像真实
#: 机器人，也无法一眼看清各轴关系。超出该轴限位时钳到限位内（留 3% 余量）。
HOME_POSE_PATTERN_DEG = (0.0, -90.0, 90.0, 0.0, 90.0, 0.0, 0.0)

#: 待机位姿相对限位的收缩比例，避免正好停在限位上导致后续 moveJ 报越界
HOME_POSE_MARGIN = 0.03


class RobotRenderer:
    """URDF 渲染模型。

    底层解析交给 :class:`orolp.render.urdf_model.RobotModel`——它支持
    **每个连杆多个 visual**、**STL/OBJ 网格**、**材质引用**，并严格按 URDF 规范
    使用 ``T·R`` 变换。本类在其之上额外维护一份**旧版字典形态**的 ``links`` /
    ``joints``，供关节滑杆、信息面板等既有代码继续使用。

    Attributes:
        model: 完整的 :class:`RobotModel`；绘制路径只用它。
        links: 兼容旧接口的字典，每个连杆取**第一个** visual 的信息。
        joints: 兼容旧接口的关节字典。
        joint_values: 关节名 → 当前弧度值。
        mesh_errors: 网格加载失败清单（不静默降级，交由 UI 提示）。
    """

    def __init__(self):
        self.model = None
        self.links = {}
        self.joints = {}
        self.root_link = None
        self.joint_values = {}
        self.urdf_dir = None
        self.robot_name = "Unknown"
        self.mesh_errors = []

    # ------------------------------------------------------------------ #
    # 加载
    # ------------------------------------------------------------------ #
    def load_urdf(self, path):
        """加载 URDF，并把关节姿态初始化为待机位姿。

        Args:
            path: URDF 文件路径。

        Returns:
            是否至少含有一个连杆。

        Raises:
            UrdfParseError: URDF 结构非法（仅在渲染模型模块可用时抛出）。
        """
        self.links.clear()
        self.joints.clear()
        self.joint_values.clear()
        self.mesh_errors = []
        self.model = None
        self.urdf_dir = Path(path).parent

        if _HAS_RENDER_MODEL:
            model = RobotModel.from_urdf(path)
            self.model = model
            self.robot_name = model.name
            self.root_link = model.root
            self.mesh_errors = list(model.mesh_errors)
            self._fill_legacy_dicts(model)
        else:  # pragma: no cover - 渲染模型缺失时的最小回退
            self._load_legacy(path)
            return len(self.links) > 0

        self._apply_home_pose()

        print(
            f"Loaded {len(self.links)} links / {len(self.joints)} joints "
            f"from URDF (root={self.root_link}, "
            f"{model.visual_count} visuals, {model.triangle_count} triangles)"
        )
        if self.mesh_errors:
            for name, reason in self.mesh_errors:
                print(f"  [mesh] 加载失败: {name} — {reason}")
        return len(self.links) > 0

    def _fill_legacy_dicts(self, model: "RobotModel") -> None:
        """把 :class:`RobotModel` 投影成旧版的 ``links`` / ``joints`` 字典。

        只保留每个连杆的第一个 visual 作为代表几何——旧接口是单几何模型，
        这里仅用于滑杆排序与信息面板，绘制不依赖它。
        """
        for name, link in model.links.items():
            visual = link.visuals[0] if link.visuals else None
            if visual is None:
                self.links[name] = {
                    "name": name,
                    "type": None,
                    "params": (),
                    "origin_xyz": (0.0, 0.0, 0.0),
                    "origin_rpy": (0.0, 0.0, 0.0),
                    "color": (0.7, 0.7, 0.75),
                }
                continue

            prim = visual.primitive
            # 从 visual 的 4×4 矩阵里拆出平移与 rpy，保持旧字段语义
            translation = visual.matrix[:3, 3]
            self.links[name] = {
                "name": name,
                "type": prim.kind,
                "params": tuple(
                    p / 1000.0 if prim.kind == "cylinder" and i == 1 else p
                    for i, p in enumerate(prim.params)
                ) if prim.kind != "mesh" else (),
                "origin_xyz": tuple(float(v) for v in translation),
                "origin_rpy": _matrix_to_rpy(visual.matrix),
                "color": tuple(float(v) for v in visual.color),
            }

        for name, joint in model.joints.items():
            self.joints[name] = {
                "name": name,
                "type": joint.type,
                "parent": joint.parent,
                "child": joint.child,
                "origin_xyz": tuple(float(v) for v in joint.matrix[:3, 3]),
                "origin_rpy": _matrix_to_rpy(joint.matrix),
                "axis": tuple(float(v) for v in joint.axis),
                "limit": {
                    "lower": joint.limits.lower,
                    "upper": joint.limits.upper,
                    "velocity": joint.limits.velocity,
                    "effort": joint.limits.effort,
                },
            }
            self.joint_values[name] = 0.0

    def _apply_home_pose(self) -> None:
        """把可动关节设置到待机位姿（全零位姿竖直朝天，不像真实机器人）。"""
        if self.model is None:
            return
        for index, joint in enumerate(self.model.actuated_joints):
            lower = joint.limits.lower
            upper = joint.limits.upper
            pattern = (
                HOME_POSE_PATTERN_DEG[index]
                if index < len(HOME_POSE_PATTERN_DEG)
                else 0.0
            )
            target = math.radians(pattern)
            if upper > lower:
                margin = HOME_POSE_MARGIN * (upper - lower)
                target = min(max(target, lower + margin), upper - margin)
            self.joint_values[joint.name] = target

    # ------------------------------------------------------------------ #
    # 关节
    # ------------------------------------------------------------------ #
    def actuated_names(self) -> List[str]:
        """可动关节名（拓扑顺序）。"""
        if self.model is not None:
            return self.model.actuated_names
        return [n for n, j in self.joints.items() if j["type"] != "fixed"]

    def joint_vector(self) -> List[float]:
        """当前关节值向量，顺序与 :meth:`actuated_names` 一致。"""
        return [float(self.joint_values.get(n, 0.0)) for n in self.actuated_names()]

    def set_joints(self, q) -> None:
        """按顺序给可动关节赋值。

        Args:
            q: 关节值序列（弧度）；顺序与 :meth:`actuated_names` 一致。

        Raises:
            ValueError: 传入数量与自由度不符。
        """
        names = self.actuated_names()
        values = [float(v) for v in q]
        if len(values) != len(names):
            raise ValueError(
                tr("关节值数量 {n} 与自由度 {dof} 不符（关节: {names}）").format(n=len(values), dof=len(names), names=names)
            )
        for name, value in zip(names, values):
            self.joint_values[name] = value

    def link_poses(self) -> Dict[str, "np.ndarray"]:
        """返回每个连杆在根坐标系下的 4×4 位姿（正向运动学）。

        无渲染模型时返回空字典，调用方需自行回退。
        """
        if self.model is None:
            return {}
        return self.model.forward_kinematics(self.joint_vector())

    def _load_legacy(self, path) -> None:  # pragma: no cover - 降级路径
        """渲染模型模块不可用时的最小 URDF 解析（仅基本体、单 visual）。"""
        import xml.etree.ElementTree as ET

        root = ET.parse(path).getroot()
        self.robot_name = root.get("name", "Unknown")
        for link in root.findall("link"):
            visual = link.find("visual")
            if visual is None:
                continue
            geom = visual.find("geometry")
            material = visual.find("material")
            mat = material.find("color") if material is not None else None
            rgba = [0.7, 0.7, 0.75, 1.0]
            if mat is not None:
                rgba = [float(x) for x in mat.get("rgba", "0.7 0.7 0.75 1").split()]
            origin = visual.find("origin")
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
            if origin is not None:
                xyz = [float(x) for x in origin.get("xyz", "0 0 0").split()]
                rpy = [float(x) for x in origin.get("rpy", "0 0 0").split()]
            geo_type, params = None, ()
            if geom.find("cylinder") is not None:
                cyl = geom.find("cylinder")
                geo_type = "cylinder"
                params = (float(cyl.get("radius", "0.03")), float(cyl.get("length", "0.15")))
            elif geom.find("box") is not None:
                box = geom.find("box")
                geo_type = "box"
                params = tuple(float(x) for x in box.get("size", "0.1 0.1 0.1").split())
            elif geom.find("sphere") is not None:
                geo_type = "sphere"
                params = (float(geom.find("sphere").get("radius", "0.02")),)
            self.links[link.get("name")] = {
                "name": link.get("name"),
                "type": geo_type,
                "params": params,
                "origin_xyz": xyz,
                "origin_rpy": rpy,
                "color": rgba[:3],
            }

        for joint in root.findall("joint"):
            jname = joint.get("name")
            origin = joint.find("origin")
            jxyz = [0.0, 0.0, 0.0]
            jrpy = [0.0, 0.0, 0.0]
            if origin is not None:
                jxyz = [float(x) for x in origin.get("xyz", "0 0 0").split()]
                jrpy = [float(x) for x in origin.get("rpy", "0 0 0").split()]
            axis = [0.0, 0.0, 1.0]
            axis_el = joint.find("axis")
            if axis_el is not None:
                axis = [float(x) for x in axis_el.get("xyz", "0 0 1").split()]
            lower, upper = -math.pi, math.pi
            limit = joint.find("limit")
            if limit is not None:
                lower = float(limit.get("lower", "-3.14159"))
                upper = float(limit.get("upper", "3.14159"))
            self.joints[jname] = {
                "name": jname,
                "type": joint.get("type", "fixed"),
                "parent": joint.find("parent").get("link"),
                "child": joint.find("child").get("link"),
                "origin_xyz": jxyz,
                "origin_rpy": jrpy,
                "axis": axis,
                "limit": {"lower": lower, "upper": upper},
            }
            self.joint_values[jname] = 0.0

        child_links = {j["child"] for j in self.joints.values()}
        for name in self.links:
            if name not in child_links:
                self.root_link = name
                break


def _matrix_to_rpy(matrix: "np.ndarray") -> Tuple[float, float, float]:
    """从 4×4 齐次矩阵里还原 URDF 语义的 ``(roll, pitch, yaw)``。

    与 :func:`orolp.render.urdf_model.rpy_to_matrix` 互为逆运算，
    即 ``R = Rz(yaw)·Ry(pitch)·Rx(roll)``。

    Args:
        matrix: 4×4 齐次矩阵（只用其左上 3×3）。

    Returns:
        ``(roll, pitch, yaw)``，弧度。
    """
    r = matrix[:3, :3]
    # R[2,0] = -sin(pitch)  ⇒ pitch = asin(-R[2,0])
    pitch = math.asin(max(-1.0, min(1.0, -float(r[2, 0]))))
    cos_pitch = math.cos(pitch)
    if abs(cos_pitch) < 1e-9:
        # 万向锁：pitch = ±90°，此时 roll 与 yaw 不可分离，约定 yaw=0
        roll = math.atan2(-float(r[0, 1]), float(r[1, 1]))
        yaw = 0.0
    else:
        roll = math.atan2(float(r[2, 1]), float(r[2, 2]))
        yaw = math.atan2(float(r[1, 0]), float(r[0, 0]))
    return (roll, pitch, yaw)


class GLWidget(QtOpenGLWidgets.QOpenGLWidget):
    objectSelected = QtCore.Signal(str)  # 右键拾取/拖动时发射对象名（未命中不发射）
    componentDropped = QtCore.Signal(str)  # 组件库拖入视口的 component_id
    robotDropped = QtCore.Signal(str)  # 机器库拖入视口的 catalog key
    #: 左键拖拽摆放完成（松手落到支承面），携带状态栏一行小结（已翻译）
    objectPlaced = QtCore.Signal(str)
    #: 视口侧改动了挂载关系（起拖自动解除 / 松手吸附装夹），携带对象名
    sceneChanged = QtCore.Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.renderer = RobotRenderer()
        #: 多机器人：每台一个渲染器，键为场景里的机器人名。
        #: ``self.renderer`` 仍是"当前那台"（关节滑杆、记录点位等既有单机器人
        #: 调用面都走它），绘制时按名字从本字典取，互不覆盖。
        self.renderers: dict = {}
        #: 当前机器人：关节滑杆与关节同步都作用到它（多机必需）
        self.current_robot: str = ROBOT_NAME
        self.camera_dist = 1.2
        self.camera_az = 45.0
        self.camera_el = 30.0
        self.dragging = False
        self._left_moved = False
        self._left_press_pos = None
        # 工作单元场景（由 MainWindow 注入）；场景内部使用毫米，
        # 而本视口沿用 URDF 的米，绘制时统一乘 MM_TO_M
        self.scene = None
        self.last_pos = None
        self.recorded_positions = []
        self.recorded_ee_positions = []
        self.show_path = True
        #: 相机注视点（米）。载入机器人后按模型尺度自动对准，否则小协作臂
        #: 只占画面一角、大臂又超出视野。
        self.view_center = (0.0, 0.0, 0.4)

        #: 右键拾取 / 坐标系 gizmo 变换状态
        self.selected_object = None
        self.drag_mode = None          # "xy" / "z" / "rz" / None
        self._drag_anchor = None       # 拖动起始参考量
        self._right_like = False       # Control+左键 视为右键（Mac 触控板兼容）

        #: 左键拖拽摆放（v1.1.7）：按在工件上拖 = 移动，松手自动落到支承面；
        #: 按在空白处拖 = 旋转视角（与既有行为一致）。
        #: ``snap_step_mm > 0`` 时落点 XY 按步长吸附（毫米）。
        self.snap_step_mm = 0.0
        #: 松手时落在机器人法兰此半径内（毫米）即自动装夹到法兰
        self.snap_radius_mm = 60.0
        self._grab_active = False
        self._grab_anchor = None

        #: 插件渲染覆盖层（v1.0.4）：第三方画的东西与它们出的错
        self.show_plugin_overlays = True
        self.plugin_overlay_context = None
        self.plugin_overlays = None
        self.plugin_overlay_errors = []

        # 选中机器人即切换"当前机器人"：关节滑杆与关节同步随之改指向。
        # 挂在内部信号上，左/右键拾取、拖拽起手等所有入口一次覆盖。
        self.objectSelected.connect(self._on_selected_changed)

        self.setAcceptDrops(True)

    def _on_selected_changed(self, name) -> None:
        """选中的是机器人时，把它设为"当前机器人"（多机下的关节归属）。"""
        if name and self.scene is not None and self.scene.is_robot(name):
            self.set_current_robot(name)

    def load_robot(self, path, name: str = ROBOT_NAME):
        """装载渲染模型；``name`` 指定登记在哪台机器人名下（多机）。"""
        renderer = self.renderers.get(name)
        if renderer is None:
            renderer = RobotRenderer()
            self.renderers[name] = renderer
        success = renderer.load_urdf(path)
        if success:
            # 当前机器人切换为新装载这台：关节滑杆、记录点位、关节同步都跟它走
            self.current_robot = name
            self.renderer = renderer
            self.frame_robot(name)
        self.update()
        return success

    def renderer_of(self, name: str = ROBOT_NAME):
        """取某台机器人的渲染器。

        未登记时**按该机器人在场景里的 URDF 懒加载一个**，并把它的关节对齐到
        该机器人的**场景关节**。两条都不能省：

        * 直接回退到"当前渲染器"会让它借用**别的机器人**的模型——于是拖关节
          时两台一起动（共用同一份 joint_values）；
        * 新建的渲染器默认是 HOME 待机位，而场景里那台有自己的关节值——不对齐
          就会"画出来的手臂"与"场景法兰"错位，挂在上面的工具离开显示的法兰。
        """
        renderer = self.renderers.get(name)
        if renderer is not None:
            return renderer
        urdf = None
        if self.scene is not None and name in self.scene.robots:
            urdf = self.scene.robots[name].urdf
        if urdf and Path(urdf).exists():
            renderer = RobotRenderer()
            if renderer.load_urdf(urdf):
                self.renderers[name] = renderer
                state = self.scene.robots.get(name)
                if state is not None:
                    for i, joint_name in enumerate(renderer.actuated_names()):
                        if i < len(state.joints_deg):
                            renderer.joint_values[joint_name] = math.radians(
                                float(state.joints_deg[i])
                            )
                return renderer
        return self.renderer

    def set_current_robot(self, name) -> None:
        """切换"当前机器人"：关节滑杆与关节同步都作用到它（多机必需）。"""
        if not name or self.scene is None or name not in self.scene.robots:
            return
        self.current_robot = name
        self.renderer = self.renderer_of(name)

    # ---------------- 拖拽：从机器组件库接收对象 ---------------- #
    def dragEnterEvent(self, event):
        if event.mimeData().hasFormat(MIME_TYPE):
            event.acceptProposedAction()
        else:
            event.ignore()

    def dropEvent(self, event):
        md = event.mimeData()
        if not md.hasFormat(MIME_TYPE):
            event.ignore()
            return
        try:
            data = json.loads(bytes(md.data(MIME_TYPE)).decode("utf-8"))
        except Exception:
            event.ignore()
            return
        kind = data.get("kind")
        if kind == "robot":
            self.robotDropped.emit(data["id"])
        elif kind == "component":
            self.componentDropped.emit(data["id"])
        event.acceptProposedAction()

    def frame_robot(self, robot: str = ROBOT_NAME) -> None:
        """按当前位姿的包围球自动取景（相机距离 + 注视点）。

        机器人尺寸跨度很大：UR3e 臂展 500 mm，IRB 6700 臂展 3200 mm。
        固定相机距离会让小机器人只占画面一角、大机器人直接出画。

        包围球按**每个 visual 的实际几何**算，而不是只取连杆原点——后者会
        低估尺寸（连杆原点只是几个点，几何体本身还有体积），导致大臂贴边出画。
        """
        bounds = self._robot_bounds(robot)
        if bounds is None:
            return
        lo, hi = bounds

        center = np.array(
            [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0],
            dtype=np.float64,
        )
        radius = 0.5 * float(np.linalg.norm(np.asarray(hi) - np.asarray(lo)))
        if radius <= 1e-6:
            radius = 0.1

        # 包围球半径 r 要在 45° 垂直 FOV（半角 22.5°）内完整入画，需
        # dist = r / sin(22.5°)；再留 10% 余量避免贴边。用包围球而非包围盒，
        # 是因为相机会绕模型公转，包围盒在不同方位下投影差异很大。
        self.camera_dist = max(0.3, radius / math.sin(math.radians(22.5)) * 1.10)
        self.view_center = (float(center[0]), float(center[1]), float(center[2]))

    def _robot_base_pose_m(self, robot: str = ROBOT_NAME):
        """机器人在场景中的**世界**基座位姿（视口米制）；无机器人时 ``None``。

        必须用 :meth:`Scene.world_transform` 而不是 ``SceneObject.pose``：机器人
        挂到底座/台面上时世界位姿 = 底座 × 局部位姿，只取局部会让渲染与场景
        数据分家——表现为"画的机器人不跟着底座走、装夹件与手臂对不上"。

        坐标系语义：机器人始终使用**自身坐标系**，基座只是一个把它整体搬到
        某处的父级变换（后续若要支持自定义基座坐标系，改这里即可）。
        """
        if self.scene is None or robot not in self.scene.objects:
            return None
        T = np.asarray(self.scene.world_transform(robot), dtype=np.float64).copy()
        T[:3, 3] *= self.MM_TO_M
        return T

    def _robot_bounds(self, robot: str = ROBOT_NAME):
        """返回**指定机器人**当前位姿下所有 visual 的世界包围盒 ``(lo, hi)``。

        Returns:
            ``(lo, hi)`` 两个长度 3 的数组；无模型时返回 ``None``。
        """
        model = self.renderer_of(robot).model
        poses = self.renderer_of(robot).link_poses()
        if model is None or not poses:
            return None

        lo = np.array([np.inf, np.inf, np.inf], dtype=np.float64)
        hi = np.array([-np.inf, -np.inf, -np.inf], dtype=np.float64)
        found = False

        for link_name in model.link_chain():
            link = model.links.get(link_name)
            pose = poses.get(link_name)
            if link is None or pose is None:
                continue
            for visual in link.visuals:
                origin = (pose @ visual.matrix)[:3, 3]
                radius = self._primitive_radius(visual.primitive)
                lo = np.minimum(lo, origin - radius)
                hi = np.maximum(hi, origin + radius)
                found = True

        if not found:
            return None
        # 基座位姿也要吃进去：否则"重置视图"会把镜头对准原点，而不是对准
        # 挂在底座/台面上的那台机器人
        base = self._robot_base_pose_m(robot)
        if base is not None and not np.allclose(base, np.eye(4)):
            corners = np.array(
                [[x, y, z, 1.0]
                 for x in (lo[0], hi[0])
                 for y in (lo[1], hi[1])
                 for z in (lo[2], hi[2])],
                dtype=np.float64,
            )
            world = (corners @ base.T)[:, :3]
            lo, hi = world.min(axis=0), world.max(axis=0)
        return lo, hi

    @staticmethod
    def _primitive_radius(primitive: object) -> float:
        """求基本体的包围球半径（米），用于快速估算取景范围。"""
        import math as _math

        kind = primitive.kind
        if kind == "box":
            sx, sy, sz = (float(p) for p in primitive.params[:3])
            return 0.5 * _math.sqrt(sx * sx + sy * sy + sz * sz)
        if kind == "cylinder":
            radius, length = (float(p) for p in primitive.params[:2])
            return _math.sqrt(radius * radius + (length / 2.0) ** 2)
        if kind == "sphere":
            return float(primitive.params[0])
        if kind == "mesh":
            mesh = primitive.mesh
            if mesh is not None and not mesh.is_empty:
                extent = mesh.vertices.max(axis=0) - mesh.vertices.min(axis=0)
                return 0.5 * float(np.linalg.norm(extent))
        return 0.0

    def set_joints(self, q):
        self.renderer.set_joints(q)
        self.update()

    def get_joint_values(self):
        return dict(self.renderer.joint_values)

    def set_joint_values(self, joint_dict):
        for name, value in joint_dict.items():
            self.renderer.joint_values[name] = value
        self.update()

    def record_position(self):
        pos = self.get_joint_values()
        if pos:
            self.recorded_positions.append(pos)
            ee_pos = self._compute_end_effector_position()
            self.recorded_ee_positions.append(ee_pos)
        return len(self.recorded_positions)

    def clear_recorded_positions(self):
        self.recorded_positions.clear()
        self.recorded_ee_positions.clear()

    def get_recorded_positions(self):
        return self.recorded_positions

    def set_recorded_positions(self, positions):
        self.recorded_positions = positions
        # 重新计算末端执行器位置
        original_joint_values = dict(self.renderer.joint_values)
        self.recorded_ee_positions.clear()
        for pos in positions:
            self.set_joint_values(pos)
            ee_pos = self._compute_end_effector_position()
            self.recorded_ee_positions.append(ee_pos)
        self.set_joint_values(original_joint_values)

    def _compute_end_effector_position(self):
        """计算末端执行器（TCP）的三维位置，单位米。

        走 :class:`RobotModel` 的正向运动学——它与 Pinocchio 的对拍误差为 0，
        且严格遵循 URDF 的 ``T·R`` 变换语义。旧实现是手写的 R·T 版本，
        一旦关节带 rpy 就会算错，这里直接换成已验证的实现。
        """
        if self.renderer.root_link is None:
            return (0.0, 0.0, 0.0)

        poses = self.renderer.link_poses()
        if not poses:
            return (0.0, 0.0, 0.0)

        # 末端 = 运动链上最后一个连杆；优先使用 TCP 标记
        model = self.renderer.model
        if model is not None:
            chain = model.link_chain()
            target = "tcp" if "tcp" in poses else (chain[-1] if chain else None)
        else:
            target = "tcp" if "tcp" in poses else None

        if target is None:
            target = next(reversed(list(poses)))
        pose = poses.get(target)
        if pose is None:
            return (0.0, 0.0, 0.0)
        return (float(pose[0, 3]), float(pose[1, 3]), float(pose[2, 3]))

    def initializeGL(self):
        gl.glClearColor(0.15, 0.15, 0.18, 1.0)
        gl.glEnable(gl.GL_DEPTH_TEST)
        self._mesh_lists = {}

        # ---- 光照 ---- #
        # 这是"显示失真"的首要成因：没有 GL_LIGHTING 时所有 glNormal3f 全部失效，
        # 箱体的六个面颜色完全一致，三维模型被渲染成二维纯色剪影。
        gl.glEnable(gl.GL_LIGHTING)
        gl.glEnable(gl.GL_LIGHT0)  # 主光：暖白，来自右上前
        gl.glEnable(gl.GL_LIGHT1)  # 补光：冷色，来自左后，避免背光面死黑
        gl.glEnable(gl.GL_LIGHT2)  # 反弹光：自下往上，压掉底部阴影
        gl.glEnable(gl.GL_NORMALIZE)  # 非均匀缩放会破坏法线单位长度
        gl.glShadeModel(gl.GL_SMOOTH)
        # 双面光照：URDF 常见开放曲面与内翻面法线，单面光照会出现黑洞
        gl.glLightModeli(gl.GL_LIGHT_MODEL_TWO_SIDE, 1)

        # 让 glColor3f 直接驱动环境/漫反射，这样既有的 glColor3f 调用无需改动
        gl.glEnable(gl.GL_COLOR_MATERIAL)
        gl.glColorMaterial(gl.GL_FRONT_AND_BACK, gl.GL_AMBIENT_AND_DIFFUSE)
        # 高光用统一材质参数：金属漆感（弱高光 + 中等锐度）
        gl.glMaterialfv(
            gl.GL_FRONT_AND_BACK,
            gl.GL_SPECULAR,
            np.array([0.32, 0.34, 0.38, 1.0], dtype=np.float32),
        )
        gl.glMaterialf(gl.GL_FRONT_AND_BACK, gl.GL_SHININESS, 38.0)

    def _setup_lights(self):
        """在相机矩阵就位后设置光源（方向光，世界坐标系）。

        必须在 ``paintGL`` 里、模型视图矩阵已切换为相机视角之后调用——光源位置会被
        调用时刻的模型视图矩阵变换，放错地方光源就会跟着相机转，明暗不再随模型变化。
        """
        gl.glLightfv(
            gl.GL_LIGHT0, gl.GL_POSITION,
            np.array([0.55, -0.75, 1.05, 0.0], dtype=np.float32),
        )
        gl.glLightfv(
            gl.GL_LIGHT0, gl.GL_DIFFUSE,
            np.array([0.95, 0.94, 0.90, 1.0], dtype=np.float32),
        )
        gl.glLightfv(
            gl.GL_LIGHT1, gl.GL_POSITION,
            np.array([-0.85, -0.35, 0.45, 0.0], dtype=np.float32),
        )
        gl.glLightfv(
            gl.GL_LIGHT1, gl.GL_DIFFUSE,
            np.array([0.30, 0.36, 0.48, 1.0], dtype=np.float32),
        )
        gl.glLightfv(
            gl.GL_LIGHT2, gl.GL_POSITION,
            np.array([0.10, 0.60, -0.70, 0.0], dtype=np.float32),
        )
        gl.glLightfv(
            gl.GL_LIGHT2, gl.GL_DIFFUSE,
            np.array([0.18, 0.18, 0.20, 1.0], dtype=np.float32),
        )
        # 全局环境光：保证最暗处仍有轮廓可辨
        gl.glLightModelfv(
            gl.GL_LIGHT_MODEL_AMBIENT,
            np.array([0.24, 0.25, 0.28, 1.0], dtype=np.float32),
        )

    def paintGL(self):
        gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        w, h = self.width(), self.height()

        gl.glMatrixMode(GL_PROJECTION)
        gl.glLoadIdentity()
        aspect = w / max(1, h)
        fov = 45.0
        near, far = 0.01, 100.0
        top = near * math.tan(math.radians(fov / 2))
        right = top * aspect
        gl.glFrustum(-right, right, -top, top, near, far)

        gl.glMatrixMode(GL_MODELVIEW)
        gl.glLoadIdentity()

        az = math.radians(self.camera_az)
        el = math.radians(self.camera_el)
        # 相机绕注视点公转，而不是绕世界原点——否则 frame_robot() 把
        # view_center 挪到模型中心后，相机仍按原点摆位，机器人直接出画。
        center = np.asarray(self.view_center, dtype=np.float64)
        eye = center + np.array(
            [
                self.camera_dist * math.cos(el) * math.cos(az),
                self.camera_dist * math.cos(el) * math.sin(az),
                self.camera_dist * math.sin(el),
            ],
            dtype=np.float64,
        )
        up_vec = np.array([0.0, 0.0, 1.0], dtype=np.float64)

        f = center - eye
        norm = np.linalg.norm(f)
        if norm > 0:
            f = f / norm
        s = np.cross(f, up_vec)
        norm = np.linalg.norm(s)
        if norm > 0:
            s = s / norm
        u = np.cross(s, f)

        # glLoadMatrixf 按**列主序**读取这 16 个数：flat[col*4 + row]。
        # 若按行写成 [s; u; -f]，GL 拿到的是转置矩阵 Rᵀ = R⁻¹（R 正交），
        # 相机等于装了反向旋转，视线偏到画面外——这是"显示失真"的根因之一。
        rot = [
            float(s[0]), float(u[0]), float(-f[0]), 0.0,
            float(s[1]), float(u[1]), float(-f[1]), 0.0,
            float(s[2]), float(u[2]), float(-f[2]), 0.0,
            0.0, 0.0, 0.0, 1.0,
        ]
        gl.glLoadMatrixf(rot)
        gl.glTranslated(float(-eye[0]), float(-eye[1]), float(-eye[2]))

        # 光源必须在相机矩阵就位后再设，否则会跟着相机一起转
        self._setup_lights()

        # 网格与路径是"示意线"，不该参与光照（否则会被明暗调制成灰暗的虚线）
        gl.glDisable(gl.GL_LIGHTING)
        self._draw_grid()
        gl.glEnable(gl.GL_LIGHTING)

        if self.scene is not None:
            self._draw_scene()
        self._draw_robot()

        if self.show_path and len(self.recorded_positions) > 1:
            gl.glDisable(gl.GL_LIGHTING)
            self._draw_recorded_path()
            gl.glEnable(gl.GL_LIGHTING)

        # 选中对象描边高亮 + 坐标系 gizmo（覆盖在最前）
        if self.selected_object is not None and self.scene is not None:
            self._highlight_object(self.selected_object, (1.0, 0.85, 0.0))
        self._draw_gizmo()

        # 插件覆盖层（v1.0.4）：第三方画的东西放在最上层，且不参与光照——
        # 覆盖层的用途是"标注"，被明暗调制反而看不清。
        if self.show_plugin_overlays:
            gl.glDisable(gl.GL_LIGHTING)
            self._draw_overlays()
            gl.glEnable(gl.GL_LIGHTING)

    # ------------------------------------------------------------------ #
    # 工作单元场景绘制
    # ------------------------------------------------------------------ #

    #: 场景毫米 → 视口米
    MM_TO_M = 0.001

    def _draw_scene(self) -> None:
        """绘制场景中的所有可见对象。

        场景用毫米、视口用米，因此这里对平移做整体缩放。旋转部分不受
        各向同性缩放影响，可直接透传。
        """
        if not _HAS_WORLD or self.scene is None:
            return
        try:
            for name in list(self.scene.objects):
                if self.scene.is_robot(name):
                    continue  # 机器人统一由 _draw_robot 负责，避免重复绘制
                obj = self.scene.objects.get(name)
                if obj is None or not obj.visible:
                    continue
                for part in self.scene.object_parts(name):
                    self._draw_part(part)

            # 干涉对用红色线框高亮，让问题一眼可见（v0.2.1 起支持连杆级：
            # 命中带 link_a 时把"撞在哪一段"的连杆也描红）
            for hit in self._last_collisions:
                if hit.obj_b != ROBOT_NAME:
                    self._highlight_object(hit.obj_b, (1.0, 0.25, 0.15))
                if getattr(hit, "link_a", None):
                    self._highlight_link(hit.link_a, (1.0, 0.25, 0.15))
        except Exception as _exc:  # 渲染异常不应拖垮整个视口
            print(f"[warning] scene draw failed: {_exc}")

    _last_collisions: list = []

    def set_collision_results(self, hits) -> None:
        """记录最近一次干涉检查结果，供绘制时高亮。

        Args:
            hits: :class:`~orolp.world.scene.Collision` 列表
        """
        self._last_collisions = list(hits)
        self.update()

    def _highlight_object(self, name: str, color=None) -> None:
        """用线框高亮指定对象。

        选中对象默认亮黄；干涉命中由调用处传入红色。关闭光照与深度测试，
        使描边始终浮于最前、且颜色不被零件原色覆盖。
        """
        obj = self.scene.objects.get(name)
        if obj is None:
            return
        color = color or (1.0, 0.85, 0.0)
        gl.glPushMatrix()
        gl.glDisable(gl.GL_LIGHTING)
        gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glColor3f(*color)
        gl.glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        gl.glLineWidth(2.5)
        # 机器人走**渲染几何**描边：_picking_parts 给的是碰撞包络（为了点得中），
        # 拿它描边会让选中瞬间轮廓变成简化体，看着像换了一台机器人
        if self.scene.is_robot(name):
            self._draw_robot_wireframe(name)
        else:
            for part in self.scene.object_parts(name):
                self._draw_part(part, force_color=color)
        gl.glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        gl.glLineWidth(1.0)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_LIGHTING)
        gl.glPopMatrix()

    def _draw_robot_wireframe(self, name):
        """把某台机器人**按当前渲染几何**再画一遍（线框，沿用当前颜色）。

        与 :meth:`_draw_one_robot` 同一套几何与基座位姿，只是**不设材质色**——
        描边必须与实体完全重合，否则会出现"描边飘在机身外/形状对不上"。
        """
        renderer = self.renderer_of(name)
        model = renderer.model
        base_tf = self._robot_base_pose_m(name)
        if base_tf is not None:
            gl.glPushMatrix()
            gl.glMultMatrixd(np.ascontiguousarray(base_tf.T))
        try:
            if model is None or model.root is None:
                if renderer.root_link is not None:
                    self._draw_link_recursive(renderer.root_link, renderer=renderer)
                return
            poses = renderer.link_poses()
            for link_name in model.link_chain():
                link = model.links.get(link_name)
                base_pose = poses.get(link_name)
                if link is None or base_pose is None:
                    continue
                for visual in link.visuals:
                    gl.glPushMatrix()
                    combined = np.asarray(base_pose @ visual.matrix, dtype=np.float64)
                    gl.glMultMatrixd(np.ascontiguousarray(combined.T))
                    self._draw_primitive(visual.primitive)
                    gl.glPopMatrix()
        finally:
            if base_tf is not None:
                gl.glPopMatrix()

    def _highlight_link(self, link_name: str, color=None) -> None:
        """用线框高亮机器人的一个连杆（碰撞定位："撞在哪一段"）。

        连杆几何取自 :meth:`scene.robot_link_parts`（世界系、按当前关节
        角摆放——场景同步会把关节摆到回放/检查位形）。连杆不存在时静默
        返回（如该 URDF 无此 link 的视觉体）。
        """
        if not (_HAS_WORLD and self.scene is not None and self.scene.has_robot):
            return
        try:
            parts = self.scene.robot_link_parts().get(link_name)
        except Exception:
            return
        if not parts:
            return
        color = color or (1.0, 0.25, 0.15)
        gl.glPushMatrix()
        gl.glDisable(gl.GL_LIGHTING)
        gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glColor3f(*color)
        gl.glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        gl.glLineWidth(2.5)
        for part in parts:
            self._draw_part(part, force_color=color)
        gl.glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        gl.glLineWidth(1.0)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_LIGHTING)
        gl.glPopMatrix()

    def _draw_part(self, part, tf_parent=None, force_color=None) -> None:
        """绘制单个零件（基本体 + 局部变换）。

        Args:
            part: :class:`~orolp.world.components.Part`
            tf_parent: 父级变换（已含世界变换）；``None`` 表示 part 自带世界变换
            force_color: 指定颜色（用于高亮描边），覆盖 part.color
        """
        m = np.array(part.transform, dtype=np.float64)
        if tf_parent is not None:
            m = tf_parent @ m
        m = m.copy()
        m[:3, 3] *= self.MM_TO_M

        gl.glPushMatrix()
        # OpenGL 采用列主序，numpy 为行主序，需转置后再展平
        gl.glMultMatrixd(m.T.flatten())
        if force_color is not None:
            gl.glColor3f(*force_color)
        else:
            gl.glColor3f(*part.color)

        prim = part.prim
        kind = getattr(prim, "kind", "")
        if kind == "box":
            sx, sy, sz = prim.size
            self._draw_box(sx * self.MM_TO_M, sy * self.MM_TO_M, sz * self.MM_TO_M)
        elif kind == "cylinder":
            self._draw_cylinder(prim.radius * self.MM_TO_M, prim.length * self.MM_TO_M)
        elif kind == "sphere":
            self._draw_sphere(prim.radius * self.MM_TO_M)
        elif kind == "mesh":
            mesh = getattr(prim, "mesh", None)
            if mesh is not None and not mesh.is_empty:
                self._draw_mesh(mesh)
        gl.glPopMatrix()

    def _draw_grid(self):
        gl.glBegin(GL_LINES)
        gl.glColor3f(0.3, 0.3, 0.35)
        size = 1.0
        steps = 20
        for i in range(steps + 1):
            x = -size + 2 * size * i / steps
            gl.glVertex3f(x, -size, 0)
            gl.glVertex3f(x, size, 0)
            gl.glVertex3f(-size, x, 0)
            gl.glVertex3f(size, x, 0)
        gl.glEnd()

    def _draw_robot(self):
        """绘制机器人。

        不再递归压栈拼变换，而是先用 :meth:`RobotRenderer.link_poses` 一次性算出
        每个连杆的世界位姿，再逐个 visual 用 ``model · visual.matrix`` 直接定位。
        这样变换只算一遍，且严格遵循 URDF 的 ``T·R`` 语义（旧实现是 ``R·T``，
        带 rpy 的连杆会绕错误的点旋转）。

        若机器人在场景中以真实对象存在，则按基座位姿（SceneObject.pose）整体平移/旋转。
        """
        if self.scene is not None and not self.scene.has_robot:
            return  # 场景已移除机器人，视口不再绘制
        # 多机器人：按场景里的机器人名逐个绘制，各自取渲染器与基座位姿
        names = (
            self.scene.robot_names
            if self.scene is not None and self.scene.robot_names
            else [ROBOT_NAME]
        )
        for name in names:
            self._draw_one_robot(name)

    def _draw_one_robot(self, name):
        """绘制单台机器人：按名字取渲染器，按**世界**基座位姿整体平移/旋转。"""
        renderer = self.renderer_of(name)
        model = renderer.model

        # 机器人基座位姿 → 视口米制（必须是世界位姿：见 _robot_base_pose_m）
        base_tf = self._robot_base_pose_m(name)

        if base_tf is not None:
            gl.glPushMatrix()
            gl.glMultMatrixd(np.ascontiguousarray(base_tf.T))

        try:
            if model is None or model.root is None:
                # 渲染模型模块不可用：回退到旧的单几何递归绘制
                if renderer.root_link is not None:
                    self._draw_link_recursive(renderer.root_link, renderer=renderer)
                return

            poses = renderer.link_poses()
            for link_name in model.link_chain():
                link = model.links.get(link_name)
                base_pose = poses.get(link_name)
                if link is None or base_pose is None:
                    continue
                for visual in link.visuals:
                    self._draw_visual(base_pose, visual)
        finally:
            if base_tf is not None:
                gl.glPopMatrix()

    def _draw_visual(self, base_pose: "np.ndarray", visual: object) -> None:
        """在世界位姿 ``base_pose`` 下绘制一个 visual。"""
        if visual.alpha < 0.999:
            gl.glEnable(gl.GL_BLEND)
            gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
            gl.glDepthMask(gl.GL_FALSE)

        gl.glPushMatrix()
        # OpenGL 要求列主序；numpy 是行主序，故转置后再展平
        combined = np.asarray(base_pose @ visual.matrix, dtype=np.float64)
        gl.glMultMatrixd(np.ascontiguousarray(combined.T))
        gl.glColor3f(*visual.color)
        self._draw_primitive(visual.primitive)
        gl.glPopMatrix()

        if visual.alpha < 0.999:
            gl.glDepthMask(gl.GL_TRUE)
            gl.glDisable(gl.GL_BLEND)

    def _draw_primitive(self, primitive: object) -> None:
        """按几何体类型分派绘制。"""
        kind = primitive.kind
        if kind == "box":
            self._draw_box(*primitive.params)
        elif kind == "cylinder":
            self._draw_cylinder(primitive.params[0], primitive.params[1])
        elif kind == "sphere":
            self._draw_sphere(primitive.params[0])
        elif kind == "mesh":
            mesh = primitive.mesh
            if mesh is not None and not mesh.is_empty:
                self._draw_mesh(mesh)

    # ------------------------------------------------------------------ #
    # 网格（STL / OBJ）
    # ------------------------------------------------------------------ #
    def _draw_mesh(self, mesh: "MeshData") -> None:
        """绘制网格，并把它编译进显示列表缓存。

        一个机器人可能带十几兆、近 20 万个三角形的网格。逐帧走顶点数组尚可接受，
        但编译成显示列表后 GPU 端几乎零开销——首次绘制后同一网格只是 ``glCallList``。

        Args:
            mesh: 已加载并算好法线的网格数据。
        """
        if not hasattr(self, "_mesh_lists"):
            self._mesh_lists = {}
        cached = self._mesh_lists.get(mesh.source)
        if cached is None:
            cached = gl.glGenLists(1)
            if cached == 0:  # pragma: no cover - 极老的驱动可能分配失败
                self._emit_mesh(mesh)
                return
            gl.glNewList(cached, gl.GL_COMPILE)
            self._emit_mesh(mesh)
            gl.glEndList()
            self._mesh_lists[mesh.source] = cached
        gl.glCallList(cached)

    def _emit_mesh(self, mesh: "MeshData") -> None:
        """用顶点数组提交一次三角形绘制。"""
        vertices = np.ascontiguousarray(mesh.vertices, dtype=np.float32)
        normals = np.ascontiguousarray(mesh.normals, dtype=np.float32)
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glEnableClientState(gl.GL_NORMAL_ARRAY)
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, vertices)
        gl.glNormalPointer(gl.GL_FLOAT, 0, normals)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, vertices.shape[0])
        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        gl.glDisableClientState(gl.GL_NORMAL_ARRAY)

    def _draw_link_recursive(self, link_name, parent_joint=None):
        """旧版递归绘制路径，仅在渲染模型模块不可用时使用。"""
        link = self.renderer.links.get(link_name)
        if link is None:
            return

        gl.glPushMatrix()

        if parent_joint is not None:
            jx, jy, jz = parent_joint["origin_xyz"]
            gl.glTranslatef(jx, jy, jz)

            jrx, jry, jrz = parent_joint["origin_rpy"]
            if any(v != 0 for v in [jrx, jry, jrz]):
                gl.glRotatef(math.degrees(jrz), 0, 0, 1)
                gl.glRotatef(math.degrees(jry), 0, 1, 0)
                gl.glRotatef(math.degrees(jrx), 1, 0, 0)

            q = renderer.joint_values.get(parent_joint["name"], 0.0)
            if parent_joint["type"] == "revolute" and q != 0:
                ax, ay, az = parent_joint["axis"]
                gl.glRotatef(math.degrees(q), ax, ay, az)

        ox, oy, oz = link["origin_xyz"]
        orx, ory, orz = link["origin_rpy"]
        if any(v != 0 for v in [orx, ory, orz]):
            gl.glRotatef(math.degrees(orz), 0, 0, 1)
            gl.glRotatef(math.degrees(ory), 0, 1, 0)
            gl.glRotatef(math.degrees(orx), 1, 0, 0)
        gl.glTranslatef(ox, oy, oz)

        gl.glColor3f(*link["color"])
        geo = link["type"]
        if geo == "cylinder":
            radius, length = link["params"]
            self._draw_cylinder(radius, length)
        elif geo == "box":
            w, h, d = link["params"]
            self._draw_box(w, h, d)
        elif geo == "sphere":
            radius = link["params"][0]
            self._draw_sphere(radius)

        for jname, joint in self.renderer.joints.items():
            if joint["parent"] == link_name:
                self._draw_link_recursive(joint["child"], joint, renderer=renderer)

        gl.glPopMatrix()

    def _draw_cylinder(self, radius, length):
        """绘制沿局部 Z 轴的圆柱（URDF 约定）。

        两处改动直接改善观感：

        * **逐顶点径向法线**（旧代码每个四边形只设一次法线）→ Gouraud 平滑着色，
          圆柱不再是一个个平面拼出的多面体。
        * **按半径自适应分段**（旧代码固定 24 段）→ 粗轴不露棱角，细螺栓不过度细分。
        """
        segments = _segments_for_radius(radius)
        half_len = length / 2.0

        cos_table = [math.cos(2 * math.pi * i / segments) for i in range(segments + 1)]
        sin_table = [math.sin(2 * math.pi * i / segments) for i in range(segments + 1)]

        # 侧面：每个四边形拆两个三角形，绕向 A→B→C→D 保证法线朝外
        gl.glBegin(GL_TRIANGLES)
        for i in range(segments):
            c1, s1 = cos_table[i], sin_table[i]
            c2, s2 = cos_table[i + 1], sin_table[i + 1]
            # A(-h, i) → B(-h, i+1) → C(+h, i+1) → D(+h, i)
            gl.glNormal3f(c1, s1, 0.0)
            gl.glVertex3f(radius * c1, radius * s1, -half_len)
            gl.glNormal3f(c2, s2, 0.0)
            gl.glVertex3f(radius * c2, radius * s2, -half_len)
            gl.glNormal3f(c2, s2, 0.0)
            gl.glVertex3f(radius * c2, radius * s2, half_len)

            gl.glNormal3f(c1, s1, 0.0)
            gl.glVertex3f(radius * c1, radius * s1, -half_len)
            gl.glNormal3f(c2, s2, 0.0)
            gl.glVertex3f(radius * c2, radius * s2, half_len)
            gl.glNormal3f(c1, s1, 0.0)
            gl.glVertex3f(radius * c1, radius * s1, half_len)
        gl.glEnd()

        # 顶盖：从 +Z 看逆时针
        gl.glBegin(GL_TRIANGLE_FAN)
        gl.glNormal3f(0, 0, 1)
        gl.glVertex3f(0, 0, half_len)
        for i in range(segments + 1):
            a = 2 * math.pi * i / segments
            gl.glVertex3f(radius * math.cos(a), radius * math.sin(a), half_len)
        gl.glEnd()
        # 底盖：从 -Z 看逆时针（顶点序反转）
        gl.glBegin(GL_TRIANGLE_FAN)
        gl.glNormal3f(0, 0, -1)
        gl.glVertex3f(0, 0, -half_len)
        for i in range(segments, -1, -1):
            a = 2 * math.pi * i / segments
            gl.glVertex3f(radius * math.cos(a), radius * math.sin(a), -half_len)
        gl.glEnd()

    def _draw_box(self, w, h, d):
        x, y, z = w/2, h/2, d/2
        gl.glBegin(GL_QUADS)
        gl.glNormal3f(0, 0, 1)
        gl.glVertex3f(-x, -y, z); gl.glVertex3f(x, -y, z)
        gl.glVertex3f(x, y, z); gl.glVertex3f(-x, y, z)
        gl.glNormal3f(0, 0, -1)
        gl.glVertex3f(-x, -y, -z); gl.glVertex3f(-x, y, -z)
        gl.glVertex3f(x, y, -z); gl.glVertex3f(x, -y, -z)
        gl.glNormal3f(1, 0, 0)
        gl.glVertex3f(x, -y, -z); gl.glVertex3f(x, y, -z)
        gl.glVertex3f(x, y, z); gl.glVertex3f(x, -y, z)
        gl.glNormal3f(-1, 0, 0)
        gl.glVertex3f(-x, -y, -z); gl.glVertex3f(-x, -y, z)
        gl.glVertex3f(-x, y, z); gl.glVertex3f(-x, y, -z)
        gl.glNormal3f(0, 1, 0)
        gl.glVertex3f(-x, y, -z); gl.glVertex3f(-x, y, z)
        gl.glVertex3f(x, y, z); gl.glVertex3f(x, y, -z)
        gl.glNormal3f(0, -1, 0)
        gl.glVertex3f(-x, -y, -z); gl.glVertex3f(x, -y, -z)
        gl.glVertex3f(x, -y, z); gl.glVertex3f(-x, -y, z)
        gl.glEnd()

    def _draw_sphere(self, radius, stacks=None, slices=None):
        """绘制球（逐顶点径向法线 + 按半径自适应细分）。

        Args:
            radius: 半径。
            stacks: 纬向分段数；``None`` 时按半径自适应。
            slices: 经向分段数；``None`` 时按半径自适应。
        """
        if slices is None:
            segments = _segments_for_radius(radius)
            slices = max(12, segments)
            stacks = max(8, slices // 2)
        for i in range(stacks):
            lat0 = math.pi * (-0.5 + i / stacks)
            lat1 = math.pi * (-0.5 + (i + 1) / stacks)
            z0 = radius * math.sin(lat0)
            zr0 = radius * math.cos(lat0)
            z1 = radius * math.sin(lat1)
            zr1 = radius * math.cos(lat1)
            gl.glBegin(GL_QUAD_STRIP)
            for j in range(slices + 1):
                lng = 2 * math.pi * j / slices
                x = math.cos(lng)
                y = math.sin(lng)
                gl.glNormal3f(x * math.cos(lat0), y * math.cos(lat0), math.sin(lat0))
                gl.glVertex3f(x * zr0, y * zr0, z0)
                gl.glNormal3f(x * math.cos(lat1), y * math.cos(lat1), math.sin(lat1))
                gl.glVertex3f(x * zr1, y * zr1, z1)
            gl.glEnd()

    def _draw_recorded_path(self):
        if not self.recorded_ee_positions or len(self.recorded_ee_positions) < 2:
            return
        # 绘制路径线
        gl.glLineWidth(2.0)
        gl.glColor3f(0.2, 0.8, 0.2)
        gl.glBegin(GL_LINE_STRIP)
        for pos in self.recorded_ee_positions:
            x, y, z = pos
            gl.glVertex3f(x, y, z)
        gl.glEnd()
        
        # 绘制路径点
        gl.glPointSize(5.0)
        gl.glColor3f(1.0, 0.3, 0.3)
        gl.glBegin(GL_POINTS)
        for pos in self.recorded_ee_positions:
            x, y, z = pos
            gl.glVertex3f(x, y, z)
        gl.glEnd()

    # ------------------------------------------------------------------ #
    # 插件渲染覆盖层（v1.0.4）
    # ------------------------------------------------------------------ #

    def set_overlay_context(self, context) -> None:
        """设置交给渲染插件的上下文（``None`` 表示暂无轨迹）。"""
        self.plugin_overlay_context = context

    def refresh_plugin_overlays(self):
        """重新收集渲染插件覆盖层。

        Returns:
            :class:`orolp.plugins.OverlayCollection`，或插件层不可用时的 ``None``
        """
        if not _HAS_PLUGINS:
            return None
        try:
            ctx = self.plugin_overlay_context
            if ctx is None:
                ctx = OverlayContext()
            collected = collect_overlays(ctx)
        except Exception as exc:  # 插件层异常不应让视口变黑
            print(f"[warning] plugin overlays failed: {exc}")
            self.plugin_overlays = None
            self.plugin_overlay_errors = [str(exc)]
            return None
        self.plugin_overlays = collected
        # 顺手把错误回灌到注册表，让"插件状态"对话框能看到真正发生的事
        for err in collected.errors:
            if _HAS_PLUGINS:
                try:
                    default_registry.add_error(err.name, err.source, err.message, err.detail)
                except Exception:
                    pass
        self.plugin_overlay_errors = [f"{e.name}: {e.message}" for e in collected.errors]
        if collected.errors:
            for msg in self.plugin_overlay_errors:
                print(f"[warning] 渲染插件: {msg}")
        self.update()
        return collected

    def _draw_overlays(self) -> None:
        """绘制插件产出的覆盖层（纯数据 → GL 调用，覆盖层坐标是毫米）。"""
        if self.plugin_overlays is None:
            return
        mm = self.MM_TO_M
        try:
            for _name, item in self.plugin_overlays.drawables:
                if isinstance(item, LineSegments):
                    gl.glLineWidth(max(1.0, float(item.width)))
                    gl.glColor3f(*item.color)
                    gl.glBegin(GL_LINES)
                    for p0, p1 in item.segments:
                        gl.glVertex3f(float(p0[0]) * mm, float(p0[1]) * mm, float(p0[2]) * mm)
                        gl.glVertex3f(float(p1[0]) * mm, float(p1[1]) * mm, float(p1[2]) * mm)
                    gl.glEnd()
                elif isinstance(item, PointCloud):
                    gl.glPointSize(max(1.0, float(item.size)))
                    gl.glColor3f(*item.color)
                    gl.glBegin(GL_POINTS)
                    for p in item.positions:
                        gl.glVertex3f(float(p[0]) * mm, float(p[1]) * mm, float(p[2]) * mm)
                    gl.glEnd()
                elif isinstance(item, FrameMarker):
                    self._draw_overlay_frame(item, mm)
        except Exception as exc:  # 画坏一个覆盖层不能毁掉整帧
            print(f"[warning] overlay draw failed: {exc}")

    def _draw_overlay_frame(self, marker, mm: float) -> None:
        """画位姿三轴（X 偏红轴、Y 偏绿轴、Z 偏蓝轴）。"""
        import numpy as _np

        pose = marker.pose
        try:
            mat = _np.asarray(pose.matrix, dtype=float)
        except Exception:
            return
        origin = mat[:3, 3] * mm
        scale = float(marker.scale) * mm
        base = tuple(float(c) for c in marker.color)
        axes = (
            (mat[:3, 0], (base[0], base[1] * 0.4, base[2] * 0.4)),
            (mat[:3, 1], (base[0] * 0.4, base[1], base[2] * 0.4)),
            (mat[:3, 2], (base[0] * 0.4, base[1] * 0.4, base[2])),
        )
        gl.glLineWidth(2.0)
        gl.glBegin(GL_LINES)
        for axis, color in axes:
            gl.glColor3f(*color)
            gl.glVertex3f(float(origin[0]), float(origin[1]), float(origin[2]))
            tip = origin + _np.asarray(axis, dtype=float) * scale
            gl.glVertex3f(float(tip[0]), float(tip[1]), float(tip[2]))
        gl.glEnd()

    # ------------------------------------------------------------------ #
    # 右键拾取 / 坐标系 gizmo 变换
    # ------------------------------------------------------------------ #

    def _view_proj_matrix(self):
        """返回 (V, P, M)，与 paintGL 完全一致的相机矩阵（米制）。"""
        w, h = self.width(), self.height()
        aspect = w / max(1, h)
        fov = 45.0
        near, far = 0.01, 100.0
        top = near * math.tan(math.radians(fov / 2))
        right = top * aspect

        az = math.radians(self.camera_az)
        el = math.radians(self.camera_el)
        center = np.asarray(self.view_center, dtype=np.float64)
        eye = center + np.array(
            [
                self.camera_dist * math.cos(el) * math.cos(az),
                self.camera_dist * math.cos(el) * math.sin(az),
                self.camera_dist * math.sin(el),
            ],
            dtype=np.float64,
        )
        up_vec = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        f = center - eye
        norm = np.linalg.norm(f)
        if norm > 0:
            f = f / norm
        s = np.cross(f, up_vec)
        norm = np.linalg.norm(s)
        if norm > 0:
            s = s / norm
        u = np.cross(s, f)

        # numpy 行主序：paintGL 用 glLoadMatrixf（列主序）加载的视图矩阵，
        # 其旋转部分的**行**依次为 s, u, -f。这里必须用【行赋值】，否则旋转
        # 被转置（正交矩阵转置=逆），反推射线方向错误，拾取必然失败。
        V = np.eye(4)
        V[0, :3] = s
        V[1, :3] = u
        V[2, :3] = -f
        V[:3, 3] = -(V[:3, :3] @ eye)

        P = np.zeros((4, 4))
        P[0, 0] = near / right
        P[1, 1] = near / top
        P[2, 2] = -(far + near) / (far - near)
        P[2, 3] = -2.0 * far * near / (far - near)
        P[3, 2] = -1.0
        M = P @ V
        return V, P, M

    def _ray_from_mouse(self, x, y):
        """返回 (origin_m, dir_m)：世界系米制射线原点与单位方向。"""
        w, h = self.width(), self.height()
        ndc_x = (2.0 * x) / w - 1.0
        ndc_y = 1.0 - (2.0 * y) / h
        _, _, M = self._view_proj_matrix()
        Minv = np.linalg.inv(M)
        near_h = Minv @ np.array([ndc_x, ndc_y, -1.0, 1.0])
        far_h = Minv @ np.array([ndc_x, ndc_y, 1.0, 1.0])
        near_p = near_h[:3] / near_h[3]
        far_p = far_h[:3] / far_h[3]
        origin = near_p
        direction = far_p - near_p
        n = np.linalg.norm(direction)
        if n > 0:
            direction = direction / n
        return origin, direction

    @staticmethod
    def _ray_aabb(origin, direction, lo, hi):
        """射线-轴对齐包围盒 slab 求交，返回最近 t（>=0），无交返回 None。"""
        tmin = -np.inf
        tmax = np.inf
        for i in range(3):
            if abs(direction[i]) < 1e-12:
                if origin[i] < lo[i] or origin[i] > hi[i]:
                    return None
            else:
                inv = 1.0 / direction[i]
                t1 = (lo[i] - origin[i]) * inv
                t2 = (hi[i] - origin[i]) * inv
                tmin = max(tmin, min(t1, t2))
                tmax = min(tmax, max(t1, t2))
        if tmax < max(tmin, 0.0):
            return None
        return max(tmin, 0.0)

    def _ray_plane_z(self, origin, direction, z_m):
        """射线与水平面 z=z_m 的交点（米）；平行或无交点返回 None。"""
        if abs(direction[2]) < 1e-9:
            return None
        t = (z_m - origin[2]) / direction[2]
        if t < 0:
            return None
        return origin + t * direction

    def _project(self, p_m):
        """世界点（米）投影到视口像素坐标 (sx, sy)，y 轴向下。"""
        w, h = self.width(), self.height()
        _, _, M = self._view_proj_matrix()
        clip = M @ np.array([p_m[0], p_m[1], p_m[2], 1.0])
        if abs(clip[3]) < 1e-9:
            return (w / 2.0, h / 2.0)
        ndc_x = clip[0] / clip[3]
        ndc_y = clip[1] / clip[3]
        sx = (ndc_x + 1.0) / 2.0 * w
        sy = (1.0 - (ndc_y + 1.0) / 2.0) * h
        return float(sx), float(sy)

    def _picking_parts(self, name):
        """对象参与**拾取与选中描边**的几何（世界系）。

        机器人必须改用**碰撞包络**：真实 URDF 的 ``<visual>`` 几乎全是
        ``<mesh>``（场景解析器不解析），用 visual 会让整台机器人只剩一个 TCP
        标记球 —— 射线打空、描边也只有一个小球，表现为"看得见却选不中"。
        干涉检查早就这么处理了，拾取与描边与此保持一致。
        """
        if self.scene.is_robot(name):
            return [
                p
                for group in self.scene.robot_world_link_parts(
                    name, source=GEOMETRY_COLLISION
                ).values()
                for p in group
            ]
        return self.scene.object_parts(name)

    def _pick_object(self, x, y):
        """射线-AABB 求交，返回最近场景对象名；无命中返回 None。"""
        if not _HAS_WORLD or self.scene is None:
            return None
        try:
            origin, direction = self._ray_from_mouse(x, y)
            best = None
            best_t = float("inf")
            for name in self.scene.objects:
                obj = self.scene.objects.get(name)
                if obj is None or not obj.visible:
                    continue
                lo = np.array([np.inf, np.inf, np.inf])
                hi = np.array([-np.inf, -np.inf, -np.inf])
                for part in self._picking_parts(name):
                    try:
                        aabb = prim_aabb(part.prim, part.transform)
                        plo, phi = aabb.lo, aabb.hi
                    except Exception:
                        # mesh 等无解析 AABB 的基本体：直接用顶点包络
                        mesh = getattr(part.prim, "mesh", None)
                        if mesh is None or getattr(mesh, "is_empty", True):
                            continue
                        R = part.transform[:3, :3]
                        t = part.transform[:3, 3]
                        pts = (np.asarray(mesh.vertices, dtype=np.float64) @ R.T) + t
                        plo, phi = pts.min(axis=0), pts.max(axis=0)
                    lo = np.minimum(lo, plo * self.MM_TO_M)
                    hi = np.maximum(hi, phi * self.MM_TO_M)
                if not np.all(np.isfinite(lo)) or not np.all(np.isfinite(hi)):
                    continue
                t = self._ray_aabb(origin, direction, lo, hi)
                if t is not None and t < best_t:
                    best_t = t
                    best = name
            return best
        except Exception as _exc:  # 拾取异常不应拖垮视口
            print(f"[warning] pick failed: {_exc}")
            return None

    def _gizmo_screen_geom(self):
        """返回 gizmo 在屏幕上的几何（像素），供绘制与命中判定共用。"""
        if self.selected_object is None or self.scene is None:
            return None
        obj = self.scene.objects.get(self.selected_object)
        if obj is None:
            return None
        tf = self.scene.world_transform(self.selected_object)  # mm
        center_m = tf[:3, 3] * self.MM_TO_M
        cx, cy = self._project(center_m)
        z_up = self._project(center_m + np.array([0.0, 0.0, 0.1]))  # 上方 0.1m
        px_per_m = math.hypot(z_up[0] - cx, z_up[1] - cy) / 0.1
        return {
            "cx": cx, "cy": cy,
            "ring_r": 55.0,      # 旋转环半径(px)
            "axis_len": 75.0,    # Z 轴长度(px)
            "xy_half": 42.0,     # XY 方块半边长(px)
            "hit": 10.0,         # 命中容差(px)
            "z_strip": 16.0,     # Z 轴竖条命中半宽(px)，加宽便于抓取
            "z_tip_r": 18.0,     # Z 轴顶端圆盘抓取半径(px)
            "px_per_m": px_per_m,
        }

    def _gizmo_hit_mode(self, x, y):
        """屏幕空间判定光标命中的手柄，返回 'xy' / 'z' / 'rz' / None。"""
        g = self._gizmo_screen_geom()
        if g is None:
            return None
        dx = x - g["cx"]
        dy = y - g["cy"]
        dist = math.hypot(dx, dy)
        # 1) Z 轴：顶端圆盘抓取点 + 整条加宽竖条，优先于 XY 方块
        if math.hypot(dx, dy + g["axis_len"]) < g["z_tip_r"]:
            return "z"
        if abs(dx) < g["z_strip"] and -g["axis_len"] - g["hit"] <= dy <= g["hit"]:
            return "z"
        # 2) 旋转环
        if abs(dist - g["ring_r"]) < g["hit"]:
            return "rz"
        # 3) XY 方块
        if abs(dx) < g["xy_half"] and abs(dy) < g["xy_half"]:
            return "xy"
        return None

    def _set_object_world_pose(self, name, x_mm, y_mm, z_mm, rz_deg):
        """把对象设到指定的世界位姿（毫米 + 绕Z角度），自动换算回局部 pose。

        世界系 → 局部系的换算只有一处定义：``world.placement.apply_world_pose``。
        """
        if not _HAS_WORLD or self.scene is None:
            return
        _placement.apply_world_pose(self.scene, name, x_mm, y_mm, z_mm, rz_deg)

    def _grabbable(self, name) -> bool:
        """对象是否允许左键拖拽摆放（机器人/挂载对象/锁定对象除外）。"""
        if not _HAS_WORLD or self.scene is None or _placement is None:
            return False
        return _placement.grabbable(self.scene, name)

    def _grab_start(self, name, x, y):
        """左键按在工件上：进入拖拽摆放，锁定"按住时的世界高度平面"。"""
        self.selected_object = name
        self.drag_mode = None
        # 已装夹到法兰的工具/工件：先摘下来再拖（detach 保持世界位姿，不跳变）
        if self._detach_mounted(name):
            self.sceneChanged.emit(name)
        self._grab_active = True
        self._grab_anchor = self._make_anchor("xy", x, y)
        self.objectSelected.emit(name)

    def _grab_move(self, x, y):
        """拖动中：沿按下时的高度平面移动 XY（Z 不变，松手才落面）。"""
        if self._grab_anchor is None or self.selected_object is None:
            return
        self._apply_drag("xy", self._grab_anchor, x, y)

    def _grab_drop(self):
        """松手：落到正下方支承面（几何算出来的，不再手填 Z）。"""
        if not _HAS_WORLD or self.scene is None or self.selected_object is None:
            return
        name = self.selected_object
        # 装夹优先于落面：把夹爪拖到臂末端松手，期望是"装上"而不是"掉地上"
        robot = self._try_snap_mount(name)
        if robot:
            self.update()
            self.objectSelected.emit(name)
            self.sceneChanged.emit(name)
            self.objectPlaced.emit(
                tr("已将 {name} 装夹到 {host} 末端").format(name=name, host=robot)
            )
            return
        tf = self.scene.world_transform(name)
        result = _placement.place_on_support(
            self.scene, name, float(tf[0, 3]), float(tf[1, 3]),
            snap_step=self.snap_step_mm,
        )
        # 吸附只作用于落点 XY：place_on_support 已按 snap_step 写回位姿，
        # 这里只负责把结论说给状态栏听
        self.update()
        self.objectSelected.emit(name)
        self.objectPlaced.emit(_placement.placement_summary(result))

    def _detach_mounted(self, name):
        """起拖前解除法兰挂载（保持世界位姿）。

        不解除的话，写回的局部位姿会变成"工具偏移"：名义上还挂在法兰下、
        位置却被挪开了，用户难以理解。只对**挂在机器人法兰下**的对象生效——
        挂在普通对象下的子对象仍由 placement 层按原规则拒绝拖动。

        Returns:
            是否真的解除过
        """
        if not name or self.scene is None or _placement is None:
            return False
        if not _placement.mounted_on_robot(self.scene, name):
            return False
        try:
            self.scene.detach(name)
        except Exception as exc:  # 只有对象不存在时才会失败
            print(f"[warning] 自动解除挂载失败: {exc}")
            return False
        return True

    def _try_snap_mount(self, name):
        """松手时若停在机器人法兰附近，就装夹到该机器人末端。

        装夹＝局部位姿归零：工具坐标系自动与法兰对齐，而不是保留当前世界位置。

        Returns:
            命中的机器人名；未装夹返回 ``None``
        """
        if not name or self.scene is None or _placement is None:
            return None
        if self.scene.is_robot(name):
            return None  # 机器人本体不能被装夹到别的机器人法兰上
        robot = _placement.flange_snap_target(
            self.scene, name, getattr(self, "snap_radius_mm", 60.0)
        )
        if robot is None or name == robot:
            return None
        try:
            _placement.mount_to_flange(self.scene, name, robot)
        except (KeyError, ValueError) as exc:
            print(f"[warning] 自动装夹失败: {exc}")
            return None
        return robot

    def _make_anchor(self, mode, x, y):
        """记录拖动起始参考量（对象世界位姿 + 模式相关的局部参考）。"""
        name = self.selected_object
        tf = self.scene.world_transform(name)  # mm
        start_x, start_y, start_z = tf[:3, 3]
        start_rz = math.degrees(math.atan2(tf[1, 0], tf[0, 0]))
        center_m = tf[:3, 3] * self.MM_TO_M
        anchor = {
            "start_x": float(start_x), "start_y": float(start_y),
            "start_z": float(start_z), "start_rz": float(start_rz),
            "center_m": center_m,
        }
        if mode == "xy":
            origin, direction = self._ray_from_mouse(x, y)
            p = self._ray_plane_z(origin, direction, center_m[2])
            anchor["plane_start"] = p if p is not None else center_m.copy()
        elif mode == "z":
            anchor["start_mouse_y"] = y
            g = self._gizmo_screen_geom()
            anchor["px_per_m"] = g["px_per_m"] if g else 1000.0
        elif mode == "rz":
            g = self._gizmo_screen_geom()
            anchor["start_cursor_angle"] = math.atan2(y - g["cy"], x - g["cx"])
        return anchor

    def _apply_drag(self, mode, anchor, x, y):
        """按已锁定的模式执行变换，并写回对象位姿。"""
        name = self.selected_object
        if mode == "xy":
            origin, direction = self._ray_from_mouse(x, y)
            p = self._ray_plane_z(origin, direction, anchor["center_m"][2])
            if p is None:
                return
            dx_m = p[0] - anchor["plane_start"][0]
            dy_m = p[1] - anchor["plane_start"][1]
            nx = anchor["start_x"] + dx_m * 1000.0
            ny = anchor["start_y"] + dy_m * 1000.0
            self._set_object_world_pose(name, nx, ny, anchor["start_z"], anchor["start_rz"])
        elif mode == "z":
            dy_px = anchor["start_mouse_y"] - y  # 屏幕向上为正
            mm_per_px = 1000.0 / max(anchor["px_per_m"], 1e-6)
            nz = anchor["start_z"] + dy_px * mm_per_px
            self._set_object_world_pose(name, anchor["start_x"], anchor["start_y"], nz, anchor["start_rz"])
        elif mode == "rz":
            g = self._gizmo_screen_geom()
            cur = math.atan2(y - g["cy"], x - g["cx"])
            delta = cur - anchor["start_cursor_angle"]
            new_rz = anchor["start_rz"] - math.degrees(delta)  # 屏幕 y 向下，取负使视觉一致
            self._set_object_world_pose(name, anchor["start_x"], anchor["start_y"], anchor["start_z"], new_rz)
        self.update()
        self.objectSelected.emit(name)  # 同步刷新左侧数值框

    def _handle_right_drag(self, pos):
        """右键拖动：先探测手柄锁定模式，再执行对应变换。"""
        if not _HAS_WORLD or self.scene is None:
            return
        x, y = pos.x(), pos.y()
        if self.drag_mode is None:
            mode = self._gizmo_hit_mode(x, y)
            if mode is None:
                return
            # 右键 gizmo 与左键一致：拖已装夹件前先摘下来
            if self._detach_mounted(self.selected_object):
                self.sceneChanged.emit(self.selected_object)
            self._drag_anchor = self._make_anchor(mode, x, y)
            self.drag_mode = mode
            self.update()
            return
        self._apply_drag(self.drag_mode, self._drag_anchor, x, y)

    def _draw_gizmo(self):
        """在 selected_object 原点绘制 XY 面 / Z 轴 / 绕Z 环手柄，高亮当前 drag_mode。"""
        if self.selected_object is None or self.scene is None:
            return
        g = self._gizmo_screen_geom()
        if g is None:
            return
        cx, cy = g["cx"], g["cy"]

        gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glDisable(gl.GL_LIGHTING)
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glPushMatrix()
        gl.glLoadIdentity()
        gl.glOrtho(0, self.width(), self.height(), 0, -1, 1)  # 屏幕像素，y 向下
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glPushMatrix()
        gl.glLoadIdentity()

        def line(x1, y1, x2, y2, color, width=2.0):
            gl.glColor3f(*color)
            gl.glLineWidth(width)
            gl.glBegin(gl.GL_LINES)
            gl.glVertex2f(x1, y1)
            gl.glVertex2f(x2, y2)
            gl.glEnd()

        def ring(cx, cy, r, color, width=2.0):
            gl.glColor3f(*color)
            gl.glLineWidth(width)
            gl.glBegin(gl.GL_LINE_LOOP)
            for i in range(48):
                a = 2 * math.pi * i / 48
                gl.glVertex2f(cx + r * math.cos(a), cy + r * math.sin(a))
            gl.glEnd()

        def rect(cx, cy, h, color, width=2.0):
            gl.glColor3f(*color)
            gl.glLineWidth(width)
            gl.glBegin(gl.GL_LINE_LOOP)
            gl.glVertex2f(cx - h, cy - h)
            gl.glVertex2f(cx + h, cy - h)
            gl.glVertex2f(cx + h, cy + h)
            gl.glVertex2f(cx - h, cy + h)
            gl.glEnd()

        def disk(cx, cy, r, color):
            gl.glColor3f(*color)
            gl.glBegin(gl.GL_TRIANGLE_FAN)
            gl.glVertex2f(cx, cy)
            for i in range(25):
                a = 2 * math.pi * i / 24
                gl.glVertex2f(cx + r * math.cos(a), cy + r * math.sin(a))
            gl.glEnd()

        mode = self.drag_mode
        z_col = (1.0, 0.3, 0.25) if mode == "z" else (0.85, 0.45, 0.4)
        line(cx, cy, cx, cy - g["axis_len"], z_col, 3.0 if mode == "z" else 2.0)
        disk(cx, cy - g["axis_len"], g["z_tip_r"] * 0.6, z_col)  # 顶端抓取圆盘
        r_col = (1.0, 0.85, 0.2) if mode == "rz" else (0.85, 0.7, 0.2)
        ring(cx, cy, g["ring_r"], r_col, 3.0 if mode == "rz" else 2.0)
        xy_col = (0.3, 0.7, 1.0) if mode == "xy" else (0.35, 0.55, 0.8)
        rect(cx, cy, g["xy_half"], xy_col, 3.0 if mode == "xy" else 2.0)
        gl.glColor3f(1.0, 1.0, 1.0)
        gl.glPointSize(5.0)
        gl.glBegin(gl.GL_POINTS)
        gl.glVertex2f(cx, cy)
        gl.glEnd()

        gl.glPopMatrix()
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glPopMatrix()
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glEnable(gl.GL_LIGHTING)
        gl.glEnable(gl.GL_DEPTH_TEST)

    def mousePressEvent(self, event):
        # Mac 触控板：两指轻点发出 RightButton；Control+左键（Control+Click）
        # 也是经典右键替代，这里统一当作“右键拾取”处理。
        is_right_like = (
            event.button() == Qt.RightButton
            or (event.button() == Qt.LeftButton
                and event.modifiers() & Qt.ControlModifier)
        )
        if is_right_like:
            self._right_like = True
            if _HAS_WORLD and self.scene is not None:
                pos = event.position().toPoint()
                name = self._pick_object(pos.x(), pos.y())
                if name is not None:
                    self.selected_object = name
                    self.drag_mode = None
                    self._drag_anchor = None
                    self.objectSelected.emit(name)
                else:
                    self.selected_object = None
                    self.drag_mode = None
                self.update()
        elif event.button() == Qt.LeftButton:
            pos = event.position().toPoint()
            self._left_press_pos = pos
            self._left_moved = False
            # 左键按在可摆放的工件/设备上 = 拖拽摆放（v1.1.7）；
            # 按在机器人或空白处 = 旋转视角（与既有行为一致）
            if _HAS_WORLD and self.scene is not None:
                name = self._pick_object(pos.x(), pos.y())
                if self._grabbable(name):
                    self._grab_start(name, pos.x(), pos.y())
                    self.update()
                    return
            self.dragging = True
            self.last_pos = pos
            # 左键按下即尝试拾取对象，让机器人/组件一点就高亮
            if _HAS_WORLD and self.scene is not None:
                name = self._pick_object(pos.x(), pos.y())
                if name is not None:
                    self.selected_object = name
                    self.objectSelected.emit(name)
                else:
                    self.selected_object = None
                self.update()

    def mouseMoveEvent(self, event):
        # 左键拖拽摆放中：只移动工件，不转视角
        if self._grab_active and (event.buttons() & Qt.LeftButton):
            self._left_moved = True
            self._grab_move(event.position().toPoint().x(),
                            event.position().toPoint().y())
            return

        # 右键（或 Control+左键）按住时做 gizmo 变换，不做视角旋转
        if self._right_like and (event.buttons() & (Qt.RightButton | Qt.LeftButton)):
            if self.selected_object is not None:
                self._handle_right_drag(event.position().toPoint())
            return

        if self.dragging and self.last_pos:
            current_pos = event.position().toPoint()
            if self._left_press_pos is not None and \
                    (current_pos - self._left_press_pos).manhattanLength() > 4:
                self._left_moved = True
            dx = current_pos.x() - self.last_pos.x()
            dy = current_pos.y() - self.last_pos.y()
            self.camera_az -= dx * 0.5
            self.camera_el += dy * 0.5
            self.camera_el = max(-89, min(89, self.camera_el))
            self.last_pos = current_pos
            self.update()

        if event.buttons() & Qt.RightButton and self.selected_object is not None:
            self._handle_right_drag(event.position().toPoint())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.dragging = False
            self._right_like = False
            if self._grab_active:
                # 拖拽摆放收尾：落到支承面并把结论报给状态栏
                self._grab_drop()
                self._grab_active = False
                self._grab_anchor = None
                self._left_moved = False
                self._left_press_pos = None
                return
            if not self._left_moved and _HAS_WORLD and self.scene is not None:
                pos = event.position().toPoint()
                name = self._pick_object(pos.x(), pos.y())
                if name is not None:
                    self.selected_object = name
                    self.objectSelected.emit(name)
                else:
                    self.selected_object = None
                self.update()
            self._left_moved = False
            self._left_press_pos = None
        elif event.button() == Qt.RightButton:
            self._right_like = False
        # 保留选中与 gizmo 显示；仅清除本次拖动模式，下次右键重新判定
        self.drag_mode = None
        self._drag_anchor = None

    def wheelEvent(self, event):
        delta = event.angleDelta().y() / 120
        self.camera_dist *= 0.9 ** delta
        self.camera_dist = max(0.3, min(10.0, self.camera_dist))
        self.update()


class PathCodeEditor(QtWidgets.QWidget):
    positionRecorded = Signal(int)
    playbackRequested = Signal(int)
    playbackStopped = Signal()
    clearRequested = Signal()
    positionsParsed = Signal()  # 新增：代码解析完成信号
    #: 状态栏提示。本类是普通 QWidget、自己没有状态栏，主窗口不在时靠这条
    #: 信号把提示送出去（无接收者时静默丢弃，不会抛异常）
    statusMessage = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.recorded_positions = []
        self.joint_names = []  # 存储关节名称顺序
        self.playback_timer = None
        self.current_playback_index = 0
        self.playback_speed = 50
        #: 与 recorded_positions 一一对应的工艺语义（PointType）。
        #: 抓取点/放置点会在播放时触发工件吸附与释放，并在生成代码时插入 IO 指令
        self.point_types = []
        #: 工作单元场景（由 MainWindow 注入），用于枚举可抓取的工件
        self._scene = None
        #: 工作单元模型（硬件列表 / PLC 程序逻辑共用同一份）
        self.cell = CellConfig() if _HAS_CELL else None
        #: 供「机器人子程序」生成用的真实机器人（含运动学）
        self._robot = None
        self._scene = None
        self._setup_ui()
        self.setPlainText("# OLP Path Code\nmovej([0, -90, 90, -90, 90, 0], vel=0.5)\n")

    def _setup_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        self.sub_tabs = QtWidgets.QTabWidget()

        # ---------- 1) 路径示教（原有能力） ----------
        teach = QtWidgets.QWidget()
        teach_layout = QtWidgets.QVBoxLayout(teach)
        teach_layout.setContentsMargins(0, 0, 0, 0)
        teach_layout.setSpacing(2)

        self.code_edit = QtWidgets.QTextEdit()
        self.code_edit.setFont(QFont("Consolas", 10))
        self.code_edit.setStyleSheet("background-color: #1a1a1a; color: #e0e0e0;")
        self.code_edit.setLineWrapMode(QtWidgets.QTextEdit.NoWrap)
        self.code_edit.setTabStopDistance(20)
        teach_layout.addWidget(self.code_edit)

        control_layout = QtWidgets.QHBoxLayout()

        self.record_btn = QtWidgets.QPushButton("● Record")
        self.record_btn.setStyleSheet("background-color: #555555; color: white;")
        self.record_btn.setCheckable(True)
        self.record_btn.clicked.connect(self._on_record_toggled)

        self.play_btn = QtWidgets.QPushButton("▶ Play")
        self.play_btn.setStyleSheet("background-color: #4CAF50; color: white;")
        self.play_btn.clicked.connect(self._on_play_clicked)

        self.pause_btn = QtWidgets.QPushButton(tr("⏸ 暂停"))
        self.pause_btn.setStyleSheet("background-color: #8a8a8a; color: white;")
        self.pause_btn.setToolTip(tr("暂停 / 继续；暂停保留当前点位，便于逐点核对"))
        self.pause_btn.clicked.connect(self._on_pause_clicked)

        self.step_btn = QtWidgets.QPushButton(tr("⏭ 单步"))
        self.step_btn.setStyleSheet("background-color: #6a6a6a; color: white;")
        self.step_btn.setToolTip(tr("走一个示教点——核对抓取/放置时机最有用的操作"))
        self.step_btn.clicked.connect(self._on_step_clicked)

        self.stop_btn = QtWidgets.QPushButton("■ Stop")
        self.stop_btn.setStyleSheet("background-color: #f44336; color: white;")
        self.stop_btn.clicked.connect(self._on_stop_clicked)

        self.clear_btn = QtWidgets.QPushButton("Clear")
        self.clear_btn.setStyleSheet("background-color: #777777; color: white;")
        self.clear_btn.clicked.connect(self._on_clear_clicked)

        self.pos_count_label = QtWidgets.QLabel("Positions: 0")
        self.pos_count_label.setFont(QFont("Consolas", 9))

        control_layout.addWidget(self.record_btn)
        control_layout.addWidget(self.play_btn)
        control_layout.addWidget(self.pause_btn)
        control_layout.addWidget(self.step_btn)
        control_layout.addWidget(self.stop_btn)
        control_layout.addWidget(self.clear_btn)
        control_layout.addWidget(self.pos_count_label)

        # ---- 抓取工艺：点位语义 + 工件选择 ----
        if _HAS_PICKPLACE:
            control_layout.addSpacing(12)
            control_layout.addWidget(QtWidgets.QLabel(tr("点位:")))
            self.point_type_combo = QtWidgets.QComboBox()
            self.point_type_combo.setToolTip(
                tr(
                    "记录时赋予该示教点的工艺语义。\n"
                    "抓取点：插入夹紧 IO，播放时把工件吸附到法兰；\n"
                    "放置点：插入松开 IO，播放时把工件释放到世界。"
                )
            )
            for t in PointType:
                self.point_type_combo.addItem(t.label, t)
            control_layout.addWidget(self.point_type_combo)

            control_layout.addWidget(QtWidgets.QLabel(tr("工件:")))
            self.workpiece_combo = QtWidgets.QComboBox()
            self.workpiece_combo.setMinimumWidth(90)
            self.workpiece_combo.setToolTip(tr("被抓取的场景对象；留空表示只演示轨迹"))
            control_layout.addWidget(self.workpiece_combo)

        teach_layout.addLayout(control_layout)

        self.sub_tabs.addTab(teach, tr("路径示教"))

        # ---------- 2) 硬件列表 ----------
        self.sub_tabs.addTab(self._build_hardware_tab(), tr("硬件列表"))

        # ---------- 3) PLC 程序逻辑 ----------
        self.sub_tabs.addTab(self._build_plc_tab(), tr("PLC 程序逻辑"))

        # ---------- 4) 机器人子程序 ----------
        self.sub_tabs.addTab(self._build_subprogram_tab(), tr("机器人子程序"))

        layout.addWidget(self.sub_tabs)

    # ---------------- 代码生成：硬件列表 ---------------- #

    def _build_hardware_tab(self):
        w = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(w)
        layout.setContentsMargins(2, 2, 2, 2)

        self.device_table = QtWidgets.QTableWidget(0, 6)
        self.device_table.setHorizontalHeaderLabels(
            [tr("名称"), tr("类型"), tr("厂商"), tr("型号"), tr("通信"), "IP"])
        self.device_table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.ResizeToContents)
        self.device_table.verticalHeader().setVisible(False)
        layout.addWidget(self.device_table, 1)

        row = QtWidgets.QHBoxLayout()
        self.btn_hw_add = QtWidgets.QPushButton(tr("添加设备"))
        self.btn_hw_add.clicked.connect(self._add_device_row)
        row.addWidget(self.btn_hw_add)
        self.btn_hw_del = QtWidgets.QPushButton(tr("删除"))
        self.btn_hw_del.clicked.connect(self._remove_device_row)
        row.addWidget(self.btn_hw_del)
        self.btn_hw_sync = QtWidgets.QPushButton(tr("从场景同步"))
        self.btn_hw_sync.setToolTip(tr("按场景里的机器人/组件补齐设备清单（已存在的保留不动）"))
        self.btn_hw_sync.clicked.connect(self._sync_devices_from_scene)
        row.addWidget(self.btn_hw_sync)
        layout.addLayout(row)

        # IO 点表（v1.1.6）：机器人程序里用的信号必须在这张表上有地址，
        # 否则下装后机器人要么发不出去、要么一直等一个没人给的输入
        row2 = QtWidgets.QHBoxLayout()
        self.btn_io_map = QtWidgets.QPushButton(tr("IO 映射…"))
        self.btn_io_map.setToolTip(
            tr("编辑当前设备的信号地址（点表与 PLC 出码共用这一份数据）")
        )
        self.btn_io_map.clicked.connect(self._edit_io_map)
        row2.addWidget(self.btn_io_map)
        self.btn_io_check = QtWidgets.QPushButton(tr("IO 校验"))
        self.btn_io_check.setToolTip(
            tr("检查未映射 / 地址重复 / 两侧同时驱动 / 机器人等一个没人给的信号")
        )
        self.btn_io_check.clicked.connect(self._check_io_map)
        row2.addWidget(self.btn_io_check)
        row2.addStretch(1)
        layout.addLayout(row2)
        return w

    # ---------------- IO 点表（v1.1.6） ---------------- #

    def _notify(self, text: str, timeout: int = 6000) -> None:
        """把一行提示送到状态栏。

        ``PathCodeEditor`` 是普通 ``QWidget``，**没有自己的状态栏**——早期代码
        直接调 ``self.statusBar()``，一点「生成 PLC」就 AttributeError。
        这里统一走：有主窗口就用主窗口的状态栏，单独实例化（测试/嵌入式）时
        退化为 ``statusMessage`` 信号，既不崩也不静默丢消息。
        """
        bar = getattr(self.window(), "statusBar", None)
        if callable(bar):
            try:
                bar().showMessage(text, timeout)
                return
            except Exception:  # pragma: no cover - 状态栏不可用时的兜底
                pass
        self.statusMessage.emit(text)

    def _signal_map(self):
        """当前单元的点表视图；单元模型不可用时返回 ``None``。"""
        if self.cell is None or SignalMap is None:
            return None
        return SignalMap(self.cell)

    def _current_program_items(self):
        """当前示教点对应的机器人程序指令（点表校验的"用途"来源）。

        与「生成 PLC」走同一条数据通路：优先按**真实示教出来的抓取 IO 时序**，
        没有示教点时返回空——不拿默认骨架冒充真实程序的用法。
        """
        if not _HAS_PICKPLACE:
            return []
        pts = self._taught_points_deg()
        if not pts:
            return []
        try:
            plan = build_pick_place(pts, self.get_point_types())
        except Exception as exc:  # pragma: no cover - 示教数据异常时不该崩
            print(f"[warning] 点表校验取程序失败: {exc}")
            return []
        return list(plan.items)

    def _edit_io_map(self):
        """编辑选中设备的 IO 映射（信号 ↔ 地址）。"""
        smap = self._signal_map()
        if smap is None:
            QtWidgets.QMessageBox.information(self, tr("IO 映射"), tr("单元模型不可用"))
            return
        device = self._selected_device()
        if device is None:
            QtWidgets.QMessageBox.information(self, tr("IO 映射"), tr("请先在设备列表里选中一行"))
            return

        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle(tr("IO 映射 · {name}").format(name=device.name))
        dialog.resize(560, 480)
        layout = QtWidgets.QVBoxLayout(dialog)

        items = self._current_program_items()
        table = QtWidgets.QTableWidget(0, 4)
        table.setHorizontalHeaderLabels(
            [tr("信号"), tr("地址（可编辑）"), tr("方向"), tr("用途")]
        )
        table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.ResizeToContents)
        table.verticalHeader().setVisible(False)
        layout.addWidget(table, 1)

        def refill():
            """重填表格：本设备声明的信号 + 机器人程序在用但没有任何设备的信号。

            后者必须出现——"程序在用、点表里没有"正是最该被补上的那类点，
            藏在别处就等于没提示。
            """
            points = [p for p in smap.points(items) if device.name in p.devices]
            covered = {p.signal for p in points}
            for p in smap.points(items):
                if p.used and p.signal not in covered:
                    points.append(p)
                    covered.add(p.signal)
            table.setRowCount(len(points))
            for r, p in enumerate(points):
                for c, text in enumerate((p.signal, p.address, p.direction_text(), p.usage_text())):
                    item = QtWidgets.QTableWidgetItem(text)
                    if c != 1:
                        item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                    table.setItem(r, c, item)

        refill()

        row = QtWidgets.QHBoxLayout()
        btn_add = QtWidgets.QPushButton(tr("添加信号"))
        btn_auto = QtWidgets.QPushButton(tr("自动编号"))
        btn_auto.setToolTip(tr("按方向自动编号，跳过已被占用的地址"))
        row.addWidget(btn_add)
        row.addWidget(btn_auto)
        row.addStretch(1)
        layout.addLayout(row)

        def add_signal():
            text, ok = QtWidgets.QInputDialog.getText(
                dialog, tr("添加信号"), tr("信号名（如 gripper_close）"))
            if ok and text.strip():
                smap.assign(text.strip(), "", device.name)
                refill()

        def auto_number():
            # 只给本设备上还没有地址的信号编号，且跳过别的信号已占用的地址
            pending = [
                table.item(r, 0).text()
                for r in range(table.rowCount())
                if not table.item(r, 1).text().strip()
            ]
            got = smap.auto_assign(device.name, pending, extra_signals=pending)
            refill()
            self._notify(
                tr("自动编号：{n} 个信号（跳过已占用地址）").format(n=len(got)))

        btn_add.clicked.connect(add_signal)
        btn_auto.clicked.connect(auto_number)

        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        layout.addWidget(buttons)

        def apply_and_close():
            # 把表格里的地址写回点表：空 = 摘掉映射（保留信号声明）
            changed = self._write_io_map_from_table(table, device.name, smap)
            self._notify(
                tr("IO 映射已更新：{name}（{n} 个信号）").format(name=device.name, n=changed))
            dialog.accept()

        buttons.accepted.connect(apply_and_close)
        buttons.rejected.connect(dialog.reject)
        dialog.exec()
        self._check_io_map(silent=True)

    @staticmethod
    def _write_io_map_from_table(table, device_name: str, smap) -> int:
        """把映射表格里的地址写回点表，返回处理过的信号数。

        抽成独立方法是为了能测：对话框里的闭包没法在无头测试里驱动，
        而"表格内容 → 点表"这一步恰恰是唯一有副作用的逻辑。
        """
        handled = 0
        for r in range(table.rowCount()):
            item = table.item(r, 0)
            if item is None:
                continue
            signal = item.text().strip()
            if not signal:
                continue
            addr_item = table.item(r, 1)
            address = addr_item.text().strip() if addr_item is not None else ""
            if address:
                smap.assign(signal, address, device_name)
            else:
                smap.unassign(signal, device_name)
            handled += 1
        return handled

    def _check_io_map(self, silent: bool = False):
        """校验 IO 点表并展示结论。``silent=True`` 时只刷新状态栏（用于编辑后）。"""
        smap = self._signal_map()
        if smap is None:
            return None
        items = self._current_program_items()
        try:
            steps = steps_from_program(items) if items else None
        except Exception:  # pragma: no cover
            steps = None
        report = smap.validate(items, steps)
        summary = report.summary()
        if silent:
            self._notify(summary, 8000)
            return report

        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle(tr("IO 点表校验"))
        box.setText(summary)
        details = report.format()
        if report.issues:
            box.setInformativeText(
                tr("有问题的点必须在下载前处理（双击「Details」看逐点清单）") if report.errors
                else tr("警告不阻塞下载，但现场必须有人看过")
            )
        box.setDetailedText(details)
        box.setIcon(
            QtWidgets.QMessageBox.Critical if report.errors else
            (QtWidgets.QMessageBox.Warning if report.warnings else QtWidgets.QMessageBox.Information)
        )
        box.exec()
        self._notify(summary, 8000)
        return report

    def _selected_device(self):
        """设备列表里当前选中的设备；无选中返回 ``None``。"""
        if self.cell is None:
            return None
        r = self.device_table.currentRow()
        if r < 0 or r >= len(self.cell.devices):
            return None
        return self.cell.devices[r]

    def _add_device_row(self, name="", type_=tr("传感器")):
        if self.cell is None:
            return
        name = name or f"设备_{len(self.cell.devices) + 1}"
        self.cell.add(Device(name=name, type=type_))
        self._refresh_device_table()

    def _remove_device_row(self):
        if self.cell is None:
            return
        r = self.device_table.currentRow()
        if r < 0 or r >= len(self.cell.devices):
            return
        self.cell.remove(self.cell.devices[r].name)
        self._refresh_device_table()

    def _refresh_device_table(self):
        if self.cell is None:
            return
        self.device_table.setRowCount(len(self.cell.devices))
        for i, d in enumerate(self.cell.devices):
            for j, val in enumerate([d.name, d.type, d.brand, d.model, d.comm, d.ip]):
                self.device_table.setItem(i, j, QtWidgets.QTableWidgetItem(val))

    def _sync_devices_from_scene(self):
        self.sync_devices(self._scene)

    def sync_devices(self, scene) -> None:
        """按场景对象补齐设备清单；已存在的保留，避免覆盖用户填的内容。"""
        if self.cell is None or scene is None:
            return
        for name in list(scene.objects):
            if self.cell.find(name):
                continue
            dev_type = "机器人" if name == ROBOT_NAME else "传感器"
            self.cell.add(Device(name=name, type=dev_type))
        self._refresh_device_table()

    # ---------------- 代码生成：PLC 程序逻辑 ---------------- #

    def _build_plc_tab(self):
        w = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(w)
        layout.setContentsMargins(2, 2, 2, 2)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel(tr("目标")))
        self.plc_target = QtWidgets.QComboBox()
        self.plc_target.addItem(tr("西门子 SCL"), "siemens_scl")
        row.addWidget(self.plc_target, 1)
        self.btn_gen_plc = QtWidgets.QPushButton(tr("生成"))
        self.btn_gen_plc.clicked.connect(self._generate_plc)
        row.addWidget(self.btn_gen_plc)
        layout.addLayout(row)

        self.plc_preview = QtWidgets.QTextEdit()
        self.plc_preview.setReadOnly(True)
        self.plc_preview.setFont(QFont("Consolas", 9))
        self.plc_preview.setLineWrapMode(QtWidgets.QTextEdit.NoWrap)
        self.plc_preview.setPlaceholderText(tr("生成的 PLC 代码（西门子 SCL）将显示在这里"))
        layout.addWidget(self.plc_preview, 1)
        return w

    def _taught_points_deg(self):
        """把示教点（关节**弧度**字典）转成**度**列表，供抓取工艺模块使用。

        关节名以**示教点里实际存在的键**为准：``joint_names`` 来自
        ``renderer.joints``（含 fixed 关节，KR16 是 7 个），而运动学只认 6 个
        可动关节，照抄会让后续的 FK 直接抛错。
        """
        positions = self.recorded_positions
        if not positions:
            return []
        keys = list(positions[0].keys())
        names = [n for n in self.joint_names if n in keys] or keys
        return [
            [math.degrees(float(p.get(n, 0.0))) for n in names]
            for p in positions
        ]

    def _generate_plc(self):
        if self.cell is None or generate_scl is None:
            self.plc_preview.setPlainText(tr("单元模型不可用"))
            return
        try:
            # 优先按**真实示教出来的抓取 IO 时序**生成步序；没有示教点时
            # 回退到默认骨架，这样 PLC 与机器人程序共用同一份 IO 定义
            items = self._current_program_items()
            derived = steps_from_program(items) if items else []
            steps = derived or None
            # program=items：点表的"用途"列按真实程序填，并能报出
            # "程序在用但没映射"的点——这类点藏在界面里等于没提示
            code = generate_scl(
                self.cell,
                program_name="CellControl",
                steps=steps,
                program=items or None,
            )
        except Exception as exc:
            self.plc_preview.setPlainText(tr("生成失败：\n{exc}").format(exc=exc))
            return
        self.plc_preview.setPlainText(code)
        # 点表问题随出码一起报：SCL 注释块里写了，但状态栏也要让人当场看见
        report = self._check_io_map(silent=True)
        if report is not None and report.errors:
            self._notify(
                tr("PLC 代码已生成，但点表有 {n} 处错误——下装前必须处理").format(
                    n=len(report.errors)),
                10000,
            )

    # ---------------- 代码生成：机器人子程序 ---------------- #

    def _build_subprogram_tab(self):
        w = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(w)
        layout.setContentsMargins(2, 2, 2, 2)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel(tr("品牌")))
        self.combo_brand = QtWidgets.QComboBox()
        self.combo_brand.addItem(tr("自动（按机器人）"), None)
        try:
            from orolp.post import get_post_processor, list_post_processors

            for name in list_post_processors():
                pp = get_post_processor(name)
                self.combo_brand.addItem(
                    f"{pp.brand.upper()} {pp.controller.upper()}", name)
        except Exception:
            pass
        row.addWidget(self.combo_brand, 1)
        self.btn_gen_sub = QtWidgets.QPushButton(tr("生成"))
        self.btn_gen_sub.clicked.connect(self._generate_subprograms)
        row.addWidget(self.btn_gen_sub)
        layout.addLayout(row)

        self.sub_preview = QtWidgets.QTextEdit()
        self.sub_preview.setReadOnly(True)
        self.sub_preview.setFont(QFont("Consolas", 9))
        self.sub_preview.setLineWrapMode(QtWidgets.QTextEdit.NoWrap)
        self.sub_preview.setPlaceholderText(tr("机器人子程序代码（需先加载机器人并示教）"))
        layout.addWidget(self.sub_preview, 1)
        return w

    def set_robot(self, robot, joint_names=None) -> None:
        """注入真实机器人（含运动学），供「机器人子程序」生成代码。"""
        self._robot = robot
        if joint_names:
            self.set_joint_names(joint_names)

    def _generate_subprograms(self):
        if self._robot is None:
            self.sub_preview.setPlainText(tr("请先加载机器人"))
            return
        if not self.recorded_positions:
            self.sub_preview.setPlainText(tr("请先在「路径示教」里 Record 至少一个示教点"))
            return
        try:
            from orolp.program import ProgramBuilder

            builder = ProgramBuilder(self._robot)
            names = list(self._robot.joint_names)
            for pos in self.recorded_positions:
                joints = [math.degrees(float(pos.get(n, 0.0))) for n in names]
                builder.moveJ(joints, speed=100)
            code = builder.build().to_post(
                self.combo_brand.currentData(),
                metadata={"program_name": getattr(self._robot, "name", "program")},
            )
        except Exception as exc:
            self.sub_preview.setPlainText(tr("生成失败：\n{exc}").format(exc=exc))
            return
        self.sub_preview.setPlainText(code)

    def _on_record_toggled(self, checked):
        if checked:
            self.record_btn.setText("● Recording")
            self.record_btn.setStyleSheet("background-color: #f44336; color: white;")
            self.record_btn.setIcon(QtGui.QIcon())
        else:
            self.record_btn.setText("● Record")
            self.record_btn.setStyleSheet("background-color: #555555; color: white;")

    def _on_play_clicked(self):
        # 在播放前解析代码编辑器中的路径代码
        self._parse_code_to_positions()
        
        if len(self.recorded_positions) == 0:
            return
        
        # 通知主窗口更新 GLWidget 的路径可视化
        self.positionsParsed.emit()
            
        if self.playback_timer is None:
            self.playback_timer = QtCore.QTimer(self)
            self.playback_timer.timeout.connect(self._on_playback_tick)
        if not self.playback_timer.isActive():
            self.current_playback_index = 0
            self.playback_timer.start(self.playback_speed)
            self.playbackRequested.emit(0)

    def _on_stop_clicked(self):
        self._stop_playback()
        self.playbackStopped.emit()

    def _ensure_timer(self) -> None:
        """惰性创建播放定时器（与 Play 分支共用同一份创建逻辑）。"""
        if self.playback_timer is None:
            self.playback_timer = QtCore.QTimer(self)
            self.playback_timer.timeout.connect(self._on_playback_tick)

    def _on_pause_clicked(self):
        """暂停 / 继续。

        暂停只停定时器、**保留当前索引**，这样继续时从断点接着走，而不是
        从头重来——否则逐点核对抓取时机根本没法做。
        """
        if not self.recorded_positions:
            return
        self._ensure_timer()
        if self.playback_timer.isActive():
            self.playback_timer.stop()
        else:
            if self.current_playback_index >= len(self.recorded_positions):
                self.current_playback_index = 0
            self.playback_timer.start(self.playback_speed)
        self._sync_pause_button()

    def _on_step_clicked(self):
        """单步：只走一个示教点。"""
        if not self.recorded_positions:
            return
        self._ensure_timer()
        if self.playback_timer.isActive():
            self.playback_timer.stop()
        if self.current_playback_index >= len(self.recorded_positions):
            self.current_playback_index = 0
        self._on_playback_tick()
        self._sync_pause_button()

    def _sync_pause_button(self):
        """按定时器状态刷新暂停按钮文案。"""
        running = self.playback_timer is not None and self.playback_timer.isActive()
        self.pause_btn.setText(tr("⏸ 暂停") if running else tr("▶ 继续"))

    def _stop_playback(self):
        # 定时器对象保留（不再置 None），否则暂停后再 Stop 会拿不到它来停
        if self.playback_timer is not None:
            self.playback_timer.stop()
        self.current_playback_index = 0
        self._sync_pause_button()

    def _on_playback_tick(self):
        if self.current_playback_index < len(self.recorded_positions):
            self.playbackRequested.emit(self.current_playback_index)
            self.current_playback_index += 1
        else:
            self._stop_playback()
            self.playbackStopped.emit()

    def _on_clear_clicked(self):
        self._stop_playback()
        self.recorded_positions.clear()
        self.point_types.clear()
        self.pos_count_label.setText("Positions: 0")
        self.code_edit.clear()
        self.code_edit.append("# OLP Path Code\n")
        self.clearRequested.emit()
        self._suggest_next_point_type()

    def record_position(self, joint_values):
        if self.record_btn.isChecked():
            self.recorded_positions.append(dict(joint_values))
            if _HAS_PICKPLACE:
                self.point_types.append(self.point_type_combo.currentData())
            self.pos_count_label.setText(f"Positions: {len(self.recorded_positions)}")
            self.positionRecorded.emit(len(self.recorded_positions))
            self._update_code_with_positions()
            self._suggest_next_point_type()

    def _suggest_next_point_type(self) -> None:
        """把点位下拉预置成"下一个"该用的语义，用户可随时改。

        例如已有 1 个点时，下一次默认建议「抓取点」——正好是五点式循环的顺序。
        """
        if not _HAS_PICKPLACE:
            return
        nxt = default_types_for_count(len(self.recorded_positions) + 1)
        if len(nxt) > len(self.recorded_positions):
            idx = self.point_type_combo.findData(nxt[len(self.recorded_positions)])
            if idx >= 0:
                self.point_type_combo.setCurrentIndex(idx)

    def sync_point_types(self) -> None:
        """把 point_types 补齐到与 recorded_positions 等长。

        MainWindow 会直接改写 ``recorded_positions``（视口 Record 的路径），
        此时语义列表要跟着同步，否则两边长度不一致会直接报错。
        """
        if not _HAS_PICKPLACE:
            return
        n = len(self.recorded_positions)
        if len(self.point_types) > n:
            self.point_types = self.point_types[:n]
        while len(self.point_types) < n:
            self.point_types.append(
                default_types_for_count(n)[len(self.point_types)]
            )

    def get_point_types(self):
        return self.point_types

    def set_scene(self, scene) -> None:
        """注入工作单元场景，用于枚举可抓取的工件。"""
        self._scene = scene
        self.refresh_workpiece_combo()

    def set_cell(self, cell) -> None:
        """整体换掉工作单元模型（工程文件载入用 v1.3.4）。

        只换数据 + 刷设备表：点表是**用户填过的内容**，工程载入必须原样恢复，
        所以这里不做"按场景补齐设备"那套推导（那是 :meth:`sync_devices` 的活）。
        """
        if cell is None or not _HAS_CELL:
            return
        self.cell = cell
        self._refresh_device_table()

    def refresh_workpiece_combo(self) -> None:
        """重建「工件」下拉：场景里除机器人外的全部对象。"""
        if not _HAS_PICKPLACE:
            return
        current = self.workpiece_combo.currentData()
        self.workpiece_combo.blockSignals(True)
        self.workpiece_combo.clear()
        self.workpiece_combo.addItem(tr("（无）"), None)
        if self._scene is not None:
            for name in self._scene.objects:
                if name == ROBOT_NAME:
                    continue
                self.workpiece_combo.addItem(name, name)
        idx = self.workpiece_combo.findData(current)
        self.workpiece_combo.setCurrentIndex(idx if idx >= 0 else 0)
        self.workpiece_combo.blockSignals(False)

    def get_workpiece(self):
        """当前选中的工件名；未选返回 None。"""
        if not _HAS_PICKPLACE:
            return None
        return self.workpiece_combo.currentData()

    def _update_code_with_positions(self):
        self.code_edit.clear()
        self.code_edit.append("# OLP Path Code")
        self.code_edit.append(f"# Recorded positions: {len(self.recorded_positions)}")
        self.code_edit.append("")
        for i, pos in enumerate(self.recorded_positions):
            # 将弧度转换为角度显示
            joints_str = ", ".join([f"{math.degrees(v):.2f}" for v in pos.values()])
            self.code_edit.append(f"movej([{joints_str}], vel=0.5)")

    def get_recorded_positions(self):
        return self.recorded_positions

    def set_recorded_positions(self, positions):
        self.recorded_positions = list(positions)
        self.pos_count_label.setText(f"Positions: {len(self.recorded_positions)}")
        self._update_code_with_positions()
        self.update()

    def toPlainText(self):
        return self.code_edit.toPlainText()

    def setPlainText(self, text):
        self.code_edit.setPlainText(text)

    def set_joint_names(self, names):
        """设置关节名称顺序，用于解析代码"""
        self.joint_names = list(names)

    def _parse_code_to_positions(self):
        """从代码编辑器中解析路径位置数据"""
        code_text = self.code_edit.toPlainText()
        # 语义不在代码文本里，解析会重建位置列表——先存下来，数量一致就沿用它，
        # 否则用户打的「抓取点 / 放置点」标记会在每次 Play 时被抹掉
        old_types = list(self.point_types)
        self.recorded_positions.clear()
        
        # 匹配 movej([value1, value2, ...], vel=X.X) 格式
        pattern = r'movej\(\s*\[([^\]]+)\]\s*(?:,\s*vel\s*=\s*[\d.]+)?\s*\)'
        matches = re.findall(pattern, code_text)
        
        for match in matches:
            # 解析数值列表（角度）
            try:
                values = [float(v.strip()) for v in match.split(',')]
                # 将角度转换为弧度
                rad_values = [math.radians(v) for v in values]
                
                if self.joint_names and len(values) == len(self.joint_names):
                    # 如果有关节名称，创建字典
                    pos_dict = {}
                    for i, name in enumerate(self.joint_names):
                        pos_dict[name] = rad_values[i]
                    self.recorded_positions.append(pos_dict)
                elif len(values) > 0:
                    # 如果没有关节名称，使用索引作为键
                    pos_dict = {f"joint_{i+1}": rad_values[i] for i in range(len(rad_values))}
                    self.recorded_positions.append(pos_dict)
            except ValueError:
                continue
        
        # 更新位置计数显示
        self.pos_count_label.setText(f"Positions: {len(self.recorded_positions)}")

        if len(old_types) == len(self.recorded_positions):
            self.point_types = old_types
        else:
            self.point_types = []
            self.sync_point_types()


class DevicePropertyPanel(QtWidgets.QWidget):
    objectChanged = Signal()  # 场景对象属性被编辑后触发视口重绘
    jointChanged = Signal(str, float)  # 关节滑块改动 → 驱动视口/记录

    def __init__(self, parent=None):
        super().__init__(parent)
        self.log_text = None
        self.name_label = None
        self.type_label = None
        self.manufacturer_label = None
        self.dof_label = None
        #: 当前场景引用（由 MainWindow 注入）；选中场景对象时用于读写属性
        self.scene = None
        self.current_object = None
        self._has_robot_model = False
        self.obj_pose_widgets = {}
        self.obj_param_widgets = {}
        self.sliders = {}
        self.value_labels = {}
        #: 设备中可动轴（关节）的滑块与数值标签
        self.axis_sliders = {}
        self.axis_value_labels = {}
        self._renderer = None
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # ---- 对象属性（选中场景对象时显示，默认隐藏）----
        # 含：信息（对象名/类型 + 机器人规格）、位置（基座位姿）、参数
        self.object_group = QtWidgets.QGroupBox(tr("对象属性"))
        obj_layout = QtWidgets.QVBoxLayout(self.object_group)

        # 信息（对象名/类型 + 机器人规格）
        self.info_group = QtWidgets.QGroupBox(tr("信息"))
        info_layout = QtWidgets.QFormLayout(self.info_group)
        self.obj_name_label = QtWidgets.QLabel("N/A")
        self.obj_type_label = QtWidgets.QLabel("N/A")
        info_layout.addRow(tr("名称:"), self.obj_name_label)
        info_layout.addRow(tr("类型:"), self.obj_type_label)

        # 挂载（把对象挂到机器人法兰或另一对象下；机器人自身不可挂载）
        self.obj_mount_widget = QtWidgets.QWidget()
        mount_layout = QtWidgets.QFormLayout(self.obj_mount_widget)
        mount_layout.setContentsMargins(0, 4, 0, 4)
        self.obj_parent_combo = QtWidgets.QComboBox()
        self.obj_parent_combo.setToolTip(
            tr(
                "挂到「机器人」＝ 安装到法兰，工具将跟随机器人运动；\n"
                "挂到其它对象 ＝ 保持当前位置，仅建立随动关系。"
            )
        )
        self.obj_parent_combo.currentIndexChanged.connect(self._on_obj_parent_edited)
        mount_layout.addRow(tr("挂载到:"), self.obj_parent_combo)

        # 装夹 / 解除装夹：与下拉等价，但不用去翻选项——法兰装夹是高频操作
        self.mount_btn_row = QtWidgets.QWidget()
        btn_layout = QtWidgets.QHBoxLayout(self.mount_btn_row)
        btn_layout.setContentsMargins(0, 2, 0, 0)
        self.btn_mount_to_robot = QtWidgets.QPushButton(tr("装夹到末端"))
        self.btn_unmount = QtWidgets.QPushButton(tr("解除装夹"))
        self.btn_mount_to_robot.setToolTip(
            tr("吸附到机器人法兰原点，工具坐标系与法兰对齐并随机器人运动")
        )
        self.btn_unmount.setToolTip(tr("解除挂载并保持当前世界位姿"))
        self.btn_mount_to_robot.clicked.connect(self._on_mount_to_robot)
        self.btn_unmount.clicked.connect(self._on_unmount)
        btn_layout.addWidget(self.btn_mount_to_robot)
        btn_layout.addWidget(self.btn_unmount)
        mount_layout.addRow(self.mount_btn_row)
        info_layout.addRow(self.obj_mount_widget)

        # 机器人规格（仅机器人对象可见）
        self.robot_spec_widget = QtWidgets.QWidget()
        spec_layout = QtWidgets.QFormLayout(self.robot_spec_widget)
        spec_layout.setContentsMargins(0, 4, 0, 4)
        self.name_label = QtWidgets.QLabel("N/A")
        self.manufacturer_label = QtWidgets.QLabel("N/A")
        self.type_label = QtWidgets.QLabel("N/A")
        self.payload_label = QtWidgets.QLabel("N/A")
        self.reach_label = QtWidgets.QLabel("N/A")
        self.repeat_label = QtWidgets.QLabel("N/A")
        self.dof_label = QtWidgets.QLabel("N/A")
        self.controller_label = QtWidgets.QLabel("N/A")
        spec_layout.addRow(tr("型号:"), self.name_label)
        spec_layout.addRow(tr("厂商:"), self.manufacturer_label)
        spec_layout.addRow(tr("类型:"), self.type_label)
        spec_layout.addRow(tr("额定负载:"), self.payload_label)
        spec_layout.addRow(tr("最大臂展:"), self.reach_label)
        spec_layout.addRow(tr("重复定位精度:"), self.repeat_label)
        spec_layout.addRow(tr("轴数:"), self.dof_label)
        spec_layout.addRow(tr("控制器:"), self.controller_label)
        info_layout.addRow(self.robot_spec_widget)
        obj_layout.addWidget(self.info_group)

        # 型号备注（数据来源、参数核验状态等）
        self.note_label = QtWidgets.QLabel("")
        self.note_label.setWordWrap(True)
        self.note_label.setStyleSheet("color: #8a6d3b; padding: 2px;")
        note_font = QFont()
        note_font.setPointSize(8)
        self.note_label.setFont(note_font)
        self.note_label.hide()
        obj_layout.addWidget(self.note_label)

        # 位置（虚拟环境坐标，可编辑）—— 6DOF：XYZ + 绕XYZ
        self.pose_grp = QtWidgets.QGroupBox(tr("位置"))
        pose_layout = QtWidgets.QFormLayout(self.pose_grp)
        for key, label, lo, hi, step in (
            ("x", "X (mm)", -20000, 20000, 10),
            ("y", "Y (mm)", -20000, 20000, 10),
            ("z", "Z (mm)", -5000, 10000, 10),
            ("rx", tr("绕 X (°)"), -180, 180, 5),
            ("ry", tr("绕 Y (°)"), -180, 180, 5),
            ("rz", tr("绕 Z (°)"), -360, 360, 5),
        ):
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(lo, hi)
            spin.setDecimals(1)
            spin.setSingleStep(step)
            spin.setValue(0.0)
            spin.valueChanged.connect(self._on_obj_pose_edited)
            self.obj_pose_widgets[key] = spin
            pose_layout.addRow(label, spin)
        obj_layout.addWidget(self.pose_grp)

        # 参数（设备模型参数化：只放尺寸参数，轴不在这里）
        self.obj_params_group = QtWidgets.QGroupBox(tr("参数"))
        self.obj_params_layout = QtWidgets.QVBoxLayout(self.obj_params_group)
        self.obj_params_layout.setContentsMargins(4, 4, 4, 4)
        self.obj_params_layout.setSpacing(4)
        obj_layout.addWidget(self.obj_params_group)

        # 关节（设备关节姿态调节：机器人 6 轴 / 多轴设备的轴）
        self.obj_joint_group = QtWidgets.QGroupBox(tr("关节"))
        self.obj_joint_layout = QtWidgets.QVBoxLayout(self.obj_joint_group)
        self.obj_joint_layout.setContentsMargins(4, 4, 4, 4)
        self.obj_joint_layout.setSpacing(4)
        obj_layout.addWidget(self.obj_joint_group)

        # 让上方各分组保持自然高度。滚动区会把多余高度分给 QGroupBox，
        # 导致「信息」这类内容少的分组被拉高、出现大片空白。
        # （注意与外层的区别：外层 layout 加 stretch 会抢滚动区高度，不能加。）
        obj_layout.addStretch(1)

        self.object_scroll = QtWidgets.QScrollArea()
        self.object_scroll.setWidgetResizable(True)
        self.object_scroll.setFrameStyle(QtWidgets.QFrame.NoFrame)
        self.object_scroll.setWidget(self.object_group)
        self.object_scroll.hide()
        # 这里不能加 addStretch：它会与 object_scroll 平分剩余高度（各占一半），
        # 把滚动可视区压到只剩约 350px，导致底部的「参数/关节控制」被挤到
        # 视口以下完全看不见。让 object_scroll 独占全部剩余空间即可把日志顶到底部。
        layout.addWidget(self.object_scroll, 1)

        log_group = QtWidgets.QGroupBox(tr("日志"))
        log_group.setMaximumHeight(150)  # 限制日志高度，给上方属性区留出可视空间
        log_layout = QtWidgets.QVBoxLayout(log_group)
        self.log_text = QtWidgets.QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 9))
        self.log_text.setStyleSheet("background-color: #1a1a1a; color: #ffffff;")
        log_layout.addWidget(self.log_text)
        layout.addWidget(log_group)
    
    def update_robot_info(self, renderer, entry=None):
        """更新机器人信息区。

        Args:
            renderer: 已加载 URDF 的渲染器
            entry: 机器人库条目；提供时可回显负载、臂展等完整规格
        """
        self._renderer = renderer
        self._has_robot_model = True
        revolute = [j for j in renderer.joints.values() if j["type"] == "revolute"]
        self.dof_label.setText(str(len(revolute)))

        if entry is not None:
            self.name_label.setText(entry.model_name)
            self.manufacturer_label.setText(entry.brand_display)
            type_map = {"industrial": tr("工业臂"), "cobot": tr("协作臂"), "scara": "SCARA"}
            self.type_label.setText(type_map.get(entry.robot_type, entry.robot_type))
            self.payload_label.setText(
                f"{entry.payload_kg:g} kg" if entry.payload_kg is not None else "N/A"
            )
            self.reach_label.setText(
                f"{entry.reach_mm:g} mm" if entry.reach_mm is not None else "N/A"
            )
            self.repeat_label.setText(
                f"±{entry.repeatability_mm:g} mm"
                if entry.repeatability_mm is not None
                else "N/A"
            )
            self.controller_label.setText(entry.controller or "N/A")
            if entry.note:
                prefix = "" if entry.verified else tr("⚠ 参数未核验 — ")
                self.note_label.setText(prefix + entry.note)
                self.note_label.show()
            else:
                self.note_label.hide()
        else:
            self.name_label.setText(renderer.robot_name)
            self.manufacturer_label.setText("N/A")
            self.type_label.setText("N/A")
            self.payload_label.setText("N/A")
            self.reach_label.setText("N/A")
            self.repeat_label.setText("N/A")
            self.controller_label.setText("N/A")
            self.note_label.hide()

        self.log_text.append(tr("已载入: {name}").format(name=renderer.robot_name))
        self.log_text.append(tr("关节数: {n}").format(n=len(revolute)))

    # --------------------------- 对象属性动态编辑 --------------------------- #

    def set_scene(self, scene) -> None:
        """注入当前工作单元场景，供选中对象时读写属性。"""
        self.scene = scene

    def set_object(self, name: Optional[str]) -> None:
        """按选中对象动态切换右面板内容：信息 / 位置 / 参数 三块。

        Args:
            name: 选中对象名；``None`` / 不在场景中 → 回到默认视图（仅信息）。
        """
        if not _HAS_WORLD or self.scene is None or not name or name not in self.scene.objects:
            # 默认视图：未选中对象 → 隐藏「对象属性」
            self.current_object = None
            self.object_scroll.hide()
            return

        self.current_object = name
        obj = self.scene.objects[name]
        is_robot = name == ROBOT_NAME
        spec = COMPONENTS.get(obj.component_id) if obj.component_id else None

        # 显示「对象属性」容器（内含信息 / 位置 / 参数）
        self.object_scroll.show()
        self.object_group.show()

        self.obj_name_label.setText(name)
        if is_robot:
            self.obj_type_label.setText(tr("机器人"))
            self.robot_spec_widget.setVisible(True)
            self.note_label.setVisible(bool(self.note_label.text()))
            # 机器人可以挂到底座/台面上（基座随之随动）。但「装夹到末端」对
            # 机器人自身没有意义——那是"把工具装到它的法兰上"，故只留下拉。
            self.obj_mount_widget.setVisible(True)
            self.mount_btn_row.setVisible(False)
            self._refresh_parent_combo(name)
        else:
            self.obj_type_label.setText(spec.name if spec else tr("自定义"))
            self.robot_spec_widget.setVisible(False)
            self.note_label.setVisible(False)
            self.obj_mount_widget.setVisible(True)
            self.mount_btn_row.setVisible(True)
            self._refresh_parent_combo(name)

        # 位姿回填（blockSignals 避免触发写回）
        # _matrix_to_rpy 返回弧度，转成角度后再填进 6 个输入框
        tf = obj.pose
        xyz = tf.matrix[:3, 3]
        rx, ry, rz = (math.degrees(v) for v in _matrix_to_rpy(tf.matrix))
        for key, val in (("x", float(xyz[0])), ("y", float(xyz[1])),
                         ("z", float(xyz[2])),
                         ("rx", rx), ("ry", ry), ("rz", rz)):
            w = self.obj_pose_widgets[key]
            w.blockSignals(True)
            w.setValue(val)
            w.blockSignals(False)

        # 参数（模型参数化）与关节（姿态调节）各归其位：
        # - 机器人：无尺寸参数，关节面板给 6 轴滑块
        # - 组件：参数面板给尺寸参数（已排除轴），关节面板给它的可动轴
        if is_robot and self._renderer is not None:
            self._rebuild_param_rows(None, {})
            self._setup_joint_controls(self._renderer)
        else:
            self._rebuild_param_rows(spec, obj.params)
            self._rebuild_axis_rows(spec, obj)

    def _clear_layout(self, layout) -> None:
        """清空任意布局内的所有控件与嵌套布局（「参数」与「关节」共用）。"""
        while layout.count():
            item = layout.takeAt(0)
            if item is None:
                continue
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
            else:
                sub = item.layout()
                if sub is not None:
                    self._clear_layout(sub)
                    sub.deleteLater()

    def _clear_params_layout(self) -> None:
        """清空参数面板。"""
        self._clear_layout(self.obj_params_layout)

    def _rebuild_param_rows(self, spec, params: dict) -> None:
        """根据组件 ParamSpec 重建参数编辑行。"""
        self._clear_params_layout()
        self.obj_param_widgets.clear()
        self.sliders.clear()
        self.value_labels.clear()

        if spec is None:
            lbl = QtWidgets.QLabel(tr("（无可调参数）"))
            lbl.setStyleSheet("color:#7f8192; font-size:11px;")
            self.obj_params_layout.addWidget(lbl)
            return

        # 用 plain_params()：轴由「关节」面板负责，这里不再重复出现
        for p in spec.plain_params():
            row = QtWidgets.QHBoxLayout()
            row.setSpacing(8)
            lbl = QtWidgets.QLabel(p.label)
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(p.minimum, p.maximum)
            spin.setDecimals(1)
            spin.setSingleStep(p.step)
            if p.unit:
                spin.setSuffix(f" {p.unit}")
            spin.blockSignals(True)
            spin.setValue(float(params.get(p.name, p.default)))
            spin.blockSignals(False)
            spin.valueChanged.connect(
                lambda v, n=p.name: self._on_obj_param_edited(n, v)
            )
            self.obj_param_widgets[p.name] = spin
            row.addWidget(lbl)
            row.addWidget(spin, 1)
            self.obj_params_layout.addLayout(row)

    def _setup_joint_controls(self, renderer) -> None:
        """为机器人在「关节」面板内构建 6 轴滑块。"""
        self._renderer = renderer
        self._clear_layout(self.obj_joint_layout)
        self.axis_sliders.clear()
        self.axis_value_labels.clear()
        self.sliders.clear()
        self.value_labels.clear()

        # 按运动学顺序取可动关节：不依赖 link 名匹配，顺序也与 FK/IK 一致
        order = list(renderer.actuated_names()) or list(renderer.joints.keys())
        joint_items = [(i, n, renderer.joints[n]) for i, n in enumerate(order)
                       if n in renderer.joints]

        for _, name, joint in joint_items:
            lo = math.degrees(joint["limit"]["lower"])
            hi = math.degrees(joint["limit"]["upper"])

            # 紧凑单行「名称 | 滑块 | 当前值」：
            # 原来是「名称/值 + 滑块 + 上下限」三行约 72px，6 个共 472px，
            # 会把「参数」整块顶出可视区；压成一行约 28px，6 个共约 180px。
            row = QtWidgets.QHBoxLayout()
            row.setSpacing(6)
            row.setContentsMargins(2, 1, 2, 1)

            label = QtWidgets.QLabel(name)
            label.setFixedWidth(58)
            label.setStyleSheet("font-size:11px;")

            slider = QtWidgets.QSlider(Qt.Horizontal)
            slider.setMinimum(int(lo * 100))
            slider.setMaximum(int(hi * 100))
            init_deg = math.degrees(renderer.joint_values.get(name, 0.0))
            slider.setValue(int(init_deg * 100))
            slider.setFixedHeight(18)
            slider.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            slider.setToolTip(tr("范围 {lo}° ~ {hi}°").format(lo=f"{lo:.1f}", hi=f"{hi:.1f}"))
            slider.valueChanged.connect(lambda v, n=name: self._on_joint_slider_changed(n, v))
            self.sliders[name] = slider

            val = QtWidgets.QLabel(f"{init_deg:.1f}°")
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val.setFixedWidth(48)
            val.setStyleSheet("font-size:11px;")
            self.value_labels[name] = val

            row.addWidget(label)
            row.addWidget(slider, 1)
            row.addWidget(val)
            self.obj_joint_layout.addLayout(row)

        if not joint_items:
            lbl = QtWidgets.QLabel(tr("（该设备无关节）"))
            lbl.setStyleSheet("color:#7f8192; font-size:11px;")
            self.obj_joint_layout.addWidget(lbl)

    def _rebuild_axis_rows(self, spec, obj) -> None:
        """为多轴设备在「关节」面板构建轴滑块。

        轴值写回 ``obj.params``，由既有的参数驱动几何通路重建几何，
        因此不需要任何渲染侧改动。
        """
        self.sliders.clear()
        self.value_labels.clear()
        self.axis_sliders.clear()
        self.axis_value_labels.clear()
        self._clear_layout(self.obj_joint_layout)

        axes = tuple(getattr(spec, "axes", ()) or ()) if spec is not None else ()
        if not axes:
            lbl = QtWidgets.QLabel(tr("（该设备无关节）"))
            lbl.setStyleSheet("color:#7f8192; font-size:11px;")
            self.obj_joint_layout.addWidget(lbl)
            return

        for axis in axes:
            row = QtWidgets.QHBoxLayout()
            row.setSpacing(6)
            row.setContentsMargins(2, 1, 2, 1)

            label = QtWidgets.QLabel(axis.label)
            label.setFixedWidth(58)
            label.setStyleSheet("font-size:11px;")

            init = float(obj.params.get(axis.param, 0.0))
            slider = QtWidgets.QSlider(Qt.Horizontal)
            slider.setMinimum(int(axis.minimum * 100))
            slider.setMaximum(int(axis.maximum * 100))
            slider.setValue(int(init * 100))
            slider.setFixedHeight(18)
            slider.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            slider.setToolTip(tr("范围 {lo} ~ {hi} {unit}").format(lo=f"{axis.minimum:g}", hi=f"{axis.maximum:g}", unit=axis.unit))
            slider.valueChanged.connect(
                lambda v, a=axis: self._on_axis_slider_changed(a, v))
            self.axis_sliders[axis.name] = slider

            val = QtWidgets.QLabel(f"{init:g}{axis.unit}")
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val.setFixedWidth(48)
            val.setStyleSheet("font-size:11px;")
            self.axis_value_labels[axis.name] = val

            row.addWidget(label)
            row.addWidget(slider, 1)
            row.addWidget(val)
            self.obj_joint_layout.addLayout(row)

    def _on_axis_slider_changed(self, axis, value) -> None:
        """轴值变化：写回 obj.params，触发几何重建。"""
        v = value / 100.0
        if axis.name in self.axis_value_labels:
            self.axis_value_labels[axis.name].setText(f"{v:g}{axis.unit}")
        if not self.current_object or self.scene is None:
            return
        obj = self.scene.objects.get(self.current_object)
        if obj is None:
            return
        obj.params[axis.param] = float(v)
        self.objectChanged.emit()

    def _on_joint_slider_changed(self, name, value):
        angle = value / 100.0
        if name in self.value_labels:
            self.value_labels[name].setText(f"{angle:.1f}°")
        self.jointChanged.emit(name, math.radians(angle))

    def update_joint_displays(self, joint_values):
        for name, value in joint_values.items():
            if name in self.value_labels:
                self.value_labels[name].setText(f"{math.degrees(value):.1f}°")
            if name in self.sliders:
                self.sliders[name].blockSignals(True)
                self.sliders[name].setValue(int(math.degrees(value) * 100))
                self.sliders[name].blockSignals(False)

    def _on_obj_pose_edited(self) -> None:
        if not self.current_object or self.scene is None:
            return
        obj = self.scene.objects.get(self.current_object)
        if obj is None:
            return
        obj.pose = Pose.from_xyz_rpy(
            self.obj_pose_widgets["x"].value(),
            self.obj_pose_widgets["y"].value(),
            self.obj_pose_widgets["z"].value(),
            self.obj_pose_widgets["rx"].value(),
            self.obj_pose_widgets["ry"].value(),
            self.obj_pose_widgets["rz"].value(),
        )
        self.objectChanged.emit()

    def _descendant_names(self, name: str) -> set:
        """对象自身及其全部后代名——挂载候选里必须排除，否则会成环。"""
        out = {name}
        changed = True
        while changed:
            changed = False
            for o in self.scene.objects.values():
                if o.parent in out and o.name not in out:
                    out.add(o.name)
                    changed = True
        return out

    def _refresh_parent_combo(self, name: str) -> None:
        """重建「挂载到」下拉：场景根 + 所有不会形成环的候选父对象。"""
        combo = self.obj_parent_combo
        combo.blockSignals(True)
        combo.clear()
        combo.addItem(tr("(场景根)"), None)
        excluded = self._descendant_names(name)
        for o in self.scene.objects.values():
            if o.name in excluded:
                continue
            combo.addItem(o.name, o.name)
        cur = self.scene.objects[name].parent
        idx = combo.findData(cur)
        combo.setCurrentIndex(idx if idx >= 0 else 0)
        combo.blockSignals(False)

    def _on_mount_to_robot(self) -> None:
        """把当前对象装夹到机器人末端（主机器人法兰，坐标系自动对齐）。"""
        name = self.current_object
        if not name or self.scene is None or not getattr(self.scene, "robots", None):
            return
        robot = self.scene.robot_names[0]
        if name == robot:
            return
        try:
            self.scene.attach(name, robot, keep_world=False)
        except (KeyError, ValueError) as exc:
            print(f"[warning] 装夹失败: {exc}")
            return
        self.set_object(name)
        self.objectChanged.emit()

    def _on_unmount(self) -> None:
        """解除挂载并保持当前世界位姿（停在原地，不再随机器人动）。"""
        name = self.current_object
        if not name or self.scene is None:
            return
        self.scene.detach(name)
        self.set_object(name)
        self.objectChanged.emit()

    def _on_obj_parent_edited(self) -> None:
        """切换挂载父对象。

        挂到机器人＝**工具安装**语义（吸附到法兰原点）；挂到其它对象＝
        **放到其上**语义（保持世界位置，只建立随动关系）。
        """
        name = self.current_object
        if not name or self.scene is None:
            return
        target = self.obj_parent_combo.currentData()
        try:
            self.scene.attach(name, target, keep_world=(target != ROBOT_NAME))
            # 机器人挂到底座/台面：要"坐上去"（上表面正中央），而不是停在
            # 原地只建立随动关系——后者看着像根本没挂上
            if (self.scene.is_robot(name) and target
                    and not self.scene.is_robot(target)):
                _placement.mount_on_top_of(self.scene, name, target)
        except (KeyError, ValueError) as exc:
            self._refresh_parent_combo(name)
            print(f"[warning] 挂载失败: {exc}")
            return
        # 局部位姿已被 attach 改写，整块回填一遍（位置分组也要跟着变）
        self.set_object(name)
        self.objectChanged.emit()

    def _on_obj_param_edited(self, name: str, value: float) -> None:
        if not self.current_object or self.scene is None:
            return
        obj = self.scene.objects.get(self.current_object)
        if obj is None:
            return
        obj.params[name] = float(value)
        self.objectChanged.emit()


class MainWindow(QtWidgets.QMainWindow):
    #: 自动保存间隔（毫秒）。有未保存改动时每 60s 落一份快照——崩溃最多
    #: 丢一个间隔内的活，且不影响手动保存（快照写到独立目录，不碰工程）。
    AUTOSAVE_INTERVAL_MS = 60_000

    def __init__(self, urdf_path=None):
        super().__init__()
        #: 当前工程（v1.3.4）：`.orolp` 工程文件的内存形态，窗口标题显示它
        self.project = Project() if _HAS_PROJECT else None
        #: 有未保存改动（标题里的"•未保存"）
        self._project_dirty = False
        self.setWindowTitle(f"RobotWorld Terminal v{_OROLP_VERSION}")
        self.resize(1400, 900)
        self._set_window_icon()
        self.setup_menu()
        self.setup_ui()
        self.urdf_path = urdf_path
        #: 供离线编程面板使用的 :class:`orolp.Robot`（含运动学）
        self.olp_robot = None
        #: 「真机连接」模式对话框（v1.1.1）：首次打开时创建，之后复用
        self._controller_dialog = None
        self._controller_panel = None
        # 会话支撑（v1.3.5）：自动保存定时器 + 恢复询问的一次性闸门
        self._recovery_prompted = False
        self._autosave_timer = None
        if _HAS_SESSION_SUPPORT:
            self._autosave_timer = QtCore.QTimer(self)
            self._autosave_timer.setInterval(self.AUTOSAVE_INTERVAL_MS)
            self._autosave_timer.timeout.connect(self._autosave_tick)
            self._autosave_timer.start()
        self._update_window_title()
    
    def showEvent(self, event):
        super().showEvent(event)
        if self.urdf_path and Path(self.urdf_path).exists():
            self.load_robot(self.urdf_path)
        elif _HAS_ROBOT_LIBRARY and hasattr(self, "library_panel"):
            self._load_default_robot()
        self.gl_widget.update()
        self.maybe_prompt_recovery()

    def closeEvent(self, event):
        super().closeEvent(event)
        # 干净退出（没有未保存改动）→ 自动保存快照已无意义，清掉，
        # 免得下次启动被一个过时的"崩溃恢复"误导；有未保存改动则留着——
        # 正常关闭路径之外（被杀/崩溃）这正是要恢复的东西。
        if _HAS_SESSION_SUPPORT and not self._project_dirty:
            clear_autosave()

    def _load_default_robot(self):
        """未通过命令行指定模型时，载入一个示例机器人，避免首屏空白。"""
        for key in DEFAULT_ROBOT_KEYS:
            entry = self.library_panel.catalog.get(key)
            if entry and entry.urdf_path and entry.urdf_path.exists():
                self.load_robot(str(entry.urdf_path))
                self.library_panel.select_key(key)
                return

    def show_robot_library(self):
        """切换到机器人库标签页。"""
        # 资源库常驻左侧，无需切换标签页
        if hasattr(self, "library_panel"):
            self.library_panel.setFocus()

    def reload_robot_library(self):
        """重新扫描 models/ 目录。"""
        if hasattr(self, "library_panel"):
            self.library_panel.refresh()

    def _add_component_from_library(self, component_id: str) -> None:
        """机器组件库双击/拖拽组件时，加入场景并切到场景标签。"""
        if not (_HAS_WORLD and hasattr(self, "scene_panel")):
            return
        self.scene_panel.add_component(component_id)
        if hasattr(self, "left_tabs"):
            self.left_tabs.setCurrentWidget(self.scene_panel)

    def _on_robot_dropped(self, key: str) -> None:
        """机器组件库拖拽机器人型号到视口时，按目录 key 解析 URDF 并载入。"""
        if not (_HAS_ROBOT_LIBRARY and hasattr(self, "library_panel")):
            return
        entry = self.library_panel.catalog.get(key)
        if entry and entry.urdf_path and entry.urdf_path.exists():
            self.load_robot(str(entry.urdf_path))
        else:
            self.statusBar().showMessage(tr("该型号 URDF 不可用"))
    
    def _setup_status_bar(self) -> None:
        """状态栏**常驻**区：节拍 / 干涉 / IO / 报错。

        必须用 ``addPermanentWidget`` 而不是 ``showMessage``：后者是临时消息，
        会被下一条直接冲掉，演示时根本看不到持续状态（而节拍、干涉这类
        指标恰恰是要一直盯着的）。
        """
        self.status_cycle = QtWidgets.QLabel(tr("节拍 —"))
        self.status_collision = QtWidgets.QLabel(tr("干涉 —"))
        self.status_io = QtWidgets.QLabel("IO —")
        self.status_error = QtWidgets.QLabel(tr("无报错"))

        for w in (self.status_cycle, self.status_collision,
                  self.status_io, self.status_error):
            w.setStyleSheet(
                "color:#c8cad4; padding:0 8px; font-size:11px;"
            )
            self.statusBar().addPermanentWidget(w)

    def _on_report_ready(self, report) -> None:
        """校验报告出来后刷新节拍与报错。"""
        cycle = getattr(report, "cycle_time", None)
        self.status_cycle.setText(
            tr("节拍 {t} s").format(t=f"{cycle:.2f}") if cycle is not None else tr("节拍 —")
        )
        issues = list(getattr(report, "issues", None) or [])
        if issues:
            self.status_error.setText(tr("报错 {n}").format(n=len(issues)))
            self.status_error.setStyleSheet(
                "color:#ff8a80; padding:0 8px; font-size:11px;"
            )
        else:
            self.status_error.setText(tr("无报错"))
            self.status_error.setStyleSheet(
                "color:#a5d6a7; padding:0 8px; font-size:11px;"
            )
        # 校验产出新轨迹 → 把新上下文交给渲染插件（v1.0.4）
        self._refresh_plugin_overlays()

    # ------------------------------------------------------------------ #
    # 插件（v1.0.4）
    # ------------------------------------------------------------------ #

    def _refresh_plugin_overlays(self, current_index: int = -1) -> None:
        """把当前轨迹交给渲染插件并重绘覆盖层。

        刻意**不**放在每帧回调里：``collect_overlays`` 每次都会构造插件对象，
        按帧调用等于每 16ms 构造一次第三方代码——插件里稍微有点初始化开销就会
        拖垮帧率。轨迹变化的时机（校验完成 / 加载演示 / 手动刷新）才是正确的
        刷新点。
        """
        if not _HAS_PLUGINS:
            return
        traj = None
        if _HAS_PROGRAM and hasattr(self, "program_panel"):
            traj = getattr(self.program_panel, "trajectory", None)
        tolerance = 1.0
        try:
            if _HAS_WORLD and hasattr(self, "scene_panel"):
                from orolp.planner import ENVELOPE_TOLERANCE_MM

                tolerance = float(ENVELOPE_TOLERANCE_MM)
        except Exception:
            pass
        robot_name = getattr(getattr(self, "olp_robot", None), "name", "") or ""
        try:
            ctx = OverlayContext.from_trajectory(
                traj,
                current_index=current_index,
                tolerance_mm=tolerance,
                robot_name=robot_name,
                extra={"has_scene": bool(self.gl_widget.scene is not None)},
            )
        except Exception as exc:
            print(f"[warning] overlay context failed: {exc}")
            return
        self.gl_widget.set_overlay_context(ctx)
        self.gl_widget.refresh_plugin_overlays()

    def toggle_plugin_overlays(self, checked: bool) -> None:
        """开关插件覆盖层显示。"""
        self.gl_widget.show_plugin_overlays = bool(checked)
        if checked:
            self._refresh_plugin_overlays()
        self.gl_widget.update()

    def reload_plugins(self) -> None:
        """重新扫描已安装包的插件入口点（装了新插件不必重启）。"""
        if not _HAS_PLUGINS:
            return
        try:
            stats = load_entry_points()
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self, tr("插件"), tr("重新扫描失败：{exc}").format(exc=exc))
            return
        self._refresh_plugin_overlays()
        QtWidgets.QMessageBox.information(
            self,
            tr("插件"),
            tr(
                "重新扫描完成：发现 {discovered} 个入口点，登记 {registered} 个，"
                "跳过 {skipped} 个，失败 {failed} 个。"
            ).format(**stats),
        )

    def show_plugin_status(self) -> None:
        """显示插件注册表现状（含失败原因——插件生态里"为什么没生效"最要紧）。"""
        if not _HAS_PLUGINS:
            QtWidgets.QMessageBox.information(self, tr("插件"), tr("当前环境无插件层（orolp.plugins 不可用）。"))
            return
        text = plugin_registry.summary()
        if self.gl_widget.plugin_overlay_errors:
            text += tr("\n  最近一次渲染覆盖层的问题:\n")
            for msg in self.gl_widget.plugin_overlay_errors:
                text += f"    ! {msg}\n"
        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle(tr("插件状态"))
        box.setText(text)
        box.setDetailedText(text)
        box.exec()

    # ------------------------------------------------------------------
    # 缓存可观测性（v1.2.4）：三类结果缓存的命中情况一屏看完
    # ------------------------------------------------------------------

    @staticmethod
    def _cache_status_text() -> str:
        """把 :func:`orolp.incremental.cache_overview` 排成可读文本。

        全局关闭的缓存仍占一行如实写"已关闭"——看不见的关闭最容易被
        当成"没这回事"。
        """
        lines = []
        for row in cache_overview():
            kind = row["kind"]
            if kind == "plan":
                lines.append(tr("规划与分段缓存（TrajectoryCache）"))
            elif kind == "collision":
                lines.append(tr("干涉检查结果缓存（CollisionResultCache）"))
            else:
                lines.append(tr("力矩校验结果缓存（DynamicsResultCache）"))
            if not row.get("enabled"):
                lines.append("  " + tr("已关闭"))
                lines.append("")
                continue
            if kind == "plan":
                lines.append(
                    "  "
                    + tr(
                        "整程序 {entries}/{max} 份（{wp} 路点） · "
                        "命中 {hits} / 未命中 {misses}（命中率 {rate:.0%}）"
                    ).format(
                        entries=row["plan_entries"],
                        max=row["max_entries"],
                        wp=row["plan_waypoints"],
                        hits=row["plan_hits"],
                        misses=row["plan_misses"],
                        rate=row["plan_hit_rate"],
                    )
                )
                lines.append(
                    "  "
                    + tr(
                        "分段 {entries}/{wp_max} 条（{wp} 路点） · "
                        "命中 {hits} / 未命中 {misses}（命中率 {rate:.0%}）"
                    ).format(
                        entries=row["segment_entries"],
                        wp_max=row["segment_max_waypoints"],
                        wp=row["segment_waypoints"],
                        hits=row["segment_hits"],
                        misses=row["segment_misses"],
                        rate=row["segment_hit_rate"],
                    )
                )
                lines.append("  " + tr("LRU 淘汰 {n} 次").format(n=row["evictions"]))
            else:
                lines.append(
                    "  "
                    + tr(
                        "条目 {entries}/{max} · 命中 {hits} / 未命中 {misses}"
                        "（命中率 {rate:.0%}）· 存储 {stores} · 淘汰 {evictions}"
                    ).format(
                        entries=row["entries"],
                        max=row["max_entries"],
                        hits=row["hits"],
                        misses=row["misses"],
                        rate=row["hit_rate"],
                        stores=row["stores"],
                        evictions=row["evictions"],
                    )
                )
            lines.append("")
        if not any(r.get("hits") or r.get("plan_hits") or r.get("segment_hits")
                   or r.get("misses") or r.get("plan_misses") or r.get("segment_misses")
                   for r in cache_overview()):
            lines.append(tr("本轮还没有缓存活动（计数全为 0）。"))
        return "\n".join(lines).rstrip()

    def show_cache_status(self) -> None:
        """显示三类结果缓存的占用与命中情况（v1.2.4）。"""
        if not _HAS_INCREMENTAL:
            QtWidgets.QMessageBox.information(
                self, tr("缓存状态"), tr("当前环境无缓存层（orolp.incremental 不可用）。")
            )
            return
        text = self._cache_status_text()
        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle(tr("缓存状态"))
        box.setText(tr("内容寻址缓存不会给出过期结果；这里的数字解释\"活少干了多少\"。"))
        box.setInformativeText(text)
        box.setDetailedText(text)
        box.exec()

    def clear_caches(self) -> None:
        """清空三类缓存条目并把统计归零（用户要求的"强制全量重算"）。"""
        if not _HAS_INCREMENTAL:
            return
        clear_all_caches()
        reset_all_cache_stats()
        self.statusBar().showMessage(
            tr("已清空全部缓存并把统计归零——下次校验将全量真算。"), 8000
        )

    def _adopt_scene(self, scene) -> None:
        """接管一个新场景，把各处引用一次性换掉。

        场景对象是**按引用共享**的：视口、属性面板、工件下拉各存了一份。
        替换场景时三处必须一起换，否则会出现"树里变了、视口没变"这类
        极难排查的不一致。
        """
        self.scene_panel.set_scene(scene)
        self.gl_widget.scene = scene
        self.code_editor.set_scene(scene)
        if hasattr(self, "property_panel"):
            self.property_panel.set_scene(scene)
        self.scene_panel._refresh_tree()
        self.scene_panel.sceneChanged.emit()
        self.gl_widget.update()

    def load_preset_cell(self) -> None:
        """一键载入预置工作单元场景（Demo 起手式）。

        预置场景放在 ``assets/cells/``：载入后机器人、夹爪（已挂法兰）、
        工作台、工件、输送线、围栏、控制柜一次到位，可以直接开始示教。
        """
        if not _HAS_WORLD or not hasattr(self, "scene_panel"):
            QtWidgets.QMessageBox.information(self, tr("预置工作站"), tr("环境建模模块不可用。"))
            return

        cells_dir = Path(__file__).resolve().parent / "assets" / "cells"
        files = sorted(cells_dir.glob("*.json")) if cells_dir.is_dir() else []
        if not files:
            QtWidgets.QMessageBox.information(
                self, tr("预置工作站"), tr("未找到预置场景（{cells} 下没有 .json）").format(cells=cells_dir)
            )
            return

        if len(files) == 1:
            target = files[0]
        else:
            names = [f.stem for f in files]
            choice, ok = QtWidgets.QInputDialog.getItem(
                self, tr("载入预置工作站"), tr("选择工作站:"), names, 0, False
            )
            if not ok:
                return
            target = files[names.index(choice)]

        try:
            scene = Scene.load(target)
        except (OSError, ValueError) as exc:
            QtWidgets.QMessageBox.critical(self, tr("载入失败"), str(exc))
            return

        # 场景里的机器人可能与当前视口的不同——先让视口加载它
        if scene.robot_urdf and Path(scene.robot_urdf).exists():
            self.gl_widget.load_robot(scene.robot_urdf)
            if scene.has_robot:
                self.gl_widget.set_joint_values(
                    {n: math.radians(v) for n, v in zip(
                        self.gl_widget.renderer.actuated_names(),
                        scene._robot_joints_deg)}
                )

        self._adopt_scene(scene)
        self.statusBar().showMessage(
            tr("已载入 {name}（{n} 个对象）").format(name=target.name, n=len(scene))
        )

    def load_grasping_workstation_demo(self) -> None:
        """一键载入抓取工作站并自动装上示例程序（S4 能力展示入口）。

        与 :meth:`load_preset_cell` 只装场景不同，这里把演示做成**一气呵成**：

        1. 构建模板场景（UR5e + 吸盘 + 输送带 + 4 工件 + 托盘）并载入视口
        2. 用模板 TCP + IK 生成五点式抓取循环示教点（含抓取/放置语义）
        3. 灌进路径编辑器与离线编程面板（与手工示教同一条数据通路）
        4. 自动校验 → 状态栏显示节拍 → 自动开始轨迹回放

        任一步失败都弹窗如实说明，不做静默降级。
        """
        if not (_HAS_TEMPLATES and _HAS_WORLD and _HAS_PROGRAM and _HAS_PICKPLACE):
            QtWidgets.QMessageBox.information(
                self, tr("抓取工作站演示"), tr("所需模块不可用（模板/环境建模/离线编程/抓取工艺）。")
            )
            return

        try:
            cell = build_grasping_workstation()
        except Exception as exc:
            QtWidgets.QMessageBox.critical(
                self, tr("抓取工作站演示"), tr("模板构建失败：{kind}: {msg}").format(kind=type(exc).__name__, msg=exc)
            )
            return

        # URDF 路径兼容仓库相对路径（模板 JSON 里存的是相对路径）
        urdf = cell.scene.robot_urdf
        if urdf and not Path(urdf).exists():
            candidate = _REPO_ROOT / urdf
            if candidate.exists():
                urdf = str(candidate)
        if not urdf or not Path(urdf).exists():
            QtWidgets.QMessageBox.critical(
                self, tr("抓取工作站演示"), tr("机器人模型不可用：{urdf}").format(urdf=urdf)
            )
            return

        # 1) 视口载机器人（顺带初始化离线编程面板与场景机器人）
        self.load_robot(urdf)
        robot = self.olp_robot
        if robot is None:
            QtWidgets.QMessageBox.warning(
                self, tr("抓取工作站演示"), tr("离线编程机器人初始化失败，无法生成示例程序。")
            )
            return

        # 2) 生成示例示教点（TCP 取模板定义的吸盘 TCP）
        try:
            positions, types = demo_taught_points(cell, robot)
        except Exception as exc:
            QtWidgets.QMessageBox.critical(
                self, tr("抓取工作站演示"),
                tr("示例程序生成失败：{kind}: {msg}").format(kind=type(exc).__name__, msg=exc),
            )
            return

        # 3) 接管模板场景 → 灌示教点（与手工示教同一条数据通路）
        self._adopt_scene(cell.scene)
        self.code_editor.recorded_positions = positions
        self.code_editor.point_types = list(types)
        self.code_editor.pos_count_label.setText(f"Positions: {len(positions)}")
        self.code_editor._update_code_with_positions()
        self.gl_widget.set_recorded_positions(positions)
        self._sync_program_points()

        # 工件下拉默认选第一个可抓工件，回放即可看到吸附/释放随动
        if cell.grasp_objects and hasattr(self.code_editor, "workpiece_combo"):
            idx = self.code_editor.workpiece_combo.findData(cell.grasp_objects[0])
            if idx >= 0:
                self.code_editor.workpiece_combo.setCurrentIndex(idx)

        # 视口关节摆到模板 Home 位（load_robot 用的是通用待机位形，与模板不同）
        names = self.gl_widget.renderer.actuated_names()
        self.gl_widget.set_joint_values(
            {n: math.radians(v) for n, v in zip(names, cell.home_joints)}
        )
        self._sync_scene_joints()

        # 4) 自动校验出节拍 → 自动回放
        report = self.program_panel.validate_program()
        if report is not None:
            self.program_panel.start_playback()
        cycle = getattr(report, "cycle_time", None)
        cycle_txt = (
            tr("，节拍 {t} s").format(t=f"{cycle:.2f}") if cycle is not None else ""
        )
        self.statusBar().showMessage(
            tr(
                "抓取工作站演示已就绪：{n} 个示例示教点{cycle_txt}，"
                "回放中（可暂停/单步/停止）"
            ).format(n=len(positions), cycle_txt=cycle_txt)
        )

    # ------------------------------------------------------------------ #
    # 工程文件（.orolp，v1.3.4）
    # ------------------------------------------------------------------ #

    def _project_name(self) -> str:
        """当前工程名（工程名 → 文件名 → 空串；显示时才用「未命名工程」兜底）。"""
        if _HAS_PROJECT and self.project is not None:
            if self.project.name:
                return self.project.name
            if self.project.path is not None:
                return self.project.path.stem
        return ""

    def _update_window_title(self) -> None:
        """标题带工程名与未保存标记——"这个界面里装的是哪个工程"一眼可见。"""
        mark = tr(" •未保存") if self._project_dirty else ""
        name = self._project_name() or tr("未命名工程")
        self.setWindowTitle(
            f"RobotWorld Terminal v{_OROLP_VERSION} — {name}{mark}"
        )

    def _set_project_dirty(self, dirty: bool = True) -> None:
        if not _HAS_PROJECT:
            return
        self._project_dirty = bool(dirty)
        self._update_window_title()

    def _collect_into(self, project) -> None:
        """把当前会话的各部分写进工程对象。

        **只收界面里真的有的东西**：没有场景就不写场景段，而不是写一段空的
        ——工程摘要里"工程里没有：场景"比"场景：0 个对象"诚实。
        """
        if _HAS_WORLD and hasattr(self, "scene_panel"):
            scene = getattr(self.scene_panel, "scene", None)
            if scene is not None and (len(scene) or getattr(scene, "has_robot", False)):
                project.set("scene", scene.to_dict())

        if hasattr(self, "code_editor"):
            types = [
                getattr(t, "value", str(t))
                for t in (self.code_editor.get_point_types() or [])
            ]
            project.set("program", {
                "code": self.code_editor.toPlainText(),
                "point_types": types,
                "joint_names": list(self.code_editor.joint_names or []),
            })

        if (
            _HAS_WORLD
            and hasattr(self, "scene_panel")
            and hasattr(self.scene_panel, "sequence_editor")
        ):
            project.set(
                "sequence", self.scene_panel.sequence_editor.sequence.to_dict()
            )

        cell = getattr(getattr(self, "code_editor", None), "cell", None)
        if _HAS_CELL and cell is not None:
            project.set("cell", cell_snapshot(cell))

        panel = getattr(self, "library_panel", None)
        sources = list(getattr(panel, "parts_sources", []) or [])
        project.set("parts", {"sources": sources})

    @staticmethod
    def _resolve_project_path(raw: str, base) -> "Path | None":
        """把工程里记的路径解析成真实文件（原样 → 工程目录 → 仓库根）。"""
        candidate = Path(raw)
        if candidate.exists():
            return candidate
        for root in (base, _REPO_ROOT):
            if root is None:
                continue
            alt = Path(root) / raw
            if alt.exists():
                return alt
        return None

    def _apply_project(self, project) -> list:
        """把工程内容应用到界面，返回**应用时发现的问题**（人话字符串）。"""
        issues: list = []

        # ---- 场景 ---- #
        payload = project.get("scene")
        if payload is not None:
            if not (_HAS_WORLD and hasattr(self, "scene_panel")):
                issues.append(tr("场景段已跳过：环境建模不可用"))
            else:
                scene = None
                try:
                    scene = Scene.from_dict(payload)
                except Exception as exc:
                    issues.append(tr("场景段载入失败：{msg}").format(msg=exc))
                if scene is not None:
                    urdf = scene.robot_urdf
                    if urdf and not Path(urdf).exists():
                        alt = _REPO_ROOT / urdf
                        if alt.exists():
                            urdf = str(alt)
                    if urdf and Path(urdf).exists():
                        self.load_robot(urdf)
                        names = self.gl_widget.renderer.actuated_names()
                        joints = getattr(scene, "_robot_joints_deg", []) or []
                        if getattr(scene, "has_robot", False) and len(joints) == len(names):
                            self.gl_widget.set_joint_values(
                                {n: math.radians(v) for n, v in zip(names, joints)}
                            )
                    elif urdf:
                        issues.append(
                            tr("场景里的机器人模型找不到：{urdf}（场景仍已载入）").format(urdf=urdf)
                        )
                    self._adopt_scene(scene)
                    if _HAS_PROGRAM and hasattr(self, "program_panel"):
                        self.program_panel.set_scene(scene)

        # ---- 示教程序（含代码文本里没有的工艺语义） ---- #
        payload = project.get("program")
        if payload is not None:
            if not hasattr(self, "code_editor"):
                issues.append(tr("示教程序段已跳过：路径编辑器不可用"))
            else:
                self.code_editor.setPlainText(payload.get("code") or "")
                self.code_editor._parse_code_to_positions()
                n_pts = len(self.code_editor.get_recorded_positions())
                types_raw = [str(t) for t in (payload.get("point_types") or [])]
                if types_raw and _HAS_PICKPLACE:
                    if len(types_raw) != n_pts:
                        issues.append(
                            tr(
                                "示教点语义 {a} 个与解析出的 {b} 个点不一致，"
                                "已按默认语义重建（不猜哪个是抓取点）"
                            ).format(a=len(types_raw), b=n_pts)
                        )
                        self.code_editor.sync_point_types()
                    else:
                        try:
                            self.code_editor.point_types = [PointType(t) for t in types_raw]
                        except ValueError as exc:
                            issues.append(
                                tr("示教点语义含未知值（{msg}），已按默认语义重建").format(msg=exc)
                            )
                            self.code_editor.sync_point_types()
                positions = self.code_editor.get_recorded_positions()
                self.gl_widget.set_recorded_positions(positions)
                self._sync_program_points()

        # ---- 工艺时序 ---- #
        payload = project.get("sequence")
        if payload is not None:
            editor = getattr(getattr(self, "scene_panel", None), "sequence_editor", None)
            if editor is None or not _HAS_SEQUENCE:
                issues.append(tr("工艺时序段已跳过：时序编辑器不可用"))
            else:
                try:
                    editor.set_sequence(ProcessSequence.from_dict(payload))
                except Exception as exc:
                    issues.append(tr("工艺时序段载入失败：{msg}").format(msg=exc))

        # ---- 设备点表 ---- #
        payload = project.get("cell")
        if payload is not None:
            if not (_HAS_CELL and hasattr(self, "code_editor")):
                issues.append(tr("设备点表段已跳过：工作单元模块不可用"))
            else:
                try:
                    cell = cell_from_snapshot(payload)
                except ValueError as exc:
                    issues.append(tr("设备点表段载入失败：{msg}").format(msg=exc))
                else:
                    self.code_editor.set_cell(cell)
                    editor = getattr(getattr(self, "scene_panel", None), "sequence_editor", None)
                    if editor is not None:
                        editor.set_available_signals(
                            sorted(
                                {sig for d in cell.devices for sig in d.io_map}
                                | set(HANDSHAKE_SIGNALS)
                            )
                        )

        # ---- 零件库来源 ---- #
        payload = project.get("parts")
        if payload is not None:
            sources = [str(s) for s in (payload.get("sources") or [])]
            panel = getattr(self, "library_panel", None)
            if not (_HAS_ROBOT_LIBRARY and panel is not None):
                if sources:
                    issues.append(tr("零件库来源已跳过：机器组件库不可用"))
            else:
                base = project.path.parent if getattr(project, "path", None) else None
                for raw in sources:
                    resolved = self._resolve_project_path(raw, base)
                    if resolved is None:
                        issues.append(
                            tr("零件库文件找不到：{src}（未导入；工程里仍记录着它）").format(src=raw)
                        )
                        continue
                    try:
                        n = panel.import_parts_file(resolved)
                    except Exception as exc:
                        issues.append(
                            tr("零件库导入失败：{src}（{msg}）").format(src=raw, msg=exc)
                        )
                    else:
                        self.statusBar().showMessage(
                            tr("已从工程恢复零件库：{n} 个自定义零件").format(n=n)
                        )
        return issues

    def save_project(self) -> None:
        """「保存工程…」——把场景/示教程序/时序/点表/零件来源收进一个 .orolp。"""
        if not _HAS_PROJECT:
            QtWidgets.QMessageBox.information(
                self, tr("保存工程"), tr("工程文件模块不可用。")
            )
            return
        # 已在某工程上工作时不新建对象：**未知段与未知顶层键要跟着走**
        # （否则"打开—保存"一趟就把别家工具写的字段抹掉了）
        project = self.project if self.project is not None else Project()
        suggested = (
            str(project.path)
            if project.path is not None
            else str(_REPO_ROOT / f"{self._project_name() or 'workcell'}{PROJECT_EXTENSION}")
        )
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, tr("保存工程"), suggested, PROJECT_FILTER
        )
        if not path:
            return
        self._collect_into(project)
        try:
            target = project.save(path)
        except ProjectError as exc:
            QtWidgets.QMessageBox.critical(self, tr("保存工程失败"), str(exc))
            return
        self.project = project
        self._set_project_dirty(False)
        self._record_recent(target)
        self._clear_autosave_snapshot()
        summary = project.summary()
        self.statusBar().showMessage(
            tr("工程已保存：{name}（{n} 段）").format(name=target.name, n=len(summary))
        )

    def open_project(self) -> None:
        """「打开工程…」——选文件后走 :meth:`open_project_path` 同一条路。"""
        if not _HAS_PROJECT:
            QtWidgets.QMessageBox.information(
                self, tr("打开工程"), tr("工程文件模块不可用。")
            )
            return
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, tr("打开工程"), "", PROJECT_FILTER
        )
        if not path:
            return
        self.open_project_path(path)

    def open_project_path(
        self, path: str, record_recent: bool = True, clear_autosave: bool = True
    ) -> bool:
        """载入 .orolp，逐段应用，并把跳过/缺失如实报出来。

        「打开工程…」对话框与「最近打开」菜单条目走的是**同一条路**，
        返回是否载入成功（最近打开菜单用它判断要不要记这条记录）。
        崩溃恢复也走这条路，但 `record_recent=False`（快照是内部槽位，
        不进最近打开）、`clear_autosave=False`（快照正是刚恢复的内容，
        在用户把它存到真实位置前还得留着）。
        """
        if not _HAS_PROJECT:
            return False
        try:
            project = Project.load(path)
        except ProjectError as exc:
            QtWidgets.QMessageBox.critical(self, tr("打开工程失败"), str(exc))
            return False

        issues = self._apply_project(project)
        self.project = project
        self._set_project_dirty(False)
        if record_recent:
            self._record_recent(project.path or Path(path))
        # 磁盘上已有一份工程了，旧的自动保存快照过时且有误导性 → 清掉
        if clear_autosave:
            self._clear_autosave_snapshot()

        report = project.load_report
        lines = [tr("已打开 {name}").format(name=Path(path).name)]
        if report is not None:
            lines.append(report.describe())
        if issues:
            lines.append(tr("应用时的问题："))
            lines.extend("  · " + s for s in issues)
        text = "\n".join(lines)
        if issues or (report is not None and not report.ok()):
            box = QtWidgets.QMessageBox(self)
            box.setWindowTitle(tr("打开工程"))
            box.setText(text)
            box.exec()
        self.statusBar().showMessage(
            tr("工程已打开：{name}").format(name=Path(path).name)
        )
        return True

    # ---------------- 最近打开（v1.3.5） ---------------- #

    def _record_recent(self, path) -> None:
        """记一条最近打开（打开成功 / 保存成功后调用；尽力而为）。"""
        if not _HAS_SESSION_SUPPORT or path is None:
            return
        recents = RecentProjects.load()
        recents.add(str(path))
        recents.save()
        self._rebuild_recent_menu()

    def _rebuild_recent_menu(self) -> None:
        """按最近打开记录重建子菜单；失效条目置灰（不悄悄藏）。"""
        if not _HAS_SESSION_SUPPORT:
            return
        menu = getattr(self, "_recent_menu", None)
        if menu is None:
            return
        menu.clear()
        recents = RecentProjects.load()
        entries = recents.paths or []
        if not entries:
            empty = menu.addAction(tr("（暂无记录）"))
            empty.setEnabled(False)
            return
        for index, path in enumerate(entries, start=1):
            action = menu.addAction(f"&{index}  {path}")
            action.setEnabled(Path(path).exists())
            if action.isEnabled():
                action.triggered.connect(
                    lambda _checked=False, p=path: self.open_project_path(p)
                )
        menu.addSeparator()
        clear_action = menu.addAction(tr("清除最近打开记录"))
        clear_action.triggered.connect(self._clear_recent_menu_records)

    def _clear_recent_menu_records(self) -> None:
        if not _HAS_SESSION_SUPPORT:
            return
        RecentProjects().save()
        self._rebuild_recent_menu()
        self.statusBar().showMessage(tr("已清除最近打开记录"))

    # ---------------- 崩溃自动恢复（v1.3.5） ---------------- #

    def _write_autosave_snapshot(self) -> bool:
        """把当前会话快照写进自动保存目录（不动正在编辑的工程对象）。

        快照走一份**新建的** Project：收集会话内容 + 带上原工程的
        未知段/未知顶层键/元信息——原工程对象的 ``path`` 不能被快照
        覆盖，否则下次手动保存的建议路径会错。
        """
        if not (_HAS_SESSION_SUPPORT and self.project is not None):
            return False
        snapshot = Project(
            name=self.project.name,
            note=self.project.note,
            created=self.project.created,
            modified=self.project.modified,
            orolp_version=self.project.orolp_version,
        )
        snapshot.raw_sections = dict(self.project.raw_sections)
        snapshot.extra = dict(self.project.extra)
        self._collect_into(snapshot)
        source_path = (
            str(self.project.path) if self.project.path is not None else None
        )
        return write_autosave(snapshot, source_path=source_path) is not None

    def _autosave_tick(self) -> None:
        """定时器心跳：只在有未保存改动时落快照（干净的会话没什么可救）。"""
        if not _HAS_SESSION_SUPPORT or not self._project_dirty:
            return
        self._write_autosave_snapshot()

    def _clear_autosave_snapshot(self) -> None:
        if _HAS_SESSION_SUPPORT:
            clear_autosave()

    def maybe_prompt_recovery(self) -> bool:
        """启动时若存在自动保存快照，询问是否恢复；恢复成功返回 True。

        只问一次（``showEvent`` 可能触发多次）；没有任何可靠元数据的
        快照不弹询问——快照文件本身仍是合法 .orolp，用户可以手动打开。
        """
        if not _HAS_SESSION_SUPPORT or self._recovery_prompted:
            return False
        self._recovery_prompted = True
        info = read_autosave()
        if info is None:
            return False
        source = info.source_path or tr("（未命名工程）")
        saved_at = info.saved_at or tr("（未知时间）")
        answer = QtWidgets.QMessageBox.question(
            self,
            tr("恢复上次会话"),
            tr(
                "检测到上次会话的自动保存。\n\n"
                "工程：{source}\n保存时间：{saved}\n\n"
                "是否恢复？（不恢复也可以稍后用「打开工程」手动打开快照：\n{snapshot}）"
            ).format(source=source, saved=saved_at, snapshot=info.snapshot),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.Yes,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return False
        if not self.open_project_path(
            str(info.snapshot), record_recent=False, clear_autosave=False
        ):
            return False
        # 恢复的内容在快照里、不在盘上的"工程位置"——按未保存改动对待，
        # 让标题的"•未保存"和自动保存继续守护它；手动保存的建议路径
        # 指回原工程位置（快照路径只是内部槽位，不该被当成保存目标）
        if self.project is not None:
            self.project.path = Path(info.source_path) if info.source_path else None
            if not info.source_path:
                self.project.name = ""
        self._set_project_dirty(True)
        self.statusBar().showMessage(
            tr("已从自动保存恢复：{saved}").format(saved=saved_at)
        )
        return True

    # ---------------- 从模板新建工程（v1.3.6） ---------------- #

    def new_project_from_template(self, template_id: str | None = None) -> bool:
        """「从模板新建工程」；传入 ``template_id`` 时跳过选择对话框。

        模板产物走 :meth:`_apply_project` 同一条应用通路（与打开工程
        一致）；新建的工程尚未落盘，按未保存改动对待（标题「•未保存」
        + 自动保存继续守护）。
        """
        if not (_HAS_PROJECT_TEMPLATES and _HAS_PROJECT):
            QtWidgets.QMessageBox.information(
                self, tr("从模板新建工程"), tr("工程模板模块不可用。")
            )
            return False
        if template_id is None:
            template_id = self._choose_project_template()
            if not template_id:
                return False
        if self._project_dirty:
            answer = QtWidgets.QMessageBox.question(
                self,
                tr("从模板新建工程"),
                tr("当前工程有未保存改动，新建模板工程会丢弃这些改动。\n\n是否继续？"),
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if answer != QtWidgets.QMessageBox.Yes:
                return False
        try:
            project = build_project_template(template_id)
        except Exception as exc:
            QtWidgets.QMessageBox.critical(
                self,
                tr("从模板新建工程"),
                tr("模板构建失败：{kind}: {msg}").format(
                    kind=type(exc).__name__, msg=exc
                ),
            )
            return False

        issues = self._apply_project(project)
        self.project = project
        self._set_project_dirty(True)
        if issues:
            box = QtWidgets.QMessageBox(self)
            box.setWindowTitle(tr("从模板新建工程"))
            box.setText(
                tr("已从模板新建：{name}").format(name=project.name or template_id)
                + "\n"
                + tr("应用时的问题：")
                + "\n"
                + "\n".join("  · " + s for s in issues)
            )
            box.exec()
        self.statusBar().showMessage(
            tr("已从模板新建：{name}（尚未保存，可直接修改后「保存工程」）").format(
                name=project.name or template_id
            )
        )
        return True

    def _choose_project_template(self) -> "str | None":
        """模板选择对话框；返回选中的模板 id，取消返回 ``None``。"""
        templates = list_project_templates()
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle(tr("从模板新建工程"))
        layout = QtWidgets.QVBoxLayout(dialog)
        layout.addWidget(
            QtWidgets.QLabel(tr("选择一个模板作为新工程的起点："))
        )
        listw = QtWidgets.QListWidget(dialog)
        for tpl in templates:
            item = QtWidgets.QListWidgetItem(tpl["name"])
            item.setData(Qt.UserRole, tpl["id"])
            item.setToolTip(tpl["description"])
            listw.addItem(item)
        desc = QtWidgets.QLabel("")
        desc.setWordWrap(True)
        layout.addWidget(listw)
        layout.addWidget(desc)

        def _sync_desc() -> None:
            item = listw.currentItem()
            tid = item.data(Qt.UserRole) if item is not None else None
            desc.setText(
                next((t["description"] for t in templates if t["id"] == tid), "")
            )

        listw.currentItemChanged.connect(lambda *_: _sync_desc())
        listw.itemDoubleClicked.connect(lambda *_: dialog.accept())
        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        listw.setMinimumWidth(420)
        listw.setCurrentRow(0)
        _sync_desc()
        if dialog.exec() != QtWidgets.QDialog.Accepted:
            return None
        item = listw.currentItem()
        return item.data(Qt.UserRole) if item is not None else None

    def setup_menu(self):
        menubar = self.menuBar()
        
        file_menu = menubar.addMenu("&File")
        open_action = QAction("&Open Model...", self)
        open_action.setShortcut(QKeySequence.Open)
        open_action.triggered.connect(self.open_model)
        file_menu.addAction(open_action)
        
        save_action = QAction("&Save Path Code...", self)
        save_action.setShortcut(QKeySequence.Save)
        save_action.triggered.connect(self.save_path_code)
        file_menu.addAction(save_action)
        
        load_action = QAction("&Load Path Code...", self)
        load_action.setShortcut("Ctrl+O")
        load_action.triggered.connect(self.load_path_code)
        file_menu.addAction(load_action)

        if _HAS_PROJECT:
            file_menu.addSeparator()
            open_project_action = QAction(tr("打开工程(&O)…"), self)
            open_project_action.setShortcut("Ctrl+Shift+O")
            open_project_action.setToolTip(
                tr(
                    "打开 .orolp 工程文件：一次载入场景、示教程序（含抓取/放置语义）、"
                    "工艺时序、设备点表与零件库来源。\n"
                    "缺段/坏段只跳过那一段并在报告里说明——不会因为一段格式问题打不开整个工程。"
                )
            )
            open_project_action.triggered.connect(self.open_project)
            file_menu.addAction(open_project_action)

            save_project_action = QAction(tr("保存工程(&S)…"), self)
            save_project_action.setShortcut("Ctrl+Shift+S")
            save_project_action.setToolTip(
                tr(
                    "把当前工作会话存成一个 .orolp 工程文件（原子写入）。\n"
                    "本库不认识的段与字段会原样保留，往返一趟不丢内容。"
                )
            )
            save_project_action.triggered.connect(self.save_project)
            file_menu.addAction(save_project_action)

            if _HAS_PROJECT_TEMPLATES:
                new_tpl_action = QAction(tr("从模板新建工程(&N)…"), self)
                new_tpl_action.setToolTip(
                    tr(
                        "从内置模板（抓取工作站 / 分拣工位 / 机床上下料 / 3C 装配 / 汽车焊装）新建工程：\n"
                        "场景与起步程序一步到位，不必从空白开始。\n"
                        "当前工程有未保存改动时会先询问。"
                    )
                )
                new_tpl_action.triggered.connect(
                    lambda: self.new_project_from_template()
                )
                file_menu.addAction(new_tpl_action)

            if _HAS_SESSION_SUPPORT:
                file_menu.addSeparator()
                self._recent_menu = file_menu.addMenu(tr("最近打开(&R)"))
                self._recent_menu.setToolTip(
                    tr(
                        "最近打开过的 .orolp 工程文件（最新在前，最多 10 条）。\n"
                        "文件已不存在的条目会置灰，不会被悄悄藏掉。"
                    )
                )
                # 展开时重建：文件被移走/改名后，失效条目立即置灰
                self._recent_menu.aboutToShow.connect(self._rebuild_recent_menu)
                self._rebuild_recent_menu()

        if _HAS_WORLD:
            file_menu.addSeparator()
            cell_action = QAction(tr("载入预置工作站(&W)..."), self)
            cell_action.triggered.connect(self.load_preset_cell)
            file_menu.addAction(cell_action)

        if _HAS_TEMPLATES and _HAS_PROGRAM and _HAS_PICKPLACE:
            demo_action = QAction(tr("抓取工作站演示（一键）(&D)"), self)
            demo_action.setToolTip(
                tr(
                    "一键载入抓取工作站（UR5e + 吸盘 + 输送带 + 托盘），\n"
                    "自动装上示例抓取程序并开始回放，状态栏显示节拍。"
                )
            )
            demo_action.triggered.connect(self.load_grasping_workstation_demo)
            file_menu.addAction(demo_action)

        file_menu.addSeparator()

        gen_action = QAction("&Generate Robot Code...", self)
        gen_action.setShortcut("Ctrl+G")
        gen_action.triggered.connect(self.generate_robot_code)
        file_menu.addAction(gen_action)

        if _HAS_CONTROLLER_PANEL:
            link_action = QAction(tr("真机连接…"), self)
            link_action.setToolTip(
                tr(
                    "连接真机控制器：部署当前程序、回读实际关节角、取回核对。\n"
                    "传输可选 Socket（orolp-socket 行协议）或 SSH（SFTP 传程序 + exec）。\n"
                    "无运动指令入口、无自由命令输入框——程序运动一律走部署。"
                )
            )
            link_action.triggered.connect(self.show_controller_dialog)
            file_menu.addAction(link_action)

        file_menu.addSeparator()
        
        exit_action = QAction("E&xit", self)
        exit_action.setShortcut(QKeySequence.Quit)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        view_menu = menubar.addMenu("&View")
        reset_view = QAction("&Reset View", self)
        reset_view.setShortcut("R")
        reset_view.triggered.connect(self.reset_view)
        view_menu.addAction(reset_view)

        if _HAS_ROBOT_LIBRARY:
            view_menu.addSeparator()
            show_lib = QAction(tr("显示&机器组件库"), self)
            show_lib.setShortcut("Ctrl+L")
            show_lib.triggered.connect(self.show_robot_library)
            view_menu.addAction(show_lib)

            reload_lib = QAction(tr("重新扫描机器组件库"), self)
            reload_lib.setShortcut("F5")
            reload_lib.triggered.connect(self.reload_robot_library)
            view_menu.addAction(reload_lib)

        if _HAS_WORLD:
            view_menu.addSeparator()
            check_col = QAction(tr("工作单元干涉检查"), self)
            check_col.setShortcut("F6")
            check_col.triggered.connect(self.check_collisions)
            view_menu.addAction(check_col)

        if _HAS_PROGRAM:
            if not _HAS_WORLD:
                view_menu.addSeparator()
            validate_prog = QAction(tr("程序校验与节拍估算"), self)
            validate_prog.setShortcut("F7")
            validate_prog.triggered.connect(self.validate_program)
            view_menu.addAction(validate_prog)

        if _HAS_PLUGINS:
            view_menu.addSeparator()
            self.action_plugin_overlays = QAction(tr("插件覆盖层"), self)
            self.action_plugin_overlays.setCheckable(True)
            self.action_plugin_overlays.setChecked(True)
            self.action_plugin_overlays.toggled.connect(self.toggle_plugin_overlays)
            view_menu.addAction(self.action_plugin_overlays)

            reload_plugins = QAction(tr("重新扫描已安装插件"), self)
            reload_plugins.triggered.connect(self.reload_plugins)
            view_menu.addAction(reload_plugins)

            show_plugins = QAction(tr("插件状态…"), self)
            show_plugins.triggered.connect(self.show_plugin_status)
            view_menu.addAction(show_plugins)

        if _HAS_INCREMENTAL:
            view_menu.addSeparator()
            show_cache = QAction(tr("缓存状态…"), self)
            show_cache.triggered.connect(self.show_cache_status)
            view_menu.addAction(show_cache)

            clear_cache = QAction(tr("清空缓存与统计"), self)
            clear_cache.triggered.connect(self.clear_caches)
            view_menu.addAction(clear_cache)

        # 语言 / Language（v1.0.10）：切换即保存配置，重启后全面生效
        lang_menu = menubar.addMenu(tr("语言 / Language"))
        self._lang_actions = {}
        for code, name in available_languages():
            act = QAction(name, self)
            act.setCheckable(True)
            act.setChecked(code == current_language())
            act.triggered.connect(lambda _checked=False, c=code: self._switch_language(c))
            self._lang_actions[code] = act
            lang_menu.addAction(act)

    def _switch_language(self, code: str) -> None:
        """切换界面语言并提示重启（面板文案在构造期写入，需重启全面生效）。"""
        if code == current_language():
            return
        set_language(code)
        for c, act in getattr(self, "_lang_actions", {}).items():
            act.setChecked(c == code)
        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle(tr("语言 / Language"))
        box.setText(tr("切换语言需要重启应用才能完全生效。现在重启吗？"))
        restart_btn = box.addButton(tr("立即重启"), QtWidgets.QMessageBox.AcceptRole)
        box.addButton(tr("稍后"), QtWidgets.QMessageBox.RejectRole)
        box.exec()
        if box.clickedButton() is restart_btn:
            if getattr(sys, "frozen", False):
                argv = [a for a in sys.argv[1:] if not a.startswith("-psn")]
            else:
                argv = [sys.argv[0]] + [a for a in sys.argv[1:] if not a.startswith("-psn")]
            QtCore.QProcess.startDetached(sys.executable, argv)
            QtWidgets.QApplication.closeAllWindows()
            sys.exit(0)

    def _set_window_icon(self):
        """设置窗口图标与**应用级**图标。

        以源码方式启动（``python app.py``）时，macOS 把进程当作 Python 解释器，
        Dock 上显示 python 图标与 ``python3.11``。设置应用级图标后，非 bundle
        进程的 Dock 图标会取用该图标。

        Dock **名称**由进程名决定，Qt 层面无法覆盖——正式发布需打包成
        ``RobotWorld.app``，由 Info.plist 的 ``CFBundleName`` 确定（见 app.spec）。
        """
        import sys
        logo_path = Path(__file__).parent / "logo.png"
        if hasattr(sys, '_MEIPASS'):
            logo_path = Path(sys._MEIPASS) / "logo.png"

        if not logo_path.exists():
            return

        icon = QtGui.QIcon(str(logo_path))
        self.setWindowIcon(icon)                      # 窗口标题栏
        QtGui.QGuiApplication.setWindowIcon(icon)     # 应用级：影响 macOS Dock
    
    def setup_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        
        main_layout = QtWidgets.QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(2)
        
        self.code_editor = PathCodeEditor()
        self.code_editor.playbackRequested.connect(self.on_playback_requested)
        self.code_editor.playbackStopped.connect(self.on_playback_stopped)
        self.code_editor.clearRequested.connect(self.on_clear_requested)
        self.code_editor.positionsParsed.connect(self.on_positions_parsed)

        if _HAS_ROBOT_LIBRARY:
            # 机器人库（eCatalog）与路径代码编辑器共用一个标签页容器
            self.library_panel = RobotLibraryPanel(models_dir=_REPO_ROOT / "models")
            self.library_panel.robotSelected.connect(self.load_robot)
            self.library_panel.componentActivated.connect(self._add_component_from_library)
            self.library_panel.statusMessage.connect(self.statusBar().showMessage)

            if _HAS_WORLD:
                # 注意：此时 gl_widget 尚未创建，信号连接在 setup_ui 末尾进行
                self.scene_panel = ScenePanel(Scene())
                self.scene_panel.statusMessage.connect(self.statusBar().showMessage)

            if _HAS_PROGRAM:
                self.program_panel = ProgramPanel()
                self.program_panel.statusMessage.connect(self.statusBar().showMessage)
                self.program_panel.playbackFrame.connect(self.on_playback_frame)
                self.program_panel.playbackFinished.connect(self.on_playback_stopped)
                # 程序级回放到 gripper() 指令时，驱动工件吸附/释放（S3）
                self.program_panel.gripperStateChanged.connect(
                    self.on_gripper_state_changed
                )
                # 轨迹编辑器（v0.2.4）改动示教点 → 回写路径代码编辑器
                self.program_panel.taughtPointsChanged.connect(
                    self.on_taught_points_changed
                )

            # ---- 左：双 Tab（① 场景设备库 ② 设备树）----
            # 左侧只负责"挑设备 + 编排层级"：Tab1 从库里选设备，Tab2 把设备摆成
            # 层级（含挂法兰自动吸合）。代码预览、导出打包仍留在右侧输出区。
            self.left_tabs = QtWidgets.QTabWidget()
            self.left_tabs.setMaximumWidth(360)
            self.left_tabs.setMinimumWidth(250)
            self.left_tabs.addTab(self.library_panel, tr("场景设备库"))
            if _HAS_WORLD and hasattr(self, "scene_panel"):
                self.left_tabs.addTab(self.scene_panel, tr("设备树"))
            main_layout.addWidget(self.left_tabs)
        else:
            # 无机器人库时左侧只保留设备树；代码编辑器已归入右侧「代码预览」
            # 标签，这里不能再挂一次，否则会被重复 reparent。
            self.left_tabs = QtWidgets.QTabWidget()
            self.left_tabs.setMaximumWidth(360)
            self.left_tabs.setMinimumWidth(250)
            if _HAS_WORLD and hasattr(self, "scene_panel"):
                self.left_tabs.addTab(self.scene_panel, tr("设备树"))
            main_layout.addWidget(self.left_tabs)
        
        self.gl_widget = GLWidget(self)
        self.gl_widget.setMinimumSize(400, 300)
        main_layout.addWidget(self.gl_widget, 1)
        
        self.property_panel = DevicePropertyPanel()
        self.property_panel.jointChanged.connect(self.on_joint_changed)
        self.property_panel.jointChanged.connect(self.on_joint_changed_for_record)

        # ---- 右：可折叠多功能标签页（核心输出区）----
        # 一次只展开一个标签 → 界面无冗余信息；属性在前，选中对象时可自动切到它。
        self.right_tabs = QtWidgets.QTabWidget()
        self.right_tabs.setMaximumWidth(330)
        self.right_tabs.setMinimumWidth(260)
        self.right_tabs.addTab(self.property_panel, tr("属性"))
        self.right_tabs.addTab(self.code_editor, tr("代码预览"))
        if _HAS_PROGRAM and hasattr(self, "program_panel"):
            self.right_tabs.addTab(self.program_panel, tr("导出打包"))
        main_layout.addWidget(self.right_tabs)

        # 场景改动需要触发视口重绘，此连接必须等 gl_widget 建好后才能做
        if _HAS_WORLD and hasattr(self, "scene_panel"):
            self.scene_panel.sceneChanged.connect(self.gl_widget.update)
            # 视口右键拾取/拖动选中对象时，同步到左侧场景面板树形选择
            self.gl_widget.objectSelected.connect(self.scene_panel._select_object)
            # 左键拖拽摆放完成（v1.1.7）：把"落在谁上面"说给状态栏听
            self.gl_widget.objectPlaced.connect(self.statusBar().showMessage)
            # 视口侧改动挂载关系（拖到法兰自动装夹 / 拖已装夹件自动解除）：
            # 属性面板回填 + 设备树层级跟着变 + 视口重绘
            self.gl_widget.sceneChanged.connect(self.property_panel.set_object)
            self.gl_widget.sceneChanged.connect(
                lambda _n: self.scene_panel._refresh_tree()
            )
            self.gl_widget.sceneChanged.connect(lambda _n: self.gl_widget.update())
            # 选中对象（视口 / 场景树两条来源）同步到右侧属性面板
            self.property_panel.set_scene(self.scene_panel.scene)
            self.gl_widget.objectSelected.connect(self.property_panel.set_object)
            self.scene_panel.objectSelected.connect(self.property_panel.set_object)
            # 场景树选中也要让视口画黄色高亮（同步 gl_widget.selected_object）
            self.scene_panel.objectSelected.connect(
                lambda n: (
                    setattr(self.gl_widget, "selected_object", n),
                    self.gl_widget.set_current_robot(n),
                    self.gl_widget.update(),
                )
            )
            # 右面板编辑对象属性后重绘视口
            self.property_panel.objectChanged.connect(self.gl_widget.update)
            # 挂载关系变化后场景树的层级也要跟着变（挂到法兰的夹爪要显示在
            # 「机器人」节点下），否则树里仍显示在根层，与视口不一致
            self.property_panel.objectChanged.connect(self.scene_panel._refresh_tree)
            # 场景增删对象后同步「工件」下拉，否则新加的工件选不到
            self.scene_panel.sceneChanged.connect(self.code_editor.refresh_workpiece_combo)
            self.code_editor.set_scene(self.scene_panel.scene)
            # 机器组件库拖入视口：组件加入场景、机器人按 catalog key 载入
            self.gl_widget.componentDropped.connect(self._add_component_from_library)
            self.gl_widget.robotDropped.connect(self._on_robot_dropped)
            # 工艺时序（v1.3.1）：把点表里真实存在的信号名注入时序编辑器，
            # 校验才能查出"序列用了但点表里没有"的信号
            self.scene_panel.sequence_editor.set_available_signals(
                sorted({sig for d in self.code_editor.cell.devices for sig in d.io_map}
                       | set(HANDSHAKE_SIGNALS))
            )
            # 工程未保存标记（v1.3.4）：会话里任何"值得存下来"的改动都置脏。
            # 刻意只挂这四条粗粒度信号——细粒度（每个对象位姿变化）噪声太大，
            # 会把标记变成常亮的装饰。
            self.scene_panel.sceneChanged.connect(self._set_project_dirty)
            self.scene_panel.sequence_editor.sequenceChanged.connect(self._set_project_dirty)
            self.code_editor.code_edit.textChanged.connect(self._set_project_dirty)

        if _HAS_PROGRAM and hasattr(self, "program_panel"):
            self.program_panel.taughtPointsChanged.connect(
                lambda *_args: self._set_project_dirty()
            )

        self._setup_status_bar()
        if _HAS_PROGRAM and hasattr(self, "program_panel"):
            self.program_panel.reportReady.connect(self._on_report_ready)

        self.statusBar().showMessage("Ready")
    
    def open_model(self):
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open Robot Model", "models", "URDF Files (*.urdf);;All Files (*)"
        )
        if file_path:
            self.load_robot(file_path)
    
    def _register_robot_in_scene(self, path) -> str:
        """把机器人登记进工作单元场景：**已有机器人时追加**，返回其名字。

        回归：此前一律走 ``Scene.set_robot``，名字固定为 ``ROBOT_NAME``，于是
        "再增加一台机器人"实际是把已有的那台顶掉。

        Returns:
            场景中的机器人名
        """
        scene = self.scene_panel.scene
        if scene.robot_names:
            # 追加并沿 X 错开基座，避免两台叠在一起（随后可自行拖放到位）
            idx = len(scene.robot_names)
            name = self.scene_panel.add_robot(
                str(path),
                pose=Pose.from_xyz_rpy(1500.0 * idx, 0.0, 0.0, 0.0, 0.0, 0.0),
            )
            return name or scene.robot_names[-1]
        self.scene_panel.set_robot(str(path))
        return scene.robot_names[0] if scene.robot_names else ROBOT_NAME

    def load_robot(self, path):
        """加载 URDF 到 3D 视口，并同步属性面板。

        若该文件属于机器人库中的某个型号，属性面板会回显负载、臂展等完整规格。
        """
        # 先决定"追加还是替换"：场景已有机器人时必须用 add_robot（唯一名），
        # 否则 set_robot 的固定名 ROBOT_NAME 会把已有那台顶掉——"再加一台"
        # 就变成了"替换"。名字要先定，渲染器才能登记到对应名下。
        robot_name = ROBOT_NAME
        if _HAS_WORLD and hasattr(self, "scene_panel"):
            robot_name = self._register_robot_in_scene(path)

        if not self.gl_widget.load_robot(path, robot_name):
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to load URDF: {path}")
            return

        # 记录当前模型路径：后处理需要据此推断品牌与关节结构。
        # 历史实现从不更新该字段，导致即使已载入机器人，
        # 依赖 urdf_path 的功能（如生成控制器代码）仍认为"未加载模型"。
        self.urdf_path = str(path)

        entry = None
        if _HAS_ROBOT_LIBRARY and hasattr(self, "library_panel"):
            entry = self.library_panel.catalog.find_by_urdf(path)

        self.property_panel.update_robot_info(self.gl_widget.renderer, entry)
        
        # 设置关节名称顺序，用于解析路径代码
        joint_names = list(self.gl_widget.renderer.joints.keys())
        self.code_editor.set_joint_names(joint_names)
        
        recorded = self.code_editor.get_recorded_positions()
        if recorded:
            self.gl_widget.set_recorded_positions(recorded)
        self.statusBar().showMessage(f"Loaded: {Path(path).name}")

        # 同步机器人到工作单元场景，使其参与干涉检查
        if _HAS_WORLD and hasattr(self, "scene_panel"):
            self.gl_widget.scene = self.scene_panel.scene
            # 必须同步：渲染器加载时会摆到 HOME 待机位（_apply_home_pose），
            # 而 Scene 里刚建的机器人是**零位**。不同步就出现"画出来的手臂是
            # 弯的、场景法兰却竖直朝天"——挂到法兰上的夹爪跑到手臂外，看着像
            # 装夹失败（位置面板却显示 0/0/0，因为挂载本身成功了）。
            self._sync_scene_joints()
            # 让「代码生成 → 硬件列表」能按场景补齐设备清单
            self.code_editor._scene = self.scene_panel.scene
            self.code_editor.sync_devices(self.scene_panel.scene)
            # 载入后即把机器人作为当前对象：右面板显示关节滑块，视口画黄色高亮
            if robot_name in self.scene_panel.scene.objects:
                self.gl_widget.selected_object = robot_name
                self.gl_widget.update()
                self.property_panel.set_object(robot_name)

        # 离线编程面板需要真正的 orolp.Robot（含运动学），而不是只用于
        # 绘制的 renderer——校验与节拍估算都建立在 FK/IK 之上
        if _HAS_PROGRAM and hasattr(self, "program_panel"):
            try:
                from orolp import Robot as OrolpRobot

                self.olp_robot = OrolpRobot(str(path), Path(path).stem)
                self.program_panel.set_robot(self.olp_robot, joint_names)
                # 「代码生成 → 机器人子程序」需要同样的真实机器人
                self.code_editor.set_robot(self.olp_robot, joint_names)
                self.program_panel.set_scene(
                    self.scene_panel.scene if _HAS_WORLD and hasattr(self, "scene_panel") else None
                )
                self._sync_program_points()
            except Exception as exc:
                print(f"[warning] 无法为离线编程面板准备机器人: {exc}")
                self.olp_robot = None

        self.gl_widget.update()

    def _sync_program_points(self) -> None:
        """把路径代码编辑器里的示教点同步给离线编程面板。

        工艺语义（抓取/放置点）一并传过去：程序面板据此在对应位置插入
        gripper() 指令，回放动画与出码共用同一份抓取时序（S3）。
        """
        if not (_HAS_PROGRAM and hasattr(self, "program_panel")):
            return
        types = self.code_editor.get_point_types() if _HAS_PICKPLACE else None
        self.program_panel.set_taught_points(
            self.code_editor.get_recorded_positions(), types
        )

    def on_taught_points_changed(self, positions, types, motions) -> None:
        """轨迹编辑器改动示教点后回写路径代码编辑器（v0.2.4）。

        两条编辑通路共用同一份示教点数据：面板里增删改了路点，左侧
        代码预览的 movej 文本与点位计数必须同步，否则下次 Play 会用旧数据。
        """
        if not hasattr(self, "code_editor"):
            return
        self.code_editor.set_recorded_positions(list(positions))
        if _HAS_PICKPLACE and types is not None:
            self.code_editor.point_types = list(types)

    def validate_program(self) -> None:
        """校验程序并给出节拍（F7）。"""
        if not (_HAS_PROGRAM and hasattr(self, "program_panel")):
            QtWidgets.QMessageBox.information(self, tr("程序校验"), tr("离线编程模块不可用。"))
            return
        if hasattr(self, "right_tabs"):
            self.right_tabs.setCurrentWidget(self.program_panel)
        self._sync_program_points()
        self.program_panel.validate_program()

    # ------------------------------------------------------------------ #
    # 真机连接（v1.1.1）：SocketLink 的界面化
    # ------------------------------------------------------------------ #

    def show_controller_dialog(self) -> None:
        """打开「真机连接」对话框（模式less：连接期间界面仍可查看仿真）。

        首次打开时创建并接线，之后复用同一个对话框——重复打开不会叠出
        多条连接。每次打开都刷新机器人引用：用户可能在两次打开之间换了型号。
        """
        if not _HAS_CONTROLLER_PANEL:
            QtWidgets.QMessageBox.information(
                self, tr("真机连接"), tr("真机连接模块不可用。")
            )
            return
        if self._controller_dialog is None:
            dialog = QtWidgets.QDialog(self)
            dialog.setWindowTitle(tr("真机连接"))
            dialog.setModal(False)
            dialog.resize(400, 780)  # v1.1.4 加了传输选择与 SSH 字段
            layout = QtWidgets.QVBoxLayout(dialog)
            panel = ControllerPanel()
            layout.addWidget(panel)
            panel.set_robot(self.olp_robot)
            panel.set_program_provider(self._controller_program_payload)
            panel.statusMessage.connect(self.statusBar().showMessage)
            panel.jointsRead.connect(self.on_controller_joints_read)
            self._controller_dialog = dialog
            self._controller_panel = panel
        else:
            self._controller_panel.set_robot(self.olp_robot)
        self._controller_dialog.show()
        self._controller_dialog.raise_()
        self._controller_dialog.activateWindow()

    def _controller_program_payload(self):
        """把离线编程面板的当前程序交给真机连接面板部署。

        与「一键导出」同一条数据通路（示教点 → build_program → to_post），
        后处理器取导出打包标签里当前选择的（``None`` = 按机器人品牌自动推断）。
        """
        if not (_HAS_PROGRAM and hasattr(self, "program_panel") and self.olp_robot):
            return None
        self._sync_program_points()
        program = self.program_panel.build_program()
        if program is None:
            return None
        return {
            "program": program,
            "post_processor": self.program_panel.combo_post.currentData(),
        }

    def on_controller_joints_read(self, joints) -> None:
        """真机回读的关节角（度）摆进视口——仿真立刻等于真机当前位形。

        只改**本侧**（视口 + 场景 + 属性面板），绝不向控制器下发任何运动指令；
        方向只有"真机 → 仿真"，这是安全的。
        """
        names = list(self.gl_widget.renderer.actuated_names())
        try:
            vals = [float(v) for v in joints]
            if len(vals) != len(names):
                raise ValueError(
                    tr("关节数不符（真机 {n}，模型 {m}）").format(n=len(vals), m=len(names))
                )
        except Exception as exc:
            self.statusBar().showMessage(tr("回读关节应用失败：{exc}").format(exc=exc))
            return
        # 与回放帧同一通路：视口 + 属性面板 + 场景同步 + 重绘
        self.on_playback_frame(
            {name: math.radians(v) for name, v in zip(names, vals)}
        )
        self.statusBar().showMessage(tr("已按真机位形摆好视口"))

    def on_playback_frame(self, joint_values: dict) -> None:
        """回放帧：按插补后的密集轨迹驱动视口，而不是在示教点之间瞬移。"""
        self.gl_widget.set_joint_values(joint_values)
        if hasattr(self, "property_panel"):
            self.property_panel.update_joint_displays(joint_values)
        self._sync_scene_joints()
        self.gl_widget.update()

    def on_joint_changed(self, name, value):
        self.gl_widget.renderer.joint_values[name] = value
        self._sync_scene_joints()
        self.gl_widget.update()

    def _sync_scene_joints(self) -> None:
        """把**当前机器人**的渲染关节同步给场景，让干涉检查/法兰反映实时位形。

        必须指名道姓：``set_robot_joints`` 默认只作用于主机器人，多机时会把
        "拖 A 的关节"写到 B 身上——表现为两台一起动、或挂在 A 上的工具离开法兰。
        """
        if not (_HAS_WORLD and hasattr(self, "scene_panel")):
            return
        scene = self.scene_panel.scene
        if not scene.has_robot:
            return
        robot = getattr(self.gl_widget, "current_robot", None) or ROBOT_NAME
        if robot not in scene.robots:
            robot = scene.robot_names[0]
        renderer = self.gl_widget.renderer_of(robot)
        # 必须用 actuated_names() 而非 joints.keys()：后者含 fixed 关节
        # （KR16 是 7 个 joints / 6 个可动），多传一个会让 FK 直接抛错
        names = list(renderer.actuated_names())
        self.scene_panel.update_robot_joints(
            [math.degrees(renderer.joint_values.get(n, 0.0)) for n in names],
            name=robot,
        )

    def check_collisions(self) -> None:
        """执行工作单元干涉检查（F6），并把结果高亮到视口。"""
        if not (_HAS_WORLD and hasattr(self, "scene_panel")):
            QtWidgets.QMessageBox.information(self, tr("干涉检查"), tr("环境建模模块不可用。"))
            return
        if hasattr(self, "left_tabs"):
            self.left_tabs.setCurrentWidget(self.scene_panel)
        hits = self.scene_panel.run_collision_check()
        self.gl_widget.set_collision_results(hits)

        # 干涉结果写进常驻状态栏：演示时一眼可见，不用去翻面板
        if hasattr(self, "status_collision"):
            self.status_collision.setText(tr("干涉 {n}").format(n=len(hits)))
            self.status_collision.setStyleSheet(
                "color:#ff8a80; padding:0 8px; font-size:11px;" if hits
                else "color:#a5d6a7; padding:0 8px; font-size:11px;"
            )

    def on_joint_changed_for_record(self, name, value):
        self.gl_widget.renderer.joint_values[name] = value
        self.gl_widget.update()
        if self.code_editor.record_btn.isChecked():
            # 当记录模式开启时，调用 GLWidget 的记录方法
            count = self.gl_widget.record_position()
            # 同步到 PathCodeEditor
            self.code_editor.recorded_positions = self.gl_widget.get_recorded_positions()
            # 上面是直接改写列表，语义列表要跟着补，否则长度不一致
            self.code_editor.sync_point_types()
            self.code_editor.pos_count_label.setText(f"Positions: {count}")
            self.code_editor._update_code_with_positions()
            self._sync_program_points()

    def on_playback_requested(self, index):
        positions = self.code_editor.get_recorded_positions()
        if 0 <= index < len(positions):
            self.gl_widget.set_joint_values(positions[index])
            self.property_panel.update_joint_displays(positions[index])
            # 必须同步到场景：视口渲染器与工作单元场景是两套状态。只更新渲染器
            # 的话场景机器人永远停在初始位形，法兰不动，挂在法兰上的夹爪与
            # 被吸附的工件都不会跟着走（干涉检查也会按错误位形算）。
            self._sync_scene_joints()
            self._apply_pick_event(index)
            self._update_io_status(index)
            self.gl_widget.update()

    def _update_io_status(self, index: int) -> None:
        """常驻状态栏显示当前工位的 IO 动作——演示时能一眼看到夹爪在干嘛。"""
        if not hasattr(self, "status_io"):
            return
        action = tr("保持")
        if _HAS_PICKPLACE:
            types = self.code_editor.get_point_types()
            t = types[index] if index < len(types) else None
            action = {PointType.PICK: tr("夹紧"), PointType.PLACE: tr("松开")}.get(t, tr("保持"))
        wp = self.code_editor.get_workpiece()
        self.status_io.setText(f"IO {action}" + (f" · {wp}" if wp else ""))

    def _apply_pick_event(self, index: int) -> None:
        """播放到某个示教点时，按工艺语义挂载/释放工件。

        工件被「吸附」的实现就是把它挂到机器人下（``parent=ROBOT_NAME``）——
        复用与夹爪挂载完全相同的父子机制，于是它自动跟随法兰运动。
        释放即解除挂载，并把当前世界位姿写回局部 pose，工件原地留下。
        """
        if not _HAS_PICKPLACE or not _HAS_WORLD:
            return
        scene = getattr(self, "scene", None) or getattr(
            getattr(self, "scene_panel", None), "scene", None
        )
        if scene is None:
            return

        types = self.code_editor.get_point_types()
        if index >= len(types):
            return
        t = types[index]
        workpiece = self.code_editor.get_workpiece()
        if not workpiece or workpiece not in scene.objects:
            return

        try:
            if t is PointType.PICK:
                scene.attach(workpiece, ROBOT_NAME, keep_world=True)
            elif t is PointType.PLACE:
                scene.detach(workpiece)
            else:
                return
        except (KeyError, ValueError) as exc:
            print(f"[warning] 工件吸附/释放失败: {exc}")
            return
        if hasattr(self, "scene_panel"):
            self.scene_panel._refresh_tree()

    def on_gripper_state_changed(self, action: str) -> None:
        """程序级回放到夹爪指令时，吸附/释放工件（S3 抓取动画）。

        与示教点回放（:meth:`_apply_pick_event`）共用同一机制：把工件
        ``attach`` 到机器人下并保持世界位姿——被夹住的瞬间停在原地，之后
        随法兰/TCP 随动；``detach`` 即原地释放。``action`` 为空串表示回放
        从头开始，此时把残留挂载的工件复位。
        """
        if not _HAS_WORLD:
            return
        scene = getattr(self, "scene", None) or getattr(
            getattr(self, "scene_panel", None), "scene", None
        )
        if scene is None:
            return
        workpiece = self.code_editor.get_workpiece()
        if not workpiece or workpiece not in scene.objects:
            return
        obj = scene.objects.get(workpiece)
        if obj is None:
            return
        try:
            if action == "close":
                if obj.parent != ROBOT_NAME:
                    scene.attach(workpiece, ROBOT_NAME, keep_world=True)
            elif obj.parent == ROBOT_NAME:  # "open" 或回放复位（""）
                scene.detach(workpiece)
            else:
                return  # 状态未变，不必刷新
        except (KeyError, ValueError) as exc:
            print(f"[warning] 工件吸附/释放失败: {exc}")
            return
        if hasattr(self, "scene_panel"):
            self.scene_panel._refresh_tree()
        if hasattr(self, "status_io"):
            label = {"close": tr("夹紧"), "open": tr("松开")}.get(action, tr("保持"))
            self.status_io.setText(f"IO {label} · {workpiece}")
        # 视口按场景世界位姿重画——工件随动就体现在这一帧帧的刷新里
        self.gl_widget.update()

    def on_playback_stopped(self):
        self.statusBar().showMessage("Playback finished")

    def on_positions_parsed(self):
        """当路径代码解析完成后，更新 GLWidget 的路径可视化"""
        positions = self.code_editor.get_recorded_positions()
        if positions:
            self.gl_widget.set_recorded_positions(positions)
            self.gl_widget.update()
        self._sync_program_points()

    def on_clear_requested(self):
        self.gl_widget.clear_recorded_positions()
        self._sync_program_points()
        self.statusBar().showMessage("Recorded positions cleared")

    def save_path_code(self):
        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save Path Code", "", "Python Files (*.py);;All Files (*)"
        )
        if file_path:
            with open(file_path, 'w') as f:
                f.write(self.code_editor.toPlainText())
            self.statusBar().showMessage(f"Saved: {Path(file_path).name}")

    def generate_robot_code(self):
        """把示教点编译成目标控制器的机器人代码。

        与tr("保存路径代码")不同，这里走的是完整的后处理链路
        （``orolp.program.Program.to_post``），输出 KRL / RAPID / TP / URScript
        等**控制器可直接加载**的源码，而不是中间的 Python 形态路径文本。

        出错时弹窗明确提示——不做静默降级。历史上后处理器一旦失败就会输出
        一堆 ``; Motion 1: ...`` 注释占位，界面却显示成功，极易误导。
        """
        positions = self.code_editor.get_recorded_positions()
        if not positions:
            QtWidgets.QMessageBox.information(
                self, tr("生成机器人代码"),
                tr("当前没有示教点。\n请先在 3D 视口中调整关节角度并点击「记录」按钮。"),
            )
            return

        urdf_path = getattr(self, "urdf_path", None)
        if not urdf_path or not Path(urdf_path).exists():
            QtWidgets.QMessageBox.warning(
                self, tr("生成机器人代码"),
                tr("未加载机器人模型，无法确定目标品牌与后处理器。\n请先从「机器人库」载入一个型号。"),
            )
            return

        try:
            from orolp import Motion, MotionType, Program, Robot
            from orolp.post import list_post_processors
            from orolp.post.base import PostProcessError

            robot = Robot(str(urdf_path), Path(urdf_path).stem)
            joint_names = self.code_editor.joint_names or robot.joint_names

            motions = []
            for pos in positions:
                # 示教点内部以弧度保存，Motion 期望角度
                joints_deg = [
                    math.degrees(float(pos.get(name, 0.0))) for name in joint_names
                ]
                motions.append(
                    Motion(MotionType.JOINTS, joints_deg, speed=50.0, blend=0.0)
                )

            program = Program(robot, motions)
            code = program.to_post(metadata={"program_name": "OROLP_PATH"})

        except PostProcessError as exc:
            # 后处理失败必须让用户看见，而不是退回注释占位文本
            QtWidgets.QMessageBox.critical(
                self, tr("代码生成失败"),
                str(exc) + tr("\n\n可用后处理器：\n  ") + "\n  ".join(list_post_processors()),
            )
            return
        except Exception as exc:
            QtWidgets.QMessageBox.critical(
                self, tr("代码生成失败"),
                f"{type(exc).__name__}: {exc}",
            )
            return

        self._show_generated_code(code, robot)

    def _show_generated_code(self, code: str, robot) -> None:
        """展示生成的控制器代码，并询问是否保存到文件。"""
        from orolp.post import resolve_for_brand

        brand = ""
        urdf_path = getattr(self, "urdf_path", "") or ""
        parts = Path(urdf_path).parts
        if "models" in parts and parts.index("models") + 1 < len(parts):
            brand = parts[parts.index("models") + 1]

        pp = resolve_for_brand(brand) if brand else None
        ext = pp.file_extension if pp else "txt"
        lang = pp.language_name if pp else "unknown"

        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle(tr("生成的机器人代码 — {lang}").format(lang=lang))
        dialog.resize(720, 520)

        layout = QtWidgets.QVBoxLayout(dialog)
        info = QtWidgets.QLabel(
            tr("机器人：{name}    目标语言：{lang}    指令数：{n}").format(name=robot.name, lang=lang, n=len(code.splitlines()))
        )
        layout.addWidget(info)

        editor = QtWidgets.QPlainTextEdit()
        editor.setPlainText(code)
        editor.setReadOnly(True)
        font = QFont("Monospace", 10)
        font.setStyleHint(QFont.Monospace)
        editor.setFont(font)
        layout.addWidget(editor)

        buttons = QtWidgets.QDialogButtonBox()
        save_btn = buttons.addButton(tr("保存为文件…"), QtWidgets.QDialogButtonBox.AcceptRole)
        buttons.addButton(tr("关闭"), QtWidgets.QDialogButtonBox.RejectRole)
        save_btn.clicked.connect(
            lambda: self._save_generated_code(code, ext, lang, dialog)
        )
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)

        dialog.exec()

    def _save_generated_code(self, code: str, ext: str, lang: str, parent) -> None:
        """把生成的代码写入文件。"""
        default = f"orolp_path.{ext}"
        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            parent, tr("保存 {lang} 代码").format(lang=lang), default,
            f"{lang} Files (*.{ext});;Text Files (*.txt);;All Files (*)",
        )
        if file_path:
            try:
                with open(file_path, "w", encoding="utf-8") as fh:
                    fh.write(code)
                self.statusBar().showMessage(tr("已保存 {lang} 代码: {name}").format(lang=lang, name=Path(file_path).name))
            except OSError as exc:
                QtWidgets.QMessageBox.critical(parent, tr("保存失败"), str(exc))
    
    def load_path_code(self):
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load Path Code", "", "Python Files (*.py);;Text Files (*.txt);;All Files (*)"
        )
        if file_path:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    code_content = f.read()
                self.code_editor.setPlainText(code_content)
                # 解析代码并更新路径
                self.code_editor._parse_code_to_positions()
                # 更新 GLWidget 的路径可视化
                positions = self.code_editor.get_recorded_positions()
                if positions:
                    self.gl_widget.set_recorded_positions(positions)
                    self.gl_widget.update()
                self.statusBar().showMessage(f"Loaded: {Path(file_path).name}")
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Error", f"Failed to load path code: {e}")
    
    def reset_view(self):
        self.gl_widget.camera_dist = 1.2
        self.gl_widget.camera_az = 45.0
        self.gl_widget.camera_el = 30.0
        self.gl_widget.update()


def main():
    fmt = QtGui.QSurfaceFormat()
    fmt.setVersion(2, 1)
    fmt.setProfile(QtGui.QSurfaceFormat.NoProfile)
    QtGui.QSurfaceFormat.setDefaultFormat(fmt)
    
    app = QtWidgets.QApplication(sys.argv)
    
    urdf_path = sys.argv[1] if len(sys.argv) > 1 else None
    window = MainWindow(urdf_path)
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
