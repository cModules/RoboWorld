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
    from orolp.render.urdf_model import rpy_to_matrix
except Exception:  # pragma: no cover - i18n 缺失时退化为恒等
    def tr(s):
        return s

    def available_languages():
        return []

    def current_language():
        return "zh"

    def set_language(lang, **kw):
        return lang

    def rpy_to_matrix(rpy):  # 兜底：与 rpy_to_matrix 同口径的最小实现
        import numpy as _np
        r, p, y = rpy
        cr, sr = _np.cos(r), _np.sin(r)
        cp, sp = _np.cos(p), _np.sin(p)
        cy, sy = _np.cos(y), _np.sin(y)
        return _np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0.0],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0.0],
            [-sp, cp * sr, cp * cr, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])

# v1.4.0 MVP 发版：构建档位开关（``python/orolp/features.py``）。
# MVP 档只暴露跑通了的离线编程闭环，未完善 / 没上过真机的入口不构建；内部开发用
# ``OROLP_FEATURES=full`` 全部恢复（2.0 全功能档同此口径）。
# 开关模块缺失时一律按 **全功能** 处理：宁可多暴露，也不要因为依赖问题把已经
# 能用的功能锁死。
try:
    from orolp.features import describe as _features_describe
    from orolp.features import enabled as _feature_enabled
except Exception:  # pragma: no cover - 仅用于 features 模块缺失时的降级

    def _feature_enabled(name: str) -> bool:
        return True

    def _features_describe() -> str:
        return "full（features 模块缺失，按全功能降级）"


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
    #: 右键点中**某个已示教路径点** → 把该台机器人驱动到那个示教点。
    #: 参数是（机器人名, 该台点表里的下标）——必须带名字：多机时点属于谁
    #: 由画出来的那一段决定，退回"当前那台"会把 A 的点驱动到 B 身上
    pathPointMoveRequested = QtCore.Signal(str, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.renderer = RobotRenderer()
        #: 多机器人：每台一个渲染器，键为场景里的机器人名。
        #: ``self.renderer`` 仍是"当前那台"（关节滑杆、记录点位等既有单机器人
        #: 调用面都走它），绘制时按名字从本字典取，互不覆盖。
        self.renderers: dict = {}
        #: 模型不可用的机器人的**专属空渲染器**（键为机器人名）。见
        #: :meth:`renderer_of`——宁可画不出来，也绝不能借用别人的渲染器。
        self._empty_renderers: dict = {}
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
        #: 多机：示教点按机器人名分桶。``recorded_positions`` 是当前那台的别名，
        #: 形态与从前一致（既有调用面都直接读写它），数据却不再串着用。
        self._recorded_by_robot: dict = {}
        self._recorded_ee_by_robot: dict = {}
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
        # 走到这里说明这台机器人的模型确实取不到（URDF 丢失、路径相对且已被
        # 移动过、或场景还没登记它）。**绝不退回 ``self.renderer``**：那会让它
        # 借用另一台机器人的模型与 ``joint_values``——于是"拖 A 的关节"连 B 一起
        # 转，而且 B 的位置画出来却是 A 的机身（几何与位形双双张冠李戴）。
        # 画不出来是诚实的，画错了是灾难：给一台专属的空渲染器。
        empty = self._empty_renderers.get(name)
        if empty is None:
            empty = RobotRenderer()
            self._empty_renderers[name] = empty
        return empty

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

    # ---------------- 示教点：多机按机器人分桶 ---------------- #
    #
    # 每台机器人各自一串示教点，混在一个列表里就会"给 A 示教的点被算进 B 的
    # 程序"。这里按名字分桶；对外仍保留 ``recorded_positions`` 当前引用形体的
    # plain 属性用法（路径绘制、代码编辑器同步都在读写它），取的是**当前那台**。

    def _recorded_bucket(self, robot: Optional[str] = None) -> list:
        """取 ``robot``（缺省当前机器人）的示教点列表（不存在就建一个）。"""
        key = robot or self.current_robot
        return self._recorded_by_robot.setdefault(key, [])

    def _recorded_ee_bucket(self, robot: Optional[str] = None) -> list:
        """同上，末端点位列表。"""
        key = robot or self.current_robot
        return self._recorded_ee_by_robot.setdefault(key, [])

    @property
    def recorded_positions(self) -> list:
        """当前机器人的示教点（多机分桶后与之等价的旧接口）。"""
        return self._recorded_bucket()

    @recorded_positions.setter
    def recorded_positions(self, value) -> None:
        self._recorded_by_robot[self.current_robot] = list(value)

    @property
    def recorded_ee_positions(self) -> list:
        """当前机器人的末端点位（同上，旧接口兼容）。"""
        return self._recorded_ee_bucket()

    @recorded_ee_positions.setter
    def recorded_ee_positions(self, value) -> None:
        self._recorded_ee_by_robot[self.current_robot] = list(value)

    def set_joints(self, q, robot: Optional[str] = None):
        self.renderer_of(robot or self.current_robot).set_joints(q)
        self.update()

    def get_joint_values(self, robot: Optional[str] = None):
        return dict(self.renderer_of(robot or self.current_robot).joint_values)

    def set_joint_values(self, joint_dict, robot: Optional[str] = None):
        renderer = self.renderer_of(robot or self.current_robot)
        for name, value in joint_dict.items():
            renderer.joint_values[name] = value
        self.update()

    def record_position(self, robot: Optional[str] = None) -> int:
        """记录 ``robot``（缺省当前机器人）当前位形为一个示教点。"""
        key = robot or self.current_robot
        pos = self.get_joint_values(key)
        if pos:
            self._recorded_bucket(key).append(pos)
            self._recorded_ee_bucket(key).append(
                self._compute_end_effector_position(key)
            )
        return len(self._recorded_bucket(key))

    def clear_recorded_positions(self, robot: Optional[str] = None) -> None:
        """清空 ``robot`` 的示教点；``None`` 表示清空所有机器人。"""
        if robot is None:
            self._recorded_by_robot.clear()
            self._recorded_ee_by_robot.clear()
        else:
            self._recorded_by_robot.pop(robot, None)
            self._recorded_ee_by_robot.pop(robot, None)

    def get_recorded_positions(self, robot: Optional[str] = None) -> list:
        """取 ``robot`` 的示教点；``None`` 取当前机器人。"""
        return self._recorded_bucket(robot)

    def set_recorded_positions(self, positions, robot: Optional[str] = None) -> None:
        """导入 ``robot`` 的示教点并重算末端点（路径可视化要用）。"""
        key = robot or self.current_robot
        bucket = self._recorded_bucket(key)
        bucket[:] = list(positions)
        # 重新计算末端执行器位置
        renderer = self.renderer_of(key)
        original_joint_values = dict(renderer.joint_values)
        ee_bucket = self._recorded_ee_bucket(key)
        ee_bucket.clear()
        for pos in positions:
            self.set_joint_values(pos, key)
            ee_bucket.append(self._compute_end_effector_position(key))
        self.set_joint_values(original_joint_values, key)

    def _compute_end_effector_position(self, robot: Optional[str] = None):
        """计算 ``robot``（缺省当前）末端执行器（TCP）的三维位置，单位米。

        走 :class:`RobotModel` 的正向运动学——它与 Pinocchio 的对拍误差为 0，
        且严格遵循 URDF 的 ``T·R`` 变换语义。旧实现是手写的 R·T 版本，
        一旦关节带 rpy 就会算错，这里直接换成已验证的实现。
        """
        renderer = self.renderer_of(robot or self.current_robot)
        if renderer.root_link is None:
            return (0.0, 0.0, 0.0)

        poses = renderer.link_poses()
        if not poses:
            return (0.0, 0.0, 0.0)

        # 末端 = 运动链上最后一个连杆；优先使用 TCP 标记
        model = renderer.model
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

        if self.show_path and self._has_any_path():
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
                if not self.scene.is_robot(hit.obj_b):
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

    def _path_robots(self) -> list:
        """有末端路径可画的机器人名（已移除的不算，免得留下幽灵路径）。"""
        names = []
        for name, pts in self._recorded_ee_by_robot.items():
            if len(pts) < 2:
                continue
            # 场景里已经没有它了 → 不该再画（否则原点会堆一条无主的线）
            if self.scene is not None and name not in self.scene.objects:
                continue
            names.append(name)
        return names

    def _has_any_path(self) -> bool:
        """是否任一机器人有可画的路径。"""
        return bool(self._path_robots())

    def _path_points_for(self, robot: Optional[str] = None) -> list:
        """``robot`` 的末端路径点，换算到**世界系米制**。

        记录的末端点是渲染器正向运动学算出来的，处在**机器人自己的基座系**。
        直接拿来画的话，第二台（以及任何被搬动过/挂上底座的机器人）的路径会
        全部叠在原点——必须左乘该机器人的世界基座位姿。
        """
        key = robot or self.current_robot
        pts = self._recorded_ee_by_robot.get(key) or []
        if not pts:
            return []
        base = self._robot_base_pose_m(key)
        if base is None:
            # 没有场景/基座位姿（独立用法）：沿用基座系，与旧行为一致
            return [tuple(float(v) for v in p) for p in pts]
        out = []
        for p in pts:
            v = np.array(
                [float(p[0]), float(p[1]), float(p[2]), 1.0], dtype=np.float64
            )
            w = base @ v
            out.append((float(w[0]), float(w[1]), float(w[2])))
        return out

    def _draw_recorded_path(self):
        """绘制每台机器人**各自**的末端路径。

        路径属于机器人：既不能只画当前那台（别的机器人示教过的路径会凭空
        消失），也不能都画在原点——末端点是基座系的，得先乘各自的基座位姿。
        """
        for name in self._path_robots():
            pts = self._path_points_for(name)
            if len(pts) < 2:
                continue
            # 当前这台亮、其余压暗：一眼看出"正在编辑谁"
            is_current = name == self.current_robot
            if is_current:
                gl.glLineWidth(2.0)
                gl.glColor3f(0.2, 0.8, 0.2)
            else:
                gl.glLineWidth(1.5)
                gl.glColor3f(0.25, 0.5, 0.25)

            gl.glBegin(GL_LINE_STRIP)
            for x, y, z in pts:
                gl.glVertex3f(x, y, z)
            gl.glEnd()

            if is_current:
                gl.glPointSize(5.0)
                gl.glColor3f(1.0, 0.3, 0.3)
            else:
                gl.glPointSize(3.5)
                gl.glColor3f(0.65, 0.3, 0.3)
            gl.glBegin(GL_POINTS)
            for x, y, z in pts:
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
            #: 本次右键是否已经拖过——拖过就是 gizmo/视角操作，松手不弹菜单
            self._right_moved = False
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
            self._right_moved = True
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

    # ---------------- 路径点：右键「移动到」 ---------------- #

    def path_point_at(self, x: float, y: float, radius_px: float = 14.0):
        """``(x, y)`` 像素位置附近**最近的已示教路径点**。

        路径点就是 :attr:`_recorded_ee_by_robot` 里的末端位姿，与
        :attr:`_recorded_by_robot` 的关节位形**一一对应**——命中第 i 个点，
        就能拿到该点对应的关节值。

        Args:
            x / y: 视口像素坐标
            radius_px: 命中半径（像素）

        Returns:
            ``(机器人名, 点索引)``；附近没有点返回 ``None``
        """
        best = None
        best_d = float(radius_px)
        for robot, ees in self._recorded_ee_by_robot.items():
            if self.scene is not None and robot not in self.scene.robots:
                continue  # 已被移出场景的机器人，它的点不该还能点
            for i, p in enumerate(ees):
                try:
                    sx, sy = self._project(p)
                except Exception:
                    continue
                d = math.hypot(sx - x, sy - y)
                if d <= best_d:
                    best_d, best = d, (robot, i)
        return best

    def contextMenuEvent(self, event):
        """右键路径点 →「移动到」：点哪个点，机器人就走到那个示教点。

        命中优先级**路径点优先**：红点是刻意去点的小目标（命中半径 14px），
        而机器人的拾取体是整台手臂的一个大包围盒——走近机器人的示教点几乎
        总被它挡住。此前按"对象优先"判断，结果靠近机器人时永远弹不出菜单
        （2026-09-24 用户反馈）。

        拖 gizmo 的手势不会被抢：右键**拖动过**就不弹菜单（见
        :attr:`_right_moved`），菜单只在"点一下就松开"时出现。
        """
        if not self._has_any_path():
            return
        # 右键拖过 = gizmo / 视角操作，不是要点菜单
        if getattr(self, "_right_moved", False):
            return
        pos = event.pos()
        hit = self.path_point_at(pos.x(), pos.y())
        if hit is None:
            return
        robot, index = hit
        menu = QtWidgets.QMenu(self)
        action = menu.addAction(
            tr("移动到 {robot} 的 P{n}").format(robot=robot, n=index + 1)
        )
        action.setToolTip(
            tr("把这一台直接走到示教点 P{n} 的关节位形").format(n=index + 1)
        )
        if menu.exec(event.globalPos()) is not None:
            self.pathPointMoveRequested.emit(robot, index)


class PathCodeEditor(QtWidgets.QWidget):
    positionRecorded = Signal(int)
    #: 参数依次是**点索引 / 这个点归属哪台机器人**。机器人名不能省：全局运行
    #: 时一行一个主人，只给索引的话主窗口只能拿"当前那台"去顶，表现为 A 的
    #: 指令把 B 拽着走
    playbackRequested = Signal(int, str)
    playbackStopped = Signal()
    clearRequested = Signal()
    positionsParsed = Signal()  # 新增：代码解析完成信号
    #: 状态栏提示。本类是普通 QWidget、自己没有状态栏，主窗口不在时靠这条
    #: 信号把提示送出去（无接收者时静默丢弃，不会抛异常）
    statusMessage = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.recorded_positions = []
        #: 与 recorded_positions **一一对应的归属机器人名**；``None`` 表示这段
        #: 没标归属（旧工程 / 手打的代码）。路径编辑器一份点表里可能同时装
        #: 着多台机器人的路径——给 B 记点不能把 A 的整段顶掉，"分别执行各台
        #: 的路径"靠的就是这条平行数组。
        self.point_robots: List[Optional[str]] = []
        self.joint_names = []  # 存储关节名称顺序
        self.playback_timer = None
        self.current_playback_index = 0
        self.playback_speed = 50
        #: 与 recorded_positions 一一对应的工艺语义（PointType）。
        #: 抓取点/放置点会在播放时触发工件吸附与释放，并在生成代码时插入 IO 指令
        self.point_types = []
        #: 这一串示教点**属于哪台机器人**。路径编辑器只有一份点表，而视口里
        #: 点选/场景树选中都会切换"当前机器人"却不换点表——播放时必须认这个
        #: 归属，否则播 A 的程序会把 B 拽着走
        self.owner_robot = None
        #: 每条指令的**行号与归属**：``[(行号, 机器人名或 None, 点索引)]``，由
        #: :meth:`_parse_code_to_positions` 建立。"从第几行起跑""只跑选中那台"
        #: 全靠它——光有点值序列是定位不了的
        self._line_map: List[Tuple[int, Optional[str], int]] = []
        #: 本次播放要走的**点索引序列**（已按"只跑某台 / 从光标行起"筛过）
        self._playback_seq: List[int] = []
        #: 手改代码后的**防抖重解析**定时器：边打字边解析既卡，又会把半截
        #: 语法（写到一半的 movej(）灌进点表。停手 300ms 再解析一次
        self._edit_parse_timer = QtCore.QTimer(self)
        self._edit_parse_timer.setSingleShot(True)
        self._edit_parse_timer.setInterval(300)
        self._edit_parse_timer.timeout.connect(self._flush_code_edit)
        #: 走到 :attr:`_playback_seq` 的第几个
        self._playback_pos: int = 0
        #: 由主窗口注入的回调：返回**当前选中对象**关联的机器人名（``None``
        #: 表示没选中或关联不到任何机器人）。编辑器自己看不到视口选中状态
        self.selection_robot_provider = None
        #: 工作单元场景（由 MainWindow 注入），用于枚举可抓取的工件
        self._scene = None
        #: 工作单元模型（硬件列表 / PLC 程序逻辑共用同一份）
        self.cell = CellConfig() if _HAS_CELL else None
        #: 供「机器人子程序」生成用的真实机器人（含运动学）
        self._robot = None
        self._scene = None
        self._setup_ui()
        # 光标一动，"从选中行号处运行"的起点就变了，范围提示要跟着更新
        self.code_edit.cursorPositionChanged.connect(self.refresh_scope)
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
        # 手改代码 → 停手后自动重解析：3D 路径与离线编程面板的点表跟着刷新
        self.code_edit.textChanged.connect(self._edit_parse_timer.start)
        # 右键菜单：标准编辑项 +「执行此行」（逐行核对的入口）
        self.code_edit.setContextMenuPolicy(Qt.CustomContextMenu)
        self.code_edit.customContextMenuRequested.connect(
            self._show_code_context_menu
        )
        teach_layout.addWidget(self.code_edit)

        # 按钮文案一律走 tr()：这一排 6 个键是**左侧栏最宽的一行**，换成中文
        # 后所需宽度从 ~475px 降到 ~300px——既去掉了中英文混用，也在 400px
        # 的栏宽里放得下了（2026-09-21 界面体检）
        self.record_btn = QtWidgets.QPushButton(tr("● 记录"))
        self.record_btn.setStyleSheet("background-color: #555555; color: white;")
        self.record_btn.setCheckable(True)
        self.record_btn.clicked.connect(self._on_record_toggled)

        self.play_btn = QtWidgets.QPushButton(tr("▶ 播放"))
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

        self.stop_btn = QtWidgets.QPushButton(tr("■ 停止"))
        self.stop_btn.setStyleSheet("background-color: #f44336; color: white;")
        self.stop_btn.clicked.connect(self._on_stop_clicked)

        self.clear_btn = QtWidgets.QPushButton(tr("清空"))
        self.clear_btn.setStyleSheet("background-color: #777777; color: white;")
        self.clear_btn.clicked.connect(self._on_clear_clicked)

        self.pos_count_label = QtWidgets.QLabel(tr("示教点：0"))
        self.pos_count_label.setFont(QFont("Consolas", 9))

        #: 将要执行的范围——按下去之前就该知道跑的是哪台、从哪一行开始
        self.scope_label = QtWidgets.QLabel("")
        self.scope_label.setFont(QFont("Consolas", 9))
        self.scope_label.setStyleSheet("color: #9ad0ff;")

        # 播放按钮排成 3×2 网格而不是一行：这个面板在左侧栏里只有 280–400px，
        # 一行摆 6 个按钮 + 2 个提示 + 2 个下拉时 Qt 不会换行，只会把每个按钮
        # 压到 33px（实测：只剩一个色块），并把最后一个下拉顶到面板外面看不见。
        btn_grid = QtWidgets.QGridLayout()
        btn_grid.setHorizontalSpacing(4)
        for _i, _btn in enumerate(
            (
                self.record_btn,
                self.play_btn,
                self.pause_btn,
                self.step_btn,
                self.stop_btn,
                self.clear_btn,
            )
        ):
            _btn.setSizePolicy(
                QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed
            )
            btn_grid.addWidget(_btn, _i // 3, _i % 3)
        teach_layout.addLayout(btn_grid)

        # 点数与"跑哪台、从哪行起"分两行：范围提示本身可以很长
        # （如「将从第 3 行起全局运行（8/8 点）」），和点数挤在一行会把它压没
        info_layout = QtWidgets.QHBoxLayout()
        info_layout.addWidget(self.pos_count_label)
        info_layout.addStretch(1)
        teach_layout.addLayout(info_layout)
        self.scope_label.setWordWrap(True)
        teach_layout.addWidget(self.scope_label)

        # ---- 抓取工艺：点位语义 + 工件选择 ----
        if _HAS_PICKPLACE:
            process_layout = QtWidgets.QHBoxLayout()
            process_layout.addWidget(QtWidgets.QLabel(tr("点位:")))
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
            # 工件名可能很长——不设下限的话它会按最长的一项撑宽整行，
            # 把右边的东西挤出可视区（这在侧栏里等于直接看不见）
            self.point_type_combo.setMinimumContentsLength(4)
            process_layout.addWidget(self.point_type_combo, 1)

            process_layout.addWidget(QtWidgets.QLabel(tr("工件:")))
            self.workpiece_combo = QtWidgets.QComboBox()
            self.workpiece_combo.setMinimumContentsLength(4)
            self.workpiece_combo.setToolTip(tr("被抓取的场景对象；留空表示只演示轨迹"))
            process_layout.addWidget(self.workpiece_combo, 1)
            teach_layout.addLayout(process_layout)

        self.sub_tabs.addTab(teach, tr("路径示教"))

        # ---------- 2) 硬件列表（MVP 隐藏：设备台账还没和出码打通，队列 P2） ----------
        if _feature_enabled("hardware_list"):
            self.sub_tabs.addTab(self._build_hardware_tab(), tr("硬件列表"))

        # ---------- 3) PLC 程序逻辑（MVP 隐藏：出码尚未完善，队列 P2） ----------
        if _feature_enabled("plc_logic"):
            self.sub_tabs.addTab(self._build_plc_tab(), tr("PLC 程序逻辑"))

        # ---------- 4) 机器人子程序（MVP 隐藏：尚未完善，队列 P2） ----------
        if _feature_enabled("subprogram"):
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
            # 用**全量**语义：pts 取自整份点表（可能含多台的路径），只给"当前
            # 编辑那台"那段语义会让点数与语义数对不上
            plan = build_pick_place(pts, self.point_types)
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

    def _device_table(self):
        """硬件列表的表格；MVP 档入口没构建时是 ``None``。"""
        return getattr(self, "device_table", None)

    def _selected_device(self):
        """设备列表里当前选中的设备；无选中返回 ``None``。"""
        if self.cell is None:
            return None
        table = self._device_table()
        if table is None:
            return None
        r = table.currentRow()
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
        table = self._device_table()
        if table is None:
            return
        r = table.currentRow()
        if r < 0 or r >= len(self.cell.devices):
            return
        self.cell.remove(self.cell.devices[r].name)
        self._refresh_device_table()

    def _refresh_device_table(self):
        """把 ``cell`` 里的设备刷到表格上。

        MVP 档表格不存在时**静默返回**而不是报错：``sync_devices`` /
        ``set_cell`` 会调到这里，而设备数据始终在 ``self.cell`` 里——
        PLC、点表读的是同一份，只是没有界面去看它。
        """
        if self.cell is None:
            return
        table = self._device_table()
        if table is None:
            return
        table.setRowCount(len(self.cell.devices))
        for i, d in enumerate(self.cell.devices):
            for j, val in enumerate([d.name, d.type, d.brand, d.model, d.comm, d.ip]):
                table.setItem(i, j, QtWidgets.QTableWidgetItem(val))

    def _sync_devices_from_scene(self):
        self.sync_devices(self._scene)

    def sync_devices(self, scene) -> None:
        """按场景对象补齐设备清单；已存在的保留，避免覆盖用户填的内容。"""
        if self.cell is None or scene is None:
            return
        for name in list(scene.objects):
            if self.cell.find(name):
                continue
            # 与设备库同一套取值：机器人 → 机器人，组件 → 分类
            if scene.is_robot(name):
                dev_type = "机器人"
            else:
                obj = scene.objects[name]
                spec = COMPONENTS.get(obj.component_id) if obj.component_id else None
                dev_type = tr(spec.category) if spec else "传感器"
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
            self.record_btn.setText(tr("● 正在记录"))
            self.record_btn.setStyleSheet("background-color: #f44336; color: white;")
            self.record_btn.setIcon(QtGui.QIcon())
        else:
            self.record_btn.setText(tr("● 记录"))
            self.record_btn.setStyleSheet("background-color: #555555; color: white;")

    def _on_play_clicked(self):
        # 在播放前解析代码编辑器中的路径代码
        self._parse_code_to_positions()
        
        if len(self.recorded_positions) == 0:
            return
        
        # 通知主窗口更新 GLWidget 的路径可视化
        self.positionsParsed.emit()
            
        # 选中了机器人（或挂在它上面的夹爪/工件）→ 只跑这台的路径；
        # 没选中 → 从代码选中行号处全局运行
        robot = self._selected_robot()
        self._playback_seq = self.playback_plan(
            robot=robot, from_cursor=robot is None
        )
        if not self._playback_seq:
            self._notify(tr("没有可运行的路径点"))
            return
        self._ensure_timer()
        if not self.playback_timer.isActive():
            self._playback_pos = 0
            self.playback_timer.start(self.playback_speed)
            self._emit_current()

    def _on_stop_clicked(self):
        self._stop_playback()
        self.playbackStopped.emit()

    def _ensure_timer(self) -> None:
        """惰性创建播放定时器（与 Play 分支共用同一份创建逻辑）。"""
        if self.playback_timer is None:
            self.playback_timer = QtCore.QTimer(self)
            self.playback_timer.timeout.connect(self._on_playback_tick)

    def _emit_current(self) -> None:
        """把当前这一步的**点索引**与**它归属那台**一起发出去。"""
        index = self._playback_seq[self._playback_pos]
        self.current_playback_index = index
        self.playbackRequested.emit(index, self.robot_at(index) or "")

    def _rebuild_seq_if_idle(self) -> None:
        """还没有播放序列（首次单步/继续）时按当前选中与光标行建一条。"""
        if self._playback_seq:
            return
        robot = self._selected_robot()
        self._playback_seq = self.playback_plan(
            robot=robot, from_cursor=robot is None
        )
        self._playback_pos = 0

    def _on_pause_clicked(self):
        """暂停 / 继续。

        暂停只停定时器、**保留当前索引**，这样继续时从断点接着走，而不是
        从头重来——否则逐点核对抓取时机根本没法做。
        """
        if not self.recorded_positions:
            return
        self._ensure_timer()
        self._rebuild_seq_if_idle()
        if self.playback_timer.isActive():
            self.playback_timer.stop()
        else:
            if self._playback_pos >= len(self._playback_seq):
                self._playback_pos = 0
            self.playback_timer.start(self.playback_speed)
        self._sync_pause_button()

    def _on_step_clicked(self):
        """单步：只走一个示教点。"""
        if not self.recorded_positions:
            return
        self._ensure_timer()
        if self.playback_timer.isActive():
            self.playback_timer.stop()
        self._rebuild_seq_if_idle()
        if self._playback_pos >= len(self._playback_seq):
            self._playback_pos = 0
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
        self._playback_pos = 0
        self._sync_pause_button()

    def _on_playback_tick(self):
        if self._playback_pos < len(self._playback_seq):
            self._emit_current()
            self._playback_pos += 1
        else:
            self._stop_playback()
            self.playbackStopped.emit()

    def _on_clear_clicked(self):
        self._stop_playback()
        self.recorded_positions.clear()
        self.point_robots.clear()
        self.point_types.clear()
        self._line_map.clear()
        self.pos_count_label.setText(tr("示教点：0"))
        self.code_edit.clear()
        self.code_edit.append("# OLP Path Code\n")
        self.clearRequested.emit()

    def record_position(self, joint_values, robot=None):
        if self.record_btn.isChecked():
            self.recorded_positions.append(dict(joint_values))
            # 归属跟着点一起进表：不记的话这一段就"没主人"，播放时只能退回
            # 整份归属，多机下会把这台的点驱动到另一台身上
            self.point_robots.append(robot or self.owner_robot)
            if _HAS_PICKPLACE:
                self.point_types.append(self.point_type_combo.currentData())
            self.pos_count_label.setText(tr("示教点：{n}").format(n=len(self.recorded_positions)))
            self.positionRecorded.emit(len(self.recorded_positions))
            self._update_code_with_positions()

    # ``_suggest_next_point_type`` 已删除：它会把点位下拉自动拨到"下一个"
    # 循环语义——用户随手连教几个点，第 2 个就被标成「抓取点」、第 4 个被标
    # 成「放置点」，导出程序的开头凭空多出 5 行夹爪开合（2026-09-21 用户反馈）。
    # 语义是工艺意图，要标请用点位下拉或「抓取向导」，程序不再替用户做主。

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
            self.point_types.append(None)
        # 补默认语义要**按段**来：默认的五点式是每台各一套
        self._fill_missing_types()

    def get_point_types(self, robot=None):
        """``robot``（缺省**当前编辑那台**）的工艺语义。

        一份点表里可能装多台的路径，语义只能一段一段地给：直接返回整份会让
        "切回 A"拿到 A+B 混在一起的语义。``robot`` 解析不到（旧工程没有归属
        标记）时退回整份，行为与从前一致。
        """
        if robot is None:
            robot = self.owner_robot
        if robot is not None and robot in self.point_robots:
            return self.types_of(robot)
        return self.point_types

    def point_type_at(self, index: int):
        """第 ``index`` 个点（**总表索引**）的工艺语义——播放按行序取值用。"""
        return self._type_at(index)

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
                if self._scene.is_robot(name):
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

    def segments(self) -> List[Tuple[Optional[str], List[int]]]:
        """点表按归属切成**连续的段**：``[(机器人名或 None, [点索引...])]``。

        出码与按机归档都用它：同一台的点聚成一段，段首写一个 ``# Robot:``
        头，于是两台机器人的路径在代码里各占一段、一眼分得清。
        """
        out: List[Tuple[Optional[str], List[int]]] = []
        for i, robot in enumerate(self.point_robots):
            if out and out[-1][0] == robot:
                out[-1][1].append(i)
            else:
                out.append((robot, [i]))
        return out

    def _update_code_with_positions(self):
        self.code_edit.clear()
        self.code_edit.append("# OLP Path Code")
        self.code_edit.append(f"# Recorded positions: {len(self.recorded_positions)}")
        # 逐段出码：每台机器人一段，段首带归属。整份只写一个 Robot 头时，
        # 第二台一记点，第一台的路径就从代码里整段消失了——这也是"看不到
        # 机器人信息"的由来。
        for robot, indices in self.segments():
            self.code_edit.append("")
            if robot:
                # 归属写进代码：从别处粘贴 / 载入工程时靠这一行认出这串点属于哪台
                self.code_edit.append(f"# Robot: {robot}")
            for i in indices:
                # 将弧度转换为角度显示
                joints_str = ", ".join(
                    f"{math.degrees(v):.2f}" for v in self.recorded_positions[i].values()
                )
                self.code_edit.append(f"movej([{joints_str}], vel=0.5)")
        self._update_count_label()

    def _update_count_label(self) -> None:
        """点数标签带上**各台各多少点**——归属只写在代码注释里太容易看不见。"""
        n = len(self.recorded_positions)
        counts: dict = {}
        for robot in self.point_robots:
            counts[robot] = counts.get(robot, 0) + 1
        named = {r: c for r, c in counts.items() if r}
        if named:
            detail = " · ".join(f"{r} {c}" for r, c in named.items())
            self.pos_count_label.setText(tr("示教点：{n}（{detail}）").format(n=n, detail=detail))
        else:
            self.pos_count_label.setText(tr("示教点：{n}").format(n=n))

    def get_recorded_positions(self):
        return self.recorded_positions

    def set_recorded_positions(self, positions, owner=None):
        """换上 ``owner`` 的示教点；``owner`` 为空表示整份替换、归属不变。

        只替 ``owner`` **那一段**，其余机器人的段原样保留：编辑器一份点表里
        同时装着多台的路径，给 B 记点若整表替换，A 的路径就被顶掉了——那既
        是"看不到机器人信息"，也是后面播放时把 A 的点算到 B 名下的源头。
        """
        positions = list(positions)
        if owner is None:
            # 整份替换：**归属按原表沿用**——docstring 一直这么承诺，实现却曾把
            # 归属整体抹成 None。无归属段渲染时不带 ``# Robot:`` 头，用户看到的
            # 就是"代码开头莫名其妙多出几行 movej"；而且此后所有按机写入都只能
            # 追加在它后面，这段幽灵行永远顶在最上面（2026-09-21 用户反馈）
            prev_own = list(self.point_robots)
            prev_owner = self.owner_robot
            self.recorded_positions = positions
            self.point_robots = [
                prev_own[i] if i < len(prev_own) else prev_owner
                for i in range(len(positions))
            ]
        else:
            new_pos, new_own, new_types = [], [], []
            old_types = {
                robot: [self._type_at(i) for i in indices]
                for robot, indices in self.segments()
            }
            placed = False
            for robot, indices in self.segments():
                if robot == owner:
                    if placed:
                        continue  # 同属的旧段已被新点取代
                    placed = True
                    prev = old_types.get(robot) or []
                    for k, pos in enumerate(positions):
                        new_pos.append(pos)
                        new_own.append(owner)
                        new_types.append(prev[k] if k < len(prev) else None)
                else:
                    for i in indices:
                        new_pos.append(self.recorded_positions[i])
                        new_own.append(robot)
                        new_types.append(self._type_at(i))
            if not placed:  # 这台还没有段：追加到末尾
                for pos in positions:
                    new_pos.append(pos)
                    new_own.append(owner)
                    new_types.append(None)
            self.recorded_positions = new_pos
            self.point_robots = new_own
            self.point_types = new_types
            self.owner_robot = owner
        self._fill_missing_types()
        self.pos_count_label.setText(tr("示教点：{n}").format(n=len(self.recorded_positions)))
        self._update_code_with_positions()
        self.update()

    def _type_at(self, index: int):
        """第 ``index`` 个点的工艺语义；越界返回 ``None``。"""
        if 0 <= index < len(self.point_types):
            return self.point_types[index]
        return None

    def _fill_missing_types(self) -> None:
        """把 ``point_types`` 的长度对齐到点表（只做截断/补空，**不发明语义**）。

        之前这里会按"五点式循环"给没标语义的点自动填 接近/抓取/提升/放置/撤离
        ——用户只是随手教一条轨迹，导出的程序开头却**自动多出 5 行夹爪开合**
        （2026-09-21 用户反馈：这 5 行不需要）。抓取语义是工艺意图，只能
        用户自己标（点位下拉 / 抓取向导），程序不再替用户做主。
        """
        if not _HAS_PICKPLACE:
            return
        n = len(self.recorded_positions)
        if len(self.point_types) > n:
            self.point_types = self.point_types[:n]
        while len(self.point_types) < n:
            self.point_types.append(None)

    def points_of(self, robot: Optional[str]) -> List[dict]:
        """``robot`` 那一段的示教点（按代码里的先后顺序）。"""
        return [self.recorded_positions[i] for i in self.indices_of(robot)]

    def indices_of(self, robot: Optional[str]) -> List[int]:
        """``robot`` 那段在总表里的点索引。"""
        if robot is None:
            return list(range(len(self.recorded_positions)))
        return [i for i, r in enumerate(self.point_robots) if r == robot]

    def types_of(self, robot: Optional[str]) -> List:
        """``robot`` 那段对应的工艺语义。"""
        return [self._type_at(i) for i in self.indices_of(robot)]

    def set_types_for(self, robot: Optional[str], types) -> None:
        """给 ``robot`` 那段回填工艺语义（切换机器人时恢复用）。"""
        types = list(types or [])
        for k, i in enumerate(self.indices_of(robot)):
            if k < len(types) and types[k] is not None:
                if i < len(self.point_types):
                    self.point_types[i] = types[k]
        self._fill_missing_types()

    def toPlainText(self):
        return self.code_edit.toPlainText()

    def setPlainText(self, text):
        self.code_edit.setPlainText(text)

    def set_joint_names(self, names):
        """设置关节名称顺序，用于解析代码"""
        self.joint_names = list(names)

    #: 段级归属头：出现一次后，其后的指令都归这台，直到下一个 Robot 头出现
    _ROBOT_HEADER_RE = re.compile(r"^#\s*Robot\s*:\s*(.+?)\s*$")

    def _parse_code_to_positions(self):
        """从代码编辑器解析点位，**并记下每条指令的行号与归属**。

        只抓出数值是不够的：「从选中行开始跑」「只跑选中那台」都要求知道每条
        指令在代码里的位置、以及它属于哪台机器人，所以这里逐行扫描，而不是
        对全文 ``findall``——后者会把位置和归属一起丢掉。
        """
        code_text = self.code_edit.toPlainText()
        # 语义不在代码文本里，解析会重建位置列表——先存下来，数量一致就沿用它，
        # 否则用户打的「抓取点 / 放置点」标记会在每次 Play 时被抹掉
        old_types = list(self.point_types)
        self.recorded_positions.clear()
        self.point_robots.clear()
        self._line_map.clear()
        
        # 匹配 movej([value1, value2, ...], vel=X.X) 格式
        pattern = r'movej\(\s*\[([^\]]+)\]\s*(?:,\s*vel\s*=\s*[\d.]+)?\s*\)'
        
        robot = self.owner_robot  # 没有 Robot 头时，整份退回这个归属
        for line_no, line in enumerate(code_text.splitlines(), start=1):
            header = self._ROBOT_HEADER_RE.match(line)
            if header:
                robot = header.group(1)
                continue
            found = re.search(pattern, line)
            if not found:
                continue
            try:
                values = [float(v.strip()) for v in found.group(1).split(',')]
                # 将角度转换为弧度
                rad_values = [math.radians(v) for v in values]
                
                if self.joint_names and len(values) == len(self.joint_names):
                    # 如果有关节名称，创建字典
                    pos_dict = {
                        name: rad_values[i] for i, name in enumerate(self.joint_names)
                    }
                elif len(values) > 0:
                    # 如果没有关节名称，使用索引作为键
                    pos_dict = {f"joint_{i+1}": rad_values[i] for i in range(len(rad_values))}
                else:
                    continue
                # 行号与归属必须与点值同生共死：它们是"从哪行起跑""跑哪台"的依据
                self._line_map.append((line_no, robot, len(self.recorded_positions)))
                self.recorded_positions.append(pos_dict)
                self.point_robots.append(robot)
            except ValueError:
                continue
        
        # 更新位置计数显示（带上各台各多少点）
        self._update_count_label()

        if len(old_types) == len(self.recorded_positions):
            self.point_types = old_types
        else:
            # 点数变了就重建：默认的五点式是**每台各一套**，按整份点数取会
            # 让第二台拿到一串途经点，循环里没有抓取点
            self.point_types = []
            self.sync_point_types()
        self.refresh_scope()

    # ---------------- 播放范围：跑哪台、从哪行起 ---------------- #

    def robot_at(self, index: int) -> Optional[str]:
        """第 ``index`` 个示教点归属哪台机器人（无映射信息时退回整份归属）。"""
        if 0 <= index < len(self._line_map):
            return self._line_map[index][1]
        if 0 <= index < len(self.point_robots):
            return self.point_robots[index]
        return self.owner_robot

    def line_of(self, index: int) -> int:
        """第 ``index`` 个示教点在代码里的行号（1 起）。"""
        if 0 <= index < len(self._line_map):
            return self._line_map[index][0]
        return index + 1

    def _cursor_line(self) -> int:
        """代码光标所在行（1 起）——"从选中行号处运行"的行号来源。"""
        return self.code_edit.textCursor().blockNumber() + 1

    def _selected_robot(self) -> Optional[str]:
        """**当前选中对象**关联的机器人；没选中或关联不到则 ``None``。"""
        provider = getattr(self, "selection_robot_provider", None)
        return provider() if callable(provider) else None

    def playback_plan(self, robot: Optional[str] = None,
                      from_cursor: bool = False) -> List[int]:
        """算出这次播放要走的**点索引序列**（顺序与代码行序一致）。

        Args:
            robot: 只跑这台机器人的指令；``None`` 表示不挑机器人
            from_cursor: 是否从光标所在行开始——未选中设备时的「全局运行」
        """
        seq = list(range(len(self.recorded_positions)))
        if robot:
            # 选中了机器人（或挂在它上面的夹爪/工件）：只跑它的指令，其余整段跳过
            return [i for i in seq if self.robot_at(i) == robot]
        if from_cursor:
            line = self._cursor_line()
            return [i for i in seq if self.line_of(i) >= line]
        return seq

    def refresh_scope(self) -> None:
        """刷新「将要执行的范围」——按下 Play 之前就该知道跑的是谁、从哪开始。"""
        if not hasattr(self, "scope_label"):
            return
        total = len(self.recorded_positions)
        if not total:
            self.scope_label.setText("")
            return
        robot = self._selected_robot()
        if robot:
            seq = self.playback_plan(robot=robot)
            self.scope_label.setText(
                tr("将运行：{robot} · {n}/{total} 点").format(
                    robot=robot, n=len(seq), total=total)
            )
        else:
            seq = self.playback_plan(from_cursor=True)
            self.scope_label.setText(
                tr("将从第 {line} 行起全局运行（{n}/{total} 点）").format(
                    line=self._cursor_line(), n=len(seq), total=total)
            )

    def _flush_code_edit(self) -> None:
        """手改代码停手后：重解析 → 广播，让 3D 路径与面板点表跟着刷新。

        只在**没在播放**时刷：播到一半被解析改写点表，回放序列与点表会错位，
        机器人直接跳到别处。
        """
        if self.playback_timer is not None and self.playback_timer.isActive():
            return
        self._parse_code_to_positions()
        self.pos_count_label.setText(
            tr("示教点：{n}").format(n=len(self.recorded_positions))
        )
        self.refresh_scope()
        # 主窗口已接 on_positions_parsed：视口路径 + 离线编程面板点表
        self.positionsParsed.emit()

    def line_of_point(self, index: int, robot: Optional[str] = None):
        """第 ``index`` 个示教点（属于 ``robot``）在代码里的行号。

        「机器人走到哪一点，代码就高亮到哪一行」靠这条反查：点索引与行号不是
        一回事——圆弧的中间点不单独出指令，手工改过的代码行号还会整体偏移。

        Args:
            index: 点表里的下标（0 起）
            robot: 该点归属的机器人；``None`` 表示只按序号找

        Returns:
            行号（1 起）；查不到返回 ``None``
        """
        self._parse_code_to_positions()
        loose = None
        for ln, owner, point_index in self._line_map:
            if point_index != index:
                continue
            if robot is None or owner == robot:
                return ln
            # 归属对不上（单机代码没有 ``# Robot:`` 头时 owner 是 None）：
            # 先记住，全部找完再用——有且只有一个匹配点时序号就是行号
            loose = ln if loose is None else loose
        if loose is not None:
            return loose
        # 点表里压根没有行号信息（手打的/刚解析的），退回按序号估算
        return self.line_of(index) if index < len(self.recorded_positions) else None

    def _line_target(self, line: int):
        """第 ``line`` 行对应的 ``(机器人名, 点索引)``；不可执行返回 ``None``。

        先解析一遍代码：手改过的文本要先落到 :attr:`_line_map`，行号才对得上。
        """
        self._parse_code_to_positions()
        for ln, robot, index in self._line_map:
            if ln == line:
                return (robot, index)
        return None

    def run_line(self, line: Optional[int] = None) -> bool:
        """把机器人驱动到第 ``line`` 行（缺省光标行）对应的示教点。

        与「播放」的区别：播是从某行**一直走到尾**，这里是**只走这一行**——
        逐行核对抓取/放置时机、检查某点是否可达时最快。

        Args:
            line: 代码行号（1 起）；``None`` 取光标所在行

        Returns:
            执行成功 ``True``；该行不可执行 ``False``（状态栏说明原因）
        """
        if line is None:
            line = self._cursor_line()
        target = self._line_target(line)
        if target is None:
            self._notify(tr("第 {line} 行不是可执行的 movej 指令").format(line=line))
            return False
        robot, index = target
        self.highlight_line(line)
        # 归属跟着点走：多机时每行的主人可能不同，认行不认"当前那台"
        self.playbackRequested.emit(index, robot or "")
        return True

    def _show_code_context_menu(self, pos) -> None:
        """代码区右键菜单：标准编辑项 +「执行此行」。"""
        menu = self.code_edit.createStandardContextMenu()
        line = self.code_edit.cursorForPosition(pos).blockNumber() + 1
        if self._line_target(line) is not None:
            menu.addSeparator()
            menu.addAction(tr("▶ 执行此行"), lambda: self.run_line(line))
        menu.exec(self.code_edit.mapToGlobal(pos))

    def highlight_line(self, line: int) -> None:
        """把光标移到第 ``line`` 行并整行选中——报错时指出是哪一行出的问题。"""
        block = self.code_edit.document().findBlockByNumber(max(0, line - 1))
        if not block.isValid():
            return
        cursor = QtGui.QTextCursor(block)
        cursor.movePosition(QtGui.QTextCursor.EndOfBlock, QtGui.QTextCursor.KeepAnchor)
        self.code_edit.setTextCursor(cursor)


class DevicePropertyPanel(QtWidgets.QWidget):
    objectChanged = Signal()  # 场景对象属性被编辑后触发视口重绘
    #: 关节滑块改动 → 驱动视口/记录。
    #:
    #: 三个参数依次是 **机器人名 / 关节名 / 弧度**。机器人名不能省：单薄的
    #: ``(关节名, 值)`` 只能落到"当前渲染器"身上，而滑块是按**选中那台**的关节
    #: 建的——两者错位时，拖 A 的滑块却把值写进了 B，表现为两台一起动。
    jointChanged = Signal(str, str, float)

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
        #: 当前「关节」滑块所属的机器人名——多机时 :attr:`jointChanged` 要据此
        #: 指名写入，不能再让 MainWindow 猜"当前渲染器"是谁
        self._active_robot: Optional[str] = None
        #: 「按机器人名取渲染器」回调，由主窗口注入 ``gl_widget.renderer_of``。
        #: 没有它，选中 A 也只能沿用一个陈旧渲染器（通常是最后加载那台）。
        self.renderer_provider = None
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
        self.obj_name_label = QtWidgets.QLabel(tr("—"))
        self.obj_type_label = QtWidgets.QLabel(tr("—"))
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
        self.name_label = QtWidgets.QLabel(tr("—"))
        self.manufacturer_label = QtWidgets.QLabel(tr("—"))
        self.type_label = QtWidgets.QLabel(tr("—"))
        self.payload_label = QtWidgets.QLabel(tr("—"))
        self.reach_label = QtWidgets.QLabel(tr("—"))
        self.repeat_label = QtWidgets.QLabel(tr("—"))
        self.dof_label = QtWidgets.QLabel(tr("—"))
        self.controller_label = QtWidgets.QLabel(tr("—"))
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
            ("x", tr("X (毫米)"), -20000, 20000, 10),
            ("y", tr("Y (毫米)"), -20000, 20000, 10),
            ("z", tr("Z (毫米)"), -5000, 10000, 10),
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
    
    def update_robot_info(self, renderer, entry=None, name: Optional[str] = None):
        """更新机器人信息区。

        Args:
            renderer: 已加载 URDF 的渲染器
            entry: 机器人库条目；提供时可回显负载、臂展等完整规格
            name: 这台机器人在场景里的名字。**必须给**：规格要按名字留存，
                否则只会记最后加载那台——多机时选中 A，规格块却显示 B 的
                型号/厂商（2026-09-21 用户反馈）。``None`` 退回旧行为（单机）。
        """
        self._renderer = renderer
        self._has_robot_model = True
        if name is not None:
            # 规格按机器人名各存一份：set_object 选中谁就回显谁的
            if not hasattr(self, "_robot_entries"):
                self._robot_entries = {}
            self._robot_entries[name] = entry
        revolute = [j for j in renderer.joints.values() if j["type"] == "revolute"]
        self.dof_label.setText(str(len(revolute)))
        self._fill_robot_spec(renderer, entry)

        self.log_text.append(tr("已载入: {name}").format(name=renderer.robot_name))
        self.log_text.append(tr("关节数: {n}").format(n=len(revolute)))

    def _fill_robot_spec(self, renderer, entry) -> None:
        """把**指定那台**的规格填进信息区（型号/厂商/负载/臂展…）。

        供两处调用：:meth:`update_robot_info`（载入时）与
        :meth:`set_object`（选中机器人时按名字回显）——后者没有这条，
        多机时规格块永远是最后加载那台的（2026-09-21 用户反馈）。
        """
        revolute = [j for j in renderer.joints.values() if j["type"] == "revolute"]
        self.dof_label.setText(str(len(revolute)))
        if entry is not None:
            self.name_label.setText(entry.model_name)
            self.manufacturer_label.setText(entry.brand_display)
            type_map = {"industrial": tr("工业臂"), "cobot": tr("协作臂"), "scara": "SCARA"}
            self.type_label.setText(type_map.get(entry.robot_type, entry.robot_type))
            self.payload_label.setText(
                f"{entry.payload_kg:g} kg" if entry.payload_kg is not None else tr("—")
            )
            self.reach_label.setText(
                f"{entry.reach_mm:g} mm" if entry.reach_mm is not None else tr("—")
            )
            self.repeat_label.setText(
                f"±{entry.repeatability_mm:g} mm"
                if entry.repeatability_mm is not None
                else tr("—")
            )
            self.controller_label.setText(entry.controller or tr("—"))
            if entry.note:
                prefix = "" if entry.verified else tr("⚠ 参数未核验 — ")
                self.note_label.setText(prefix + entry.note)
                self.note_label.show()
            else:
                self.note_label.hide()
        else:
            self.name_label.setText(renderer.robot_name)
            self.manufacturer_label.setText(tr("—"))
            self.type_label.setText(tr("—"))
            self.payload_label.setText(tr("—"))
            self.reach_label.setText(tr("—"))
            self.repeat_label.setText(tr("—"))
            self.controller_label.setText(tr("—"))
            self.note_label.hide()

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
        # 必须用 is_robot 而不是"名字等于主机器人名"：多机时第二台叫
        # 「机器人_2」，按名字比会让规格块/关节滑块/挂载下拉统统不显示
        is_robot = self.scene.is_robot(name)
        spec = COMPONENTS.get(obj.component_id) if obj.component_id else None

        # 显示「对象属性」容器（内含信息 / 位置 / 参数）
        self.object_scroll.show()
        self.object_group.show()

        self.obj_name_label.setText(name)
        if is_robot:
            self.obj_type_label.setText(tr("机器人"))
            self.robot_spec_widget.setVisible(True)
            # 规格块按**选中的这台**回显：载入时各台的规格已按名字留存
            # （update_robot_info），只回显最后加载那台的话，选中 A 显示的
            # 却是 B 的型号/厂商（2026-09-21 用户反馈）
            self._fill_robot_spec(
                self._renderer_for_robot(name),
                getattr(self, "_robot_entries", {}).get(name),
            )
            self.note_label.setVisible(bool(self.note_label.text()))
            # 机器人可以挂到底座/台面上（基座随之随动）。但「装夹到末端」对
            # 机器人自身没有意义——那是"把工具装到它的法兰上"，故只留下拉。
            self.obj_mount_widget.setVisible(True)
            self.mount_btn_row.setVisible(False)
            self._refresh_parent_combo(name)
        else:
            # 与设备库同一套取值：显示**分类**，组件名由「名称」行承载
            self.obj_type_label.setText(tr(spec.category) if spec else tr("自定义"))
            self.robot_spec_widget.setVisible(False)
            self.note_label.setVisible(False)
            self.obj_mount_widget.setVisible(True)
            self.mount_btn_row.setVisible(True)
            self._refresh_parent_combo(name)

        # 位姿回填（blockSignals 避免触发写回）
        # _matrix_to_rpy 返回弧度，转成角度后再填进 6 个输入框
        # 用**世界坐标**回填：挂载件（如装到底座上的机器人）的局部 pose 是相对
        # 父对象的——两台挂法相同的机器人局部值一模一样（0,0,底座高），看起来
        # "位置永远不变"，也和视口里拖拽落到的位置对不上（2026-09-21 用户反馈）
        tf_matrix = self.scene.world_transform(name)
        xyz = tf_matrix[:3, 3]
        rx, ry, rz = (math.degrees(v) for v in _matrix_to_rpy(tf_matrix))
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
        # 机器人必须按**自己的名字**取渲染器：沿用 :attr:`_renderer`（最后加载
        # 那台）会让选中 A 时滑块仍是 B 的关节组——多机串扰的源头在这里
        robot_renderer = self._renderer_for_robot(name) if is_robot else None
        if is_robot and robot_renderer is not None:
            self._rebuild_param_rows(None, {})
            self._setup_joint_controls(robot_renderer, name)
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

    def set_renderer_provider(self, fn) -> None:
        """注入「按机器人名取渲染器」的回调（多机必需）。

        Args:
            fn: ``Callable[[str], RobotRenderer]``；主窗口传
                ``gl_widget.renderer_of``

        Note:
            不注入的话，本面板只能沿用 :attr:`_renderer`（由
            :meth:`update_robot_info` 写入，通常是**最后加载**那台）。于是选中
            A 时滑块仍是 B 的关节组——拖动会把值写进别的机器人，两台一起动。
        """
        self.renderer_provider = fn

    def _renderer_for_robot(self, name: str):
        """按名字取该机器人的渲染器；取不到时退回 :attr:`_renderer`。"""
        if self.renderer_provider is not None:
            try:
                return self.renderer_provider(name)
            except Exception:
                return None
        return self._renderer

    def _setup_joint_controls(self, renderer, robot_name: Optional[str] = None) -> None:
        """为机器人在「关节」面板内构建 6 轴滑块。

        Args:
            renderer: 渲染器（取关节清单与初值）
            robot_name: 这台机器人的**名字**。滑块回调要记下来一起发出去——
                否则 MainWindow 无从判断该把关节值写到哪一台
        """
        self._renderer = renderer
        self._active_robot = robot_name
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
        # 带上所属机器人名：多机时 MainWindow 靠它定位该写哪台渲染器
        self.jointChanged.emit(self._active_robot or "", name, math.radians(angle))

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
        # 输入框的语义与显示一致：**世界坐标**。挂载件必须先经父系逆变换换回
        # 局部 pose 再落盘——直接把世界值当局部值写，对象会被"瞬移"到父系
        # 原点附近（世界系 → 局部系的换算口径与 placement.apply_world_pose 一致）
        world = rpy_to_matrix([
            math.radians(self.obj_pose_widgets["rx"].value()),
            math.radians(self.obj_pose_widgets["ry"].value()),
            math.radians(self.obj_pose_widgets["rz"].value()),
        ])
        world[:3, 3] = [
            self.obj_pose_widgets["x"].value(),
            self.obj_pose_widgets["y"].value(),
            self.obj_pose_widgets["z"].value(),
        ]
        parent_tf = self.scene.parent_transform(self.current_object)
        local = np.linalg.inv(parent_tf) @ world
        lx, ly, lz = (float(v) for v in local[:3, 3])
        lrx, lry, lrz = (math.degrees(v) for v in _matrix_to_rpy(local))
        obj.pose = Pose.from_xyz_rpy(lx, ly, lz, lrx, lry, lrz)
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
            # 挂到**任意一台**机器人（不只是主机器人）的法兰都要走装夹对齐
            self.scene.attach(
                name, target, keep_world=(target not in self.scene.robots)
            )
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
        #: 各机器人**抓取的工件**，键为机器人名。理由同上：工件下拉也是全局
        #: 一份，多机同时回放时不分开就会两台抢同一个工件（A 刚夹起就被 B 抢走）
        self._workpiece_by_robot: dict = {}
        #: 各机器人的**示教点语义**（PICK/PLACE…），键为机器人名。
        #: 路径编辑器一次只显示一台的点表，它的 ``point_types`` 就是"当前那台"
        #: 的；切换机器人时必须把旧的存进来、新的取出来，否则 B 会顶着 A 的
        #: 抓取语义出码——B 的程序里根本没有夹爪指令。
        self._point_types_by_robot: dict = {}
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
        self._focus_tab(self.scene_panel)

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

        # 先接管场景：机器人的名字以**场景**为准（与"抓取工作站演示"同口径），
        # 顺序反过来会让视口按追加分到另一个名字（详见那里的注释）
        self._adopt_scene(scene)

        # 场景里的机器人可能与当前视口的不同——让它加载场景里那台
        if scene.robot_urdf and Path(scene.robot_urdf).exists():
            name = scene.robot_names[0] if scene.robot_names else ROBOT_NAME
            self.gl_widget.load_robot(scene.robot_urdf, name)
            if scene.has_robot:
                self.gl_widget.set_joint_values(
                    {n: math.radians(v) for n, v in zip(
                        self.gl_widget.renderer.actuated_names(),
                        scene._robot_joints_deg)}
                )
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

        # 1) 先接管模板场景 → 再**认领**里面的机器人（顺序是不能反的关键）
        #
        #    反过来的话：load_robot 会在**旧场景**里按"追加"分到下一个名字
        #    （机器人_2），随后 _adopt_scene 换上模板场景，名字又从 机器人 开始
        #    编号——视口与离线编程面板手里是旧名字、场景里是新名字，于是：
        #    ① 回放吸附工件时 attach 到一个不存在的父对象（工件不随动）；
        #    ② 存工程时多出一台"幽灵机器人"的程序段，重开后当前那台看不到点。
        #    先换场景再认领，两边同名，这类错位从根上不成立。
        self._adopt_scene(cell.scene)
        self.load_robot(urdf, adopt_existing=True)
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

        # 3) 灌示教点：**走和手工示教同一条通路**（set_recorded_positions 带归属）。
        #    之前这里是直接给 recorded_positions 赋值，点有了但没归属，
        #    _update_code_with_positions 里 segments() 空、代码一个 movej 都不写——
        #    点数看着 8 个，存盘再打开发现一个点都没有（program 段只有两行注释）。
        #    场景已在第 1 步接管。
        self.code_editor.set_recorded_positions(
            positions, owner=self.gl_widget.current_robot
        )
        self.code_editor.point_types = list(types)  # set 后会上一套默认值，这里盖回去
        self.code_editor.pos_count_label.setText(tr("示教点：{n}").format(n=len(positions)))
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
            # 存**整份**语义：代码里装着所有机器人的段，只存"当前编辑那台"
            # 一段的话，载入时点数对不上，只能全部按默认语义重建（抓取点丢失）
            types = [
                getattr(t, "value", str(t))
                for t in (self.code_editor.point_types or [])
            ]
            project.set("program", {
                "code": self.code_editor.toPlainText(),
                "point_types": types,
                "joint_names": list(self.code_editor.joint_names or []),
            })

        # 多机：每台机器人**各自**的示教点与抓取语义。上面那段 ``program`` 只
        # 装得下当前那台（路径编辑器一次只显示一台），多机工程存盘再打开会丢掉
        # 其余机器人教好的抓取循环——抓取编程必须存得住。
        if _HAS_PROGRAM and hasattr(self, "program_panel"):
            snapshot = self.program_panel.robot_snapshot()
            for name, payload in snapshot.items():
                # 工件归属一并存：谁抓哪件是工艺的一部分
                payload["workpiece"] = self._workpiece_by_robot.get(name)
            if snapshot:
                project.set("programs", snapshot)

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
                    # 场景必须先落地：机器人是随场景一起恢复的，认领它要在
                    # adopt 之后——否则 load_robot 操作的是**旧的**场景，会往
                    # 里面再追加一台默认名的机器人，多机工程于是凭空多一台
                    self._adopt_scene(scene)
                    if urdf and Path(urdf).exists():
                        # 只认领，不追加
                        self.load_robot(urdf, adopt_existing=True)
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
                    if _HAS_PROGRAM and hasattr(self, "program_panel"):
                        self.program_panel.set_scene(scene)
                        # 场景里**每台**机器人都要登记：只登记主机器人的话，
                        # 其余机器人的抓取程序就成了没有主人的孤儿
                        self._restore_robot_programs(scene)

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
                # 按归属回灌：代码里的 ``# Robot:`` 段头已经写明了每段属于谁
                self._sync_viewport_paths_from_editor()
                self._sync_program_points()

        # ---- 多机：每台机器人各自的示教点与抓取语义 ---- #
        payload = project.get("programs")
        if payload is not None:
            if not (_HAS_PROGRAM and hasattr(self, "program_panel")):
                issues.append(tr("多机示教点段已跳过：离线编程模块不可用"))
            else:
                self.program_panel.restore_robots(payload)
                for name, data in payload.items():
                    data = data or {}
                    if data.get("workpiece"):
                        self._workpiece_by_robot[name] = data["workpiece"]
                    positions = data.get("positions") or []
                    if positions:
                        # 视口也要按名回填：路径可视化与各台的点表都从它读
                        self.gl_widget.set_recorded_positions(positions, name)

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
        
        file_menu = menubar.addMenu(tr("文件(&F)"))
        open_action = QAction(tr("打开模型(&O)…"), self)
        open_action.setShortcut(QKeySequence.Open)
        open_action.triggered.connect(self.open_model)
        file_menu.addAction(open_action)
        
        save_action = QAction(tr("保存路径代码(&S)…"), self)
        save_action.setShortcut(QKeySequence.Save)
        save_action.triggered.connect(self.save_path_code)
        file_menu.addAction(save_action)
        
        load_action = QAction(tr("载入路径代码(&L)…"), self)
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

        gen_action = QAction(tr("生成机器人代码(&G)…"), self)
        gen_action.setShortcut("Ctrl+G")
        gen_action.triggered.connect(self.generate_robot_code)
        file_menu.addAction(gen_action)

        # 真机连接（Socket / SSH）：从未上过真机验证，MVP 不敢当产品能力放出去
        if _HAS_CONTROLLER_PANEL and _feature_enabled("controller_link"):
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
        
        exit_action = QAction(tr("退出(&X)"), self)
        exit_action.setShortcut(QKeySequence.Quit)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        view_menu = menubar.addMenu(tr("视图(&V)"))
        reset_view = QAction(tr("重置视图(&R)"), self)
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
            # 多机时一台一台切过去按 F7 太笨：一键把全部机器人校完并汇总
            validate_all_prog = QAction(tr("校验全部机器人"), self)
            validate_all_prog.setShortcut("Shift+F7")
            validate_all_prog.triggered.connect(self.validate_all_programs)
            view_menu.addAction(validate_all_prog)
            grasp_wizard = QAction(tr("抓取向导…"), self)
            grasp_wizard.setShortcut("Ctrl+Shift+G")
            grasp_wizard.triggered.connect(self.open_grasp_wizard)
            view_menu.addAction(grasp_wizard)

        # 插件生态未成型，属开发者项
        if _HAS_PLUGINS and _feature_enabled("plugins"):
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

        # 缓存状态 / 清空缓存：纯开发者项
        if _HAS_INCREMENTAL and _feature_enabled("dev_tools"):
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
        # 编辑器看不到视口选中状态：把"选中对象关联到哪台"的解析交给它，
        # 「只跑选中机器人」与「从选中行号处全局运行」这两个分支才做得出来
        self.code_editor.selection_robot_provider = self._robot_for_selection
        self.code_editor.playbackRequested.connect(self.on_playback_requested)
        self.code_editor.playbackStopped.connect(self.on_playback_stopped)
        self.code_editor.clearRequested.connect(self.on_clear_requested)
        self.code_editor.positionsParsed.connect(self.on_positions_parsed)
        # 工件下拉是全局一份，但"谁抓哪件"是**每台机器人**的事。选中的当下就
        # 记进本机名下——只在切换机器人时补记是不够的：用户给 A 选完工件若没
        # 切走过，A 名下就是空的，多机回放时两台会去抢同一件工件。
        if hasattr(self.code_editor, "workpiece_combo"):
            self.code_editor.workpiece_combo.currentIndexChanged.connect(
                self._on_workpiece_combo_changed
            )

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
                # 面板要校验/回放前先要点最新数据（它的槽可能是旧快照）
                self.program_panel.syncRequested.connect(
                    self._on_program_sync_requested
                )
                # 「生成代码」按回放同一套口径：选中设备 → 只出那台的；
                # 没选中 → 出全部。探针由主窗口注入（面板看不到视口选中）
                self.program_panel.selection_robot_provider = (
                    self._robot_for_selection
                )
                # 校验会把场景关节短暂写走再恢复（见面板 _restore_scene_joints）：
                # 恢复之后视口得重画一次，夹爪才不会停留在半路
                self.program_panel.reportReady.connect(
                    lambda _report: self.gl_widget.update()
                )
                # 面板顶部「机器人」下拉换了人 → 视口/示教点显示同步到那台
                self.program_panel.activeRobotChanged.connect(
                    self._on_program_robot_changed
                )

            # ---- 左：双 Tab（① 场景设备库 ② 代码预览）----
            # 左手边是"输入区"：Tab1 从库里挑设备拖进场景，Tab2 是紧贴视口的代码
            # 预览——看着机器人动、同时读/改 movej 那几行，改完当场看得见效果，
            # 不用在窗口两头来回横跳。设备树已挪到右侧（见下）。
            self.left_tabs = QtWidgets.QTabWidget()
            self.left_tabs.setMaximumWidth(400)
            self.left_tabs.setMinimumWidth(280)
            self.left_tabs.addTab(self.library_panel, tr("场景设备库"))
            self.left_tabs.addTab(self.code_editor, tr("代码预览"))
            main_layout.addWidget(self.left_tabs)
        else:
            # 无机器人库时左侧只剩代码预览；设备树放在右侧那组里。一个面板只能挂
            # 一次——在两边都挂的话会被 reparent 到后挂的那组，先挂的位置就空了。
            self.left_tabs = QtWidgets.QTabWidget()
            self.left_tabs.setMaximumWidth(400)
            self.left_tabs.setMinimumWidth(280)
            self.left_tabs.addTab(self.code_editor, tr("代码预览"))
            main_layout.addWidget(self.left_tabs)
        
        self.gl_widget = GLWidget(self)
        self.gl_widget.setMinimumSize(400, 300)
        main_layout.addWidget(self.gl_widget, 1)
        
        self.property_panel = DevicePropertyPanel()
        self.property_panel.jointChanged.connect(self.on_joint_changed)
        self.property_panel.jointChanged.connect(self.on_joint_changed_for_record)
        # 多机：给属性面板「按名字取渲染器」的能力。不给它，选中 A 时面板只能
        # 沿用最后加载那台的关节组——拖 A 的滑块却驱动 B（两台一起动）。
        self.property_panel.set_renderer_provider(self.gl_widget.renderer_of)

        # ---- 右：可折叠多功能标签页（结构 + 输出）----
        # 属性在最前（选中对象时自动切到它），设备树紧随其后：层级编排和属性本来
        # 就是同一件事的两面——树上点一下选中、这里改位姿，左右手不用换地方。
        self.right_tabs = QtWidgets.QTabWidget()
        self.right_tabs.setMaximumWidth(360)
        self.right_tabs.setMinimumWidth(260)
        self.right_tabs.addTab(self.property_panel, tr("属性"))
        if _HAS_WORLD and hasattr(self, "scene_panel"):
            self.right_tabs.addTab(self.scene_panel, tr("设备树"))
        if _HAS_PROGRAM and hasattr(self, "program_panel"):
            self.right_tabs.addTab(self.program_panel, tr("导出打包"))
        main_layout.addWidget(self.right_tabs)

        # 场景改动需要触发视口重绘，此连接必须等 gl_widget 建好后才能做
        if _HAS_WORLD and hasattr(self, "scene_panel"):
            self.scene_panel.sceneChanged.connect(self.gl_widget.update)
            # 「加载场景」整体替换场景引用时统一接管：视口/属性面板/工件下拉
            # 各持一份引用，漏掉任何一处都会出现"树里是新的、别处还是旧的"
            self.scene_panel.sceneReplaced.connect(self._adopt_scene)
            # 视口右键拾取/拖动选中对象时，同步到左侧场景面板树形选择
            self.gl_widget.objectSelected.connect(self.scene_panel._select_object)
            # 视口右键路径点「移动到」：把那一台驱动到该示教点
            self.gl_widget.pathPointMoveRequested.connect(
                self.on_path_point_move_requested
            )
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
                    # 选中一变，"跑哪台"就变了，范围提示要跟着换
                    self.code_editor.refresh_scope(),
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
            # 校验才能查出"序列用了但点表里没有"的信号。
            # MVP 档没有时序编辑器（入口隐藏），这段连接一并跳过
            _seq_editor = getattr(self.scene_panel, "sequence_editor", None)
            if _seq_editor is not None:
                _seq_editor.set_available_signals(
                    sorted({sig for d in self.code_editor.cell.devices for sig in d.io_map}
                           | set(HANDSHAKE_SIGNALS))
                )
            # 工程未保存标记（v1.3.4）：会话里任何"值得存下来"的改动都置脏。
            # 刻意只挂这四条粗粒度信号——细粒度（每个对象位姿变化）噪声太大，
            # 会把标记变成常亮的装饰。
            self.scene_panel.sceneChanged.connect(self._set_project_dirty)
            # 机器人被移出场景 → 它的程序/轨迹/示教点/渲染器一并清除
            self.scene_panel.robotRemoved.connect(self._on_robot_removed)
            if _seq_editor is not None:
                _seq_editor.sequenceChanged.connect(self._set_project_dirty)
            self.code_editor.code_edit.textChanged.connect(self._set_project_dirty)

        if _HAS_PROGRAM and hasattr(self, "program_panel"):
            self.program_panel.taughtPointsChanged.connect(
                lambda *_args: self._set_project_dirty()
            )

        self._setup_status_bar()
        if _HAS_PROGRAM and hasattr(self, "program_panel"):
            self.program_panel.reportReady.connect(self._on_report_ready)
            # 一键校验全部完成 → 出汇总
            if hasattr(self.program_panel, "allValidated"):
                self.program_panel.allValidated.connect(self._show_validation_summary)
            # 抓取向导写完点 → 按机器人名回灌视口
            if hasattr(self.program_panel, "graspCycleApplied"):
                self.program_panel.graspCycleApplied.connect(
                    self._on_grasp_cycle_applied
                )

        self.statusBar().showMessage(tr("就绪"))
    
    def open_model(self):
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open Robot Model", "models", "URDF Files (*.urdf);;All Files (*)"
        )
        if file_path:
            self.load_robot(file_path)
    
    def _register_robot_in_scene(self, path, adopt_existing: bool = False) -> str:
        """把机器人登记进工作单元场景：**已有机器人时追加**，返回其名字。

        回归：此前一律走 ``Scene.set_robot``，名字固定为 ``ROBOT_NAME``，于是
        "再增加一台机器人"实际是把已有的那台顶掉。

        Args:
            path: URDF 路径
            adopt_existing: ``True`` 表示**认领**场景里已有的机器人（工程载入
                用），不再追加。工程载入时机器人已经随场景一起恢复了，再追加
                一台就会凭空多出一个默认名的机器人

        Returns:
            场景中的机器人名
        """
        scene = self.scene_panel.scene
        if scene.robot_names and adopt_existing:
            return scene.robot_names[0]
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

    def load_robot(self, path, adopt_existing: bool = False):
        """加载 URDF 到 3D 视口，并同步属性面板。

        若该文件属于机器人库中的某个型号，属性面板会回显负载、臂展等完整规格。

        Args:
            path: URDF 路径
            adopt_existing: ``True`` 表示机器人**已在场景里**（工程载入），只
                认领不再追加
        """
        # 先决定"追加、认领还是替换"：场景已有机器人时必须用 add_robot（唯一名），
        # 否则 set_robot 的固定名 ROBOT_NAME 会把已有那台顶掉——"再加一台"
        # 就变成了"替换"。名字要先定，渲染器才能登记到对应名下。
        robot_name = ROBOT_NAME
        if _HAS_WORLD and hasattr(self, "scene_panel"):
            robot_name = self._register_robot_in_scene(path, adopt_existing)

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

        # 规格必须按场景里的机器人名留存：多机时选中谁就回显谁的型号/厂商，
        # 否则永远显示最后加载那台（2026-09-21 用户反馈）
        self.property_panel.update_robot_info(
            self.gl_widget.renderer, entry, name=robot_name
        )
        
        # 设置关节名称顺序，用于解析路径代码
        joint_names = list(self.gl_widget.renderer.joints.keys())
        self.code_editor.set_joint_names(joint_names)
        
        # 按归属回灌：整份灌进来的话，新加的这台会凭空"继承"别人教好的路径
        self._sync_viewport_paths_from_editor()
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
                # 必须按**场景里的机器人名**登记：多机同型号时 URDF 文件名相同
                # （都是 ur5e），按它做键会把两台的程序合并成一份。
                self.program_panel.set_robot(
                    self.olp_robot, joint_names, name=robot_name
                )
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

    def _register_robot_for_programming(self, name: str, urdf: str) -> bool:
        """把场景里**已有**的 ``name`` 登记进离线编程面板（**不碰场景**）。

        工程载入时机器人已经在场景里了，此时不能走 :meth:`load_robot`——它会调
        ``_register_robot_in_scene`` 再加一台，固定名 ``ROBOT_NAME`` 还会把已有
        那台顶掉。这里只做"登记"，不改动场景。

        Returns:
            登记成功 ``True``；模型不可用或未启用编程模块返回 ``False``
        """
        if not (_HAS_PROGRAM and hasattr(self, "program_panel")):
            return False
        if not urdf or not Path(urdf).exists():
            return False
        try:
            from orolp import Robot as OrolpRobot

            robot = OrolpRobot(str(urdf), Path(urdf).stem)
        except Exception as exc:  # pragma: no cover - 模型损坏时不能带崩载入
            print(f"[warning] 无法为 {name} 准备运动学: {exc}")
            return False
        # 渲染器按名懒加载：取不到会给它一台专属空渲染器，绝不借用别人的
        renderer = self.gl_widget.renderer_of(name)
        joint_names = (
            list(renderer.joints.keys())
            if renderer is not None and getattr(renderer, "joints", None)
            else list(robot.joint_names)
        )
        self.program_panel.set_robot(robot, joint_names, name=name)
        return True

    def _restore_robot_programs(self, scene) -> None:
        """工程载入：把场景里每台机器人登记进编程面板。

        只登记主机器人是不够的——多机工程里第二台若没登记，它的抓取程序就
        "没有主人"，既不能校验也不能出码。
        """
        if not (_HAS_PROGRAM and hasattr(self, "program_panel")):
            return
        for name in scene.robot_names:
            state = scene.robots.get(name)
            urdf = getattr(state, "urdf", None) if state is not None else None
            if urdf and not Path(urdf).exists():
                alt = _REPO_ROOT / urdf
                if alt.exists():
                    urdf = str(alt)
            self._register_robot_for_programming(name, urdf or "")

    def _sync_program_points(self, robot: Optional[str] = None) -> None:
        """把某台机器人（缺省当前）的示教点同步给离线编程面板。

        工艺语义（抓取/放置点）一并传过去：程序面板据此在对应位置插入
        ``gripper()`` 指令，回放动画与出码共用同一份抓取时序（S3）。

        Args:
            robot: 机器人名；``None`` 取当前机器人。给**当前**这台同步时沿用
                路径代码编辑器的数据（轨迹编辑器改点后先回写到它那里），其余
                机器读视口里各自那一串——这样给 A 示教的点不会算进 B 的程序。
        """
        if not (_HAS_PROGRAM and hasattr(self, "program_panel")):
            return
        current = getattr(self.gl_widget, "current_robot", None)
        key = robot or current
        if key == current:
            # 只给这台自己那一段：点表里可能还装着别的机器人的路径，全拿过去
            # 会让 A 的程序里混进 B 的点
            positions, types = self._editor_points_for(key)
            # 顺手归档本机语义：路径编辑器那份随时会被别的机器人顶掉，只在
            # "切换时"补记的话，用户给 A 示教完就直接「导出全部」，A 名下还是空的
            if types and len(types) == len(positions):
                self._point_types_by_robot[key] = list(types)
        else:
            # 非当前那台：读它自己那份语义（切换时存进 _point_types_by_robot 的）
            positions = self.gl_widget.get_recorded_positions(key)
            types = (
                list(self._point_types_by_robot.get(key) or [])
                if _HAS_PICKPLACE
                else None
            ) or None
        if types is not None and len(types) != len(positions):
            types = None
        self.program_panel.set_taught_points(positions, types, robot=key)

    def _editor_points_for(self, key: Optional[str]):
        """路径编辑器里属于 ``key`` 的点与语义。

        旧工程 / 单机器人那份点表没有归属标记（``point_robots`` 全是 ``None``），
        此时整份都算当前这台的——与改归属之前的行为一致。
        """
        ed = self.code_editor
        if key and key in ed.point_robots:
            return ed.points_of(key), (ed.types_of(key) if _HAS_PICKPLACE else None)
        if any(r is not None for r in ed.point_robots):
            return [], ([] if _HAS_PICKPLACE else None)  # 已分段，这台名下没有点
        return list(ed.recorded_positions), (
            list(ed.point_types) if _HAS_PICKPLACE else None
        )

    def _sync_viewport_paths_from_editor(self) -> None:
        """把路径编辑器的点表**按归属**回写视口各机器人的路径桶。

        单机时代的写法是"整份灌进当前机器人"（``set_recorded_positions`` 不指名
        就写 ``current_robot``）。多机下它就是一台复制机：加载新机器人时
        ``current_robot`` 已被切成新名字，于是**别人**教好的点被整份写进新机器
        人名下、画在它旁边；等原主人被删掉，那份副本既不在代码里也回放不了，
        变成屏幕上的幽灵路径。

        这里按每条指令的归属（``robot_at``）分桶回写；没有归属标记的旧点表
        整份退回**当前**那台，与旧行为一致。
        """
        positions = self.code_editor.get_recorded_positions()
        if not positions:
            return
        by_robot: dict = {}
        for i, pos in enumerate(positions):
            key = self.code_editor.robot_at(i) or self.gl_widget.current_robot
            by_robot.setdefault(key, []).append(pos)
        for key, pts in by_robot.items():
            self.gl_widget.set_recorded_positions(pts, key)
        # 语义同样按段归档，否则导出程序的夹爪时序会串到别的机器人上
        for key in by_robot:
            self._point_types_by_robot[key] = self.code_editor.types_of(key)
        self.gl_widget.update()

    def _on_robot_removed(self, name: str) -> None:
        """机器人被移出场景：把属于它的**全部**派生状态清掉。

        只从场景里删对象会留下一堆孤儿状态：离线编程面板还挂着这台的程序
        （照样能校验、能出码，却已经没有对应的机器人）、视口还留着它的渲染器
        与示教点、路径可能还在画。这里一并收回。

        Args:
            name: 被移除的机器人名
        """
        if not name:
            return
        if _HAS_PROGRAM and hasattr(self, "program_panel"):
            self.program_panel.remove_robot(name)

        # 视口：渲染器与示教点（路径可视化也会随之消失，见 _path_robots）
        self.gl_widget.renderers.pop(name, None)
        self.gl_widget._empty_renderers.pop(name, None)
        self.gl_widget.clear_recorded_positions(name)
        self._point_types_by_robot.pop(name, None)
        self._workpiece_by_robot.pop(name, None)

        # 路径编辑器：只删**它那一段**，其余机器人的段原样留着。
        #
        # 以前是"被删的那台正好是当前台才整表清空"：既漏掉了它**不是**当前台的
        # 情况（段留在代码里，回放能跑到一台已经不存在的机器人），又会在它是
        # 当前台时把别人教好的路径一起抹掉——视口还画着（分桶没动）、代码与
        # 回放里却没了，正是那条形同幽灵的路径。
        self.code_editor.set_recorded_positions([], name)
        if getattr(self.code_editor, "owner_robot", None) == name:
            # 没有段头的旧点表靠它找归属；指着一个已删的名字会让后续播放直接
            # 判"机器人已不在场景"而停表
            self.code_editor.owner_robot = None

        if self.gl_widget.current_robot == name:
            # 编辑视角还停在被删那台上：挪到剩下的一台
            remaining = []
            if _HAS_WORLD and hasattr(self, "scene_panel"):
                remaining = self.scene_panel.scene.robot_names
            if remaining:
                self.gl_widget.set_current_robot(remaining[0])

        self.gl_widget.update()

    def _workpiece_for(self, robot: Optional[str] = None) -> Optional[str]:
        """``robot``（缺省当前）这台机器人抓的工件；没指定过则退回下拉当前值。

        工件下拉是全局一份，两台机器人各抓各的。回放/吸附时**必须**按机器人
        解析，不能直接用"下拉里当前显示的那件"——否则 A 的夹紧事件会把 B 选中
        的工件抓起来，看起来就是"两台抢同一件"。
        """
        key = robot or getattr(self.gl_widget, "current_robot", None)
        if key is not None and key in self._workpiece_by_robot:
            return self._workpiece_by_robot.get(key)
        return self.code_editor.get_workpiece() if _HAS_PICKPLACE else None

    def _on_workpiece_combo_changed(self, _index: int) -> None:
        """工件下拉换了件：记到**当前机器人**名下。

        切换机器人时也会走这里（下拉被回填成那台的工件），此时写回的是同一个
        值——幂等，不会把刚取出来的覆盖成错的。
        """
        if not _HAS_PICKPLACE or not hasattr(self.code_editor, "workpiece_combo"):
            return
        key = getattr(self.gl_widget, "current_robot", None)
        if not key:
            return
        self._workpiece_by_robot[key] = self.code_editor.workpiece_combo.currentData()

    def _on_program_robot_changed(self, name: str) -> None:
        """离线编程面板切换了机器人：视口与示教点显示都跟到那台上去。

        不同步就会出现"在 A 名下编辑、视口却跟着 B 动"——下拉只是换了数据槽，
        视口的当前机器人、路径编辑器里显示的点位都得一起换。
        """
        if not name:
            return
        # 切走之前先把**原来那台**的抓取语义存起来——路径编辑器的语义是按段
        # 存的，不归档就会被新那台的段顶掉
        prev = self.gl_widget.current_robot
        if prev and prev != name:
            self._point_types_by_robot[prev] = list(
                self.code_editor.get_point_types(prev) or []
            )
            self._workpiece_by_robot[prev] = self.code_editor.get_workpiece()
        self.gl_widget.set_current_robot(name)
        if (
            _HAS_WORLD
            and hasattr(self, "scene_panel")
            and name in self.scene_panel.scene.objects
        ):
            self.gl_widget.selected_object = name
            if hasattr(self, "property_panel"):
                self.property_panel.set_object(name)

        # 换成这台自己的示教点**与抓取语义**：只替它那一段，别的机器人的
        # 路径留在表里（都显示在代码预览里，各带自己的 Robot 头）
        positions = self.gl_widget.get_recorded_positions(name)
        self.code_editor.set_recorded_positions(positions, name)
        saved = self._point_types_by_robot.get(name)
        if saved is not None:
            self.code_editor.set_types_for(name, saved)
        # 抓取对象同样换成本台的（存过才动，没存过沿用当前选择）
        if name in self._workpiece_by_robot:
            combo = getattr(self.code_editor, "workpiece_combo", None)
            if combo is not None:
                idx = combo.findData(self._workpiece_by_robot[name])
                if idx >= 0:
                    combo.setCurrentIndex(idx)
        # 长度要与示教点对齐，多退少补（缺的按默认语义补）
        self.code_editor.sync_point_types()
        self.code_editor.pos_count_label.setText(tr("示教点：{n}").format(n=len(positions)))
        self.code_editor._update_code_with_positions()
        self._sync_program_points(name)
        self.gl_widget.update()

    def on_taught_points_changed(self, positions, types, motions) -> None:
        """轨迹编辑器改动示教点后回写路径代码编辑器（v0.2.4）。

        两条编辑通路共用同一份示教点数据：面板里增删改了路点，左侧
        代码预览的 movej 文本与点位计数必须同步，否则下次 Play 会用旧数据。

        **必须按机器人名回灌**：整机替换（``owner=None``）会把点表里的
        ``# Robot:`` 归属标记抹成 ``None``——其余机器人的点当场从表里蒸发，
        代码里再也看不见他们的 movej。这也是"删了/移了别人的点"的源头。
        """
        if not hasattr(self, "code_editor"):
            return
        robot = getattr(self.program_panel, "active_robot", None)
        positions = list(positions)
        self.code_editor.set_recorded_positions(positions, robot)
        if _HAS_PICKPLACE and types is not None:
            # 语义跟着新行序回填：移动过的点带着自己的"抓取/放置"一起走
            for k, i in enumerate(self.code_editor.indices_of(robot)):
                if k < len(types) and i < len(self.code_editor.point_types):
                    self.code_editor.point_types[i] = types[k]
        self.code_editor.pos_count_label.setText(
            tr("示教点：{n}").format(n=len(self.code_editor.recorded_positions))
        )
        self.code_editor._update_code_with_positions()
        # 视口路径同步**这一台**那一段：否则 3D 里画的还是移动前的折线顺序
        self.gl_widget.set_recorded_positions(positions, robot)
        self.gl_widget.update()

    def _on_program_sync_requested(self, name: str) -> None:
        """离线编程面板要动数据了：按名字把最新示教点回灌给它。

        面板只在"记录点 / 切换机器人"这类时刻被同步过，它的槽位可能是旧快照
        （例如先给 A 教完点、再把下拉切到自己还没教点的 C）。此时点「校验程序」
        会用空数据校一遍，看起来就像按钮没反应。这里补一次回灌，让点面板自己
        的按钮和按 F7 走同一份数据。
        """
        if not (_HAS_PROGRAM and hasattr(self, "program_panel")):
            return
        if name and name not in self.program_panel.robot_names:
            return
        if name:
            state = self.program_panel._state(name)
            if state is not None and state.taught:
                # 面板那台已经有内容——**绝不**用别处的版本覆盖它。示教点有两条
                # 通路（面板直接 set_taught_points、以及主窗口回灌），合并规则说
                # 不清时宁可不动：覆盖掉用户刚教的点比"少同步一次"严重得多
                return
        self._sync_program_points(name or None)

    def validate_program(self) -> None:
        """校验程序并给出节拍（F7）。"""
        if not (_HAS_PROGRAM and hasattr(self, "program_panel")):
            QtWidgets.QMessageBox.information(self, tr("程序校验"), tr("离线编程模块不可用。"))
            return
        self._focus_tab(self.program_panel)
        # 校验的是**视口当前正在看的那台**：否则会出现"选了 B、却校了 A"
        current = getattr(self.gl_widget, "current_robot", None)
        if current and current in self.program_panel.robot_names:
            self.program_panel.set_active_robot(current)
        self._sync_program_points()
        self.program_panel.validate_program()

    def validate_all_programs(self) -> None:
        """一键校验**全部**机器人（Shift+F7）。

        多机时一台一台切过去按 F7 太笨——切走就把上一台的报告刷掉了，前面几台
        校出问题等于没看见。这里逐台校完再汇总。
        """
        if not (_HAS_PROGRAM and hasattr(self, "program_panel")):
            QtWidgets.QMessageBox.information(self, tr("程序校验"), tr("离线编程模块不可用。"))
            return
        if not getattr(self.program_panel, "validate_all", None):
            QtWidgets.QMessageBox.information(self, tr("程序校验"), tr("当前版本不支持批量校验。"))
            return
        self._focus_tab(self.program_panel)
        self._sync_program_points()
        reports = self.program_panel.validate_all()
        if not reports:
            QtWidgets.QMessageBox.information(
                self, tr("校验全部机器人"),
                tr("还没有登记任何机器人——请先载入机器人型号。"),
            )

    def _show_validation_summary(self, reports: dict) -> None:
        """一键校验全部的汇总：每台一行，哪台有问题一眼看见。

        汇总只给结论，详情仍走编程面板——点「看这台」切过去并显示它的报告。
        """
        if not reports:
            return
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle(tr("校验全部机器人"))
        dlg.setMinimumSize(560, 320)
        layout = QtWidgets.QVBoxLayout(dlg)

        table = QtWidgets.QTableWidget(len(reports), 5, dlg)
        table.setHorizontalHeaderLabels(
            [tr("机器人"), tr("结果"), tr("错误"), tr("警示"), tr("节拍 (s)")]
        )
        table.verticalHeader().setVisible(False)
        table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        table.horizontalHeader().setStretchLastSection(True)

        red = QtGui.QColor("#ff6b6b")
        green = QtGui.QColor("#6fdc6f")
        orange = QtGui.QColor("#ffb86b")
        names = list(reports)
        for row, name in enumerate(names):
            report = reports[name]
            cells = [name]
            if report is None:
                cells += [tr("没程序（未示教）"), "—", "—", "—"]
                color = orange
            elif report.ok:
                cells += [tr("通过"), "0", str(len(report.warnings)),
                          f"{report.cycle_time:.2f}" if report.cycle_time is not None else "—"]
                color = green if not report.warnings else orange
            else:
                cells += [tr("不通过"), str(len(report.errors)), str(len(report.warnings)),
                          f"{report.cycle_time:.2f}" if report.cycle_time is not None else "—"]
                color = red
            for col, text in enumerate(cells):
                item = QtWidgets.QTableWidgetItem(text)
                if col == 1:
                    item.setForeground(color)
                table.setItem(row, col, item)
        table.resizeColumnsToContents()
        table.selectRow(0)
        layout.addWidget(table, 1)

        btn_row = QtWidgets.QHBoxLayout()
        btn_row.addStretch(1)
        btn_view = QtWidgets.QPushButton(tr("看这台"))
        btn_close = QtWidgets.QPushButton(tr("关闭"))
        btn_row.addWidget(btn_view)
        btn_row.addWidget(btn_close)
        layout.addLayout(btn_row)

        def goto() -> None:
            row = table.currentRow()
            if row < 0:
                return
            name = names[row]
            # 报告已经算好了，切过去显示即可——不必再校一遍
            self.program_panel.focus_report(name)
            dlg.accept()

        btn_view.clicked.connect(goto)
        btn_close.clicked.connect(dlg.reject)
        table.doubleClicked.connect(lambda _idx: goto())
        dlg.exec()

    def open_grasp_wizard(self) -> None:
        """抓取向导（Ctrl+Shift+G）：由抓取点/放置点自动生成接近/提升/撤离点。"""
        if not (_HAS_PROGRAM and hasattr(self, "program_panel")):
            QtWidgets.QMessageBox.information(
                self, tr("抓取向导"), tr("离线编程模块不可用。")
            )
            return
        if not getattr(self.program_panel, "open_grasp_wizard", None):
            QtWidgets.QMessageBox.information(
                self, tr("抓取向导"), tr("当前版本不支持抓取向导。")
            )
            return
        self._focus_tab(self.program_panel)
        self._sync_program_points()
        self.program_panel.open_grasp_wizard()

    def _on_grasp_cycle_applied(self, name: str) -> None:
        """抓取向导写完示教点：把点位回灌**那一台**的视口与路径编辑器。

        否则向导生成的点只躺在编程面板里——视口的路径可视化还画着旧的那串，
        用户会以为向导没生效。按名字回灌是多机的安全底线：写错桶等于把这台
        的点画到另一台身上。
        """
        if not (_HAS_PROGRAM and hasattr(self, "program_panel")):
            return
        getter = getattr(self.program_panel, "get_taught_points", None)
        if getter is None:
            return
        current = getattr(self.gl_widget, "current_robot", None)
        key = name or current
        positions, types, _motions = getter(key)
        self.gl_widget.set_recorded_positions(list(positions), key)
        self.gl_widget.update()
        if key == current:
            self.code_editor.set_recorded_positions(list(positions), key)
            if _HAS_PICKPLACE and types is not None:
                self.code_editor.point_types = list(types)
                self._point_types_by_robot[key] = list(types)
            self.code_editor.sync_point_types()
            self.code_editor.pos_count_label.setText(tr("示教点：{n}").format(n=len(positions)))
            self.code_editor._update_code_with_positions()
            self._sync_program_points(key)

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
        # 与回放帧同一通路：视口 + 属性面板 + 场景同步 + 重绘。
        # 必须带上机器人名——否则真机的关节会被套到别的机器人身上。
        self.on_playback_frame(
            getattr(self.gl_widget, "current_robot", "") or "",
            {name: math.radians(v) for name, v in zip(names, vals)},
        )
        self.statusBar().showMessage(tr("已按真机位形摆好视口"))

    def _highlight_point_line(self, index: int, robot: Optional[str] = None) -> None:
        """把代码编辑器的光标/选区落到第 ``index`` 个点对应的那一行。

        3D 里点了哪个点、回放走到哪一点，代码就停在那一行——逐点核对时不
        必再数行号。查不到对应行就什么都不做（手打的代码行号本就可能偏）。
        """
        if not hasattr(self, "code_editor"):
            return
        line = self.code_editor.line_of_point(index, robot)
        if line is not None:
            self.code_editor.highlight_line(line)

    def on_path_point_move_requested(self, robot_name: str, index: int) -> None:
        """视口右键「移动到」：把该台机器人驱动到示教点 ``index`` 的位形。

        Args:
            robot_name: 点属于哪台（画出来的那一段说了算，不是"当前那台"）
            index: 那台点表里的下标（0 起，界面上显示为 P{index+1}）
        """
        if not robot_name:
            return
        pts = self.gl_widget.get_recorded_positions(robot_name)
        if not (0 <= index < len(pts)):
            return
        self.gl_widget.set_current_robot(robot_name)
        self.gl_widget.set_joint_values(pts[index], robot_name)
        # 必须同步到场景：只改渲染器的话法兰不动，挂在法兰上的夹爪/工件
        # 会留在原地——看着就是"机器人走了、工具没跟"
        self._sync_scene_joints(robot_name)
        # 面板只跟着"正在被查看的那台"刷新，避免把 A 的帧递给 B 的滑块
        if hasattr(self, "property_panel") and (
            self.property_panel.current_object == robot_name
        ):
            self.property_panel.update_joint_displays(pts[index])
        self.gl_widget.update()
        self.statusBar().showMessage(
            tr("已把 {robot} 移动到示教点 P{n}").format(
                robot=robot_name, n=index + 1
            )
        )
        # 3D 里点了哪个点，代码就停在哪一行——便于对着这一行改数值
        self._highlight_point_line(index, robot_name)

    def on_playback_frame(self, robot_name: str, joint_values: dict) -> None:
        """回放帧：按插补后的密集轨迹驱动**指定机器人**，而不是在示教点之间瞬移。

        Args:
            robot_name: 这一帧属于哪台机器人。多机回放时每帧都必须落对口，
                否则两台会跟着同一条轨迹一起抖（看起来像"联动故障"）
            joint_values: ``{关节名: 弧度}``
        """
        robot = robot_name or getattr(self.gl_widget, "current_robot", None)
        self.gl_widget.set_joint_values(joint_values, robot)
        # 面板只跟着"正在被查看的那台"刷新，避免递给 A 的帧去拉动 B 的滑块
        if hasattr(self, "property_panel") and (
            self.property_panel.current_object == robot
        ):
            self.property_panel.update_joint_displays(joint_values)
        self._sync_scene_joints(robot)
        self.gl_widget.update()

    def on_joint_changed(self, robot_name, name, value):
        """关节滑块 → 视口：值写进**发信号那台**的渲染器，而不是"当前"那台。

        Args:
            robot_name: 滑块所属机器人名（属性面板的 :attr:`_active_robot`）；
                空串表示老调用方——退回当前机器人
            name: 关节名
            value: 弧度

        Note:
            多机的关键就在这一个参数：不带名字时只能写 ``gl_widget.renderer``，
            而"当前渲染器"未必是"滑块所属那台"（选中与加载顺序会错位），于是
            拖 A 的滑块让 B 转起来——这就是"两台一起动"。
        """
        renderer = (
            self.gl_widget.renderer_of(robot_name) if robot_name else self.gl_widget.renderer
        )
        renderer.joint_values[name] = value
        self._sync_scene_joints(robot_name or None)
        self.gl_widget.update()

    def _sync_scene_joints(self, robot: Optional[str] = None) -> None:
        """把某台机器人的渲染关节同步给场景，让干涉检查/法兰反映实时位形。

        Args:
            robot: 目标机器人名；``None`` 取当前机器人

        Note:
            必须指名道姓：``set_robot_joints`` 默认只作用于主机器人，多机时会把
            "拖 A 的关节"写到 B 身上——表现为两台一起动、或挂在 A 上的工具离开法兰。
        """
        if not (_HAS_WORLD and hasattr(self, "scene_panel")):
            return
        scene = self.scene_panel.scene
        if not scene.has_robot:
            return
        target = robot or getattr(self.gl_widget, "current_robot", None) or ROBOT_NAME
        if target not in scene.robots:
            target = scene.robot_names[0]
        renderer = self.gl_widget.renderer_of(target)
        # 必须用 actuated_names() 而非 joints.keys()：后者含 fixed 关节
        # （KR16 是 7 个 joints / 6 个可动），多传一个会让 FK 直接抛错
        names = list(renderer.actuated_names())
        if not names:
            # 这台模型不可用（退化渲染器）：写空列表会把场景里已有的关节角清空，
            # 让法兰/干涉检查掉回零位——宁可不同步，也不要把好数据抹成坏数据。
            return
        self.scene_panel.update_robot_joints(
            [math.degrees(renderer.joint_values.get(n, 0.0)) for n in names],
            name=target,
        )

    def _focus_tab(self, widget) -> None:
        """把某个面板切到它所在那一组的前台。

        面板在左还是在右是布局的事（历史上一换就是好几处 ``setCurrentWidget``
        忘了跟着改，点了按钮却看不到对应面板）。这里按"它在哪组里"找，
        以后再挪位置不用改调用方。
        """
        for tabs in (getattr(self, "left_tabs", None), getattr(self, "right_tabs", None)):
            if tabs is not None and tabs.indexOf(widget) >= 0:
                tabs.setCurrentWidget(widget)
                return

    def check_collisions(self) -> None:
        """执行工作单元干涉检查（F6），并把结果高亮到视口。"""
        if not (_HAS_WORLD and hasattr(self, "scene_panel")):
            QtWidgets.QMessageBox.information(self, tr("干涉检查"), tr("环境建模模块不可用。"))
            return
        self._focus_tab(self.scene_panel)
        hits = self.scene_panel.run_collision_check()
        self.gl_widget.set_collision_results(hits)

        # 干涉结果写进常驻状态栏：演示时一眼可见，不用去翻面板
        if hasattr(self, "status_collision"):
            self.status_collision.setText(tr("干涉 {n}").format(n=len(hits)))
            self.status_collision.setStyleSheet(
                "color:#ff8a80; padding:0 8px; font-size:11px;" if hits
                else "color:#a5d6a7; padding:0 8px; font-size:11px;"
            )

    def on_joint_changed_for_record(self, robot_name, name, value):
        """同 :meth:`on_joint_changed`，并顺带把位形记成示教点。"""
        renderer = (
            self.gl_widget.renderer_of(robot_name) if robot_name else self.gl_widget.renderer
        )
        renderer.joint_values[name] = value
        self.gl_widget.update()
        if self.code_editor.record_btn.isChecked():
            # 记到**发信号那台**名下：多机时不指定就会把 B 的位形记成 A 的点
            key = robot_name or self.gl_widget.current_robot
            count = self.gl_widget.record_position(key)
            # 同步到 PathCodeEditor：只换**这台**那一段，其余机器人的路径留在
            # 表里。整表替换的话，给 B 记第一个点就把 A 的路径全抹了
            self.code_editor.set_recorded_positions(
                self.gl_widget.get_recorded_positions(key), key
            )
            self._sync_program_points(key)

    def _robot_for_selection(self) -> Optional[str]:
        """**选中对象**关联到哪台机器人；没选中 / 关联不到返回 ``None``。

        选中夹爪、被夹住的工件同样算选中了那台——它们本来就挂在法兰上，路径
        是那台的，挂载件随法兰一起动。台面、地上没被夹住的工件关联不到任何
        机器人，返回 ``None``，走"从代码选中行号处全局运行"。
        """
        scene = getattr(self.gl_widget, "scene", None)
        selected = getattr(self.gl_widget, "selected_object", None)
        if scene is None or not selected:
            return None
        if scene.is_robot(selected):
            return selected
        return scene.mounted_robot(selected)

    def _robot_usable(self, name: Optional[str]) -> bool:
        """这台机器人现在**真的驱动得动**吗——在场景里、且有运动学链。

        URDF 失效时场景里会留一个"壳"：名字还在、也选得中，但没有运动学，
        驱动它没有意义。
        """
        scene = getattr(self.gl_widget, "scene", None)
        if not name or scene is None or not scene.is_robot(name):
            return False
        return scene.kinematics_of(name) is not None

    def _halt_playback(self, robot: Optional[str], index: int) -> None:
        """归属机器人已不在场景里：立刻停表，并把是哪一行说清楚。"""
        self.code_editor._stop_playback()
        self.code_editor.playbackStopped.emit()
        line = self.code_editor.line_of(index)
        self.statusBar().showMessage(
            tr("第 {line} 行：机器人“{robot}”已不在场景中，已停止运行").format(
                line=line, robot=robot or tr("（未指定）")),
            8000,
        )
        self.code_editor.highlight_line(line)

    def on_playback_requested(self, index, robot=None):
        positions = self.code_editor.get_recorded_positions()
        if 0 <= index < len(positions):
            # 播的是**这条指令归属的那台**：全局运行时一行一个主人，只认
            # 整份归属或"当前选中"都会把这台的程序拽到别台身上
            target = robot or self.code_editor.owner_robot or getattr(
                self.gl_widget, "current_robot", None
            )
            # 归属那台已不在场景里（工程载入后被删掉）：停下来报错。硬跑下去只是
            # 画面不动却显示"运行中"，比当场报错难排查得多
            if not self._robot_usable(target):
                self._halt_playback(target, index)
                return
            self.gl_widget.set_joint_values(positions[index], target)
            # 走到哪一点，代码就高亮到哪一行。用**已有**的 _line_map：回放期间
            # 重新解析会改写点表，回放序列与点表错位会让机器人跳点
            if hasattr(self, "code_editor"):
                line = self.code_editor.line_of(index)
                if line:
                    self.code_editor.highlight_line(line)
            # 面板只跟着"正在被查看的那台"刷新，避免递给 A 的帧去拉动 B 的滑块
            if hasattr(self, "property_panel") and (
                self.property_panel.current_object == target
            ):
                self.property_panel.update_joint_displays(positions[index])
            # 必须同步到场景：视口渲染器与工作单元场景是两套状态。只更新渲染器
            # 的话场景机器人永远停在初始位形，法兰不动，挂在法兰上的夹爪与
            # 被吸附的工件都不会跟着走（干涉检查也会按错误位形算）。
            self._sync_scene_joints(target)
            self._apply_pick_event(index, target)
            self._update_io_status(index, target)
            self.gl_widget.update()

    def _update_io_status(self, index: int, robot: Optional[str] = None) -> None:
        """常驻状态栏显示当前工位的 IO 动作——演示时能一眼看到夹爪在干嘛。

        Args:
            robot: 这条动作属于哪台机器人；``None`` 取当前那台。多机时 A 的夹紧
                不能显示成 B 的工件
        """
        if not hasattr(self, "status_io"):
            return
        action = tr("保持")
        if _HAS_PICKPLACE:
            # 按**总表索引**取语义：播放是按代码行序走的点序列，而
            # get_point_types() 给的是"当前编辑那台"的一段，索引对不上
            t = self.code_editor.point_type_at(index)
            action = {PointType.PICK: tr("夹紧"), PointType.PLACE: tr("松开")}.get(t, tr("保持"))
        wp = self._workpiece_for(robot)
        self.status_io.setText(f"IO {action}" + (f" · {wp}" if wp else ""))

    def _apply_pick_event(self, index: int, robot: Optional[str] = None) -> None:
        """播放到某个示教点时，按工艺语义挂载/释放工件。

        工件被「吸附」的实现就是把它挂到机器人下——复用与夹爪挂载完全相同的
        父子机制，于是它自动跟随法兰运动。释放即解除挂载，并把当前世界位姿
        写回局部 pose，工件原地留下。

        Args:
            robot: 这一帧属于哪台机器人；``None`` 取当前那台。逐点回放时传
                **点表归属**，否则会拿 A 的抓取去挂 B 手里的工件

        Note:
            挂给**谁**必须取当前机器人。写死主机器人 ``ROBOT_NAME`` 时，多机
            场景下 B 的抓取会把工件挂到 A 的法兰上——工件横穿整个工位飞过去。
        """
        if not _HAS_PICKPLACE or not _HAS_WORLD:
            return
        scene = getattr(self, "scene", None) or getattr(
            getattr(self, "scene_panel", None), "scene", None
        )
        if scene is None:
            return

        # 总表索引取语义：点表是多台混排的，按"当前那台的一段"取值会错位
        t = self.code_editor.point_type_at(index)
        if t is None:
            return
        workpiece = self._workpiece_for(robot)
        if not workpiece or workpiece not in scene.objects:
            return
        current = robot or self.gl_widget.current_robot
        holder = (
            current
            if current in scene.robots
            else (scene.robot_names[0] if scene.robot_names else ROBOT_NAME)
        )

        try:
            if t is PointType.PICK:
                scene.attach(workpiece, holder, keep_world=True)
            elif t is PointType.PLACE:
                # 只释放**自己**夹着的：别的机器人手里的工件不能被这台摘掉
                if scene.objects[workpiece].parent != holder:
                    return
                scene.detach(workpiece)
            else:
                return
        except (KeyError, ValueError) as exc:
            print(f"[warning] 工件吸附/释放失败: {exc}")
            return
        if hasattr(self, "scene_panel"):
            self.scene_panel._refresh_tree()

    def on_gripper_state_changed(self, robot_name: str, action: str) -> None:
        """程序级回放到夹爪指令时，吸附/释放工件（S3 抓取动画）。

        Args:
            robot_name: 这条夹爪指令属于哪台机器人——多机回放时 A 夹紧不能把
                工件挂到 B 的法兰上
            action: ``"close"`` / ``"open"`` / ``""``

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
        # 抓的是**这台机器人**的工件，而不是"下拉里当前显示的那件"：多机回放
        # 时 A/B 会交替发夹爪事件，共用一份选择就会互相抢工件
        workpiece = self._workpiece_for(robot_name)
        if not workpiece or workpiece not in scene.objects:
            return
        obj = scene.objects.get(workpiece)
        if obj is None:
            return
        # 夹这件东西的那双手：多机时必须是指定的那台，不能再写死主机器人
        holder = robot_name or (
            scene.robot_names[0] if scene.robot_names else ROBOT_NAME
        )
        try:
            if action == "close":
                if obj.parent != holder:
                    scene.attach(workpiece, holder, keep_world=True)
            elif obj.parent == holder:  # "open" 或回放复位（""）
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
        """当路径代码解析完成后，更新 GLWidget 的路径可视化。

        按**每条指令的归属**分桶回写：整份按 ``current_robot`` 写回时，选中
        B 播一次就会把编辑器里 A 的点复制到 B 名下，B 自己的路径被顶掉——
        用户看到的就是"选了第二台，第一台的路径被复制过去替换了"。
        """
        self._sync_viewport_paths_from_editor()
        self._sync_program_points()

    def on_clear_requested(self):
        # 路径属于机器人：清的是**当前这台**的——不能把别的机器人示教的一并抹掉
        key = self.gl_widget.current_robot
        self.gl_widget.clear_recorded_positions(key)
        self.code_editor.set_recorded_positions([], key)
        self.code_editor.pos_count_label.setText(tr("示教点：0"))
        self.code_editor._update_code_with_positions()
        self._sync_program_points(key)
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
                # 更新 GLWidget 的路径可视化：按段头归属分桶，别整份塞给当前那台
                self._sync_viewport_paths_from_editor()
                self.statusBar().showMessage(f"Loaded: {Path(file_path).name}")
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Error", f"Failed to load path code: {e}")
    
    def reset_view(self):
        self.gl_widget.camera_dist = 1.2
        self.gl_widget.camera_az = 45.0
        self.gl_widget.camera_el = 30.0
        self.gl_widget.update()


# --------------------------------------------------------------------------- #
# 崩溃兜底（v1.4.0）
#
# 打包出去的是 console=False 的 GUI：Windows 上没有终端，未捕获异常的对话框
# 一闪而过（macOS 双击 dist 里的 app 同样看不到任何输出），现场问题最后只能
# 靠口头猜。这里把 traceback 落到与自动保存同一处的日志目录，配合 v1.3.5 的
# 崩溃自动恢复，至少保证「现场有据可查」。
# --------------------------------------------------------------------------- #
def crash_log_dir() -> "Path":
    """崩溃日志目录（``$OROLP_CONFIG_DIR/logs`` > ``~/.orolp/logs``）。

    与 :mod:`orolp.session` 的用户级配置根目录同口径：找自动保存的文件在一个
    地方，找崩溃日志也在同一个地方。
    """
    base = os.environ.get("OROLP_CONFIG_DIR")
    return (Path(base) if base else Path.home() / ".orolp") / "logs"


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
            f"RobotWorld v{_OROLP_VERSION} 崩溃日志",
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

    app = QtWidgets.QApplication(sys.argv)
    
    urdf_path = sys.argv[1] if len(sys.argv) > 1 else None
    window = MainWindow(urdf_path)
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
