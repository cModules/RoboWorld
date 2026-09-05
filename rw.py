#!/usr/bin/env python3
"""
RobotWorld - OLP Terminal
"""

import sys
import math
import re
from typing import Dict, List, Optional, Tuple  # noqa: F401

import numpy as np
from pathlib import Path

# 本文件既可作为脚本直接运行，也可被打包；确保 python/ 下的 orolp 包可被导入
_REPO_ROOT = Path(__file__).resolve().parent
_PYTHON_DIR = _REPO_ROOT / "python"
if str(_PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(_PYTHON_DIR))

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

# 环境建模（工作单元场景 + 参数化组件库 + 干涉检查）
try:
    from orolp.world.panel import ScenePanel
    from orolp.world.scene import Scene
    from orolp.world.geometry import prim_aabb
    from orolp.pose import Pose

    _HAS_WORLD = True
except Exception as _exc:  # pragma: no cover
    print(f"[warning] world modeling unavailable: {_exc}")
    _HAS_WORLD = False
    ScenePanel = None
    Scene = None

# 离线编程（轨迹展开 + 节拍估算 + 程序校验面板）
try:
    from orolp.program_panel import ProgramPanel

    _HAS_PROGRAM = True
except Exception as _exc:  # pragma: no cover
    print(f"[warning] program panel unavailable: {_exc}")
    _HAS_PROGRAM = False
    ProgramPanel = None

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
                f"关节值数量 {len(values)} 与自由度 {len(names)} 不符（关节: {names}）"
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

    def __init__(self, parent=None):
        super().__init__(parent)
        self.renderer = RobotRenderer()
        self.camera_dist = 1.2
        self.camera_az = 45.0
        self.camera_el = 30.0
        self.dragging = False
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

    def load_robot(self, path):
        success = self.renderer.load_urdf(path)
        if success:
            self.frame_robot()
        self.update()
        return success

    def frame_robot(self) -> None:
        """按当前位姿的包围球自动取景（相机距离 + 注视点）。

        机器人尺寸跨度很大：UR3e 臂展 500 mm，IRB 6700 臂展 3200 mm。
        固定相机距离会让小机器人只占画面一角、大机器人直接出画。

        包围球按**每个 visual 的实际几何**算，而不是只取连杆原点——后者会
        低估尺寸（连杆原点只是几个点，几何体本身还有体积），导致大臂贴边出画。
        """
        bounds = self._robot_bounds()
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

    def _robot_bounds(self):
        """返回当前位姿下机器人所有 visual 的世界包围盒 ``(lo, hi)``。

        Returns:
            ``(lo, hi)`` 两个长度 3 的数组；无模型时返回 ``None``。
        """
        model = self.renderer.model
        poses = self.renderer.link_poses()
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

        return (lo, hi) if found else None

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
            self._highlight_object(self.selected_object)
        self._draw_gizmo()

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
                obj = self.scene.objects.get(name)
                if obj is None or not obj.visible:
                    continue
                for part in self.scene.object_parts(name):
                    self._draw_part(part)

            # 干涉对用红色线框高亮，让问题一眼可见
            for hit in self._last_collisions:
                self._highlight_object(hit.obj_b)
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

    def _highlight_object(self, name: str) -> None:
        """用红色线框标出发生干涉的对象。"""
        obj = self.scene.objects.get(name)
        if obj is None:
            return
        gl.glPushMatrix()
        gl.glColor3f(1.0, 0.25, 0.15)
        gl.glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        gl.glLineWidth(2.0)
        for part in self.scene.object_parts(name):
            self._draw_part(part)
        gl.glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        gl.glLineWidth(1.0)
        gl.glPopMatrix()

    def _draw_part(self, part, tf_parent=None) -> None:
        """绘制单个零件（基本体 + 局部变换）。

        Args:
            part: :class:`~orolp.world.components.Part`
            tf_parent: 父级变换（已含世界变换）；``None`` 表示 part 自带世界变换
        """
        m = np.array(part.transform, dtype=np.float64)
        if tf_parent is not None:
            m = tf_parent @ m
        m = m.copy()
        m[:3, 3] *= self.MM_TO_M

        gl.glPushMatrix()
        # OpenGL 采用列主序，numpy 为行主序，需转置后再展平
        gl.glMultMatrixd(m.T.flatten())
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
        """
        model = self.renderer.model
        if model is None or model.root is None:
            # 渲染模型模块不可用：回退到旧的单几何递归绘制
            if self.renderer.root_link is not None:
                self._draw_link_recursive(self.renderer.root_link)
            return

        poses = self.renderer.link_poses()
        for link_name in model.link_chain():
            link = model.links.get(link_name)
            base_pose = poses.get(link_name)
            if link is None or base_pose is None:
                continue
            for visual in link.visuals:
                self._draw_visual(base_pose, visual)

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

            q = self.renderer.joint_values.get(parent_joint["name"], 0.0)
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
                self._draw_link_recursive(joint["child"], joint)

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
                for part in self.scene.object_parts(name):
                    aabb = prim_aabb(part.prim, part.transform)
                    lo = np.minimum(lo, aabb.lo * self.MM_TO_M)
                    hi = np.maximum(hi, aabb.hi * self.MM_TO_M)
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
        """把对象设到指定的世界位姿（毫米 + 绕Z角度），自动换算回局部 pose。"""
        if not _HAS_WORLD or self.scene is None:
            return
        obj = self.scene.objects.get(name)
        if obj is None:
            return
        wx, wy, wz = x_mm, y_mm, z_mm
        if obj.parent and obj.parent in self.scene.objects:
            parent_tf = self.scene.world_transform(obj.parent)
            inv = np.linalg.inv(parent_tf)
            lp = inv @ np.array([wx, wy, wz, 1.0])
            lx, ly, lz = lp[:3]
            pr = math.degrees(math.atan2(parent_tf[1, 0], parent_tf[0, 0]))
            lrz = rz_deg - pr
        else:
            lx, ly, lz, lrz = wx, wy, wz, rz_deg
        obj.pose = Pose.from_xyz_rpy(lx, ly, lz, 0.0, 0.0, lrz)

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
            self.dragging = True
            self.last_pos = event.position().toPoint()

    def mouseMoveEvent(self, event):
        # 右键（或 Control+左键）按住时做 gizmo 变换，不做视角旋转
        if self._right_like and (event.buttons() & (Qt.RightButton | Qt.LeftButton)):
            if self.selected_object is not None:
                self._handle_right_drag(event.position().toPoint())
            return

        if self.dragging and self.last_pos:
            current_pos = event.position().toPoint()
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

    def __init__(self, parent=None):
        super().__init__(parent)
        self.recorded_positions = []
        self.joint_names = []  # 存储关节名称顺序
        self.playback_timer = None
        self.current_playback_index = 0
        self.playback_speed = 50
        self._setup_ui()
        self.setPlainText("# OLP Path Code\nmovej([0, -90, 90, -90, 90, 0], vel=0.5)\n")

    def _setup_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        self.code_edit = QtWidgets.QTextEdit()
        self.code_edit.setFont(QFont("Consolas", 10))
        self.code_edit.setStyleSheet("background-color: #1a1a1a; color: #e0e0e0;")
        self.code_edit.setLineWrapMode(QtWidgets.QTextEdit.NoWrap)
        self.code_edit.setTabStopDistance(20)
        layout.addWidget(self.code_edit)

        control_layout = QtWidgets.QHBoxLayout()

        self.record_btn = QtWidgets.QPushButton("● Record")
        self.record_btn.setStyleSheet("background-color: #555555; color: white;")
        self.record_btn.setCheckable(True)
        self.record_btn.clicked.connect(self._on_record_toggled)

        self.play_btn = QtWidgets.QPushButton("▶ Play")
        self.play_btn.setStyleSheet("background-color: #4CAF50; color: white;")
        self.play_btn.clicked.connect(self._on_play_clicked)

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
        control_layout.addWidget(self.stop_btn)
        control_layout.addWidget(self.clear_btn)
        control_layout.addWidget(self.pos_count_label)
        layout.addLayout(control_layout)

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

    def _stop_playback(self):
        if self.playback_timer and self.playback_timer.isActive():
            self.playback_timer.stop()
        self.playback_timer = None
        self.current_playback_index = 0

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
        self.pos_count_label.setText("Positions: 0")
        self.code_edit.clear()
        self.code_edit.append("# OLP Path Code\n")
        self.clearRequested.emit()

    def record_position(self, joint_values):
        if self.record_btn.isChecked():
            self.recorded_positions.append(dict(joint_values))
            self.pos_count_label.setText(f"Positions: {len(self.recorded_positions)}")
            self.positionRecorded.emit(len(self.recorded_positions))
            self._update_code_with_positions()

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


class DevicePropertyPanel(QtWidgets.QWidget):
    jointChanged = Signal(str, float)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.sliders = {}
        self.value_labels = {}
        self.joint_layout = None
        self.log_text = None
        self.name_label = None
        self.type_label = None
        self.manufacturer_label = None
        self.dof_label = None
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        info_group = QtWidgets.QGroupBox("机器人信息")
        info_layout = QtWidgets.QFormLayout(info_group)
        self.name_label = QtWidgets.QLabel("N/A")
        self.manufacturer_label = QtWidgets.QLabel("N/A")
        self.type_label = QtWidgets.QLabel("N/A")
        self.payload_label = QtWidgets.QLabel("N/A")
        self.reach_label = QtWidgets.QLabel("N/A")
        self.repeat_label = QtWidgets.QLabel("N/A")
        self.dof_label = QtWidgets.QLabel("N/A")
        self.controller_label = QtWidgets.QLabel("N/A")
        info_layout.addRow("名称:", self.name_label)
        info_layout.addRow("厂商:", self.manufacturer_label)
        info_layout.addRow("类型:", self.type_label)
        info_layout.addRow("额定负载:", self.payload_label)
        info_layout.addRow("最大臂展:", self.reach_label)
        info_layout.addRow("重复定位精度:", self.repeat_label)
        info_layout.addRow("轴数:", self.dof_label)
        info_layout.addRow("控制器:", self.controller_label)
        layout.addWidget(info_group)

        # 型号备注（数据来源、参数核验状态等）
        self.note_label = QtWidgets.QLabel("")
        self.note_label.setWordWrap(True)
        self.note_label.setStyleSheet("color: #8a6d3b; padding: 2px;")
        note_font = QFont()
        note_font.setPointSize(8)
        self.note_label.setFont(note_font)
        self.note_label.hide()
        layout.addWidget(self.note_label)

        joint_group = QtWidgets.QGroupBox("关节控制")
        self.joint_layout = QtWidgets.QVBoxLayout(joint_group)
        layout.addWidget(joint_group)
        
        log_group = QtWidgets.QGroupBox("日志")
        log_layout = QtWidgets.QVBoxLayout(log_group)
        self.log_text = QtWidgets.QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 9))
        self.log_text.setStyleSheet("background-color: #1a1a1a; color: #ffffff;")
        log_layout.addWidget(self.log_text)
        layout.addWidget(log_group)
        
        layout.addStretch()
    
    def setup_joints(self, renderer):
        while self.joint_layout.count() > 0:
            item = self.joint_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        
        self.sliders.clear()
        self.value_labels.clear()
        
        joint_items = []
        for name, joint in renderer.joints.items():
            if joint["type"] == "revolute":
                child = joint["child"]
                if child in renderer.links:
                    try:
                        idx = list(renderer.links.keys()).index(child)
                        joint_items.append((idx, name, joint))
                    except ValueError:
                        pass
        
        joint_items.sort(key=lambda x: x[0])
        
        for _, name, joint in joint_items:
            frame = QtWidgets.QFrame()
            frame.setFrameStyle(QtWidgets.QFrame.StyledPanel)
            flayout = QtWidgets.QVBoxLayout(frame)
            flayout.setContentsMargins(5, 5, 5, 5)
            flayout.setSpacing(4)

            grid = QtWidgets.QGridLayout()
            grid.setColumnStretch(0, 1)   # 关节名列可随面板扩展
            grid.setColumnStretch(1, 0)   # 数值列保持最小宽度
            grid.setColumnMinimumWidth(0, 80)  # 保证 "Joint 1" 完整显示
            grid.setColumnMinimumWidth(1, 70)  # 保证角度值完整显示
            grid.setHorizontalSpacing(8)

            label = QtWidgets.QLabel(f"<b>{name}</b>")
            label.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
            label.setWordWrap(False)

            self.value_labels[name] = QtWidgets.QLabel("0.0°")
            self.value_labels[name].setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            self.value_labels[name].setSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
            self.value_labels[name].setMinimumWidth(70)

            grid.addWidget(label, 0, 0)
            grid.addWidget(self.value_labels[name], 0, 1)

            slider = QtWidgets.QSlider(Qt.Horizontal)
            slider.setMinimum(int(math.degrees(joint["limit"]["lower"]) * 100))
            slider.setMaximum(int(math.degrees(joint["limit"]["upper"]) * 100))
            slider.setValue(0)
            slider.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            slider.valueChanged.connect(lambda v, n=name: self._on_joint_changed(n, v))
            grid.addWidget(slider, 1, 0, 1, 2)
            self.sliders[name] = slider

            min_label = QtWidgets.QLabel(f"{math.degrees(joint['limit']['lower']):.1f}°")
            max_label = QtWidgets.QLabel(f"{math.degrees(joint['limit']['upper']):.1f}°")
            max_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            min_label.setSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
            max_label.setSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
            grid.addWidget(min_label, 2, 0)
            grid.addWidget(max_label, 2, 1)

            flayout.addLayout(grid)
            
            self.joint_layout.addWidget(frame)
    
    def _on_joint_changed(self, name, value):
        angle = value / 100.0
        if name in self.value_labels:
            self.value_labels[name].setText(f"{angle:.2f}°")
        self.jointChanged.emit(name, math.radians(angle))
    
    def update_robot_info(self, renderer, entry=None):
        """更新机器人信息区。

        Args:
            renderer: 已加载 URDF 的渲染器
            entry: 机器人库条目；提供时可回显负载、臂展等完整规格
        """
        revolute = [j for j in renderer.joints.values() if j["type"] == "revolute"]
        self.dof_label.setText(str(len(revolute)))

        if entry is not None:
            self.name_label.setText(entry.model_name)
            self.manufacturer_label.setText(entry.brand_display)
            type_map = {"industrial": "工业臂", "cobot": "协作臂", "scara": "SCARA"}
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
                prefix = "" if entry.verified else "⚠ 参数未核验 — "
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

        self.log_text.append(f"已载入: {renderer.robot_name}")
        self.log_text.append(f"关节数: {len(revolute)}")

    def update_joint_displays(self, joint_values):
        for name, value in joint_values.items():
            if name in self.value_labels:
                self.value_labels[name].setText(f"{math.degrees(value):.2f}°")
            if name in self.sliders:
                self.sliders[name].blockSignals(True)
                self.sliders[name].setValue(int(math.degrees(value) * 100))
                self.sliders[name].blockSignals(False)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, urdf_path=None):
        super().__init__()
        self.setWindowTitle("RobotWorld Terminal v0.1.0")
        self.resize(1400, 900)
        self._set_window_icon()
        self.setup_menu()
        self.setup_ui()
        self.urdf_path = urdf_path
        #: 供离线编程面板使用的 :class:`orolp.Robot`（含运动学）
        self.olp_robot = None
    
    def showEvent(self, event):
        super().showEvent(event)
        if self.urdf_path and Path(self.urdf_path).exists():
            self.load_robot(self.urdf_path)
        elif _HAS_ROBOT_LIBRARY and hasattr(self, "library_panel"):
            self._load_default_robot()
        self.gl_widget.update()

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
        if hasattr(self, "left_tabs"):
            self.left_tabs.setCurrentIndex(0)

    def reload_robot_library(self):
        """重新扫描 models/ 目录。"""
        if hasattr(self, "library_panel"):
            self.library_panel.refresh()
    
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

        file_menu.addSeparator()

        gen_action = QAction("&Generate Robot Code...", self)
        gen_action.setShortcut("Ctrl+G")
        gen_action.triggered.connect(self.generate_robot_code)
        file_menu.addAction(gen_action)

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
            show_lib = QAction("显示&机器人库", self)
            show_lib.setShortcut("Ctrl+L")
            show_lib.triggered.connect(self.show_robot_library)
            view_menu.addAction(show_lib)

            reload_lib = QAction("重新扫描机器人库", self)
            reload_lib.setShortcut("F5")
            reload_lib.triggered.connect(self.reload_robot_library)
            view_menu.addAction(reload_lib)

        if _HAS_WORLD:
            view_menu.addSeparator()
            check_col = QAction("工作单元干涉检查", self)
            check_col.setShortcut("F6")
            check_col.triggered.connect(self.check_collisions)
            view_menu.addAction(check_col)

        if _HAS_PROGRAM:
            if not _HAS_WORLD:
                view_menu.addSeparator()
            validate_prog = QAction("程序校验与节拍估算", self)
            validate_prog.setShortcut("F7")
            validate_prog.triggered.connect(self.validate_program)
            view_menu.addAction(validate_prog)
    
    def _set_window_icon(self):
        import sys
        logo_path = Path(__file__).parent / "logo.png"
        if hasattr(sys, '_MEIPASS'):
            logo_path = Path(sys._MEIPASS) / "logo.png"
        
        if logo_path.exists():
            icon = QtGui.QIcon(str(logo_path))
            self.setWindowIcon(icon)
    
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
            self.library_panel.statusMessage.connect(self.statusBar().showMessage)

            self.left_tabs = QtWidgets.QTabWidget()
            self.left_tabs.addTab(self.library_panel, "机器人库")
            self.left_tabs.addTab(self.code_editor, "路径代码")

            if _HAS_WORLD:
                # 注意：此时 gl_widget 尚未创建，信号连接在 setup_ui 末尾进行
                self.scene_panel = ScenePanel(Scene())
                self.scene_panel.statusMessage.connect(self.statusBar().showMessage)
                self.left_tabs.addTab(self.scene_panel, "场景")

            if _HAS_PROGRAM:
                self.program_panel = ProgramPanel()
                self.program_panel.statusMessage.connect(self.statusBar().showMessage)
                self.program_panel.playbackFrame.connect(self.on_playback_frame)
                self.program_panel.playbackFinished.connect(self.on_playback_stopped)
                self.left_tabs.addTab(self.program_panel, "离线编程")

            self.left_tabs.setMaximumWidth(360)
            self.left_tabs.setMinimumWidth(250)
            main_layout.addWidget(self.left_tabs)
        else:
            left_panel = QtWidgets.QWidget()
            left_panel.setMaximumWidth(350)
            left_panel.setMinimumWidth(250)
            left_layout = QtWidgets.QVBoxLayout(left_panel)
            left_layout.setContentsMargins(5, 5, 5, 5)
            left_layout.addWidget(QtWidgets.QLabel("<h3>Path Code Editor</h3>"))
            left_layout.addWidget(self.code_editor)
            main_layout.addWidget(left_panel)
        
        self.gl_widget = GLWidget(self)
        self.gl_widget.setMinimumSize(400, 300)
        main_layout.addWidget(self.gl_widget, 1)
        
        self.property_panel = DevicePropertyPanel()
        self.property_panel.setMaximumWidth(300)
        self.property_panel.setMinimumWidth(250)
        self.property_panel.jointChanged.connect(self.on_joint_changed)
        main_layout.addWidget(self.property_panel)

        # 场景改动需要触发视口重绘，此连接必须等 gl_widget 建好后才能做
        if _HAS_WORLD and hasattr(self, "scene_panel"):
            self.scene_panel.sceneChanged.connect(self.gl_widget.update)
            # 视口右键拾取/拖动选中对象时，同步到左侧场景面板树形选择
            self.gl_widget.objectSelected.connect(self.scene_panel._select_object)

        self.statusBar().showMessage("Ready")
    
    def open_model(self):
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open Robot Model", "models", "URDF Files (*.urdf);;All Files (*)"
        )
        if file_path:
            self.load_robot(file_path)
    
    def load_robot(self, path):
        """加载 URDF 到 3D 视口，并同步属性面板。

        若该文件属于机器人库中的某个型号，属性面板会回显负载、臂展等完整规格。
        """
        if not self.gl_widget.load_robot(path):
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to load URDF: {path}")
            return

        # 记录当前模型路径：后处理需要据此推断品牌与关节结构。
        # 历史实现从不更新该字段，导致即使已载入机器人，
        # 依赖 urdf_path 的功能（如生成控制器代码）仍认为"未加载模型"。
        self.urdf_path = str(path)

        entry = None
        if _HAS_ROBOT_LIBRARY and hasattr(self, "library_panel"):
            entry = self.library_panel.catalog.find_by_urdf(path)

        self.property_panel.setup_joints(self.gl_widget.renderer)
        self.property_panel.update_robot_info(self.gl_widget.renderer, entry)
        self.property_panel.jointChanged.connect(self.on_joint_changed_for_record)
        
        # 设置关节名称顺序，用于解析路径代码
        joint_names = list(self.gl_widget.renderer.joints.keys())
        self.code_editor.set_joint_names(joint_names)
        
        recorded = self.code_editor.get_recorded_positions()
        if recorded:
            self.gl_widget.set_recorded_positions(recorded)
        self.statusBar().showMessage(f"Loaded: {Path(path).name}")

        # 同步机器人到工作单元场景，使其参与干涉检查
        if _HAS_WORLD and hasattr(self, "scene_panel"):
            self.scene_panel.set_robot(str(path))
            self.gl_widget.scene = self.scene_panel.scene

        # 离线编程面板需要真正的 orolp.Robot（含运动学），而不是只用于
        # 绘制的 renderer——校验与节拍估算都建立在 FK/IK 之上
        if _HAS_PROGRAM and hasattr(self, "program_panel"):
            try:
                from orolp import Robot as OrolpRobot

                self.olp_robot = OrolpRobot(str(path), Path(path).stem)
                self.program_panel.set_robot(self.olp_robot, joint_names)
                self.program_panel.set_scene(
                    self.scene_panel.scene if _HAS_WORLD and hasattr(self, "scene_panel") else None
                )
                self._sync_program_points()
            except Exception as exc:
                print(f"[warning] 无法为离线编程面板准备机器人: {exc}")
                self.olp_robot = None

        self.gl_widget.update()

    def _sync_program_points(self) -> None:
        """把路径代码编辑器里的示教点同步给离线编程面板。"""
        if not (_HAS_PROGRAM and hasattr(self, "program_panel")):
            return
        self.program_panel.set_taught_points(self.code_editor.get_recorded_positions())

    def validate_program(self) -> None:
        """校验程序并给出节拍（F7）。"""
        if not (_HAS_PROGRAM and hasattr(self, "program_panel")):
            QtWidgets.QMessageBox.information(self, "程序校验", "离线编程模块不可用。")
            return
        if hasattr(self, "left_tabs"):
            self.left_tabs.setCurrentWidget(self.program_panel)
        self._sync_program_points()
        self.program_panel.validate_program()

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
        """把当前关节角同步给场景，让干涉检查反映实时位形。"""
        if not (_HAS_WORLD and hasattr(self, "scene_panel")):
            return
        if not self.scene_panel.scene.has_robot:
            return
        names = list(self.gl_widget.renderer.joints.keys())
        self.scene_panel.update_robot_joints(
            [math.degrees(self.gl_widget.renderer.joint_values.get(n, 0.0)) for n in names]
        )

    def check_collisions(self) -> None:
        """执行工作单元干涉检查（F6），并把结果高亮到视口。"""
        if not (_HAS_WORLD and hasattr(self, "scene_panel")):
            QtWidgets.QMessageBox.information(self, "干涉检查", "环境建模模块不可用。")
            return
        self.left_tabs.setCurrentWidget(self.scene_panel)
        hits = self.scene_panel.run_collision_check()
        self.gl_widget.set_collision_results(hits)

    def on_joint_changed_for_record(self, name, value):
        self.gl_widget.renderer.joint_values[name] = value
        self.gl_widget.update()
        if self.code_editor.record_btn.isChecked():
            # 当记录模式开启时，调用 GLWidget 的记录方法
            count = self.gl_widget.record_position()
            # 同步到 PathCodeEditor
            self.code_editor.recorded_positions = self.gl_widget.get_recorded_positions()
            self.code_editor.pos_count_label.setText(f"Positions: {count}")
            self.code_editor._update_code_with_positions()
            self._sync_program_points()

    def on_playback_requested(self, index):
        positions = self.code_editor.get_recorded_positions()
        if 0 <= index < len(positions):
            self.gl_widget.set_joint_values(positions[index])
            self.property_panel.update_joint_displays(positions[index])
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

        与"保存路径代码"不同，这里走的是完整的后处理链路
        （``orolp.program.Program.to_post``），输出 KRL / RAPID / TP / URScript
        等**控制器可直接加载**的源码，而不是中间的 Python 形态路径文本。

        出错时弹窗明确提示——不做静默降级。历史上后处理器一旦失败就会输出
        一堆 ``; Motion 1: ...`` 注释占位，界面却显示成功，极易误导。
        """
        positions = self.code_editor.get_recorded_positions()
        if not positions:
            QtWidgets.QMessageBox.information(
                self, "生成机器人代码",
                "当前没有示教点。\n请先在 3D 视口中调整关节角度并点击「记录」按钮。",
            )
            return

        urdf_path = getattr(self, "urdf_path", None)
        if not urdf_path or not Path(urdf_path).exists():
            QtWidgets.QMessageBox.warning(
                self, "生成机器人代码",
                "未加载机器人模型，无法确定目标品牌与后处理器。\n"
                "请先从「机器人库」载入一个型号。",
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
                self, "代码生成失败",
                f"{exc}\n\n可用后处理器：\n  " + "\n  ".join(list_post_processors()),
            )
            return
        except Exception as exc:
            QtWidgets.QMessageBox.critical(
                self, "代码生成失败",
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
        dialog.setWindowTitle(f"生成的机器人代码 — {lang}")
        dialog.resize(720, 520)

        layout = QtWidgets.QVBoxLayout(dialog)
        info = QtWidgets.QLabel(
            f"机器人：{robot.name}    目标语言：{lang}    指令数：{len(code.splitlines())}"
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
        save_btn = buttons.addButton("保存为文件…", QtWidgets.QDialogButtonBox.AcceptRole)
        buttons.addButton("关闭", QtWidgets.QDialogButtonBox.RejectRole)
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
            parent, f"保存 {lang} 代码", default,
            f"{lang} Files (*.{ext});;Text Files (*.txt);;All Files (*)",
        )
        if file_path:
            try:
                with open(file_path, "w", encoding="utf-8") as fh:
                    fh.write(code)
                self.statusBar().showMessage(f"已保存 {lang} 代码: {Path(file_path).name}")
            except OSError as exc:
                QtWidgets.QMessageBox.critical(parent, "保存失败", str(exc))
    
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
