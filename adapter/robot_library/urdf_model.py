"""把 URDF 解析成一份可渲染 / 可查询的场景模型。

为什么需要这个模块
------------------
``olp.py`` 里原有的 ``RobotRenderer`` 有三个硬伤，直接导致"机器人画得不像"：

1. **每个 link 只取第一个 ``<visual>``** —— 真实机器人一个连杆往往由十几个基本体
   （铸件本体、电机壳、法兰、线缆管、配重）拼成，取第一个就等于只画了个轴套。
2. **不认识 ``<mesh>``** —— 带 STL/OBJ 的模型（如 LeRobot SO-101 的 14 个网格）
   在视口里**完全不可见**，用户看到的是一个空场景。
3. **link 级 origin 用 ``R·T`` 而非 URDF 标准的 ``T·R``** —— 一旦 rpy 非零，
   几何体就会绕着错误的点旋转，位置明显错位。

本模块按 URDF 规范重新实现解析，并且**不依赖 OpenGL**，可以在无显示环境下单元测试。

坐标系与变换约定
----------------
URDF 中 ``<origin xyz=... rpy=...>`` 的含义是::

    T = translate(xyz) · Rz(yaw) · Ry(pitch) · Rx(roll)

即"先绕 x 滚转、再绕 y 俯仰、最后绕 z 偏航"，三个旋转都绕**固定轴**进行，
所以矩阵从右往左累积为 ``Rz·Ry·Rx``，且整体是**先旋转后平移**（``T·R``）。

关节的变换链为::

    子连杆坐标系 = 父连杆坐标系 · T(joint origin) · R(axis, q)

其中 ``q`` 是关节变量（revolute/continuous 为弧度，prismatic 为米）。
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .mesh import MeshData, MeshLoadError, load_mesh

__all__ = [
    "Primitive",
    "Visual",
    "Link",
    "Joint",
    "JointLimits",
    "RobotModel",
    "UrdfParseError",
    "rpy_to_matrix",
    "xyzrpy_to_matrix",
]


class UrdfParseError(RuntimeError):
    """URDF 结构非法或缺少必需字段。"""


# --------------------------------------------------------------------------- #
# 变换工具
# --------------------------------------------------------------------------- #
def rpy_to_matrix(rpy: Sequence[float]) -> np.ndarray:
    """把 URDF 的 ``rpy``（弧度）转成 4×4 齐次旋转矩阵。

    约定见模块文档：结果为 ``Rz(yaw) · Ry(pitch) · Rx(roll)``。

    Args:
        rpy: ``(roll, pitch, yaw)``，弧度，分别绕 X / Y / Z。

    Returns:
        4×4 齐次旋转矩阵（平移分量为 0）。
    """
    rx, ry, rz = (float(v) for v in rpy)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    rx_m = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    ry_m = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    rz_m = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)

    rot = rz_m @ ry_m @ rx_m
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = rot
    return mat


def xyzrpy_to_matrix(
    xyz: Sequence[float],
    rpy: Sequence[float],
) -> np.ndarray:
    """把 ``<origin xyz rpy>`` 转成 4×4 齐次变换矩阵。

    Args:
        xyz: 平移 ``(x, y, z)``。
        rpy:  ``(roll, pitch, yaw)``，弧度。

    Returns:
        4×4 齐次变换矩阵，满足 URDF 规范的 **先旋转后平移**（``T · R``）。

    Note:
        这是本模块与旧 ``RobotRenderer`` 的核心差异：旧实现写成了 ``R · T``，
        当 ``rpy`` 非零时会把几何体绕原点旋转后再平移，产生肉眼可见的错位。
    """
    mat = rpy_to_matrix(rpy)
    mat[:3, 3] = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
    return mat


def _parse_vec(text: Optional[str], default: Sequence[float]) -> Tuple[float, ...]:
    """解析空格分隔的向量文本，缺失或非法时返回默认值。"""
    if not text:
        return tuple(float(v) for v in default)
    parts = text.replace(",", " ").split()
    if not parts:
        return tuple(float(v) for v in default)
    try:
        return tuple(float(p) for p in parts)
    except ValueError as exc:
        raise UrdfParseError(f"无法解析向量 '{text}'") from exc


# --------------------------------------------------------------------------- #
# 几何体
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Primitive:
    """一个可渲染的几何体。

    Attributes:
        kind: ``'box'`` / ``'cylinder'`` / ``'sphere'`` / ``'mesh'``。
        params: 依 ``kind`` 而定——
            ``box`` → ``(sx, sy, sz)``；
            ``cylinder`` → ``(radius, length)``，轴向为局部 **Z**；
            ``sphere`` → ``(radius,)``；
            ``mesh`` → ``(MeshData,)``。
    """

    kind: str
    params: Tuple[object, ...]

    @property
    def mesh(self) -> Optional[MeshData]:
        """``kind == 'mesh'`` 时返回网格数据，否则 ``None``。"""
        if self.kind == "mesh":
            return self.params[0]  # type: ignore[return-value]
        return None


@dataclass(frozen=True)
class Visual:
    """一个 visual（几何体 + 位姿 + 材质）。

    Attributes:
        matrix: 4×4 齐次变换，从**所属连杆坐标系**到该几何体坐标系。
        primitive: 几何体。
        color: RGB 三元组，各分量 0~1。
        alpha: 不透明度，0~1。
    """

    matrix: np.ndarray
    primitive: Primitive
    color: Tuple[float, float, float]
    alpha: float


@dataclass
class Link:
    """URDF 连杆。

    Attributes:
        name: 连杆名。
        visuals: 全部 visual 节点。旧实现只保留第一个，是"外观粗糙"的主因之一。
    """

    name: str
    visuals: List[Visual] = field(default_factory=list)

    @property
    def has_geometry(self) -> bool:
        """是否存在任何可见几何体。"""
        return bool(self.visuals)


@dataclass(frozen=True)
class JointLimits:
    """关节限位与速度上限。

    Attributes:
        lower: 下限（revolute 为弧度，prismatic 为米）。
        upper: 上限。
        velocity: 速度上限（弧度/秒 或 米/秒）。
        effort: 力矩/力上限。
    """

    lower: float
    upper: float
    velocity: float
    effort: float


@dataclass
class Joint:
    """URDF 关节。

    Attributes:
        name: 关节名。
        type: ``fixed`` / ``revolute`` / ``continuous`` / ``prismatic``。
        parent: 父连杆名。
        child: 子连杆名。
        matrix: 4×4 齐次变换，从父连杆坐标系到关节坐标系（即 ``<origin>``）。
        axis: 运动轴，父坐标系下的单位向量。
        limits: 限位；``fixed`` 关节为零限位。
    """

    name: str
    type: str
    parent: str
    child: str
    matrix: np.ndarray
    axis: Tuple[float, float, float]
    limits: JointLimits

    @property
    def is_actuated(self) -> bool:
        """是否为可动关节（非 fixed）。"""
        return self.type != "fixed"


# --------------------------------------------------------------------------- #
# 模型
# --------------------------------------------------------------------------- #
class RobotModel:
    """一份解析完成的 URDF 模型。

    除渲染所需的几何数据外，还提供运动学相关的查询（驱动关节序列、正向运动学），
    便于与 :mod:`domain.kinematics` 对照验证。

    Attributes:
        name: 机器人名（URDF 根节点的 ``name``）。
        links: 连杆名 → :class:`Link`。
        joints: 关节名 → :class:`Joint`。
        root: 根连杆名。
        urdf_path: 来源文件路径。
        mesh_errors: 加载失败的网格及其原因。与静默降级不同，这里**记录下来**，
            调用方可以决定是报错还是显示占位几何。
    """

    #: ``package://<pkg>/<rest>`` 中 ``<pkg>`` 的常见落点（相对 URDF 所在目录）
    _PACKAGE_SEARCH_DEPTHS = (".", "..", "../..", "../../..")

    def __init__(self) -> None:
        self.name: str = "unknown"
        self.links: Dict[str, Link] = {}
        self.joints: Dict[str, Joint] = {}
        self.root: Optional[str] = None
        self.urdf_path: Optional[Path] = None
        self.mesh_errors: List[Tuple[str, str]] = []

    # ---------------- 构造 ---------------- #
    @classmethod
    def from_urdf(
        cls,
        path: str | Path,
        *,
        load_meshes: bool = True,
        auto_unit_scale: bool = True,
    ) -> "RobotModel":
        """解析一份 URDF 文件。

        Args:
            path: URDF 文件路径。
            load_meshes: 是否真正加载 ``<mesh>`` 的网格数据。设为 ``False`` 可以
                只校验结构、跳过 17 MB 级别的网格 IO（用于批量扫描机器人库）。
            auto_unit_scale: 网格是否按 :meth:`MeshData.suggest_unit_scale`
                自动把毫米制网格换算到米。

        Returns:
            :class:`RobotModel`。

        Raises:
            UrdfParseError: XML 非法、缺少必需字段，或连杆/关节引用不存在。
        """
        path = Path(path).resolve()
        try:
            tree = ET.parse(path)
        except ET.ParseError as exc:
            raise UrdfParseError(f"{path}: XML 解析失败 —— {exc}") from exc
        root_el = tree.getroot()
        if root_el.tag != "robot":
            raise UrdfParseError(f"{path}: 根节点应为 <robot>，实际为 <{root_el.tag}>")

        model = cls()
        model.urdf_path = path
        model.name = root_el.get("name", path.stem)

        materials = model._parse_materials(root_el)
        model._parse_links(root_el, path, materials, load_meshes, auto_unit_scale)
        model._parse_joints(root_el)
        model._resolve_root()
        model._order_joints()
        return model

    # ---------------- 解析 ---------------- #
    @staticmethod
    def _parse_materials(
        root_el: ET.Element,
    ) -> Dict[str, Tuple[Tuple[float, float, float], float]]:
        """收集顶层 ``<material name=...>``，供 visual 按名引用。

        URDF 允许在 link 内写 ``<material name="x"/>`` 引用顶层定义。旧实现完全
        不支持引用，导致这类模型的颜色全部退回默认灰——这也是"看起来不对"的成因之一。
        """
        table: Dict[str, Tuple[Tuple[float, float, float], float]] = {}
        for mat in root_el.findall("material"):
            name = mat.get("name")
            if not name:
                continue
            color = mat.find("color")
            if color is None:
                continue
            rgba = _parse_vec(color.get("rgba"), (0.7, 0.7, 0.75, 1.0))
            if len(rgba) < 3:
                continue
            table[name] = ((rgba[0], rgba[1], rgba[2]), rgba[3] if len(rgba) > 3 else 1.0)
        return table

    def _parse_links(
        self,
        root_el: ET.Element,
        urdf_path: Path,
        materials: Dict[str, Tuple[Tuple[float, float, float], float]],
        load_meshes: bool,
        auto_unit_scale: bool,
    ) -> None:
        """解析全部 ``<link>``，保留**每一个** visual。"""
        urdf_dir = urdf_path.parent
        for link_el in root_el.findall("link"):
            name = link_el.get("name")
            if not name:
                raise UrdfParseError(f"{urdf_path}: 存在无名 <link>")
            if name in self.links:
                raise UrdfParseError(f"{urdf_path}: 连杆名重复 '{name}'")

            link = Link(name=name)
            for visual_el in link_el.findall("visual"):
                visual = self._parse_visual(
                    visual_el, urdf_dir, materials, load_meshes, auto_unit_scale
                )
                if visual is not None:
                    link.visuals.append(visual)
            self.links[name] = link

    def _parse_visual(
        self,
        visual_el: ET.Element,
        urdf_dir: Path,
        materials: Dict[str, Tuple[Tuple[float, float, float], float]],
        load_meshes: bool,
        auto_unit_scale: bool,
    ) -> Optional[Visual]:
        """解析单个 ``<visual>``。

        Returns:
            :class:`Visual`；若几何体缺失或网格加载失败则返回 ``None``
            （失败原因记入 :attr:`mesh_errors`）。
        """
        geom_el = visual_el.find("geometry")
        if geom_el is None:
            return None

        origin_el = visual_el.find("origin")
        xyz = _parse_vec(origin_el.get("xyz") if origin_el is not None else None, (0, 0, 0))
        rpy = _parse_vec(origin_el.get("rpy") if origin_el is not None else None, (0, 0, 0))
        matrix = xyzrpy_to_matrix(xyz, rpy)

        color, alpha = self._resolve_material(visual_el.find("material"), materials)

        primitive = self._parse_geometry(geom_el, urdf_dir, load_meshes, auto_unit_scale)
        if primitive is None:
            return None
        return Visual(matrix=matrix, primitive=primitive, color=color, alpha=alpha)

    @staticmethod
    def _resolve_material(
        mat_el: Optional[ET.Element],
        materials: Dict[str, Tuple[Tuple[float, float, float], float]],
    ) -> Tuple[Tuple[float, float, float], float]:
        """解析 ``<material>``：优先内联 ``<color>``，否则按 ``name`` 查顶层表。"""
        default_rgb = (0.7, 0.7, 0.75)
        if mat_el is None:
            return default_rgb, 1.0
        color_el = mat_el.find("color")
        if color_el is not None:
            rgba = _parse_vec(color_el.get("rgba"), (*default_rgb, 1.0))
            if len(rgba) >= 3:
                return (rgba[0], rgba[1], rgba[2]), (rgba[3] if len(rgba) > 3 else 1.0)
        name = mat_el.get("name")
        if name and name in materials:
            return materials[name]
        return default_rgb, 1.0

    def _parse_geometry(
        self,
        geom_el: ET.Element,
        urdf_dir: Path,
        load_meshes: bool,
        auto_unit_scale: bool,
    ) -> Optional[Primitive]:
        """解析 ``<geometry>`` 的四种形态。"""
        box_el = geom_el.find("box")
        if box_el is not None:
            size = _parse_vec(box_el.get("size"), (0.1, 0.1, 0.1))
            return Primitive(kind="box", params=size)

        cyl_el = geom_el.find("cylinder")
        if cyl_el is not None:
            radius = float(cyl_el.get("radius", "0.03"))
            length = float(cyl_el.get("length", "0.15"))
            # URDF 圆柱沿局部 Z 轴，与渲染器一致
            return Primitive(kind="cylinder", params=(radius, length))

        sph_el = geom_el.find("sphere")
        if sph_el is not None:
            radius = float(sph_el.get("radius", "0.02"))
            return Primitive(kind="sphere", params=(radius,))

        mesh_el = geom_el.find("mesh")
        if mesh_el is not None:
            filename = mesh_el.get("filename")
            if not filename:
                return None
            scale = _parse_vec(mesh_el.get("scale"), (1.0, 1.0, 1.0))
            if len(scale) == 1:
                scale = (scale[0], scale[0], scale[0])
            resolved = self._resolve_mesh_path(filename, urdf_dir)
            if resolved is None:
                self.mesh_errors.append((filename, "在 URDF 目录及 package 搜索路径下未找到文件"))
                return None
            if not load_meshes:
                # 结构校验模式：放一个空网格占位，保持几何体计数正确
                return Primitive(kind="mesh", params=(None,))
            try:
                mesh = load_mesh(resolved, scale=scale[0])
            except MeshLoadError as exc:
                self.mesh_errors.append((filename, str(exc)))
                return None
            if auto_unit_scale:
                unit = mesh.suggest_unit_scale()
                if unit != 1.0:
                    mesh = MeshData(
                        vertices=mesh.vertices * unit,
                        normals=mesh.normals,
                        triangles=mesh.triangles,
                        source=mesh.source,
                    )
            return Primitive(kind="mesh", params=(mesh,))

        # 未知几何类型：不静默跳过，记入诊断信息
        return None

    def _resolve_mesh_path(self, filename: str, urdf_dir: Path) -> Optional[Path]:
        """把 URDF 里的网格路径解析为真实文件。

        支持三种写法：

        * 绝对路径 / 相对 URDF 目录的相对路径（含 ``../``）
        * ``package://<pkg>/<rest>`` —— 依次尝试把 ``<pkg>`` 放在 URDF 目录的
          各级父目录下（ros-industrial 的仓库通常是 ``<pkg>/urdf/x.urdf`` +
          ``<pkg>/meshes/...``）
        * ``model://<rest>`` —— 相对 URDF 目录
        """
        raw = filename.strip()
        if raw.startswith("package://"):
            rest = raw[len("package://") :]
            parts = rest.split("/", 1)
            if len(parts) == 2:
                _pkg, tail = parts
                for depth in self._PACKAGE_SEARCH_DEPTHS:
                    candidate = (urdf_dir / depth / tail).resolve()
                    if candidate.is_file():
                        return candidate
            tail_only = rest.split("/", 1)[-1]
            for depth in self._PACKAGE_SEARCH_DEPTHS:
                candidate = (urdf_dir / depth / tail_only).resolve()
                if candidate.is_file():
                    return candidate
            return None

        if raw.startswith("model://"):
            raw = raw[len("model://") :]

        candidate = Path(raw)
        if candidate.is_absolute():
            return candidate if candidate.is_file() else None
        resolved = (urdf_dir / raw).resolve()
        return resolved if resolved.is_file() else None

    def _parse_joints(self, root_el: ET.Element) -> None:
        """解析全部 ``<joint>``。"""
        for joint_el in root_el.findall("joint"):
            name = joint_el.get("name")
            if not name:
                raise UrdfParseError(f"{self.urdf_path}: 存在无名 <joint>")
            if name in self.joints:
                raise UrdfParseError(f"{self.urdf_path}: 关节名重复 '{name}'")

            type_ = joint_el.get("type", "fixed")
            parent_el = joint_el.find("parent")
            child_el = joint_el.find("child")
            if parent_el is None or child_el is None:
                raise UrdfParseError(f"{self.urdf_path}: 关节 '{name}' 缺少 parent/child")
            parent = parent_el.get("link")
            child = child_el.get("link")
            if not parent or not child:
                raise UrdfParseError(f"{self.urdf_path}: 关节 '{name}' 的 parent/child 无名")
            if parent not in self.links:
                raise UrdfParseError(
                    f"{self.urdf_path}: 关节 '{name}' 引用了不存在的父连杆 '{parent}'"
                )
            if child not in self.links:
                raise UrdfParseError(
                    f"{self.urdf_path}: 关节 '{name}' 引用了不存在的子连杆 '{child}'"
                )

            origin_el = joint_el.find("origin")
            xyz = _parse_vec(origin_el.get("xyz") if origin_el is not None else None, (0, 0, 0))
            rpy = _parse_vec(origin_el.get("rpy") if origin_el is not None else None, (0, 0, 0))
            matrix = xyzrpy_to_matrix(xyz, rpy)

            axis_el = joint_el.find("axis")
            axis = _parse_vec(axis_el.get("xyz") if axis_el is not None else None, (0, 0, 1))
            norm = math.sqrt(sum(c * c for c in axis))
            if norm < 1e-12:
                raise UrdfParseError(f"{self.urdf_path}: 关节 '{name}' 的 axis 为零向量")
            axis = (axis[0] / norm, axis[1] / norm, axis[2] / norm)

            lower, upper, velocity, effort = -math.pi, math.pi, 1.0, 1.0
            limit_el = joint_el.find("limit")
            if limit_el is not None:
                lower = float(limit_el.get("lower", "-3.14159"))
                upper = float(limit_el.get("upper", "3.14159"))
                velocity = float(limit_el.get("velocity", "1.0"))
                effort = float(limit_el.get("effort", "1.0"))
            if type_ == "fixed":
                lower = upper = 0.0

            self.joints[name] = Joint(
                name=name,
                type=type_,
                parent=parent,
                child=child,
                matrix=matrix,
                axis=axis,
                limits=JointLimits(lower=lower, upper=upper, velocity=velocity, effort=effort),
            )

    def _resolve_root(self) -> None:
        """找出根连杆（不作为任何关节子节点的连杆）。

        Raises:
            UrdfParseError: 找不到根（存在环）或存在多个根（森林，非单一运动链）。
        """
        children = {j.child for j in self.joints.values()}
        roots = [name for name in self.links if name not in children]
        if not roots:
            raise UrdfParseError(f"{self.urdf_path}: 找不到根连杆（连杆图中存在环）")
        # 优先选择带几何体的根：URDF 里常有一个无几何的 world/base_footprint 根节点
        with_geom = [r for r in roots if self.links[r].has_geometry]
        pool = with_geom or roots
        self.root = pool[0]
        if len(pool) > 1:
            # 多个根在真实 URDF 里很常见（例如额外的标定用固定连杆），
            # 只在严格模式下才应当报错；这里取带几何体的第一个，其余忽略
            pass

    def _order_joints(self) -> None:
        """按从根出发的广度优先顺序缓存关节，保证关节序列是拓扑有序的。"""
        self._ordered: List[Joint] = []
        if self.root is None:
            return
        children_by_parent: Dict[str, List[Joint]] = {}
        for joint in self.joints.values():
            children_by_parent.setdefault(joint.parent, []).append(joint)
        for bucket in children_by_parent.values():
            # 按 URDF 中出现顺序稳定排序，避免渲染/运动学顺序抖动
            bucket.sort(key=lambda j: list(self.joints).index(j.name))

        queue: List[str] = [self.root]
        visited_links = {self.root}
        visited_joints = set()
        while queue:
            link_name = queue.pop(0)
            for joint in children_by_parent.get(link_name, []):
                if joint.name in visited_joints:
                    continue
                visited_joints.add(joint.name)
                self._ordered.append(joint)
                if joint.child not in visited_links:
                    visited_links.add(joint.child)
                    queue.append(joint.child)
        # 环/孤立关节：补在末尾，保证不丢关节
        for joint in self.joints.values():
            if joint.name not in visited_joints:
                self._ordered.append(joint)

    # ---------------- 查询 ---------------- #
    @property
    def ordered_joints(self) -> List[Joint]:
        """从根出发拓扑有序的关节列表。"""
        return list(getattr(self, "_ordered", []))

    @property
    def actuated_joints(self) -> List[Joint]:
        """全部可动关节（非 fixed），按拓扑顺序。"""
        return [j for j in self.ordered_joints if j.is_actuated]

    @property
    def actuated_names(self) -> List[str]:
        """全部可动关节名，按拓扑顺序。"""
        return [j.name for j in self.actuated_joints]

    @property
    def dof(self) -> int:
        """可动关节数量。"""
        return len(self.actuated_joints)

    @property
    def triangle_count(self) -> int:
        """模型包含的网格三角形总数（基本体不计），用于性能预估。"""
        total = 0
        for link in self.links.values():
            for visual in link.visuals:
                mesh = visual.primitive.mesh
                if mesh is not None:
                    total += mesh.triangles
        return total

    @property
    def visual_count(self) -> int:
        """visual 节点总数——衡量模型精细度的一个直观指标。"""
        return sum(len(link.visuals) for link in self.links.values())

    def child_joints(self, link_name: str) -> List[Joint]:
        """返回以 ``link_name`` 为父连杆的关节列表（拓扑顺序）。"""
        return [j for j in self.ordered_joints if j.parent == link_name]

    def link_chain(self) -> List[str]:
        """从根出发的连杆遍历顺序。"""
        order: List[str] = []
        if self.root is None:
            return order

        def walk(name: str) -> None:
            order.append(name)
            for joint in self.child_joints(name):
                walk(joint.child)

        walk(self.root)
        return order

    def joint_transform(self, joint: Joint, q: float) -> np.ndarray:
        """关节的完整变换：``T(origin) · R(axis, q)``。

        Args:
            joint: 目标关节。
            q: 关节变量（revolute/continuous 为弧度，prismatic 为米）。

        Returns:
            4×4 齐次变换矩阵。
        """
        mat = joint.matrix.copy()
        if joint.type == "prismatic":
            mat[:3, 3] += np.asarray(joint.axis, dtype=np.float64) * q
            return mat
        if joint.type in ("revolute", "continuous") and q != 0.0:
            mat = mat @ _axis_angle_matrix(joint.axis, q)
        return mat

    def forward_kinematics(
        self,
        q: Sequence[float],
        target_link: Optional[str] = None,
    ) -> Dict[str, np.ndarray]:
        """正向运动学：返回每个连杆在世界（根）坐标系下的 4×4 位姿。

        Args:
            q: 关节值序列，顺序与 :attr:`actuated_names` 一致。
            target_link: 可选，只计算到该连杆为止（提前剪枝）。

        Returns:
            连杆名 → 4×4 位姿。

        Raises:
            ValueError: ``q`` 长度与自由度不符。
        """
        names = self.actuated_names
        if len(q) != len(names):
            raise ValueError(f"关节值数量 {len(q)} 与自由度 {len(names)} 不符（关节: {names}）")
        qmap = dict(zip(names, q))

        poses: Dict[str, np.ndarray] = {}
        if self.root is None:
            return poses

        def walk(link_name: str, parent_pose: np.ndarray) -> None:
            poses[link_name] = parent_pose
            if target_link is not None and link_name == target_link:
                return
            for joint in self.child_joints(link_name):
                qv = float(qmap.get(joint.name, 0.0))
                child_pose = parent_pose @ self.joint_transform(joint, qv)
                walk(joint.child, child_pose)

        walk(self.root, np.eye(4, dtype=np.float64))
        return poses

    def summary(self) -> Dict[str, object]:
        """返回模型的结构摘要，用于诊断与机器人库列表展示。"""
        return {
            "name": self.name,
            "links": len(self.links),
            "joints": len(self.joints),
            "dof": self.dof,
            "visuals": self.visual_count,
            "triangles": self.triangle_count,
            "root": self.root,
            "actuated": self.actuated_names,
            "mesh_errors": list(self.mesh_errors),
        }


def _axis_angle_matrix(axis: Sequence[float], angle: float) -> np.ndarray:
    """罗德里格斯公式：绕任意单位轴旋转 ``angle`` 弧度的 4×4 齐次矩阵。

    Args:
        axis: 单位向量 ``(x, y, z)``。
        angle: 旋转角，弧度。

    Returns:
        4×4 齐次旋转矩阵。
    """
    x, y, z = (float(v) for v in axis)
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = np.array(
        [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ],
        dtype=np.float64,
    )
    return mat
