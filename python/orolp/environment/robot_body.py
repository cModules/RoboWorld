"""机器人碰撞体：从 URDF 抽取连杆几何，按关节角计算世界位姿。

与 :mod:`orolp.kinematics` 的分工：

* ``kinematics`` 只关心**运动链**（可动关节），服务于正/逆解；
* ``robot_body`` 关心**全部连杆**（含固定关节），服务于碰撞检测与渲染。

固定关节不能丢——法兰上的 TCP、夹具、焊枪都是 fixed 关节挂上去的，
丢了它们等于机器人末端"看不见"自己撞了什么。因此本模块自带一次
完整的运动树遍历，而不复用 ``link_transforms()``。

几何来源优先级：``<collision>`` > ``<visual>``。URDF 规范中两者可以不同
（collision 常做简化），有 collision 就优先用；只有 visual 的 URDF（本仓库
程序化生成的 27 个型号即如此）自动回退。

``<mesh>`` 暂不参与碰撞：加载 STL 需要额外的网格库，且厂商网格往往是
非凸的。遇到 mesh 连杆会记入 :attr:`RobotBody.skipped_links`，调用方可据此
提示用户"该型号碰撞模型不完整"。
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .geometry import BoxGeometry, ConvexGeometry, CylinderGeometry, SphereGeometry

_DEFAULT_COLOR = (0.16, 0.16, 0.19)


def _vec3(text: Optional[str], default: Sequence[float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    """解析以空白或逗号分隔的浮点串，取前 ``len(default)`` 个。"""
    parts = (text or "").replace(",", " ").split()
    if len(parts) < len(default):
        return np.array(default, dtype=np.float64)
    return np.array([float(p) for p in parts[: len(default)]], dtype=np.float64)


def _rotmat_rpy(rpy: np.ndarray) -> np.ndarray:
    """rpy（弧度，XYZ 内旋）转旋转矩阵。"""
    rx, ry, rz = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _rot_about_axis(axis: np.ndarray, angle: float) -> np.ndarray:
    """绕任意单位轴旋转（Rodrigues 公式）。

    Args:
        axis: 旋转轴（无需归一化，内部会处理）
        angle: 弧度
    """
    a = np.asarray(axis, dtype=np.float64).ravel()
    n = np.linalg.norm(a)
    if n < 1e-12:
        return np.eye(3)
    u = a / n
    x, y, z = u
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)


def _transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """由旋转矩阵与平移构造 4x4 齐次矩阵。"""
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = np.asarray(t, dtype=np.float64).ravel()
    return M


@dataclass
class LinkBody:
    """一个连杆上的碰撞体。

    Attributes:
        name: 连杆名
        geometry: 凸体几何（局部坐标，米）
        T_origin: 几何相对连杆坐标系的 4x4 变换（即 URDF 的 ``<origin>``）
        color: 显示颜色 RGB
        parent_joint: 挂载该连杆的关节名；根连杆为 ``None``
    """

    name: str
    geometry: ConvexGeometry
    T_origin: np.ndarray
    color: Tuple[float, float, float]
    parent_joint: Optional[str]

    @property
    def local_radius(self) -> float:
        """含原点偏移的外接球半径，用于粗判剔除。"""
        offset = float(np.linalg.norm(self.T_origin[:3, 3]))
        return self.geometry.bounding_radius + offset


@dataclass
class _Joint:
    """内部用的关节记录。"""

    name: str
    type: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


class RobotBody:
    """机器人的碰撞体模型。

    Example:
        >>> body = RobotBody.from_urdf("models/kuka/kr6_r900/kr6_r900.urdf")
        >>> body.dof
        6
        >>> bodies = body.link_bodies_at(np.zeros(body.dof))
        >>> len(bodies) > 0
        True
    """

    def __init__(
        self,
        urdf_path: str | Path,
        joints: Sequence[_Joint],
        links: Sequence[LinkBody],
        root_link: str,
    ):
        """
        Args:
            urdf_path: URDF 文件路径
            joints: 全部关节
            links: 全部连杆碰撞体
            root_link: 根连杆名
        """
        self.urdf_path = Path(urdf_path)
        self._joints: List[_Joint] = list(joints)
        self._links: List[LinkBody] = list(links)
        self.root_link = root_link
        self.skipped_links: List[str] = []

    # ------------------------------------------------------------------ #
    # 构造
    # ------------------------------------------------------------------ #
    @classmethod
    def from_urdf(cls, urdf_path: str | Path) -> "RobotBody":
        """解析 URDF 并构造碰撞体模型。

        Args:
            urdf_path: URDF 文件路径

        Returns:
            :class:`RobotBody`

        Raises:
            FileNotFoundError: 文件不存在
            ValueError: XML 解析失败，或没有任何可用连杆
        """
        path = Path(urdf_path)
        if not path.is_file():
            raise FileNotFoundError(f"URDF 不存在：{path}")
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as exc:
            raise ValueError(f"URDF 解析失败：{path} — {exc}") from exc

        skipped: List[str] = []
        links: List[LinkBody] = []
        for link_el in root.findall("link"):
            name = link_el.get("name")
            if name is None:
                continue
            color = _DEFAULT_COLOR
            visual = link_el.find("visual")
            if visual is not None:
                mat = visual.find("material")
                col_el = mat.find("color") if mat is not None else None
                if col_el is not None:
                    rgba = _vec3(col_el.get("rgba", ""), (_DEFAULT_COLOR[0], _DEFAULT_COLOR[1], _DEFAULT_COLOR[2]))
                    color = (float(rgba[0]), float(rgba[1]), float(rgba[2]))

            # collision 优先，无则回退 visual
            source = link_el.find("collision")
            if source is None:
                source = visual
            if source is None:
                skipped.append(name)
                continue

            geom_el = source.find("geometry")
            if geom_el is None:
                skipped.append(name)
                continue

            try:
                geometry = _parse_geometry(geom_el)
            except GeometryUnsupportedError:
                skipped.append(name)
                continue

            origin = source.find("origin")
            xyz = _vec3(origin.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0))
            rpy = _vec3(origin.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0))
            links.append(
                LinkBody(name, geometry, _transform(_rotmat_rpy(rpy), xyz), color, None)
            )

        joints: List[_Joint] = []
        for j_el in root.findall("joint"):
            jname = j_el.get("name")
            parent_el, child_el = j_el.find("parent"), j_el.find("child")
            if jname is None or parent_el is None or child_el is None:
                continue
            origin = j_el.find("origin")
            axis_el = j_el.find("axis")
            limit_el = j_el.find("limit")
            joints.append(
                _Joint(
                    name=jname,
                    type=j_el.get("type", "fixed"),
                    parent=parent_el.get("link", ""),
                    child=child_el.get("link", ""),
                    xyz=_vec3(origin.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0)),
                    rpy=_vec3(origin.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0)),
                    axis=_vec3(axis_el.get("xyz") if axis_el is not None else None, (0.0, 0.0, 1.0)),
                    lower=float(limit_el.get("lower", "-1e9")) if limit_el is not None else -1e9,
                    upper=float(limit_el.get("upper", "1e9")) if limit_el is not None else 1e9,
                )
            )

        if not links:
            raise ValueError(f"URDF 中没有任何可用（非 mesh）连杆几何：{path}")

        # 回填 parent_joint
        joint_by_child = {j.child: j.name for j in joints}
        for lb in links:
            lb.parent_joint = joint_by_child.get(lb.name)

        child_links = {j.child for j in joints}
        root_link = next((lb.name for lb in links if lb.name not in child_links), links[0].name)

        body = cls(path, joints, links, root_link)
        body.skipped_links = skipped
        return body

    # ------------------------------------------------------------------ #
    # 查询
    # ------------------------------------------------------------------ #
    @property
    def movable_joints(self) -> List[_Joint]:
        """可动关节（revolute / continuous / prismatic）列表。"""
        return [j for j in self._joints if j.type in ("revolute", "continuous", "prismatic")]

    @property
    def joint_names(self) -> List[str]:
        """可动关节名，顺序即关节向量的顺序。"""
        return [j.name for j in self.movable_joints]

    @property
    def dof(self) -> int:
        """自由度。"""
        return len(self.movable_joints)

    @property
    def link_names(self) -> List[str]:
        """参与碰撞的连杆名。"""
        return [lb.name for lb in self._links]

    def limits(self) -> Tuple[np.ndarray, np.ndarray]:
        """关节上下限（弧度）。

        Returns:
            ``(lower, upper)``，各为长度 :attr:`dof` 的数组
        """
        joints = self.movable_joints
        lower = np.array([j.lower for j in joints], dtype=np.float64)
        upper = np.array([j.upper for j in joints], dtype=np.float64)
        return lower, upper

    # ------------------------------------------------------------------ #
    # 运动学
    # ------------------------------------------------------------------ #
    def link_frames(self, q: Sequence[float], q_is_radians: bool = True) -> Dict[str, np.ndarray]:
        """遍历**完整运动树**（含固定关节）计算每个连杆坐标系的世界变换。

        Args:
            q: 可动关节值，长度须等于 :attr:`dof`
            q_is_radians: ``True`` 表示旋转关节为弧度；``False`` 表示度

        Returns:
            连杆名 -> 4x4 齐次矩阵（米）

        Raises:
            ValueError: q 长度与自由度不符
        """
        q = np.asarray(q, dtype=np.float64).ravel()
        movable = self.movable_joints
        if q.size != len(movable):
            raise ValueError(f"期望 {len(movable)} 个关节值，收到 {q.size}")
        values = {j.name: float(q[i]) for i, j in enumerate(movable)}

        frames: Dict[str, np.ndarray] = {self.root_link: np.eye(4)}
        pending = list(self._joints)
        while pending:
            progressed = False
            for joint in list(pending):
                if joint.parent not in frames:
                    continue
                T = frames[joint.parent] @ _transform(_rotmat_rpy(joint.rpy), joint.xyz)
                if joint.type in ("revolute", "continuous"):
                    angle = values.get(joint.name, 0.0)
                    if not q_is_radians:
                        angle = math.radians(angle)
                    T = T @ _transform(_rot_about_axis(joint.axis, angle), np.zeros(3))
                elif joint.type == "prismatic":
                    T = T @ _transform(
                        np.eye(3), np.asarray(joint.axis, dtype=np.float64).ravel() * values.get(joint.name, 0.0)
                    )
                frames[joint.child] = T
                pending.remove(joint)
                progressed = True
            if not progressed:
                # 断链（父连杆不在树中）——放弃剩余关节，避免死循环
                break
        return frames

    def link_bodies_at(
        self,
        q: Sequence[float],
        q_is_radians: bool = True,
        base_T: Optional[np.ndarray] = None,
    ) -> List[Tuple[LinkBody, np.ndarray]]:
        """计算机器人在给定关节角下各碰撞体的世界变换。

        Args:
            q: 可动关节值
            q_is_radians: 旋转关节是否为弧度
            base_T: 机器人基座的世界变换（米）；``None`` 表示单位阵

        Returns:
            ``[(LinkBody, 4x4 世界变换), ...]``，已剔除不属于运动树的连杆
        """
        frames = self.link_frames(q, q_is_radians)
        base = np.eye(4) if base_T is None else np.asarray(base_T, dtype=np.float64)
        out: List[Tuple[LinkBody, np.ndarray]] = []
        for lb in self._links:
            frame = frames.get(lb.name)
            if frame is None:
                continue  # 断链连杆，无法定位
            out.append((lb, base @ frame @ lb.T_origin))
        return out

    def __repr__(self) -> str:
        extra = f", 跳过 {len(self.skipped_links)} 个 mesh 连杆" if self.skipped_links else ""
        return f"RobotBody({self.urdf_path.name}, dof={self.dof}, {len(self._links)} 连杆{extra})"


class GeometryUnsupportedError(ValueError):
    """URDF 中的几何类型不受支持（如 ``<mesh>``）。"""


def _parse_geometry(geom_el: ET.Element) -> ConvexGeometry:
    """把 URDF 的 ``<geometry>`` 节点转成 :class:`ConvexGeometry`。

    Args:
        geom_el: ``<geometry>`` XML 节点

    Returns:
        凸体几何

    Raises:
        GeometryUnsupportedError: 无可识别图元，或为 ``<mesh>``
    """
    if geom_el.find("box") is not None:
        size = _vec3(geom_el.find("box").get("size"), (0.1, 0.1, 0.1))
        return BoxGeometry(tuple(float(v) for v in size))

    if geom_el.find("cylinder") is not None:
        cyl = geom_el.find("cylinder")
        radius = float(cyl.get("radius", "0.05"))
        length = float(cyl.get("length", "0.1"))
        return CylinderGeometry(radius, length)

    if geom_el.find("sphere") is not None:
        radius = float(geom_el.find("sphere").get("radius", "0.05"))
        return SphereGeometry(radius)

    if geom_el.find("mesh") is not None:
        raise GeometryUnsupportedError("mesh 几何暂不参与碰撞检测")

    raise GeometryUnsupportedError("未识别的几何类型")
