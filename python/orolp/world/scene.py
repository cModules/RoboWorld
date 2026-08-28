"""工作单元场景图：对象层级、坐标系、碰撞检测与持久化。

一个场景（Scene）= 若干 :class:`SceneObject` + 一个可选的机器人。
对象之间可以有父子关系（例如工件放在变位机台面上，随台面一起运动）。

碰撞检测把**机器人连杆**也纳入进来：机器人按当前关节位形，把 URDF 中的
``<visual>`` 基本体通过正运动学变换到世界系，与场景对象逐一求交。
这正是 OLP 的核心价值——在离线阶段就发现干涉，而不是到现场撞机。

单位：毫米（与 :class:`~orolp.pose.Pose` 一致）。
"""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..pose import Pose
from .components import Part, build_component, get_component
from .geometry import Box, Cylinder, Sphere, collides, prim_aabb

__all__ = [
    "SceneObject",
    "Collision",
    "Scene",
    "load_robot_visuals",
]

#: 场景文件格式版本，用于将来兼容旧文件
SCENE_FORMAT_VERSION = 1


@dataclass
class SceneObject:
    """场景中的一个对象。

    Attributes:
        name: 唯一名称（场景内不重复）
        component_id: 来源组件 id；``None`` 表示外部导入的裸几何
        params: 组件参数
        pose: 相对父对象的位姿
        parent: 父对象名；``None`` 表示挂在场景根下
        visible: 是否可见
        locked: 是否锁定（不可拖动）
        color: 颜色覆盖；``None`` 表示使用组件自带配色
        parts: 直接提供的几何（component_id 为 ``None`` 时使用）
    """

    name: str
    component_id: Optional[str] = None
    params: Dict[str, float] = field(default_factory=dict)
    pose: Pose = field(default_factory=Pose)
    parent: Optional[str] = None
    visible: bool = True
    locked: bool = False
    color: Optional[Tuple[float, float, float]] = None
    parts: Optional[List[Part]] = None


@dataclass(frozen=True)
class Collision:
    """一次碰撞检测结果。

    Attributes:
        obj_a: 第一个对象名（``robot`` 表示机器人）
        obj_b: 第二个对象名
        part_a: 第一个对象中发生干涉的零件序号
        part_b: 第二个对象中发生干涉的零件序号
        time: 沿轨迹检查时**首次**检出该干涉的时刻（秒）。静态检查（
            :meth:`Scene.check_collisions`）不填，保持 ``None``
    """

    obj_a: str
    obj_b: str
    part_a: int = -1
    part_b: int = -1
    time: Optional[float] = None

    def __str__(self) -> str:
        return f"{self.obj_a} ↔ {self.obj_b}"


# --------------------------------------------------------------------------- #
# URDF 视觉几何解析
# --------------------------------------------------------------------------- #


def _vec3(text: Optional[str], default: Sequence[float]) -> np.ndarray:
    """解析空白/逗号分隔浮点串，兼容 3 分量 (xyz/size) 与 4 分量 (rgba)。"""
    raw = (text or "").replace(",", " ").split()
    if len(raw) < len(default):
        return np.array(default, dtype=np.float64)
    return np.array([float(v) for v in raw[: len(default)]], dtype=np.float64)


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    """固定轴 XYZ 序欧拉角转 3x3。"""
    from scipy.spatial.transform import Rotation

    return Rotation.from_euler("XYZ", rpy).as_matrix()


def load_robot_visuals(urdf_path: str | Path) -> List[Tuple[str, Part]]:
    """解析 URDF 中的 ``<visual>`` 基本体。

    只支持 box / cylinder / sphere——与 :mod:`orolp.robot_library.urdf_builder`
    生成的内容一致。``<mesh>`` 会被跳过（GUI 渲染器同样不解析）。

    Args:
        urdf_path: URDF 文件路径

    Returns:
        ``(link 名, Part)`` 列表，几何位于**对应 link 的局部坐标系**

    Raises:
        OSError: 文件不存在
        ET.ParseError: XML 非法
    """
    root = ET.parse(urdf_path).getroot()
    out: List[Tuple[str, Part]] = []

    for link_el in root.findall("link"):
        link_name = link_el.get("name", "")
        for vis in link_el.findall("visual"):
            origin_el = vis.find("origin")
            xyz = _vec3(origin_el.get("xyz") if origin_el is not None else None, (0, 0, 0))
            rpy = _vec3(origin_el.get("rpy") if origin_el is not None else None, (0, 0, 0))

            color = (0.6, 0.6, 0.65)
            color_el = vis.find("material/color")
            if color_el is not None:
                rgba = _vec3(color_el.get("rgba"), (0.6, 0.6, 0.65))
                color = (float(rgba[0]), float(rgba[1]), float(rgba[2]))

            geom = vis.find("geometry")
            if geom is None:
                continue
            tf = np.eye(4)
            tf[:3, :3] = _rpy_matrix(rpy)
            tf[:3, 3] = xyz * 1000.0  # URDF 用米，场景用毫米

            box = geom.find("box")
            cyl = geom.find("cylinder")
            sph = geom.find("sphere")
            if box is not None:
                size = _vec3(box.get("size"), (0.1, 0.1, 0.1)) * 1000.0
                out.append((link_name, Part(Box(tuple(size)), tf, color)))
            elif cyl is not None:
                r = float(cyl.get("radius", 0.05)) * 1000.0
                length = float(cyl.get("length", 0.1)) * 1000.0
                out.append((link_name, Part(Cylinder(r, length), tf, color)))
            elif sph is not None:
                r = float(sph.get("radius", 0.02)) * 1000.0
                out.append((link_name, Part(Sphere(r), tf, color)))
    return out


# --------------------------------------------------------------------------- #
# 场景
# --------------------------------------------------------------------------- #


class Scene:
    """工作单元场景。

    Attributes:
        objects: 对象字典，键为对象名
        robot_urdf: 当前机器人的 URDF 路径；``None`` 表示未加载
    """

    def __init__(self) -> None:
        self.objects: Dict[str, SceneObject] = {}
        self.robot_urdf: Optional[str] = None
        self._robot_joints_deg: List[float] = []
        self._robot_visuals: List[Tuple[str, Part]] = []
        self._kinematics = None
        self._name_counter: Dict[str, int] = {}

    # --------------------------- 对象管理 --------------------------- #

    def _unique_name(self, base: str) -> str:
        """生成不重复的对象名（如 ``工作台``、``工作台_2``）。"""
        if base not in self.objects:
            return base
        n = self._name_counter.get(base, 1) + 1
        while f"{base}_{n}" in self.objects:
            n += 1
        self._name_counter[base] = n
        return f"{base}_{n}"

    def add(
        self,
        component_id: str,
        pose: Optional[Pose] = None,
        name: Optional[str] = None,
        params: Optional[Dict[str, float]] = None,
        parent: Optional[str] = None,
    ) -> SceneObject:
        """往场景里加一个组件。

        Args:
            component_id: 组件 id
            pose: 位姿；``None`` 表示放在原点
            name: 对象名；``None`` 时按组件名自动编号
            params: 组件参数；``None`` 取默认值
            parent: 父对象名

        Returns:
            新建的对象

        Raises:
            KeyError: 组件 id 不存在
            ValueError: 父对象不存在
        """
        spec = get_component(component_id)
        if parent is not None and parent not in self.objects:
            raise ValueError(f"父对象不存在: {parent}")

        final_name = self._unique_name(name or spec.name)
        obj = SceneObject(
            name=final_name,
            component_id=component_id,
            params=dict(params or {}),
            pose=pose or Pose(),
            parent=parent,
        )
        self.objects[final_name] = obj
        return obj

    def remove(self, name: str) -> None:
        """删除对象，其所有子对象一并删除。

        Args:
            name: 对象名

        Raises:
            KeyError: 对象不存在
        """
        if name not in self.objects:
            raise KeyError(name)
        # 先把直接子对象重新挂到祖父上，避免"删除父对象导致子对象悬空"
        obj = self.objects[name]
        for child in [o for o in self.objects.values() if o.parent == name]:
            child.parent = obj.parent
        del self.objects[name]

    def clear(self) -> None:
        """清空所有对象（保留机器人）。"""
        self.objects.clear()
        self._name_counter.clear()

    # --------------------------- 变换 --------------------------- #

    def world_transform(self, name: str) -> np.ndarray:
        """对象在世界坐标系下的 4x4 变换（含父级链）。

        Args:
            name: 对象名

        Returns:
            4x4 齐次矩阵

        Raises:
            KeyError: 对象不存在
        """
        obj = self.objects[name]
        local = obj.pose.matrix
        if obj.parent is None:
            return local
        return self.world_transform(obj.parent) @ local

    def object_parts(self, name: str) -> List[Part]:
        """对象在世界坐标系下的零件几何。

        Args:
            name: 对象名

        Returns:
            :class:`Part` 列表（世界系）
        """
        obj = self.objects[name]
        tf = self.world_transform(name)

        if obj.component_id is not None:
            local_parts = build_component(obj.component_id, **obj.params)
        else:
            local_parts = list(obj.parts or [])

        out: List[Part] = []
        for p in local_parts:
            color = obj.color or p.color
            out.append(Part(p.prim, tf @ p.transform, color))
        return out

    # --------------------------- 机器人 --------------------------- #

    def set_robot(self, urdf_path: str | Path, joints_deg: Optional[Sequence[float]] = None) -> None:
        """设置场景中的机器人。

        Args:
            urdf_path: URDF 路径
            joints_deg: 关节角（**度**）；``None`` 时取零位

        Raises:
            OSError: URDF 不存在
            ET.ParseError: URDF 非法
        """
        self.robot_urdf = str(urdf_path)
        self._robot_visuals = load_robot_visuals(self.robot_urdf)

        from ..kinematics import RobotKinematics

        self._kinematics = RobotKinematics(self.robot_urdf)
        n = self._kinematics.dof
        self._robot_joints_deg = list(joints_deg) if joints_deg else [0.0] * n

        if not self._robot_visuals:
            # 不抛异常——机器人仍可显示与编程，只是无法参与干涉检查。
            # 但必须让用户知道，否则会误以为"检查通过 = 不会撞机"。
            print(
                f"[warning] {self.robot_urdf}: 未解析到任何 box/cylinder/sphere 视觉体"
                f"（多为 <mesh> 几何），该机器人将不参与干涉检查"
            )

    def set_robot_joints(self, joints_deg: Sequence[float]) -> None:
        """更新机器人关节角。

        Args:
            joints_deg: 关节角（**度**）
        """
        self._robot_joints_deg = [float(v) for v in joints_deg]

    @property
    def has_robot(self) -> bool:
        """场景中是否已加载机器人。"""
        return self._kinematics is not None

    def robot_parts(self) -> List[Part]:
        """机器人当前位形下的连杆几何（世界系）。

        Returns:
            :class:`Part` 列表；未加载机器人时为空列表
        """
        if self._kinematics is None:
            return []
        # kinematics 内部用米；此处换算到场景的毫米
        q_rad = np.radians(self._robot_joints_deg)
        # 用完整运动树而非主链：fixed 关节挂的工具/夹爪/TCP 也必须参与干涉检查
        link_tf = self._kinematics.all_link_transforms(q_rad)

        out: List[Part] = []
        for link_name, part in self._robot_visuals:
            T = link_tf.get(link_name)
            if T is None:
                continue
            tf_mm = T.copy()
            tf_mm[:3, 3] *= 1000.0
            out.append(Part(part.prim, tf_mm @ part.transform, part.color))
        return out

    # --------------------------- 碰撞检测 --------------------------- #

    def check_collisions(
        self,
        include_robot: bool = True,
        margin: float = 0.0,
    ) -> List[Collision]:
        """检测场景中所有对象之间（含机器人）的碰撞。

        Args:
            include_robot: 是否把机器人连杆计入
            margin: 安全裕度（毫米）

        Returns:
            :class:`Collision` 列表；空列表表示无干涉
        """
        names = [n for n, o in self.objects.items() if o.visible]

        # 预先算出所有世界系零件，避免重复计算
        cache: Dict[str, List[Part]] = {n: self.object_parts(n) for n in names}
        robot: List[Part] = self.robot_parts() if (include_robot and self.has_robot) else []

        results: List[Collision] = []

        # 机器人 vs 场景对象
        if robot:
            for name in names:
                hit = _parts_collide(robot, cache[name], margin)
                if hit is not None:
                    ia, ib = hit
                    results.append(Collision("robot", name, ia, ib))

        # 场景对象两两之间
        for i, na in enumerate(names):
            for nb in names[i + 1 :]:
                hit = _parts_collide(cache[na], cache[nb], margin)
                if hit is not None:
                    ia, ib = hit
                    results.append(Collision(na, nb, ia, ib))

        return results

    # --------------------------- 持久化 --------------------------- #

    def to_dict(self) -> Dict[str, Any]:
        """序列化为可 JSON 化的字典。"""
        return {
            "format_version": SCENE_FORMAT_VERSION,
            "robot_urdf": self.robot_urdf,
            "robot_joints_deg": self._robot_joints_deg,
            "objects": [
                {
                    "name": o.name,
                    "component_id": o.component_id,
                    "params": o.params,
                    "pose": o.pose.matrix.flatten().tolist(),
                    "parent": o.parent,
                    "visible": o.visible,
                    "locked": o.locked,
                    "color": list(o.color) if o.color else None,
                }
                for o in self.objects.values()
            ],
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Scene":
        """从字典恢复场景。

        Args:
            data: :meth:`to_dict` 的输出

        Returns:
            恢复后的场景
        """
        scene = cls()
        urdf = data.get("robot_urdf")
        if urdf and Path(urdf).exists():
            scene.set_robot(urdf, data.get("robot_joints_deg") or None)
        elif urdf:
            # URDF 路径失效时保留记录，但不再构建运动学链
            scene.robot_urdf = urdf

        for raw in data.get("objects", []):
            obj = SceneObject(
                name=raw["name"],
                component_id=raw.get("component_id"),
                params=dict(raw.get("params") or {}),
                pose=Pose(np.array(raw["pose"], dtype=np.float64).reshape(4, 4)),
                parent=raw.get("parent"),
                visible=bool(raw.get("visible", True)),
                locked=bool(raw.get("locked", False)),
                color=tuple(raw["color"]) if raw.get("color") else None,
            )
            scene.objects[obj.name] = obj
        return scene

    def save(self, path: str | Path) -> None:
        """保存场景到 JSON 文件。

        Args:
            path: 目标路径

        Raises:
            OSError: 写入失败
        """
        Path(path).write_text(
            json.dumps(self.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8"
        )

    @classmethod
    def load(cls, path: str | Path) -> "Scene":
        """从 JSON 文件加载场景。

        Args:
            path: 场景文件路径

        Returns:
            场景对象

        Raises:
            OSError: 读取失败
            json.JSONDecodeError: 文件不是合法 JSON
        """
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    # --------------------------- 统计 --------------------------- #

    def bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """场景整体包围盒 ``(lo, hi)``（含机器人）。"""
        lo = np.full(3, np.inf)
        hi = np.full(3, -np.inf)
        for name in self.objects:
            for p in self.object_parts(name):
                bb = prim_aabb(p.prim, p.transform)
                lo = np.minimum(lo, bb.lo)
                hi = np.maximum(hi, bb.hi)
        for p in self.robot_parts():
            bb = prim_aabb(p.prim, p.transform)
            lo = np.minimum(lo, bb.lo)
            hi = np.maximum(hi, bb.hi)
        if not np.isfinite(lo).all():
            return np.zeros(3), np.zeros(3)
        return lo, hi

    def __len__(self) -> int:
        return len(self.objects)

    def __repr__(self) -> str:
        return f"Scene(objects={list(self.objects)}, robot={self.robot_urdf!r})"


def _parts_collide(
    parts_a: Sequence[Part],
    parts_b: Sequence[Part],
    margin: float = 0.0,
) -> Optional[Tuple[int, int]]:
    """两组零件之间是否存在碰撞。

    Args:
        parts_a: 第一组零件（世界系）
        parts_b: 第二组零件（世界系）
        margin: 安全裕度

    Returns:
        发生碰撞的 ``(索引a, 索引b)``；无碰撞返回 ``None``
    """
    for i, pa in enumerate(parts_a):
        for j, pb in enumerate(parts_b):
            if collides(pa.prim, pa.transform, pb.prim, pb.transform, margin):
                return (i, j)
    return None
