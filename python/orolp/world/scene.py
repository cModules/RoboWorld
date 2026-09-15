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
    "RobotState",
    "Collision",
    "Scene",
    "load_robot_visuals",
    "load_robot_collision_geometry",
    "ROBOT_NAME",
    "GEOMETRY_VISUAL",
    "GEOMETRY_COLLISION",
]

#: 场景文件格式版本，用于将来兼容旧文件
SCENE_FORMAT_VERSION = 1

#: 机器人在场景对象字典中的保留名（作为真实 SceneObject 存在）
ROBOT_NAME = "机器人"

#: 机器人几何用途：渲染（``<visual>``）
GEOMETRY_VISUAL = "visual"

#: 机器人几何用途：干涉检查（``<collision>``，缺失时回退 visual）
GEOMETRY_COLLISION = "collision"


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
        link_a: 连杆级检查（:mod:`orolp.collision`）时，机器人侧的连杆名；
            整机级检查不填，保持 ``None``
        link_b: 自碰撞（连杆-连杆）时第二个连杆名；其余场景为 ``None``
        motion: 沿轨迹检查时干涉所在**指令序号**（0 起，由规划器的
            ``motion_at`` 回填）；静态/无定位信息时为 ``None``
    """

    obj_a: str
    obj_b: str
    part_a: int = -1
    part_b: int = -1
    time: Optional[float] = None
    link_a: Optional[str] = None
    link_b: Optional[str] = None
    motion: Optional[int] = None

    def __str__(self) -> str:
        a = f"{self.obj_a}:{self.link_a}" if self.link_a else self.obj_a
        b = f"{self.obj_b}:{self.link_b}" if self.link_b else self.obj_b
        return f"{a} ↔ {b}"


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


def _parse_link_primitives(
    urdf_path: str | Path, tags: Sequence[str]
) -> List[Tuple[str, Part]]:
    """解析 URDF 中指定标签（``visual`` / ``collision``）下的基本体。

    只支持 box / cylinder / sphere——与 :mod:`orolp.robot_library.urdf_builder`
    生成的内容一致。``<mesh>`` 会被跳过（场景渲染器同样不解析）。

    Args:
        urdf_path: URDF 文件路径
        tags: 要解析的块标签，按顺序取

    Returns:
        ``(link 名, Part)`` 列表，几何位于**对应 link 的局部坐标系**
    """
    root = ET.parse(urdf_path).getroot()
    out: List[Tuple[str, Part]] = []

    for link_el in root.findall("link"):
        link_name = link_el.get("name", "")
        for tag in tags:
            for vis in link_el.findall(tag):
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


def load_robot_visuals(urdf_path: str | Path) -> List[Tuple[str, Part]]:
    """解析 URDF 中的 ``<visual>`` 基本体（**渲染**用几何）。

    Args:
        urdf_path: URDF 文件路径

    Returns:
        ``(link 名, Part)`` 列表，几何位于**对应 link 的局部坐标系**

    Raises:
        OSError: 文件不存在
        ET.ParseError: XML 非法
    """
    return _parse_link_primitives(urdf_path, ("visual",))


def load_robot_collision_geometry(
    urdf_path: str | Path,
) -> List[Tuple[str, Part]]:
    """解析 URDF 中的 ``<collision>`` 基本体（**干涉检查**用几何）。

    为什么要单独一套
    ----------------
    真实机器人 URDF 里 ``<visual>`` 基本都是 ``<mesh>``（几十个 STL 三角面），
    ``<collision>`` 才是厂商给的**简化包络体**（UR5e 是 7 个圆柱/长方体）。
    早期实现只读 ``<visual>`` 且跳过 mesh，于是整个机器人只剩一个 TCP 标记球
    参与干涉检查——"检查通过"并不代表不会撞。多机器人互避碰更是完全查不出
    （两台机器人各自的 TCP 球几乎不可能重合）。

    本函数专供碰撞通道；渲染仍用 :func:`load_robot_visuals`，两不相干。

    链接级回退：某 link 没有 ``<collision>`` 时（如 UR5e 的 ``tcp``/``tool0``
    只有视觉球），退用它的 ``<visual>`` 基本体，避免工具末端从检查里消失。

    Args:
        urdf_path: URDF 文件路径

    Returns:
        ``(link 名, Part)`` 列表（link 局部系）；URDF 无任何基本体时为空列表
    """
    coll = _parse_link_primitives(urdf_path, ("collision",))
    vis = _parse_link_primitives(urdf_path, ("visual",))
    if not coll:
        return vis
    covered = {link for link, _ in coll}
    coll.extend((link, part) for link, part in vis if link not in covered)
    return coll


# --------------------------------------------------------------------------- #
# 场景
# --------------------------------------------------------------------------- #


@dataclass
class RobotState:
    """场景中一台机器人的完整运行状态（v1.0.1 多机器人）。

    多机器人协同要求每台机器人各有**自己的**运动学链、关节角与视觉几何——
    早期实现把这三样直接摊在 :class:`Scene` 上（``_kinematics`` /
    ``_robot_joints_deg`` / ``_robot_visuals``），只能放下一台机器人。

    本类把这组状态打包，由 :class:`Scene` 用 ``robots`` 字典按名字持有；
    为了让既有代码（app.py、碰撞检查器、模板、测试）继续可用，Scene 上
    保留了原来那几个属性名作为**主机器人**的代理属性。

    Attributes:
        name: 机器人在场景中的对象名（也是 ``objects`` 里的键）
        urdf: URDF 路径
        joints_deg: 当前关节角（**度**）
        visuals: ``(link 名, Part)`` 列表，几何在各自 link 的局部系
        kinematics: :class:`~orolp.kinematics.RobotKinematics`；URDF 不可读时为 ``None``
    """

    name: str
    urdf: str
    joints_deg: List[float] = field(default_factory=list)
    visuals: List[Tuple[str, Part]] = field(default_factory=list)
    kinematics: Any = None
    #: 干涉检查用几何（``<collision>`` 基本体；无 collision 时回退 visual）
    collision_parts: List[Tuple[str, Part]] = field(default_factory=list)

    @property
    def dof(self) -> int:
        """自由度（优先取运动学链，退化为关节值个数）。"""
        if self.kinematics is not None:
            return int(self.kinematics.dof)
        return len(self.joints_deg)

    def geometry(self, source: str = GEOMETRY_COLLISION) -> List[Tuple[str, Part]]:
        """按用途取几何：``"collision"``（干涉检查）或 ``"visual"``（渲染）。

        ``collision_parts`` 为空（URDF 完全没有基本体）时两边都退回 visual，
        保证不会出现"机器人不参与检查"的静默降级。
        """
        if source == GEOMETRY_VISUAL:
            return self.visuals
        return self.collision_parts or self.visuals

    def __repr__(self) -> str:
        return f"RobotState({self.name!r}, urdf={self.urdf!r}, dof={self.dof})"


class Scene:
    """工作单元场景。

    支持**多台机器人**共享同一场景（v1.0.1）：每台机器人是一个
    :class:`RobotState`，同时也在 ``objects`` 里有一条 :class:`SceneObject`
    ——所以机器人基座位姿可以像普通对象一样被拖动/存盘，也能被别的对象
    挂在下面。

    为兼容单机器人时代的写法，下面这些成员都指向**主机器人**
    （``robots`` 里的第一个，通常名为 :data:`ROBOT_NAME`）：``robot_urdf``、
    ``has_robot``、``kinematics``、``flange_transform()``、``robot_parts()``、
    ``robot_link_parts()``、``set_robot_joints()``。

    Attributes:
        objects: 对象字典，键为对象名（含机器人）
        robots: 机器人字典，键为机器人名（插入顺序即主机器人顺序）
    """

    def __init__(self) -> None:
        self.objects: Dict[str, SceneObject] = {}
        self.robots: Dict[str, RobotState] = {}
        #: 无机器人时保留的 URDF 记录（场景文件里 URDF 路径失效时不丢信息）
        self._robot_urdf_hint: Optional[str] = None
        self._name_counter: Dict[str, int] = {}

    # --------------------------- 机器人状态代理 --------------------------- #
    #
    # 以下三个属性名在单机器人时代是普通实例属性，外部（app.py、碰撞检查器、
    # 测试）直接读写。改成指向主机器人的 property，既保住了旧调用面，又让
    # 多机器人的真实状态只存在一处（robots 字典），不会出现两份数据打架。

    def _primary(self) -> Optional[RobotState]:
        """主机器人（第一个加入的）；无机器人时 ``None``。"""
        for state in self.robots.values():
            return state
        return None

    def _primary_or_raise(self) -> RobotState:
        state = self._primary()
        if state is None:
            raise ValueError("场景中没有机器人")
        return state

    @property
    def robot_urdf(self) -> Optional[str]:
        """主机器人 URDF 路径；无机器人时返回遗留记录。"""
        state = self._primary()
        return state.urdf if state is not None else self._robot_urdf_hint

    @robot_urdf.setter
    def robot_urdf(self, value: Optional[str]) -> None:
        state = self._primary()
        if state is not None:
            state.urdf = str(value) if value else ""
        else:
            self._robot_urdf_hint = str(value) if value else None

    @property
    def _robot_joints_deg(self) -> List[float]:
        state = self._primary()
        return state.joints_deg if state is not None else []

    @_robot_joints_deg.setter
    def _robot_joints_deg(self, values: Sequence[float]) -> None:
        state = self._primary()
        if state is not None:
            state.joints_deg = [float(v) for v in values]

    @property
    def _robot_visuals(self) -> List[Tuple[str, Part]]:
        state = self._primary()
        return state.visuals if state is not None else []

    @_robot_visuals.setter
    def _robot_visuals(self, values: List[Tuple[str, Part]]) -> None:
        state = self._primary()
        if state is not None:
            state.visuals = list(values)

    @property
    def _kinematics(self):
        state = self._primary()
        return state.kinematics if state is not None else None

    @_kinematics.setter
    def _kinematics(self, value) -> None:
        state = self._primary()
        if state is not None:
            state.kinematics = value

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
        if name in self.robots:
            self.remove_robot(name)
        # 先把直接子对象重新挂到祖父上，避免"删除父对象导致子对象悬空"
        obj = self.objects[name]
        for child in [o for o in self.objects.values() if o.parent == name]:
            child.parent = obj.parent
        del self.objects[name]

    def attach(
        self,
        name: str,
        parent: Optional[str],
        keep_world: bool = False,
    ) -> None:
        """把对象挂到父对象下；``parent`` 为 :data:`ROBOT_NAME` 即挂到法兰。

        Args:
            name: 待挂载的对象名
            parent: 父对象名；``None`` 表示挂回场景根
            keep_world: 是否保持当前世界位姿——

                * ``False``（默认）：局部位姿归零，对象**吸附到父对象原点**。
                  把夹爪装到法兰上要的是这个行为（夹爪应当出现在法兰处，
                  而不是留在原地不动却"名义上"属于机器人）。
                * ``True``：反解局部位姿使世界位置不变。抓取工件时要这个
                  行为——工件被夹住的瞬间应当停在原地，之后才跟着法兰走。

        Raises:
            KeyError: 对象不存在
            ValueError: 父对象不存在、挂到自己身上、或会形成父子环
        """
        if name not in self.objects:
            raise KeyError(name)
        if parent is not None and parent not in self.objects:
            raise ValueError(f"父对象不存在: {parent}")
        if parent == name:
            raise ValueError("不能把对象挂到自己身上")

        # 沿父链向上走：若撞回自己说明会成环，world_transform 将无限递归
        cur = parent
        while cur is not None:
            if cur == name:
                raise ValueError(f"挂载会形成循环父子关系: {name} → {parent}")
            cur = self.objects[cur].parent

        obj = self.objects[name]
        if keep_world and parent is not None:
            # 先用**旧**父系算当前世界位姿，再反解新父系下的局部位姿
            world_now = self.world_transform(name)
            parent_tf = self.world_transform(parent)
            if parent in self.robots:
                parent_tf = parent_tf @ self.flange_transform(parent)
            obj.pose = Pose(np.linalg.inv(parent_tf) @ world_now)
        elif not keep_world:
            obj.pose = Pose()
        obj.parent = parent

    def detach(self, name: str) -> None:
        """解除挂载、挂回场景根，并**保持世界位姿不变**。

        这就是"放置"动作的实现：工件在释放瞬间停在当前位置，不再跟随法兰。

        Raises:
            KeyError: 对象不存在
        """
        if name not in self.objects:
            raise KeyError(name)
        obj = self.objects[name]
        if obj.parent is None:
            return
        obj.pose = Pose(self.world_transform(name))
        obj.parent = None

    def _clear_robot(self) -> None:
        """清空**所有**机器人的运动学状态（对象壳由调用方负责移除）。"""
        self.robots.clear()
        self._robot_urdf_hint = None

    def clear(self) -> None:
        """清空所有对象（保留机器人对象壳）。"""
        robot_names = list(self.robots)
        if not robot_names and self._primary() is None and ROBOT_NAME in self.objects:
            robot_names = [ROBOT_NAME]
        self.objects.clear()
        self._name_counter.clear()
        self.robots.clear()
        self._robot_urdf_hint = None
        for rname in robot_names:
            self.objects[rname] = SceneObject(
                name=rname, component_id=None, pose=Pose(), params={}
            )

    # --------------------------- 变换 --------------------------- #

    def flange_transform(self, robot: Optional[str] = None) -> np.ndarray:
        """机器人法兰在**机器人基坐标系**下的 4x4 变换（毫米）。

        由当前关节角正向求解。未加载机器人时返回单位阵——此时即便有对象挂在
        机器人下也退化为"不跟随"，不会崩。

        注意返回的是**基坐标系**而非世界系：调用方需自行左乘机器人基座位姿，
        通常直接用 :meth:`parent_transform` 更省事。

        Args:
            robot: 机器人名；``None`` 取主机器人（v1.0.1）
        """
        state = self._robot_state(robot)
        if state is None or state.kinematics is None:
            return np.eye(4)
        q_rad = np.radians(state.joints_deg)
        T_m = state.kinematics.forward_kinematics(q_rad)
        T_mm = T_m.copy()
        T_mm[:3, 3] *= 1000.0  # kinematics 内部用米，场景用毫米
        return T_mm

    def parent_transform(self, name: str) -> np.ndarray:
        """对象父系在世界坐标系下的完整变换（毫米）。

        与直接取 ``world_transform(obj.parent)`` 的区别：父对象是机器人时，
        额外右乘**当前法兰位姿**——这就是"夹爪/工件挂在法兰上随之运动"的
        全部实现。渲染、干涉检查、拖动都经这里，改一处即全局生效。

        单独抽成方法，是为了让拖动这类需要"父系逆变换"的调用方复用同一份
        逻辑，避免两处各写一套导致挂载后拖动错位。

        Args:
            name: 子对象名

        Returns:
            4x4 齐次矩阵；父对象缺失时返回单位阵（退化为不跟随）

        Raises:
            KeyError: 对象不存在
        """
        obj = self.objects[name]
        if obj.parent is None or obj.parent not in self.objects:
            # 父对象缺失（手工编辑过的场景文件）：退化为不跟随，不抛异常
            return np.eye(4)
        tf = self.world_transform(obj.parent)
        if obj.parent in self.robots:
            tf = tf @ self.flange_transform(obj.parent)
        return tf

    def world_transform(self, name: str) -> np.ndarray:
        """对象在世界坐标系下的 4x4 变换（含父级链）。

        父对象为机器人时自动追加法兰位姿，使挂载件跟随机器人运动。

        Args:
            name: 对象名

        Returns:
            4x4 齐次矩阵

        Raises:
            KeyError: 对象不存在
        """
        obj = self.objects[name]
        local = obj.pose.matrix
        if obj.parent is None or obj.parent not in self.objects:
            return local
        return self.parent_transform(name) @ local

    def object_parts(self, name: str) -> List[Part]:
        """对象在世界坐标系下的零件几何。

        Args:
            name: 对象名

        Returns:
            :class:`Part` 列表（世界系）
        """
        # 机器人作为真实场景对象：其几何随基座位姿变换
        if name in self.robots:
            base = self.world_transform(name)  # 毫米
            out = []
            for p in self.robot_parts(name):
                out.append(Part(p.prim, base @ p.transform, p.color))
            return out

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

    def _robot_state(self, name: Optional[str] = None) -> Optional[RobotState]:
        """按名字取机器人状态；``name`` 为 ``None`` 时取主机器人。"""
        if name is None:
            return self._primary()
        return self.robots.get(name)

    def is_robot(self, name: str) -> bool:
        """该对象名是否是场景中的机器人。"""
        return name in self.robots

    @property
    def robot_names(self) -> List[str]:
        """场景中所有机器人名（插入顺序，第一个是主机器人）。"""
        return list(self.robots)

    def _build_robot_state(
        self, name: str, urdf_path: str | Path, joints_deg: Optional[Sequence[float]]
    ) -> RobotState:
        """读取 URDF、建运动学链，组装一个 :class:`RobotState`。"""
        urdf = str(urdf_path)
        visuals = load_robot_visuals(urdf)
        collision = load_robot_collision_geometry(urdf)

        from ..kinematics import RobotKinematics

        kin = RobotKinematics(urdf)
        n = kin.dof
        joints = [float(v) for v in joints_deg] if joints_deg else [0.0] * n
        if len(joints) != n:
            joints = (joints + [0.0] * n)[:n]

        if not collision:
            # 不抛异常——机器人仍可显示与编程，只是无法参与干涉检查。
            # 但必须让用户知道，否则会误以为"检查通过 = 不会撞机"。
            print(
                f"[warning] {urdf}: 未解析到任何 box/cylinder/sphere 视觉体"
                f"（多为 <mesh> 几何），该机器人将不参与干涉检查"
            )
        return RobotState(
            name=name,
            urdf=urdf,
            joints_deg=joints,
            visuals=visuals,
            kinematics=kin,
            collision_parts=collision,
        )

    def _register_robot(self, state: RobotState, pose: Optional[Pose] = None) -> None:
        """把机器人状态登记进场景（同时注册其 :class:`SceneObject` 基座壳）。"""
        self.robots[state.name] = state
        existing = self.objects.get(state.name)
        if existing is None:
            self.objects[state.name] = SceneObject(
                name=state.name, component_id=None, pose=pose or Pose(), params={}
            )
        elif pose is not None:
            existing.pose = pose

    def set_robot(
        self,
        urdf_path: str | Path,
        joints_deg: Optional[Sequence[float]] = None,
        name: Optional[str] = None,
    ) -> None:
        """设置场景中的机器人（同名则替换）。

        单机器人用法（历史接口）保持不变：``scene.set_robot(urdf)`` 装载的
        就是主机器人 :data:`ROBOT_NAME`。多机器人时传 ``name``，或用
        :meth:`add_robot` 自动命名。

        Args:
            urdf_path: URDF 路径
            joints_deg: 关节角（**度**）；``None`` 时取零位
            name: 机器人名；``None`` 取 :data:`ROBOT_NAME`（v1.0.1）

        Raises:
            OSError: URDF 不存在
            ET.ParseError: URDF 非法
        """
        state = self._build_robot_state(name or ROBOT_NAME, urdf_path, joints_deg)
        self._register_robot(state)
        if self._primary() is state:
            self._robot_urdf_hint = None

    def add_robot(
        self,
        urdf_path: str | Path,
        joints_deg: Optional[Sequence[float]] = None,
        name: Optional[str] = None,
        pose: Optional[Pose] = None,
    ) -> str:
        """追加一台机器人（自动起唯一名），返回其名字（v1.0.1）。

        与 :meth:`set_robot` 的区别：本方法**不会覆盖**已有的机器人——
        需要两台以上机器人共享场景时用它。

        Args:
            urdf_path: URDF 路径
            joints_deg: 关节角（度）
            name: 期望的名字；重名时自动加序号
            pose: 机器人基座位姿（世界系）

        Returns:
            实际使用的机器人名
        """
        base = name or ROBOT_NAME
        final = self._unique_name(base)
        state = self._build_robot_state(final, urdf_path, joints_deg)
        self._register_robot(state, pose=pose)
        return final

    def remove_robot(self, name: Optional[str] = None) -> None:
        """移除机器人（默认主机器人），保留其对象壳。

        Args:
            name: 机器人名

        Raises:
            KeyError: 该名字不是场景中的机器人
        """
        target = name
        if target is None:
            state = self._primary()
            if state is None:
                raise KeyError("场景中没有机器人")
            target = state.name
        if target not in self.robots:
            raise KeyError(target)
        del self.robots[target]

    def set_robot_joints(
        self, joints_deg: Sequence[float], name: Optional[str] = None
    ) -> None:
        """更新机器人关节角。

        Args:
            joints_deg: 关节角（**度**）。数量应与自由度一致；不一致时自动
                补齐/截断并告警——URDF 里常含 fixed 关节，调用方很容易把
                ``renderer.joints``（含 fixed）当成可动关节传进来，而这类
                错误表现为"法兰不动"，极难定位。
            name: 机器人名；``None`` 取主机器人（v1.0.1）
        """
        state = self._robot_state(name)
        if state is None:
            return
        vals = [float(v) for v in joints_deg]
        n = state.dof
        if len(vals) != n:
            print(
                f"[warning] 关节值数量({len(vals)})与自由度({n})不符，"
                f"已自动补齐/截断；请检查是否误传了 fixed 关节"
            )
            vals = (vals + [0.0] * n)[:n]
        state.joints_deg = vals

    @property
    def has_robot(self) -> bool:
        """场景中是否已加载机器人。"""
        return bool(self.robots)

    @property
    def kinematics(self):
        """**主机器人**运动学实例（:class:`~orolp.kinematics.RobotKinematics`）。

        碰撞检测（连杆树遍历、自碰撞邻接关系）需要读关节树结构；
        未加载机器人时为 ``None``。多机器人请用 :meth:`kinematics_of`。
        """
        state = self._primary()
        return state.kinematics if state is not None else None

    def kinematics_of(self, name: str):
        """指定机器人的运动学实例；不存在时 ``None``（v1.0.1）。"""
        state = self.robots.get(name)
        return state.kinematics if state is not None else None

    def robot_link_parts(
        self,
        name: Optional[str] = None,
        joints_deg: Optional[Sequence[float]] = None,
        source: str = GEOMETRY_COLLISION,
    ) -> Dict[str, List[Part]]:
        """机器人当前位形下的**按连杆分组**的几何（机器人基坐标系）。

        与 :meth:`robot_parts` 的区别在于保留连杆归属——碰撞检测要定位
        "撞在哪一段"，必须知道每个零件属于哪个 link。

        Args:
            name: 机器人名；``None`` 取主机器人（v1.0.1）
            joints_deg: 关节角（度）覆盖值；给定时用这组值现算，**不改动**
                场景里存的关节角（多机器人同时刻位形检查要用）
            source: ``"collision"``（默认，干涉检查用 ``<collision>`` 包络体）
                或 ``"visual"``（渲染用 ``<visual>`` 基本体）

        Returns:
            ``{link 名: [Part, ...]}``；机器人不存在时为空字典
        """
        state = self._robot_state(name)
        if state is None or state.kinematics is None:
            return {}
        joints = state.joints_deg if joints_deg is None else list(joints_deg)
        q_rad = np.radians(np.asarray(joints, dtype=np.float64))
        link_tf = state.kinematics.all_link_transforms(q_rad)

        out: Dict[str, List[Part]] = {}
        for link_name, part in state.geometry(source):
            T = link_tf.get(link_name)
            if T is None:
                continue
            tf_mm = T.copy()
            tf_mm[:3, 3] *= 1000.0
            out.setdefault(link_name, []).append(
                Part(part.prim, tf_mm @ part.transform, part.color)
            )
        return out

    def robot_parts(
        self,
        name: Optional[str] = None,
        joints_deg: Optional[Sequence[float]] = None,
    ) -> List[Part]:
        """机器人当前位形下的连杆几何（**机器人基坐标系**，渲染用几何）。

        Args:
            name: 机器人名；``None`` 取主机器人（v1.0.1）
            joints_deg: 关节角覆盖值（见 :meth:`robot_link_parts`）

        Returns:
            :class:`Part` 列表；机器人不存在时为空列表
        """
        parts: List[Part] = []
        for link_parts in self.robot_link_parts(
            name, joints_deg, source=GEOMETRY_VISUAL
        ).values():
            parts.extend(link_parts)
        return parts

    def robot_world_link_parts(
        self,
        name: Optional[str] = None,
        joints_deg: Optional[Sequence[float]] = None,
        source: str = GEOMETRY_COLLISION,
    ) -> Dict[str, List[Part]]:
        """**世界系**下按连杆分组的机器人几何（含基座位姿，干涉检查用）。

        与 :meth:`robot_link_parts` 的区别是额外左乘机器人基座位姿——
        多机器人互避碰必须用世界系，否则两台机器人的几何都落在各自基座系
        里，叠在一起看起来永远在撞。

        Args:
            name: 机器人名；``None`` 取主机器人
            joints_deg: 关节角（度）覆盖值
            source: 几何用途，默认 ``"collision"``

        Returns:
            ``{link 名: [Part, ...]}``（世界系）；机器人不存在时为空字典
        """
        state = self._robot_state(name)
        if state is None:
            return {}
        base = self.world_transform(state.name)
        return {
            link: [Part(p.prim, base @ p.transform, p.color) for p in parts]
            for link, parts in self.robot_link_parts(
                state.name, joints_deg, source=source
            ).items()
        }

    # --------------------------- 碰撞检测 --------------------------- #

    def mounted_robot(self, name: str) -> Optional[str]:
        """对象挂在**哪台**机器人下（含多级子孙）；未挂载返回 ``None``。

        v1.0.1 多机器人后，"挂在机器人下"必须指明是哪一台：工件挂在 A 上
        时，只应豁免 A 的连杆，仍然要和 B 做干涉检查。

        Args:
            name: 对象名

        Returns:
            机器人名；未挂载在机器人下返回 ``None``
        """
        seen = set()
        cur = self.objects.get(name)
        while cur is not None and cur.name not in seen:
            if cur.parent in self.robots:
                return cur.parent
            seen.add(cur.name)
            cur = self.objects.get(cur.parent) if cur.parent else None
        return None

    def _mounted_on_robot(self, name: str) -> bool:
        """对象是否挂在机器人下（含多级子孙）。

        用于把挂载件排除在"机器人 vs 对象"的干涉检查之外——它们本来就与
        法兰固连，永远接触。

        Args:
            name: 对象名

        Returns:
            挂在任一机器人下返回 ``True``
        """
        return self.mounted_robot(name) is not None

    def check_collisions(
        self,
        include_robot: bool = True,
        margin: float = 0.0,
        include_self: bool = False,
        include_robot_robot: Optional[bool] = None,
    ) -> List[Collision]:
        """检测场景中所有对象之间（含机器人）的碰撞。

        机器人侧使用 :mod:`orolp.collision` 的连杆级检查：命中结果带
        ``link_a``（撞在哪一段）；``include_self=True`` 时同时做连杆-连杆
        自碰撞检查（结果 ``obj_a``/``obj_b`` 均为机器人名，``link_a``/
        ``link_b`` 为两个连杆名）。

        Args:
            include_robot: 是否把机器人连杆计入
            margin: 安全裕度（毫米）
            include_self: 是否检查机器人自碰撞（默认关——多数场景只关心
                机器人撞环境，自碰撞留给专门的体检入口）
            include_robot_robot: 是否检查**机器人之间**的干涉（v1.0.1）。
                ``None``（默认）表示按场景自动决定：只有一台机器人时不检查，
                多台时检查——这才是用户的预期

        Returns:
            :class:`Collision` 列表；空列表表示无干涉
        """
        # 延迟导入避免模块环（collision 依赖本模块的数据结构）
        from ..collision import CollisionChecker

        names = [
            n for n, o in self.objects.items() if o.visible and n not in self.robots
        ]
        results: List[Collision] = []

        # 机器人 vs 场景对象（连杆级，命中带 link_a；自碰撞只在
        # include_self=True 的专用块做——默认路径保持"只查机器人撞环境"）
        if include_robot and self.has_robot:
            checker = CollisionChecker(self, margin=margin)
            results.extend(checker.check_config(include_self=False))

        # 场景对象两两之间
        cache: Dict[str, List[Part]] = {n: self.object_parts(n) for n in names}
        for i, na in enumerate(names):
            for nb in names[i + 1 :]:
                hit = _parts_collide(cache[na], cache[nb], margin)
                if hit is not None:
                    ia, ib = hit
                    results.append(Collision(na, nb, ia, ib))

        # 连杆-连杆自碰撞（可选）
        if include_self and self.has_robot:
            checker = getattr(self, "_self_checker", None)
            if checker is None or checker.margin != margin:
                checker = CollisionChecker(self, margin=margin)
                self._self_checker = checker
            for robot_name in self.robot_names:
                results.extend(
                    h
                    for h in checker.check_config(include_self=True, robot=robot_name)
                    if h.obj_b == robot_name
                )

        # 机器人-机器人干涉（v1.0.1；多机器人时默认开）
        if include_robot_robot is None:
            include_robot_robot = len(self.robots) > 1
        if include_robot_robot and len(self.robots) > 1:
            checker = getattr(self, "_multi_checker", None)
            if checker is None or checker.margin != margin:
                checker = CollisionChecker(self, margin=margin)
                self._multi_checker = checker
            results.extend(checker.check_robot_pairs())

        return results

    # --------------------------- 持久化 --------------------------- #

    def to_dict(self) -> Dict[str, Any]:
        """序列化为可 JSON 化的字典。

        ``robot_urdf`` / ``robot_joints_deg`` 两个键是**主机器人**的历史
        写法（v1.0.1 前只有一台机器人），多机器人时额外写 ``robots`` 数组。
        两处信息一致，旧版本读新文件仍能拿到主机器人。
        """
        primary = self._primary()
        data: Dict[str, Any] = {
            "format_version": SCENE_FORMAT_VERSION,
            "robot_urdf": primary.urdf if primary is not None else self._robot_urdf_hint,
            "robot_joints_deg": list(primary.joints_deg) if primary is not None else [],
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
                if o.name not in self.robots
            ],
        }
        if len(self.robots) > 1:
            data["robots"] = [
                {
                    "name": state.name,
                    "urdf": state.urdf,
                    "joints_deg": list(state.joints_deg),
                    "pose": self.objects[state.name].pose.matrix.flatten().tolist(),
                }
                for state in self.robots.values()
            ]
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Scene":
        """从字典恢复场景。

        Args:
            data: :meth:`to_dict` 的输出

        Returns:
            恢复后的场景
        """
        scene = cls()

        robots_raw = data.get("robots")
        if robots_raw:
            # 多机器人场景：逐个重建（保序，第一个仍是主机器人）。
            # 每条记录都自带 urdf，无需再看 robot_urdf（那是主机器人的副本）
            for raw in robots_raw:
                urdf = raw.get("urdf")
                if not urdf:
                    continue
                pose_raw = raw.get("pose")
                pose = (
                    Pose(np.array(pose_raw, dtype=np.float64).reshape(4, 4))
                    if pose_raw
                    else None
                )
                name = raw.get("name") or ROBOT_NAME
                if Path(urdf).exists():
                    try:
                        state = scene._build_robot_state(
                            name, urdf, raw.get("joints_deg") or None
                        )
                    except Exception:
                        state = None
                    if state is not None:
                        scene._register_robot(state, pose=pose)
                        continue
                # URDF 失效：保留记录 + 对象壳，不构建运动学链
                scene._register_robot_shell(name, urdf, pose)
            return scene

        urdf = data.get("robot_urdf")
        if urdf and Path(urdf).exists():
            scene.set_robot(urdf, data.get("robot_joints_deg") or None)
        elif urdf:
            # URDF 路径失效时保留记录，但不再构建运动学链
            scene._robot_urdf_hint = urdf

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

    def _register_robot_shell(
        self, name: str, urdf: str, pose: Optional[Pose] = None
    ) -> None:
        """登记一个**没有运动学链**的机器人（URDF 丢失时的降级路径）。"""
        self.robots[name] = RobotState(name=name, urdf=str(urdf))
        if name not in self.objects:
            self.objects[name] = SceneObject(
                name=name, component_id=None, pose=pose or Pose(), params={}
            )
        elif pose is not None:
            self.objects[name].pose = pose

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
        if not np.isfinite(lo).all():
            return np.zeros(3), np.zeros(3)
        return lo, hi

    def __len__(self) -> int:
        return len(self.objects)

    def __repr__(self) -> str:
        robots = list(self.robots)
        return (
            f"Scene(objects={list(self.objects)}, "
            f"robots={robots or None})"
        )


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
