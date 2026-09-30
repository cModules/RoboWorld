"""URDF 渲染模型视图（ADR-002 拆解第一步）。

归属：App / GUI 层 ``app.views``——按 ADR-002「Renderer → views」的顺序，
这是 ``main.py`` 迁出的第一个类，是**应用层的渲染适配器**：URDF 解析与几何交给
:mod:`adapter.robot_library`，本类在其之上维护旧版字典形态的 ``links`` / ``joints``。
依赖：``app → domain(pose) / render``，不 import services。
搬迁纪律：只换位置、不改行为（ADR-002 禁止"顺手重构"）。
"""

import math
from pathlib import Path
from typing import Dict, List

import numpy as np

from app.logging import logger
from support.i18n import tr
from domain.pose import matrix_to_rpy

# 渲染模型（URDF 解析 + STL/OBJ 网格）。模块自洽：不复用 main.py 的全局 try 块，
# 免得 GUI 之外的调用方（测试/未来 CLI）拿不到它。
try:
    from adapter.robot_library.urdf_model import RobotModel

    _HAS_RENDER_MODEL = True
except Exception:  # pragma: no cover - 渲染模型缺失时回退到内置最小解析器
    _HAS_RENDER_MODEL = False
    RobotModel = None

#: 各轴"待机位姿"的目标角（度）。工业臂的经典 HOME 是「大臂后仰至水平、
#: 小臂竖起、法兰朝前」，形如举手；全零位姿会让整条臂竖直朝天，既不像真实
#: 机器人，也无法一眼看清各轴关系。超出该轴限位时钳到限位内（留 3% 余量）。
HOME_POSE_PATTERN_DEG = (0.0, -90.0, 90.0, 0.0, 90.0, 0.0, 0.0)

#: 待机位姿相对限位的收缩比例，避免正好停在限位上导致后续 moveJ 报越界
HOME_POSE_MARGIN = 0.03


class RobotRenderer:
    """URDF 渲染模型。

    底层解析交给 :class:`adapter.robot_library.urdf_model.RobotModel`——它支持
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

        logger.info(
            "urdf_loaded: links=%d joints=%d root=%s visuals=%d triangles=%d",
            len(self.links),
            len(self.joints),
            self.root_link,
            model.visual_count,
            model.triangle_count,
        )
        for name, reason in self.mesh_errors:
            logger.warning("mesh_load_failed: %s: %s", name, reason)
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
                "params": (
                    tuple(
                        p / 1000.0 if prim.kind == "cylinder" and i == 1 else p
                        for i, p in enumerate(prim.params)
                    )
                    if prim.kind != "mesh"
                    else ()
                ),
                "origin_xyz": tuple(float(v) for v in translation),
                "origin_rpy": matrix_to_rpy(visual.matrix),
                "color": tuple(float(v) for v in visual.color),
            }

        for name, joint in model.joints.items():
            self.joints[name] = {
                "name": name,
                "type": joint.type,
                "parent": joint.parent,
                "child": joint.child,
                "origin_xyz": tuple(float(v) for v in joint.matrix[:3, 3]),
                "origin_rpy": matrix_to_rpy(joint.matrix),
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
            pattern = HOME_POSE_PATTERN_DEG[index] if index < len(HOME_POSE_PATTERN_DEG) else 0.0
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
                tr("关节值数量 {n} 与自由度 {dof} 不符（关节: {names}）").format(
                    n=len(values), dof=len(names), names=names
                )
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
