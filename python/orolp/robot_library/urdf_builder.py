"""程序化 URDF 生成器 (Procedural URDF builder).

把 :class:`~orolp.robot_library.specs.RobotSpec` 中的规格参数转换为一份完整、可被
``olp.py`` 直接加载的 URDF 文件。

为什么要程序化生成
------------------
``olp.py`` 的 ``RobotRenderer`` 只解析 ``box`` / ``cylinder`` / ``sphere`` 三种基本几何体，
不解析 ``<mesh>`` 标签。因此不能直接搬运 ros-industrial 的 DAE/STL 模型。
这里改为按真实规格（臂展、负载）推导一套**自洽的连杆比例**，用基本体拼装出机器人外形：

* 零外部依赖、零许可风险（不含任何厂商网格资产）
* 单文件约 6~10 KB，全库 27 个型号总计数百 KB，加载瞬时
* 参数即事实、外形即近似 —— 选型对比与运动学仿真不受影响

坐标系约定
----------
URDF 标准：**Z 轴向上**，单位米/弧度。渲染视口同样是 Z-up，无需额外转换。

连杆只在 ``<visual><origin xyz=...>`` 中做平移，**不使用 rpy**。原因：
``RobotRenderer._draw_link_recursive`` 对 link 级 origin 的实现是 ``R·T``（先平移后旋转
的逆序），与 URDF 标准的 ``T·R`` 相反；当 rpy 全为 0 时两者等价。
因此本生成器一律用 ``box`` 的 ``size=(x, y, z)`` 来表达朝向，完全绕开该问题。

运动学构型
----------
6 轴（工业臂 / 协作臂）：:

    base ─J1(Z)─ 腰转台 ─J2(Y)─ 大臂 ─J3(Y)─ 小臂 ─J4(Z)─ 腕1 ─J5(Y)─ 腕2 ─J6(Z)─ 法兰

7 轴（如 LBR iiwa）：在肘部插入一个自转关节，形成 ``Z,Y,Z,Y,Z,Y,Z`` 交替链。

各段长度按臂展 R 等比分配，保证 ``L2 + L3 + L4 + L5 + 法兰 = R``，即水平完全伸展时
TCP 到 J1 轴线的距离等于标称臂展。
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Optional, Tuple  # noqa: F401

from xml.dom import minidom

from .specs import RobotSpec

__all__ = [
    "GeometryProfile",
    "derive_geometry",
    "build_urdf_string",
    "forward_kinematics",
    "TCP_MARKER_COLOR",
]

#: TCP 标记小球的颜色（青绿），用于在视口中直观区分工具中心点
TCP_MARKER_COLOR: Tuple[float, float, float] = (0.15, 0.85, 0.55)


# --------------------------------------------------------------------------- #
# 几何比例推导
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class GeometryProfile:
    """由规格推导出的连杆尺寸（单位：米）。"""

    base_radius: float  # 底座圆柱半径
    base_height: float  # 底座高度
    turret_radius: float  # 腰转台半径
    turret_height: float  # J1 → J2 的高度
    link_thickness: float  # 臂杆截面尺寸（方箱边长）
    upper_arm: float  # L2：J2 → J3
    forearm: float  # L3：J3 → J4
    wrist_1: float  # L4：J4 → J5
    wrist_2: float  # L5：J5 → J6
    flange_length: float  # 法兰长度
    flange_radius: float  # 法兰半径

    @property
    def shoulder_height(self) -> float:
        """J2（肩关节）相对地面的高度。"""
        return self.base_height + self.turret_height

    @property
    def kinematic_reach(self) -> float:
        """水平完全伸展时 TCP 到 J1 轴线的距离（应等于标称臂展）。"""
        return self.upper_arm + self.forearm + self.wrist_1 + self.wrist_2 + self.flange_length


def _link_thickness(payload_kg: float) -> float:
    """按额定负载估算臂杆截面边长（米）。

    用立方根缩放：负载翻 8 倍，截面才翻 1 倍，符合结构件的经验规律。
    """
    return 0.05 + 0.013 * (max(payload_kg, 0.1) ** (1.0 / 3.0))


def derive_geometry(spec: RobotSpec) -> GeometryProfile:
    """根据型号规格推导连杆几何尺寸。

    Args:
        spec: 型号规格

    Returns:
        几何尺寸（米）
    """
    reach = spec.reach_mm / 1000.0
    thickness = _link_thickness(spec.payload_kg)

    # 协作臂整体更纤细一点
    if spec.robot_type == "cobot":
        thickness *= 0.88

    # 分段长度：五项之和恰好等于标称臂展
    upper_arm = 0.46 * reach
    forearm = 0.36 * reach
    wrist_1 = 0.06 * reach
    wrist_2 = 0.06 * reach
    flange_len = 0.06 * reach

    # 7 轴臂多一个肘部自转关节，把大臂/小臂各让出一点长度给它
    if spec.dof == 7:
        upper_arm = 0.42 * reach
        forearm = 0.32 * reach
        wrist_1 = 0.06 * reach
        wrist_2 = 0.06 * reach
        flange_len = 0.06 * reach
    else:
        pass  # 6/7 轴都已通过 _elbow_length(spec) 单独处理；此处不再需要占位变量。

    base_height = 0.09 * reach
    turret_height = 0.11 * reach

    return GeometryProfile(
        base_radius=max(thickness * 1.55, 0.055),
        base_height=base_height,
        turret_radius=max(thickness * 1.20, 0.04),
        turret_height=turret_height,
        link_thickness=thickness,
        upper_arm=upper_arm,
        forearm=forearm,
        wrist_1=wrist_1,
        wrist_2=wrist_2,
        flange_length=flange_len,
        flange_radius=max(thickness * 0.42, 0.022),
        # 7 轴的额外肘部长度通过 upper_arm 之外的字段传递
    )


def _elbow_length(spec: RobotSpec) -> float:
    """7 轴机型肘部自转节的长度（米），6 轴为 0。"""
    return 0.08 * spec.reach_mm / 1000.0 if spec.dof == 7 else 0.0


# --------------------------------------------------------------------------- #
# 关节链定义
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class _Segment:
    """一个关节 + 其后连杆的描述。"""

    joint_name: str
    link_name: str
    axis: Tuple[float, float, float]  # 旋转轴（父坐标系下）
    offset: Tuple[float, float, float]  # 相对父连杆坐标系的平移
    length: float  # 连杆沿 +Z 的长度
    shape: str  # 'box' | 'cylinder'
    radius: float  # cylinder 用
    thickness: float  # box 的截面边长
    color: Tuple[float, float, float]
    limit_deg: Tuple[float, float]
    speed_deg_s: float


def _axis_sequence(dof: int) -> List[Tuple[float, float, float]]:
    """生成关节旋转轴序列。

    6 轴：``Z, Y, Y, Z, Y, Z``（腰转 / 大臂俯仰 / 小臂俯仰 / 小臂自转 / 腕俯仰 / 法兰自转）
    7 轴：``Z, Y, Z, Y, Z, Y, Z``（在肘部插入自转）
    """
    if dof == 7:
        return [(0, 0, 1), (0, 1, 0), (0, 0, 1), (0, 1, 0), (0, 0, 1), (0, 1, 0), (0, 0, 1)]
    if dof == 6:
        return [(0, 0, 1), (0, 1, 0), (0, 1, 0), (0, 0, 1), (0, 1, 0), (0, 0, 1)]
    raise ValueError(f"Unsupported DOF for procedural generation: {dof}")


def _build_segments(spec: RobotSpec, g: GeometryProfile) -> List[_Segment]:
    """构造除 base_link 以外的全部关节-连杆段。"""
    primary = spec.brand.primary
    accent = spec.brand.accent
    thickness = g.link_thickness
    axes = _axis_sequence(spec.dof)
    limits = spec.joint_limits_deg
    speeds = spec.joint_speeds_deg_s

    segments: List[_Segment] = [
        # --- J1：腰转，腰转台 ---
        _Segment(
            joint_name="joint_1",
            link_name="link_1",
            axis=axes[0],
            offset=(0.0, 0.0, g.base_height),
            length=g.turret_height,
            shape="cylinder",
            radius=g.turret_radius,
            thickness=thickness,
            color=accent,
            limit_deg=limits[0],
            speed_deg_s=speeds[0],
        ),
        # --- J2：大臂俯仰 ---
        _Segment(
            joint_name="joint_2",
            link_name="link_2",
            axis=axes[1],
            offset=(0.0, 0.0, g.turret_height),
            length=g.upper_arm,
            shape="box",
            radius=0.0,
            thickness=thickness,
            color=primary,
            limit_deg=limits[1],
            speed_deg_s=speeds[1],
        ),
    ]

    if spec.dof == 7:
        # --- J3：肘部自转（7 轴特有） ---
        segments.append(
            _Segment(
                joint_name="joint_3",
                link_name="link_3",
                axis=axes[2],
                offset=(0.0, 0.0, g.upper_arm),
                length=_elbow_length(spec),
                shape="cylinder",
                radius=thickness * 0.62,
                thickness=thickness,
                color=accent,
                limit_deg=limits[2],
                speed_deg_s=speeds[2],
            )
        )
        # --- J4：小臂俯仰 ---
        segments.append(
            _Segment(
                joint_name="joint_4",
                link_name="link_4",
                axis=axes[3],
                offset=(0.0, 0.0, _elbow_length(spec)),
                length=g.forearm,
                shape="box",
                radius=0.0,
                thickness=thickness,
                color=primary,
                limit_deg=limits[3],
                speed_deg_s=speeds[3],
            )
        )
        # --- J5：腕自转 ---
        segments.append(
            _Segment(
                joint_name="joint_5",
                link_name="link_5",
                axis=axes[4],
                offset=(0.0, 0.0, g.forearm),
                length=g.wrist_1,
                shape="cylinder",
                radius=thickness * 0.52,
                thickness=thickness,
                color=accent,
                limit_deg=limits[4],
                speed_deg_s=speeds[4],
            )
        )
        # --- J6：腕俯仰 ---
        segments.append(
            _Segment(
                joint_name="joint_6",
                link_name="link_6",
                axis=axes[5],
                offset=(0.0, 0.0, g.wrist_1),
                length=g.wrist_2,
                shape="box",
                radius=0.0,
                thickness=thickness * 0.85,
                color=primary,
                limit_deg=limits[5],
                speed_deg_s=speeds[5],
            )
        )
        # --- J7：法兰自转 ---
        segments.append(
            _Segment(
                joint_name="joint_7",
                link_name="tool0",
                axis=axes[6],
                offset=(0.0, 0.0, g.wrist_2),
                length=g.flange_length,
                shape="cylinder",
                radius=g.flange_radius,
                thickness=thickness,
                color=accent,
                limit_deg=limits[6],
                speed_deg_s=speeds[6],
            )
        )
        return segments

    # ---------------- 6 轴 ---------------- #
    segments += [
        # --- J3：小臂俯仰 ---
        _Segment(
            joint_name="joint_3",
            link_name="link_3",
            axis=axes[2],
            offset=(0.0, 0.0, g.upper_arm),
            length=g.forearm,
            shape="box",
            radius=0.0,
            thickness=thickness * 0.94,
            color=primary,
            limit_deg=limits[2],
            speed_deg_s=speeds[2],
        ),
        # --- J4：小臂自转 ---
        _Segment(
            joint_name="joint_4",
            link_name="link_4",
            axis=axes[3],
            offset=(0.0, 0.0, g.forearm),
            length=g.wrist_1,
            shape="cylinder",
            radius=thickness * 0.60,
            thickness=thickness,
            color=accent,
            limit_deg=limits[3],
            speed_deg_s=speeds[3],
        ),
        # --- J5：腕俯仰 ---
        _Segment(
            joint_name="joint_5",
            link_name="link_5",
            axis=axes[4],
            offset=(0.0, 0.0, g.wrist_1),
            length=g.wrist_2,
            shape="box",
            radius=0.0,
            thickness=thickness * 0.85,
            color=primary,
            limit_deg=limits[4],
            speed_deg_s=speeds[4],
        ),
        # --- J6：法兰自转 ---
        _Segment(
            joint_name="joint_6",
            link_name="tool0",
            axis=axes[5],
            offset=(0.0, 0.0, g.wrist_2),
            length=g.flange_length,
            shape="cylinder",
            radius=g.flange_radius,
            thickness=thickness,
            color=accent,
            limit_deg=limits[5],
            speed_deg_s=speeds[5],
        ),
    ]
    return segments


# --------------------------------------------------------------------------- #
# URDF 生成
# --------------------------------------------------------------------------- #
def _fmt(value: float) -> str:
    """格式化浮点数，避免科学计数法与冗余零。"""
    s = f"{value:.6f}"
    s = s.rstrip("0").rstrip(".")
    return s if s not in ("", "-0") else "0.0"


def _vec(values: Tuple[float, ...]) -> str:
    return " ".join(_fmt(v) for v in values)


def _rgb(color: Tuple[float, float, float]) -> str:
    return f"{_fmt(color[0])} {_fmt(color[1])} {_fmt(color[2])} 1"


def _add_geometry(parent: ET.Element, seg: _Segment, xyz: Tuple[float, float, float]) -> None:
    """为一个 link 追加 visual 与 collision 节点。"""
    for tag in ("visual", "collision"):
        node = ET.SubElement(parent, tag)
        ET.SubElement(node, "origin", {"xyz": _vec(xyz), "rpy": "0 0 0"})
        geom = ET.SubElement(node, "geometry")
        if seg.shape == "cylinder":
            ET.SubElement(
                geom,
                "cylinder",
                {"radius": _fmt(seg.radius), "length": _fmt(seg.length)},
            )
        elif seg.shape == "box":
            # box 的 size 为 (x, y, z)：沿 Z 为长度方向，其余为截面
            ET.SubElement(
                geom,
                "box",
                {
                    "size": _vec(
                        (seg.thickness, seg.thickness * 0.82, seg.length)
                    )
                },
            )
        elif seg.shape == "sphere":
            ET.SubElement(geom, "sphere", {"radius": _fmt(seg.radius)})
        if tag == "visual":
            mat = ET.SubElement(node, "material", {"name": f"{seg.link_name}_mat"})
            ET.SubElement(mat, "color", {"rgba": _rgb(seg.color)})


def build_urdf_string(spec: RobotSpec) -> str:
    """生成指定型号的 URDF 文本内容。

    Args:
        spec: 型号规格

    Returns:
        格式化的 URDF XML 字符串
    """
    g = derive_geometry(spec)
    segments = _build_segments(spec, g)

    robot = ET.Element("robot", {"name": spec.urdf_name})

    comment = ET.Comment(
        f" {spec.brand.name_en} {spec.model_name} — procedurally generated by "
        f"orolp.robot_library.urdf_builder "
        f"(payload {spec.payload_kg:g} kg, reach {spec.reach_mm:g} mm, "
        f"{spec.dof} DOF, repeatability ±{spec.repeatability_mm:g} mm). "
        f"Geometry uses primitives only (no vendor CAD assets). "
        f"Parameters verified={spec.verified}. "
    )
    robot.append(comment)

    # ---- base_link ---- #
    base = ET.SubElement(robot, "link", {"name": "base_link"})
    base_seg = _Segment(
        joint_name="",
        link_name="base_link",
        axis=(0, 0, 1),
        offset=(0, 0, 0),
        length=g.base_height,
        shape="cylinder",
        radius=g.base_radius,
        thickness=0.0,
        color=spec.brand.accent,
        limit_deg=(0, 0),
        speed_deg_s=0.0,
    )
    _add_geometry(base, base_seg, (0.0, 0.0, g.base_height / 2.0))

    # ---- 关节链 ---- #
    parent_link = "base_link"
    for seg in segments:
        joint = ET.SubElement(
            robot, "joint", {"name": seg.joint_name, "type": "revolute"}
        )
        ET.SubElement(joint, "parent", {"link": parent_link})
        ET.SubElement(joint, "child", {"link": seg.link_name})
        ET.SubElement(joint, "origin", {"xyz": _vec(seg.offset), "rpy": "0 0 0"})
        ET.SubElement(joint, "axis", {"xyz": _vec(seg.axis)})
        # effort 按负载粗估，velocity 用标称最大角速度
        effort = 40.0 + 6.0 * spec.payload_kg
        ET.SubElement(
            joint,
            "limit",
            {
                "lower": _fmt(math.radians(seg.limit_deg[0])),
                "upper": _fmt(math.radians(seg.limit_deg[1])),
                "effort": _fmt(effort),
                "velocity": _fmt(math.radians(seg.speed_deg_s)),
            },
        )

        link = ET.SubElement(robot, "link", {"name": seg.link_name})
        # 连杆几何体沿 +Z 生长，中心位于长度中点
        _add_geometry(link, seg, (0.0, 0.0, seg.length / 2.0))

        parent_link = seg.link_name

    # ---- 法兰 TCP（固定关节，标记工具中心点）---- #
    # 注意：tcp 必须带 visual 几何。``RobotRenderer.load_urdf`` 会跳过无 visual 的 link，
    # 而末端位置计算依赖该 link 存在，否则 TCP 偏移会被丢弃。
    tcp_link = ET.SubElement(robot, "link", {"name": "tcp"})
    tcp_joint = ET.SubElement(robot, "joint", {"name": "tcp_joint", "type": "fixed"})
    ET.SubElement(tcp_joint, "parent", {"link": "tool0"})
    ET.SubElement(tcp_joint, "child", {"link": "tcp"})
    ET.SubElement(
        tcp_joint, "origin", {"xyz": _vec((0.0, 0.0, g.flange_length)), "rpy": "0 0 0"}
    )
    tcp_seg = _Segment(
        joint_name="tcp_joint",
        link_name="tcp",
        axis=(0, 0, 1),
        offset=(0, 0, 0),
        length=0.0,
        shape="sphere",
        radius=g.flange_radius * 0.34,
        thickness=0.0,
        color=TCP_MARKER_COLOR,
        limit_deg=(0, 0),
        speed_deg_s=0.0,
    )
    _add_geometry(tcp_link, tcp_seg, (0.0, 0.0, 0.0))

    xml_bytes = ET.tostring(robot, encoding="utf-8")
    pretty = minidom.parseString(xml_bytes).toprettyxml(indent="  ", encoding="utf-8")
    text = pretty.decode("utf-8")
    # 去掉 minidom 在注释节点上多出的空行
    lines = [ln for ln in text.splitlines() if ln.strip()]
    return "\n".join(lines) + "\n"


# --------------------------------------------------------------------------- #
# 正运动学（用于自校验）
# --------------------------------------------------------------------------- #
def _rot_z(a: float) -> List[List[float]]:
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def _rot_y(a: float) -> List[List[float]]:
    c, s = math.cos(a), math.sin(a)
    return [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]


def _rot_x(a: float) -> List[List[float]]:
    c, s = math.cos(a), math.sin(a)
    return [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]


def _rot_axis(axis: Tuple[float, float, float], angle: float) -> List[List[float]]:
    """绕任意单位轴旋转（Rodrigues 公式）。"""
    ax, ay, az = axis
    n = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
    ax, ay, az = ax / n, ay / n, az / n
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return [
        [t * ax * ax + c, t * ax * ay - s * az, t * ax * az + s * ay],
        [t * ax * ay + s * az, t * ay * ay + c, t * ay * az - s * ax],
        [t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c],
    ]


def _mat_mul(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mat_vec(a: List[List[float]], v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (
        a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
        a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
        a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2],
    )


def forward_kinematics(
    spec: RobotSpec, q_deg: Optional[List[float]] = None
) -> Tuple[Tuple[float, float, float], List[Tuple[float, float, float]]]:
    """按 URDF 标准语义计算正运动学。

    变换链与生成的 URDF 完全一致：每个关节先平移 ``origin.xyz``，再绕 ``axis``
    旋转关节角，连杆几何沿 +Z 生长。

    Args:
        spec: 型号规格
        q_deg: 各轴角度（度）。为 None 时使用全零位姿

    Returns:
        ``(tcp_position, joint_positions)``，均为基座坐标系下的点，单位米。
        ``tcp_position`` 为法兰末端 TCP 位置；``joint_positions`` 依次为每个关节原点。
    """
    g = derive_geometry(spec)
    segments = _build_segments(spec, g)
    n = len(segments)

    if q_deg is None:
        q_deg = [0.0] * n
    if len(q_deg) != n:
        raise ValueError(f"Expected {n} joint values, got {len(q_deg)}")

    rot: List[List[float]] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    pos: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    joint_positions: List[Tuple[float, float, float]] = []

    for i, seg in enumerate(segments):
        # T = Translate(offset) · Rotate(axis, q)   （URDF 标准语义）
        offset_world = _mat_vec(rot, seg.offset)
        pos = (pos[0] + offset_world[0], pos[1] + offset_world[1], pos[2] + offset_world[2])
        rot = _mat_mul(rot, _rot_axis(seg.axis, math.radians(q_deg[i])))
        joint_positions.append(pos)

        # 连杆沿 +Z 生长到下一个关节原点（由下一段的 offset 体现）
        # 这里只记录关节位置，末端 TCP 在循环外补算

    # 末端：从最后一个关节原点沿当前 Z 方向走完法兰长度的两段
    # 最后一个 segment 的 link 长度为 flange_length，其几何中心在 length/2，
    # tcp 关节再从 tool0 原点偏移 flange_length
    last_len = segments[-1].length
    advance = _mat_vec(rot, (0.0, 0.0, last_len))
    tcp = (pos[0] + advance[0], pos[1] + advance[1], pos[2] + advance[2])

    return tcp, joint_positions
