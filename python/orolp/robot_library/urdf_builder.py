"""程序化 URDF 生成器 (Procedural URDF builder).

把 :class:`~orolp.robot_library.specs.RobotSpec` 中的规格参数转换为一份完整、可被
``olp.py`` 直接加载的 URDF 文件。

为什么要程序化生成
------------------
零外部依赖、零许可风险（不含任何厂商网格资产），单文件十几 KB，加载瞬时。
参数即事实、外形即近似 —— 选型对比与运动学仿真不受影响。

外观精细度
----------
早期版本每个连杆只生成**一个**基本体（大臂就是一根 83×68×741 mm 的长方体），
所有品牌结构完全一致，看起来像"几根彩色棍子"。这一版改为**多基本体拼装**，
每个连杆由 6~14 个基本体组成，并区分两种造型风格：

* ``industrial``（库卡 / ABB / 发那科 / 埃斯顿）—— 铸造感：
  锥形臂梁、**J2 尾部配重**、外露的电机壳与线缆管、加强筋、地脚螺栓圈。
* ``cobot``（优傲 / 节卡 / iiwa）—— 管状感：
  等径圆管 + 关节色环，无配重、无外露电机。

配色取自 :data:`~orolp.robot_library.specs.BRANDS`，并派生出金属灰、深灰两个角色色，
让电机壳、线缆、螺栓与主体涂装区分开——**层次感是"看起来精细"的关键**。

坐标系约定
----------
URDF 标准：**Z 轴向上**，单位米/弧度；``<origin xyz rpy>`` 表示 ``T · R``
（先旋转后平移），rpy 语义为 ``Rz(yaw)·Ry(pitch)·Rx(roll)``。

.. note::
   旧版本刻意**不使用 rpy**，因为当时 ``RobotRenderer`` 把 link 级 origin 实现成了
   ``R·T``（顺序相反），带 rpy 会错位。现在渲染器已按 URDF 规范修正为 ``T·R``
   （见 :mod:`orolp.render.urdf_model`），本生成器可以自由使用 rpy 表达朝向，
   例如"沿 Y 轴的电机壳"= ``rpy=(π/2, 0, 0)``。

运动学构型
----------
6 轴（工业臂 / 协作臂）：::

    base ─J1(Z)─ 腰转台 ─J2(Y)─ 大臂 ─J3(Y)─ 小臂 ─J4(Z)─ 腕1 ─J5(Y)─ 腕2 ─J6(Z)─ 法兰

7 轴（如 LBR iiwa）：在肘部插入一个自转关节，形成 ``Z,Y,Z,Y,Z,Y,Z`` 交替链。

各段长度按臂展 R 等比分配，保证 ``L2 + L3 + L4 + L5 + 法兰 = R``，即水平完全伸展时
TCP 到 J1 轴线的距离等于标称臂展。
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple  # noqa: F401

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
    )


def _elbow_length(spec: RobotSpec) -> float:
    """7 轴机型肘部自转节的长度（米），6 轴为 0。"""
    return 0.08 * spec.reach_mm / 1000.0 if spec.dof == 7 else 0.0


# --------------------------------------------------------------------------- #
# 外观：基本体零件 + 配色
# --------------------------------------------------------------------------- #
#: 把「局部 +Z 轴」的圆柱摆成轴向沿 X / Y / Z 所需的 rpy（URDF 固定轴语义）
AXIS_X: Tuple[float, float, float] = (0.0, math.pi / 2, 0.0)
AXIS_Y: Tuple[float, float, float] = (math.pi / 2, 0.0, 0.0)
AXIS_Z: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class _Part:
    """构成某个连杆的一个基本体。

    Attributes:
        shape: ``'box'`` / ``'cylinder'`` / ``'sphere'``。
        params: ``box`` → ``(sx, sy, sz)``；``cylinder`` → ``(radius, length)``；
            ``sphere`` → ``(radius,)``。
        xyz: 相对连杆坐标系原点的平移（米）。
        rpy: 相对连杆坐标系的旋转（弧度）。配合 :data:`AXIS_X` 等常量可把圆柱
            摆成任意轴向。
        role: 配色角色，取 ``primary`` / ``accent`` / ``metal`` / ``dark`` / ``marker``。
        collision: 是否参与碰撞体。碰撞检测只保留每个连杆最粗的一个包络体，
            避免 SAT 计算量随精细度线性膨胀。
    """

    shape: str
    params: Tuple[float, ...]
    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float] = AXIS_Z
    role: str = "primary"
    collision: bool = False


@dataclass(frozen=True)
class _Palette:
    """一个型号的完整配色表。

    除了品牌的主色/点缀色，再派生三种功能色：金属件、深色件、关节环。
    单一颜色的模型无论多少基本体都像一块橡皮泥，**分层配色**才是精细感来源。
    """

    primary: Tuple[float, float, float]
    accent: Tuple[float, float, float]
    metal: Tuple[float, float, float]
    dark: Tuple[float, float, float]
    joint: Tuple[float, float, float]

    def get(self, role: str) -> Tuple[float, float, float]:
        """按角色名取颜色，未知角色退回 ``primary``。"""
        return {
            "primary": self.primary,
            "accent": self.accent,
            "metal": self.metal,
            "dark": self.dark,
            "joint": self.joint,
            "marker": TCP_MARKER_COLOR,
        }.get(role, self.primary)


def _shade(color: Tuple[float, float, float], factor: float) -> Tuple[float, float, float]:
    """按系数缩放颜色亮度（>1 提亮，<1 压暗），各分量截断到 0~1。"""
    return tuple(min(1.0, max(0.0, c * factor)) for c in color)  # type: ignore[return-value]


def _make_palette(spec: RobotSpec) -> _Palette:
    """由品牌配色派生完整调色板。"""
    primary = spec.brand.primary
    accent = spec.brand.accent
    # 主色太亮时（FANUC 黄、KUKA 橙）点缀色用中性金属灰，避免整体过于花哨
    metal = (0.60, 0.63, 0.67) if _luminance(accent) < 0.5 else (0.42, 0.45, 0.50)
    dark = _shade(accent, 0.85)
    joint = _shade(metal, 0.72)
    return _Palette(
        primary=primary,
        accent=accent,
        metal=metal,
        dark=dark,
        joint=joint,
    )


def _luminance(color: Tuple[float, float, float]) -> float:
    """感知亮度（Rec. 709 权重）。"""
    return 0.2126 * color[0] + 0.7152 * color[1] + 0.0722 * color[2]


# --- 零件构造糖 --- #
def _box(
    sx: float,
    sy: float,
    sz: float,
    xyz: Tuple[float, float, float],
    rpy: Tuple[float, float, float] = AXIS_Z,
    role: str = "primary",
    collision: bool = False,
) -> _Part:
    return _Part("box", (sx, sy, sz), xyz, rpy, role, collision)


def _cyl(
    radius: float,
    length: float,
    xyz: Tuple[float, float, float],
    rpy: Tuple[float, float, float] = AXIS_Z,
    role: str = "primary",
    collision: bool = False,
) -> _Part:
    return _Part("cylinder", (radius, length), xyz, rpy, role, collision)


def _sph(
    radius: float,
    xyz: Tuple[float, float, float],
    role: str = "primary",
) -> _Part:
    return _Part("sphere", (radius,), xyz, AXIS_Z, role, False)


def _bolt_ring(
    count: int,
    ring_radius: float,
    z: float,
    stud_radius: float,
    stud_length: float,
    role: str = "metal",
) -> List[_Part]:
    """沿半径为 ``ring_radius`` 的圆周均匀布置一圈螺栓柱（沿 Z 轴）。

    地脚螺栓圈与法兰螺纹孔是机器人最容易辨认的细节之一——成本极低（每圈 6~8 个
    小圆柱），却能让"圆柱+方块"的粗糙感立刻消失。
    """
    parts: List[_Part] = []
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        parts.append(
            _cyl(
                stud_radius,
                stud_length,
                (ring_radius * math.cos(angle), ring_radius * math.sin(angle), z),
                AXIS_Z,
                role,
            )
        )
    return parts


# --------------------------------------------------------------------------- #
# 连杆外观配方
# --------------------------------------------------------------------------- #
def _base_parts(g: GeometryProfile, pal: _Palette, cobot: bool) -> List[_Part]:
    """底座（base_link）：地脚板 + 螺栓圈 + 锥台 + 立柱 + 顶部凸缘。"""
    h = g.base_height
    rb = g.base_radius
    parts: List[_Part] = []

    if cobot:
        # 协作臂底座更简洁：小底盘 + 立柱，无外露螺栓
        parts += [
            _cyl(rb * 1.18, h * 0.20, (0.0, 0.0, h * 0.10), AXIS_Z, "dark", True),
            _cyl(rb * 1.02, h * 0.16, (0.0, 0.0, h * 0.28), AXIS_Z, "primary"),
            _cyl(rb * 0.92, h * 0.56, (0.0, 0.0, h * 0.64), AXIS_Z, "primary", True),
            _cyl(rb * 0.97, h * 0.08, (0.0, 0.0, h * 0.94), AXIS_Z, "accent"),
        ]
        parts += _bolt_ring(4, rb * 0.98, h * 0.14, rb * 0.07, h * 0.22)
        return parts

    parts += [
        # 地脚板（最宽，深灰，像铸铁）
        _cyl(rb * 1.34, h * 0.16, (0.0, 0.0, h * 0.08), AXIS_Z, "dark"),
        # 过渡锥台
        _cyl(rb * 1.12, h * 0.14, (0.0, 0.0, h * 0.23), AXIS_Z, "primary"),
        # 主立柱
        _cyl(rb, h * 0.62, (0.0, 0.0, h * 0.61), AXIS_Z, "primary", True),
        # 顶部凸缘（与腰转台回转支承相连）
        _cyl(rb * 1.06, h * 0.10, (0.0, 0.0, h * 0.95), AXIS_Z, "accent"),
        # 铭牌
        _box(rb * 0.52, rb * 0.12, h * 0.26, (rb * 0.90, 0.0, h * 0.62), AXIS_Z, "accent"),
        # 线缆 / 气路接口（沿 X 伸出的短管）
        _cyl(rb * 0.17, rb * 0.30, (rb * 0.92, 0.0, h * 0.30), AXIS_X, "dark"),
    ]
    # 地脚螺栓圈：6 颗
    parts += _bolt_ring(6, rb * 1.14, h * 0.13, rb * 0.075, h * 0.22)
    return parts


def _turret_parts(g: GeometryProfile, pal: _Palette, cobot: bool) -> List[_Part]:
    """腰转台（link_1）：回转支承 + 肩部铸造 + 叉臂 + J2 电机壳。

    J1 轴线为局部 Z，J2 轴线在顶部 ``z = turret_height`` 处、方向为 Y。
    """
    h = g.turret_height
    t = g.link_thickness
    parts: List[_Part] = []

    if cobot:
        parts += [
            _cyl(g.turret_radius * 1.06, h * 0.26, (0.0, 0.0, h * 0.13), AXIS_Z, "dark"),
            _cyl(g.turret_radius * 0.96, h * 0.62, (0.0, 0.0, h * 0.60), AXIS_Z, "primary", True),
            _cyl(g.turret_radius * 1.02, h * 0.10, (0.0, 0.0, h * 0.95), AXIS_Z, "accent"),
            # 肩部两侧的关节色环（沿 Y）
            _cyl(t * 0.56, t * 1.18, (0.0, 0.0, h * 0.97), AXIS_Y, "joint"),
            # 品牌色竖条
            _box(t * 0.10, t * 0.86, h * 0.50, (g.turret_radius * 0.90, 0.0, h * 0.48),
                 AXIS_Z, "accent"),
        ]
        return parts

    parts += [
        # 回转支承（深灰，视觉上把底座与转台分开）
        _cyl(g.turret_radius, t * 0.32, (0.0, 0.0, t * 0.16), AXIS_Z, "dark"),
        # 肩部铸造下段（宽）
        _box(t * 1.52, t * 1.30, h * 0.46, (0.0, 0.0, t * 0.32 + h * 0.23), AXIS_Z, "primary"),
        # 肩部铸造上段（窄，形成锥度）
        _box(t * 1.34, t * 1.12, h * 0.34, (0.0, 0.0, t * 0.32 + h * 0.46 + h * 0.17),
             AXIS_Z, "primary"),
        # 叉臂两侧板：夹住大臂根部的轴套
        _box(t * 1.40, t * 0.34, h * 0.40, (0.0, t * 0.66, h * 0.80), AXIS_Z, "primary"),
        _box(t * 1.40, t * 0.34, h * 0.40, (0.0, -t * 0.66, h * 0.80), AXIS_Z, "primary"),
        # J2 电机壳：与 J2 轴同轴，从 -Y 侧伸出（工业臂的标志性鼓包）
        _cyl(t * 0.44, t * 1.05, (0.0, -(t * 0.66 + t * 0.52), h * 0.86), AXIS_Y, "metal"),
        _cyl(t * 0.30, t * 0.16, (0.0, -(t * 0.66 + t * 1.10), h * 0.86), AXIS_Y, "dark"),
        # 线缆管（沿 Z 走在 -X 侧）
        _cyl(t * 0.11, h * 0.50, (-t * 0.70, -t * 0.28, h * 0.42), AXIS_Z, "dark"),
        # 品牌色侧条
        _box(t * 0.10, t * 1.16, h * 0.44, (t * 0.72, 0.0, h * 0.42), AXIS_Z, "accent"),
    ]
    return parts


def _upper_arm_parts(g: GeometryProfile, pal: _Palette, cobot: bool) -> List[_Part]:
    """大臂（link_2）：锥形主梁 + J2 尾部配重 + J3 电机壳。

    这是外观改进最大的一处。旧版只是一根等截面长方体；现在有锥度、
    有配重、有电机壳，轮廓一眼可辨。
    """
    length = g.upper_arm
    t = g.link_thickness
    parts: List[_Part] = []

    if cobot:
        parts += [
            # 协作臂：等径圆管沿 +Z
            _cyl(t * 0.50, length, (0.0, 0.0, length * 0.5), AXIS_Z, "primary", True),
            # 两端关节环（沿 Y）
            _cyl(t * 0.56, t * 0.34, (0.0, 0.0, 0.0), AXIS_Y, "joint"),
            _cyl(t * 0.52, t * 0.32, (0.0, 0.0, length), AXIS_Y, "joint"),
            # 轴端盖
            _cyl(t * 0.30, t * 0.16, (0.0, t * 0.30, 0.0), AXIS_Y, "dark"),
            _cyl(t * 0.30, t * 0.16, (0.0, -t * 0.30, 0.0), AXIS_Y, "dark"),
            _cyl(t * 0.28, t * 0.16, (0.0, t * 0.28, length), AXIS_Y, "dark"),
            _cyl(t * 0.28, t * 0.16, (0.0, -t * 0.28, length), AXIS_Y, "dark"),
            # 品牌色腰线
            _cyl(t * 0.52, length * 0.10, (0.0, 0.0, length * 0.72), AXIS_Z, "accent"),
        ]
        return parts

    parts += [
        # 主梁下段（粗）
        _box(t * 0.98, t * 0.84, length * 0.54, (0.0, 0.0, length * 0.27), AXIS_Z, "primary"),
        # 主梁上段（细，两段叠出锥度）
        _box(t * 0.80, t * 0.68, length * 0.50, (0.0, 0.0, length * 0.75), AXIS_Z, "primary"),
        # 侧面加强筋（比主梁略宽，形成一道凸脊）
        _box(t * 1.16, t * 0.30, length * 0.44, (0.0, 0.0, length * 0.38), AXIS_Z, "accent"),
        # J2 端轴套（沿 Y，贯穿整个肩宽）
        _cyl(t * 0.58, t * 1.28, (0.0, 0.0, 0.0), AXIS_Y, "metal"),
        _cyl(t * 0.36, t * 0.10, (0.0, t * 0.70, 0.0), AXIS_Y, "dark"),
        _cyl(t * 0.36, t * 0.10, (0.0, -t * 0.70, 0.0), AXIS_Y, "dark"),
        # 尾部配重：J2 轴线后方（-Z 方向）的铸铁块，
        # 用于平衡大臂+小臂的力矩——工业臂最醒目的外形特征
        _box(t * 1.08, t * 0.90, length * 0.15, (0.0, 0.0, -length * 0.072), AXIS_Z, "dark"),
        _cyl(t * 0.58, length * 0.04, (0.0, 0.0, -length * 0.152), AXIS_Z, "dark"),
        # J3 电机壳（肘部，沿 Y）
        _cyl(t * 0.50, t * 1.10, (0.0, 0.0, length * 0.93), AXIS_Y, "metal"),
        # 肘部轴承座
        _cyl(t * 0.56, t * 0.86, (0.0, 0.0, length), AXIS_Y, "accent"),
        # 线缆管（沿 Z，在 -X 侧）
        _cyl(t * 0.09, length * 0.62, (-t * 0.46, 0.0, length * 0.42), AXIS_Z, "dark"),
    ]
    return parts


def _forearm_parts(g: GeometryProfile, pal: _Palette, cobot: bool) -> List[_Part]:
    """小臂（link_3）：锥形梁 + J4 电机壳 / 腕部管。"""
    length = g.forearm
    t = g.link_thickness
    parts: List[_Part] = []

    if cobot:
        parts += [
            _cyl(t * 0.44, length, (0.0, 0.0, length * 0.5), AXIS_Z, "primary", True),
            _cyl(t * 0.52, t * 0.32, (0.0, 0.0, 0.0), AXIS_Y, "joint"),
            _cyl(t * 0.46, t * 0.30, (0.0, 0.0, length), AXIS_Y, "joint"),
            _cyl(t * 0.52, length * 0.08, (0.0, 0.0, length * 0.62), AXIS_Z, "accent"),
            # 腕部过渡环
            _cyl(t * 0.40, t * 0.22, (0.0, 0.0, length * 0.96), AXIS_Z, "metal"),
        ]
        return parts

    parts += [
        # 主梁：两段落差
        _box(t * 0.92, t * 0.78, length * 0.52, (0.0, 0.0, length * 0.26), AXIS_Z, "primary"),
        _box(t * 0.74, t * 0.62, length * 0.52, (0.0, 0.0, length * 0.74), AXIS_Z, "primary"),
        # J3 端轴套（肘关节，沿 Y）
        _cyl(t * 0.54, t * 1.06, (0.0, 0.0, 0.0), AXIS_Y, "metal"),
        _cyl(t * 0.34, t * 0.10, (0.0, t * 0.58, 0.0), AXIS_Y, "dark"),
        _cyl(t * 0.34, t * 0.10, (0.0, -t * 0.58, 0.0), AXIS_Y, "dark"),
        # 加强筋
        _box(t * 1.08, t * 0.28, length * 0.40, (0.0, 0.0, length * 0.36), AXIS_Z, "accent"),
        # J4 电机壳：位于小臂末端，轴线为 Z（小臂自转）
        _cyl(t * 0.46, t * 0.78, (0.0, 0.0, length * 0.90), AXIS_Z, "metal"),
        # 腕部过渡法兰
        _cyl(t * 0.50, t * 0.20, (0.0, 0.0, length * 0.99), AXIS_Z, "accent"),
        # 线缆管
        _cyl(t * 0.08, length * 0.56, (-t * 0.42, 0.0, length * 0.44), AXIS_Z, "dark"),
    ]
    return parts


def _elbow_parts(g: GeometryProfile, pal: _Palette, cobot: bool) -> List[_Part]:
    """7 轴机型的肘部自转节（link_3）：短圆柱 + 两端色环。"""
    length = _elbow_length_from_profile(g)
    t = g.link_thickness
    return [
        _cyl(t * 0.46, length * 0.92, (0.0, 0.0, length * 0.5), AXIS_Z, "primary", True),
        _cyl(t * 0.52, t * 0.26, (0.0, 0.0, length * 0.10), AXIS_Z, "joint"),
        _cyl(t * 0.52, t * 0.26, (0.0, 0.0, length * 0.90), AXIS_Z, "joint"),
        _cyl(t * 0.54, length * 0.10, (0.0, 0.0, length * 0.5), AXIS_Z, "accent"),
    ]


def _elbow_length_from_profile(g: GeometryProfile) -> float:
    """从几何剖面反推 7 轴肘部节长度（与 :func:`_elbow_length` 保持同比例）。"""
    return 0.08 / 0.42 * g.upper_arm


def _wrist_roll_parts(g: GeometryProfile, pal: _Palette, cobot: bool) -> List[_Part]:
    """腕部自转节（link_4）：沿 Z 的短筒 + 色环。"""
    length = g.wrist_1
    t = g.link_thickness
    parts: List[_Part] = [
        _cyl(t * 0.42, length * 0.94, (0.0, 0.0, length * 0.5), AXIS_Z, "primary", True),
        _cyl(t * 0.46, length * 0.16, (0.0, 0.0, length * 0.10), AXIS_Z, "joint"),
        _cyl(t * 0.46, length * 0.16, (0.0, 0.0, length * 0.90), AXIS_Z, "joint"),
    ]
    if not cobot:
        parts.append(_cyl(t * 0.48, length * 0.10, (0.0, 0.0, length * 0.5), AXIS_Z, "accent"))
    return parts


def _wrist_pitch_parts(g: GeometryProfile, pal: _Palette, cobot: bool) -> List[_Part]:
    """腕部俯仰节（link_5）：叉臂 + 摆动体 + 端盖。"""
    length = g.wrist_2
    t = g.link_thickness
    parts: List[_Part] = [
        # 腕叉臂（沿 Y 的轴套）
        _cyl(t * 0.40, t * 0.92, (0.0, 0.0, 0.0), AXIS_Y, "metal", True),
        # 摆动体
        _cyl(t * 0.38, length * 0.86, (0.0, 0.0, length * 0.45), AXIS_Z, "primary"),
        # 端部法兰座
        _cyl(t * 0.42, length * 0.16, (0.0, 0.0, length * 0.94), AXIS_Z, "accent"),
        # 两侧端盖
        _cyl(t * 0.24, t * 0.10, (0.0, t * 0.50, 0.0), AXIS_Y, "dark"),
        _cyl(t * 0.24, t * 0.10, (0.0, -t * 0.50, 0.0), AXIS_Y, "dark"),
    ]
    return parts


def _flange_parts(g: GeometryProfile, pal: _Palette, cobot: bool) -> List[_Part]:
    """法兰（tool0）：盘体 + 定心凸台 + 螺纹孔圈。"""
    length = g.flange_length
    rf = g.flange_radius
    parts: List[_Part] = [
        _cyl(rf, length * 0.52, (0.0, 0.0, length * 0.26), AXIS_Z, "metal", True),
        _cyl(rf * 0.62, length * 0.30, (0.0, 0.0, length * 0.67), AXIS_Z, "metal"),
        _cyl(rf * 1.06, length * 0.14, (0.0, 0.0, length * 0.07), AXIS_Z, "accent"),
    ]
    # 法兰螺纹孔圈（6 个沉孔，用深色小圆柱表示）
    parts += _bolt_ring(6, rf * 0.78, length * 0.50, rf * 0.09, length * 0.22, "dark")
    return parts


def _tcp_parts(g: GeometryProfile, pal: _Palette) -> List[_Part]:
    """TCP 标记小球。"""
    return [_sph(g.flange_radius * 0.34, (0.0, 0.0, 0.0), "marker")]


# --------------------------------------------------------------------------- #
# 关节链定义
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class _Segment:
    """一个关节 + 其后连杆的描述。

    Attributes:
        joint_name: 关节名；base_link / tcp 这两节为空串或占位名。
        link_name: 连杆名。
        axis: 旋转轴（父坐标系下）。
        offset: 相对父连杆坐标系的平移。
        length: 连杆沿 +Z 的长度。
        shape: **保留字段**，表示碰撞包络体的形状（``box`` / ``cylinder`` / ``sphere``）。
        radius: 碰撞包络体半径（cylinder / sphere 用）。
        thickness: 碰撞包络体截面边长（box 用）。
        color: **保留字段**，回退用的连杆主色。
        limit_deg: 关节限位（度）。
        speed_deg_s: 关节最大角速度（度/秒）。
        parts: 构成该连杆外观的全部基本体。
    """

    joint_name: str
    link_name: str
    axis: Tuple[float, float, float]  # 旋转轴（父坐标系下）
    offset: Tuple[float, float, float]  # 相对父连杆坐标系的平移
    length: float  # 连杆沿 +Z 的长度
    shape: str  # 'box' | 'cylinder' | 'sphere'（碰撞包络体）
    radius: float  # cylinder 用
    thickness: float  # box 的截面边长
    color: Tuple[float, float, float]
    limit_deg: Tuple[float, float]
    speed_deg_s: float
    parts: Tuple[_Part, ...] = ()
    palette: Optional[_Palette] = None


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
    pal = _make_palette(spec)
    cobot = spec.robot_type == "cobot"
    thickness = g.link_thickness
    axes = _axis_sequence(spec.dof)
    limits = spec.joint_limits_deg
    speeds = spec.joint_speeds_deg_s

    # 碰撞包络体参数由 :func:`_add_envelope` 从 ``shape/radius/thickness`` 读取，
    # 外观则由 ``parts`` 决定；两者解耦，精细度不影响碰撞检测规模。

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
            color=pal.accent,
            limit_deg=limits[0],
            speed_deg_s=speeds[0],
            parts=tuple(_turret_parts(g, pal, cobot)),
            palette=pal,
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
            color=pal.primary,
            limit_deg=limits[1],
            speed_deg_s=speeds[1],
            parts=tuple(_upper_arm_parts(g, pal, cobot)),
            palette=pal,
        ),
    ]

    if spec.dof == 7:
        elbow_len = _elbow_length(spec)
        # --- J3：肘部自转（7 轴特有） ---
        segments.append(
            _Segment(
                joint_name="joint_3",
                link_name="link_3",
                axis=axes[2],
                offset=(0.0, 0.0, g.upper_arm),
                length=elbow_len,
                shape="cylinder",
                radius=thickness * 0.50,
                thickness=thickness,
                color=pal.accent,
                limit_deg=limits[2],
                speed_deg_s=speeds[2],
                parts=tuple(_elbow_parts(g, pal, cobot)),
                palette=pal,
            )
        )
        # --- J4：小臂俯仰 ---
        segments.append(
            _Segment(
                joint_name="joint_4",
                link_name="link_4",
                axis=axes[3],
                offset=(0.0, 0.0, elbow_len),
                length=g.forearm,
                shape="box",
                radius=0.0,
                thickness=thickness,
                color=pal.primary,
                limit_deg=limits[3],
                speed_deg_s=speeds[3],
                parts=tuple(_forearm_parts(g, pal, cobot)),
                palette=pal,
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
                radius=thickness * 0.44,
                thickness=thickness,
                color=pal.accent,
                limit_deg=limits[4],
                speed_deg_s=speeds[4],
                parts=tuple(_wrist_roll_parts(g, pal, cobot)),
                palette=pal,
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
                thickness=thickness * 0.78,
                color=pal.primary,
                limit_deg=limits[5],
                speed_deg_s=speeds[5],
                parts=tuple(_wrist_pitch_parts(g, pal, cobot)),
                palette=pal,
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
                color=pal.accent,
                limit_deg=limits[6],
                speed_deg_s=speeds[6],
                parts=tuple(_flange_parts(g, pal, cobot)),
                palette=pal,
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
            color=pal.primary,
            limit_deg=limits[2],
            speed_deg_s=speeds[2],
            parts=tuple(_forearm_parts(g, pal, cobot)),
            palette=pal,
        ),
        # --- J4：小臂自转 ---
        _Segment(
            joint_name="joint_4",
            link_name="link_4",
            axis=axes[3],
            offset=(0.0, 0.0, g.forearm),
            length=g.wrist_1,
            shape="cylinder",
            radius=thickness * 0.44,
            thickness=thickness,
            color=pal.accent,
            limit_deg=limits[3],
            speed_deg_s=speeds[3],
            parts=tuple(_wrist_roll_parts(g, pal, cobot)),
            palette=pal,
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
            color=pal.primary,
            limit_deg=limits[4],
            speed_deg_s=speeds[4],
            parts=tuple(_wrist_pitch_parts(g, pal, cobot)),
            palette=pal,
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
            color=pal.accent,
            limit_deg=limits[5],
            speed_deg_s=speeds[5],
            parts=tuple(_flange_parts(g, pal, cobot)),
            palette=pal,
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


def _write_geometry(parent: ET.Element, part: _Part) -> None:
    """向 ``parent``（一个 ``<geometry>`` 节点）写入一个基本体定义。"""
    if part.shape == "cylinder":
        ET.SubElement(
            parent,
            "cylinder",
            {"radius": _fmt(part.params[0]), "length": _fmt(part.params[1])},
        )
    elif part.shape == "box":
        ET.SubElement(parent, "box", {"size": _vec(part.params)})
    elif part.shape == "sphere":
        ET.SubElement(parent, "sphere", {"radius": _fmt(part.params[0])})
    else:  # pragma: no cover - 防御：配方里不应出现未知形状
        raise ValueError(f"未知几何体类型: {part.shape}")


def _add_envelope(link_el: ET.Element, seg: _Segment, length: float) -> None:
    """写入碰撞包络体。

    碰撞检测对每个连杆只用一个最粗的体：精细外观带来的几十个基本体若全部参与
    SAT，检测耗时将随精细度线性增长，而工程上包络体本就该是简化模型。
    """
    node = ET.SubElement(link_el, "collision")
    ET.SubElement(node, "origin", {"xyz": _vec((0.0, 0.0, length / 2.0)), "rpy": "0 0 0"})
    geom = ET.SubElement(node, "geometry")
    if seg.shape == "cylinder":
        ET.SubElement(
            geom,
            "cylinder",
            {"radius": _fmt(seg.radius), "length": _fmt(seg.length)},
        )
    elif seg.shape == "sphere":
        ET.SubElement(geom, "sphere", {"radius": _fmt(seg.radius)})
    else:
        # box 的 size 为 (x, y, z)：沿 Z 为长度方向，其余为截面
        ET.SubElement(
            geom,
            "box",
            {"size": _vec((seg.thickness, seg.thickness * 0.82, seg.length))},
        )


def _add_parts(link_el: ET.Element, seg: _Segment, pal: _Palette) -> None:
    """写入全部 visual 基本体。

    rpy 现在可以放心使用——``olp.py`` 的渲染器已按 URDF 规范实现 ``T·R``。
    """
    for index, part in enumerate(seg.parts):
        node = ET.SubElement(link_el, "visual")
        ET.SubElement(
            node,
            "origin",
            {"xyz": _vec(part.xyz), "rpy": _vec(part.rpy)},
        )
        geom = ET.SubElement(node, "geometry")
        _write_geometry(geom, part)
        mat = ET.SubElement(node, "material", {"name": f"{seg.link_name}_mat_{index}"})
        ET.SubElement(mat, "color", {"rgba": _rgb(pal.get(part.role))})


def build_urdf_string(spec: RobotSpec) -> str:
    """生成指定型号的 URDF 文本内容。

    Args:
        spec: 型号规格

    Returns:
        格式化的 URDF XML 字符串
    """
    g = derive_geometry(spec)
    pal = _make_palette(spec)
    cobot = spec.robot_type == "cobot"
    segments = _build_segments(spec, g)
    n_parts = len(_base_parts(g, pal, cobot)) + sum(len(s.parts) for s in segments) + 1

    robot = ET.Element("robot", {"name": spec.urdf_name})

    comment = ET.Comment(
        f" {spec.brand.name_en} {spec.model_name} — procedurally generated by "
        f"orolp.robot_library.urdf_builder "
        f"(payload {spec.payload_kg:g} kg, reach {spec.reach_mm:g} mm, "
        f"{spec.dof} DOF, repeatability ±{spec.repeatability_mm:g} mm). "
        f"Appearance: {n_parts} primitives in '{spec.robot_type}' style "
        f"(primary/accent/metal/dark 4-role palette). "
        f"No vendor CAD assets. "
        f"Parameters verified={spec.verified}. "
    )
    robot.append(comment)

    # ---- base_link ---- #
    base = ET.SubElement(robot, "link", {"name": "base_link"})
    base_parts = _base_parts(g, pal, cobot)
    for index, part in enumerate(base_parts):
        node = ET.SubElement(base, "visual")
        ET.SubElement(node, "origin", {"xyz": _vec(part.xyz), "rpy": _vec(part.rpy)})
        geom = ET.SubElement(node, "geometry")
        _write_geometry(geom, part)
        mat = ET.SubElement(node, "material", {"name": f"base_link_mat_{index}"})
        ET.SubElement(mat, "color", {"rgba": _rgb(pal.get(part.role))})
    # 底座碰撞包络：一根涵盖全高的圆柱
    _add_envelope(
        base,
        _Segment("", "base_link", (0, 0, 1), (0, 0, 0), g.base_height,
                 "cylinder", max(g.base_radius, g.link_thickness), 0.0,
                 pal.accent, (0, 0), 0.0),
        g.base_height,
    )

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
        _add_parts(link, seg, pal)
        _add_envelope(link, seg, seg.length)

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
    for part in _tcp_parts(g, pal):
        node = ET.SubElement(tcp_link, "visual")
        ET.SubElement(node, "origin", {"xyz": _vec(part.xyz), "rpy": _vec(part.rpy)})
        geom = ET.SubElement(node, "geometry")
        _write_geometry(geom, part)
        mat = ET.SubElement(node, "material", {"name": "tcp_mat"})
        ET.SubElement(mat, "color", {"rgba": _rgb(pal.get(part.role))})

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
