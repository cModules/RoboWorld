#!/usr/bin/env python3
"""高保真程序化建模配方。

与 :mod:`urdf_builder` 中基本体配方的区别
-----------------------------------------

基本体配方（``_box`` / ``_cyl`` / ``_sph``）堆得再多也脱不掉"积木风"，
因为 URDF 基本体表达不了工业机器人的三件事：

1. **变截面** —— 大臂从肩到肘持续收窄，等径圆柱做不出来
2. **铸造圆角** —— 真机没有一条锐边，全是 R10~R30 的大圆角
3. **连续锥度** —— 底座、关节壳都是带台阶和拔模斜度的回转体

本模块用 :mod:`orolp.render.primitives` 的曲面原语重新写了一遍各连杆的
配方：主梁用放样体（loft）做变截面，铸造件用圆角箱体（rbox），回转轴系
用旋转体（revolve），线缆用扫掠管（tube）。

设计上刻意保持与基本体配方**相同的函数签名**，这样 :func:`_build_segments`
只需切换一个字典即可在两套外观之间来回切换，便于 A/B 对比与回退。
"""

from __future__ import annotations

import math
from typing import List, Sequence, Tuple

from .urdf_builder import AXIS_Y, AXIS_Z, GeometryProfile, _MeshSpec, _Palette, _Part

__all__ = ["HIFI_BUILDERS"]


# --------------------------------------------------------------------------- #
# 便捷构造
# --------------------------------------------------------------------------- #
def _loft_part(
    sections: Sequence[Tuple[float, float, float, float]],
    xyz: Tuple[float, float, float],
    rpy: Tuple[float, float, float] = AXIS_Z,
    role: str = "primary",
    segments: int = 36,
    squareness: float = 4.0,
) -> _Part:
    """构造一个放样零件。

    Args:
        sections: ``(z, hx, hy, squareness)`` 序列，按 z 升序。
        xyz: 零件在连杆坐标系中的位置。
        rpy: 朝向。
        role: 配色角色。
        segments: 周向分段数。
        squareness: 未显式给出时使用的默认截面方度。

    Returns:
        带网格规格的零件。
    """
    from ..render.primitives import Section

    secs = tuple(
        Section(z=s[0], hx=s[1], hy=s[2], squareness=(s[3] if len(s) > 3 else squareness))
        for s in sections
    )
    hx_max = max(s.hx for s in secs)
    hy_max = max(s.hy for s in secs)
    z_lo = min(s.z for s in secs)
    z_hi = max(s.z for s in secs)
    return _Part(
        shape="box",
        params=(2.0 * hx_max, 2.0 * hy_max, z_hi - z_lo),
        xyz=xyz,
        rpy=rpy,
        role=role,
        mesh=_MeshSpec("loft", (secs, segments)),
    )


def _rbox_part(
    sx: float,
    sy: float,
    sz: float,
    radius: float,
    xyz: Tuple[float, float, float],
    rpy: Tuple[float, float, float] = AXIS_Z,
    role: str = "primary",
    quality: int = 7,
) -> _Part:
    """构造一个圆角箱体零件。"""
    return _Part(
        shape="box",
        params=(sx, sy, sz),
        xyz=xyz,
        rpy=rpy,
        role=role,
        mesh=_MeshSpec("rbox", (sx, sy, sz, radius, quality)),
    )


def _revolve_part(
    profile: Sequence[Tuple[float, float]],
    xyz: Tuple[float, float, float],
    rpy: Tuple[float, float, float] = AXIS_Z,
    role: str = "primary",
    segments: int = 32,
) -> _Part:
    """构造一个回转体零件（沿局部 +Z 轴）。"""
    prof = tuple((float(a), float(b)) for a, b in profile)
    r_max = max(p[0] for p in prof)
    z_lo = min(p[1] for p in prof)
    z_hi = max(p[1] for p in prof)
    return _Part(
        shape="cylinder",
        params=(r_max, z_hi - z_lo),
        xyz=xyz,
        rpy=rpy,
        role=role,
        mesh=_MeshSpec("revolve", (prof, segments)),
    )


def _tube_part(
    points: Sequence[Sequence[float]],
    radius: float,
    role: str = "dark",
    segments: int = 10,
) -> _Part:
    """构造一段管路/线缆。路径点用**世界（连杆）坐标**，不做额外偏移。"""
    pts = tuple((float(p[0]), float(p[1]), float(p[2])) for p in points)
    arr = pts
    lo = [min(p[i] for p in arr) - radius for i in range(3)]
    hi = [max(p[i] for p in arr) + radius for i in range(3)]
    return _Part(
        shape="box",
        params=(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]),
        xyz=(0.0, 0.0, 0.0),
        rpy=AXIS_Z,
        role=role,
        mesh=_MeshSpec("tube", (pts, radius, segments)),
    )


def _bolt_ring(
    count: int,
    ring_radius: float,
    z: float,
    stud_radius: float,
    stud_height: float,
    role: str = "metal",
) -> List[_Part]:
    """沿一圈布置螺栓（小圆台，比胶囊省三角形）。"""
    parts: List[_Part] = []
    for i in range(count):
        a = 2.0 * math.pi * i / count
        parts.append(
            _revolve_part(
                [
                    (0.0, 0.0),
                    (stud_radius, 0.0),
                    (stud_radius, stud_height * 0.7),
                    (stud_radius * 0.72, stud_height),
                    (0.0, stud_height),
                ],
                (ring_radius * math.cos(a), ring_radius * math.sin(a), z),
                AXIS_Z,
                role,
                segments=8,
            )
        )
    return parts


# --------------------------------------------------------------------------- #
# 各连杆配方
# --------------------------------------------------------------------------- #
def base(g: GeometryProfile, pal: _Palette, cobot: bool) -> List[_Part]:
    """底座：落地法兰 + 螺栓圈 + 带拔模斜度的锥形立柱。

    真机底座是一个铸铁法兰盘，靠 4~6 颗地脚螺栓固定，立柱带明显的
    拔模斜度（上细下粗）——这些用基本体拼会留下难看的台阶。
    """
    r = g.base_radius
    h = g.base_height
    parts: List[_Part] = []

    if cobot:
        parts.append(
            _revolve_part(
                [
                    (0.0, 0.0),
                    (r * 1.00, 0.0),
                    (r * 1.00, h * 0.06),
                    (r * 0.90, h * 0.10),
                    (r * 0.62, h * 0.34),
                    (r * 0.56, h * 0.88),
                    (r * 0.58, h),
                    (0.0, h),
                ],
                (0.0, 0.0, 0.0),
                AXIS_Z,
                "primary",
            )
        )
        parts += _bolt_ring(6, r * 0.76, 0.0, r * 0.055, h * 0.05)
        return parts

    # 工业机型：法兰盘 + 锥柱 + 地脚螺栓
    parts.append(
        _revolve_part(
            [
                (0.0, 0.0),
                (r * 1.00, 0.0),
                (r * 1.00, h * 0.10),
                (r * 0.94, h * 0.14),
                (r * 0.70, h * 0.30),
                (r * 0.64, h * 0.80),
                (r * 0.66, h * 0.92),
                (0.0, h),
            ],
            (0.0, 0.0, 0.0),
            AXIS_Z,
            "dark",
        )
    )
    # 主色外罩：套在锥柱下半部，形成"底座黑、机身品牌色"的分层。
    # 轮廓首尾不落到 r=0，是一个无端盖的套筒，故意保留开口以省面数。
    parts.append(
        _revolve_part(
            [
                (r * 0.68, 0.0),
                (r * 0.66, h * 0.52),
                (r * 0.645, h * 0.62),
            ],
            (0.0, 0.0, h * 0.30),
            AXIS_Z,
            "primary",
        )
    )
    parts += _bolt_ring(4, r * 0.82, 0.0, r * 0.075, h * 0.055)
    return parts


def turret(g: GeometryProfile, pal: _Palette, cobot: bool) -> List[_Part]:
    """腰转台：主体 + J2 叉臂 + 线缆出口。"""
    r = g.turret_radius
    h = g.turret_height
    t = g.link_thickness
    parts: List[_Part] = []

    if cobot:
        parts.append(
            _revolve_part(
                [
                    (0.0, 0.0),
                    (r * 0.92, 0.0),
                    (r * 0.96, h * 0.22),
                    (r * 0.88, h * 0.80),
                    (r * 0.80, h),
                    (0.0, h),
                ],
                (0.0, 0.0, 0.0),
                AXIS_Z,
                "primary",
            )
        )
        parts.append(
            _revolve_part(
                [(0.0, 0.0), (t * 0.40, 0.0), (t * 0.40, h * 0.10), (0.0, h * 0.10)],
                (0.0, 0.0, h * 0.86), AXIS_Z, "accent", segments=20,
            )
        )
        return parts

    # 主体：下粗上细，顶部收成叉臂座
    parts.append(
        _revolve_part(
            [
                (0.0, 0.0),
                (r * 1.00, 0.0),
                (r * 1.00, h * 0.18),
                (r * 0.96, h * 0.30),
                (r * 0.82, h * 0.72),
                (r * 0.62, h * 0.88),
                (0.0, h),
            ],
            (0.0, 0.0, 0.0),
            AXIS_Z,
            "primary",
        )
    )
    # J2 叉臂：两片耳板夹住大臂根部，工业臂的标志性结构
    for sign in (+1.0, -1.0):
        parts.append(
            _rbox_part(
                t * 0.42, t * 0.34, h * 0.62, t * 0.14,
                (0.0, sign * r * 0.56, h * 0.74),
                AXIS_Z, "primary",
            )
        )
        # 轴承端盖
        parts.append(
            _revolve_part(
                [
                    (0.0, 0.0), (t * 0.30, 0.0), (t * 0.30, t * 0.10),
                    (t * 0.24, t * 0.14), (0.0, t * 0.14),
                ],
                (0.0, sign * r * 0.56, h * 0.86), AXIS_Y, "metal", segments=20,
            )
        )
    # 后部线缆出口（工业臂从腰背后出线）
    parts.append(
        _tube_part(
            [(-r * 0.55, 0.0, h * 0.30), (-r * 0.78, 0.0, h * 0.18),
             (-r * 0.92, 0.0, h * 0.05)],
            t * 0.075, "dark",
        )
    )
    return parts


def upper_arm(g: GeometryProfile, pal: _Palette, cobot: bool) -> List[_Part]:
    """大臂：变截面铸造主梁 + J2 配重 + 侧脊 + 线缆 + J3 电机壳。

    这是整机观感差距最大的一处。基本体配方用两段不同粗细的箱体硬凑锥度，
    接缝处必定出现台阶；这里用放样体一次成型，从肩到肘连续收窄，且截面
    由圆角矩形逐渐过渡到更圆润的形状。
    """
    length = g.upper_arm
    t = g.link_thickness
    parts: List[_Part] = []

    if cobot:
        # 协作臂：等径圆管 + 两端关节环
        parts.append(
            _revolve_part(
                [
                    (0.0, 0.0), (t * 0.50, 0.0), (t * 0.50, length * 0.94),
                    (t * 0.47, length), (0.0, length),
                ],
                (0.0, 0.0, 0.0), AXIS_Z, "primary",
            )
        )
        for z in (0.0, length):
            parts.append(
                _revolve_part(
                    [
                        (t * 0.40, -t * 0.17), (t * 0.56, -t * 0.15),
                        (t * 0.56, t * 0.15), (t * 0.40, t * 0.17),
                    ],
                    (0.0, 0.0, z), AXIS_Y, "joint", segments=24,
                )
            )
        # 品牌色腰线：开口薄环，贴在管身外表面略微外扩处
        parts.append(
            _revolve_part(
                [
                    (t * 0.505, 0.0),
                    (t * 0.505, length * 0.10),
                ],
                (0.0, 0.0, length * 0.66),
                AXIS_Z,
                "accent",
                segments=28,
            )
        )
        return parts

    # ---- 变截面主梁：肩粗 → 肘细 ---- #
    # 6 个截面描述一条连续收窄的铸造梁；squareness 从 4.4 递减到 3.2，
    # 越靠近腕部越圆润，这正是真机铸件的外观规律。
    parts.append(
        _loft_part(
            [
                (-length * 0.10, t * 0.50, t * 0.44, 4.4),
                (0.0, t * 0.50, t * 0.43, 4.3),
                (length * 0.22, t * 0.46, t * 0.39, 4.2),
                (length * 0.48, t * 0.41, t * 0.35, 4.0),
                (length * 0.74, t * 0.37, t * 0.31, 3.6),
                (length * 0.94, t * 0.34, t * 0.29, 3.4),
                (length, t * 0.33, t * 0.28, 3.3),
            ],
            (0.0, 0.0, 0.0),
            AXIS_Z,
            "primary",
            segments=40,
        )
    )

    # 侧向加强脊：铸件上那条凸起的筋，兼作品牌色点缀
    parts.append(
        _loft_part(
            [
                (length * 0.10, t * 0.53, t * 0.10, 3.0),
                (length * 0.50, t * 0.44, t * 0.09, 3.0),
                (length * 0.88, t * 0.36, t * 0.08, 3.0),
            ],
            (0.0, 0.0, 0.0),
            AXIS_Z,
            "accent",
            segments=24,
        )
    )

    # ---- J2 尾部配重：J2 轴线后方的铸铁块 ---- #
    # 用来平衡大臂+小臂的力矩，是工业臂最醒目的外形特征，缺了它一眼假
    parts.append(
        _rbox_part(
            t * 1.02, t * 0.86, length * 0.20, t * 0.16,
            (0.0, 0.0, -length * 0.075),
            AXIS_Z, "dark",
        )
    )
    parts.append(
        _revolve_part(
            [
                (0.0, 0.0), (t * 0.54, 0.0), (t * 0.54, length * 0.035),
                (t * 0.46, length * 0.055), (0.0, length * 0.055),
            ],
            (0.0, 0.0, -length * 0.195), AXIS_Z, "dark", segments=24,
        )
    )

    # ---- J2 轴套 + 端盖（沿 Y）---- #
    parts.append(
        _revolve_part(
            [
                (0.0, 0.0), (t * 0.56, 0.0), (t * 0.56, t * 1.16),
                (t * 0.50, t * 1.22), (0.0, t * 1.22),
            ],
            (0.0, -t * 0.61, 0.0), AXIS_Y, "metal", segments=24,
        )
    )
    for sign in (+1.0, -1.0):
        parts.append(
            _revolve_part(
                [
                    (0.0, 0.0), (t * 0.36, 0.0), (t * 0.36, t * 0.08),
                    (t * 0.30, t * 0.12), (0.0, t * 0.12),
                ],
                (0.0, sign * t * 0.66, 0.0), AXIS_Y, "dark", segments=20,
            )
        )

    # ---- J3 电机壳 + 肘部轴承座 ---- #
    parts.append(
        _revolve_part(
            [
                (0.0, 0.0), (t * 0.50, 0.0), (t * 0.50, t * 1.00),
                (t * 0.44, t * 1.08), (0.0, t * 1.08),
            ],
            (0.0, -t * 0.55, length * 0.94), AXIS_Y, "metal", segments=24,
        )
    )
    parts.append(
        _revolve_part(
            [
                (t * 0.30, 0.0), (t * 0.56, 0.0), (t * 0.56, t * 0.72),
                (t * 0.50, t * 0.80), (t * 0.30, t * 0.80),
            ],
            (0.0, -t * 0.40, length), AXIS_Y, "accent", segments=24,
        )
    )

    # ---- 线缆：沿大臂背面走，带一点下垂弧度 ---- #
    parts.append(
        _tube_part(
            [
                (-t * 0.40, 0.0, length * 0.06),
                (-t * 0.46, 0.0, length * 0.34),
                (-t * 0.42, 0.0, length * 0.62),
                (-t * 0.36, 0.0, length * 0.86),
            ],
            t * 0.075, "dark",
        )
    )
    return parts


def forearm(g: GeometryProfile, pal: _Palette, cobot: bool) -> List[_Part]:
    """小臂：更细的变截面梁 + 腕部电机壳 + 线缆。"""
    length = g.forearm
    t = g.link_thickness
    parts: List[_Part] = []

    if cobot:
        parts.append(
            _revolve_part(
                [
                    (0.0, 0.0), (t * 0.44, 0.0), (t * 0.42, length * 0.92),
                    (t * 0.38, length), (0.0, length),
                ],
                (0.0, 0.0, 0.0), AXIS_Z, "primary",
            )
        )
        for z in (0.0, length):
            parts.append(
                _revolve_part(
                    [
                        (t * 0.35, -t * 0.15), (t * 0.50, -t * 0.13),
                        (t * 0.50, t * 0.13), (t * 0.35, t * 0.15),
                    ],
                    (0.0, 0.0, z), AXIS_Y, "joint", segments=24,
                )
            )
        return parts

    # 变截面主梁：肘端粗 → 腕端细
    parts.append(
        _loft_part(
            [
                (0.0, t * 0.40, t * 0.35, 4.2),
                (length * 0.26, t * 0.36, t * 0.31, 4.0),
                (length * 0.55, t * 0.31, t * 0.27, 3.7),
                (length * 0.80, t * 0.27, t * 0.24, 3.4),
                (length, t * 0.25, t * 0.22, 3.2),
            ],
            (0.0, 0.0, 0.0),
            AXIS_Z,
            "primary",
            segments=36,
        )
    )
    # 侧脊
    parts.append(
        _loft_part(
            [
                (length * 0.08, t * 0.42, t * 0.08, 3.0),
                (length * 0.50, t * 0.34, t * 0.07, 3.0),
                (length * 0.90, t * 0.26, t * 0.06, 3.0),
            ],
            (0.0, 0.0, 0.0), AXIS_Z, "accent", segments=20,
        )
    )
    # 肘部连接座
    parts.append(
        _revolve_part(
            [
                (t * 0.26, 0.0), (t * 0.46, 0.0), (t * 0.46, t * 0.66),
                (t * 0.40, t * 0.74), (t * 0.26, t * 0.74),
            ],
            (0.0, -t * 0.37, 0.0), AXIS_Y, "accent", segments=24,
        )
    )
    # 腕部电机壳
    parts.append(
        _revolve_part(
            [
                (0.0, 0.0), (t * 0.38, 0.0), (t * 0.38, t * 0.62),
                (t * 0.32, t * 0.70), (0.0, t * 0.70),
            ],
            (0.0, -t * 0.35, length * 0.90), AXIS_Y, "metal", segments=24,
        )
    )
    # 线缆
    parts.append(
        _tube_part(
            [
                (-t * 0.32, 0.0, length * 0.08),
                (-t * 0.37, 0.0, length * 0.42),
                (-t * 0.32, 0.0, length * 0.80),
            ],
            t * 0.065, "dark",
        )
    )
    return parts


def elbow(g: GeometryProfile, pal: _Palette, cobot: bool) -> List[_Part]:
    """7 轴机型特有的肘部自转段。"""
    from .urdf_builder import _elbow_length_from_profile

    t = g.link_thickness
    length = _elbow_length_from_profile(g)
    return [
        _revolve_part(
            [
                (0.0, 0.0), (t * 0.44, 0.0), (t * 0.44, length * 0.86),
                (t * 0.40, length), (0.0, length),
            ],
            (0.0, 0.0, 0.0), AXIS_Z, "accent",
        ),
        _revolve_part(
            [
                (t * 0.30, 0.0), (t * 0.47, 0.0), (t * 0.47, t * 0.18),
                (t * 0.30, t * 0.18),
            ],
            (0.0, 0.0, length * 0.40), AXIS_Z, "metal", segments=24,
        ),
    ]


def wrist_roll(g: GeometryProfile, pal: _Palette, cobot: bool) -> List[_Part]:
    """腕自转段：带台阶的回转壳体。"""
    length = g.wrist_1
    t = g.link_thickness
    return [
        _revolve_part(
            [
                (0.0, 0.0), (t * 0.40, 0.0), (t * 0.39, length * 0.30),
                (t * 0.34, length * 0.72), (t * 0.32, length),
                (0.0, length),
            ],
            (0.0, 0.0, 0.0), AXIS_Z, "primary",
        ),
        _revolve_part(
            [
                (t * 0.28, 0.0), (t * 0.43, 0.0), (t * 0.43, t * 0.16),
                (t * 0.28, t * 0.16),
            ],
            (0.0, 0.0, length * 0.42), AXIS_Z, "joint", segments=24,
        ),
        _revolve_part(
            [
                (0.0, 0.0), (t * 0.34, 0.0), (t * 0.34, t * 0.12),
                (t * 0.28, t * 0.17), (0.0, t * 0.17),
            ],
            (0.0, 0.0, length), AXIS_Z, "metal", segments=24,
        ),
    ]


def wrist_pitch(g: GeometryProfile, pal: _Palette, cobot: bool) -> List[_Part]:
    """腕俯仰段：叉形壳体 + 轴承盖。"""
    length = g.wrist_2
    t = g.link_thickness
    parts = [
        _revolve_part(
            [
                (0.0, 0.0), (t * 0.38, 0.0), (t * 0.38, length * 0.72),
                (t * 0.34, length), (0.0, length),
            ],
            (0.0, 0.0, 0.0), AXIS_Z, "primary",
        ),
    ]
    for sign in (+1.0, -1.0):
        parts.append(
            _revolve_part(
                [
                    (0.0, 0.0), (t * 0.30, 0.0), (t * 0.30, t * 0.10),
                    (t * 0.24, t * 0.14), (0.0, t * 0.14),
                ],
                (0.0, sign * t * 0.32, length * 0.50), AXIS_Y, "dark", segments=20,
            )
        )
    return parts


def flange(g: GeometryProfile, pal: _Palette, cobot: bool) -> List[_Part]:
    """工具法兰：法兰盘 + 螺栓圈 + 定位销。

    法兰是用户装夹治具的界面，上面必须有螺栓孔和定位销——这是"能干活
    的机器人"和"示意图"的分界线。
    """
    r = g.flange_radius
    length = g.flange_length
    parts = [
        _revolve_part(
            [
                (0.0, 0.0), (r * 0.94, 0.0), (r * 1.00, length * 0.22),
                (r * 1.00, length * 0.74), (r * 0.88, length * 0.90),
                (0.0, length),
            ],
            (0.0, 0.0, 0.0), AXIS_Z, "metal",
        ),
        # 定位销（ISO 9409-1 法兰的典型特征）
        _revolve_part(
            [
                (0.0, 0.0), (r * 0.14, 0.0), (r * 0.14, length * 0.16),
                (r * 0.10, length * 0.20), (0.0, length * 0.20),
            ],
            (0.0, r * 0.78, length * 0.90), AXIS_Z, "dark", segments=12,
        ),
    ]
    parts += _bolt_ring(6, r * 0.72, length * 0.90, r * 0.085, length * 0.10)
    return parts


def tcp(g: GeometryProfile, pal: _Palette, cobot: bool = False) -> List[_Part]:
    """TCP 标记：小锥体，指向工具接近方向。"""
    r = g.flange_radius
    return [
        _revolve_part(
            [
                (0.0, 0.0), (r * 0.22, 0.0), (r * 0.16, r * 0.30),
                (0.0, r * 0.34),
            ],
            (0.0, 0.0, 0.0), AXIS_Z, "marker", segments=16,
        ),
    ]


#: 高保真配方分派表，键与 :mod:`urdf_builder` 中的基本体配方一一对应。
HIFI_BUILDERS = {
    "base": base,
    "turret": turret,
    "upper_arm": upper_arm,
    "forearm": forearm,
    "elbow": elbow,
    "wrist_roll": wrist_roll,
    "wrist_pitch": wrist_pitch,
    "flange": flange,
    "tcp": tcp,
}
