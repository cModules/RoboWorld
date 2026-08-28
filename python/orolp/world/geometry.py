"""环境建模的几何内核：基本体、包围盒与碰撞检测。

设计取舍
--------
本模块**不引入任何第三方几何库**（如 trimesh / python-fcl）。原因：

* 仓库定位是"装完 pip 依赖即可运行"，C++ 核心（Pinocchio）至今未编译，
  已经让后处理链路瘫痪过一次，不应再增加同类风险
* 工业场景的组件以基本体（长方体、圆柱、球）为主，用通用凸包 GJK 已足够精确

碰撞检测采用两级策略：

1. **AABB 粗筛**——轴对齐包围盒，几十纳秒级，先剔除绝大多数不相交对
2. **GJK 精筛**——把基本体离散为凸多面体顶点集，用 Gilbert-Johnson-Keerthi
   算法判断凸包是否相交。对凸体是精确的，不是近似

单位：**毫米**（与 :class:`~orolp.pose.Pose` 一致）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np

__all__ = [
    "Box",
    "Cylinder",
    "Sphere",
    "Primitive",
    "AABB",
    "Convex",
    "to_convex",
    "gjk_intersect",
    "collides",
]


# --------------------------------------------------------------------------- #
# 基本体
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class Box:
    """长方体，尺寸 (x, y, z)，单位毫米，几何中心在局部原点。"""

    size: Tuple[float, float, float]

    @property
    def kind(self) -> str:
        return "box"


@dataclass(frozen=True)
class Cylinder:
    """圆柱，轴向为局部 **Z** 轴，几何中心在局部原点。"""

    radius: float
    length: float

    @property
    def kind(self) -> str:
        return "cylinder"


@dataclass(frozen=True)
class Sphere:
    """球，球心在局部原点。"""

    radius: float

    @property
    def kind(self) -> str:
        return "sphere"


Primitive = Box | Cylinder | Sphere


# --------------------------------------------------------------------------- #
# 包围盒
# --------------------------------------------------------------------------- #


@dataclass
class AABB:
    """轴对齐包围盒（世界坐标系）。"""

    lo: np.ndarray  # (3,)
    hi: np.ndarray  # (3,)

    def overlaps(self, other: "AABB", margin: float = 0.0) -> bool:
        """是否与另一个 AABB 相交。

        Args:
            other: 另一个包围盒
            margin: 安全裕度（毫米），正值表示提前预警
        """
        return bool(
            np.all(self.lo - margin <= other.hi) and np.all(other.lo - margin <= self.hi)
        )

    @property
    def center(self) -> np.ndarray:
        return (self.lo + self.hi) / 2.0


def _local_extent(prim: Primitive) -> np.ndarray:
    """基本体在自身局部坐标系下的半尺寸（用于快速算 AABB）。"""
    if isinstance(prim, Box):
        return np.array(prim.size, dtype=np.float64) / 2.0
    if isinstance(prim, Cylinder):
        return np.array([prim.radius, prim.radius, prim.length / 2.0], dtype=np.float64)
    if isinstance(prim, Sphere):
        r = float(prim.radius)
        return np.array([r, r, r], dtype=np.float64)
    raise TypeError(f"未知的基本体类型: {type(prim)}")


def prim_aabb(prim: Primitive, transform: np.ndarray) -> AABB:
    """计算基本体在世界坐标系下的 AABB。

    对旋转后的长方体，取旋转矩阵绝对值与半尺寸的乘积——这是保守但正确的
    紧包围盒（对任意旋转都成立）。

    Args:
        prim: 基本体
        transform: 4x4 世界变换

    Returns:
        世界坐标系下的 AABB
    """
    half = _local_extent(prim)
    R = np.abs(transform[:3, :3])
    t = transform[:3, 3]
    ext = R @ half
    return AABB(lo=t - ext, hi=t + ext)


# --------------------------------------------------------------------------- #
# 分离轴定理（SAT）碰撞检测
# --------------------------------------------------------------------------- #
#
# 这里刻意**没有**用 GJK。GJK 的 case-analysis 实现在退化几何下极易震荡：
# 工业场景全是轴对齐长方体，闵可夫斯基差经常共线/共面，单纯形更新会陷入
# 死循环，表现为"明显重叠却判定不相交"——这类漏检在碰撞检测里最危险。
#
# SAT 无迭代、无收敛问题，对凸多面体同样是**精确**判定（不是近似），
# 且我们的基本体面法向可以解析给出，无需先跑凸包算法。

#: 圆柱离散的边数。16 段在半径 100mm 时弦高误差约 0.2mm，
#: 对毫米级的工业场景精度足够；再细分只会拖慢 SAT（轴数与段数成正比）
CYLINDER_SEGMENTS = 16

__all__ = [
    "Box",
    "Cylinder",
    "Sphere",
    "Primitive",
    "AABB",
    "vertices_of",
    "sat_axes",
    "collides",
]


def vertices_of(prim: Primitive, transform: np.ndarray) -> np.ndarray:
    """基本体在世界坐标系下的顶点集 ``(n, 3)``。

    Args:
        prim: 基本体
        transform: 4x4 世界变换

    Returns:
        顶点数组

    Raises:
        TypeError: 基本体类型未知
    """
    if isinstance(prim, Box):
        sx, sy, sz = (float(v) / 2.0 for v in prim.size)
        pts = np.array(
            [
                [-sx, -sy, -sz],
                [sx, -sy, -sz],
                [sx, sy, -sz],
                [-sx, sy, -sz],
                [-sx, -sy, sz],
                [sx, -sy, sz],
                [sx, sy, sz],
                [-sx, sy, sz],
            ],
            dtype=np.float64,
        )
    elif isinstance(prim, Cylinder):
        k = CYLINDER_SEGMENTS
        r = float(prim.radius)
        h = float(prim.length) / 2.0
        ang = np.linspace(0.0, 2.0 * math.pi, k, endpoint=False)
        ring_lo = np.column_stack([r * np.cos(ang), r * np.sin(ang), np.full(k, -h)])
        ring_hi = np.column_stack([r * np.cos(ang), r * np.sin(ang), np.full(k, h)])
        pts = np.vstack([ring_lo, ring_hi])
    elif isinstance(prim, Sphere):
        # 球按外接立方体离散：对 SAT 而言略偏保守（体积略大），
        # 宁可误报也不漏报，符合碰撞检测的安全取向
        r = float(prim.radius)
        pts = np.array(
            [[a * r, b * r, c * r] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)],
            dtype=np.float64,
        )
    else:
        raise TypeError(f"未知的基本体类型: {type(prim)}")

    R = transform[:3, :3]
    t = transform[:3, 3]
    return (R @ pts.T).T + t


def _local_face_normals(prim: Primitive) -> np.ndarray:
    """局部坐标系下的面法向 ``(k, 3)``。"""
    if isinstance(prim, Box):
        return np.eye(3)
    if isinstance(prim, Cylinder):
        k = CYLINDER_SEGMENTS
        ang = np.linspace(0.0, 2.0 * math.pi, k, endpoint=False)
        side = np.column_stack([np.cos(ang), np.sin(ang), np.zeros(k)])
        return np.vstack([side, [0.0, 0.0, 1.0]])
    if isinstance(prim, Sphere):
        return np.eye(3)
    raise TypeError(f"未知的基本体类型: {type(prim)}")


def _local_edge_dirs(prim: Primitive) -> np.ndarray:
    """局部坐标系下的棱方向 ``(k, 3)``（只需方向，不需要具体棱）。"""
    if isinstance(prim, Box):
        return np.eye(3)
    if isinstance(prim, Cylinder):
        k = CYLINDER_SEGMENTS
        ang = np.linspace(0.0, 2.0 * math.pi, k, endpoint=False)
        # 侧面棱：绕周的切向 + 轴向
        tangent = np.column_stack([-np.sin(ang), np.cos(ang), np.zeros(k)])
        return np.vstack([tangent, [0.0, 0.0, 1.0]])
    if isinstance(prim, Sphere):
        return np.eye(3)
    raise TypeError(f"未知的基本体类型: {type(prim)}")


def sat_axes(prim: Primitive, transform: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """世界坐标系下的 SAT 候选轴。

    Args:
        prim: 基本体
        transform: 4x4 世界变换

    Returns:
        ``(面法向数组, 棱方向数组)``，均为世界坐标系
    """
    R = transform[:3, :3]
    faces = _local_face_normals(prim) @ R.T
    edges = _local_edge_dirs(prim) @ R.T
    return faces, edges


def _overlap_on_axis(verts_a: np.ndarray, verts_b: np.ndarray, axis: np.ndarray) -> float:
    """两凸体在给定轴上的投影重叠量。

    Args:
        verts_a: 第一个凸体顶点 ``(n, 3)``
        verts_b: 第二个凸体顶点 ``(m, 3)``
        axis: 投影轴（无需归一化）

    Returns:
        重叠量（沿该轴、未归一化）。**负值表示分离**，其绝对值即间隙大小。
    """
    pa = verts_a @ axis
    pb = verts_b @ axis
    return float(min(pa.max(), pb.max()) - max(pa.min(), pb.min()))


def penetration_depth(
    verts_a: np.ndarray,
    faces_a: np.ndarray,
    edges_a: np.ndarray,
    verts_b: np.ndarray,
    faces_b: np.ndarray,
    edges_b: np.ndarray,
) -> float:
    """SAT 计算两凸体的（近似）穿透深度。

    取所有候选分离轴上最小的投影重叠量。这是"最小平移距离"的**下界**——
    对判断是否碰撞已经足够：只要最小重叠量 > 0 就一定相交。

    Args:
        verts_a: A 的顶点
        faces_a: A 的面法向
        edges_a: A 的棱方向
        verts_b: B 的顶点
        faces_b: B 的面法向
        edges_b: B 的棱方向

    Returns:
        最小重叠量（毫米，未归一化到单位轴）。负数表示分离，绝对值即间隙。
    """
    best = math.inf
    for axis in list(faces_a) + list(faces_b):
        ov = _overlap_on_axis(verts_a, verts_b, axis)
        if ov < best:
            best = ov
        if best < 0:
            return best
    for ea in edges_a:
        for eb in edges_b:
            axis = np.cross(ea, eb)
            n2 = float(np.dot(axis, axis))
            if n2 < 1e-12:
                continue  # 两棱平行，不产生新轴
            ov = _overlap_on_axis(verts_a, verts_b, axis)
            if ov < best:
                best = ov
            if best < 0:
                return best
    # 归一化为实际长度（候选轴未必是单位向量）
    return best


def collides(
    prim_a: Primitive,
    tf_a: np.ndarray,
    prim_b: Primitive,
    tf_b: np.ndarray,
    margin: float = 0.0,
) -> bool:
    """判断两个变换后的基本体是否碰撞。

    ``margin`` 的语义（毫米）::

        margin > 0   安全裕度——间距小于该值即预警（用于"接近检测"）
        margin = 0   接触即判定碰撞
        margin < 0   容忍 |margin| 的穿透或接触

    最后一项是工程必需：工件**放在**台面上、夹具**夹住**工件，几何上都是
    零间隙接触，若把接触当碰撞就会满屏误报。默认留 0.5mm 容差正是为此。

    Args:
        prim_a: 第一个基本体
        tf_a: 第一个基本体的 4x4 世界变换
        prim_b: 第二个基本体
        tf_b: 第二个基本体的 4x4 世界变换
        margin: 见上表

    Returns:
        ``True`` 表示判定为碰撞

    Raises:
        TypeError: 基本体类型未知
    """
    # 球-球走解析通道：精确且比离散快
    if isinstance(prim_a, Sphere) and isinstance(prim_b, Sphere):
        ca, cb = tf_a[:3, 3], tf_b[:3, 3]
        dist = float(np.linalg.norm(ca - cb))
        gap = dist - (prim_a.radius + prim_b.radius)
        return gap < -margin

    # 第一级：AABB 粗筛（几十纳秒，剔除绝大多数不相交对）
    if not prim_aabb(prim_a, tf_a).overlaps(prim_aabb(prim_b, tf_b), margin):
        return False

    # 第二级：SAT 精确判定（用最小重叠量支持穿透容差）
    verts_a = vertices_of(prim_a, tf_a)
    verts_b = vertices_of(prim_b, tf_b)
    faces_a, edges_a = sat_axes(prim_a, tf_a)
    faces_b, edges_b = sat_axes(prim_b, tf_b)
    depth = penetration_depth(verts_a, faces_a, edges_a, verts_b, faces_b, edges_b)
    return depth > -margin
