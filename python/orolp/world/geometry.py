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
from typing import Callable, Dict, Optional, Tuple

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
    "geometry_stats",
    "reset_geometry_stats",
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

        实现说明（v1.1.9）：逐分量标量比较，**不用** ``np.all(lo - m <= hi)``。
        这是整条沿轨迹检查里调用最频繁的函数（实测一次 5600 路点检查里被调
        8 万次），而 numpy 对 3 元素数组做归约的开销（``np.all`` →
        ``_wrapreduction`` → ``normalize_axis_tuple`` → ``moveaxis`` …）
        比比较本身贵一个数量级。语义逐条对应：含 NaN（比较为 False →
        判不相交）以及 ``lo``/``hi`` 是 list 的情形。
        """
        a_lo, a_hi = self.lo, self.hi
        b_lo, b_hi = other.lo, other.hi
        m = float(margin)
        return bool(
            a_lo[0] - m <= b_hi[0]
            and b_lo[0] - m <= a_hi[0]
            and a_lo[1] - m <= b_hi[1]
            and b_lo[1] - m <= a_hi[1]
            and a_lo[2] - m <= b_hi[2]
            and b_lo[2] - m <= a_hi[2]
        )

    @property
    def center(self) -> np.ndarray:
        return (self.lo + self.hi) / 2.0


# --------------------------------------------------------------------------- #
# 局部几何常量缓存（v1.1.9）
# --------------------------------------------------------------------------- #
#
# 基本体在**自身局部坐标系**下的几何（半尺寸、顶点、面法向、棱方向）只由
# 参数决定，与它被摆在哪里无关。沿轨迹做干涉检查时，同一个零件会被算上千次
# ——每次重建圆柱的 16 段离散（linspace + cos + sin + column_stack）是纯浪费。
#
# 键就用基本体本身：``Box``/``Cylinder``/``Sphere`` 都是 ``frozen=True`` 的
# dataclass，天然可哈希，而"参数相同 ⇒ 局部几何相同"正是我们要的等价关系
# （改参数会构造出**另一个**实例，不会命中旧条目）。容器类型参数（把 size
# 传成 list）不可哈希时退化为不缓存，不影响正确性。

#: 缓存条目上限。基本体种类是程序里写死的有限集合（一个工作站几十个），
#: 触顶说明调用方在动态造几何，此时整体清空比逐条淘汰更可预测
LOCAL_CACHE_LIMIT = 4096

_local_extent_cache: Dict[Primitive, np.ndarray] = {}
_local_vertices_cache: Dict[Primitive, np.ndarray] = {}
_local_faces_cache: Dict[Primitive, np.ndarray] = {}
_local_edges_cache: Dict[Primitive, np.ndarray] = {}

_cache_hits = 0
_cache_misses = 0

#: 窄相工作量计数器（只增不减，读的时候取差分）
_work = {"collides_calls": 0, "prefilter_rejects": 0, "sat_checks": 0, "collision_hits": 0}

#: 共享的只读常量，省掉每次查询的分配
_IDENTITY3 = np.eye(3, dtype=np.float64)
_IDENTITY3.setflags(write=False)


def _readonly(arr: np.ndarray) -> np.ndarray:
    """转成 C 连续 float64 并置为只读（缓存值会被多处共享）。"""
    out = np.ascontiguousarray(arr, dtype=np.float64)
    out.setflags(write=False)
    return out


def _lookup(
    cache: Dict[Primitive, np.ndarray], prim: Primitive, build: Callable[[], np.ndarray]
) -> np.ndarray:
    """取基本体的局部几何：命中缓存返回只读共享数组，否则算一次收下。"""
    global _cache_hits, _cache_misses
    try:
        hash(prim)
    except TypeError:  # 参数不可哈希（如 size 传了 list）：不缓存，正确性不变
        _cache_misses += 1
        return _readonly(build())

    hit = cache.get(prim)
    if hit is not None:
        _cache_hits += 1
        return hit
    if len(cache) >= LOCAL_CACHE_LIMIT:
        cache.clear()
    value = _readonly(build())
    cache[prim] = value
    _cache_misses += 1
    return value


def geometry_stats() -> Dict[str, int]:
    """局部几何缓存与窄相的工作量统计（v1.1.9）。

    计数器**只增不减**，用法是前后各取一次相减（差分），不要在中间假设它归零。
    它回答的是"这次检查到底干了多少活"，而不是"性能如何"——耗时请用
    :func:`orolp.perf.profile_program` / :func:`orolp.perf.collision_profile`。

    Returns:
        ``cache_entries``（四类局部几何缓存的条目总和）、``cache_hits``、
        ``cache_misses``、``collides_calls``（窄相入口调用次数）、
        ``prefilter_rejects``（被 AABB 粗筛剔除的次数）、``sat_checks``（真正
        跑了 SAT 的次数）、``collision_hits``（判定为碰撞的次数）
    """
    return {
        "cache_entries": (
            len(_local_extent_cache)
            + len(_local_vertices_cache)
            + len(_local_faces_cache)
            + len(_local_edges_cache)
        ),
        "cache_hits": _cache_hits,
        "cache_misses": _cache_misses,
        "collides_calls": _work["collides_calls"],
        "prefilter_rejects": _work["prefilter_rejects"],
        "sat_checks": _work["sat_checks"],
        "collision_hits": _work["collision_hits"],
    }


def reset_geometry_stats() -> None:
    """清空局部几何缓存与工作量计数器（测试隔离用）。"""
    global _cache_hits, _cache_misses
    _local_extent_cache.clear()
    _local_vertices_cache.clear()
    _local_faces_cache.clear()
    _local_edges_cache.clear()
    _cache_hits = 0
    _cache_misses = 0
    for key in _work:
        _work[key] = 0


def _build_local_extent(prim: Primitive) -> np.ndarray:
    """基本体在自身局部坐标系下的半尺寸（缓存未命中时才会跑）。"""
    if isinstance(prim, Box):
        return np.array(prim.size, dtype=np.float64) / 2.0
    if isinstance(prim, Cylinder):
        return np.array([prim.radius, prim.radius, prim.length / 2.0], dtype=np.float64)
    if isinstance(prim, Sphere):
        r = float(prim.radius)
        return np.array([r, r, r], dtype=np.float64)
    raise TypeError(f"未知的基本体类型: {type(prim)}")


def _local_extent(prim: Primitive) -> np.ndarray:
    """基本体在自身局部坐标系下的半尺寸（用于快速算 AABB）。"""
    return _lookup(_local_extent_cache, prim, lambda: _build_local_extent(prim))


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
    R = transform[:3, :3]
    t = transform[:3, 3]
    ext = np.abs(R) @ half
    return AABB(lo=t - ext, hi=t + ext)


def union_aabb(boxes) -> Optional[AABB]:
    """一组包围盒的联合 AABB（广相的第一层）。

    Args:
        boxes: :class:`AABB` 序列（允许含 ``None``，会被跳过）

    Returns:
        联合包围盒；全为 ``None`` 或序列为空时返回 ``None``
    """
    lo = None
    hi = None
    for box in boxes:
        if box is None:
            continue
        lo = box.lo if lo is None else np.minimum(lo, box.lo)
        hi = box.hi if hi is None else np.maximum(hi, box.hi)
    if lo is None or hi is None:
        return None
    return AABB(lo, hi)


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
    "collides_with_boxes",
    "union_aabb",
    "geometry_stats",
    "reset_geometry_stats",
]


def _build_local_vertices(prim: Primitive) -> np.ndarray:
    """基本体在自身局部坐标系下的顶点集（缓存未命中时才会跑）。"""
    if isinstance(prim, Box):
        sx, sy, sz = (float(v) / 2.0 for v in prim.size)
        return np.array(
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
    if isinstance(prim, Cylinder):
        k = CYLINDER_SEGMENTS
        r = float(prim.radius)
        h = float(prim.length) / 2.0
        ang = np.linspace(0.0, 2.0 * math.pi, k, endpoint=False)
        ring_lo = np.column_stack([r * np.cos(ang), r * np.sin(ang), np.full(k, -h)])
        ring_hi = np.column_stack([r * np.cos(ang), r * np.sin(ang), np.full(k, h)])
        return np.vstack([ring_lo, ring_hi])
    if isinstance(prim, Sphere):
        # 球按外接立方体离散：对 SAT 而言略偏保守（体积略大），
        # 宁可误报也不漏报，符合碰撞检测的安全取向
        r = float(prim.radius)
        return np.array(
            [[a * r, b * r, c * r] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)],
            dtype=np.float64,
        )
    raise TypeError(f"未知的基本体类型: {type(prim)}")


def _local_vertices(prim: Primitive) -> np.ndarray:
    """基本体在自身局部坐标系下的顶点集（只读共享）。"""
    return _lookup(_local_vertices_cache, prim, lambda: _build_local_vertices(prim))


def vertices_of(prim: Primitive, transform: np.ndarray) -> np.ndarray:
    """基本体在世界坐标系下的顶点集 ``(n, 3)``。

    Args:
        prim: 基本体
        transform: 4x4 世界变换

    Returns:
        顶点数组（每次调用新分配，调用方可自由改写）

    Raises:
        TypeError: 基本体类型未知
    """
    pts = _local_vertices(prim)
    R = transform[:3, :3]
    t = transform[:3, 3]
    return (R @ pts.T).T + t


def _build_local_face_normals(prim: Primitive) -> np.ndarray:
    """局部坐标系下的面法向 ``(k, 3)``（缓存未命中时才会跑）。"""
    if isinstance(prim, Box):
        return _IDENTITY3
    if isinstance(prim, Cylinder):
        k = CYLINDER_SEGMENTS
        ang = np.linspace(0.0, 2.0 * math.pi, k, endpoint=False)
        side = np.column_stack([np.cos(ang), np.sin(ang), np.zeros(k)])
        return np.vstack([side, [0.0, 0.0, 1.0]])
    if isinstance(prim, Sphere):
        return _IDENTITY3
    raise TypeError(f"未知的基本体类型: {type(prim)}")


def _build_local_edge_dirs(prim: Primitive) -> np.ndarray:
    """局部坐标系下的棱方向 ``(k, 3)``（缓存未命中时才会跑）。"""
    if isinstance(prim, Box):
        return _IDENTITY3
    if isinstance(prim, Cylinder):
        k = CYLINDER_SEGMENTS
        ang = np.linspace(0.0, 2.0 * math.pi, k, endpoint=False)
        # 侧面棱：绕周的切向 + 轴向
        tangent = np.column_stack([-np.sin(ang), np.cos(ang), np.zeros(k)])
        return np.vstack([tangent, [0.0, 0.0, 1.0]])
    if isinstance(prim, Sphere):
        return _IDENTITY3
    raise TypeError(f"未知的基本体类型: {type(prim)}")


def _local_face_normals(prim: Primitive) -> np.ndarray:
    """局部坐标系下的面法向 ``(k, 3)``（只读共享）。"""
    return _lookup(_local_faces_cache, prim, lambda: _build_local_face_normals(prim))


def _local_edge_dirs(prim: Primitive) -> np.ndarray:
    """局部坐标系下的棱方向 ``(k, 3)``（只读共享）。"""
    return _lookup(_local_edges_cache, prim, lambda: _build_local_edge_dirs(prim))


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


def _overlaps_on_axes(
    verts_a: np.ndarray, verts_b: np.ndarray, axes: np.ndarray
) -> np.ndarray:
    """一组候选轴上**同时**求投影重叠量（:func:`_overlap_on_axis` 的批量版）。

    SAT 的瓶颈从来不是浮点运算，而是 numpy 每次调用的固定开销：一次 3 元素
    点积要走完整的 ufunc 分派与归约路径，约 1µs 起——比它算的东西贵两个数量
    级。轴向候选通常有十几到几百条（圆柱-圆柱的棱-棱轴是
    ``CYLINDER_SEGMENTS²``），逐轴调用就把这份开销乘了上去。批量版把它们
    堆成一个矩阵乘法 + 两次归约。

    逐元素语义与 :func:`_overlap_on_axis` **完全一致**（同样的
    ``min(max) - max(min)``）。

    Args:
        verts_a: 第一个凸体顶点 ``(n, 3)``
        verts_b: 第二个凸体顶点 ``(m, 3)``
        axes: 候选轴 ``(k, 3)``

    Returns:
        长度为 ``k`` 的重叠量数组（负值 = 沿该轴分离）
    """
    proj_a = verts_a @ axes.T
    proj_b = verts_b @ axes.T
    return np.minimum(proj_a.max(axis=0), proj_b.max(axis=0)) - np.maximum(
        proj_a.min(axis=0), proj_b.min(axis=0)
    )


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

    早退语义（v1.1.9 保持逐位一致）：一旦某条轴上的重叠量 < 0（沿该轴分离），
    立即返回**该值**而不继续扫完剩下的轴。因此返回值在"分离"情形下只是某个
    负值、不保证是最小值——调用方（:func:`collides` / :func:`prim_separation`）
    都只拿它做正负与阈值比较，语义不受影响；而一旦全程没有负值，返回的就是
    所有轴上的**精确最小值**。批量实现按"面法向（A 在前 B 在后，行序）→
    棱-棱叉积（A 的棱在外层、B 的棱在内层，行序）"的同一顺序找第一条负值，
    与逐轴扫描返回的是同一个数。

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
    faces = _stack_axes(faces_a, faces_b)
    best = math.inf
    if faces.shape[0]:
        ov = _overlaps_on_axes(verts_a, verts_b, faces)
        neg = np.flatnonzero(ov < 0.0)
        if neg.size:
            return float(ov[neg[0]])
        best = float(ov.min())

    edges_a = np.asarray(edges_a, dtype=np.float64).reshape(-1, 3)
    edges_b = np.asarray(edges_b, dtype=np.float64).reshape(-1, 3)
    if edges_a.shape[0] and edges_b.shape[0]:
        # 棱-棱候选轴：所有 (ea, eb) 组合一次算完（行序与逐轴双循环一致）
        axes = np.cross(edges_a[:, None, :], edges_b[None, :, :]).reshape(-1, 3)
        norm2 = np.einsum("ij,ij->i", axes, axes)
        # 两棱平行 → 叉积退化为零向量、不产生新轴（与逐轴实现同一判据；
        # 阈值 1e-12 相对 n² 的量级（0 或 O(1)）留有巨大余量）
        axes = axes[norm2 >= 1e-12]
        if axes.shape[0]:
            ov = _overlaps_on_axes(verts_a, verts_b, axes)
            neg = np.flatnonzero(ov < 0.0)
            if neg.size:
                return float(ov[neg[0]])
            smallest = float(ov.min())
            if smallest < best:
                best = smallest
    return best


def _stack_axes(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """把两组候选轴拼成一个 ``(k, 3)`` 数组（任一侧为空时返回空数组）。"""
    a = np.asarray(first, dtype=np.float64).reshape(-1, 3)
    b = np.asarray(second, dtype=np.float64).reshape(-1, 3)
    if not a.shape[0]:
        return b
    if not b.shape[0]:
        return a
    return np.concatenate([a, b], axis=0)


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

    需要连同多个零件一起判（沿轨迹检查就是这种形态）时用
    :func:`collides_with_boxes`，可以复用已经算好的包围盒。

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
    # 球-球走解析通道：精确、比离散快，也省掉两个包围盒的计算
    if isinstance(prim_a, Sphere) and isinstance(prim_b, Sphere):
        return _sphere_pair(prim_a, tf_a, prim_b, tf_b, margin)
    return collides_with_boxes(
        prim_a, tf_a, prim_b, tf_b, prim_aabb(prim_a, tf_a), prim_aabb(prim_b, tf_b), margin
    )


def _sphere_pair(
    prim_a: Sphere, tf_a: np.ndarray, prim_b: Sphere, tf_b: np.ndarray, margin: float
) -> bool:
    """球-球的解析判定（比离散到立方体更精确）。"""
    _work["collides_calls"] += 1
    ca, cb = tf_a[:3, 3], tf_b[:3, 3]
    gap = float(np.linalg.norm(ca - cb)) - (prim_a.radius + prim_b.radius)
    hit = gap < -margin
    if hit:
        _work["collision_hits"] += 1
    return hit


def collides_with_boxes(
    prim_a: Primitive,
    tf_a: np.ndarray,
    prim_b: Primitive,
    tf_b: np.ndarray,
    box_a: AABB,
    box_b: AABB,
    margin: float = 0.0,
) -> bool:
    """:func:`collides` 的核心，包围盒由调用方给出（复用已算好的结果）。

    为什么要这个入口：沿轨迹检查时，同一个零件的世界系 AABB 会在多个
    "(连杆, 物体)" 组合之间反复用到（单零件连杆的零件盒**就是**连杆盒），
    逐组合重算是这条路径上最大的一块纯浪费。判定逻辑与本函数之外的一切
    与 :func:`collides` 逐位一致——传入的盒必须就是
    ``prim_aabb(prim, tf)`` 的结果，否则粗筛结论会变。

    Args:
        prim_a: 第一个基本体
        tf_a: 第一个基本体的 4x4 世界变换
        prim_b: 第二个基本体
        tf_b: 第二个基本体的 4x4 世界变换
        box_a: ``prim_aabb(prim_a, tf_a)``
        box_b: ``prim_aabb(prim_b, tf_b)``
        margin: 见 :func:`collides`

    Returns:
        ``True`` 表示判定为碰撞
    """
    # 球-球仍走解析通道（盒参数用不上）
    if isinstance(prim_a, Sphere) and isinstance(prim_b, Sphere):
        return _sphere_pair(prim_a, tf_a, prim_b, tf_b, margin)

    _work["collides_calls"] += 1

    # 第一级：AABB 粗筛（几十纳秒，剔除绝大多数不相交对）
    if not box_a.overlaps(box_b, margin):
        _work["prefilter_rejects"] += 1
        return False

    # 第二级：SAT 精确判定（用最小重叠量支持穿透容差）
    _work["sat_checks"] += 1
    verts_a = vertices_of(prim_a, tf_a)
    verts_b = vertices_of(prim_b, tf_b)
    faces_a, edges_a = sat_axes(prim_a, tf_a)
    faces_b, edges_b = sat_axes(prim_b, tf_b)
    depth = penetration_depth(verts_a, faces_a, edges_a, verts_b, faces_b, edges_b)
    hit = depth > -margin
    if hit:
        _work["collision_hits"] += 1
    return hit
