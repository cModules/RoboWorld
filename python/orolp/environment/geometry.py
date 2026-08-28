"""凸体几何图元 —— 碰撞检测与渲染的**唯一**数据源。

为什么只支持凸体：碰撞检测内核用 GJK/EPA，它要求参与计算的形状是凸的。
机器人连杆、工作台、围栏立柱这类零件天然是凸的；非凸零件（如料仓、围栏
整体）在组件库中拆成多个凸体 part 即可，效果等价于凸分解。

单位：**米**。局部坐标系下，圆柱/圆锥的轴向为 **+Z**（与 URDF 一致）。

支撑函数（support function）是 GJK 唯一需要的接口::

    support(d) -> argmax_{p in shape} dot(p, d)

本模块对每种图元给出**解析**支撑函数，而非在离散顶点里取最大值——对圆柱
和球面，离散化会引入百分之几的距离误差，直接表现为碰撞检测"虚警/漏报"。
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from typing import ClassVar, Tuple

import numpy as np

_EPS = 1e-12


class ConvexGeometry(ABC):
    """凸体几何基类。

    Attributes:
        kind: 图元类型字符串，供渲染层走快速绘制路径
    """

    kind: ClassVar[str] = "abstract"

    # ------------------------------------------------------------------ #
    # GJK 内核接口
    # ------------------------------------------------------------------ #
    @abstractmethod
    def support(self, direction: np.ndarray) -> np.ndarray:
        """返回局部坐标系下沿 ``direction`` 最远的点。

        Args:
            direction: 任意非零三维方向向量（无需归一化）

        Returns:
            形状为 (3,) 的支撑点
        """
        raise NotImplementedError

    # ------------------------------------------------------------------ #
    # 渲染 / 包围盒接口
    # ------------------------------------------------------------------ #
    @abstractmethod
    def _make_vertices(self) -> np.ndarray:
        """生成用于包围盒与三角化的离散顶点，形状 (N, 3)。"""
        raise NotImplementedError

    @abstractmethod
    def _make_triangles(self) -> np.ndarray:
        """生成三角面片，形状 (M, 3, 3)，每片为局部坐标的三个顶点。"""
        raise NotImplementedError

    @property
    def vertices(self) -> np.ndarray:
        """离散顶点（惰性缓存），形状 (N, 3)。"""
        cached = self.__dict__.get("_vertices_cache")
        if cached is None:
            cached = np.ascontiguousarray(self._make_vertices(), dtype=np.float64)
            self.__dict__["_vertices_cache"] = cached
        return cached

    @property
    def triangles(self) -> np.ndarray:
        """三角面片（惰性缓存），形状 (M, 3, 3)。"""
        cached = self.__dict__.get("_triangles_cache")
        if cached is None:
            cached = np.ascontiguousarray(self._make_triangles(), dtype=np.float64)
            self.__dict__["_triangles_cache"] = cached
        return cached

    @property
    def bounding_radius(self) -> float:
        """以局部原点为中心的外接球半径，用于粗判加速。"""
        cached = self.__dict__.get("_radius_cache")
        if cached is None:
            cached = float(np.max(np.linalg.norm(self.vertices, axis=1)))
            self.__dict__["_radius_cache"] = cached
        return cached

    def aabb(self, T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """计算在世界变换 ``T`` 下的轴对齐包围盒。

        Args:
            T: 4x4 齐次变换矩阵（米）

        Returns:
            ``(lower, upper)``，各为形状 (3,) 的数组
        """
        R = T[:3, :3]
        t = T[:3, 3]
        # 直接变换全部顶点：N 通常 <= 200，比逐轴投影 8 角点更省心且同样快
        pts = self.vertices @ R.T + t
        return pts.min(axis=0), pts.max(axis=0)

    def __repr__(self) -> str:
        return f"{type(self).__name__}({self.describe()})"

    def describe(self) -> str:
        """供 UI 显示的参数摘要。"""
        return ""


class BoxGeometry(ConvexGeometry):
    """长方体，中心位于局部原点，边与坐标轴平行。"""

    kind = "box"

    def __init__(self, size: Tuple[float, float, float]):
        """
        Args:
            size: ``(sx, sy, sz)`` 三边长度（米）

        Raises:
            ValueError: 任一维非正
        """
        s = np.asarray(size, dtype=np.float64).ravel()
        if s.size != 3 or np.any(s <= 0.0):
            raise ValueError(f"box 尺寸必须为正的三元组，收到 {size!r}")
        self.size: np.ndarray = s
        self.half: np.ndarray = s / 2.0

    def support(self, direction: np.ndarray) -> np.ndarray:
        """沿方向取最远的角点；分量为 0 时取正侧（不影响结果）。"""
        d = np.asarray(direction, dtype=np.float64).ravel()
        return np.where(d >= 0.0, self.half, -self.half)

    def _make_vertices(self) -> np.ndarray:
        hx, hy, hz = self.half
        return np.array(
            [
                [-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
                [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz],
            ],
            dtype=np.float64,
        )

    def _make_triangles(self) -> np.ndarray:
        v = self.vertices
        idx = np.array(
            [
                [0, 2, 1], [0, 3, 2],  # -Z
                [4, 5, 6], [4, 6, 7],  # +Z
                [0, 1, 5], [0, 5, 4],  # -Y
                [3, 7, 6], [3, 6, 2],  # +Y
                [0, 4, 7], [0, 7, 3],  # -X
                [1, 2, 6], [1, 6, 5],  # +X
            ],
            dtype=np.int64,
        )
        return v[idx]

    def describe(self) -> str:
        return f"{self.size[0]:.3f}x{self.size[1]:.3f}x{self.size[2]:.3f}m"

    @property
    def params(self) -> Tuple[float, float, float]:
        """供 GL 快速绘制的 ``(w, h, d)``。"""
        return (float(self.size[0]), float(self.size[1]), float(self.size[2]))


class CylinderGeometry(ConvexGeometry):
    """圆柱，轴沿局部 +Z，中心位于局部原点。"""

    kind = "cylinder"

    def __init__(self, radius: float, length: float, segments: int = 24):
        """
        Args:
            radius: 半径（米）
            length: 轴向长度（米）
            segments: 侧面离散段数（仅影响渲染与包围盒，不影响碰撞精度）

        Raises:
            ValueError: 半径或长度非正
        """
        if radius <= 0.0 or length <= 0.0:
            raise ValueError(f"圆柱半径与长度必须为正，收到 r={radius}, l={length}")
        self.radius = float(radius)
        self.length = float(length)
        self.half_length = float(length) / 2.0
        self.segments = int(segments)

    def support(self, direction: np.ndarray) -> np.ndarray:
        """解析支撑点。

        ``max over (r·cosθ, r·sinθ, z)`` 的解为：径向取单位方向、轴向取符号，
        故支撑点恒落在上下端面的圆周上（|dz|→0 时退化为侧棱）。
        """
        d = np.asarray(direction, dtype=np.float64).ravel()
        norm = np.linalg.norm(d)
        if norm < _EPS:
            return np.array([self.radius, 0.0, self.half_length])
        d = d / norm
        radial = math.hypot(d[0], d[1])
        if radial < _EPS:
            # 方向沿 ±Z：整个端面圆都是支撑集，任取一点即可
            return np.array([self.radius, 0.0, math.copysign(self.half_length, d[2])])
        return np.array(
            [
                self.radius * d[0] / radial,
                self.radius * d[1] / radial,
                math.copysign(self.half_length, d[2]),
            ]
        )

    def _make_vertices(self) -> np.ndarray:
        n = self.segments
        theta = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
        ring = np.stack([np.cos(theta), np.sin(theta)], axis=1) * self.radius
        bottom = np.column_stack([ring, np.full(n, -self.half_length)])
        top = np.column_stack([ring, np.full(n, self.half_length)])
        return np.vstack([bottom, top])

    def _make_triangles(self) -> np.ndarray:
        n = self.segments
        v = self.vertices
        center_b = np.array([0.0, 0.0, -self.half_length])
        center_t = np.array([0.0, 0.0, self.half_length])
        tris = []
        for i in range(n):
            j = (i + 1) % n
            b0, b1, t0, t1 = v[i], v[j], v[n + i], v[n + j]
            tris.append([b0, b1, t1])        # 侧面
            tris.append([b0, t1, t0])
            tris.append([center_t, t0, t1])  # 上盖
            tris.append([center_b, b1, b0])  # 下盖
        return np.array(tris, dtype=np.float64)

    def describe(self) -> str:
        return f"r={self.radius:.3f}m L={self.length:.3f}m"

    @property
    def params(self) -> Tuple[float, float]:
        """供 GL 快速绘制的 ``(radius, length)``。"""
        return (self.radius, self.length)


class SphereGeometry(ConvexGeometry):
    """球体，球心位于局部原点。"""

    kind = "sphere"

    def __init__(self, radius: float, segments: int = 16, rings: int = 10):
        """
        Args:
            radius: 半径（米）
            segments: 经线方向分段数
            rings: 纬线方向分层数（含极点）
        """
        if radius <= 0.0:
            raise ValueError(f"球半径必须为正，收到 {radius}")
        self.radius = float(radius)
        self.segments = int(segments)
        self.rings = int(rings)

    def support(self, direction: np.ndarray) -> np.ndarray:
        """球面支撑点即半径乘以单位方向（解析解，无离散误差）。"""
        d = np.asarray(direction, dtype=np.float64).ravel()
        norm = np.linalg.norm(d)
        if norm < _EPS:
            return np.array([0.0, 0.0, self.radius])
        return d * (self.radius / norm)

    def _make_vertices(self) -> np.ndarray:
        seg, ring = self.segments, self.rings
        verts = [np.array([0.0, 0.0, self.radius]), np.array([0.0, 0.0, -self.radius])]
        for i in range(1, ring):
            phi = math.pi * i / ring
            z = self.radius * math.cos(phi)
            r = self.radius * math.sin(phi)
            for k in range(seg):
                theta = 2.0 * math.pi * k / seg
                verts.append(np.array([r * math.cos(theta), r * math.sin(theta), z]))
        return np.array(verts, dtype=np.float64)

    def _make_triangles(self) -> np.ndarray:
        seg, ring = self.segments, self.rings
        v = self.vertices
        tris = []
        # 顶点 0 = 北极，1 = 南极，之后 (ring-1) 层 × seg 个
        base = 2
        for k in range(seg):
            nxt = (k + 1) % seg
            tris.append([v[0], v[base + k], v[base + nxt]])
        for i in range(ring - 2):
            row0 = base + i * seg
            row1 = base + (i + 1) * seg
            for k in range(seg):
                nxt = (k + 1) % seg
                a, b, c, d = v[row0 + k], v[row0 + nxt], v[row1 + nxt], v[row1 + k]
                tris.append([a, b, c])
                tris.append([a, c, d])
        last = base + (ring - 2) * seg
        for k in range(seg):
            nxt = (k + 1) % seg
            tris.append([v[1], v[last + nxt], v[last + k]])
        return np.array(tris, dtype=np.float64)

    def describe(self) -> str:
        return f"r={self.radius:.3f}m"

    @property
    def params(self) -> Tuple[float]:
        """供 GL 快速绘制的 ``(radius,)``。"""
        return (self.radius,)


class MeshGeometry(ConvexGeometry):
    """任意三角网格。

    碰撞计算只用到 :meth:`support`，而"在顶点集合上取最大投影"恰好等价于
    **该点集的凸包**的支撑函数。因此即使传入非凸网格，GJK 也会自动按凸包
    处理——无需额外的凸包计算，也不必引入 SciPy 依赖。

    代价：凹处（如料仓内腔）会被填平。需要凹腔时请把零件拆成多个凸体 part。
    """

    kind = "mesh"

    def __init__(self, vertices):
        """
        Args:
            vertices: 形状 (N, 3) 的顶点数组（米）

        Raises:
            ValueError: 顶点为空或形状不对
        """
        v = np.asarray(vertices, dtype=np.float64).reshape(-1, 3)
        if v.shape[0] == 0:
            raise ValueError("网格顶点不能为空")
        self._verts = np.ascontiguousarray(v)

    def support(self, direction: np.ndarray) -> np.ndarray:
        """在所有顶点中取投影最大者（等价于凸包支撑函数）。"""
        d = np.asarray(direction, dtype=np.float64).ravel()
        dots = self._verts @ d
        return self._verts[int(np.argmax(dots))]

    def _make_vertices(self) -> np.ndarray:
        return self._verts

    def _make_triangles(self) -> np.ndarray:
        # 无拓扑信息时无法重建面片；渲染层对 mesh 直接使用 vertices 的点云
        # 或调用方自带的三角面。返回空数组以保持接口一致。
        return np.zeros((0, 3, 3), dtype=np.float64)

    def describe(self) -> str:
        return f"{self._verts.shape[0]} verts"

    @property
    def params(self) -> tuple:
        """mesh 无 GL 快速路径，返回空元组。"""
        return ()
