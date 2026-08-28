"""碰撞检测内核：GJK 求最近距离 + EPA 求穿透深度。

为什么不用现成库：
    ``scipy`` 只有凸包算法没有 GJK；``python-fcl`` / ``trimesh`` 依赖 C++ 扩展
    与 VTK，会让零依赖的纯 Python 安装路径崩掉。GJK/EPA 本体不到 300 行，
    自己实现可以完全掌控退化情形（共线、共面、相切）的处理策略。

支撑函数约定：
    对 Minkowski 差 :math:`A \\ominus B`，支撑点为
    :math:`s_{A\\ominus B}(d) = s_A(d) - s_B(-d)`。
    本模块只通过支撑函数访问几何，因此任何实现了 :meth:`support` 的凸体
    （含未来的 STL 网格）都能直接接入。

结果语义：
    :attr:`Contact.normal` 恒为 **由 A 指向 B 的单位向量**。干涉时把 A 沿
    ``-normal * depth``（或把 B 沿 ``+normal * depth``）平移即可分离。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

from .geometry import ConvexGeometry

_TOL = 1e-9
_EPA_TOL = 1e-7

SupportFn = Callable[[np.ndarray], np.ndarray]


@dataclass
class Contact:
    """两个凸体的接触/干涉信息。

    Attributes:
        penetrating: 是否发生干涉（穿透）
        distance: 分离时的最近距离（米），干涉时为 ``0.0``
        depth: 干涉深度（米），分离时为 ``0.0``
        normal: 单位向量，**由 A 指向 B**
        point: 近似接触点（世界坐标，米），用于可视化标记
    """

    penetrating: bool
    distance: float
    depth: float
    normal: np.ndarray
    point: np.ndarray

    @property
    def signed_distance(self) -> float:
        """有符号距离：分离为正、干涉为负（米）。

        这是安全间距检查最好用的量——``signed_distance < margin`` 即报警。
        """
        return -self.depth if self.penetrating else self.distance

    def __repr__(self) -> str:
        state = f"干涉 {self.depth * 1000:.2f}mm" if self.penetrating else f"间距 {self.distance * 1000:.2f}mm"
        return f"Contact({state})"


# --------------------------------------------------------------------------- #
# 支撑函数构造
# --------------------------------------------------------------------------- #
def make_support(
    geom_a: ConvexGeometry, T_a: np.ndarray, geom_b: ConvexGeometry, T_b: np.ndarray
) -> SupportFn:
    """构造 Minkowski 差 :math:`A \\ominus B` 的世界坐标支撑函数。

    Args:
        geom_a: A 的几何（局部坐标）
        T_a: A 的世界变换 4x4（米）
        geom_b: B 的几何（局部坐标）
        T_b: B 的世界变换 4x4（米）

    Returns:
        接受世界方向向量、返回世界坐标支撑点差值的函数

    Note:
        方向向量需先旋回局部系（乘 :math:`R^\\top`）再取支撑点，结果再旋回世界系。
        因为 :math:`R` 是正交阵，:math:`R^{-1} = R^\\top`，省掉一次矩阵求逆。
    """
    Ra = np.asarray(T_a, dtype=np.float64)[:3, :3]
    ta = np.asarray(T_a, dtype=np.float64)[:3, 3]
    Rb = np.asarray(T_b, dtype=np.float64)[:3, :3]
    tb = np.asarray(T_b, dtype=np.float64)[:3, 3]
    Rai = Ra.T
    Rbi = Rb.T

    def support(direction: np.ndarray) -> np.ndarray:
        d = np.asarray(direction, dtype=np.float64).ravel()
        pa = Ra @ geom_a.support(Rai @ d) + ta
        pb = Rb @ geom_b.support(Rbi @ (-d)) + tb
        return pa - pb

    return support


def _initial_direction(T_a: np.ndarray, T_b: np.ndarray) -> np.ndarray:
    """初始搜索方向：A 中心指向 B 中心；重合时退化为 +X。"""
    d = np.asarray(T_b, dtype=np.float64)[:3, 3] - np.asarray(T_a, dtype=np.float64)[:3, 3]
    if np.linalg.norm(d) < _TOL:
        return np.array([1.0, 0.0, 0.0])
    return d


# --------------------------------------------------------------------------- #
# 单纯形到原点的最近点
# --------------------------------------------------------------------------- #
def _closest_on_segment(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """原点到线段 AB 的最近点。"""
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom < _TOL:
        return a.copy()
    t = float(np.dot(-a, ab) / denom)
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return a + t * ab


def _closest_on_triangle(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> np.ndarray:
    """原点到三角形 ABC 的最近点（Ericson《Real-Time Collision Detection》算法）。"""
    ab, ac = b - a, c - a
    ap = -a
    d1 = float(np.dot(ab, ap))
    d2 = float(np.dot(ac, ap))
    if d1 <= 0.0 and d2 <= 0.0:
        return a.copy()

    bp = -b
    d3 = float(np.dot(ab, bp))
    d4 = float(np.dot(ac, bp))
    if d3 >= 0.0 and d4 <= d3:
        return b.copy()

    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        v = d1 / (d1 - d3) if (d1 - d3) != 0.0 else 0.0
        return a + v * ab

    cp = -c
    d5 = float(np.dot(ab, cp))
    d6 = float(np.dot(ac, cp))
    if d6 >= 0.0 and d5 <= d6:
        return c.copy()

    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        w = d2 / (d2 - d6) if (d2 - d6) != 0.0 else 0.0
        return a + w * ac

    va = d3 * d6 - d5 * d4
    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        denom = (d4 - d3) + (d5 - d6)
        w = (d4 - d3) / denom if denom != 0.0 else 0.0
        return b + w * (c - b)

    denom = 1.0 / (va + vb + vc)
    v = vb * denom
    w = vc * denom
    return a + ab * v + ac * w


def _closest_to_origin(simplex: Sequence[np.ndarray]) -> np.ndarray:
    """原点到单纯形（1~4 个点）的最近点。"""
    n = len(simplex)
    if n == 1:
        return simplex[0].copy()
    if n == 2:
        return _closest_on_segment(simplex[0], simplex[1])
    if n == 3:
        return _closest_on_triangle(simplex[0], simplex[1], simplex[2])
    # 4 点：取四个三角面上最近点的更近者（四面体退化时的兜底）
    a, b, c, d = simplex
    best = None
    best_d2 = math.inf
    for tri in ((a, b, c), (a, b, d), (a, c, d), (b, c, d)):
        p = _closest_on_triangle(*tri)
        d2 = float(np.dot(p, p))
        if d2 < best_d2:
            best_d2, best = d2, p
    return best


# --------------------------------------------------------------------------- #
# 单纯形演化
# --------------------------------------------------------------------------- #
def _do_line(a: np.ndarray, b: np.ndarray) -> Tuple[bool, List[np.ndarray], Optional[np.ndarray]]:
    """线段情形：判断原点在 AB 段的哪一侧。

    Args:
        a: 最新加入的点
        b: 另一点
    """
    ab = b - a
    ao = -a
    if float(np.dot(ab, ao)) > 0.0:
        # 原点在 AB 段方向上 → 垂直 AB 指向原点
        d = np.cross(np.cross(ab, ao), ab)
        if np.linalg.norm(d) < _TOL:
            # a、b、原点共线且原点在 b 侧 —— 相切
            return True, [a, b], None
        return False, [a, b], d
    # 原点在 a 的背面 → 只保留 a
    return False, [a], ao


def _do_triangle(
    a: np.ndarray, b: np.ndarray, c: np.ndarray
) -> Tuple[bool, List[np.ndarray], Optional[np.ndarray]]:
    """三角形情形：判断原点在三条边的哪一侧，或需沿法线继续搜索。"""
    ab = b - a
    ac = c - a
    ao = -a

    # 垂直于边、且指向三角形外侧的向量
    ab_perp = np.cross(np.cross(ab, ac), ab)
    ac_perp = np.cross(np.cross(ac, ab), ac)

    if float(np.dot(ab_perp, ao)) > 0.0:
        if float(np.dot(ab, ao)) > 0.0:
            return _do_line(a, b)
        return _do_line(a, c)

    if float(np.dot(ac_perp, ao)) > 0.0:
        if float(np.dot(ac, ao)) > 0.0:
            return _do_line(a, c)
        return _do_line(a, b)

    # 投影落在三角形内部：3D 下原点可能在面的任一侧
    normal = np.cross(ab, ac)
    nn = np.linalg.norm(normal)
    if nn < _TOL:
        # 退化三角形（三点共线），按接触处理
        return True, [a, b, c], None
    normal = normal / nn
    proj = float(np.dot(normal, ao))
    if abs(proj) < _TOL:
        # 原点恰在三角形所在平面上 → 相切
        return True, [a, b, c], None
    if proj < 0.0:
        normal = -normal
    return False, [a, b, c], normal


def _do_tetrahedron(
    a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray
) -> Tuple[bool, List[np.ndarray], Optional[np.ndarray]]:
    """四面体情形：原点被包住即干涉，否则退化到朝向原点的那个面。

    四个面全部检查（而非常见的"只查含新点的三个面"）——多一次点积远比
    漏判一次干涉便宜，也省掉了"为什么第四个面不会漏"的心智负担。
    """
    verts = [a, b, c, d]
    for opp_i in range(4):
        tri = [verts[i] for i in range(4) if i != opp_i]
        opp = verts[opp_i]
        p1, p2, p3 = tri
        n = np.cross(p2 - p1, p3 - p1)
        if np.linalg.norm(n) < _TOL:
            continue  # 退化面（四点共面），跳过
        if float(np.dot(n, p1 - opp)) < 0.0:
            n = -n  # 强制为外法线（背离对角顶点）
        if float(np.dot(n, -p1)) > _TOL:
            # 原点在该面外侧 → 退化为三角形，沿外法线继续
            return False, [p1, p2, p3], n
    # 原点在所有面之内 → 干涉
    return True, verts, None


def _do_simplex(
    simplex: List[np.ndarray],
) -> Tuple[bool, List[np.ndarray], Optional[np.ndarray]]:
    """按当前单纯形规模分派；``simplex[-1]`` 为最新加入的点。"""
    n = len(simplex)
    if n == 2:
        return _do_line(simplex[-1], simplex[-2])
    if n == 3:
        return _do_triangle(simplex[-1], simplex[-2], simplex[-3])
    if n == 4:
        return _do_tetrahedron(simplex[-1], simplex[-2], simplex[-3], simplex[-4])
    raise AssertionError(f"单纯形规模非法：{n}")


# --------------------------------------------------------------------------- #
# GJK
# --------------------------------------------------------------------------- #
def gjk(
    support: SupportFn,
    initial_dir: Optional[np.ndarray] = None,
    max_iter: int = 64,
) -> Tuple[bool, List[np.ndarray], np.ndarray]:
    """Gilbert-Johnson-Keerthi 算法：判断原点是否在 :math:`A\\ominus B` 内。

    Args:
        support: Minkowski 差的支撑函数
        initial_dir: 初始搜索方向，缺省为 +X
        max_iter: 最大迭代次数

    Returns:
        ``(hit, simplex, closest)``
        ``hit`` 为真表示干涉；``closest`` 为单纯形上离原点最近的点，
        未干涉时其模长即最近距离。
    """
    d = np.array([1.0, 0.0, 0.0]) if initial_dir is None else np.asarray(initial_dir, float).ravel()
    if np.linalg.norm(d) < _TOL:
        d = np.array([1.0, 0.0, 0.0])

    simplex: List[np.ndarray] = [np.asarray(support(d), dtype=np.float64)]
    d = -simplex[0]

    for _ in range(max_iter):
        if np.linalg.norm(d) < _TOL:
            # 原点已落在单纯形上 —— 相切
            return True, simplex, np.zeros(3)

        p = np.asarray(support(d), dtype=np.float64)
        if float(np.dot(p, d)) < -_TOL:
            # 沿 d 无法再朝原点推进 → 分离
            return False, simplex, _closest_to_origin(simplex)

        simplex.append(p)
        hit, simplex, d = _do_simplex(simplex)
        if hit:
            return True, simplex, np.zeros(3)

    # 迭代耗尽：按分离处理，返回当前最优估计
    return False, simplex, _closest_to_origin(simplex)


# --------------------------------------------------------------------------- #
# EPA
# --------------------------------------------------------------------------- #
def _make_face(
    pts: Sequence[np.ndarray], i: int, j: int, k: int
) -> Optional[Tuple[int, int, int, np.ndarray, float]]:
    """构造一个面，并保证法线背离原点（原点在凸包内部）。

    Returns:
        ``(i, j, k, normal, dist)``；退化面（三点共线）返回 ``None``
    """
    n = np.cross(pts[j] - pts[i], pts[k] - pts[i])
    ln = np.linalg.norm(n)
    if ln < _TOL:
        return None
    n = n / ln
    dist = float(np.dot(n, pts[i]))
    if dist < 0.0:
        i, k = k, i  # 反转绕向
        n = -n
        dist = -dist
    return (i, j, k, n, dist)


def epa(
    support: SupportFn,
    simplex: Sequence[np.ndarray],
    max_iter: int = 64,
    tol: float = _EPA_TOL,
) -> Tuple[float, np.ndarray]:
    """Expanding Polytope Algorithm：求干涉深度与分离方向。

    Args:
        support: Minkowski 差的支撑函数
        simplex: GJK 得到的、包含原点的四面体（4 个点）
        max_iter: 最大迭代次数
        tol: 收敛阈值（米）

    Returns:
        ``(depth, normal)``，``normal`` 由 A 指向 B。
        单纯形退化或迭代耗尽时返回 ``(0.0, 默认方向)``——布尔意义上的
        "干涉"仍然成立，只是深度不精确。
    """
    pts: List[np.ndarray] = [np.asarray(p, dtype=np.float64) for p in simplex]
    if len(pts) < 4:
        # 相切/退化：无体积可扩张，深度为 0
        c = _closest_to_origin(pts)
        fallback = c / np.linalg.norm(c) if np.linalg.norm(c) > _TOL else np.array([0.0, 0.0, 1.0])
        return 0.0, fallback

    faces = []
    for tri in ((0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3)):
        f = _make_face(pts, *tri)
        if f is not None:
            faces.append(f)
    if not faces:
        return 0.0, np.array([0.0, 0.0, 1.0])

    for _ in range(max_iter):
        m = min(range(len(faces)), key=lambda idx: faces[idx][4])
        fi, fj, fk, normal, dist = faces[m]

        p = np.asarray(support(normal), dtype=np.float64)
        pd = float(np.dot(normal, p))
        if pd - dist <= tol:
            # 无法再向外扩张 → 收敛
            return dist, normal

        # 找出从 p 可见的面（法线朝向 p 的一侧）
        visible = set()
        for idx, (i, j, k, n, _) in enumerate(faces):
            if float(np.dot(n, p - pts[i])) > 0.0:
                visible.add(idx)

        if not visible:
            return dist, normal

        # 轮廓边：被两个可见面共享的内部边成对抵消，剩下的反向不再出现的边
        edges: dict = {}
        for idx in visible:
            i, j, k = faces[idx][0], faces[idx][1], faces[idx][2]
            for a, b in ((i, j), (j, k), (k, i)):
                if (b, a) in edges:
                    del edges[(b, a)]  # 内部边，抵消
                else:
                    edges[(a, b)] = idx

        new_idx = len(pts)
        pts.append(p)
        faces = [f for idx, f in enumerate(faces) if idx not in visible]
        for (a, b) in edges:
            nf = _make_face(pts, a, b, new_idx)
            if nf is not None:
                faces.append(nf)

        if not faces:
            return dist, normal

    # 迭代耗尽：返回当前最近面的结果
    m = min(range(len(faces)), key=lambda idx: faces[idx][4])
    _, _, _, normal, dist = faces[m]
    return dist, normal


# --------------------------------------------------------------------------- #
# 对外主入口
# --------------------------------------------------------------------------- #
def _contact_point(
    geom_a: ConvexGeometry,
    T_a: np.ndarray,
    geom_b: ConvexGeometry,
    T_b: np.ndarray,
    normal: np.ndarray,
) -> np.ndarray:
    """估算接触点：A 沿 ``+normal`` 的极点与 B 沿 ``-normal`` 的极点的中点。

    浅穿透下这个估计足够用于可视化标记；不是严格的接触流形。
    """
    Ra, ta = T_a[:3, :3], T_a[:3, 3]
    Rb, tb = T_b[:3, :3], T_b[:3, 3]
    pa = Ra @ geom_a.support(Ra.T @ normal) + ta
    pb = Rb @ geom_b.support(Rb.T @ (-normal)) + tb
    return (pa + pb) / 2.0


def collide(
    geom_a: ConvexGeometry,
    T_a: np.ndarray,
    geom_b: ConvexGeometry,
    T_b: np.ndarray,
    max_iter: int = 64,
) -> Contact:
    """计算两个凸体的接触状态。

    Args:
        geom_a: A 的几何（局部坐标，米）
        T_a: A 的世界变换 4x4（米）
        geom_b: B 的几何（局部坐标，米）
        T_b: B 的世界变换 4x4（米）
        max_iter: GJK/EPA 迭代上限

    Returns:
        :class:`Contact`，含是否干涉、距离/深度、法线与接触点。
        :attr:`Contact.normal` 为由 A 指向 B 的单位向量。
    """
    T_a = np.asarray(T_a, dtype=np.float64)
    T_b = np.asarray(T_b, dtype=np.float64)
    support = make_support(geom_a, T_a, geom_b, T_b)

    hit, simplex, closest = gjk(support, _initial_direction(T_a, T_b), max_iter)

    if not hit:
        dist = float(np.linalg.norm(closest))
        if dist < _TOL:
            normal = np.array([0.0, 0.0, 1.0])
            point = (T_a[:3, 3] + T_b[:3, 3]) / 2.0
            return Contact(True, 0.0, 0.0, normal, point)
        # closest = pA - pB，指向 B→A，故 A→B 取反
        normal = -closest / dist
        return Contact(False, dist, 0.0, normal, _contact_point(geom_a, T_a, geom_b, T_b, normal))

    depth, normal = epa(support, simplex, max_iter)
    depth = max(float(depth), 0.0)
    nn = np.linalg.norm(normal)
    normal = normal / nn if nn > _TOL else np.array([0.0, 0.0, 1.0])
    return Contact(True, 0.0, depth, normal, _contact_point(geom_a, T_a, geom_b, T_b, normal))


def aabb_overlap(
    lower_a: np.ndarray, upper_a: np.ndarray, lower_b: np.ndarray, upper_b: np.ndarray, margin: float = 0.0
) -> bool:
    """带安全余量的轴对齐包围盒相交测试（宽相位）。

    Args:
        lower_a / upper_a: A 的包围盒
        lower_b / upper_b: B 的包围盒
        margin: 额外膨胀量（米），用于"安全间距"预警

    Returns:
        两个盒子（各向外扩 ``margin``）是否相交
    """
    return bool(
        np.all(lower_a - margin <= upper_b + margin) and np.all(lower_b - margin <= upper_a + margin)
    )


def sphere_reject(T_a: np.ndarray, r_a: float, T_b: np.ndarray, r_b: float, margin: float = 0.0) -> bool:
    """外接球快速剔除：中心距超过半径和则必然分离。

    Returns:
        ``True`` 表示**确定分离**，可跳过精确检测
    """
    d = np.linalg.norm(T_b[:3, 3] - T_a[:3, 3])
    return bool(d > r_a + r_b + margin)
