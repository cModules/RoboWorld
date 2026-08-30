#!/usr/bin/env python3
"""程序化曲面建模原语。

为什么需要这个模块
-------------------

URDF 只有 ``box`` / ``cylinder`` / ``sphere`` 三种基本体。用它们堆机器人，
无论堆多少个，结果都是"积木风"——因为真实工业机器人的造型特征恰恰是
基本体表达不出来的：

* **变截面**：大臂从肩部到肘部逐渐收窄，不是等径圆柱
* **铸造圆角**：所有棱边都是大圆角，没有一条锐边
* **非圆截面**：臂体截面是圆角矩形或扁圆，不是正圆
* **空间走向**：线束、管路沿曲面走

本模块把这些特征做成可参数化的网格生成器，产出带平滑法线的三角网格。
与基本体相比：

* 造型自由（放样体能表达任意变截面）
* 圆角、锥度等细节一次生成，不必用几十个基本体去逼近
* 碰撞体仍用粗基本体，SAT 开销不随外观精度增长

所有原语只依赖 NumPy，可在无 OpenGL 环境下单元测试。

坐标系约定：与 URDF 一致，长度单位为米，圆柱/放样体沿局部 **+Z** 轴。
"""

from __future__ import annotations

import math
from dataclasses import dataclass  # field 留作未来扩展（isort/flake8: noqa: F401）
from typing import List, Sequence, Tuple

import numpy as np

from .mesh import MeshData, compute_normals

__all__ = [
    "Section",
    "loft",
    "rounded_box",
    "revolve",
    "tapered_cylinder",
    "capsule",
    "tube",
    "write_stl",
    "merge",
]


# --------------------------------------------------------------------------- #
# 内部工具
# --------------------------------------------------------------------------- #
def _assemble(
    vertices: np.ndarray,
    faces: np.ndarray,
    source: str = "",
    smooth_angle_deg: float = 50.0,
) -> MeshData:
    """把 ``(V,3)`` 顶点 + ``(M,3)`` 面索引组装成 :class:`MeshData`。

    Args:
        vertices: 顶点坐标，形状 ``(V, 3)``。
        faces: 三角形索引，形状 ``(M, 3)``。
        source: 来源标识，用于诊断与显示列表缓存键。
        smooth_angle_deg: 相邻面夹角超过此值则保留硬边（不平滑）。

    Returns:
        带平滑法线的网格数据。
    """
    if faces.size == 0:
        empty = np.zeros((0, 3), dtype=np.float32)
        return MeshData(empty, empty.copy(), 0, source)

    corner_v, corner_n, _face_n = compute_normals(
        np.ascontiguousarray(vertices, dtype=np.float64),
        np.ascontiguousarray(faces, dtype=np.int64),
        smooth_angle_deg,
    )
    return MeshData(
        vertices=np.ascontiguousarray(corner_v, dtype=np.float32),
        normals=np.ascontiguousarray(corner_n, dtype=np.float32),
        triangles=int(faces.shape[0]),
        source=source,
    )


def _superellipse(theta: np.ndarray, hx: float, hy: float, n: float) -> Tuple[np.ndarray, np.ndarray]:
    """超椭圆截面：``n=2`` 为椭圆，``n`` 越大越接近矩形。

    ``n=4`` 左右就是工业臂体那种"圆角矩形"观感——既有平面感，又没有锐边。

    Args:
        theta: 参数角，形状 ``(K,)``。
        hx: X 方向半尺寸。
        hy: Y 方向半尺寸。
        n: 超椭圆指数，必须 > 0。

    Returns:
        ``(x, y)`` 两个形状 ``(K,)`` 的数组。
    """
    ct, st = np.cos(theta), np.sin(theta)
    e = 2.0 / max(1e-6, n)
    x = hx * np.sign(ct) * np.abs(ct) ** e
    y = hy * np.sign(st) * np.abs(st) ** e
    return x, y


def _arc_dense_samples(half: float, radius: float, count: int) -> np.ndarray:
    """生成沿单轴、在圆角处加密的采样坐标。

    圆角箱体的表面由平面段和两个圆角段拼成。若均匀采样，圆角那点弧长
    只能分到很少的点，成品就成了"倒角"而不是"圆角"。这里按**弧长比例**
    分配采样点：平面段 ``2·a``，每端圆角 ``(π/2)·r``。

    Args:
        half: 该轴的半尺寸。
        radius: 圆角半径。
        count: 采样点总数（>= 2）。

    Returns:
        升序排列的坐标数组，形状 ``(count,)``，范围 ``[-half, half]``。
    """
    r = min(radius, half)
    if r <= 1e-9 or count < 5:
        return np.linspace(-half, half, max(2, count))

    a = half - r                       # 内缩后的核心半尺寸
    arc = 0.5 * math.pi * r            # 单端圆角弧长
    flat = 2.0 * a                     # 平面段长度
    total = flat + 2.0 * arc
    n_arc = max(2, int(round(count * arc / total)))
    n_flat = max(2, count - 2 * n_arc)

    # 低端圆角：从 -half 收到 -a，按角度均匀（即弧长均匀）
    phi = np.linspace(0.0, 0.5 * math.pi, n_arc, endpoint=False)
    low = -a - r * np.sin(phi)[::-1]          # -half → -a（升序）
    # 平面段
    mid = np.linspace(-a, a, n_flat, endpoint=False)
    # 高端圆角：从 a 到 half
    high = a + r * np.sin(phi)
    return np.concatenate([low, mid, high, [half]])


# --------------------------------------------------------------------------- #
# 放样：变截面体的主力原语
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Section:
    """放样的一个截面。

    Attributes:
        z: 截面所在高度（米）。
        hx: X 方向半尺寸（米）。
        hy: Y 方向半尺寸（米）。
        squareness: 截面形状，``2`` 为椭圆，``4~5`` 为圆角矩形，越大越方。
        offset: 截面中心相对放样轴的 ``(dx, dy)`` 偏移，用于表达弯曲的臂体。
        twist: 截面绕 Z 轴的扭转角（弧度），用于表达螺旋走向的造型。
    """

    z: float
    hx: float
    hy: float
    squareness: float = 4.0
    offset: Tuple[float, float] = (0.0, 0.0)
    twist: float = 0.0


def loft(
    sections: Sequence[Section],
    segments: int = 40,
    cap_bottom: bool = True,
    cap_top: bool = True,
    source: str = "",
) -> MeshData:
    """沿局部 +Z 轴把若干截面放样成一个变截面体。

    这是本模块最重要的原语：工业机器人的大臂、小臂、底座、关节壳体本质上
    都是"截面沿轴变化"的铸造件，一个 ``loft`` 就能表达，而用基本体逼近
    需要十几个圆柱/箱体还做不出锥度和圆角。

    Args:
        sections: 截面列表，按 ``z`` 升序；至少 2 个。
        segments: 截面周向分段数。圆角矩形建议 32~48，椭圆 24 即可。
        cap_bottom: 是否封底。
        cap_top: 是否封顶。
        source: 来源标识。

    Returns:
        放样得到的网格。

    Raises:
        ValueError: 截面少于 2 个。
    """
    if len(sections) < 2:
        raise ValueError(f"放样至少需要 2 个截面，收到 {len(sections)} 个")

    theta = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
    rings: List[np.ndarray] = []
    for sec in sections:
        x, y = _superellipse(theta, sec.hx, sec.hy, sec.squareness)
        if sec.twist:
            c, s = math.cos(sec.twist), math.sin(sec.twist)
            x, y = c * x - s * y, s * x + c * y
        ring = np.empty((segments, 3), dtype=np.float64)
        ring[:, 0] = x + sec.offset[0]
        ring[:, 1] = y + sec.offset[1]
        ring[:, 2] = sec.z
        rings.append(ring)

    verts: List[np.ndarray] = []
    faces: List[np.ndarray] = []
    base = 0

    # ---- 侧壁 ---- #
    # 共享环顶点（不复制），焊接后能得到沿轴向连续的平滑法线
    side_verts = np.concatenate(rings, axis=0)      # (S*K, 3)
    verts.append(side_verts)
    n_rings = len(rings)
    for i in range(n_rings - 1):
        a0 = base + i * segments
        b0 = a0 + segments
        i0 = np.arange(segments)
        i1 = (i0 + 1) % segments
        # 两个三角形组成一段四边形，绕向保证法线朝外
        f1 = np.stack([a0 + i0, b0 + i0, b0 + i1], axis=1)
        f2 = np.stack([a0 + i0, b0 + i1, a0 + i1], axis=1)
        faces.append(f1)
        faces.append(f2)
    base += side_verts.shape[0]

    # ---- 端盖 ---- #
    # 端盖顶点必须独立复制：与侧壁共享会让焊缝处的法线被平均成斜的，
    # 棱边就"塌"了。compute_normals 的夹角保护会兜底，但独立顶点更干净。
    def _cap(ring: np.ndarray, flip: bool) -> None:
        nonlocal base
        center = ring.mean(axis=0)
        v = np.vstack([ring, center[None, :]])
        idx = base
        verts.append(v)
        i0 = np.arange(segments)
        i1 = (i0 + 1) % segments
        if flip:
            faces.append(np.stack([np.full(segments, idx + segments), idx + i1, idx + i0], axis=1))
        else:
            faces.append(np.stack([np.full(segments, idx + segments), idx + i0, idx + i1], axis=1))
        base += v.shape[0]

    if cap_bottom:
        _cap(rings[0], flip=True)
    if cap_top:
        _cap(rings[-1], flip=False)

    return _assemble(np.concatenate(verts, axis=0), np.concatenate(faces, axis=0), source)


# --------------------------------------------------------------------------- #
# 圆角箱体：Minkowski 和（box ⊕ sphere）
# --------------------------------------------------------------------------- #
def rounded_box(
    sx: float,
    sy: float,
    sz: float,
    radius: float,
    quality: int = 8,
    source: str = "",
) -> MeshData:
    """带圆角的箱体（箱体与球的 Minkowski 和）。

    圆角是"铸造感"的来源。锐边箱体在光照下会出现一条突兀的明暗分界，
    一眼假；圆角把这条分界摊成一段渐变的圆弧，观感立刻不同。

    实现：对立方体表面点 ``p``，取它在内缩核心箱体上的最近点 ``c``，
    再沿 ``p - c`` 方向外推 ``radius``。角落自然得到球面，棱边得到圆柱面，
    面心保持不变——一个公式覆盖全部三种情况。

    Args:
        sx: X 方向尺寸（米）。
        sy: Y 方向尺寸（米）。
        sz: Z 方向尺寸（米）。
        radius: 圆角半径（米）；自动截断到不超过最小半尺寸。
        quality: 每个面每边的采样密度，越大圆角越平滑。
        source: 来源标识。

    Returns:
        圆角箱体网格。
    """
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    r = min(radius, hx, hy, hz)
    ax, ay, az = hx - r, hy - r, hz - r

    # 每个面在圆角处加密采样，否则圆角只剩两三个点，退化成倒角
    xs = _arc_dense_samples(hx, r, quality)
    ys = _arc_dense_samples(hy, r, quality)
    zs = _arc_dense_samples(hz, r, quality)

    verts: List[np.ndarray] = []
    faces: List[np.ndarray] = []
    base = 0

    def _face(fixed_axis: int, fixed_sign: float, u_axis: int, v_axis: int,
              u_vals: np.ndarray, v_vals: np.ndarray) -> None:
        nonlocal base
        uu, vv = np.meshgrid(u_vals, v_vals, indexing="ij")
        half = (hx, hy, hz)[fixed_axis]

        p = np.empty((uu.size, 3), dtype=np.float64)
        p[:, fixed_axis] = fixed_sign * half
        p[:, u_axis] = uu.ravel()
        p[:, v_axis] = vv.ravel()

        # clamp 到核心箱体 → 最近点 c；方向 n = p - c；表面 = c + r·normalize(n)
        core = p.copy()
        core[:, 0] = np.clip(core[:, 0], -ax, ax)
        core[:, 1] = np.clip(core[:, 1], -ay, ay)
        core[:, 2] = np.clip(core[:, 2], -az, az)
        n = p - core
        length = np.linalg.norm(n, axis=1, keepdims=True)
        length = np.where(length < 1e-12, 1.0, length)
        surf = core + r * (n / length)

        nu, nv = len(u_vals), len(v_vals)
        idx = np.arange(nu * nv).reshape(nu, nv) + base
        # 绕向随面的朝向修正，保证法线朝外
        if fixed_sign > 0:
            f1 = np.stack([idx[:-1, :-1], idx[1:, :-1], idx[1:, 1:]], axis=-1)
            f2 = np.stack([idx[:-1, :-1], idx[1:, 1:], idx[:-1, 1:]], axis=-1)
        else:
            f1 = np.stack([idx[:-1, :-1], idx[1:, 1:], idx[1:, :-1]], axis=-1)
            f2 = np.stack([idx[:-1, :-1], idx[:-1, 1:], idx[1:, 1:]], axis=-1)
        faces.append(f1.reshape(-1, 3))
        faces.append(f2.reshape(-1, 3))
        verts.append(surf)
        base += surf.shape[0]

    _face(0, +1.0, 1, 2, ys, zs)   # +X
    _face(0, -1.0, 1, 2, ys, zs)   # -X
    _face(1, +1.0, 2, 0, zs, xs)   # +Y
    _face(1, -1.0, 2, 0, zs, xs)   # -Y
    _face(2, +1.0, 0, 1, xs, ys)   # +Z
    _face(2, -1.0, 0, 1, xs, ys)   # -Z

    return _assemble(np.concatenate(verts, axis=0), np.concatenate(faces, axis=0), source)


# --------------------------------------------------------------------------- #
# 旋转体 / 锥台 / 胶囊 / 管路
# --------------------------------------------------------------------------- #
def revolve(
    profile: Sequence[Tuple[float, float]],
    segments: int = 40,
    cap: bool = True,
    source: str = "",
) -> MeshData:
    """把 ``(radius, z)`` 轮廓绕 Z 轴旋转成回转体。

    用来做底座法兰、关节端盖、电机壳这类"带台阶的圆柱"——一个轮廓就能
    表达锥度、倒角、退刀槽，基本体则要用好几个圆柱去拼还留缝。

    Args:
        profile: ``(radius, z)`` 序列，按 z 升序。
        segments: 周向分段数。
        cap: 是否封端（轮廓首尾半径大于 0 时需要）。
        source: 来源标识。

    Returns:
        回转体网格。

    Raises:
        ValueError: 轮廓少于 2 个点。
    """
    if len(profile) < 2:
        raise ValueError(f"旋转轮廓至少需要 2 个点，收到 {len(profile)} 个")

    prof = np.asarray(profile, dtype=np.float64)
    theta = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
    ct, st = np.cos(theta), np.sin(theta)

    # (P, K) 网格：行=轮廓点，列=周向
    r = prof[:, 0:1]
    z = prof[:, 1:2]
    x = r * ct[None, :]
    y = r * st[None, :]
    grid = np.stack([np.broadcast_to(x, (len(prof), segments)),
                     np.broadcast_to(y, (len(prof), segments)),
                     np.broadcast_to(z, (len(prof), segments))], axis=-1)

    verts = [grid.reshape(-1, 3)]
    faces: List[np.ndarray] = []
    k = np.arange(segments)
    k1 = (k + 1) % segments
    for i in range(len(prof) - 1):
        a = i * segments
        b = (i + 1) * segments
        faces.append(np.stack([a + k, b + k, b + k1], axis=1))
        faces.append(np.stack([a + k, b + k1, a + k1], axis=1))
    base = verts[0].shape[0]

    if cap:
        for row, flip in ((0, True), (len(prof) - 1, False)):
            if prof[row, 0] <= 1e-9:
                continue
            ring = grid[row]
            center = ring.mean(axis=0)
            verts.append(np.vstack([ring, center[None, :]]))
            c = base + segments
            if flip:
                faces.append(np.stack([np.full(segments, c), base + k1, base + k], axis=1))
            else:
                faces.append(np.stack([np.full(segments, c), base + k, base + k1], axis=1))
            base += segments + 1

    return _assemble(np.concatenate(verts, axis=0), np.concatenate(faces, axis=0), source)


def tapered_cylinder(
    radius_bottom: float,
    radius_top: float,
    length: float,
    segments: int = 40,
    source: str = "",
) -> MeshData:
    """锥台（截头圆锥），沿局部 +Z 轴，中心在原点。

    臂体从关节到关节是收窄的——等径圆柱做不出这种"铸造锥度"。

    Args:
        radius_bottom: 底面半径（米）。
        radius_top: 顶面半径（米）。
        length: 长度（米）。
        segments: 周向分段数。
        source: 来源标识。

    Returns:
        锥台网格。
    """
    half = length / 2.0
    return revolve(
        [(radius_bottom, -half), (radius_top, +half)],
        segments=segments,
        cap=True,
        source=source,
    )


def capsule(radius: float, length: float, segments: int = 28, source: str = "") -> MeshData:
    """胶囊体（圆柱 + 两端半球），沿局部 +Z 轴。

    用于线缆、连杆这类"圆头圆脑"的部件，比圆柱多一份有机感。

    Args:
        radius: 半径（米）。
        length: 圆柱段长度（不含两端半球，米）。
        segments: 周向分段数。
        source: 来源标识。

    Returns:
        胶囊体网格。
    """
    half = max(0.0, length) / 2.0
    arc = max(3, segments // 4)
    profile: List[Tuple[float, float]] = []
    # 下半球
    for i in range(arc + 1):
        phi = -0.5 * math.pi + 0.5 * math.pi * i / arc
        profile.append((radius * math.cos(phi), -half + radius * math.sin(phi)))
    # 上半球
    for i in range(arc + 1):
        phi = 0.5 * math.pi * i / arc
        profile.append((radius * math.cos(phi), half + radius * math.sin(phi)))
    return revolve(profile, segments=segments, cap=False, source=source)


def tube(
    points: Sequence[Sequence[float]],
    radius: float,
    segments: int = 12,
    closed: bool = False,
    source: str = "",
) -> MeshData:
    """沿空间折线扫掠出等径管路，用于线束、气管、线缆包。

    线缆是"真实感"的廉价高分项：一台真机上到处是外露的线缆和气管，
    没有它们的机器人模型一眼就是渲染图。

    Args:
        points: 路径点序列，形状 ``(N, 3)``，至少 2 个。
        radius: 管半径（米）。
        segments: 周向分段数（管路细，8~16 足够）。
        closed: 是否首尾相连。
        source: 来源标识。

    Returns:
        管路网格。

    Raises:
        ValueError: 路径点少于 2 个。
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.shape[0] < 2:
        raise ValueError(f"管路至少需要 2 个路径点，收到 {pts.shape[0]} 个")

    # 沿路径的切线与两个法向（用平行传输避免管子扭转跳变）
    tangents = np.diff(pts, axis=0)
    norms = np.linalg.norm(tangents, axis=1, keepdims=True)
    norms = np.where(norms < 1e-12, 1.0, norms)
    tangents = tangents / norms

    ref = np.array([0.0, 0.0, 1.0])
    if abs(float(tangents[0] @ ref)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    n1 = np.cross(tangents[0], ref)
    n1 /= np.linalg.norm(n1)
    n2 = np.cross(tangents[0], n1)

    normals1 = [n1]
    normals2 = [n2]
    for i in range(1, len(tangents)):
        # 平行传输：把上一组法向投影到当前切线的垂直平面
        v1 = normals1[-1] - tangents[i] * float(normals1[-1] @ tangents[i])
        if np.linalg.norm(v1) < 1e-9:
            v1 = np.cross(tangents[i], ref)
        v1 /= np.linalg.norm(v1)
        normals1.append(v1)
        normals2.append(np.cross(tangents[i], v1))

    normals1 = np.array(normals1)
    normals2 = np.array(normals2)
    theta = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
    ct, st = np.cos(theta), np.sin(theta)

    rings = []
    for i in range(pts.shape[0]):
        # 端点复用相邻切线，保证端面垂直
        ti = min(i, len(tangents) - 1)
        a, b = normals1[ti], normals2[ti]
        ring = pts[i][None, :] + radius * (ct[:, None] * a[None, :] + st[:, None] * b[None, :])
        rings.append(ring)

    verts = [np.concatenate(rings, axis=0)]
    faces: List[np.ndarray] = []
    k = np.arange(segments)
    k1 = (k + 1) % segments
    n_pts = pts.shape[0]
    last = n_pts if closed else n_pts - 1
    for i in range(last):
        j = (i + 1) % n_pts
        a0, b0 = i * segments, j * segments
        faces.append(np.stack([a0 + k, b0 + k, b0 + k1], axis=1))
        faces.append(np.stack([a0 + k, b0 + k1, a0 + k1], axis=1))
    base = verts[0].shape[0]

    if not closed:
        for ring, flip in ((rings[0], True), (rings[-1], False)):
            center = ring.mean(axis=0)
            verts.append(np.vstack([ring, center[None, :]]))
            c = base + segments
            if flip:
                faces.append(np.stack([np.full(segments, c), base + k1, base + k], axis=1))
            else:
                faces.append(np.stack([np.full(segments, c), base + k, base + k1], axis=1))
            base += segments + 1

    return _assemble(np.concatenate(verts, axis=0), np.concatenate(faces, axis=0), source)


# --------------------------------------------------------------------------- #
# 组合与导出
# --------------------------------------------------------------------------- #
def merge(meshes: Sequence[MeshData], source: str = "") -> MeshData:
    """把多个网格合并成一个（仅拼接，不做布尔运算）。

    用于把同一材质、同一连杆的多个部件合成单个 STL，减少绘制调用与
    文件数量。材质不同的部件**不要**合并，否则会丢失配色层次——分层配色
    本身就是精细感的重要来源。

    Args:
        meshes: 待合并的网格。
        source: 合并后的来源标识。

    Returns:
        合并后的网格。
    """
    parts = [m for m in meshes if not m.is_empty]
    if not parts:
        empty = np.zeros((0, 3), dtype=np.float32)
        return MeshData(empty, empty.copy(), 0, source)
    if len(parts) == 1:
        return MeshData(parts[0].vertices, parts[0].normals, parts[0].triangles, source or parts[0].source)
    return MeshData(
        vertices=np.concatenate([m.vertices for m in parts], axis=0),
        normals=np.concatenate([m.normals for m in parts], axis=0),
        triangles=sum(m.triangles for m in parts),
        source=source,
    )


def write_stl(mesh: MeshData, path) -> int:
    """把网格写成**二进制** STL。

    二进制 STL 每三角形固定 50 字节，比 ASCII 小约 5 倍，且解析时不需要
    逐行正则——17 万三角形的模型从 1.2 秒降到 0.1 秒。

    Args:
        mesh: 待写入的网格。
        path: 输出路径（``str`` 或 :class:`~pathlib.Path`）。

    Returns:
        写入的三角形数量。
    """
    from pathlib import Path

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    v = np.ascontiguousarray(mesh.vertices, dtype=np.float32).reshape(-1, 3, 3)
    n = np.ascontiguousarray(mesh.normals, dtype=np.float32).reshape(-1, 3, 3)
    # 面法线取三顶点法线均值并归一化；退化面回退到叉积
    face_n = n.sum(axis=1)
    cross = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    length = np.linalg.norm(face_n, axis=1, keepdims=True)
    face_n = np.where(length < 1e-12, cross, face_n)
    length = np.linalg.norm(face_n, axis=1, keepdims=True)
    face_n = np.where(length < 1e-12, 1.0, length)  # 防除零
    face_n = face_n / np.where(length < 1e-12, 1.0, length)

    dtype = np.dtype([
        ("n", "<f4", 3),
        ("v", "<f4", (3, 3)),
        ("attr", "<u2"),
    ])
    body = np.zeros(v.shape[0], dtype=dtype)
    body["n"] = face_n
    body["v"] = v
    body["attr"] = 0

    with open(path, "wb") as fh:
        fh.write(b"\0" * 80)
        fh.write(np.uint32(v.shape[0]).tobytes())
        fh.write(body.tobytes())
    return int(v.shape[0])
