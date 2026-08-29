"""三角形网格数据结构与加载器。

工业机器人模型常见的网格格式有两类：

* ``STL`` — ros-industrial / LeRobot 等开源模型库的主力格式，分二进制与 ASCII 两种。
  二进制 STL 体积紧凑（LeRobot SO-101 一套 14 个文件约 17 MB），ASCII STL 可读但臃肿。
* ``OBJ`` — 多数 CAD 导出流程与网格处理工具（MeshLab / Blender）的中间格式。

本模块只依赖 NumPy，不涉及任何 OpenGL 调用，因此可以在无显示环境下单元测试。

关键设计
--------
1. **法线重算（weld + 角度阈值）**：STL 只存逐面法线，且顶点在三角形之间完全重复。
   直接按面法线绘制会把圆角、圆柱面画成多面体。这里先按位置焊接顶点、用面积加权
   累加得到平滑法线，再与面法线做夹角比较：**夹角超过阈值（默认 50°）的面保留自身
   平面法线**，从而让圆角光滑、棱边锐利——这正是 CAD 模型看起来"有质感"的关键。
2. **退化三角形剔除**：零面积三角形会产生 NaN 法线，导致该处渲染成黑洞。
3. **顶点只写不索引**：焊接后一个顶点可能同时属于光滑面与棱边面，无法共用同一法线，
   因此输出展开为逐三角形顶点（每三角形 3 个独立顶点）。代价是内存翻倍，换来正确性。

单位约定
--------
STL / OBJ 文件本身**不携带单位信息**。URDF 一律使用米，因此调用方需自行判断：
若网格数值量级明显偏大（包围盒对角线 > 10），通常说明文件是按毫米导出的。
:func:`MeshData.suggest_unit_scale` 提供了这一启发式判断。
"""

from __future__ import annotations

import math
import re
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import numpy as np

__all__ = [
    "MeshData",
    "MeshLoadError",
    "load_mesh",
    "load_stl",
    "load_obj",
    "weld_vertices",
    "compute_normals",
]


#: 平滑着色的夹角阈值（度）。相邻面法线夹角小于该值时共用平滑法线，
#: 大于该值则判定为棱边、保留各自的平面法线。
SMOOTH_ANGLE_DEG = 50.0

#: 顶点焊接的量化精度（米）。1e-6 m = 1 微米，足以合并 STL 的重复顶点，
#: 又不会误合并相邻但不同的顶点。
WELD_PRECISION = 1e-6


class MeshLoadError(RuntimeError):
    """网格文件解析失败。

    与项目中其它 fail-loud 约定一致：调用方应显式处理，而不是静默降级为空模型
    ——空的机器人比报错更难排查。
    """


@dataclass(frozen=True)
class MeshData:
    """一份可直接送进 OpenGL 顶点数组的网格数据。

    Attributes:
        vertices: 展开后的顶点坐标，形状 ``(3M, 3)``，float32。已应用 ``scale``。
        normals: 与 ``vertices`` 一一对应的单位法线，形状 ``(3M, 3)``，float32。
        triangles: 三角形数量 ``M``。
        source: 来源文件路径，用于诊断信息。
    """

    vertices: np.ndarray
    normals: np.ndarray
    triangles: int
    source: str = ""

    @property
    def is_empty(self) -> bool:
        """网格是否不含任何有效三角形。"""
        return self.triangles == 0

    def bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """返回包围盒的 ``(min, max)``；空网格返回两个零向量。"""
        if self.triangles == 0:
            zero = np.zeros(3, dtype=np.float64)
            return zero, zero
        return self.vertices.min(axis=0), self.vertices.max(axis=0)

    def diagonal(self) -> float:
        """包围盒对角线长度，用于判断量级与自动缩放相机。"""
        lo, hi = self.bounds()
        return float(np.linalg.norm(hi - lo))

    def suggest_unit_scale(self) -> float:
        """启发式判断网格当前单位，返回"换算到米"所需的系数。

        STL/OBJ 不记录单位。经验规则：

        * 包围盒对角线 < 10 → 已按米导出，返回 ``1.0``
        * 否则 → 按毫米导出，返回 ``0.001``

        这对 LeRobot、ros-industrial 的模型都成立：工业机器人臂展在 0.5~4 m 之间，
        而按毫米导出时数值会是 500~4000。

        Returns:
            乘到顶点上即可得到米制坐标的系数。
        """
        if self.triangles == 0:
            return 1.0
        return 0.001 if self.diagonal() > 10.0 else 1.0


# --------------------------------------------------------------------------- #
# 顶点焊接
# --------------------------------------------------------------------------- #
def weld_vertices(vertices: np.ndarray, precision: float = WELD_PRECISION) -> np.ndarray:
    """把位置重合的顶点映射到同一个 id。

    STL 中每个三角形的 3 个顶点都是独立存储的，一个被 6 个三角形共享的角点会
    出现 6 份完全相同的坐标。焊接后才能累加相邻面法线、得到平滑着色。

    Args:
        vertices: 顶点坐标，形状 ``(N, 3)``。
        precision: 量化精度，单位与顶点一致。

    Returns:
        形状 ``(N,)`` 的 int64 数组，``weld[i]`` 是第 ``i`` 个顶点所属的焊接 id。
    """
    if vertices.size == 0:
        return np.zeros(0, dtype=np.int64)
    quantized = np.round(np.asarray(vertices, dtype=np.float64) / precision).astype(np.int64)
    # 用结构化视图做整行去重，比 lexsort 快得多
    view = np.ascontiguousarray(quantized).view([("", np.int64)] * 3).ravel()
    _, inverse = np.unique(view, return_inverse=True)
    return inverse.ravel().astype(np.int64)


def compute_normals(
    vertices: np.ndarray,
    tris: np.ndarray,
    smooth_angle_deg: float = SMOOTH_ANGLE_DEG,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """计算带棱边保护的平滑法线。

    算法（三趟，全部向量化，无 Python 层循环）：

    1. 计算逐面法线（未归一化的叉积，其长度正比于 2 倍面积，天然实现面积加权）。
    2. 把面法线按焊接后的顶点 id 累加，得到每个顶点的平滑法线。
    3. 对每个面，比较其法线与三顶点平滑法线的平均方向：夹角大于阈值则判定为棱边面，
       该面三个顶点改用面法线。

    Args:
        vertices: 顶点坐标，形状 ``(N, 3)``。
        tris: 三角形索引，形状 ``(M, 3)``。
        smooth_angle_deg: 判定棱边的夹角阈值（度）。

    Returns:
        ``(out_verts, out_normals, out_tris)``——顶点被展开为逐三角形形式，
        ``out_tris`` 为 ``arange(3M).reshape(M, 3)``。
    """
    if tris.size == 0:
        empty = np.zeros((0, 3), dtype=np.float32)
        return empty, empty.copy(), np.zeros((0, 3), dtype=np.int64)

    v = np.asarray(vertices, dtype=np.float64)
    tris = np.asarray(tris, dtype=np.int64)

    # ---- 1. 面法线（长度 = 2×面积） ---- #
    v0 = v[tris[:, 0]]
    v1 = v[tris[:, 1]]
    v2 = v[tris[:, 2]]
    face_n = np.cross(v1 - v0, v2 - v0)

    area2 = np.linalg.norm(face_n, axis=1)
    # 退化三角形：面积为 0，法线无意义，直接剔除以免产生 NaN
    valid = area2 > 1e-20
    if not np.all(valid):
        tris = tris[valid]
        if tris.size == 0:
            empty = np.zeros((0, 3), dtype=np.float32)
            return empty, empty.copy(), np.zeros((0, 3), dtype=np.int64)
        v0 = v[tris[:, 0]]
        v1 = v[tris[:, 1]]
        v2 = v[tris[:, 2]]
        face_n = np.cross(v1 - v0, v2 - v0)
        area2 = np.linalg.norm(face_n, axis=1)

    face_unit = face_n / area2[:, None]

    # ---- 2. 焊接 + 面积加权累加 ---- #
    weld = weld_vertices(v)
    n_welded = int(weld.max()) + 1

    flat_idx = tris.ravel()                    # (3M,)
    flat_weld = weld[flat_idx]                 # (3M,)
    flat_fn = np.repeat(face_n, 3, axis=0)     # (3M, 3)

    acc = np.empty((n_welded, 3), dtype=np.float64)
    for c in range(3):
        # bincount 比 np.add.at 快一个数量级，是这里能处理 30 万面的关键
        acc[:, c] = np.bincount(flat_weld, weights=flat_fn[:, c], minlength=n_welded)

    acc_len = np.linalg.norm(acc, axis=1)
    # 累加结果为零向量：只可能出现在退化情形，退化面已剔除，这里兜底避免除零
    safe_len = np.maximum(acc_len, 1e-20)
    smooth = acc / safe_len[:, None]
    # 焊接后仍为零长度的顶点，法线退化为 +Z，保证后续渲染不出现 NaN
    degenerate = acc_len < 1e-20
    if np.any(degenerate):
        smooth[degenerate] = (0.0, 0.0, 1.0)

    # ---- 3. 棱边判定 ---- #
    m = tris.shape[0]
    per_corner_smooth = smooth[flat_weld].reshape(m, 3, 3)
    face_smooth = per_corner_smooth.mean(axis=1)
    face_smooth_len = np.linalg.norm(face_smooth, axis=1, keepdims=True)
    face_smooth_unit = face_smooth / np.maximum(face_smooth_len, 1e-20)

    cos_thresh = math.cos(math.radians(smooth_angle_deg))
    dot = np.einsum("ij,ij->i", face_smooth_unit, face_unit)
    # 平滑法线不可信（长度趋零 → 该面附近法线互相抵消）时也退化为平面法线
    flat_face = (dot < cos_thresh) | (face_smooth_len[:, 0] < 1e-6)

    corner_normals = per_corner_smooth.copy()
    corner_normals[flat_face] = face_unit[flat_face][:, None, :]

    # 逐顶点法线归一化（平滑法线已是单位向量，平面法线亦然，这里只消除数值误差）
    out_normals = corner_normals.reshape(-1, 3)
    out_normals /= np.maximum(np.linalg.norm(out_normals, axis=1, keepdims=True), 1e-20)

    out_verts = v[flat_idx]
    out_tris = np.arange(m * 3, dtype=np.int64).reshape(m, 3)
    return (
        out_verts.astype(np.float32),
        out_normals.astype(np.float32),
        out_tris,
    )


# --------------------------------------------------------------------------- #
# STL
# --------------------------------------------------------------------------- #
def _looks_like_binary(raw: bytes) -> bool:
    """判断一段 STL 数据是否为二进制格式。

    ASCII STL 一定以 ``solid`` 开头；二进制 STL 前 80 字节是任意头部，
    可能恰好也以 ``solid`` 开头，因此再校验：三角形数量与文件长度是否自洽。
    """
    if len(raw) < 84:
        return False
    if not raw[:5].lower().startswith(b"solid"):
        return True
    count = struct.unpack_from("<I", raw, 80)[0]
    expected = 84 + count * 50
    return expected == len(raw)


def load_stl(path: Path, scale: float = 1.0) -> MeshData:
    """加载 STL 网格（自动识别二进制 / ASCII）。

    Args:
        path: STL 文件路径。
        scale: 统一缩放系数（URDF ``<mesh scale="...">`` 的 x 分量或统一值）。

    Returns:
        已应用缩放、法线经过棱边保护平滑的 :class:`MeshData`。

    Raises:
        MeshLoadError: 文件不存在、格式非法或不含任何有效三角形。
    """
    path = Path(path)
    if not path.is_file():
        raise MeshLoadError(f"STL 文件不存在: {path}")

    raw = path.read_bytes()
    if _looks_like_binary(raw):
        verts, tris = _parse_stl_binary(raw, path)
    else:
        verts, tris = _parse_stl_ascii(raw, path)

    if tris.size == 0:
        raise MeshLoadError(f"STL 文件不含有效三角形: {path}")

    verts = np.asarray(verts, dtype=np.float64) * float(scale)
    out_v, out_n, out_t = compute_normals(verts, tris)
    return MeshData(
        vertices=out_v,
        normals=out_n,
        triangles=int(out_t.shape[0]),
        source=str(path),
    )


def _parse_stl_binary(raw: bytes, path: Path) -> Tuple[np.ndarray, np.ndarray]:
    """解析二进制 STL。

    布局：80 字节头部 + uint32 三角形数 + 每三角形 50 字节
    （法线 3×f4 + 三顶点 9×f4 + uint16 属性）。文件自带的面法线**不参与**着色，
    因为我们要用面积加权的重算法线消除多面体感；这里只读取顶点。
    """
    count = struct.unpack_from("<I", raw, 80)[0]
    expected = 84 + count * 50
    if expected > len(raw):
        raise MeshLoadError(
            f"{path}: 二进制 STL 长度不足——头部声明 {count} 个三角形"
            f"（需 {expected} 字节），实际 {len(raw)} 字节"
        )

    # 直接把 50 字节的块视作结构化数组，一次读出全部三角形，避免逐面 struct.unpack
    dtype = np.dtype([
        ("n", "<f4", 3),
        ("v", "<f4", (3, 3)),
        ("attr", "<u2"),
    ])
    body = np.frombuffer(raw, dtype=dtype, count=count, offset=84)
    verts = body["v"].reshape(-1, 3).astype(np.float64)
    tris = np.arange(verts.shape[0], dtype=np.int64).reshape(-1, 3)
    return verts, tris


_ASCII_FACET_RE = re.compile(
    r"facet\s+normal\s+(?P<nx>\S+)\s+(?P<ny>\S+)\s+(?P<nz>\S+)"
    r".*?outer\s+loop(?P<loop>.*?)endloop",
    re.IGNORECASE | re.DOTALL,
)
_ASCII_VERTEX_RE = re.compile(
    r"vertex\s+(?P<x>[-+0-9.eE]+)\s+(?P<y>[-+0-9.eE]+)\s+(?P<z>[-+0-9.eE]+)"
)


def _parse_stl_ascii(raw: bytes, path: Path) -> Tuple[np.ndarray, np.ndarray]:
    """解析 ASCII STL。

    只读取顶点坐标，法线一律重算（理由同二进制分支）。
    """
    text = raw.decode("utf-8", errors="replace")
    coords: List[float] = []
    for facet in _ASCII_FACET_RE.finditer(text):
        # 面法线已在正则里跳过——与二进制分支一样，一律重算
        loop = facet.group("loop")
        pts = _ASCII_VERTEX_RE.findall(loop)
        if len(pts) != 3:
            continue
        for px, py, pz in pts:
            coords.extend((float(px), float(py), float(pz)))

    if not coords:
        raise MeshLoadError(f"{path}: ASCII STL 未解析到任何顶点")

    verts = np.asarray(coords, dtype=np.float64).reshape(-1, 3)
    tris = np.arange(verts.shape[0], dtype=np.int64).reshape(-1, 3)
    return verts, tris


# --------------------------------------------------------------------------- #
# OBJ
# --------------------------------------------------------------------------- #
def load_obj(path: Path, scale: float = 1.0) -> MeshData:
    """加载 Wavefront OBJ 网格（顶点 + 面，忽略材质/纹理/曲线）。

    OBJ 的面可能是四边形或更高阶多边形，这里统一做扇形三角化。

    Args:
        path: OBJ 文件路径。
        scale: 统一缩放系数。

    Returns:
        :class:`MeshData`。

    Raises:
        MeshLoadError: 文件不存在或未含任何面。
    """
    path = Path(path)
    if not path.is_file():
        raise MeshLoadError(f"OBJ 文件不存在: {path}")

    verts: List[Tuple[float, float, float]] = []
    tris: List[Tuple[int, int, int]] = []
    with path.open("r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            if line.startswith("v "):
                parts = line.split()
                verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif line.startswith("f "):
                # 顶点索引可能是 "v"、"v/vt"、"v/vt/vn" 三种写法，取第一段
                idx = [int(tok.split("/")[0]) for tok in line.split()[1:]]
                if len(idx) < 3:
                    continue
                # OBJ 索引从 1 开始，负索引表示从末尾倒数
                base = idx[0]
                start = base - 1 if base > 0 else len(verts) + base
                for k in range(1, len(idx) - 1):
                    a = idx[k]
                    b = idx[k + 1]
                    tris.append((
                        start,
                        a - 1 if a > 0 else len(verts) + a,
                        b - 1 if b > 0 else len(verts) + b,
                    ))

    if not tris:
        raise MeshLoadError(f"{path}: OBJ 未解析到任何面")

    v = np.asarray(verts, dtype=np.float64) * float(scale)
    t = np.asarray(tris, dtype=np.int64)
    out_v, out_n, out_t = compute_normals(v, t)
    return MeshData(vertices=out_v, normals=out_n, triangles=int(out_t.shape[0]), source=str(path))


# --------------------------------------------------------------------------- #
# 统一入口
# --------------------------------------------------------------------------- #
def load_mesh(path: Path, scale: float = 1.0) -> MeshData:
    """按扩展名分派网格加载器。

    Args:
        path: 网格文件路径（``.stl`` / ``.obj``，大小写不敏感）。
        scale: 统一缩放系数。

    Returns:
        :class:`MeshData`。

    Raises:
        MeshLoadError: 扩展名不受支持，或底层加载器失败。
    """
    suffix = Path(path).suffix.lower()
    if suffix == ".stl":
        return load_stl(path, scale)
    if suffix == ".obj":
        return load_obj(path, scale)
    raise MeshLoadError(f"不支持的网格格式 '{suffix}': {path}（仅支持 .stl / .obj）")
