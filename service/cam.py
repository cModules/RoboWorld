"""STEP/CAM 轨迹生成（v0.3.2）——从 CAD 模型提取焊缝/轮廓并生成机器人路径。

三个能力块
----------
1. **CAD 导入与边提取**（OCP 通道，与 ``tools/build_vendor_urdf.py`` 复用同一套
   OpenCASCADE 绑定；OCP 是软依赖，未安装时本模块其余纯 numpy 功能照常可用）：
   ``load_cad`` 读 STEP/IGES → ``extract_edge_polylines`` 把每条拓扑边离散成
   折线 → ``chain_polylines`` 把共点边串成连续焊缝/轮廓链 → ``extract_seams``
   一步到位并按长度/闭合过滤。

2. **路径 → 程序生成**（纯 numpy，不依赖 OCP）：``detect_corners`` 识别折线
   转角点，``path_to_program`` 按「平滑段用 moveC 圆弧、转角/短段用 moveL」
   的策略生成 :class:`~service.program.ProgramBuilder`，可链式补 moveJ 进退刀、
   ``gripper()`` 等指令后 ``.build()``。

3. **点云扫描偏差比对**（纯 numpy）：``load_ply`` 读 ASCII / 二进制（大小端）
   PLY 点云 → ``scan_deviation`` 计算每个扫描点到规划路径折线的最近距离，
   输出 max/mean/rms 与最差点定位（复用 :class:`~service.debug.Deviation` 的
   数据语义，供后续 GUI 可视化）。

单位约定
--------
与规划器一致：全部坐标为 **毫米**。OpenCASCADE 的 STEP/IGES 读取默认按 mm
返回几何，因此 ``load_cad`` 不做额外缩放。

用法::

    from service.cam import load_cad, extract_seams, path_to_program, scan_deviation

    shape = load_cad("bracket.step")
    seams = extract_seams(shape, min_length=10.0)
    builder = path_to_program(seams[0].points, robot, z_lift=2.0)
    program = builder.build()
    report = scan_deviation(load_ply("scan.ply"), seams[0].points)
    print(report.format())
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

__all__ = [
    "SeamPath",
    "DeviationReport",
    "load_cad",
    "extract_edge_polylines",
    "chain_polylines",
    "extract_seams",
    "select_seams",
    "detect_corners",
    "path_to_program",
    "load_ply",
    "scan_deviation",
]


# --------------------------------------------------------------------------- #
# 数据类
# --------------------------------------------------------------------------- #
@dataclass
class SeamPath:
    """一条提取出的连续路径（焊缝 / 轮廓 / 切割线）。

    Attributes:
        points: (N, 3) 路径点，单位 mm，按路径顺序排列
        closed: 首尾闭合标志（闭合焊缝如环形角焊缝）
        source: 来源描述（如 "step:bracket.step" 或 "chain"）
    """

    points: np.ndarray
    closed: bool = False
    source: str = ""

    @property
    def length(self) -> float:
        """路径总长度（mm）；闭合路径计入收尾段。"""
        pts = _as_points(self.points)
        if len(pts) < 2:
            return 0.0
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()
        if self.closed and len(pts) > 2:
            seg += float(np.linalg.norm(pts[0] - pts[-1]))
        return float(seg)

    def __len__(self) -> int:
        return len(self.points)


@dataclass
class DeviationReport:
    """扫描点云相对规划路径的偏差比对结果。

    Attributes:
        distances: 每个扫描点到规划路径折线的最近距离（mm）
        path_length: 规划路径折线总长（mm）
    """

    distances: np.ndarray
    path_length: float = 0.0

    @property
    def n_points(self) -> int:
        return len(self.distances)

    @property
    def max_dev(self) -> float:
        return float(np.max(self.distances)) if len(self.distances) else 0.0

    @property
    def mean_dev(self) -> float:
        return float(np.mean(self.distances)) if len(self.distances) else 0.0

    @property
    def rms_dev(self) -> float:
        return float(np.sqrt(np.mean(np.square(self.distances)))) if len(self.distances) else 0.0

    @property
    def worst_index(self) -> int:
        return int(np.argmax(self.distances)) if len(self.distances) else -1

    def format(self) -> str:
        if not len(self.distances):
            return "扫描偏差比对：无扫描点"
        return (
            f"扫描偏差比对（{self.n_points} 点 vs 路径 {self.path_length:.1f} mm）\n"
            f"  max  = {self.max_dev:.3f} mm（点 #{self.worst_index}）\n"
            f"  mean = {self.mean_dev:.3f} mm\n"
            f"  rms  = {self.rms_dev:.3f} mm"
        )


def _as_points(seq: Sequence[Sequence[float]]) -> np.ndarray:
    pts = np.asarray(seq, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(f"路径点必须是 (N, 3)，收到 {pts.shape}")
    if len(pts) < 2:
        raise ValueError("路径至少需要 2 个点")
    return pts


# --------------------------------------------------------------------------- #
# CAD 导入与边提取（OCP 通道，软依赖）
# --------------------------------------------------------------------------- #
def load_cad(path: str | Path) -> "object":
    """读取 STEP / IGES 文件为 OpenCASCADE 形状（单位 mm）。

    Args:
        path: ``.step/.stp/.iges/.igs`` 文件路径

    Returns:
        ``OCP.TopoDS.TopoDS_Shape``（顶层 compound/shape）

    Raises:
        ImportError: 未安装 ``cadquery-ocp``
        ValueError: 后缀不支持或文件读取失败
    """
    p = Path(path)
    ext = p.suffix.lower()
    if ext in (".step", ".stp"):
        from OCP.STEPControl import STEPControl_Reader
        from OCP.IFSelect import IFSelect_RetDone

        reader = STEPControl_Reader()
        if reader.ReadFile(str(p)) != IFSelect_RetDone:
            raise ValueError(f"STEP 读取失败: {p}")
        reader.TransferRoots()
        if reader.NbShapes() < 1:
            raise ValueError(f"STEP 文件中无几何: {p}")
        return reader.OneShape()
    if ext in (".iges", ".igs"):
        from OCP.IGESControl import IGESControl_Reader
        from OCP.IFSelect import IFSelect_RetDone

        reader = IGESControl_Reader()
        if reader.ReadFile(str(p)) != IFSelect_RetDone:
            raise ValueError(f"IGES 读取失败: {p}")
        reader.TransferRoots()
        if reader.NbShapes() < 1:
            raise ValueError(f"IGES 文件中无几何: {p}")
        return reader.OneShape()
    raise ValueError(f"不支持的 CAD 后缀 {ext!r}（支持 .step/.stp/.iges/.igs）")


def extract_edge_polylines(
    shape,
    deflection: float = 0.2,
    angle_rad: float = 0.35,
) -> List[np.ndarray]:
    """把形状的所有拓扑边离散成折线点列。

    先对整体做三角网格化（``BRepMesh_IncrementalMesh``），再用
    ``GCPnts_QuasiUniformDeflection`` 按给定弦高容差离散每条边；共享边经
    ``TopExp.MapShapes_s`` 去重，退化边（长度 < 1e-9）丢弃。

    Args:
        shape: OpenCASCADE ``TopoDS_Shape``
        deflection: 弦高容差（mm），越小点越密
        angle_rad: 转角离散容差（弧度）

    Returns:
        ``[(N_i, 3) ndarray, ...]``，每条边一条折线
    """
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_EDGE
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.GCPnts import GCPnts_QuasiUniformDeflection

    from adapter.ocpcompat import edge_of

    BRepMesh_IncrementalMesh(shape, deflection, False, angle_rad, True)

    # 共享边会被多个面各访问一次（且朝向可能不同），用 IsSame（TShape+位置，
    # 忽略朝向）去重
    seen: List = []
    polylines: List[np.ndarray] = []
    explorer = TopExp_Explorer(shape, TopAbs_EDGE)
    while explorer.More():
        edge = explorer.Current()
        explorer.Next()
        if any(edge.IsSame(s) for s in seen):
            continue
        seen.append(edge)
        curve = BRepAdaptor_Curve(edge_of(edge))
        disc = GCPnts_QuasiUniformDeflection(curve, deflection)
        if not disc.IsDone() or disc.NbPoints() < 2:
            continue
        pts = np.array(
            [
                [disc.Value(j).X(), disc.Value(j).Y(), disc.Value(j).Z()]
                for j in range(1, disc.NbPoints() + 1)
            ]
        )
        # 区分「真退化边」与「闭合边」（整圆焊缝：起点=终点但覆盖整个回路）
        if np.linalg.norm(pts[-1] - pts[0]) < 1e-9:
            extent = np.linalg.norm(pts - pts[0], axis=1).max()
            if not (getattr(curve, "IsClosed", lambda: False)() and extent > 1e-6):
                continue  # 退化边
        polylines.append(pts)
    return polylines


def chain_polylines(
    polylines: Sequence[np.ndarray],
    tol: float = 1e-3,
) -> List[SeamPath]:
    """把离散折线按端点共点关系串成连续链（焊缝提取的核心）。

    贪心拼接：每条折线尝试接在某个既有链的头/尾（含反向）；若一条链的
    尾端与头端相遇（距离 ≤ ``tol``）则标记闭合。

    Args:
        polylines: ``[(N_i, 3)]`` 折线点列
        tol: 端点匹配容差（mm）

    Returns:
        连接后的 :class:`SeamPath` 列表
    """
    if not polylines:
        return []
    chains: List[dict] = []
    for pl in polylines:
        pts = np.asarray(pl, dtype=np.float64)
        self_closed = len(pts) > 3 and np.linalg.norm(pts[0] - pts[-1]) <= tol
        if self_closed:
            # 闭合回路本身就是完整焊缝，独立成链，不再吸收其它边
            chains.append({"pts": pts.copy(), "closed": True})
            continue
        # 找能与当前折线首/尾相连的开放链
        merged_into: Optional[dict] = None
        for ch in chains:
            if ch["closed"]:
                continue
            if np.linalg.norm(pts[0] - ch["pts"][-1]) <= tol:
                ch["pts"] = np.vstack([ch["pts"], pts])
                merged_into = ch
                break
            if np.linalg.norm(pts[-1] - ch["pts"][-1]) <= tol:
                ch["pts"] = np.vstack([ch["pts"], pts[::-1]])
                merged_into = ch
                break
            if np.linalg.norm(pts[-1] - ch["pts"][0]) <= tol:
                ch["pts"] = np.vstack([pts, ch["pts"]])
                merged_into = ch
                break
            if np.linalg.norm(pts[0] - ch["pts"][0]) <= tol:
                ch["pts"] = np.vstack([pts[::-1], ch["pts"]])
                merged_into = ch
                break
        if merged_into is None:
            closed = (
                len(pts) > 3 and np.linalg.norm(pts[0] - pts[-1]) <= tol
            )  # 单条闭合折线（如整圆焊缝）
            chains.append({"pts": pts.copy(), "closed": closed})
            continue
        # 合并后检查是否首尾相遇 → 闭合
        tail = merged_into["pts"][-1]
        head = merged_into["pts"][0]
        if len(merged_into["pts"]) > 3 and np.linalg.norm(tail - head) <= tol:
            merged_into["closed"] = True
    return [
        SeamPath(
            points=_dedupe_consecutive(ch["pts"], tol),
            closed=ch["closed"],
            source="chain",
        )
        for ch in chains
    ]


def _dedupe_consecutive(pts: np.ndarray, tol: float) -> np.ndarray:
    """去掉连续重复的接合顶点（链拼接在共享端点处会复制一次）。"""
    if len(pts) < 2:
        return pts
    keep = [pts[0]]
    for p in pts[1:]:
        if np.linalg.norm(p - keep[-1]) > tol:
            keep.append(p)
    return np.asarray(keep, dtype=np.float64)


def extract_seams(
    cad_input,
    deflection: float = 0.2,
    chain_tol: float = 1e-3,
) -> List[SeamPath]:
    """从 CAD 形状或文件一步提取焊缝/轮廓链。

    Args:
        cad_input: ``TopoDS_Shape`` 或 STEP/IGES 文件路径
        deflection: 边离散弦高容差（mm）
        chain_tol: 链拼接端点容差（mm）

    Returns:
        全部 :class:`SeamPath`（未过滤，用 :func:`select_seams` 筛选）
    """
    shape = load_cad(cad_input) if isinstance(cad_input, (str, Path)) else cad_input
    edges = extract_edge_polylines(shape, deflection=deflection)
    return chain_polylines(edges, tol=chain_tol)


def select_seams(
    seams: Sequence[SeamPath],
    min_length: float = 0.0,
    closed: Optional[bool] = None,
    index: Optional[int] = None,
) -> List[SeamPath]:
    """按长度 / 闭合 / 序号筛选焊缝。

    Args:
        seams: 候选焊缝列表
        min_length: 最小路径长度（mm）
        closed: ``True`` 只留闭合缝，``False`` 只留开放缝，``None`` 不过滤
        index: 直接按序号取一条（优先于其余条件）

    Returns:
        筛选后的焊缝列表
    """
    if index is not None:
        if not 0 <= index < len(seams):
            raise IndexError(f"焊缝序号 {index} 超出范围（共 {len(seams)} 条）")
        return [seams[index]]
    out = [s for s in seams if s.length >= min_length]
    if closed is not None:
        out = [s for s in out if s.closed == closed]
    return out


# --------------------------------------------------------------------------- #
# 路径 → 程序生成（纯 numpy）
# --------------------------------------------------------------------------- #
def _turn_angles(pts: np.ndarray, closed: bool = False) -> np.ndarray:
    """每个点的方向变化角（度）；开放路径首尾点为 0。"""
    n = len(pts)
    turns = np.zeros(n)
    idxs = range(n) if closed else range(1, n - 1)
    for i in idxs:
        d1 = pts[i] - pts[(i - 1) % n]
        d2 = pts[(i + 1) % n] - pts[i]
        n1 = np.linalg.norm(d1)
        n2 = np.linalg.norm(d2)
        if n1 > 1e-12 and n2 > 1e-12:
            cos_a = np.clip(float(d1 @ d2) / (n1 * n2), -1.0, 1.0)
            turns[i] = np.degrees(np.arccos(cos_a))
    return turns


def detect_corners(
    points: np.ndarray,
    angle_deg: float = 20.0,
    closed: bool = False,
) -> List[int]:
    """识别折线上的转角点（相邻段方向变化超过阈值）。

    Args:
        points: (N, 3) 路径点
        angle_deg: 转角阈值（度），方向变化大于该值的点视为转角
        closed: 闭合路径时首尾点也参与转角判别（环形焊缝拐角）

    Returns:
        转角点的索引列表
    """
    pts = _as_points(points)
    if len(pts) < 3:
        return []
    turns = _turn_angles(pts, closed=closed)
    return [i for i, a in enumerate(turns) if a > angle_deg]


def _chunk_arc_run(
    seq: List[int],
    turns: np.ndarray,
    max_arc_span_deg: float,
) -> List[Tuple[int, int]]:
    """把一条平滑运行段（点索引序列 ``seq``）按累计转角切成弧段。

    Returns:
        ``[(起点位置, 终点位置), ...]``（位置是 ``seq`` 的下标），首尾相接
        覆盖整个 ``seq``。
    """
    chunks: List[Tuple[int, int]] = []
    last = 0
    acc = 0.0
    for pos in range(1, len(seq) - 1):
        acc += turns[seq[pos]]
        if acc >= max_arc_span_deg and pos - last >= 2:
            chunks.append((last, pos))
            last = pos
            acc = 0.0
    chunks.append((last, len(seq) - 1))
    return chunks


def path_to_program(
    points,
    robot,
    *,
    corner_angle_deg: float = 20.0,
    min_arc_points: int = 4,
    max_arc_span_deg: float = 120.0,
    speed: float = 100.0,
    velocity: float = 50.0,
    blend: float = 1.0,
    z_lift: float = 0.0,
    normal: Sequence[float] = (0.0, 0.0, 1.0),
    rpy: Sequence[float] = (0.0, 180.0, 0.0),
    closed: bool = False,
):
    """把路径点列转成机器人程序（平滑段 moveC 圆弧、转角/短段 moveL）。

    策略：

    1. :func:`detect_corners` 找转角点，路径在转角处断开；
    2. 两个转角（或端点）之间的平滑段按累计转角切片，每片不超过
       ``max_arc_span_deg``（真实控制器 moveC 无法执行整圆），每片用一条
       ``moveC``（经由点取片中点）拟合；点数过少的片段逐点 ``moveL``；
    3. 闭合路径（``closed=True``）按环形处理，最后一段自动回到起点。

    生成的 :class:`~service.program.ProgramBuilder` 未 build，可继续链式追加
    进退刀 ``moveJ``、``gripper()`` 等指令。

    Args:
        points: (N, 3) 路径点（mm）；闭合路径若首尾重复会自动去重
        robot: :class:`~domain.robot.Robot`，用于构建程序
        corner_angle_deg: 转角判别阈值（度）
        min_arc_points: 一段至少多少个点才用 moveC 圆弧
        max_arc_span_deg: 单条 moveC 允许的最大圆心角（度），防止整圆指令
        speed: 速度百分比 (0-100)
        velocity: TCP 线速度覆盖（mm/s）
        blend: 融合半径（mm）
        z_lift: 路径整体沿 ``normal`` 抬高量（mm，焊炬偏置/防碰撞）
        normal: 工件表面法向（默认 +Z），与 ``z_lift`` 配合
        rpy: 固定工具姿态（度，RPY ZYX 欧拉），默认焊炬垂直向下
        closed: 路径是否闭合（环形焊缝/轮廓）

    Returns:
        :class:`~service.program.ProgramBuilder`（调用 ``.build()`` 得到 Program）
    """
    from domain.pose import Pose
    from .program import ProgramBuilder

    pts = _as_points(points).copy()
    # 防御性去重：连续重复点会让转角检测/圆弧拟合退化
    pts = _dedupe_consecutive(pts, 1e-6)
    # 闭合输入去重尾点
    if closed and len(pts) > 2 and np.linalg.norm(pts[0] - pts[-1]) < 1e-6:
        pts = pts[:-1]
    # 抬高偏置 + 固定姿态
    nrm = np.asarray(normal, dtype=np.float64)
    nrm = nrm / np.linalg.norm(nrm)
    pts = pts + z_lift * nrm
    rpy_pose = Pose.from_xyz_rpy(0.0, 0.0, 0.0, *rpy)
    poses = []
    for p in pts:
        pose = rpy_pose.copy()
        pose.matrix[:3, 3] = p
        poses.append(pose)

    n = len(pts)
    turns = _turn_angles(pts, closed=closed)
    corners = sorted(i for i, a in enumerate(turns) if a > corner_angle_deg)

    builder = ProgramBuilder(robot)

    def emit_run(seq: List[int]) -> None:
        """把一段（点索引序列 seq，首点是当前已到达位置）加入程序。"""
        if len(seq) < 2:
            return
        for a, b in _chunk_arc_run(seq, turns, max_arc_span_deg):
            if b - a + 1 >= min_arc_points:
                via = seq[(a + b) // 2]
                builder.moveC(
                    poses[via % n],
                    poses[seq[b] % n],
                    speed=speed,
                    velocity=velocity,
                    blend=blend,
                )
            else:
                for pos in range(a + 1, b + 1):
                    builder.moveL(
                        poses[seq[pos] % n],
                        speed=speed,
                        velocity=velocity,
                        blend=blend,
                    )

    def circ_seq(i: int, j: int) -> List[int]:
        """闭合路径上从点 i（含）沿正向到点 j（含）的索引序列（可绕圈）。"""
        seq = [i % n]
        k = i
        while k % n != j % n or len(seq) == 1 and i == j:
            k += 1
            seq.append(k % n)
            if k % n == j % n:
                break
            if len(seq) > 2 * n + 2:  # 防御性保护
                break
        return seq

    if closed:
        start = corners[0] if corners else 0
        builder.moveL(poses[start], speed=speed, velocity=velocity)
        if corners:
            ordered = sorted(corners)
            for i, j in zip(ordered, ordered[1:] + ordered[:1]):
                emit_run(circ_seq(i, j))
        else:
            emit_run(circ_seq(0, 0))
    else:
        breaks = sorted(set([0] + corners + [n - 1]))
        builder.moveL(poses[0], speed=speed, velocity=velocity)
        for a, b in zip(breaks[:-1], breaks[1:]):
            emit_run(list(range(a, b + 1)))
    return builder


# --------------------------------------------------------------------------- #
# PLY 点云导入（纯 numpy，无外部依赖）
# --------------------------------------------------------------------------- #
_PLY_SCALARS = {
    "char": ("i1", 1),
    "int8": ("i1", 1),
    "uchar": ("u1", 1),
    "uint8": ("u1", 1),
    "short": ("i2", 2),
    "int16": ("i2", 2),
    "ushort": ("u2", 2),
    "uint16": ("u2", 2),
    "int": ("i4", 4),
    "int32": ("i4", 4),
    "uint": ("u4", 4),
    "uint32": ("u4", 4),
    "float": ("f4", 4),
    "float32": ("f4", 4),
    "double": ("f8", 8),
    "float64": ("f8", 8),
}


def load_ply(path: str | Path) -> np.ndarray:
    """读取 PLY 点云的顶点坐标（支持 ASCII 与 binary 大/小端）。

    只解析第一个 ``element vertex`` 块；后续 element（如 face 列表属性）跳过。
    顶点属性可含 nx/ny/nz、rgb 等，仅取 x/y/z。

    Args:
        path: ``.ply`` 文件路径

    Returns:
        (N, 3) float64 点阵

    Raises:
        ValueError: 非 PLY 文件、缺少 vertex element 或 x/y/z 属性
    """
    p = Path(path)
    with open(p, "rb") as f:
        raw = f.read()
    if not raw.startswith(b"ply"):
        raise ValueError(f"不是 PLY 文件: {p}")
    # 头部到 end_header（ascii 解析对二进制安全：头部一定是文本）
    header_end = raw.find(b"end_header")
    if header_end < 0:
        raise ValueError(f"PLY 头部不完整: {p}")
    header_text = raw[:header_end].decode("ascii", errors="replace")
    lines = [ln.strip() for ln in header_text.splitlines() if ln.strip()]

    fmt: Optional[str] = None
    elements: List[dict] = []
    cur: Optional[dict] = None
    for ln in lines:
        tok = ln.split()
        if tok[0] == "format":
            fmt = tok[1]
            if fmt not in ("ascii", "binary_little_endian", "binary_big_endian"):
                raise ValueError(f"未知 PLY 格式: {fmt}")
        elif tok[0] == "element":
            cur = {"name": tok[1], "count": int(tok[2]), "props": []}
            elements.append(cur)
        elif tok[0] == "property" and cur is not None:
            if tok[1] == "list":
                cur["props"].append(
                    {"list": True, "count_type": tok[2], "item_type": tok[3], "name": tok[4]}
                )
            else:
                cur["props"].append({"list": False, "type": tok[1], "name": tok[2]})

    vertex = next((e for e in elements if e["name"] == "vertex"), None)
    if vertex is None:
        raise ValueError(f"PLY 缺少 vertex element: {p}")
    names = [pr["name"] for pr in vertex["props"] if not pr.get("list")]
    for axis in ("x", "y", "z"):
        if axis not in names:
            raise ValueError(f"PLY vertex 缺少 {axis} 属性: {p}")
    xi, yi, zi = names.index("x"), names.index("y"), names.index("z")

    body = raw[header_end + len("end_header") :]
    if body[:1] in (b"\r", b"\n"):
        body = body[1:]
    n = vertex["count"]

    if fmt == "ascii":
        pts = np.empty((n, 3), dtype=np.float64)
        filled = 0
        for ln in body.decode("ascii", errors="replace").splitlines():
            toks = ln.split()
            if not toks:
                continue
            # 属性可能带 list（如面数据在顶点后），只取前 len(names) 个标量
            if len(toks) < len(names):
                continue
            vals = [float(t) for t in toks[: len(names)]]
            pts[filled] = (vals[xi], vals[yi], vals[zi])
            filled += 1
            if filled == n:
                break
        if filled != n:
            raise ValueError(f"PLY 顶点数不足: 期望 {n}，读到 {filled}")
        return pts

    # 二进制
    endian = "<" if fmt == "binary_little_endian" else ">"
    dtype_specs = []
    for pr in vertex["props"]:
        if pr.get("list"):
            # list 属性（顶点块通常没有；若有则逐项变长，暂不支持）
            raise ValueError("vertex element 含 list 属性，暂不支持")
        code, size = _PLY_SCALARS[pr["type"]]
        dtype_specs.append((pr["name"], endian + code))
    dtype = np.dtype(dtype_specs)
    rec = np.frombuffer(body[: dtype.itemsize * n], dtype=dtype, count=n)
    return np.column_stack(
        [rec["x"].astype(np.float64), rec["y"].astype(np.float64), rec["z"].astype(np.float64)]
    )


# --------------------------------------------------------------------------- #
# 扫描偏差比对（纯 numpy）
# --------------------------------------------------------------------------- #
def _point_segment_distance(points: np.ndarray, seg_a: np.ndarray, seg_b: np.ndarray) -> np.ndarray:
    """每个点到线段 [seg_a, seg_b] 的距离（向量化）。"""
    ab = seg_b - seg_a  # (3,)
    ap = points - seg_a  # (N, 3)
    denom = float(ab @ ab)
    if denom < 1e-18:
        return np.linalg.norm(ap, axis=1)
    t = np.clip(ap @ ab / denom, 0.0, 1.0)
    proj = seg_a[None, :] + t[:, None] * ab[None, :]
    return np.linalg.norm(points - proj, axis=1)


def scan_deviation(points, path_points) -> DeviationReport:
    """计算扫描点云相对规划路径折线的逐点最近距离。

    Args:
        points: (N, 3) 扫描点云
        path_points: (M, 3) 规划路径折线

    Returns:
        :class:`DeviationReport`（max/mean/rms + 最差点定位）
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(f"扫描点必须是 (N, 3)，收到 {pts.shape}")
    path = _as_points(path_points)
    if len(pts) == 0:
        return DeviationReport(distances=np.empty(0), path_length=0.0)
    dist = np.full(len(pts), np.inf)
    for a, b in zip(path[:-1], path[1:]):
        dist = np.minimum(dist, _point_segment_distance(pts, a, b))
    if path[0] is not None and len(path) >= 2 and not np.all(np.isfinite(dist)):
        # 理论上折线段覆盖所有点，这里兜底
        dist = np.where(
            np.isfinite(dist),
            dist,
            np.linalg.norm(pts[:, None, :] - path[None, :, :], axis=2).min(axis=1),
        )
    length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
    return DeviationReport(distances=dist, path_length=length)
