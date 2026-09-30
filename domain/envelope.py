"""包络接触容差：逐机器人配置与几何估算（v1.0.9）。

背景
----
厂商 URDF 的 ``<collision>`` 是**保守包络体**（UR5e 前臂包络盒 59.7×49mm vs
真实 Ø54mm 圆管），浅穿透属模型误差而非撞机——因此干涉检查要留容差
（:data:`service.planner.ENVELOPE_TOLERANCE_MM`，全局默认 20mm，UR5e 实测标定）。

v1.0.9 回答两个问题：

1. **逐机器人可配**：换品牌（FANUC/KUKA 包络盒胖瘦不同）不该被迫共用一个
   全局值。``Robot(envelope_tolerance_mm=…)`` 给单机覆盖；
   :func:`envelope_tolerance_mm` 统一解析——逐机值优先，未配置回退全局。

2. **几何估算（诚实的下界）**：:func:`estimate_envelope_tolerance_mm` 逐连杆
   把 ``<visual>`` mesh 顶点变换进每个 ``<collision>`` 包络体坐标系，在包络体
   跨度内取 mesh 最大径向距离，与包络体横截面角点半径相减——得到"包络体在
   贴着 mesh 的地方最多还能戳出去多少"的**几何下界**。

   实测教训（UR5e）：官方包络体基本贴着 mesh 建，几何下界只有 0–4mm；而
   抓取工作站实测最深包络穿透 18.6mm——差距来自 mesh 的**凹陷处**与姿态
   相关的接触位置，纯几何法看不到。所以这个估算的正确用法是：①新品牌
   包络"胖瘦"的横向对比（谁最需要大容差、最胖的连杆在哪）；②手工改过
   包络体后的回归检查。**它不能替代实测标定**——上机前按 ``docs/`` 流程
   实测复核，取 ``max(几何下界, 实测值)``。
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

from service.planner import ENVELOPE_TOLERANCE_MM

__all__ = [
    "EnvelopeEstimate",
    "envelope_tolerance_mm",
    "estimate_envelope_tolerance_mm",
    "envelope_estimate_details",
]


# --------------------------------------------------------------------------- #
# 逐机器人解析
# --------------------------------------------------------------------------- #


def envelope_tolerance_mm(robot: Optional[object] = None) -> float:
    """解析某台机器人应使用的包络接触容差 (mm)。

    解析顺序：``robot.envelope_tolerance_mm``（逐机覆盖，未标定为 ``None``）
    → :data:`service.planner.ENVELOPE_TOLERANCE_MM`（全局默认）。鸭子类型友好：
    没有该属性的对象（测试桩/旧代码）直接走全局值。
    """
    if robot is not None:
        per = getattr(robot, "envelope_tolerance_mm", None)
        if per is not None:
            return float(per)
    return float(ENVELOPE_TOLERANCE_MM)


# --------------------------------------------------------------------------- #
# 几何估算
# ---------------------------------------------------------------------------


@dataclass
class EnvelopeEstimate:
    """一次包络容差估算的结果。

    Attributes:
        value_mm: 建议初值 (mm)；无法估算时为 ``None``
        per_link: 逐连杆的外扩量 (mm)，只含有「mesh visual + collision 包络」
            的可比连杆——估算依据全部摊开，不接受黑盒数字
        note: 口径说明（估算是下界初值，上机前需实测复核）
    """

    value_mm: Optional[float]
    per_link: Dict[str, float]
    note: str = ""


def _mesh_stl_bbox(path: Path) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """读 STL（二进制/ASCII）的轴对齐包围盒，返回 (min, max)（原始单位）。

    只取顶点极值，不建拓扑——估算只需要包围盒，10 万面片的模型也是一次
    ``frombuffer`` 的事。
    """
    try:
        raw = path.read_bytes()
    except OSError:
        return None
    if len(raw) < 84:
        return None
    # 二进制 STL：80 字节头 + uint32 三角形数 + 每三角形 50 字节
    n_tri = int(np.frombuffer(raw[80:84], dtype="<u4")[0])
    if 84 + n_tri * 50 == len(raw) and n_tri > 0:
        arr = np.frombuffer(
            raw[84 : 84 + n_tri * 50],
            dtype=np.dtype(
                [
                    ("normal", "<f4", (3,)),
                    ("verts", "<f4", (3, 3)),
                    ("attr", "<u2"),
                ]
            ),
        )
        pts = arr["verts"].reshape(-1, 3).astype(np.float64)
        return pts.min(axis=0), pts.max(axis=0)
    # ASCII STL：逐行抠 vertex
    verts = []
    try:
        text = raw.decode("ascii", errors="ignore")
    except Exception:
        return None
    for line in text.splitlines():
        parts = line.strip().split()
        if len(parts) == 4 and parts[0] == "vertex":
            try:
                verts.append([float(v) for v in parts[1:4]])
            except ValueError:
                return None
    if not verts:
        return None
    pts = np.asarray(verts, dtype=np.float64)
    return pts.min(axis=0), pts.max(axis=0)


def _mesh_stl_points(path: Path) -> Optional[np.ndarray]:
    """读 STL（二进制/ASCII）的全部顶点 (N,3)（原始单位）；失败返回 None。

    估算需要**真实顶点**而不是包围盒——包围盒角点会虚增径向距离，把外扩
    低估成 0（盒角 vs 管壁的穿透恰恰发生在包围盒覆盖不到的凹陷处）。
    """
    try:
        raw = path.read_bytes()
    except OSError:
        return None
    if len(raw) < 84:
        return None
    n_tri = int(np.frombuffer(raw[80:84], dtype="<u4")[0])
    if n_tri > 0 and 84 + n_tri * 50 == len(raw):
        arr = np.frombuffer(
            raw[84 : 84 + n_tri * 50],
            dtype=np.dtype(
                [
                    ("normal", "<f4", (3,)),
                    ("verts", "<f4", (3, 3)),
                    ("attr", "<u2"),
                ]
            ),
        )
        return arr["verts"].reshape(-1, 3).astype(np.float64)
    try:
        text = raw.decode("ascii", errors="ignore")
    except Exception:
        return None
    verts = []
    for line in text.splitlines():
        parts = line.strip().split()
        if len(parts) == 4 and parts[0] == "vertex":
            try:
                verts.append([float(v) for v in parts[1:4]])
            except ValueError:
                return None
    return np.asarray(verts, dtype=np.float64) if verts else None


def _rpy_matrix(rpy: Tuple[float, float, float]) -> np.ndarray:
    """URDF rpy（固定轴 XYZ 欧拉角）→ 旋转矩阵。"""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _origin(el: Optional[ET.Element]) -> Tuple[np.ndarray, np.ndarray]:
    """解析 <origin> 的 xyz 与 rpy（缺省 0）。"""
    if el is None:
        return np.zeros(3), np.zeros(3)
    xyz = np.asarray([float(v) for v in (el.get("xyz") or "0 0 0").split()], dtype=np.float64)
    rpy = np.asarray([float(v) for v in (el.get("rpy") or "0 0 0").split()], dtype=np.float64)
    return xyz, rpy


def _urdf_scale(root: ET.Element, urdf_dir: Path) -> float:
    """长度单位 → mm。URDF 规范是米；个别工具产出毫米 URDF（数值大 1000 倍），
    按最大 mesh 尺寸粗判一次，避免把毫米模型再放大 1000 倍。"""
    for mesh in root.iter("mesh"):
        if not str(mesh.get("filename", "")).lower().endswith(".stl"):
            continue
        bbox = _mesh_stl_bbox(_resolve_mesh(urdf_dir, mesh.get("filename")))
        if bbox is not None:
            span = float((bbox[1] - bbox[0]).max())
            return 1000.0 if span < 5.0 else 1.0  # <5 个长度单位 → 米
    return 1000.0  # 规范默认：米


def _resolve_mesh(urdf_dir: Path, filename: str) -> Path:
    p = Path(filename)
    if p.is_absolute() and p.is_file():
        return p
    cand = urdf_dir / p
    if cand.is_file():
        return cand
    # <mesh filename="meshes/x.stl"> 是相对 URDF 目录的常见写法；也兼容
    # package:// 前缀被剥掉后落在同级的布局
    name = p.name
    alt = urdf_dir.parent / name
    return alt if alt.is_file() else cand


def _link_mesh_points(link: ET.Element, urdf_dir: Path, scale: float) -> Optional[np.ndarray]:
    """收集一个连杆全部 visual mesh 的顶点，摆进连杆坐标系 (mm)。

    只留顶点不建拓扑；没有可比 mesh 时返回 ``None``。
    """
    chunks = []
    for vis in link.findall("visual"):
        geom = vis.find("geometry")
        mesh = geom.find("mesh") if geom is not None else None
        if mesh is None or not str(mesh.get("filename", "")).lower().endswith(".stl"):
            continue
        pts_raw = _mesh_stl_points(_resolve_mesh(urdf_dir, mesh.get("filename")))
        if pts_raw is None or len(pts_raw) == 0:
            continue
        xyz, rpy = _origin(vis.find("origin"))
        R = _rpy_matrix(tuple(rpy))
        chunks.append(pts_raw @ R.T + xyz)
    if not chunks:
        return None
    return np.vstack(chunks) * scale


def _slice_mask(local: np.ndarray, half: float, eps_mm: float = 0.01) -> Optional[np.ndarray]:
    """包络体轴向切片掩码：|z| ≤ half + eps。

    STL 顶点是 float32——恰好落在包络体端面上的顶点会因舍入差几个微米被
    切片漏掉，所以留 0.01mm 容差；仍没有顶点时回退**全部**顶点（r_mesh 只会
    更大 → 外扩只会更小），估算始终朝着"少放行"的反方向偏，安全。
    """
    mask = np.abs(local[:, 2]) <= half + eps_mm
    if mask.any():
        return mask
    return np.ones(len(local), dtype=bool) if len(local) else None


def envelope_estimate_details(urdf_path: str) -> Dict[str, dict]:
    """逐连杆拆解包络外扩估算（:func:`estimate_envelope_tolerance_mm` 的依据）。

    口径：对每个 ``<collision>`` 包络体，把该连杆的 mesh 顶点变换进**包络体
    坐标系**，在包络体跨度内取 mesh 最大径向距离 ``r_mesh``；包络体横截面
    角点半径 ``r_corner``（圆柱/盒都是 ``√(hx²+hy²)``, 球是 ``r``）比它多出
    的部分就是该处可能的**最大穿透**。逐连杆取最大，全部依据摊开、不接受
    黑盒数字。

    Returns:
        ``{link: {"expansion_mm": float, "detail": str}}``
    """
    urdf = Path(urdf_path)
    root = ET.parse(urdf).getroot()
    urdf_dir = urdf.parent
    scale = _urdf_scale(root, urdf_dir)

    details: Dict[str, dict] = {}
    for link in root.iter("link"):
        pts = _link_mesh_points(link, urdf_dir, scale)
        if pts is None or len(pts) == 0:
            continue

        best = None  # (expansion, 描述)
        for col in link.findall("collision"):
            geom = col.find("geometry")
            if geom is None:
                continue
            xyz, rpy = _origin(col.find("origin"))
            t = xyz * scale
            R = _rpy_matrix(tuple(rpy))
            local = (pts - t) @ R  # 顶点 → 包络体坐标系
            if geom.find("sphere") is not None:
                r = float(geom.find("sphere").get("radius", 0.0)) * scale
                r_mesh = float(np.linalg.norm(local, axis=1).max()) if len(local) else 0.0
                exp, desc = max(0.0, r - r_mesh), f"sphere r={r:.1f}"
            elif (cyl := geom.find("cylinder")) is not None:
                r = float(cyl.get("radius", 0.0)) * scale
                h = float(cyl.get("length", 0.0)) * scale
                in_slice = _slice_mask(local, h / 2.0)
                if in_slice is None:
                    continue
                r_mesh = float(np.hypot(local[in_slice, 0], local[in_slice, 1]).max())
                exp, desc = max(0.0, r - r_mesh), f"cylinder r={r:.1f}"
            elif (box := geom.find("box")) is not None:
                size = np.asarray([float(v) for v in box.get("size", "0 0 0").split()]) * scale
                hx, hy, hz = size / 2.0
                in_slice = _slice_mask(local, hz)
                if in_slice is None:
                    continue
                r_mesh = float(np.hypot(local[in_slice, 0], local[in_slice, 1]).max())
                r_corner = float(math.hypot(hx, hy))
                exp, desc = max(0.0, r_corner - r_mesh), f"box {hx*2:.1f}×{hy*2:.1f}"
            else:
                continue  # mesh 形状的 collision：与 mesh 同源，没有"外扩"可言
            if best is None or exp > best[0]:
                best = (exp, desc)
        if best is None:
            continue
        details[link.get("name", "?")] = {
            "expansion_mm": round(float(best[0]), 2),
            "detail": best[1],
        }
    return details


def estimate_envelope_tolerance_mm(
    urdf_path: str,
    *,
    max_value: float = 100.0,
) -> Optional[float]:
    """按 URDF 几何估算该机器人的包络接触容差初值 (mm)。

    方法：逐连杆比较 ``<collision>`` 包络体与 ``<visual>`` mesh 的包围盒，
    取「包络角点半径（含中心偏移）− mesh 角点半径」的最大值。这个指标对
    UR5e 的"盒角比圆管外扩"给出 ≈11.6mm，与量纲直觉一致。

    Args:
        urdf_path: URDF 文件路径（visual 需是 STL mesh、collision 需是
            box/cylinder/sphere 原语，两者都有才可比）
        max_value: 结果上限 (mm)——估算只该当**初值**，封顶防止异常几何
            （坏 mesh、单位搞错）给出荒谬容差把真撞放掉

    Returns:
        建议初值 (mm，一位小数)；没有任何可比连杆时 ``None``
        （调用方应回退全局默认并提示人工标定）

    Note:
        **这是几何下界**：UR5e 实测几何下界只有 0–4mm，而抓取工作站实测
        最深包络穿透 18.6mm——穿透发生在 mesh 凹陷处与姿态相关的接触位置，
        纯几何法测不到。正确用法：①品牌间包络"胖瘦"横向对比（找最胖的
        连杆）；②手工修改包络体后的回归检查；③给新品牌的**起步参考**，
        上机前必须实测复核并取 ``max(几何下界, 实测值)``。
    """
    details = envelope_estimate_details(urdf_path)
    if not details:
        return None
    raw = max(v["expansion_mm"] for v in details.values())
    return round(min(raw, float(max_value)), 1)
