#!/usr/bin/env python3
"""把厂商官网抓到的真机 STEP 装配体，接成可挂进仿真/OLP 的 URDF。

为什么单独做一步
----------------
:mod:`vendor_cad_import` 只把 STEP 转成「按顶层组件分组的 STL」（``grp_NN.stl``），
每个网格处在世界坐标系、姿态是厂商出厂 CAD 的折叠姿态，并没有和 URDF 的连杆链
对上。本工具补上这最后一步：

1. 用 ``TopoDS_Iterator`` 按 base→tip 顺序拿到顶层「已烘焙定位」的组件形状（与
   ``vendor_cad_import`` 完全同坐标系，保证和已渲染的 ``grp_*.stl`` 严格对齐）。
2. 几何按各品牌的轴系约定（JAKA 是 Y-up mm → Z-up m 换轴+缩放；KUKA/ABB/FANUC/
   Estun 等工业 CAD 多为 Z-up mm）落到世界系。
3. 运动学（关节轴 / 限位 / 父子关系）**直接复用已验证的程序化 URDF**——那是对
   的真实参数；只把每个关节原点改成 CAD 实际姿态算出来的旋转中心，让真机外形
   与既有运动学对齐。
4. 每个连杆网格存成「相对本连杆坐标系」的 STL（顶点减去旋转中心），写进
   ``models/<brand>/<model>/meshes_real/``，并生成 ``<model>_real.urdf``。

这样产出的机器人：外形是真机的，关节轴/限位是验证过的，零位姿态就是 CAD 出厂
姿态，且能正确摆动（每个关节绕真实的连杆连接中心旋转）。

多品牌：所有品牌共用同一套代码，差异只在 ``BRANDS`` 配置表（轴系约定、目录、
已验证标记）。新品牌只需在表里加一行。运动学来源是 ``models/<brand>/<model>/
<model>.urdf``（程序化模型）。没有 STEP 时可用 ``--probe`` 先看组件数与包围盒，
用来确认轴系约定。

依赖：``cadquery-ocp``（OpenCASCADE 绑定）。

用法::

    python3 tools/build_vendor_urdf.py --brand jaka --all
    python3 tools/build_vendor_urdf.py --brand kuka --model kr6_r900
    python3 tools/build_vendor_urdf.py --brand kuka --probe      # 先看 STEP 结构
    python3 tools/build_vendor_urdf.py --all                     # 所有有 STEP 的品牌
"""

from __future__ import annotations

import argparse
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
VENDOR_STEP = REPO_ROOT / "assets" / "vendor"  # gitignored，可重新生成
MODELS_DIR = REPO_ROOT / "models"

# 复用库侧的 OCP 版本兼容层（不自己写一套 getattr）——它同时也是「哪一代绑定
# 该用哪个符号名」的唯一出处，见 python/orolp/ocpcompat.py 的模块说明。
if str(REPO_ROOT / "python") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "python"))

from orolp.ocpcompat import bbox_of, describe, face_of, triangulation_of  # noqa: E402


# --------------------------------------------------------------------------- #
# 品牌配置表
# --------------------------------------------------------------------------- #
@dataclass
class BrandSpec:
    key: str
    label: str            # 厂商名（也用于 STEP 文件名归一化匹配）
    model_dir: str        # models/<model_dir>/<model>/<model>.urdf
    step_subdir: str      # assets/vendor/<step_subdir>/*.STEP
    coord: str = "z_up_mm"   # y_up_mm | z_up_mm | z_up_m
    verified: bool = False    # 是否已用真实 STEP 验证过轴系/组件映射
    note: str = ""


# 轴系约定说明：
#   y_up_mm —— 源 Y-up（如 JAKA 出厂 CAD），需 YZ 换轴到 Z-up，mm→m 缩放 0.001
#   z_up_mm —— 源 Z-up（多数工业 CAD），仅 mm→m 缩放 0.001
#   z_up_m  —— 源 Z-up 且单位已是 m，无缩放
BRANDS: Dict[str, BrandSpec] = {
    "jaka":  BrandSpec("jaka",  "JAKA",  "jaka",  "jaka/step",  coord="y_up_mm", verified=True,
                       note="已验证：7 型号全过，Y-up mm→Z-up m；组件即连杆（base+link_1..5+tool0），多出的并入 tool0"),
    "kuka":  BrandSpec("kuka",  "KUKA",  "kuka",  "kuka/step",  coord="z_up_mm", verified=False,
                       note="待验证：KUKA CAD 多为 Z-up mm；组件映射待 STEP 到位后调"),
    "abb":   BrandSpec("abb",   "ABB",   "abb",   "abb/step",   coord="z_up_mm", verified=False,
                       note="待验证：ABB CAD 多为 Z-up mm；组件映射待 STEP 到位后调"),
    "fanuc": BrandSpec("fanuc", "FANUC", "fanuc", "fanuc/step", coord="z_up_mm", verified=False,
                       note="待验证：FANUC CAD 多为 Z-up mm；组件映射待 STEP 到位后调"),
    "estun": BrandSpec("estun", "ESTUN", "estun", "estun/step", coord="z_up_mm", verified=False,
                       note="待验证：Estun CAD 多 Z-up mm；组件映射待 STEP 到位后调"),
}


# --------------------------------------------------------------------------- #
# 几何变换（与 vendor_cad_import.convert 一致的轴系/单位约定）
# --------------------------------------------------------------------------- #
def _coord_trsf(coord: str):
    """按品牌轴系约定，返回 源→Z-up m 的 gp_Trsf。"""
    from OCP.gp import gp_Trsf, gp_Pnt, gp_Dir, gp_Ax3

    rot = gp_Trsf()
    if coord == "y_up_mm":
        # 源 Y-up → 目标 Z-up。实际点映射是 (x,y,z)→(x,z,-y)（绕 X 轴 −90°）：
        # 源的竖直轴 Y 落到目标的竖直轴 Z 上、方向相反，右手系保持（det>0，不是镜像）。
        # 轴符号由已验证的渲染/生成结果决定，锁住它的不是硬编码符号而是
        # test_vendor_jaka.test_coord_trsf_yup_is_proper_uniform 的不变量
        # （均匀缩放 0.001 + 保持手性）。
        src_ax = gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0), gp_Dir(1, 0, 0))
        dst_ax = gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0))
        rot.SetTransformation(src_ax, dst_ax)
    elif coord in ("z_up_mm", "z_up_m"):
        pass  # 已是 Z-up，无需换轴
    else:
        raise ValueError(f"未知 coord: {coord}")

    scale = gp_Trsf()
    scale.SetScaleFactor(0.001 if coord.endswith("mm") else 1.0)
    return scale * rot


# --------------------------------------------------------------------------- #
# STEP → 组件（与 vendor_cad_import.convert 完全一致的 TopoDS_Iterator 路径）
# --------------------------------------------------------------------------- #
def load_step_components(step_path: Path, trsf) -> List[Tuple[str, "TopoDS_Shape"]]:
    """读取 STEP，返回 (组件名, 已变换到世界系的形状) 列表，base→tip 顺序。

    与 ``vendor_cad_import.convert`` **完全一致**的做法：先对整个 root 施加
    轴系+缩放变换，再用 ``TopoDS_Iterator`` 取顶层「已烘焙」形状。这样得到的几何
    与已验证、已渲染的 ``grp_*.stl`` 严格同坐标系。
    """
    from OCP.STEPControl import STEPControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.TopoDS import TopoDS_Iterator
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform

    reader = STEPControl_Reader()
    if reader.ReadFile(str(step_path)) != IFSelect_RetDone:
        raise SystemExit(f"STEP 读取失败: {step_path}")
    reader.TransferRoots()
    root = reader.OneShape()
    root = BRepBuilderAPI_Transform(root, trsf, True).Shape()

    comps: List[Tuple[str, "TopoDS_Shape"]] = []
    it = TopoDS_Iterator(root)
    idx = 0
    while it.More():
        sh = it.Value()
        comps.append((f"part_{idx:02d}", sh))
        idx += 1
        it.Next()
    return comps


# --------------------------------------------------------------------------- #
# 网格化（OpenCASCADE 内置 IncrementalMesh → 取三角形顶点）
# --------------------------------------------------------------------------- #
def mesh_component(shape, rel_deflection: float = 0.01, angular: float = 0.5) -> np.ndarray:
    """把一个已定位形状三角化，返回 (N*3,3) 的世界系顶点数组（Z-up m）。

    注意：形状传入前应已施加轴系+缩放变换（见 ``load_step_components``），
    本函数不再做坐标变换。
    """
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopLoc import TopLoc_Location

    BRepMesh_IncrementalMesh(shape, rel_deflection, True, angular, True)
    verts: List[np.ndarray] = []
    fx = TopExp_Explorer(shape, TopAbs_FACE)
    while fx.More():
        face = face_of(fx.Current())
        loc = TopLoc_Location()
        tri = triangulation_of(face, loc)
        if tri is None:
            fx.Next()
            continue
        for k in range(1, tri.NbTriangles() + 1):
            t = tri.Triangles().Value(k)
            i1, i2, i3 = t.Get()
            for idx in (i1, i2, i3):
                p = tri.Node(idx).Transformed(loc.Transformation())
                verts.append([p.X(), p.Y(), p.Z()])
        fx.Next()
    return np.asarray(verts, dtype=np.float64).reshape(-1, 3)


def write_stl_binary(path: Path, verts: np.ndarray) -> None:
    """把 (N,3) 顶点（每 3 个一组为一个三角面）写成二进制 STL。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    n_tri = verts.shape[0] // 3
    dt = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    arr = np.zeros(n_tri, dtype=dt)
    v = verts.reshape(n_tri, 3, 3).astype("<f4")
    # 计算法向（用于正确缠绕方向的渲染器）
    n1 = v[:, 1] - v[:, 0]
    n2 = v[:, 2] - v[:, 0]
    norm = np.cross(n1, n2)
    ln = np.linalg.norm(norm, axis=1, keepdims=True)
    ln[ln == 0] = 1.0
    arr["normal"] = (norm / ln).astype("<f4")
    arr["v"] = v
    with open(path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(np.uint32(n_tri).tobytes())
        f.write(arr.tobytes())


# --------------------------------------------------------------------------- #
# 程序化 URDF 解析（取运动学骨架）
# --------------------------------------------------------------------------- #
@dataclass
class Kinematics:
    """从程序化 URDF 取出的运动学骨架。"""
    bodies: List[str] = field(default_factory=list)      # base→tip 的刚体顺序
    joints: Dict[str, dict] = field(default_factory=dict)  # joint名→{parent,child,axis,limit,xyz}
    tcp_offset: Optional[List[float]] = None


_JOINT_RE = re.compile(r"<joint\s+name=\"([^\"]+)\"")
_LINK_RE = re.compile(r"<link\s+name=\"([^\"]+)\"")
_PARENT_RE = re.compile(r"<parent\s+link=\"([^\"]+)\"")
_CHILD_RE = re.compile(r"<child\s+link=\"([^\"]+)\"")
_AXIS_RE = re.compile(r"<axis\s+xyz=\"([^\"]+)\"")
_XYZ_RE = re.compile(r"<origin\s+xyz=\"([^\"]+)\"")
_LIMIT_RE = re.compile(r"<limit\s+([^>]*)>")
_TYPE_RE = re.compile(r"<joint\s+name=\"[^\"]+\"\s+type=\"([^\"]+)\"")


def parse_kinematics(urdf_path: Path) -> Kinematics:
    txt = urdf_path.read_text(encoding="utf-8")
    links = _LINK_RE.findall(txt)
    kin = Kinematics(bodies=[l for l in links if l != "tcp"])
    # 逐 joint 块解析
    joint_blocks = re.findall(r"<joint\b[^>]*>.*?</joint>", txt, re.S)
    for blk in joint_blocks:
        name = _JOINT_RE.search(blk)
        if not name:
            continue
        jname = name.group(1)
        p = _PARENT_RE.search(blk)
        c = _CHILD_RE.search(blk)
        ax = _AXIS_RE.search(blk)
        xyz = _XYZ_RE.search(blk)
        lim = _LIMIT_RE.search(blk)
        jtype = _TYPE_RE.search(blk)
        d = {
            "parent": p.group(1) if p else "",
            "child": c.group(1) if c else "",
            "axis": [float(x) for x in ax.group(1).split()] if ax else [0, 0, 1],
            "xyz": [float(x) for x in xyz.group(1).split()] if xyz else [0, 0, 0],
            "type": jtype.group(1) if jtype else "revolute",
        }
        if lim:
            for kv in re.findall(r'(\w+)="([^"]+)"', lim.group(1)):
                if kv[0] in ("lower", "upper", "effort", "velocity"):
                    d[kv[0]] = float(kv[1])
        kin.joints[jname] = d
        if jname == "tcp_joint" or (c and c.group(1) == "tcp"):
            kin.tcp_offset = d["xyz"]
    return kin


# --------------------------------------------------------------------------- #
# 骨架重建：每个刚体的旋转中心
# --------------------------------------------------------------------------- #
def compute_frames(centroids: List[np.ndarray]) -> List[np.ndarray]:
    """返回每个刚体坐标系原点（世界系）。

    base(0) 原点取世界原点；第 i 个刚体的原点 = 其网格中离第 i-1 个刚体质心
    最近的顶点 —— 即两个连杆的实际连接处，作为关节旋转中心。
    """
    n = len(centroids)
    origins = [np.zeros(3)]
    verts_cache = compute_frames._verts  # type: ignore[attr-defined]
    for i in range(1, n):
        prev_c = centroids[i - 1]
        v = verts_cache[i]
        d = np.linalg.norm(v - prev_c, axis=1)
        origins.append(v[int(np.argmin(d))].copy())
    return origins


# --------------------------------------------------------------------------- #
# STEP 文件名 ↔ 型号 归一化匹配（解决前缀冲突：zu3 vs zu30、kr6 vs kr60）
# --------------------------------------------------------------------------- #
def _norm(s: str) -> str:
    return re.sub(r"[^a-z0-9]", "", s.lower())


def _match_step(brand_norm: str, model_norm: str, stem_norm: str) -> bool:
    """STEP stem 归一化后，是否对应 (brand, model)。

    要点：model 可能是别的 model 的前缀（zu3⊂zu30），所以用「下一位非数字」来
    卡边界，避免 zu3 误匹配 Zu30。
    """
    pref = brand_norm + model_norm
    if not stem_norm.startswith(pref):
        return False
    nxt = stem_norm[len(pref):len(pref) + 1]
    return nxt == "" or not nxt.isdigit()


def step_path_for(brand: str, model: str) -> Optional[Path]:
    bspec = BRANDS[brand]
    brand_norm = _norm(bspec.label)
    model_norm = _norm(model)
    cands = []
    sub = VENDOR_STEP / bspec.step_subdir
    if sub.exists():
        for p in sub.glob("*.STEP"):
            if _match_step(brand_norm, model_norm, _norm(p.stem)):
                cands.append(p)
    return sorted(cands)[0] if cands else None


def model_list(brand: str) -> List[str]:
    """该品牌全部有程序化 URDF 的型号（目录名 == urdf 名）。"""
    bspec = BRANDS[brand]
    out = []
    root = MODELS_DIR / bspec.model_dir
    if root.exists():
        for d in sorted(root.iterdir()):
            if d.is_dir() and (d / f"{d.name}.urdf").exists():
                out.append(d.name)
    return out


def models_with_step(brand: str) -> List[str]:
    return [m for m in model_list(brand) if step_path_for(brand, m) is not None]


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #
def probe_step(brand: str, model: Optional[str]) -> int:
    """打印 STEP 的组件数与包围盒（应用品牌轴系约定后），用于确认 coord/映射。"""
    bspec = BRANDS[brand]
    if model:
        steps = [step_path_for(brand, model)]
        if steps[0] is None:
            print(f"[probe] {brand}/{model}: 未找到 STEP")
            return 1
    else:
        sub = VENDOR_STEP / bspec.step_subdir
        steps = sorted(sub.glob("*.STEP")) if sub.exists() else []
        if not steps:
            print(f"[probe] {brand}: {sub} 下没有 STEP（先放厂商 CAD 进来）")
            return 1

    info = describe()
    print(f"[probe] OCP: {info['distribution']} {info['version']}  "
          f"TopoDS.{info['casts']['Face']}  BRep_Tool.{info['functions']['Triangulation']}")

    trsf = _coord_trsf(bspec.coord)
    for step in steps:
        comps = load_step_components(step, trsf)
        print(f"\n=== probe {brand}: {step.name} ===")
        print(f"  coord={bspec.coord}  组件数={len(comps)}")
        allmin = np.full(3, np.inf)
        allmax = np.full(3, -np.inf)
        for name, sh in comps:
            bb = bbox_of(sh)
            if bb is None:
                continue
            lo, hi = bb
            allmin = np.minimum(allmin, lo)
            allmax = np.maximum(allmax, hi)
            print(f"  {name}: x[{lo[0]:+.3f},{hi[0]:+.3f}] "
                  f"y[{lo[1]:+.3f},{hi[1]:+.3f}] z[{lo[2]:+.3f},{hi[2]:+.3f}]")
        print(f"  总包围盒: x[{allmin[0]:+.3f},{allmax[0]:+.3f}] "
              f"y[{allmin[1]:+.3f},{allmax[1]:+.3f}] z[{allmin[2]:+.3f},{allmax[2]:+.3f}]")
    return 0


def build_model(brand: str, model: str, rel_deflection: float, angular: float) -> int:
    bspec = BRANDS[brand]
    step = step_path_for(brand, model)
    if step is None:
        print(f"[skip] {brand}/{model}: 找不到 STEP（放进 {VENDOR_STEP / bspec.step_subdir}）")
        return 1
    proc_urdf = MODELS_DIR / bspec.model_dir / model / f"{model}.urdf"
    if not proc_urdf.exists():
        print(f"[skip] {brand}/{model}: 没有对应的程序化 URDF {proc_urdf}")
        return 1

    kin = parse_kinematics(proc_urdf)
    print(f"\n=== {brand}/{model} ===  刚体顺序: {kin.bodies}")
    print(f"  关节: " + ", ".join(
        f"{j}:{d['axis']}" for j, d in kin.joints.items() if d['type'] != 'fixed'))

    trsf = _coord_trsf(bspec.coord)
    comps = load_step_components(step, trsf)
    print(f"  STEP 组件数: {len(comps)}  -> {[c[0] for c in comps]}")

    n_bodies = len(kin.bodies)
    if len(comps) < n_bodies:
        print(f"  [warn] 组件数 {len(comps)} < 刚体数 {n_bodies}，跳过")
        return 1

    meshes: List[np.ndarray] = []
    centroids: List[np.ndarray] = []
    for name, sh in comps[:n_bodies]:
        v = mesh_component(sh, rel_deflection, angular)
        meshes.append(v)
        centroids.append(v.mean(axis=0))
        print(f"    {name:8} tris={v.shape[0]//3:<7} centroid=({centroids[-1][0]:+.3f},"
              f"{centroids[-1][1]:+.3f},{centroids[-1][2]:+.3f})")

    compute_frames._verts = meshes  # 供 compute_frames 用
    origins = compute_frames(centroids)

    # 额外的组件（如 Zu18 的第 8 个）追加到最后一个刚体
    extra_v = []
    for name, sh in comps[n_bodies:]:
        extra_v.append(mesh_component(sh, rel_deflection, angular))
    if extra_v:
        print(f"  [info] {len(extra_v)} 个额外组件并入 {kin.bodies[-1]}")

    out_dir = MODELS_DIR / bspec.model_dir / model / "meshes_real"
    for i, body in enumerate(kin.bodies):
        rel = meshes[i] - origins[i]
        write_stl_binary(out_dir / f"{body}.stl", rel)
    for k, ev in enumerate(extra_v):
        rel = ev - origins[-1]
        write_stl_binary(out_dir / f"{kin.bodies[-1]}_extra{k}.stl", rel)

    write_urdf(MODELS_DIR / bspec.model_dir / model / f"{model}_real.urdf",
               brand, model, kin, origins, out_dir, meshes, extra_v)
    return 0


def write_urdf(path: Path, brand: str, model: str, kin: Kinematics,
               origins: List[np.ndarray], out_dir: Path,
               meshes: List[np.ndarray], extra_v: List[np.ndarray]) -> None:
    chain = kin.bodies
    joint_by_child = {d["child"]: (j, d) for j, d in kin.joints.items()}
    robot_name = f"{brand}_{model}_real"

    lines = [f'<?xml version="1.0" encoding="utf-8"?>',
             f'<robot name="{robot_name}">',
             f'  <!-- {BRANDS[brand].label} {model} — 真机 CAD（厂商官网 STEP，'
             f'{Path(__file__).name} 生成，coord={BRANDS[brand].coord}）。'
             f'几何为真实外形，运动学（轴/限位）复用程序化 URDF。 -->']

    materials = {
        "metal": "0.80 0.82 0.85 1",
        "accent": "0.20 0.55 0.85 1",
        "dark": "0.45 0.47 0.50 1",
        "jointcap": "0.85 0.86 0.88 1",
    }

    for i, body in enumerate(chain):
        lines.append(f'  <link name="{body}">')
        lines.append('    <visual>')
        lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append(f'      <geometry><mesh filename="meshes_real/{body}.stl"/></geometry>')
        lines.append(f'      <material name="{body}_m"><color rgba="{materials["metal"]}"/></material>')
        lines.append('    </visual>')
        if i == len(chain) - 1:
            for k in range(len(extra_v)):
                lines.append('    <visual>')
                lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                lines.append(f'      <geometry><mesh filename="meshes_real/{body}_extra{k}.stl"/></geometry>')
                lines.append(f'      <material name="{body}_ex{k}"><color rgba="{materials["dark"]}"/></material>')
                lines.append('    </visual>')
        lines.append('    <collision>')
        lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append(f'      <geometry><mesh filename="meshes_real/{body}.stl"/></geometry>')
        lines.append('    </collision>')
        lines.append('  </link>')

    if kin.tcp_offset is not None:
        lines.append('  <link name="tcp">')
        lines.append('    <visual>')
        lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append('      <geometry><sphere radius="0.008"/></geometry>')
        lines.append('      <material name="tcp_m"><color rgba="0.15 0.85 0.55 1"/></material>')
        lines.append('    </visual>')
        lines.append('  </link>')

    for i in range(1, len(chain)):
        child = chain[i]
        entry = joint_by_child.get(child)
        if entry is None:
            continue
        jname, d = entry
        origin_xyz = origins[i] - origins[i - 1]
        ax = " ".join(f"{x:g}" for x in d["axis"])
        lines.append(f'  <joint name="{jname}" type="{d["type"]}">')
        lines.append(f'    <parent link="{d["parent"]}"/>')
        lines.append(f'    <child link="{child}"/>')
        lines.append(f'    <origin xyz="{origin_xyz[0]:g} {origin_xyz[1]:g} {origin_xyz[2]:g}" rpy="0 0 0"/>')
        lines.append(f'    <axis xyz="{ax}"/>')
        if d["type"] != "fixed":
            lo = d.get("lower", -6.283185)
            hi = d.get("upper", 6.283185)
            eff = d.get("effort", 58)
            vel = d.get("velocity", 3.141593)
            lines.append(f'    <limit lower="{lo:g}" upper="{hi:g}" effort="{eff:g}" velocity="{vel:g}"/>')
        lines.append('  </joint>')

    if kin.tcp_offset is not None:
        off = " ".join(f"{x:g}" for x in kin.tcp_offset)
        lines.append('  <joint name="tcp_joint" type="fixed">')
        lines.append('    <parent link="tool0"/>')
        lines.append('    <child link="tcp"/>')
        lines.append(f'    <origin xyz="{off}" rpy="0 0 0"/>')
        lines.append('  </joint>')

    lines.append('</robot>')
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    total = sum(meshes[i].shape[0] // 3 for i in range(len(meshes)))
    size = sum((out_dir / f"{b}.stl").stat().st_size for b in chain) / 1024
    print(f"  写出 {path.name}: {len(chain)} 连杆 / {total} 三角面 / {size:.0f} KB 网格")


def main() -> int:
    ap = argparse.ArgumentParser(description="把厂商真机 STEP 接成 URDF（多品牌）")
    ap.add_argument("--brand", default="jaka",
                    help="品牌键，见 BRANDS：jaka/kuka/abb/fanuc/estun（默认 jaka）")
    ap.add_argument("--model", help="型号，如 zu3 / kr6_r900")
    ap.add_argument("--all", action="store_true",
                    help="对该品牌所有「有 STEP」的型号，或（无 --brand 之外的）所有品牌")
    ap.add_argument("--probe", action="store_true",
                    help="只探查 STEP 的组件数与包围盒，不生成 URDF")
    ap.add_argument("--rel-deflection", type=float, default=0.01,
                    help="相对弦高（默认 0.01，越小越细）")
    ap.add_argument("--angular-deflection", type=float, default=0.5)
    args = ap.parse_args()

    if args.brand not in BRANDS:
        ap.error(f"未知品牌 {args.brand}，可选: {', '.join(BRANDS)}")

    if args.probe:
        return probe_step(args.brand, args.model)

    if args.all:
        if args.brand == "jaka" and not args.model:
            # 默认 --all 不指品牌时，跑所有有 STEP 的品牌
            brands = [b for b in BRANDS if models_with_step(b)]
        else:
            brands = [args.brand]
        models = [(b, m) for b in brands for m in models_with_step(b)]
    elif args.model:
        models = [(args.brand, args.model)]
    else:
        ap.print_help()
        return 0

    if not models:
        print("没有找到任何「型号 + STEP」组合。把厂商 STEP 放进 "
              f"{VENDOR_STEP}/<brand>/step/ 后再试。")
        return 1

    rc = 0
    for b, m in models:
        rc |= build_model(b, m, args.rel_deflection, args.angular_deflection)
    return rc


if __name__ == "__main__":
    sys.exit(main())
