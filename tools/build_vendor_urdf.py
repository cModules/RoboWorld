#!/usr/bin/env python3
"""把厂商官网抓到的真机 STEP 装配体，接成可挂进仿真/OLP 的 URDF。

为什么单独做一步
----------------
:mod:`vendor_cad_import` 只把 STEP 转成「按顶层组件分组的 STL」（``grp_NN.stl``），
每个网格处在世界坐标系、姿态是厂商出厂 CAD 的折叠姿态，并没有和 URDF 的连杆链
对上。本工具补上这最后一步：

1. 用 XCAF 读装配树，按 base→tip 顺序拿到 7 个刚体组件（JAKA 是
   base + link_1..5 + tool0，外加一个非物理的 ``tcp`` 标记）。
2. 几何按与 ``vendor_cad_import`` 一致的 Y-up mm → Z-up m 变换，落到世界系。
3. 运动学（关节轴 / 限位 / 父子关系）**直接复用已验证的程序化 URDF**——那是对
   的 JAKA 参数；只把每个关节原点改成 CAD 实际姿态算出来的旋转中心，让真机外形
   与既有运动学对齐。
4. 每个连杆网格存成「相对本连杆坐标系」的 STL（顶点减去旋转中心），写进
   ``models/<brand>/<model>/meshes_real/``，并生成 ``<model>_real.urdf``。

这样产出的机器人：外形是真机的，关节轴/限位是验证过的，零位姿态就是 CAD 出厂
姿态，且能正确摆动（每个关节绕真实的连杆连接中心旋转）。

依赖：``cadquery-ocp``（OpenCASCADE 绑定）。

用法::

    python3 tools/build_vendor_urdf.py --model zu3
    python3 tools/build_vendor_urdf.py --all
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


# --------------------------------------------------------------------------- #
# 几何变换（与 vendor_cad_import.convert 完全一致）
# --------------------------------------------------------------------------- #
def _yz_swap_scale_trsf(scale: float = 0.001):
    from OCP.gp import gp_Trsf, gp_Pnt, gp_Dir, gp_Ax3

    trsf = gp_Trsf()
    src_ax = gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0), gp_Dir(1, 0, 0))  # Y-up
    dst_ax = gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0))  # Z-up
    trsf.SetTransformation(src_ax, dst_ax)
    scale_trsf = gp_Trsf()
    scale_trsf.SetScaleFactor(scale)
    return scale_trsf * trsf


# --------------------------------------------------------------------------- #
# STEP → 组件（XCAF）
# --------------------------------------------------------------------------- #
def load_step_components(step_path: Path, trsf) -> List[Tuple[str, "TopoDS_Shape"]]:
    """读取 STEP，返回 (组件名, 已变换到世界系的形状) 列表，base→tip 顺序。

    与 ``vendor_cad_import.convert`` **完全一致**的做法：先对整个 root 施加
    Y-up mm → Z-up m 的换轴+缩放，再用 ``TopoDS_Iterator`` 取顶层「已烘焙」形状。
    这样得到的几何与已验证、已渲染的 ``grp_*.stl`` 严格同坐标系。
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
def mesh_component(shape, rel_deflection: float = 0.01, angular: float = 0.5,
                   trsf=None) -> np.ndarray:
    """把一个已定位形状三角化，返回 (N*3,3) 的世界系顶点数组（Z-up m）。

    注意：形状传入前应已施加 Y-up→Z-up 的换轴+缩放（见 ``load_step_components``），
    本函数不再做坐标变换。
    """
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopoDS import TopoDS
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE
    from OCP.BRep import BRep_Tool
    from OCP.TopLoc import TopLoc_Location

    BRepMesh_IncrementalMesh(shape, rel_deflection, True, angular, True)
    verts: List[np.ndarray] = []
    fx = TopExp_Explorer(shape, TopAbs_FACE)
    while fx.More():
        face = TopoDS.Face_s(fx.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
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
        if jname == "tcp_joint" or c and c.group(1) == "tcp":
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
# 主流程
# --------------------------------------------------------------------------- #
def step_path_for(model: str) -> Optional[Path]:
    """按型号精确匹配 STEP 文件。

    文件名形如 ``JAKA_Zu_3-V2.0_20231016.STEP``（型号里有下划线），而内部型号名是
    ``zu3``，直接用 ``capitalize()`` 拼 glob 会把 ``zu3`` 误匹配到 ``Zu30``。这里
    遍历全部 STEP，去掉下划线后精确比对型号数字。
    """
    target = model.replace("_", "")
    for p in VENDOR_STEP.glob("jaka/step/*.STEP"):
        m = re.match(r"JAKA_(Zu[\w]+)-", p.stem)
        if m and m.group(1).lower().replace("_", "").replace("step", "") == target:
            return p
    return None


def model_list() -> List[str]:
    out = []
    for p in sorted(VENDOR_STEP.glob("jaka/step/*.STEP")):
        m = re.match(r"JAKA_(Zu[\w]+)-", p.stem)
        if m:
            # 文件名里常见 Zu_3 / Zu_18step，下划线无意义，"step" 是后缀噪音。
            # 归一化成纯型号：zu3 / zu18，与 models/jaka/ 目录对齐。
            name = m.group(1).lower().replace("_", "").replace("step", "")
            out.append(name)
    # 去重保序
    seen = set()
    uniq = []
    for m in out:
        if m not in seen:
            seen.add(m)
            uniq.append(m)
    return uniq


def build_model(model: str, rel_deflection: float, angular: float) -> int:
    from OCP.gp import gp_Trsf

    step = step_path_for(model)
    if step is None:
        print(f"[skip] {model}: 找不到 STEP（先跑 vendor_cad_import --fetch --convert）")
        return 1
    proc_urdf = MODELS_DIR / "jaka" / model / f"{model}.urdf"
    if not proc_urdf.exists():
        print(f"[skip] {model}: 没有对应的程序化 URDF {proc_urdf}")
        return 1

    kin = parse_kinematics(proc_urdf)
    print(f"\n=== {model} ===  刚体顺序: {kin.bodies}")
    print(f"  关节: " + ", ".join(
        f"{j}:{d['axis']}" for j, d in kin.joints.items() if d['type'] != 'fixed'))

    trsf = _yz_swap_scale_trsf(0.001)
    comps = load_step_components(step, trsf)
    print(f"  XCAF 组件数: {len(comps)}  -> {[c[0] for c in comps]}")

    # 取前 len(kin.bodies) 个组件；多出来的并入最后一个刚体
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

    # 额外的组件（如 Zu18 的第 8 个）追加到 tool0
    extra_v = []
    for name, sh in comps[n_bodies:]:
        extra_v.append(mesh_component(sh, rel_deflection, angular))
    if extra_v:
        print(f"  [info] {len(extra_v)} 个额外组件并入 {kin.bodies[-1]}")

    # 写出 link-local STL
    out_dir = MODELS_DIR / "jaka" / model / "meshes_real"
    for i, body in enumerate(kin.bodies):
        rel = meshes[i] - origins[i]
        write_stl_binary(out_dir / f"{body}.stl", rel)
    for k, ev in enumerate(extra_v):
        rel = ev - origins[-1]
        write_stl_binary(out_dir / f"{kin.bodies[-1]}_extra{k}.stl", rel)

    # 生成 URDF
    write_urdf(MODELS_DIR / "jaka" / model / f"{model}_real.urdf",
               model, kin, origins, out_dir, meshes, extra_v)
    return 0


def write_urdf(path: Path, model: str, kin: Kinematics, origins: List[np.ndarray],
               out_dir: Path, meshes: List[np.ndarray], extra_v: List[np.ndarray]) -> None:
    # 关节顺序：按父子关系从 base 往下排
    chain = kin.bodies  # base, link_1.., tool0
    # 找到每个 child 对应的 joint
    joint_by_child = {d["child"]: (j, d) for j, d in kin.joints.items()}

    lines = [f'<?xml version="1.0" encoding="utf-8"?>',
             f'<robot name="jaka_{model}_real">',
             f'  <!-- JAKA {model} — 真机 CAD（厂商官网 STEP，{__file__.split("/")[-1]} 生成）。'
             f'几何为真实外形，运动学（轴/限位）复用程序化 URDF。 -->']

    materials = {
        "metal": "0.80 0.82 0.85 1",
        "accent": "0.20 0.55 0.85 1",
        "dark": "0.45 0.47 0.50 1",
        "jointcap": "0.85 0.86 0.88 1",
    }

    # links
    for i, body in enumerate(chain):
        lines.append(f'  <link name="{body}">')
        # 主网格
        lines.append('    <visual>')
        lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append(f'      <geometry><mesh filename="meshes_real/{body}.stl"/></geometry>')
        lines.append(f'      <material name="{body}_m"><color rgba="{materials["metal"]}"/></material>')
        lines.append('    </visual>')
        # 额外组件
        if i == len(chain) - 1:
            for k in range(len(extra_v)):
                lines.append('    <visual>')
                lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
                lines.append(f'      <geometry><mesh filename="meshes_real/{body}_extra{k}.stl"/></geometry>')
                lines.append(f'      <material name="{body}_ex{k}"><color rgba="{materials["dark"]}"/></material>')
                lines.append('    </visual>')
        # 碰撞用网格本身（真机外形，碰撞最准）
        lines.append('    <collision>')
        lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append(f'      <geometry><mesh filename="meshes_real/{body}.stl"/></geometry>')
        lines.append('    </collision>')
        lines.append('  </link>')

    # tcp 标记（固定，非物理）
    if kin.tcp_offset is not None:
        lines.append('  <link name="tcp">')
        lines.append('    <visual>')
        lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append('      <geometry><sphere radius="0.008"/></geometry>')
        lines.append('      <material name="tcp_m"><color rgba="0.15 0.85 0.55 1"/></material>')
        lines.append('    </visual>')
        lines.append('  </link>')

    # joints（base 无 joint；其余按 chain 顺序）
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

    # tcp 固定关节
    if kin.tcp_offset is not None:
        off = " ".join(f"{x:g}" for x in kin.tcp_offset)
        lines.append('  <joint name="tcp_joint" type="fixed">')
        lines.append('    <parent link="tool0"/>')
        lines.append('    <child link="tcp"/>')
        lines.append(f'    <origin xyz="{off}" rpy="0 0 0"/>')
        lines.append('  </joint>')

    lines.append('</robot>')
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    # 体积统计
    total = sum(meshes[i].shape[0] // 3 for i in range(len(meshes)))
    size = sum((out_dir / f"{b}.stl").stat().st_size for b in chain) / 1024
    print(f"  写出 {path.name}: {len(chain)} 连杆 / {total} 三角面 / {size:.0f} KB 网格")


def main() -> int:
    ap = argparse.ArgumentParser(description="把厂商真机 STEP 接成 URDF")
    ap.add_argument("--model", help="型号，如 zu3")
    ap.add_argument("--all", action="store_true", help="全部 jaka 型号")
    ap.add_argument("--rel-deflection", type=float, default=0.01,
                    help="相对弦高（默认 0.01，越小越细）")
    ap.add_argument("--angular-deflection", type=float, default=0.5)
    args = ap.parse_args()

    if args.all:
        models = model_list()
    elif args.model:
        models = [args.model]
    else:
        ap.print_help()
        return 0

    rc = 0
    for m in models:
        rc |= build_model(m, args.rel_deflection, args.angular_deflection)
    return rc


if __name__ == "__main__":
    sys.exit(main())
