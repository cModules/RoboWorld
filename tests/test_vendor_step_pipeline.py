#!/usr/bin/env python3
"""真机 CAD（STEP）→ URDF 全链路的**可执行**验收测试。

为什么需要这个文件
------------------
``tests/test_vendor_jaka.py`` 守的是**已提交产物**（``models/jaka/*/*_real.urdf``）与
工具的纯函数，而生成链路本身挂在「本地没有厂商 STEP 就 skip」的冒烟测试后面。于是
2026-09-14 装上 ``cadquery-ocp 8.0.1``（OCP 8 把 ``TopoDS.Face_s`` 改名为
``TopoDS.Face``）之后，``tools/build_vendor_urdf.py`` 与 ``tools/vendor_cad_import.py``
的网格化分支已经**全线不可用**，却没有任何测试因此变红——skip 是绿的。

本文件用代码**合成**一个多组件 STEP 装配体，把整条链真跑一遍::

    BRepPrimAPI_MakeBox × N → TopoDS_Compound → STEPControl_Writer → *.STEP
      → load_step_components → mesh_component → write_stl_binary
      → build_model → write_urdf → <model>_real.urdf → orolp.Robot
    （另一条分支：vendor_cad_import.convert → grp_NN.stl）

因此不再依赖厂商 CAD 是否投放，改名/接线错误会当场失败。它同时也是 v0.2.3
（5 品牌真机 CAD 接入）在「等 STEP」期间的替代验收：链路一旦可用，剩下要做的
只是把合成 STEP 换成真实 STEP、再调各品牌的组件映射。

覆盖的边界
----------
① OCP 版本兼容层（``orolp.ocpcompat``）：符号解析、软依赖（import 期不加载 OCP）、
   ``Face_s``/``Edge_s`` 改名后工具侧不得再出现裸调用；
② STEP 读写往返：组件数与**顺序**（base→tip）、单位缩放；
③ 网格写出与回读：二进制 STL 头/面数/法线/顶点；
④ 端到端 ``build_model``：连杆链与关节（轴/限位/parent-child）来自程序化 URDF、
   每个关节原点确实是「子网格中离父连杆质心最近的顶点」且与写出的 STL 一致、
   额外组件并入末连杆；
⑤ 生成的 ``_real.urdf`` 能被 ``orolp.Robot`` 真加载（DOF 与 FK 可用）；
⑥ ``vendor_cad_import.convert`` / ``convert_all`` 的分组 STL 产物；
⑦ CLI（``--probe`` / ``--all``）在进程内跑通；
⑧ 仓库级交叉约束：程序化 URDF 的关节 origin rpy 必须全为 0——否则
   ``write_urdf`` 里硬编码的 ``rpy="0 0 0"`` 会静默丢掉真实关节朝向。
"""

from __future__ import annotations

import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
for _p in (str(ROOT), str(ROOT / "python")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

pytest.importorskip("OCP", reason="需要 cadquery-ocp（本文件的合成装配体用 OCP 构建）")

from OCP.BRep import BRep_Builder  # noqa: E402
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox  # noqa: E402
from OCP.IFSelect import IFSelect_RetDone  # noqa: E402
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer  # noqa: E402
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE  # noqa: E402
from OCP.TopExp import TopExp_Explorer  # noqa: E402
from OCP.TopoDS import TopoDS_Compound  # noqa: E402
from OCP.gp import gp_Pnt  # noqa: E402

from orolp import cam, ocpcompat  # noqa: E402
from orolp.robot import Robot  # noqa: E402
from tools import build_vendor_urdf as bv  # noqa: E402
from tools import vendor_cad_import as vc  # noqa: E402

# --------------------------------------------------------------------------- #
# 合成装配体：source 为 Z-up mm（coord="z_up_mm" 只缩放不换轴，断言最直观）
# --------------------------------------------------------------------------- #
BRAND = "tbtest"
BRAND_LABEL = "OrolpTestBrand"
MODEL_DIR = "tbsynth"
MODEL = "tb2"
STEP_SUBDIR = "tb/step"
STEP_NAME = f"{BRAND_LABEL}{MODEL.upper()}.STEP"  # 归一化后 = 品牌+型号
OTHER_MODEL = "tb9"  # 有程序化 URDF、但没有 STEP —— 验证 --all 会跳过它

#: (角点 mm, 尺寸 mm)，base→tip；各盒子的垂直跨度刻意不以上一盒质心对称，
#: 使「离父连杆质心最近的顶点」唯一（不然最近点会并列、断言随遍历顺序漂）
BOXES: Tuple[Tuple[Tuple[float, float, float], Tuple[float, float, float]], ...] = (
    ((0.0, 0.0, 0.0), (300.0, 200.0, 100.0)),      # base
    ((250.0, 10.0, 90.0), (70.0, 120.0, 510.0)),   # link_1
    ((310.0, 20.0, 550.0), (210.0, 140.0, 70.0)),  # link_2
    ((530.0, 30.0, 570.0), (40.0, 60.0, 40.0)),    # tool0
)
#: 第 5 个组件（超出刚体数）——应被并入最后一个刚体
EXTRA_BOX = ((600.0, 30.0, 570.0), (30.0, 60.0, 40.0))

#: ``compute_frames`` 给出的旋转中心（米，手工推算，每个候选角点唯一最优）：
#:   o1 = base 质心 (0.150,0.100,0.050)       → link_1 角点 (0.250,0.130,0.090)
#:   o2 = link_1 质心 (0.285,0.070,0.345)     → link_2 角点 (0.310,0.020,0.550)
#:   o3 = link_2 质心 (0.415,0.090,0.585)     → tool0 角点 (0.530,0.090,0.570)
EXPECTED_ORIGINS = np.array(
    [
        [0.000, 0.000, 0.000],
        [0.250, 0.130, 0.090],
        [0.310, 0.020, 0.550],
        [0.530, 0.090, 0.570],
    ]
)

#: 合成程序化 URDF（运动学骨架的来源）。口径与 models/<brand>/<model>/<model>.urdf 一致：
#: ``<joint name=".." type="..">``（type 紧跟在 name 之后，解析器依赖这一点）、
#: ``<origin xyz=.. rpy=..>``、``<axis xyz=..>``、``<limit ..>``，tcp 走 fixed 关节。
PROGRAMMATIC_URDF = """<?xml version="1.0" encoding="utf-8"?>
<robot name="tb2">
  <link name="base"/>
  <joint name="j1" type="revolute">
    <parent link="base"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.141593" upper="3.141593" effort="50" velocity="3.14"/>
  </joint>
  <link name="link_1"/>
  <joint name="j2" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2" upper="2" effort="40" velocity="2.5"/>
  </joint>
  <link name="link_2"/>
  <joint name="j3" type="revolute">
    <parent link="link_2"/>
    <child link="tool0"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.5" upper="1.5" effort="20" velocity="5"/>
  </joint>
  <link name="tool0"/>
  <joint name="tcp_joint" type="fixed">
    <parent link="tool0"/>
    <child link="tcp"/>
    <origin xyz="0 0 0.08" rpy="0 0 0"/>
  </joint>
  <link name="tcp"/>
</robot>
"""

BODIES = ["base", "link_1", "link_2", "tool0"]  # = parse_kinematics 的 bodies 顺序


# --------------------------------------------------------------------------- #
# 合成装配体的构造 / 读取
# --------------------------------------------------------------------------- #
def make_assembly(path: Path, boxes=BOXES) -> Path:
    """把若干长方体打成一个 STEP 装配体（每个 box 一个顶层组件）。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    builder = BRep_Builder()
    comp = TopoDS_Compound()
    builder.MakeCompound(comp)
    for corner, size in boxes:
        builder.Add(comp, BRepPrimAPI_MakeBox(gp_Pnt(*corner), *size).Shape())
    writer = STEPControl_Writer()
    writer.Transfer(comp, STEPControl_AsIs)
    if writer.Write(str(path)) != IFSelect_RetDone:
        raise RuntimeError(f"STEP 写出失败: {path}")
    return path


def expected_bbox(corner, size) -> Tuple[np.ndarray, np.ndarray]:
    """source mm → 世界系米（z_up_mm 只缩放 0.001）。"""
    lo = np.asarray(corner, dtype=np.float64) / 1000.0
    hi = lo + np.asarray(size, dtype=np.float64) / 1000.0
    return lo, hi


def read_stl(path: Path) -> np.ndarray:
    """读回 ``write_stl_binary`` 写的二进制 STL，返回逐三角面的 (n,3,3) 顶点。"""
    raw = path.read_bytes()
    n = int(np.frombuffer(raw[80:84], dtype="<u4")[0])
    dt = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    tris = np.frombuffer(raw[84:84 + n * 50], dtype=dt)
    assert len(tris) == n, "STL 头里的面数与实际字节数不符"
    return tris["v"].astype(np.float64)


def read_stl_normals(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    n = int(np.frombuffer(raw[80:84], dtype="<u4")[0])
    dt = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
    return np.frombuffer(raw[84:84 + n * 50], dtype=dt)["normal"].astype(np.float64)


def joint_map(urdf: Path) -> Dict[str, dict]:
    """读 URDF 的关节表：名 → {type,parent,child,xyz,axis,limit}。"""
    root = ET.parse(urdf).getroot()
    out: Dict[str, dict] = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        ax = j.find("axis")
        lim = j.find("limit")
        out[j.get("name")] = {
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "xyz": np.array([float(v) for v in o.get("xyz").split()]),
            "rpy": np.array([float(v) for v in o.get("rpy").split()]),
            "axis": np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else None,
            "limit": {k: float(v) for k, v in lim.attrib.items()} if lim is not None else {},
        }
    return out


def link_names(urdf: Path) -> List[str]:
    return [l.get("name") for l in ET.parse(urdf).getroot().findall("link")]


# --------------------------------------------------------------------------- #
# 夹具
# --------------------------------------------------------------------------- #
@dataclass
class Synth:
    root: Path
    step: Path
    proc_urdf: Path
    real_urdf: Path
    meshes: Path


@pytest.fixture
def synth(tmp_path, monkeypatch) -> Synth:
    """把 ``assets/vendor`` 与 ``models`` 重定向到临时目录，并注册一个合成品牌。"""
    step = make_assembly(tmp_path / "assets" / "vendor" / STEP_SUBDIR / STEP_NAME)
    mdir = tmp_path / "models" / MODEL_DIR / MODEL
    mdir.mkdir(parents=True, exist_ok=True)
    proc = mdir / f"{MODEL}.urdf"
    proc.write_text(PROGRAMMATIC_URDF, encoding="utf-8")

    monkeypatch.setattr(bv, "VENDOR_STEP", tmp_path / "assets" / "vendor")
    monkeypatch.setattr(bv, "MODELS_DIR", tmp_path / "models")
    monkeypatch.setitem(
        bv.BRANDS,
        BRAND,
        bv.BrandSpec(BRAND, BRAND_LABEL, MODEL_DIR, STEP_SUBDIR,
                     coord="z_up_mm", verified=False, note="合成装配体（测试）"),
    )
    return Synth(tmp_path, step, proc, mdir / f"{MODEL}_real.urdf", mdir / "meshes_real")


# --------------------------------------------------------------------------- #
# ① OCP 版本兼容层
# --------------------------------------------------------------------------- #
def test_ocpcompat_import_does_not_load_ocp():
    """软依赖承诺：只 import 兼容层不该把 OCP 拖进来（启动不该因此慢几百毫秒）。"""
    code = (
        "import sys; sys.path.insert(0, 'python'); "
        "import orolp.ocpcompat as m; "
        "print('OCP' in sys.modules, m.has_ocp())"
    )
    out = subprocess.run([sys.executable, "-c", code], cwd=str(ROOT),
                         capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    assert out.stdout.split() == ["False", "True"]


def test_ocpcompat_describe_matches_reality():
    """describe() 报出来的符号名必须在真实模块上存在（改名后这里会先炸）。"""
    info = ocpcompat.describe()
    assert info["available"] is True
    assert info["version"], "应能读到 cadquery-ocp 的版本号"
    mod = ocpcompat.topods()
    assert hasattr(mod, info["casts"]["Face"])
    assert hasattr(mod, info["casts"]["Edge"])
    from OCP.BRep import BRep_Tool
    from OCP.BRepBndLib import BRepBndLib

    assert hasattr(BRep_Tool, info["functions"]["Triangulation"])
    assert hasattr(BRepBndLib, info["functions"]["Add"])


def test_face_of_and_edge_of_return_expected_types():
    shape = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 10.0, 20.0, 30.0).Shape()
    faces = TopExp_Explorer(shape, TopAbs_FACE)
    assert faces.More()
    face = ocpcompat.face_of(faces.Current())
    assert face.ShapeType() == TopAbs_FACE

    edges = TopExp_Explorer(shape, TopAbs_EDGE)
    assert edges.More()
    assert ocpcompat.edge_of(edges.Current()).ShapeType() == TopAbs_EDGE


def test_bbox_of_reports_source_units():
    """bbox_of 不做任何缩放/换轴——它只是把 Bnd_Box 拆成两个数组。"""
    shape = BRepPrimAPI_MakeBox(gp_Pnt(1.0, 2.0, 3.0), 10.0, 20.0, 30.0).Shape()
    lo, hi = ocpcompat.bbox_of(shape)
    assert np.allclose(lo, [1.0, 2.0, 3.0])
    assert np.allclose(hi, [11.0, 22.0, 33.0])


def test_tools_have_no_legacy_static_cast_calls():
    """工具里不得再出现裸的 ``Face_s(``/``Edge_s(``。

    这正是 2026-09-14 那次「装了 OCP 8 之后整条链不可用、却没有任何测试变红」的
    根因：OCP 把 ``TopoDS.Face_s`` 改名成 ``TopoDS.Face``，两处调用当场 AttributeError。
    """
    offenders = []
    for name in ("build_vendor_urdf.py", "vendor_cad_import.py"):
        for i, line in enumerate((ROOT / "tools" / name).read_text(encoding="utf-8").splitlines(), 1):
            if re.search(r"\b(Face_s|Edge_s|Vertex_s)\s*\(", line):
                offenders.append(f"{name}:{i}: {line.strip()}")
    assert offenders == [], (
        "OCP 7 时代的静态转型写法必须走 orolp.ocpcompat：\n  " + "\n  ".join(offenders)
    )


def test_programmatic_urdfs_have_zero_joint_rpy():
    """仓库级交叉约束：所有程序化 URDF 的**关节** origin rpy 必须为 0。

    ``build_vendor_urdf.write_urdf`` 把关节 origin 的 rpy 硬编码成 ``0 0 0``（几何来自
    CAD，朝向沿用程序化模型的轴系约定）。只要程序化 URDF 的关节 rpy 都是 0，这个
    硬编码就是等价的；一旦有人加了带非零关节 rpy 的品牌，真机 URDF 会**静默丢掉**
    真实关节朝向——所以这条约束必须被自动化盯着（``<visual>``/``<collision>`` 的
    rpy 不在此列，它们与关节朝向无关）。
    """
    bad = []
    for p in sorted(ROOT.glob("models/*/*/*.urdf")):
        if p.name.endswith("_real.urdf"):
            continue
        for j in ET.parse(p).getroot().findall("joint"):
            o = j.find("origin")
            if o is None:
                continue
            rpy = [abs(float(v)) for v in o.get("rpy", "0 0 0").split()]
            if any(v > 1e-12 for v in rpy):
                bad.append(f"{p.relative_to(ROOT)}:{j.get('name')} rpy={o.get('rpy')}")
    assert bad == [], (
        "程序化 URDF 出现了非零关节 rpy，write_urdf 的硬编码 rpy=0 会让真机 URDF 丢朝向，"
        "需先把 rpy 透传补上：\n  " + "\n  ".join(bad)
    )


# --------------------------------------------------------------------------- #
# ② STEP 读写往返
# --------------------------------------------------------------------------- #
def test_step_round_trip_keeps_component_count_and_order(synth):
    comps = bv.load_step_components(synth.step, bv._coord_trsf("z_up_mm"))
    assert [name for name, _ in comps] == [f"part_{i:02d}" for i in range(len(BOXES))]
    for (_, sh), (corner, size) in zip(comps, BOXES):
        lo, hi = ocpcompat.bbox_of(sh)
        e_lo, e_hi = expected_bbox(corner, size)
        # atol=1e-6：OCCT 的 Bnd_Box 带 1e-7 的默认间隙，包围盒会比真实几何略胖
        assert np.allclose(lo, e_lo, atol=1e-6), f"组件顺序/位置不符: {lo} vs {e_lo}"
        assert np.allclose(hi, e_hi, atol=1e-6), f"组件顺序/位置不符: {hi} vs {e_hi}"


def test_step_round_trip_z_up_convention_is_scale_only(synth):
    """``z_up_mm`` 只做 mm→m 缩放（不换轴）——合成装配体的断言基准。"""
    comps = bv.load_step_components(synth.step, bv._coord_trsf("z_up_mm"))
    lo, hi = ocpcompat.bbox_of(comps[0][1])
    src_lo, src_hi = expected_bbox(*BOXES[0])
    assert np.allclose(lo, src_lo, atol=1e-6)
    assert np.allclose(hi, src_hi, atol=1e-6)


def test_step_round_trip_y_up_convention_is_proper_and_vertical(synth):
    """``y_up_mm`` 必须把**竖直轴**送到 Z 轴、均匀缩放、且不是镜像。

    与 ``test_vendor_jaka.test_coord_trsf_yup_is_proper_uniform`` 同一纪律：锁不变量
    （竖直轴落位 + 均匀缩放 + 保持手性），不硬编码轴符号——符号由已验证的渲染结果
    决定。这里补的是「整条读链路」，而不只是 ``_coord_trsf`` 这一个函数。
    """
    from OCP.gp import gp_Pnt

    trsf = bv._coord_trsf("y_up_mm")

    def point(x, y, z):
        p = gp_Pnt(x, y, z).Transformed(trsf)
        return np.array([p.X(), p.Y(), p.Z()])

    ex, ey, ez = point(1, 0, 0), point(0, 1, 0), point(0, 0, 1)
    # 源的竖直轴 Y 落到目标的竖直轴 Z 上（X、Y 方向无分量）
    assert abs(abs(ey[2]) - 1e-3) < 1e-12 and abs(ey[0]) < 1e-12 and abs(ey[1]) < 1e-12
    # 均匀缩放 0.001（三个基向量等长，网格不被压扁）
    assert abs(np.linalg.norm(ex) - 1e-3) < 1e-12
    assert abs(np.linalg.norm(ey) - 1e-3) < 1e-12
    assert abs(np.linalg.norm(ez) - 1e-3) < 1e-12
    # 保持手性（不是反射镜像）
    assert float(np.linalg.det(np.column_stack([ex, ey, ez]))) > 0

    # 端到端：同一份 STEP 用 y_up_mm 读进来，**源的竖直轴 Y 的跨度**落到目标 Z 轴
    src_lo, src_hi = expected_bbox(*BOXES[1])
    src_span = src_hi - src_lo
    # 三个方向的跨度互不相同，这条断言才有判别力（否则靠对称性就能蒙对）
    assert len({round(float(v), 9) for v in src_span}) == 3, f"合成基准退化: {src_span}"
    comps = bv.load_step_components(synth.step, bv._coord_trsf("y_up_mm"))
    lo, hi = ocpcompat.bbox_of(comps[1][1])
    dst_span = hi - lo
    assert abs(dst_span[2] - src_span[1]) < 1e-6, (
        f"源的竖直 Y 跨度 {src_span[1]} 没有落到目标 Z 轴（实得 {dst_span[2]}）"
    )
    assert np.allclose(np.sort(dst_span), np.sort(src_span), atol=1e-6)


def test_mesh_component_triangle_count_and_units(synth):
    comps = bv.load_step_components(synth.step, bv._coord_trsf("z_up_mm"))
    verts = bv.mesh_component(comps[0][1], 0.01, 0.5)
    assert verts.shape == (36, 3), "长方体应三角化成 12 个面 × 3 个顶点"
    lo, hi = expected_bbox(*BOXES[0])
    assert np.allclose(verts.min(axis=0), lo, atol=1e-9)
    assert np.allclose(verts.max(axis=0), hi, atol=1e-9)

# --------------------------------------------------------------------------- #
# ③ 网格写出与回读
# --------------------------------------------------------------------------- #
def test_write_stl_binary_round_trip(tmp_path):
    verts = np.array([
        [0, 0, 0], [1, 0, 0], [0, 1, 0],
        [0, 0, 0], [0, 1, 0], [0, 0, 1],
    ], dtype=np.float64)
    out = tmp_path / "tri.stl"
    bv.write_stl_binary(out, verts)
    assert out.stat().st_size == 84 + 2 * 50
    tris = read_stl(out)
    assert tris.shape == (2, 3, 3)
    assert np.allclose(tris, verts.reshape(2, 3, 3), atol=1e-6)


def test_write_stl_binary_normals_are_unit_length(tmp_path):
    verts = np.array([[0, 0, 0], [3, 0, 0], [0, 4, 0]], dtype=np.float64)
    out = tmp_path / "one.stl"
    bv.write_stl_binary(out, verts)
    normals = read_stl_normals(out)
    assert normals.shape == (1, 3)
    assert abs(np.linalg.norm(normals[0]) - 1.0) < 1e-6
    assert np.allclose(normals[0], [0, 0, 1], atol=1e-6)  # +Z 面


def test_write_stl_binary_handles_degenerate_triangle(tmp_path):
    """零面积三角形的法向长度会是 0——写出时不能变成 NaN（旧实现的 ln=0 保护）。"""
    verts = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]], dtype=np.float64)
    out = tmp_path / "deg.stl"
    bv.write_stl_binary(out, verts)
    assert np.isfinite(read_stl_normals(out)).all()


# --------------------------------------------------------------------------- #
# ④ STEP 文件名 ↔ 型号匹配（合成品牌，补 test_vendor_jaka 的 zu3/zu30 边界）
# --------------------------------------------------------------------------- #
def test_step_path_for_finds_synthetic_step(synth):
    assert bv.step_path_for(BRAND, MODEL) == synth.step


def test_match_step_does_not_cross_models():
    bn, mn = bv._norm(BRAND_LABEL), bv._norm(MODEL)
    assert bv._match_step(bn, mn, bv._norm("OrolpTestBrandTB2"))
    assert not bv._match_step(bn, mn, bv._norm("OrolpTestBrandTB29"))  # 下一位是数字 → 另一个型号
    assert not bv._match_step(bn, mn, bv._norm("OtherBrandTB2"))


# --------------------------------------------------------------------------- #
# ⑤ 端到端 build_model → write_urdf
# --------------------------------------------------------------------------- #
def test_build_model_end_to_end_writes_urdf(synth):
    assert bv.build_model(BRAND, MODEL, 0.01, 0.5) == 0
    assert synth.real_urdf.is_file()
    for body in BODIES:
        mesh = synth.meshes / f"{body}.stl"
        assert mesh.is_file() and mesh.stat().st_size > 84, f"缺网格 {mesh}"


def test_generated_urdf_reuses_programmatic_kinematics(synth):
    bv.build_model(BRAND, MODEL, 0.01, 0.5)
    gen = joint_map(synth.real_urdf)
    src = joint_map(synth.proc_urdf)

    # 连杆集合与顺序（tcp 由 tcp_joint 追加在末尾）
    assert link_names(synth.real_urdf) == BODIES + ["tcp"]
    # 关节命名/类型/父子/轴/限位全部沿用程序化模型
    assert set(gen) == set(src)
    for name, s in src.items():
        g = gen[name]
        assert g["type"] == s["type"], name
        assert g["parent"] == s["parent"] and g["child"] == s["child"], name
        # 轴：revolute 关节两侧都该有；fixed（tcp_joint）两侧都没有
        if s["axis"] is None:
            assert g["axis"] is None, f"{name} 不该有 <axis>"
        else:
            assert np.allclose(g["axis"], s["axis"]), name
        # 数值按 %g（6 位有效数字）写出，见
        # test_generated_urdf_numbers_follow_six_significant_digit_policy
        assert set(g["limit"]) == set(s["limit"]), name
        for key, val in s["limit"].items():
            assert g["limit"][key] == pytest.approx(val, rel=1e-5, abs=1e-6), f"{name}.{key}"
    # tcp 是固定关节，从程序化模型拿偏移
    assert np.allclose(gen["tcp_joint"]["xyz"], src["tcp_joint"]["xyz"])


def test_generated_urdf_numbers_follow_six_significant_digit_policy(synth):
    """``write_urdf`` 用 ``%g``（**6 位有效数字**）写所有数值，包括继承来的关节限位。

    这是既定格式策略，不是四舍五入 bug：1 m 尺度上 6 位有效数字约 1 µm，关节限位
    的相对误差 ~1e-6 rad（物理上无意义）；已提交的 JAKA 真机产物也是这个精度
    （``0.0449633`` 正好 6 位）。把策略写明白，免得日后有人看到
    ``-3.14159 != -3.141593`` 以为是缺陷来回改。
    """
    bv.build_model(BRAND, MODEL, 0.01, 0.5)
    out = synth.real_urdf.read_text(encoding="utf-8")
    # 程序化模型里写的是 -3.141593 / 3.141593，6 位有效数字后是 -3.14159 / 3.14159
    assert 'lower="-3.14159" upper="3.14159"' in out
    # 且不会退化成更少的位数
    assert 'effort="50" velocity="3.14"' in out
    assert 'effort="40" velocity="2.5"' in out


def test_generated_urdf_joint_origins_match_expected_frames(synth):
    bv.build_model(BRAND, MODEL, 0.01, 0.5)
    gen = joint_map(synth.real_urdf)
    for i, name in enumerate(("j1", "j2", "j3"), start=1):
        expected = EXPECTED_ORIGINS[i] - EXPECTED_ORIGINS[i - 1]
        assert np.allclose(gen[name]["xyz"], expected, atol=1e-5), (
            f"{name} 原点 {gen[name]['xyz']} != 预期 {expected}"
        )
    # 关节朝向仍按程序化约定硬编码为 0（见 test_programmatic_urdfs_have_zero_joint_rpy）
    for name in ("j1", "j2", "j3"):
        assert np.allclose(gen[name]["rpy"], [0, 0, 0])


def test_joint_origin_is_a_vertex_of_the_child_mesh_and_the_nearest_one(synth):
    """关节原点的两条可证伪断言，只用磁盘上的产物（URDF 声明 vs 实际 STL）。

    ``compute_frames`` 的定义：第 i 个刚体的原点 = 第 i 个网格里离第 i-1 个刚体
    **质心**最近的**顶点**。链路把它写成 STL 时减去了该原点，于是同一件事在 STL 上
    变成两句可以独立验证的话：

    ① 该连杆的 STL（已按原点平移）**必有一个顶点落在本地原点**——即「原点确实是一个
       网格顶点」，而不是质心、包围盒角点之类的近似；
    ② 那个顶点就是离父连杆质心最近的那个（最近性是平移不变量，换算到局部系比较即可）。

    质心按链路的口径取「三角面顶点列表的均值」（不是几何形心——顶点多重性不同）。
    """
    bv.build_model(BRAND, MODEL, 0.01, 0.5)
    gen = joint_map(synth.real_urdf)

    verts = [read_stl(synth.meshes / f"{b}.stl").reshape(-1, 3) for b in BODIES]
    # 网格已减去各自原点，加回来就是世界系顶点云
    world = [verts[i] + EXPECTED_ORIGINS[i] for i in range(len(BODIES))]

    for i, name in enumerate(("j1", "j2", "j3"), start=1):
        # 从 URDF 链反推该连杆坐标系原点（世界系）
        o_i = EXPECTED_ORIGINS[i - 1] + gen[name]["xyz"]

        # ① 本地原点处必须真有顶点
        d_origin = np.linalg.norm(verts[i], axis=1)
        assert d_origin.min() < 1e-5, (
            f"{name}: {BODIES[i]} 的网格里没有顶点落在关节原点上"
            f"（最近 {d_origin.min():.6f} m）"
        )

        # ② 且它就是离父连杆质心最近的顶点
        prev_centroid = world[i - 1].mean(axis=0)
        d_prev = np.linalg.norm(verts[i] - (prev_centroid - o_i), axis=1)
        assert int(np.argmin(d_origin)) == int(np.argmin(d_prev)), (
            f"{name}: 关节原点不是离父连杆质心最近的顶点"
        )


def test_link_mesh_is_written_in_its_joint_frame(synth):
    """副产物不变量：每个连杆的 STL 都在自己的关节坐标系里（本地原点 = 关节原点）。

    这正是「关节绕真实连杆连接中心旋转」能成立的原因——网格与关节同原点，转动直接
    绕关节原点发生。若哪天改成保留世界坐标，真机会绕着世界原点甩出去。
    """
    bv.build_model(BRAND, MODEL, 0.01, 0.5)
    for i, body in enumerate(BODIES):
        v = read_stl(synth.meshes / f"{body}.stl").reshape(-1, 3)
        assert np.linalg.norm(v, axis=1).min() < 1e-5, f"{body}: 本地原点处没有顶点"
        # 世界系包围盒又必须等于源盒子
        src_lo, src_hi = expected_bbox(*BOXES[i])
        assert np.allclose(v.min(axis=0) + EXPECTED_ORIGINS[i], src_lo, atol=1e-4), body
        assert np.allclose(v.max(axis=0) + EXPECTED_ORIGINS[i], src_hi, atol=1e-4), body


def test_mesh_vertex_cloud_matches_expected_size(synth):
    """每个连杆网格已按原点平移，其顶点云应等价于源盒子的世界顶点（平移 + f4 精度）。"""
    bv.build_model(BRAND, MODEL, 0.01, 0.5)
    for i, body in enumerate(BODIES):
        v = read_stl(synth.meshes / f"{body}.stl").reshape(-1, 3)
        src_lo, src_hi = expected_bbox(*BOXES[i])
        got_lo = v.min(axis=0) + EXPECTED_ORIGINS[i]
        got_hi = v.max(axis=0) + EXPECTED_ORIGINS[i]
        assert np.allclose(got_lo, src_lo, atol=1e-4), body
        assert np.allclose(got_hi, src_hi, atol=1e-4), body


def test_generated_urdf_is_loadable_by_orolp_robot(synth):
    """最终目的：这个 URDF 能被库真加载，DOF 对、FK 能算、关节真带动末端。"""
    bv.build_model(BRAND, MODEL, 0.01, 0.5)
    robot = Robot(str(synth.real_urdf), "tb2_synth")
    assert robot.dof == 3, "3 个 revolute 关节（tcp 是 fixed）"
    home = robot.forward_kinematics([0.0, 0.0, 0.0])
    moved = robot.forward_kinematics([30.0, 20.0, 10.0])
    assert np.isfinite(np.asarray(home.matrix, dtype=float)).all()
    assert not np.allclose(np.asarray(home.matrix), np.asarray(moved.matrix)), "关节未带动末端"


def test_generated_urdf_geometry_is_mesh_only(synth):
    """连杆几何是 ``<mesh>``（真机外形），因此该机器人**不参与干涉检查**——既定契约。

    ``world/scene.py`` 的包络几何通道只认基本体（box/cylinder/sphere），mesh 会被跳过
    并给出提示。本测试把这个「已知且可接受」的行为写下来（唯一例外是 tcp 的示意球），
    免得日后被当成缺陷来回改。
    """
    bv.build_model(BRAND, MODEL, 0.01, 0.5)
    root = ET.parse(synth.real_urdf).getroot()
    for link in root.findall("link"):
        tags = {g.tag for g in link.iter() if g.tag in ("mesh", "box", "cylinder", "sphere")}
        if link.get("name") == "tcp":
            assert tags == {"sphere"}, "tcp 是示意球（无真机几何）"
        else:
            assert tags == {"mesh"}, f"{link.get('name')} 的几何应为 mesh，实为 {tags}"
    for m in root.iter("mesh"):
        assert m.get("filename", "").startswith("meshes_real/")


def test_extra_components_are_merged_into_last_body(synth, monkeypatch):
    # 第二个 STEP：多出一个组件（5 个 > 4 个刚体）。注意文件名带尾部 X，
    # 仍会被 _match_step 认作同一型号（前缀后的字符不是数字）
    extra_step = make_assembly(
        synth.root / "assets" / "vendor" / STEP_SUBDIR / "OrolpTestBrandTB2X.STEP",
        BOXES + (EXTRA_BOX,),
    )
    comps = bv.load_step_components(extra_step, bv._coord_trsf("z_up_mm"))
    assert len(comps) == 5

    monkeypatch.setattr(bv, "step_path_for", lambda b, m: extra_step)
    assert bv.build_model(BRAND, MODEL, 0.01, 0.5) == 0
    extra_mesh = synth.meshes / "tool0_extra0.stl"
    assert extra_mesh.is_file() and extra_mesh.stat().st_size > 84
    text = synth.real_urdf.read_text(encoding="utf-8")
    assert "meshes_real/tool0_extra0.stl" in text
    assert link_names(synth.real_urdf) == BODIES + ["tcp"], "额外组件不新增连杆"


def test_build_model_without_step_returns_one(synth, monkeypatch):
    monkeypatch.setattr(bv, "step_path_for", lambda b, m: None)
    assert bv.build_model(BRAND, MODEL, 0.01, 0.5) == 1


def test_build_model_without_programmatic_urdf_returns_one(synth):
    synth.proc_urdf.unlink()
    assert bv.build_model(BRAND, MODEL, 0.01, 0.5) == 1


def test_build_model_warns_when_components_fewer_than_bodies(synth, monkeypatch, capsys):
    two = make_assembly(synth.root / "assets" / "vendor" / STEP_SUBDIR / "OrolpTestBrandTB2Y.STEP",
                        BOXES[:2])
    monkeypatch.setattr(bv, "step_path_for", lambda b, m: two)
    assert bv.build_model(BRAND, MODEL, 0.01, 0.5) == 1
    assert "组件数" in capsys.readouterr().out


# --------------------------------------------------------------------------- #
# ⑥ probe / CLI
# --------------------------------------------------------------------------- #
def test_probe_step_reports_ocp_and_components(synth, capsys):
    assert bv.probe_step(BRAND, MODEL) == 0
    out = capsys.readouterr().out
    assert "OCP:" in out and "cadquery-ocp" in out
    assert f"组件数={len(BOXES)}" in out
    assert "总包围盒" in out


def test_cli_probe_runs_in_process(synth, monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["build_vendor_urdf.py", "--brand", BRAND,
                                      "--model", MODEL, "--probe"])
    assert bv.main() == 0
    assert "组件数=4" in capsys.readouterr().out


def test_cli_all_generates_every_model_with_step(synth, monkeypatch):
    # 再放一个「有程序化 URDF、没有 STEP」的型号：--all 必须跳过它
    other = synth.root / "models" / MODEL_DIR / OTHER_MODEL
    other.mkdir(parents=True)
    (other / f"{OTHER_MODEL}.urdf").write_text(PROGRAMMATIC_URDF, encoding="utf-8")
    assert bv.models_with_step(BRAND) == [MODEL]

    monkeypatch.setattr(sys, "argv", ["build_vendor_urdf.py", "--brand", BRAND, "--all"])
    assert bv.main() == 0
    assert synth.real_urdf.is_file()
    assert not (other / f"{OTHER_MODEL}_real.urdf").exists()


def test_probe_reports_missing_step(synth, capsys):
    assert bv.probe_step(BRAND, OTHER_MODEL) == 1
    assert "未找到 STEP" in capsys.readouterr().out


def test_cli_without_model_prints_help(synth, monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["build_vendor_urdf.py", "--brand", BRAND])
    assert bv.main() == 0
    assert "usage" in capsys.readouterr().out.lower()


# --------------------------------------------------------------------------- #
# ⑦ vendor_cad_import：分组 STL
# --------------------------------------------------------------------------- #
def test_vendor_cad_import_convert_writes_grouped_stl(synth, tmp_path):
    outdir = tmp_path / "grp"
    n, tris = vc.convert(synth.step, outdir, rel_deflection=0.01, angular_deflection=0.5)
    assert (n, tris) == (len(BOXES), 12 * len(BOXES))
    files = sorted(outdir.glob("grp_*.stl"))
    assert [f.name for f in files] == [f"grp_{i:02d}.stl" for i in range(len(BOXES))]
    for f in files:
        assert f.stat().st_size > 84


def test_vendor_cad_import_convert_all_writes_per_model_subdir(tmp_path):
    src = tmp_path / "step"
    make_assembly(src / "OrolpTestBrandTB2.STEP")
    make_assembly(src / "OrolpTestBrandTB9.STEP")
    outroot = tmp_path / "out"
    vc.convert_all(src, outroot, rel_deflection=0.01, angular_deflection=0.5)
    got = sorted(p.name for p in outroot.iterdir() if p.is_dir())
    assert got == ["OrolpTestBrandTB2", "OrolpTestBrandTB9"]
    assert len(list((outroot / "OrolpTestBrandTB2").glob("grp_*.stl"))) == len(BOXES)


def test_vendor_cad_import_convert_all_empty_dir_is_noop(tmp_path, capsys):
    vc.convert_all(tmp_path / "empty", tmp_path / "out")
    assert "没有 STEP" in capsys.readouterr().err


# --------------------------------------------------------------------------- #
# ⑧ ocpcompat 的另一个既有客户端：cam.py 的边提取
# --------------------------------------------------------------------------- #
def test_cam_edge_extraction_still_uses_compat_layer():
    """``cam.extract_edge_polylines`` 走 ``TopoDS.Edge``（OCP 8 改名过），是真客户端。"""
    shape = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 100.0, 100.0, 100.0).Shape()
    polylines = cam.extract_edge_polylines(shape, deflection=0.5)
    assert len(polylines) == 12, "长方体 12 条棱"
    for pl in polylines:
        assert pl.ndim == 2 and pl.shape[1] == 3
        assert np.isfinite(pl).all()
    lengths = sorted(float(np.linalg.norm(pl[-1] - pl[0])) for pl in polylines)
    assert np.allclose(lengths[:4], 100.0, atol=1e-6)


# --------------------------------------------------------------------------- #
# ⑨ 真实厂商 STEP 的只读冒烟（本地有 CAD 时自动生效，覆盖全部品牌）
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("brand", sorted(bv.BRANDS))
def test_real_vendor_step_probes_cleanly(brand, capsys):
    """对每个「真有 STEP」的品牌跑一次 ``--probe``（只读，不写 models/）。

    没有 STEP 时 skip —— 但链路本身已被上面的合成用例覆盖，这里的价值是：
    用户把官方 CAD 投放进来之后，**第一次**跑测试就能知道轴系/组件数是否可读。
    """
    models = bv.models_with_step(brand)
    if not models:
        pytest.skip(f"本地没有 {brand} 的 STEP")
    for m in models:
        assert bv.probe_step(brand, m) == 0
    out = capsys.readouterr().out
    assert f"=== probe {brand}:" in out
