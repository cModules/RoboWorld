#!/usr/bin/env python3
"""v0.3.2 STEP/CAM 轨迹生成测试（``service.cam``）。

覆盖三个能力块：
1. PLY 点云导入（ASCII / 二进制大小端 / 带面与额外属性）；
2. 扫描偏差比对（解析验证 max/mean/rms/最差点）；
3. 路径 → 程序生成（转角检测、moveC 圆弧分片、闭合环形、z_lift、姿态、规划校验）；
4. CAD 导入与焊缝提取（OCP 通道：STEP/IGES 往返、盒体边链、圆柱整圆焊缝、
   长度≈2πr 物理校验、select_seams 过滤）。

OCP 相关测试在未安装 ``cadquery-ocp`` 时自动跳过（CI 兼容）。
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from service.cam import (
    SeamPath,
    chain_polylines,
    detect_corners,
    extract_edge_polylines,
    extract_seams,
    load_ply,
    path_to_program,
    scan_deviation,
    select_seams,
)
from domain.pose import Pose  # noqa: E402
from domain.robot import Robot  # noqa: E402

URDF = REPO_ROOT / "assets" / "models" / "kuka" / "kr16_2" / "kr16_2.urdf"

_HAS_OCP = True
try:
    import OCP  # noqa: F401
except ImportError:
    _HAS_OCP = False


@pytest.fixture
def robot() -> Robot:
    if not URDF.exists():
        pytest.skip(f"缺少测试用 URDF: {URDF}")
    return Robot(str(URDF), "kr16_2")


# --------------------------------------------------------------------------- #
# PLY 点云导入
# --------------------------------------------------------------------------- #
def _write_ply_ascii(path, pts, extra_header=(), extra_cols=None):
    n = len(pts)
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        for h in extra_header:
            f.write(h + "\n")
        f.write(f"element vertex {n}\nproperty float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        for i, p in enumerate(pts):
            tail = "" if extra_cols is None else extra_cols(i)
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}{tail}\n")


def _write_ply_binary(path, pts, endian="<"):
    n = len(pts)
    fmt = f"format binary_{'little' if endian == '<' else 'big'}_endian 1.0"
    with open(path, "wb") as f:
        f.write(f"ply\n{fmt}\nelement vertex {n}\n".encode())
        f.write(b"property double x\nproperty double y\nproperty double z\n")
        f.write(b"end_header\n")
        f.write(np.asarray(pts, dtype=endian + "f8").tobytes())


@pytest.fixture
def sample_points():
    return np.array([[0, 0, 0], [10, 0, 0], [10, 10, 0], [0, 10, 0], [0, 0, 5]], dtype=float)


def test_ply_ascii(tmp_path, sample_points):
    p = tmp_path / "a.ply"
    _write_ply_ascii(p, sample_points)
    assert np.allclose(load_ply(p), sample_points)


def test_ply_binary_little_and_big_endian(tmp_path, sample_points):
    for endian in ("<", ">"):
        p = tmp_path / f"b_{'le' if endian == '<' else 'be'}.ply"
        _write_ply_binary(p, sample_points, endian)
        assert np.allclose(load_ply(p), sample_points)


def test_ply_with_normals_and_faces(tmp_path, sample_points):
    """额外属性（法线/颜色）与后续 face element 均被正确跳过。"""
    p = tmp_path / "c.ply"
    _write_ply_ascii(
        p,
        sample_points,
        extra_header=["element face 1"],
        extra_cols=lambda i: f" 0 0 1 255",
    )
    with open(p, "a") as f:
        f.write("3 0 1 2\n")
    assert np.allclose(load_ply(p), sample_points)


def test_ply_missing_axis_raises(tmp_path, sample_points):
    p = tmp_path / "d.ply"
    with open(p, "w") as f:
        f.write(
            "ply\nformat ascii 1.0\nelement vertex 1\nproperty float x\nproperty float y\nend_header\n1 2\n"
        )
    with pytest.raises(ValueError):
        load_ply(p)


def test_ply_not_ply_raises(tmp_path):
    p = tmp_path / "e.txt"
    p.write_text("hello")
    with pytest.raises(ValueError):
        load_ply(p)


# --------------------------------------------------------------------------- #
# 扫描偏差比对
# --------------------------------------------------------------------------- #
def test_deviation_zero_for_on_path_points():
    path = np.array([[0, 0, 0], [100, 0, 0]], dtype=float)
    pts = np.linspace([0, 0, 0], [100, 0, 0], 11)
    r = scan_deviation(pts, path)
    assert r.max_dev == pytest.approx(0.0, abs=1e-9)


def test_deviation_offset_stats():
    path = np.array([[0, 0, 0], [100, 0, 0]], dtype=float)
    pts = np.linspace([0, 0, 2], [100, 0, 2], 11)  # 整体抬高 2mm
    r = scan_deviation(pts, path)
    assert r.max_dev == pytest.approx(2.0, abs=1e-9)
    assert r.mean_dev == pytest.approx(2.0, abs=1e-9)
    assert r.rms_dev == pytest.approx(2.0, abs=1e-9)
    assert r.n_points == 11
    assert r.path_length == pytest.approx(100.0)


def test_deviation_worst_point_localized():
    path = np.array([[0, 0, 0], [100, 0, 0]], dtype=float)
    pts = np.array([[10, 0, 0], [50, 0, 5], [90, 0, 0]], dtype=float)
    r = scan_deviation(pts, path)
    assert r.worst_index == 1
    assert r.max_dev == pytest.approx(5.0)
    assert "max" in r.format()


def test_deviation_point_beyond_path_ends():
    """端点外的点按最近端点计距。"""
    path = np.array([[0, 0, 0], [100, 0, 0]], dtype=float)
    pts = np.array([[150, 0, 0]], dtype=float)
    r = scan_deviation(pts, path)
    assert r.max_dev == pytest.approx(50.0)


def test_deviation_shape_validation():
    with pytest.raises(ValueError):
        scan_deviation(np.zeros((3, 2)), np.zeros((2, 3)))


# --------------------------------------------------------------------------- #
# 转角检测
# --------------------------------------------------------------------------- #
def test_detect_corners_rectangle():
    rect = np.array([[0, 0, 0], [10, 0, 0], [10, 10, 0], [0, 10, 0]], dtype=float)
    assert detect_corners(rect, angle_deg=20.0) == [1, 2]


def test_detect_corners_smooth_circle_no_corners():
    t = np.linspace(0, 2 * np.pi, 72, endpoint=False)
    pts = np.column_stack([np.cos(t), np.sin(t), np.zeros_like(t)])
    assert detect_corners(pts, angle_deg=20.0) == []


def test_detect_corners_closed_includes_wrap_corner():
    """闭合矩形：起点（首尾接缝处）的 90° 转角也要被识别。"""
    rect = np.array([[0, 0, 0], [10, 0, 0], [10, 10, 0], [0, 10, 0]], dtype=float)
    assert detect_corners(rect, angle_deg=20.0, closed=True) == [0, 1, 2, 3]


# --------------------------------------------------------------------------- #
# 路径 → 程序
# --------------------------------------------------------------------------- #
def _motion_kinds(prog):
    return [
        getattr(i, "type", None).name if hasattr(i, "type") else type(i).__name__
        for i in prog.items
    ]


def test_path_to_program_closed_circle_uses_arc_chunks(robot):
    """整圆闭合缝：moveC 按 ≤120° 分片，不做整圆指令。"""
    t = np.linspace(0, 2 * np.pi, 61, endpoint=False)
    pts = np.column_stack([600 + 25 * np.cos(t), 25 * np.sin(t), np.full_like(t, 500.0)])
    prog = path_to_program(pts, robot, closed=True, z_lift=2.0).build()
    kinds = _motion_kinds(prog)
    assert kinds[0] == "LINEAR"  # 先走到起点
    n_circ = kinds.count("CIRCULAR")
    assert 3 <= n_circ <= 4  # 360° / 120° = 3 段（浮点累计可能切 4）


def test_path_to_program_rectangle_corners_all_linear(robot):
    rect = np.array([[500, 0, 400], [700, 0, 400], [700, 100, 400], [500, 100, 400]], dtype=float)
    prog = path_to_program(rect, robot, closed=True, corner_angle_deg=10).build()
    kinds = _motion_kinds(prog)
    assert all(k == "LINEAR" for k in kinds)
    assert len(kinds) == 5  # 起点 + 4 段


def test_path_to_program_open_arc(robot):
    t = np.linspace(0, np.pi, 25)
    pts = np.column_stack([500 + 300 * np.cos(t), 200 * np.sin(t), np.full_like(t, 400.0)])
    prog = path_to_program(pts, robot).build()
    kinds = _motion_kinds(prog)
    assert kinds[0] == "LINEAR"
    assert kinds.count("CIRCULAR") >= 2


def test_path_to_program_z_lift_and_orientation(robot):
    pts = np.array([[500, 0, 400], [600, 0, 400], [700, 0, 400]], dtype=float)
    prog = path_to_program(pts, robot, z_lift=3.0, rpy=(0, 180, 0)).build()
    motion = prog.items[0]
    pos = motion.target.matrix[:3, 3]
    assert pos[2] == pytest.approx(403.0)
    # 姿态为 RPY(0,180,0)（焊炬向下）
    rot = motion.target.matrix[:3, :3]
    expected = Pose.from_xyz_rpy(0, 0, 0, 0, 180, 0).matrix[:3, :3]
    assert np.allclose(rot, expected, atol=1e-9)


def test_path_to_program_return_builder_chainable(robot):
    """返回未 build 的 builder，可继续追加 IO/退刀指令。"""
    pts = np.array([[500, 0, 400], [600, 0, 400]], dtype=float)
    builder = path_to_program(pts, robot)
    prog = builder.gripper("open").build()
    assert type(prog.items[-1]).__name__ != "Motion"


def test_path_to_program_plan_and_cycle_time(robot):
    t = np.linspace(0, 2 * np.pi, 49, endpoint=False)
    pts = np.column_stack([600 + 25 * np.cos(t), 25 * np.sin(t), np.full_like(t, 500.0)])
    prog = path_to_program(pts, robot, closed=True, velocity=30.0).build()
    report = prog.validate()
    errors = [i for i in report.issues if i.severity == "error"]
    assert errors == []
    assert report.cycle_time > 0


# --------------------------------------------------------------------------- #
# 链拼接（纯 numpy，不依赖 OCP）
# --------------------------------------------------------------------------- #
def test_chain_polylines_merges_and_closes():
    a = np.array([[0, 0, 0], [10, 0, 0]], dtype=float)
    b = np.array([[10, 0, 0], [10, 10, 0]], dtype=float)  # 顺向衔接
    c = np.array([[10, 10, 0], [0, 10, 0]], dtype=float)  # 继续延伸
    d = np.array([[0, 10, 0], [0, 0, 0]], dtype=float)  # 反向衔接回起点 → 闭合
    iso = np.array([[50, 50, 0], [60, 50, 0]], dtype=float)  # 孤立段
    seams = chain_polylines([a, b, c, d, iso], tol=1e-6)
    assert len(seams) == 2
    closed = [s for s in seams if s.closed]
    assert len(closed) == 1
    assert closed[0].length == pytest.approx(40.0)  # 4×10
    open_seams = [s for s in seams if not s.closed]
    assert len(open_seams) == 1 and open_seams[0].length == pytest.approx(10.0)


def test_seam_path_length_closed_and_open():
    open_tri = SeamPath(points=np.array([[0, 0, 0], [10, 0, 0], [0, 10, 0]], dtype=float))
    assert open_tri.length == pytest.approx(10.0 + 10.0 * np.sqrt(2))
    closed_tri = SeamPath(points=open_tri.points.copy(), closed=True)
    assert closed_tri.length == pytest.approx(20.0 + 10.0 * np.sqrt(2))


# --------------------------------------------------------------------------- #
# CAD 导入与焊缝提取（OCP 通道）
# --------------------------------------------------------------------------- #
@pytest.mark.skipif(not _HAS_OCP, reason="需要 cadquery-ocp")
def test_load_cad_unsupported_suffix():
    from service.cam import load_cad

    with pytest.raises(ValueError):
        load_cad("/tmp/not_a_cad.stl")


@pytest.mark.skipif(not _HAS_OCP, reason="需要 cadquery-ocp")
def test_step_roundtrip_and_box_edges(tmp_path):
    """盒体 STEP 往返 → 12 条边 → 链拼接成闭合轮廓链。"""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs

    box = BRepPrimAPI_MakeBox(100.0, 50.0, 20.0).Shape()
    w = STEPControl_Writer()
    w.Transfer(box, STEPControl_AsIs)
    step_path = tmp_path / "box.step"
    assert w.Write(str(step_path)).name == "IFSelect_RetDone"

    edges = extract_edge_polylines(load_shape(step_path))
    assert len(edges) == 12
    seams = chain_polylines(edges, tol=1e-3)
    assert len(seams) >= 3  # 12 条边至少串成若干条链
    assert any(s.closed for s in seams)


@pytest.mark.skipif(not _HAS_OCP, reason="需要 cadquery-ocp")
def test_cylinder_top_circle_seam_length(tmp_path):
    """圆柱顶面整圆焊缝：闭合、长度≈2πr——焊缝提取的物理校验。"""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder

    r_cyl = 25.0
    cyl = BRepPrimAPI_MakeCylinder(r_cyl, 40.0).Shape()
    seams = extract_seams(cyl, deflection=0.05)
    circles = select_seams(seams, closed=True)
    assert circles, "圆柱必须提取出闭合圆缝"
    for s in circles:
        assert s.length == pytest.approx(2 * np.pi * r_cyl, rel=0.02)


@pytest.mark.skipif(not _HAS_OCP, reason="需要 cadquery-ocp")
def test_iges_roundtrip(tmp_path):
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder
    from OCP.IGESControl import IGESControl_Writer

    cyl = BRepPrimAPI_MakeCylinder(10.0, 20.0).Shape()
    w = IGESControl_Writer()
    w.AddShape(cyl)
    w.ComputeModel()
    iges_path = tmp_path / "cyl.igs"
    assert w.Write(str(iges_path))
    seams = extract_seams(iges_path)
    assert seams


@pytest.mark.skipif(not _HAS_OCP, reason="需要 cadquery-ocp")
def test_select_seams_filters(tmp_path):
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder

    seams = extract_seams(BRepPrimAPI_MakeCylinder(25.0, 40.0).Shape(), deflection=0.05)
    closed = select_seams(seams, closed=True)
    assert all(s.closed for s in closed)
    opened = select_seams(seams, closed=False)
    assert all(not s.closed for s in opened)
    long_only = select_seams(seams, min_length=100.0)
    assert all(s.length >= 100.0 for s in long_only)
    one = select_seams(seams, index=0)
    assert len(one) == 1
    with pytest.raises(IndexError):
        select_seams(seams, index=len(seams))


@pytest.mark.skipif(not _HAS_OCP, reason="需要 cadquery-ocp")
def test_real_seam_to_program_validates(robot):
    """端到端：CAD 整圆焊缝 → 程序 → 规划校验零错误。"""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder

    seams = extract_seams(BRepPrimAPI_MakeCylinder(25.0, 40.0).Shape(), deflection=0.05)
    circle = select_seams(seams, closed=True)[0]
    pts = circle.points + np.array([600.0, 0.0, 500.0])  # 挪进工作空间
    prog = path_to_program(pts, robot, closed=True, z_lift=2.0, velocity=30.0).build()
    report = prog.validate()
    assert [i for i in report.issues if i.severity == "error"] == []


@pytest.mark.skipif(not _HAS_OCP, reason="需要 cadquery-ocp")
def test_scan_deviation_against_real_seam(tmp_path):
    """点云相对真实 CAD 焊缝的偏差：人为偏移 0.8mm 可精确回收。"""
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder

    seams = extract_seams(BRepPrimAPI_MakeCylinder(25.0, 40.0).Shape(), deflection=0.05)
    circle = select_seams(seams, closed=True)[0]
    cloud = circle.points + np.array([0.0, 0.0, 0.8])
    p = tmp_path / "scan.ply"
    _write_ply_binary(p, cloud, "<")
    report = scan_deviation(load_ply(p), circle.points)
    assert report.max_dev == pytest.approx(0.8, abs=1e-6)


# --------------------------------------------------------------------------- #
# 内部工具
# --------------------------------------------------------------------------- #
def load_shape(path):
    from service.cam import load_cad

    return load_cad(path)
