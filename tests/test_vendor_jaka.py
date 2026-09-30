#!/usr/bin/env python3
"""JAKA 真机 CAD → URDF 的回归测试（``assets/models/jaka/<m>/<m>_real.urdf``）。

这批测试守两件事：
1. **已提交产物不被悄悄破坏**（CI 可跑，不需要 OCP/STEP）：每个型号的
   ``_real.urdf`` 结构正确、引用的 mesh 文件存在且非空、连杆链 base→tool0→tcp
   连通、用轻量正运动学证明「零位可达且摆动确实改变末端位姿」。
2. **生成工具的纯函数不被改坏**：``_match_step`` 的前缀边界（zu3⊂zu30、kr6⊂kr60
   不能误匹配）、``_coord_trsf`` 的 Y-up→Z-up 映射。

生成链路本身（需要 OCP + 厂商 STEP，CI 没有）用带 skip 的冒烟测试守住——
本地有 STEP 时会真正重建 zu3 到临时目录并校验。
"""

import importlib.util
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools import build_vendor_urdf as bv  # noqa: E402

JAKA = ROOT / "assets" / "models" / "jaka"
MODELS = ["zu3", "zu5", "zu7", "zu12", "zu18", "zu20", "zu30"]

_HAS_OCP = importlib.util.find_spec("OCP") is not None


# --------------------------------------------------------------------------- #
# 轻量正运动学（仅用于连通性 / 摆动冒烟，不依赖渲染引擎）
# --------------------------------------------------------------------------- #
def _translate(x, y, z):
    T = np.eye(4)
    T[:3, 3] = (x, y, z)
    return T


def _rot(axis, theta):
    ax = np.asarray(axis, dtype=float)
    ax = ax / np.linalg.norm(ax)
    x, y, z = ax
    c, s = np.cos(theta), np.sin(theta)
    # Rodrigues
    R = np.array(
        [
            [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
            [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
            [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
        ]
    )
    T = np.eye(4)
    T[:3, :3] = R
    return T


def _load(m):
    return ET.parse(JAKA / m / f"{m}_real.urdf").getroot()


# --------------------------------------------------------------------------- #
# 已提交产物
# --------------------------------------------------------------------------- #
def test_files_present():
    for m in MODELS:
        urdf = JAKA / m / f"{m}_real.urdf"
        assert urdf.exists(), f"{m}: 缺少 {urdf.name}"
        root = _load(m)
        for link in root.findall("link"):
            for mesh in link.iter("mesh"):
                fn = mesh.get("filename")
                p = JAKA / m / fn
                assert p.exists(), f"{m}: 引用了不存在的网格 {fn}"
                assert p.stat().st_size > 100, f"{m}: 网格为空 {fn}"


def test_structure_each_model():
    for m in MODELS:
        root = _load(m)
        links = [l.get("name") for l in root.findall("link")]
        assert links[0] == "base_link", f"{m}: 首连杆应为 base_link"
        assert "tool0" in links and "tcp" in links, f"{m}: 缺 tool0/tcp"
        rev = [j for j in root.findall("joint") if j.get("type") != "fixed"]
        assert len(rev) == 6, f"{m}: 应有 6 个旋转关节，实际 {len(rev)}"
        fixed = [j for j in root.findall("joint") if j.get("type") == "fixed"]
        assert any(
            j.find("child").get("link") == "tcp" for j in fixed
        ), f"{m}: 没有把 tcp 用固定关节挂上"
        for j in rev:
            assert j.find("axis") is not None, f"{m}/{j.get('name')}: 缺 axis"
            assert j.find("limit") is not None, f"{m}/{j.get('name')}: 缺 limit"
            lim = j.find("limit")
            lo, hi = float(lim.get("lower")), float(lim.get("upper"))
            assert lo < hi, f"{m}/{j.get('name')}: limit 下界>=上界"


def test_chain_connected():
    expect = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "tool0"]
    for m in MODELS:
        root = _load(m)
        parent_of = {}
        for j in root.findall("joint"):
            p = j.find("parent").get("link")
            c = j.find("child").get("link")
            parent_of[c] = p
        seq, node = [], "tool0"
        while node in parent_of:
            seq.append(node)
            node = parent_of[node]
        seq.append(node)
        seq.reverse()
        assert seq == expect, f"{m}: 连杆链应为 {expect}，实际 {seq}"


def test_fk_articulation():
    """轻量 FK：零位可达且有限；转几个关节后末端位置必须改变。"""
    chain = ["base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "tool0"]
    for m in MODELS:
        root = _load(m)
        joints = {j.get("name"): j for j in root.findall("joint") if j.get("type") != "fixed"}
        j_by_child = {j.find("child").get("link"): j for j in joints.values()}

        def child_tf(j, theta):
            o = j.find("origin").get("xyz").split()
            ox, oy, oz = map(float, o)
            ax = list(map(float, j.find("axis").get("xyz").split()))
            return _translate(ox, oy, oz) @ _rot(ax, theta)

        def fk(thetas):
            T = np.eye(4)
            for link in chain[1:]:
                T = T @ child_tf(j_by_child[link], thetas.get(j_by_child[link].get("name"), 0.0))
            return T[:3, 3]

        home = fk({})
        assert np.all(np.isfinite(home)), f"{m}: 零位末端坐标非有限"
        assert np.linalg.norm(home) > 0.05, f"{m}: 零位到达距离过小（{home}）"
        posed = fk({"joint_2": 0.7, "joint_3": -0.7, "joint_5": 0.6})
        assert np.linalg.norm(posed - home) > 1e-3, f"{m}: 摆动未改变末端位姿"


# --------------------------------------------------------------------------- #
# 生成工具纯函数
# --------------------------------------------------------------------------- #
def test_match_step_prefix_boundary():
    # zu3 不能误匹配 Zu30；kr6 不能误匹配 kr60
    assert bv._match_step("jaka", "zu3", "jakazu3v2020231016") is True
    assert bv._match_step("jaka", "zu3", "jakazu30v0120241220") is False
    assert bv._match_step("jaka", "zu30", "jakazu30v0120241220") is True
    assert bv._match_step("kuka", "kr6", "kukakr6r900...") is True
    assert bv._match_step("kuka", "kr6", "kukakr60r2700...") is False
    assert bv._match_step("kuka", "kr60", "kukakr60r2700...") is True


@pytest.mark.skipif(not _HAS_OCP, reason="需要 OCP（cadquery-ocp）")
def test_coord_trsf_yup_is_proper_uniform():
    """y_up_mm 变换必须是「均匀缩放 0.001 + 保持右手系」的刚体缩放。

    锁住不变量比锁死某个轴符号更有意义：一旦变换变成反射（手性翻转）或
    非均匀缩放，真机网格会被镜像/压扁，而这类 bug 用视觉很难第一时间发现。
    具体的轴系符号由已验证的渲染结果决定，不在本测试内硬编码。
    """
    trsf = bv._coord_trsf("y_up_mm")
    from OCP.gp import gp_Pnt

    def P(x, y, z):
        p = gp_Pnt(x, y, z).Transformed(trsf)
        return np.array([p.X(), p.Y(), p.Z()])

    a = P(0, 0, 0)
    b = P(1, 0, 0)
    c = P(0, 1, 0)
    d = P(0, 0, 1)
    # 均匀缩放 0.001（防止非均匀缩放压扁网格）
    assert abs(np.linalg.norm(b - a) - 0.001) < 1e-9
    assert abs(np.linalg.norm(c - a) - 0.001) < 1e-9
    assert abs(np.linalg.norm(d - a) - 0.001) < 1e-9
    # 保持右手系（有向体积同号，防止反射镜像）
    vol_src = np.dot(np.cross([1, 0, 0], [0, 1, 0]), [0, 0, 1])  # = +1
    vol_dst = np.dot(np.cross(b - a, c - a), d - a)
    assert vol_src * vol_dst > 0


@pytest.mark.skipif(not _HAS_OCP, reason="需要 OCP + JAKA STEP")
def test_build_zu3_to_tmp(tmp_path, monkeypatch):
    _step_dir = ROOT / "assets" / "vendor" / "jaka" / "step"
    step_src = next(_step_dir.glob("JAKA_Zu_3*.STEP"), None) if _step_dir.exists() else None
    if not step_src:
        pytest.skip("本地没有 JAKA STEP")
    # 把程序化 URDF 与 STEP 复制到临时目录，避免触碰已提交产物
    md = tmp_path / "jaka" / "zu3"
    md.mkdir(parents=True)
    shutil.copy(JAKA / "zu3" / "zu3.urdf", md / "zu3.urdf")
    vs = tmp_path / "vendor" / "jaka" / "step"
    vs.mkdir(parents=True)
    shutil.copy(step_src, vs / step_src.name)
    monkeypatch.setattr(bv, "MODELS_DIR", tmp_path)
    monkeypatch.setattr(bv, "VENDOR_STEP", tmp_path / "vendor")

    rc = bv.build_model("jaka", "zu3", 0.02, 0.5)
    assert rc == 0
    out_urdf = tmp_path / "jaka" / "zu3" / "zu3_real.urdf"
    out_mesh = tmp_path / "jaka" / "zu3" / "meshes_real" / "base_link.stl"
    assert out_urdf.exists() and out_mesh.exists()
    assert out_mesh.stat().st_size > 100
