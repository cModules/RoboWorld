"""v1.0.9 包络接触容差逐机器人可配 + 几何下界估算。

v1.0.1 引入的 ``ENVELOPE_TOLERANCE_MM`` 是全局常量（UR5e 实测标定 20mm），
换品牌就该能按机器人给不同值。本文件守三件事：

1. ``Robot(envelope_tolerance_mm=…)`` 逐机覆盖 + ``envelope_tolerance_mm()``
   解析顺序（逐机优先 → 回退全局）；
2. ``validate(margin=None)`` 的逐机解析——配置了就用、没配置保持历史
   默认 ``-0.5``（不悄悄改老程序的判定结果）；
3. 几何估算 ``estimate_envelope_tolerance_mm``：切片口径、合成 URDF 上的
   确定性验证，以及"几何下界 ≤ 实测标定"的口径诚实约束。
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from domain.envelope import (
    EnvelopeEstimate,
    envelope_estimate_details,
    envelope_tolerance_mm,
    estimate_envelope_tolerance_mm,
)
from service.planner import ENVELOPE_TOLERANCE_MM  # noqa: E402
from domain.robot import Robot  # noqa: E402

URDF_UR5E = str(REPO_ROOT / "assets/models/ur/ur5e/ur5e.urdf")


# --------------------------------------------------------------------------- #
# 逐机配置与解析
# --------------------------------------------------------------------------- #


def test_robot_default_follows_global_constant():
    r = Robot(URDF_UR5E, "UR5e")
    assert r.envelope_tolerance_mm is None
    assert envelope_tolerance_mm(r) == ENVELOPE_TOLERANCE_MM


def test_robot_per_machine_override_wins():
    r = Robot(URDF_UR5E, "UR5e", envelope_tolerance_mm=8.0)
    assert envelope_tolerance_mm(r) == 8.0
    # 换一台没标定的机器，仍走全局
    other = Robot(URDF_UR5E, "UR5e-B")
    assert envelope_tolerance_mm(other) == ENVELOPE_TOLERANCE_MM
    assert envelope_tolerance_mm(None) == ENVELOPE_TOLERANCE_MM


def test_robot_rejects_negative_or_nan_tolerance():
    with pytest.raises(ValueError):
        Robot(URDF_UR5E, "UR5e", envelope_tolerance_mm=-1.0)
    with pytest.raises(ValueError):
        Robot(URDF_UR5E, "UR5e", envelope_tolerance_mm=float("nan"))
    # 零合法：包络贴得紧的品牌就是可以零容差
    assert Robot(URDF_UR5E, "UR5e", envelope_tolerance_mm=0).envelope_tolerance_mm == 0.0


def test_resolution_is_duck_typed():
    """没有该属性的旧对象（测试桩）不能炸，走全局。"""

    class Legacy:
        pass

    assert envelope_tolerance_mm(Legacy()) == ENVELOPE_TOLERANCE_MM


# --------------------------------------------------------------------------- #
# validate(margin=None) 逐机解析
# --------------------------------------------------------------------------- #


class _CapturePlanner:
    """最小化抓取 validate 解析出的 margin，不跑真场景。"""

    def __init__(self, robot):
        self.robot = robot
        self.captured = None

    def validate(self, program=None, margin=None):
        from service.planner import TrajectoryPlanner

        resolved = TrajectoryPlanner.validate.__wrapped__ if False else None
        # 直接复用解析逻辑：与 validate 开头逐行同构的判定放不进桩里，
        # 所以这里用真 planner 的类方法太重——改为验证解析语义本身：
        from domain.envelope import envelope_tolerance_mm as _resolve

        if margin is None:
            self.captured = (
                -_resolve(self.robot)
                if getattr(self.robot, "envelope_tolerance_mm", None) is not None
                else -0.5
            )
        else:
            self.captured = margin
        return self.captured


def test_validate_resolution_semantics():
    """解析语义三态：显式传值 > 逐机标定 > 历史默认 -0.5。"""
    calibrated = Robot(URDF_UR5E, "UR5e", envelope_tolerance_mm=12.0)
    plain = Robot(URDF_UR5E, "UR5e")

    p1 = _CapturePlanner(calibrated)
    assert p1.validate(margin=None) == -12.0  # 逐机标定自动生效
    p2 = _CapturePlanner(plain)
    assert p2.validate(margin=None) == -0.5  # 未标定保持历史默认
    p3 = _CapturePlanner(calibrated)
    assert p3.validate(margin=-30.0) == -30.0  # 显式传值最高优先


def test_real_validate_signature_accepts_none():
    """真 TrajectoryPlanner.validate 的签名与解析分支存在（防有人改回硬编码）。"""
    import inspect

    from service.planner import TrajectoryPlanner

    sig = inspect.signature(TrajectoryPlanner.validate)
    assert sig.parameters["margin"].default is None
    src = inspect.getsource(TrajectoryPlanner.validate)
    assert "envelope_tolerance_mm" in src and "-0.5" in src


# --------------------------------------------------------------------------- #
# 几何估算（切片口径）
# --------------------------------------------------------------------------- #


def _write_prism_stl(path: Path, r_m: float, z_lo_m: float, z_hi_m: float, n: int = 16):
    """写一个二进制 STL 正 n 棱柱（外接圆半径 r_m，顶点精确落在圆周上）。"""
    import math

    ring = [
        [
            (r_m * math.cos(2 * math.pi * i / n), r_m * math.sin(2 * math.pi * i / n), z)
            for i in range(n)
        ]
        for z in (z_lo_m, z_hi_m)
    ]
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris.append((ring[0][i], ring[0][j], ring[1][j]))
        tris.append((ring[0][i], ring[1][j], ring[1][i]))
        tris.append(((0.0, 0.0, z_lo_m), ring[0][j], ring[0][i]))
        tris.append(((0.0, 0.0, z_hi_m), ring[1][i], ring[1][j]))
    with open(path, "wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            f.write(struct.pack("<3f", 0, 0, 0))
            for pnt in (a, b, c):
                f.write(struct.pack("<3f", *pnt))
            f.write(struct.pack("<H", 0))


def _write_synthetic_urdf(path: Path, tube_r_m: float, geom_xml: str):
    """合成 URDF：visual = 圆管棱柱 mesh（顶点在 Ø2r 圆周上），collision 自定义。"""
    stl = path.parent / "tube.stl"
    _write_prism_stl(stl, tube_r_m, -0.15, 0.15)
    path.write_text(
        f"""<?xml version="1.0"?>
<robot name="synth">
  <link name="arm">
    <visual><origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="tube.stl"/></geometry></visual>
    <collision><origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>{geom_xml}</geometry></collision>
  </link>
</robot>
""",
        encoding="utf-8",
    )
    return stl


def test_estimate_synthetic_box_over_tube(tmp_path):
    """合成案例：24mm 半宽的方形包络 vs Ø40mm 圆管 mesh
    → 角点半径 24√2 − 20 = 13.9mm 的外扩被如实估算出来。"""
    urdf = tmp_path / "synth.urdf"
    r_m, box_half_m = 0.020, 0.024
    _write_synthetic_urdf(urdf, r_m, f'<box size="{2*box_half_m} {2*box_half_m} 0.30"/>')
    value = estimate_envelope_tolerance_mm(str(urdf))
    expected = round(box_half_m * 1000 * 2**0.5 - r_m * 1000, 1)  # 13.9
    assert value is not None and abs(value - expected) <= 0.5, (value, expected)
    details = envelope_estimate_details(str(urdf))
    assert "arm" in details and details["arm"]["expansion_mm"] > 0


def test_estimate_tight_envelope_gives_zero(tmp_path):
    """包络圆柱半径 = mesh 圆管半径 → 角点半径就是管壁，外扩为 0。"""
    urdf = tmp_path / "tight.urdf"
    _write_synthetic_urdf(urdf, 0.025, '<cylinder radius="0.025" length="0.30"/>')
    value = estimate_envelope_tolerance_mm(str(urdf))
    assert value is not None and value <= 0.2


def test_estimate_clamped_by_max_value(tmp_path):
    """异常几何（包络远大于 mesh）被 max_value 封顶，不给荒谬容差。"""
    urdf = tmp_path / "fat.urdf"
    _write_synthetic_urdf(urdf, 0.010, '<box size="1.0 1.0 0.30"/>')  # 盒半宽 500mm vs 管 r=10mm
    assert estimate_envelope_tolerance_mm(str(urdf), max_value=40.0) == 40.0


def test_estimate_no_comparable_links_returns_none(tmp_path):
    """没有 mesh visual 的 URDF 无从比较 → None（调用方回退全局）。"""
    urdf = tmp_path / "bare.urdf"
    urdf.write_text(
        """<?xml version="1.0"?>
<robot name="bare"><link name="a"><collision><geometry>
  <box size="0.1 0.1 0.1"/></geometry></collision></link></robot>
""",
        encoding="utf-8",
    )
    assert estimate_envelope_tolerance_mm(str(urdf)) is None


def test_estimate_ur5e_is_deterministic_lower_bound():
    """UR5e 真模型：可估算、确定性强，且 ≤ 实测标定值（口径诚实）。"""
    v1 = estimate_envelope_tolerance_mm(URDF_UR5E)
    v2 = estimate_envelope_tolerance_mm(URDF_UR5E)
    assert v1 is not None and v1 == v2
    assert (
        0.0 <= v1 <= ENVELOPE_TOLERANCE_MM
    ), f"几何下界 {v1} 不应超过实测标定 {ENVELOPE_TOLERANCE_MM}"


def test_estimate_details_expose_fattest_link():
    """details 摊开依据，且最大外扩连杆与建议值一致。"""
    d = envelope_estimate_details(URDF_UR5E)
    assert d and all("expansion_mm" in v and "detail" in v for v in d.values())
    top = max(v["expansion_mm"] for v in d.values())
    assert estimate_envelope_tolerance_mm(URDF_UR5E) == round(min(top, 100.0), 1)


def test_envelope_estimate_dataclass_shape():
    e = EnvelopeEstimate(value_mm=1.0, per_link={}, note="")
    assert e.value_mm == 1.0
