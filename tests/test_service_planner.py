"""service/planner.py 拆分（types/math/dynamics/expand 四模块）的守门测试。

守住的契约：

* **公开导入面不变** —— 外部模块在用的全部符号仍可从 ``service.planner``
  导入（re-export 兜底）。
* **re-export 是同一对象** —— ``planner.X is planner_types.X`` 等，
  防止"看似 re-export 实则复制了一份"导致 ``isinstance``/``is`` 判空失效。
* **薄 stub 正确转发** —— 主类同名方法把首参 ``self`` 作为 ``planner``
  传给对应模块级函数（monkeypatch 契约，见 tests/test_incremental.py）。
* **速度规划数值不回归** —— 搬运后梯形/S 曲线时间公式结果不变。

运行::

    pytest tests/test_service_planner.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from service import planner, planner_expand, planner_math, planner_types  # noqa: E402


# --------------------------------------------------------------------------- #
# 公开导入面
# --------------------------------------------------------------------------- #


def test_public_import_surface():
    """外部模块在用的符号，全部仍可从 service.planner 导入。"""
    from service.planner import (  # noqa: F401
        DEFAULT_ANGULAR_RES,
        ENVELOPE_TOLERANCE_MM,
        PROFILE_SCURVE,
        PROFILE_TRAPEZOID,
        SEVERITY_ERROR,
        SEVERITY_WARNING,
        SINGULARITY_COND,
        VALID_PROFILES,
        AxisLimits,
        Issue,
        PlanningError,
        ProgramReport,
        TrajectoryPlanner,
        _synchronized_time,
        s_curve_time,
        trapezoid_time,
    )


# --------------------------------------------------------------------------- #
# re-export 同一性
# --------------------------------------------------------------------------- #


def test_reexport_identity_types():
    for name in (
        "PlanningError",
        "AxisLimits",
        "Issue",
        "ProgramReport",
        "SEVERITY_ERROR",
        "SEVERITY_WARNING",
        "DEFAULT_ANGULAR_RES",
        "ENVELOPE_TOLERANCE_MM",
    ):
        assert getattr(planner, name) is getattr(planner_types, name), name


def test_reexport_identity_math():
    for name in (
        "trapezoid_time",
        "s_curve_time",
        "_synchronized_time",
        "PROFILE_TRAPEZOID",
        "PROFILE_SCURVE",
        "VALID_PROFILES",
    ):
        assert getattr(planner, name) is getattr(planner_math, name), name
    assert planner.SINGULARITY_COND is planner_expand.SINGULARITY_COND


# --------------------------------------------------------------------------- #
# 薄 stub 转发
# --------------------------------------------------------------------------- #


def test_stub_forwards_to_expand_module(monkeypatch):
    """``TrajectoryPlanner._expand_joint`` 必须把 self 作为首参转发给模块函数。"""
    calls = []

    def fake_expand_joint(planner, q_start, motion, index):
        calls.append((planner, q_start, motion, index))
        return "sentinel"

    monkeypatch.setattr(planner_expand, "_expand_joint", fake_expand_joint)
    bare = object.__new__(planner.TrajectoryPlanner)  # 裸实例：stub 不碰任何属性
    sentinel = object()
    assert bare._expand_joint("q0", "motion", 7) == "sentinel"
    assert calls == [(bare, "q0", "motion", 7)]
    assert sentinel is not None  # 占位，避免误删 calls 断言


def test_stub_forwards_to_dynamics_module(monkeypatch):
    """``check_trajectory_collisions`` 经 planner_dynamics 转发，首参为实例本身。"""
    from service import planner_dynamics

    calls = []

    def fake_check(planner, traj, **kwargs):
        calls.append((planner, traj, kwargs))
        return []

    monkeypatch.setattr(planner_dynamics, "check_trajectory_collisions", fake_check)
    bare = object.__new__(planner.TrajectoryPlanner)
    assert bare.check_trajectory_collisions("traj") == []
    # 首参是实例本身；stub 的默认关键字参数原样透传
    assert calls == [
        (bare, "traj", {"margin": -0.5, "stride": 0, "max_joint_step": None})
    ]


# --------------------------------------------------------------------------- #
# 数值回归
# --------------------------------------------------------------------------- #


def test_profile_time_formulas_regression():
    """梯形/S 曲线时间公式搬运后数值关系不变（参考 tests/test_scurve.py）。"""
    d, v, a = 500.0, 500.0, 2000.0
    t_s = planner.s_curve_time(d, v, a)
    t_t = planner.trapezoid_time(d, v, a)
    assert t_t > 0
    assert t_s > t_t
    assert t_s < t_t * 1.6
    assert planner.s_curve_time(0.0, v, a) == 0.0
    assert planner.s_curve_time(d, 0.0, a) == 0.0
