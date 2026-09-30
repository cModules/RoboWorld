"""v1.1.8 规模与性能基准 + 正运动学常量缓存的等价性守卫。

这个文件盯的是两件事：

1. **优化不许改变结果**。v1.1.8 给正运动学加了关节原点旋转/轴向的记忆化，
   并把 :meth:`TrajectoryPlanner.motion_at` 从顺序扫描换成二分。两者都属"等价
   改写"，因此测试里放了一份**独立写法的朴素实现**（:func:`reference_link_transforms`
   与文件内的线性扫描），逐位比对——只用"结果看起来对"是守不住的。
2. **基准工具本身可信**。:func:`profile_program` / :func:`scale_report` 报出来的
   数字要被用在文档与网页上，所以它们的字段口径、best-of 语义、CLI 行为、
   en 模式下无中文残留都要有断言。
"""

from __future__ import annotations

import json
import math
import re
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from domain.robot import Robot  # noqa: E402
from support import i18n  # noqa: E402
from domain import kinematics as kinmod  # noqa: E402
from service.perf import (
    PhaseTiming,
    PlanProfile,
    SWEEP_BASE,
    _best_of,
    format_scale_table,
    profile_program,
    reference_link_transforms,
    reference_rodrigues,
    scale_report,
    sweep_program,
)
from service.planner import TrajectoryPlanner  # noqa: E402
from domain.world.templates import build_grasping_workstation  # noqa: E402

UR5E = REPO_ROOT / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"
KR16 = REPO_ROOT / "assets" / "models" / "kuka" / "kr16_2" / "kr16_2.urdf"
IRB1600 = REPO_ROOT / "assets" / "models" / "abb" / "irb1600" / "irb1600.urdf"

_CJK = re.compile(r"[\u4e00-\u9fff]")


# --------------------------------------------------------------------------- #
# 夹具
# --------------------------------------------------------------------------- #


def make_robot(urdf: Path = UR5E, name: str = "UR5e") -> Robot:
    """建机器人并摆到基准位形。"""
    robot = Robot(str(urdf), name)
    robot.set_joints(list(SWEEP_BASE))
    return robot


@pytest.fixture
def ur5e():
    return make_robot()


@pytest.fixture(autouse=True)
def _fresh_cache():
    """每个用例前后清一次常量缓存，避免用例之间互相影响统计数字。"""
    kinmod.reset_cache()
    yield
    kinmod.reset_cache()


@pytest.fixture
def en_language():
    """临时切到英文界面（用例结束后还原；``save=False`` 不碰用户配置文件）。"""
    previous = i18n.current_language()
    i18n.set_language("en", save=False)
    try:
        yield
    finally:
        i18n.set_language(previous, save=False)


def _configs(kin, n=8, seed=7):
    """一组有代表性的关节角（弧度），含零位与限位附近。"""
    rng = np.random.default_rng(seed)
    out = [np.zeros(kin.dof), np.array(SWEEP_BASE) * math.pi / 180.0]
    for _ in range(n):
        out.append(np.radians(rng.uniform(-150.0, 150.0, size=kin.dof)))
    return out


# --------------------------------------------------------------------------- #
# 1. 等价性：朴素参考实现 vs 缓存实现
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize("urdf", [UR5E, KR16, IRB1600])
def test_link_transforms_match_reference_bitwise(urdf):
    """FK 逐位一致（不是"接近"）——缓存只是把同一个结果留下来。"""
    kin = Robot(str(urdf), "R").kinematics
    for q in _configs(kin):
        got = kin.link_transforms(q)
        ref = reference_link_transforms(kin, q)
        assert set(got) == set(ref)
        for name, T in got.items():
            assert np.array_equal(np.asarray(T), np.asarray(ref[name])), name


def test_forward_kinematics_matches_reference(ur5e):
    """末端的 4x4 逐位一致（robot 层的 mm/deg 包装不算数，比的是米/弧度）。"""
    kin = ur5e.kinematics
    tip = kin.chain[-1].child
    for q in _configs(kin):
        assert np.array_equal(kin.forward_kinematics(q), reference_link_transforms(kin, q)[tip])


def test_link_transforms_accepts_degrees(ur5e):
    """``q_is_radians=False`` 分支同样与参考实现一致（度 → 弧度换算在链内做）。"""
    kin = ur5e.kinematics
    q_deg = np.array(SWEEP_BASE, dtype=np.float64)
    got = kin.link_transforms(q_deg, q_is_radians=False)
    ref = reference_link_transforms(kin, np.radians(q_deg))
    for name, T in got.items():
        assert np.allclose(np.asarray(T), np.asarray(ref[name]), atol=1e-15, rtol=0)


def test_joint_origins_and_axes_match_reference(ur5e):
    """``joint_origins`` / ``joint_axes`` 也走了缓存路径，同样要比 (雅可比靠它们)。"""
    kin = ur5e.kinematics
    actuated = [j for j in kin.chain if j.movable]
    for q in _configs(kin, n=4):
        ref = reference_link_transforms(kin, q)
        origins = kin.joint_origins(q)
        axes = kin.joint_axes(q)
        assert len(origins) == len(actuated) == len(axes)
        for col, joint in enumerate(actuated):
            parent = np.asarray(ref[joint.parent], dtype=np.float64)
            origin = np.asarray(kin._origin_transform(joint), dtype=np.float64)
            T = parent @ origin
            assert np.allclose(origins[col], T[:3, 3])
            assert np.allclose(axes[col], T[:3, :3] @ np.asarray(joint.axis, dtype=np.float64))
        assert np.allclose(kin.jacobian(q), _reference_jacobian(kin, q))


def _reference_jacobian(kin, q) -> np.ndarray:
    """朴素雅可比（用参考实现的变换），用于交叉验证缓存路径没改变结果。

    刻意写成几何定义直译：转动关节的 ``J_v = z × (p_tip − p_joint)``、
    ``J_w = z``；移动关节 ``J_v = z``、``J_w = 0``。
    """
    ref = reference_link_transforms(kin, q)
    tip = ref[kin.chain[-1].child]
    p_tip = np.asarray(tip, dtype=np.float64)[:3, 3]
    J = np.zeros((6, kin.dof), dtype=np.float64)
    col = 0
    for joint in kin.chain:
        if not joint.movable:
            continue
        parent = np.asarray(ref[joint.parent], dtype=np.float64)
        origin_r = kinmod._rotmat_rpy(joint.origin_rpy)
        T = parent @ kinmod._transform(origin_r, joint.origin_xyz)
        z = T[:3, :3] @ np.asarray(joint.axis, dtype=np.float64)
        if joint.is_prismatic:
            J[:3, col] = z
            J[3:, col] = 0.0
        else:
            J[:3, col] = np.cross(z, p_tip - T[:3, 3])
            J[3:, col] = z
        col += 1
    return J


def test_jacobian_matches_reference(ur5e):
    """雅可比与朴素几何定义一致（三条轴的 FK 缓存都被这条覆盖）。"""
    kin = ur5e.kinematics
    for q in _configs(kin, n=4):
        assert np.allclose(kin.jacobian(q), _reference_jacobian(kin, q), atol=1e-12)


# --------------------------------------------------------------------------- #
# 2. 缓存本身的正确性
# --------------------------------------------------------------------------- #


def test_rotmat_rpy_is_memoized(ur5e):
    """同一个 rpy 第二次不再进 scipy。"""
    kinmod.reset_cache()
    rpy = np.array([0.1, -0.2, 0.3])
    first = kinmod._rotmat_rpy(rpy)
    stats = kinmod.cache_stats()
    assert stats["rotmat_computed"] == 1 and stats["rotmat_hits"] == 0
    second = kinmod._rotmat_rpy(rpy.copy())
    assert second is first  # 同一个实例：确实复用了
    stats = kinmod.cache_stats()
    assert stats["rotmat_computed"] == 1 and stats["rotmat_hits"] == 1


def test_rotmat_rpy_is_readonly(ur5e):
    """缓存矩阵只读——想改的调用方必须自己复制，否则会污染别的使用者。"""
    mat = kinmod._rotmat_rpy(np.array([0.0, 0.0, 0.0]))
    assert mat.flags.writeable is False
    with pytest.raises(ValueError):
        mat[0, 0] = 1.0


def test_rotmat_rpy_distinguishes_different_values():
    """不同 rpy 不能共用同一条缓存（键是真值字节，不是"看起来像"）。"""
    kinmod.reset_cache()
    a = kinmod._rotmat_rpy(np.array([0.1, 0.2, 0.3]))
    b = kinmod._rotmat_rpy(np.array([0.1, 0.2, 0.4]))
    assert not np.array_equal(a, b)
    assert kinmod.cache_stats()["rotmat_entries"] == 2


def test_rotmat_cache_is_bounded():
    """缓存触顶后整体清空，不会无限涨（有人逐帧喂随机 rpy 也不至于吃爆内存）。"""
    kinmod.reset_cache()
    limit = kinmod.ROTMAT_CACHE_LIMIT
    for i in range(limit + 50):
        kinmod._rotmat_rpy(np.array([i * 1e-3, 0.0, 0.0]))
    assert kinmod.cache_stats()["rotmat_entries"] <= limit


def test_origin_cache_hits_after_first_fk(ur5e):
    """第二次 FK 不再为任何关节重算原点变换。"""
    kin = ur5e.kinematics
    kin.reset_origin_cache()
    kinmod.reset_cache()
    kin.link_transforms(np.zeros(kin.dof))
    computed_once = kinmod.cache_stats()["rotmat_computed"]
    assert computed_once >= 1
    for _ in range(5):
        kin.link_transforms(np.zeros(kin.dof))
    assert kinmod.cache_stats()["rotmat_computed"] == computed_once


def test_fk_warm_cache_does_not_touch_scipy(ur5e, monkeypatch):
    """外部证据：预热之后 scipy 的欧拉角转换**一次都不再被调用**。

    直接数 scipy 调用，而不是数自己的计数器——这是"缓存确实省掉了真活"
    最不容易自欺的验证方式。
    """
    kin = ur5e.kinematics
    calls = {"n": 0}
    original = kinmod.Rotation.from_euler

    def counting(*args, **kwargs):
        calls["n"] += 1
        return original(*args, **kwargs)

    monkeypatch.setattr(kinmod.Rotation, "from_euler", staticmethod(counting))
    kin.link_transforms(np.zeros(kin.dof))  # 预热
    before = calls["n"]
    for q in _configs(kin, n=5):
        kin.link_transforms(q)
    assert calls["n"] == before


def test_origin_cache_invalidates_when_joint_origin_changes(ur5e):
    """关节原点被原地改动（`JointSpec` 冻结，但数组本身可写）→ 缓存必须失效。"""
    kin = ur5e.kinematics
    joint = kin.chain[1]
    kin.forward_kinematics(np.zeros(kin.dof))  # 先把缓存灌满
    joint.origin_xyz[2] += 0.05  # 冻结 dataclass 只能原地改数组内容
    got = kin.forward_kinematics(np.zeros(kin.dof))
    ref = reference_link_transforms(kin, np.zeros(kin.dof))[kin.chain[-1].child]
    assert np.array_equal(got, ref)


def test_origin_cache_invalidates_when_joint_rpy_changes(ur5e):
    """rpy 被原地改动同样必须失效——否则会静默用旧旋转矩阵算出错误位姿。"""
    kin = ur5e.kinematics
    joint = kin.chain[2]
    kin.forward_kinematics(np.zeros(kin.dof))
    joint.origin_rpy[2] += 0.3
    got = kin.forward_kinematics(np.zeros(kin.dof))
    ref = reference_link_transforms(kin, np.zeros(kin.dof))[kin.chain[-1].child]
    assert np.array_equal(got, ref)


def test_axis_cache_invalidates_when_axis_changes(ur5e):
    """转轴改了，单位化缓存也要失效。"""
    kin = ur5e.kinematics
    joint = kin.chain[3]
    kin.forward_kinematics(np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0]))
    joint.axis[:] = np.array([0.0, 1.0, 0.0])
    q = np.array([0.7, 0.1, 0.2, 0.3, 0.4, 0.5])
    got = kin.forward_kinematics(q)
    ref = reference_link_transforms(kin, q)[kin.chain[-1].child]
    assert np.array_equal(got, ref)


def test_caches_are_per_robot_not_shared():
    """两台不同机器人各自缓存，互不串味。"""
    a = Robot(str(UR5E), "A").kinematics
    b = Robot(str(KR16), "B").kinematics
    for kin in (a, b):
        for q in _configs(kin, n=3):
            assert np.allclose(
                kin.forward_kinematics(q),
                reference_link_transforms(kin, q)[kin.chain[-1].child],
            )


def test_reset_cache_keeps_results_correct(ur5e):
    """清缓存只影响速度，不影响结果。"""
    kin = ur5e.kinematics
    q = np.radians(np.array(SWEEP_BASE))
    first = kin.forward_kinematics(q)
    kinmod.reset_cache()
    kin.reset_origin_cache()
    assert np.array_equal(first, kin.forward_kinematics(q))


def test_reference_rodrigues_zero_length_axis():
    """退化轴：朴素实现返回单位矩阵（与生产实现对同一条规则）。"""
    assert np.array_equal(reference_rodrigues(np.zeros(3), 0.5), np.eye(3))
    assert np.array_equal(reference_rodrigues(np.array([0.0, 0.0, 1.0]), 0.0), np.eye(3))


def test_reference_rodrigues_rotates_as_expected():
    """绕 Z 轴 90° 把 X 轴送到 Y 轴——参考实现自身也要被验一次。"""
    R = reference_rodrigues(np.array([0.0, 0.0, 1.0]), math.pi / 2.0)
    assert np.allclose(R @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-12)


# --------------------------------------------------------------------------- #
# 3. motion_at 二分等价
# --------------------------------------------------------------------------- #


def _linear_motion_at(planner, t: float) -> int:
    """旧实现的线性扫描（测试内的独立参考）。"""
    spans = planner._motion_spans
    for i, (t0, t1) in enumerate(spans):
        if t0 <= t <= t1:
            return i
    return len(spans) - 1


def _program_with_failed_motion(robot):
    """含"规划失败指令"（零长区间）与"零行程指令"的程序——最容易把二分写错。"""
    from service.program import ProgramBuilder

    builder = ProgramBuilder(robot)
    base = list(SWEEP_BASE)
    builder.moveJ(base, speed=80)
    builder.moveJ([base[0] + 20.0] + base[1:], speed=80)
    builder.setDO("gripper_close", True)
    builder.moveJ([base[0] + 20.0] + base[1:], speed=80)  # 零行程 → 零长区间
    builder.moveJ([1e6, 0.0, 0.0, 0.0, 0.0, 0.0], speed=80)  # 越限 → 失败零长区间
    builder.waitTime(0.3)
    builder.moveJ([base[0] - 25.0] + base[1:], speed=80)
    builder.moveJ([base[0] - 25.0, base[1] + 15.0] + base[2:], speed=80)
    return builder.build()


def test_motion_at_matches_linear_scan_everywhere(ur5e):
    """时间轴扫一遍（含边界、负时刻、越界、跨零长区间）逐点与线性扫描一致。"""
    planner = TrajectoryPlanner(ur5e)
    program = _program_with_failed_motion(ur5e)
    traj = planner.plan(program)
    assert any(t0 == t1 for t0, t1 in planner._motion_spans), "用例应含零长区间"

    end = float(traj.waypoints[-1].timestamp)
    probes = [w.timestamp for w in traj.waypoints]
    probes += [0.0, -1.0, end, end + 5.0, end / 2.0, float("nan")]
    probes += list(np.linspace(-0.1, end + 0.1, 257))
    for t in probes:
        assert planner.motion_at(float(t)) == _linear_motion_at(planner, float(t)), t


def test_motion_at_returns_first_matching_span(ur5e):
    """零长区间紧跟正常区间时，边界时刻仍归**前一条**指令（与旧语义一致）。"""
    planner = TrajectoryPlanner(ur5e)
    program = _program_with_failed_motion(ur5e)
    planner.plan(program)
    spans = planner._motion_spans
    for i, (t0, t1) in enumerate(spans):
        if t0 == t1:
            assert planner.motion_at(t0) == _linear_motion_at(planner, t0)


def test_motion_at_empty_timeline(ur5e):
    """还没规划过时返回 -1（保持既有契约）。"""
    planner = TrajectoryPlanner(ur5e)
    planner._motion_spans = []
    assert planner.motion_at(0.0) == -1


def test_motion_at_handles_growing_timeline(ur5e):
    """同一个规划器连跑两条程序，区间缓存必须跟着换（长度/末元素都会变）。"""
    planner = TrajectoryPlanner(ur5e)
    for program in (_program_with_failed_motion(ur5e), _program_with_failed_motion(ur5e)):
        traj = planner.plan(program)
        for wp in traj.waypoints[::37]:
            assert planner.motion_at(float(wp.timestamp)) == _linear_motion_at(
                planner, float(wp.timestamp)
            )


# --------------------------------------------------------------------------- #
# 4. 基准工具
# --------------------------------------------------------------------------- #


def test_sweep_program_is_reproducible(ur5e):
    """同一个 seed 逐位可复现——基准数字要能被人按同一命令复算出来。"""
    a = sweep_program(ur5e, 8, seed=11)
    b = sweep_program(ur5e, 8, seed=11)
    c = sweep_program(ur5e, 8, seed=12)
    joints_a = [list(m.target) for m in a.motions]
    assert joints_a == [list(m.target) for m in b.motions]
    assert joints_a != [list(m.target) for m in c.motions]


def test_sweep_program_mixes_io_and_dwell(ur5e):
    """基准程序要覆盖真实形态：IO 与驻留混排，而不是纯净的运动序列。"""
    program = sweep_program(ur5e, 10)
    kinds = {type(it).__name__ for it in program.items}
    assert {"Motion", "SetDO", "WaitTime"} <= kinds


def test_sweep_program_rejects_negative(ur5e):
    with pytest.raises(ValueError):
        sweep_program(ur5e, -1)


def test_profile_program_reports_phases(ur5e):
    """分段计时：三段都在、量纲合理、路点数与节拍对得上。"""
    planner = TrajectoryPlanner(ur5e)
    program = sweep_program(ur5e, 12)
    profile = profile_program(planner, program, check_collision=False)

    assert isinstance(profile, PlanProfile)
    assert profile.n_motions == 12
    assert profile.n_commands > 12  # 含 IO / 驻留
    assert profile.n_waypoints > 12
    assert profile.cycle_time > 0.0
    assert profile.wall_s > 0.0
    assert profile.ms_per_waypoint > 0.0
    assert profile.phase("规划") is not None
    assert profile.phase("干涉检查") is not None
    assert profile.scene is False
    assert "未接场景" in profile.phase("干涉检查").note
    assert profile.repeats == 1
    assert profile.errors == 0


def test_profile_program_with_scene_times_interference(ur5e):
    """接了场景就把干涉检查单独计时，并给出接触处数。"""
    cell = build_grasping_workstation()
    planner = TrajectoryPlanner(ur5e, scene=cell.scene)
    program = sweep_program(ur5e, 6)
    profile = profile_program(planner, program, check_collision=True)
    phase = profile.phase("干涉检查")
    assert profile.scene is True
    assert phase.seconds > 0.0
    assert "接触" in phase.note


def test_profile_program_does_not_mutate_program(ur5e):
    """基准是只读的：跑完程序还得是原样（否则"测完再部署"会出大事）。"""
    planner = TrajectoryPlanner(ur5e)
    program = sweep_program(ur5e, 6)
    fingerprint = [(type(it).__name__, repr(getattr(it, "target", None))) for it in program.items]
    planner.plan(program)
    cycle_before = float(planner.plan(program).waypoints[-1].timestamp)
    profile_program(planner, program, check_collision=False)
    assert [
        (type(it).__name__, repr(getattr(it, "target", None))) for it in program.items
    ] == fingerprint
    assert profile_program(planner, program, check_collision=False).cycle_time == pytest.approx(
        cycle_before
    )


def test_profile_reports_cache_counters(ur5e):
    """报告里带上"关节原点旋转实算/命中"——优化是否真的省下工作的一手证据。"""
    planner = TrajectoryPlanner(ur5e)
    program = sweep_program(ur5e, 20)
    profile = profile_program(planner, program, check_collision=False)
    delta_computed = int(profile.cache_after["rotmat_computed"]) - int(
        profile.cache_before["rotmat_computed"]
    )
    delta_hits = int(profile.cache_after["rotmat_hits"]) - int(profile.cache_before["rotmat_hits"])
    # 这段基准跑了几万次 FK，实算次数必须远小于调用次数（缓存命中占绝大多数）
    assert delta_hits > 0
    assert delta_computed < 50
    assert "关节原点旋转" in profile.format()


def test_profile_to_dict_is_json_serializable(ur5e):
    planner = TrajectoryPlanner(ur5e)
    profile = profile_program(planner, sweep_program(ur5e, 5), check_collision=False)
    payload = json.loads(json.dumps(profile.to_dict(), ensure_ascii=False))
    assert payload["motions"] == 5
    assert payload["waypoints"] == profile.n_waypoints
    assert set(payload["phases"]) >= {"规划", "干涉检查", "校验（含内部重新规划）"}


def test_profile_format_is_readable(ur5e):
    planner = TrajectoryPlanner(ur5e)
    text = profile_program(planner, sweep_program(ur5e, 5), check_collision=False).format()
    assert "规模与性能基准" in text
    assert "UR5e" in text
    assert "ms/路点" in text or "单路点规划" in text


def test_best_of_returns_minimum():
    """:func:`_best_of` 取最小值，且调用次数正确（best-of 语义不能靠猜）。"""
    calls = {"n": 0}

    def fn():
        calls["n"] += 1
        return calls["n"]

    seconds, result = _best_of(fn, 3)
    assert calls["n"] == 3 and result == 3
    assert seconds >= 0.0


def test_best_of_keeps_fastest_run():
    """第一次故意慢、后两次快 → 返回值必须取快的（否则平均值会把抖动算进去）。"""
    import time as _time

    calls = {"n": 0}

    def fn():
        calls["n"] += 1
        if calls["n"] == 1:
            _time.sleep(0.05)
        return calls["n"]

    seconds, _ = _best_of(fn, 3)
    assert seconds < 0.04


def test_phase_timing_format_contains_note():
    ph = PhaseTiming("planning", 0.0123, "note")
    assert "12.3" in ph.format() and "note" in ph.format()


def test_scale_report_rows_grow_with_size(ur5e):
    """规模表：指令越多路点越多、耗时越长（单调性是这张表存在的前提）。"""
    report = scale_report(
        sweep_program,
        planner_factory=lambda: TrajectoryPlanner(make_robot()),
        sizes=(6, 18),
        repeats=1,
    )
    assert len(report.rows) == 2
    small, big = report.rows
    assert small.n_waypoints < big.n_waypoints
    assert small.plan_s < big.plan_s
    assert small.ms_per_waypoint > 0.0
    table = format_scale_table(report)
    assert "规模表" in table and "ms/路点" in table
    assert report.to_dict()["rows"][1]["motions"] == 18


def test_scale_report_uses_fresh_planner(ur5e):
    """每个规模新建规划器——共用会把上一条程序的规划器状态带进来，计时不可比。"""
    seen = []

    def factory():
        planner = TrajectoryPlanner(make_robot())
        seen.append(id(planner))
        return planner

    scale_report(sweep_program, planner_factory=factory, sizes=(4, 8), repeats=1)
    assert len(set(seen)) == len(seen) == 2


def test_scale_report_with_scene_records_collision(ur5e):
    cell = build_grasping_workstation()
    report = scale_report(
        sweep_program,
        planner_factory=lambda: TrajectoryPlanner(make_robot(), scene=cell.scene),
        sizes=(4,),
        repeats=1,
    )
    assert report.rows[0].collision_s > 0.0


# --------------------------------------------------------------------------- #
# 5. 真实速度对比（优化确实省掉了工作）
# --------------------------------------------------------------------------- #


def test_cached_fk_beats_reference_implementation(ur5e):
    """缓存实现在同一台机器上明显快于朴素实现。

    阈值刻意放到 1.2x（实测 4x+）：这条用例的目的是"优化被意外回退时报警"，
    不是精确跑分——机器负载会让数字浮动，写死 4x 会变成 flaky 测试。
    """
    import time as _time

    kin = ur5e.kinematics
    qs = _configs(kin, n=60) * 3

    for q in qs[:20]:  # 预热
        kin.forward_kinematics(q)
        reference_link_transforms(kin, q)

    t0 = _time.perf_counter()
    for q in qs:
        kin.forward_kinematics(q)
    cached = _time.perf_counter() - t0

    t0 = _time.perf_counter()
    for q in qs:
        reference_link_transforms(kin, q)
    naive = _time.perf_counter() - t0

    assert naive > cached * 1.2, f"缓存 FK={cached * 1000:.1f}ms 朴素={naive * 1000:.1f}ms"


def test_reference_link_transforms_rejects_wrong_dof(ur5e):
    with pytest.raises(ValueError):
        reference_link_transforms(ur5e.kinematics, np.zeros(3))


# --------------------------------------------------------------------------- #
# 6. i18n 与 CLI
# --------------------------------------------------------------------------- #


def test_profile_report_is_english_in_en_mode(ur5e, en_language):
    """en 模式下报告里不留中文（库层消息 i18n 的延续）。"""
    planner = TrajectoryPlanner(ur5e)
    text = profile_program(planner, sweep_program(ur5e, 5), check_collision=False).format()
    assert _CJK.search(text) is None, text


def test_scale_table_is_english_in_en_mode(ur5e, en_language):
    report = scale_report(
        sweep_program,
        planner_factory=lambda: TrajectoryPlanner(make_robot()),
        sizes=(4,),
        repeats=1,
    )
    assert _CJK.search(report.format()) is None, report.format()


def test_cli_json_output():
    """CLI 真实子进程验收：JSON 可解析、字段齐全。"""
    proc = subprocess.run(
        [sys.executable, "-m", "service.perf", "--motions", "6", "--sizes", "4,8", "--json"],
        cwd=str(REPO_ROOT),
        env={"PYTHONPATH": str(REPO_ROOT), "PATH": "/usr/bin:/bin"},
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert proc.returncode == 0, proc.stderr
    payload = json.loads(proc.stdout)
    assert payload["motions"] == 6
    assert payload["waypoints"] > 0
    assert payload["scale"]["rows"][1]["motions"] == 8


def test_cli_prints_human_report():
    proc = subprocess.run(
        [sys.executable, "-m", "service.perf", "--motions", "4"],
        cwd=str(REPO_ROOT),
        env={"PYTHONPATH": str(REPO_ROOT), "PATH": "/usr/bin:/bin"},
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert proc.returncode == 0, proc.stderr
    assert "规模与性能基准" in proc.stdout


def test_cli_missing_robot_returns_two():
    proc = subprocess.run(
        [sys.executable, "-m", "service.perf", "--robot", "/nonexistent/robot.urdf"],
        cwd=str(REPO_ROOT),
        env={"PYTHONPATH": str(REPO_ROOT), "PATH": "/usr/bin:/bin"},
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert proc.returncode == 2
    assert "URDF" in proc.stderr or "urdf" in proc.stderr


# --------------------------------------------------------------------------- #
# 7. 示例脚本（子进程真实运行）
# --------------------------------------------------------------------------- #


def test_example_benchmark_demo_runs():
    """``examples/perf_benchmark_demo.py`` 一条命令跑通，且等价性结论是真的。

    示例里"逐位相同 ✅"那句结论必须由它自己的输出推出——这里断言它也把
    差异量打了出来（0.00e+00），避免哪天实现真的偏了、结论却还在说相同。
    """
    proc = subprocess.run(
        [
            sys.executable,
            str(REPO_ROOT / "examples" / "perf_benchmark_demo.py"),
            "--motions",
            "8",
            "--sizes",
            "4,8",
        ],
        cwd=str(REPO_ROOT),
        env={"PYTHONPATH": str(REPO_ROOT), "PATH": "/usr/bin:/bin"},
        capture_output=True,
        text=True,
        timeout=600,
    )
    assert proc.returncode == 0, proc.stderr
    out = proc.stdout
    assert "位姿矩阵最大差异：0.00e+00" in out
    assert "等价性结论：逐位相同" in out
    assert "ms/路点" in out
    assert "口径说明" in out  # "这份基准不回答什么"必须写清楚
