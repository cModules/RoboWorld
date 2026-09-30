"""力控柔顺（v1.0.2）的测试：阻抗 / 导纳规划与品牌力控指令。

守的是「力控方案在仿真里算得清、出码后真机控制器认」：

* 串联系刚度 ``k_eff = k_r·k_e/(k_r+k_e)`` 与力/位移换算（力控精度的第一性原理）
* 导纳仿真（数值积分）与解析阶跃响应**两套独立算法**互相印证
* ``to_ur_force_mode`` 单位换算（mm→m、deg/s→rad/s）逐字段核对
* ``ForceMode`` 进程序后对规划/节拍**完全透明**（与 IO 指令同一契约）
* UR 原生 ``force_mode()`` / ABB ``FCRefForce`` / 其余品牌降级 + TODO
* ``validate`` 配对检查：多余的 end 报错、程序结束仍在力控给警告

运行::

    pytest tests/test_compliance.py -q
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from service.compliance import (
    AXIS_NAMES,
    ComplianceSpec,
    check_force_capacity,
    contact_force,
    force_sensitivity,
    predict_contact_force,
    required_depth,
    series_stiffness,
    simulate_admittance,
    wrench_joint_torques,
)
from service.planner import SEVERITY_ERROR, SEVERITY_WARNING  # noqa: E402
from domain.pose import Pose  # noqa: E402
from adapter.post import get_post_processor  # noqa: E402
from adapter.post.base import PostProcessError  # noqa: E402
from service.program import Motion, MotionType, Program, ProgramBuilder  # noqa: E402
from domain.robot import Robot  # noqa: E402

URDF = REPO_ROOT / "assets" / "models" / "kuka" / "kr16_2" / "kr16_2.urdf"


@pytest.fixture
def robot() -> Robot:
    if not URDF.exists():
        pytest.skip(f"缺少测试用 URDF: {URDF}")
    return Robot(str(URDF), "kr16_2")


def _codes(severity):
    return {SEVERITY_ERROR: "error", SEVERITY_WARNING: "warning"}[severity]


# --------------------------------------------------------------------------- #
# 串联系刚度：力控精度的第一性原理
# --------------------------------------------------------------------------- #


def test_series_stiffness_value():
    """k_r=10, k_e=100 → k_eff = 10*100/110 ≈ 9.09（被软的一方拖住）。"""
    assert series_stiffness(10.0, 100.0) == pytest.approx(1000.0 / 110.0)


def test_series_stiffness_limits():
    """极限行为：工件远硬于机器人 → k_eff≈k_r；工件远软 → k_eff≈k_e。"""
    assert series_stiffness(10.0, 1e6) == pytest.approx(10.0, rel=1e-4)
    assert series_stiffness(1e6, 10.0) == pytest.approx(10.0, rel=1e-4)


def test_series_stiffness_symmetric():
    assert series_stiffness(3.0, 7.0) == pytest.approx(series_stiffness(7.0, 3.0))


@pytest.mark.parametrize("kr,ke", [(0.0, 10.0), (-1.0, 10.0), (10.0, 0.0), (10.0, -1.0)])
def test_series_stiffness_rejects_nonpositive(kr, ke):
    with pytest.raises(ValueError):
        series_stiffness(kr, ke)


def test_contact_force_and_required_depth_roundtrip():
    """F = k_eff·Δ 与反解 Δ = F/k_eff 互为逆运算。"""
    k_eff = series_stiffness(8.0, 120.0)
    assert required_depth(contact_force(8.0, 120.0, 2.5), 8.0, 120.0) == pytest.approx(2.5)
    assert contact_force(8.0, 120.0, 2.5) == pytest.approx(k_eff * 2.5)


def test_force_sensitivity_is_series_stiffness():
    """力对位置误差的敏感度在数值上就是 k_eff（同一个数、两种解读）。"""
    assert force_sensitivity(10.0, 40.0) == pytest.approx(series_stiffness(10.0, 40.0))


# --------------------------------------------------------------------------- #
# 单轴动力学参数
# --------------------------------------------------------------------------- #


def test_axis_compliance_damping_and_frequency():
    from service.compliance import AxisCompliance, _MM_TO_M

    ac = AxisCompliance(stiffness=10.0, damping_ratio=0.9, mass=0.5, si_factor=_MM_TO_M)
    k_si = 10.0 * _MM_TO_M  # N/m
    assert ac.damping == pytest.approx(2 * 0.9 * math.sqrt(k_si * 0.5))
    assert ac.natural_frequency == pytest.approx(math.sqrt(k_si / 0.5))
    assert ac.static_offset(30.0) == pytest.approx(3.0)  # F/k = 30/10 mm
    assert ac.force_for_offset(3.0) == pytest.approx(30.0)


def test_axis_compliance_rejects_bad_params():
    from service.compliance import AxisCompliance

    for kwargs in (
        dict(stiffness=0.0),
        dict(stiffness=-1.0),
        dict(mass=0.0),
        dict(damping_ratio=0.0),
    ):
        with pytest.raises(ValueError):
            AxisCompliance(**kwargs)


# --------------------------------------------------------------------------- #
# 导纳仿真（力 → 位移）与阻抗预测（位移 → 力）
# --------------------------------------------------------------------------- #


def test_admittance_matches_analytic_step_response():
    """数值积分 vs 解析阶跃响应：两条独立算法必须给出同一条曲线。"""
    from service.compliance import AxisCompliance

    spec = ComplianceSpec.constant_force("z", -30.0, stiffness=10.0)
    ac = spec.axis_compliance("z")
    res = simulate_admittance(spec, -30.0, duration=1.0, dt=2e-4)
    t_a, x_a = ac.step_response(-30.0, duration=1.0, dt=1e-4)
    x_num = res.axis_curve("z")
    # 时间轴不同步，按稳态与末段抽样比对
    assert res.steady_state_offset("z") == pytest.approx(-3.0, abs=1e-3)
    assert x_a[-1] == pytest.approx(-3.0, abs=1e-6)
    idx = np.searchsorted(t_a, 0.5)
    x_num_at = x_num[int(round(0.5 / 2e-4))]
    assert x_num_at == pytest.approx(x_a[idx], rel=5e-3, abs=1e-4)


def test_admittance_underdamped_overshoots():
    """ζ=0.2 欠阻尼必须有超调；ζ=0.9 几乎无超调。"""
    soft = ComplianceSpec.constant_force("z", -50.0, damping_ratio=0.2)
    stiff_damped = ComplianceSpec.constant_force("z", -50.0, damping_ratio=0.9)
    r1 = simulate_admittance(soft, -50.0, duration=2.0, dt=5e-4)
    r2 = simulate_admittance(stiff_damped, -50.0, duration=2.0, dt=5e-4)
    assert r1.overshoot_percent("z") > 10.0
    assert r2.overshoot_percent("z") < 2.0


def test_admittance_position_axes_stay_zero():
    """位置控轴不参与柔顺，恒为 0——那里由位置环决定。"""
    spec = ComplianceSpec.constant_force("z", -30.0)
    res = simulate_admittance(spec, np.array([10.0, 10.0, -30.0, 1.0, 1.0, 1.0]), duration=0.5)
    assert abs(res.offsets[:, 0]).max() == pytest.approx(0.0)
    assert abs(res.offsets[:, 3]).max() == pytest.approx(0.0)
    assert abs(res.steady_state_offset("z") + 3.0) < 1e-3


def test_admittance_rejects_no_compliant_axis_and_bad_time():
    spec = ComplianceSpec(selection=(0, 0, 0, 0, 0, 0))
    with pytest.raises(ValueError):
        simulate_admittance(spec, -30.0)
    with pytest.raises(ValueError):
        simulate_admittance(ComplianceSpec.constant_force("z"), -30.0, duration=0.0)


def test_admittance_time_varying_force_and_axis_force():
    """力可以是时间函数；axis_force 把标量力打到指定轴。"""
    spec = ComplianceSpec.constant_force("z", -30.0)
    res = simulate_admittance(spec, lambda t: -30.0 * (1.0 - math.exp(-5 * t)), duration=1.5)
    assert res.steady_state_offset("z") == pytest.approx(-3.0, abs=0.05)

    res2 = simulate_admittance(
        ComplianceSpec(selection=(1, 0, 0, 0, 0, 0), stiffness=(5.0,) * 6),
        -10.0,
        duration=1.0,
        axis_force="x",
    )
    assert res2.steady_state_offset("x") == pytest.approx(-2.0, abs=1e-3)


def test_predict_contact_force_series_stiffness():
    """阻抗预测：F = k_eff·(depth − error)；位置误差直接吃掉压入量。"""
    spec = ComplianceSpec.constant_force("z", -30.0, stiffness=10.0)
    k_eff = series_stiffness(10.0, 100.0)
    f = predict_contact_force(spec, 100.0, surface_error=0.0, commanded_depth=3.0)
    assert f[2] == pytest.approx(k_eff * 3.0)
    f_err = predict_contact_force(spec, 100.0, surface_error=1.0, commanded_depth=3.0)
    assert f_err[2] == pytest.approx(k_eff * 2.0)
    assert f[0] == 0.0 and f[5] == 0.0  # 非柔顺轴不产力


# --------------------------------------------------------------------------- #
# 坐标系换算与关节载荷
# --------------------------------------------------------------------------- #


def test_wrench_in_base_rotates_by_frame():
    """绕基座 Z 转 90° 的任务系里 +X 方向 10N，在基座系里应指向 +Y。"""
    spec = ComplianceSpec(
        frame=Pose.from_xyz_rpy(100.0, 0.0, 0.0, 0.0, 0.0, 90.0),
        selection=(1, 0, 0, 0, 0, 0),
        wrench=(10.0, 0, 0, 0, 0, 0),
    )
    w = spec.wrench_in_base()
    assert w[0] == pytest.approx(0.0, abs=1e-9)
    assert w[1] == pytest.approx(10.0, abs=1e-9)


def test_wrench_joint_torques_matches_JtF(robot):
    """τ = Jᵀ·F：直接对雅可比做交叉验证（帧内力要先旋到基座系）。"""
    q = [10.0, -60.0, 80.0, -45.0, 90.0, 0.0]
    wrench = np.array([5.0, 0.0, -20.0, 0.5, 0.0, 0.0])
    frame = Pose.from_xyz_rpy(0, 0, 0, 0, 0, 30.0)
    tau = wrench_joint_torques(robot, q, wrench, frame)
    rot = np.asarray(frame.matrix[:3, :3])
    w_base = np.concatenate([rot @ wrench[:3], rot @ wrench[3:]])
    jac = robot.kinematics.jacobian(np.radians(np.asarray(q)))
    assert np.allclose(tau, jac.T @ w_base, atol=1e-9)


def test_wrench_joint_torques_rejects_bad_wrench(robot):
    with pytest.raises(ValueError):
        wrench_joint_torques(robot, [0] * 6, [1.0, 2.0])


def test_check_force_capacity_over_limit_errors(robot):
    """接触力大到离谱必须报 error；正常量级给通过摘要（warning 性质）。"""
    q = [10.0, -60.0, 80.0, -45.0, 90.0, 0.0]
    issues = check_force_capacity(robot, q, [0.0, 0.0, -50000.0, 0, 0, 0])
    assert any(i.code == "force_torque_limit" and i.severity == SEVERITY_ERROR for i in issues)

    ok = check_force_capacity(robot, q, [0.0, 0.0, -5.0, 0, 0, 0])
    assert any(i.code == "force_capacity_ok" for i in ok)


def test_check_force_capacity_without_kinematics():
    class NoKin:
        name = "ghost"
        kinematics = None

    issues = check_force_capacity(NoKin(), [0] * 6, [0] * 6)
    assert len(issues) == 1 and issues[0].code == "force_capacity_skip"


# --------------------------------------------------------------------------- #
# ComplianceSpec：构造 / limits / 校验 / 出码参数
# --------------------------------------------------------------------------- #


def test_constant_force_construction():
    spec = ComplianceSpec.constant_force("z", -30.0, stiffness=8.0)
    assert spec.selection == (0, 0, 1, 0, 0, 0)
    assert spec.wrench[2] == -30.0
    assert spec.stiffness[2] == 8.0
    assert spec.compliant_axes == (2,)
    assert spec.position_axes == (0, 1, 3, 4, 5)


def test_spec_rejects_bad_shapes():
    with pytest.raises(ValueError):
        ComplianceSpec(selection=(1, 0, 0))
    with pytest.raises(ValueError):
        ComplianceSpec(wrench=(1, 2, 3))
    with pytest.raises(ValueError):
        ComplianceSpec(limits=(1, 2, 3, 4, 5))


def test_effective_limits_and_units():
    """默认 limits：柔顺轴 = 速度上限，非柔顺轴 = 允许偏差（UR 手册语义）。"""
    spec = ComplianceSpec.constant_force("z")
    lims = spec.effective_limits()
    units = spec.limit_units()
    assert lims[2] == 100.0 and units[2] == "mm/s"  # 柔顺线轴：速度上限
    assert lims[0] == 10.0 and units[0] == "mm"  # 位置线轴：允许偏差
    assert lims[3] == 5.0 and units[3] == "deg"  # 位置角轴：允许偏差
    assert lims[5] == 5.0 and units[5] == "deg"

    spec2 = ComplianceSpec.constant_force("rx", limits=(1, 2, 3, 4, 5, 6))
    assert spec2.effective_limits() == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)


def test_validate_reports_no_compliant_axis_and_rigid_wrench():
    empty = ComplianceSpec(selection=(0, 0, 0, 0, 0, 0))
    issues = {i.code for i in empty.validate()}
    assert "no_compliant_axis" in issues

    spec = ComplianceSpec.constant_force("z", -30.0)
    spec.wrench = (0.0, 5.0, -30.0, 0, 0, 0)  # x 是位置控却给了力
    codes = [i.code for i in spec.validate()]
    assert "wrench_on_rigid_axis" in codes
    assert "no_compliant_axis" not in codes


def test_validate_reports_bad_stiffness_instead_of_crashing():
    """刚度非法 → 返回带诊断的 Issue 列表，不得抛异常。"""
    spec = ComplianceSpec.constant_force("z", -30.0)
    spec.stiffness = (10.0, 10.0, 0.0, 5.0, 5.0, 5.0)
    issues = spec.validate()
    assert any(i.code == "bad_stiffness" and i.severity == SEVERITY_ERROR for i in issues)


def test_validate_force_too_sensitive_with_env_stiffness():
    """工件很硬（500 N/mm）时 10 N/mm 的机器人刚度 → k_eff≈9.8，必须预警。"""
    spec = ComplianceSpec.constant_force("z", -30.0, stiffness=10.0)
    codes = [i.code for i in spec.validate(env_stiffness=500.0)]
    assert "force_too_sensitive" in codes


def test_design_report_contains_key_numbers():
    spec = ComplianceSpec.constant_force("z", -30.0, stiffness=10.0)
    report = spec.design_report(env_stiffness=100.0)
    assert "k_eff" in report and "标称压入" in report
    assert required_depth(30.0, 10.0, 100.0) == pytest.approx(30.0 / series_stiffness(10.0, 100.0))


def test_to_ur_force_mode_units():
    """逐字段核对 UR 单位：mm→m、deg/s→rad/s、deg→rad、力原样。"""
    spec = ComplianceSpec(
        frame=Pose.from_xyz_rpy(100.0, 50.0, 0.0, 0.0, 0.0, 90.0),
        selection=(0, 0, 1, 0, 0, 0),
        wrench=(0.0, 0.0, -30.0, 0, 0, 0),
        type=2,
        limits=(10.0, 10.0, 100.0, 5.0, 5.0, 30.0),
        stiffness=(10.0, 10.0, 10.0, 5.0, 5.0, 5.0),
    )
    fm = spec.to_ur_force_mode()
    assert fm["task_frame"][:3] == pytest.approx([0.1, 0.05, 0.0], abs=1e-9)
    # 90° 绕 Z 的旋转向量 = [0, 0, π/2]
    assert fm["task_frame"][5] == pytest.approx(math.pi / 2, abs=1e-9)
    assert fm["wrench"] == pytest.approx([0.0, 0.0, -30.0, 0, 0, 0])
    assert fm["type"] == 2
    assert fm["limits"][2] == pytest.approx(0.1, abs=1e-9)  # 100 mm/s → 0.1 m/s
    assert fm["limits"][5] == pytest.approx(math.radians(30.0), abs=1e-9)  # deg/s → rad/s
    assert fm["limits"][0] == pytest.approx(0.01, abs=1e-9)  # 10 mm → 0.01 m
    assert fm["selection_vector"] == [0, 0, 1, 0, 0, 0]


def test_static_pose_offsets_in_task_frame():
    """稳态让步发生在任务坐标系：任务系绕 Z 转 90° 后，z 轴让步仍是 z 向。"""
    spec = ComplianceSpec(
        frame=Pose.from_xyz_rpy(0.0, 0.0, 0.0, 0.0, 0.0, 90.0),
        selection=(0, 0, 1, 0, 0, 0),
        wrench=(0.0, 0.0, -30.0, 0, 0, 0),
    )
    pose = spec.static_pose()
    xyz = pose.to_xyz_rpy()[:3]
    assert xyz[2] == pytest.approx(-3.0, abs=1e-9)
    assert abs(xyz[0]) < 1e-9 and abs(xyz[1]) < 1e-9


# --------------------------------------------------------------------------- #
# Program / ProgramBuilder 集成
# --------------------------------------------------------------------------- #


def _force_program(robot) -> Program:
    spec = ComplianceSpec.constant_force("z", -30.0, stiffness=8.0)
    b = ProgramBuilder(robot)
    b.moveJ([0, -90, 0, -90, 0, 0], speed=30)
    b.force_mode(spec)
    b.moveJ([0, -90, -10, -90, 0, 0], speed=30)
    b.force_mode_end()
    b.moveJ([0, -90, 0, -90, 0, 0], speed=30)
    return b.build()


def test_force_mode_is_transparent_to_planning(robot):
    """力控指令不是运动：节拍/路点数与不含力控的同款程序完全一致。"""
    spec = ComplianceSpec.constant_force("z", -30.0, stiffness=8.0)
    with_fm = _force_program(robot)

    b = ProgramBuilder(robot)
    b.moveJ([0, -90, 0, -90, 0, 0], speed=30)
    b.moveJ([0, -90, -10, -90, 0, 0], speed=30)
    b.moveJ([0, -90, 0, -90, 0, 0], speed=30)
    without_fm = b.build()

    t1, t2 = with_fm.plan(), without_fm.plan()
    assert t1.duration() == pytest.approx(t2.duration(), rel=1e-9)
    assert len(t1.waypoints) == len(t2.waypoints)
    assert len(with_fm.motions) == 3  # motions 属性只含运动
    assert with_fm.items[1].spec.wrench[2] == -30.0  # spec 原样挂在指令上


def test_builder_force_mode_chain_and_then(robot):
    spec = ComplianceSpec.constant_force("z", -30.0)
    b = ProgramBuilder(robot)
    b.moveJ([0, -90, 0, -90, 0, 0], speed=30)
    b.force_mode(spec)
    b.force_mode_end()

    other = Program(robot)
    other.add(Motion(MotionType.JOINTS, [0, -90, 0, -90, 0, 0], speed=30))
    merged = b.then(other).build()
    kinds = [type(i).__name__ for i in merged.items]
    assert kinds == ["Motion", "ForceMode", "ForceMode", "Motion"]


def test_validate_force_mode_pairing(robot):
    """多余的 end 报错；未退出的力控给警告；合法配对无力控相关 issue。"""
    from service.program import ForceMode

    prog = _force_program(robot)
    report = prog.validate(check_dynamics=False)
    assert not [i for i in report.issues if i.code.startswith("force_mode")]

    bad = Program(robot)
    bad.add(Motion(MotionType.JOINTS, [0, -90, 0, -90, 0, 0], speed=30))
    bad.items.append(ForceMode(end=True))  # 多余的退出
    codes = [i.code for i in bad.validate(check_dynamics=False).issues]
    assert "force_mode_unbalanced" in codes

    leak = Program(robot)
    leak.add(Motion(MotionType.JOINTS, [0, -90, 0, -90, 0, 0], speed=30))
    leak.force_mode(ComplianceSpec.constant_force("z", -30.0))
    leak.add(Motion(MotionType.JOINTS, [0, -90, -10, -90, 0, 0], speed=30))
    codes = [i.code for i in leak.validate(check_dynamics=False).issues]
    assert "force_mode_active_at_end" in codes


def test_validate_propagates_spec_issues(robot):
    """spec 自身的诊断（如无柔顺轴）应浮到程序校验报告里。"""
    prog = Program(robot)
    prog.add(Motion(MotionType.JOINTS, [0, -90, 0, -90, 0, 0], speed=30))
    prog.force_mode(ComplianceSpec(selection=(0, 0, 0, 0, 0, 0)))
    codes = [i.code for i in prog.validate(check_dynamics=False).issues]
    assert "no_compliant_axis" in codes


# --------------------------------------------------------------------------- #
# 品牌出码
# --------------------------------------------------------------------------- #


def test_ur_native_force_mode(robot):
    prog = _force_program(robot)
    code = prog.to_post("ur_polyscope")
    assert "force_mode([" in code
    assert "[0, 0, 1, 0, 0, 0]" in code
    assert "-30.00000" in code
    assert "end_force_mode()" in code
    # 顺序：force_mode 在 end_force_mode 之前
    assert code.index("force_mode(") < code.index("end_force_mode()")


def test_abb_force_control_output(robot):
    prog = _force_program(robot)
    code = prog.to_post("abb_irc5")
    assert "FCRefForce [0.000, 0.000, -30.000, 0.000, 0.000, 0.000];" in code
    assert "FCAct;" in code
    assert "FCDeact;" in code
    assert "TODO" in code  # 实参语法随选件版本而变，必须提示人工核对


def test_kuka_fanuc_fallback_with_todo(robot):
    prog = _force_program(robot)
    kuka = prog.to_post("kuka_krc4")
    assert "FORCE_MODE_START(" in kuka and "FORCE_MODE_END()" in kuka
    assert "TODO" in kuka
    fanuc = prog.to_post("fanuc_r30ib")
    assert "FORCE_MODE_START(" in fanuc and "TODO" in fanuc


def test_force_mode_without_spec_raises(robot):
    prog = Program(robot)
    prog.add(Motion(MotionType.JOINTS, [0, -90, 0, -90, 0, 0], speed=30))
    from service.program import ForceMode

    prog.items.append(ForceMode())  # 进入力控却没带 spec
    pp = get_post_processor("ur_polyscope")
    with pytest.raises(PostProcessError):
        pp.generate(prog.items, metadata={"program_name": "t"})


def test_builder_force_mode_kwargs_shortcut(robot):
    """不给 spec 时用关键字直接构造单轴恒力。"""
    b = ProgramBuilder(robot)
    b.moveJ([0, -90, 0, -90, 0, 0], speed=30)
    b.force_mode(selection=(0, 0, 1, 0, 0, 0), wrench=(0, 0, -30, 0, 0, 0))
    prog = b.build()
    assert prog.items[1].spec.wrench[2] == -30.0
    with pytest.raises(ValueError):
        b.force_mode(ComplianceSpec.constant_force("z"), wrench=(0, 0, -1, 0, 0, 0))


# --------------------------------------------------------------------------- #
# PLC 步序导出：力控是机器人侧的事
# --------------------------------------------------------------------------- #


def test_plc_scl_ignores_force_mode(robot):
    from service.plc_scl import steps_from_program

    prog = _force_program(robot)
    prog.setDO("gripper_close", True)
    steps = steps_from_program(prog.items)
    assert steps  # SetDO 正常进步序
    text = "\n".join(getattr(s, "name", "") for s in steps)
    assert "FORCE" not in text
