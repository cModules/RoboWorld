"""动力学/力矩校验（v0.3.1）的测试。

覆盖 :mod:`domain.dynamics` 与规划器集成：

* 牛顿-欧拉逆动力学的物理正确性——单摆重力力矩解析值、负载力矩与
  ``Jᵀ·F`` 交叉验证、质量矩阵对称性、fixed 末端 link 合并
* 惯性参数双轨——URDF ``<inertial>`` 自动解析、``set_link_inertial``
  手动录入、``set_payload`` 末端负载、缺参连杆清单
* 力矩校验集成——报表字段、超额定 error（GUI 标红依据）、逼近额定
  warning、缺惯性参数时的跳过警告、``check_dynamics=False`` 静默
* GUI——力矩超限的错误级 Issue 把路点行标红（Qt offscreen）
"""

import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from domain.dynamics import (
    DEFAULT_JOINT_EFFORT,
    DynamicsModel,
    LinkInertial,
)
from domain.kinematics import RobotKinematics  # noqa: E402
from domain.pose import Pose  # noqa: E402
from service.program import ProgramBuilder  # noqa: E402
from domain.robot import Robot  # noqa: E402

G = 9.81

PENDULUM_URDF = """<?xml version="1.0"?>
<robot name="pend">
  <link name="base_link"/>
  <link name="arm">
    <inertial><mass value="2.0"/><origin xyz="0.3 0 0"/>
      <inertia ixx="0.01" iyy="0.02" izz="0.015"/></inertial>
  </link>
  <link name="tcp"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/><child link="arm"/>
    <origin xyz="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="50" velocity="3.0"/>
  </joint>
  <joint name="jt" type="fixed">
    <parent link="arm"/><child link="tcp"/><origin xyz="0.5 0 0"/>
  </joint>
</robot>"""

TWO_LINK_URDF = """<?xml version="1.0"?>
<robot name="two">
  <link name="base_link"/>
  <link name="l1"><inertial><mass value="1.5"/><origin xyz="0.25 0 0"/>
    <inertia ixx="0.01" iyy="0.02" izz="0.015"/></inertial></link>
  <link name="l2"><inertial><mass value="1.0"/><origin xyz="0.2 0 0"/>
    <inertia ixx="0.008" iyy="0.012" izz="0.01"/></inertial></link>
  <joint name="j1" type="revolute"><parent link="base_link"/><child link="l1"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="40" velocity="3.0"/></joint>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/>
    <origin xyz="0.5 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="30" velocity="3.0"/></joint>
</robot>"""


@pytest.fixture(scope="module")
def pendulum(tmp_path_factory):
    p = tmp_path_factory.mktemp("urdf") / "pendulum.urdf"
    p.write_text(PENDULUM_URDF)
    return RobotKinematics(p)


@pytest.fixture(scope="module")
def two_link(tmp_path_factory):
    p = tmp_path_factory.mktemp("urdf") / "two.urdf"
    p.write_text(TWO_LINK_URDF)
    return RobotKinematics(p)


# --------------------------------------------------------------------------- #
# 逆动力学物理正确性
# --------------------------------------------------------------------------- #


def test_pendulum_gravity_torque_matches_analytic(pendulum):
    """水平单摆：|τ| = m·g·L；摆到竖直方向力矩归零。"""
    model = DynamicsModel(pendulum)
    tau_horiz = model.inverse_dynamics([0.0], [0.0], [0.0])
    assert tau_horiz[0] == pytest.approx(-2.0 * G * 0.3, rel=1e-9)
    tau_vert = model.inverse_dynamics([math.pi / 2], [0.0], [0.0])
    assert abs(tau_vert[0]) < 1e-9


def test_payload_torque_cross_check_with_jacobian_transpose(pendulum):
    """末端负载的静态力矩差必须等于 Jᵀ·(0,0,m_p·g)。"""
    model = DynamicsModel(pendulum)
    tau0 = model.inverse_dynamics([0.0], [0.0], [0.0])
    model.set_payload(1.5)
    tau1 = model.inverse_dynamics([0.0], [0.0], [0.0])
    delta = tau1 - tau0
    J = pendulum.jacobian([0.0])
    expected = J[:3, 0] @ np.array([0.0, 0.0, 1.5 * G])
    assert float(delta[0]) == pytest.approx(float(expected), rel=1e-9)


def test_mass_matrix_symmetry(two_link):
    """M(q) 各列由 (q, 0, e_i, 无重力) 的逆动力学给出，必须对称。"""
    model = DynamicsModel(two_link)
    q = [0.3, -0.5]
    M = np.zeros((2, 2))
    for i in range(2):
        e = np.zeros(2)
        e[i] = 1.0
        M[:, i] = model.inverse_dynamics(q, [0.0, 0.0], e, gravity=[0, 0, 0])
    assert np.abs(M - M.T).max() < 1e-9
    assert np.all(np.diag(M) > 0)


def test_mass_matrix_diagonal_matches_parallel_axis(two_link):
    """腕关节（j2，轴过自身连杆）的 M₂₂ = I_yy + m·d²，手算可验。"""
    model = DynamicsModel(two_link)
    q = [0.3, -0.5]
    e = np.array([0.0, 1.0])
    M22 = model.inverse_dynamics(q, [0, 0], e, gravity=[0, 0, 0])[1]
    assert M22 == pytest.approx(0.012 + 1.0 * 0.2**2, rel=1e-9)


def test_inertial_tensor_contributes(two_link):
    """惯性张量非零时，角加速度产生附加力矩（I·α 项）。"""
    model = DynamicsModel(two_link)
    # 无重力、无速度，j2 单独 α=1：力矩 = I_yy_world ≥ I_yy
    tau = model.inverse_dynamics([0.0, 0.0], [0, 0], [0.0, 1.0], gravity=[0, 0, 0])
    assert tau[1] >= 0.012


def test_fixed_link_merged_into_chain(pendulum):
    """fixed 关节的末端 link 参与递推（惯性为零不改变力矩，但链完整）。"""
    model = DynamicsModel(pendulum)
    assert [j.name for j in model.chain] == ["j1", "jt"]
    # 末端 tcp 无惯性参数不算缺失（fixed 按零质量处理）
    assert model.has_inertial
    assert model.missing_links == []


def test_chain_property_exposes_main_chain(pendulum):
    assert len(pendulum.chain) == 2
    assert pendulum.chain[0].name == "j1"
    assert pendulum.chain[1].joint_type == "fixed"


# --------------------------------------------------------------------------- #
# 惯性参数双轨
# --------------------------------------------------------------------------- #


def test_manual_entry_reproduces_analytic_torque(tmp_path):
    """URDF 无惯性参数时手动录入，结果与解析值一致。"""
    urdf = """<?xml version="1.0"?>
<robot name="bare">
  <link name="base_link"/>
  <link name="arm"/>
  <joint name="j1" type="revolute"><parent link="base_link"/><child link="arm"/>
    <axis xyz="0 1 0"/><limit effort="50" velocity="3.0"/></joint>
</robot>"""
    p = tmp_path / "bare.urdf"
    p.write_text(urdf)
    kin = RobotKinematics(p)
    model = DynamicsModel(kin)
    assert not model.has_inertial
    assert model.missing_links == ["arm"]

    model.set_link_inertial("arm", 2.0, com=(0.3, 0, 0), source="datasheet")
    assert model.has_inertial
    tau = model.inverse_dynamics([0.0], [0.0], [0.0])
    assert tau[0] == pytest.approx(-2.0 * G * 0.3, rel=1e-9)


def test_link_inertial_merge_parallel_axis():
    """负载合并遵循平行轴定理：合成质心与张量手算可验。"""
    a = LinkInertial(1.0, np.array([0.0, 0.0, 0.0]), np.eye(3) * 0.01)
    b = LinkInertial.point_mass(1.0, np.array([0.4, 0.0, 0.0]))
    m = a.merged_with(b)
    assert m.mass == pytest.approx(2.0)
    assert m.com[0] == pytest.approx(0.2)
    # 沿 X 平移：Ixx 不变；Iyy/Izz 各加 m·d²（a：1·0.2²，b：1·0.2²）
    assert m.inertia[0, 0] == pytest.approx(0.01, rel=1e-9)
    assert m.inertia[1, 1] == pytest.approx(0.01 + 0.04 + 0.04, rel=1e-9)


def test_torque_limits_from_urdf_effort(two_link):
    model = DynamicsModel(two_link)
    assert model.torque_limits.tolist() == [40.0, 30.0]


def test_torque_limits_fallback_when_effort_missing(tmp_path):
    """URDF 不带 effort 时退回默认额定值，而不是抛异常。"""
    urdf = """<?xml version="1.0"?>
<robot name="noeffort">
  <link name="base_link"/>
  <link name="arm"><inertial><mass value="1.0"/><origin xyz="0.1 0 0"/>
    <inertia ixx="0" iyy="0" izz="0"/></inertial></link>
  <joint name="j1" type="revolute"><parent link="base_link"/><child link="arm"/>
    <axis xyz="0 1 0"/><limit velocity="3.0"/></joint>
</robot>"""
    p = tmp_path / "n.urdf"
    p.write_text(urdf)
    model = DynamicsModel(RobotKinematics(p))
    assert model.torque_limits.tolist() == [DEFAULT_JOINT_EFFORT]


# --------------------------------------------------------------------------- #
# 与规划器/报表集成
# --------------------------------------------------------------------------- #


def _overload_robot(tmp_path):
    """小额定力矩的三连杆机器人——任意大幅运动必超限。"""
    urdf = """<?xml version="1.0"?>
<robot name="weak">
  <link name="base_link"/>
  <link name="l1"><inertial><mass value="2.0"/><origin xyz="0.15 0 0"/>
    <inertia ixx="0.01" iyy="0.01" izz="0.01"/></inertial></link>
  <link name="l2"><inertial><mass value="1.5"/><origin xyz="0.12 0 0"/>
    <inertia ixx="0.008" iyy="0.008" izz="0.008"/></inertial></link>
  <link name="l3"><inertial><mass value="1.0"/><origin xyz="0.1 0 0"/>
    <inertia ixx="0.005" iyy="0.005" izz="0.005"/></inertial></link>
  <joint name="j1" type="revolute"><parent link="base_link"/><child link="l1"/>
    <origin xyz="0 0 0.2"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="0.5" velocity="3.0"/></joint>
  <joint name="j2" type="revolute"><parent link="l1"/><child link="l2"/>
    <origin xyz="0.3 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="0.5" velocity="3.0"/></joint>
  <joint name="j3" type="revolute"><parent link="l2"/><child link="l3"/>
    <origin xyz="0.25 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="0.5" velocity="3.0"/></joint>
</robot>"""
    p = tmp_path / "weak.urdf"
    p.write_text(urdf)
    return Robot(str(p), "Weak")


def test_validate_report_contains_torque_summary(tmp_path):
    """校验报表带力矩/功率字段，format 输出力矩小节。"""
    robot = _overload_robot(tmp_path)
    prog = ProgramBuilder(robot).moveJ([0.0, -60.0, 30.0]).moveJ([40.0, -30.0, 0.0]).build()
    report = prog.validate()
    assert report.torque_peaks is not None
    assert len(report.torque_peaks) == 3
    assert report.torque_limits == pytest.approx([0.5, 0.5, 0.5])
    assert report.peak_power is not None and report.peak_power > 0.0
    text = report.format()
    assert "力矩校验" in text
    assert "机械功率峰值" in text


def test_validate_flags_torque_overload_as_error(tmp_path):
    """超额定 → error 级 torque_limit（GUI 标红依据），报告不再 ok。"""
    robot = _overload_robot(tmp_path)
    prog = ProgramBuilder(robot).moveJ([0.0, -90.0, 45.0]).build()
    report = prog.validate()
    overloads = [i for i in report.errors if i.code == "torque_limit"]
    assert overloads, report.format()
    assert not report.ok
    for issue in overloads:
        assert "超出额定" in issue.message
        assert issue.motion_index == 0


def test_validate_near_limit_warning(tmp_path):
    """峰值达到额定 85% 以上但未超限时给黄色预警。"""
    robot = _overload_robot(tmp_path)
    # 把额定提到「重力+动态都够不到、又接近」的区间：手工设 30 N·m
    urdf_path = Path(robot.urdf_path)
    text = urdf_path.read_text().replace('effort="0.5"', 'effort="30.0"')
    urdf_path.write_text(text)
    robot2 = Robot(str(urdf_path), "Weak30")
    prog = ProgramBuilder(robot2).moveJ([0.0, -90.0, 45.0]).build()
    report = prog.validate()
    # 30 N·m 下大概率有轴落在 85%~100% 或干脆超不了；两种都不该是 error
    assert not [i for i in report.errors if i.code == "torque_limit"]
    near = [i for i in report.issues if i.code == "torque_near"]
    if report.torque_peaks is not None:
        ratios = [pk / lim for pk, lim in zip(report.torque_peaks, report.torque_limits)]
        if any(0.85 <= r <= 1.0 for r in ratios):
            assert near
        else:
            assert not near


def test_validate_skips_without_inertial():
    """UR5e 无惯性参数：力矩校验跳过并挂警告，不影响其余校验结论。"""
    urdf = REPO_ROOT / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"
    robot = Robot(str(urdf), "UR5e")
    prog = ProgramBuilder(robot).moveJ([0.0, -90.0, 90.0, -90.0, 0.0, 0.0]).build()
    report = prog.validate()
    skips = [i for i in report.issues if i.code == "dynamics_skip"]
    assert skips
    assert skips[0].severity == "warning"
    assert report.torque_peaks is None
    assert "力矩校验跳过" in skips[0].message


def test_check_dynamics_false_is_silent():
    """check_dynamics=False 时不产生任何动力学相关 Issue。"""
    urdf = REPO_ROOT / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"
    robot = Robot(str(urdf), "UR5e")
    prog = ProgramBuilder(robot).moveJ([0.0, -90.0, 90.0, -90.0, 0.0, 0.0]).build()
    report = prog.validate(check_dynamics=False)
    assert not [i for i in report.issues if i.code == "dynamics_skip"]


def test_planner_dynamics_model_payload(tmp_path):
    """通过 planner.dynamics_model 录入负载后校验生效（so101 自带惯性）。"""
    urdf = REPO_ROOT / "assets" / "models" / "lerobot" / "so101" / "urdf" / "so101.urdf"
    if not urdf.exists():
        pytest.skip("so101 URDF 不存在")
    robot = Robot(str(urdf), "SO101")
    prog = ProgramBuilder(robot).moveJ([0.0] * robot.dof).build()
    # 零位→零位是零长运动，换一个真实目标
    prog = ProgramBuilder(robot).moveJ([0.0] * robot.dof).moveJ([10.0] * robot.dof).build()
    from service.planner import TrajectoryPlanner

    planner = TrajectoryPlanner(robot)
    assert planner.dynamics_model is not None
    planner.dynamics_model.set_payload(0.5)
    report = planner.validate(prog)
    assert report.torque_peaks is not None
    assert report.cycle_time is not None


def test_power_zero_at_rest(pendulum):
    """静止（qd=0）时机械功率为零。"""
    from service.simulation import Trajectory, Waypoint
    from domain.pose import Pose as _Pose

    model = DynamicsModel(pendulum)
    scale = math.radians(1.0)
    wps = [
        Waypoint(
            joints=np.array([0.0]),
            pose=_Pose(np.eye(4)),
            timestamp=0.0,
        ),
        Waypoint(
            joints=np.array([10.0]),
            pose=_Pose(np.eye(4)),
            timestamp=1.0,
        ),
    ]
    wps[0].velocity = np.zeros(1)
    wps[0].acceleration = np.zeros(1)
    wps[1].velocity = np.zeros(1)
    wps[1].acceleration = np.zeros(1)
    _, powers = model.joint_torques_along(Trajectory(wps))
    assert float(np.max(powers)) == pytest.approx(0.0, abs=1e-12)


# --------------------------------------------------------------------------- #
# GUI：超限标红
# --------------------------------------------------------------------------- #


def test_gui_torque_overload_marks_point_red(tmp_path, qapp=None):
    """力矩超限的错误级 Issue 把对应路点行标红。"""
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtWidgets

    from app.views.program_panel import ProgramPanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    robot = _overload_robot(tmp_path)
    panel = ProgramPanel()
    panel.set_robot(robot, [f"j{i}" for i in range(1, 4)])
    joint_names = robot.joint_names
    positions = [
        {n: math.radians(v) for n, v in zip(joint_names, joints)}
        for joints in ([0.0, -90.0, 45.0], [30.0, -60.0, 20.0])
    ]
    panel.set_taught_points(positions)
    report = panel.validate_program()
    assert report is not None
    assert [i for i in report.errors if i.code == "torque_limit"]
    red = panel._point_issue_level
    assert any(level == "error" for level in red.values()), red
    item = panel.point_list.item(0)
    assert item is not None
    panel.deleteLater()
