"""离线编程（轨迹规划 + 程序校验）的回归测试。

这些测试守的是 OLP 的立身之本：

* **插补是真的** —— ``moveL`` 的 TCP 必须真的走直线，``moveC`` 必须真的走圆弧。
  若退化成"只对终点求一次逆解"，前两个测试立刻挂掉。
* **时间是可信的** —— 节拍估算要能拿去报价。
* **问题会报出来** —— 不可达、越限、干涉必须变成错误，而不是被悄悄吞掉。
* **接触不等于干涉** —— 工件放在台面上不该报警，否则满屏误报等于没有检查。

运行::

    pytest tests/test_olp.py -q
"""

from __future__ import annotations

import math
import sys
import warnings
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from service.planner import (
    AxisLimits,
    PlanningError,
    TrajectoryPlanner,
    _synchronized_time,
    trapezoid_time,
)
from domain.pose import Pose  # noqa: E402
from service.program import Motion, MotionType, Program, ProgramBuilder  # noqa: E402
from domain.robot import Robot  # noqa: E402
from domain.world.scene import Scene  # noqa: E402

warnings.filterwarnings("ignore")  # RPY 在腕部奇异处会报 gimbal lock，与被测逻辑无关

URDF = REPO_ROOT / "assets/models/kuka/kr16_2/kr16_2.urdf"
#: KR16-2 的一个舒展位形，TCP 约在 (458, 0, 1454)
HOME = [0.0, -20.0, 70.0, 0.0, 40.0, 0.0]


def make_robot() -> Robot:
    """加载 KR16-2 并置到 HOME 位形。"""
    robot = Robot(str(URDF), "kr16_2")
    robot.set_joints(HOME)
    return robot


def tcp_xyz(pose: Pose) -> np.ndarray:
    """取位姿的平移部分。"""
    return np.asarray(pose.to_xyz_quat()[:3], dtype=np.float64)


def orientation_of(pose: Pose) -> np.ndarray:
    """取位姿的旋转矩阵。"""
    return np.asarray(pose.matrix[:3, :3], dtype=np.float64)


def angle_between_rotations(a: np.ndarray, b: np.ndarray) -> float:
    """两个旋转矩阵之间的夹角（弧度）。"""
    c = float(np.clip((np.trace(a @ b.T) - 1.0) / 2.0, -1.0, 1.0))
    return math.acos(c)


# --------------------------------------------------------------------------- #
# 速度规划
# --------------------------------------------------------------------------- #


def test_trapezoid_has_cruise_phase_for_long_moves():
    """长距离能加速到 vmax：t = d/v + v/a。"""
    v, a = 100.0, 500.0
    d = 100.0  # d ≥ v²/a = 20，属梯形
    assert trapezoid_time(d, v, a) == pytest.approx(d / v + v / a)


def test_trapezoid_degrades_to_triangle_for_short_moves():
    """短距离达不到 vmax：t = 2·√(d/a)。"""
    v, a = 100.0, 500.0
    d = 5.0  # d < v²/a = 20，属三角形
    assert trapezoid_time(d, v, a) == pytest.approx(2.0 * math.sqrt(d / a))
    # 且必然慢于"全程按 vmax 跑"
    assert trapezoid_time(d, v, a) > d / v


def test_trapezoid_is_continuous_at_the_boundary():
    """梯形与三角形在 d = v²/a 处必须无缝衔接。"""
    v, a = 100.0, 500.0
    d = v * v / a
    long_side = trapezoid_time(d * (1 + 1e-9), v, a)
    short_side = trapezoid_time(d * (1 - 1e-9), v, a)
    assert long_side == pytest.approx(2.0 * v / a, rel=1e-6)
    assert short_side == pytest.approx(2.0 * v / a, rel=1e-6)


def test_trapezoid_zero_distance_is_zero():
    assert trapezoid_time(0.0, 100.0, 500.0) == 0.0


def test_ptp_time_governed_by_slowest_axis():
    """多轴同步：耗时由"最吃力的那一轴"决定，不是各轴时间之和。"""
    deltas = np.array([90.0, 10.0])
    vmax = np.array([180.0, 180.0])
    amax = np.array([720.0, 720.0])
    total = _synchronized_time(deltas, vmax, amax)
    alone_fast = _synchronized_time(np.array([90.0, 0.0]), vmax, amax)
    assert total == pytest.approx(alone_fast)
    assert total > _synchronized_time(np.array([0.0, 10.0]), vmax, amax)


def test_ptp_time_scales_with_axis_travel():
    """同一轴行程翻倍，时间必须变长（且短行程受加速段主导而非严格翻倍）。"""
    vmax = np.array([180.0])
    amax = np.array([720.0])
    t1 = _synchronized_time(np.array([30.0]), vmax, amax)
    t2 = _synchronized_time(np.array([60.0]), vmax, amax)
    assert t2 > t1


# --------------------------------------------------------------------------- #
# moveJ
# --------------------------------------------------------------------------- #


def test_moveJ_lands_exactly_on_target():
    robot = make_robot()
    target = [30.0, -10.0, 60.0, 0.0, 20.0, 10.0]
    traj = ProgramBuilder(robot).moveJ(target).build().plan()

    assert len(traj.waypoints) > 2, "moveJ 必须插补出中间点，而不是一步跳过去"
    np.testing.assert_allclose(traj.waypoints[-1].joints, target, atol=1e-9)
    times = traj.get_timestamps()
    assert np.all(np.diff(times) > 0), "时间戳必须严格递增"


def test_moveJ_cycle_time_is_physically_plausible():
    """轴 1 转 90°、额定 180°/s：加速段 0.25 s + 匀速段 0.5 s 左右。"""
    robot = make_robot()
    traj = ProgramBuilder(robot).moveJ([90.0, -20.0, 70.0, 0.0, 40.0, 0.0]).build().plan()
    t = traj.duration()
    assert 0.5 < t < 1.2, f"90° 单轴运动耗时 {t:.3f}s 不合理"


def test_moveJ_beyond_limits_is_reported_not_clamped():
    """目标越限必须报错——悄悄夹回来会让现场点位对不上。"""
    robot = make_robot()
    lo, hi = robot.kinematics.limits()
    lo_deg, hi_deg = np.degrees(lo), np.degrees(hi)
    # 把轴 2 推到上限位之外（KR16-2 轴2 上限 +35°）
    beyond = float(hi_deg[1]) + 20.0
    report = ProgramBuilder(robot).moveJ([0.0, beyond, 70.0, 0.0, 40.0, 0.0]).build().validate()
    assert not report.ok
    assert any(i.code == "planning" for i in report.errors)
    assert "限位" in report.errors[0].message


# --------------------------------------------------------------------------- #
# moveL —— 核心：TCP 必须真的走直线
# --------------------------------------------------------------------------- #


def test_moveL_tcp_stays_on_the_straight_segment():
    """插补出的每个 TCP 点到首尾连线的距离必须≈0。

    这是对"逐点逆解"最直接的检验：只对终点求逆解的话，中间点会沿关节空间的
    弧线走，偏离直线可达几十毫米。
    """
    robot = make_robot()
    p0 = robot.forward_kinematics()
    delta = np.array([150.0, 60.0, -100.0])
    p1 = p0.with_position(*(tcp_xyz(p0) + delta))

    traj = ProgramBuilder(robot).moveL(p1).build().plan()
    assert len(traj.waypoints) > 100, "1mm 分辨率下 200mm 的位移应插补出足够多的点"

    a = tcp_xyz(p0)
    direction = delta / np.linalg.norm(delta)
    deviations = []
    for wp in traj.waypoints:
        v = tcp_xyz(wp.pose) - a
        # 点到直线的距离 = |v - (v·d)d|
        deviations.append(np.linalg.norm(v - float(v @ direction) * direction))
    max_dev = max(deviations)
    assert max_dev < 0.05, f"TCP 偏离直线 {max_dev:.4f} mm，插补没有真正生效"


def test_moveL_progresses_monotonically():
    """沿运动方向的投影必须单调不减（不能中途倒退）。"""
    robot = make_robot()
    p0 = robot.forward_kinematics()
    delta = np.array([150.0, 0.0, -100.0])
    p1 = p0.with_position(*(tcp_xyz(p0) + delta))
    traj = ProgramBuilder(robot).moveL(p1).build().plan()

    direction = delta / np.linalg.norm(delta)
    a = tcp_xyz(p0)
    proj = [float((tcp_xyz(wp.pose) - a) @ direction) for wp in traj.waypoints]
    assert proj[0] == pytest.approx(0.0, abs=1e-6)
    assert proj[-1] == pytest.approx(float(np.linalg.norm(delta)), rel=1e-3)
    assert np.all(np.diff(proj) > -1e-6), "直线插补出现倒退"


def test_moveL_orientation_changes_smoothly():
    """姿态用 slerp：相邻点之间的姿态跳变必须很小。"""
    robot = make_robot()
    p0 = robot.forward_kinematics()
    p1 = p0.with_position(*(tcp_xyz(p0) + np.array([120.0, 0.0, -80.0])))
    # 让终点姿态也变化，才能检验 slerp
    p1.matrix[:3, :3] = p0.matrix[:3, :3] @ np.array(
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    )
    traj = ProgramBuilder(robot).moveL(p1).build().plan()

    rots = [orientation_of(wp.pose) for wp in traj.waypoints]
    steps = [angle_between_rotations(rots[i], rots[i + 1]) for i in range(len(rots) - 1)]
    assert max(steps) < math.radians(2.0), f"姿态跳变 {math.degrees(max(steps)):.2f}° 过大"


def test_moveL_cycle_time_reflects_tcp_speed():
    """200mm 直线、500mm/s：约 0.4 s 巡航 + 0.25 s 加速段。"""
    robot = make_robot()
    p0 = robot.forward_kinematics()
    p1 = p0.with_position(*(tcp_xyz(p0) + np.array([200.0, 0.0, 0.0])))
    t = ProgramBuilder(robot).moveL(p1).build().plan().duration()
    assert 0.4 < t < 1.5, f"200mm 直线耗时 {t:.3f}s 不合理"


# --------------------------------------------------------------------------- #
# moveC —— 核心：必须真的走圆弧
# --------------------------------------------------------------------------- #


def _arc_poses(robot: Robot) -> tuple[Pose, Pose, np.ndarray, float]:
    """构造一个半径 80mm、90° 的水平圆弧。

    Returns:
        ``(via, end, center, radius)``；圆心落在 p0 + (-80, 0, 0)
    """
    p0 = robot.forward_kinematics()
    base = tcp_xyz(p0)
    center = base + np.array([-80.0, 0.0, 0.0])
    radius = 80.0

    def point(deg: float) -> np.ndarray:
        a = math.radians(deg)
        return center + np.array([radius * math.cos(a), radius * math.sin(a), 0.0])

    via = p0.with_position(*point(45.0))
    end = p0.with_position(*point(90.0))
    return via, end, center, radius


def test_moveC_points_lie_on_the_circle():
    """插补点到圆心的距离必须处处等于半径。"""
    robot = make_robot()
    via, end, center, radius = _arc_poses(robot)
    traj = ProgramBuilder(robot).moveC(via, end).build().plan()
    assert len(traj.waypoints) > 50

    radii = [np.linalg.norm(tcp_xyz(wp.pose) - center) for wp in traj.waypoints]
    err = max(abs(r - radius) for r in radii)
    assert err < 0.05, f"偏离圆弧 {err:.4f} mm"


def test_moveC_passes_through_the_via_point():
    """轨迹上必须有采样点落在 via 点附近。"""
    robot = make_robot()
    via, end, _, _ = _arc_poses(robot)
    traj = ProgramBuilder(robot).moveC(via, end).build().plan()

    via_xyz = tcp_xyz(via)
    dists = [np.linalg.norm(tcp_xyz(wp.pose) - via_xyz) for wp in traj.waypoints]
    assert min(dists) < 1.0, f"轨迹没经过 via 点，最近距离 {min(dists):.2f} mm"


def test_moveC_sweeps_the_full_angle():
    """圆弧总转角应接近 90°。"""
    robot = make_robot()
    via, end, center, _ = _arc_poses(robot)
    traj = ProgramBuilder(robot).moveC(via, end).build().plan()

    start_vec = tcp_xyz(traj.waypoints[0].pose) - center
    end_vec = tcp_xyz(traj.waypoints[-1].pose) - center
    cos_a = float(start_vec @ end_vec) / (np.linalg.norm(start_vec) * np.linalg.norm(end_vec))
    assert math.degrees(math.acos(np.clip(cos_a, -1, 1))) == pytest.approx(90.0, abs=2.0)


def test_moveC_rejects_collinear_points():
    """三点共线时无法确定圆弧，必须报错而不是退化成直线。"""
    robot = make_robot()
    p0 = robot.forward_kinematics()
    base = tcp_xyz(p0)
    via = p0.with_position(*(base + np.array([50.0, 0.0, 0.0])))
    end = p0.with_position(*(base + np.array([120.0, 0.0, 0.0])))
    report = ProgramBuilder(robot).moveC(via, end).build().validate()
    assert not report.ok
    assert "共线" in report.errors[0].message


# --------------------------------------------------------------------------- #
# 可达性与报错
# --------------------------------------------------------------------------- #


def test_unreachable_target_is_reported():
    """远在工作空间之外的目标必须变成错误，而不是返回一个"差不多"的位形。"""
    robot = make_robot()
    p0 = robot.forward_kinematics()
    far = p0.with_position(20000.0, 20000.0, 20000.0)
    report = ProgramBuilder(robot).moveL(far).build().validate()
    assert not report.ok
    assert any(i.code == "planning" for i in report.errors)


def test_failed_motion_does_not_fake_a_trajectory():
    """规划失败时轨迹必须停在失败处，而不是补齐到终点。"""
    robot = make_robot()
    p0 = robot.forward_kinematics()
    far = p0.with_position(20000.0, 0.0, 0.0)
    traj = ProgramBuilder(robot).moveL(far).build().plan()
    # 只有起点，没有任何一个插补点
    assert len(traj.waypoints) == 1


def test_spline_now_supported():
    """SPLINE 插补已实现：规划可达、节拍为正，不再被拒绝。"""
    robot = make_robot()
    prog = Program(
        robot,
        [Motion(MotionType.JOINTS, [0.0] * 6), Motion(MotionType.SPLINE, [10.0] * 6)],
    )
    report = prog.validate()
    assert report.ok, report.format()
    assert report.cycle_time and report.cycle_time > 0


def test_issue_string_is_actionable():
    """问题文本要能定位到第几条指令、什么时刻。"""
    issue = __import__("service.planner", fromlist=["Issue"]).Issue(
        "error", "collision", 3, "干涉：robot ↔ 工作台", progress=0.5, time=1.25
    )
    text = str(issue)
    assert "指令 4" in text and "50%" in text and "1.25s" in text


# --------------------------------------------------------------------------- #
# 节拍与速度百分比
# --------------------------------------------------------------------------- #


def test_half_speed_takes_roughly_twice_as_long():
    robot = make_robot()
    target = [90.0, -20.0, 70.0, 0.0, 40.0, 0.0]
    t_full = ProgramBuilder(robot).moveJ(target, speed=100.0).build().plan().duration()
    t_half = ProgramBuilder(robot).moveJ(target, speed=50.0).build().plan().duration()
    assert t_half > t_full * 1.5, f"半速 {t_half:.3f}s 未显著长于全速 {t_full:.3f}s"


def test_report_carries_cycle_time_and_path_length():
    robot = make_robot()
    p0 = robot.forward_kinematics()
    p1 = p0.with_position(*(tcp_xyz(p0) + np.array([200.0, 0.0, 0.0])))
    report = ProgramBuilder(robot).moveL(p1).build().validate()
    assert report.ok, report.format()
    assert report.cycle_time > 0
    assert report.path_length == pytest.approx(200.0, abs=1.0)
    assert "节拍" in report.format()


def test_moveJ_contributes_no_tcp_path_length():
    """moveJ 的 TCP 不走直线，路径长度不应计入。"""
    robot = make_robot()
    report = ProgramBuilder(robot).moveJ([90.0, -20.0, 70.0, 0.0, 40.0, 0.0]).build().validate()
    assert report.path_length == pytest.approx(0.0)


# --------------------------------------------------------------------------- #
# 干涉检查
# --------------------------------------------------------------------------- #


def build_scene(robot_urdf: Path, table_x: float = 1200.0) -> Scene:
    """机器人 + 工作台 + 台面工件。"""
    scene = Scene()
    scene.set_robot(str(robot_urdf))
    scene.add(
        "worktable",
        name="工作台",
        params={"length": 1400.0, "width": 900.0, "height": 780.0},
        pose=Pose.from_xyz_rpy(table_x, 0.0, 0.0, 0.0, 0.0, 0.0),
    )
    scene.add(
        "workpiece_box",
        name="工件",
        params={"length": 300.0, "width": 200.0, "height": 150.0},
        pose=Pose.from_xyz_rpy(table_x, 0.0, 780.0, 0.0, 0.0, 0.0),
    )
    return scene


def test_contact_is_not_reported_as_collision():
    """工件静置在台面上属于零间隙接触，不该报警（否则满屏误报）。"""
    scene = build_scene(URDF, table_x=2500.0)  # 放远，排除机器人干扰
    hits = scene.check_collisions(include_robot=False, margin=-0.5)
    assert hits == [], f"把接触误判为干涉：{[str(h) for h in hits]}"


def test_real_penetration_is_reported():
    """真的嵌进去了就必须报。"""
    scene = build_scene(URDF, table_x=2500.0)
    scene.objects["工件"].pose = Pose.from_xyz_rpy(2500.0, 0.0, 770.0, 0.0, 0.0, 0.0)
    hits = scene.check_collisions(include_robot=False, margin=-0.5)
    assert hits, "嵌入 10mm 却没检出干涉"


def test_collision_along_path_is_detected():
    """机器人扫过工作台——沿路径检查必须抓到。"""
    robot = make_robot()
    scene = build_scene(URDF, table_x=1200.0)
    scene.set_robot_joints(HOME)

    # 从 HOME 直线下探到台面高度以下，必然扫过工作台
    p0 = robot.forward_kinematics()
    target = p0.with_position(900.0, 0.0, 500.0)
    report = ProgramBuilder(robot).moveL(target).build().validate(scene=scene)
    assert not report.ok, "机器人扫过工作台却报告无干涉"
    assert any(i.code == "collision" for i in report.errors)


def test_collision_issue_is_located_in_time():
    """干涉问题要带时刻，才能定位到具体指令。"""
    robot = make_robot()
    scene = build_scene(URDF, table_x=1200.0)
    scene.set_robot_joints(HOME)
    p0 = robot.forward_kinematics()
    report = (
        ProgramBuilder(robot)
        .moveL(p0.with_position(900.0, 0.0, 500.0))
        .build()
        .validate(scene=scene)
    )
    coll = [i for i in report.issues if i.code == "collision"]
    assert coll, report.format()
    assert coll[0].time is not None and coll[0].time >= 0.0


def test_clean_program_passes_with_scene():
    """一个远离工作台的小幅运动应当干净通过。"""
    robot = make_robot()
    scene = build_scene(URDF, table_x=2500.0)
    scene.set_robot_joints(HOME)
    p0 = robot.forward_kinematics()
    # 在机器人自己身前小幅平移，不触碰任何场景对象
    target = p0.with_position(*(tcp_xyz(p0) + np.array([0.0, 0.0, -60.0])))
    report = ProgramBuilder(robot).moveL(target).build().validate(scene=scene)
    assert report.ok, report.format()


# --------------------------------------------------------------------------- #
# 端到端：校验 → 生成代码
# --------------------------------------------------------------------------- #


def test_validated_program_generates_kuka_code():
    """校验通过后应能生成可下载的 KRL。"""
    robot = make_robot()
    p0 = robot.forward_kinematics()
    prog = (
        ProgramBuilder(robot)
        .moveJ([30.0, -25.0, 75.0, 0.0, 45.0, 0.0])
        .moveL(p0.with_position(*(tcp_xyz(p0) + np.array([120.0, 0.0, -60.0]))))
        .build()
    )
    report = prog.validate()
    assert report.ok, report.format()

    code = prog.to_post()
    assert "DEF " in code and "END" in code
    assert "LIN" in code or "PTP" in code


def test_simulation_without_scene_raises_instead_of_lying():
    """要求干涉检查却不给场景，必须报错——不能假装检查过了。"""
    robot = make_robot()
    prog = ProgramBuilder(robot).moveJ([30.0, -20.0, 70.0, 0.0, 40.0, 0.0]).build()
    with pytest.raises(ValueError, match="scene"):
        prog.simulate(collision_check=True)


# --------------------------------------------------------------------------- #
# 全型号回归
# --------------------------------------------------------------------------- #


def ready_pose(robot: Robot) -> np.ndarray:
    """按各轴行程的 25% 取一个"预备位形"。

    直接用零位是不行的：绝大多数工业机器人在零位是"大臂笔直朝上"的奇异
    位形，雅可比退化，直线插补必然失败。取行程的 25% 能让所有 29 个型号
    都落在舒展、非奇异的位形上。
    """
    lo, hi = robot.kinematics.limits()
    lo_deg, hi_deg = np.degrees(lo), np.degrees(hi)
    finite = np.isfinite(lo_deg) & np.isfinite(hi_deg)
    q = np.where(finite, lo_deg + 0.25 * (hi_deg - lo_deg), 0.0)
    if q.size:
        q[0] = 0.0  # 腰转回正，避免起始就贴着限位
    return q


# --------------------------------------------------------------------------- #
# 状态一致性与机器人 API
# --------------------------------------------------------------------------- #


def test_accessing_kinematics_preserves_taught_joints():
    """访问 ``robot.kinematics`` 不得把已示教的关节值清零。

    历史上惰性构建运动学链时会无条件 ``_joint_positions = zeros()``，
    于是"设好位形 → 再规划"这种最普通的用法会静默失效。
    """
    robot = make_robot()
    _ = robot.kinematics  # 触发惰性构建
    np.testing.assert_allclose(robot.get_joints(), HOME)
    # 再走一遍完整 API 也不能丢
    _ = TrajectoryPlanner(robot).limits
    np.testing.assert_allclose(robot.get_joints(), HOME)


def test_ik_inverts_fk_with_a_tool_attached():
    """装了工具（焊枪/夹爪）之后，FK 与 IK 必须仍然互逆。

    历史实现里 IK 把目标当法兰盘位姿用，忽略了 ``tool`` 和 ``base``——
    一旦装上焊枪，机器人就会被导到完全不同的位置。对离线编程是致命的。
    """
    robot = make_robot()
    # 一根 200mm 长、沿 +X 偏出的焊枪
    gun = Pose.from_xyz_rpy(50.0, 0.0, 200.0, 0.0, 0.0, 0.0)
    robot.tool = gun

    q_known = np.array([15.0, -25.0, 65.0, 10.0, 35.0, -20.0])
    tcp = robot.forward_kinematics(q_known)
    q_solved = robot.inverse_kinematics(tcp, guess=HOME)

    # 关节解可以落在另一分支，但还原出的 TCP 必须一致
    tcp_back = robot.forward_kinematics(q_solved)
    np.testing.assert_allclose(tcp_back.to_xyz_quat()[:3], tcp.to_xyz_quat()[:3], atol=0.05)
    robot.tool = Pose()  # 复原，避免影响同进程内其他测试


#: 工业品牌目录。笛卡尔规划要求这些目录下的型号**必须**通过
INDUSTRIAL_BRANDS = ("kuka", "abb", "fanuc", "jaka", "ur", "estun")


def test_all_robot_models_plan_joint_motions():
    """全型号冒烟：加载 → 关节空间插补 → 节拍。

    用 moveJ 而不是 moveL，是因为关节空间插补不依赖工作空间几何——
    "模型能不能用"这件事不该被"自动挑的起始姿态好不好"绑架。
    """
    urdfs = sorted((REPO_ROOT / "assets" / "models").rglob("*.urdf"))
    assert len(urdfs) >= 20, f"模型数量异常：{len(urdfs)}"

    failures = []
    for path in urdfs:
        tag = f"{path.parent.parent.name}/{path.parent.name}"
        try:
            robot = Robot(str(path), path.stem)
            if robot.kinematics is None:
                failures.append(f"{tag}: 运动学链构建失败")
                continue
            q = ready_pose(robot)
            robot.set_joints(q)

            q2 = np.array(q, dtype=np.float64)
            q2[0] += 10.0
            traj = ProgramBuilder(robot).moveJ(q2).build().plan()
            if len(traj.waypoints) < 2:
                failures.append(f"{tag}: moveJ 轨迹为空")
            elif not math.isfinite(traj.duration()) or traj.duration() <= 0:
                failures.append(f"{tag}: moveJ 节拍异常")
            else:
                np.testing.assert_allclose(traj.waypoints[-1].joints, q2, atol=1e-9)
        except Exception as exc:
            failures.append(f"{tag}: {type(exc).__name__}: {exc}")

    assert not failures, "以下型号规划失败：\n  " + "\n  ".join(failures)


def test_industrial_brands_plan_cartesian_motions():
    """六大工业品牌的全部型号都必须能做笛卡尔直线插补。

    这是"能不能拿去给客户做方案"的门槛。若某天新增品牌的型号在这里挂掉，
    说明它的 URDF 或限位有问题，得查模型而不是放宽这个断言。
    """
    failures = []
    checked = 0
    for brand in INDUSTRIAL_BRANDS:
        for path in sorted((REPO_ROOT / "assets" / "models" / brand).rglob("*.urdf")):
            tag = f"{brand}/{path.parent.name}"
            checked += 1
            try:
                robot = Robot(str(path), path.stem)
                robot.set_joints(ready_pose(robot))
                p0 = robot.forward_kinematics()
                # 沿工具接近方向后退 10mm（退刀），标准且几乎总是可达
                approach = np.asarray(p0.matrix[:3, 2], dtype=np.float64)
                target = p0.with_position(*(tcp_xyz(p0) - 10.0 * approach))
                traj = ProgramBuilder(robot).moveL(target).build().plan()
                if len(traj.waypoints) < 2:
                    failures.append(f"{tag}: 退刀轨迹为空")
                elif traj.duration() <= 0:
                    failures.append(f"{tag}: 退刀节拍异常")
                else:
                    got = tcp_xyz(traj.waypoints[-1].pose)
                    np.testing.assert_allclose(got, tcp_xyz(target), atol=0.1)
            except Exception as exc:
                failures.append(f"{tag}: {type(exc).__name__}: {exc}")

    assert checked >= 15, f"覆盖的工业型号过少：{checked}"
    assert not failures, "以下工业型号笛卡尔规划失败：\n  " + "\n  ".join(failures)


def test_unreachable_cartesian_motion_fails_cleanly():
    """够不到的目标必须变成可报告的问题，而不是崩掉或假装成功。

    LeRobot SO-101 是 hobby 级桌面臂，臂展有限：从自动生成的折叠姿态出发，
    沿工具轴退刀 150mm 明确落在工作空间之外，用来检验失败路径是否体面。

    同时用 10mm 退刀做**正向对照**：同一个姿态下小幅退刀必须可达。这条
    对照是 rpy 组合顺序回归的守门员——URDF 的 rpy 是固定轴（外旋），
    若误按内旋 ``Rx·Ry·Rz`` 组合，正解给出的末端位姿自相矛盾，反而会把
    近在咫尺的目标判成不可达。
    """
    path = REPO_ROOT / "assets/models/lerobot/so101/urdf/so101.urdf"
    if not path.exists():
        pytest.skip("SO-101 模型不存在")

    robot = Robot(str(path), "so101")
    robot.set_joints(ready_pose(robot))
    p0 = robot.forward_kinematics()
    approach = np.asarray(p0.matrix[:3, 2], dtype=np.float64)

    # 正向对照：小幅退刀必须可达，证明运动学本身是自洽的
    near = p0.with_position(*(tcp_xyz(p0) - 10.0 * approach))
    assert (
        ProgramBuilder(robot).moveL(near).build().validate().ok
    ), "10mm 退刀本应可达——检查 rpy 是否又退化成内旋组合"

    target = p0.with_position(*(tcp_xyz(p0) - 150.0 * approach))
    report = ProgramBuilder(robot).moveL(target).build().validate()
    assert not report.ok, "明明够不到却报告成功"
    # 必须能定位到具体指令、给出可读原因，且轨迹不是"补齐"出来的
    assert report.errors, report.format()
    assert "指令 1" in str(report.errors[0])
    assert report.num_waypoints == 1


def test_axis_limits_read_from_urdf():
    """轴参数要真的来自 URDF，而不是拍脑袋的默认值。"""
    robot = make_robot()
    limits = AxisLimits.from_kinematics(robot.kinematics)
    assert limits.max_speed.shape == (robot.dof,)
    assert np.all(limits.max_speed > 0)
    assert np.all(limits.max_accel > 0)
    # KR16-2 轴 2 的下限位是 -155°，必须读得出来。
    # URDF 里存的是弧度，来回换算会有 ~1e-4 度的误差，容差按此取
    assert limits.lower[1] == pytest.approx(-155.0, abs=1e-3)
    assert limits.upper[1] == pytest.approx(35.0, abs=1e-3)


def test_urdf_rpy_is_fixed_axis_not_intrinsic():
    """URDF 的 ``rpy`` 是固定轴（外旋），组合结果必须是 ``Rz·Ry·Rx``。

    scipy 的大小写语义极易搞反：``"XYZ"`` 是内旋（得到 ``Rx·Ry·Rz``），
    ``"xyz"`` 才是外旋。两者只在单轴旋转时等价，而程序化生成的工业机型
    恰好全是单轴 rpy，所以这个错误能悄悄潜伏——直到遇到 CAD 导出的、
    带完整三轴 rpy 的真实模型（如 SO-101）才暴露。
    """
    from domain.kinematics import _rotmat_rpy

    rpy = np.array([0.3, -0.75, 1.1])  # 三轴都非零，内外旋必然分道扬镳

    cx, sx = math.cos(rpy[0]), math.sin(rpy[0])
    cy, sy = math.cos(rpy[1]), math.sin(rpy[1])
    cz, sz = math.cos(rpy[2]), math.sin(rpy[2])
    rx_m = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry_m = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz_m = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])

    # URDF 规范：固定轴 X→Y→Z 依次绕世界轴，等价于 Rz(yaw)·Ry(pitch)·Rx(roll)
    np.testing.assert_allclose(_rotmat_rpy(rpy), rz_m @ ry_m @ rx_m, atol=1e-12)
    # 与内旋（绕运动轴）必须有明显区别，否则说明又写成了大写 "XYZ"
    assert not np.allclose(_rotmat_rpy(rpy), rx_m @ ry_m @ rz_m, atol=1e-6)
