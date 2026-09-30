#!/usr/bin/env python3
"""v0.3.4 标定与手眼测试（``service.calibration``）。

覆盖六块：

1. **刚体配准与坐标系标定**：Kabsch/SVD 解的唯一性与退化拦截、3 点法正交化；
2. **TCP 标定**：4 点法（位置）、6 点法（位置 + 姿态）、残差与质量判据、
   位形退化/数据不足的显式报错；
3. **法兰位姿提取**：``FK = base · flange · tool`` 的反推；
4. **手眼标定**：eye-in-hand / eye-to-hand 两种模式在合成数据上精确复原、
   噪声下的精度量级、模式传错与"绕同一根轴转"必须被拦下；
5. **相机模型与视觉抓取**：投影/反投影往返、射线求交、像素 → 抓取位姿闭环；
6. **标定集**：JSON 往返、写回机器人、写回 URDF（新末端 link 的 FK 一致性、
   幂等）、出码头部携带标定摘要，以及示例脚本的子进程验收。

所有"真值"都由已知位姿反算生成，因此断言可以做到数值级（1e-9 量级），
同时包含"必须报错"的负例——标定出错时沉默地给出一个数比报错危险得多。
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from service.calibration import (
    CalibrationError,
    CalibrationSet,
    CameraIntrinsics,
    TcpCalibration,
    TouchProbe,
    calibrate_base_3point,
    calibrate_frame,
    calibrate_tcp_4point,
    calibrate_tcp_6point,
    calibrate_tcp_from_joints,
    flange_poses_from_joints,
    generate_hand_eye_dataset,
    grasp_pose_from_pixel,
    intersect_plane,
    pixel_of_point,
    pixel_to_ray_in_base,
    rigid_transform,
    solve_hand_eye,
)
from domain.kinematics import RobotKinematics  # noqa: E402
from domain.pose import Pose  # noqa: E402
from service.program import ProgramBuilder  # noqa: E402
from domain.robot import Robot  # noqa: E402

UR5E_URDF = REPO_ROOT / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"
KR16_URDF = REPO_ROOT / "assets" / "models" / "kuka" / "kr16_2" / "kr16_2.urdf"

#: 4 个腕部姿态差异明显的真实可达位形（4 点法要求姿态多样）
TOUCH_JOINTS = [
    [10.0, -80.0, 80.0, -90.0, 90.0, 0.0],
    [25.0, -70.0, 100.0, -55.0, 65.0, 35.0],
    [-20.0, -85.0, 70.0, -125.0, 50.0, -30.0],
    [0.0, -75.0, 95.0, -105.0, 80.0, 70.0],
]

TCP_TRUTH = np.array([2.6, -1.4, 78.9])
TIP_REF = np.array([420.0, -260.0, 320.0])


def _mat(R: np.ndarray, t) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = R
    m[:3, 3] = np.asarray(t, dtype=np.float64)
    return m


def _random_rotation(rng: np.random.Generator, lo=10.0, hi=45.0) -> np.ndarray:
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    return Rotation.from_rotvec(np.radians(rng.uniform(lo, hi)) * axis).as_matrix()


def _touch_poses(rotations, tip_offset=TCP_TRUTH, ref=TIP_REF):
    """构造"尖端都落在 ref"的法兰位姿序列。"""
    return [Pose(_mat(R, ref - R @ np.asarray(tip_offset))) for R in rotations]


@pytest.fixture
def ur5e() -> Robot:
    if not UR5E_URDF.exists():
        pytest.skip(f"缺少测试用 URDF: {UR5E_URDF}")
    return Robot(str(UR5E_URDF), "UR5e")


@pytest.fixture
def ur5e_rotations() -> list[np.ndarray]:
    if not UR5E_URDF.exists():
        pytest.skip(f"缺少测试用 URDF: {UR5E_URDF}")
    robot = Robot(str(UR5E_URDF), "UR5e")
    return [p.matrix[:3, :3] for p in flange_poses_from_joints(robot, TOUCH_JOINTS)]


# --------------------------------------------------------------------------- #
# 1. 刚体配准与坐标系标定
# --------------------------------------------------------------------------- #
def test_rigid_transform_recovers_known_pose():
    rng = np.random.default_rng(0)
    T = _mat(_random_rotation(rng), [120.0, -60.0, 45.0])
    src = rng.uniform(-200, 200, (5, 3))
    dst = src @ T[:3, :3].T + T[:3, 3]
    R, t, res = rigid_transform(src, dst)
    assert np.allclose(R, T[:3, :3], atol=1e-9)
    assert np.allclose(t, T[:3, 3], atol=1e-9)
    assert res.max() < 1e-9


def test_rigid_transform_rejects_degenerate_input():
    with pytest.raises(CalibrationError, match="至少需要 3 个点"):
        rigid_transform([[0, 0, 0], [1, 0, 0]], [[0, 0, 0], [1, 0, 0]])
    with pytest.raises(CalibrationError, match="点数不匹配"):
        rigid_transform([[0, 0, 0], [1, 0, 0], [0, 1, 0]], [[0, 0, 0], [1, 0, 0]])
    # 共线：绕该直线的旋转不确定，不能给出唯一解
    collinear = [[0, 0, 0], [10, 0, 0], [20, 0, 0]]
    with pytest.raises(CalibrationError, match="共线"):
        rigid_transform(collinear, [[0, 0, 0], [10, 0, 0], [20, 5, 0]])


def test_calibrate_frame_maps_source_into_target():
    rng = np.random.default_rng(3)
    T = _mat(_random_rotation(rng), [-300.0, 150.0, 60.0])
    src = rng.uniform(-100, 100, (4, 3))
    dst = src @ T[:3, :3].T + T[:3, 3]
    pose = calibrate_frame(src, dst)
    for p_src, p_dst in zip(src, dst):
        assert np.allclose(pose.matrix[:3, :3] @ p_src + pose.matrix[:3, 3], p_dst, atol=1e-9)


def test_calibrate_base_3point_builds_orthonormal_frame():
    origin, xp, yp = [10.0, 20.0, 30.0], [110.0, 20.0, 30.0], [10.0, 120.0, 30.0]
    pose = calibrate_base_3point(origin, xp, yp)
    R = pose.matrix[:3, :3]
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-12)  # 正交
    assert np.isclose(np.linalg.det(R), 1.0)  # 右手系
    assert np.allclose(pose.matrix[:3, 3], origin)
    assert np.allclose(R[:, 0], [1.0, 0.0, 0.0], atol=1e-12)

    # 现场徒手点不出直角：Y 向点歪一点也要归一成正交系
    skewed = calibrate_base_3point(origin, xp, [10.0, 120.0, 34.0])
    assert np.allclose(skewed.matrix[:3, :3].T @ skewed.matrix[:3, :3], np.eye(3), atol=1e-12)


def test_calibrate_base_3point_rejects_degenerate():
    with pytest.raises(CalibrationError, match="重合"):
        calibrate_base_3point([0, 0, 0], [0, 0, 0], [0, 10, 0])
    with pytest.raises(CalibrationError, match="共线"):
        calibrate_base_3point([0, 0, 0], [10, 0, 0], [20, 0, 0])


# --------------------------------------------------------------------------- #
# 2. TCP 标定
# --------------------------------------------------------------------------- #
def test_tcp_4point_recovers_offset_exactly(ur5e_rotations):
    cal = calibrate_tcp_4point(_touch_poses(ur5e_rotations), ref_point=TIP_REF)
    assert np.allclose(cal.offset.matrix[:3, 3], TCP_TRUTH, atol=1e-9)
    assert np.allclose(cal.offset.matrix[:3, :3], np.eye(3))  # 4 点法只定位置
    assert cal.residual_max_mm < 1e-9 and cal.residual_rms_mm < 1e-9
    assert cal.ok and cal.n_points == 4 and cal.method == "4point"
    assert "TCP(4point)" in cal.summary()


def test_tcp_4point_reports_noise_as_residual(ur5e_rotations):
    rng = np.random.default_rng(11)
    noisy = []
    for p in _touch_poses(ur5e_rotations):
        m = p.matrix.copy()
        m[:3, 3] += rng.normal(0.0, 0.4, 3)  # 操作工每次碰到同一位置有 ~0.4mm 的抖动
        noisy.append(Pose(m))
    cal = calibrate_tcp_4point(noisy, ref_point=TIP_REF)
    assert cal.residual_max_mm > 0.05
    assert cal.residual_rms_mm <= cal.residual_max_mm
    assert cal.ok == (cal.residual_max_mm <= 0.5)
    # 噪声不该把解带偏太远
    assert np.linalg.norm(cal.offset.matrix[:3, 3] - TCP_TRUTH) < 1.5


def test_tcp_4point_requires_enough_poses_and_orientation_diversity(ur5e_rotations):
    with pytest.raises(CalibrationError, match="至少需要 4 个法兰位姿"):
        calibrate_tcp_4point(_touch_poses(ur5e_rotations[:3]))
    # 姿态全相同 → 方程秩亏（真机上就是"没换姿态碰了四次"）
    same = _touch_poses([ur5e_rotations[0]] * 5)
    with pytest.raises(CalibrationError, match="姿态差异不足"):
        calibrate_tcp_4point(same)


def test_tcp_6point_recovers_orientation(ur5e_rotations):
    poses = _touch_poses(ur5e_rotations)
    tool_R = Rotation.from_euler("xyz", np.radians([0.0, 0.0, 2.5])).as_matrix()
    R_f0 = poses[0].matrix[:3, :3]
    x_probe = TouchProbe(poses[0], TIP_REF + R_f0 @ tool_R @ np.array([80.0, 0.0, 0.0]))
    z_probe = TouchProbe(poses[0], TIP_REF + R_f0 @ tool_R @ np.array([0.0, 0.0, 80.0]))

    cal = calibrate_tcp_6point(poses, x_probe, z_probe, ref_point=TIP_REF)
    assert cal.method == "6point"
    # 姿态与位置都要解出来
    err = Rotation.from_matrix(cal.offset.matrix[:3, :3] @ tool_R.T).as_rotvec()
    assert np.degrees(np.linalg.norm(err)) < 1e-6
    assert np.allclose(cal.offset.matrix[:3, 3], TCP_TRUTH, atol=1e-9)
    assert "6point" in cal.summary()


def test_tcp_6point_rejects_bad_probes(ur5e_rotations):
    poses = _touch_poses(ur5e_rotations)
    R_f0 = poses[0].matrix[:3, :3]
    ok_x = TouchProbe(poses[0], TIP_REF + R_f0 @ np.array([80.0, 0.0, 0.0]))
    with pytest.raises(CalibrationError, match="与基准点重合"):
        calibrate_tcp_6point(poses, TouchProbe(poses[0], TIP_REF), ok_x, ref_point=TIP_REF)
    # 两次方向触碰几乎同向 → 定不出姿态
    near_same = TouchProbe(poses[0], TIP_REF + R_f0 @ np.array([80.0, 5.0, 0.0]))
    with pytest.raises(CalibrationError, match="几乎同向"):
        calibrate_tcp_6point(poses, ok_x, near_same, ref_point=TIP_REF)


def test_tcp_calibration_dict_roundtrip(ur5e_rotations):
    cal = calibrate_tcp_4point(_touch_poses(ur5e_rotations), ref_point=TIP_REF)
    back = TcpCalibration.from_dict(cal.to_dict())
    assert np.allclose(back.offset.matrix, cal.offset.matrix)
    assert np.allclose(back.reference_point, cal.reference_point)
    assert back.method == cal.method and back.n_points == cal.n_points


def test_flange_poses_from_joints_strips_tool(ur5e):
    tool = Pose.from_xyz_rpy(3.0, -2.0, 90.0, 0.0, 0.0, 5.0)
    ur5e.tool = tool
    joints = [0.0, -90.0, 90.0, -90.0, 90.0, 0.0]
    flange = flange_poses_from_joints(ur5e, [joints])[0]
    # FK = base · flange · tool  ⇒  flange · tool == FK
    assert np.allclose((flange * tool).matrix, ur5e.forward_kinematics(joints).matrix, atol=1e-9)


def test_calibrate_tcp_from_joints_roundtrip(ur5e, ur5e_rotations):
    """真机流程：示教器上带**未标定**的工具去碰点，由关节角反推法兰位姿再解 TCP。

    先把构造好的法兰位姿逆解成关节角（此时法兰即末端），再把工具换回名义值调用
    API——等价于现场"拿着名义 TCP 的机器人去碰点"，解出的偏移应当回到真值。
    """
    touch = _touch_poses(ur5e_rotations)
    ur5e.tool = Pose()  # 法兰即末端，IK 目标就是法兰位姿
    joints = [
        ur5e.inverse_kinematics(pose, pos_tol_mm=1e-6, rot_tol_deg=1e-6, max_iter=500).tolist()
        for pose in touch
    ]
    ur5e.tool = Pose.from_xyz_rpy(0.0, 0.0, 10.0, 0.0, 0.0, 0.0)  # 名义工具（未标定）

    cal = calibrate_tcp_from_joints(ur5e, joints, ref_point=TIP_REF)
    assert cal.method == "4point"
    assert np.allclose(cal.offset.matrix[:3, 3], TCP_TRUTH, atol=1e-3)
    assert cal.residual_max_mm < 1e-2


# --------------------------------------------------------------------------- #
# 3. 手眼标定
# --------------------------------------------------------------------------- #
MARKER_IN_FLANGE = Pose.from_xyz_rpy(30.0, 0.0, 90.0, 180.0, 0.0, 0.0)
CAMERA_TRUTH = Pose.from_xyz_rpy(0.0, -400.0, 1100.0, 180.0, 0.0, 0.0)

WRIST_OFFSETS = (
    (0, 0, 0),
    (28, 18, -22),
    (-32, 22, 38),
    (18, -26, 52),
    (-22, -16, -42),
    (34, 12, 16),
)


def _hand_eye_poses(ur5e) -> list[Pose]:
    """真实 UR5e 位形 + 腕部偏转：绕不同方向转动是手眼可观测的前提。"""
    home_points = [
        [0.0, -90.0, 90.0, -90.0, 90.0, 0.0],
        [20.0, -75.0, 95.0, -85.0, 85.0, 20.0],
        [-15.0, -95.0, 85.0, -100.0, 95.0, -15.0],
        [10.0, -80.0, 100.0, -70.0, 75.0, 40.0],
        [-25.0, -85.0, 75.0, -110.0, 100.0, -35.0],
        [5.0, -70.0, 105.0, -60.0, 60.0, 25.0],
    ]
    joints = [
        [p[0], p[1], p[2], p[3] + d4, p[4] + d5, p[5] + d6]
        for p, (d4, d5, d6) in zip(home_points, WRIST_OFFSETS)
    ]
    return flange_poses_from_joints(ur5e, joints)


@pytest.mark.parametrize("mode", ["eye_in_hand", "eye_to_hand"])
def test_hand_eye_recovers_truth_exactly(ur5e, mode):
    poses = _hand_eye_poses(ur5e)
    dataset = generate_hand_eye_dataset(MARKER_IN_FLANGE, CAMERA_TRUTH, mode, flange_poses=poses)
    result = solve_hand_eye(dataset.flange_poses, dataset.camera_readings, mode)
    assert result.mode == mode
    assert result.residual_mm < 1e-6 and result.residual_deg < 1e-6
    assert np.allclose(result.transform.matrix, MARKER_IN_FLANGE.matrix, atol=1e-9)
    if mode == "eye_in_hand":
        # 相机随法兰运动：首帧位形下的相机位姿 = 法兰位姿 · X
        expect = poses[0].matrix @ MARKER_IN_FLANGE.matrix
    else:
        # 相机固定在工位：外参就是生成数据用的真值
        expect = CAMERA_TRUTH.matrix
    assert np.allclose(result.camera_pose_at(poses[0]).matrix, expect, atol=1e-9)
    assert "mm" in result.summary() and "残差" in result.summary()


def test_hand_eye_eye_to_hand_reports_constant_extrinsics(ur5e):
    """eye-to-hand 相机固定在工位：外参应当就是生成数据用的真值，且与位形无关。"""
    dataset = generate_hand_eye_dataset(
        MARKER_IN_FLANGE, CAMERA_TRUTH, "eye_to_hand", flange_poses=_hand_eye_poses(ur5e)
    )
    result = solve_hand_eye(dataset.flange_poses, dataset.camera_readings, "eye_to_hand")
    assert np.allclose(result.camera_in_base.matrix, CAMERA_TRUTH.matrix, atol=1e-9)


def test_hand_eye_camera_pose_at(ur5e):
    poses = _hand_eye_poses(ur5e)
    ds_in = generate_hand_eye_dataset(
        MARKER_IN_FLANGE, CAMERA_TRUTH, "eye_in_hand", flange_poses=poses
    )
    res_in = solve_hand_eye(ds_in.flange_poses, ds_in.camera_readings, "eye_in_hand")
    # eye-in-hand：相机随法兰运动，位形一变相机位姿就变（取它必须用当前法兰位姿）
    p0 = res_in.camera_pose_at(poses[0])
    p1 = res_in.camera_pose_at(poses[1])
    assert np.linalg.norm(p0.matrix[:3, 3] - p1.matrix[:3, 3]) > 10.0
    assert np.allclose(p0.matrix, poses[0].matrix @ MARKER_IN_FLANGE.matrix, atol=1e-9)

    ds_to = generate_hand_eye_dataset(
        MARKER_IN_FLANGE, CAMERA_TRUTH, "eye_to_hand", flange_poses=poses
    )
    res_to = solve_hand_eye(ds_to.flange_poses, ds_to.camera_readings, "eye_to_hand")
    # eye-to-hand：与位形无关，永远是同一处
    assert np.allclose(
        res_to.camera_pose_at(poses[0]).matrix, res_to.camera_pose_at(poses[3]).matrix
    )


def test_hand_eye_tolerates_measurement_noise(ur5e):
    poses = _hand_eye_poses(ur5e)
    noisy = generate_hand_eye_dataset(
        MARKER_IN_FLANGE,
        CAMERA_TRUTH,
        "eye_to_hand",
        flange_poses=poses,
        noise_mm=0.3,
        noise_deg=0.05,
        seed=9,
    )
    result = solve_hand_eye(noisy.flange_poses, noisy.camera_readings, "eye_to_hand")
    assert result.residual_mm < 5.0
    # 0.3mm / 0.05° 量级的视觉噪声下，外参位置误差应在毫米级
    assert np.linalg.norm(result.camera_in_base.matrix[:3, 3] - CAMERA_TRUTH.matrix[:3, 3]) < 3.0


def test_hand_eye_rejects_wrong_mode_and_bad_input(ur5e):
    poses = _hand_eye_poses(ur5e)
    ds = generate_hand_eye_dataset(
        MARKER_IN_FLANGE, CAMERA_TRUTH, "eye_to_hand", flange_poses=poses
    )
    # B 的两种模式互为逆：传错不会报数值错，只会得到永远对不上的外参 → 必须拦下
    with pytest.raises(CalibrationError, match="mode"):
        solve_hand_eye(ds.flange_poses, ds.camera_readings, "eye_in_hand")
    with pytest.raises(CalibrationError, match="只支持"):
        solve_hand_eye(ds.flange_poses, ds.camera_readings, "eye_sideways")
    with pytest.raises(CalibrationError, match="至少需要 3 组数据"):
        solve_hand_eye(ds.flange_poses[:2], ds.camera_readings[:2])
    with pytest.raises(CalibrationError, match="不一致"):
        solve_hand_eye(ds.flange_poses, ds.camera_readings[:-1])


def test_hand_eye_rejects_parallel_rotation_axes(ur5e):
    """只绕一根轴转：另外两个姿态自由度没被约束，最小二乘会给出荒谬的解。"""
    nominal = Pose.from_xyz_rpy(400.0, 0.0, 400.0, 180.0, 0.0, 0.0).matrix
    poses = []
    for ang in (0.0, 20.0, 40.0, 60.0):
        R = Rotation.from_rotvec(np.radians(ang) * np.array([0.0, 0.0, 1.0])).as_matrix()
        poses.append(Pose(_mat(nominal[:3, :3] @ R, nominal[:3, 3])))
    ds = generate_hand_eye_dataset(
        MARKER_IN_FLANGE, CAMERA_TRUTH, "eye_to_hand", flange_poses=poses
    )
    with pytest.raises(CalibrationError, match="旋转轴"):
        solve_hand_eye(ds.flange_poses, ds.camera_readings, "eye_to_hand")


def test_hand_eye_rejects_insufficient_motion(ur5e):
    """相邻位姿几乎不动 → 解不出旋转轴，应报"有效运动不足"而非硬解。"""
    poses = [Pose(CAMERA_TRUTH.matrix) for _ in range(4)]
    datasets = generate_hand_eye_dataset(
        MARKER_IN_FLANGE, CAMERA_TRUTH, "eye_to_hand", flange_poses=poses
    )
    with pytest.raises(CalibrationError, match="有效运动不足"):
        solve_hand_eye(datasets.flange_poses, datasets.camera_readings, "eye_to_hand")


def test_hand_eye_result_dict_roundtrip(ur5e):
    ds = generate_hand_eye_dataset(
        MARKER_IN_FLANGE, CAMERA_TRUTH, "eye_to_hand", flange_poses=_hand_eye_poses(ur5e)
    )
    result = solve_hand_eye(ds.flange_poses, ds.camera_readings, "eye_to_hand")
    from service.calibration import HandEyeResult

    back = HandEyeResult.from_dict(result.to_dict())
    assert np.allclose(back.transform.matrix, result.transform.matrix)
    assert np.allclose(back.camera_in_base.matrix, result.camera_in_base.matrix)
    assert back.mode == result.mode and back.n_motions == result.n_motions


# --------------------------------------------------------------------------- #
# 4. 相机模型与视觉抓取
# --------------------------------------------------------------------------- #
def test_camera_from_fov_and_projection_roundtrip():
    cam = CameraIntrinsics.from_fov(1280, 720, 60.0)
    assert cam.cx == 640.0 and cam.cy == 360.0
    assert np.isclose(cam.fx, (640.0) / np.tan(np.radians(30.0)))
    for pixel in ((0.0, 0.0), (1279.0, 719.0), (640.0, 360.0), (300.0, 500.0)):
        depth = 800.0
        p = cam.unproject(pixel, depth)
        assert np.allclose(cam.project(p), pixel, atol=1e-9)
    assert cam.contains((0.0, 0.0)) and not cam.contains((1280.0, 0.0))
    assert "fx=" in cam.summary()

    with pytest.raises(CalibrationError, match="相机后方"):
        cam.project((0.0, 0.0, -1.0))
    with pytest.raises(CalibrationError, match="深度必须为正"):
        cam.unproject((10.0, 10.0), 0.0)
    with pytest.raises(CalibrationError, match="视场角"):
        CameraIntrinsics.from_fov(1280, 720, 200.0)


def test_camera_dict_roundtrip():
    cam = CameraIntrinsics.from_fov(1920, 1080, 45.0)
    back = CameraIntrinsics.from_dict(cam.to_dict())
    assert (back.fx, back.fy, back.cx, back.cy) == (cam.fx, cam.fy, cam.cx, cam.cy)
    assert (back.width, back.height) == (cam.width, cam.height)


def test_pixel_ray_and_plane_intersection():
    cam = CameraIntrinsics.from_fov(1280, 720, 60.0)
    origin, direction = pixel_to_ray_in_base(cam, CAMERA_TRUTH, (640.0, 360.0))
    assert np.allclose(origin, CAMERA_TRUTH.matrix[:3, 3])
    assert np.allclose(direction, [0.0, 0.0, -1.0], atol=1e-9)  # 正下方
    hit = intersect_plane(origin, direction, [0.0, 0.0, 300.0])
    assert np.allclose(hit, [0.0, -400.0, 300.0], atol=1e-9)
    with pytest.raises(CalibrationError, match="平行"):
        intersect_plane(origin, [1.0, 0.0, 0.0], [0.0, 0.0, 300.0])


def test_grasp_pose_from_pixel_closes_the_loop():
    """把工件顶面中心投到像素，再用像素反推抓取位姿——应当回到原处。"""
    cam = CameraIntrinsics.from_fov(1280, 720, 60.0)
    target = (280.0, -400.0, 268.0)  # 工件底面中心（皮带面上）
    pixel = pixel_of_point(cam, CAMERA_TRUTH, target)
    assert cam.contains(pixel)

    grasp = grasp_pose_from_pixel(cam, CAMERA_TRUTH, pixel, plane_z=268.0, grasp_z=328.0)
    assert np.allclose(grasp.matrix[:3, 3], [280.0, -400.0, 328.0], atol=1e-6)
    x, y, z, rx, ry, rz = grasp.to_xyz_rpy()
    assert np.allclose([rx, ry, rz], [180.0, 0.0, 0.0], atol=1e-9)  # 默认垂直向下

    offset = grasp_pose_from_pixel(
        cam, CAMERA_TRUTH, pixel, plane_z=268.0, grasp_z=328.0, xy_offset=(2.0, -3.0)
    )
    assert np.allclose(offset.matrix[:3, 3], [282.0, -403.0, 328.0], atol=1e-6)


def test_pixel_of_point_rejects_points_behind_camera():
    cam = CameraIntrinsics.from_fov(1280, 720, 60.0)
    with pytest.raises(CalibrationError, match="相机后方"):
        pixel_of_point(cam, CAMERA_TRUTH, [0.0, -400.0, 2000.0])  # 相机之上


# --------------------------------------------------------------------------- #
# 5. 标定集：落盘、写回机器人 / URDF、出码
# --------------------------------------------------------------------------- #
def _sample_set(ur5e_rotations) -> CalibrationSet:
    poses = _touch_poses(ur5e_rotations)
    tool_R = Rotation.from_euler("xyz", np.radians([0.0, 0.0, 2.5])).as_matrix()
    R_f0 = poses[0].matrix[:3, :3]
    tcp = calibrate_tcp_6point(
        poses,
        TouchProbe(poses[0], TIP_REF + R_f0 @ tool_R @ np.array([80.0, 0.0, 0.0])),
        TouchProbe(poses[0], TIP_REF + R_f0 @ tool_R @ np.array([0.0, 0.0, 80.0])),
        ref_point=TIP_REF,
    )
    return CalibrationSet(
        name="cell_a",
        tool=tcp,
        base=Pose.from_xyz_rpy(0, 0, 0, 0, 0, 0),
        camera=CameraIntrinsics.from_fov(1280, 720, 60.0),
        notes="单测",
    ).set_frame("托盘坐标系", Pose.from_xyz_rpy(350.0, 450.0, 144.0, 0.0, 0.0, 15.0))


def test_calibration_set_dict_roundtrip(ur5e_rotations):
    cal = _sample_set(ur5e_rotations)
    back = CalibrationSet.from_dict(cal.to_dict())
    assert back.name == cal.name and back.notes == cal.notes
    assert np.allclose(back.tool.offset.matrix, cal.tool.offset.matrix)
    assert np.allclose(back.base.matrix, cal.base.matrix)
    assert np.allclose(back.get_frame("托盘坐标系").matrix, cal.get_frame("托盘坐标系").matrix)
    assert back.get_frame("不存在") is None
    assert back.camera.fx == cal.camera.fx
    lines = back.summary_lines()
    assert any("TCP" in line for line in lines)
    assert any("用户坐标系" in line for line in lines)


def test_calibration_set_save_load(tmp_path, ur5e_rotations):
    cal = _sample_set(ur5e_rotations)
    path = cal.save(tmp_path / "calib.json")
    reloaded = CalibrationSet.load(path)
    assert reloaded.name == "cell_a"
    assert np.allclose(reloaded.tool.offset.matrix, cal.tool.offset.matrix)
    assert json.loads(path.read_text(encoding="utf-8"))["schema"] == "service.calibration/1"


def test_apply_to_robot_sets_tool_and_base(ur5e, ur5e_rotations):
    cal = _sample_set(ur5e_rotations)
    cal.base = Pose.from_xyz_rpy(100.0, 0.0, 0.0, 0.0, 0.0, 90.0)
    applied = cal.apply_to_robot(ur5e)
    assert len(applied) == 2
    assert np.allclose(ur5e.tool.matrix, cal.tool.offset.matrix)
    assert np.allclose(ur5e.base.matrix, cal.base.matrix)

    # 只更新 TCP：基座保持不动
    ur5e.base = Pose()
    cal.apply_to_robot(ur5e, apply_base=False)
    assert np.allclose(ur5e.base.matrix, np.eye(4))

    # 空标定集：必须报错，而不是安静地什么都不做
    with pytest.raises(CalibrationError, match="没有 TCP / 基坐标"):
        CalibrationSet(name="empty").apply_to_robot(ur5e)


def test_write_urdf_writes_calibrated_tcp_as_kinematic_tip(tmp_path, ur5e_rotations):
    if not UR5E_URDF.exists():
        pytest.skip(f"缺少测试用 URDF: {UR5E_URDF}")
    import xml.etree.ElementTree as ET

    cal = _sample_set(ur5e_rotations)
    out = cal.write_urdf(UR5E_URDF, tmp_path / "ur5e_cal.urdf")
    assert out.exists()

    kin0 = RobotKinematics(UR5E_URDF)
    kin1 = RobotKinematics(out)
    assert kin1.tip_link == "tcp"  # 标定后的 TCP 就是运动学末端
    assert kin1.dof == kin0.dof  # 没有多出可动关节

    root = ET.parse(out).getroot()
    joint = next(j for j in root.findall("joint") if j.get("name") == "tcp_joint")
    parent = joint.find("parent").get("link")
    assert joint.find("child").get("link") == "tcp"

    q = np.radians([10.0, -80.0, 80.0, 90.0, 45.0, 30.0])
    offset_m = cal.tool.offset.matrix.copy()
    offset_m[:3, 3] /= 1000.0  # URDF 的 origin 用米
    flange = kin0.all_link_transforms(q)[parent]
    assert np.allclose(kin1.forward_kinematics(q), flange @ offset_m, atol=1e-9)


def test_write_urdf_keeps_existing_tcp_geometry_and_is_idempotent(tmp_path, ur5e_rotations):
    """UR5e 模型自带 tcp link（含显示球）：只改 origin，不删不建，重复写不叠加。"""
    if not UR5E_URDF.exists():
        pytest.skip(f"缺少测试用 URDF: {UR5E_URDF}")
    import xml.etree.ElementTree as ET

    cal = _sample_set(ur5e_rotations)
    first = cal.write_urdf(UR5E_URDF, tmp_path / "a.urdf")
    second = cal.write_urdf(first, tmp_path / "b.urdf")

    root = ET.parse(second).getroot()
    assert len([j for j in root.findall("joint") if j.get("name") == "tcp_joint"]) == 1
    assert len([lk for lk in root.findall("link") if lk.get("name") == "tcp"]) == 1
    # 原有显示几何必须还在（说明是"更新"而不是"删了重建"）
    tcp_link = next(lk for lk in root.findall("link") if lk.get("name") == "tcp")
    assert tcp_link.find("visual") is not None

    q = np.radians([0.0, -90.0, 90.0, -90.0, 90.0, 0.0])
    assert np.allclose(
        RobotKinematics(second).forward_kinematics(q),
        RobotKinematics(first).forward_kinematics(q),
        atol=1e-12,
    )


def test_write_urdf_without_tool_raises(tmp_path):
    with pytest.raises(CalibrationError, match="没有 TCP 结果"):
        CalibrationSet(name="empty").write_urdf(UR5E_URDF, tmp_path / "x.urdf")


def test_post_carries_calibration_header(ur5e, ur5e_rotations):
    if not KR16_URDF.exists():
        pytest.skip(f"缺少测试用 URDF: {KR16_URDF}")
    robot = Robot(str(KR16_URDF), "kr16_2")
    cal = _sample_set(ur5e_rotations)
    prog = (
        ProgramBuilder(robot)
        .apply_calibration(cal)
        .moveJ([0.0] * robot.dof)
        .moveJ([10.0, -20.0, 30.0, 0.0, 10.0, 0.0])
        .build()
    )
    assert prog.calibration is cal
    assert np.allclose(robot.tool.matrix, cal.tool.offset.matrix)  # 标定同时写进机器人

    code = prog.to_post("kuka_krc4", metadata={"program_name": "CAL_TEST"})
    assert "TCP(6point)" in code
    assert "标定集 cell_a" in code
    # 无标定时不得多出任何头部注释（互不影响）
    plain = ProgramBuilder(robot).moveJ([0.0] * robot.dof).build().to_post("kuka_krc4")
    assert "标定集" not in plain and "TCP(6point)" not in plain


# --------------------------------------------------------------------------- #
# 6. 示例脚本（子进程真实运行）
# --------------------------------------------------------------------------- #
def test_vision_pick_demo_script_runs():
    script = REPO_ROOT / "examples" / "vision_pick_demo.py"
    if not script.exists():
        pytest.skip(f"缺少示例脚本: {script}")
    proc = subprocess.run(
        [sys.executable, str(script), "--pixel-noise", "0.0"],
        cwd=str(REPO_ROOT),
        capture_output=True,
        text=True,
        timeout=600,
    )
    assert proc.returncode == 0, proc.stderr[-2000:]
    out = proc.stdout
    for token in ("相机触发抓取演示", "TCP 4 点法", "手眼标定", "视觉 → 抓取位姿", "校验错误 0 条"):
        assert token in out, f"输出缺少 {token!r}\n{out[-1500:]}"

    # 标定集落盘且能被加载（GUI/程序后续可直接用）
    calib_file = REPO_ROOT / "examples" / "assets" / "grasping_workstation_calibration.json"
    assert calib_file.exists()
    loaded = CalibrationSet.load(calib_file)
    assert loaded.tool is not None and loaded.hand_eye is not None
    assert loaded.get_frame("托盘坐标系") is not None
