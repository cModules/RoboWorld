#!/usr/bin/env python3
"""相机触发抓取演示（v0.3.4 标定与手眼）——一条命令跑通"标定 → 视觉 → 抓取"。

这个示例要回答一个很实际的问题：**相机看到工件之后，机器人凭什么知道该去哪儿抓？**
链条是这样的：

1. **TCP 标定**（4 点法 + 6 点法）：真机上吸盘装上去总有偏差，"法兰原点"不是吸盘
   中心。用 4 个不同姿态触碰同一点解出偏移，再沿工具 +X / +Z 各碰一次把方向也
   定下来。
2. **基坐标（工件坐标系）标定**（3 点法）：把输送带/托盘坐标系对齐到机器人基坐标系。
3. **手眼标定**（eye-to-hand）：相机固定在工位上方俯视输送带，解出相机在基坐标系
   中的外参。
4. **视觉检测 → 抓取位姿**：相机给出工件在图像上的像素，经标定后的外参反投影到
   工件支承面，得到机器人该去的目标位姿。
5. **生成程序**：位姿 → IK → 带回放/IO 的程序 → 校验节拍 → 品牌出码。出码头部会
   带上这套标定摘要，下装前现场一眼能核。

全部数据都在本机真实算出来（模板场景、UR5e 运动学、真实后处理），不需要真机；
`noise_mm` 等参数用来模拟视觉与标定噪声，方便看精度量级。

运行::

    python examples/vision_pick_demo.py            # 默认参数
    python examples/vision_pick_demo.py --noise 1.0  # 放大视觉噪声看精度退化
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

from orolp.calibration import (  # noqa: E402
    CalibrationSet,
    CameraIntrinsics,
    TouchProbe,
    calibrate_base_3point,
    calibrate_tcp_4point,
    calibrate_tcp_6point,
    flange_poses_from_joints,
    grasp_pose_from_pixel,
    pixel_of_point,
    solve_hand_eye,
)
from orolp.pose import Pose  # noqa: E402
from orolp.program import ProgramBuilder  # noqa: E402
from orolp.robot import Robot  # noqa: E402
from orolp.world import build_grasping_workstation  # noqa: E402
from orolp.world.demo import demo_pick_place_points  # noqa: E402
from orolp.world.templates import (  # noqa: E402
    _BELT_TOP_Z,
    _CONVEYOR_Y,
    _WORKPIECE_GAP_Z,
    _WORKPIECE_XS,
    WORKPIECE_HEIGHT_MM,
)

#: 出码用的 IO 点表（信号逻辑名 → 物理地址）
IO_MAP = {"gripper_close": "Q0.1", "gripper_open": "Q0.0", "clamped": "I0.3"}

#: 标定集默认落盘位置（示例产物，可被 GUI/程序直接加载）
CALIB_JSON = REPO_ROOT / "examples" / "assets" / "grasping_workstation_calibration.json"

#: 相机安装位（基坐标系，mm + 度）：输送带正上方俯视
CAMERA_MOUNT = (0.0, _CONVEYOR_Y, 1100.0, 180.0, 0.0, 0.0)

#: 真值（"现场实测出来的那套值"）——演示里故意让它与名义值不同，好看到标定确实在起作用
TRUTH_TCP_OFFSET = (2.6, -1.4, 78.9)     # 名义值 (0, 0, 77)
TRUTH_TCP_RPY = (0.0, 0.0, 2.5)          # 4 点法解不出的方向，交给 6 点法
TRUTH_PALLET_FRAME = (350.0, 450.0, 144.0, 0.0, 0.0, 15.0)  # 托盘坐标系：摆在基坐标系里，且旋转 15°（工件没码正）

#: 手眼数据里的标定板（装在夹爪上）：相对法兰的位姿真值
TRUTH_MARKER_IN_FLANGE = (30.0, 0.0, 90.0, 180.0, 0.0, 0.0)


def _flange_poses_with_rotations(rotations: list[np.ndarray], tip_offset: np.ndarray,
                                 ref: np.ndarray) -> list[Pose]:
    """构造"尖端都落在同一点"的法兰位姿（TCP 4 点法数据）。

    只给旋转：位置由 ``t = ref − R·p_tcp`` 反算，保证每个姿态下尖端都落在 ``ref``。
    姿态取自真实 UR5e 可达位形（见 :func:`run_vision_pick`），不是凭空造的旋转。
    """
    poses = []
    for R in rotations:
        m = np.eye(4)
        m[:3, :3] = R
        m[:3, 3] = ref - R @ tip_offset
        poses.append(Pose(m))
    return poses


def run_vision_pick(
    noise_mm: float = 0.15,
    noise_deg: float = 0.03,
    pixel_noise_px: float = 0.2,
    workpiece_index: int = 3,
    seed: int = 7,
    save_calibration: bool = True,
) -> dict:
    """跑通"标定 → 视觉 → 抓取"全流程，返回所有可展示的真实数据。

    Args:
        noise_mm: 手眼标定数据的位置噪声标准差 (mm)
        noise_deg: 手眼标定数据的姿态噪声标准差（度）
        pixel_noise_px: 视觉检测像素噪声标准差（像素）
        workpiece_index: 抓取第几个工件（0 起）
        seed: 随机种子
        save_calibration: 是否把标定集写成 JSON

    Returns:
        含各阶段真值、标定值、误差、节拍与品牌代码的字典
    """
    rng = np.random.default_rng(seed)

    # ------------------------- 0. 抓取工作站 ------------------------- #
    cell = build_grasping_workstation()
    robot = Robot("models/ur/ur5e/ur5e.urdf", "UR5e")
    robot.tool = cell.tcp_flange  # 名义 TCP（还没标定）

    # 示例示教点：给手眼标定提供"真实可达、姿态多样"的机器人位姿
    points_deg, _types = demo_pick_place_points(cell, robot, workpiece_index=workpiece_index)
    flange_poses = flange_poses_from_joints(robot, points_deg)

    # ------------------------- 1. TCP 4 点法 ------------------------- #
    # 取 4 个腕部姿态差异明显的位形（TCP 标定要求姿态多样，否则方程退化）。
    # 注意：抓放示教点的姿态都是 RPY(180,0,0)——用它做 4 点法会直接判"位形退化"，
    # 真机上操作工也是靠**扭手腕**换姿态的，这里如实模拟。
    touch_joints = [
        [10.0, -80.0, 80.0, -90.0, 90.0, 0.0],
        [25.0, -70.0, 100.0, -55.0, 65.0, 35.0],
        [-20.0, -85.0, 70.0, -125.0, 50.0, -30.0],
        [0.0, -75.0, 95.0, -105.0, 80.0, 70.0],
    ]
    rot_sets = [
        p.matrix[:3, :3] for p in flange_poses_from_joints(robot, touch_joints)
    ]
    tip_ref = np.array([420.0, -260.0, 320.0])  # 工装尖碰的那个固定点
    p_true = np.array(TRUTH_TCP_OFFSET, dtype=np.float64)
    touch_poses = _flange_poses_with_rotations(rot_sets, p_true, tip_ref)
    tcp4 = calibrate_tcp_4point(touch_poses, ref_point=tip_ref)
    tcp4_error = float(np.linalg.norm(tcp4.offset.matrix[:3, 3] - p_true))

    # ------------------------- 2. TCP 6 点法（含姿态） ------------------------- #
    R_flange0 = touch_poses[0].matrix[:3, :3]
    tool_R_true = Rotation.from_euler("xyz", np.radians(TRUTH_TCP_RPY)).as_matrix()
    x_probe = TouchProbe(touch_poses[0], tip_ref + R_flange0 @ tool_R_true @ np.array([80.0, 0.0, 0.0]))
    z_probe = TouchProbe(touch_poses[0], tip_ref + R_flange0 @ tool_R_true @ np.array([0.0, 0.0, 80.0]))
    tcp6 = calibrate_tcp_6point(touch_poses, x_probe, z_probe, ref_point=tip_ref)
    tcp6_rot_error = float(
        np.degrees(
            np.linalg.norm(
                Rotation.from_matrix(
                    tcp6.offset.matrix[:3, :3] @ tool_R_true.T
                ).as_rotvec()
            )
        )
    )

    # 标定结果写进仿真：此后 FK / IK / 规划都按标定后的 TCP 算
    robot.tool = tcp6.offset

    # ------------------------- 3. 托盘坐标系 3 点法 ------------------------- #
    # 工件落件位置不写死常数，而是标出一个**托盘坐标系**，落件点按托盘局部坐标算：
    # 托盘挪动/转了个角，只需重标这一次，程序一个字不用改。
    T_truth = Pose.from_xyz_rpy(*TRUTH_PALLET_FRAME).matrix
    local_pts = [(0.0, 0.0, 0.0), (300.0, 0.0, 0.0), (0.0, 300.0, 0.0)]
    base_pts = [T_truth[:3, :3] @ np.array(p) + T_truth[:3, 3] for p in local_pts]
    pallet_frame = calibrate_base_3point(*base_pts)
    frame_error = float(np.linalg.norm(pallet_frame.matrix[:3, 3] - T_truth[:3, 3]))
    frame_angle_error = float(
        np.degrees(
            np.linalg.norm(
                Rotation.from_matrix(pallet_frame.matrix[:3, :3] @ T_truth[:3, :3].T).as_rotvec()
            )
        )
    )

    # ------------------------- 4. 手眼标定（eye-to-hand） ------------------------- #
    camera_truth = Pose.from_xyz_rpy(*CAMERA_MOUNT)
    marker_in_flange = Pose.from_xyz_rpy(*TRUTH_MARKER_IN_FLANGE)

    # 采集手眼数据同样要"扭手腕"：绕同一根轴转是解不出外参的（求解器会直接拦下）。
    # 这里在工位上方取 6 个真实可达位形，每个叠加不同的腕部偏转。
    wrist_offsets = (
        (0.0, 0.0, 0.0),
        (28.0, 18.0, -22.0),
        (-32.0, 22.0, 38.0),
        (18.0, -26.0, 52.0),
        (-22.0, -16.0, -42.0),
        (34.0, 12.0, 16.0),
    )
    hand_eye_joints = [
        [p[0], p[1], p[2], p[3] + d4, p[4] + d5, p[5] + d6]
        for p, (d4, d5, d6) in zip(points_deg[1:7], wrist_offsets)
    ]
    hand_eye_poses = flange_poses_from_joints(robot, hand_eye_joints)

    # 用真实位姿 + 真值外参反算"相机应该读到什么"，再叠加视觉噪声
    from orolp.calibration import generate_hand_eye_dataset

    dataset = generate_hand_eye_dataset(
        marker_in_flange,
        camera_truth,
        mode="eye_to_hand",
        flange_poses=hand_eye_poses,
        noise_mm=noise_mm,
        noise_deg=noise_deg,
    )
    hand_eye = solve_hand_eye(dataset.flange_poses, dataset.camera_readings, mode="eye_to_hand")
    eye_pos_error = float(
        np.linalg.norm(hand_eye.camera_in_base.matrix[:3, 3] - camera_truth.matrix[:3, 3])
    )
    eye_angle_error = float(
        np.degrees(
            np.linalg.norm(
                Rotation.from_matrix(
                    hand_eye.camera_in_base.matrix[:3, :3] @ camera_truth.matrix[:3, :3].T
                ).as_rotvec()
            )
        )
    )

    # ------------------------- 5. 相机看到工件 → 抓取位姿 ------------------------- #
    camera = CameraIntrinsics.from_fov(1280, 720, 60.0)
    idx = min(max(workpiece_index, 0), len(_WORKPIECE_XS) - 1)
    object_bottom_z = _BELT_TOP_Z + _WORKPIECE_GAP_Z
    grasp_z = object_bottom_z + WORKPIECE_HEIGHT_MM
    truth_center = [float(_WORKPIECE_XS[idx]), float(_CONVEYOR_Y), object_bottom_z]

    # 真值相机"看到"的像素（+ 检测噪声），再只用标定结果去反推抓取位姿
    px_truth = np.array(pixel_of_point(camera, camera_truth, truth_center))
    pixel = px_truth + rng.normal(0.0, pixel_noise_px, 2)
    grasp_pose = grasp_pose_from_pixel(
        camera,
        hand_eye.camera_in_base,
        pixel,
        plane_z=object_bottom_z,
        grasp_z=grasp_z,
    )
    truth_grasp = Pose.from_xyz_rpy(truth_center[0], truth_center[1], grasp_z, 180.0, 0.0, 0.0)
    grasp_error = float(
        np.linalg.norm(grasp_pose.matrix[:3, 3] - truth_grasp.matrix[:3, 3])
    )

    # ------------------------- 6. 生成程序 + 校验 + 出码 ------------------------- #
    # 抓取点来自视觉；落件点按**标定出的托盘坐标系**算局部坐标 (0, 0, 工件高)，
    # 而不是写死常数——托盘转 15° 也没关系，位置自动跟着走
    lift = grasp_pose.with_z_offset(120.0)
    place_local = np.array([0.0, 0.0, WORKPIECE_HEIGHT_MM])
    place_xyz = pallet_frame.matrix[:3, :3] @ place_local + pallet_frame.matrix[:3, 3]
    place = Pose.from_xyz_rpy(
        float(place_xyz[0]), float(place_xyz[1]), float(place_xyz[2]), 180.0, 0.0, 0.0
    )
    home = [float(v) for v in cell.home_joints]

    calib = CalibrationSet(
        name="grasping_workstation",
        tool=tcp6,
        hand_eye=hand_eye,
        camera=camera,
        notes=f"vision_pick_demo：像素噪声 {pixel_noise_px}px，手眼噪声 {noise_mm}mm/{noise_deg}°",
    ).set_frame("托盘坐标系", pallet_frame)

    program = (
        ProgramBuilder(robot)
        .apply_calibration(calib)
        .moveJ(home, speed=80)
        .moveL(grasp_pose.with_z_offset(140.0), speed=100)
        .moveL(grasp_pose, speed=40)
        .gripper("close")
        .moveL(lift, speed=80)
        .moveL(place.with_z_offset(150.0), speed=100, velocity=350.0, acceleration=1400.0)
        .moveL(place, speed=40)
        .gripper("open")
        .moveL(place.with_z_offset(150.0), speed=80)
        .moveJ(home, speed=80)
        .build()
    )

    report = program.validate()
    errors = [i for i in report.issues if str(getattr(i, "severity", "")).lower() == "error"]
    traj = program.plan()
    posts = {
        brand: program.to_post(brand, metadata={"program_name": "VISION_PICK", "io_map": IO_MAP})
        for brand in ("ur_polyscope", "fanuc_r30ib")
    }
    calib_path = str(calib.save(CALIB_JSON)) if save_calibration else ""

    return {
        "cell_id": cell.id,
        "workpiece_index": idx,
        "tcp_nominal_mm": cell.tcp_flange.to_xyz_rpy()[:3],
        "tcp_truth_mm": TRUTH_TCP_OFFSET,
        "tcp4_offset_mm": tcp4.offset.to_xyz_rpy()[:3],
        "tcp4_error_mm": tcp4_error,
        "tcp4_residual_mm": tcp4.residual_max_mm,
        "tcp4_ok": tcp4.ok,
        "tcp6_rot_error_deg": tcp6_rot_error,
        "tcp6_summary": tcp6.summary(),
        "frame_error_mm": frame_error,
        "frame_angle_error_deg": frame_angle_error,
        "hand_eye_mode": hand_eye.mode,
        "hand_eye_motions": hand_eye.n_motions,
        "hand_eye_residual_mm": hand_eye.residual_mm,
        "hand_eye_pos_error_mm": eye_pos_error,
        "hand_eye_angle_error_deg": eye_angle_error,
        "camera_truth_pos_mm": camera_truth.to_xyz_rpy()[:3],
        "camera_solved_pos_mm": hand_eye.camera_in_base.to_xyz_rpy()[:3],
        "camera_intrinsics": camera.summary(),
        "pixel": (float(pixel[0]), float(pixel[1])),
        "pixel_truth": (float(px_truth[0]), float(px_truth[1])),
        "grasp_pose": grasp_pose.to_xyz_rpy(),
        "grasp_error_mm": grasp_error,
        "place_pose": place.to_xyz_rpy(),
        "cycle_time": traj.duration(),
        "n_motions": len(program.motions),
        "n_errors": len(errors),
        "error_msgs": [str(getattr(i, "message", i)) for i in errors[:3]],
        "posts": posts,
        "calibration_lines": calib.summary_lines(),
        "calibration_path": calib_path,
        "robot": robot,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="RobotWorld 相机触发抓取演示（v0.3.4）")
    parser.add_argument("--noise", type=float, default=0.15, help="手眼标定位置噪声 (mm)")
    parser.add_argument("--noise-deg", type=float, default=0.03, help="手眼标定姿态噪声 (度)")
    parser.add_argument("--pixel-noise", type=float, default=0.2, help="视觉像素噪声 (px)")
    parser.add_argument("--workpiece", type=int, default=3, help="抓取第几个工件（0 起）")
    parser.add_argument("--code", action="store_true", help="打印品牌出码片段")
    args = parser.parse_args()

    data = run_vision_pick(
        noise_mm=args.noise,
        noise_deg=args.noise_deg,
        pixel_noise_px=args.pixel_noise,
        workpiece_index=args.workpiece,
    )

    print("=" * 74)
    print("RobotWorld · 相机触发抓取演示（标定 → 手眼 → 视觉 → 抓取）")
    print("=" * 74)
    print(f"工作站: {data['cell_id']} | 目标工件: 第 {data['workpiece_index'] + 1} 个")

    print("\n[1] TCP 4 点法（只定位置）")
    print(f"    名义 TCP   : {np.round(data['tcp_nominal_mm'], 2)} mm")
    print(f"    真值 TCP   : {np.round(data['tcp_truth_mm'], 2)} mm")
    print(f"    标定结果   : {np.round(data['tcp4_offset_mm'], 3)} mm")
    print(f"    误差       : {data['tcp4_error_mm']:.2e} mm | "
          f"残差 {data['tcp4_residual_mm']:.4f} mm（{'合格' if data['tcp4_ok'] else '超差'}）")

    print("\n[2] TCP 6 点法（补姿态：沿工具 +X / +Z 各碰一点）")
    print(f"    姿态误差   : {data['tcp6_rot_error_deg']:.2e} °")
    print(f"    {data['tcp6_summary']}")

    print("\n[3] 托盘坐标系 3 点法（落件点不写死：按标定出的托盘系算局部坐标）")
    print(f"    原点误差   : {data['frame_error_mm']:.2e} mm | "
          f"姿态误差 {data['frame_angle_error_deg']:.2e} °")

    print("\n[4] 手眼标定（相机固定在工位上方，eye-to-hand）")
    print(f"    有效运动   : {data['hand_eye_motions']} 段 | "
          f"方程残差 {data['hand_eye_residual_mm']:.3f} mm")
    print(f"    相机真值   : {np.round(data['camera_truth_pos_mm'], 2)} mm")
    print(f"    标定外参   : {np.round(data['camera_solved_pos_mm'], 2)} mm")
    print(f"    位置误差   : {data['hand_eye_pos_error_mm']:.3f} mm | "
          f"姿态误差 {data['hand_eye_angle_error_deg']:.4f} °")

    print("\n[5] 视觉 → 抓取位姿（像素反投影到工件支承面）")
    print(f"    相机内参   : {data['camera_intrinsics']}")
    print(f"    检测像素   : ({data['pixel'][0]:.1f}, {data['pixel'][1]:.1f})"
          f"（真值 {data['pixel_truth'][0]:.1f}, {data['pixel_truth'][1]:.1f}）")
    print(f"    抓取位姿   : {np.round(data['grasp_pose'][:3], 2)} mm")
    print(f"    定位误差   : {data['grasp_error_mm']:.3f} mm（含标定误差 + 像素噪声）")

    print("\n[6] 程序校验与出码")
    print(f"    指令数     : {data['n_motions']} | 节拍 {data['cycle_time']:.2f} s | "
          f"校验错误 {data['n_errors']} 条")
    for msg in data["error_msgs"]:
        print(f"      ! {msg}")
    print(f"    标定集     : {data['calibration_path']}")
    print("    出码头部将携带以下标定摘要（现场核对用）：")
    for line in data["calibration_lines"]:
        print(f"      · {line}")

    if args.code:
        print("\n" + "-" * 74 + "\nURScript（前 22 行）：")
        print("\n".join(data["posts"]["ur_polyscope"].splitlines()[:22]))

    print("\n完成：一条命令跑通了“TCP/基坐标/手眼标定 → 视觉定位 → 抓取程序 → 品牌出码”。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
