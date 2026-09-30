#!/usr/bin/env python3
"""RobotWorld 示例：数字孪生与云协作全流程（v1.0.3）。

一条命令跑通"仿真 → 云端 → 回读 → 偏差比对 → 成因定位"的闭环，数据全部来自
真实 API（模板工作站 + IK + 规划器 + 调试会话 + 孪生层），无 GUI：

    抓取工作站程序 → 规划轨迹
      → ① 仿真孪生轨迹（SimLink + DebugSession）
      → ② "真机"孪生轨迹（合成传动误差模型：零位偏置/传动比/噪声/伺服滞后）
      → ③ 上传云端（程序 + 两条轨迹），真机上报状态并检查"回显新鲜度"
      → ④ 跨源对齐 + 偏差统计 + 高/低速分解（标定 vs 伺服）
      → ⑤ 偏差曲线 SVG 落盘

关于"真机数据"的诚实说明
------------------------

**本示例没有真机**。②里的"真机"是 :class:`service.twinsync.PlantModel` 合成的传动
误差模型——它按物理规律作用（常量偏置、随关节角线性的传动比误差、一阶伺服滞后
的稳态误差 = τ·ω），因此可以用来验证整条孪生链路、也可以当作故障注入来标定
"偏差多大该报警"，但**它的数字不是现场实测，不能当验收依据引用**。代码里所有
这些轨迹都带 ``source="plant"``，报告抬头也照实写。

接真机时：把 :class:`~domain.link.SimLink` / :class:`~service.twinsync.PlantLink` 换成
真机 :class:`~domain.link.ControllerLink` 实现即可，孪生层与报告逻辑一行不用改。

运行::

    python examples/twin_sync_demo.py
    # 或未安装 core 时：PYTHONPATH=python python examples/twin_sync_demo.py
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
for p in (str(REPO_ROOT), str(REPO_ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)

from domain.robot import Robot
from service.program import ProgramBuilder  # noqa: E402
from service.debug import DebugSession  # noqa: E402
from domain.link import SimLink  # noqa: E402
from service.pickplace import gripper_action_for  # noqa: E402
from service.planner import TrajectoryPlanner  # noqa: E402
from service.twinsync import (
    CloudLink,
    LocalCloudStore,
    PlantModel,
    capture_from_session,
    capture_plant_trace,
    compare_traces,
    deviation_chart_svg,
)
from domain.world.demo import demo_pick_place_points  # noqa: E402
from domain.world.templates import build_grasping_workstation  # noqa: E402

PROGRAM_NAME = "GRASP_DEMO"

#: 场景一：调好的机器——小偏置、小传动误差、伺服带宽够
TUNED_PLANT = PlantModel(
    joint_bias_deg=[0.01, -0.008, 0.012, 0.0, -0.006, 0.005],
    scale_error=2e-5,
    noise_deg=0.0008,
    lag_tau=0.004,
    seed=7,
)

#: 场景二：零点动过 / 减速比标称误差大 / 位置环增益偏低
POOR_PLANT = PlantModel(
    joint_bias_deg=[0.35, -0.28, 0.30, 0.0, -0.22, 0.18],
    scale_error=8e-4,
    noise_deg=0.01,
    lag_tau=0.02,
    seed=7,
)


def build_program(robot: Robot, cell):
    """构建抓取程序——与 GUI 一键入口同一条数据通路。"""
    points_deg, types = demo_pick_place_points(cell, robot)
    builder = ProgramBuilder(robot)
    for q, t in zip(points_deg, types):
        builder.moveJ([float(v) for v in q], speed=80)
        action = gripper_action_for(t)
        if action:
            builder.gripper(action)
    return builder.build()


def main() -> int:
    print("=" * 72)
    print("RobotWorld 数字孪生与云协作演示（v1.0.3 · 无 GUI 全流程）")
    print("=" * 72)

    # 1) 规划：模板工作站 → 示教点 → 程序 → 轨迹
    cell = build_grasping_workstation()
    robot = Robot(cell.scene.robot_urdf, "UR5e")
    program = build_program(robot, cell)
    planner = TrajectoryPlanner(robot, scene=cell.scene)
    traj = planner.plan(program)
    print(f"\n[1] 轨迹就绪：{len(traj.waypoints)} 个路点，节拍 {traj.duration():.2f} s")

    # 2) 仿真侧孪生轨迹（复用 DebugSession.deviation 的定义）
    sim_link = SimLink(robot)
    sim_link.connect()
    sim_trace = capture_from_session(
        DebugSession(traj, sim_link, robot=robot),
        source="sim",
        robot_name="UR5e",
        program_name=PROGRAM_NAME,
    )
    print(
        f"\n[2] 仿真孪生轨迹：{len(sim_trace)} 个样本，"
        f"规划 vs 回读偏差峰值 {sim_trace.max_pos_dev():.4f} mm（FK/IK 数值精度量级）"
    )

    # 3) 云端：程序 + 轨迹上行
    cloud_dir = Path(tempfile.mkdtemp(prefix="robotworld_cloud_"))
    cloud = CloudLink(LocalCloudStore(cloud_dir), robot=robot, namespace="demo-cell")
    cloud.connect()
    ur_code = program.to_post("ur_polyscope", metadata={"program_name": PROGRAM_NAME})
    print(f"\n[3] 云端（文件式对象存储 {cloud_dir}）")
    print(f"    程序上传  : {cloud.upload_program(PROGRAM_NAME, ur_code, {'ext': 'urp', 'brand': 'ur'})}")
    print(f"    轨迹上传  : {cloud.publish_trace(sim_trace)}")
    manifest = cloud.program_manifest(PROGRAM_NAME)
    print(f"    部署清单  : {manifest['lines']} 行 / {manifest['bytes']} 字节")

    # 4) 真机侧（合成模型）→ 上报状态 → 上传轨迹
    results = []
    for label, plant in (("调好的机器", TUNED_PLANT), ("差调机的机器", POOR_PLANT)):
        real_trace = capture_plant_trace(
            traj, robot, plant, program_name=PROGRAM_NAME, notes=f"{label}：{plant.summarize()}"
        )
        name = f"{PROGRAM_NAME}_{'tuned' if plant is TUNED_PLANT else 'poor'}"
        cloud.publish_trace(real_trace, name=name)
        cloud.push_status(real_trace.snapshots[-1].actual_q, source="plant")
        report = compare_traces(
            sim_trace, real_trace, tol_pos_mm=1.0, tol_rot_deg=1.0, max_dt=0.05
        )
        results.append((label, plant, real_trace, report))

        # 真机上报新鲜度：回显的是"云端最后一条上报"，有多旧必须能查
        age = cloud.status_age()
        print(f"\n[4] {label}（{plant.summarize()}）")
        print(f"    真机轨迹上传: cloud://demo-cell/traces/{name}.jsonl")
        print(f"    状态回读    : {cloud.read_joints().round(3)}")
        print(f"    回显新鲜度  : {age:.3f} s（阈值 5s 内视为实时 → {cloud.is_fresh(5.0)}）")

    # 5) 偏差报告 + 成因定位 + 曲线落盘
    out_dir = REPO_ROOT / "deploy" / "web" / "assets"
    out_dir.mkdir(parents=True, exist_ok=True)
    print("\n[5] 跨源偏差报告（仿真 vs 真机）")
    for label, plant, real_trace, report in results:
        print("-" * 72)
        print(f"### {label}")
        print(report.format(max_rows=5))
        print(report.format_causes())
        svg_path = out_dir / f"twin_deviation_{'tuned' if plant is TUNED_PLANT else 'poor'}.svg"
        svg_path.write_text(
            deviation_chart_svg(
                report,
                theme="dark",
                title=f"TCP 偏差 · {label}（sim vs plant）· 容差 {report.tol_pos_mm:g} mm",
            ),
            encoding="utf-8",
        )
        print(f"  曲线已落盘: {svg_path.relative_to(REPO_ROOT)}")

    # 6) 云端清单
    print("\n[6] 云端现状")
    print(f"    程序: {cloud.list_programs()}")
    print(f"    轨迹: {cloud.list_traces()}")
    for name in cloud.list_traces():
        meta = cloud.trace_meta(name)
        print(
            f"      · {name}: source={meta['source']} samples={meta['samples']} "
            f"max_dev={meta['max_pos_mm']} mm"
        )
    back = cloud.fetch_trace(f"{PROGRAM_NAME}_tuned")
    print(f"    回读校验: {len(back)} 个样本，source={back.source}")

    print("=" * 72)
    print("闭环完成：仿真孪生 → 云上传 → 真机回读 → 偏差比对 → 成因定位 → 曲线")
    print("注意：本例'真机'为 PlantModel 合成模型（source=plant），非现场实测数据。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
