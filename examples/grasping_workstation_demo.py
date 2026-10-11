#!/usr/bin/env python3
"""RoboWorld 示例：抓取工作站一键全流程（S5 展示物料）。

无 GUI、一条命令跑通「抓取工作站仿真编程」能力展示的完整链路，
数据全部来自真实 API（模板场景 + 数值 IK + 规划器 + 品牌后处理），
与 GUI「抓取工作站演示（一键）」入口、网页演示站同源：

    模板工作站 → IK 示例示教点 → 程序构建（自动插夹爪 IO）
    → 校验 + 轨迹仿真 → 节拍 → UR / KUKA / ABB / FANUC 出码

运行::

    python examples/grasping_workstation_demo.py
    # 或未安装 core 时：PYTHONPATH=python python examples/grasping_workstation_demo.py

GUI 对照：app.py 菜单 File → 抓取工作站演示（一键）(&D)。
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
for p in (str(REPO_ROOT), str(REPO_ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)

from domain.robot import Robot
from service.program import ProgramBuilder  # noqa: E402
from service.pickplace import gripper_action_for  # noqa: E402
from service.planner import ENVELOPE_TOLERANCE_MM, TrajectoryPlanner  # noqa: E402
from domain.world.demo import demo_pick_place_points  # noqa: E402
from domain.world.templates import build_grasping_workstation  # noqa: E402

#: 演示程序用的 IO 点表（信号逻辑名 → 物理地址），与网页演示一致
IO_MAP = {
    "gripper_close": "Q0.1",
    "gripper_open": "Q0.0",
    "clamped": "I0.3",
}

#: 出码品牌（后处理名）
BRANDS = ("ur_polyscope", "kuka_krc4", "abb_irc5", "fanuc_r30ib")

BRAND_TITLES = {
    "ur_polyscope": "URScript (UR PolyScope)",
    "kuka_krc4": "KUKA KRL (KRC4)",
    "abb_irc5": "ABB RAPID (IRC5)",
    "fanuc_r30ib": "FANUC TP (R-30iB)",
}


def build_demo_program(robot: Robot, cell) -> Program:  # noqa: F821
    """由示教点构建抓取程序——与 GUI ``ProgramPanel.build_program`` 同逻辑。

    全部 moveJ + 按工艺语义自动插入 ``gripper("close"/"open")``，
    保证回放动画、校验、出码共用同一份抓取时序。
    """
    points_deg, types = demo_pick_place_points(cell, robot)
    builder = ProgramBuilder(robot)
    for q, t in zip(points_deg, types):
        builder.moveJ([float(v) for v in q], speed=80)
        action = gripper_action_for(t)
        if action:
            builder.gripper(action)
    return builder.build()


def main() -> int:
    print("=" * 62)
    print("RoboWorld 抓取工作站演示（无 GUI 全流程）")
    print("=" * 62)

    # 1) 一键搭建标准抓取工作站（UR5e + 吸盘 + 输送带 + 4 工件 + 托盘）
    cell = build_grasping_workstation()
    print(f"\n[1] 工作站模板: {cell.name} — {cell.description}")
    print(f"    场景对象: {', '.join(cell.scene.objects)}")
    print(f"    可抓取工件: {', '.join(cell.grasp_objects)}")
    print(f"    落件区: {cell.place_target}")

    # 2) 加载机器人并生成示例示教点（模板 TCP + 数值 IK）
    urdf = cell.scene.robot_urdf
    if urdf and not Path(urdf).is_absolute() and not Path(urdf).exists():
        candidate = REPO_ROOT / urdf
        if candidate.exists():
            urdf = str(candidate)
    robot = Robot(urdf, "UR5e")
    points_deg, types = demo_pick_place_points(cell, robot)
    print(f"\n[2] 示例示教点: {len(points_deg)} 个（五点式抓取循环）")
    print("    语义: " + " → ".join(t.name for t in types))

    # 3) 构建程序（自动插入夹爪 IO）并校验
    #
    # 干涉判定用的是 URDF ``<collision>`` 里的**厂商简化包络体**（UR5e 是
    # 7 个柱/盒），它比真实零件胖：前臂包络盒 59.7×49mm，而真机是 Ø54mm
    # 圆管，盒角外扩最多约 11.6mm。所以这里按 ENVELOPE_TOLERANCE_MM 放行
    # "包络接触"，只把更深的穿透算撞机——否则每个 6 轴机器人的程序都会
    # 因为腕部/前臂的包络互相贴合而满屏报错。
    program = build_demo_program(robot, cell)
    planner = TrajectoryPlanner(robot, scene=cell.scene)
    report = planner.validate(program, margin=-ENVELOPE_TOLERANCE_MM)
    traj = planner.plan(program)
    # 严格口径下（margin=0）的接触清单 = 被容差放行的"包络接触"
    envelope_contacts = planner.check_trajectory_collisions(traj, margin=0.0)
    n_io = sum(
        1
        for item in program.items
        if type(item).__name__ in ("SetDO", "WaitDI", "WaitTime")
    )
    print(f"\n[3] 程序校验: {'通过' if report.ok else '存在错误'}"
          f"（干涉按包络容差 {ENVELOPE_TOLERANCE_MM:g} mm 判定）")
    if envelope_contacts:
        print(f"    包络接触 {len(envelope_contacts)} 处"
              f"（穿透 ≤{ENVELOPE_TOLERANCE_MM:g} mm，厂商包络体是保守模型、"
              f"非真撞；实机落地前建议现场复核）")
        for hit in envelope_contacts[:4]:
            print(f"      ~ {hit}")
        if len(envelope_contacts) > 4:
            print(f"      ~ …另有 {len(envelope_contacts) - 4} 处")
    if not report.ok:
        for err in report.errors:
            print(f"    - {err}")
    print(f"    指令数: {len(program.items)}（含 IO 指令 {n_io} 条）")

    # 4) 轨迹仿真 → 节拍
    cycle = traj.duration()
    print(f"\n[4] 仿真完成: {len(traj.waypoints)} 个路点，节拍 {cycle:.2f} s")

    # 5) 品牌机器人代码
    meta = {"program_name": "GRASP_DEMO", "io_map": IO_MAP}
    for brand in BRANDS:
        code = program.to_post(brand, metadata=meta)
        print(f"\n{'-' * 62}")
        print(f"--- {BRAND_TITLES.get(brand, brand)} ---")
        print("-" * 62)
        print(code)

    print("=" * 62)
    print("演示完成：模板场景 + IK 示教 + 夹爪 IO + 节拍 + 4 品牌出码")
    print("GUI 一键入口：app.py 菜单 File → 抓取工作站演示（一键）")
    print("=" * 62)
    return 0 if report.ok else 1


if __name__ == "__main__":
    sys.exit(main())
