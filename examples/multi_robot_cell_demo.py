#!/usr/bin/env python3
"""RobotWorld 示例：双机器人共享工作单元（v1.0.1 多机器人协同）。

场景
----
两台 UR5e 面对面站在同一张工作台两侧（各自坐在 800mm 高的立柱上），
中间是**共享交接区**：

    机器人A 取件台          共享工作台（交接点）           机器人B 出料台
    x=-1150                    x=0                          x=+1150
      ▲                          ▲                            ▲
      └── 机器人A 基座 x=-620 ────┘   └──── 机器人B 基座 x=+620

A 把工件从左侧托盘搬到中间交接点，B 再把它从交接点搬到右侧出料箱。
两台机器人的轨迹都穿过共享交接区（**前后错开时间共用同一片空间**）——这正是
多机器人现场最典型的冲突来源：单看每一台的轨迹都合法，**放在同一条时间轴上**
才会发现两机在同刻擦肩。

关于"冲突"的口径（本示例刻意说清楚，不用夸张说法）
--------------------------------------------------
本示例**不人为制造穿透**——那要靠改几何才做得出来，属测试夹具的活
（见 ``tests/test_multirobot.py`` 里判定"必须错峰"的用例）。现场真正会遇到的
是另一件事：

* **擦肩**：净空取严格值（0mm）时两机"没撞"，但最近只差几十毫米——现场没人
  敢这么跑，所以真正的判据是**安全净空**（``clearance``）；
* **错峰**：把净空放到 60mm，同刻齐发会暴露一片近失；``greedy`` 只需把后一台
  机器人推迟几秒就能全部消掉，总节拍仍显著优于串行。

本示例演示三件事
----------------
1. **共享场景**：``Scene.add_robot()`` 加两台机器人，各自独立基座位姿、
   关节与运动学链；
2. **互避碰**：``CollisionChecker.check_robot_pair()`` 检出机器人-机器人
   干涉（含最近连杆对与最小间距）；
3. **节拍统筹**：``schedule_cell()`` 把两条轨迹排到一条时间轴上——严格口径看
   干涉、安全净空口径看擦肩，再看 ``greedy`` 如何错峰把它们清干净。

用法::

    python examples/multi_robot_cell_demo.py                # 完整演示
    python examples/multi_robot_cell_demo.py --json out.json # 顺带导出摘要
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

import numpy as np  # noqa: E402

from domain.pose import Pose
from domain.robot import Robot  # noqa: E402
from domain.collision import CollisionChecker  # noqa: E402
from service.multirobot import RobotJob, schedule_cell  # noqa: E402
from service.planner import ENVELOPE_TOLERANCE_MM, TrajectoryPlanner  # noqa: E402
from domain.world.scene import Scene  # noqa: E402

UR5E = REPO_ROOT / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"

#: 工作平面高度：机器人立柱顶面 / 台面 / 托盘面（毫米）
PLANE_Z = 800.0

#: 工件边长（毫米）
PART_SIZE = 60.0

#: 夹爪 TCP 相对法兰的偏移（毫米）
TOOL = Pose.from_xyz_rpy(0.0, 0.0, 80.0, 0.0, 0.0, 0.0)

#: 抓取姿态：工具 Z 轴朝下
GRASP_RPY = (180.0, 0.0, 0.0)

#: 接近点抬高（毫米）
APPROACH = 100.0

#: 工件垫块高度（毫米）：把抓取点抬高，避免前臂包络蹭台面。
#: 标定值 80：实测垫块 0/40/60/100mm 时最深包络穿透分别为 29.2/25.9/23.4/22.8mm
#: （超过 ENVELOPE_TOLERANCE_MM=20mm → 判为撞机），80mm 时降到 19.1mm 且两
#: 台机器人单机校验都通过。再高会让 TCP 远离取件台、腕部在交接点更难分开。
RISER_H = 80.0

#: 机器人基座位置（面对面）
BASE_A = Pose.from_xyz_rpy(-620.0, 0.0, PLANE_Z, 0.0, 0.0, 0.0)
BASE_B = Pose.from_xyz_rpy(620.0, 0.0, PLANE_Z, 0.0, 0.0, 180.0)

#: 关键位置：(x, y) 平面坐标，z 由 PAR T_SIZE 与 PLANE_Z 推出
FEED_A = (-1150.0, 0.0)   # A 的取件托盘中心
HANDOFF = (0.0, 0.0)      # 共享交接点
BIN_B = (1150.0, 0.0)     # B 的出料箱中心

#: 关节零位/待机位形（UR5e 常用 home）
HOME = [0.0, -90.0, 90.0, -90.0, 90.0, 0.0]

#: 节拍统筹采样步长（秒）
SAMPLE_DT = 0.05

#: 安全净空（毫米）：现场口径——间距小于它就算"擦肩"，即便没穿透。
#: 60mm 约等于一台 UR5e 前臂的半径，是有实际意义的保守值；净空取 0 只能回答
#: "撞没撞"，回答不了"敢不敢跑"。
CLEARANCE_MM = 60.0


# --------------------------------------------------------------------------- #
# 场景
# --------------------------------------------------------------------------- #


def build_cell() -> Scene:
    """搭出双机器人共享工作单元。"""
    scene = Scene()

    # 两台机器人：各自基座位姿、独立运动学链
    scene.add_robot(UR5E, HOME, name="机器人A", pose=BASE_A)
    scene.add_robot(UR5E, HOME, name="机器人B", pose=BASE_B)

    # 立柱（把机器人垫到工作平面高度）
    for name, base in (("立柱A", BASE_A), ("立柱B", BASE_B)):
        x, y = float(base.matrix[0, 3]), float(base.matrix[1, 3])
        scene.add(
            "pedestal", name=name,
            pose=Pose.from_xyz_rpy(float(x), float(y), 0.0, 0.0, 0.0, 0.0),
            params={"height": PLANE_Z, "top_size": 400.0},
        )

    # 共享工作台（中间）与两侧托盘/料箱，台面都在 PLANE_Z
    scene.add(
        "worktable", name="共享工作台",
        pose=Pose.from_xyz_rpy(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        params={"length": 900.0, "width": 700.0, "height": PLANE_Z},
    )
    for name, (x, y) in (("取件台", FEED_A), ("出料台", BIN_B)):
        scene.add(
            "worktable", name=name,
            pose=Pose.from_xyz_rpy(x, y, 0.0, 0.0, 0.0, 0.0),
            params={"length": 500.0, "width": 500.0, "height": PLANE_Z},
        )
        # 垫块：把抓取点抬高 RISER_H，远离台面（保守包络贴台面会误报）。
        # 注意 workpiece_box 的几何是**底面锚定**（_T(0,0,h/2)），
        # 所以位姿 z 就是它的底面高度，直接放在台面 PLANE_Z 上。
        scene.add(
            "workpiece_box", name=f"{name}垫块",
            pose=Pose.from_xyz_rpy(x, y, PLANE_Z, 0.0, 0.0, 0.0),
            params={"length": 240.0, "width": 240.0, "height": RISER_H},
        )

    # 工件：待在 A 的取件台上，底面贴垫块顶面
    scene.add(
        "workpiece_box", name="工件",
        pose=Pose.from_xyz_rpy(FEED_A[0], FEED_A[1], _part_bottom_z(), 0.0, 0.0, 0.0),
        params={"length": PART_SIZE, "width": PART_SIZE, "height": PART_SIZE},
    )
    return scene


# --------------------------------------------------------------------------- #
# 程序
# --------------------------------------------------------------------------- #


def _part_bottom_z() -> float:
    """工件**底面**高度（毫米）= 垫块顶面。

    ``workpiece_box`` 的几何是底面锚定的（组件内部用 ``_T(0,0,h/2)``
    把盒子抬到 ``[0, h]``），所以场景里给的位姿 z 就是底面高度——这一点
    很容易和"位姿 z = 几何中心"混掉，混掉的结果是工件半数沉进垫块里、
    抓取点也跟着低 30mm。
    """
    return PLANE_Z + RISER_H


def _grasp_z(lift: float = 0.0) -> float:
    """抓取点高度（毫米）：工件**顶面** + 抬高量。

    吸取式抓取的 TCP 落在工件顶面，而不是工件中心。
    """
    return _part_bottom_z() + PART_SIZE + lift


def _grasp_pose(x: float, y: float, lift: float = 0.0) -> Pose:
    """(x, y) 处的抓取位姿；``lift`` 抬高量（毫米）。"""
    return Pose.from_xyz_rpy(x, y, _grasp_z(lift), *GRASP_RPY)


def _ik(robot: Robot, pose: Pose, seed: list[float], what: str) -> list[float]:
    """反解；失败直接抛出可读错误（示例脚本不该静默降级）。"""
    xyz = pose.matrix[:3, 3]
    where = f"({xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f})"
    try:
        joints = robot.inverse_kinematics(pose, guess=seed)
    except Exception as exc:  # KinematicsError：超工作空间 / 奇异
        raise RuntimeError(
            f"{robot.name} 反解失败：{what} @ {where} —— {exc}"
        ) from exc
    if joints is None:
        raise RuntimeError(f"{robot.name} 反解失败：{what} @ {where}")
    return [float(v) for v in joints]


def build_program(robot: Robot, source: tuple[float, float],
                  target: tuple[float, float]) -> "RobotProgram":  # noqa: F821
    """取件 → 搬运 → 放下 → 回待机。

    路径刻意穿过共享交接区，让多机器人冲突真实存在。
    """
    pick = _grasp_pose(*source)
    pick_above = _grasp_pose(*source, lift=APPROACH)
    place = _grasp_pose(*target)
    place_above = _grasp_pose(*target, lift=APPROACH)

    q_pick = _ik(robot, pick, HOME, "取件点")
    q_pick_above = _ik(robot, pick_above, q_pick, "取件接近点")
    q_place = _ik(robot, place, q_pick_above, "放置点")
    q_place_above = _ik(robot, place_above, q_place, "放置接近点")

    return (
        robot.moveJ(HOME, speed=80)
        .then(robot.moveJ(q_pick_above, speed=60))
        .then(robot.moveL(pick, speed=40))
        .gripper("close")
        .then(robot.moveL(pick_above, speed=40))
        .then(robot.moveJ(q_place_above, speed=60))
        .then(robot.moveL(place, speed=40))
        .gripper("open")
        .then(robot.moveL(place_above, speed=40))
        .then(robot.moveJ(HOME, speed=80))
    )


# --------------------------------------------------------------------------- #
# 演示
# --------------------------------------------------------------------------- #


def _robot_robot(schedule) -> list:
    """只挑机器人-机器人干涉（机器人-环境由单机轨迹决定，错峰消不掉）。"""
    return [i for i in schedule.interferences if i.kind == "robot-robot"]


def _jobs_for_schedule(jobs: list) -> list:
    """重建一批 ``RobotJob``：``schedule_cell`` 会原地改写 ``start``。"""
    return [
        RobotJob(robot=j.robot, trajectory=j.trajectory, label=j.label)
        for j in jobs
    ]


def run_demo(export_json: Path | None = None) -> dict:
    """跑通整个演示，返回结构化摘要。"""
    scene = build_cell()

    robots = {
        "机器人A": Robot(str(UR5E), "机器人A"),
        "机器人B": Robot(str(UR5E), "机器人B"),
    }
    robots["机器人A"].base = BASE_A
    robots["机器人B"].base = BASE_B
    for r in robots.values():
        r.tool = TOOL

    print("=" * 74)
    print("双机器人共享工作单元 · 多机器人协同演示（v1.0.1）")
    print("=" * 74)
    print(f"场景对象：{list(scene.objects)}")
    print(f"机器人数：{len(scene.robot_names)} -> {scene.robot_names}")
    print(f"主机器人：{scene.robot_names[0]}，独立运动学链已建立"
          f"（自由度 {scene.kinematics_of('机器人A').dof}）")

    # ---------------- 1. 各自规划与校验 ----------------
    jobs: list[RobotJob] = []
    per_robot = {}
    print("\n[1] 单机规划与校验（各自独立视角）")
    for name, (src, dst) in (
        ("机器人A", (FEED_A, HANDOFF)),
        ("机器人B", (HANDOFF, BIN_B)),
    ):
        program = build_program(robots[name], src, dst)
        planner = TrajectoryPlanner(robots[name], scene=scene, robot_name=name)
        # 干涉按包络容差判定：URDF <collision> 是厂商保守包络盒（比真机胖），
        # 浅穿透是模型误差而非真撞，详见 service.planner.ENVELOPE_TOLERANCE_MM
        report = planner.validate(program, margin=-ENVELOPE_TOLERANCE_MM)
        traj = planner.plan(program)
        envelope = planner.check_trajectory_collisions(traj, margin=0.0)
        jobs.append(RobotJob(robot=name, trajectory=traj, program=program))
        per_robot[name] = {
            "cycle_time": round(traj.duration(), 3),
            "waypoints": traj.num_waypoints(),
            "errors": sum(1 for i in report.issues if i.severity == "error"),
            "envelope_contacts": len(envelope),
            "ok": report.ok,
        }
        print(f"    {name}：节拍 {traj.duration():.2f} s，"
              f"{traj.num_waypoints()} 个轨迹点，"
              f"校验{'通过' if report.ok else '失败'}"
              f"（错误 {per_robot[name]['errors']}，"
              f"包络接触 {len(envelope)} 处）")
        if not report.ok:
            for issue in report.issues:
                if issue.severity == "error":
                    print(f"      ! {issue.message}")

    # ---------------- 2. 互避碰：同刻位形 ----------------
    print("\n[2] 互避碰：两台机器人同时在交接点")
    checker = CollisionChecker(scene, margin=-0.5)
    q_handoff = {}
    for name in ("机器人A", "机器人B"):
        q_handoff[name] = _ik(
            robots[name], _grasp_pose(*HANDOFF), HOME, f"{name} 交接点"
        )
    hits = checker.check_robot_pair(
        "机器人A", "机器人B", q_handoff["机器人A"], q_handoff["机器人B"]
    )
    sep, la, lb = checker.separation_between(
        "机器人A", "机器人B", q_handoff["机器人A"], q_handoff["机器人B"]
    )
    pair_report = {
        "collisions": [str(h) for h in hits],
        "separation_mm": round(float(sep), 2),
        "closest_links": [la, lb],
    }
    print(f"    干涉 {len(hits)} 处；最近连杆对 {la} ↔ {lb}，"
          f"最小间距 {sep:+.1f} mm")
    for h in hits[:5]:
        print(f"      · {h}")

    # ---------------- 3. 节拍统筹 ----------------
    print("\n[3] 节拍统筹（同一条时间轴，采样步长 "
          f"{SAMPLE_DT:g}s；净空 0 = 只问撞没撞，"
          f"净空 {CLEARANCE_MM:g}mm = 问敢不敢跑）")
    schedules = {}
    for strategy in ("parallel", "serial", "greedy"):
        sched = schedule_cell(
            scene,
            _jobs_for_schedule(jobs),
            strategy=strategy,
            sample_dt=SAMPLE_DT,
            clearance=0.0,
            search_step=0.5,
        )
        schedules[strategy] = sched
        print(f"\n  --- 策略 {strategy} ---")
        print("  " + sched.format().replace("\n", "\n  "))

    # 现场真正会用的口径：留安全净空。净空取 0 只能回答"撞没撞"，回答不了
    # "敢不敢跑"——两机在交接区前后错开时间共用空间，严格口径下不穿透，但
    # 间距只有几十毫米。同一批作业、同一组轨迹，仅把判据从"穿透"改成"擦肩"，
    # 结论就完全不同：齐发必须错峰。
    guarded = schedule_cell(
        scene,
        _jobs_for_schedule(jobs),
        strategy="greedy",
        sample_dt=SAMPLE_DT,
        clearance=CLEARANCE_MM,
        search_step=0.5,
    )
    guarded_parallel = schedule_cell(
        scene,
        _jobs_for_schedule(jobs),
        strategy="parallel",
        sample_dt=SAMPLE_DT,
        clearance=CLEARANCE_MM,
        search_step=0.5,
    )

    par = schedules["parallel"]
    ser = schedules["serial"]
    gre = schedules["greedy"]
    par60 = _robot_robot(guarded_parallel)
    gre60 = _robot_robot(guarded)
    closest = min(par60, key=lambda i: i.separation_mm) if par60 else None

    print(f"\n  --- 安全净空 {CLEARANCE_MM:g}mm：同刻齐发 vs 贪心错峰 ---")
    if closest is not None:
        print(f"    同刻齐发：{len(par60)} 处近失，最近 {closest}"
              f"—— 擦肩而非穿透（净空 0 时判为'不干涉'）")
    else:
        print("    同刻齐发：无近失")
    print(f"    贪心错峰：机器人之间 "
          + ("无干涉" if not gre60 else f"{len(gre60)} 处")
          + f"，总节拍 {guarded.makespan:.2f} s（起始偏移 "
          + "，".join(f"{j.name} {j.start:.2f}s" for j in guarded.jobs.values())
          + "）")

    summary = {
        "robots": per_robot,
        "handoff": pair_report,
        "clearance_mm": CLEARANCE_MM,
        "near_miss": {
            "clearance": CLEARANCE_MM,
            "n_parallel": len(par60),
            "n_greedy": len(gre60),
            "closest": str(closest) if closest is not None else None,
            "min_separation_mm": (
                round(closest.separation_mm, 2) if closest is not None else None
            ),
            "closest_at_s": (
                round(closest.time, 2) if closest is not None else None
            ),
            "closest_links": (
                [f"{closest.robot_a}:{closest.link_a}", f"{closest.other}:{closest.link_b}"]
                if closest is not None
                else None
            ),
        },
        "schedules": {
            "parallel": par.summary(),
            "serial": ser.summary(),
            "greedy": gre.summary(),
            "parallel_clearance_60": guarded_parallel.summary(),
            "greedy_clearance_60": guarded.summary(),
        },
        "makespan": {
            "parallel": round(par.makespan, 3),
            "serial": round(ser.makespan, 3),
            "greedy": round(gre.makespan, 3),
            "greedy_clearance_60": round(guarded.makespan, 3),
        },
    }

    print("\n[4] 结论")
    print(f"    净空 0 口径（只问撞没撞）：并行 {len(par.interferences)} 处干涉，"
          f"总节拍 {par.makespan:.2f} s —— 与贪心同值，错峰无事可做")
    if closest is not None:
        print(f"    净空 {CLEARANCE_MM:g}mm 口径（问敢不敢跑）：同刻齐发 "
              f"{len(par60)} 处近失，最近 {closest.separation_mm:+.1f} mm"
              f" @ t={closest.time:.2f}s（{closest.link_a} ↔ {closest.link_b}）")
        print(f"      → 贪心把 {max(guarded.jobs.values(), key=lambda j: j.start).name} "
              f"推迟 {max(j.start for j in guarded.jobs.values()):.2f}s，"
              f"{len(gre60)} 处近失清零，总节拍 {guarded.makespan:.2f} s")
        if guarded.makespan < ser.makespan:
            print(f"      → 即便如此仍比串行 {ser.makespan:.2f} s 省 "
                  f"{ser.makespan - guarded.makespan:.2f} s"
                  f"（耗时仅为串行的 {guarded.makespan / ser.makespan:.0%}）")
    print(f"    总节拍：并行 {par.makespan:.2f} s / 串行 {ser.makespan:.2f} s / "
          f"贪心 {gre.makespan:.2f} s")
    if gre.makespan < ser.makespan:
        print(f"    净空 0 口径下贪心比串行省 {ser.makespan - gre.makespan:.2f} s"
              f"（耗时 {gre.makespan / ser.makespan:.0%}，即省 "
              f"{1 - gre.makespan / ser.makespan:.0%}）")
    if gre.env_interferences:
        print(f"    另有机器人-环境包络触碰 {len(gre.env_interferences)} 处"
              f"（由单机轨迹决定，错峰消不掉）：")
        for hit in gre.env_interferences[:5]:
            print(f"      · {hit}")

    if export_json is not None:
        export_json.write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print(f"\n摘要已写入 {export_json}")

    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description="双机器人共享工作单元演示")
    parser.add_argument("--json", type=Path, default=None, help="导出摘要 JSON")
    args = parser.parse_args()
    run_demo(export_json=args.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
