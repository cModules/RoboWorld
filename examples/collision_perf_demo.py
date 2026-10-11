#!/usr/bin/env python3
"""RoboWorld 示例：干涉检查的性能与采样口径（v1.1.9）。

一条命令跑通四件事，全部是现场实算的数字（没有写死的常数）：

1. **等价性对拍**——把几何内核（有缓存、批量 SAT、标量 AABB）与同文件的朴素
   参考实现逐位对拍：AABB / 顶点 / SAT 轴 / 穿透深度 / 碰撞判定。
2. **分段实测**——在真实抓取工作站场景上，按不同采样口径量"要花多少、查到
   多少"，并给出每几个路点查一次的**采样密度**。
3. **广相账本**——候选对里有多少被 AABB 直接挡掉、多少次真的进了 SAT。
4. **口径说明**——这份基准不回答什么。

运行::

    python examples/collision_perf_demo.py
    python examples/collision_perf_demo.py --motions 300      # 加大规模
    PYTHONPATH=python python examples/collision_perf_demo.py  # 未安装 core 时
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
for p in (str(REPO_ROOT), str(REPO_ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)

from domain.robot import Robot  # noqa: E402
from domain.collision import (
    collision_stats,
    reset_collision_stats,
    sample_indices,
)
from service.perf import (
    SWEEP_BASE,
    collision_profile,
    compare_collision_kernels,
    sweep_program,
)
from service.planner import TrajectoryPlanner  # noqa: E402
from domain.world.geometry import geometry_stats, reset_geometry_stats  # noqa: E402
from domain.world.templates import build_grasping_workstation  # noqa: E402

URDF = REPO_ROOT / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"


def section(title: str) -> None:
    print()
    print("=" * 74)
    print(title)
    print("=" * 74)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="干涉检查性能与采样口径基准")
    parser.add_argument("--motions", type=int, default=200, help="基准程序的运动条数")
    parser.add_argument("--repeat", type=int, default=2, help="每行重复次数（取最优）")
    args = parser.parse_args(argv)

    # ---- 1) 等价性对拍 ----
    section("[1] 几何内核等价性对拍（生产实现 vs 朴素参考实现）")
    cmp = compare_collision_kernels()
    print(cmp.format())
    print("  结论：{}".format("逐位一致 ✅" if cmp.identical else "存在差异 ❌"))

    # ---- 2) 分段实测 ----
    section("[2] 干涉检查分段实测（真实抓取工作站场景）")
    cell = build_grasping_workstation()
    robot = Robot(str(URDF), "UR5e")
    robot.set_joints(list(SWEEP_BASE))
    planner = TrajectoryPlanner(robot, scene=cell.scene)
    program = sweep_program(robot, args.motions)
    profile = collision_profile(planner, program, repeats=max(1, args.repeat))
    print(profile.format())

    # ---- 3) 广相账本 ----
    section("[3] 广相 / 窄相账本（默认口径那一行的工作量）")
    traj = planner.plan(program)
    n_samples = len(sample_indices(traj.waypoints))
    reset_collision_stats()
    reset_geometry_stats()
    # v1.2.2 起干涉检查有结果缓存——账本必须量"真算"，计时/统计期间临时关闭
    from service.incremental import get_collision_cache, set_collision_cache

    _saved_cache = get_collision_cache()
    set_collision_cache(None)
    try:
        hits = planner.check_trajectory_collisions(traj, margin=profile.margin)
    finally:
        set_collision_cache(_saved_cache)
    col = collision_stats()
    geo = geometry_stats()
    print("  采样 {n} 次（每 {per:.1f} 路点 1 次）· 命中 {h} 处".format(
        n=n_samples, per=profile.n_waypoints / max(1, n_samples), h=len(hits)))
    print("  连杆-物体候选对 {tests} 次 · 广相 AABB 挡掉 {rej} 次（{rate:.1f}%）".format(
        tests=col["level1_tests"], rej=col["level1_rejects"],
        rate=100.0 * col["level1_rejects"] / max(1, col["level1_tests"])))
    print("  进入窄相的零件对 {narrow} 次 · 其中真的跑 SAT 的 {sat} 次（{rate:.1f}%）".format(
        narrow=col["narrow_pairs"], sat=geo["sat_checks"],
        rate=100.0 * geo["sat_checks"] / max(1, col["narrow_pairs"])))
    print("  局部几何缓存：条目 {e} 条 · 命中 {h} 次 / 未命中 {m} 次（命中率 {r:.1f}%）".format(
        e=geo["cache_entries"], h=geo["cache_hits"], m=geo["cache_misses"],
        r=100.0 * geo["cache_hits"] / max(1, geo["cache_hits"] + geo["cache_misses"])))
    print("  说明：局部几何（顶点/面法向/棱方向）只由基本体参数决定，与该零件被"
          "摆在哪里无关——沿轨迹检查时同一零件会被算上千次。")

    # ---- 4) 口径说明 ----
    section("[4] 口径说明（这份基准不回答什么）")
    print("  · 耗时与机器负载强相关：数字用于**同一台机器上的横向比较**与回归预警，不是跑分。")
    print("  · 每次采样 ~0.17 ms 是把**整个场景**（连杆-环境 + 自碰撞）过一遍的成本；")
    print("    它与采样口径无关——口径只决定采几次，不决定每次多贵。")
    print("  · 默认口径按「总采样数约 300」取样：程序越长，采样点之间隔得越远（表里")
    print("    的「每采样路点」就是它），因此**检查的保证随规模变弱**。要给一份不随")
    print("    规模稀释的检查，传 max_joint_step（度的关节步长口径）。")
    print("  · 采样永远是采样：两条采样点之间的瞬时穿透仍可能被跳过，除非逐路点（0°）。")
    print("  · 包络容差（表里的干涉裕度）是厂商保守包络盒的建模误差补偿，不是安全余量。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
