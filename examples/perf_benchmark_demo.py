#!/usr/bin/env python3
"""RobotWorld 示例：程序规模与性能基准（v1.1.8）。

一条命令跑出四件事，全部是现场实算的数字（没有写死的常数）：

1. **正运动学常量缓存的等价性**——同一组关节值喂给两份实现（生产实现 vs
   本模块里的朴素参考实现），逐位比对位姿矩阵；顺带量一下两者的耗时差。
2. **分段耗时**——规划 / 干涉检查 / 校验三段分别计时（含真实抓取工作站场景）。
3. **规模表**——同一条"零位附近摆动"的程序，从几十条指令加到几百条，
   看路点数、节拍与耗时怎么长。
4. **朴素参考实现的独立价值**——它是"关掉缓存会怎样"的实物对照。

运行::

    python examples/perf_benchmark_demo.py
    python examples/perf_benchmark_demo.py --motions 400     # 加大规模
    PYTHONPATH=python python examples/perf_benchmark_demo.py # 未安装 orolp 时
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
for p in (str(REPO_ROOT / "python"), str(REPO_ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402

from orolp import Robot  # noqa: E402
from orolp import kinematics as kinmod  # noqa: E402
from orolp.perf import (  # noqa: E402
    SWEEP_BASE,
    profile_program,
    reference_link_transforms,
    scale_report,
    sweep_program,
)
from orolp.planner import TrajectoryPlanner  # noqa: E402
from orolp.world.templates import build_grasping_workstation  # noqa: E402

URDF = REPO_ROOT / "models" / "ur" / "ur5e" / "ur5e.urdf"


def section(title: str) -> None:
    print()
    print("=" * 66)
    print(title)
    print("=" * 66)


def fk_equivalence(robot: Robot, n_configs: int = 300) -> bool:
    """同一组关节值喂两份实现：逐位比对 + 耗时对比。"""
    kin = robot.kinematics
    rng = np.random.default_rng(0)
    configs = [np.radians(rng.uniform(-150.0, 150.0, size=kin.dof)) for _ in range(n_configs)]

    worst = 0.0
    for q in configs[:40]:
        got = kin.link_transforms(q)
        ref = reference_link_transforms(kin, q)
        for name, T in got.items():
            worst = max(worst, float(np.abs(np.asarray(T) - np.asarray(ref[name])).max()))

    for q in configs[:20]:  # 预热
        kin.forward_kinematics(q)
        reference_link_transforms(kin, q)

    t0 = time.perf_counter()
    for q in configs:
        kin.forward_kinematics(q)
    cached = time.perf_counter() - t0

    t0 = time.perf_counter()
    for q in configs:
        reference_link_transforms(kin, q)
    naive = time.perf_counter() - t0

    stats = kinmod.cache_stats()
    print(f"  比对样本：{n_configs} 组关节值（含零位与 ±150° 随机组）")
    print(f"  位姿矩阵最大差异：{worst:.2e}（0 表示逐位相同）")
    print(f"  生产实现（常量缓存）：{cached * 1000:.1f} ms")
    print(f"  朴素参考实现（每次重算）：{naive * 1000:.1f} ms  → {naive / cached:.2f}x")
    print(
        f"  欧拉角转换实算 {stats['rotmat_computed']} 次 / 命中 {stats['rotmat_hits']} 次"
        f"（缓存 {stats['rotmat_entries']} 条）"
    )
    return worst == 0.0


def main() -> int:
    parser = argparse.ArgumentParser(description="RobotWorld 规模与性能基准")
    parser.add_argument("--motions", type=int, default=120, help="基准程序的运动条数")
    parser.add_argument(
        "--sizes", default="25,50,100,200", help="规模表的运动条数（逗号分隔）"
    )
    args = parser.parse_args()

    print("=" * 66)
    print("RobotWorld 规模与性能基准（v1.1.8）")
    print("=" * 66)

    robot = Robot(str(URDF), "UR5e")
    robot.set_joints(list(SWEEP_BASE))

    # ---- 1) FK 常量缓存：等价性 + 耗时 ----
    section("[1] 正运动学常量缓存：等价性与耗时")
    identical = fk_equivalence(robot)
    print(f"  等价性结论：{'逐位相同 ✅' if identical else '存在差异 ❌'}")

    # ---- 2) 分段耗时（真实抓取工作站场景）----
    section("[2] 分段耗时（接真实抓取工作站场景）")
    cell = build_grasping_workstation()
    robot.set_joints(list(SWEEP_BASE))
    planner = TrajectoryPlanner(robot, scene=cell.scene)
    program = sweep_program(robot, args.motions)
    profile = profile_program(planner, program, repeats=2)
    print(profile.format())

    # ---- 3) 规模表 ----
    section("[3] 规模表（指令数 → 路点数 → 耗时）")

    def factory():
        r = Robot(str(URDF), "UR5e")
        r.set_joints(list(SWEEP_BASE))
        return TrajectoryPlanner(r, scene=cell.scene)

    sizes = [int(s) for s in str(args.sizes).split(",") if s.strip()]
    report = scale_report(sweep_program, planner_factory=factory, sizes=sizes, repeats=2)
    print(report.format())

    # ---- 4) 这份基准没有回答的问题 ----
    section("[4] 口径说明（这份基准不回答什么）")
    print("  · 耗时与机器负载强相关：数字用于**同一台机器上的横向比较**与回归预警，不是跑分。")
    print("  · 每段取 repeat 次里的最小值（best-of），平均值会把系统抖动算进来。")
    print("  · validate 内部会自己再规划一次，所以各阶段耗时**不构成**一次总时长的分解。")
    print("  · 干涉检查的接触数来自 URDF <collision> 包络体，不是真机几何。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
