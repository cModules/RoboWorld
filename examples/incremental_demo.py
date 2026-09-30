#!/usr/bin/env python3
"""RobotWorld 示例：增量重算——改一处不必把整条轨迹重跑一遍（v1.2.0）。

一条命令跑通四件事，全部是现场实算的数字（没有写死的常数）：

1. **缓存账本**——同一份程序规划两次、改一条再规划一次，看两层缓存各命中了多少。
2. **分场景基准**——「改末尾 / 改中段 / 改首条 / 只改速度 / 插驻留 / 删一条」各自的
   耗时、加速倍数、分段复用率，并与**关缓存的实算结果**逐位比对（关节/位姿/时刻/速度）。
3. **两种目标写法的差别**——moveJ（终点是绝对关节值）与 moveL（终点是位姿、要经逆解）
   的复用率为什么差那么多，用同一份场景集量给你看。
4. **口径说明**——这份基准不回答什么。

运行::

    python examples/incremental_demo.py
    python examples/incremental_demo.py --motions 120
    PYTHONPATH=python python examples/incremental_demo.py   # 未安装 core 时
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
for p in (str(REPO_ROOT), str(REPO_ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)

from domain.robot import Robot  # noqa: E402
from support import i18n  # noqa: E402
from service.incremental import (
    TrajectoryCache,
    incremental_stats,
    reset_incremental_stats,
    set_cache,
)
from service.perf import (
    SWEEP_BASE,
    _clone_items,
    linear_program,
    plan_cache_profile,
    sweep_program,
)
from service.planner import TrajectoryPlanner  # noqa: E402
from service.program import Motion, Program, ProgramBuilder, WaitTime  # noqa: E402

URDF = REPO_ROOT / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"


def section(title: str) -> None:
    print()
    print("=" * 74)
    print(title)
    print("=" * 74)


def _robot() -> Robot:
    robot = Robot(str(URDF), "UR5e")
    robot.set_joints(list(SWEEP_BASE))
    return robot


def _planner() -> TrajectoryPlanner:
    return TrajectoryPlanner(_robot())


def _rebuild(items, robot) -> Program:
    return Program(robot, list(_clone_items(items, robot).items))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="增量重算：缓存账本与分场景基准")
    parser.add_argument("--motions", type=int, default=60, help="moveJ 基准程序的运动条数")
    parser.add_argument(
        "--cartesian", type=int, default=0, help="moveL 基准程序的运动条数（0 = 取 1/2）"
    )
    parser.add_argument("--repeat", type=int, default=1, help="每个场景重复次数（取最优）")
    args = parser.parse_args(argv)

    n_cartesian = args.cartesian or max(4, args.motions // 2)

    # ---- 1) 缓存账本 ----
    section("[1] 缓存账本：同一份程序规划两次，再改一条")
    robot = _robot()
    builder = ProgramBuilder(robot)
    base = list(SWEEP_BASE)
    for i in range(8):
        builder.moveJ([float(v) for v in (base[0] + i, *base[1:])], speed=75.0)
        if i == 3:
            builder.waitTime(0.3)
    items = list(builder.build().items)
    motions = [x for x in items if isinstance(x, Motion)]

    set_cache(TrajectoryCache())
    reset_incremental_stats()
    cold_start = time.perf_counter()
    _planner().plan(_rebuild(items, robot))  # 第 1 次：冷启动，两层都空
    cold = time.perf_counter() - cold_start
    print("  第 1 次（冷启动）            {:8.2f} ms   {}".format(
        cold * 1000.0, _fmt(incremental_stats())))

    reset_incremental_stats()
    warm_start = time.perf_counter()
    _planner().plan(_rebuild(items, robot))  # 第 2 次：整程序命中
    warm = time.perf_counter() - warm_start
    print("  第 2 次（整程序不变）        {:8.2f} ms   {}".format(
        warm * 1000.0, _fmt(incremental_stats())))
    print("  加速 {:.0f}x —— 整程序命中时连逐路点的正运动学都不再跑".format(
        cold / max(warm, 1e-9)))

    reset_incremental_stats()
    edited = list(_clone_items(items, robot).items)
    [x for x in edited if isinstance(x, Motion)][-1].speed = 33.0
    edit_start = time.perf_counter()
    _planner().plan(_rebuild(edited, robot))  # 第 3 次：只改末尾一条的速度
    edit = time.perf_counter() - edit_start
    print("  第 3 次（只改末尾速度）      {:8.2f} ms   {}".format(
        edit * 1000.0, _fmt(incremental_stats())))
    print("  说明：改速度不动关节值，所以**前缀与后缀全部照旧命中**，"
          "只有被改的那一条要重算。")

    # ---- 2) 分场景基准（moveJ）----
    section("[2] moveJ 程序：改动之后要重算多少")
    joint_profile = plan_cache_profile(
        _planner, lambda rb: sweep_program(rb, args.motions), repeats=max(1, args.repeat)
    )
    print(joint_profile.format())

    section("[3] moveL 程序：终点是位姿，扰动会沿链传下去")
    cart_profile = plan_cache_profile(
        _planner, lambda rb: linear_program(rb, n_cartesian), repeats=max(1, args.repeat)
    )
    print(cart_profile.format())

    section("[4] 两种目标写法的对照")
    print("  场景            moveJ 复用   moveL 复用   差别从哪来")
    print("  " + "-" * 68)
    for label, _fn in _scenarios():
        j = joint_profile.row(label)
        c = cart_profile.row(label)
        if j is None or c is None:
            continue
        print("  {label:<14}{j:>8.0%}      {c:>8.0%}     {note}".format(
            label=label, j=j.reuse_rate, c=c.reuse_rate, note=_note(label)))
    print()
    print("  · moveJ 的终点就是**绝对关节目标**，第 k 条的扰动在下一条指令处就被吸收，")
    print("    链条立刻回到原轨；moveL 的终点由**逆解**给出，而逆解以上一条的关节值为")
    print("    初值——扰动要么一直传下去，要么等逆解恰好回到同一分支才复位。")
    print("  · 所以『增量重算能省多少』取决于改哪儿，也取决于程序用什么目标写法。")
    print("    给一个笼统的百分比是没有意义的。")

    # ---- 5) 口径说明 ----
    section("[5] 口径说明（这份基准不回答什么）")
    print("  · 耗时与机器负载强相关：数字用于**同一台机器上的横向比较**与回归预警，不是跑分。")
    print("  · **首次规划不会更快**。冷启动仍要完整算一遍，收益全部来自第二次及以后。")
    print("  · 它只回答『轨迹展开要不要重算』：**干涉检查与动力学不在缓存范围内**，")
    print("    照常每次真算。")
    print("  · 它不能把『改前面一条的几何』变成免费——串行运动链里第 k 条的插补以第 k 条的")
    print("    起点为初值，位置改了终点就变，其后整条链的起点都变。能省的是三类：")
    print("    **前缀**、**不动关节值的改动（速度/驻留）**、**终点回到原处的改动**。")
    print("  · 等价列是**硬门槛**：与关缓存的实算结果逐位比对关节/位姿/时刻/速度，")
    print("    任何一项不为 0 都会直接显示出来，而不是悄悄给一个更快的错结果。")
    return 0


def _fmt(stats: dict) -> str:
    return "整程序 命中 {ph}/未命中 {pm} · 分段 命中 {sh}/未命中 {sm}".format(
        ph=stats.get("plan_hits", 0), pm=stats.get("plan_misses", 0),
        sh=stats.get("segment_hits", 0), sm=stats.get("segment_misses", 0),
    )


def _scenarios():
    from service.perf import default_edit_scenarios

    return default_edit_scenarios()


def _note(label: str) -> str:
    table = {
        i18n.tr("无改动"): "整程序命中，与目标写法无关",
        i18n.tr("改末尾运动"): "前缀 + 逆解初值都没变，两者都能全中",
        i18n.tr("改中段运动"): "关节目标在下一条被吸收；位姿目标要等逆解回同一分支",
        i18n.tr("改首条运动"): "位姿目标反而比改中段更友好：链条有更多机会复位",
        i18n.tr("只改速度"): "不动关节值，后缀照旧命中",
        i18n.tr("插入驻留"): "驻留不进分段层，运动部分全中",
        i18n.tr("删除一条"): "被删处之后要重新接链",
    }
    return table.get(label, "")


if __name__ == "__main__":
    raise SystemExit(main())
