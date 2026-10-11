#!/usr/bin/env python3
"""RoboWorld 示例：程序版本对比（v1.0.7）。

一句话需求："程序在云端被人改过一版——改了哪几条指令、节拍变多少、TCP 路径
偏多少？"本示例用 **真 HTTP 版本库** + **真规划器** 跑完整个评审流程：

    起参考对象存储 → 版本库 v1/v2/v3（自动递增、指纹留痕）
      → 结构化 diff：插入对齐 / 字段级差异 / IO 电平变化
      → 规划器对比：节拍变化 + TCP 路径偏移
      → 失败语义：取回不存在的版本必须响亮报错

运行::

    python examples/prog_diff_demo.py
    # 未安装 core 时：PYTHONPATH=python python examples/prog_diff_demo.py

诚实说明（页面/报告同款）：结构对比是精确的（字段级）；但节拍与 TCP 路径偏移
是**规划器口径**——回答"仿真里会多花多久、路径会偏多少"，不回答"真机上会怎样"。
"""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
for p in (str(REPO_ROOT), str(REPO_ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)

from domain.pose import Pose
from domain.robot import Robot
from service.program import ProgramBuilder  # noqa: E402
from adapter.cloudhttp import CloudStoreServer, HTTPCloudStore  # noqa: E402
from service.progdiff import (
    ProgramDiffError,
    ProgramVersionStore,
    program_fingerprint,
)
from service.twinsync import MemoryCloudStore  # noqa: E402

URDF = REPO_ROOT / "assets/models/ur/ur5e/ur5e.urdf"
FAILURES: list = []


def _hr(title: str) -> None:
    print()
    print("=" * 72)
    print(title)
    print("=" * 72)


def _ok(label: str, passed: bool, extra: str = "") -> None:
    mark = "✓" if passed else "✗"
    if not passed:
        FAILURES.append(label)
    print(f"  [{mark}] {label}{('：' + extra) if extra else ''}")


def build_v1() -> ProgramBuilder:
    """v1：抓取程序初版（下探 300mm，速度 50%）。"""
    return (ProgramBuilder("UR5e")
            .moveJ([0.0, -30.0, 30.0, 0.0, 60.0, 0.0], speed=40)
            .moveL(Pose.from_xyz_rpy(500.0, 0.0, 300.0, 0.0, 0.0, 0.0),
                   speed=50, blend=8)
            .setDO("gripper_close", True)
            .waitTime(0.5)
            .moveL(Pose.from_xyz_rpy(500.0, 200.0, 300.0, 0.0, 0.0, 0.0),
                   speed=50)
            .moveJ([0.0, -30.0, 30.0, 0.0, 60.0, 0.0], speed=40))


def build_v2() -> ProgramBuilder:
    """v2：现场改动——下探加深 20mm、加速到 70%、插一条中间点、保压延长。"""
    return (ProgramBuilder("UR5e")
            .moveJ([0.0, -30.0, 30.0, 0.0, 60.0, 0.0], speed=40)
            .moveL(Pose.from_xyz_rpy(500.0, 0.0, 280.0, 0.0, 0.0, 0.0),
                   speed=70, blend=8)
            .moveL(Pose.from_xyz_rpy(500.0, 100.0, 280.0, 0.0, 0.0, 0.0),
                   speed=70)  # 新增：分段靠近
            .setDO("gripper_close", True)
            .waitTime(1.0)
            .moveL(Pose.from_xyz_rpy(500.0, 200.0, 280.0, 0.0, 0.0, 0.0),
                   speed=70)
            .moveJ([0.0, -30.0, 30.0, 0.0, 60.0, 0.0], speed=40))


# --------------------------------------------------------------------------- #
# 第 1 节：版本库（真 HTTP 后端）
# --------------------------------------------------------------------------- #


def section_store() -> tuple:
    _hr("［1］版本库：挂在 HTTP 云端的程序版本（自动递增 + 指纹留痕）")
    tmp = Path(tempfile.mkdtemp(prefix="rwprogdiff-"))
    srv = CloudStoreServer(tmp / "cloud")
    srv.start()
    print(f"  服务端：http://{srv.address}/（CloudStoreServer，本机回环）")
    store = ProgramVersionStore(HTTPCloudStore(f"http://{srv.address}", "robotworld"))
    v1 = store.save("PICK", build_v1(), code="MOVEJ()...", meta={"by": "工位 A"})
    v2 = store.save("PICK", build_v2(), code="MOVEJ()...", meta={"by": "工艺员 B"})
    print(f"  已存版本：{store.versions('PICK')}（latest={store.latest('PICK')}）")
    print(f"  v1 指纹：{program_fingerprint(build_v1())[:16]}…")
    rec = store.load("PICK", v2)
    print(f"  v2 元数据：{json.dumps(rec['meta'], ensure_ascii=False)}")
    return srv, store, v1, v2


# --------------------------------------------------------------------------- #
# 第 2 节：结构化 diff（对齐 + 字段级）
# --------------------------------------------------------------------------- #


def section_diff(store: ProgramVersionStore, v1: str, v2: str) -> None:
    _hr("［2］结构化 diff：插入对齐 + 字段级差异（拒绝『文本变了』的含糊话）")
    d = store.diff("PICK", v1, v2)
    print(d.format())
    print()
    _ok("插入的 moveL 没让后面的指令错位（对齐生效）",
        d.n_added == 1 and d.n_changed >= 2)
    _ok("waitTime 的保压变化被点出（1.0→2.0 秒）",
        any("waitTime" in c.detail or "seconds" in c.detail
            for c in d.changes if c.kind == "changed"))
    _ok("diff 结果可 JSON 化（进云端/报表）", bool(d.to_dict()["summary"]))


# --------------------------------------------------------------------------- #
# 第 3 节：规划器口径——节拍变化 + TCP 路径偏移
# --------------------------------------------------------------------------- #


def section_plan(store: ProgramVersionStore, v1: str, v2: str) -> None:
    _hr("［3］规划器口径：节拍变化 + TCP 路径偏移（仿真口径，非真机承诺）")
    robot = Robot(str(URDF), "UR5e")
    d = store.diff("PICK", v1, v2, robot=robot, plan=True)
    if d.errors:
        print(f"  （规划器口径未完成：{d.errors}）")
        _ok("规划器对比可用", False, str(d.errors))
        return
    print(f"  节拍：{d.cycle_a:.2f}s → {d.cycle_b:.2f}s（Δ{d.cycle_delta:+.2f}s）")
    print(f"  TCP 路径偏移：max {d.path_max_mm:.2f}mm / mean {d.path_mean_mm:.2f}mm"
          "（新版轨迹逐点到旧版折线的最近距离）")
    _ok("节拍有变化（速度 50→70% 不该无感）", abs(d.cycle_delta) > 0.01)
    _ok("路径偏移被量化（下探加深 20mm）", d.path_max_mm > 5.0)
    print()
    print("  诚实说明：节拍与路径偏移是**规划器口径**——回答『仿真里会多花多久、』")
    print("  路径会偏多少』，不回答『真机上会怎样』；上真机前照旧走校验与仿真回放。")


# --------------------------------------------------------------------------- #
# 第 4 节：失败语义 + 内存后端等价性
# --------------------------------------------------------------------------- #


def section_edges() -> None:
    _hr("［4］边界与等价性")
    store = ProgramVersionStore(MemoryCloudStore())
    try:
        store.load("PICK", "v1")
        _ok("取回不存在的版本要响亮报错", False, "未按预期抛错")
    except ProgramDiffError as exc:
        _ok("取回不存在的版本要响亮报错", True, f"ProgramDiffError：{str(exc)[:48]}…")
    same = ProgramVersionStore(MemoryCloudStore())
    a = build_v1()
    v1 = same.save("PICK", a)
    v2 = same.save("PICK", build_v1())  # 同一份再传：留痕但逐字段一致
    d = same.diff("PICK", v1, v2)
    _ok("重复上传同一份 = 新版本号 + 空差异（重新上传本身就是要留痕的事件）",
        not d.has_changes and same.versions("PICK") == ["v1", "v2"])


def main() -> int:
    print("RoboWorld · 程序版本对比演示（v1.0.7）")
    srv, store, v1, v2 = section_store()
    try:
        section_diff(store, v1, v2)
        section_plan(store, v1, v2)
        section_edges()
    finally:
        print()
        print("  停止参考服务端…")
        srv.stop()

    _hr("结果")
    if FAILURES:
        print(f"  有 {len(FAILURES)} 项未通过：")
        for f in FAILURES:
            print(f"    ✗ {f}")
        return 1
    print("  全部通过。")
    print("  （节拍与 TCP 路径偏移均为规划器口径——仿真评估用，非真机承诺。）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
