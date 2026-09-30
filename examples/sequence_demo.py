#!/usr/bin/env python3
"""
RobotWorld Example: 工艺时序（动态时序）一条命令跑通

演示 ``service.sequence`` 的完整使用路径——设备树侧的工序编排如何落到两个
既有出口，全程不新增任何指令语法：

  1. 建工作单元点表（CellConfig + Device，信号名复用 HANDSHAKE_SIGNALS）
  2. 编排工序序列（SignalStep / WaitStep / CallStep / DwellStep）
  3. 校验（signal_names / subprograms / devices 全部来自真实点表与设备清单）
  4. 机器人侧出口：to_program() → 既有 SetDO / WaitDI / WaitTime → UR 出码
  5. PLC 侧出口：to_plc_steps() → generate_scl() 出 SCL
  6. JSON 往返（save_json / load_json）

诚实性口径（与模块文档一致）：序列只有顺序，没有实测节拍——
驻留（秒）是唯一可计算的时间；子程序调用不是程序指令，
to_program() 会把它如实记进 notes，本示例把这些 notes 打印出来。

运行：
    PYTHONPATH=python python examples/sequence_demo.py
"""

from pathlib import Path

from domain.robot import Robot
from service.program import ProgramBuilder
from service.cell import CellConfig, Device, HANDSHAKE_SIGNALS
from service.plc_scl import generate_scl
from service.sequence import (
    CallStep,
    DwellStep,
    ProcessSequence,
    SignalStep,
    WaitStep,
)


def build_cell() -> CellConfig:
    """抓取工作单元的设备清单与点表（信号名全部复用握手词汇表）。"""
    cell = CellConfig()
    cell.add(Device(
        name="机器人", type="机器人", brand="Universal Robots", model="UR5e",
        comm="Socket", ip="192.168.1.10",
        # 控制柜侧视角：I=进（PLC/传感器 → 机器人），Q=出（机器人 → 外部）
        io_map={
            "robot_start": "I0.0", "part_present": "I0.1", "safety_ok": "I0.2",
            "gripper_close": "Q0.0", "gripper_open": "Q0.1", "robot_done": "Q0.2",
        },
    ))
    cell.add(Device(
        name="PLC", type="PLC", brand="Siemens", model="S7-1500",
        comm="Profinet", ip="192.168.1.1",
        io_map={sig: f"M10.{i}" for i, sig in enumerate(HANDSHAKE_SIGNALS)},
    ))
    cell.add(Device(
        name="夹具", type="夹具", brand="Schmalz", model="Gripper-X",
        comm="硬接线 IO",
        io_map={"gripper_close": "Q0.0", "gripper_open": "Q0.1"},
    ))
    cell.add(Device(
        name="传感器", type="传感器", brand="Keyence", model="PZ-G",
        comm="硬接线 IO",
        io_map={"part_present": "I0.0", "safety_ok": "I0.1"},
    ))
    return cell


def build_sequence() -> ProcessSequence:
    """抓取循环的工序编排（泳道 = 设备名）。

    刻意安排两处"真实世界的不完美"，让校验输出有内容可看：
    * 「等待启动循环」不给超时 → 无限等待警告（现场必须配看门狗）；
    * 「调用 PickPart」是子程序调用 → 机器人侧出口跳过并记 notes。
    """
    return ProcessSequence([
        WaitStep("等待启动循环", "PLC", "robot_start", trigger="rising", value=True),
        WaitStep("等待安全就绪", "PLC", "safety_ok", value=True, timeout=5.0),
        WaitStep("等待工件到位", "传感器", "part_present", trigger="rising", value=True, timeout=10.0),
        CallStep("调用抓取子程序", "机器人", "PickPart"),
        SignalStep("夹爪闭合", "机器人", "gripper_close", True),
        DwellStep("夹紧保压", "机器人", 0.5),
        SignalStep("夹爪松开", "机器人", "gripper_open", True),
        DwellStep("开爪确认", "机器人", 0.2),
        SignalStep("上报循环完成", "机器人", "robot_done", True),
    ])


def main() -> None:
    print("=" * 60)
    print("1. 工作单元点表")
    print("=" * 60)
    cell = build_cell()
    for d in cell.devices:
        print(f"  {d.name}（{d.type} · {d.brand} {d.model}）：")
        for sig, addr in d.io_map.items():
            print(f"    {sig:<14s} -> {addr}")

    print()
    print("=" * 60)
    print("2. 工序编排（四种工步）")
    print("=" * 60)
    seq = build_sequence()
    for i, st in enumerate(seq):
        kind = type(st).__name__
        lane = st.device
        detail = getattr(st, "signal", None) or getattr(st, "subprogram", None)
        extra = ""
        if isinstance(st, WaitStep):
            extra = f" trigger={st.trigger} timeout={st.timeout}s"
        elif isinstance(st, DwellStep):
            extra = f" {st.seconds}s"
        print(f"  {i + 1:>2}. [{kind:<10s}] {lane:<4s} | {st.name}"
              + (f" ({detail})" if detail else "") + extra)

    print()
    print("=" * 60)
    print("3. 校验（信号/子程序/设备全部来自真实点表与设备清单）")
    print("=" * 60)
    subprograms = ["PickPart", "PlacePart"]
    issues = seq.validate(
        signal_names=cell.all_signals(),
        subprograms=subprograms,
        devices=[d.name for d in cell.devices],
    )
    if issues:
        for issue in issues:
            at = f"工步 {issue.index + 1}" if issue.index is not None else "（结构）"
            print(f"  [{issue.severity}] {issue.code}: {issue.message}  @ {at}")
    else:
        print("  无问题")
    # 自证：故意用一个点表里不存在的信号，校验必须报 error
    bad = ProcessSequence([WaitStep("坏等待", "传感器", "not_in_table", timeout=1.0)])
    bad_issues = bad.validate(signal_names=cell.all_signals())
    assert any(i.severity == "error" and i.code == "unknown_signal" for i in bad_issues), \
        "未知信号必须报 error——校验不是摆设"
    print(f"  （自证）往序列里塞未知信号 not_in_table -> 报 {len(bad_issues)} 条 error，校验真实生效")

    print()
    print("=" * 60)
    print("4. 组成摘要（诚实口径：只有驻留是可计算的时间）")
    print("=" * 60)
    summary = seq.summary()
    print(f"  工步 {summary['steps']} 条 | 泳道 {' / '.join(summary['lanes'])}")
    print(f"  组成：{summary['counts']}")
    print(f"  驻留合计：{summary['dwell_total_s']}s")
    print(f"  {summary['note']}")

    print()
    print("=" * 60)
    print("5. 机器人侧出口：to_program() -> 既有指令 -> UR 出码")
    print("=" * 60)
    robot = Robot("examples/assets/test_6dof.urdf", name="SeqDemoRobot")
    builder = ProgramBuilder(robot)
    notes = seq.to_program(builder)
    program = builder.build()
    for note in notes:
        print(f"  [notes] {note}")
    # 点表接进后处理：io_map 走 metadata（与 grasping_workstation_demo 同一通路）
    io_map = cell.robot().io_map
    ur_code = program.to_post("ur_polyscope", metadata={"io_map": io_map})
    io_lines = [
        ln for ln in ur_code.splitlines()
        if "set_digital_out" in ln or "wait_digital_in" in ln or "TODO" in ln
    ]
    print(f"  UR 出码共 {len(ur_code.splitlines())} 行，其中 IO/等待行：")
    for ln in io_lines:
        print(f"    {ln.strip()}")

    print()
    print("=" * 60)
    print("6. PLC 侧出口：to_plc_steps() -> generate_scl() 出 SCL")
    print("=" * 60)
    scl = generate_scl(cell, program_name="SeqDemoCell", steps=seq.to_plc_steps())
    scl_lines = scl.splitlines()
    print(f"  SCL 共 {len(scl_lines)} 行，前 18 行：")
    for ln in scl_lines[:18]:
        print(f"    {ln}")

    print()
    print("=" * 60)
    print("7. JSON 往返")
    print("=" * 60)
    out_path = Path(__file__).parent / "assets" / "pick_sequence.json"
    seq.save_json(out_path)
    back = ProcessSequence.load_json(out_path)
    assert len(back) == len(seq), "往返后工步数必须一致"
    lanes_before, lanes_after = seq.lanes(), back.lanes()
    assert lanes_before == lanes_after, "往返后泳道顺序必须一致"
    print(f"  已写入 {out_path.relative_to(Path(__file__).parent.parent)}")
    print(f"  读回：{len(back)} 条工步，泳道 {' / '.join(lanes_after)} —— 与写入前一致")

    print()
    print("完成：序列 -> 校验 -> 机器人程序 / PLC SCL / JSON 三条出路全部走通。")


if __name__ == "__main__":
    main()
