"""西门子 SCL（结构化文本）生成器。

只生成 **SCL**：纯文本、可程序化生成、TIA Portal 可直接导入源文件。
LAD / KOP / FBD 是图形化（且带二进制布局）格式，程序化生成不现实，不做。

生成物是一个 ``FUNCTION_BLOCK``，包含：

* 安全联锁（急停 / 安全未就绪时复位全部输出并回到步 0）
* 步序控制（``CASE step OF``），每步可置位输出并等待一个输入信号
* 每个信号带 IO 地址注释（取自 :class:`~service.cell.Device.io_map`）

.. warning::
   生成结果是**工程模板**，未经真机/TIA 验证。现场下装前必须核对 IO 地址
   与工艺步骤，不能直接当成品使用。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

from .cell import CellConfig, HANDSHAKE_SIGNALS
from .iomap import PLC_INPUT_SIGNALS, PLC_OUTPUT_SIGNALS, validate_cell_io
from .planner import SEVERITY_ERROR
from .program import Motion, SetDO, WaitDI, WaitTime
from .program import ArcStart, ArcEnd, SprayOn, SprayOff, ForceMode

__all__ = [
    "Step",
    "default_steps",
    "steps_from_program",
    "generate_scl",
    "SCLPostProcessor",
]


#: PLC 主动驱动的信号（输出）。
#: **定义在 :mod:`service.iomap`**——"谁驱动谁"只允许有一处定义，否则点表校验
#: 与出码会各说各话。这里保留私有旧名以兼容既有引用。
_OUTPUT_SIGNALS: Sequence[str] = PLC_OUTPUT_SIGNALS

#: PLC 采样的信号（输入）；顺序即声明顺序。
_INPUT_SIGNALS: Sequence[str] = PLC_INPUT_SIGNALS


@dataclass
class Step:
    """一个工步。

    Attributes:
        name: 步名（写进注释）
        outputs: 进入该步时置位的输出，如 ``{"gripper_close": True}``
        wait: 等待该输入满足条件才进入下一步；``None`` 表示不等待
        wait_value: ``wait`` 期望的电平（默认等待为真）
        delay: 进入下一步前的保压/延时（秒）。``0`` 表示不延时。
            需要它是因为夹紧、松开这类动作都有机械响应时间，PLC 必须等
            气压真正建立后再走下一步，否则会掉件
    """

    name: str
    outputs: Dict[str, bool] = field(default_factory=dict)
    wait: Optional[str] = None
    wait_value: bool = True
    delay: float = 0.0


def default_steps() -> List[Step]:
    """默认的「取料—搬运—放置」五步骨架。"""
    return [
        Step("等待工件到位", wait="part_present"),
        Step("夹紧工件", outputs={"gripper_close": True, "gripper_open": False}, wait="clamped"),
        Step("机器人搬运", outputs={"robot_start": True}, wait="robot_done"),
        Step(
            "松开工件",
            outputs={"gripper_open": True, "gripper_close": False, "robot_start": False},
            wait=None,
        ),
        Step("复位回原点", outputs={"gripper_open": False}, wait=None),
    ]


def steps_from_program(program) -> List[Step]:
    """把机器人程序的 IO 时序翻译成 PLC 工步。

    遍历指令序列，**只关心 IO 指令**——运动由机器人自己执行，不进 PLC 步序。
    规则：

    * ``SetDO``：累积进当前步的输出
    * ``WaitDI``：作为该步的等待条件，并收束这一步
    * ``WaitTime``：收束当前步，作为该步的保压延时（夹紧/松开的机械响应时间）

    为什么需要它：:func:`default_steps` 是写死的五步骨架，与实际示教出来的
    抓取动作无关。PLC 和机器人各说各话，现场联调时必然对不上。接入之后
    两边共用同一份 IO 定义。

    Args:
        program: :class:`~service.program.Program`，或直接传指令列表
            （duck typing，便于传入尚未包成 Program 的 items）

    Returns:
        工步列表。**没有任何 IO 指令时返回空列表**，由调用方回退到
        :func:`default_steps`——程序里没有抓取动作时，不该硬凑出工步来。
    """
    items = getattr(program, "items", program) or []

    steps: List[Step] = []
    pending: Dict[str, bool] = {}

    def flush(
        name: str, wait: Optional[str] = None, wait_value: bool = True, delay: float = 0.0
    ) -> None:
        steps.append(
            Step(name=name, outputs=dict(pending), wait=wait, wait_value=wait_value, delay=delay)
        )
        pending.clear()

    for item in items:
        if isinstance(item, SetDO):
            pending[item.signal] = item.value
        elif isinstance(item, WaitDI):
            flush(f"等待 {item.signal}", wait=item.signal, wait_value=item.value)
        elif isinstance(item, WaitTime):
            flush(f"保压 {item.seconds:.2f}s", delay=item.seconds)
        elif isinstance(item, (ArcStart, ArcEnd, SprayOn, SprayOff, ForceMode)):
            pass  # 工艺指令由机器人侧执行，不进 PLC 步序
        elif isinstance(item, Motion):
            continue  # 运动不进 PLC 步序

    if pending:
        flush("输出保持")

    if not steps:
        return []

    # 循环起始：工件到位才开工，否则会空抓
    return [Step("等待工件到位", wait="part_present")] + steps


def _format_pt(seconds: float) -> str:
    """秒 → S7 的 TIME 字面量（如 ``T#400MS``、``T#1S500MS``）。"""
    total_ms = int(round(seconds * 1000.0))
    if total_ms <= 0:
        return "T#0MS"
    s, ms = divmod(total_ms, 1000)
    if s and ms:
        return f"T#{s}S{ms}MS"
    if s:
        return f"T#{s}S"
    return f"T#{ms}MS"


def _addr_comment(cell: CellConfig, signal: str) -> str:
    """取信号的 IO 地址注释。

    信号在点表里**没有地址**时不再留空——留空在现场等于"看起来没问题"，
    而这个声明恰恰是下装前必须补的东西。直接写出来，让它在 TIA 里就看得见。
    """
    for dev in cell.devices:
        addr = dev.io_address(signal)
        if addr:
            return f"    // {dev.name}: {addr}"
    return "    // >>> 未映射（下装前必须补地址）"


def _point_table_lines(
    cell: CellConfig,
    program=None,
    steps: Optional[Sequence[Step]] = None,
) -> List[str]:
    """IO 点表注释块：逐点列地址/方向/用途/归属 + 校验摘要。

    点表必须**跟 SCL 一起**交给现场：TIA 能编译只说明语法没问题，不代表两侧
    点位对得上。未映射的点直接写在文件里，而不是只在某个界面里红一下。
    """
    extra: List[str] = []
    for st in steps or ():
        for sig in getattr(st, "outputs", {}):
            extra.append(str(sig))
        if getattr(st, "wait", None):
            extra.append(str(st.wait))

    report = validate_cell_io(cell, program, steps, extra_signals=extra)

    lines = [
        "(* ---------------- IO 点表（自动生成，下装前必须核对） ---------------- *)",
        f"(*   信号 | 地址 | 方向 | 用途 | 归属 *)",
    ]
    for p in report.points:
        lines.append(
            "(*   {sig} | {addr} | {dir} | {usage} | {dev} *)".format(
                sig=p.signal,
                addr=p.address or ">>> 未映射",
                dir=p.direction_text(),
                usage=p.usage_text(),
                dev=p.device or "—",
            )
        )
    lines.append("(*   {summary} *)".format(summary=report.summary()))
    for issue in report.issues:
        mark = "!!!" if issue.severity == SEVERITY_ERROR else "注意"
        lines.append(f"(*   {mark} [{issue.signal or '-'}] {issue.message} *)")
    lines.append("")
    return lines


def generate_scl(
    cell: CellConfig,
    program_name: str = "CellControl",
    steps: Optional[Sequence[Step]] = None,
    program=None,
    include_point_table: bool = True,
) -> str:
    """由工作单元配置生成 SCL 源码。

    Args:
        cell: 工作单元配置（决定出现哪些信号与地址注释）
        program_name: 生成的 FB 名
        steps: 工步序列；``None`` 时用 :func:`default_steps`
        program: 机器人程序（或指令列表）；给出后点表的"用途"列按它真实填写，
            并能报出"程序在用但没映射"的点
        include_point_table: 是否在文件头附 IO 点表注释块（默认附）

    Returns:
        SCL 源码文本
    """
    step_list = list(steps) if steps else default_steps()

    L: List[str] = []
    a = L.append

    a(f"(* ============================================================ *)")
    a(f"    RobotWorld 自动生成 —— 西门子 SCL 单元控制逻辑")
    a(f"    程序名 : {program_name}")
    a(f"    设备数 : {len(cell.devices)}")
    a(f"    生成于 : {time.strftime('%Y-%m-%d %H:%M:%S')}")
    a(f"")
    a(f"    警告：本文件是工程模板，未经真机/TIA 验证。")
    a(f"          下装前必须核对 IO 地址与工艺步骤。")
    a(f" ============================================================ *)")
    a("")
    if include_point_table:
        for line in _point_table_lines(cell, program=program, steps=step_list):
            a(line)
    a(f'FUNCTION_BLOCK "{program_name}"')
    a("{ S7_Optimized_Access := 'TRUE' }")
    a("VERSION : 0.1")
    a("")

    # 声明的信号 = 默认集合 ∪ 步序里实际用到的。
    # 只按默认集合声明的话，用户自定义信号会出现"用了没声明"的编译错误。
    out_sigs = list(_OUTPUT_SIGNALS)
    in_sigs = list(_INPUT_SIGNALS)
    has_delay = any(s.delay > 0 for s in step_list)
    for st in step_list:
        for sig in st.outputs:
            if sig not in out_sigs:
                out_sigs.append(sig)
        if st.wait and st.wait not in in_sigs:
            in_sigs.append(st.wait)

    # ---- VAR_INPUT ----
    a("VAR_INPUT")
    for sig in in_sigs:
        a(f"    {sig:<14} : BOOL;{_addr_comment(cell, sig)}")
    a("END_VAR")
    a("")

    # ---- VAR_OUTPUT ----
    a("VAR_OUTPUT")
    for sig in out_sigs:
        a(f"    {sig:<14} : BOOL;{_addr_comment(cell, sig)}")
    a("END_VAR")
    a("")

    # ---- VAR ----
    a("VAR")
    a("    step : INT := 0;")
    if has_delay:
        a("    dwellTimer : TON;  // 保压/延时定时器（夹紧、松开的机械响应时间）")
    a("END_VAR")
    a("")

    # ---- BEGIN ----
    a("BEGIN")
    a("    // ---------------- 安全联锁 ----------------")
    a("    // 急停或安全回路未就绪：复位全部输出并回到初始步")
    a("    IF NOT #safety_ok OR #estop THEN")
    for sig in out_sigs:
        a(f"        #{sig} := FALSE;")
    if has_delay:
        a("        #dwellTimer(IN := FALSE);  // 急停时定时器一并复位")
    a("        #step := 0;")
    a("        RETURN;")
    a("    END_IF;")
    a("")
    a("    // ---------------- 步序控制 ----------------")
    a("    CASE #step OF")

    for idx, st in enumerate(step_list):
        a(f"        {idx}:  // {st.name}")
        for sig, val in st.outputs.items():
            a(f"            #{sig} := {'TRUE' if val else 'FALSE'};")

        nxt = f"                #step := {idx + 1};"
        delay_body = [
            f"                #dwellTimer(IN := TRUE, PT := {_format_pt(st.delay)});",
            "                IF #dwellTimer.Q THEN",
            "                    #dwellTimer(IN := FALSE);",
            f"                    #step := {idx + 1};",
            "                END_IF;",
        ]

        if st.wait:
            cond = f"#{st.wait}" if st.wait_value else f"NOT #{st.wait}"
            a(f"            IF {cond} THEN")
            if st.delay > 0:
                for line in delay_body:
                    a(line)
            else:
                a(nxt)
            a("            END_IF;")
        elif st.delay > 0:
            a(f"            #dwellTimer(IN := TRUE, PT := {_format_pt(st.delay)});")
            a("            IF #dwellTimer.Q THEN")
            a("                #dwellTimer(IN := FALSE);")
            a(f"                #step := {idx + 1};")
            a("            END_IF;")
        else:
            a(f"            #step := {idx + 1};")
        a("")

    last = len(step_list)
    a(f"        {last}:  // 循环回到初始步")
    a("            #step := 0;")
    a("")
    a("    ELSE  // 未知步号：复位")
    a("        #step := 0;")
    a("    END_CASE;")
    a("")
    a("END_FUNCTION_BLOCK")

    return "\n".join(L) + "\n"


class SCLPostProcessor:
    """SCL 生成器，接口与 :mod:`core.post` 的后处理器保持一致。

    这样「设备联调」里若将来要按同一套接口枚举 PLC 目标，可以复用。
    """

    brand = "siemens"
    controller = "scl"

    @property
    def full_name(self) -> str:
        return f"{self.brand}_{self.controller}"

    @property
    def file_extension(self) -> str:
        return "scl"

    def generate(
        self,
        cell: CellConfig,
        metadata: Optional[Dict] = None,
        items: Optional[Sequence] = None,
        include_point_table: bool = True,
    ) -> str:
        """生成 SCL。

        Args:
            cell: 工作单元配置
            metadata: 附加信息（``program_name`` 等）
            items: 机器人程序指令序列；给出时**按其真实 IO 时序**生成步序，
                否则回退到 :func:`default_steps` 骨架
            include_point_table: 是否附 IO 点表注释块（默认附）
        """
        meta = metadata or {}
        steps = steps_from_program(items) if items else None
        return generate_scl(
            cell,
            program_name=meta.get("program_name", "CellControl"),
            steps=steps,
            program=items,
            include_point_table=include_point_table,
        )
