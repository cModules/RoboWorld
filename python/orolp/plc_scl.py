"""西门子 SCL（结构化文本）生成器。

只生成 **SCL**：纯文本、可程序化生成、TIA Portal 可直接导入源文件。
LAD / KOP / FBD 是图形化（且带二进制布局）格式，程序化生成不现实，不做。

生成物是一个 ``FUNCTION_BLOCK``，包含：

* 安全联锁（急停 / 安全未就绪时复位全部输出并回到步 0）
* 步序控制（``CASE step OF``），每步可置位输出并等待一个输入信号
* 每个信号带 IO 地址注释（取自 :class:`~orolp.cell.Device.io_map`）

.. warning::
   生成结果是**工程模板**，未经真机/TIA 验证。现场下装前必须核对 IO 地址
   与工艺步骤，不能直接当成品使用。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

from .cell import CellConfig, HANDSHAKE_SIGNALS

__all__ = ["Step", "default_steps", "generate_scl", "SCLPostProcessor"]


#: PLC 主动驱动的信号（输出）
_OUTPUT_SIGNALS: Sequence[str] = ("robot_start", "gripper_open", "gripper_close")

#: PLC 采样的信号（输入）；顺序即声明顺序
_INPUT_SIGNALS: Sequence[str] = (
    "robot_done",
    "part_present",
    "safety_ok",
    "estop",
    "clamped",
    "at_home",
    "error",
)


@dataclass
class Step:
    """一个工步。

    Attributes:
        name: 步名（写进注释）
        outputs: 进入该步时置位的输出，如 ``{"gripper_close": True}``
        wait: 等待该输入为真才进入下一步；``None`` 表示立即进入下一步
    """

    name: str
    outputs: Dict[str, bool] = field(default_factory=dict)
    wait: Optional[str] = None


def default_steps() -> List[Step]:
    """默认的「取料—搬运—放置」五步骨架。"""
    return [
        Step("等待工件到位", wait="part_present"),
        Step("夹紧工件",
             outputs={"gripper_close": True, "gripper_open": False},
             wait="clamped"),
        Step("机器人搬运",
             outputs={"robot_start": True},
             wait="robot_done"),
        Step("松开工件",
             outputs={"gripper_open": True, "gripper_close": False,
                      "robot_start": False},
             wait=None),
        Step("复位回原点",
             outputs={"gripper_open": False},
             wait=None),
    ]


def _addr_comment(cell: CellConfig, signal: str) -> str:
    """取信号的 IO 地址注释；未映射时留空。"""
    for dev in cell.devices:
        addr = dev.io_address(signal)
        if addr:
            return f"    // {dev.name}: {addr}"
    return ""


def generate_scl(
    cell: CellConfig,
    program_name: str = "CellControl",
    steps: Optional[Sequence[Step]] = None,
) -> str:
    """由工作单元配置生成 SCL 源码。

    Args:
        cell: 工作单元配置（决定出现哪些信号与地址注释）
        program_name: 生成的 FB 名
        steps: 工步序列；``None`` 时用 :func:`default_steps`

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
    a(f'FUNCTION_BLOCK "{program_name}"')
    a("{ S7_Optimized_Access := 'TRUE' }")
    a("VERSION : 0.1")
    a("")

    # ---- VAR_INPUT ----
    a("VAR_INPUT")
    for sig in _INPUT_SIGNALS:
        a(f"    {sig:<14} : BOOL;{_addr_comment(cell, sig)}")
    a("END_VAR")
    a("")

    # ---- VAR_OUTPUT ----
    a("VAR_OUTPUT")
    for sig in _OUTPUT_SIGNALS:
        a(f"    {sig:<14} : BOOL;{_addr_comment(cell, sig)}")
    a("END_VAR")
    a("")

    # ---- VAR ----
    a("VAR")
    a("    step : INT := 0;")
    a("END_VAR")
    a("")

    # ---- BEGIN ----
    a("BEGIN")
    a("    // ---------------- 安全联锁 ----------------")
    a("    // 急停或安全回路未就绪：复位全部输出并回到初始步")
    a("    IF NOT #safety_ok OR #estop THEN")
    for sig in _OUTPUT_SIGNALS:
        a(f"        #{sig} := FALSE;")
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
        if st.wait:
            a(f"            IF #{st.wait} THEN")
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
    """SCL 生成器，接口与 :mod:`orolp.post` 的后处理器保持一致。

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

    def generate(self, cell: CellConfig, metadata: Optional[Dict] = None) -> str:
        meta = metadata or {}
        return generate_scl(cell, program_name=meta.get("program_name", "CellControl"))
