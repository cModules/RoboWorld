"""PLC ↔ 机器人 IO 点表：视图、编辑、校验与出码联动。

现场返工最常见的原因不是算法，而是**点表对不上**：

* 机器人程序里写 ``SetDO("gripper_close")``，PLC 侧却把这个信号接在别的地方；
* 两个信号被塞进同一个物理地址——按一个键动两个机构；
* 同一信号被机器人 ``SetDO`` 与 PLC 输出**同时驱动**，谁后写谁生效；
* 机器人在等一个**没人给**的输入，现场就卡在那一行。

这些在各自一侧都"看起来没问题"，只有把两侧放到**同一张表**上才看得出来。
本模块把这件事做成可编辑、可校验、可往返的一等能力，并让 SCL 出码带上点表。

设计取舍（重要）：

* **唯一数据源仍是** :class:`~orolp.cell.CellConfig`——:class:`SignalMap`
  是它的视图 + 编辑器，不另存一份点表。``cell.py`` 的模块文档已经写明：
  硬件列表 / PLC 逻辑 / 机器人程序共用同一份模型，"各存各的必然对不上"。
* **方向是声明，不是推断结果**：方向取自 :data:`DEFAULT_DIRECTIONS` 或用户
  显式设置；程序实际怎么用（谁写谁读）单独记在 ``written``/``read`` 上。
  两者不一致正是要报出来的东西（见 ``double_drive``）。
* **地址格式只判"明显不像地址"**：只识别西门子/通用 IEC 风格写法，
  别的厂商写法原样接受（宁可不说，也不假装认识）。

用法::

    from orolp.cell import CellConfig, Device
    from orolp.iomap import SignalMap

    cell = CellConfig()
    cell.add(Device("夹具_1", "夹具", io_map={"gripper_close": "Q0.2"}))
    smap = SignalMap(cell)
    smap.auto_assign("夹具_1", ["gripper_open", "clamped"])
    report = smap.validate(program)
    print(report.format())
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .cell import HANDSHAKE_SIGNALS, CellConfig, Device
from .i18n import tr
from .planner import SEVERITY_ERROR, SEVERITY_WARNING

__all__ = [
    "DIRECTIONS",
    "DIRECTION_LABELS",
    "DEFAULT_DIRECTIONS",
    "PLC_OUTPUT_SIGNALS",
    "PLC_INPUT_SIGNALS",
    "ParsedAddress",
    "SignalUsage",
    "SignalPoint",
    "SignalIssue",
    "SignalReport",
    "SignalMap",
    "used_signals",
    "parse_address",
    "format_address",
    "address_problem",
    "validate_cell_io",
]


# --------------------------------------------------------------------------- #
# 方向与默认信号集合
# --------------------------------------------------------------------------- #

#: PLC 驱动、机器人/夹具采样（PLC → 机器人）
DIRECTION_PLC_TO_ROBOT = "plc_to_robot"
#: 机器人驱动、PLC 采样（机器人 → PLC）
DIRECTION_ROBOT_TO_PLC = "robot_to_plc"
#: 传感器/安全回路驱动，PLC 采样（传感器 → PLC）
DIRECTION_SENSOR_TO_PLC = "sensor_to_plc"

#: 全部方向
DIRECTIONS: Tuple[str, ...] = (
    DIRECTION_PLC_TO_ROBOT,
    DIRECTION_ROBOT_TO_PLC,
    DIRECTION_SENSOR_TO_PLC,
)

#: 方向的显示名（GUI / 报告 / 网页共用）
DIRECTION_LABELS: Dict[str, str] = {
    DIRECTION_PLC_TO_ROBOT: "PLC→机器人",
    DIRECTION_ROBOT_TO_PLC: "机器人→PLC",
    DIRECTION_SENSOR_TO_PLC: "传感器→PLC",
}


#: 握手信号与常见工艺信号的**默认方向**。
#: 名字尽量与 :data:`~orolp.cell.HANDSHAKE_SIGNALS` /
#: :data:`~orolp.cell.DEVICE_CATALOG` / :mod:`orolp.process` 的默认信号名一致。
DEFAULT_DIRECTIONS: Dict[str, str] = {
    # 握手（见 cell.HANDSHAKE_SIGNALS 的注释）
    "robot_start": DIRECTION_PLC_TO_ROBOT,
    "robot_done": DIRECTION_ROBOT_TO_PLC,
    "gripper_open": DIRECTION_PLC_TO_ROBOT,
    "gripper_close": DIRECTION_PLC_TO_ROBOT,
    "part_present": DIRECTION_SENSOR_TO_PLC,
    "safety_ok": DIRECTION_SENSOR_TO_PLC,
    # 机器人状态回馈（DEVICE_CATALOG 的「机器人」默认 IO）
    "error": DIRECTION_ROBOT_TO_PLC,
    "at_home": DIRECTION_ROBOT_TO_PLC,
    # 夹具与安全（DEVICE_CATALOG 的「夹具」/「安全」默认 IO）
    "clamped": DIRECTION_SENSOR_TO_PLC,
    "estop": DIRECTION_SENSOR_TO_PLC,
    # 工艺信号（process.WeldProcess / SprayProcess 的默认信号名）
    "arc_start": DIRECTION_ROBOT_TO_PLC,
    "arc_end": DIRECTION_ROBOT_TO_PLC,
    "arc_established": DIRECTION_SENSOR_TO_PLC,
    "spray_on": DIRECTION_ROBOT_TO_PLC,
    "spray_off": DIRECTION_ROBOT_TO_PLC,
}


#: PLC 逻辑**主动驱动**的信号（出码时声明为 ``VAR_OUTPUT``）。
#: 放在这里而不是 ``plc_scl`` 里，是为了让"谁驱动谁"只有一处定义——
#: ``plc_scl`` 反向引用本模块。
PLC_OUTPUT_SIGNALS: Tuple[str, ...] = ("robot_start", "gripper_open", "gripper_close")

#: PLC **采样**的信号（出码时声明为 ``VAR_INPUT``）；顺序即声明顺序。
PLC_INPUT_SIGNALS: Tuple[str, ...] = (
    "robot_done",
    "part_present",
    "safety_ok",
    "estop",
    "clamped",
    "at_home",
    "error",
)


# --------------------------------------------------------------------------- #
# 地址解析
# --------------------------------------------------------------------------- #

#: 识别范围刻意窄：西门子/通用 IEC 风格。``%`` 前缀（TIA 写法）会先剥掉。
_ADDRESS_RE = re.compile(
    r"^(?:(?P<db>DB\d+)\.)?"
    r"(?P<area>DBX|DBW|DBD|IB|QB|MB|IW|QW|MW|ID|QD|MD|I|Q|M|E|A|X|Y)"
    r"(?P<byte>\d+)(?:\.(?P<bit>[0-7]))?$",
    re.IGNORECASE,
)

#: 区域字母 → 归一化名。``E``/``A`` 是西门子的德文写法（E0.0 就是 I0.0），
#: 归并后判重才准。``X``/``Y``（见于三菱等）**刻意不归并**——那套编号是"位序号"
#: 而不是"字节.位"，硬当成同一套会造出**假重复**；判重宁可漏报也不误报。
_AREA_ALIASES: Dict[str, str] = {"E": "I", "A": "Q"}


@dataclass
class ParsedAddress:
    """解析出的物理地址。

    Attributes:
        raw: 原样地址
        area: 归一化区域（``I``/``Q``/``M``/``DBX``…）
        byte: 字节号
        bit: 位号（``None`` 表示整字节/整字访问）
        db: 数据块名（如 ``DB10``；非 DB 地址为空）
    """

    raw: str
    area: str
    byte: int
    bit: Optional[int] = None
    db: str = ""

    @property
    def key(self) -> str:
        """用于判重的归一化键（大小写与 ``%`` 前缀不敏感）。"""
        base = f"{self.db}.{self.area}" if self.db else self.area
        return f"{base}{self.byte}.{self.bit}" if self.bit is not None else f"{base}{self.byte}"


def parse_address(address: str) -> Optional[ParsedAddress]:
    """解析 IO 地址；不认识的写法返回 ``None``（不报错、不猜）。"""
    if not address:
        return None
    text = str(address).strip().lstrip("%")
    m = _ADDRESS_RE.match(text)
    if not m:
        return None
    area = m.group("area").upper()
    if area in _AREA_ALIASES:
        area = _AREA_ALIASES[area]
    return ParsedAddress(
        raw=str(address),
        area=area,
        byte=int(m.group("byte")),
        bit=int(m.group("bit")) if m.group("bit") is not None else None,
        db=(m.group("db") or "").upper(),
    )


def format_address(prefix: str, index: int) -> str:
    """把序号排成西门子风格的位地址：``Q0.0 … Q0.7, Q1.0 …``。

    Args:
        prefix: 区域字母（``Q`` 输出 / ``I`` 输入 / ``M`` 中间位）
        index: 位序号（从 0 计）
    """
    byte, bit = divmod(max(0, int(index)), 8)
    return f"{prefix}{byte}.{bit}"


def address_problem(address: str) -> Optional[str]:
    """地址**明显不像地址**时返回原因；不认识的厂商写法返回 ``None``。

    只在"根本不像地址"时出声（空、含空白/逗号、一个数字都没有）。带 DB 的
    复杂写法、厂商私有写法一律放过——本模块没有能力判断它们对不对，
    假装认识只会让用户不敢用。
    """
    text = str(address or "").strip()
    if not text:
        return tr("地址为空")
    if any(ch.isspace() for ch in text):
        return tr("地址里有空白字符")
    if "," in text or "，" in text:
        return tr("地址里有逗号——一行只能写一个地址")
    if not any(ch.isdigit() for ch in text):
        return tr("地址里没有任何数字")
    return None


# --------------------------------------------------------------------------- #
# 程序里的信号用途
# --------------------------------------------------------------------------- #


@dataclass
class SignalUsage:
    """机器人在程序里怎么用某个信号（由指令序列真实提取，不是猜的）。

    Attributes:
        written: 机器人是否写它（``SetDO`` / 工艺输出）
        read: 机器人是否读它（``WaitDI`` / 工艺反馈）
        commands: 出现过的指令类型名（便于定位）
    """

    written: bool = False
    read: bool = False
    commands: List[str] = field(default_factory=list)

    def merge(self, other: "SignalUsage") -> None:
        self.written = self.written or other.written
        self.read = self.read or other.read
        for name in other.commands:
            if name not in self.commands:
                self.commands.append(name)


def _process_signals(item) -> Tuple[List[str], List[str]]:
    """工艺指令 → ``(机器人写的信号, 机器人读的信号)``。

    起弧/熄弧/喷涂各自带输出信号；焊接还带一个"起弧成功"反馈输入。
    """
    written: List[str] = []
    read: List[str] = []
    proc = getattr(item, "process", None)
    if proc is None:
        return written, read
    for attr, target in (
        ("arc_start_signal", written),
        ("arc_end_signal", written),
        ("spray_on_signal", written),
        ("spray_off_signal", written),
        ("arc_ok_signal", read),
    ):
        name = getattr(proc, attr, "")
        if name and name not in target:
            target.append(str(name))
    return written, read


def used_signals(program) -> Dict[str, SignalUsage]:
    """从机器人程序里提取信号用途表。

    Args:
        program: :class:`~orolp.program.Program`，或直接传指令列表
            （duck typing，便于传入尚未包成 Program 的 items）

    Returns:
        ``{信号名: SignalUsage}``。运动、力控等指令不进表——它们不对话 PLC。
    """
    # 局部 import：program 是重模块，而本模块在 GUI 冷启动路径上
    from .program import ArcEnd, ArcStart, SetDO, SprayOff, SprayOn, WaitDI

    items = getattr(program, "items", program) or []
    table: Dict[str, SignalUsage] = {}

    def slot(signal: str) -> SignalUsage:
        return table.setdefault(str(signal), SignalUsage())

    for item in items:
        if isinstance(item, SetDO):
            usage = slot(item.signal)
            usage.written = True
            if "SetDO" not in usage.commands:
                usage.commands.append("SetDO")
        elif isinstance(item, WaitDI):
            usage = slot(item.signal)
            usage.read = True
            if "WaitDI" not in usage.commands:
                usage.commands.append("WaitDI")
        elif isinstance(item, (ArcStart, ArcEnd, SprayOn, SprayOff)):
            written, read = _process_signals(item)
            for name in written:
                usage = slot(name)
                usage.written = True
                if type(item).__name__ not in usage.commands:
                    usage.commands.append(type(item).__name__)
            for name in read:
                usage = slot(name)
                usage.read = True
                if type(item).__name__ not in usage.commands:
                    usage.commands.append(type(item).__name__)
    return table


# --------------------------------------------------------------------------- #
# 点表视图
# --------------------------------------------------------------------------- #


@dataclass
class SignalPoint:
    """点表里的一行（一个逻辑信号）。

    Attributes:
        signal: 逻辑信号名（机器人侧与 PLC 侧共用的那个名字）
        device: 负责这台设备的设备名；多台设备同时声明时取第一个
        address: 物理地址；未映射时为空串
        devices: 声明了该信号的全部设备名（>1 说明点表有歧义）
        direction: 声明方向（:data:`DIRECTIONS` 之一）；空 = 未声明
        written: 机器人程序是否写它
        read: 机器人程序是否读它
    """

    signal: str
    device: str = ""
    address: str = ""
    devices: List[str] = field(default_factory=list)
    direction: str = ""
    written: bool = False
    read: bool = False

    @property
    def mapped(self) -> bool:
        return bool(self.address)

    @property
    def used(self) -> bool:
        """机器人程序是否真的用到它。"""
        return self.written or self.read

    def usage_text(self) -> str:
        """用途的简短描述（报告/表格/网页共用）。"""
        if self.written and self.read:
            return tr("机器人读写")
        if self.written:
            return tr("机器人写")
        if self.read:
            return tr("机器人读")
        return tr("机器人未用")

    def direction_text(self) -> str:
        """方向的显示名。

        ``DIRECTION_LABELS`` 是**词表数据**（与 ``Device.type`` 同类），所以
        赋值处保持中文原样，只在显示处经 ``tr()`` 翻译——否则 en 界面会中英混排。
        """
        label = DIRECTION_LABELS.get(self.direction)
        return tr(label) if label else tr("未声明")

    def to_row(self) -> Dict[str, str]:
        """转成纯数据行（JSON/YAML/GUI 表格/网页共用同一形状）。"""
        return {
            "signal": self.signal,
            "device": self.device,
            "address": self.address,
            "direction": self.direction,
            "direction_text": self.direction_text(),
            "usage": self.usage_text(),
            "written": "1" if self.written else "",
            "read": "1" if self.read else "",
            "mapped": "1" if self.mapped else "",
        }


@dataclass
class SignalIssue:
    """点表校验发现的一条问题。

    Attributes:
        severity: ``"error"`` / ``"warning"``（与 planner 同一口径）
        code: 问题代码（稳定，供测试与统计用）
        message: 面向现场的中文说明
        signal: 相关信号名；与信号无关时为空
    """

    severity: str
    code: str
    message: str
    signal: str = ""

    def __str__(self) -> str:  # pragma: no cover - 仅调试用
        where = f"[{self.signal}] " if self.signal else ""
        return f"{where}{self.message}"


@dataclass
class SignalReport:
    """点表校验报告。"""

    issues: List[SignalIssue] = field(default_factory=list)
    points: List[SignalPoint] = field(default_factory=list)

    @property
    def errors(self) -> List[SignalIssue]:
        return [i for i in self.issues if i.severity == SEVERITY_ERROR]

    @property
    def warnings(self) -> List[SignalIssue]:
        return [i for i in self.issues if i.severity == SEVERITY_WARNING]

    @property
    def ok(self) -> bool:
        """是否无错误（警告不阻塞下装，但必须看见）。"""
        return not self.errors

    @property
    def mapped_count(self) -> int:
        return sum(1 for p in self.points if p.mapped)

    @property
    def unmapped_points(self) -> List[SignalPoint]:
        return [p for p in self.points if not p.mapped]

    def summary(self) -> str:
        """一行汇总（状态栏/日志用）。"""
        return tr("点表 {total} 点：已映射 {mapped} / 未映射 {unmapped}，{errors} 处错误 {warnings} 处警告").format(
            total=len(self.points),
            mapped=self.mapped_count,
            unmapped=len(self.unmapped_points),
            errors=len(self.errors),
            warnings=len(self.warnings),
        )

    def format(self) -> str:
        """多行可读报告。

        用 ``|`` 分隔而不是硬填充列宽：信号名是 ASCII 而方向/用途是中文，
        补空格在等宽编辑器里**看起来并不对齐**（中文双宽），假对齐不如不排。
        """
        lines = [self.summary(), ""]
        if not self.issues:
            lines.append(tr("  ✓ 点表未发现问题"))
        for issue in self.issues:
            mark = "✗" if issue.severity == SEVERITY_ERROR else "⚠"
            where = f"[{issue.signal}] " if issue.signal else ""
            lines.append(f"  {mark} {where}{issue.message}")
        lines.append("")
        lines.append(
            "  " + " | ".join((
                tr("信号"), tr("地址"), tr("方向"), tr("用途"), tr("归属"),
            ))
        )
        for p in self.points:
            lines.append(
                "  " + " | ".join((
                    p.signal,
                    p.address or "—",
                    p.direction_text(),
                    p.usage_text(),
                    p.device or "—",
                ))
            )
        return "\n".join(lines)

    def to_dict(self) -> Dict[str, object]:
        """纯数据形状（供 GUI 表格 / 网页 / 测试断言）。"""
        return {
            "ok": self.ok,
            "total": len(self.points),
            "mapped": self.mapped_count,
            "unmapped": len(self.unmapped_points),
            "errors": [
                {"code": i.code, "signal": i.signal, "message": i.message}
                for i in self.errors
            ],
            "warnings": [
                {"code": i.code, "signal": i.signal, "message": i.message}
                for i in self.warnings
            ],
            "points": [p.to_row() for p in self.points],
        }


# --------------------------------------------------------------------------- #
# 点表本身
# --------------------------------------------------------------------------- #


class SignalMap:
    """IO 点表：``CellConfig`` 的视图 + 编辑器。

    Args:
        cell: 工作单元配置（**唯一数据源**，本类不另存点表）
    """

    def __init__(self, cell: CellConfig) -> None:
        if cell is None:
            raise ValueError(tr("点表需要工作单元配置（CellConfig）"))
        self.cell = cell

    # ---------------- 视图 ---------------- #

    def _device_name(self, signal: str) -> List[str]:
        return [d.name for d in self.cell.devices if signal in d.io_map]

    def points(self, program=None, extra_signals: Sequence[str] = ()) -> List[SignalPoint]:
        """当前点表的全部行。

        行的来源有三个，合并去重：握手信号、各设备 ``io_map`` 里的键、
        （给出 ``program`` 时）机器人程序实际用到的信号。第三个必须并进来——
        程序里用了而点表里没有的信号，正是"没人发现的未映射"。
        """
        usage = used_signals(program) if program is not None else {}
        order: List[str] = []
        for sig in HANDSHAKE_SIGNALS:
            if sig not in order:
                order.append(sig)
        for d in self.cell.devices:
            for sig in d.io_map:
                if sig not in order:
                    order.append(sig)
        for sig in usage:
            if sig not in order:
                order.append(sig)
        for sig in extra_signals:
            if sig not in order:
                order.append(sig)

        result: List[SignalPoint] = []
        for sig in order:
            devices = self._device_name(sig)
            address = ""
            for d in self.cell.devices:
                if sig in d.io_map and d.io_map[sig]:
                    address = str(d.io_map[sig])
                    break
            u = usage.get(sig)
            result.append(
                SignalPoint(
                    signal=sig,
                    device=devices[0] if devices else "",
                    address=address,
                    devices=devices,
                    direction=self.direction_of(sig),
                    written=bool(u and u.written),
                    read=bool(u and u.read),
                )
            )
        return result

    def direction_of(self, signal: str) -> str:
        """信号方向：默认表 → 显式设置（存在 ``M`` 位无此概念，故只看默认表）。"""
        return DEFAULT_DIRECTIONS.get(str(signal), "")

    def address_of(self, signal: str) -> str:
        """信号地址；未映射返回空串。"""
        for d in self.cell.devices:
            addr = d.io_map.get(signal, "")
            if addr:
                return str(addr)
        return ""

    def to_rows(self, program=None, extra_signals: Sequence[str] = ()) -> List[Dict[str, str]]:
        """纯数据行列表（往返、GUI、网页共用）。"""
        return [p.to_row() for p in self.points(program, extra_signals)]

    # ---------------- 编辑（直接写回 cell） ---------------- #

    def assign(self, signal: str, address: str, device_name: Optional[str] = None) -> bool:
        """给信号写地址。

        Args:
            signal: 逻辑信号名
            address: 物理地址
            device_name: 归属设备；``None`` 时优先复用已声明该信号的设备，
                其次 PLC 设备，最后第一台设备

        Returns:
            是否写入成功（没有任何设备可挂时返回 ``False``）
        """
        device = self._pick_device(signal, device_name)
        if device is None:
            return False
        device.io_map[str(signal)] = str(address)
        return True

    def unassign(self, signal: str, device_name: Optional[str] = None) -> bool:
        """摘掉信号的地址（保留设备上的信号声明与否由 ``drop`` 决定）。"""
        targets = (
            [d for d in self.cell.devices if d.name == device_name]
            if device_name
            else self.cell.devices
        )
        hit = False
        for d in targets:
            if str(signal) in d.io_map:
                del d.io_map[str(signal)]
                hit = True
        return hit

    def _pick_device(self, signal: str, device_name: Optional[str]) -> Optional[Device]:
        if device_name is not None:
            return self.cell.find(device_name)
        owners = self._device_name(signal)
        if owners:
            return self.cell.find(owners[0])
        plc = self.cell.plc()
        if plc is not None:
            return plc
        return self.cell.devices[0] if self.cell.devices else None

    def auto_assign(
        self,
        device_name: Optional[str] = None,
        signals: Optional[Iterable[str]] = None,
        start: int = 0,
        output_prefix: str = "Q",
        input_prefix: str = "I",
        extra_signals: Sequence[str] = (),
        program=None,
    ) -> Dict[str, str]:
        """给未映射的信号自动编号（西门子风格位地址）。

        规则：

        * 传感器→PLC 方向、或机器人**读**的信号 → 用 ``input_prefix``；
          其余（PLC 驱动 / 机器人写）→ ``output_prefix``；
        * **跳过已被占用的地址**——包括别的信号已经占了的，避免自动编号
          把两个信号编到同一个点上（那正是本模块要抓的问题，不能自己造出来）；
        * 已经是映射状态的信号不动。

        Args:
            device_name: 全部写进这台设备；``None`` 时按信号各自归属
            signals: 要编号的信号；``None`` 时取当前点表里所有未映射的
            start: 输入/输出各自从第几位开始
            output_prefix / input_prefix: 区域字母
            extra_signals: 一并纳入点表的额外信号名
            program: 机器人程序；给出后"机器人只读"的信号按输入编号

        Returns:
            ``{信号名: 新地址}``（只含本次真正新分配的）
        """
        points = self.points(program, extra_signals)
        taken = set()
        for p in points:
            parsed = parse_address(p.address)
            if parsed is not None:
                taken.add(parsed.key)

        usage = used_signals(program) if program is not None else {}
        reads_only = {
            sig for sig, u in usage.items() if u.read and not u.written
        }

        wanted = (
            list(signals)
            if signals is not None
            else [p.signal for p in points if not p.mapped]
        )

        assigned: Dict[str, str] = {}
        counters = {output_prefix: start, input_prefix: start}
        for sig in wanted:
            if self.address_of(str(sig)):
                continue
            is_input = (
                self.direction_of(str(sig)) == DIRECTION_SENSOR_TO_PLC
                or str(sig) in reads_only
            )
            prefix = input_prefix if is_input else output_prefix
            index = counters[prefix]
            candidate = format_address(prefix, index)
            parsed = parse_address(candidate)
            while parsed is not None and parsed.key in taken:
                index += 1
                candidate = format_address(prefix, index)
                parsed = parse_address(candidate)
            counters[prefix] = index + 1
            if self.assign(str(sig), candidate, device_name):
                parsed_new = parse_address(candidate)
                taken.add(parsed_new.key if parsed_new is not None else candidate)
                assigned[str(sig)] = candidate
        return assigned

    def load_rows(
        self, rows: Sequence[Dict[str, str]], device_name: Optional[str] = None
    ) -> Dict[str, object]:
        """把纯数据行写回点表（往返的另一半）。

        只处理 ``address`` 字段：行里的设备名不存在时就退回 ``device_name``
        或自动归属。返回 ``{"applied", "unchanged", "skipped", "skipped_signals"}``
        ——**跳过哪些行必须报出来**：静默丢点是最难查的一类问题，而
        "文件里有、导入后没了"正是现场最容易踩的坑。三种结果的区别：

        * ``applied``：这次真的改动了点表
        * ``unchanged``：本来就没地址、也不在任何设备上——无事可做（不是失败）
        * ``skipped``：本该写进去却没写成功（设备名对不上等），必须人工看

        Args:
            rows: :meth:`to_rows` 产出的行（或同形状的手写数据）
            device_name: 兜底设备名
        """
        applied = 0
        unchanged: List[str] = []
        skipped: List[str] = []
        for row in rows:
            signal = str(row.get("signal") or "").strip()
            address = str(row.get("address") or "").strip()
            if not signal:
                skipped.append("")
                continue
            target = str(row.get("device") or "").strip() or device_name
            if address:
                ok = self.assign(signal, address, target)
            elif not self._device_name(signal):
                ok = False
                unchanged.append(signal)
            else:
                ok = self.unassign(signal, target or None)
            if ok:
                applied += 1
            elif signal not in unchanged:
                skipped.append(signal)
        return {
            "applied": applied,
            "unchanged": len(unchanged),
            "skipped": len(skipped),
            "skipped_signals": skipped,
        }

    # ---------------- 校验 ---------------- #

    def validate(self, program=None, steps=None, extra_signals: Sequence[str] = ()) -> SignalReport:
        """校验点表。

        Args:
            program: 机器人程序（或指令列表）；给出后检查"程序用了但没映射"
                与"两侧同时驱动"
            steps: PLC 工步（:class:`~orolp.plc_scl.Step` 序列，duck typing）；
                给出后其输出与等待也算作驱动/引用
            extra_signals: 一并纳入点表的额外信号名（如只在工步里出现的信号）

        Returns:
            :class:`SignalReport`
        """
        points = self.points(program, extra_signals)
        issues: List[SignalIssue] = []

        # PLC 侧与工步的驱动/等待
        plc_drives = set(PLC_OUTPUT_SIGNALS)
        plc_waits = set(PLC_INPUT_SIGNALS)
        for st in steps or ():
            for sig, value in getattr(st, "outputs", {}).items():
                (plc_drives if value else plc_waits).add(str(sig))
            wait = getattr(st, "wait", None)
            if wait:
                plc_waits.add(str(wait))

        # 1) 未映射：程序真在用的必须是 error（机器人下装后发不出/收不到）
        for p in points:
            if p.mapped:
                continue
            if p.used:
                issues.append(SignalIssue(
                    severity=SEVERITY_ERROR,
                    code="unmapped_signal",
                    signal=p.signal,
                    message=tr("机器人程序在用 {usage}，但没有任何设备给出物理地址——下装前必须补上").format(
                        usage=p.usage_text()
                    ),
                ))
            else:
                issues.append(SignalIssue(
                    severity=SEVERITY_WARNING,
                    code="unmapped_signal",
                    signal=p.signal,
                    message=tr("点表里有它但没有地址（下装前补上，或从点表里删掉）"),
                ))

        # 2) 地址重复：两个信号同一个物理点——现场按一个键动两个机构
        seen: Dict[str, SignalPoint] = {}
        for p in points:
            parsed = parse_address(p.address)
            if parsed is None:
                continue
            other = seen.get(parsed.key)
            if other is not None and other.signal != p.signal:
                issues.append(SignalIssue(
                    severity=SEVERITY_ERROR,
                    code="duplicate_address",
                    signal=p.signal,
                    message=tr("{addr} 已被 {other} 占用——两个信号不能共用一个物理地址").format(
                        addr=p.address, other=other.signal
                    ),
                ))
            else:
                seen[parsed.key] = p

        for p in points:
            # 3) 同一信号被多台设备声明：读代码的人不知道该信谁
            if len(p.devices) > 1:
                issues.append(SignalIssue(
                    severity=SEVERITY_WARNING,
                    code="ambiguous_owner",
                    signal=p.signal,
                    message=tr("{devices} 都声明了这个信号——归属必须唯一").format(
                        devices="、".join(p.devices)
                    ),
                ))

            # 4) 地址明显不像地址
            problem = address_problem(p.address) if p.mapped else None
            if problem is not None:
                issues.append(SignalIssue(
                    severity=SEVERITY_WARNING,
                    code="address_format",
                    signal=p.signal,
                    message=tr("{addr!r} 不像一个 IO 地址：{why}").format(addr=p.address, why=problem),
                ))

            # 5) 两侧同时驱动：谁后写谁生效，现场表现为"偶发动作"
            if p.written and p.signal in plc_drives:
                issues.append(SignalIssue(
                    severity=SEVERITY_WARNING,
                    code="double_drive",
                    signal=p.signal,
                    message=tr("机器人（SetDO）与 PLC 都在驱动它——必须明确主控方，否则谁后写谁生效"),
                ))

            # 6) 机器人等一个没人给的输入（已映射的前提下才说，避免与第 1 条重复）
            if p.mapped and p.read and not p.written:
                if p.signal not in plc_drives and self.direction_of(p.signal) != DIRECTION_SENSOR_TO_PLC:
                    issues.append(SignalIssue(
                        severity=SEVERITY_WARNING,
                        code="wait_without_source",
                        signal=p.signal,
                        message=tr("机器人在等它，但机器人自己不写、PLC 也没驱动它——现场会卡在这一步"),
                    ))

        return SignalReport(issues=issues, points=points)

    # ---------------- 往返 ---------------- #

    def to_json(self, program=None) -> str:
        """点表 → JSON（严格合法 JSON；``program`` 给出时带上用途列）。"""
        return json.dumps(
            {"version": 1, "points": self.to_rows(program)},
            ensure_ascii=False,
            indent=2,
        )

    @classmethod
    def from_json(cls, cell: CellConfig, text: str) -> Dict[str, object]:
        """JSON → 点表（写回 ``cell``）；返回 ``load_rows`` 的统计。"""
        data = json.loads(text)
        rows = data.get("points", data) if isinstance(data, dict) else data
        return cls(cell).load_rows(rows or [])

    def save_yaml(self, path: str | Path, program=None) -> None:
        """点表 → YAML（``allow_unicode``，中文信号名不被转义）。"""
        import yaml

        payload = {"version": 1, "points": self.to_rows(program)}
        Path(path).write_text(
            yaml.safe_dump(payload, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )

    @classmethod
    def load_yaml(cls, cell: CellConfig, path: str | Path) -> Dict[str, object]:
        """YAML → 点表（写回 ``cell``）。"""
        import yaml

        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        rows = raw.get("points", raw) if isinstance(raw, dict) else raw
        return cls(cell).load_rows(rows or [])


def validate_cell_io(
    cell: CellConfig,
    program=None,
    steps=None,
    extra_signals: Sequence[str] = (),
) -> SignalReport:
    """便捷入口：``SignalMap(cell).validate(program, steps, extra_signals)``。"""
    return SignalMap(cell).validate(program, steps, extra_signals)
