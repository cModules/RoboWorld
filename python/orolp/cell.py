"""工作单元（Cell）模型：设备清单与 IO 握手契约。

「代码生成」的三块产出——**硬件列表**、**PLC 程序逻辑**、**机器人子程序**——
共用这一份模型。三者若各存各的数据，必然对不上（IO 点表不一致是最常见的
现场返工原因），因此设备与信号只在这里定义一次。

典型的单元拓扑::

    传感器(part_present) ─┐
    夹具(open/close)     ─┼─ PLC ── 机器人(start/done)
    安全(safety_ok/estop)─┘
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

__all__ = [
    "HANDSHAKE_SIGNALS",
    "DEVICE_CATALOG",
    "DEVICE_TYPES",
    "Device",
    "CellConfig",
]


#: PLC 与机器人之间的握手信号——两者的**共同契约**。
#: PLC 程序逻辑与机器人子程序都必须按这套名字对接。
HANDSHAKE_SIGNALS: Tuple[str, ...] = (
    "robot_start",     # PLC → 机器人：启动循环
    "robot_done",      # 机器人 → PLC：动作完成
    "gripper_open",    # PLC → 夹具：松开
    "gripper_close",   # PLC → 夹具：夹紧
    "part_present",    # 传感器 → PLC：工件到位
    "safety_ok",       # 安全回路 → PLC：安全就绪
)


#: 设备类型目录：每类设备的默认 IO 点。
#: ``io`` 里的名字尽量复用 :data:`HANDSHAKE_SIGNALS`，保证与机器人侧同名。
DEVICE_CATALOG: Dict[str, Dict[str, object]] = {
    "机器人": {
        "brand": "",
        "io": ["robot_start", "robot_done", "error", "at_home"],
    },
    "PLC": {
        "brand": "Siemens",
        "io": [],
    },
    "夹具": {
        "brand": "",
        "io": ["gripper_open", "gripper_close", "clamped"],
    },
    "传感器": {
        "brand": "",
        "io": ["part_present"],
    },
    "安全": {
        "brand": "",
        "io": ["safety_ok", "estop"],
    },
}

#: 设备类型名（顺序即下拉框顺序）
DEVICE_TYPES: Tuple[str, ...] = tuple(DEVICE_CATALOG)


@dataclass
class Device:
    """工作单元里的一台设备。

    Attributes:
        name: 设备名（场景内唯一，如 ``机器人``、``夹具_1``）
        type: 设备类型，取 :data:`DEVICE_TYPES` 之一
        brand: 厂商（KUKA / ABB / Siemens / …）
        model: 型号
        comm: 通信方式（Profinet / EtherCAT / Modbus / 硬接线 IO）
        ip: 网络地址（Profinet/EtherCAT 设备用）
        io_map: 信号名 → 地址（如 ``{"robot_start": "Q0.0"}``）
    """

    name: str
    type: str = "传感器"
    brand: str = ""
    model: str = ""
    comm: str = ""
    ip: str = ""
    io_map: Dict[str, str] = field(default_factory=dict)

    def io_address(self, signal: str) -> str:
        """取信号地址；未映射时返回空串。"""
        return self.io_map.get(signal, "")

    def default_signals(self) -> List[str]:
        """该类型设备的默认信号列表。"""
        spec = DEVICE_CATALOG.get(self.type)
        return list(spec["io"]) if spec else []  # type: ignore[index]


@dataclass
class CellConfig:
    """工作单元配置：设备清单 + 步序。"""

    devices: List[Device] = field(default_factory=list)
    #: 步序：``{"name": ..., "action": ..., "wait": ...}``
    steps: List[Dict[str, str]] = field(default_factory=list)

    def add(self, device: Device) -> None:
        self.devices.append(device)

    def remove(self, name: str) -> None:
        self.devices = [d for d in self.devices if d.name != name]

    def find(self, name: str) -> Optional[Device]:
        for d in self.devices:
            if d.name == name:
                return d
        return None

    def by_type(self, type_: str) -> List[Device]:
        return [d for d in self.devices if d.type == type_]

    def robot(self) -> Optional[Device]:
        r = self.by_type("机器人")
        return r[0] if r else None

    def plc(self) -> Optional[Device]:
        p = self.by_type("PLC")
        return p[0] if p else None

    def all_signals(self) -> List[str]:
        """清单里出现过的全部信号名（握手信号 + 各设备已映射信号）。"""
        seen: List[str] = []
        for sig in HANDSHAKE_SIGNALS:
            seen.append(sig)
        for d in self.devices:
            for sig in d.io_map:
                if sig not in seen:
                    seen.append(sig)
        return seen
