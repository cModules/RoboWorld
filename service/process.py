"""焊接 / 喷涂工艺模板库（v0.3.3 剩余部分）。

工艺参数（电流、电压、送丝速度、保护气……）做成**可配置模板**：程序里只
引用模板名，换工艺改模板不改程序。内置一套保守默认模板，可用 YAML 覆盖
或扩展（支持 `` WeldProcess `` / `` SprayProcess `` 混存一个文件）。

用法::

    from service.process import WeldProcess, ProcessLibrary

    lib = ProcessLibrary.default()
    weld = lib.require("default_weld")

    # 换成现场工艺：改模板参数即可，程序不动
    weld.current, weld.voltage = 240.0, 25.5

    robot.moveL(start_pose).arc_start("my_weld").moveL(weld_pose).arc_end("my_weld")

YAML 格式::

    default_weld:
      kind: weld
      current: 240
      voltage: 25.5
      wire_feed: 9.0
    thin_plate:
      kind: weld
      current: 120
      voltage: 18.0
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Dict, Optional, Union

__all__ = [
    "WeldProcess",
    "SprayProcess",
    "ProcessLibrary",
    "resolve_weld",
    "resolve_spray",
]


@dataclass
class WeldProcess:
    """熔化极气体保护焊（MAG/MIG）工艺模板。

    Attributes:
        name: 模板名（程序里 ``arc_start("name")`` 引用）
        arc_start_signal: 起弧数字输出逻辑名
        arc_end_signal: 熄弧数字输出逻辑名
        arc_ok_signal: 起弧成功反馈数字输入逻辑名
        current: 焊接电流 (A)
        voltage: 焊接电压 (V)
        wire_feed: 送丝速度 (m/min)
        gas_flow: 保护气流量 (L/min)
        gas_preflow: 起弧前保护气预流时间 (s)
        gas_postflow: 熄弧后保护气滞流时间 (s)
        arc_ok_timeout: 等待起弧成功反馈的超时 (s)
    """

    name: str = "default_weld"
    arc_start_signal: str = "arc_start"
    arc_end_signal: str = "arc_end"
    arc_ok_signal: str = "arc_established"
    current: float = 220.0
    voltage: float = 24.0
    wire_feed: float = 8.0
    gas_flow: float = 15.0
    gas_preflow: float = 0.3
    gas_postflow: float = 1.0
    arc_ok_timeout: float = 3.0

    kind: str = "weld"  # YAML 反序列化判别字段

    def __post_init__(self) -> None:
        if self.current <= 0:
            raise ValueError(f"焊接电流必须 > 0，收到 {self.current}")
        if self.voltage <= 0:
            raise ValueError(f"焊接电压必须 > 0，收到 {self.voltage}")
        if self.arc_ok_timeout < 0:
            raise ValueError(f"起弧反馈超时不能为负，收到 {self.arc_ok_timeout}")

    def summary(self) -> str:
        """给后处理注释用的一行工艺参数摘要。"""
        return (
            f"焊接 电流{self.current:.0f}A 电压{self.voltage:.1f}V "
            f"送丝{self.wire_feed:.1f}m/min 保护气{self.gas_flow:.0f}L/min "
            f"预流{self.gas_preflow:.1f}s 后流{self.gas_postflow:.1f}s"
        )


@dataclass
class SprayProcess:
    """空气雾化喷涂工艺模板。

    Attributes:
        name: 模板名
        spray_on_signal: 喷涂开数字输出逻辑名
        spray_off_signal: 喷涂关数字输出逻辑名
        flow: 流量 (ml/min)
        fan_width: 扇幅 (mm)
        atomizing: 雾化压力 (bar)
    """

    name: str = "default_spray"
    spray_on_signal: str = "spray_on"
    spray_off_signal: str = "spray_off"
    flow: float = 300.0
    fan_width: float = 80.0
    atomizing: float = 2.5

    kind: str = "spray"

    def __post_init__(self) -> None:
        if self.flow <= 0:
            raise ValueError(f"喷涂流量必须 > 0，收到 {self.flow}")
        if self.atomizing <= 0:
            raise ValueError(f"雾化压力必须 > 0，收到 {self.atomizing}")

    def summary(self) -> str:
        return (
            f"喷涂 流量{self.flow:.0f}ml/min 扇幅{self.fan_width:.0f}mm "
            f"雾化{self.atomizing:.1f}bar"
        )


Process = Union[WeldProcess, SprayProcess]


def _from_dict(name: str, data: dict) -> Process:
    """按 ``kind`` 字段反序列化一条模板；缺 ``kind`` 按字段猜测。"""
    kind = data.get("kind")
    if kind is None:
        kind = "spray" if "spray_on_signal" in data or "flow" in data else "weld"
    data = {k: v for k, v in data.items() if k != "kind"}
    data["name"] = name
    if kind == "weld":
        valid = {f.name for f in fields(WeldProcess)}
        return WeldProcess(**{k: v for k, v in data.items() if k in valid})
    if kind == "spray":
        valid = {f.name for f in fields(SprayProcess)}
        return SprayProcess(**{k: v for k, v in data.items() if k in valid})
    raise ValueError(f"未知工艺类型 {kind!r}（支持 weld/spray）")


class ProcessLibrary:
    """工艺模板库：内置默认 + YAML 覆盖/扩展。"""

    def __init__(self, processes: Optional[Dict[str, Process]] = None):
        #: 模板名 → 模板实例
        self.processes: Dict[str, Process] = dict(processes or {})

    @classmethod
    def builtin(cls) -> "ProcessLibrary":
        """内置保守默认模板（现场参数请用 YAML 覆盖）。"""
        return cls(
            {
                "default_weld": WeldProcess(),
                "default_spray": SprayProcess(),
            }
        )

    _default: Optional["ProcessLibrary"] = None

    @classmethod
    def default(cls) -> "ProcessLibrary":
        """全局默认库（程序里 ``arc_start("name")`` 按名解析用）。"""
        if cls._default is None:
            cls._default = cls.builtin()
        return cls._default

    def require(self, name: str) -> Process:
        """按名取模板，不存在时抛带候选清单的 KeyError。"""
        if name not in self.processes:
            raise KeyError(f"工艺模板 {name!r} 不存在；可用：{', '.join(sorted(self.processes))}")
        return self.processes[name]

    def add(self, process: Process) -> None:
        """注册/覆盖一条模板。"""
        self.processes[process.name] = process

    def save_yaml(self, path: str | Path) -> None:
        """导出全部模板为 YAML。"""
        import yaml

        data = {}
        for p in self.processes.values():
            d = asdict(p)
            data[p.name] = d
        Path(path).write_text(
            yaml.safe_dump(data, allow_unicode=True, sort_keys=True), encoding="utf-8"
        )

    @classmethod
    def load_yaml(cls, path: str | Path) -> "ProcessLibrary":
        """从 YAML 加载模板，**叠加在内置默认之上**（同名覆盖）。"""
        import yaml

        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        lib = cls.builtin()
        for name, data in raw.items():
            lib.add(_from_dict(name, data or {}))
        return lib


def resolve_weld(process: Union[WeldProcess, str, None]) -> WeldProcess:
    """把 ``WeldProcess 实例 / 模板名 / None`` 归一成实例。"""
    if process is None:
        return WeldProcess()
    if isinstance(process, WeldProcess):
        return process
    if isinstance(process, str):
        p = ProcessLibrary.default().require(process)
        if not isinstance(p, WeldProcess):
            raise TypeError(f"模板 {process!r} 不是焊接工艺（是 {type(p).__name__}）")
        return p
    raise TypeError(f"无法解析焊接工艺: {process!r}")


def resolve_spray(process: Union[SprayProcess, str, None]) -> SprayProcess:
    if process is None:
        return SprayProcess()
    if isinstance(process, SprayProcess):
        return process
    if isinstance(process, str):
        p = ProcessLibrary.default().require(process)
        if not isinstance(p, SprayProcess):
            raise TypeError(f"模板 {process!r} 不是喷涂工艺（是 {type(p).__name__}）")
        return p
    raise TypeError(f"无法解析喷涂工艺: {process!r}")
