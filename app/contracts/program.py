"""离线编程面板与窗口之间的数据契约。"""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping


@dataclass(frozen=True)
class PlaybackFrame:
    """一帧回放数据，关节角统一使用弧度。"""

    robot_name: str
    joints_rad: Mapping[str, float]

    def __post_init__(self) -> None:
        object.__setattr__(self, "joints_rad", MappingProxyType(dict(self.joints_rad)))
