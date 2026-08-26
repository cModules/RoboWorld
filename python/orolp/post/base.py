from abc import ABC, abstractmethod
from typing import List, Dict, Any
from dataclasses import dataclass
import jinja2
from .. import _core


@dataclass
class PostConfig:
    precision: int = 3
    use_joint_names: bool = False
    speed_unit: str = "%"
    io_format: str = "digital"
    comment_char: str = ";"
    indent: str = "    "


class BasePostProcessor(ABC):
    def __init__(self, config: PostConfig = None):
        self.config = config or PostConfig()
        self.env = jinja2.Environment(
            trim_blocks=True,
            lstrip_blocks=True,
            keep_trailing_newline=True,
        )

    @property
    @abstractmethod
    def brand(self) -> str:
        pass

    @property
    @abstractmethod
    def controller(self) -> str:
        pass

    @property
    def full_name(self) -> str:
        return f"{self.brand}_{self.controller}"

    def generate(self, waypoints: List[_core.Waypoint],
                 metadata: Dict[str, Any] = None) -> str:
        context = {
            "waypoints": [self._convert_wp(wp) for wp in waypoints],
            "meta": metadata or {},
            "config": self.config,
            "post": self,
        }
        template = self.env.from_string(self._get_template())
        return template.render(**context)

    @abstractmethod
    def _convert_wp(self, wp: _core.Waypoint) -> Dict[str, Any]:
        pass

    @abstractmethod
    def _get_template(self) -> str:
        pass

    def fmt_joints(self, q: List[float]) -> str:
        return ", ".join(f"{v:.{self.config.precision}f}" for v in q)

    def fmt_pose(self, pose_flat: List[float], order: str = "xyz") -> str:
        import numpy as np
        T = np.array(pose_flat).reshape(4, 4)
        x, y, z = T[:3, 3]
        R = T[:3, :3]
        if order == "xyz":
            ry = np.arcsin(R[2, 0])
            rx = np.arctan2(-R[2, 1], R[2, 2])
            rz = np.arctan2(-R[1, 0], R[0, 0])
        elif order == "zyx":
            ry = np.arcsin(-R[2, 0])
            rx = np.arctan2(R[2, 1], R[2, 2])
            rz = np.arctan2(R[1, 0], R[0, 0])
        else:
            raise ValueError(f"Unsupported order: {order}")
        rx, ry, rz = np.degrees([rx, ry, rz])
        return f"{x:.{self.config.precision}f}, {y:.{self.config.precision}f}, {z:.{self.config.precision}f}, {rx:.{self.config.precision}f}, {ry:.{self.config.precision}f}, {rz:.{self.config.precision}f}"

    def fmt_speed(self, speed: float, move_type: _core.MoveType) -> str:
        if self.config.speed_unit == "%":
            return f"{speed:.1f}"
        elif self.config.speed_unit == "mm/s":
            return f"v{int(speed)}"
        return str(speed)
