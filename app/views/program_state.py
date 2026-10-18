"""离线编程面板的状态槽：每台机器人一份私有编程状态，外加回放定时器口径。

纯数据模块，不依赖 Qt。``RobotProgram`` 让面板按机器人名各自隔离示教点、
报告、轨迹等状态（多机不再互相顶掉）；``PLAYBACK_INTERVAL_MS`` 是回放帧率
对应的定时器间隔，面板初始化与回放推进（``program_playback``）共用这一处口径。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Dict, List, Optional

if TYPE_CHECKING:  # 仅类型标注
    from domain.robot import Robot
    from domain.world.animation import GraspAnimator
    from service.pickplace import PointType
    from service.planner import ProgramReport, TrajectoryPlanner
    from service.simulation import Trajectory

#: 回放帧率对应的定时器间隔 (ms)
PLAYBACK_INTERVAL_MS = 40


@dataclass
class RobotProgram:
    """一台机器人的**私有**离线编程状态。

    面板的所有既有方法（``build_program`` / ``validate_program`` /
    ``start_playback`` …）写的都是"当前机器人的数据"。以前这些数据直接是
    面板的字段，多机时第二台会把第一台顶掉——现在每台各占一个本类的实例，
    面板按 :attr:`ProgramPanel._active` 取用。
    """

    robot: Optional["Robot"] = None
    joint_names: List[str] = field(default_factory=list)
    taught: List[Dict[str, float]] = field(default_factory=list)
    point_types: Optional[List[PointType]] = None
    point_motions: List[str] = field(default_factory=list)
    build_notes: List[str] = field(default_factory=list)
    trajectory: Optional["Trajectory"] = None
    report: Optional[ProgramReport] = None
    planner: Optional[TrajectoryPlanner] = None
    gripper_animator: Optional[GraspAnimator] = None
    last_gripper_state: str = ""
    point_issue_level: Dict[int, str] = field(default_factory=dict)
    last_point_count: Optional[int] = None
    last_code: str = ""
    playhead: float = 0.0
