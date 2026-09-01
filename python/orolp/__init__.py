"""OpenRoboOLP - Open-source Offline Programming Software for Industrial Robots

A free, extensible alternative to commercial OLP tools like RoboDK.
Supports multi-brand robots via open URDF standards and community-driven
post-processors.
"""

__version__ = "0.1.0"

# Core C++ bindings (optional)
try:
    from . import orolp_core as _core
    _has_core = True
except ImportError:
    _has_core = False
    _core = None

# Python user-layer API
from .pose import Pose
from .robot import Robot
from .program import Program, Motion, ProgramBuilder, MotionType
from .simulation import SimulationWorld, Trajectory, Waypoint
from .planner import (
    AxisLimits,
    Issue,
    PlanningError,
    ProgramReport,
    TrajectoryPlanner,
)
from .link import ControllerLink, FileLink, SimLink, Deployment
from .debug import DebugSession, Deviation

__all__ = [
    "Robot",
    "Pose",
    "Program",
    "Motion",
    "ProgramBuilder",
    "MotionType",
    "SimulationWorld",
    "Trajectory",
    "Waypoint",
    # 离线编程：轨迹展开 + 节拍估算 + 程序校验
    "TrajectoryPlanner",
    "ProgramReport",
    "Issue",
    "AxisLimits",
    "PlanningError",
    # 离线编程最后一公里：代码上传 + 在线调试
    "ControllerLink",
    "FileLink",
    "SimLink",
    "Deployment",
    "DebugSession",
    "Deviation",
]
