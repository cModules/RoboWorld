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
]
