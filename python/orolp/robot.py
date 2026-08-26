"""Robot class for representing industrial robots."""

from typing import List, Optional, Dict, Any
import numpy as np
from .pose import Pose


class Robot:
    """Represents an industrial robot with URDF model."""

    def __init__(self, urdf_path: str, name: str = "Robot"):
        """
        Initialize a robot from a URDF file.

        Args:
            urdf_path: Path to URDF file
            name: Robot name
        """
        self.name = name
        self.urdf_path = urdf_path
        self._tool = Pose()
        self._base = Pose()
        self._joint_positions: np.ndarray | None = None
        self._dof = 6  # Default 6 DOF
        self._joint_names: List[str] = []

        # Load basic info from URDF if available
        self._parse_urdf_basic(urdf_path)

    def _parse_urdf_basic(self, urdf_path: str):
        """
        Parse basic info from URDF file.

        Args:
            urdf_path: Path to URDF file
        """
        try:
            import xml.etree.ElementTree as ET
            tree = ET.parse(urdf_path)
            root = tree.getroot()

            # Count joints
            self._joint_names = []
            for joint in root.findall("joint"):
                joint_type = joint.get("type", "")
                if joint_type in ["revolute", "prismatic"]:
                    joint_name = joint.get("name", f"joint{len(self._joint_names)}")
                    self._joint_names.append(joint_name)

            self._dof = len(self._joint_names)
            if self._dof == 0:
                self._dof = 6  # Fallback to default
                self._joint_names = [f"joint{i}" for i in range(6)]
        except Exception as e:
            print(f"Warning: Could not parse URDF fully: {e}")
            self._dof = 6
            self._joint_names = [f"joint{i}" for i in range(6)]

    @property
    def tool(self) -> Pose:
        """Get the tool (end-effector) pose."""
        return self._tool

    @tool.setter
    def tool(self, pose: Pose):
        """
        Set the tool (end-effector) pose.

        Args:
            pose: Tool pose relative to flange
        """
        self._tool = pose

    @property
    def base(self) -> Pose:
        """Get the base pose."""
        return self._base

    @base.setter
    def base(self, pose: Pose):
        """
        Set the base pose.

        Args:
            pose: Base pose relative to world
        """
        self._base = pose

    @property
    def dof(self) -> int:
        """Get the number of degrees of freedom."""
        return self._dof

    @property
    def joint_names(self) -> List[str]:
        """Get the list of joint names."""
        return self._joint_names.copy()

    def set_joints(self, joints: List[float] | np.ndarray):
        """
        Set the joint positions.

        Args:
            joints: Joint angles in degrees for revolute joints,
                    millimeters for prismatic joints
        """
        if len(joints) != self._dof:
            raise ValueError(f"Expected {self._dof} joints, got {len(joints)}")
        self._joint_positions = np.array(joints, dtype=np.float64)

    def get_joints(self) -> np.ndarray:
        """
        Get the current joint positions.

        Returns:
            Joint positions as NumPy array
        """
        if self._joint_positions is None:
            return np.zeros(self._dof)
        return self._joint_positions.copy()

    def forward_kinematics(self, joints: List[float] | np.ndarray | None = None) -> Pose:
        """
        Compute forward kinematics.

        Args:
            joints: Joint positions (optional, uses current if None)

        Returns:
            Tool pose in base frame
        """
        if joints is None:
            joints = self.get_joints()
        else:
            joints = np.array(joints)

        # Simplified FK for demo
        # In real implementation, this would use Pinocchio
        pose = Pose()

        # Just return current base * tool for now
        return self.base * self._tool

    def inverse_kinematics(self, target: Pose, guess: List[float] | np.ndarray | None = None) -> np.ndarray:
        """
        Compute inverse kinematics.

        Args:
            target: Target tool pose in base frame
            guess: Initial guess for solver (optional)

        Returns:
            Joint positions as NumPy array
        """
        # Simplified IK for demo
        # In real implementation, this would use Pinocchio
        if guess is not None:
            return np.array(guess)
        return np.zeros(self._dof)

    def moveJ(self, joints: List[float] | np.ndarray, speed: float = 100.0, blend: float = 0.0) -> "ProgramBuilder":
        """
        Create a joint space motion.

        Args:
            joints: Target joint positions
            speed: Speed percentage (0-100)
            blend: Blend radius in millimeters

        Returns:
            ProgramBuilder for chaining
        """
        from .program import ProgramBuilder
        return ProgramBuilder(self).moveJ(joints, speed, blend)

    def moveL(self, pose: Pose, speed: float = 100.0, blend: float = 0.0) -> "ProgramBuilder":
        """
        Create a linear motion in Cartesian space.

        Args:
            pose: Target tool pose
            speed: Speed percentage (0-100)
            blend: Blend radius in millimeters

        Returns:
            ProgramBuilder for chaining
        """
        from .program import ProgramBuilder
        return ProgramBuilder(self).moveL(pose, speed, blend)

    def moveC(self, via: Pose, target: Pose, speed: float = 100.0, blend: float = 0.0) -> "ProgramBuilder":
        """
        Create a circular motion.

        Args:
            via: Via point (midpoint)
            target: Target point
            speed: Speed percentage (0-100)
            blend: Blend radius in millimeters

        Returns:
            ProgramBuilder for chaining
        """
        from .program import ProgramBuilder
        return ProgramBuilder(self).moveC(via, target, speed, blend)

    def __repr__(self) -> str:
        return f"Robot(name='{self.name}', dof={self._dof})"