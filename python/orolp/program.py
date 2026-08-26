"""Program class for robot motion programming and post-processing."""

from typing import List, Optional, Dict, Any, Union
import numpy as np
from enum import Enum
from .pose import Pose
from .robot import Robot


class MotionType(Enum):
    """Type of robot motion."""
    JOINTS = "JOINTS"
    LINEAR = "LINEAR"
    CIRCULAR = "CIRCULAR"
    SPLINE = "SPLINE"


class Motion:
    """Represents a single robot motion."""

    def __init__(
        self,
        type: MotionType,
        target: Union[List[float], Pose],
        via: Optional[Union[List[float], Pose]] = None,
        speed: float = 100.0,
        blend: float = 0.0,
    ):
        """
        Initialize a motion.

        Args:
            type: Motion type
            target: Target (joints or pose)
            via: Via point (for circular motion)
            speed: Speed percentage (0-100)
            blend: Blend radius in millimeters
        """
        self.type = type
        self.target = target
        self.via = via
        self.speed = speed
        self.blend = blend


class Program:
    """Represents a complete robot program."""

    def __init__(self, robot: Robot, motions: List[Motion] | None = None):
        """
        Initialize a program.

        Args:
            robot: Robot this program is for
            motions: List of motions (optional)
        """
        self.robot = robot
        self.motions = motions.copy() if motions else []

    def simulate(self, collision_check: bool = False) -> "Trajectory":
        """
        Simulate the program.

        Args:
            collision_check: Whether to check for collisions (not implemented yet)

        Returns:
            Trajectory object
        """
        from .simulation import Trajectory, Waypoint
        waypoints: List[Waypoint] = []

        for i, motion in enumerate(self.motions):
            if motion.type == MotionType.JOINTS:
                target_joints = np.array(motion.target)
            else:
                target_joints = self.robot.inverse_kinematics(motion.target)

            waypoints.append(Waypoint(
                joints=target_joints,
                pose=self.robot.forward_kinematics(target_joints),
                timestamp=i * 1.0,  # Simplified time
            ))

        return Trajectory(waypoints)

    def to_post(self, post_processor: str = "ur_polyscope") -> str:
        """
        Generate robot code using post-processor.

        Args:
            post_processor: Name of post-processor to use

        Returns:
            Generated robot code as string
        """
        # Import post-processor
        try:
            if post_processor == "ur_polyscope":
                from .post.ur_script import UrScriptPostProcessor
                pp = UrScriptPostProcessor(self.robot)
            elif post_processor == "kuka_krc4":
                from .post.kuka_krc4 import KukaKrc4PostProcessor
                pp = KukaKrc4PostProcessor(self.robot)
            elif post_processor == "abb_rapid":
                from .post.abb_rapid import AbbRapidPostProcessor
                pp = AbbRapidPostProcessor(self.robot)
            elif post_processor == "fanuc_tp":
                from .post.fanuc_tp import FanucTpPostProcessor
                pp = FanucTpPostProcessor(self.robot)
            elif post_processor == "huichuan_inoteach":
                from .post.huichuan_inoteach import HuichuanInoteachPostProcessor
                pp = HuichuanInoteachPostProcessor(self.robot)
            else:
                raise ValueError(f"Unknown post-processor: {post_processor}")

            return pp.process(self.motions)
        except Exception as e:
            print(f"Warning: Post-processor error: {e}")
            return self._fallback_post_processing()

    def _fallback_post_processing(self) -> str:
        """
        Simple fallback post-processing.

        Returns:
            Generated code as string
        """
        lines = []
        lines.append(f"; Program for {self.robot.name}")
        lines.append("")

        for i, motion in enumerate(self.motions):
            line = f"; Motion {i+1}: {motion.type.value}"
            if motion.type == MotionType.JOINTS:
                line += f" - joints: {[f'{x:.2f}' for x in motion.target]}"
            else:
                x, y, z, rx, ry, rz = motion.target.to_xyz_rpy()
                line += f" - pose: ({x:.1f}, {y:.1f}, {z:.1f}, {rx:.1f}°, {ry:.1f}°, {rz:.1f}°)"
            line += f", speed: {motion.speed:.1f}%"
            lines.append(line)

        return "\n".join(lines)


class ProgramBuilder:
    """Builder for creating robot programs with chainable API."""

    def __init__(self, robot: Robot):
        """
        Initialize a builder.

        Args:
            robot: Robot this program is for
        """
        self.robot = robot
        self.motions: List[Motion] = []

    def moveJ(
        self,
        joints: List[float] | np.ndarray,
        speed: float = 100.0,
        blend: float = 0.0,
    ) -> "ProgramBuilder":
        """
        Add a joint space motion.

        Args:
            joints: Target joint positions
            speed: Speed percentage (0-100)
            blend: Blend radius in millimeters

        Returns:
            This builder for chaining
        """
        if isinstance(joints, np.ndarray):
            joints = joints.tolist()
        self.motions.append(Motion(MotionType.JOINTS, joints, speed=speed, blend=blend))
        return self

    def moveL(
        self,
        pose: Pose,
        speed: float = 100.0,
        blend: float = 0.0,
    ) -> "ProgramBuilder":
        """
        Add a linear motion in Cartesian space.

        Args:
            pose: Target tool pose
            speed: Speed percentage (0-100)
            blend: Blend radius in millimeters

        Returns:
            This builder for chaining
        """
        self.motions.append(Motion(MotionType.LINEAR, pose, speed=speed, blend=blend))
        return self

    def moveC(
        self,
        via: Pose,
        target: Pose,
        speed: float = 100.0,
        blend: float = 0.0,
    ) -> "ProgramBuilder":
        """
        Add a circular motion.

        Args:
            via: Via point (midpoint)
            target: Target point
            speed: Speed percentage (0-100)
            blend: Blend radius in millimeters

        Returns:
            This builder for chaining
        """
        self.motions.append(Motion(MotionType.CIRCULAR, target, via=via, speed=speed, blend=blend))
        return self

    def then(self, other: Union["ProgramBuilder", "Program"]) -> "ProgramBuilder":
        """
        Append another program or builder.

        Args:
            other: Program or builder to append

        Returns:
            This builder for chaining
        """
        if isinstance(other, ProgramBuilder):
            self.motions.extend(other.motions)
        elif isinstance(other, Program):
            self.motions.extend(other.motions)
        return self

    def build(self) -> Program:
        """
        Build the program.

        Returns:
            Program object
        """
        return Program(self.robot, self.motions)

    def simulate(self, collision_check: bool = False) -> "Trajectory":
        """
        Simulate the program.

        Args:
            collision_check: Whether to check for collisions (not implemented yet)

        Returns:
            Trajectory object
        """
        return self.build().simulate()

    def to_post(self, post_processor: str = "ur_polyscope") -> str:
        """
        Generate robot code.

        Args:
            post_processor: Name of post-processor

        Returns:
            Generated code as string
        """
        return self.build().to_post(post_processor)

    def __call__(self) -> Program:
        """
        Build the program when builder is called.

        Returns:
            Program object
        """
        return self.build()