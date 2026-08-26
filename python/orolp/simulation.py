"""Simulation classes for robot trajectory simulation."""

from typing import List, Optional
import numpy as np
from .pose import Pose


class Waypoint:
    """Represents a single waypoint in a trajectory."""

    def __init__(
        self,
        joints: np.ndarray,
        pose: Pose,
        timestamp: float = 0.0,
        velocity: Optional[np.ndarray] = None,
        acceleration: Optional[np.ndarray] = None,
    ):
        """
        Initialize a waypoint.

        Args:
            joints: Joint positions
            pose: Tool pose
            timestamp: Timestamp in seconds
            velocity: Joint velocities (optional)
            acceleration: Joint accelerations (optional)
        """
        self.joints = np.array(joints, dtype=np.float64)
        self.pose = pose
        self.timestamp = timestamp
        self.velocity = np.array(velocity, dtype=np.float64) if velocity is not None else None
        self.acceleration = np.array(acceleration, dtype=np.float64) if acceleration is not None else None

    def __repr__(self) -> str:
        x, y, z, rx, ry, rz = self.pose.to_xyz_rpy()
        return f"Waypoint(t={self.timestamp:.2f}s, pose=({x:.1f}, {y:.1f}, {z:.1f}))"


class Trajectory:
    """Represents a robot trajectory consisting of waypoints."""

    def __init__(self, waypoints: List[Waypoint]):
        """
        Initialize a trajectory.

        Args:
            waypoints: List of waypoints
        """
        self.waypoints = waypoints

    def duration(self) -> float:
        """
        Get the total duration of the trajectory.

        Returns:
            Duration in seconds
        """
        if not self.waypoints:
            return 0.0
        return self.waypoints[-1].timestamp - self.waypoints[0].timestamp

    def num_waypoints(self) -> int:
        """
        Get the number of waypoints.

        Returns:
            Number of waypoints
        """
        return len(self.waypoints)

    def get_waypoint(self, index: int) -> Waypoint:
        """
        Get a waypoint by index.

        Args:
            index: Waypoint index

        Returns:
            Waypoint object
        """
        return self.waypoints[index]

    def get_joint_trajectory(self) -> np.ndarray:
        """
        Get the joint trajectory as a 2D array.

        Returns:
            Array of shape (num_waypoints, dof)
        """
        if not self.waypoints:
            return np.array([])
        return np.array([wp.joints for wp in self.waypoints])

    def get_pose_trajectory(self) -> List[Pose]:
        """
        Get the pose trajectory.

        Returns:
            List of Pose objects
        """
        return [wp.pose for wp in self.waypoints]

    def get_timestamps(self) -> np.ndarray:
        """
        Get all timestamps.

        Returns:
            Array of timestamps
        """
        return np.array([wp.timestamp for wp in self.waypoints])

    def interpolate(self, t: float) -> Waypoint:
        """
        Interpolate the trajectory at a given time.

        Args:
            t: Time in seconds

        Returns:
            Interpolated waypoint
        """
        if not self.waypoints:
            raise ValueError("Trajectory is empty")

        if t <= self.waypoints[0].timestamp:
            return self.waypoints[0]

        if t >= self.waypoints[-1].timestamp:
            return self.waypoints[-1]

        # Find surrounding waypoints
        for i in range(len(self.waypoints) - 1):
            t0 = self.waypoints[i].timestamp
            t1 = self.waypoints[i + 1].timestamp

            if t0 <= t <= t1:
                # Linear interpolation
                alpha = (t - t0) / (t1 - t0) if t1 > t0 else 0.0

                joints = (1 - alpha) * self.waypoints[i].joints + alpha * self.waypoints[i + 1].joints

                # Simple pose interpolation
                pose = Pose()  # Simplified for demo

                if self.waypoints[i].velocity is not None and self.waypoints[i + 1].velocity is not None:
                    velocity = (1 - alpha) * self.waypoints[i].velocity + alpha * self.waypoints[i + 1].velocity
                else:
                    velocity = None

                if self.waypoints[i].acceleration is not None and self.waypoints[i + 1].acceleration is not None:
                    acceleration = (1 - alpha) * self.waypoints[i].acceleration + alpha * self.waypoints[i + 1].acceleration
                else:
                    acceleration = None

                return Waypoint(joints, pose, t, velocity, acceleration)

        return self.waypoints[-1]

    def __repr__(self) -> str:
        return f"Trajectory({len(self.waypoints)} waypoints, duration={self.duration():.2f}s)"


class SimulationWorld:
    """Represents a simulation world with robots and objects."""

    def __init__(self):
        """Initialize an empty simulation world."""
        self.robots: dict = {}
        self.objects: dict = {}

    def add_robot(self, name: str, robot):
        """
        Add a robot to the world.

        Args:
            name: Robot name
            robot: Robot object
        """
        self.robots[name] = robot

    def add_object(self, name: str, pose: Pose):
        """
        Add an object to the world.

        Args:
            name: Object name
            pose: Object pose
        """
        self.objects[name] = pose

    def remove_robot(self, name: str):
        """
        Remove a robot from the world.

        Args:
            name: Robot name
        """
        if name in self.robots:
            del self.robots[name]

    def remove_object(self, name: str):
        """
        Remove an object from the world.

        Args:
            name: Object name
        """
        if name in self.objects:
            del self.objects[name]

    def check_collisions(self) -> bool:
        """
        Check for collisions in the world.

        Returns:
            True if any collisions detected
        """
        # Simplified for demo - always return False
        return False

    def __repr__(self) -> str:
        return f"SimulationWorld(robots={list(self.robots.keys())}, objects={list(self.objects.keys())})"