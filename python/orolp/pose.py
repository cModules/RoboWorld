"""Pose class for representing robot poses in 3D space."""

import numpy as np
from scipy.spatial.transform import Rotation


class Pose:
    """Represents a 3D pose (position + orientation)."""

    def __init__(self, matrix: np.ndarray | None = None):
        """
        Initialize a pose from a 4x4 transformation matrix.

        Args:
            matrix: 4x4 homogeneous transformation matrix. If None, identity matrix is used.
        """
        if matrix is None:
            self.matrix = np.eye(4)
        else:
            self.matrix = np.array(matrix, dtype=np.float64)
            if self.matrix.shape != (4, 4):
                raise ValueError("Transformation matrix must be 4x4")

    @classmethod
    def from_xyz_rpy(cls, x: float, y: float, z: float, rx: float, ry: float, rz: float) -> "Pose":
        """
        Create a pose from XYZ position and RPY angles (roll, pitch, yaw in degrees).

        Args:
            x, y, z: Position in millimeters
            rx, ry, rz: Roll, pitch, yaw in degrees (Euler ZYX order)

        Returns:
            Pose object
        """
        rot = Rotation.from_euler("ZYX", [np.radians(rz), np.radians(ry), np.radians(rx)])
        mat = np.eye(4)
        mat[:3, :3] = rot.as_matrix()
        mat[:3, 3] = [x, y, z]
        return cls(mat)

    @classmethod
    def from_xyz_quat(cls, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> "Pose":
        """
        Create a pose from XYZ position and quaternion orientation.

        Args:
            x, y, z: Position in millimeters
            qx, qy, qz, qw: Quaternion (x, y, z, w)

        Returns:
            Pose object
        """
        rot = Rotation.from_quat([qx, qy, qz, qw])
        mat = np.eye(4)
        mat[:3, :3] = rot.as_matrix()
        mat[:3, 3] = [x, y, z]
        return cls(mat)

    def to_xyz_rpy(self) -> tuple[float, float, float, float, float, float]:
        """
        Convert pose to XYZ position and RPY angles.

        Returns:
            Tuple (x, y, z, rx, ry, rz) where angles are in degrees
        """
        x, y, z = self.matrix[:3, 3]
        rot = Rotation.from_matrix(self.matrix[:3, :3])
        rz, ry, rx = rot.as_euler("ZYX", degrees=True)
        return (x, y, z, rx, ry, rz)

    def to_xyz_quat(self) -> tuple[float, float, float, float, float, float, float]:
        """
        Convert pose to XYZ position and quaternion orientation.

        Returns:
            Tuple (x, y, z, qx, qy, qz, qw)
        """
        x, y, z = self.matrix[:3, 3]
        rot = Rotation.from_matrix(self.matrix[:3, :3])
        qx, qy, qz, qw = rot.as_quat()
        return (x, y, z, qx, qy, qz, qw)

    def relative_to(self, base: "Pose") -> "Pose":
        """
        Compute this pose relative to a base pose.

        Args:
            base: Base pose to compute relative to

        Returns:
            Relative pose: base.inv() * this
        """
        return Pose(np.linalg.inv(base.matrix) @ self.matrix)

    def inv(self) -> "Pose":
        """
        Invert the pose.

        Returns:
            Inverted pose
        """
        inv_mat = np.linalg.inv(self.matrix)
        return Pose(inv_mat)

    def __mul__(self, other: "Pose") -> "Pose":
        """
        Multiply two poses (compose transformations).

        Args:
            other: Another pose

        Returns:
            Product pose: this * other
        """
        return Pose(self.matrix @ other.matrix)

    def copy(self) -> "Pose":
        """
        Create a copy of this pose.

        Returns:
            New Pose object with the same transformation matrix
        """
        return Pose(self.matrix.copy())

    def __repr__(self) -> str:
        x, y, z, rx, ry, rz = self.to_xyz_rpy()
        return f"Pose(x={x:.1f}, y={y:.1f}, z={z:.1f}, rx={rx:.1f}°, ry={ry:.1f}°, rz={rz:.1f}°)"

    def __getitem__(self, index: int) -> float:
        """
        Get element from transformation matrix.

        Args:
            index: Row-major index (0-15)

        Returns:
            Matrix element
        """
        return self.matrix[index // 4, index % 4]

    def __setitem__(self, index: int, value: float):
        """
        Set element in transformation matrix.

        Args:
            index: Row-major index (0-15)
            value: Value to set
        """
        self.matrix[index // 4, index % 4] = value

    def __array__(self) -> np.ndarray:
        """
        Convert to NumPy array.

        Returns:
            4x4 NumPy array
        """
        return self.matrix

    def with_z_offset(self, offset: float) -> "Pose":
        """
        Create a new pose with a Z offset.

        Args:
            offset: Z offset in millimeters

        Returns:
            New pose with offset applied
        """
        new_pose = self.copy()
        new_pose.matrix[2, 3] += offset
        return new_pose

    def with_position(self, x: float | None = None, y: float | None = None, z: float | None = None) -> "Pose":
        """
        Create a new pose with modified position.

        Args:
            x: New X position (None to keep current)
            y: New Y position (None to keep current)
            z: New Z position (None to keep current)

        Returns:
            New pose with position modified
        """
        new_pose = self.copy()
        if x is not None:
            new_pose.matrix[0, 3] = x
        if y is not None:
            new_pose.matrix[1, 3] = y
        if z is not None:
            new_pose.matrix[2, 3] = z
        return new_pose