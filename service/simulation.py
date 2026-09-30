"""Simulation classes for robot trajectory simulation."""

from pathlib import Path
from typing import TYPE_CHECKING, List, Optional

import numpy as np

from domain.pose import Pose
from domain.world.components import COMPONENTS
from domain.world.scene import Scene

if TYPE_CHECKING:  # 仅用于类型标注，避免运行时循环导入
    from domain.world.scene import SceneObject


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
        self.acceleration = (
            np.array(acceleration, dtype=np.float64) if acceleration is not None else None
        )

    def __repr__(self) -> str:
        x, y, z, rx, ry, rz = self.pose.to_xyz_rpy()
        return f"Waypoint(t={self.timestamp:.2f}s, pose=({x:.1f}, {y:.1f}, {z:.1f}))"


def _interpolate_pose(a: Pose, b: Pose, alpha: float) -> Pose:
    """在两个位姿之间插值。

    位置用线性插值，姿态用四元数球面插值（slerp）——直接对欧拉角做线性
    插值会在大角度旋转时走出错误的路径，甚至经过奇异位形。

    Args:
        a: 起始位姿
        b: 结束位姿
        alpha: 插值参数，0 对应 ``a``、1 对应 ``b``

    Returns:
        插值得到的位姿
    """
    from scipy.spatial.transform import Rotation, Slerp

    xa, ya, za, qax, qay, qaz, qaw = a.to_xyz_quat()
    xb, yb, zb, qbx, qby, qbz, qbw = b.to_xyz_quat()

    pos = np.array([xa, ya, za]) * (1.0 - alpha) + np.array([xb, yb, zb]) * alpha

    # scipy 的对外四元数顺序是 (x, y, z, w)，与 Pose 一致
    key = Rotation.from_quat([[qax, qay, qaz, qaw], [qbx, qby, qbz, qbw]])
    qx, qy, qz, qw = Slerp([0.0, 1.0], key)(alpha).as_quat()
    return Pose.from_xyz_quat(float(pos[0]), float(pos[1]), float(pos[2]), qx, qy, qz, qw)


class Trajectory:
    """Represents a robot trajectory consisting of waypoints."""

    def __init__(self, waypoints: List[Waypoint], collisions: Optional[list] = None):
        """
        Initialize a trajectory.

        Args:
            waypoints: List of waypoints
            collisions: 沿轨迹检出的干涉（:class:`~domain.world.scene.Collision`
                列表）。由 :meth:`service.planner.TrajectoryPlanner.validate` 填充，
                未做干涉检查时为空列表
        """
        self.waypoints = waypoints
        self.collisions = list(collisions) if collisions else []

    def duration(self) -> float:
        """
        Get the total duration of the trajectory.

        Returns:
            Duration in seconds
        """
        if not self.waypoints:
            return 0.0
        return self.waypoints[-1].timestamp - self.waypoints[0].timestamp

    def cycle_time(self) -> float:
        """节拍时间（秒），即 :meth:`duration` 的别名。

        叫 ``cycle_time`` 是因为节拍估算在 OLP 里是个独立卖点——客户拿它报价。
        """
        return self.duration()

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

                joints = (1 - alpha) * self.waypoints[i].joints + alpha * self.waypoints[
                    i + 1
                ].joints

                # 位姿插补：位置线性插值，姿态用四元数球面插值（slerp）。
                # 历史实现这里直接 ``Pose()`` 返回单位阵——轨迹上任意中间时刻
                # 都被报告成"位于原点、零姿态"，属于静默给出错误结果。
                pose = _interpolate_pose(self.waypoints[i].pose, self.waypoints[i + 1].pose, alpha)

                if (
                    self.waypoints[i].velocity is not None
                    and self.waypoints[i + 1].velocity is not None
                ):
                    velocity = (1 - alpha) * self.waypoints[i].velocity + alpha * self.waypoints[
                        i + 1
                    ].velocity
                else:
                    velocity = None

                if (
                    self.waypoints[i].acceleration is not None
                    and self.waypoints[i + 1].acceleration is not None
                ):
                    acceleration = (1 - alpha) * self.waypoints[
                        i
                    ].acceleration + alpha * self.waypoints[i + 1].acceleration
                else:
                    acceleration = None

                return Waypoint(joints, pose, t, velocity, acceleration)

        return self.waypoints[-1]

    def __repr__(self) -> str:
        return f"Trajectory({len(self.waypoints)} waypoints, duration={self.duration():.2f}s)"


class SimulationWorld(Scene):
    """带机器人与轨迹的仿真世界。
    历史上这个类是个空壳——:meth:`check_collisions` 直接 ``return False``，
    :meth:`add_object` 只存一个 :class:`Pose`、根本没有几何体，等于"仿真软件
    永远告诉你没撞"。现已改为继承 :class:`~domain.world.scene.Scene`，复用
    参数化组件库与 SAT 碰撞检测，具备真实的环境建模与干涉检查能力。

    保留本类（而不是直接改用 ``Scene``）是为了兼容旧的 API 名称
    （``add_robot`` / ``remove_robot`` / ``add_object``）。
    """

    def add_robot(self, name: str, robot) -> None:
        """登记一个机器人。

        Args:
            name: 机器人名
            robot: Robot 实例或 URDF 路径

        Raises:
            ValueError: robot 既不是路径也不是带 urdf_path 的对象
        """
        path = robot if isinstance(robot, (str, Path)) else getattr(robot, "urdf_path", None)
        if not path:
            raise ValueError("robot 必须是 URDF 路径，或带 urdf_path 属性的对象")
        self.set_robot(str(path))

    def remove_robot(self, name: str) -> None:
        """移除机器人（保留场景对象）。"""
        self.robot_urdf = None
        self._kinematics = None
        self._robot_visuals = []
        self._robot_joints_deg = []

    def add_object(
        self,
        name: str,
        component_id: Optional[str] = None,
        pose: Optional["Pose"] = None,
        params: Optional[dict] = None,
        parent: Optional[str] = None,
    ) -> "SceneObject":
        """往场景中添加对象。

        旧签名 ``add_object(name, pose)`` 已废弃：只有位姿没有几何，
        碰撞检测无从谈起。请显式给出 ``component_id``。

        Args:
            name: 对象名
            component_id: 组件 id（见 :data:`core.world.COMPONENTS`）
            pose: 位姿；``None`` 表示原点
            params: 组件参数；``None`` 取默认值
            parent: 父对象名

        Returns:
            新建的 :class:`~domain.world.scene.SceneObject`

        Raises:
            ValueError: 未给出 component_id
        """
        if component_id is None:
            raise ValueError(
                "add_object 需要 component_id；仅给出位姿的对象没有几何体，"
                "无法参与碰撞检测。可用组件：" + ", ".join(sorted(COMPONENTS))
            )
        return self.add(component_id, pose=pose, name=name, params=params, parent=parent)

    def remove_object(self, name: str) -> None:
        """删除对象（子对象自动上提到祖父）。"""
        self.remove(name)

    def has_collisions(self, include_robot: bool = True, margin: float = -0.5) -> bool:
        """是否存在干涉（布尔版）。

        注意：这里**没有**覆盖 :meth:`Scene.check_collisions`——后者返回
        :class:`~domain.world.scene.Collision` 列表（空列表即无干涉）。
        在布尔语境下空列表本身就是假，因此 ``if world.check_collisions():``
        依然成立；本方法只是让意图更明确。

        Args:
            include_robot: 是否计入机器人连杆
            margin: 安全裕度（毫米）。负值表示容忍轻微接触——工件放在台面上、
                夹具夹住工件都是零间隙接触，若把接触当干涉会满屏误报

        Returns:
            ``True`` 表示存在干涉
        """
        return bool(self.check_collisions(include_robot=include_robot, margin=margin))
