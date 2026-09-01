"""Robot class for representing industrial robots."""

import math
from typing import TYPE_CHECKING, List, Optional, Union
import numpy as np
from .pose import Pose
from .kinematics import KinematicsError, RobotKinematics

if TYPE_CHECKING:  # 仅类型检查：program 依赖 robot，此处不可反向导入
    from .program import ProgramBuilder


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
        # 惰性构建的运动学求解器；None 表示"尚未尝试"或"构建失败"
        self._kinematics: Optional["RobotKinematics"] = None
        self._kinematics_loaded = False

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
    def kinematics(self) -> Optional["RobotKinematics"]:
        """运动学求解器（惰性构建）。

        C++ 核心可用时返回 ``None``（由核心承担计算）；否则返回纯 Python 的
        :class:`~orolp.kinematics.RobotKinematics`。URDF 解析失败时返回 ``None``，
        调用方需据此降级。
        """
        if self._kinematics_loaded:
            return self._kinematics

        self._kinematics_loaded = True
        try:
            from .kinematics import RobotKinematics

            self._kinematics = RobotKinematics(self.urdf_path)
            # 用真实关节数覆盖 URDF 基础解析的估计值
            self._dof = self._kinematics.dof
            self._joint_names = self._kinematics.joint_names
            # 仅在尚未设置（或自由度变了、长度对不上）时才初始化为零位。
            # 历史上这里无条件 ``np.zeros()``，后果是任何一次对 ``.kinematics``
            # 的惰性访问都会把用户刚示教好的位形悄悄清零——而 FK/IK 内部都会
            # 碰这个属性，于是"设了位形再算"这种最普通的用法会随机失效。
            if self._joint_positions is None or len(self._joint_positions) != self._dof:
                self._joint_positions = np.zeros(self._dof)
        except Exception as exc:
            print(f"Warning: 无法构建运动学链 {self.urdf_path}: {exc}")
            self._kinematics = None
        return self._kinematics

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
        """计算机器人正运动学（FK）。

        优先使用 C++ 核心（``orolp._core``，基于 Pinocchio）；若核心未编译，
        则回退到 :class:`~orolp.kinematics.RobotKinematics`——一套**真正可用**的
        纯 Python 实现，而非占位。

        Args:
            joints: 关节角，单位**度**；为 ``None`` 时使用当前关节值

        Returns:
            末端工具位姿，单位**毫米 + 度**
        """
        if joints is None:
            joints = self.get_joints()
        else:
            joints = np.asarray(joints, dtype=np.float64)

        kin = self.kinematics
        if kin is None:
            return self.base * self._tool

        # kinematics 内部用弧度/米，Pose 对外用度/毫米
        T_m = kin.forward_kinematics(np.radians(joints))
        T_mm = T_m.copy()
        T_mm[:3, 3] *= 1000.0
        return self.base * Pose(T_mm) * self._tool

    def inverse_kinematics(
        self,
        target: Pose,
        guess: List[float] | np.ndarray | None = None,
        pos_tol_mm: float = 0.01,
        rot_tol_deg: float = 0.006,
        max_iter: int = 200,
    ) -> np.ndarray:
        """逆运动学（IK）。

        与 :meth:`forward_kinematics` 严格互逆：FK 返回
        ``base · flange · tool``，这里就先把 ``target`` 反解回法兰盘位姿
        ``base⁻¹ · target · tool⁻¹`` 再交给求解器。

        Args:
            target: 目标**工具（TCP）**位姿，世界坐标系，单位毫米 + 度
            guess: 迭代初值（度）。给出上一步的解可以保证位形连续、不翻腕
            pos_tol_mm: 位置收敛阈值（毫米）。默认 0.01 mm 适用于示教点；
                沿路径密集插补时可放宽到 0.05 mm 左右，避免在无工程意义的
                精度上白白失败
            rot_tol_deg: 姿态收敛阈值（度）
            max_iter: 最大迭代次数

        Returns:
            关节值数组（度），长度 :attr:`dof`

        Raises:
            KinematicsError: 逆解失败（目标超出工作空间或处于奇异位形）

        Note:
            历史实现直接拿 ``target`` 当法兰盘位姿用，于是只要设置了
            :attr:`tool`（焊枪、夹爪……）或 :attr:`base`，FK 与 IK 就会
            互相矛盾——机器人会被导到一个完全不同的位置。对离线编程而言
            这是致命的：装了焊枪之后整条程序都是错的。
        """
        kin = self.kinematics
        if kin is None:
            # 没有运动链时无法给出有意义的解；返回 guess 会掩盖问题，
            # 但上层（GUI 拖动示教等）依赖有返回值，故仅在此极端情况兜底。
            if guess is not None:
                return np.asarray(guess, dtype=np.float64)
            return np.zeros(self._dof)

        # 世界系 TCP → 基坐标系法兰盘：T_flange = base⁻¹ · T_tcp · tool⁻¹
        T_flange = (self.base.inv() * target * self.tool.inv()).matrix.copy()
        # 换算到运动学内部单位：米 + 弧度
        T_flange[:3, 3] /= 1000.0
        q_init = np.radians(guess) if guess is not None else None

        try:
            q_rad = kin.inverse_kinematics(
                T_flange,
                q_init=q_init,
                max_iter=max_iter,
                pos_tol=pos_tol_mm / 1000.0,
                rot_tol=math.radians(rot_tol_deg),
            )
        except KinematicsError as exc:
            raise KinematicsError(
                f"{self.name}: 逆解失败（目标可能超出工作空间或处于奇异位形）: {exc}"
            ) from exc
        return np.degrees(q_rad)

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

    def moveSpline(
        self,
        target: Union[List[float], "Pose"],
        speed: float = 100.0,
        blend: float = 0.0,
    ) -> "ProgramBuilder":
        """Create a smooth (minimum-jerk) joint-space motion.

        详见 :meth:`orolp.program.ProgramBuilder.moveSpline`。
        """
        from .program import ProgramBuilder
        return ProgramBuilder(self).moveSpline(target, speed, blend)

    def __repr__(self) -> str:
        return f"Robot(name='{self.name}', dof={self._dof})"