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

    def __init__(
        self,
        urdf_path: str,
        name: str = "Robot",
        kinematics_plugin: Optional[str] = None,
        kinematics_options: Optional[dict] = None,
        envelope_tolerance_mm: Optional[float] = None,
    ):
        """
        Initialize a robot from a URDF file.

        Args:
            urdf_path: Path to URDF file
            name: Robot name
            kinematics_plugin: 运动学插件名（v1.0.4）。给定时用插件提供的求解器
                替代内置 :class:`~orolp.kinematics.RobotKinematics`——真机常需要
                自己的解析解或标定过的几何模型。插件会收到 ``urdf_path=`` 加上
                ``kinematics_options`` 里的全部键。
            kinematics_options: 传给运动学插件工厂的额外参数（无插件时忽略）。
                刻意用显式字典而不是 ``**kwargs``：写错关键字的调用应当在
                ``Robot(...)`` 处就报错，而不是被静静吞掉。
            envelope_tolerance_mm: 本机专属的包络接触容差 (mm)（v1.0.9）。
                厂商 <collision> 包络体的"胖瘦"因品牌而异，全局默认
                :data:`orolp.planner.ENVELOPE_TOLERANCE_MM`（20mm，UR5e 实测
                标定）未必适合每台机器——给本机标定过的值即可覆盖。``None``
                表示未标定，走全局默认。负值直接拒绝：容差是放行量，写错
                符号会把"放行浅穿透"变成"要求重叠才算撞"，必须当场报错。
        """
        if envelope_tolerance_mm is not None:
            envelope_tolerance_mm = float(envelope_tolerance_mm)
            if envelope_tolerance_mm < 0.0 or not math.isfinite(envelope_tolerance_mm):
                raise ValueError(
                    f"envelope_tolerance_mm 必须是非负有限值，收到 {envelope_tolerance_mm!r}"
                )
        self.envelope_tolerance_mm = envelope_tolerance_mm
        self.name = name
        self.urdf_path = urdf_path
        self._kinematics_plugin = kinematics_plugin
        self._kinematics_options = dict(kinematics_options or {})
        self._tool = Pose()
        self._base = Pose()
        self._joint_positions: np.ndarray | None = None
        self._dof = 6  # Default 6 DOF
        self._joint_names: List[str] = []
        # 惰性构建的运动学求解器；None 表示"尚未尝试"或"构建失败"
        self._kinematics: Optional["RobotKinematics"] = None
        self._kinematics_loaded = False
        self._kinematics_contract: Optional[str] = None

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

        若构造机器人时给了 ``kinematics_plugin``，则优先用插件提供的实现
        （v1.0.4）；插件构造失败时**回退到内置求解器**并在 ``_kinematics_contract``
        里留下原因——"装了个坏插件就再也算不了运动学"是不可接受的降级方式。
        """
        if self._kinematics_loaded:
            return self._kinematics

        self._kinematics_loaded = True

        if self._kinematics_plugin:
            try:
                from .plugins import KIND_KINEMATICS, check_kinematics, default_registry

                impl = default_registry.create(
                    self._kinematics_plugin,
                    kind=KIND_KINEMATICS,
                    urdf_path=self.urdf_path,
                    **self._kinematics_options,
                )
                self._kinematics = impl
                self._kinematics_contract = check_kinematics(impl).summary()
            except Exception as exc:
                from .plugins import default_registry

                default_registry.add_error(
                    name=self._kinematics_plugin,
                    source="create",
                    message=f"运动学插件加载失败，已回退内置求解器：{exc}",
                )
                print(
                    f"Warning: 运动学插件 {self._kinematics_plugin!r} 加载失败"
                    f"（{exc}），回退内置求解器"
                )
                self._kinematics_plugin = None

        try:
            if self._kinematics is None:
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
    def kinematics_plugin(self) -> Optional[str]:
        """当前生效的运动学插件名（未用插件或插件加载失败后为 ``None``）。"""
        return self._kinematics_plugin

    @property
    def kinematics_contract(self) -> Optional[str]:
        """运动学实现的契约检查结果（插件路径下才有值）。"""
        return self._kinematics_contract

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

    def moveJ(self, joints: List[float] | np.ndarray, speed: float = 100.0, blend: float = 0.0, velocity: float | None = None, acceleration: float | None = None) -> "ProgramBuilder":
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
        return ProgramBuilder(self).moveJ(joints, speed, blend, velocity, acceleration)

    def moveL(self, pose: Pose, speed: float = 100.0, blend: float = 0.0, velocity: float | None = None, acceleration: float | None = None) -> "ProgramBuilder":
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
        return ProgramBuilder(self).moveL(pose, speed, blend, velocity, acceleration)

    def moveC(self, via: Pose, target: Pose, speed: float = 100.0, blend: float = 0.0, velocity: float | None = None, acceleration: float | None = None) -> "ProgramBuilder":
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
        return ProgramBuilder(self).moveC(via, target, speed, blend, velocity, acceleration)

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

    # ------------------------- IO / 工艺便捷入口 ------------------------- #
    # 与 moveJ 同模式：新建 ProgramBuilder 并链上首条指令，方便
    # robot.waitDI(...).moveJ(...) 这类以 IO 开头的程序书写。

    def waitDI(
        self, signal: str, value: bool = True, timeout: float = 0.0, trigger: str = "level"
    ) -> "ProgramBuilder":
        """便捷入口，详见 :meth:`orolp.program.ProgramBuilder.waitDI`。"""
        from .program import ProgramBuilder
        return ProgramBuilder(self).waitDI(signal, value, timeout, trigger)

    def setDO(self, signal: str, value: bool = True) -> "ProgramBuilder":
        """便捷入口，详见 :meth:`orolp.program.ProgramBuilder.setDO`。"""
        from .program import ProgramBuilder
        return ProgramBuilder(self).setDO(signal, value)

    def waitTime(self, seconds: float) -> "ProgramBuilder":
        """便捷入口，详见 :meth:`orolp.program.ProgramBuilder.waitTime`。"""
        from .program import ProgramBuilder
        return ProgramBuilder(self).waitTime(seconds)

    def arc_start(self, process=None) -> "ProgramBuilder":
        """便捷入口，详见 :meth:`orolp.program.ProgramBuilder.arc_start`。"""
        from .program import ProgramBuilder
        return ProgramBuilder(self).arc_start(process)

    def arc_end(self, process=None) -> "ProgramBuilder":
        """便捷入口，详见 :meth:`orolp.program.ProgramBuilder.arc_end`。"""
        from .program import ProgramBuilder
        return ProgramBuilder(self).arc_end(process)

    def spray_on(self, process=None) -> "ProgramBuilder":
        """便捷入口，详见 :meth:`orolp.program.ProgramBuilder.spray_on`。"""
        from .program import ProgramBuilder
        return ProgramBuilder(self).spray_on(process)

    def spray_off(self, process=None) -> "ProgramBuilder":
        """便捷入口，详见 :meth:`orolp.program.ProgramBuilder.spray_off`。"""
        from .program import ProgramBuilder
        return ProgramBuilder(self).spray_off(process)

    def __repr__(self) -> str:
        return f"Robot(name='{self.name}', dof={self._dof})"