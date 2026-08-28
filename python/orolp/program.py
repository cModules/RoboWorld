"""Program class for robot motion programming and post-processing."""

from typing import TYPE_CHECKING, Any, Dict, List, Optional, Union
import numpy as np
from enum import Enum
from .pose import Pose
from .robot import Robot

if TYPE_CHECKING:  # 仅类型检查：simulation 依赖 pose，不反向依赖 program
    from .planner import ProgramReport
    from .simulation import Trajectory
    from .world.scene import Scene


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

    def plan(self) -> "Trajectory":
        """把程序展开为真实轨迹（插补 + 逆解 + 节拍估算）。

        与 :meth:`simulate` 的区别只是不额外读取场景，语义上更明确：
        "我要这条轨迹"，而不是"我要跑个仿真"。

        Returns:
            :class:`~orolp.simulation.Trajectory`，时间戳为真实估算值
        """
        from .planner import TrajectoryPlanner

        return TrajectoryPlanner(self.robot).plan(self)

    def simulate(
        self,
        collision_check: bool = False,
        scene: Optional["Scene"] = None,
        margin: float = -0.5,
    ) -> "Trajectory":
        """展开程序为可播放的轨迹。

        历史实现在这里把 ``collision_check`` 收下却什么都不做（源码注释直接
        写着 "not implemented yet"），时间戳更是硬编码成 ``i * 1.0``——
        于是"仿真"既查不出干涉，也算不出节拍。现在两者都真实生效。

        Args:
            collision_check: 是否沿路径做干涉检查
            scene: 工作单元场景；``collision_check=True`` 时**必须**给出，
                否则直接报错（而不是假装检查过了）
            margin: 干涉裕度 (mm)。负值容忍轻微接触——工件放在台面上、
                夹具夹住工件都是零间隙接触，把接触当干涉会满屏误报

        Returns:
            :class:`~orolp.simulation.Trajectory`，时间戳为梯形速度规划的估算值

        Raises:
            ValueError: 要求干涉检查却没有给出场景
        """
        from .planner import TrajectoryPlanner

        if collision_check and scene is None:
            raise ValueError(
                "collision_check=True 需要传入 scene（工作单元场景）；"
                "没有场景就无法判断机器人撞了什么"
            )

        planner = TrajectoryPlanner(self.robot, scene=scene)
        traj = planner.plan(self)
        if collision_check:
            traj.collisions = planner.check_trajectory_collisions(traj, margin=margin)
        return traj

    def validate(
        self,
        scene: Optional["Scene"] = None,
        margin: float = -0.5,
    ) -> "ProgramReport":
        """生成代码前的全面体检。

        检查项：不可达、关节越限、接近奇异、配置跳变、沿路径干涉。

        Args:
            scene: 工作单元场景；``None`` 时跳过干涉检查
            margin: 干涉裕度 (mm)

        Returns:
            :class:`~orolp.planner.ProgramReport`
        """
        from .planner import TrajectoryPlanner

        return TrajectoryPlanner(self.robot, scene=scene).validate(
            self, check_collision=scene is not None, margin=margin
        )

    def to_post(
        self,
        post_processor: str | None = None,
        metadata: Dict[str, Any] | None = None,
    ) -> str:
        """生成目标控制器的机器人代码。

        Args:
            post_processor: 后处理器名称（如 ``kuka_krc4``）；为 ``None`` 时
                根据机器人 URDF 所在目录自动推断品牌
            metadata: 附加信息（程序名、机器人名、版本号等）

        Returns:
            生成的机器人源代码文本

        Raises:
            PostProcessError: 后处理器不存在、品牌无法推断，或指令转换失败

        Note:
            历史实现用 ``except Exception`` 兜住一切错误并输出 ``; Motion 1: ...``
            这类注释占位文本，导致调用方（含 GUI）误以为生成成功。这里改为
            **一律抛出**——宁可让调用方看到明确报错，也不要悄悄产出废代码。
        """
        from .post import get_post_processor

        name = post_processor or self._infer_post_processor()
        pp = get_post_processor(name)

        meta: Dict[str, Any] = {
            "robot_name": self.robot.name,
            "dof": self.robot.dof,
        }
        if metadata:
            meta.update(metadata)

        return pp.generate(self.motions, metadata=meta)

    def _infer_post_processor(self) -> str:
        """根据 URDF 路径中的品牌目录推断后处理器。

        Returns:
            后处理器的 ``full_name``

        Raises:
            PostProcessError: 路径中找不到品牌目录，或该品牌尚无后处理器
        """
        from pathlib import Path

        from .post import list_post_processors, resolve_for_brand
        from .post.base import PostProcessError

        urdf_path = getattr(self.robot, "urdf_path", "") or ""
        parts = Path(urdf_path).parts
        if "models" in parts:
            idx = parts.index("models")
            if idx + 1 < len(parts):
                brand = parts[idx + 1]
                pp = resolve_for_brand(brand)
                if pp is not None:
                    return pp.full_name
                raise PostProcessError(
                    f"品牌 {brand!r} 尚无后处理器；"
                    f"可用：{', '.join(list_post_processors())}"
                )

        raise PostProcessError(
            f"无法从 URDF 路径推断品牌（{urdf_path!r}），请显式指定 post_processor；"
            f"可用：{', '.join(list_post_processors())}"
        )


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

    def simulate(
        self,
        collision_check: bool = False,
        scene: Optional["Scene"] = None,
        margin: float = -0.5,
    ) -> "Trajectory":
        """展开程序为轨迹。参数含义同 :meth:`Program.simulate`。"""
        return self.build().simulate(collision_check, scene, margin)

    def validate(
        self, scene: Optional["Scene"] = None, margin: float = -0.5
    ) -> "ProgramReport":
        """生成代码前的全面体检。参数含义同 :meth:`Program.validate`。"""
        return self.build().validate(scene, margin)

    def to_post(
        self,
        post_processor: str | None = None,
        metadata: Dict[str, Any] | None = None,
    ) -> str:
        """生成机器人代码。

        Args:
            post_processor: 后处理器名称；``None`` 时按机器人品牌自动推断
            metadata: 附加信息

        Returns:
            生成的源代码文本

        Raises:
            PostProcessError: 后处理失败
        """
        return self.build().to_post(post_processor, metadata=metadata)

    def __call__(self) -> Program:
        """
        Build the program when builder is called.

        Returns:
            Program object
        """
        return self.build()