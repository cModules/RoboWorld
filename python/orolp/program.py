"""Program class for robot motion programming and post-processing."""

from dataclasses import dataclass
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


# --------------------------------------------------------------------------- #
# 非运动指令：IO 与流程控制
# --------------------------------------------------------------------------- #


@dataclass
class SetDO:
    """置位/复位一个数字输出（夹爪开合、焊枪起弧……）。

    Attributes:
        signal: 信号**逻辑名**（如 ``gripper_close``），取自
            :data:`~orolp.cell.HANDSHAKE_SIGNALS`。物理地址留到后处理期按
            :class:`~orolp.cell.Device.io_map` 解析——同一份程序换台 PLC
            只改映射，不改程序。
        value: ``True`` 置位、``False`` 复位
    """

    signal: str
    value: bool = True


@dataclass
class WaitDI:
    """等待一个数字输入达到期望电平。

    Attributes:
        signal: 信号逻辑名
        value: 期望电平
        timeout: 超时（秒）；``<= 0`` 表示无限等待
    """

    signal: str
    value: bool = True
    timeout: float = 0.0


@dataclass
class WaitTime:
    """纯延时（秒）——夹具夹紧/松开的保压时间之类。"""

    seconds: float = 0.0


#: 允许出现在 :attr:`Program.items` 中的指令类型
Command = Union[Motion, SetDO, WaitDI, WaitTime]


class Program:
    """Represents a complete robot program."""

    def __init__(self, robot: Robot, motions: List[Command] | None = None):
        """
        Initialize a program.

        Args:
            robot: Robot this program is for
            motions: 指令序列（运动与 IO 混排）；为兼容旧调用名仍叫 motions
        """
        self.robot = robot
        #: 完整指令序列，**运动与 IO 混排**，顺序即执行顺序
        self.items: List[Command] = list(motions) if motions else []

    @property
    def motions(self) -> List[Motion]:
        """仅运动指令，按原顺序。

        保留为属性是为了向后兼容：轨迹规划、节拍估算、干涉检查都只认 Motion，
        新增的 IO 指令对它们应当**完全透明**——它们照旧工作，无需改动。
        """
        return [i for i in self.items if isinstance(i, Motion)]

    def add(self, item: Command) -> None:
        """追加一条指令（运动或 IO）。"""
        self.items.append(item)

    # ------------------------- IO 链式指令 ------------------------- #
    # 与运动指令同源混排：插在哪条运动之后，就什么时候执行。

    def setDO(self, signal: str, value: bool = True) -> "Program":
        """追加一条置位/复位数字输出，返回自身以便链式书写。

        Args:
            signal: 信号**逻辑名**（如 ``gripper_close``），物理地址由
                后处理期 ``io_map`` 解析
            value: ``True`` 置位、``False`` 复位
        """
        self.items.append(SetDO(signal, value))
        return self

    def waitDI(self, signal: str, value: bool = True, timeout: float = 0.0) -> "Program":
        """追加一条等待数字输入，返回自身以便链式书写。

        Args:
            signal: 信号逻辑名
            value: 期望电平
            timeout: 超时（秒）；``<= 0`` 表示无限等待
        """
        self.items.append(WaitDI(signal, value, timeout))
        return self

    def waitTime(self, seconds: float) -> "Program":
        """追加一条纯延时（秒），返回自身以便链式书写。"""
        self.items.append(WaitTime(seconds))
        return self

    def gripper(self, action: str, confirm: bool = True, timeout: float = 2.0) -> "Program":
        """追加一条夹爪开/合指令组，返回自身以便链式书写。

        语义与 :func:`~orolp.pickplace.build_pick_place` 保持一致：

        * ``"close"``：先复位 ``gripper_open``，再置位 ``gripper_close``；
          ``confirm=True`` 时追加 ``WaitDI("clamped", True, timeout)``
        * ``"open"``：先复位 ``gripper_close``，再置位 ``gripper_open``
          （两个信号互锁，避免同时为真）

        Args:
            action: ``"close"`` 或 ``"open"``
            confirm: 是否等待夹紧反馈信号（``open`` 无反馈信号，忽略此项）
            timeout: 等待 ``clamped`` 的超时（秒）

        Returns:
            自身（:class:`Program`），支持 ``robot.moveL(...).gripper("close")`` 链
        """
        if action == "close":
            self.items.append(SetDO("gripper_open", False))
            self.items.append(SetDO("gripper_close", True))
            if confirm:
                self.items.append(WaitDI("clamped", True, timeout))
        elif action == "open":
            self.items.append(SetDO("gripper_close", False))
            self.items.append(SetDO("gripper_open", True))
        else:
            raise ValueError(f"gripper 动作只能是 'open'/'close'，收到 {action!r}")
        return self

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

        # 传 items 而非 motions：IO 指令也要进目标代码，否则生成的程序
        # 只有轨迹、没有抓取动作
        return pp.generate(self.items, metadata=meta)

    def deploy(
        self,
        link: "ControllerLink",
        post_processor: str | None = None,
        name: str | None = None,
        metadata: Dict[str, Any] | None = None,
    ) -> "Deployment":
        """把程序部署到一台控制器（在线或仿真）。

        离线编程的最后一公里：先 :meth:`to_post` 生成代码，再交给连接层落盘/
        烧录。等同于手动 ``code = prog.to_post(...); link.upload_program(...)``
        的组合，但会把品牌、扩展名等一并写进部署清单。

        Args:
            link: 控制器连接（:class:`~orolp.link.FileLink` /
                :class:`~orolp.link.SimLink` / 真实控制器）
            post_processor: 后处理器名；``None`` 时按机器人品牌自动推断
            name: 程序名；``None`` 时取 metadata 的 ``program_name`` 或 ``"program"``
            metadata: 附加信息

        Returns:
            :class:`~orolp.link.Deployment`

        Raises:
            PostProcessError: 后处理失败（与 :meth:`to_post` 一致）
        """
        from .link import Deployment

        code = self.to_post(post_processor, metadata)
        chosen_pp = post_processor or self._infer_post_processor()

        meta: Dict[str, Any] = {
            "post_processor": chosen_pp,
            "robot_name": self.robot.name,
        }
        # 品牌：复用 _infer_post_processor 的目录推断（仅用于清单，不报错）
        try:
            from pathlib import Path as _Path

            from .post import resolve_for_brand as _resolve

            parts = _Path(self.robot.urdf_path).parts
            if "models" in parts:
                _brand = parts[parts.index("models") + 1]
                if _resolve(_brand) is not None:
                    meta["brand"] = _brand
        except Exception:
            pass
        if metadata:
            meta.update(metadata)

        # 让部署目录按正确的文件扩展名落盘（如 UR 的 .script、KUKA 的 .src）
        try:
            from .post import get_post_processor as _get_pp

            meta.setdefault("ext", _get_pp(chosen_pp).file_extension)
        except Exception:
            meta.setdefault("ext", "src")

        prog_name = name or meta.get("program_name") or "program"
        handle = link.upload_program(prog_name, code, meta)
        return Deployment(
            program_name=prog_name,
            post_processor=chosen_pp,
            brand=meta.get("brand"),
            code=code,
            handle=handle,
            ok=True,
            notes=(
                "支持实际位置回读" if getattr(link, "is_actual_feedback", False)
                else "离线导出，无实际位置回读"
            ),
        )

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
        #: 指令序列（运动与 IO 混排）；保留历史名 ``motions`` 以兼容旧调用方
        self.motions: List[Command] = []

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

    def moveSpline(
        self,
        target: Union[List[float], "Pose"],
        speed: float = 100.0,
        blend: float = 0.0,
    ) -> "ProgramBuilder":
        """Add a smooth (minimum-jerk) joint-space motion.

        比 :meth:`moveJ` 多一段连续加减速，适合需要柔和过渡的搬运/装配段。
        ``target`` 是关节列表时直接走关节空间平滑曲线；是位姿时先逆解到关节再
        平滑过去。

        Args:
            target: 目标关节角或工具位姿
            speed: 速度百分比 (0-100)
            blend: 融合半径 (mm)

        Returns:
            This builder for chaining
        """
        self.motions.append(Motion(MotionType.SPLINE, target, speed=speed, blend=blend))
        return self

    # ------------------------- IO 链式指令 ------------------------- #
    # 与运动指令同源混排，语义与 Program 上的同名方法一致。

    def setDO(self, signal: str, value: bool = True) -> "ProgramBuilder":
        """追加一条置位/复位数字输出，返回自身以便链式书写。"""
        self.motions.append(SetDO(signal, value))
        return self

    def waitDI(self, signal: str, value: bool = True, timeout: float = 0.0) -> "ProgramBuilder":
        """追加一条等待数字输入，返回自身以便链式书写。"""
        self.motions.append(WaitDI(signal, value, timeout))
        return self

    def waitTime(self, seconds: float) -> "ProgramBuilder":
        """追加一条纯延时（秒），返回自身以便链式书写。"""
        self.motions.append(WaitTime(seconds))
        return self

    def gripper(self, action: str, confirm: bool = True, timeout: float = 2.0) -> "ProgramBuilder":
        """追加夹爪开/合指令组，返回自身以便链式书写。

        ``robot.moveL(pick).gripper("close")`` 即可得一段完整的抓取动作：
        先复位对侧信号，再夹紧并等待 ``clamped`` 反馈。
        """
        if action == "close":
            self.motions.append(SetDO("gripper_open", False))
            self.motions.append(SetDO("gripper_close", True))
            if confirm:
                self.motions.append(WaitDI("clamped", True, timeout))
        elif action == "open":
            self.motions.append(SetDO("gripper_close", False))
            self.motions.append(SetDO("gripper_open", True))
        else:
            raise ValueError(f"gripper 动作只能是 'open'/'close'，收到 {action!r}")
        return self

    def then(self, other: Union["ProgramBuilder", "Program"]) -> "ProgramBuilder":
        """
        Append another program or builder.

        Args:
            other: Program or builder to append

        Returns:
            This builder for chaining

        Note:
            合并 :class:`Program` 时取完整 ``items``（含 IO 指令）——
            早期实现取 ``motions`` 会悄悄丢掉抓取动作。
        """
        if isinstance(other, ProgramBuilder):
            self.motions.extend(other.motions)
        elif isinstance(other, Program):
            self.motions.extend(other.items)
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