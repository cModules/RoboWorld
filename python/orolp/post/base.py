"""后处理器基类与公共格式化工具。

数据契约
--------
后处理器的输入是 :class:`~orolp.program.Motion` 列表，输出是该品牌控制器
能直接加载的源代码文本。子类只需实现两个抽象方法：

* :meth:`BasePostProcessor._convert_motion` —— 把一个 Motion 转成模板字典
* :meth:`BasePostProcessor._get_template` —— Jinja2 模板字符串

单位换算
--------
:class:`~orolp.pose.Pose` 内部使用 **毫米 + 度**，而各品牌控制器并不统一：

============  ========  ========
品牌          线位移    角度
============  ========  ========
KUKA          毫米      度
ABB           毫米      度
FANUC         毫米      度
ESTUN         毫米      度
UR            **米**    **弧度**
JAKA          **米**    **弧度**
============  ========  ========

换算集中在本模块的 :attr:`PostConfig.linear_unit` / :attr:`PostConfig.angle_unit`
与 :meth:`BasePostProcessor.fmt_pose`，子类不要各自做换算，避免单位重复或遗漏。
"""

from __future__ import annotations

import re
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from ..pose import Pose
from ..program import Motion, MotionType, SetDO, WaitDI, WaitTime

__all__ = ["PostConfig", "PostProcessError", "BasePostProcessor"]


class PostProcessError(RuntimeError):
    """后处理失败。

    与历史行为的关键区别：后处理出错时**必须抛出**，而不是静默降级成注释文本。
    旧实现用 ``except Exception`` 兜住一切并输出 ``; Motion 1: ...`` 这样的占位
    注释，导致用户在 GUI 里看到"生成成功"实则拿到一堆废文本。
    """


@dataclass
class PostConfig:
    """后处理器配置。

    Attributes:
        precision: 数值输出的小数位数
        use_joint_names: 是否输出关节名（部分语言支持具名关节）
        speed_unit: 速度呈现方式，``"%"`` 或 ``"mm/s"``
        io_format: IO 指令风格
        comment_char: 注释前缀
        indent: 缩进字符串
        linear_unit: 线位移单位，``"mm"`` 或 ``"m"``
        angle_unit: 角度单位，``"deg"`` 或 ``"rad"``
    """

    precision: int = 3
    use_joint_names: bool = False
    speed_unit: str = "%"
    io_format: str = "digital"
    comment_char: str = ";"
    indent: str = "    "
    linear_unit: str = "mm"
    angle_unit: str = "deg"


class BasePostProcessor(ABC):
    """后处理器基类。

    Args:
        config: 后处理配置；为 ``None`` 时使用默认值
    """

    def __init__(self, config: PostConfig | None = None):
        self.config = config or PostConfig()
        # 延迟导入 Jinja2：未安装时只有真正用到后处理才报错，
        # 不至于让 `import orolp` 整体失败。
        import jinja2

        self.env = jinja2.Environment(
            trim_blocks=True,
            lstrip_blocks=True,
            keep_trailing_newline=True,
        )

    # ------------------------- 元信息 ------------------------- #

    @property
    @abstractmethod
    def brand(self) -> str:
        """品牌标识，需与规格库中的 ``brand_id`` 一致。"""

    @property
    @abstractmethod
    def controller(self) -> str:
        """控制器型号标识。"""

    @property
    def full_name(self) -> str:
        """形如 ``kuka_krc4`` 的完整标识。"""
        return f"{self.brand}_{self.controller}"

    @property
    def language_name(self) -> str:
        """目标语言的可读名称，用于文件扩展名选择与界面展示。"""
        return self.controller.upper()

    @property
    def file_extension(self) -> str:
        """建议的文件扩展名（不含点）。"""
        return "txt"

    # ------------------------- 主入口 ------------------------- #

    def generate(
        self,
        motions: Sequence[Any],
        metadata: Dict[str, Any] | None = None,
    ) -> str:
        """把指令序列渲染为目标控制器源码。

        Args:
            motions: 指令序列，**运动与 IO 混排**（见
                :attr:`~orolp.program.Program.items`）。沿用历史参数名
                ``motions`` 以兼容既有调用方。
            metadata: 附加信息（程序名、机器人名、版本等）。其中
                ``io_map`` 为「信号逻辑名 → 物理地址」映射，用于把
                ``gripper_close`` 这类逻辑名解析成 ``Q0.1``。

        Returns:
            生成的源代码文本

        Raises:
            PostProcessError: 指令类型不被该后处理器支持，或模板渲染失败
        """
        io_map: Dict[str, str] = dict((metadata or {}).get("io_map") or {})

        if any(not isinstance(m, Motion) for m in motions) and not self._io_adapted():
            print(
                f"[warning] {self.full_name} 尚未适配 IO 指令，"
                f"将输出通用伪语法；下装前必须人工核对"
            )

        converted: List[Dict[str, Any]] = []
        for i, item in enumerate(motions):
            try:
                converted.append(self._dispatch(item, io_map))
            except PostProcessError:
                raise
            except Exception as exc:  # 保留异常链，便于定位具体指令
                raise PostProcessError(
                    f"{self.full_name}: 第 {i + 1} 条指令转换失败: {exc}"
                ) from exc

        context = {
            "motions": converted,
            # 兼容历史模板中的 `waypoints` 命名
            "waypoints": converted,
            "meta": metadata or {},
            "config": self.config,
            "post": self,
        }
        try:
            template = self.env.from_string(self._get_template())
            return template.render(**context)
        except Exception as exc:
            raise PostProcessError(
                f"{self.full_name}: 模板渲染失败: {exc}"
            ) from exc

    # ------------------------- 子类实现点 ------------------------- #

    @abstractmethod
    def _convert_motion(self, motion: Motion) -> Dict[str, Any]:
        """把单个 Motion 转换为模板可用的字典。

        Args:
            motion: 运动指令

        Returns:
            供 Jinja2 模板使用的字典

        Raises:
            PostProcessError: 该运动类型不被支持
        """

    @abstractmethod
    def _get_template(self) -> str:
        """返回 Jinja2 模板字符串。"""

    def _dispatch(self, item: Any, io_map: Dict[str, str]) -> Dict[str, Any]:
        """把一条指令（运动或 IO）转换为模板可用的字典。

        Args:
            item: :class:`~orolp.program.Motion` 或
                :class:`~orolp.program.SetDO` / ``WaitDI`` / ``WaitTime``
            io_map: 信号逻辑名 → 物理地址

        Returns:
            供 Jinja2 模板使用的字典
        """
        if isinstance(item, Motion):
            return self._dispatch_motion(item)
        return self._dispatch_io(item, io_map)

    def _dispatch_motion(self, motion: Motion) -> Dict[str, Any]:
        """把一条 Motion 分派给具体转换逻辑。

        SPLINE 目前**统一降级**为等价的 PTP（关节目标）或 LIN（位姿目标）：
        目的地与 SPLINE 完全一致，且所有品牌控制器都能直接加载，无需各自
        实现样条块。若某品牌想改用原生样条指令（KUKA ``SPLINE`` 块、ABB ``Spl``
        等），只需在子类里实现 :meth:`_convert_spline` 覆盖即可。

        Args:
            motion: 运动指令

        Returns:
            供 Jinja2 模板使用的字典
        """
        if motion.type == MotionType.SPLINE:
            if type(self)._convert_spline is not BasePostProcessor._convert_spline:
                return self._convert_spline(motion)
            return self._convert_motion(self._lower_spline(motion))
        return self._convert_motion(motion)

    def _lower_spline(self, motion: Motion) -> Motion:
        """把 SPLINE 运动降级为等价的 PTP / LIN 运动（目的地不变）。

        Args:
            motion: SPLINE 指令

        Returns:
            关节目标转成 :class:`MotionType.JOINTS`，位姿目标转成
            :class:`MotionType.LINEAR`；速度与过渡半径沿用原指令
        """
        if isinstance(motion.target, Pose):
            return Motion(MotionType.LINEAR, motion.target, speed=motion.speed, blend=motion.blend)
        joints = [float(v) for v in np.asarray(motion.target, dtype=np.float64).ravel()]
        return Motion(MotionType.JOINTS, joints, speed=motion.speed, blend=motion.blend)

    def _convert_spline(self, motion: Motion) -> Dict[str, Any]:
        """SPLINE 的原生转换（默认未启用，子类可覆盖以输出品牌样条指令）。"""
        raise PostProcessError(
            f"{self.full_name} 尚未实现原生 SPLINE 指令；"
            f"默认已将其降级为 PTP/LIN（目的地一致）"
        )

    # ------------------------- IO 指令 ------------------------- #

    def _io_adapted(self) -> bool:
        """子类是否覆盖了任一 IO 格式化方法。

        没覆盖的品牌会输出通用伪语法，此时 :meth:`generate` 会打印告警——
        宁可让用户看到"没适配"，也不能悄悄产出语法错误的代码。
        """
        t = type(self)
        return any(
            getattr(t, m) is not getattr(BasePostProcessor, m)
            for m in ("format_setdo", "format_waitdi", "format_waittime")
        )

    def resolve_signal(self, signal: str, io_map: Dict[str, str]) -> str:
        """逻辑信号名 → 物理地址。

        解析不到时**绝不编造假地址**：返回 ``TODO_<信号名>`` 占位并打印告警。
        这样下装前必然被人工发现，而不是悄悄写错点位。
        """
        addr = io_map.get(signal, "")
        if addr:
            return addr
        print(
            f"[warning] {self.full_name}: 信号 {signal!r} 未映射到物理地址，"
            f"已输出 TODO 占位（下装前必须补全）"
        )
        return f"TODO_{signal}"

    @staticmethod
    def io_index(addr: str) -> Optional[int]:
        """把 ``Q0.1`` / ``I1.3`` 这类地址解析成数字量序号（byte*8+bit）。

        解析不了返回 ``None``——调用方**必须给出可见的 TODO**，不得凭空猜一个
        序号。写错点位会导致夹爪不动作甚至误动作，是现场最贵的一类错误。
        """
        m = re.fullmatch(r"\s*([QI])\s*(\d+)\s*\.\s*(\d+)\s*", addr, re.IGNORECASE)
        if not m:
            return None
        return int(m.group(2)) * 8 + int(m.group(3))

    @staticmethod
    def io_kind(addr: str) -> str:
        """地址方向：``"out"``（Q）/ ``"in"``（I）；无法判断返回空串。"""
        m = re.fullmatch(r"\s*([QI])\s*(\d+)\s*\.\s*(\d+)\s*", addr, re.IGNORECASE)
        return ("out" if m.group(1).upper() == "Q" else "in") if m else ""

    def io_symbol(self, signal: str, addr: str) -> str:
        """ABB 这类用**符号名**寻址的语言取信号标识符。

        RAPID 的 ``SetDO`` 要求信号在系统参数里声明过名字，直接用逻辑名最合适；
        物理地址以注释附在行尾，便于核对。
        """
        return signal

    def format_setdo(self, signal: str, value: bool, io_map: Dict[str, str]) -> str:
        """置位/复位数字输出。

        默认实现是**通用伪语法**。子类应按品牌覆盖：KUKA ``$OUT``、
        ABB ``SetDO``、UR ``set_digital_out``。
        """
        return f"SET_DO({self.resolve_signal(signal, io_map)}, {str(value).upper()})"

    def format_waitdi(
        self,
        signal: str,
        value: bool,
        timeout: float,
        io_map: Dict[str, str],
    ) -> str:
        """等待数字输入达到期望电平。``timeout <= 0`` 表示无限等待。"""
        addr = self.resolve_signal(signal, io_map)
        level = str(value).upper()
        if timeout and timeout > 0:
            return f"WAIT_DO({addr}, {level}, TIMEOUT={timeout:.3f})"
        return f"WAIT_DO({addr}, {level})"

    def format_waittime(self, seconds: float) -> str:
        """纯延时（秒）。"""
        return f"WAIT_TIME({seconds:.3f})"

    def _dispatch_io(self, item: Any, io_map: Dict[str, str]) -> Dict[str, Any]:
        """把一条 IO / 流程指令转换为模板可用的字典。

        统一带 ``cmd`` 与 ``text`` 两个键：模板只需判断 ``cmd`` 是否为
        ``SETDO`` / ``WAITDI`` / ``WAITTIME``，是则直接输出 ``text``。
        """
        if isinstance(item, SetDO):
            return {
                "cmd": "SETDO",
                "text": self.format_setdo(item.signal, item.value, io_map),
            }
        if isinstance(item, WaitDI):
            return {
                "cmd": "WAITDI",
                "text": self.format_waitdi(
                    item.signal, item.value, item.timeout, io_map
                ),
            }
        if isinstance(item, WaitTime):
            return {"cmd": "WAITTIME", "text": self.format_waittime(item.seconds)}
        raise PostProcessError(
            f"{self.full_name}: 不支持的指令类型: {type(item).__name__}"
        )

    # ------------------------- 格式化工具 ------------------------- #

    def fmt_joints(self, q: Sequence[float]) -> str:
        """格式化关节值。

        Args:
            q: 关节角，单位**度**

        Returns:
            逗号分隔的数值串
        """
        values = [float(v) if self.config.angle_unit == "deg" else np.radians(v) for v in q]
        return ", ".join(f"{v:.{self.config.precision}f}" for v in values)

    @staticmethod
    def as_matrix(target: Any) -> np.ndarray:
        """把各种目标表示统一成 4x4 齐次矩阵（**毫米**）。

        Args:
            target: :class:`Pose`、4x4 矩阵、16 元展平数组，或
                ``[x, y, z, rx, ry, rz]``（毫米 / 度）

        Returns:
            4x4 齐次矩阵

        Raises:
            PostProcessError: 目标表示无法识别
        """
        if isinstance(target, Pose):
            return target.matrix.copy()

        arr = np.asarray(target, dtype=np.float64).ravel()
        if arr.size == 16:
            return arr.reshape(4, 4)
        if arr.size == 6:
            x, y, z, rx, ry, rz = arr
            return Pose.from_xyz_rpy(x, y, z, rx, ry, rz).matrix
        if arr.size == 3:
            mat = np.eye(4)
            mat[:3, 3] = arr
            return mat
        raise PostProcessError(f"无法识别的目标表示，长度 {arr.size}（期望 3/6/16）")

    def fmt_pose(self, target: Any, order: str = "xyz") -> str:
        """格式化笛卡尔位姿为 ``x, y, z, rx, ry, rz``。

        输入 :class:`Pose` 为毫米 + 度；输出按 :attr:`PostConfig.linear_unit`
        与 :attr:`PostConfig.angle_unit` 换算（UR / JAKA 需要米 + 弧度）。

        Args:
            target: 位姿（见 :meth:`as_matrix`）
            order: 欧拉角约定，``"xyz"`` 或 ``"zyx"``

        Returns:
            逗号分隔的六元数值串

        Raises:
            PostProcessError: order 非法或目标无法识别
        """
        T = self.as_matrix(target)
        xyz = T[:3, 3].copy()
        R = T[:3, :3]

        if order == "xyz":
            ry = np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
            rx = np.arctan2(-R[2, 1], R[2, 2])
            rz = np.arctan2(-R[1, 0], R[0, 0])
        elif order == "zyx":
            ry = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
            rx = np.arctan2(R[2, 1], R[2, 2])
            rz = np.arctan2(R[1, 0], R[0, 0])
        else:
            raise PostProcessError(f"不支持的欧拉角顺序: {order}")

        if self.config.linear_unit == "m":
            xyz = xyz / 1000.0
        elif self.config.linear_unit != "mm":
            raise PostProcessError(f"不支持的线位移单位: {self.config.linear_unit}")

        if self.config.angle_unit == "rad":
            pass  # 上述欧拉角本身即弧度
        elif self.config.angle_unit == "deg":
            rx, ry, rz = np.degrees([rx, ry, rz])
        else:
            raise PostProcessError(f"不支持的角度单位: {self.config.angle_unit}")

        p = self.config.precision
        return f"{xyz[0]:.{p}f}, {xyz[1]:.{p}f}, {xyz[2]:.{p}f}, {rx:.{p}f}, {ry:.{p}f}, {rz:.{p}f}"

    def fmt_pose_quat(self, target: Any) -> str:
        """格式化为 ``x, y, z, qx, qy, qz, qw``（UR 的 pose 形式）。

        Args:
            target: 位姿（见 :meth:`as_matrix`）

        Returns:
            逗号分隔的七元数值串
        """
        T = self.as_matrix(target)
        xyz = T[:3, 3].copy()
        if self.config.linear_unit == "m":
            xyz = xyz / 1000.0
        quat = Rotation.from_matrix(T[:3, :3]).as_quat()  # (x, y, z, w)
        p = self.config.precision
        return (
            f"{xyz[0]:.{p}f}, {xyz[1]:.{p}f}, {xyz[2]:.{p}f}, "
            f"{quat[0]:.{p}f}, {quat[1]:.{p}f}, {quat[2]:.{p}f}, {quat[3]:.{p}f}"
        )

    def fmt_pose_rotvec(self, target: Any) -> str:
        """格式化为 ``x, y, z, rx, ry, rz``，姿态用**旋转向量**（弧度）。

        URScript 与 JAKA 脚本的位姿姿态部分是轴角（旋转向量）而非欧拉角，
        直接用 :meth:`fmt_pose` 输出 RPY 会得到错误的姿态。

        Args:
            target: 位姿（见 :meth:`as_matrix`）

        Returns:
            逗号分隔的六元数值串
        """
        T = self.as_matrix(target)
        xyz = T[:3, 3].copy()
        if self.config.linear_unit == "m":
            xyz = xyz / 1000.0
        rotvec = Rotation.from_matrix(T[:3, :3]).as_rotvec()  # 弧度
        if self.config.angle_unit == "deg":
            rotvec = np.degrees(rotvec)
        elif self.config.angle_unit != "rad":
            raise PostProcessError(f"不支持的角度单位: {self.config.angle_unit}")
        p = self.config.precision
        return (
            f"{xyz[0]:.{p}f}, {xyz[1]:.{p}f}, {xyz[2]:.{p}f}, "
            f"{rotvec[0]:.{p}f}, {rotvec[1]:.{p}f}, {rotvec[2]:.{p}f}"
        )

    def fmt_speed(self, speed: float, motion_type: MotionType) -> str:
        """按配置的单位格式化速度。

        Args:
            speed: 速度值，含义由 :attr:`PostConfig.speed_unit` 决定
            motion_type: 运动类型（部分语言对关节/线性运动速度写法不同）

        Returns:
            格式化后的速度串
        """
        if self.config.speed_unit == "%":
            return f"{speed:.1f}"
        if self.config.speed_unit == "mm/s":
            return f"v{int(round(speed))}"
        if self.config.speed_unit == "m/s":
            return f"{speed:.{self.config.precision}f}"
        return str(speed)

    def fmt_zone(self, blend: float) -> str:
        """格式化过渡半径/逼近区。

        Args:
            blend: 过渡半径，单位毫米；``0`` 表示精确定位

        Returns:
            过渡参数串
        """
        return f"{blend:.{self.config.precision}f}"

    # ------------------------- 运动类型判定 ------------------------- #

    @staticmethod
    def is_joint(motion: Motion) -> bool:
        """是否为关节运动。"""
        return motion.type == MotionType.JOINTS

    @staticmethod
    def is_linear(motion: Motion) -> bool:
        """是否为直线运动。"""
        return motion.type == MotionType.LINEAR

    @staticmethod
    def is_circular(motion: Motion) -> bool:
        """是否为圆弧运动。"""
        return motion.type == MotionType.CIRCULAR

    def __repr__(self) -> str:
        return f"{type(self).__name__}(brand={self.brand!r}, controller={self.controller!r})"
