"""力控与柔顺（v1.0.2）：阻抗 / 导纳规划与品牌力控指令。

为什么"阻抗"和"导纳"要一起说
-----------------------------
控制同一个物理量（接触力），两条路线互为对偶，用的是**同一个二阶系统**：

* **导纳（admittance）**：输入力、输出位移。``m·ẍ + c·ẋ + k·x = F_ext``
  ——"我只想维持一个恒力，位置自己找"（恒力打磨、轴孔装配、用限位面定位）。
  机器人表现为"软"：外力一压就退让。
* **阻抗（impedance）**：输入位移、输出力。同一个方程换个方向读
  ——"我按既定轨迹走，遇到东西就按刚度反推一个力"（贴面、倒角、过盈压装）。

UR 的 ``force_mode`` 属于前者：你给期望力，控制器调整位置去达到它。所以本模块
把两个方向都做出来（:func:`simulate_admittance` 与 :func:`predict_contact_force`），
而不是只做一半。

一个必须记住的结论
------------------
机器人柔顺与工件刚度是**串联**的，等效刚度 ``k_eff = k_r·k_e/(k_r + k_e)``：

* 工件很硬（``k_e ≫ k_r``）时 ``k_eff ≈ k_r``——**你设定的刚度就是实际的力-位
  位移比例**，此时 1mm 的位置误差直接变成 ``k_r`` 牛顿的力误差。所以"力控"并不
  等于"力就准了"，它只是把误差从"位置"搬到了"力"，而换算系数就是 ``k_eff``。
* 工件很软（``k_e ≪ k_r``）时 ``k_eff ≈ k_e``——此时机器人刚度设多高都没用，
  力主要由工件决定。

单位与约定
----------
* 长度 **mm**、角度 **度**、时间 **s**、力 **N**、力矩 **N·m**（与仓库其余部分一致）；
  仅在交给品牌控制器时换算成控制器单位（UR 要 m/rad/N/N·m）。
* ``selection`` 六维 0/1，与 UR ``force_mode`` 的 ``selection_vector`` **完全同一
  语义**：1 = 该轴柔顺（力控），0 = 该轴位置控。
* ``wrench`` 六维 ``[Fx, Fy, Fz, Tx, Ty, Tz]``，与 UR 同一约定，在**任务坐标系**
  下表达。
* ``limits`` 六维：柔顺轴是**速度上限**（mm/s、deg/s），非柔顺轴是**允许偏差**
  （mm、deg）——这正是 UR 手册对 ``force_mode`` 的定义，本模块按轴区分单位。

本模块**不发明语法**
--------------------
只有能从厂商文档确认的指令才会被真正输出：

* UR：``force_mode(...)`` / ``end_force_mode()`` / ``force_mode_set_damping()``
  / ``zero_ftsensor()``（URScript 手册 13.1.7~13.1.12）；
* ABB：``FCRefForce`` / ``FCAct`` / ``FCRefStart`` / ``FCRefStop`` / ``FCDeact``
  （*Application manual - Force Control*，RobotWare 7/8）。

KUKA（KUKA.ForceTorqueControl 选件）、FANUC（Force Control 选件）等的 KRL/TP
指令名随选件版本而变，本仓库拿不到权威手册，因此**只输出力控参数 + 显式 TODO**，
绝不臆造一行看起来像代码的东西（与 IO 信号地址同一条规矩）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np

from domain.pose import Pose
from .planner import Issue

__all__ = [
    "AXIS_NAMES",
    "FORCE_AXES",
    "TORQUE_AXES",
    "FORCE_FRAME_TYPES",
    "AxisCompliance",
    "ComplianceSpec",
    "AdmittanceResult",
    "series_stiffness",
    "contact_force",
    "required_depth",
    "force_sensitivity",
    "simulate_admittance",
    "predict_contact_force",
    "wrench_in_base",
    "wrench_joint_torques",
    "check_force_capacity",
]

#: 六维向量的轴名（顺序与 UR 的 selection_vector / wrench / limits 一致）
AXIS_NAMES: Tuple[str, ...] = ("x", "y", "z", "rx", "ry", "rz")

#: 力轴（线）与力矩轴（角）的索引
FORCE_AXES: Tuple[int, ...] = (0, 1, 2)
TORQUE_AXES: Tuple[int, ...] = (3, 4, 5)

#: UR ``force_mode`` 的 ``type`` 取值（任务坐标系的变换方式），见 URScript 手册
FORCE_FRAME_TYPES: Tuple[int, ...] = (1, 2, 3)

#: 单位换算因子：把"显示单位"换到 SI。
#: 线轴 mm ↔ m，因子 1000；角轴 deg ↔ rad，因子 180/π ≈ 57.296。
#: **同一个因子两边都用**：刚度乘以它（N/mm→N/m）、位移除以它（mm→m），
#: 所以位移换算回显示单位也是乘它。写错一处，动态响应就会差 1000 倍。
_MM_TO_M = 1000.0
_DEG_TO_RAD = math.pi / 180.0


def _si_factor(axis: int) -> float:
    """该轴的显示单位 → SI 换算因子。"""
    return _MM_TO_M if axis in FORCE_AXES else 1.0 / _DEG_TO_RAD


#: 默认刚度：线轴 N/mm、角轴 N·m/deg。10 N/mm 是"能手推得动、又不至于撞上就跑"
#: 的常用起点；改成 0.5 会变成"一碰就退"，改成 200 基本等于刚性。
DEFAULT_STIFFNESS: Tuple[float, ...] = (10.0, 10.0, 10.0, 5.0, 5.0, 5.0)

#: 默认允许偏差/速度上限：非柔顺轴 10mm / 5deg（UB 的偏差允许量，给小了力控一
#: 有干扰就报错退出），柔顺轴 100mm/s / 30deg/s（力控下的接近速度上限）
DEFAULT_DEVIATION_MM = 10.0
DEFAULT_DEVIATION_DEG = 5.0
DEFAULT_COMPLIANT_SPEED = 100.0
DEFAULT_COMPLIANT_SPEED_DEG = 30.0

#: 力控阻尼比默认值：0.9 ≈ 临界阻尼略欠，几乎无超调且不过慢。
DEFAULT_DAMPING_RATIO = 0.9

#: 学习/告警用的参考力（N）：超过它提醒核对传感器量程与安全设置。
FORCE_ADVISORY_N = 250.0


# --------------------------------------------------------------------------- #
# 串联系刚度：力控精度的第一性原理
# --------------------------------------------------------------------------- #


def series_stiffness(k_robot: float, k_env: float) -> float:
    """机器人柔顺与环境刚度**串联**后的等效刚度 (N/mm)。

    ``k_eff = k_r·k_e / (k_r + k_e)``

    接触力 ``F = k_eff · Δ``（Δ = 命令位置压入量 − 工件表面位置误差）。
    这个式子解释了力控里最常见的两个误解：

    1. "我把机器人刚度调低，力就更准了"——只有当工件比机器人**硬**时才成立；
       若工件是橡胶（``k_e`` 小），力由工件决定，调机器人刚度没有用。
    2. "力控之后就不怕定位误差了"——不，位置误差按 ``k_eff`` 直接变成力误差。

    Args:
        k_robot: 机器人柔顺轴的设定刚度 (N/mm)，必须 > 0
        k_env: 环境/工件刚度 (N/mm)，必须 > 0

    Raises:
        ValueError: 任一刚度非正（物理上无意义，且会让后续除法爆炸）
    """
    if not math.isfinite(k_robot) or k_robot <= 0.0:
        raise ValueError(f"机器人刚度必须为正的有限值，收到 {k_robot!r}")
    if not math.isfinite(k_env) or k_env <= 0.0:
        raise ValueError(f"环境刚度必须为正的有限值，收到 {k_env!r}")
    return k_robot * k_env / (k_robot + k_env)


def contact_force(k_robot: float, k_env: float, depth: float) -> float:
    """预测接触力 (N)：给定压入量，算出稳态接触力。

    Args:
        k_robot: 机器人柔顺刚度 (N/mm)
        k_env: 环境/工件刚度 (N/mm)
        depth: 压入量 (mm) = 命令位置超过工件名义表面的距离减去表面误差
    """
    return series_stiffness(k_robot, k_env) * float(depth)


def required_depth(force: float, k_robot: float, k_env: float) -> float:
    """反解：要得到 ``force`` (N) 的接触力，需要命令压入多少毫米。

    这是"位置控 + 力控"混合编程里**最该先算的数**——把标称下压量算出来做接近
    轨迹，再用力控兜住剩余误差，比"力控一开就让机器人自己找"安全得多。
    """
    return float(force) / series_stiffness(k_robot, k_env)


def force_sensitivity(k_robot: float, k_env: float) -> float:
    """力对**位置误差**的敏感度 (N/mm) = ``k_eff``。

    直接可用的工程判据：``k_eff = 20 N/mm`` 时，工件摆放偏 0.5mm 就是 10N 的力
    偏差。要更准的力，要么降刚度（牺牲位置精度），要么先把位置误差压下去。
    """
    return series_stiffness(k_robot, k_env)


# --------------------------------------------------------------------------- #
# 单轴柔顺参数
# --------------------------------------------------------------------------- #


@dataclass
class AxisCompliance:
    """单轴的质量-弹簧-阻尼参数。

    Attributes:
        stiffness: 刚度。线轴 N/mm，角轴 N·m/deg
        damping_ratio: 阻尼比 ζ。``c = 2ζ√(k·m)``；ζ<1 欠阻尼（会超调）、
            ζ=1 临界、ζ>1 过阻尼（慢但稳）。力控常用 0.7~1.0
        mass: 惯性参数。线轴 kg，角轴 kg·m²。只影响**动态**（多快到位），
            不影响稳态力
    """

    stiffness: float = 10.0
    damping_ratio: float = DEFAULT_DAMPING_RATIO
    mass: float = 0.5
    #: 显示单位 → SI 的换算因子（线轴 1000，角轴 180/π）。
    #: 由 :meth:`ComplianceSpec.axis_compliance` 按轴自动填好，手写时保持默认即可
    si_factor: float = _MM_TO_M

    def __post_init__(self) -> None:
        if not math.isfinite(self.stiffness) or self.stiffness <= 0.0:
            raise ValueError(f"刚度必须为正的有限值，收到 {self.stiffness!r}")
        if not math.isfinite(self.mass) or self.mass <= 0.0:
            raise ValueError(f"惯性参数必须为正的有限值，收到 {self.mass!r}")
        if not math.isfinite(self.damping_ratio) or self.damping_ratio <= 0.0:
            raise ValueError(f"阻尼比必须为正的有限值，收到 {self.damping_ratio!r}")

    # ---------------- 幅频特性（解析，供规划期判断"来不来得及"） ---------------- #

    @property
    def damping(self) -> float:
        """阻尼系数 ``c = 2ζ√(k·m)``（SI 单位）。"""
        return 2.0 * self.damping_ratio * math.sqrt(self.stiffness * self.si_factor * self.mass)

    @property
    def natural_frequency(self) -> float:
        """无阻尼自然频率 ``ω_n = √(k/m)``（rad/s，SI）。"""
        return math.sqrt(self.stiffness * self.si_factor / self.mass)

    @property
    def natural_frequency_hz(self) -> float:
        """自然频率 (Hz)——力控环的"能跟多快"上限。"""
        return self.natural_frequency / (2.0 * math.pi)

    @property
    def damping_frequency(self) -> float:
        """阻尼振荡频率 ``ω_d = ω_n√(1-ζ²)``（rad/s）；ζ≥1 时返回 0。"""
        z = self.damping_ratio
        if z >= 1.0:
            return 0.0
        return self.natural_frequency * math.sqrt(1.0 - z * z)

    @property
    def is_overdamped(self) -> bool:
        return self.damping_ratio >= 1.0

    @property
    def settling_time(self) -> float:
        """5% 整定时间 (s)：``≈ 3/(ζ·ω_n)``。

        注意这是"力控把力调到位"的时间量级——如果工艺要求 0.1s 内跟上轮廓，
        但整定时间是 0.4s，那是设定问题，不是机器人不行。
        """
        return 3.0 / (self.damping_ratio * self.natural_frequency)

    @property
    def rise_time(self) -> float:
        """欠阻尼阶跃上升时间 (s)：``(π - arccos ζ)/ω_d``；过阻尼时给整定时间。"""
        if self.is_overdamped:
            return self.settling_time
        return (math.pi - math.acos(self.damping_ratio)) / self.damping_frequency

    # ---------------- 静/动态求解 ---------------- #

    def static_offset(self, force: float) -> float:
        """稳态柔顺位移 ``x = F/k``（mm 或 deg）。"""
        return float(force) / self.stiffness

    def force_for_offset(self, offset: float) -> float:
        """已知柔顺位移，反推产生的力（``F = k·x``）。"""
        return self.stiffness * float(offset)

    def step_response(
        self, force: float, duration: float, dt: float = 1e-4
    ) -> Tuple[np.ndarray, np.ndarray]:
        """解析阶跃响应（单位力阶跃，力从 0 跳到 ``force``）。

        用于和数值积分交叉验证——两套独立算法给出同一条曲线，才敢说积分器没写错。

        Returns:
            ``(t, x)``：时间 (s) 与柔顺位移 (mm 或 deg)
        """
        if duration <= 0.0 or dt <= 0.0:
            raise ValueError("duration 与 dt 必须为正")
        n = int(round(duration / dt)) + 1
        t = np.linspace(0.0, duration, n)
        wn = self.natural_frequency
        z = self.damping_ratio
        xs = self.static_offset(force)
        if z < 1.0:
            wd = self.damping_frequency
            env = np.exp(-z * wn * t)
            x = xs * (1.0 - env * (np.cos(wd * t) + (z / math.sqrt(1.0 - z * z)) * np.sin(wd * t)))
        elif z == 1.0:
            x = xs * (1.0 - (1.0 + wn * t) * np.exp(-wn * t))
        else:
            r = math.sqrt(z * z - 1.0)
            s1 = -wn * (z - r)
            s2 = -wn * (z + r)
            # 初值 x(0)=0, ẋ(0)=0
            c2 = -xs * s1 / (s2 - s1)
            c1 = xs - c2
            x = xs + c1 * np.exp(s1 * t) + c2 * np.exp(s2 * t)
        return t, x


# --------------------------------------------------------------------------- #
# 力控规格
# --------------------------------------------------------------------------- #


@dataclass
class ComplianceSpec:
    """一次力控（阻抗/导纳）操作的完整规格。

    这个对象是"可出码的一组力控参数"：UR 的 ``force_mode`` 五个实参、ABB 的
    ``FCRefForce`` 各轴参考值，都从这里换算出去。

    Attributes:
        frame: **任务坐标系**（相对机器人基座），力/力矩在该坐标系下表达。
            UR ``type=2`` 时就是它本身；``type=1/3`` 会被控制器再变换一次
        selection: 六维 0/1，1 = 柔顺（力控）轴
        wrench: 六维期望力/力矩 ``[Fx,Fy,Fz,Tx,Ty,Tz]``（N、N·m）
        type: UR ``force_mode`` 的帧变换方式（1/2/3）；其他品牌由后处理换算
        limits: 六维；柔顺轴 = 速度上限（mm/s、deg/s），非柔顺轴 = 允许偏差
            （mm、deg）。``None`` 用 :meth:`effective_limits` 的保守默认值
        stiffness: 六维刚度（线 N/mm、角 N·m/deg）
        damping_ratio: 阻尼比（对六轴统一取值）
        mass: 六维惯性参数（kg、kg·m²），只影响动态
        name: 名称，出现在出码注释与报表里
    """

    frame: Pose = field(default_factory=Pose)
    selection: Sequence[int] = (0, 0, 1, 0, 0, 0)
    wrench: Sequence[float] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    type: int = 2
    limits: Optional[Sequence[float]] = None
    stiffness: Sequence[float] = DEFAULT_STIFFNESS
    damping_ratio: float = DEFAULT_DAMPING_RATIO
    mass: Sequence[float] = (0.5, 0.5, 0.5, 0.05, 0.05, 0.05)
    name: str = "force_mode"

    def __post_init__(self) -> None:
        self.selection = tuple(int(v) for v in self.selection)
        self.wrench = tuple(float(v) for v in self.wrench)
        if len(self.selection) != 6:
            raise ValueError(f"selection 必须是六维，收到 {len(self.selection)} 维")
        if len(self.wrench) != 6:
            raise ValueError(f"wrench 必须是六维，收到 {len(self.wrench)} 维")
        self.stiffness = tuple(float(v) for v in self.stiffness)
        if len(self.stiffness) != 6:
            raise ValueError(f"stiffness 必须是六维，收到 {len(self.stiffness)} 维")
        if len(self.mass) != 6:
            raise ValueError(f"mass 必须是六维，收到 {len(self.mass)} 维")
        self.mass = tuple(float(v) for v in self.mass)
        if self.limits is not None:
            self.limits = tuple(float(v) for v in self.limits)
            if len(self.limits) != 6:
                raise ValueError(f"limits 必须是六维，收到 {len(self.limits)} 维")

    # ---------------- 便捷构造 ---------------- #

    @classmethod
    def constant_force(
        cls,
        axis: Union[int, str] = "z",
        force: float = -30.0,
        frame: Optional[Pose] = None,
        stiffness: float = 10.0,
        **kwargs: Any,
    ) -> "ComplianceSpec":
        """最常见的用法：**单轴恒力**（如竖直方向压 30N 打磨）。

        Args:
            axis: 受力轴，``0~5`` 或 ``"x"/"y"/"z"/"rx"/"ry"/"rz"``
            force: 期望力/力矩（N 或 N·m），正负按任务坐标系
            frame: 任务坐标系，``None`` 用单位阵（即基座坐标系）
            stiffness: 该轴刚度（N/mm 或 N·m/deg）
            **kwargs: 其余字段透传
        """
        idx = _axis_index(axis)
        selection = [0] * 6
        selection[idx] = 1
        wrench = [0.0] * 6
        wrench[idx] = float(force)
        ks = list(DEFAULT_STIFFNESS)
        ks[idx] = float(stiffness)
        return cls(
            frame=frame if frame is not None else Pose(),
            selection=tuple(selection),
            wrench=tuple(wrench),
            stiffness=tuple(ks),
            **kwargs,
        )

    # ---------------- 轴访问 ---------------- #

    @property
    def compliant_axes(self) -> Tuple[int, ...]:
        """柔顺（力控）轴索引。"""
        return tuple(i for i, s in enumerate(self.selection) if s)

    @property
    def position_axes(self) -> Tuple[int, ...]:
        """位置控轴索引。"""
        return tuple(i for i, s in enumerate(self.selection) if not s)

    def axis_compliance(self, axis: Union[int, str]) -> AxisCompliance:
        """取某一轴的 :class:`AxisCompliance`。"""
        i = _axis_index(axis)
        return AxisCompliance(
            stiffness=self.stiffness[i],
            damping_ratio=self.damping_ratio,
            mass=self.mass[i],
            si_factor=_si_factor(i),
        )

    def effective_limits(self) -> Tuple[float, ...]:
        """实际生效的六维 limits（``limits`` 为 ``None`` 时给保守默认值）。"""
        if self.limits is not None:
            return tuple(self.limits)
        out: List[float] = []
        for i, sel in enumerate(self.selection):
            if sel:
                out.append(
                    DEFAULT_COMPLIANT_SPEED if i in FORCE_AXES else DEFAULT_COMPLIANT_SPEED_DEG
                )
            else:
                out.append(DEFAULT_DEVIATION_MM if i in FORCE_AXES else DEFAULT_DEVIATION_DEG)
        return tuple(out)

    def limit_units(self) -> Tuple[str, ...]:
        """逐轴 limits 的单位说明（UR 手册对柔顺/非柔顺轴的定义不同）。"""
        return tuple(
            (
                ("mm/s" if i in FORCE_AXES else "deg/s")
                if self.selection[i]
                else ("mm" if i in FORCE_AXES else "deg")
            )
            for i in range(6)
        )

    # ---------------- 静/动态 ---------------- #

    def static_offsets(self, wrench: Optional[Sequence[float]] = None) -> np.ndarray:
        """稳态柔顺偏移（六维，任务坐标系）；位置控轴恒为 0。

        这就是"给定力，机器人会退让多少"——也是力控能不能用的第一个判据：
        偏移量必须落在 ``limits`` 给非柔顺轴留的偏差内，否则控制器会直接报错退出。
        """
        w = self.wrench if wrench is None else tuple(float(v) for v in wrench)
        out = np.zeros(6, dtype=np.float64)
        for i in self.compliant_axes:
            out[i] = self.axis_compliance(i).static_offset(w[i])
        return out

    def static_pose(self, wrench: Optional[Sequence[float]] = None) -> Pose:
        """稳态下的 TCP 位姿（任务坐标系偏移后变换回基座）。

        偏移在**任务坐标系**里表达，所以先构一个"纯平移+纯旋转"的局部位姿再左乘
        任务坐标系——这也是仿真里工件轮廓会跟着让步的原因。
        """
        off = self.static_offsets(wrench)
        local = Pose.from_xyz_rpy(*off)
        return self.frame * local

    def wrench_in_base(self, wrench: Optional[Sequence[float]] = None) -> np.ndarray:
        """把任务坐标系下的六维力/力矩旋到基座坐标系（力矩用同一旋转矩阵）。"""
        w = np.asarray(self.wrench if wrench is None else wrench, dtype=np.float64)
        rot = np.asarray(self.frame.matrix[:3, :3], dtype=np.float64)
        out = np.zeros(6, dtype=np.float64)
        out[:3] = rot @ w[:3]
        out[3:] = rot @ w[3:]
        return out

    # ---------------- 校验 ---------------- #

    def validate(self, env_stiffness: Optional[float] = None) -> List[Issue]:
        """检查这组力控参数的物理与安全合理性。

        Args:
            env_stiffness: 环境/工件刚度 (N/mm)。给了就能算出**力误差敏感度**
                与所需下压量——力控方案里最该先看的两个数

        Returns:
            :class:`~service.planner.Issue` 列表（``severity`` 为 error/warning）
        """
        issues: List[Issue] = []

        def add(severity: str, code: str, message: str) -> None:
            issues.append(Issue(severity=severity, code=code, motion_index=None, message=message))

        if not self.compliant_axes:
            add(
                "error", "no_compliant_axis", "selection 全为 0：没有任何轴是柔顺的，等于没有进力控"
            )
        if self.type not in FORCE_FRAME_TYPES:
            add("error", "bad_frame_type", f"type 只能是 {FORCE_FRAME_TYPES}，收到 {self.type}")

        for i in self.position_axes:
            if abs(self.wrench[i]) > 1e-9:
                add(
                    "warning",
                    "wrench_on_rigid_axis",
                    f"{AXIS_NAMES[i]} 轴是位置控，但给了 {self.wrench[i]:.3g} 的期望力/力矩"
                    f"——UR 明确说明该值对非柔顺轴**无效**，若本意是力控请把 selection 置 1",
                )

        bad_stiffness = False
        for i in range(6):
            k = self.stiffness[i]
            if not math.isfinite(k) or k <= 0.0:
                add("error", "bad_stiffness", f"{AXIS_NAMES[i]} 轴刚度必须为正，收到 {k!r}")
                bad_stiffness = True
                continue
            if i in FORCE_AXES:
                if k < 0.5:
                    add(
                        "warning",
                        "very_soft_axis",
                        f"{AXIS_NAMES[i]} 轴刚度 {k:g} N/mm 偏软：轻微偏摆就会跑出很大位移，"
                        f"整定时间 {AxisCompliance(k, self.damping_ratio, self.mass[i]).settling_time:.2f}s",
                    )
                elif k > 200.0:
                    add(
                        "warning",
                        "very_stiff_axis",
                        f"{AXIS_NAMES[i]} 轴刚度 {k:g} N/mm 已接近刚性：力对位置误差近乎 1:1 放大"
                        f"（{k:g} N/mm），力控几乎失去意义",
                    )

        if self.damping_ratio >= 2.0:
            add(
                "warning",
                "heavy_damping",
                f"阻尼比 {self.damping_ratio:g} 过大：不会超调，但响应会明显变慢",
            )

        units = self.limit_units()
        lims = self.effective_limits()
        for i, v in enumerate(lims):
            if not math.isfinite(v) or v <= 0.0:
                add(
                    "error",
                    "bad_limit",
                    f"{AXIS_NAMES[i]} 轴 limit 必须为正（柔顺轴 = 速度上限，非柔顺轴 = 允许偏差），"
                    f"收到 {v!r}",
                )

        # 期望力是否落在"该核对传感器量程"的量级
        peak = max(abs(v) for v in self.wrench) if self.wrench else 0.0
        if peak > FORCE_ADVISORY_N:
            add(
                "warning",
                "force_advisory",
                f"期望力/力矩峰值 {peak:.1f} 超过 {FORCE_ADVISORY_N:.0f}（N 或 N·m）："
                f"请核对力传感器量程、工具刚性以及安全设置",
            )

        # 柔顺偏移是否超出非柔顺轴的允许偏差（控制器会直接报错退出力控）。
        # 刚度非法时 AxisCompliance 会抛 ValueError——校验器应返回诊断而非崩溃，
        # 因此跳过依赖刚度合法性的两项检查
        if not bad_stiffness:
            offs = self.static_offsets()
            for i in self.compliant_axes:
                if abs(offs[i]) > 0.0 and abs(offs[i]) > max(lims[i], 1e-12):
                    add(
                        "warning",
                        "offset_beyond_limit",
                        f"{AXIS_NAMES[i]} 轴稳态柔顺偏移 {offs[i]:+.3f} 超过该轴 limit "
                        f"{lims[i]:g}（若该轴被当作速度上限解释则忽略此条）",
                    )

        if env_stiffness is not None:
            # 已知工件刚度时，力对位置误差的敏感度是**可定量**的。敏感度过高属于
            # 方案问题（换刚度、或先修位置精度），不是"参考信息"，所以报 warning。
            for i in self.compliant_axes:
                if i not in FORCE_AXES:
                    continue
                k_r = self.stiffness[i]
                k_eff = force_sensitivity(k_r, float(env_stiffness))
                if k_eff >= 5.0:
                    add(
                        "warning",
                        "force_too_sensitive",
                        f"{AXIS_NAMES[i]} 轴 k_eff = {k_eff:.2f} N/mm：工件位置每偏 1mm 就是 "
                        f"{k_eff:.2f}N 的力误差（工件越硬越敏感）。要么降低该轴刚度，"
                        f"要么先把工件定位精度提上去",
                    )
        return issues

    def design_report(self, env_stiffness: float) -> str:
        """力控方案的**设计算稿**（不是问题清单，而是"该先算的数"）。

        力控最容易失败在"参数是拍的"：写个 30N 就下发，既不知道要压多深，也不知道
        工件偏一点力会差多少。这个方法把三个数一次算清：

        * 串联系刚度 ``k_eff``（数值上就等于"力对位置误差的敏感度"）；
        * 达到期望力所需的**标称压入量**（用来排接近轨迹，别让力控自己去"找"）；
        * 各柔顺轴的稳态让步量、自然频率与整定时间。

        Args:
            env_stiffness: 工件/环境刚度 (N/mm)
        """
        lines = [f"[{self.name}] 力控方案算稿（工件刚度 {env_stiffness:g} N/mm）"]
        for i in self.compliant_axes:
            ac = self.axis_compliance(i)
            unit = "N/mm" if i in FORCE_AXES else "N·m/deg"
            pos_unit = "mm" if i in FORCE_AXES else "deg"
            lines.append(
                f"  {AXIS_NAMES[i]} 轴：刚度 {ac.stiffness:g} {unit}，期望 "
                f"{self.wrench[i]:+.2f}，稳态让步 "
                f"{ac.static_offset(self.wrench[i]):+.3f}{pos_unit}"
                f"（ω_n={ac.natural_frequency:.1f} rad/s，整定 {ac.settling_time:.3f}s）"
            )
            if i in FORCE_AXES:
                k_eff = force_sensitivity(ac.stiffness, float(env_stiffness))
                depth = required_depth(abs(self.wrench[i]), ac.stiffness, float(env_stiffness))
                lines.append(
                    f"      k_eff={k_eff:.2f} N/mm → 标称压入 {depth:.3f}mm；"
                    f"位置误差 1mm ⇒ 力误差 {k_eff:.2f}N"
                )
        lines.append(
            f"  limits：{list(self.effective_limits())}（单位 {list(self.limit_units())}）"
        )
        return "\n".join(lines)

    # ---------------- 出码 ---------------- #

    def to_ur_force_mode(self) -> Dict[str, Any]:
        """换算成 UR ``force_mode()`` 的实参（**米 / 弧度 / 牛 / 牛米**）。

        UR 的 ``task_frame`` 是相对基座的位姿，姿态用旋转向量；``limits`` 柔顺轴
        是 m/s（或 rad/s），非柔顺轴是 m（或 rad）——与手册一致。
        """
        frame = self.frame.matrix
        xyz = np.asarray(frame[:3, 3], dtype=np.float64) / _MM_TO_M
        rpy = np.asarray(self.frame.to_xyz_rpy(), dtype=np.float64)[3:]
        rotvec = _rpy_to_rotvec_deg(rpy)
        # limits 的换算与刚度同源：柔顺轴是速度（mm/s→m/s、deg/s→rad/s），
        # 非柔顺轴是偏差（mm→m、deg→rad）——单位不同、因子相同
        lims = [v / _si_factor(i) for i, v in enumerate(self.effective_limits())]
        return {
            "task_frame": [float(v) for v in xyz] + [float(v) for v in rotvec],
            "selection_vector": [int(v) for v in self.selection],
            "wrench": [float(v) for v in self.wrench],
            "type": int(self.type),
            "limits": lims,
        }

    def summary(self) -> str:
        """一行式摘要（出码注释与报表共用）。"""
        axes = "/".join(AXIS_NAMES[i] for i in self.compliant_axes) or "无"
        parts = [f"{self.name}：柔顺轴 {axes}"]
        wr = ", ".join(f"{AXIS_NAMES[i]}={self.wrench[i]:+.1f}" for i in self.compliant_axes)
        parts.append(f"期望 {wr}")
        ks = ", ".join(f"{AXIS_NAMES[i]}={self.stiffness[i]:g}" for i in self.compliant_axes)
        parts.append(f"刚度 {ks} (N/mm 或 N·m/deg)")
        parts.append(f"type={self.type}")
        return "；".join(parts)

    def __repr__(self) -> str:  # pragma: no cover - 便于调试
        return f"<ComplianceSpec {self.summary()}>"


# --------------------------------------------------------------------------- #
# 导纳仿真（力 → 位移）与阻抗预测（位移 → 力）
# --------------------------------------------------------------------------- #


@dataclass
class AdmittanceResult:
    """导纳仿真的结果。

    Attributes:
        times: 时间轴 (s)
        offsets: ``(n, 6)`` 任务坐标系下的柔顺偏移（mm / deg）
        forces: ``(n, 6)`` 作用在 TCP 上的外力（输入）
        velocities: ``(n, 6)`` 偏移速度
    """

    times: np.ndarray
    offsets: np.ndarray
    forces: np.ndarray
    velocities: np.ndarray
    spec: ComplianceSpec

    def axis_curve(self, axis: Union[int, str]) -> np.ndarray:
        """某一轴的时间曲线。"""
        return self.offsets[:, _axis_index(axis)]

    def steady_state_offset(self, axis: Union[int, str]) -> float:
        """末态偏移（与解析稳态 ``F/k`` 对比用）。"""
        return float(self.offsets[-1, _axis_index(axis)])

    def peak_offset(self, axis: Union[int, str]) -> float:
        """峰值偏移——超调的直接体现。"""
        return float(np.max(np.abs(self.offsets[:, _axis_index(axis)])))

    def overshoot_percent(self, axis: Union[int, str]) -> float:
        """超调百分比（相对稳态值）。"""
        i = _axis_index(axis)
        final = abs(self.offsets[-1, i])
        if final < 1e-12:
            return 0.0
        return float(max(0.0, (self.peak_offset(i) - final) / final * 100.0))

    def settling_time(self, axis: Union[int, str], tolerance: float = 0.05) -> float:
        """进入并保持 ±``tolerance`` 稳态带的时间 (s)；未进入返回 ``inf``。"""
        i = _axis_index(axis)
        final = self.offsets[-1, i]
        band = abs(final) * tolerance
        inside = np.abs(self.offsets[:, i] - final) <= band
        idx = None
        for k in range(len(inside) - 1, -1, -1):
            if not inside[k]:
                idx = k
                break
        if idx is None:
            return 0.0
        return float(self.times[idx + 1]) if idx + 1 < len(self.times) else float("inf")


def simulate_admittance(
    spec: ComplianceSpec,
    force: Union[float, Sequence[float], Callable[[float], Union[float, Sequence[float]]]],
    duration: float = 1.0,
    dt: float = 0.001,
    axis_force: Optional[Union[float, Callable[[float], float]]] = None,
) -> AdmittanceResult:
    """导纳仿真：**给定外力序列，解出柔顺偏移序列**（力 → 位移）。

    逐柔顺轴独立求解一维二阶系统 ``m·ẍ + c·ẋ + k·x = F_ext``（半隐式欧拉）。
    位置控轴恒为 0——它由位置环决定，不参与柔顺。

    Args:
        spec: 力控规格（刚度/阻尼/惯性取各轴自己的值）
        force: 外力。可以是标量（施加到**唯一/全部柔顺轴**）、六维常量序列，
            或 ``f(t) -> 标量|六维`` 的时间函数
        duration: 仿真时长 (s)
        dt: 步长 (s)
        axis_force: 只给某一轴单独指定外力（``force`` 给标量时用不上）；
            给了它就忽略 ``force`` 的标量含义，把该力打到这一轴上

    Returns:
        :class:`AdmittanceResult`

    Raises:
        ValueError: 没有柔顺轴，或时长/步长非正
    """
    if duration <= 0.0 or dt <= 0.0:
        raise ValueError("duration 与 dt 必须为正")
    axes = spec.compliant_axes
    if not axes:
        raise ValueError("没有任何柔顺轴，导纳仿真无对象（请先设置 selection）")

    n = int(round(duration / dt)) + 1
    times = np.linspace(0.0, duration, n)
    forces = np.zeros((n, 6), dtype=np.float64)
    offsets = np.zeros((n, 6), dtype=np.float64)
    velocities = np.zeros((n, 6), dtype=np.float64)

    target_axes = axes if axis_force is None else (_axis_index(axes[0]),)
    for k, t in enumerate(times):
        f = _eval_force(force, t)
        if np.ndim(f) == 0:
            for i in target_axes:
                forces[k, i] = float(f)
        else:
            arr = np.asarray(f, dtype=np.float64).ravel()
            if arr.size != 6:
                raise ValueError(f"外力序列必须是标量或六维，收到 {arr.size} 维")
            forces[k] = arr

    for i in axes:
        ac = spec.axis_compliance(i)
        f_unit = ac.si_factor  # 1000（线轴）或 180/π（角轴）
        k_si = ac.stiffness * f_unit  # N/m 或 N·m/rad
        c = ac.damping
        m = ac.mass
        x = 0.0  # SI：m 或 rad
        v = 0.0
        for k in range(n - 1):
            force_k = forces[k, i]
            acc = (force_k - c * v - k_si * x) / m
            v_next = v + acc * dt
            x_next = x + v_next * dt  # 半隐式欧拉：先更新速度再更新位置
            x, v = x_next, v_next
            offsets[k + 1, i] = x * f_unit  # 换回显示单位（mm / deg）
            velocities[k + 1, i] = v * f_unit
    return AdmittanceResult(times, offsets, forces, velocities, spec)


def predict_contact_force(
    spec: ComplianceSpec,
    env_stiffness: float,
    surface_error: Union[float, Sequence[float]] = 0.0,
    commanded_depth: Union[float, Sequence[float]] = 0.0,
) -> np.ndarray:
    """阻抗预测：**给定压入量与工件位置误差，算出稳态接触力**（位移 → 力）。

    ``F_i = k_eff,i · (depth_i − error_i)``，其中 ``k_eff`` 由机器人柔顺刚度与
    工件刚度串联得到。位置控轴恒为 0（那里由位置环保证，不由柔顺决定，也不该
    指望它"扛住"外力）。

    Args:
        spec: 力控规格
        env_stiffness: 工件/环境刚度 (N/mm)
        surface_error: 工件表面位置误差 (mm)，正 = 比名义位置更高（压得更深）
        commanded_depth: 命令压入量 (mm)

    Returns:
        六维接触力 (N / N·m)，仅在柔顺的线轴上非零
    """
    depth = np.broadcast_to(np.asarray(commanded_depth, dtype=np.float64), (6,)).copy()
    err = np.broadcast_to(np.asarray(surface_error, dtype=np.float64), (6,)).copy()
    out = np.zeros(6, dtype=np.float64)
    for i in spec.compliant_axes:
        if i not in FORCE_AXES:
            continue
        k_eff = series_stiffness(spec.stiffness[i], float(env_stiffness))
        out[i] = k_eff * (depth[i] - err[i])
    return out


def _eval_force(
    force: Union[float, Sequence[float], Callable[[float], Any]], t: float
) -> np.ndarray:
    """把 force 的三种写法统一求值成 ndarray（标量返回 0 维数组）。"""
    if callable(force):
        return np.asarray(force(t), dtype=np.float64)
    return np.asarray(force, dtype=np.float64)


# --------------------------------------------------------------------------- #
# 力 → 关节载荷（与 v0.3.1 动力学衔接）
# --------------------------------------------------------------------------- #


def wrench_in_base(spec: ComplianceSpec, wrench: Optional[Sequence[float]] = None) -> np.ndarray:
    """任务坐标系下的六维力/力矩 → 基座坐标系（模块级便捷入口）。"""
    return spec.wrench_in_base(wrench)


def wrench_joint_torques(
    robot: Any,
    joints: Sequence[float],
    wrench: Sequence[float],
    frame: Optional[Pose] = None,
) -> np.ndarray:
    """接触力引起的**关节附加力矩** ``τ = Jᵀ·F``。

    力控最容易被忽略的一件事：机器人不仅要"输出"这个力，还要**用关节力矩把它
    顶住**。同一个 30N，在臂完全伸直的位形下要的关节力矩可能是收拢位形的数倍。

    Args:
        robot: :class:`~core.Robot`
        joints: 关节值（**度**，与本仓库 API 一致）
        wrench: 六维力/力矩（N、N·m）
        frame: 该力的表达坐标系；``None`` 表示已是基座坐标系

    Returns:
        各关节的附加力矩 (N·m)

    Raises:
        ValueError: 机器人没有可用的运动学链
    """
    kin = getattr(robot, "kinematics", None)
    if kin is None:
        raise ValueError(
            f"{getattr(robot, 'name', 'robot')} 没有可用的运动学链，无法计算力到关节力矩"
        )
    w = np.asarray(wrench, dtype=np.float64).ravel()
    if w.size != 6:
        raise ValueError(f"wrench 必须是六维，收到 {w.size} 维")
    if frame is not None:
        rot = np.asarray(frame.matrix[:3, :3], dtype=np.float64)
        w = np.concatenate([rot @ w[:3], rot @ w[3:]])
    q_rad = np.radians(np.asarray(joints, dtype=np.float64).ravel())
    jac = kin.jacobian(q_rad)
    return jac.T @ w


def check_force_capacity(
    robot: Any,
    joints: Sequence[float],
    wrench: Sequence[float],
    frame: Optional[Pose] = None,
    near: float = 0.85,
    dynamics: Any = None,
) -> List[Issue]:
    """核对"这个力在当前位形下关节扛不扛得住"（重力 + 接触力，对比额定力矩）。

    这是 v0.3.1 力矩校验的力控版：静态重力矩用牛顿-欧拉逆动力学算（速度、加速度
    取零），叠加 ``Jᵀ·F`` 的接触载荷，再与 URDF ``<limit effort>`` 比。

    Args:
        robot: :class:`~core.Robot`
        joints: 关节值（度）
        wrench: 六维力/力矩；``frame`` 不为 ``None`` 时在其坐标系下
        frame: 力的表达坐标系
        near: 达到额定的该比例即给 warning
        dynamics: 可选的 :class:`~domain.dynamics.DynamicsModel`（如
            ``planner.dynamics_model``），用于带上已录入的末端负载

    Returns:
        :class:`~service.planner.Issue` 列表
    """
    from domain.dynamics import DynamicsModel

    issues: List[Issue] = []
    kin = getattr(robot, "kinematics", None)
    if kin is None:
        return [
            Issue(
                severity="warning",
                code="force_capacity_skip",
                motion_index=None,
                message="机器人没有可用运动学链，跳过关节载荷核对",
            )
        ]
    q_deg = np.asarray(joints, dtype=np.float64).ravel()
    tau_contact = wrench_joint_torques(robot, q_deg, wrench, frame)

    # 想让"已录入的末端负载/连杆惯性"生效，把 planner.dynamics_model 传进来即可；
    # 不传就现建一个（URDF <inertial> 解析一次，代价可接受）
    dyn = dynamics if dynamics is not None else DynamicsModel(kin)
    tau_gravity = dyn.inverse_dynamics(np.radians(q_deg), np.zeros(kin.dof), np.zeros(kin.dof))
    # DynamicsModel.torque_limits 是 property；兼容万一改成方法的形式
    tl = dyn.torque_limits
    limits = np.asarray(tl() if callable(tl) else tl, dtype=np.float64)

    n = min(len(tau_gravity), len(tau_contact), len(limits))
    total = tau_gravity[:n] + tau_contact[:n]
    for i in range(n):
        util = abs(total[i]) / limits[i] if limits[i] else 0.0
        extra = abs(tau_contact[i])
        if util >= 1.0:
            issues.append(
                Issue(
                    severity="error",
                    code="force_torque_limit",
                    motion_index=None,
                    message=(
                        f"轴 {i + 1}：重力矩 {tau_gravity[i]:+.2f} + 接触力附加力矩 "
                        f"{tau_contact[i]:+.2f} = {total[i]:+.2f} N·m，超额定 {limits[i]:.1f} N·m"
                    ),
                )
            )
        elif util >= near:
            issues.append(
                Issue(
                    severity="warning",
                    code="force_torque_near",
                    motion_index=None,
                    message=(
                        f"轴 {i + 1}：合力矩 {total[i]:+.2f} N·m 达额定的 {util * 100:.0f}%"
                        f"（其中接触力占 {extra:.2f} N·m）"
                    ),
                )
            )
    if not issues:
        worst = int(np.argmax(np.abs(total) / np.where(limits > 0, limits, 1.0)))
        issues.append(
            Issue(
                severity="warning",
                code="force_capacity_ok",
                motion_index=None,
                message=(
                    f"关节载荷核对通过：最紧的是轴 {worst + 1}，"
                    f"{abs(total[worst]):.2f}/{limits[worst]:.1f} N·m"
                    f"（{abs(total[worst]) / limits[worst] * 100:.0f}%）"
                ),
            )
        )
    return issues


# --------------------------------------------------------------------------- #
# 内部工具
# --------------------------------------------------------------------------- #


def _axis_index(axis: Union[int, str]) -> int:
    """``"z"`` / ``2`` → 轴索引；非法值显式报错（不做静默兜底）。"""
    if isinstance(axis, str):
        key = axis.strip().lower()
        if key in AXIS_NAMES:
            return AXIS_NAMES.index(key)
        raise ValueError(f"轴名只能是 {AXIS_NAMES}，收到 {axis!r}")
    idx = int(axis)
    if 0 <= idx < 6:
        return idx
    raise ValueError(f"轴索引必须在 0~5，收到 {axis!r}")


def _rpy_to_rotvec_deg(rpy_deg: Sequence[float]) -> Tuple[float, float, float]:
    """RPY（度）→ 旋转向量（弧度）。URScript 的位姿姿态部分是旋转向量。"""
    from domain.pose import Pose as _Pose

    rot = np.asarray(_Pose.from_xyz_rpy(0.0, 0.0, 0.0, *rpy_deg).matrix[:3, :3], dtype=np.float64)
    # 旋转矩阵 → 轴角：迹法 + 反对称部分取轴
    cos_t = max(-1.0, min(1.0, (np.trace(rot) - 1.0) / 2.0))
    theta = math.acos(cos_t)
    if theta < 1e-12:
        return (0.0, 0.0, 0.0)
    if abs(math.pi - theta) < 1e-6:
        # θ≈π：用 (R+I)/2 的列取轴（反对称部分退化）
        m = (rot + np.eye(3)) / 2.0
        diag = np.clip(np.diag(m), 0.0, None)
        k = int(np.argmax(diag))
        axis_v = m[:, k] / math.sqrt(max(diag[k], 1e-18))
    else:
        axis_v = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]]) / (
            2.0 * math.sin(theta)
        )
    vec = axis_v * theta
    return (float(vec[0]), float(vec[1]), float(vec[2]))
