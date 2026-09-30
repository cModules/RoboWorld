"""离线编程核心：轨迹展开、节拍估算与程序校验。

这是 OLP 与「能画个机器人」之间的分水岭。一个 :class:`~service.program.Program`
只有若干条 :class:`~service.program.Motion`（目标点），本机模块负责把它变成
**控制器真正会走的那条路径**：

* :meth:`TrajectoryPlanner.plan` —— 逐条展开成密集的、带时间戳的路径点
* :meth:`TrajectoryPlanner.validate` —— 在生成代码**之前**把不可达、超限位、
  奇异位形、配置跳变、干涉全部报出来

单位约定
--------
对外一律 **毫米 + 度 + 秒**（与 :class:`~domain.pose.Pose` 一致）；
调用运动学内部接口时换算为米 / 弧度。

为什么必须做插补
----------------
``moveL`` 只给出了末端的**终点**位姿。若像历史实现那样只对终点做一次 IK，
那么两点之间机器人究竟怎么走是未知的——直线插补的整段路径可能撞工件，
也可能在中途翻腕。真实控制器（以及 RoboDK）都是在笛卡尔空间密集插补、
逐点求逆解的，这里必须照做。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Dict, List, Optional, Sequence, Tuple

import numpy as np

from support.i18n import tr
from domain.pose import Pose
from .simulation import Trajectory, Waypoint, _interpolate_pose

if TYPE_CHECKING:  # 仅类型标注，避免运行时循环导入
    from .program import Motion, Program
    from domain.robot import Robot
    from domain.world.scene import Collision, Scene

    # _expand_segment 的返回值：incremental 反向依赖 planner 的签名，
    # 运行时不能互相 import，注解只好靠这里借名字（此前是裸字符串，
    # pyflakes 的 F821 说明它根本没被解析过）
    from .incremental import SegmentPayload

__all__ = [
    "PlanningError",
    "Issue",
    "ProgramReport",
    "AxisLimits",
    "TrajectoryPlanner",
    "trapezoid_time",
    "s_curve_time",
    "PROFILE_TRAPEZOID",
    "PROFILE_SCURVE",
    "ENVELOPE_TOLERANCE_MM",
]


class PlanningError(RuntimeError):
    """轨迹规划失败（圆弧退化、指令不支持等）。"""


# --------------------------------------------------------------------------- #
# 默认动态参数
# --------------------------------------------------------------------------- #

#: 未从 URDF 读到 ``<limit velocity>`` 时使用的每轴最大角速度 (deg/s)
DEFAULT_JOINT_SPEED_DEG = 180.0
#: 未从 URDF 读到时的每轴最大角加速度 (deg/s²)。取 4×速度，与常见
#: 工业机器人 0.2~0.3 s 的加速时间量级相符
DEFAULT_JOINT_ACCEL_DEG = 720.0
#: 默认 TCP 直线运动速度 (mm/s)，约合 0.5 m/s
DEFAULT_TCP_SPEED = 500.0
#: 默认 TCP 加速度 (mm/s²)，约合 2 m/s²
DEFAULT_TCP_ACCEL = 2000.0
#: 直线插补的位置分辨率 (mm)。越小越精确但路径点越多
DEFAULT_LINEAR_RES = 1.0
#: 直线插补的姿态分辨率 (deg)
DEFAULT_ANGULAR_RES = 1.0
#: 单条指令最多插补步数，防止病态输入把内存和耗时拖爆
MAX_STEPS_PER_MOTION = 4000
#: 相邻插补点单轴跳变超过该角度即判定为「配置跳变」（多半是过奇异点翻腕）
FLIP_WARN_DEG = 25.0
#: 雅可比条件数超过该值即判定为接近奇异位形
SINGULARITY_COND = 1e4
#: 判定「超出限位」的容差 (deg)，避免浮点噪声误报
LIMIT_TOL_DEG = 1e-6

#: **包络接触容差**（毫米）：干涉检查里小于该穿透量的接触算「包络接触」
#: 而非撞机，对应 ``validate(margin=-ENVELOPE_TOLERANCE_MM)``。
#:
#: 为什么需要它
#: ------------
#: 干涉检查用的是 URDF ``<collision>`` 里的**厂商简化包络体**（UR5e 是 7 个
#: 柱/盒），而不是真实曲面。盒子比它所近似的零件大：UR5e 前臂包络盒
#: 59.7×49mm，而真实前臂是 Ø54mm 圆管——盒角比管壁外扩最多约 11.6mm。
#: 于是"包络层穿透几毫米"在实机上往往还有净空，把这种接触判成撞机会让
#: 每个 6 轴机器人的程序都满屏报错（UR 官方之所以提供
#: "allowed collision matrix"，正是同一个原因）。
#:
#: 20mm 是本仓库用 UR5e + 抓取工作站实测标定的值：整条取放轨迹上最深的
#: 包络穿透为 18.6mm。换机器人/换包络模型应重新标定——这也是为什么它
#: 是显式常量而不是藏在校验函数里的默认值。
ENVELOPE_TOLERANCE_MM = 20.0


# --------------------------------------------------------------------------- #
# 速度规划
# --------------------------------------------------------------------------- #


def trapezoid_time(distance: float, vmax: float, amax: float) -> float:
    """梯形（或三角形）速度规划下的运动时间。

    加速段与减速段对称。距离足够长时能加速到 ``vmax`` 并存在匀速段（梯形）；
    距离太短时还达不到 ``vmax`` 就得开始减速，退化为三角形。

    Args:
        distance: 运动距离（无量纲，与 vmax/amax 同单位制）
        vmax: 最大速度
        amax: 最大加速度

    Returns:
        运动时间

    Note:
        两段的衔接是连续的：当 ``distance = vmax²/amax`` 时，
        梯形公式给出 ``d/vmax + vmax/amax = 2·vmax/amax``，
        三角形公式给出 ``2·sqrt(d/amax) = 2·vmax/amax``，两者相等。
    """
    if distance <= 0.0 or vmax <= 0.0 or amax <= 0.0:
        return 0.0
    # 加速到 vmax 再减速回零所需的最小距离 v²/a
    d_ramp = vmax * vmax / amax
    if distance >= d_ramp:
        return distance / vmax + vmax / amax
    return 2.0 * math.sqrt(distance / amax)


# --------------------------------------------------------------------------- #
# S 曲线速度规划（cosine 缓动段，起停加速度连续、无加加速度突跳）
# --------------------------------------------------------------------------- #

#: 速度曲线风格：梯形（默认，节拍最优）
PROFILE_TRAPEZOID = "trapezoid"
#: 速度曲线风格：S 曲线（cosine 缓动，起停柔和、减少机械冲击）
PROFILE_SCURVE = "scurve"
#: 合法的速度曲线风格
VALID_PROFILES = (PROFILE_TRAPEZOID, PROFILE_SCURVE)


def _scurve_ramp_pos(v: float, tau: float, t: float) -> float:
    """cosine 缓动段内的位移：v(t) = v/2·(1 − cos(πt/τ)) 的积分。

    段末（t = τ）位移为 ``v·τ/2``，速度达到 ``v``，加速度回零。
    """
    x = math.pi * min(max(t / tau, 0.0), 1.0)
    return v * t / 2.0 - v * tau / (2.0 * math.pi) * math.sin(x)


def _scurve_ramp_time(v: float, tau: float, s: float) -> float:
    """cosine 缓动段内由位移反解时刻（s(t) 单调，二分求根）。"""
    lo, hi = 0.0, tau
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if _scurve_ramp_pos(v, tau, mid) < s:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def s_curve_time(distance: float, vmax: float, amax: float) -> float:
    """S 曲线（cosine 缓动段）速度规划下的运动时间。

    速度曲线在起停端是 cosine 缓动：速度、加速度均从零连续起步/收尾，
    没有梯形规划在 t=0 处的加速度突跳，机械冲击更小；代价是同参数下
    节拍略长于 :func:`trapezoid_time`。

    Args:
        distance: 运动距离（与 vmax/amax 同单位制）
        vmax: 最大速度
        amax: 最大加速度

    Returns:
        运动时间
    """
    if distance <= 0.0 or vmax <= 0.0 or amax <= 0.0:
        return 0.0
    tau = math.pi * vmax / (2.0 * amax)
    d_ramp = vmax * tau / 2.0  # 单个缓动段的位移 π·v²/(4a)
    if distance >= 2.0 * d_ramp:
        return 2.0 * tau + (distance - 2.0 * d_ramp) / vmax
    # 三角形 S 曲线：达不到 vmax，峰值速度 vp 满足 π·vp²/(2a) = distance
    vp = math.sqrt(2.0 * amax * distance / math.pi)
    return math.pi * vp / amax


def _profile_segments(
    distance: float, v: float, a: float, profile: str
) -> Tuple[float, float, float]:
    """把 (distance, vmax, amax, profile) 解析成一致的分段参数。

    距离足够长时能加速到 ``v``（梯形段）；太短时退化为三角形，
    实际峰值速度 ``v_eff < v``。三种情况统一为：

    * ``v_eff``：实际巡航峰值速度
    * ``tau``：单侧缓动/加速段时长
    * ``T``：总时长（与 :func:`trapezoid_time` / :func:`s_curve_time` 一致）

    Returns:
        ``(v_eff, tau, T)``
    """
    if profile == PROFILE_SCURVE:
        d_one = math.pi * v * v / (4.0 * a)  # 单侧缓动段位移
        if distance >= 2.0 * d_one:
            tau = math.pi * v / (2.0 * a)
            return v, tau, distance / v + tau
        vp = math.sqrt(2.0 * a * distance / math.pi)
        return vp, math.pi * vp / (2.0 * a), math.pi * vp / a
    d_one = v * v / (2.0 * a)  # 单侧加速段位移
    if distance >= 2.0 * d_one:
        return v, v / a, distance / v + v / a
    vp = math.sqrt(a * distance)
    return vp, vp / a, 2.0 * math.sqrt(distance / a)


def _profile_pos(
    distance: float, v: float, a: float, profile: str, t: float, total: float
) -> float:
    """给定速度曲线在时刻 ``t`` 的位移 s(t)（t ∈ [0, total]）。"""
    v_eff, tau, _ = _profile_segments(distance, v, a, profile)
    if profile == PROFILE_SCURVE:
        if t <= tau:
            return _scurve_ramp_pos(v_eff, tau, t)
        if t >= total - tau:
            return distance - _scurve_ramp_pos(v_eff, tau, total - t)
        return v_eff * tau / 2.0 + v_eff * (t - tau)
    a_eff = v_eff / tau
    if t <= tau:
        return 0.5 * a_eff * t * t
    if t >= total - tau:
        return distance - 0.5 * a_eff * (total - t) ** 2
    return v_eff * tau / 2.0 + v_eff * (t - tau)


def _profile_time(
    distance: float, v: float, a: float, profile: str, s: float, total: float
) -> float:
    """给定速度曲线下位移 ``s`` 对应的时刻（:func:`_profile_pos` 的反函数）。"""
    v_eff, tau, _ = _profile_segments(distance, v, a, profile)
    d_half = v_eff * tau / 2.0
    if profile == PROFILE_SCURVE:
        if s <= d_half:
            return _scurve_ramp_time(v_eff, tau, s)
        if s >= distance - d_half:
            return total - _scurve_ramp_time(v_eff, tau, distance - s)
        return tau + (s - d_half) / v_eff
    a_eff = v_eff / tau
    if s <= d_half:
        return math.sqrt(2.0 * s / a_eff)
    if s >= distance - d_half:
        return total - math.sqrt(2.0 * (distance - s) / a_eff)
    return tau + (s - d_half) / v_eff


def _normalized_params(total: float, ratio: float, profile: str) -> Tuple[float, float]:
    """给定期望总时长，反推 s ∈ [0,1] 归一化域的速度/加速度参数。

    归一化域中距离恒为 1，加速度/速度比固定为 ``ratio``（本机所有约束
    来源——轴加速度 = 4×轴速度、默认 TCP 加速度 = 4×TCP 速度——都满足），
    因此参数由 ``total`` 唯一确定：时间戳与既有节拍公式完全一致，速度曲线
    只改变"怎么走"，不改变"走多久"。

    Args:
        total: 期望运动时间（秒），> 0
        ratio: 加速度/速度比（a = ratio·v）
        profile: ``"trapezoid"`` 或 ``"scurve"``

    Returns:
        ``(v, a)`` 归一化域参数
    """
    if total <= 0.0:
        return 1.0, ratio
    if profile == PROFILE_SCURVE:
        # s_curve_time(1, v, r·v) 随 v 单调递减，二分反解
        lo, hi = 1e-9, 1.0
        for _ in range(200):
            if s_curve_time(1.0, hi, ratio * hi) > total:
                hi *= 2.0
                if hi > 1e12:
                    break
            else:
                break
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            if s_curve_time(1.0, mid, ratio * mid) > total:
                lo = mid
            else:
                hi = mid
        v = 0.5 * (lo + hi)
        return v, ratio * v
    boundary = 2.0 / ratio  # 梯形/三角形分支的边界（distance = v²/a）
    if total >= boundary:
        v = 1.0 / (total - 1.0 / ratio)
        return v, ratio * v
    # 三角形分支：T = 2·sqrt(1/a) → a = 4/T²
    a = 4.0 / (total * total)
    return a / ratio, a


def _synchronized_time(deltas: np.ndarray, vmax: np.ndarray, amax: np.ndarray) -> float:
    """多轴同步运动的时间：由"最吃力的那一轴"决定。

    把路径归一化到 ``s ∈ [0, 1]``，则第 j 轴的位移是 ``|Δq_j| · s``。
    它在 s 域的速度/加速度上限为 ``v_j / |Δq_j|`` 与 ``a_j / |Δq_j|``，
    取所有轴的最小值即为 s 域的整体约束。这就是真实控制器 PTP 的
    "所有轴同时启停"语义。

    Args:
        deltas: 各轴位移量绝对值
        vmax: 各轴最大速度
        amax: 各轴最大加速度

    Returns:
        运动时间
    """
    return _sync_params(deltas, vmax, amax)[0]


def _sync_params(
    deltas: np.ndarray, vmax: np.ndarray, amax: np.ndarray
) -> Tuple[float, float, float]:
    """:meth:`_synchronized_time` 的扩展版：同时返回 s 域的 (T, v, a)。"""
    nonzero = deltas > 1e-12
    if not np.any(nonzero):
        return 0.0, 0.0, 0.0
    d = deltas[nonzero]
    v = float(np.min(vmax[nonzero] / d))
    a = float(np.min(amax[nonzero] / d))
    return trapezoid_time(1.0, v, a), v, a


# --------------------------------------------------------------------------- #
# 轴参数
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class AxisLimits:
    """每轴的运动范围与动态上限。

    旋转关节用 deg / deg·s⁻¹ / deg·s⁻²，移动关节用 mm / mm·s⁻¹ / mm·s⁻²。

    Attributes:
        lower: 下限位
        upper: 上限位
        max_speed: 最大速度
        max_accel: 最大加速度
    """

    lower: np.ndarray
    upper: np.ndarray
    max_speed: np.ndarray
    max_accel: np.ndarray

    @classmethod
    def from_kinematics(cls, kin) -> "AxisLimits":
        """从运动学对象读取轴参数。

        Args:
            kin: :class:`~domain.kinematics.RobotKinematics` 实例

        Returns:
            :class:`AxisLimits`

        Raises:
            PlanningError: 运动学不可用
        """
        if kin is None:
            raise PlanningError("机器人没有可用的运动学模型（URDF 解析失败？）")

        lower_rad, upper_rad = kin.limits()
        speeds = []
        for j in kin.actuated:
            if j.is_prismatic:
                # URDF 中移动关节的速度/限位单位是 m、m/s
                speeds.append(j.max_velocity * 1000.0)
            else:
                speeds.append(math.degrees(j.max_velocity))

        max_speed = np.array(speeds, dtype=np.float64)
        # URDF 不提供加速度，按「约 0.25 s 加速到额定速度」估计
        max_accel = 4.0 * max_speed

        finite = np.isfinite(lower_rad) & np.isfinite(upper_rad)
        lower = np.where(finite, lower_rad, -np.inf)
        upper = np.where(finite, upper_rad, np.inf)

        # continuous 关节或缺失 velocity 时退化成默认值
        fallback = np.array(
            [DEFAULT_JOINT_SPEED_DEG if not j.is_prismatic else 100.0 for j in kin.actuated],
            dtype=np.float64,
        )
        bad = ~np.isfinite(max_speed) | (max_speed <= 0.0)
        max_speed = np.where(bad, fallback, max_speed)
        # 加速度同理；URDF 从不提供，这里始终按速度推算
        max_accel = 4.0 * max_speed

        is_prism = np.array([j.is_prismatic for j in kin.actuated])
        lower = np.where(is_prism, lower * 1000.0, np.degrees(lower))
        upper = np.where(is_prism, upper * 1000.0, np.degrees(upper))
        return cls(lower=lower, upper=upper, max_speed=max_speed, max_accel=max_accel)

    def clamp(self, q: np.ndarray) -> np.ndarray:
        """把关节值夹回限位内（±∞ 限位不受影响）。"""
        return np.clip(q, self.lower, self.upper)

    def violation(self, q: np.ndarray) -> Optional[str]:
        """检查是否越限。

        Args:
            q: 关节值（deg 或 mm）

        Returns:
            越限描述文本；未越限时返回 ``None``
        """
        for i, (lo, hi, v) in enumerate(zip(self.lower, self.upper, q)):
            if np.isfinite(lo) and v < lo - LIMIT_TOL_DEG:
                return tr("轴 {axis} 低于下限位：{value:.3f} < {limit:.3f}").format(
                    axis=i + 1, value=float(v), limit=float(lo)
                )
            if np.isfinite(hi) and v > hi + LIMIT_TOL_DEG:
                return tr("轴 {axis} 超出上限位：{value:.3f} > {limit:.3f}").format(
                    axis=i + 1, value=float(v), limit=float(hi)
                )
        return None


# --------------------------------------------------------------------------- #
# 校验结果
# --------------------------------------------------------------------------- #

SEVERITY_ERROR = "error"
SEVERITY_WARNING = "warning"


@dataclass(frozen=True)
class Issue:
    """程序校验发现的一个问题。

    Attributes:
        severity: ``"error"`` 或 ``"warning"``
        code: 机器可读的问题码
        motion_index: 出问题的指令序号（从 0 计）
        message: 人类可读的描述
        progress: 该指令内部的进度 ``0~1``，``None`` 表示整条指令
        time: 出问题的时刻（秒，相对程序起点）
    """

    severity: str
    code: str
    motion_index: int
    message: str
    progress: Optional[float] = None
    time: Optional[float] = None

    def __str__(self) -> str:
        mark = "✗" if self.severity == SEVERITY_ERROR else "!"
        # motion_index 为 None = 这条问题不属于任何一条指令（程序级/整机级），
        # 例如力控参数校验、关节载荷核对。旧实现直接 +1，一打印就 TypeError。
        where = (
            tr("程序级")
            if self.motion_index is None
            else tr("指令 {n}").format(n=self.motion_index + 1)
        )
        if self.progress is not None:
            where += f" ({self.progress * 100:.0f}%)"
        if self.time is not None:
            where += f" t={self.time:.2f}s"
        return f"{mark} [{where}] {self.message}"


@dataclass
class ProgramReport:
    """程序校验报告。

    Attributes:
        issues: 全部问题
        cycle_time: 估算节拍（秒），含所有指令；``None`` 表示规划未完成
        path_length: TCP 路径总长（mm），只统计 moveL / moveC
        num_motions: 指令条数
        num_waypoints: 展开后的路径点数
    """

    issues: List[Issue] = field(default_factory=list)
    cycle_time: Optional[float] = None
    path_length: float = 0.0
    num_motions: int = 0
    num_waypoints: int = 0
    #: 每条运动指令的耗时（秒），与 :attr:`~service.program.Program.motions`
    #: 对齐；``None`` 表示规划未完成
    motion_times: Optional[List[float]] = None
    #: 每条运动指令的类型标签（如 ``"moveJ"``），与 ``motion_times`` 对齐
    motion_labels: Optional[List[str]] = None
    #: 驻留（waitTime）合计时长（秒），**已计入** cycle_time（v1.1.3 起）
    dwell_total: float = 0.0
    #: 驻留段条数
    num_dwells: int = 0
    #: 每个关节的峰值力矩 |τ|max (N·m；移动关节 N)；``None`` 表示
    #: 力矩校验未执行（缺惯性参数或运动学不可用）
    torque_peaks: Optional[List[float]] = None
    #: 每个关节的额定力矩 (N·m)，与 ``torque_peaks`` 对齐
    torque_limits: Optional[List[float]] = None
    #: 全程最大机械功率 (W)，各关节 |τ·q̇| 之和的峰值
    peak_power: Optional[float] = None
    #: 平均机械功率 (W)
    avg_power: Optional[float] = None

    @property
    def errors(self) -> List[Issue]:
        """错误级问题（会导致程序跑不起来）。"""
        return [i for i in self.issues if i.severity == SEVERITY_ERROR]

    @property
    def warnings(self) -> List[Issue]:
        """警告级问题（能跑，但有风险）。"""
        return [i for i in self.issues if i.severity == SEVERITY_WARNING]

    @property
    def ok(self) -> bool:
        """是否无错误。"""
        return not self.errors

    def format(self) -> str:
        """生成人类可读的多行报告。"""
        lines: List[str] = []
        head = (
            tr("✓ 程序校验通过")
            if self.ok
            else tr("✗ 程序校验未通过（{n} 处错误）").format(n=len(self.errors))
        )
        lines.append(head)
        lines.append(
            tr("  指令 {motions} 条 / 路径点 {waypoints} 个 / TCP 路径 {length:.1f} mm").format(
                motions=self.num_motions, waypoints=self.num_waypoints, length=self.path_length
            )
        )
        if self.cycle_time is not None:
            lines.append(tr("  估算节拍 {time:.2f} s").format(time=self.cycle_time))
        if self.motion_times:
            lines.append(tr("  节拍明细（按指令）："))
            total_share = sum(self.motion_times) or 1.0
            for i, dt in enumerate(self.motion_times):
                label = ""
                if self.motion_labels and i < len(self.motion_labels):
                    label = f" {self.motion_labels[i]}"
                share = dt / total_share * 100.0
                bar = "#" * max(1, int(round(share / 5.0)))
                lines.append(f"    #{i + 1}{label:<10} {dt:6.2f} s  {share:4.1f}% {bar}")
        if self.dwell_total > 0.0:
            lines.append(
                tr("  驻留合计 {total:.2f} s（{n} 段 waitTime，已计入节拍）").format(
                    total=self.dwell_total, n=self.num_dwells
                )
            )
        if self.torque_peaks is not None and self.torque_limits:
            lines.append(tr("  力矩校验（峰值 / 额定）："))
            for j, (pk, lim) in enumerate(zip(self.torque_peaks, self.torque_limits)):
                pct = pk / lim if lim > 0 else 0.0
                mark = "✗" if pct > 1.0 else ("!" if pct >= 0.85 else " ")
                bar = "#" * max(1, int(round(min(pct, 1.5) * 20)))
                lines.append(
                    f"   {mark} 轴 {j + 1}  {pk:7.1f} / {lim:6.1f} N·m  " f"{pct * 100:5.1f}% {bar}"
                )
            if self.peak_power is not None:
                avg = (
                    tr("（平均 {power:.1f} W）").format(power=self.avg_power)
                    if self.avg_power is not None
                    else ""
                )
                lines.append(
                    tr("  机械功率峰值 {power:.1f} W{avg}").format(power=self.peak_power, avg=avg)
                )
        if self.warnings:
            lines.append(tr("  警告 {n} 处：").format(n=len(self.warnings)))
            lines.extend(f"    {w}" for w in self.warnings)
        if self.errors:
            lines.append(tr("  错误 {n} 处：").format(n=len(self.errors)))
            lines.extend(f"    {e}" for e in self.errors)
        return "\n".join(lines)


# --------------------------------------------------------------------------- #
# 几何辅助
# --------------------------------------------------------------------------- #


def _rodrigues(v: np.ndarray, k: np.ndarray, angle: float) -> np.ndarray:
    """绕单位轴 ``k`` 旋转向量 ``v``（Rodrigues 公式）。"""
    c = math.cos(angle)
    s = math.sin(angle)
    return v * c + np.cross(k, v) * s + k * float(k @ v) * (1.0 - c)


def _circle_from_three(
    p0: np.ndarray, p1: np.ndarray, p2: np.ndarray
) -> Tuple[np.ndarray, float, np.ndarray, float]:
    """三点定圆。

    Args:
        p0: 起点
        p1: 中间点（via）
        p2: 终点

    Returns:
        ``(center, radius, axis_unit, total_angle)``，
        ``total_angle`` 带符号，表示从 p0 经 p1 到 p2 的转角（弧度）

    Raises:
        PlanningError: 三点（近）共线，无法确定圆弧
    """
    u = p1 - p0
    v = p2 - p0
    w = np.cross(u, v)
    w_norm = float(np.linalg.norm(w))
    scale = max(1.0, float(np.linalg.norm(u)) * float(np.linalg.norm(v)))
    if w_norm < 1e-9 * scale:
        raise PlanningError(
            tr(
                "圆弧的起点、中间点、终点（近）共线，无法确定圆弧；"
                "请检查 via 点是否落在起终点的连线上"
            )
        )
    w_hat = w / w_norm
    uu = float(u @ u)
    vv = float(v @ v)
    # 外接圆心：p0 + (|u|²·v − |v|²·u) × (u × v) / (2|u × v|²)
    center = p0 + np.cross(uu * v - vv * u, w) / (2.0 * w_norm**2)

    r0 = p0 - center
    r1 = p1 - center
    r2 = p2 - center
    radius = float(np.linalg.norm(r0))

    def _signed(a: np.ndarray, b: np.ndarray) -> float:
        return math.atan2(float(np.cross(a, b) @ w_hat), float(a @ b))

    theta = _signed(r0, r2)
    # atan2 的值域是 (−π, π]，超过半圈的圆弧会被折回短弧。
    # 用 via 点的走向判断真实旋转方向，把长弧补回来。
    theta_via = _signed(r0, r1)
    if theta_via != 0.0 and math.copysign(1.0, theta) != math.copysign(1.0, theta_via):
        theta -= math.copysign(2.0 * math.pi, theta)
    return center, radius, w_hat, theta


def _orientation_angle_deg(p0: Pose, p1: Pose) -> float:
    """两个位姿之间的姿态夹角（度）。"""
    R0 = p0.matrix[:3, :3]
    R1 = p1.matrix[:3, :3]
    R_err = R1 @ R0.T
    # 数值误差可能让 trace 略微越界
    c = float(np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(c))


# --------------------------------------------------------------------------- #
# 规划器
# --------------------------------------------------------------------------- #


class TrajectoryPlanner:
    """把 :class:`~service.program.Program` 展开为可执行轨迹。

    典型用法::

        planner = TrajectoryPlanner(robot, scene=scene)
        traj = planner.plan(program)
        report = planner.validate(program)
        if not report.ok:
            print(report.format())
        else:
            code = program.to_post()

    Args:
        robot: 机器人实例
        scene: 工作单元场景；给出后 :meth:`validate` 会做沿路径干涉检查
        linear_res: 直线插补位置分辨率 (mm)
        angular_res: 直线插补姿态分辨率 (deg)
        tcp_speed: TCP 最大线速度 (mm/s)
        tcp_accel: TCP 最大加速度 (mm/s²)
        path_tol_mm: 中间插补点的位置容差 (mm)
        path_rot_tol_deg: 中间插补点的姿态容差 (deg)
        target_tol_mm: 示教点（指令终点）的位置容差 (mm)
        target_rot_tol_deg: 示教点的姿态容差 (deg)
        profile: 速度曲线风格，``"trapezoid"``（默认，梯形/三角形）或
            ``"scurve"``（cosine 缓动 S 曲线，起停更柔和、节拍略长）
        robot_name: 该机器人在**场景里**的名字（v1.0.1 多机器人）。
            ``None`` 表示场景中的主机器人——单机器人用法不变；多机器人共享
            场景时必须指定，否则 B 的程序会拿 A 的位形去查干涉
        cache: 规划结果缓存实例（:class:`~service.incremental.TrajectoryCache`）。
            ``None``（默认）用全局默认缓存——界面上的"校验一下"会构造多个规划器
            实例算同一份轨迹（校验内一次、面板回放一次），共享缓存才能把它们
            合成一次计算。传实例则只用该实例
        use_cache: 置 ``False`` 完全关掉缓存（首次规划不受影响，见
            :mod:`service.incremental` 的说明）
        collision_cache: 干涉检查**结果**缓存实例
            （:class:`~service.incremental.CollisionResultCache`，v1.2.2）。
            ``None``（默认）用全局默认缓存；键含场景签名，场景一动自然失效
        use_collision_cache: 置 ``False`` 完全关掉干涉检查结果缓存
            （基准与计时场景应关——命中测的不是检查）
        dynamics_cache: 力矩/功率校验**结果**缓存实例
            （:class:`~service.incremental.DynamicsResultCache`，v1.2.3）。
            ``None``（默认）用全局默认缓存；键含模型内容签名
            （改负载/惯性即失效），场景/模型一动自然失效
        use_dynamics_cache: 置 ``False`` 完全关掉力矩校验结果缓存
            （基准与计时场景应关——命中测的不是校验）

    Attributes:
        issues: 最近一次 :meth:`plan` / :meth:`validate` 收集到的问题
    """

    def __init__(
        self,
        robot: "Robot",
        scene: Optional["Scene"] = None,
        linear_res: float = DEFAULT_LINEAR_RES,
        angular_res: float = DEFAULT_ANGULAR_RES,
        tcp_speed: float = DEFAULT_TCP_SPEED,
        tcp_accel: float = DEFAULT_TCP_ACCEL,
        path_tol_mm: float = 0.05,
        path_rot_tol_deg: float = 0.05,
        target_tol_mm: float = 0.01,
        target_rot_tol_deg: float = 0.01,
        profile: str = PROFILE_TRAPEZOID,
        robot_name: Optional[str] = None,
        cache: Optional[object] = None,
        use_cache: bool = True,
        collision_cache: Optional[object] = None,
        use_collision_cache: bool = True,
        dynamics_cache: Optional[object] = None,
        use_dynamics_cache: bool = True,
    ):
        if profile not in VALID_PROFILES:
            raise PlanningError(
                tr("未知速度曲线风格 {profile}；可用：{available}").format(
                    profile=repr(profile), available=" / ".join(VALID_PROFILES)
                )
            )
        self.robot = robot
        self.scene = scene
        self.linear_res = float(linear_res)
        self.angular_res = float(angular_res)
        self.tcp_speed = float(tcp_speed)
        self.tcp_accel = float(tcp_accel)
        # 插补容差（中间点）与示教点容差分开：见 _ik 的说明
        self.path_tol_mm = float(path_tol_mm)
        self.path_rot_tol_deg = float(path_rot_tol_deg)
        self.target_tol_mm = float(target_tol_mm)
        self.target_rot_tol_deg = float(target_rot_tol_deg)
        self.profile = profile
        self.robot_name = robot_name
        # 增量重算（v1.2.0）：内容寻址的规划结果缓存，见 service/incremental.py。
        # 默认走全局缓存——否则"校验一次 + 回放一次"必然把同一份轨迹算两遍
        self.cache = cache
        self.use_cache = bool(use_cache)
        # 干涉检查结果缓存（v1.2.2）：键含场景签名，场景一动自然失效；
        # 校验里规划命中缓存后，干涉检查是剩下的重复大头
        self.collision_cache = collision_cache
        self.use_collision_cache = bool(use_collision_cache)
        # 力矩校验结果缓存（v1.2.3）：键含模型内容签名（改负载/惯性即失效）；
        # 逐路点逆动力学是校验里最贵的一段（实测 5632 路点 ≈ 185ms）
        self.dynamics_cache = dynamics_cache
        self.use_dynamics_cache = bool(use_dynamics_cache)
        self.issues: List[Issue] = []
        self._limits = AxisLimits.from_kinematics(robot.kinematics)
        # 每条指令在轨迹时间轴上的区间 [(t_start, t_end), ...]，用于把干涉
        # 定位回具体指令；plan() 每次重建
        self._motion_spans: List[Tuple[float, float]] = []
        # 区间时刻表的二分索引（惰性重建，见 _span_index）
        self._span_cache_key: Optional[Tuple[int, Optional[Tuple[float, float]]]] = None
        self._span_cache: Tuple[np.ndarray, np.ndarray] = (
            np.empty(0, dtype=np.float64),
            np.empty(0, dtype=np.float64),
        )
        self._last_path_length = 0.0
        # 动力学模型惰性构建（缺 URDF 惯性参数时为 None）；用户可先取
        # 该属性录入负载/连杆惯性，再调用 validate()
        self._dynamics_model: Optional[object] = None
        self._dynamics_tried = False

    # ---------------------------- 对外接口 ---------------------------- #

    @property
    def limits(self) -> AxisLimits:
        """机器人轴参数。"""
        return self._limits

    @property
    def dynamics_model(self):
        """动力学模型（惰性构建），供录入负载与连杆惯性。

        返回 :class:`~domain.dynamics.DynamicsModel`；运动学不可用或 URDF
        无法解析时返回 ``None``。典型用法::

            planner.dynamics_model.set_payload(3.0)   # 3 kg 末端负载
            report = planner.validate(program)
        """
        if not self._dynamics_tried:
            self._dynamics_tried = True
            kin = self.robot.kinematics
            if kin is not None:
                try:
                    from domain.dynamics import DynamicsModel

                    self._dynamics_model = DynamicsModel(kin)
                except Exception:
                    self._dynamics_model = None
        return self._dynamics_model

    def plan(self, program: "Program") -> Trajectory:
        """展开程序为密集轨迹。

        逐条指令插补，失败（不可达 / 越限 / 几何退化）时记录错误并**跳过该条
        指令的剩余部分**——不假装它走完了。

        时间轴语义（v1.1.3 起）：

        * :class:`~service.program.Motion` —— 占时间（插补展开）；
        * :class:`~service.program.WaitTime` —— **驻留段**：关节值恒定、
          时间推进 ``seconds`` 秒，计入节拍与多机时间轴（A 在交接点驻留、
          B 扫过来这类冲突从此可见）；
        * 其余 IO 指令（SetDO/WaitDI/起弧/喷涂/力控）—— 不占时间，对轨迹
          透明（WaitDI 时长取决于现场信号，规划期不可知）。

        Args:
            program: 待展开的程序

        Returns:
            带时间戳的 :class:`~service.simulation.Trajectory`

        Note:
            v1.2.0 起走**增量重算**（:mod:`service.incremental`）：整程序内容、
            机器人结构、起始关节值与规划器参数全都没变时直接交还上次的结果；
            只有部分指令变了时，逐条复用没变的那部分。缓存的键全部由内容派生，
            所以不存在"读到过期结果"这种情况；不想要缓存就传 ``use_cache=False``。
            交还的路点/位姿是**共享对象**，当只读用（改列表容器无妨）。
        """
        from .program import Motion, MotionType, WaitTime
        from .incremental import (
            PlanSnapshot,
            SegmentPayload,
            item_signature,
            planner_signature,
            program_digest,
            resolve_cache,
            robot_signature,
        )

        # 遍历**完整指令序列**（items / ProgramBuilder.motions 都是"全部指令"），
        # 但 _motion_spans 仍只按 Motion 次序记录——与
        # :func:`domain.world.animation.gripper_events` 的"第 n 条运动"保持
        # 一一对齐，夹爪事件时刻不会因混排 IO 或驻留而错位。
        # 历史缺陷备忘：曾经只遍历 program.motions（纯 Motion 视图），
        # WaitTime 对规划器完全不可见——节拍少算全部驻留时间，多机时间轴
        # 也看不见"驻留占用"。
        items = getattr(program, "items", None)
        if items is None:
            items = getattr(program, "motions", [])

        self.issues = []
        waypoints: List[Waypoint] = []
        t_now = 0.0
        path_length = 0.0
        self._motion_spans = []
        self._dwell_segments = []  # (指令序号, 秒, t_start, t_end)

        # 起点：机器人当前关节角
        q_cur = np.asarray(self.robot.get_joints(), dtype=np.float64).ravel()
        if q_cur.size != self.robot.dof:
            q_cur = np.zeros(self.robot.dof)
        q_cur = self._limits.clamp(q_cur)

        # ---- 增量重算：整程序层 ----
        cache = resolve_cache(self)
        token: Optional[Tuple[bytes, bytes]] = None
        signatures: List[str] = []
        plan_key: Optional[Tuple[object, str]] = None
        if cache is not None:
            signatures = [item_signature(item) for item in items]
            token = (planner_signature(self), robot_signature(self.robot))
            plan_key = (token, program_digest(signatures), q_cur.tobytes())
            snapshot = cache.lookup_plan(plan_key)
            if snapshot is not None:
                # 命中：把 plan() 在 self 上的全部副作用按原样放回去
                self._motion_spans = list(snapshot.spans)
                self._dwell_segments = list(snapshot.dwell_segments)
                self._last_path_length = snapshot.path_length
                self.issues = list(snapshot.issues)
                return Trajectory(list(snapshot.waypoints))

        waypoints.append(Waypoint(joints=q_cur.copy(), pose=self._fk(q_cur), timestamp=t_now))

        motion_idx = -1  # _motion_spans / Issue.motion_index 的口径：第几条 Motion
        for cmd_idx, item in enumerate(items):
            # ---- 驻留段：WaitTime 占时间，关节恒定 ---- #
            if isinstance(item, WaitTime):
                try:
                    seconds = max(0.0, float(item.seconds))
                except (TypeError, ValueError):
                    seconds = 0.0
                if seconds > 0.0:
                    t_start = t_now
                    t_now += seconds
                    waypoints.append(
                        Waypoint(joints=q_cur.copy(), pose=self._fk(q_cur), timestamp=t_now)
                    )
                    self._dwell_segments.append((cmd_idx, seconds, t_start, t_now))
                continue
            # ---- 其余 IO：对轨迹透明 ---- #
            if not isinstance(item, Motion):
                continue
            motion_idx += 1
            motion = item
            idx = motion_idx

            # ---- 增量重算：分段层 ----
            # 键 = 规划器/机器人签名 + 该条指令的内容签名 + **该条指令的起始关节值**。
            # 起始关节值绝不能省：第 k 条指令的插补以第 k 条的起点为初值（moveL/moveC
            # 还拿它当逆解初值），前一条改了终点，这条的起点就变了——少了这一项就会
            # 拿旧起点算出的轨迹去接新链，轨迹在接缝处直接跳一下（开发期实测：
            # 改中段一条时会错误地"命中"其后全部 59 条）。反过来说，只改速度/插驻留
            # 这类不动关节值的改动，起点不变 → 后缀照旧命中，这正是增量重算的主要收益。
            segment_key = (
                (token, signatures[cmd_idx], q_cur.tobytes()) if cache is not None else None
            )
            payload = cache.lookup_segment(segment_key) if cache is not None else None
            if payload is None:
                payload = self._expand_segment(q_cur, motion, idx)
                if cache is not None:
                    cache.store_segment(segment_key, payload)

            if payload.failed:
                self.issues.append(
                    Issue(
                        SEVERITY_ERROR, payload.error_code or "planning", idx, payload.error or ""
                    )
                )
                # 失败的指令也要在时间轴上占一个（空）区间——下游
                # （如抓取动画的 IO 事件映射）按 motions 索引对齐 spans，
                # 少一段就会把后续事件的时刻整体错位
                self._motion_spans.append((t_now, t_now))
                continue
            if payload.empty:
                self._motion_spans.append((t_now, t_now))
                continue

            t_start = t_now
            for q, pose, step_dt in zip(payload.pts, payload.poses, payload.step_dts):
                t_now += step_dt
                waypoints.append(Waypoint(joints=np.asarray(q), pose=pose, timestamp=t_now))
            self._motion_spans.append((t_start, t_now))
            q_cur = np.asarray(payload.pts[-1], dtype=np.float64)
            path_length += payload.path_length

            # 终点审计的**序号与时刻按当下值重建**：缓存里只存文案，因为
            # 序号随插入/删除而变、时刻随前缀时长而变，两者都不是这条指令自己的属性
            if payload.warn:
                self.issues.append(
                    Issue(SEVERITY_WARNING, "singularity", idx, payload.warn, time=t_now)
                )

        traj = Trajectory(waypoints)
        self._last_path_length = path_length
        self._populate_dynamics(traj)
        if cache is not None and plan_key is not None:
            cache.store_plan(
                plan_key,
                PlanSnapshot(
                    waypoints=tuple(waypoints),
                    spans=tuple(self._motion_spans),
                    dwell_segments=tuple(self._dwell_segments),
                    path_length=path_length,
                    issues=tuple(self.issues),
                    waypoint_count=len(waypoints),
                ),
            )
        return traj

    def _expand_segment(
        self, q_start: np.ndarray, motion: "Motion", index: int
    ) -> "SegmentPayload":
        """展开一条指令，并把结果（含失败与警告）打成可缓存的分段载荷。

        与"直接插补"的唯一区别是**失败也缓存**：一条指令的失败是确定的（目标越限、
        几何退化），不缓存它就会在每次重算时重放同一个异常、并让命中率虚高。
        异常文案里的指令序号由调用方重建——这里只留文案。
        """
        from .program import MotionType
        from .incremental import SegmentPayload

        try:
            if motion.type == MotionType.JOINTS:
                pts, poses, dt, plen = self._expand_joint(q_start, motion, index)
            elif motion.type == MotionType.LINEAR:
                pts, poses, dt, plen = self._expand_linear(q_start, motion, index)
            elif motion.type == MotionType.CIRCULAR:
                pts, poses, dt, plen = self._expand_circular(q_start, motion, index)
            elif motion.type == MotionType.SPLINE:
                pts, poses, dt, plen = self._expand_spline(q_start, motion, index)
            else:
                raise PlanningError(tr("未知的指令类型：{type}").format(type=repr(motion.type)))
        except PlanningError as exc:
            return SegmentPayload(error=str(exc), error_code="planning")
        except Exception as exc:  # IK 等底层异常统一收敛为可报告的问题
            return SegmentPayload(error=f"{type(exc).__name__}: {exc}", error_code="planning")

        if not pts:
            return SegmentPayload(empty=True)

        # dt 为标量（均匀时间步）或逐点时长序列（速度曲线导致的非均匀步）
        if isinstance(dt, (list, tuple, np.ndarray)):
            step_dts = tuple(float(d) for d in dt)
        else:
            step_dts = tuple([float(dt)] * len(pts))

        q_end = np.asarray(pts[-1], dtype=np.float64)
        return SegmentPayload(
            pts=tuple(pts),
            poses=tuple(poses),
            step_dts=step_dts,
            path_length=float(plen),
            warn=self._audit_text(q_end),
        )

    def _populate_dynamics(self, traj: "Trajectory") -> None:
        """按时间戳对整条轨迹做差分，填充每个路点的关节速度/加速度。

        :class:`~service.simulation.Waypoint` 一直预留着 ``velocity`` /
        ``acceleration`` 字段，历史实现却从未填充。这里用时间戳差分补全：
        速度曲线（梯形/S 曲线）给出的非均匀时间分布会如实反映在数值里。
        同时按轴限幅校核：峰值轴速度超出限位（>5%）时给警告——
        规划已按限位收敛，超限只会出现在弯路径局部，属于需要人工留意的风险。
        """
        wps = traj.waypoints
        if len(wps) < 2:
            return
        qs = np.array([w.joints for w in wps], dtype=np.float64)
        ts = np.array([w.timestamp for w in wps], dtype=np.float64)
        vel = np.zeros_like(qs)
        acc = np.zeros_like(qs)

        seg_v = np.zeros_like(qs)  # 每段（i → i+1）的平均速度
        for i in range(len(wps) - 1):
            dt = ts[i + 1] - ts[i]
            if dt > 1e-9:
                seg_v[i] = (qs[i + 1] - qs[i]) / dt

        for i in range(len(wps)):
            if i == 0:
                vel[i] = seg_v[0]
            elif i == len(wps) - 1:
                vel[i] = seg_v[-1]
            else:
                dt = ts[i + 1] - ts[i - 1]
                if dt > 1e-9:
                    vel[i] = (qs[i + 1] - qs[i - 1]) / dt
            if 0 < i < len(wps) - 1:
                dt2 = ts[i + 1] - ts[i - 1]
                if dt2 > 1e-9:
                    acc[i] = (seg_v[i] - seg_v[i - 1]) / dt2

        for i, wp in enumerate(wps):
            wp.velocity = vel[i].copy()
            wp.acceleration = acc[i].copy()

        # 端点速度解析已知：整条轨迹从静止出发、以静止收束（每条指令的
        # 速度曲线末端速度都是零），差分估计在边界只有半段信息，直接置零
        wps[0].velocity = np.zeros_like(vel[0])
        wps[-1].velocity = np.zeros_like(vel[-1])
        wps[0].acceleration = np.zeros_like(acc[0])
        wps[-1].acceleration = np.zeros_like(acc[-1])

        # 按轴限幅校核：每个运动指令只报一次最坏越限，避免刷屏
        reported: set = set()
        tol = 1.05
        for i, wp in enumerate(wps):
            ratio = np.abs(vel[i]) / np.where(
                self._limits.max_speed > 0, self._limits.max_speed, np.inf
            )
            worst = int(np.argmax(ratio))
            if ratio[worst] > tol:
                seg = self.motion_at(float(ts[i]))
                if seg in reported:
                    continue
                reported.add(seg)
                self.issues.append(
                    Issue(
                        SEVERITY_WARNING,
                        "axis_speed",
                        seg,
                        tr(
                            "轴 {axis} 峰值速度 {speed:.1f} 超出限位 {limit:.1f}"
                            "（{ratio:.0f}%），该段路径弯折较急，建议放缓速度或增加过渡点"
                        ).format(
                            axis=worst + 1,
                            speed=abs(vel[i][worst]),
                            limit=float(self._limits.max_speed[worst]),
                            ratio=ratio[worst] * 100.0,
                        ),
                        time=float(ts[i]),
                    )
                )

    def _span_index(self) -> Tuple[np.ndarray, np.ndarray]:
        """指令区间时刻表的 ``(starts, ends)`` 数组（供二分查询，惰性重建）。

        ``_motion_spans`` 只由 :meth:`plan` 追加写入，因此 ``(长度, 末元素)``
        就是可靠的失效键——长度变了或最后一段变了就重建。
        """
        spans = self._motion_spans
        key = (len(spans), spans[-1] if spans else None)
        if self._span_cache_key != key:
            self._span_cache_key = key
            self._span_cache = (
                np.array([s[0] for s in spans], dtype=np.float64),
                np.array([s[1] for s in spans], dtype=np.float64),
            )
        return self._span_cache

    def motion_at(self, t: float) -> int:
        """查询某个时刻属于第几条指令。

        区间按时间递增、且各段首尾相接（规划失败的指令记零长区间），因此
        ``ends`` 单调不减——可以二分。原实现是顺序扫描，在密集轨迹上被调用
        上万次（干涉定位、夹爪事件、节拍明细），程序一大就是 O(点数×指令数)。
        命中语义与顺序扫描逐位一致：取**最早**一个把 t 含在区间内的指令。

        Args:
            t: 时间（秒，相对轨迹起点）

        Returns:
            指令序号（从 0 计）；找不到时返回 ``-1``（仅当还没有时间轴时）
        """
        spans = self._motion_spans
        if not spans:
            return -1
        starts, ends = self._span_index()
        i = int(np.searchsorted(ends, float(t), side="left"))
        if i >= len(spans) or starts[i] > float(t):
            return len(spans) - 1
        return i

    def validate(
        self,
        program: "Program",
        check_collision: bool = True,
        margin: Optional[float] = None,
        check_dynamics: bool = True,
    ) -> ProgramReport:
        """在生成代码前把程序审一遍。

        Args:
            program: 待校验的程序
            check_collision: 是否做沿路径干涉检查（需要构造时给了 ``scene``）
            margin: 干涉裕度 (mm)。负值容忍轻微接触——工件放在台面上、
                夹具夹住工件都是零间隙接触，把接触当干涉会满屏误报。

                - 显式传值（如 ``-ENVELOPE_TOLERANCE_MM``）：按原样使用
                - ``None``（默认，v1.0.9 起）：**逐机解析**——机器人配置了
                  ``envelope_tolerance_mm`` 时用 ``-该值``；未配置时保持
                  既有默认 ``-0.5``。既让逐机标定自动生效，又不悄悄改老程序
                  的判定结果
            check_dynamics: 是否做力矩/功率校验。缺惯性参数时自动跳过
                （报告一条警告），不阻塞其余校验

        Returns:
            :class:`ProgramReport`
        """
        if margin is None:
            # 逐机覆盖优先（v1.0.9）；未标定保持历史默认 -0.5。
            # 惰性导入：envelope 反向依赖本模块的常量，顶层导入成环
            from domain.envelope import envelope_tolerance_mm as _resolve_tol

            margin = (
                -_resolve_tol(self.robot)
                if getattr(self.robot, "envelope_tolerance_mm", None) is not None
                else -0.5
            )
        traj = self.plan(program)
        motion_times = [t1 - t0 for (t0, t1) in self._motion_spans]
        dwell_segments = list(getattr(self, "_dwell_segments", []))
        motion_labels = []
        from .program import Motion, MotionType as _MT, ForceMode as _ForceMode

        # 与 plan() 同一套过滤规则：IO 指令不是运动，不能进节拍明细
        # （ProgramBuilder.motions 是全部指令的历史别名）
        motions = [m for m in program.motions if isinstance(m, Motion)]
        for m in motions:
            motion_labels.append(
                {
                    _MT.JOINTS: "moveJ",
                    _MT.LINEAR: "moveL",
                    _MT.CIRCULAR: "moveC",
                    _MT.SPLINE: "moveSpline",
                }.get(m.type, str(m.type))
            )
        report = ProgramReport(
            issues=list(self.issues),
            cycle_time=traj.duration(),
            path_length=getattr(self, "_last_path_length", 0.0),
            num_motions=len(motions),
            num_waypoints=len(traj.waypoints),
            motion_times=motion_times,
            motion_labels=motion_labels,
            dwell_total=sum(seg[1] for seg in dwell_segments),
            num_dwells=len(dwell_segments),
        )

        if check_collision:
            if self.scene is None:
                report.issues.append(
                    Issue(
                        SEVERITY_WARNING,
                        "no_scene",
                        0,
                        tr("未传入工作单元场景，跳过干涉检查"),
                    )
                )
            else:
                hits = self.check_trajectory_collisions(traj, margin=margin)
                for hit in hits:
                    # 定位到"撞在哪一段"：指令序号进 Issue.index，连杆名进文案
                    seg = self.motion_at(hit.time) if hit.time is not None else 0
                    detail = str(hit)  # 含连杆名（如 robot:forearm ↔ 托盘）
                    report.issues.append(
                        Issue(
                            SEVERITY_ERROR,
                            "collision",
                            seg,
                            tr("干涉：{detail}").format(detail=detail),
                            time=hit.time,
                        )
                    )
                traj.collisions = hits

        if check_dynamics:
            self._check_dynamics(traj, report)

        # 力控指令（v1.0.2）：逐条校验 ComplianceSpec 的物理与安全合理性；
        # 配对检查——end 多于 start 会真机报错，程序结束仍处力控是安全隐患。
        # validate 也接受 ProgramBuilder（duck typing，历史用法）——它的完整
        # 指令列表叫 motions；Program 优先取 items（含 IO，motions 会滤掉）
        all_items = getattr(program, "items", None)
        if all_items is None:
            all_items = getattr(program, "motions", []) or []
        active_force = 0
        for item in all_items:
            if not isinstance(item, _ForceMode):
                continue
            if item.end:
                active_force -= 1
                if active_force < 0:
                    report.issues.append(
                        Issue(
                            SEVERITY_ERROR,
                            "force_mode_unbalanced",
                            0,
                            tr(
                                "force_mode_end() 没有对应的进入力控指令——真机上 "
                                "end_force_mode/FCDeact 会直接报错"
                            ),
                        )
                    )
                    active_force = 0  # 避免一条多余 end 刷出多条同因报错
                continue
            active_force += 1
            if item.spec is not None:
                report.issues.extend(item.spec.validate())
        if active_force > 0:
            report.issues.append(
                Issue(
                    SEVERITY_WARNING,
                    "force_mode_active_at_end",
                    0,
                    tr(
                        "程序结束时仍处于力控状态（进入力控 N 次但未退出）——"
                        "确认这是有意为之，否则请补 force_mode_end()"
                    ),
                )
            )
        return report

    def _check_dynamics(self, traj: Trajectory, report: ProgramReport) -> None:
        """力矩/功率校验：逐路点逆动力学，超额定报错、逼近额定预警。

        结果同时写进报表的 ``torque_peaks`` 等字段，供 :meth:`ProgramReport.format`
        打印力矩报表。缺惯性参数（URDF 没写 ``<inertial>`` 且未手动录入）
        时只挂一条跳过警告——力矩校验是增强项，不该阻塞其余校验。

        结果缓存（v1.2.3）：键由模型内容签名（连杆惯性/末端负载/额定力矩）+
        逐路点关节位置/速度/加速度 + 指令时间区间派生——改负载、改惯性、
        改程序都换键；命中时走与实算同一条"应用到报表"路径，结论逐字段一致。
        """
        model = self.dynamics_model
        if model is None:
            report.issues.append(
                Issue(
                    SEVERITY_WARNING,
                    "dynamics_skip",
                    0,
                    tr("力矩校验跳过：机器人没有可用的运动学模型"),
                )
            )
            return
        if not model.has_inertial:
            report.issues.append(
                Issue(
                    SEVERITY_WARNING,
                    "dynamics_skip",
                    0,
                    tr(
                        "力矩校验跳过：主链连杆缺少惯性参数（{links}）"
                        "——可从 CAD 导出带 <inertial> 的 URDF，"
                        "或用 DynamicsModel.set_link_inertial 手动录入"
                    ).format(links=", ".join(model.missing_links)),
                )
            )
            return

        from domain.dynamics import dynamics_cache_key
        from .incremental import resolve_dynamics_cache

        cache = resolve_dynamics_cache(self)
        key: Optional[bytes] = None
        if cache is not None:
            key = dynamics_cache_key(model, traj, self._motion_spans)
            cached = cache.lookup(key)
            if cached is not None:
                self._apply_dynamics_result(report, cached)
                return

        result = model.check_trajectory(traj, motion_at=self.motion_at)
        if key is not None:
            cache.store(key, result)
        self._apply_dynamics_result(report, result)

    def _apply_dynamics_result(self, report: ProgramReport, result: dict) -> None:
        """把 :meth:`DynamicsModel.check_trajectory` 的结果写进报表。

        实算与缓存命中共用这一条路径——Issue 文案由结果值确定性生成，
        命中与实算对调用方无差别。
        """
        report.torque_peaks = result["torque_peaks"]
        report.torque_limits = result["torque_limits"]
        report.peak_power = result["peak_power"]
        report.avg_power = result["avg_power"]

        for seg, joint, ratio, torque, t in result["overloads"]:
            report.issues.append(
                Issue(
                    SEVERITY_ERROR,
                    "torque_limit",
                    seg,
                    tr(
                        "关节 {joint} 峰值力矩 {torque:.1f} N·m 超出额定 {limit:.1f} N·m"
                        "（{ratio:.0f}%）——该段加减速或姿态对关节负载过重，"
                        "建议放缓速度、增加过渡点或减小负载"
                    ).format(
                        joint=joint + 1,
                        torque=abs(torque),
                        limit=float(result["torque_limits"][joint]),
                        ratio=ratio * 100.0,
                    ),
                    time=t,
                )
            )
        for seg, joint, ratio, torque, t in result["near_limits"]:
            report.issues.append(
                Issue(
                    SEVERITY_WARNING,
                    "torque_near",
                    seg,
                    tr(
                        "关节 {joint} 峰值力矩 {torque:.1f} N·m 达到额定 {limit:.1f} N·m 的"
                        "{ratio:.0f}%，余量偏小，建议留意"
                    ).format(
                        joint=joint + 1,
                        torque=abs(torque),
                        limit=float(result["torque_limits"][joint]),
                        ratio=ratio * 100.0,
                    ),
                    time=t,
                )
            )

    def check_trajectory_collisions(
        self,
        traj: Trajectory,
        margin: float = -0.5,
        stride: int = 0,
        max_joint_step: Optional[float] = None,
    ) -> List["Collision"]:
        """沿整条轨迹做干涉检查（连杆级）。

        不只是检查路径点本身——两点之间机器人可能已经撞上了，所以必须密集采样。
        检查由 :class:`domain.collision.CollisionChecker` 完成（广相 AABB →
        窄相 SAT）：命中带连杆名（``link_a``，撞在哪一段连杆）与指令序号
        （``motion``，撞在哪段轨迹），连杆-连杆自碰撞一并覆盖。

        几何来自 URDF 的 ``<collision>`` 包络体（保守盒/柱），所以
        ``margin`` 给了负值时小于该穿透量的接触不算命中——
        :data:`ENVELOPE_TOLERANCE_MM` 是本仓库对该容差的推荐值。

        采样口径（v1.1.9 显式化，见 :func:`domain.collision.sample_indices`）：
        默认按轨迹长度把检查次数控制在约 :data:`domain.collision.COLLISION_SAMPLE_TARGET`
        次——**程序越长，采样点之间隔得越远**，这是"检查成本恒定"的取舍。
        要按"机器人可能移动多远"取样（不随程序变长而稀释），传
        ``max_joint_step``（度）。

        结果缓存（v1.2.2）：键由全部影响判定的输入派生（场景签名 + 逐路点
        关节值 + margin/采样口径），命中时交还同一份结论并**复现"写回场景
        关节"的副作用**，对调用方与实算路径无差别。基准/计时场景请关掉
        （``use_collision_cache=False`` 或 :func:`service.incremental.set_collision_cache(None)`，
        :mod:`service.perf` 已自动这么做）。

        Args:
            traj: 展开后的轨迹
            margin: 干涉裕度 (mm)；负值 = 包络接触容差
            stride: 采样步长（每隔几个路径点查一次）。``0`` 表示按默认口径
            max_joint_step: 关节空间采样步长（度）；相邻两个采样点之间任一
                关节的变化不超过它。``None`` 表示用默认口径，``0`` 表示逐路点

        Returns:
            :class:`~domain.world.scene.Collision` 列表，每次干涉只记首次发生

        Raises:
            ValueError: ``max_joint_step`` 为负或非有限值
        """
        scene = self.scene
        if scene is None or not traj.waypoints:
            return []

        from domain.collision import CollisionChecker, collision_cache_key, sample_indices
        from .incremental import resolve_collision_cache

        cache = resolve_collision_cache(self)
        key: Optional[bytes] = None
        if cache is not None:
            key = collision_cache_key(scene, self.robot_name, traj, margin, stride, max_joint_step)
            hit = cache.lookup(key)
            if hit is not None:
                # 副作用复现：实算路径经 check_config 把最后一个采样点的关节
                # 写回场景（"规划器沿轨迹检查依赖这一点"的历史契约）。命中
                # 路径跳过了 check_config，必须把同一份写回补上，否则紧随其
                # 后的单点位形检查会拿到过期关节
                indices = sample_indices(traj.waypoints, stride, max_joint_step)
                if indices:
                    q_last = np.asarray(traj.waypoints[indices[-1]].joints, dtype=np.float64)
                    if self.robot_name is None:
                        scene.set_robot_joints(q_last)
                    else:
                        scene.set_robot_joints(q_last, self.robot_name)
                return hit

        checker = CollisionChecker(scene, margin=margin)
        hits = checker.check_trajectory(
            traj,
            stride=stride,
            motion_at=self.motion_at,
            robot=self.robot_name,
            max_joint_step=max_joint_step,
        )
        if key is not None:
            cache.store(key, hits)
        return hits

    # ---------------------------- 内部：单条指令 ---------------------------- #

    def _fk(self, q_deg: Sequence[float]) -> Pose:
        """正解（deg → Pose）。"""
        return self.robot.forward_kinematics(np.asarray(q_deg, dtype=np.float64))

    def _ik(
        self, pose: Pose, guess_deg: np.ndarray, progress: float, is_target: bool = False
    ) -> np.ndarray:
        """逆解，失败时抛出 :class:`PlanningError`（带指令内定位信息）。

        中间插补点用较松的容差：沿路径要求 0.01mm 精度没有工程意义，却会让
        小型机器人（如 SO-101）在明明可达的目标上白白失败。示教点（每条指令
        的终点）才是必须精确到达的位置，用紧容差。

        Args:
            pose: 目标位姿
            guess_deg: 迭代初值（deg）——用上一步的解保证位形连续
            progress: 该点在指令内的进度 ``0~1``，仅用于错误信息
            is_target: 是否为该条指令的终点（示教点）

        Returns:
            关节角（deg）

        Raises:
            PlanningError: 逆解失败
        """
        try:
            return np.asarray(
                self.robot.inverse_kinematics(
                    pose,
                    guess=guess_deg,
                    pos_tol_mm=self.target_tol_mm if is_target else self.path_tol_mm,
                    rot_tol_deg=self.target_rot_tol_deg if is_target else self.path_rot_tol_deg,
                ),
                dtype=np.float64,
            ).ravel()
        except Exception as exc:
            raise PlanningError(
                tr(
                    "该段 {progress:.0f}% 处逆解失败——目标可能超出工作空间、"
                    "位于奇异位形附近，或与相邻点之间要求过大的姿态变化（{exc}）"
                ).format(progress=progress * 100.0, exc=exc)
            ) from exc

    def _speed_scale(self, motion: "Motion") -> float:
        """``speed`` 百分比 → 速度/加速度缩放系数。"""
        return float(np.clip((motion.speed or 100.0) / 100.0, 0.01, 1.0))

    def _expand_joint(
        self, q_start: np.ndarray, motion: "Motion", index: int
    ) -> Tuple[List[np.ndarray], List[Pose], float, float]:
        """moveJ：关节空间线性插补，所有轴同步启停。

        Returns:
            ``(joints_list, pose_list, dt, path_length)``，
            ``path_length`` 对 moveJ 恒为 0（TCP 不走直线，长度无意义）

        Raises:
            PlanningError: 目标关节数不符，或目标超出限位
        """
        q_end = np.asarray(motion.target, dtype=np.float64).ravel()
        if q_end.size != q_start.size:
            raise PlanningError(
                tr("moveJ 目标关节数 {n} 与机器人自由度 {dof} 不符").format(
                    n=q_end.size, dof=q_start.size
                )
            )
        # 目标越限必须报错而不是悄悄夹回来——示教点写错了就该让用户知道，
        # 夹取之后机器人走到的是另一个位置，现场对不上点位极难排查
        violation = self._limits.violation(q_end)
        if violation:
            raise PlanningError(tr("moveJ 目标{violation}").format(violation=violation))

        delta = q_end - q_start
        if not np.any(np.abs(delta) > 1e-9):
            return [], [], 0.0, 0.0

        scale = self._speed_scale(motion)
        vmax = self._limits.max_speed * scale
        amax = self._limits.max_accel * scale

        # 插补步数：按关节空间的最大单轴行程定，保证每步不超过分辨率
        step_res = np.where(
            np.array([j.is_prismatic for j in self.robot.kinematics.actuated]),
            self.linear_res,
            self.angular_res,
        )
        n = int(np.ceil(float(np.max(np.abs(delta) / step_res))))
        n = int(np.clip(n, 2, MAX_STEPS_PER_MOTION))

        total, v_s, a_s = _sync_params(np.abs(delta), vmax, amax)
        if self.profile == PROFILE_SCURVE:
            # S 曲线下同样参数走得更久——总时长必须按曲线风格计算，
            # 否则归一化进度在 total 时刻到不了 1，终点差半个身位
            total = s_curve_time(1.0, v_s, a_s)

        pts: List[np.ndarray] = []
        poses: List[Pose] = []
        for i in range(1, n + 1):
            # 时间均匀采样、空间按速度曲线推进：真实控制器即如此执行 PTP
            t_i = total * i / n
            alpha = _profile_pos(1.0, v_s, a_s, self.profile, t_i, total)
            alpha = float(np.clip(alpha, 0.0, 1.0))
            q = q_start + delta * alpha
            pts.append(q.copy())
            poses.append(self._fk(q))
        # 每步时长：整段时间均分（关节位置随速度曲线非均匀推进）
        return pts, poses, total / n, 0.0

    def _resolve_q_end(
        self, motion: "Motion", q_start: np.ndarray, index: int, is_target: bool = True
    ) -> np.ndarray:
        """把指令终点解析成关节角（deg）。

        ``target`` 是关节列表时直接采用；是 :class:`~domain.pose.Pose` 时做逆解
        （用上一步的关节作初值，保证位形连续、且能命中与相邻指令一致的配置）。

        Args:
            motion: 当前指令
            q_start: 起始关节角（逆解初值）
            index: 指令序号（仅用于错误信息）
            is_target: 是否为示教点（决定逆解容差）

        Returns:
            关节角（deg）

        Raises:
            PlanningError: 关节数不符、逆解失败或（对 moveJ 类）目标越限
        """
        from .program import MotionType

        if isinstance(motion.target, Pose):
            q = self._ik(motion.target, np.asarray(q_start), 1.0, is_target=is_target)
            return np.asarray(q, dtype=np.float64).ravel()
        try:
            q_end = np.asarray(motion.target, dtype=np.float64).ravel()
        except Exception as exc:
            # 文案里**不写指令序号**：Issue 自己会渲染 ``[指令 N]`` 前缀，写进去
            # 会重复；更要紧的是，增量重算（v1.2.0）要按"指令内容"缓存失败结果，
            # 而序号是**位置**的函数、不是这条指令的属性——写进文案就没法复用了
            raise PlanningError(
                tr("target 既不是关节列表也不是位姿：{exc}").format(exc=exc)
            ) from exc
        if q_end.size != q_start.size:
            raise PlanningError(
                tr("目标关节数 {n} 与自由度 {dof} 不符").format(n=q_end.size, dof=q_start.size)
            )
        return q_end

    def _expand_spline(
        self, q_start: np.ndarray, motion: "Motion", index: int
    ) -> Tuple[List[np.ndarray], List[Pose], float, float]:
        """moveSpline：关节空间最小抖动（5 次多项式）平滑插补，等效平滑 PTP。

        比 :meth:`_expand_joint` 的线性插补多了加减速的连续段——起点/终点速度、
        加速度均为零，适合需要柔和过渡、减少机械冲击的搬运/装配段。

        这是 SPLINE 插补的**最小可用实现**：在两端点之间做一条平滑曲线。
        若需要"穿过多个示教点的笛卡尔样条路径"，按历史实现的建议用 moveL
        密集点位近似即可（控制器也是逐段直线逼近样条）。

        Returns:
            ``(joints_list, pose_list, dt, path_length)``，
            ``path_length`` 对 moveSpline 恒为 0（关节空间运动，TCP 不走直线）
        """
        from .program import MotionType

        q_end = self._resolve_q_end(motion, q_start, index)
        # 目标越限同样必须报错，与 moveJ 一致——示教点写错了就该让用户知道，
        # 夹回后机器人走到的是别的位置，现场对不上点位极难排查
        violation = self._limits.violation(q_end)
        if violation:
            raise PlanningError(tr("moveSpline 目标{violation}").format(violation=violation))

        delta = q_end - q_start
        if not np.any(np.abs(delta) > 1e-9):
            return [], [], 0.0, 0.0

        scale = self._speed_scale(motion)

        # 步数：随最大单轴行程定（与 moveJ 同），保证每步不超过插补分辨率
        step_res = np.where(
            np.array([j.is_prismatic for j in self.robot.kinematics.actuated]),
            self.linear_res,
            self.angular_res,
        )
        n = int(np.ceil(float(np.max(np.abs(delta) / step_res))))
        n = int(np.clip(n, 2, MAX_STEPS_PER_MOTION))

        vmax = self._limits.max_speed * scale
        amax = self._limits.max_accel * scale
        total = _synchronized_time(np.abs(delta), vmax, amax)

        pts: List[np.ndarray] = []
        poses: List[Pose] = []
        for i in range(1, n + 1):
            alpha = i / n
            # 5 次最小抖动基函数：s(0)=0, s(1)=1, s'(0)=s'(1)=0, s''(0)=s''(1)=0
            s = 10.0 * alpha**3 - 15.0 * alpha**4 + 6.0 * alpha**5
            q = q_start + delta * s
            pts.append(q.copy())
            poses.append(self._fk(q))
        return pts, poses, total / n, 0.0

    def _expand_linear(
        self, q_start: np.ndarray, motion: "Motion", index: int
    ) -> Tuple[List[np.ndarray], List[Pose], float, float]:
        """moveL：笛卡尔空间密集插补 + 逐点逆解。

        这是"离线编程"与"只对终点求个逆解"的本质区别——不逐点插补就无从判断
        路径中途是否撞机、是否翻腕。
        """
        p0 = self._fk(q_start)
        p1 = motion.target if isinstance(motion.target, Pose) else Pose(np.asarray(motion.target))
        return self._walk_cartesian(q_start, p0, [p1], motion, arc=None, index=index)

    def _expand_circular(
        self, q_start: np.ndarray, motion: "Motion", index: int
    ) -> Tuple[List[np.ndarray], List[Pose], float, float]:
        """moveC：经 via 点的圆弧插补 + 逐点逆解。"""
        if motion.via is None:
            raise PlanningError(tr("moveC 缺少 via（中间点）"))
        p0 = self._fk(q_start)
        via = motion.via if isinstance(motion.via, Pose) else Pose(np.asarray(motion.via))
        p1 = motion.target if isinstance(motion.target, Pose) else Pose(np.asarray(motion.target))
        return self._walk_cartesian(q_start, p0, [p1], motion, arc=via, index=index)

    def _poses_along(
        self, p0: Pose, p1: Pose, arc: Optional[Pose], n: int
    ) -> Tuple[List[Pose], float]:
        """生成插补位姿序列与 TCP 路径长度。

        Args:
            p0: 起点
            p1: 终点
            arc: ``None`` 表示直线；否则为圆弧的 via 点
            n: 步数

        Returns:
            ``(poses, path_length_mm)``
        """
        start = np.array(p0.to_xyz_quat()[:3], dtype=np.float64)
        end = np.array(p1.to_xyz_quat()[:3], dtype=np.float64)

        if arc is None:
            length = float(np.linalg.norm(end - start))
            n_eff = max(2, int(n))
            poses = [_interpolate_pose(p0, p1, i / n_eff) for i in range(1, n_eff + 1)]
            return poses, length

        via_xyz = np.array(arc.to_xyz_quat()[:3], dtype=np.float64)
        center, radius, axis, theta = _circle_from_three(start, via_xyz, end)
        # 弧长 = r·|θ|（姿态变化量已在 _walk_cartesian 里参与步数决策）
        length = radius * abs(theta)
        n_eff = max(2, int(n))
        r0 = start - center
        poses = []
        for i in range(1, n_eff + 1):
            t = i / n_eff
            pos = center + _rodrigues(r0, axis, theta * t)
            # 位置走圆弧，姿态单独从起点 slerp 到终点（与主流控制器一致）
            frame = _interpolate_pose(p0, p1, t)
            poses.append(frame.with_position(float(pos[0]), float(pos[1]), float(pos[2])))
        return poses, length

    def _walk_cartesian(
        self,
        q_start: np.ndarray,
        p0: Pose,
        targets: List[Pose],
        motion: "Motion",
        arc: Optional[Pose],
        index: int,
    ) -> Tuple[List[np.ndarray], List[Pose], float, float]:
        """沿给定几何逐点逆解，并估算该段耗时。

        Args:
            q_start: 起始关节角 (deg)
            p0: 起始位姿
            targets: 终点序列（当前实现只用最后一个；via 已通过 arc 传入）
            motion: 原始指令，用于读取速度百分比
            arc: 圆弧 via 点；``None`` 为直线
            index: 指令序号，用于生成可定位的警告

        Returns:
            ``(joints_list, pose_list, dt, path_length)``；``dt`` 为逐点时长
            序列（速度曲线导致的非均匀时间分布）

        Raises:
            PlanningError: 逆解失败或关节越限
        """
        p1 = targets[-1]
        start_xyz = np.array(p0.to_xyz_quat()[:3], dtype=np.float64)

        # 先按几何估算长度，据此决定插补步数
        if arc is None:
            end_xyz = np.array(p1.to_xyz_quat()[:3], dtype=np.float64)
            rough_length = float(np.linalg.norm(end_xyz - start_xyz))
        else:
            via_xyz = np.array(arc.to_xyz_quat()[:3], dtype=np.float64)
            end_xyz = np.array(p1.to_xyz_quat()[:3], dtype=np.float64)
            try:
                _, radius, _, theta = _circle_from_three(start_xyz, via_xyz, end_xyz)
                rough_length = radius * abs(theta)
            except PlanningError:
                rough_length = float(np.linalg.norm(end_xyz - start_xyz))

        angle = _orientation_angle_deg(p0, p1)
        n = max(
            int(math.ceil(rough_length / max(self.linear_res, 1e-6))),
            int(math.ceil(angle / max(self.angular_res, 1e-6))),
        )
        n = int(np.clip(n, 2, MAX_STEPS_PER_MOTION))

        poses, length = self._poses_along(p0, p1, arc, n)

        scale = self._speed_scale(motion)
        tcp_v = self.tcp_speed * scale
        tcp_a = self.tcp_accel * scale
        vmax = self._limits.max_speed * scale
        amax = self._limits.max_accel * scale

        pts: List[np.ndarray] = []
        q_prev = np.asarray(q_start, dtype=np.float64)
        joint_travel = np.zeros(q_prev.size)

        for i, pose in enumerate(poses):
            progress = (i + 1) / n
            # 用上一步的解作初值：既保证位形连续（不翻腕），也显著加快收敛。
            # 最后一个插补点即示教点，按紧容差要求精确到达。
            q = self._ik(pose, q_prev, progress, is_target=(i == len(poses) - 1))
            violation = self._limits.violation(q)
            if violation:
                raise PlanningError(
                    tr(
                        "该段 {progress:.0f}% 处{violation}"
                        "——直线/圆弧插补会把关节推出限位，需调整路径或工件位置"
                    ).format(progress=progress * 100.0, violation=violation)
                )
            # 配置跳变：相邻插补点之间单轴突变，典型成因是路径穿过奇异位形
            # 导致腕部翻转。真实机器人会以极高的关节速度执行，属于危险动作。
            step = float(np.max(np.abs(q - q_prev)))
            if step > FLIP_WARN_DEG:
                worst = int(np.argmax(np.abs(q - q_prev)))
                self.issues.append(
                    Issue(
                        SEVERITY_WARNING,
                        "config_flip",
                        index,
                        tr(
                            "该段 {progress:.0f}% 处轴 {axis} 跳变 {step:.1f}°"
                            "（> {limit}°），可能穿越奇异位形发生翻腕"
                        ).format(
                            progress=progress * 100.0,
                            axis=worst + 1,
                            step=step,
                            limit=FLIP_WARN_DEG,
                        ),
                        progress=progress,
                    )
                )
            joint_travel += np.abs(q - q_prev)
            pts.append(q.copy())
            q_prev = q

        # 耗时取「TCP 约束」与「各轴约束」的较大者：
        # 关节空间的实际行程可能远大于端点差（路径弯曲时尤其明显），
        # 只按 TCP 速度估算会低估节拍。
        if self.profile == PROFILE_SCURVE:
            t_tcp = s_curve_time(length, tcp_v, tcp_a)
            t_joint = max(
                (
                    s_curve_time(float(d), float(v), float(a))
                    for d, v, a in zip(joint_travel, vmax, amax)
                ),
                default=0.0,
            )
        else:
            t_tcp = trapezoid_time(length, tcp_v, tcp_a)
            t_joint = max(
                (
                    trapezoid_time(float(d), float(v), float(a))
                    for d, v, a in zip(joint_travel, vmax, amax)
                ),
                default=0.0,
            )
        total = max(t_tcp, t_joint)
        if not pts:
            return pts, poses, [], length

        # 速度曲线的时间分配：把既有节拍公式给出的总时长按归一化速度曲线
        # 分配到各空间插补点上——总时长不变（无节拍回归），但起停有加减速
        ratio = (
            (self.tcp_accel / self.tcp_speed)
            if t_tcp >= t_joint and self.tcp_speed > 0
            else 4.0  # 轴约束主导：AxisLimits 的加速度恒为 4×速度
        )
        v_n, a_n = _normalized_params(total, ratio, self.profile)
        times = [
            _profile_time(1.0, v_n, a_n, self.profile, i / len(pts), total)
            for i in range(1, len(pts) + 1)
        ]
        dts = [times[0]] + [times[i] - times[i - 1] for i in range(1, len(times))]
        return pts, poses, dts, length

    def _audit_text(self, q_end: np.ndarray) -> Optional[str]:
        """终点审计的**文案**（不含序号与时刻）。

        拆出来是为了增量重算：序号随插入/删除而变、时刻随前缀时长而变，
        只有文案是这条指令自己的属性，可以进分段缓存。
        """
        kin = self.robot.kinematics
        if kin is None:
            return None
        try:
            J = kin.jacobian(np.radians(q_end))
            cond = float(np.linalg.cond(J))
            if np.isfinite(cond) and cond > SINGULARITY_COND:
                return tr(
                    "接近奇异位形（雅可比条件数 {cond:.2e}），"
                    "关节速度可能突变，建议调整姿态或路径"
                ).format(cond=cond)
        except Exception:
            return None
        return None

    def _audit_motion(self, index: int, q_end: np.ndarray, t: float) -> None:
        """对指令终点做奇异性与配置跳变检查（警告级）。"""
        text = self._audit_text(q_end)
        if text:
            self.issues.append(Issue(SEVERITY_WARNING, "singularity", index, text, time=t))
