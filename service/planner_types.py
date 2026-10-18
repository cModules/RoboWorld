"""轨迹规划的数据类型：异常、轴限、校验问题与报告（从 planner.py 拆出，纯搬运）。

归口 :mod:`service.planner` 统一 re-export，外部用法不变。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

from support.i18n import tr


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
