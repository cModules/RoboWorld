"""速度规划（梯形/S 曲线）与几何工具：纯函数，不依赖规划器实例。

从 planner.py 拆出（纯搬运）；公开函数由 :mod:`service.planner` re-export。
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np

from support.i18n import tr
from domain.pose import Pose
from .planner_types import PlanningError

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
