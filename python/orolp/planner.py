"""离线编程核心：轨迹展开、节拍估算与程序校验。

这是 OLP 与「能画个机器人」之间的分水岭。一个 :class:`~orolp.program.Program`
只有若干条 :class:`~orolp.program.Motion`（目标点），本机模块负责把它变成
**控制器真正会走的那条路径**：

* :meth:`TrajectoryPlanner.plan` —— 逐条展开成密集的、带时间戳的路径点
* :meth:`TrajectoryPlanner.validate` —— 在生成代码**之前**把不可达、超限位、
  奇异位形、配置跳变、干涉全部报出来

单位约定
--------
对外一律 **毫米 + 度 + 秒**（与 :class:`~orolp.pose.Pose` 一致）；
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

from .pose import Pose
from .simulation import Trajectory, Waypoint, _interpolate_pose

if TYPE_CHECKING:  # 仅类型标注，避免运行时循环导入
    from .program import Motion, Program
    from .robot import Robot
    from .world.scene import Collision, Scene

__all__ = [
    "PlanningError",
    "Issue",
    "ProgramReport",
    "AxisLimits",
    "TrajectoryPlanner",
    "trapezoid_time",
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


def _synchronized_time(
    deltas: np.ndarray, vmax: np.ndarray, amax: np.ndarray
) -> float:
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
    nonzero = deltas > 1e-12
    if not np.any(nonzero):
        return 0.0
    d = deltas[nonzero]
    v = np.min(vmax[nonzero] / d)
    a = np.min(amax[nonzero] / d)
    return trapezoid_time(1.0, v, a)


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
            kin: :class:`~orolp.kinematics.RobotKinematics` 实例

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
            [DEFAULT_JOINT_SPEED_DEG if not j.is_prismatic else 100.0
             for j in kin.actuated],
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
                return f"轴 {i + 1} 低于下限位：{v:.3f} < {lo:.3f}"
            if np.isfinite(hi) and v > hi + LIMIT_TOL_DEG:
                return f"轴 {i + 1} 超出上限位：{v:.3f} > {hi:.3f}"
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
        where = f"指令 {self.motion_index + 1}"
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
        head = "✓ 程序校验通过" if self.ok else f"✗ 程序校验未通过（{len(self.errors)} 处错误）"
        lines.append(head)
        lines.append(
            f"  指令 {self.num_motions} 条 / 路径点 {self.num_waypoints} 个 / "
            f"TCP 路径 {self.path_length:.1f} mm"
        )
        if self.cycle_time is not None:
            lines.append(f"  估算节拍 {self.cycle_time:.2f} s")
        if self.warnings:
            lines.append(f"  警告 {len(self.warnings)} 处：")
            lines.extend(f"    {w}" for w in self.warnings)
        if self.errors:
            lines.append(f"  错误 {len(self.errors)} 处：")
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
            "圆弧的起点、中间点、终点（近）共线，无法确定圆弧；"
            "请检查 via 点是否落在起终点的连线上"
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
    """把 :class:`~orolp.program.Program` 展开为可执行轨迹。

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
    ):
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
        self.issues: List[Issue] = []
        self._limits = AxisLimits.from_kinematics(robot.kinematics)
        # 每条指令在轨迹时间轴上的区间 [(t_start, t_end), ...]，用于把干涉
        # 定位回具体指令；plan() 每次重建
        self._motion_spans: List[Tuple[float, float]] = []
        self._last_path_length = 0.0

    # ---------------------------- 对外接口 ---------------------------- #

    @property
    def limits(self) -> AxisLimits:
        """机器人轴参数。"""
        return self._limits

    def plan(self, program: "Program") -> Trajectory:
        """展开程序为密集轨迹。

        逐条指令插补，失败（不可达 / 越限 / 几何退化）时记录错误并**跳过该条
        指令的剩余部分**——不假装它走完了。

        Args:
            program: 待展开的程序

        Returns:
            带时间戳的 :class:`~orolp.simulation.Trajectory`
        """
        from .program import MotionType

        self.issues = []
        waypoints: List[Waypoint] = []
        t_now = 0.0
        path_length = 0.0
        self._motion_spans = []

        # 起点：机器人当前关节角
        q_cur = np.asarray(self.robot.get_joints(), dtype=np.float64).ravel()
        if q_cur.size != self.robot.dof:
            q_cur = np.zeros(self.robot.dof)
        q_cur = self._limits.clamp(q_cur)

        waypoints.append(
            Waypoint(joints=q_cur.copy(), pose=self._fk(q_cur), timestamp=t_now)
        )

        for idx, motion in enumerate(program.motions):
            try:
                if motion.type == MotionType.JOINTS:
                    pts, poses, dt, plen = self._expand_joint(q_cur, motion, idx)
                elif motion.type == MotionType.LINEAR:
                    pts, poses, dt, plen = self._expand_linear(q_cur, motion, idx)
                elif motion.type == MotionType.CIRCULAR:
                    pts, poses, dt, plen = self._expand_circular(q_cur, motion, idx)
                elif motion.type == MotionType.SPLINE:
                    raise PlanningError(
                        "SPLINE 插补尚未实现；请改用 moveL 密集点位，"
                        "或先拆分为多段直线/圆弧"
                    )
                else:
                    raise PlanningError(f"未知的指令类型：{motion.type!r}")
            except PlanningError as exc:
                self.issues.append(
                    Issue(SEVERITY_ERROR, "planning", idx, str(exc))
                )
                continue
            except Exception as exc:  # IK 等底层异常统一收敛为可报告的问题
                self.issues.append(
                    Issue(SEVERITY_ERROR, "planning", idx, f"{type(exc).__name__}: {exc}")
                )
                continue

            if not pts:
                self._motion_spans.append((t_now, t_now))
                continue

            t_start = t_now
            for q, pose in zip(pts, poses):
                t_now += dt
                waypoints.append(
                    Waypoint(joints=np.asarray(q), pose=pose, timestamp=t_now)
                )
            self._motion_spans.append((t_start, t_now))
            q_cur = np.asarray(pts[-1], dtype=np.float64)
            path_length += plen

            self._audit_motion(idx, q_cur, t_now)

        traj = Trajectory(waypoints)
        self._last_path_length = path_length
        return traj

    def motion_at(self, t: float) -> int:
        """查询某个时刻属于第几条指令。

        Args:
            t: 时间（秒，相对轨迹起点）

        Returns:
            指令序号（从 0 计）；找不到时返回 ``-1``
        """
        for i, (t0, t1) in enumerate(self._motion_spans):
            if t0 <= t <= t1:
                return i
        return len(self._motion_spans) - 1

    def validate(
        self,
        program: "Program",
        check_collision: bool = True,
        margin: float = -0.5,
    ) -> ProgramReport:
        """在生成代码前把程序审一遍。

        Args:
            program: 待校验的程序
            check_collision: 是否做沿路径干涉检查（需要构造时给了 ``scene``）
            margin: 干涉裕度 (mm)。负值容忍轻微接触——工件放在台面上、
                夹具夹住工件都是零间隙接触，把接触当干涉会满屏误报

        Returns:
            :class:`ProgramReport`
        """
        traj = self.plan(program)
        report = ProgramReport(
            issues=list(self.issues),
            cycle_time=traj.duration(),
            path_length=getattr(self, "_last_path_length", 0.0),
            num_motions=len(program.motions),
            num_waypoints=len(traj.waypoints),
        )

        if check_collision:
            if self.scene is None:
                report.issues.append(
                    Issue(
                        SEVERITY_WARNING,
                        "no_scene",
                        0,
                        "未传入工作单元场景，跳过干涉检查",
                    )
                )
            else:
                hits = self.check_trajectory_collisions(traj, margin=margin)
                for hit in hits:
                    report.issues.append(
                        Issue(
                            SEVERITY_ERROR,
                            "collision",
                            self.motion_at(hit.time) if hit.time is not None else 0,
                            f"干涉：{hit.obj_a} ↔ {hit.obj_b}",
                            time=hit.time,
                        )
                    )
                traj.collisions = hits
        return report

    def check_trajectory_collisions(
        self, traj: Trajectory, margin: float = -0.5, stride: int = 0
    ) -> List["Collision"]:
        """沿整条轨迹做干涉检查。

        不只是检查路径点本身——两点之间机器人可能已经撞上了，所以必须密集采样。

        Args:
            traj: 展开后的轨迹
            margin: 干涉裕度 (mm)
            stride: 采样步长（每隔几个路径点查一次）。``0`` 表示自动：
                按轨迹长度控制在约 300 次检查以内

        Returns:
            :class:`~orolp.world.scene.Collision` 列表，每次干涉只记首次发生
        """
        scene = self.scene
        if scene is None or not traj.waypoints:
            return []

        n = len(traj.waypoints)
        if stride <= 0:
            stride = max(1, n // 300)

        seen: Dict[Tuple[str, str], "Collision"] = {}

        def _collect(t: float) -> None:
            for hit in scene.check_collisions(include_robot=True, margin=margin):
                key = (hit.obj_a, hit.obj_b)
                if key not in seen:
                    # Collision 是 frozen dataclass，不能就地改；用 replace 生成
                    # 带时刻的副本。（早先版本直接 ``hit.time = ...``，抛出的
                    # FrozenInstanceError 被 except 吞掉，导致时刻永远是 None）
                    seen[key] = replace(hit, time=t)

        for i in range(0, n, stride):
            wp = traj.waypoints[i]
            scene.set_robot_joints(np.asarray(wp.joints, dtype=np.float64))
            _collect(float(wp.timestamp))
        # 末点必查，避免 stride 刚好跳过收尾位姿
        last = traj.waypoints[-1]
        scene.set_robot_joints(np.asarray(last.joints, dtype=np.float64))
        _collect(float(last.timestamp))
        return list(seen.values())

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
                f"该段 {progress * 100:.0f}% 处逆解失败——目标可能超出工作空间、"
                f"位于奇异位形附近，或与相邻点之间要求过大的姿态变化（{exc}）"
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
                f"moveJ 目标关节数 {q_end.size} 与机器人自由度 {q_start.size} 不符"
            )
        # 目标越限必须报错而不是悄悄夹回来——示教点写错了就该让用户知道，
        # 夹取之后机器人走到的是另一个位置，现场对不上点位极难排查
        violation = self._limits.violation(q_end)
        if violation:
            raise PlanningError(f"moveJ 目标{violation}")

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

        total = _synchronized_time(np.abs(delta), vmax, amax)

        pts: List[np.ndarray] = []
        poses: List[Pose] = []
        for i in range(1, n + 1):
            alpha = i / n
            q = q_start + delta * alpha
            pts.append(q.copy())
            poses.append(self._fk(q))
        # 每步时长：整段时间均分（关节空间线性插补，控制器即如此）
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
            raise PlanningError("moveC 缺少 via（中间点）")
        p0 = self._fk(q_start)
        via = motion.via if isinstance(motion.via, Pose) else Pose(np.asarray(motion.via))
        p1 = motion.target if isinstance(motion.target, Pose) else Pose(np.asarray(motion.target))
        return self._walk_cartesian(q_start, p0, [p1], motion, arc=via, index=index)

    def _poses_along(self, p0: Pose, p1: Pose, arc: Optional[Pose], n: int) -> Tuple[List[Pose], float]:
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
            ``(joints_list, pose_list, dt, path_length)``

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
                    f"该段 {progress * 100:.0f}% 处{violation}——"
                    f"直线/圆弧插补会把关节推出限位，需调整路径或工件位置"
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
                        f"该段 {progress * 100:.0f}% 处轴 {worst + 1} 跳变 "
                        f"{step:.1f}°（> {FLIP_WARN_DEG}°），可能穿越奇异位形发生翻腕",
                        progress=progress,
                    )
                )
            joint_travel += np.abs(q - q_prev)
            pts.append(q.copy())
            q_prev = q

        # 耗时取「TCP 约束」与「各轴约束」的较大者：
        # 关节空间的实际行程可能远大于端点差（路径弯曲时尤其明显），
        # 只按 TCP 速度估算会低估节拍。
        t_tcp = trapezoid_time(length, tcp_v, tcp_a)
        t_joint = max(
            (trapezoid_time(float(d), float(v), float(a))
             for d, v, a in zip(joint_travel, vmax, amax)),
            default=0.0,
        )
        total = max(t_tcp, t_joint)
        return pts, poses, total / max(len(pts), 1), length

    def _audit_motion(self, index: int, q_end: np.ndarray, t: float) -> None:
        """对指令终点做奇异性与配置跳变检查（警告级）。"""
        kin = self.robot.kinematics
        if kin is None:
            return
        try:
            J = kin.jacobian(np.radians(q_end))
            cond = float(np.linalg.cond(J))
            if np.isfinite(cond) and cond > SINGULARITY_COND:
                self.issues.append(
                    Issue(
                        SEVERITY_WARNING,
                        "singularity",
                        index,
                        f"接近奇异位形（雅可比条件数 {cond:.2e}），"
                        f"关节速度可能突变，建议调整姿态或路径",
                        time=t,
                    )
                )
        except Exception:
            pass
