"""规划器的单条指令展开段（从 planner.py 拆出，纯搬运）。

模块级函数的首参 ``planner`` 即 :class:`~service.planner.TrajectoryPlanner`
实例；主类保留同名薄 stub 转发，外部调用面不变。
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, List, Optional, Sequence, Tuple

import numpy as np

from support.i18n import tr
from domain.pose import Pose
from .planner_math import (
    PROFILE_SCURVE,
    _circle_from_three,
    _normalized_params,
    _orientation_angle_deg,
    _profile_pos,
    _profile_time,
    _rodrigues,
    _sync_params,
    _synchronized_time,
    s_curve_time,
    trapezoid_time,
)
from .planner_types import Issue, PlanningError, SEVERITY_WARNING
from .simulation import _interpolate_pose

if TYPE_CHECKING:  # 仅类型标注，避免运行时循环导入
    from .planner import TrajectoryPlanner
    from .program import Motion

    # _expand_segment 的返回值：incremental 反向依赖 planner 的签名，
    # 运行时不能互相 import，注解只好靠这里借名字（此前是裸字符串，
    # pyflakes 的 F821 说明它根本没被解析过）
    from .incremental import SegmentPayload

#: 单条指令最多插补步数，防止病态输入把内存和耗时拖爆
MAX_STEPS_PER_MOTION = 4000
#: 相邻插补点单轴跳变超过该角度即判定为「配置跳变」（多半是过奇异点翻腕）
FLIP_WARN_DEG = 25.0
#: 雅可比条件数超过该值即判定为接近奇异位形
SINGULARITY_COND = 1e4


def _expand_segment(
    planner: "TrajectoryPlanner", q_start: np.ndarray, motion: "Motion", index: int
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
            pts, poses, dt, plen = _expand_joint(planner, q_start, motion, index)
        elif motion.type == MotionType.LINEAR:
            pts, poses, dt, plen = _expand_linear(planner, q_start, motion, index)
        elif motion.type == MotionType.CIRCULAR:
            pts, poses, dt, plen = _expand_circular(planner, q_start, motion, index)
        elif motion.type == MotionType.SPLINE:
            pts, poses, dt, plen = _expand_spline(planner, q_start, motion, index)
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
        warn=_audit_text(planner, q_end),
    )


def motion_at(planner: "TrajectoryPlanner", t: float) -> int:
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
    spans = planner._motion_spans
    if not spans:
        return -1
    starts, ends = planner._span_index()
    i = int(np.searchsorted(ends, float(t), side="left"))
    if i >= len(spans) or starts[i] > float(t):
        return len(spans) - 1
    return i


# ---------------------------- 内部：单条指令 ----------------------------


def _fk(planner: "TrajectoryPlanner", q_deg: Sequence[float]) -> Pose:
    """正解（deg → Pose）。"""
    return planner.robot.forward_kinematics(np.asarray(q_deg, dtype=np.float64))


def _ik(
    planner: "TrajectoryPlanner",
    pose: Pose,
    guess_deg: np.ndarray,
    progress: float,
    is_target: bool = False,
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
            planner.robot.inverse_kinematics(
                pose,
                guess=guess_deg,
                pos_tol_mm=planner.target_tol_mm if is_target else planner.path_tol_mm,
                rot_tol_deg=planner.target_rot_tol_deg if is_target else planner.path_rot_tol_deg,
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


def _speed_scale(planner: "TrajectoryPlanner", motion: "Motion") -> float:
    """``speed`` 百分比 → 速度/加速度缩放系数。"""
    return float(np.clip((motion.speed or 100.0) / 100.0, 0.01, 1.0))


def _expand_joint(
    planner: "TrajectoryPlanner", q_start: np.ndarray, motion: "Motion", index: int
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
    violation = planner._limits.violation(q_end)
    if violation:
        raise PlanningError(tr("moveJ 目标{violation}").format(violation=violation))

    delta = q_end - q_start
    if not np.any(np.abs(delta) > 1e-9):
        return [], [], 0.0, 0.0

    scale = _speed_scale(planner, motion)
    vmax = planner._limits.max_speed * scale
    amax = planner._limits.max_accel * scale

    # 插补步数：按关节空间的最大单轴行程定，保证每步不超过分辨率
    step_res = np.where(
        np.array([j.is_prismatic for j in planner.robot.kinematics.actuated]),
        planner.linear_res,
        planner.angular_res,
    )
    n = int(np.ceil(float(np.max(np.abs(delta) / step_res))))
    n = int(np.clip(n, 2, MAX_STEPS_PER_MOTION))

    total, v_s, a_s = _sync_params(np.abs(delta), vmax, amax)
    if planner.profile == PROFILE_SCURVE:
        # S 曲线下同样参数走得更久——总时长必须按曲线风格计算，
        # 否则归一化进度在 total 时刻到不了 1，终点差半个身位
        total = s_curve_time(1.0, v_s, a_s)

    pts: List[np.ndarray] = []
    poses: List[Pose] = []
    for i in range(1, n + 1):
        # 时间均匀采样、空间按速度曲线推进：真实控制器即如此执行 PTP
        t_i = total * i / n
        alpha = _profile_pos(1.0, v_s, a_s, planner.profile, t_i, total)
        alpha = float(np.clip(alpha, 0.0, 1.0))
        q = q_start + delta * alpha
        pts.append(q.copy())
        poses.append(_fk(planner, q))
    # 每步时长：整段时间均分（关节位置随速度曲线非均匀推进）
    return pts, poses, total / n, 0.0


def _resolve_q_end(
    planner: "TrajectoryPlanner",
    motion: "Motion",
    q_start: np.ndarray,
    index: int,
    is_target: bool = True,
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
        q = _ik(planner, motion.target, np.asarray(q_start), 1.0, is_target=is_target)
        return np.asarray(q, dtype=np.float64).ravel()
    try:
        q_end = np.asarray(motion.target, dtype=np.float64).ravel()
    except Exception as exc:
        # 文案里**不写指令序号**：Issue 自己会渲染 ``[指令 N]`` 前缀，写进去
        # 会重复；更要紧的是，增量重算（v1.2.0）要按"指令内容"缓存失败结果，
        # 而序号是**位置**的函数、不是这条指令的属性——写进文案就没法复用了
        raise PlanningError(tr("target 既不是关节列表也不是位姿：{exc}").format(exc=exc)) from exc
    if q_end.size != q_start.size:
        raise PlanningError(
            tr("目标关节数 {n} 与自由度 {dof} 不符").format(n=q_end.size, dof=q_start.size)
        )
    return q_end


def _expand_spline(
    planner: "TrajectoryPlanner", q_start: np.ndarray, motion: "Motion", index: int
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

    q_end = _resolve_q_end(planner, motion, q_start, index)
    # 目标越限同样必须报错，与 moveJ 一致——示教点写错了就该让用户知道，
    # 夹回后机器人走到的是别的位置，现场对不上点位极难排查
    violation = planner._limits.violation(q_end)
    if violation:
        raise PlanningError(tr("moveSpline 目标{violation}").format(violation=violation))

    delta = q_end - q_start
    if not np.any(np.abs(delta) > 1e-9):
        return [], [], 0.0, 0.0

    scale = _speed_scale(planner, motion)

    # 步数：随最大单轴行程定（与 moveJ 同），保证每步不超过插补分辨率
    step_res = np.where(
        np.array([j.is_prismatic for j in planner.robot.kinematics.actuated]),
        planner.linear_res,
        planner.angular_res,
    )
    n = int(np.ceil(float(np.max(np.abs(delta) / step_res))))
    n = int(np.clip(n, 2, MAX_STEPS_PER_MOTION))

    vmax = planner._limits.max_speed * scale
    amax = planner._limits.max_accel * scale
    total = _synchronized_time(np.abs(delta), vmax, amax)

    pts: List[np.ndarray] = []
    poses: List[Pose] = []
    for i in range(1, n + 1):
        alpha = i / n
        # 5 次最小抖动基函数：s(0)=0, s(1)=1, s'(0)=s'(1)=0, s''(0)=s''(1)=0
        s = 10.0 * alpha**3 - 15.0 * alpha**4 + 6.0 * alpha**5
        q = q_start + delta * s
        pts.append(q.copy())
        poses.append(_fk(planner, q))
    return pts, poses, total / n, 0.0


def _expand_linear(
    planner: "TrajectoryPlanner", q_start: np.ndarray, motion: "Motion", index: int
) -> Tuple[List[np.ndarray], List[Pose], float, float]:
    """moveL：笛卡尔空间密集插补 + 逐点逆解。

    这是"离线编程"与"只对终点求个逆解"的本质区别——不逐点插补就无从判断
    路径中途是否撞机、是否翻腕。
    """
    p0 = _fk(planner, q_start)
    p1 = motion.target if isinstance(motion.target, Pose) else Pose(np.asarray(motion.target))
    return _walk_cartesian(planner, q_start, p0, [p1], motion, arc=None, index=index)


def _expand_circular(
    planner: "TrajectoryPlanner", q_start: np.ndarray, motion: "Motion", index: int
) -> Tuple[List[np.ndarray], List[Pose], float, float]:
    """moveC：经 via 点的圆弧插补 + 逐点逆解。"""
    if motion.via is None:
        raise PlanningError(tr("moveC 缺少 via（中间点）"))
    p0 = _fk(planner, q_start)
    via = motion.via if isinstance(motion.via, Pose) else Pose(np.asarray(motion.via))
    p1 = motion.target if isinstance(motion.target, Pose) else Pose(np.asarray(motion.target))
    return _walk_cartesian(planner, q_start, p0, [p1], motion, arc=via, index=index)


def _poses_along(
    planner: "TrajectoryPlanner", p0: Pose, p1: Pose, arc: Optional[Pose], n: int
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
    planner: "TrajectoryPlanner",
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
        int(math.ceil(rough_length / max(planner.linear_res, 1e-6))),
        int(math.ceil(angle / max(planner.angular_res, 1e-6))),
    )
    n = int(np.clip(n, 2, MAX_STEPS_PER_MOTION))

    poses, length = _poses_along(planner, p0, p1, arc, n)

    scale = _speed_scale(planner, motion)
    tcp_v = planner.tcp_speed * scale
    tcp_a = planner.tcp_accel * scale
    vmax = planner._limits.max_speed * scale
    amax = planner._limits.max_accel * scale

    pts: List[np.ndarray] = []
    q_prev = np.asarray(q_start, dtype=np.float64)
    joint_travel = np.zeros(q_prev.size)

    for i, pose in enumerate(poses):
        progress = (i + 1) / n
        # 用上一步的解作初值：既保证位形连续（不翻腕），也显著加快收敛。
        # 最后一个插补点即示教点，按紧容差要求精确到达。
        q = _ik(planner, pose, q_prev, progress, is_target=(i == len(poses) - 1))
        violation = planner._limits.violation(q)
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
            planner.issues.append(
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
    if planner.profile == PROFILE_SCURVE:
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
        (planner.tcp_accel / planner.tcp_speed)
        if t_tcp >= t_joint and planner.tcp_speed > 0
        else 4.0  # 轴约束主导：AxisLimits 的加速度恒为 4×速度
    )
    v_n, a_n = _normalized_params(total, ratio, planner.profile)
    times = [
        _profile_time(1.0, v_n, a_n, planner.profile, i / len(pts), total)
        for i in range(1, len(pts) + 1)
    ]
    dts = [times[0]] + [times[i] - times[i - 1] for i in range(1, len(times))]
    return pts, poses, dts, length


def _audit_text(planner: "TrajectoryPlanner", q_end: np.ndarray) -> Optional[str]:
    """终点审计的**文案**（不含序号与时刻）。

    拆出来是为了增量重算：序号随插入/删除而变、时刻随前缀时长而变，
    只有文案是这条指令自己的属性，可以进分段缓存。
    """
    kin = planner.robot.kinematics
    if kin is None:
        return None
    try:
        J = kin.jacobian(np.radians(q_end))
        cond = float(np.linalg.cond(J))
        if np.isfinite(cond) and cond > SINGULARITY_COND:
            return tr(
                "接近奇异位形（雅可比条件数 {cond:.2e}），" "关节速度可能突变，建议调整姿态或路径"
            ).format(cond=cond)
    except Exception:
        return None
    return None


def _audit_motion(planner: "TrajectoryPlanner", index: int, q_end: np.ndarray, t: float) -> None:
    """对指令终点做奇异性与配置跳变检查（警告级）。"""
    text = _audit_text(planner, q_end)
    if text:
        planner.issues.append(Issue(SEVERITY_WARNING, "singularity", index, text, time=t))
