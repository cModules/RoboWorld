"""规划器的动力学与干涉检查段（从 planner.py 拆出，纯搬运）。

模块级函数的首参 ``planner`` 即 :class:`~service.planner.TrajectoryPlanner`
实例；主类保留同名薄 stub 转发，外部调用面不变。
"""

from __future__ import annotations

from typing import TYPE_CHECKING, List, Optional, Tuple

import numpy as np

from support.i18n import tr
from .planner_types import Issue, SEVERITY_ERROR, SEVERITY_WARNING

if TYPE_CHECKING:  # 仅类型标注，避免运行时循环导入
    from domain.world.scene import Collision
    from .planner import TrajectoryPlanner
    from .planner_types import ProgramReport
    from .simulation import Trajectory


def _populate_dynamics(planner: "TrajectoryPlanner", traj: "Trajectory") -> None:
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
            planner._limits.max_speed > 0, planner._limits.max_speed, np.inf
        )
        worst = int(np.argmax(ratio))
        if ratio[worst] > tol:
            seg = planner.motion_at(float(ts[i]))
            if seg in reported:
                continue
            reported.add(seg)
            planner.issues.append(
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
                        limit=float(planner._limits.max_speed[worst]),
                        ratio=ratio[worst] * 100.0,
                    ),
                    time=float(ts[i]),
                )
            )


def _span_index(planner: "TrajectoryPlanner") -> Tuple[np.ndarray, np.ndarray]:
    """指令区间时刻表的 ``(starts, ends)`` 数组（供二分查询，惰性重建）。

    ``_motion_spans`` 只由 :meth:`plan` 追加写入，因此 ``(长度, 末元素)``
    就是可靠的失效键——长度变了或最后一段变了就重建。
    """
    spans = planner._motion_spans
    key = (len(spans), spans[-1] if spans else None)
    if planner._span_cache_key != key:
        planner._span_cache_key = key
        planner._span_cache = (
            np.array([s[0] for s in spans], dtype=np.float64),
            np.array([s[1] for s in spans], dtype=np.float64),
        )
    return planner._span_cache


def _check_dynamics(planner: "TrajectoryPlanner", traj: Trajectory, report: ProgramReport) -> None:
    """力矩/功率校验：逐路点逆动力学，超额定报错、逼近额定预警。

    结果同时写进报表的 ``torque_peaks`` 等字段，供 :meth:`ProgramReport.format`
    打印力矩报表。缺惯性参数（URDF 没写 ``<inertial>`` 且未手动录入）
    时只挂一条跳过警告——力矩校验是增强项，不该阻塞其余校验。

    结果缓存（v1.2.3）：键由模型内容签名（连杆惯性/末端负载/额定力矩）+
    逐路点关节位置/速度/加速度 + 指令时间区间派生——改负载、改惯性、
    改程序都换键；命中时走与实算同一条"应用到报表"路径，结论逐字段一致。
    """
    model = planner.dynamics_model
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

    cache = resolve_dynamics_cache(planner)
    key: Optional[bytes] = None
    if cache is not None:
        key = dynamics_cache_key(model, traj, planner._motion_spans)
        cached = cache.lookup(key)
        if cached is not None:
            _apply_dynamics_result(planner, report, cached)
            return

    result = model.check_trajectory(traj, motion_at=planner.motion_at)
    if key is not None:
        cache.store(key, result)
    _apply_dynamics_result(planner, report, result)


def _apply_dynamics_result(
    planner: "TrajectoryPlanner", report: ProgramReport, result: dict
) -> None:
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
    planner: "TrajectoryPlanner",
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
    scene = planner.scene
    if scene is None or not traj.waypoints:
        return []

    from domain.collision import CollisionChecker, collision_cache_key, sample_indices
    from .incremental import resolve_collision_cache

    cache = resolve_collision_cache(planner)
    key: Optional[bytes] = None
    if cache is not None:
        key = collision_cache_key(scene, planner.robot_name, traj, margin, stride, max_joint_step)
        hit = cache.lookup(key)
        if hit is not None:
            # 副作用复现：实算路径经 check_config 把最后一个采样点的关节
            # 写回场景（"规划器沿轨迹检查依赖这一点"的历史契约）。命中
            # 路径跳过了 check_config，必须把同一份写回补上，否则紧随其
            # 后的单点位形检查会拿到过期关节
            indices = sample_indices(traj.waypoints, stride, max_joint_step)
            if indices:
                q_last = np.asarray(traj.waypoints[indices[-1]].joints, dtype=np.float64)
                if planner.robot_name is None:
                    scene.set_robot_joints(q_last)
                else:
                    scene.set_robot_joints(q_last, planner.robot_name)
            return hit

    checker = CollisionChecker(scene, margin=margin)
    hits = checker.check_trajectory(
        traj,
        stride=stride,
        motion_at=planner.motion_at,
        robot=planner.robot_name,
        max_joint_step=max_joint_step,
    )
    if key is not None:
        cache.store(key, hits)
    return hits
