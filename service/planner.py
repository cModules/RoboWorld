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

from typing import TYPE_CHECKING, List, Optional, Sequence, Tuple

import numpy as np

from support.i18n import tr
from domain.pose import Pose
from .planner_types import (
    DEFAULT_ANGULAR_RES, DEFAULT_LINEAR_RES, DEFAULT_TCP_ACCEL, DEFAULT_TCP_SPEED,
    ENVELOPE_TOLERANCE_MM, AxisLimits, Issue, PlanningError, ProgramReport,
    SEVERITY_ERROR, SEVERITY_WARNING,
)
from .planner_math import (
    PROFILE_SCURVE, PROFILE_TRAPEZOID, VALID_PROFILES, _synchronized_time,
    s_curve_time, trapezoid_time,
)
from .planner_expand import SINGULARITY_COND
from . import planner_dynamics, planner_expand
from .simulation import Trajectory, Waypoint

if TYPE_CHECKING:  # 仅类型标注，避免运行时循环导入
    from domain.world.scene import Collision, Scene
    from .program import Motion, Program
    from .incremental import SegmentPayload
    from domain.robot import Robot

__all__ = [
    "PlanningError", "Issue", "ProgramReport", "AxisLimits", "TrajectoryPlanner",
    "trapezoid_time", "s_curve_time", "PROFILE_TRAPEZOID", "PROFILE_SCURVE",
    "ENVELOPE_TOLERANCE_MM",
]


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
    def _expand_segment(self, q_start: np.ndarray, motion: "Motion", index: int) -> "SegmentPayload":
        return planner_expand._expand_segment(self, q_start, motion, index)

    def _populate_dynamics(self, traj: "Trajectory") -> None:
        return planner_dynamics._populate_dynamics(self, traj)

    def _span_index(self) -> Tuple[np.ndarray, np.ndarray]:
        return planner_dynamics._span_index(self)

    def motion_at(self, t: float) -> int:
        return planner_expand.motion_at(self, t)

    def _check_dynamics(self, traj: Trajectory, report: ProgramReport) -> None:
        return planner_dynamics._check_dynamics(self, traj, report)

    def _apply_dynamics_result(self, report: ProgramReport, result: dict) -> None:
        return planner_dynamics._apply_dynamics_result(self, report, result)

    def check_trajectory_collisions(
        self, traj: Trajectory, margin: float = -0.5, stride: int = 0,
        max_joint_step: Optional[float] = None,
    ) -> List["Collision"]:
        return planner_dynamics.check_trajectory_collisions(
            self, traj, margin=margin, stride=stride, max_joint_step=max_joint_step
        )

    def _fk(self, q_deg: Sequence[float]) -> Pose:
        return planner_expand._fk(self, q_deg)

    def _ik(
        self, pose: Pose, guess_deg: np.ndarray, progress: float, is_target: bool = False
    ) -> np.ndarray:
        return planner_expand._ik(self, pose, guess_deg, progress, is_target)

    def _speed_scale(self, motion: "Motion") -> float:
        return planner_expand._speed_scale(self, motion)

    def _expand_joint(
        self, q_start: np.ndarray, motion: "Motion", index: int
    ) -> Tuple[List[np.ndarray], List[Pose], float, float]:
        return planner_expand._expand_joint(self, q_start, motion, index)

    def _resolve_q_end(
        self, motion: "Motion", q_start: np.ndarray, index: int, is_target: bool = True
    ) -> np.ndarray:
        return planner_expand._resolve_q_end(self, motion, q_start, index, is_target)

    def _expand_spline(
        self, q_start: np.ndarray, motion: "Motion", index: int
    ) -> Tuple[List[np.ndarray], List[Pose], float, float]:
        return planner_expand._expand_spline(self, q_start, motion, index)

    def _expand_linear(
        self, q_start: np.ndarray, motion: "Motion", index: int
    ) -> Tuple[List[np.ndarray], List[Pose], float, float]:
        return planner_expand._expand_linear(self, q_start, motion, index)

    def _expand_circular(
        self, q_start: np.ndarray, motion: "Motion", index: int
    ) -> Tuple[List[np.ndarray], List[Pose], float, float]:
        return planner_expand._expand_circular(self, q_start, motion, index)

    def _poses_along(
        self, p0: Pose, p1: Pose, arc: Optional[Pose], n: int
    ) -> Tuple[List[Pose], float]:
        return planner_expand._poses_along(self, p0, p1, arc, n)

    def _walk_cartesian(
        self, q_start: np.ndarray, p0: Pose, targets: List[Pose], motion: "Motion",
        arc: Optional[Pose], index: int,
    ) -> Tuple[List[np.ndarray], List[Pose], float, float]:
        return planner_expand._walk_cartesian(self, q_start, p0, targets, motion, arc, index)

    def _audit_text(self, q_end: np.ndarray) -> Optional[str]:
        return planner_expand._audit_text(self, q_end)

    def _audit_motion(self, index: int, q_end: np.ndarray, t: float) -> None:
        return planner_expand._audit_motion(self, index, q_end, t)
