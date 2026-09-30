"""多机器人协同（v1.0.1）：共享场景、互避碰、节拍统筹。

为什么需要单独一层
------------------
两台机器人各跑各的程序，单看每一台都"校验通过"，但**放在一起**就可能撞：
一台在取件时另一台正伸过同一片空间，或者一台待机位形恰好挡住另一台的
路径。这类问题在单机器人视角下不可能发现——它只存在于**时间轴**上。

本模块把 N 条轨迹放到同一条时间轴上（每条带自己的起始偏移），按固定
步长采样，检查：

1. **机器人-机器人干涉**（``kind="robot-robot"``）——任意两台同时刻的
   连杆几何是否相交，附带最小间距与最近连杆对；
2. **机器人-环境干涉**（``kind="robot-env"``）——机器人运行或**待机**时
   是否压到输送带/料箱等环境几何。待机不再"隐形"：A 在等 B 的时候，
   A 就停在它轨迹的两端位形上，那也是一个真实位姿。

节拍统筹
--------
:func:`schedule_cell` 支持四种策略：

``parallel``
    全部从 0 开始——最激进，也最容易暴露冲突（用于体检）。
``serial``
    依次串行（后一台等前一台跑完）——一定不冲突？**不一定**：待机的机器人
    仍可能挡住别人的路径，所以依旧要检查。
``stagger``
    调用方给定各机器人起始偏移。
``greedy``（默认）
    依次放置：每台机器人从 0 开始以小步长向后找**最早**的无冲突起点，
    实在找不到（或超过 ``max_wait``）就退化为串行。目标是最小化总节拍。

单位：毫米 / 秒（关节角用度）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .simulation import Trajectory
from domain.world.scene import Scene

__all__ = [
    "STRATEGIES",
    "RobotJob",
    "Interference",
    "CellSchedule",
    "build_job",
    "scan_timeline",
    "schedule_cell",
]

#: 支持的节拍统筹策略
STRATEGIES: Tuple[str, ...] = ("greedy", "parallel", "serial", "stagger")

#: 逐台搜索最早起点时的步长（秒）
SEARCH_STEP_S = 0.25


@dataclass
class RobotJob:
    """一台机器人在这条时间轴上的一份作业。

    Attributes:
        robot: 场景中的机器人名
        trajectory: 该机器人展开后的轨迹（关节值在**机器人自身基座系**）
        start: 起始偏移（秒）——0 表示与单元节拍起点同步
        label: 报表里显示的名字（默认取机器人名）
        program: 可选，原始程序（报表用）
    """

    robot: str
    trajectory: Trajectory
    start: float = 0.0
    label: str = ""
    program: Any = None

    @property
    def name(self) -> str:
        """报表显示名。"""
        return self.label or self.robot

    @property
    def duration(self) -> float:
        """本作业自身耗时（秒）。"""
        return float(self.trajectory.duration())

    @property
    def end(self) -> float:
        """本作业在时间轴上的结束时刻（秒）。"""
        return self.start + self.duration

    def joints_at(self, t: float) -> Optional[np.ndarray]:
        """时间轴时刻 ``t`` 处的关节角（度）。

        超出作业区间时**夹紧**到两端：起点前是首路点位形（待机），
        终点后是末路点位形（停在结束姿态）。这正是现实里机器人做的事，
        也保证待机位形同样参与干涉检查。

        Args:
            t: 时间轴时刻（秒）

        Returns:
            关节角数组；轨迹为空时 ``None``
        """
        if not self.trajectory.waypoints:
            return None
        local = t - self.start
        wp = self.trajectory.interpolate(local)
        return np.asarray(wp.joints, dtype=np.float64)


@dataclass(frozen=True)
class Interference:
    """一次干涉记录（多机器人时间轴检查的结果）。

    Attributes:
        kind: ``"robot-robot"`` 或 ``"robot-env"``
        time: 时间轴上的时刻（秒）
        robot_a: 主机器人名
        link_a: 主机器人侧发生干涉（或最近）的连杆名
        other: 另一台机器人名，或环境对象名
        link_b: 另一侧的连杆名（``robot-env`` 时为 ``None``）
        separation_mm: 最小间距（毫米）；负值表示穿透
    """

    kind: str
    time: float
    robot_a: str
    link_a: Optional[str]
    other: str
    link_b: Optional[str]
    separation_mm: float

    def __str__(self) -> str:
        a = f"{self.robot_a}:{self.link_a}" if self.link_a else self.robot_a
        b = f"{self.other}:{self.link_b}" if self.link_b else self.other
        return f"t={self.time:.2f}s {a} ↔ {b} ({self.separation_mm:+.1f} mm)"


@dataclass
class CellSchedule:
    """多机器人节拍统筹结果。

    Attributes:
        jobs: 机器人名 → :class:`RobotJob`（含最终起始偏移）
        interferences: **机器人-机器人**干涉（按时刻排序）——这才是节拍统筹
            要解决的问题：错开起始时刻就能消掉
        env_interferences: **机器人-环境**干涉。注意它由单机轨迹决定，错峰
            消不掉，所以单独列出，不参与起点搜索的取舍
        strategy: 实际使用的策略名
        sample_dt: 采样步长（秒）
        clearance: 预警净空（毫米）——间距小于它就记一条干涉
        iterations: 逐台搜索起点的尝试次数（``greedy`` 策略）
    """

    jobs: Dict[str, RobotJob] = field(default_factory=dict)
    interferences: List[Interference] = field(default_factory=list)
    env_interferences: List[Interference] = field(default_factory=list)
    strategy: str = "parallel"
    sample_dt: float = 0.1
    clearance: float = 0.0
    iterations: int = 0

    @property
    def makespan(self) -> float:
        """总节拍（秒）：所有作业结束时刻的最大值。"""
        if not self.jobs:
            return 0.0
        return max(job.end for job in self.jobs.values())

    @property
    def robots_ok(self) -> bool:
        """机器人之间是否互不干涉。"""
        return not self.interferences

    @property
    def ok(self) -> bool:
        """机器人之间、以及机器人与环境是否都无干涉。"""
        return not self.interferences and not self.env_interferences

    def summary(self) -> Dict[str, Any]:
        """结构化摘要（报表/网页/JSON 通用）。"""
        return {
            "strategy": self.strategy,
            "makespan": round(self.makespan, 3),
            "sample_dt": self.sample_dt,
            "clearance": self.clearance,
            "iterations": self.iterations,
            "ok": self.ok,
            "robots_ok": self.robots_ok,
            "robots": [
                {
                    "robot": job.robot,
                    "start": round(job.start, 3),
                    "duration": round(job.duration, 3),
                    "end": round(job.end, 3),
                }
                for job in self.jobs.values()
            ],
            "interferences": [_interference_dict(i) for i in self.interferences],
            "env_interferences": [_interference_dict(i) for i in self.env_interferences],
        }

    def format(self, width: int = 28) -> str:
        """可读报表：甘特式时间条 + 干涉清单。"""
        lines: List[str] = []
        lines.append(f"多机器人节拍统筹（策略 {self.strategy}）")
        lines.append(
            f"  总节拍 {self.makespan:.2f} s，采样步长 {self.sample_dt:.2f} s"
            f"，预警净空 {self.clearance:g} mm"
        )
        if not self.jobs:
            lines.append("  （无作业）")
            return "\n".join(lines)

        span = max(self.makespan, 1e-9)
        bar = max(10, min(width, 40))
        for job in self.jobs.values():
            head = int(round(job.start / span * bar))
            body = max(1, int(round(job.duration / span * bar)))
            strip = " " * head + "█" * body
            lines.append(
                f"  {job.name:<8s} 起 {job.start:6.2f}s 时长 {job.duration:6.2f}s "
                f"止 {job.end:6.2f}s |{strip:<{bar}}|"
            )
        if self.interferences:
            lines.append(f"  机器人-机器人干涉 {len(self.interferences)} 处：")
            for hit in self.interferences[:20]:
                lines.append(f"    · {hit}")
            if len(self.interferences) > 20:
                lines.append(f"    · …另有 {len(self.interferences) - 20} 处")
        else:
            lines.append("  机器人之间无干涉（含待机位形）")
        if self.env_interferences:
            lines.append(
                f"  机器人-环境触碰 {len(self.env_interferences)} 处"
                f"（由单机轨迹决定，错峰无法消除）："
            )
            for hit in self.env_interferences[:10]:
                lines.append(f"    · {hit}")
            if len(self.env_interferences) > 10:
                lines.append(f"    · …另有 {len(self.env_interferences) - 10} 处")
        return "\n".join(lines)


def _interference_dict(hit: Interference) -> Dict[str, Any]:
    """干涉 → 可 JSON 化字典。"""
    return {
        "kind": hit.kind,
        "time": round(hit.time, 3),
        "robot_a": hit.robot_a,
        "link_a": hit.link_a,
        "other": hit.other,
        "link_b": hit.link_b,
        "separation_mm": round(hit.separation_mm, 3),
    }


# --------------------------------------------------------------------------- #
# 构造作业
# --------------------------------------------------------------------------- #


def build_job(
    robot: str,
    trajectory: Trajectory,
    start: float = 0.0,
    label: str = "",
    program: Any = None,
) -> RobotJob:
    """由轨迹直接构造一份作业（已在别处规划好时用）。"""
    return RobotJob(
        robot=robot, trajectory=trajectory, start=float(start), label=label, program=program
    )


def _sample_times(makespan: float, dt: float) -> np.ndarray:
    """采样时刻序列（含终点）。"""
    if makespan <= 0.0:
        return np.asarray([0.0])
    n = max(1, int(np.ceil(makespan / dt)))
    times = np.linspace(0.0, makespan, n + 1)
    return times


def _env_parts(scene: Scene, robot: str) -> List[Tuple[str, list]]:
    """该机器人需要面对的环境对象（排除机器人自身与其挂载件）。"""
    out: List[Tuple[str, list]] = []
    for name, obj in scene.objects.items():
        if scene.is_robot(name) or not obj.visible:
            continue
        if scene.mounted_robot(name) == robot:
            continue  # 自己的工具/工件
        out.append((name, scene.object_parts(name)))
    return out


class _PoseCache:
    """``(机器人, 起始偏移, 时刻) → 世界系连杆几何`` 的缓存。

    节拍统筹要对"某台机器人挪到不同起始偏移"反复试算，已经定下来的机器人
    位形在每次试算里都相同——不缓存的话同一份几何会被反复重算（FK + 蒙皮
    变换），``greedy`` 策略下这是主要的耗时来源。
    """

    def __init__(self, scene: Scene, jobs: Sequence[RobotJob]):
        self.scene = scene
        self.jobs: Dict[str, RobotJob] = {job.robot: job for job in jobs}
        self._cache: Dict[Tuple[str, float, float], Dict[str, list]] = {}

    def link_parts(self, robot: str, start: float, t: float) -> Dict[str, list]:
        """时刻 ``t`` 该机器人（起始偏移 ``start``）的连杆几何（世界系）。"""
        key = (robot, round(float(start), 6), round(float(t), 9))
        if key not in self._cache:
            job = self.jobs.get(robot)
            joints = job.joints_at(t) if job is not None else None
            self._cache[key] = self.scene.robot_world_link_parts(robot, joints)
        return self._cache[key]


def scan_timeline(
    scene: Scene,
    jobs: Sequence[RobotJob],
    dt: float = 0.1,
    clearance: float = 0.0,
    check_env: bool = True,
    env_cache: Optional[Dict[str, List[Tuple[str, list]]]] = None,
) -> Tuple[List[Interference], List[Interference]]:
    """沿时间轴采样，检出机器人-机器人 / 机器人-环境干涉。

    每一对（机器人对、连杆对）只记**最早**发生的一次，避免几百个采样点
    把同一处擦碰刷屏。

    Args:
        scene: 场景（机器人基座位姿从 ``scene.objects`` 读取）
        jobs: 各机器人的作业（起始偏移已确定）
        dt: 采样步长（秒）
        clearance: 预警净空（毫米）：间距低于它即记一条
        check_env: 是否同时检查机器人与环境（搜索起点时可以关掉：
            环境干涉由单机轨迹决定，错峰消不掉，只会拖慢搜索）
        env_cache: 复用上一轮的环境几何（避免重复解析组件几何）

    Returns:
        ``(机器人-机器人干涉, 机器人-环境干涉)``，各自按时刻排序
    """
    from domain.collision import parts_aabb, parts_separation

    job_list = list(jobs)
    makespan = max((j.end for j in job_list), default=0.0)
    times = _sample_times(makespan, dt)
    poses = _PoseCache(scene, job_list)
    env = (
        env_cache
        if env_cache is not None
        else {job.robot: _env_parts(scene, job.robot) for job in job_list}
    )
    env_aabb = {
        robot: [(name, parts, parts_aabb(parts)) for name, parts in items]
        for robot, items in env.items()
    }

    broad = max(clearance, 0.0)
    found: Dict[Tuple[str, ...], Interference] = {}
    env_found: Dict[Tuple[str, ...], Interference] = {}

    for t in times:
        link = {job.robot: poses.link_parts(job.robot, job.start, t) for job in job_list}
        box = {r: {k: parts_aabb(v) for k, v in lp.items()} for r, lp in link.items()}

        # 机器人-机器人
        for i, ja in enumerate(job_list):
            for jb in job_list[i + 1 :]:
                pa, pb = link[ja.robot], link[jb.robot]
                if not pa or not pb:
                    continue
                for la, parts_a in pa.items():
                    box_a = box[ja.robot][la]
                    if box_a is None:
                        continue
                    for lb, parts_b in pb.items():
                        box_b = box[jb.robot][lb]
                        if box_b is None or not box_a.overlaps(box_b, margin=broad):
                            continue
                        sep = parts_separation(parts_a, parts_b)
                        if sep > clearance:
                            continue
                        key = ("robot-robot", ja.robot, la, jb.robot, lb)
                        if key not in found:
                            found[key] = Interference(
                                kind="robot-robot",
                                time=float(t),
                                robot_a=ja.robot,
                                link_a=la,
                                other=jb.robot,
                                link_b=lb,
                                separation_mm=float(sep),
                            )

        # 机器人-环境（含待机位形）
        if not check_env:
            continue
        for ja in job_list:
            pa = link[ja.robot]
            if not pa:
                continue
            for ename, eparts, ebox in env_aabb[ja.robot]:
                if ebox is None:
                    continue
                for la, parts_a in pa.items():
                    box_a = box[ja.robot][la]
                    if box_a is None or not box_a.overlaps(ebox, margin=broad):
                        continue
                    sep = parts_separation(parts_a, eparts)
                    if sep > clearance:
                        continue
                    key = ("robot-env", ja.robot, la, ename, "")
                    if key not in env_found:
                        env_found[key] = Interference(
                            kind="robot-env",
                            time=float(t),
                            robot_a=ja.robot,
                            link_a=la,
                            other=ename,
                            link_b=None,
                            separation_mm=float(sep),
                        )

    return (
        sorted(found.values(), key=lambda h: h.time),
        sorted(env_found.values(), key=lambda h: h.time),
    )


# --------------------------------------------------------------------------- #
# 节拍统筹
# --------------------------------------------------------------------------- #


def _finalize(
    scene: Scene,
    jobs: Sequence[RobotJob],
    strategy: str,
    dt: float,
    clearance: float,
    iterations: int = 0,
) -> CellSchedule:
    """定稿：做一次带环境检查的完整扫描并打包结果。"""
    env = {job.robot: _env_parts(scene, job.robot) for job in jobs}
    robot_hits, env_hits = scan_timeline(
        scene, jobs, dt=dt, clearance=clearance, check_env=True, env_cache=env
    )
    return CellSchedule(
        jobs={j.robot: j for j in jobs},
        interferences=robot_hits,
        env_interferences=env_hits,
        strategy=strategy,
        sample_dt=dt,
        clearance=clearance,
        iterations=iterations,
    )


def _robot_conflicts(
    scene: Scene,
    jobs: Sequence[RobotJob],
    dt: float,
    clearance: float,
) -> List[Interference]:
    """只查机器人之间的干涉（搜索起点用的快速通道）。

    环境干涉由单机轨迹决定，挪起始偏移消不掉；把它算进来只会让搜索永远
    找不到"干净"的候选点，还会白花几倍时间。
    """
    hits, _ = scan_timeline(scene, jobs, dt=dt, clearance=clearance, check_env=False)
    return hits


def schedule_cell(
    scene: Scene,
    jobs: Iterable[RobotJob],
    strategy: str = "greedy",
    sample_dt: float = 0.1,
    clearance: float = 0.0,
    offsets: Optional[Dict[str, float]] = None,
    max_wait: Optional[float] = None,
    search_step: float = SEARCH_STEP_S,
) -> CellSchedule:
    """把多台机器人的作业排到一条时间轴上，并检查互相干涉。

    Args:
        scene: 场景（机器人几何与基座位姿都从这里取）
        jobs: 各机器人的作业（:class:`RobotJob`）
        strategy: ``"greedy"``（默认）/ ``"parallel"`` / ``"serial"`` / ``"stagger"``
        sample_dt: 采样步长（秒）；越小越不容易漏掉快速擦碰，代价是耗时
        clearance: 预警净空（毫米）。``0`` 表示只要不撞就算过；给正值则
            "靠得比这个还近"也算干涉（多机器人现场的实用做法）
        offsets: ``strategy="stagger"`` 时的 ``{机器人名: 起始秒}``
        max_wait: ``greedy`` 时单台允许的最大等待（秒）；``None`` 表示
            上限取"其余作业全部串行完成"这一保守时长
        search_step: ``greedy`` 搜索起点的步长（秒）

    Returns:
        :class:`CellSchedule`（各机器人最终起始偏移 + 干涉清单 + 总节拍）
            注意 ``interferences`` 只含**机器人-机器人**干涉（可被错峰消除），
            机器人与环境的触碰在 ``env_interferences`` 里单列

    Raises:
        ValueError: 策略名非法，或 ``stagger`` 未给 ``offsets``
    """
    if strategy not in STRATEGIES:
        raise ValueError(f"未知策略 {strategy!r}；可用：{' / '.join(STRATEGIES)}")
    job_list = list(jobs)
    for job in job_list:
        if job.robot not in scene.robots:
            raise ValueError(f"场景里没有机器人 {job.robot!r}")

    if strategy == "stagger":
        if not offsets:
            raise ValueError("strategy='stagger' 需要给出 offsets={机器人名: 起始秒}")
        for job in job_list:
            job.start = float(offsets.get(job.robot, 0.0))
        return _finalize(scene, job_list, strategy, sample_dt, clearance)

    if strategy == "parallel":
        for job in job_list:
            job.start = 0.0
        return _finalize(scene, job_list, strategy, sample_dt, clearance)

    if strategy == "serial":
        t = 0.0
        for job in job_list:
            job.start = t
            t = job.end
        return _finalize(scene, job_list, strategy, sample_dt, clearance)

    # ---- greedy：逐台找最早的无冲突起点 ----
    total = sum(j.duration for j in job_list)
    placed: List[RobotJob] = []
    iterations = 0
    for job in job_list:
        upper = total if max_wait is None else float(max_wait)
        # 已放置作业结束时刻之后的"无缝衔接"位置：这是必然可行的兜底起点。
        # 但用户给出 max_wait 就说明他不接受更长的等待，故一并夹紧到上限——
        # 否则"最大等待 1s"的实现会悄悄等到 3s，接口就成了摆设。
        tail = min(max((j.end for j in placed), default=0.0), upper)
        # 候选起点：0, step, 2*step, …（上限是"全部串行"这个必然可行的上界）；
        # 末尾补一个"紧接已放置作业之后"，保证一定收敛
        candidates = [float(v) for v in np.arange(0.0, upper + 1e-9, search_step)]
        if not candidates or candidates[-1] < upper:
            candidates.append(float(upper))
        candidates.append(float(tail))

        chosen: Optional[float] = None
        for cand in candidates:
            iterations += 1
            trial = RobotJob(
                robot=job.robot,
                trajectory=job.trajectory,
                start=float(cand),
                label=job.label,
                program=job.program,
            )
            if not _robot_conflicts(scene, placed + [trial], sample_dt, clearance):
                chosen = float(cand)
                break
        if chosen is None:
            chosen = float(tail)
        job.start = chosen
        placed.append(job)

    return _finalize(scene, job_list, strategy, sample_dt, clearance, iterations)
