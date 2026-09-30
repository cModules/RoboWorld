"""程序规模与性能基准（v1.1.8）。

离线编程的"能用"与"好用"之间隔着规模：几十个示教点的演示跑得飞快，几千个
路点的程序（激光焊、打磨、喷涂的典型形态）会把"每次改动全量重算"的代价暴露
出来。本模块提供两件事：

1. **可复核的基准**——:func:`profile_program` 把一次「规划 + 校验」拆成
   规划/干涉/校验三段分别计时，:func:`scale_report` 给出"指令数 → 路点数 →
   耗时"的规模表。所有数字都是**现场跑出来的**，不写死任何常数。
2. **朴素参考实现**——:func:`reference_link_transforms` 用逐次 scipy 调用重写
   一遍正运动学。它与 :meth:`~domain.kinematics.RobotKinematics.link_transforms`
   数值等价（测试里逐位比对），存在的意义是给"缓存到底省掉了什么"一个可测量
   的对照：关掉缓存时，UR5e 一条 5600 路点的轨迹要重复算上万次同一个关节原点
   旋转。

口径（重要，别把这里的数字当成绝对性能承诺）：

* 耗时与机器负载强相关，同一台机器上不同时刻也会差 20%～30%。本模块的用途是
  **同一台机器上的横向比较**（不同规模、不同实现）与**回归预警**，不是跑分。
* :func:`profile_program` 默认取 ``repeat`` 次里的**最小值**（best-of），这是
  基准的常规做法——最小值最接近"没有干扰时的代价"，而平均值会把系统抖动算进去。
* ``validate`` 内部会自己再规划一次，因此各阶段耗时**不构成**一次总时长的
  分解，模块如实给出 ``wall_s``（整轮基准的墙钟）。

命令行::

    python -m service.perf                      # 默认 200 条指令的基准
    python -m service.perf --motions 800        # 指定规模
    python -m service.perf --sizes 50,100,200   # 只出规模表
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import unicodedata
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, Iterator, List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from support.i18n import tr
from domain.kinematics import RobotKinematics
from domain.world.geometry import (
    AABB,
    CYLINDER_SEGMENTS,
    Box,
    Cylinder,
    Sphere,
    collides,
    penetration_depth,
    prim_aabb,
    sat_axes,
    vertices_of,
)

__all__ = [
    "PhaseTiming",
    "PlanProfile",
    "SamplingRow",
    "CollisionProfile",
    "collision_profile",
    "ScaleRow",
    "ScaleReport",
    "profile_program",
    "scale_report",
    "format_scale_table",
    "reference_link_transforms",
    "reference_rodrigues",
    "sweep_program",
    "default_urdf",
    "main",
    "KernelComparison",
    "compare_collision_kernels",
    "reference_collides",
    "reference_prim_aabb",
    "reference_vertices_of",
    "reference_sat_axes",
    "reference_penetration_depth",
    "reference_overlap_on_axis",
    "CacheScenario",
    "PlanCacheProfile",
    "plan_cache_profile",
    "default_edit_scenarios",
    "linear_program",
]


# --------------------------------------------------------------------------- #
# 朴素参考实现（对照用，不是热路径）
# --------------------------------------------------------------------------- #


def reference_rodrigues(axis: Sequence[float], angle: float) -> np.ndarray:
    """绕任意轴旋转的 3x3 矩阵——朴素写法（每次都从零算）。

    与 :func:`domain.kinematics._rot_about_axis` 数值等价，但刻意不做任何
    缓存/预分配：它是"没有优化时是什么样"的实物证据。
    """
    a = np.asarray(axis, dtype=np.float64).reshape(3)
    n = float(np.linalg.norm(a))
    if n < 1e-12 or abs(angle) < 1e-15:
        return np.eye(3)
    k = a / n
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]], dtype=np.float64)
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def reference_link_transforms(
    kin: RobotKinematics, q: Sequence[float], q_is_radians: bool = True
) -> Dict[str, np.ndarray]:
    """朴素版正运动学：每条链节都重新转 rpy、重新组装 4x4。

    语义与 :meth:`RobotKinematics.link_transforms` 完全一致（同样的 rpy 约定：
    ``Rotation.from_euler("xyz", …)`` 即 URDF 的外旋 / 固定轴），但**不做任何
    记忆化**——关节原点的欧拉角转换、转轴单位化、4x4 组装全部逐次重算。
    它在这里的作用是对照组：把同一组关节值分别喂给两份实现，耗时差就是缓存
    省下的工作量，数值差（若有）就是优化引入的偏差。

    Args:
        kin: 运动学对象
        q: 关节值
        q_is_radians: ``True`` 表示旋转关节按弧度解释

    Returns:
        link 名 → 4x4 齐次矩阵（米）

    Raises:
        ValueError: q 长度与自由度不符
    """
    q = np.asarray(q, dtype=np.float64).ravel()
    if q.size != kin.dof:
        raise ValueError(f"期望 {kin.dof} 个关节值，收到 {q.size} 个")

    chain = kin.chain
    transforms: Dict[str, np.ndarray] = {chain[0].parent: np.eye(4)}
    qi = 0
    for joint in chain:
        parent_T = transforms[joint.parent]
        origin = np.eye(4)
        origin[:3, :3] = Rotation.from_euler("xyz", joint.origin_rpy).as_matrix()
        origin[:3, 3] = joint.origin_xyz
        T = parent_T @ origin
        if joint.movable:
            value = float(q[qi])
            if joint.is_prismatic:
                move = np.eye(4)
                move[:3, 3] = np.asarray(joint.axis, dtype=np.float64) * value
                T = T @ move
            else:
                angle = value if q_is_radians else math.radians(value)
                spin = np.eye(4)
                spin[:3, :3] = reference_rodrigues(joint.axis, angle)
                T = T @ spin
            qi += 1
        transforms[joint.child] = T
    return transforms


# --------------------------------------------------------------------------- #
# 计时
# --------------------------------------------------------------------------- #


def _best_of(fn: Callable[[], object], repeats: int) -> Tuple[float, object]:
    """跑 ``repeats`` 次取**最小值**（best-of）；返回值与最后一次的结果。"""
    best = math.inf
    result: object = None
    for _ in range(max(1, repeats)):
        t0 = time.perf_counter()
        result = fn()
        dt = time.perf_counter() - t0
        best = min(best, dt)
    return best, result


@contextmanager
def _caches_off(planner=None) -> Iterator[None]:
    """计时期间临时关闭结果缓存（v1.2.2 干涉检查、v1.2.3 动力学）。

    缓存命中的"检查"测的不是检查——基准里所有经过
    ``check_trajectory_collisions`` / 力矩校验的计时（含 ``validate`` 内部
    那几次）都必须真算。进入时存档、退出时还原，不动调用方的全局状态。

    两级都要关：全局默认缓存换成 ``None`` 只是断掉"落到全局"的那条路，
    ``resolve_*_cache`` 的优先级里**实例缓存高于全局**——规划器自带实例
    缓存时只关全局照样命中（v1.2.3 诚实性测试抓出的 v1.2.2 缺口）。
    传入 ``planner`` 时同时把它的 ``use_collision_cache`` /
    ``use_dynamics_cache`` 置 False（一票否决，实例缓存一并失效）。

    注：规划结果缓存（v1.2.0）刻意不在其列——规划基准本身就以
    "冷/热缓存"两种工况为观测对象，关掉反而测不出来。
    """
    from service.incremental import (
        get_collision_cache,
        get_dynamics_cache,
        set_collision_cache,
        set_dynamics_cache,
    )

    saved_collision = get_collision_cache()
    saved_dynamics = get_dynamics_cache()
    set_collision_cache(None)
    set_dynamics_cache(None)

    saved_flags: dict = {}
    if planner is not None:
        for attr in ("use_collision_cache", "use_dynamics_cache"):
            if hasattr(planner, attr):
                saved_flags[attr] = getattr(planner, attr)
                setattr(planner, attr, False)
    try:
        yield
    finally:
        set_collision_cache(saved_collision)
        set_dynamics_cache(saved_dynamics)
        for attr, value in saved_flags.items():
            setattr(planner, attr, value)


# v1.2.2 引入时的旧名，保留别名避免外部脚本（若有）引用报错。
_collision_cache_off = _caches_off


@dataclass
class PhaseTiming:
    """单个阶段的耗时。"""

    name: str
    seconds: float
    note: str = ""

    def format(self) -> str:
        line = f"  {self.name:<26} {self.seconds * 1000.0:9.1f} ms"
        return f"{line}   {self.note}" if self.note else line


@dataclass
class PlanProfile:
    """一次「规划 + 校验」基准的全部实测数字。"""

    robot_name: str
    n_commands: int
    n_motions: int
    n_waypoints: int
    cycle_time: float
    path_length: float
    repeats: int
    wall_s: float
    phases: List[PhaseTiming] = field(default_factory=list)
    errors: int = 0
    warnings: int = 0
    scene: bool = False
    cache_before: Dict[str, object] = field(default_factory=dict)
    cache_after: Dict[str, object] = field(default_factory=dict)

    def phase(self, name: str) -> Optional[PhaseTiming]:
        """按名字取阶段耗时（不存在返回 ``None``）。"""
        for ph in self.phases:
            if ph.name == name:
                return ph
        return None

    @property
    def ms_per_waypoint(self) -> float:
        """每个路点的平均规划耗时（毫秒）——规模化的关键指标。"""
        if self.n_waypoints <= 0:
            return 0.0
        plan = self.phase(tr("规划"))
        base = plan.seconds if plan is not None else self.wall_s
        return base * 1000.0 / self.n_waypoints

    def format(self) -> str:
        """人类可读的基准报告。"""
        lines = [
            tr("规模与性能基准 · {robot}").format(robot=self.robot_name),
            tr("  指令 {commands} 条（其中运动 {motions} 条）→ 路点 {wps} 个").format(
                commands=self.n_commands, motions=self.n_motions, wps=self.n_waypoints
            ),
            tr("  仿真节拍 {cycle:.2f} s · 路径长 {length:.1f} mm").format(
                cycle=self.cycle_time, length=self.path_length
            ),
            tr("  校验问题：{errors} 错误 / {warnings} 警告").format(
                errors=self.errors, warnings=self.warnings
            ),
            tr("  阶段耗时（{repeats} 次取最优）").format(repeats=self.repeats),
        ]
        lines.extend(ph.format() for ph in self.phases)
        lines.append(
            tr("  单路点规划 {ms:.3f} ms · 整轮墙钟 {wall:.2f} s").format(
                ms=self.ms_per_waypoint, wall=self.wall_s
            )
        )
        if not self.scene:
            lines.append(tr("  （未接场景，干涉检查跳过）"))
        delta = _cache_delta(self.cache_before, self.cache_after)
        if delta is not None:
            lines.append(
                tr("  关节原点旋转：实算 {computed} 次 / 命中缓存 {hits} 次").format(
                    computed=delta["computed"], hits=delta["hits"]
                )
            )
        return "\n".join(lines)

    def to_dict(self) -> dict:
        """结构化输出（供网页/CI 直接取数）。"""
        return {
            "robot": self.robot_name,
            "commands": self.n_commands,
            "motions": self.n_motions,
            "waypoints": self.n_waypoints,
            "cycle_time": self.cycle_time,
            "path_length": self.path_length,
            "repeats": self.repeats,
            "wall_s": self.wall_s,
            "ms_per_waypoint": self.ms_per_waypoint,
            "errors": self.errors,
            "warnings": self.warnings,
            "scene": self.scene,
            "phases": {ph.name: ph.seconds for ph in self.phases},
            "cache": dict(self.cache_after),
        }


def _cache_delta(before: Dict[str, object], after: Dict[str, object]) -> Optional[Dict[str, int]]:
    """两次 :func:`~domain.kinematics.cache_stats` 之间的差分（缺字段返回 ``None``）。"""
    if not before or not after:
        return None
    try:
        return {
            "computed": int(after["rotmat_computed"]) - int(before["rotmat_computed"]),
            "hits": int(after["rotmat_hits"]) - int(before["rotmat_hits"]),
            "entries": int(after["rotmat_entries"]),
        }
    except (KeyError, TypeError, ValueError):
        return None


def profile_program(
    planner,
    program,
    *,
    check_collision: bool = True,
    repeats: int = 1,
) -> PlanProfile:
    """对「规划 + 校验」分段计时。

    刻意分成三段独立计时，而不是给 :meth:`TrajectoryPlanner.validate` 塞探针：
    干涉检查（:meth:`~service.planner.TrajectoryPlanner.check_trajectory_collisions`）
    本身是公开 API，单独计时不需要改动规划器的任何行为。

    Args:
        planner: :class:`~service.planner.TrajectoryPlanner`
        program: 待测程序
        check_collision: 是否单独计时干涉检查（需要 planner 带场景）
        repeats: 每段重复次数，取**最小值**（best-of）

    Returns:
        :class:`PlanProfile`
    """
    from domain.kinematics import cache_stats

    items = getattr(program, "items", None)
    if items is None:
        items = getattr(program, "motions", [])
    n_commands = len(items)
    n_motions = len(getattr(program, "motions", []) or [])

    # 缓存统计差分：报告里给出"这一段基准里关节原点的欧拉角转换真正算了几次、
    # 命中缓存几次"。计数器只增不减（除非显式 reset_cache），差分因此可靠。
    cache_before = cache_stats()

    wall0 = time.perf_counter()
    plan_s, traj = _best_of(lambda: planner.plan(program), repeats)

    phases = [
        PhaseTiming(
            tr("规划"),
            plan_s,
            tr("{per:.3f} ms/路点").format(per=plan_s * 1000.0 / max(1, len(traj.waypoints))),
        )
    ]

    if check_collision and getattr(planner, "scene", None) is not None:
        from domain.collision import sample_indices

        with _caches_off(planner):
            col_s, hits = _best_of(lambda: planner.check_trajectory_collisions(traj), repeats)
        # 采样密度必须写在报告里：干涉检查的保证强弱由它决定，而它随程序变长
        # 而变弱（默认口径按"总采样数"取样）。不写出来，规模化的代价就是隐形的。
        n_samples = len(sample_indices(traj.waypoints))
        phases.append(
            PhaseTiming(
                tr("干涉检查"),
                col_s,
                tr("{n} 处接触 · 采样 {k} 次（每 {m} 路点 1 次）").format(
                    n=len(hits),
                    k=n_samples,
                    m="{:.1f}".format(len(traj.waypoints) / max(1, n_samples)),
                ),
            )
        )
    else:
        phases.append(PhaseTiming(tr("干涉检查"), 0.0, tr("未接场景，跳过")))

    with _caches_off(planner):
        # validate 内部会再跑一次干涉检查——同样必须真算
        val_s, report = _best_of(lambda: planner.validate(program), repeats)
    phases.append(
        PhaseTiming(
            tr("校验（含内部重新规划）"),
            val_s,
            tr("{n} 条问题").format(n=len(report.issues)),
        )
    )
    wall_s = time.perf_counter() - wall0
    cache_after = cache_stats()

    traj_dt = _cycle_time(traj)
    path_length = _path_length(traj)
    return PlanProfile(
        robot_name=getattr(planner.robot, "name", "Robot"),
        n_commands=n_commands,
        n_motions=n_motions,
        n_waypoints=len(traj.waypoints),
        cycle_time=traj_dt,
        path_length=path_length,
        repeats=max(1, repeats),
        wall_s=wall_s,
        phases=phases,
        errors=len(report.errors),
        warnings=len(report.warnings),
        scene=getattr(planner, "scene", None) is not None,
        cache_before=cache_before,
        cache_after=cache_after,
    )


# --------------------------------------------------------------------------- #
# 干涉检查分段基准（v1.1.9）
# --------------------------------------------------------------------------- #


def _display_width(text: str) -> int:
    """终端显示宽度：CJK 全角字符算 2 列。"""
    return sum(2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in text)


def _pad(text: str, width: int, *, right: bool = False) -> str:
    """按**显示宽度**对齐（``str.ljust`` 拿字符数对齐，中文表格会错位）。"""
    space = " " * max(0, width - _display_width(text))
    return (space + text) if right else (text + space)


# --------------------------------------------------------------------------- #
# 几何内核的朴素参考实现（v1.1.9）
# --------------------------------------------------------------------------- #
#
# 与 :func:`reference_link_transforms` 同一个用意：v1.1.9 给几何内核加了局部
# 几何常量缓存、把 AABB 相交判定从 ``np.all`` 换成标量比较、把 SAT 的逐轴投影
# 换成批量矩阵运算——这些都是"等价改写"，而等价改写最需要的就是**对照物**。
# 下面这组函数是改动前实现的逐字重写（每次都重建局部几何、逐轴 numpy 调用），
# 唯一用途是被对拍与被计时。


def reference_prim_aabb(prim, transform: np.ndarray) -> "AABB":
    """朴素版 AABB：每次都重建半尺寸数组，不做任何缓存。"""
    from domain.world.geometry import AABB

    if isinstance(prim, Box):
        half = np.array(prim.size, dtype=np.float64) / 2.0
    elif isinstance(prim, Cylinder):
        half = np.array([prim.radius, prim.radius, prim.length / 2.0], dtype=np.float64)
    elif isinstance(prim, Sphere):
        r = float(prim.radius)
        half = np.array([r, r, r], dtype=np.float64)
    else:  # pragma: no cover - 与生产实现同样的类型守卫
        raise TypeError(f"未知的基本体类型: {type(prim)}")
    R = np.abs(transform[:3, :3])
    t = transform[:3, 3]
    ext = R @ half
    return AABB(lo=t - ext, hi=t + ext)


def reference_vertices_of(prim, transform: np.ndarray) -> np.ndarray:
    """朴素版顶点变换：每次重建局部顶点（圆柱的 linspace/cos/sin 也重建）。"""
    k = CYLINDER_SEGMENTS
    if isinstance(prim, Box):
        sx, sy, sz = (float(v) / 2.0 for v in prim.size)
        pts = np.array(
            [
                [-sx, -sy, -sz],
                [sx, -sy, -sz],
                [sx, sy, -sz],
                [-sx, sy, -sz],
                [-sx, -sy, sz],
                [sx, -sy, sz],
                [sx, sy, sz],
                [-sx, sy, sz],
            ],
            dtype=np.float64,
        )
    elif isinstance(prim, Cylinder):
        r = float(prim.radius)
        h = float(prim.length) / 2.0
        ang = np.linspace(0.0, 2.0 * math.pi, k, endpoint=False)
        pts = np.vstack(
            [
                np.column_stack([r * np.cos(ang), r * np.sin(ang), np.full(k, -h)]),
                np.column_stack([r * np.cos(ang), r * np.sin(ang), np.full(k, h)]),
            ]
        )
    elif isinstance(prim, Sphere):
        r = float(prim.radius)
        pts = np.array(
            [[a * r, b * r, c * r] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)],
            dtype=np.float64,
        )
    else:  # pragma: no cover
        raise TypeError(f"未知的基本体类型: {type(prim)}")
    R = transform[:3, :3]
    t = transform[:3, 3]
    return (R @ pts.T).T + t


def reference_sat_axes(prim, transform: np.ndarray):
    """朴素版 SAT 候选轴：每次都重建局部面法向与棱方向。"""
    k = CYLINDER_SEGMENTS
    if isinstance(prim, Box) or isinstance(prim, Sphere):
        faces = np.eye(3)
        edges = np.eye(3)
    elif isinstance(prim, Cylinder):
        ang = np.linspace(0.0, 2.0 * math.pi, k, endpoint=False)
        side = np.column_stack([np.cos(ang), np.sin(ang), np.zeros(k)])
        faces = np.vstack([side, [0.0, 0.0, 1.0]])
        tangent = np.column_stack([-np.sin(ang), np.cos(ang), np.zeros(k)])
        edges = np.vstack([tangent, [0.0, 0.0, 1.0]])
    else:  # pragma: no cover
        raise TypeError(f"未知的基本体类型: {type(prim)}")
    R = transform[:3, :3]
    return faces @ R.T, edges @ R.T


def reference_overlap_on_axis(verts_a, verts_b, axis) -> float:
    """朴素版单轴投影重叠量（每次一次 numpy 点积 + 两次归约）。"""
    pa = verts_a @ axis
    pb = verts_b @ axis
    return float(min(pa.max(), pb.max()) - max(pa.min(), pb.min()))


def reference_penetration_depth(verts_a, faces_a, edges_a, verts_b, faces_b, edges_b) -> float:
    """朴素版 SAT 穿透深度：逐轴扫描 + 逐对棱叉积（改动前的实现）。"""
    best = math.inf
    for axis in list(faces_a) + list(faces_b):
        ov = reference_overlap_on_axis(verts_a, verts_b, axis)
        if ov < best:
            best = ov
        if best < 0:
            return best
    for ea in edges_a:
        for eb in edges_b:
            axis = np.cross(ea, eb)
            if float(np.dot(axis, axis)) < 1e-12:
                continue  # 两棱平行，不产生新轴
            ov = reference_overlap_on_axis(verts_a, verts_b, axis)
            if ov < best:
                best = ov
            if best < 0:
                return best
    return best


def reference_collides(prim_a, tf_a, prim_b, tf_b, margin: float = 0.0) -> bool:
    """朴素版碰撞判定：``np.all`` 粗筛 + 逐轴 SAT，无任何缓存与批量。"""
    if isinstance(prim_a, Sphere) and isinstance(prim_b, Sphere):
        ca, cb = tf_a[:3, 3], tf_b[:3, 3]
        return float(np.linalg.norm(ca - cb)) - (prim_a.radius + prim_b.radius) < -margin
    box_a = reference_prim_aabb(prim_a, tf_a)
    box_b = reference_prim_aabb(prim_b, tf_b)
    if not (
        bool(np.all(box_a.lo - margin <= box_b.hi)) and bool(np.all(box_b.lo - margin <= box_a.hi))
    ):
        return False
    verts_a = reference_vertices_of(prim_a, tf_a)
    verts_b = reference_vertices_of(prim_b, tf_b)
    faces_a, edges_a = reference_sat_axes(prim_a, tf_a)
    faces_b, edges_b = reference_sat_axes(prim_b, tf_b)
    depth = reference_penetration_depth(verts_a, faces_a, edges_a, verts_b, faces_b, edges_b)
    return depth > -margin


@dataclass
class KernelComparison:
    """生产实现与朴素参考实现的现场对拍结果。"""

    n_pairs: int
    n_cases: int
    margins: Tuple[float, ...]
    mismatches: Dict[str, int] = field(default_factory=dict)
    exact_depth: int = 0
    worst_depth_delta: float = 0.0
    production_s: float = 0.0
    reference_s: float = 0.0

    #: 穿透深度的可接受偏差（毫米）。批量 SAT 把逐轴点积换成矩阵乘法，
    #: BLAS 的归约顺序可能与逐轴版本差最后一位（实测对基本体几何为 0），
    #: 所以这里按一个**远低于任何工程意义**的容差判定，而不是要求逐位相同。
    DEPTH_TOLERANCE = 1e-9

    @property
    def identical(self) -> bool:
        """所有对拍项零差异，且穿透深度在 :data:`DEPTH_TOLERANCE` 之内。"""
        return (
            all(v == 0 for v in self.mismatches.values())
            and self.worst_depth_delta <= self.DEPTH_TOLERANCE
        )

    @property
    def speedup(self) -> float:
        """生产实现相对朴素实现的速度倍数（>1 为更快）。"""
        if self.production_s <= 0.0:
            return 0.0
        return self.reference_s / self.production_s

    def format(self) -> str:
        """人类可读的对拍报告。"""
        lines = [
            tr("几何内核等价性对拍（生产实现 vs 朴素参考实现）"),
            tr("  {pairs} 对基本体 × {cases} 组位姿/裕度 · 裕度 {margins}").format(
                pairs=self.n_pairs,
                cases=self.n_cases,
                margins=", ".join("{:g}".format(m) for m in self.margins),
            ),
            tr(
                "  碰撞判定差异 {n} 处 · 穿透深度完全相同 {exact}/{pairs} 次 · 最大数值差 {delta:.3e}"
            ).format(
                n=self.mismatches.get("collides", 0),
                exact=self.exact_depth,
                pairs=self.n_pairs,
                delta=self.worst_depth_delta,
            ),
            tr("  AABB {a} · 顶点 {v} · SAT 轴 {s} 差异（均应为 0）").format(
                a=self.mismatches.get("prim_aabb", 0),
                v=self.mismatches.get("vertices_of", 0),
                s=self.mismatches.get("sat_axes", 0),
            ),
            tr("  同一批判定：生产 {fast:.3f} s · 朴素 {slow:.3f} s → {x:.1f}× 更快").format(
                fast=self.production_s, slow=self.reference_s, x=self.speedup
            ),
        ]
        return "\n".join(lines)

    def to_dict(self) -> dict:
        """结构化输出（供网页/CI 直接取数）。"""
        return {
            "n_pairs": self.n_pairs,
            "n_cases": self.n_cases,
            "mismatches": dict(self.mismatches),
            "identical": self.identical,
            "exact_depth": self.exact_depth,
            "worst_depth_delta": self.worst_depth_delta,
            "production_s": self.production_s,
            "reference_s": self.reference_s,
            "speedup": self.speedup,
        }


def _random_prims(rng, n: int) -> List[object]:
    """随机基本体（尺寸取工业场景的常见量级）。"""
    out: List[object] = []
    for _ in range(n):
        kind = int(rng.integers(0, 3))
        if kind == 0:
            out.append(Box(tuple(float(v) for v in rng.uniform(5.0, 400.0, 3))))
        elif kind == 1:
            out.append(Cylinder(float(rng.uniform(5.0, 100.0)), float(rng.uniform(5.0, 500.0))))
        else:
            out.append(Sphere(float(rng.uniform(5.0, 150.0))))
    return out


def _random_transform(rng) -> np.ndarray:
    """随机世界变换（随机姿态 + ±400mm 平移）。"""
    m = np.eye(4)
    m[:3, :3] = Rotation.random(random_state=int(rng.integers(1 << 30))).as_matrix()
    m[:3, 3] = rng.uniform(-400.0, 400.0, 3)
    return m


def compare_collision_kernels(
    n_prims: int = 24,
    repeats: int = 2,
    seed: int = 20260915,
    margins: Sequence[float] = (0.0, -0.5, 20.0, -20.0),
) -> KernelComparison:
    """把几何内核与朴素参考实现现场对拍（等价性 + 速度）。

    对拍项：AABB、顶点集、SAT 候选轴、穿透深度（逐位）、碰撞判定（每个裕度）。
    耗时比较用同一批 "基本体对 × 位姿"，两边各跑一遍取最优——**朴素实现是
    "没有缓存与批量会怎样"的实物证据**，不是估算。

    Args:
        n_prims: 随机基本体个数（两两组合成 ``n·(n+1)/2`` 对）
        repeats: 每种实现重复次数，取最小值
        seed: 随机种子（固定后完全可复现）
        margins: 参与对拍的干涉裕度（毫米）

    Returns:
        :class:`KernelComparison`
    """
    rng = np.random.default_rng(seed)
    prims = _random_prims(rng, n_prims)
    pairs: List[Tuple[object, np.ndarray, object, np.ndarray]] = []
    for i, pa in enumerate(prims):
        for pb in prims[i:]:
            pairs.append((pa, _random_transform(rng), pb, _random_transform(rng)))

    mismatches = {
        "prim_aabb": 0,
        "vertices_of": 0,
        "sat_axes": 0,
        "collides": 0,
        "penetration_depth": 0,
    }
    exact_depth = 0
    worst_delta = 0.0
    n_cases = 0

    for pa, ta, pb, tb in pairs:
        box_a, ref_box = prim_aabb(pa, ta), reference_prim_aabb(pa, ta)
        if not (np.array_equal(box_a.lo, ref_box.lo) and np.array_equal(box_a.hi, ref_box.hi)):
            mismatches["prim_aabb"] += 1

        verts_a, verts_b = vertices_of(pa, ta), vertices_of(pb, tb)
        if not np.array_equal(verts_a, reference_vertices_of(pa, ta)):
            mismatches["vertices_of"] += 1
        faces_a, edges_a = sat_axes(pa, ta)
        r_faces, r_edges = reference_sat_axes(pa, ta)
        if not (np.array_equal(faces_a, r_faces) and np.array_equal(edges_a, r_edges)):
            mismatches["sat_axes"] += 1

        faces_b, edges_b = sat_axes(pb, tb)
        depth = penetration_depth(verts_a, faces_a, edges_a, verts_b, faces_b, edges_b)
        ref_depth = reference_penetration_depth(
            verts_a, r_faces, r_edges, verts_b, *reference_sat_axes(pb, tb)
        )
        if depth == ref_depth:
            exact_depth += 1
        else:
            mismatches["penetration_depth"] += 1
            worst_delta = max(worst_delta, abs(depth - ref_depth))

        for margin in margins:
            n_cases += 1
            if bool(collides(pa, ta, pb, tb, margin)) != bool(
                reference_collides(pa, ta, pb, tb, margin)
            ):
                mismatches["collides"] += 1

    def _run(impl) -> float:
        best = math.inf
        for _ in range(max(1, repeats)):
            t0 = time.perf_counter()
            for pa, ta, pb, tb in pairs:
                for margin in margins:
                    impl(pa, ta, pb, tb, margin)
            best = min(best, time.perf_counter() - t0)
        return best

    production = _run(collides)
    reference = _run(reference_collides)

    return KernelComparison(
        n_pairs=len(pairs),
        n_cases=n_cases,
        margins=tuple(float(m) for m in margins),
        mismatches=mismatches,
        exact_depth=exact_depth,
        worst_depth_delta=worst_delta,
        production_s=production,
        reference_s=reference,
    )


@dataclass
class SamplingRow:
    """一种采样口径下的实测结果。"""

    label: str
    max_joint_step: Optional[float]
    samples: int
    seconds: float
    hits: int
    level1_tests: int
    level1_rejects: int
    narrow_pairs: int
    sat_checks: int
    #: 每几个路点查一次（采样密度的直观量），由 :func:`collision_profile` 回填
    waypoints_per_sample: float = 0.0

    @property
    def ms_per_sample(self) -> float:
        """每次采样的平均耗时（毫秒）。"""
        return self.seconds * 1000.0 / max(1, self.samples)

    @property
    def broad_phase_reject_rate(self) -> float:
        """广相剔除率 ``0~1``（候选对里有多少被 AABB 直接挡掉）。"""
        if self.level1_tests <= 0:
            return 0.0
        return self.level1_rejects / self.level1_tests


@dataclass
class CollisionProfile:
    """沿轨迹干涉检查的分段实测：采样口径 × 开销 × 检出。

    存在的理由是一句话：**干涉检查的保证强弱由采样密度决定**，而"密度"是
    个看不见的量。本报告把它与开销、检出数并排列出来——尤其要让"默认口径在
    大程序上每几十个路点才查一次"这件事从隐性变成显性。
    """

    robot_name: str
    n_waypoints: int
    n_command: int
    margin: float
    n_robot_parts: int
    n_env_objects: int
    n_env_parts: int
    repeats: int
    rows: List[SamplingRow] = field(default_factory=list)

    def row(self, label: str) -> Optional[SamplingRow]:
        """按标签取行（不存在返回 ``None``）。"""
        for r in self.rows:
            if r.label == label:
                return r
        return None

    def format(self) -> str:
        """人类可读的分段报告。"""
        cols = [
            ("", 18, False),
            (tr("采样数"), 8, True),
            (tr("每采样路点"), 12, True),
            (tr("总耗时"), 12, True),
            (tr("每次采样"), 12, True),
            (tr("接触"), 6, True),
            (tr("窄相对"), 9, True),
            (tr("广相剔除"), 10, True),
        ]
        lines = [
            tr("干涉检查分段基准 · {robot}").format(robot=self.robot_name),
            tr("  指令 {commands} 条 → 路点 {wps} 个 · 干涉裕度 {margin:.1f} mm").format(
                commands=self.n_command, wps=self.n_waypoints, margin=self.margin
            ),
            tr("  机器人零件 {parts} 个 · 环境对象 {objs} 个（零件 {eparts} 个）").format(
                parts=self.n_robot_parts, objs=self.n_env_objects, eparts=self.n_env_parts
            ),
            "  " + "-" * 84,
            "  "
            + "".join(
                _pad(text if text else tr("采样口径"), width, right=right)
                for text, width, right in cols
            ),
        ]
        for r in self.rows:
            values = [
                r.label,
                str(r.samples),
                "{:.1f}".format(r.waypoints_per_sample),
                "{:.1f} ms".format(r.seconds * 1000.0),
                "{:.3f} ms".format(r.ms_per_sample),
                str(r.hits),
                str(r.narrow_pairs),
                "{:.1f}%".format(r.broad_phase_reject_rate * 100.0),
            ]
            lines.append(
                "  "
                + "".join(
                    _pad(value, width, right=right)
                    for value, (_t, width, right) in zip(values, cols)
                )
            )
        lines.append(
            tr(
                "  口径说明：「{default}」按总采样数约 300 的预算取样，程序越长"
                "采样点隔得越远；给定关节步长的口径按「机器人可能移动多远」取样，"
                "不随程序变长而稀释。"
            ).format(default=tr("默认"))
        )
        return "\n".join(lines)

    def to_dict(self) -> dict:
        """结构化输出（供网页/CI 直接取数）。"""
        return {
            "robot": self.robot_name,
            "waypoints": self.n_waypoints,
            "commands": self.n_command,
            "margin": self.margin,
            "robot_parts": self.n_robot_parts,
            "env_objects": self.n_env_objects,
            "env_parts": self.n_env_parts,
            "repeats": self.repeats,
            "rows": [
                {
                    "label": r.label,
                    "max_joint_step": r.max_joint_step,
                    "samples": r.samples,
                    "waypoints_per_sample": r.waypoints_per_sample,
                    "seconds": r.seconds,
                    "ms_per_sample": r.ms_per_sample,
                    "hits": r.hits,
                    "level1_tests": r.level1_tests,
                    "level1_rejects": r.level1_rejects,
                    "narrow_pairs": r.narrow_pairs,
                    "sat_checks": r.sat_checks,
                    "broad_phase_reject_rate": r.broad_phase_reject_rate,
                }
                for r in self.rows
            ],
        }


def collision_profile(
    planner,
    program,
    *,
    steps: Sequence[Optional[float]] = (None, 20.0, 10.0, 5.0),
    margin: Optional[float] = None,
    repeats: int = 2,
) -> CollisionProfile:
    """量一遍"干涉检查在不同采样口径下要花多少、查到多少"。

    每个口径独立计时（best-of）并**独立取工作量计数**（前后各取一次统计做差分），
    因此表里的"窄相对/广相剔除率"确实是那一行跑出来的，不是估算。

    干涉裕度默认取 :func:`domain.envelope.envelope_tolerance_mm` 解析出的每机
    容差（与 :meth:`~service.planner.TrajectoryPlanner.validate` 同一口径），
    所以表里的"接触"数与校验报告是同一件事。

    Args:
        planner: :class:`~service.planner.TrajectoryPlanner`（需带场景）
        program: 待测程序（内部先规划一次）
        steps: 每行一个采样口径：``None`` 表示默认口径（约 300 次采样），
            数值表示 ``max_joint_step``（度）
        margin: 干涉裕度（mm）；``None`` 表示用每机包络容差的负值
        repeats: 每行重复次数，取最小值（best-of）

    Returns:
        :class:`CollisionProfile`

    Raises:
        ValueError: planner 未接场景
    """
    from domain.collision import (
        COLLISION_SAMPLE_TARGET,
        collision_stats,
        reset_collision_stats,
        sample_indices,
    )
    from domain.world.geometry import geometry_stats, reset_geometry_stats

    scene = getattr(planner, "scene", None)
    if scene is None:
        raise ValueError("干涉检查需要 planner 带场景（scene=…）")

    if margin is None:
        from domain.envelope import envelope_tolerance_mm

        margin = -float(envelope_tolerance_mm(getattr(planner, "robot", None)))

    traj = planner.plan(program)
    waypoints = len(traj.waypoints)
    title = getattr(getattr(planner, "robot", None), "name", "Robot")

    # 场景侧规模要用**场景里**那台机器人的名字：planner.robot_name 是 None 时
    # 检查走的是场景主机器人（check_config 的 robot=None 语义）。用规划器手上的
    # Robot 名字去查场景几何会查空，机械臂零件数就会显示成 0。
    scene_robot = getattr(planner, "robot_name", None)
    if scene_robot is None:
        names = scene.robot_names
        scene_robot = names[0] if names else None

    # 场景侧规模：让"每次采样要过多少对"这件事有分母。这里刻意复用检查器自己的
    # 环境挑选逻辑（可见/非机器人/挂载豁免），而不是在基准里另写一套过滤——
    # 两套过滤迟早会不一致，那时"分母"就不再是真正被检查的那一批了。
    env_objects = 0
    env_parts = 0
    n_robot_parts = 0
    try:
        from domain.collision import CollisionChecker

        checker = CollisionChecker(scene, margin=margin)
        for _name, parts, _box in checker._env_items(scene_robot):
            env_objects += 1
            env_parts += len(parts)
        link_parts = scene.robot_world_link_parts(scene_robot)
        n_robot_parts = sum(len(v) for v in link_parts.values())
    except Exception:  # 场景形状变化不该让基准崩掉，规模字段退化为 0
        pass

    rows: List[SamplingRow] = []
    for step in steps:
        kwargs = {"margin": margin}
        if step is not None:
            kwargs["max_joint_step"] = float(step)
        n_samples = len(sample_indices(traj.waypoints, 0, step))

        # 计数清零后跑：统计差分即这一行的真实工作量
        reset_collision_stats()
        reset_geometry_stats()
        with _caches_off(planner):
            seconds, hits = _best_of(
                lambda: planner.check_trajectory_collisions(traj, **kwargs), repeats
            )
        col = collision_stats()
        geo = geometry_stats()

        rows.append(
            SamplingRow(
                label=(
                    tr("默认（约 {n} 次）").format(n=COLLISION_SAMPLE_TARGET)
                    if step is None
                    else tr("关节步长 {deg:g}°").format(deg=float(step))
                ),
                max_joint_step=step,
                samples=n_samples,
                seconds=seconds,
                hits=len(hits),
                level1_tests=col["level1_tests"],
                level1_rejects=col["level1_rejects"],
                narrow_pairs=col["narrow_pairs"],
                sat_checks=geo["sat_checks"],
                waypoints_per_sample=waypoints / max(1, n_samples),
            )
        )

    return CollisionProfile(
        robot_name=title,
        n_waypoints=waypoints,
        n_command=len(getattr(program, "items", None) or getattr(program, "motions", []) or []),
        margin=float(margin),
        n_robot_parts=n_robot_parts,
        n_env_objects=env_objects,
        n_env_parts=env_parts,
        repeats=max(1, repeats),
        rows=rows,
    )


def _cycle_time(traj) -> float:
    """轨迹节拍（最后一个路点的时间戳）。"""
    wps = traj.waypoints
    return float(wps[-1].timestamp) if wps else 0.0


def _path_length(traj) -> float:
    """轨迹上 TCP 位置的累计行程 (mm)——纯规模参考，不代表关节运动的路径长。

    关节空间运动（moveJ）的 TCP 走的是空间曲线，规划器对它的 ``path_length``
    恒为 0；这里按路点位姿求和，给出一致的、"轨迹有多长"的直观量。
    """
    wps = traj.waypoints
    if len(wps) < 2:
        return 0.0
    total = 0.0
    prev: Optional[np.ndarray] = None
    for wp in wps:
        pose = getattr(wp, "pose", None)
        matrix = getattr(pose, "matrix", None)
        if matrix is None:
            continue
        pos = np.asarray(matrix, dtype=np.float64)[:3, 3]
        if prev is not None:
            total += float(np.linalg.norm(pos - prev))
        prev = pos
    return total


# --------------------------------------------------------------------------- #
# 规模表
# --------------------------------------------------------------------------- #


@dataclass
class ScaleRow:
    """规模表的一行。"""

    n_motions: int
    n_waypoints: int
    cycle_time: float
    plan_s: float
    validate_s: float
    collision_s: float

    @property
    def ms_per_waypoint(self) -> float:
        """每路点规划耗时（毫秒）。"""
        return self.plan_s * 1000.0 / self.n_waypoints if self.n_waypoints else 0.0


@dataclass
class ScaleReport:
    """规模表（多条不同指令数的程序实测）。"""

    robot_name: str
    rows: List[ScaleRow] = field(default_factory=list)
    repeats: int = 1

    def format(self) -> str:
        """等宽文本表——直接贴进文档/网页。"""
        header = tr("运动条数  路点数  节拍(s)  规划(ms)  校验(ms)  干涉(ms)  ms/路点")
        lines = [
            tr("规模表 · {robot}（{repeats} 次取最优）").format(
                robot=self.robot_name, repeats=self.repeats
            ),
            header,
        ]
        for r in self.rows:
            lines.append(
                f"{r.n_motions:>8}  {r.n_waypoints:>7}  {r.cycle_time:>7.2f}"
                f"  {r.plan_s * 1000.0:>8.1f}  {r.validate_s * 1000.0:>8.1f}"
                f"  {r.collision_s * 1000.0:>8.1f}  {r.ms_per_waypoint:>8.3f}"
            )
        return "\n".join(lines)

    def to_dict(self) -> dict:
        """结构化输出。"""
        return {
            "robot": self.robot_name,
            "repeats": self.repeats,
            "rows": [
                {
                    "motions": r.n_motions,
                    "waypoints": r.n_waypoints,
                    "cycle_time": r.cycle_time,
                    "plan_s": r.plan_s,
                    "validate_s": r.validate_s,
                    "collision_s": r.collision_s,
                    "ms_per_waypoint": r.ms_per_waypoint,
                }
                for r in self.rows
            ],
        }


def scale_report(
    make_program: Callable[[object, int], object],
    *,
    planner_factory,
    sizes: Sequence[int] = (20, 50, 100, 200),
    repeats: int = 1,
    robot_name: Optional[str] = None,
) -> ScaleReport:
    """跑一组不同规模的程序，给出规模表。

    Args:
        make_program: ``(robot, n_motions) -> 程序`` 的构造函数，
            用 :func:`sweep_program` 即可
        planner_factory: ``() -> TrajectoryPlanner``；每个规模**新建**一个
            规划器，避免上一条程序的干涉缓存影响下一条的计时
        sizes: 运动条数列表
        repeats: 每段重复次数（取最小值）
        robot_name: 结果里的机器人名（缺省从规划器读）

    Returns:
        :class:`ScaleReport`
    """
    rows: List[ScaleRow] = []
    name = robot_name or "Robot"
    for n in sizes:
        planner = planner_factory()
        name = getattr(planner.robot, "name", name)
        program = make_program(planner.robot, int(n))
        plan_s, traj = _best_of(lambda: planner.plan(program), repeats)
        if getattr(planner, "scene", None) is not None:
            with _caches_off(planner):
                col_s, _ = _best_of(lambda: planner.check_trajectory_collisions(traj), repeats)
        else:
            col_s = 0.0
        with _caches_off(planner):
            val_s, _ = _best_of(lambda: planner.validate(program), repeats)
        rows.append(
            ScaleRow(
                n_motions=len(getattr(program, "motions", []) or []),
                n_waypoints=len(traj.waypoints),
                cycle_time=_cycle_time(traj),
                plan_s=plan_s,
                validate_s=val_s,
                collision_s=col_s,
            )
        )
    return ScaleReport(robot_name=name, rows=rows, repeats=max(1, repeats))


def format_scale_table(report: ScaleReport) -> str:
    """:meth:`ScaleReport.format` 的函数式入口。"""
    return report.format()


# --------------------------------------------------------------------------- #
# 增量重算基准（v1.2.0）
# --------------------------------------------------------------------------- #


@dataclass
class CacheScenario:
    """一次改动场景下的实测结果。

    Attributes:
        label: 场景名（如「改中段运动」）
        seconds: 该场景规划耗时（best-of）
        segment_hits: 命中的指令数
        segment_total: 需要的指令数（命中 + 未命中）
        plan_hits: 整程序层命中次数（0 或 1）
        waypoints: 该场景的路点数
        equivalent: 与**关缓存**的实算结果是否逐位相同
        max_delta: 与实算结果的最大数值差（关节/位姿/时刻/速度取最大）
        baseline_seconds: 冷启动（关缓存）耗时，用于算加速比
    """

    label: str
    seconds: float
    segment_hits: int
    segment_total: int
    plan_hits: int
    waypoints: int
    equivalent: bool
    max_delta: float
    baseline_seconds: float = 0.0

    @property
    def reuse_rate(self) -> float:
        """分段复用率 ``0~1``。"""
        if self.segment_total <= 0:
            return 1.0 if self.plan_hits else 0.0
        return self.segment_hits / self.segment_total

    @property
    def speedup(self) -> float:
        """相对冷启动的倍数（>1 为更快）。"""
        if self.seconds <= 0.0:
            return 0.0
        return self.baseline_seconds / self.seconds


@dataclass
class PlanCacheProfile:
    """增量重算的分场景实测。

    存在的理由是让一句容易含糊的话落地：**「增量重算能省多少」完全取决于改哪儿**。
    改末尾一条、改中间一条、只改速度、插一条驻留——省下的比例会不一样，而
    「改首条运动」在关节目标程序里之所以也能省很多，是因为**绝对关节目标把扰动
    在下一条指令处吸收掉了**（终点就是目标，链条立刻回到原轨）；换成 pose 目标
    （moveL/moveC，终点由逆解给出）扰动会沿链传下去，省得就少。这个差别必须
    能被量出来，而不是靠一句"增量重算很快"糊过去。
    """

    robot_name: str
    n_command: int
    n_motions: int
    n_waypoints: int
    baseline_seconds: float
    repeats: int
    scenarios: List[CacheScenario] = field(default_factory=list)

    def row(self, label: str) -> Optional[CacheScenario]:
        """按标签取场景（不存在返回 ``None``）。"""
        for s in self.scenarios:
            if s.label == label:
                return s
        return None

    def format(self) -> str:
        """人类可读的分场景报告。"""
        cols = [
            ("", 16, False),
            (tr("耗时"), 11, True),
            (tr("加速"), 7, True),
            (tr("分段复用"), 11, True),
            (tr("整程序"), 8, True),
            (tr("等价"), 8, True),
        ]
        lines = [
            tr("增量重算分场景基准 · {robot}").format(robot=self.robot_name),
            tr(
                "  指令 {commands} 条（运动 {motions} 条）→ 路点 {wps} 个 · 冷启动 {base:.1f} ms"
            ).format(
                commands=self.n_command,
                motions=self.n_motions,
                wps=self.n_waypoints,
                base=self.baseline_seconds * 1000.0,
            ),
            "  " + "-" * 64,
            "  "
            + "".join(
                _pad(text if text else tr("场景"), width, right=right)
                for text, width, right in cols
            ).rstrip(),
        ]
        for s in self.scenarios:
            cells = [
                (s.label, 16, False),
                (f"{s.seconds * 1000.0:.1f} ms", 11, True),
                (f"{s.speedup:.1f}x", 7, True),
                (f"{s.segment_hits}/{s.segment_total}", 11, True),
                (tr("命中") if s.plan_hits else "-", 8, True),
                ("OK" if s.equivalent else tr("差 {d:.1e}").format(d=s.max_delta), 8, True),
            ]
            lines.append(
                "  "
                + "".join(_pad(text, width, right=right) for text, width, right in cells).rstrip()
            )
        lines.append(
            tr(
                "  分段复用率的分母是「指令数」而不是「路点数」——一条指令命中，"
                "意味着它那几十个路点全部免算"
            )
        )
        lines.append(
            tr(
                "  等价列＝与关缓存的实算结果逐位比对（关节/位姿/时刻/速度），"
                "任何一项不为 0 都会在这里显示出来"
            )
        )
        return "\n".join(lines)

    def to_dict(self) -> dict:
        """JSON 友好的字典。"""
        return {
            "robot_name": self.robot_name,
            "n_command": self.n_command,
            "n_motions": self.n_motions,
            "n_waypoints": self.n_waypoints,
            "baseline_s": self.baseline_seconds,
            "repeats": self.repeats,
            "scenarios": [
                {
                    "label": s.label,
                    "seconds": s.seconds,
                    "speedup": s.speedup,
                    "segment_hits": s.segment_hits,
                    "segment_total": s.segment_total,
                    "plan_hits": s.plan_hits,
                    "waypoints": s.waypoints,
                    "equivalent": s.equivalent,
                    "max_delta": s.max_delta,
                }
                for s in self.scenarios
            ],
        }


def default_edit_scenarios() -> List[Tuple[str, Callable[[list], None]]]:
    """默认场景集：覆盖「整程序没变」到「改目标关节」的常见编辑动作。

    Returns:
        ``(标签, 改动函数)`` 列表；改动函数**就地修改指令列表**。
    """
    from service.program import Motion, WaitTime

    def positions(items: List[object]) -> List[int]:
        return [i for i, x in enumerate(items) if isinstance(x, Motion)]

    def bump(items: List[object], k: int, dz: float = 5.0) -> None:
        """把第 k 条运动的**目标**沿世界 X 挪 ``dz``。

        必须分两种目标写法处理，曾经因为只写了关节那一支而量出假数据：
        :class:`~domain.pose.Pose` 是可迭代的（迭代出拍平的 4×4 共 16 个数），
        于是 ``list(pose); target[0] += dz`` 会把它变成一个"16 个浮点数的列表"塞回
        ``motion.target``——后续 :func:`~service.planner.TrajectoryPlanner._expand_linear`
        当场报 ``ValueError: Transformation matrix must be 4x4``。那样量出来的
        「位姿程序的复用率」其实是**失败复用的命中率**，与几何编辑毫无关系。
        """
        motion = items[positions(items)[k]]
        matrix = getattr(motion.target, "matrix", None)
        if matrix is not None:
            from domain.pose import Pose

            moved = np.array(matrix, dtype=np.float64).copy()
            moved[0, 3] += float(dz)
            motion.target = Pose(matrix=moved)
        else:
            target = list(motion.target)
            target[0] += dz
            motion.target = target

    def scenario_noop(items: List[object]) -> None:
        return None

    def scenario_tail(items: List[object]) -> None:
        bump(items, len(positions(items)) - 1)

    def scenario_middle(items: List[object]) -> None:
        bump(items, len(positions(items)) // 2)

    def scenario_head(items: List[object]) -> None:
        bump(items, 0)

    def scenario_speed(items: List[object]) -> None:
        items[positions(items)[len(positions(items)) // 2]].speed = 60.0

    def scenario_insert_dwell(items: List[object]) -> None:
        items.insert(len(items) // 2, WaitTime(0.5))

    def scenario_delete(items: List[object]) -> None:
        pos = positions(items)
        items.pop(pos[len(pos) // 2 + 1])

    return [
        (tr("无改动"), scenario_noop),
        (tr("改末尾运动"), scenario_tail),
        (tr("改中段运动"), scenario_middle),
        (tr("改首条运动"), scenario_head),
        (tr("只改速度"), scenario_speed),
        (tr("插入驻留"), scenario_insert_dwell),
        (tr("删除一条"), scenario_delete),
    ]


def _clone_items(items, robot):
    """重建一份指令序列（新建 Motion 对象，目标也另做一份）。

    **必须新建**：``Program(robot, list(other.items))`` 是浅拷贝，原地改 target
    会把"基准程序"一起改掉——开发期踩到过，现象是所有场景都显示"整程序命中"，
    因为基准程序跟着一起变成了编辑后的样子。

    位姿目标同样要另做一份：:class:`~domain.pose.Pose` 的 ``matrix`` 是 ndarray，
    "共享 Pose 对象 + 原地改矩阵"这个组合能把基准程序悄悄改掉，和上面的浅拷贝
    是同一个坑的两种表现。复制一个 4×4 的代价可以忽略。
    """
    from service.program import Motion, Program

    out = []
    for item in items or []:
        if isinstance(item, Motion):
            target = item.target
            matrix = getattr(target, "matrix", None)
            if matrix is not None:
                from domain.pose import Pose

                target = Pose(matrix=np.array(matrix, dtype=np.float64).copy())
            else:
                target = list(target)
            out.append(
                Motion(
                    item.type,
                    target,
                    via=item.via,
                    speed=item.speed,
                    blend=item.blend,
                    velocity=item.velocity,
                    acceleration=item.acceleration,
                )
            )
        else:
            out.append(item)
    return Program(robot, out)


def plan_cache_profile(
    planner_factory: Callable[[], object],
    make_program: Callable[[object], object],
    *,
    scenarios: Optional[Sequence[Tuple[str, Callable[[list], None]]]] = None,
    repeats: int = 1,
) -> PlanCacheProfile:
    """量一遍「改动之后要重算多少」。

    每个场景都在**干净缓存**上做同一件事：先把基准程序规划一次（预热），
    再规划改动后的程序并计时。同时用**关缓存**的规划器把改动后的程序实算一遍
    作为真值逐位比对——增量路径一旦与实算不一致，报告里会直接显示差异，
    而不是悄悄给一个更快的错结果。

    Args:
        planner_factory: ``() -> TrajectoryPlanner``（每场景新建，避免串味）
        make_program: ``(robot) -> 程序``，用 :func:`sweep_program` /
            :func:`linear_program` 即可
        scenarios: ``(标签, 改动函数)`` 列表；默认 :func:`default_edit_scenarios`
        repeats: 每个场景重复次数（取最小值）

    Returns:
        :class:`PlanCacheProfile`
    """
    from service.incremental import (
        TrajectoryCache,
        incremental_stats,
        reset_incremental_stats,
        set_cache,
    )
    from service.program import Motion

    scenario_list = list(scenarios or default_edit_scenarios())
    probe = planner_factory()
    name = getattr(getattr(probe, "robot", None), "name", "Robot")
    robot = probe.robot

    # 冷启动（关缓存）：这才是"不走增量"的代价
    plain = planner_factory()
    plain.use_cache = False
    cold_program = make_program(robot)
    cold_items = list(getattr(cold_program, "items", []) or [])
    cold_s, cold_traj = _best_of(lambda: plain.plan(cold_program), repeats)
    n_motions = sum(1 for i in cold_items if isinstance(i, Motion))

    def build_edited(mutate) -> object:
        items = list(_clone_items(cold_items, robot).items)
        mutate(items)
        from service.program import Program

        return Program(robot, items)

    results: List[CacheScenario] = []
    for label, mutate in scenario_list:
        # 真值：关缓存实算
        truth_planner = planner_factory()
        truth_planner.use_cache = False
        truth = truth_planner.plan(build_edited(mutate))

        # 增量路径：干净缓存 → 预热基准 → 规划改动后的程序
        set_cache(TrajectoryCache())
        planner_factory().plan(build_edited(lambda items: None))
        reset_incremental_stats()
        warm = planner_factory()
        seconds, traj = _best_of(lambda: warm.plan(build_edited(mutate)), repeats)
        stats = incremental_stats()

        same_len = len(truth.waypoints) == len(traj.waypoints)
        delta = 0.0
        if same_len:
            for a, b in zip(truth.waypoints, traj.waypoints):
                delta = max(delta, float(np.abs(a.joints - b.joints).max()))
                delta = max(
                    delta,
                    float(np.abs(np.asarray(a.pose.matrix) - np.asarray(b.pose.matrix)).max()),
                )
                delta = max(delta, abs(float(a.timestamp) - float(b.timestamp)))
                if a.velocity is not None and b.velocity is not None:
                    delta = max(delta, float(np.abs(a.velocity - b.velocity).max()))
        equivalent = bool(same_len and delta == 0.0 and truth.duration() == traj.duration())

        results.append(
            CacheScenario(
                label=label,
                seconds=seconds,
                segment_hits=int(stats.get("segment_hits", 0)),
                segment_total=int(stats.get("segment_hits", 0))
                + int(stats.get("segment_misses", 0)),
                plan_hits=int(stats.get("plan_hits", 0)),
                waypoints=len(traj.waypoints),
                equivalent=equivalent,
                max_delta=delta,
                baseline_seconds=cold_s,
            )
        )

    return PlanCacheProfile(
        robot_name=name,
        n_command=len(cold_items),
        n_motions=n_motions,
        n_waypoints=len(cold_traj.waypoints),
        baseline_seconds=cold_s,
        repeats=max(1, repeats),
        scenarios=results,
    )


# --------------------------------------------------------------------------- #
# 基准程序
# --------------------------------------------------------------------------- #

#: 基准程序的基准位形（UR5e 常见零位附近，避开奇异与限位）
SWEEP_BASE = (0.0, -90.0, 90.0, -90.0, -90.0, 0.0)


def sweep_program(robot, n_motions: int, seed: int = 3, amplitude: float = 35.0):
    """生成一条"在零位附近来回摆动"的基准程序（可复现，固定 seed）。

    刻意做成关节运动：它的路点数由关节行程与插补分辨率决定，与机器人的
    工作空间无关，因此换机器人也能横向比较。同时掺入 ``waitTime`` 与
    ``SetDO``——基准要覆盖真实程序里 IO/驻留混排的形态，而不是纯净的运动序列。

    Args:
        robot: :class:`~domain.robot.Robot`
        n_motions: 运动指令条数
        seed: 随机种子（固定后完全可复现）
        amplitude: 单轴最大摆幅（度）——别超过限位，否则规划会报错

    Returns:
        已 build 的 :class:`~service.program.ProgramBuilder` 程序

    Raises:
        ValueError: ``n_motions`` 为负
    """
    from service.program import ProgramBuilder

    if n_motions < 0:
        raise ValueError(f"n_motions 必须非负，收到 {n_motions}")
    rng = np.random.default_rng(seed)
    base = np.array(SWEEP_BASE, dtype=np.float64)
    builder = ProgramBuilder(robot)
    for i in range(int(n_motions)):
        q = base + rng.uniform(-amplitude, amplitude, size=6)
        builder.moveJ([float(v) for v in q], speed=80)
        if i % 3 == 0:
            builder.waitTime(0.2)
        if i % 5 == 0:
            builder.setDO("gripper_close", True)
    return builder.build()


def default_urdf() -> Optional[Path]:
    """仓库内置的默认基准机器人 URDF（不在仓库布局里时返回 ``None``）。"""
    root = Path(__file__).resolve().parents[1]
    candidate = root / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"
    return candidate if candidate.exists() else None


def linear_program(
    robot,
    n_motions: int,
    *,
    amplitude: float = 40.0,
    step: float = 25.0,
    seed: int = 5,
):
    """生成一条 **moveL 为主**的基准程序（TCP 在起始位姿附近小幅往复）。

    为什么要跟 :func:`sweep_program` 分开：两者对增量重算的友好程度不同，而且
    差别是**结构性的**、不是实现细节。

    * :func:`sweep_program` 是 moveJ，目标即终点——改一条指令的关节目标，它在
      下一条指令处就被"绝对目标"吸收掉，链条立刻回到原轨，所以复用率高；
    * 本函数的终点是 **位姿**，要经逆解得到，而逆解以上一条的关节值为初值：
      改一条指令的目标位姿，其后整条链的初值都变了，解可能跟着漂（甚至翻腕），
      复用率因此明显更低。

    Args:
        robot: :class:`~domain.robot.Robot`
        n_motions: 运动指令条数
        amplitude: 相对起始位姿的 XY 摆幅 (mm)
        step: 每条指令之间的相位步进（度）
        seed: 随机种子（固定后完全可复现）

    Returns:
        已 build 的 :class:`~service.program.Program`

    Raises:
        ValueError: ``n_motions`` 为负
    """
    from domain.pose import Pose
    from service.program import ProgramBuilder

    if n_motions < 0:
        raise ValueError(f"n_motions 必须非负，收到 {n_motions}")
    q0 = np.asarray(robot.get_joints(), dtype=np.float64).ravel()
    base = robot.forward_kinematics(q0)
    base_matrix = np.asarray(base.matrix, dtype=np.float64)
    # 绕世界 X 轴倾斜 6°：SWEEP_BASE 那把姿态的俯仰正落在 ±90°（欧拉角万向锁），
    # 直线插补里算姿态变化量时会触发 "Gimbal lock" 警告。倾斜一点点既避开它，
    # 也更接近现场（末端很少正对着地面）。
    tilt = np.eye(4)
    a = np.radians(6.0)
    tilt[:3, :3] = np.array(
        [[1.0, 0.0, 0.0], [0.0, np.cos(a), -np.sin(a)], [0.0, np.sin(a), np.cos(a)]]
    )
    base_matrix = tilt @ base_matrix
    builder = ProgramBuilder(robot)
    for i in range(int(n_motions)):
        phase = np.radians(step * i)
        target = Pose(base_matrix.copy())
        target.matrix[0, 3] += amplitude * np.cos(phase)
        target.matrix[1, 3] += amplitude * np.sin(phase)
        target.matrix[2, 3] += 0.5 * amplitude * np.sin(phase * 0.5)
        builder.moveL(target, speed=80)
    return builder.build()


# --------------------------------------------------------------------------- #
# 命令行
# --------------------------------------------------------------------------- #


def main(argv: Optional[Sequence[str]] = None) -> int:
    """``python -m service.perf`` 入口。"""
    parser = argparse.ArgumentParser(
        prog="python -m service.perf",
        description=tr("RobotWorld 规划/校验规模基准"),
    )
    parser.add_argument("--robot", default=None, help=tr("机器人 URDF 路径"))
    parser.add_argument("--motions", type=int, default=200, help=tr("单次基准的运动条数"))
    parser.add_argument(
        "--sizes",
        default=None,
        help=tr("规模表用的运动条数（逗号分隔，如 20,50,100）"),
    )
    parser.add_argument("--repeat", type=int, default=1, help=tr("每段重复次数（取最优）"))
    parser.add_argument(
        "--collision",
        action="store_true",
        help=tr("附加干涉检查分段基准（接仓库内置抓取工作站场景）"),
    )
    parser.add_argument(
        "--collision-steps",
        default="20,10,5",
        help=tr("干涉基准的关节采样步长（度，逗号分隔）"),
    )
    parser.add_argument(
        "--incremental",
        action="store_true",
        help=tr("加跑增量重算分场景基准（moveJ 程序）"),
    )
    parser.add_argument(
        "--incremental-cartesian",
        action="store_true",
        help=tr("增量基准再加跑 moveL（位姿目标）程序——两者复用率差别是结构性的"),
    )
    parser.add_argument("--json", action="store_true", help=tr("输出 JSON"))
    args = parser.parse_args(list(argv) if argv is not None else None)

    from service.planner import TrajectoryPlanner
    from domain.robot import Robot

    urdf = Path(args.robot) if args.robot else default_urdf()
    if urdf is None or not urdf.exists():
        print(tr("找不到机器人模型；用 --robot 指定 URDF 路径"), file=sys.stderr)
        return 2

    def factory():
        robot = Robot(str(urdf), urdf.stem)
        robot.set_joints(list(SWEEP_BASE))
        return TrajectoryPlanner(robot)

    def scene_factory():
        """带场景的规划器（干涉检查必须接场景才有意义）。"""
        from domain.world.templates import build_grasping_workstation

        robot = Robot(str(urdf), urdf.stem)
        robot.set_joints(list(SWEEP_BASE))
        return TrajectoryPlanner(robot, scene=build_grasping_workstation().scene)

    planner = factory()
    program = sweep_program(planner.robot, args.motions)
    profile = profile_program(planner, program, check_collision=False, repeats=max(1, args.repeat))

    # 干涉检查分段基准：用带场景的独立规划器（上面的 factory 不带场景）
    col_profile = None
    if args.collision:
        steps = [None]
        for part in str(args.collision_steps).split(","):
            if part.strip():
                steps.append(float(part))
        scene_planner = scene_factory()
        col_profile = collision_profile(
            scene_planner,
            sweep_program(scene_planner.robot, args.motions),
            steps=tuple(steps),
            repeats=max(1, args.repeat),
        )

    # 增量重算基准：moveJ（关节目标）与 moveL（位姿目标）各跑一遍——
    # 两者对增量重算的友好程度差别是结构性的，只报一个会误导
    cache_profiles: List[PlanCacheProfile] = []
    if args.incremental:
        cache_profiles.append(
            plan_cache_profile(
                factory,
                lambda rb: sweep_program(rb, args.motions),
                repeats=max(1, args.repeat),
            )
        )
        if args.incremental_cartesian:
            cache_profiles.append(
                plan_cache_profile(
                    factory,
                    lambda rb: linear_program(rb, max(1, args.motions // 2)),
                    repeats=max(1, args.repeat),
                )
            )

    if args.json:
        payload = profile.to_dict()
        if col_profile is not None:
            payload["collision"] = col_profile.to_dict()
        if cache_profiles:
            payload["incremental"] = [p.to_dict() for p in cache_profiles]
        if args.sizes:
            sizes = [int(s) for s in str(args.sizes).split(",") if s.strip()]
            payload["scale"] = scale_report(
                sweep_program,
                planner_factory=factory,
                sizes=sizes,
                repeats=max(1, args.repeat),
            ).to_dict()
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0

    print(profile.format())
    if col_profile is not None:
        print()
        print(col_profile.format())
    for cache_profile in cache_profiles:
        print()
        print(cache_profile.format())
    if args.sizes:
        sizes = [int(s) for s in str(args.sizes).split(",") if s.strip()]
        print()
        print(
            scale_report(
                sweep_program,
                planner_factory=factory,
                sizes=sizes,
                repeats=max(1, args.repeat),
            ).format()
        )
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI
    raise SystemExit(main())
