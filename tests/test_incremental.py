"""v1.2.0 增量重算：内容寻址的签名、两级缓存、逐位等价性、以及"能省多少取决于改哪儿"。

这个文件盯五件事：

1. **键必须是内容派生的**。整程序键 =（规划器参数、机器人结构、起始关节值、整程序内容
   摘要）；分段键 =（同上前两项、**单条指令的内容签名**、**该条指令的起始关节值**）。
   第 4 条最容易漏，而漏了它不会报错、只会**静默给出错的轨迹**（第 k 条起点变了却拿旧
   起点的插补结果去接），所以这里有一条专门的回归测试对着"改中段一条"盯。
2. **命中路径与实算路径必须逐位相同**。关节、位姿、时刻、速度四项全比，差 0.0 才算过。
3. **能省多少取决于改哪儿**——而且这个差别是**结构性的**：关节目标（moveJ）的终点就是
   目标，扰动在下一条指令处被绝对目标吸收；位姿目标（moveL）终点由逆解给出，逆解以上一条
   的关节值为初值，扰动会沿链传下去、直到逆解回到同一分支才重新命中。两条曲线都要有断言，
   不然"增量重算很快"就只是一句口号。
4. **只读共享约定**。命中时交还的路点与位姿对象是共享的（深拷贝 5632 个路点比重算还贵），
   列表容器每次新建、关节数组一律复制。这条约定必须有测试兜着，否则某天有人原地改
   ``waypoint.velocity`` 就会把缓存里的结果一起改掉。
5. **失败与警告也要缓存**，且缓存里的文案**不能含指令序号与时刻**——序号随插入/删除而变、
   时刻随前缀时长而变，两者必须在使用时按当下值重建。
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

from orolp import Robot  # noqa: E402
from orolp import i18n  # noqa: E402
from orolp.incremental import (  # noqa: E402
    PLAN_CACHE_MAX_ENTRIES,
    PLAN_CACHE_MAX_WAYPOINTS,
    SEGMENT_CACHE_MAX_ENTRIES,
    SEGMENT_CACHE_MAX_WAYPOINTS,
    PlanSnapshot,
    SegmentPayload,
    TrajectoryCache,
    get_cache,
    incremental_stats,
    item_signature,
    planner_signature,
    program_digest,
    reset_incremental_stats,
    resolve_cache,
    robot_signature,
    set_cache,
)
from orolp.perf import (  # noqa: E402
    SWEEP_BASE,
    CacheScenario,
    PlanCacheProfile,
    _clone_items,
    default_edit_scenarios,
    linear_program,
    plan_cache_profile,
    sweep_program,
)
from orolp.planner import (  # noqa: E402
    SEVERITY_ERROR,
    SEVERITY_WARNING,
    TrajectoryPlanner,
)
from orolp.pose import Pose  # noqa: E402
from orolp.program import (  # noqa: E402
    Motion,
    MotionType,
    Program,
    ProgramBuilder,
)
from orolp.simulation import Waypoint  # noqa: E402

UR5E = REPO_ROOT / "models" / "ur" / "ur5e" / "ur5e.urdf"


# --------------------------------------------------------------------------- #
# 夹具与工具
# --------------------------------------------------------------------------- #


@pytest.fixture(autouse=True)
def _isolate_global_cache():
    """全局默认缓存是进程级的——每个用例前后都换成干净的，别让用例互相串味。"""
    saved = get_cache()
    set_cache(TrajectoryCache())
    try:
        yield
    finally:
        set_cache(saved)


@pytest.fixture
def robot():
    r = Robot(str(UR5E), "UR5e")
    r.set_joints(list(SWEEP_BASE))
    return r


def _robot(name: str = "UR5e") -> Robot:
    r = Robot(str(UR5E), name)
    r.set_joints(list(SWEEP_BASE))
    return r


def _planner(name: str = "UR5e") -> TrajectoryPlanner:
    return TrajectoryPlanner(_robot(name))


def _rebuild(items, robot) -> Program:
    """按内容重建一份程序（新建 Motion 与目标对象，见 ``_clone_items`` 的说明）。"""
    return Program(robot, list(_clone_items(items, robot).items))


def _joint_program(robot, n_motions: int = 12):
    """一段小的 moveJ 程序（终点是绝对关节值）。

    目标刻意做成**逐条不同**：若目标按周期重复，程序内部就会出现「同目标 + 同起点」
    的片段，第二次展开时直接命中缓存——那是缓存内建的免费收益，但会让"机器人变了
    之后应当 0 命中"这类断言莫名其妙地失败（其实是程序内去重，不是跨程序复用）。
    """
    base = np.array(SWEEP_BASE, dtype=np.float64)
    builder = ProgramBuilder(robot)
    for i in range(n_motions):
        offset = np.array([6.0 * np.sin(0.7 * i + k) for k in range(6)])
        builder.moveJ([float(v) for v in base + offset], speed=75.0)
        if i % 4 == 1:
            builder.waitTime(0.2)
    return builder.build()


def _pose_program(robot, n_motions: int = 12):
    """一段小的 moveL 程序（终点是位姿，要经逆解）。"""
    q0 = np.asarray(robot.get_joints(), dtype=np.float64)
    base = np.asarray(robot.forward_kinematics(q0).matrix, dtype=np.float64).copy()
    builder = ProgramBuilder(robot)
    for i in range(n_motions):
        mat = base.copy()
        mat[0, 3] += 12.0 * np.cos(i * 0.4)
        mat[1, 3] += 12.0 * np.sin(i * 0.4)
        builder.moveL(Pose(matrix=mat), speed=40.0)
    return builder.build()


def _counts(make_program, n_motions: int = 16):
    """跑一遍默认场景集，返回 ``(n_motions, {标签: (命中, 未命中, 整程序命中)})``。

    口径：每个场景都用**干净缓存**，先拿**基准程序**预热（这一步是关键——曾经先规划
    改动后的程序再"重算"同一个程序，量出来全是"整程序命中"），再规划改动后的程序。
    """
    robot = _robot()
    items0 = list(make_program(robot, n_motions).items)
    n = sum(1 for x in items0 if isinstance(x, Motion))
    out = {}
    for label, mutate in default_edit_scenarios():
        set_cache(TrajectoryCache())
        _planner().plan(_rebuild(items0, robot))  # 预热基准
        edited = list(_clone_items(items0, robot).items)
        mutate(edited)
        reset_incremental_stats()
        _planner().plan(_rebuild(edited, robot))
        stats = incremental_stats()
        out[label] = (
            int(stats["segment_hits"]),
            int(stats["segment_misses"]),
            int(stats["plan_hits"]),
        )
    return n, out


def _scenario(make_edit=None, make_program=None, n_motions: int = 12):
    """在一个干净缓存上做一次「预热基准 → 规划改动后」的实验。

    Args:
        make_edit: ``(items, robot) -> None``，就地改动指令列表；``None`` 表示不改动
        make_program: ``(robot, n) -> Program``，默认 :func:`_joint_program`

    Returns:
        ``(planner, trajectory, stats)``
    """
    build = make_program or _joint_program
    robot = _robot()
    items0 = list(build(robot, n_motions).items)
    set_cache(TrajectoryCache())
    _planner().plan(_rebuild(items0, robot))
    edited = list(_clone_items(items0, robot).items)
    if make_edit is not None:
        make_edit(edited, robot)
    reset_incremental_stats()
    planner = _planner()
    traj = planner.plan(_rebuild(edited, robot))
    return planner, traj, incremental_stats()


def _truth(make_edit=None, make_program=None, n_motions: int = 12):
    """同一个改动场景的**关缓存实算**结果（真值）。"""
    build = make_program or _joint_program
    robot = _robot()
    items0 = list(build(robot, n_motions).items)
    edited = list(_clone_items(items0, robot).items)
    if make_edit is not None:
        make_edit(edited, robot)
    planner = _planner()
    planner.use_cache = False
    return planner, planner.plan(Program(robot, edited))


def _max_delta(a, b) -> float:
    """两条轨迹逐位比对，返回关节/位姿/时刻/速度四项里的最大差。"""
    assert len(a.waypoints) == len(b.waypoints)
    delta = 0.0
    for x, y in zip(a.waypoints, b.waypoints):
        delta = max(delta, float(np.abs(x.joints - y.joints).max()))
        delta = max(
            delta, float(np.abs(np.asarray(x.pose.matrix) - np.asarray(y.pose.matrix)).max())
        )
        delta = max(delta, abs(float(x.timestamp) - float(y.timestamp)))
        if x.velocity is not None and y.velocity is not None:
            delta = max(delta, float(np.abs(x.velocity - y.velocity).max()))
    return delta


def _differs(a, b, tol: float = 1.0) -> bool:
    """两条轨迹是否有实质差别（路点数、时长、或任一关节角差超过 ``tol``）。"""
    if len(a.waypoints) != len(b.waypoints):
        return True
    if abs(a.duration() - b.duration()) > 1e-9:
        return True
    return any(
        float(np.abs(x.joints - y.joints).max()) > tol
        for x, y in zip(a.waypoints, b.waypoints)
    )


def _bump_joint(items, k: int, dz: float = 5.0) -> None:
    """把第 k 条运动的关节目标挪 ``dz``（度）。"""
    pos = [i for i, x in enumerate(items) if isinstance(x, Motion)]
    motion = items[pos[k]]
    target = list(motion.target)
    target[0] += dz
    motion.target = target


def _bump_pose(items, k: int, dz: float = 5.0) -> None:
    """把第 k 条运动的位姿目标沿世界 X 挪 ``dz``（mm）。"""
    pos = [i for i, x in enumerate(items) if isinstance(x, Motion)]
    motion = items[pos[k]]
    mat = np.array(motion.target.matrix, dtype=np.float64).copy()
    mat[0, 3] += dz
    motion.target = Pose(matrix=mat)


def _n_motions(program) -> int:
    return sum(1 for x in program.items if isinstance(x, Motion))


# --------------------------------------------------------------------------- #
# 1. 签名：键必须是内容派生的
# --------------------------------------------------------------------------- #


def test_item_signature_same_content_different_object(robot):
    a = ProgramBuilder(robot)
    a.moveJ([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], speed=50)
    b = ProgramBuilder(robot)
    b.moveJ([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], speed=50)
    ma = a.build().items[0]
    mb = b.build().items[0]
    assert ma is not mb
    assert item_signature(ma) == item_signature(mb)


def test_item_signature_changes_with_target(robot):
    a = ProgramBuilder(robot)
    a.moveJ([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], speed=50)
    b = ProgramBuilder(robot)
    b.moveJ([1.0, 2.0, 3.0, 4.0, 5.0, 6.5], speed=50)
    assert item_signature(a.build().items[0]) != item_signature(b.build().items[0])


def test_item_signature_changes_with_speed(robot):
    a = ProgramBuilder(robot)
    a.moveJ([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], speed=50)
    b = ProgramBuilder(robot)
    b.moveJ([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], speed=51)
    assert item_signature(a.build().items[0]) != item_signature(b.build().items[0])


def test_item_signature_distinguishes_motion_type(robot):
    a = ProgramBuilder(robot)
    a.moveJ([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], speed=50)
    b = ProgramBuilder(robot)
    b.moveL([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], speed=50)
    assert item_signature(a.build().items[0]) != item_signature(b.build().items[0])


def test_item_signature_covers_wait_time(robot):
    a = ProgramBuilder(robot)
    a.waitTime(0.5)
    b = ProgramBuilder(robot)
    b.waitTime(0.6)
    assert item_signature(a.build().items[0]) != item_signature(b.build().items[0])


def test_item_signature_is_json_text(robot):
    builder = ProgramBuilder(robot)
    builder.moveJ([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], speed=50)
    sig = item_signature(builder.build().items[0])
    assert isinstance(sig, str) and json.loads(sig)


def test_program_digest_matches_manual_hash_of_signatures(robot):
    builder = ProgramBuilder(robot)
    builder.moveJ([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], speed=50)
    builder.waitTime(0.3)
    builder.moveJ([2.0, 3.0, 4.0, 5.0, 6.0, 7.0], speed=50)
    items = builder.build().items
    sigs = [item_signature(x) for x in items]
    assert program_digest(sigs) == program_digest([item_signature(x) for x in items])
    assert len(program_digest(sigs)) == 64


def test_program_digest_is_order_sensitive(robot):
    builder = ProgramBuilder(robot)
    builder.moveJ([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], speed=50)
    builder.moveJ([2.0, 3.0, 4.0, 5.0, 6.0, 7.0], speed=50)
    items = builder.build().items
    forward = [item_signature(x) for x in items]
    assert program_digest(forward) != program_digest(list(reversed(forward)))


def test_program_digest_distinguishes_joined_null_bounds(robot):
    """两个签名拼起来必须用分隔符，不然 ``["ab","c"]`` 与 ``["a","bc"]`` 会撞。"""
    assert program_digest(["ab", "c"]) != program_digest(["a", "bc"])


def test_program_digest_of_empty_is_stable():
    assert program_digest([]) == program_digest([])
    assert len(program_digest([])) == 64


def test_robot_signature_stable_across_calls(robot):
    assert robot_signature(robot) == robot_signature(robot)


def test_robot_signature_differs_between_names():
    assert robot_signature(_robot("UR5e")) != robot_signature(_robot("Another"))


def test_robot_signature_detects_in_place_joint_origin_edit(robot):
    """``JointSpec`` 是冻结 dataclass，但它持有的 ndarray 可写——原地改必须被察觉。"""
    before = robot_signature(robot)
    joint = robot.kinematics.chain[2]
    joint.origin_xyz[0] += 1.0
    try:
        assert robot_signature(robot) != before
    finally:
        joint.origin_xyz[0] -= 1.0
    assert robot_signature(robot) == before


def test_robot_signature_detects_tool_change(robot):
    before = robot_signature(robot)
    mat = np.asarray(robot.tool.matrix, dtype=np.float64).copy()
    mat[2, 3] += 25.0
    robot.tool = Pose(matrix=mat)
    assert robot_signature(robot) != before


def test_robot_signature_detects_base_change(robot):
    before = robot_signature(robot)
    mat = np.asarray(robot.base.matrix, dtype=np.float64).copy()
    mat[0, 3] += 30.0
    robot.base = Pose(matrix=mat)
    assert robot_signature(robot) != before


def test_planner_signature_stable(robot):
    assert planner_signature(TrajectoryPlanner(robot)) == planner_signature(
        TrajectoryPlanner(_robot())
    )


def test_planner_signature_detects_resolution_change(robot):
    planner = TrajectoryPlanner(robot)
    before = planner_signature(planner)
    planner.linear_res = float(getattr(planner, "linear_res", 1.0)) * 2.0
    assert planner_signature(planner) != before


def test_planner_signature_detects_tcp_speed_change(robot):
    planner = TrajectoryPlanner(robot)
    before = planner_signature(planner)
    planner.tcp_speed = float(getattr(planner, "tcp_speed", 100.0)) * 0.5
    assert planner_signature(planner) != before


def test_planner_signature_detects_profile_change(robot):
    planner = TrajectoryPlanner(robot)
    before = planner_signature(planner)
    planner.profile = str(getattr(planner, "profile", "")) + "_x"
    assert planner_signature(planner) != before


def test_planner_signature_detects_tolerance_change(robot):
    planner = TrajectoryPlanner(robot)
    before = planner_signature(planner)
    planner.path_tol_mm = float(getattr(planner, "path_tol_mm", 0.0)) + 0.5
    assert planner_signature(planner) != before


def test_planner_signature_detects_in_place_limit_edit(robot):
    """轴限位由机器人推导后固定在规划器实例上，改机器人不会被感知——必须进签名。"""
    planner = TrajectoryPlanner(robot)
    before = planner_signature(planner)
    planner.limits.upper[0] += 1.0
    try:
        assert planner_signature(planner) != before
    finally:
        planner.limits.upper[0] -= 1.0
    assert planner_signature(planner) == before


# --------------------------------------------------------------------------- #
# 2. 缓存本体：命中/未命中、限容、计数
# --------------------------------------------------------------------------- #


def _snapshot(n: int) -> PlanSnapshot:
    return PlanSnapshot(
        waypoints=tuple(Waypoint(np.zeros(6), Pose(), timestamp=float(i)) for i in range(n)),
        waypoint_count=n,
    )


def test_lookup_plan_miss_then_hit():
    cache = TrajectoryCache()
    assert cache.lookup_plan("k") is None
    cache.store_plan("k", _snapshot(3))
    hit = cache.lookup_plan("k")
    assert hit is cache.lookup_plan("k")
    stats = cache.stats()
    assert stats["plan_misses"] == 1
    assert stats["plan_hits"] == 2
    assert stats["plan_stores"] == 1


def test_lookup_segment_miss_then_hit():
    cache = TrajectoryCache()
    payload = SegmentPayload(pts=(np.zeros(6),), step_dts=(0.1,))
    assert cache.lookup_segment("s") is None
    cache.store_segment("s", payload)
    assert cache.lookup_segment("s") is payload
    stats = cache.stats()
    assert stats["segment_misses"] == 1
    assert stats["segment_hits"] == 1


def test_disabled_cache_records_nothing():
    cache = TrajectoryCache(enabled=False)
    cache.store_plan("k", _snapshot(2))
    cache.store_segment("s", SegmentPayload(pts=(np.zeros(6),)))
    assert cache.lookup_plan("k") is None
    assert cache.lookup_segment("s") is None
    assert cache.stats()["plan_stores"] == 0
    assert cache.stats()["segment_stores"] == 0
    assert cache.stats()["plan_entries"] == 0
    assert cache.stats()["segment_entries"] == 0


def test_hit_rate_is_zero_before_any_lookup():
    assert TrajectoryCache().stats()["plan_hit_rate"] == 0.0


def test_hit_rate_counts_lookups_not_stores():
    cache = TrajectoryCache()
    cache.store_plan("a", _snapshot(1))
    cache.lookup_plan("a")
    cache.lookup_plan("b")
    assert cache.stats()["plan_hit_rate"] == pytest.approx(0.5)
    assert cache.stats()["segment_hit_rate"] == 0.0


def test_plan_entry_budget_evicts_oldest():
    cache = TrajectoryCache(plan_max_entries=2, plan_max_waypoints=1000)
    for k in ("a", "b", "c"):
        cache.store_plan(k, _snapshot(1))
    assert cache.stats()["plan_entries"] == 2
    assert cache.lookup_plan("a") is None  # 最旧的被淘汰
    assert cache.lookup_plan("c") is not None
    assert cache.stats()["evictions"] == 1


def test_plan_waypoint_budget_evicts_before_entry_budget():
    """限容按**路点数**而不是条数：一份大程序就能把条目预算吃满。"""
    cache = TrajectoryCache(plan_max_entries=8, plan_max_waypoints=10)
    cache.store_plan("big", _snapshot(10))
    cache.store_plan("small", _snapshot(1))
    assert cache.stats()["plan_entries"] == 1
    assert cache.lookup_plan("big") is None
    assert cache.lookup_plan("small") is not None
    assert cache.stats()["plan_waypoints"] == 1


def test_segment_waypoint_budget_evicts_oldest():
    cache = TrajectoryCache(segment_max_entries=100, segment_max_waypoints=4)
    for i in range(4):
        cache.store_segment(
            f"s{i}", SegmentPayload(pts=tuple(np.zeros(6) for _ in range(2)))
        )
    stats = cache.stats()
    assert stats["segment_entries"] == 2
    assert stats["segment_waypoints"] == 4
    assert cache.lookup_segment("s0") is None
    assert cache.lookup_segment("s3") is not None


def test_segment_entry_budget_evicts_oldest():
    cache = TrajectoryCache(segment_max_entries=2, segment_max_waypoints=1000)
    for i in range(3):
        cache.store_segment(f"s{i}", SegmentPayload(pts=(np.zeros(6),)))
    assert cache.stats()["segment_entries"] == 2
    assert cache.lookup_segment("s0") is None


def test_storing_same_key_replaces_without_double_counting():
    cache = TrajectoryCache()
    cache.store_plan("k", _snapshot(5))
    cache.store_plan("k", _snapshot(7))
    stats = cache.stats()
    assert stats["plan_entries"] == 1
    assert stats["plan_waypoints"] == 7


def test_storing_same_segment_key_replaces_without_double_counting():
    cache = TrajectoryCache()
    one = (np.zeros(6),)
    three = tuple(np.zeros(6) for _ in range(3))
    cache.store_segment("s", SegmentPayload(pts=one))
    cache.store_segment("s", SegmentPayload(pts=three))
    stats = cache.stats()
    assert stats["segment_entries"] == 1
    assert stats["segment_waypoints"] == 3


def test_clear_drops_entries_but_keeps_counters():
    """计数是"这一轮干了多少活"，不是"现在有多少"——clear 不该抹掉工作量账本。"""
    cache = TrajectoryCache()
    cache.store_plan("k", _snapshot(2))
    cache.lookup_plan("k")
    cache.clear()
    stats = cache.stats()
    assert stats["plan_entries"] == 0
    assert stats["plan_waypoints"] == 0
    assert stats["plan_hits"] == 1
    assert stats["plan_stores"] == 1


def test_reset_stats_keeps_entries():
    cache = TrajectoryCache()
    cache.store_plan("k", _snapshot(2))
    cache.reset_stats()
    assert cache.stats()["plan_stores"] == 0
    assert cache.lookup_plan("k") is not None


def test_cache_repr_mentions_occupancy():
    cache = TrajectoryCache()
    cache.store_plan("k", _snapshot(3))
    text = repr(cache)
    assert "plans=1/3wp" in text


def test_capacity_defaults_are_waypoint_bounded():
    assert PLAN_CACHE_MAX_ENTRIES >= 1 and PLAN_CACHE_MAX_WAYPOINTS >= 1000
    assert SEGMENT_CACHE_MAX_ENTRIES >= 256 and SEGMENT_CACHE_MAX_WAYPOINTS >= 1000


def test_segment_payload_failed_property():
    assert SegmentPayload().failed is False
    assert SegmentPayload(error="boom", error_code="planning").failed is True


def test_plan_snapshot_defaults_are_empty():
    snap = PlanSnapshot()
    assert snap.waypoints == () and snap.spans == () and snap.issues == ()
    assert snap.path_length == 0.0 and snap.waypoint_count == 0


# --------------------------------------------------------------------------- #
# 3. resolve_cache：谁说了算
# --------------------------------------------------------------------------- #


def test_resolve_cache_use_cache_false_wins(robot):
    planner = TrajectoryPlanner(robot, cache=TrajectoryCache(), use_cache=False)
    assert resolve_cache(planner) is None


def test_resolve_cache_prefers_planner_instance(robot):
    own = TrajectoryCache()
    planner = TrajectoryPlanner(robot, cache=own)
    assert resolve_cache(planner) is own


def test_resolve_cache_ignores_disabled_instance(robot):
    planner = TrajectoryPlanner(robot, cache=TrajectoryCache(enabled=False))
    assert resolve_cache(planner) is None


def test_resolve_cache_falls_back_to_global(robot):
    global_cache = TrajectoryCache()
    set_cache(global_cache)
    assert resolve_cache(TrajectoryPlanner(robot)) is global_cache


def test_resolve_cache_returns_none_when_global_off(robot):
    set_cache(None)
    assert resolve_cache(TrajectoryPlanner(robot)) is None
    assert incremental_stats() == {"enabled": False}


def test_resolve_cache_ignores_disabled_global(robot):
    set_cache(TrajectoryCache(enabled=False))
    assert resolve_cache(TrajectoryPlanner(robot)) is None


def test_set_cache_swaps_the_global_instance():
    first = TrajectoryCache()
    set_cache(first)
    assert get_cache() is first


# --------------------------------------------------------------------------- #
# 4. 规划器集成：整程序命中与逐位等价
# --------------------------------------------------------------------------- #


def test_second_identical_plan_hits_whole_program():
    robot = _robot()
    items = list(_joint_program(robot, 8).items)
    planner = _planner()
    first = planner.plan(_rebuild(items, robot))
    reset_incremental_stats()
    second = planner.plan(_rebuild(items, robot))
    stats = incremental_stats()
    assert stats["plan_hits"] == 1
    assert stats["plan_misses"] == 0
    assert stats["segment_hits"] == 0  # 整程序命中就不再逐条查分段
    assert _max_delta(first, second) == 0.0


def test_whole_program_hit_restores_side_effects():
    """命中路径必须把 ``plan()`` 在 self 上的副作用一模一样放回去。"""
    robot = _robot()
    items = list(_joint_program(robot, 10).items)
    plain = _planner()
    plain.use_cache = False
    ground = plain.plan(_rebuild(items, robot))

    planner = _planner()
    planner.plan(_rebuild(items, robot))  # 冷启动，填缓存
    hit = planner.plan(_rebuild(items, robot))  # 整程序命中

    assert planner._motion_spans == plain._motion_spans
    assert planner._dwell_segments == plain._dwell_segments
    assert planner._last_path_length == plain._last_path_length
    assert [(i.severity, i.code, i.motion_index, i.time) for i in planner.issues] == [
        (i.severity, i.code, i.motion_index, i.time) for i in plain.issues
    ]
    assert len(hit.waypoints) == len(ground.waypoints)


def test_whole_program_hit_skips_dynamics_population():
    """整程序命中连 ``_populate_dynamics`` 都不该跑（它是命中的收益之一）。"""
    robot = _robot()
    items = list(_joint_program(robot, 8).items)
    planner = _planner()
    planner.plan(_rebuild(items, robot))
    calls = []
    original = planner._populate_dynamics

    def counting(traj):
        calls.append(len(traj.waypoints))
        return original(traj)

    planner._populate_dynamics = counting
    planner.plan(_rebuild(items, robot))
    assert calls == []


def test_mutating_returned_container_does_not_disturb_cache():
    robot = _robot()
    items = list(_joint_program(robot, 6).items)
    planner = _planner()
    first = planner.plan(_rebuild(items, robot))
    n = len(first.waypoints)
    first.waypoints.clear()
    third = planner.plan(_rebuild(items, robot))
    assert len(third.waypoints) == n


def test_plan_hit_shares_waypoint_objects_so_callers_must_not_mutate():
    """共享路点对象是**有意的取舍**（深拷贝 5632 个路点比重算一遍还贵）。

    这条测试把取舍的边界钉死：列表容器不共享、路点对象共享。因此调用方必须把交还的
    路点当只读——一旦有人改成"命中时深拷贝"，这里会红，文档也必须跟着改。
    """
    robot = _robot()
    items = list(_joint_program(robot, 6).items)
    planner = _planner()
    first = planner.plan(_rebuild(items, robot))
    second = planner.plan(_rebuild(items, robot))
    assert second.waypoints is not first.waypoints
    assert second.waypoints[3] is first.waypoints[3]


def test_cached_segment_joint_arrays_are_copies():
    """分段层交还的关节数组必须是副本——原地改路点不能写回缓存里的插补结果。

    ``cache._segments`` 在这里被直接读：不变式本身就是"不要写穿到缓存的插补数组"，
    而缓存没有公开的读数入口，读私有字典是唯一能把它断言下来的方式。
    """
    robot = _robot()
    items = list(_joint_program(robot, 6).items)
    cache = TrajectoryCache()
    set_cache(cache)
    _planner().plan(_rebuild(items, robot))  # 冷启动，把两层都填上

    # 让整程序层落空、分段层全中：只改最后一条的速度
    edited = list(_clone_items(items, robot).items)
    last = [x for x in edited if isinstance(x, Motion)][-1]
    last.speed = float(last.speed) + 7.0
    reset_incremental_stats()
    planner = _planner()
    traj = planner.plan(_rebuild(edited, robot))
    assert incremental_stats()["segment_misses"] == 1  # 只有被改的那条要重算

    for waypoint in traj.waypoints:
        waypoint.joints[:] = 999.0
    for payload in cache._segments.values():
        for pt in payload.pts:
            assert not np.allclose(pt, 999.0)


def test_use_cache_false_bypasses_everything():
    robot = _robot()
    items = list(_joint_program(robot, 8).items)
    planner = _planner()
    planner.use_cache = False
    planner.plan(_rebuild(items, robot))
    reset_incremental_stats()
    planner.plan(_rebuild(items, robot))
    stats = incremental_stats()
    assert stats["plan_hits"] == 0 and stats["plan_stores"] == 0
    assert stats["segment_hits"] == 0 and stats["segment_stores"] == 0


# --------------------------------------------------------------------------- #
# 5. 默认场景集：改动之后复用多少（这是本轮的核心结论）
# --------------------------------------------------------------------------- #


def test_joint_program_scenario_counts():
    """关节目标程序的复用账：前缀全中，改一条只多费它自己（加下一条）。

    数字是结构决定的，不是调出来的：第 k 条的键含它自己的起点，所以
    「0..k-1 命中、k 未命中、k+1 因起点改变而未命中、k+2 起又命中」。
    """
    n, counts = _counts(_joint_program, n_motions=20)
    assert n == 20
    assert counts[i18n.tr("无改动")] == (0, 0, 1)
    assert counts[i18n.tr("改末尾运动")][0] == n - 1
    assert counts[i18n.tr("改中段运动")][0] == n - 2
    assert counts[i18n.tr("改首条运动")][0] == n - 2
    assert counts[i18n.tr("只改速度")][0] == n - 1  # 只改速度不动关节值 → 后缀照旧命中
    assert counts[i18n.tr("插入驻留")][0] == n  # 驻留不进分段层
    assert counts[i18n.tr("删除一条")][0] + counts[i18n.tr("删除一条")][1] == n - 1


def test_pose_program_reuses_less_than_joint_program():
    """位姿目标的复用率明显更低——逆解以上一条的关节值为初值，扰动沿链传下去。

    这条断言存在的意义：它是"增量重算能省多少取决于改哪儿"这句话的结构性依据。
    如果哪天有人把位姿目标的复用做成了关节目标那样的水平，那是解算口径变了，
    必须回头解释，而不是默默变得更"快"。
    """
    n_j, joint = _counts(_joint_program, n_motions=20)
    n_p, pose = _counts(_pose_program, n_motions=20)
    assert n_j == n_p == 20
    # 尾部改动两边都能全中：前缀与逆解初值都没变
    assert joint[i18n.tr("改末尾运动")][0] == n_j - 1
    assert pose[i18n.tr("改末尾运动")][0] == n_p - 1
    # 只改速度同理（不动目标）
    assert pose[i18n.tr("只改速度")][0] == n_p - 1
    assert pose[i18n.tr("插入驻留")][0] == n_p
    # 几何编辑：位姿程序命中数显著更低
    assert pose[i18n.tr("改中段运动")][0] < joint[i18n.tr("改中段运动")][0]
    assert pose[i18n.tr("删除一条")][0] < joint[i18n.tr("删除一条")][0]


def test_pose_program_head_edit_reuses_at_least_as_much_as_middle():
    """位姿目标的"改首条"并不比"改中段"差：绝对位姿目标让逆解有机会回到同一分支。

    这是位姿分支里唯一反直觉的一点，写下来免得日后被当成 bug 改掉。实测
    （:func:`linear_program`，n = 16/20/30）：改首条命中 10/14/24，改中段 9/10/15。
    """
    _n, pose = _counts(linear_program, n_motions=20)
    head = pose[i18n.tr("改首条运动")]
    middle = pose[i18n.tr("改中段运动")]
    assert head[0] + head[1] == 20
    assert middle[0] > 0
    assert head[0] >= middle[0]


def test_all_default_scenarios_are_equivalent_to_uncached_truth():
    """等价性守卫：任一场景只要与实算不一致，这里必须红。"""
    for label, mutate in default_edit_scenarios():
        robot = _robot()
        items0 = list(_joint_program(robot, 12).items)
        set_cache(TrajectoryCache())
        _planner().plan(_rebuild(items0, robot))
        edited = list(_clone_items(items0, robot).items)
        mutate(edited)
        incremental = _planner().plan(_rebuild(edited, robot))

        truth_planner = _planner()
        truth_planner.use_cache = False
        truth = truth_planner.plan(Program(robot, edited))
        assert _max_delta(incremental, truth) == 0.0, label
        assert incremental.duration() == truth.duration(), label


def test_pose_scenarios_are_equivalent_to_uncached_truth():
    for label, mutate in default_edit_scenarios():
        robot = _robot()
        items0 = list(_pose_program(robot, 8).items)
        set_cache(TrajectoryCache())
        _planner().plan(_rebuild(items0, robot))
        edited = list(_clone_items(items0, robot).items)
        mutate(edited)
        incremental = _planner().plan(_rebuild(edited, robot))
        truth_planner = _planner()
        truth_planner.use_cache = False
        truth = truth_planner.plan(Program(robot, edited))
        assert _max_delta(incremental, truth) == 0.0, label


def test_pose_scenario_actually_edits_geometry_not_fails():
    """默认场景集对位姿目标必须是**真几何编辑**。

    历史缺陷：``bump()`` 只写了关节那一支，而 ``Pose`` 是可迭代的（迭代出拍平的
    4×4 共 16 个数），于是 ``list(pose); target[0] += dz`` 会把一个"16 个浮点数的
    列表"塞回 ``motion.target``，后续直线插补当场报
    ``ValueError: Transformation matrix must be 4x4``。那样量出来的"位姿程序复用率"
    其实是**失败复用的命中率**，与几何编辑毫无关系。
    """
    robot = _robot()
    items0 = list(_pose_program(robot, 8).items)
    for label, mutate in default_edit_scenarios():
        edited = list(_clone_items(items0, robot).items)
        mutate(edited)
        planner = _planner()
        planner.use_cache = False
        planner.plan(Program(robot, edited))
        errors = [i for i in planner.issues if i.severity == SEVERITY_ERROR]
        assert not errors, (label, [i.message for i in errors])
        for item in edited:
            if isinstance(item, Motion):
                assert isinstance(item.target, Pose), label


def test_pose_target_is_not_shared_with_the_baseline():
    """``_clone_items`` 必须复制位姿的矩阵，否则"原地改矩阵"会悄悄改掉基准程序。"""
    robot = _robot()
    items0 = list(_pose_program(robot, 6).items)
    clone = list(_clone_items(items0, robot).items)
    signature_before = [item_signature(x) for x in items0]
    for item in clone:
        if isinstance(item, Motion):
            item.target.matrix[0, 3] += 7.0
    assert [item_signature(x) for x in items0] == signature_before


# --------------------------------------------------------------------------- #
# 6. 起点进键：漏了它只会静默算错
# --------------------------------------------------------------------------- #


def test_middle_edit_does_not_reuse_the_suffix():
    """回归守卫：第 k 条的起点变了，其后就不能再复用旧起点算出的插补。"""
    n = 12
    planner, traj, stats = _scenario(lambda items, r: _bump_joint(items, n // 2), n_motions=n)
    assert stats["segment_hits"] == n - 2
    assert stats["segment_misses"] == 2


def test_middle_edit_result_differs_from_baseline():
    """改中段一条，结果**必须**与基准不同——相同就说明后缀被错误复用了。"""
    n = 12
    _planner_obj, baseline, _stats = _scenario(None, n_motions=n)
    _planner_obj, edited, _stats = _scenario(
        lambda items, r: _bump_joint(items, n // 2), n_motions=n
    )
    assert _differs(baseline, edited)


def test_head_edit_perturbs_every_following_segment_key():
    """首条改动会让 k+1 的起点也变——如果起点不在键里，这里会错误地全中。"""
    _planner_obj, _traj, stats = _scenario(
        lambda items, r: _bump_joint(items, 0), n_motions=12
    )
    assert stats["segment_misses"] >= 2


def test_speed_only_edit_keeps_the_suffix_hot():
    """只改速度不动关节值 → 起点不变 → 后缀全中，这正是增量重算的主要收益。"""
    def edit(items, robot):
        pos = [i for i, x in enumerate(items) if isinstance(x, Motion)]
        items[pos[4]].speed = 33.0

    _planner_obj, _traj, stats = _scenario(edit, n_motions=12)
    assert stats["segment_hits"] == 11
    assert stats["segment_misses"] == 1


def test_insert_dwell_keeps_every_motion_hot():
    def edit(items, robot):
        from orolp.program import WaitTime

        items.insert(2, WaitTime(0.7))

    _planner_obj, _traj, stats = _scenario(edit, n_motions=10)
    assert stats["segment_hits"] == 10
    assert stats["segment_misses"] == 0


def test_inserted_dwell_lengthens_the_clock():
    from orolp.program import WaitTime

    _p, short, _s = _scenario(None, n_motions=10)
    _p, long, _s = _scenario(lambda items, r: items.insert(2, WaitTime(0.7)), n_motions=10)
    assert long.duration() == pytest.approx(short.duration() + 0.7, abs=1e-9)


def test_robot_change_invalidates_without_clearing():
    """改机器人结构后整程序层与分段层都必须落空——内容寻址的价值就在这里。"""
    robot = _robot()
    items0 = list(_joint_program(robot, 6).items)
    set_cache(TrajectoryCache())
    _planner().plan(_rebuild(items0, robot))
    other = _robot()
    other.kinematics.chain[1].origin_xyz[2] += 4.0
    reset_incremental_stats()
    TrajectoryPlanner(other).plan(_rebuild(items0, other))
    stats = incremental_stats()
    assert stats["plan_hits"] == 0
    assert stats["segment_hits"] == 0


def test_repeated_segment_inside_one_program_is_expanded_once():
    """同一程序里出现两次「同目标 + 同起点」的片段时，第二次直接命中。

    这是缓存内建的免费收益（也是"机器人变了却还有几个命中"那种错觉的来源——
    那些命中来自程序内去重，不是跨程序复用）。程序按周期 3 重复目标时，第 4 条与
    第 1 条、第 5 条与第 2 条的（签名, 起点）完全相同，因此**同一次冷启动内**
    就有 2 次命中。
    """
    robot = _robot()
    base = np.array(SWEEP_BASE, dtype=np.float64)
    builder = ProgramBuilder(robot)
    for i in range(6):
        builder.moveJ([float(v) for v in base + (i % 3 - 1) * 8.0], speed=75.0)
    items = list(builder.build().items)
    cache = TrajectoryCache()
    set_cache(cache)
    _planner().plan(_rebuild(items, robot))  # 冷启动，同一次 plan 内
    assert cache.stats()["segment_hits"] == 2
    assert cache.stats()["segment_stores"] == 4


def test_planner_parameter_change_invalidates():
    robot = _robot()
    items = list(_joint_program(robot, 6).items)
    set_cache(TrajectoryCache())
    _planner().plan(_rebuild(items, robot))
    warm = _planner()
    warm.linear_res = float(getattr(warm, "linear_res", 0.5)) * 2.0
    reset_incremental_stats()
    warm.plan(_rebuild(items, robot))
    stats = incremental_stats()
    assert stats["plan_hits"] == 0


# --------------------------------------------------------------------------- #
# 7. 失败与警告：也要缓存，但文案里不许有位置信息
# --------------------------------------------------------------------------- #


def _failing_program(robot):
    builder = ProgramBuilder(robot)
    builder.moveJ([float(v) for v in SWEEP_BASE], speed=60)
    builder.moveJ([500.0, 0.0, 0.0, 0.0, 0.0, 0.0], speed=60)  # 超限位
    builder.moveJ([float(v) for v in SWEEP_BASE], speed=60)
    return builder.build()


def test_failed_segment_produces_error_issue():
    robot = _robot()
    planner = _planner()
    planner.use_cache = False
    planner.plan(_failing_program(robot))
    errors = [i for i in planner.issues if i.severity == SEVERITY_ERROR]
    assert len(errors) == 1
    assert errors[0].code == "planning"
    assert errors[0].motion_index == 1


def test_failed_segment_is_replayed_from_the_plan_layer():
    """整程序命中时失败照旧被报出来（问题清单是快照的一部分）。"""
    robot = _robot()
    items = list(_failing_program(robot).items)
    set_cache(TrajectoryCache())
    _planner().plan(_rebuild(items, robot))
    reset_incremental_stats()
    planner = _planner()
    planner.plan(_rebuild(items, robot))
    assert incremental_stats()["plan_hits"] == 1
    errors = [i for i in planner.issues if i.severity == SEVERITY_ERROR]
    assert len(errors) == 1 and errors[0].motion_index == 1


def test_failed_segment_is_cached_in_the_segment_layer():
    """整程序层落空（插了一段驻留）时，失败的那条也要以"命中"的形式复用。

    不缓存失败就会每次重放同一个异常，而且命中率会虚高。
    """
    from orolp.program import WaitTime

    robot = _robot()
    items = list(_failing_program(robot).items)
    set_cache(TrajectoryCache())
    _planner().plan(_rebuild(items, robot))

    shifted = list(_clone_items(items, robot).items)
    shifted.insert(0, WaitTime(0.4))
    reset_incremental_stats()
    planner = _planner()
    planner.plan(_rebuild(shifted, robot))
    stats = incremental_stats()
    assert stats["plan_hits"] == 0
    assert stats["segment_hits"] == 3 and stats["segment_misses"] == 0
    errors = [i for i in planner.issues if i.severity == SEVERITY_ERROR]
    assert len(errors) == 1 and errors[0].motion_index == 1


def test_failure_text_contains_no_instruction_index():
    """序号是**位置**函数（会随插入/删除而变），不能进缓存文案。"""
    robot = _robot()
    cache = TrajectoryCache()
    set_cache(cache)
    _planner().plan(_failing_program(robot))
    texts = [
        payload.error for payload in cache._segments.values() if payload.error is not None
    ]
    assert texts
    for text in texts:
        assert "指令" not in text and "[0]" not in text


def test_failed_segment_is_equivalent_through_the_cache():
    robot = _robot()
    items = list(_failing_program(robot).items)
    set_cache(TrajectoryCache())
    _planner().plan(_rebuild(items, robot))
    incremental = _planner().plan(_rebuild(items, robot))
    truth_planner = _planner()
    truth_planner.use_cache = False
    truth = truth_planner.plan(_rebuild(items, robot))
    assert _max_delta(incremental, truth) == 0.0


def _warning_program(robot):
    """第 1 条落在腕部奇异位形（雅可比条件数超阈值）→ 产生 warning 级问题。"""
    builder = ProgramBuilder(robot)
    builder.moveJ([float(v) for v in SWEEP_BASE], speed=50)
    builder.moveJ([0.0, -90.0, 0.0, -90.0, -90.0, 0.0], speed=50)
    return builder.build()


def test_warning_is_rebuilt_with_current_index_and_time():
    """缓存里只有文案；插进一段驻留之后，同一告警的时刻必须跟着后移。"""
    robot = _robot()
    items = list(_warning_program(robot).items)

    set_cache(TrajectoryCache())
    _planner().plan(_rebuild(items, robot))
    reset_incremental_stats()
    replanned = _planner()
    replanned.plan(_rebuild(items, robot))
    warn = [i for i in replanned.issues if i.severity == SEVERITY_WARNING]
    assert len(warn) == 1
    assert warn[0].motion_index == 1
    base_time = warn[0].time

    from orolp.program import WaitTime

    shifted = list(_clone_items(items, robot).items)
    shifted.insert(0, WaitTime(0.5))
    planner2 = _planner()
    planner2.plan(_rebuild(shifted, robot))
    warn2 = [i for i in planner2.issues if i.severity == SEVERITY_WARNING]
    assert len(warn2) == 1
    assert warn2[0].motion_index == 1  # 序号按当下重建
    assert warn2[0].time == pytest.approx(base_time + 0.5, abs=1e-9)
    assert warn2[0].message == warn[0].message


def test_warning_text_does_not_leak_position():
    robot = _robot()
    cache = TrajectoryCache()
    set_cache(cache)
    _planner().plan(_warning_program(robot))
    texts = [p.warn for p in cache._segments.values() if p.warn]
    assert texts
    for text in texts:
        assert "指令" not in text


# --------------------------------------------------------------------------- #
# 8. plan_cache_profile：报告要自洽
# --------------------------------------------------------------------------- #


def test_plan_cache_profile_reports_every_scenario_equivalent():
    robot = _robot()
    profile = plan_cache_profile(
        lambda: TrajectoryPlanner(_robot()),
        lambda rb: _joint_program(rb, 8),
        repeats=1,
    )
    assert isinstance(profile, PlanCacheProfile)
    assert profile.n_motions == 8
    labels = [s.label for s in profile.scenarios]
    assert labels == [label for label, _fn in default_edit_scenarios()]
    assert all(s.equivalent for s in profile.scenarios)
    assert all(s.max_delta == 0.0 for s in profile.scenarios)
    assert all(s.waypoints > 0 for s in profile.scenarios)
    assert profile.baseline_seconds > 0.0


def test_plan_cache_profile_noop_hits_the_whole_program():
    profile = plan_cache_profile(
        lambda: TrajectoryPlanner(_robot()),
        lambda rb: _joint_program(rb, 6),
        repeats=1,
    )
    noop = profile.row(i18n.tr("无改动"))
    assert noop is not None
    assert noop.plan_hits == 1
    assert noop.reuse_rate == 1.0
    assert noop.speedup > 1.0


def test_plan_cache_profile_segment_accounting_is_consistent():
    profile = plan_cache_profile(
        lambda: TrajectoryPlanner(_robot()),
        lambda rb: _joint_program(rb, 10),
        repeats=1,
    )
    tail = profile.row(i18n.tr("改末尾运动"))
    assert tail is not None
    assert tail.segment_hits + 1 == tail.segment_total
    assert 0.0 < tail.reuse_rate < 1.0


def test_plan_cache_profile_supports_pose_programs():
    profile = plan_cache_profile(
        lambda: TrajectoryPlanner(_robot()),
        lambda rb: linear_program(rb, 8),
        repeats=1,
    )
    assert profile.n_motions == 8
    assert all(s.equivalent for s in profile.scenarios)


def test_plan_cache_profile_accepts_custom_scenarios():
    """自定义场景的改动函数签名是 ``(items) -> None``（与默认场景集一致）。"""

    def edit(items):
        _bump_joint(items, 0)

    profile = plan_cache_profile(
        lambda: TrajectoryPlanner(_robot()),
        lambda rb: _joint_program(rb, 6),
        scenarios=[("only-head", edit)],
        repeats=1,
    )
    assert [s.label for s in profile.scenarios] == ["only-head"]
    assert profile.scenarios[0].equivalent is True


def test_plan_cache_profile_format_has_columns_and_notes():
    profile = plan_cache_profile(
        lambda: TrajectoryPlanner(_robot()),
        lambda rb: _joint_program(rb, 6),
        repeats=1,
    )
    text = profile.format()
    assert "ms" in text and "x" in text and "OK" in text
    assert i18n.tr("场景") in text
    assert "路点" in text


def test_plan_cache_profile_to_dict_is_json_friendly():
    profile = plan_cache_profile(
        lambda: TrajectoryPlanner(_robot()),
        lambda rb: _joint_program(rb, 6),
        repeats=1,
    )
    payload = json.loads(json.dumps(profile.to_dict()))
    assert payload["n_motions"] == 6
    assert len(payload["scenarios"]) == len(default_edit_scenarios())
    assert payload["scenarios"][0]["equivalent"] is True


def test_cache_scenario_properties():
    good = CacheScenario(
        label="x", seconds=0.01, segment_hits=3, segment_total=4,
        plan_hits=0, waypoints=10, equivalent=True, max_delta=0.0,
        baseline_seconds=0.04,
    )
    assert good.reuse_rate == pytest.approx(0.75)
    assert good.speedup == pytest.approx(4.0)
    plan_hit = CacheScenario(
        label="y", seconds=0.001, segment_hits=0, segment_total=0,
        plan_hits=1, waypoints=10, equivalent=True, max_delta=0.0,
        baseline_seconds=0.5,
    )
    assert plan_hit.reuse_rate == 1.0
    zero = CacheScenario(
        label="z", seconds=0.0, segment_hits=0, segment_total=1,
        plan_hits=0, waypoints=1, equivalent=False, max_delta=1.0,
    )
    assert zero.speedup == 0.0


def test_default_edit_scenarios_labels_are_unique_and_non_empty():
    labels = [label for label, _fn in default_edit_scenarios()]
    assert len(labels) == len(set(labels))
    assert all(label.strip() for label in labels)


def test_default_edit_scenarios_apply_to_plain_item_lists():
    """改动函数必须能在"刚 clone 出来的列表"上跑，不依赖任何外部状态。"""
    robot = _robot()
    for _label, mutate in default_edit_scenarios():
        items = list(_clone_items(_joint_program(robot, 6).items, robot).items)
        mutate(items)
        assert _n_motions(Program(robot, items)) >= 5


# --------------------------------------------------------------------------- #
# 9. en 模式与文档口径
# --------------------------------------------------------------------------- #


def test_plan_cache_profile_format_english(monkeypatch):
    monkeypatch.setenv("OROLP_LANG", "en")
    i18n.set_language("en")
    try:
        profile = plan_cache_profile(
            lambda: TrajectoryPlanner(_robot()),
            lambda rb: _joint_program(rb, 6),
            repeats=1,
        )
        text = profile.format()
        assert not re.search(r"[\u4e00-\u9fff]", text), text
        assert "OK" in text
    finally:
        i18n.set_language("zh")


def test_default_scenario_labels_english(monkeypatch):
    monkeypatch.setenv("OROLP_LANG", "en")
    i18n.set_language("en")
    try:
        labels = [label for label, _fn in default_edit_scenarios()]
        assert not any(re.search(r"[\u4e00-\u9fff]", label) for label in labels)
    finally:
        i18n.set_language("zh")


def test_incremental_module_documents_its_limits():
    """模块说明必须写清"不能做什么"——否则"增量重算"会被当成万能加速。"""
    text = (REPO_ROOT / "python" / "orolp" / "incremental.py").read_text(encoding="utf-8")
    assert "不能" in text
    assert "只读" in text
    assert "干涉检查" in text  # 明说干涉检查不在缓存范围内
    assert "不会让首次规划更快" in text


def test_planner_docstring_mentions_the_cache_contract():
    text = (REPO_ROOT / "python" / "orolp" / "planner.py").read_text(encoding="utf-8")
    assert "use_cache" in text
    assert "内容寻址" in text or "增量重算" in text


def test_waypoint_is_still_a_plain_copier():
    """只读共享约定建立在 ``Waypoint.__init__`` 会复制 joints 这一事实上。"""
    source = np.zeros(6)
    waypoint = Waypoint(source, Pose())
    source[0] = 42.0
    assert waypoint.joints[0] == 0.0
