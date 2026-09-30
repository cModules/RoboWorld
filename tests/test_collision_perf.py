"""v1.1.9 干涉检查性能：几何内核等价性 + 采样口径 + 工作量账本。

这个文件盯三件事：

1. **优化不许改变结果**。v1.1.9 给几何内核做了三处等价改写——局部几何常量缓存
   （顶点/面法向/棱方向/半尺寸）、AABB 相交判定从 ``np.all`` 换成标量比较、SAT
   从逐轴 numpy 调用换成批量矩阵运算。改写的等价性由
   :func:`service.perf.reference_collides` 一系（改动前实现的逐字重写）在现场对拍
   守住：不只比对布尔结论，还逐位比对 AABB、顶点集、候选轴与穿透深度。
2. **零件盒复用不许改变判定**。``check_config`` 现在把零件 AABB 算一次、宽相与
   窄相共用；本文件用一个**只用公开 API 的暴力实现**独立算一遍，逐项对齐。
3. **采样口径必须写在明处**。干涉检查是采样检查，"采样多密"决定了保证的强弱；
   :func:`sample_indices` 的三种口径、末点必查、步长保证、非法输入，以及
   "默认口径随程序变长而稀释"这一取舍，都要有断言——检查的强度不能是隐形的。
"""

from __future__ import annotations

import json
import math
import re
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from domain.robot import Robot  # noqa: E402
from support import i18n  # noqa: E402
from domain.collision import (
    COLLISION_SAMPLE_TARGET,
    DEFAULT_MAX_JOINT_STEP_DEG,
    CollisionChecker,
    collision_stats,
    parts_separation,
    reset_collision_stats,
    sample_indices,
)
from service.perf import (
    CollisionProfile,
    SamplingRow,
    SWEEP_BASE,
    collision_profile,
    compare_collision_kernels,
    profile_program,
    reference_collides,
    reference_overlap_on_axis,
    reference_penetration_depth,
    reference_prim_aabb,
    reference_sat_axes,
    reference_vertices_of,
    sweep_program,
)
from service.planner import TrajectoryPlanner  # noqa: E402
from domain.world import geometry as geo  # noqa: E402
from domain.world.geometry import (
    AABB,
    Box,
    Cylinder,
    Sphere,
    collides,
    collides_with_boxes,
    penetration_depth,
    prim_aabb,
    sat_axes,
    union_aabb,
    vertices_of,
)
from domain.world.templates import build_grasping_workstation  # noqa: E402

UR5E = REPO_ROOT / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"


@pytest.fixture(autouse=True)
def _clean_stats():
    """每个用例前后都清一遍缓存与计数器（它们是进程级全局）。"""
    geo.reset_geometry_stats()
    reset_collision_stats()
    yield
    geo.reset_geometry_stats()
    reset_collision_stats()


@pytest.fixture
def workstation():
    return build_grasping_workstation().scene


@pytest.fixture
def planner(workstation):
    robot = Robot(str(UR5E), "UR5e")
    robot.set_joints(list(SWEEP_BASE))
    return TrajectoryPlanner(robot, scene=workstation)


@pytest.fixture
def no_collision_result_cache():
    """临时关闭 v1.2.2 干涉检查**结果**缓存（测完还原）。

    本文件大量用例靠 ``collision_stats`` / ``geometry_stats`` 断言"真算的
    工作量"——命中缓存的结果不进 checker，计数器不动，断言会假失败。
    """
    from service.incremental import get_collision_cache, set_collision_cache

    saved = get_collision_cache()
    set_collision_cache(None)
    yield
    set_collision_cache(saved)


# --------------------------------------------------------------------------- #
# 1. 几何内核等价性（对拍朴素参考实现）
# --------------------------------------------------------------------------- #


def _random_prims(rng, n):
    out = []
    for _ in range(n):
        k = int(rng.integers(0, 3))
        if k == 0:
            out.append(Box(tuple(float(v) for v in rng.uniform(5.0, 400.0, 3))))
        elif k == 1:
            out.append(Cylinder(float(rng.uniform(5.0, 100.0)), float(rng.uniform(5.0, 500.0))))
        else:
            out.append(Sphere(float(rng.uniform(5.0, 150.0))))
    return out


def _random_transform(rng):
    from scipy.spatial.transform import Rotation

    m = np.eye(4)
    m[:3, :3] = Rotation.random(random_state=int(rng.integers(1 << 30))).as_matrix()
    m[:3, 3] = rng.uniform(-400.0, 400.0, 3)
    return m


def test_prim_aabb_matches_reference_bitwise():
    rng = np.random.default_rng(11)
    for prim in _random_prims(rng, 24):
        for _ in range(3):
            tf = _random_transform(rng)
            got, ref = prim_aabb(prim, tf), reference_prim_aabb(prim, tf)
            assert np.array_equal(got.lo, ref.lo)
            assert np.array_equal(got.hi, ref.hi)


def test_vertices_and_axes_match_reference_bitwise():
    rng = np.random.default_rng(12)
    for prim in _random_prims(rng, 24):
        tf = _random_transform(rng)
        assert np.array_equal(vertices_of(prim, tf), reference_vertices_of(prim, tf))
        faces, edges = sat_axes(prim, tf)
        r_faces, r_edges = reference_sat_axes(prim, tf)
        assert np.array_equal(faces, r_faces)
        assert np.array_equal(edges, r_edges)


def test_penetration_depth_matches_reference():
    """批量 SAT 必须与逐轴扫描给出**同一个数**（不只是同一个正负）。

    早退语义也要一致：一旦某条轴上分离，返回的是**那条轴上**的值（逐轴实现
    扫到哪条返回哪条），批量实现按同一顺序找第一条负值。这条语义决定了
    ``prim_separation`` 报出的穿透深度——多机器人的"最近仅 +22.9mm"之类的
    数字就来自它，不能因为改写而变。
    """
    rng = np.random.default_rng(13)
    prims = _random_prims(rng, 18)
    compared = 0
    exact = 0
    for i, pa in enumerate(prims):
        for pb in prims[i:]:
            ta, tb = _random_transform(rng), _random_transform(rng)
            va, vb = vertices_of(pa, ta), vertices_of(pb, tb)
            fa, ea = sat_axes(pa, ta)
            fb, eb = sat_axes(pb, tb)
            got = penetration_depth(va, fa, ea, vb, fb, eb)
            ref = reference_penetration_depth(va, fa, ea, vb, fb, eb)
            assert abs(got - ref) <= 1e-9, (pa, pb, got, ref)
            exact += int(got == ref)
            compared += 1
    assert compared > 100
    # 基本体几何上实测逐位相同；留 1 条的余地只防 BLAS 实现差异
    assert exact >= compared - 1


def test_collides_matches_reference_over_all_margins():
    """碰撞判定在正/零/负裕度上都必须与朴素实现一致。

    负裕度是工程必需（包络接触容差），它把 ``depth > -margin`` 变成
    "穿透超过阈量才算撞"——这条分支最容易被批量改写搞错方向。
    """
    rng = np.random.default_rng(14)
    prims = _random_prims(rng, 20)
    margins = (0.0, -0.5, 5.0, 20.0, -20.0, -1000.0)
    cases = 0
    for i, pa in enumerate(prims):
        for pb in prims[i:]:
            for _ in range(2):
                ta, tb = _random_transform(rng), _random_transform(rng)
                for margin in margins:
                    cases += 1
                    assert collides(pa, ta, pb, tb, margin) == reference_collides(
                        pa, ta, pb, tb, margin
                    ), (pa, pb, margin)
    assert cases > 1000


def test_compare_collision_kernels_reports_identity():
    """对拍工具本身：零差异 + 逐位相同 + 字段自洽。"""
    cmp = compare_collision_kernels(n_prims=12, repeats=1)
    assert cmp.identical, cmp.mismatches
    assert cmp.mismatches["collides"] == 0
    assert cmp.exact_depth <= cmp.n_pairs
    assert cmp.worst_depth_delta <= cmp.DEPTH_TOLERANCE
    assert cmp.n_cases == cmp.n_pairs * len(cmp.margins)
    assert cmp.production_s > 0.0 and cmp.reference_s > 0.0
    assert cmp.speedup > 0.0
    payload = cmp.to_dict()
    assert payload["identical"] is True
    assert payload["worst_depth_delta"] <= cmp.DEPTH_TOLERANCE
    assert set(payload["mismatches"]) == {
        "prim_aabb",
        "vertices_of",
        "sat_axes",
        "collides",
        "penetration_depth",
    }
    assert "0.000e+00" in cmp.format()


def test_reference_overlap_on_axis_matches_batch():
    """逐轴版与批量版在单轴上的结果一致（批量版就是它的向量化）。

    这里用**任意**随机点云（不是基本体几何）——批量版把逐轴的
    ``verts @ axis`` 换成一次 ``verts @ axes.T``，BLAS 的归约顺序在任意数据上
    可能差最后一位（实测对基本体几何为 0，见
    :func:`test_penetration_depth_matches_reference`）。所以按一个远低于任何
    工程意义的容差判定，而不是要求逐位相同。
    """
    rng = np.random.default_rng(15)
    va = rng.uniform(-100, 100, (8, 3))
    vb = rng.uniform(-100, 100, (32, 3))
    axes = rng.uniform(-1, 1, (5, 3))
    batch = geo._overlaps_on_axes(va, vb, axes)
    for i, axis in enumerate(axes):
        ref = reference_overlap_on_axis(va, vb, axis)
        assert abs(batch[i] - ref) <= 1e-9 * max(1.0, abs(ref))


# --------------------------------------------------------------------------- #
# 2. AABB 标量化
# --------------------------------------------------------------------------- #


def _reference_overlaps(a: AABB, b: AABB, margin: float = 0.0) -> bool:
    return bool(np.all(a.lo - margin <= b.hi) and np.all(b.lo - margin <= a.hi))


def test_aabb_overlaps_matches_numpy_version():
    rng = np.random.default_rng(21)
    boxes = [AABB(rng.uniform(-50, 50, 3), rng.uniform(50, 150, 3)) for _ in range(40)]
    checked = 0
    for a in boxes:
        for b in boxes:
            for margin in (0.0, 5.0, -20.0, 1e-9):
                checked += 1
                assert a.overlaps(b, margin) == _reference_overlaps(a, b, margin)
    assert checked > 6000


def test_aabb_overlaps_nan_is_never_overlapping():
    """NaN 语义与 ``np.all`` 一致：比较为 False → 判不相交。"""
    a = AABB(np.array([0.0, 0.0, 0.0]), np.array([10.0, 10.0, 10.0]))
    b = AABB(np.array([np.nan, 0.0, 0.0]), np.array([10.0, 10.0, 10.0]))
    assert a.overlaps(b) is False
    assert b.overlaps(a) is False
    assert _reference_overlaps(a, b) is False


def test_aabb_overlaps_accepts_lists():
    """``lo``/``hi`` 是 list 时也要能判（标量索引对 list 同样成立）。"""
    a = AABB([0.0, 0.0, 0.0], [10.0, 10.0, 10.0])
    b = AABB([5.0, 5.0, 5.0], [20.0, 20.0, 20.0])
    c = AABB([11.0, 0.0, 0.0], [20.0, 10.0, 10.0])
    assert a.overlaps(b) is True
    assert a.overlaps(c) is False
    assert a.overlaps(c, margin=2.0) is True


def test_union_aabb_matches_parts_union():
    from domain.world.components import Part

    rng = np.random.default_rng(22)
    parts = [
        Part(Box((10.0, 20.0, 30.0)), _random_transform(rng), (1.0, 1.0, 1.0)) for _ in range(5)
    ]
    from domain.collision import parts_aabb

    union = union_aabb(prim_aabb(p.prim, p.transform) for p in parts)
    direct = parts_aabb(parts)
    assert np.array_equal(union.lo, direct.lo)
    assert np.array_equal(union.hi, direct.hi)
    assert union_aabb([]) is None
    assert union_aabb([None, None]) is None


# --------------------------------------------------------------------------- #
# 3. 局部几何缓存
# --------------------------------------------------------------------------- #


def test_local_geometry_cache_hits_after_warmup():
    prim = Cylinder(30.0, 200.0)
    tf = np.eye(4)
    geo.reset_geometry_stats()
    # 预热：顶点 / 面法向 / 棱方向 / 半尺寸四类缓存各一次
    vertices_of(prim, tf)
    sat_axes(prim, tf)
    prim_aabb(prim, tf)
    first = geo.geometry_stats()
    assert first["cache_misses"] == 4
    for _ in range(5):
        vertices_of(prim, tf)
        sat_axes(prim, tf)
        prim_aabb(prim, tf)
    after = geo.geometry_stats()
    assert after["cache_hits"] > first["cache_hits"]
    assert after["cache_misses"] == first["cache_misses"], "热身之后不该再有未命中"
    assert after["cache_entries"] >= 3


def test_local_geometry_cache_returns_readonly():
    """缓存值在多处共享，必须只读——否则一个调用方的改写会污染所有人。"""
    for prim in (Box((1.0, 2.0, 3.0)), Cylinder(5.0, 10.0), Sphere(5.0)):
        vertices = geo._local_vertices(prim)
        faces = geo._local_face_normals(prim)
        edges = geo._local_edge_dirs(prim)
        extent = geo._local_extent(prim)
        for arr in (vertices, faces, edges, extent):
            assert arr.flags.writeable is False
            with pytest.raises(ValueError):
                arr[0, 0] = 99.0
    # 变换后的顶点是新分配的、调用方可写（下游会就地改）
    out = vertices_of(Sphere(5.0), np.eye(4))
    out[0, 0] = 1.0


def test_local_geometry_cache_distinguishes_parameters():
    """参数不同 ⇒ 局部几何不同（键就是基本体本身，不会串味）。"""
    a, b = Cylinder(10.0, 100.0), Cylinder(20.0, 100.0)
    va, vb = geo._local_vertices(a), geo._local_vertices(b)
    assert not np.array_equal(va, vb)
    assert float(np.abs(va[:, :2]).max()) == pytest.approx(10.0)
    assert float(np.abs(vb[:, :2]).max()) == pytest.approx(20.0)


def test_local_geometry_cache_is_bounded(monkeypatch):
    """触顶整体清空：缓存不许无界增长。"""
    monkeypatch.setattr(geo, "LOCAL_CACHE_LIMIT", 4)
    for i in range(12):
        geo._local_extent(Box((float(i) + 1.0, 2.0, 3.0)))
    entries = len(geo._local_extent_cache)
    assert 0 < entries <= 4


def test_uncacheable_primitive_still_correct():
    """参数不可哈希（size 传 list）时退化为不缓存，结果不变。"""
    prim = Box([10.0, 20.0, 30.0])  # type: ignore[arg-type]
    tf = np.eye(4)
    assert np.array_equal(prim_aabb(prim, tf).lo, reference_prim_aabb(prim, tf).lo)
    assert np.array_equal(vertices_of(prim, tf), reference_vertices_of(prim, tf))
    assert geo.geometry_stats()["cache_hits"] == 0


def test_geometry_stats_reset():
    vertices_of(Sphere(1.0), np.eye(4))
    assert geo.geometry_stats()["cache_misses"] >= 1
    geo.reset_geometry_stats()
    stats = geo.geometry_stats()
    assert stats["cache_entries"] == 0 and stats["cache_misses"] == 0
    assert stats["collides_calls"] == 0


# --------------------------------------------------------------------------- #
# 4. collides_with_boxes：判定必须与 collides 完全一致
# --------------------------------------------------------------------------- #


def test_collides_with_boxes_matches_collides():
    rng = np.random.default_rng(31)
    prims = _random_prims(rng, 16)
    for i, pa in enumerate(prims):
        for pb in prims[i:]:
            ta, tb = _random_transform(rng), _random_transform(rng)
            ba, bb = prim_aabb(pa, ta), prim_aabb(pb, tb)
            for margin in (0.0, -0.5, 20.0, -20.0):
                assert collides_with_boxes(pa, ta, pb, tb, ba, bb, margin) == collides(
                    pa, ta, pb, tb, margin
                )


def test_collides_with_boxes_counts_work():
    """宽相账本：挡掉的对记 prefilter_rejects，进 SAT 的记 sat_checks。"""
    geo.reset_geometry_stats()
    # 用盒而不是球：球-球走解析通道，不经过 AABB 粗筛
    far_a, far_b = Box((10.0, 10.0, 10.0)), Box((10.0, 10.0, 10.0))
    m_a = np.eye(4)
    m_b = np.eye(4)
    m_b[:3, 3] = [1000.0, 0.0, 0.0]
    collides_with_boxes(
        far_a,
        m_a,
        far_b,
        m_b,
        prim_aabb(far_a, m_a),
        prim_aabb(far_b, m_b),
        0.0,
    )
    stats = geo.geometry_stats()
    assert stats["collides_calls"] == 1
    assert stats["prefilter_rejects"] == 1
    assert stats["sat_checks"] == 0
    assert stats["collision_hits"] == 0


# --------------------------------------------------------------------------- #
# 5. 零件盒复用：check_config 判定不变（暴力对照）
# --------------------------------------------------------------------------- #


def _brute_force_hits(scene, joints, margin, robot=None):
    """只用公开 API 的暴力实现：逐个零件对调 :func:`collides`。

    它不共享 ``check_config`` 里的零件盒复用与任何缓存，因此可以守住
    "复用零件盒改变了判定"这类错误。
    """
    from domain.world.scene import ROBOT_NAME

    names = scene.robot_names
    robot_name = robot or (names[0] if names else None)
    scene.set_robot_joints(joints, robot_name)
    link_parts = scene.robot_world_link_parts(robot_name)
    hits = set()
    env = []
    for name, obj in scene.objects.items():
        if scene.is_robot(name) or not obj.visible:
            continue
        if scene.mounted_robot(name) == robot_name:
            continue
        for j, part in enumerate(scene.object_parts(name)):
            env.append((name, j, part))
    for lname, parts in link_parts.items():
        for i, lp in enumerate(parts):
            for ename, j, ep in env:
                if collides(lp.prim, lp.transform, ep.prim, ep.transform, margin):
                    hits.add((lname, "", robot_name, ename))
    # 自碰撞：跳过关节直连对与 fixed 刚体组（与 checker 同一豁免口径）
    checker = CollisionChecker(scene, margin=margin)
    adj, group = checker._self_exemption(robot_name)
    names_sorted = sorted(link_parts)
    for i, la in enumerate(names_sorted):
        for lb in names_sorted[i + 1 :]:
            if (la, lb) in adj:
                continue
            ga, gb = group.get(la), group.get(lb)
            if ga is not None and ga == gb:
                continue
            for pa in link_parts[la]:
                for pb in link_parts[lb]:
                    if collides(pa.prim, pa.transform, pb.prim, pb.transform, margin):
                        hits.add((la, lb, robot_name, robot_name))
    assert ROBOT_NAME  # 引用保持（多机器人场景里 obj 名就是机器人名）
    return hits


def test_check_config_matches_brute_force(workstation):
    """真实场景 + 多个位形：check_config 与暴力实现逐项一致。"""
    rng = np.random.default_rng(41)
    checker = CollisionChecker(workstation, margin=-20.0)
    checked = 0
    for _ in range(6):
        joints = list(SWEEP_BASE) + list(rng.uniform(-20.0, 20.0, 6))
        joints = [SWEEP_BASE[i] + float(rng.uniform(-20.0, 20.0)) for i in range(6)]
        got = {
            (h.link_a or "", h.link_b or "", h.obj_a, h.obj_b)
            for h in checker.check_config(joints, include_self=True)
        }
        expected = _brute_force_hits(workstation, joints, -20.0)
        assert got == expected, (joints, got, expected)
        checked += 1
    assert checked == 6


def test_check_config_matches_brute_force_strict_margin(workstation):
    """严格裕度（0.0）下也必须一致——正负裕度走的是不同的判定分支。"""
    checker = CollisionChecker(workstation, margin=0.0)
    joints = [10.0, -80.0, 100.0, -100.0, 80.0, 5.0]
    got = {
        (h.link_a or "", h.link_b or "", h.obj_a, h.obj_b)
        for h in checker.check_config(joints, include_self=True)
    }
    expected = _brute_force_hits(workstation, joints, 0.0)
    assert got == expected


def test_env_part_boxes_cache_invalidated(workstation):
    """环境零件盒随 invalidate_env 一起失效（对象动过之后不能拿旧盒）。"""
    checker = CollisionChecker(workstation, margin=-20.0)
    robot = Robot(str(UR5E), "UR5e")
    planner = TrajectoryPlanner(robot, scene=workstation)
    traj = planner.plan(sweep_program(robot, n_motions=10))
    # 零件盒只在广相判定"这一对要细看"时才需要——扫到位形真的逼近某个物体为止
    for wp in traj.waypoints[::25]:
        checker.check_config(list(wp.joints))
        if checker._env_part_boxes:
            break
    assert checker._env_part_boxes, "轨迹扫过场景后环境零件盒应当已缓存"
    checker.invalidate_env()
    assert checker._env_part_boxes == {}
    assert checker._env_cache == {}


# --------------------------------------------------------------------------- #
# 6. 采样口径
# --------------------------------------------------------------------------- #


class _WP:
    """只带 joints/timestamp 的最小路点（采样只看关节值）。"""

    def __init__(self, joints, timestamp=0.0):
        self.joints = list(joints)
        self.timestamp = timestamp


def _linear_waypoints(n, dof=6, per_step=1.0):
    return [_WP([i * per_step] * dof, float(i)) for i in range(n)]


def test_sample_indices_empty():
    assert sample_indices([]) == []
    assert sample_indices([], stride=5) == []
    assert sample_indices([], max_joint_step=5.0) == []


def test_sample_indices_default_policy_hits_target():
    idx = sample_indices(_linear_waypoints(3000))
    assert len(idx) <= COLLISION_SAMPLE_TARGET + 1
    assert idx[-1] == 2999
    assert idx == sorted(set(idx))
    assert idx[0] == 0


def test_sample_indices_default_density_thins_with_length():
    """默认口径的已知取舍：程序越长、采样点之间隔得越远。"""
    short = sample_indices(_linear_waypoints(600))
    long = sample_indices(_linear_waypoints(12000))
    assert 600 / len(short) < 12000 / len(long)


def test_sample_indices_stride_is_exact():
    idx = sample_indices(_linear_waypoints(10), stride=3)
    assert idx == [0, 3, 6, 9]
    idx = sample_indices(_linear_waypoints(11), stride=3)
    assert idx == [0, 3, 6, 9, 10], "末点必查"
    idx = sample_indices(_linear_waypoints(10), stride=1)
    assert idx == list(range(10)), "步长 1 不得重复末点"


def test_sample_indices_joint_step_guarantee():
    """关节步长口径的核心保证：相邻采样点之间任一关节变化 ≤ 步长。"""
    wps = _linear_waypoints(200, per_step=0.7)
    idx = sample_indices(wps, max_joint_step=5.0)
    assert idx[0] == 0 and idx[-1] == 199
    for a, b in zip(idx, idx[1:]):
        delta = max(abs(x - y) for x, y in zip(wps[a].joints, wps[b].joints))
        assert delta >= 5.0 or b == 199, (a, b, delta)
    assert idx == sorted(set(idx))


def test_sample_indices_joint_step_follows_motion_not_length():
    """关节步长口径：采样数由**运动量**决定，与路点数无关。

    同一条关节行程（500mm 等价的 500° 总行程）写成 500 个路点还是 5000 个
    路点，采样数不变；而默认口径的采样数只由路点数决定——两条曲线因此分道
    扬镳。这就是"检查密度不随程序变长而稀释"的确切含义。
    """
    wps_coarse = _linear_waypoints(500, per_step=1.0)
    wps_fine = _linear_waypoints(5000, per_step=0.1)
    coarse = sample_indices(wps_coarse, max_joint_step=10.0)
    fine = sample_indices(wps_fine, max_joint_step=10.0)
    assert abs(len(coarse) - len(fine)) <= 1, "同一行程、不同路点数 ⇒ 采样数相同"

    for idx, wps in ((coarse, wps_coarse), (fine, wps_fine)):
        for a, b in zip(idx, idx[1:]):
            span = max(abs(x - y) for x, y in zip(wps[a].joints, wps[b].joints))
            assert span <= 10.0 + 1e-9, (a, b, span)

    # 默认口径则相反：路点越多，采样数封顶在预算附近（密度被稀释）
    default_long = sample_indices(_linear_waypoints(12000))
    default_short = sample_indices(_linear_waypoints(1200))
    assert len(default_short) <= COLLISION_SAMPLE_TARGET + 1
    assert len(default_long) <= COLLISION_SAMPLE_TARGET + 1
    assert 12000 / len(default_long) > 1200 / len(default_short) * 5


def test_sample_indices_joint_step_zero_is_every_waypoint():
    assert sample_indices(_linear_waypoints(40), max_joint_step=0.0) == list(range(40))


def test_sample_indices_rejects_negative_step():
    with pytest.raises(ValueError):
        sample_indices(_linear_waypoints(10), max_joint_step=-1.0)
    with pytest.raises(ValueError):
        sample_indices(_linear_waypoints(10), max_joint_step=float("nan"))
    with pytest.raises(ValueError):
        sample_indices(_linear_waypoints(10), max_joint_step=float("inf"))


def test_sample_indices_tolerates_ragged_joint_counts():
    """关节数不一致（损坏轨迹）时按重叠部分比较，而不是崩掉。"""
    wps = [_WP([0.0, 0.0, 0.0]), _WP([0.0, 0.0]), _WP([50.0, 0.0, 0.0])]
    idx = sample_indices(wps, max_joint_step=10.0)
    assert idx[0] == 0 and idx[-1] == 2


def test_default_max_joint_step_constant_is_sane():
    assert 0.0 < DEFAULT_MAX_JOINT_STEP_DEG <= 30.0
    assert COLLISION_SAMPLE_TARGET == 300


# --------------------------------------------------------------------------- #
# 7. 采样口径接进规划器 / 校验器
# --------------------------------------------------------------------------- #


def test_check_trajectory_collisions_accepts_joint_step(planner):
    program = sweep_program(planner.robot, n_motions=20)
    traj = planner.plan(program)
    default = planner.check_trajectory_collisions(traj, margin=-20.0)
    denser = planner.check_trajectory_collisions(
        traj, margin=-20.0, max_joint_step=DEFAULT_MAX_JOINT_STEP_DEG
    )
    assert default and denser
    # 更密的采样只会发现更多（去重前的）接触，不会"发现更少"
    assert len(denser) >= len(default)


def test_check_trajectory_collisions_rejects_bad_step(planner):
    program = sweep_program(planner.robot, n_motions=5)
    traj = planner.plan(program)
    with pytest.raises(ValueError):
        planner.check_trajectory_collisions(traj, max_joint_step=-5.0)


def test_validate_default_sampling_unchanged(planner, no_collision_result_cache):
    """``validate`` 不传采样参数 ⇒ 走历史口径（不改变既有判定）。"""
    program = sweep_program(planner.robot, n_motions=10)
    traj = planner.plan(program)
    reset_collision_stats()
    planner.validate(program)
    checks = collision_stats()["config_checks"]
    expected = len(sample_indices(traj.waypoints))
    # validate 内部重新规划，路点数一致；采样数应与之相符
    assert checks == expected


def test_sample_indices_stride_matches_legacy_grid():
    """``stride`` 口径与改动前的 ``range(0, n, stride) + [n-1]`` 同一格子。"""
    n, stride = 37, 4
    legacy = list(range(0, n, stride)) + [n - 1]
    assert sample_indices(_linear_waypoints(n), stride=stride) == sorted(set(legacy))


# --------------------------------------------------------------------------- #
# 8. 工作量账本
# --------------------------------------------------------------------------- #


def test_collision_stats_consistent(planner, no_collision_result_cache):
    # v1.2.2：本用例的契约是"计数器反映真算的工作量"，与干涉检查**结果**缓存
    # 在语义上不兼容——命中路径不进 checker，计数器自然是 0。账本类测试一律
    # 绕过结果缓存（与 service.perf 的计时同一口径）。
    program = sweep_program(planner.robot, n_motions=20)
    traj = planner.plan(program)
    reset_collision_stats()
    hits = planner.check_trajectory_collisions(traj, margin=-20.0)
    stats = collision_stats()
    assert stats["config_checks"] == len(sample_indices(traj.waypoints))
    assert stats["level1_tests"] > 0
    assert 0 <= stats["level1_rejects"] <= stats["level1_tests"]
    assert stats["narrow_pairs"] > 0
    # 判定级原始命中次数由 geometry_stats 统一记（同一条幅会在多个采样点上重复计数）
    raw = geo.geometry_stats()["collision_hits"]
    assert raw >= len(hits) > 0


def test_geometry_stats_broad_phase_actually_rejects(planner, no_collision_result_cache):
    """广相必须真的在挡东西（否则这一层就是摆设）——也是性能结论的依据。
    （账本类测试，绕过 v1.2.2 的结果缓存，见 test_collision_stats_consistent。）"""
    program = sweep_program(planner.robot, n_motions=20)
    traj = planner.plan(program)
    reset_collision_stats()
    geo.reset_geometry_stats()
    planner.check_trajectory_collisions(traj, margin=-20.0)
    col = collision_stats()
    geos = geo.geometry_stats()
    assert col["level1_rejects"] / col["level1_tests"] > 0.5, "广相应挡掉多数候选对"
    assert geos["sat_checks"] < geos["collides_calls"], "窄相粗筛也要挡掉多数零件对"
    assert geos["cache_hits"] > geos["cache_misses"], "局部几何缓存应基本全命中"


def test_collision_stats_reset():
    reset_collision_stats()
    assert set(collision_stats()) == {
        "config_checks",
        "level1_tests",
        "level1_rejects",
        "narrow_pairs",
    }
    assert all(v == 0 for v in collision_stats().values())


def test_separation_still_finite_after_kernel_change(workstation):
    """``parts_separation`` 依赖穿透深度，改写后仍须给有限值（不是 NaN/inf）。"""
    from domain.world.components import Part

    parts_a = [Part(Box((50.0, 50.0, 50.0)), np.eye(4), (1.0, 1.0, 1.0))]
    tf = np.eye(4)
    tf[:3, 3] = [200.0, 0.0, 0.0]
    parts_b = [Part(Box((50.0, 50.0, 50.0)), tf, (1.0, 1.0, 1.0))]
    assert parts_separation(parts_a, parts_b) == pytest.approx(150.0)
    tf2 = np.eye(4)
    tf2[:3, 3] = [10.0, 0.0, 0.0]
    parts_c = [Part(Box((50.0, 50.0, 50.0)), tf2, (1.0, 1.0, 1.0))]
    assert parts_separation(parts_a, parts_c) == pytest.approx(-40.0)


# --------------------------------------------------------------------------- #
# 9. perf：干涉检查分段基准
# --------------------------------------------------------------------------- #


def test_collision_profile_rows_and_density(planner):
    program = sweep_program(planner.robot, n_motions=20)
    prof = collision_profile(planner, program, steps=(None, 20.0, 10.0), repeats=1)
    assert isinstance(prof, CollisionProfile)
    assert len(prof.rows) == 3
    assert prof.n_waypoints > 0
    assert prof.n_robot_parts > 0, "场景侧规模字段必须真的取到值"
    assert prof.n_env_objects > 0 and prof.n_env_parts >= prof.n_env_objects
    assert prof.margin < 0.0, "默认裕度是每机包络容差的负值"

    default, coarse, fine = prof.rows
    assert default.max_joint_step is None
    assert coarse.max_joint_step == 20.0
    assert fine.max_joint_step == 10.0
    # 步长越小 → 采样越多 → 每采样路点越少
    assert coarse.samples < fine.samples
    assert coarse.waypoints_per_sample > fine.waypoints_per_sample
    for row in prof.rows:
        assert isinstance(row, SamplingRow)
        assert row.samples > 0 and row.seconds > 0.0
        assert row.ms_per_sample > 0.0
        assert 0.0 <= row.broad_phase_reject_rate <= 1.0
        assert row.narrow_pairs > 0
        assert row.sat_checks <= row.narrow_pairs
        assert row.hits >= 0


def test_collision_profile_requires_scene():
    robot = Robot(str(UR5E), "UR5e")
    bare = TrajectoryPlanner(robot)
    program = sweep_program(robot, n_motions=3)
    with pytest.raises(ValueError):
        collision_profile(bare, program, repeats=1)


def test_collision_profile_to_dict(planner):
    program = sweep_program(planner.robot, n_motions=10)
    payload = collision_profile(planner, program, steps=(None, 10.0), repeats=1).to_dict()
    assert set(payload) >= {
        "robot",
        "waypoints",
        "commands",
        "margin",
        "robot_parts",
        "env_objects",
        "env_parts",
        "repeats",
        "rows",
    }
    assert len(payload["rows"]) == 2
    row = payload["rows"][0]
    assert set(row) >= {
        "label",
        "max_joint_step",
        "samples",
        "waypoints_per_sample",
        "seconds",
        "ms_per_sample",
        "hits",
        "level1_tests",
        "level1_rejects",
        "narrow_pairs",
        "sat_checks",
        "broad_phase_reject_rate",
    }
    assert json.loads(json.dumps(payload))  # 必须能序列化（网页要用）


def test_collision_profile_row_lookup(planner):
    program = sweep_program(planner.robot, n_motions=10)
    prof = collision_profile(planner, program, steps=(None,), repeats=1)
    assert prof.row(prof.rows[0].label) is prof.rows[0]
    assert prof.row("不存在的口径") is None


def test_plan_profile_reports_sampling_density(planner):
    """：class:`PlanProfile` 的干涉行必须写出采样密度（保证强弱不能隐形）。"""
    program = sweep_program(planner.robot, n_motions=10)
    prof = profile_program(planner, program, repeats=1)
    phase = prof.phase(i18n.tr("干涉检查"))
    assert phase is not None
    assert "路点 1 次" in phase.note, phase.note
    assert prof.to_dict()["phases"][i18n.tr("干涉检查")] > 0.0


def test_comparison_format_is_readable():
    cmp = compare_collision_kernels(n_prims=6, repeats=1)
    text = cmp.format()
    assert "完全相同" in text and "更快" in text and "0.000e+00" in text


# --------------------------------------------------------------------------- #
# 10. en 模式（库层消息必须零中文残留）
# --------------------------------------------------------------------------- #


def test_collision_profile_format_english(planner, monkeypatch):
    monkeypatch.setenv("CORE_LANG", "en")
    i18n.set_language("en")
    try:
        program = sweep_program(planner.robot, n_motions=10)
        prof = collision_profile(planner, program, steps=(None, 10.0), repeats=1)
        text = prof.format()
        assert not re.search(r"[\u4e00-\u9fff]", text), text
        assert "sampling policy" in text
    finally:
        i18n.set_language("zh")


def test_kernel_comparison_format_english(monkeypatch):
    monkeypatch.setenv("CORE_LANG", "en")
    i18n.set_language("en")
    try:
        cmp = compare_collision_kernels(n_prims=4, repeats=1)
        assert not re.search(r"[\u4e00-\u9fff]", cmp.format())
    finally:
        i18n.set_language("zh")


def test_sample_indices_error_english(monkeypatch):
    monkeypatch.setenv("CORE_LANG", "en")
    i18n.set_language("en")
    try:
        with pytest.raises(ValueError) as exc:
            sample_indices(_linear_waypoints(5), max_joint_step=-1.0)
        assert not re.search(r"[\u4e00-\u9fff]", str(exc.value))
    finally:
        i18n.set_language("zh")


# --------------------------------------------------------------------------- #
# 11. 示例脚本与 CLI（子进程真跑）
# --------------------------------------------------------------------------- #


def _run(cmd, cwd=None, timeout=300):
    env = dict(**__import__("os").environ)
    env["PYTHONPATH"] = str(REPO_ROOT) + ":" + str(REPO_ROOT)
    env["QT_QPA_PLATFORM"] = "offscreen"
    return subprocess.run(
        cmd,
        cwd=str(cwd or REPO_ROOT),
        env=env,
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def test_example_collision_perf_demo_runs():
    proc = _run(
        [sys.executable, "examples/collision_perf_demo.py", "--motions", "8", "--repeat", "1"]
    )
    assert proc.returncode == 0, proc.stderr[-2000:]
    out = proc.stdout
    assert "逐位一致" in out
    assert "干涉检查分段基准" in out
    assert "广相 AABB 挡掉" in out
    assert "这份基准不回答什么" in out


def test_cli_collision_flag_runs():
    proc = _run(
        [
            sys.executable,
            "-m",
            "service.perf",
            "--motions",
            "8",
            "--repeat",
            "1",
            "--collision",
            "--collision-steps",
            "10",
        ]
    )
    assert proc.returncode == 0, proc.stderr[-2000:]
    assert "干涉检查分段基准" in proc.stdout
    assert "关节步长 10°" in proc.stdout


def test_cli_collision_json_runs():
    proc = _run(
        [
            sys.executable,
            "-m",
            "service.perf",
            "--motions",
            "8",
            "--repeat",
            "1",
            "--collision",
            "--json",
        ]
    )
    assert proc.returncode == 0, proc.stderr[-2000:]
    payload = json.loads(proc.stdout)
    assert payload["collision"]["rows"]
    assert payload["collision"]["waypoints"] > 0


def test_collision_sample_target_documented():
    """采样预算与推荐步长是**写在明处的工程决定**，不许悄悄改数。"""
    doc = __import__("domain.collision", fromlist=["x"]).__doc__
    assert "采样口径" in doc
    assert math.isfinite(DEFAULT_MAX_JOINT_STEP_DEG)
