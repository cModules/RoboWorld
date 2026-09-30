"""力矩校验结果缓存（v1.2.3）的行为契约。

被测对象：

* :func:`domain.dynamics.dynamics_cache_key` —— 内容寻址键：模型内容签名
  （连杆惯性/末端负载/额定力矩）+ 逐路点关节位置/速度/加速度 + 指令时间区间；
* :class:`service.incremental.DynamicsResultCache` —— LRU 结果缓存骨架的实例化；
* :meth:`service.planner.TrajectoryPlanner._check_dynamics` 的缓存接线 ——
  命中与实算走同一条"应用到报表"路径，结论逐字段一致。

核心约定与 v1.2.2 干涉检查结果缓存同构：**改负载、改惯性、改程序都换键**，
不存在"改了负载还用旧力矩表"。
"""

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

# noqa: E402
from domain.robot import Robot  # noqa: E402
from domain.dynamics import dynamics_cache_key  # noqa: E402
from service.incremental import (
    DYNAMICS_CACHE_MAX_ENTRIES,
    DynamicsResultCache,
    get_dynamics_cache,
    resolve_dynamics_cache,
    set_dynamics_cache,
)
from service.planner import TrajectoryPlanner  # noqa: E402
from domain.world.scene import Scene  # noqa: E402

# 与 test_collision_cache.py 同款三连杆：几何给 Scene 用，动力学靠
# set_link_inertial 手动补（URDF 本体不带 <inertial>，正好用来测
# "缺惯性跳过分支不进缓存"）。effort 写死，额定力矩才可预期。
THREE_LINK_URDF = """<?xml version="1.0"?>
<robot name="three_link">
  <link name="base">
    <visual><origin xyz="0.25 0 0.05"/><geometry><box size="0.5 0.1 0.1"/></geometry></visual>
  </link>
  <link name="arm">
    <visual><origin xyz="0.3 0 0.05"/><geometry><box size="0.6 0.1 0.1"/></geometry></visual>
  </link>
  <link name="tool">
    <visual><origin xyz="-0.25 0 0.05"/><geometry><box size="0.5 0.1 0.1"/></geometry></visual>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="arm"/>
    <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="50"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="arm"/><child link="tool"/>
    <origin xyz="0.6 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="50"/>
  </joint>
</robot>
"""


@pytest.fixture(scope="module")
def three_link_urdf(tmp_path_factory):
    path = tmp_path_factory.mktemp("urdf") / "three_link.urdf"
    path.write_text(THREE_LINK_URDF)
    return str(path)


@pytest.fixture
def isolated_global_cache():
    """换上独立的动力学缓存实例，测完还原（不污染其他测试的全局状态）。"""
    saved = get_dynamics_cache()
    cache = DynamicsResultCache()
    set_dynamics_cache(cache)
    yield cache
    set_dynamics_cache(saved)


def make_planner(three_link_urdf, fill_inertials=True, **kwargs) -> TrajectoryPlanner:
    """全新三连杆规划器；``fill_inertials`` 给主链连杆手动补惯性参数。"""
    scene = Scene()
    scene.set_robot(three_link_urdf)
    robot = Robot(three_link_urdf, "three_link")
    planner = TrajectoryPlanner(scene=scene, robot=robot, **kwargs)
    if fill_inertials:
        planner.dynamics_model.set_link_inertial(
            "arm",
            mass=2.0,
            com=(0.0, 0.0, 0.1),
            inertia=[[0.001, 0, 0], [0, 0.001, 0], [0, 0, 0.001]],
        )
        planner.dynamics_model.set_link_inertial(
            "tool",
            mass=0.5,
            com=(0.0, 0.0, 0.05),
            inertia=[[0.0002, 0, 0], [0, 0.0002, 0], [0, 0, 0.0002]],
        )
    return planner


def prog_swing(robot):
    """干净摆动程序：从 (0,175) 摆到 (90,175) 再回来。

    起点必须显式摆好——规划从机器人**当前关节值**出发（与干涉检查测试
    同一条夹具教训；虽然力矩校验不看碰撞，但规划阶段仍要求位形合法）。
    """
    robot.set_joints([0.0, 175.0])
    return robot.moveJ([90.0, 175.0]).then(robot.moveJ([0.0, 175.0]))


def prog_other(robot):
    """另一段程序：不同的目标角——内容与 ``prog_swing`` 不同，键必须不同。"""
    robot.set_joints([0.0, 175.0])
    return robot.moveJ([-90.0, 100.0]).then(robot.moveJ([30.0, 175.0]))


def key_of(planner, program):
    traj = planner.plan(program)
    return dynamics_cache_key(planner.dynamics_model, traj, planner._motion_spans)


# --------------------------------------------------------------------------- #
# 1. 内容寻址键：改什么就该换什么
# --------------------------------------------------------------------------- #


def test_key_is_deterministic(three_link_urdf):
    p = make_planner(three_link_urdf)
    program = prog_swing(p.robot)
    assert key_of(p, program) == key_of(p, program)


def test_key_changes_with_program(three_link_urdf):
    p = make_planner(three_link_urdf)
    assert key_of(p, prog_swing(p.robot)) != key_of(p, prog_other(p.robot))


def test_key_changes_with_payload(three_link_urdf):
    p = make_planner(three_link_urdf)
    program = prog_swing(p.robot)
    before = key_of(p, program)
    p.dynamics_model.payload_mass = 1.0
    assert key_of(p, program) != before


def test_key_changes_with_inertial(three_link_urdf):
    p = make_planner(three_link_urdf)
    program = prog_swing(p.robot)
    before = key_of(p, program)
    p.dynamics_model.set_link_inertial(
        "arm",
        mass=2.5,
        com=(0.0, 0.0, 0.1),
        inertia=[[0.001, 0, 0], [0, 0.001, 0], [0, 0, 0.001]],
    )
    assert key_of(p, program) != before


def test_key_changes_with_spans(three_link_urdf):
    """spans 决定超限定位的 motion_index，进键防跨状态复用旧定位。"""
    p = make_planner(three_link_urdf)
    program = prog_swing(p.robot)
    traj = p.plan(program)
    spans = p._motion_spans
    base = dynamics_cache_key(p.dynamics_model, traj, spans)
    shifted = [(t0 + 0.5, t1 + 0.5) for t0, t1 in spans]
    assert dynamics_cache_key(p.dynamics_model, traj, shifted) != base


def test_key_changes_with_torque_limits(tmp_path):
    """额定力矩来自 URDF effort，改 effort 即换键（content_signature 覆盖）。"""
    heavy = THREE_LINK_URDF.replace('effort="50"', 'effort="80"')
    light_path = tmp_path / "light.urdf"
    heavy_path = tmp_path / "heavy.urdf"
    light_path.write_text(THREE_LINK_URDF)
    heavy_path.write_text(heavy)

    p_light = make_planner(str(light_path))
    p_heavy = make_planner(str(heavy_path))
    assert p_light.dynamics_model.content_signature() != p_heavy.dynamics_model.content_signature()


# --------------------------------------------------------------------------- #
# 2. 命中与等价性
# --------------------------------------------------------------------------- #


def test_second_identical_validate_hits_cache(three_link_urdf):
    p = make_planner(three_link_urdf)
    cache = DynamicsResultCache()
    p.dynamics_cache = cache
    program = prog_swing(p.robot)

    p.validate(program)
    assert cache.stats()["entries"] == 1
    p.validate(program)
    stats = cache.stats()
    assert stats["hits"] == 1
    assert stats["entries"] == 1


def test_hit_report_equals_uncached(three_link_urdf, isolated_global_cache):
    """缓存命中与全量实算的报表逐字段一致（同一条应用路径的硬约定）。"""
    uncached = make_planner(three_link_urdf, use_dynamics_cache=False)
    cached = make_planner(three_link_urdf)
    cache = DynamicsResultCache()
    cached.dynamics_cache = cache

    program = prog_swing(cached.robot)
    program_uncached = prog_swing(uncached.robot)

    r_cold = cached.validate(program)  # 实算（写缓存）
    r_hit = cached.validate(program)  # 命中
    r_ref = uncached.validate(program_uncached)  # 参考实算

    for report in (r_hit, r_ref):
        assert report.torque_peaks == r_cold.torque_peaks
        assert report.torque_limits == r_cold.torque_limits
        assert report.peak_power == pytest.approx(r_cold.peak_power)
        assert report.avg_power == pytest.approx(r_cold.avg_power)
    assert [i.code for i in r_hit.issues] == [i.code for i in r_cold.issues]
    assert [i.segment for i in r_hit.issues] == [i.segment for i in r_cold.issues]
    assert cache.stats()["hits"] == 1


def test_cached_result_is_shared_object(three_link_urdf):
    """命中交还的是存储的同一结果 dict（只读共享约定，容器不复制）。"""
    p = make_planner(three_link_urdf)
    cache = DynamicsResultCache()
    p.dynamics_cache = cache
    program = prog_swing(p.robot)

    p.validate(program)
    traj = p.plan(program)
    key = dynamics_cache_key(p.dynamics_model, traj, p._motion_spans)
    first = cache.lookup(key)
    assert cache.lookup(key) is first


def test_payload_change_is_miss(three_link_urdf):
    p = make_planner(three_link_urdf)
    cache = DynamicsResultCache()
    p.dynamics_cache = cache
    program = prog_swing(p.robot)

    p.validate(program)
    p.dynamics_model.payload_mass = 1.0
    p.validate(program)
    stats = cache.stats()
    assert stats["entries"] == 2
    assert stats["hits"] == 0


def test_inertial_change_is_miss(three_link_urdf):
    p = make_planner(three_link_urdf)
    cache = DynamicsResultCache()
    p.dynamics_cache = cache
    program = prog_swing(p.robot)

    p.validate(program)
    p.dynamics_model.set_link_inertial(
        "tool",
        mass=1.5,
        com=(0.0, 0.0, 0.05),
        inertia=[[0.0002, 0, 0], [0, 0.0002, 0], [0, 0, 0.0002]],
    )
    p.validate(program)
    assert cache.stats()["entries"] == 2


def test_program_change_is_miss(three_link_urdf):
    p = make_planner(three_link_urdf)
    cache = DynamicsResultCache()
    p.dynamics_cache = cache

    p.validate(prog_swing(p.robot))
    p.validate(prog_other(p.robot))
    stats = cache.stats()
    assert stats["entries"] == 2
    assert stats["hits"] == 0


def test_no_inertial_skip_does_not_cache(three_link_urdf):
    """缺惯性参数走跳过分支——没算结果，自然不能有东西进缓存。"""
    p = make_planner(three_link_urdf, fill_inertials=False)
    cache = DynamicsResultCache()
    p.dynamics_cache = cache
    program = prog_swing(p.robot)

    report = p.validate(program)
    assert any(i.code == "dynamics_skip" for i in report.issues)
    p.validate(program)
    assert cache.stats()["entries"] == 0
    assert cache.stats()["hits"] == 0


# --------------------------------------------------------------------------- #
# 3. 开关与优先级（三级裁决）
# --------------------------------------------------------------------------- #


def test_use_dynamics_cache_false_vetoes_instance(three_link_urdf):
    """三级裁决第一优先级：use_dynamics_cache=False 一票否决实例缓存。"""
    p = make_planner(three_link_urdf, use_dynamics_cache=False)
    cache = DynamicsResultCache()
    p.dynamics_cache = cache
    program = prog_swing(p.robot)

    p.validate(program)
    p.validate(program)
    assert cache.stats()["entries"] == 0
    assert resolve_dynamics_cache(p) is None


def test_instance_cache_wins_over_global(three_link_urdf, isolated_global_cache):
    """三级裁决第二优先级：实例缓存优先于全局默认。"""
    p = make_planner(three_link_urdf)
    mine = DynamicsResultCache()
    p.dynamics_cache = mine
    program = prog_swing(p.robot)

    p.validate(program)
    p.validate(program)
    assert mine.stats()["hits"] == 1
    assert isolated_global_cache.stats()["entries"] == 0


def test_falls_back_to_global_cache(three_link_urdf, isolated_global_cache):
    """三级裁决第三优先级：实例未给时用全局默认。"""
    p = make_planner(three_link_urdf)
    program = prog_swing(p.robot)

    p.validate(program)
    p.validate(program)
    assert isolated_global_cache.stats()["hits"] == 1


def test_global_cache_disabled_still_validates(three_link_urdf):
    """全局关闭（None）只是不缓存，校验本身照常出全量结论。"""
    saved = get_dynamics_cache()
    set_dynamics_cache(None)
    try:
        p = make_planner(three_link_urdf)
        program = prog_swing(p.robot)
        r1 = p.validate(program)
        r2 = p.validate(program)
        assert r1.torque_peaks == r2.torque_peaks
        assert resolve_dynamics_cache(p) is None
    finally:
        set_dynamics_cache(saved)


def test_resolve_default_when_nothing_set(three_link_urdf):
    """实例与全局都没给时，resolve 落到模块级默认缓存（默认开启）。"""
    p = make_planner(three_link_urdf)
    assert resolve_dynamics_cache(p) is get_dynamics_cache()


# --------------------------------------------------------------------------- #
# 4. LRU 骨架行为
# --------------------------------------------------------------------------- #


def test_lru_eviction(three_link_urdf):
    cache = DynamicsResultCache(max_entries=2)
    p = make_planner(three_link_urdf)
    p.dynamics_cache = cache

    p.validate(prog_swing(p.robot))
    p.validate(prog_other(p.robot))
    p.validate(
        (
            lambda r: (
                r.set_joints([0.0, 175.0]),
                r.moveJ([45.0, 120.0]).then(r.moveJ([-45.0, 175.0])),
            )[1]
        )(p.robot)
    )
    assert cache.stats()["entries"] == 2
    assert cache.stats()["evictions"] == 1


def test_store_same_key_is_noop(three_link_urdf):
    """同键重复存不覆盖、不计数（首次写后的内容保持权威）。"""
    cache = DynamicsResultCache()
    p = make_planner(three_link_urdf)
    program = prog_swing(p.robot)
    traj = p.plan(program)
    key = dynamics_cache_key(p.dynamics_model, traj, p._motion_spans)

    cache.store(key, {"marker": 1})
    cache.store(key, {"marker": 2})
    assert cache.lookup(key) == {"marker": 1}
    assert cache.stats()["stores"] == 1


def test_disabled_cache_lookup_store_noop(three_link_urdf):
    cache = DynamicsResultCache(enabled=False)
    assert cache.lookup(b"k") is None
    cache.store(b"k", {"x": 1})
    assert cache.lookup(b"k") is None
    assert cache.stats()["entries"] == 0


def test_max_entries_default():
    assert DYNAMICS_CACHE_MAX_ENTRIES == 64
    assert DynamicsResultCache().max_entries == DYNAMICS_CACHE_MAX_ENTRIES


# --------------------------------------------------------------------------- #
# 5. 计时诚实性（service.perf）
# --------------------------------------------------------------------------- #


def test_profile_program_does_not_touch_dynamics_cache(three_link_urdf):
    """基准报告的是真算耗时——profile_program 不得读写动力学结果缓存。"""
    from service.perf import profile_program

    p = make_planner(three_link_urdf)
    cache = DynamicsResultCache()
    p.dynamics_cache = cache
    program = prog_swing(p.robot)

    profile_program(p, program, check_collision=False, repeats=1)
    assert cache.stats()["entries"] == 0
    assert cache.stats()["hits"] == 0
    assert cache.stats()["misses"] == 0


def test_caches_off_restores_globals(three_link_urdf):
    """_caches_off 不带 planner：全局默认换成 None，退出原样还原。"""
    from service.perf import _caches_off

    dyn = DynamicsResultCache()
    set_dynamics_cache(dyn)
    try:
        with _caches_off():
            assert get_dynamics_cache() is None
            # CM 关的是"解析路径"——独立实例的方法本身不受影响
            dyn.store(b"k", {"x": 1})
            assert dyn.lookup(b"k") == {"x": 1}
        assert get_dynamics_cache() is dyn
    finally:
        set_dynamics_cache(None)


def test_caches_off_vetoes_instance_caches(three_link_urdf):
    """_caches_off(planner)：实例缓存高于全局，必须连实例开关一起否决。

    这是 v1.2.3 诚实性测试抓出的 v1.2.2 缺口的回归测试：只关全局时，
    自带实例缓存的规划器在基准计时里照样命中。
    """
    from service.perf import _caches_off

    p = make_planner(three_link_urdf)
    cache = DynamicsResultCache()
    p.dynamics_cache = cache
    program = prog_swing(p.robot)

    with _caches_off(p):
        p.validate(program)
        p.validate(program)
        assert cache.stats()["entries"] == 0
        assert p.use_dynamics_cache is False

    # 退出还原：开关回 True，缓存重新生效
    assert p.use_dynamics_cache is True
    p.validate(program)
    assert cache.stats()["entries"] == 1


# --------------------------------------------------------------------------- #
# 6. 顶层导出
# --------------------------------------------------------------------------- #
