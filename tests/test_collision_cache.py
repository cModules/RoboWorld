"""v1.2.2 干涉检查结果缓存：内容寻址、场景签名、副作用复现、基准诚实性。

夹具沿用 tests/test_collision.py 的三连杆最小场景（几何是手工设计的，
碰撞结论确定）：

* ``base`` 沿 +X 长条（x∈[0,500]mm，z∈[0,100]mm），``arm`` 同向铺开，
  ``tool`` 从 (600,0) 折回 -X——q2=0° 时 tool 压在 base 上（自碰撞），
  q2=180° 时甩到 +X 远端（干净）。
* 环境盒「路障」放在 (300,0,50)（与 base 重叠 → 任何位形都撞）或
  (800,0,300)（高处 → 永不撞）。

核心纪律（见 service/incremental.py 模块文档）：

* 键由**全部影响判定的输入**派生——命中就不可能给出过期结论；
  专门有一条「把工件拖进机器人路径后必须重新报干涉」的正确性测试。
* ``check_config`` 有把关节写回场景的历史契约，命中路径必须复现。
* 基准（``service.perf``）计时期间必须真算——缓存命中的"干涉检查"
  测的不是检查。
"""

import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from domain.robot import Robot  # noqa: E402
from domain.collision import collision_cache_key  # noqa: E402
from service.incremental import (
    CollisionResultCache,
    get_collision_cache,
    resolve_collision_cache,
    set_collision_cache,
)
from service.planner import TrajectoryPlanner  # noqa: E402
from domain.pose import Pose  # noqa: E402
from domain.world.scene import ROBOT_NAME, Scene  # noqa: E402

THREE_LINK_URDF = """<?xml version="1.0"?>
<robot name="three_link">
  <link name="base">
    <visual>
      <origin xyz="0.25 0 0.05" rpy="0 0 0"/>
      <geometry><box size="0.5 0.1 0.1"/></geometry>
    </visual>
  </link>
  <link name="arm">
    <visual>
      <origin xyz="0.3 0 0.05" rpy="0 0 0"/>
      <geometry><box size="0.6 0.1 0.1"/></geometry>
    </visual>
  </link>
  <link name="tool">
    <visual>
      <origin xyz="-0.25 0 0.05" rpy="0 0 0"/>
      <geometry><box size="0.5 0.1 0.1"/></geometry>
    </visual>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="arm"/><child link="tool"/>
    <origin xyz="0.6 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14"/>
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
    """换上独立的缓存实例，测完还原（不污染其他测试的全局状态）。"""
    saved = get_collision_cache()
    cache = CollisionResultCache()
    set_collision_cache(cache)
    yield cache
    set_collision_cache(saved)


def make_scene(three_link_urdf, box_xyz=None) -> Scene:
    """全新三连杆场景；``box_xyz`` 给了就放一个 100mm 路障盒。"""
    scene = Scene()
    scene.set_robot(three_link_urdf)
    if box_xyz is not None:
        scene.add(
            "workpiece_box",
            name="路障",
            pose=Pose.from_xyz_rpy(*box_xyz, 0.0, 0.0, 0.0),
            params={"length": 100.0, "width": 100.0, "height": 100.0},
        )
    return scene


def make_planner(scene, **kwargs) -> TrajectoryPlanner:
    return TrajectoryPlanner(
        scene=scene,
        **{
            "robot": Robot(_robot_urdf_of(scene), "three_link"),
            **kwargs,
        },
    )


def _robot_urdf_of(scene) -> str:
    return scene.robots[scene.robot_names[0]].urdf


def clean_program(robot):
    """干净程序：从 (0,175) 摆到 (90,175) 再回来，全程无干涉。

    起点必须显式摆干净——规划从机器人**当前关节值**出发，而零位 (0,0)
    正是 tool 折回压在 base 上的自碰撞位形（首轮跑测试抓出的夹具缺陷）。
    """
    robot.set_joints([0.0, 175.0])
    return robot.moveJ([90.0, 175.0]).then(robot.moveJ([0.0, 175.0]))


def hit_program(robot):
    """自碰撞程序：从零位 (0,0) 出发摆到 (0,175)——首个采样点即自碰撞。"""
    robot.set_joints([0.0, 0.0])
    return robot.moveJ([0.0, 175.0])


def check(planner, program, **kwargs):
    traj = planner.plan(program)
    return traj, planner.check_trajectory_collisions(traj, **kwargs)


# --------------------------------------------------------------------------- #
# 命中与等价性
# --------------------------------------------------------------------------- #


def test_second_identical_check_hits_cache(three_link_urdf, isolated_global_cache):
    scene = make_scene(three_link_urdf)
    planner = make_planner(scene)
    program = hit_program(planner.robot)

    _, first = check(planner, program)
    assert isolated_global_cache.stats()["misses"] == 1
    assert isolated_global_cache.stats()["stores"] == 1
    assert isolated_global_cache.stats()["hits"] == 0

    _, second = check(planner, program)
    assert isolated_global_cache.stats()["hits"] == 1
    assert second == first, "命中结果必须与实算完全一致"


def test_cached_result_equals_uncached(three_link_urdf, isolated_global_cache):
    """命中结果与「完全关掉缓存」的实算逐字段一致（等价性守卫）。"""
    scene_a = make_scene(three_link_urdf)
    scene_b = make_scene(three_link_urdf)
    planner_off = make_planner(scene_a, use_collision_cache=False)
    planner_on = make_planner(scene_b)
    program = hit_program(planner_off.robot)

    _, uncached = check(planner_off, program)
    _, cached = check(planner_on, program)  # 第一次：实算并存储
    _, again = check(planner_on, program)  # 第二次：命中
    assert again == uncached
    assert cached == uncached


def test_program_change_misses(three_link_urdf, isolated_global_cache):
    scene = make_scene(three_link_urdf)
    planner = make_planner(scene)
    _, hits_clean = check(planner, clean_program(planner.robot))
    assert hits_clean == []

    misses_before = isolated_global_cache.stats()["misses"]
    _, hits_hit = check(planner, hit_program(planner.robot))
    assert isolated_global_cache.stats()["misses"] == misses_before + 1
    assert len(hits_hit) > 0, "夹具前提失效：自碰撞程序应报干涉"


def test_different_trajectory_same_program_after_replan_hits(
    three_link_urdf, isolated_global_cache
):
    """同一程序重新 plan 出的轨迹内容相同 → 命中（规划缓存共享路点）。"""
    scene = make_scene(three_link_urdf)
    planner = make_planner(scene)
    program = hit_program(planner.robot)
    check(planner, program)
    hits_before = isolated_global_cache.stats()["hits"]
    _, second = check(planner, program)
    assert isolated_global_cache.stats()["hits"] == hits_before + 1
    assert len(second) > 0


# --------------------------------------------------------------------------- #
# 场景签名：场景一动键就变
# --------------------------------------------------------------------------- #


def test_object_moved_into_path_is_detected(three_link_urdf, isolated_global_cache):
    """核心正确性：把工件拖进机器人路径后必须重新报干涉，绝不能返回过期"无干涉"。"""
    scene = make_scene(three_link_urdf, box_xyz=(800.0, 0.0, 300.0))
    planner = make_planner(scene)
    program = clean_program(planner.robot)
    _, hits = check(planner, program)
    assert hits == []

    # 拖到机器人身体里（v1.1.7 拖拽摆放的等价操作）
    misses_before = isolated_global_cache.stats()["misses"]
    scene.objects["路障"].pose = Pose.from_xyz_rpy(300.0, 0.0, 50.0, 0, 0, 0)
    _, hits_after = check(planner, program)
    assert isolated_global_cache.stats()["misses"] == misses_before + 1
    assert len(hits_after) > 0, "场景变了还返回旧结论——缓存键漏了场景状态"


def test_object_moved_back_hits_original_result(three_link_urdf, isolated_global_cache):
    scene = make_scene(three_link_urdf, box_xyz=(800.0, 0.0, 300.0))
    planner = make_planner(scene)
    program = clean_program(planner.robot)
    _, original = check(planner, program)
    scene.objects["路障"].pose = Pose.from_xyz_rpy(300.0, 0.0, 50.0, 0, 0, 0)
    check(planner, program)
    scene.objects["路障"].pose = Pose.from_xyz_rpy(800.0, 0.0, 300.0, 0, 0, 0)
    _, again = check(planner, program)
    assert again == original, "拖回原处：几何与原键完全相同，应命中原结论"


def test_object_added_and_removed_change_key(three_link_urdf, isolated_global_cache):
    scene = make_scene(three_link_urdf, box_xyz=(800.0, 0.0, 300.0))
    planner = make_planner(scene)
    program = clean_program(planner.robot)
    _, base_hits = check(planner, program)

    scene.add(
        "workpiece_box",
        name="新障碍",
        pose=Pose.from_xyz_rpy(300.0, 0.0, 50.0, 0, 0, 0),
        params={"length": 100.0, "width": 100.0, "height": 100.0},
    )
    _, with_extra = check(planner, program)
    assert len(with_extra) > len(base_hits)

    scene.remove("新障碍")
    _, after_remove = check(planner, program)
    assert after_remove == base_hits


def test_invisible_object_excluded_from_key(three_link_urdf, isolated_global_cache):
    """可见性参与键：隐藏对象不再参与检查（与 _env_items 同一口径）。"""
    scene = make_scene(three_link_urdf, box_xyz=(300.0, 0.0, 50.0))
    planner = make_planner(scene)
    program = clean_program(planner.robot)
    _, visible = check(planner, program)
    assert len(visible) > 0

    scene.objects["路障"].visible = False
    _, hidden = check(planner, program)
    assert hidden == []
    assert isolated_global_cache.stats()["misses"] >= 2


def test_checked_robot_scene_joints_do_not_change_key(three_link_urdf, isolated_global_cache):
    """被查机器人在场景里的关节角不进键——轨迹逐路点驱动它。"""
    scene = make_scene(three_link_urdf, box_xyz=(800.0, 0.0, 300.0))
    planner = make_planner(scene)
    program = clean_program(planner.robot)
    check(planner, program)
    hits_before = isolated_global_cache.stats()["hits"]

    scene.set_robot_joints([45.0, 90.0])
    _, second = check(planner, program)
    assert isolated_global_cache.stats()["hits"] == hits_before + 1
    assert second == []


def test_robot_base_move_changes_key(three_link_urdf, isolated_global_cache):
    """机器人基座挪走：键必须变，且结论随新位置更新。"""
    scene = make_scene(three_link_urdf, box_xyz=(300.0, 0.0, 50.0))
    planner = make_planner(scene)
    program = clean_program(planner.robot)
    _, before = check(planner, program)
    assert len(before) > 0

    pose = scene.objects[ROBOT_NAME].pose.matrix.copy()
    pose[0, 3] += 1000.0  # 沿 X 挪出 1m，远离路障
    scene.objects[ROBOT_NAME].pose = Pose(matrix=pose)
    _, after = check(planner, program)
    assert after == [], "基座挪走后仍报干涉——键漏了基座位姿"


def test_parent_chain_move_changes_key(three_link_urdf, isolated_global_cache):
    """父对象移动带动子对象（世界位姿经父级链折进键）。"""
    scene = make_scene(three_link_urdf)
    # 垫块抬高到 tool 扫掠环带（半径 600~1100mm、z 0~100mm）之外
    scene.add(
        "workpiece_box",
        name="垫块",
        pose=Pose.from_xyz_rpy(800.0, 0.0, 300.0, 0, 0, 0),
        params={"length": 100.0, "width": 100.0, "height": 100.0},
    )
    scene.add(
        "workpiece_box",
        name="工件",
        pose=Pose.from_xyz_rpy(0.0, 0.0, 50.0, 0, 0, 0),
        params={"length": 50.0, "width": 50.0, "height": 50.0},
        parent="垫块",
    )
    planner = make_planner(scene)
    program = clean_program(planner.robot)
    _, first = check(planner, program)
    assert first == []

    misses_before = isolated_global_cache.stats()["misses"]
    pose = scene.objects["垫块"].pose.matrix.copy()
    pose[0, 3] = 300.0  # 连孩子一起拖进机器人身体（与 base 重叠）
    pose[2, 3] = 0.0
    scene.objects["垫块"].pose = Pose(matrix=pose)
    _, second = check(planner, program)
    assert isolated_global_cache.stats()["misses"] == misses_before + 1
    assert len(second) > 0


def test_mounted_on_checked_robot_changes_do_not_change_key(three_link_urdf, isolated_global_cache):
    """挂在被查机器人下的对象被豁免（与 _env_items 同一口径）——
    它的位姿不进键，改它不影响命中。"""
    scene = make_scene(three_link_urdf, box_xyz=(800.0, 0.0, 300.0))
    scene.add(
        "workpiece_box",
        name="吸着的工件",
        pose=Pose.from_xyz_rpy(0.0, 0.0, 50.0, 0, 0, 0),
        params={"length": 40.0, "width": 40.0, "height": 40.0},
        parent=ROBOT_NAME,
    )
    planner = make_planner(scene)
    program = clean_program(planner.robot)
    _, first = check(planner, program)
    hits_before = isolated_global_cache.stats()["hits"]

    pose = scene.objects["吸着的工件"].pose.matrix.copy()
    pose[2, 3] += 200.0
    scene.objects["吸着的工件"].pose = Pose(matrix=pose)
    _, second = check(planner, program)
    assert isolated_global_cache.stats()["hits"] == hits_before + 1
    assert second == first


# --------------------------------------------------------------------------- #
# 参数与口径
# --------------------------------------------------------------------------- #


def test_margin_change_misses(three_link_urdf, isolated_global_cache):
    scene = make_scene(three_link_urdf, box_xyz=(300.0, 0.0, 50.0))
    planner = make_planner(scene)
    program = clean_program(planner.robot)
    _, strict = check(planner, program, margin=0.0)
    _, absurd = check(planner, program, margin=-1000.0)
    assert len(strict) > 0
    assert absurd == [], "夹具前提失效：-1000mm 裕度应把 100mm 级穿透放行"
    assert isolated_global_cache.stats()["misses"] == 2


def test_stride_and_joint_step_change_misses(three_link_urdf, isolated_global_cache):
    scene = make_scene(three_link_urdf, box_xyz=(300.0, 0.0, 50.0))
    planner = make_planner(scene)
    program = clean_program(planner.robot)
    traj = planner.plan(program)
    for kwargs in ({"stride": 1}, {"max_joint_step": 0.0}):
        hits = planner.check_trajectory_collisions(traj, **kwargs)
        assert len(hits) > 0
    assert isolated_global_cache.stats()["misses"] >= 2


def test_key_is_deterministic(three_link_urdf):
    scene_a = make_scene(three_link_urdf, box_xyz=(800.0, 0.0, 300.0))
    scene_b = make_scene(three_link_urdf, box_xyz=(800.0, 0.0, 300.0))
    planner_a = make_planner(scene_a)
    planner_b = make_planner(scene_b)
    traj_a = planner_a.plan(clean_program(planner_a.robot))
    traj_b = planner_b.plan(clean_program(planner_b.robot))
    key_a = collision_cache_key(scene_a, None, traj_a, -0.5, 0, None)
    key_b = collision_cache_key(scene_b, None, traj_b, -0.5, 0, None)
    assert key_a == key_b
    assert key_a != collision_cache_key(scene_a, None, traj_a, 0.0, 0, None)
    assert key_a != collision_cache_key(scene_a, None, traj_a, -0.5, 3, None)


def test_key_changes_when_urdf_geometry_mutated_in_memory(three_link_urdf):
    """连杆几何被原地改（ JointSpec 数组可写同类风险）时键必须变。"""
    scene = make_scene(three_link_urdf, box_xyz=(800.0, 0.0, 300.0))
    planner = make_planner(scene)
    traj = planner.plan(clean_program(planner.robot))
    key_before = collision_cache_key(scene, None, traj, -0.5, 0, None)

    state = scene.robots[scene.robot_names[0]]
    _lname, part = state.geometry("collision")[0]
    part.transform[0, 3] += 12345.0  # 原地改零件变换
    key_after = collision_cache_key(scene, None, traj, -0.5, 0, None)
    assert key_before != key_after


# --------------------------------------------------------------------------- #
# 开关与隔离
# --------------------------------------------------------------------------- #


def test_use_collision_cache_false_bypasses(three_link_urdf, isolated_global_cache):
    scene = make_scene(three_link_urdf)
    planner = make_planner(scene, use_collision_cache=False)
    program = hit_program(planner.robot)
    _, first = check(planner, program)
    _, second = check(planner, program)
    assert first == second
    assert isolated_global_cache.stats() == {
        "enabled": True,
        "hits": 0,
        "misses": 0,
        "stores": 0,
        "evictions": 0,
        "entries": 0,
    }


def test_global_cache_disabled_bypasses(three_link_urdf):
    saved = get_collision_cache()
    set_collision_cache(None)
    try:
        scene = make_scene(three_link_urdf)
        planner = make_planner(scene)
        program = hit_program(planner.robot)
        _, first = check(planner, program)
        _, second = check(planner, program)
        assert first == second
    finally:
        set_collision_cache(saved)


def test_custom_cache_instance_is_used(three_link_urdf, isolated_global_cache):
    scene = make_scene(three_link_urdf)
    custom = CollisionResultCache()
    planner = make_planner(scene, collision_cache=custom)
    program = hit_program(planner.robot)
    check(planner, program)
    check(planner, program)
    assert custom.stats()["stores"] == 1
    assert custom.stats()["hits"] == 1
    # 全局缓存不被触碰
    assert isolated_global_cache.stats()["stores"] == 0


def test_disabled_custom_cache_bypasses(three_link_urdf):
    scene = make_scene(three_link_urdf)
    disabled = CollisionResultCache(enabled=False)
    planner = make_planner(scene, collision_cache=disabled)
    assert resolve_collision_cache(planner) is None
    program = hit_program(planner.robot)
    _, first = check(planner, program)
    _, second = check(planner, program)
    assert first == second
    assert disabled.stats()["entries"] == 0


# --------------------------------------------------------------------------- #
# 副作用复现与共享约定
# --------------------------------------------------------------------------- #


def test_hit_replicates_scene_joint_writeback(three_link_urdf, isolated_global_cache):
    """实算路径经 check_config 把最后采样点的关节写回场景（历史契约）；
    命中路径必须复现同一份写回，否则后续单点检查拿到过期位形。"""
    scene_off = make_scene(three_link_urdf, box_xyz=(800.0, 0.0, 300.0))
    planner_off = make_planner(scene_off, use_collision_cache=False)
    program = clean_program(planner_off.robot)
    traj_off, _ = check(planner_off, program)
    robot_name = scene_off.robot_names[0]
    joints_uncached = np.array(scene_off.robots[robot_name].joints_deg, dtype=np.float64)

    scene_on = make_scene(three_link_urdf, box_xyz=(800.0, 0.0, 300.0))
    planner_on = make_planner(scene_on)
    traj_on, _ = check(planner_on, program)  # 实算并存储
    check(planner_on, program)  # 命中：也应写回
    joints_cached = np.array(scene_on.robots[scene_on.robot_names[0]].joints_deg, dtype=np.float64)

    assert np.array_equal(joints_cached, joints_uncached)
    # 且确实等于最后一个采样点的关节值
    from domain.collision import sample_indices

    idx = sample_indices(traj_on.waypoints)
    assert np.allclose(
        joints_cached,
        np.asarray(traj_on.waypoints[idx[-1]].joints, dtype=np.float64),
    )


def test_hit_returns_new_list_container(three_link_urdf, isolated_global_cache):
    """列表容器每次新建；改容器不影响缓存（Collision 是冻结 dataclass，共享只读）。"""
    scene = make_scene(three_link_urdf)
    planner = make_planner(scene)
    program = hit_program(planner.robot)
    check(planner, program)
    first = planner.check_trajectory_collisions(planner.plan(program))
    first.append("垃圾")
    second = planner.check_trajectory_collisions(planner.plan(program))
    assert "垃圾" not in second
    assert len(second) > 0  # 缓存内容未被容器改动污染


def test_validate_twice_identical_reports(three_link_urdf, isolated_global_cache):
    """校验两次：报告完全一致，第二次的干涉检查来自缓存。"""
    from service.planner import TrajectoryPlanner as TP

    scene = make_scene(three_link_urdf, box_xyz=(300.0, 0.0, 50.0))
    planner = TP(Robot(three_link_urdf, "three_link"), scene=scene)
    program = clean_program(planner.robot)
    report_a = planner.validate(program)
    misses_before = isolated_global_cache.stats()["misses"]
    report_b = planner.validate(program)
    assert isolated_global_cache.stats()["misses"] == misses_before
    codes_a = sorted(str(i) for i in report_a.issues if i.code == "collision")
    codes_b = sorted(str(i) for i in report_b.issues if i.code == "collision")
    assert codes_a == codes_b
    assert len(codes_a) > 0


def test_lru_eviction(three_link_urdf):
    scene = make_scene(three_link_urdf, box_xyz=(800.0, 0.0, 300.0))
    cache = CollisionResultCache(max_entries=2)
    planner = make_planner(scene, collision_cache=cache)
    program = clean_program(planner.robot)
    traj = planner.plan(program)
    for margin in (0.0, -1.0, -2.0):
        planner.check_trajectory_collisions(traj, margin=margin)
    assert cache.stats()["entries"] == 2
    assert cache.stats()["evictions"] == 1
    # 最旧的 0.0 被淘汰：同键重查是未命中（重新实算，结论一致）
    again = planner.check_trajectory_collisions(traj, margin=0.0)
    assert again == []
    assert cache.stats()["stores"] == 4


def test_store_same_key_twice_is_noop():
    cache = CollisionResultCache()
    cache.store(b"k", [1, 2])
    cache.store(b"k", [3])
    assert cache.stats()["stores"] == 1
    assert cache.lookup(b"k") == [1, 2]


def test_clear_keeps_counters():
    cache = CollisionResultCache()
    cache.store(b"k", [1])
    cache.lookup(b"k")
    cache.clear()
    assert cache.stats()["entries"] == 0
    assert cache.stats()["hits"] == 1


def test_stats_roundtrip():
    cache = CollisionResultCache()
    cache.store(b"a", [])
    cache.lookup(b"a")
    cache.lookup(b"missing")
    stats = cache.stats()
    assert stats["stores"] == 1 and stats["hits"] == 1 and stats["misses"] == 1
    cache.reset_stats()
    assert cache.stats()["stores"] == 0
    assert cache.stats()["entries"] == 1  # 条目保留


# --------------------------------------------------------------------------- #
# 基准诚实性：perf 计时期间必须真算
# --------------------------------------------------------------------------- #


def test_perf_profile_does_not_touch_collision_cache(three_link_urdf, isolated_global_cache):
    """profile_program 的干涉/校验计时关掉结果缓存——命中测的不是检查。"""
    from service.perf import profile_program

    scene = make_scene(three_link_urdf, box_xyz=(300.0, 0.0, 50.0))
    planner = make_planner(scene)
    program = clean_program(planner.robot)
    profile_program(planner, program, repeats=2)
    assert isolated_global_cache.stats()["stores"] == 0
    assert isolated_global_cache.stats()["hits"] == 0


def test_perf_collision_profile_does_not_touch_cache(three_link_urdf, isolated_global_cache):
    from service.perf import collision_profile

    scene = make_scene(three_link_urdf, box_xyz=(300.0, 0.0, 50.0))
    planner = make_planner(scene)
    program = clean_program(planner.robot)
    collision_profile(planner, program, repeats=1)
    assert isolated_global_cache.stats()["stores"] == 0
    assert isolated_global_cache.stats()["hits"] == 0


# --------------------------------------------------------------------------- #
# 包级导出
# --------------------------------------------------------------------------- #


def test_resolve_defaults(three_link_urdf):
    planner = make_planner(make_scene(three_link_urdf))
    assert resolve_collision_cache(planner) is get_collision_cache()
