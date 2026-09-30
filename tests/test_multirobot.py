"""v1.0.1 多机器人协同：共享场景、互避碰、节拍统筹。

测试夹具里手工设计的 ``flag.urdf`` 是一自由度机构：

* ``base``——沿 +X 铺开的 0.4 m 长条，**只有** ``<collision>``
  （``<visual>`` 是网格）；
* ``arm``——经绕 Z 的转动关节 j1 接在原点，长 0.6 m，**只有** ``<visual>``。

这样设计有两个目的：

1. 几何尺度固定（最长 0.6 m）——机器人相距 5 m 时**必然**无干涉，
   基座重合时**必然**有干涉，两种判定都不依赖数值细节；
2. 一次覆盖 v1.0.1 的几何通道改动：``base`` 验证"``<collision>`` 优先"，
   ``arm`` 验证"某 link 没有 ``<collision>`` 时退用 ``<visual>``"。

真实 UR5e 用于核对量级：``<visual>`` 只剩 1 个 TCP 球，``<collision>`` 有
7 个厂商包络体（+1 个回退的 TCP 球），这正是"只读 ``<visual>`` 等于机器人
只剩一个球参与干涉检查"那个缺陷的现场。

``examples/multi_robot_cell_demo.py`` 以子进程真实跑一遍（缓存复用），
断言退出码、分段输出与结构化摘要——多机器人链路只有端到端跑通才算数。
"""

import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from domain.collision import (
    CollisionChecker,
    parts_separation,
    prim_separation,
)
from service.multirobot import (
    STRATEGIES,
    RobotJob,
    build_job,
    scan_timeline,
    schedule_cell,
)
from domain.pose import Pose  # noqa: E402
from service.simulation import Trajectory, Waypoint  # noqa: E402
from domain.world.components import Part  # noqa: E402
from domain.world.geometry import Box, Sphere  # noqa: E402
from domain.world.scene import (
    GEOMETRY_COLLISION,
    GEOMETRY_VISUAL,
    ROBOT_NAME,
    RobotState,
    Scene,
    load_robot_collision_geometry,
    load_robot_visuals,
)

UR5E = REPO_ROOT / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"
DEMO = REPO_ROOT / "examples" / "multi_robot_cell_demo.py"

FLAG_URDF = """<?xml version="1.0"?>
<robot name="flag">
  <link name="base">
    <collision>
      <origin xyz="0.2 0 0" rpy="0 0 0"/>
      <geometry><box size="0.4 0.1 0.1"/></geometry>
    </collision>
    <visual>
      <origin xyz="0.2 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="package://flag/base.stl"/></geometry>
    </visual>
  </link>
  <link name="arm">
    <visual>
      <origin xyz="0.3 0 0" rpy="0 0 0"/>
      <geometry><box size="0.6 0.1 0.1"/></geometry>
    </visual>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.15" upper="3.15"/>
  </joint>
</robot>
"""


@pytest.fixture(scope="module")
def flag_urdf(tmp_path_factory) -> str:
    """一自由度测试用 URDF（见模块 docstring）。"""
    path = tmp_path_factory.mktemp("urdf") / "flag.urdf"
    path.write_text(FLAG_URDF, encoding="utf-8")
    return str(path)


def _traj(*points: tuple[float, float]) -> Trajectory:
    """一自由度轨迹：``(时刻 s, 关节角 deg)`` 序列。"""
    home = Pose()
    return Trajectory([Waypoint(np.array([deg], dtype=float), home, float(t)) for t, deg in points])


def _home_traj(joints: float, duration: float) -> Trajectory:
    """停在 ``joints`` 度不动的轨迹。"""
    return _traj((0.0, joints), (duration, joints))


def _two_robot_scene(flag_urdf, gap_mm=0.0, joints_a=0.0, joints_b=0.0):
    """两台 flag 机器人：A 在原点，B 沿 +X 偏移 ``gap_mm``。"""
    scene = Scene()
    a = scene.add_robot(flag_urdf, [joints_a], name="机器人A")
    b = scene.add_robot(
        flag_urdf,
        [joints_b],
        name="机器人B",
        pose=Pose.from_xyz_rpy(gap_mm, 0.0, 0.0, 0.0, 0.0, 0.0),
    )
    return scene, a, b


def _jobs(scene, jobs: list[RobotJob]) -> list[RobotJob]:
    """按名字取同名机器人重建作业（``schedule_cell`` 会就地改 ``start``）。"""
    return [RobotJob(robot=j.robot, trajectory=j.trajectory, label=j.label) for j in jobs]


# --------------------------------------------------------------------------- #
# 1. URDF 几何通道：<collision> 优先 + 逐 link 回退 <visual>
# --------------------------------------------------------------------------- #


def test_collision_geometry_prefers_collision_and_falls_back(flag_urdf):
    """``<collision>`` 有就用它；某 link 缺 ``<collision>`` 时退用 ``<visual>``。"""
    collision = load_robot_collision_geometry(flag_urdf)
    # base 取自 <collision>，arm 由 <visual> 回退补上（文档顺序）
    assert [link for link, _ in collision] == ["base", "arm"]
    assert all(isinstance(p.prim, Box) for _, p in collision)
    # visual 通道里 base 的 <mesh> 被跳过，只剩 arm
    assert [link for link, _ in load_robot_visuals(flag_urdf)] == ["arm"]


def test_ur5e_collision_channel_has_real_envelopes():
    """UR5e：``<visual>`` 只有 1 个 TCP 球，``<collision>`` 才是真几何。"""
    visuals = load_robot_visuals(UR5E)
    collision = load_robot_collision_geometry(UR5E)
    assert len(collision) > len(visuals)
    assert len(collision) >= 7
    links = {link for link, _ in collision}
    for required in ("base_link", "link_2", "link_5", "tool0", "tcp"):
        assert required in links, f"缺少连杆几何: {required}"
    # tcp 只有视觉球 → 回退路径必须把它带上，否则工具末端从检查里消失
    assert "tcp" in {link for link, _ in visuals}


def test_robot_state_geometry_source_switch():
    """``RobotState.geometry()`` 按用途取几何；collision 缺失时两边都退 visual。"""
    box = Part(Box((10.0, 10.0, 10.0)), np.eye(4), (1.0, 0.0, 0.0))
    sphere = Part(Sphere(5.0), np.eye(4), (0.0, 1.0, 0.0))

    full = RobotState(name="r", urdf="x", visuals=[sphere], collision_parts=[box])
    assert full.geometry(GEOMETRY_COLLISION) == [box]
    assert full.geometry(GEOMETRY_VISUAL) == [sphere]

    degraded = RobotState(name="r", urdf="x", visuals=[sphere], collision_parts=[])
    assert degraded.geometry(GEOMETRY_COLLISION) == [sphere]
    assert degraded.geometry(GEOMETRY_VISUAL) == [sphere]

    assert RobotState(name="r", urdf="x").dof == 0


# --------------------------------------------------------------------------- #
# 2. 多机器人场景
# --------------------------------------------------------------------------- #


def test_add_robot_assigns_unique_names(flag_urdf):
    scene = Scene()
    a = scene.add_robot(flag_urdf)
    b = scene.add_robot(flag_urdf)
    assert a == ROBOT_NAME
    assert b != a
    assert scene.robot_names == [a, b]
    assert scene.is_robot(a) and scene.is_robot(b)
    assert not scene.is_robot("工作台")
    assert scene.has_robot


def test_add_robot_keeps_per_robot_state(flag_urdf):
    """每台机器人各有自己的关节角与运动学链，互不干扰。"""
    scene = Scene()
    a = scene.add_robot(flag_urdf, [0.0], name="机器人A")
    b = scene.add_robot(flag_urdf, [90.0], name="机器人B")

    assert scene.robots[a].joints_deg == [0.0]
    assert scene.robots[b].joints_deg == [90.0]
    assert scene.robots[a].dof == 1

    scene.set_robot_joints([30.0], name=b)
    assert scene.robots[b].joints_deg == [30.0]
    assert scene.robots[a].joints_deg == [0.0]  # A 不受影响

    assert scene.kinematics_of(a) is scene.robots[a].kinematics
    assert scene.kinematics is scene.kinematics_of(a)  # 主机器人
    assert scene.kinematics_of("不存在") is None


def test_set_robot_is_single_robot_history_surface(flag_urdf):
    """``set_robot`` 装载主机器人，且旧属性名全部代理到它。"""
    scene = Scene()
    scene.set_robot(flag_urdf, [0.0])
    assert scene.robot_names == [ROBOT_NAME]
    assert scene.robot_urdf == flag_urdf
    assert scene._robot_joints_deg == [0.0]
    assert [link for link, _ in scene._robot_visuals] == ["arm"]
    assert scene._kinematics is scene.kinematics

    # 代理属性可写，写进的是同一个 RobotState
    scene._robot_joints_deg = [45.0]
    assert scene.robots[ROBOT_NAME].joints_deg == [45.0]
    scene.robot_urdf = flag_urdf
    assert scene.robots[ROBOT_NAME].urdf == flag_urdf


def test_add_robot_without_name_never_overwrites_set_robot(flag_urdf):
    """``set_robot`` 后 ``add_robot`` 不能把主机器人顶掉。"""
    scene = Scene()
    scene.set_robot(flag_urdf, [10.0])
    second = scene.add_robot(flag_urdf, [20.0])
    assert second != ROBOT_NAME
    assert scene.robots[ROBOT_NAME].joints_deg == [10.0]
    assert scene.robots[second].joints_deg == [20.0]
    assert len(scene.robots) == 2


def test_remove_robot_default_removes_primary(flag_urdf):
    scene, a, b = _two_robot_scene(flag_urdf)
    scene.remove_robot()
    assert scene.robot_names == [b]


def test_remove_robot_keeps_object_shell_and_raises_when_absent(flag_urdf):
    scene, a, b = _two_robot_scene(flag_urdf)
    scene.remove_robot(b)
    assert scene.robot_names == [a]
    assert b in scene.objects, "机器人对象壳应保留（基座位姿仍可拖动/存盘）"
    assert not scene.is_robot(b)
    with pytest.raises(KeyError):
        scene.remove_robot(b)


def test_remove_object_of_robot_also_drops_robot(flag_urdf):
    scene, a, b = _two_robot_scene(flag_urdf)
    scene.remove(a)
    assert a not in scene.robots
    assert a not in scene.objects
    assert scene.robot_names == [b]


def test_clear_keeps_robot_shells(flag_urdf):
    scene, a, b = _two_robot_scene(flag_urdf)
    scene.add("worktable", name="工作台")
    scene.clear()
    assert scene.robots == {}
    assert not scene.has_robot
    assert a in scene.objects and b in scene.objects
    assert "工作台" not in scene.objects


def test_flange_transform_is_per_robot(flag_urdf):
    """法兰位形按各机器人**自己的**关节角算（返回基坐标系，不含基座位姿）。"""
    scene, a, b = _two_robot_scene(flag_urdf, gap_mm=1000.0)
    scene.set_robot_joints([90.0], name=a)
    scene.set_robot_joints([0.0], name=b)

    assert np.allclose(scene.flange_transform(b), np.eye(4))
    rotated = np.array(
        [[0.0, -1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    )
    assert np.allclose(scene.flange_transform(a), rotated)

    # 改 B 的关节不能影响 A 的法兰
    scene.set_robot_joints([-45.0], name=b)
    assert np.allclose(scene.flange_transform(a), rotated)


def test_world_link_parts_adds_base_pose_per_robot(flag_urdf):
    """世界系几何 = 基座系几何左乘基座位姿——互避碰必须用世界系。"""
    scene, a, b = _two_robot_scene(flag_urdf, gap_mm=1000.0)
    base_a = scene.robot_link_parts(a, [0.0])
    world_a = scene.robot_world_link_parts(a, [0.0])
    world_b = scene.robot_world_link_parts(b, [0.0])

    # A 在原点：基座系与世界系一致
    assert np.allclose(base_a["arm"][0].transform, world_a["arm"][0].transform)
    # B 偏移 1000mm：世界系里也整体偏移 1000mm（基座系看不出来）
    shift = world_b["arm"][0].transform[:3, 3] - world_a["arm"][0].transform[:3, 3]
    assert np.allclose(shift, [1000.0, 0.0, 0.0])


def test_scene_repr_lists_robots(flag_urdf):
    scene, a, b = _two_robot_scene(flag_urdf)
    text = repr(scene)
    assert a in text and b in text


# --------------------------------------------------------------------------- #
# 3. 持久化
# --------------------------------------------------------------------------- #


def test_single_robot_to_dict_has_no_robots_array(flag_urdf):
    scene = Scene()
    scene.set_robot(flag_urdf, [30.0])
    data = scene.to_dict()
    assert "robots" not in data, "单机器人时保持旧格式，旧版本要能读"
    assert data["robot_urdf"] == flag_urdf
    assert data["robot_joints_deg"] == [30.0]


def test_single_robot_roundtrip(flag_urdf):
    scene = Scene()
    scene.set_robot(flag_urdf, [30.0])
    scene.add("worktable", name="工作台")
    back = Scene.from_dict(json.loads(json.dumps(scene.to_dict())))
    assert back.robot_names == [ROBOT_NAME]
    assert back.robots[ROBOT_NAME].joints_deg == [30.0]
    assert "工作台" in back.objects


def test_multi_robot_roundtrip_keeps_scene_objects(flag_urdf):
    """双机器人场景存盘再打开，底座/工件不能丢。

    回归：``from_dict`` 的多机器人分支曾在恢复对象**之前**就 ``return``，
    于是文件里明明写着 ``objects``，加载后却一个都不剩。
    """
    scene = Scene()
    scene.add_robot(flag_urdf, [10.0], name="机器人A")
    scene.add_robot(flag_urdf, [20.0], name="机器人B")
    scene.add("worktable", name="底座")

    data = scene.to_dict()
    assert [o["name"] for o in data["objects"]] == ["底座"]

    back = Scene.from_dict(json.loads(json.dumps(data)))

    assert "底座" in back.objects, "双机器人场景加载后非机器人对象丢失"
    assert back.robot_names == ["机器人A", "机器人B"]


def test_multi_robot_roundtrip_keeps_names_joints_and_poses(flag_urdf):
    scene = Scene()
    a = scene.add_robot(flag_urdf, [10.0], name="机器人A")
    b = scene.add_robot(
        flag_urdf,
        [20.0],
        name="机器人B",
        pose=Pose.from_xyz_rpy(1200.0, 0.0, 0.0, 0.0, 0.0, 90.0),
    )
    scene.add("worktable", name="工作台")

    data = json.loads(json.dumps(scene.to_dict()))
    assert [raw["name"] for raw in data["robots"]] == [a, b]
    assert all(raw["name"] != a for raw in data["objects"]), "机器人不进 objects 列表"
    # 主机器人仍写旧键，旧版本读同一份文件不至于丢机器人
    assert data["robot_urdf"] == flag_urdf
    assert data["robot_joints_deg"] == [10.0]

    back = Scene.from_dict(data)
    assert back.robot_names == [a, b]
    assert back.robots[a].joints_deg == [10.0]
    assert back.robots[b].joints_deg == [20.0]
    assert np.allclose(back.objects[b].pose.matrix, scene.objects[b].pose.matrix, atol=1e-9)


def test_from_dict_falls_back_to_shell_when_urdf_missing(tmp_path):
    """URDF 路径失效时保留记录与对象壳，不构建运动学链（不抛异常）。"""
    data = {
        "format_version": 1,
        "robots": [
            {"name": "机器人A", "urdf": str(tmp_path / "丢失.urdf"), "joints_deg": []},
            {"name": "机器人B", "urdf": str(tmp_path / "丢失.urdf"), "joints_deg": []},
        ],
        "objects": [],
    }
    scene = Scene.from_dict(data)
    assert scene.robot_names == ["机器人A", "机器人B"]
    assert scene.robots["机器人A"].kinematics is None
    assert scene.robots["机器人A"].urdf.endswith("丢失.urdf")
    # 无运动学链 → 无几何 → 不参与干涉检查（而不是崩）
    assert scene.robot_world_link_parts("机器人A") == {}


# --------------------------------------------------------------------------- #
# 4. 机器人-机器人干涉与最小间距
# --------------------------------------------------------------------------- #


def test_robot_pair_far_apart_is_clean(flag_urdf):
    scene, a, b = _two_robot_scene(flag_urdf, gap_mm=5000.0)
    checker = CollisionChecker(scene)
    assert checker.check_robot_pair(a, b) == []
    assert checker.check_robot_pairs() == []

    dist, la, lb = checker.separation_between(a, b)
    assert np.isfinite(dist) and dist > 1000.0
    assert la and lb, "净空时也要给出最近连杆对"


def test_robot_pair_same_base_collides(flag_urdf):
    scene, a, b = _two_robot_scene(flag_urdf)
    checker = CollisionChecker(scene)
    hits = checker.check_robot_pair(a, b)
    assert hits
    assert all(h.obj_a == a and h.obj_b == b for h in hits)
    assert all(h.link_a and h.link_b for h in hits)

    dist, la, lb = checker.separation_between(a, b)
    assert dist < 0.0, "基座重合时应有穿透（负间距）"
    assert la and lb


def test_robot_pair_needs_two_distinct_robots(flag_urdf):
    scene, a, b = _two_robot_scene(flag_urdf)
    checker = CollisionChecker(scene)
    assert checker.check_robot_pair(a, a) == []
    # 单机器人场景做两两检查 = 没有对
    single = Scene()
    single.set_robot(flag_urdf)
    assert CollisionChecker(single).check_robot_pairs() == []


def test_check_robot_pair_uses_explicit_joints_without_touching_state(flag_urdf):
    scene, a, b = _two_robot_scene(flag_urdf, gap_mm=5000.0)
    checker = CollisionChecker(scene)
    before = list(scene.robots[b].joints_deg)
    # 只改传参不能改场景状态（节拍统筹要在同一时刻比较两台机器人的位形）
    checker.check_robot_pair(a, b, [0.0], [180.0])
    assert scene.robots[b].joints_deg == before


def test_scene_check_collisions_auto_enables_robot_pairs(flag_urdf):
    single = Scene()
    single.set_robot(flag_urdf)
    assert single.check_collisions() == []

    scene, a, b = _two_robot_scene(flag_urdf)
    hits = scene.check_collisions()
    assert any(h.obj_a == a and h.obj_b == b for h in hits)

    off = scene.check_collisions(include_robot_robot=False)
    assert not [h for h in off if h.obj_a == a and h.obj_b == b]


def test_mounted_object_exempt_only_for_its_own_robot(flag_urdf):
    """挂在 A 下的工具：对 A 豁免，对 B 仍要检查。"""
    scene, a, b = _two_robot_scene(flag_urdf)
    scene.add("tool_gripper", name="A的夹爪")
    scene.attach("A的夹爪", a, keep_world=False)

    assert scene.mounted_robot("A的夹爪") == a
    assert scene.mounted_robot(a) is None

    checker = CollisionChecker(scene)
    assert [n for n, _, _ in checker._env_items(a)] == []
    assert [n for n, _, _ in checker._env_items(b)] == ["A的夹爪"]

    # 行为上也一致：A 报不出与自家工具的干涉，B 能
    assert [h for h in checker.check_config(robot=a) if h.obj_b == "A的夹爪"] == []
    assert [h for h in checker.check_config(robot=b) if h.obj_b == "A的夹爪"]


def test_mounted_robot_walks_through_grandchildren(flag_urdf):
    scene, a, b = _two_robot_scene(flag_urdf)
    scene.add("tool_gripper", name="夹爪")
    scene.attach("夹爪", b, keep_world=False)
    scene.add("workpiece_box", name="工件", parent="夹爪")
    assert scene.mounted_robot("工件") == b


# --------------------------------------------------------------------------- #
# 5. 分离距离度量
# --------------------------------------------------------------------------- #


def test_prim_separation_sphere_sphere_is_analytic():
    a = Sphere(10.0)
    b = Sphere(10.0)
    near = np.eye(4)
    far = np.eye(4)
    far[:3, 3] = [100.0, 0.0, 0.0]
    assert prim_separation(a, near, b, near) == pytest.approx(-20.0)
    assert prim_separation(a, near, b, far) == pytest.approx(80.0)


def test_parts_separation_empty_group_is_infinite():
    part = Part(Box((10.0, 10.0, 10.0)), np.eye(4), (1.0, 1.0, 1.0))
    assert parts_separation([], [part]) == float("inf")
    assert parts_separation([part], []) == float("inf")


# --------------------------------------------------------------------------- #
# 6. RobotJob
# --------------------------------------------------------------------------- #


def test_robot_job_basic_fields(flag_urdf):
    job = build_job("机器人A", _traj((0.0, 0.0), (2.5, 90.0)), start=1.5)
    assert job.robot == "机器人A"
    assert job.name == "机器人A"  # label 为空时回退机器人名
    assert job.duration == pytest.approx(2.5)
    assert job.end == pytest.approx(4.0)

    labeled = build_job("机器人A", _traj((0.0, 0.0), (1.0, 0.0)), label="取件")
    assert labeled.name == "取件"


def test_robot_job_joints_at_clamps_outside_window():
    """区间外夹紧到两端位形——待机位形也要参与干涉检查。"""
    job = build_job("机器人A", _traj((0.0, 10.0), (2.0, 50.0)), start=3.0)
    assert job.joints_at(0.0)[0] == pytest.approx(10.0)  # 未开工 → 首路点
    assert job.joints_at(3.0)[0] == pytest.approx(10.0)  # 开工瞬间
    assert job.joints_at(4.0)[0] == pytest.approx(30.0)  # 中段插值
    assert job.joints_at(9.0)[0] == pytest.approx(50.0)  # 收工后 → 末路点


def test_robot_job_empty_trajectory_has_no_pose():
    job = build_job("机器人A", Trajectory([]))
    assert job.joints_at(0.0) is None
    assert job.duration == 0.0
    assert job.end == 0.0


# --------------------------------------------------------------------------- #
# 7. scan_timeline
# --------------------------------------------------------------------------- #


def test_scan_timeline_reports_robot_robot_interference(flag_urdf):
    scene, a, b = _two_robot_scene(flag_urdf)
    jobs = [
        build_job(a, _home_traj(180.0, 3.0)),
        build_job(b, _home_traj(0.0, 2.0)),
    ]
    robot_hits, env_hits = scan_timeline(scene, jobs, dt=0.25)
    assert robot_hits and not env_hits
    assert all(h.kind == "robot-robot" for h in robot_hits)
    assert all(h.robot_a == a and h.other == b for h in robot_hits)
    # 每个（连杆对）只记最早一次，不能被上百个采样点刷屏
    pairs = {(h.link_a, h.link_b) for h in robot_hits}
    assert len(pairs) == len(robot_hits)
    assert [h.time for h in robot_hits] == sorted(h.time for h in robot_hits)


def test_scan_timeline_checks_idle_robot_pose(flag_urdf):
    """B 还没开工也应被检查：它停在首路点位形上，那是一个真实位姿。"""
    scene, a, b = _two_robot_scene(flag_urdf)
    jobs = [
        build_job(a, _home_traj(180.0, 0.0)),
        build_job(b, _home_traj(0.0, 0.0), start=5.0),
    ]
    robot_hits, _ = scan_timeline(scene, jobs, dt=0.1)
    assert robot_hits, "B 待机在 0° 时与 A 的 180° 位形重叠，必须报出来"
    assert robot_hits[0].time == pytest.approx(0.0)


def test_scan_timeline_check_env_switch(flag_urdf):
    scene, a, b = _two_robot_scene(flag_urdf, gap_mm=5000.0)
    scene.add("workpiece_box", pose=Pose(), name="工件")
    # 前提：工件确实压在 A 的几何里，否则本测试失去意义
    assert CollisionChecker(scene).check_config(robot=a), "夹具前提失效"

    jobs = [
        build_job(a, _home_traj(0.0, 2.0)),
        build_job(b, _home_traj(0.0, 2.0)),
    ]
    robot_hits, env_hits = scan_timeline(scene, jobs, dt=0.25)
    assert not robot_hits and env_hits
    assert all(h.kind == "robot-env" and h.link_b is None for h in env_hits)

    _, off_hits = scan_timeline(scene, jobs, dt=0.25, check_env=False)
    assert off_hits == []


def test_scan_timeline_clearance_flags_near_misses(flag_urdf):
    """净空预警：给足裕度后，本来"不撞"的两台机器人会靠近到被记一条。"""
    scene, a, b = _two_robot_scene(flag_urdf, gap_mm=5000.0)
    jobs = [
        build_job(a, _home_traj(0.0, 2.0)),
        build_job(b, _home_traj(0.0, 2.0)),
    ]
    tight, _ = scan_timeline(scene, jobs, dt=0.5, clearance=0.0)
    assert tight == []
    # 间距 4400mm < 6000mm 裕度 → 记一条预警
    loose, _ = scan_timeline(scene, jobs, dt=0.5, clearance=6000.0)
    assert loose and all(h.separation_mm <= 6000.0 for h in loose)


def test_scan_timeline_empty_jobs():
    scene = Scene()
    assert scan_timeline(scene, [], dt=0.1) == ([], [])


# --------------------------------------------------------------------------- #
# 8. schedule_cell：四种策略
# --------------------------------------------------------------------------- #


def test_schedule_parallel_and_serial_makespan(flag_urdf):
    """相距 5 m：并发总节拍 = 最长一条；串行 = 两条之和。"""
    scene, a, b = _two_robot_scene(flag_urdf, gap_mm=5000.0)
    jobs = [
        build_job(a, _home_traj(0.0, 4.0)),
        build_job(b, _home_traj(0.0, 2.0)),
    ]

    par = schedule_cell(scene, _jobs(scene, jobs), strategy="parallel", sample_dt=0.5)
    assert par.strategy == "parallel"
    assert [j.start for j in par.jobs.values()] == [0.0, 0.0]
    assert par.makespan == pytest.approx(4.0)
    assert par.ok and par.robots_ok

    ser = schedule_cell(scene, _jobs(scene, jobs), strategy="serial", sample_dt=0.5)
    starts = [j.start for j in ser.jobs.values()]
    assert starts == [0.0, pytest.approx(4.0)]
    assert ser.makespan == pytest.approx(6.0)
    assert ser.ok


def test_schedule_stagger_uses_given_offsets(flag_urdf):
    scene, a, b = _two_robot_scene(flag_urdf, gap_mm=5000.0)
    jobs = [
        build_job(a, _home_traj(0.0, 4.0)),
        build_job(b, _home_traj(0.0, 2.0)),
    ]
    sched = schedule_cell(
        scene,
        _jobs(scene, jobs),
        strategy="stagger",
        sample_dt=0.5,
        offsets={a: 0.5, b: 1.5},
    )
    assert sched.strategy == "stagger"
    assert sched.jobs[a].start == pytest.approx(0.5)
    assert sched.jobs[b].start == pytest.approx(1.5)
    assert sched.makespan == pytest.approx(4.5)


def test_schedule_greedy_never_worse_than_serial(flag_urdf):
    """贪心的目标是最小化总节拍——任何场景下都不该比串行差。"""
    scene, a, b = _two_robot_scene(flag_urdf)
    jobs = [
        build_job(a, _home_traj(180.0, 3.0)),
        build_job(b, _home_traj(0.0, 2.0)),
    ]
    gre = schedule_cell(scene, _jobs(scene, jobs), strategy="greedy", sample_dt=0.25)
    ser = schedule_cell(scene, _jobs(scene, jobs), strategy="serial", sample_dt=0.25)
    assert gre.makespan <= ser.makespan + 1e-9
    # 基座重合且位形恒定：错峰救不了，贪心退化为串行
    assert gre.jobs[b].start == pytest.approx(ser.jobs[b].start)
    assert not gre.robots_ok and not ser.robots_ok


def test_schedule_greedy_staggers_to_clear_a_real_conflict(flag_urdf):
    """A 扫过 +X 区时 B 正在甩臂——并发必撞，贪心把 B 往后挪开。

    采样步长必须给细（0.05s）：B 的甩臂只有约 0.2s 处在危险角内，
    0.1s 采样会直接漏掉这次擦碰（这正是文档里"越小越不容易漏掉快速擦碰"
    的现场，同时也说明为什么默认 `sample_dt` 是 0.1 而不是 0.05）。
    """
    scene, a, b = _two_robot_scene(flag_urdf, gap_mm=1100.0)
    jobs = [
        build_job(a, _traj((0.0, 0.0), (4.0, 180.0), (8.0, 0.0))),
        build_job(b, _traj((0.0, 0.0), (0.5, 180.0), (1.0, 0.0))),
    ]
    kwargs = dict(sample_dt=0.05, search_step=0.25)

    par = schedule_cell(scene, _jobs(scene, jobs), strategy="parallel", **kwargs)
    assert not par.robots_ok, "并发必须报出这次擦碰"

    ser = schedule_cell(scene, _jobs(scene, jobs), strategy="serial", **kwargs)
    gre = schedule_cell(scene, _jobs(scene, jobs), strategy="greedy", **kwargs)

    assert gre.robots_ok, "贪心应找到一个无干涉的起始偏移"
    assert gre.jobs[b].start > 0.0, "B 必须被推迟，不能原地开工"
    assert gre.iterations > 0
    assert gre.makespan < ser.makespan
    assert gre.makespan <= par.makespan + 1e-9

    # 定稿结论必须能被独立复算出来（_PoseCache 复用位形时的缓存键若写错，
    # 这里会立刻对不上）
    rescan, _ = scan_timeline(scene, list(gre.jobs.values()), dt=0.05)
    assert rescan == []


def test_schedule_greedy_respects_max_wait(flag_urdf):
    """``max_wait`` 限制单台最大等待——超了就接受冲突而不是无限推迟。"""
    scene, a, b = _two_robot_scene(flag_urdf)
    jobs = [
        build_job(a, _home_traj(180.0, 3.0)),
        build_job(b, _home_traj(0.0, 2.0)),
    ]
    gre = schedule_cell(scene, _jobs(scene, jobs), strategy="greedy", sample_dt=0.5, max_wait=1.0)
    assert gre.jobs[b].start <= 1.0 + 1e-9
    assert not gre.robots_ok


def test_schedule_cell_rejects_bad_input(flag_urdf):
    scene, a, b = _two_robot_scene(flag_urdf, gap_mm=5000.0)
    jobs = [build_job(a, _home_traj(0.0, 1.0))]

    with pytest.raises(ValueError, match="未知策略"):
        schedule_cell(scene, jobs, strategy="magic")
    with pytest.raises(ValueError, match="offsets"):
        schedule_cell(scene, jobs, strategy="stagger")
    with pytest.raises(ValueError, match="机器人"):
        schedule_cell(scene, [build_job("不存在", _home_traj(0.0, 1.0))])

    assert set(STRATEGIES) == {"greedy", "parallel", "serial", "stagger"}


def test_schedule_cell_splits_robot_robot_from_env(flag_urdf):
    """环境干涉由单机轨迹决定，错峰消不掉——必须与机器人-机器人干涉分列。"""
    scene, a, b = _two_robot_scene(flag_urdf, gap_mm=5000.0)
    scene.add("workpiece_box", pose=Pose(), name="工件")
    assert CollisionChecker(scene).check_config(robot=a), "夹具前提失效"

    jobs = [
        build_job(a, _home_traj(0.0, 2.0)),
        build_job(b, _home_traj(0.0, 2.0)),
    ]
    for strategy in ("parallel", "serial", "greedy"):
        sched = schedule_cell(scene, _jobs(scene, jobs), strategy=strategy, sample_dt=0.5)
        assert sched.interferences == [], f"{strategy}: 机器人之间本无干涉"
        assert sched.env_interferences, f"{strategy}: 环境触碰不该被错峰消掉"
        assert sched.robots_ok and not sched.ok


# --------------------------------------------------------------------------- #
# 9. CellSchedule 报表
# --------------------------------------------------------------------------- #


def test_cell_schedule_summary_shape(flag_urdf):
    scene, a, b = _two_robot_scene(flag_urdf, gap_mm=5000.0)
    jobs = [
        build_job(a, _home_traj(0.0, 4.0), label="取件"),
        build_job(b, _home_traj(0.0, 2.0), label="出料"),
    ]
    sched = schedule_cell(scene, jobs, strategy="serial", sample_dt=0.5)
    data = sched.summary()

    assert set(data) >= {
        "strategy",
        "makespan",
        "sample_dt",
        "clearance",
        "iterations",
        "ok",
        "robots_ok",
        "robots",
        "interferences",
        "env_interferences",
    }
    assert data["strategy"] == "serial"
    assert data["makespan"] == pytest.approx(6.0)
    assert [r["robot"] for r in data["robots"]] == [a, b]
    assert [r["start"] for r in data["robots"]] == [0.0, 4.0]
    assert json.dumps(data)  # 摘要必须可 JSON 化（网页/报表共用）


def test_cell_schedule_format_is_readable(flag_urdf):
    scene, a, b = _two_robot_scene(flag_urdf, gap_mm=5000.0)
    jobs = [
        build_job(a, _home_traj(0.0, 4.0), label="取件"),
        build_job(b, _home_traj(0.0, 2.0), label="出料"),
    ]
    sched = schedule_cell(scene, jobs, strategy="parallel", sample_dt=0.5)
    text = sched.format()
    assert "多机器人节拍统筹" in text and "parallel" in text
    assert "总节拍" in text
    assert "取件" in text and "出料" in text
    assert "机器人之间无干涉" in text


def test_cell_schedule_format_empty():
    from service.multirobot import CellSchedule

    empty = CellSchedule()
    assert empty.makespan == 0.0
    assert empty.ok and empty.robots_ok
    assert "（无作业）" in empty.format()


def test_interference_str_and_kind(flag_urdf):
    scene, a, b = _two_robot_scene(flag_urdf)
    jobs = [
        build_job(a, _home_traj(180.0, 1.0)),
        build_job(b, _home_traj(0.0, 1.0)),
    ]
    robot_hits, _ = scan_timeline(scene, jobs, dt=0.5)
    hit = robot_hits[0]
    text = str(hit)
    assert "↔" in text and "mm" in text
    assert f"{hit.robot_a}:{hit.link_a}" in text
    from service.multirobot import _interference_dict

    assert _interference_dict(hit)["kind"] == "robot-robot"


# --------------------------------------------------------------------------- #
# 10. 端到端：双机器人示例脚本
# --------------------------------------------------------------------------- #


@pytest.fixture(scope="module")
def demo_run(tmp_path_factory):
    """真跑一遍 ``examples/multi_robot_cell_demo.py``（子进程 + 导出摘要）。"""
    out_dir = tmp_path_factory.mktemp("multirobot_demo")
    json_path = out_dir / "summary.json"
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([str(REPO_ROOT), str(REPO_ROOT), env.get("PYTHONPATH", "")])
    proc = subprocess.run(
        [sys.executable, str(DEMO), "--json", str(json_path)],
        cwd=str(REPO_ROOT),
        env=env,
        capture_output=True,
        text=True,
        timeout=900,
    )
    assert proc.returncode == 0, (
        f"示例脚本退出码 {proc.returncode}\n--- stdout ---\n"
        f"{proc.stdout[-2000:]}\n--- stderr ---\n{proc.stderr[-2000:]}"
    )
    return proc, json.loads(json_path.read_text(encoding="utf-8"))


def test_multi_robot_demo_sections(demo_run):
    out = demo_run[0].stdout
    for marker in (
        "多机器人协同演示（v1.0.1）",
        "机器人数：2",
        "[1] 单机规划与校验（各自独立视角）",
        "[2] 互避碰：两台机器人同时在交接点",
        "[3] 节拍统筹（同一条时间轴",
        "--- 策略 parallel ---",
        "--- 策略 serial ---",
        "--- 策略 greedy ---",
        "[4] 结论",
    ):
        assert marker in out, f"缺少分段输出: {marker}"
    assert "机器人A" in out and "机器人B" in out
    assert "干涉 0 处" in out or "无干涉" in out
    # 干涉按包络容差判定：必须把放行量说出来（见 ENVELOPE_TOLERANCE_MM）
    assert "包络接触" in out
    # 严格口径（净空 0）与现场口径（净空 60mm）必须都摆出来，并说清"擦肩
    # 而非穿透"——否则读者会把"0 处干涉"读成"两机永远不会靠得近"。
    assert "安全净空 60mm：同刻齐发 vs 贪心错峰" in out
    assert "擦肩而非穿透" in out


def test_multi_robot_demo_summary_invariants(demo_run):
    summary = demo_run[1]
    robots = summary["robots"]
    assert set(robots) == {"机器人A", "机器人B"}
    durations = {}
    for name, info in robots.items():
        assert info["cycle_time"] > 0.0
        assert info["waypoints"] > 0
        durations[name] = info["cycle_time"]

    # 交接点：两台机器人同时进去 → 这就是多机器人协同要解决的问题
    assert summary["handoff"]["collisions"]
    assert summary["handoff"]["closest_links"][0]
    assert isinstance(summary["handoff"]["separation_mm"], float)

    scheds = summary["schedules"]
    assert set(scheds) == {
        "parallel",
        "serial",
        "greedy",
        "parallel_clearance_60",
        "greedy_clearance_60",
    }
    # 并发 = 最长一条；串行 = 两条之和（这两条是恒等式，最能反映调度语义）
    assert scheds["parallel"]["makespan"] == pytest.approx(max(durations.values()), abs=1e-3)
    assert scheds["serial"]["makespan"] == pytest.approx(sum(durations.values()), abs=1e-3)
    # 贪心必须能排到机器人之间零干涉，且不差于串行
    assert scheds["greedy"]["robots_ok"] is True
    assert scheds["greedy"]["makespan"] <= scheds["serial"]["makespan"] + 1e-6
    # 留 60mm 净空重排：更保守，只会更慢或持平
    assert scheds["greedy_clearance_60"]["makespan"] >= scheds["greedy"]["makespan"] - 1e-6
    assert scheds["greedy_clearance_60"]["clearance"] == pytest.approx(60.0)

    # ---- 「擦肩」口径：这是本示例真正的教学点 ----
    # 净空 0 时两机判为不干涉（没穿透）；把判据换成 60mm 安全净空，同刻齐发立刻
    # 暴露一片近失。若这条断言失败，说明轨迹被改得"太平顺"——演示就失去了说服力。
    assert not scheds["parallel"]["interferences"], "净空 0 口径：并发应无穿透"
    assert scheds["parallel_clearance_60"][
        "interferences"
    ], "净空 60mm 口径下同刻齐发应有近失，否则'必须错峰'的结论不成立"
    # 近失全部由贪心错峰清掉，且总节拍仍优于串行
    assert scheds["greedy_clearance_60"]["robots_ok"] is True
    assert scheds["greedy_clearance_60"]["makespan"] <= scheds["serial"]["makespan"] + 1e-6
    # 且贪心确实动了起始时刻（不是"躺着也能过"）
    starts = {r["robot"]: r["start"] for r in scheds["greedy_clearance_60"]["robots"]}
    assert max(starts.values()) > 0.0

    near = summary["near_miss"]
    assert near["clearance"] == pytest.approx(60.0)
    assert near["n_parallel"] > 0 and near["n_greedy"] == 0
    # 最近处是"擦肩"（正间距）而不是穿透（负间距）——这正是要与测试夹具区分开的地方
    assert near["min_separation_mm"] is not None
    assert 0.0 < near["min_separation_mm"] < 60.0
    assert near["closest_at_s"] is not None
    assert len(near["closest_links"]) == 2 and all(near["closest_links"])
    assert all(":" in s for s in near["closest_links"]), "最近连杆对应带机器人名"
