"""v0.2.1 碰撞检测模块：包围体广相、连杆级定位、自碰撞、奇异软预警。

测试夹具里的 ``three_link.urdf`` 是一个手工设计的最小自碰撞场景：

* ``base``——沿 +X 的长条盒（x∈[0, 0.5] m，z∈[0, 0.1] m）
* ``arm``——经转动关节 j1（绕 Z，原点在原点）连到 base，静止时同样
  沿 +X 铺开（与 base 直连 → 豁免对）
* ``tool``——经转动关节 j2（绕 Z，原点在 (0.6, 0, 0)）连到 arm，盒体
  沿**局部 -X** 方向延伸 0.5 m

于是 q2=0° 时 tool 翻回来压在 base 上（非直连 → 必须报自碰撞），
q2=180° 时 tool 甩到 +X 远端（干净位形）。环境干涉与轨迹定位用
真实 UR5e + 抓取工作站验证（把 TCP 规划进输送带体内）。
"""

import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from domain.robot import Robot  # noqa: E402
from domain.collision import CollisionChecker, parts_aabb, singularity_proximity  # noqa: E402
from service.planner import ENVELOPE_TOLERANCE_MM  # noqa: E402
from domain.pose import Pose  # noqa: E402
from domain.world.components import Part  # noqa: E402
from domain.world.geometry import Box  # noqa: E402
from domain.world.scene import ROBOT_NAME, Scene  # noqa: E402

UR5E = REPO_ROOT / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"

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


def make_three_link_scene(three_link_urdf) -> Scene:
    """全新的三连杆场景（每个测试独立，避免共享可变状态）。"""
    scene = Scene()
    scene.set_robot(three_link_urdf)
    return scene


@pytest.fixture(scope="module")
def workstation_scene():
    """真实 UR5e + 抓取工作站（只读，用于环境干涉与轨迹定位）。"""
    from domain.world import build_grasping_workstation

    return build_grasping_workstation().scene


# --------------------------------------------------------------------------- #
# 包围体
# --------------------------------------------------------------------------- #


def test_parts_aabb_union():
    parts = [
        Part(Box((100, 100, 100)), np.eye(4), (0.6, 0.6, 0.65)),
        Part(
            Box((100, 100, 100)),
            np.array([[1, 0, 0, 300], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], float),
            (0.6, 0.6, 0.65),
        ),
    ]
    box = parts_aabb(parts)
    assert box is not None
    assert box.lo[0] == pytest.approx(-50)
    assert box.hi[0] == pytest.approx(350)


def test_parts_aabb_empty():
    assert parts_aabb([]) is None


# --------------------------------------------------------------------------- #
# 连杆-环境：命中带连杆归属
# --------------------------------------------------------------------------- #


def test_link_env_collision_reports_link(three_link_urdf):
    scene = make_three_link_scene(three_link_urdf)
    # 工件盒放在原点附近：与 base 长条（x∈[0,0.5]m）重叠
    scene.add("workpiece_box", name="障碍物", pose=Pose.from_xyz_rpy(150, 0, 25, 0, 0, 0))
    checker = CollisionChecker(scene, margin=-0.5)
    hits = checker.check_config([0.0, 180.0])  # tool 甩远端，只剩 base 撞工件
    env_hits = [h for h in hits if h.obj_b == "障碍物"]
    assert env_hits, f"应检出连杆-环境碰撞: {hits}"
    hit = env_hits[0]
    assert hit.obj_a == ROBOT_NAME
    assert hit.link_a == "base"  # 定位：撞在 base 段
    assert hit.link_b is None


def test_clean_config_no_collision(three_link_urdf):
    scene = make_three_link_scene(three_link_urdf)
    checker = CollisionChecker(scene, margin=-0.5)
    assert checker.check_config([0.0, 180.0]) == []


def test_env_cache_invalidation(three_link_urdf):
    scene = make_three_link_scene(three_link_urdf)
    checker = CollisionChecker(scene, margin=-0.5)
    scene.add("workpiece_box", name="障碍物", pose=Pose.from_xyz_rpy(150, 0, 25, 0, 0, 0))
    assert checker.check_config([0.0, 180.0])  # 有障碍 → 碰撞
    scene.remove("障碍物")
    checker.invalidate_env()
    assert checker.check_config([0.0, 180.0]) == []  # 移除后不再报


# --------------------------------------------------------------------------- #
# 自碰撞：连杆-连杆
# --------------------------------------------------------------------------- #


def test_self_collision_detected(three_link_urdf):
    scene = make_three_link_scene(three_link_urdf)
    checker = CollisionChecker(scene, margin=-0.5)
    hits = checker.check_config([0.0, 0.0], include_self=True)
    self_hits = [h for h in hits if h.obj_a == ROBOT_NAME and h.obj_b == ROBOT_NAME]
    assert self_hits, "tool 翻回 base 上方，应检出非直连自碰撞"
    pair = {self_hits[0].link_a, self_hits[0].link_b}
    assert pair == {"base", "tool"}


def test_adjacent_links_exempt(three_link_urdf):
    scene = make_three_link_scene(three_link_urdf)
    checker = CollisionChecker(scene, margin=-0.5)
    hits = checker.check_config([0.0, 0.0], include_self=True)
    for h in hits:
        pair = {h.link_a, h.link_b}
        assert pair not in ({"base", "arm"}, {"arm", "tool"}), "关节直连的连杆对不应报告自碰撞"


def test_include_self_off(three_link_urdf):
    scene = make_three_link_scene(three_link_urdf)
    checker = CollisionChecker(scene, margin=-0.5)
    hits = checker.check_config([0.0, 0.0], include_self=False)
    assert all(not (h.obj_a == ROBOT_NAME and h.obj_b == ROBOT_NAME) for h in hits)


def test_scene_check_collisions_flags(three_link_urdf):
    scene = make_three_link_scene(three_link_urdf)
    # 默认不自检：q=(0,0) 的自碰撞不报
    scene.set_robot_joints([0.0, 0.0])
    assert scene.check_collisions(include_robot=True) == []
    # 显式开自检：报出并带连杆名
    hits = scene.check_collisions(include_robot=True, include_self=True)
    assert hits and hits[0].link_a == "base" and hits[0].link_b == "tool"


# --------------------------------------------------------------------------- #
# 真实场景：UR5e + 抓取工作站，轨迹定位
# --------------------------------------------------------------------------- #


def test_trajectory_collision_localized_to_motion(workstation_scene):
    robot = Robot(str(UR5E), "UR5e")
    home = [0, -90, 90, -90, 90, 0]
    # 第二段 moveL 把 TCP 规划进输送带体内（皮带面 z=268mm，点取 z=120）
    program = robot.moveJ(home, speed=80).then(
        robot.moveL(Pose.from_xyz_rpy(200, -400, 120, 180, 0, 0), speed=40)
    )
    report = program.validate(scene=workstation_scene)
    col_issues = [i for i in report.issues if i.code == "collision"]
    assert col_issues, "TCP 插进输送带，校验必须报干涉"
    assert not report.ok
    # 定位"哪一段"：干涉在第二条指令（motion_index=1）
    assert all(i.motion_index == 1 for i in col_issues)
    # 文案带连杆名（撞在哪一段）
    assert f"{ROBOT_NAME}:" in col_issues[0].message
    # simulate(collision_check=True) 同样检出，并带时刻与指令定位
    traj = program.simulate(collision_check=True, scene=workstation_scene)
    assert traj.collisions
    hit = traj.collisions[0]
    assert hit.time is not None and hit.time > 0
    assert hit.motion == 1
    assert hit.link_a  # 连杆级定位


def test_envelope_tolerance_does_not_blind_deep_collisions():
    """包络容差只放行「包络接触」，真撞（深穿透）必须照样报错。

    这条守的是 ``ENVELOPE_TOLERANCE_MM`` 的边界：容差是为了滤掉厂商保守
    包络盒的模型误差，不是把干涉检查关掉。

    场景是"机器人正前方立一块 400mm 高的实心块，把 TCP 规划进它体内"——
    用三级 margin 把结论钉死：严格口径报 → 容差口径仍报 → 荒谬口径
    （-1000mm）才不报，说明真实穿透量落在 20~1000mm 之间，远深于容差。
    """
    from domain.world.templates import build_grasping_workstation

    scene = build_grasping_workstation().scene
    scene.add(
        "workpiece_box",
        name="实心块",
        pose=Pose.from_xyz_rpy(0.0, -250.0, 0.0, 0.0, 0.0, 0.0),
        params={"length": 300.0, "width": 300.0, "height": 400.0},
    )
    robot = Robot(str(UR5E), "UR5e")
    program = robot.moveJ([0, -90, 90, -90, 90, 0], speed=80).then(
        robot.moveL(Pose.from_xyz_rpy(0, -250, 200, 180, 0, 0), speed=40)
    )

    def collisions(margin: float) -> int:
        report = program.validate(scene=scene, margin=margin)
        return len([i for i in report.issues if i.code == "collision"])

    assert collisions(0.0) > 0, "夹具前提失效：TCP 应该插进实心块"
    assert collisions(-ENVELOPE_TOLERANCE_MM) > 0, "容差不能把深穿透也放行"
    assert collisions(-1000.0) == 0
    assert 0.0 < ENVELOPE_TOLERANCE_MM < 1000.0


def test_workstation_home_no_collision(workstation_scene):
    """工作站布局经可达性校准：Home 位形不得误报连杆-环境干涉。

    必须**显式摆到 Home**：fixture 是 module 级共享的，前面跑轨迹检查的用例
    会把机器人留在轨迹末点（曾因此让本用例实际检查的是一个折臂深蹲位形）。
    """
    home = [0.0, -90.0, 90.0, -90.0, 90.0, 0.0]
    workstation_scene.set_robot_joints(home)
    checker = CollisionChecker(workstation_scene, margin=-0.5)
    hits = checker.check_config(home)
    assert hits == [], f"Home 位形误报: {hits}"


# --------------------------------------------------------------------------- #
# 奇异软预警
# --------------------------------------------------------------------------- #


def test_singularity_proximity_none_without_kin():
    assert singularity_proximity(None, [0, 0, 0, 0, 0, 0]) is None


def test_singularity_proximity_finite(three_link_urdf):
    kin = Robot(three_link_urdf, "three_link").kinematics
    cond = singularity_proximity(kin, [0.0, 0.0])
    assert cond is not None and cond > 0 and np.isfinite(cond)


def test_check_config_warnings_type(three_link_urdf):
    scene = make_three_link_scene(three_link_urdf)
    checker = CollisionChecker(scene, margin=-0.5)
    warnings = checker.check_config_warnings([0.0, 180.0])
    assert isinstance(warnings, list)
    for w in warnings:
        assert "奇异" in w
