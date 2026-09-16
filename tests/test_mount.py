"""挂载（父子随动）机制的回归测试。

守的是「零件抓取工作站」的地基：夹爪装到法兰上必须真的跟着机器人走，
工件被夹住时必须停在原地、松开时必须留在原地。

若这些行为退化，抓取演示会变成"夹爪留在原地不动"或"工件瞬移"，
而且从代码上看不出错——必须靠数值断言守住。

运行::

    pytest tests/test_mount.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

from orolp.pose import Pose  # noqa: E402
from orolp.world.scene import ROBOT_NAME, Scene  # noqa: E402

URDF = REPO_ROOT / "models/kuka/kr16_2/kr16_2.urdf"

Q_A = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
Q_B = [30.0, -20.0, 15.0, 0.0, 45.0, 10.0]


@pytest.fixture
def scene() -> Scene:
    """已载入 KR16 的空白场景。"""
    if not URDF.exists():
        pytest.skip(f"缺少测试用 URDF: {URDF}")
    s = Scene()
    s.set_robot(URDF)
    return s


def xyz(s: Scene, name: str) -> np.ndarray:
    return s.world_transform(name)[:3, 3]


# --------------------------------------------------------------------------- #
# 法兰跟随
# --------------------------------------------------------------------------- #


def test_flange_transform_changes_with_joints(scene):
    """法兰位姿必须随关节角变化——否则"跟随"无从谈起。"""
    scene.set_robot_joints(Q_A)
    t_a = scene.flange_transform()[:3, 3].copy()
    scene.set_robot_joints(Q_B)
    t_b = scene.flange_transform()[:3, 3].copy()

    assert np.linalg.norm(t_b - t_a) > 1.0, "法兰不随关节变化"


def test_flange_is_identity_without_robot():
    """没载入机器人时返回单位阵，而不是抛异常。"""
    assert np.allclose(Scene().flange_transform(), np.eye(4))


def test_child_of_robot_follows_flange(scene):
    """挂在机器人下的对象必须跟着法兰走。"""
    g = scene.add("tool_gripper", parent=ROBOT_NAME)

    scene.set_robot_joints(Q_A)
    at_a = xyz(scene, g.name).copy()
    scene.set_robot_joints(Q_B)
    at_b = xyz(scene, g.name).copy()

    assert np.linalg.norm(at_b - at_a) > 1.0, "挂载件未跟随法兰"
    # 局部位姿为零时，世界位置应当与法兰完全重合
    assert np.allclose(at_b, scene.flange_transform()[:3, 3], atol=1e-6)


def test_non_robot_children_unaffected(scene):
    """挂在普通对象下的对象不受法兰影响——不能误伤既有父子逻辑。"""
    table = scene.add("worktable")
    box = scene.add("workpiece_box", parent=table.name)

    scene.set_robot_joints(Q_A)
    at_a = xyz(scene, box.name).copy()
    scene.set_robot_joints(Q_B)
    at_b = xyz(scene, box.name).copy()

    assert np.allclose(at_a, at_b), "非机器人子对象不应受法兰影响"


# --------------------------------------------------------------------------- #
# attach / detach 语义
# --------------------------------------------------------------------------- #


def test_attach_keep_world_preserves_position(scene):
    """keep_world=True：挂载瞬间世界位置不能漂移（抓取语义）。"""
    wp = scene.add("workpiece_box", pose=Pose.from_xyz_rpy(500, 200, 900, 0, 0, 0))
    before = xyz(scene, wp.name).copy()

    scene.attach(wp.name, ROBOT_NAME, keep_world=True)

    assert np.allclose(xyz(scene, wp.name), before, atol=1e-6), "抓取时工件瞬移了"


def test_attach_snap_to_flange(scene):
    """keep_world=False：吸附到法兰原点（安装工具语义）。"""
    g = scene.add("tool_gripper", pose=Pose.from_xyz_rpy(999, 999, 999, 0, 0, 0))
    scene.attach(g.name, ROBOT_NAME, keep_world=False)
    assert np.allclose(xyz(scene, g.name),
                       scene.flange_transform()[:3, 3], atol=1e-6)


def test_detach_keeps_world_position_and_stops_following(scene):
    """放置：释放瞬间停在原地，之后不再跟随法兰。"""
    wp = scene.add("workpiece_box", pose=Pose.from_xyz_rpy(500, 200, 900, 0, 0, 0))
    scene.attach(wp.name, ROBOT_NAME, keep_world=True)

    scene.set_robot_joints(Q_B)
    at_release = xyz(scene, wp.name).copy()
    scene.detach(wp.name)

    assert scene.objects[wp.name].parent is None
    assert np.allclose(xyz(scene, wp.name), at_release, atol=1e-6), "释放时工件瞬移了"

    scene.set_robot_joints(Q_A)
    assert np.allclose(xyz(scene, wp.name), at_release, atol=1e-6), "释放后仍在跟随"


def test_detach_on_root_object_is_noop(scene):
    wp = scene.add("workpiece_box")
    scene.detach(wp.name)  # 不应抛异常
    assert scene.objects[wp.name].parent is None


# --------------------------------------------------------------------------- #
# 防御
# --------------------------------------------------------------------------- #


def test_attach_rejects_cycle(scene):
    """成环会让 world_transform 无限递归——必须拦下。"""
    a = scene.add("workpiece_box", name="A")
    b = scene.add("workpiece_box", name="B")
    scene.attach("B", "A")
    with pytest.raises(ValueError):
        scene.attach("A", "B")


def test_attach_rejects_self(scene):
    a = scene.add("workpiece_box", name="A")
    with pytest.raises(ValueError):
        scene.attach("A", "A")


def test_attach_rejects_unknown_parent(scene):
    a = scene.add("workpiece_box", name="A")
    with pytest.raises(ValueError):
        scene.attach("A", "不存在的对象")


def test_set_robot_joints_tolerates_length_mismatch(scene):
    """误把 fixed 关节一起传进来不能让 FK 崩掉——应补齐/截断。

    KR16 的 URDF 是 7 个 joints / 6 个可动。调用方若用
    ``renderer.joints``（含 fixed）取值就会传 7 个，FK 会直接抛
    "期望 6 个关节值，收到 7 个"。这类错误表现为"夹爪不动"，极难定位。
    """
    scene.set_robot_joints([10.0] * 7)
    assert len(scene._robot_joints_deg) == 6
    assert np.allclose(scene.flange_transform()[:3, 3],
                       scene.flange_transform()[:3, 3])  # 不抛异常即可


def test_missing_parent_degrades_gracefully(scene):
    """手工编辑过的场景文件可能指向不存在的父对象：退化为不跟随，不崩。"""
    a = scene.add("workpiece_box", name="A")
    a.parent = "幽灵父对象"
    assert np.allclose(scene.world_transform("A"), a.pose.matrix)


# --------------------------------------------------------------------------- #
# 持久化
# --------------------------------------------------------------------------- #


def test_mounted_object_excluded_from_robot_collision(scene):
    """挂在法兰上的夹爪与机器人本体必然接触——不能报成干涉。

    否则"装了工具"会被恒定报成撞机，干涉检查直接失去意义。
    """
    g = scene.add("tool_gripper", parent=ROBOT_NAME)
    hits = scene.check_collisions()
    assert not any({h.obj_a, h.obj_b} == {ROBOT_NAME, g.name} for h in hits), (
        f"挂载件被误报为干涉: {hits}"
    )


def test_unmounted_object_still_checked_against_robot(scene):
    """未挂载对象仍必须参与机器人干涉检查——不能借排除规则一起放过。

    注意几何现实：这些 URDF 的 42 个 ``<visual>`` 里绝大多数是 ``<mesh>``，
    而场景解析器只认 box/cylinder/sphere，因此机器人侧目前实际上是拿
    **TCP 标记球**在参与检查。测试把工件精确摆到 TCP 处，这样结论不依赖
    将来补齐连杆包围体之后仍然成立。
    """
    tcp = scene.flange_transform()[:3, 3]
    box = scene.add(
        "workpiece_box", name="挡在TCP",
        params={"length": 60.0, "width": 60.0, "height": 60.0},
        pose=Pose.from_xyz_rpy(float(tcp[0]), float(tcp[1]), float(tcp[2]), 0, 0, 0),
    )
    hits = scene.check_collisions()
    assert any({h.obj_a, h.obj_b} == {ROBOT_NAME, box.name} for h in hits), (
        f"未挂载对象被漏检: {hits}"
    )


def test_robot_mounted_on_base_is_exempt_from_collision(scene):
    """把机器人装到底座上：基座与底座必然接触，不能报成干涉。

    否则"给机器人配个基座"这一常规布局动作会被恒定报成撞机。
    """
    base = scene.add(
        "worktable", name="底座",
        params={"length": 400.0, "width": 400.0, "height": 200.0},
    )
    scene.attach(ROBOT_NAME, base.name, keep_world=True)
    assert scene.objects[ROBOT_NAME].parent == base.name

    hits = scene.check_collisions()
    assert not any({h.obj_a, h.obj_b} == {ROBOT_NAME, base.name} for h in hits), (
        f"装在底座上的机器人被误报为干涉: {hits}"
    )


def test_robot_follows_its_base(scene):
    """底座挪走，装在上面的机器人（基座）要跟着走。"""
    base = scene.add("worktable", name="底座")
    scene.attach(ROBOT_NAME, base.name, keep_world=True)
    before = scene.world_transform(ROBOT_NAME)[:3, 3].copy()

    base.pose = Pose.from_xyz_rpy(500.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    after = scene.world_transform(ROBOT_NAME)[:3, 3]
    assert np.allclose(after - before, [500.0, 0.0, 0.0], atol=1e-6)


def test_robot_mounted_on_base_sits_on_top_center(scene):
    """机器人挂到底座上，应坐在底座**上表面正中央**，安装面贴合顶面。

    回归：world_aabb 曾按 visual 取数，机器人只剩一个 TCP 标记球，
    ground_offset 算出约 -1000mm，会把机器人塞到地底下。
    """
    from orolp.world.placement import (
        ground_offset,
        mount_on_top_of,
        world_aabb,
    )

    base = scene.add(
        "worktable", name="底座",
        params={"length": 400.0, "width": 400.0, "height": 200.0},
        pose=Pose.from_xyz_rpy(1000.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    )
    scene.attach(ROBOT_NAME, base.name, keep_world=True)
    assert mount_on_top_of(scene, ROBOT_NAME, base.name) is True

    box = world_aabb(scene, base.name)
    p = scene.world_transform(ROBOT_NAME)[:3, 3]
    assert p[0] == pytest.approx(0.5 * (box.lo[0] + box.hi[0]), abs=1e-6)
    assert p[1] == pytest.approx(0.5 * (box.lo[1] + box.hi[1]), abs=1e-6)
    assert p[2] == pytest.approx(
        float(box.hi[2]) - ground_offset(scene, ROBOT_NAME), abs=1e-6
    )
    # 机器人基座原点就在安装面，落点应正好等于顶面高度
    assert ground_offset(scene, ROBOT_NAME) == pytest.approx(0.0, abs=1e-6)


def test_serialization_preserves_robot_mounted_on_base(scene):
    """机器人挂在底座上必须能存进场景文件——否则重开就散架。"""
    base = scene.add("worktable", name="底座")
    scene.attach(ROBOT_NAME, base.name, keep_world=True)
    expected = scene.world_transform(ROBOT_NAME)[:3, 3].copy()

    restored = Scene.from_dict(scene.to_dict())

    assert restored.objects[ROBOT_NAME].parent == base.name
    assert np.allclose(
        restored.world_transform(ROBOT_NAME)[:3, 3], expected, atol=1e-6
    )


def test_serialization_preserves_mount(scene):
    """挂载关系必须能存进场景文件——否则预置工作站存不住夹爪。"""
    g = scene.add("tool_gripper", parent=ROBOT_NAME)
    scene.set_robot_joints(Q_B)
    expected = xyz(scene, g.name).copy()

    restored = Scene.from_dict(scene.to_dict())

    assert restored.objects[g.name].parent == ROBOT_NAME
    assert np.allclose(xyz(restored, g.name), expected, atol=1e-6)
