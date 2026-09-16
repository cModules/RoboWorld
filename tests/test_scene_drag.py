"""场景拖拽摆放（v1.1.7）的测试。

右键 gizmo（xy/z/rz）此前已能"选中后微调"，但"把工件/料箱摆到工作台上"
仍要在属性面板里手填 Z。本轮补齐摆放语义：**左键按在工件上拖 = 移动，
松手自动落到正下方的支承面**（几何算出来的，不是猜的）。覆盖：

* 落点计算（纯几何，无 Qt）：`world_aabb` / `support_height_at` /
  `ground_offset` / `snap`
* `place_on_support`：落台面 / 落地面 / 吸附 / 锁定与挂载对象明确拒绝
* `apply_world_pose`：父对象（含旋转父系）换算正确——从 GLWidget 收敛进
  world 层的"世界系 → 局部系"唯一实现
* GLWidget 左键拖拽全流程（offscreen 真事件）：按下 → 跟手拖动（Z 不变）→
  松手落面 → `objectPlaced` 状态栏小结
* 既有行为不回归：空白处左键拖仍是旋转视角、机器人不可摆放、
  右键 gizmo 变换照旧
* en 界面：状态栏小结零中文残留

运行::

    pytest tests/test_scene_drag.py -q
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))
sys.path.insert(0, str(REPO_ROOT))

from orolp import i18n  # noqa: E402
from orolp.pose import Pose  # noqa: E402
from orolp.world import placement  # noqa: E402
from orolp.world.components import Part  # noqa: E402
from orolp.world.geometry import Box  # noqa: E402
from orolp.world.scene import (  # noqa: E402
    GEOMETRY_COLLISION,
    ROBOT_NAME,
    Scene,
    SceneObject,
)

URDF = REPO_ROOT / "models" / "ur" / "ur5e" / "ur5e.urdf"

from PySide6.QtCore import QEvent, QPointF, Qt  # noqa: E402
from PySide6.QtGui import QMouseEvent  # noqa: E402


def _box_object(name, size, xyz, *, locked=False, parent=None, parts=None,
                visible=True):
    """造一个"裸几何"场景对象（不走组件注册表，测试自持）。"""
    obj = SceneObject(
        name=name,
        parts=parts if parts is not None else [Part(Box(size), np.eye(4), None)],
        pose=Pose.from_xyz_rpy(*xyz, 0.0, 0.0, 0.0),
        locked=locked,
        parent=parent,
        visible=visible,
    )
    return obj


@pytest.fixture
def scene():
    """工作台（顶面 z=800）+ 工件（原点在几何中心，悬在台面上方）。"""
    s = Scene()
    s.objects["工作台"] = _box_object("工作台", (400, 300, 800), (0, 0, 400))
    s.objects["工件"] = _box_object("工件", (50, 50, 40), (100, 0, 820))
    return s


# --------------------------------------------------------------------------- #
# 落点计算（纯几何）
# --------------------------------------------------------------------------- #


def test_world_aabb_matches_geometry(scene):
    box = placement.world_aabb(scene, "工作台")
    assert box is not None
    assert np.allclose(box.lo, [-200, -150, 0])
    assert np.allclose(box.hi, [200, 150, 800])


def test_support_under_table_is_table_top(scene):
    z, name = placement.support_height_at(scene, 0.0, 0.0)
    assert (z, name) == (800.0, "工作台")


def test_support_off_table_is_ground(scene):
    z, name = placement.support_height_at(scene, 1000.0, -1000.0)
    assert (z, name) == (0.0, placement.GROUND_NAME)


def test_support_can_ignore_the_object_itself(scene):
    """不排除自己 → 工件永远"落在自己头上"，拖拽就没法用了。"""
    z, name = placement.support_height_at(scene, 100.0, 0.0, ignore=("工件",))
    assert (z, name) == (800.0, "工作台")
    z_self, name_self = placement.support_height_at(scene, 100.0, 0.0)
    assert name_self == "工件" and z_self == pytest.approx(840.0)


def test_support_skips_robots_and_invisible_objects(scene):
    scene.set_robot(URDF, joints_deg=[0.0] * 6)
    scene.objects["隐形箱"] = _box_object("隐形箱", (400, 300, 1600), (0, 0, 800),
                                          visible=False)
    # 机器人与不可见对象都不算支承——工件不该"落在机器人头上"
    z, name = placement.support_height_at(scene, 0.0, 0.0, ignore=("工件",))
    assert (z, name) == (800.0, "工作台")


def test_support_picks_the_highest_surface(scene):
    scene.objects["料箱"] = _box_object("料箱", (100, 100, 120), (0, 0, 860))
    z, name = placement.support_height_at(scene, 0.0, 0.0, ignore=("工件",))
    assert (z, name) == (920.0, "料箱")


def test_ground_offset_is_origin_to_bottom(scene):
    # 工件原点在几何中心（820），底面在 800 → 原点比底面高 20 → offset = -20
    assert placement.ground_offset(scene, "工件") == pytest.approx(-20.0)


def test_snap_aligns_to_step():
    assert placement.snap(33.0, 25.0) == pytest.approx(25.0)
    assert placement.snap(11.0, 25.0) == pytest.approx(0.0)
    assert placement.snap(-37.0, 25.0) == pytest.approx(-25.0)


def test_snap_disabled_or_invalid_step_returns_value():
    assert placement.snap(33.7, 0) == pytest.approx(33.7)
    assert placement.snap(33.7, -5) == pytest.approx(33.7)


# --------------------------------------------------------------------------- #
# place_on_support
# --------------------------------------------------------------------------- #


def test_place_on_table_lands_bottom_on_top(scene):
    result = placement.place_on_support(scene, "工件", -50.0, 30.0)
    assert result.moved and result.support == "工作台"
    assert (result.x, result.y) == (-50.0, 30.0)
    # 底面贴台面：z = 800 - (-20)
    assert result.z == pytest.approx(820.0)
    xyz = scene.objects["工件"].pose.to_xyz_rpy()[:3]
    assert np.allclose(xyz, [-50.0, 30.0, 820.0])


def test_place_off_table_lands_on_ground(scene):
    result = placement.place_on_support(scene, "工件", 1000.0, 0.0)
    assert result.support == placement.GROUND_NAME
    assert result.z == pytest.approx(20.0)   # 底面贴地：0 - (-20)


def test_place_snaps_xy_when_step_given(scene):
    result = placement.place_on_support(scene, "工件", 33.0, -37.0, snap_step=25.0)
    assert (result.x, result.y) == (25.0, -25.0)


def test_place_keeps_orientation(scene):
    tf = scene.world_transform("工件")
    rz = math.degrees(math.atan2(tf[1, 0], tf[0, 0]))
    scene.objects["工件"].pose = Pose.from_xyz_rpy(100, 0, 820, 0, 0, 45.0)
    placement.place_on_support(scene, "工件", -50, 30)
    tf2 = scene.world_transform("工件")
    rz2 = math.degrees(math.atan2(tf2[1, 0], tf2[0, 0]))
    assert rz2 == pytest.approx(45.0, abs=1e-9)


def test_place_locked_object_is_rejected_with_reason(scene):
    scene.objects["夹具"] = _box_object("夹具", (50, 50, 50), (0, 0, 825), locked=True)
    before = scene.objects["夹具"].pose.to_xyz_rpy()
    result = placement.place_on_support(scene, "夹具", 500.0, 500.0)
    assert result.moved is False
    assert "锁定" in result.reason
    assert scene.objects["夹具"].pose.to_xyz_rpy() == pytest.approx(before)


def test_place_mounted_object_is_rejected(scene):
    scene.objects["法兰件"] = _box_object("法兰件", (30, 30, 30), (0, 0, 850),
                                          parent="工作台")
    result = placement.place_on_support(scene, "法兰件", 0.0, 0.0)
    assert result.moved is False
    assert "挂载" in result.reason


def test_place_robot_lands_on_support_top(scene):
    """机器人可拖动，落点遵循与其它对象同一规则：正下方支承面顶面。

    把机器人放到底座/台面上是常规布局；掉到台面之外才落地。
    """
    scene.set_robot(URDF, joints_deg=[0.0] * 6)
    robot = scene.robot_names[0]
    offset = placement.ground_offset(scene, robot)

    on_table = placement.place_on_support(scene, robot, 0.0, 0.0)
    assert on_table.moved is True
    assert on_table.support == "工作台"
    assert on_table.z == pytest.approx(800.0 - offset)

    off_table = placement.place_on_support(scene, robot, 2000.0, 2000.0)
    assert off_table.support == placement.GROUND_NAME
    assert off_table.z == pytest.approx(-offset)


def test_place_unknown_object_is_rejected(scene):
    result = placement.place_on_support(scene, "不存在的对象", 0.0, 0.0)
    assert result.moved is False and result.reason


def test_grabbable_rules(scene):
    assert placement.grabbable(scene, "工件") is True
    assert placement.grabbable(scene, "机器人") is False          # 场景里没有
    assert placement.grabbable(scene, None) is False
    scene.objects["锁件"] = _box_object("锁件", (10, 10, 10), (0, 0, 900), locked=True)
    assert placement.grabbable(scene, "锁件") is False


def test_apply_world_pose_with_rotated_parent():
    """父对象带 yaw 时，世界系目标必须换算进局部系——这是拖动挂载件不出错的前提。"""
    s = Scene()
    s.objects["底座"] = _box_object("底座", (200, 200, 200), (1000, 0, 100))
    s.objects["底座"].pose = Pose.from_xyz_rpy(1000, 0, 100, 0, 0, 90.0)
    s.objects["子件"] = _box_object("子件", (40, 40, 40), (0, 0, 240),
                                    parent="底座")
    placement.apply_world_pose(s, "子件", 1000.0, 500.0, 260.0, 30.0)
    tf = s.world_transform("子件")
    assert np.allclose(tf[:3, 3], [1000.0, 500.0, 260.0], atol=1e-9)
    # 世界系 yaw == 30°（局部 30° + 父 90° 对世界系朝向的贡献已折算）
    world_rz = math.degrees(math.atan2(tf[1, 0], tf[0, 0]))
    assert world_rz == pytest.approx(30.0, abs=1e-9)


def test_summary_is_human_readable(scene):
    result = placement.place_on_support(scene, "工件", -50, 30)
    text = placement.placement_summary(result)
    assert "工件" in text and "落在 工作台 上" in text and "820.0" in text
    ground = placement.place_on_support(scene, "工件", 5000, 0)
    assert "落在 地面 上" in placement.placement_summary(ground)


# --------------------------------------------------------------------------- #
# GLWidget 左键拖拽（offscreen 真事件）
# --------------------------------------------------------------------------- #


@pytest.fixture(scope="module")
def qapp():
    from PySide6 import QtWidgets

    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture()
def window(qapp):
    import app as rw

    win = rw.MainWindow()
    yield win
    win.close()
    win.deleteLater()


@pytest.fixture()
def gl_scene(window):
    """把合成场景注入视口；返回 (gl_widget, scene)。"""
    s = Scene()
    s.objects["工作台"] = _box_object("工作台", (400, 300, 800), (0, 0, 400))
    s.objects["工件"] = _box_object("工件", (50, 50, 40), (100, 0, 820))
    window.gl_widget.scene = s
    window.gl_widget.resize(640, 480)
    return window.gl_widget, s


def _mouse(kind, x, y):
    # 事件口径与 Qt 一致：PressEvent/ReleaseEvent 的 button = LeftButton，
    # MoveEvent 的 button = NoButton（buttons 才是按住状态）
    if kind == QEvent.MouseButtonPress:
        button, buttons = Qt.LeftButton, Qt.LeftButton
    elif kind == QEvent.MouseMove:
        button, buttons = Qt.NoButton, Qt.LeftButton
    else:
        button, buttons = Qt.LeftButton, Qt.NoButton
    return QMouseEvent(kind, QPointF(x, y), button, buttons, Qt.NoModifier)


def _screen_of(gl, scene, name, dz_mm=0.0):
    tf = scene.world_transform(name)
    p = tf[:3, 3] * gl.MM_TO_M
    p[2] += dz_mm * gl.MM_TO_M
    return gl._project(p)


def test_left_press_on_workpiece_starts_grab(gl_scene):
    gl, scene = gl_scene
    x, y = _screen_of(gl, scene, "工件")
    gl.mousePressEvent(_mouse(QEvent.MouseButtonPress, x, y))
    assert gl._grab_active is True
    assert gl.selected_object == "工件"
    assert gl.dragging is False          # 不能同时转视角
    gl.mouseReleaseEvent(_mouse(QEvent.MouseButtonRelease, x, y))


def test_left_press_on_empty_space_still_orbits(gl_scene):
    gl, _scene = gl_scene
    gl.mousePressEvent(_mouse(QEvent.MouseButtonPress, 5, 5))
    assert gl._grab_active is False
    assert gl.dragging is True           # 空白处保持"拖 = 旋转视角"
    gl.mouseReleaseEvent(_mouse(QEvent.MouseButtonRelease, 5, 5))


def test_left_press_on_robot_grabs(gl_scene):
    """机器人进场景后同样可点选拖动——搬基座是常规布局操作。

    这里顺带守住根因：机器人必须用**碰撞包络**参与拾取，否则整个机器人只剩
    一个 TCP 标记球，点机身射线打空（看得见却选不中）。
    """
    gl, scene = gl_scene
    scene.set_robot(URDF, joints_deg=[0.0] * 6)
    robot = scene.robot_names[0]
    scene.objects["工件"].visible = False   # 别让工件挡住射向机器人的视线
    # 工作台从机器人脚下挪开：否则射线先穿台面，命中的是工作台而不是机器人
    scene.objects["工作台"].pose = Pose.from_xyz_rpy(3000, 0, 400, 0, 0, 0)

    parts = [
        p
        for group in scene.robot_world_link_parts(
            robot, source=GEOMETRY_COLLISION
        ).values()
        for p in group
    ]
    assert len(parts) > 1, "机器人应有多个碰撞包络参与拾取，否则点机身选不中"

    p = np.asarray(parts[0].transform[:3, 3], dtype=float) * gl.MM_TO_M
    x, y = gl._project(p)
    gl.mousePressEvent(_mouse(QEvent.MouseButtonPress, x, y))
    assert gl._grab_active is True
    assert gl.dragging is False          # 不再是"选中 + 旋转视角"
    assert gl.selected_object == robot
    gl.mouseReleaseEvent(_mouse(QEvent.MouseButtonRelease, x, y))
    gl.mouseReleaseEvent(_mouse(QEvent.MouseButtonRelease, x, y))


def test_drag_moves_xy_and_keeps_height(gl_scene):
    gl, scene = gl_scene
    x0, y0 = _screen_of(gl, scene, "工件")
    gl.mousePressEvent(_mouse(QEvent.MouseButtonPress, x0, y0))
    target = scene.world_transform("工件")[:3, 3].copy()
    target[0], target[1] = -50.0, 30.0
    x1, y1 = gl._project(target * gl.MM_TO_M)
    gl.mouseMoveEvent(_mouse(QEvent.MouseMove, x1, y1))
    xyz = scene.objects["工件"].pose.to_xyz_rpy()[:3]
    assert xyz[2] == pytest.approx(820.0)        # 拖动中 Z 不变
    # XY 有像素取整 + 透视视差（事件坐标是整数像素），毫米级误差属正常
    assert xyz[0] == pytest.approx(-50.0, abs=6.0)
    assert xyz[1] == pytest.approx(30.0, abs=6.0)
    gl.mouseReleaseEvent(_mouse(QEvent.MouseButtonRelease, x1, y1))


def test_release_drops_on_support_and_emits_summary(gl_scene):
    gl, scene = gl_scene
    seen = []
    gl.objectPlaced.connect(seen.append)
    x0, y0 = _screen_of(gl, scene, "工件")
    gl.mousePressEvent(_mouse(QEvent.MouseButtonPress, x0, y0))
    target = scene.world_transform("工件")[:3, 3].copy()
    target[0], target[1] = -50.0, 30.0
    x1, y1 = gl._project(target * gl.MM_TO_M)
    gl.mouseMoveEvent(_mouse(QEvent.MouseMove, x1, y1))
    gl.mouseReleaseEvent(_mouse(QEvent.MouseButtonRelease, x1, y1))

    xyz = scene.objects["工件"].pose.to_xyz_rpy()[:3]
    assert xyz[2] == pytest.approx(820.0)        # 松手落到台面：底面贴 800
    assert len(seen) == 1
    assert "工件" in seen[0] and "落在 工作台 上" in seen[0]
    assert gl._grab_active is False


def test_release_on_empty_ground_drops_to_floor(gl_scene):
    gl, scene = gl_scene
    x0, y0 = _screen_of(gl, scene, "工件")
    gl.mousePressEvent(_mouse(QEvent.MouseButtonPress, x0, y0))
    # 台面半宽 200mm，挪到 (500, -100) —— 台面之外、但仍在相机前方可投影处
    target = scene.world_transform("工件")[:3, 3].copy()
    target[0], target[1] = 500.0, -100.0
    x1, y1 = gl._project(target * gl.MM_TO_M)
    gl.mouseMoveEvent(_mouse(QEvent.MouseMove, x1, y1))
    gl.mouseReleaseEvent(_mouse(QEvent.MouseButtonRelease, x1, y1))
    xyz = scene.objects["工件"].pose.to_xyz_rpy()[:3]
    assert xyz[2] == pytest.approx(20.0)         # 地面：0 - (-20)
    assert scene.objects["工件"].pose.to_xyz_rpy()[0] == pytest.approx(500.0, abs=6.0)


def test_snap_step_applies_on_release(gl_scene):
    gl, scene = gl_scene
    gl.snap_step_mm = 25.0
    x0, y0 = _screen_of(gl, scene, "工件")
    gl.mousePressEvent(_mouse(QEvent.MouseButtonPress, x0, y0))
    target = scene.world_transform("工件")[:3, 3].copy()
    target[0], target[1] = -50.0, 30.0
    x1, y1 = gl._project(target * gl.MM_TO_M)
    gl.mouseMoveEvent(_mouse(QEvent.MouseMove, x1, y1))
    gl.mouseReleaseEvent(_mouse(QEvent.MouseButtonRelease, x1, y1))
    xyz = scene.objects["工件"].pose.to_xyz_rpy()[:3]
    assert xyz[0] == pytest.approx(-50.0)        # -50 本就是 25 的倍数
    assert xyz[1] == pytest.approx(25.0)         # 30 → 25
    assert xyz[2] == pytest.approx(820.0)


def test_status_bar_receives_placement_summary(qapp, gl_scene):
    """MainWindow 必须把 objectPlaced 接到状态栏——摆放结论要让人当场看见。"""
    gl, scene = gl_scene
    win = gl.window()
    x0, y0 = _screen_of(gl, scene, "工件")
    gl.mousePressEvent(_mouse(QEvent.MouseButtonPress, x0, y0))
    gl.mouseReleaseEvent(_mouse(QEvent.MouseButtonRelease, x0, y0))
    text = win.statusBar().currentMessage()
    assert "工件" in text and "落在 工作台 上" in text


def test_right_gizmo_still_works_after_grab_feature(gl_scene):
    """回归：右键 gizmo 的变换流程不受左键拖拽影响。

    右键第一段移动锁定手柄模式，第二段执行变换——松手后模式复位。
    """
    from PySide6.QtCore import QEvent, QPointF, Qt
    from PySide6.QtGui import QMouseEvent

    gl, scene = gl_scene
    x0, y0 = _screen_of(gl, scene, "工件")
    press = QMouseEvent(QEvent.MouseButtonPress, QPointF(x0, y0),
                        Qt.RightButton, Qt.RightButton, Qt.NoModifier)
    gl.mousePressEvent(press)
    assert gl.selected_object == "工件"

    before = scene.world_transform("工件")[:3, 3].copy()
    before_rz = math.atan2(before[1], before[0])
    # 第一段移动：锁定模式（对象中心命中的是 Z 轴竖条或 XY 方块）
    gl.mouseMoveEvent(QMouseEvent(QEvent.MouseMove, QPointF(x0 + 2, y0 - 2),
                                  Qt.RightButton, Qt.RightButton, Qt.NoModifier))
    mode = gl.drag_mode
    assert mode in ("xy", "z", "rz")
    # 第二段移动：执行变换
    gl.mouseMoveEvent(QMouseEvent(QEvent.MouseMove, QPointF(x0 + 60, y0 - 30),
                                  Qt.RightButton, Qt.RightButton, Qt.NoModifier))
    gl.mouseReleaseEvent(QMouseEvent(
        QEvent.MouseButtonRelease, QPointF(x0 + 60, y0 - 30),
        Qt.RightButton, Qt.NoButton, Qt.NoModifier))

    after = scene.world_transform("工件")[:3, 3]
    after_rz = math.atan2(after[1], after[0])
    changed = (not np.allclose(before, after)) or not math.isclose(
        before_rz, after_rz, abs_tol=1e-12)
    assert changed, f"模式 {mode} 的 gizmo 变换没有改变对象位姿"
    assert gl.drag_mode is None


def test_en_summary_has_no_chinese(gl_scene, monkeypatch, tmp_path):
    """en 界面：状态栏小结零中文残留。"""
    monkeypatch.setenv("OROLP_CONFIG_DIR", str(tmp_path / "cfg"))
    i18n.set_language(i18n.LANG_EN, save=False)
    try:
        gl, scene = gl_scene
        result = placement.place_on_support(scene, "工件", -50, 30)
        text = placement.placement_summary(result)
        assert "落在" not in text
        assert "placed on" in text or "工作台" in text  # 对象名是数据，保留原样
        assert not any("\u4e00" <= c <= "\u9fff" for c in text.replace("工件", "").replace("工作台", ""))
    finally:
        i18n.set_language(i18n.LANG_ZH, save=False)


# --------------------------------------------------------------------------- #
# 法兰自动装夹（拖到末端 → 自动吸合；拖已装夹件 → 先解除）
# --------------------------------------------------------------------------- #


def _add_box(scene, name, size, xyz):
    """往场景里塞一个裸几何对象，测试自持、不依赖组件注册表。"""
    scene.objects[name] = _box_object(name, size, xyz)
    return scene.objects[name]


def _flange_world(scene, robot):
    """法兰原点在世界系下的位置（毫米）。"""
    return (scene.world_transform(robot) @ scene.flange_transform(robot))[:3, 3]


def test_flange_snap_target_is_none_without_robot(scene):
    assert placement.flange_snap_target(scene, "工件", 60.0) is None


def test_flange_snap_target_hits_only_within_radius(scene):
    scene.set_robot(URDF, joints_deg=[0.0] * 6)
    robot = scene.robot_names[0]
    flange = _flange_world(scene, robot)
    g = _add_box(scene, "夹爪", (40, 40, 60),
                 (float(flange[0]), float(flange[1]), float(flange[2])))

    assert placement.flange_snap_target(scene, "夹爪", 60.0) == robot

    # 挪出判定半径之外即不再命中——否则会因为"某个机器人离得近"而误装
    g.pose = Pose.from_xyz_rpy(float(flange[0]) + 500.0, float(flange[1]),
                               float(flange[2]), 0, 0, 0)
    assert placement.flange_snap_target(scene, "夹爪", 60.0) is None


def test_grab_mounted_tool_auto_detaches(gl_scene):
    """拖已装夹到法兰的工具 → 起拖即解除挂载（不再是"工具偏移"）。"""
    gl, scene = gl_scene
    scene.set_robot(URDF, joints_deg=[0.0] * 6)
    robot = scene.robot_names[0]
    # 放到远离机身包络处再挂到机器人：夹爪若正贴法兰，射线会先打到机器人的
    # 碰撞包络而选中机器人（这是"机器人可拾取"带来的必然副作用）
    g = _add_box(scene, "夹爪", (40, 40, 60), (600.0, 600.0, 300.0))
    scene.attach("夹爪", robot, keep_world=True)
    assert scene.objects["夹爪"].parent == robot

    before = scene.world_transform("夹爪")[:3, 3].copy()
    x0, y0 = _screen_of(gl, scene, "夹爪")
    gl.mousePressEvent(_mouse(QEvent.MouseButtonPress, x0, y0))

    assert gl._grab_active is True, "已装夹件必须可以起拖"
    assert scene.objects["夹爪"].parent is None, "起拖未自动解除挂载"
    # 解除必须保持世界位姿——否则视觉上会瞬移
    assert np.allclose(scene.world_transform("夹爪")[:3, 3], before, atol=1e-6)
    gl.mouseReleaseEvent(_mouse(QEvent.MouseButtonRelease, x0, y0))


def test_release_near_flange_mounts_instead_of_dropping(gl_scene):
    """松手落在法兰附近＝装夹（保 mounts 优先于落到支承面）。"""
    gl, scene = gl_scene
    scene.set_robot(URDF, joints_deg=[0.0] * 6)
    robot = scene.robot_names[0]
    flange = _flange_world(scene, robot)
    _add_box(scene, "夹爪", (40, 40, 60),
             (float(flange[0]), float(flange[1]), float(flange[2])))

    x0, y0 = _screen_of(gl, scene, "夹爪")
    # 夹爪正贴法兰时射线会先命中机器人的碰撞包络，这里直接起拖，专注于验证
    # "松手是否装夹"这一件事
    gl.selected_object = "夹爪"
    gl._grab_start("夹爪", x0, y0)
    gl.mouseReleaseEvent(_mouse(QEvent.MouseButtonRelease, x0, y0))

    assert scene.objects["夹爪"].parent == robot, "拖到法兰松手未自动装夹"
    # 装夹＝局部位姿归零，工具原点与法兰重合（坐标系对齐）
    assert np.allclose(scene.world_transform("夹爪")[:3, 3], flange, atol=1e-6)


def test_device_tree_reparent_mounts_on_flange(qapp):
    """设备树拖拽：挂到机器人＝装夹到法兰并对齐坐标系。"""
    from orolp.world.panel import ScenePanel

    s = Scene()
    s.set_robot(URDF, joints_deg=[0.0] * 6)
    robot = s.robot_names[0]
    g = s.add("tool_gripper", pose=Pose.from_xyz_rpy(999, 999, 999, 0, 0, 0))

    panel = ScenePanel(s)
    try:
        panel._on_item_reparented(g.name, robot)
    finally:
        panel.deleteLater()

    assert s.objects[g.name].parent == robot
    assert np.allclose(s.world_transform(g.name)[:3, 3],
                       _flange_world(s, robot), atol=1e-6)


def test_device_tree_reparent_keeps_world_for_plain_parent(qapp):
    """挂到普通对象＝保持世界位姿建立随动（不能瞬移）。"""
    from orolp.world.panel import ScenePanel

    s = Scene()
    table = s.add("worktable", pose=Pose.from_xyz_rpy(0, 0, 400, 0, 0, 0))
    wp = s.add("workpiece_box", pose=Pose.from_xyz_rpy(100, 0, 820, 0, 0, 0))

    panel = ScenePanel(s)
    try:
        before = s.world_transform(wp.name)[:3, 3].copy()
        panel._on_item_reparented(wp.name, table.name)
    finally:
        panel.deleteLater()

    assert s.objects[wp.name].parent == table.name
    assert np.allclose(s.world_transform(wp.name)[:3, 3], before, atol=1e-6)


def test_device_tree_reparent_robot_sits_on_base_top(qapp):
    """树里把机器人拖到底座上：应坐到上表面中心（接线必须生效）。"""
    from orolp.world.panel import ScenePanel
    from orolp.world.placement import ground_offset, world_aabb

    s = Scene()
    s.set_robot(URDF, joints_deg=[0.0] * 6)
    robot = s.robot_names[0]
    base = s.add(
        "worktable", name="底座",
        params={"length": 400.0, "width": 400.0, "height": 200.0},
        pose=Pose.from_xyz_rpy(1000.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    )

    panel = ScenePanel(s)
    try:
        panel._on_item_reparented(robot, base.name)
    finally:
        panel.deleteLater()

    assert s.objects[robot].parent == base.name
    box = world_aabb(s, base.name)
    p = s.world_transform(robot)[:3, 3]
    assert p[0] == pytest.approx(0.5 * (box.lo[0] + box.hi[0]), abs=1e-6)
    assert p[2] == pytest.approx(
        float(box.hi[2]) - ground_offset(s, robot), abs=1e-6
    )


def test_device_tree_reparent_rejects_robot_on_robot_flange(qapp):
    """整机挂到另一台机器人的法兰上没有意义，必须拦下。"""
    from orolp.world.panel import ScenePanel

    s = Scene()
    a = s.add_robot(URDF, [0.0] * 6, name="机器人A")
    b = s.add_robot(URDF, [0.0] * 6, name="机器人B")

    panel = ScenePanel(s)
    try:
        panel._on_item_reparented(a, b)
    finally:
        panel.deleteLater()

    assert s.objects[a].parent is None


def test_load_robot_keeps_scene_joints_in_sync(window):
    """载入机器人后，场景关节必须等于渲染器当前位形。

    渲染器加载完会摆到 HOME 待机位（弯的），而 Scene 里新建的机器人是零位。
    不同步则"画面手臂是弯的、场景法兰却竖直朝天"，挂到法兰上的夹爪会跑到
    手臂外面——表现为装夹失败，但位置面板显示 0/0/0（挂载其实成功了）。
    """
    window.load_robot(str(URDF))
    r = window.gl_widget.renderer
    scene = window.scene_panel.scene
    assert scene.has_robot
    robot = scene.robot_names[0]
    assert scene.robots[robot].joints_deg == pytest.approx(
        [math.degrees(r.joint_values[n]) for n in r.actuated_names()], abs=1e-6
    )
    # HOME 不是零位——若同步丢了，这条会先挂，避免"零位法兰"再次得逞
    assert any(abs(v) > 1e-6 for v in scene.robots[robot].joints_deg)


def test_two_robots_have_independent_renderers_and_joints(window):
    """两台机器人各自独立：渲染器不共用、关节互不影响。

    回归：``renderer_of`` 曾回退到"当前渲染器"，于是两台共用一份 joint_values，
    拖动关节时一起动；同步又只写主机器人，挂在另一台上的工具会离开其法兰。
    """
    window.load_robot(str(URDF))
    window.load_robot(str(URDF))
    gl, scene = window.gl_widget, window.scene_panel.scene
    a, b = scene.robot_names
    assert gl.renderers[a] is not gl.renderers[b], "两台机器人共用了同一个渲染器"

    gl.set_current_robot(a)
    before_b = list(scene.robots[b].joints_deg)
    joint_name = gl.renderer.actuated_names()[0]
    gl.renderer.joint_values[joint_name] = 0.9
    window._sync_scene_joints()

    assert scene.robots[a].joints_deg[0] == pytest.approx(math.degrees(0.9))
    assert scene.robots[b].joints_deg == pytest.approx(before_b), (
        "拖 A 的关节把 B 也改了"
    )


def test_lazy_renderer_matches_its_own_urdf_and_scene_joints(window):
    """未登记渲染器的机器人必须按**自己**的 URDF 建，并把关节对齐到场景值。

    否则"画出来的手臂"与"场景法兰"错位，挂在它上面的工具会离开显示的法兰。
    """
    window.load_robot(str(URDF))
    gl, scene = window.gl_widget, window.scene_panel.scene
    a = scene.robot_names[0]
    # 手工补一台没有渲染器的机器人（模拟从场景文件载入的情况）
    want = [10.0, -20.0, 30.0, 0.0, 45.0, 0.0]
    b = scene.add_robot(str(URDF), want)
    assert b not in gl.renderers, "前置条件：B 尚未登记渲染器"

    rb = gl.renderer_of(b)
    assert rb is not gl.renderers[a], "B 借用了 A 的渲染器"
    got = [math.degrees(float(rb.joint_values[n])) for n in rb.actuated_names()]
    assert got == pytest.approx(want, abs=1e-6)


def test_draw_scene_skips_all_robots(window, monkeypatch):
    """``_draw_scene`` 必须跳过**所有**机器人，而不只是主机器人。

    否则第二台会被当普通场景对象再画一遍——``object_parts`` 给机器人的是
    visual（只剩一个 TCP 标记球），画面上就是一个孤立的小球。
    """
    window.load_robot(str(URDF))
    window.load_robot(str(URDF))
    gl = window.gl_widget
    drawn = []
    monkeypatch.setattr(gl, "_draw_part", lambda part, *a, **k: drawn.append(part))

    gl._draw_scene()

    assert drawn == [], f"机器人被当成普通场景对象重复绘制: {drawn}"


def test_adding_second_robot_appends_instead_of_replacing(window):
    """再载入一台机器人必须**追加**，不能顶掉已有的那台。

    回归：加入口一律走 ``Scene.set_robot``，名字固定为 ``ROBOT_NAME``，于是
    "再增加一台"实际是替换（且视口只有一台的渲染器）。
    """
    window.load_robot(str(URDF))
    scene = window.scene_panel.scene
    assert scene.robot_names == [ROBOT_NAME]

    window.load_robot(str(URDF))

    assert len(scene.robot_names) == 2, (
        f"第二台机器人把已有的顶掉了: {scene.robot_names}"
    )
    first, second = scene.robot_names
    assert first == ROBOT_NAME and second != first
    # 每台都要有各自的渲染器，且渲染基座位姿互不相同（不能叠在一起）
    assert set(window.gl_widget.renderers) >= set(scene.robot_names)
    xs = [
        float(window.gl_widget._robot_base_pose_m(n)[0, 3])
        for n in scene.robot_names
    ]
    assert abs(xs[0] - xs[1]) > 1e-6, "两台机器人渲染基座位姿相同"


def test_robot_render_base_follows_mount(window):
    """机器人挂到带偏移的底座后，渲染用的基座位姿必须跟着走。

    回归：_draw_robot 曾用 SceneObject.pose（局部），挂载后世界位姿 =
    底座 × 局部，渲染却只吃局部 → 画的机器人不跟底座、与场景分家。
    """
    window.load_robot(str(URDF))
    gl, scene = window.gl_widget, window.scene_panel.scene
    robot = scene.robot_names[0]

    before = gl._robot_base_pose_m()[:3, 3].copy()
    base = scene.add(
        "worktable", name="底座",
        pose=Pose.from_xyz_rpy(1000.0, 500.0, 0.0, 0.0, 0.0, 0.0),
    )
    scene.attach(robot, base.name, keep_world=True)

    # keep_world 保持世界位姿 → 渲染基座位姿不该动（不能瞬移）
    assert np.allclose(gl._robot_base_pose_m()[:3, 3], before, atol=1e-6)

    # 底座挪走后，渲染基座位姿必须跟着挪（mm → m）
    base.pose = Pose.from_xyz_rpy(2000.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert np.allclose(
        gl._robot_base_pose_m()[:3, 3] - before, [1.0, -0.5, 0.0], atol=1e-6
    )
