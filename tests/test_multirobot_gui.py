"""多机器人 GUI 链路：关节归属隔离、回放按机寻址、出码按机独立。

这里守住的是**视口 ↔ 属性面板 ↔ 场景**这条链路上"值写到了哪台机器人"的问题：

* 拖 A 的关节滑块，写进的必须是 A 的渲染器（B 不能跟着动）；
* 「关节」滑块必须按**选中那台**的关节重建，而不是沿用最后加载那台；
* 场景关节同步（法兰/干涉检查）必须只作用于目标机器人。

回归背景：``jointChanged`` 原先只带 ``(关节名, 值)``，MainWindow 只能写入
"当前渲染器"，而滑块是按面板里一个陈旧的 ``_renderer`` 建的——两者错位时
拖 A 却驱动了 B，表现为"两台机器人一起运动"。

测试用的一自由度 URDF 见 :mod:`tests.test_multirobot` 的模块 docstring。

运行::

    pytest tests/test_multirobot_gui.py -q
"""

import math
import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))
sys.path.insert(0, str(REPO_ROOT))

from orolp.pickplace import PointType  # noqa: E402
from orolp.pose import Pose  # noqa: E402
from orolp.kinematics import KinematicsError  # noqa: E402
from orolp.world.scene import Scene  # noqa: E402

# 出码/校验要真机器人（含运动学 + 可推断品牌），故用仓库里的真实型号
UR5E = REPO_ROOT / "models" / "ur" / "ur5e" / "ur5e.urdf"

FLAG_URDF = """<?xml version="1.0"?>
<robot name="flag">
  <link name="base">
    <visual>
      <origin xyz="0.2 0 0" rpy="0 0 0"/>
      <geometry><box size="0.4 0.1 0.1"/></geometry>
    </visual>
    <collision>
      <origin xyz="0.2 0 0" rpy="0 0 0"/>
      <geometry><box size="0.4 0.1 0.1"/></geometry>
    </collision>
  </link>
  <link name="arm">
    <visual>
      <origin xyz="0.3 0 0" rpy="0 0 0"/>
      <geometry><box size="0.6 0.1 0.1"/></geometry>
    </visual>
    <collision>
      <origin xyz="0.3 0 0" rpy="0 0 0"/>
      <geometry><box size="0.6 0.1 0.1"/></geometry>
    </collision>
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
def qapp():
    from PySide6 import QtWidgets

    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture(scope="module")
def flag_urdf(tmp_path_factory) -> str:
    """一自由度 URDF（可视 + 碰撞都是实体盒，保证渲染器与场景都能起来）。"""
    path = tmp_path_factory.mktemp("urdf") / "flag.urdf"
    path.write_text(FLAG_URDF, encoding="utf-8")
    return str(path)


@pytest.fixture()
def window(qapp):
    import app as rw

    win = rw.MainWindow()
    yield win
    win.close()
    win.deleteLater()


@pytest.fixture()
def two_robots(window, flag_urdf):
    """窗口里放两台相距 2 m 的同型机器人；返回 (win, scene, A, B)。"""
    scene = window.scene_panel.scene
    scene.clear()
    a = scene.add_robot(flag_urdf, [0.0], name="机器人A")
    b = scene.add_robot(
        flag_urdf,
        [0.0],
        name="机器人B",
        pose=Pose.from_xyz_rpy(2000.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    )
    window.gl_widget.scene = scene
    window.property_panel.set_scene(scene)
    return window, scene, a, b


# --------------------------------------------------------------------------- #
# 1. 关节滑块必须属于「选中那台」
# --------------------------------------------------------------------------- #


def test_sliders_are_built_from_selected_robot(two_robots):
    """选中 A → 滑块按 A 的渲染器重建，且回调记住了所属机器人名。

    回归：面板以往沿用 ``_renderer``（最后加载那台）。此断言直接检查
    :attr:`_active_robot` 是否与选中对象一致——它是关节写入的目标依据。
    """
    win, scene, a, b = two_robots
    panel = win.property_panel

    panel.set_object(a)
    assert panel._active_robot == a
    renderer_a = win.gl_widget.renderer_of(a)
    assert panel.sliders and set(panel.sliders) == set(renderer_a.actuated_names())

    panel.set_object(b)
    assert panel._active_robot == b, "切到 B 后滑块仍指向 A，会串台"


def test_joint_changed_signal_carries_robot_name(two_robots):
    """滑块变化发出的信号必须带机器人名——MainWindow 靠它定位写入目标。"""
    win, scene, a, b = two_robots
    panel = win.property_panel
    panel.set_object(b)

    seen = []
    panel.jointChanged.connect(lambda robot, joint, val: seen.append((robot, joint, val)))

    slider = list(panel.sliders.values())[0]
    slider.blockSignals(False)
    slider.setValue(3000)  # 30.00°

    assert seen, "滑块没发出信号"
    assert seen[-1][0] == b, f"信号里的机器人名错了：{seen[-1][0]}"

    panel.set_object(a)
    slider = list(panel.sliders.values())[0]
    slider.setValue(-1500)
    assert seen[-1][0] == a, "切回 A 后信号仍报 B"


# --------------------------------------------------------------------------- #
# 2. 拖动 A 的关节，B 不能动
# --------------------------------------------------------------------------- #


def test_dragging_joints_only_moves_that_robot(two_robots):
    """核心回归：拖 B 的关节 → 只有 B 的渲染器与场景关节变化。

    这就是"两台机器人一起运动"那个现象的直接断言。修复前，写入目标是
    ``gl_widget.renderer``（当前那台），此处会连带改动另一台。
    """
    win, scene, a, b = two_robots
    panel = win.property_panel

    # 先把两台都逼迫成不同初值，便于分辨谁动了
    scene.set_robot_joints([10.0], name=a)
    scene.set_robot_joints([20.0], name=b)
    win.gl_widget.load_robot(
        scene.robots[a].urdf, name=a
    )
    win.gl_widget.load_robot(
        scene.robots[b].urdf, name=b
    )
    win.gl_widget.renderer_of(a).joint_values["j1"] = np.radians(10.0)
    win.gl_widget.renderer_of(b).joint_values["j1"] = np.radians(20.0)

    panel.set_object(b)
    renderer_of_a = win.gl_widget.renderer_of(a)
    renderer_of_b = win.gl_widget.renderer_of(b)
    before_a = dict(renderer_of_a.joint_values)
    before_a_scene = list(scene.robots[a].joints_deg)

    # 事件走真实链路：滑块 → jointChanged → MainWindow.on_joint_changed
    slider = list(panel.sliders.values())[0]
    slider.setValue(4500)  # 45.00°

    assert abs(renderer_of_b.joint_values["j1"] - np.radians(45.0)) < 1e-6, (
        "B 的关节没被驱动到 45°"
    )
    assert renderer_of_a.joint_values == before_a, "拖 B 却改动了 A 的渲染关节"
    assert scene.robots[a].joints_deg == before_a_scene, "场景里 A 的关节被连带改动"
    # B 自己的场景关节要同步到位（法兰/干涉检查靠它）
    assert abs(scene.robots[b].joints_deg[0] - 45.0) < 1e-6


def test_sync_scene_joints_targets_named_robot(two_robots):
    """``_sync_scene_joints(name)`` 必须只动指名那台，不让另一台跟着同步。"""
    win, scene, a, b = two_robots
    scene.set_robot_joints([5.0], name=a)
    scene.set_robot_joints([15.0], name=b)

    win.gl_widget.renderer_of(a).joint_values["j1"] = np.radians(5.0)
    win.gl_widget.renderer_of(b).joint_values["j1"] = np.radians(15.0)

    win.gl_widget.renderer_of(a).joint_values["j1"] = np.radians(77.0)
    win._sync_scene_joints(a)

    assert abs(scene.robots[a].joints_deg[0] - 77.0) < 1e-6
    assert abs(scene.robots[b].joints_deg[0] - 15.0) < 1e-6, "同步 A 时不许改写到 B"


def test_degraded_robot_never_borrows_another_renderer(two_robots):
    """模型取不到时**绝不**借用别人的渲染器。

    回归根因：``renderer_of`` 以前在 URDF 失效时退回 ``self.renderer``。两台机器人
    于是共用一份几何与 ``joint_values``——表现为"拖关节两台一起动" +
    "这台的位置画的却是另一台的机身"。画不出来是诚实的，画错了是灾难。
    """
    win, scene, a, b = two_robots
    # 工程换 machine / URDF 被挪走是常见现场：路径还在，文件没了
    # 现实路径：用户正看着 A（它是当前机器人），B 的文件却在换机后失效了
    win.gl_widget.set_current_robot(a)
    scene.robots[b].urdf = "/不存在/机器人.urdf"
    win.gl_widget.renderers.pop(b, None)
    win.gl_widget._empty_renderers.pop(b, None)

    ra = win.gl_widget.renderer_of(a)
    rb = win.gl_widget.renderer_of(b)

    assert rb is not ra, "B 借用了 A 的渲染器——联动与错模型的根因在此"
    assert rb.model is None, "退化时不应有可画模型，宁可不画也不画错"

    ra.joint_values["j1"] = np.radians(33.0)
    assert rb.joint_values.get("j1") in (None, 0.0), (
        "共用渲染器：改 A 的关节会写到 B 身上"
    )


def test_renderers_are_independent_per_robot(two_robots):
    """两台机器人各有自己的渲染器实例与关节字典（同型号也不能共用）。"""
    win, scene, a, b = two_robots
    ra, rb = win.gl_widget.renderer_of(a), win.gl_widget.renderer_of(b)

    assert ra is not rb
    assert ra.joint_values is not rb.joint_values
    assert list(ra.actuated_names()) == list(rb.actuated_names()) == ["j1"]

    ra.joint_values["j1"] = np.radians(60.0)
    assert abs(rb.joint_values["j1"] - np.radians(0.0)) < 1e-6


def test_renderer_aligns_to_its_own_scene_joints(two_robots):
    """懒加载时把关节对齐到**该机器人**的场景关节——法兰才有指导意义。

    挂到法兰的夹爪之所以跑到手臂外，就是因为新渲染器停在 HOME 待机位，
    而场景里的机器人是另一个位形。
    """
    win, scene, a, b = two_robots
    scene.set_robot_joints([30.0], name=a)
    scene.set_robot_joints([80.0], name=b)
    win.gl_widget.renderers.clear()  # 强制走懒加载分支

    ra, rb = win.gl_widget.renderer_of(a), win.gl_widget.renderer_of(b)

    assert abs(ra.joint_values["j1"] - np.radians(30.0)) < 1e-6
    assert abs(rb.joint_values["j1"] - np.radians(80.0)) < 1e-6, (
        "第二台渲染器没对齐自己的场景关节"
    )


def test_mounted_tool_sits_on_its_own_flange(two_robots):
    """挂在 A 上的夹爪贴 A 的法兰，挂在 B 上的贴 B 的——不能张冠李戴。

    用户报的"夹爪挂载上去的机器人模型不再挂在显示法兰上面"，本质就是场景法兰
    与渲染位形对不上；这里从几何上直接核对两者的**世界坐标**是否重合。
    """
    win, scene, a, b = two_robots
    scene.set_robot_joints([30.0], name=a)
    scene.set_robot_joints([60.0], name=b)
    scene.add("tool_gripper", name="A的夹爪")
    scene.add("tool_gripper", name="B的夹爪")
    scene.attach("A的夹爪", a, keep_world=False)
    scene.attach("B的夹爪", b, keep_world=False)

    for robot, tool in ((a, "A的夹爪"), (b, "B的夹爪")):
        flange_world = scene.world_transform(robot) @ scene.flange_transform(robot)
        tool_world = scene.world_transform(tool)
        assert np.allclose(tool_world[:3, 3], flange_world[:3, 3], atol=1e-6), (
            f"{tool} 没贴在 {robot} 的法兰上"
        )

    # 两台的法兰位置本来就该不同（B 偏移 2 m 且关节不同），否则本测试没意义
    assert not np.allclose(
        scene.world_transform("A的夹爪")[:3, 3],
        scene.world_transform("B的夹爪")[:3, 3],
    )


def test_flange_follows_its_own_robot(two_robots):
    """场景法兰按各自关节算——改 B 不能挪动 A 的法兰（工具挂在谁身上不串）。"""
    win, scene, a, b = two_robots
    scene.set_robot_joints([0.0], name=a)
    scene.set_robot_joints([0.0], name=b)

    before_a = scene.flange_transform(a).copy()
    scene.set_robot_joints([90.0], name=b)

    assert np.allclose(scene.flange_transform(a), before_a)
    rotated_b = scene.flange_transform(b)
    assert abs(rotated_b[1, 0] - 1.0) < 1e-6, "B 绕 Z 转 90° 后 X 轴应指向 Y"


# --------------------------------------------------------------------------- #
# 3. 多机出码 / 回放（真实型号 UR5E，才能真的后处理）
# --------------------------------------------------------------------------- #

@pytest.fixture()
def two_ur5e(window):
    """面板里登记两台 UR5E：名字不同、型号相同（多机最典型的现场）。"""
    scene = window.scene_panel.scene
    scene.clear()
    a = scene.add_robot(str(UR5E), [0.0] * 6, name="机器人A")
    b = scene.add_robot(
        str(UR5E),
        [0.0] * 6,
        name="机器人B",
        pose=Pose.from_xyz_rpy(2000.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    )
    window.gl_widget.scene = scene

    from orolp import Robot as OrolpRobot

    panel = window.program_panel
    panel.set_scene(scene)
    panel.set_robot(OrolpRobot(str(UR5E), "机器人A"), name=a)
    panel.set_robot(OrolpRobot(str(UR5E), "机器人B"), name=b)
    return window, scene, a, b


def _points(panel, robot, first_deg: float):
    """给 ``robot`` 造两个示教点：首关节分别停在 0° 与 ``first_deg``。"""
    names = panel._state(robot).joint_names
    p0 = {n: 0.0 for n in names}
    p1 = dict(p0)
    p1[names[0]] = math.radians(first_deg)
    return [p0, p1]


def test_each_robot_gets_its_own_code(two_ur5e):
    """多机出码：两台各出一份，程序名与关节值都不串。"""
    win, scene, a, b = two_ur5e
    panel = win.program_panel
    panel.set_taught_points(_points(panel, a, 30.0), robot=a)
    panel.set_taught_points(_points(panel, b, 75.0), robot=b)

    codes = panel.generate_all_code()

    assert set(codes) == {a, b}, "每台机器人都该有一份"
    assert codes[a] != codes[b], "两台的代码不该是同一份"
    # 程序名用场景机器人名：同型号时不至于互相覆盖
    assert a in codes[a] and b in codes[b]
    # 关节值以**弧度**落进代码，必须按真实数值断言——拿角度字符串比对会
    # 永远查不到，测试就成了摆设
    a_rad = f"{math.radians(30.0):.4f}"
    b_rad = f"{math.radians(75.0):.4f}"
    assert a_rad in codes[a], "A 自己的关节值没进 A 的程序"
    assert b_rad in codes[b], "B 自己的关节值没进 B 的程序"
    assert b_rad not in codes[a], "B 的关节角串进了 A 的程序（出码没按机器人分家）"


def test_generate_all_keeps_preview_on_active_robot(two_ur5e):
    """批量生成后，预览仍停在用户正在看的那台——不该悄悄换成别人家的代码。"""
    win, scene, a, b = two_ur5e
    panel = win.program_panel
    panel.set_taught_points(_points(panel, a, 30.0), robot=a)
    panel.set_taught_points(_points(panel, b, 75.0), robot=b)
    panel.set_active_robot(a)

    codes = panel.generate_all_code()

    assert panel.code_preview.toPlainText() == codes[a]


def test_switching_robot_preserves_each_ones_points(two_ur5e):
    """切换下拉时，两台的示教点各自留着，不会互相覆盖。"""
    win, scene, a, b = two_ur5e
    panel = win.program_panel
    panel.set_taught_points(_points(panel, a, 10.0), robot=a)
    panel.set_taught_points(_points(panel, b, 50.0), robot=b)

    panel.set_active_robot(a)
    assert len(panel._taught) == 2
    panel.set_active_robot(b)
    assert len(panel._taught) == 2
    assert panel._taught[1][panel._joint_names[0]] == pytest.approx(math.radians(50.0))
    panel.set_active_robot(a)
    assert panel._taught[1][panel._joint_names[0]] == pytest.approx(math.radians(10.0))


def test_program_robot_switch_moves_the_viewport(two_ur5e):
    """面板换机器人 → 视口的"当前机器人"也跟过去（否则编辑 A、动的是 B）。"""
    win, scene, a, b = two_ur5e
    win.gl_widget.set_current_robot(a)
    assert win.gl_widget.current_robot == a

    win._on_program_robot_changed(b)
    assert win.gl_widget.current_robot == b


def test_playback_all_drives_every_robot(two_ur5e):
    """多机回放：帧按机器人名分开发，且各自的关节值不同。

    回归背景：回放帧原先不带机器人名，多机时只能套到"当前"那台身上。
    """
    win, scene, a, b = two_ur5e
    panel = win.program_panel
    panel.set_taught_points(_points(panel, a, 20.0), robot=a)
    panel.set_taught_points(_points(panel, b, 60.0), robot=b)

    panel.set_active_robot(a)
    assert panel.validate_program() is not None
    panel.set_active_robot(b)
    assert panel.validate_program() is not None

    seen = []
    panel.playbackFrame.connect(lambda robot, vals: seen.append((robot, vals)))
    panel.start_playback_all()

    for _ in range(5):
        panel._on_tick()
    panel.stop_playback()

    robots = {robot for robot, _ in seen}
    assert {a, b} <= robots, f"两台都该收到帧，实际只有 {robots}"

    frames_a = [vals for robot, vals in seen if robot == a]
    frames_b = [vals for robot, vals in seen if robot == b]
    assert frames_a and frames_b
    assert frames_a[0] != frames_b[0], "两台不该跟着同一条轨迹走"


def test_step_playback_drives_points_owner_not_selection(two_ur5e):
    """逐点回放驱动的是**点表归属**那台，不是"当前选中"那台。

    回归：``on_playback_requested`` 原先一律写进当前渲染器。而视口点选 /
    场景树选中都只换"当前机器人"、不换路径编辑器的点表——用户在视口里点一下
    B 再点播放，A 的位形就被拽到 B 身上了。
    """
    win, scene, a, b = two_ur5e
    _teach_pick_cycle(win, a, 10.0)
    _teach_pick_cycle(win, b, -10.0)

    # 路径编辑器显示 A 的点，视口却停在 B（点选只换选中，不换点表）
    win.code_editor.set_recorded_positions(
        win.gl_widget.get_recorded_positions(a), a
    )
    win.gl_widget.set_current_robot(b)

    def first_joint(name):
        r = win.gl_widget.renderer_of(name)
        return r.joint_values[list(r.actuated_names())[0]]

    before_b = first_joint(b)
    win.on_playback_requested(0)  # A 的第 1 个点，与两台当前位形都不同

    assert first_joint(a) == pytest.approx(0.0, abs=1e-9), "A 是点的主人，该被驱动"
    assert first_joint(b) == pytest.approx(before_b, abs=1e-9), "B 不该被 A 的点拽动"


# --------------------------------------------------------------------------- #
# 4. 路径可视化属于机器人
# --------------------------------------------------------------------------- #


def _seed_two_points(win, scene, *names):
    """给每台机器人各记两个相同位形的示教点（路径几何才好对比）。"""
    for name in names:
        scene.set_robot_joints([30.0], name=name)
        win.gl_widget.renderer_of(name).joint_values["j1"] = math.radians(30.0)
        win.gl_widget.record_position(name)
        win.gl_widget.record_position(name)


def test_recorded_path_is_offset_by_its_own_base(two_robots):
    """末端路径按机器人归属：各自乘自己的基座位姿，不再全叠在原点。

    记录的末端点是渲染器 FK 算的、处在**机器人基座系**；不乘基座位姿的话，
    第二台的路径会整条落在原点上。
    """
    win, scene, a, b = two_robots
    _seed_two_points(win, scene, a, b)

    pa = win.gl_widget._path_points_for(a)
    pb = win.gl_widget._path_points_for(b)
    assert len(pa) == 2 and len(pb) == 2
    # 两台末端位形相同 → 世界系里只应差出基座那 2 m 偏移
    assert abs((pb[0][0] - pa[0][0]) - 2.0) < 1e-6
    assert abs(pb[0][1] - pa[0][1]) < 1e-6
    assert abs(pb[0][2] - pa[0][2]) < 1e-6


def test_path_robots_covers_every_robot(two_robots):
    """每台示教过的机器人都该被画出来，而不是只画当前那台。"""
    win, scene, a, b = two_robots
    _seed_two_points(win, scene, a, b)

    assert set(win.gl_widget._path_robots()) == {a, b}
    assert win.gl_widget._has_any_path() is True

    win.gl_widget.clear_recorded_positions()
    assert win.gl_widget._path_robots() == []


def test_pick_event_attaches_to_current_robot(two_robots):
    """示教点回放的抓取：工件挂到**当前那台**，不是写死的主机器人。

    回归：``_apply_pick_event`` 原先硬编码 ``ROBOT_NAME``，多机时 B 抓取会把
    工件挂到 A 的法兰上——工件横穿工位飞过去。
    """
    win, scene, a, b = two_robots
    scene.add("workpiece_box", name="工件")
    win.code_editor._scene = scene
    win.code_editor.refresh_workpiece_combo()
    idx = win.code_editor.workpiece_combo.findData("工件")
    win.code_editor.workpiece_combo.setCurrentIndex(idx)

    # 当前切到 B，第 0 个点标成抓取点
    win.gl_widget.set_current_robot(b)
    win.code_editor.recorded_positions = [{}, {}]
    win.code_editor.sync_point_types()
    win.code_editor.point_types = [PointType.PICK, PointType.PLACE]

    win._apply_pick_event(0)
    assert scene.objects["工件"].parent == b, "抓取没挂到当前机器人 B 上"

    # 释放只摘自己手里的：此时工件在 B 手上，切到 A 不该被 A 摘掉
    win.gl_widget.set_current_robot(a)
    win._apply_pick_event(1)
    assert scene.objects["工件"].parent == b, "A 不该摘掉 B 手里的工件"

    win.gl_widget.set_current_robot(b)
    win._apply_pick_event(1)
    assert scene.objects["工件"].parent is None, "B 自己该能释放"


def test_multi_robot_grasp_uses_each_ones_workpiece(two_robots):
    """多机同时抓取：各夹各的工件，不互相抢。

    回归：抓取对象原先是全局单选，B 一夹紧就把 A 手里那件抢过去了。
    """
    win, scene, a, b = two_robots
    scene.add("workpiece_box", name="工件A")
    scene.add("workpiece_box", name="工件B")
    win.code_editor._scene = scene
    win.code_editor.refresh_workpiece_combo()
    win._workpiece_by_robot = {a: "工件A", b: "工件B"}

    win.on_gripper_state_changed(a, "close")
    win.on_gripper_state_changed(b, "close")

    assert scene.objects["工件A"].parent == a
    assert scene.objects["工件B"].parent == b, "B 抢走了 A 的工件（或没夹自己的）"


def test_workpiece_selection_is_remembered_per_robot(two_ur5e):
    """下拉里选的工件**当场**记到当前机器人名下，不必等切换机器人。

    回归：``_workpiece_by_robot`` 原先只在切换机器人时补记。用户给 A 选完工件
    若没切走过，A 名下就是空的——多机回放时两台会去抢同一件工件。
    """
    win, scene, a, b = two_ur5e
    scene.add("workpiece_box", name="工件1")
    scene.add("workpiece_box", name="工件2")
    win.code_editor.set_scene(scene)
    win.code_editor.refresh_workpiece_combo()
    combo = win.code_editor.workpiece_combo

    win.gl_widget.set_current_robot(a)
    combo.setCurrentIndex(combo.findData("工件1"))
    win.gl_widget.set_current_robot(b)
    combo.setCurrentIndex(combo.findData("工件2"))

    assert win._workpiece_for(a) == "工件1"
    assert win._workpiece_for(b) == "工件2"


def test_switching_robot_restores_its_own_workpiece(two_ur5e):
    """切回 A 时，下拉回填成 A 的工件（不是停留在 B 那件）。"""
    win, scene, a, b = two_ur5e
    scene.add("workpiece_box", name="工件1")
    scene.add("workpiece_box", name="工件2")
    win.code_editor.set_scene(scene)
    win.code_editor.refresh_workpiece_combo()
    combo = win.code_editor.workpiece_combo

    win.gl_widget.set_current_robot(a)
    combo.setCurrentIndex(combo.findData("工件1"))
    win._on_program_robot_changed(b)
    combo.setCurrentIndex(combo.findData("工件2"))

    win._on_program_robot_changed(a)
    assert win.code_editor.get_workpiece() == "工件1"
    assert win._workpiece_for(b) == "工件2", "切走不该抹掉 B 的选择"


#: 作业位形（肩/肘/腕角度，度）：TCP 落在 z≈240 mm，抬高 40~70 mm 都在工作空间内
WORK_BASE_DEG = [-45.0, -80.0, -90.0, 90.0, 0.0]


def _mute_validation_dialog(win) -> None:
    """断开「校验全部」的汇总弹窗。

    真实交互里 Shift+F7 校完要弹**模态**汇总框给用户看，而自动化测试里没人
    点「关闭」，``validate_all()`` 就会一直停在 ``dlg.exec()`` 不返回——先把
    信号断掉，被测的仍是校验本身。
    """
    try:
        win.program_panel.allValidated.disconnect()
    except (RuntimeError, TypeError):  # pragma: no cover - 未连接时忽略
        pass


def _teach_pick_cycle(win, robot, step_deg: float, base_deg=None) -> None:
    """走真实示教通路给 ``robot`` 记一套五点式抓取循环（关节名取可动轴）。

    Args:
        base_deg: 首关节之外各轴的基准角度（度），缺省全零。UR 的全零位形是
            大臂竖直伸展、TCP 停在可达空间**上边缘**（UR5e 约 z=1020 mm），
            再抬高一点点都会逆解失败。只验证"点有没有记下来"的用例用全零
            足够；要按 TCP 算偏移点的用例（抓取向导）必须传一个作业位形。
    """
    win.gl_widget.set_current_robot(robot)
    renderer = win.gl_widget.renderer_of(robot)
    names = list(renderer.actuated_names())
    first = names[0]
    for i, name in enumerate(names[1:]):
        renderer.joint_values[name] = math.radians(
            base_deg[i] if base_deg and i < len(base_deg) else 0.0
        )
    for i in range(5):
        renderer.joint_values[first] = math.radians(i * step_deg)
        win.gl_widget.record_position(robot)
    # 带上归属：路径编辑器这份点表属于 ``robot``（真实通路里记录时会记下）
    win.code_editor.set_recorded_positions(
        win.gl_widget.get_recorded_positions(robot), robot
    )
    win.code_editor.sync_point_types()
    win._sync_program_points(robot)


def test_pick_semantics_are_archived_without_switching(two_ur5e):
    """示教完就归档本机抓取语义——不能只在切换机器人时才补记。

    否则用户给 A 示教完直接点「导出全部」，A 名下还是空的：程序里根本没有
    夹爪指令，出码看起来"能跑"却不夹东西。
    """
    win, scene, a, b = two_ur5e
    _teach_pick_cycle(win, a, 10.0)

    saved = win._point_types_by_robot.get(a) or []
    assert len(saved) == 5, f"语义没归档：{saved}"
    assert PointType.PICK in saved and PointType.PLACE in saved


def test_multi_robot_pick_playback_grasps_each_own_workpiece(two_ur5e):
    """端到端：两台各示教一套抓取循环，全部回放时各夹各的工件。"""
    win, scene, a, b = two_ur5e
    scene.add("workpiece_box", name="工件1")
    scene.add("workpiece_box", name="工件2")
    win.code_editor.set_scene(scene)
    win.code_editor.refresh_workpiece_combo()
    combo = win.code_editor.workpiece_combo

    _teach_pick_cycle(win, a, 10.0)
    combo.setCurrentIndex(combo.findData("工件1"))
    win.gl_widget.set_current_robot(b)
    _teach_pick_cycle(win, b, -10.0)
    combo.setCurrentIndex(combo.findData("工件2"))

    panel = win.program_panel
    for name in (a, b):
        panel.set_active_robot(name)
        assert panel.validate_program() is not None, f"{name} 校验没出轨迹"

    events = []
    # 夹紧那一刻工件挂到了谁的法兰下——抓取循环以 place 收尾，播完后工件本来
    # 就该是放下的，所以只能在事件当下核对，不能等播完再看
    attached = []

    def on_event(robot, act):
        wp = win._workpiece_for(robot)
        events.append((robot, act, wp))
        if act == "close":
            attached.append((robot, scene.objects[wp].parent))

    panel.gripperStateChanged.connect(on_event)
    panel.start_playback_all()
    for _ in range(80):
        panel._on_tick()
    panel.stop_playback()

    closed = {robot: wp for robot, act, wp in events if act == "close"}
    assert closed.get(a) == "工件1", f"A 夹的该是工件1，实际 {closed.get(a)}"
    assert closed.get(b) == "工件2", f"B 夹的该是工件2，实际 {closed.get(b)}"
    assert (a, a) in attached and (b, b) in attached, (
        f"工件没挂到各自的机器人上：{attached}"
    )


# --------------------------------------------------------------------------- #
# 6. 多机程序要存得住（工程往返）
# --------------------------------------------------------------------------- #


def test_project_round_trip_keeps_every_robots_pick_program(two_ur5e, tmp_path):
    """存盘 → 重新打开：每台机器人的抓取循环、语义、工件都还在。

    回归：工程里原先只有一份 ``program`` 段（来自只显示当前那台的路径编辑器），
    多机工程存盘再打开会丢掉其余机器人教好的抓取程序——"存不住"的抓取编程
    等于没做。
    """
    win, scene, a, b = two_ur5e
    scene.add("workpiece_box", name="工件1")
    scene.add("workpiece_box", name="工件2")
    win.code_editor.set_scene(scene)
    win.code_editor.refresh_workpiece_combo()
    combo = win.code_editor.workpiece_combo

    _teach_pick_cycle(win, a, 10.0)
    combo.setCurrentIndex(combo.findData("工件1"))
    win.gl_widget.set_current_robot(b)
    _teach_pick_cycle(win, b, -10.0)
    combo.setCurrentIndex(combo.findData("工件2"))

    from orolp import Project

    proj = Project(name="多机抓取")
    win._collect_into(proj)
    saved = proj.get("programs")
    assert saved is not None and set(saved) == {a, b}, f"存盘内容：{saved}"

    target = tmp_path / "多机抓取.orolp"
    proj.save(target)
    loaded = Project.load(target)
    assert loaded.has("programs")

    # 换一个**全新**的窗口打开，才能证明"登记 + 恢复"真能从文件里重建
    import app as rw

    fresh = rw.MainWindow()
    try:
        issues = fresh._apply_project(loaded)
        assert not issues, f"载入有问题：{issues}"
        panel = fresh.program_panel
        assert set(panel.robot_names) == {a, b}, "第二台机器人没被登记进编程面板"

        for name in (a, b):
            state = panel._state(name)
            assert len(state.taught) == 5, f"{name} 的示教点没回来"
            assert PointType.PICK in (state.point_types or []), (
                f"{name} 的抓取语义没回来"
            )
        assert fresh._workpiece_for(a) == "工件1"
        assert fresh._workpiece_for(b) == "工件2"
        # 回来就能直接出码（各机一份，且带夹爪指令）
        assert set(panel.generate_all_code()) == {a, b}
    finally:
        fresh.close()


def test_old_project_without_programs_still_opens(two_ur5e, tmp_path):
    """旧工程（只有单份 ``program`` 段）仍能打开——向后兼容。"""
    from orolp import Project

    win, scene, a, b = two_ur5e
    old = Project(name="旧工程")
    old.set("program", {"code": "movej([0,0,0,0,0,0])", "point_types": ["via"],
                        "joint_names": []})

    import app as rw

    fresh = rw.MainWindow()
    try:
        issues = fresh._apply_project(old)
        assert not issues, f"旧工程不该报问题：{issues}"
    finally:
        fresh.close()


# --------------------------------------------------------------------------- #
# 7. 一键校验全部（Shift+F7）
# --------------------------------------------------------------------------- #


def test_validate_all_gives_each_robot_its_own_result(two_ur5e):
    """一键校验：每台**各出各的**结果，界面还停在用户正在看的那台。

    回归：批量处理最容易犯的错是"切过去算完没切回来"——界面最后只剩最后一
    台的结论，前面几台校出问题等于没看见。
    """
    win, scene, a, b = two_ur5e
    panel = win.program_panel
    _mute_validation_dialog(win)
    _teach_pick_cycle(win, a, 10.0)
    _teach_pick_cycle(win, b, -10.0)
    # 两台路点数不同：串台立刻暴露（A 只剩 3 点）
    panel.set_taught_points(panel._state(a).taught[:3], robot=a)

    panel.set_active_robot(a)
    reports = panel.validate_all()

    assert set(reports) == {a, b}
    assert reports[a] is not None and reports[b] is not None
    assert len(panel._state(a).taught) == 3
    assert len(panel._state(b).taught) == 5
    # 各自的轨迹与报告分别落在自己名下，没有被最后一台顶掉
    assert panel._state(a).trajectory is not None
    assert panel._state(b).trajectory is not None
    assert reports[a].num_waypoints < reports[b].num_waypoints, (
        "A 的报告被 B 顶掉了"
    )
    # 界面没跟着乱跳
    assert panel._active == a


def test_validate_all_marks_programless_robot_as_no_result(two_ur5e):
    """没示教过的那台记为"没结果"，**不能**当成通过。"""
    win, scene, a, b = two_ur5e
    panel = win.program_panel
    _mute_validation_dialog(win)
    _teach_pick_cycle(win, a, 10.0)  # B 没教过

    reports = panel.validate_all()
    assert reports[a] is not None
    assert reports[b] is None, "没程序的机器人必须如实记为没结果"


def test_validate_all_makes_playback_all_ready(two_ur5e):
    """校完就能直接「全部回放」——不必逐台按 F7 攒轨迹。"""
    win, scene, a, b = two_ur5e
    panel = win.program_panel
    _mute_validation_dialog(win)
    _teach_pick_cycle(win, a, 10.0)
    _teach_pick_cycle(win, b, -10.0)

    assert panel._playable() == [], "还没校验就不该有可回放的轨迹"
    panel.validate_all()
    assert set(panel._playable()) == {a, b}


def test_focus_report_switches_to_that_robot(two_ur5e):
    """汇总里点「看这台」：切过去、显示它的报告、视口也跟过去。"""
    win, scene, a, b = two_ur5e
    panel = win.program_panel
    _mute_validation_dialog(win)
    _teach_pick_cycle(win, a, 10.0)
    _teach_pick_cycle(win, b, -10.0)
    panel.validate_all()

    assert panel.focus_report(b) is True
    assert panel._active == b
    assert win.gl_widget.current_robot == b, "视口没跟过去——看 A 的报告画 B 就乱了"


# --------------------------------------------------------------------------- #
# 8. 抓取向导
# --------------------------------------------------------------------------- #


def _fk_z(panel, robot_name: str, pos: dict) -> float:
    """示教点对应的 TCP 高度（毫米）——用 FK 回算，而不是信返回的关节值。"""
    state = panel._state(robot_name)
    q = [math.degrees(float(pos[n])) for n in state.joint_names]
    return float(state.robot.forward_kinematics(q).matrix[2, 3])


def test_grasp_wizard_puts_safety_points_straight_above(two_ur5e):
    """向导的全部价值：接近/提升/撤离点**真的**在示教点正上方指定高度。

    手示教"抬高 50 mm"全凭手感，向导按 TCP 位姿精确解算。故这里用 FK 回算验证，
    而不是只比对返回的关节值——关节值对了不代表末端到了该到的地方。
    """
    win, scene, a, b = two_ur5e
    panel = win.program_panel
    _teach_pick_cycle(win, a, 10.0, WORK_BASE_DEG)

    positions, types, motions = panel.grasp_cycle_points(
        0, 3, approach_mm=40.0, lift_mm=70.0, retract_mm=60.0, robot=a
    )
    assert [t.value for t in types] == ["approach", "pick", "lift", "place", "retract"]
    # 竖直升降用直线，转运段用关节
    assert motions == ["J", "L", "L", "J", "L"]

    # 容差取 0.05 mm：IK 默认位置收敛阈值是 0.01 mm，实测残差约 0.006 mm。
    # 对示教而言这已是过头精度（手示教的误差以毫米计）
    z_pick = _fk_z(panel, a, positions[1])
    assert _fk_z(panel, a, positions[0]) - z_pick == pytest.approx(40.0, abs=0.05)
    assert _fk_z(panel, a, positions[2]) - z_pick == pytest.approx(70.0, abs=0.05)
    z_place = _fk_z(panel, a, positions[3])
    assert _fk_z(panel, a, positions[4]) - z_place == pytest.approx(60.0, abs=0.05)
    # 抓取点必须是用户示教的那个位形，一个字节都不能重解
    assert positions[1] == panel._state(a).taught[0]


def test_grasp_wizard_pick_only_makes_three_points(two_ur5e):
    """只给抓取点（不给放置点）时生成抓取侧三点，不臆造放置位。"""
    win, scene, a, b = two_ur5e
    panel = win.program_panel
    _teach_pick_cycle(win, a, 10.0, WORK_BASE_DEG)

    positions, types, motions = panel.grasp_cycle_points(0, None, robot=a)
    assert [t.value for t in types] == ["approach", "pick", "lift"]
    assert motions == ["J", "L", "L"]
    assert len(positions) == 3


def test_grasp_wizard_refuses_unreachable_offset(two_ur5e):
    """抬到工作空间外：如实报错，且**不动**用户的示教点。

    宁可让用户把高度调小重试，也不能给一串残缺或够不着的点位。
    """
    win, scene, a, b = two_ur5e
    panel = win.program_panel
    # 作业位形下抬 5 m：够不着的是**高度**，不是这个位形本身
    _teach_pick_cycle(win, a, 10.0, WORK_BASE_DEG)
    before = [dict(p) for p in panel._state(a).taught]

    with pytest.raises(KinematicsError):
        panel.grasp_cycle_points(0, None, approach_mm=5000.0, robot=a)
    assert panel._state(a).taught == before, "算不出来就不该动用户的示教点"


def test_grasp_wizard_append_and_replace(two_ur5e):
    """默认追加（不抹掉用户一个个摆出来的点），勾选后才替换。"""
    win, scene, a, b = two_ur5e
    panel = win.program_panel
    _teach_pick_cycle(win, a, 10.0, WORK_BASE_DEG)
    n0 = len(panel._state(a).taught)

    total = panel.apply_grasp_cycle(0, 3, replace=False, robot=a)
    assert total == n0 + 5
    positions, types, _motions = panel.get_taught_points(a)
    assert len(types) == total, "追加后语义必须补齐，否则整串错位"

    total2 = panel.apply_grasp_cycle(0, 3, replace=True, robot=a)
    assert total2 == 5
    assert [t.value for t in panel.get_taught_points(a)[1]] == [
        "approach", "pick", "lift", "place", "retract"
    ]


def test_grasp_wizard_leaves_other_robot_alone(two_ur5e):
    """给 A 用向导，B 一个点都不能动——多机最怕的就是这笔串台。"""
    win, scene, a, b = two_ur5e
    panel = win.program_panel
    _teach_pick_cycle(win, a, 10.0, WORK_BASE_DEG)
    _teach_pick_cycle(win, b, -10.0, WORK_BASE_DEG)
    win.gl_widget.set_recorded_positions(panel.get_taught_points(b)[0], b)

    b_points = [dict(p) for p in panel._state(b).taught]
    b_view = len(win.gl_widget.get_recorded_positions(b))
    win.gl_widget.set_current_robot(b)  # 视口停在 B

    panel.apply_grasp_cycle(0, 3, robot=a)

    assert panel._state(b).taught == b_points, "给 A 用向导却改了 B 的点"
    assert len(win.gl_widget.get_recorded_positions(b)) == b_view
    assert win.gl_widget.current_robot == b
    # A 的新点要回灌到 A 自己的视口桶，否则视口里看不见向导生成的路径
    assert len(win.gl_widget.get_recorded_positions(a)) == len(panel._state(a).taught)


def test_grasp_target_is_kept_per_robot(two_robots):
    """切换机器人时，抓取对象跟着换成本台那件。"""
    win, scene, a, b = two_robots
    scene.add("workpiece_box", name="工件A")
    scene.add("workpiece_box", name="工件B")
    win.code_editor._scene = scene
    win.code_editor.refresh_workpiece_combo()
    combo = win.code_editor.workpiece_combo

    win.gl_widget.set_current_robot(a)
    combo.setCurrentIndex(combo.findData("工件A"))

    win._on_program_robot_changed(b)
    combo.setCurrentIndex(combo.findData("工件B"))
    assert win.code_editor.get_workpiece() == "工件B"

    win._on_program_robot_changed(a)
    assert win.code_editor.get_workpiece() == "工件A", "切回 A 后抓取对象被顶掉了"


def test_point_types_are_kept_per_robot(two_robots):
    """抓取/放置语义按机器人各自保存：切走再切回来不能丢。

    丢了语义 = 程序里没有夹爪指令，多机抓取直接残废。
    """
    win, scene, a, b = two_robots
    win.gl_widget.set_current_robot(a)

    # 给 A 记两个点，语义 抓取/放置
    win.gl_widget.record_position(a)
    win.gl_widget.record_position(a)
    win._sync_program_points(a)
    win.code_editor.point_types = [PointType.PICK, PointType.PLACE]

    # 切到 B（内部会把 A 的语义存起来）
    win._on_program_robot_changed(b)

    # 给 B 设成相反的语义
    win.gl_widget.record_position(b)
    win.gl_widget.record_position(b)
    win._sync_program_points(b)
    win.code_editor.point_types = [PointType.PLACE, PointType.PICK]

    # 切回 A：语义必须是 A 自己的那份
    win._on_program_robot_changed(a)
    assert list(win.code_editor.get_point_types()) == [PointType.PICK, PointType.PLACE]

    # 再切到 B：语义是 B 自己的那份
    win._on_program_robot_changed(b)
    assert list(win.code_editor.get_point_types()) == [PointType.PLACE, PointType.PICK]


def test_removing_robot_clears_its_program(two_ur5e):
    """删除机器人 → 它的程序/示教点/渲染器一并清除，且**不牵连**其他机器人。

    回归：只从场景删对象会留下孤儿状态——面板里还挂着这台的程序，照样能
    校验、能出码，可对应的机器人已经不在了。
    """
    win, scene, a, b = two_ur5e
    panel = win.program_panel
    panel.set_taught_points(_points(panel, a, 30.0), robot=a)
    panel.set_taught_points(_points(panel, b, 75.0), robot=b)
    win.gl_widget.record_position(a)
    assert a in win.gl_widget.renderers
    assert set(panel.robot_names) == {a, b}

    win.scene_panel._selected = a
    win.scene_panel._remove_selected()

    assert a not in panel.robot_names, "删掉的机器人程序没被清除"
    assert panel.robot_names == [b], "不该牵连还在场上的机器人"
    assert a not in win.gl_widget.renderers, "渲染器没回收"
    assert win.gl_widget.get_recorded_positions(a) == [], "示教点没清除"
    # 剩下的那台照旧能用（程序没被误伤）
    assert set(panel.generate_all_code()) == {b}


def test_clearing_scene_clears_all_robot_programs(two_ur5e, monkeypatch):
    """清空场景 → 所有机器人的程序一并清除（不留无主的条目）。"""
    from PySide6 import QtWidgets as _QtWidgets

    monkeypatch.setattr(
        _QtWidgets.QMessageBox,
        "question",
        staticmethod(lambda *a, **k: _QtWidgets.QMessageBox.Yes),
    )
    win, scene, a, b = two_ur5e
    panel = win.program_panel
    panel.set_taught_points(_points(panel, a, 30.0), robot=a)
    panel.set_taught_points(_points(panel, b, 75.0), robot=b)

    win.scene_panel._clear_scene()

    assert panel.robot_names == [], "清空场景后不该还剩机器人程序"
    assert panel.generate_all_code() == {}


def test_path_robots_skips_deleted_robot(two_robots):
    """机器人从场景删掉后，它的路径不该留下一条无主的线。"""
    win, scene, a, b = two_robots
    _seed_two_points(win, scene, a, b)
    assert set(win.gl_widget._path_robots()) == {a, b}

    scene.remove(a)
    assert win.gl_widget._path_robots() == [b]


# --------------------------------------------------------------------------- #
# 9. 执行范围：只跑选中那台 / 从选中行号处全局运行
# --------------------------------------------------------------------------- #


def _two_robot_code(a: str, b: str) -> str:
    """两台机器人混排的一段 OPL 码：A 三条、B 两条，归属靠 ``# Robot:`` 分段。

    行号（1 起）：1 码头、2 归属头、3~5 归 A、6 归属头、7~8 归 B。
    """
    lines = ["# OLP Path Code", f"# Robot: {a}"]
    lines += [f"movej([{i * 10}.00], vel=0.5)" for i in range(3)]
    lines.append(f"# Robot: {b}")
    lines += [f"movej([{i * 10}.00], vel=0.5)" for i in range(3, 5)]
    return "\n".join(lines) + "\n"


def _load_code(win, code: str, robot):
    """给路径编辑器换上 ``code``，并按 ``robot`` 的关节名重建解析。"""
    editor = win.code_editor
    editor.set_joint_names(list(win.gl_widget.renderer_of(robot).actuated_names()))
    editor.setPlainText(code)
    editor._parse_code_to_positions()
    return editor


def test_path_code_carries_which_robot_each_move_belongs_to(two_robots):
    """归属按段解析——"跑哪台"全靠这份映射，光有点值序列是定位不了的。"""
    win, scene, a, b = two_robots
    editor = _load_code(win, _two_robot_code(a, b), a)

    assert len(editor.recorded_positions) == 5
    assert [editor.robot_at(i) for i in range(5)] == [a, a, a, b, b]
    # 指令行号：码头两行之后，A 占 3~5 行，B 占 7~8 行
    assert [editor.line_of(i) for i in range(5)] == [3, 4, 5, 7, 8]


def test_playback_runs_only_the_selected_robots_path(two_robots):
    """选中 A → 只排 A 的三条；真播一步，B 的关节值一动不动。"""
    win, scene, a, b = two_robots
    editor = _load_code(win, _two_robot_code(a, b), a)
    win.gl_widget.selected_object = a

    assert win._robot_for_selection() == a
    seq = editor.playback_plan(robot=win._robot_for_selection())
    assert seq == [0, 1, 2], f"选中 A 却把 B 的指令排了进来：{seq}"

    before_b = dict(win.gl_widget.renderer_of(b).joint_values)
    editor._playback_seq = seq
    editor._playback_pos = 0
    editor._emit_current()
    assert dict(win.gl_widget.renderer_of(b).joint_values) == before_b


def test_global_playback_starts_at_the_cursor_line(two_robots):
    """不选设备 → 从光标所在行起跑，前面的指令不执行。"""
    from PySide6 import QtGui

    win, scene, a, b = two_robots
    editor = _load_code(win, _two_robot_code(a, b), a)
    win.gl_widget.selected_object = None
    assert win._robot_for_selection() is None

    # 光标落到第 4 行（A 的第二条指令）
    block = editor.code_edit.document().findBlockByNumber(3)
    editor.code_edit.setTextCursor(QtGui.QTextCursor(block))
    assert editor._cursor_line() == 4

    seq = editor.playback_plan(from_cursor=True)
    assert seq == [1, 2, 3, 4], f"该从第 4 行起跑，实际是 {seq}"


def test_selection_of_mounted_object_runs_that_robots_path(two_robots):
    """选中挂在机器人上的东西 = 选中那台；没挂载的关联不到 → 走全局运行。"""
    win, scene, a, b = two_robots
    _load_code(win, _two_robot_code(a, b), a)

    held = scene.add("workpiece_box", name="工件1").name
    scene.attach(held, a, keep_world=True)  # 被 A 夹住
    win.gl_widget.selected_object = held
    assert win._robot_for_selection() == a, "工件挂在 A 上，就该跑 A 的路径"

    # 放在场景根，没挂在任何机器人下
    loose = scene.add("workpiece_box", name="工件2").name
    win.gl_widget.selected_object = loose
    assert win._robot_for_selection() is None, "关联不到机器人 → 全局运行"


def test_playback_stops_when_the_owner_robot_is_gone(two_robots):
    """归属那台已不在场景里：停下来报错，而不是画面不动却显示"运行中"。 """
    win, scene, a, b = two_robots
    editor = _load_code(win, "# OLP Path Code\nmovej([10.00], vel=0.5)\n", a)
    editor.owner_robot = "早就删掉的机器人"
    editor._parse_code_to_positions()

    stopped = []
    editor.playbackStopped.connect(lambda: stopped.append(1))
    editor._playback_seq = editor.playback_plan()
    editor._playback_pos = 0
    editor._emit_current()

    assert stopped, "机器人不在了必须停表"
    assert editor._playback_pos == 0, "停表后不该继续往下走"
    assert "已不在场景" in win.statusBar().currentMessage()
