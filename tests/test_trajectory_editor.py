"""GUI 轨迹编辑器（v0.2.4）的无头测试。

覆盖离线编程面板新增的轨迹编辑能力：

* 逐点运动类型（moveJ / moveL）——moveL 以 FK 位姿进程序，规划器沿路 IK
* 路点插入（复制源点）/ 删除 / 上下移，且 ``taughtPointsChanged`` 广播
* 关节角手填（度 → 弧度）与运动类型联改
* 校验后路点着色：错误红 / 警告橙 / 正常绿，并携带逐指令节拍
* 一键导出：不先「生成代码」也能按品牌扩展名落盘（UR → .script）
* 外部重复导入不覆盖编辑器已改的运动类型（同长度沿用）

全部跑在 Qt offscreen 平台上，不需要真实显示服务器。
"""

import math
import os
import sys
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from PySide6 import QtWidgets  # noqa: E402

from domain.robot import Robot  # noqa: E402
from domain.pose import Pose  # noqa: E402
from service.program import MotionType  # noqa: E402
from app.views.program_panel import ProgramPanel  # noqa: E402

URDF = REPO_ROOT / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"

JOINTS = [f"joint_{i}" for i in range(1, 7)]
#: UR5e 一个非奇异常用位形（度）
HOME = [0.0, -90.0, 90.0, -90.0, 0.0, 0.0]
#: 与 HOME 保持一定距离、直线可逆解的第二个位形（度）
SECOND = [15.0, -80.0, 80.0, -85.0, 10.0, 0.0]


@pytest.fixture(scope="module")
def qapp():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _positions(joints_list):
    return [{name: math.radians(v) for name, v in zip(JOINTS, joints)} for joints in joints_list]


@pytest.fixture()
def panel(qapp):
    robot = Robot(str(URDF), "UR5e")
    p = ProgramPanel()
    p.set_robot(robot, JOINTS)
    p.set_taught_points(_positions([HOME, SECOND]))
    return p


# --------------------------------------------------------------------------- #
# 运动类型：默认 moveJ / 逐点切 moveL
# --------------------------------------------------------------------------- #


def test_default_motions_are_joint(panel):
    _, _, motions = panel.get_taught_points()
    assert motions == ["J", "J"]
    program = panel.build_program()
    assert program is not None
    assert all(m.type == MotionType.JOINTS for m in program.motions)


def test_point_motion_switches_to_linear(panel):
    assert panel.set_point_motion(1, "L") is True
    _, _, motions = panel.get_taught_points()
    assert motions == ["J", "L"]
    program = panel.build_program()
    types = [m.type for m in program.motions]
    assert types == [MotionType.JOINTS, MotionType.LINEAR]
    # moveL 的目标是该示教点 FK 得到的 TCP 位姿（毫米+度）
    expected = panel._robot.forward_kinematics(SECOND)
    target = program.motions[1].target
    assert isinstance(target, Pose)
    assert target.to_xyz_quat() == pytest.approx(expected.to_xyz_quat(), abs=1e-6)


def test_linear_point_plans_and_validates(panel):
    """moveL 点走规划器：直线插补 + 逐点逆解，节拍可算、校验通过。"""
    panel.set_point_motion(1, "L")
    report = panel.validate_program()
    assert report is not None
    assert report.ok
    assert report.cycle_time is not None


def test_move_l_motion_type_appears_in_list(panel):
    panel.set_point_motion(0, "L")
    text = panel.point_list.item(0).text()
    assert "moveL" in text
    assert "moveJ" in panel.point_list.item(1).text()


# --------------------------------------------------------------------------- #
# 路点增删移改 + 变更广播
# --------------------------------------------------------------------------- #


def test_insert_deletes_and_moves_broadcast(panel):
    events = []
    panel.taughtPointsChanged.connect(lambda p, t, m: events.append((len(p), list(m))))
    assert panel.insert_point(0) == 1
    assert len(panel._taught) == 3
    # 复制源点的关节值
    assert panel._taught[1] == panel._taught[0]
    assert events[-1] == (3, ["J", "J", "J"])

    assert panel.delete_point(1) is True  # 删掉副本，保留原两点
    assert len(panel._taught) == 2

    assert panel.move_point(0, 1) is True
    # 顺序确实交换：原 P2（SECOND）现在在前面
    assert math.degrees(panel._taught[0]["joint_2"]) == pytest.approx(-80.0)
    assert math.degrees(panel._taught[1]["joint_2"]) == pytest.approx(-90.0)
    assert events[-1] == (2, ["J", "J"])


def test_insert_without_selection_appends(panel):
    panel.point_list.clearSelection()
    panel.point_list.setCurrentRow(-1)
    assert panel.insert_point() == len(panel._taught) - 1
    assert len(panel._taught) == 3


# --------------------------------------------------------------------------- #
# 按钮链路：``clicked`` 自带 bool，不能灌进带可选参数的槽
# --------------------------------------------------------------------------- #


def test_buttons_act_on_selected_row(panel):
    """插入/删除/上移按钮必须作用于**选中**那一行。

    回归：``clicked`` 带一个 bool 参数，直接连到 ``insert_point(index=None)``
    / ``delete_point(index=None)`` 会把 ``False``（等于 0）当成行号——删除永远
    砍掉 P1，用户选中谁都没用。
    """
    panel.set_taught_points(_positions([HOME, SECOND, [30.0] + HOME[1:], [45.0] + HOME[1:]]))

    panel.point_list.setCurrentRow(2)  # 选中 P3
    panel.btn_insert_point.click()
    assert panel.point_list.currentRow() == 3, "插入后选中该落在新那一行"
    assert len(panel._taught) == 5

    panel.btn_delete_point.click()
    assert len(panel._taught) == 4
    assert panel.point_list.currentRow() == 3, "删除后选中要落在顶上来的那行"

    panel.point_list.setCurrentRow(1)
    second = dict(panel._taught[1])
    panel.btn_move_up.click()
    assert panel._taught[0] == second, "上移没把选中的点搬上去"
    assert panel.point_list.currentRow() == 0, "移动的点是选中的，选中要跟着它"


def test_move_renumbers_and_reports(panel):
    """移动后 P 序号按新行序重排（第 N 行 = P{N}），并有状态栏回执。"""
    panel.set_taught_points(_positions([HOME, SECOND, [30.0] + HOME[1:]]))
    panel.point_list.setCurrentRow(2)
    messages = []
    panel.statusMessage.connect(messages.append)

    panel.btn_move_up.click()

    assert panel.point_list.item(0).text().startswith("P1")
    assert messages, "移动后没有任何回执，用户会以为按钮没反应"
    assert "P3" in messages[-1] and "P2" in messages[-1]


def test_move_keeps_row_selected_not_just_current(panel):
    """移动后那一行必须是**选中**（蓝色高亮），而不只是"当前行"。

    回归：列表整体重建后 ``setCurrentRow`` 只设当前行、不设选中——深色主题
    下重建出来的行没有选中态，交替行淡色透出来，看着就是"选中的颜色变淡了"
    （2026-09-21 用户反馈）。
    """
    panel.set_taught_points(_positions([HOME, SECOND, [30.0] + HOME[1:]]))
    panel.point_list.setCurrentRow(2)

    panel.btn_move_up.click()

    assert panel.point_list.currentRow() == 1
    assert panel.point_list.item(
        1
    ).isSelected(), "移动过的那一行应保持选中高亮，而不是淡出的交替行色"


def test_real_machine_buttons_hidden_in_mvp(panel, monkeypatch):
    """MVP 档不建「上传真机 / 真机验证」入口：藏掉，而不是置灰。"""
    import app.views.program_panel as pp_mod

    assert hasattr(panel, "btn_upload"), "测试档（全功能）应有这两个按钮"
    monkeypatch.setattr(pp_mod, "_feature_enabled", lambda name: False)
    hidden = type(panel)()
    try:
        assert not hasattr(hidden, "btn_upload"), "MVP 档不该有「上传真机」"
        assert not hasattr(hidden, "btn_verify"), "MVP 档不该有「真机验证」"
    finally:
        hidden.deleteLater()


def test_no_motion_type_buttons_in_panel(panel):
    """运动类型不在这里改：面板里没有「设为 moveJ/moveL/moveC」那排按钮。"""
    for name in ("btn_motion_j", "btn_motion_l", "btn_motion_c"):
        assert not hasattr(panel, name), f"{name} 还在，改动入口又多了一处"


def test_update_point_degrees_to_radians(panel):
    new = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    assert panel.update_point(0, new, "L") is True
    pos = panel._taught[0]
    for name, v in zip(JOINTS, new):
        assert math.degrees(pos[name]) == pytest.approx(v)
    _, _, motions = panel.get_taught_points()
    assert motions == ["L", "J"]


def test_external_resync_same_length_keeps_motions(panel):
    """主窗口回灌同长度示教点（如代码编辑器文本重建）时，逐点运动类型保留。"""
    panel.set_point_motion(0, "L")
    panel.set_taught_points(_positions([HOME, SECOND]))
    _, _, motions = panel.get_taught_points()
    assert motions == ["L", "J"]


def test_external_resync_new_length_resets_motions(panel):
    panel.set_point_motion(0, "L")
    panel.set_taught_points(_positions([HOME, SECOND, HOME]))
    _, _, motions = panel.get_taught_points()
    assert motions == ["J", "J", "J"]


def test_edit_changes_invalidate_previous_trajectory(panel):
    assert panel.validate_program() is not None
    assert panel.trajectory is not None
    panel.update_point(0, [5.0, -88.0, 88.0, -88.0, 5.0, 0.0])
    # 旧轨迹/报告作废：编辑后必须重新校验才能回放
    assert panel.trajectory is None
    assert panel.btn_play_all.isEnabled() is False


# --------------------------------------------------------------------------- #
# 校验着色：错误红 / 警告橙 / 正常绿 + 逐指令节拍
# --------------------------------------------------------------------------- #


def _item_color(panel, row):
    return panel.point_list.item(row).foreground().color().name()


def test_coloring_matches_report(panel):
    assert panel.validate_program() is not None
    levels = panel._point_issue_level
    for i in range(len(panel._taught)):
        color = _item_color(panel, i)
        level = levels.get(i)
        if level == "error":
            assert color == "#ff8080"
        elif level == "warning":
            assert color == "#ffc866"
        else:
            assert color == "#6fdc6f"
    # 列表文本带逐指令节拍与运动类型
    text = panel.point_list.item(0).text()
    assert "moveJ" in text
    assert "s" in text.split("·")[-1]


def test_coloring_red_on_error(panel):
    # 肩部 500° 远超关节限位 → 该指令规划失败（error 级）
    bad = dict(_positions([[500.0, -90.0, 90.0, -90.0, 0.0, 0.0]])[0])
    panel._taught = [bad, _positions([SECOND])[0]]
    panel._point_motions = ["J", "J"]
    panel._refresh_point_list()
    report = panel.validate_program()
    assert report is not None and not report.ok
    assert panel._point_issue_level.get(0) == "error"
    assert _item_color(panel, 0) == "#ff8080"


# --------------------------------------------------------------------------- #
# 一键导出：不先「生成代码」也能按品牌扩展名落盘
# --------------------------------------------------------------------------- #


def test_quick_export_writes_brand_extension(panel, tmp_path):
    program = panel.build_program()
    assert program is not None
    assert panel.export_program_to(program, str(tmp_path)) is True
    # UR 品牌自动推断 → .script
    files = [f for f in tmp_path.rglob("*") if f.is_file()]
    assert files, "导出目录为空"
    assert any(f.suffix == ".script" for f in files)
    assert any(f.name.endswith(".manifest.json") for f in files)


def test_kuka_export_uses_src(panel, tmp_path):
    for i in range(panel.combo_post.count()):
        data = panel.combo_post.itemData(i)
        if data and "kuka" in str(data):
            panel.combo_post.setCurrentIndex(i)
            break
    program = panel.build_program()
    assert panel.export_program_to(program, str(tmp_path)) is True
    assert any(f.suffix == ".src" for f in tmp_path.rglob("*.src"))
