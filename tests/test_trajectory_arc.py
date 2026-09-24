"""GUI 轨迹编辑器圆弧（moveC）编辑的无头测试（v1.1.5）。

背景：v0.2.4 的轨迹编辑器只能逐点切 moveJ / moveL，圆弧只能靠外部程序灌
示教点。本轮的圆弧编辑把 moveC 也搬进界面，并顺带修掉"路点 ↔ 运动指令一一
对应"这个旧前提——圆弧的**中间点被并进上一条指令**，因此节拍明细与校验着色
都必须经映射取值。

编辑器采用的口径（值得先记住，否则测试里的索引很容易看错）：

    第 N 行的运动类型描述的是**"到达第 N 点"的那段运动**。
    ``C`` 表示"以上一点为中间点、再上一点为起点，画圆弧到达本点"。

于是第 1、2 行不可能有圆弧（没有两个前驱点），相邻两行不能同为圆弧，抓取/
放置点不能当中间点（中间点不停机，夹爪动作会失去落点）。

全部跑在 Qt offscreen 平台上。
"""

import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

from PySide6 import QtWidgets  # noqa: E402

from orolp import Robot  # noqa: E402
from orolp.i18n import set_language  # noqa: E402
from orolp.pickplace import PointType  # noqa: E402
from orolp.planner import (  # noqa: E402
    SEVERITY_ERROR,
    Issue,
    ProgramReport,
)
from orolp.program import MotionType  # noqa: E402
from orolp.program_panel import ProgramPanel  # noqa: E402

URDF = REPO_ROOT / "models" / "ur" / "ur5e" / "ur5e.urdf"

JOINTS = [f"joint_{i}" for i in range(1, 7)]

#: 起点
P_START = [0.0, -90.0, 90.0, -90.0, 0.0, 0.0]
#: 圆弧中间点（TCP 明显偏离起终点连线，弧才真的是弧）
P_VIA = [30.0, -70.0, 70.0, -80.0, 20.0, 0.0]
#: 圆弧终点
P_ARC_END = [45.0, -60.0, 60.0, -75.0, 30.0, 0.0]
#: 后续直线/关节点
P_TAIL = [60.0, -50.0, 50.0, -70.0, 40.0, 0.0]

POINTS = [P_START, P_VIA, P_ARC_END, P_TAIL]

#: 编辑器里 moveC 的代号
ARC = "C"


@pytest.fixture(scope="module")
def qapp():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _positions(joints_list):
    return [
        {name: math.radians(v) for name, v in zip(JOINTS, joints)}
        for joints in joints_list
    ]


@pytest.fixture()
def panel(qapp):
    """四点面板：起点 / 中间点 / 圆弧终点 / 尾点。"""
    p = ProgramPanel()
    p.set_robot(Robot(str(URDF), "UR5e"), JOINTS)
    p.set_taught_points(_positions(POINTS))
    return p


@pytest.fixture()
def recorder(panel):
    """收集 ``statusMessage`` 的辅助对象。"""
    messages = []
    panel.statusMessage.connect(messages.append)
    return messages


def _tcp(panel, joints_deg):
    return np.array(panel._robot.forward_kinematics(list(joints_deg)).to_xyz_quat()[:3])


# --------------------------------------------------------------------------- #
# 圆弧可行性判据（arc_rejection）
# --------------------------------------------------------------------------- #


def test_first_two_rows_cannot_be_arc(panel):
    """P1/P2 没有两个前驱点，不能设圆弧。"""
    for index in (0, 1):
        reason = panel.arc_rejection(index)
        assert reason is not None
        assert "P%d" % (index + 1) in reason
        assert "moveC" in reason


def test_third_row_accepts_arc(panel):
    assert panel.arc_rejection(2) is None
    assert panel.set_point_motion(2, ARC) is True
    assert panel.get_taught_points()[2] == ["J", "J", "C", "J"]


def test_arc_rejected_when_row_out_of_range(panel):
    assert panel.arc_rejection(99) is not None


def test_adjacent_arcs_rejected_after_earlier_arc(panel, recorder):
    """P3 已是圆弧终点 → P4 不能把 P3 当中间点。"""
    assert panel.set_point_motion(2, ARC) is True
    recorder.clear()
    assert panel.set_point_motion(3, ARC) is False
    assert panel.get_taught_points()[2] == ["J", "J", "C", "J"]
    assert "中间点" in recorder[-1]


def test_adjacent_arcs_rejected_before_later_arc(panel, recorder):
    """P4 把 P3 当中间点 → P3 不能同时作为圆弧终点。"""
    assert panel.set_point_motion(3, ARC) is True
    assert panel.get_taught_points()[2] == ["J", "J", "J", "C"]
    recorder.clear()
    assert panel.set_point_motion(2, ARC) is False
    assert panel.get_taught_points()[2] == ["J", "J", "J", "C"]
    assert "圆弧终点" in recorder[-1]


def test_pick_point_cannot_be_via(panel, recorder):
    """抓取点不能当中间点——中间点不停机，夹爪动作会失去落点。"""
    panel.set_taught_points(
        _positions(POINTS),
        types=[PointType.PICK, PointType.PICK, PointType.PLACE, PointType.PLACE],
    )
    assert panel.arc_rejection(2) is not None
    recorder.clear()
    assert panel.set_point_motion(2, ARC) is False
    assert panel.get_taught_points()[2] == ["J", "J", "J", "J"]
    assert "抓取/放置" in recorder[-1]


def test_edit_dialog_path_rejects_arc_and_says_why(panel, recorder):
    """``update_point``（编辑对话框的落点）同样受判据约束，且必须说明原因。"""
    recorder.clear()
    assert panel.update_point(1, P_VIA, ARC) is True
    # 关节值写进去了，但运动类型被拒（没能悄悄变成圆弧）
    assert panel.get_taught_points()[2] == ["J", "J", "J", "J"]
    assert "moveC" in recorder[-1]


# --------------------------------------------------------------------------- #
# 运动序列解析（motion_plan / instruction_of_point）
# --------------------------------------------------------------------------- #


def test_plan_merges_via_into_the_arc(panel):
    """``C`` 点解析为 (C, 终点行, 中间点行)，中间点不再单独出指令。"""
    panel.set_point_motion(2, ARC)
    plan, notes = panel.motion_plan()
    assert plan == [("J", 0, None), ("C", 2, 1), ("J", 3, None)]
    assert notes == []
    assert panel.arc_via_indices() == {1}


def test_instruction_of_point_skips_the_via(panel):
    panel.set_point_motion(2, ARC)
    mapping = panel.instruction_of_point()
    # 4 个路点 → 3 条指令，中间点没有自己的指令
    assert mapping == {0: 0, 2: 1, 3: 2}
    assert 1 not in mapping


def test_via_row_has_no_cycle_time_and_later_rows_keep_theirs(panel):
    """节拍明细经映射取值——中间点之后的每一行都不能错位一行。"""
    panel.set_point_motion(2, ARC)
    report = panel.validate_program()
    assert report is not None and report.ok
    assert report.motion_times is not None and len(report.motion_times) == 3

    via_text = panel.point_list.item(1).text()
    assert "moveC 中间点" in via_text
    assert "s" not in via_text.split("·")[-1]  # 中间点行不带节拍

    # 尾点拿到的是第 3 条指令的节拍（不是第 4 条——那条不存在）
    tail_text = panel.point_list.item(3).text()
    assert f"{report.motion_times[2]:.2f}s" in tail_text


def test_arc_rejection_at_declaration_level_degrades_with_a_note(panel):
    """声明期就不可行的圆弧（如 P2 设 C）不会进程序，但会在 plan 里降级并记明原因。"""
    panel._point_motions = ["J", "C", "J", "J"]  # 绕过 UI 直接造出非法声明
    plan, notes = panel.motion_plan()
    assert plan == [("J", 0, None), ("J", 1, None), ("J", 2, None), ("J", 3, None)]
    assert len(notes) == 1
    assert "moveJ" in notes[0]


# --------------------------------------------------------------------------- #
# 程序构建：moveC 指令 + 几何真的是弧
# --------------------------------------------------------------------------- #


def test_build_program_emits_circular_motion(panel):
    panel.set_point_motion(2, ARC)
    program = panel.build_program()
    assert program is not None
    types = [m.type for m in program.motions]
    assert types == [MotionType.JOINTS, MotionType.CIRCULAR, MotionType.JOINTS]

    arc = program.motions[1]
    # 目标＝圆弧终点，via＝中间点，两者都是各自示教点的 FK 位姿
    assert arc.target.to_xyz_quat() == pytest.approx(
        panel._robot.forward_kinematics(P_ARC_END).to_xyz_quat(), abs=1e-6
    )
    assert arc.via is not None
    assert arc.via.to_xyz_quat() == pytest.approx(
        panel._robot.forward_kinematics(P_VIA).to_xyz_quat(), abs=1e-6
    )
    assert panel.last_build_notes() == []


def test_arc_validates_and_shows_movec(panel):
    panel.set_point_motion(2, ARC)
    report = panel.validate_program()
    assert report is not None
    assert report.ok and not report.errors
    assert report.motion_labels == ["moveJ", "moveC", "moveJ"]
    assert report.motion_times is not None and len(report.motion_times) == 3


def test_arc_merges_one_instruction_away(panel):
    """同一组示教点：全关节 4 条指令，带圆弧 3 条（中间点被并进圆弧）。"""
    joint_report = panel.validate_program()
    assert joint_report is not None
    assert joint_report.motion_labels == ["moveJ"] * 4

    panel.set_point_motion(2, ARC)
    arc_report = panel.validate_program()
    assert arc_report is not None
    assert len(arc_report.motion_times) == 3
    # 弧长 > 弦长：经过中间点的弧比关节运动的直达路径更长
    assert arc_report.cycle_time > joint_report.cycle_time


def test_arc_path_really_curves(panel):
    """规划出的 TCP 必须真的贴着中间点绕过去，而不是走成直线。"""
    panel.set_point_motion(2, ARC)
    report = panel.validate_program()
    assert report is not None and report.ok
    planner = panel._planner
    assert planner is not None

    start = _tcp(panel, P_START)
    via = _tcp(panel, P_VIA)
    end = _tcp(panel, P_ARC_END)

    # 只取第 2 条指令（圆弧段）的路径点，别把前后两段关节运动的点算进来
    arc_pts = [
        np.array(w.pose.to_xyz_quat()[:3])
        for w in panel.trajectory.waypoints
        if planner.motion_at(w.timestamp) == 1
    ]
    assert len(arc_pts) > 10, "圆弧段路径点太少，检验无意义"

    # 1) 经过中间点附近
    nearest = min(float(np.linalg.norm(p - via)) for p in arc_pts)
    assert nearest < 5.0, f"TCP 离中间点最近也有 {nearest:.1f}mm，圆弧没经过中间点"

    # 2) 明显偏离弦线（否则就是一条直线，圆弧白设了）
    chord = end - start
    length = float(np.linalg.norm(chord))
    unit = chord / length
    deviations = []
    for p in arc_pts:
        v = p - start
        proj = min(max(float(np.dot(v, unit)), 0.0), length)
        deviations.append(float(np.linalg.norm(v - proj * unit)))
    assert max(deviations) > 20.0, "路径几乎是弦线，圆弧退化成了直线"
    assert max(deviations) < length, "偏离量比弦长还大，说明取到了别的段"


# --------------------------------------------------------------------------- #
# 降级：正解失败时如实说明，且**不吞掉中间点**
# --------------------------------------------------------------------------- #


def _break_fk_for(panel, joints_deg, times=None):
    """把某个示教点的正解打坏（模拟位形奇异等）。

    Args:
        times: ``None`` 表示一直失败；给整数则只在最初 N 次命中该位形时失败。

    Note:
        ``validate_program`` 是"构建 → 校验 → 规划"共用同一个机器人对象，
        规划器也要靠 FK 填每个路径点的位姿——永久失败会让规划器也报错，那是
        另一回事，不该混进"构建期降级"的验收里。
    """
    real = panel._robot.forward_kinematics
    bad = [float(v) for v in joints_deg]
    state = {"left": times}

    def flaky(joints, *args, **kwargs):
        hit = len(joints) == len(bad) and all(
            abs(float(x) - y) < 1e-9 for x, y in zip(joints, bad)
        )
        if hit and (state["left"] is None or state["left"] > 0):
            if state["left"] is not None:
                state["left"] -= 1
            raise ValueError("simulated FK failure")
        return real(joints, *args, **kwargs)

    panel._robot.forward_kinematics = flaky


def test_arc_degradation_discloses_via_and_keeps_it(panel):
    """圆弧降级不能让中间点跟着消失——示教了 4 个点就得走 4 个点。"""
    panel.set_point_motion(2, ARC)
    _break_fk_for(panel, P_VIA)

    program = panel.build_program()
    assert program is not None
    types = [m.type for m in program.motions]
    assert MotionType.CIRCULAR not in types
    assert len(program.motions) == 4, "中间点被吞掉了"

    notes = panel.last_build_notes()
    assert len(notes) == 1
    assert "P3" in notes[0] and "P2" in notes[0]
    assert "moveJ" in notes[0]


def test_arc_degradation_surfaces_in_status(panel, recorder):
    panel.set_point_motion(2, ARC)
    _break_fk_for(panel, P_VIA, times=1)  # 构建那一刻的正解失败
    recorder.clear()
    report = panel.validate_program()
    assert report is not None
    assert "降级" in recorder[-1]
    assert any(m.startswith("⚠") for m in recorder)
    # 降级了也得能跑：程序 4 条关节运动、校验通过（示教点一个不少）
    assert report.motion_labels == ["moveJ"] * 4
    assert report.ok
    assert report.cycle_time is not None


# --------------------------------------------------------------------------- #
# 校验着色：指令序号 → 路点行号的反向映射
# --------------------------------------------------------------------------- #


def test_arc_coloring_lands_on_the_arc_end_row(panel):
    """指向圆弧指令的问题，着色必须落在圆弧终点那一行（不是中间点、更不是下一行）。"""
    panel.set_point_motion(2, ARC)
    report = ProgramReport(
        issues=[
            Issue(
                severity=SEVERITY_ERROR,
                code="simulated",
                motion_index=1,  # 第 2 条指令＝圆弧
                message="boom",
            )
        ]
    )
    panel._report = report
    panel._apply_point_colors(report)

    assert panel._point_issue_level == {2: "error"}
    assert panel.point_list.item(2).foreground().color().name() == "#ff8080"
    assert panel.point_list.item(1).foreground().color().name() != "#ff8080"


def test_arc_edit_invalidates_previous_results(panel):
    assert panel.validate_program() is not None
    assert panel.trajectory is not None
    panel.set_point_motion(2, ARC)
    assert panel.trajectory is None
    assert panel.btn_play_all.isEnabled() is False


# --------------------------------------------------------------------------- #
# 品牌出码：圆弧必须落到各家的原生圆弧指令上
# --------------------------------------------------------------------------- #


@pytest.mark.parametrize(
    "brand, marker",
    [
        ("ur_polyscope", "movec("),
        ("kuka_krc4", "CIRC"),
        ("abb_irc5", "MoveC"),
    ],
)
def test_arc_brand_output(panel, brand, marker):
    panel.set_point_motion(2, ARC)
    program = panel.build_program()
    assert program is not None
    code = program.to_post(brand, metadata={"program_name": "arc_demo"})
    assert marker in code
    assert "movec(" in code or "CIRC" in code or "MoveC" in code


def test_arc_export_writes_arc_code(panel, tmp_path):
    """一键导出走真实出码通路，落盘内容里圆弧指令在场。"""
    import orolp.post as post_module

    post = post_module.get_post_processor("ur_polyscope")
    panel.set_point_motion(2, ARC)
    program = panel.build_program()
    assert panel.export_program_to(program, str(tmp_path)) is True
    files = [f for f in tmp_path.rglob("*.script")]
    assert files, "导出目录里没有 .script"
    assert "movec(" in files[0].read_text()
    assert post.file_extension == "script"


# --------------------------------------------------------------------------- #
# i18n：圆弧相关文案在英文界面下不留中文
# --------------------------------------------------------------------------- #


def test_arc_messages_are_english_in_en_mode(panel):
    set_language("en")
    try:
        for index in (0, 1):
            reason = panel.arc_rejection(index)
            assert reason is not None
            assert not any("\u4e00" <= ch <= "\u9fff" for ch in reason), reason
        assert panel.set_point_motion(2, ARC) is True
        # 中间点行标签也走目录
        assert panel.point_list.item(1).text().split(" · ")[1] == "moveC via"
        localized = panel._point_motions
        assert localized == ["J", "J", "C", "J"]
    finally:
        set_language("zh")
