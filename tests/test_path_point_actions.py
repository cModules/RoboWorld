"""路径点位 ↔ 机器人动作的关联（v1.4.2）。

两条从"看得见的点"到"机器人真的走过去"的通路：

* 3D 视口里右键某个红色路径点 → 「移动到」→ 那台机器人走到该示教点；
* 「路径示教」代码区右键某一行 → 「执行此行」→ 机器人走该行对应的点位。

它们共同的前提是**认点不认"当前那台"**：多机时点属于谁由画出来的那一段
（或代码行归属）决定，退回当前机器人就会把 A 的点驱动到 B 身上。

跑法::

    QT_QPA_PLATFORM=offscreen pytest tests/test_path_point_actions.py -q
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest
from PySide6 import QtCore, QtWidgets

REPO_ROOT = Path(__file__).resolve().parents[1]
for _p in (REPO_ROOT, REPO_ROOT / "python"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

UR5E = REPO_ROOT / "models" / "ur" / "ur5e" / "ur5e.urdf"


@pytest.fixture(scope="module")
def qapp():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture()
def window(qapp):
    import app as rw

    win = rw.MainWindow()
    yield win
    win.close()
    win.deleteLater()


@pytest.fixture()
def two(window):
    """场景里两台 UR5E（相距 2 m），各教 3 / 2 个点。"""
    if not UR5E.exists():
        pytest.skip(f"缺少测试用 URDF: {UR5E}")
    from orolp.pose import Pose

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

    for name, n, step in ((a, 3, 10.0), (b, 2, -12.0)):
        window.gl_widget.set_current_robot(name)
        renderer = window.gl_widget.renderer_of(name)
        first = renderer.actuated_names()[0]
        for i in range(n):
            renderer.joint_values[first] = math.radians(i * step)
            window.gl_widget.record_position(name)
        window._sync_scene_joints(name)
    return window, scene, a, b


def _first_deg(window, robot):
    renderer = window.gl_widget.renderer_of(robot)
    return round(math.degrees(renderer.joint_values[renderer.actuated_names()[0]]), 2)


# --------------------------------------------------------------------------- #
# 1) 视口：右键路径点 → 移动到
# --------------------------------------------------------------------------- #


def test_path_point_hit_returns_its_owner(two):
    """投影到某个路径点上，命中的必须是**那一台**的**那一个**点。"""
    window, scene, a, b = two
    gl = window.gl_widget

    ee = gl._recorded_ee_by_robot[b][1]          # B 的 P2
    sx, sy = gl._project(ee)
    assert gl.path_point_at(sx, sy) == (b, 1), "点错台或点错序号"

    # 空白处不该命中（否则右键随便一点都会弹菜单）
    assert gl.path_point_at(3, 3) is None


def test_move_to_path_point_drives_only_that_robot(two):
    """「移动到」只动被点的那台，另一台原地不动。"""
    window, scene, a, b = two

    before_a = _first_deg(window, a)
    window.on_path_point_move_requested(b, 1)    # B → P2（-12°）

    assert _first_deg(window, b) == pytest.approx(-12.0, abs=0.01)
    assert _first_deg(window, a) == pytest.approx(before_a, abs=0.01), (
        "另一台被带着动了"
    )
    # 场景关节也要跟着：否则法兰不动，挂在法兰上的夹爪不跟随
    assert round(scene.robots[b].joints_deg[0], 2) == pytest.approx(-12.0, abs=0.01)


def test_context_menu_offers_move_to_on_path_point(two, monkeypatch):
    """右键点在路径点上要给出「移动到」——哪怕该位置拾取到机器人。

    回归：曾按"对象优先"判断（命中对象就 return），而机器人的拾取体是整台
    手臂的一个大包围盒，走近机器人的示教点几乎总被它挡住，于是菜单**永远
    弹不出来**（2026-09-24 用户反馈）。
    """
    from PySide6 import QtCore, QtGui

    window, scene, a, b = two
    gl = window.gl_widget

    ee = gl._recorded_ee_by_robot[b][0]
    sx, sy = gl._project(ee)
    assert gl._pick_object(int(sx), int(sy)) is not None, (
        "这个用例要先确认该点确实被对象遮挡（否则测不到回归）"
    )
    # 就近可能落在相邻点上（两点投影挨得很近），以实际命中的那个为准
    owner, hit_index = gl.path_point_at(sx, sy) or (b, 0)

    shown = []

    class _Action:
        def setToolTip(self, _t):
            pass

    class FakeMenu:
        """QMenu 替身：离屏下真建菜单会卡住，这里只验证逻辑。"""

        def __init__(self, parent=None):
            self._actions = []

        def addAction(self, text):
            self._actions.append(text)
            return _Action()

        def actions(self):
            return list(self._actions)

        def addSeparator(self):
            pass

        def exec(self, *_a, **_k):
            shown.append(list(self._actions))
            return None

    monkeypatch.setattr(QtWidgets, "QMenu", FakeMenu)

    def fire(moved):
        shown.clear()
        gl._right_moved = moved
        gl.contextMenuEvent(
            QtGui.QContextMenuEvent(
                QtGui.QContextMenuEvent.Mouse, QtCore.QPoint(int(sx), int(sy))
            )
        )
        return shown[0] if shown else None

    # 点一下就松开 → 有菜单，且说清是"哪一台的 P 几"
    assert fire(False) == [f"移动到 {owner} 的 P{hit_index + 1}"], fire(False)
    # 右键**拖过**（gizmo/视角）→ 不该抢手势弹菜单
    assert fire(True) is None

    # 空白处：没有路径点，不该有菜单
    shown.clear()
    gl._right_moved = False
    gl.contextMenuEvent(
        QtGui.QContextMenuEvent(QtGui.QContextMenuEvent.Mouse, QtCore.QPoint(5, 5))
    )
    assert shown == []


def test_move_to_out_of_range_point_is_ignored(two):
    """点序号越界（例如删点后残留的引用）不能崩，也不能乱动。"""
    window, scene, a, b = two
    before = _first_deg(window, b)

    window.on_path_point_move_requested(b, 99)
    window.on_path_point_move_requested("", 0)

    assert _first_deg(window, b) == pytest.approx(before, abs=0.01)


# --------------------------------------------------------------------------- #
# 2) 代码区：右键行 → 执行此行
# --------------------------------------------------------------------------- #


def _code(robot_b: str) -> str:
    return (
        "# OLP Path Code\n"
        f"# Robot: {robot_b}\n"
        "movej([0.00, -90.00, 90.00, 0.00, 90.00, 0.00], vel=0.5)\n"
        "movej([-12.00, -90.00, 90.00, 0.00, 90.00, 0.00], vel=0.5)\n"
        "movej([-24.00, -90.00, 90.00, 0.00, 90.00, 0.00], vel=0.5)\n"
    )


def test_run_line_drives_the_robot_that_owns_it(two):
    """执行某行走的是**该行归属**那台，不是"当前那台"。"""
    window, scene, a, b = two
    ed = window.code_editor

    window.gl_widget.set_current_robot(a)        # 故意把"当前"停在 A
    before_a = _first_deg(window, a)

    ed.setPlainText(_code(b))
    ed._parse_code_to_positions()

    assert [owner for _ln, owner, _i in ed._line_map] == [b, b, b]

    assert ed.run_line(5) is True                # 第 5 行 = B 的 P3（-24°）
    assert _first_deg(window, b) == pytest.approx(-24.0, abs=0.01)
    assert _first_deg(window, a) == pytest.approx(before_a, abs=0.01), (
        "把 B 的行执行到了 A 身上"
    )


def test_run_line_rejects_non_movej(two):
    """注释/空行不给出可执行入口——点了要如实说，不能静默。"""
    window, scene, a, b = two
    ed = window.code_editor
    ed.setPlainText(_code(b))
    ed._parse_code_to_positions()

    assert ed._line_target(1) is None, "注释行不该有对应点位"
    assert ed.run_line(1) is False
    assert ed._line_target(4) is not None, "movej 行要有对应点位"


# --------------------------------------------------------------------------- #
# 3) 手改代码 → 3D 刷新；走到哪一点 → 代码高亮到哪一行
# --------------------------------------------------------------------------- #


def _plain(n_points: int) -> str:
    """没有 Robot 头的单机代码（最朴素的手打场景）。"""
    body = "".join(
        f"movej([{i * 10:.2f}, -90.00, 90.00, 0.00, 90.00, 0.00], vel=0.5)\n"
        for i in range(n_points)
    )
    return "# OLP Path Code\n" + body


def _headed(robot: str, n_points: int) -> str:
    """带 ``# Robot:`` 头的代码——多机时归属必须写在代码里。"""
    return _plain(n_points).replace(
        "# OLP Path Code\n", f"# OLP Path Code\n# Robot: {robot}\n"
    )


def test_editing_code_refreshes_viewport_path(two):
    """手改路径示教代码后，3D 里的路径要说变就变。

    回归：文本改动只置了工程脏标记，3D 路径与面板点表要等下一次 Play 才刷新
    ——改完点不见了/还在旧位置（2026-09-25 用户提出）。
    """
    window, scene, a, b = two
    gl = window.gl_widget
    ed = window.code_editor
    panel = window.program_panel

    # 多机下必须带 Robot 头，否则改完的点会整份归到"当前那台"
    gl.set_current_robot(a)
    ed.setPlainText(_headed(a, 4))
    ed._flush_code_edit()

    assert len(gl.get_recorded_positions(a)) == 4, "视口点表没跟着改"
    assert len(gl._recorded_ee_by_robot[a]) == 4, "末端路径（3D 折线）没重算"
    assert len(panel._state(a).taught) == 4, "离线编程面板的点表没同步"
    assert ed.pos_count_label.text() == "示教点：4"
    # 另一台不该被这次改动波及
    assert len(gl.get_recorded_positions(b)) == 2

    # 再删两行：同样要立刻生效
    ed.setPlainText(_headed(a, 2))
    ed._flush_code_edit()
    assert len(gl.get_recorded_positions(a)) == 2
    assert len(gl._recorded_ee_by_robot[a]) == 2
    assert len(gl.get_recorded_positions(b)) == 2, "改 A 不该动到 B"


def test_moving_to_point_highlights_its_code_line(two):
    """3D 里右键走到某个点，代码就停在那一点对应的行上。"""
    window, scene, a, b = two
    gl = window.gl_widget
    ed = window.code_editor

    ed.setPlainText(_plain(3))
    ed._flush_code_edit()

    window.on_path_point_move_requested(a, 2)      # 走到 P3
    assert _first_deg(window, a) == pytest.approx(20.0, abs=0.01)
    cursor_line = ed.code_edit.textCursor().blockNumber() + 1
    assert cursor_line == 4, f"高亮到了第 {cursor_line} 行，P3 应在第 4 行"


def test_single_step_highlights_current_line(two):
    """回放/单步走到某一点时，代码高亮跟着走（便于对着那一行改）。"""
    window, scene, a, b = two
    ed = window.code_editor

    ed.setPlainText(_plain(3))
    ed._flush_code_edit()

    window.on_playback_requested(0, a)             # 走第 1 个点
    assert ed.code_edit.textCursor().blockNumber() + 1 == 2

    window.on_playback_requested(2, a)             # 走第 3 个点
    assert ed.code_edit.textCursor().blockNumber() + 1 == 4


def test_flush_skipped_while_playing(two):
    """播放中不重解析：否则点表被改写，回放序列与点表错位会让机器人跳点。"""
    window, scene, a, b = two
    ed = window.code_editor

    ed.setPlainText(_plain(3))
    ed._flush_code_edit()
    before = len(ed.recorded_positions)

    timer = QtCore.QTimer()
    timer.setInterval(60_000)
    timer.start()
    ed.playback_timer = timer
    try:
        assert timer.isActive()
        ed.setPlainText(_plain(5))       # 播到一半改文本
        ed._flush_code_edit()
        assert len(ed.recorded_positions) == before, "播放中不该改写点表"
    finally:
        timer.stop()
        ed.playback_timer = None
