"""离线编程面板：程序校验、节拍估算、轨迹回放。

面板本身不含业务逻辑，只负责把 :class:`~service.planner.TrajectoryPlanner`
的结果呈现出来：

* **估算节拍** —— OLP 的核心卖点。客户拿它报价、算产线产能
* **问题清单** —— 不可达 / 越限 / 奇异 / 翻腕 / 干涉，带指令定位
* **真实插补回放** —— 按插补后的密集轨迹播放，而不是在示教点之间瞬移。
  这两者的区别正是tr("直线插补")与"关节乱跳"

单位：面板内部一律 **度**（与 :class:`~service.planner` 一致）；向视口发送
回放帧时换算为 **弧度**（``GLWidget.renderer.joint_values`` 用弧度）。

**结构（v2 类内拆解）**：本类只留信号、构造、多机状态槽代理与各方法的薄 stub；
实现按职责拆到同目录 ``program_state`` / ``program_ui`` / ``program_robot`` /
``program_playback`` / ``program_grasp`` / ``program_export`` / ``program_editor`` /
``program_validate`` 八个模块，每个被拆出的方法是目标模块里以 ``panel`` 为
第一参数的模块级函数。
"""

from __future__ import annotations

from contextlib import contextmanager
from typing import TYPE_CHECKING, Dict, List, Optional, Sequence, Tuple

from PySide6 import QtCore, QtWidgets

from app.features import enabled as _feature_enabled  # noqa: F401 —— 测试补丁打在本模块此名上
from app.views import program_editor as _program_editor
from app.views import program_export as _program_export
from app.views import program_grasp as _program_grasp
from app.views import program_playback as _program_playback
from app.views import program_robot as _program_robot
from app.views import program_ui as _program_ui
from app.views import program_validate as _program_validate
from app.views.program_state import PLAYBACK_INTERVAL_MS, RobotProgram
from app.views.program_ui import PANEL_STYLE

if TYPE_CHECKING:  # 仅类型标注
    from domain.robot import Robot
    from domain.world.scene import Scene
    from service.pickplace import PointType
    from service.planner import ProgramReport
    from service.program import Program
    from service.simulation import Trajectory

__all__ = ["ProgramPanel", "PANEL_STYLE"]


class ProgramPanel(QtWidgets.QWidget):
    """离线编程面板。

    Signals:
        statusMessage(str): 适合放进状态栏的提示
        playbackFrame(str, dict): 回放帧，``(机器人名, {关节名: 弧度})``。
            机器人名不能省——多机时帧必须落在对应那台身上
        playbackFinished(): 回放结束
        gripperStateChanged(str, str): 夹爪状态变化，
            ``(机器人名, "close"/"open"/"")``；空串表示回放复位、未夹持，
            供主窗口把工件吸附/释放到**对应那台**机器人上
        reportReady(object): :class:`ProgramReport`，供其他面板复用
        allValidated(dict): 一键校验全部完成，
            ``{机器人名: ProgramReport 或 None}``；``None`` 表示那台条件不足
            （没型号/没示教点）或校验本身出错。供主窗口出汇总
    """

    statusMessage = QtCore.Signal(str)
    #: 回放帧：``(机器人名, {关节名: 弧度})``。机器人名不能省——多机时否则
    #: 帧会被套到"当前"那台身上，两台一起抖
    playbackFrame = QtCore.Signal(str, dict)
    playbackFrameDTO = QtCore.Signal(object)
    playbackFinished = QtCore.Signal()
    #: 夹爪状态变化：``(机器人名, "close"/"open"/"")``
    gripperStateChanged = QtCore.Signal(str, str)
    reportReady = QtCore.Signal(object)
    #: 轨迹编辑器改动示教点后广播：``(positions, types, motions)``
    #: positions 为 ``{关节名: 弧度}`` 字典列表；types 为工艺语义（可为 ``None``）；
    #: motions 为与点位对齐的运动类型（``"J"``/``"L"``）。主窗口据此把改动
    #: 回写到路径代码编辑器，保持两条编辑通路数据一致
    taughtPointsChanged = QtCore.Signal(list, list, list)
    #: 面板顶部的「机器人」下拉换了人。主窗口据此把当前机器人、示教点显示
    #: 同步到被选中的那台——否则"编辑的是 A、示范的是 B"
    activeRobotChanged = QtCore.Signal(str)
    #: **要点：校验/回放前请主窗口按这个名字回灌一次最新示教点**。
    #: 面板自己的数据槽是上次同步时的快照——比如先在 A 名下教点、再把下拉切到
    #: B，B 那一槽可能还没灌过点；不先回灌就点校验，等于拿空数据校了个寂寞
    #: （按钮看着像没反应）。主窗口持有最新的点表，由它回灌最可靠。
    syncRequested = QtCore.Signal(str)
    #: 一键校验全部完成：``{机器人名: ProgramReport 或 None}``
    allValidated = QtCore.Signal(dict)
    #: 抓取向导写完示教点：**带机器人名**。主窗口据此把点位回灌视口——面板
    #: 里改了点，视口的路径可视化却还画着旧的那串，用户会以为向导没生效。
    #: 不复用 :attr:`taughtPointsChanged`：那个不带机器人名，多机时会把这台
    #: 的点写进另一台的视口桶
    graspCycleApplied = QtCore.Signal(str)

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.setStyleSheet(PANEL_STYLE)

        #: 工作单元场景——所有机器人共用同一台 Scene（干涉检查看的是一个工位）
        self._scene: Optional["Scene"] = None
        #: 每台机器人各一份私有状态，键为机器人名
        self._states: Dict[str, RobotProgram] = {}
        #: 当前正在编辑/回放哪一台
        self._active: Optional[str] = None
        #: 还没登记任何机器人时的临时槽，让读路径不必处处判空
        self._scratch: RobotProgram = RobotProgram()
        #: 主窗口注入的"视口里选中了哪台"探针（``() -> str | None``）。
        #: 「生成代码」按回放同一套口径：选中设备 → 出那台的；没选 → 出全部。
        self.selection_robot_provider: Optional[callable] = None

        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(PLAYBACK_INTERVAL_MS)
        self._timer.timeout.connect(self._on_tick)
        #: 正在回放的机器人名；空列表表示没在播（多机锁步时会有多个）
        self._playing: List[str] = []

        self._build_ui()

    # ---------------- 状态槽代理 ---------------- #
    # 既有的一千行逻辑（build_program / validate_program / 轨迹编辑 / 回放）
    # 都是按"当前机器人只有一份数据"写的。把它们逐个改成查字典要动上千行，
    # 风险远大于收益——这里用同名属性代理到当前机器人的 :class:`RobotProgram`
    # 槽：读写形态一字不改，数据却按机器人各自隔开。

    def _state(self, name: Optional[str] = None) -> RobotProgram:
        """取 ``name`` 的状态槽；``None`` 取当前机器人。"""
        key = name if name is not None else self._active
        if key is not None and key in self._states:
            return self._states[key]
        return self._scratch

    @contextmanager
    def _using_robot(self, name: Optional[str]):
        """临时切到 ``name`` 的状态槽（``None``/未登记则不动）。

        批量处理多台机器人（出码/导出全部）时要逐个取它们的程序，但不能把
        用户正在编辑的那台换掉——于是 Borrow-and-Restore。
        """
        prev = self._active
        if name and name in self._states and name != prev:
            self._active = name
            try:
                yield self._states[name]
            finally:
                self._active = prev
        else:
            yield self._state(name)

    def _ensure_state(self, name: str) -> RobotProgram:
        """确保 ``name`` 有状态槽并返回它（多机：每台按需建立）。"""
        if name not in self._states:
            self._states[name] = RobotProgram()
        return self._states[name]

    @property
    def _robot(self):
        return self._state().robot

    @_robot.setter
    def _robot(self, v):
        self._state().robot = v

    @property
    def _joint_names(self):
        return self._state().joint_names

    @_joint_names.setter
    def _joint_names(self, v):
        self._state().joint_names = v

    @property
    def _taught(self):
        return self._state().taught

    @_taught.setter
    def _taught(self, v):
        self._state().taught = v

    @property
    def _point_types(self):
        return self._state().point_types

    @_point_types.setter
    def _point_types(self, v):
        self._state().point_types = v

    @property
    def _point_motions(self):
        return self._state().point_motions

    @_point_motions.setter
    def _point_motions(self, v):
        self._state().point_motions = v

    @property
    def _build_notes(self):
        return self._state().build_notes

    @_build_notes.setter
    def _build_notes(self, v):
        self._state().build_notes = v

    @property
    def _trajectory(self):
        return self._state().trajectory

    @_trajectory.setter
    def _trajectory(self, v):
        self._state().trajectory = v

    @property
    def _report(self):
        return self._state().report

    @_report.setter
    def _report(self, v):
        self._state().report = v

    @property
    def _planner(self):
        return self._state().planner

    @_planner.setter
    def _planner(self, v):
        self._state().planner = v

    @property
    def _gripper_animator(self):
        return self._state().gripper_animator

    @_gripper_animator.setter
    def _gripper_animator(self, v):
        self._state().gripper_animator = v

    @property
    def _last_gripper_state(self):
        return self._state().last_gripper_state

    @_last_gripper_state.setter
    def _last_gripper_state(self, v):
        self._state().last_gripper_state = v

    @property
    def _point_issue_level(self):
        return self._state().point_issue_level

    @_point_issue_level.setter
    def _point_issue_level(self, v):
        self._state().point_issue_level = v

    @property
    def _last_point_count(self):
        return self._state().last_point_count

    @_last_point_count.setter
    def _last_point_count(self, v):
        self._state().last_point_count = v

    @property
    def _last_code(self):
        return self._state().last_code

    @_last_code.setter
    def _last_code(self, v):
        self._state().last_code = v

    @property
    def _playhead(self):
        return self._state().playhead

    @_playhead.setter
    def _playhead(self, v):
        self._state().playhead = v

    # ---------------------------- 外部注入 ---------------------------- #

    def set_robot(
        self,
        robot: Optional["Robot"],
        joint_names: Optional[Sequence[str]] = None,
        name: Optional[str] = None,
    ) -> None:
        return _program_robot.set_robot(self, robot, joint_names, name)

    # ---------------------------- 多机器人 ---------------------------- #

    @property
    def active_robot(self) -> Optional[str]:
        return _program_robot.active_robot(self)

    @property
    def robot_names(self) -> List[str]:
        return _program_robot.robot_names(self)

    def set_active_robot(self, name: Optional[str]) -> bool:
        return _program_robot.set_active_robot(self, name)

    def remove_robot(self, name: Optional[str]) -> None:
        return _program_robot.remove_robot(self, name)

    def _on_robot_combo_changed(self, _index: int) -> None:
        return _program_robot._on_robot_combo_changed(self, _index)

    def set_scene(self, scene: Optional["Scene"]) -> None:
        return _program_robot.set_scene(self, scene)

    # ---------------------------- 轨迹编辑 ---------------------------- #

    def set_taught_points(
        self,
        positions: Sequence[Dict[str, float]],
        types: Optional[Sequence["PointType"]] = None,
        motions: Optional[Sequence[str]] = None,
        robot: Optional[str] = None,
    ) -> None:
        return _program_editor.set_taught_points(self, positions, types, motions, robot)

    def get_taught_points(self, robot: Optional[str] = None) -> tuple:
        return _program_editor.get_taught_points(self, robot)

    def _selected_index(self) -> Optional[int]:
        return _program_editor._selected_index(self)

    def _motion_kind(self, index: int) -> str:
        return _program_editor._motion_kind(self, index)

    def _gripper_action_at(self, index: int) -> Optional[str]:
        return _program_editor._gripper_action_at(self, index)

    def arc_rejection(self, index: int) -> Optional[str]:
        return _program_editor.arc_rejection(self, index)

    def motion_plan(self) -> Tuple[List[Tuple[str, int, Optional[int]]], List[str]]:
        return _program_editor.motion_plan(self)

    def arc_via_indices(self) -> set:
        return _program_editor.arc_via_indices(self)

    def instruction_of_point(self) -> Dict[int, int]:
        return _program_editor.instruction_of_point(self)

    def last_build_notes(self) -> List[str]:
        return _program_editor.last_build_notes(self)

    def _emit_taught_changed(self) -> None:
        return _program_editor._emit_taught_changed(self)

    def _invalidate_results(self) -> None:
        return _program_editor._invalidate_results(self)

    def _keep_row_selected(self, row: int) -> None:
        return _program_editor._keep_row_selected(self, row)

    def insert_point(self, index: Optional[int] = None) -> Optional[int]:
        return _program_editor.insert_point(self, index)

    def delete_point(self, index: Optional[int] = None) -> bool:
        return _program_editor.delete_point(self, index)

    def move_point(self, src: int, dst: int) -> bool:
        return _program_editor.move_point(self, src, dst)

    def shift_selected_point(self, delta: int) -> bool:
        return _program_editor.shift_selected_point(self, delta)

    def set_point_motion(self, index: int, motion: str) -> bool:
        return _program_editor.set_point_motion(self, index, motion)

    def update_point(
        self,
        index: int,
        joints_deg: Sequence[float],
        motion: Optional[str] = None,
    ) -> bool:
        return _program_editor.update_point(self, index, joints_deg, motion)

    def edit_selected_point(self) -> None:
        return _program_editor.edit_selected_point(self)

    def _refresh_point_list(self) -> None:
        return _program_editor._refresh_point_list(self)

    def _apply_point_colors(self, report: ProgramReport) -> None:
        return _program_editor._apply_point_colors(self, report)

    # ---------------------------- 业务逻辑 ---------------------------- #

    @property
    def trajectory(self) -> Optional["Trajectory"]:
        """最近一次展开的轨迹。"""
        return self._trajectory

    @property
    def report(self) -> Optional[ProgramReport]:
        """最近一次校验报告。"""
        return self._report

    def build_program(self, robot: Optional[str] = None) -> Optional[Program]:
        return _program_validate.build_program(self, robot)

    def _build_program_impl(self) -> Optional[Program]:
        return _program_validate._build_program_impl(self)

    def _scene_joint_snapshot(self, name: Optional[str]):
        return _program_validate._scene_joint_snapshot(self, name)

    def _restore_scene_joints(self, snapshot) -> None:
        return _program_validate._restore_scene_joints(self, snapshot)

    def validate_program(self) -> Optional[ProgramReport]:
        return _program_validate.validate_program(self)

    def validate_all(self) -> Dict[str, Optional[ProgramReport]]:
        return _program_validate.validate_all(self)

    def focus_report(self, name: Optional[str] = None) -> bool:
        return _program_validate.focus_report(self, name)

    def _point_levels(self, report: ProgramReport) -> Dict[int, str]:
        return _program_validate._point_levels(self, report)

    # ---------------------------- 设备联调 ---------------------------- #

    def generate_code(self) -> None:
        return _program_export.generate_code(self)

    def _generate_active_code(self) -> None:
        return _program_export._generate_active_code(self)

    def export_all_programs(self) -> None:
        return _program_export.export_all_programs(self)

    def export_program_to(
        self, program: Program, dest: str, robot: Optional[str] = None
    ) -> bool:
        return _program_export.export_program_to(self, program, dest, robot)

    def _export_impl(self, program: Program, dest: str) -> bool:
        return _program_export._export_impl(self, program, dest)

    def robot_snapshot(self) -> Dict[str, dict]:
        return _program_export.robot_snapshot(self)

    def restore_robots(self, data: Optional[dict]) -> None:
        return _program_export.restore_robots(self, data)

    def generate_all_code(self) -> Dict[str, str]:
        return _program_export.generate_all_code(self)

    # ---------------------------- 回放 ---------------------------- #

    def start_playback(self, robots: Optional[Sequence[str]] = None) -> None:
        return _program_playback.start_playback(self, robots)

    def start_playback_all(self) -> None:
        return _program_playback.start_playback_all(self)

    def _playable(self, name: Optional[str] = None) -> List[str]:
        return _program_playback._playable(self, name)

    def stop_playback(self) -> None:
        return _program_playback.stop_playback(self)

    def _on_tick(self) -> None:
        return _program_playback._on_tick(self)

    def _tick_one(self, name: str) -> bool:
        return _program_playback._tick_one(self, name)

    def _emit_one(self, name: str, joints_deg: Sequence[float]) -> None:
        return _program_playback._emit_one(self, name, joints_deg)

    def _emit_frame(self, joints_deg: Sequence[float]) -> None:
        return _program_playback._emit_frame(self, joints_deg)

    def _set_gripper_for(self, name: str, action: str) -> None:
        return _program_playback._set_gripper_for(self, name, action)

    def _set_gripper_state(self, action: str) -> None:
        return _program_playback._set_gripper_state(self, action)

    # ---------------------------- 抓取向导 ---------------------------- #

    def grasp_cycle_points(
        self,
        pick_index: int,
        place_index: Optional[int] = None,
        approach_mm: float = 50.0,
        lift_mm: float = 50.0,
        retract_mm: float = 50.0,
        robot: Optional[str] = None,
    ) -> Tuple[List[Dict[str, float]], List[PointType], List[str]]:
        return _program_grasp.grasp_cycle_points(
            self, pick_index, place_index, approach_mm, lift_mm, retract_mm, robot
        )

    def apply_grasp_cycle(
        self,
        pick_index: int,
        place_index: Optional[int] = None,
        approach_mm: float = 50.0,
        lift_mm: float = 50.0,
        retract_mm: float = 50.0,
        replace: bool = False,
        robot: Optional[str] = None,
    ) -> int:
        return _program_grasp.apply_grasp_cycle(
            self, pick_index, place_index, approach_mm, lift_mm, retract_mm, replace, robot
        )

    def open_grasp_wizard(self, robot: Optional[str] = None) -> bool:
        return _program_grasp.open_grasp_wizard(self, robot)

    # ---------------------------- 界面 ---------------------------- #

    def _build_ui(self) -> None:
        return _program_ui._build_ui(self)

    def _show_report(self, report: ProgramReport) -> None:
        return _program_ui._show_report(self, report)

    def _clear_metrics(self) -> None:
        return _program_ui._clear_metrics(self)

    def _display_active(self) -> None:
        return _program_ui._display_active(self)

    def _refresh_robot_combo(self, selected: Optional[str] = None) -> None:
        return _program_ui._refresh_robot_combo(self, selected)

    def _refresh_export_all_enabled(self) -> None:
        return _program_ui._refresh_export_all_enabled(self)
