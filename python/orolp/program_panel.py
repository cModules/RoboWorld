"""离线编程面板：程序校验、节拍估算、轨迹回放。

面板本身不含业务逻辑，只负责把 :class:`~orolp.planner.TrajectoryPlanner`
的结果呈现出来：

* **估算节拍** —— OLP 的核心卖点。客户拿它报价、算产线产能
* **问题清单** —— 不可达 / 越限 / 奇异 / 翻腕 / 干涉，带指令定位
* **真实插补回放** —— 按插补后的密集轨迹播放，而不是在示教点之间瞬移。
  这两者的区别正是tr("直线插补")与"关节乱跳"

单位：面板内部一律 **度**（与 :class:`~orolp.planner` 一致）；向视口发送
回放帧时换算为 **弧度**（``GLWidget.renderer.joint_values`` 用弧度）。
"""

from __future__ import annotations

import math
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Dict, List, Optional, Sequence, Tuple

from PySide6 import QtCore, QtGui, QtWidgets

from .planner import ProgramReport, SEVERITY_ERROR, TrajectoryPlanner
from .pickplace import (
    PointType,
    default_types_for_count,
    gripper_action_for,
)
from .program import Program, ProgramBuilder
from .pose import Pose
from .kinematics import KinematicsError
from .post import get_post_processor, list_post_processors
from .world.animation import GraspAnimator
from .link import FileLink
from .i18n import tr

if TYPE_CHECKING:  # 仅类型标注
    from .robot import Robot
    from .simulation import Trajectory
    from .world.scene import Scene

__all__ = ["ProgramPanel", "PANEL_STYLE"]

#: 回放帧率对应的定时器间隔 (ms)
PLAYBACK_INTERVAL_MS = 40

PANEL_STYLE = """
QWidget { background-color: #2b2b2b; color: #e0e0e0; font-size: 12px; }
QGroupBox {
    border: 1px solid #555; border-radius: 4px;
    /* margin-top 必须容纳一行标题文字(约16px)，否则标题上半截会被部件顶边裁掉 */
    margin-top: 16px; padding-top: 8px; font-weight: bold;
}
QGroupBox::title {
    subcontrol-origin: margin; subcontrol-position: top left;
    left: 8px; padding: 0 4px;
}
QPushButton {
    background-color: #3c3c3c; border: 1px solid #666;
    border-radius: 3px; padding: 5px 10px;
}
QPushButton:hover { background-color: #4a4a4a; }
QPushButton:pressed { background-color: #555; }
QPushButton:disabled { color: #777; background-color: #333; }
QListWidget { background-color: #1e1e1e; border: 1px solid #555; border-radius: 3px; }
QLabel#metric { color: #7fd3ff; font-weight: bold; }
QLabel#ok { color: #6fdc6f; font-weight: bold; }
QLabel#bad { color: #ff8080; font-weight: bold; }
QSpinBox { background-color: #1e1e1e; border: 1px solid #555; padding: 3px; }
"""


@dataclass
class RobotProgram:
    """一台机器人的**私有**离线编程状态。

    面板的所有既有方法（``build_program`` / ``validate_program`` /
    ``start_playback`` …）写的都是"当前机器人的数据"。以前这些数据直接是
    面板的字段，多机时第二台会把第一台顶掉——现在每台各占一个本类的实例，
    面板按 :attr:`ProgramPanel._active` 取用。
    """

    robot: Optional["Robot"] = None
    joint_names: List[str] = field(default_factory=list)
    taught: List[Dict[str, float]] = field(default_factory=list)
    point_types: Optional[List[PointType]] = None
    point_motions: List[str] = field(default_factory=list)
    build_notes: List[str] = field(default_factory=list)
    trajectory: Optional["Trajectory"] = None
    report: Optional[ProgramReport] = None
    planner: Optional[TrajectoryPlanner] = None
    gripper_animator: Optional[GraspAnimator] = None
    last_gripper_state: str = ""
    point_issue_level: Dict[int, str] = field(default_factory=dict)
    last_point_count: Optional[int] = None
    last_code: str = ""
    playhead: float = 0.0


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
    def _robot(self): return self._state().robot

    @_robot.setter
    def _robot(self, v): self._state().robot = v

    @property
    def _joint_names(self): return self._state().joint_names

    @_joint_names.setter
    def _joint_names(self, v): self._state().joint_names = v

    @property
    def _taught(self): return self._state().taught

    @_taught.setter
    def _taught(self, v): self._state().taught = v

    @property
    def _point_types(self): return self._state().point_types

    @_point_types.setter
    def _point_types(self, v): self._state().point_types = v

    @property
    def _point_motions(self): return self._state().point_motions

    @_point_motions.setter
    def _point_motions(self, v): self._state().point_motions = v

    @property
    def _build_notes(self): return self._state().build_notes

    @_build_notes.setter
    def _build_notes(self, v): self._state().build_notes = v

    @property
    def _trajectory(self): return self._state().trajectory

    @_trajectory.setter
    def _trajectory(self, v): self._state().trajectory = v

    @property
    def _report(self): return self._state().report

    @_report.setter
    def _report(self, v): self._state().report = v

    @property
    def _planner(self): return self._state().planner

    @_planner.setter
    def _planner(self, v): self._state().planner = v

    @property
    def _gripper_animator(self): return self._state().gripper_animator

    @_gripper_animator.setter
    def _gripper_animator(self, v): self._state().gripper_animator = v

    @property
    def _last_gripper_state(self): return self._state().last_gripper_state

    @_last_gripper_state.setter
    def _last_gripper_state(self, v): self._state().last_gripper_state = v

    @property
    def _point_issue_level(self): return self._state().point_issue_level

    @_point_issue_level.setter
    def _point_issue_level(self, v): self._state().point_issue_level = v

    @property
    def _last_point_count(self): return self._state().last_point_count

    @_last_point_count.setter
    def _last_point_count(self, v): self._state().last_point_count = v

    @property
    def _last_code(self): return self._state().last_code

    @_last_code.setter
    def _last_code(self, v): self._state().last_code = v

    @property
    def _playhead(self): return self._state().playhead

    @_playhead.setter
    def _playhead(self, v): self._state().playhead = v

    # ---------------------------- 界面 ---------------------------- #

    def _build_ui(self) -> None:
        # 内容自然高度约 798px，而标签页可视高度随窗口变化（1280×800 时仅 725px）。
        # 不套滚动容器会把底部的「后处理与导出」挤出可视区——与右侧属性面板
        # 曾经踩过的坑同源。
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameStyle(QtWidgets.QFrame.NoFrame)
        inner = QtWidgets.QWidget()
        scroll.setWidget(inner)
        outer.addWidget(scroll)

        root = QtWidgets.QVBoxLayout(inner)
        root.setContentsMargins(6, 6, 6, 6)

        # --- 多机器人选择：出码/校验/回放全部按这里选中的那台走 ---
        robot_row = QtWidgets.QHBoxLayout()
        robot_row.addWidget(QtWidgets.QLabel(tr("机器人")))
        self.combo_robot = QtWidgets.QComboBox()
        self.combo_robot.setSizeAdjustPolicy(
            QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToContents
        )
        self.combo_robot.currentIndexChanged.connect(self._on_robot_combo_changed)
        robot_row.addWidget(self.combo_robot, 1)
        root.addLayout(robot_row)

        # --- 节拍与规模 ---
        box = QtWidgets.QGroupBox(tr("程序概览"))
        form = QtWidgets.QFormLayout(box)
        self.lbl_status = QtWidgets.QLabel(tr("未加载机器人"))
        self.lbl_status.setObjectName("bad")
        form.addRow(tr("状态"), self.lbl_status)
        self.lbl_cycle = QtWidgets.QLabel("—")
        self.lbl_cycle.setObjectName("metric")
        form.addRow(tr("估算节拍"), self.lbl_cycle)
        self.lbl_length = QtWidgets.QLabel("—")
        self.lbl_length.setObjectName("metric")
        form.addRow(tr("TCP 路径"), self.lbl_length)
        self.lbl_points = QtWidgets.QLabel("—")
        form.addRow(tr("路径点"), self.lbl_points)
        root.addWidget(box)

        # --- 操作 ---
        ops = QtWidgets.QGroupBox(tr("操作"))
        ops_layout = QtWidgets.QVBoxLayout(ops)

        speed_row = QtWidgets.QHBoxLayout()
        speed_row.addWidget(QtWidgets.QLabel(tr("进给速度")))
        self.spin_speed = QtWidgets.QSpinBox()
        self.spin_speed.setRange(1, 100)
        self.spin_speed.setValue(100)
        self.spin_speed.setSuffix(" %")
        speed_row.addWidget(self.spin_speed, 1)
        ops_layout.addLayout(speed_row)

        btn_row = QtWidgets.QHBoxLayout()
        self.btn_validate = QtWidgets.QPushButton(tr("校验程序 (F7)"))
        self.btn_validate.clicked.connect(self.validate_program)
        btn_row.addWidget(self.btn_validate)
        self.btn_play = QtWidgets.QPushButton(tr("回放"))
        self.btn_play.clicked.connect(self.start_playback)
        self.btn_play.setEnabled(False)
        btn_row.addWidget(self.btn_play)
        self.btn_stop = QtWidgets.QPushButton(tr("停止"))
        self.btn_stop.clicked.connect(self.stop_playback)
        self.btn_stop.setEnabled(False)
        btn_row.addWidget(self.btn_stop)
        # 多机才有意义：所有机器人按各自时间轴锁步同播（看协同是否打架）
        self.btn_play_all = QtWidgets.QPushButton(tr("全部回放"))
        self.btn_play_all.setToolTip(
            tr("所有已校验的机器人同时回放——检查多机协同是否撞车")
        )
        self.btn_play_all.clicked.connect(self.start_playback_all)
        self.btn_play_all.setEnabled(False)
        self.btn_play_all.setVisible(False)
        btn_row.addWidget(self.btn_play_all)
        self.btn_validate_all = QtWidgets.QPushButton(tr("校验全部"))
        self.btn_validate_all.setToolTip(
            tr("逐台校验所有机器人并汇总结果——不必一台一台切过去按 F7")
        )
        self.btn_validate_all.clicked.connect(lambda: self.validate_all())
        self.btn_validate_all.setVisible(False)
        btn_row.addWidget(self.btn_validate_all)
        # 单机也要有：示教是逐台做的活，跟有几台机器人无关
        self.btn_grasp_wizard = QtWidgets.QPushButton(tr("抓取向导…"))
        self.btn_grasp_wizard.setToolTip(
            tr("由抓取点/放置点自动生成接近、提升、撤离点——不必逐个手动示教")
        )
        self.btn_grasp_wizard.clicked.connect(lambda: self.open_grasp_wizard())
        btn_row.addWidget(self.btn_grasp_wizard)
        ops_layout.addLayout(btn_row)
        root.addWidget(ops)

        # --- 轨迹编辑器（v0.2.4）：路点增删移改 + 运动类型切换 ---
        editor = QtWidgets.QGroupBox(tr("轨迹编辑"))
        editor_layout = QtWidgets.QVBoxLayout(editor)

        self.point_list = QtWidgets.QListWidget()
        self.point_list.setAlternatingRowColors(True)
        self.point_list.setWordWrap(True)
        self.point_list.itemDoubleClicked.connect(
            lambda _item: self.edit_selected_point()
        )
        editor_layout.addWidget(self.point_list, 1)

        edit_btn_row = QtWidgets.QHBoxLayout()
        self.btn_insert_point = QtWidgets.QPushButton(tr("插入"))
        self.btn_insert_point.setToolTip(tr("在选中点之后插入一个路点（复制其关节值）"))
        self.btn_insert_point.clicked.connect(self.insert_point)
        edit_btn_row.addWidget(self.btn_insert_point)
        self.btn_delete_point = QtWidgets.QPushButton(tr("删除"))
        self.btn_delete_point.clicked.connect(self.delete_point)
        edit_btn_row.addWidget(self.btn_delete_point)
        self.btn_edit_point = QtWidgets.QPushButton(tr("编辑位姿…"))
        self.btn_edit_point.setToolTip(tr("修改选中路点的关节角（度）与运动类型（双击同效）"))
        self.btn_edit_point.clicked.connect(self.edit_selected_point)
        edit_btn_row.addWidget(self.btn_edit_point)
        editor_layout.addLayout(edit_btn_row)

        move_btn_row = QtWidgets.QHBoxLayout()
        self.btn_move_up = QtWidgets.QPushButton(tr("上移"))
        self.btn_move_up.clicked.connect(lambda: self.shift_selected_point(-1))
        move_btn_row.addWidget(self.btn_move_up)
        self.btn_move_down = QtWidgets.QPushButton(tr("下移"))
        self.btn_move_down.clicked.connect(lambda: self.shift_selected_point(1))
        move_btn_row.addWidget(self.btn_move_down)
        self.btn_motion_j = QtWidgets.QPushButton(tr("设为 moveJ"))
        self.btn_motion_j.setToolTip(tr("选中点改为关节运动（走平滑关节插值，不保证直线）"))
        self.btn_motion_j.clicked.connect(lambda: self.set_selected_motion("J"))
        move_btn_row.addWidget(self.btn_motion_j)
        self.btn_motion_l = QtWidgets.QPushButton(tr("设为 moveL"))
        self.btn_motion_l.setToolTip(tr("选中点改为直线运动（TCP 沿直线，走 IK 插补）"))
        self.btn_motion_l.clicked.connect(lambda: self.set_selected_motion("L"))
        move_btn_row.addWidget(self.btn_motion_l)
        self.btn_motion_c = QtWidgets.QPushButton(tr("设为 moveC"))
        self.btn_motion_c.setToolTip(
            tr(
                "选中点改为圆弧运动：以上一点为中间点、再上一点为起点画圆弧到达本点。\n"
                "中间点不再单独出指令（它只是一段圆弧的途经点），因此 P1/P2 不能设为圆弧，\n"
                "相邻两行也不能同为圆弧，抓取/放置点不能当中间点。"
            )
        )
        self.btn_motion_c.clicked.connect(lambda: self.set_selected_motion("C"))
        move_btn_row.addWidget(self.btn_motion_c)
        editor_layout.addLayout(move_btn_row)

        hint = QtWidgets.QLabel(
            tr(
                "改动后需重新校验：着色与节拍会随校验结果刷新"
                "（红=错误 / 橙=警告 / 绿=正常）\n"
                "圆弧口径：第 N 行的运动类型描述「到达第 N 点」的运动——"
                "moveC 以上一点为中间点、再上一点为起点；中间点行显示为「moveC 中间点」"
            )
        )
        hint.setWordWrap(True)
        editor_layout.addWidget(hint)
        root.addWidget(editor)

        # --- 问题清单 ---
        issues = QtWidgets.QGroupBox(tr("校验结果"))
        issues_layout = QtWidgets.QVBoxLayout(issues)
        self.issue_list = QtWidgets.QListWidget()
        self.issue_list.setAlternatingRowColors(True)
        self.issue_list.setWordWrap(True)
        issues_layout.addWidget(self.issue_list)
        root.addWidget(issues, 1)

        # --- 设备联调：后处理 / 导出 / 真机（预留） ---
        comm = QtWidgets.QGroupBox(tr("后处理与导出"))
        comm_layout = QtWidgets.QVBoxLayout(comm)

        pp_row = QtWidgets.QHBoxLayout()
        pp_row.addWidget(QtWidgets.QLabel(tr("后处理器")))
        self.combo_post = QtWidgets.QComboBox()
        self.combo_post.addItem(tr("自动（按机器人品牌）"), None)
        for name in list_post_processors():
            pp = get_post_processor(name)
            self.combo_post.addItem(f"{pp.brand.upper()} {pp.controller.upper()}", name)
        pp_row.addWidget(self.combo_post, 1)
        comm_layout.addLayout(pp_row)

        gen_row = QtWidgets.QHBoxLayout()
        self.btn_generate = QtWidgets.QPushButton(tr("生成代码"))
        self.btn_generate.clicked.connect(self.generate_code)
        self.btn_generate.setEnabled(False)
        gen_row.addWidget(self.btn_generate)
        self.btn_export = QtWidgets.QPushButton(tr("一键导出…"))
        self.btn_export.setToolTip(tr("一步构建程序并按品牌导出（UR .script / KUKA .src / ABB .mod / FANUC .tp）"))
        self.btn_export.clicked.connect(self.export_program)
        self.btn_export.setEnabled(False)
        gen_row.addWidget(self.btn_export)
        # 多机才有意义：一次把所有机器人按品牌落成交控制器文件
        self.btn_generate_all = QtWidgets.QPushButton(tr("导出全部…"))
        self.btn_generate_all.setToolTip(
            tr("为每一台机器人各生成一份程序并导出到同一个目录（多机出码）")
        )
        self.btn_generate_all.clicked.connect(self.export_all_programs)
        self.btn_generate_all.setEnabled(False)
        self.btn_generate_all.setVisible(False)
        gen_row.addWidget(self.btn_generate_all)
        comm_layout.addLayout(gen_row)

        self.code_preview = QtWidgets.QTextEdit()
        self.code_preview.setReadOnly(True)
        self.code_preview.setLineWrapMode(QtWidgets.QTextEdit.NoWrap)
        mono = QtGui.QFont("Consolas", 9)
        mono.setStyleHint(QtGui.QFont.Monospace)
        self.code_preview.setFont(mono)
        self.code_preview.setFixedHeight(150)
        self.code_preview.setPlaceholderText(tr("生成的控制器代码将显示在这里"))
        comm_layout.addWidget(self.code_preview)

        future_row = QtWidgets.QHBoxLayout()
        self.btn_upload = QtWidgets.QPushButton(tr("上传真机"))
        self.btn_upload.setEnabled(False)
        self.btn_upload.setToolTip(tr("预留：接入 ControllerLink 后支持直连控制器上传"))
        future_row.addWidget(self.btn_upload)
        self.btn_verify = QtWidgets.QPushButton(tr("真机验证"))
        self.btn_verify.setEnabled(False)
        self.btn_verify.setToolTip(tr("预留：接入 ControllerLink.read_joints 后做真机回读比对"))
        future_row.addWidget(self.btn_verify)
        comm_layout.addLayout(future_row)

        root.addWidget(comm)

        self.btn_validate.setEnabled(False)

    # ---------------------------- 外部注入 ---------------------------- #

    def set_robot(
        self,
        robot: Optional["Robot"],
        joint_names: Optional[Sequence[str]] = None,
        name: Optional[str] = None,
    ) -> None:
        """登记一台机器人：**按名字各自留存**，多机不再互相顶掉。

        Args:
            robot: 机器人实例；``None`` 表示卸载 ``name``（或当前这台）
            joint_names: 示教点字典里用的关节名（视口渲染器的命名，可能比
                可驱动关节多——URDF 里的 ``fixed`` 关节，如 ``tcp_joint``，
                也会被渲染器登记）。为 ``None`` 时取 ``robot.joint_names``
            name: 场景里的机器人名。缺省取 ``robot.name``。登记后自动切到它，
                这样示教/校验/出码/回放都作用在刚加进来的那台上

        Note:
            这里必须把关节名过滤到**可驱动**的那些。渲染器会把 fixed 关节
            一并登记，照单全收的话 ``moveJ`` 会收到多出来的分量，直接被
            自由度校验拒掉——表现为"每条指令都失败"，极难排查。
        """
        if robot is None:
            self.remove_robot(name or self._active)
            return

        key = name or robot.name
        # 必须先把「当前」指过去：下面的赋值都经由代理属性落到这一台的槽里
        self._active = key
        state = self._ensure_state(key)

        self.stop_playback()
        state.trajectory = None
        state.report = None
        state.point_issue_level = {}
        state.build_notes = []
        self.issue_list.clear()

        source = list(joint_names or robot.joint_names)
        actuated = list(robot.joint_names)
        known = set(actuated)
        filtered = [n for n in source if n in known]
        # 命名对不上时宁可退回权威列表，也不要留下长度不匹配的隐患
        state.joint_names = filtered if len(filtered) == len(actuated) else actuated
        self._robot = robot
        self.btn_generate.setEnabled(True)
        # 「一键导出」的门槛与「生成代码」完全相同——它内部自己构建程序
        # （见 export_program 的文档），不该逼用户先点一次「生成代码」才亮。
        self.btn_export.setEnabled(True)
        self.lbl_status.setText(tr("{name} · 就绪").format(name=key))
        self.lbl_status.setObjectName("ok")
        # 重新套用样式表，objectName 改变后需手动刷新
        self.lbl_status.style().polish(self.lbl_status)
        self._refresh_robot_combo(key)

    # ---------------------------- 多机器人 ---------------------------- #

    @property
    def active_robot(self) -> Optional[str]:
        """当前正在编辑/回放的机器人名。"""
        return self._active

    @property
    def robot_names(self) -> List[str]:
        """已登记的机器人名（插入顺序）。"""
        return list(self._states)

    def set_active_robot(self, name: Optional[str]) -> bool:
        """切换当前编辑的机器人，界面随之回填**它自己**的数据。

        Returns:
            切换成功 ``True``；名字未登记返回 ``False``
        """
        if not name or name not in self._states:
            return False
        if name != self._active:
            self.stop_playback()
        self._active = name
        self._refresh_robot_combo(name)
        self._display_active()
        return True

    def remove_robot(self, name: Optional[str]) -> None:
        """注销一台机器人（场景里删掉它时调用），遗留状态一并清理。"""
        if not name or name not in self._states:
            return
        if name == self._active:
            self.stop_playback()
        self._states.pop(name, None)
        self._refresh_robot_combo()
        if name == self._active:
            self._active = next(iter(self._states), None)
            self._display_active()

    def _display_active(self) -> None:
        """按当前机器人的状态回填整个界面（点表 / 报告 / 代码 / 按钮可用性）。"""
        state = self._state()
        self._refresh_point_list()
        self.code_preview.setPlainText(state.last_code)

        if state.report is None:
            self.issue_list.clear()
            self._clear_metrics()
        else:
            self._show_report(state.report)
            self._apply_point_colors(state.report)

        self.btn_validate.setEnabled(state.robot is not None and bool(state.taught))
        self.btn_play.setEnabled(
            bool(state.trajectory and len(state.trajectory.waypoints) > 1)
        )
        # 「全部回放」只要有一台能播就该能点
        if getattr(self, "btn_play_all", None) is not None:
            self.btn_play_all.setEnabled(bool(self._playable()))
        self.btn_generate.setEnabled(state.robot is not None)
        self.btn_export.setEnabled(state.robot is not None)
        # 「导出全部」看的是全局：任何一台有程序就该能点（它会跳过没示教的）
        if getattr(self, "btn_generate_all", None) is not None:
            self.btn_generate_all.setEnabled(
                any(s.robot is not None and s.taught for s in self._states.values())
            )

        if state.robot is None:
            self.lbl_status.setText(tr("未加载机器人"))
            self.lbl_status.setObjectName("bad")
        else:
            self.lbl_status.setText(tr("{name} · 就绪").format(name=self._active))
            self.lbl_status.setObjectName("ok")
        self.lbl_status.style().polish(self.lbl_status)

    def _refresh_robot_combo(self, selected: Optional[str] = None) -> None:
        """把已登记的机器人名填进下拉，并选中 ``selected``（或当前这台）。

        重建时必须屏蔽信号：填项过程会让 ``currentIndex`` 跳来跳去，触发
        ``_on_robot_combo_changed`` 会在回填途中反复切换机器人。
        """
        if getattr(self, "combo_robot", None) is None:
            return
        target = selected or self._active
        self.combo_robot.blockSignals(True)
        self.combo_robot.clear()
        for name in self._states:
            self.combo_robot.addItem(name, name)
        if target is not None:
            idx = self.combo_robot.findData(target)
            if idx >= 0:
                self.combo_robot.setCurrentIndex(idx)
        self.combo_robot.blockSignals(False)
        # 只有一台时下拉没什么可挑，藏起来保持单机界面的样子；「导出全部」
        # 同理——单机没必要多一个按钮
        multi = len(self._states) > 1
        self.combo_robot.setVisible(multi)
        self.btn_generate_all.setVisible(multi)
        self.btn_play_all.setVisible(multi)
        # 「校验全部」同理：单机时按一次 F7 就够，不必多一个按钮
        if getattr(self, "btn_validate_all", None) is not None:
            self.btn_validate_all.setVisible(multi)
            self.btn_validate_all.setEnabled(len(self._states) > 0)
        if getattr(self, "btn_grasp_wizard", None) is not None:
            self.btn_grasp_wizard.setEnabled(bool(self._states))

    def _on_robot_combo_changed(self, _index: int) -> None:
        """下拉换人：切到对应机器人，并广播给主窗口同步示教点/视口。"""
        name = self.combo_robot.currentData()
        if not name or name == self._active:
            return
        self.set_active_robot(name)
        self.activeRobotChanged.emit(name)

    def set_scene(self, scene: Optional["Scene"]) -> None:
        """绑定工作单元场景；给出后校验会包含沿路径干涉检查。"""
        self._scene = scene

    def set_taught_points(
        self,
        positions: Sequence[Dict[str, float]],
        types: Optional[Sequence[PointType]] = None,
        motions: Optional[Sequence[str]] = None,
        robot: Optional[str] = None,
    ) -> None:
        """导入示教点（``{关节名: 弧度}``）。

        Args:
            positions: 示教点序列
            types: 与示教点对齐的工艺语义（``PointType.PICK/PLACE…``）；
                给出后 ``build_program`` 会在抓取/放置点自动插入
                ``gripper()`` 指令，程序与动画共用同一份抓取定义
            motions: 与示教点对齐的运动类型（``"J"``/``"L"``/``"C"``）。
                为 ``None`` 且数量不变时沿用当前逐点运动类型（轨迹编辑器
                的改动不被外部同步覆盖），否则重置为全 moveJ
            robot: 写到哪台机器人的名下；``None`` 表示当前这台。给非当前机器人
                喂数据时界面不会跟着跳（主窗口批量同步多台时要这个语义）
        """
        # 喂给非当前那台时临时切过去写，写完切换回来——界面不能跟着跳
        hidden = robot is not None and robot in self._states and robot != self._active
        prev = self._active
        if hidden:
            self._active = robot
        try:
            self._taught = [dict(p) for p in positions]
            self._point_types = list(types) if types is not None else None
            # 长度必须对齐：语义少了会漏动作，多了会错位
            if self._point_types is not None and len(self._point_types) != len(self._taught):
                self._point_types = None
            if motions is not None and len(motions) == len(self._taught):
                self._point_motions = [
                    k if k in ("L", "C") else "J"
                    for k in (str(m).upper() for m in motions)
                ]
            elif len(self._point_motions) != len(self._taught):
                self._point_motions = ["J"] * len(self._taught)
            n = len(self._taught)
            # 点数变了，旧报告的节拍明细/着色不再对得上号——直接作废
            if getattr(self, "_last_point_count", None) != n:
                self._report = None
                self._point_issue_level = {}
                self._last_point_count = n
        finally:
            self._active = prev

        if hidden:
            return
        self.statusMessage.emit(tr("导入 {n} 个示教点").format(n=n))
        self._refresh_point_list()
        if self._robot is not None:
            self.btn_validate.setEnabled(n >= 1)

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
        """由一个**抓取点**（加可选放置点）生成完整的抓取循环点位。

        用户只需把机器人摆到"要抓的地方"和"要放的地方"各示教一次，接近 / 提升 /
        撤离这些安全高度点由向导按竖直偏移算出来——这是示教里最费时间、也最容
        易教歪的一步（手抖一下，接近点就可能离工件太远或撞上去）。

        偏移沿**世界竖直方向（+Z）**、单位毫米：接近点在抓取点上方
        ``approach_mm``，提升点在抓取点上方 ``lift_mm``，撤离点在放置点上方
        ``retract_mm``。

        Args:
            pick_index: 抓取点在当前示教点里的下标（0 起）
            place_index: 放置点下标；``None`` 表示只生成抓取侧（接近/抓取/提升）
            approach_mm / lift_mm / retract_mm: 竖直偏移，毫米
            robot: 算哪台机器人的；``None`` 表示当前这台

        Returns:
            ``(positions, types, motions)``——positions 为 ``{关节名: 弧度}``，
            types 为对应的 :class:`PointType`，motions 为运动类型
            （竖直升降段用 moveL，转运段用 moveJ）

        Raises:
            KinematicsError: 条件不足，或某个偏移点逆解失败（超出工作空间 /
                奇异位形）。**不返回残缺序列**——少一个点就不是完整的抓取循环，
                宁可让用户把高度调小重试，也不要给一串看着完整、实则缺动作的
                示教点
        """
        with self._using_robot(robot) as state:
            if state.robot is None or not state.joint_names:
                raise KinematicsError(tr("还没有载入机器人，无法计算抓取点位"))
            taught = state.taught
            names = state.joint_names
            if not taught:
                raise KinematicsError(
                    tr("还没有示教点——请先把机器人摆到抓取位并记录一个点")
                )
            if not (0 <= pick_index < len(taught)):
                raise KinematicsError(
                    tr("抓取点下标 {i} 超出示教点范围（共 {n} 个）").format(
                        i=pick_index, n=len(taught)
                    )
                )
            if place_index is not None and not (0 <= place_index < len(taught)):
                raise KinematicsError(
                    tr("放置点下标 {i} 超出示教点范围（共 {n} 个）").format(
                        i=place_index, n=len(taught)
                    )
                )

            def joints_at(index: int) -> List[float]:
                """第 ``index`` 个示教点的关节值（度）。"""
                return [math.degrees(float(taught[index].get(n, 0.0))) for n in names]

            def lifted(index: int, dz: float, guess: List[float],
                       label: str) -> List[float]:
                """把第 ``index`` 个示教点的 TCP 沿世界 +Z 抬高 ``dz`` 后逆解。

                以 ``guess`` 为初值（这里传抓取/放置位形本身）：目标就在正上方，
                解出来自然与示教位形同构，不会翻腕翻到另一个姿态去。
                """
                pose = state.robot.forward_kinematics(joints_at(index))
                target = pose.matrix.copy()
                target[2, 3] += float(dz)
                try:
                    return [float(v) for v in
                            state.robot.inverse_kinematics(Pose(target), guess=guess)]
                except KinematicsError as exc:
                    raise KinematicsError(
                        tr("{label}（抬高 {dz:.0f} mm）逆解失败：{msg}").format(
                            label=label, dz=dz, msg=exc
                        )
                    ) from exc

            pick_q = joints_at(pick_index)

            def merged(source_index: int, q: List[float]) -> Dict[str, float]:
                """把解出的可动轴覆盖到来源示教点上，其余关节沿用示教值。

                末端工具轴（``tcp_joint``）这类非驱动关节不在 ``names`` 里，只按
                可动轴重建字典就会把它们整条丢掉——生成的点与用户示教的点结构
                对不上，写回场景时工具的关节值被悄悄抹成 0。
                """
                out = {n: float(v) for n, v in taught[source_index].items()}
                out.update(
                    {n: math.radians(float(q[i])) for i, n in enumerate(names)}
                )
                return out

            sequence = [
                (pick_index, lifted(pick_index, approach_mm, pick_q, tr("接近点")),
                 PointType.APPROACH, "J"),
                (pick_index, pick_q, PointType.PICK, "L"),
                (pick_index, lifted(pick_index, lift_mm, pick_q, tr("提升点")),
                 PointType.LIFT, "L"),
            ]
            if place_index is not None:
                place_q = joints_at(place_index)
                sequence += [
                    (place_index, place_q, PointType.PLACE, "J"),
                    (place_index,
                     lifted(place_index, retract_mm, place_q, tr("撤离点")),
                     PointType.RETRACT, "L"),
                ]

            positions = [merged(src, q) for src, q, _t, _m in sequence]
            return (
                positions,
                [t for _src, _q, t, _m in sequence],
                [m for _src, _q, _t, m in sequence],
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
        """生成抓取循环并写入 ``robot`` 的示教点。

        Args:
            replace: ``True`` 用生成的序列**替换**原有示教点；``False``（默认）
                **追加**到末尾。默认追加：原有示教点是用户一个个摆出来的，凭
                一次向导就抹掉太可惜——真要替换得由用户明确勾选
            robot: 写到哪台；``None`` 表示当前这台

        Returns:
            写入后的示教点总数

        Raises:
            KinematicsError: 点位算不出来（见 :meth:`grasp_cycle_points`）。
                此时**不改动**任何示教点
        """
        new_pos, new_types, new_motions = self.grasp_cycle_points(
            pick_index, place_index, approach_mm, lift_mm, retract_mm, robot
        )
        with self._using_robot(robot) as state:
            cur = [dict(p) for p in state.taught]
            if replace or not cur:
                positions, types, motions = new_pos, new_types, new_motions
            else:
                cur_types = state.point_types
                if cur_types is None or len(cur_types) != len(cur):
                    # 语义缺失或长度对不上时按默认补齐——宁可给用户一份能跑的
                    # 猜测，也不要让追加后整串语义错位
                    cur_types = default_types_for_count(len(cur))
                cur_motions = (
                    list(state.point_motions)
                    if len(state.point_motions) == len(cur)
                    else ["J"] * len(cur)
                )
                positions = cur + new_pos
                types = list(cur_types) + new_types
                motions = cur_motions + new_motions
            self.set_taught_points(positions, types, motions, robot=robot)
            name = robot or self._active
        # 面板内部的编辑广播不带机器人名（多机会串台），故走专用信号；当前这台
        # 额外走一次既有广播，让路径代码编辑器同步（与轨迹编辑器改点同一条路）
        self.graspCycleApplied.emit(name or "")
        if name == self._active:
            self._emit_taught_changed()
        self.statusMessage.emit(
            tr("抓取向导已生成 {n} 个点（{seq}）").format(
                n=len(new_pos),
                seq=" → ".join(t.label for t in new_types),
            )
        )
        return len(positions)

    def open_grasp_wizard(self, robot: Optional[str] = None) -> bool:
        """弹出抓取向导对话框，按用户给的两个示教点生成完整抓取循环。

        Args:
            robot: 对哪台机器人用向导；``None`` 表示当前这台

        Returns:
            确实写入了点位 ``True``；用户取消或算不出来 ``False``
        """
        with self._using_robot(robot) as state:
            taught = state.taught
            name = robot or self._active
        if not taught:
            QtWidgets.QMessageBox.information(
                self, tr("抓取向导"),
                tr("请先记录至少一个示教点——把机器人摆到要抓的位置，点「记录」"),
            )
            return False
        if state.robot is None:
            QtWidgets.QMessageBox.information(
                self, tr("抓取向导"), tr("还没有载入机器人，无法计算抓取点位")
            )
            return False

        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle(tr("抓取向导"))
        form = QtWidgets.QFormLayout(dlg)

        def point_label(i: int) -> str:
            return tr("P{n}").format(n=i + 1)

        combo_pick = QtWidgets.QComboBox()
        combo_place = QtWidgets.QComboBox()
        for i in range(len(taught)):
            combo_pick.addItem(point_label(i), i)
            combo_place.addItem(point_label(i), i)
        combo_place.addItem(tr("（不生成放置侧）"), -1)
        # 默认：抓取点取当前选中行，放置点默认不选（多数人先只想把抓取侧做出来）
        selected = self._selected_index()
        combo_pick.setCurrentIndex(selected if selected is not None else len(taught) - 1)
        combo_place.setCurrentIndex(len(taught))  # 最后一项 = 不生成

        spin_approach = QtWidgets.QDoubleSpinBox()
        spin_lift = QtWidgets.QDoubleSpinBox()
        spin_retract = QtWidgets.QDoubleSpinBox()
        for spin, value in ((spin_approach, 50.0), (spin_lift, 50.0),
                            (spin_retract, 50.0)):
            spin.setRange(0.0, 2000.0)
            spin.setDecimals(1)
            spin.setSuffix(" mm")
            spin.setValue(value)
        spin_retract.setEnabled(False)

        check_replace = QtWidgets.QCheckBox(tr("替换原有示教点（默认追加到末尾）"))

        preview = QtWidgets.QLabel(tr("接近点 → 抓取点 → 提升点"))
        preview.setWordWrap(True)

        def refresh_preview() -> None:
            spin_retract.setEnabled(combo_place.currentData() != -1)
            seq = [tr("接近点"), tr("抓取点"), tr("提升点")]
            if combo_place.currentData() != -1:
                seq += [tr("放置点"), tr("撤离点")]
            preview.setText(tr("将生成 {n} 个点：{seq}").format(
                n=len(seq), seq=" → ".join(seq)
            ))

        combo_place.currentIndexChanged.connect(lambda _i: refresh_preview())
        refresh_preview()

        form.addRow(tr("抓取点"), combo_pick)
        form.addRow(tr("放置点"), combo_place)
        form.addRow(tr("接近高度（抓取点上方）"), spin_approach)
        form.addRow(tr("提升高度（抓取点上方）"), spin_lift)
        form.addRow(tr("撤离高度（放置点上方）"), spin_retract)
        form.addRow(check_replace)
        form.addRow(preview)

        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        buttons.button(QtWidgets.QDialogButtonBox.Ok).setText(tr("生成"))
        buttons.button(QtWidgets.QDialogButtonBox.Cancel).setText(tr("取消"))
        buttons.accepted.connect(dlg.accept)
        buttons.rejected.connect(dlg.reject)
        form.addRow(buttons)

        if not dlg.exec():
            return False

        place = combo_place.currentData()
        try:
            self.apply_grasp_cycle(
                pick_index=int(combo_pick.currentData()),
                place_index=None if place == -1 else int(place),
                approach_mm=spin_approach.value(),
                lift_mm=spin_lift.value(),
                retract_mm=spin_retract.value(),
                replace=check_replace.isChecked(),
                robot=name,
            )
        except KinematicsError as exc:
            # 逆解失败不能悄无声息：告诉用户是哪个点、抬了多少，让他调小高度重试
            QtWidgets.QMessageBox.warning(
                self, tr("抓取向导"),
                tr("点位算不出来，示教点未改动：\n{msg}").format(msg=exc),
            )
            return False
        return True

    # ---------------------------- 轨迹编辑器 ---------------------------- #

    def get_taught_points(self, robot: Optional[str] = None) -> tuple:
        """读取示教点数据（供外部同步与测试）。

        Args:
            robot: 读哪台的；``None`` 表示当前这台。抓取向导写完点后主窗口要
                按名字把点位回灌视口，故支持指定机器人

        Returns:
            ``(positions, types, motions)``——positions 为
            ``{关节名: 弧度}`` 字典列表的拷贝；types 可为 ``None``；
            motions 为 ``"J"/"L"/"C"`` 列表
        """
        with self._using_robot(robot) as state:
            positions = [dict(p) for p in state.taught]
            types = list(state.point_types) if state.point_types is not None else None
            return positions, types, list(state.point_motions)

    def _selected_index(self) -> Optional[int]:
        """当前选中行号；未选中返回 ``None``。"""
        row = self.point_list.currentRow()
        return row if 0 <= row < len(self._taught) else None

    # ---------------------------- 运动类型解析（v1.1.5 圆弧） ---------------------------- #

    def _motion_kind(self, index: int) -> str:
        """第 ``index`` 行声明的运动类型（``"J"`` / ``"L"`` / ``"C"``）。"""
        if index < len(self._point_motions):
            kind = str(self._point_motions[index]).upper()
            if kind in ("J", "L", "C"):
                return kind
        return "J"

    def _gripper_action_at(self, index: int) -> Optional[str]:
        """第 ``index`` 行是否带夹爪动作（抓取/放置语义）。"""
        if self._point_types is not None and index < len(self._point_types):
            return gripper_action_for(self._point_types[index])
        return None

    def arc_rejection(self, index: int) -> Optional[str]:
        """把第 ``index`` 行设为 ``moveC`` 是否可行；不可行时返回原因（可直接展示）。

        圆弧的约定（轨迹编辑器里的口径）：**第 N 行的运动类型描述"到达第 N 点"
        的运动**——``C`` 表示"以上一点为中间点、再上一点为起点，圆弧到达本点"。
        因此：

        * 第 1、2 行不能是圆弧（没有两个前驱点）；
        * 相邻两行不能同为圆弧（中间点会同时成为上一段的终点，几何上矛盾）；
        * 带工艺语义的点（抓取/放置）不能当中间点——中间点不停机，夹爪动作
          会失去落点。

        Returns:
            拒绝原因（中文，可直接显示）；可行时 ``None``
        """
        if not (0 <= index < len(self._taught)):
            return tr("路点不存在")
        if index < 2:
            return tr("P{n} 缺少两个前驱点——圆弧需要起点与中间点，P1/P2 不能设为 moveC").format(
                n=index + 1
            )
        if self._motion_kind(index - 1) == "C":
            return tr("P{n} 已是圆弧终点，不能再作为本段的中间点").format(n=index)
        if index + 1 < len(self._taught) and self._motion_kind(index + 1) == "C":
            return tr("P{n} 已把 P{k} 当中间点——本行不能同时作为圆弧终点").format(
                n=index + 2, k=index + 1
            )
        if self._gripper_action_at(index - 1) is not None:
            return tr("P{n} 是抓取/放置点，不能作为圆弧中间点（中间点不停机）").format(n=index)
        return None

    def motion_plan(self) -> Tuple[List[Tuple[str, int, Optional[int]]], List[str]]:
        """把逐点运动类型解析成可执行的运动序列。

        Returns:
            ``(plan, notes)``：plan 每项为 ``(kind, point_index, via_index)``，
            ``kind`` ∈ ``{"J","L","C"}``，``via_index`` 仅圆弧有值；
            notes 为降级说明（圆弧条件不满足时退化为关节运动，这里如实记下）。
        """
        n = len(self._taught)
        declared = [self._motion_kind(i) for i in range(n)]
        accepted: Dict[int, int] = {}  # 圆弧终点行 → 中间点行
        notes: List[str] = []
        for i in range(n):
            if declared[i] != "C":
                continue
            reason = self.arc_rejection(i)
            if reason is not None:
                notes.append(tr("{reason}——已按关节运动（moveJ）处理").format(reason=reason))
                continue
            accepted[i] = i - 1
        vias = set(accepted.values())
        plan: List[Tuple[str, int, Optional[int]]] = []
        for i in range(n):
            if i in vias:  # 中间点不再单独出运动指令
                continue
            if i in accepted:
                plan.append(("C", i, accepted[i]))
            else:
                kind = declared[i]
                plan.append((("J" if kind == "C" else kind), i, None))
        return plan, notes

    def arc_via_indices(self) -> set:
        """当前被当作圆弧中间点（不再单独出指令）的路点行号集合。"""
        return {via for _kind, _i, via in self.motion_plan()[0] if via is not None}

    def instruction_of_point(self) -> Dict[int, int]:
        """路点行号 → 运动指令序号（圆弧把中间点并进上一条指令，两者不再一一对应）。"""
        plan, _notes = self.motion_plan()
        return {index: order for order, (_k, index, _v) in enumerate(plan)}

    def last_build_notes(self) -> List[str]:
        """最近一次 ``build_program`` 的降级说明（空表表示没有降级）。"""
        return list(self._build_notes)

    def _emit_taught_changed(self) -> None:
        """示教点被编辑后广播，供主窗口回写路径代码编辑器。"""
        positions, types, motions = self.get_taught_points()
        self.taughtPointsChanged.emit(positions, types or [], motions)

    def _invalidate_results(self) -> None:
        """示教点变了，旧轨迹/报告/回放全部作废。"""
        self.stop_playback()
        self._trajectory = None
        self._report = None
        self._planner = None
        self._gripper_animator = None
        self.btn_play.setEnabled(False)

    def insert_point(self, index: Optional[int] = None) -> Optional[int]:
        """在 ``index`` 之后插入路点，复制其关节值与运动类型。

        Args:
            index: 插入位置（新点落在该序号后一位）；``None`` 取当前
                选中行，无选中时追加到末尾。首点为空时填零位

        Returns:
            新点序号；示教点未初始化时返回 ``None``
        """
        if not self._taught:
            if self._joint_names:
                zero = {name: 0.0 for name in self._joint_names}
                self._taught = [zero]
                self._point_types = None
                self._point_motions = ["J"]
            else:
                self.statusMessage.emit(tr("请先载入机器人再添加路点"))
                return None
        else:
            if index is None:
                index = self._selected_index()
            if index is None or not (0 <= index < len(self._taught)):
                index = len(self._taught) - 1
            src = dict(self._taught[index])
            self._taught.insert(index + 1, src)
            if self._point_types is not None:
                self._point_types.insert(index + 1, self._point_types[index])
            # 复制运动类型：同一位置附近通常保持同一种走法
            self._point_motions.insert(index + 1, self._point_motions[index])
            index = index + 1
        self._invalidate_results()
        self._refresh_point_list()
        self.point_list.setCurrentRow(index)
        self._emit_taught_changed()
        self.statusMessage.emit(tr("已插入路点 {n}，请重新校验").format(n=index + 1))
        return index

    def delete_point(self, index: Optional[int] = None) -> bool:
        """删除 ``index`` 处的路点（``None`` 取当前选中行）。"""
        if not self._taught:
            return False
        if index is None:
            index = self._selected_index()
        if index is None or not (0 <= index < len(self._taught)):
            self.statusMessage.emit(tr("请先在列表中选中要删除的路点"))
            return False
        del self._taught[index]
        if self._point_types is not None and index < len(self._point_types):
            del self._point_types[index]
        if index < len(self._point_motions):
            del self._point_motions[index]
        self._invalidate_results()
        self._refresh_point_list()
        self._emit_taught_changed()
        self.statusMessage.emit(tr("已删除路点 {n}，请重新校验").format(n=index + 1))
        return True

    def move_point(self, src: int, dst: int) -> bool:
        """把路点从 ``src`` 移动到 ``dst``（``dst`` 为移动后的新序号）。"""
        n = len(self._taught)
        if not (0 <= src < n) or not (0 <= dst < n) or src == dst:
            return False
        for seq in (self._taught, self._point_motions):
            seq.insert(dst, seq.pop(src))
        if self._point_types is not None and len(self._point_types) == n:
            self._point_types.insert(dst, self._point_types.pop(src))
        self._invalidate_results()
        self._refresh_point_list()
        self.point_list.setCurrentRow(dst)
        self._emit_taught_changed()
        return True

    def shift_selected_point(self, delta: int) -> bool:
        """把当前选中路点上移（``delta=-1``）或下移（``delta=1``）。"""
        index = self._selected_index()
        if index is None:
            return False
        return self.move_point(index, index + delta)

    def set_point_motion(self, index: int, motion: str) -> bool:
        """设置单个路点的运动类型（``"J"``=moveJ / ``"L"``=moveL / ``"C"``=moveC）。

        ``"C"`` 不满足圆弧条件时**拒绝设置并说明原因**（见 :meth:`arc_rejection`）——
        悄悄改成别的运动类型会让"我以为这是圆弧"变成现场查不动的差异。
        """
        if not (0 <= index < len(self._taught)):
            return False
        kind = str(motion).upper()
        if kind == "C":
            reason = self.arc_rejection(index)
            if reason is not None:
                self.statusMessage.emit(reason)
                return False
        motion = kind if kind in ("L", "C") else "J"
        if 0 <= index < len(self._point_motions) and self._point_motions[index] == motion:
            return False
        while index >= len(self._point_motions):
            self._point_motions.append("J")
        self._point_motions[index] = motion
        self._invalidate_results()
        self._refresh_point_list()
        self._emit_taught_changed()
        return True

    def set_selected_motion(self, motion: str) -> bool:
        """把当前选中路点设为指定运动类型（按钮入口）。"""
        index = self._selected_index()
        if index is None:
            self.statusMessage.emit(tr("请先在列表中选中要改运动类型的路点"))
            return False
        labels = {
            "L": tr("moveL（直线）"),
            "C": tr("moveC（圆弧）"),
        }
        changed = self.set_point_motion(index, motion)
        if changed:
            label = labels.get(str(motion).upper(), tr("moveJ（关节）"))
            self.statusMessage.emit(tr("{n} 已设为 {label}，请重新校验").format(n=index + 1, label=label))
        return changed

    def update_point(
        self,
        index: int,
        joints_deg: Sequence[float],
        motion: Optional[str] = None,
    ) -> bool:
        """写入路点的关节角（**度**）与可选的运动类型。

        Args:
            index: 路点序号
            joints_deg: 各关节角度值，顺序与 ``_joint_names`` 对齐
            motion: ``"J"/"L"/"C"``；``None`` 保持不变
        """
        if not (0 <= index < len(self._taught)):
            return False
        pos = self._taught[index]
        for i, name in enumerate(self._joint_names):
            if i < len(joints_deg):
                pos[name] = math.radians(float(joints_deg[i]))
        if motion is not None:
            self.set_point_motion(index, motion)
        self._invalidate_results()
        self._refresh_point_list()
        self._emit_taught_changed()
        return True

    def edit_selected_point(self) -> None:
        """弹出对话框编辑当前选中路点的关节角与运动类型。"""
        index = self._selected_index()
        if index is None or not self._taught:
            self.statusMessage.emit(tr("请先在列表中选中要编辑的路点"))
            return
        joints = [math.degrees(float(self._taught[index].get(n, 0.0)))
                  for n in self._joint_names]
        motion = (
            self._point_motions[index]
            if index < len(self._point_motions) else "J"
        )

        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle(tr("编辑路点 {n}").format(n=index + 1))
        form = QtWidgets.QFormLayout(dialog)
        spins: List[QtWidgets.QDoubleSpinBox] = []
        for i, name in enumerate(self._joint_names):
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(-360.0, 360.0)
            spin.setDecimals(2)
            spin.setSuffix(" °")
            if i < len(joints):
                spin.setValue(joints[i])
            spins.append(spin)
            form.addRow(name, spin)
        combo = QtWidgets.QComboBox()
        combo.addItem(tr("moveJ（关节插值）"), "J")
        combo.addItem(tr("moveL（直线插值）"), "L")
        combo.addItem(tr("moveC（圆弧，以上一点为中间点）"), "C")
        combo.setCurrentIndex({"L": 1, "C": 2}.get(str(motion).upper(), 0))
        form.addRow(tr("运动类型"), combo)
        if motion != "C":
            hint = self.arc_rejection(index)
            hint_label = QtWidgets.QLabel(
                tr("可设为 moveC（圆弧）") if hint is None
                else tr("不可设为 moveC：{reason}").format(reason=hint)
            )
            hint_label.setWordWrap(True)
            hint_label.setStyleSheet("color:#90caf9; font-size:11px;")
            form.addRow(hint_label)
        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        form.addRow(buttons)
        if dialog.exec() != QtWidgets.QDialog.Accepted:
            return
        self.update_point(
            index,
            [spin.value() for spin in spins],
            combo.currentData(),
        )
        self.statusMessage.emit(tr("{n} 已更新，请重新校验").format(n=index + 1))

    def _refresh_point_list(self) -> None:
        """按当前示教点数据重建列表，并套用最近一次校验的着色。

        圆弧（v1.1.5）会打破"每个路点恰好一条运动指令"的旧前提——圆弧的中间点
        被并进上一条指令。因此节拍明细必须经路点→指令序号映射取值，**不能按下标
        硬取**（否则圆弧之后的每一行节拍都会错位一行）。
        """
        report = self._report
        motion_times = report.motion_times if report is not None else None
        labels = report.motion_labels if report is not None else None
        levels = getattr(self, "_point_issue_level", {}) if report is not None else {}
        point_to_instruction = self.instruction_of_point() if report is not None else {}
        vias = self.arc_via_indices()

        self.point_list.clear()
        for i, pos in enumerate(self._taught):
            mtype = self._motion_kind(i)
            if i in vias:
                label = tr("moveC 中间点")
            elif mtype == "C":
                label = "moveC"
            elif mtype == "L":
                label = "moveL"
            else:
                label = "moveJ"
            joints = ", ".join(
                f"{math.degrees(float(pos.get(n, 0.0))):.1f}" for n in self._joint_names
            )
            text = f"P{i + 1} · {label} · [{joints}]"
            instruction = point_to_instruction.get(i)
            if (
                motion_times is not None
                and instruction is not None
                and instruction < len(motion_times)
            ):
                kind = (
                    labels[instruction]
                    if labels and instruction < len(labels) else label
                )
                text += f" · {kind} {motion_times[instruction]:.2f}s"
            item = QtWidgets.QListWidgetItem(text)
            level = levels.get(i)
            if level == "error":
                item.setForeground(QtGui.QColor("#ff8080"))
                item.setToolTip(tr("该校验错误涉及此路点（见校验结果）"))
            elif level == "warning":
                item.setForeground(QtGui.QColor("#ffc866"))
                item.setToolTip(tr("该校验警告涉及此路点（见校验结果）"))
            elif report is not None:
                item.setForeground(QtGui.QColor("#6fdc6f"))
            self.point_list.addItem(item)

    def _apply_point_colors(self, report: ProgramReport) -> None:
        """从校验报告提取每个路点的最严重问题级别并刷新列表着色。

        ``Issue.motion_index`` 是**运动指令**序号，而列表行是**路点**序号。无圆弧时
        两者一一对应，有圆弧时中间点被并进上一条指令（指令数少于路点数），因此这里
        反向映射指令→路点；指向中间点的指令把着色落在**圆弧终点**那一行——问题实际
        出在圆弧段上，落在被并入的那一行最容易被看见。
        """
        self._point_issue_level = self._point_levels(report)
        self._refresh_point_list()

    def _point_levels(self, report: "ProgramReport") -> Dict[int, str]:
        """报告里每个路点的最严重问题级别——**只算，不刷界面**。

        映射规则同 :meth:`_apply_point_colors`，但不碰列表控件：一键校验全部
        时要逐台算完存进各自的状态槽，界面从头到尾只显示当前那台。
        """
        levels: Dict[int, str] = {}
        plan, _notes = self.motion_plan()
        instruction_to_point = {
            order: index for order, (_kind, index, _via) in enumerate(plan)
        }
        for issue in report.issues:
            point_index = instruction_to_point.get(issue.motion_index, issue.motion_index)
            level = "error" if issue.severity == SEVERITY_ERROR else "warning"
            if levels.get(point_index) != "error":
                levels[point_index] = level
        return levels

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
        """组装**指定机器人**（默认当前这台）的程序。

        Args:
            robot: 机器人名；``None`` 表示当前正在编辑的那台。批量出全部機器人
                的代码时逐个指名即可，界面不会跟着切走
        """
        with self._using_robot(robot):
            return self._build_program_impl()

    def _build_program_impl(self) -> Optional[Program]:
        # 实际装配：调用方已把当前槽切到目标机器人
        """把示教点组装成程序。

        示教点内部以弧度保存，:class:`Motion` 期望角度。
        带工艺语义的示教点（抓取/放置）会自动追加 ``gripper()`` 指令组，
        于是回放动画、程序校验与后处理出码共用同一份抓取时序。
        运动类型逐点取自轨迹编辑器（:meth:`motion_plan`）：``"L"`` 的点位先对该
        关节配置做 FK 得到 TCP 位姿、再以 ``moveL`` 直线插补过去；``"C"`` 的点位
        以**上一点为中间点**做 ``moveC``（中间点不再单独出指令）。

        圆弧正解失败（位形奇异/越限等）时**退化为关节运动并记入**
        :meth:`last_build_notes`，绝不静默改成另一条几何路径却让界面以为还是圆弧。

        Returns:
            :class:`Program`；无机器人或无示教点时返回 ``None``
        """
        self._build_notes = []
        if self._robot is None or not self._taught:
            return None
        speed = float(self.spin_speed.value())
        builder = ProgramBuilder(self._robot)
        plan, notes = self.motion_plan()
        self._build_notes.extend(notes)

        def _pose_of(index: int):
            """路点关节配置 → TCP 位姿（度）。"""
            pos = self._taught[index]
            joints = [math.degrees(float(pos.get(n, 0.0))) for n in self._joint_names]
            return self._robot.forward_kinematics(joints), joints

        def _joint_move(index: int) -> None:
            """关节运动到某个示教点——直接下发关节值、不走 FK，任何位形都可用。"""
            joints = [
                math.degrees(float(self._taught[index].get(n, 0.0)))
                for n in self._joint_names
            ]
            builder.moveJ(joints, speed=speed)

        def _gripper_after(index: int) -> None:
            """该点的工艺动作（抓取/放置）——回放与出码共用这份时序。"""
            t = (
                self._point_types[index]
                if self._point_types and index < len(self._point_types)
                else None
            )
            action = gripper_action_for(t)
            if action:
                builder.gripper(action)

        for kind, i, via_index in plan:
            if kind == "C":
                moved = False
                try:
                    via_pose, _ = _pose_of(via_index)
                    target_pose, _ = _pose_of(i)
                except Exception:
                    self._build_notes.append(
                        tr("P{n} 的圆弧正解失败——退化为关节运动（moveJ）；中间点 P{k} 改为单独经过").format(
                            n=i + 1, k=via_index + 1
                        )
                    )
                else:
                    try:
                        builder.moveC(via_pose, target_pose, speed=speed)
                        moved = True
                    except Exception:
                        self._build_notes.append(
                            tr("P{n} 的圆弧无法加入程序——退化为关节运动（moveJ）；中间点 P{k} 改为单独经过").format(
                                n=i + 1, k=via_index + 1
                            )
                        )
                if not moved:
                    # 圆弧没了，中间点也不能跟着一起消失：示教了 4 个点却只走 3 个，
                    # 现场表现就是"这一拐没绕过去"。补一条 moveJ 单独经过中间点——
                    # 精度不如圆弧，但经过的示教点一个不少。
                    _joint_move(via_index)
                    _joint_move(i)
            elif kind == "L":
                moved = False
                try:
                    pose, _ = _pose_of(i)
                    builder.moveL(pose, speed=speed)
                    moved = True
                except Exception:
                    # FK 失败（位形奇异/越限等）退回关节运动：宁可走不到
                    # 直线，也不能让整个程序构建崩掉
                    pass
                if not moved:
                    _joint_move(i)
            else:
                _joint_move(i)
            _gripper_after(i)
        return builder.build()

    def validate_program(self) -> Optional[ProgramReport]:
        """校验程序并刷新界面。

        Returns:
            :class:`ProgramReport`；条件不足时返回 ``None``
        """
        if self._robot is None:
            self.statusMessage.emit(tr("请先载入机器人型号，再校验程序"))
            return None

        program = self.build_program()
        if program is None:
            self.statusMessage.emit(tr("没有示教点可供校验——请先记录若干关节位形"))
            return None

        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            report = program.validate(scene=self._scene)
            planner = TrajectoryPlanner(self._robot, scene=self._scene)
            self._trajectory = planner.plan(program)
        except Exception as exc:
            # 校验本身崩了也要让用户看见——这类异常几乎总是模型数据的问题
            self.statusMessage.emit(tr("校验异常：{kind}: {msg}").format(kind=type(exc).__name__, msg=exc))
            return None
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()

        # 规划器要留着：其内部记录了每条指令的时间区间，抓取动画靠它把
        # gripper() 指令映射到轨迹时刻
        self._planner = planner
        self._gripper_animator = GraspAnimator(program, planner)

        self._report = report
        self._show_report(report)
        # 路点列表着色：错误红/警告橙/正常绿 + 逐指令节拍（v0.2.4）
        self._apply_point_colors(report)
        self.reportReady.emit(report)

        # 有轨迹就能回放——即使校验没通过，也允许用户看看走到哪儿崩的
        self.btn_play.setEnabled(bool(self._trajectory and len(self._trajectory.waypoints) > 1))
        if getattr(self, "btn_play_all", None) is not None:
            self.btn_play_all.setEnabled(bool(self._playable()))
        message = tr("校验完成：{res}").format(
            res=tr("通过") if report.ok else tr("{n} 处错误").format(n=len(report.errors))
        )
        if self._build_notes:
            # 圆弧降级这类"几何被换掉了"的事必须说出来，不能只在日志里躺着
            message += tr("；{n} 项运动降级（见日志）").format(n=len(self._build_notes))
            for note in self._build_notes:
                self.statusMessage.emit(tr("⚠ {note}").format(note=note))
        self.statusMessage.emit(message)
        return report

    def validate_all(self) -> Dict[str, Optional["ProgramReport"]]:
        """逐台校验**所有**已登记的机器人（一键校验全部）。

        装配与规划和 :meth:`validate_program` 完全一致，区别在于**不逐台刷界面**
        ——界面始终显示当前那台。否则问题列表会被一遍遍刷掉，用户只看见最后
        一台的结论，前面几台校出问题等于没看见。每台的结果（轨迹 / 报告 /
        夹爪时序）各存进自己的状态槽，随后「全部回放」直接复用这些轨迹。

        Returns:
            ``{机器人名: ProgramReport}``；没型号、没示教点或校验本身出错的那台
            对应 ``None``——"没结果"和"通过"是两回事，不混为一谈
        """
        reports: Dict[str, Optional["ProgramReport"]] = {}
        for name in list(self._states):
            state = self._states.get(name)
            if state is None or state.robot is None:
                continue
            with self._using_robot(name):
                program = self._build_program_impl()
                if program is None:
                    reports[name] = None  # 这台还没教过程序
                    continue
                try:
                    report = program.validate(scene=self._scene)
                    planner = TrajectoryPlanner(self._robot, scene=self._scene)
                    state.trajectory = planner.plan(program)
                except Exception as exc:
                    # 一台崩了不能带崩全部：如实记下来，继续校其余的
                    state.trajectory = None
                    state.report = None
                    reports[name] = None
                    self.statusMessage.emit(
                        tr("{name} 校验异常：{kind}: {msg}").format(
                            name=name, kind=type(exc).__name__, msg=exc
                        )
                    )
                    continue
                state.planner = planner
                state.gripper_animator = GraspAnimator(program, planner)
                state.report = report
                state.point_issue_level = self._point_levels(report)
                reports[name] = report

        # 当前这台若也校过了，顺手把它的结论刷进界面——用户正看着它
        active = self._active
        if active in reports and reports[active] is not None:
            self._show_report(reports[active])
            self._apply_point_colors(reports[active])
        if getattr(self, "btn_play_all", None) is not None:
            self.btn_play_all.setEnabled(bool(self._playable()))

        passed = [n for n, r in reports.items() if r is not None and r.ok]
        failed = [n for n, r in reports.items() if r is not None and not r.ok]
        skipped = [n for n, r in reports.items() if r is None]
        parts = []
        if passed:
            parts.append(tr("{n} 台通过").format(n=len(passed)))
        if failed:
            parts.append(tr("{n} 台有问题").format(n=len(failed)))
        if skipped:
            parts.append(tr("{n} 台没程序").format(n=len(skipped)))
        self.statusMessage.emit(
            tr("校验全部：{res}").format(
                res="、".join(parts) or tr("没有可校验的机器人")
            )
        )
        self.allValidated.emit(reports)
        return reports

    def focus_report(self, name: Optional[str] = None) -> bool:
        """切到 ``name`` 并把它**已存的**校验报告刷进界面。

        汇总对话框里点「看这台」走这里：报告已经算过了，不必重校一遍。

        Returns:
            有报告并已显示 ``True``；这台还没校过返回 ``False``
        """
        key = name or self._active
        state = self._state(key)
        if state is None or state.report is None:
            self.statusMessage.emit(
                tr("{name} 还没有校验结果").format(name=key or tr("这台"))
            )
            return False
        self.set_active_robot(key)
        self._show_report(state.report)
        self._apply_point_colors(state.report)
        # 视口与示教点显示也要跟过去——否则"看的是 A 的报告，画的是 B"
        self.activeRobotChanged.emit(key)
        return True

    # ---------------------------- 设备联调 ---------------------------- #

    def generate_code(self) -> None:
        """按选择的后处理器生成控制器代码并预览。

        失败时如实抛错并清空预览——绝不留下"看起来成功"的废代码。
        """
        program = self.build_program()
        if program is None:
            self.statusMessage.emit(tr("请先加载机器人并示教至少一个点"))
            return
        choice = self.combo_post.currentData()  # None = 自动（按机器人品牌推断）
        # 程序名用**场景里的机器人名**而不是 URDF 文件名：多机同型号时后者会
        # 重名，落出 second.src / second.src 互相覆盖
        prog_name = self._active or (self._robot.name if self._robot else "program")
        try:
            code = program.to_post(choice, metadata={"program_name": prog_name})
        except Exception as exc:
            self._last_code = ""
            self.btn_export.setEnabled(False)
            self.code_preview.setPlainText(tr("生成失败：\n{exc}").format(exc=exc))
            self.statusMessage.emit(tr("生成失败: {exc}").format(exc=exc))
            return
        self._last_code = code
        self.code_preview.setPlainText(code)
        self.btn_export.setEnabled(True)
        self.statusMessage.emit(
            tr("已生成 {choice} 代码，共 {lines} 行").format(
                choice=choice or tr("自动"), lines=len(code.splitlines())
            )
        )

    def export_program(self) -> None:
        """一键导出程序到指定目录（走 ``FileLink`` 离线部署通道）。

        不要求先点「生成代码」：内部直接构建程序并后处理，按品牌落出
        正确扩展名的文件（UR ``.script`` / KUKA ``.src`` / ABB ``.mod`` /
        FANUC ``.tp``…），并写 ``.manifest.json`` 部署清单。
        """
        program = self.build_program()
        if program is None:
            self.statusMessage.emit(tr("请先加载机器人并示教至少一个点"))
            return
        dest = QtWidgets.QFileDialog.getExistingDirectory(
            self, tr("选择导出目录"), str(Path.cwd())
        )
        if not dest:
            return
        self.export_program_to(program, dest)

    def export_program_to(
        self, program: Program, dest: str, robot: Optional[str] = None
    ) -> bool:
        """把 ``program`` 导出到 ``dest`` 目录（无 UI 逻辑，供测试复用）。

        Args:
            program: 已构建好的程序
            dest: 目标目录
            robot: 这台机器人的名字（决定文件名与其槽位写入）；``None`` 取当前

        Returns:
            成功 ``True``；后处理/落盘失败 ``False``（状态栏带原因）
        """
        with self._using_robot(robot):
            return self._export_impl(program, dest)

    def _export_impl(self, program: Program, dest: str) -> bool:
        name = self._active or (self._robot.name if self._robot else "program")
        try:
            link = FileLink(dest)
            link.connect()
            deployment = program.deploy(
                link,
                self.combo_post.currentData(),
                name=name,
                metadata={"program_name": name},
            )
        except Exception as exc:
            self.statusMessage.emit(tr("导出失败: {exc}").format(exc=exc))
            return False
        self._last_code = deployment.code
        self.btn_export.setEnabled(True)
        note = tr("（{note}）").format(note=deployment.notes) if deployment.notes else ""
        self.statusMessage.emit(tr("已导出：{handle}{note}").format(handle=deployment.handle, note=note))
        return True

    # ---------------------------- 存盘 / 恢复 ---------------------------- #

    def robot_snapshot(self) -> Dict[str, dict]:
        """各台机器人的示教点与抓取语义快照，供工程存盘。

        工程里原先只有一份 ``program`` 段（来自只显示当前那台的路径编辑器），
        多机工程存盘再打开会丢掉其余机器人教好的抓取循环——这段就是为此加的。

        Returns:
            ``{机器人名: {"joint_names": [...], "positions": [{关节名: 弧度}],
            "point_types": [...]}}``
        """
        out: Dict[str, dict] = {}
        for name, st in self._states.items():
            if st.robot is None:
                continue
            out[name] = {
                "joint_names": list(st.joint_names),
                "positions": [
                    {k: float(v) for k, v in p.items()} for p in st.taught
                ],
                "point_types": [
                    getattr(t, "value", None) for t in (st.point_types or [])
                ],
            }
        return out

    def restore_robots(self, data: Optional[dict]) -> None:
        """把 :meth:`robot_snapshot` 的快照灌回各台机器人。

        场景里已经没有的机器人直接跳过——存档比场景多是正常的（用户删过机器
        人），不该为它凭空造一个条目出来。语义数量对不上时按默认语义重建，
        绝不猜哪个是抓取点。
        """
        for name, payload in (data or {}).items():
            if name not in self._states:
                continue
            positions = payload.get("positions") or []
            if not positions:
                continue
            raw = payload.get("point_types") or []
            types = None
            if raw and len(raw) == len(positions):
                try:
                    types = [PointType(t) for t in raw]
                except ValueError:
                    types = None
            self.set_taught_points(positions, types, robot=name)

    # ---------------------------- 多机出码 ---------------------------- #

    def generate_all_code(self) -> Dict[str, str]:
        """给**每一台**机器人各生成一份目标控制器代码。

        后处理器统一用下拉里选的那个（``None`` = 各自按品牌推断），这样一次按钮
        就能拿到"整条产线"的程序，而不是逐台点一遍。

        Returns:
            ``{机器人名: 代码}``；某台生成失败时不进结果，原因已广播到状态栏
        """
        codes: Dict[str, str] = {}
        if not self._states:
            self.statusMessage.emit(tr("请先加载机器人并示教至少一个点"))
            return codes
        choice = self.combo_post.currentData()
        for name in list(self._states):
            program = self.build_program(name)
            if program is None:
                self.statusMessage.emit(tr("{name}：无示教点，已跳过").format(name=name))
                continue
            try:
                codes[name] = program.to_post(choice, metadata={"program_name": name})
            except Exception as exc:
                self.statusMessage.emit(
                    tr("{name} 生成失败：{exc}").format(name=name, exc=exc)
                )
        # 预览仍保持当前这台：点了"导出全部"就把预览换成别人家的代码，很容易
        # 让人以为当前编辑的那份丢了
        if self._active in codes:
            self._last_code = codes[self._active]
            self.code_preview.setPlainText(codes[self._active])
        if codes:
            self.statusMessage.emit(tr("已生成 {n} 台机器人的代码").format(n=len(codes)))
        return codes

    def export_all_programs(self) -> None:
        """把**所有**机器人的程序一次导出到同一个目录（每台一个文件）。"""
        codes = self.generate_all_code()
        if not codes:
            self.statusMessage.emit(tr("没有可导出的机器人程序"))
            return
        dest = QtWidgets.QFileDialog.getExistingDirectory(
            self, tr("选择导出目录"), str(Path.cwd())
        )
        if not dest:
            return
        ok = 0
        for name in list(codes):
            program = self.build_program(name)
            if program is None:
                continue
            if self.export_program_to(program, dest, robot=name):
                ok += 1
        self.statusMessage.emit(
            tr("已导出 {n}/{total} 个机器人程序到 {dest}").format(
                n=ok, total=len(codes), dest=dest
            )
        )

    def _show_report(self, report: ProgramReport) -> None:
        """把报告填进界面。"""
        self.lbl_cycle.setText(
            f"{report.cycle_time:.2f} s" if report.cycle_time is not None else "—"
        )
        self.lbl_length.setText(f"{report.path_length:.1f} mm")
        self.lbl_points.setText(tr("{w} 个 / {m} 条指令").format(w=report.num_waypoints, m=report.num_motions))

        self.issue_list.clear()
        if report.ok and not report.warnings:
            item = QtWidgets.QListWidgetItem(tr("✓ 程序校验通过，未发现问题"))
            item.setForeground(QtGui.QColor("#6fdc6f"))
            self.issue_list.addItem(item)
            return

        for issue in report.errors:
            item = QtWidgets.QListWidgetItem(str(issue))
            item.setForeground(QtGui.QColor("#ff8080"))
            item.setToolTip(issue.message)
            self.issue_list.addItem(item)
        for issue in report.warnings:
            item = QtWidgets.QListWidgetItem(str(issue))
            item.setForeground(QtGui.QColor("#ffc866"))
            item.setToolTip(issue.message)
            self.issue_list.addItem(item)

    def _clear_metrics(self) -> None:
        self.lbl_cycle.setText("—")
        self.lbl_length.setText("—")
        self.lbl_points.setText("—")

    # ---------------------------- 回放 ---------------------------- #

    def start_playback(self, robots: Optional[Sequence[str]] = None) -> None:
        """从轨迹起点开始回放。

        Args:
            robots: 要回放的机器人名序列；``None`` 表示当前这台。
                多台时**各自按自己的时间轴**推进但共用同一个定时器——这就是
                多机协同回放：能直观看出两台是否会抢同一个工位
        """
        wanted = list(robots) if robots is not None else self._playable()
        names = [n for n in wanted if self._playable(n)]
        if not names:
            # 不用模态弹窗：无头/自动化场景下会永久阻塞，且"还没校验"只是
            # 普通状态提示，不值得打断用户
            self.statusMessage.emit(tr("请先校验程序以生成轨迹，再回放"))
            return

        for name in names:
            state = self._states[name]
            traj = state.trajectory
            state.playhead = float(traj.waypoints[0].timestamp)
            # 回放从头开始：夹爪状态复位为未夹持，主窗口会据此放下工件
            if state.gripper_animator is not None:
                state.gripper_animator.reset()
            self._set_gripper_for(name, "")

        self._playing = names
        self.btn_play.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self._timer.start()
        if len(names) > 1:
            self.statusMessage.emit(tr("多机回放中…（{n} 台）").format(n=len(names)))
        else:
            self.statusMessage.emit(tr("轨迹回放中…"))

    def start_playback_all(self) -> None:
        """所有已校验过的机器人**锁步同播**（多机回放）。"""
        if not self._playable():
            self.statusMessage.emit(tr("没有已生成轨迹的机器人——请先校验程序"))
            return
        self.start_playback(self._playable())

    def _playable(self, name: Optional[str] = None) -> List[str]:
        """可回放的机器人名——必须已校验出多于一个路点的轨迹。

        Args:
            name: 只看这台；``None`` 看全部
        """
        keys = [name] if name is not None else list(self._states)
        out = []
        for key in keys:
            state = self._states.get(key)
            if state is None:
                continue
            traj = state.trajectory
            if traj is not None and len(traj.waypoints) > 1 and state.joint_names:
                out.append(key)
        return out

    def stop_playback(self) -> None:
        """停止回放。"""
        was_running = self._timer.isActive()
        self._timer.stop()
        self._playing = []
        self.btn_stop.setEnabled(False)
        if self._trajectory and len(self._trajectory.waypoints) > 1:
            self.btn_play.setEnabled(True)
        if was_running:
            self.playbackFinished.emit()

    def _on_tick(self) -> None:
        """推进所有在播机器人的播放头并发射关节帧。"""
        if not getattr(self, "_playing", None):
            self.stop_playback()
            return
        # 到终点的机器人自己退出；全部到点后整体收尾
        self._playing = [name for name in self._playing if self._tick_one(name)]
        if not self._playing:
            self.stop_playback()

    def _tick_one(self, name: str) -> bool:
        """推进 ``name`` 一帧并发给它自己的视口；未到终点返回 ``True``。"""
        state = self._states.get(name)
        traj = state.trajectory if state is not None else None
        if traj is None or not state.joint_names:
            return False

        # 用定时器标称间隔推进（按速度百分比缩放）。定时器本身有抖动，
        # 但对"看一遍路径怎么走"这个目的足够，不值得引入实时时钟补偿的复杂度
        scale = 100.0 / max(self.spin_speed.value(), 1)
        state.playhead += (PLAYBACK_INTERVAL_MS / 1000.0) / scale

        t_end = float(traj.waypoints[-1].timestamp)
        done = state.playhead >= t_end
        if done:
            # 终点处的事件（程序以 gripper 收尾）也要应用
            self._emit_one(name, traj.waypoints[-1].joints)
            if state.gripper_animator is not None:
                self._set_gripper_for(
                    name, state.gripper_animator.state_at(t_end) or ""
                )
            return False

        waypoint = traj.interpolate(state.playhead)
        self._emit_one(name, waypoint.joints)
        # 越过夹爪事件时刻时通知主窗口吸附/释放工件
        if state.gripper_animator is not None:
            self._set_gripper_for(
                name, state.gripper_animator.state_at(state.playhead) or ""
            )
        return True

    def _emit_one(self, name: str, joints_deg: Sequence[float]) -> None:
        """把 ``name`` 这一帧的关节角（度）换算为弧度发给它自己的机器人。"""
        state = self._states[name]
        values = {}
        for i, joint in enumerate(state.joint_names):
            if i < len(joints_deg):
                values[joint] = math.radians(float(joints_deg[i]))
        self.playbackFrame.emit(name, values)

    def _emit_frame(self, joints_deg: Sequence[float]) -> None:
        """兼容旧调用面：给**当前**机器人发一帧。"""
        if self._active is not None and self._active in self._states:
            self._emit_one(self._active, joints_deg)

    def _set_gripper_for(self, name: str, action: str) -> None:
        """广播 ``name`` 这台机器人的夹爪状态（去重，避免每帧刷屏）。"""
        state = self._states.get(name)
        if state is None:
            return
        if action != state.last_gripper_state:
            state.last_gripper_state = action
            self.gripperStateChanged.emit(name, action)

    def _set_gripper_state(self, action: str) -> None:
        """兼容旧调用面：给**当前**机器人设夹爪状态。"""
        if self._active is not None:
            self._set_gripper_for(self._active, action)
