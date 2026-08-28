"""离线编程面板：程序校验、节拍估算、轨迹回放。

面板本身不含业务逻辑，只负责把 :class:`~orolp.planner.TrajectoryPlanner`
的结果呈现出来：

* **估算节拍** —— OLP 的核心卖点。客户拿它报价、算产线产能
* **问题清单** —— 不可达 / 越限 / 奇异 / 翻腕 / 干涉，带指令定位
* **真实插补回放** —— 按插补后的密集轨迹播放，而不是在示教点之间瞬移。
  这两者的区别正是"直线插补"与"关节乱跳"

单位：面板内部一律 **度**（与 :class:`~orolp.planner` 一致）；向视口发送
回放帧时换算为 **弧度**（``GLWidget.renderer.joint_values`` 用弧度）。
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Dict, List, Optional, Sequence

from PySide6 import QtCore, QtGui, QtWidgets

from .planner import ProgramReport, TrajectoryPlanner
from .program import Program, ProgramBuilder

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
    margin-top: 10px; padding-top: 8px; font-weight: bold;
}
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
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


class ProgramPanel(QtWidgets.QWidget):
    """离线编程面板。

    Signals:
        statusMessage(str): 适合放进状态栏的提示
        playbackFrame(dict): 回放帧，``{关节名: 弧度}``
        playbackFinished(): 回放结束
        reportReady(object): :class:`ProgramReport`，供其他面板复用
    """

    statusMessage = QtCore.Signal(str)
    playbackFrame = QtCore.Signal(dict)
    playbackFinished = QtCore.Signal()
    reportReady = QtCore.Signal(object)

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.setStyleSheet(PANEL_STYLE)

        self._robot: Optional["Robot"] = None
        self._scene: Optional["Scene"] = None
        self._joint_names: List[str] = []
        self._taught: List[Dict[str, float]] = []
        self._trajectory: Optional["Trajectory"] = None
        self._report: Optional[ProgramReport] = None

        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(PLAYBACK_INTERVAL_MS)
        self._timer.timeout.connect(self._on_tick)
        self._playhead = 0.0

        self._build_ui()

    # ---------------------------- 界面 ---------------------------- #

    def _build_ui(self) -> None:
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)

        # --- 节拍与规模 ---
        box = QtWidgets.QGroupBox("程序概览")
        form = QtWidgets.QFormLayout(box)
        self.lbl_status = QtWidgets.QLabel("未加载机器人")
        self.lbl_status.setObjectName("bad")
        form.addRow("状态", self.lbl_status)
        self.lbl_cycle = QtWidgets.QLabel("—")
        self.lbl_cycle.setObjectName("metric")
        form.addRow("估算节拍", self.lbl_cycle)
        self.lbl_length = QtWidgets.QLabel("—")
        self.lbl_length.setObjectName("metric")
        form.addRow("TCP 路径", self.lbl_length)
        self.lbl_points = QtWidgets.QLabel("—")
        form.addRow("路径点", self.lbl_points)
        root.addWidget(box)

        # --- 操作 ---
        ops = QtWidgets.QGroupBox("离线编程")
        ops_layout = QtWidgets.QVBoxLayout(ops)

        speed_row = QtWidgets.QHBoxLayout()
        speed_row.addWidget(QtWidgets.QLabel("进给速度"))
        self.spin_speed = QtWidgets.QSpinBox()
        self.spin_speed.setRange(1, 100)
        self.spin_speed.setValue(100)
        self.spin_speed.setSuffix(" %")
        speed_row.addWidget(self.spin_speed, 1)
        ops_layout.addLayout(speed_row)

        btn_row = QtWidgets.QHBoxLayout()
        self.btn_validate = QtWidgets.QPushButton("校验程序 (F7)")
        self.btn_validate.clicked.connect(self.validate_program)
        btn_row.addWidget(self.btn_validate)
        self.btn_play = QtWidgets.QPushButton("回放")
        self.btn_play.clicked.connect(self.start_playback)
        self.btn_play.setEnabled(False)
        btn_row.addWidget(self.btn_play)
        self.btn_stop = QtWidgets.QPushButton("停止")
        self.btn_stop.clicked.connect(self.stop_playback)
        self.btn_stop.setEnabled(False)
        btn_row.addWidget(self.btn_stop)
        ops_layout.addLayout(btn_row)
        root.addWidget(ops)

        # --- 问题清单 ---
        issues = QtWidgets.QGroupBox("校验结果")
        issues_layout = QtWidgets.QVBoxLayout(issues)
        self.issue_list = QtWidgets.QListWidget()
        self.issue_list.setAlternatingRowColors(True)
        self.issue_list.setWordWrap(True)
        issues_layout.addWidget(self.issue_list)
        root.addWidget(issues, 1)

        self.btn_validate.setEnabled(False)

    # ---------------------------- 外部注入 ---------------------------- #

    def set_robot(self, robot: Optional["Robot"], joint_names: Optional[Sequence[str]] = None) -> None:
        """设置当前机器人。

        Args:
            robot: 机器人实例；``None`` 表示卸载
            joint_names: 示教点字典里用的关节名（视口渲染器的命名，可能比
                可驱动关节多——URDF 里的 ``fixed`` 关节，如 ``tcp_joint``，
                也会被渲染器登记）。为 ``None`` 时取 ``robot.joint_names``

        Note:
            这里必须把关节名过滤到**可驱动**的那些。渲染器会把 fixed 关节
            一并登记，照单全收的话 ``moveJ`` 会收到多出来的分量，直接被
            自由度校验拒掉——表现为"每条指令都失败"，极难排查。
        """
        self._robot = robot
        self.stop_playback()
        self._trajectory = None
        self._report = None
        self.issue_list.clear()
        if robot is None:
            self._joint_names = []
            self.lbl_status.setText("未加载机器人")
            self.lbl_status.setObjectName("bad")
            self.btn_validate.setEnabled(False)
            self.btn_play.setEnabled(False)
            self._clear_metrics()
        else:
            source = list(joint_names or robot.joint_names)
            actuated = list(robot.joint_names)
            known = set(actuated)
            filtered = [n for n in source if n in known]
            # 命名对不上时宁可退回权威列表，也不要留下长度不匹配的隐患
            self._joint_names = filtered if len(filtered) == len(actuated) else actuated
            self.lbl_status.setText(f"{robot.name} · 就绪")
            self.lbl_status.setObjectName("ok")
        # 重新套用样式表，objectName 改变后需手动刷新
        self.lbl_status.style().polish(self.lbl_status)

    def set_scene(self, scene: Optional["Scene"]) -> None:
        """绑定工作单元场景；给出后校验会包含沿路径干涉检查。"""
        self._scene = scene

    def set_taught_points(self, positions: Sequence[Dict[str, float]]) -> None:
        """导入示教点（``{关节名: 弧度}``）。"""
        self._taught = [dict(p) for p in positions]
        n = len(self._taught)
        self.statusMessage.emit(f"导入 {n} 个示教点")
        if self._robot is not None:
            self.btn_validate.setEnabled(n >= 1)

    # ---------------------------- 业务逻辑 ---------------------------- #

    @property
    def trajectory(self) -> Optional["Trajectory"]:
        """最近一次展开的轨迹。"""
        return self._trajectory

    @property
    def report(self) -> Optional[ProgramReport]:
        """最近一次校验报告。"""
        return self._report

    def build_program(self) -> Optional[Program]:
        """把示教点组装成程序。

        示教点内部以弧度保存，:class:`Motion` 期望角度。

        Returns:
            :class:`Program`；无机器人或无示教点时返回 ``None``
        """
        if self._robot is None or not self._taught:
            return None
        speed = float(self.spin_speed.value())
        builder = ProgramBuilder(self._robot)
        for pos in self._taught:
            joints = [math.degrees(float(pos.get(n, 0.0))) for n in self._joint_names]
            builder.moveJ(joints, speed=speed)
        return builder.build()

    def validate_program(self) -> Optional[ProgramReport]:
        """校验程序并刷新界面。

        Returns:
            :class:`ProgramReport`；条件不足时返回 ``None``
        """
        if self._robot is None:
            self.statusMessage.emit("请先载入机器人型号，再校验程序")
            return None

        program = self.build_program()
        if program is None:
            self.statusMessage.emit("没有示教点可供校验——请先记录若干关节位形")
            return None

        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            report = program.validate(scene=self._scene)
            self._trajectory = TrajectoryPlanner(
                self._robot, scene=self._scene
            ).plan(program)
        except Exception as exc:
            # 校验本身崩了也要让用户看见——这类异常几乎总是模型数据的问题
            self.statusMessage.emit(f"校验异常：{type(exc).__name__}: {exc}")
            return None
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()

        self._report = report
        self._show_report(report)
        self.reportReady.emit(report)

        # 有轨迹就能回放——即使校验没通过，也允许用户看看走到哪儿崩的
        self.btn_play.setEnabled(bool(self._trajectory and len(self._trajectory.waypoints) > 1))
        self.statusMessage.emit(
            f"校验完成：{'通过' if report.ok else f'{len(report.errors)} 处错误'}"
        )
        return report

    def _show_report(self, report: ProgramReport) -> None:
        """把报告填进界面。"""
        self.lbl_cycle.setText(
            f"{report.cycle_time:.2f} s" if report.cycle_time is not None else "—"
        )
        self.lbl_length.setText(f"{report.path_length:.1f} mm")
        self.lbl_points.setText(f"{report.num_waypoints} 个 / {report.num_motions} 条指令")

        self.issue_list.clear()
        if report.ok and not report.warnings:
            item = QtWidgets.QListWidgetItem("✓ 程序校验通过，未发现问题")
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

    def start_playback(self) -> None:
        """从轨迹起点开始回放。"""
        traj = self._trajectory
        if traj is None or len(traj.waypoints) < 2:
            # 不用模态弹窗：无头/自动化场景下会永久阻塞，且"还没校验"只是
            # 普通状态提示，不值得打断用户
            self.statusMessage.emit("请先校验程序以生成轨迹，再回放")
            return
        self._playhead = float(traj.waypoints[0].timestamp)
        self.btn_play.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self._timer.start()
        self.statusMessage.emit("轨迹回放中…")

    def stop_playback(self) -> None:
        """停止回放。"""
        was_running = self._timer.isActive()
        self._timer.stop()
        self.btn_stop.setEnabled(False)
        if self._trajectory and len(self._trajectory.waypoints) > 1:
            self.btn_play.setEnabled(True)
        if was_running:
            self.playbackFinished.emit()

    def _on_tick(self) -> None:
        """推进播放头并发射关节帧。"""
        traj = self._trajectory
        if traj is None or not self._joint_names:
            self.stop_playback()
            return

        # 用定时器标称间隔推进（按速度百分比缩放）。定时器本身有抖动，
        # 但对"看一遍路径怎么走"这个目的足够，不值得引入实时时钟补偿的复杂度
        scale = 100.0 / max(self.spin_speed.value(), 1)
        self._playhead += (PLAYBACK_INTERVAL_MS / 1000.0) / scale

        t_end = float(traj.waypoints[-1].timestamp)
        if self._playhead >= t_end:
            self._emit_frame(traj.waypoints[-1].joints)
            self.stop_playback()
            return

        waypoint = traj.interpolate(self._playhead)
        self._emit_frame(waypoint.joints)

    def _emit_frame(self, joints_deg: Sequence[float]) -> None:
        """把关节角（度）换算为弧度并发射给视口。"""
        values = {}
        for i, name in enumerate(self._joint_names):
            if i < len(joints_deg):
                values[name] = math.radians(float(joints_deg[i]))
        self.playbackFrame.emit(values)
