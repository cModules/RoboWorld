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
from pathlib import Path
from typing import TYPE_CHECKING, Dict, List, Optional, Sequence, Tuple

from PySide6 import QtCore, QtGui, QtWidgets

from .planner import ProgramReport, SEVERITY_ERROR, TrajectoryPlanner
from .pickplace import PointType, gripper_action_for
from .program import Program, ProgramBuilder
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


class ProgramPanel(QtWidgets.QWidget):
    """离线编程面板。

    Signals:
        statusMessage(str): 适合放进状态栏的提示
        playbackFrame(dict): 回放帧，``{关节名: 弧度}``
        playbackFinished(): 回放结束
        gripperStateChanged(str): 夹爪状态变化（``"close"/"open"/""``，
            空串表示回放复位、未夹持），供主窗口驱动工件吸附/释放
        reportReady(object): :class:`ProgramReport`，供其他面板复用
    """

    statusMessage = QtCore.Signal(str)
    playbackFrame = QtCore.Signal(dict)
    playbackFinished = QtCore.Signal()
    gripperStateChanged = QtCore.Signal(str)
    reportReady = QtCore.Signal(object)
    #: 轨迹编辑器改动示教点后广播：``(positions, types, motions)``
    #: positions 为 ``{关节名: 弧度}`` 字典列表；types 为工艺语义（可为 ``None``）；
    #: motions 为与点位对齐的运动类型（``"J"``/``"L"``）。主窗口据此把改动
    #: 回写到路径代码编辑器，保持两条编辑通路数据一致
    taughtPointsChanged = QtCore.Signal(list, list, list)

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.setStyleSheet(PANEL_STYLE)

        self._robot: Optional["Robot"] = None
        self._scene: Optional["Scene"] = None
        self._joint_names: List[str] = []
        self._taught: List[Dict[str, float]] = []
        #: 与 ``_taught`` 对齐的示教点工艺语义（PICK/PLACE…）；``None`` 表示无语义
        self._point_types: Optional[List[PointType]] = None
        #: 与 ``_taught`` 对齐的运动类型（``"J"``=moveJ / ``"L"``=moveL /
        #: ``"C"``=moveC 圆弧）。轨迹编辑器（v0.2.4）允许逐点切换运动类型，
        #: v1.1.5 起支持圆弧
        self._point_motions: List[str] = []
        #: 最近一次构建程序时的降级说明（圆弧缺前驱点/相邻冲突等）——
        #: 绝不静默改变几何，构建过程里发生的事都要能被界面说出来
        self._build_notes: List[str] = []
        self._trajectory: Optional["Trajectory"] = None
        self._report: Optional[ProgramReport] = None
        #: 最近一次校验的规划器——保留 IO 事件映射所需的指令→时间区间
        self._planner: Optional[TrajectoryPlanner] = None
        #: 抓取动画驱动器（按轨迹时间给出夹爪状态）
        self._gripper_animator: Optional[GraspAnimator] = None
        self._last_gripper_state: str = ""
        #: 校验报告映射到路点的最严重问题级别（``{路点序号: "error"/"warning"}``）
        self._point_issue_level: Dict[int, str] = {}
        #: 上一次导入的示教点数量——点数变化时旧报告作废
        self._last_point_count: Optional[int] = None

        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(PLAYBACK_INTERVAL_MS)
        self._timer.timeout.connect(self._on_tick)
        self._playhead = 0.0
        self._last_code: str = ""

        self._build_ui()

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
            自由度校验拒掉——表现为tr("每条指令都失败")，极难排查。
        """
        self._robot = robot
        self.stop_playback()
        self._trajectory = None
        self._report = None
        self.issue_list.clear()
        if robot is None:
            self._joint_names = []
            self.lbl_status.setText(tr("未加载机器人"))
            self.lbl_status.setObjectName("bad")
            self.btn_validate.setEnabled(False)
            self.btn_play.setEnabled(False)
            self.btn_generate.setEnabled(False)
            self.btn_export.setEnabled(False)
            self._clear_metrics()
        else:
            source = list(joint_names or robot.joint_names)
            actuated = list(robot.joint_names)
            known = set(actuated)
            filtered = [n for n in source if n in known]
            # 命名对不上时宁可退回权威列表，也不要留下长度不匹配的隐患
            self._joint_names = filtered if len(filtered) == len(actuated) else actuated
            self.lbl_status.setText(tr("{name} · 就绪").format(name=robot.name))
            self.lbl_status.setObjectName("ok")
            self.btn_generate.setEnabled(True)
            # 「一键导出」的门槛与「生成代码」完全相同——它内部自己构建程序
            # （见 export_program 的文档），不该逼用户先点一次「生成代码」才亮。
            self.btn_export.setEnabled(True)
        # 重新套用样式表，objectName 改变后需手动刷新
        self.lbl_status.style().polish(self.lbl_status)

    def set_scene(self, scene: Optional["Scene"]) -> None:
        """绑定工作单元场景；给出后校验会包含沿路径干涉检查。"""
        self._scene = scene

    def set_taught_points(
        self,
        positions: Sequence[Dict[str, float]],
        types: Optional[Sequence[PointType]] = None,
        motions: Optional[Sequence[str]] = None,
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
        """
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
        self.statusMessage.emit(tr("导入 {n} 个示教点").format(n=n))
        self._refresh_point_list()
        if self._robot is not None:
            self.btn_validate.setEnabled(n >= 1)

    # ---------------------------- 轨迹编辑器 ---------------------------- #

    def get_taught_points(self) -> tuple:
        """读取当前示教点数据（供外部同步与测试）。

        Returns:
            ``(positions, types, motions)``——positions 为
            ``{关节名: 弧度}`` 字典列表的拷贝；types 可为 ``None``；
            motions 为 ``"J"/"L"/"C"`` 列表
        """
        positions = [dict(p) for p in self._taught]
        types = list(self._point_types) if self._point_types is not None else None
        return positions, types, list(self._point_motions)

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
        self._point_issue_level = levels
        self._refresh_point_list()

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
        try:
            code = program.to_post(
                choice, metadata={"program_name": self._robot.name or "program"}
            )
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

    def export_program_to(self, program: Program, dest: str) -> bool:
        """把 ``program`` 导出到 ``dest`` 目录（无 UI 逻辑，供测试复用）。

        Returns:
            成功 ``True``；后处理/落盘失败 ``False``（状态栏带原因）
        """
        name = self._robot.name or "program"
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

    def start_playback(self) -> None:
        """从轨迹起点开始回放。"""
        traj = self._trajectory
        if traj is None or len(traj.waypoints) < 2:
            # 不用模态弹窗：无头/自动化场景下会永久阻塞，且"还没校验"只是
            # 普通状态提示，不值得打断用户
            self.statusMessage.emit(tr("请先校验程序以生成轨迹，再回放"))
            return
        self._playhead = float(traj.waypoints[0].timestamp)
        self.btn_play.setEnabled(False)
        self.btn_stop.setEnabled(True)
        # 回放从头开始：夹爪状态复位为未夹持，主窗口会据此放下工件
        if self._gripper_animator is not None:
            self._gripper_animator.reset()
        self._set_gripper_state("")
        self._timer.start()
        self.statusMessage.emit(tr("轨迹回放中…"))

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
            # 终点处的事件（程序以 gripper 收尾）也要应用
            if self._gripper_animator is not None:
                self._set_gripper_state(
                    self._gripper_animator.state_at(t_end) or ""
                )
            self.stop_playback()
            return

        waypoint = traj.interpolate(self._playhead)
        self._emit_frame(waypoint.joints)
        # 越过关爪事件时刻时通知主窗口吸附/释放工件
        if self._gripper_animator is not None:
            self._set_gripper_state(
                self._gripper_animator.state_at(self._playhead) or ""
            )

    def _emit_frame(self, joints_deg: Sequence[float]) -> None:
        """把关节角（度）换算为弧度并发射给视口。"""
        values = {}
        for i, name in enumerate(self._joint_names):
            if i < len(joints_deg):
                values[name] = math.radians(float(joints_deg[i]))
        self.playbackFrame.emit(values)

    def _set_gripper_state(self, action: str) -> None:
        """夹爪状态去重后广播（避免每帧重复发射刷屏）。"""
        if action != self._last_gripper_state:
            self._last_gripper_state = action
            self.gripperStateChanged.emit(action)
