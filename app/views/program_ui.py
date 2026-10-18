"""离线编程面板的界面构建与渲染：布局、控件、报告与指标回填。

本模块只管"长什么样"——所有控件都是 ``ProgramPanel`` 的实例属性
（``panel.combo_robot`` / ``panel.btn_*`` / ``panel.point_list`` …），
业务逻辑在 ``program_validate`` / ``program_editor`` 等模块。信号连接的
目标一律是 ``panel.<方法>`` 薄 stub，由面板转发到对应实现模块。
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from PySide6 import QtGui, QtWidgets

from adapter.post import get_post_processor, list_post_processors
from support.i18n import tr

if TYPE_CHECKING:  # 仅类型标注
    from service.planner import ProgramReport

    from app.views.program_panel import ProgramPanel

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


def _button_grid(buttons, columns=2):
    """把按钮排成**等宽网格**——侧栏里的按钮组一律用它，不要再用单行 QHBoxLayout。

    这个面板在右侧栏里只有 260–360px。一行摆 4–5 个按钮时 ``QHBoxLayout`` 不会
    换行，只会把每个按钮压到 minimumSize 以下（文字被截成几个字甚至只剩色块），
    并且——更糟的——它会把内容**撑到比可视区更宽**（实测最小内容宽约 400px），
    于是右侧一整列被裁到看不见，用户看到的就是"控件重叠/显示不全"。
    网格让它按列数自己排，窄面板里也保持可读。

    :param columns: 每行几个；末行只剩一个时自动跨满整行，不留半边空白。
    """
    grid = QtWidgets.QGridLayout()
    grid.setHorizontalSpacing(4)
    grid.setVerticalSpacing(4)
    span_all = len(buttons) % columns == 1
    for i, btn in enumerate(buttons):
        btn.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        row, col = divmod(i, columns)
        # 末行落单的那个跨满整行
        last = i == len(buttons) - 1
        grid.addWidget(btn, row, col, 1, columns if (last and span_all) else 1)
    return grid


def _build_ui(panel) -> None:
    # 内容自然高度约 798px，而标签页可视高度随窗口变化（1280×800 时仅 725px）。
    # 不套滚动容器会把底部的「后处理与导出」挤出可视区——与右侧属性面板
    # 曾经踩过的坑同源。
    outer = QtWidgets.QVBoxLayout(panel)
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
    panel.combo_robot = QtWidgets.QComboBox()
    panel.combo_robot.setSizeAdjustPolicy(QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToContents)
    panel.combo_robot.currentIndexChanged.connect(panel._on_robot_combo_changed)
    robot_row.addWidget(panel.combo_robot, 1)
    root.addLayout(robot_row)

    # --- 节拍与规模 ---
    box = QtWidgets.QGroupBox(tr("程序概览"))
    form = QtWidgets.QFormLayout(box)
    panel.lbl_status = QtWidgets.QLabel(tr("未加载机器人"))
    panel.lbl_status.setObjectName("bad")
    form.addRow(tr("状态"), panel.lbl_status)
    panel.lbl_cycle = QtWidgets.QLabel("—")
    panel.lbl_cycle.setObjectName("metric")
    form.addRow(tr("估算节拍"), panel.lbl_cycle)
    panel.lbl_length = QtWidgets.QLabel("—")
    panel.lbl_length.setObjectName("metric")
    form.addRow(tr("TCP 路径"), panel.lbl_length)
    panel.lbl_points = QtWidgets.QLabel("—")
    form.addRow(tr("路径点"), panel.lbl_points)
    root.addWidget(box)

    # --- 操作 ---
    ops = QtWidgets.QGroupBox(tr("操作"))
    ops_layout = QtWidgets.QVBoxLayout(ops)

    speed_row = QtWidgets.QHBoxLayout()
    speed_row.addWidget(QtWidgets.QLabel(tr("进给速度")))
    panel.spin_speed = QtWidgets.QSpinBox()
    panel.spin_speed.setRange(1, 100)
    panel.spin_speed.setValue(100)
    panel.spin_speed.setSuffix(" %")
    speed_row.addWidget(panel.spin_speed, 1)
    ops_layout.addLayout(speed_row)

    # 「校验程序 / 回放」这组**单机**按钮删掉了：和「校验全部 / 全部回放」
    # 完全重复——只有一台时"全部"就是那一台，两排按钮只会让人疑惑该点哪个
    # （2026-09-21 用户反馈）。F7 / Shift+F7 快捷键保留，走主窗口菜单。
    # 顺序按用户要求：校验全部 → 全部回放 → 停止 → 抓取向导。
    panel.btn_validate_all = QtWidgets.QPushButton(tr("校验全部"))
    panel.btn_validate_all.setToolTip(
        tr("逐台校验所有机器人并汇总结果——不必一台一台切过去按 F7")
    )
    panel.btn_validate_all.clicked.connect(lambda: panel.validate_all())
    panel.btn_play_all = QtWidgets.QPushButton(tr("全部回放"))
    panel.btn_play_all.setToolTip(tr("所有已校验的机器人同时回放——检查多机协同是否撞车"))
    # ``clicked`` 带一个 bool 参数：**不能**直接连到带可选参数的槽——
    # ``start_playback_all()`` 这类槽一律无参，或用 lambda 吃掉 checked。
    panel.btn_play_all.clicked.connect(panel.start_playback_all)
    panel.btn_play_all.setEnabled(False)
    panel.btn_stop = QtWidgets.QPushButton(tr("停止"))
    panel.btn_stop.clicked.connect(panel.stop_playback)
    panel.btn_stop.setEnabled(False)
    # 单机也要有：示教是逐台做的活，跟有几台机器人无关
    panel.btn_grasp_wizard = QtWidgets.QPushButton(tr("抓取向导…"))
    panel.btn_grasp_wizard.setToolTip(
        tr("由抓取点/放置点自动生成接近、提升、撤离点——不必逐个手动示教")
    )
    panel.btn_grasp_wizard.clicked.connect(lambda: panel.open_grasp_wizard())
    ops_layout.addLayout(
        _button_grid(
            [
                panel.btn_validate_all,
                panel.btn_play_all,
                panel.btn_stop,
                panel.btn_grasp_wizard,
            ]
        )
    )
    root.addWidget(ops)

    # --- 轨迹编辑器（v0.2.4）：路点增删移改 + 运动类型切换 ---
    editor = QtWidgets.QGroupBox(tr("轨迹编辑"))
    editor_layout = QtWidgets.QVBoxLayout(editor)

    panel.point_list = QtWidgets.QListWidget()
    panel.point_list.setAlternatingRowColors(True)
    panel.point_list.setWordWrap(True)
    panel.point_list.itemDoubleClicked.connect(lambda _item: panel.edit_selected_point())
    editor_layout.addWidget(panel.point_list, 1)

    panel.btn_insert_point = QtWidgets.QPushButton(tr("插入"))
    panel.btn_insert_point.setToolTip(tr("在选中点之后插入一个路点（复制其关节值）"))
    # 同 btn_play：把 ``clicked`` 的 bool 吃掉，否则 False 会被当成第 0 行
    panel.btn_insert_point.clicked.connect(lambda _checked=False: panel.insert_point())
    panel.btn_delete_point = QtWidgets.QPushButton(tr("删除"))
    panel.btn_delete_point.clicked.connect(lambda _checked=False: panel.delete_point())
    panel.btn_edit_point = QtWidgets.QPushButton(tr("编辑位姿…"))
    panel.btn_edit_point.setToolTip(tr("修改选中路点的关节角（度）与运动类型（双击同效）"))
    panel.btn_edit_point.clicked.connect(panel.edit_selected_point)
    editor_layout.addLayout(
        _button_grid([panel.btn_insert_point, panel.btn_delete_point, panel.btn_edit_point])
    )

    panel.btn_move_up = QtWidgets.QPushButton(tr("上移"))
    panel.btn_move_up.clicked.connect(lambda: panel.shift_selected_point(-1))
    panel.btn_move_down = QtWidgets.QPushButton(tr("下移"))
    panel.btn_move_down.clicked.connect(lambda: panel.shift_selected_point(1))
    # 运动类型（moveJ/moveL/moveC）**不在这里改**——面板只留位置调整。
    # 要改指令类型去「编辑位姿…」对话或直接在路径代码里改：放在列表旁一排
    # 按钮反而容易误点，且与代码里的 ``movej`` 行两处可改、互不一致。
    editor_layout.addLayout(_button_grid([panel.btn_move_up, panel.btn_move_down]))

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
    panel.issue_list = QtWidgets.QListWidget()
    panel.issue_list.setAlternatingRowColors(True)
    panel.issue_list.setWordWrap(True)
    issues_layout.addWidget(panel.issue_list)
    root.addWidget(issues, 1)

    # --- 设备联调：后处理 / 导出 / 真机（预留） ---
    comm = QtWidgets.QGroupBox(tr("后处理与导出"))
    comm_layout = QtWidgets.QVBoxLayout(comm)

    pp_row = QtWidgets.QHBoxLayout()
    pp_row.addWidget(QtWidgets.QLabel(tr("后处理器")))
    panel.combo_post = QtWidgets.QComboBox()
    panel.combo_post.addItem(tr("自动（按机器人品牌）"), None)
    for name in list_post_processors():
        pp = get_post_processor(name)
        panel.combo_post.addItem(f"{pp.brand.upper()} {pp.controller.upper()}", name)
    pp_row.addWidget(panel.combo_post, 1)
    comm_layout.addLayout(pp_row)

    panel.btn_generate = QtWidgets.QPushButton(tr("生成代码"))
    panel.btn_generate.clicked.connect(panel.generate_code)
    panel.btn_generate.setEnabled(False)
    # 「一键导出…」（只导当前那台）去掉了：实际产线是多台的，逐个点太笨
    # （2026-09-21 用户反馈）。导出入口统一为「导出全部…」：每台机器人
    # 各落一份**独立程序文件**——各自的品牌扩展名、各自的机器人名，
    # 不是拼成一个大文件。
    panel.btn_export_all = QtWidgets.QPushButton(tr("导出全部…"))
    panel.btn_export_all.setToolTip(
        tr(
            "为每一台机器人各落一份独立程序文件到同一目录"
            "（UR .script / KUKA .src / ABB .mod / FANUC .tp）"
        )
    )
    panel.btn_export_all.clicked.connect(panel.export_all_programs)
    panel.btn_export_all.setEnabled(False)
    comm_layout.addLayout(_button_grid([panel.btn_generate, panel.btn_export_all]))

    panel.code_preview = QtWidgets.QTextEdit()
    panel.code_preview.setReadOnly(True)
    panel.code_preview.setLineWrapMode(QtWidgets.QTextEdit.NoWrap)
    mono = QtGui.QFont("Consolas", 9)
    mono.setStyleHint(QtGui.QFont.Monospace)
    panel.code_preview.setFont(mono)
    panel.code_preview.setFixedHeight(150)
    panel.code_preview.setPlaceholderText(tr("生成的控制器代码将显示在这里"))
    comm_layout.addWidget(panel.code_preview)

    # 「上传真机 / 真机验证」：从未上过真机验证，MVP 档**藏掉**而不是置灰
    # ——灰色按钮同样会被当成产品能力（2026-09-21 用户反馈：等 MVP 做完
    # 再开启）。CORE_FEATURES=full 或放开清单即恢复。
    # 门控必须查 program_panel 的模块字典（迟绑定）：既有测试把补丁打在
    # pp_mod._feature_enabled 上，查本模块自己的全局会让补丁静默失效。
    from app.views import program_panel as _pp

    if _pp._feature_enabled("real_machine_upload"):
        panel.btn_upload = QtWidgets.QPushButton(tr("上传真机"))
        panel.btn_upload.setEnabled(False)
        panel.btn_upload.setToolTip(tr("预留：接入 ControllerLink 后支持直连控制器上传"))
        panel.btn_verify = QtWidgets.QPushButton(tr("真机验证"))
        panel.btn_verify.setEnabled(False)
        panel.btn_verify.setToolTip(tr("预留：接入 ControllerLink.read_joints 后做真机回读比对"))
        comm_layout.addLayout(_button_grid([panel.btn_upload, panel.btn_verify]))

    root.addWidget(comm)


def _show_report(panel, report: "ProgramReport") -> None:
    """把报告填进界面。"""
    panel.lbl_cycle.setText(
        f"{report.cycle_time:.2f} s" if report.cycle_time is not None else "—"
    )
    panel.lbl_length.setText(f"{report.path_length:.1f} mm")
    panel.lbl_points.setText(
        tr("{w} 个 / {m} 条指令").format(w=report.num_waypoints, m=report.num_motions)
    )

    panel.issue_list.clear()
    if report.ok and not report.warnings:
        item = QtWidgets.QListWidgetItem(tr("✓ 程序校验通过，未发现问题"))
        item.setForeground(QtGui.QColor("#6fdc6f"))
        panel.issue_list.addItem(item)
        return

    for issue in report.errors:
        item = QtWidgets.QListWidgetItem(str(issue))
        item.setForeground(QtGui.QColor("#ff8080"))
        item.setToolTip(issue.message)
        panel.issue_list.addItem(item)
    for issue in report.warnings:
        item = QtWidgets.QListWidgetItem(str(issue))
        item.setForeground(QtGui.QColor("#ffc866"))
        item.setToolTip(issue.message)
        panel.issue_list.addItem(item)


def _clear_metrics(panel) -> None:
    panel.lbl_cycle.setText("—")
    panel.lbl_length.setText("—")
    panel.lbl_points.setText("—")


def _display_active(panel) -> None:
    """按当前机器人的状态回填整个界面（点表 / 报告 / 代码 / 按钮可用性）。"""
    state = panel._state()
    panel._refresh_point_list()
    panel.code_preview.setPlainText(state.last_code)

    if state.report is None:
        panel.issue_list.clear()
        panel._clear_metrics()
    else:
        panel._show_report(state.report)
        panel._apply_point_colors(state.report)

    # 「校验全部」只要登记过机器人就能点（没示教的那台会被跳过并如实说明）
    if getattr(panel, "btn_validate_all", None) is not None:
        panel.btn_validate_all.setEnabled(bool(panel._states))
    # 「全部回放」只要有一台能播就该能点
    if getattr(panel, "btn_play_all", None) is not None:
        panel.btn_play_all.setEnabled(bool(panel._playable()))
    panel.btn_generate.setEnabled(state.robot is not None)
    # 「导出全部」看的是全局：任何一台有程序就该能点（它会跳过没示教的）
    panel._refresh_export_all_enabled()

    if state.robot is None:
        panel.lbl_status.setText(tr("未加载机器人"))
        panel.lbl_status.setObjectName("bad")
    else:
        panel.lbl_status.setText(tr("{name} · 就绪").format(name=panel._active))
        panel.lbl_status.setObjectName("ok")
    panel.lbl_status.style().polish(panel.lbl_status)


def _refresh_robot_combo(panel, selected: Optional[str] = None) -> None:
    """把已登记的机器人名填进下拉，并选中 ``selected``（或当前这台）。

    重建时必须屏蔽信号：填项过程会让 ``currentIndex`` 跳来跳去，触发
    ``_on_robot_combo_changed`` 会在回填途中反复切换机器人。
    """
    if getattr(panel, "combo_robot", None) is None:
        return
    target = selected or panel._active
    panel.combo_robot.blockSignals(True)
    panel.combo_robot.clear()
    for name in panel._states:
        panel.combo_robot.addItem(name, name)
    if target is not None:
        idx = panel.combo_robot.findData(target)
        if idx >= 0:
            panel.combo_robot.setCurrentIndex(idx)
    panel.combo_robot.blockSignals(False)
    # 只有一台时下拉没什么可挑，藏起来保持单机界面的样子。
    # 「校验全部 / 全部回放」**单机也显示**：它们就是校验/回放入口本身，
    # 只有依据"有几台"决定藏不藏会让单机用户找不到按钮
    multi = len(panel._states) > 1
    panel.combo_robot.setVisible(multi)
    if getattr(panel, "btn_validate_all", None) is not None:
        panel.btn_validate_all.setEnabled(bool(panel._states))
    if getattr(panel, "btn_play_all", None) is not None:
        panel.btn_play_all.setEnabled(bool(panel._playable()))
    if getattr(panel, "btn_grasp_wizard", None) is not None:
        panel.btn_grasp_wizard.setEnabled(bool(panel._states))


def _refresh_export_all_enabled(panel) -> None:
    """「导出全部…」的可用性：任何一台有程序就该能点（它会跳过没示教的）。"""
    if getattr(panel, "btn_export_all", None) is None:
        return
    panel.btn_export_all.setEnabled(
        any(s.robot is not None and s.taught for s in panel._states.values())
    )
