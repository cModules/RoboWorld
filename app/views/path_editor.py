"""路径编辑器视图（ADR-002 拆解：``PathCodeEditor`` → ``app.views``）。

归属：App / GUI 层 ``app.views``。它是**示教点表 + 代码编辑器**——把 ``movej/movel``
代码解析成示教点、按点播放、生成 PLC 与机器人子程序；点表、归属机器人、行号映射
这些状态都由本类自己维护，主窗口只注入场景与机器人。

依赖：``app → service / domain``（``CellConfig`` · ``Device`` · ``SignalMap`` ·
``PointType`` · ``build_pick_place`` · ``generate_scl`` · ``steps_from_program`` ·
``COMPONENTS``）与 ``support.i18n`` / ``app.features``；**不 import main**
（方向单向：入口脚本 import 本模块）。

搬迁纪律：只换位置、不改行为（ADR-002 禁止"顺手重构"）。``service`` 侧的三个可选依赖
沿用与 ``main.py`` 同口径的降级块（``_HAS_CELL`` / ``_HAS_PICKPLACE``）——**重复是暂时的**：
等 ``MainWindow`` 也搬进 ``app/mainwindow/`` 后，这些降级块应收敛到一处共享模块
（已记入 ``docs/develop/BUILD_QUEUE.md``）。
"""

from __future__ import annotations

import math
import re
from typing import List, Optional, Tuple

from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont

from app.features import enabled as _feature_enabled
from app.logging import logger
from support.i18n import tr

# 环境建模的组件目录（与 main.py 同口径：缺失时置 None，不阻塞导入）
try:
    from domain.world.components import COMPONENTS
except Exception:  # pragma: no cover - 与 main.py 的 _HAS_WORLD 块同口径
    COMPONENTS = None

# 工作单元模型 + PLC（西门子 SCL）代码生成（main.py 同口径 → _HAS_CELL）
try:
    from service.cell import CellConfig, Device
    from service.iomap import SignalMap
    from service.plc_scl import generate_scl, steps_from_program

    _HAS_CELL = True
except Exception:  # pragma: no cover - 依赖缺失时降级
    _HAS_CELL = False
    CellConfig = None
    Device = None
    SignalMap = None
    generate_scl = None
    steps_from_program = None

# 抓取工艺（示教点语义 → IO 指令 + 吸附事件）（main.py 同口径 → _HAS_PICKPLACE）
try:
    from service.pickplace import PointType, build_pick_place

    _HAS_PICKPLACE = True
except Exception:  # pragma: no cover - 依赖缺失时降级
    _HAS_PICKPLACE = False
    PointType = None
    build_pick_place = None


class PathCodeEditor(QtWidgets.QWidget):
    positionRecorded = Signal(int)
    #: 参数依次是**点索引 / 这个点归属哪台机器人**。机器人名不能省：全局运行
    #: 时一行一个主人，只给索引的话主窗口只能拿"当前那台"去顶，表现为 A 的
    #: 指令把 B 拽着走
    playbackRequested = Signal(int, str)
    playbackStopped = Signal()
    clearRequested = Signal()
    positionsParsed = Signal()  # 新增：代码解析完成信号
    #: 状态栏提示。本类是普通 QWidget、自己没有状态栏，主窗口不在时靠这条
    #: 信号把提示送出去（无接收者时静默丢弃，不会抛异常）
    statusMessage = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.recorded_positions = []
        #: 与 recorded_positions **一一对应的归属机器人名**；``None`` 表示这段
        #: 没标归属（旧工程 / 手打的代码）。路径编辑器一份点表里可能同时装
        #: 着多台机器人的路径——给 B 记点不能把 A 的整段顶掉，"分别执行各台
        #: 的路径"靠的就是这条平行数组。
        self.point_robots: List[Optional[str]] = []
        self.joint_names = []  # 存储关节名称顺序
        self.playback_timer = None
        self.current_playback_index = 0
        self.playback_speed = 50
        #: 与 recorded_positions 一一对应的工艺语义（PointType）。
        #: 抓取点/放置点会在播放时触发工件吸附与释放，并在生成代码时插入 IO 指令
        self.point_types = []
        #: 这一串示教点**属于哪台机器人**。路径编辑器只有一份点表，而视口里
        #: 点选/场景树选中都会切换"当前机器人"却不换点表——播放时必须认这个
        #: 归属，否则播 A 的程序会把 B 拽着走
        self.owner_robot = None
        #: 每条指令的**行号与归属**：``[(行号, 机器人名或 None, 点索引)]``，由
        #: :meth:`_parse_code_to_positions` 建立。"从第几行起跑""只跑选中那台"
        #: 全靠它——光有点值序列是定位不了的
        self._line_map: List[Tuple[int, Optional[str], int]] = []
        #: 本次播放要走的**点索引序列**（已按"只跑某台 / 从光标行起"筛过）
        self._playback_seq: List[int] = []
        #: 手改代码后的**防抖重解析**定时器：边打字边解析既卡，又会把半截
        #: 语法（写到一半的 movej(）灌进点表。停手 300ms 再解析一次
        self._edit_parse_timer = QtCore.QTimer(self)
        self._edit_parse_timer.setSingleShot(True)
        self._edit_parse_timer.setInterval(300)
        self._edit_parse_timer.timeout.connect(self._flush_code_edit)
        #: 走到 :attr:`_playback_seq` 的第几个
        self._playback_pos: int = 0
        #: 由主窗口注入的回调：返回**当前选中对象**关联的机器人名（``None``
        #: 表示没选中或关联不到任何机器人）。编辑器自己看不到视口选中状态
        self.selection_robot_provider = None
        #: 工作单元场景（由 MainWindow 注入），用于枚举可抓取的工件
        self._scene = None
        #: 工作单元模型（硬件列表 / PLC 程序逻辑共用同一份）
        self.cell = CellConfig() if _HAS_CELL else None
        #: 供「机器人子程序」生成用的真实机器人（含运动学）
        self._robot = None
        self._scene = None
        self._setup_ui()
        # 光标一动，"从选中行号处运行"的起点就变了，范围提示要跟着更新
        self.code_edit.cursorPositionChanged.connect(self.refresh_scope)
        self.setPlainText("# OLP Path Code\nmovej([0, -90, 90, -90, 90, 0], vel=0.5)\n")

    def _setup_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        self.sub_tabs = QtWidgets.QTabWidget()

        # ---------- 1) 路径示教（原有能力） ----------
        teach = QtWidgets.QWidget()
        teach_layout = QtWidgets.QVBoxLayout(teach)
        teach_layout.setContentsMargins(0, 0, 0, 0)
        teach_layout.setSpacing(2)

        self.code_edit = QtWidgets.QTextEdit()
        self.code_edit.setFont(QFont("Consolas", 10))
        self.code_edit.setStyleSheet("background-color: #1a1a1a; color: #e0e0e0;")
        self.code_edit.setLineWrapMode(QtWidgets.QTextEdit.NoWrap)
        self.code_edit.setTabStopDistance(20)
        # 手改代码 → 停手后自动重解析：3D 路径与离线编程面板的点表跟着刷新
        self.code_edit.textChanged.connect(self._edit_parse_timer.start)
        # 右键菜单：标准编辑项 +「执行此行」（逐行核对的入口）
        self.code_edit.setContextMenuPolicy(Qt.CustomContextMenu)
        self.code_edit.customContextMenuRequested.connect(self._show_code_context_menu)
        teach_layout.addWidget(self.code_edit)

        # 按钮文案一律走 tr()：这一排 6 个键是**左侧栏最宽的一行**，换成中文
        # 后所需宽度从 ~475px 降到 ~300px——既去掉了中英文混用，也在 400px
        # 的栏宽里放得下了（2026-09-21 界面体检）
        self.record_btn = QtWidgets.QPushButton(tr("● 记录"))
        self.record_btn.setStyleSheet("background-color: #555555; color: white;")
        self.record_btn.setCheckable(True)
        self.record_btn.clicked.connect(self._on_record_toggled)

        self.play_btn = QtWidgets.QPushButton(tr("▶ 播放"))
        self.play_btn.setStyleSheet("background-color: #4CAF50; color: white;")
        self.play_btn.clicked.connect(self._on_play_clicked)

        self.pause_btn = QtWidgets.QPushButton(tr("⏸ 暂停"))
        self.pause_btn.setStyleSheet("background-color: #8a8a8a; color: white;")
        self.pause_btn.setToolTip(tr("暂停 / 继续；暂停保留当前点位，便于逐点核对"))
        self.pause_btn.clicked.connect(self._on_pause_clicked)

        self.step_btn = QtWidgets.QPushButton(tr("⏭ 单步"))
        self.step_btn.setStyleSheet("background-color: #6a6a6a; color: white;")
        self.step_btn.setToolTip(tr("走一个示教点——核对抓取/放置时机最有用的操作"))
        self.step_btn.clicked.connect(self._on_step_clicked)

        self.stop_btn = QtWidgets.QPushButton(tr("■ 停止"))
        self.stop_btn.setStyleSheet("background-color: #f44336; color: white;")
        self.stop_btn.clicked.connect(self._on_stop_clicked)

        self.clear_btn = QtWidgets.QPushButton(tr("清空"))
        self.clear_btn.setStyleSheet("background-color: #777777; color: white;")
        self.clear_btn.clicked.connect(self._on_clear_clicked)

        self.pos_count_label = QtWidgets.QLabel(tr("示教点：0"))
        self.pos_count_label.setFont(QFont("Consolas", 9))

        #: 将要执行的范围——按下去之前就该知道跑的是哪台、从哪一行开始
        self.scope_label = QtWidgets.QLabel("")
        self.scope_label.setFont(QFont("Consolas", 9))
        self.scope_label.setStyleSheet("color: #9ad0ff;")

        # 播放按钮排成 3×2 网格而不是一行：这个面板在左侧栏里只有 280–400px，
        # 一行摆 6 个按钮 + 2 个提示 + 2 个下拉时 Qt 不会换行，只会把每个按钮
        # 压到 33px（实测：只剩一个色块），并把最后一个下拉顶到面板外面看不见。
        btn_grid = QtWidgets.QGridLayout()
        btn_grid.setHorizontalSpacing(4)
        for _i, _btn in enumerate(
            (
                self.record_btn,
                self.play_btn,
                self.pause_btn,
                self.step_btn,
                self.stop_btn,
                self.clear_btn,
            )
        ):
            _btn.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            btn_grid.addWidget(_btn, _i // 3, _i % 3)
        teach_layout.addLayout(btn_grid)

        # 点数与"跑哪台、从哪行起"分两行：范围提示本身可以很长
        # （如「将从第 3 行起全局运行（8/8 点）」），和点数挤在一行会把它压没
        info_layout = QtWidgets.QHBoxLayout()
        info_layout.addWidget(self.pos_count_label)
        info_layout.addStretch(1)
        teach_layout.addLayout(info_layout)
        self.scope_label.setWordWrap(True)
        teach_layout.addWidget(self.scope_label)

        # ---- 抓取工艺：点位语义 + 工件选择 ----
        if _HAS_PICKPLACE:
            process_layout = QtWidgets.QHBoxLayout()
            process_layout.addWidget(QtWidgets.QLabel(tr("点位:")))
            self.point_type_combo = QtWidgets.QComboBox()
            self.point_type_combo.setToolTip(
                tr(
                    "记录时赋予该示教点的工艺语义。\n"
                    "抓取点：插入夹紧 IO，播放时把工件吸附到法兰；\n"
                    "放置点：插入松开 IO，播放时把工件释放到世界。"
                )
            )
            for t in PointType:
                self.point_type_combo.addItem(t.label, t)
            # 工件名可能很长——不设下限的话它会按最长的一项撑宽整行，
            # 把右边的东西挤出可视区（这在侧栏里等于直接看不见）
            self.point_type_combo.setMinimumContentsLength(4)
            process_layout.addWidget(self.point_type_combo, 1)

            process_layout.addWidget(QtWidgets.QLabel(tr("工件:")))
            self.workpiece_combo = QtWidgets.QComboBox()
            self.workpiece_combo.setMinimumContentsLength(4)
            self.workpiece_combo.setToolTip(tr("被抓取的场景对象；留空表示只演示轨迹"))
            process_layout.addWidget(self.workpiece_combo, 1)
            teach_layout.addLayout(process_layout)

        self.sub_tabs.addTab(teach, tr("路径示教"))

        # ---------- 2) 硬件列表（MVP 隐藏：设备台账还没和出码打通，队列 P2） ----------
        if _feature_enabled("hardware_list"):
            self.sub_tabs.addTab(self._build_hardware_tab(), tr("硬件列表"))

        # ---------- 3) PLC 程序逻辑（MVP 隐藏：出码尚未完善，队列 P2） ----------
        if _feature_enabled("plc_logic"):
            self.sub_tabs.addTab(self._build_plc_tab(), tr("PLC 程序逻辑"))

        # ---------- 4) 机器人子程序（MVP 隐藏：尚未完善，队列 P2） ----------
        if _feature_enabled("subprogram"):
            self.sub_tabs.addTab(self._build_subprogram_tab(), tr("机器人子程序"))

        layout.addWidget(self.sub_tabs)

    # ---------------- 代码生成：硬件列表 ---------------- #

    def _build_hardware_tab(self):
        w = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(w)
        layout.setContentsMargins(2, 2, 2, 2)

        self.device_table = QtWidgets.QTableWidget(0, 6)
        self.device_table.setHorizontalHeaderLabels(
            [tr("名称"), tr("类型"), tr("厂商"), tr("型号"), tr("通信"), "IP"]
        )
        self.device_table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.ResizeToContents
        )
        self.device_table.verticalHeader().setVisible(False)
        layout.addWidget(self.device_table, 1)

        row = QtWidgets.QHBoxLayout()
        self.btn_hw_add = QtWidgets.QPushButton(tr("添加设备"))
        self.btn_hw_add.clicked.connect(self._add_device_row)
        row.addWidget(self.btn_hw_add)
        self.btn_hw_del = QtWidgets.QPushButton(tr("删除"))
        self.btn_hw_del.clicked.connect(self._remove_device_row)
        row.addWidget(self.btn_hw_del)
        self.btn_hw_sync = QtWidgets.QPushButton(tr("从场景同步"))
        self.btn_hw_sync.setToolTip(tr("按场景里的机器人/组件补齐设备清单（已存在的保留不动）"))
        self.btn_hw_sync.clicked.connect(self._sync_devices_from_scene)
        row.addWidget(self.btn_hw_sync)
        layout.addLayout(row)

        # IO 点表（v1.1.6）：机器人程序里用的信号必须在这张表上有地址，
        # 否则下装后机器人要么发不出去、要么一直等一个没人给的输入
        row2 = QtWidgets.QHBoxLayout()
        self.btn_io_map = QtWidgets.QPushButton(tr("IO 映射…"))
        self.btn_io_map.setToolTip(tr("编辑当前设备的信号地址（点表与 PLC 出码共用这一份数据）"))
        self.btn_io_map.clicked.connect(self._edit_io_map)
        row2.addWidget(self.btn_io_map)
        self.btn_io_check = QtWidgets.QPushButton(tr("IO 校验"))
        self.btn_io_check.setToolTip(
            tr("检查未映射 / 地址重复 / 两侧同时驱动 / 机器人等一个没人给的信号")
        )
        self.btn_io_check.clicked.connect(self._check_io_map)
        row2.addWidget(self.btn_io_check)
        row2.addStretch(1)
        layout.addLayout(row2)
        return w

    # ---------------- IO 点表（v1.1.6） ---------------- #

    def _notify(self, text: str, timeout: int = 6000) -> None:
        """把一行提示送到状态栏。

        ``PathCodeEditor`` 是普通 ``QWidget``，**没有自己的状态栏**——早期代码
        直接调 ``self.statusBar()``，一点「生成 PLC」就 AttributeError。
        这里统一走：有主窗口就用主窗口的状态栏，单独实例化（测试/嵌入式）时
        退化为 ``statusMessage`` 信号，既不崩也不静默丢消息。
        """
        bar = getattr(self.window(), "statusBar", None)
        if callable(bar):
            try:
                bar().showMessage(text, timeout)
                return
            except Exception:  # pragma: no cover - 状态栏不可用时的兜底
                pass
        self.statusMessage.emit(text)

    def _signal_map(self):
        """当前单元的点表视图；单元模型不可用时返回 ``None``。"""
        if self.cell is None or SignalMap is None:
            return None
        return SignalMap(self.cell)

    def _current_program_items(self):
        """当前示教点对应的机器人程序指令（点表校验的"用途"来源）。

        与「生成 PLC」走同一条数据通路：优先按**真实示教出来的抓取 IO 时序**，
        没有示教点时返回空——不拿默认骨架冒充真实程序的用法。
        """
        if not _HAS_PICKPLACE:
            return []
        pts = self._taught_points_deg()
        if not pts:
            return []
        try:
            # 用**全量**语义：pts 取自整份点表（可能含多台的路径），只给"当前
            # 编辑那台"那段语义会让点数与语义数对不上
            plan = build_pick_place(pts, self.point_types)
        except Exception as exc:  # pragma: no cover - 示教数据异常时不该崩
            logger.warning("point_table_program_failed: %s", exc)
            return []
        return list(plan.items)

    def _edit_io_map(self):
        """编辑选中设备的 IO 映射（信号 ↔ 地址）。"""
        smap = self._signal_map()
        if smap is None:
            QtWidgets.QMessageBox.information(self, tr("IO 映射"), tr("单元模型不可用"))
            return
        device = self._selected_device()
        if device is None:
            QtWidgets.QMessageBox.information(self, tr("IO 映射"), tr("请先在设备列表里选中一行"))
            return

        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle(tr("IO 映射 · {name}").format(name=device.name))
        dialog.resize(560, 480)
        layout = QtWidgets.QVBoxLayout(dialog)

        items = self._current_program_items()
        table = QtWidgets.QTableWidget(0, 4)
        table.setHorizontalHeaderLabels([tr("信号"), tr("地址（可编辑）"), tr("方向"), tr("用途")])
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        table.verticalHeader().setVisible(False)
        layout.addWidget(table, 1)

        def refill():
            """重填表格：本设备声明的信号 + 机器人程序在用但没有任何设备的信号。

            后者必须出现——"程序在用、点表里没有"正是最该被补上的那类点，
            藏在别处就等于没提示。
            """
            points = [p for p in smap.points(items) if device.name in p.devices]
            covered = {p.signal for p in points}
            for p in smap.points(items):
                if p.used and p.signal not in covered:
                    points.append(p)
                    covered.add(p.signal)
            table.setRowCount(len(points))
            for r, p in enumerate(points):
                for c, text in enumerate((p.signal, p.address, p.direction_text(), p.usage_text())):
                    item = QtWidgets.QTableWidgetItem(text)
                    if c != 1:
                        item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                    table.setItem(r, c, item)

        refill()

        row = QtWidgets.QHBoxLayout()
        btn_add = QtWidgets.QPushButton(tr("添加信号"))
        btn_auto = QtWidgets.QPushButton(tr("自动编号"))
        btn_auto.setToolTip(tr("按方向自动编号，跳过已被占用的地址"))
        row.addWidget(btn_add)
        row.addWidget(btn_auto)
        row.addStretch(1)
        layout.addLayout(row)

        def add_signal():
            text, ok = QtWidgets.QInputDialog.getText(
                dialog, tr("添加信号"), tr("信号名（如 gripper_close）")
            )
            if ok and text.strip():
                smap.assign(text.strip(), "", device.name)
                refill()

        def auto_number():
            # 只给本设备上还没有地址的信号编号，且跳过别的信号已占用的地址
            pending = [
                table.item(r, 0).text()
                for r in range(table.rowCount())
                if not table.item(r, 1).text().strip()
            ]
            got = smap.auto_assign(device.name, pending, extra_signals=pending)
            refill()
            self._notify(tr("自动编号：{n} 个信号（跳过已占用地址）").format(n=len(got)))

        btn_add.clicked.connect(add_signal)
        btn_auto.clicked.connect(auto_number)

        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        layout.addWidget(buttons)

        def apply_and_close():
            # 把表格里的地址写回点表：空 = 摘掉映射（保留信号声明）
            changed = self._write_io_map_from_table(table, device.name, smap)
            self._notify(
                tr("IO 映射已更新：{name}（{n} 个信号）").format(name=device.name, n=changed)
            )
            dialog.accept()

        buttons.accepted.connect(apply_and_close)
        buttons.rejected.connect(dialog.reject)
        dialog.exec()
        self._check_io_map(silent=True)

    @staticmethod
    def _write_io_map_from_table(table, device_name: str, smap) -> int:
        """把映射表格里的地址写回点表，返回处理过的信号数。

        抽成独立方法是为了能测：对话框里的闭包没法在无头测试里驱动，
        而"表格内容 → 点表"这一步恰恰是唯一有副作用的逻辑。
        """
        handled = 0
        for r in range(table.rowCount()):
            item = table.item(r, 0)
            if item is None:
                continue
            signal = item.text().strip()
            if not signal:
                continue
            addr_item = table.item(r, 1)
            address = addr_item.text().strip() if addr_item is not None else ""
            if address:
                smap.assign(signal, address, device_name)
            else:
                smap.unassign(signal, device_name)
            handled += 1
        return handled

    def _check_io_map(self, silent: bool = False):
        """校验 IO 点表并展示结论。``silent=True`` 时只刷新状态栏（用于编辑后）。"""
        smap = self._signal_map()
        if smap is None:
            return None
        items = self._current_program_items()
        try:
            steps = steps_from_program(items) if items else None
        except Exception:  # pragma: no cover
            steps = None
        report = smap.validate(items, steps)
        summary = report.summary()
        if silent:
            self._notify(summary, 8000)
            return report

        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle(tr("IO 点表校验"))
        box.setText(summary)
        details = report.format()
        if report.issues:
            box.setInformativeText(
                tr("有问题的点必须在下载前处理（双击「Details」看逐点清单）")
                if report.errors
                else tr("警告不阻塞下载，但现场必须有人看过")
            )
        box.setDetailedText(details)
        box.setIcon(
            QtWidgets.QMessageBox.Critical
            if report.errors
            else (
                QtWidgets.QMessageBox.Warning
                if report.warnings
                else QtWidgets.QMessageBox.Information
            )
        )
        box.exec()
        self._notify(summary, 8000)
        return report

    def _device_table(self):
        """硬件列表的表格；MVP 档入口没构建时是 ``None``。"""
        return getattr(self, "device_table", None)

    def _selected_device(self):
        """设备列表里当前选中的设备；无选中返回 ``None``。"""
        if self.cell is None:
            return None
        table = self._device_table()
        if table is None:
            return None
        r = table.currentRow()
        if r < 0 or r >= len(self.cell.devices):
            return None
        return self.cell.devices[r]

    def _add_device_row(self, name="", type_=tr("传感器")):
        if self.cell is None:
            return
        name = name or f"设备_{len(self.cell.devices) + 1}"
        self.cell.add(Device(name=name, type=type_))
        self._refresh_device_table()

    def _remove_device_row(self):
        if self.cell is None:
            return
        table = self._device_table()
        if table is None:
            return
        r = table.currentRow()
        if r < 0 or r >= len(self.cell.devices):
            return
        self.cell.remove(self.cell.devices[r].name)
        self._refresh_device_table()

    def _refresh_device_table(self):
        """把 ``cell`` 里的设备刷到表格上。

        MVP 档表格不存在时**静默返回**而不是报错：``sync_devices`` /
        ``set_cell`` 会调到这里，而设备数据始终在 ``self.cell`` 里——
        PLC、点表读的是同一份，只是没有界面去看它。
        """
        if self.cell is None:
            return
        table = self._device_table()
        if table is None:
            return
        table.setRowCount(len(self.cell.devices))
        for i, d in enumerate(self.cell.devices):
            for j, val in enumerate([d.name, d.type, d.brand, d.model, d.comm, d.ip]):
                table.setItem(i, j, QtWidgets.QTableWidgetItem(val))

    def _sync_devices_from_scene(self):
        self.sync_devices(self._scene)

    def sync_devices(self, scene) -> None:
        """按场景对象补齐设备清单；已存在的保留，避免覆盖用户填的内容。"""
        if self.cell is None or scene is None:
            return
        for name in list(scene.objects):
            if self.cell.find(name):
                continue
            # 与设备库同一套取值：机器人 → 机器人，组件 → 分类
            if scene.is_robot(name):
                dev_type = "机器人"
            else:
                obj = scene.objects[name]
                spec = COMPONENTS.get(obj.component_id) if obj.component_id else None
                dev_type = tr(spec.category) if spec else "传感器"
            self.cell.add(Device(name=name, type=dev_type))
        self._refresh_device_table()

    # ---------------- 代码生成：PLC 程序逻辑 ---------------- #

    def _build_plc_tab(self):
        w = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(w)
        layout.setContentsMargins(2, 2, 2, 2)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel(tr("目标")))
        self.plc_target = QtWidgets.QComboBox()
        self.plc_target.addItem(tr("西门子 SCL"), "siemens_scl")
        row.addWidget(self.plc_target, 1)
        self.btn_gen_plc = QtWidgets.QPushButton(tr("生成"))
        self.btn_gen_plc.clicked.connect(self._generate_plc)
        row.addWidget(self.btn_gen_plc)
        layout.addLayout(row)

        self.plc_preview = QtWidgets.QTextEdit()
        self.plc_preview.setReadOnly(True)
        self.plc_preview.setFont(QFont("Consolas", 9))
        self.plc_preview.setLineWrapMode(QtWidgets.QTextEdit.NoWrap)
        self.plc_preview.setPlaceholderText(tr("生成的 PLC 代码（西门子 SCL）将显示在这里"))
        layout.addWidget(self.plc_preview, 1)
        return w

    def _taught_points_deg(self):
        """把示教点（关节**弧度**字典）转成**度**列表，供抓取工艺模块使用。

        关节名以**示教点里实际存在的键**为准：``joint_names`` 来自
        ``renderer.joints``（含 fixed 关节，KR16 是 7 个），而运动学只认 6 个
        可动关节，照抄会让后续的 FK 直接抛错。
        """
        positions = self.recorded_positions
        if not positions:
            return []
        keys = list(positions[0].keys())
        names = [n for n in self.joint_names if n in keys] or keys
        return [[math.degrees(float(p.get(n, 0.0))) for n in names] for p in positions]

    def _generate_plc(self):
        if self.cell is None or generate_scl is None:
            self.plc_preview.setPlainText(tr("单元模型不可用"))
            return
        try:
            # 优先按**真实示教出来的抓取 IO 时序**生成步序；没有示教点时
            # 回退到默认骨架，这样 PLC 与机器人程序共用同一份 IO 定义
            items = self._current_program_items()
            derived = steps_from_program(items) if items else []
            steps = derived or None
            # program=items：点表的"用途"列按真实程序填，并能报出
            # "程序在用但没映射"的点——这类点藏在界面里等于没提示
            code = generate_scl(
                self.cell,
                program_name="CellControl",
                steps=steps,
                program=items or None,
            )
        except Exception as exc:
            self.plc_preview.setPlainText(tr("生成失败：\n{exc}").format(exc=exc))
            return
        self.plc_preview.setPlainText(code)
        # 点表问题随出码一起报：SCL 注释块里写了，但状态栏也要让人当场看见
        report = self._check_io_map(silent=True)
        if report is not None and report.errors:
            self._notify(
                tr("PLC 代码已生成，但点表有 {n} 处错误——下装前必须处理").format(
                    n=len(report.errors)
                ),
                10000,
            )

    # ---------------- 代码生成：机器人子程序 ---------------- #

    def _build_subprogram_tab(self):
        w = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(w)
        layout.setContentsMargins(2, 2, 2, 2)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel(tr("品牌")))
        self.combo_brand = QtWidgets.QComboBox()
        self.combo_brand.addItem(tr("自动（按机器人）"), None)
        try:
            from adapter.post import get_post_processor, list_post_processors

            for name in list_post_processors():
                pp = get_post_processor(name)
                self.combo_brand.addItem(f"{pp.brand.upper()} {pp.controller.upper()}", name)
        except Exception:
            pass
        row.addWidget(self.combo_brand, 1)
        self.btn_gen_sub = QtWidgets.QPushButton(tr("生成"))
        self.btn_gen_sub.clicked.connect(self._generate_subprograms)
        row.addWidget(self.btn_gen_sub)
        layout.addLayout(row)

        self.sub_preview = QtWidgets.QTextEdit()
        self.sub_preview.setReadOnly(True)
        self.sub_preview.setFont(QFont("Consolas", 9))
        self.sub_preview.setLineWrapMode(QtWidgets.QTextEdit.NoWrap)
        self.sub_preview.setPlaceholderText(tr("机器人子程序代码（需先加载机器人并示教）"))
        layout.addWidget(self.sub_preview, 1)
        return w

    def set_robot(self, robot, joint_names=None) -> None:
        """注入真实机器人（含运动学），供「机器人子程序」生成代码。"""
        self._robot = robot
        if joint_names:
            self.set_joint_names(joint_names)

    def _generate_subprograms(self):
        if self._robot is None:
            self.sub_preview.setPlainText(tr("请先加载机器人"))
            return
        if not self.recorded_positions:
            self.sub_preview.setPlainText(tr("请先在「路径示教」里 Record 至少一个示教点"))
            return
        try:
            from service.program import ProgramBuilder

            builder = ProgramBuilder(self._robot)
            names = list(self._robot.joint_names)
            for pos in self.recorded_positions:
                joints = [math.degrees(float(pos.get(n, 0.0))) for n in names]
                builder.moveJ(joints, speed=100)
            code = builder.build().to_post(
                self.combo_brand.currentData(),
                metadata={"program_name": getattr(self._robot, "name", "program")},
            )
        except Exception as exc:
            self.sub_preview.setPlainText(tr("生成失败：\n{exc}").format(exc=exc))
            return
        self.sub_preview.setPlainText(code)

    def _on_record_toggled(self, checked):
        if checked:
            self.record_btn.setText(tr("● 正在记录"))
            self.record_btn.setStyleSheet("background-color: #f44336; color: white;")
            self.record_btn.setIcon(QtGui.QIcon())
        else:
            self.record_btn.setText(tr("● 记录"))
            self.record_btn.setStyleSheet("background-color: #555555; color: white;")

    def _on_play_clicked(self):
        # 在播放前解析代码编辑器中的路径代码
        self._parse_code_to_positions()

        if len(self.recorded_positions) == 0:
            return

        # 通知主窗口更新 GLWidget 的路径可视化
        self.positionsParsed.emit()

        # 选中了机器人（或挂在它上面的夹爪/工件）→ 只跑这台的路径；
        # 没选中 → 从代码选中行号处全局运行
        robot = self._selected_robot()
        self._playback_seq = self.playback_plan(robot=robot, from_cursor=robot is None)
        if not self._playback_seq:
            self._notify(tr("没有可运行的路径点"))
            return
        self._ensure_timer()
        if not self.playback_timer.isActive():
            self._playback_pos = 0
            self.playback_timer.start(self.playback_speed)
            self._emit_current()

    def _on_stop_clicked(self):
        self._stop_playback()
        self.playbackStopped.emit()

    def _ensure_timer(self) -> None:
        """惰性创建播放定时器（与 Play 分支共用同一份创建逻辑）。"""
        if self.playback_timer is None:
            self.playback_timer = QtCore.QTimer(self)
            self.playback_timer.timeout.connect(self._on_playback_tick)

    def _emit_current(self) -> None:
        """把当前这一步的**点索引**与**它归属那台**一起发出去。"""
        index = self._playback_seq[self._playback_pos]
        self.current_playback_index = index
        self.playbackRequested.emit(index, self.robot_at(index) or "")

    def _rebuild_seq_if_idle(self) -> None:
        """还没有播放序列（首次单步/继续）时按当前选中与光标行建一条。"""
        if self._playback_seq:
            return
        robot = self._selected_robot()
        self._playback_seq = self.playback_plan(robot=robot, from_cursor=robot is None)
        self._playback_pos = 0

    def _on_pause_clicked(self):
        """暂停 / 继续。

        暂停只停定时器、**保留当前索引**，这样继续时从断点接着走，而不是
        从头重来——否则逐点核对抓取时机根本没法做。
        """
        if not self.recorded_positions:
            return
        self._ensure_timer()
        self._rebuild_seq_if_idle()
        if self.playback_timer.isActive():
            self.playback_timer.stop()
        else:
            if self._playback_pos >= len(self._playback_seq):
                self._playback_pos = 0
            self.playback_timer.start(self.playback_speed)
        self._sync_pause_button()

    def _on_step_clicked(self):
        """从当前代码光标所在的可执行行向下执行一个示教点。"""
        # 代码编辑采用防抖解析；用户可能在 300ms 内直接点击单步。
        # 先提交待解析文本，避免 recorded_positions 仍为空而静默返回。
        if self._edit_parse_timer.isActive():
            self._edit_parse_timer.stop()
            self._parse_code_to_positions()
        if not self.recorded_positions:
            return
        self._ensure_timer()
        if self.playback_timer.isActive():
            self.playback_timer.stop()

        # 单步以当前光标为起点重新规划，避免用户移动光标后仍沿用旧序列。
        robot = self._selected_robot()
        self._playback_seq = self.playback_plan(
            robot=robot,
            from_cursor=robot is None,
        )
        self._playback_pos = 0
        if not self._playback_seq:
            self._sync_pause_button()
            return

        index = self._playback_seq[0]
        self._on_playback_tick()

        # 回放处理会高亮已执行行；单步完成后把编辑光标推进到下一条指令，
        # 下一次点击即可继续向下执行，而不会重复当前行。
        next_line = self.line_of(index) + 1
        for next_index in self._playback_seq[1:]:
            candidate = self.line_of(next_index)
            if candidate > self.line_of(index):
                next_line = candidate
                break
        self.code_edit.moveCursor(QtGui.QTextCursor.StartOfBlock)
        block = self.code_edit.document().findBlockByNumber(max(0, next_line - 1))
        if block.isValid():
            self.code_edit.setTextCursor(QtGui.QTextCursor(block))
        self._sync_pause_button()

    def _sync_pause_button(self):
        """按定时器状态刷新暂停按钮文案。"""
        running = self.playback_timer is not None and self.playback_timer.isActive()
        self.pause_btn.setText(tr("⏸ 暂停") if running else tr("▶ 继续"))

    def _stop_playback(self):
        # 定时器对象保留（不再置 None），否则暂停后再 Stop 会拿不到它来停
        if self.playback_timer is not None:
            self.playback_timer.stop()
        self.current_playback_index = 0
        self._playback_pos = 0
        self._sync_pause_button()

    def _on_playback_tick(self):
        if self._playback_pos < len(self._playback_seq):
            self._emit_current()
            self._playback_pos += 1
        else:
            self._stop_playback()
            self.playbackStopped.emit()

    def _on_clear_clicked(self):
        self._stop_playback()
        self.recorded_positions.clear()
        self.point_robots.clear()
        self.point_types.clear()
        self._line_map.clear()
        self.pos_count_label.setText(tr("示教点：0"))
        self.code_edit.clear()
        self.code_edit.append("# OLP Path Code\n")
        self.clearRequested.emit()

    def record_position(self, joint_values, robot=None):
        if self.record_btn.isChecked():
            self.recorded_positions.append(dict(joint_values))
            # 归属跟着点一起进表：不记的话这一段就"没主人"，播放时只能退回
            # 整份归属，多机下会把这台的点驱动到另一台身上
            self.point_robots.append(robot or self.owner_robot)
            if _HAS_PICKPLACE:
                self.point_types.append(self.point_type_combo.currentData())
            self.pos_count_label.setText(tr("示教点：{n}").format(n=len(self.recorded_positions)))
            self.positionRecorded.emit(len(self.recorded_positions))
            self._update_code_with_positions()

    # ``_suggest_next_point_type`` 已删除：它会把点位下拉自动拨到"下一个"
    # 循环语义——用户随手连教几个点，第 2 个就被标成「抓取点」、第 4 个被标
    # 成「放置点」，导出程序的开头凭空多出 5 行夹爪开合（2026-09-21 用户反馈）。
    # 语义是工艺意图，要标请用点位下拉或「抓取向导」，程序不再替用户做主。

    def sync_point_types(self) -> None:
        """把 point_types 补齐到与 recorded_positions 等长。

        MainWindow 会直接改写 ``recorded_positions``（视口 Record 的路径），
        此时语义列表要跟着同步，否则两边长度不一致会直接报错。
        """
        if not _HAS_PICKPLACE:
            return
        n = len(self.recorded_positions)
        if len(self.point_types) > n:
            self.point_types = self.point_types[:n]
        while len(self.point_types) < n:
            self.point_types.append(None)
        # 补默认语义要**按段**来：默认的五点式是每台各一套
        self._fill_missing_types()

    def get_point_types(self, robot=None):
        """``robot``（缺省**当前编辑那台**）的工艺语义。

        一份点表里可能装多台的路径，语义只能一段一段地给：直接返回整份会让
        "切回 A"拿到 A+B 混在一起的语义。``robot`` 解析不到（旧工程没有归属
        标记）时退回整份，行为与从前一致。
        """
        if robot is None:
            robot = self.owner_robot
        if robot is not None and robot in self.point_robots:
            return self.types_of(robot)
        return self.point_types

    def point_type_at(self, index: int):
        """第 ``index`` 个点（**总表索引**）的工艺语义——播放按行序取值用。"""
        return self._type_at(index)

    def set_scene(self, scene) -> None:
        """注入工作单元场景，用于枚举可抓取的工件。"""
        self._scene = scene
        self.refresh_workpiece_combo()

    def set_cell(self, cell) -> None:
        """整体换掉工作单元模型（工程文件载入用 v1.3.4）。

        只换数据 + 刷设备表：点表是**用户填过的内容**，工程载入必须原样恢复，
        所以这里不做"按场景补齐设备"那套推导（那是 :meth:`sync_devices` 的活）。
        """
        if cell is None or not _HAS_CELL:
            return
        self.cell = cell
        self._refresh_device_table()

    def refresh_workpiece_combo(self) -> None:
        """重建「工件」下拉：场景里除机器人外的全部对象。"""
        if not _HAS_PICKPLACE:
            return
        current = self.workpiece_combo.currentData()
        self.workpiece_combo.blockSignals(True)
        self.workpiece_combo.clear()
        self.workpiece_combo.addItem(tr("（无）"), None)
        if self._scene is not None:
            for name in self._scene.objects:
                if self._scene.is_robot(name):
                    continue
                self.workpiece_combo.addItem(name, name)
        idx = self.workpiece_combo.findData(current)
        self.workpiece_combo.setCurrentIndex(idx if idx >= 0 else 0)
        self.workpiece_combo.blockSignals(False)

    def get_workpiece(self):
        """当前选中的工件名；未选返回 None。"""
        if not _HAS_PICKPLACE:
            return None
        return self.workpiece_combo.currentData()

    def segments(self) -> List[Tuple[Optional[str], List[int]]]:
        """点表按归属切成**连续的段**：``[(机器人名或 None, [点索引...])]``。

        出码与按机归档都用它：同一台的点聚成一段，段首写一个 ``# Robot:``
        头，于是两台机器人的路径在代码里各占一段、一眼分得清。
        """
        out: List[Tuple[Optional[str], List[int]]] = []
        for i, robot in enumerate(self.point_robots):
            if out and out[-1][0] == robot:
                out[-1][1].append(i)
            else:
                out.append((robot, [i]))
        return out

    def _update_code_with_positions(self):
        self.code_edit.clear()
        self.code_edit.append("# OLP Path Code")
        self.code_edit.append(f"# Recorded positions: {len(self.recorded_positions)}")
        # 逐段出码：每台机器人一段，段首带归属。整份只写一个 Robot 头时，
        # 第二台一记点，第一台的路径就从代码里整段消失了——这也是"看不到
        # 机器人信息"的由来。
        for robot, indices in self.segments():
            self.code_edit.append("")
            if robot:
                # 归属写进代码：从别处粘贴 / 载入工程时靠这一行认出这串点属于哪台
                self.code_edit.append(f"# Robot: {robot}")
            for i in indices:
                # 将弧度转换为角度显示
                joints_str = ", ".join(
                    f"{math.degrees(v):.2f}" for v in self.recorded_positions[i].values()
                )
                self.code_edit.append(f"movej([{joints_str}], vel=0.5)")
        self._update_count_label()

    def _update_count_label(self) -> None:
        """点数标签带上**各台各多少点**——归属只写在代码注释里太容易看不见。"""
        n = len(self.recorded_positions)
        counts: dict = {}
        for robot in self.point_robots:
            counts[robot] = counts.get(robot, 0) + 1
        named = {r: c for r, c in counts.items() if r}
        if named:
            detail = " · ".join(f"{r} {c}" for r, c in named.items())
            self.pos_count_label.setText(tr("示教点：{n}（{detail}）").format(n=n, detail=detail))
        else:
            self.pos_count_label.setText(tr("示教点：{n}").format(n=n))

    def get_recorded_positions(self):
        return self.recorded_positions

    def set_recorded_positions(self, positions, owner=None):
        """换上 ``owner`` 的示教点；``owner`` 为空表示整份替换、归属不变。

        只替 ``owner`` **那一段**，其余机器人的段原样保留：编辑器一份点表里
        同时装着多台的路径，给 B 记点若整表替换，A 的路径就被顶掉了——那既
        是"看不到机器人信息"，也是后面播放时把 A 的点算到 B 名下的源头。
        """
        positions = list(positions)
        if owner is None:
            # 整份替换：**归属按原表沿用**——docstring 一直这么承诺，实现却曾把
            # 归属整体抹成 None。无归属段渲染时不带 ``# Robot:`` 头，用户看到的
            # 就是"代码开头莫名其妙多出几行 movej"；而且此后所有按机写入都只能
            # 追加在它后面，这段幽灵行永远顶在最上面（2026-09-21 用户反馈）
            prev_own = list(self.point_robots)
            prev_owner = self.owner_robot
            self.recorded_positions = positions
            self.point_robots = [
                prev_own[i] if i < len(prev_own) else prev_owner for i in range(len(positions))
            ]
        else:
            new_pos, new_own, new_types = [], [], []
            old_types = {
                robot: [self._type_at(i) for i in indices] for robot, indices in self.segments()
            }
            placed = False
            for robot, indices in self.segments():
                if robot == owner:
                    if placed:
                        continue  # 同属的旧段已被新点取代
                    placed = True
                    prev = old_types.get(robot) or []
                    for k, pos in enumerate(positions):
                        new_pos.append(pos)
                        new_own.append(owner)
                        new_types.append(prev[k] if k < len(prev) else None)
                else:
                    for i in indices:
                        new_pos.append(self.recorded_positions[i])
                        new_own.append(robot)
                        new_types.append(self._type_at(i))
            if not placed:  # 这台还没有段：追加到末尾
                for pos in positions:
                    new_pos.append(pos)
                    new_own.append(owner)
                    new_types.append(None)
            self.recorded_positions = new_pos
            self.point_robots = new_own
            self.point_types = new_types
            self.owner_robot = owner
        self._fill_missing_types()
        self.pos_count_label.setText(tr("示教点：{n}").format(n=len(self.recorded_positions)))
        self._update_code_with_positions()
        self.update()

    def _type_at(self, index: int):
        """第 ``index`` 个点的工艺语义；越界返回 ``None``。"""
        if 0 <= index < len(self.point_types):
            return self.point_types[index]
        return None

    def _fill_missing_types(self) -> None:
        """把 ``point_types`` 的长度对齐到点表（只做截断/补空，**不发明语义**）。

        之前这里会按"五点式循环"给没标语义的点自动填 接近/抓取/提升/放置/撤离
        ——用户只是随手教一条轨迹，导出的程序开头却**自动多出 5 行夹爪开合**
        （2026-09-21 用户反馈：这 5 行不需要）。抓取语义是工艺意图，只能
        用户自己标（点位下拉 / 抓取向导），程序不再替用户做主。
        """
        if not _HAS_PICKPLACE:
            return
        n = len(self.recorded_positions)
        if len(self.point_types) > n:
            self.point_types = self.point_types[:n]
        while len(self.point_types) < n:
            self.point_types.append(None)

    def points_of(self, robot: Optional[str]) -> List[dict]:
        """``robot`` 那一段的示教点（按代码里的先后顺序）。"""
        return [self.recorded_positions[i] for i in self.indices_of(robot)]

    def indices_of(self, robot: Optional[str]) -> List[int]:
        """``robot`` 那段在总表里的点索引。"""
        if robot is None:
            return list(range(len(self.recorded_positions)))
        return [i for i, r in enumerate(self.point_robots) if r == robot]

    def types_of(self, robot: Optional[str]) -> List:
        """``robot`` 那段对应的工艺语义。"""
        return [self._type_at(i) for i in self.indices_of(robot)]

    def set_types_for(self, robot: Optional[str], types) -> None:
        """给 ``robot`` 那段回填工艺语义（切换机器人时恢复用）。"""
        types = list(types or [])
        for k, i in enumerate(self.indices_of(robot)):
            if k < len(types) and types[k] is not None:
                if i < len(self.point_types):
                    self.point_types[i] = types[k]
        self._fill_missing_types()

    def toPlainText(self):
        return self.code_edit.toPlainText()

    def setPlainText(self, text):
        self.code_edit.setPlainText(text)

    def set_joint_names(self, names):
        """设置关节名称顺序，用于解析代码"""
        self.joint_names = list(names)

    #: 段级归属头：出现一次后，其后的指令都归这台，直到下一个 Robot 头出现
    _ROBOT_HEADER_RE = re.compile(r"^#\s*Robot\s*:\s*(.+?)\s*$")

    def _parse_code_to_positions(self):
        """从代码编辑器解析点位，**并记下每条指令的行号与归属**。

        只抓出数值是不够的：「从选中行开始跑」「只跑选中那台」都要求知道每条
        指令在代码里的位置、以及它属于哪台机器人，所以这里逐行扫描，而不是
        对全文 ``findall``——后者会把位置和归属一起丢掉。
        """
        code_text = self.code_edit.toPlainText()
        # 语义不在代码文本里，解析会重建位置列表——先存下来，数量一致就沿用它，
        # 否则用户打的「抓取点 / 放置点」标记会在每次 Play 时被抹掉
        old_types = list(self.point_types)
        self.recorded_positions.clear()
        self.point_robots.clear()
        self._line_map.clear()

        # 匹配 movej([value1, value2, ...], vel=X.X) 格式
        pattern = r"movej\(\s*\[([^\]]+)\]\s*(?:,\s*vel\s*=\s*[\d.]+)?\s*\)"

        robot = self.owner_robot  # 没有 Robot 头时，整份退回这个归属
        for line_no, line in enumerate(code_text.splitlines(), start=1):
            header = self._ROBOT_HEADER_RE.match(line)
            if header:
                robot = header.group(1)
                continue
            found = re.search(pattern, line)
            if not found:
                continue
            try:
                values = [float(v.strip()) for v in found.group(1).split(",")]
                # 将角度转换为弧度
                rad_values = [math.radians(v) for v in values]

                if self.joint_names and len(values) == len(self.joint_names):
                    # 如果有关节名称，创建字典
                    pos_dict = {name: rad_values[i] for i, name in enumerate(self.joint_names)}
                elif len(values) > 0:
                    # 如果没有关节名称，使用索引作为键
                    pos_dict = {f"joint_{i+1}": rad_values[i] for i in range(len(rad_values))}
                else:
                    continue
                # 行号与归属必须与点值同生共死：它们是"从哪行起跑""跑哪台"的依据
                self._line_map.append((line_no, robot, len(self.recorded_positions)))
                self.recorded_positions.append(pos_dict)
                self.point_robots.append(robot)
            except ValueError:
                continue

        # 更新位置计数显示（带上各台各多少点）
        self._update_count_label()

        if len(old_types) == len(self.recorded_positions):
            self.point_types = old_types
        else:
            # 点数变了就重建：默认的五点式是**每台各一套**，按整份点数取会
            # 让第二台拿到一串途经点，循环里没有抓取点
            self.point_types = []
            self.sync_point_types()
        self.refresh_scope()

    # ---------------- 播放范围：跑哪台、从哪行起 ---------------- #

    def robot_at(self, index: int) -> Optional[str]:
        """第 ``index`` 个示教点归属哪台机器人（无映射信息时退回整份归属）。"""
        if 0 <= index < len(self._line_map):
            return self._line_map[index][1]
        if 0 <= index < len(self.point_robots):
            return self.point_robots[index]
        return self.owner_robot

    def line_of(self, index: int) -> int:
        """第 ``index`` 个示教点在代码里的行号（1 起）。"""
        if 0 <= index < len(self._line_map):
            return self._line_map[index][0]
        return index + 1

    def _cursor_line(self) -> int:
        """代码光标所在行（1 起）——"从选中行号处运行"的行号来源。"""
        return self.code_edit.textCursor().blockNumber() + 1

    def _selected_robot(self) -> Optional[str]:
        """**当前选中对象**关联的机器人；没选中或关联不到则 ``None``。"""
        provider = getattr(self, "selection_robot_provider", None)
        return provider() if callable(provider) else None

    def playback_plan(self, robot: Optional[str] = None, from_cursor: bool = False) -> List[int]:
        """算出这次播放要走的**点索引序列**（顺序与代码行序一致）。

        Args:
            robot: 只跑这台机器人的指令；``None`` 表示不挑机器人
            from_cursor: 是否从光标所在行开始——未选中设备时的「全局运行」
        """
        seq = list(range(len(self.recorded_positions)))
        if robot:
            # 选中了机器人（或挂在它上面的夹爪/工件）：只跑它的指令。
            seq = [i for i in seq if self.robot_at(i) == robot]
        if from_cursor:
            line = self._cursor_line()
            seq = [i for i in seq if self.line_of(i) >= line]
        return seq

    def refresh_scope(self) -> None:
        """刷新「将要执行的范围」——按下 Play 之前就该知道跑的是谁、从哪开始。"""
        if not hasattr(self, "scope_label"):
            return
        total = len(self.recorded_positions)
        if not total:
            self.scope_label.setText("")
            return
        robot = self._selected_robot()
        if robot:
            seq = self.playback_plan(robot=robot)
            self.scope_label.setText(
                tr("将运行：{robot} · {n}/{total} 点").format(robot=robot, n=len(seq), total=total)
            )
        else:
            seq = self.playback_plan(from_cursor=True)
            self.scope_label.setText(
                tr("将从第 {line} 行起全局运行（{n}/{total} 点）").format(
                    line=self._cursor_line(), n=len(seq), total=total
                )
            )

    def _flush_code_edit(self) -> None:
        """手改代码停手后：重解析 → 广播，让 3D 路径与面板点表跟着刷新。

        只在**没在播放**时刷：播到一半被解析改写点表，回放序列与点表会错位，
        机器人直接跳到别处。
        """
        if self.playback_timer is not None and self.playback_timer.isActive():
            return
        self._parse_code_to_positions()
        self.pos_count_label.setText(tr("示教点：{n}").format(n=len(self.recorded_positions)))
        self.refresh_scope()
        # 主窗口已接 on_positions_parsed：视口路径 + 离线编程面板点表
        self.positionsParsed.emit()

    def line_of_point(self, index: int, robot: Optional[str] = None):
        """第 ``index`` 个示教点（属于 ``robot``）在代码里的行号。

        「机器人走到哪一点，代码就高亮到哪一行」靠这条反查：点索引与行号不是
        一回事——圆弧的中间点不单独出指令，手工改过的代码行号还会整体偏移。

        Args:
            index: 点表里的下标（0 起）
            robot: 该点归属的机器人；``None`` 表示只按序号找

        Returns:
            行号（1 起）；查不到返回 ``None``
        """
        self._parse_code_to_positions()
        loose = None
        for ln, owner, point_index in self._line_map:
            if point_index != index:
                continue
            if robot is None or owner == robot:
                return ln
            # 归属对不上（单机代码没有 ``# Robot:`` 头时 owner 是 None）：
            # 先记住，全部找完再用——有且只有一个匹配点时序号就是行号
            loose = ln if loose is None else loose
        if loose is not None:
            return loose
        # 点表里压根没有行号信息（手打的/刚解析的），退回按序号估算
        return self.line_of(index) if index < len(self.recorded_positions) else None

    def _line_target(self, line: int):
        """第 ``line`` 行对应的 ``(机器人名, 点索引)``；不可执行返回 ``None``。

        先解析一遍代码：手改过的文本要先落到 :attr:`_line_map`，行号才对得上。
        """
        self._parse_code_to_positions()
        for ln, robot, index in self._line_map:
            if ln == line:
                return (robot, index)
        return None

    def run_line(self, line: Optional[int] = None) -> bool:
        """把机器人驱动到第 ``line`` 行（缺省光标行）对应的示教点。

        与「播放」的区别：播是从某行**一直走到尾**，这里是**只走这一行**——
        逐行核对抓取/放置时机、检查某点是否可达时最快。

        Args:
            line: 代码行号（1 起）；``None`` 取光标所在行

        Returns:
            执行成功 ``True``；该行不可执行 ``False``（状态栏说明原因）
        """
        if line is None:
            line = self._cursor_line()
        target = self._line_target(line)
        if target is None:
            self._notify(tr("第 {line} 行不是可执行的 movej 指令").format(line=line))
            return False
        robot, index = target
        self.highlight_line(line)
        # 归属跟着点走：多机时每行的主人可能不同，认行不认"当前那台"
        self.playbackRequested.emit(index, robot or "")
        return True

    def _show_code_context_menu(self, pos) -> None:
        """代码区右键菜单：标准编辑项 +「执行此行」。"""
        menu = self.code_edit.createStandardContextMenu()
        line = self.code_edit.cursorForPosition(pos).blockNumber() + 1
        if self._line_target(line) is not None:
            menu.addSeparator()
            menu.addAction(tr("▶ 执行此行"), lambda: self.run_line(line))
        menu.exec(self.code_edit.mapToGlobal(pos))

    def highlight_line(self, line: int) -> None:
        """把光标移到第 ``line`` 行并整行选中——报错时指出是哪一行出的问题。"""
        block = self.code_edit.document().findBlockByNumber(max(0, line - 1))
        if not block.isValid():
            return
        cursor = QtGui.QTextCursor(block)
        cursor.movePosition(QtGui.QTextCursor.EndOfBlock, QtGui.QTextCursor.KeepAnchor)
        self.code_edit.setTextCursor(cursor)
