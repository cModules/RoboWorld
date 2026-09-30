"""主窗口（ADR-002 拆解第 10 步 / ADR-002 目标态）。

# TECH_DEBT: 存量 GUI 骨架超过 600 行，后续拆分菜单/UI 装配；一次只迁移一个职责。

从启动壳 ``main.py`` 整体搬来：菜单、布局、信号接线与窗口级状态初始化。
行为体（插件/缓存、真机连接、程序校验、机器人场景、回放、多机同步、工程会话、
零散动作）已先按职责拆成 ``app/window/`` 下的 mixin，本模块只留**骨架**并继承它们。

搬迁纪律与各组一致：方法体一字未改，只是换了位置；``self.*`` 全部照旧，
因此调用点（含既有测试与工具）零调整。启动壳保留 ``from app.window.main_window
import MainWindow`` 作为历史调用面。

一处**必须改**的地方：``_set_window_icon`` 必须使用 ``packaging/logo.png``；源码
运行时通过共享的仓库根定位，冻结包运行时则使用 PyInstaller 展开的根目录资源。
"""

from __future__ import annotations

import sys
from pathlib import Path

from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Qt
from PySide6.QtGui import QAction, QKeySequence

# 可选依赖（i18n / 版本号 / 场景 / 工作单元 / 抓取工艺 / 插件 / 工程文件 / 会话…）
# 全部由 ``app.window.capabilities`` 单点探测——本模块不再各留一份 try/except。
from app.window.capabilities import (
    HANDSHAKE_SIGNALS,
    ProgramPanel,
    Project,
    RobotLibraryPanel,
    Scene,
    ScenePanel,
    _CORE_VERSION,
    _HAS_CONTROLLER_PANEL,
    _HAS_INCREMENTAL,
    _HAS_PICKPLACE,
    _HAS_PLUGINS,
    _HAS_PROGRAM,
    _HAS_PROJECT,
    _HAS_PROJECT_TEMPLATES,
    _HAS_ROBOT_LIBRARY,
    _HAS_SESSION_SUPPORT,
    _HAS_TEMPLATES,
    _HAS_WORLD,
    _REPO_ROOT,
    _feature_enabled,
    available_languages,
    clear_autosave,
    current_language,
    set_language,
    tr,
)

# 视图组件（ADR-002：PySide6 的唯一归宿是 app/）
from app.views.gl_widget import GLWidget
from app.views.path_editor import PathCodeEditor
from app.views.property_panel import DevicePropertyPanel

# 行为体：MainWindow 的职责已按组拆成这些 mixin（顺序即 MRO）
from app.window.controller_link import ControllerLinkMixin
from app.window.misc_actions import MiscActionsMixin
from app.window.multi_robot_sync import MultiRobotSyncMixin
from app.window.playback import PlaybackMixin
from app.window.plugin_cache import PluginCacheMixin
from app.window.program_ops import ProgramOpsMixin
from app.window.project_session import ProjectSessionMixin
from app.window.robot_scene import RobotSceneMixin


class MainWindow(
    MiscActionsMixin,
    ProjectSessionMixin,
    PlaybackMixin,
    MultiRobotSyncMixin,
    PluginCacheMixin,
    ControllerLinkMixin,
    ProgramOpsMixin,
    RobotSceneMixin,
    QtWidgets.QMainWindow,
):
    #: 自动保存间隔（毫秒）。有未保存改动时每 60s 落一份快照——崩溃最多
    #: 丢一个间隔内的活，且不影响手动保存（快照写到独立目录，不碰工程）。
    AUTOSAVE_INTERVAL_MS = 60_000

    def __init__(self, urdf_path=None):
        super().__init__()
        #: 当前工程（v1.3.4）：`.core` 工程文件的内存形态，窗口标题显示它
        self.project = Project() if _HAS_PROJECT else None
        #: 有未保存改动（标题里的"•未保存"）
        self._project_dirty = False
        self.setWindowTitle(f"RobotWorld Terminal v{_CORE_VERSION}")
        self.resize(1400, 900)
        self._set_window_icon()
        self.setup_menu()
        self.setup_ui()
        self.urdf_path = urdf_path
        #: 供离线编程面板使用的 :class:`core.Robot`（含运动学）
        self.olp_robot = None
        #: 各机器人**抓取的工件**，键为机器人名。理由同上：工件下拉也是全局
        #: 一份，多机同时回放时不分开就会两台抢同一个工件（A 刚夹起就被 B 抢走）
        self._workpiece_by_robot: dict = {}
        #: 各机器人的**示教点语义**（PICK/PLACE…），键为机器人名。
        #: 路径编辑器一次只显示一台的点表，它的 ``point_types`` 就是"当前那台"
        #: 的；切换机器人时必须把旧的存进来、新的取出来，否则 B 会顶着 A 的
        #: 抓取语义出码——B 的程序里根本没有夹爪指令。
        self._point_types_by_robot: dict = {}
        #: 「真机连接」模式对话框（v1.1.1）：首次打开时创建，之后复用
        self._controller_dialog = None
        self._controller_panel = None
        # 会话支撑（v1.3.5）：自动保存定时器 + 恢复询问的一次性闸门
        self._recovery_prompted = False
        self._autosave_timer = None
        if _HAS_SESSION_SUPPORT:
            self._autosave_timer = QtCore.QTimer(self)
            self._autosave_timer.setInterval(self.AUTOSAVE_INTERVAL_MS)
            self._autosave_timer.timeout.connect(self._autosave_tick)
            self._autosave_timer.start()
        self._update_window_title()

    def showEvent(self, event):
        super().showEvent(event)
        if self.urdf_path and Path(self.urdf_path).exists():
            self.load_robot(self.urdf_path)
        elif hasattr(self, "library_panel"):
            self._load_default_robot()
        self.gl_widget.update()
        self.maybe_prompt_recovery()

    def closeEvent(self, event):
        super().closeEvent(event)
        # 干净退出（没有未保存改动）→ 自动保存快照已无意义，清掉，
        # 免得下次启动被一个过时的"崩溃恢复"误导；有未保存改动则留着——
        # 正常关闭路径之外（被杀/崩溃）这正是要恢复的东西。
        if _HAS_SESSION_SUPPORT and not self._project_dirty:
            clear_autosave()

    def _setup_status_bar(self) -> None:
        """状态栏常驻显示 logo 与节拍、干涉、IO、报错指标。"""
        self.status_logo = QtWidgets.QLabel(self)
        logo_path = self._logo_path()
        if logo_path.is_file():
            pixmap = QtGui.QPixmap(str(logo_path))
            if not pixmap.isNull():
                self.status_logo.setPixmap(
                    pixmap.scaled(18, 18, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                )
                self.status_logo.setToolTip(tr("RobotWorld"))
        self.status_logo.setFixedSize(22, 20)

        self.status_cycle = QtWidgets.QLabel(tr("节拍 —"))
        self.status_collision = QtWidgets.QLabel(tr("干涉 —"))
        self.status_io = QtWidgets.QLabel(tr("IO —"))
        self.status_error = QtWidgets.QLabel(tr("无报错"))

        self.statusBar().addPermanentWidget(self.status_logo)
        for w in (self.status_cycle, self.status_collision, self.status_io, self.status_error):
            w.setStyleSheet("color:#c8cad4; padding:0 8px; font-size:11px;")
            self.statusBar().addPermanentWidget(w)

    def _on_report_ready(self, report) -> None:
        """校验报告出来后刷新节拍与报错。"""
        cycle = getattr(report, "cycle_time", None)
        self.status_cycle.setText(
            tr("节拍 {t} s").format(t=f"{cycle:.2f}") if cycle is not None else tr("节拍 —")
        )
        issues = list(getattr(report, "issues", None) or [])
        if issues:
            self.status_error.setText(tr("报错 {n}").format(n=len(issues)))
            self.status_error.setStyleSheet("color:#ff8a80; padding:0 8px; font-size:11px;")
        else:
            self.status_error.setText(tr("无报错"))
            self.status_error.setStyleSheet("color:#a5d6a7; padding:0 8px; font-size:11px;")
        # 校验产出新轨迹 → 把新上下文交给渲染插件（v1.0.4）
        self._refresh_plugin_overlays()

    # ------------------------------------------------------------------ #
    # 工程文件（.core，v1.3.4）
    # ------------------------------------------------------------------ #

    # ---------------- 最近打开（v1.3.5） ---------------- #

    # ---------------- 崩溃自动恢复（v1.3.5） ---------------- #

    # ---------------- 从模板新建工程（v1.3.6） ---------------- #

    def setup_menu(self):
        menubar = self.menuBar()

        file_menu = menubar.addMenu(tr("文件(&F)"))
        open_action = QAction(tr("打开模型(&O)…"), self)
        open_action.setShortcut(QKeySequence.Open)
        open_action.triggered.connect(self.open_model)
        file_menu.addAction(open_action)

        save_action = QAction(tr("保存路径代码(&S)…"), self)
        save_action.setShortcut(QKeySequence.Save)
        save_action.triggered.connect(self.save_path_code)
        file_menu.addAction(save_action)

        load_action = QAction(tr("载入路径代码(&L)…"), self)
        load_action.setShortcut("Ctrl+O")
        load_action.triggered.connect(self.load_path_code)
        file_menu.addAction(load_action)

        if _HAS_PROJECT:
            file_menu.addSeparator()
            open_project_action = QAction(tr("打开工程(&O)…"), self)
            open_project_action.setShortcut("Ctrl+Shift+O")
            open_project_action.setToolTip(
                tr(
                    "打开 .core 工程文件：一次载入场景、示教程序（含抓取/放置语义）、"
                    "工艺时序、设备点表与零件库来源。\n"
                    "缺段/坏段只跳过那一段并在报告里说明——不会因为一段格式问题打不开整个工程。"
                )
            )
            open_project_action.triggered.connect(self.open_project)
            file_menu.addAction(open_project_action)

            save_project_action = QAction(tr("保存工程(&S)…"), self)
            save_project_action.setShortcut("Ctrl+Shift+S")
            save_project_action.setToolTip(
                tr(
                    "把当前工作会话存成一个 .core 工程文件（原子写入）。\n"
                    "本库不认识的段与字段会原样保留，往返一趟不丢内容。"
                )
            )
            save_project_action.triggered.connect(self.save_project)
            file_menu.addAction(save_project_action)

            if _HAS_PROJECT_TEMPLATES:
                new_tpl_action = QAction(tr("从模板新建工程(&N)…"), self)
                new_tpl_action.setToolTip(
                    tr(
                        "从内置模板（抓取工作站 / 分拣工位 / 机床上下料 / 3C 装配 / 汽车焊装）新建工程：\n"
                        "场景与起步程序一步到位，不必从空白开始。\n"
                        "当前工程有未保存改动时会先询问。"
                    )
                )
                new_tpl_action.triggered.connect(lambda: self.new_project_from_template())
                file_menu.addAction(new_tpl_action)

            if _HAS_SESSION_SUPPORT:
                file_menu.addSeparator()
                self._recent_menu = file_menu.addMenu(tr("最近打开(&R)"))
                self._recent_menu.setToolTip(
                    tr(
                        "最近打开过的 .core 工程文件（最新在前，最多 10 条）。\n"
                        "文件已不存在的条目会置灰，不会被悄悄藏掉。"
                    )
                )
                # 展开时重建：文件被移走/改名后，失效条目立即置灰
                self._recent_menu.aboutToShow.connect(self._rebuild_recent_menu)
                self._rebuild_recent_menu()

        if _HAS_WORLD:
            file_menu.addSeparator()
            cell_action = QAction(tr("载入预置工作站(&W)..."), self)
            cell_action.triggered.connect(self.load_preset_cell)
            file_menu.addAction(cell_action)

        if _HAS_TEMPLATES and _HAS_PROGRAM and _HAS_PICKPLACE:
            demo_action = QAction(tr("抓取工作站演示（一键）(&D)"), self)
            demo_action.setToolTip(
                tr(
                    "一键载入抓取工作站（UR5e + 吸盘 + 输送带 + 托盘），\n"
                    "自动装上示例抓取程序并开始回放，状态栏显示节拍。"
                )
            )
            demo_action.triggered.connect(self.load_grasping_workstation_demo)
            file_menu.addAction(demo_action)

        file_menu.addSeparator()

        gen_action = QAction(tr("生成机器人代码(&G)…"), self)
        gen_action.setShortcut("Ctrl+G")
        gen_action.triggered.connect(self.generate_robot_code)
        file_menu.addAction(gen_action)

        # 真机连接（Socket / SSH）：从未上过真机验证，MVP 不敢当产品能力放出去
        if _HAS_CONTROLLER_PANEL and _feature_enabled("controller_link"):
            link_action = QAction(tr("真机连接…"), self)
            link_action.setToolTip(
                tr(
                    "连接真机控制器：部署当前程序、回读实际关节角、取回核对。\n"
                    "传输可选 Socket（core-socket 行协议）或 SSH（SFTP 传程序 + exec）。\n"
                    "无运动指令入口、无自由命令输入框——程序运动一律走部署。"
                )
            )
            link_action.triggered.connect(self.show_controller_dialog)
            file_menu.addAction(link_action)

        file_menu.addSeparator()

        exit_action = QAction(tr("退出(&X)"), self)
        exit_action.setShortcut(QKeySequence.Quit)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        view_menu = menubar.addMenu(tr("视图(&V)"))
        reset_view = QAction(tr("重置视图(&R)"), self)
        reset_view.setShortcut("R")
        reset_view.triggered.connect(self.reset_view)
        view_menu.addAction(reset_view)

        if _HAS_ROBOT_LIBRARY:
            view_menu.addSeparator()
            show_lib = QAction(tr("显示&机器组件库"), self)
            show_lib.setShortcut("Ctrl+L")
            show_lib.triggered.connect(self.show_robot_library)
            view_menu.addAction(show_lib)

            reload_lib = QAction(tr("重新扫描机器组件库"), self)
            reload_lib.setShortcut("F5")
            reload_lib.triggered.connect(self.reload_robot_library)
            view_menu.addAction(reload_lib)

        if _HAS_WORLD:
            view_menu.addSeparator()
            check_col = QAction(tr("工作单元干涉检查"), self)
            check_col.setShortcut("F6")
            check_col.triggered.connect(self.check_collisions)
            view_menu.addAction(check_col)

        if _HAS_PROGRAM:
            if not _HAS_WORLD:
                view_menu.addSeparator()
            validate_prog = QAction(tr("程序校验与节拍估算"), self)
            validate_prog.setShortcut("F7")
            validate_prog.triggered.connect(self.validate_program)
            view_menu.addAction(validate_prog)
            # 多机时一台一台切过去按 F7 太笨：一键把全部机器人校完并汇总
            validate_all_prog = QAction(tr("校验全部机器人"), self)
            validate_all_prog.setShortcut("Shift+F7")
            validate_all_prog.triggered.connect(self.validate_all_programs)
            view_menu.addAction(validate_all_prog)
            grasp_wizard = QAction(tr("抓取向导…"), self)
            grasp_wizard.setShortcut("Ctrl+Shift+G")
            grasp_wizard.triggered.connect(self.open_grasp_wizard)
            view_menu.addAction(grasp_wizard)

        # 插件生态未成型，属开发者项
        if _HAS_PLUGINS and _feature_enabled("plugins"):
            view_menu.addSeparator()
            self.action_plugin_overlays = QAction(tr("插件覆盖层"), self)
            self.action_plugin_overlays.setCheckable(True)
            self.action_plugin_overlays.setChecked(True)
            self.action_plugin_overlays.toggled.connect(self.toggle_plugin_overlays)
            view_menu.addAction(self.action_plugin_overlays)

            reload_plugins = QAction(tr("重新扫描已安装插件"), self)
            reload_plugins.triggered.connect(self.reload_plugins)
            view_menu.addAction(reload_plugins)

            show_plugins = QAction(tr("插件状态…"), self)
            show_plugins.triggered.connect(self.show_plugin_status)
            view_menu.addAction(show_plugins)

        # 缓存状态 / 清空缓存：纯开发者项
        if _HAS_INCREMENTAL and _feature_enabled("dev_tools"):
            view_menu.addSeparator()
            show_cache = QAction(tr("缓存状态…"), self)
            show_cache.triggered.connect(self.show_cache_status)
            view_menu.addAction(show_cache)

            clear_cache = QAction(tr("清空缓存与统计"), self)
            clear_cache.triggered.connect(self.clear_caches)
            view_menu.addAction(clear_cache)

        # 语言 / Language（v1.0.10）：切换即保存配置，重启后全面生效
        lang_menu = menubar.addMenu(tr("语言 / Language"))
        self._lang_actions = {}
        for code, name in available_languages():
            act = QAction(name, self)
            act.setCheckable(True)
            act.setChecked(code == current_language())
            act.triggered.connect(lambda _checked=False, c=code: self._switch_language(c))
            self._lang_actions[code] = act
            lang_menu.addAction(act)

    def _switch_language(self, code: str) -> None:
        """切换界面语言并提示重启（面板文案在构造期写入，需重启全面生效）。"""
        if code == current_language():
            return
        set_language(code)
        for c, act in getattr(self, "_lang_actions", {}).items():
            act.setChecked(c == code)
        box = QtWidgets.QMessageBox(self)
        box.setWindowTitle(tr("语言 / Language"))
        box.setText(tr("切换语言需要重启应用才能完全生效。现在重启吗？"))
        restart_btn = box.addButton(tr("立即重启"), QtWidgets.QMessageBox.AcceptRole)
        box.addButton(tr("稍后"), QtWidgets.QMessageBox.RejectRole)
        box.exec()
        if box.clickedButton() is restart_btn:
            if getattr(sys, "frozen", False):
                argv = [a for a in sys.argv[1:] if not a.startswith("-psn")]
            else:
                argv = [sys.argv[0]] + [a for a in sys.argv[1:] if not a.startswith("-psn")]
            QtCore.QProcess.startDetached(sys.executable, argv)
            QtWidgets.QApplication.closeAllWindows()
            sys.exit(0)

    @staticmethod
    def _logo_path() -> Path:
        """返回源码树或冻结包中的 logo 资源路径。"""
        if hasattr(sys, "_MEIPASS"):
            return Path(sys._MEIPASS) / "logo.png"
        return _REPO_ROOT / "packaging" / "logo.png"

    def _set_window_icon(self):
        """设置窗口图标与**应用级**图标。

        以源码方式启动（``python app.py``）时，macOS 把进程当作 Python 解释器，
        Dock 上显示 python 图标与 ``python3.11``。设置应用级图标后，非 bundle
        进程的 Dock 图标会取用该图标。

        Dock **名称**由进程名决定，Qt 层面无法覆盖——正式发布需打包成
        ``RobotWorld.app``，由 Info.plist 的 ``CFBundleName`` 确定（见 app.spec）。
        """
        import sys

        # 源码树中的 logo 位于 packaging/；冻结包由 app.spec 复制到资源根目录。
        logo_path = self._logo_path()
        if not logo_path.exists():
            return

        icon = QtGui.QIcon(str(logo_path))
        self.setWindowIcon(icon)  # 窗口标题栏
        QtGui.QGuiApplication.setWindowIcon(icon)  # 应用级：影响 macOS Dock

    def setup_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        main_layout = QtWidgets.QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(2)

        self.code_editor = PathCodeEditor()
        # 编辑器看不到视口选中状态：把"选中对象关联到哪台"的解析交给它，
        # 「只跑选中机器人」与「从选中行号处全局运行」这两个分支才做得出来
        self.code_editor.selection_robot_provider = self._robot_for_selection
        self.code_editor.playbackRequested.connect(self.on_playback_requested)
        self.code_editor.playbackStopped.connect(self.on_playback_stopped)
        self.code_editor.clearRequested.connect(self.on_clear_requested)
        self.code_editor.positionsParsed.connect(self.on_positions_parsed)
        # 工件下拉是全局一份，但"谁抓哪件"是**每台机器人**的事。选中的当下就
        # 记进本机名下——只在切换机器人时补记是不够的：用户给 A 选完工件若没
        # 切走过，A 名下就是空的，多机回放时两台会去抢同一件工件。
        if hasattr(self.code_editor, "workpiece_combo"):
            self.code_editor.workpiece_combo.currentIndexChanged.connect(
                self._on_workpiece_combo_changed
            )

        if _HAS_ROBOT_LIBRARY:
            # 机器人库（eCatalog）与路径代码编辑器共用一个标签页容器
            self.library_panel = RobotLibraryPanel(models_dir=_REPO_ROOT / "assets" / "models")
            self.library_panel.robotSelected.connect(self.load_robot)
            self.library_panel.componentActivated.connect(self._add_component_from_library)
            self.library_panel.statusMessage.connect(self.statusBar().showMessage)

            if _HAS_WORLD:
                # 注意：此时 gl_widget 尚未创建，信号连接在 setup_ui 末尾进行
                self.scene_panel = ScenePanel(Scene())
                self.scene_panel.statusMessage.connect(self.statusBar().showMessage)

            if _HAS_PROGRAM:
                self.program_panel = ProgramPanel()
                self.program_panel.statusMessage.connect(self.statusBar().showMessage)
                self.program_panel.playbackFrame.connect(self.on_playback_frame)
                self.program_panel.playbackFrameDTO.connect(self.on_playback_frame_dto)
                self.program_panel.playbackFinished.connect(self.on_playback_stopped)
                # 程序级回放到 gripper() 指令时，驱动工件吸附/释放（S3）
                self.program_panel.gripperStateChanged.connect(self.on_gripper_state_changed)
                # 轨迹编辑器（v0.2.4）改动示教点 → 回写路径代码编辑器
                self.program_panel.taughtPointsChanged.connect(self.on_taught_points_changed)
                # 面板要校验/回放前先要点最新数据（它的槽可能是旧快照）
                self.program_panel.syncRequested.connect(self._on_program_sync_requested)
                # 「生成代码」按回放同一套口径：选中设备 → 只出那台的；
                # 没选中 → 出全部。探针由主窗口注入（面板看不到视口选中）
                self.program_panel.selection_robot_provider = self._robot_for_selection
                # 校验会把场景关节短暂写走再恢复（见面板 _restore_scene_joints）：
                # 恢复之后视口得重画一次，夹爪才不会停留在半路
                self.program_panel.reportReady.connect(lambda _report: self.gl_widget.update())
                # 面板顶部「机器人」下拉换了人 → 视口/示教点显示同步到那台
                self.program_panel.activeRobotChanged.connect(self._on_program_robot_changed)

            # ---- 左：双 Tab（① 场景设备库 ② 代码预览）----
            # 左手边是"输入区"：Tab1 从库里挑设备拖进场景，Tab2 是紧贴视口的代码
            # 预览——看着机器人动、同时读/改 movej 那几行，改完当场看得见效果，
            # 不用在窗口两头来回横跳。设备树已挪到右侧（见下）。
            self.left_tabs = QtWidgets.QTabWidget()
            self.left_tabs.setMaximumWidth(400)
            self.left_tabs.setMinimumWidth(280)
            self.left_tabs.addTab(self.library_panel, tr("场景设备库"))
            self.left_tabs.addTab(self.code_editor, tr("代码预览"))
            main_layout.addWidget(self.left_tabs)
        else:
            # 无机器人库时左侧只剩代码预览；设备树放在右侧那组里。一个面板只能挂
            # 一次——在两边都挂的话会被 reparent 到后挂的那组，先挂的位置就空了。
            self.left_tabs = QtWidgets.QTabWidget()
            self.left_tabs.setMaximumWidth(400)
            self.left_tabs.setMinimumWidth(280)
            self.left_tabs.addTab(self.code_editor, tr("代码预览"))
            main_layout.addWidget(self.left_tabs)

        self.gl_widget = GLWidget(self)
        self.gl_widget.setMinimumSize(400, 300)
        main_layout.addWidget(self.gl_widget, 1)

        self.property_panel = DevicePropertyPanel()
        self.property_panel.jointChanged.connect(self.on_joint_changed)
        self.property_panel.jointChanged.connect(self.on_joint_changed_for_record)
        # 多机：给属性面板「按名字取渲染器」的能力。不给它，选中 A 时面板只能
        # 沿用最后加载那台的关节组——拖 A 的滑块却驱动 B（两台一起动）。
        self.property_panel.set_renderer_provider(self.gl_widget.renderer_of)

        # ---- 右：可折叠多功能标签页（结构 + 输出）----
        # 属性在最前（选中对象时自动切到它），设备树紧随其后：层级编排和属性本来
        # 就是同一件事的两面——树上点一下选中、这里改位姿，左右手不用换地方。
        self.right_tabs = QtWidgets.QTabWidget()
        self.right_tabs.setMaximumWidth(360)
        self.right_tabs.setMinimumWidth(260)
        self.right_tabs.addTab(self.property_panel, tr("属性"))
        if hasattr(self, "scene_panel"):
            self.right_tabs.addTab(self.scene_panel, tr("设备树"))
        if hasattr(self, "program_panel"):
            self.right_tabs.addTab(self.program_panel, tr("导出打包"))
        main_layout.addWidget(self.right_tabs)

        # 场景改动需要触发视口重绘，此连接必须等 gl_widget 建好后才能做
        if hasattr(self, "scene_panel"):
            self.scene_panel.sceneChanged.connect(self.gl_widget.update)
            # 「加载场景」整体替换场景引用时统一接管：视口/属性面板/工件下拉
            # 各持一份引用，漏掉任何一处都会出现"树里是新的、别处还是旧的"
            self.scene_panel.sceneReplaced.connect(self._adopt_scene)
            # 视口右键拾取/拖动选中对象时，同步到左侧场景面板树形选择
            self.gl_widget.objectSelected.connect(self.scene_panel._select_object)
            # 视口右键路径点「移动到」：把那一台驱动到该示教点
            self.gl_widget.pathPointMoveRequested.connect(self.on_path_point_move_requested)
            # 左键拖拽摆放完成（v1.1.7）：把"落在谁上面"说给状态栏听
            self.gl_widget.objectPlaced.connect(self.statusBar().showMessage)
            # 视口侧改动挂载关系（拖到法兰自动装夹 / 拖已装夹件自动解除）：
            # 属性面板回填 + 设备树层级跟着变 + 视口重绘
            self.gl_widget.sceneChanged.connect(self.property_panel.set_object)
            self.gl_widget.sceneChanged.connect(lambda _n: self.scene_panel._refresh_tree())
            self.gl_widget.sceneChanged.connect(lambda _n: self.gl_widget.update())
            # 选中对象（视口 / 场景树两条来源）同步到右侧属性面板
            self.property_panel.set_scene(self.scene_panel.scene)
            self.gl_widget.objectSelected.connect(self.property_panel.set_object)
            self.scene_panel.objectSelected.connect(self.property_panel.set_object)
            # 场景树选中也要让视口画黄色高亮（同步 gl_widget.selected_object）
            self.scene_panel.objectSelected.connect(
                lambda n: (
                    setattr(self.gl_widget, "selected_object", n),
                    self.gl_widget.set_current_robot(n),
                    self.gl_widget.update(),
                    # 选中一变，"跑哪台"就变了，范围提示要跟着换
                    self.code_editor.refresh_scope(),
                )
            )
            # 右面板编辑对象属性后重绘视口
            self.property_panel.objectChanged.connect(self.gl_widget.update)
            # 挂载关系变化后场景树的层级也要跟着变（挂到法兰的夹爪要显示在
            # 「机器人」节点下），否则树里仍显示在根层，与视口不一致
            self.property_panel.objectChanged.connect(self.scene_panel._refresh_tree)
            # 场景增删对象后同步「工件」下拉，否则新加的工件选不到
            self.scene_panel.sceneChanged.connect(self.code_editor.refresh_workpiece_combo)
            self.code_editor.set_scene(self.scene_panel.scene)
            # 机器组件库拖入视口：组件加入场景、机器人按 catalog key 载入
            self.gl_widget.componentDropped.connect(self._add_component_from_library)
            self.gl_widget.robotDropped.connect(self._on_robot_dropped)
            # 工艺时序（v1.3.1）：把点表里真实存在的信号名注入时序编辑器，
            # 校验才能查出"序列用了但点表里没有"的信号。
            # MVP 档没有时序编辑器（入口隐藏），这段连接一并跳过
            _seq_editor = getattr(self.scene_panel, "sequence_editor", None)
            if _seq_editor is not None:
                _seq_editor.set_available_signals(
                    sorted(
                        {sig for d in self.code_editor.cell.devices for sig in d.io_map}
                        | set(HANDSHAKE_SIGNALS)
                    )
                )
            # 工程未保存标记（v1.3.4）：会话里任何"值得存下来"的改动都置脏。
            # 刻意只挂这四条粗粒度信号——细粒度（每个对象位姿变化）噪声太大，
            # 会把标记变成常亮的装饰。
            self.scene_panel.sceneChanged.connect(self._set_project_dirty)
            # 机器人被移出场景 → 它的程序/轨迹/示教点/渲染器一并清除
            self.scene_panel.robotRemoved.connect(self._on_robot_removed)
            if _seq_editor is not None:
                _seq_editor.sequenceChanged.connect(self._set_project_dirty)
            self.code_editor.code_edit.textChanged.connect(self._set_project_dirty)

        if hasattr(self, "program_panel"):
            self.program_panel.taughtPointsChanged.connect(lambda *_args: self._set_project_dirty())

        self._setup_status_bar()
        if hasattr(self, "program_panel"):
            self.program_panel.reportReady.connect(self._on_report_ready)
            # 一键校验全部完成 → 出汇总
            if hasattr(self.program_panel, "allValidated"):
                self.program_panel.allValidated.connect(self._show_validation_summary)
            # 抓取向导写完点 → 按机器人名回灌视口
            if hasattr(self.program_panel, "graspCycleApplied"):
                self.program_panel.graspCycleApplied.connect(self._on_grasp_cycle_applied)

        self.statusBar().showMessage(tr("就绪"))
