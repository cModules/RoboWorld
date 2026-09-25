# TECH_DEBT: 存量 GUI 债务（ADR-002）——迁移到 orolp/app/views 之前冻结新功能；
# 清单见 tools/check_arch_rules.py 的 GUI_DEBT，口径见 docs/CODING_RULES.md §3.2。

"""真机连接面板（v1.1.1 建立，v1.1.4 支持双传输）：把真机连接变成界面能力。

v1.0.5 落地了真机 Socket 连接（``orolp-socket`` 行协议），v1.1.2 补齐了 SSH 形态
（SFTP 传程序 + SSH exec）——但两者都始终是**代码 API**，现场工程师不会去写
Python。本模块把最常用的几件事搬进 GUI：

* **连接**：主机 / 端口 / 超时（+ SSH 的用户名 / 密码 / 私钥 / 远端目录）→
  真实握手。状态区**按传输如实展示**：Socket 展示服务端自述、协议版本、关节数
  与能力表（协商结果直接可见，而不是藏在 ``caps`` 字典里）；SSH 没有常驻服务端
  也就没有能力协商，界面如实写"无能力协商 · 不支持关节回读"，不假装有一套。
* **部署**：把离线编程面板当前示教的程序（由宿主注入的 provider 提供）
  走 :meth:`Program.deploy` 下发——与「一键导出」共用同一条生成通路，
  不会出现"界面部署的代码和导出的代码不一样"。
* **回读**：读回控制器实际关节角并发出 :attr:`jointsRead` 信号——宿主
  （app.py）拿它把视口摆成真机当前位形，仿真 vs 真机的对比从这里开始。
  控制器不支持回读（``caps.feedback=false``、或 SSH 这类无回读通道的形态）时
  **如实显示**，绝不伪造数据。
* **核对**（v1.1.4）：把控制器上的程序原文**取回**与本次生成的代码逐字节比对。
  "部署报成功"与"控制器上确实躺着这份代码"是两件事，尤其 SSH 形态下程序落在
  远端文件系统里，值得点一下再下班。

为什么需要 SSH 传输（v1.1.4）
=============================
**相当一部分控制器不给上位机开放自定协议的 TCP socket，但都带着 SSH**——KUKA 的
KRC4 本体就是个 Linux 盒子，国产"控制器 PC 一体机"也是。对这类机器，"把程序文件
放进控制器文件系统"就是真实工作流，而 v1.1.2 的 :class:`~orolp.link.SSHLink`
此前只能在脚本里用。传输选择器把两种形态收进同一个面板：切过去，端口/超时默认值
自动跟着换（22 / 10s），SSH 专属字段才出现。

安全边界（刻意的设计选择）
==========================
GUI **不提供任何运动指令入口**——没有"发送关节角到控制器"按钮，也**不提供
SSH 自由命令输入框**（:meth:`SSHLink.run_command` 是脚本化部署流水线的能力）。
真机侧的 ``set_joints`` 与任意 exec 都能让机器人动起来，把它们做成一个文本
输入框等于在产线现场放一个"随意驱动机器人"的入口；程序运动一律走"部署经评审
的程序"这条可审计的路径。

测试用 :class:`~orolp.linkstub.ControllerStubServer`（真 socket）与
:class:`~orolp.sshstub.SSHStubServer`（真 SSH）做对端，全链路走真协议；
全部跑在 Qt offscreen 平台上。
"""
from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional

import numpy as np

from PySide6 import QtCore, QtWidgets

from .i18n import tr
from .link import (
    DEFAULT_SOCKET_PORT,
    DEFAULT_SOCKET_TIMEOUT,
    DEFAULT_SSH_PORT,
    DEFAULT_SSH_TIMEOUT,
    LinkError,
    SocketLink,
    SSHLink,
)

__all__ = [
    "ControllerPanel",
    "TRANSPORT_SOCKET",
    "TRANSPORT_SSH",
    "TRANSPORT_LABELS",
]

#: 传输形态标识（:attr:`ControllerPanel.transport` 的取值）
TRANSPORT_SOCKET = "socket"
TRANSPORT_SSH = "ssh"

#: 传输下拉框的显示文案（探测顺序即下拉顺序）
TRANSPORT_LABELS: Dict[str, str] = {
    TRANSPORT_SOCKET: "Socket（orolp-socket 行协议）",
    TRANSPORT_SSH: "SSH（SFTP 传程序 + exec）",
}

#: 每种传输的端口/超时默认值——切换传输时"还是默认值"的那一项跟着换，
#: 用户手工改过的值不碰（改过的值是他自己的现场参数，不该被我们覆盖）。
_TRANSPORT_DEFAULTS: Dict[str, Dict[str, float]] = {
    TRANSPORT_SOCKET: {"port": DEFAULT_SOCKET_PORT, "timeout": DEFAULT_SOCKET_TIMEOUT},
    TRANSPORT_SSH: {"port": DEFAULT_SSH_PORT, "timeout": DEFAULT_SSH_TIMEOUT},
}

#: SSH 默认远端程序目录（与 :class:`~orolp.link.SSHLink` 的构造默认值一致）
DEFAULT_SSH_REMOTE_DIR = "/tmp/orolp_programs"

#: 程序提供者签名：返回 ``{"program": Program, "post_processor": str|None}``，
#: 没有可部署的程序时返回 ``None``。宿主负责把面板与离线编程面板接起来。
ProgramProvider = Callable[[], Optional[Dict[str, Any]]]


class ControllerPanel(QtWidgets.QWidget):
    """「真机连接」面板：连接 / 部署 / 回读 / 核对，支持 Socket 与 SSH 两种传输。

    Signals:
        statusMessage(str): 一行状态，宿主通常接到主窗口状态栏
        jointsRead(object): 成功回读的关节角（**度**，``numpy.ndarray``）；
            宿主据此把视口摆成真机当前位形
        connectionChanged(bool): 连接建立 / 断开（部署与回读按钮的联动依据）
    """

    statusMessage = QtCore.Signal(str)
    jointsRead = QtCore.Signal(object)
    connectionChanged = QtCore.Signal(bool)

    def __init__(self, parent: Optional[QtWidgets.QWidget] = None) -> None:
        super().__init__(parent)
        # 两种传输共用一个槽位：面板关心的只是 ControllerLink 的四个方法，
        # 类型差异收敛在 _build_link() 一处。
        self._link: Optional[Any] = None
        self._robot: Any = None
        self._program_provider: Optional[ProgramProvider] = None
        self._last_deployment: Any = None
        self._last_ext: str = "src"
        self._last_verify: Optional[Dict[str, Any]] = None
        self._build_ui()
        self._apply_state()

    # ------------------------------------------------------------------ #
    # 宿主注入
    # ------------------------------------------------------------------ #

    def set_robot(self, robot: Any) -> None:
        """给 :class:`SocketLink` 配上运动学（服务端无位姿回读时正解 TCP 用）。

        连接建立后更换机器人不影响已建立的 socket——运动学只在本侧使用。
        SSH 形态没有回读通道，因此不持有运动学（也不接这个参数）。
        """
        self._robot = robot
        if self._link is not None and hasattr(self._link, "robot"):
            self._link.robot = robot

    def set_program_provider(self, provider: Optional[ProgramProvider]) -> None:
        """注入程序提供者（宿主把离线编程面板的当前程序递过来）。"""
        self._program_provider = provider

    # ------------------------------------------------------------------ #
    # 传输选择
    # ------------------------------------------------------------------ #

    @property
    def transport(self) -> str:
        """当前传输形态（:data:`TRANSPORT_SOCKET` / :data:`TRANSPORT_SSH`）。"""
        return str(self.combo_transport.currentData() or TRANSPORT_SOCKET)

    def set_transport(self, transport: str) -> None:
        """程序化切换传输（供宿主与测试使用，等同于用户拉下拉框）。

        Raises:
            ValueError: 未知传输名。宁可当场报错，也不要悄悄退回 Socket——
                "以为在连 SSH、实际发了 socket"在现场是查不动的问题。
        """
        if transport not in TRANSPORT_LABELS:
            raise ValueError(f"未知传输形态：{transport!r}（可选 {sorted(TRANSPORT_LABELS)}）")
        index = self.combo_transport.findData(transport)
        if index < 0:  # pragma: no cover - 目录与下拉项同源，防御第三方改写
            raise ValueError(f"传输 {transport!r} 未出现在下拉框中")
        self.combo_transport.setCurrentIndex(index)

    # ------------------------------------------------------------------ #
    # 状态
    # ------------------------------------------------------------------ #

    @property
    def link(self) -> Optional[Any]:
        """当前连接（未连接时 ``None``）；类型为 :class:`SocketLink` 或 :class:`SSHLink`。"""
        return self._link

    @property
    def is_connected(self) -> bool:
        """是否已建立连接。"""
        return self._link is not None and self._link.connected

    @property
    def last_joints(self) -> Optional[np.ndarray]:
        """最近一次成功回读的关节角（度）；从未回读过为 ``None``。"""
        return None if self._link is None else getattr(self._link, "_last_read_joints", None)

    @property
    def last_deployment(self) -> Any:
        """最近一次成功部署的结果（:class:`~orolp.link.Deployment`）；没有则 ``None``。"""
        return self._last_deployment

    @property
    def last_verify(self) -> Optional[Dict[str, Any]]:
        """最近一次「取回核对」的结果字典；没核对过为 ``None``。"""
        return self._last_verify

    # ------------------------------------------------------------------ #
    # UI
    # ------------------------------------------------------------------ #

    def _build_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        # ---- 连接 ----
        conn_box = QtWidgets.QGroupBox(tr("连接"))
        form = QtWidgets.QFormLayout(conn_box)
        self.combo_transport = QtWidgets.QComboBox()
        for key, label in TRANSPORT_LABELS.items():
            self.combo_transport.addItem(tr(label), key)
        self.edit_host = QtWidgets.QLineEdit("127.0.0.1")
        self.spin_port = QtWidgets.QSpinBox()
        self.spin_port.setRange(1, 65535)
        self.spin_port.setValue(DEFAULT_SOCKET_PORT)
        self.spin_timeout = QtWidgets.QDoubleSpinBox()
        self.spin_timeout.setRange(0.5, 60.0)
        self.spin_timeout.setSingleStep(0.5)
        self.spin_timeout.setValue(DEFAULT_SOCKET_TIMEOUT)
        self.spin_timeout.setSuffix(" s")
        form.addRow(tr("传输"), self.combo_transport)
        form.addRow(tr("主机"), self.edit_host)
        form.addRow(tr("端口"), self.spin_port)
        form.addRow(tr("超时"), self.spin_timeout)

        # SSH 专属字段：只在选中 SSH 传输时出现（Socket 形态下这些值无意义，
        # 摆在界面上只会让人以为"连 socket 还要填密码"）。
        self.ssh_box = QtWidgets.QWidget()
        ssh_form = QtWidgets.QFormLayout(self.ssh_box)
        ssh_form.setContentsMargins(0, 0, 0, 0)
        self.edit_user = QtWidgets.QLineEdit("root")
        self.edit_password = QtWidgets.QLineEdit("")
        self.edit_password.setEchoMode(QtWidgets.QLineEdit.Password)
        self.edit_password.setPlaceholderText(tr("与私钥二选一"))
        self.edit_keyfile = QtWidgets.QLineEdit("")
        self.edit_keyfile.setPlaceholderText(tr("私钥文件路径，可留空"))
        self.edit_remote_dir = QtWidgets.QLineEdit(DEFAULT_SSH_REMOTE_DIR)
        ssh_form.addRow(tr("用户名"), self.edit_user)
        ssh_form.addRow(tr("密码"), self.edit_password)
        ssh_form.addRow(tr("私钥"), self.edit_keyfile)
        ssh_form.addRow(tr("远端目录"), self.edit_remote_dir)
        form.addRow(self.ssh_box)

        btn_row = QtWidgets.QHBoxLayout()
        self.btn_connect = QtWidgets.QPushButton(tr("连接"))
        self.btn_disconnect = QtWidgets.QPushButton(tr("断开"))
        btn_row.addWidget(self.btn_connect, 1)
        btn_row.addWidget(self.btn_disconnect, 1)
        form.addRow(btn_row)

        self.label_state = QtWidgets.QLabel(tr("未连接"))
        self.label_state.setStyleSheet("color:#c8cad4; padding:2px 0;")
        form.addRow(self.label_state)
        self.label_info = QtWidgets.QLabel("")
        self.label_info.setWordWrap(True)
        self.label_info.setStyleSheet("color:#90caf9; font-size:11px;")
        form.addRow(self.label_info)
        layout.addWidget(conn_box)

        # ---- 程序部署 ----
        deploy_box = QtWidgets.QGroupBox(tr("程序部署"))
        deploy_layout = QtWidgets.QVBoxLayout(deploy_box)
        self.btn_deploy = QtWidgets.QPushButton(tr("部署当前程序"))
        deploy_layout.addWidget(self.btn_deploy)
        self.list_programs = QtWidgets.QListWidget()
        self.list_programs.setMaximumHeight(110)
        deploy_layout.addWidget(self.list_programs)
        verify_row = QtWidgets.QHBoxLayout()
        self.btn_refresh = QtWidgets.QPushButton(tr("刷新列表"))
        self.btn_verify = QtWidgets.QPushButton(tr("取回核对"))
        self.btn_verify.setToolTip(
            tr("把控制器上的程序原文取回，与本次生成的代码逐字节比对——\n"
               "「部署报成功」与「控制器上确实是这份代码」是两件事")
        )
        verify_row.addWidget(self.btn_refresh, 1)
        verify_row.addWidget(self.btn_verify, 1)
        deploy_layout.addLayout(verify_row)
        layout.addWidget(deploy_box)

        # ---- 回读 ----
        fb_box = QtWidgets.QGroupBox(tr("回读"))
        fb_layout = QtWidgets.QVBoxLayout(fb_box)
        self.btn_read = QtWidgets.QPushButton(tr("回读关节"))
        fb_layout.addWidget(self.btn_read)
        self.label_joints = QtWidgets.QLabel(tr("关节角 —"))
        self.label_joints.setWordWrap(True)
        self.label_joints.setStyleSheet("font-family:Menlo,monospace; font-size:11px;")
        fb_layout.addWidget(self.label_joints)
        layout.addWidget(fb_box)

        # ---- 日志 ----
        log_box = QtWidgets.QGroupBox(tr("日志"))
        log_layout = QtWidgets.QVBoxLayout(log_box)
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        log_layout.addWidget(self.log_view)
        layout.addWidget(log_box, 1)

        # ---- 信号 ----
        self.combo_transport.currentIndexChanged.connect(self._on_transport_changed)
        self.btn_connect.clicked.connect(self._on_connect_clicked)
        self.btn_disconnect.clicked.connect(self._on_disconnect_clicked)
        self.btn_deploy.clicked.connect(self._on_deploy_clicked)
        self.btn_refresh.clicked.connect(self._on_refresh_clicked)
        self.btn_verify.clicked.connect(self._on_verify_clicked)
        self.btn_read.clicked.connect(self._on_read_clicked)

        self._apply_transport_visibility()

    # ------------------------------------------------------------------ #
    # 传输切换
    # ------------------------------------------------------------------ #

    def _on_transport_changed(self) -> None:
        self._apply_transport_visibility()

    def _apply_transport_visibility(self) -> None:
        """按当前传输显示/隐藏 SSH 专属字段，并把"仍是默认值"的端口/超时换成新默认。

        只动"还等于另一传输默认值"的那些框：用户手工填过的现场参数，
        切换传输不该被我们改掉。
        """
        transport = self.transport
        self.ssh_box.setVisible(transport == TRANSPORT_SSH)
        # QFormLayout 对隐藏行的高度处理在不同 Qt 版本上不一致——能用
        # setRowVisible 就显式收起整行，避免留一条空行。
        form = self.ssh_box.parentWidget().layout() if self.ssh_box.parentWidget() else None
        if isinstance(form, QtWidgets.QFormLayout) and hasattr(form, "setRowVisible"):
            form.setRowVisible(self.ssh_box, transport == TRANSPORT_SSH)

        defaults = _TRANSPORT_DEFAULTS[transport]
        others = [
            values for key, values in _TRANSPORT_DEFAULTS.items() if key != transport
        ]
        if any(int(v["port"]) == self.spin_port.value() for v in others):
            self.spin_port.setValue(int(defaults["port"]))
        if any(abs(float(v["timeout"]) - self.spin_timeout.value()) < 1e-9 for v in others):
            self.spin_timeout.setValue(float(defaults["timeout"]))
        if transport == TRANSPORT_SSH and self._link is None:
            # SSH 形态没有回读通道——先如实说，别等用户点了才发现
            self.label_joints.setText(tr("SSH 形态无回读通道"))
        elif transport == TRANSPORT_SOCKET and self._link is None:
            self.label_joints.setText(tr("关节角 —"))

    def _apply_state(self) -> None:
        """按连接状态统一启停按钮——状态只有一种来源，不散落在各个回调里。"""
        connected = self.is_connected
        self.btn_connect.setEnabled(not connected)
        self.btn_disconnect.setEnabled(connected)
        for btn in (self.btn_deploy, self.btn_refresh, self.btn_verify, self.btn_read):
            btn.setEnabled(connected)
        self.label_state.setText(tr("已连接") if connected else tr("未连接"))
        self.label_state.setStyleSheet(
            "color:#a5d6a7; padding:2px 0;" if connected else "color:#c8cad4; padding:2px 0;"
        )

    def _log(self, message: str) -> None:
        """带时刻的日志行——现场排障时"什么时候发生的"和"发生了什么"一样重要。"""
        stamp = QtCore.QTime.currentTime().toString("HH:mm:ss")
        self.log_view.appendPlainText(f"[{stamp}] {message}")

    # ------------------------------------------------------------------ #
    # 动作
    # ------------------------------------------------------------------ #

    def _on_connect_clicked(self) -> None:
        self.connect_to_controller()

    def _build_link(self) -> Any:
        """按当前输入构造连接对象（构造不发网络请求，connect() 才发）。

        Socket 与 SSH 的形态差异只在这里出现——面板其余部分（部署/列表/核对）
        走的是 :class:`~orolp.link.ControllerLink` 的同一组方法。
        """
        host = self.edit_host.text().strip() or "127.0.0.1"
        port = int(self.spin_port.value())
        timeout = float(self.spin_timeout.value())
        if self.transport == TRANSPORT_SSH:
            return SSHLink(
                host,
                port,
                username=self.edit_user.text().strip() or "root",
                password=self.edit_password.text() or None,
                key_filename=self.edit_keyfile.text().strip() or None,
                timeout=timeout,
                remote_dir=self.edit_remote_dir.text().strip() or DEFAULT_SSH_REMOTE_DIR,
            )
        return SocketLink(host, port, timeout=timeout, robot=self._robot)

    def _describe_link(self, link: Any) -> str:
        """状态区那一行——**按传输如实展示**，不把没有的能力写成有。

        Socket 有握手，因此服务端自述 / 协议版本 / 关节数 / 能力表都是协商来的
        事实；SSH 没有常驻对端服务，也就没有可协商的东西：远端目录是配置项，
        回读能力**不存在**（孪生偏差链路仍得走 Socket 或厂商协议）。
        """
        if isinstance(link, SSHLink):
            return tr("SSH 无能力协商 · 远端目录 {dir} · 不支持关节回读").format(
                dir=link.remote_dir
            )
        caps = ",".join(k for k, v in sorted(link.caps.items()) if v) or "-"
        dof = "-" if link.dof is None else str(link.dof)
        server = str(link.server_info.get("server") or link.server_info.get("name") or tr("未知"))
        return tr("服务端 {server} · 协议 v{api} · 关节数 {dof} · 能力 {caps}").format(
            server=server, api=1, dof=dof, caps=caps
        )

    def connect_to_controller(self) -> bool:
        """按当前输入建立连接（Socket 握手 / SSH 登录 + 建远端目录）。

        Returns:
            是否成功。失败时日志里给出原因（含打桩服务端的启动提示），
            界面回到未连接状态，绝不假装连上了。
        """
        transport = self.transport
        host = self.edit_host.text().strip() or "127.0.0.1"
        port = int(self.spin_port.value())
        self._log(
            tr("正在连接 {transport} {host}:{port} …").format(
                transport=TRANSPORT_LABELS[transport], host=host, port=port
            )
        )
        link = self._build_link()
        try:
            link.connect()
        except LinkError as exc:
            self._link = None
            self._log(tr("连接失败：{exc}").format(exc=exc))
            self.statusMessage.emit(tr("连接失败：{exc}").format(exc=exc))
            self._apply_state()
            return False
        except Exception as exc:  # paramiko 服务端异常等——按连接失败处理，不放任崩 GUI
            self._link = None
            detail = f"{type(exc).__name__}: {exc}"
            self._log(tr("连接失败：{exc}").format(exc=detail))
            self.statusMessage.emit(tr("连接失败：{exc}").format(exc=detail))
            self._apply_state()
            return False
        self._link = link
        self.label_info.setText(self._describe_link(link))
        self._log(tr("已连接：{desc}").format(desc=link.describe()))
        self.statusMessage.emit(tr("已连接 {name}").format(name=link.name))
        self._apply_state()
        self.connectionChanged.emit(True)
        self._on_refresh_clicked()
        return True

    def _on_disconnect_clicked(self) -> None:
        self.disconnect_from_controller()

    def disconnect_from_controller(self) -> None:
        """断开连接（发 ``bye`` 尽力而为）。对端已经走了也算断开成功。"""
        if self._link is None:
            self._apply_state()
            return
        name = self._link.name
        try:
            self._link.disconnect()
        except LinkError:  # pragma: no cover - disconnect 不抛，防御第三方子类
            pass
        self._link = None
        self.label_info.setText("")
        self._log(tr("已断开 {name}").format(name=name))
        self.statusMessage.emit(tr("已断开 {name}").format(name=name))
        self._apply_state()
        self.connectionChanged.emit(False)

    def _on_deploy_clicked(self) -> None:
        self.deploy_current_program()

    def deploy_current_program(self):
        """把宿主提供的当前程序下发到控制器。

        走 :meth:`Program.deploy`（生成代码 + 部署 + 清单），与「一键导出」
        同一条生成通路。业务性拒绝（服务端 ``ok:false``）只记日志——链路还
        活着，一次程序名笔误不该废掉整个会话。

        Returns:
            :class:`~orolp.link.Deployment`；未连接或没有程序时 ``None``
        """
        if not self.is_connected:
            self._log(tr("未连接控制器——请先建立连接"))
            return None
        if self._program_provider is None:
            self._log(tr("未设置程序来源——请从离线编程面板打开本窗口"))
            return None
        payload = self._program_provider()
        if payload is None:
            self._log(tr("没有可部署的程序——请先加载机器人并示教至少一个点"))
            return None
        program = payload.get("program")
        if program is None:
            self._log(tr("没有可部署的程序——请先加载机器人并示教至少一个点"))
            return None
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            deployment = program.deploy(
                self._link,
                post_processor=payload.get("post_processor") or None,
            )
        except LinkError as exc:
            self._log(tr("部署失败：{exc}").format(exc=exc))
            self.statusMessage.emit(tr("部署失败：{exc}").format(exc=exc))
            return None
        except Exception as exc:  # 后处理失败等——如实报告，不断连接
            self._log(tr("部署失败：{exc}").format(exc=exc))
            self.statusMessage.emit(tr("部署失败：{exc}").format(exc=exc))
            return None
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()
        note = tr("（{note}）").format(note=deployment.notes) if deployment.notes else ""
        self._log(tr("已部署 {name}：{handle}{note}").format(
            name=deployment.program_name, handle=deployment.handle, note=note
        ))
        self.statusMessage.emit(tr("已部署 {name}：{handle}").format(
            name=deployment.program_name, handle=deployment.handle
        ))
        # 留下这一份，供「取回核对」逐字节比对（扩展名要按实际后处理器解析，
        # 否则取回时会去要一个不存在的 .src）
        self._last_deployment = deployment
        self._last_ext = self._extension_for(deployment.post_processor)
        self._on_refresh_clicked()
        return deployment

    @staticmethod
    def _extension_for(post_processor: Optional[str]) -> str:
        """后处理器对应的文件扩展名（与 :meth:`Program.deploy` 写清单时同一来源）。

        拿不到就退回 ``"src"``——与 :meth:`Program.deploy` 的兜底值一致，
        不会凭空造出第三种口径。
        """
        if not post_processor:
            return "src"
        try:
            from .post import get_post_processor

            return str(get_post_processor(post_processor).file_extension).lstrip(".") or "src"
        except Exception:  # pragma: no cover - 插件后处理器被卸载等
            return "src"

    def _on_verify_clicked(self) -> None:
        self.verify_deployed()

    def verify_deployed(self, name: Optional[str] = None):
        """把控制器上的程序原文取回，与本次生成的代码逐字节比对。

        "部署报成功"和"控制器上确实躺着这份代码"是两件事——SSH 形态下程序落在
        远端文件系统里，中间任何一跳出问题（权限、编码、被别的进程覆盖）都只有
        取回来才知道。列表里选中一条就核对那条，否则核对最近一次部署的程序。

        Args:
            name: 要核对的程序名；``None`` 时取列表选中项，再退回最近部署的程序

        Returns:
            结果字典（``name`` / ``lines`` / ``bytes`` / ``matches`` / ``text``）；
            未连接 / 没有可比对的部署 / 控制器上取不到时为 ``None``
        """
        if not self.is_connected:
            self._log(tr("未连接控制器——请先建立连接"))
            return None
        item = self.list_programs.currentItem()
        target = name or (item.text() if item is not None else "")
        if not target and self._last_deployment is not None:
            target = self._last_deployment.program_name
        if not target:
            self._log(tr("没有可核对的程序——请先部署一次"))
            return None
        try:
            text = self._link.fetch_program(target, ext=self._last_ext)
        except LinkError as exc:
            self._log(tr("取回失败：{exc}").format(exc=exc))
            return None
        except TypeError as exc:  # 第三方连接实现签名不符——如实报告
            self._log(tr("取回失败：{exc}").format(exc=exc))
            return None
        if text is None:
            self._log(tr("控制器上没有 {name}——部署可能没落盘").format(name=target))
            self.statusMessage.emit(tr("控制器上没有 {name}").format(name=target))
            return None

        original = (
            self._last_deployment.code
            if self._last_deployment is not None
            and self._last_deployment.program_name == target
            else None
        )
        result: Dict[str, Any] = {
            "name": target,
            "text": text,
            "lines": len(text.splitlines()),
            "bytes": len(text.encode("utf-8")),
            "matches": None if original is None else text == original,
        }
        if result["matches"] is True:
            self._log(tr("核对 {name}：与本次生成的代码逐字节一致（{lines} 行）").format(
                name=target, lines=result["lines"]
            ))
        elif result["matches"] is False:
            self._log(tr("核对 {name}：**与本次生成的代码不一致**"
                         "（控制器 {lines} 行 / 本地 {local} 行）——先别下线").format(
                name=target,
                lines=result["lines"],
                local=len(original.splitlines()),
            ))
        else:
            self._log(tr("核对 {name}：已取回 {lines} 行（本次会话不是由本面板部署的它，"
                         "只报了行数，不冒充逐字节结论）").format(
                name=target, lines=result["lines"]
            ))
        self._last_verify = result
        self.statusMessage.emit(tr("核对 {name}：{lines} 行").format(
            name=target, lines=result["lines"]
        ))
        return result

    def _on_refresh_clicked(self) -> None:
        self.refresh_programs()

    def refresh_programs(self) -> List[str]:
        """列出控制器上已部署的程序并刷新列表。

        Returns:
            程序名列表；未连接或查询失败时为空表。
        """
        if not self.is_connected:
            return []
        try:
            names = self._link.list_programs()
        except LinkError as exc:
            self._log(tr("获取程序列表失败：{exc}").format(exc=exc))
            return []
        self.list_programs.clear()
        self.list_programs.addItems(names)
        self._log(
            tr("控制器程序（{n}）").format(n=len(names))
            if names
            else tr("控制器上还没有程序")
        )
        return names

    def _on_read_clicked(self) -> None:
        self.read_joints()

    def read_joints(self) -> Optional[np.ndarray]:
        """回读控制器实际关节角（度）。

        控制器不支持回读时**如实显示**——用本机运动学伪造一份"实际位置"
        会让孪生对比变成自欺欺人。成功时发 :attr:`jointsRead`，宿主把视口
        摆成真机当前位形。

        Returns:
            关节角（度）；未连接 / 不支持 / 失败时 ``None``
        """
        if not self.is_connected:
            self._log(tr("未连接控制器——请先建立连接"))
            return None
        if self.transport == TRANSPORT_SSH:
            # SSH 形态没有回读通道（无能力协商，也没有常驻对端服务）——不必
            # 白跑一趟 SFTP 再报错，直接如实说明。
            self._log(tr("SSH 形态没有关节回读通道——回读请用 Socket；不伪造数据"))
            self.label_joints.setText(tr("SSH 形态无回读通道"))
            return None
        try:
            q = self._link.read_joints()
        except LinkError as exc:
            self._log(tr("回读失败：{exc}").format(exc=exc))
            return None
        if q is None:
            self._log(tr("控制器不支持关节回读（能力协商 feedback=false）——不伪造数据"))
            self.label_joints.setText(tr("控制器不支持回读"))
            return None
        vals = ", ".join(f"{v:.2f}" for v in q)
        self.label_joints.setText(tr("关节角（度）：{vals}").format(vals=vals))
        self._log(tr("回读关节（度）：{vals}").format(vals=vals))
        setattr(self._link, "_last_read_joints", q)
        self.jointsRead.emit(q)
        return q
