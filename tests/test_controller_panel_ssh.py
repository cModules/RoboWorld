"""真机连接面板（v1.1.4）：SSH 传输与「取回核对」的无头测试。

v1.1.1 把 :class:`SocketLink` 界面化后，v1.1.2 的 :class:`SSHLink` 仍只能在脚本里
用。本文件验收 v1.1.4 补上的那一段——同一个面板切到 SSH 传输，对端是**真 SSH**：

* 传输切换——SSH 专属字段出现、端口/超时默认值跟着换（22 / 10s），
  用户手工填过的现场参数不被覆盖；未知传输名当场报错
* 连接——真 SFTP + exec 走通，状态区**如实**写"无能力协商 · 不支持关节回读"
  （Socket 那套服务端自述/能力表在 SSH 形态下不存在，不假装有）
* 失败——端口没有服务端、没给凭据两类都返回 False 并说明原因，界面回未连接
* 部署——程序经 SFTP **真实落到远端目录**，清单可被远端读取
* 核对（新能力）——取回原文与本次生成的代码逐字节比对；远端被改动后
  **必须报不一致**（这是这个按钮存在的唯一理由）；没部署过/程序不存在如实说明
* 回读——SSH 形态没有回读通道，如实说明且不发伪造信号
* i18n——en 模式下按钮/日志（含 link.describe()）为英文

全部跑在 Qt offscreen 平台上，不需要真实显示服务器与真机。
"""

import os
import sys
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

pytest.importorskip("paramiko", reason="SSH 打桩服务端与 SSHLink 都需要 paramiko")

from PySide6 import QtWidgets  # noqa: E402

import support.i18n as i18n  # noqa: E402
from domain.robot import Robot  # noqa: E402
from app.views.controller_panel import (
    TRANSPORT_SOCKET,
    TRANSPORT_SSH,
    ControllerPanel,
)
from domain.link import DEFAULT_SSH_PORT, DEFAULT_SSH_TIMEOUT, SSHLink  # noqa: E402
from service.program import ProgramBuilder  # noqa: E402
from adapter.sshstub import SSHStubServer  # noqa: E402

URDF = REPO_ROOT / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"
HOME = [0.0, -90.0, 90.0, -90.0, 0.0, 0.0]
SECOND = [15.0, -80.0, 80.0, -85.0, 10.0, 0.0]
REMOTE_DIR = "/programs"
PASSWORD = "pw"


@pytest.fixture(scope="module")
def qapp():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture(scope="module")
def robot():
    return Robot(str(URDF), "UR5e")


@pytest.fixture()
def stub(tmp_path):
    """本机回环 SSH 打桩控制器（真实落盘到 tmp_path，构造后即在监听）。"""
    server = SSHStubServer(tmp_path / "stubroot")
    try:
        yield server
    finally:
        server.stop()


def _provider(robot, post=None):
    def provider():
        builder = ProgramBuilder(robot)
        builder.moveJ(HOME, speed=50.0)
        builder.moveJ(SECOND, speed=50.0)
        return {"program": builder.build(), "post_processor": post}

    return provider


@pytest.fixture()
def panel(qapp, stub, robot):
    """已切到 SSH 传输并填好凭据的面板（未连接）。"""
    p = ControllerPanel()
    p.set_robot(robot)
    p.set_program_provider(_provider(robot))
    p.set_transport(TRANSPORT_SSH)
    p.edit_host.setText("127.0.0.1")
    p.edit_user.setText("tester")
    p.edit_password.setText(PASSWORD)
    p.edit_remote_dir.setText(REMOTE_DIR)
    p.spin_port.setValue(stub.port)
    p.spin_timeout.setValue(10.0)
    yield p
    if p.is_connected:
        p.disconnect_from_controller()
    p.deleteLater()


def _log_text(panel):
    return panel.log_view.toPlainText()


# --------------------------------------------------------------------------- #
# 传输切换
# --------------------------------------------------------------------------- #


def test_default_transport_is_socket(panel):
    """默认仍是 Socket——v1.1.1 的行为对既有用户零变化。"""
    fresh = ControllerPanel()
    assert fresh.transport == TRANSPORT_SOCKET
    assert fresh.ssh_box.isHidden()
    assert fresh.spin_port.value() == 30002
    fresh.deleteLater()


def test_switch_to_ssh_reveals_fields_and_switches_defaults(panel):
    fresh = ControllerPanel()
    assert fresh.ssh_box.isHidden()
    fresh.set_transport(TRANSPORT_SSH)
    assert fresh.transport == TRANSPORT_SSH
    assert not fresh.ssh_box.isHidden()
    assert fresh.spin_port.value() == DEFAULT_SSH_PORT
    assert fresh.spin_timeout.value() == pytest.approx(DEFAULT_SSH_TIMEOUT)
    # 切回去：SSH 默认值被识别为"还是默认值"，跟着换回 Socket 默认
    fresh.set_transport(TRANSPORT_SOCKET)
    assert fresh.ssh_box.isHidden()
    assert fresh.spin_port.value() == 30002
    fresh.deleteLater()


def test_user_edited_port_survives_transport_switch(panel):
    """现场参数是用户填的——切换传输不该把它改掉。"""
    fresh = ControllerPanel()
    fresh.spin_port.setValue(4022)
    fresh.spin_timeout.setValue(3.5)
    fresh.set_transport(TRANSPORT_SSH)
    assert fresh.spin_port.value() == 4022
    assert fresh.spin_timeout.value() == pytest.approx(3.5)
    fresh.deleteLater()


def test_unknown_transport_rejected(panel):
    with pytest.raises(ValueError):
        panel.set_transport("telnet")


# --------------------------------------------------------------------------- #
# 连接
# --------------------------------------------------------------------------- #


def test_connect_over_ssh_builds_ssh_link(panel, stub):
    assert panel.connect_to_controller() is True
    assert panel.is_connected
    link = panel.link
    assert isinstance(link, SSHLink)
    assert link.remote_dir == REMOTE_DIR
    assert link.username == "tester"


def test_connect_over_ssh_is_honest_about_capabilities(panel, stub):
    """SSH 没有能力协商——状态区不许出现服务端自述/能力表那套词。"""
    panel.connect_to_controller()
    info = panel.label_info.text()
    assert "无能力协商" in info
    assert REMOTE_DIR in info
    assert "不支持关节回读" in info
    assert "服务端" not in info
    assert "能力 " not in info


def test_connect_ssh_failure_returns_false(panel, stub):
    panel.spin_port.setValue(1)  # 不会有 SSH 服务端在监听
    panel.spin_timeout.setValue(0.5)
    assert panel.connect_to_controller() is False
    assert not panel.is_connected
    assert "连接失败" in _log_text(panel)
    assert not panel.btn_deploy.isEnabled()


def test_connect_ssh_without_credentials_fails_honestly(panel, stub):
    """密码与私钥都空 → paramiko 无可用的认证方式；如实失败，不静默。"""
    panel.edit_password.setText("")
    panel.edit_keyfile.setText("")
    assert panel.connect_to_controller() is False
    assert not panel.is_connected
    assert "连接失败" in _log_text(panel)


# --------------------------------------------------------------------------- #
# 部署（SFTP 真落盘）
# --------------------------------------------------------------------------- #


def test_deploy_over_ssh_lands_in_remote_root(panel, stub):
    panel.connect_to_controller()
    deployment = panel.deploy_current_program()
    assert deployment is not None and deployment.ok
    assert deployment.handle.startswith("ssh://")
    # 打桩根目录里真的躺着这份代码（UR5e → ur 后处理 → .script）
    landed = stub.root / REMOTE_DIR.lstrip("/") / "program.script"
    assert landed.exists()
    assert landed.read_text(encoding="utf-8") == deployment.code
    assert panel.list_programs.count() == 1
    assert panel.last_deployment is deployment


def test_deploy_note_is_honest_for_each_transport(panel, robot, tmp_path):
    """部署备注按连接形态如实说话——往真机部署时说"离线导出"是误导。"""
    from domain.link import FileLink

    panel.connect_to_controller()
    notes_ssh = panel.deploy_current_program().notes
    assert "ssh 连接" in notes_ssh
    assert "离线导出" not in notes_ssh

    robot_file = Robot(str(URDF), "UR5e")
    file_link = FileLink(tmp_path / "deploy")
    file_link.connect()
    builder = ProgramBuilder(robot_file)
    builder.moveJ(HOME, speed=50.0)
    assert "离线导出" in builder.build().deploy(file_link).notes


# --------------------------------------------------------------------------- #
# 取回核对（v1.1.4 新能力）
# --------------------------------------------------------------------------- #


def test_verify_reports_byte_identical(panel, stub):
    panel.connect_to_controller()
    panel.deploy_current_program()
    result = panel.verify_deployed()
    assert result is not None
    assert result["name"] == "program"
    assert result["matches"] is True
    assert result["lines"] > 0
    assert "逐字节一致" in _log_text(panel)
    assert panel.last_verify is result


def test_verify_detects_tampered_remote_file(panel, stub):
    """远端被改动（或被别的进程覆盖）必须报不一致——这是这个按钮存在的理由。"""
    panel.connect_to_controller()
    panel.deploy_current_program()
    landed = stub.root / REMOTE_DIR.lstrip("/") / "program.script"
    landed.write_text("; 被人改过的程序\n", encoding="utf-8")
    result = panel.verify_deployed()
    assert result is not None
    assert result["matches"] is False
    assert "不一致" in _log_text(panel)


def test_verify_without_deploy_warns(panel, stub):
    panel.connect_to_controller()
    assert panel.verify_deployed() is None
    assert "没有可核对的程序" in _log_text(panel)


def test_verify_missing_program_warns(panel, stub):
    panel.connect_to_controller()
    panel.deploy_current_program()
    assert panel.verify_deployed("nope") is None
    assert "控制器上没有" in _log_text(panel)
    # 取不到不影响会话
    assert panel.is_connected


def test_verify_without_connection_warns(panel):
    assert panel.verify_deployed() is None
    assert "未连接" in _log_text(panel)


# --------------------------------------------------------------------------- #
# 回读：SSH 形态没有通道
# --------------------------------------------------------------------------- #


def test_ssh_read_joints_is_honest(panel, stub, qapp):
    got = []
    panel.jointsRead.connect(lambda q: got.append(q))
    panel.connect_to_controller()
    assert panel.read_joints() is None
    assert not got  # 绝不发伪造的关节角
    assert "没有关节回读通道" in _log_text(panel)
    assert panel.label_joints.text() == "SSH 形态无回读通道"
    assert panel.last_joints is None
    assert panel.is_connected


def test_ssh_link_carries_no_kinematics(panel, stub):
    """SSH 形态不回读位姿，运动学不该被挂上去（挂了也没人用，只会误导）。"""
    panel.set_robot(Robot(str(URDF), "UR5e"))
    panel.connect_to_controller()
    assert isinstance(panel.link, SSHLink)
    assert not hasattr(panel.link, "robot")


# --------------------------------------------------------------------------- #
# i18n
# --------------------------------------------------------------------------- #


def test_panel_text_en_ssh(monkeypatch, tmp_path, stub, robot):
    """en 模式下传输项、按钮、日志（含 link.describe()）都是英文。"""
    monkeypatch.delenv("CORE_LANG", raising=False)
    monkeypatch.setenv("CORE_CONFIG_DIR", str(tmp_path / "cfg"))
    i18n.set_language(i18n.LANG_EN, save=False)
    try:
        p = ControllerPanel()
        p.set_robot(robot)
        p.set_transport(TRANSPORT_SSH)
        assert p.combo_transport.currentText() == "SSH (SFTP upload + exec)"
        assert p.btn_verify.text() == "Fetch & Verify"
        assert p.btn_read.text() == "Read Joints"
        assert p.label_joints.text() == "SSH transport has no feedback channel"
        p.set_program_provider(_provider(robot))
        p.edit_host.setText("127.0.0.1")
        p.edit_user.setText("tester")
        p.edit_password.setText(PASSWORD)
        p.edit_remote_dir.setText(REMOTE_DIR)
        p.spin_port.setValue(stub.port)
        p.spin_timeout.setValue(10.0)
        assert p.connect_to_controller() is True
        log = _log_text(p)
        assert "Connected:" in log
        assert "dir=" in log and "uploaded=0" in log  # describe() 也随语言
        assert p.label_info.text().startswith("SSH: no capability negotiation")
        p.disconnect_from_controller()
        p.deleteLater()
    finally:
        i18n.set_language(i18n.LANG_ZH, save=False)
