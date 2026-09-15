"""真机连接面板（v1.1.1）的无头测试。

把 v1.0.5 的 :class:`SocketLink` 界面化后，用本机回环打桩控制器
（:class:`ControllerStubServer`）做对端，真 socket 验收 GUI 全链路：

* 初始状态——未连接时部署/回读/刷新按钮不可用，连接可用
* 连接成功——握手后状态区展示服务端自述 / 关节数 / 能力表，程序列表刷新
* 连接失败——连不上时界面回到未连接状态，日志给出原因
* 程序部署——provider 提供的程序经 ``Program.deploy`` 真下发到控制器
  （打桩端 ``state.programs`` 里能取回原文），后处理器选择被如实携带
* 失败语义——未连接 / 无程序 / 服务端业务拒绝（``ok:false``）三类都只记
  日志不断会话；部署后连接仍然可用
* 关节回读——回读值（度）发出 ``jointsRead`` 信号；控制器声明不支持回读
  （``caps.feedback=false``）时如实显示，绝不伪造
* 断开——按钮状态复位，可再次连接
* MainWindow 接线——``on_controller_joints_read`` 把真机位形摆进视口

全部跑在 Qt offscreen 平台上，不需要真实显示服务器与真机。
"""

import math
import os
import sys
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

from PySide6 import QtWidgets  # noqa: E402

import orolp.i18n as i18n  # noqa: E402
from orolp import Robot  # noqa: E402
from orolp.controller_panel import ControllerPanel  # noqa: E402
from orolp.linkstub import ControllerStubServer  # noqa: E402
from orolp.program import ProgramBuilder  # noqa: E402

URDF = REPO_ROOT / "models" / "ur" / "ur5e" / "ur5e.urdf"
JOINTS = [f"joint_{i}" for i in range(1, 7)]
HOME = [0.0, -90.0, 90.0, -90.0, 0.0, 0.0]
SECOND = [15.0, -80.0, 80.0, -85.0, 10.0, 0.0]


@pytest.fixture(scope="module")
def qapp():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.fixture(scope="module")
def robot():
    return Robot(str(URDF), "UR5e")


@pytest.fixture()
def stub(qapp):
    """默认能力的打桩控制器：程序 / 回读 / 位姿全开。"""
    server = ControllerStubServer("127.0.0.1", 0, dof=6)
    port = server.start()
    yield server
    server.stop()
    del port


@pytest.fixture()
def panel(qapp, stub, robot):
    """已注入机器人与程序 provider 的面板（默认未连接，端口对准打桩端）。"""
    p = ControllerPanel()
    p.set_robot(robot)
    p.set_program_provider(_provider(robot))
    p.spin_port.setValue(stub.port)
    p.spin_timeout.setValue(2.0)
    yield p
    if p.is_connected:
        p.disconnect_from_controller()
    p.deleteLater()


def _provider(robot, post=None):
    """两步示教程序（moveJ×2），模拟离线编程面板的当前程序。"""
    def provider():
        builder = ProgramBuilder(robot)
        builder.moveJ(HOME, speed=50.0)
        builder.moveJ(SECOND, speed=50.0)
        return {"program": builder.build(), "post_processor": post}

    return provider


def _log_text(panel):
    return panel.log_view.toPlainText()


# --------------------------------------------------------------------------- #
# 初始状态
# --------------------------------------------------------------------------- #

def test_initial_state_not_connected(panel):
    assert not panel.is_connected
    assert panel.link is None
    assert panel.btn_connect.isEnabled()
    assert not panel.btn_disconnect.isEnabled()
    for btn in (panel.btn_deploy, panel.btn_refresh, panel.btn_read):
        assert not btn.isEnabled()


def test_connect_failure_returns_false(panel, qapp):
    """连一个没有服务端的端口：返回 False、日志说明原因、状态回未连接。"""
    panel.edit_host.setText("127.0.0.1")
    panel.spin_port.setValue(1)  # 不会有服务端在监听
    panel.spin_timeout.setValue(0.5)
    assert panel.connect_to_controller() is False
    assert not panel.is_connected
    assert "连接失败" in _log_text(panel)
    assert panel.btn_deploy.isEnabled() is False


# --------------------------------------------------------------------------- #
# 连接
# --------------------------------------------------------------------------- #

def test_connect_success_shows_caps(panel, stub):
    assert panel.connect_to_controller() is True
    assert panel.is_connected
    info = panel.label_info.text()
    assert "orolp.ControllerStubServer" in info
    assert "dof 6" in info.replace("关节数 6", "dof 6") or "6" in info
    # 能力表如实展示（默认全开）
    assert "feedback" in info
    # 连接后部署/回读/刷新可用，连接按钮禁用
    assert not panel.btn_connect.isEnabled()
    for btn in (panel.btn_deploy, panel.btn_refresh, panel.btn_read):
        assert btn.isEnabled()


def test_disconnect_resets_state(panel, stub):
    panel.connect_to_controller()
    panel.disconnect_from_controller()
    assert not panel.is_connected
    assert panel.link is None
    assert panel.btn_connect.isEnabled()
    assert not panel.btn_deploy.isEnabled()
    assert "已断开" in _log_text(panel)
    # 断开后可以再连
    assert panel.connect_to_controller() is True


def test_duplicate_connect_is_reconnect(panel, stub):
    """重复点连接 = 重连，不叠第二条 socket。"""
    assert panel.connect_to_controller() is True
    assert panel.connect_to_controller() is True
    assert panel.is_connected
    assert stub.state.clients == 2  # 第二次是断开重连，不是并行连接


# --------------------------------------------------------------------------- #
# 程序部署
# --------------------------------------------------------------------------- #

def test_deploy_without_connection_warns(panel):
    panel.set_program_provider(_provider(Robot(str(URDF), "UR5e")))
    assert panel.deploy_current_program() is None
    assert "未连接" in _log_text(panel)


def test_deploy_without_program_warns(panel, stub):
    panel.set_program_provider(lambda: None)
    assert panel.connect_to_controller() is True
    assert panel.deploy_current_program() is None
    assert "没有可部署的程序" in _log_text(panel)
    # 失败不影响连接
    assert panel.is_connected


def test_deploy_program_reaches_controller(panel, stub, robot):
    panel.set_program_provider(_provider(robot))  # UR5e → ur 后处理
    panel.connect_to_controller()
    deployment = panel.deploy_current_program()
    assert deployment is not None
    assert deployment.ok
    assert deployment.handle.startswith("sock://")
    # 打桩端真收到了代码原文
    assert list(stub.state.programs.keys()) == ["program"]
    code = stub.state.programs["program"]
    assert "movej" in code.lower()
    # 列表里能看到它
    assert panel.list_programs.count() == 1
    assert "已部署" in _log_text(panel)


def test_deploy_honors_selected_post_processor(panel, stub, robot):
    """导出打包标签里选的后处理器要被如实携带。"""
    panel.set_program_provider(_provider(robot, post="fanuc_r30ib"))
    panel.connect_to_controller()
    deployment = panel.deploy_current_program()
    assert deployment is not None
    assert deployment.post_processor == "fanuc_r30ib"
    assert "ARCON" not in stub.state.programs["program"]  # 无工艺指令的普通程序


def test_deploy_business_refusal_keeps_session(panel):
    """服务端业务性拒绝（ok:false）只记日志，不断会话。"""
    server = ControllerStubServer("127.0.0.1", 0, dof=6, reject_ops={"upload"})
    port = server.start()
    try:
        robot = Robot(str(URDF), "UR5e")
        panel.set_program_provider(_provider(robot))
        panel.spin_port.setValue(port)
        assert panel.connect_to_controller() is True
        assert panel.deploy_current_program() is None
        assert "部署失败" in _log_text(panel)
        # 会话还活着：回读照常工作
        q = panel.read_joints()
        assert q is not None and q.size == 6
        assert panel.is_connected
    finally:
        server.stop()


def test_deploy_without_provider_warns(panel, stub):
    panel.set_program_provider(None)
    panel.connect_to_controller()
    assert panel.deploy_current_program() is None
    assert "未设置程序来源" in _log_text(panel)


# --------------------------------------------------------------------------- #
# 关节回读
# --------------------------------------------------------------------------- #

def test_read_joints_emits_signal(panel, stub, qapp):
    got = []
    panel.jointsRead.connect(lambda q: got.append(q))
    panel.connect_to_controller()
    q = panel.read_joints()
    assert q is not None
    assert q.size == stub.state.dof
    assert got and got[-1].size == 6
    assert "回读关节" in _log_text(panel)
    assert "关节角（度）" in panel.label_joints.text()
    assert panel.last_joints is not None


def test_read_joints_unsupported_is_honest(panel):
    """caps.feedback=false 时如实显示——不伪造数据。"""
    server = ControllerStubServer("127.0.0.1", 0, dof=6, caps={"feedback": False})
    port = server.start()
    try:
        got = []
        panel.jointsRead.connect(lambda q: got.append(q))
        panel.spin_port.setValue(port)
        panel.connect_to_controller()
        assert panel.read_joints() is None
        assert not got  # 没有假数据发出
        assert "不伪造数据" in _log_text(panel)
    finally:
        server.stop()


def test_read_joints_without_connection(panel):
    assert panel.read_joints() is None
    assert "未连接" in _log_text(panel)


# --------------------------------------------------------------------------- #
# MainWindow 接线
# --------------------------------------------------------------------------- #

def test_main_window_applies_read_joints_to_viewport(qapp, monkeypatch, tmp_path):
    """app.py 的 on_controller_joints_read 把真机关节（度）摆进视口。"""
    import importlib.util

    monkeypatch.setenv("OROLP_CONFIG_DIR", str(tmp_path / "cfg"))
    i18n.set_language(i18n.LANG_ZH, save=False)
    spec = importlib.util.spec_from_file_location("app_under_test", REPO_ROOT / "app.py")
    app_mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(app_mod)

    win = app_mod.MainWindow()
    robot_deg = [10.0, -70.0, 80.0, -80.0, 20.0, 5.0]
    win.on_controller_joints_read(robot_deg)

    names = list(win.gl_widget.renderer.actuated_names())
    for name, deg in zip(names, robot_deg):
        got = math.degrees(win.gl_widget.renderer.joint_values[name])
        assert abs(got - deg) < 1e-6
    # 场景侧也同步（干涉检查按真实位形算）
    if hasattr(win, "scene_panel") and win.scene_panel.scene.has_robot:
        scene_joints = win.scene_panel.scene._robot_joints_deg
        assert abs(scene_joints[0] - robot_deg[0]) < 1e-6
    win.deleteLater()
    i18n.set_language(i18n.LANG_ZH, save=False)


# --------------------------------------------------------------------------- #
# i18n
# --------------------------------------------------------------------------- #

def test_panel_text_en(monkeypatch, tmp_path, stub, robot):
    """en 模式下面板按钮与状态文案为英文。"""
    monkeypatch.delenv("OROLP_LANG", raising=False)
    monkeypatch.setenv("OROLP_CONFIG_DIR", str(tmp_path / "cfg"))
    i18n.set_language(i18n.LANG_EN, save=False)
    try:
        p = ControllerPanel()
        p.set_robot(robot)
        assert p.btn_connect.text() == "Connect"
        assert p.btn_deploy.text() == "Deploy Current Program"
        assert p.btn_read.text() == "Read Joints"
        assert p.label_state.text() == "Not connected"
        p.set_program_provider(_provider(robot))
        p.spin_port.setValue(stub.port)
        p.spin_timeout.setValue(2.0)
        assert p.connect_to_controller() is True
        assert p.label_state.text() == "Connected"
        assert "Connected:" in _log_text(p)
        p.disconnect_from_controller()
        p.deleteLater()
    finally:
        i18n.set_language(i18n.LANG_ZH, save=False)
