"""ADR-002 拆解第 2 步：``DevicePropertyPanel`` 迁入 ``app/views`` 的回归测试。

守四条：

1. **不断链**：``main.DevicePropertyPanel`` 仍是同一个类（MainWindow 与既有测试
   都按这个名字取），且 main.py 里不留副本；
2. **不制造新的超限文件**：拆出来的三个模块都在 GUI 组件 300 行阈值内
   （AGENTS.md #26）——这正是当初不能直接整块搬进来的原因；
3. **拆出去的方法仍然好用**：UI 骨架挂全了控件，动态行/滑块由 mixin 提供；
4. **多机串扰的守门**：关节回调必须带上机器人名（拖 A 的滑块不能写进 B）。
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_ROOT))

from PySide6 import QtWidgets  # noqa: E402

from app.views import property_panel as views_mod  # noqa: E402
from app.views.property_controls import ControlRowsMixin  # noqa: E402
from app.views.robot_renderer import RobotRenderer  # noqa: E402

_QT_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _fake_renderer() -> RobotRenderer:
    """一台"合成"两轴机器人：够建滑块，不必加载 URDF。"""
    renderer = RobotRenderer()
    renderer.robot_name = "合成双轴"
    renderer.joints = {
        f"j{i}": {
            "type": "revolute",
            "parent": "base",
            "child": f"link{i}",
            "origin_xyz": (0.0, 0.0, 0.0),
            "origin_rpy": (0.0, 0.0, 0.0),
            "axis": (0.0, 0.0, 1.0),
            "limit": {"lower": -3.14, "upper": 3.14, "velocity": 1.0, "effort": 10.0},
        }
        for i in (1, 2)
    }
    renderer.joint_values = {"j1": 0.0, "j2": 0.5}
    return renderer


def test_app_module_still_exports_the_moved_class():
    """main.py 迁出后必须仍导出同名符号。"""
    import main  # noqa: E402

    assert main.DevicePropertyPanel is views_mod.DevicePropertyPanel


def test_app_py_no_longer_defines_the_class():
    """搬走了就别留副本——两份实现迟早各自漂移。"""
    source = (_ROOT / "main.py").read_text(encoding="utf-8")
    assert "class DevicePropertyPanel" not in source


def test_new_view_files_within_gui_size_budget():
    """迁进来的文件不许超 GUI 组件 300 行阈值（AGENTS.md #26）。

    665 行的面板之所以要拆成三块，就是为了不制造新的超限文件——这条守的是
    "拆完之后别又被堆回去"。
    """
    base = Path(views_mod.__file__).parent
    for name in (
        "property_panel.py",
        "property_panel_ui.py",
        "property_controls.py",
        "robot_renderer.py",
    ):
        n = sum(1 for _ in (base / name).open(encoding="utf-8"))
        assert n <= 300, f"{name} {n} 行 > GUI 组件阈值 300"


def test_mixin_supplies_the_dynamic_rows():
    """参数/关节/轴三类动态行来自 mixin，不是被丢在别处。"""
    assert ControlRowsMixin in views_mod.DevicePropertyPanel.__mro__
    for name in (
        "_rebuild_param_rows",
        "_setup_joint_controls",
        "_rebuild_axis_rows",
        "update_joint_displays",
    ):
        owner = getattr(views_mod.DevicePropertyPanel, name).__qualname__
        assert owner.startswith("ControlRowsMixin"), f"{name} 不在 mixin 里"


def test_ui_skeleton_is_built():
    """``build_ui`` 必须把骨架挂全：位置 6 个输入框、日志、滚动区都在。"""
    panel = views_mod.DevicePropertyPanel()
    assert sorted(panel.obj_pose_widgets) == ["rx", "ry", "rz", "x", "y", "z"]
    assert panel.log_text is not None
    assert panel.object_scroll is not None
    # 未选中对象时「对象属性」默认隐藏
    panel.set_object(None)
    assert panel.object_scroll.isHidden()


def test_joint_slider_emits_robot_name():
    """关节回调必须带机器人名——否则 MainWindow 无从判断该写哪台（多机串扰）。"""
    panel = views_mod.DevicePropertyPanel()
    panel._setup_joint_controls(_fake_renderer(), "机器人A")
    assert list(panel.sliders) == ["j1", "j2"]

    received = []
    panel.jointChanged.connect(lambda *args: received.append(args))
    panel._on_joint_slider_changed("j1", 3000)  # 滑块单位是 1/100 度

    assert received, "拖滑块没发出信号"
    robot, joint, value = received[-1]
    assert robot == "机器人A", f"关节信号丢了机器人名：{received[-1]}"
    assert joint == "j1"
    assert abs(value - 0.5236) < 1e-3  # 30° → 弧度


def test_joint_display_reflects_external_values():
    """回放等外部来源改关节值时，滑块与标签要跟着动（blockSignals 防回环）。"""
    panel = views_mod.DevicePropertyPanel()
    panel._setup_joint_controls(_fake_renderer(), "机器人A")
    panel.update_joint_displays({"j2": 1.0})
    # 滑块单位是 1/100 度且经过 int() 截断：degrees(1.0) = 57.2957… → 5729
    assert abs(panel.sliders["j2"].value() - 5729) <= 1
    assert panel.value_labels["j2"].text() == "57.3°"
