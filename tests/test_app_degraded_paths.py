"""main.py 降级/兜底路径的回归测试（2026-09-25 由 flake8 F821 扫出）。

这四个 bug 的共同点：**都藏在"平时不走"或"错了也被吞"的分支里**，所以上线很久
没人发现——它们不是格式问题，是 flake8 的 F821（未定义名）替我们把它们挖出来的：

1. `crash_log_dir()` 用了 `os.environ`，但 main.py **从没 import os** → NameError
   被 `write_crash_log` 的 `except Exception: return None` 吞掉，**崩溃日志从来
   没写出来过**；
2. 插件错误回灌写了 `default_registry`（导入时是 `as plugin_registry`，这个名字
   没绑定），NameError 又被 `except Exception: pass` 吞掉 → 插件状态里查不到错误；
3. `GLWidget._draw_link_recursive` 降级路径里写裸 `renderer`（应为 `self.renderer`）；
4. 同一处递归调用多传了签名里没有的 `renderer=` 关键字（TypeError）。

后两条只在渲染模型不可用时才执行（实际上永不触发），用源码断言守住，避免真实
GL 上下文的重型依赖。
"""

from __future__ import annotations

import inspect
import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_ROOT))

import main  # noqa: E402
from app.views.robot_renderer import RobotRenderer  # noqa: E402


def test_crash_log_dir_does_not_raise(monkeypatch, tmp_path):
    """回归：``os`` 没导入时这里是 NameError，崩溃日志静默失效。"""
    monkeypatch.setenv("CORE_CONFIG_DIR", str(tmp_path))
    assert main.crash_log_dir() == tmp_path / "logs"


def test_write_crash_log_really_writes_a_file(monkeypatch, tmp_path):
    """崩溃钩子必须真能落盘——它被 except 兜住，坏了是没有任何提示的。"""
    monkeypatch.setenv("CORE_CONFIG_DIR", str(tmp_path))
    try:
        raise ValueError("合成崩溃，用于验证日志链路")
    except ValueError:
        path = main.write_crash_log(*sys.exc_info())

    assert path is not None, "崩溃日志没写出来（函数名被 except 吞掉了异常）"
    assert path.exists()
    text = path.read_text(encoding="utf-8")
    assert "ValueError" in text
    assert "合成崩溃" in text


def test_plugin_error_feedback_uses_the_bound_registry_name():
    """回归：``default_registry`` 未绑定，错误回灌被 except 静默吞掉。"""
    source = (_ROOT / "main.py").read_text(encoding="utf-8")
    assert "default_registry.add_error" not in source, (
        "default_registry 未绑定（导入时已 as plugin_registry），"
        "写回去会在 try/except 里静默失效"
    )
    if main._HAS_PLUGINS:
        assert callable(main.plugin_registry.add_error)


class _NullGL:
    """把 OpenGL 调用变成空操作：这条路径只验证"取哪个渲染器"，不验证绘制。"""

    def __getattr__(self, _name):
        return lambda *args, **kwargs: None


def _fake_renderer(name: str, link_name: str):
    """一台只有单个 box 连杆的"合成"渲染器，够走降级递归绘制。"""
    renderer = RobotRenderer()
    renderer.robot_name = name
    renderer.root_link = link_name
    renderer.links = {
        link_name: {
            "name": link_name,
            "type": "box",
            "params": (0.1, 0.1, 0.1),
            "origin_xyz": (0.0, 0.0, 0.0),
            "origin_rpy": (0.0, 0.0, 0.0),
            "color": (0.5, 0.5, 0.5),
        }
    }
    renderer.joints = {}
    renderer.joint_values = {}
    return renderer


def test_legacy_draw_accepts_the_robot_renderer():
    """回归：``renderer`` 形参丢失时，调用点传 ``renderer=`` 直接 TypeError。"""
    params = inspect.signature(main.GLWidget._draw_link_recursive).parameters
    assert "renderer" in params, "签名丢了 renderer 形参，三处调用点会 TypeError"


def test_legacy_draw_uses_the_passed_renderer_not_self(monkeypatch):
    """多机降级绘制必须画**传入那台**的连杆。

    回归：形参丢失后函数体退回 ``self.renderer``（主/最后加载那台），多机时
    会把别人的连杆画到这台身上——与当初属性面板"选中 A 却显示 B 的规格"
    是同一类串扰。用哑 self 直接调未绑定方法，绕开真实 GL 上下文。
    """
    # 补丁要打在**定义处**：GLWidget 自 ADR-002 Step 2 起住在 app/views/gl_widget.py，
    # 它查的是**自己模块**的 `gl`。打 `main.gl` 是个空操作 → 真 GL 调用 → 段错误（实测）。
    from app.views import gl_widget

    monkeypatch.setattr(gl_widget, "gl", _NullGL())
    draw_fn = gl_widget.GLWidget._draw_link_recursive

    class _Self:
        def __init__(self, renderer):
            self.renderer = renderer
            self.drawn = []

        def _draw_box(self, *args):
            self.drawn.append(args)

    # self.renderer 里只有 A 的连杆，传入的是 B——若退回 self.renderer，
    # 会因为找不到 link_B 而直接 return，一次绘制都不会发生
    stub = _Self(_fake_renderer("A", "link_A"))
    draw_fn(stub, "link_B", None, renderer=_fake_renderer("B", "link_B"))
    assert stub.drawn, "降级绘制退回了 self.renderer——多机会画错机器人"

    # 不传时退回 self.renderer：单机兼容，A 的连杆要画出来
    stub2 = _Self(_fake_renderer("A", "link_A"))
    draw_fn(stub2, "link_A")
    assert stub2.drawn, "未传 renderer 时应退回 self.renderer"
