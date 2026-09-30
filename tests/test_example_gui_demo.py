"""``examples/gui_demo`` 的最小冒烟测试。

来历：仓库根原先躺着一个 ``test_simple.py``——把 stdout/stderr 重定向到
``test_output.log``、用相对路径 ``sys.path.insert``、跑完就删不掉那两个日志。
它既不进 CI，也不会被 pytest 正常收集（名字像测试，行为是手工脚本）。

**P0-3 目录重整**把它挪进 ``tests/``，顺手改成真测试：它当初想确认的事其实只有
一件——示例自带的最小 viewer 能不能解析一个真实 URDF。现在这件事有断言守着。

注：``examples/gui_demo/python_viewer.py`` 里那份 ``RobotRenderer`` 与
``app/views/robot_renderer.py`` 是**两套实现**（前者是独立演示，不依赖
core 包）。收敛它们要连带改 ``tools/smoke_gui_demo.py`` 与截图脚本，属独立
任务，见 docs/develop/BUILD_QUEUE.md。
"""

from __future__ import annotations

import importlib
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
DEMO_DIR = REPO_ROOT / "examples" / "gui_demo"
SO101 = REPO_ROOT / "assets" / "models" / "lerobot" / "so101" / "urdf" / "so101.urdf"


@pytest.fixture(scope="module")
def viewer():
    """导入示例 viewer；缺 PySide6/PyOpenGL 时跳过（它自己会 sys.exit）。"""
    if not (DEMO_DIR / "python_viewer.py").is_file():
        pytest.skip("examples/gui_demo/python_viewer.py 不存在")
    if str(DEMO_DIR) not in sys.path:
        sys.path.insert(0, str(DEMO_DIR))
    try:
        return importlib.import_module("python_viewer")
    except SystemExit:  # 模块级缺 GUI 依赖时它直接 sys.exit(1)
        pytest.skip("示例 viewer 需要 PySide6 / PyOpenGL")


def test_demo_viewer_parses_a_real_urdf(viewer):
    """示例 viewer 能解析真实 URDF，并给出连杆/关节/根连杆。"""
    if not SO101.is_file():
        pytest.skip(f"缺少测试用 URDF: {SO101}")

    renderer = viewer.RobotRenderer()
    assert renderer.load_urdf(str(SO101)), "load_urdf 返回 False（一个连杆都没解析出来）"
    assert renderer.links, "没有解析到连杆"
    assert renderer.joints, "没有解析到关节"
    assert renderer.root_link in renderer.links, f"根连杆 {renderer.root_link!r} 不在连杆表里"


def test_demo_viewer_clears_state_between_loads(viewer):
    """重复载入必须清干净——否则第二次载入会把上一次的零件留在场景里。"""
    if not SO101.is_file():
        pytest.skip(f"缺少测试用 URDF: {SO101}")

    renderer = viewer.RobotRenderer()
    renderer.load_urdf(str(SO101))
    first = len(renderer.links)
    renderer.load_urdf(str(SO101))
    assert len(renderer.links) == first, "重复载入后连杆数变了（状态没清）"
