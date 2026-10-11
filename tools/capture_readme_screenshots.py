#!/usr/bin/env python3
"""README 基础界面截图重拍（docs/screenshots/）。

与 ``tools/capture_grasp_screenshots.py`` 的分工：那个脚本只拍「抓取工作站演示」
三张；这里拍 README 前面那几张**基础界面**图——主视图、代码预览、离线编程面板、
厂商 CAD 机器人。

驱动方式与 GUI 冒烟（``tools/smoke_gui_demo.py``）一致：起真实主窗口 → 走真实
入口（载入 URDF / 记示教点 / 切标签 / 校验程序）→ ``widget.grab()``。之所以强调
"真实"：README 是给没装过的人看的第一眼，合成图会让人对界面产生错误预期。

几个刻意的处理（否则进程会卡住或拍到脏画面）：

* ``CORE_CONFIG_DIR`` 指向临时目录：不受开发机上残留的自动保存快照影响；
* 模态对话框一律 stub：QMessageBox / QDialog 都不真正弹；
* 每张图用**新窗口**：``showEvent`` 会自动载入默认机器人，同窗再 load 会变成
  "场景里两台机器人"，画面与文案都对不上；
* 载入后清掉选中高亮并 ``frame_robot()`` 自动取景：机器人完整入画、不带黄色高亮。

注意：**不能用** ``QT_QPA_PLATFORM=offscreen``——offscreen 插件不支持
QOpenGLWidget，视口会拍成空白（见 ``tools/render_robot_gallery.py`` 的说明）。

运行::

    python tools/capture_readme_screenshots.py
    # 可选: --out /tmp/rw-shots 先拍到别处人工过目，再覆盖 docs/screenshots
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

_CONFIG_DIR = Path(tempfile.mkdtemp(prefix="rw-readme-shots-"))
os.environ["CORE_CONFIG_DIR"] = str(_CONFIG_DIR)

from PySide6 import QtWidgets  # noqa: E402

import main  # noqa: E402

UR5E_URDF = REPO_ROOT / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"
ZU5_REAL_URDF = REPO_ROOT / "assets" / "models" / "jaka" / "zu5" / "zu5_real.urdf"


def _stub_dialogs() -> None:
    """把会阻塞的交互全部驯服：行程照跑，只是不等人类输入。"""
    for name in ("information", "warning", "critical", "question"):
        setattr(
            QtWidgets.QMessageBox,
            name,
            staticmethod(lambda *a, **k: QtWidgets.QMessageBox.Ok),
        )
    QtWidgets.QDialog.exec = lambda self, *a, **k: QtWidgets.QDialog.Rejected
    QtWidgets.QFileDialog.getSaveFileName = staticmethod(lambda *a, **k: ("", ""))
    QtWidgets.QFileDialog.getOpenFileName = staticmethod(lambda *a, **k: ("", ""))


def _pump(qapp, rounds: int = 30) -> None:
    """把待处理的事件跑完：渲染/布局都是事件驱动的，不 pump 就拍到空壳。"""
    for _ in range(rounds):
        qapp.processEvents()


def _open_window(urdf: Path):
    """起一个干净主窗口并载入 ``urdf``（走 showEvent 的真实载入路径）。"""
    qapp = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    window = main.MainWindow(str(urdf))
    window.resize(1680, 1050)
    window.show()
    _pump(qapp)
    # 清掉载入即选中的高亮（黄色轮廓），并按包围球自动取景让机器人完整入画；
    # 属性面板内容是载入时 set 的，清选中不会清掉它
    window.gl_widget.selected_object = None
    window.gl_widget.frame_robot()
    window.gl_widget.update()
    _pump(qapp)
    return window


def _grab(window, out_dir: Path, name: str) -> bool:
    """抓主窗口整屏并落盘；返回是否成功。"""
    pix = window.grab()
    path = out_dir / name
    if not pix.save(str(path)):
        print(f"[error] 保存失败: {path}")
        return False
    print(f"已保存: {path}  ({pix.width()}x{pix.height()})")
    return True


def _select_tab(tab_widget, widget) -> None:
    """切到指定页；面板不存在（功能被档位隐藏）时静默跳过。"""
    index = tab_widget.indexOf(widget)
    if index >= 0:
        tab_widget.setCurrentIndex(index)


def _show_joint_sliders(window) -> None:
    """把属性面板滚动到「关节」滑块：截图高度有限，滑块默认在折叠线以下。

    厂商 CAD 图的重点正是真机关节限位（如 Zu5 的 joint_2 -85..265°），
    拍不到滑块这张图就没拍到重点。
    """
    scroll = getattr(window.property_panel, "object_scroll", None)
    if scroll is None:
        return
    bar = scroll.verticalScrollBar()
    bar.setValue(bar.maximum())


def _teach_arc(window, steps: int = 5, step_deg: float = 30.0) -> None:
    """让当前机器人绕基座转出一 arc 示教点，并同步到代码编辑器。

    与 tests/test_path_point_actions.py 同一条通路：改渲染器关节值 →
    ``gl_widget.record_position()`` → ``set_recorded_positions`` 回灌编辑器，
    代码文本与视口路径线随之生成。
    """
    renderer = window.gl_widget.renderer
    if renderer is None:
        return
    first = renderer.actuated_names()[0]
    for i in range(steps):
        renderer.joint_values[first] = math.radians(i * step_deg)
        window.gl_widget.update()
        window.gl_widget.record_position()
    owner = window.gl_widget.current_robot
    window.code_editor.set_recorded_positions(
        window.gl_widget.get_recorded_positions(owner), owner
    )
    window.code_editor.sync_point_types()


def capture() -> int:
    parser = argparse.ArgumentParser(description="重拍 README 基础界面截图")
    parser.add_argument("--out", default=str(REPO_ROOT / "docs" / "screenshots"))
    args = parser.parse_args()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    _stub_dialogs()
    qapp = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    ok = []

    # 1) 主视图：UR5e + 左侧资源库 + 右侧属性（机器人信息 + 关节滑块）
    window = _open_window(UR5E_URDF)
    ok.append(_grab(window, out_dir, "main_ur5e.png"))

    # 2) 代码预览：先教一段弧线示教点，再切到左侧「代码预览」标签
    _teach_arc(window)
    _select_tab(window.left_tabs, window.code_editor)
    _pump(qapp)
    ok.append(_grab(window, out_dir, "path_code.png"))

    # 3) 离线编程：先跑一次校验（F7），面板里才有节拍与校验结论
    window.validate_program()
    _select_tab(window.right_tabs, window.program_panel)
    _pump(qapp, 40)
    ok.append(_grab(window, out_dir, "offline_programming.png"))

    window.close()
    _pump(qapp, 10)

    # 4) 厂商 CAD：JAKA Zu5 真机网格（新窗口，避免与 UR5e 同场景）
    #    属性面板滚到「关节」段：这张图的重点是真机关节限位
    window = _open_window(ZU5_REAL_URDF)
    _show_joint_sliders(window)
    _pump(qapp)
    ok.append(_grab(window, out_dir, "jaka_zu5_real.png"))

    window.close()
    _pump(qapp, 10)

    all_ok = all(ok)
    print("README SHOTS OK" if all_ok else "README SHOTS FAIL")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(capture())
