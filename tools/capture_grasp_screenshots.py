#!/usr/bin/env python3
"""S5 展示物料：为「抓取工作站演示」抓取 GUI 截图（docs/screenshots/）。

复用 GUI 冒烟测试（tools/smoke_gui_demo.py）的驱动方式：实例化真实
主窗口 → 触发一键演示入口 → 回放动画，用 ``widget.grab()`` 抓三张图：

* ``grasping_workstation.png``       主视图（3D 视口 + 工作站全景）
* ``grasping_playback.png``          回放中（机器人抓取动作 + 工件）
* ``grasping_offline_programming.png`` 离线编程面板（校验报告 + 节拍）

运行::

    python tools/capture_grasp_screenshots.py
    # 可选: --out docs/screenshots 指定输出目录
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

# 冷启动＝干净用户：隔离用户级配置目录，否则 showEvent 里的「自动恢复询问」
# 会弹真对话框，截图进程卡在等人类点按钮
_CONFIG_DIR = Path(tempfile.mkdtemp(prefix="rw-shots-"))
os.environ["CORE_CONFIG_DIR"] = str(_CONFIG_DIR)

from PySide6 import QtWidgets, QtCore  # noqa: E402

import main  # noqa: E402
from app.views.robot_renderer import HOME_POSE_PATTERN_DEG  # noqa: E402


def _apply_home_pose(gl_widget) -> None:
    """把当前机器人摆回待机位姿（与渲染器载入时同一套 HOME 角）。"""
    renderer = gl_widget.renderer
    if renderer is None:
        return
    values = {
        name: math.radians(HOME_POSE_PATTERN_DEG[i])
        for i, name in enumerate(renderer.actuated_names())
        if i < len(HOME_POSE_PATTERN_DEG)
    }
    gl_widget.set_joint_values(values)


def capture() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default=str(REPO_ROOT / "docs" / "screenshots"))
    args = parser.parse_args()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 模态弹窗一律吞掉：自动化截图不能被对话框阻塞
    for name in ("information", "warning", "critical", "question"):
        setattr(
            QtWidgets.QMessageBox,
            name,
            staticmethod(lambda *a, **k: QtWidgets.QMessageBox.Ok),
        )
    QtWidgets.QDialog.exec = lambda self, *a, **k: QtWidgets.QDialog.Rejected

    # 变量名不能叫 ``app``：它会遮蔽 MainWindow 所在的 ``app`` 包
    qapp = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    window = main.MainWindow()
    window.resize(1680, 1050)
    window.show()
    for _ in range(20):
        qapp.processEvents()

    window.load_grasping_workstation_demo()
    for _ in range(30):
        qapp.processEvents()

    # 演示会自动选中机器人（视口画黄色高亮）；截图前清掉选中，恢复常规配色。
    # 属性面板内容不受影响（载入时已 set），回放/工件跟随也不依赖选中态。
    window.gl_widget.selected_object = None

    # 回放起点受时序竞争影响，逐帧位形不可复现；先停回放、把机器人摆回
    # 待机位姿再拍全景——否则相机取景是按载入位姿算的，回放中途的位形
    # 可能顶出画面（手臂出画）。全景拍完再重新开始回放拍「回放中」。
    window.program_panel.stop_playback()
    renderer = window.gl_widget.renderer
    if renderer is not None:
        _apply_home_pose(window.gl_widget)
    window.gl_widget.update()
    for _ in range(10):
        qapp.processEvents()

    saved = []

    def grab(widget, name: str) -> None:
        pix = widget.grab()
        path = out_dir / name
        if pix.save(str(path)):
            saved.append(path.name)
            print(f"已保存: {path}  ({pix.width()}x{pix.height()})")
        else:
            print(f"[error] 保存失败: {path}")
            saved.append("")

    # 1) 主视图：工作站全景（机器人 + 输送带 + 托盘 + 状态栏节拍）
    grab(window, "grasping_workstation.png")

    # 2) 回放中：重新开始回放，跑 0.8 秒（约到输送带上方抓取段）再抓视口
    window.program_panel.start_playback()
    timer = QtCore.QTimer()
    timer.setSingleShot(True)
    done = {"flag": False}
    timer.timeout.connect(lambda: done.update(flag=True))
    timer.start(800)
    while not done["flag"]:
        qapp.processEvents()
    window.raise_()
    grab(window.gl_widget, "grasping_playback.png")

    # 3) 离线编程面板：校验报告 + 节拍（演示载入时已自动校验，
    #    回放进行中再校验会与关节回写竞争、产生假错误，这里不再重复调）
    window.right_tabs.setCurrentWidget(window.program_panel)
    for _ in range(20):
        qapp.processEvents()
    grab(window, "grasping_offline_programming.png")

    window.program_panel.stop_playback()
    window.close()

    ok = len([s for s in saved if s]) == 3
    print("SCREENSHOTS OK" if ok else "SCREENSHOTS FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(capture())
