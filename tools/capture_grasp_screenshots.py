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
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))
sys.path.insert(0, str(REPO_ROOT))

from PySide6 import QtWidgets, QtCore  # noqa: E402

import rw  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default=str(REPO_ROOT / "docs" / "screenshots"))
    args = parser.parse_args()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 模态弹窗一律吞掉：自动化截图不能被对话框阻塞
    for name in ("information", "warning", "critical"):
        setattr(
            QtWidgets.QMessageBox,
            name,
            staticmethod(lambda *a, **k: QtWidgets.QMessageBox.Ok),
        )

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    window = rw.MainWindow()
    window.resize(1680, 1050)
    window.show()
    for _ in range(20):
        app.processEvents()

    window.load_grasping_workstation_demo()
    for _ in range(30):
        app.processEvents()

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

    # 2) 回放中：让回放跑 2.2 秒（约到抓取接近段）再抓视口
    timer = QtCore.QTimer()
    timer.setSingleShot(True)
    done = {"flag": False}
    timer.timeout.connect(lambda: done.update(flag=True))
    timer.start(2200)
    while not done["flag"]:
        app.processEvents()
    window.raise_()
    grab(window.gl_widget, "grasping_playback.png")

    # 3) 离线编程面板：校验报告 + 节拍
    window.right_tabs.setCurrentWidget(window.program_panel)
    for _ in range(20):
        app.processEvents()
    grab(window, "grasping_offline_programming.png")

    window.program_panel.stop_playback()
    window.close()

    ok = len([s for s in saved if s]) == 3
    print("SCREENSHOTS OK" if ok else "SCREENSHOTS FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
