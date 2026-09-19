#!/usr/bin/env python3
"""GUI 冒烟测试：「抓取工作站演示」一键入口全流程不崩溃（S4）。

离屏（QT_QPA_PLATFORM=offscreen）实例化主窗口，触发一键入口，跑完
「载场景 → IK 示例程序 → 校验出节拍 → 自动回放」全链路，并验证：

* 回放帧持续到达视口（轨迹插补在动）
* 夹爪状态信号发出（吸附/释放时序生效）
* 校验报告给出节拍
* 状态栏 / 工件下拉 / 路径编辑器数据一致

运行::

    python tools/smoke_gui_demo.py          # 退出码 0 = 通过
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))
sys.path.insert(0, str(REPO_ROOT))

from PySide6 import QtWidgets, QtCore  # noqa: E402

import app  # noqa: E402


def main() -> int:
    # 弹窗一律立即关闭：冒烟测试不能被模态对话框阻塞
    class _NoDialog:
        def __init__(self, *a, **k):
            pass

        def exec(self):
            return 0

        def show(self):
            pass

    for name in ("information", "warning", "critical"):
        setattr(
            QtWidgets.QMessageBox,
            name,
            staticmethod(lambda *a, **k: QtWidgets.QMessageBox.Ok),
        )
    QtWidgets.QMessageBox.__init__ = _NoDialog.__init__
    QtWidgets.QMessageBox.exec = _NoDialog.exec

    # 变量名不能叫 ``app``：它会遮蔽模块 ``app``（MainWindow 的所在），
    # 于是 ``app.MainWindow()`` 变成在 QApplication 上找属性——必然 AttributeError
    qapp = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    window = app.MainWindow()
    window.show()

    frames = []
    gripper_states = []
    # 多机改造后这两个信号都带上了机器人名：``(机器人名, 载荷)``
    window.program_panel.playbackFrame.connect(
        lambda _robot, values: frames.append(values)
    )
    window.program_panel.gripperStateChanged.connect(
        lambda _robot, action: gripper_states.append(action)
    )

    # 触发一键入口
    window.load_grasping_workstation_demo()
    for _ in range(30):
        qapp.processEvents()

    problems = []

    # 1) 示例程序已灌入且语义齐全
    positions = window.code_editor.get_recorded_positions()
    types = window.code_editor.get_point_types()
    if len(positions) != 8:
        problems.append(f"示教点数量 {len(positions)} != 8")
    if [t.name for t in types].count("PICK") != 1 or \
            [t.name for t in types].count("PLACE") != 1:
        problems.append(f"工艺语义不完整: {[t.name for t in types]}")

    # 2) 校验报告给出节拍
    report = window.program_panel.report
    if report is None or report.cycle_time is None:
        problems.append("校验报告缺少节拍")
    else:
        print(f"节拍: {report.cycle_time:.2f} s")

    # 3) 回放进行：让事件循环转 3 秒，统计到达视口的帧
    timer = QtCore.QTimer()
    timer.setSingleShot(True)
    done = {"flag": False}
    timer.timeout.connect(lambda: done.update(flag=True))
    timer.start(3000)
    while not done["flag"]:
        qapp.processEvents()
    if len(frames) < 10:
        problems.append(f"回放帧过少: {len(frames)}")
    else:
        print(f"回放帧: {len(frames)} 帧 / 3s")
    if not any(s in ("close", "open") for s in gripper_states):
        problems.append(f"未见夹爪状态变化: {gripper_states}")
    else:
        print(f"夹爪状态序列样例: {gripper_states[:6]}")

    # 4) 工件下拉已默认选中第一个工件（吸附/释放有对象可作用）
    if window.code_editor.get_workpiece() is None:
        problems.append("工件下拉未选中任何工件")

    window.program_panel.stop_playback()
    window.close()

    if problems:
        print("SMOKE FAIL:")
        for p in problems:
            print("  -", p)
        return 1
    print("SMOKE OK: 一键演示全流程通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
