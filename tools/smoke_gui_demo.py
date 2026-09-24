#!/usr/bin/env python3
"""GUI 冒烟 / MVP 验收清单（v1.4.0）。

离屏（``QT_QPA_PLATFORM=offscreen``）跑完**对外承诺的那条闭环**：

    冷启动 → 一键演示 → 干涉检查(F6) → 校验与节拍(F7) → 回放（含抓取随动）
    → 出机器人代码 → 存 .orolp → 关 → 冷启动重开同一工程 → 退出

与 pytest 的分工：单元测试证明「每个零件算得对」，这里只问一句——
**新手第一次打开、照着演示点一遍，会不会撞上什么**。每一步都打印可见结果，
任何一步没达到预期就汇总到末尾一起报（不中途退出，一次跑完看全坏了几项）。

两个刻意的隔离（否则脚本会 hang 在等待人类点对话框）：

* ``OROLP_CONFIG_DIR`` 指向临时目录：冷启动＝干净用户，不会被开发者机器上
  残留的自动保存快照挡住（``maybe_prompt_recovery`` 会在 showEvent 里问人）；
* 模态对话框一律 stub：QMessageBox / QFileDialog / QDialog 都不真正弹。

卡死也不会无声无息：``faulthandler`` 超时自动 dump 全部线程栈——
最有信息量的失败报告是「卡在哪一行的现场」，而不是一句 timeout。

运行::

    python tools/smoke_gui_demo.py                  # 默认档（非 pytest 下＝MVP）
    python tools/smoke_gui_demo.py --features full  # 全功能档（2.0 内部用）对照

退出码 0 = 通过。
"""

from __future__ import annotations

import argparse
import faulthandler
import json
import os
import sys
import tempfile
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))
sys.path.insert(0, str(REPO_ROOT))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="GUI 冒烟 / MVP 验收清单")
    parser.add_argument(
        "--features",
        choices=("mvp", "full"),
        default=None,
        help="显式指定构建档位；不指定就用当前默认档（非 pytest 下＝mvp）",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=180,
        help="整体看门狗秒数：卡住时 dump 线程栈并退出（默认 180）",
    )
    return parser.parse_args(argv)


args = parse_args()

if args.features:
    os.environ["OROLP_FEATURES"] = args.features

# 冷启动＝干净用户：隔离用户级配置目录，避免读到开发机上的自动保存/最近打开
_CONFIG_DIR = Path(tempfile.mkdtemp(prefix="rw-smoke-"))
os.environ["OROLP_CONFIG_DIR"] = str(_CONFIG_DIR)

# 卡住不留悬案
faulthandler.dump_traceback_later(args.timeout, exit=True)

from PySide6 import QtCore, QtWidgets  # noqa: E402

import app  # noqa: E402

#: 构建档位（打印出来，报告里才知道这份结果属于哪一档）
try:
    from orolp.features import describe as _features_describe
except Exception:  # pragma: no cover - features 缺失时按全功能对待
    _features_describe = lambda: "full（features 模块缺失）"  # noqa: E731

from orolp.i18n import tr  # noqa: E402

#: 出码演示暂时替换掉的展示入口（finally 里还原）
_ORIGINAL_SHOW_GENERATED = app.MainWindow._show_generated_code


class _Checklist:
    """一边打印一边收集失败项。"""

    def __init__(self):
        self.problems = []

    def check(self, ok, message, detail=""):
        suffix = f" — {detail}" if detail else ""
        if ok:
            print(f"    ✓ {message}{suffix}")
        else:
            print(f"    ✗ {message}{suffix}")
            self.problems.append(f"{message}{suffix}")
        return bool(ok)

    def note(self, message):
        print(f"    · {message}")


def _stub_dialogs(projects_dir: Path):
    """把会阻塞的交互全部驯服。

    替换而不是禁用：行程还是要跑（生成的代码进了 `_show_generated_code` 就说明
    整条后处理链路通了），只是不等待人类输入。
    """
    _No = QtWidgets.QMessageBox.No
    for name in ("information", "warning", "critical", "question"):
        setattr(
            QtWidgets.QMessageBox,
            name,
            staticmethod(lambda *a, **k: _No),
        )
    QtWidgets.QDialog.exec = lambda self, *a, **k: QtWidgets.QDialog.Rejected
    QtWidgets.QFileDialog.getSaveFileName = staticmethod(
        lambda *a, **k: (str(projects_dir / "smoke.orolp"), "")
    )
    QtWidgets.QFileDialog.getOpenFileName = staticmethod(
        lambda *a, **k: (str(projects_dir / "smoke.orolp"), "")
    )


def _pump(qapp, rounds: int = 30):
    for _ in range(rounds):
        qapp.processEvents()


def main() -> int:
    print(f"构建档位: {_features_describe()}")
    print(f"用户配置目录（隔离）: {_CONFIG_DIR}")
    projects_dir = _CONFIG_DIR / "projects"
    projects_dir.mkdir(parents=True, exist_ok=True)
    _stub_dialogs(projects_dir)

    cl = _Checklist()

    # ---------------------------------------------------------------- 1 冷启动
    print("[1/9] 冷启动")
    qapp = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    # 变量名不能叫 ``app``：它会遮蔽模块 ``app``（MainWindow 的所在），
    # 于是 ``app.MainWindow()`` 变成在 QApplication 上找属性——必然 AttributeError
    window = app.MainWindow()
    window.show()
    _pump(qapp)
    cl.check(window.isVisible(), "主窗口已显示")
    cl.note(f"窗口标题: {window.windowTitle()}")

    # -------------------------------------------------------------- 2 一键演示
    print("[2/9] 一键演示（抓取工作站）")
    frames = []
    gripper_states = []
    # 多机改造后这两个信号都带上了机器人名：``(机器人名, 载荷)``
    window.program_panel.playbackFrame.connect(
        lambda _robot, values: frames.append(values)
    )
    window.program_panel.gripperStateChanged.connect(
        lambda _robot, action: gripper_states.append(action)
    )
    window.load_grasping_workstation_demo()
    _pump(qapp)

    positions = window.code_editor.get_recorded_positions()
    types = [t.name for t in window.code_editor.get_point_types()]
    cl.check(len(positions) == 8, "示教点 8 个", f"实得 {len(positions)}")
    cl.check(
        types.count("PICK") == 1 and types.count("PLACE") == 1,
        "抓取/放置语义完整",
        f"{types}",
    )
    cl.check(window.code_editor.get_workpiece() is not None, "工件下拉已选中")

    # ----------------------------------------------------------- 3 干涉检查 F6
    print("[3/9] 干涉检查 (F6)")
    window.check_collisions()
    _pump(qapp)
    label = window.scene_panel.collision_label.text()
    cl.check(
        label and label != tr("未检查"),
        "干涉检查已出结果",
        label.replace("\n", " ")[:60],
    )

    # ------------------------------------------------------- 4 校验与节拍 F7
    print("[4/9] 程序校验与节拍 (F7)")
    window.validate_program()
    _pump(qapp)
    report = window.program_panel.report
    if cl.check(report is not None and report.cycle_time is not None, "校验报告给出节拍"):
        cl.note(f"节拍 {report.cycle_time:.2f} s / 错误 {len(report.errors)}")

    # ---------------------------------------------------------------- 5 回放
    print("[5/9] 仿真回放（3 秒）")
    frames.clear()
    gripper_states.clear()
    timer = QtCore.QTimer()
    timer.setSingleShot(True)
    done = {"flag": False}
    timer.timeout.connect(lambda: done.update(flag=True))
    timer.start(3000)
    while not done["flag"]:
        qapp.processEvents()
    cl.check(len(frames) >= 10, "回放帧持续到达视口", f"{len(frames)} 帧 / 3s")
    cl.check(
        any(s in ("close", "open") for s in gripper_states),
        "夹爪状态随轨迹变化",
        f"{gripper_states[:6]}",
    )
    window.program_panel.stop_playback()

    # ---------------------------------------------------------------- 6 出码
    print("[6/9] 生成机器人代码 (Ctrl+G)")
    captured = {}
    app.MainWindow._show_generated_code = lambda self, code, robot: captured.update(
        code=code, robot=robot
    )
    try:
        window.generate_robot_code()
        _pump(qapp)
        code = captured.get("code")
        cl.check(bool(code and code.strip()), "后处理产出非空代码",
                 f"{len(code.splitlines()) if code else 0} 行")
    finally:
        app.MainWindow._show_generated_code = _ORIGINAL_SHOW_GENERATED

    # ---------------------------------------------------------------- 7 存工程
    print("[7/9] 保存工程 (.orolp)")
    window.save_project()
    _pump(qapp)
    saved = projects_dir / "smoke.orolp"
    if cl.check(saved.exists(), "工程文件已落盘", str(saved.name)):
        payload = json.loads(saved.read_text(encoding="utf-8"))
        cl.note(f"顶层键: {sorted(payload.keys())}")

    # ------------------------------------------------- 8 重开同一工程（冷启动）
    print("[8/9] 关闭并重开同一工程")
    window.program_panel.stop_playback()
    window.close()
    _pump(qapp, 10)

    reopened = app.MainWindow()
    try:
        reopened.show()
        _pump(qapp)
        ok = reopened.open_project_path(str(saved), record_recent=False)
        _pump(qapp)
        cl.check(ok, "工程载入成功")
        restored = reopened.code_editor.get_recorded_positions()
        cl.check(
            len(restored) == len(positions),
            "示教点往返一致",
            f"{len(restored)} vs {len(positions)}",
        )
    finally:
        reopened.program_panel.stop_playback()
        reopened.close()

    # --------------------------------------------------------------- 9 干净退出
    print("[9/9] 退出")
    _pump(qapp, 10)
    # 崩溃日志清理过了：本次冒烟没有写 ~/.orolp（隔离目录），也不用为一時的
    # 临时目录负责——tmppath 由系统回收（macOS NSTemporaryDirectory 自动清理）

    if cl.problems:
        print(f"\nSMOKE FAIL（档位 {_features_describe()}）:")
        for p in cl.problems:
            print("  -", p)
        return 1
    print(f"\nSMOKE OK（档位 {_features_describe()}）: MVP 全流程通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
