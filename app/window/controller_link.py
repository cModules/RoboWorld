"""窗口级「真机连接」对话框（ADR-002 拆解第 3 步）。

从 ``main.py`` 的 ``MainWindow`` 原样搬出（方法体未改一字）。这一组是
**真机连接**的界面化（v1.1.1）：创建/复用模式less 对话框、把离线编程面板的
当前程序交出去部署、把真机回读的关节角摆进视口。

安全方向在本模块里是硬约束：关节回读**只改本侧**（视口 + 场景 + 属性面板），
绝不向控制器下发运动指令——方向只有"真机 → 仿真"。
"""

from __future__ import annotations

import math

from PySide6 import QtWidgets

from app.window.capabilities import (
    ControllerPanel,
    _HAS_CONTROLLER_PANEL,
    tr,
)

from support.i18n import tr


# 真机连接面板（v1.1.1）：SocketLink 的界面化——连接 / 部署 / 回读
class ControllerLinkMixin:
    """真机连接对话框的创建/复用与关节回读落地（宿主：``MainWindow``）。"""

    def show_controller_dialog(self) -> None:
        """打开「真机连接」对话框（模式less：连接期间界面仍可查看仿真）。

        首次打开时创建并接线，之后复用同一个对话框——重复打开不会叠出
        多条连接。每次打开都刷新机器人引用：用户可能在两次打开之间换了型号。
        """
        if not _HAS_CONTROLLER_PANEL:
            QtWidgets.QMessageBox.information(self, tr("真机连接"), tr("真机连接模块不可用。"))
            return
        if self._controller_dialog is None:
            dialog = QtWidgets.QDialog(self)
            dialog.setWindowTitle(tr("真机连接"))
            dialog.setModal(False)
            dialog.resize(400, 780)  # v1.1.4 加了传输选择与 SSH 字段
            layout = QtWidgets.QVBoxLayout(dialog)
            panel = ControllerPanel()
            layout.addWidget(panel)
            panel.set_robot(self.olp_robot)
            panel.set_program_provider(self._controller_program_payload)
            panel.statusMessage.connect(self.statusBar().showMessage)
            panel.jointsRead.connect(self.on_controller_joints_read)
            self._controller_dialog = dialog
            self._controller_panel = panel
        else:
            self._controller_panel.set_robot(self.olp_robot)
        self._controller_dialog.show()
        self._controller_dialog.raise_()
        self._controller_dialog.activateWindow()

    def _controller_program_payload(self):
        """把离线编程面板的当前程序交给真机连接面板部署。

        与「一键导出」同一条数据通路（示教点 → build_program → to_post），
        后处理器取导出打包标签里当前选择的（``None`` = 按机器人品牌自动推断）。
        """
        # 只判 hasattr：宿主窗口仅在编程面板可用时才创建它（从不置 None），
        # 所以 hasattr 已蕴含原 main.py 的 _HAS_PROGRAM 标志。
        if not (hasattr(self, "program_panel") and self.olp_robot):
            return None
        self._sync_program_points()
        program = self.program_panel.build_program()
        if program is None:
            return None
        return {
            "program": program,
            "post_processor": self.program_panel.combo_post.currentData(),
        }

    def on_controller_joints_read(self, joints) -> None:
        """真机回读的关节角（度）摆进视口——仿真立刻等于真机当前位形。

        只改**本侧**（视口 + 场景 + 属性面板），绝不向控制器下发任何运动指令；
        方向只有"真机 → 仿真"，这是安全的。
        """
        names = list(self.gl_widget.renderer.actuated_names())
        try:
            vals = [float(v) for v in joints]
            if len(vals) != len(names):
                raise ValueError(
                    tr("关节数不符（真机 {n}，模型 {m}）").format(n=len(vals), m=len(names))
                )
        except Exception as exc:
            self.statusBar().showMessage(tr("回读关节应用失败：{exc}").format(exc=exc))
            return
        # 与回放帧同一通路：视口 + 属性面板 + 场景同步 + 重绘。
        # 必须带上机器人名——否则真机的关节会被套到别的机器人身上。
        self.on_playback_frame(
            getattr(self.gl_widget, "current_robot", "") or "",
            {name: math.radians(v) for name, v in zip(names, vals)},
        )
        self.statusBar().showMessage(tr("已按真机位形摆好视口"))
