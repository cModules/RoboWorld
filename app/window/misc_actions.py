"""窗口级零散动作（ADR-002 拆解第 9 步）。

体量小、彼此只有一个共同点：都是**菜单/信号直接触发的一次性动作**——抓取向导
入口、干涉检查、选中机器人解析、路径编辑器的回调、视图复位。方法体从 ``main.py``
原样搬出，可选依赖的降级探测复制到本模块。

``_focus_tab`` 是这组的公共小工具：面板在左还是在右是布局的事，按"它在哪组里"
查找，以后挪位置不用改调用方。
"""

from __future__ import annotations

from typing import Optional

from PySide6 import QtWidgets

from app.window.capabilities import (
    _HAS_PICKPLACE,
    tr,
)

from support.i18n import tr


# 抓取语义（示教点类型）。缺失时点类型的回灌整体跳过——这是真标志，不是
# hasattr 能蕴含的（面板存在不代表语义模块可用），故**保留**判断。
class MiscActionsMixin:
    """菜单/信号直接触发的一次性动作。"""

    def open_grasp_wizard(self) -> None:
        """抓取向导（Ctrl+Shift+G）：由抓取点/放置点自动生成接近/提升/撤离点。"""
        # 只判 hasattr：宿主窗口仅在编程面板可用时才创建它（从不置 None）
        if not hasattr(self, "program_panel"):
            QtWidgets.QMessageBox.information(self, tr("抓取向导"), tr("离线编程模块不可用。"))
            return
        if not getattr(self.program_panel, "open_grasp_wizard", None):
            QtWidgets.QMessageBox.information(self, tr("抓取向导"), tr("当前版本不支持抓取向导。"))
            return
        self._focus_tab(self.program_panel)
        self._sync_program_points()
        self.program_panel.open_grasp_wizard()

    def _on_grasp_cycle_applied(self, name: str) -> None:
        """抓取向导写完示教点：把点位回灌**那一台**的视口与路径编辑器。

        否则向导生成的点只躺在编程面板里——视口的路径可视化还画着旧的那串，
        用户会以为向导没生效。按名字回灌是多机的安全底线：写错桶等于把这台
        的点画到另一台身上。
        """
        if not hasattr(self, "program_panel"):
            return
        getter = getattr(self.program_panel, "get_taught_points", None)
        if getter is None:
            return
        current = getattr(self.gl_widget, "current_robot", None)
        key = name or current
        positions, types, _motions = getter(key)
        self.gl_widget.set_recorded_positions(list(positions), key)
        self.gl_widget.update()
        if key == current:
            self.code_editor.set_recorded_positions(list(positions), key)
            if _HAS_PICKPLACE and types is not None:
                self.code_editor.point_types = list(types)
                self._point_types_by_robot[key] = list(types)
            self.code_editor.sync_point_types()
            self.code_editor.pos_count_label.setText(tr("示教点：{n}").format(n=len(positions)))
            self.code_editor._update_code_with_positions()
            self._sync_program_points(key)

    def _focus_tab(self, widget) -> None:
        """把某个面板切到它所在那一组的前台。

        面板在左还是在右是布局的事（历史上一换就是好几处 ``setCurrentWidget``
        忘了跟着改，点了按钮却看不到对应面板）。这里按"它在哪组里"找，
        以后再挪位置不用改调用方。
        """
        for tabs in (getattr(self, "left_tabs", None), getattr(self, "right_tabs", None)):
            if tabs is not None and tabs.indexOf(widget) >= 0:
                tabs.setCurrentWidget(widget)
                return

    def check_collisions(self) -> None:
        """执行工作单元干涉检查（F6），并把结果高亮到视口。"""
        # 只判 hasattr：宿主窗口仅在环境建模可用时才创建场景面板（从不置 None）
        if not hasattr(self, "scene_panel"):
            QtWidgets.QMessageBox.information(self, tr("干涉检查"), tr("环境建模模块不可用。"))
            return
        self._focus_tab(self.scene_panel)
        hits = self.scene_panel.run_collision_check()
        self.gl_widget.set_collision_results(hits)

        # 干涉结果写进常驻状态栏：演示时一眼可见，不用去翻面板
        if hasattr(self, "status_collision"):
            self.status_collision.setText(tr("干涉 {n}").format(n=len(hits)))
            self.status_collision.setStyleSheet(
                "color:#ff8a80; padding:0 8px; font-size:11px;"
                if hits
                else "color:#a5d6a7; padding:0 8px; font-size:11px;"
            )

    def _robot_for_selection(self) -> Optional[str]:
        """**选中对象**关联到哪台机器人；没选中 / 关联不到返回 ``None``。

        选中夹爪、被夹住的工件同样算选中了那台——它们本来就挂在法兰上，路径
        是那台的，挂载件随法兰一起动。台面、地上没被夹住的工件关联不到任何
        机器人，返回 ``None``，走"从代码选中行号处全局运行"。
        """
        scene = getattr(self.gl_widget, "scene", None)
        selected = getattr(self.gl_widget, "selected_object", None)
        if scene is None or not selected:
            return None
        if scene.is_robot(selected):
            return selected
        return scene.mounted_robot(selected)

    def _robot_usable(self, name: Optional[str]) -> bool:
        """这台机器人现在**真的驱动得动**吗——在场景里、且有运动学链。

        URDF 失效时场景里会留一个"壳"：名字还在、也选得中，但没有运动学，
        驱动它没有意义。
        """
        scene = getattr(self.gl_widget, "scene", None)
        if not name or scene is None or not scene.is_robot(name):
            return False
        return scene.kinematics_of(name) is not None

    def on_positions_parsed(self):
        """当路径代码解析完成后，更新 GLWidget 的路径可视化。

        按**每条指令的归属**分桶回写：整份按 ``current_robot`` 写回时，选中
        B 播一次就会把编辑器里 A 的点复制到 B 名下，B 自己的路径被顶掉——
        用户看到的就是"选了第二台，第一台的路径被复制过去替换了"。
        """
        self._sync_viewport_paths_from_editor()
        self._sync_program_points()

    def on_clear_requested(self):
        # 路径属于机器人：清的是**当前这台**的——不能把别的机器人示教的一并抹掉
        key = self.gl_widget.current_robot
        self.gl_widget.clear_recorded_positions(key)
        self.code_editor.set_recorded_positions([], key)
        self.code_editor.pos_count_label.setText(tr("示教点：0"))
        self.code_editor._update_code_with_positions()
        self._sync_program_points(key)
        self.statusBar().showMessage("Recorded positions cleared")

    def reset_view(self):
        self.gl_widget.camera_dist = 1.2
        self.gl_widget.camera_az = 45.0
        self.gl_widget.camera_el = 30.0
        self.gl_widget.update()
