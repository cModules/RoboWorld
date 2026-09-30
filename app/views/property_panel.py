"""设备属性面板（ADR-002 拆解第 2 步）。

归属：App / GUI 层 ``app.views``——选中对象后编辑信息 / 位置 / 参数 / 关节 /
挂载。拆成三块只为不制造新的超限文件（AGENTS.md #26：GUI 组件 300 行），行为未变：
UI 骨架见 ``property_panel_ui``，动态行见 ``property_controls``（mixin）。
"""

import math
from typing import Optional

import numpy as np
from PySide6 import QtWidgets
from PySide6.QtCore import Signal

from app.views.property_controls import ControlRowsMixin
from app.views.property_panel_ui import build_ui, fill_robot_spec
from app.logging import logger
from support.i18n import tr
from domain.pose import Pose, matrix_to_rpy
from adapter.robot_library.urdf_model import rpy_to_matrix

# 环境建模（组件库参数规格 + 放置工具）。模块自洽：不复用 main.py 的全局 try 块，
# 免得面板被单独实例化（测试/未来的其它窗口）时拿不到。
try:
    from domain.world import placement as _placement
    from domain.world.components import COMPONENTS

    _HAS_WORLD = True
except Exception:  # pragma: no cover - world 模块缺失时退化为只读面板
    _HAS_WORLD = False
    COMPONENTS = None
    _placement = None


class DevicePropertyPanel(ControlRowsMixin, QtWidgets.QWidget):
    """选中对象的属性编辑面板。"""

    objectChanged = Signal()  # 场景对象属性被编辑后触发视口重绘
    #: 关节滑块改动 → 驱动视口/记录。
    #:
    #: 三个参数依次是 **机器人名 / 关节名 / 弧度**。机器人名不能省：单薄的
    #: ``(关节名, 值)`` 只能落到"当前渲染器"身上，而滑块是按**选中那台**的关节
    #: 建的——两者错位时，拖 A 的滑块却把值写进了 B，表现为两台一起动。
    jointChanged = Signal(str, str, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.log_text = None
        self.name_label = None
        self.type_label = None
        self.manufacturer_label = None
        self.dof_label = None
        #: 当前场景引用（由 MainWindow 注入）；选中场景对象时用于读写属性
        self.scene = None
        self.current_object = None
        self._has_robot_model = False
        self.obj_pose_widgets = {}
        self.obj_param_widgets = {}
        self.sliders = {}
        self.value_labels = {}
        #: 设备中可动轴（关节）的滑块与数值标签
        self.axis_sliders = {}
        self.axis_value_labels = {}
        self._renderer = None
        #: 当前「关节」滑块所属的机器人名——多机时 :attr:`jointChanged` 要据此
        #: 指名写入，不能再让 MainWindow 猜"当前渲染器"是谁
        self._active_robot: Optional[str] = None
        #: 「按机器人名取渲染器」回调，由主窗口注入 ``gl_widget.renderer_of``。
        #: 没有它，选中 A 也只能沿用一个陈旧渲染器（通常是最后加载那台）。
        self.renderer_provider = None
        build_ui(self)

    def update_robot_info(self, renderer, entry=None, name: Optional[str] = None):
        """更新机器人信息区。

        Args:
            renderer: 已加载 URDF 的渲染器
            entry: 机器人库条目；提供时可回显负载、臂展等完整规格
            name: 这台机器人在场景里的名字。**必须给**：规格要按名字留存，
                否则只会记最后加载那台——多机时选中 A，规格块却显示 B 的
                型号/厂商（2026-09-21 用户反馈）。``None`` 退回旧行为（单机）。
        """
        self._renderer = renderer
        self._has_robot_model = True
        if name is not None:
            # 规格按机器人名各存一份：set_object 选中谁就回显谁的
            if not hasattr(self, "_robot_entries"):
                self._robot_entries = {}
            self._robot_entries[name] = entry
        revolute = [j for j in renderer.joints.values() if j["type"] == "revolute"]
        self.dof_label.setText(str(len(revolute)))
        fill_robot_spec(self, renderer, entry)

        self.log_text.append(tr("已载入: {name}").format(name=renderer.robot_name))
        self.log_text.append(tr("关节数: {n}").format(n=len(revolute)))

    # --------------------------- 对象属性动态编辑 --------------------------- #

    def set_scene(self, scene) -> None:
        """注入当前工作单元场景，供选中对象时读写属性。"""
        self.scene = scene

    def set_object(self, name: Optional[str]) -> None:
        """按选中对象动态切换右面板内容：信息 / 位置 / 参数 三块。

        Args:
            name: 选中对象名；``None`` / 不在场景中 → 回到默认视图（仅信息）。
        """
        if not _HAS_WORLD or self.scene is None or not name or name not in self.scene.objects:
            # 默认视图：未选中对象 → 隐藏「对象属性」
            self.current_object = None
            self.object_scroll.hide()
            return

        self.current_object = name
        obj = self.scene.objects[name]
        # 必须用 is_robot 而不是"名字等于主机器人名"：多机时第二台叫
        # 「机器人_2」，按名字比会让规格块/关节滑块/挂载下拉统统不显示
        is_robot = self.scene.is_robot(name)
        spec = COMPONENTS.get(obj.component_id) if obj.component_id else None

        # 显示「对象属性」容器（内含信息 / 位置 / 参数）
        self.object_scroll.show()
        self.object_group.show()

        self.obj_name_label.setText(name)
        if is_robot:
            self.obj_type_label.setText(tr("机器人"))
            self.robot_spec_widget.setVisible(True)
            # 规格块按**选中的这台**回显：载入时各台的规格已按名字留存
            # （update_robot_info），只回显最后加载那台的话，选中 A 显示的
            # 却是 B 的型号/厂商（2026-09-21 用户反馈）
            fill_robot_spec(
                self,
                self._renderer_for_robot(name),
                getattr(self, "_robot_entries", {}).get(name),
            )
            self.note_label.setVisible(bool(self.note_label.text()))
            # 机器人可以挂到底座/台面上（基座随之随动）。但「装夹到末端」对
            # 机器人自身没有意义——那是"把工具装到它的法兰上"，故只留下拉。
            self.obj_mount_widget.setVisible(True)
            self.mount_btn_row.setVisible(False)
            self._refresh_parent_combo(name)
        else:
            # 与设备库同一套取值：显示**分类**，组件名由「名称」行承载
            self.obj_type_label.setText(tr(spec.category) if spec else tr("自定义"))
            self.robot_spec_widget.setVisible(False)
            self.note_label.setVisible(False)
            self.obj_mount_widget.setVisible(True)
            self.mount_btn_row.setVisible(True)
            self._refresh_parent_combo(name)

        # 位姿回填（blockSignals 避免触发写回）
        # matrix_to_rpy 返回弧度，转成角度后再填进 6 个输入框
        # 用**世界坐标**回填：挂载件（如装到底座上的机器人）的局部 pose 是相对
        # 父对象的——两台挂法相同的机器人局部值一模一样（0,0,底座高），看起来
        # "位置永远不变"，也和视口里拖拽落到的位置对不上（2026-09-21 用户反馈）
        tf_matrix = self.scene.world_transform(name)
        xyz = tf_matrix[:3, 3]
        rx, ry, rz = (math.degrees(v) for v in matrix_to_rpy(tf_matrix))
        for key, val in (
            ("x", float(xyz[0])),
            ("y", float(xyz[1])),
            ("z", float(xyz[2])),
            ("rx", rx),
            ("ry", ry),
            ("rz", rz),
        ):
            w = self.obj_pose_widgets[key]
            w.blockSignals(True)
            w.setValue(val)
            w.blockSignals(False)

        # 参数（模型参数化）与关节（姿态调节）各归其位：
        # - 机器人：无尺寸参数，关节面板给 6 轴滑块
        # - 组件：参数面板给尺寸参数（已排除轴），关节面板给它的可动轴
        # 机器人必须按**自己的名字**取渲染器：沿用 :attr:`_renderer`（最后加载
        # 那台）会让选中 A 时滑块仍是 B 的关节组——多机串扰的源头在这里
        robot_renderer = self._renderer_for_robot(name) if is_robot else None
        if is_robot and robot_renderer is not None:
            self._rebuild_param_rows(None, {})
            self._setup_joint_controls(robot_renderer, name)
        else:
            self._rebuild_param_rows(spec, obj.params)
            self._rebuild_axis_rows(spec, obj)

    def _on_obj_pose_edited(self) -> None:
        if not self.current_object or self.scene is None:
            return
        obj = self.scene.objects.get(self.current_object)
        if obj is None:
            return
        # 输入框的语义与显示一致：**世界坐标**。挂载件必须先经父系逆变换换回
        # 局部 pose 再落盘——直接把世界值当局部值写，对象会被"瞬移"到父系
        # 原点附近（世界系 → 局部系的换算口径与 placement.apply_world_pose 一致）
        world = rpy_to_matrix(
            [
                math.radians(self.obj_pose_widgets["rx"].value()),
                math.radians(self.obj_pose_widgets["ry"].value()),
                math.radians(self.obj_pose_widgets["rz"].value()),
            ]
        )
        world[:3, 3] = [
            self.obj_pose_widgets["x"].value(),
            self.obj_pose_widgets["y"].value(),
            self.obj_pose_widgets["z"].value(),
        ]
        parent_tf = self.scene.parent_transform(self.current_object)
        local = np.linalg.inv(parent_tf) @ world
        lx, ly, lz = (float(v) for v in local[:3, 3])
        lrx, lry, lrz = (math.degrees(v) for v in matrix_to_rpy(local))
        obj.pose = Pose.from_xyz_rpy(lx, ly, lz, lrx, lry, lrz)
        self.objectChanged.emit()

    def _descendant_names(self, name: str) -> set:
        """对象自身及其全部后代名——挂载候选里必须排除，否则会成环。"""
        out = {name}
        changed = True
        while changed:
            changed = False
            for o in self.scene.objects.values():
                if o.parent in out and o.name not in out:
                    out.add(o.name)
                    changed = True
        return out

    def _refresh_parent_combo(self, name: str) -> None:
        """重建「挂载到」下拉：场景根 + 所有不会形成环的候选父对象。"""
        combo = self.obj_parent_combo
        combo.blockSignals(True)
        combo.clear()
        combo.addItem(tr("(场景根)"), None)
        excluded = self._descendant_names(name)
        for o in self.scene.objects.values():
            if o.name in excluded:
                continue
            combo.addItem(o.name, o.name)
        cur = self.scene.objects[name].parent
        idx = combo.findData(cur)
        combo.setCurrentIndex(idx if idx >= 0 else 0)
        combo.blockSignals(False)

    def _on_mount_to_robot(self) -> None:
        """把当前对象装夹到机器人末端（主机器人法兰，坐标系自动对齐）。"""
        name = self.current_object
        if not name or self.scene is None or not getattr(self.scene, "robots", None):
            return
        robot = self.scene.robot_names[0]
        if name == robot:
            return
        try:
            self.scene.attach(name, robot, keep_world=False)
        except (KeyError, ValueError) as exc:
            logger.warning("component_grip_failed: %s", exc)
            return
        self.set_object(name)
        self.objectChanged.emit()

    def _on_unmount(self) -> None:
        """解除挂载并保持当前世界位姿（停在原地，不再随机器人动）。"""
        name = self.current_object
        if not name or self.scene is None:
            return
        self.scene.detach(name)
        self.set_object(name)
        self.objectChanged.emit()

    def _on_obj_parent_edited(self) -> None:
        """切换挂载父对象。

        挂到机器人＝**工具安装**语义（吸附到法兰原点）；挂到其它对象＝
        **放到其上**语义（保持世界位置，只建立随动关系）。
        """
        name = self.current_object
        if not name or self.scene is None:
            return
        target = self.obj_parent_combo.currentData()
        try:
            # 挂到**任意一台**机器人（不只是主机器人）的法兰都要走装夹对齐
            self.scene.attach(name, target, keep_world=(target not in self.scene.robots))
            # 机器人挂到底座/台面：要"坐上去"（上表面正中央），而不是停在
            # 原地只建立随动关系——后者看着像根本没挂上
            if self.scene.is_robot(name) and target and not self.scene.is_robot(target):
                _placement.mount_on_top_of(self.scene, name, target)
        except (KeyError, ValueError) as exc:
            self._refresh_parent_combo(name)
            logger.warning("object_mount_failed: %s", exc)
            return
        # 局部位姿已被 attach 改写，整块回填一遍（位置分组也要跟着变）
        self.set_object(name)
        self.objectChanged.emit()

    def _on_obj_param_edited(self, name: str, value: float) -> None:
        if not self.current_object or self.scene is None:
            return
        obj = self.scene.objects.get(self.current_object)
        if obj is None:
            return
        obj.params[name] = float(value)
        self.objectChanged.emit()
