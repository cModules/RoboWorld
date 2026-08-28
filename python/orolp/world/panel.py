"""工作单元场景面板：组件库 + 场景树 + 位姿/参数编辑 + 碰撞检查。

与 :mod:`orolp.robot_library.panel` 的风格保持一致（深色配色、卡片列表），
放在主窗口左侧标签页里，与「机器人库」「路径代码」并列。

组件库部分完全由 :class:`~orolp.world.components.ComponentSpec` 的
``params`` 定义**自动生成**控件——新增组件只需在 components.py 里注册，
界面无需改动。
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional

from PySide6 import QtCore, QtWidgets

from .components import COMPONENTS, ComponentSpec, categories
from .scene import Scene

try:
    from ..pose import Pose

    _HAS_POSE = True
except Exception:  # pragma: no cover - pose 依赖 scipy，缺失时降级为纯平移
    _HAS_POSE = False

__all__ = ["ScenePanel"]

#: 与视口背景一致的深色配色
PANEL_STYLE = """
QWidget#ScenePanel { background: #26262e; }
QTreeWidget, QListWidget, QComboBox, QSpinBox, QDoubleSpinBox {
    background: #20212a; color: #d8d9e0;
    border: 1px solid #34353f; border-radius: 3px;
    selection-background-color: #3a3d4a;
}
QLabel { color: #9a9bab; }
QGroupBox {
    color: #c8cad4; border: 1px solid #34353f;
    border-radius: 4px; margin-top: 8px; padding-top: 6px;
}
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
QPushButton {
    background: #33343f; color: #d8d9e0;
    border: 1px solid #41424e; border-radius: 3px; padding: 4px 10px;
}
QPushButton:hover { background: #3d3e4a; }
QPushButton:pressed { background: #2a2b34; }
"""


class ScenePanel(QtWidgets.QWidget):
    """场景编辑面板。

    Signals:
        sceneChanged: 场景内容发生变化
        statusMessage(str): 需要显示到状态栏的提示
        collisionResult(int): 碰撞检查结果（干涉对数）
    """

    sceneChanged = QtCore.Signal()
    statusMessage = QtCore.Signal(str)
    collisionResult = QtCore.Signal(int)

    def __init__(self, scene: Optional[Scene] = None, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.setObjectName("ScenePanel")
        self.setStyleSheet(PANEL_STYLE)

        self.scene = scene or Scene()
        self._param_widgets: Dict[str, QtWidgets.QDoubleSpinBox] = {}
        self._selected: Optional[str] = None
        self._updating = False

        self._setup_ui()
        self._refresh_component_list()
        self._refresh_tree()

    # --------------------------- 界面搭建 --------------------------- #

    def _setup_ui(self) -> None:
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(5, 5, 5, 5)
        root.setSpacing(5)

        # ---- 组件库 ----
        lib_group = QtWidgets.QGroupBox("组件库（参数化生成）")
        lib_layout = QtWidgets.QVBoxLayout(lib_group)

        row = QtWidgets.QHBoxLayout()
        self.category_combo = QtWidgets.QComboBox()
        self.category_combo.addItems(categories())
        self.category_combo.currentTextChanged.connect(self._refresh_component_list)
        row.addWidget(self.category_combo, 1)
        lib_layout.addLayout(row)

        self.component_list = QtWidgets.QListWidget()
        self.component_list.currentItemChanged.connect(self._on_component_selected)
        self.component_list.setMaximumHeight(120)
        lib_layout.addWidget(self.component_list)

        self.component_desc = QtWidgets.QLabel("")
        self.component_desc.setWordWrap(True)
        self.component_desc.setStyleSheet("color:#7f8192; font-size:11px;")
        lib_layout.addWidget(self.component_desc)

        self.params_form = QtWidgets.QWidget()
        self.params_layout = QtWidgets.QFormLayout(self.params_form)
        self.params_layout.setContentsMargins(0, 0, 0, 0)
        lib_layout.addWidget(self.params_form)

        add_row = QtWidgets.QHBoxLayout()
        self.add_btn = QtWidgets.QPushButton("添加到场景")
        self.add_btn.clicked.connect(self._add_selected_component)
        add_row.addWidget(self.add_btn)
        lib_layout.addLayout(add_row)
        root.addWidget(lib_group)

        # ---- 场景树 ----
        tree_group = QtWidgets.QGroupBox("场景对象")
        tree_layout = QtWidgets.QVBoxLayout(tree_group)

        self.tree = QtWidgets.QTreeWidget()
        self.tree.setHeaderLabels(["对象", "类型"])
        self.tree.setRootIsDecorated(True)
        self.tree.setMaximumHeight(160)
        self.tree.currentItemChanged.connect(self._on_tree_selected)
        tree_layout.addWidget(self.tree)

        btn_row = QtWidgets.QHBoxLayout()
        self.remove_btn = QtWidgets.QPushButton("删除")
        self.remove_btn.clicked.connect(self._remove_selected)
        btn_row.addWidget(self.remove_btn)
        self.clear_btn = QtWidgets.QPushButton("清空")
        self.clear_btn.clicked.connect(self._clear_scene)
        btn_row.addWidget(self.clear_btn)
        tree_layout.addLayout(btn_row)
        root.addWidget(tree_group)

        # ---- 选中对象的位姿 ----
        pose_group = QtWidgets.QGroupBox("选中对象位姿")
        pose_layout = QtWidgets.QFormLayout(pose_group)
        self.pose_widgets: Dict[str, QtWidgets.QDoubleSpinBox] = {}
        for key, label, lo, hi in (
            ("x", "X", -20000, 20000),
            ("y", "Y", -20000, 20000),
            ("z", "Z", -5000, 10000),
            ("rz", "绕 Z (°)", -360, 360),
        ):
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(lo, hi)
            spin.setDecimals(1)
            spin.setSingleStep(10 if key != "rz" else 5)
            spin.setValue(0.0)
            spin.setEnabled(False)
            spin.valueChanged.connect(self._on_pose_edited)
            self.pose_widgets[key] = spin
            pose_layout.addRow(f"{label} (mm)" if key != "rz" else label, spin)
        root.addWidget(pose_group)

        # ---- 碰撞检查 ----
        check_group = QtWidgets.QGroupBox("干涉检查")
        check_layout = QtWidgets.QVBoxLayout(check_group)
        self.check_btn = QtWidgets.QPushButton("检查干涉 (F6)")
        self.check_btn.clicked.connect(self.run_collision_check)
        check_layout.addWidget(self.check_btn)
        self.collision_label = QtWidgets.QLabel("未检查")
        self.collision_label.setWordWrap(True)
        check_layout.addWidget(self.collision_label)
        root.addWidget(check_group)

        # ---- 场景文件 ----
        file_row = QtWidgets.QHBoxLayout()
        self.save_btn = QtWidgets.QPushButton("保存场景…")
        self.save_btn.clicked.connect(self.save_scene)
        self.load_btn = QtWidgets.QPushButton("加载场景…")
        self.load_btn.clicked.connect(self.load_scene)
        file_row.addWidget(self.save_btn)
        file_row.addWidget(self.load_btn)
        root.addLayout(file_row)

        self.stats_label = QtWidgets.QLabel("0 个对象")
        self.stats_label.setStyleSheet("color:#7f8192; font-size:11px;")
        root.addWidget(self.stats_label)
        root.addStretch(1)

    # --------------------------- 组件库 --------------------------- #

    def _refresh_component_list(self) -> None:
        cat = self.category_combo.currentText()
        self.component_list.clear()
        for spec in COMPONENTS.values():
            if spec.category == cat:
                item = QtWidgets.QListWidgetItem(spec.name)
                item.setData(QtCore.Qt.UserRole, spec.id)
                self.component_list.addItem(item)
        if self.component_list.count():
            self.component_list.setCurrentRow(0)

    def _on_component_selected(self, current, previous) -> None:
        if current is None:
            return
        spec = COMPONENTS[current.data(QtCore.Qt.UserRole)]
        self.component_desc.setText(spec.description)

        # 清空旧参数控件，按 spec 重建
        while self.params_layout.count():
            child = self.params_layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()
        self._param_widgets.clear()

        for p in spec.params:
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(p.minimum, p.maximum)
            spin.setDecimals(0 if p.step >= 1 else 1)
            spin.setSingleStep(p.step)
            spin.setValue(p.default)
            spin.setSuffix(f" {p.unit}")
            self._param_widgets[p.name] = spin
            self.params_layout.addRow(p.label, spin)

    def _add_selected_component(self) -> None:
        item = self.component_list.currentItem()
        if item is None:
            return
        spec: ComponentSpec = COMPONENTS[item.data(QtCore.Qt.UserRole)]
        params = {name: w.value() for name, w in self._param_widgets.items()}
        try:
            obj = self.scene.add(spec.id, params=params)
        except ValueError as exc:
            self.statusMessage.emit(f"添加失败: {exc}")
            return
        self._refresh_tree()
        self._select_object(obj.name)
        self.sceneChanged.emit()
        self.statusMessage.emit(f"已添加 {obj.name}")

    # --------------------------- 场景树 --------------------------- #

    def _refresh_tree(self) -> None:
        self.tree.clear()
        items: Dict[str, QtWidgets.QTreeWidgetItem] = {}

        # 先建所有节点
        for name, obj in self.scene.objects.items():
            kind = COMPONENTS[obj.component_id].name if obj.component_id else "自定义"
            items[name] = QtWidgets.QTreeWidgetItem([name, kind])
            items[name].setData(0, QtCore.Qt.UserRole, name)

        # 再按父子关系挂载（父不存在则挂根）
        for name, obj in self.scene.objects.items():
            parent_item = items.get(obj.parent) if obj.parent else None
            if parent_item is not None:
                parent_item.addChild(items[name])
            else:
                self.tree.addTopLevelItem(items[name])

        self.tree.expandAll()
        self.stats_label.setText(f"{len(self.scene)} 个对象")

    def _on_tree_selected(self, current, previous) -> None:
        if current is None:
            self._selected = None
            for w in self.pose_widgets.values():
                w.setEnabled(False)
            return
        name = current.data(0, QtCore.Qt.UserRole)
        if name not in self.scene.objects:
            return
        self._selected = name
        self._updating = True
        tf = self.scene.objects[name].pose
        xyz = tf.matrix[:3, 3] if hasattr(tf, "matrix") else [0, 0, 0]
        import math

        rz = math.degrees(math.atan2(tf.matrix[1, 0], tf.matrix[0, 0])) if hasattr(tf, "matrix") else 0.0
        self.pose_widgets["x"].setValue(float(xyz[0]))
        self.pose_widgets["y"].setValue(float(xyz[1]))
        self.pose_widgets["z"].setValue(float(xyz[2]))
        self.pose_widgets["rz"].setValue(rz)
        for w in self.pose_widgets.values():
            w.setEnabled(True)
        self._updating = False

    def _on_pose_edited(self) -> None:
        if self._updating or not self._selected:
            return
        if not _HAS_POSE:
            return
        obj = self.scene.objects.get(self._selected)
        if obj is None:
            return
        obj.pose = Pose.from_xyz_rpy(
            self.pose_widgets["x"].value(),
            self.pose_widgets["y"].value(),
            self.pose_widgets["z"].value(),
            0.0,
            0.0,
            self.pose_widgets["rz"].value(),
        )
        self.sceneChanged.emit()

    def _select_object(self, name: str) -> None:
        it = self.tree.findItems(name, QtCore.Qt.MatchExactly | QtCore.Qt.MatchRecursive, 0)
        if it:
            self.tree.setCurrentItem(it[0])

    def _remove_selected(self) -> None:
        if not self._selected:
            return
        self.scene.remove(self._selected)
        self._selected = None
        self._refresh_tree()
        self.sceneChanged.emit()
        self.statusMessage.emit("已删除对象")

    def _clear_scene(self) -> None:
        ret = QtWidgets.QMessageBox.question(
            self, "清空场景", "确定要移除所有场景对象吗？（机器人不受影响）"
        )
        if ret != QtWidgets.QMessageBox.Yes:
            return
        self.scene.clear()
        self._selected = None
        self._refresh_tree()
        self.sceneChanged.emit()
        self.statusMessage.emit("场景已清空")

    # --------------------------- 碰撞检查 --------------------------- #

    def run_collision_check(self) -> list:
        """执行干涉检查、更新显示，并返回干涉对列表。

        Returns:
            :class:`~orolp.world.scene.Collision` 列表；空列表表示无干涉。
            返回值（而非仅内部显示）是为了让视口复用同一次结果做高亮——
            否则同一帧要把 SAT 算两遍。
        """
        if not self.scene.has_robot and len(self.scene) == 0:
            self.collision_label.setText("场景为空")
            self.collision_label.setStyleSheet("color:#9a9bab;")
            self.collisionResult.emit(0)
            return []
        try:
            hits = self.scene.check_collisions(margin=-0.5)
        except Exception as exc:
            self.collision_label.setText(f"检查失败: {exc}")
            self.collision_label.setStyleSheet("color:#e0603a;")
            self.collisionResult.emit(0)
            return []

        if not hits:
            self.collision_label.setText("✓ 无干涉")
            self.collision_label.setStyleSheet("color:#4ec97a;")
            self.statusMessage.emit("干涉检查通过：无干涉")
        else:
            shown = "、".join(str(h) for h in hits[:6])
            more = f" …等共 {len(hits)} 处" if len(hits) > 6 else ""
            self.collision_label.setText(f"✗ {len(hits)} 处干涉：{shown}{more}")
            self.collision_label.setStyleSheet("color:#e0603a;")
            self.statusMessage.emit(f"发现 {len(hits)} 处干涉")
        self.collisionResult.emit(len(hits))
        return hits

    # --------------------------- 场景文件 --------------------------- #

    def save_scene(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "保存场景", "workcell.json", "场景文件 (*.json)"
        )
        if not path:
            return
        try:
            self.scene.save(path)
        except OSError as exc:
            QtWidgets.QMessageBox.critical(self, "保存失败", str(exc))
            return
        self.statusMessage.emit(f"场景已保存: {Path(path).name}")

    def load_scene(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "加载场景", "", "场景文件 (*.json)"
        )
        if not path:
            return
        try:
            self.scene = Scene.load(path)
        except (OSError, ValueError) as exc:
            QtWidgets.QMessageBox.critical(self, "加载失败", str(exc))
            return
        self._selected = None
        self._refresh_tree()
        self.sceneChanged.emit()
        self.statusMessage.emit(f"场景已加载: {Path(path).name}（{len(self.scene)} 个对象）")

    # --------------------------- 外部接口 --------------------------- #

    def set_scene(self, scene: Scene) -> None:
        """替换面板持有的场景对象。"""
        self.scene = scene
        self._selected = None
        self._refresh_tree()

    def set_robot(self, urdf_path: str, joints_deg: Optional[List[float]] = None) -> None:
        """设置场景中的机器人。

        Args:
            urdf_path: URDF 路径
            joints_deg: 关节角（度）
        """
        try:
            self.scene.set_robot(urdf_path, joints_deg)
        except Exception as exc:
            self.statusMessage.emit(f"机器人加载失败: {exc}")

    def update_robot_joints(self, joints_deg: List[float]) -> None:
        """同步机器人关节角（供视口拖动时调用）。"""
        if self.scene.has_robot:
            self.scene.set_robot_joints(joints_deg)
