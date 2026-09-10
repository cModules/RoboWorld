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

from .components import COMPONENTS
from .scene import Scene, ROBOT_NAME

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
    /* margin-top 必须容纳一行标题文字(约16px)，否则标题上半截会被部件顶边裁掉 */
    border-radius: 4px; margin-top: 16px; padding-top: 6px;
}
QGroupBox::title {
    subcontrol-origin: margin; subcontrol-position: top left;
    left: 8px; padding: 0 4px;
}
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
    objectSelected = QtCore.Signal(str)  # 当前树选中对象名（None 表示清除）

    def __init__(self, scene: Optional[Scene] = None, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.setObjectName("ScenePanel")
        self.setStyleSheet(PANEL_STYLE)

        self.scene = scene or Scene()
        self._selected: Optional[str] = None

        self._setup_ui()
        self._refresh_tree()

    # --------------------------- 界面搭建 --------------------------- #

    def _setup_ui(self) -> None:
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(5, 5, 5, 5)
        root.setSpacing(5)

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

        # ---- 工艺时序（泳道图占位） ---- #
        swim_group = QtWidgets.QGroupBox("工艺时序（泳道图）")
        swim_layout = QtWidgets.QVBoxLayout(swim_group)
        swim_hint = QtWidgets.QLabel(
            "拖入对象即可生成泳道，按时间轴编排工艺步骤，后续将驱动“工艺 → 机器动作”的自动映射。"
        )
        swim_hint.setWordWrap(True)
        swim_hint.setStyleSheet("color:#7f8192; font-size:11px;")
        swim_layout.addWidget(swim_hint)
        self.swimlane_view = _ProcessSwimlane()
        swim_layout.addWidget(self.swimlane_view, 1)
        root.addWidget(swim_group, 1)

        root.addStretch(1)

    # --------------------------- 组件添加（供库拖拽/双击调用） --------------------------- #

    def add_component(self, component_id: str, params=None, select: bool = True):
        """把一个组件加入场景，并刷新树与属性面板。

        Args:
            component_id: 组件 id（见 ``COMPONENTS``）
            params: 可选参数字典，缺省用组件默认值
            select: 添加后是否自动选中
        Returns:
            新建的 SceneObject，或失败返回 None
        """
        try:
            obj = self.scene.add(component_id, params=params)
        except ValueError as exc:
            self.statusMessage.emit(f"添加失败: {exc}")
            return None
        self._refresh_tree()
        self.sceneChanged.emit()
        if select:
            self._select_object(obj.name)
        self.statusMessage.emit(f"已添加 {obj.name}")
        return obj

    # --------------------------- 场景树 --------------------------- #

    def _refresh_tree(self) -> None:
        self.tree.clear()
        items: Dict[str, QtWidgets.QTreeWidgetItem] = {}

        # 先建所有节点
        for name, obj in self.scene.objects.items():
            if name == ROBOT_NAME:
                kind = "机器人"
            else:
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
        if hasattr(self, "swimlane_view"):
            self.swimlane_view.set_scene(self.scene)

    def _on_tree_selected(self, current, previous) -> None:
        if current is None:
            self._selected = None
            self.objectSelected.emit(None)
            return
        name = current.data(0, QtCore.Qt.UserRole)
        if name not in self.scene.objects:
            return
        self._selected = name
        self.objectSelected.emit(name)

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
        self.objectSelected.emit(None)
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
        self.objectSelected.emit(None)
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
            if hasattr(self, "swimlane_view"):
                self.swimlane_view.set_scene(self.scene)
            self._refresh_tree()
            self.sceneChanged.emit()
        except Exception as exc:
            self.statusMessage.emit(f"机器人加载失败: {exc}")

    def update_robot_joints(self, joints_deg: List[float]) -> None:
        """同步机器人关节角（供视口拖动时调用）。"""
        if self.scene.has_robot:
            self.scene.set_robot_joints(joints_deg)


class _ProcessSwimlane(QtWidgets.QWidget):
    """工艺时序泳道图占位。

    现阶段仅展示「每个场景对象一行」的空壳，后续接入：拖拽生成步骤、
    时间轴缩放、步骤与机器动作的绑定。
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        self.header = QtWidgets.QLabel("时间 →")
        self.header.setStyleSheet("color:#9a9bab; font-size:10px;")
        layout.addWidget(self.header)
        self.list_widget = QtWidgets.QListWidget()
        layout.addWidget(self.list_widget, 1)

    def set_scene(self, scene) -> None:
        self.list_widget.clear()
        for name in scene.objects:
            self.list_widget.addItem(f"▢ {name}")
        if getattr(scene, "has_robot", False):
            self.list_widget.addItem("▢ 机器人")
