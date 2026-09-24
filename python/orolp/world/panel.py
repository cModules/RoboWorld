"""工作单元场景面板：组件库 + 场景树 + 位姿/参数编辑 + 碰撞检查 + 工艺时序。

与 :mod:`orolp.robot_library.panel` 的风格保持一致（深色配色、卡片列表），
放在主窗口左侧标签页里，与「机器人库」「路径代码」并列。

组件库部分完全由 :class:`~orolp.world.components.ComponentSpec` 的
``params`` 定义**自动生成**控件——新增组件只需在 components.py 里注册，
界面无需改动。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Optional

from PySide6 import QtCore, QtGui, QtWidgets

from .components import COMPONENTS
from .placement import mount_on_top_of as _mount_on_top_of
from .scene import Scene, ROBOT_NAME
from ..cell import HANDSHAKE_SIGNALS
from ..planner import SEVERITY_ERROR, SEVERITY_WARNING
from ..sequence import (
    CallStep,
    DwellStep,
    ProcessSequence,
    SeqIssue,
    SignalStep,
    WaitStep,
)
from ..i18n import tr
from ..features import enabled as _feature_enabled

__all__ = ["ScenePanel", "SequenceEditor", "SequenceCanvas"]

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


class _DeviceTree(QtWidgets.QTreeWidget):
    """可拖拽改写层级的设备树。

    拖拽**不搬运节点**——只把"谁挂到谁"发成 :attr:`itemReparented` 信号，
    由 :class:`ScenePanel` 落到 ``Scene.attach`` 再整树重建。若放任基类
    ``InternalMove`` 自行搬节点，树结构与场景数据会立刻脱钩。
    """

    itemReparented = QtCore.Signal(str, object)  # (子对象名, 新父名或 None)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setDragEnabled(True)
        self.setAcceptDrops(True)
        self.setDropIndicatorShown(True)
        self.setDragDropMode(QtWidgets.QAbstractItemView.InternalMove)
        self.setDefaultDropAction(QtCore.Qt.MoveAction)

    def dropEvent(self, event):
        dragged = self.currentItem()
        pos = event.position().toPoint() if hasattr(event, "position") else event.pos()
        target = self.itemAt(pos)
        name = dragged.data(0, QtCore.Qt.UserRole) if dragged is not None else None
        new_parent = target.data(0, QtCore.Qt.UserRole) if target is not None else None
        # 让基类放弃内建搬移：树由 ScenePanel 依场景数据重建
        event.setDropAction(QtCore.Qt.IgnoreAction)
        event.ignore()
        if name and name != new_parent:
            self.itemReparented.emit(name, new_parent)


class ScenePanel(QtWidgets.QWidget):
    """场景编辑面板。

    Signals:
        sceneChanged: 场景内容发生变化
        statusMessage(str): 需要显示到状态栏的提示
        collisionResult(int): 碰撞检查结果（干涉对数）
        robotRemoved(str): 某台机器人被移出场景（删除对象 / 清空场景）。
            主窗口据此把它的程序、轨迹、示教点、渲染器一并清除——否则会留下
            "能校验能出码、却没有对应机器人"的孤儿状态
        sceneReplaced(object): 场景引用被**整体替换**（加载场景文件）。
            视口、属性面板、工件下拉各持一份场景引用，替换必须让主窗口
            统一接管（_adopt_scene），否则各处引用错开，选中对象在属性面板
            里查不到（面板整个收起，看似"内容弄丢了"）
    """

    sceneChanged = QtCore.Signal()
    sceneReplaced = QtCore.Signal(object)
    statusMessage = QtCore.Signal(str)
    collisionResult = QtCore.Signal(int)
    objectSelected = QtCore.Signal(str)  # 当前树选中对象名（None 表示清除）
    robotRemoved = QtCore.Signal(str)

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
        # 内容自然高度约 700px，而标签页可视高度随窗口变化（小窗口时更矮）。
        # 不套滚动容器会把底部的分组挤出可视区——与右侧属性面板同源的问题。
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameStyle(QtWidgets.QFrame.NoFrame)
        inner = QtWidgets.QWidget()
        scroll.setWidget(inner)
        outer.addWidget(scroll)

        root = QtWidgets.QVBoxLayout(inner)
        root.setContentsMargins(5, 5, 5, 5)
        root.setSpacing(5)

        # ---- 场景树 ----
        tree_group = QtWidgets.QGroupBox(tr("场景对象"))
        tree_layout = QtWidgets.QVBoxLayout(tree_group)

        # 设备树：拖拽节点即可改写挂载层级（挂到机器人＝安装到法兰）
        self.tree = _DeviceTree()
        self.tree.setHeaderLabels([tr("对象"), tr("类型")])
        self.tree.setRootIsDecorated(True)
        self.tree.setMaximumHeight(320)
        self.tree.setToolTip(
            tr("拖拽节点改写挂载层级：挂到机器人＝安装到法兰，自动对齐坐标系")
        )
        self.tree.currentItemChanged.connect(self._on_tree_selected)
        self.tree.itemReparented.connect(self._on_item_reparented)
        tree_layout.addWidget(self.tree)

        btn_row = QtWidgets.QHBoxLayout()
        self.remove_btn = QtWidgets.QPushButton(tr("删除"))
        self.remove_btn.clicked.connect(self._remove_selected)
        btn_row.addWidget(self.remove_btn)
        self.clear_btn = QtWidgets.QPushButton(tr("清空"))
        self.clear_btn.clicked.connect(self._clear_scene)
        btn_row.addWidget(self.clear_btn)
        tree_layout.addLayout(btn_row)
        root.addWidget(tree_group)

        # ---- 碰撞检查 ----
        check_group = QtWidgets.QGroupBox(tr("干涉检查"))
        check_layout = QtWidgets.QVBoxLayout(check_group)
        self.check_btn = QtWidgets.QPushButton(tr("检查干涉 (F6)"))
        self.check_btn.clicked.connect(self.run_collision_check)
        check_layout.addWidget(self.check_btn)
        self.collision_label = QtWidgets.QLabel(tr("未检查"))
        self.collision_label.setWordWrap(True)
        check_layout.addWidget(self.collision_label)
        root.addWidget(check_group)

        # ---- 场景文件 ----
        file_row = QtWidgets.QHBoxLayout()
        self.save_btn = QtWidgets.QPushButton(tr("保存场景…"))
        self.save_btn.clicked.connect(self.save_scene)
        self.load_btn = QtWidgets.QPushButton(tr("加载场景…"))
        self.load_btn.clicked.connect(self.load_scene)
        file_row.addWidget(self.save_btn)
        file_row.addWidget(self.load_btn)
        root.addLayout(file_row)

        self.stats_label = QtWidgets.QLabel(tr("0 个对象"))
        self.stats_label.setStyleSheet("color:#7f8192; font-size:11px;")
        root.addWidget(self.stats_label)

        # ---- 工艺时序（动态时序：工序触发 / 信号等待 / 子程序调用，v1.3.1） ---- #
        # v1.4.0 MVP：泳道图编辑器还在迭代（队列 P5），MVP 档整组不构建。
        # 隐藏的是**入口**而不是数据——工程里的时序段依旧保真载入
        # （见 ``app.MainWindow._apply_project`` 的"跳过"分支）。
        if _feature_enabled("sequence_editor"):
            swim_group = QtWidgets.QGroupBox(tr("工艺时序（泳道图）"))
            swim_layout = QtWidgets.QVBoxLayout(swim_group)
            self.sequence_editor = SequenceEditor()
            self.sequence_editor.statusMessage.connect(self.statusMessage)
            self.sequence_editor.sequenceChanged.connect(self.sceneChanged)
            swim_layout.addWidget(self.sequence_editor, 1)
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
            self.statusMessage.emit(tr("添加失败: {exc}").format(exc=exc))
            return None
        self._refresh_tree()
        self.sceneChanged.emit()
        if select:
            self._select_object(obj.name)
        self.statusMessage.emit(tr("已添加 {name}").format(name=obj.name))
        return obj

    # --------------------------- 场景树 --------------------------- #

    def _refresh_tree(self) -> None:
        # 编辑参数时 objectChanged 会触发本函数；clear() 会让当前选中项变 None，
        # 进而误发 objectSelected(None) 把右侧属性面板关掉。先记住选中项并屏蔽信号。
        selected = self._selected
        self.tree.blockSignals(True)
        try:
            self.tree.clear()
            items: Dict[str, QtWidgets.QTreeWidgetItem] = {}

            # 先建所有节点
            for name, obj in self.scene.objects.items():
                # 类型保持与「场景设备库」一致：机器人 → 机器人；组件 → 它的
                # **分类**（基础设施 / 工件 / 夹具 / 末端工具…），而不是组件名
                # （组件名是"两指夹爪"这种具体型号，留给「名称」列）。
                # 判定必须用 is_robot：多机时第二台叫「机器人_2」，按名字比会
                # 落到"自定义"。
                if self.scene.is_robot(name):
                    kind = tr("机器人")
                else:
                    spec = COMPONENTS.get(obj.component_id) if obj.component_id else None
                    kind = tr(spec.category) if spec else tr("自定义")
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
        finally:
            self.tree.blockSignals(False)

        # 恢复选中项（若对象仍存在）。恢复时也屏蔽信号，避免重复触发当前项变更。
        if selected and selected in self.scene.objects:
            self.tree.blockSignals(True)
            self._select_object(selected)
            self.tree.blockSignals(False)
        else:
            self._selected = None

        self.stats_label.setText(tr("{n} 个对象").format(n=len(self.scene)))
        if hasattr(self, "sequence_editor"):
            self.sequence_editor.set_scene(self.scene)

    def _on_item_reparented(self, name: str, new_parent) -> None:
        """拖拽改写挂载层级。

        挂到**机器人**＝工具安装语义：局部位姿归零，工具吸附到法兰原点并与
        法兰坐标系对齐；挂到其它对象／场景根＝保持当前世界位姿建立随动。
        自挂、成环、未知父对象由 ``Scene.attach`` 抛出 ValueError 拦下。
        """
        if self.scene is None or name not in self.scene.objects:
            return
        # 机器人可以挂到底座/台面上（坐到上表面中心）；但**整机挂到另一台
        # 机器人的法兰上**没有意义——那只能装工具，不能装机器人
        if (self.scene.is_robot(name) and new_parent
                and new_parent in self.scene.robots):
            self.statusMessage.emit(
                tr("{name} 是机器人，不能挂到 {host} 的法兰上").format(
                    name=name, host=new_parent
                )
            )
            return
        try:
            mounted_on_robot = new_parent in self.scene.robots
            self.scene.attach(name, new_parent, keep_world=not mounted_on_robot)
            # 机器人挂到底座/台面 → 坐到上表面中心（不是停在原地只建随动）
            if (self.scene.is_robot(name) and new_parent
                    and not self.scene.is_robot(new_parent)):
                _mount_on_top_of(self.scene, name, new_parent)
        except (KeyError, ValueError) as exc:
            self.statusMessage.emit(tr("挂载失败: {exc}").format(exc=exc))
            return
        self._refresh_tree()
        self.sceneChanged.emit()
        host = new_parent or tr("场景根")
        self.statusMessage.emit(
            tr("已将 {name} 挂到 {host}").format(name=name, host=host)
        )

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
        name = self._selected
        # 「是不是机器人」必须在 remove **之前**取：remove 会把它从
        # ``scene.robots`` 里摘掉，之后再问就永远是 False 了
        was_robot = self.scene.is_robot(name)
        self.scene.remove(name)
        self._selected = None
        self._refresh_tree()
        self.sceneChanged.emit()
        self.objectSelected.emit(None)
        self.statusMessage.emit(tr("已删除对象"))
        if was_robot:
            self.robotRemoved.emit(name)

    def _clear_scene(self) -> None:
        ret = QtWidgets.QMessageBox.question(
            self, tr("清空场景"), tr("确定要移除所有场景对象吗？（机器人不受影响）")
        )
        if ret != QtWidgets.QMessageBox.Yes:
            return
        # ``clear()`` 会把机器人一并摘掉（对象壳保留，但不再算机器人）——
        # 名字要先记下来，否则清空后就无从通知"谁的程序该清掉了"
        gone = list(self.scene.robots)
        self.scene.clear()
        self._selected = None
        self._refresh_tree()
        self.sceneChanged.emit()
        self.objectSelected.emit(None)
        self.statusMessage.emit(tr("场景已清空"))
        for name in gone:
            self.robotRemoved.emit(name)

    # --------------------------- 碰撞检查 --------------------------- #

    def run_collision_check(self) -> list:
        """执行干涉检查、更新显示，并返回干涉对列表。

        Returns:
            :class:`~orolp.world.scene.Collision` 列表；空列表表示无干涉。
            返回值（而非仅内部显示）是为了让视口复用同一次结果做高亮——
            否则同一帧要把 SAT 算两遍。
        """
        if not self.scene.has_robot and len(self.scene) == 0:
            self.collision_label.setText(tr("场景为空"))
            self.collision_label.setStyleSheet("color:#9a9bab;")
            self.collisionResult.emit(0)
            return []
        try:
            hits = self.scene.check_collisions(margin=-0.5)
        except Exception as exc:
            self.collision_label.setText(tr("检查失败: {exc}").format(exc=exc))
            self.collision_label.setStyleSheet("color:#e0603a;")
            self.collisionResult.emit(0)
            return []

        if not hits:
            self.collision_label.setText(tr("✓ 无干涉"))
            self.collision_label.setStyleSheet("color:#4ec97a;")
            self.statusMessage.emit(tr("干涉检查通过：无干涉"))
        else:
            shown = tr("、").join(str(h) for h in hits[:6])
            more = tr(" …等共 {n} 处").format(n=len(hits)) if len(hits) > 6 else ""
            self.collision_label.setText(tr("✗ {n} 处干涉：{shown}{more}").format(n=len(hits), shown=shown, more=more))
            self.collision_label.setStyleSheet("color:#e0603a;")
            self.statusMessage.emit(tr("发现 {n} 处干涉").format(n=len(hits)))
        self.collisionResult.emit(len(hits))
        return hits

    # --------------------------- 场景文件 --------------------------- #

    def save_scene(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, tr("保存场景"), "workcell.json", tr("场景文件 (*.json)")
        )
        if not path:
            return
        try:
            self.scene.save(path)
        except OSError as exc:
            QtWidgets.QMessageBox.critical(self, tr("保存失败"), str(exc))
            return
        self.statusMessage.emit(tr("场景已保存: {name}").format(name=Path(path).name))

    def load_scene(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, tr("加载场景"), "", tr("场景文件 (*.json)")
        )
        if not path:
            return
        try:
            new_scene = Scene.load(path)
        except (OSError, ValueError) as exc:
            QtWidgets.QMessageBox.critical(self, tr("加载失败"), str(exc))
            return
        # 换引用要走统一入口（内部换引用 + 刷新树），再把"替换"广播给主窗口——
        # 视口 / 属性面板 / 代码编辑器的场景引用都靠它一起换掉
        self.set_scene(new_scene)
        self.sceneReplaced.emit(new_scene)
        self.sceneChanged.emit()
        self.statusMessage.emit(tr("场景已加载: {name}（{n} 个对象）").format(name=Path(path).name, n=len(self.scene)))

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
            if hasattr(self, "sequence_editor"):
                self.sequence_editor.set_scene(self.scene)
            self._refresh_tree()
            self.sceneChanged.emit()
        except Exception as exc:
            self.statusMessage.emit(tr("机器人加载失败: {exc}").format(exc=exc))

    def add_robot(
        self,
        urdf_path: str,
        joints_deg: Optional[List[float]] = None,
        name: Optional[str] = None,
        pose=None,
    ) -> str:
        """**追加**一台机器人（不覆盖已有的），返回其名字。

        与 :meth:`set_robot` 的区别同 ``Scene.add_robot``：场景里已有机器人时
        必须用它——``set_robot`` 的名字固定为 ``ROBOT_NAME``，会把已有那台顶掉，
        表现为"再增加一台却变成替换"。

        Returns:
            实际使用的机器人名；失败返回空串
        """
        try:
            robot = self.scene.add_robot(urdf_path, joints_deg, name=name, pose=pose)
        except Exception as exc:
            self.statusMessage.emit(tr("机器人加载失败: {exc}").format(exc=exc))
            return ""
        if hasattr(self, "sequence_editor"):
            self.sequence_editor.set_scene(self.scene)
        self._refresh_tree()
        self.sceneChanged.emit()
        return robot

    def update_robot_joints(
        self, joints_deg: List[float], name: Optional[str] = None
    ) -> None:
        """同步机器人关节角（供视口拖动时调用）；``name`` 指定哪一台。

        多机必需：``Scene.set_robot_joints`` 的 ``name=None`` 只作用于**主机器
        人**，不指名会把"拖 A 的关节"写到 B 身上。
        """
        if self.scene.has_robot:
            self.scene.set_robot_joints(joints_deg, name=name)


#: 泳道块配色（kind → 填充色），与面板深色底呼应
_KIND_COLORS = {
    "call": "#7a3fa0",
    "signal": "#2f6feb",
    "wait": "#b26a00",
    "dwell": "#4a4d5c",
}


class SequenceCanvas(QtWidgets.QWidget):
    """泳道画布：行＝泳道（设备），块＝工步。

    诚实口径：横轴是**步骤序号**，不是实测时间——触发与等待的真实耗时
    取决于现场机构，只有驻留（秒）可计算，因此驻留块按秒比例加宽、其余
    等宽，页头写明"非实测节拍"。
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._lanes: List[str] = []
        self._blocks: List[Dict[str, object]] = []
        self.setMinimumHeight(96)
        self.setStyleSheet("background: #20212a; border: 1px solid #34353f;")

    def set_data(self, lanes: List[str], blocks: List[Dict[str, object]]) -> None:
        self._lanes = list(lanes)
        self._blocks = list(blocks)
        self.updateGeometry()
        self.update()

    def sizeHint(self):  # noqa: N802 - Qt 命名
        lane_h = 26
        h = max(96, 24 + len(self._lanes) * lane_h + 6)
        return QtCore.QSize(320, h)

    def paintEvent(self, event) -> None:  # noqa: N802 - Qt 命名
        if self._lanes:
            return self._paint_lanes(event)
        painter = QtGui.QPainter(self)
        painter.setPen(QtGui.QPen(QtGui.QColor("#7f8192")))
        painter.drawText(self.rect(), QtCore.Qt.AlignCenter,
                         tr("还没有工步——点「添加工步」把工序排进泳道"))

    def _paint_lanes(self, event) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        lane_h = 26
        lane_w = min(96, max(48, int(self.width() * 0.22)))
        top = 2
        base_w = 56.0

        # 泳道行
        painter.setFont(QtGui.QFont(self.font().family(), 8))
        for r, lane in enumerate(self._lanes):
            y = top + r * lane_h
            painter.setPen(QtGui.QPen(QtGui.QColor("#34353f")))
            painter.drawLine(0, y + lane_h - 1, self.width(), y + lane_h - 1)
            painter.setPen(QtGui.QPen(QtGui.QColor("#9a9bab")))
            painter.drawText(
                QtCore.QRect(2, y, lane_w - 6, lane_h - 2),
                QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft,
                lane,
            )

        # 工步块：横轴＝步骤序号（驻留按秒加宽）
        x = float(lane_w)
        for blk in self._blocks:
            units = float(blk.get("width_units", 1.0))
            w = base_w * max(1.0, units)
            try:
                y = top + self._lanes.index(str(blk.get("lane"))) * lane_h
            except ValueError:
                continue
            rect = QtCore.QRectF(x + 2, y + 2, w - 4, lane_h - 5)
            painter.setPen(QtGui.QPen(QtGui.QColor("#14151b")))
            painter.setBrush(QtGui.QBrush(QtGui.QColor(_KIND_COLORS.get(str(blk.get("kind")), "#4a4d5c"))))
            painter.drawRoundedRect(rect, 3, 3)
            painter.setPen(QtGui.QPen(QtGui.QColor("#e6e8f0")))
            painter.setFont(QtGui.QFont(self.font().family(), 7))
            painter.drawText(rect, QtCore.Qt.AlignCenter, str(blk.get("label", "")))
            x += w

        # 页头：如实说明横轴口径
        painter.setPen(QtGui.QPen(QtGui.QColor("#7f8192")))
        painter.drawText(
            QtCore.QRect(lane_w, 0, self.width() - lane_w, 14),
            QtCore.Qt.AlignLeft,
            tr("步骤序号 →（驻留按秒加宽；非实测节拍）"),
        )


class _StepDialog(QtWidgets.QDialog):
    """添加工步对话框：按类型给出对应字段。"""

    def __init__(self, kind: str, devices: List[str], signals: List[str],
                 subprograms: List[str], parent=None):
        super().__init__(parent)
        self.kind = kind
        titles = {
            "call": tr("添加工步：子程序调用"),
            "signal": tr("添加工步：触发信号"),
            "wait": tr("添加工步：等待信号"),
            "dwell": tr("添加工步：驻留"),
        }
        self.setWindowTitle(titles.get(kind, tr("添加工步")))
        form = QtWidgets.QFormLayout(self)

        self.name_edit = QtWidgets.QLineEdit()
        form.addRow(tr("工步名:"), self.name_edit)

        self.device_combo = QtWidgets.QComboBox()
        self.device_combo.addItems(devices or [tr("机器人")])
        form.addRow(tr("泳道（设备）:"), self.device_combo)

        self.subprogram_combo: Optional[QtWidgets.QComboBox] = None
        self.signal_combo: Optional[QtWidgets.QComboBox] = None
        self.value_check: Optional[QtWidgets.QCheckBox] = None
        self.trigger_combo: Optional[QtWidgets.QComboBox] = None
        self.timeout_spin: Optional[QtWidgets.QDoubleSpinBox] = None
        self.seconds_spin: Optional[QtWidgets.QDoubleSpinBox] = None

        if kind == "call":
            self.subprogram_combo = QtWidgets.QComboBox()
            self.subprogram_combo.setEditable(True)
            self.subprogram_combo.addItems(subprograms)
            form.addRow(tr("子程序名:"), self.subprogram_combo)
        elif kind in ("signal", "wait"):
            self.signal_combo = QtWidgets.QComboBox()
            self.signal_combo.setEditable(True)
            self.signal_combo.addItems(signals)
            form.addRow(tr("信号:"), self.signal_combo)
            if kind == "signal":
                self.value_check = QtWidgets.QCheckBox(tr("置位（勾选＝True，不勾＝False）"))
                self.value_check.setChecked(True)
                form.addRow(tr("动作:"), self.value_check)
            else:
                self.trigger_combo = QtWidgets.QComboBox()
                for code, label in (
                    ("level", tr("电平")),
                    ("rising", tr("上升沿")),
                    ("falling", tr("下降沿")),
                ):
                    self.trigger_combo.addItem(label, code)
                form.addRow(tr("触发方式:"), self.trigger_combo)
                self.value_check = QtWidgets.QCheckBox(tr("期望电平为 True"))
                self.value_check.setChecked(True)
                form.addRow(tr("期望电平:"), self.value_check)
                self.timeout_spin = QtWidgets.QDoubleSpinBox()
                self.timeout_spin.setRange(0.0, 3600.0)
                self.timeout_spin.setDecimals(1)
                self.timeout_spin.setSingleStep(0.5)
                self.timeout_spin.setValue(0.0)
                self.timeout_spin.setToolTip(tr("0＝无限等待。现场必须配看门狗，否则一步卡死整条线"))
                form.addRow(tr("超时 (s):"), self.timeout_spin)
        else:  # dwell
            self.seconds_spin = QtWidgets.QDoubleSpinBox()
            self.seconds_spin.setRange(0.0, 3600.0)
            self.seconds_spin.setDecimals(2)
            self.seconds_spin.setSingleStep(0.1)
            self.seconds_spin.setValue(0.5)
            form.addRow(tr("驻留 (s):"), self.seconds_spin)

        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)

    def values(self) -> Dict[str, object]:
        out: Dict[str, object] = {
            "name": self.name_edit.text().strip(),
            "device": self.device_combo.currentText(),
        }
        if self.kind == "call":
            out["subprogram"] = self.subprogram_combo.currentText().strip()
        elif self.kind == "signal":
            out["signal"] = self.signal_combo.currentText().strip()
            out["value"] = self.value_check.isChecked()
        elif self.kind == "wait":
            out["signal"] = self.signal_combo.currentText().strip()
            out["trigger"] = self.trigger_combo.currentData()
            out["value"] = self.value_check.isChecked()
            out["timeout"] = float(self.timeout_spin.value())
        else:
            out["seconds"] = float(self.seconds_spin.value())
        return out


class SequenceEditor(QtWidgets.QWidget):
    """工艺时序编辑器：步骤表 + 泳道画布 + 校验 + JSON 往返。

    信号/子程序名由外部注入（:meth:`set_available_signals` /
    :meth:`set_known_subprograms`）——面板不知道点表与子程序清单的存在，
    校验强度随注入而增强，不给也能用（只做结构校验）。
    """

    sequenceChanged = QtCore.Signal()
    statusMessage = QtCore.Signal(str)

    _COLS = (tr("序号"), tr("泳道"), tr("类型"), tr("内容"))

    def __init__(self, parent=None):
        super().__init__(parent)
        self.sequence = ProcessSequence()
        self._devices: List[str] = []
        self._signals: List[str] = list(HANDSHAKE_SIGNALS)
        self._subprograms: List[str] = []
        self._name_seq = 0
        self._setup_ui()

    # ---------------- 界面 ---------------- #

    def _setup_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        btn_row = QtWidgets.QHBoxLayout()
        self.add_btn = QtWidgets.QToolButton()
        self.add_btn.setText(tr("添加工步"))
        self.add_btn.setPopupMode(QtWidgets.QToolButton.InstantPopup)
        menu = QtWidgets.QMenu(self.add_btn)
        for kind, label in (
            ("call", tr("子程序调用")),
            ("signal", tr("触发信号")),
            ("wait", tr("等待信号")),
            ("dwell", tr("驻留")),
        ):
            menu.addAction(label, lambda k=kind: self._add_via_dialog(k))
        self.add_btn.setMenu(menu)
        btn_row.addWidget(self.add_btn)

        self.remove_btn = QtWidgets.QToolButton()
        self.remove_btn.setText(tr("删除"))
        self.remove_btn.clicked.connect(self.remove_current)
        btn_row.addWidget(self.remove_btn)

        self.up_btn = QtWidgets.QToolButton()
        self.up_btn.setText(tr("上移"))
        self.up_btn.clicked.connect(lambda: self.move_current(-1))
        btn_row.addWidget(self.up_btn)

        self.down_btn = QtWidgets.QToolButton()
        self.down_btn.setText(tr("下移"))
        self.down_btn.clicked.connect(lambda: self.move_current(1))
        btn_row.addWidget(self.down_btn)

        self.validate_btn = QtWidgets.QToolButton()
        self.validate_btn.setText(tr("校验时序"))
        self.validate_btn.clicked.connect(self.validate_now)
        btn_row.addWidget(self.validate_btn)

        self.save_btn = QtWidgets.QToolButton()
        self.save_btn.setText(tr("保存时序…"))
        self.save_btn.clicked.connect(self.save_dialog)
        btn_row.addWidget(self.save_btn)

        self.load_btn = QtWidgets.QToolButton()
        self.load_btn.setText(tr("载入时序…"))
        self.load_btn.clicked.connect(self.load_dialog)
        btn_row.addWidget(self.load_btn)
        btn_row.addStretch(1)
        layout.addLayout(btn_row)

        self.table = QtWidgets.QTableWidget(0, len(self._COLS))
        self.table.setHorizontalHeaderLabels(list(self._COLS))
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(
            3, QtWidgets.QHeaderView.Stretch
        )
        self.table.setMaximumHeight(150)
        layout.addWidget(self.table)

        self.canvas = SequenceCanvas()
        layout.addWidget(self.canvas, 1)

        self.validate_label = QtWidgets.QLabel(tr("未校验"))
        self.validate_label.setWordWrap(True)
        layout.addWidget(self.validate_label)

    # ---------------- 数据入口（程序化 API，测试与外部调用共用） ---------------- #

    def set_scene(self, scene) -> None:
        """泳道随场景对象走；已排好的工步不动。"""
        self._devices = list(scene.objects.keys()) if scene is not None else []
        if getattr(scene, "has_robot", False) and "机器人" not in self._devices:
            pass  # 场景对象里本身就有"机器人"
        self._refresh()

    def set_available_signals(self, signals: Sequence[str]) -> None:
        """注入点表里真实存在的信号名（追加到握手信号之后）。"""
        for sig in signals:
            if sig not in self._signals:
                self._signals.append(str(sig))

    def available_signals(self) -> List[str]:
        return list(self._signals)

    def set_known_subprograms(self, names: Sequence[str]) -> None:
        self._subprograms = [str(n) for n in names]

    def add_step(self, kind: str, **params: object) -> int:
        """按类型添加一个工步，返回其下标。参数缺省值在这里统一给。"""
        self._name_seq += 1
        default_name = tr("工步 {n}").format(n=self._name_seq)
        device = str(params.get("device") or (self._devices[0] if self._devices else tr("机器人")))
        if kind == "call":
            step = CallStep(
                name=str(params.get("name") or default_name),
                device=device,
                subprogram=str(params.get("subprogram") or ""),
            )
        elif kind == "signal":
            step = SignalStep(
                name=str(params.get("name") or default_name),
                device=device,
                signal=str(params.get("signal") or ""),
                value=bool(params.get("value", True)),
            )
        elif kind == "wait":
            step = WaitStep(
                name=str(params.get("name") or default_name),
                device=device,
                signal=str(params.get("signal") or ""),
                trigger=str(params.get("trigger") or "level"),
                value=bool(params.get("value", True)),
                timeout=float(params.get("timeout") or 0.0),
            )
        elif kind == "dwell":
            step = DwellStep(
                name=str(params.get("name") or default_name),
                device=device,
                seconds=float(params.get("seconds") or 0.0),
            )
        else:
            raise ValueError(tr("未知工步类型：{kind}").format(kind=kind))
        self.sequence.add(step)
        self._refresh()
        self.sequenceChanged.emit()
        return len(self.sequence) - 1

    def remove_step(self, index: int) -> None:
        self.sequence.remove(index)
        self._refresh()
        self.sequenceChanged.emit()

    def move_step(self, index: int, delta: int) -> bool:
        moved = self.sequence.move(index, delta)
        if moved:
            self._refresh()
            self.sequenceChanged.emit()
        return moved

    # ---------------- 交互 ---------------- #

    def _current_index(self) -> Optional[int]:
        row = self.table.currentRow()
        return row if row >= 0 else None

    def remove_current(self) -> None:
        idx = self._current_index()
        if idx is None:
            self.statusMessage.emit(tr("先在步骤表里选中一行"))
            return
        self.remove_step(idx)

    def move_current(self, delta: int) -> None:
        idx = self._current_index()
        if idx is None:
            self.statusMessage.emit(tr("先在步骤表里选中一行"))
            return
        if not self.move_step(idx, delta):
            self.statusMessage.emit(tr("已经到头了"))

    def _add_via_dialog(self, kind: str) -> None:
        dialog = _StepDialog(kind, self._devices, self._signals, self._subprograms, self)
        if dialog.exec() != QtWidgets.QDialog.Accepted:
            return
        values = dialog.values()
        if kind in ("signal", "wait") and not values.get("signal"):
            self.statusMessage.emit(tr("信号名不能为空"))
            return
        if kind == "call" and not values.get("subprogram"):
            self.statusMessage.emit(tr("子程序名不能为空"))
            return
        self.add_step(kind, **values)
        self.statusMessage.emit(tr("已添加工步 {name}").format(name=values.get("name")))

    # ---------------- 校验 / 保存 / 载入 ---------------- #

    def validate_now(self) -> List[SeqIssue]:
        issues = self.sequence.validate(
            signal_names=self._signals,
            subprograms=self._subprograms or None,
            devices=self._devices or None,
        )
        errors = [i for i in issues if i.severity == SEVERITY_ERROR]
        warnings = [i for i in issues if i.severity == SEVERITY_WARNING]
        if not issues:
            self.validate_label.setText(tr("校验通过：{n} 个工步，驻留合计 {s}s").format(
                n=len(self.sequence), s=self.sequence.dwell_total()
            ))
            self.validate_label.setStyleSheet("color:#7fbf7f;")
        else:
            mark = "✗" if errors else "⚠"
            head = tr("{mark} {e} 处错误 / {w} 处警告").format(
                mark=mark, e=len(errors), w=len(warnings)
            )
            detail = "；".join(i.message for i in issues[:3])
            more = tr("（其余 {n} 条见日志）").format(n=len(issues) - 3) if len(issues) > 3 else ""
            self.validate_label.setText(f"{head}：{detail}{more}")
            self.validate_label.setStyleSheet("color:#d8a657;" if not errors else "color:#e06c75;")
            for i in issues:
                self.statusMessage.emit(i.message)
        return issues

    def summary_text(self) -> str:
        s = self.sequence.summary()
        return tr("{n} 个工步 ｜ 触发 {signal} ｜ 等待 {wait} ｜ 调用 {call} ｜ 驻留合计 {dwell}s").format(
            n=s["steps"], signal=s["counts"]["signal"], wait=s["counts"]["wait"],
            call=s["counts"]["call"], dwell=s["dwell_total_s"],
        )

    def save_dialog(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, tr("保存工艺时序"), "process_sequence.json", "JSON (*.json)"
        )
        if not path:
            return
        self.save_to(path)

    def save_to(self, path) -> None:
        self.sequence.save_json(path)
        self.statusMessage.emit(tr("时序已保存：{path}").format(path=path))

    def load_dialog(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, tr("载入工艺时序"), "", "JSON (*.json)"
        )
        if not path:
            return
        self.load_from(path)

    def load_from(self, path) -> None:
        try:
            self.sequence = ProcessSequence.load_json(path)
        except (ValueError, OSError) as exc:
            self.statusMessage.emit(tr("载入失败: {exc}").format(exc=exc))
            return
        self._refresh()
        self.sequenceChanged.emit()
        self.statusMessage.emit(self.summary_text())

    def set_sequence(self, sequence: "ProcessSequence") -> None:
        """整体替换时序（工程文件载入用 v1.3.4）。

        只做替换 + 刷新 + 发信号，**不校验**：调用方（工程载入）自己决定
        要不要在之后跑 ``validate_now()``，避免这里悄悄改动别人的判定口径。
        """
        self.sequence = sequence if sequence is not None else ProcessSequence()
        self._name_seq = max(self._name_seq, len(self.sequence))
        self._refresh()
        self.sequenceChanged.emit()

    # ---------------- 刷新 ---------------- #

    def _refresh(self) -> None:
        kind_labels = {"call": tr("调用"), "signal": tr("触发"),
                       "wait": tr("等待"), "dwell": tr("驻留")}
        self.table.setRowCount(len(self.sequence))
        for r, st in enumerate(self.sequence):
            kind = _step_kind(st)
            cells = (
                str(r + 1),
                st.device,
                kind_labels[kind],
                _step_content(st),
            )
            for c, text in enumerate(cells):
                item = QtWidgets.QTableWidgetItem(text)
                item.setData(QtCore.Qt.UserRole, r)
                self.table.setItem(r, c, item)

        lanes = self._devices or self.sequence.lanes() or [tr("机器人")]
        blocks = []
        x_units = 0.0
        for st in self.sequence:
            kind = _step_kind(st)
            units = 1.0 + (float(st.seconds) if kind == "dwell" else 0.0)
            blocks.append({
                "lane": st.device if st.device in lanes else lanes[0],
                "label": st.name,
                "kind": kind,
                "width_units": units,
                "x_units": x_units,
            })
            x_units += units
        self.canvas.set_data(lanes, blocks)


def _step_kind(step) -> str:
    from ..sequence import _step_kind as _k

    return _k(step)


def _step_content(step) -> str:
    from ..sequence import CallStep, DwellStep, SignalStep, WaitStep

    if isinstance(step, CallStep):
        return tr("调用 {sub}").format(sub=step.subprogram)
    if isinstance(step, SignalStep):
        return tr("{signal} = {value}").format(
            signal=step.signal, value="True" if step.value else "False"
        )
    if isinstance(step, WaitStep):
        if step.trigger == "level":
            return tr("等 {signal} = {value}").format(
                signal=step.signal, value="True" if step.value else "False"
            )
        return tr("等 {signal}（{trg}）").format(
            signal=step.signal,
            trg={"rising": tr("上升沿"), "falling": tr("下降沿")}.get(step.trigger, step.trigger),
        )
    return tr("驻留 {s}s").format(s=step.seconds)
