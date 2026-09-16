"""机器人库面板 (Robot library panel)。

对标 Visual Components / RoboDK 的 eCatalog：一个可停靠的组件库侧栏，
支持浏览品牌、搜索型号、按负载/臂展/轴数筛选，并一键把机器人载入 3D 视口。

面板只负责tr("选")，不负责tr("载") —— 选中后通过 :attr:`robotSelected` 信号把
URDF 路径抛给主窗口，由主窗口完成加载与联动，保持职责单一。

用法::

    panel = RobotLibraryPanel(models_dir=Path("models"))
    panel.robotSelected.connect(main_window.load_robot)
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Optional

from PySide6.QtCore import QByteArray, QMimeData, QSize, Qt, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPixmap
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from .catalog import CatalogEntry, RobotCatalog
from ..i18n import tr

try:  # 组件库（环境建模）缺失时不阻塞机器人库本身
    from orolp.world.components import COMPONENTS, categories as _component_categories

    _HAS_COMPONENTS = True
except Exception:  # pragma: no cover - 仅缺失世界建模时降级
    COMPONENTS = None
    _component_categories = lambda: []
    _HAS_COMPONENTS = False

__all__ = ["RobotLibraryPanel"]

#: 分类组合框虚拟项
_CAT_ALL = "__all__"
_CAT_ROBOT = "__robot__"
_CAT_COMPONENT = "__component__"

#: 拖拽 MIME 类型，视口据此识别落点动作
MIME_TYPE = "application/x-machine-item"

# 与 3D 视口背景 (0.15, 0.15, 0.18) 呼应的深色侧栏样式
PANEL_STYLESHEET = """
RobotLibraryPanel, RobotLibraryPanel QWidget {
    background-color: #26262e;
    color: #d8d8e0;
}
RobotLibraryPanel QLineEdit, RobotLibraryPanel QComboBox {
    background-color: #1c1c22;
    border: 1px solid #3a3a46;
    border-radius: 4px;
    padding: 4px 6px;
    color: #e6e6ee;
    selection-background-color: #ff6a13;
}
RobotLibraryPanel QLineEdit:focus, RobotLibraryPanel QComboBox:focus {
    border: 1px solid #ff6a13;
}
RobotLibraryPanel QComboBox::drop-down { border: none; width: 16px; }
RobotLibraryPanel QComboBox QAbstractItemView {
    background-color: #1c1c22;
    border: 1px solid #3a3a46;
    color: #e6e6ee;
    selection-background-color: #3a3a46;
}
RobotLibraryPanel QListWidget {
    background-color: #202028;
    border: 1px solid #34343f;
    border-radius: 4px;
    outline: none;
}
RobotLibraryPanel QListWidget::item { border-bottom: 1px solid #2c2c36; }
RobotLibraryPanel QListWidget::item:selected { background-color: #34343f; }
RobotLibraryPanel QListWidget::item:hover { background-color: #2c2c36; }
RobotLibraryPanel QPushButton {
    background-color: #33333d;
    border: 1px solid #45455280;
    border-radius: 4px;
    padding: 4px 10px;
    color: #d8d8e0;
}
RobotLibraryPanel QPushButton:hover { background-color: #3f3f4c; }
RobotLibraryPanel QPushButton:pressed { background-color: #4a4a58; }
"""

# 负载筛选区间（kg）：(显示文本, min, max)
PAYLOAD_FILTERS = [
    (tr("全部负载"), None, None),
    ("≤ 5 kg", None, 5.0),
    ("5 ~ 20 kg", 5.0, 20.0),
    ("20 ~ 60 kg", 20.0, 60.0),
    ("> 60 kg", 60.0, None),
]

# 臂展筛选区间（mm）
REACH_FILTERS = [
    (tr("全部臂展"), None, None),
    ("< 1000 mm", None, 1000.0),
    ("1000 ~ 2000 mm", 1000.0, 2000.0),
    ("> 2000 mm", 2000.0, None),
]


def _brand_chip(color: tuple, text: str, size: int = 28) -> QPixmap:
    """生成品牌色圆角小图标，文字取型号首字母缩写。"""
    pm = QPixmap(size, size)
    pm.fill(Qt.transparent)
    painter = QPainter(pm)
    painter.setRenderHint(QPainter.Antialiasing)
    r, g, b = (int(round(c * 255)) for c in color)
    painter.setBrush(QColor(r, g, b))
    painter.setPen(Qt.NoPen)
    painter.drawRoundedRect(0, 0, size, size, 6, 6)
    painter.setPen(QColor(20, 20, 26) if (r + g + b) > 380 else QColor(235, 235, 245))
    font = QFont()
    font.setPointSize(9)
    font.setBold(True)
    painter.setFont(font)
    painter.drawText(pm.rect(), Qt.AlignCenter, text)
    painter.end()
    return pm


class _RobotCard(QFrame):
    """列表中单个型号卡片。"""

    def __init__(self, entry: CatalogEntry):
        super().__init__()
        self.entry = entry
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(8)

        # 品牌色图标
        icon = QLabel()
        icon.setPixmap(_brand_chip(entry.color, self._initials(entry.model_name)))
        icon.setFixedSize(28, 28)
        layout.addWidget(icon)

        # 文本区
        text_layout = QVBoxLayout()
        text_layout.setSpacing(1)
        text_layout.setContentsMargins(0, 0, 0, 0)

        name_row = QHBoxLayout()
        name_row.setSpacing(6)
        name_label = QLabel(entry.model_name)
        name_font = QFont()
        name_font.setBold(True)
        name_font.setPointSize(9)
        name_label.setFont(name_font)
        name_label.setStyleSheet("color: #f0f0f6;")
        name_row.addWidget(name_label)

        if not entry.verified:
            badge = QLabel(tr("参数未核验"))
            badge.setStyleSheet(
                "color: #d0a03a; background-color: #3a2f16; border-radius: 3px; padding: 0 4px;"
            )
            badge_font = QFont()
            badge_font.setPointSize(7)
            badge.setFont(badge_font)
            name_row.addWidget(badge)
        name_row.addStretch()
        text_layout.addLayout(name_row)

        # 规格摘要行
        specs = []
        specs.append(tr("{n} 轴").format(n=entry.dof))
        if entry.payload_kg is not None:
            specs.append(f"{entry.payload_kg:g} kg")
        if entry.reach_mm is not None:
            specs.append(f"{entry.reach_mm:g} mm")
        if entry.repeatability_mm is not None:
            specs.append(f"±{entry.repeatability_mm:g} mm")
        if not specs:
            specs.append(tr("规格未知"))

        spec_label = QLabel(" · ".join(specs))
        spec_label.setStyleSheet("color: #9a9aab;")
        spec_font = QFont()
        spec_font.setPointSize(8)
        spec_label.setFont(spec_font)
        text_layout.addWidget(spec_label)

        layout.addLayout(text_layout, 1)

    @staticmethod
    def _initials(model_name: str) -> str:
        """取型号名首字母缩写，最多 2 个字符。"""
        parts = [p for p in model_name.replace("-", " ").split() if p]
        if not parts:
            return "?"
        if len(parts) == 1:
            return parts[0][:2].upper()
        return (parts[0][0] + parts[1][0]).upper()


#: 列表行高（宽度由布局自动拉伸，此处仅高度生效）
_RobotCard.SIZE_HINT = QSize(10, 46)


class RobotLibraryPanel(QWidget):
    """机器人库侧栏面板。

    Args:
        models_dir: 模型根目录
        parent: 父控件

    Signals:
        robotSelected: 用户双击某型号，参数为 URDF 文件路径字符串
        statusMessage: 需要展示到状态栏的提示文本
        partsImported: 零件库导入成功，参数为导入的自定义零件数
    """

    robotSelected = Signal(str)
    componentActivated = Signal(str)
    statusMessage = Signal(str)
    partsImported = Signal(int)

    def __init__(self, models_dir: Path, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setStyleSheet(PANEL_STYLESHEET)
        self.setMinimumWidth(260)

        self.catalog = RobotCatalog(models_dir)
        self._updating = False
        #: 已导入零件库的**来源文件**（去重、按导入顺序）——v1.3.4 工程文件记录它
        self.parts_sources: list = []

        self._build_ui()
        self._populate_brand_filter()
        self.refresh()

    # ---------------- UI 构建 ---------------- #
    def _build_ui(self) -> None:
        # 本面板是左侧「场景设备库」Tab 的内容，高度完全跟随窗口：搜索框 + 两排
        # 筛选 + 列表 + 统计 + 提示的自然高度约 290px，不套滚动容器时小窗口会把
        # 底部的「刷新」与拖拽提示挤出可视区。滚动容器让"内容比可视区高"退化成
        # 滚动条，而不是裁切（PRD §6「任一标签页内容在小窗口下不被裁切」）。
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameStyle(QFrame.NoFrame)
        inner = QWidget()
        scroll.setWidget(inner)
        outer.addWidget(scroll)

        layout = QVBoxLayout(inner)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        # --- 搜索框 --- #
        self.search_edit = QLineEdit()
        self.search_edit.setPlaceholderText(tr("搜索型号 / 品牌 / 组件…"))
        self.search_edit.setClearButtonEnabled(True)
        self.search_edit.textChanged.connect(self._on_filters_changed)
        layout.addWidget(self.search_edit)

        # --- 分类（机器人 / 组件） --- #
        self.cat_combo = QComboBox()
        self.cat_combo.addItem(tr("全部"), _CAT_ALL)
        self.cat_combo.addItem(tr("机器人"), _CAT_ROBOT)
        self.cat_combo.addItem(tr("组件"), _CAT_COMPONENT)
        self.cat_combo.currentIndexChanged.connect(self._on_filters_changed)
        layout.addWidget(self.cat_combo)

        # --- 品牌 / 机型 --- #
        row1 = QHBoxLayout()
        row1.setSpacing(6)
        self.brand_combo = QComboBox()
        self.brand_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.brand_combo.currentIndexChanged.connect(self._on_filters_changed)
        row1.addWidget(self.brand_combo, 3)

        self.type_combo = QComboBox()
        self.type_combo.addItem(tr("全部机型"), None)
        self.type_combo.addItem(tr("工业臂"), "industrial")
        self.type_combo.addItem(tr("协作臂"), "cobot")
        self.type_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.type_combo.currentIndexChanged.connect(self._on_filters_changed)
        row1.addWidget(self.type_combo, 2)
        layout.addLayout(row1)

        # --- 负载 / 臂展 --- #
        row2 = QHBoxLayout()
        row2.setSpacing(6)
        self.payload_combo = QComboBox()
        for text, lo, hi in PAYLOAD_FILTERS:
            self.payload_combo.addItem(text, (lo, hi))
        self.payload_combo.currentIndexChanged.connect(self._on_filters_changed)
        row2.addWidget(self.payload_combo)

        self.reach_combo = QComboBox()
        for text, lo, hi in REACH_FILTERS:
            self.reach_combo.addItem(text, (lo, hi))
        self.reach_combo.currentIndexChanged.connect(self._on_filters_changed)
        row2.addWidget(self.reach_combo)
        layout.addLayout(row2)

        # --- 型号列表 --- #
        self.list_widget = QListWidget()
        # 给列表一个下限高度：否则小窗口下面板会把剩余高度全给列表、筛选行反而
        # 先被压扁；有下限后"内容高于可视区"由外层滚动条承接。
        self.list_widget.setMinimumHeight(160)
        self.list_widget.setSelectionMode(QAbstractItemView.SingleSelection)
        self.list_widget.setSpacing(0)
        self.list_widget.setDragEnabled(True)
        self.list_widget.setDragDropMode(QAbstractItemView.DragOnly)
        self.list_widget.itemDoubleClicked.connect(self._on_item_activated)
        self.list_widget.itemSelectionChanged.connect(self._on_selection_changed)
        layout.addWidget(self.list_widget, 1)

        # --- 底部：统计 + 导入零件 + 刷新 --- #
        bottom = QHBoxLayout()
        bottom.setSpacing(6)
        self.stats_label = QLabel()
        self.stats_label.setStyleSheet("color: #8a8a99;")
        stats_font = QFont()
        stats_font.setPointSize(8)
        self.stats_label.setFont(stats_font)
        bottom.addWidget(self.stats_label, 1)

        import_btn = QPushButton(tr("导入零件…"))
        import_btn.setToolTip(
            tr("导入零件库 JSON（orolp.parts/1，参数化基本体的自定义工件/结构件）")
        )
        import_btn.clicked.connect(self.on_import_parts)
        bottom.addWidget(import_btn)

        refresh_btn = QPushButton(tr("刷新"))
        refresh_btn.setToolTip(tr("重新扫描 models/ 目录"))
        refresh_btn.clicked.connect(self.refresh)
        bottom.addWidget(refresh_btn)
        layout.addLayout(bottom)

        hint = QLabel(tr("拖拽到右侧视口加入场景 · 双击载入"))
        hint.setStyleSheet("color: #6f6f7d;")
        hint_font = QFont()
        hint_font.setPointSize(7)
        hint.setFont(hint_font)
        layout.addWidget(hint)

    def _populate_brand_filter(self) -> None:
        """填充品牌下拉框（含各自的型号数量）。"""
        self.brand_combo.clear()
        self.brand_combo.addItem(tr("全部品牌"), None)
        for brand_id, display, count in self.catalog.brands():
            self.brand_combo.addItem(f"{display} ({count})", brand_id)

    # ---------------- 刷新与筛选 ---------------- #
    def refresh(self) -> None:
        """重新扫描模型目录并重建列表。"""
        self.catalog.refresh()
        # 记住当前品牌选择，刷新后尽量恢复
        current = self.brand_combo.currentData()
        self._populate_brand_filter()
        idx = self.brand_combo.findData(current)
        self.brand_combo.setCurrentIndex(idx if idx >= 0 else 0)
        self._apply_filters()
        self.statusMessage.emit(tr("机器组件库：共 {n} 个型号 / 组件已并入").format(n=len(self.catalog)))

    # ---------------- 零件库导入（PRD §4.1 P1） ---------------- #
    def import_parts_file(self, path) -> int:
        """从零件库 JSON 导入自定义零件并注册进组件目录。

        幂等：注册前先撤掉上一批自定义零件（``custom_`` 前缀），
        重复导入同一文件不会撞名、也不会残留旧几何。

        Returns:
            导入的零件数

        Raises:
            ValueError: schema 不符或零件内容非法（库层校验抛出）
            RuntimeError: 组件库（世界建模）不可用
        """
        if not _HAS_COMPONENTS:
            raise RuntimeError(tr("组件库不可用，无法导入零件"))
        from ..world.parts_library import (  # 局部导入：面板不因缺世界建模而挂
            CUSTOM_ID_PREFIX,
            COMPONENTS,
            CustomPartsLibrary,
            register_custom_parts,
            unregister_custom_parts,
        )

        lib = CustomPartsLibrary.load_json(path)
        previous = [cid for cid in list(COMPONENTS) if cid.startswith(CUSTOM_ID_PREFIX)]
        unregister_custom_parts(previous)
        ids = register_custom_parts(lib)
        # 记下来源文件（v1.3.4 工程文件用：工程里存的是"从哪几个文件导的零件"，
        # 而不是注册后的组件——组件目录里那批是可被替换的派生结果）
        source = str(path)
        if source not in self.parts_sources:
            self.parts_sources.append(source)
        self.refresh()
        return len(ids)

    def on_import_parts(self) -> None:
        """「导入零件…」按钮：选文件 → 导入 → 状态栏反馈。"""
        from PySide6.QtWidgets import QFileDialog

        path, _ = QFileDialog.getOpenFileName(
            self, tr("导入零件库"), "", tr("零件库 JSON (*.json);;所有文件 (*)")
        )
        if not path:
            return
        try:
            n = self.import_parts_file(path)
        except Exception as exc:
            self.statusMessage.emit(
                tr("导入零件失败：{kind}: {msg}").format(kind=type(exc).__name__, msg=exc)
            )
            return
        self.statusMessage.emit(
            tr("零件库已导入：{n} 个自定义零件（「自定义」分类下，可拖入视口）").format(n=n)
        )
        self.partsImported.emit(n)

    def _on_filters_changed(self) -> None:
        if self._updating:
            return
        self._apply_filters()

    def _apply_filters(self) -> None:
        """按当前筛选条件重建列表（机器人 + 组件）。"""
        self._updating = True
        try:
            cat = self.cat_combo.currentData()
            payload = self.payload_combo.currentData() or (None, None)
            reach = self.reach_combo.currentData() or (None, None)

            self.list_widget.clear()
            added = 0

            # 机器人：按品牌二级分组
            if cat in (_CAT_ALL, _CAT_ROBOT):
                entries = self.catalog.search(
                    keyword=self.search_edit.text(),
                    brand_id=self.brand_combo.currentData() if cat == _CAT_ROBOT else None,
                    robot_type=self.type_combo.currentData(),
                    min_payload=payload[0],
                    max_payload=payload[1],
                    min_reach=reach[0],
                    max_reach=reach[1],
                    sort_by="brand",
                )
                if entries and cat != _CAT_ROBOT:
                    self._add_header(tr("机器人"))
                last_brand = None
                for entry in entries:
                    if cat == _CAT_ROBOT and entry.brand_id != last_brand:
                        self._add_header(entry.brand_display)
                        last_brand = entry.brand_id
                    self._add_robot_item(entry)
                    added += 1

            # 组件：按分类分组
            if _HAS_COMPONENTS and cat in (_CAT_ALL, _CAT_COMPONENT):
                kw = self.search_edit.text().strip().lower()
                specs = [s for s in COMPONENTS.values() if kw in s.name.lower()]
                if cat == _CAT_ALL:
                    by_cat = {}
                    for s in specs:
                        by_cat.setdefault(s.category, []).append(s)
                    for c in _component_categories():
                        items = by_cat.get(c)
                        if items:
                            self._add_header(c)
                            for s in items:
                                self._add_component_item(s)
                                added += 1
                else:
                    for s in specs:
                        self._add_component_item(s)
                        added += 1

            self.stats_label.setText(tr("{n} 项").format(n=added))
        finally:
            self._updating = False

    @staticmethod
    def _tooltip(entry: CatalogEntry) -> str:
        """列表项的悬浮提示。"""
        lines = entry.summary_lines()
        if entry.note:
            lines.append("")
            lines.append(entry.note)
        if entry.urdf_path:
            lines.append("")
            lines.append(f"URDF: {entry.urdf_path}")
        return "\n".join(lines)

    # ---------------- 交互 ---------------- #
    def _current_entry(self) -> Optional[CatalogEntry]:
        item = self.list_widget.currentItem()
        if item is None:
            return None
        data = item.data(Qt.UserRole)
        if isinstance(data, dict) and data.get("kind") == "robot":
            return self.catalog.get(data["id"])
        return None

    def _on_selection_changed(self) -> None:
        entry = self._current_entry()
        if entry and entry.urdf_path:
            self.statusMessage.emit(tr("{name} — 双击载入").format(name=entry.display_name))

    def _on_item_activated(self, item: QListWidgetItem) -> None:
        data = item.data(Qt.UserRole)
        if not isinstance(data, dict):
            return  # 分组标题，忽略
        if data["kind"] == "robot":
            entry = self.catalog.get(data["id"])
            if entry is None:
                return
            if not entry.urdf_path or not entry.urdf_path.exists():
                self.statusMessage.emit(tr("{name} 的 URDF 不可用").format(name=entry.display_name))
                return
            self.robotSelected.emit(str(entry.urdf_path))
        elif data["kind"] == "component":
            self.componentActivated.emit(data["id"])

    def select_key(self, key: str) -> bool:
        """按 key 选中列表中的型号（不触发加载）。

        Returns:
            是否找到并选中
        """
        for i in range(self.list_widget.count()):
            data = self.list_widget.item(i).data(Qt.UserRole)
            if isinstance(data, dict):
                if data.get("kind") == "robot" and data.get("id") == key:
                    self.list_widget.setCurrentRow(i)
                    self.list_widget.scrollToItem(self.list_widget.item(i))
                    return True
            elif data == key:
                self.list_widget.setCurrentRow(i)
                self.list_widget.scrollToItem(self.list_widget.item(i))
                return True
        return False

    # ---------------- 组件与拖拽辅助 ---------------- #

    @staticmethod
    def _component_card(spec) -> QFrame:
        """组件列表卡片（与机器人卡片风格一致）。"""
        frame = QFrame()
        frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(8)

        icon = QLabel()
        icon.setPixmap(_brand_chip((0.36, 0.62, 0.86), spec.name[:1]))
        icon.setFixedSize(28, 28)
        layout.addWidget(icon)

        text = QVBoxLayout()
        text.setSpacing(1)
        text.setContentsMargins(0, 0, 0, 0)

        name = QLabel(spec.name)
        name.setStyleSheet("color: #f0f0f6; font-weight: bold; font-size: 9pt;")
        text.addWidget(name)

        summary = []
        for p in spec.params[:3]:
            summary.append(f"{p.label}: {p.default:g}{p.unit}")
        if not summary:
            summary.append(tr("固定几何"))
        sub = QLabel(" · ".join(summary))
        sub.setStyleSheet("color: #9a9aab; font-size: 8pt;")
        text.addWidget(sub)

        layout.addLayout(text, 1)
        return frame

    def _add_header(self, text: str) -> None:
        item = QListWidgetItem(text)
        item.setFlags(Qt.NoItemFlags)
        item.setData(Qt.UserRole, None)
        item.setForeground(Qt.gray)
        font = item.font()
        font.setBold(True)
        item.setFont(font)
        self.list_widget.addItem(item)

    def _add_robot_item(self, entry: CatalogEntry) -> None:
        item = QListWidgetItem(self.list_widget)
        item.setData(Qt.UserRole, {"kind": "robot", "id": entry.key})
        item.setSizeHint(_RobotCard.SIZE_HINT)
        self.list_widget.setItemWidget(item, _RobotCard(entry))
        item.setToolTip(self._tooltip(entry))

    def _add_component_item(self, spec) -> None:
        item = QListWidgetItem(self.list_widget)
        item.setData(Qt.UserRole, {"kind": "component", "id": spec.id})
        item.setSizeHint(QSize(10, 46))
        self.list_widget.setItemWidget(item, self._component_card(spec))
        item.setToolTip(spec.name)

    def mimeData(self, items):
        data = items[0].data(Qt.UserRole) if items else None
        if isinstance(data, dict):
            m = QMimeData()
            m.setData(MIME_TYPE, QByteArray(json.dumps(data).encode("utf-8")))
            return m
        return super().mimeData(items)
