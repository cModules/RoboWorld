---
name: fix_property_panel_hidden_on_edit
overview: 修复编辑位置/参数时右侧「对象属性」面板被隐藏的问题：让场景树刷新后保留选中项。
todos:
  - id: fix-refresh-tree
    content: 修改 panel.py 的 _refresh_tree()，用 blockSignals 保留并恢复场景树选中项
    status: completed
  - id: verify-panel
    content: 重启 app.py 并验证键盘输入参数时属性面板不再隐藏
    status: completed
    dependencies:
      - fix-refresh-tree
---

## 用户需求

修复 app.py 右侧属性面板在键盘输入「位置/参数」数值（如夹爪位姿）时，参数面板被自动隐藏、仅剩底部日志的问题。

## 产品概述

RobotWorld 桌面 GUI 在 3D 视口右侧提供「属性」标签页，用于编辑选中对象（机器人/夹爪/工件）的位姿与模型参数。当前编辑数值时面板突然消失，影响连续调参体验。

## 核心特性

- 编辑对象位姿或参数时，右侧「对象属性」面板保持显示，不再因场景树刷新而隐藏
- 场景树在刷新后保留当前选中项，选中高亮与属性回填保持一致
- 修复仅针对选中状态丢失这一根因，不改变现有属性编辑、树层级、干涉检查等其余行为

## 技术栈

- 语言/框架：Python + PySide6（Qt Widgets），桌面 GUI
- 涉及模块：`python/orolp/world/panel.py`（场景树面板）、`app.py`（主窗口与信号连线）

## 实现思路

**根因**：编辑数值 → `QDoubleSpinBox.valueChanged` → `_on_obj_pose_edited/_on_obj_param_edited` 修改场景对象并 `emit objectChanged()` → 该信号连接 `scene_panel._refresh_tree()` 与 `gl_widget.update()`（app.py:3241）。`_refresh_tree()` 直接 `self.tree.clear()` 清空重建且未保留选中项，`clear()` 后当前项变为 `None`，触发 `_on_tree_selected(None, ...)` → `objectSelected.emit(None)` → `property_panel.set_object(None)` 将 `object_scroll` 区域 `hide()`，导致面板消失。

**方案**：在 `python/orolp/world/panel.py` 的 `_refresh_tree()` 中保留并恢复选中状态。

1. 进入前记录 `self._selected`；
2. 用 `self.tree.blockSignals(True)` 包裹 `clear()` + 重建过程，避免清空中触发 `currentItemChanged(None)`；
3. 重建后若 `self._selected` 仍存在，用 `blockSignals` 包裹 `self._select_object(selected)` 恢复高亮（不误发 `objectSelected(None)`）；若不存在则置 `self._selected = None`；
4. 恢复信号后更新统计标签与泳道视图。

**关键技术决策**

- 采用 `blockSignals` + 保存/恢复选中项，而非改用 `editingFinished`：根因是任何 `objectChanged`（含挂载/层级变化）都会触发树刷新丢失选中，仅改信号源无法根治，且会牺牲实时调参反馈。
- 保留 `objectChanged → _refresh_tree` 连线不变，避免影响挂载后层级同步等既有逻辑。

## 性能与可靠性

- `_refresh_tree` 调用频率低（仅编辑/挂载/增删时），保留选中项为 O(对象数) 线性，无性能风险。
- `blockSignals` 成对包裹，避免信号误发；`finally` 确保异常时也能恢复信号，不破坏后续交互。

## 实现备注

- 仅修改 `_refresh_tree()` 内部逻辑，不改动其签名与外层调用。
- 修改后需重启本地运行中的 `app.py` 进程才能生效（当前为运行时状态，热更新无效）。
- 验证路径：选中夹爪 → 在「位置/参数」输入框键入数值（含回车与失焦）→ 确认右侧面板不消失、场景树选中项与属性回填保持。

## 目录结构

```
python/orolp/world/
└── panel.py   # [MODIFY] 修改 _refresh_tree()，刷新前记录 self._selected，用 blockSignals 包裹 clear+重建，重建后恢复选中项（不存在则置 None）。修复编辑参数时属性面板被误隐藏。
```