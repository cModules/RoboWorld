---
name: fix-viewport-rightclick-pick
overview: 修复视口右键拾取“无反应”：根因是 _view_proj_matrix 中相机视图矩阵的旋转写成了列向量（被转置），导致反推射线方向错误、射线-AABB 永远打不中对象。改为按 paintGL 一致的行向量构造，并顺带增强 Mac 触控板兼容性（Control+左键作为右键别名）。
todos:
  - id: fix-view-matrix
    content: 修正 GLWidget._view_proj_matrix 视图矩阵旋转为行赋值，使其与 paintGL 相机一致
    status: completed
  - id: mac-right-alias
    content: 在 mousePress/Move/ReleaseEvent 增加 Control+左键作为右键别名（self._right_like）
    status: completed
    dependencies:
      - fix-view-matrix
  - id: verify-pick
    content: 重启 python olp.py 验证右键/Control+左键选中、gizmo 与位姿双向同步
    status: completed
    dependencies:
      - mac-right-alias
---

## 用户需求

MacBook 触控板用户在 3D 视口中右键点中场景对象后，没有任何选中反馈（无描边高亮、无坐标系 gizmo、左侧“选中对象位姿”列表也不高亮），需要修复右键拾取使其正常工作。

## 核心问题

- 根因已通过只读核查定位：`GLWidget._view_proj_matrix()` 构造的视图矩阵旋转部分用了 numpy 的**列赋值**（`V[:3,0]=s` 等），而 numpy 为行主序，实际写成了正确矩阵的转置（正交矩阵转置=逆）。这导致反推的射线原点/方向错误，射线-包围盒求交（`_pick_object`）几乎必然返回 `None`，右键分支始终 `selected_object=None`，故无任何选中反馈。
- 投影矩阵 `P` 实现正确，无需改动。
- 平移项因列赋值“凑巧”算对，但旋转错误已足以让拾取失效。

## 修复目标

1. 修正 `_view_proj_matrix` 的视图矩阵旋转为行赋值，使其与 `paintGL` 的 `glLoadMatrixf(rot)`（列主序，行依次为 `s,u,-f`）完全一致，恢复射线拾取、投影、gizmo 命中的正确坐标。
2. 增强 Mac 触控板兼容性：将 `Control+左键` 也视作“右键拾取/拖动”别名（触控板两指轻点本就发 `Qt.RightButton`，已天然支持；Control+Click 是 Mac 经典右键替代）。
3. 验证：视口右键（或 Control+左键）点中对象 → 描边高亮 + 三色 gizmo + 左侧位姿框启用回填；光标移到各手柄锁定模式并拖动生效；左键无 Control 时不触发拾取。

## 技术栈

- GUI：PySide6（`QOpenGLWidget` / 鼠标事件 / `Qt.ControlModifier`）
- 数值：numpy（行主序矩阵）
- 复用：`GLWidget.paintGL` 现有相机模型、`geometry.prim_aabb`、`Pose.from_xyz_rpy`

## 实现方案

### 总体策略

仅修正相机视图矩阵的构造方式（行主序下应做“行赋值”），并补充 Mac 触控板的 Control+左键右键别名。不动投影矩阵、不动 `ScenePanel` 公共 API、不引入新依赖。

### 关键技术决策

1. **修正视图矩阵旋转（核心修复）**
`paintGL` 用 `glLoadMatrixf(rot)` 按列主序加载，旋转部分**行**依次为 `s, u, -f`。在 numpy（行主序）中必须用**行赋值**表达同一矩阵：

```python
V = np.eye(4)
V[0, :3] = s
V[1, :3] = u
V[2, :3] = -f
V[:3, 3] = -(V[:3, :3] @ eye)
```

原代码 `V[:3,0]=s` 是把 `s` 写进了**列 0**，等价于把 `(s0,u0,-f0)` 当作行，旋转被转置。改为行赋值后，`_ray_from_mouse` / `_project` / `_pick_object` / `_gizmo_hit_mode` 全部回到正确坐标系。
投影矩阵 `P`（`glFrustum` 对应）实现正确，保持不动。

2. **Mac 触控板兼容（增强）**

- `mousePressEvent`：判定 `is_right = (event.button() == Qt.RightButton) or (event.button() == Qt.LeftButton and event.modifiers() & Qt.ControlModifier)`；命中时记录 `self._right_like = True`，并走既有拾取/选中逻辑；左键无 Control 时仅做相机旋转。
- `mouseMoveEvent`：右键拖动判定改为 `event.buttons() & Qt.RightButton or (self._right_like and event.buttons() & Qt.LeftButton)`，保证 Control+左键按住拖动也触发 gizmo 变换。
- `mouseReleaseEvent`：在释放时清理 `self._right_like = False`（右键与左键分支同步）。
- 触控板两指轻点发送的 `Qt.RightButton` 已天然被原逻辑覆盖，本增强只补充 Control+Click 路径。

### 性能与可靠性

- 仅一处矩阵构造修正 + 少量事件分支，开销为零；拾取/拖动仍在右键事件触发，非逐帧。
- 单位统一：场景/几何毫米、视口米，`MM_TO_M=0.001` 不变。
- 容错：`scene` 为 None / 无对象 / 未命中时 `_pick_object` 安全返回 `None`，不报错。

## 架构设计

修改仅局限于 `GLWidget` 内部，不改变对外信号（`objectSelected`）与 `ScenePanel` 接线，架构无变化。

## 目录结构

```
/Users/jayhan/RobotWorld/
└── olp.py   # [MODIFY] GLWidget._view_proj_matrix：视图矩阵旋转改行赋值；
             #          mousePressEvent / mouseMoveEvent / mouseReleaseEvent：
             #          增加 Control+左键 作为右键别名（self._right_like 标志）。
```

## 验证

- 重启 `python olp.py`；视口右键（或 Control+左键）点中工作台/控制柜/机器人底座 → 对象描边高亮、原点出现三色 gizmo、左侧“选中对象位姿”框启用并回填。
- 光标移到蓝色方块/红线/黄环分别锁定 XY 移动 / Z 升降 / 绕Z 旋转，按住拖动生效，左侧数值实时变化。
- 左键（无 Control）仍可旋转视角；左键无 Control 时不触发拾取。