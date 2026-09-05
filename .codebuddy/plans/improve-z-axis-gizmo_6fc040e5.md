---
name: improve-z-axis-gizmo
overview: 修复 Z 轴手柄难以选中：扩大命中区（竖条 9→16px + 顶端 18px 抓取圆盘）并在视口绘制可见抓手，使右键/Control+左键拖动能稳定做 Z 方向上下移动。
todos:
  - id: geom-z-params
    content: 修改 GLWidget._gizmo_screen_geom，新增 z_strip/z_tip_r 并将 hit 由 9 调到 10
    status: completed
  - id: hit-z-mode
    content: 修改 GLWidget._gizmo_hit_mode，使 Z 顶端圆盘与加宽竖条优先命中
    status: completed
    dependencies:
      - geom-z-params
  - id: draw-z-disk
    content: 修改 GLWidget._draw_gizmo，新增 disk 助手、加粗 Z 轴并绘制顶端抓手圆盘
    status: completed
    dependencies:
      - geom-z-params
  - id: verify-z-pick
    content: 重启 python olp.py，验证 Z 轴圆盘/竖条可抓取并上下移动，XY 与绕Z 不受影响
    status: completed
    dependencies:
      - hit-z-mode
      - draw-z-disk
---

## 用户需求

MacBook 触控板用户在 3D 视口里右键（或 Control+左键）选中场景对象后，红色 **Z 轴手柄难以选中**，导致无法进行 Z 方向上下移动。

## 产品概述

当前 gizmo 的 Z 轴仅是一条从中心向上画的细红线（线宽 2px），且命中判定局限于中心线上 **9px 宽**的极窄竖条，肉眼几乎无法对准，经常出现“点空”落到 XY 方块或空白区的情况。需要让 Z 轴既**更好看、更易抓**，又**不改已有交互语义**。

## 核心功能

- 扩大 Z 轴命中区域：由 9px 窄竖条扩展为「16px 加宽竖条 + 18px 顶端圆盘」。
- 在 Z 轴顶端绘制一个实心圆盘作为可见抓手，并把 Z 轴线加粗（3px）。
- Z 轴命中优先级高于 XY 方块；XY 方块、绕 Z 黄环、右键拾取/拖动等既有功能不受影响。

## 技术栈

- GUI：PySide6（`QOpenGLWidget` / 屏幕像素空间绘制 `GL_TRIANGLE_FAN` / `GL_LINES`）
- 数值：numpy（已用于射线与投影，本次不新增依赖）
- 复用：`GLWidget._gizmo_screen_geom`（几何参数）、`_gizmo_hit_mode`（命中判定）、`_draw_gizmo`（绘制）现有结构；不动对外信号 `objectSelected` 与 `ScenePanel` 接线。

## 实现方案

### 总体策略

仅对 `GLWidget` 的 gizmo 三处方法做局部增强：① 几何参数增加 Z 专用命中尺寸；② 命中判定优先识别 Z 顶端圆盘与加宽竖条；③ 绘制层面新增圆盘助手并加粗 Z 轴。改动局限在 `GLWidget` 内部，零新依赖、零架构变化。

### 关键技术决策

1. **几何参数（`_gizmo_screen_geom`）**：`hit` 9→10；新增 `z_strip=16.0`（竖条命中半宽）、`z_tip_r=18.0`（顶端圆盘抓取半径）。把“命中尺寸”集中在一处，便于后续整体调参。
2. **命中判定（`_gizmo_hit_mode`）**：Z 轴判定前置——先判顶端圆盘 `math.hypot(dx, dy + g["axis_len"]) < g["z_tip_r"]`，再判加宽竖条 `abs(dx) < g["z_strip"] and -g["axis_len"] - g["hit"] <= dy <= g["hit"]`，其后才是旋转环、XY 方块。优先 Z 可避免中心列被 XY 方块抢走，但加宽竖条仅 16px、小于 XY 方块 42px，互不冲突。
3. **绘制（`_draw_gizmo`）**：新增 `disk(cx,cy,r,color)`（`GL_TRIANGLE_FAN` 实心圆）；Z 轴 `line(..., 3.0)` 加粗，并在顶端 `disk(cx, cy - axis_len, z_tip_r*0.6, z_col)` 画实心抓手，提供明确可点击视觉目标。

### 性能与可靠性

- 纯像素空间运算与少量绘制调用，开销为零；命中判定仅在右键按下/拖动时触发，非逐帧。
- 单位与现有一致（屏幕像素），`px_per_m` 等复用不变。
- 容错：`_gizmo_screen_geom` 在 `selected_object is None` 时返回 `None`，各调用点已有空值保护；圆盘/竖条命中越界安全返回 `None`。

## 目录结构

```
/Users/jayhan/RobotWorld/
└── olp.py   # [MODIFY] GLWidget._gizmo_screen_geom：新增 z_strip/z_tip_r、hit 9→10；
             #          _gizmo_hit_mode：Z 顶端圆盘 + 加宽竖条优先判定；
             #          _draw_gizmo：新增 disk 助手、Z 轴加粗并绘制顶端抓手圆盘。
```

## 关键代码结构（命中判定核心）

```python
# _gizmo_hit_mode 中 Z 轴判定（其余旋转环/XY 方块保持不变，仅顺序前置 Z）
# 1) Z 轴：顶端圆盘抓取点 + 整条加宽竖条，优先于 XY 方块
if math.hypot(dx, dy + g["axis_len"]) < g["z_tip_r"]:
    return "z"
if abs(dx) < g["z_strip"] and -g["axis_len"] - g["hit"] <= dy <= g["hit"]:
    return "z"
```