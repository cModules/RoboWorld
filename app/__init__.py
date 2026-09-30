"""GUI 层包：PySide6 / OpenGL 的唯一归宿（ADR-002；2026-09-26 提升为**顶层包**）。

位置：仓库根 ``app/``——与引擎包 ``core``（源码在 ``core/``）并列的两个
顶层包之一。依赖方向 ``app → core`` **单向**，引擎侧禁止 import 本包。

背景：启动壳（原 ``app.py``，现 ``main.py``）曾是 7121 行单文件、5 个类互相纠缠，
是仓库最大的可维护性风险。本包是它的目标态——按 ADR-002 逐个类迁进来：

1. ``views/robot_renderer.py``   ← 第一步（2026-09-25）
2. ``views/property_panel*.py``  ← 第二步（2026-09-25，665 行按职责拆三块）
3. ``views/gl_widget.py``、``views/path_code_editor.py``（待迁，需先定子模块边界）
4. ``mainwindow/``  主窗口编排
5. ``controllers/`` 用户操作 → 领域服务的薄适配（不含算法）

纪律（ADR-002 / AGENTS.md #5）：**一次只搬一个类，测试全绿再提交**，
禁止"顺手重构"——搬迁只换位置，不改行为。``main.py`` 在全部迁完前保留为启动壳。
"""
