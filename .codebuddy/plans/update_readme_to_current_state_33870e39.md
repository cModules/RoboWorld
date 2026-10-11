---
name: update_readme_to_current_state
overview: 根据当前软件实际状态，更新 README.md 的使用说明与中文描述（GUI 面板/标签名、测试计数、云端开发容器说明）。
todos:
  - id: verify-state
    content: 复核 README 与实际代码状态差异清单（面板名/标签名/测试计数）
    status: completed
  - id: fix-gui-names
    content: 更新进度表与架构图的 GUI 面板与标签名称
    status: completed
    dependencies:
      - verify-state
  - id: fix-feature-demo
    content: 改写关键功能演示：主视图、代码预览、导出打包 及对象位姿/参数编辑
    status: completed
    dependencies:
      - fix-gui-names
  - id: fix-test-count
    content: 将 Tests 段测试计数由 263 改为 281
    status: completed
    dependencies:
      - verify-state
  - id: add-cloud-section
    content: 新增云端开发容器（Cloud Studio · noVNC）小节
    status: completed
    dependencies:
      - verify-state
---

## 用户需求

根据 RoboWorld 当前实际运行状态，更新 `README.md` 的使用说明与中文描述，使其与代码真实一致。

## 产品概述

`README.md` 当前多处 UI 描述与 `app.py` 实际面板结构不符，测试计数前后不一致，且缺少云端开发容器（Cloud Studio / noVNC）的使用说明。需校正这些偏差，提升文档可用性。

## 核心特性（待更新点）

- GUI 面板/标签名对齐真实代码：左侧「资源库」；右侧标签页「属性 / 动作与从属 / 代码预览 / 导出打包」
- 关键功能演示章节补充：选中组件可编辑「位置（位姿 XYZ+RPY）」与「模型参数」
- 测试计数统一为 281（徽章与进度表已为 281，Tests 段仍写 263）
- 新增「云端 / 容器运行（Cloud Studio · noVNC）」小节：`.ide/Dockerfile` 已用 `PySide6-Essentials` 缓解 OOM，并集成 Xvfb+x11vnc+websockify+noVNC，端口 6080

## 备注

上一轮发现本环境 `replace_in_file` / `execute_command` 工具未注册时无法直接写文件；落地时先确认编辑工具是否可用，若不可用则改为输出可直接粘贴的 diff 供用户应用，不阻塞文档内容正确性。