---
name: translate_readme_to_chinese
overview: 将 README.md 改写为全中文（保留代码块、文件路径、标识符与英文术语），同步修正与当前软件不符的表述，并删除已过时的 docs/README_zh.md 中文镜像。
todos:
  - id: translate-top-progress
    content: 翻译 README 顶部简介、开发进展表、支持机器人表为中文并校正 GUI 标签
    status: completed
  - id: translate-body
    content: 翻译 安装/架构/路线图/测试/贡献/许可/致谢 各段，校对右侧子标签
    status: completed
  - id: finalize-header-proofread
    content: 移除顶部「中文使用说明」链接并通读中文术语一致性
    status: completed
    dependencies:
      - translate-top-progress
      - translate-body
  - id: delete-stale-zh
    content: 删除 docs/README_zh.md 消除重复与过时内容
    status: completed
---

## 用户需求

- 将仓库主文档 `README.md` 改写为**全中文**使用说明：正文全部中文化，但代码块、文件路径、标识符（如 `orolp`、`moveJ`、`FileLink`）、以及英文专有术语（URDF / OLP / URScript / KRL / RAPID / PySide6 / Jinja2 等）保留原文。
- 检查软件实际功能与 `docs/screenshots/` 下的 7 张截图，更新使用说明，使其与 `app.py` 当前 GUI 完全一致。
- 删除已过时、与软件状态不符的 `docs/README_zh.md`，消除重复与错误信息。

## 产品概述

`README.md` 将由「英文主文档 + 部分中文」转变为**唯一的中文主文档**；顶部导航不再保留指向 `docs/README_zh.md` 的「中文使用说明」链接（文档本身即中文），仅保留英文补充文档链接（后处理器开发指南 / 厂商 CAD 接入 / PRD·计划）。`docs/README_zh.md` 直接删除。

## 核心特性（文档更新点）

- **全文中文化**：顶部简介与愿景、Development Progress 表、Supported robots 表、Installation、Architecture（含框内英文）、Roadmap、Tests、Contributing、License、Acknowledgements 全部翻译为中文。
- **GUI 标签对齐实测**（`grep app.py` 确认）：左侧为「资源库」；右侧多标签为「属性 / 动作与从属 / 代码预览 / 导出打包」；其中「代码预览」内含子标签「路径示教 / 硬件列表 / PLC 程序逻辑 / 机器人子程序」——使用说明据此校正（此前部分段落仍用英文旧名）。
- **截图核对**：`docs/screenshots/` 下 7 张图片（main_ur5e / path_code / offline_programming / jaka_zu5_real / grasping_workstation / grasping_playback / grasping_offline_programming）均存在且 README 引用完整，无需增删；抓取工作站 3 张截图说明（S1–S5 一键演示、回放、离线编程校验）保留。
- **测试计数统一**：Tests 段与徽章已为 281；Roadmap 历史里程碑计数（137→233→251→263→267→281）属递进记录予以保留，但路线图开头运动学条目的孤立「137 tests passing」按 281 上下文统一措辞，避免与当前总数冲突。
- **删除** `docs/README_zh.md`：该文件缺失 S1–S5 抓取工作站、3 张工作站截图、云端小节与右侧新标签名，且误将 Scene/collision 标为「规划中」，删除后由 `README.md` 单一中文来源取代。