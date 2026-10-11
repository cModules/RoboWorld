---
name: robotworld-arch-guard
description: RoboWorld（/Users/jayhan/RoboWorld，工业机器人离线编程软件）架构合规技能。任何涉及写代码、改代码、重构、评审的任务，编码前必须加载本技能：确定新代码归属层与模块、执行行数阈值与依赖方向自检、运行 tools/check_arch_rules.py 守护脚本。触发词：写代码/改代码/重构/新增功能/修 bug/代码评审/模块归属/拆分文件/架构检查。
version: 1.0.0
---

# RoboWorld 架构守护 · 编码前强制自检

> 规范全文：`docs/CODING_RULES.md`（v2）· 边界裁决：`AGENTS.md` · 架构依据：`docs/ARCHITECTURE.md`
> 本技能是三份文档的**执行入口**：编码前过一遍，收尾跑一次守护脚本。

## 0. 编码前三问（30 秒定归属）

1. **这是什么？** 业务规则 → Domain；业务用例 → Service；可替换技术边界（模型解析/协议/后处理/插件）→ Adapter；界面编排/视图/面板 → App（`orolp/app/`）。
2. **它依赖谁？** 只允许 `app → services → domain`、`adapters → domain`。Domain 零 GUI 零 IO（引用 `orolp.plugins` 注册表机制合法）。
3. **放哪？** 按 `docs/CODING_RULES.md` §1 查表；表中没有的模块先停下来问用户（AGENTS.md #22）。

## 1. 硬性红线（违规即停）

- PySide6/OpenGL 只准进 `orolp/app/`；存量债务文件 `controller_panel.py`、`program_panel.py`、`robot_library/panel.py`、`world/panel.py` 冻结新功能，只许迁移。
- `orolp_core` / `orolp._core` 只准在 `__init__.py`、`kinematics.py`、`collision.py` 出现。
- Domain 禁 import services/app/render/GUI；Adapter 禁 import services。
- 核心包内禁止引入事件总线（ADR-005）；跨模块只走抽象接口 + 注册表 + DTO。
- 业务状态必须枚举/状态机，禁止裸 bool、魔法数字；GUI 对内核对象只读。
- 角度弧度存储、长度 mm、速度 mm/s；用户可见字符串走 i18n 双语。

## 2. 行数阈值

| 类型 | 新文件上限 | 函数上限 |
|---|---|---|
| Domain / Service | 400 | 业务 80 / 纯计算 100 |
| Adapter / GUI 组件 | 300 | 80 |
| DTO/枚举/常量 | 200 | — |
| 测试 | 500 | — |

- 改动使原文件增量 >1/3 → 必须新建文件。
- 存量文件：600 告警 / 1000 硬限；超限文件在 `tools/check_arch_rules.py` 的 `LEGACY_DEBT` 清单登记，头部标 `# TECH_DEBT: 待重构拆分，关联 <条目>`。债务文件只许修 bug + "一次一个类"拆出。
- 巨型 if/else：改状态机 / 枚举字典 / 策略映射。

## 3. 扩展点

新品牌/新协议/新模型库 → 走注册表（`post/` 模板注册、`plugins.py` PluginRegistry、controller_link 协议接口），禁止调用处 if-brand 硬编码。注册表未覆盖的临时扩展点：`# EXTENSION_POINT: <用途>` + 在 `docs/BUILD_QUEUE.md` 留收口条目。

## 4. 收尾自检（每次改动必做）

```bash
python3 tools/check_arch_rules.py   # 退出码必须为 0；WARN 项只许减不许增
```

脚本检查项（H1-H6 + L1）：GUI 归宿 / C++ 边界 / Domain·Adapter 纯净 / **i18n 棘轮（中文字面量超基线即违规）** / **features 键注册一致性** / 行数分档。

- [ ] import 方向无越界（脚本 0 硬违规）
- [ ] 新增用户可见字符串走 i18n zh/en 双键（脚本 H5 拦截硬编码中文）
- [ ] 新功能门控键先在 features.py ALL_FEATURES 注册（脚本 H6 拦截未注册键）
- [ ] 新 GUI 入口注册 `features.py` ALL_FEATURES，尊重 MVP 隐藏清单
- [ ] `pyproject.toml` packages 同步（动了包结构时）
- [ ] `pytest tests/` 全绿；测试只加不减
- [ ] 提交信息 `<area>: <what>`；一次提交一件事
