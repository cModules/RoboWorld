# spec — 产品需求与开发计划

本目录集中存放两类文档，便于产品与研发对齐：

| 子目录 | 内容 | 说明 |
|---|---|---|
| [`prd/`](prd/) | **产品需求书**（PRD） | 描述"要做什么、为什么做、做到什么算完成" |
| [`plan/`](plan/) | **开发计划**（Plan） | 描述"怎么做、分几期、每项任务的验收标准" |

## 与既有文档的分工

| 文档 | 定位 |
|---|---|
| `docs/ROADMAP.md` | **长期**特性规划（v0.2 / v0.3 / v1.0），按月~季度演进 |
| `spec/prd/*` | 单个特性的**产品需求书**，偏"需求与验收" |
| `spec/plan/*` | 单个特性的**实施计划**，偏"任务拆解与排期" |
| `README.md` | 已交付能力清单 |

一句话：`ROADMAP` 管方向，`spec` 管落地。

## 文件命名约定

- PRD：`prd/<英文短名>.md`，如 `prd/ui-architecture.md`
- 计划：`plan/<英文短名>.md`，如 `plan/ui-refactor.md`
- 模板：`_TEMPLATE.md`（下划线开头，排在前面）
- 一篇文档只讲一件事，避免"大杂烩"需求书

## 状态标记

在文档头部用统一字段标注状态，便于快速筛选：

| 状态 | 含义 |
|---|---|
| `草稿` | 正在写，内容可能变 |
| `评审中` | 待确认 |
| `已确认` | 需求冻结，可进入开发 |
| `实施中` | 开发中 |
| `已完成` | 已交付并验证 |
| `已废弃` | 不再执行（保留以备追溯） |

## 归档规则

开发计划由 CodeBuddy 生成在 `.codebuddy/plans/`，属过程性目录；**执行完成后归档到
`spec/plan/`**，成为可追溯的项目资产。

- **判定依据**：只看 plan 文件 frontmatter 里 `todos[].status`，**不看文件名**。
  所有条目处于终态（`completed` / `cancelled`）即视为完成。
  文件名里的「(未完成)」是人工标记，可能与真实状态不符，不作依据。
- **归档动作**：移动到 `spec/plan/<name>.md`（去掉文件名末尾 8 位 hash），
  并自动更新下面「开发计划」索引表。**仓库内只保留一份**，避免两处内容漂移。
- **触发方式**：**计划完成时立即执行一次**归档脚本（不设定时任务）。

```bash
python tools/archive_plans.py --dry-run   # 预览，不改动文件
python tools/archive_plans.py             # 正式归档
```

脚本幂等：已归档且内容一致的会跳过；同名但内容不同的会加 hash 后缀保留，**绝不覆盖**。
未完成的 plan 留在 `.codebuddy/plans/` 并打印剩余项数。

## 索引

### 产品需求书

| 文档 | 主题 | 状态 |
|---|---|---|
| [`prd/ui-architecture.md`](prd/ui-architecture.md) | 界面架构（精简版） | 实施中 |

### 开发计划

| 文档 | 主题 | 状态 |
|---|---|---|
| [`plan/ui-refactor-and-macos-app.md`](plan/ui-refactor-and-macos-app.md) | 界面重构 + macOS 应用化 | 实施中 |
| [`plan/device-axis-and-joint-panel.md`](plan/device-axis-and-joint-panel.md) | 为变位机等多轴设备补齐几何上的回转自由度，并在右… | 已完成 |
| [`plan/dock-icon-and-app-bundle.md`](plan/dock-icon-and-app-bundle.md) | 让 RobotWorld 在 macOS 下显示… | 已完成 |
| [`plan/fix-viewport-rightclick-pick.md`](plan/fix-viewport-rightclick-pick.md) | 修复视口右键拾取“无反应”：根因是 _view_… | 已完成 |
| [`plan/improve-z-axis-gizmo.md`](plan/improve-z-axis-gizmo.md) | 修复 Z 轴手柄难以选中：扩大命中区（竖条 9→… | 已完成 |
| [`plan/viewport-gizmo-transform.md`](plan/viewport-gizmo-transform.md) | 右键按住选中场景对象后，在对象原点弹出变换坐标系… | 已完成 |
| [`plan/右面板对象属性动态编辑与高亮.md`](plan/右面板对象属性动态编辑与高亮.md) | 选中对象后：视口亮黄描边高亮（修正现有高亮色被覆… | 已完成 |
