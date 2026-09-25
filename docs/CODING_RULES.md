# RobotWorld 编码规范 v2（落地版）

> 版本：v2.0 · 日期：2026-09-25
> 定位：本文件是 `AGENTS.md` 的**详细版**，与 `docs/ARCHITECTURE.md`（架构依据）、`tools/check_arch_rules.py`（可执行守护）三位一体。
> 关系约定：边界类规则以 `AGENTS.md` 为最终裁决；本文负责归属查表、阈值分档、治理流程与自检清单。
> 来源：在通用"机器人仿真八领域规则模板"基础上，按仓库实际盘点（2026-09-25）本地化修订。

---

## 0. 适用范围

- 适用代码：`python/orolp/`（Python 主体）、`src/core/`（C++ 加速层）、`tests/`、`tools/`
- 适用场景：代码生成、补全、功能迭代、重构、Bug 修复、代码评审、AI 自动开发循环
- 生效方式：AI 编码前必须先过 §8 自检清单；CI / 本地用 `python tools/check_arch_rules.py` 复核

---

## 1. 领域与模块归属（查表规则）

> 通用模板中的"八大领域"（grasp / model_import / virtual_device / unit_simulation / path_planner /
> plc_runtime / hardware_gateway / market）与本仓库实际模块不对应，**以本表为准**。
> 一个文件、类、函数只允许归属一行；跨行协作只能走抽象接口 + DTO（见 §4）。

| 模块（`python/orolp/`） | 所属层 | 职责边界 | 禁止事项 |
|---|---|---|---|
| `robot.py` `pose.py` `link.py` | Domain | 机器人模型、FK/IK、位姿、连杆几何 | 零 GUI、零 IO、不 import services/adapter |
| `world/` | Domain | 场景/单元/工件/动画/放置（`panel.py` 除外，见 §2 注②） | 同上 |
| `motion.py` `collision.py` `dynamics.py` `envelope.py` | Domain | 轨迹、碰撞、动力学、包络 | 同上 |
| `planner.py` `process.py` `pickplace.py` `plc_scl.py` | Service | 路径/工艺/抓放/PLC 业务用例 | 不 import PySide6；不碰底层原生 API |
| `calibration.py` `twinsync.py` `multirobot.py` `progdiff.py` `cam.py` `simulation.py` `sequence.py` `program.py` `project.py` `cell.py` `incremental.py` `compliance.py` `iomap.py` | Service | 各业务用例 | 同上 |
| `post/` | Adapter | 8 品牌后处理（Jinja2 模板 + 注册表） | 无业务语义；禁止 import services |
| `controller_panel.py` `linkstub.py` `sshstub.py` `cloudhttp.py` | Adapter / 集成 | 真机链路、云接口（`cloudhttp.py` 待归位评审） | 同上 |
| `robot_library/` | Adapter | URDF/STL 模型加载、目录、规格 | `panel.py` 是存量 GUI 债务 |
| `plugins.py` | Adapter | 插件注册表（entry_points + API 版本门控） | 唯一外部扩展入口 |
| `render/` | View 支撑 | 网格/图元/URDF 渲染几何 | 只被 GUI 调用，不 import services |
| `features.py` `i18n.py` `debug.py` | App 支撑 | 功能门控、i18n、结构化日志 | 面向用户字符串必须走 i18n |
| `app.py` → 目标 `orolp/app/` | App / GUI | 窗口编排、视图、控制器 | 全项目唯一的 PySide6 归宿（见 §2 注②） |

**新模块归属决策**：动手前先回答——"它是业务规则（Domain）？业务用例（Service）？可替换技术边界（Adapter）？还是界面编排（App）？"答案写进 PR 描述（AGENTS.md #6）。拿不准，停下来问。

---

## 2. 分层与依赖方向（五层，单向无环）

```
App(或olp/app) ──→ Service ──→ Domain ←── Adapter
                                  ↑
Accelerator(_orolp_core, pybind11 可选加速)
```

- **依赖方向**：`app → services → domain`，`adapters → domain`。禁止反向、禁止跨层跳跃。
- **Adapter 不归 Service 管**：适配器只依赖 Domain 抽象，Service 通过注册表/协议调用适配器，不是上下级。
- 注① Accelerator 是**边界**不是层：只有 `kinematics.py` / `collision.py` 内部可 `import _orolp_core`（透明降级），其余任何代码禁止直接 import（AGENTS.md #4）。
- 注② **PySide6 / OpenGL 归宿**：目标态只有 `orolp/app/` 允许 import；存量 `controller_panel.py`、`program_panel.py`、`robot_library/panel.py`、`world/panel.py` 是已登记债务，按 ADR-002 拆解节奏迁入 `orolp/app/views`，迁移完成前冻结新功能（见 §7）。
- 注③ **不引入事件总线**（ADR-005）：模块化单体，跨模块联动用"抽象接口 + 注册表 + DTO 直接调用"；事件/消息仅允许出现在跨进程边界（未来 twinsync 进程化、插件回调、controller_link 协议）。

---

## 3. 行数阈值与分档治理

通用模板的统一 ≤400 行会立刻判死全部存量文件（app.py 7049 行、program_panel 2196 行…），
因此分两档执行：

### 3.1 目标态（新增文件、新增类必须遵守）

| 文件类型 | 上限 | 函数上限 |
|---|---|---|
| Domain 领域文件 | 400 | 业务 80 / 纯计算 100 |
| Service 文件 | 400 | 80 |
| Adapter 文件 | 300 | 80 |
| 基础设施/数学工具 | 250 | 100 |
| GUI 组件（views/panel/editor） | 300 | — |
| 接口/枚举/常量/DTO | 200 | — |
| 单元测试文件 | 500 | — |

- 新增功能使原文件增量超过 1/3 → 必须新建文件，禁止堆叠扩容。
- 禁止巨型 if/else：多分支用状态机、策略映射、枚举字典，禁止裸 switch 瀑布。

### 3.2 过渡态（存量文件，衔接 AGENTS.md #5）

- 存量上限：软 600 行（告警）、硬 1000 行（禁止提交超 1000 行的新改动堆叠）。
- 超过 600 行的存量文件：头部标注 `# TECH_DEBT: 待重构拆分，关联 issue 或 docs/BUILD_QUEUE.md 条目`，进入 §7 治理流程。
- 拆分纪律：**一次 PR 只拆一个类**，测试全绿再提交（ADR-002）。

---

## 4. 跨模块通信

1. 跨领域/跨层调用只允许：**抽象接口、注册表、标准 DTO**。禁止 import 对方内部实现、私有方法。
2. DTO 定义与业务实现文件分离；DTO 文件 ≤200 行；跨模块数据禁止传引擎原生对象引用。
3. 状态变更联动优先"接口回调/注册表订阅"，核心包内**不建事件总线**（ADR-005）。
4. 全局禁止循环依赖；`tools/check_arch_rules.py` 做 import 方向静态检查。

---

## 5. 状态与对象管理

- 所有业务状态用 `Enum` 定义（状态机优先），禁止裸 bool / 魔法数字表达阶段含义。
- 仿真/场景核心对象状态由 Domain/Service 层管理，GUI 只读快照展示；GUI 禁止直接修改内核实体、禁止编写仿真计算逻辑。
- 业务判断归 Service/Domain，物理执行与约束创建归 Adapter（如 PLC 业务判断在 `plc_scl.py`，协议调用在 controller_link 侧）。
- 单位约定沿用 AGENTS.md #12：角度弧度存储（UI 层 `*_deg` 显式命名）、长度 mm、速度 mm/s。

---

## 6. 扩展点规范

> 新硬件接入、新工艺单元开发的完整操作步骤见 **`docs/EXTENSION_GUIDE.md`**（两条标准流水线）。

1. **注册表优先**：新增后处理品牌、控制器协议、模型库、设备适配器，必须走现有注册表（`post/` 模板注册、`plugins.py` PluginRegistry、controller_link 协议接口），禁止调用处 if-brand 硬编码（AGENTS.md #8）。
2. 注册表暂未覆盖的临时扩展点，用注释显式标记：`# EXTENSION_POINT: <用途说明>`，并在 `docs/BUILD_QUEUE.md` 留收口条目。
3. 接口定义与实现分离；禁止预写未验证的冗余实现（接口先行、实现后置）。

---

## 7. 技术债治理

- 头部标记：`# TECH_DEBT: 待重构拆分，关联 <issue/队列条目>`。
- 债务文件**只允许 Bug 修复 + 按"一次一个类"的拆解迁出**；禁止在债务文件里新增业务功能（新增功能必须落在新文件、新层）。
- 存量 GUI 债务（§2 注②四个文件）与千行 Service 文件按 ADR-002 / ARCHITECTURE.md Phase 1 节奏消化。

---

## 8. AI 强制自检清单（每次生成代码前逐条过）

任一不过 → 停止生成，标注违规条目与位置，给出拆分/修正方案（"我打算 X，代价是 Y，替代方案是 Z"，AGENTS.md #22）：

1. 归属：文件/类/函数是否只属于 §1 表中一行？
2. 依赖：是否单向无环？是否越过 `app → services → domain` 方向？
3. 边界：是否直接调用了底层原生 API（绕过 Adapter）或 `_orolp_core`（绕过降级接口）？
4. 规模：新文件是否超 §3.1 阈值？改动文件是否把超限文件推得更大？
5. 混杂：业务逻辑是否与物理计算/协议解析/文件 IO/GUI 渲染混写？
6. GUI：PySide6 是否只在 `orolp/app/` 与已登记债务文件内？GUI 是否只做渲染与指令下发？
7. 状态：状态是否用枚举？是否存在 GUI 改内核状态？
8. 扩展：新品牌/新协议是否走注册表？`EXTENSION_POINT` 标记是否规范？
9. 收尾：features 门控注册了吗？i18n 键补了吗？pyproject packages 同步了吗？测试全绿了吗？

---

## 附录 A：通用模板 → 本仓库映射表

| 通用模板（原文） | 本仓库实际 | 处理结论 |
|---|---|---|
| 八大领域（grasp/market/…） | Domain 8 模块 + Service 13 模块 + Adapter 4 组 | **废弃模板领域名，以 §1 查表为准** |
| 四层 App/Simulation/Engine/Infrastructure | 五层 App/Service/Domain/Adapter + Accelerator 边界 | 模板把 Service 与 Domain 合并、且漏了 C++ 边界，**按 §2 五层执行** |
| 强制事件总线（Skill 4） | ADR-005 明确不上事件总线 | **冲突，改"接口+注册表+DTO"，事件仅限跨进程** |
| 业务文件 ≤400 行 | 存量 8 个 600~2200 行文件 | **分档：目标态 ≤400，过渡态 600/1000 + TECH_DEBT** |
| 前端/内核分离（Skill 6） | 桌面单体，GUI=orolp/app，内核=services/domain | 语义对齐，表述按 §2/§5 本地化 |
| EXTENSION_POINT 注释 | 注册表机制已成熟（ADR-004） | 注册表优先，注释仅兜底（§6） |
| 市场领域 | 仅有 `cloudhttp.py` 雏形 | 不预设立法；成立时走插件/独立包评审 |
