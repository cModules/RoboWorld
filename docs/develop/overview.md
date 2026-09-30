# 任务概览：RobotWorld 整体架构设计与开发规则

## 完成内容
1. **代码库盘点**（Explore 代理 + 人工核实；**以下为 2026-09-25 诊断快照**，当前值见 `docs/develop/ARCHITECTURE.md` §1）：Python 层约 4.2 万行为事实主体；C++ 核心仅 ~700 行近乎闲置（try/except 可选加载，kinematics.py 有纯 Python 重复实现）；启动壳单体 6,895 行（现 `main.py` **160 行**，ADR-002 启动壳拆解已完成：`MainWindow` 落地
`app/window/main_window.py`，`app/` 层内部拆解为下一阶段）；pyproject 打包配置漂移（packages 漏 render/world/robot_library，入口点指向不存在的 cli.py）；插件机制设计成熟是亮点。
2. **`docs/develop/ARCHITECTURE.md`**：现状诊断 → C4 目标架构（app · service · domain · adapter 四层 + support 支撑层 + accel 硬件加速层 + 依赖方向铁律）→ **7 条 ADR**（ADR-001 Python 主线+C++ 加速层、ADR-002 启动壳拆解、ADR-003 打包即代码、ADR-004 插件唯一扩展机制、ADR-005 模块化单体拒绝微服务、ADR-006 分层物理化——`core` 拆出顶层 `domain`/`service`/`adapter`、ADR-007 `core` 收窄为纯硬件加速层并新设 `support/`）→ 四阶段演进路线图（Phase 0 卫生止血 → Phase 1 GUI 拆解 → Phase 2 领域边界 → Phase 3 真机与生态 → Phase 4 产品化）→ 质量属性守护手段。
3. **`AGENTS.md`**（仓库根，开发规则入口）：22 条规则分五类——架构边界硬约束（依赖方向/GUI 隔离/领域纯净/C++ 唯一边界）、代码组织（600/1000 行上限、features 门控、注册表扩展）、风格实现（单位约定、i18n、依赖纪律）、测试纪律（只加不减）、提交与仓库卫生，附快速自检清单。

## 关键决策
- 不做 C++ 重写：C++ 定位为有基准数据支撑的可选加速层，透明降级接口。
- 不拆微服务：桌面单机软件保持带清晰内部分层的单体，真机链路是唯一网络边界。
- 首要止血项：修复 pyproject 打包（wheel 目前已损坏）。

## 建议下一步
按 Phase 0 执行：修打包 → CI 冒烟 → .gitignore 清理 → AGENTS.md 团队宣贯。

---

# 2026-09-25 追加：编码规范体系 v2 落地（规则核对 + 补充）

## 核对结论（用户提供的八领域模板 vs 仓库实际）

| 模板内容 | 结论 |
|---|---|
| 八大领域（grasp/market/unit_simulation…） | ❌ 与实际模块不对应，废弃；改为按仓库盘点的归属查表 |
| 四层架构（App/Simulation/Engine/Infra） | ⚠️ 缺 Service 层与 C++ Accelerator 边界，改为五层 |
| 强制事件总线 | ❌ 与 ADR-005 直接冲突，改为接口+注册表+DTO，事件仅限跨进程 |
| 业务文件统一 ≤400 行 | ⚠️ 会判死全部存量（启动壳一度 7000+ 行），改为目标态/过渡态分档治理 |
| EXTENSION_POINT 注释 | ⚠️ 弱于既有注册表机制，改为注册表优先、注释兜底 |
| 领域边界/单向依赖/状态枚举/DTO/自检清单 | ✅ 方向正确，保留并本地化 |

## 本轮交付物

1. **docs/develop/CODING_RULES.md**（v2）：模块归属查表、五层依赖、行数分档、状态/DTO、TECH_DEBT 治理、AI 自检清单、模板映射附录
2. **AGENTS.md**：新增第六节硬性规则 #23-#29 + 附二编码前自检
3. **tools/check_arch_rules.py**：守护脚本（H1 GUI 归宿 · H2 C++ 边界 · H3/H4 分层纯净 · H5 i18n 棘轮 · H6 features 键 · H7 加速层纯净 · H8 支撑层底部性 · L1 行数分档）
4. **项目级技能 robotworld-arch-guard**（.workbuddy/skills/）：团队共享编码前强制自检
5. **robotworld-auto-dev-loop 技能**：挂接 arch-guard 引用

## 首轮扫描实际发现

- robot_library/urdf_builder.py 1217 行超硬限未登记 → 已登记 LEGACY_DEBT；11 个 600~800 行文件同批登记
- 4 个存量 GUI 文件 PySide6 泄漏（controller_panel/program_panel/world.panel/robot_library.panel）→ 冻结新功能
- 文档漂移：C++ 边界实际在加速层包的 __init__.py（非 kinematics/collision 内部）→ 脚本白名单如实登记
- robot.py（Domain）import adapter.plugins 判定为合法 DI 用法（注册表机制），脚本放行

## 后续建议

CI 接入守护脚本做退出码门禁；4 个 GUI 债务文件补 TECH_DEBT 头标注；Phase 1 按"一次一个类"消化 LEGACY_DEBT（只许减不许增）。
