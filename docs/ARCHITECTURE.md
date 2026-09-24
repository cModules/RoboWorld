# RobotWorld 整体架构设计与长期演进规划

> 版本：v1.0 · 日期：2026-09-21 · 基于仓库实际盘点（非理想化设计）
> 原则：**领域优先，技术其次；每个抽象必须证明自己的复杂度是值得的；优先可逆决策。**

---

## 1. 现状诊断（盘点的客观事实）

| 维度 | 现状 | 结论 |
|------|------|------|
| Python 层 | `python/orolp/` 约 42,000 行，含 post(8 品牌后处理)/world/render/robot_library | **事实主体** |
| C++ 层 | `src/core/` 约 700 行（robot/motion/collision/binding），namespace `orolp` | 存在但近乎闲置：`orolp_core` 为可选 try/except，`kinematics.py` 有纯 Python 重复实现 |
| GUI | `app.py` **单文件 6,895 行**，5 个类（Renderer/GLWidget/Editor/Panel/MainWindow） | 最大技术债 |
| 插件机制 | `orolp/plugins.py`（PluginRegistry + entry_points + API 版本门控） | **设计成熟**，是架构亮点 |
| 测试 | `tests/` 62 文件 / 1,651 个测试函数，纯 assert 风格 | 覆盖广但组织松散 |
| 打包 | pyproject `packages = ["orolp", "orolp.post"]` 漏 render/world/robot_library；入口点 `orolp.cli:main` 指向不存在的 cli.py | **打包已损坏** |
| 发布 | PyInstaller 单可执行（Win/macOS）+ Docker 无头冒烟 | 形态清晰 |

**三个必须先解决的问题**（按序）：① 打包配置漂移 ② app.py 单体 ③ C++/Python 边界名存实亡。

---

## 2. 目标架构（C4 Container 视图）

```
┌────────────────────────────────────────────────────────────────┐
│  Application 层（桌面壳）                                        │
│  app.py → 拆为 python/orolp/app/                                │
│  ├─ mainwindow/   主窗口编排（分域：场景/程序/面板/状态栏）        │
│  ├─ views/        GLWidget、PathCodeEditor、PropertyPanel        │
│  └─ controllers/  用户操作 → 领域服务的薄适配（不含算法）          │
├────────────────────────────────────────────────────────────────┤
│  Service 层（业务用例，纯 Python，可被 GUI/CLI/测试复用）          │
│  planner · process · calibration · twinsync · multirobot        │
│  iomap · pickplace · plc_scl · progdiff · cam                   │
├────────────────────────────────────────────────────────────────┤
│  Domain 层（核心领域，零 GUI/零 IO 依赖）                         │
│  robot(FK/IK/雅可比) · world(单元/工件) · motion(轨迹/规划)       │
│  collision · dynamics · envelope · pose · link                  │
├────────────────────────────────────────────────────────────────┤
│  Adapter 层（可替换边界，全部走注册表/协议）                       │
│  ├─ post/         8 品牌后处理器（Jinja2 模板 + 注册表）          │
│  ├─ controller_link/  真机链路（Socket/SSH，协议适配器接口）      │
│  ├─ robot_library/    URDF/STL 模型加载                          │
│  └─ plugins.py    插件注册表（entry_points，API 版本门控）        │
├────────────────────────────────────────────────────────────────┤
│  Accelerator 边界（pybind11，可选加速，见 ADR-002）               │
│  _orolp_core：碰撞 broadphase / FK 批量 / IK 数值热路径           │
└────────────────────────────────────────────────────────────────┘
```

### 依赖方向铁律（AI 编程与人工开发同守）

```
app → services → domain ← adapters → domain
                  adapters ✗→ app        （适配层禁止反向依赖 GUI）
                  domain   ✗→ services   （领域层禁止依赖业务用例）
                  orolp 核心包 ✗→ PySide6 （GUI 依赖只在 orolp.app 内）
```

---

## 3. 关键架构决策（ADR 摘要）

### ADR-001：Python 为主线，C++ 定位为可选加速层
- **背景**：Python 4.2 万行 vs C++ 700 行，且纯 Python IK/碰撞已在生产使用；Pinocchio/FCL 的 C++ 路径与 scipy 路径功能重复。
- **决策**：不追求"C++ 重写核心"。C++ 仅承接有基准数据证明的热点（批量 FK、碰撞 broadphase），通过 `_orolp_core` 单一 pybind11 边界暴露，Python 侧 `kinematics.py`/`collision.py` 内部做"有 C++ 用 C++、无则纯 Python"的透明降级。对外 API 不感知实现。
- **代价**：放弃 C++ 的性能上限与类型安全收益；换来单一技术栈的团队效率与跨平台构建简单性（PyInstaller 无需处理 Qt6 C++ 编译矩阵）。
- **可逆性**：高。降级接口模式保证随时可以增删 C++ 实现而不动调用方。

### ADR-002：app.py 拆解为 `orolp.app` 包，按"视图/控制器"分域
- **背景**：6,895 行单文件，5 个类互相纠缠，是当前最大可维护性风险；`python/` 手动插入 sys.path 是临时手段。
- **决策**：目标结构 `python/orolp/app/{mainwindow,views,controllers}/`，拆分顺序：Renderer→views、GLWidget→views、PropertyPanel/Editor→views、最后 MainWindow 编排。**每次搬移一个类，测试全绿再提交**，禁止"顺手重构"。
- **代价**：短期搬运成本 + 导入路径变更；换来任何改动的影响范围可预判。
- **注意**：拆完 app.py 后同步修 pyproject `packages`（见 ADR-003）。

### ADR-003：打包即代码——pyproject 与包结构单一事实源
- **背景**：`packages` 列表漏 3 个子包、`cli:main` 指向不存在的文件，wheel 已损坏。
- **决策**：① 立即修复 packages 列表并补齐 CLI 入口（`orolp/cli.py` 实现子命令：`orolp post / world / doctor`）；② 增加 CI 校验：构建 wheel → 干净 venv 安装 → `import orolp` + 冒烟测试，防止再次漂移；③ 后续迁移 setuptools 自动发现（`[tool.setuptools.packages.find]`）。
- **代价**：CI 多 1-2 分钟；换来"pip install orolp"从此可信，插件生态（依赖 entry_points）才有可能成立。

### ADR-004：插件为唯一外部扩展机制，拒绝配置式扩展
- **背景**：`orolp/plugins.py` 已有 PluginRegistry + importlib.metadata 入口点 + API 版本门控，根目录 `plugins/` 目录反而形同虚设。
- **决策**：所有第三方扩展点收敛到插件 API（后处理器、控制器协议、模型库）；根目录 `plugins/` 要么承载真实本地插件示例，要么删除，不留误导。插件 API 版本按主版本兼容（PLUGIN_API_VERSION 模式保留）。
- **代价**：新扩展点必须走注册表协议，比"加个配置项"慢；换来核心包边界永不腐烂。

### ADR-005：模块化单体，不拆微服务
- **背景**：桌面离线编程软件，天然单机、离线、单用户。真机链路（controller_link）是对外唯一网络边界。
- **决策**：架构上就是**带清晰内部分层的单体**（如上 C4 图），不做本地服务进程拆分、不上事件总线。若未来需要数字孪生云同步（twinsync 已有雏形），以独立进程 + 文件/socket 协议对接，不在核心包内嵌服务端。
- **代价**：无法独立扩展子系统（目前无此需求）；换来一个 PyInstaller 包走天下的发布简单性。

---

## 4. 长期演进路线图

### Phase 0（立即，1-2 周）：卫生与止血
- [ ] 修复 pyproject packages 列表 + 实现 `orolp/cli.py`（ADR-003）
- [ ] CI 增加 wheel 构建安装冒烟校验
- [ ] 仓库根清理：build/ dist/ egg-info/ 日志/ __pycache__ 加 .gitignore
- [ ] 落地 `AGENTS.md`（AI 编程规则，见仓库根）

### Phase 1（1-2 月）：GUI 拆解
- [ ] 按 ADR-002 顺序拆 app.py → `orolp.app`（每类一 PR，测试全绿）
- [ ] 消灭 sys.path hack；app.py 保留为 ≤50 行启动壳（兼容老入口）
- [ ] program_panel/planner/twinsync/perf 等千行文件按同一纪律逐步分域

### Phase 2（1-2 月）：领域边界固化
- [ ] domain 层零依赖审查（禁止 import PySide6/scipy.io 等，用 CI import-linter 守护）
- [ ] 碰撞/FK 热点基准测试：数据说话决定 C++ 加速实现哪些（ADR-001 落地）
- [ ] `kinematics.py` 双实现合并为统一降级接口，删除重复代码

### Phase 3（2-3 月）：真机与生态
- [ ] controller_link 从 features 门控转正：协议适配器接口 + 仿真回环测试（虚拟控制器）
- [ ] 插件 API v2：补充生命周期钩子与文档；插件市场目录（本地 json 清单起步）
- [ ] ROS2 optional 依赖与 rclpy 链路验证

### Phase 4（长期）：产品化
- [ ] 数字孪生同步（twinsync）独立进程化
- [ ] 多机器人协同（multirobot）场景编排升级
- [ ] 文档站（readthedocs 已在 pyproject 占位）+ 教程示例矩阵

**每一阶段的准出条件：测试全绿 + wheel 冒烟通过 + 新增文档同步。**

---

## 5. 质量属性与守护手段

| 属性 | 手段 |
|------|------|
| 可维护性 | 依赖方向铁律 + import-linter CI 守护 + 文件行数软上限（600 行告警） |
| 可靠性 | 1,651 个既有测试是资产：拆解期**只加不减**；真机链路必须先过虚拟控制器回环 |
| 可扩展性 | 插件注册表 + 后处理器模板 + controller_link 协议接口，三个可替换边界 |
| 可观测性 | `orolp/debug.py` 已有基础；统一到结构化日志，禁 print（AI 规则 #9） |
| 发布可靠性 | wheel 冒烟 CI + PyInstaller 每版本打包验证（Win/macOS 双平台） |
