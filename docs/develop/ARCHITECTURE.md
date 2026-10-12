# RoboWorld 整体架构设计与长期演进规划

> 版本：v1.0 · 日期：2026-09-21 · 基于仓库实际盘点（非理想化设计）
> 原则：**领域优先，技术其次；每个抽象必须证明自己的复杂度是值得的；优先可逆决策。**

---

## 1. 现状诊断（盘点的客观事实）

| 维度 | 现状 | 结论 |
|------|------|------|
| Python 层 | 约 4.9 万行，分五处：`domain/`(1.0 万) · `service/`(1.5 万) · `adapter/`(0.9 万) · `app/`(0.5 万) · `support/` | **事实主体** |
| C++ 层 | **`accel/src/`** 约 560 行（robot/motion/collision/binding 的 `.cpp/.h`），namespace `accel` | 存在但近乎闲置：`accel_native` 为可选 try/except，`domain/kinematics.py` 有纯 Python 重复实现 |
| GUI | `app.py` **单文件 6,895 行**，5 个类（Renderer/GLWidget/Editor/Panel/MainWindow） | 最大技术债 |
| 插件机制 | `adapter/plugins.py`（PluginRegistry + entry_points + API 版本门控） | **设计成熟**，是架构亮点 |
| 测试 | `tests/` 62 文件 / 1,651 个测试函数，纯 assert 风格 | 覆盖广但组织松散 |
| 打包 | pyproject `packages = ["core", "core.post"]` 漏 render/world/robot_library；入口点 `app.cli:main` 指向不存在的 cli.py | **打包已损坏** |
| 发布 | PyInstaller 单可执行（Win/macOS/Linux）+ 三端安装器（Inno Setup / dmg / XDG 桌面条目+MIME）+ Docker 无头冒烟 | 形态清晰 |

**三个必须先解决的问题**（按序）：① 打包配置漂移 ② app.py 单体 ③ C++/Python 边界名存实亡。

---

## 2. 目标架构（C4 Container 视图）

```
┌────────────────────────────────────────────────────────────────┐
│  Application 层（桌面壳）                                        │
│  main.py + app/（2026-09-26 起 GUI 与引擎分顶层包）               │
│  ├─ cli.py · session.py   命令行入口、桌面会话持久化              │
│  ├─ mainwindow/   主窗口编排（分域：场景/程序/面板/状态栏）        │
│  ├─ views/        GLWidget、PathCodeEditor、PropertyPanel        │
│  └─ controllers/  用户操作 → 领域服务的薄适配（不含算法）          │
├────────────────────────────────────────────────────────────────┤
│  Service 层（业务用例，纯 Python，可被 GUI/CLI/测试复用）          │
│  service/  planner · process · calibration · twinsync           │
│            multirobot · iomap · pickplace · plc_scl · progdiff  │
│            cam · simulation · sequence · program · project      │
│            cell · incremental · compliance · debug（在线调试）    │
│            perf（性能度量）· project_templates（工程模板）        │
├────────────────────────────────────────────────────────────────┤
│  Domain 层（核心领域，零 GUI/零 IO 依赖）                         │
│  domain/   robot(FK/IK/雅可比) · kinematics · world(单元/工件)    │
│            collision · dynamics · envelope · pose · link         │
├────────────────────────────────────────────────────────────────┤
│  Adapter 层（可替换边界，全部走注册表/协议）                       │
│  adapter/  ├─ post/         8 品牌后处理器（Jinja2 + 注册表）      │
│            ├─ robot_library/  URDF/STL 模型加载                   │
│            ├─ linkstub · sshstub · cloudhttp  真机/云链路         │
│            ├─ ocpcompat      OCP/CAD 内核版本防腐层               │
│            └─ plugins.py    插件注册表（entry_points + 版本门控）  │
├────────────────────────────────────────────────────────────────┤
│  Support 层（跨层共享支撑，无业务语义，自身不依赖任何层）           │
│  support/  i18n（国际化设施：tr / 语言检测与持久化）              │
│            被 domain/service/adapter/app 共同依赖                 │
├────────────────────────────────────────────────────────────────┤
│  Core = 纯硬件加速层（只放加速代码，守护 H7 白名单）                │
│  accel/src/   C++ 实现：binding / collision / motion / robot      │
│  accel/__init__.py 唯一 C++ 边界（加载 accel_native，失败透明降级）│
│                    + 过渡态兼容门面（惰性，core.Robot 等旧写法）    │
│  core_native（pybind11 产物）：碰撞 broadphase / FK 批量 / IK 热路径│
└────────────────────────────────────────────────────────────────┘
```

### 依赖方向铁律（所有开发参与者共同遵守）

```
app → service → domain ← adapter → domain
                 adapter ✗→ app        （适配层禁止反向依赖 GUI）
                 adapter ✗→ service    （适配层只依赖抽象，不调业务用例）
                 domain  ✗→ service    （领域层禁止依赖业务用例）
                 support ✗→ 任何层      （支撑层被各层依赖，自己不依赖谁）
                 core    ✗→ 任何上层、✗→ PySide6（GUI 只在 app/ 内）
```

> 判据一句话：**底层不得 import 上层**。2026-09-26 走过三步：
> ① 第二轮归位——`cli`（依赖 adapter+domain）、`perf`/`project_templates`（依赖 service+domain）、
> `debug`（依赖 service）因这条被判出内核，分别归 App / Service；
> ② 引擎拆三层——`domain` / `service` / `adapter` 上移为顶层包；
> ③ **`core` 收窄为纯硬件加速层**（ADR-007）——留在里面的 `i18n`（被三层真实 import）
> 进新设的 `support/`，`features` 进 `app/`，`render/` 的 URDF 解析与网格/几何原语
> 进 `adapter/robot_library/`。至此加速层只剩 C++ 与边界——2026-09-26 当晚它由
> `core/` 更名 `accel/` 并下线兼容门面，见 **ADR-008**；口径由守护 H7 钉住。

---

## 3. 关键架构决策（ADR 摘要）

### ADR-001：Python 为主线，C++ 定位为可选加速层
- **背景**：Python 4.2 万行 vs C++ 700 行，且纯 Python IK/碰撞已在生产使用；Pinocchio/FCL 的 C++ 路径与 scipy 路径功能重复。
- **决策**：不追求"C++ 重写核心"。C++ 仅承接有基准数据证明的热点（批量 FK、碰撞 broadphase），通过 `_core_native` 单一 pybind11 边界暴露，Python 侧 `kinematics.py`/`collision.py` 内部做"有 C++ 用 C++、无则纯 Python"的透明降级。对外 API 不感知实现。
- **代价**：放弃 C++ 的性能上限与类型安全收益；换来单一技术栈的团队效率与跨平台构建简单性（PyInstaller 无需处理 Qt6 C++ 编译矩阵）。
- **可逆性**：高。降级接口模式保证随时可以增删 C++ 实现而不动调用方。

### ADR-002：app.py 拆解为 `core.app` 包，按"视图/控制器"分域
- **背景**：6,895 行单文件，5 个类互相纠缠，是当前最大可维护性风险；`python/` 手动插入 sys.path 是临时手段。
- **决策**：目标结构 `python/app/{mainwindow,views,controllers}/`，拆分顺序：Renderer→views、GLWidget→views、PropertyPanel/Editor→views、最后 MainWindow 编排。**每次搬移一个类，测试全绿再提交**，禁止"顺手重构"。
- **代价**：短期搬运成本 + 导入路径变更；换来任何改动的影响范围可预判。
- **注意**：拆完 app.py 后同步修 pyproject `packages`（见 ADR-003）。

> **推进记录（2026-09-26）**：**顺序微调**——`PathCodeEditor` 先于 `GLWidget` 搬。
> 理由：它对本文件其它类**零耦合**（AST 实测：既不依赖任何类，也不依赖任何模块级常量），
> 最适合当"搬迁机制"的试点——一次性验证「新模块自带降级块 + CJK 基线重映射 +
> `app.spec.hiddenimports`」这三件最容易漏的事。
> 已完成：Renderer ✅ · PropertyPanel/Editor ✅ · **PathCodeEditor ✅ → `app/views/path_editor.py`**
> · **GLWidget ✅ → `app/views/gl_widget.py`**（连带 15 个 `GL_*` 常量、`MIME_TYPE`、
> `_TESSELLATION_*` 与 `_segments_for_radius`；`main.py` 累计 6193 → **3071 行**；
> 入口 re-export 保留，既有"搬迁不断链"约定）
> 待办：`MainWindow` → `app/mainwindow/`；最后 `main.py` 瘦身。
>
> **搬迁"漏搬名字"的坑**：`GLWidget` 那步漏搬了模块级函数 `_segments_for_radius`，
> 而当时**全部测试仍全绿**（该路径未被覆盖）。现已固化为守门测试
> `test_module_needs_no_names_from_entry_shell`（`tests/test_app_views_*.py`）：
> "新模块用到 ∩ 入口脚本提供 − 新模块自己定义" 非空即失败。

> **修订（2026-09-26）：GUI 从 `core.app` 提升为顶层包 `app/`。**
> 背景：桌面应用需要一个**能与引擎分开独立演进**的落点（桌面功能开发不想和引擎发布节奏耦合）。
> 决策：目标态定为**两个顶层包**——`app/`（桌面应用：`main.py` 启动壳 + `views/` +
> 待建的 `mainwindow/`、`controllers/`）与 `domain/`·`service/`·`adapter/`（引擎，零 GUI）。依赖方向
> `app → core` 单向，`core` 不得 import `app`。
> 已完成：`python/app/` → 顶层 `app/`；启动壳 `app.py` → `main.py`
> （**同名遮蔽**：同目录下 `app/` 包会遮蔽 `app.py` 模块，`import app` 会静默换对象）。
> 未完成：GLWidget → PathCodeEditor → MainWindow 仍待按"一次一个类"迁入 `app/`
> （见 `docs/develop/BUILD_QUEUE.md`「ADR-002」）。
> **守护脚本同步**：`tools/check_arch_rules.py` 的扫描范围已扩到顶层 `app/` 与
> `main.py`——不改这里，守护会"继续报 0 违规但实际漏扫整个 GUI"（比报错更危险）。

### ADR-003：打包即代码——pyproject 与包结构单一事实源
- **背景**：`packages` 列表漏 3 个子包、`cli:main` 指向不存在的文件，wheel 已损坏。
- **决策**：① 立即修复 packages 列表并补齐 CLI 入口（`app/cli.py` 实现子命令：`core post / world / doctor`）；② 增加 CI 校验：构建 wheel → 干净 venv 安装 → `import core` + 冒烟测试，防止再次漂移；③ 后续迁移 setuptools 自动发现（`[tool.setuptools.packages.find]`）。
- **代价**：CI 多 1-2 分钟；换来"pip install core"从此可信，插件生态（依赖 entry_points）才有可能成立。

### ADR-004：插件为唯一外部扩展机制，拒绝配置式扩展
- **背景**：`adapter/plugins.py` 已有 PluginRegistry + importlib.metadata 入口点 + API 版本门控，根目录 `plugins/` 目录反而形同虚设。
- **决策**：所有第三方扩展点收敛到插件 API（后处理器、控制器协议、模型库）；根目录 `plugins/` 要么承载真实本地插件示例，要么删除，不留误导。插件 API 版本按主版本兼容（PLUGIN_API_VERSION 模式保留）。
- **代价**：新扩展点必须走注册表协议，比"加个配置项"慢；换来核心包边界永不腐烂。

### ADR-005：模块化单体，不拆微服务
- **背景**：桌面离线编程软件，天然单机、离线、单用户。真机链路（controller_link）是对外唯一网络边界。
- **决策**：架构上就是**带清晰内部分层的单体**（如上 C4 图），不做本地服务进程拆分、不上事件总线。若未来需要数字孪生云同步（twinsync 已有雏形），以独立进程 + 文件/socket 协议对接，不在核心包内嵌服务端。
- **代价**：无法独立扩展子系统（目前无此需求）；换来一个 PyInstaller 包走天下的发布简单性。

### ADR-006：分层物理化——`core` 拆出顶层 `domain` / `service` / `adapter`
- **背景**：分层此前只存在于**文档与守护脚本的手工文件名表**里，文件本身平铺在 `core/` 根下。这带来两个后果：①表与目录两处维护，必然漂移；②守护脚本 H3（Domain 禁依赖 Service）与 H4（Adapter 禁依赖 Service）拿**裸模块名**去比**带 `.py` 的文件名表**，比较恒为假——两条规则**从未生效**，17 条真实跨层依赖被静默放过（见下"代价"）。
- **决策**：把三层提升为**顶层包**（`app/` 同级），**目录即分层**：
  - `domain/`（原 robot/pose/link/collision/dynamics/envelope + world/）
  - `service/`（原 planner/program/project/calibration/twinsync/… 共 17 个模块）
  - `adapter/`（原 post/ robot_library/ plugins/linkstub/sshstub/cloudhttp）
  - `core/` 收敛为**内核**（`i18n` · `features` · `render/` · C++ 边界 · `__init__.py` 门面）
    ——**当日第三轮已再收窄为纯硬件加速层，见 ADR-007**
  - 守卫脚本删除两张手工表，改为**目录前缀判定**；公共 API 由 `core/__init__.py` 惰性门面（PEP 562）继续提供，`core.Robot` / `core.world` / `from core import Pose` 等写法不变。
- **第二轮（2026-09-26 同日）**：`core/` 剩余 9 个文件按判据「**内核不得 import 任何上层**」各归其位——
  `kinematics.py` → `domain/`（纯 FK/IK/雅可比，零依赖）；`debug.py` · `perf.py` · `project_templates.py` → `service/`（前者顶层就 `import service.simulation`，后两者编排 domain+service 用例）；
  `ocpcompat.py` → `adapter/`（OCP/CAD 内核的版本防腐层）；`cli.py` · `session.py` → `app/`；两个存量 Qt 面板 `controller_panel.py` · `program_panel.py` → `app/views/`（**ADR-002 的 GUI 债务就此清空**）。
  留在内核的三个模块共同点是**零上层依赖**：`i18n`（被全部 6 个层、31 处引用）· `features`（零内部依赖）· `render/`（被 Adapter 的 `robot_library` 与 App 共用，因此**不能**归 App——否则 Adapter→App 反向依赖）。
  验证：1752 passed / 12 skipped；守卫 0 硬违规 / 17 警告；wheel 五包齐全并过干净 venv 冒烟与 `core doctor`。
- **代价**：①`core/__init__.py` 成为"依赖全部三层"的门面，且**必须惰性**（eager 会与层内 `from support.i18n import tr` 形成循环导入，实测炸过）；②三个新顶层名会进入 wheel 的顶层命名空间；③17 条存量跨层依赖进入棘轮清单（只许减不许增），其中 9 条同一根因——**`Program` 是领域模型却定义在 Service**，8 个品牌后处理器与 `world/animation` 都得读它，修法是把它下移 `domain/`。
- **不做的事**：不把三层做成独立发行物（ADR-005 是模块化单体）；`core/__init__.py` 门面是否长期保留待评——它让 `core` 依赖全部三层，与"底层不依赖上层"的理想相反（→ 见 ADR-007）。

### ADR-007：`core` 收窄为**纯硬件加速层**，新设 `support/` 支撑层
- **背景**：ADR-006 之后 `core/` 仍装着三样非加速的东西——`i18n`(1538 行) · `features`(293) · `render/`(1881)。
  它们是"暂时没处去的支撑"，不是硬件加速。口径（2026-09-26）：**`core` 中只放硬件加速代码，
  包括 GPU、C++ 实现等**。这条口径若只写在文档里，下次还会有人顺手往 `core` 塞东西
  （这三样就是这么进去的）——所以要变成**可执行的门禁**。
- **决策**（落点全部按 **AST 实测的真实消费者**定，不按感觉）：
  - 加速层 = `src/`（C++ 实现）+ `__init__.py`（唯一 C++ 边界；兼容门面**当晚即由
    ADR-008 下线**，因为"内部改名"与"对外兼容"是两件事，一起做会拖住改名）。
    **守护 H7 白名单强制**：该目录下出现任何其它 `.py` 判硬违规（目录名见 ADR-008）。
  - `i18n` → **新设顶层 `support/`**（跨层共享支撑层）。归属依据：它在 domain 5 处 /
    service 7 处 / adapter 8 处被真实 import（错误与提示消息也要本地化），放任何一层都会
    造成反向依赖；**守护 H8** 禁止 `support/` import 任何层。
  - `features` → **`app/features.py`**：真实消费者只有 `main.py` / `app/cli.py` /
    `app/views/program_panel.py`（+ 已迁走的 GUI 债务面板）——"界面功能展示开关"本就属 App。
  - `core/render/` → **`adapter/robot_library/{urdf_model,mesh,primitives}.py`**：
    `primitives`（几何原语 + STL 写出＝**模型构建**）的消费者只有 `robot_library`；
    `urdf_model`（URDF **加载**）归此与 `urdf_builder` 同族，**顺带消掉"core 里解析 URDF、
    adapter 里构建 URDF"的双通路**；`mesh`（`MeshData` 容器）被两者共用，必须跟 primitives 走，
    否则 adapter 要反向 import app。**渲染代码本来就在 app**（`main.py` 的 `GLWidget` +
    `app/views/robot_renderer.py`），`core/render/` 里**一行 OpenGL 都没有**——它只是 GPU
    数据的 CPU 侧准备（`MeshData` → 顶点数组）。GPU 绘制归 App 是 Qt 边界决定的（H1）。
  - 旧写法 `core.i18n` / `core.features` / `core.render` **下线**：不静默转发（那只会让人以为
    还是老结构），而是报错并直接给出新地址（当时的 `_MOVED_MODULES`；门面已于
    ADR-008 整体下线，改为 import 期直接报 `ModuleNotFoundError`）。
- **代价**：①三个旧写法失效（调用点已全量改写，无第三方在用）；②`support` 成为新的顶层名
  进 wheel 与 `top_level.txt`；③`[tool.core.features]` 段名、`CORE_FEATURES` / `CORE_LANG`
  环境变量、`~/.core` 配置目录**保持不动**——它们是**产品级数据契约**，与层名解耦，
  改名要单独一轮 + 数据迁移（已记入 BUILD_QUEUE 待拍板）。
- **验证**：1753 passed / 12 skipped；守卫 0 硬违规（**H7/H8 做了反向验证**：塞违规文件即判
  硬违规、删掉即恢复，`tests/test_arch_guard.py` 钉住）；wheel 六包齐全 + 干净 venv 冒烟 +
  `core doctor`；GUI 正常启动。

### ADR-008：加速层更名 `core/` → `accel/`，兼容门面整体下线
- **背景**：ADR-007 之后加速层已是纯 C++/边界，但**目录名还在说"core"**——它读起来像"核心/主干"，
  而真实内容是**可选加速**（没编译 C++ 时 `has_accel = False`）。同一个词还在别处承担**产品级契约**：
  PyPI 分发名 `core`、CLI `core doctor`、`~/.core/` 配置目录、`.core` 工程扩展名、
  `CORE_*` 环境变量、schema `core.project/1`。**目录名与产品名撞词**，正是"新人看不懂、
  老手说错话"的根源。
- **决策**：
  - 目录 `core/` → **`accel/`**（与 ADR-001「C++ 定位为**可选加速层**」、CODING_RULES §2 注①
    「**Accelerator** 是边界不是层」同源）；C++ 模块 `core_native` → **`accel_native`**
    （CMake target / `OUTPUT_NAME` / `PYBIND11_MODULE` / Python import 四处同步），
    `namespace core` → `namespace accel`。
  - **兼容门面整体下线**：`core.Robot` / `from core import Pose` / `core.world` 一律不再可用，
    69 个调用点（多在 `tests/` 与 `examples/`）改写为 `from domain.robot import Robot` 等；
    随之作废的还有 5 个"包根可导出"测试（它们的存在意义就是锁门面）。
    理由：门面当年是为"改名不让调用方重新学布局"，但它让**最底层反向依赖全部上层**，
    且兜不住点号形式；仓库尚未对外发布，此刻下线成本最低。
  - **产品级契约一律不动**（与目录改名解耦）：分发名 `core`、CLI `core`、`~/.core/`、`.core`、
    `CORE_*`、`[tool.core.features]`、schema `core.*/1`——它们承载**用户数据与既有脚本**，
    改名要单独一轮 + 数据迁移（待办见 BUILD_QUEUE）。
  - 顺带结清两处**陈年错名**：`CMakeLists.txt` 的 `project(openrobolp VERSION 0.1.0)`
    → `project(robotworld)`（版本不再在此重复）；`pyproject.toml` 的 `[project.urls]`
    仍指向 `your-org/openrobolp`（**未改，属产品名那批**）。
  - **版本号单一来源**：`support/__init__.py` 的 `__version__` 字面量；`pyproject.toml` 用
    `dynamic = ["version"]` + `version = {attr = "support.__version__"}` 派生，`app.spec` 直接
    import 它。此前四处手写（pyproject / app.spec / 包内 / 窗口标题），v1.0.1 就漏改过 pyproject。
- **代价**：①旧写法作废（无第三方在用，但 `docs`/示例里的旧句子要跟着改）；②`accel_native`
  改名后**端到端验证只能等 CI 的 `build-cpp`**——本机没编译过 C++，`cmake` 能发现 target 名不一致，
  但 `PYBIND11_MODULE` ↔ `OUTPUT_NAME` 的一致性只有"构建后 import 一次"才能证明；
  ③`CORE_*` 等产品契约与 `accel/` 并存，短期看着像"两套名字"——这是**有意的**：一个说层结构，
  一个说产品。
- **验证**：见 `docs/develop/BUILD_QUEUE.md`「加速层更名 `accel`」条目。

---

## 4. 长期演进路线图

### Phase 0（立即，1-2 周）：卫生与止血
- [x] 修复 pyproject packages 列表 + 实现 `app/cli.py`（ADR-003）——**2026-09-25 完成**：
  `packages` 改为 `[tool.setuptools.packages.find]`（where=python / include=core*）自动发现，
  不再手写（手写曾漏 render/world/robot_library）；新增 `app/cli.py` 实现
  `core doctor / post / world` 三个只读诊断子命令（i18n 双语，36 条键）。
  验证：构建 wheel → 干净 venv 安装 → `import core, domain.world, adapter.robot_library,
  adapter.robot_library` 与 `core doctor` 全部通过（2026-09-26 归位后路径已更新）。
- [x] CI 增加 wheel 构建安装冒烟校验——**2026-09-25 完成**：新增 `wheel-smoke` 作业
  （build wheel → 干净 venv 安装 → import 冒烟 + `core doctor`）；同时修掉
  `build-python` 的历史缺陷：pytest 不再传不存在的 `python/tests/`、装 `.[gui,dev]`
  （此前缺 PySide6，GUI 回归等于没跑）、lint 扩到根目录 `app.py`、新增
  `tools/check_arch_rules.py` 门禁与 F821/E9 硬门禁。
- [x] 仓库根清理：build/ dist/ egg-info/ 日志/ __pycache__ 加 .gitignore——已满足；
  **2026-09-25 追加目录重整**：`app.spec`/`logo.*`/`build_win64.ps1` → `packaging/`，
  安装脚本 → `scripts/`，散落文档 → `docs/`，根目录手工脚本 `test_simple.py` → `tests/`
  （正规化为真测试）；删除死代码 `src/gui/`（13 个从未构建的 Qt6 C++ 文件）与
  `plugins/`（仅 README，ADR-004 要求不留误导）。
  ⚠️ **更正（2026-09-26 核实）**：同一句里当年写的"`models/**/*.stl`（1041 个）转 Git LFS"
  **并未真正生效**——这些 STL 一直以普通 git 对象入库（`git lfs ls-files` 为 0、索引 blob
  与磁盘同体积）。目录重整后模型库在 `assets/models/`，LFS 规则暂以注释保留，原因与启用
  方式见 `.gitattributes` 与 `docs/develop/BUILD_QUEUE.md`。
- [x] 落地 `AGENTS.md`（开发规则入口，见仓库根）+ 详细版 `docs/develop/CODING_RULES.md`
  + 可执行守护 `tools/check_arch_rules.py`

### Phase 1（1-2 月）：GUI 拆解
- [x] 按 ADR-002 顺序拆启动壳 → 顶层 `app/`（一次搬一组，测试全绿再提交）——**启动壳已完成**：
  2026-09-25 第 1～2 步（`RobotRenderer`、`DevicePropertyPanel` 迁出）；
  2026-09-26 GUI 包提升为顶层 `app/`、启动壳 `app.py` → `main.py`（ADR-002 修订）；
  `PathCodeEditor` → `app/views/path_editor.py`、`GLWidget` → `app/views/gl_widget.py`（均整类搬移）；
  `MainWindow`（2604 行 / 88 方法）**按方法组拆**为 8 个 mixin + 骨架 → 新建 `app/window/`
  （`PluginCacheMixin` / `ControllerLinkMixin` / `ProgramOpsMixin` / `RobotSceneMixin` /
  `PlaybackMixin` / `MultiRobotSyncMixin` / `ProjectSessionMixin` / `MiscActionsMixin`，
  方法体一字未改），第 10 步 `MainWindow` 整体落地 `app/window/main_window.py`；
  **启动壳 3043 → 160 行**（只剩 `sys.path` 前导 + 崩溃处理 + `main()` + 历史调用面），
  可选依赖降级块收敛到 `app/window/capabilities.py`（33 个 try 块 → 15 个）。
  逐个进度见 `docs/develop/BUILD_QUEUE.md`「main.py 拆解」小节
- [ ] **下一阶段（B 组）**：`app/` 层内部继续拆——`views/program_panel.py`(2193) ·
  `views/gl_widget.py`(1992) · `views/path_editor.py`(1298) · `views/controller_panel.py`(683)；
  合计 6166 行、占 `app/` 的 63%。方案（各组的方法聚类、依赖方向、执行顺序）见
  `docs/develop/BUILD_QUEUE.md`「B 组」小节
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
| 可观测性 | `service/debug.py` 已有基础；统一到结构化日志，禁 print（AI 规则 #9） |
| 发布可靠性 | wheel 冒烟 CI + PyInstaller 每版本打包验证（Win/macOS 双平台） |
