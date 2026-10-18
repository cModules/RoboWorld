# RoboWorld 自动开发队列（BUILD_QUEUE）

> 本文件是「自动开发循环」的执行入口。定时任务每次运行时按此文件工作，完成后通过飞书通知用户。
> 手动触发方式：对 WorkBuddy 说"继续 RoboWorld 队列"即可。


> **归档规则（2026-09-27 起）**：本文件是**活跃队列**，只保留**未完成条目**；
> 已完成的条目按 **ISO 周**归档到 `docs/develop/queue/archive/<年>-W<周>.md`——
> **每周集中归档一次**（活跃文件只保留当前未完成任务，上下文成本不随历史增长）。
> 周末运行 `python tools/archive_queue.py --apply --week YYYY-Www`；历史记录不在本文件里重复记账，
> 要查「做过了什么」请翻对应周归档。

## 归档索引（已完成）

| 周 | 条目 | 文件 |
|---|---|---|
| 2026-W37 | 9 | [`docs/develop/queue/archive/2026-W37.md`](queue/archive/2026-W37.md) |
| 2026-W37及更早 | 18 | [`docs/develop/queue/archive/2026-W37及更早.md`](queue/archive/2026-W37及更早.md) |
| 2026-W38 | 47 | [`docs/develop/queue/archive/2026-W38.md`](queue/archive/2026-W38.md) |
| 2026-W39 | 26 | [`docs/develop/queue/archive/2026-W39.md`](queue/archive/2026-W39.md) |

---

## 🎯 短期目标：发布「抓取工作站仿真编程」能力展示

> 2026-09-12 确定。目的：拿得出手的演示——在 GUI 里一键加载抓取工作站，机器人完成 pick & place 循环动画（含抓取/释放、工件随动、节拍显示），并输出真实品牌机器人代码。
> 现状基线：`examples/pick_and_place.py` 已有 API 级演示（工具系/wobj/链式程序/5 品牌出码），缺 IO 指令、工作站场景、抓取动画、GUI 集成。


### 展示任务（优先于其他 Must 执行）


### 🎯 短期目标（最高优先）：v1.4.0 MVP 发版（macOS + Windows 双包）

> 2026-09-19 用户提出：**尽快发一个 MVP 版本出去，把不成熟的功能先去掉**；既定——①多机保留；②mac + win 双平台交付；③版本 **v1.4.0**（全功能再升 **2.0**）。
> MVP 定位＝**单机/多机各自独立的离线编程闭环**：搭场景 → 示教 → 校验与节拍 → 干涉检查 → 回放（含抓取随动）→ 出品牌代码 → 存 `.core`。这条链路有 `S3/S4/S5` 真示例，是唯一敢对外承诺的。

**Day1 冒出来的两个「绿着的坑」（既有，未修，Day2 处理）**

1. **全量 pytest 会卡在恢复询问**：前面的 GUI 测试把自动保存快照写进了**真实的** `~/.core/autosave`，后续新建 `MainWindow` 的 `showEvent → maybe_prompt_recovery` 弹出模态框等人点——今天全量跑到 99%（`tests/test_ui_layout.py::test_no_tab_page_is_clipped_at_minimum_window`）就停在那儿了。单个文件跑全绿，所以这不是 UI 测试本身的错，而是**测试没有隔离用户目录**。修法取向：根 `tests/conftest.py` 里加 autouse fixture 把 `CORE_CONFIG_DIR` 指到 `tmp_path`（与冒烟脚本同口径）。**这条会直接卡死 CI，Windows/CNB 流水线之前必须先落地。**
2. **`examples/assets/grasping_workstation_calibration.json` 被写回**：跑相关演示会重做手眼标定并把新的 `created` 时间戳写回仓库里的示例文件（`examples/vision_pick_demo.py:69` 的 `CALIB_JSON`）。打包后资源目录通常是只读的，这条有可能在客户机器上变成写失败。

**Day2 待办**：Windows 三坑（`install_pip_deps.ps1` 缺 scipy/jinja2/pyyaml/pydantic → 改 `pip install -e ".[gui]"`；`app.spec` 的 `icon='logo.png'` 在 Windows 不合法 → 平台分支 + 新增 `logo.ico`；`build_win64.ps1` 的「spec 不存在就生成旧 spec」兜底分支会造成缺 `python/` pathex 的空壳包 → 删除并加打包后自检）+ CNB `.cnb.yml`（测试门禁 / macOS 自托管打出 `.app` / Windows 自托管 or GitHub Actions 补位 / `git:release` + `cnbcool/attachments`）+ 发布说明（`MVP 范围与隐藏项清单`、`CORE_FEATURES=full` 开启方式、未签名的两处绕法、2.0 路线）。

> 记账规则照旧。当前已知账：`tests/test_features.py` **15 passed**（新增）；受影响子集 `test_project_gui + test_sequence + test_iomap + test_plugins` + `test_ui_layout` **263 + 12 passed**（无回归）；**全量未跑完**——见上「坑 1」，修好后补记真基线（历史基线 1684 passed + 6 skipped）。


### 🎯 短期目标（新）：GUI 交互打磨（2026-09-19 用户指定，优先于主队列）

> 2026-09-19 用户提出。目标从「跑得通」转向「好用」——面板布局、代码预览子面板、抓取交互、视口操作、动作时序编辑五处。
> 执行顺序建议 **P1 → P5**（P1 改布局，先定，后面几项都在新布局之上做）。

> 记账规则照旧：完成一项勾一项、同步 `docs/develop/ROADMAP.md` §3 与 `README.md` 开发进展表；测试不得引入回归（当前基线见上：1684 passed + 6 skipped）。


- [ ] **P1 面板位置调换：「代码预览」移到左侧，「设备树」移到右侧**（`app.py`：`left_tabs` 约 5442 / `right_tabs` 约 5462）：
  现状：左＝设备树（250–360px），右＝属性 / 代码预览 / 导出打包（260–330px）。
  要点：①两个 `QTabWidget` 的 min/max 宽度要跟着内容换（代码预览需要更宽的可写区，设备树窄一点够用）；②所有「切到某个标签」的调用要跟着改——`_add_component_from_library()`（约 4169）里 `left_tabs.setCurrentWidget(self.scene_panel)`、以及选中对象/生成代码时自动切页的逻辑；③`tools/smoke_gui_demo.py` 与截图脚本里按左右取面板的地方一并复核。
  验收：offscreen GUI 测试断言「代码预览在左、设备树在右」，拖组件入库 / 切页 / 选中同步三条通路仍通。

- [ ] **P2 代码预览下的子面板完善（尤其「PLC 程序逻辑」「机器人子程序」）**（`app.py` `_build_plc_tab()` 约 2507 / `_build_subprogram_tab()` 约 2510，`sub_tabs` 约 2500）：
  现状：两块都能出码，但交互薄弱（用户明确点名这两块）。
  要点：①**PLC 程序逻辑**：与点表/IO 映射（`SignalMap`、`cell.devices[].io_map`）编辑联动、信号增删重命名、按品牌（西门子 SCL 等）出码预览与导出；②**机器人子程序**：子程序命名与参数（工件/工位）、「选中若干示教点 → 生成子程序」的绑定、跨品牌出码复用、与抓取/放置语义对齐（PICK/PLACE 落在子程序里而不是散在主程序）；③两块共同：空态提示、错误可定位（点一下跳到对应示教点/代码行）、en 界面文案、与 v1.1.6 点表校验的问题互通。
  验收：新增 `tests/test_codegen_tabs.py`，覆盖「选点生成子程序 → 出码 → 回读一致」与空态/报错路径。

- [ ] **P3 抓取交互功能**（`domain/world/animation.py` 的 `GraspAnimator`、`program_panel.py` 的 gripper 自动插入、`app.py` 的 `gripperStateChanged` 接线）：
  现状：**只有回放时按程序指令自动吸附/释放**（S3 已交付），没有「人指定抓什么、怎么抓」的交互。
  要点：①在视口/属性面板指定「用哪个工具抓哪个工件」（工具 TCP、工件、抓取位姿），落到既有 PICK/PLACE 语义里；②手动试抓 / 试放并**回写程序**（生成或更新 `gripper()` 指令 + 工件随动关系），让交互与回放、出码共用同一份时序；③抓不住时的诊断要说得清为什么（够不着 / 超负载 / 干涉 / 工件不在可抓范围），而不是静默失败；④与 P2 子程序联动（一次抓取＝一段可复用子程序）。
  验收：新增 `tests/test_grasp_interaction.py`，覆盖「交互指定 → 程序生成 → 回放吸附 → 出码含夹爪指令」。

- [ ] **P4 3D 视口：视图移动与视角切换**（`GLWidget`：`camera_az/el` 约 637、`wheelEvent` 约 2348、`reset_view` 约 6586；View 菜单约 5253）：
  现状只有左键旋转 + 滚轮缩放 + `R` 复位。
  要点：①**平移视图**（中键拖 / Shift+左键拖 / 方向键），并把观察中心移到选中对象（双击对象居中）；②**视角预设**：前/后/左/右/顶/等轴测一键切换 + 数字键快捷键，能回到上一个视角；③聚焦选中对象（Fit to selection）与缩放上下限；④状态栏/提示条显示当前视角与操作提示（左键旋转、中键平移、R 复位）。
  验收：offscreen 测试断言相机参数随预设切换、平移改变观察中心、快捷键接线存在。

- [ ] **P5 设备树「动作时序」编辑器做成可用**（`world/panel.py` `SequenceEditor` 约 688 + `service/sequence.py`）：
  现状是「能存能校验」的最小版本（步骤表 + 泳道画布 + JSON 往返），真编排一批工步时不可用——用户判断「要做大量工作」。
  要点：①泳道（机器人 / 设备）增删并按场景对象自动同步（多机每台一条泳道）；②工步增删改、拖拽排序、复制粘贴、撤销重做；③时间轴缩放与吸附、驻留（`waitTime`）/ 握手信号可视化、与程序 IO 事件对齐；④校验结果**可点击定位**（跳到对应工步并标红）并给出修复建议；⑤与 P2 子程序联动（工步引用子程序名）。
  验收：新增/扩充 `tests/test_sequence_editor.py`，并实测「编排 20+ 工步仍能流畅增删改」。


### 展示任务与主队列的关系

S2 完成后把 v0.3.3 剩余部分（工艺模板库、waitDI 完整语义）留在原任务继续；S3 的碰撞校验依赖 v0.2.1，上线后回补。S6 在 S1 完成后即可开始搭建，之后作为持续维护项：每次任务完成后同步更新并重新发布，链接保持不变。


## 执行规则

1. **取任务**：优先取「🎯 短期目标」区第一个未完成任务（S1→S6 顺序；S6 为持续维护项）；短期目标全部完成后，再按主队列顺序取第一个未完成任务。
2. **实现**：参考 `docs/develop/ROADMAP.md` 对应小节的技术方案；动手前先读相关源码（`core/`、`app.py`）。
3. **测试**：为新增功能编写/更新测试（`tests/`、`python/tests/`），运行 `pytest tests python/tests` 全量通过后才算完成（2026-09-16 基线 1684 passed + 6 skipped，不得引入回归；旧的 1681、1678、1663、1637、1562、1533、1525、1492、1473、1460、1434、1403、1366、1276、1224、1177、1147、1057、1033、1015、1004、974、958/942/926/913/909/885/807/749/622/522/481/430/421/386/363/334/315/301/281/267/263/251/219/137 基线已过时）。
4. **记账**：完成后勾选本文件对应项，同步勾选 `docs/develop/ROADMAP.md` 第 3 节，并把能力写入 `README.md` 开发进展表。
5. **通知**：通过飞书给用户发消息，包含：完成内容摘要、测试结果、改动文件清单、**需要用户反馈/决策的问题**、下一个任务预告。
6. **阻塞处理**：遇到无法自行决策的阻塞（如缺厂商 STEP、缺真机参数），在任务后标注 `[阻塞: 原因]`，跳到下一个任务，并在飞书消息中说明。不空转、不瞎猜。
7. **归档（每轮必做）**：条目勾选完成后，运行 `python tools/archive_queue.py --apply`
   把**已完成**条目按 ISO 周归档到 `docs/develop/queue/archive/<年>-W<周>.md`——**本文件只留
   未完成项**。为什么必须做：本文件是每轮必读的执行入口，已完成条目若一直堆积，
   上下文成本会随时间线性增长（2026-09-27 首次归档时已达 234KB / 最长单行 17KB，
   归档后活跃文件 45KB）。脚本幂等：已归档的不会重复写入，且会校验条目数守恒。


## 任务队列


### Must（决定软件"能用"）


- [ ] **v0.2.3 5 品牌真机 CAD 接入** `[阻塞: 等用户把官方 STEP 放入 assets/vendor/<brand>/step/]`：工具 `tools/build_vendor_urdf.py` 已就绪（--probe 验轴系 → --all 生成 URDF+网格+预览），逐品牌复制 test_vendor_<brand>.py 套件（仿 test_vendor_jaka.py）。汇川仅有后处理无 URDF，CAD 到位后一并补**（OCP 8 兼容这一步已于 2026-09-15 由 v1.2.1 做完**：新增 `adapter/ocpcompat.py` 收拢版本相关符号名，`tools/build_vendor_urdf.py` 与 `tools/vendor_cad_import.py` 均已改走它，链路重新可用；并且 `tests/test_vendor_step_pipeline.py` 用**合成装配体**把整条链跑成了可执行覆盖——解锁时只需把官方 STEP 放进 `assets/vendor/<brand>/step/`、跑 `--probe` 确认轴系与组件映射，**不再有代码前置工作**。）

### Should（决定软件"好用"）


### ADR-002：启动壳拆解为**顶层 `app/`**（Phase 1，一次一个类，测试全绿再提交）

> **2026-09-26 目标态修订**：GUI 从 `core.app` 提升为**顶层包 `app/`**——桌面应用要与引擎
> 分开独立演进（依赖方向 `app → core` 单向，引擎禁止 import `app`）。详见
> `docs/develop/ARCHITECTURE.md` ADR-002 的修订块。


- [ ] **第 3 步**：`PathCodeEditor`（1251 行）→ 需先定拆分方案（代码模型 / 高亮 / gutter / 面板）
- [ ] **第 4 步**：`GLWidget`（1809 行）→ 需**先定**"相机/拾取 / 绘制 / 交互事件"的子模块拆分方案再迁（顺带带走只有它在用的 `_segments_for_radius` 与 `_TESSELLATION_*`）
- [ ] **第 5 步**：`MainWindow` → `app/mainwindow/`（按场景/程序/面板/状态栏分域），app.py 收敛为 ≤50 行启动壳
- [ ] **收尾**：图标化（路径示教工具栏 + 代码行 gutter 图标）落在已迁出的 `app/` 内
- [ ] **遗留技术债**：`RobotRenderer.load_urdf` 的 `print` 应改结构化日志（AGENTS.md #11），但 `service/debug.py` 当前只有 `DebugSession`/`Deviation`、**没有日志 API**，需先补 API 再全项目收敛


### 静态检查扫出的既有缺陷（2026-09-25 装上 black/flake8 后首扫）


- [ ] **全项目格式化（必须单独一次提交，别混进功能 PR）**：`app.py` 60 条 flake8（28×W293 空行含空格 / 12×E702 一行多语句 / 6×E501 / 5×F401 未使用导入 / 1×F841）+ black diff 约 2000 行；`core/` 257 条。建议顺序：先清 F401/F841 → 再跑 black 全量（此时 diff 最大，单独评审）→ 最后 E501/E702/W293。
- [ ] **把 black/flake8 接进 CI**：`pyproject` 的 `dev` extra 早就声明了这两个工具，但既没装进 `.venv`、CI 也没跑，F821 才潜伏到现在。建议 CI 至少对 `core/` 与 `tests/` 执行 `black --check` + `flake8 --max-line-length 100`。


### P0：目录重整与打包止血（2026-09-25，用户授权分批实施）

按「先止血、后动业务代码」的顺序做完四批，每批独立可验证：


- [ ] **Git LFS 历史迁移（需明确授权）—— 2026-09-26 核实后更正了本条的描述**：
  事实是**规则从未真正接管**这些文件——1041 个 STL（约 40MB）一直以**普通 git 对象**入库
  （`git lfs ls-files --all` 为 0；索引 blob 与磁盘同体积，例：`ur16e/link_5_00.stl` 的
  blob `b36a8ab…` = 9684 字节）。当年"已转 Git LFS / 新提交会走 LFS"的说法不成立。
  **规则现已注释保留**（见 `.gitattributes`，路径随目录重整指向 `assets/models/**`）。
  为什么不能简单地把规则指过去：这些文件的索引内容是真文件，而 LFS clean filter 会产出
  指针（实测同一文件 `git hash-object --path=…` 得 `721354f1…` ≠ 索引的 `b36a8ab…`）→
  1041 个 STL 会立刻被判成"已修改"，且任一次 `git add` 都会把真内容换成约 130 字节的
  指针，而 LFS 服务端并没有这些对象 —— **新克隆会检出坏指针，等同丢数据**。
  真要启用需重写历史（会改动全部提交、影响所有已克隆副本）：
  `git lfs migrate import --include="assets/models/**/*.stl,assets/models/**/*.STL"`
  40MB 体量下收益有限，建议等一次专门的窗口做。
- [ ] **Docker 冒烟与 GUI 依赖**：`Dockerfile` 的 CMD 跑全量 `pytest tests/` 但不装 PySide6，而 `app.py` 顶层 import PySide6 → 该冒烟必然失败（历史遗留，未被 CI 引用）。要么镜像加 `.[gui]` + libGL，要么 CMD 限定为不依赖 GUI 的子集。
- [ ] **`examples/gui_demo/python_viewer.py` 的重复 `RobotRenderer`**：与 `app/views/robot_renderer.py` 是两套实现；收敛需连带改 `tools/smoke_gui_demo.py` 与 `tools/capture_grasp_screenshots.py`，属独立任务（P0-4 已降级处理，未动）。


### 分层物理化：`core` 拆出 Service / Domain / Adapter（2026-09-26 前置件）


- [ ] **17 条跨层依赖按根因分三类清**（与分层搬迁同批做，逐条从棘轮清单删除）：
  1. **`Program` 是领域模型却定义在 Service**（9 条：8 个品牌后处理器 + `world/animation.py`）——后处理器与动画都要读程序表示，说明程序表示该下移 Domain；
  2. **常量 / 协议放错层**（6 条）：`SINGULARITY_COND` · `ENVELOPE_TOLERANCE_MM`（planner→domain）、`Trajectory`（simulation→domain，轨迹本就是领域概念）、`CloudTransport`/`normalize_key`（twinsync→domain/infra，Adapter 只该依赖抽象）；
  3. **反向工厂调用**（1 条）：`robot.py` 的 11 个 `moveX()` 方法反向构造 `ProgramBuilder`，需单独一 PR 把流式 API 移到 Service（动公开 API）。
- [ ] **收尾（第三轮）**：`core/__init__.py` 门面是否长期保留待评（它让 `core` 依赖全部三层，
  与"内核在最底层"的理想相反）。
- [ ] **写文档/工具的坑（本轮实际踩到，值得记）**：用脚本批量改注释与 docstring 时，
  ① ``ast`` 的 ``col_offset`` 是**该行的 UTF-8 字节偏移**，中文行按字符下标算会整体错位、
  把下一行文本吃掉（曾一次写坏 21 个文件）；
  ② 不要拿 ``src[start]`` 当引号去重新拼接，会把三引号 docstring 降级成单引号；
  ③ 正确做法是 ``ast.get_source_segment`` 取整段原文（保留引号与前缀）、只改片段内部，
  并**先 ``ast.parse`` 自检通过再落盘**；注释另用 ``tokenize`` 的 COMMENT token 处理。


### core 收窄为纯硬件加速层（2026-09-26 第三轮，ADR-007）


- [ ] **待拍板（本轮刻意不动）**：`[tool.core.features]` 段名、`CORE_FEATURES` / `CORE_LANG` /
  `CORE_CONFIG_DIR` 环境变量、`~/.core` 配置目录——它们是**产品级数据契约**，与层名解耦；
  改名（如 `ROBOTWORLD_*` + `~/.robotworld`）需单独一轮 + 数据迁移，否则用户的语言选择 /
  最近打开会静默丢失。
- [ ] **GPU 代码的下一站**：拆 `main.py` 的 `GLWidget`(1845 行) 时，把 **Qt-free 的 GPU 部分**
  （GLSL 着色器、VBO/VAO 布局、绘制助手、相机矩阵）落进 `core/gpu/`，Qt 部件留 `app/views/`
  ——这才是 ADR-007 口径下"GPU 代码进 core"的形态（现在 core 放的是 GPU 数据准备，绘制在
  app 是 Qt 边界所致）。
- [ ] **`core_native` 仍需一次端到端验证**：本机没编译过 C++，
  `PYBIND11_MODULE(core_native)` / `OUTPUT_NAME` / Python 侧 import 名三者的一致性，
  只能靠 CI 的 `build-cpp` 外加一步 `import` 冒烟确认。


### 加速层更名 `accel`（2026-09-26 第四轮，ADR-008）


- [ ] **待拍板：产品名那批**（与层结构解耦，单独一轮 + 数据迁移）：分发名 `core`（PyPI 撞名）、
  CLI `core doctor/post/world`、`~/.core/`、`.core` 工程扩展名、`CORE_*` 环境变量、
  `[tool.core.features]`、schema `core.*/1`。**动它们会静默丢用户设置**（语言/最近打开/自动保存）。
- [ ] **`accel_native` 的端到端验证仍欠**：本机没编译过 C++，`PYBIND11_MODULE` ↔ `OUTPUT_NAME`
  ↔ Python import 名三者一致只能靠 CI 的 `build-cpp` + 一步 `import` 冒烟证明（建议本次一并加）。
- [ ] **GPU 代码的下一站**：拆 `main.py` 的 `GLWidget`(1845 行) 时，把 Qt-free 的 GPU 部分
  （GLSL、VBO/VAO 布局、绘制助手、相机矩阵）落进 `accel/gpu/`，Qt 部件留 `app/views/`。


### main.py 拆解（ADR-002 Phase 1，按步推进）

**现状（AST 实测）**：`main.py` 6193 行 = 355 行前导（17 个 `_HAS_*` 降级块 + 16 个 `GL_*` 常量）
+ 三个巨型类（`GLWidget` 1875 行/79 方法 · `PathCodeEditor` 1236/69 · `MainWindow` 2604/88）
+ 崩溃处理与 `main()`。**三类之间几乎零耦合**（`PathCodeEditor` 零依赖；`GLWidget` 只要
`_segments_for_radius` + 14 个 `GL_*`；`MainWindow` 只要那两个类 + `DEFAULT_ROBOT_KEYS`/`_REPO_ROOT`）
→ 类级拆分是**纯搬运**，风险不在代码逻辑，而在两处"静默门禁"（CJK 基线、hiddenimports）。


- [ ] **测试覆盖缺口（本轮暴露）**：`MainWindow.validate_program` / `generate_robot_code` /
  `save_path_code` / `load_path_code` 这些**菜单动作没有直接测试**——`tests/test_trajectory_editor*.py`
  里的 `panel.validate_program()` 测的是 **`ProgramPanel` 自己的**同名方法，与窗口那层无关。
  搬移本身有结构测试兜底（归属模块 / MRO / 无遗漏名 / 门禁登记），但"点这个菜单会不会炸"
  没人守。建议补一个 offscreen 的**窗口级动作冒烟**（从空输入且安全返回的入口入手，如
  `_show_validation_summary({})`、`on_clear_requested()`），比逐个子系统写断言便宜得多。
- [ ] **发现的小瑕疵（未修，本轮刻意保持"纯搬运"）**：`generate_robot_code` 的 docstring 开头是
  `与tr("保存路径代码")不同`——早期一次 i18n 批量替换把 docstring 里的"与保存路径代码不同"
  改坏了（docstring 不该含 `tr(...)`）。纯文案瑕疵、无功能影响，下次动这个文件时顺手改。
- [x] **Step 12（B 组首个落地）**：`app/views/program_panel.py` 类内拆解完成（2026-10-18）——
  拆出 8 个模块（`program_state`/`program_ui`/`program_robot`/`program_playback`/`program_grasp`/
  `program_export`/`program_editor`/`program_validate`），原文件 2202 → **534 行**（< 600 软限，
  已摘除 TECH_DEBT 并从 LEGACY_DEBT 移除），公开方法签名全保留（stub 转发至模块级函数），
  1852 测试全绿，6 条守门测试已建（`tests/test_app_views_program_panel.py`）。
  `gl_widget.py`(1992) / `path_editor.py`(1298) / `controller_panel.py`(683) 仍待拆。
  - ⚠️ **第二个事故——测试的"补丁目标"没跟着走（段错误）**：`tests/test_app_degraded_paths.py`
    原本写 `monkeypatch.setattr(main, "gl", _NullGL())` 再调 `main.GLWidget._draw_link_recursive`。
    类搬进 `app/views/gl_widget.py` 后，它查的是**自己模块**的 `gl` → **补丁成了空操作** →
    真去调 OpenGL → 在没有上下文的测试环境里**整轮测试段错误**（退出码 139，连失败清单都拿不到）。
    修法：补丁打在**定义处**（`setattr(gl_widget, "gl", …)`）。全仓只此一处（已 grep 确认）。
    → 固化为守门测试 **⑥ `test_no_test_patches_globals_on_entry_shell`**：禁止对入口脚本
    （`main`/`rw`）的模块全局打补丁。**对 Step 3 的教训：搬类要连"谁在给旧地址打补丁"一起查——
    这类失效不会失败、只会**静默变成真调用**，后果取决于被调用的是什么（这里是 GL）。**
- [ ] **Step 3**：`MainWindow` → `app/mainwindow/window.py`（先整体搬；它要 import 上面两个类
  + 8 个面板 + 7 个降级块）。
- [ ] **Step 4**：`main.py` 瘦身到 ≈90 行（只剩 `sys.path` 前导 + 崩溃处理 + `main()`）。
- [ ] **Step 5+（真正降体积的那部分）**：类内继续拆——`GLWidget` 绘制层 19 方法/680 行、
  `MainWindow` 事件同步 28/759 与工程 IO 14/515、`app/views/program_panel.py` 2198 行。
- [ ] **暂记的重复**：三个可选依赖降级块（world / cell / pickplace）在 `main.py` 与新视图各留一份
  ——等 `MainWindow` 也搬进 `app/mainwindow/` 后，应收敛到一处共享模块。


### 结构评审（2026-09-26，对着现状逐项核实；不是猜测）

**已核实为健康**（写下来避免下次重复怀疑）：
- 分层已落地：`core/` 4210 行/7 文件 · `domain/` 10212/18 · `service/` 15402/21 ·
  `adapter/` 9098/21 · `app/` 4523/10 · `main.py` 6219；依赖方向真实无环。
- `adapter/robot_library/` 留内核**是对的**（被 Adapter 的 `robot_library` 与 App 共用，不是"只被 GUI 调用"）。
- `adapter/plugins.py` 里的 PySide6 只是**注释提及**（解释为何 `import core` 不能变重），H1 合规。
- `pyproject` 的 `core = ["assets/cells/*.json"]` 是**前瞻声明**（注释已写明 P2 才把 `assets/cells`
  收进包），当前匹配不到任何文件属预期，不是 bug。

**待办（本轮新发现，按性价比排序）**：

- [ ] **门禁覆盖扩到 `tools/` 与 `tests/`**（最高性价比）：守护脚本 `iter_py_files()` 只扫
  `main.py` + 五个层目录，flake8/black 的命令行同样不含这两个目录——于是
  `tools/build_web_site.py` **6179 行**（全仓第二大文件）、`tools/render_preview.py` 694、
  `tests/` **17 个文件 >500 行**（§3.1 测试上限）**全部在门禁之外**。
  与"H3/H4 曾是死检查"同型：**覆盖不足 → 债不可见**。修法同上次：扩范围 + 棘轮锁存量
  （工具脚本软限 600、测试文件上限 500），不炸 CI。
- [ ] `tests/`（70 文件 / 28168 行）**平铺**：建议镜像分层 `tests/{domain,service,adapter,app}/`，
  收益是"改动某层只跑该层测试"、CI 可分片并行。
- [ ] **静态检查配置落文件**：flake8 参数只写在 CI 命令行（本地裸跑结果不同）→ 加 `.flake8`；
  `pyproject` 有 `[tool.mypy]` 但 **CI/pre-commit 从不跑 mypy**（配了等于"有类型检查"的错觉）→
  加 informational 步骤或删配置；无 `[tool.pytest.ini_options]`（缺 testpaths/markers）。
- [ ] `docs/develop/overview.md` 索引陈旧（已修）：曾写"5 条 ADR"（实际 6，缺 ADR-006）、
  "main.py 6,895 行"（实际 6219）——**索引型文档错了会误导**，改动后要顺手核。
- [ ] `docs/develop/DOMAIN_ARCHITECTURE.svg` 仍是"7 业务域 + 内核"的**概念方案**，与已落地的分层口径
  不一致 → 改造，或显式标注"概念方案（未落地）"。
- [ ] **C++ 加速层零端到端验证**：CI 有 `build-cpp` 作业（cmake + make + ctest ✅），但
  **没有任何一步 import 产出的模块** → `PYBIND11_MODULE(core_native)` / `OUTPUT_NAME` /
  Python 侧 import 名三者不一致时，CI 全绿而用户侧 ImportError。build-cpp 补"构建后
  `import core_native` + 调一个函数"的冒烟（本机未编译过 C++，此风险尚未排除）。
- [ ] **数据契约无单一事实源**：schema 串（`core.project/1` · `core.parts/1` · `core.autosave/1`）
  分散在 `service/project.py` · `domain/world/parts_library.py` · `app/session.py`，**没有迁移
  函数、没有兼容性测试** → 建 `core/schema.py`（集中常量 + `migrate(data)`）+ `docs/DATA_CONTRACTS.md`。
- [ ] **设备适配器通路不存在**：`docs/manual/EXTENSION_GUIDE.md` 流水线 A 要求 `adapter/devices/` 目录
  与 `IDevice` 协议，两者都还没建——真接硬件前必须先立这个扩展点。
- [ ] `scripts/` 只剩 2 个 Windows 安装脚本，与 `tools/`（15 个开发工具）职责重叠 →
  合并，或在 CONTRIBUTING 写清"scripts = 用户侧 / tools = 开发侧"。
- [ ] `core/__init__.py` 门面（447 行）让**内核依赖全部三层**（与"内核在最底层"矛盾）——
  它是兼容层，54 个文件在用旧写法 → ADR-006 补**退出条件**（如：新代码禁用
  `core.<上层模块>`，用棘轮统计调用点只许减，至 v2.0 移除 re-export）。
- [ ] **33 个文件超 600 行软限**（其中 **13 个 >1000 行**；4 个 >1500：`main.py` 6219 ·
  `service/planner.py` 1920 · `service/perf.py` 1830 · `service/twinsync.py` 1828）。
  **分层搬迁不解决体积**——搬完还是这么大；按 ARCHITECTURE Phase 1/2 节奏每轮挑 1-2 个拆。
  已完成：`app/views/program_panel.py` 2202→534（2026-10-18 类内拆 8 模块，见 Step 12 B 组）、
  `support/i18n.py` 1479→118（词条表外置 `support/i18n_catalog.py`，i18n 逻辑零改动）。


## 新增功能想法（评审区）

本区只记录想法与评审结论，不自动执行。只有用户明确确认后，才转入上方 `Must / Should / Could` 正式队列。

### 评审反馈格式

用户可直接回复：`确认：<编号>，进入 <Must|Should|Could>，范围是 <...>`、`补充信息：<...>`、`暂缓：<原因>` 或 `拒绝：<原因>`。AI 负责补齐范围、依赖、验收标准并更新队列。

状态仅使用：`待评审`、`需补充信息`、`已确认`、`暂缓`、`拒绝`、`已转正式任务`。

### F-001：本地工程分发包（Marketplace P0）

- 评审状态：**待评审**
- 用户价值：让场景、零件、自定义设备、插件和 `.core` 工程可以安全交付与复用。
- 当前基础：`Project` 容错与保真、`program_fingerprint`、插件发现、现有 `.core` 工程格式。
- 建议范围：导出/导入带 manifest 的本地分发包；记录作者、版本、依赖、指纹和许可；导入时校验干涉、越限、IO 映射。
- 明确不包含：在线目录、账号、支付、分成、云端审核。
- 架构归属：Service / Adapter，GUI 仅提供导入导出入口。
- 外部依赖：无硬依赖；厂商 URDF/STL 版权由内容提供者负责。
- 验收标准：可往返导出导入；缺依赖可诊断；指纹不一致拒绝导入；未知字段保真；失败不破坏原工程。
- 建议结论：优先评审是否进入 `Should`，后续 Marketplace P1–P3 另行评审。

### F-002：路径示教外设属性可视化

- 评审状态：**待评审**
- 用户价值：让用户一眼识别路径点对应的夹爪、焊枪、喷涂或其他外设动作。
- 当前基础：`PointType`、`gripper_action_for()`、`_line_map` 已提供语义与代码行映射。
- 建议范围：路径示教页工具栏图标、代码行 gutter 图标、统一语义到图标的映射表；图标资源纳入打包与 i18n。
- 明确不包含：改变控制器代码语义；导出代码中的外设注释默认关闭。
- 架构归属：App GUI；语义映射通过现有 Service 接口读取。
- 外部依赖：需先完成 `PathCodeEditor` / `MainWindow` 拆分。
- 验收标准：工具栏与下拉状态一致；PICK/PLACE/普通轨迹图标可区分；图标行数与 `movej` 行数一致；中英文界面和冻结包资源通过检查。
- 建议结论：拆分完成后评审是否进入 `Should`。

### F-003：导出打包点位编辑并入路径示教

- 评审状态：**待评审**
- 用户想法：将当前「导出打包」中的点位编辑能力，未来合并到「路径示教」中，并统一采用图标按键形式。
- 用户价值：在同一个路径示教工作区完成点位创建、调整和导出，减少在「导出打包」与「路径示教」之间切换；图标按键可降低文字按钮密度并提升操作识别度。
- 当前基础：已有 GUI 轨迹编辑器、示教点编辑能力，以及「导出打包」面板；已有图标化方向记录在 GUI 收尾任务中。
- 建议范围：将点位编辑入口迁移到路径示教；保留新增、删除、上移、下移、编辑位姿、运动类型切换等能力；以图标按钮承载常用操作，并提供 tooltip、快捷键或无障碍文本；导出打包仅保留格式、品牌和文件输出相关操作。
- 交互原则：路径示教负责编辑路径语义，导出打包负责生成和交付文件；编辑点位后实时同步代码预览、校验结果和导出内容；危险操作（删除、清空）需明确确认或支持撤销。
- 明确不包含：本轮不改变机器人程序语义、不新增控制器指令、不直接执行真机运动。
- 架构归属：App GUI；点位与程序变更复用现有 Service / `ProgramPanel` 接口，图标资源纳入打包与 i18n。
- 外部依赖：需结合 `PathCodeEditor` / `MainWindow` 拆分，以及现有左右面板位置调整一起设计，避免重复迁移入口。
- 验收标准：点位编辑仅有一个主入口；路径示教内可完成全部原有编辑操作；图标有 tooltip 与中英文文案；编辑结果同步代码预览、校验、回放和导出；冻结包包含图标资源；无图标字体或资源缺失时仍有可诊断的降级表现。
- 建议结论：先完成交互方案评审，再决定是否进入 `Should`，实施时优先做入口合并和图标工具栏，随后清理导出打包中的重复入口。

### F-004：`waitTime` 驻留段时间轴（历史候选）

- 评审状态：**已完成**（v1.1.3）
- 处理结果：已纳入正式队列并交付，后续只保留在归档中，不再作为新想法重复评审。

### F-005：外部输入阻塞项（不进入自动开发）

- 评审状态：**暂缓**
- 包含：5 品牌官方 STEP、真机实测、真实云端点、厂商原生标定协议。
- 处理方式：收到外部输入或协议决策后，再拆成正式任务；在此之前只保留阻塞记录，不自动尝试实现。

### 评审转队列规则

1. `待评审` 想法不执行、不计入当前开发任务。
2. 用户确认后，补齐编号、范围、依赖、架构归属、验收标准和优先级。
3. 将条目复制到 `Must / Should / Could`，标记为 `已转正式任务`；如需产品路线同步，再更新 `docs/develop/ROADMAP.md`。
4. 功能交付并通过测试后，写入 `README.md` 开发进展表，再按周归档到 `docs/develop/queue/archive/<年>-W<周>.md`。
5. 未确认的 Marketplace P1–P3、定价、账号、支付、审核和托管服务不得自动开发。
