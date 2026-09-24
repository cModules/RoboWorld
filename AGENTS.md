# AGENTS.md — RobotWorld AI 编程规则

> 本文件约束所有 AI 编程助手（Claude Code / Cursor / Copilot 等）在本仓库的行为。
> 违反边界类规则（#1-#4）的代码必须回滚，不允许"先合入再改"。
> 架构依据见 `docs/ARCHITECTURE.md`。

## 一、架构边界（硬性，违反即回滚）

1. **依赖方向**：`app → services → domain`，`adapters → domain`。禁止反向：domain 不得 import services/app；适配层不得 import GUI；`orolp` 核心包（`python/orolp/` 除 `orolp/app/` 外）**禁止 import PySide6/OpenGL**。
2. **GUI 逻辑只进 `orolp/app/`**：视图（GLWidget/渲染/面板/编辑器）、窗口编排、快捷键、样式表一律放 `python/orolp/app/`，算法与业务逻辑一律放 services/domain 层。
3. **领域层纯净**：`robot/world/motion/collision/dynamics/envelope` 等领域模块不做文件 IO、不弹对话框、不打印 UI 消息；交互通过返回值与异常传递。
4. **C++ 边界唯一**：Python 侧只允许通过 `kinematics.py` / `collision.py` 内部的降级接口使用 `_orolp_core`，任何新代码不得直接 `import _orolp_core`。新增 C++ 实现前必须先给出基准数据（基准脚本 + 数字），无数据不动 C++。

## 二、代码组织

5. **文件行数软上限 600 行**（告警线），硬上限 1000 行。新文件超 1000 行禁止提交；改既有巨型文件（app.py、planner.py、perf.py 等）时，改到哪类就把该类拆到对应模块，**一次 PR 只拆一个类**。
6. **新模块先问归属**：动手前按架构分层判断该放哪层，在回复/PR 描述中写明"本改动位于 X 层，因为……"。拿不准就先在 issue/对话中确认。
7. **打包同步**：新增/移动/删除 `python/orolp/` 下的包或模块时，必须同步检查 `pyproject.toml` 的 `packages`/`package-data` 与入口点，并确认 `python -c "import orolp"` 在干净安装下可用。
8. **扩展点走注册表**：新增后处理器品牌、控制器协议、模型库，必须走现有注册表机制（`post/` 模板注册、`plugins.py` PluginRegistry、controller_link 协议接口），禁止在调用处写 if-brand 分支硬编码。
9. **功能门控**：新增用户可见入口（菜单/面板/按钮）必须注册到 `python/orolp/features.py` 的 ALL_FEATURES 并尊重 `[tool.orolp.features]` 的 MVP 隐藏清单，禁止绕过 features 门控直接展示。

## 三、代码风格与实现

10. **风格**：black（line-length=100）+ flake8 + mypy（配置见 pyproject）。类型注解：公共 API 必须标注，内部尽力。
11. **禁止 print**：使用 `orolp/debug.py` 的结构化日志；错误信息必须包含"出了什么问题 + 用户该做什么"。
12. **单位约定**：角度一律弧度存储（UI 层负责度/弧度转换并显式命名 `*_deg`），长度 mm，速度 mm/s。变量命名必须能看出单位。
13. **禁硬编码路径**：不写绝对路径、不写 `sys.path.insert` hack；资源经 `assets/`、`models/` 相对包路径解析。
14. **依赖纪律**：新增第三方依赖需在 PR 中说明理由与替代方案；GUI 相关新依赖只能进 `[project.optional-dependencies].gui`。禁止引入网络框架/重量级运行时（见 ADR-005 单体决策）。
15. **中文注释与 i18n**：面向用户的字符串必须走 `i18n.py` 双语机制（zh/en 键都要加），不得写死中文串在代码逻辑里。

## 四、测试

16. **只加不减**：既有 1,651 个测试是拆解与重构的安全网，禁止删除或跳过（skip 需在 PR 中说明原因与期限）。
17. **新测试**：pytest 风格（`test_*.py`、`assert` + 必要时 `pytest.raises`），放 `tests/` 平铺目录，文件名与被测模块对应（如 `test_collision_broadphase.py`）。
18. **改动准出**：任何 PR 完成 = `pytest tests/` 全绿 + 涉及打包时 wheel 冒烟通过。GUI 改动需在 PR 描述附截图或说明手动验证路径。

## 五、提交与仓库卫生

19. **提交信息**：`<area>: <what>`（如 `post: 增加汇川 IR 系列模板`、`app: 拆分 GLWidget 到 views`），一次提交一件事。
20. **构建产物不进库**：`build/ dist/ *.egg-info/ __pycache__/ *.log` 禁止提交；发现已入库的产物只报告不主动删除（留给人工确认）。
21. **文档同步**：改公共 API、包结构、features 清单、打包方式时，必须同步更新 `docs/ARCHITECTURE.md` 对应章节或 README，文档与代码不许漂移。
22. **不确定就问**：AI 遇到归属不清、会破坏既有测试、需要删代码 >100 行的情况，先停下来给出方案与权衡（"我打算 X，代价是 Y，替代方案是 Z"），不要猜。

## 附：快速自检清单（每次改动后）

- [ ] import 没有越过依赖方向？
- [ ] 新入口注册 features 了吗？
- [ ] 注册表/模板机制用了吗（没有硬编码品牌分支）？
- [ ] 用户可见字符串走 i18n 了吗？
- [ ] pyproject packages 同步了吗？
- [ ] 测试全绿 + 只加不减？
- [ ] 文件 <1000 行 / 公共 API 有类型注解？
