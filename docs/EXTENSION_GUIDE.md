# RobotWorld 扩展开发指南：新硬件接入 & 新工艺单元

> 版本 v1.0 · 2026-09-25 · 配套 `docs/CODING_RULES.md`（§1 归属查表、§6 扩展点）与 `docs/DOMAIN_ARCHITECTURE.svg`
> 核心原则：**领域归属按业务产出价值判定；扩展点一律走注册表；接口先行、实现后置。**
> 两条流水线共用同一套收尾门禁：features 门控 + i18n + 守护脚本 0 违规 + 测试全绿。

---

## 流水线 A：新硬件接入（相机 / 3D 扫描仪 / 力传感器 / 夹爪 / 光源…）

### A0. 归属判定（三问，30 秒）

| 问 | 答 | 落位 |
|---|---|---|
| SDK/驱动/协议/触发 IO？ | 是 → Adapter | `adapters/devices/`（新建设备适配器注册表） |
| 产出什么业务价值？ | 扫描建工件 → workcell；标定/视觉引导 → process；检测 → 预留 vision 域（EXTENSION_POINT） | 对应业务域的 service |
| 要无硬件验收？ | 是 → simulation | `domains/simulation/` 虚拟桩 |

同一台设备会同时出现在三处——这是设计使然，不是重复。

### A1. 定义设备 DTO（共享内核）

- 新数据类型（点云/图像帧/深度图）进 `kernel` DTO 文件，**≤200 行**，无任何业务语义：
  ```python
  # kernel/dto_device.py
  @dataclass(frozen=True)
  class PointCloudDTO: points: np.ndarray; frame: str; meta: dict
  ```
- 已有类型（如 `DevicePoseDTO`）直接复用，禁止传厂商 SDK 原生对象。

### A2. 实现设备适配器（≤300 行，零业务）

- 实现统一设备接口（首次接入时先定义接口，放 `adapters/devices/base.py`）：
  ```python
  class IDevice(Protocol):
      def connect(self) -> None: ...
      def capture(self) -> DeviceFrameDTO: ...   # 只返回 DTO
      def disconnect(self) -> None: ...
  ```
- 一台设备一个文件：`adapters/devices/camera_hik.py`、`scanner_lnx.py`；厂商 SDK 只允许出现在这里。
- 错误信息必须包含"出了什么问题 + 用户该做什么"（AGENTS.md #11），用 `debug.py` 结构化日志，禁 print。

### A3. 注册到设备注册表

- 走 `plugins.py` PluginRegistry（与 post/ 后处理、controller_link 协议同一机制）：
  ```python
  registry.register("camera.hik", factory="orolp.adapters.devices.camera_hik:HikCamera",
                    api_version=PLUGIN_API_VERSION)
  ```
- 禁止调用处 if-brand 硬编码；取设备永远 `registry.create(...)`。

### A4. 虚拟桩 + 回环验收（simulation 域）

- 写 `domains/simulation/virtual_camera.py`（或 virtual_scanner），实现同一个 IDevice，产出合成数据。
- 参照 `linkstub.py` / `sshstub.py` 思路：**无硬件也能跑通"采集 → 业务用例 → 落库/出码"全流程**，作为设备测试夹具长期保留。

### A5. 业务用例接线（对应业务域）

- 在业务域写 service（≤400 行），只消费 DTO、只调注册表，禁止 import 适配器内部：
  - 3D 扫描 → `domains/workcell/`：点云入库为工件/零件（与 `parts_library` 联动）
  - 相机 → `domains/process/`：手眼标定（扩展 `calibration`）、视觉引导抓取（联动 `pickplace`）
- 跨域联动走接口 + DTO，不建事件总线（ADR-005）。

### A6. GUI 入口（可选，有界面才做）

- 面板/视图放 `orolp/app/views/`，只读快照 + 指令下发，禁业务逻辑。
- 注册 `features.py` ALL_FEATURES，尊重 MVP 隐藏清单；面板文件 ≤300 行。
- 用户可见字符串 i18n zh/en 双键。

### A7. 收尾门禁（每次必做）

```bash
python3 tools/check_arch_rules.py          # 必须 0 硬违规
$PY -m pytest tests python/tests -q        # 全绿，只加不减
```
- 测试：`tests/test_device_<name>.py`（≤500 行）+ 虚拟桩回环用例。
- 打包同步：动了包结构必须改 `pyproject.toml` packages（ADR-003）。
- 记账：`docs/BUILD_QUEUE.md` 留条目；提交 `<area>: <what>`。

---

## 流水线 B：新工艺单元开发（焊 → 铆接/涂胶/拧紧/检测…）

### B0. 归属判定

| 内容 | 落位 |
|---|---|
| 工艺参数、模板、约束规则 | process 域 |
| 新的物理动作/运动语义 | robot / workcell 域（经接口扩展，不反向） |
| 轨迹/节拍/程序校验 | programming 域（planner/program 接口） |
| 新控制器品牌出码 | post 适配器（模板注册） |
| 时序编排 + IO 联动 | programming(sequence) + plc 域（iomap/plc_scl） |

### B1. 工艺数据与状态

- 工艺参数用 dataclass/pydantic DTO；阶段状态一律 `Enum` + 状态机，禁止裸 bool（AGENTS.md #24）。
- 模板文件 ≤400 行，超了按"参数/规则/校验"拆文件。

### B2. 工艺模板实现（process 域）

- 参照 `process.py` 既有焊/涂模板的结构与注册方式，**策略注册，不是 if-branch**：
  ```python
  PROCESS_REGISTRY.register("riveting", RivetingProcess)
  ```
- 业务判断归 process 域，物理执行语义经接口下发，不直接操作底层。

### B3. 轨迹与节拍（programming 域接口）

- 调 planner 现有接口生成轨迹/校验，增量重算走 `incremental`，不重跑全量。
- CAM 类工艺（如涂胶沿曲线）扩展 `cam.py` 时只加提取器策略，不改主流程。

### B4. 出码适配（post 适配器）

- 已有 8 品牌：零改动，`program.py` 出码自动生效。
- 新品牌：`post/` 加一个 Jinja2 模板 + 注册，继承 `post/base.py`，**新品牌 = 新模板**，禁止改调用方。

### B5. 时序与 IO 联动

- 工序编排进 `sequence`（时序模型），PLC 侧经 `iomap` 点表 + `plc_scl` 出码，全部走接口。
- 握手契约复用 `cell.py` 的 IO 契约，不另起炉灶。

### B6. 仿真验收（simulation 域）

- 时序仿真跑通"工艺步 → 设备动作 → 附着/释放事件"（world/animation 模式）。
- 多机节拍验证用 `multirobot`；能虚拟验收的不上真机。

### B7. GUI + 收尾门禁

- 同 A6/A7：features 门控、i18n、守护脚本 0 违规、测试全绿、pyproject 同步、BUILD_QUEUE 记账。

---

## 通用红线（两条流水线都适用）

1. 归属拿不准 → 停下问（AGENTS.md #22），按"我打算 X，代价是 Y，替代方案是 Z"格式给方案。
2. 增量 > 原文件 1/3 → 必须新建文件；存量债务文件禁止加新功能（只许修 bug + 一次一类迁出）。
3. 新扩展点暂无注册表归宿 → `# EXTENSION_POINT: <用途>` 标记 + BUILD_QUEUE 收口条目。
4. 一次提交一件事；PR 描述写明"本改动位于 X 层/域，因为…"。
