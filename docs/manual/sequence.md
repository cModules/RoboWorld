# 工艺时序（动态时序）使用指南

> 对应模块：`service/sequence.py`（v1.3.1 落地，v1.3.2 补示例工程与本文档）
> 示例：`examples/sequence_demo.py`（一条命令跑通全部内容）
> 运行：`PYTHONPATH=python python examples/sequence_demo.py`

## 1. 解决什么问题

设备树此前只表达**静态绑定**（谁挂在谁下面，坐标系自动吸合，v1.3.0）。
真实产线还需要**动态时序**：谁先触发什么信号、等什么信号到位、调哪个
子程序、夹紧后保压多久——这些工序编排以前只能写在工作交流里，现在
编成一条有序序列，并自动落到机器人与 PLC 两侧的既有出口。

## 2. 四种工步

| 工步 | 类型 | 字段 | 语义 |
|---|---|---|---|
| 工序触发 | `SignalStep` | `signal`、`value` | 置位/复位一个数字输出 |
| 信号等待 | `WaitStep` | `signal`、`trigger`、`value`、`timeout` | 等输入达到电平/上升沿/下降沿 |
| 子程序调用 | `CallStep` | `subprogram` | 记录"调谁"；**不是程序指令**（见 §4） |
| 驻留 | `DwellStep` | `seconds` | 纯延时（夹紧保压、机构响应） |

每条工步都有 `name` 与 `device`（泳道 = 设备名）。

## 3. 信号名从哪来

信号**逻辑名**（如 `gripper_close`）尽量复用 `service.cell.HANDSHAKE_SIGNALS`
（`robot_start` / `robot_done` / `gripper_open` / `gripper_close` /
`part_present` / `safety_ok`），物理地址留给工作单元点表
（`Device.io_map`）。**同一份序列换台 PLC 只改映射，不改编排**。

## 4. 两个出口，零新增语法

### 机器人侧

```python
builder = orp.ProgramBuilder(robot)
notes = seq.to_program(builder)
program = builder.build()
code = program.to_post("ur_polyscope", metadata={"io_map": robot_device.io_map})
```

- 翻译成**既有**指令：`SetDO` / `WaitDI` / `WaitTime`，不新增任何指令语法。
- `CallStep` 不是程序指令（各品牌主程序/子程序的组织方式不同），
  `to_program` 把它**如实记进返回的 notes**——调用方应把 notes 展示出来，
  而不是静默丢弃。本模块绝不臆造 `CALL`/`Job` 调用语法。
- `metadata["io_map"]` 传入点表后，出码把逻辑名解析成真实数字量序号
  （`gripper_close` → `set_digital_out(0, True)`）；解析不了会留
  `TODO` 占位，绝不编造假地址。

### PLC 侧

```python
from service.plc_scl import generate_scl
scl = generate_scl(cell, program_name="CellControl", steps=seq.to_plc_steps())
```

- 翻成 `plc_scl.Step` 工步，规则与 `steps_from_program` 逐字段一致
  （输出累积、等待收束、驻留转延时），可直接出西门子 SCL。
- 子程序调用在 PLC 侧**跳过并记 note**：PLC 触发机器人程序走的是
  既有握手信号，不是"调用"。

## 5. 校验

```python
issues = seq.validate(
    signal_names=cell.all_signals(),      # 真实点表 ∪ 握手信号
    subprograms=["PickPart", "PlacePart"],
    devices=[d.name for d in cell.devices],
)
```

返回 `SeqIssue` 列表（无问题 = 空列表）。七类问题：

| code | 级别 | 含义 |
|---|---|---|
| `empty` | warning | 空序列 |
| `duplicate_name` | warning | 工步重名 |
| `unknown_device` | **error** | 泳道不在场景设备里 |
| `unknown_signal` | **error** | 信号不在点表里——下装后发不出/收不到 |
| `unknown_subprogram` | **error** | 调用的子程序不存在 |
| `no_timeout` | warning | 无限等待——现场必须配看门狗 |
| `edge_ignores_value` | warning | 边沿触发时期望电平不参与判定 |

`issue.index` 指回出问题的工步下标，GUI 里据此定位行。

## 6. 诚实性口径（重要）

序列只有**顺序**，没有实测节拍。驻留（秒）是唯一可计算的时间；
`summary()` 只报驻留合计与各类型步数，note 明写
**「驻留合计是唯一可计算的时间；触发与等待的耗时取决于现场机构」**——
绝不编一个"预计节拍"。同样，边沿触发降级为电平、超时看门狗待补，
都会在出码里留显式 `TODO` 注释。

## 7. 持久化

```python
seq.save_json("examples/assets/pick_sequence.json")
back = ProcessSequence.load_json("examples/assets/pick_sequence.json")
```

JSON schema：`service.sequence/1`，每条工步带 `kind` 字段
（`call` / `signal` / `wait` / `dwell`）；坏 schema 与未知 `kind` 都抛
`ValueError`。

## 8. GUI

设备树标签下的「工艺时序（泳道图）」面板（v1.3.1）：

- 工步表 + 自绘泳道画布（x 轴 = 步骤序号，驻留块按秒加宽，四色区分）；
- 添加（对话框按工步类型变字段）/ 删除 / 上移 / 下移 / 校验 / 保存 / 载入；
- 信号下拉由主窗口从**真实点表 ∪ 握手信号**注入，选不到空气信号；
- 校验结果逐条列出并定位到工步行。

## 9. 完整示例

`examples/sequence_demo.py` 按顺序演示：建点表 → 编排 9 条工步 →
校验（含一条刻意安排的无限等待警告 + 未知信号自证）→ 摘要 →
机器人侧 UR 出码 → PLC 侧 SCL 出码 → JSON 往返。其验收测试
`tests/test_sequence_demo.py` 以子进程真实运行脚本逐段断言。
