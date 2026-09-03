# RobotWorld 中文使用说明

[English README](../README.md) · [后处理器开发指南](post_dev_guide.md) · [厂商 CAD 接入](VENDOR_CAD_SETUP.md)

**开源机器人编程与仿真平台**

一个面向工业机器人的开放、可扩展、社区驱动的生态，基于开放标准（URDF）、透明算法与社区维护的后处理器库。RobotWorld 不止于离线编程（OLP）：它将机器人建模、轨迹规划、仿真与代码生成统一到一条可审计的工具链中，作为 RoboDK / Robotmaster / DELMIA 等封闭生态的免费替代方案。

> **愿景**：通过开放标准降低工业机器人编程与仿真的门槛，让整个工具链开放、可审计、可改造、可学习。

---

## ✨ 开发进展

截至 `main` 最新提交的实际状况。测试与模型库数据以当前代码为准。

| 模块 | 状态 | 当前可用能力 |
|---|---|---|
| **机器人模型库** | ✅ 28 个型号 | UR / KUKA / ABB / FANUC / Estun / JAKA / 汇川；程序化 URDF + 7 个 JAKA 真机 CAD（厂商 STEP 转网格） |
| **离线编程 API** | ✅ 完成 | 链式 `Robot.moveJ / moveL / moveC / moveSpline`；轨迹展开；节拍估算；可达/限位校验 |
| **后处理器** | ✅ 7 个品牌 | Jinja2 模板：`ur_polyscope` / `kuka_krc4` / `abb_rapid` / `fanuc_tp` / `estun_er` / `jaka_zu` / `huichuan_inoteach`；SPLINE 自动降级为 PTP/LIN |
| **代码上传 (`link.py`)** | ✅ 完成 | `ControllerLink` 抽象 — `FileLink`（离线导出 + manifest.json）、`SimLink`（进程内运动学回读）；真实控制器按同接口接入 |
| **在线调试 (`debug.py`)** | ✅ 完成 | `DebugSession`：单步/连续播放、回读实际位姿、与规划值偏差（mm + deg）、示教 jog |
| **运动学** | ✅ 纯 Python | FK / IK / 雅可比回退（C++ 核心可选）；**137 个测试通过** |
| **3D GUI (`olp.py`)** | ✅ 完成 | PySide6 + OpenGL 视口、关节滑条、机器人库、路径代码编辑器、场景与离线编程面板 |
| **场景/碰撞** | 🔧 脚手架 | 连杆级 SAT 干涉检查可用；FCL 集成计划 v0.2 |
| **ROS 2 数字孪生** | 📋 规划中 | 原生 `rclpy` 桥接，目标 v0.3 |
| **轨迹优化** | 📋 规划中 | TOPP-RA 时间最优参数化，目标 v1.0 |

---

## 🖼 主要功能演示

`olp.py` 一个文件就是 GUI——加载任意 URDF，左侧切标签页，右侧拖关节滑条。

### 主界面 · 3D 视口 + 机器人库 + 关节控制
![主界面](../docs/screenshots/main_ur5e.png)

- 左侧 **机器人库** 列出 `models/` 下全部型号，支持搜索 + 品牌/负载/臂展筛选；单击即可加载。
- 中央是 OpenGL 视口（上图是程序生成的 UR5e，下方还有 JAKA Zu5 真机 CAD）。
- 右侧显示 **6 路关节滑条**，限位按当前型号规格加载——拖动即摆动。

### 路径代码 · 示教点/生成的机器人代码编辑器
![路径代码](../docs/screenshots/path_code.png)

- **路径代码** 标签页：文本编辑器 + Record / Play / Stop / Clear 按钮。
- 实时解析：每识别到一条 `movej` / `movel` 就在 3D 视口里画出对应的示教轨迹。

### 离线编程 · 程序校验 + 节拍估算 + 回放
![离线编程](../docs/screenshots/offline_programming.png)

- **离线编程** 标签页：程序摘要（节拍 / TCP 路径 / 路径点数）、**校验程序 (F7)** 做可达/限位/干涉检查、**回放** 连续运行、**停止** 中断。
- 背后直接调用 `orolp.Planner`——用真实运动学，不是 mock。

### 厂商 CAD 接入 · JAKA Zu5 真机网格
![JAKA Zu5 真机](../docs/screenshots/jaka_zu5_real.png)

- 加载 `models/jaka/zu5/zu5_real.urdf`，网格来自**官方 STEP**（工具链在 `tools/build_vendor_urdf.py`，详见 [VENDOR_CAD_SETUP.md](VENDOR_CAD_SETUP.md)）。
- 右侧关节限位按 **Zu5 真机规格**（`joint_2 -85..265°`、`joint_3 ±175°`、`joint_4 -85..265°`），不是泛化的 ±360°。
- 这是已跑通的标杆接入；KUKA / ABB / FANUC / Estun 走同一条流水线——把官方 STEP 放进 `assets/vendor/<brand>/step/` 再跑工具即可。

---

## 🤖 支持的机器人与控制器

| 品牌 | 控制器 | 后处理器 | 真机 CAD |
|---|---|---|---|
| Universal Robots | PolyScope 5 / 6 | `ur_polyscope` | ✅ UR3 / UR5 / UR10 / UR3e / UR5e / UR10e / UR16e |
| JAKA | Zu App | `jaka_zu` | ✅ **Zu3 / Zu5 / Zu7 / Zu12 / Zu18 / Zu20 / Zu30**（厂商 STEP → 网格） |
| KUKA | KRC4 | `kuka_krc4` | 🔄 流水线就绪，待 STEP |
| ABB | IRC5 | `abb_rapid` | 🔄 流水线就绪，待 STEP |
| FANUC | R-30iB | `fanuc_tp` | 🔄 流水线就绪，待 STEP |
| Estun | ER | `estun_er` | 🔄 流水线就绪，待 STEP |
| 汇川 | InoTeach | `huichuan_inoteach` | 🔄 仅后处理器 |

> **新增一个品牌 < 30 分钟**：把 STEP 放进 `assets/vendor/<brand>/step/`，跑 `tools/build_vendor_urdf.py --brand <brand>`，再写一个 Jinja2 模板。详见 [post_dev_guide.md](post_dev_guide.md)。

---

## 🚀 安装与快速开始

### 1. 拉取代码
```bash
git clone <your-fork-url>          # 实际仓库地址由维护者提供
cd RobotWorld
```

### 2. 准备 Python（推荐 conda）
```bash
conda create -y -n robotworld python=3.11
conda activate robotworld
```
也可用任何 virtualenv，支持 Python **3.9 ~ 3.12**。

### 3. 安装依赖 + `orolp` 包
```bash
pip install -e ".[gui]"
```
这条会一次性装好核心（numpy / scipy / jinja2 / pyyaml / pydantic）+ GUI（PySide6 / PyOpenGL），并以可编辑方式安装 `orolp` 包本身。**无需编译 C++**——纯 Python 运动学回退已随包发布，并由 137 个测试守护。

若 PyPI 镜像慢：
```bash
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
```

### 4. 启动 GUI
```bash
python olp.py models/ur/ur5e/ur5e.urdf
```
- **macOS / Windows** 有桌面：直接弹窗。
- **无桌面 Linux**（CI / 沙箱）：用 `Xvfb :99 -screen 0 1600x1000x24` 起虚拟显示，`DISPLAY=:99 python olp.py …`。

### 5. 第一个程序（无 GUI，纯脚本）
```python
# examples/olp_full_workflow.py — 端到端可运行示例
from orolp import Robot, Program, Planner
from orolp.post import process as post_process
from orolp.link import FileLink, SimLink
from orolp.debug import DebugSession

robot = Robot("models/ur/ur5e/ur5e.urdf", name="UR5")
prog = (
    Program(robot)
    .moveJ([0, -1.5708, 1.5708, -1.5708, 1.5708, 0])
    .moveL_to((0.50, 0.20, 0.30, 0, 0, 0))
    .moveL_to((0.50, 0.30, 0.30, 0, 0, 0))
)
Planner().plan(prog)                       # 展开 + 节拍 + 校验
print(post_process(prog, "ur_polyscope"))  # 输出 URScript

link = FileLink(out_dir="out/")            # 离线导出 + manifest.json
prog.deploy(link, "ur_polyscope", name="pick_and_place")

# SimLink：通过运动学回读（真机换成 ControllerLink 即可对接控制器）
sim = SimLink(robot)
dbg = DebugSession(prog, sim)
for deviation in dbg.play():
    print(deviation)                      # mm + deg 与规划值的偏差
```

---

## 🏗 架构

```
┌───────────────────────────────────────────────────────────────┐
│  GUI 层 — olp.py（PySide6 + OpenGL）                           │
│  ──────────────────────────────────────────────────────────── │
│  MainWindow · 标签页：机器人库 / 路径代码 / 场景 / 离线编程  │
│  GLWidget 视口 · 关节滑条 · 路径代码编辑器                    │
├───────────────────────────────────────────────────────────────┤
│  Python API — `orolp` 包                                       │
│  ──────────────────────────────────────────────────────────── │
│  Robot · Pose · Motion(JOINT/LINEAR/CIRCULAR/SPLINE)          │
│  Program · Planner · SimulationWorld · Link · DebugSession    │
├───────────────────────────────────────────────────────────────┤
│  运动学核心                                                    │
│  ──────────────────────────────────────────────────────────── │
│  纯 Python FK·IK·雅可比（默认，CI 与测试使用）               │
│  C++ Pinocchio/FCL 核心（可选，通过 CMakeLists.txt 构建）     │
├───────────────────────────────────────────────────────────────┤
│  后处理器生态 — Jinja2 模板                                    │
│  ──────────────────────────────────────────────────────────── │
│  ur_polyscope · kuka_krc4 · abb_rapid · fanuc_tp              │
│  estun_er · jaka_zu · huichuan_inoteach                        │
└───────────────────────────────────────────────────────────────┘
```

可扩展接口：
- **`Link` 接口** —— 把 `FileLink`（离线）换成 `SimLink`（仿真）或 `ControllerLink`（真机，如 RTDE / PROFINET / 厂商 SDK），应用层无需改代码。
- **后处理 `_convert_spline`** —— 若某品牌控制器原生支持 SPLINE，重写该方法即可保留；默认降级为 PTP/LIN。

---

## 🛣 路线图

### ✅ v0.1.0（当前 — Alpha）
- [x] 基于 URDF 的统一模型库（28 个型号，7 个品牌）
- [x] 链式运动 API + 规划器展开 + 节拍
- [x] 7 个品牌后处理器（Jinja2）+ 单元测试
- [x] `Link` 接口 + `FileLink` / `SimLink` 实现
- [x] `DebugSession`：回读 + 偏差
- [x] 纯 Python 运动学回退（FK / IK / 雅可比）—— 137 测试通过
- [x] PySide6 + OpenGL 终端 GUI（机器人库 / 路径代码 / 场景 / 离线编程 标签）
- [x] JAKA 7 型号真机 CAD 接入（厂商 STEP → 网格）
- [x] 多品牌 STEP 接入工具（`tools/build_vendor_urdf.py`）

### 🔧 v0.2.0（Beta）
- [ ] FCL 碰撞集成（替换连杆级 SAT 脚手架）
- [ ] 笛卡尔直线 + IK 多解选择
- [ ] 工具/工件坐标系标定
- [ ] CI + 真机 URDF 回归 + LFS 资产缓存

### 📋 v0.3.0（生产候选版）
- [ ] 节点式可视化编程
- [ ] ROS 2 数字孪生桥接（`rclpy`）
- [ ] 插件市场

### 📋 v1.0.0
- [ ] TOPP-RA 时间最优轨迹参数化
- [ ] 多机器人协同
- [ ] 力控工艺仿真
- [ ] IEC 61131-3 PLC 代码生成

---

## 🧪 测试

```bash
pytest tests/ python/tests/    # 137 个测试，覆盖运动学 / 后处理 / link / debug / 厂商 CAD
```
CI 工作流：`.github/workflows/ci.yml`。

---

## 🤝 贡献

欢迎机器人工程师、研究者、集成商共同完善。

- **新增品牌** —— 最简单的入口：写一个 Jinja2 模板 + 把 STEP 放进 `assets/vendor/<brand>/step/`。详见 [post_dev_guide.md](post_dev_guide.md)。
- **核心算法改进** —— IK 求解器、路径规划、标定。
- **硬件验证报告** —— 用生成的 URScript / KRL / RAPID 在真实控制器上跑过的结果。

详见 [CONTRIBUTING.md](../CONTRIBUTING.md)。

---

## 📄 许可证

RobotWorld 采用 **Apache License 2.0**。

`models/` 中的第三方机器人 CAD 模型可能带有原作者的独立许可证——详见各子目录的 `README.md`。

---

## 🙏 致谢

- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) — INRIA / LAAS-CNRS
- [FCL](https://github.com/flexible-collision-library/fcl) — Flexible Collision Library
- [OMPL](https://ompl.kavrakilab.org/) — Open Motion Planning Library
- [RoboDK](https://robodk.com/) — 灵感来源与行业标杆
- 所有在真实控制器上做过验证的贡献者

---

**由开源机器人社区用 ❤️ 打造。**