# RoboWorld

> 🌐 **语言 / Language**：**简体中文**（当前） · [English](./README_EN.md)

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Python](https://img.shields.io/badge/Python-3.9%2B-blue.svg)](https://python.org)
[![Tests](https://img.shields.io/badge/tests-1846%20passed-brightgreen.svg)](#tests)
[![Platform](https://img.shields.io/badge/platform-Linux%20%7C%20macOS%20%7C%20Windows-lightgrey.svg)](#install)

**开源机器人编程与仿真平台**

一个面向工业机器人的桌面编程与仿真工具。RoboWorld 将机器人建模、3D 场景编辑、轨迹规划、碰撞检查、程序回放和控制器代码生成集中在一个工作界面中，适合快速体验离线编程流程，也适合工程团队继续扩展。

打开应用即可看到 3D 视口、机器人与零件资源库、设备树、属性编辑器、路径代码预览和导出面板：载入 URDF，拖动关节检查姿态，编辑示教点，校验轨迹，再导出目标控制器程序。

[⬇️ 下载与安装](#install) · [🖼 查看界面与功能](#ui) · [📖 用户手册](docs/manual/INDEX.md) · [GitHub 仓库](https://github.com/cModules/RoboWorld) · [提交 Issue](https://github.com/cModules/RoboWorld/issues)

**关键词**：industrial robotics · offline programming · robot simulation · robotics digital twin · trajectory planning · motion planning · inverse kinematics · collision checking · multi-robot coordination · force control · CAD/CAM · PLC · URDF · Python · PySide6

---

<!-- 显式锚点：标题带 emoji，GitHub 自动生成的 id 会附带前导连字符且随渲染实现而变，
     导航链接因此指向这里的固定 id（#ui / #install / #tests）而不依赖自动生成规则。 -->
<a id="ui"></a>

## 🖥️ 界面与功能

RoboWorld 的主界面围绕一次完整的机器人编程工作流设计：

- **3D 视口**：查看机器人、夹具、工件和工作站布局，拖动关节滑块检查姿态与可达性。
- **场景设备库与设备树**：搜索并载入 URDF 模型，管理机器人、工具、输送带、料箱和托盘的装配关系。
- **属性编辑器**：修改对象位姿、关节角、模型参数和挂载关系，夹爪可直接吸合到机器人法兰。
- **代码预览与路径示教**：编辑路点和运动类型，录制、播放、停止轨迹，并在视口中同步显示路径。
- **导出打包**：执行可达性、关节限位、碰撞与节拍检查，生成 UR、KUKA、ABB、FANUC 等控制器程序。
- **工作站演示**：一键载入抓取工作站，体验 IK、夹爪 IO、工件跟随、轨迹回放和多品牌出码。

## 🖼 关键功能演示

GUI 就是一个 `main.py`：加载任意 URDF，左侧「资源库」选设备，右侧标签页编辑属性、查看场景与代码。

### 主视图 · 3D 视口 + 资源库 + 关节控制
![主视图](docs/screenshots/main_ur5e.png)

- 左侧 **资源库** 列出 `assets/models/` 下所有模型，支持搜索与品牌/负载/臂展筛选；点击条目即可载入。
- 中间是 OpenGL 视口：载入 UR5e 后可拖动关节、查看轨迹路径线与选中对象。
- 右侧 **属性** 标签显示选中对象的规格信息与六轴 **关节滑块**（限位随机型）；拖动即可实时运动。选中夹爪/工件等组件时，还可直接编辑其 **位置（位姿 XYZ+RPY）** 与 **模型参数**。

### 路径代码编辑器（代码预览）· 编辑示教点 / 生成机器人代码
![路径代码](docs/screenshots/path_code.png)

- **代码预览** 标签：文本编辑器 + 录制（Record）/ 播放（Play）/ 停止（Stop）/ 清空（Clear）。
- 实时解析：每识别到一条 `movej` / `movel`，都会在 3D 视口中以轨迹点连线显示。
- 代码预览标签内含 **路径示教** 子页，管理示教点语义（接近/抓取/放置点）与工件绑定。

### 离线编程（导出打包）· 程序校验 + 节拍估算 + 回放
![离线编程](docs/screenshots/offline_programming.png)

- **导出打包** 标签：程序摘要（节拍、TCP 路径、轨迹点数），**校验程序 (F7)** 检查可达性 / 限位 / 干涉，**回放** 连续运行，**停止** 中断。
- 规划与校验走 `service.planner`（真实运动学 + 梯形 / S 曲线速度规划），而非模拟。

### 厂商 CAD 接入 · JAKA Zu5 真机网格
![JAKA Zu5 真机](docs/screenshots/jaka_zu5_real.png)

- 从 `assets/models/jaka/zu5/zu5_real.urdf` 加载 JAKA Zu5，网格来自**官方 STEP**（工具链见 `tools/build_vendor_urdf.py`，详见 [docs/manual/VENDOR_CAD_SETUP.md](docs/manual/VENDOR_CAD_SETUP.md)）。
- 右侧关节限位按 **Zu5 真机规格**（`joint_2 -85..265°`、`joint_3 ±175°`、`joint_4 -85..265°`），而非泛化的 ±360°。
- 这是标杆级接入；KUKA / ABB / FANUC / Estun 走同一条流水线——把 STEP 放入 `assets/vendor/<brand>/step/` 再运行该工具即可。

### 抓取工作站 · 一键演示（S1–S5）
![抓取工作站](docs/screenshots/grasping_workstation.png)

- **文件 → 抓取工作站演示（一键）**：构建标准抓取工作站（UR5e + 带 TCP 的真空夹爪 + 含 4 个零件的输送带 + 料箱 + 托盘），并自动载入一个 8 点五抓循环演示程序——校验、节拍（状态栏）与回放自动开始。
- 同一条管线的无头一键 showcase：`python examples/grasping_workstation_demo.py` → 模板场景 → IK 演示点 → 夹爪 IO → 节拍 → UR / KUKA / ABB / FANUC 代码，全部基于真实 API。

![回放](docs/screenshots/grasping_playback.png)

- 回放时示教的 PICK/PLACE 语义驱动夹爪：零件吸附到法兰、跟随 TCP，并在释放处留在托盘上。
- 在线演示（同数据，浏览器渲染）：见 **deploy/web**——由 `tools/build_web_site.py` 构建。

![离线编程](docs/screenshots/grasping_offline_programming.png)

- **导出打包** 面板基于真实场景校验演示程序（5.4 s 节拍、638 个 TCP 轨迹点、8 段运动 + 5 条 IO 指令），并从同一份程序定义导出各品牌代码。

### 相机触发抓取 · 一条命令跑通"标定 → 视觉 → 抓取"（v0.3.4）

```bash
python examples/vision_pick_demo.py          # 默认噪声
python examples/vision_pick_demo.py --noise 1.0 --code   # 放大标定噪声 + 打印出码
```

- **TCP 标定**：4 点法解出吸盘偏移（实测误差 1e-14 mm 量级），6 点法沿工具 +X/+Z 触碰补上方向（姿态误差 1e-14°）。
- **托盘坐标系 3 点法**：落件点按托盘局部坐标算——托盘转了 15° 也自动跟着走，程序里的常数不再是"魔法数字"。
- **手眼标定**（eye-to-hand，真实 UR5e 位形 + 腕部多向偏转）：0.15 mm / 0.03° 采集噪声下外参误差 **0.67 mm**，方程残差 0.415 mm。
- **视觉定位**：工件检测像素经标定外参反投影到支承面 → 抓取位姿，闭环定位误差 **0.63 mm**；随后 IK → 程序 → 校验（0 错误）→ 品牌出码，代码头部自动带标定摘要。
- 标定集落盘为 `examples/assets/grasping_workstation_calibration.json`，可被 GUI / 程序直接加载复用。

---

## 🤖 支持的机器人与控制器

| 品牌 | 控制器 | 后处理器 | 真机 CAD |
|---|---|---|---|
| Universal Robots | PolyScope 5 / 6 | `ur_polyscope` | 🔄 程序化模型 ur3e / ur5e / ur10e / ur16e，待 STEP |
| JAKA | Zu App | `jaka_zu` | ✅ **Zu3 / Zu5 / Zu7 / Zu12 / Zu18 / Zu20 / Zu30**（厂商 STEP → 网格） |
| KUKA | KRC4 | `kuka_krc4` | 🔄 流水线就绪，待 STEP |
| ABB | IRC5 | `abb_irc5` | 🔄 流水线就绪，待 STEP |
| FANUC | R-30iB | `fanuc_r30ib` | 🔄 流水线就绪，待 STEP |
| Estun | ER | `estun_er` | 🔄 流水线就绪，待 STEP |
| 汇川 | InoTeach | `huichuan_inoteach` | 🔄 仅后处理器 |

> **新增一个品牌 < 30 分钟**：把 STEP 放入 `assets/vendor/<brand>/step/`，运行 `tools/build_vendor_urdf.py --brand <brand>`，再写一个 Jinja2 模板。详见 [docs/manual/post_dev_guide.md](docs/manual/post_dev_guide.md)。

---

<a id="install"></a>

## 🚀 安装与快速开始

### 1. 下载或获取代码

- **源码运行**：从 [GitHub 仓库](https://github.com/cModules/RoboWorld) 下载 ZIP，或执行：

```bash
git clone https://github.com/cModules/RoboWorld.git
cd RoboWorld
```

- **桌面安装包**：从 [Releases](https://github.com/cModules/RoboWorld/releases) 下载与你的系统匹配的安装包；当前发布页若暂无安装包，请使用源码方式运行。

### 2. 准备 Python（推荐 conda）
```bash
conda create -y -n robotworld python=3.11
conda activate robotworld
```
也可使用任意 virtualenv——支持 Python **3.9 ~ 3.12**。

### 3. 安装依赖（可编辑安装）
```bash
pip install -e ".[gui]"
```
该命令一次性安装核心依赖（numpy / scipy / jinja2 / pyyaml / pydantic）**与** GUI（PySide6 / PyOpenGL），并以可编辑方式安装 RoboWorld 的各分层包（`domain` / `service` / `adapter` / `app` / `support`）。无需编译 C++——纯 Python 运动学回退已随包发布，并由 1846 个测试守护。

若位于较慢的 PyPI 镜像后：
```bash
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
```

### 4. 启动 GUI
```bash
python main.py assets/models/ur/ur5e/ur5e.urdf
```
- **macOS / Windows** 有桌面：直接弹窗。
- **无桌面 Linux**（如 CI / 沙箱）：用 `Xvfb :99 -screen 0 1600x1000x24` 起虚拟显示，再 `DISPLAY=:99 python main.py …`。

### 4.1 云端 / 容器运行（Cloud Studio · noVNC）

仓库内置 `.ide/Dockerfile` 构建「云端开发容器」：在 `node:20` 之上叠加 Python 3.11 + RoboWorld 运行/测试/GUI 依赖，并集成 `Xvfb + x11vnc + websockify + noVNC`，让 `main.py` 跑在虚拟桌面并经浏览器访问。

- 为缓解 Cloud Studio 等**内存受限构建机**在安装 GUI 时的 OOM，GUI 依赖已改用 `PySide6-Essentials`（仅含 QtCore/QtGui/QtWidgets/QtOpenGLWidgets，不含 3D/Charts 等 Addons），并启用 `PIP_NO_BUILD_ISOLATION` / `PIP_PREFER_BINARY` 与国内镜像 + 超时重试，降低内存峰值与依赖回溯失败。
- 构建并运行：

```bash
docker build -f .ide/Dockerfile -t robotworld .
docker run -p 6080:6080 robotworld      # 浏览器打开 http://localhost:6080 即可看到 main.py 桌面
```

> 容器默认通过 `.ide/start.sh` 启动 noVNC 试运行栈（`Xvfb → main.py → x11vnc → websockify/noVNC`）；IDE 由 Cloud Studio 平台另行拉起，互不冲突。

### 5. 第一个程序（无 GUI，纯脚本）
```python
# 与 examples/olp_full_workflow.py 同源：无 GUI，纯脚本跑通全链路
# 这里用 KUKA KR16-2（首发品牌，节卡同理）
import numpy as np
from domain.robot import Robot
from domain.link import FileLink, SimLink
from service.program import ProgramBuilder
from service.debug import DebugSession

# 1) 载入机器人（URDF）
robot = Robot("assets/models/kuka/kr16_2/kr16_2.urdf", name="KR16")

# 2) 编程序：目标位姿取自真实关节角的**正解**，因此换品牌也一定可达，
#    不必为每个品牌手调笛卡尔坐标
pose_b = robot.forward_kinematics(np.array([30.0, -30.0, 30.0, -30.0, 30.0, 0.0]))
prog = (
    ProgramBuilder(robot)
    .moveJ([0.0] * robot.dof, speed=40)     # 关节插补
    .moveL(pose_b, speed=60, blend=10)      # 直线插补（blend = 过渡半径 mm）
    .build()
)

# 3) 校验：可达性 / 关节限位 / 奇异 / 干涉，并给出节拍
report = prog.validate()
print(report.ok, report.num_motions, f"{report.cycle_time:.2f}s")

# 4) 出码 + 部署：FileLink 离线导出程序与 manifest.json（换成 ControllerLink 即上真机）
print(prog.to_post("kuka_krc4"))           # 输出 KUKA KRL（.src）
link = FileLink("out/")
link.connect()
print(prog.deploy(link, "kuka_krc4", name="pick_and_place").handle)

# 5) 在线调试：在仿真连接上播放，读回位姿与规划值比对偏差
sim = SimLink(robot)
sim.connect()
summary = DebugSession(prog.simulate(), sim, robot=robot).run()
print(summary["max_pos_deviation_mm"], "mm /", summary["max_rot_deviation_deg"], "deg")
```

换品牌只需改两处——模型路径与后处理器名。例如节卡：`assets/models/jaka/zu5/zu5_real.urdf`（厂商 STEP 真机网格）+ `jaka_zu`；`--brand` 可切换的机型见 `examples/olp_full_workflow.py`。

---

## 🏗 架构

```
┌───────────────────────────────────────────────────────────────┐
│  GUI 层 — main.py（PySide6 + OpenGL）                           │
│  ──────────────────────────────────────────────────────────── │
│  左双 Tab：① 场景设备库（机器人 / 组件库）② 设备树            │
│  右标签：属性 / 代码预览 / 导出打包（内含「路径示教」子页）     │
│  GLWidget 视口 · 关节滑块 · 对象位姿/参数编辑 · 路径代码编辑器 │
├───────────────────────────────────────────────────────────────┤
│  Python API — 分层包（domain / service / adapter）            │
│  ──────────────────────────────────────────────────────────── │
│  domain：Robot · Pose · Link · 运动学 / 碰撞 / 包络           │
│  service：ProgramBuilder · Program · TrajectoryPlanner        │
│  service：DebugSession · SimulationWorld · 场景与工程         │
│  adapter：post（Jinja2 后处理）· plugins · robot_library      │
├───────────────────────────────────────────────────────────────┤
│  运动学核心                                                    │
│  ──────────────────────────────────────────────────────────── │
│  纯 Python FK·IK·雅可比（默认，CI 与测试使用）               │
│  C++ Pinocchio/FCL 核心（可选，通过 CMakeLists.txt 构建）     │
├───────────────────────────────────────────────────────────────┤
│  后处理器生态 — Jinja2 模板                                    │
│  ──────────────────────────────────────────────────────────── │
│  ur_polyscope · kuka_krc4 · abb_irc5 · fanuc_r30ib            │
│  estun_er · jaka_zu · huichuan_inoteach                        │
│  + 插件品牌（adapter.plugins）                                    │
└───────────────────────────────────────────────────────────────┘
```

关键可扩展接口：
- **`Link` 接口** —— 把 `FileLink`（离线）换成 `SimLink`（仿真）或真实 `ControllerLink`（如 RTDE / PROFINET / 厂商 SDK），应用层无需改代码。
- **各品牌模板中的 `_convert_motion(motion)`** —— 重写 `_convert_spline` 可在控制器原生支持样条时保留之；默认降级为 PTP/LIN。

---

<a id="tests"></a>

## 🧪 测试

```bash
pytest tests/    # 1846 个测试，覆盖运动学 / 后处理 / link（含真机 Socket 与 SSH 通路及打桩控制器）/ PLC↔机器人 IO 点表 / 驻留段时间轴 / 轨迹编辑器圆弧编辑 / 场景拖拽摆放 / 规模与性能基准（FK 常量缓存等价性 + motion_at 二分对拍）/ 干涉检查采样口径与几何缓存（内核对拍 + 三级漏斗账本）/ 增量重算（内容寻址缓存 + 逐位等价 + 起点进键的回归守卫）/ debug / 厂商 CAD / 标定与手眼 / 多机器人 / 力控 / 数字孪生 / 插件接口 / 云通道 / 程序版本对比 / 插件包 / 包络容差 / i18n / 真机连接面板（Socket + SSH 双传输与取回核对）/ 真机 CAD 链路（合成装配体端到端 + OCP 版本兼容层 + 静态守卫）/ 干涉检查结果缓存（场景签名键 + 拖入路径正确性守卫 + 基准诚实性）/ 力矩校验结果缓存（模型内容签名键 + 命中等价 + 计时诚实性）/ 缓存可观测性（三类缓存统一快照 + 关闭态如实呈现 + GUI 缓存状态对话框）/ 界面架构验收（左侧双 Tab 与右侧三标签结构 + 小窗口不裁切换算 + 一键演示后按钮可达性 + en 标签译文）/ 工艺时序（序列模型与七类校验 + 机器人/PLC 双出口 + 与 `steps_from_program` 逐字段等价 + `to_plc_steps → generate_scl` 端到端出码 + JSON 往返 + 泳道编辑器 offscreen 行为）/ 动态时序使用体验（示例脚本子进程验收）/ 零件库与预置场景库（模板零干涉 + 零件库导入 + 面板 offscreen）/ 工程文件 `.rwproj`（兼容 `.robotproj` / `.core`；六段信封 + 逐段容错 + 未知段保真 + 原子写入 + GUI 打开/保存工程与未保存标记）/ 会话支撑（最近打开 + 自动恢复 + 恢复询问 + 干净退出清快照）/ 工程模板（三个内置模板 + 8 点抓取循环真实 IK + 全新构建）/ 打包配置
```
CI 工作流位于 `.github/workflows/ci.yml`。

---

## 🤝 贡献

欢迎机器人工程师、研究者与集成商参与。

- **新增品牌** —— 最易上手的入口：写一个 Jinja2 模板 + 把 STEP 放入 `assets/vendor/<brand>/step/`。详见 [docs/manual/post_dev_guide.md](docs/manual/post_dev_guide.md)。
- **改进核心算法** —— IK 求解器、路径规划、标定例程。
- **硬件验证报告** —— 在真实控制器上跑过生成的 URScript / KRL / RAPID 的结果。

详见 [CONTRIBUTING.md](CONTRIBUTING.md)。

---

## 📄 许可证

**本项目采用 Apache License 2.0。**

本仓库内的代码与文档按 Apache-2.0 发布；`assets/models/` 中的第三方机器人 CAD 模型可能带有各自的许可证，请以各子目录的 `README.md` 为准。完整许可文本见 [`LICENSE`](LICENSE)。


---

## 🙏 致谢

- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) — INRIA / LAAS-CNRS
- [FCL](https://github.com/flexible-collision-library/fcl) — Flexible Collision Library
- [OMPL](https://ompl.kavrakilab.org/) — Open Motion Planning Library
- [RoboDK](https://robodk.com/) — 灵感来源与行业标杆
- 每一位在真实控制器上做过验证的贡献者

---

**由开源机器人社区用 ❤️ 打造。**
