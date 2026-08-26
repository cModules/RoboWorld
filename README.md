# OpenRoboOLP

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Python-3.9%2B-blue.svg)](https://python.org)

**开源工业机器人离线编程软件**

一款免费、可扩展且由社区驱动的商业 OLP 工具替代品，可替代 RoboDK、Robotmaster 和 Delmia。

> **愿景**：通过开放标准（URDF）、透明算法以及社区维护的后处理器生态，普及工业机器人编程。

---

## 核心功能

| 功能 | 状态 | 说明 |
|---------|--------|-------------|
| **多品牌机器人** | ✅ MVP | 基于 URDF 的统一建模；支持 UR、KUKA、ABB、FANUC、汇川 |
| **离线编程** | ✅ MVP | Python API + 可链式运动指令（MoveJ / MoveL） |
| **后处理器** | ✅ MVP | 基于模板（Jinja2）的代码生成；社区可扩展 |
| **运动学** | ✅ MVP | 基于 Pinocchio 的正/逆运动学；解析 + 数值求解器 |
| **碰撞检测** | 🔄 脚手架 | FCL 集成计划 v0.2 完成 |
| **3D GUI** | 📋 规划中 | Qt6 + OpenGL 视口（v0.3） |
| **ROS2 数字孪生** | 📋 规划中 | 原生 rclpy 桥接，实现实时同步 |
| **轨迹优化** | 📋 规划中 | TOPP-RA 时间最优参数化 |

---

## 快速开始

### 前置依赖

- **C++**: GCC 9+ / Clang 12+ / MSVC 2019+
- **CMake**: 3.16+
- **Python**: 3.9、3.10、3.11 或 3.12
- **系统依赖**: `eigen3`、`pinocchio`、`fcl`、`pybind11`

**Ubuntu 22.04/24.04:**
```bash
sudo apt update
sudo apt install -y build-essential cmake libeigen3-dev     libpinocchio-dev libfcl-dev pybind11-dev python3-dev     python3-pip qt6-base-dev  # GUI 可选
```

**macOS:**
```bash
brew install cmake eigen pinocchio fcl pybind11 qt@6
```

### 编译与安装

```bash
git clone https://github.com/your-org/openrobolp.git
cd openrobolp

# 创建虚拟环境（推荐）
python3 -m venv .venv
source .venv/bin/activate

# 安装 Python 依赖
pip install -e ".[dev]"

# 编译 C++ 核心 + Python 绑定
mkdir build && cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
cd ..

# C++ 模块现在可通过 `orolp.orolp_core` 导入
python -c "import orolp; print(orolp.__version__)"
```

### 第一个程序

```python
import orolp as orp

# 加载机器人模型
robot = orp.Robot("models/ur/ur5e.urdf", name="UR5")
robot.tool = orp.Pose.from_xyz_rpy(0, 0, 150, 0, 0, 0)

# 定义抓取与放置
pick = orp.Pose.from_xyz_rpy(500, 200, 100, 0, 0, 0)
place = orp.Pose.from_xyz_rpy(600, 300, 100, 0, 0, 0)

program = (
    robot.moveJ([0, -90, 90, -90, 90, 0], speed=80)
    .then(robot.moveL(pick, speed=50, blend=5))
    .then(robot.moveL(place, speed=80, blend=5))
)

# 仿真并生成原生代码
traj = program.simulate()
print(f"节拍时间: {traj.duration():.2f}s")

ur_code = program.to_post("ur_polyscope")
print(ur_code)
```

更多示例请见 `examples/`：抓取放置、焊接路径、多机器人同步等。

---

## 架构

```
┌─────────────────────────────────────────────────────────────┐
│  Python API 层（用户脚本、节点编辑器）                       │
│  ─────────────────────────────────────────────────────────  │
│  Robot · Pose · Motion · Program · SimulationWorld           │
├─────────────────────────────────────────────────────────────┤
│  C++ 核心引擎（性能关键路径）                                │
│  ─────────────────────────────────────────────────────────  │
│  RobotModel (Pinocchio) · MotionPlanner · CollisionWorld    │
├─────────────────────────────────────────────────────────────┤
│  后处理器生态（Jinja2 模板）                                 │
│  ─────────────────────────────────────────────────────────  │
│  KUKA KRL · ABB RAPID · FANUC TP · URScript · 汇川         │
└─────────────────────────────────────────────────────────────┘
```

---

## 支持的机器人与控制器

| 品牌 | 控制器 | 后处理器 | 已测试 |
|-------|-----------|----------------|--------|
| Universal Robots | PolyScope | `ur_polyscope` | ✅ |
| KUKA | KRC4 | `kuka_krc4` | 🔄 |
| ABB | IRC5 | `abb_irc5` | 🔄 |
| FANUC | R-30iB | `fanuc_r30ib` | 🔄 |
| 汇川 | InoTeach | `huichuan_inoteach` | 🔄 |

> 🔄 = 模板已实现，待真实硬件验证

**添加一个新品牌不到 30 分钟。** 详见 [docs/post_dev_guide.md](docs/post_dev_guide.md)。

---

## 路线图

### v0.1.0（当前 — Alpha）
- [x] 基于 Pinocchio 的 C++ 正/逆运动学核心
- [x] Python 绑定（pybind11）
- [x] Qt6 3D 视口
- [x] 1 个机器人模型
- [x] 轨迹记录与编程



### v0.2.0（Beta）
- [ ] 轨迹插值 
- [ ] 2 个参考后处理器
- [ ] FCL 碰撞检测
- [ ] 带 IK 种子点的笛卡尔直线路径
- [ ] 工件 / 工具坐标系标定
- [ ] 带真实 URDF 回归测试的 CI/CD

### v0.3.0（生产候选版）
- [ ] 可链式运动 API
- [ ] 节点式可视化编程
- [ ] ROS2 数字孪生桥接
- [ ] 插件市场

### v1.0.0（稳定版）
- [ ] TOPP-RA 轨迹优化
- [ ] 多机器人协同
- [ ] 力控工艺仿真
- [ ] IEC 61131-3 PLC 代码生成

---

## 贡献

我们欢迎机器人工程师、研究人员和制造商的贡献。

- **后处理器**：添加你的机器人品牌 —— 最简单的切入点
- **核心算法**：逆运动学求解器、路径规划、标定
- **文档**：教程、API 文档、翻译
- **测试**：硬件验证报告

贡献指南请见 [CONTRIBUTING.md](CONTRIBUTING.md)，后处理器开发指南请见 [docs/post_dev_guide.md](docs/post_dev_guide.md)。

---

## 许可证

OpenRoboOLP 采用 **Apache License 2.0** 许可。

`models/` 中的机器人 CAD 模型（URDF、网格）可能带有原作者的独立许可证。详见各目录下的 `README.md`。

---

## 致谢

- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) — INRIA / LAAS-CNRS
- [FCL](https://github.com/flexible-collision-library/fcl) — Flexible Collision Library
- [OMPL](https://ompl.kavrakilab.org/) — Open Motion Planning Library
- [RoboDK](https://robodk.com/) — 灵感来源与行业标杆

---

**由开源机器人社区用 ❤️ 打造。**
