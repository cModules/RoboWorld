# OpenRoboOLP 示例程序

本目录包含 OpenRoboOLP 的各种示例程序，演示如何使用 Python API 进行机器人离线编程。

## 目录结构

```
examples/
├── pick_and_place.py    # 抓取放置示例
├── welding_path.py      # 焊接路径示例
├── test_opengl.py       # OpenGL 环境诊断工具
└── gui_demo/            # 3D 可视化演示
    ├── README.md
    └── python_viewer.py # Python 3D 查看器
```

## 环境准备

运行示例前，请确保已安装必要的依赖：

```bash
# 激活虚拟环境（推荐）
python -m venv .venv
.venv\Scripts\activate  # Windows
# source .venv/bin/activate  # Linux/macOS

# 安装核心依赖
pip install numpy scipy jinja2 pyyaml pydantic

# 安装 GUI 依赖（用于 test_opengl.py 和 python_viewer.py）
pip install PySide6 PyOpenGL
```

## 示例说明

### 1. pick_and_place.py - 抓取放置示例

**功能演示：**
- 机器人模型加载（URDF）
- 工具坐标系设置
- 工件坐标系定义
- 关节运动（moveJ）和直线运动（moveL）
- 多品牌后处理器代码生成

**运行方式：**
```bash
python examples/pick_and_place.py
```

**输出内容：**
- 估算的节拍时间
- 5 种机器人品牌的原生代码：
  - URScript（Universal Robots）
  - KUKA KRL
  - ABB RAPID
  - FANUC TP
  - 汇川 InoTeach

**代码要点：**
```python
import orolp as orp

# 加载机器人
robot = orp.Robot("models/ur/ur5e.urdf", name="UR5_1")

# 设置工具坐标系
robot.tool = orp.Pose.from_xyz_rpy(0, 0, 150, 0, 0, 0)

# 链式运动编程
program = (
    robot.moveJ([0, -90, 90, -90, 90, 0], speed=80)
    .then(robot.moveL(pick, speed=50, blend=5))
    .then(robot.moveL(place, speed=80, blend=5))
)

# 仿真并生成代码
traj = program.simulate()
ur_code = program.to_post("ur_polyscope")
```

---

### 2. welding_path.py - 焊接路径示例

**功能演示：**
- 笛卡尔空间路径规划
- 正弦摆动（weave）模式生成
- 连续工艺路径编程
- KUKA 机器人代码导出

**运行方式：**
```bash
python examples/welding_path.py
```

**输出内容：**
- 焊接路径总时长
- 生成 `welding_seam.src` 文件（KUKA KRL 格式）

**代码要点：**
```python
import numpy as np

# 定义焊缝起点和终点
start_pt = np.array([400, 200, 100])
end_pt = np.array([800, 200, 100])

# 生成带摆动的路径点
for i in range(num_points):
    t = i / (num_points - 1)
    base = start_pt + t * (end_pt - start_pt)
    weave = 5 * np.sin(2 * np.pi * 3 * t)  # 5mm 振幅摆动
    pos = base + np.array([0, weave, 0])
    # 创建路点...
```

---

### 3. test_opengl.py - OpenGL 环境诊断

**功能说明：**
- 检测 PySide6 和 PyOpenGL 是否正确安装
- 验证 OpenGL 固定管线是否可用
- 显示 OpenGL 版本和配置信息
- 诊断图形驱动兼容性问题

**运行方式：**
```bash
python examples/test_opengl.py
```

**预期结果：**
- 弹出一个 400x400 窗口
- 显示一个彩色三角形（红、绿、蓝顶点）
- 控制台输出 OpenGL 版本信息

**故障排查：**
| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| "PySide6 导入失败" | 未安装 PySide6 | `pip install PySide6` |
| "PyOpenGL 未安装" | 未安装 PyOpenGL | `pip install PyOpenGL` |
| 窗口显示但无三角形 | 固定管线不可用 | 更新显卡驱动或使用 OpenGL 3.3+ |

---

### 4. gui_demo/python_viewer.py - 3D 机器人查看器

**功能特性：**
- 加载并可视化 URDF 机器人模型
- 实时关节控制（滑块）
- 轨道相机（旋转、缩放）
- 支持多种几何体（圆柱、长方体、球体）
- 零编译，纯 Python 实现

**运行方式：**
```bash
# 使用默认模型
python examples/gui_demo/python_viewer.py

# 指定 URDF 文件
python examples/gui_demo/python_viewer.py models/ur/test_6dof.urdf
```

**操作说明：**
| 操作 | 功能 |
|------|------|
| 左键拖拽 | 旋转视角 |
| 滚轮 | 缩放 |
| 关节滑块 | 调整关节角度 |
| "Zero All" 按钮 | 所有关节归零 |
| "Home Pose" 按钮 | 回到初始位姿 |

**界面布局：**
```
┌─────────────────────────────────────────┬──────────────┐
│                                         │  Joint Control│
│    [3D 机器人视图]                      │  ────────────│
│                                         │  J1: 0.0°    │
│    旋转：左键拖拽                       │  [━━━━━━━━]  │
│    缩放：滚轮                           │  J2: -90.0°  │
│                                         │  [━━━━━━━━]  │
│                                         │  ...         │
│                                         │              │
│                                         │ [Zero All]   │
│                                         │ [Home Pose]  │
└─────────────────────────────────────────┴──────────────┘
```

详细说明请参阅 [gui_demo/README.md](gui_demo/README.md)。

## 快速开始

### 最小示例（无需编译）

```bash
# 1. 安装依赖
pip install numpy scipy jinja2 pyyaml pydantic

# 2. 运行抓取放置示例（需要先编译 C++ 核心）
python examples/pick_and_place.py
```

### 3D 可视化（零编译）

```bash
# 1. 安装 GUI 依赖
pip install PySide6 PyOpenGL numpy

# 2. 运行 3D 查看器
python examples/gui_demo/python_viewer.py models/ur/test_6dof.urdf
```

### 完整编译（包含 C++ 核心）

```bash
# Ubuntu 22.04/24.04
sudo apt install build-essential cmake libeigen3-dev \
    libpinocchio-dev libfcl-dev pybind11-dev python3-dev

mkdir build && cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON
make -j$(nproc)
cd ..

# 运行示例
python examples/pick_and_place.py
```

## 常见问题

### Q: 运行 pick_and_place.py 提示 "No module named 'orolp'"

**A:** 需要先编译 C++ 核心模块。请按照上述"完整编译"步骤操作。

### Q: python_viewer.py 无法显示机器人

**A:** 检查以下几点：
1. URDF 文件路径是否正确
2. URDF 文件格式是否有效
3. 查看控制台是否有错误信息

### Q: test_opengl.py 显示黑屏

**A:** 可能是显卡驱动问题。尝试：
1. 更新显卡驱动
2. 检查是否支持 OpenGL 2.1
3. 尝试使用软件渲染

## 进阶学习

- [后处理器开发指南](../docs/post_dev_guide.md)
- [API 文档](https://openrobolp.readthedocs.io)
- [贡献指南](../CONTRIBUTING.md)

## 技术支持

如有问题，请提交 Issue：https://github.com/your-org/openrobolp/issues