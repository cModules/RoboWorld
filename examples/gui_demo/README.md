# Qt6 3D 查看器演示

这是 OpenRoboOLP 的一个最小可运行 3D 可视化演示。

## 编译说明

### 前置依赖
- Qt6 (Core, Widgets, OpenGLWidgets, OpenGL)
- CMake 3.16+
- C++17 编译器


### 快速启动（三种方式）
方式一：Python 脚本（无需编译，立即可用）
```bash
pip install numpy scipy jinja2 pyyaml pydantic
# 运行示例
python examples/pick_and_place.py
```

方式二：Python 3D 视窗（零编译）
```bash
pip install PySide6 numpy
python examples/gui_demo/python_viewer.py models/ur/test_6dof.urdf
```

方式三：C++ Qt6 原生视窗（性能最佳）
```bash
sudo apt install qt6-base-dev libqt6opengl6-dev libqt6openglwidgets6 \
    libeigen3-dev libpinocchio-dev libfcl-dev pybind11-dev

mkdir build && cd build
cmake .. -DBUILD_GUI=ON -DBUILD_PYTHON_BINDINGS=ON
make -j$(nproc)
./bin/orolp_viewer ../models/ur/test_6dof.urdf
```


### Ubuntu 22.04/24.04
```bash
sudo apt install qt6-base-dev libqt6opengl6-dev libqt6openglwidgets6
```

### macOS
```bash
brew install qt@6
```

### 编译
```bash
cd openrobolp
mkdir build && cd build
cmake .. -DBUILD_GUI=ON -DBUILD_PYTHON_BINDINGS=ON
make -j$(nproc)
```

### 运行
```bash
./bin/orolp_viewer ../models/ur/test_6dof.urdf
```

## 功能特性
- **轨道相机**：左键拖拽绕机器人旋转
- **缩放**：鼠标滚轮
- **关节滑块**：右侧面板支持实时关节控制
- **回零位**：一键返回标准初始位姿
- **自动加载**：通过命令行参数传入 URDF 路径

## 界面示意图
```
┌─────────────────────────────────────────┬──────────────┐
│                                         │  关节控制    │
│    [3D 机器人视图]                      │  ────────────│
│                                         │  J1: 0.0°    │
│    旋转：左键拖拽                       │  [━━━━━━━]   │
│    缩放：滚轮                           │  J2: -90.0°  │
│                                         │  [━━━━━━━]   │
│                                         │  ...         │
│                                         │              │
│                                         │ [全部归零]   │
│                                         │ [回零位]     │
└─────────────────────────────────────────┴──────────────┘
```
