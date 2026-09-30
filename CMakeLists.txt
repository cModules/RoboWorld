cmake_minimum_required(VERSION 3.16)
project(robotworld LANGUAGES CXX)   # 版本号单一来源在 support/__init__.py，这里不再重复

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
# 注意：曾经有 BUILD_GUI 开关与 src/gui/（Qt6 C++ 界面）——与 Python 侧 GUI
# 完全重复、且从未构建过，已按 ADR-002/005 删除（GUI 只存在于 app/）。
#
# 布局（2026-09-26）：C++ 加速层源码放在 **`accel/src/`**——`accel/` 是**纯硬件加速层**
# （只放 C++/GPU 与唯一边界 `accel/__init__.py`，守护 H7 强制）。历史沿革：
# 原 `src/core/` 与 `core/` 同名同源 → 合并进 `core/` → 2026-09-26 更名 `accel/`
# 并收窄为纯加速层（`i18n`/`features`/`render` 已迁往 `support`/`app`/`adapter`，见 ADR-007/008）。
# `*.cpp` / `*.h` 不参与 Python 分层判定，也不会进 wheel（setuptools 只收 .py
# 与 package-data 里声明的资源；`accel/src/` 没有 `__init__.py`，也不是一个包）。
option(BUILD_PYTHON_BINDINGS "Build Python bindings via pybind11" ON)
option(BUILD_TESTS "Build C++ unit tests" ON)

# Find dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)
find_package(fcl REQUIRED)

# Core library
add_library(accel_core STATIC
    accel/src/robot.cpp
    accel/src/motion.cpp
    accel/src/collision.cpp
)

target_include_directories(accel_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(accel_core PUBLIC
    Eigen3::Eigen
    pinocchio::pinocchio
    fcl
)

# Python bindings
if(BUILD_PYTHON_BINDINGS)
    find_package(pybind11 REQUIRED)
    pybind11_add_module(_accel_native
        accel/src/binding.cpp
    )
    target_link_libraries(_accel_native PRIVATE accel_core)
    set_target_properties(_accel_native PROPERTIES OUTPUT_NAME "accel_native")
    install(TARGETS _accel_native DESTINATION accel)
endif()

# Tests
#
# 历史遗留：这里曾写死 ``add_subdirectory(python/tests)``——那是 **Python** 侧的
# 测试目录（里面只有 test_robot.py，没有 CMakeLists.txt），于是
# ``cmake -DBUILD_TESTS=ON`` 直接 configure 失败（CI 的 build-cpp 因此从未真正跑起来）。
# 该 Python 测试已归位到仓库根的 tests/；C++ 单测目录尚未建立，改为**存在才纳入**：
# 没建目录时不阻塞构建，建了自动生效。
if(BUILD_TESTS)
    enable_testing()
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/accel/src/tests/CMakeLists.txt")
        add_subdirectory(accel/src/tests)
    else()
        message(STATUS "accel/src/tests 尚未建立：跳过 C++ 单测（ctest 将无测试可跑）")
    endif()
endif()

# Install headers
# 显式列文件（不再 install(DIRECTORY ...)）：C++ 源码与 Python 内核同在 `accel/` 树下，
# 整目录安装会去遍历一堆无关文件——虽然 FILES_MATCHING 能过滤掉，但列出来意图更清楚。
install(FILES
        accel/src/robot.h
        accel/src/motion.h
        accel/src/collision.h
        DESTINATION include/robotworld/accel)
install(TARGETS accel_core DESTINATION lib)
