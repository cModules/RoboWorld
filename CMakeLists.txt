cmake_minimum_required(VERSION 3.16)
project(openrobolp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(BUILD_GUI "Build Qt6 GUI" OFF)
option(BUILD_PYTHON_BINDINGS "Build Python bindings via pybind11" ON)
option(BUILD_TESTS "Build unit tests" ON)

# Find dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)
find_package(fcl REQUIRED)

# Core library
add_library(orolp_core STATIC
    src/core/robot.cpp
    src/core/motion.cpp
    src/core/collision.cpp
)

target_include_directories(orolp_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/core>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(orolp_core PUBLIC
    Eigen3::Eigen
    pinocchio::pinocchio
    fcl
)

# Python bindings
if(BUILD_PYTHON_BINDINGS)
    find_package(pybind11 REQUIRED)
    pybind11_add_module(_orolp_core
        src/core/binding.cpp
    )
    target_link_libraries(_orolp_core PRIVATE orolp_core)
    set_target_properties(_orolp_core PROPERTIES OUTPUT_NAME "orolp_core")
    install(TARGETS _orolp_core DESTINATION orolp)
endif()

# GUI (optional)
if(BUILD_GUI)
    find_package(Qt6 REQUIRED COMPONENTS Core Widgets OpenGLWidgets)
    add_subdirectory(src/gui)
    message(STATUS "Building GUI module")
endif()

# Tests
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(python/tests)
endif()

# Install headers
install(DIRECTORY src/core/ DESTINATION include/orolp/core
        FILES_MATCHING PATTERN "*.h")
install(TARGETS orolp_core DESTINATION lib)
