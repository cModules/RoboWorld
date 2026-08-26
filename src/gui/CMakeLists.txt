# GUI Sub-module: Qt6 + OpenGL 3D Viewer

find_package(Qt6 REQUIRED COMPONENTS Core Widgets OpenGLWidgets OpenGL)
find_package(assimp QUIET)  # Optional: for mesh loading

set(GUI_SOURCES
    main.cpp
    main_window.cpp
    viewport_3d.cpp
    robot_loader.cpp
    joint_panel.cpp
    shader_manager.cpp
)

set(GUI_HEADERS
    main_window.h
    viewport_3d.h
    robot_loader.h
    joint_panel.h
    shader_manager.h
)

qt_add_executable(orolp_viewer
    ${GUI_SOURCES}
    ${GUI_HEADERS}
)

target_link_libraries(orolp_viewer PRIVATE
    Qt6::Core
    Qt6::Widgets
    Qt6::OpenGLWidgets
    Qt6::OpenGL
    orolp_core
    Eigen3::Eigen
)

if(assimp_FOUND)
    target_link_libraries(orolp_viewer PRIVATE assimp::assimp)
    target_compile_definitions(orolp_viewer PRIVATE HAS_ASSIMP)
endif()

target_include_directories(orolp_viewer PRIVATE
    ${CMAKE_SOURCE_DIR}/src/core
    ${CMAKE_SOURCE_DIR}/src/gui
)

set_target_properties(orolp_viewer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
