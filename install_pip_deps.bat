@echo off
chcp 65001 >nul
echo 正在安装 OpenRoboOLP 依赖 ...

REM 优先使用 .venv 虚拟环境，否则使用系统 python
if exist "%~dp0.venv\Scripts\python.exe" (
    set "PYTHON_EXE=%~dp0.venv\Scripts\python.exe"
    echo 使用 .venv 虚拟环境
) else (
    set "PYTHON_EXE=python"
    echo 使用系统 Python，请确保已激活正确的环境
)

"%PYTHON_EXE%" -m pip install numpy PySide6 PyOpenGL
echo.
echo 安装完成，按任意键退出 ...
pause >nul
