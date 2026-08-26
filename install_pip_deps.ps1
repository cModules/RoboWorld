# 安装 OpenRoboOLP 所需 Python 依赖

# 优先使用 .venv 虚拟环境，否则使用系统 python
$venvPython = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
if (Test-Path $venvPython) {
    $pythonExe = $venvPython
    Write-Host "使用 .venv 虚拟环境" -ForegroundColor Green
} else {
    $pythonExe = "python"
    Write-Host "使用系统 Python，请确保已激活正确的环境" -ForegroundColor Yellow
}

Write-Host "Installing numpy ..."
& $pythonExe -m pip install numpy

Write-Host "Installing PySide6 ..."
& $pythonExe -m pip install PySide6

Write-Host "Installing PyOpenGL ..."
& $pythonExe -m pip install PyOpenGL

Write-Host ""
Write-Host "Verifying installations ..."
& $pythonExe -c "import numpy; print('numpy:', numpy.__version__)"
& $pythonExe -c "import PySide6; print('PySide6:', PySide6.__version__)"
& $pythonExe -c "import OpenGL.GL; print('PyOpenGL: OK')"

Write-Host ""
Write-Host "All dependencies installed!" -ForegroundColor Green
Write-Host "Press any key to exit ..."
[Console]::ReadKey() | Out-Null
