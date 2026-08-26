# OpenRoboOLP Windows 64位打包脚本

Write-Host "========================================" -ForegroundColor Cyan
Write-Host " OpenRoboOLP 打包脚本 (Windows 64-bit)" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# 设置路径
$ScriptDir = $PSScriptRoot
$DistDir = Join-Path $ScriptDir "dist"
$BuildDir = Join-Path $ScriptDir "build"

# 清理旧的构建文件
Write-Host "清理旧的构建文件..." -ForegroundColor Yellow
if (Test-Path $DistDir) {
    Remove-Item -Path $DistDir -Recurse -Force
}
if (Test-Path $BuildDir) {
    Remove-Item -Path $BuildDir -Recurse -Force
}

# 检查虚拟环境
$venvPython = Join-Path $ScriptDir ".venv\Scripts\python.exe"
if (Test-Path $venvPython) {
    $pythonExe = $venvPython
    $pyInstallerExe = Join-Path $ScriptDir ".venv\Scripts\pyinstaller.exe"
    Write-Host "使用 .venv 虚拟环境" -ForegroundColor Green
} else {
    $pythonExe = "python"
    $pyInstallerExe = "pyinstaller"
    Write-Host "使用系统 Python，请确保已激活正确的环境" -ForegroundColor Yellow
}

# 检查 PyInstaller 是否安装
Write-Host ""
Write-Host "检查 PyInstaller..." -ForegroundColor Cyan
try {
    & $pythonExe -c "import PyInstaller"
    Write-Host "PyInstaller 已安装" -ForegroundColor Green
} catch {
    Write-Host "正在安装 PyInstaller..." -ForegroundColor Yellow
    & $pythonExe -m pip install pyinstaller
}

# 创建 spec 文件（如果不存在）
$specFile = Join-Path $ScriptDir "olp.spec"
if (-not (Test-Path $specFile)) {
    Write-Host ""
    Write-Host "创建 PyInstaller spec 文件..." -ForegroundColor Cyan
    $specContent = @"
# -*- mode: python ; coding: utf-8 -*-

block_cipher = None

a = Analysis(
    ['olp.py'],
    pathex=[r'$ScriptDir'],
    binaries=[],
    datas=[
        ('logo.png', '.'),
        ('models', 'models'),
        ('examples', 'examples'),
    ],
    hiddenimports=[
        'PySide6',
        'PySide6.QtCore',
        'PySide6.QtGui',
        'PySide6.QtWidgets',
        'PySide6.QtOpenGLWidgets',
        'OpenGL',
        'OpenGL.GL',
        'numpy',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='OpenRoboOLP',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon='logo.png',
)
"@
    $specContent | Out-File -FilePath $specFile -Encoding utf8
    Write-Host "已创建 olp.spec" -ForegroundColor Green
}

# 执行打包
Write-Host ""
Write-Host "开始打包..." -ForegroundColor Cyan
Write-Host "这可能需要几分钟时间，请耐心等待..." -ForegroundColor Yellow
Write-Host ""

& $pythonExe -m PyInstaller --clean olp.spec

# 检查打包结果
if (Test-Path (Join-Path $DistDir "OpenRoboOLP.exe")) {
    Write-Host ""
    Write-Host "========================================" -ForegroundColor Green
    Write-Host " 打包成功！" -ForegroundColor Green
    Write-Host "========================================" -ForegroundColor Green
    Write-Host ""
    Write-Host "可执行文件位置: " -ForegroundColor Cyan -NoNewline
    Write-Host (Join-Path $DistDir "OpenRoboOLP.exe") -ForegroundColor White
    Write-Host ""
    
    # 计算文件大小
    $exePath = Join-Path $DistDir "OpenRoboOLP.exe"
    $fileSize = (Get-Item $exePath).Length / 1MB
    Write-Host "文件大小: $([math]::Round($fileSize, 2)) MB" -ForegroundColor Yellow
} else {
    Write-Host ""
    Write-Host "========================================" -ForegroundColor Red
    Write-Host " 打包失败！" -ForegroundColor Red
    Write-Host "========================================" -ForegroundColor Red
    Write-Host ""
    Write-Host "请检查错误信息并重试" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "按任意键退出..."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
