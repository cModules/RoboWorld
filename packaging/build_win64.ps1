# RobotWorld Windows 64位打包脚本
#
# 本脚本位于 packaging/（P0-3 目录重整），因此：
#   $PSScriptRoot = …/packaging
#   $RepoRoot     = 它的上一级（仓库根）——venv / dist / build / app.py 都在那里

$ErrorActionPreference = "Stop"
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

Write-Host "========================================" -ForegroundColor Cyan
Write-Host " RobotWorld 打包脚本 (Windows 64-bit)" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# 设置路径
$ScriptDir = $PSScriptRoot                          # packaging/
$RepoRoot  = Split-Path -Parent $ScriptDir          # 仓库根
$DistDir   = Join-Path $RepoRoot "dist"
$BuildDir  = Join-Path $RepoRoot "build"
$specFile  = Join-Path $ScriptDir "app.spec"

# 清理旧的构建文件
Write-Host "清理旧的构建文件..." -ForegroundColor Yellow
if (Test-Path $DistDir) {
    Remove-Item -Path $DistDir -Recurse -Force
}
if (Test-Path $BuildDir) {
    Remove-Item -Path $BuildDir -Recurse -Force
}

# 检查虚拟环境
$venvPython = Join-Path $RepoRoot ".venv\Scripts\python.exe"
if (Test-Path $venvPython) {
    $pythonExe = $venvPython
    Write-Host "使用 .venv 虚拟环境" -ForegroundColor Green
} else {
    $pythonExe = "python"
    Write-Host "使用系统 Python，请确保已激活正确的环境" -ForegroundColor Yellow
}

# 检查 PyInstaller 是否安装
Write-Host ""
Write-Host "检查 PyInstaller..." -ForegroundColor Cyan
& $pythonExe -c "import PyInstaller" 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host "正在安装 PyInstaller..." -ForegroundColor Yellow
    & $pythonExe -m pip install pyinstaller
}

# spec 必须存在。**不要**在这里"自动生成一份"：早先那段内联模板早已过时
# （datas 缺 assets/、pathex 不带 python/、路径按 spec 所在目录解析），
# 生成出来只会静默打出一个空壳包。
if (-not (Test-Path $specFile)) {
    Write-Host "找不到 $specFile —— spec 是版本化配置文件（含 assets/models/python 路径），" -ForegroundColor Red
    Write-Host "请从 git 恢复，不要手工重建。" -ForegroundColor Red
    exit 1
}

# 执行打包（在仓库根执行：spec 内部会自己算仓库根，位置不影响结果）
Write-Host ""
Write-Host "开始打包..." -ForegroundColor Cyan
Write-Host "这可能需要几分钟时间，请耐心等待..." -ForegroundColor Yellow
Write-Host ""

Push-Location $RepoRoot
try {
    & $pythonExe -m PyInstaller --clean --noconfirm $specFile
} finally {
    Pop-Location
}

# 检查打包结果
if (Test-Path (Join-Path $DistDir "RobotWorld.exe")) {
    Write-Host ""
    Write-Host "========================================" -ForegroundColor Green
    Write-Host " 打包成功！" -ForegroundColor Green
    Write-Host "========================================" -ForegroundColor Green
    Write-Host ""
    Write-Host "可执行文件位置: " -ForegroundColor Cyan -NoNewline
    Write-Host (Join-Path $DistDir "RobotWorld.exe") -ForegroundColor White
    Write-Host ""

    # 计算文件大小
    $exePath = Join-Path $DistDir "RobotWorld.exe"
    $fileSize = (Get-Item $exePath).Length / 1MB
    Write-Host "文件大小: $([math]::Round($fileSize, 2)) MB" -ForegroundColor Yellow
} else {
    Write-Host ""
    Write-Host "========================================" -ForegroundColor Red
    Write-Host " 打包失败！" -ForegroundColor Red
    Write-Host "========================================" -ForegroundColor Red
    Write-Host ""
    Write-Host "请检查错误信息并重试" -ForegroundColor Yellow
    exit 1
}

Write-Host ""
Write-Host "按任意键退出..."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
