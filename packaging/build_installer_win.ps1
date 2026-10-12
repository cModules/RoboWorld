# RoboWorld Windows 安装器构建脚本
#
# 前置：先跑 packaging\build_win64.ps1 产出 dist\RoboWorld.exe；本脚本只负责
# 把它包成安装包（Inno Setup）。顺序反了会得到"打包成功但装完是空壳"——
# 所以这里**硬校验** dist\RoboWorld.exe 存在，而不是静默打一个空安装器。
#
# 版本号不在这里手写：从 support.__version__ 读（与 app.spec、pyproject 同源），
# 通过 ISCC 的 /D 传进 setup.iss。历史上版本号四处手写，v1.0.1 就漏改过。

$ErrorActionPreference = "Stop"
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

Write-Host "========================================" -ForegroundColor Cyan
Write-Host " RoboWorld 安装器构建 (Windows)" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# 路径：$PSScriptRoot = packaging/，仓库根是它的上一级
$ScriptDir = $PSScriptRoot
$RepoRoot  = Split-Path -Parent $ScriptDir
$DistDir   = Join-Path $RepoRoot "dist"
$IssFile   = Join-Path $ScriptDir "windows\setup.iss"
$ExePath   = Join-Path $DistDir "RoboWorld.exe"

# 1. 校验 PyInstaller 产物
if (-not (Test-Path $ExePath)) {
    Write-Host "找不到 $ExePath" -ForegroundColor Red
    Write-Host "请先运行 packaging\build_win64.ps1 生成可执行文件，再构建安装器。" -ForegroundColor Yellow
    exit 1
}
if (-not (Test-Path $IssFile)) {
    Write-Host "找不到 $IssFile —— 它是版本化配置，请从 git 恢复。" -ForegroundColor Red
    exit 1
}

# 2. 版本号取自代码（唯一真源）
$venvPython = Join-Path $RepoRoot ".venv\Scripts\python.exe"
$pythonExe  = if (Test-Path $venvPython) { $venvPython } else { "python" }

Push-Location $RepoRoot
try {
    $Version = (& $pythonExe -c "from support import __version__; print(__version__)").Trim()
} finally {
    Pop-Location
}
if (-not $Version) {
    Write-Host "无法读取 support.__version__（Python 环境不可用？）" -ForegroundColor Red
    exit 1
}
Write-Host "版本号: $Version" -ForegroundColor Green

# 3. 找 ISCC（Inno Setup 6 编译器）
$iscc = $null
$isccCmd = Get-Command ISCC.exe -ErrorAction SilentlyContinue
if ($isccCmd) { $iscc = $isccCmd.Source }
if (-not $iscc) {
    $candidates = @(
        (Join-Path ${env:ProgramFiles(x86)} "Inno Setup 6\ISCC.exe"),
        (Join-Path $env:ProgramFiles "Inno Setup 6\ISCC.exe")
    )
    foreach ($c in $candidates) {
        if ($c -and (Test-Path $c)) { $iscc = $c; break }
    }
}
if (-not $iscc) {
    Write-Host ""
    Write-Host "未找到 Inno Setup 6（ISCC.exe）。安装方式二选一：" -ForegroundColor Yellow
    Write-Host "  winget install JRSoftware.InnoSetup.6" -ForegroundColor White
    Write-Host "  https://jrsoftware.org/isdl.php" -ForegroundColor White
    Write-Host ""
    Write-Host "装好后重新运行本脚本。注意：Inno Setup 6 以下版本缺 WizardStyle 等指令。" -ForegroundColor Yellow
    exit 1
}
Write-Host "编译器: $iscc" -ForegroundColor Green

# 4. 编译。只传版本号：dist 路径用 setup.iss 里的相对默认值（..\..\dist），
#    避免仓库路径含空格时 /D 传值的引号处理踩坑。
Write-Host ""
Write-Host "开始编译安装器..." -ForegroundColor Cyan
& $iscc "/DMyAppVersion=$Version" "`"$IssFile`""
if ($LASTEXITCODE -ne 0) {
    Write-Host "ISCC 编译失败（退出码 $LASTEXITCODE）" -ForegroundColor Red
    exit $LASTEXITCODE
}

# 5. 校验产物
$SetupExe = Join-Path $DistDir "RoboWorld-Setup-$Version.exe"
if (Test-Path $SetupExe) {
    $fileSize = (Get-Item $SetupExe).Length / 1MB
    Write-Host ""
    Write-Host "========================================" -ForegroundColor Green
    Write-Host " 安装器构建成功！" -ForegroundColor Green
    Write-Host "========================================" -ForegroundColor Green
    Write-Host ""
    Write-Host "安装包: $SetupExe" -ForegroundColor White
    Write-Host "大小: $([math]::Round($fileSize, 2)) MB" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "验证清单：" -ForegroundColor Cyan
    Write-Host "  1. 装完后双击一个 .rwproj，应直接在本程序里打开（不再弹『如何打开』）；" -ForegroundColor White
    Write-Host "  2. 资源管理器里该文件的类型列显示 RoboWorld Project；" -ForegroundColor White
    Write-Host "  3. 程序已在运行时再双击工程，应切到已有窗口而不是再开一个；" -ForegroundColor White
    Write-Host "  4. 控制面板卸载后，.rwproj 的关联应被清掉。" -ForegroundColor White
} else {
    Write-Host ""
    Write-Host "编译结束，但没找到预期的 $SetupExe" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "按任意键退出..."
$null = $Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown")
