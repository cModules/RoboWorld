# -*- mode: python ; coding: utf-8 -*-
"""
RobotWorld - PyInstaller Spec File

跨平台打包：
* Windows / Linux：产出 RobotWorld 可执行文件，图标取 logo.png
* macOS：额外生成 RobotWorld.app，**Dock 名称与图标由 Info.plist 的
  CFBundleName 和 logo.icns 决定**（需先由 logo.png 生成 logo.icns）

生成 logo.icns（macOS 自带工具，无需装依赖）：
    for s in 16 32 128 256 512; do
      sips -z $s $s logo.png --out logo.iconset/icon_${s}x${s}.png
      sips -z $((s*2)) $((s*2)) logo.png --out logo.iconset/icon_${s}x${s}@2x.png
    done
    iconutil -c icns logo.iconset -o logo.icns
"""

block_cipher = None

# 获取当前脚本目录
import os
spec_dir = os.path.dirname(os.path.abspath(SPEC))

a = Analysis(
    ['app.py'],
    # app.py 会在运行时把 <repo>/python 插进 sys.path 再 import orolp.*；
    # 打包时 pathex 必须同样包含 python/，否则 PyInstaller 解析不到 orolp，
    # 冻结后的应用会在所有 try/except 里静默降级（机器人库/场景/OLP 全失效）。
    pathex=[spec_dir, os.path.join(spec_dir, 'python')],
    binaries=[],
    datas=[
        ('logo.png', '.'),
        # 功能开关清单放在 pyproject.toml 里：打包后 orolp.features 也要读得到，
        # 否则冻结版会用兜底清单——开发机上改了文件、安装包却还是老口径。
        ('pyproject.toml', '.'),
        ('models', 'models'),
        ('examples', 'examples'),
        ('README.md', '.'),
    ],
    hiddenimports=[
        'PySide6',
        'PySide6.QtCore',
        'PySide6.QtGui',
        'PySide6.QtWidgets',
        'PySide6.QtOpenGLWidgets',
        'PySide6.QtOpenGL',
        'OpenGL',
        'OpenGL.GL',
        'OpenGL.GLUT',
        'OpenGL.GLU',
        'numpy',
        'numpy.core',
        'numpy.core.multiarray',
        'numpy.core._multiarray_umath',
        'scipy',
        'scipy.spatial.transform',
        'scipy.optimize',
        'jinja2',
        'orolp',
        'orolp.calibration',
        'orolp.cam',
        'orolp.collision',
        'orolp.ocpcompat',
        'orolp.dynamics',
        'orolp.process',
        'orolp.project',
        'orolp.sequence',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        'tkinter',
        'matplotlib',
        'PIL',
        # OCP = OpenCASCADE 内核（cadquery-ocp），只有 CAD/CAM 通道用，且是
        # 函数内的软依赖（未装时 orolp.cam 的纯 numpy 部分照常可用）。
        # 不排除的话会被 orolp.cam 拖进来，包体 66MB → 166MB。
        'OCP',
        'cadquery',
    ],
)

pyz = PYZ(a.pure)

# 顺手剔掉 __pycache__：datas 是整目录拷贝，会把开发机上的 .pyc（甚至别的
# Python 版本的）一并塞进安装包，既没用又白白撑大体积。
a.datas = [d for d in a.datas if '__pycache__' not in str(d[0])]

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='RobotWorld',
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

# --------------------------------------------------------------------------- #
# macOS：生成正式 .app
#
# 没有这一段时，macOS 下只会产出裸可执行文件，Dock 上显示 python 图标与
# "python3.11"。有了 BUNDLE + info_plist，Dock 名称取自 CFBundleName、
# 图标取自 logo.icns —— 这是 macOS 上唯一确定的生效方式。
# 非 macOS 平台 PyInstaller 会忽略该段，不影响 Windows 打包。
# --------------------------------------------------------------------------- #
_icns = os.path.join(spec_dir, 'logo.icns')

app = BUNDLE(
    exe,
    name='RobotWorld.app',
    icon=_icns if os.path.exists(_icns) else None,
    bundle_identifier='com.openrobot.work.robotworld',
    info_plist={
        'CFBundleName': 'RobotWorld',
        'CFBundleDisplayName': 'RobotWorld',
        'CFBundleShortVersionString': '1.4.0',
        'NSHighResolutionCapable': True,
    },
)
