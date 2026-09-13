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
    ['rw.py'],
    pathex=[spec_dir],
    binaries=[],
    datas=[
        ('logo.png', '.'),
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
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        'tkinter',
        'matplotlib',
        'PIL',
    ],
)

pyz = PYZ(a.pure)

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
        'CFBundleShortVersionString': '0.2.0',
        'NSHighResolutionCapable': True,
    },
)
