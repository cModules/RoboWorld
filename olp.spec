# -*- mode: python ; coding: utf-8 -*-
"""
OpenRoboOLP - PyInstaller Spec File
用于打包 Windows 64位版本
"""

block_cipher = None

# 获取当前脚本目录
import os
spec_dir = os.path.dirname(os.path.abspath(SPEC))

a = Analysis(
    ['olp.py'],
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
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        'tkinter',
        'matplotlib',
        'PIL',
        'scipy',
    ],
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
