# -*- mode: python ; coding: utf-8 -*-
"""
RoboWorld - PyInstaller Spec File

**本文件位于 ``packaging/``**（P0-3 目录重整），因此所有资源路径都必须先算出
仓库根：``SPECPATH`` 是 ``packaging/``，往上**一级**才是仓库根。早先它躺在仓库
根、直接拿 ``os.path.dirname(SPEC)`` 当仓库根用；搬进来后若照抄，``models/``、
``examples/``、``python/`` 全会指到 ``packaging/`` 下——PyInstaller 找不到就
静默少打（应用能启动、功能悄悄失效）。

跨平台打包：
* Windows / Linux：产出 RoboWorld 可执行文件，图标取 logo.png
* macOS：额外生成 RoboWorld.app，**Dock 名称与图标由 Info.plist 的
  CFBundleName 和 logo.icns 决定**（需先由 logo.png 生成 logo.icns）
"""

block_cipher = None

import os
_pkg_dir = os.path.dirname(os.path.abspath(SPEC))     # …/packaging
_repo_root = os.path.dirname(_pkg_dir)                # 仓库根

# 版本号从**代码**读，不在这里手写（与 pyproject 的 dynamic/attr 同源）：
# 历史上有四处手写（本文件 / pyproject / 包内 __version__ / 窗口标题），漏一处就对不上。
# support 零依赖，spec 求值期直接 import 是安全的。
import sys
sys.path.insert(0, _repo_root)
from support import __version__ as _VERSION           # noqa: E402

a = Analysis(
    # 入口是根启动壳 main.py（原 app.py：改名是为了不与顶层 GUI 包 app/ 同名——
    # 同目录下重名时 Python 取**包**，`import app` 会从"启动壳"悄悄变成"GUI 包"）
    [os.path.join(_repo_root, 'main.py')],
    # 引擎各层（core/ 硬件加速 + domain/service/adapter/support）与顶层包 app/
    # 都在仓库根下——所以 pathex 只需仓库根。早先它还依赖 <repo>/python 那一层；
    # 写成错的路径不会报错，只会让冻结包**静默降级**
    # （机器人库/场景/OLP 全失效，表面"冒烟存活"）。
    pathex=[_repo_root],
    binaries=[],
    datas=[
        (os.path.join(_pkg_dir, 'logo.png'), '.'),
        # 功能开关清单放在 pyproject.toml 里：打包后 app.features 也要读得到，
        # 否则冻结版会用兜底清单——开发机上改了文件、安装包却还是老口径。
        (os.path.join(_repo_root, 'pyproject.toml'), '.'),
        (os.path.join(_repo_root, 'examples'), 'examples'),
        # 应用资源整目录：assets/cells（预置工作站）+ assets/models（机器人模型库）。
        # 2026-09-26 目录重整：模型库由仓库根 models/ 并入 assets/models/，**这一条
        # 已覆盖两者**，原先单独的 models 条目已删除（那条路径已不存在，留着会让
        # PyInstaller 面对缺失源——不同版本行为不同，不该赌）。
        # 漏了 assets 冻结版会弹"未找到预置场景"、机器人库也会空——单元测试全绿，
        # 只有真打包才暴露。
        (os.path.join(_repo_root, 'assets'), 'assets'),
        (os.path.join(_repo_root, 'README.md'), '.'),
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
        'accel',
        # 顶层 GUI 包 app/（2026-09-26 桌面应用独立）：main.py 以 ``from app.views…``
        # 导入，PyInstaller 的静态分析对"运行时插 sys.path"的包不可靠，显式登记
        'app',
        'app.views',
        'app.views.robot_renderer',
        'app.views.gl_widget',
        'app.views.path_editor',
        'app.views.property_panel',
        'app.views.property_panel_ui',
        'app.views.property_controls',
        # 功能展示开关（2026-09-26 从 core 迁来）：main.py 里是 try/except 导入，
        # 显式登记避免被 PyInstaller 判为未使用而漏打
        'app.features',
        # 三层（2026-09-26 由 core 拆出）：main.py 里多处是 try/except 导入，
        # 显式登记可以避免"某个子模块被 PyInstaller 判为未使用"而漏打
        'domain',
        'service',
        'adapter',
        # 跨层共享支撑层（2026-09-26 新建）：i18n 被各层 import，且门面是惰性解析
        'support',
        'support.i18n',
        # 场景 / 模型资产（URDF 解析 + 网格 + 几何原语）／后处理
        'domain.world',
        'adapter.robot_library',
        'adapter.robot_library.mesh',
        'adapter.robot_library.primitives',
        'adapter.robot_library.urdf_model',
        'adapter.post',
        'service.calibration',
        'service.cam',
        'domain.collision',
        'adapter.ocpcompat',
        'domain.dynamics',
        'service.process',
        'service.project',
        'service.sequence',
        # 第二轮归位（2026-09-26）：core/ 剩余文件按分层判定各归其位
        'app.cli',
        'app.session',
        'app.views.controller_panel',
        'app.views.program_panel',
        'domain.kinematics',
        'service.debug',
        'service.perf',
        'service.project_templates',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        'tkinter',
        'matplotlib',
        'PIL',
        # OCP = OpenCASCADE 内核（cadquery-ocp），只有 CAD/CAM 通道用，且是
        # 函数内的软依赖（未装时 service.cam 的纯 numpy 部分照常可用）。
        # 不排除的话会被 service.cam 拖进来，包体 66MB → 166MB。
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
    name='RoboWorld',
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
    icon=os.path.join(_pkg_dir, 'logo.png'),
)

# --------------------------------------------------------------------------- #
# macOS：生成正式 .app
#
# 没有这一段时，macOS 下只会产出裸可执行文件，Dock 上显示 python 图标与
# "python3.11"。有了 BUNDLE + info_plist，Dock 名称取自 CFBundleName、
# 图标取自 logo.icns —— 这是 macOS 上唯一确定的生效方式。
# 非 macOS 平台 PyInstaller 会忽略该段，不影响 Windows 打包。
# --------------------------------------------------------------------------- #
_icns = os.path.join(_pkg_dir, 'logo.icns')

app = BUNDLE(
    exe,
    name='RoboWorld.app',
    icon=_icns if os.path.exists(_icns) else None,
    bundle_identifier='com.openrobot.work.robotworld',
    info_plist={
        'CFBundleName': 'RoboWorld',
        'CFBundleDisplayName': 'RoboWorld',
        'CFBundleShortVersionString': _VERSION,
        'NSHighResolutionCapable': True,
    },
)
