"""打包配置回归测试。

背景（2026-09-14 里程碑发现）：`app.py` 运行时会把 `<repo>/python` 插进
`sys.path`，再 `import accel.*` / `import domain.*`；而 `app.spec` 的 `pathex` 只有仓库根目录，
PyInstaller 因此解析不到 `core` 包——冻结后的应用在 app.py 里那一圈
`try/except Exception: _HAS_XXX = False` 里**静默降级**，能启动但机器人库、
场景建模、离线编程全部失效（表面"冒烟存活"，实际是个空壳）。

这类缺陷不会让任何功能测试变红，只能靠对 spec 的静态约束守住。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
#: P0-3 目录重整后 spec 位于 packaging/（与 logo / 打包脚本同处）。
#: 注意 spec 内部必须自己算仓库根（SPECPATH 的上一级）——本测试只校路径，
#: 真跑打包由 CI 的 wheel/冻结冒烟负责。
SPEC = REPO_ROOT / "packaging" / "app.spec"

# 本文件里有两个用例要真的 import domain.world.templates（验冻结资源根解析）。
# 之前它们依赖别的测试模块先把 python/ 插进 sys.path——单独跑本文件时就会
# ModuleNotFoundError。自己插一次，保证单文件也能跑。
_SRC = str(REPO_ROOT)
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)


def _spec_text() -> str:
    return SPEC.read_text(encoding="utf-8")


def test_spec_exists():
    assert SPEC.is_file(), "app.spec 缺失，无法打包"


def test_pathex_covers_core_source_dir():
    """pathex 必须包含仓库根，否则冻结包里没有 core（引擎包）。

    2026-09-26 起引擎包是顶层 ``core/``（原 ``core``，``python/`` 这层已取消），
    所以 pathex 只需 ``_repo_root``——但**必须**有它：漏了不会报错，
    只会让冻结包在 try/except 里静默降级成空壳。
    """
    text = _spec_text()
    assert re.search(
        r"pathex=\[_repo_root\]", text
    ), "app.spec 的 pathex 未包含仓库根——打包后 core 包会缺失，应用静默降级为空壳"


def test_scipy_not_excluded():
    """运动学顶层依赖 scipy，excludes 里不能再出现它。"""
    text = _spec_text()
    excludes = re.search(r"excludes=\[(.*?)\]", text, re.S)
    assert excludes, "app.spec 缺少 excludes 段"
    assert "scipy" not in excludes.group(1), (
        "scipy 被 excludes 排除，但 domain/kinematics.py 顶层 import scipy，" "打包后运动学不可用"
    )


def test_accel_hiddenimports_present():
    """jinja2（后处理模板）与加速层/三层子模块必须显式声明。

    ``adapter.ocpcompat`` 是 OCP 版本兼容层（2026-09-26 由 ``core/`` 按分层判定归位到
    Adapter），由 ``service.cam`` 以函数级导入引用——和 ``service.cam`` 同属
    「必须显式声明，否则冻结包里悄悄缺失」的那类。
    """
    text = _spec_text()
    hidden = re.search(r"hiddenimports=\[(.*?)\],\s*\n\s*hookspath", text, re.S)
    assert hidden, "app.spec 缺少 hiddenimports 段"
    body = hidden.group(1)
    for name in (
        "jinja2",
        "'accel'",
        "'support'",
        "service.calibration",
        "service.cam",
        "adapter.ocpcompat",
        "domain.kinematics",
        "app.cli",
    ):
        assert name in body, f"hiddenimports 缺少 {name}"


def test_datas_include_models_and_examples():
    """模型库与示例必须随包分发。

    2026-09-26 目录重整后模型库在 ``assets/models/``——由 datas 里的 ``assets``
    整目录条目覆盖（原先单独的 ``models`` 条目已删，那条路径不再存在）。
    这里因此断言 ``assets`` 与 ``examples``；"模型库确实在 assets 下"由文件系统
    断言守住（见下一个用例）。
    """
    text = _spec_text()
    datas = re.search(r"datas=\[(.*?)\],\s*\n\s*hiddenimports", text, re.S)
    assert datas, "app.spec 缺少 datas 段"
    body = datas.group(1)
    assert "'assets'" in body, "模型库随 assets 整目录分发，缺它机器人库会空"
    assert "'examples'" in body
    assert (
        REPO_ROOT / "assets" / "models"
    ).is_dir(), "模型库不在 assets/models/ 下——datas 的 assets 条目就覆盖不到它"


def test_datas_include_assets_for_preset_cells():
    """``assets/`` 必须随包分发——「一键载入预置工作站」读 assets/cells/*.json。

    回归（2026-09-25 目录重整时发现）：datas 里只有 assets/models/examples，
    ``app.py`` 却按 ``Path(__file__).parent / "assets" / "cells"`` 去找预置场景——
    冻结包里那个目录**根本不存在**，功能直接失效，而单元测试全绿（源码树里有）。
    """
    text = _spec_text()
    datas = re.search(r"datas=\[(.*?)\],\s*\n\s*hiddenimports", text, re.S)
    assert datas, "app.spec 缺少 datas 段"
    assert "'assets'" in datas.group(
        1
    ), "datas 缺少 assets/：冻结版载入预置工作站会报「未找到预置场景」"


def test_spec_resolves_repo_root_from_packaging_dir():
    """spec 必须自己算出仓库根，而不是把 packaging/ 当仓库根。

    搬进 ``packaging/`` 之后，``os.path.dirname(SPEC)`` 是 ``packaging/``；
    照抄旧写法会让 assets/models/examples/python 全部指向 ``packaging/`` 下——
    PyInstaller 找不到就静默少打，应用能启动但功能悄悄失效。
    """
    text = _spec_text()
    assert (
        "os.path.dirname(_pkg_dir)" in text or "_repo_root = os.path.dirname" in text
    ), "spec 未从 packaging/ 往上算仓库根"
    assert "pathex=[_repo_root" in text, "pathex 未使用算出来的仓库根"


def test_spec_python_syntax_ok():
    """spec 至少要是可编译的 Python（PyInstaller 会 exec 它）。"""
    import ast

    ast.parse(_spec_text())


def test_ocp_excluded_to_avoid_bloat():
    """OCP（OpenCASCADE）必须排除：它只有 CAD/CAM 函数内的软依赖用到，
    不排除会被 service.cam 拖进包，包体从 66MB 涨到 166MB。"""
    text = _spec_text()
    excludes = re.search(r"excludes=\[(.*?)\]", text, re.S)
    assert excludes, "app.spec 缺少 excludes 段"
    body = excludes.group(1)
    assert "'OCP'" in body or '"OCP"' in body, "OCP 未排除，打包体积会翻倍"


def test_templates_repo_root_follows_meipass(monkeypatch, tmp_path):
    """冻结后资源根必须取 sys._MEIPASS（assets/models/ 在那里），不能算到包外。

    源码树里 ``domain/world/templates.py`` 上三级是仓库根，但冻结后
    ``__file__`` 位于 ``_MEIPASS/domain/world/templates.py``，上三级会跑到
    ``_MEIPASS`` 的父目录（临时目录）。此时若仍用上三级，模板里的
    ``assets/models/ur5e/ur5e.urdf`` 就找不到，打包版「一键抓取工作站」直接失效。
    """
    from domain.world import templates

    fake_root = tmp_path / "meipass"
    (fake_root / "assets" / "models").mkdir(parents=True)
    monkeypatch.setattr(sys, "_MEIPASS", str(fake_root), raising=False)
    monkeypatch.setattr(templates, "__file__", str(fake_root / "domain" / "world" / "templates.py"))
    assert templates.repo_root() == fake_root


def test_templates_repo_root_prefers_source_tree_without_meipass():
    """非冻结运行时仍指向真实仓库根（含 assets/models/）。"""
    from domain.world import templates

    if hasattr(sys, "_MEIPASS"):
        return  # 本用例只针对源码运行
    root = templates.repo_root()
    assert (root / "assets" / "models").is_dir()
    assert (root / "main.py").is_file()


def test_bundle_identifier_matches_app_identity():
    """macOS 的 bundle id 必须与 :data:`app.app_identity.APPLICATION_ID` 同源。

    三端身份（Windows AppUserModelID / macOS bundle id / 单实例 socket 名）各写
    一份的后果见 ``app/app_identity.py``：任务栏把两个窗口分到两组、Jump List 的
    "最近"挂到别的条目下、文件关联指向一个不存在的身份——三种都表现为"随机的"。

    spec 里那一份由这条断言守住（与"仓库根算错"同一条纪律：错的打包配置
    不会让任何单元测试变红，只能静态守）。
    """
    from app.app_identity import APPLICATION_ID

    match = re.search(r"bundle_identifier\s*=\s*'([^']+)'", _spec_text())
    assert match, "app.spec 缺少 bundle_identifier（macOS 文件关联靠它）"
    assert match.group(1) == APPLICATION_ID, (
        f"spec 的 bundle_identifier {match.group(1)!r} 与 APPLICATION_ID "
        f"{APPLICATION_ID!r} 不一致：三端身份会分裂"
    )
