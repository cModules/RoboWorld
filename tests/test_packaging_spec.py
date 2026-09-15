"""打包配置回归测试。

背景（2026-09-14 里程碑发现）：`app.py` 运行时会把 `<repo>/python` 插进
`sys.path`，再 `import orolp.*`；而 `app.spec` 的 `pathex` 只有仓库根目录，
PyInstaller 因此解析不到 `orolp` 包——冻结后的应用在 app.py 里那一圈
`try/except Exception: _HAS_XXX = False` 里**静默降级**，能启动但机器人库、
场景建模、离线编程全部失效（表面"冒烟存活"，实际是个空壳）。

这类缺陷不会让任何功能测试变红，只能靠对 spec 的静态约束守住。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SPEC = REPO_ROOT / "app.spec"

# 本文件里有两个用例要真的 import orolp.world.templates（验冻结资源根解析）。
# 之前它们依赖别的测试模块先把 python/ 插进 sys.path——单独跑本文件时就会
# ModuleNotFoundError。自己插一次，保证单文件也能跑。
_SRC = str(REPO_ROOT / "python")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)


def _spec_text() -> str:
    return SPEC.read_text(encoding="utf-8")


def test_spec_exists():
    assert SPEC.is_file(), "app.spec 缺失，无法打包"


def test_pathex_covers_orolp_source_dir():
    """pathex 必须包含 python/，否则冻结包里没有 orolp。"""
    text = _spec_text()
    assert re.search(r"pathex=\[[^\]]*'python'", text), (
        "app.spec 的 pathex 未包含仓库内的 python/ 目录——"
        "打包后 orolp 包会缺失，应用静默降级为空壳"
    )


def test_scipy_not_excluded():
    """运动学顶层依赖 scipy，excludes 里不能再出现它。"""
    text = _spec_text()
    excludes = re.search(r"excludes=\[(.*?)\]", text, re.S)
    assert excludes, "app.spec 缺少 excludes 段"
    assert "scipy" not in excludes.group(1), (
        "scipy 被 excludes 排除，但 orolp/kinematics.py 顶层 import scipy，"
        "打包后运动学不可用"
    )


def test_core_hiddenimports_present():
    """jinja2（后处理模板）与核心 orolp 子模块必须显式声明。

    ``orolp.ocpcompat`` 是 v1.2.1 新增的 OCP 版本兼容层，由 ``orolp.cam`` 以函数级
    相对导入引用——和 ``orolp.cam`` 同属「必须显式声明，否则冻结包里悄悄缺失」的那类。
    """
    text = _spec_text()
    hidden = re.search(r"hiddenimports=\[(.*?)\],\s*\n\s*hookspath", text, re.S)
    assert hidden, "app.spec 缺少 hiddenimports 段"
    body = hidden.group(1)
    for name in ("jinja2", "'orolp'", "orolp.calibration", "orolp.cam", "orolp.ocpcompat"):
        assert name in body, f"hiddenimports 缺少 {name}"


def test_datas_include_models_and_examples():
    """models/ 与 examples/ 必须随包分发（机器人库与示例程序都要用）。"""
    text = _spec_text()
    datas = re.search(r"datas=\[(.*?)\],\s*\n\s*hiddenimports", text, re.S)
    assert datas, "app.spec 缺少 datas 段"
    body = datas.group(1)
    assert "'models'" in body
    assert "'examples'" in body


def test_spec_python_syntax_ok():
    """spec 至少要是可编译的 Python（PyInstaller 会 exec 它）。"""
    import ast

    ast.parse(_spec_text())


def test_ocp_excluded_to_avoid_bloat():
    """OCP（OpenCASCADE）必须排除：它只有 CAD/CAM 函数内的软依赖用到，
    不排除会被 orolp.cam 拖进包，包体从 66MB 涨到 166MB。"""
    text = _spec_text()
    excludes = re.search(r"excludes=\[(.*?)\]", text, re.S)
    assert excludes, "app.spec 缺少 excludes 段"
    body = excludes.group(1)
    assert "'OCP'" in body or '"OCP"' in body, "OCP 未排除，打包体积会翻倍"


def test_templates_repo_root_follows_meipass(monkeypatch, tmp_path):
    """冻结后资源根必须取 sys._MEIPASS（models/ 在那里），不能算到包外。

    源码树里 ``python/orolp/world/templates.py`` 上三级是仓库根，但冻结后
    ``__file__`` 位于 ``_MEIPASS/orolp/world/templates.py``，上三级会跑到
    ``_MEIPASS`` 的父目录（临时目录）。此时若仍用上三级，模板里的
    ``models/ur5e/ur5e.urdf`` 就找不到，打包版「一键抓取工作站」直接失效。
    """
    from orolp.world import templates

    fake_root = tmp_path / "meipass"
    (fake_root / "models").mkdir(parents=True)
    monkeypatch.setattr(sys, "_MEIPASS", str(fake_root), raising=False)
    monkeypatch.setattr(templates, "__file__", str(fake_root / "orolp" / "world" / "templates.py"))
    assert templates.repo_root() == fake_root


def test_templates_repo_root_prefers_source_tree_without_meipass():
    """非冻结运行时仍指向真实仓库根（含 models/）。"""
    from orolp.world import templates

    if hasattr(sys, "_MEIPASS"):
        return  # 本用例只针对源码运行
    root = templates.repo_root()
    assert (root / "models").is_dir()
    assert (root / "app.py").is_file()

