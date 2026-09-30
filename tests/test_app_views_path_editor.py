"""`PathCodeEditor` 拆解（ADR-002 Step 1）的守门测试。

拆解这件事本身没有功能行为可测（类原文一字未改），真正会退化的是**结构纪律**，
所以这里钉住三条——缺一条，搬迁就会在两三次提交内慢慢烂回去：

1. **不断链**：``main.PathCodeEditor`` 必须仍指向同一个类。入口脚本是历史调用面
   （15 个测试/工具用 ``main.PathCodeEditor``），re-export 是**有意保留**的过渡约定，
   与 ``test_app_views_robot_renderer.py`` 同一条思路；
2. **方向**：``app/views/path_editor.py`` 不得 import ``main``。方向是**单向**的
   （入口 import 视图）；一旦反过来，入口脚本的任何改动都会把视图拖下水；
3. **自带依赖**：``service`` 侧的可选依赖与降级标志（``_HAS_CELL`` / ``_HAS_PICKPLACE``）
   必须在新模块的命名空间里，**不能靠 main 提供**——否则视图在 main 之外（单测、
   未来 CLI）就不可用。
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import pytest

pytest.importorskip("PySide6")

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

_MOD = _ROOT / "app" / "views" / "path_editor.py"


def test_class_moved_to_views_but_still_reachable_from_entry():
    """① 搬迁完成 + 入口不断链。"""
    import main

    from app.views import path_editor

    assert (
        main.PathCodeEditor is path_editor.PathCodeEditor
    ), "main.PathCodeEditor 断链了——搬迁后入口必须继续 re-export（见本文件头注释）"
    assert (
        path_editor.PathCodeEditor.__module__ == "app.views.path_editor"
    ), "类的归属模块不对：应已迁入 app.views.path_editor"


def test_view_does_not_import_entry_shell():
    """② 依赖方向：视图不得反向 import 入口脚本。"""
    tree = ast.parse(_MOD.read_text(encoding="utf-8"))
    imported: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {a.name for a in node.names}
        elif isinstance(node, ast.ImportFrom) and not node.level and node.module:
            imported.add(node.module)
    bad = {m for m in imported if m == "main" or m.startswith("main.")}
    assert not bad, f"app/views/path_editor.py 反向依赖了入口脚本：{bad}"


def test_view_owns_its_optional_dependencies():
    """③ 降级块跟着类走：可选依赖与标志由视图模块自己提供。"""
    from app.views import path_editor

    for flag in ("_HAS_CELL", "_HAS_PICKPLACE"):
        assert hasattr(path_editor, flag), f"{flag} 应由视图模块自己定义（不能靠 main）"
    for name in (
        "CellConfig",
        "Device",
        "SignalMap",
        "PointType",
        "build_pick_place",
        "generate_scl",
        "steps_from_program",
        "_feature_enabled",
        "tr",
    ):
        assert hasattr(path_editor, name), f"{name} 未带进视图模块命名空间"


def test_module_needs_no_names_from_entry_shell():
    """⑤ 搬迁完整性：本模块用到的名字必须能自己解析，不得指望入口脚本提供。

    与 ``test_app_views_gl_widget.py`` 同一条纪律——那里踩过真事故
    （模块级函数 ``_segments_for_radius`` 漏搬，测试却全绿）。
    """
    import main

    from app.views import path_editor

    tree = ast.parse(_MOD.read_text(encoding="utf-8"))
    defined = {"self"}
    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            defined |= {a.asname or a.name.split(".")[0] for a in node.names}
        elif isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
            defined.add(node.id)
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            defined.add(node.name)
        elif isinstance(node, ast.ExceptHandler) and node.name:
            defined.add(node.name)
    used = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)}
    missing = sorted((used & set(dir(main))) - defined)
    assert not missing, f"这些名字本模块自己解析不了、只能靠入口脚本提供（搬迁漏了）：{missing}"


def test_cjk_baseline_registered_in_guard():
    """结构门禁同步：新文件必须在守护的 CJK 基线与存量债务表里有条目。

    这条拦的是**搬迁时最容易漏的一步**：中文串跟着代码进了新文件，而守卫按文件记账，
    漏登记就会让整个仓库报一堆无关的硬违规（实测过）。
    """
    guard = _ROOT / "tools" / "check_arch_rules.py"
    text = guard.read_text(encoding="utf-8")
    assert (
        '"app/views/path_editor.py"' in text
    ), "新文件未登记进 LEGACY_DEBT / CJK_BASELINE —— 拆解搬移必须同步守护脚本"
