"""`GLWidget` 拆解（ADR-002 Step 2）的守门测试。

与 `test_app_views_path_editor.py` 同一套纪律——拆解本身没有功能行为可测
（类原文一字未改），会退化的是**结构**：

1. **不断链**：``main.GLWidget`` 仍须指向同一个类（入口 re-export 是有意保留的过渡约定）；
2. **方向**：``app/views/gl_widget.py`` 不得 import ``main``；
3. **随行的常量与降级块**：``GL_*`` / ``MIME_TYPE`` / ``_HAS_WORLD`` / ``_HAS_PLUGINS``
   必须住在新模块里（它们只服务本类，留在 main.py 就会变成两处维护的漂移源）；
4. **门禁同步**：新文件已登记进守护（这条专拦"搬完忘记同步基线"）。
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import pytest

pytest.importorskip("PySide6")
pytest.importorskip("OpenGL")

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

_MOD = _ROOT / "app" / "views" / "gl_widget.py"


def test_class_moved_to_views_but_still_reachable_from_entry():
    """① 搬迁完成 + 入口不断链。"""
    import main

    from app.views import gl_widget

    assert main.GLWidget is gl_widget.GLWidget, "main.GLWidget 断链了——搬迁后入口必须继续 re-export"
    assert gl_widget.GLWidget.__module__ == "app.views.gl_widget"


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
    assert not bad, f"app/views/gl_widget.py 反向依赖了入口脚本：{bad}"


def test_gl_enums_and_flags_live_with_the_view():
    """③ GL 枚举 / MIME / 降级标志随类搬走（只服务本类，不该留在入口脚本）。"""
    from app.views import gl_widget

    for name in (
        "GL_COLOR_BUFFER_BIT",
        "GL_DEPTH_BUFFER_BIT",
        "GL_TRIANGLES",
        "GL_LINES",
        "GL_LINE_STRIP",
        "GL_POINTS",
        "GL_MODELVIEW",
        "GL_PROJECTION",
        "GL_DEPTH_TEST",
        "MIME_TYPE",
    ):
        assert hasattr(gl_widget, name), f"{name} 未随类搬进视图模块"
    assert gl_widget.GL_TRIANGLES == 0x00000004
    assert gl_widget.MIME_TYPE == "application/x-machine-item"
    for flag in ("_HAS_WORLD", "_HAS_PLUGINS"):
        assert hasattr(gl_widget, flag), f"{flag} 应由视图模块自己定义"


def test_cjk_baseline_registered_in_guard():
    """④ 结构门禁同步：新文件必须在守护的存量债务表与 CJK 基线里有条目。"""
    guard = (_ROOT / "tools" / "check_arch_rules.py").read_text(encoding="utf-8")
    assert (
        '"app/views/gl_widget.py"' in guard
    ), "新文件未登记进 LEGACY_DEBT / CJK_BASELINE —— 搬迁必须同步守护脚本"


def test_module_needs_no_names_from_entry_shell():
    """⑤ 搬迁完整性：本模块用到的名字必须能自己解析，不得指望入口脚本提供。

    **真实事故**（Step 2）：`_segments_for_radius` 是 main.py 的**模块级函数**，
    搬类时漏了它——而**全部测试仍然全绿**（那条代码路径当时没被覆盖），
    只会在真实交互里炸成 NameError。这类"漏搬一个名字"编译器不会说话、
    覆盖不到也不会说话，只能靠静态检查钉住。
    """
    import main

    from app.views import gl_widget

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


def test_no_test_patches_globals_on_entry_shell():
    """⑥ 测试不得对**入口脚本**的模块全局打补丁——补丁要打在**定义处**。

    **真实事故**（Step 2，段错误）：`tests/test_app_degraded_paths.py` 曾写
    ``monkeypatch.setattr(main, "gl", _NullGL())`` 再调 ``main.GLWidget._draw_link_recursive``。
    类搬进 ``app/views/gl_widget.py`` 后，它查的是**自己模块**的 ``gl``——补丁成了空操作，
    于是真去调 OpenGL，在没有上下文的测试环境里直接 **segmentation fault**（不是断言失败，
    是整轮测试崩掉，连失败清单都拿不到）。

    规则：``main.py`` 只是启动壳 + 导入桥，对它的全局打补丁**只对还住在它里面的代码有效**，
    这种"看起来在打补丁、其实没打上"的失效方式极难从失败信息里看出来，所以直接禁掉。
    """
    import re

    tests_dir = _ROOT / "tests"
    pattern = re.compile(r"setattr\(\s*(?:main|rw)\s*,")
    # 跳过本文件：它的文档字符串里正举着这个反例
    offenders = sorted(
        f"{p.name}:{i}"
        for p in tests_dir.glob("*.py")
        if p.name != Path(__file__).name
        for i, line in enumerate(p.read_text(encoding="utf-8").splitlines(), 1)
        if pattern.search(line)
    )
    assert not offenders, (
        "这些地方对入口脚本的模块全局打补丁——改为打在**定义该名字的模块**上：" f"{offenders}"
    )
