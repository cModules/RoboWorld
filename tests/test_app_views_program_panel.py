"""`ProgramPanel` 类内拆解（BUILD_QUEUE B1）的守门测试。

与 `test_app_views_gl_widget.py` 同一套纪律——拆解本身没有功能行为可测
（方法体一字未改，只从类方法改成"以 panel 为第一参数的模块级函数 + 薄 stub"），
会退化的是**结构**：

1. **不断链**：``ProgramPanel`` 仍可从 ``app.views.program_panel`` 导入且是类；
2. **方向**：八个新模块不得 import ``main``；
3. **随行的常量与数据类**：``PANEL_STYLE`` / ``RobotProgram`` /
   ``PLAYBACK_INTERVAL_MS`` 必须仍能从 ``app.views.program_panel`` 访问
   （re-export 是有意保留的兼容面）；
4. **门禁同步**：含中文字面量的新文件已登记进守护的 CJK_BASELINE；
5. **搬迁完整性**：新模块用到的名字必须能自己解析，不得指望入口脚本提供；
6. **测试不得对入口脚本的模块全局打补丁**——补丁要打在**定义处**。
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import pytest

pytest.importorskip("PySide6")

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

_MOD = _ROOT / "app" / "views" / "program_panel.py"
_NEW_MODS = [
    _ROOT / "app" / "views" / f"program_{part}.py"
    for part in (
        "state",
        "ui",
        "robot",
        "playback",
        "grasp",
        "export",
        "editor",
        "validate",
    )
]


def test_class_still_importable_from_same_module():
    """① 拆解完成 + 导入面不变。"""
    from app.views import program_panel

    assert isinstance(program_panel.ProgramPanel, type)
    assert program_panel.ProgramPanel.__module__ == "app.views.program_panel"
    assert program_panel.__all__ == ["ProgramPanel", "PANEL_STYLE"]


def test_new_modules_do_not_import_entry_shell():
    """② 依赖方向：八个新模块不得反向 import 入口脚本。"""
    for mod in [*_NEW_MODS, _MOD]:
        tree = ast.parse(mod.read_text(encoding="utf-8"))
        imported: set[str] = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported |= {a.name for a in node.names}
            elif isinstance(node, ast.ImportFrom) and not node.level and node.module:
                imported.add(node.module)
        bad = {m for m in imported if m == "main" or m.startswith("main.")}
        assert not bad, f"{mod.name} 反向依赖了入口脚本：{bad}"


def test_constants_and_dataclass_still_reachable_from_panel_module():
    """③ 常量与数据类经 program_panel re-export，旧导入面不断。"""
    import app.views.program_panel as pp_mod

    assert isinstance(pp_mod.PANEL_STYLE, str) and "QWidget" in pp_mod.PANEL_STYLE
    assert pp_mod.PLAYBACK_INTERVAL_MS == 40
    assert isinstance(pp_mod.RobotProgram, type)
    # 数据类实例化路径也不能断（面板 __init__ 的 _scratch 依赖它）
    assert pp_mod.RobotProgram().taught == []


def test_cjk_baseline_registered_in_guard():
    """④ 结构门禁同步：含中文字面量的新文件必须在 CJK_BASELINE 有键。"""
    guard = (_ROOT / "tools" / "check_arch_rules.py").read_text(encoding="utf-8")
    for part in ("editor", "export", "grasp", "playback", "robot", "ui", "validate"):
        assert (
            f'"app/views/program_{part}.py"' in guard
        ), f"program_{part}.py 未登记进 CJK_BASELINE —— 搬迁必须同步守护脚本"


def test_module_needs_no_names_from_entry_shell():
    """⑤ 搬迁完整性：本模块用到的名字必须能自己解析，不得指望入口脚本提供。

    **真实事故**（ADR-002 Step 2）：`_segments_for_radius` 是 main.py 的
    **模块级函数**，搬类时漏了它——而**全部测试仍然全绿**（那条代码路径
    当时没被覆盖），只会在真实交互里炸成 NameError。这类"漏搬一个名字"
    编译器不会说话、覆盖不到也不会说话，只能靠静态检查钉住。
    """
    import main

    for mod in [*_NEW_MODS, _MOD]:
        tree = ast.parse(mod.read_text(encoding="utf-8"))
        defined = {"self", "panel"}
        for node in ast.walk(tree):
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                defined |= {a.asname or a.name.split(".")[0] for a in node.names}
            elif isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
                defined.add(node.id)
            elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                defined.add(node.name)
            elif isinstance(node, ast.ExceptHandler) and node.name:
                defined.add(node.name)
        used = {
            n.id for n in ast.walk(tree) if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)
        }
        missing = sorted((used & set(dir(main))) - defined)
        assert not missing, (
            f"{mod.name} 这些名字本模块自己解析不了、只能靠入口脚本提供"
            f"（搬迁漏了）：{missing}"
        )


def test_no_test_patches_globals_on_entry_shell():
    """⑥ 测试不得对**入口脚本**的模块全局打补丁——补丁要打在**定义处**。

    拆解后名字的定义处分散在八个新模块；对 ``main`` 打补丁只对还住在它
    里面的代码有效，这种"看起来在打补丁、其实没打上"的失效方式极难从
    失败信息里看出来，所以直接禁掉。
    """
    import re

    tests_dir = _ROOT / "tests"
    pattern = re.compile(r"setattr\(\s*(?:main|rw)\s*,")
    # 跳过本文件与同纪律的 gl_widget 守门测试：它们的文档字符串里正举着这个反例
    skip = {Path(__file__).name, "test_app_views_gl_widget.py"}
    offenders = sorted(
        f"{p.name}:{i}"
        for p in tests_dir.glob("*.py")
        if p.name not in skip
        for i, line in enumerate(p.read_text(encoding="utf-8").splitlines(), 1)
        if pattern.search(line)
    )
    assert not offenders, (
        "这些地方对入口脚本的模块全局打补丁——改为打在**定义该名字的模块**上：" f"{offenders}"
    )
