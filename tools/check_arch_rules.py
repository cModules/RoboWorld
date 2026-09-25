#!/usr/bin/env python3
"""RobotWorld 架构守护脚本（对应 docs/CODING_RULES.md v2 / AGENTS.md）

检查项：
  H1  PySide6/OpenGL 只允许出现在 GUI 归宿文件（orolp/app/ + 已登记债务文件）
  H2  C++ 核心（orolp_core / orolp._core）只允许在单一边界白名单内引用
  H3  Domain 层禁止 import services / app / GUI
  H4  Adapter 层（post / robot_library 非面板部分）禁止 import services / GUI
  H5  i18n 棘轮：中文串字面量（排除 docstring/注释）超过基线即为硬违规，只许减不许增
  H6  features 一致性：enabled/_feature_enabled 的功能键必须在 ALL_FEATURES 注册
  L1  行数分档：>600 行须有 TECH_DEBT 标注或在存量债务清单；>1000 行为硬违规

用法：
  python tools/check_arch_rules.py            # 报告模式，硬违规时退出码 1
  python tools/check_arch_rules.py --lines    # 额外打印行数明细
退出码：0=通过；1=存在硬违规（import 方向或未登记的超限文件）
"""
from __future__ import annotations

import argparse
import ast
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PKG = ROOT / "python" / "orolp"

# ---------------------------------------------------------------------------
# 分层归属清单（与 docs/CODING_RULES.md §1 查表保持同步；改表须同时改这里）
# ---------------------------------------------------------------------------
DOMAIN_FILES = {"robot.py", "pose.py", "link.py", "motion.py",
                "collision.py", "dynamics.py", "envelope.py"}
DOMAIN_PKGS = {"world"}  # world/ 整包，panel.py 除外（见 GUI_DEBT）

SERVICE_FILES = {
    "planner.py", "process.py", "pickplace.py", "plc_scl.py",
    "calibration.py", "twinsync.py", "multirobot.py", "progdiff.py",
    "cam.py", "simulation.py", "sequence.py", "program.py", "project.py",
    "cell.py", "incremental.py", "compliance.py", "iomap.py",
}

# PySide6/OpenGL 合法归宿：orolp/app/ 整包 + 存量债务文件（ADR-002 拆解后清零）
GUI_DEBT = {
    "controller_panel.py",
    "program_panel.py",
    "robot_library/panel.py",
    "world/panel.py",
}

# C++ 核心单一边界（AGENTS.md #4；现状边界在 __init__.py，见 CODING_RULES §2 注①）
CORE_BOUNDARY = {"__init__.py", "kinematics.py", "collision.py"}
CORE_TOKENS = {"orolp_core", "_core"}

# 存量超限债务清单（>600 行，Phase 1 按"一次一个类"消化；新增超限文件不在此列，
# 必须头部标注 TECH_DEBT 否则判硬违规）
LEGACY_DEBT = {
    "app.py": "ADR-002：app.py 拆解为 orolp/app/",
    "controller_panel.py": "迁移 GUI 至 orolp/app/views",
    "program_panel.py": "迁移 GUI 至 orolp/app/views",
    "planner.py": "Service 拆分（Phase 1）",
    "perf.py": "Service 拆分（Phase 1）",
    "twinsync.py": "Service 拆分（Phase 1）",
    "i18n.py": "词条数据外置（Phase 1）",
    "calibration.py": "Service 拆分（Phase 1）",
    "cloudhttp.py": "Adapter 归位评审（CODING_RULES §1）",
    "link.py": "Domain 拆分（Phase 2）",
    "plugins.py": "插件 API v2（Phase 3）",
    "compliance.py": "Service 拆分（Phase 1）",
    "iomap.py": "Service 拆分（Phase 1）",
    "program.py": "Service 拆分（Phase 1）",
    "collision.py": "Domain 拆分（Phase 2）",
    "kinematics.py": "双实现合并（Phase 2）",
    "incremental.py": "Service 拆分（Phase 1）",
    "project.py": "Service 拆分（Phase 1）",
    "cam.py": "Service 拆分（Phase 1）",
    "progdiff.py": "Service 拆分（Phase 1）",
    "world/panel.py": "迁移 GUI 至 orolp/app/views",
    "robot_library/panel.py": "迁移 GUI 至 orolp/app/views",
    "world/scene.py": "Domain 拆分（Phase 2）",
    # —— 2026-09-25 首轮扫描新登记（CODING_RULES §3.2 过渡态）——
    "robot_library/urdf_builder.py": "Adapter 拆分（首轮扫描 1217 行）",
    "robot_library/hifi.py": "Adapter 拆分（Phase 1）",
    "robot_library/specs.py": "Adapter 拆分（Phase 1）",
    "dynamics.py": "Domain 拆分（Phase 2）",
    "linkstub.py": "Adapter 拆分（Phase 1）",
    "post/base.py": "Adapter 拆分（Phase 1）",
    "render/primitives.py": "View 支撑拆分（Phase 1）",
    "render/urdf_model.py": "View 支撑拆分（Phase 1）",
    "world/components.py": "Domain 拆分（Phase 2）",
    "world/geometry.py": "Domain 拆分（Phase 2）",
    "world/templates.py": "Domain 拆分（Phase 2）",
}

SOFT_LIMIT = 600
HARD_LIMIT = 1000

# ---------------------------------------------------------------------------
# H5: i18n 棘轮基线（2026-09-25 首扫：58 文件 / 2558 条，i18n.py 词典 925 条豁免）
# 规则：文件内"非 docstring 的中文字符串字面量"数量 ≤ 基线；超过即硬违规；
# 减少后把基线同步下调（只许减不许增，逐步收敛到 i18n 键）。
# ---------------------------------------------------------------------------
I18N_HOME = {"i18n.py"}
CJK_RE = re.compile(r"[\u4e00-\u9fff]")
CJK_BASELINE = {
    "app.py": 352, "program_panel.py": 138, "world/components.py": 115, "world/panel.py": 112,
    "twinsync.py": 92, "perf.py": 78, "controller_panel.py": 76,
    "project.py": 74, "compliance.py": 72, "plugins.py": 70,
    "world/templates.py": 67, "calibration.py": 66, "link.py": 59,
    "cloudhttp.py": 49, "linkstub.py": 43, "planner.py": 39,
    "robot_library/specs.py": 38, "robot_library/panel.py": 29,
    "progdiff.py": 29, "plc_scl.py": 27, "iomap.py": 27, "cam.py": 25,
    "process.py": 24, "sequence.py": 22, "render/urdf_model.py": 22,
    "multirobot.py": 21, "world/parts_library.py": 17, "post/base.py": 16,
    "robot_library/catalog.py": 13, "project_templates.py": 13,
    "program.py": 13, "world/scene.py": 11, "render/mesh.py": 11,
    "kinematics.py": 11, "post/fanuc_tp.py": 10, "pickplace.py": 10,
    "session.py": 8, "world/placement.py": 7, "robot.py": 7,
    "post/ur_script.py": 7, "post/kuka_krc4.py": 7, "render/primitives.py": 6,
    "cell.py": 6, "features.py": 5, "debug.py": 5, "world/geometry.py": 4,
    "sshstub.py": 4, "post/abb_rapid.py": 4, "ocpcompat.py": 4,
    "post/jaka_zu.py": 3, "post/estun_er.py": 3, "post/__init__.py": 3,
    "collision.py": 3, "simulation.py": 2, "robot_library/urdf_builder.py": 2,
    "post/huichuan_inoteach.py": 2, "world/demo.py": 1, "world/animation.py": 1,
}

# H6: 功能门控键调用形态（含面板本地包装 _feature_enabled）
FEATURE_CALL_NAMES = {"enabled", "_feature_enabled", "hidden_in_mvp"}


def iter_py_files():
    yield ROOT / "app.py"
    for p in sorted(PKG.rglob("*.py")):
        if "__pycache__" in p.parts:
            continue
        yield p


def rel_id(path: Path) -> str:
    """统一标识：仓库根 app.py 用 'app.py'，包内用相对 orolp 的 posix 路径。"""
    if path == ROOT / "app.py":
        return "app.py"
    return path.relative_to(PKG).as_posix()


def module_name(rel: str) -> str:
    return rel[:-3].replace("/", ".") if rel.endswith(".py") else rel


def parse_imports(path: Path):
    """返回 (外部模块名集合, orolp 内部模块名集合)。"""
    try:
        tree = ast.parse(path.read_text(encoding="utf-8", errors="replace"))
    except SyntaxError as e:
        return None, None, f"{path}: 语法解析失败 {e}"
    ext, internal = set(), set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                (internal if a.name.startswith("orolp") else ext).add(a.name)
        elif isinstance(node, ast.ImportFrom):
            if node.level:  # 相对导入，映射为 orolp.*
                base = module_name(rel_id(path)).rsplit(".", node.level)[0] \
                    if False else None
                pkg_parts = module_name(rel_id(path)).split(".")
                parts = pkg_parts[:len(pkg_parts) - node.level + (1 if len(pkg_parts) > 1 else 0)]
                if node.module:
                    parts += node.module.split(".")
                internal.add("orolp." + ".".join(parts))
            elif node.module:
                (internal if node.module.startswith("orolp") else ext).add(node.module)
    return ext, internal, None


def has_tech_debt_marker(path: Path) -> bool:
    try:
        head = "".join(path.read_text(encoding="utf-8", errors="replace")
                       .splitlines(keepends=True)[:15])
    except OSError:
        return False
    return "TECH_DEBT" in head


def classify(rel: str) -> str:
    if rel == "app.py" or rel.startswith("app/"):
        return "app"
    if rel in GUI_DEBT or rel.startswith(("render/",)):
        return "gui"
    if rel in DOMAIN_FILES or rel.split("/")[0] in DOMAIN_PKGS:
        return "domain"
    if rel in SERVICE_FILES:
        return "service"
    if rel.startswith(("post/", "robot_library/")) or rel in {
            "plugins.py", "linkstub.py", "sshstub.py", "cloudhttp.py"}:
        return "adapter"
    return "infra/support"


def _docstring_node_ids(tree) -> set:
    """模块/类/函数首语句的字符串常量视为 docstring，不参与 CJK 计数。"""
    ids = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef,
                             ast.FunctionDef, ast.AsyncFunctionDef)):
            if node.body and isinstance(node.body[0], ast.Expr) \
                    and isinstance(node.body[0].value, ast.Constant):
                ids.add(id(node.body[0].value))
    return ids


def scan_strings(path: Path):
    """单次 ast 遍历，返回 (中文字面量条数, 功能键未注册列表, 错误或 None)。"""
    try:
        tree = ast.parse(path.read_text(encoding="utf-8", errors="replace"))
    except SyntaxError as e:
        return 0, [], f"{path}: 语法解析失败 {e}"
    doc_ids = _docstring_node_ids(tree)
    cjk = 0
    bad_keys: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str) \
                and CJK_RE.search(node.value) and id(node) not in doc_ids:
            cjk += 1
        if isinstance(node, ast.Call):
            fn = node.func
            name = fn.id if isinstance(fn, ast.Name) else (
                fn.attr if isinstance(fn, ast.Attribute) else "")
            if name in FEATURE_CALL_NAMES:
                for arg in node.args:
                    if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
                        bad_keys.append(arg.value)
    return cjk, bad_keys, None


def extract_all_features() -> set[str] | None:
    """从 features.py 静态提取 ALL_FEATURES（DEFAULT_HIDDEN_IN_MVP ∪ {multirobot}）。"""
    try:
        tree = ast.parse((PKG / "features.py").read_text(encoding="utf-8"))
    except (OSError, SyntaxError):
        return None
    hidden: set[str] = set()
    for node in ast.walk(tree):
        targets = []
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, ast.AnnAssign):
            targets = [node.target]
        for t in targets:
            if isinstance(t, ast.Name) and t.id == "DEFAULT_HIDDEN_IN_MVP":
                value = node.value
                # 兼容两种写法：裸集合字面量，或 frozenset({...}) 调用包裹
                if isinstance(value, ast.Call) and isinstance(value.func, ast.Name) \
                        and value.func.id in {"frozenset", "set", "list", "tuple"} \
                        and value.args:
                    value = value.args[0]
                if isinstance(value, (ast.Set, ast.List, ast.Tuple)):
                    hidden = {e.value for e in value.elts
                              if isinstance(e, ast.Constant) and isinstance(e.value, str)}
    return hidden | {"multirobot"}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--lines", action="store_true", help="打印行数明细")
    args = ap.parse_args()

    errors: list[str] = []
    warns: list[str] = []
    all_features = extract_all_features()
    if all_features is None:
        warns.append("H6 [features.py] 无法解析 ALL_FEATURES，跳过功能键校验")

    for path in iter_py_files():
        rel = rel_id(path)
        layer = classify(rel)
        ext, internal, err = parse_imports(path)
        if err:
            errors.append(err)
            continue
        rel_str = " ".join(sorted(ext)) + " " + " ".join(sorted(internal))

        # H1: GUI 依赖归宿
        if ("PySide6" in rel_str or "OpenGL" in rel_str) and layer != "gui" and rel != "app.py":
            errors.append(f"H1 [{rel}] ({layer}) 引用了 PySide6/OpenGL，"
                          f"GUI 依赖只允许 orolp/app/ 或已登记债务文件")
        if rel in GUI_DEBT and not has_tech_debt_marker(path):
            warns.append(f"H1 [{rel}] 存量 GUI 债务未在头部标注 TECH_DEBT（迁移前冻结新功能）")

        # H2: C++ 核心边界
        if any(t in rel_str for t in ("orolp_core", "orolp._core")) and rel not in CORE_BOUNDARY:
            errors.append(f"H2 [{rel}] 直接引用 C++ 核心（orolp_core/_core），"
                          f"只允许经 {sorted(CORE_BOUNDARY)} 的降级边界")

        # H3: Domain 纯净（orolp.plugins 是注册表机制/DI 容器，允许引用；
        # 详见 CODING_RULES §6：扩展点走注册表）
        if layer == "domain":
            for m in internal:
                tail = m.split(".")[1] if m.count(".") >= 1 else m
                if m.endswith("plugins"):
                    continue
                if tail in SERVICE_FILES or tail.startswith(("post", "app", "render")) \
                        or "PySide6" in m or "OpenGL" in m:
                    errors.append(f"H3 [{rel}] Domain 依赖了 services/app/GUI（{m}）")

        # H4: Adapter 禁业务禁 GUI
        if layer == "adapter":
            for m in internal:
                tail = m.split(".")[1] if m.count(".") >= 1 else m
                if tail in SERVICE_FILES:
                    errors.append(f"H4 [{rel}] Adapter 依赖了 services（{m}）")
            if ("PySide6" in rel_str or "OpenGL" in rel_str) \
                    and rel not in GUI_DEBT:
                errors.append(f"H4 [{rel}] Adapter 引用 GUI 库")

        # L1: 行数分档
        n = sum(1 for _ in path.open(encoding="utf-8", errors="replace"))
        if n > HARD_LIMIT and rel not in LEGACY_DEBT:
            errors.append(f"L1 [{rel}] {n} 行 > 硬限 {HARD_LIMIT} 且未登记债务")
        elif n > SOFT_LIMIT and rel not in LEGACY_DEBT:
            warns.append(f"L1 [{rel}] {n} 行 > 软限 {SOFT_LIMIT}，需标注 TECH_DEBT 或登记债务")
        if args.lines:
            print(f"  {n:>5}  {layer:<8} {rel}")

        # H5 / H6: 字符串字面量与功能键
        cjk, keys, serr = scan_strings(path)
        if serr:
            errors.append(serr)
        else:
            if rel not in I18N_HOME and cjk:
                base = CJK_BASELINE.get(rel, 0)
                if cjk > base:
                    errors.append(f"H5 [{rel}] 中文字面量 {cjk} 条 > 基线 {base}，"
                                  f"新增用户可见串必须走 i18n（AGENTS.md #15）")
                elif cjk < base:
                    warns.append(f"H5 [{rel}] 中文字面量 {cjk} 条 < 基线 {base}，"
                                 f"基线可下调（棘轮收敛）")
            if all_features is not None:
                for k in keys:
                    if k not in all_features:
                        errors.append(f"H6 [{rel}] 功能键 \"{k}\" 未在 ALL_FEATURES 注册")

    print("=" * 60)
    for w in warns:
        print(f"WARN  {w}")
    for e in errors:
        print(f"ERROR {e}")
    print(f"—— 检查完成：{len(errors)} 硬违规 / {len(warns)} 警告")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
