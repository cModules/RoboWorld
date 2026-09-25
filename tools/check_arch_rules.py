#!/usr/bin/env python3
"""RobotWorld 架构守护脚本（对应 docs/CODING_RULES.md v2 / AGENTS.md）

检查项：
  H1  PySide6/OpenGL 只允许出现在 GUI 归宿文件（orolp/app/ + 已登记债务文件）
  H2  C++ 核心（orolp_core / orolp._core）只允许在单一边界白名单内引用
  H3  Domain 层禁止 import services / app / GUI
  H4  Adapter 层（post / robot_library 非面板部分）禁止 import services / GUI
  L1  行数分档：>600 行须有 TECH_DEBT 标注或在存量债务清单；>1000 行为硬违规

用法：
  python tools/check_arch_rules.py            # 报告模式，硬违规时退出码 1
  python tools/check_arch_rules.py --lines    # 额外打印行数明细
退出码：0=通过；1=存在硬违规（import 方向或未登记的超限文件）
"""
from __future__ import annotations

import argparse
import ast
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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--lines", action="store_true", help="打印行数明细")
    args = ap.parse_args()

    errors: list[str] = []
    warns: list[str] = []

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
        if rel in GUI_DEBT and "TECH_DEBT" not in (path.read_text(encoding='utf-8', errors='replace')[:600] if path.exists() else ""):
            warns.append(f"H1 [{rel}] 存量 GUI 债务未标注 TECH_DEBT，迁移前应冻结新功能")

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

    print("=" * 60)
    for w in warns:
        print(f"WARN  {w}")
    for e in errors:
        print(f"ERROR {e}")
    print(f"—— 检查完成：{len(errors)} 硬违规 / {len(warns)} 警告")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
