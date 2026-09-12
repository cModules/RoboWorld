#!/usr/bin/env python3
"""把已完成的开发计划从 .codebuddy/plans 归档到 docs/plan。

背景
----
CodeBuddy 生成的开发计划存放在 ``.codebuddy/plans/``，属于过程性目录。
计划执行完成后应沉淀为项目资产，归档到仓库的 ``docs/plan/`` 并同步索引。

判定依据
--------
**只看 frontmatter 里 todos 的 status，不看文件名**。文件名里的
``(未完成)`` 是人工标记，可能与实际状态不一致，不能作为依据。
所有 todo 的 ``status`` 均处于终态（``completed`` / ``cancelled``）即视为完成。

用法::

    python tools/archive_plans.py --dry-run   # 预览，不改动任何文件
    python tools/archive_plans.py             # 正式归档

特性
----
* 归档为 ``docs/plan/<name>.md``（去掉文件名末尾 8 位 hash 与「(未完成)」标记）
* 自动在 ``docs/spec.md`` 的「开发计划」索引表新增/更新条目
* 幂等：同名目标内容相同则跳过；内容不同则加 hash 后缀保留，绝不覆盖
* frontmatter 解析优先用 PyYAML，缺失时降级为手写解析
"""

from __future__ import annotations

import argparse
import hashlib
import re
import shutil
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

#: 仓库根目录（本文件位于 <root>/tools/）
ROOT = Path(__file__).resolve().parent.parent
PLANS_DIR = ROOT / ".codebuddy" / "plans"
SPEC_PLAN_DIR = ROOT / "docs" / "plan"
SPEC_README = ROOT / "docs" / "spec.md"

#: 终态：处于这些状态的 todo 不会再变动
TERMINAL_STATES = {"completed", "cancelled"}

#: 文件名末尾的 hash 后缀，形如 ``_3273be2c``
_HASH_SUFFIX = re.compile(r"_[0-9a-f]{8}$")
#: 人工标记「(未完成)」
_UNFINISHED_MARK = re.compile(r"[（(]\s*未完成\s*[)）]")

INDEX_HEADER = "| 文档 | 主题 | 状态 |"
PLAN_SECTION = "### 开发计划"


# --------------------------------------------------------------------------- #
# frontmatter 解析
# --------------------------------------------------------------------------- #

def _parse_with_yaml(text: str) -> Optional[dict]:
    try:
        import yaml  # type: ignore
    except ImportError:
        return None
    if not text.startswith("---"):
        return None
    parts = text.split("---", 2)
    if len(parts) < 3:
        return None
    try:
        data = yaml.safe_load(parts[1])
    except Exception:
        return None
    return data if isinstance(data, dict) else None


def _parse_manually(text: str) -> Optional[dict]:
    """PyYAML 不可用时的降级解析。

    plan 文件的结构固定：``todos`` 是列表，每项含 ``- id:`` 与 ``status:``。
    这里只提取判定所需的字段，不追求完整 YAML 语义。
    """
    if not text.startswith("---"):
        return None
    parts = text.split("---", 2)
    if len(parts) < 3:
        return None
    block = parts[1]

    meta: Dict[str, object] = {}
    name = re.search(r"^name:\s*(.+)$", block, re.M)
    if name:
        meta["name"] = name.group(1).strip()
    overview = re.search(r"^overview:\s*(.+)$", block, re.M)
    if overview:
        meta["overview"] = overview.group(1).strip()

    statuses = re.findall(r"^\s*status:\s*(\w+)\s*$", block, re.M)
    meta["todos"] = [{"status": s} for s in statuses]
    return meta


def parse_frontmatter(path: Path) -> Optional[dict]:
    """解析 plan 文件的 frontmatter；失败返回 None。"""
    text = path.read_text(encoding="utf-8")
    return _parse_with_yaml(text) or _parse_manually(text)


# --------------------------------------------------------------------------- #
# 判定与命名
# --------------------------------------------------------------------------- #

def is_completed(meta: dict) -> Tuple[bool, int]:
    """判定是否完成，返回 ``(是否完成, 未完成条目数)``。"""
    todos = meta.get("todos") or []
    if not todos:
        return False, 0
    remaining = sum(
        1 for t in todos
        if str(t.get("status", "")).lower() not in TERMINAL_STATES
    )
    return remaining == 0, remaining


def archive_status(meta: dict) -> str:
    """归档后在索引里显示的状态。"""
    todos = meta.get("todos") or []
    states = [str(t.get("status", "")).lower() for t in todos]
    if states and all(s == "cancelled" for s in states):
        return "已废弃"
    if any(s == "cancelled" for s in states):
        return "已完成（含终止项）"
    return "已完成"


def clean_name(stem: str) -> str:
    """规范归档文件名：去掉 hash 后缀与「(未完成)」标记。"""
    name = _UNFINISHED_MARK.sub("", stem)
    name = _HASH_SUFFIX.sub("", name.strip())
    return name.strip() or stem


def topic_of(meta: dict, fallback: str) -> str:
    """索引里的主题：取 overview 前 24 字，无则回退 name。"""
    overview = str(meta.get("overview") or "").strip()
    if overview:
        return overview[:24] + ("…" if len(overview) > 24 else "")
    return str(meta.get("name") or fallback)


# --------------------------------------------------------------------------- #
# 索引更新
# --------------------------------------------------------------------------- #

def update_index(doc_name: str, topic: str, status: str, dry_run: bool) -> bool:
    """在 docs/spec.md 的「开发计划」表中插入或更新一行。

    Returns:
        成功返回 True；找不到表头返回 False（此时**不改动**文件）。
    """
    lines = SPEC_README.read_text(encoding="utf-8").splitlines()

    try:
        sec = lines.index(PLAN_SECTION)
    except ValueError:
        print("[warn] docs/spec.md 找不到 '### 开发计划' 小节，跳过索引更新")
        return False

    header_at = None
    for i in range(sec, len(lines)):
        if lines[i].strip() == INDEX_HEADER:
            header_at = i
            break
    if header_at is None:
        print("[warn] docs/spec.md 找不到开发计划索引表头，跳过索引更新")
        return False

    rel = f"plan/{doc_name}"
    new_row = f"| [`{rel}`]({rel}) | {topic} | {status} |"

    # 已存在同名条目则替换
    for i in range(header_at + 1, len(lines)):
        if f"plan/{doc_name}" in lines[i]:
            lines[i] = new_row
            break
    else:
        # 追加到该表末尾（跳过后续空行/新小节）
        insert_at = header_at + 2
        while insert_at < len(lines) and lines[insert_at].strip().startswith("|"):
            insert_at += 1
        lines.insert(insert_at, new_row)

    if not dry_run:
        SPEC_README.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return True


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #

def _same_content(a: Path, b: Path) -> bool:
    if not b.exists():
        return False
    ha = hashlib.md5(a.read_bytes()).hexdigest()
    hb = hashlib.md5(b.read_bytes()).hexdigest()
    return ha == hb


def main() -> int:
    ap = argparse.ArgumentParser(description="归档已完成的开发计划到 docs/plan")
    ap.add_argument("--dry-run", action="store_true",
                    help="只打印将要执行的操作，不改动任何文件")
    args = ap.parse_args()

    if not PLANS_DIR.is_dir():
        print(f"源目录不存在：{PLANS_DIR}")
        return 1
    SPEC_PLAN_DIR.mkdir(parents=True, exist_ok=True)

    archived: List[str] = []
    skipped: List[Tuple[str, str]] = []

    for path in sorted(PLANS_DIR.glob("*.md")):
        meta = parse_frontmatter(path)
        if meta is None:
            skipped.append((path.name, "frontmatter 解析失败"))
            continue

        done, remaining = is_completed(meta)
        if not done:
            reason = "无 todos" if not (meta.get("todos") or []) \
                else f"仍有 {remaining} 项未完成"
            skipped.append((path.name, reason))
            continue

        if _UNFINISHED_MARK.search(path.stem):
            print(f"[warn] {path.name} 判定为完成，但文件名带「未完成」标记，请人工核对")

        target_name = clean_name(path.stem) + ".md"
        target = SPEC_PLAN_DIR / target_name
        if target.exists() and not _same_content(path, target):
            # 内容不同：加 hash 后缀保全新内容，绝不覆盖
            digest = hashlib.md5(path.read_bytes()).hexdigest()[:8]
            target = SPEC_PLAN_DIR / f"{Path(target_name).stem}_{digest}.md"
            print(f"[info] 同名但内容不同，改存为 {target.name}")

        if target.exists() and _same_content(path, target):
            skipped.append((path.name, "已归档（内容一致）"))
            continue

        action = "将归档" if args.dry_run else "已归档"
        print(f"{action}: {path.name}  ->  docs/plan/{target.name}")
        if not args.dry_run:
            shutil.move(str(path), str(target))
            update_index(
                target.name,
                topic_of(meta, path.stem),
                archive_status(meta),
                dry_run=False,
            )
        archived.append(target.name)

    print()
    print(f"归档 {len(archived)} 个，跳过 {len(skipped)} 个"
          + ("（--dry-run 预览，未改动文件）" if args.dry_run else ""))
    for name, why in skipped:
        print(f"  跳过 {name}：{why}")

    if args.dry_run:
        print()
        print("确认清单无误后，去掉 --dry-run 再执行一次即可正式归档。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
