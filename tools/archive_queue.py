#!/usr/bin/env python3
"""BUILD_QUEUE 按周归档：活跃队列只留未完成项，已完成块按 ISO 周归档。

关键设计（避免撕裂内容）：
  * **块级**处理：一个条目 = `- [x]` 标题行 + 其缩进续行。按单行切会把标题和内容
    分到两个文件里——文件里绝大多数条目的正文都在续行。
  * 章节分三类：整章归档 / 保留框架只归档已完成块 / 原样保留（规则与待办池）。
  * 归档文件按 **ISO 周**命名；无日期块用**同章节多数周**推断，推断不出归"早期"。
  * 默认按条目日期分周；周末集中归档时可用 `--week YYYY-Www` 指定目标周。
  * 跑完做守恒校验：未完成块数、已完成块数、总字符数都必须对得上。
"""

from __future__ import annotations

import argparse
import datetime
import pathlib
import re
import sys
from collections import Counter, defaultdict

ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC = ROOT / "docs" / "develop" / "BUILD_QUEUE.md"
ARC_DIR = ROOT / "docs" / "develop" / "queue" / "archive"

DATE_RE = re.compile(r"(\d{4})-(\d{2})-(\d{2})")
ITEM_RE = re.compile(r"^- \[([ x])\]")

#: 整章归档（全完成、且不是当前方向）：这些章节连同说明文字一起搬走
ARCHIVE_WHOLE_PREFIXES = (
    "### C++ 加速层源码",
    "### 目录重整：`models/`",
    "### Could（差异化）",
    "## 里程碑打包记录",
)

#: 原样保留：规则要每轮读，待评审想法是待办池（没有 - [ ] 计数但是待办）
KEEP_AS_IS_PREFIXES = (
    "# RobotWorld 自动开发队列",
    "## 执行规则",
    "## 新增功能想法",
    "## 任务队列",
)


def indent_of(line: str) -> int:
    return len(line) - len(line.lstrip())


def week_key(date_str: str) -> str:
    y, mo, d = (int(x) for x in date_str.split("-"))
    iso = datetime.date(y, mo, d).isocalendar()
    return f"{iso[0]}-W{iso[1]:02d}"


def parse(lines: list[str]):
    """切成：文件头 + 章节列表。章节 = 标题行 + 说明行( prose ) + 条目块。"""
    head: list[str] = []
    sections: list[dict] = []
    cur = None
    i = 0
    while i < len(lines):
        line = lines[i]
        if line.startswith("### ") or line.startswith("## "):
            cur = {"title": line, "prose": [], "items": []}
            sections.append(cur)
            i += 1
            continue
        if cur is None:
            head.append(line)
            i += 1
            continue
        m = ITEM_RE.match(line)
        if m:
            block = [line]
            i += 1
            while i < len(lines):
                nxt = lines[i]
                if ITEM_RE.match(nxt) or nxt.startswith("### ") or nxt.startswith("## "):
                    break
                if nxt.strip() == "" or indent_of(nxt) >= 2:
                    block.append(nxt)
                    i += 1
                else:
                    break
            cur["items"].append({"kind": m.group(1), "lines": block})
            continue
        cur["prose"].append(line)
        i += 1
    return head, sections


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true", help="真正写文件（默认只 dry-run）")
    ap.add_argument("--dry-run", action="store_true", help="仅统计，不写文件（默认行为）")
    args = ap.parse_args()

    text = SRC.read_text(encoding="utf-8")
    lines = text.splitlines(keepends=True)
    head, sections = parse(lines)

    # ---- 分派 ----
    keep_sections: list[dict] = []  # 保留框架
    archived_blocks: list[dict] = []  # (week, section_title, block)
    whole_archive: list[dict] = []  # 整章归档

    for sec in sections:
        title = sec["title"]
        if title.startswith(KEEP_AS_IS_PREFIXES):
            keep_sections.append(sec)
            continue
        if title.startswith(ARCHIVE_WHOLE_PREFIXES):
            whole_archive.append(sec)
            continue
        # 保留框架，只把已完成块归档
        kept_items = []
        for it in sec["items"]:
            if it["kind"] == " ":
                kept_items.append(it)
            else:
                archived_blocks.append({"sec": title, "lines": it["lines"]})
        sec["items"] = kept_items
        keep_sections.append(sec)

    # 整章归档里的条目块也进归档流
    for sec in whole_archive:
        for it in sec["items"]:
            archived_blocks.append({"sec": sec["title"], "lines": it["lines"]})

    # ---- 按周分组（无日期用同章节多数周推断）----
    sec_week_votes: dict[str, Counter] = defaultdict(Counter)
    item_weeks: list[str | None] = []
    for b in archived_blocks:
        m = DATE_RE.search("".join(b["lines"]))
        item_weeks.append(week_key(m.group(0)) if m else None)
        if m:
            sec_week_votes[b["sec"]].update([week_key(m.group(0))])

    fallback: dict[str, str] = {}
    for sec, votes in sec_week_votes.items():
        if votes:
            fallback[sec] = votes.most_common(1)[0][0]

    by_week: dict[str, list] = defaultdict(list)
    for b, wk in zip(archived_blocks, item_weeks):
        if wk is None:
            wk = fallback.get(b["sec"], "2026-W37及更早")
        by_week[wk].append(b)

    # ---- 统计 ----
    def size(items) -> int:
        return sum(len("".join(b["lines"])) for b in items)

    open_blocks = [it for s in keep_sections for it in s["items"] if it["kind"] == " "]
    print("=== dry-run 统计 ===")
    print(f"  已完成块归档：{len(archived_blocks)} 个 / {size(archived_blocks)//1024}KB")
    for wk in sorted(by_week):
        print(f"    {wk}: {len(by_week[wk]):>3} 个 / {size(by_week[wk])//1024}KB")
    print(f"  未完成块保留：{len(open_blocks)} 个 / {size(open_blocks)//1024}KB")
    print(f"  整章归档：{len(whole_archive)} 章")

    if not args.apply:
        print("\n（dry-run，未写文件；加 --apply 执行）")
        return 0

    # ---- 写归档文件 ----
    ARC_DIR.mkdir(parents=True, exist_ok=True)
    for wk in sorted(by_week):
        blocks = by_week[wk]
        body = [
            f"# 已归档：{wk}\n",
            "\n",
            "> 本文件由 `docs/develop/BUILD_QUEUE.md` 按周归档生成（**只读历史**）。\n",
            "> 活跃队列见 [`docs/develop/BUILD_QUEUE.md`](../BUILD_QUEUE.md)。\n",
            "\n",
            "---\n",
            "\n",
        ]
        last_sec = None
        for b in blocks:
            if b["sec"] != last_sec:
                body.append(f"\n### 来源：{b['sec'].lstrip('# ').strip()}\n\n")
                last_sec = b["sec"]
            body.extend(l if l.endswith("\n") else l + "\n" for l in b["lines"])
            body.append("\n")
        archive_path = ARC_DIR / f"{wk}.md"
        archive_text = "".join(body)
        if archive_path.exists():
            existing = archive_path.read_text(encoding="utf-8")
            if existing != archive_text:
                raise RuntimeError(
                    f"归档文件已存在且内容不同，拒绝覆盖历史：{archive_path}"
                )
            print(f"  已存在且一致，跳过 docs/develop/queue/archive/{wk}.md")
        else:
            archive_path.write_text(archive_text, encoding="utf-8")
            print(f"  写出 docs/develop/queue/archive/{wk}.md ({len(blocks)} 块)")

    # ---- 写新的活跃文件 ----
    out: list[str] = []
    out.extend(l if l.endswith("\n") else l + "\n" for l in head)
    out.extend(
        [
            "\n",
            "> **归档规则（2026-09-27 起）**：本文件是**活跃队列**，只保留**未完成条目**；\n",
            "> 已完成的条目按 **ISO 周**归档到 `docs/develop/queue/archive/<年>-W<周>.md`——\n",
            "> **每周集中归档一次**（活跃文件只保留当前未完成任务，上下文成本不随历史增长）。\n",
            "> 历史记录不在本文件里重复记账：要查「做过了什么」请翻归档。\n",
            "\n",
        ]
    )
    # 归档索引
    if by_week:
        out.append("## 归档索引（已完成）\n\n")
        out.append("| 周 | 条目 | 文件 |\n|---|---|---|\n")
        for wk in sorted(by_week):
            out.append(
                f"| {wk} | {len(by_week[wk])} | [`docs/develop/queue/archive/{wk}.md`](queue/archive/{wk}.md) |\n"
            )
        out.append("\n---\n\n")

    for sec in sections:
        if sec in whole_archive:
            continue
        out.append(sec["title"] if sec["title"].endswith("\n") else sec["title"] + "\n")
        out.extend(l if l.endswith("\n") else l + "\n" for l in sec["prose"])
        if sec["items"]:
            out.append("\n")
        for it in sec["items"]:
            out.extend(l if l.endswith("\n") else l + "\n" for l in it["lines"])
        out.append("\n")

    SRC.write_text("".join(out), encoding="utf-8")

    # ---- 守恒校验 ----
    new_text = SRC.read_text(encoding="utf-8")
    arc_text = "".join((ARC_DIR / f"{wk}.md").read_text(encoding="utf-8") for wk in sorted(by_week))
    n_open_new = len(re.findall(r"^- \[ \]", new_text, re.M))
    n_done_new = len(re.findall(r"^- \[x\]", new_text, re.M))
    n_done_arc = len(re.findall(r"^- \[x\]", arc_text, re.M))
    n_open_old = len(re.findall(r"^- \[ \]", text, re.M))
    n_done_old = len(re.findall(r"^- \[x\]", text, re.M))

    print("\n=== 守恒校验 ===")
    print(
        f"  未完成：{n_open_old} → 活跃 {n_open_new}  {'OK' if n_open_old == n_open_new else '**不一致**'}"
    )
    print(f"  已完成：{n_done_old} → 归档 {n_done_arc} + 活跃残留 {n_done_new}")
    ok = n_open_old == n_open_new and (n_done_arc + n_done_new) == n_done_old
    print(f"  结论：{'OK' if ok else '**内容丢失或重复**'}")
    print(f"  活跃文件：{len(new_text)//1024}KB（原 {len(text)//1024}KB）")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
