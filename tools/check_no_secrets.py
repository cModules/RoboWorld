#!/usr/bin/env python3
"""仓库卫生检查：禁止把私有仓库平台信息与凭据写进代码库。

为什么要这个脚本
----------------
代码库一旦被公开分发（本项目是 Apache-2.0 开源），任何提交进来的东西都
等于永久公开且无法撤回——即使后面再删掉，git 历史里仍然留着。以下几类
内容一旦进库就很麻烦：

1. **私有仓库平台地址**（如 CNB 仓库 URL）——暴露内部协作拓扑与命名
2. **URL 里内联的凭据** ``https://user:token@host/...``  ——一次 clone 就泄漏
3. **私钥 / 访问令牌** ——直接等同于交出仓库写权限

本脚本在 CI 里跑，也可以挂成本地 pre-commit，在提交前就拦下来。

设计取舍
--------
* 只做**正则 + 哈希**匹配，不接第三方 secret-scanning 服务——离线可跑，
  不把仓库内容外发。
* 已知泄漏的令牌**只用 SHA-256 前缀存指纹**，绝不明文写进脚本；否则
  "防泄漏的脚本"自己就成了泄漏源。
* 域名模式用字符串拼接（``_DOMAIN_PARTS``），避免脚本匹配到自己。

用法::

    python3 tools/check_no_secrets.py              # 扫描当前仓库
    python3 tools/check_no_secrets.py --staged     # 只扫已暂存的（pre-commit 用）
    python3 tools/check_no_secrets.py --install-hook   # 装成本地 pre-commit

退出码：0 干净；1 发现问题。
"""

from __future__ import annotations

import argparse
import hashlib
import re
import subprocess
import sys
from pathlib import Path
from typing import Iterator, List, NamedTuple, Pattern

REPO_ROOT = Path(__file__).resolve().parent.parent

# --------------------------------------------------------------------------- #
# 规则定义
# --------------------------------------------------------------------------- #
#: 私有仓库平台的域名片段。运行时拼成完整域名再匹配，避免本文件自匹配。
#: 新增平台时在这里加一行即可。
_DOMAIN_PARTS: List[tuple[str, str]] = [
    ("cnb", ".cool"),  # CNB
]

#: 需要排除的目录（前缀匹配，相对仓库根）
_SKIP_DIRS = (
    ".git",
    "build",
    "cmake-build-debug",
    "cmake-build-release",
    "dist",
    "__pycache__",
    ".pytest_cache",
    ".mypy_cache",
    "docs/_build",
    ".venv",
    "venv",
)

#: 视为二进制、不做文本扫描的扩展名
_BINARY_SUFFIXES = frozenset(
    {
        ".stl",
        ".ply",
        ".step",
        ".stp",
        ".iges",
        ".igs",
        ".png",
        ".jpg",
        ".jpeg",
        ".gif",
        ".ico",
        ".pdf",
        ".so",
        ".pyd",
        ".dll",
        ".dylib",
        ".zip",
        ".gz",
        ".whl",
    }
)

#: 已知已泄漏令牌的 SHA-256 前 16 位（64 bit）。只存指纹，不存明文。
_KNOWN_TOKEN_SHA256_PREFIXES = frozenset(
    {
        "eb6e39e9fdec159b",  # CNB 令牌，已于 2026-08 在对话中暴露，应轮换
    }
)


class Rule(NamedTuple):
    """一条检查规则。"""

    name: str
    pattern: Pattern[str]
    hint: str
    #: 命中片段是否脱敏。域名这类"位置信息"应完整显示便于定位，
    #: 真正的秘密值才需要掐头去尾。
    redact: bool = True


def _build_rules() -> List[Rule]:
    """构造规则表。

    Returns:
        规则列表。域名类规则在此处由 :data:`_DOMAIN_PARTS` 拼装。
    """
    domains = "|".join(re.escape(a + b) for a, b in _DOMAIN_PARTS)
    return [
        Rule(
            name="private-repo-host",
            # 允许出现在 :file:`.git/config` 等本地配置里（那些不进版本库）
            pattern=re.compile(r"https?://[^\s\"'<>]*(?:%s)" % domains, re.IGNORECASE),
            hint="私有仓库平台地址不应写进代码库，请改用占位符或放到本地配置",
            # 域名本身不是秘密，完整显示便于定位到具体文件行
            redact=False,
        ),
        Rule(
            name="url-inline-credentials",
            # 候选值里排除 `<` `>`：文档里的占位符写作 <user>:<token>@<host>，
            # 那是说明性文字不是真凭据，按约定放行，避免误报。
            pattern=re.compile(r"[a-z][a-z0-9+.\-]*://[^\s/:@<>]+:[^\s/:@<>]+@", re.IGNORECASE),
            hint="URL 中内联了用户名与密码/令牌，应改用 credential helper 或 SSH",
        ),
        Rule(
            name="private-key",
            pattern=re.compile(r"-----BEGIN (?:RSA |EC |DSA |OPENSSH |PGP )?PRIVATE KEY-----"),
            hint="私钥不应进库",
        ),
        Rule(
            name="hardcoded-token",
            # token/secret/password/api_key = "..." 且值够长够随机
            pattern=re.compile(r"""(?ix)
                # tok / token / secret / password / api_key / access_key / auth
                \b(?:tok|token|secret|password|passwd|pwd|api[_-]?key|access[_-]?key|auth)
                \b\s*[:=]\s*["']?(?P<value>[A-Za-z0-9_\-]{20,})["']?
                """),
            hint="疑似硬编码凭据；请用环境变量或密钥管理服务",
        ),
        Rule(
            name="cnb-cli-token",
            pattern=re.compile(r"\bcnb[_-]?token\b\s*[:=]\s*\S+", re.IGNORECASE),
            hint="CNB CLI 令牌不应进库",
        ),
    ]


# --------------------------------------------------------------------------- #
# 文件收集
# --------------------------------------------------------------------------- #
def _is_binary(path: Path) -> bool:
    """判断文件是否应跳过（按扩展名或含 NUL 字节）。"""
    if path.suffix.lower() in _BINARY_SUFFIXES:
        return True
    try:
        with path.open("rb") as handle:
            return b"\x00" in handle.read(4096)
    except OSError:
        return True


def _iter_files(root: Path, staged: bool) -> Iterator[Path]:
    """枚举待扫描文件。

    Args:
        root: 仓库根目录。
        staged: 为 ``True`` 时只枚举 git 已暂存（``git diff --cached``）的文件。

    Yields:
        待检查的文件路径。
    """
    if staged:
        out = subprocess.run(
            ["git", "diff", "--cached", "--name-only", "--diff-filter=ACM", "-z"],
            cwd=root,
            capture_output=True,
            text=True,
        )
        for raw in out.stdout.split("\0"):
            if not raw.strip():
                continue
            candidate = root / raw
            if candidate.is_file():
                yield candidate
        return

    for path in root.rglob("*"):
        if not path.is_file():
            continue
        rel = path.relative_to(root)
        if any(part in _SKIP_DIRS for part in rel.parts):
            continue
        yield path


def _sha256_prefix(text: str, length: int = 16) -> str:
    """计算字符串的 SHA-256 十六进制前缀。

    Args:
        text: 待哈希文本。
        length: 取前几个十六进制字符。

    Returns:
        十六进制前缀。
    """
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:length]


# --------------------------------------------------------------------------- #
# 扫描
# --------------------------------------------------------------------------- #
class Finding(NamedTuple):
    """一条命中记录。"""

    path: str
    line: int
    rule: str
    hint: str
    excerpt: str


def _redact(text: str, keep: int = 4) -> str:
    """把命中片段掐头去尾，只留少量上下文，避免脚本输出再次泄漏全量值。

    Args:
        text: 原始片段。
        keep: 首尾各保留几个字符。

    Returns:
        脱敏后的片段。
    """
    text = _safe_excerpt(text)
    if len(text) <= keep * 2 + 3:
        return "*" * len(text)
    return f"{text[:keep]}…{text[-keep:]}"


#: URL 内联凭据 ``scheme://user:pass@host``
_CRED_PATTERN = re.compile(r"([a-z][a-z0-9+.\-]*://)([^/\s:@]+):([^/\s:@]+)@", re.IGNORECASE)


def _safe_excerpt(text: str) -> str:
    """让任意片段都变得可以安全地打印到终端 / CI 日志。

    域名类规则为了便于定位需要完整显示，可这一行里可能同时带着内联
    凭据——两条规则的要求会打架。所以统一在这里做兜底清洗：

    1. URL 里的 ``user:pass@`` 换成 ``user:***@``
    2. 任何命中已知泄漏指纹的令牌整体替换成 ``<redacted>``

    Args:
        text: 任意片段。

    Returns:
        可安全输出的片段。
    """
    text = _CRED_PATTERN.sub(lambda m: f"{m.group(1)}{m.group(2)}:***@", text)
    for token in set(re.findall(r"[A-Za-z0-9]{20,}", text)):
        if _sha256_prefix(token) in _KNOWN_TOKEN_SHA256_PREFIXES:
            text = text.replace(token, "<redacted>")
    return text.strip()


def scan(root: Path, staged: bool) -> List[Finding]:
    """扫描仓库，返回全部命中。

    Args:
        root: 仓库根目录。
        staged: 是否只扫已暂存文件。

    Returns:
        命中列表。
    """
    rules = _build_rules()
    findings: List[Finding] = []

    for path in _iter_files(root, staged):
        # 本脚本自身含规则定义，跳过以免自匹配
        if path.resolve() == Path(__file__).resolve():
            continue
        if _is_binary(path):
            continue
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError as exc:  # pragma: no cover - 权限问题等
            print(f"  跳过 {path}: {exc}", file=sys.stderr)
            continue

        rel = path.relative_to(root).as_posix()

        for lineno, line in enumerate(text.splitlines(), start=1):
            for rule in rules:
                match = rule.pattern.search(line)
                if match is None:
                    continue
                # hardcode 规则要求值确实够随机：至少含一个数字或一个大小写混合
                if rule.name == "hardcoded-token":
                    value = match.group("value")
                    if not (any(c.isdigit() for c in value) and any(c.isalpha() for c in value)):
                        continue
                # 域名规则要完整显示便于定位，但输出前仍要过一遍清洗，
                # 防止这一行恰好也带着内联凭据或已知令牌
                excerpt = _redact(match.group(0)) if rule.redact else _safe_excerpt(match.group(0))
                findings.append(Finding(rel, lineno, rule.name, rule.hint, excerpt))

            # 已知泄漏令牌：对整个文件做滑动窗口哈希比对
            for token in re.findall(r"[A-Za-z0-9]{20,}", line):
                if _sha256_prefix(token) in _KNOWN_TOKEN_SHA256_PREFIXES:
                    findings.append(
                        Finding(
                            rel,
                            lineno,
                            "known-leaked-token",
                            "命中已知泄漏令牌指纹，请立即轮换",
                            _redact(token),
                        )
                    )

    return findings


# --------------------------------------------------------------------------- #
# pre-commit 钩子
# --------------------------------------------------------------------------- #
_HOOK_MARKER = "check_no_secrets"


def install_hook(root: Path) -> int:
    """把本脚本装成本仓库的 ``pre-commit`` 钩子。

    已存在同名钩子时不覆盖，只追加一行调用。

    Args:
        root: 仓库根目录。

    Returns:
        进程退出码。
    """
    hook_path = root / ".git" / "hooks" / "pre-commit"
    if not (root / ".git").is_dir():
        print("不是 git 仓库，跳过安装", file=sys.stderr)
        return 1

    invocation = (
        f"\n# {_HOOK_MARKER}: 防止私有仓库地址与凭据进库\n"
        f"python3 tools/check_no_secrets.py --staged || exit 1\n"
    )

    existing = hook_path.read_text(encoding="utf-8") if hook_path.exists() else ""
    if _HOOK_MARKER in existing:
        print(f"pre-commit 钩子已包含 {_HOOK_MARKER}，无需重复安装")
        return 0

    if existing and not existing.startswith("#!"):
        existing = "#!/bin/sh\n" + existing
    elif not existing:
        existing = "#!/bin/sh\n"

    hook_path.write_text(existing + invocation, encoding="utf-8")
    hook_path.chmod(0o755)
    print(f"已安装 pre-commit 钩子 → {hook_path}")
    return 0


# --------------------------------------------------------------------------- #
# 入口
# --------------------------------------------------------------------------- #
def main() -> int:
    parser = argparse.ArgumentParser(description="仓库卫生检查（私有仓库地址 / 凭据）")
    parser.add_argument("--staged", action="store_true", help="只扫描 git 已暂存的文件")
    parser.add_argument("--install-hook", action="store_true", help="安装为本地 pre-commit 钩子")
    parser.add_argument("--root", default=str(REPO_ROOT), help="仓库根目录")
    args = parser.parse_args()

    root = Path(args.root).resolve()
    if args.install_hook:
        return install_hook(root)

    findings = scan(root, staged=args.staged)
    scope = "已暂存文件" if args.staged else "工作区"

    if not findings:
        print(f"✓ 仓库卫生检查通过（{scope}，规则 {len(_build_rules()) + 1} 条）")
        return 0

    print(f"✗ 在{scope}发现 {len(findings)} 处需要处理的命中：\n")
    for item in findings:
        print(f"  {item.path}:{item.line}  [{item.rule}]")
        print(f"      {item.excerpt}")
        print(f"      → {item.hint}\n")

    print("提示：这些内容一旦提交就永久留在 git 历史里。")
    print("      若已提交，需要 git filter-repo 重写历史并强制推送。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
