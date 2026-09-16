"""工作会话支撑：最近打开列表 + 崩溃自动恢复（v1.3.5）。

v1.3.4 把一次工作会话收进了单一 ``.orolp`` 工程文件，但界面侧还剩两条
每天都会踩的缝：

* **「昨天那个工程在哪来着」**——工程文件本身不解决"再找到它"的问题。
  本模块提供最近打开列表（:class:`RecentProjects`），文件菜单据此生成
  「最近打开」子菜单。
* **「改了一下午还没保存，应用崩了/被杀了」**——保存是用户动作，崩溃
  不等人。本模块提供崩溃自动恢复（:func:`write_autosave` /
  :func:`read_autosave`）：GUI 在工程有未保存改动时定期把快照写进
  自动保存目录，下次启动时询问是否恢复。**快照本身就是合法的
  ``.orolp`` 文件**（``Project.save`` 原子写入），恢复走的就是打开工程
  的同一条路；就算自动保存机制本身坏了，用户也能拿
  ``python -m orolp.project`` 直接看快照内容。

存储位置与其他用户级配置一致：``~/.orolp/``，可用 ``OROLP_CONFIG_DIR``
环境变量重定向（测试/便携部署）。**每次调用都重新解析环境变量**——
测试之间重定向互不污染。

容错口径：这两样都是**锦上添花的机制**，任何故障（配置目录建不了、
记录文件损坏、快照半截）都只能"当它不存在"，绝不能让主流程报错——
最近打开打不开就当没记录过，自动保存读不回来就不弹恢复询问，
但**绝不抛异常打断启动或保存**。
"""

from __future__ import annotations

import json
import os
import tempfile
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

from .i18n import tr

__all__ = [
    "RECENTS_MAX",
    "RecentProjects",
    "AUTOSAVE_SCHEMA",
    "autosave_dir",
    "has_autosave",
    "write_autosave",
    "read_autosave",
    "clear_autosave",
    "autosave_snapshot_name",
]

#: 最近打开列表上限（超出从最旧端挤掉）
RECENTS_MAX = 10

#: 自动保存快照元数据的 schema
AUTOSAVE_SCHEMA = "orolp.autosave/1"

#: 快照工程文件名（目录内固定一个槽位——自动保存只关心"最近一次"）
AUTOSAVE_SNAPSHOT = "session.orolp"

#: 快照元数据文件名（与快照一一对应的 sidecar）
AUTOSAVE_META = "session.json"


def _config_base() -> Path:
    """用户级配置目录（``OROLP_CONFIG_DIR`` > ``~/.orolp``）。"""
    base = os.environ.get("OROLP_CONFIG_DIR")
    return Path(base) if base else Path.home() / ".orolp"


def _now() -> str:
    return datetime.now().isoformat(timespec="seconds")


def _atomic_write_text(target: Path, text: str) -> None:
    """原子写文本：同目录临时文件 + ``os.replace``。失败抛 OSError。"""
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(
        dir=str(target.parent), prefix=target.name + ".", suffix=".tmp"
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(text)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp_name, target)
    except BaseException:
        try:
            os.unlink(tmp_name)
        except OSError:
            pass
        raise


# --------------------------------------------------------------------------- #
# 最近打开
# --------------------------------------------------------------------------- #


@dataclass
class RecentProjects:
    """最近打开的工程文件列表（最新在前，去重，上限 :data:`RECENTS_MAX`）。

    列表条目是**绝对路径字符串**。持久化到 ``<配置目录>/recent_projects.json``；
    文件缺失/损坏按空列表处理——坏记录不该让人打不开软件。
    """

    paths: List[str] = field(default_factory=list)

    @classmethod
    def store_path(cls) -> Path:
        return _config_base() / "recent_projects.json"

    @classmethod
    def load(cls) -> "RecentProjects":
        """读记录；缺失/损坏一律返回空列表（不抛异常）。"""
        try:
            text = cls.store_path().read_text(encoding="utf-8")
            data = json.loads(text)
        except (OSError, ValueError):
            return cls()
        if isinstance(data, dict) and isinstance(data.get("paths"), list):
            items = [str(p) for p in data["paths"] if isinstance(p, str) and p]
        else:
            items = []
        return cls(paths=items[:RECENTS_MAX])

    def save(self) -> bool:
        """写记录。失败返回 ``False``（尽力而为，不向上抛）。"""
        try:
            _atomic_write_text(
                self.store_path(),
                json.dumps({"schema": "orolp.recents/1", "paths": self.paths},
                           ensure_ascii=False, indent=2),
            )
        except OSError:
            return False
        return True

    def add(self, path: str) -> None:
        """记一条：已存在挪到最前，不存在插到最前，超出上限从最旧端挤掉。"""
        raw = str(path or "").strip()
        if not raw:
            return
        try:  # 相对路径按当前位置定死成绝对路径，菜单里的条目才可点
            resolved = str(Path(raw).resolve())
        except OSError:
            resolved = raw
        if resolved in self.paths:
            self.paths.remove(resolved)
        self.paths.insert(0, resolved)
        del self.paths[RECENTS_MAX:]

    def clear(self) -> None:
        self.paths.clear()

    def existing(self) -> List[str]:
        """还真实存在的条目（菜单据此把失效条目置灰而不是悄悄藏掉）。"""
        return [p for p in self.paths if Path(p).exists()]

    def prune_missing(self) -> List[str]:
        """删掉已不存在的条目，返回被删的（GUI 可选提示）。"""
        gone = [p for p in self.paths if not Path(p).exists()]
        self.paths = [p for p in self.paths if Path(p).exists()]
        return gone


# --------------------------------------------------------------------------- #
# 崩溃自动恢复
# --------------------------------------------------------------------------- #


def autosave_dir() -> Path:
    """自动保存目录（``<配置目录>/autosave``）。"""
    return _config_base() / "autosave"


def autosave_snapshot_name() -> str:
    return AUTOSAVE_SNAPSHOT


def write_autosave(project, source_path: Optional[str] = None) -> Optional[Path]:
    """把工程快照写进自动保存目录，返回快照路径；写不了返回 ``None``。

    * ``project`` 是 :class:`orolp.project.Project`（通常是 GUI 另建的
      **快照对象**，不动正在编辑的那份——快照保存会改写 ``path``/``name``）；
    * ``source_path`` 是被保存会话的**真实**工程路径（快照落盘后其
      ``project.path`` 会指向快照自己，真实路径记在 sidecar 里）；
    * 快照用 ``Project.save`` 原子写入——它本身就是合法 ``.orolp``；
    * sidecar 记 ``{source_path, saved_at, app_version}``，恢复询问里
      "上次保存到什么时候、改的是哪个工程"就是它。
    """
    directory = autosave_dir()
    snapshot = directory / AUTOSAVE_SNAPSHOT
    meta_path = directory / AUTOSAVE_META
    try:
        project.save(snapshot)
        _atomic_write_text(
            meta_path,
            json.dumps(
                {
                    "schema": AUTOSAVE_SCHEMA,
                    "source_path": str(source_path) if source_path else "",
                    "saved_at": _now(),
                    "app_version": _library_version(),
                },
                ensure_ascii=False, indent=2,
            ),
        )
    except Exception:
        # 快照/元数据任何一步失败：尽力把残件清掉，别留一个"看起来能恢复"的空壳
        _best_effort_unlink(snapshot)
        _best_effort_unlink(meta_path)
        return None
    return snapshot


@dataclass
class AutosaveInfo:
    """读回的自动保存：快照工程 + 元数据。"""

    snapshot: Path
    #: 被保存会话当时的工程路径（可能为空——未命名工程没存过盘）
    source_path: str
    saved_at: str
    app_version: str


def read_autosave() -> Optional[AutosaveInfo]:
    """读自动保存；快照或元数据不可用（缺失/损坏/schema 不对）返回 ``None``。

    元数据坏但快照还在时也返回 ``None``——没有"保存于何时"的可靠说法，
    就不给用户一个来路不明的恢复选项；快照文件本身仍是合法 ``.orolp``，
    用户可以手动打开它。
    """
    directory = autosave_dir()
    snapshot = directory / AUTOSAVE_SNAPSHOT
    meta_path = directory / AUTOSAVE_META
    if not snapshot.exists() or not meta_path.exists():
        return None
    try:
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    if not isinstance(meta, dict) or str(meta.get("schema") or "") != AUTOSAVE_SCHEMA:
        return None
    return AutosaveInfo(
        snapshot=snapshot,
        source_path=str(meta.get("source_path") or ""),
        saved_at=str(meta.get("saved_at") or ""),
        app_version=str(meta.get("app_version") or ""),
    )


def has_autosave() -> bool:
    return read_autosave() is not None


def clear_autosave() -> None:
    """清掉自动保存（保存/打开成功、干净退出时调用；尽力而为）。"""
    directory = autosave_dir()
    _best_effort_unlink(directory / AUTOSAVE_SNAPSHOT)
    _best_effort_unlink(directory / AUTOSAVE_META)


def _best_effort_unlink(path: Path) -> None:
    try:
        path.unlink()
    except OSError:
        pass


def _library_version() -> str:
    try:
        from . import __version__

        return str(__version__)
    except Exception:  # pragma: no cover
        return "unknown"


def _main(argv: Optional[List[str]] = None) -> int:  # pragma: no cover
    """CLI：不开 GUI 看一眼最近打开与自动保存状态。"""
    import argparse

    parser = argparse.ArgumentParser(
        prog="python -m orolp.session",
        description=tr("查看最近打开记录与崩溃自动恢复状态"),
    )
    parser.parse_args(argv)

    recents = RecentProjects.load()
    print(tr("最近打开（{n} 条）：").format(n=len(recents.paths)))
    for p in recents.paths:
        mark = "" if Path(p).exists() else tr("  （文件已不存在）")
        print(f"  {p}{mark}")
    info = read_autosave()
    if info is None:
        print(tr("自动保存：无"))
    else:
        print(
            tr("自动保存：{saved_at} ｜ 源工程 {src}").format(
                saved_at=info.saved_at or tr("（未知时间）"),
                src=info.source_path or tr("（未命名工程）"),
            )
        )
        print(tr("快照：{path}").format(path=info.snapshot))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(_main())
