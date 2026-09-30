"""会话支撑（v1.3.5）：最近打开列表 + 崩溃自动恢复的库层测试。

GUI 侧（菜单/定时器/恢复询问）在 ``tests/test_project_session_gui.py``；
这里只管机制本身：

* 最近打开：去重、最新在前、上限、持久化往返、坏文件当空、失效条目；
* 自动保存：快照写入/读回/清除，sidecar 元数据，快照本身就是合法
  ``.core``（``Project.load`` 能读、未知段跟着走）；
* **容错口径**：任何一步坏了都"当它不存在"——机制坏了不能让主流程报错。

跑法::

    pytest tests/test_project_session.py -q
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
for _p in (REPO_ROOT, REPO_ROOT):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from service.project import Project
from support import __version__
from app.session import (
    AUTOSAVE_SCHEMA,
    RecentProjects,
    autosave_dir,
    clear_autosave,
    has_autosave,
    read_autosave,
    write_autosave,
)


@pytest.fixture()
def cfg_dir(tmp_path, monkeypatch):
    """把用户级配置目录重定向到临时目录（每次调用都重新解析环境变量）。"""
    base = tmp_path / "core-config"
    monkeypatch.setenv("CORE_CONFIG_DIR", str(base))
    return base


# --------------------------------------------------------------------------- #
# 最近打开
# --------------------------------------------------------------------------- #


def test_recents_empty_when_no_file(cfg_dir):
    assert RecentProjects.load().paths == []
    assert RecentProjects.load().store_path() == cfg_dir / "recent_projects.json"


def test_recents_add_dedupe_and_most_recent_first(cfg_dir, tmp_path):
    a = str(Path(tmp_path / "a.core").resolve())
    b = str(Path(tmp_path / "b.core").resolve())
    recents = RecentProjects.load()
    recents.add(a)
    recents.add(b)
    recents.add(a)  # 已存在 → 挪到最前，不重复
    assert recents.paths == [a, b]


def test_recents_relative_path_resolved_to_absolute(cfg_dir, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    recents = RecentProjects.load()
    recents.add("cell.core")
    assert recents.paths == [str(Path(tmp_path / "cell.core").resolve())]


def test_recents_cap_at_max(cfg_dir, tmp_path):
    recents = RecentProjects.load()
    for i in range(15):
        recents.add(str(Path(tmp_path / f"p{i}.core").resolve()))
    assert len(recents.paths) == 10
    assert recents.paths[0].endswith("p14.core")  # 最新在前
    assert recents.paths[-1].endswith("p5.core")  # 最旧的 5 条被挤掉


def test_recents_roundtrip_and_corrupt_file_is_empty(cfg_dir, tmp_path):
    a = str(Path(tmp_path / "a.core").resolve())
    b = str(Path(tmp_path / "b.core").resolve())
    recents = RecentProjects.load()
    recents.add(a)
    recents.add(b)
    assert recents.save() is True

    again = RecentProjects.load()
    assert again.paths == [b, a]

    RecentProjects.store_path().write_text("{ 不是 JSON", encoding="utf-8")
    assert RecentProjects.load().paths == []  # 坏记录当没记录过，不抛异常


def test_recents_clear_and_existing_and_prune(cfg_dir, tmp_path):
    live = tmp_path / "live.core"
    live.write_text("x", encoding="utf-8")
    gone_path = str(Path(tmp_path / "gone.core").resolve())
    recents = RecentProjects.load()
    recents.add(str(live))
    recents.add(gone_path)
    assert recents.existing() == [str(live)]  # 失效条目置灰用的口径
    gone = recents.prune_missing()
    assert gone == [gone_path]
    assert recents.paths == [str(live)]
    recents.clear()
    assert recents.paths == []


# --------------------------------------------------------------------------- #
# 自动保存
# --------------------------------------------------------------------------- #


def _demo_project(name="demo"):
    project = Project(name=name)
    project.set(
        "program",
        {"code": "# OLP\nmovej([0,-90,90,-90,90,0])\n", "point_types": ["via"], "joint_names": []},
    )
    project.set("parts", {"sources": []})
    return project


def test_autosave_dir_follows_config_redirect(cfg_dir):
    assert autosave_dir() == cfg_dir / "autosave"
    assert has_autosave() is False
    assert read_autosave() is None  # 没有快照时不抛异常


def test_write_read_clear_autosave_roundtrip(cfg_dir):
    source = "/somewhere/my_cell.core"
    snapshot = write_autosave(_demo_project(), source_path=source)
    assert snapshot is not None
    assert snapshot.exists()
    assert has_autosave() is True

    info = read_autosave()
    assert info is not None
    assert info.snapshot == snapshot
    assert info.source_path == source
    assert info.saved_at  # 有可靠的时间说法
    assert info.app_version == __version__

    clear_autosave()
    assert has_autosave() is False
    assert read_autosave() is None


def test_autosave_snapshot_is_a_valid_project_file(cfg_dir):
    project = _demo_project()
    project.raw_sections["vision"] = {"cameras": [{"id": "cam1"}]}  # 未知段跟着走
    snapshot = write_autosave(project, source_path="/somewhere/x.core")
    assert snapshot is not None

    loaded = Project.load(snapshot)
    assert loaded.has("program")
    assert loaded.get("program")["point_types"] == ["via"]
    assert loaded.raw("vision") == {"cameras": [{"id": "cam1"}]}


def test_autosave_write_failure_returns_none_and_leaves_no_shell(cfg_dir):
    """写不了（比如目录位置被文件占住）→ 返回 None，且不留"看起来能恢复"的空壳。"""
    cfg_dir.mkdir(parents=True, exist_ok=True)
    blocker = cfg_dir / "autosave"
    blocker.write_text("不是目录", encoding="utf-8")
    assert write_autosave(_demo_project()) is None
    assert has_autosave() is False
    assert list(blocker.parent.iterdir()) == [blocker]  # 目录位置没有残件


def test_autosave_corrupt_meta_is_tolerated_as_missing(cfg_dir):
    snapshot = write_autosave(_demo_project())
    assert snapshot is not None
    meta = autosave_dir() / "session.json"
    meta.write_text("{ 坏了", encoding="utf-8")
    assert read_autosave() is None  # 没有可靠元数据就不给恢复选项，不抛异常

    meta.write_text(
        json.dumps({"schema": "别的 schema", "saved_at": "2026-01-01"}),
        encoding="utf-8",
    )
    assert read_autosave() is None  # schema 不对同样不认


def test_autosave_clear_is_best_effort_when_absent(cfg_dir):
    clear_autosave()  # 没有快照时清一下也不该抛异常
    assert has_autosave() is False
