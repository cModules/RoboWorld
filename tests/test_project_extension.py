"""工程扩展名：``.rwproj`` 为主，``.robotproj`` / ``.core`` 为兼容别名。

2026-10-12 更名。**格式一个字节都没改**（schema 仍是 ``service.project/1``），
所以"老工程照开"不是一句承诺，而是结构上的事实——这份测试把它钉住，
防止将来有人给 :meth:`Project.load` 加上后缀校验，把兼容性别名悄悄关掉。

为什么单独成文件：``tests/test_project.py`` 已经 608 行（超过测试档 500 的上限），
再往里堆会继续恶化；新增断言一律放这里。
"""

import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from service.project import (  # noqa: E402
    PROJECT_EXTENSION,
    PROJECT_EXTENSIONS,
    PROJECT_FILTER,
    PROJECT_SCHEMA,
    Project,
)


def _project() -> Project:
    """一个够用的最小工程（不依赖场景/示教内容）。"""
    project = Project()
    project.name = "cell"
    return project


def test_primary_extension_is_rwproj():
    """主扩展名：自解释、便于三端注册文件关联。"""
    assert PROJECT_EXTENSION == ".rwproj"


def test_aliases_keep_old_projects_openable():
    assert PROJECT_EXTENSIONS == (".rwproj", ".robotproj", ".core")
    assert PROJECT_EXTENSIONS[0] == PROJECT_EXTENSION


def test_filter_offers_every_extension():
    """文件对话框要能筛到三种后缀，否则用户找不到自己的老工程。"""
    for ext in PROJECT_EXTENSIONS:
        assert ext in PROJECT_FILTER


@pytest.mark.parametrize("suffix", PROJECT_EXTENSIONS)
def test_round_trip_under_every_extension(tmp_path, suffix):
    """同一份内容、三种后缀，都能存能读——别名不是"能打开"而已，要能存。"""
    target = _project().save(tmp_path / f"cell{suffix}")
    assert target.suffix == suffix
    assert target.exists()

    back = Project.load(target)
    assert back.name == "cell"


def test_legacy_core_file_still_loads(tmp_path):
    """历史 ``.core`` 工程必须原样能开（没有迁移步骤、没有迁移工具）。"""
    legacy = tmp_path / "old_cell.core"
    legacy.write_text(
        json.dumps({"schema": PROJECT_SCHEMA, "meta": {"name": "legacy"}}, ensure_ascii=False),
        encoding="utf-8",
    )
    back = Project.load(legacy)
    assert back.name == "legacy"


def test_save_without_suffix_uses_primary_extension(tmp_path):
    """没写后缀 → 补主扩展名（不能补已经被取代的 ``.core``）。"""
    target = _project().save(tmp_path / "cell")
    assert target.name == "cell" + PROJECT_EXTENSION
    assert target.exists()


def test_save_keeps_an_explicit_legacy_suffix(tmp_path):
    """显式存成 ``.core`` 时保持原样：不偷偷改名，用户的文件不该被改写后缀。"""
    target = _project().save(tmp_path / "kept.core")
    assert target.name == "kept.core"
