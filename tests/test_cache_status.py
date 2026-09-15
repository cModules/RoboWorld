"""缓存可观测性（v1.2.4）的行为契约。

被测对象：

* :func:`orolp.incremental.cache_overview` —— 三类结果缓存（规划与分段 /
  干涉检查 / 力矩校验）的统一快照；
* :func:`orolp.incremental.reset_all_cache_stats` / :func:`orolp.incremental.clear_all_caches`
  —— 全局计数归零 / 条目清空；
* GUI「缓存状态…」对话框 —— :meth:`app.MainWindow._cache_status_text` 的
  文本与 View 菜单接线（Qt offscreen 无头跑）。

核心约定：**全局关闭的缓存也在列**（``enabled=False`` 如实呈现"关着"，
不从面板上消失）；命中数字与既有 v1.2.0/1.2.2/1.2.3 各缓存自己的
``stats()`` 同源，不另立口径。
"""

import importlib.util
import os
import sys
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

import orolp  # noqa: E402
import orolp.i18n as i18n  # noqa: E402
from orolp import Robot  # noqa: E402
from orolp.incremental import (  # noqa: E402
    cache_overview,
    clear_all_caches,
    get_cache,
    get_collision_cache,
    get_dynamics_cache,
    reset_all_cache_stats,
    set_cache,
    set_collision_cache,
    set_dynamics_cache,
)
from orolp.planner import TrajectoryPlanner  # noqa: E402
from orolp.world.scene import Scene  # noqa: E402

from PySide6 import QtWidgets  # noqa: E402

# 三连杆最小夹具（与 test_dynamics_cache.py 同款：URDF 本体不带 <inertial>，
# 动力学靠 set_link_inertial 手动补；干涉检查靠 visual 盒）。
THREE_LINK_URDF = """<?xml version="1.0"?>
<robot name="three_link">
  <link name="base">
    <visual><origin xyz="0.25 0 0.05"/><geometry><box size="0.5 0.1 0.1"/></geometry></visual>
  </link>
  <link name="arm">
    <visual><origin xyz="0.3 0 0.05"/><geometry><box size="0.6 0.1 0.1"/></geometry></visual>
  </link>
  <link name="tool">
    <visual><origin xyz="-0.25 0 0.05"/><geometry><box size="0.5 0.1 0.1"/></geometry></visual>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="arm"/>
    <origin xyz="0 0 0"/><axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14" effort="50"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="arm"/><child link="tool"/>
    <origin xyz="0.6 0 0"/><axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14" effort="50"/>
  </joint>
</robot>
"""


@pytest.fixture(scope="module")
def three_link_urdf(tmp_path_factory):
    path = tmp_path_factory.mktemp("urdf") / "three_link.urdf"
    path.write_text(THREE_LINK_URDF)
    return str(path)


@pytest.fixture
def isolated_caches():
    """三类全局缓存全部换上独立实例，测完还原（不污染其他测试）。"""
    saved = (get_cache(), get_collision_cache(), get_dynamics_cache())
    set_cache(type(saved[0])())
    set_collision_cache(type(saved[1])())
    set_dynamics_cache(type(saved[2])())
    yield
    set_cache(saved[0])
    set_collision_cache(saved[1])
    set_dynamics_cache(saved[2])


def kinds(overview):
    return [row["kind"] for row in overview]


def row_of(overview, kind):
    return next(row for row in overview if row["kind"] == kind)


# --------------------------------------------------------------------------- #
# 1. 统一快照
# --------------------------------------------------------------------------- #


def test_overview_lists_three_kinds(isolated_caches):
    assert kinds(cache_overview()) == ["plan", "collision", "dynamics"]


def test_overview_defaults_enabled_with_caps(isolated_caches):
    overview = cache_overview()
    plan = row_of(overview, "plan")
    assert plan["enabled"] is True
    assert plan["max_entries"] == 4  # PLAN_CACHE_MAX_ENTRIES
    assert plan["plan_max_waypoints"] == 20_000
    assert plan["segment_max_waypoints"] == 40_000
    for kind in ("collision", "dynamics"):
        row = row_of(overview, kind)
        assert row["enabled"] is True
        assert row["max_entries"] == 64
        assert row["entries"] == 0


def test_overview_reports_disabled_as_present_not_absent(isolated_caches):
    """全局关闭的缓存仍占一行、enabled=False——"关着"必须看得见。"""
    saved = (get_cache(), get_collision_cache(), get_dynamics_cache())
    try:
        set_cache(None)
        set_collision_cache(None)
        set_dynamics_cache(None)
        overview = cache_overview()
        assert kinds(overview) == ["plan", "collision", "dynamics"]
        assert all(row["enabled"] is False for row in overview)
    finally:
        set_cache(saved[0])
        set_collision_cache(saved[1])
        set_dynamics_cache(saved[2])


def test_overview_reflects_validate_hits(three_link_urdf, isolated_caches):
    """跑一次「校验两次」后，三类缓存的命中数字同源增长。"""
    scene = Scene()
    scene.set_robot(three_link_urdf)
    robot = Robot(three_link_urdf, "three_link")
    planner = TrajectoryPlanner(scene=scene, robot=robot)
    planner.dynamics_model.set_link_inertial(
        "arm", mass=2.0, com=(0.0, 0.0, 0.1),
        inertia=[[0.001, 0, 0], [0, 0.001, 0], [0, 0, 0.001]],
    )
    planner.dynamics_model.set_link_inertial(
        "tool", mass=0.5, com=(0.0, 0.0, 0.05),
        inertia=[[0.0002, 0, 0], [0, 0.0002, 0], [0, 0, 0.0002]],
    )
    robot.set_joints([0.0, 175.0])
    program = robot.moveJ([90.0, 175.0]).then(robot.moveJ([0.0, 175.0]))

    planner.validate(program)
    planner.validate(program)

    plan = row_of(cache_overview(), "plan")
    assert plan["plan_hits"] >= 1 and plan["plan_misses"] >= 1
    assert 0.0 < plan["plan_hit_rate"] < 1.0

    dynamics = row_of(cache_overview(), "dynamics")
    assert dynamics["hits"] == 1
    assert dynamics["hit_rate"] == pytest.approx(0.5)
    assert dynamics["entries"] == 1


def test_hit_rate_zero_when_no_lookups(isolated_caches):
    assert row_of(cache_overview(), "collision")["hit_rate"] == 0.0


# --------------------------------------------------------------------------- #
# 2. 全局重置 / 清空
# --------------------------------------------------------------------------- #


def test_reset_all_stats_keeps_entries(three_link_urdf, isolated_caches):
    scene = Scene()
    scene.set_robot(three_link_urdf)
    robot = Robot(three_link_urdf, "three_link")
    planner = TrajectoryPlanner(scene=scene, robot=robot)
    robot.set_joints([0.0, 175.0])
    program = robot.moveJ([90.0, 175.0])
    planner.validate(program)

    assert row_of(cache_overview(), "plan")["plan_stores"] >= 1
    reset_all_cache_stats()
    plan = row_of(cache_overview(), "plan")
    assert plan["plan_stores"] == 0 and plan["plan_entries"] == 1  # 账清了、条目还在


def test_clear_all_caches_keeps_counters(three_link_urdf, isolated_caches):
    scene = Scene()
    scene.set_robot(three_link_urdf)
    robot = Robot(three_link_urdf, "three_link")
    planner = TrajectoryPlanner(scene=scene, robot=robot)
    robot.set_joints([0.0, 175.0])
    program = robot.moveJ([90.0, 175.0])
    planner.validate(program)

    clear_all_caches()
    assert row_of(cache_overview(), "plan")["plan_entries"] == 0
    assert row_of(cache_overview(), "plan")["plan_stores"] >= 1  # 条目没了、账还在


def test_reset_and_clear_tolerate_disabled_caches(isolated_caches):
    saved = (get_cache(), get_collision_cache(), get_dynamics_cache())
    try:
        set_cache(None)
        set_collision_cache(None)
        set_dynamics_cache(None)
        reset_all_cache_stats()  # 不抛错即为过
        clear_all_caches()
    finally:
        set_cache(saved[0])
        set_collision_cache(saved[1])
        set_dynamics_cache(saved[2])


def test_top_level_exports():
    for name in ("cache_overview", "reset_all_cache_stats", "clear_all_caches"):
        assert hasattr(orolp, name), f"orolp.{name} 未导出"


# --------------------------------------------------------------------------- #
# 3. GUI：缓存状态对话框与菜单接线（offscreen）
# --------------------------------------------------------------------------- #


@pytest.fixture(scope="module")
def qapp():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _load_app_module():
    spec = importlib.util.spec_from_file_location("rw_app_cache", REPO_ROOT / "app.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_main_window_has_cache_actions(qapp, isolated_caches):
    mod = _load_app_module()
    win = mod.MainWindow()
    try:
        texts = [a.text() for a in win.menuBar().actions()
                 for a in a.menu().actions()]
        assert any("缓存状态" in t for t in texts)
        assert any("清空缓存与统计" in t for t in texts)
        # 对话框文本可独立构造（不弹窗即可断言内容）
        text = win._cache_status_text()
        assert "TrajectoryCache" in text and "CollisionResultCache" in text
        assert "DynamicsResultCache" in text
    finally:
        win.deleteLater()


def test_cache_status_dialog_shows_activity(
    qapp, three_link_urdf, isolated_caches, monkeypatch
):
    """点菜单弹出的对话框内容必须反映真实缓存活动（此处为 0 活动 + 关闭态）。"""
    mod = _load_app_module()
    win = mod.MainWindow()
    try:
        shown = {}
        monkeypatch.setattr(
            QtWidgets.QMessageBox, "exec", lambda self: shown.setdefault("called", True)
        )
        set_dynamics_cache(None)  # 顺手验证"已关闭"行真的会出现
        win.show_cache_status()
        assert shown.get("called") is True
        text = win._cache_status_text()
        assert "已关闭" in text
    finally:
        win.deleteLater()


def test_cache_status_text_en(qapp, isolated_caches, monkeypatch, tmp_path):
    """en 模式下对话框文案为英文、无中文残留。"""
    monkeypatch.setenv("OROLP_CONFIG_DIR", str(tmp_path / "cfg"))
    i18n.set_language(i18n.LANG_EN, save=False)
    try:
        mod = _load_app_module()
        win = mod.MainWindow()
        try:
            texts = [a.text() for a in win.menuBar().actions()
                     for a in a.menu().actions()]
            assert any("Cache status" in t for t in texts)
            text = win._cache_status_text()
            assert "Plan & segment cache" in text
            assert "Torque-check result cache" in text
            assert not any("\u4e00" <= ch <= "\u9fff" for ch in text), "en 模式残留中文"
        finally:
            win.deleteLater()
    finally:
        i18n.set_language(i18n.LANG_ZH, save=False)


def test_clear_caches_action_feedback(qapp, isolated_caches, monkeypatch):
    mod = _load_app_module()
    win = mod.MainWindow()
    try:
        win.clear_caches()
        assert win.statusBar().currentMessage(), "清空后状态栏应给出反馈"
    finally:
        win.deleteLater()
