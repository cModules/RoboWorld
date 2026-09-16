"""工程模板的库层测试（v1.3.6 首版，v1.3.8 扩容到五个模板）。

GUI 侧（菜单/对话框/脏确认）在 ``tests/test_project_templates_gui.py``；
这里管模板产物本身：

* 五个内置模板的清单与产物结构（只收模板真给的段）；
* 抓取工作站＝场景 + **完整** 8 点抓取循环（含抓取/放置语义），
  与一键演示同一数据源，真实 IK；
* 模板产物就是普通工程：保存 → 载入 → 摘要全套契约照用；
* 每次全新构建，两个模板工程互不共享状态。

跑法::

    pytest tests/test_project_templates.py -q
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
for _p in (REPO_ROOT, REPO_ROOT / "python"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from orolp.project import Project
from orolp.project_templates import (
    build_project_template,
    list_project_templates,
    save_project_template,
)


def test_list_has_five_templates_with_metadata():
    templates = list_project_templates()
    ids = [t["id"] for t in templates]
    assert ids == [
        "grasping_workstation",
        "sorting_station",
        "machine_tending",
        "3c_assembly",
        "auto_welding",
    ]
    for t in templates:
        assert t["name"]
        assert t["description"]


def test_grasping_template_scene_and_full_program():
    project = build_project_template("grasping_workstation")
    # 只收模板真给的段——没有时序/点表就不写那段
    assert project.has("scene")
    assert project.has("program")
    assert not project.has("sequence")
    assert not project.has("cell")
    assert not project.has("parts")
    assert not project.has("calibration")

    scene = project.get("scene")
    objects = scene.get("objects") or []
    assert len(objects) >= 4  # 输送带/料箱/托盘/工件/吸盘……不只是机器人

    program = project.get("program")
    types = program["point_types"]
    assert len(types) == 8  # 五点式抓取循环：8 点
    assert types.count("pick") == 1
    assert types.count("place") == 1
    # 示教点逐条可解析（正文里是真 movej 行）
    moves = [ln for ln in program["code"].splitlines() if ln.startswith("movej")]
    assert len(moves) == 8
    assert len(program["joint_names"]) == 6  # UR5e


def test_grasping_template_roundtrip_is_a_normal_project(tmp_path):
    project = build_project_template("grasping_workstation")
    target = project.save(tmp_path / "tpl.orolp")
    assert target.exists()

    back = Project.load(target)
    assert back.name == project.name
    assert back.has("scene") and back.has("program")
    assert back.get("program")["point_types"] == project.get("program")["point_types"]
    # 摘要如实说"工程里没有"哪些段
    text = back.describe()
    assert "工艺时序" in text
    assert "设备点表" in text


def test_templates_build_fresh_instances_every_time():
    a = build_project_template("grasping_workstation")
    b = build_project_template("grasping_workstation")
    assert a is not b
    assert a.get("scene") is not b.get("scene")


@pytest.mark.parametrize(
    "tid", ["sorting_station", "machine_tending", "3c_assembly", "auto_welding"]
)
def test_scene_templates_have_home_starter_program(tid):
    project = build_project_template(tid)
    assert project.has("scene")
    program = project.get("program")
    assert program["code"].startswith("# OLP Path Code")
    moves = [ln for ln in program["code"].splitlines() if ln.startswith("movej")]
    assert len(moves) == 1  # Home 位起步
    assert program["point_types"] == ["via"]
    assert "Home 位起步" in project.note


def test_expanded_templates_carry_their_v137_scene():
    """v1.3.8：3C 装配 / 汽车焊装工程模板的场景与 v1.3.7 预置场景库同源。"""
    p3c = build_project_template("3c_assembly")
    names_3c = {o["name"] for o in p3c.get("scene")["objects"]}
    assert {"上料托盘", "板件_1", "装配工作台", "装配治具", "成品托盘", "安全围栏"} <= names_3c

    pweld = build_project_template("auto_welding")
    names_weld = {o["name"] for o in pweld.get("scene")["objects"]}
    assert {"焊接变位机", "焊接工件", "输送辊道", "待焊钣金_1", "焊接控制柜"} <= names_weld


def test_unknown_template_id_raises_keyerror():
    with pytest.raises(KeyError) as excinfo:
        build_project_template("nope")
    assert "machine_tending" in str(excinfo.value)  # 错误里说清可用的选项


def test_save_project_template_helper(tmp_path):
    target = save_project_template("sorting_station", tmp_path / "s.orolp")
    back = Project.load(target)
    assert back.has("scene")
    raw = json.loads(target.read_text(encoding="utf-8"))
    assert raw["schema"] == "orolp.project/1"
