"""预置场景库内容（v1.3.3 分拣工位/机床上下料，v1.3.7 3C 装配/汽车焊装）的验收测试。

守的东西：

* **PRD §4.1 P2 的"内容就绪再开放入口"**——``assets/cells/`` 里现在有
  五个内置场景（抓取工作站 + 分拣工位 + 机床上下料 + 3C 装配 + 汽车焊装），
  GUI「载入预置工作站」无需任何改动即可列出（内容驱动，不做空壳入口）。
* **模板是完整可用的**——机器人、工具、TCP、工件、落件区一个不少，
  场景自身零干涉（布局经过碰撞 oracle 校准，不是摆出来就行）。
* **诚实口径**——机床上下料的机床是柜体几何简化示意，3C 的板件/治具
  与焊装的变位机/机器人本体也是工程示意，描述文案里必须写明
  （真机 CAD 走厂商包通路，焊枪 TCP 不猜、走标定通路）。
"""

from __future__ import annotations

import sys
import warnings
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

from orolp.world import Scene, build_template, list_templates  # noqa: E402
from orolp.world.templates import (  # noqa: E402
    AUTO_WELDING_ID,
    MACHINE_TENDING_ID,
    SORTING_STATION_ID,
    THREE_C_ASSEMBLY_ID,
)

warnings.filterwarnings("ignore")

_PRESET_IDS = (
    SORTING_STATION_ID,
    MACHINE_TENDING_ID,
    THREE_C_ASSEMBLY_ID,
    AUTO_WELDING_ID,
)


def test_list_templates_has_five_builtins():
    ids = {t["id"] for t in list_templates()}
    assert {"grasping_workstation", *_PRESET_IDS} <= ids


def test_preset_cell_json_files_exist_and_load():
    """「载入预置工作站」走 assets/cells/*.json——内容必须真实在盘、可 Scene.load。"""
    cells = REPO_ROOT / "assets" / "cells"
    for tid in _PRESET_IDS:
        path = cells / f"{tid}.json"
        assert path.exists(), f"缺少预置场景文件 {path}"
        scene = Scene.load(path)
        assert scene.has_robot
        assert len(scene.objects) >= 6


def test_sorting_station_is_complete_and_collision_free():
    cell = build_template(SORTING_STATION_ID)
    names = set(cell.scene.objects)
    for required in ("来料输送带", "料箱_合格", "料箱_不合格", "安全围栏", "吸盘工具"):
        assert required in names, f"分拣工位缺少 {required}"
    assert cell.scene.robot_urdf and "ur5e" in cell.scene.robot_urdf.lower()
    assert len(cell.grasp_objects) == 3
    assert cell.place_target == "料箱_合格"
    assert cell.tcp_flange is not None
    assert cell.scene.check_collisions() == []


def test_machine_tending_is_complete_and_collision_free():
    cell = build_template(MACHINE_TENDING_ID)
    names = set(cell.scene.objects)
    for required in ("上料输送带", "加工工作台", "台钳", "数控机床（示意）", "成品料箱", "夹爪工具"):
        assert required in names, f"机床上下料缺少 {required}"
    assert len(cell.grasp_objects) == 2
    assert cell.place_target == "成品料箱"
    # 台钳挂在工作台下（父级局部坐标，随台移动）
    assert cell.scene.objects["台钳"].parent == "加工工作台"
    assert cell.scene.check_collisions() == []


def test_machine_tending_discloses_simplified_machine():
    """诚实口径：机床是简化示意这件事必须写在描述里。"""
    cell = build_template(MACHINE_TENDING_ID)
    assert "简化示意" in cell.description
    cell2 = build_template(SORTING_STATION_ID)
    assert "示意" in cell2.description


def test_3c_assembly_is_complete_and_collision_free():
    """v1.3.7：3C 装配工位——板件上料 → 治具压装 → 成品托盘。"""
    cell = build_template(THREE_C_ASSEMBLY_ID)
    names = set(cell.scene.objects)
    for required in (
        "上料托盘", "板件_1", "板件_2", "板件_3",
        "装配工作台", "装配治具", "成品托盘", "安全围栏", "夹爪工具",
    ):
        assert required in names, f"3C 装配工位缺少 {required}"
    assert cell.scene.robot_urdf and "ur5e" in cell.scene.robot_urdf.lower()
    assert cell.grasp_objects == ["板件_1", "板件_2", "板件_3"]
    assert cell.place_target == "装配治具"
    assert cell.tcp_flange is not None
    # 治具挂在工作台下（父级局部坐标，随台移动）
    assert cell.scene.objects["装配治具"].parent == "装配工作台"
    assert cell.scene.check_collisions() == []


def test_auto_welding_is_complete_and_collision_free():
    """v1.3.7：汽车焊装工位——变位机弧焊 + 辊道来料。

    焊枪 TCP 依焊嘴型号而定，模板**不猜**（tcp_flange=None），
    真实值走现场标定（v0.3.4 通路）——这是刻意设计，不是遗漏。
    """
    cell = build_template(AUTO_WELDING_ID)
    names = set(cell.scene.objects)
    for required in (
        "焊接变位机", "焊接工件", "输送辊道",
        "待焊钣金_1", "待焊钣金_2", "焊接控制柜", "安全围栏", "焊枪工具",
    ):
        assert required in names, f"汽车焊装工位缺少 {required}"
    assert cell.grasp_objects == ["待焊钣金_1", "待焊钣金_2"]
    assert cell.place_target == "焊接变位机"
    assert cell.tcp_flange is None
    assert cell.scene.check_collisions() == []


def test_new_cells_disclose_simplified_geometry():
    """诚实口径：3C 板件/治具与焊装变位机/机器人本体都是工程示意。"""
    cell = build_template(THREE_C_ASSEMBLY_ID)
    assert "示意" in cell.description
    cell2 = build_template(AUTO_WELDING_ID)
    assert "示意" in cell2.description
    assert "单轴" in cell2.description  # 变位机不做联动，要说清楚


def test_preset_cells_scene_roundtrip(tmp_path):
    for tid in _PRESET_IDS:
        cell = build_template(tid)
        path = tmp_path / f"{tid}.json"
        cell.save(path)
        back = Scene.load(path)
        assert set(back.objects) == set(cell.scene.objects)
