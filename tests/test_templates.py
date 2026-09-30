"""工作单元场景模板（templates.py）的回归测试。

守的东西：

* **一键工作站是完整可用的**——机器人、工具、TCP、工件、落件区一个不少，
  场景自身零干涉（贴面摆放不得误报）。
* **JSON 往返无损**——模板元数据随场景一起存取，且纯 Scene.load 不受
  ``template`` 键影响（通用交换格式不被污染）。
* **组件库新增件几何正确**——料箱/空托盘能生成零件。
"""

from __future__ import annotations

import json
import sys
import warnings
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from domain.pose import Pose  # noqa: E402
from domain.world import (
    COMPONENTS,
    Scene,
    build_component,
    build_grasping_workstation,
    build_template,
    list_templates,
    load_workcell,
)
from domain.world.components import get_component  # noqa: E402
from domain.world.scene import ROBOT_NAME  # noqa: E402
from domain.world.templates import GRASPING_WORKSTATION_ID  # noqa: E402

warnings.filterwarnings("ignore")

ASSET = REPO_ROOT / "examples/assets/grasping_workstation.json"


# --------------------------------------------------------------------------- #
# 抓取工作站模板
# --------------------------------------------------------------------------- #


def test_template_registry_contains_grasping_workstation():
    ids = {t["id"] for t in list_templates()}
    assert GRASPING_WORKSTATION_ID in ids


def test_build_template_by_id():
    cell = build_template(GRASPING_WORKSTATION_ID)
    assert cell.id == GRASPING_WORKSTATION_ID
    assert cell.scene.has_robot
    # 每次构建返回独立实例
    other = build_template(GRASPING_WORKSTATION_ID)
    assert other.scene is not cell.scene


def test_workstation_contents():
    cell = build_grasping_workstation()
    scene = cell.scene
    assert scene.robot_urdf.endswith("ur5e.urdf")

    names = set(scene.objects)
    for expected in ("吸盘工具", "输送带", "料箱", "托盘", "工件_1", "工件_4"):
        assert expected in names, f"缺少对象: {expected}"

    # 恰好 4 个可抓工件，且都挂在场景根（不在机器人上）
    assert len(cell.grasp_objects) == 4
    for name in cell.grasp_objects:
        assert scene.objects[name].parent is None

    # 工具挂在法兰上
    assert scene.objects[cell.tool].parent == ROBOT_NAME
    # 落件区存在
    assert cell.place_target in scene.objects

    # 首载关节角与机器人自由度一致（6 轴）
    assert len(cell.home_joints) == 6


def test_tcp_definition():
    cell = build_grasping_workstation()
    tcp = cell.tcp_flange
    assert tcp is not None
    x, y, z = tcp.matrix[:3, 3]
    assert (x, y, z) == pytest.approx((0.0, 0.0, 77.0), abs=1e-6)
    # TCP 姿态与法兰对齐（程序生成用 approach = 法兰 +Z）
    assert np.allclose(tcp.matrix[:3, :3], np.eye(3), atol=1e-9)


def test_tool_follows_flange():
    cell = build_grasping_workstation()
    scene = cell.scene
    # 工具世界位姿应包含法兰变换：零位 + 工具安装高度下 Z 明显大于台面
    tf = scene.world_transform(cell.tool)
    assert tf[2, 3] > 300.0
    # 工件不随机器人动（未挂载）
    before = scene.world_transform("工件_1")[:3, 3].copy()
    scene.set_robot_joints([10, -60, 60, -60, 60, 10])
    after = scene.world_transform("工件_1")[:3, 3]
    assert np.allclose(before, after)
    # 而工具跟着动
    tool_after = scene.world_transform(cell.tool)[:3, 3]
    assert not np.allclose(tf[:3, 3], tool_after)


def test_workstation_collision_free():
    """工件贴面摆在皮带上（留 2mm 间隙），整站零干涉。"""
    cell = build_grasping_workstation()
    hits = cell.scene.check_collisions()
    assert hits == [], f"工作站自身不应有干涉: {hits}"


def test_workpieces_within_reach():
    """4 个工件全部落在 UR5e 850mm 臂展内（抓取平面）。"""
    cell = build_grasping_workstation()
    for name in cell.grasp_objects:
        p = cell.scene.world_transform(name)[:3, 3]
        r = float(np.hypot(p[0], p[1]))
        assert r <= 850.0, f"{name} 超出臂展: {r:.0f}mm"


# --------------------------------------------------------------------------- #
# 持久化
# --------------------------------------------------------------------------- #


def test_save_load_roundtrip(tmp_path):
    cell = build_grasping_workstation()
    path = tmp_path / "cell.json"
    cell.save(path)

    loaded = load_workcell(path)
    assert loaded.id == cell.id
    assert loaded.tool == cell.tool
    assert loaded.place_target == cell.place_target
    assert loaded.grasp_objects == cell.grasp_objects
    assert np.allclose(loaded.tcp_flange.matrix, cell.tcp_flange.matrix)
    assert len(loaded.scene.objects) == len(cell.scene.objects)
    assert loaded.scene.has_robot


def test_scene_load_ignores_template_metadata(tmp_path):
    """通用 Scene.load 不受 template 键影响——交换格式不被污染。"""
    cell = build_grasping_workstation()
    path = tmp_path / "cell.json"
    cell.save(path)

    scene = Scene.load(path)
    assert scene.has_robot
    assert set(scene.objects) == set(cell.scene.objects)


def test_json_asset_delivered_and_loadable():
    """examples/assets/grasping_workstation.json 必须存在且可加载。"""
    assert ASSET.exists(), "缺少交付资产 examples/assets/grasping_workstation.json"
    data = json.loads(ASSET.read_text(encoding="utf-8"))
    assert data.get("template", {}).get("id") == GRASPING_WORKSTATION_ID

    loaded = load_workcell(ASSET)
    assert loaded.scene.has_robot
    assert loaded.scene.robot_urdf.endswith("ur5e.urdf")
    assert len(loaded.grasp_objects) == 4
    # 加载后场景自身零干涉
    assert loaded.scene.check_collisions() == []


# --------------------------------------------------------------------------- #
# 新增组件：料箱 / 空托盘
# --------------------------------------------------------------------------- #


def test_material_bin_registered_and_buildable():
    spec = get_component("material_bin")
    assert spec.category == "工件"
    parts = build_component("material_bin")
    assert len(parts) >= 5  # 底板 + 四壁
    # 参数生效：更大的箱生成更高的壁
    small = build_component("material_bin", height=150)
    big = build_component("material_bin", height=600)
    assert _max_z(big) > _max_z(small)


def test_pallet_base_registered_and_buildable():
    spec = get_component("pallet_base")
    assert spec.category == "工件"
    parts = build_component("pallet_base")
    assert len(parts) >= 5  # 面板 + 三纵梁 + 底垫
    top = _max_z(build_component("pallet_base", height=144))
    assert top == pytest.approx(144.0, abs=0.5)


def _max_z(parts) -> float:
    """零件组的最高点（局部系）。"""
    from domain.world.geometry import vertices_of

    return max(float(vertices_of(p.prim, p.transform)[:, 2].max()) for p in parts)
