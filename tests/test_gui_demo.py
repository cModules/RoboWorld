"""抓取工作站一键演示（S4）：示教点生成逻辑的无头测试。

GUI 侧（app.py 的菜单入口）只负责把这里生成的示教点灌进路径编辑器与
离线编程面板；点位计算全部在本模块覆盖：

* 点数 / 工艺语义序列 / 自由度对齐
* 抓取点与落件点的 FK 回读（TCP 落在工件顶面，位置误差 < 0.5 mm）
* Home 起止一致、相邻解位形连续（guess 链不翻腕）
* GUI 数据形态（``{关节名: 弧度}``）与语义等长对齐
"""

import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from domain.robot import Robot  # noqa: E402
from service.pickplace import PointType  # noqa: E402
from domain.world.demo import demo_pick_place_points, demo_taught_points  # noqa: E402
from domain.world.templates import (
    _BELT_TOP_Z,
    _CONVEYOR_Y,
    _WORKPIECE_GAP_Z,
    _WORKPIECE_XS,
    WORKPIECE_HEIGHT_MM,
    build_grasping_workstation,
)

URDF = REPO_ROOT / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"

#: FK 回读容差（毫米）——IK 收敛阈值 0.01 mm，留足余量
POS_TOL_MM = 0.5


@pytest.fixture(scope="module")
def cell():
    return build_grasping_workstation()


@pytest.fixture(scope="module")
def robot():
    return Robot(str(URDF), "UR5e")


@pytest.fixture(scope="module")
def points_and_types(cell, robot):
    return demo_pick_place_points(cell, robot)


def _fk_position(robot, joints_deg):
    pose = robot.forward_kinematics(list(joints_deg))
    return tuple(float(v) for v in pose.matrix[:3, 3])


# --------------------------------------------------------------------------- #
# 点数与语义
# --------------------------------------------------------------------------- #


def test_point_count_matches_types(points_and_types, robot):
    points, types = points_and_types
    assert len(points) == 8
    assert len(types) == len(points)
    for q in points:
        assert len(q) == robot.dof


def test_type_sequence(points_and_types):
    _, types = points_and_types
    assert types == [
        PointType.VIA,  # Home
        PointType.APPROACH,  # 抓取接近
        PointType.PICK,  # 抓取（闭合）
        PointType.LIFT,  # 提升
        PointType.VIA,  # 落件接近
        PointType.PLACE,  # 落件（张开）
        PointType.RETRACT,  # 撤离
        PointType.VIA,  # 回 Home
    ]


def test_gripper_semantics_present(points_and_types):
    """恰好一次抓取、一次放置——回放动画与出码的 IO 都从这里来。"""
    _, types = points_and_types
    assert types.count(PointType.PICK) == 1
    assert types.count(PointType.PLACE) == 1


# --------------------------------------------------------------------------- #
# FK 回读：示教点真的落在工件/托盘上
# --------------------------------------------------------------------------- #


def test_pick_point_fk_roundtrip(points_and_types, robot, cell):
    points, _ = points_and_types
    x, y, z = _fk_position(robot, points[2])
    assert x == pytest.approx(_WORKPIECE_XS[0], abs=POS_TOL_MM)
    assert y == pytest.approx(_CONVEYOR_Y, abs=POS_TOL_MM)
    assert z == pytest.approx(_BELT_TOP_Z + _WORKPIECE_GAP_Z + WORKPIECE_HEIGHT_MM, abs=POS_TOL_MM)


def test_place_point_fk_roundtrip(points_and_types, robot, cell):
    points, _ = points_and_types
    x, y, z = _fk_position(robot, points[5])
    pallet_obj = cell.scene.objects[cell.place_target]
    pallet_h = float(pallet_obj.params["height"])
    px, py = (float(v) for v in pallet_obj.pose.matrix[:2, 3])
    assert x == pytest.approx(px, abs=POS_TOL_MM)
    assert y == pytest.approx(py, abs=POS_TOL_MM)
    assert z == pytest.approx(pallet_h + WORKPIECE_HEIGHT_MM, abs=POS_TOL_MM)


def test_workpiece_index_selects_target(cell, robot):
    points, _ = demo_pick_place_points(cell, robot, workpiece_index=2)
    x, y, _ = _fk_position(robot, points[2])
    assert x == pytest.approx(_WORKPIECE_XS[2], abs=POS_TOL_MM)
    assert y == pytest.approx(_CONVEYOR_Y, abs=POS_TOL_MM)


# --------------------------------------------------------------------------- #
# 位形质量
# --------------------------------------------------------------------------- #


def test_home_start_and_end_identical(points_and_types, cell):
    points, _ = points_and_types
    home = [float(v) for v in cell.home_joints]
    assert points[0] == pytest.approx(home, abs=1e-9)
    assert points[-1] == pytest.approx(home, abs=1e-9)


def test_adjacent_solutions_continuous(points_and_types):
    """相邻示教点没有接近满圈的腕部翻转。

    上限取 300°：基座/腕部轴（±360° 限位）回 Home 时允许大角度关节插值
    （实测约 232°），但仍应远小于 360°——出现接近满圈的差值说明 guess 链
    失效、解在多值分支间跳变。
    """
    points, _ = points_and_types
    for a, b in zip(points, points[1:]):
        for va, vb in zip(a, b):
            assert abs(vb - va) < 300.0


def test_template_layout_reachable(cell, robot):
    """回归守卫：模板布局必须保持 IK 可达。

    2026-09-13 曾发现输送带（y=-600 / 皮带面 768mm）超出 UR5e 模型工作空间，
    IK 全链路无解且被上游静默跳过——此测试保证布局改动永远先过 IK 这关。
    """
    points, types = demo_pick_place_points(cell, robot)
    assert len(points) == len(types) == 8


def test_unreachable_pose_raises(cell, robot, monkeypatch):
    """落件点被挪到工作空间外时，IK 失败必须抛错而不是返回假解。"""
    import domain.world.demo as demo

    def far_place_top_z(_cell):
        return 5000.0

    monkeypatch.setattr(demo, "_place_top_z", far_place_top_z)
    with pytest.raises(Exception):  # KinematicsError
        demo_pick_place_points(cell, robot)


# --------------------------------------------------------------------------- #
# GUI 数据形态
# --------------------------------------------------------------------------- #


def test_demo_taught_points_shape(cell, robot):
    positions, types = demo_taught_points(cell, robot)
    names = list(robot.joint_names)
    assert len(positions) == len(types)
    for pos in positions:
        assert set(pos.keys()) == set(names)
        for v in pos.values():
            # 关节限位 ±360°（j1/j4/j6 为 ±2π），求解器解可越出 ±π
            assert abs(v) <= 2 * math.pi + 1e-3  # 弧度


def test_demo_taught_points_matches_degrees(cell, robot):
    points_deg, types_deg = demo_pick_place_points(cell, robot)
    positions, types = demo_taught_points(cell, robot)
    names = list(robot.joint_names)
    for q, pos in zip(points_deg, positions):
        for name, value in zip(names, q):
            assert pos[name] == pytest.approx(math.radians(value), abs=1e-9)
    assert types == types_deg
