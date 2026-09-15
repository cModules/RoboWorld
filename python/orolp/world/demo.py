"""抓取工作站 GUI 一键演示——由模板 + IK 生成示例示教点（无头可测）。

为什么单独一个模块
------------------
「抓取工作站演示」入口（``app.py`` 菜单一键加载）要在载入场景后**自动装上
一份示例程序**，让用户不示教就能直接回放动画、看到节拍、出品牌代码。
示教点的计算依赖逆运动学（IK），与 Qt 无关——放进 GUI 代码既没法测试，
也会把 app.py 越撑越大。本模块把"模板场景 → 五点式抓取循环示教点"做成
纯函数：GUI 只负责把结果灌进路径编辑器与离线编程面板。

点位设计与 ``tools/build_web_site.py`` 的演示程序同源（同一抓取/落件位姿、
同一 TCP 定义），保证"网页演示、GUI 回放、品牌出码"讲的是同一个工作站。

单位：示教点为**度**；返回给 GUI 的 recorded_positions 为 ``{关节名: 弧度}``
（与 ``PathCodeEditor.recorded_positions`` 的既有约定一致）。
"""

from __future__ import annotations

import math
from typing import Dict, List, Sequence, Tuple

import numpy as np

from ..pickplace import PointType
from ..pose import Pose
from ..robot import Robot
from .templates import (
    WORKPIECE_HEIGHT_MM,
    _BELT_TOP_Z,
    _CONVEYOR_Y,
    _WORKPIECE_GAP_Z,
    _WORKPIECE_XS,
    WorkcellTemplate,
)

__all__ = ["demo_pick_place_points", "demo_taught_points"]

#: 托盘落件点（世界 XY，mm）——与 templates.build_grasping_workstation
#: 里托盘的中心一致（该位置经过可达性校准，见模板注释）
_PLACE_XY = (350.0, 450.0)

#: 接近/撤离安全高度（相对工件顶面）
_APPROACH_DZ = 100.0
_LIFT_DZ = 120.0
_PLACE_APPROACH_DZ = 150.0


def _place_top_z(cell: WorkcellTemplate) -> float:
    """落件高度：托盘顶面 + 工件高度（TCP 抓着工件时停在工件顶面）。"""
    height = WORKPIECE_HEIGHT_MM
    if cell.place_target and cell.place_target in cell.scene.objects:
        height = float(
            cell.scene.objects[cell.place_target].params.get(
                "height", WORKPIECE_HEIGHT_MM
            )
        )
    return height + WORKPIECE_HEIGHT_MM


def demo_pick_place_points(
    cell: WorkcellTemplate,
    robot: Robot,
    workpiece_index: int = 0,
) -> Tuple[List[List[float]], List[PointType]]:
    """为抓取工作站生成演示示教点（关节角，度）与工艺语义。

    点位序列（8 点，7 段运动）::

        Home → 接近(VIA) → 抓取接近(APPROACH) → 抓取(PICK) → 提升(LIFT)
             → 落件接近(VIA) → 落件(PLACE) → 撤离(RETRACT) → Home(VIA)

    实际是 8 个点：Home 与落件接近点无工艺动作，标 VIA。抓取点/放置点
    的语义交给 :func:`orolp.pickplace.gripper_action_for`，由程序构建侧
    自动插入 ``gripper()`` 指令——回放动画与出码共用同一份时序。

    Args:
        cell: 抓取工作站模板实例（提供 TCP、Home 关节角、工件/落件位置）
        robot: :class:`orolp.Robot`（含运动学）。其 :attr:`Robot.tool`
            会被设为模板定义的 TCP——演示程序、节拍与出码因此都是
            TCP 语义的，与真实吸盘一致
        workpiece_index: 抓取第几个工件（0 起）；超出范围时取最后一个

    Returns:
        ``(points_deg, types)``——关节角列表（度，长度 = 自由度）与
        等长的 :class:`PointType` 语义列表

    Raises:
        KinematicsError: 某个示教位姿逆解失败（ unreachable / 奇异）
    """
    robot.tool = cell.tcp_flange

    home = [float(v) for v in cell.home_joints]

    idx = min(max(workpiece_index, 0), len(_WORKPIECE_XS) - 1)
    pick_z = _BELT_TOP_Z + _WORKPIECE_GAP_Z + WORKPIECE_HEIGHT_MM
    pick = Pose.from_xyz_rpy(
        _WORKPIECE_XS[idx], _CONVEYOR_Y, pick_z, 180, 0, 0
    )
    place = Pose.from_xyz_rpy(*_PLACE_XY, _place_top_z(cell), 180, 0, 0)

    # 初值用 Home，之后每一步拿上一步的解当 guess——保证位形连续、不翻腕。
    # 数值求解器对初值敏感（DLS 局部极小），链式 guess 失败时再补几组
    # 随机初值重试，避免演示入口被一次偶发的不收敛卡死。
    guess: np.ndarray = np.asarray(home, dtype=np.float64)

    def ik(pose: Pose) -> List[float]:
        nonlocal guess
        attempts = [list(guess)]
        rng = np.random.default_rng(42)
        attempts += [
            [float(v) for v in rng.uniform(-180.0, 180.0, robot.dof)]
            for _ in range(4)
        ]
        for g in attempts:
            try:
                q = robot.inverse_kinematics(pose, guess=g)
            except Exception:
                continue
            guess = np.asarray(q, dtype=np.float64)
            return [float(v) for v in q]
        raise RuntimeError(
            f"演示示教点逆解失败（含随机重试）: "
            f"TCP=({pose.matrix[0, 3]:.0f}, {pose.matrix[1, 3]:.0f}, "
            f"{pose.matrix[2, 3]:.0f}) mm"
        )

    points_deg: List[List[float]] = [home]
    types: List[PointType] = [PointType.VIA]

    plan = (
        (pick.with_z_offset(_APPROACH_DZ), PointType.APPROACH),
        (pick, PointType.PICK),
        (pick.with_z_offset(_LIFT_DZ), PointType.LIFT),
        (place.with_z_offset(_PLACE_APPROACH_DZ), PointType.VIA),
        (place, PointType.PLACE),
        (place.with_z_offset(_PLACE_APPROACH_DZ), PointType.RETRACT),
        (home, PointType.VIA),
    )
    for pose, t in plan[:-1]:
        points_deg.append(ik(pose))
        types.append(t)
    # 回 Home 用关节插值：直接用已知关节角，不再做 IK
    points_deg.append(home)
    types.append(plan[-1][1])

    return points_deg, types


def demo_taught_points(
    cell: WorkcellTemplate,
    robot: Robot,
    workpiece_index: int = 0,
) -> Tuple[List[Dict[str, float]], List[PointType]]:
    """生成 GUI 直接可用的示教点：``{关节名: 弧度}`` 字典列表 + 语义。

    Args:
        cell: 抓取工作站模板实例
        robot: :class:`orolp.Robot`（会被设置 TCP，见
            :func:`demo_pick_place_points`）
        workpiece_index: 抓取第几个工件（0 起）

    Returns:
        ``(positions, types)``——``positions`` 可直接赋给
        ``PathCodeEditor.recorded_positions``，``types`` 赋给
        ``point_types``；两者等长对齐
    """
    points_deg, types = demo_pick_place_points(cell, robot, workpiece_index)
    names = list(robot.joint_names)
    positions = [
        {n: math.radians(v) for n, v in zip(names, q)} for q in points_deg
    ]
    return positions, list(types)
