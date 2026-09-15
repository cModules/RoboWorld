"""拖拽摆放：落点计算（纯几何，无 Qt 依赖）。

右键 gizmo（xy / z / rz）解决的是"已选中对象的精确微调"；本模块解决的是
"**把工件/料箱摆到工作台上**"的落点语义——拖动只管 XY，松手时**自动落到
正下方的支承面**：要么落在某个对象的顶面上，要么落在地面。此前 Z 只能
在属性面板里手填，摆一次工件要点三次、填一个数。

设计取舍：

* **支承面由几何算出来，不是猜的**：向下找 XY 覆盖落点、且顶面最高的
  可见对象（机器人不算支承——工件不该"落在"机器人头上）；
* **落的是"底面"不是"原点"**：对象原点常在几何中心（Box 中心在局部
  原点），直接把原点放到支承面会让工件半个沉进台面——用
  :func:`ground_offset` 补上"原点到自身包围盒底部"的差；
* **挂载对象不参与摆放**：挂在法兰/工作台上的子对象有自己的父系约束，
  松手"落到地面"等于把装配关系拆了——明确拒绝并说明原因；
* 本模块只算与写 pose，**不碰 Qt**：同一套落点逻辑既给 GLWidget 的
  左键拖拽用，也给无头测试与脚本用。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

from ..i18n import tr
from ..pose import Pose
from .geometry import AABB, prim_aabb
from .scene import Scene

__all__ = [
    "GROUND_NAME",
    "PlacementResult",
    "world_aabb",
    "support_height_at",
    "ground_offset",
    "snap",
    "apply_world_pose",
    "place_on_support",
    "placement_summary",
    "grabbable",
]

#: 没有任何对象覆盖落点时的支承者名（显示时经 tr() 翻译）
GROUND_NAME = "地面"


@dataclass
class PlacementResult:
    """一次摆放的结果。

    Attributes:
        name: 对象名
        x / y / z: 最终世界坐标（毫米；``moved=False`` 时为对象原位）
        support: 支承者名（对象名或 ``GROUND_NAME``）
        moved: 是否真的写回了位姿（不可拖动的对象返回 ``False``）
        reason: ``moved=False`` 时给人看的说明
    """

    name: str
    x: float
    y: float
    z: float
    support: str = GROUND_NAME
    moved: bool = True
    reason: str = ""


def world_aabb(scene: Scene, name: str) -> Optional[AABB]:
    """对象在**世界系**下的合并 AABB（毫米）；没有可显示几何时返回 ``None``。

    与 GLWidget 拾取用的是同一套"解析 AABB 优先、mesh 顶点包络兜底"的口径。
    """
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    for part in scene.object_parts(name):
        try:
            box = prim_aabb(part.prim, part.transform)
            plo, phi = box.lo, box.hi
        except Exception:
            mesh = getattr(part.prim, "mesh", None)
            if mesh is None or getattr(mesh, "is_empty", True):
                continue
            rot = part.transform[:3, :3]
            t = part.transform[:3, 3]
            pts = (np.asarray(mesh.vertices, dtype=np.float64) @ rot.T) + t
            plo, phi = pts.min(axis=0), pts.max(axis=0)
        lo = np.minimum(lo, plo)
        hi = np.maximum(hi, phi)
    if not np.all(np.isfinite(lo)):
        return None
    return AABB(lo, hi)


def support_height_at(
    scene: Scene, x: float, y: float, ignore: Iterable[str] = ()
) -> Tuple[float, str]:
    """点 ``(x, y)`` 正下方的支承面高度（毫米）与支承者名。

    规则：在所有**可见、非机器人**的对象里，取 XY 包围落点、顶面最高的
    那一个；没有任何对象覆盖时落到地面（``z=0``）。``ignore`` 用于把
    正在拖动的对象自己排除掉——否则工件永远"落在自己头上"。
    """
    skipped = set(ignore)
    best_z = 0.0
    best_name = GROUND_NAME
    for name in scene.objects:
        if name in skipped or name in scene.robots:
            continue
        obj = scene.objects.get(name)
        if obj is None or not obj.visible:
            continue
        box = world_aabb(scene, name)
        if box is None:
            continue
        if box.lo[0] <= x <= box.hi[0] and box.lo[1] <= y <= box.hi[1]:
            top = float(box.hi[2])
            if top > best_z:
                best_z = top
                best_name = name
    return best_z, best_name


def ground_offset(scene: Scene, name: str) -> float:
    """对象**原点**到自身包围盒底部的世界 Z 差（毫米，通常 ≤ 0）。

    "把底面放到支承面上" ⇒ ``origin_z = 支承面高度 - ground_offset``。
    没有几何时返回 0（原点即底面，落点就是支承面高度）。
    """
    box = world_aabb(scene, name)
    if box is None:
        return 0.0
    origin = scene.world_transform(name)[:3, 3]
    return float(box.lo[2] - origin[2])


def snap(value: float, step: float) -> float:
    """按 ``step``（毫米）吸附；``step <= 0`` 表示不吸附，原样返回。"""
    if not step or step <= 0:
        return float(value)
    return round(float(value) / float(step)) * float(step)


def grabbable(scene: Scene, name: Optional[str]) -> bool:
    """对象是否允许拖拽摆放。

    机器人（本体）不参与——拖动机器人基座不是"摆放"；**任何挂载对象**
    （挂在法兰/工作台上的子对象）也不参与——松手"落到地面"等于把装配
    关系拆了；``locked`` 的对象同样明确拒绝。
    """
    if not name or name not in scene.objects:
        return False
    if scene.is_robot(name):
        return False
    obj = scene.objects.get(name)
    if obj is None or not obj.visible or obj.locked:
        return False
    if obj.parent is not None:
        return False
    return True


def apply_world_pose(
    scene: Scene, name: str, x_mm: float, y_mm: float, z_mm: float, rz_deg: float
) -> None:
    """把对象设到指定**世界**位姿（毫米 + 绕 Z 角度），自动换算回局部 pose。

    挂载对象必须用 ``parent_transform``（含父对象位姿）求逆，否则会错位。
    GLWidget 的 ``_set_object_world_pose`` 即本函数的薄包装——世界系 →
    局部系的换算只有这一处。
    """
    obj = scene.objects.get(name)
    if obj is None:
        return
    if obj.parent and obj.parent in scene.objects:
        parent_tf = scene.parent_transform(name)
        inv = np.linalg.inv(parent_tf)
        lp = inv @ np.array([x_mm, y_mm, z_mm, 1.0])
        lx, ly, lz = lp[:3]
        parent_rz = float(np.degrees(np.arctan2(parent_tf[1, 0], parent_tf[0, 0])))
        local_rz = rz_deg - parent_rz
    else:
        lx, ly, lz, local_rz = x_mm, y_mm, z_mm, rz_deg
    obj.pose = Pose.from_xyz_rpy(lx, ly, lz, 0.0, 0.0, local_rz)


def place_on_support(
    scene: Scene,
    name: str,
    x: float,
    y: float,
    *,
    snap_step: float = 0.0,
) -> PlacementResult:
    """把场景对象摆到 ``(x, y)`` 并落到正下方支承面（直接写 pose）。

    Args:
        scene: 工作单元场景（毫米）
        name: 对象名；机器人 / 挂载对象 / ``locked`` 对象返回 ``moved=False``
        x / y: 落点（毫米，对象**原点**的世界 XY）
        snap_step: XY 吸附步长（毫米）；``<=0`` 不吸附

    Returns:
        :class:`PlacementResult`（``support`` 说明落在了谁上面）
    """
    obj = scene.objects.get(name) if name else None
    if obj is None:
        return PlacementResult(name or "", x, y, 0.0, moved=False,
                               reason=tr("场景里没有 {name}").format(name=name))
    if scene.is_robot(name):
        return PlacementResult(name, x, y, 0.0, moved=False,
                               reason=tr("{name} 是机器人——拖拽摆放只用于工件与设备").format(name=name))
    if obj.locked:
        return PlacementResult(name, x, y, 0.0, moved=False,
                               reason=tr("{name} 不能拖动：对象已锁定").format(name=name))
    if obj.parent is not None:
        return PlacementResult(name, x, y, 0.0, moved=False,
                               reason=tr("{name} 不能拖动：已挂载在 {host} 上，请先解除挂载").format(
                                   name=name, host=obj.parent))

    sx = snap(x, snap_step)
    sy = snap(y, snap_step)
    support_z, support_name = support_height_at(scene, sx, sy, ignore=(name,))
    z = support_z - ground_offset(scene, name)

    tf = scene.world_transform(name)
    rz = float(np.degrees(np.arctan2(tf[1, 0], tf[0, 0])))
    apply_world_pose(scene, name, sx, sy, z, rz)
    return PlacementResult(name, sx, sy, z, support_name)


def placement_summary(result: PlacementResult) -> str:
    """状态栏一行小结（用户可见文案，经 ``tr()`` 翻译）。"""
    if not result.moved:
        return result.reason or tr("{name} 位置未变").format(name=result.name)
    support = tr("地面") if result.support == GROUND_NAME else result.support
    return tr("{name} → ({x:.1f}, {y:.1f}, {z:.1f}) mm · 落在 {support} 上").format(
        name=result.name, x=result.x, y=result.y, z=result.z, support=support,
    )
