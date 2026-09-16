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
from .scene import GEOMETRY_COLLISION, Scene

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
    "mounted_on_robot",
    "flange_snap_target",
    "mount_to_flange",
    "mount_on_top_of",
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
    # 同拾取口径：机器人 <visual> 全是 mesh，object_parts 只剩一个 TCP 标记球，
    # 用它算"机器人的底面"等于按 TCP 球去贴合，会得出离谱的高度
    if scene.is_robot(name):
        parts = [
            p
            for group in scene.robot_world_link_parts(
                name, source=GEOMETRY_COLLISION
            ).values()
            for p in group
        ]
    else:
        parts = scene.object_parts(name)
    for part in parts:
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


def mount_on_top_of(scene: Scene, name: str, host: str) -> bool:
    """把对象摆到 ``host`` 的**上表面中心**（保持当前朝向）。

    "机器人坐在底座上"要的就是这个：底座上表面正中央，安装面贴合顶面。
    不把朝向归零，避免把用户调好的安装方位洗掉——只写平移。

    机器人基座原点就在安装面，配合按碰撞包络算的 :func:`ground_offset`
    （≈0），落点自然等于顶面高度。

    Args:
        scene: 工作单元场景（毫米）
        name: 被摆放的对象名
        host: 支承它的对象名（非机器人）

    Returns:
        是否写入成功；``host`` 没有可显示几何时返回 ``False``
    """
    box = world_aabb(scene, host)
    if box is None:
        return False
    cx = 0.5 * (float(box.lo[0]) + float(box.hi[0]))
    cy = 0.5 * (float(box.lo[1]) + float(box.hi[1]))
    z = float(box.hi[2]) - ground_offset(scene, name)

    world = np.asarray(scene.world_transform(name), dtype=np.float64).copy()
    world[:3, 3] = [cx, cy, z]
    local = np.linalg.inv(scene.world_transform(host)) @ world
    scene.objects[name].pose = Pose(local)
    return True


def snap(value: float, step: float) -> float:
    """按 ``step``（毫米）吸附；``step <= 0`` 表示不吸附，原样返回。"""
    if not step or step <= 0:
        return float(value)
    return round(float(value) / float(step)) * float(step)


def mounted_on_robot(scene: Scene, name: Optional[str]) -> bool:
    """对象当前是否挂在某台机器人的法兰下（直接子对象）。"""
    obj = scene.objects.get(name) if name else None
    if obj is None or obj.parent is None:
        return False
    return obj.parent in scene.robots


def flange_snap_target(scene: Scene, name: str, radius_mm: float) -> Optional[str]:
    """对象原点是否落在某台机器人法兰的 ``radius_mm`` 半径内。

    多台机器人时取**最近**的一台——工作站里常有两台臂，装错就是撞机。

    Args:
        scene: 工作单元场景（毫米）
        name: 待判定对象名
        radius_mm: 吸附判定半径（毫米）

    Returns:
        命中的机器人名；未命中（或无机器人）返回 ``None``
    """
    if name not in scene.objects or not scene.robots:
        return None
    p = scene.world_transform(name)[:3, 3]
    best: Optional[str] = None
    best_d = float("inf")
    for r in scene.robot_names:
        flange_w = (scene.world_transform(r) @ scene.flange_transform(r))[:3, 3]
        d = float(np.linalg.norm(p - flange_w))
        if d <= radius_mm and d < best_d:
            best, best_d = r, d
    return best


def mount_to_flange(scene: Scene, name: str, robot: str) -> None:
    """把对象装夹到机器人法兰：局部位姿归零，工具坐标系与法兰对齐。"""
    scene.attach(name, robot, keep_world=False)


def grabbable(scene: Scene, name: Optional[str]) -> bool:
    """对象是否允许拖拽摆放。

    **机器人本体可以拖**——搬运机器人基座是工作单元布局的常规操作（拖到
    哪就落地安装，见 :func:`place_on_support`）。``locked`` 的对象明确拒绝。
    **挂在普通对象下的子对象**也不参与——松手"落到地面"等于把装配关系拆了。
    例外是**挂在机器人法兰下的工具/工件**：它可以在起拖时先自动解除挂载再
    拖（由调用方 detach），这是"拆下工具"的正常语义。
    """
    if not name or name not in scene.objects:
        return False
    obj = scene.objects.get(name)
    if obj is None or not obj.visible or obj.locked:
        return False
    if obj.parent is not None and obj.parent not in scene.robots:
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
        # 机器人与工件/设备同规则：落到**正下方支承面**顶面。把机器人放到底座
        # 或台面上是常规布局，强制落地反而会把它从底座上拽回地面。
        sx = snap(x, snap_step)
        sy = snap(y, snap_step)
        support_z, support_name = support_height_at(scene, sx, sy, ignore=(name,))
        z = support_z - ground_offset(scene, name)
        tf = scene.world_transform(name)
        rz = float(np.degrees(np.arctan2(tf[1, 0], tf[0, 0])))
        apply_world_pose(scene, name, sx, sy, z, rz)
        return PlacementResult(name, sx, sy, z, support_name)
    if obj.locked:
        return PlacementResult(name, x, y, 0.0, moved=False,
                               reason=tr("{name} 不能拖动：对象已锁定").format(name=name))
    if obj.parent is not None and obj.parent not in scene.robots:
        # 挂在机器人法兰下的工具由调用方在起拖时 detach，这里不再重复拒绝
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
