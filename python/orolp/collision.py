"""碰撞检测模块（v0.2.1）：包围体层次 + 连杆级干涉定位 + 奇异软预警。

为什么单独一个模块
------------------
:func:`orolp.world.scene.Scene.check_collisions` 早期把机器人当成**一坨**
几何：只知道"机器人撞了托盘"，不知道撞的是大臂还是小臂，更查不出
机器人自己拧成麻花（自碰撞）。本模块把碰撞检测升级为三级结构：

1. **广相（包围体层次）**——每个连杆/每个场景对象先算一个 AABB
   （世界系），AABB 不相交的部件对直接跳过，窄相 SAT 只在包围盒
   重叠时才跑。沿几百个路点的轨迹检查时，这一层决定了整体耗时。
2. **窄相（SAT 精确判定）**——复用 :func:`orolp.world.geometry.collides`
   （盒/柱/球凸体分离轴，支持穿透容差）。
3. **结果定位**——命中带连杆名（``link_a``）、零件序号，沿轨迹检查时
   还能回填首次发生时刻（``time``）与指令序号（``motion``），即
   "撞在哪一段、撞在哪个物体"。

自碰撞（连杆-连杆）
--------------------
URDF 里任何两个由关节直连的 link（无论可动还是 fixed）在装配位形下
本来就走在一起，检查它们只会恒定误报；fixed 关节串联的 link 是刚体
整体，同样豁免。其余连杆对全部参与检查——例如 UR5e 腕部三轴折叠
时 ``forearm ↔ wrist_1`` 这类跨关节干涉就是靠它们抓出来的。

奇异软预警
----------
复用 :class:`~orolp.kinematics.RobotKinematics.jacobian`：雅可比条件数
超过阈值（与规划器同一标准）时给出**警告级**提示——不是错误，但
关节速度可能突变，值得在编程阶段看见。

单位：场景一致用毫米；关节角用**度**（与 Scene 约定一致）。
"""

from __future__ import annotations

from dataclasses import replace
from typing import TYPE_CHECKING, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .world.geometry import AABB, collides, prim_aabb
from .world.scene import ROBOT_NAME, Collision, Scene

if TYPE_CHECKING:  # 仅类型标注，避免运行时导入
    from .simulation import Trajectory

__all__ = [
    "CollisionChecker",
    "parts_aabb",
    "singularity_proximity",
]


# --------------------------------------------------------------------------- #
# 包围体
# --------------------------------------------------------------------------- #


def parts_aabb(parts: Sequence) -> Optional[AABB]:
    """一组世界系零件的联合 AABB（包围体层次的第一层）。

    Args:
        parts: :class:`~orolp.world.components.Part` 序列

    Returns:
        :class:`~orolp.world.geometry.AABB`；空序列返回 ``None``
    """
    lo: Optional[np.ndarray] = None
    hi: Optional[np.ndarray] = None
    for p in parts:
        b = prim_aabb(p.prim, p.transform)
        lo = b.lo if lo is None else np.minimum(lo, b.lo)
        hi = b.hi if hi is None else np.maximum(hi, b.hi)
    if lo is None or hi is None:
        return None
    return AABB(lo, hi)


# --------------------------------------------------------------------------- #
# 奇异软预警
# --------------------------------------------------------------------------- #


def singularity_proximity(kin, joints_deg: Sequence[float]) -> Optional[float]:
    """计算关节位形的雅可比条件数（奇异接近程度的度量）。

    与规划器共用同一个判断标准（:data:`orolp.planner.SINGULARITY_COND`，
    见 :meth:`CollisionChecker.check_config_warnings`）。只做度量，不判定。

    Args:
        kin: :class:`~orolp.kinematics.RobotKinematics`；``None`` 直接返回 ``None``
        joints_deg: 关节角（度）

    Returns:
        条件数（有限正数）；运动学不可用或计算失败返回 ``None``
    """
    if kin is None:
        return None
    try:
        J = kin.jacobian(np.radians(np.asarray(joints_deg, dtype=np.float64)))
        cond = float(np.linalg.cond(J))
        return cond if np.isfinite(cond) and cond > 0 else None
    except Exception:
        return None


# --------------------------------------------------------------------------- #
# 连杆级碰撞检查器
# --------------------------------------------------------------------------- #


class CollisionChecker:
    """连杆级碰撞检查器：广相 AABB → 窄相 SAT → 连杆/物体定位。

    一个 checker 绑定一个场景。环境对象在检查期间视为静止，其世界系
    几何与 AABB 只算一次（:meth:`invalidate_env` 可手动失效缓存）；
    机器人连杆几何每次查询按当前关节角现算。

    Args:
        scene: :class:`~orolp.world.scene.Scene`（需已 ``set_robot``）
        margin: 干涉裕度（毫米）。负值容忍轻微接触（默认 -0.5，与
            程序校验一致）——工件贴台面、夹爪夹工件都是零间隙接触
    """

    def __init__(self, scene: Scene, margin: float = -0.5):
        self.scene = scene
        self.margin = float(margin)
        self._env_cache: Optional[List[Tuple[str, list, AABB]]] = None

    # ---------------------------- 缓存管理 ---------------------------- #

    def invalidate_env(self) -> None:
        """环境几何缓存失效（对象增删/移动后调用）。"""
        self._env_cache = None

    def _env_items(self) -> List[Tuple[str, list, AABB]]:
        """可见、非机器人、非挂载对象的世界系零件与联合 AABB（缓存）。"""
        if self._env_cache is None:
            items: List[Tuple[str, list, AABB]] = []
            for name, obj in self.scene.objects.items():
                if name == ROBOT_NAME or not obj.visible:
                    continue
                # 挂在机器人下的对象（工具、被抓工件）与连杆必然接触，豁免
                if self.scene._mounted_on_robot(name):
                    continue
                parts = self.scene.object_parts(name)
                box = parts_aabb(parts)
                if parts and box is not None:
                    items.append((name, parts, box))
            self._env_cache = items
        return self._env_cache

    # ---------------------------- 邻接关系 ---------------------------- #

    def _self_exemption(self) -> Tuple[set, Dict[str, str]]:
        """连杆对的豁免结构与刚体分组（自碰撞检查用）。

        Returns:
            ``(adj, group)``——``adj`` 是关节直连的连杆名对集合
            （无序化保存为 tuple 排序键）；``group`` 把 link 名映射到
            fixed 刚体组的代表元
        """
        kin = self.scene.kinematics
        adj: set = set()
        parent: Dict[str, str] = {}

        def find(x: str) -> str:
            while parent.get(x, x) != x:
                parent[x] = parent.get(parent[x], parent[x])
                x = parent[x]
            return x

        def union(a: str, b: str) -> None:
            parent.setdefault(a, a)
            parent.setdefault(b, b)
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[ra] = rb

        if kin is not None:
            for joint in kin.joints:
                a, b = joint.parent, joint.child
                adj.add((a, b) if a <= b else (b, a))
                if not joint.movable:
                    union(a, b)  # fixed 串联 = 刚体整体
        return adj, {k: find(k) for k in list(parent)}

    # ---------------------------- 单点查询 ---------------------------- #

    def check_config(
        self,
        joints_deg: Sequence[float],
        include_self: bool = True,
    ) -> List[Collision]:
        """检查给定关节位形下的干涉（连杆-环境 + 可选自碰撞）。

        会把场景机器人摆到 ``joints_deg``（与 :meth:`Scene.set_robot_joints`
        同一状态通道），与规划器沿轨迹检查的做法一致。

        Args:
            joints_deg: 关节角（度），长度 = 自由度
            include_self: 是否做连杆-连杆自碰撞检查

        Returns:
            :class:`~orolp.world.scene.Collision` 列表（连杆-环境命中
            ``obj_a="robot"``、``link_a=<连杆名>``；自碰撞 ``obj_a=obj_b=
            "robot"``、``link_a``/``link_b`` 为两个连杆名）
        """
        self.scene.set_robot_joints(joints_deg)
        link_parts = self.scene.robot_link_parts()
        if not link_parts:
            return []

        link_aabb: Dict[str, AABB] = {}
        for lname, parts in link_parts.items():
            box = parts_aabb(parts)
            if box is not None:
                link_aabb[lname] = box

        env = self._env_items()
        results: List[Collision] = []

        # 广相裕度：正裕度（提前预警）需要把包围盒向外扩；负裕度（容忍
        # 接触）时 AABB 本身就是保守超集，不再收缩——宁可多查不可漏检
        broad_margin = max(self.margin, 0.0)

        # 连杆 vs 环境：广相 AABB，重叠才进窄相 SAT
        for lname, lbox in link_aabb.items():
            for ename, eparts, ebox in env:
                if not lbox.overlaps(ebox, margin=broad_margin):
                    continue
                for i, lp in enumerate(link_parts[lname]):
                    for j, ep in enumerate(eparts):
                        if collides(
                            lp.prim, lp.transform, ep.prim, ep.transform,
                            margin=self.margin,
                        ):
                            results.append(
                                Collision(
                                    "robot", ename, i, j, link_a=lname
                                )
                            )
                            break
                    else:
                        continue
                    break

        # 自碰撞：连杆两两，跳过关节直连与 fixed 刚体组
        if include_self and len(link_aabb) > 1:
            adj, group = self._self_exemption()
            names = sorted(link_aabb)
            for i, la in enumerate(names):
                for lb in names[i + 1:]:
                    if (la, lb) in adj:
                        continue
                    ga, gb = group.get(la), group.get(lb)
                    if ga is not None and ga == gb:
                        continue
                    if not link_aabb[la].overlaps(
                        link_aabb[lb], margin=broad_margin
                    ):
                        continue
                    for pi, pa in enumerate(link_parts[la]):
                        for pj, pb in enumerate(link_parts[lb]):
                            if collides(
                                pa.prim, pa.transform, pb.prim, pb.transform,
                                margin=self.margin,
                            ):
                                results.append(
                                    Collision(
                                        "robot", "robot", pi, pj,
                                        link_a=la, link_b=lb,
                                    )
                                )
                                break
                        else:
                            continue
                        break
        return results

    def check_config_warnings(self, joints_deg: Sequence[float]) -> List[str]:
        """给定位形的奇异软预警（警告级，不影响"是否可执行"的判定）。"""
        cond = singularity_proximity(self.scene.kinematics, joints_deg)
        if cond is None:
            return []
        from .planner import SINGULARITY_COND

        if cond > SINGULARITY_COND:
            return [
                f"接近奇异位形（雅可比条件数 {cond:.2e}），"
                "关节速度可能突变，建议调整姿态或路径"
            ]
        return []

    # ---------------------------- 轨迹查询 ---------------------------- #

    def check_trajectory(
        self,
        traj: "Trajectory",
        stride: int = 0,
        motion_at=None,
    ) -> List[Collision]:
        """沿轨迹逐路点（密集采样）检查干涉。

        两路点之间机器人可能已经撞上，因此按 stride 密集采样而不是只查
        路点本身；末点必查。

        Args:
            traj: :class:`~orolp.simulation.Trajectory`
            stride: 采样步长；``0`` 表示自动（按轨迹长度控制在约 300 次）
            motion_at: ``t(秒) -> 指令序号`` 的回调（规划器提供），用于把
                干涉定位回"哪一段"；``None`` 则不填 ``motion``

        Returns:
            去重后的 :class:`~orolp.world.scene.Collision` 列表（同一对
            连杆/物体只记首次发生，带 ``time`` 与 ``motion``）
        """
        wps = traj.waypoints
        if not wps or not self.scene.has_robot:
            return []

        n = len(wps)
        if stride <= 0:
            stride = max(1, n // 300)

        seen: Dict[Tuple[str, str, str, str], Collision] = {}
        for i in list(range(0, n, stride)) + [n - 1]:
            wp = wps[i]
            for hit in self.check_config(
                np.asarray(wp.joints, dtype=np.float64)
            ):
                key = (
                    hit.link_a or "",
                    hit.link_b or "",
                    hit.obj_a,
                    hit.obj_b,
                )
                if key in seen:
                    continue
                motion = None
                if motion_at is not None and wp.timestamp is not None:
                    try:
                        motion = int(motion_at(float(wp.timestamp)))
                    except Exception:
                        motion = None
                seen[key] = replace(
                    hit, time=float(wp.timestamp), motion=motion
                )
        return list(seen.values())
