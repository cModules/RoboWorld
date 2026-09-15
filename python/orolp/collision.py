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

采样口径（v1.1.9 显式化）
-------------------------
沿轨迹检查不可能逐点细看（一条几千路点的程序逐点检查会把校验拖到几十秒），
所以必须采样；**采多密**是一个必须写在明处的工程决定，而不是内部实现细节。
三种口径见 :func:`sample_indices`：固定步长、历史口径（按长度控制在
:data:`COLLISION_SAMPLE_TARGET` 次左右）、关节空间步长
（``max_joint_step``，密度由"机器人可能移动多远"决定）。默认保持历史口径
以免改变既有判定；关心"会不会漏检"的场合应显式给 ``max_joint_step``。

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

import hashlib
from dataclasses import replace
from typing import TYPE_CHECKING, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .i18n import tr
from .world.geometry import (
    AABB,
    Sphere,
    collides,
    collides_with_boxes,
    penetration_depth,
    prim_aabb,
    sat_axes,
    union_aabb,
    vertices_of,
)
from .world.scene import ROBOT_NAME, Collision, Scene

if TYPE_CHECKING:  # 仅类型标注，避免运行时导入
    from .simulation import Trajectory

__all__ = [
    "COLLISION_SAMPLE_TARGET",
    "DEFAULT_MAX_JOINT_STEP_DEG",
    "CollisionChecker",
    "collision_stats",
    "parts_aabb",
    "parts_separation",
    "prim_separation",
    "reset_collision_stats",
    "sample_indices",
    "singularity_proximity",
]


# --------------------------------------------------------------------------- #
# 工作量统计（v1.1.9）
# --------------------------------------------------------------------------- #
#
# "这次检查到底干了多少活"与"花了多久"是两个问题：耗时随机器负载抖动，
# 而工作量（采样了几次、多少候选对进了窄相、广相挡掉了多少）是可复现的。
# 计数器只增不减，用差分读。

# 注意：这里**不**再记"命中几次"——判定级的原始命中次数由
# :func:`orolp.world.geometry.geometry_stats` 的 ``collision_hits`` 统一计数
# （沿轨迹检查每次采样命中同一条幅都会 +1，与去重后的结果条数不是一回事）。
_stats = {
    "config_checks": 0,
    "level1_tests": 0,
    "level1_rejects": 0,
    "narrow_pairs": 0,
}


def collision_stats() -> Dict[str, int]:
    """沿轨迹干涉检查的工作量统计（v1.1.9）。

    Returns:
        ``config_checks``（检查过的位形数，即采样次数）、``level1_tests``
        （连杆-物体/连杆-连杆的候选对测试次数）、``level1_rejects``
        （被广相 AABB 剔除的候选对数）、``narrow_pairs``（进入窄相的零件对次数）。

        判定级的原始命中次数见 :func:`orolp.world.geometry.geometry_stats` 的
        ``collision_hits``；沿轨迹检查**去重后**的结果条数是返回值本身。
    """
    return dict(_stats)


def reset_collision_stats() -> None:
    """清零工作量计数器（测试隔离用）。"""
    for key in _stats:
        _stats[key] = 0


# --------------------------------------------------------------------------- #
# 采样口径
# --------------------------------------------------------------------------- #

#: 默认采样预算（``stride=0`` 且未给 ``max_joint_step``）：把检查次数控制在
#: 这个量级。它是**与程序长度相关的密度**——程序越长，采样点之间隔得越远。
COLLISION_SAMPLE_TARGET = 300

#: 关节空间采样步长的推荐值（度）。给 ``max_joint_step`` 用：相邻两个采样点
#: 之间任一关节的变化不超过它。取 10° 的理由是它与 :func:`orolp.planner` 的
#: 轨迹插补尺度同量级——规划器按关节行程决定路点密度，检查按同一把尺子取样
#: 才不会出现"规划看得见、检查看不见"的缝。
DEFAULT_MAX_JOINT_STEP_DEG = 10.0


def _joints_of(waypoint) -> np.ndarray:
    """路点的关节值（一维 float64 数组）。"""
    return np.asarray(waypoint.joints, dtype=np.float64).ravel()


def sample_indices(
    waypoints: Sequence,
    stride: int = 0,
    max_joint_step: Optional[float] = None,
    target: int = COLLISION_SAMPLE_TARGET,
) -> List[int]:
    """沿轨迹选择要检查的路点下标（采样口径只在这一处定义）。

    三种口径，优先级从上到下：

    1. ``max_joint_step`` 给定 —— **关节空间步长**。贪心地走：一旦相对上一个
       采样点任一关节的变化达到该度数，就出下一个采样点。它保证"任意两个采样
       点之间，没有关节走过超过 ``max_joint_step``"，于是采样密度由**机器人
       可能移动多远**决定，不随程序变长而稀释。``0`` 表示逐路点。
    2. ``stride > 0`` —— 固定步长。
    3. 都未给（默认）—— **历史口径**：``max(1, n // target)``，使检查次数大致
       恒定在 ``target`` 次。程序越长、相邻采样点隔得越远，这是"检查成本恒定、
       保证随规模变弱"的取舍，保留它只为不改变既有判定。

    三种口径都**必查末点**，且返回的下标严格递增、无重复。

    Args:
        waypoints: :class:`~orolp.simulation.Waypoint` 序列
        stride: 固定步长；``<=0`` 表示不用这一口径
        max_joint_step: 关节空间步长（度）；``None`` 表示不用这一口径
        target: 历史口径的目标采样次数

    Returns:
        升序下标列表；``waypoints`` 为空时返回空列表

    Raises:
        ValueError: ``max_joint_step`` 为负或非有限值
    """
    n = len(waypoints)
    if n == 0:
        return []

    if max_joint_step is not None:
        step = float(max_joint_step)
        if not np.isfinite(step) or step < 0.0:
            raise ValueError(
                tr(
                    "max_joint_step 必须是非负有限值，收到 {value}——它是采样判据"
                    "（度），负值没有意义"
                ).format(value=max_joint_step)
            )
        if step == 0.0:
            return list(range(n))
        indices = [0]
        q_ref = _joints_of(waypoints[0])
        width = q_ref.size
        for i in range(1, n):
            q = _joints_of(waypoints[i])
            if q.size != width:
                # 关节数变化只可能来自损坏的轨迹；按实际重叠部分比较而不是崩掉
                width_here = min(width, q.size)
                delta = q[:width_here] - q_ref[:width_here]
            else:
                delta = q - q_ref
            if delta.size and float(np.abs(delta).max()) >= step:
                indices.append(i)
                q_ref = q
        if indices[-1] != n - 1:
            indices.append(n - 1)
        return indices

    if stride > 0:
        step_int = int(stride)
    else:
        step_int = max(1, n // max(1, int(target)))
    indices = list(range(0, n, step_int))
    if indices[-1] != n - 1:
        indices.append(n - 1)
    return indices


# --------------------------------------------------------------------------- #
# 干涉检查结果缓存键（v1.2.2）
# --------------------------------------------------------------------------- #


def collision_cache_key(
    scene: Scene,
    robot_name: Optional[str],
    traj: "Trajectory",
    margin: float,
    stride: int,
    max_joint_step: Optional[float],
) -> bytes:
    """沿轨迹干涉检查的**内容寻址键**（v1.2.2，供结果缓存使用）。

    键由全部影响判定结果的输入派生——命中就不可能给出过期结论：

    * 被查机器人：名字、基座**世界位姿**、内存里的连杆几何（URDF 改了盘
      而场景没重建也不串味——键取的是场景实际持有的基本体）；
    * 环境对象：逐对象取名字 + 世界系零件几何。世界位姿经
      :meth:`~orolp.world.scene.Scene.world_transform` 计算，父级链
      （含挂在其他机器人下时随法兰变动的部分）天然被折进键里；
      **挂在被查机器人下的对象被豁免**（:meth:`CollisionChecker._env_items`
      的同一口径），因此它们的位姿不进键——被查机器人在场景里的关节角
      变化也就不影响键；
    * 轨迹：只有关节值影响判定（逐路点字节），加路点数兜底；
    * 参数：``margin`` / ``stride`` / ``max_joint_step``。

    刻意**不**进键的东西：其他机器人在场景里的关节角——它们不是环境项
    （:meth:`CollisionChecker.check_trajectory` 只查「被查机器人 vs 环境 +
    自碰撞」），其几何只在机器人-机器人对检查里出现，而那条路径不经过本键。

    Returns:
        32 字节 SHA256 摘要
    """
    h = hashlib.sha256()
    h.update(b"orolp-collision-v1")
    name = robot_name or (scene.robot_names[0] if scene.robot_names else "")
    h.update(name.encode("utf-8"))
    h.update(b"\x00")
    h.update(repr(float(margin)).encode("ascii"))
    h.update(b"\x00")
    h.update(repr(int(stride)).encode("ascii"))
    h.update(b"\x00")
    h.update(repr(None if max_joint_step is None else float(max_joint_step)).encode("ascii"))
    h.update(b"\x00")

    # 被查机器人：基座世界位姿 + 连杆几何
    state = scene.robots.get(name)
    if state is not None:
        try:
            base = scene.world_transform(name)
        except KeyError:
            base = None
        if base is not None:
            h.update(np.ascontiguousarray(base, dtype=np.float64).tobytes())
        for lname, part in state.geometry("collision"):
            h.update(lname.encode("utf-8"))
            h.update(b"\x01")
            h.update(repr(part.prim).encode("utf-8"))
            h.update(b"\x01")
            h.update(
                np.ascontiguousarray(
                    np.asarray(part.transform, dtype=np.float64)
                ).tobytes()
            )
    h.update(b"\x02")

    # 环境对象：可见、非机器人、不挂在被查机器人下
    for oname, obj in scene.objects.items():
        if scene.is_robot(oname) or not obj.visible:
            continue
        if scene.mounted_robot(oname) == name:
            continue
        h.update(oname.encode("utf-8"))
        h.update(b"\x03")
        for part in scene.object_parts(oname):
            h.update(repr(part.prim).encode("utf-8"))
            h.update(b"\x03")
            h.update(
                np.ascontiguousarray(
                    np.asarray(part.transform, dtype=np.float64)
                ).tobytes()
            )
    h.update(b"\x02")

    wps = traj.waypoints
    h.update(len(wps).to_bytes(8, "little"))
    for wp in wps:
        joints = getattr(wp, "joints", None)
        if joints is None:
            h.update(b"N")
        else:
            h.update(
                np.ascontiguousarray(
                    np.asarray(joints, dtype=np.float64).ravel()
                ).tobytes()
            )
    return h.digest()


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
    return union_aabb(prim_aabb(p.prim, p.transform) for p in parts)


# --------------------------------------------------------------------------- #
# 分离距离（多机器人"离多近"的度量）
# --------------------------------------------------------------------------- #


def prim_separation(
    prim_a,
    tf_a: np.ndarray,
    prim_b,
    tf_b: np.ndarray,
) -> float:
    """两个基本体之间的最小间距（毫米）。

    ``>0`` 表示净空距离，``<0`` 表示穿透深度（取绝对值意义上的重叠量）。

    精度说明：净空距离用**顶点云**近似（:func:`vertices_of` 给出的凸包顶点）。
    对盒/柱侧面相对的情形是准确的；两柱轴线交叉、或小零件贴在大平面中部
    时会偏大（保守）。它用于"两台机器人离多近"的趋势判断与预警，不要当
    验收判据——真正的判定仍走 :func:`collides` 的 SAT。

    Args:
        prim_a: 第一个基本体
        tf_a: 第一个基本体世界变换
        prim_b: 第二个基本体
        tf_b: 第二个基本体世界变换

    Returns:
        最小间距（毫米）
    """
    # 球-球解析
    if isinstance(prim_a, Sphere) and isinstance(prim_b, Sphere):
        dist = float(np.linalg.norm(tf_a[:3, 3] - tf_b[:3, 3]))
        return dist - (float(prim_a.radius) + float(prim_b.radius))

    box_a = prim_aabb(prim_a, tf_a)
    box_b = prim_aabb(prim_b, tf_b)
    if box_a.overlaps(box_b):
        verts_a = vertices_of(prim_a, tf_a)
        verts_b = vertices_of(prim_b, tf_b)
        faces_a, edges_a = sat_axes(prim_a, tf_a)
        faces_b, edges_b = sat_axes(prim_b, tf_b)
        depth = penetration_depth(verts_a, faces_a, edges_a, verts_b, faces_b, edges_b)
        if depth > 0.0:
            return -float(depth)
        return _cloud_distance(verts_a, verts_b)
    # 包围盒分离：顶点云距离即可（比逐面最近点便宜，且此处是净空量级判断）
    return _cloud_distance(vertices_of(prim_a, tf_a), vertices_of(prim_b, tf_b))


def _cloud_distance(pts_a: np.ndarray, pts_b: np.ndarray) -> float:
    """两组点的最小两两距离。"""
    if pts_a.size == 0 or pts_b.size == 0:
        return float("inf")
    d = pts_a[:, None, :] - pts_b[None, :, :]
    return float(np.sqrt(np.einsum("ijk,ijk->ij", d, d).min()))


def parts_separation(parts_a: Sequence, parts_b: Sequence) -> float:
    """两组世界系零件之间的最小间距（毫米）：见 :func:`prim_separation`。

    Args:
        parts_a: 第一组零件
        parts_b: 第二组零件

    Returns:
        最小间距；任一组为空时返回 ``inf``
    """
    if not parts_a or not parts_b:
        return float("inf")
    best = float("inf")
    for pa in parts_a:
        for pb in parts_b:
            d = prim_separation(pa.prim, pa.transform, pb.prim, pb.transform)
            if d < best:
                best = d
    return best


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
        # 环境几何缓存按机器人名分桶：挂着 A 的工具要豁免 A 的检查，但
        # 仍然要和 B 检查（A 撞上 B 夹着的工件是真实干涉）
        self._env_cache: Dict[str, List[Tuple[str, list, AABB]]] = {}
        # 环境对象**逐零件**的 AABB（按对象名缓存）。对象在检查期间静止，
        # 零件盒只算一次；不缓存的话每个 (连杆, 物体) 组合都要重算一遍，
        # 是沿轨迹检查里最大的一块重复计算（v1.1.9）
        self._env_part_boxes: Dict[str, List[AABB]] = {}

    # ---------------------------- 缓存管理 ---------------------------- #

    def invalidate_env(self) -> None:
        """环境几何缓存失效（对象增删/移动后调用）。"""
        self._env_cache = {}
        self._env_part_boxes = {}

    def _env_boxes(self, name: str, parts: Sequence) -> List[AABB]:
        """环境对象逐零件的世界系 AABB（按对象名缓存）。

        与 :meth:`_env_items` 的联合盒配套使用：联合盒回答"这一对要不要细看"，
        零件盒回答"细看时先筛掉哪些零件对"。两者随 :meth:`invalidate_env`
        一起失效。
        """
        boxes = self._env_part_boxes.get(name)
        if boxes is None:
            boxes = [prim_aabb(p.prim, p.transform) for p in parts]
            self._env_part_boxes[name] = boxes
        return boxes

    def _primary_name(self) -> Optional[str]:
        """主机器人名（未加载机器人时 ``None``）。"""
        names = self.scene.robot_names
        return names[0] if names else None

    def _env_items(
        self, robot: Optional[str] = None
    ) -> List[Tuple[str, list, AABB]]:
        """可见、非机器人对象的世界系零件与联合 AABB（按机器人缓存）。"""
        key = robot or ""
        if key not in self._env_cache:
            items: List[Tuple[str, list, AABB]] = []
            for name, obj in self.scene.objects.items():
                if self.scene.is_robot(name) or not obj.visible:
                    continue
                # 挂在**被检查的这台**机器人下的对象（工具、被抓工件）
                # 与它必然接触，豁免；挂在别的机器人下不豁免
                if robot is not None and self.scene.mounted_robot(name) == robot:
                    continue
                parts = self.scene.object_parts(name)
                box = parts_aabb(parts)
                if parts and box is not None:
                    items.append((name, parts, box))
            self._env_cache[key] = items
        return self._env_cache[key]

    # ---------------------------- 邻接关系 ---------------------------- #

    def _self_exemption(self, robot: Optional[str] = None) -> Tuple[set, Dict[str, str]]:
        """连杆对的豁免结构与刚体分组（自碰撞检查用）。

        Args:
            robot: 机器人名；``None`` 取主机器人

        Returns:
            ``(adj, group)``——``adj`` 是关节直连的连杆名对集合
            （无序化保存为 tuple 排序键）；``group`` 把 link 名映射到
            fixed 刚体组的代表元
        """
        kin = self.scene.kinematics_of(robot) if robot else self.scene.kinematics
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
        joints_deg: Optional[Sequence[float]] = None,
        include_self: bool = True,
        robot: Optional[str] = None,
    ) -> List[Collision]:
        """检查给定关节位形下的干涉（连杆-环境 + 可选自碰撞）。

        机器人侧按世界系计算几何（含基座位姿），因此多台机器人放在不同
        基座上时互不干扰。默认取**主机器人**，与历史行为一致。

        Args:
            joints_deg: 关节角（度），长度 = 自由度；``None`` 表示用该机器人
                当前已存的关节角（v1.0.1：多机器人时不必来回切状态）
            include_self: 是否做连杆-连杆自碰撞检查
            robot: 机器人名；``None`` 取主机器人（v1.0.1）

        Returns:
            :class:`~orolp.world.scene.Collision` 列表（连杆-环境命中
            ``obj_a=<机器人名>``、``link_a=<连杆名>``；自碰撞 ``obj_a=obj_b=
            <机器人名>``、``link_a``/``link_b`` 为两个连杆名）
        """
        robot_name = robot or self._primary_name()
        if robot_name is None:
            return []
        if joints_deg is not None:
            if robot is None:
                # 历史通道：写回场景状态（规划器沿轨迹检查依赖这一点）
                self.scene.set_robot_joints(joints_deg)
            else:
                self.scene.set_robot_joints(joints_deg, robot_name)

        _stats["config_checks"] += 1

        link_parts = self.scene.robot_world_link_parts(robot_name)
        if not link_parts:
            return []

        # 零件盒一次算好，宽相与窄相共用同一份：单零件连杆的零件盒**就是**
        # 连杆盒，而每个 (连杆, 物体) 组合都重算一遍是这条路径上最大的纯浪费
        link_boxes: Dict[str, List[AABB]] = {}
        link_aabb: Dict[str, AABB] = {}
        for lname, parts in link_parts.items():
            boxes = [prim_aabb(p.prim, p.transform) for p in parts]
            link_boxes[lname] = boxes
            box = union_aabb(boxes)
            if box is not None:
                link_aabb[lname] = box

        env = self._env_items(robot_name)
        results: List[Collision] = []

        # 广相裕度：正裕度（提前预警）需要把包围盒向外扩；负裕度（容忍
        # 接触）时 AABB 本身就是保守超集，不再收缩——宁可多查不可漏检
        broad_margin = max(self.margin, 0.0)

        # 连杆 vs 环境：广相 AABB，重叠才进窄相 SAT
        for lname, lbox in link_aabb.items():
            lparts = link_parts[lname]
            lboxes = link_boxes[lname]
            for ename, eparts, ebox in env:
                _stats["level1_tests"] += 1
                if not lbox.overlaps(ebox, margin=broad_margin):
                    _stats["level1_rejects"] += 1
                    continue
                eboxes = self._env_boxes(ename, eparts)
                for i, lp in enumerate(lparts):
                    for j, ep in enumerate(eparts):
                        _stats["narrow_pairs"] += 1
                        if collides_with_boxes(
                            lp.prim, lp.transform, ep.prim, ep.transform,
                            lboxes[i], eboxes[j], margin=self.margin,
                        ):
                            results.append(
                                Collision(
                                    robot_name, ename, i, j, link_a=lname
                                )
                            )
                            break
                    else:
                        continue
                    break

        # 自碰撞：连杆两两，跳过关节直连与 fixed 刚体组
        if include_self and len(link_aabb) > 1:
            adj, group = self._self_exemption(robot_name)
            names = sorted(link_aabb)
            for i, la in enumerate(names):
                for lb in names[i + 1:]:
                    if (la, lb) in adj:
                        continue
                    ga, gb = group.get(la), group.get(lb)
                    if ga is not None and ga == gb:
                        continue
                    _stats["level1_tests"] += 1
                    if not link_aabb[la].overlaps(
                        link_aabb[lb], margin=broad_margin
                    ):
                        _stats["level1_rejects"] += 1
                        continue
                    for pi, pa in enumerate(link_parts[la]):
                        for pj, pb in enumerate(link_parts[lb]):
                            _stats["narrow_pairs"] += 1
                            if collides_with_boxes(
                                pa.prim, pa.transform, pb.prim, pb.transform,
                                link_boxes[la][pi], link_boxes[lb][pj],
                                margin=self.margin,
                            ):
                                results.append(
                                    Collision(
                                        robot_name, robot_name, pi, pj,
                                        link_a=la, link_b=lb,
                                    )
                                )
                                break
                        else:
                            continue
                        break
        return results

    # ---------------------------- 机器人-机器人 ---------------------------- #

    def check_robot_pair(
        self,
        robot_a: str,
        robot_b: str,
        joints_a: Optional[Sequence[float]] = None,
        joints_b: Optional[Sequence[float]] = None,
    ) -> List[Collision]:
        """检查两台机器人**之间**的干涉（v1.0.1 多机器人协同）。

        世界系计算（各自基座位姿不同），广相 AABB 后进窄相 SAT。关节角可
        显式给出——节拍统筹要在同一时刻比较两台机器人的位形，不能改状态。

        Args:
            robot_a: 机器人 A 名
            robot_b: 机器人 B 名
            joints_a: A 的关节角（度）；``None`` 用当前存的
            joints_b: B 的关节角（度）；``None`` 用当前存的

        Returns:
            :class:`Collision` 列表（``obj_a=robot_a``、``obj_b=robot_b``、
            ``link_a``/``link_b`` 为发生干涉的两个连杆名）
        """
        if robot_a == robot_b:
            return []
        parts_a = self.scene.robot_world_link_parts(robot_a, joints_a)
        parts_b = self.scene.robot_world_link_parts(robot_b, joints_b)
        if not parts_a or not parts_b:
            return []

        boxes_a = {
            k: [prim_aabb(p.prim, p.transform) for p in v] for k, v in parts_a.items()
        }
        boxes_b = {
            k: [prim_aabb(p.prim, p.transform) for p in v] for k, v in parts_b.items()
        }
        aabb_a = {k: union_aabb(v) for k, v in boxes_a.items()}
        aabb_b = {k: union_aabb(v) for k, v in boxes_b.items()}
        broad_margin = max(self.margin, 0.0)

        results: List[Collision] = []
        for la, box_a in aabb_a.items():
            if box_a is None:
                continue
            for lb, box_b in aabb_b.items():
                if box_b is None:
                    continue
                _stats["level1_tests"] += 1
                if not box_a.overlaps(box_b, margin=broad_margin):
                    _stats["level1_rejects"] += 1
                    continue
                for pi, pa in enumerate(parts_a[la]):
                    for pj, pb in enumerate(parts_b[lb]):
                        _stats["narrow_pairs"] += 1
                        if collides_with_boxes(
                            pa.prim, pa.transform, pb.prim, pb.transform,
                            boxes_a[la][pi], boxes_b[lb][pj],
                            margin=self.margin,
                        ):
                            results.append(
                                Collision(
                                    robot_a, robot_b, pi, pj,
                                    link_a=la, link_b=lb,
                                )
                            )
                            break
                    else:
                        continue
                    break
        return results

    def check_robot_pairs(self) -> List[Collision]:
        """检查场景中**所有**机器人两两之间的干涉（当前位形）。

        Returns:
            去重后的 :class:`Collision` 列表；单机器人场景返回空列表
        """
        names = self.scene.robot_names
        results: List[Collision] = []
        for i, na in enumerate(names):
            for nb in names[i + 1:]:
                results.extend(self.check_robot_pair(na, nb))
        return results

    def separation_between(
        self,
        robot_a: str,
        robot_b: str,
        joints_a: Optional[Sequence[float]] = None,
        joints_b: Optional[Sequence[float]] = None,
    ) -> Tuple[float, Optional[str], Optional[str]]:
        """两台机器人之间的**最小间距**与对应连杆对（v1.0.1）。

        节拍统筹真正关心的是"离多近"而不只是"撞没撞"：两台机器人同时作业
        往往永远不撞，但会擦着走。本方法给出间距与最近的一对连杆，便于
        报表与预警。

        Args:
            robot_a: 机器人 A
            robot_b: 机器人 B
            joints_a: A 的关节角（度）；``None`` 用当前存的
            joints_b: B 的关节角（度）；``None`` 用当前存的

        Returns:
            ``(间距 mm, A 的连杆名, B 的连杆名)``；任一侧无几何时距离为 ``inf``
        """
        parts_a = self.scene.robot_world_link_parts(robot_a, joints_a)
        parts_b = self.scene.robot_world_link_parts(robot_b, joints_b)
        best = float("inf")
        best_pair: Tuple[Optional[str], Optional[str]] = (None, None)
        for la, pa in parts_a.items():
            for lb, pb in parts_b.items():
                d = parts_separation(pa, pb)
                if d < best:
                    best, best_pair = d, (la, lb)
        return best, best_pair[0], best_pair[1]

    def check_config_warnings(
        self, joints_deg: Sequence[float], robot: Optional[str] = None
    ) -> List[str]:
        """给定位形的奇异软预警（警告级，不影响"是否可执行"的判定）。"""
        kin = self.scene.kinematics_of(robot) if robot else self.scene.kinematics
        cond = singularity_proximity(kin, joints_deg)
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
        robot: Optional[str] = None,
        max_joint_step: Optional[float] = None,
    ) -> List[Collision]:
        """沿轨迹逐路点（密集采样）检查干涉。

        两路点之间机器人可能已经撞上，因此按采样判据密集检查而不是只查
        路点本身；末点必查。采样判据见 :func:`sample_indices`——默认是历史
        口径（按轨迹长度控制在约 :data:`COLLISION_SAMPLE_TARGET` 次），
        给 ``max_joint_step`` 则改为关节空间步长口径。

        Args:
            traj: :class:`~orolp.simulation.Trajectory`
            stride: 采样步长；``0`` 表示按默认口径自动
            motion_at: ``t(秒) -> 指令序号`` 的回调（规划器提供），用于把
                干涉定位回"哪一段"；``None`` 则不填 ``motion``
            robot: 机器人名；``None`` 取主机器人（v1.0.1）
            max_joint_step: 关节空间采样步长（度）；``None`` 表示用默认口径

        Returns:
            去重后的 :class:`~orolp.world.scene.Collision` 列表（同一对
            连杆/物体只记首次发生，带 ``time`` 与 ``motion``）

        Raises:
            ValueError: ``max_joint_step`` 为负或非有限值
        """
        wps = traj.waypoints
        if not wps or not self.scene.has_robot:
            return []

        seen: Dict[Tuple[str, str, str, str], Collision] = {}
        for i in sample_indices(wps, stride, max_joint_step):
            wp = wps[i]
            for hit in self.check_config(
                np.asarray(wp.joints, dtype=np.float64), robot=robot
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
