"""增量重算：改一个点不必把整条轨迹重跑一遍。

## 为什么需要它

v1.1.8 的基准把大程序的耗时逐段量了出来；v1.1.9 又把沿路径干涉检查压到了约 0.17 ms/采样。
剩下的大头就只剩一件事：**每一次改动都全量重算**。

实测（UR5e + 抓取工作站，120 条运动 = 184 条指令 / 5632 路点）：

* ``plan()`` 约 263 ms，其中 **约 80% 是逐路点的正运动学**（5632 次 ``link_transforms``）；
* ``validate()`` 内部会**自己再规划一次**；
* 界面上"校验一下"这个动作因此会规划 **三次**（程序校验 → 面板回放 → 各自的规划器实例），
  而三次的输入**逐字段相同**。

## 这是怎么做到的

两层，都是**内容寻址**（content-addressed）而不是"记住上次的对象"：

1. **整程序层**：键 =（规划器参数、机器人结构、起始关节值、整程序内容摘要）。
   命中则直接交还一份 :class:`PlanSnapshot`——路点、指令时间轴、驻留段、路径长度、
   问题清单全部复用，连 ``_populate_dynamics`` 都不再跑。
2. **分段层**：键 =（规划器参数、机器人结构、**单条指令的内容签名**、**该条指令的起始关节值**）。
   逐条指令复用插补结果（关节序列 / 位姿序列 / 每步时长 / 路径长度 / 失败与警告）。
   于是"改了第 k 条"时，**0..k-1 全部命中**；若改动只影响时间（改速度、插驻留）或
   终点回到了原处，**k+1 之后也照样命中**。

## 它**不能**做什么（写在最前面，省得被误解）

* **改前面一条运动的位置，后面就复用了**——不能。第 k 条指令的插补以第 k 条的起点为初值，
  位置改了终点就变了，其后整条链的起点都变，分段键随之改变。这是串行运动链的固有性质，
  不是实现偷懒。真正能省的是"前缀 + 非几何改动 + 终点回原处"这三类。
* **它缓存干涉检查**的**结果**（v1.2.2 起，见 :class:`CollisionResultCache`），**也缓存动力学**
  的**结果**（v1.2.3 起，见 :class:`DynamicsResultCache`——力矩/功率报表与超限定位）。
  两者的键都由"全部影响结果的输入"派生（场景签名 / 模型内容签名 + 逐路点数据），
  场景或模型一动键就变，不存在读到过期结论。本模块只回答"要不要重算"，不改变任何判定。
* **它不会让首次规划更快**。冷启动仍然要完整算一遍，收益全部来自"第二次及以后"。

## 共享与只读约定

命中时交还的 :class:`~orolp.simulation.Waypoint` 与 :class:`~orolp.pose.Pose` 是**共享对象**
（不深拷贝——深拷贝 5632 个路点比重新算一遍还贵）。因此调用方**必须把它们当只读**：
``Trajectory.waypoints`` 这个**列表容器**是每次新建的，改容器无妨；但不要原地改
``waypoint.velocity`` 或 ``pose.matrix``。仓库内唯一的写入点是
:meth:`~orolp.planner.TrajectoryPlanner._populate_dynamics`，它只写**本次新建**的路点，
且整程序命中时整段跳过，所以不会污染缓存。

分段层交还的关节数组**不是**共享的——``Waypoint.__init__`` 会对 ``joints`` 做一次复制，
缓存里的数组交不出去。
"""

from __future__ import annotations

import hashlib
from collections import OrderedDict
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

if TYPE_CHECKING:  # pragma: no cover - 只为类型检查
    from .planner import Issue
    from .pose import Pose
    from .simulation import Waypoint


__all__ = [
    "PLAN_CACHE_MAX_ENTRIES",
    "PLAN_CACHE_MAX_WAYPOINTS",
    "SEGMENT_CACHE_MAX_ENTRIES",
    "SEGMENT_CACHE_MAX_WAYPOINTS",
    "SegmentPayload",
    "PlanSnapshot",
    "TrajectoryCache",
    "item_signature",
    "program_digest",
    "robot_signature",
    "planner_signature",
    "resolve_cache",
    "get_cache",
    "set_cache",
    "incremental_stats",
    "reset_incremental_stats",
    "COLLISION_CACHE_MAX_ENTRIES",
    "CollisionResultCache",
    "get_collision_cache",
    "set_collision_cache",
    "resolve_collision_cache",
    "DYNAMICS_CACHE_MAX_ENTRIES",
    "DynamicsResultCache",
    "get_dynamics_cache",
    "set_dynamics_cache",
    "resolve_dynamics_cache",
]


# --------------------------------------------------------------------------- #
# 容量上限
# --------------------------------------------------------------------------- #
#
# 缓存键是内容寻址的，所以"过期"这件事不存在；需要管的只有**内存**。
# 一个 Waypoint 约 0.6~0.7 KB（关节数组 + 4×4 位姿 + 速度 + 加速度），
# 5632 路点的程序一份快照就是 3~4 MB——按"条数"限容会悄悄吃掉几百 MB，
# 所以这里按**路点数**限容，条数只作为兜底。

#: 整程序快照最多保留几份
PLAN_CACHE_MAX_ENTRIES = 4
#: 整程序快照合计最多保留多少个路点（超出按最旧淘汰）
PLAN_CACHE_MAX_WAYPOINTS = 20_000
#: 分段缓存最多保留几条指令
SEGMENT_CACHE_MAX_ENTRIES = 4_096
#: 分段缓存合计最多保留多少个路点
SEGMENT_CACHE_MAX_WAYPOINTS = 40_000


def _f64b(value: Any) -> bytes:
    """把一个数值数组变成规范化字节串（缓存键用）。"""
    arr = np.ascontiguousarray(np.asarray(value, dtype=np.float64).ravel())
    return arr.tobytes()


# --------------------------------------------------------------------------- #
# 签名
# --------------------------------------------------------------------------- #


def item_signature(item: Any) -> str:
    """单条指令的内容签名（规范化 JSON，与程序指纹同一套编码）。"""
    from .progdiff import instruction_signature

    return instruction_signature(item)


def program_digest(signatures: Sequence[str]) -> str:
    """整程序内容摘要：由逐条指令签名拼接后哈希。

    与 :func:`orolp.progdiff.program_fingerprint` 同一口径（逐字段相同才相同），
    但接受**已经算好的**逐条签名——分段缓存本来就需要它们，不该算两遍。
    """
    h = hashlib.sha256()
    for sig in signatures:
        h.update(sig.encode("utf-8"))
        h.update(b"\x00")
    return h.hexdigest()


def robot_signature(robot: Any) -> bytes:
    """机器人几何签名：结构变了就不许复用旧轨迹。

    刻意**逐关节取原始字节**而不是只记 URDF 路径：``JointSpec`` 是冻结 dataclass，
    但它持有的 ndarray 本身可写，现场与测试都可能原地改链长做实验
    （同样的理由写在 :mod:`orolp.kinematics` 的常量缓存里）。取字节能兜住这种情况，
    代价只有几十微秒。
    """
    kin = getattr(robot, "kinematics", None)
    parts: List[bytes] = [
        b"orolp-robot-v1",
        str(getattr(robot, "name", "")).encode("utf-8"),
        int(getattr(robot, "dof", 0)).to_bytes(4, "little"),
    ]
    if kin is not None:
        parts.append(int(getattr(kin, "dof", 0)).to_bytes(4, "little"))
        for joint in kin.chain:
            parts.append(str(getattr(joint, "name", "")).encode("utf-8"))
            parts.append(_f64b(getattr(joint, "origin_xyz", ())))
            parts.append(_f64b(getattr(joint, "origin_rpy", ())))
            parts.append(_f64b(getattr(joint, "axis", ())))
            parts.append(b"t" if getattr(joint, "is_prismatic", False) else b"r")
    # 工具与基座参与 FK，必须进签名
    for attr in ("tool", "base"):
        pose = getattr(robot, attr, None)
        matrix = getattr(pose, "matrix", None)
        if matrix is not None:
            parts.append(_f64b(matrix))
    return b"\x01".join(parts)


def planner_signature(planner: Any) -> bytes:
    """规划器参数签名：任何影响插补结果的设置变了就换键。

    含插补分辨率、TCP 速度/加速度上限、三组容差、速度曲线风格，以及
    :class:`~orolp.planner.AxisLimits` 的四个数组（它们虽由机器人推导，但
    规划器构造后就固定在自己的实例上，改机器人不会被感知）。
    """
    from .planner import (  # 局部导入：planner 反向依赖本模块，顶层导入成环
        DEFAULT_ANGULAR_RES,
    )

    text = "|".join(
        [
            f"{float(getattr(planner, 'linear_res', 0.0))!r}",
            f"{float(getattr(planner, 'angular_res', DEFAULT_ANGULAR_RES))!r}",
            f"{float(getattr(planner, 'tcp_speed', 0.0))!r}",
            f"{float(getattr(planner, 'tcp_accel', 0.0))!r}",
            f"{float(getattr(planner, 'path_tol_mm', 0.0))!r}",
            f"{float(getattr(planner, 'path_rot_tol_deg', 0.0))!r}",
            f"{float(getattr(planner, 'target_tol_mm', 0.0))!r}",
            f"{float(getattr(planner, 'target_rot_tol_deg', 0.0))!r}",
            str(getattr(planner, "profile", "")),
        ]
    )
    parts = [b"orolp-planner-v1", text.encode("utf-8")]
    limits = getattr(planner, "limits", None)
    if limits is not None:
        for key in ("lower", "upper", "max_speed", "max_accel"):
            parts.append(_f64b(getattr(limits, key, ())))
    return b"\x02".join(parts)


# --------------------------------------------------------------------------- #
# 缓存条目
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class SegmentPayload:
    """一条指令的插补展开结果（分段复用的最小单位）。

    Attributes:
        pts: 插补关节序列（每步一个数组）——交给 :class:`~orolp.simulation.Waypoint`
            时会被复制，因此这里的数组不会被外部改到
        poses: 插补位姿序列（**共享对象，只读**）
        step_dts: 每步时长 (s)
        path_length: 该条指令贡献的 TCP 路径长度 (mm)
        error: 展开失败的文案；``None`` 表示展开成功
        error_code: 失败时的问题码（与 :class:`~orolp.planner.Issue` 同口径）
        warn: 指令终点审计的警告文案（**不含序号与时刻**——序号随插入删除而变，
            时刻随前缀时长而变，两者都在应用时按当下值重建）
        empty: 展开成功但零位移（起点即终点）
    """

    pts: Tuple[np.ndarray, ...] = ()
    poses: Tuple["Pose", ...] = ()
    step_dts: Tuple[float, ...] = ()
    path_length: float = 0.0
    error: Optional[str] = None
    error_code: str = ""
    warn: Optional[str] = None
    empty: bool = False

    @property
    def failed(self) -> bool:
        """该条指令是否展开失败。"""
        return self.error is not None


@dataclass(frozen=True)
class PlanSnapshot:
    """整程序命中时直接交还的完整规划结果。

    字段与 :meth:`~orolp.planner.TrajectoryPlanner.plan` 在 ``self`` 上留下的副作用
    一一对应（``_motion_spans`` / ``_dwell_segments`` / ``_last_path_length`` / ``issues``），
    这样命中路径与实算路径对调用方完全无差别。

    Attributes:
        waypoints: 路点元组（**共享对象，只读**）
        spans: 每条 :class:`~orolp.program.Motion` 的时间区间
        dwell_segments: 驻留段 ``(指令序号, 秒, t0, t1)``
        path_length: TCP 路径总长 (mm)
        issues: 本次规划产生的问题（含时刻——整程序相同则时刻必然相同）
        waypoint_count: 路点数（限容与统计用，免得每次 ``len()``）
    """

    waypoints: Tuple["Waypoint", ...] = ()
    spans: Tuple[Tuple[float, float], ...] = ()
    dwell_segments: Tuple[Tuple[int, float, float, float], ...] = ()
    path_length: float = 0.0
    issues: Tuple["Issue", ...] = ()
    waypoint_count: int = 0


# --------------------------------------------------------------------------- #
# 缓存本体
# --------------------------------------------------------------------------- #


class TrajectoryCache:
    """内容寻址的规划结果缓存（整程序层 + 分段层）。

    线程安全不是本类的目标（规划器本来就不是线程安全的），但**所有键都是内容派生的**，
    因此多线程各自持有一份缓存也不会出现"读到别人的结果"这类脏数据。
    """

    def __init__(
        self,
        *,
        enabled: bool = True,
        plan_max_entries: int = PLAN_CACHE_MAX_ENTRIES,
        plan_max_waypoints: int = PLAN_CACHE_MAX_WAYPOINTS,
        segment_max_entries: int = SEGMENT_CACHE_MAX_ENTRIES,
        segment_max_waypoints: int = SEGMENT_CACHE_MAX_WAYPOINTS,
    ):
        self.enabled = bool(enabled)
        self.plan_max_entries = max(1, int(plan_max_entries))
        self.plan_max_waypoints = max(1, int(plan_max_waypoints))
        self.segment_max_entries = max(1, int(segment_max_entries))
        self.segment_max_waypoints = max(1, int(segment_max_waypoints))
        self._plans: "OrderedDict[Any, PlanSnapshot]" = OrderedDict()
        self._segments: "OrderedDict[Any, SegmentPayload]" = OrderedDict()
        self._plan_waypoints = 0
        self._segment_waypoints = 0
        self._counters: Dict[str, int] = {
            "plan_hits": 0,
            "plan_misses": 0,
            "plan_stores": 0,
            "segment_hits": 0,
            "segment_misses": 0,
            "segment_stores": 0,
            "evictions": 0,
        }

    # ---------------------------- 整程序层 ---------------------------- #

    def lookup_plan(self, key: Any) -> Optional[PlanSnapshot]:
        """查整程序快照；无论命中与否都会记一次计数。"""
        if not self.enabled:
            return None
        snapshot = self._plans.get(key)
        if snapshot is None:
            self._counters["plan_misses"] += 1
            return None
        self._plans.move_to_end(key)
        self._counters["plan_hits"] += 1
        return snapshot

    def store_plan(self, key: Any, snapshot: PlanSnapshot) -> None:
        """存整程序快照（超限按最旧淘汰）。"""
        if not self.enabled:
            return
        old = self._plans.pop(key, None)
        if old is not None:
            self._plan_waypoints -= old.waypoint_count
        self._plans[key] = snapshot
        self._plan_waypoints += snapshot.waypoint_count
        self._counters["plan_stores"] += 1
        self._trim()

    # ---------------------------- 分段层 ---------------------------- #

    def lookup_segment(self, key: Any) -> Optional[SegmentPayload]:
        """查一条指令的展开结果；无论命中与否都会记一次计数。"""
        if not self.enabled:
            return None
        payload = self._segments.get(key)
        if payload is None:
            self._counters["segment_misses"] += 1
            return None
        self._segments.move_to_end(key)
        self._counters["segment_hits"] += 1
        return payload

    def store_segment(self, key: Any, payload: SegmentPayload) -> None:
        """存一条指令的展开结果（含失败与警告）。"""
        if not self.enabled:
            return
        old = self._segments.pop(key, None)
        if old is not None:
            self._segment_waypoints -= len(old.pts)
        self._segments[key] = payload
        self._segment_waypoints += len(payload.pts)
        self._counters["segment_stores"] += 1
        self._trim()

    # ---------------------------- 维护 ---------------------------- #

    def _trim(self) -> None:
        """按路点预算与条数预算双重淘汰（最旧的先走）。"""
        while (
            len(self._plans) > self.plan_max_entries
            or self._plan_waypoints > self.plan_max_waypoints
        ) and self._plans:
            _key, snapshot = self._plans.popitem(last=False)
            self._plan_waypoints -= snapshot.waypoint_count
            self._counters["evictions"] += 1
        while (
            len(self._segments) > self.segment_max_entries
            or self._segment_waypoints > self.segment_max_waypoints
        ) and self._segments:
            _key, payload = self._segments.popitem(last=False)
            self._segment_waypoints -= len(payload.pts)
            self._counters["evictions"] += 1

    def clear(self) -> None:
        """清空两层的条目（计数保留——计数是"这一轮干了多少活"，不是"现在有多少"）。"""
        self._plans.clear()
        self._segments.clear()
        self._plan_waypoints = 0
        self._segment_waypoints = 0

    def reset_stats(self) -> None:
        """计数器归零（条目保留）。"""
        for key in self._counters:
            self._counters[key] = 0

    def stats(self) -> Dict[str, Any]:
        """当前占用与累计命中情况（可直接进报告）。"""
        plan_lookups = self._counters["plan_hits"] + self._counters["plan_misses"]
        seg_lookups = self._counters["segment_hits"] + self._counters["segment_misses"]
        return {
            "enabled": self.enabled,
            **self._counters,
            "plan_entries": len(self._plans),
            "plan_waypoints": self._plan_waypoints,
            "segment_entries": len(self._segments),
            "segment_waypoints": self._segment_waypoints,
            "plan_hit_rate": (
                self._counters["plan_hits"] / plan_lookups if plan_lookups else 0.0
            ),
            "segment_hit_rate": (
                self._counters["segment_hits"] / seg_lookups if seg_lookups else 0.0
            ),
        }

    def __repr__(self) -> str:  # pragma: no cover - 调试用
        return (
            f"TrajectoryCache(enabled={self.enabled}, "
            f"plans={len(self._plans)}/{self._plan_waypoints}wp, "
            f"segments={len(self._segments)}/{self._segment_waypoints}wp)"
        )


# --------------------------------------------------------------------------- #
# 全局默认缓存
# --------------------------------------------------------------------------- #

_default_cache: Optional[TrajectoryCache] = TrajectoryCache()


def get_cache() -> Optional[TrajectoryCache]:
    """取全局默认缓存（``None`` 表示已全局关闭）。

    默认开启的理由：**界面上的"校验一下"会构造多个规划器实例**（程序校验一个、
    面板回放一个），它们对同一份程序算出同一份轨迹——不共享缓存就只能各算一遍。
    内容寻址的键保证共享不会串味。
    """
    return _default_cache


def set_cache(cache: Optional[TrajectoryCache]) -> None:
    """设置/关闭全局默认缓存（``None`` 关闭；传实例即替换）。"""
    global _default_cache
    _default_cache = cache


def resolve_cache(planner: Any) -> Optional[TrajectoryCache]:
    """按规划器上的设置解析出实际要用的缓存。

    * ``planner.use_cache is False`` → 不用（无论全局开着与否）
    * ``planner.cache`` 给了实例 → 用它
    * 否则 → 全局默认缓存
    """
    if not getattr(planner, "use_cache", True):
        return None
    own = getattr(planner, "cache", None)
    if own is not None:
        return own if getattr(own, "enabled", True) else None
    cache = get_cache()
    if cache is None or not getattr(cache, "enabled", True):
        return None
    return cache


def incremental_stats() -> Dict[str, Any]:
    """全局缓存的统计（没开缓存时返回一个 ``enabled: False`` 的字典）。"""
    cache = get_cache()
    if cache is None:
        return {"enabled": False}
    return cache.stats()


def reset_incremental_stats() -> None:
    """全局缓存计数器归零（条目保留）——测试与基准的口径从这里开始。"""
    cache = get_cache()
    if cache is not None:
        cache.reset_stats()


# --------------------------------------------------------------------------- #
# 干涉检查结果缓存（v1.2.2）
# --------------------------------------------------------------------------- #

#: 干涉检查结果缓存最多保留几份。一份结果是**去重后**的 :class:`Collision`
#: 列表（冻结 dataclass，通常几条到几十条），比一份路点快照小三个数量级，
#: 按条数限容即可。
COLLISION_CACHE_MAX_ENTRIES = 64


class _LRUResultCache:
    """结果缓存的公共骨架（LRU 按条数限容 + 命中/未命中计数）。

    :class:`CollisionResultCache`（v1.2.2）与 :class:`DynamicsResultCache`
    （v1.2.3）共用：两者的键都内容寻址、值都是小体积的纯数据结果，
    管理策略完全一致——只有"谁的键、谁的值"不同。
    """

    def __init__(
        self,
        max_entries: int = 64,
        enabled: bool = True,
    ):
        self.enabled = bool(enabled)
        self.max_entries = max(1, int(max_entries))
        self._entries: "OrderedDict[bytes, Any]" = OrderedDict()
        self._hits = 0
        self._misses = 0
        self._stores = 0
        self._evictions = 0

    def lookup(self, key: bytes) -> Optional[Any]:
        """查缓存；命中返回存储的值（容器/共享约定见子类文档）。"""
        if not self.enabled:
            return None
        entry = self._entries.get(key)
        if entry is None:
            self._misses += 1
            return None
        self._hits += 1
        self._entries.move_to_end(key)
        return entry

    def store(self, key: bytes, value: Any) -> None:
        """存一份结果；同键重复存是空操作。"""
        if not self.enabled:
            return
        if key in self._entries:
            return
        self._entries[key] = value
        self._stores += 1
        while len(self._entries) > self.max_entries:
            self._entries.popitem(last=False)
            self._evictions += 1

    def clear(self) -> None:
        """清空全部条目（计数器保留）。"""
        self._entries.clear()

    def stats(self) -> Dict[str, Any]:
        """缓存统计（``hits`` / ``misses`` / ``stores`` / ``evictions`` / ``entries``）。"""
        return {
            "enabled": self.enabled,
            "hits": self._hits,
            "misses": self._misses,
            "stores": self._stores,
            "evictions": self._evictions,
            "entries": len(self._entries),
        }

    def reset_stats(self) -> None:
        """计数器归零（条目保留）。"""
        self._hits = 0
        self._misses = 0
        self._stores = 0
        self._evictions = 0

    def __repr__(self) -> str:  # pragma: no cover - 调试用
        return (
            f"{type(self).__name__}(enabled={self.enabled}, "
            f"entries={len(self._entries)}/{self.max_entries})"
        )


class CollisionResultCache(_LRUResultCache):
    """沿轨迹干涉检查的结果缓存（v1.2.2）。

    键由 :func:`orolp.collision.collision_cache_key` 从**全部影响判定的输入**
    派生（被查机器人基座与几何、每个环境对象的世界系几何、逐路点关节值、
    margin/采样口径），内容寻址——**场景一动键就变**，不存在"读到过期结论"。

    共享与只读约定与 :class:`TrajectoryCache` 相同：命中时交还的**列表容器**
    是每次新建的，容器里的 :class:`~orolp.world.scene.Collision` 是**共享的
    冻结 dataclass**（只读——冻结实例本来也改不了字段）。

    条目按 LRU 淘汰（上限 :data:`COLLISION_CACHE_MAX_ENTRIES`）。
    """

    def __init__(
        self,
        max_entries: int = COLLISION_CACHE_MAX_ENTRIES,
        enabled: bool = True,
    ):
        super().__init__(max_entries=max_entries, enabled=enabled)

    def lookup(self, key: bytes) -> Optional[List[Any]]:
        """查缓存；命中返回**新建列表**（内容为共享只读的 Collision）。"""
        entry = super().lookup(key)
        return list(entry) if entry is not None else None

    def store(self, key: bytes, collisions: Sequence[Any]) -> None:
        """存一份结果（内部转为不可变元组）。"""
        super().store(key, tuple(collisions))


_default_collision_cache: Optional[CollisionResultCache] = CollisionResultCache()


def get_collision_cache() -> Optional[CollisionResultCache]:
    """取全局默认干涉检查结果缓存（``None`` 表示已全局关闭）。"""
    return _default_collision_cache


def set_collision_cache(cache: Optional[CollisionResultCache]) -> None:
    """设置/关闭全局默认干涉检查结果缓存（``None`` 关闭；传实例即替换）。

    基准与计时场景应临时关闭（``orolp.perf`` 内部已这样做）——缓存命中的
    "干涉检查"测的不是检查。
    """
    global _default_collision_cache
    _default_collision_cache = cache


def resolve_collision_cache(planner: Any) -> Optional[CollisionResultCache]:
    """按规划器上的设置解析出实际要用的干涉检查结果缓存。

    * ``planner.use_collision_cache is False`` → 不用（无论全局开着与否）
    * ``planner.collision_cache`` 给了实例 → 用它
    * 否则 → 全局默认缓存
    """
    if not getattr(planner, "use_collision_cache", True):
        return None
    own = getattr(planner, "collision_cache", None)
    if own is not None:
        return own if getattr(own, "enabled", True) else None
    cache = get_collision_cache()
    if cache is None or not getattr(cache, "enabled", True):
        return None
    return cache


# --------------------------------------------------------------------------- #
# 力矩校验结果缓存（v1.2.3）
# --------------------------------------------------------------------------- #

#: 力矩校验结果缓存最多保留几份。一份结果是一个纯数据 dict（每关节峰值
#: 列表 + 超限定位元组列表），与干涉检查结果同量级，按条数限容即可。
DYNAMICS_CACHE_MAX_ENTRIES = 64


class DynamicsResultCache(_LRUResultCache):
    """沿轨迹力矩/功率校验的结果缓存（v1.2.3）。

    键由 :func:`orolp.dynamics.dynamics_cache_key` 从**全部影响结果的输入**
    派生（模型内容签名：连杆惯性/末端负载/额定力矩 + 逐路点关节
    位置/速度/加速度 + 指令时间区间），内容寻址——**改负载、改惯性、
    改程序都换键**，不存在"改了负载还用旧力矩表"。

    命中交还的 result dict **是共享对象**：本仓库对它只读（报表字段取引用、
    Issue 由值确定性重建）；调用方不要原地改 ``torque_peaks`` 等字段。

    条目按 LRU 淘汰（上限 :data:`DYNAMICS_CACHE_MAX_ENTRIES`）。
    """

    def __init__(
        self,
        max_entries: int = DYNAMICS_CACHE_MAX_ENTRIES,
        enabled: bool = True,
    ):
        super().__init__(max_entries=max_entries, enabled=enabled)


_default_dynamics_cache: Optional[DynamicsResultCache] = DynamicsResultCache()


def get_dynamics_cache() -> Optional[DynamicsResultCache]:
    """取全局默认力矩校验结果缓存（``None`` 表示已全局关闭）。"""
    return _default_dynamics_cache


def set_dynamics_cache(cache: Optional[DynamicsResultCache]) -> None:
    """设置/关闭全局默认力矩校验结果缓存（``None`` 关闭；传实例即替换）。

    基准与计时场景应临时关闭（``orolp.perf`` 内部已这样做）——缓存命中的
    "力矩校验"测的不是校验。
    """
    global _default_dynamics_cache
    _default_dynamics_cache = cache


def resolve_dynamics_cache(planner: Any) -> Optional[DynamicsResultCache]:
    """按规划器上的设置解析出实际要用的力矩校验结果缓存。

    * ``planner.use_dynamics_cache is False`` → 不用（无论全局开着与否）
    * ``planner.dynamics_cache`` 给了实例 → 用它
    * 否则 → 全局默认缓存
    """
    if not getattr(planner, "use_dynamics_cache", True):
        return None
    own = getattr(planner, "dynamics_cache", None)
    if own is not None:
        return own if getattr(own, "enabled", True) else None
    cache = get_dynamics_cache()
    if cache is None or not getattr(cache, "enabled", True):
        return None
    return cache


# --------------------------------------------------------------------------- #
# 缓存可观测性（v1.2.4）：三类结果缓存的统一快照
# --------------------------------------------------------------------------- #


def cache_overview() -> List[Dict[str, Any]]:
    """三类结果缓存的统一快照（GUI「缓存状态」对话框与测试共用）。

    每行 = ``{"kind": ..., "max_entries": ..., **cache.stats()}``；
    ``kind`` 取 ``"plan"``（规划与分段）/ ``"collision"``（干涉检查）/
    ``"dynamics"``（力矩校验）。

    **全局关闭（``None``）的缓存也在列**——以 ``enabled=False`` 如实呈现
    "关着"，而不是从面板上消失：看不见的关闭最容易被当成"没这回事"。
    """
    rows: List[Dict[str, Any]] = []

    traj = get_cache()
    if traj is None:
        rows.append({"kind": "plan", "enabled": False})
    else:
        row = {"kind": "plan", "max_entries": traj.plan_max_entries, **traj.stats()}
        row["plan_max_waypoints"] = traj.plan_max_waypoints
        row["segment_max_waypoints"] = traj.segment_max_waypoints
        rows.append(row)

    for kind, getter in (
        ("collision", get_collision_cache),
        ("dynamics", get_dynamics_cache),
    ):
        cache = getter()
        if cache is None:
            rows.append({"kind": kind, "enabled": False})
            continue
        row = {"kind": kind, "max_entries": cache.max_entries, **cache.stats()}
        lookups = row["hits"] + row["misses"]
        row["hit_rate"] = row["hits"] / lookups if lookups else 0.0
        rows.append(row)

    return rows


def reset_all_cache_stats() -> None:
    """三类缓存（规划与分段 / 干涉检查 / 力矩校验）的**计数器**归零。

    条目保留——清掉的是"这一轮干了多少活"的账，不是缓存本身。
    """
    for cache in (get_cache(), get_collision_cache(), get_dynamics_cache()):
        if cache is not None:
            cache.reset_stats()


def clear_all_caches() -> None:
    """清空三类缓存的全部条目（计数保留）。

    适用场景：改了一大圈模型/程序之后想要"强制全量重算"——内容寻址
    本来就不该给出过期结果，但这不是性能口径下"想看真实耗时"时的替代品
    （那应该临时关缓存，见 ``orolp.perf``）。
    """
    for cache in (get_cache(), get_collision_cache(), get_dynamics_cache()):
        if cache is not None:
            cache.clear()
