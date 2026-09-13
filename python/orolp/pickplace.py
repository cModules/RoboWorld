"""抓取（pick & place）工艺：示教点语义 → 机器人程序 + 吸附事件。

为什么需要这个模块
------------------
示教出来的点位本身只是一串关节值，机器不知道哪个点是"去抓"、哪个点是"去放"。
本模块给每个示教点打上**工艺语义**（:class:`PointType`），据此：

1. 在正确的位置插入 IO 指令（夹紧 / 松开 / 等待夹紧完成 / 保压）
2. 产出一串**吸附事件**，供播放器在对应时刻把工件挂到法兰上或放回世界

生成的 :attr:`PickPlacePlan.items` 可直接喂给
:class:`~orolp.program.Program`，于是机器人程序与仿真动画共用**同一份**定义，
不会出现"程序里有抓取、仿真里没动作"这类脱节。

信号一律用 :data:`~orolp.cell.HANDSHAKE_SIGNALS` 里的**逻辑名**，物理地址
留到后处理期按 :class:`~orolp.cell.Device.io_map` 解析。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Sequence, Tuple

from .program import Motion, MotionType, SetDO, WaitDI, WaitTime

__all__ = [
    "PointType",
    "DEFAULT_SEQUENCE",
    "PickPlacePlan",
    "build_pick_place",
    "default_types_for_count",
    "gripper_action_for",
]


class PointType(Enum):
    """示教点的工艺语义。"""

    APPROACH = "approach"   #: 接近点：位于抓取位正上方，安全进入
    PICK = "pick"           #: 抓取点：夹爪闭合，工件吸附到法兰
    LIFT = "lift"           #: 提升点：带着工件抬到安全高度
    PLACE = "place"         #: 放置点：夹爪张开，工件释放到世界
    RETRACT = "retract"     #: 撤离点：空爪退到安全位
    VIA = "via"             #: 普通途经点，无工艺动作

    @property
    def label(self) -> str:
        """中文名，用于界面展示。"""
        return _LABELS[self]


_LABELS: Dict[PointType, str] = {
    PointType.APPROACH: "接近点",
    PointType.PICK: "抓取点",
    PointType.LIFT: "提升点",
    PointType.PLACE: "放置点",
    PointType.RETRACT: "撤离点",
    PointType.VIA: "途经点",
}

#: 五点式抓取循环的推荐顺序
DEFAULT_SEQUENCE: Tuple[PointType, ...] = (
    PointType.APPROACH,
    PointType.PICK,
    PointType.LIFT,
    PointType.PLACE,
    PointType.RETRACT,
)


def gripper_action_for(point_type: Optional[PointType]) -> Optional[str]:
    """示教点工艺语义 → 夹爪动作（``"close"/"open"``）。

    抓取点闭合、放置点张开，其余语义（接近/提升/撤离/途经/无）不动夹爪。
    程序构建（回放动画 + 后处理出码）与事件映射共用这一份对应关系，
    避免两处各写一份映射后悄悄不一致。

    Args:
        point_type: 示教点语义；``None`` 表示无语义

    Returns:
        夹爪动作；该点不需要动夹爪时为 ``None``
    """
    if point_type is PointType.PICK:
        return "close"
    if point_type is PointType.PLACE:
        return "open"
    return None


def default_types_for_count(n: int) -> List[PointType]:
    """按示教点数量给出默认语义。

    5 个点时正好是标准五点式循环；多于 5 个时，首尾保持接近/撤离，中间多出来的
    都算途经点——宁可让用户在界面上改，也不要瞎猜哪个是抓取点。

    Args:
        n: 示教点数量

    Returns:
        长度为 ``n`` 的语义列表
    """
    if n <= 0:
        return []
    if n == len(DEFAULT_SEQUENCE):
        return list(DEFAULT_SEQUENCE)
    if n < len(DEFAULT_SEQUENCE):
        return list(DEFAULT_SEQUENCE[:n])
    # 多于 5 个：首尾固定，中间补途经点
    return (
        [PointType.APPROACH]
        + [PointType.VIA] * (n - 2)
        + [PointType.RETRACT]
    )


@dataclass
class PickPlacePlan:
    """抓取工艺计划。

    Attributes:
        items: 指令序列（运动 + IO 混排），可直接交给
            :class:`~orolp.program.Program`
        events: ``(示教点索引, 动作)`` 列表，动作为 ``"pick"`` 或 ``"place"``。
            播放器在走到该示教点时据此挂载 / 释放工件
    """

    items: List[object] = field(default_factory=list)
    events: List[Tuple[int, str]] = field(default_factory=list)

    @property
    def pick_indices(self) -> List[int]:
        """需要吸附工件的示教点索引。"""
        return [i for i, a in self.events if a == "pick"]

    @property
    def place_indices(self) -> List[int]:
        """需要释放工件的示教点索引。"""
        return [i for i, a in self.events if a == "place"]


def build_pick_place(
    points_deg: Sequence[Sequence[float]],
    types: Optional[Sequence[PointType]] = None,
    *,
    close_signal: str = "gripper_close",
    open_signal: str = "gripper_open",
    clamped_signal: str = "clamped",
    dwell: float = 0.4,
    close_wait: float = 2.0,
    speed: float = 100.0,
) -> PickPlacePlan:
    """由示教点生成抓取程序与吸附事件。

    Args:
        points_deg: 示教点关节值（**度**），每项长度等于机器人自由度
        types: 每个点的工艺语义；``None`` 时用 :func:`default_types_for_count`
        close_signal: 夹紧输出信号（逻辑名）
        open_signal: 松开输出信号（逻辑名）
        clamped_signal: 夹紧完成反馈信号（逻辑名）
        dwell: 夹紧/松开后的保压时间（秒）
        close_wait: 等待夹紧完成的超时（秒）；``<= 0`` 表示无限等待
        speed: 运动速度百分比

    Returns:
        :class:`PickPlacePlan`

    Raises:
        ValueError: 示教点为空，或 ``types`` 长度与示教点不一致
    """
    pts = [list(map(float, q)) for q in points_deg]
    if not pts:
        raise ValueError("没有示教点，无法生成抓取程序")

    tps = list(types) if types is not None else default_types_for_count(len(pts))
    if len(tps) != len(pts):
        raise ValueError(
            f"语义数量({len(tps)})与示教点数量({len(pts)})不一致"
        )

    items: List[object] = []
    events: List[Tuple[int, str]] = []

    for idx, (q, t) in enumerate(zip(pts, tps)):
        items.append(Motion(MotionType.JOINTS, list(q), speed=speed))

        if t is PointType.PICK:
            # 先确保松开位复位，再夹紧——两个信号互锁，避免同时为真
            items.append(SetDO(open_signal, False))
            items.append(SetDO(close_signal, True))
            items.append(WaitDI(clamped_signal, True, timeout=close_wait))
            items.append(WaitTime(dwell))
            events.append((idx, "pick"))

        elif t is PointType.PLACE:
            items.append(SetDO(close_signal, False))
            items.append(SetDO(open_signal, True))
            items.append(WaitTime(dwell))
            events.append((idx, "place"))

    return PickPlacePlan(items=items, events=list(events))
