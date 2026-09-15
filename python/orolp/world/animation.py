"""程序级抓取动画（S3）：夹爪指令 → 工件吸附/释放事件。

把三层连起来：

* :class:`~orolp.program.Program` 里的 ``gripper()`` / ``setDO()`` 指令
  （S2 引入，定义了"什么时候抓、什么时候放"）
* :class:`~orolp.planner.TrajectoryPlanner` 展开出的轨迹时间轴
  （每条运动指令占一个 ``(t_start, t_end)`` 区间）
* :class:`~orolp.world.scene.Scene` 的父子挂载机制
  （``attach`` 到机器人下＝工件随法兰/TCP 随动；``detach``＝原地释放）

时间语义
--------
IO 指令是瞬时动作，发生在「上一条运动结束、下一条运动开始」的边界时刻：
程序 ``moveJ → gripper("close") → moveJ`` 里，夹紧事件落在第一条 moveJ
的终点时刻。回放时一旦播放头越过该时刻，工件就应吸附到法兰上随动。

典型用法::

    planner = TrajectoryPlanner(robot, scene=scene)
    traj = planner.plan(program)
    animator = GraspAnimator(program, planner)
    # 回放循环里：
    animator.sync(scene, "工件1", t_current)   # 越过事件即挂载/释放

单元约定：时间秒，与轨迹时间戳一致；位姿毫米，与 :mod:`orolp.world` 一致。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

from ..program import Motion, Program, SetDO
from .scene import ROBOT_NAME, Scene

__all__ = ["GripperEvent", "gripper_events", "GraspAnimator"]


@dataclass(frozen=True)
class GripperEvent:
    """一次夹爪开/合事件。

    Attributes:
        time: 事件时刻（秒，相对轨迹起点）
        action: ``"close"``（夹紧→吸附工件）或 ``"open"``（松开→释放工件）
    """

    time: float
    action: str


def gripper_events(program: Program, planner) -> List[GripperEvent]:
    """把程序里的夹爪 IO 指令映射到轨迹时间轴。

    遍历 ``program.items``（运动与 IO 混排），遇到 ``gripper_close`` 置位
    记一次 ``"close"``、``gripper_open`` 置位记一次 ``"open"``；事件时刻取
    「已完成运动」在轨迹上的结束时间（规划器为每条运动记录了
    ``(t_start, t_end)`` 区间，与 ``program.motions`` 索引一一对齐）。

    Args:
        program: 含 IO 指令的 :class:`~orolp.program.Program`
        planner: 已对该程序执行过 :meth:`~orolp.planner.TrajectoryPlanner.plan`
            的规划器（提供指令 → 时间区间的映射）

    Returns:
        按时间升序的 :class:`GripperEvent` 列表；连续重复的动作只保留首次
    """
    spans = list(getattr(planner, "_motion_spans", []))
    events: List[GripperEvent] = []
    motion_idx = -1  # 尚未经过任何运动指令

    for item in program.items:
        if isinstance(item, Motion):
            motion_idx += 1
            continue
        if not isinstance(item, SetDO):
            continue
        if item.signal == "gripper_close" and item.value:
            action = "close"
        elif item.signal == "gripper_open" and item.value:
            action = "open"
        else:
            # 复位对侧信号（gripper() 展开的互锁对）不构成事件
            continue
        if events and events[-1].action == action:
            continue  # 连续同动作去抖
        # 事件发生在上一条运动结束的边界；还没有任何运动时就是轨迹起点
        t = float(spans[motion_idx][1]) if 0 <= motion_idx < len(spans) else 0.0
        events.append(GripperEvent(time=t, action=action))

    return events


class GraspAnimator:
    """按轨迹时间驱动场景工件吸附/释放的无头动画器。

    不依赖 Qt，GUI（app.py）与离线脚本共用同一份抓取语义；回放器只需在
    每帧调用 :meth:`sync`，越过事件时刻即自动挂载/释放。

    Args:
        program: 含 ``gripper()`` 指令的程序；与 ``planner`` 成对给出
        planner: 已 plan 过该程序的规划器
        events: 直接给事件序列（跳过映射，便于测试/自定义时序）
    """

    def __init__(
        self,
        program: Optional[Program] = None,
        planner=None,
        events: Optional[Sequence[GripperEvent]] = None,
    ):
        if events is None:
            events = (
                gripper_events(program, planner)
                if program is not None and planner is not None
                else []
            )
        self._events: List[GripperEvent] = sorted(events, key=lambda e: e.time)
        #: 已处理（应用于场景）的事件数
        self.cursor = 0

    # ---------------------------- 查询 ---------------------------- #

    @property
    def events(self) -> List[GripperEvent]:
        """全部事件（按时间升序）。"""
        return list(self._events)

    @property
    def done(self) -> bool:
        """是否所有事件都已处理。"""
        return self.cursor >= len(self._events)

    def state_at(self, t: float) -> Optional[str]:
        """``t`` 时刻夹爪应处的状态。

        Returns:
            ``"close"`` / ``"open"``；事件发生前为 ``None``
        """
        state: Optional[str] = None
        for ev in self._events:
            if ev.time <= t:
                state = ev.action
            else:
                break
        return state

    # ---------------------------- 驱动 ---------------------------- #

    def reset(self) -> None:
        """回到播放起点（重新回放时调用）。"""
        self.cursor = 0

    def sync(self, scene: Scene, workpiece: str, t: float) -> Optional[str]:
        """处理 ``t`` 之前的所有事件，驱动工件的吸附/释放。

        Args:
            scene: 工作单元场景
            workpiece: 被抓取的场景对象名
            t: 当前播放头时刻（秒）

        Returns:
            本次实际执行的最后一个动作（``"close"``/``"open"``）；
            没有越过任何事件时为 ``None``
        """
        applied: Optional[str] = None
        while self.cursor < len(self._events) and self._events[self.cursor].time <= t:
            ev = self._events[self.cursor]
            self.cursor += 1
            self.apply(scene, workpiece, ev.action)
            applied = ev.action
        return applied

    # ---------------------------- 场景操作 ---------------------------- #

    @staticmethod
    def apply(scene: Scene, workpiece: str, action: str) -> None:
        """执行单次吸附/释放。

        * ``"close"``：把工件挂到机器人下并**保持世界位姿**——被夹住的瞬间
          停在原地，之后随法兰（TCP）随动
        * ``"open"``：解除挂载，当前世界位姿写回局部 pose，工件原地留下

        Raises:
            KeyError: 工件不存在
            ValueError: 挂载关系非法
        """
        if action == "close":
            scene.attach(workpiece, ROBOT_NAME, keep_world=True)
        elif action == "open":
            scene.detach(workpiece)
        else:
            raise ValueError(f"未知夹爪动作: {action!r}")
