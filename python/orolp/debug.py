"""在线调试会话：把规划好的轨迹在控制器（真实或仿真）上跑起来并做比对。

离线编程生成、部署之后，现场最常被问的两个问题是：

* "机器人实际走到哪儿了？" —— 读回控制器里的实际关节角 / TCP 位姿；
* "和我在软件里编的点对得上吗？" —— 把实际位姿与规划位姿做偏差比对。

:class:`DebugSession` 用一个 :class:`~orolp.simulation.Trajectory`（规划结果）
加一个 :class:`~orolp.link.ControllerLink`（控制器）把这两件事串起来：

* :meth:`step` / :meth:`run` —— 单步或连续播放轨迹，逐点把规划关节角下发给
  控制器；
* :meth:`read_actual` / :meth:`deviation` —— 读回实际位姿，与规划位姿算偏差
  （位置 mm + 姿态 deg）；
* :meth:`goto_waypoint` / :meth:`goto_time` —— 跳到任意路点 / 时刻（排查某一段）；
* :meth:`jog` —— 手动微调某个关节（示教 / 对中）。

当连接具备实际回读能力（:class:`~orolp.link.SimLink` 或真实控制器）时，偏差
报告才有意义；离线导出（:class:`~orolp.link.FileLink`）下会话仍可播放，但
``feedback_available`` 为 ``False``，偏差比对会明确标注"无回读"。
"""

from __future__ import annotations

import inspect
from typing import Any, Dict, List, Optional

import numpy as np

from .simulation import Waypoint

__all__ = ["DebugSession", "Deviation"]


def _accepts_kwarg(func: Any, name: str) -> bool:
    """判断 ``func`` 是否接受名为 ``name`` 的关键字参数（用于能力探测）。

    连接类型是开放的（``ControllerLink`` 家族之外还可能有合成真机、真机 SDK 包装
    等），所以用签名探测而不是硬编码类型列表；探测失败一律当"不接受"，退回老调用
    方式——宁可不传时间，也不能因为探测把调试链路搞崩。
    """
    if func is None:
        return False
    try:
        sig = inspect.signature(func)
    except (TypeError, ValueError):
        return False
    for p in sig.parameters.values():
        if p.name == name:
            return True
        if p.kind is inspect.Parameter.VAR_KEYWORD:
            return True
    return False


class Deviation:
    """规划位姿与实际位姿的偏差。

    Attributes:
        available: 是否具备回读（无回读时为 ``False``，数值无意义）
        pos_mm: 位置偏差（mm）
        rot_deg: 姿态偏差（度）
        waypoint_index: 比对发生在第几个路点
    """

    def __init__(
        self,
        available: bool,
        pos_mm: float = 0.0,
        rot_deg: float = 0.0,
        waypoint_index: int = -1,
    ):
        self.available = available
        self.pos_mm = pos_mm
        self.rot_deg = rot_deg
        self.waypoint_index = waypoint_index

    def __repr__(self) -> str:
        if not self.available:
            return "Deviation(无回读)"
        return (
            f"Deviation(路点 {self.waypoint_index}: "
            f"位置 {self.pos_mm:.3f} mm, 姿态 {self.rot_deg:.3f}°)"
        )


class DebugSession:
    """在控制器上播放规划轨迹并做实际/规划偏差比对的调试会话。"""

    def __init__(self, trajectory: "object", link: "object", robot: Any = None):
        """
        Args:
            trajectory: 规划得到的 :class:`~orolp.simulation.Trajectory`
            link: 控制器连接（:class:`~orolp.link.ControllerLink`）
            robot: 可选机器人实例；当连接不提供 ``get_pose`` 时用于正解实际位姿
        """
        self.traj = trajectory
        self.link = link
        self._robot = robot
        self.feedback_available = bool(getattr(link, "is_actual_feedback", False))
        # 有些连接（如合成真机 PlantLink）需要知道"这一刻是几点"才能建模伺服滞后，
        # 因此下发时把时间戳一并给它；不接受的连接（FileLink/SimLink/真机驱动）
        # 保持老签名不变，行为与以前逐字节一致。
        self._link_takes_time = _accepts_kwarg(getattr(link, "set_joints", None), "at_time")
        self._idx = 0
        self._t = trajectory.waypoints[0].timestamp if trajectory.waypoints else 0.0
        self._executed = 0
        self._max_pos_dev = 0.0
        self._max_rot_dev = 0.0

    # --------------------------- 播放 --------------------------- #

    def _command(self, q: np.ndarray, t: float) -> None:
        """向连接下发关节角，并把时刻一并传给需要它的连接（如合成真机）。

        老连接（``FileLink`` / ``SimLink`` / 真机驱动）的 ``set_joints(joints)`` 签名
        不变，行为与以前完全一致。
        """
        if self._link_takes_time:
            self.link.set_joints(q, at_time=t)
        else:
            self.link.set_joints(q)

    def reset(self) -> None:
        """回到轨迹起点。"""
        self._idx = 0
        self._t = self.traj.waypoints[0].timestamp if self.traj.waypoints else 0.0
        self._executed = 0
        self._max_pos_dev = 0.0
        self._max_rot_dev = 0.0

    @property
    def current_index(self) -> int:
        """当前所在路点序号（从 0 计）。"""
        return self._idx

    @property
    def current_time(self) -> float:
        """当前时刻（秒）。"""
        return self._t

    def step(self) -> Optional[Waypoint]:
        """单步：把下一路径点下发给控制器。

        Returns:
            刚执行的 :class:`~orolp.simulation.Waypoint`；已到末尾则返回 ``None``
        """
        wps = self.traj.waypoints
        if self._idx >= len(wps) - 1:
            return None
        self._idx += 1
        wp = wps[self._idx]
        self._t = float(wp.timestamp)
        self._command(np.asarray(wp.joints, dtype=np.float64), self._t)
        self._executed += 1
        self._record_deviation()
        return wp

    def run(self, sample_dt: float = 0.02) -> Dict[str, Any]:
        """连续播放：从当前位置一路跑到轨迹末尾。

        Args:
            sample_dt: 播放步进（秒），仅用于日志/节流；实际路点逐一下发

        Returns:
            包含最大偏差与执行统计的摘要字典
        """
        while self.step() is not None:
            pass
        return self.summary()

    def goto_waypoint(self, index: int) -> Waypoint:
        """跳到指定路点（不发中间点，用于排查某一段）。"""
        wps = self.traj.waypoints
        i = int(np.clip(index, 0, len(wps) - 1))
        self._idx = i
        wp = wps[i]
        self._t = float(wp.timestamp)
        self._command(np.asarray(wp.joints, dtype=np.float64), self._t)
        self._executed += 1
        self._record_deviation()
        return wp

    def goto_time(self, t: float) -> Waypoint:
        """跳到指定时刻对应的路点（按时间戳就近选取）。"""
        wps = self.traj.waypoints
        if not wps:
            raise ValueError("轨迹为空")
        best = 0
        best_d = abs(wps[0].timestamp - t)
        for k, wp in enumerate(wps):
            d = abs(wp.timestamp - t)
            if d < best_d:
                best_d = d
                best = k
        return self.goto_waypoint(best)

    # --------------------------- 回读与比对 --------------------------- #

    def planned_waypoint(self) -> Waypoint:
        """当前路点的规划值。"""
        return self.traj.waypoints[self._idx]

    def _actual_pose(self, q: np.ndarray):
        """由实际关节角求出 TCP 位姿（优先用连接的 get_pose，其次机器人正解）。"""
        if hasattr(self.link, "get_pose"):
            try:
                return self.link.get_pose()
            except Exception:
                pass
        if self._robot is not None:
            return self._robot.forward_kinematics(q)
        return None

    def read_actual(self) -> Optional[Waypoint]:
        """读回当前实际位姿（含关节角与 TCP 位姿）。

        无回读能力的连接返回 ``None``。
        """
        if not self.feedback_available:
            return None
        q = self.link.read_joints()
        if q is None:
            return None
        pose = self._actual_pose(q)
        return Waypoint(joints=np.asarray(q, dtype=np.float64), pose=pose, timestamp=self._t)

    def deviation(self) -> Deviation:
        """当前路点：规划位姿 vs 实际位姿的偏差。"""
        if not self.feedback_available:
            return Deviation(available=False, waypoint_index=self._idx)
        actual = self.read_actual()
        if actual is None or actual.pose is None:
            return Deviation(available=False, waypoint_index=self._idx)
        planned = self.planned_waypoint().pose
        pos_mm = float(np.linalg.norm(
            np.array(actual.pose.to_xyz_rpy()[:3], dtype=np.float64)
            - np.array(planned.to_xyz_rpy()[:3], dtype=np.float64)
        ))
        rot_deg = _orientation_angle_deg(planned, actual.pose)
        return Deviation(available=True, pos_mm=pos_mm, rot_deg=rot_deg, waypoint_index=self._idx)

    def _record_deviation(self) -> None:
        dev = self.deviation()
        if dev.available:
            self._max_pos_dev = max(self._max_pos_dev, dev.pos_mm)
            self._max_rot_dev = max(self._max_rot_dev, dev.rot_deg)

    # --------------------------- 手动示教 --------------------------- #

    def jog(self, joint_index: int, delta_deg: float) -> Optional[np.ndarray]:
        """手动微调某个关节（示教 / 对中）。

        Args:
            joint_index: 关节序号（从 0 计）
            delta_deg: 增量（度）

        Returns:
            调整后的实际关节角；无回读能力时返回 ``None``
        """
        if not self.feedback_available:
            return None
        q = self.link.read_joints()
        if q is None:
            return None
        q = q.copy()
        q[joint_index] = q[joint_index] + float(delta_deg)
        self.link.set_joints(q)
        return q

    # --------------------------- 摘要 --------------------------- #

    def summary(self) -> Dict[str, Any]:
        """返回会话统计摘要。"""
        return {
            "feedback_available": self.feedback_available,
            "num_waypoints": len(self.traj.waypoints),
            "executed": self._executed,
            "current_index": self._idx,
            "current_time": self._t,
            "max_pos_deviation_mm": self._max_pos_dev,
            "max_rot_deviation_deg": self._max_rot_dev,
        }


def _orientation_angle_deg(p0, p1) -> float:
    """两个位姿之间的姿态夹角（度）。"""
    from scipy.spatial.transform import Rotation

    R0 = p0.matrix[:3, :3]
    R1 = p1.matrix[:3, :3]
    R_err = R1 @ R0.T
    c = float(np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0))
    return float(np.degrees(np.arccos(c)))
