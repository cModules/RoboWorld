"""数字孪生与云协作（v1.0.3）：把"仿真算出来的"和"真机上跑出来的"放到同一条时间轴上。

离线编程真正的闭环不是"生成代码"，而是：**仿真里规划 → 传到控制器 → 真机跑 → 回读
实际轨迹 → 和仿真对差**。[`service.debug`][service.debug] 已经能回答"此刻差多少"，但它
回答不了工程上更关心的那类问题：

* 整条轨迹上偏差**长什么样**？是全程均匀 0.3mm，还是某个拐角突然 4mm？
* 偏差**从哪来**？——零位偏置是常量偏移，传动比误差随关节角线性放大，跟踪滞后
  在速度突变处最明显，三者形状完全不同，看曲线就能分辨；
* 现场那台机器和仿真**是不是一回事**？换个批次、重标定之后偏差有没有变小？

这一层把 :class:`~service.debug.DebugSession` 的**逐时刻偏差**采样成时间序列
（:class:`TwinTrace`），再把两条来自不同源的轨迹按时间对齐做差分
(:func:`compare_traces` / :class:`TwinReport`)，最后给出曲线与报表。

模块构成
--------

* :class:`TwinSnapshot` / :class:`TwinTrace` —— 一条轨迹的偏差时间序列，支持 JSON /
  JSONL 往返（JSONL 便于云端按行增量上报与回放）。
* :class:`PlantModel` / :class:`PlantLink` —— **无真机时的可复现"真机"**。真机偏差
  不是噪声，是有物理形状的：关节零位偏置、减速比标称误差、编码器量化、伺服跟踪
  滞后。把它们写成显式模型，既能在没有机器的机器上验证整条孪生链路，也能作为
  故障注入用来测"偏差多大时该报警"。**这是合成模型，不是真机数据**——报告里会
  一律标注 ``source="plant"``，不要把它的数字当现场实测引用。
* :func:`compare_traces` / :class:`TwinReport` / :func:`deviation_chart_svg` —— 跨源
  偏差统计与可视化（真机 vs 仿真）。
* :class:`CloudTransport` / :class:`LocalCloudStore` / :class:`CloudLink` —— 落地
  :class:`~domain.link.ControllerLink` 一直预留的"云"通道：程序与孪生轨迹上传、
  真机状态上报与**新鲜度**回读。传输层是抽象接口，默认文件式存储（离线可用、
  零依赖），换成 HTTP/S3 只需实现 :class:`CloudTransport`。

关于"实时"的诚实说明
--------------------

:meth:`CloudLink.status_age` / :meth:`CloudLink.is_fresh` 存在的意义就是**不要让
"实时回显"变成一句营销话术**：回显的是"云端最后一条上报"，它有多旧必须能查。超
过 ``max_age`` 就被判为陈旧，孪生视图应显示"数据过期"而不是拿旧姿势当真机现状。
"""

from __future__ import annotations

import inspect
import json
import math
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from domain.link import ControllerLink, SimLink

__all__ = [
    "TwinError",
    "TwinSnapshot",
    "TwinTrace",
    "AlignedSample",
    "TwinReport",
    "PlantModel",
    "PlantLink",
    "capture_from_session",
    "capture_plant_trace",
    "pose_deviation",
    "align_traces",
    "compare_traces",
    "deviation_chart_svg",
    "CloudTransport",
    "MemoryCloudStore",
    "LocalCloudStore",
    "CloudLink",
    "TRACE_HEADER_KIND",
    "TRACE_SAMPLE_KIND",
]


#: JSONL 首行的 ``kind`` 标记（元数据行）
TRACE_HEADER_KIND = "robotworld.twin.trace/1"
#: JSONL 采样行的 ``kind`` 标记
TRACE_SAMPLE_KIND = "sample"


class TwinError(RuntimeError):
    """数字孪生 / 云协作相关的错误（轨迹缺失、传输 key 非法、缺运动学支撑等）。"""


# --------------------------------------------------------------------------- #
# 位姿偏差
# --------------------------------------------------------------------------- #


def pose_deviation(p0: Any, p1: Any) -> Tuple[float, float]:
    """两个 TCP 位姿之间的偏差 ``(位置 mm, 姿态 deg)``。

    姿态差用旋转矩阵的迹求夹角，因此与绕轴顺序无关。

    Args:
        p0: 参考位姿（:class:`~domain.pose.Pose`）
        p1: 实测位姿

    Returns:
        ``(位置偏差 mm, 姿态夹角 deg)``
    """
    a = np.asarray(p0.to_xyz_rpy()[:3], dtype=np.float64)
    b = np.asarray(p1.to_xyz_rpy()[:3], dtype=np.float64)
    pos_mm = float(np.linalg.norm(b - a))

    r0 = np.asarray(p0.matrix, dtype=np.float64)[:3, :3]
    r1 = np.asarray(p1.matrix, dtype=np.float64)[:3, :3]
    r_err = r1 @ r0.T
    cos_a = float(np.clip((np.trace(r_err) - 1.0) / 2.0, -1.0, 1.0))
    rot_deg = float(np.degrees(math.acos(cos_a)))
    return pos_mm, rot_deg


# --------------------------------------------------------------------------- #
# 偏差时间序列
# --------------------------------------------------------------------------- #


@dataclass
class TwinSnapshot:
    """某一时刻的孪生快照：规划值、实测值、以及两者的差。

    Attributes:
        t: 时刻（秒，相对轨迹起点）
        planned_q: 规划关节角（deg）
        actual_q: 实测关节角（deg）；无回读时为 ``None``
        planned_pose: 规划 TCP 位姿
        actual_pose: 实测 TCP 位姿
        pos_dev_mm: TCP 位置偏差（mm）；无回读时为 ``nan``
        rot_dev_deg: TCP 姿态偏差（deg）；无回读时为 ``nan``
        joint_dev_deg: 各轴最大关节角偏差（deg）；无回读时为 ``nan``
    """

    t: float
    planned_q: np.ndarray
    actual_q: Optional[np.ndarray] = None
    planned_pose: Any = None
    actual_pose: Any = None
    pos_dev_mm: float = float("nan")
    rot_dev_deg: float = float("nan")
    joint_dev_deg: float = float("nan")

    @property
    def available(self) -> bool:
        """是否有实测数据（决定偏差数值是否有意义）。"""
        return self.actual_q is not None

    def to_dict(self, with_poses: bool = True) -> Dict[str, Any]:
        """转成可 JSON 序列化的字典（位姿展开成 6 个数字，避免嵌套对象）。"""
        d: Dict[str, Any] = {
            "kind": TRACE_SAMPLE_KIND,
            "t": float(self.t),
            "planned_q": [float(v) for v in np.ravel(self.planned_q)],
            "actual_q": (
                [float(v) for v in np.ravel(self.actual_q)] if self.actual_q is not None else None
            ),
        }
        if with_poses:
            d["planned_pose"] = (
                [float(v) for v in self.planned_pose.to_xyz_rpy()]
                if self.planned_pose is not None
                else None
            )
            d["actual_pose"] = (
                [float(v) for v in self.actual_pose.to_xyz_rpy()]
                if self.actual_pose is not None
                else None
            )
        d["pos_dev_mm"] = None if math.isnan(self.pos_dev_mm) else float(self.pos_dev_mm)
        d["rot_dev_deg"] = None if math.isnan(self.rot_dev_deg) else float(self.rot_dev_deg)
        d["joint_dev_deg"] = None if math.isnan(self.joint_dev_deg) else float(self.joint_dev_deg)
        return d

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "TwinSnapshot":
        """从 :meth:`to_dict` 的结果还原。"""
        from domain.pose import Pose

        def _pose(seq) -> Any:
            if not seq:
                return None
            return Pose.from_xyz_rpy(*[float(v) for v in seq])

        def _dev(v) -> float:
            return float("nan") if v is None else float(v)

        return cls(
            t=float(d["t"]),
            planned_q=np.asarray(d.get("planned_q") or [], dtype=np.float64),
            actual_q=(
                None if d.get("actual_q") is None else np.asarray(d["actual_q"], dtype=np.float64)
            ),
            planned_pose=_pose(d.get("planned_pose")),
            actual_pose=_pose(d.get("actual_pose")),
            pos_dev_mm=_dev(d.get("pos_dev_mm")),
            rot_dev_deg=_dev(d.get("rot_dev_deg")),
            joint_dev_deg=_dev(d.get("joint_dev_deg")),
        )


@dataclass
class TwinTrace:
    """一条轨迹的孪生快照序列（规划 vs 实测）。

    Attributes:
        snapshots: 按时间升序的快照列表
        source: 数据来源标签——``"sim"`` 纯仿真回读、``"plant"`` 合成真机模型、
            ``"real"`` 现场真机。**存储位置不改变来源**：从云端取回的真机轨迹仍标
            ``"real"``（"是不是真机测的"比"存在哪"重要得多），"经过云端"这件事记在
            :attr:`notes` 里。
        robot_name: 机器人标识
        program_name: 程序名
        created_at: 生成时刻（Unix 秒）
        notes: 附注（例如"合成模型，非真机实测"）
    """

    snapshots: List[TwinSnapshot] = field(default_factory=list)
    source: str = "sim"
    robot_name: str = ""
    program_name: str = ""
    created_at: float = field(default_factory=time.time)
    notes: str = ""

    # ------------------------------ 基本信息 ------------------------------ #

    def __len__(self) -> int:
        return len(self.snapshots)

    def __iter__(self):
        return iter(self.snapshots)

    def __getitem__(self, index):
        return self.snapshots[index]

    @property
    def duration(self) -> float:
        """轨迹时长（秒），空轨迹为 0。"""
        if not self.snapshots:
            return 0.0
        return float(self.snapshots[-1].t - self.snapshots[0].t)

    @property
    def feedback_available(self) -> bool:
        """是否含实测数据（否则只剩规划值，偏差无意义）。"""
        return any(s.available for s in self.snapshots)

    @property
    def times(self) -> np.ndarray:
        """所有时刻组成的数组（秒）。"""
        return np.asarray([s.t for s in self.snapshots], dtype=np.float64)

    def pos_dev_series(self) -> np.ndarray:
        """TCP 位置偏差序列（mm）；无实测处为 ``nan``。"""
        return np.asarray([s.pos_dev_mm for s in self.snapshots], dtype=np.float64)

    def rot_dev_series(self) -> np.ndarray:
        """TCP 姿态偏差序列（deg）。"""
        return np.asarray([s.rot_dev_deg for s in self.snapshots], dtype=np.float64)

    def joint_dev_series(self) -> np.ndarray:
        """各轴最大关节角偏差序列（deg）。"""
        return np.asarray([s.joint_dev_deg for s in self.snapshots], dtype=np.float64)

    def joint_speed_series(self) -> np.ndarray:
        """各快照处的**规划**关节速度（deg/s，取各轴最大绝对速度）。

        用它把偏差分成"低速段"和"高速段"：低速/停点处还剩下的偏差只可能来自
        **标定**（零位、传动比、连杆尺寸），高速处多出来的那部分才是**伺服跟踪**
        能力。同一个 12mm，这两类的处理方式完全不同。
        """
        n = len(self.snapshots)
        speeds = np.zeros(n, dtype=np.float64)
        if n < 2:
            return speeds
        for i in range(n):
            j = i - 1 if i == n - 1 else i
            k = j + 1
            dt = float(self.snapshots[k].t - self.snapshots[j].t)
            if dt <= 0:
                continue
            dq = np.asarray(self.snapshots[k].planned_q, dtype=np.float64) - np.asarray(
                self.snapshots[j].planned_q, dtype=np.float64
            )
            speeds[i] = float(np.max(np.abs(dq))) / dt
        return speeds

    def max_pos_dev(self) -> float:
        """位置偏差峰值（mm）；无实测时返回 ``nan``。"""
        data = self.pos_dev_series()
        data = data[~np.isnan(data)]
        return float(np.max(data)) if data.size else float("nan")

    def max_rot_dev(self) -> float:
        """姿态偏差峰值（deg）；无实测时返回 ``nan``。"""
        data = self.rot_dev_series()
        data = data[~np.isnan(data)]
        return float(np.max(data)) if data.size else float("nan")

    def worst_index(self) -> int:
        """位置偏差最大的快照序号（无实测返回 ``-1``）。"""
        data = self.pos_dev_series()
        if not np.any(~np.isnan(data)):
            return -1
        return int(np.nanargmax(data))

    def tolerances_exceeded(self, tol_pos_mm: float = 1.0, tol_rot_deg: float = 1.0) -> List[int]:
        """超出容差的快照序号列表（无实测的点不计）。"""
        out: List[int] = []
        for i, s in enumerate(self.snapshots):
            if not s.available:
                continue
            if (
                not math.isnan(s.pos_dev_mm)
                and s.pos_dev_mm > tol_pos_mm
                or not math.isnan(s.rot_dev_deg)
                and s.rot_dev_deg > tol_rot_deg
            ):
                out.append(i)
        return out

    def stats(self) -> Dict[str, Any]:
        """统计摘要（峰值 / 均值 / RMS / 最差点）。"""
        pos = self.pos_dev_series()
        rot = self.rot_dev_series()
        joint = self.joint_dev_series()
        valid_pos = pos[~np.isnan(pos)]
        valid_rot = rot[~np.isnan(rot)]
        valid_joint = joint[~np.isnan(joint)]
        wi = self.worst_index()
        return {
            "source": self.source,
            "samples": len(self.snapshots),
            "feedback_available": self.feedback_available,
            "duration_s": self.duration,
            "max_pos_mm": float(valid_pos.max()) if valid_pos.size else float("nan"),
            "mean_pos_mm": float(valid_pos.mean()) if valid_pos.size else float("nan"),
            "rms_pos_mm": (
                float(np.sqrt(np.mean(valid_pos**2))) if valid_pos.size else float("nan")
            ),
            "max_rot_deg": float(valid_rot.max()) if valid_rot.size else float("nan"),
            "max_joint_deg": (float(valid_joint.max()) if valid_joint.size else float("nan")),
            "worst_index": wi,
            "worst_t": float(self.snapshots[wi].t) if wi >= 0 else float("nan"),
        }

    # ------------------------------ 序列化 ------------------------------ #

    def header_dict(self) -> Dict[str, Any]:
        """JSONL 首行（元数据）。"""
        return {
            "kind": TRACE_HEADER_KIND,
            "source": self.source,
            "robot_name": self.robot_name,
            "program_name": self.program_name,
            "created_at": float(self.created_at),
            "samples": len(self.snapshots),
            "notes": self.notes,
        }

    def to_dict(self) -> Dict[str, Any]:
        """转成单个 JSON 文档（含全部快照）。"""
        d = dict(self.header_dict())
        d.pop("kind")
        d["type"] = TRACE_HEADER_KIND
        d["snapshots"] = [s.to_dict() for s in self.snapshots]
        return d

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "TwinTrace":
        """从 :meth:`to_dict` 结果还原。"""
        raw = d.get("snapshots")
        if raw is None:
            raise TwinError("轨迹字典缺少 snapshots 字段")
        return cls(
            snapshots=[TwinSnapshot.from_dict(x) for x in raw],
            source=str(d.get("source", "sim")),
            robot_name=str(d.get("robot_name", "")),
            program_name=str(d.get("program_name", "")),
            created_at=float(d.get("created_at", time.time())),
            notes=str(d.get("notes", "")),
        )

    def to_jsonl(self, with_poses: bool = True) -> str:
        """序列化为 JSONL——首行元数据，其后每行一个快照。

        JSONL 便于云端**按行增量上报**：真机跑一段就能 push 一段，无需等整条轨迹
        结束；也便于事后逐行回放。
        """
        lines = [json.dumps(self.header_dict(), ensure_ascii=False)]
        for s in self.snapshots:
            lines.append(json.dumps(s.to_dict(with_poses=with_poses), ensure_ascii=False))
        return "\n".join(lines) + "\n"

    @classmethod
    def from_jsonl(cls, text: str) -> "TwinTrace":
        """从 JSONL 文本还原。"""
        header: Dict[str, Any] = {}
        snaps: List[TwinSnapshot] = []
        for lineno, raw in enumerate(text.splitlines(), 1):
            raw = raw.strip()
            if not raw:
                continue
            try:
                obj = json.loads(raw)
            except json.JSONDecodeError as exc:  # pragma: no cover - 明确报错分支
                raise TwinError(f"JSONL 第 {lineno} 行解析失败: {exc}") from exc
            kind = obj.get("kind")
            if kind == TRACE_HEADER_KIND:
                header = obj
            elif kind == TRACE_SAMPLE_KIND:
                snaps.append(TwinSnapshot.from_dict(obj))
            else:
                raise TwinError(f"JSONL 第 {lineno} 行 kind 非法: {kind!r}")
        if not header:
            raise TwinError("JSONL 缺少元数据首行")
        return cls(
            snapshots=snaps,
            source=str(header.get("source", "sim")),
            robot_name=str(header.get("robot_name", "")),
            program_name=str(header.get("program_name", "")),
            created_at=float(header.get("created_at", time.time())),
            notes=str(header.get("notes", "")),
        )

    def save(self, path: str | Path, fmt: Optional[str] = None) -> Path:
        """落盘（``.jsonl`` 走流式，其余走单文件 JSON）。"""
        p = Path(path)
        if fmt is None:
            fmt = "jsonl" if p.suffix.lower() == ".jsonl" else "json"
        if fmt == "jsonl":
            p.write_text(self.to_jsonl(), encoding="utf-8")
        elif fmt == "json":
            p.write_text(json.dumps(self.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8")
        else:
            raise TwinError(f"不支持的轨迹格式: {fmt!r}（用 json / jsonl）")
        return p

    @classmethod
    def load(cls, path: str | Path, fmt: Optional[str] = None) -> "TwinTrace":
        """读盘（按后缀自动判断，``.jsonl`` 走流式）。"""
        p = Path(path)
        if fmt is None:
            fmt = "jsonl" if p.suffix.lower() == ".jsonl" else "json"
        text = p.read_text(encoding="utf-8")
        if fmt == "jsonl":
            return cls.from_jsonl(text)
        if fmt == "json":
            return cls.from_dict(json.loads(text))
        raise TwinError(f"不支持的轨迹格式: {fmt!r}（用 json / jsonl）")

    def shifted(self, dt: float) -> "TwinTrace":
        """时间整体平移 ``dt``（秒），返回新轨迹——用于对齐两台不同触发时刻的设备。"""
        snaps = []
        for s in self.snapshots:
            snaps.append(
                TwinSnapshot(
                    t=float(s.t + dt),
                    planned_q=s.planned_q.copy(),
                    actual_q=None if s.actual_q is None else s.actual_q.copy(),
                    planned_pose=s.planned_pose,
                    actual_pose=s.actual_pose,
                    pos_dev_mm=s.pos_dev_mm,
                    rot_dev_deg=s.rot_dev_deg,
                    joint_dev_deg=s.joint_dev_deg,
                )
            )
        return TwinTrace(
            snapshots=snaps,
            source=self.source,
            robot_name=self.robot_name,
            program_name=self.program_name,
            created_at=self.created_at,
            notes=self.notes,
        )


# --------------------------------------------------------------------------- #
# 采集：把 DebugSession 的逐时刻偏差变成时间序列
# --------------------------------------------------------------------------- #


def capture_from_session(
    session: "object",
    source: str = "sim",
    robot_name: str = "",
    program_name: str = "",
    notes: str = "",
) -> TwinTrace:
    """逐路点驱动一个 :class:`~service.debug.DebugSession`，采集孪生轨迹。

    这里刻意**复用** :meth:`DebugSession.deviation` —— 偏差的定义（位置欧氏距离 +
    姿态旋转夹角）只在一处实现，孪生层只负责把它采样成时间序列。

    Args:
        session: :class:`~service.debug.DebugSession` 实例（轨迹 + 控制器连接）
        source: 来源标签（``"sim"`` / ``"plant"`` / ``"real"``）
        robot_name: 机器人标识
        program_name: 程序名
        notes: 附注

    Returns:
        采集到的 :class:`TwinTrace`（快照数与路点数一致，首个路点也在内）
    """
    traj = session.traj
    waypoints = list(traj.waypoints)
    if not waypoints:
        raise TwinError("轨迹为空，无法采集孪生数据")

    snaps: List[TwinSnapshot] = []
    session.reset()
    for i in range(len(waypoints)):
        wp = session.goto_waypoint(i)
        dev = session.deviation()
        actual = session.read_actual() if session.feedback_available else None
        actual_q = None if actual is None else np.asarray(actual.joints, dtype=np.float64)
        joint_dev = (
            float(np.max(np.abs(actual_q - np.asarray(wp.joints, dtype=np.float64))))
            if actual_q is not None
            else float("nan")
        )
        snaps.append(
            TwinSnapshot(
                t=float(wp.timestamp),
                planned_q=np.asarray(wp.joints, dtype=np.float64).copy(),
                actual_q=actual_q,
                planned_pose=wp.pose,
                actual_pose=None if actual is None else actual.pose,
                pos_dev_mm=dev.pos_mm if dev.available else float("nan"),
                rot_dev_deg=dev.rot_deg if dev.available else float("nan"),
                joint_dev_deg=joint_dev,
            )
        )
    return TwinTrace(
        snapshots=snaps,
        source=source,
        robot_name=robot_name,
        program_name=program_name,
        notes=notes,
    )


# --------------------------------------------------------------------------- #
# 可复现的"真机"：传动误差模型
# --------------------------------------------------------------------------- #


@dataclass
class PlantModel:
    """真机传动/传感误差模型（合成，非真机实测数据）。

    真机偏差有形状、可分解，不是白噪声。这里按工程上最常见的四项建模：

    * ``joint_bias_deg`` —— 关节零位偏置：**常量偏移**，整条轨迹上同号；
    * ``scale_error`` —— 减速比标称误差（相对量，如 ``1e-3`` 表示千分之一）：
      偏差随关节角**线性放大**，所以大幅运动的关节贡献最大；
    * ``noise_deg`` —— 编码器量化/抖动：**零均值随机**，量级小；
    * ``lag_tau`` —— 伺服跟踪一阶滞后（秒）：位置环跟不上指令，**速度突变处**
      （起停、拐角）偏差最大，匀速段几乎无偏差。

    四项叠加后，看偏差曲线的形状就能反推主要成因——这正是数字孪生比"一个
    max 值"有用的地方。

    Attributes:
        joint_bias_deg: 各轴零位偏置（标量=全轴相同，序列=逐轴）
        scale_error: 各轴减速比相对误差（标量=全轴相同，序列=逐轴）
        noise_deg: 编码器噪声标准差（度）
        lag_tau: 一阶滞后时间常数（秒）；``0`` 表示不建模滞后
        seed: 随机种子（保证可复现）
    """

    joint_bias_deg: Any = 0.0
    scale_error: Any = 0.0
    noise_deg: float = 0.0
    lag_tau: float = 0.0
    seed: int = 0

    def __post_init__(self) -> None:
        if self.lag_tau < 0:
            raise TwinError(f"lag_tau 不能为负: {self.lag_tau}")
        if self.noise_deg < 0:
            raise TwinError(f"noise_deg 不能为负: {self.noise_deg}")
        for name in ("joint_bias_deg", "scale_error"):
            vals = self._as_list(getattr(self, name))
            if any(v <= -1.0 for v in vals) and name == "scale_error":
                raise TwinError("scale_error 必须大于 -1（否则减速比非正）")

    @staticmethod
    def _as_list(value: Any) -> List[float]:
        if isinstance(value, (int, float)):
            return [float(value)]
        return [float(v) for v in value]

    @staticmethod
    def _expand(value: Any, dof: int) -> np.ndarray:
        vals = PlantModel._as_list(value)
        if len(vals) == 1:
            return np.full(dof, vals[0], dtype=np.float64)
        if len(vals) != dof:
            raise TwinError(f"参数长度 {len(vals)} 与关节数 {dof} 不符")
        return np.asarray(vals, dtype=np.float64)

    def bias_vector(self, dof: int) -> np.ndarray:
        """展开成 ``dof`` 维零位偏置向量（度）。"""
        if dof <= 0:
            raise TwinError("关节数必须为正")
        return self._expand(self.joint_bias_deg, dof)

    def scale_vector(self, dof: int) -> np.ndarray:
        """展开成 ``dof`` 维传动比误差向量（相对量）。"""
        if dof <= 0:
            raise TwinError("关节数必须为正")
        return self._expand(self.scale_error, dof)

    def summarize(self) -> str:
        """简短描述，用于报告抬头（明确标注是合成模型）。"""
        parts = []
        bias = self._as_list(self.joint_bias_deg)
        scale = self._as_list(self.scale_error)
        if any(abs(v) > 0 for v in bias):
            parts.append(f"零位偏置 {max(abs(v) for v in bias):g}°")
        if any(abs(v) > 0 for v in scale):
            parts.append(f"传动比误差 {max(abs(v) for v in scale) * 100:g}%")
        if self.noise_deg > 0:
            parts.append(f"编码器噪声 σ={self.noise_deg:g}°")
        if self.lag_tau > 0:
            parts.append(f"伺服滞后 τ={self.lag_tau:g}s")
        return "合成真机模型（" + ("、".join(parts) if parts else "无误差") + "）"


class PlantLink(SimLink):
    """把"真机传动误差"接到 :class:`~domain.link.SimLink` 上的回读层。

    与 :class:`~domain.link.SimLink` 的区别只有一处：``set_joints`` 记录的是**指令
    值**，而 ``read_joints`` 回读的是**经传动模型污染后的传感值**。因此把
    :class:`~service.debug.DebugSession` 接在它上面，既有的 :meth:`deviation` 通路
    就会自然产出非零偏差——整条孪生链路无需真机即可验证。

    回读值在**每次下发时采样一次**并缓存：``read_joints`` 与 ``get_pose`` 永远一致，
    同一份数据反复读不会"越读越抖"。

    Args:
        robot: 机器人实例（正/逆运动学 + 关节状态）
        plant: 误差模型；``None`` 表示用默认模型（无误差）
    """

    protocol = "plant"

    def __init__(self, robot: "object", plant: Optional[PlantModel] = None):
        super().__init__(robot)
        self.plant = plant or PlantModel()
        self._dof = int(getattr(robot, "dof", 0)) or 6
        self._bias = self.plant.bias_vector(self._dof)
        self._scale = self.plant.scale_vector(self._dof)
        self._rng = np.random.default_rng(self.plant.seed)
        self._connected = False
        self._q_cmd: Optional[np.ndarray] = None
        self._q_lag: Optional[np.ndarray] = None
        self._q_sensed: Optional[np.ndarray] = None
        self._t_prev: Optional[float] = None

    # ------------------------------ 连接 ------------------------------ #

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        self._connected = True
        try:
            q0 = np.asarray(self.robot.get_joints(), dtype=np.float64).ravel().copy()
        except Exception:
            q0 = np.zeros(self._dof, dtype=np.float64)
        if q0.size != self._dof:
            q0 = np.zeros(self._dof, dtype=np.float64)
        self._q_cmd = q0.copy()
        self._q_lag = q0.copy()
        self._q_sensed = self._sense(q0)
        self._t_prev = None

    def disconnect(self) -> None:
        self._connected = False

    def _require_connected(self) -> None:
        if not self._connected:
            raise TwinError("PlantLink 未连接；请先调用 connect()")

    # ------------------------------ 误差模型 ------------------------------ #

    def _sense(self, q_lag: np.ndarray) -> np.ndarray:
        """把滞后后的真实关节角经"传感器"读成带误差的观测值。"""
        observed = q_lag * (1.0 + self._scale) + self._bias
        if self.plant.noise_deg > 0:
            observed = observed + self._rng.normal(0.0, self.plant.noise_deg, size=observed.size)
        return observed

    def command(self, joints: Any, at_time: Optional[float] = None) -> np.ndarray:
        """下发指令并推进传动模型（``at_time`` 用于计算滞后步长）。

        Returns:
            本次回读到的关节角（deg）
        """
        self._require_connected()
        q = np.asarray(joints, dtype=np.float64).ravel().copy()
        if q.size != self._dof:
            raise TwinError(f"关节数不符：收到 {q.size}，期望 {self._dof}")
        if self.plant.lag_tau > 0 and at_time is not None:
            if self._t_prev is None or at_time < self._t_prev:
                # 两种情况都不建模滞后：
                # ① 首条指令＝"启动前机器人已在轨迹起点"——否则会凭空造出一个
                #    "从 Home 追到起点"的巨大假偏差（首个采样点直接差出几百毫米）；
                # ② **时间倒流＝上层把会话回绕到起点重播**（示教/复测里很常见：
                #    DebugSession.reset() / capture_from_session 都会先回起点）。
                #    机器人物理上已经被搬回起点，若沿用上一次终点的滞后状态，第一个
                #    采样点会差出**整条轨迹**的量级——本轮用 Socket 真机通路实测抓出：
                #    首点 902mm / 133°，看起来像"机器人坏了"，其实是模型没处理回绕。
                self._q_lag = q.copy()
            else:
                dt = max(0.0, at_time - self._t_prev)
                alpha = 1.0 - math.exp(-dt / self.plant.lag_tau)
                self._q_lag = self._q_lag + alpha * (q - self._q_lag)
        else:
            # 没有时刻信息＝"立即到位"（手动 jog / 无时间轴的调用），不建模滞后
            self._q_lag = q.copy()
        if at_time is not None:
            self._t_prev = float(at_time)
        self._q_cmd = q.copy()
        self._q_sensed = self._sense(self._q_lag)
        # 视口/真机状态以"实际"为准（数字孪生要看的是真机在哪，不是它被要求去哪）
        try:
            self.robot.set_joints(self._q_sensed)
        except Exception:
            pass
        return self._q_sensed.copy()

    def set_joints(self, joints: Any, at_time: Optional[float] = None) -> None:
        """兼容 :class:`~domain.link.ControllerLink` 接口的下发入口。"""
        self.command(joints, at_time=at_time)

    def read_joints(self) -> Optional[np.ndarray]:
        """回读（带误差的）实际关节角。"""
        self._require_connected()
        return None if self._q_sensed is None else self._q_sensed.copy()

    @property
    def is_actual_feedback(self) -> bool:
        return True

    def get_pose(self):
        """回读实际 TCP 位姿（由实际关节角正解）。"""
        self._require_connected()
        if self._q_sensed is None:
            return None
        return self.robot.forward_kinematics(self._q_sensed)

    def upload_program(
        self, name: str, code: str, metadata: Optional[Dict[str, Any]] = None
    ) -> str:
        """合成真机不上传程序（只做回读），登记句柄以便与其它连接同构。"""
        self._require_connected()
        return f"plant://{name}"

    def list_programs(self) -> List[str]:
        self._require_connected()
        return []

    @property
    def true_joints(self) -> Optional[np.ndarray]:
        """滞后后的"真实"关节角（未加传感误差）——便于分辨"机械偏差"与"读数偏差"。"""
        return None if self._q_lag is None else self._q_lag.copy()


def capture_plant_trace(
    trajectory: "object",
    robot: "object",
    plant: Optional[PlantModel] = None,
    program_name: str = "",
    notes: str = "",
) -> TwinTrace:
    """用合成真机模型采集一条"真机"轨迹（无真机也能验证整条孪生链路）。

    Args:
        trajectory: 规划得到的 :class:`~service.simulation.Trajectory`
        robot: 机器人实例
        plant: 误差模型；``None`` 用默认（无误差，偏差为 0）
        program_name: 程序名
        notes: 附注；默认自动写入模型摘要

    Returns:
        ``source="plant"`` 的 :class:`TwinTrace`
    """
    from service.debug import DebugSession

    model = plant or PlantModel()
    link = PlantLink(robot, model)
    link.connect()
    session = DebugSession(trajectory, link, robot=robot)
    trace = capture_from_session(
        session,
        source="plant",
        robot_name=getattr(robot, "name", "") or "",
        program_name=program_name,
        notes=notes or model.summarize(),
    )
    return trace


# --------------------------------------------------------------------------- #
# 跨源对齐与差分
# --------------------------------------------------------------------------- #


@dataclass
class AlignedSample:
    """一条对齐后的跨源偏差样本。

    Attributes:
        t: 参考轨迹上的时刻（秒）
        ref_t / meas_t: 两侧原始时刻（秒）——差值就是触发/时钟偏差
        pos_dev_mm: 跨源 TCP 位置差（mm）
        rot_dev_deg: 跨源 TCP 姿态差（deg）
        joint_dev_deg: 跨源关节角最大差（deg）
        over_tol: 是否超出容差
        joint_speed_deg_s: 该样本处的规划关节速度（deg/s）——用于区分
            "低速段残余偏差（=标定问题）"与"高速段附加偏差（=伺服跟踪问题）"
    """

    t: float
    ref_t: float
    meas_t: float
    pos_dev_mm: float
    rot_dev_deg: float
    joint_dev_deg: float
    over_tol: bool = False
    joint_speed_deg_s: float = float("nan")


def _align_indices(
    reference: TwinTrace,
    measured: TwinTrace,
    max_dt: float,
) -> List[Tuple[int, int, float]]:
    """对齐的索引形式：``[(参考序号, 实测序号, dt), ...]``。"""
    if max_dt <= 0:
        raise TwinError(f"max_dt 必须为正: {max_dt}")
    if len(reference) == 0 or len(measured) == 0:
        return []

    ref_times = reference.times
    order = np.argsort(ref_times)
    ref_times_sorted = ref_times[order]

    out: List[Tuple[int, int, float]] = []
    for mi, snap in enumerate(measured.snapshots):
        pos = int(np.searchsorted(ref_times_sorted, snap.t))
        cands = [p for p in (pos - 1, pos, pos + 1) if 0 <= p < ref_times_sorted.size]
        if not cands:
            continue
        best = min(cands, key=lambda p: abs(ref_times_sorted[p] - snap.t))
        dt = float(ref_times_sorted[best] - snap.t)
        if abs(dt) > max_dt:
            continue
        out.append((int(order[best]), mi, dt))
    out.sort(key=lambda item: item[0])
    return out


def align_traces(
    reference: TwinTrace,
    measured: TwinTrace,
    max_dt: float = 0.05,
) -> List[Tuple[TwinSnapshot, TwinSnapshot, float]]:
    """按时间把两条轨迹对齐（最近邻 + 容差窗口）。

    真机与仿真的时间戳不会严格相等（触发时刻、时钟漂移、控制器插补差异），所以
    对齐是**必需**的一步而不是可选优化——不然"偏差"里混进的全是时间错位。

    Args:
        reference: 参考轨迹（通常是仿真）
        measured: 待比对轨迹（通常是真机）
        max_dt: 允许的时间错位（秒）；超出则丢弃该样本

    Returns:
        ``[(ref_snapshot, meas_snapshot, dt), ...]``，按参考轨迹的 ``t`` 升序
    """
    return [
        (reference.snapshots[ri], measured.snapshots[mi], dt)
        for ri, mi, dt in _align_indices(reference, measured, max_dt)
    ]


@dataclass
class TwinReport:
    """真机 vs 仿真（或任意两条轨迹）的偏差报告。

    Attributes:
        samples: 对齐后的逐点偏差
        ref_source / meas_source: 两侧来源标签
        tol_pos_mm / tol_rot_deg: 容差
        unmatched: 因时间错位过大而被丢弃的样本数
        max_dt: 对齐窗口（秒）
    """

    samples: List[AlignedSample] = field(default_factory=list)
    ref_source: str = ""
    meas_source: str = ""
    tol_pos_mm: float = 1.0
    tol_rot_deg: float = 1.0
    unmatched: int = 0
    max_dt: float = 0.05
    notes: str = ""

    def __len__(self) -> int:
        return len(self.samples)

    @property
    def available(self) -> bool:
        """是否有可用的比对样本。"""
        return bool(self.samples)

    def pos_series(self) -> np.ndarray:
        """跨源位置偏差序列（mm）。"""
        return np.asarray([s.pos_dev_mm for s in self.samples], dtype=np.float64)

    def rot_series(self) -> np.ndarray:
        """跨源姿态偏差序列（deg）。"""
        return np.asarray([s.rot_dev_deg for s in self.samples], dtype=np.float64)

    @property
    def max_pos_mm(self) -> float:
        """跨源位置偏差峰值（mm）。"""
        return float(np.max(self.pos_series())) if self.samples else float("nan")

    @property
    def mean_pos_mm(self) -> float:
        """跨源位置偏差均值（mm）。"""
        return float(np.mean(self.pos_series())) if self.samples else float("nan")

    @property
    def rms_pos_mm(self) -> float:
        """跨源位置偏差 RMS（mm）。"""
        if not self.samples:
            return float("nan")
        data = self.pos_series()
        return float(np.sqrt(np.mean(data**2)))

    @property
    def max_rot_deg(self) -> float:
        """跨源姿态偏差峰值（deg）。"""
        return float(np.max(self.rot_series())) if self.samples else float("nan")

    def speed_split(self, slow_below_deg_s: float = 5.0) -> Dict[str, Any]:
        """按关节速度把偏差拆成"标定分量"与"伺服分量"。

        这是本模块最有实用价值的一个数字：**低速段还剩下的偏差，伺服没有责任**
        ——关节几乎不动时跟踪误差趋零，此时仍存在的偏差只能来自零位/传动比/连杆
        尺寸/工具坐标（也就是标定）。高速段超出低速段的那部分，才是位置环带宽
        不够（或速度前馈没配）导致的跟踪误差。

        两者处理方式完全不同：前者要重新标定，后者要调增益或降速，改错了方向
        白费功夫。

        Args:
            slow_below_deg_s: 判定"低速/近停点"的速度阈值（deg/s）

        Returns:
            含 ``slow_max_mm`` / ``fast_max_mm`` / ``n_slow`` / ``n_fast`` /
            ``servo_share``（伺服分量占比）的字典；无样本时数值为 ``nan``
        """
        if not self.samples:
            return {
                "slow_max_mm": float("nan"),
                "fast_max_mm": float("nan"),
                "n_slow": 0,
                "n_fast": 0,
                "servo_share": float("nan"),
            }
        speeds = np.asarray([s.joint_speed_deg_s for s in self.samples], dtype=np.float64)
        pos = self.pos_series()
        valid = ~np.isnan(speeds)
        slow = valid & (speeds <= slow_below_deg_s)
        fast = valid & (speeds > slow_below_deg_s)
        slow_max = float(np.max(pos[slow])) if slow.any() else float("nan")
        fast_max = float(np.max(pos[fast])) if fast.any() else float("nan")
        share = (
            float(max(0.0, (fast_max - slow_max) / fast_max))
            if fast.any() and fast_max > 0
            else float("nan")
        )
        return {
            "slow_max_mm": slow_max,
            "fast_max_mm": fast_max,
            "n_slow": int(slow.sum()),
            "n_fast": int(fast.sum()),
            "servo_share": share,
        }

    @property
    def worst_sample(self) -> Optional[AlignedSample]:
        """偏差最大的样本。"""
        if not self.samples:
            return None
        return max(self.samples, key=lambda s: s.pos_dev_mm)

    @property
    def over_tol(self) -> List[AlignedSample]:
        """超出容差的样本列表。"""
        return [s for s in self.samples if s.over_tol]

    @property
    def ok(self) -> bool:
        """是否全部样本都在容差内。"""
        return self.available and not self.over_tol

    def verdict(self) -> str:
        """一句话结论。"""
        if not self.available:
            return "无可用比对样本（时间窗口内未对齐上任何点）"
        if self.ok:
            return (
                f"通过：全程最大 {self.max_pos_mm:.2f} mm / {self.max_rot_deg:.2f}°，"
                f"在 {self.tol_pos_mm:g} mm / {self.tol_rot_deg:g}° 容差内"
            )
        w = self.worst_sample
        return (
            f"超差：{len(self.over_tol)}/{len(self.samples)} 个样本越过容差，"
            f"最差 t={w.t:.2f}s 处 {w.pos_dev_mm:.2f} mm"
        )

    def to_dict(self) -> Dict[str, Any]:
        """转成可序列化字典。"""
        return {
            "ref_source": self.ref_source,
            "meas_source": self.meas_source,
            "tol_pos_mm": self.tol_pos_mm,
            "tol_rot_deg": self.tol_rot_deg,
            "max_dt": self.max_dt,
            "unmatched": self.unmatched,
            "notes": self.notes,
            "max_pos_mm": self.max_pos_mm,
            "mean_pos_mm": self.mean_pos_mm,
            "rms_pos_mm": self.rms_pos_mm,
            "max_rot_deg": self.max_rot_deg,
            "n_over_tol": len(self.over_tol),
            "ok": self.ok,
            "samples": [
                {
                    "t": s.t,
                    "ref_t": s.ref_t,
                    "meas_t": s.meas_t,
                    "pos_dev_mm": s.pos_dev_mm,
                    "rot_dev_deg": s.rot_dev_deg,
                    "joint_dev_deg": s.joint_dev_deg,
                    "joint_speed_deg_s": s.joint_speed_deg_s,
                    "over_tol": s.over_tol,
                }
                for s in self.samples
            ],
        }

    def format(self, max_rows: int = 12) -> str:
        """文本报表：抬头 + 统计 + 偏差最大的若干行。"""
        lines = [
            f"数字孪生偏差报告（{self.ref_source} vs {self.meas_source}）",
            f"  对齐 {len(self.samples)} 个样本（窗口 ±{self.max_dt * 1000:.0f} ms，"
            f"丢弃 {self.unmatched} 个）",
        ]
        if self.notes:
            lines.append(f"  注：{self.notes}")
        if not self.available:
            lines.append("  无可用样本。")
            return "\n".join(lines)
        split = self.speed_split()
        lines += [
            f"  位置偏差：最大 {self.max_pos_mm:.3f} mm / 均值 {self.mean_pos_mm:.3f} / "
            f"RMS {self.rms_pos_mm:.3f}",
            f"  姿态偏差：最大 {self.max_rot_deg:.3f}°",
            f"  容差 {self.tol_pos_mm:g} mm / {self.tol_rot_deg:g}° → "
            f"{'通过' if self.ok else f'{len(self.over_tol)} 个超差'}",
            f"  速度分解：低速段（≤5°/s，{split['n_slow']} 点）最大 "
            f"{split['slow_max_mm']:.3f} mm（≈标定分量）｜ 高速段（{split['n_fast']} 点）"
            f"最大 {split['fast_max_mm']:.3f} mm（含伺服跟踪）",
            "  偏差最大的样本：",
            "    t(s)     位置(mm)   姿态(°)   关节(°)  速度(°/s)  超差",
        ]
        ranked = sorted(self.samples, key=lambda s: s.pos_dev_mm, reverse=True)[:max_rows]
        for s in ranked:
            lines.append(
                f"    {s.t:6.2f}  {s.pos_dev_mm:9.3f}  {s.rot_dev_deg:8.3f}  "
                f"{s.joint_dev_deg:8.3f}  {s.joint_speed_deg_s:9.2f}   "
                f"{'是' if s.over_tol else '否'}"
            )
        if len(self.samples) > max_rows:
            lines.append(f"    …（共 {len(self.samples)} 个样本）")
        lines.append(f"  结论：{self.verdict()}")
        return "\n".join(lines)

    def format_causes(self) -> str:
        """按偏差曲线的形状给"主要成因"提示。

        这不是诊断结论，而是**读数的方法**：把三种典型误差的指纹列出来，让工程师
        一眼判断该查零位、查减速比、还是查伺服增益。
        """
        if not self.available:
            return "无可用样本，无法给出成因提示。"
        pos = self.pos_series()
        rot = self.rot_series()
        split = self.speed_split()
        constant = float(np.abs(np.mean(pos)))
        spread = float(np.std(pos))
        peak = float(np.max(pos))
        lines = [
            "偏差形状提示（按经验指纹，供排查方向参考）：",
            f"  常量分量 {constant:.3f} mm / 波动分量 {spread:.3f} mm / 峰值 {peak:.3f} mm",
            f"  低速段峰值 {split['slow_max_mm']:.3f} mm（{split['n_slow']} 点，"
            f"伺服几乎不背锅）｜ 高速段峰值 {split['fast_max_mm']:.3f} mm",
        ]
        if peak <= max(1e-9, 1e-3):
            lines.append("  · 全程接近零：仿真与真机一致，无需调参")
            return "\n".join(lines)

        slow = split["slow_max_mm"]
        fast = split["fast_max_mm"]
        has_split = not math.isnan(slow) and not math.isnan(fast) and split["n_slow"] > 0
        servo_dominant = has_split and fast > 1.5 * slow

        # 速度分解是**直接证据**，优先级高于"常量分量占主导"这类形状启发式：
        # 同号的跟踪误差一样会把均值抬得很高，只看均值会把伺服问题误判成标定问题。
        if has_split and slow > 0.3 and not servo_dominant:
            lines.append(
                "  · **低速段偏差就已超标** → 这是标定问题，调伺服增益不会有改善："
                "重新做 TCP/关节零点/连杆尺寸标定"
            )
        elif servo_dominant:
            lines.append(
                f"  · 高速段比低速段高出 {fast / max(slow, 1e-9):.1f} 倍 → 主要是"
                "**伺服跟踪误差**（降速、加前馈、调位置环增益）。注意此时均值/常量分量"
                "也会偏高，但那只是同号跟踪误差抬起来的，**别据此去重新标定**"
            )
        elif has_split:
            lines.append("  · 低速段与高速段量级相当 → 标定与伺服都有贡献，建议分开整治")
        else:
            lines.append(
                "  · 全轨迹无低速/停点样本 → 无法分离标定与伺服，建议在关键点加驻留"
                "（停一下再走）再复测"
            )
            if constant > max(1e-9, 0.5 * peak):
                lines.append(
                    "  · 常量偏移占主导 → 优先查**关节零位/工具坐标**（仅形状推测，未证实）"
                )

        if spread > 0.25 * peak:
            lines.append("  · 随关节角增长的波动 → 顺带查**减速比标称误差 / 连杆尺寸标定**")
        if float(np.max(rot)) > 0.5:
            lines.append("  · 姿态偏差明显 → 工具姿态标定或关节零位，而非纯位置偏置")
        return "\n".join(lines)


def compare_traces(
    reference: TwinTrace,
    measured: TwinTrace,
    tol_pos_mm: float = 1.0,
    tol_rot_deg: float = 1.0,
    max_dt: float = 0.05,
    notes: str = "",
) -> TwinReport:
    """对齐两条轨迹并逐点差分，得到跨源偏差报告。

    与 :class:`~service.debug.DebugSession` 的关系：``DebugSession.deviation`` 给的是
    **单一来源内部**"规划 vs 实际"的差；这里给的是**跨来源**"仿真 vs 真机"的差。
    两个口径都要看——前者看控制器伺服跟得怎么样，后者看模型和机器像不像。

    Args:
        reference: 参考轨迹（仿真）
        measured: 待比对轨迹（真机）
        tol_pos_mm: 位置容差（mm）
        tol_rot_deg: 姿态容差（deg）
        max_dt: 时间对齐窗口（秒）
        notes: 附注（会出现在报告抬头）

    Returns:
        :class:`TwinReport`
    """
    pairs = _align_indices(reference, measured, max_dt=max_dt)
    ref_speeds = reference.joint_speed_series()

    samples: List[AlignedSample] = []
    for ri, mi, dt in pairs:
        ref = reference.snapshots[ri]
        meas = measured.snapshots[mi]
        if ref.planned_pose is None or meas.actual_pose is None:
            # 退化为关节空间比较（缺位姿时仍能给出关节差）
            rq = np.asarray(ref.planned_q, dtype=np.float64)
            mq = np.asarray(meas.actual_q, dtype=np.float64)
            if mq.size != rq.size:
                continue
            joint_dev = float(np.max(np.abs(mq - rq)))
            pos_dev = float("nan")
            rot_dev = float("nan")
        else:
            pos_dev, rot_dev = pose_deviation(ref.planned_pose, meas.actual_pose)
            rq = np.asarray(ref.planned_q, dtype=np.float64)
            mq = np.asarray(meas.actual_q, dtype=np.float64) if meas.actual_q is not None else rq
            joint_dev = float(np.max(np.abs(mq - rq))) if mq.size == rq.size else float("nan")
        over = False
        if not math.isnan(pos_dev):
            over = over or pos_dev > tol_pos_mm
        if not math.isnan(rot_dev):
            over = over or rot_dev > tol_rot_deg
        samples.append(
            AlignedSample(
                t=float(ref.t),
                ref_t=float(ref.t),
                meas_t=float(meas.t),
                pos_dev_mm=pos_dev,
                rot_dev_deg=rot_dev,
                joint_dev_deg=joint_dev,
                over_tol=over,
                joint_speed_deg_s=float(ref_speeds[ri]),
            )
        )

    unmatched = max(0, min(len(reference), len(measured)) - len(samples))
    return TwinReport(
        samples=samples,
        ref_source=reference.source,
        meas_source=measured.source,
        tol_pos_mm=tol_pos_mm,
        tol_rot_deg=tol_rot_deg,
        unmatched=unmatched,
        max_dt=max_dt,
        notes=notes or (measured.notes if measured.notes else ""),
    )


# --------------------------------------------------------------------------- #
# 偏差可视化（无依赖 SVG）
# --------------------------------------------------------------------------- #


_PALETTES = {
    "dark": {
        "bg": "#161a20",
        "title": "#e8a33d",
        "axis": "#3a4250",
        "grid": "#232a34",
        "text": "#9aa5b5",
        "curve": "#6fdc6f",
        "band": "#4a90d9",
        "over": "#e05a5a",
    },
    "light": {
        "bg": "#ffffff",
        "title": "#b26a00",
        "axis": "#c3cad6",
        "grid": "#eceff4",
        "text": "#5a6472",
        "curve": "#1f8f4a",
        "band": "#1f6fd0",
        "over": "#c62828",
    },
}


def deviation_chart_svg(
    report: TwinReport,
    width: int = 760,
    height: int = 300,
    theme: str = "dark",
    max_points: int = 400,
    title: Optional[str] = None,
) -> str:
    """把跨源偏差报告画成一张独立 SVG（无 matplotlib 依赖，可直接入页/入库）。

    Args:
        report: :func:`compare_traces` 的结果
        width / height: 画布尺寸（px）
        theme: ``"dark"`` / ``"light"``
        max_points: 折线最多采样点数（超长轨迹等距抽稀，仅影响绘制）
        title: 标题；``None`` 时自动生成

    Returns:
        完整 ``<svg>`` 字符串
    """
    if theme not in _PALETTES:
        raise TwinError(f"未知主题: {theme!r}（用 dark / light）")
    if width < 200 or height < 120:
        raise TwinError(f"画布过小: {width}x{height}")

    pal = _PALETTES[theme]
    pad_l, pad_r, pad_t, pad_b = 56, 20, 34, 34
    plot_w = width - pad_l - pad_r
    plot_h = height - pad_t - pad_b

    head = (
        f'<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg" '
        f'width="{width}" height="{height}" role="img">'
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="{pal["bg"]}" rx="10"/>'
    )
    title_text = title or (
        f"TCP 偏差（{report.ref_source} vs {report.meas_source}）· "
        f"容差 {report.tol_pos_mm:g} mm"
    )
    head += (
        f'<text x="{pad_l}" y="22" fill="{pal["title"]}" font-size="13" '
        f'font-weight="bold">{_xml_escape(title_text)}</text>'
    )

    if not report.available:
        return (
            head + f'<text x="{width / 2:.0f}" y="{height / 2:.0f}" fill="{pal["text"]}" '
            f'font-size="13" text-anchor="middle">无可用比对样本</text></svg>'
        )

    ts = np.asarray([s.t for s in report.samples], dtype=np.float64)
    pos = report.pos_series()
    t0, t1 = float(ts[0]), float(ts[-1])
    if t1 - t0 <= 0:
        t1 = t0 + 1e-6
    y_max = float(np.max(pos))
    y_scale = max(y_max, report.tol_pos_mm) * 1.15
    if y_scale <= 0:
        y_scale = 1.0

    def sx(t: float) -> float:
        return pad_l + (t - t0) / (t1 - t0) * plot_w

    def sy(v: float) -> float:
        return pad_t + plot_h - min(v, y_scale) / y_scale * plot_h

    # 网格 + 刻度
    for k in range(5):
        y = pad_t + plot_h * k / 4.0
        val = y_scale * (1 - k / 4.0)
        head += (
            f'<line x1="{pad_l}" y1="{y:.1f}" x2="{pad_l + plot_w}" y2="{y:.1f}" '
            f'stroke="{pal["grid"]}" stroke-width="1"/>'
            f'<text x="{pad_l - 6}" y="{y + 4:.1f}" fill="{pal["text"]}" font-size="10" '
            f'text-anchor="end">{val:.1f}</text>'
        )
    for k in range(5):
        t = t0 + (t1 - t0) * k / 4.0
        x = sx(t)
        head += (
            f'<text x="{x:.1f}" y="{pad_t + plot_h + 16}" fill="{pal["text"]}" '
            f'font-size="10" text-anchor="middle">{t:.1f}</text>'
        )
    head += (
        f'<line x1="{pad_l}" y1="{pad_t}" x2="{pad_l}" y2="{pad_t + plot_h}" '
        f'stroke="{pal["axis"]}" stroke-width="1"/>'
        f'<line x1="{pad_l}" y1="{pad_t + plot_h}" x2="{pad_l + plot_w}" '
        f'y2="{pad_t + plot_h}" stroke="{pal["axis"]}" stroke-width="1"/>'
    )

    # 容差带
    tol_y = sy(report.tol_pos_mm)
    if report.tol_pos_mm <= y_scale:
        head += (
            f'<line x1="{pad_l}" y1="{tol_y:.1f}" x2="{pad_l + plot_w}" y2="{tol_y:.1f}" '
            f'stroke="{pal["band"]}" stroke-width="1" stroke-dasharray="6 4"/>'
            f'<text x="{pad_l + plot_w}" y="{tol_y - 4:.1f}" fill="{pal["band"]}" '
            f'font-size="10" text-anchor="end">容差 {report.tol_pos_mm:g} mm</text>'
        )

    # 折线（等距抽稀）
    step = max(1, int(math.ceil(len(ts) / max_points)))
    idx = list(range(0, len(ts), step))
    if idx[-1] != len(ts) - 1:
        idx.append(len(ts) - 1)
    pts = " ".join(f"{sx(ts[i]):.1f},{sy(pos[i]):.1f}" for i in idx)
    head += f'<polyline points="{pts}" fill="none" stroke="{pal["curve"]}" stroke-width="2"/>'

    # 超差点
    for s in report.samples:
        if s.over_tol:
            head += (
                f'<circle cx="{sx(s.t):.1f}" cy="{sy(s.pos_dev_mm):.1f}" r="3.5" '
                f'fill="{pal["over"]}"/>'
            )

    # 最差点标注
    w = report.worst_sample
    if w is not None:
        head += (
            f'<circle cx="{sx(w.t):.1f}" cy="{sy(w.pos_dev_mm):.1f}" r="3" '
            f'fill="{pal["title"]}"/>'
            f'<text x="{sx(w.t):.1f}" y="{sy(w.pos_dev_mm) - 8:.1f}" fill="{pal["title"]}" '
            f'font-size="10" text-anchor="middle">峰值 {w.pos_dev_mm:.2f} mm'
            f" @ {w.t:.2f}s</text>"
        )
    head += (
        f'<text x="{pad_l - 40}" y="{pad_t - 8}" fill="{pal["text"]}" font-size="10">'
        f"mm</text>"
        f'<text x="{pad_l + plot_w}" y="{height - 6}" fill="{pal["text"]}" '
        f'font-size="10" text-anchor="end">t (s)</text>'
        f"</svg>"
    )
    return head


def _xml_escape(text: str) -> str:
    return (
        str(text)
        .replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


# --------------------------------------------------------------------------- #
# 云协作：把 ControllerLink 预留的"云"通道落地
# --------------------------------------------------------------------------- #


class CloudTransport(ABC):
    """云端存储的传输抽象（键值语义）。

    默认实现是文件式（:class:`LocalCloudStore`），离线可用、零依赖；接真实云
    （HTTP / S3 / 对象存储）时只需实现这四个方法——孪生层不关心底层是网盘还是
    本机目录。
    """

    @abstractmethod
    def put(self, key: str, data: bytes) -> str:
        """写入对象，返回可读句柄（如 ``cloud://<key>``）。"""

    @abstractmethod
    def get(self, key: str) -> Optional[bytes]:
        """读取对象；不存在返回 ``None``。"""

    @abstractmethod
    def keys(self, prefix: str = "") -> List[str]:
        """列出前缀下的全部键（升序）。"""

    @abstractmethod
    def delete(self, key: str) -> bool:
        """删除对象；返回是否真的删掉了。"""

    def exists(self, key: str) -> bool:
        """对象是否存在。"""
        return self.get(key) is not None


def normalize_key(key: str) -> str:
    """规范化云端键：去首尾斜杠、拒绝 ``..`` 与空键（防止路径穿越）。"""
    k = str(key).strip().strip("/")
    if not k:
        raise TwinError("云端键不能为空")
    parts = [p for p in k.split("/") if p not in ("", ".")]
    if any(p == ".." for p in parts):
        raise TwinError(f"云端键不允许包含 '..': {key!r}")
    return "/".join(parts)


class MemoryCloudStore(CloudTransport):
    """内存云端（测试 / 单进程演示用）。"""

    def __init__(self) -> None:
        self._data: Dict[str, bytes] = {}

    def put(self, key: str, data: bytes) -> str:
        k = normalize_key(key)
        self._data[k] = bytes(data)
        return f"cloud://{k}"

    def get(self, key: str) -> Optional[bytes]:
        return self._data.get(normalize_key(key))

    def keys(self, prefix: str = "") -> List[str]:
        p = normalize_key(prefix) if prefix.strip("/") else ""
        return sorted(k for k in self._data if k.startswith(p))

    def delete(self, key: str) -> bool:
        return self._data.pop(normalize_key(key), None) is not None


class LocalCloudStore(CloudTransport):
    """文件式云端：把一个本机目录当作对象存储根。

    用于离线环境与演示——没有网络也能把"上传 / 回读 / 下载"整条协议跑通，接真云
    时替换实现即可。
    """

    def __init__(self, root: str | Path):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)

    def _path(self, key: str) -> Path:
        return self.root / normalize_key(key)

    def put(self, key: str, data: bytes) -> str:
        p = self._path(key)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(bytes(data))
        return f"cloud://{normalize_key(key)}"

    def get(self, key: str) -> Optional[bytes]:
        p = self._path(key)
        if not p.is_file():
            return None
        return p.read_bytes()

    def keys(self, prefix: str = "") -> List[str]:
        """列出前缀下的全部键（升序）。

        前缀是**字符串前缀**语义（``"b/1"`` 能匹配 ``b/1.txt``），与
        :class:`MemoryCloudStore` 和 HTTP 实现保持一致——三种传输的契约
        必须逐字相同（见 ``tests/test_cloudhttp.py`` 的契约一致性用例）。
        """
        p = normalize_key(prefix) if prefix.strip("/") else ""
        if not self.root.exists():
            return []
        out: List[str] = []
        for f in self.root.rglob("*"):
            if f.is_file():
                rel = f.relative_to(self.root).as_posix()
                if rel.startswith(p):
                    out.append(rel)
        return sorted(out)

    def delete(self, key: str) -> bool:
        p = self._path(key)
        if p.is_file():
            p.unlink()
            return True
        return False


class CloudLink(ControllerLink):
    """云侧控制器连接：程序上传 + 孪生轨迹托管 + 真机状态回读。

    这是 :class:`~domain.link.ControllerLink` 里预留的那条通道的落地实现。它和
    :class:`~domain.link.FileLink`（本地落盘）/:class:`~domain.link.SimLink`（进程内仿真）
    是**同一个抽象下的第三种传输**——上层（deploy / debug / 孪生层）不需要知道
    程序是写在磁盘上还是某个对象存储里。

    云端目录约定::

        <namespace>/programs/<name>.<ext>          生成的控制器代码
        <namespace>/programs/<name>.manifest.json  部署清单（品牌/行数/字节数）
        <namespace>/traces/<name>.jsonl            孪生轨迹（首行元数据 + 逐行快照）
        <namespace>/status.json                    真机最近一次状态上报

    Args:
        transport: 传输实现（:class:`LocalCloudStore` / :class:`MemoryCloudStore` / 自建）
        robot: 可选机器人实例——有它才能由上报关节角正解出 TCP 位姿
        namespace: 云端命名空间（多台设备 / 多个项目隔离用）
    """

    protocol = "cloud"
    #: 标记该连接具备云端能力（上层据此决定是否上传孪生轨迹）
    supports_cloud = True

    def __init__(
        self,
        transport: CloudTransport,
        robot: "object" = None,
        namespace: str = "robotworld",
    ):
        if transport is None:
            raise TwinError("CloudLink 需要传输实现（如 LocalCloudStore / MemoryCloudStore）")
        super().__init__()
        self.transport = transport
        self.robot = robot
        self.namespace = normalize_key(namespace)
        self._connected = False

    # ------------------------------ 连接 ------------------------------ #

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        self._connected = True

    def disconnect(self) -> None:
        self._connected = False

    def _require_connected(self) -> None:
        if not self._connected:
            raise TwinError("CloudLink 未连接；请先调用 connect()")

    def _key(self, *parts: str) -> str:
        return normalize_key("/".join([self.namespace, *parts]))

    # ------------------------------ 程序部署 ------------------------------ #

    def upload_program(
        self, name: str, code: str, metadata: Optional[Dict[str, Any]] = None
    ) -> str:
        """把生成的控制器代码上传到云端并登记清单。

        Returns:
            云端句柄 ``cloud://<namespace>/programs/<safe>.<ext>``
        """
        self._require_connected()
        meta = dict(metadata or {})
        ext = str(meta.get("ext", "src")).lstrip(".")
        safe = _safe_name(name)
        code_key = self._key("programs", f"{safe}.{ext}")
        handle = self.transport.put(code_key, code.encode("utf-8"))
        manifest = {
            "name": name,
            "ext": ext,
            "post_processor": meta.get("post_processor"),
            "brand": meta.get("brand"),
            "robot_name": meta.get("robot_name"),
            "lines": code.count("\n") + (1 if code and not code.endswith("\n") else 0),
            "bytes": len(code.encode("utf-8")),
            "uploaded_at": time.time(),
        }
        self.transport.put(
            self._key("programs", f"{safe}.manifest.json"),
            json.dumps(manifest, ensure_ascii=False, indent=2).encode("utf-8"),
        )
        return handle

    def list_programs(self) -> List[str]:
        """列出云端已部署的程序名（同名不同扩展名只记一次）。"""
        self._require_connected()
        names: List[str] = []
        for key in self.transport.keys(self._key("programs")):
            base = key.rsplit("/", 1)[-1]
            if base.endswith(".manifest.json"):
                continue
            stem = base.rsplit(".", 1)[0]
            if stem not in names:
                names.append(stem)
        return names

    def fetch_program(self, name: str, ext: str = "src") -> Optional[str]:
        """取回云端代码文本；不存在返回 ``None``。"""
        self._require_connected()
        data = self.transport.get(self._key("programs", f"{_safe_name(name)}.{ext.lstrip('.')}"))
        return None if data is None else data.decode("utf-8")

    def program_manifest(self, name: str) -> Optional[Dict[str, Any]]:
        """取回部署清单。"""
        self._require_connected()
        data = self.transport.get(self._key("programs", f"{_safe_name(name)}.manifest.json"))
        return None if data is None else json.loads(data.decode("utf-8"))

    # ------------------------------ 孪生轨迹 ------------------------------ #

    def publish_trace(self, trace: TwinTrace, name: Optional[str] = None) -> str:
        """上传一条孪生轨迹（JSONL，可逐行进增量上报）。

        Returns:
            云端句柄 ``cloud://<namespace>/traces/<name>.jsonl``
        """
        self._require_connected()
        trace_name = name or trace.program_name or f"trace_{int(trace.created_at)}"
        safe = _safe_name(trace_name)
        key = self._key("traces", f"{safe}.jsonl")
        handle = self.transport.put(key, trace.to_jsonl().encode("utf-8"))
        # 索引：记录来源与样本数，避免为了"看一眼有哪些轨迹"就下载全部内容
        max_pos = trace.max_pos_dev()
        self.transport.put(
            self._key("traces", f"{safe}.meta.json"),
            json.dumps(
                {
                    "name": trace_name,
                    "source": trace.source,
                    "robot_name": trace.robot_name,
                    "program_name": trace.program_name,
                    "samples": len(trace),
                    "duration_s": trace.duration,
                    # nan 不是合法 JSON（strict 解析器会炸），无实测数据时写 null
                    "max_pos_mm": None if math.isnan(max_pos) else max_pos,
                    "created_at": trace.created_at,
                    "notes": trace.notes,
                },
                ensure_ascii=False,
                indent=2,
            ).encode("utf-8"),
        )
        return handle

    def list_traces(self) -> List[str]:
        """列出云端已有轨迹名。"""
        self._require_connected()
        names: List[str] = []
        for key in self.transport.keys(self._key("traces")):
            base = key.rsplit("/", 1)[-1]
            if base.endswith(".meta.json") or not base.endswith(".jsonl"):
                continue
            names.append(base[: -len(".jsonl")])
        return names

    def trace_meta(self, name: str) -> Optional[Dict[str, Any]]:
        """取回轨迹索引（不下载轨迹本体）。"""
        self._require_connected()
        data = self.transport.get(self._key("traces", f"{_safe_name(name)}.meta.json"))
        return None if data is None else json.loads(data.decode("utf-8"))

    def fetch_trace(self, name: str) -> TwinTrace:
        """取回云端轨迹；不存在抛 :class:`TwinError`。

        ``source`` **保持原样**（``sim`` / ``plant`` / ``real``）——把来源改写成
        ``"cloud"`` 会丢掉"这条数据到底是不是真机测的"这个最要紧的信息。
        "经过云端"这件事追加到 :attr:`~service.twinsync.TwinTrace.notes` 里，
        这样再把它传一手时，链路痕迹也不会被抹掉。
        """
        self._require_connected()
        data = self.transport.get(self._key("traces", f"{_safe_name(name)}.jsonl"))
        if data is None:
            raise TwinError(f"云端没有轨迹 {name!r}（可用 list_traces() 查看）")
        trace = TwinTrace.from_jsonl(data.decode("utf-8"))
        mark = f"（{time.strftime('%Y-%m-%d %H:%M:%S')} 从云端取回，来源仍标 {trace.source!r}）"
        trace.notes = f"{trace.notes} {mark}".strip()
        return trace

    # ------------------------------ 真机状态回读 ------------------------------ #

    def push_status(
        self,
        joints: Any,
        t: Optional[float] = None,
        source: str = "real",
        extra: Optional[Dict[str, Any]] = None,
    ) -> str:
        """真机侧上报一次状态（关节角 + 时刻），供孪生视图回显。"""
        self._require_connected()
        q = np.asarray(joints, dtype=np.float64).ravel()
        payload: Dict[str, Any] = {
            "joints": [float(v) for v in q],
            "reported_at": float(time.time() if t is None else t),
            "source": source,
        }
        if extra:
            payload.update(extra)
        return self.transport.put(
            self._key("status.json"),
            json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8"),
        )

    def status(self) -> Optional[Dict[str, Any]]:
        """云端最近一次状态上报（``None`` 表示从未上报）。"""
        self._require_connected()
        data = self.transport.get(self._key("status.json"))
        return None if data is None else json.loads(data.decode("utf-8"))

    def status_age(self, now: Optional[float] = None) -> Optional[float]:
        """上报数据有多旧（秒）；从未上报返回 ``None``。

        "实时回显"必须能被质疑——这个数字就是质疑的依据。
        """
        st = self.status()
        if st is None:
            return None
        return float((time.time() if now is None else now) - float(st["reported_at"]))

    def is_fresh(self, max_age: float = 5.0, now: Optional[float] = None) -> bool:
        """上报是否仍然新鲜（≤ ``max_age`` 秒）。"""
        age = self.status_age(now=now)
        return age is not None and age <= max_age

    @property
    def is_actual_feedback(self) -> bool:
        """有状态上报时可回读实际关节角，孪生视图即可工作。"""
        return self.status() is not None

    def read_joints(self) -> Optional[np.ndarray]:
        """回读真机上报的关节角（deg）；无上报返回 ``None``。"""
        st = self.status()
        if st is None:
            return None
        return np.asarray(st["joints"], dtype=np.float64)

    def get_pose(self):
        """由上报关节角正解 TCP 位姿；缺机器人实例时返回 ``None``。"""
        q = self.read_joints()
        if q is None or self.robot is None:
            return None
        return self.robot.forward_kinematics(q)

    def set_joints(self, joints: Any) -> None:
        """云端连接不反向下发指令（只读回显），静默忽略。"""
        return None

    # ------------------------------ 综合 ------------------------------ #

    def summary(self) -> Dict[str, Any]:
        """云端现状摘要（程序数、轨迹数、回显新鲜度）。"""
        st = self.status()
        age = None if st is None else (time.time() - float(st["reported_at"]))
        return {
            "namespace": self.namespace,
            "protocol": self.protocol,
            "connected": self.connected,
            "programs": self.list_programs() if self.connected else [],
            "traces": self.list_traces() if self.connected else [],
            "has_status": st is not None,
            "status_age_s": age,
        }


def _safe_name(name: str) -> str:
    """把任意名字规范成安全的键片段（去路径语义、去前导点、防 ``..`` 残留）。"""
    safe = "".join(c if (c.isalnum() or c in "-_.") else "_" for c in str(name))
    while ".." in safe:
        safe = safe.replace("..", "_")
    safe = safe.lstrip(".")
    return safe or "unnamed"
