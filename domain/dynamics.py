"""机器人动力学：连杆惯性参数 + 牛顿-欧拉逆动力学 + 力矩/功率校验。

运动学回答"机器人能不能到"，动力学回答"到了之后关节扛不扛得住"。
本模块做三件事：

1. **惯性参数双轨录入**——自动解析 URDF ``<inertial>``（CAD 导出轨道），
   缺的连杆可用 :meth:`DynamicsModel.set_link_inertial` 手动补
   （手册/估算轨道）；末端负载用 :meth:`DynamicsModel.set_payload`。
2. **逆动力学**——:meth:`DynamicsModel.inverse_dynamics` 用世界系
   牛顿-欧拉递推（Luh-Walker-Paul）计算给定 ``(q, qd, qdd)`` 的关节
   力矩，含重力、离心/科氏项与连杆惯性张量。
3. **轨迹力矩校验**——:meth:`DynamicsModel.check_trajectory` 沿规划器
   展开的轨迹逐路点算力矩与机械功率，输出峰值/额定利用率，供
   :class:`~service.planner.ProgramReport` 生成力矩报表、GUI 超限标红。

单位约定
--------
对外力矩 N·m（移动关节 N）、功率 W、质量 kg；关节值/速度/加速度在
:meth:`inverse_dynamics` 内部一律 **米 + 弧度 + 秒**（URDF 同制），
调用方传入 :class:`~service.simulation.Waypoint` 的 deg / mm 值时由
:meth:`check_trajectory` 负责换算。

局限
----
* 只处理主链（根到末端），主链之外的分支连杆不计入惯性——串联臂
  与法兰工具都在主链上，普通场景不受影响。
* 重力沿基坐标系 **−Z** 方向（工业机器人的常规安装姿态）；侧装/
  倒装需要另行定义基座姿态，暂不支持。
"""

from __future__ import annotations

import math
import hashlib
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Dict, List, Optional, Sequence, Tuple

import numpy as np

if TYPE_CHECKING:  # 仅类型标注：注解里的 Trajectory 借名字，避免运行时依赖仿真层
    from service.simulation import Trajectory

__all__ = [
    "LinkInertial",
    "DynamicsModel",
    "DEFAULT_JOINT_EFFORT",
    "TORQUE_WARN_RATIO",
    "GRAVITY",
]

#: 重力加速度（基坐标系 Z 轴向上）
GRAVITY = np.array([0.0, 0.0, -9.81])

#: URDF 未给出 ``<limit effort>`` 时的默认额定力矩（N·m；移动关节 N）。
#: 取常见 6 轴工业臂肩部关节额定值附近，宁松勿紧——校验的价值在
#: 相对比较与超限报警，不在绝对精度
DEFAULT_JOINT_EFFORT = 100.0

#: 峰值力矩达到额定的该比例时给黄色预警（超 100% 是红色错误）
TORQUE_WARN_RATIO = 0.85


# --------------------------------------------------------------------------- #
# 连杆惯性
# --------------------------------------------------------------------------- #


@dataclass
class LinkInertial:
    """一个连杆的惯性参数（连杆坐标系下）。

    Attributes:
        mass: 质量 (kg)
        com: 质心位置，连杆坐标系 (m)
        inertia: 关于质心的 3×3 惯性张量 (kg·m²)；点质量给零矩阵
    """

    mass: float
    com: np.ndarray
    inertia: np.ndarray

    def __post_init__(self) -> None:
        self.com = np.asarray(self.com, dtype=np.float64).ravel()
        self.inertia = np.asarray(self.inertia, dtype=np.float64).reshape(3, 3)

    @classmethod
    def point_mass(cls, mass: float, com: Sequence[float]) -> "LinkInertial":
        """点质量（忽略自身转动惯量）——手册数据常只给质量与质心。"""
        return cls(
            mass=mass,
            com=np.asarray(com, dtype=np.float64),
            inertia=np.zeros((3, 3)),
        )

    def merged_with(self, other: "LinkInertial") -> "LinkInertial":
        """两个刚性相连的惯性合并为一个（平行轴定理），用于负载叠加。"""
        m = self.mass + other.mass
        if m <= 0.0:
            return LinkInertial(0.0, np.zeros(3), np.zeros((3, 3)))
        com = (self.mass * self.com + other.mass * other.com) / m
        # 惯性张量平移：I_new = I + m·((d·d)E − d dᵀ)，d = 原质心 → 新质心
        inertia = self.inertia.copy()

        def _parallel_axis(mass: float, d: np.ndarray) -> np.ndarray:
            return mass * (float(d @ d) * np.eye(3) - np.outer(d, d))

        inertia += _parallel_axis(self.mass, self.com - com)
        inertia += other.inertia + _parallel_axis(other.mass, other.com - com)
        return LinkInertial(mass=m, com=com, inertia=inertia)


def _parse_inertial_el(el) -> Optional[LinkInertial]:
    """解析 URDF ``<inertial>`` 元素。"""
    mass_el = el.find("mass")
    if mass_el is None:
        return None
    try:
        mass = float(mass_el.get("value", "0"))
    except ValueError:
        return None
    if mass <= 0.0:
        return None

    origin_el = el.find("origin")
    if origin_el is not None:
        xyz = [float(v) for v in (origin_el.get("xyz") or "0 0 0").split()]
    else:
        xyz = [0.0, 0.0, 0.0]
    com = np.array(xyz[:3] if len(xyz) >= 3 else xyz + [0.0] * (3 - len(xyz)))

    inertia = np.zeros((3, 3))
    i_el = el.find("inertia")
    if i_el is not None:
        try:
            ixx = float(i_el.get("ixx", "0"))
            iyy = float(i_el.get("iyy", "0"))
            izz = float(i_el.get("izz", "0"))
            ixy = float(i_el.get("ixy", "0"))
            ixz = float(i_el.get("ixz", "0"))
            iyz = float(i_el.get("iyz", "0"))
        except ValueError:
            ixx = iyy = izz = ixy = ixz = iyz = 0.0
        inertia = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
    return LinkInertial(mass=mass, com=com, inertia=inertia)


# --------------------------------------------------------------------------- #
# 动力学模型
# --------------------------------------------------------------------------- #


class DynamicsModel:
    """基于主链的刚体动力学模型。

    Args:
        kin: :class:`~domain.kinematics.RobotKinematics` 实例；
            构造时自动从其 URDF 解析各连杆 ``<inertial>``

    Example::

        model = DynamicsModel(robot.kinematics)
        model.set_payload(3.0)               # 末端 3 kg 负载
        tau = model.inverse_dynamics(q, qd, qdd)   # N·m
    """

    def __init__(self, kin):
        self.kin = kin
        self.urdf_path = Path(kin.urdf_path)
        #: 主链关节序列（含 fixed）
        self.chain = kin.chain
        #: 可驱动关节在主链中的下标 -> dof 索引
        self._actuated_idx = [i for i, j in enumerate(self.chain) if j.movable]
        #: 主链各连杆的惯性（key = link 名，不含根连杆——基座固定不产生力矩）
        self.inertials: Dict[str, LinkInertial] = {}
        self._parse_urdf_inertials()
        #: 末端负载（kg），以点质量形式合并到末端连杆
        self.payload_mass: float = 0.0
        #: 手动录入标记（连杆名 -> 来源），用于诊断信息
        self.manual_sources: Dict[str, str] = {}

    # --------------------------- 参数管理 --------------------------- #

    def _parse_urdf_inertials(self) -> None:
        """从 URDF 解析主链各连杆的 ``<inertial>``。"""
        try:
            root = ET.parse(self.urdf_path).getroot()
        except (OSError, ET.ParseError):
            return
        by_name = {el.get("name", ""): el for el in root.findall("link")}
        for joint in self.chain:
            link = joint.child
            el = by_name.get(link)
            if el is None:
                continue
            inertial = el.find("inertial")
            if inertial is None:
                continue
            parsed = _parse_inertial_el(inertial)
            if parsed is not None:
                self.inertials[link] = parsed

    def set_link_inertial(
        self,
        link_name: str,
        mass: float,
        com: Sequence[float] = (0.0, 0.0, 0.0),
        inertia: Optional[np.ndarray] = None,
        source: str = "manual",
    ) -> None:
        """手动录入一个连杆的惯性参数（手册/估算轨道）。

        Args:
            link_name: 连杆名（主链上的 child link，可用
                :attr:`missing_links` 查缺哪些）
            mass: 质量 (kg)
            com: 质心，连杆坐标系 (m)
            inertia: 关于质心的 3×3 惯性张量 (kg·m²)；``None`` 按点质量处理
            source: 数据来源备注，仅用于诊断信息
        """
        if inertia is None:
            inertial = LinkInertial.point_mass(mass, com)
        else:
            inertial = LinkInertial(mass=mass, com=com, inertia=inertia)
        self.inertials[link_name] = inertial
        self.manual_sources[link_name] = source

    def set_payload(self, mass: float) -> None:
        """设置末端负载质量 (kg)，以点质量形式落在末端连杆原点（TCP 处）。"""
        self.payload_mass = max(0.0, float(mass))

    @property
    def missing_links(self) -> List[str]:
        """主链上还缺惯性参数的连杆名（根连杆与 fixed 关节末端除外）。

        可动关节的子连杆必须有惯性参数才能算准力矩；fixed 关节连接的
        末端 link（如 ``tcp``/``tool0``）缺省按零质量处理——URDF 导出
        常不带这类虚拟连杆的惯性，强行判缺会把整个校验卡死。
        """
        return [j.child for j in self.chain if j.movable and j.child not in self.inertials]

    @property
    def has_inertial(self) -> bool:
        """主链所有连杆是否都有惯性参数（力矩校验的前提）。"""
        return not self.missing_links

    @property
    def torque_limits(self) -> np.ndarray:
        """各可驱动关节的额定力矩 (N·m；移动关节 N)。

        优先取 URDF ``<limit effort>``，缺失时用 :data:`DEFAULT_JOINT_EFFORT`。
        """
        limits = []
        for j in self.kin.actuated:
            limits.append(j.max_effort if j.max_effort > 0 else DEFAULT_JOINT_EFFORT)
        return np.array(limits, dtype=np.float64)

    def _effective_inertials(self) -> Dict[str, LinkInertial]:
        """叠加末端负载后的连杆惯性表。"""
        result = dict(self.inertials)
        if self.payload_mass > 0.0 and self.chain:
            tip = self.chain[-1].child
            payload = LinkInertial.point_mass(self.payload_mass, np.zeros(3))
            base = result.get(tip, LinkInertial(0.0, np.zeros(3), np.zeros((3, 3))))
            result[tip] = base.merged_with(payload)
        return result

    def content_signature(self) -> bytes:
        """动力学模型的内容签名（v1.2.3，结果缓存键用）。

        覆盖**全部影响力矩结果的模型输入**：主链各关节的名称/原点/转轴/
        可动性、各连杆惯性（质量/质心/张量，逐字节）、末端负载、额定力矩。
        任何一项变了签名就变——``set_payload`` / ``set_link_inertial``
        这类运行期改动天然被感知，不存在"改了负载还用旧力矩表"。
        """
        h = hashlib.sha256()
        h.update(b"core-dynamics-v1")
        for joint in self.chain:
            h.update(str(getattr(joint, "name", "")).encode("utf-8"))
            h.update(b"\x01")
            for attr in ("origin_xyz", "origin_rpy", "axis"):
                arr = np.ascontiguousarray(
                    np.asarray(getattr(joint, attr, ()), dtype=np.float64).ravel()
                )
                h.update(arr.tobytes())
            h.update(b"\x01")
            h.update(b"p" if getattr(joint, "is_prismatic", False) else b"r")
            h.update(b"\x01")
        h.update(b"\x02")
        for link in sorted(self.inertials):
            inertial = self.inertials[link]
            h.update(link.encode("utf-8"))
            h.update(b"\x03")
            h.update(repr(float(inertial.mass)).encode("ascii"))
            h.update(np.ascontiguousarray(inertial.com, dtype=np.float64).tobytes())
            h.update(np.ascontiguousarray(inertial.inertia, dtype=np.float64).tobytes())
            h.update(b"\x03")
        h.update(b"\x02")
        h.update(repr(float(self.payload_mass)).encode("ascii"))
        h.update(np.ascontiguousarray(self.torque_limits, dtype=np.float64).tobytes())
        return h.digest()

    # --------------------------- 逆动力学 --------------------------- #

    def inverse_dynamics(
        self,
        q: Sequence[float],
        qd: Sequence[float],
        qdd: Sequence[float],
        gravity: Optional[Sequence[float]] = None,
    ) -> np.ndarray:
        """牛顿-欧拉逆动力学（世界系递推）。

        Args:
            q: 关节位置（弧度 / 米）
            qd: 关节速度（rad/s / m/s）
            qdd: 关节加速度（rad/s² / m/s²）
            gravity: 重力向量（基坐标系）；``None`` 用 :data:`GRAVITY`，
                传 ``(0, 0, 0)`` 可得到不含重力的力矩（质量矩阵列）

        Returns:
            各可驱动关节的驱动力矩 (N·m；移动关节 N)，顺序同
            :attr:`~domain.kinematics.RobotKinematics.actuated`
        """
        g = GRAVITY if gravity is None else np.asarray(gravity, dtype=np.float64)
        q = np.asarray(q, dtype=np.float64).ravel()
        qd = np.asarray(qd, dtype=np.float64).ravel()
        qdd = np.asarray(qdd, dtype=np.float64).ravel()

        transforms = self.kin.link_transforms(q)
        inertials = self._effective_inertials()

        # ---------- 正向递推：各连杆的速度与加速度 ----------
        # 世界系下逐链传播；基座固定，其加速度取 −g（重力等效技巧，
        # 使 F = m·a 自然含重力项）
        omega: Dict[str, np.ndarray] = {}
        alpha: Dict[str, np.ndarray] = {}
        acc: Dict[str, np.ndarray] = {}
        forces: Dict[str, np.ndarray] = {}  # 质心惯性力 F = m·a_c
        moments: Dict[str, np.ndarray] = {}  # 关于质心的惯性力矩
        origins: Dict[str, np.ndarray] = {}
        coms: Dict[str, np.ndarray] = {}  # 质心的世界坐标

        omega[self.chain[0].parent] = np.zeros(3)
        alpha[self.chain[0].parent] = np.zeros(3)
        acc[self.chain[0].parent] = -g
        origins[self.chain[0].parent] = transforms[self.chain[0].parent][:3, 3].copy()

        qi = 0
        for joint in self.chain:
            T_i = transforms[joint.child]
            T_p = transforms[joint.parent]
            o_i = T_i[:3, 3].copy()
            o_p = origins[joint.parent]
            r = o_i - o_p
            w_p = omega[joint.parent]
            a_p = alpha[joint.parent]
            v_p = acc[joint.parent]
            origins[joint.child] = o_i

            if joint.movable:
                # 轴向：关节坐标系 = 子连杆坐标系，轴在父系下表达为
                # R_p·R_origin；绕轴旋转不改变轴向，故直接取 R_origin 后的轴
                axis_w = T_p[:3, :3] @ _origin_rotation(joint) @ joint.axis
                qd_i = float(qd[qi])
                qdd_i = float(qdd[qi])
                if joint.is_prismatic:
                    omega[joint.child] = w_p
                    alpha[joint.child] = a_p
                    acc[joint.child] = (
                        v_p
                        + np.cross(a_p, r)
                        + w_p @ _skew2(w_p) @ r
                        + 2.0 * np.cross(w_p, qd_i * axis_w)
                        + qdd_i * axis_w
                    )
                else:
                    omega[joint.child] = w_p + qd_i * axis_w
                    alpha[joint.child] = a_p + np.cross(w_p, qd_i * axis_w) + qdd_i * axis_w
                    acc[joint.child] = v_p + np.cross(a_p, r) + w_p @ _skew2(w_p) @ r
                qi += 1
            else:
                omega[joint.child] = w_p
                alpha[joint.child] = a_p
                acc[joint.child] = v_p + np.cross(a_p, r) + w_p @ _skew2(w_p) @ r

            inertial = inertials.get(joint.child)
            if inertial is not None and inertial.mass > 0.0:
                R_i = T_i[:3, :3]
                c_world = o_i + R_i @ inertial.com
                e = c_world - o_i
                w_i = omega[joint.child]
                a_i = alpha[joint.child]
                a_c = acc[joint.child] + np.cross(a_i, e) + w_i @ _skew2(w_i) @ e
                forces[joint.child] = inertial.mass * a_c
                I_world = R_i @ inertial.inertia @ R_i.T
                moments[joint.child] = I_world @ a_i + np.cross(w_i, I_world @ w_i)
                coms[joint.child] = c_world
            else:
                forces[joint.child] = np.zeros(3)
                moments[joint.child] = np.zeros(3)
                coms[joint.child] = o_i

        # ---------- 反向递推：从末端往根部累加内力 ----------
        # 力矩基准点取各连杆原点：惯性力作用在质心，先折算 (c−o)×F
        net_f = {link: forces[link].copy() for link in forces}
        net_n = {
            link: moments[link] + np.cross(coms[link] - origins[link], forces[link])
            for link in forces
        }

        torques = np.zeros(self.kin.dof)
        # 反向遍历：链上第 k 个关节的子连杆把力传给第 k-1 个关节的父连杆
        for k in range(len(self.chain) - 1, -1, -1):
            joint = self.chain[k]
            child = joint.child
            f = net_f[child]
            n = net_n[child]
            if joint.movable:
                T_p = transforms[joint.parent]
                T_i = transforms[child]
                axis_w = T_p[:3, :3] @ _origin_rotation(joint) @ joint.axis
                idx = self._actuated_idx.index(k)
                if joint.is_prismatic:
                    torques[idx] = float(axis_w @ f)
                else:
                    torques[idx] = float(axis_w @ n)
            # 把该连杆的合力/合力矩（关于其原点）折算到父连杆原点
            o_child = transforms[child][:3, 3]
            o_parent = transforms[joint.parent][:3, 3]
            parent_link = joint.parent
            if parent_link in net_f:
                net_f[parent_link] = net_f[parent_link] + f
                net_n[parent_link] = net_n[parent_link] + n + np.cross(o_child - o_parent, f)

        return torques

    # --------------------------- 轨迹校验 --------------------------- #

    # 关节值（deg/mm）→ SI（rad/m）的换算系数，按轴类型
    def _si_scales(self) -> Tuple[np.ndarray, np.ndarray]:
        """返回 (位置换算系数, 速度换算系数)：deg→rad 或 mm→m。"""
        pos = np.array(
            [math.radians(1.0) if not j.is_prismatic else 1e-3 for j in self.kin.actuated]
        )
        return pos, pos  # 速度/加速度同为线性换算

    def joint_torques_along(self, traj) -> Tuple[np.ndarray, np.ndarray]:
        """沿轨迹逐路点计算关节力矩与机械功率。

        Args:
            traj: :class:`~service.simulation.Trajectory`（路点的
                velocity/acceleration 需已由规划器填充，单位 deg/s、deg/s²）

        Returns:
            ``(torques, powers)``：形状 ``(n_waypoints, dof)`` 的力矩 (N·m)
            与 ``(n_waypoints,)`` 的总机械功率 (W，取各关节功率绝对值之和)
        """
        scale, _ = self._si_scales()
        n = len(traj.waypoints)
        dof = self.kin.dof
        torques = np.zeros((n, dof))
        powers = np.zeros(n)
        for i, wp in enumerate(traj.waypoints):
            q = np.asarray(wp.joints, dtype=np.float64).ravel() * scale
            qd = np.asarray(wp.velocity, dtype=np.float64).ravel() * scale
            qdd = np.asarray(wp.acceleration, dtype=np.float64).ravel() * scale
            tau = self.inverse_dynamics(q, qd, qdd)
            torques[i] = tau
            powers[i] = float(np.sum(np.abs(tau * qd)))
        return torques, powers

    def check_trajectory(
        self,
        traj,
        motion_at=None,
        warn_ratio: float = TORQUE_WARN_RATIO,
    ) -> dict:
        """沿轨迹做力矩校验，输出报表数据与超限定位。

        Args:
            traj: 规划器展开的轨迹
            motion_at: 时刻 → 指令序号的回调（规划器提供），用于把超限
                定位回具体指令；``None`` 时统一记到指令 0
            warn_ratio: 峰值达到额定的该比例给黄色预警

        Returns:
            字典::

                {
                    "torque_peaks":  [每关节峰值 |τ| (N·m)],
                    "torque_limits": [每关节额定值 (N·m)],
                    "peak_power":    全程最大机械功率 (W),
                    "avg_power":     平均机械功率 (W),
                    "overloads":     [(motion_index, joint, ratio, torque, time)],
                    "near_limits":   [(motion_index, joint, ratio, torque, time)],
                }
        """
        torques, powers = self.joint_torques_along(traj)
        limits = self.torque_limits
        peaks = np.max(np.abs(torques), axis=0) if len(torques) else np.zeros(self.kin.dof)

        overloads: List[Tuple[int, int, float, float, Optional[float]]] = []
        near_limits: List[Tuple[int, int, float, float, Optional[float]]] = []
        ts = [wp.timestamp for wp in traj.waypoints]

        def _locate(i: int) -> Tuple[int, Optional[float]]:
            if motion_at is None:
                return 0, None
            t = ts[i] if i < len(ts) else 0.0
            return motion_at(t), t

        # 每个关节只报一次最坏越限，避免刷屏
        reported: set = set()
        for j in range(self.kin.dof):
            if limits[j] <= 0:
                continue
            ratios = np.abs(torques[:, j]) / limits[j]
            i_worst = int(np.argmax(ratios))
            ratio = float(ratios[i_worst])
            if ratio > 1.0:
                seg, t = _locate(i_worst)
                overloads.append((seg, j, ratio, float(torques[i_worst, j]), t))
                reported.add(j)
            elif ratio >= warn_ratio:
                seg, t = _locate(i_worst)
                near_limits.append((seg, j, ratio, float(torques[i_worst, j]), t))
                reported.add(j)

        return {
            "torque_peaks": peaks.tolist(),
            "torque_limits": limits.tolist(),
            "peak_power": float(np.max(powers)) if len(powers) else 0.0,
            "avg_power": float(np.mean(powers)) if len(powers) else 0.0,
            "overloads": overloads,
            "near_limits": near_limits,
        }


# --------------------------------------------------------------------------- #
# 力矩校验结果缓存键（v1.2.3）
# --------------------------------------------------------------------------- #


def dynamics_cache_key(
    model: DynamicsModel, traj: "Trajectory", spans: Sequence[Tuple[float, float]]
) -> bytes:
    """沿轨迹力矩校验的**内容寻址键**（v1.2.3，供结果缓存使用）。

    键由全部影响结果的输入派生：

    * 模型内容签名（:meth:`DynamicsModel.content_signature`）——连杆惯性、
      末端负载、额定力矩任一变动即换键；
    * 轨迹逐路点的关节位置/速度/加速度（逆动力学的全部输入）；
    * 指令时间区间 ``spans``——超限定位的 ``motion_index`` 由它决定，
      进键可防"换了规划器状态还复用旧定位"。

    Returns:
        32 字节 SHA256 摘要
    """
    h = hashlib.sha256()
    h.update(b"core-dynamics-check-v1")
    h.update(model.content_signature())
    h.update(b"\x00")
    for t0, t1 in spans:
        h.update(repr(float(t0)).encode("ascii"))
        h.update(b"\x01")
        h.update(repr(float(t1)).encode("ascii"))
        h.update(b"\x02")
    h.update(b"\x00")
    wps = traj.waypoints
    h.update(len(wps).to_bytes(8, "little"))
    for wp in wps:
        for attr in ("joints", "velocity", "acceleration"):
            value = getattr(wp, attr, None)
            if value is None:
                h.update(b"N")
            else:
                h.update(
                    np.ascontiguousarray(np.asarray(value, dtype=np.float64).ravel()).tobytes()
                )
    return h.digest()


# --------------------------------------------------------------------------- #
# 数学小工具
# --------------------------------------------------------------------------- #


def _origin_rotation(joint) -> np.ndarray:
    """关节原点的 rpy 旋转矩阵（缓存的惰性计算，供轴向变换用）。"""
    cached = getattr(joint, "_origin_rotation", None)
    if cached is None:
        from domain.kinematics import _rotmat_rpy

        cached = _rotmat_rpy(joint.origin_rpy)
        try:
            object.__setattr__(joint, "_origin_rotation", cached)
        except AttributeError:
            pass  # 不可缓存就直接每次算
    return cached


def _skew2(v: np.ndarray) -> np.ndarray:
    """向量的叉乘矩阵 [v]×，使 [v]×·w = v × w。"""
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
