"""纯 Python 机器人运动学（正解 FK / 逆解 IK / 雅可比）。

本模块不依赖 C++ 扩展（``orolp._core``）。它直接从 URDF 构建运动链，
因此只要装了 numpy + scipy 就能用——无需编译 Pinocchio。

为什么要自己写：
    C++ 核心 ``src/core`` 依赖 Pinocchio / Eigen / pybind11，仓库里默认并
    未编译，``orolp._core`` 为 ``None``。历史上 ``robot.py`` 因此退化为占位
    实现（FK 忽略关节角、IK 返回 guess 或全零），导致后处理链路上每一个
    目标位姿都是 ``0,0,0,0,0,0``。此处把"占位的降级路径"补成**真正可用
    的降级实现**；若将来编译了 C++ 核心，上层仍会优先使用真货。

单位约定（重要）:
    * 模块内部一律使用 **米** 与 **弧度**（与 URDF 一致）
    * :class:`~orolp.pose.Pose` 对外使用 **毫米** 与 **度**，转换只发生在
      :mod:`orolp.robot` 的边界处，本模块不做隐式换算

支持的 URDF 子集:
    * 关节类型：``revolute`` / ``continuous`` / ``prismatic`` / ``fixed``
    * 原点：``xyz`` + ``rpy``（完整支持，不像 GUI 渲染器那样限制 rpy=0）
    * 末端：优先取名为 ``tcp`` / ``tool0`` 的 link，否则取链的最末端
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

__all__ = [
    "KinematicsError",
    "IKError",
    "JointSpec",
    "RobotKinematics",
]


# --------------------------------------------------------------------------- #
# 异常
# --------------------------------------------------------------------------- #


class KinematicsError(Exception):
    """运动学计算的通用错误（URDF 结构非法、链断裂等）。"""


class IKError(KinematicsError):
    """逆运动学未能在给定迭代次数内收敛。"""


# --------------------------------------------------------------------------- #
# URDF 结构
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class JointSpec:
    """URDF 中的一个关节。

    Attributes:
        name: 关节名
        parent: 父 link 名
        child: 子 link 名
        origin_xyz: 关节原点相对父 link 的平移 (m)
        origin_rpy: 关节原点相对父 link 的旋转 (rad, 固定轴 XYZ 序)
        axis: 转轴（关节坐标系下的单位向量）
        joint_type: ``revolute`` / ``continuous`` / ``prismatic`` / ``fixed``
        lower: 下限位（旋转关节为 rad，移动关节为 m）
        upper: 上限位
        max_velocity: 最大速度（rad/s 或 m/s）
        max_effort: 最大力矩/力
    """

    name: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray
    joint_type: str
    lower: float
    upper: float
    max_velocity: float
    max_effort: float

    @property
    def movable(self) -> bool:
        """是否为可驱动关节（fixed 不可驱动）。"""
        return self.joint_type in ("revolute", "continuous", "prismatic")

    @property
    def is_prismatic(self) -> bool:
        """是否为移动关节。"""
        return self.joint_type == "prismatic"


def _parse_vec3(text: Optional[str], default: Sequence[float]) -> np.ndarray:
    """解析空白/逗号分隔的浮点串，兼容 3 分量 (xyz) 与 4 分量 (rgba)。"""
    parts = (text or "").replace(",", " ").split()
    if len(parts) < len(default):
        return np.array(default, dtype=np.float64)
    return np.array([float(p) for p in parts[: len(default)]], dtype=np.float64)


# --------------------------------------------------------------------------- #
# 变换工具
# --------------------------------------------------------------------------- #


def _rotmat_rpy(rpy: np.ndarray) -> np.ndarray:
    """URDF 的 ``rpy``（固定轴 / 外旋，X→Y→Z 依次绕**世界**轴）转 3×3 旋转矩阵。

    数学上等于 ``Rz(yaw) · Ry(pitch) · Rx(roll)``。

    scipy 的大小写语义是这里唯一的坑：

    * ``"XYZ"``（大写）= **内旋**，绕运动轴，得到 ``Rx·Ry·Rz``
    * ``"xyz"``（小写）= **外旋**，绕固定轴，得到 ``Rz·Ry·Rx`` ← URDF 要的是这个

    URDF 规范明确 rpy 是固定轴（fixed axis）旋转，因此必须用小写。单轴旋转
    （如 ``(0, π/2, 0)``）下两者恰好等价，所以这个错误在只含单轴 rpy 的
    程序化模型上不会暴露——但真实 CAD 导出的 URDF 几乎都带完整三轴 rpy，
    一旦出现，正逆解的末端位姿会明显偏离真值。

    Args:
        rpy: ``(roll, pitch, yaw)``，弧度。

    Returns:
        3×3 旋转矩阵。
    """
    return Rotation.from_euler("xyz", rpy).as_matrix()


def _transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """由 3x3 旋转与 3 维平移组装 4x4 齐次矩阵。"""
    m = np.eye(4)
    m[:3, :3] = R
    m[:3, 3] = t
    return m


def _rot_about_axis(axis: np.ndarray, angle: float) -> np.ndarray:
    """绕任意轴旋转的 3x3 矩阵（Rodrigues 公式）。"""
    n = float(np.linalg.norm(axis))
    if n < 1e-12 or abs(angle) < 1e-15:
        return np.eye(3)
    k = axis / n
    K = np.array(
        [[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]], dtype=np.float64
    )
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


# --------------------------------------------------------------------------- #
# 运动学主体
# --------------------------------------------------------------------------- #

#: 末端 link 的优先名称，按顺序尝试
_TIP_CANDIDATES = ("tcp", "tool0", "tool", "flange", "ee_link")


class RobotKinematics:
    """从 URDF 构建的运动链，提供 FK / 雅可比 / IK。

    Args:
        urdf_path: URDF 文件路径

    Raises:
        KinematicsError: URDF 无法解析，或找不到根 link / 末端 link

    Example:
        >>> kin = RobotKinematics("models/kuka/kr16_2/kr16_2.urdf")
        >>> q = np.radians([0, -90, 90, 0, 0, 0])
        >>> T = kin.forward_kinematics(q)          # 4x4, 单位米
        >>> q_ik = kin.inverse_kinematics(T)       # 反解，单位弧度
    """

    def __init__(self, urdf_path: str | Path):
        self.urdf_path = Path(urdf_path)
        self.joints: List[JointSpec] = []
        self._links: List[str] = []
        self._root: str = ""
        self._tip: str = ""
        self._children: Dict[str, List[JointSpec]] = {}

        self._parse()
        self._chain: List[JointSpec] = self._build_chain()
        self._actuated: List[JointSpec] = [j for j in self._chain if j.movable]

        if not self._actuated:
            raise KinematicsError(f"{self.urdf_path}: URDF 中没有可驱动关节")

    # --------------------------- 解析 --------------------------- #

    def _parse(self) -> None:
        """解析 URDF，填充 joints / links / root / tip。"""
        try:
            root = ET.parse(self.urdf_path).getroot()
        except (OSError, ET.ParseError) as exc:
            raise KinematicsError(f"无法解析 URDF {self.urdf_path}: {exc}") from exc

        self._links = [el.get("name", "") for el in root.findall("link")]
        self._links = [n for n in self._links if n]

        children: set[str] = set()
        for joint_el in root.findall("joint"):
            parent_el = joint_el.find("parent")
            child_el = joint_el.find("child")
            if parent_el is None or child_el is None:
                continue

            origin_el = joint_el.find("origin")
            xyz = _parse_vec3(origin_el.get("xyz") if origin_el is not None else None, (0.0, 0.0, 0.0))
            rpy = _parse_vec3(origin_el.get("rpy") if origin_el is not None else None, (0.0, 0.0, 0.0))
            axis_el = joint_el.find("axis")
            axis = _parse_vec3(axis_el.get("xyz") if axis_el is not None else None, (0.0, 0.0, 1.0))
            if np.linalg.norm(axis) < 1e-12:
                axis = np.array([0.0, 0.0, 1.0])

            limit_el = joint_el.find("limit")
            jtype = joint_el.get("type", "fixed")
            if limit_el is not None:
                lower = float(limit_el.get("lower", -math.pi))
                upper = float(limit_el.get("upper", math.pi))
                vel = float(limit_el.get("velocity", 1.0))
                effort = float(limit_el.get("effort", 0.0))
            else:
                # continuous / fixed 常常不带 limit
                lower, upper = (-math.inf, math.inf) if jtype == "continuous" else (0.0, 0.0)
                vel, effort = 1.0, 0.0

            spec = JointSpec(
                name=joint_el.get("name", ""),
                parent=parent_el.get("link", ""),
                child=child_el.get("link", ""),
                origin_xyz=xyz,
                origin_rpy=rpy,
                axis=axis / np.linalg.norm(axis),
                joint_type=jtype,
                lower=lower,
                upper=upper,
                max_velocity=vel,
                max_effort=effort,
            )
            self.joints.append(spec)
            children.add(spec.child)
            self._children.setdefault(spec.parent, []).append(spec)

        roots = [n for n in self._links if n not in children]
        if not roots:
            raise KinematicsError(f"{self.urdf_path}: 找不到根 link（存在环或全部是子节点）")
        self._root = "base_link" if "base_link" in roots else roots[0]
        self._tip = self._select_tip(children)

    def _select_tip(self, children: set[str]) -> str:
        """挑选末端 link：优先约定名，其次取链的叶子节点。"""
        for name in _TIP_CANDIDATES:
            if name in self._links and name not in self._children:
                return name
        leaves = [n for n in self._links if n not in self._children]
        if not leaves:
            raise KinematicsError(f"{self.urdf_path}: 找不到末端 link")
        # 叶子里挑"名字最像末端"的，其次取第一个
        for name in _TIP_CANDIDATES:
            if name in leaves:
                return name
        return leaves[0]

    def _build_chain(self) -> List[JointSpec]:
        """从根到末端走一条主链（忽略分支）。"""
        chain: List[JointSpec] = []
        current = self._root
        visited = {current}
        while True:
            kids = self._children.get(current, [])
            # 优先走能通向约定末端名的分支
            nxt = self._pick_next(kids)
            if nxt is None:
                break
            chain.append(nxt)
            if nxt.child in visited:
                break
            visited.add(nxt.child)
            current = nxt.child
        return chain

    def _pick_next(self, kids: List[JointSpec]) -> Optional[JointSpec]:
        """在一组子关节中选出朝末端前进的那一个。"""
        if not kids:
            return None
        for j in kids:
            if j.child == self._tip:
                return j
        for j in kids:
            if j.movable:
                return j
        return kids[0]

    # --------------------------- 基本属性 --------------------------- #

    @property
    def dof(self) -> int:
        """可驱动关节数（自由度）。"""
        return len(self._actuated)

    @property
    def joint_names(self) -> List[str]:
        """可驱动关节名，顺序与 FK/IK 的 q 一致。"""
        return [j.name for j in self._actuated]

    @property
    def tip_link(self) -> str:
        """末端 link 名。"""
        return self._tip

    @property
    def root_link(self) -> str:
        """根 link 名。"""
        return self._root

    @property
    def actuated(self) -> List[JointSpec]:
        """可驱动关节列表，顺序与 FK/IK 的 ``q`` 索引一致。

        对外暴露而非让调用方访问 ``_actuated``：轨迹规划需要逐轴读取
        ``joint_type`` 与 ``max_velocity``，只有拿到 :class:`JointSpec`
        才能正确区分旋转关节（rad/s）与移动关节（m/s）。
        """
        return list(self._actuated)

    def limits(self) -> Tuple[np.ndarray, np.ndarray]:
        """返回 ``(lower, upper)`` 限位数组，单位与关节类型一致（rad 或 m）。"""
        lower = np.array([j.lower for j in self._actuated], dtype=np.float64)
        upper = np.array([j.upper for j in self._actuated], dtype=np.float64)
        return lower, upper

    # --------------------------- 正运动学 --------------------------- #

    def link_transforms(
        self, q: Sequence[float], q_is_radians: bool = True
    ) -> Dict[str, np.ndarray]:
        """计算链上每个 link 的世界变换。

        Args:
            q: 关节值，长度须等于 :attr:`dof`
            q_is_radians: ``True`` 表示 q 为弧度（旋转关节）/ 米（移动关节）；
                设为 ``False`` 时旋转关节按度解释

        Returns:
            link 名 -> 4x4 齐次矩阵（米）

        Raises:
            ValueError: q 长度与自由度不符
        """
        q = np.asarray(q, dtype=np.float64).ravel()
        if q.size != self.dof:
            raise ValueError(f"期望 {self.dof} 个关节值，收到 {q.size} 个")

        transforms: Dict[str, np.ndarray] = {self._root: np.eye(4)}
        qi = 0
        for joint in self._chain:
            parent_T = transforms[joint.parent]
            # 关节原点（含 rpy）
            T = parent_T @ _transform(_rotmat_rpy(joint.origin_rpy), joint.origin_xyz)
            if joint.movable:
                value = float(q[qi])
                if joint.is_prismatic:
                    T = T @ _transform(np.eye(3), joint.axis * value)
                else:
                    angle = value if q_is_radians else math.radians(value)
                    T = T @ _transform(_rot_about_axis(joint.axis, angle), np.zeros(3))
                qi += 1
            transforms[joint.child] = T
        return transforms

    def all_link_transforms(
        self, q: Sequence[float], q_is_radians: bool = True
    ) -> Dict[str, np.ndarray]:
        """遍历**全部**关节（含分支与 fixed 关节），返回每个 link 的世界变换。

        与 :meth:`link_transforms` 的区别在于后者只沿主链走一趟，会丢掉分支。
        碰撞检测必须用这个方法：法兰上挂的焊枪、夹爪、TCP 标记都是 fixed
        关节连接的，属于主链之外的分支——丢了它们，机器人就"看不见"自己
        末端撞上了什么。

        Args:
            q: 关节值，长度须等于 :attr:`dof`
            q_is_radians: q 是否为弧度（旋转关节）

        Returns:
            link 名 -> 4x4 齐次矩阵（米）
        """
        q = np.asarray(q, dtype=np.float64).ravel()
        # 按关节名建立 q 的索引映射，确保与 :attr:`joint_names` 顺序一致，
        # 不受遍历顺序（含分支）影响
        q_index = {j.name: i for i, j in enumerate(self._actuated)}

        transforms: Dict[str, np.ndarray] = {self._root: np.eye(4)}
        pending = list(self.joints)
        while pending:
            remaining: List[JointSpec] = []
            progressed = False
            for joint in pending:
                parent_T = transforms.get(joint.parent)
                if parent_T is None:
                    remaining.append(joint)
                    continue
                T = parent_T @ _transform(_rotmat_rpy(joint.origin_rpy), joint.origin_xyz)
                idx = q_index.get(joint.name)
                if joint.movable and idx is not None:
                    value = float(q[idx]) if idx < q.size else 0.0
                    if joint.is_prismatic:
                        T = T @ _transform(np.eye(3), joint.axis * value)
                    else:
                        angle = value if q_is_radians else math.radians(value)
                        T = T @ _transform(_rot_about_axis(joint.axis, angle), np.zeros(3))
                transforms[joint.child] = T
                progressed = True
            pending = remaining
            if not progressed:
                break  # 剩余关节的父节点不可达（URDF 有环或断链），忽略
        return transforms

    def forward_kinematics(
        self, q: Sequence[float], q_is_radians: bool = True
    ) -> np.ndarray:
        """末端位姿的 4x4 齐次矩阵（**米**）。

        Args:
            q: 关节值
            q_is_radians: q 是否为弧度（旋转关节）

        Returns:
            4x4 齐次变换矩阵
        """
        return self.link_transforms(q, q_is_radians)[self._tip]

    def joint_origins(self, q: Sequence[float], q_is_radians: bool = True) -> List[np.ndarray]:
        """每个可驱动关节原点在世界系下的位置 (m)，供雅可比使用。"""
        transforms = self.link_transforms(q, q_is_radians)
        origins: List[np.ndarray] = []
        for joint in self._actuated:
            T = transforms[joint.parent] @ _transform(
                _rotmat_rpy(joint.origin_rpy), joint.origin_xyz
            )
            origins.append(T[:3, 3].copy())
        return origins

    def joint_axes(self, q: Sequence[float], q_is_radians: bool = True) -> List[np.ndarray]:
        """每个可驱动关节转轴在世界系下的方向（单位向量）。"""
        transforms = self.link_transforms(q, q_is_radians)
        axes: List[np.ndarray] = []
        for joint in self._actuated:
            T = transforms[joint.parent] @ _transform(
                _rotmat_rpy(joint.origin_rpy), joint.origin_xyz
            )
            axes.append(T[:3, :3] @ joint.axis)
        return axes

    # --------------------------- 雅可比 --------------------------- #

    def jacobian(self, q: Sequence[float], q_is_radians: bool = True) -> np.ndarray:
        """几何雅可比，形状 ``(6, dof)``。

        上三行为线速度分量，下三行为角速度分量，均在基坐标系下表达。
        移动关节的角速度分量为零，线速度分量即其轴向。

        Args:
            q: 关节值
            q_is_radians: q 是否为弧度

        Returns:
            ``(6, dof)`` 雅可比矩阵
        """
        T_tip = self.forward_kinematics(q, q_is_radians)
        p_tip = T_tip[:3, 3]
        origins = self.joint_origins(q, q_is_radians)
        axes = self.joint_axes(q, q_is_radians)

        J = np.zeros((6, self.dof), dtype=np.float64)
        for i, joint in enumerate(self._actuated):
            z = axes[i]
            if joint.is_prismatic:
                J[:3, i] = z
                J[3:, i] = 0.0
            else:
                J[:3, i] = np.cross(z, p_tip - origins[i])
                J[3:, i] = z
        return J

    # --------------------------- 逆运动学 --------------------------- #

    @staticmethod
    def _pose_error(T_cur: np.ndarray, T_goal: np.ndarray) -> np.ndarray:
        """位姿误差 ``[dx, dy, dz, drx, dry, drz]``（米 / 弧度）。

        姿态误差用旋转向量表示：``rotvec(R_goal · R_curᵀ)``，
        表达在世界（基）坐标系下，可直接与雅可比下半部分相乘。
        """
        dp = T_goal[:3, 3] - T_cur[:3, 3]
        R_err = T_goal[:3, :3] @ T_cur[:3, :3].T
        dr = Rotation.from_matrix(R_err).as_rotvec()
        return np.concatenate([dp, dr])

    def inverse_kinematics(
        self,
        target: np.ndarray,
        q_init: Optional[Sequence[float]] = None,
        max_iter: int = 200,
        pos_tol: float = 1e-5,
        rot_tol: float = 1e-4,
        damping: float = 1e-4,
        respect_limits: bool = True,
        seed: Optional[int] = 0,
        nullspace_gain: float = 0.05,
    ) -> np.ndarray:
        """阻尼最小二乘（DLS）迭代逆解。

        Args:
            target: 目标末端位姿 4x4 矩阵（**米**）
            q_init: 迭代初值（弧度）；为 ``None`` 时取限位中值，仍不行则取零位
            max_iter: 最大迭代次数
            pos_tol: 位置收敛阈值（米），默认 0.01 mm
            rot_tol: 姿态收敛阈值（弧度），默认约 0.006°
            damping: DLS 阻尼系数 λ，越大越稳但收敛越慢
            respect_limits: 是否把中间解夹回关节限位内
            seed: 随机种子；收敛失败时会用带种子的扰动重启。**默认固定为 0
                而非 None**——离线编程要求可复现：同一个程序今天和明年生成的
                关节解必须一致，否则现场调试无从下手。需要真正的随机重启时
                显式传 ``seed=None``。
            nullspace_gain: 零空间偏置增益。在不改变末端位姿（一阶意义）的前提
                下把关节往限位中值拉，避免解贴着限位边界。冗余臂（7 轴）收益
                最大；设为 ``0`` 关闭。

        Returns:
            关节值数组（**弧度**），长度 :attr:`dof`

        Raises:
            ValueError: target 不是 4x4 矩阵
            IKError: 在 ``max_iter`` 内未收敛（含单次迭代与多次重启）
        """
        target = np.asarray(target, dtype=np.float64)
        if target.shape != (4, 4):
            raise ValueError(f"target 必须是 4x4 矩阵，收到 {target.shape}")

        lower, upper = self.limits()
        rng = np.random.default_rng(seed)

        # 初值：显式初值 → 限位中值（±∞ 收敛到 0）→ 零位
        if q_init is not None:
            starts = [np.asarray(q_init, dtype=np.float64).ravel()]
        else:
            mid = (lower + upper) / 2.0
            mid = np.where(np.isfinite(mid), mid, 0.0)
            starts = [mid, np.zeros(self.dof)]
        # 追加若干随机重启，帮助跳出奇异位形
        for _ in range(4):
            if respect_limits and np.isfinite(lower).all() and np.isfinite(upper).all():
                starts.append(lower + rng.random(self.dof) * (upper - lower))
            else:
                starts.append(rng.uniform(-math.pi, math.pi, self.dof))

        # 只记录最佳残差用于失败时的诊断信息；不返回次优解——
        # 收敛失败就必须让调用方知道，而不是悄悄给出一个"差不多"的位形
        best_err = math.inf

        for start in starts:
            q = start.copy()
            if respect_limits:
                q = np.clip(q, lower, upper)

            for _ in range(max_iter):
                T_cur = self.forward_kinematics(q)
                err = self._pose_error(T_cur, target)
                pos_err = float(np.linalg.norm(err[:3]))
                rot_err = float(np.linalg.norm(err[3:]))
                score = pos_err + rot_err
                if score < best_err:
                    best_err = score
                if pos_err < pos_tol and rot_err < rot_tol:
                    return q

                J = self.jacobian(q)
                # DLS: dq = Jᵀ (J Jᵀ + λ²I)⁻¹ e
                A = J @ J.T + (damping**2) * np.eye(6)
                try:
                    J_dls = J.T @ np.linalg.inv(A)
                except np.linalg.LinAlgError:
                    J_dls = J.T @ np.linalg.pinv(A)
                dq = J_dls @ err

                # 零空间偏置：把关节往限位中值拉，避免解贴边（对冗余臂尤其有效）
                if nullspace_gain > 0.0:
                    mid = np.where(
                        np.isfinite(lower) & np.isfinite(upper),
                        (lower + upper) / 2.0,
                        0.0,
                    )
                    N = np.eye(self.dof) - J_dls @ J
                    dq = dq + nullspace_gain * (N @ (mid - q))

                # 限制单步步长，避免远离目标时震荡发散
                step = float(np.linalg.norm(dq))
                max_step = 0.25
                if step > max_step:
                    dq = dq * (max_step / step)

                q = q + dq
                if respect_limits:
                    q = np.clip(q, lower, upper)

        # 兜底：硬夹紧会破坏 DLS 的迭代路径——中间步被截断后下降方向失真，
        # 可能卡死在限位边界上（实测 6 轴臂约 6% 概率）。夹紧只是引导手段，
        # 解的合法性取决于最终值而非迭代路径，因此改用无约束求解：
        # 只要最终解落在限位内就接受。
        if respect_limits:
            try:
                q_free = self.inverse_kinematics(
                    target,
                    q_init=None,
                    max_iter=max_iter,
                    pos_tol=pos_tol,
                    rot_tol=rot_tol,
                    damping=damping,
                    respect_limits=False,
                    seed=seed,
                    nullspace_gain=0.0,
                )
            except IKError:
                q_free = None
            if q_free is not None:
                in_limits = bool(
                    np.all(q_free >= lower - 1e-9) and np.all(q_free <= upper + 1e-9)
                )
                if in_limits:
                    return q_free

        raise IKError(
            f"{self.urdf_path.name}: IK 在 {max_iter}×{len(starts)} 次迭代内未收敛 "
            f"（最佳残差 {best_err:.3e}，位置+姿态混合范数）"
        )

    def __repr__(self) -> str:
        return (
            f"RobotKinematics(name={self.urdf_path.stem!r}, dof={self.dof}, "
            f"root={self._root!r}, tip={self._tip!r})"
        )
