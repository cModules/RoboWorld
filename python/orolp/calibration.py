"""标定与手眼视觉（v0.3.4）——让仿真用上"真值"。

为什么要这个模块
----------------
离线编程的每一个数都建立在两个前提上：**TCP 长在法兰的什么位置**、
**机器人装在世界的什么位置**。这两个数在真机上得靠标定测出来，而现场交到
手上的通常只是一张标定报告（或者操作工拿示教器戳出来的几个点）。把这些数
手工抄进 URDF 的某个 joint、或者抄进程序，是现场最常见的一类低级错误——
抄错一个符号，整条程序姿态就偏了，而且现场往往到撞机才发现。

本模块把标定做成**可计算、可校验、可落盘**的一等能力：

* **TCP 4 点法**（:func:`calibrate_tcp_4point`）：工装尖端以 4 个以上不同
  姿态触碰同一固定点，最小二乘解出 TCP 相对法兰的偏移；同时给出**残差**。
  残差大就意味着"这几下碰的不是同一点"，标定不可信——必须让用户看见，
  而不是照单全收。
* **TCP 6 点法**（:func:`calibrate_tcp_6point`）：4 点只能定位置。焊枪/吸盘
  还要方向：再沿工具 +X / +Z 各触碰一点，把姿态一起解出来。
* **基坐标 3 点法与点对配准**（:func:`calibrate_base_3point` /
  :func:`calibrate_frame`，Kabsch-SVD）：定工件坐标系、用户坐标系在机器人
  基坐标系中的位姿。
* **手眼标定**（:func:`solve_hand_eye`）：eye-in-hand（相机装在法兰上）与
  eye-to-hand（相机固定在工位上）两种配置，解 ``A·X = X·B`` 得相机外参，
  并给残余误差。
* **相机模型与视觉抓取**（:class:`CameraIntrinsics` +
  :func:`grasp_pose_from_pixel`）：把"图像上的一个像素"经标定后的外参变成
  "机器人该去哪儿抓"——相机触发抓取的最后一环。
* **标定集**（:class:`CalibrationSet`）：把上述结果打包成一个 JSON；
  :meth:`CalibrationSet.apply_to_robot` 直接灌进仿真；
  :meth:`CalibrationSet.write_urdf` 把 TCP 写回 URDF（此后 FK 的末端就是
  标定后的 TCP，仿真与真机同值）；出码时标定摘要自动进程序头注释。

单位与记号
----------
对外接口一律 **毫米 + 度**（与 :class:`orolp.pose.Pose` 一致），只有相机
内参是无量纲像素。

``T_a_b`` 表示"**把 b 系的坐标变换到 a 系**"的 4x4 齐次矩阵，于是
``T_a_c = T_a_b · T_b_c``、``T_b_a = T_a_b⁻¹``。下文公式都按这个记号写：
``T_b_f`` 法兰在基座系、``T_b_c`` 相机在基座系、``T_c_t`` 标定板在相机系。
"""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from .pose import Pose

__all__ = [
    "CalibrationError",
    "TcpCalibration",
    "TouchProbe",
    "HandEyeResult",
    "HandEyeDataset",
    "CameraIntrinsics",
    "CalibrationSet",
    "TCP_RESIDUAL_TOL_MM",
    "rigid_transform",
    "calibrate_frame",
    "calibrate_base_3point",
    "calibrate_tcp_4point",
    "calibrate_tcp_6point",
    "calibrate_tcp_from_joints",
    "flange_poses_from_joints",
    "solve_hand_eye",
    "generate_hand_eye_dataset",
    "pixel_to_ray_in_base",
    "intersect_plane",
    "pixel_of_point",
    "grasp_pose_from_pixel",
]

#: TCP 4 点法残差经验阈值 (mm)。超过它意味着"四次触碰没落在同一点"，
#: 标定结果不可信。做成常量而非硬编码，方便按工装精度调整。
TCP_RESIDUAL_TOL_MM = 0.5

#: 手眼标定每段运动的**最小转角**（度）。转角太小的两帧姿态差几乎相同，
#: 解不出旋转轴，把这类运动排除比让它污染最小二乘更诚实。
DEFAULT_MIN_ROTATION_DEG = 5.0

#: 手眼方程病态判据：Kronecker 矩阵最小/最大奇异值之比的上限。合成数据实测
#: 1mm/0.2° 视觉噪声下约 6e-3，而"模式传错"这类结构性错误在 0.1 量级——
#: 这里取 0.03 作分界，噪声放行、结构错误拦下。
HAND_EYE_MAX_SV_RATIO = 0.03

#: 手眼解残差上限（相对运动幅度）。1mm/0.2° 噪声实测约 0.037，模式传错约 0.18。
HAND_EYE_MAX_RESIDUAL_RATIO = 0.10

#: 运动轴夹角下限（度）。全部运动都绕同一根轴转时，姿态的另外两个自由度根本
#: 没被约束，最小二乘会给出一个看似收敛、实则发散的解——必须在求解前拦下。
HAND_EYE_MIN_AXIS_ANGLE_DEG = 10.0

_EPS = 1e-9


# --------------------------------------------------------------------------- #
# 异常与内部工具
# --------------------------------------------------------------------------- #


class CalibrationError(RuntimeError):
    """标定无法完成（数据不足、位形退化、结果不可信）。"""


def _as_matrix(value: Any) -> np.ndarray:
    """把 :class:`Pose` / 4x4 矩阵 / 16 元展平数组统一成 4x4（毫米）。"""
    if isinstance(value, Pose):
        return value.matrix.copy()
    arr = np.asarray(value, dtype=np.float64)
    if arr.shape == (4, 4):
        return arr.copy()
    if arr.size == 16:
        return arr.reshape(4, 4)
    raise CalibrationError(f"无法识别的位姿表示，形状 {arr.shape}（期望 Pose 或 4x4）")


def _pose(matrix: np.ndarray) -> Pose:
    """4x4 → :class:`Pose`。"""
    return Pose(np.asarray(matrix, dtype=np.float64))


def _apply(matrix: np.ndarray, point: Sequence[float]) -> np.ndarray:
    """把一个点（mm）用 4x4 变换过去。"""
    p = np.asarray(point, dtype=np.float64).ravel()
    if p.size != 3:
        raise CalibrationError(f"点必须是 3 维，收到 {p.size} 维")
    return matrix[:3, :3] @ p + matrix[:3, 3]


def _rotation_deg(matrix: np.ndarray) -> float:
    """旋转矩阵对应的旋转角（度）。"""
    return float(np.degrees(np.linalg.norm(Rotation.from_matrix(matrix[:3, :3]).as_rotvec())))


def _average_transforms(mats: Sequence[np.ndarray]) -> np.ndarray:
    """多个变换的平均：平移取算术平均，旋转取四元数意义下的均值。

    手眼标定用它把"每帧各自推出的相机外参"合并成一个更稳的估计——单帧估计
    会被该帧的视觉噪声带偏，取均值能把随机噪声压下去。
    """
    stacked = np.stack([m[:3, :3] for m in mats])
    R = Rotation.from_matrix(stacked).mean().as_matrix()
    t = np.mean([m[:3, 3] for m in mats], axis=0)
    out = np.eye(4)
    out[:3, :3] = R
    out[:3, 3] = t
    return out


def _orthonormalize(R: np.ndarray) -> np.ndarray:
    """把近似旋转矩阵投影回 SO(3)（SVD，含镜像修正）。

    Kronecker 线性解出的 3x3 只是"最小二乘意义下最像旋转"的矩阵，行列式可能
    是 -1（镜像）。直接当旋转用会让姿态翻面，必须投影。
    """
    U, _, Vt = np.linalg.svd(R)
    d = np.sign(np.linalg.det(U @ Vt))
    if d == 0:
        d = 1.0
    return U @ np.diag([1.0, 1.0, d]) @ Vt


def _transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """由 3x3 旋转与 3 维平移组装 4x4。"""
    m = np.eye(4)
    m[:3, :3] = R
    m[:3, 3] = np.asarray(t, dtype=np.float64).ravel()
    return m


# --------------------------------------------------------------------------- #
# 刚体配准（基坐标 / 用户坐标系标定）
# --------------------------------------------------------------------------- #


def rigid_transform(
    src_points: Sequence[Sequence[float]],
    dst_points: Sequence[Sequence[float]],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Kabsch/SVD 刚体配准：求 ``dst ≈ R · src + t`` 的 ``(R, t, 残差)``。

    Args:
        src_points: 源点列（mm），至少 3 个
        dst_points: 目标点列（mm），与源点一一对应

    Returns:
        ``(R, t, residuals)``——3x3 旋转、3 维平移、每个点的残差 (mm)

    Raises:
        CalibrationError: 点数不足、数量不匹配，或源点共线（旋转面内不确定，
            此时任何绕该直线的解都等价，不能给出唯一答案）
    """
    src = np.asarray(src_points, dtype=np.float64).reshape(-1, 3)
    dst = np.asarray(dst_points, dtype=np.float64).reshape(-1, 3)
    if len(src) != len(dst):
        raise CalibrationError(f"点数不匹配：源 {len(src)} 个，目标 {len(dst)} 个")
    if len(src) < 3:
        raise CalibrationError(f"刚体配准至少需要 3 个点，收到 {len(src)} 个")

    c_src, c_dst = src.mean(axis=0), dst.mean(axis=0)
    H = (src - c_src).T @ (dst - c_dst)
    U, S, Vt = np.linalg.svd(H)
    if S[1] <= _EPS * max(S[0], _EPS):
        raise CalibrationError("源点共线（或重合），无法确定唯一的刚体变换")
    d = np.sign(np.linalg.det(Vt.T @ U.T)) or 1.0
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = c_dst - R @ c_src
    residuals = np.linalg.norm(src @ R.T + t - dst, axis=1)
    return R, t, residuals


def calibrate_frame(
    src_points: Sequence[Sequence[float]],
    dst_points: Sequence[Sequence[float]],
) -> Pose:
    """任意对应点对的坐标系标定：返回使 ``dst ≈ T · src`` 成立的位姿。

    典型用法：把 3 个以上点分别用示教器点在机器人基坐标系下、又在（图纸给的）
    世界坐标系下读出坐标，即可求出机器人基座相对世界系的位姿。

    Args:
        src_points: 源坐标系下的点 (mm)
        dst_points: 目标坐标系下的对应点 (mm)

    Returns:
        :class:`Pose`——``src`` 系在 ``dst`` 系中的位姿（mm + 度）
    """
    R, t, _ = rigid_transform(src_points, dst_points)
    return _pose(_transform(R, t))


def calibrate_base_3point(
    origin: Sequence[float],
    x_point: Sequence[float],
    y_point: Sequence[float],
) -> Pose:
    """基坐标 / 用户坐标系 3 点法标定。

    依次把工具尖点碰在：坐标系原点、X 轴正方向上的点、XY 平面第一象限内的点。
    由原点、X 向、XY 面内点构造正交基（Gram-Schmidt）：

    * ``X = normalize(x_point − origin)``
    * ``Z = normalize(X × (y_point − origin))`` —— 法向由 "X × Y 向" 给出
    * ``Y = Z × X``

    这样即便操作工点的 Y 向点不严格垂直于 X，得到的仍是正交右手系——真实现场
    没人能徒手点出直角，做正交化比直接用两个非正交向量更靠谱。

    Args:
        origin: 坐标系原点在机器人基坐标系中的位置 (mm)
        x_point: X 轴正方向上的点 (mm)，须与原点距离 > 0
        y_point: XY 平面内的点 (mm)，不得与原点、X 轴共线

    Returns:
        :class:`Pose`——该坐标系在机器人基坐标系中的位姿（mm + 度）

    Raises:
        CalibrationError: 点重合或三点共线（退化为无法定姿态）
    """
    o = np.asarray(origin, dtype=np.float64).ravel()
    ex = np.asarray(x_point, dtype=np.float64).ravel() - o
    ey = np.asarray(y_point, dtype=np.float64).ravel() - o
    for name, v in (("X", ex), ("Y", ey)):
        if np.linalg.norm(v) < _EPS:
            raise CalibrationError(f"{name} 向点与原点重合，无法确定坐标轴方向")
    ex = ex / np.linalg.norm(ex)
    ez = np.cross(ex, ey)
    if np.linalg.norm(ez) < _EPS:
        raise CalibrationError("三点共线：Y 向点落在 X 轴上，无法确定坐标系姿态")
    ez = ez / np.linalg.norm(ez)
    ey = np.cross(ez, ex)
    return _pose(_transform(np.column_stack([ex, ey, ez]), o))


# --------------------------------------------------------------------------- #
# TCP 标定
# --------------------------------------------------------------------------- #


@dataclass
class TcpCalibration:
    """TCP 标定结果。

    Attributes:
        offset: TCP 相对**法兰**的位姿（mm + 度）。4 点法只解位置（旋转为单位
            阵）；6 点法同时给出姿态
        reference_point: 标定基准点（工具尖触碰的那个固定点）在机器人基坐标系
            中的位置 (mm)。改天复查标定是否漂移，就看这个点是否还在原处
        residual_max_mm: 各次触碰反推出的尖端位置到基准点的最大距离 (mm)
        residual_rms_mm: 同上，均方根 (mm)
        n_points: 参与求解的触碰次数
        method: ``"4point"`` / ``"6point"``
    """

    offset: Pose
    reference_point: np.ndarray
    residual_max_mm: float = 0.0
    residual_rms_mm: float = 0.0
    n_points: int = 0
    method: str = "4point"

    @property
    def ok(self) -> bool:
        """残差是否在经验阈值内（仅作质量提示，不是硬性门禁）。"""
        return self.residual_max_mm <= TCP_RESIDUAL_TOL_MM

    def summary(self) -> str:
        """一句话摘要，用于日志/程序头注释。"""
        x, y, z, rx, ry, rz = self.offset.to_xyz_rpy()
        extra = " + 2 方向点" if self.method == "6point" else ""
        return (
            f"TCP({self.method}) 偏移 = ({x:.2f}, {y:.2f}, {z:.2f}) mm / "
            f"RPY ({rx:.2f}, {ry:.2f}, {rz:.2f})°，"
            f"残差 max {self.residual_max_mm:.3f} mm（{self.n_points} 点{extra}，"
            f"{'合格' if self.ok else '超差需复查'}）"
        )

    def to_dict(self) -> Dict[str, Any]:
        """序列化为可 JSON 化的字典。"""
        return {
            "offset": self.offset.matrix.flatten().tolist(),
            "reference_point": np.asarray(self.reference_point, float).tolist(),
            "residual_max_mm": float(self.residual_max_mm),
            "residual_rms_mm": float(self.residual_rms_mm),
            "n_points": int(self.n_points),
            "method": self.method,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TcpCalibration":
        """从 :meth:`to_dict` 的结果恢复。"""
        return cls(
            offset=_pose(np.array(data["offset"], dtype=np.float64).reshape(4, 4)),
            reference_point=np.array(data["reference_point"], dtype=np.float64).reshape(3),
            residual_max_mm=float(data.get("residual_max_mm", 0.0)),
            residual_rms_mm=float(data.get("residual_rms_mm", 0.0)),
            n_points=int(data.get("n_points", 0)),
            method=data.get("method", "4point"),
        )


@dataclass
class TouchProbe:
    """6 点法的一次"方向触碰"记录。

    Attributes:
        flange: 触碰时的**法兰位姿**（机器人基坐标系，mm + 度）——方向要换算到
            法兰系必须知道当时的姿态
        point: 工具尖在机器人基坐标系中的位置 (mm)
    """

    flange: Pose
    point: Sequence[float]


def _solve_tcp_offset(flange: Sequence[np.ndarray]) -> Tuple[np.ndarray, np.ndarray]:
    """4 点法核心：由 N 个法兰位姿解出 TCP 偏移与基准点。

    原理：4 次触碰的共同点是"工具尖始终落在同一个物理点 P"，于是
    ``R_i·p + t_i = P``。对 i 求均值后相减消掉未知的 P，得到关于 p 的线性
    方程组 ``(R_i − R̄)·p = −(t_i − t̄)``，最小二乘即可解出 ``p``。

    Returns:
        ``(p_tcp, tcp_points)``——法兰系中的 TCP 偏移与各次触碰反推出的尖端位置
    """
    mats = [_as_matrix(f) for f in flange]
    R = np.stack([m[:3, :3] for m in mats])
    t = np.stack([m[:3, 3] for m in mats])
    R_bar, t_bar = R.mean(axis=0), t.mean(axis=0)

    A = np.vstack([Ri - R_bar for Ri in R])
    b = np.concatenate([-(ti - t_bar) for ti in t])
    # 姿态若几乎一致（操作工没换姿态），A 秩亏，解不唯一——必须报错而不是
    # 给出一个"看起来有数"的结果
    sv = np.linalg.svd(A, compute_uv=False)
    if sv[0] <= _EPS or sv[-1] / sv[0] < 1e-6:
        raise CalibrationError(
            "法兰姿态差异不足（位形退化）：4 点法要求每次触碰换一个明显不同的姿态"
        )
    p_tcp, *_ = np.linalg.lstsq(A, b, rcond=None)
    tcp_points = np.stack([_apply(m, p_tcp) for m in mats])
    return p_tcp, tcp_points


def calibrate_tcp_4point(
    flange_poses: Sequence[Any],
    ref_point: Optional[Sequence[float]] = None,
) -> TcpCalibration:
    """TCP 4 点法标定（只定位置）。

    Args:
        flange_poses: 4 个以上**法兰位姿**（机器人基坐标系，mm + 度），每次
            姿态不同、工具尖都碰在同一个物理点上
        ref_point: 基准点真值 (mm)；``None`` 时用各次反推结果的平均——不传就
            只能看到"自洽性"，传了才能看出相对真值的系统偏差

    Returns:
        :class:`TcpCalibration`

    Raises:
        CalibrationError: 位姿少于 4 个、姿态差异不足

    Note:
        为什么输入是**法兰位姿**而不是"TCP 位姿"：TCP 正是要标定的未知量，
        拿它当输入是循环论证。真机上从控制器读到的是法兰位姿（或加了未标定
        TCP 的位姿），用 :func:`flange_poses_from_joints` 可以从关节角把
        示教器上那台机器人的法兰位姿直接算出来。
    """
    if len(flange_poses) < 4:
        raise CalibrationError(
            f"TCP 4 点法至少需要 4 个法兰位姿，收到 {len(flange_poses)} 个"
        )
    p_tcp, tcp_points = _solve_tcp_offset(flange_poses)
    ref = (
        np.asarray(ref_point, dtype=np.float64).ravel()
        if ref_point is not None
        else tcp_points.mean(axis=0)
    )
    dist = np.linalg.norm(tcp_points - ref, axis=1)
    return TcpCalibration(
        offset=_pose(_transform(np.eye(3), p_tcp)),
        reference_point=ref,
        residual_max_mm=float(dist.max()),
        residual_rms_mm=float(np.sqrt((dist**2).mean())),
        n_points=len(flange_poses),
        method="4point",
    )


def calibrate_tcp_6point(
    flange_poses: Sequence[Any],
    x_probe: TouchProbe,
    z_probe: TouchProbe,
    ref_point: Optional[Sequence[float]] = None,
) -> TcpCalibration:
    """TCP 6 点法标定（位置 + 姿态）。

    在 4 点法的基础上再沿工具 **+X** 与 **+Z** 各触碰一点。工具轴在基坐标系中
    的方向 = 触碰点相对基准点的位移方向；再左乘法兰姿态的转置即可换算到法兰系：

    ``工具轴(法兰系) = R_flange⁻¹ · normalize(触碰点 − 基准点)``

    两个方向做 Gram-Schmidt 正交化后补齐第三轴（``Y = Z × X``）得到右手系。
    操作要点：这两次触碰要**沿工具轴方向离开基准点一定距离**（几厘米即可），
    方向才稳定——距离太近时噪声会把方向带偏。

    Args:
        flange_poses: 4 点法的位姿数据（≥4 个）
        x_probe: 沿工具 +X 方向的触碰记录
        z_probe: 沿工具 +Z 方向的触碰记录
        ref_point: 基准点真值 (mm)，同 4 点法

    Returns:
        :class:`TcpCalibration`（``method="6point"``，``offset`` 含旋转）

    Raises:
        CalibrationError: 方向触碰与基准点重合，或两个方向几乎同向（无法定姿态）
    """
    base = calibrate_tcp_4point(flange_poses, ref_point=ref_point)
    ref = base.reference_point

    def axis(probe: TouchProbe, label: str) -> np.ndarray:
        delta = np.asarray(probe.point, dtype=np.float64).ravel() - ref
        n = float(np.linalg.norm(delta))
        if n < _EPS:
            raise CalibrationError(
                f"沿工具 {label} 的触碰点与基准点重合，无法确定方向（请沿轴移动数厘米再碰）"
            )
        R_f = _as_matrix(probe.flange)[:3, :3]
        return R_f.T @ (delta / n)

    u = axis(x_probe, "+X")
    w = axis(z_probe, "+Z")
    if abs(float(u @ w)) > 0.5:
        raise CalibrationError(
            f"两次方向触碰几乎同向（夹角余弦 {abs(float(u @ w)):.2f}），"
            f"无法定出工具姿态：+X 与 +Z 应近似垂直"
        )
    u = u / np.linalg.norm(u)
    w = w - (w @ u) * u
    w = w / np.linalg.norm(w)
    v = np.cross(w, u)  # 右手系：X × Y = Z
    R_tool = np.column_stack([u, v, w])
    p_tcp = base.offset.matrix[:3, 3]
    return TcpCalibration(
        offset=_pose(_transform(R_tool, p_tcp)),
        reference_point=ref,
        residual_max_mm=base.residual_max_mm,
        residual_rms_mm=base.residual_rms_mm,
        n_points=base.n_points,
        method="6point",
    )


def flange_poses_from_joints(
    robot: Any,
    joint_sets: Sequence[Sequence[float]],
) -> List[Pose]:
    """由关节角算出**法兰位姿**（机器人基坐标系）。

    真机上标定 TCP 时读到的是带未标定（或名义）工具的姿态；本函数用
    ``FK = base · flange · tool`` 反推法兰：``flange = base⁻¹ · FK · tool⁻¹``，
    因此机器人的 :attr:`Robot.base` / :attr:`Robot.tool` 会**按现状参与换算**：

    * 若 :attr:`Robot.tool` 还是名义值，得到的就是法兰位姿（标定 TCP 的正确输入）
    * 若已把标定值设进 :attr:`Robot.tool`，得到的是"去掉工具"的纯法兰位姿，
      可以用来复核标定（把标定结果代回去，尖端应回到同一个点）

    Args:
        robot: :class:`~orolp.robot.Robot`
        joint_sets: 关节角序列（度），每个元素长度 = 自由度

    Returns:
        :class:`~orolp.pose.Pose` 列表，单位 mm + 度
    """
    out: List[Pose] = []
    for joints in joint_sets:
        fk = robot.forward_kinematics(joints)
        out.append(robot.base.inv() * fk * robot.tool.inv())
    return out


def calibrate_tcp_from_joints(
    robot: Any,
    joint_sets: Sequence[Sequence[float]],
    ref_point: Optional[Sequence[float]] = None,
) -> TcpCalibration:
    """直接用手教出来的关节角做 TCP 4 点法（:func:`flange_poses_from_joints` 的糖）。"""
    return calibrate_tcp_4point(
        flange_poses_from_joints(robot, joint_sets), ref_point=ref_point
    )


# --------------------------------------------------------------------------- #
# 手眼标定（A·X = X·B）
# --------------------------------------------------------------------------- #


@dataclass
class HandEyeResult:
    """手眼标定结果。

    Attributes:
        mode: ``"eye_in_hand"``（相机装在法兰上）/ ``"eye_to_hand"``（相机固定）
        transform: 解出的未知量。eye-in-hand 为 ``T_flange_camera``（相机在法兰
            系中的位姿）；eye-to-hand 为 ``T_flange_marker``（标定板在法兰系中
            的位姿，即"工件/标定板怎么装在夹爪上"）
        camera_in_base: 相机在机器人**基坐标系**中的外参。eye-to-hand 下相机固定，
            这就是全工位通用的外参；eye-in-hand 下相机随法兰运动，此处给的是
            **首帧标定位姿下**的相机位姿，其他位形请用 :meth:`camera_pose_at`
        n_motions: 参与求解的运动段数（相邻位姿对）
        residual_mm: 方程 ``A·X = X·B`` 的最大平移残差 (mm)
        residual_deg: 同上，最大旋转残差（度）
    """

    mode: str
    transform: Pose
    camera_in_base: Pose
    n_motions: int = 0
    residual_mm: float = 0.0
    residual_deg: float = 0.0

    def camera_pose_at(self, flange_pose: Any) -> Pose:
        """给定位形下的相机位姿（视觉抓取要用的是这个）。

        * eye-in-hand：相机装在法兰上，``T_b_c = T_b_f · X``——换位形相机就跟着动，
          所以必须用**当前**法兰位姿算
        * eye-to-hand：相机固定在工位上，与位形无关，直接返回标定外参

        Args:
            flange_pose: 当前法兰位姿（机器人基坐标系）

        Returns:
            相机在基坐标系中的位姿
        """
        if self.mode == "eye_in_hand":
            return _pose(_as_matrix(flange_pose) @ self.transform.matrix)
        return self.camera_in_base

    def summary(self) -> str:
        """一句话摘要，用于日志/程序头注释。"""
        x, y, z, rx, ry, rz = self.camera_in_base.to_xyz_rpy()
        label = "eye-in-hand 首帧位姿下" if self.mode == "eye_in_hand" else "eye-to-hand"
        return (
            f"手眼标定({label}) 相机在基座系 = ({x:.1f}, {y:.1f}, {z:.1f}) mm / "
            f"RPY ({rx:.2f}, {ry:.2f}, {rz:.2f})°，{self.n_motions} 段运动，"
            f"残差 {self.residual_mm:.3f} mm / {self.residual_deg:.4f}°"
        )

    def to_dict(self) -> Dict[str, Any]:
        """序列化为可 JSON 化的字典。"""
        return {
            "mode": self.mode,
            "transform": self.transform.matrix.flatten().tolist(),
            "camera_in_base": self.camera_in_base.matrix.flatten().tolist(),
            "n_motions": int(self.n_motions),
            "residual_mm": float(self.residual_mm),
            "residual_deg": float(self.residual_deg),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "HandEyeResult":
        """从 :meth:`to_dict` 的结果恢复。"""
        return cls(
            mode=data.get("mode", "eye_in_hand"),
            transform=_pose(np.array(data["transform"], dtype=np.float64).reshape(4, 4)),
            camera_in_base=_pose(
                np.array(data["camera_in_base"], dtype=np.float64).reshape(4, 4)
            ),
            n_motions=int(data.get("n_motions", 0)),
            residual_mm=float(data.get("residual_mm", 0.0)),
            residual_deg=float(data.get("residual_deg", 0.0)),
        )


@dataclass
class HandEyeDataset:
    """一组手眼标定数据（真机采集或 :func:`generate_hand_eye_dataset` 仿真生成）。

    Attributes:
        mode: 配置模式
        flange_poses: 各次位姿的法兰位姿（基坐标系）
        camera_readings: 各次位姿的**相机读数**——eye-in-hand 为标定板在相机系
            中的位姿 ``T_cam_target``；eye-to-hand 为标定板（装在夹爪上）在相机
            系中的位姿 ``T_cam_marker``
        transform: 生成时使用的真值 ``X``（采集数据时为 ``None``）
        camera_in_base: 生成时使用的真值外参（采集数据时为 ``None``）
    """

    mode: str
    flange_poses: List[Pose]
    camera_readings: List[Pose]
    transform: Optional[Pose] = None
    camera_in_base: Optional[Pose] = None


def _vectorize(R: np.ndarray) -> np.ndarray:
    """按**列优先**把 3x3 矩阵拉直成 9 维列向量（Kronecker 恒等式的约定）。

    ``vec`` 的约定必须是列优先（Fortran 序）——因为 ``vec(A·X·B) = (Bᵀ⊗A)·vec(X)``
    这个恒等式只在列优先下成立。用 numpy 默认的行优先 ``reshape`` 会让解出来的
    9 维向量被按错误顺序还原成矩阵，症状是"标定结果看起来像随机数"。
    """
    return np.asarray(R, dtype=np.float64).reshape(9, order="F")


def _unvectorize(v: np.ndarray) -> np.ndarray:
    """:func:`_vectorize` 的逆操作（列优先）。"""
    return np.asarray(v, dtype=np.float64).reshape(3, 3, order="F")


def _check_axis_diversity(rotations: Sequence[np.ndarray]) -> None:
    """校验各段运动的旋转轴是否足够分散。

    手眼标定的可观性要求：运动旋转轴不能全部平行。若都绕同一根轴转，X 的姿态
    在另外两个方向上完全不受约束——最小二乘会给出一个残差看着不大、实际发散到
    天文数字的解（真机上表现为"标定结果完全不能用"）。这里提前拦住，
    并直接告诉用户该怎么改动作。

    Raises:
        CalibrationError: 所有有效运动的旋转轴两两夹角都小于阈值
    """
    axes = []
    for R in rotations:
        rotvec = Rotation.from_matrix(R).as_rotvec()
        n = float(np.linalg.norm(rotvec))
        if n > _EPS:
            axes.append(rotvec / n)
    if len(axes) < 2:
        raise CalibrationError("有效运动的旋转轴不足：至少需要两段绕不同方向的转动")
    best = 0.0
    for i in range(len(axes)):
        for j in range(i + 1, len(axes)):
            cos = float(np.clip(abs(axes[i] @ axes[j]), -1.0, 1.0))
            best = max(best, math.degrees(math.acos(cos)))
    if best < HAND_EYE_MIN_AXIS_ANGLE_DEG:
        raise CalibrationError(
            f"所有运动的旋转轴几乎平行（最大夹角 {best:.1f}° < "
            f"{HAND_EYE_MIN_AXIS_ANGLE_DEG:g}°）：标定姿态不可观测。"
            f"请让机器人**绕不同方向转动**（例如同时改变腕部两个以上关节），"
            f"只转一个关节是解不出外参的"
        )


def _solve_ax_xb(A_list: Sequence[np.ndarray], B_list: Sequence[np.ndarray]) -> np.ndarray:
    """求解 ``A·X = X·B`` 的 ``X``（Kronecker 线性化 + SVD 投影）。

    旋转部分用向量化恒等式 ``vec(A·X·B) = (Bᵀ⊗A)·vec(X)``：把 ``R_A·R_X = R_X·R_B``
    写成 ``(I⊗R_A − R_Bᵀ⊗I)·vec(R_X) = 0``。每段运动贡献 9 行，堆起来后
    ``vec(R_X)`` 就是该齐次方程的**最小奇异向量**；reshape（列优先）后投影回
    SO(3) 即可。

    相比 Tsai-Lenz 那类基于旋转轴对齐的闭式解，这个写法没有"两轴平行时退化"
    的分支，实现短、对噪声稳；代价是一次 SVD，对离线标定完全无所谓。

    一个容易踩的坑：齐次方程的解在 ``v`` 与 ``−v`` 两个方向上都成立，而
    "投影到 SO(3)"**不是**符号对称的——``−R`` 的行列式是 −1，投影回去得到的是
    另一个（差 180° 的）旋转。所以这里对两个符号各取一次最近旋转、代回方程比
    残差择优，避免掷硬币掷出反向解。

    平移部分：由 ``(R_A − I)·t_X = R_X·t_B − t_A`` 最小二乘解 ``t_X``。

    Raises:
        CalibrationError: 旋转数据退化（方程病态），或最优解仍不自洽
            （常见于 ``mode`` 与实际安装方式不符——两种模式的 ``B`` 互为逆，
            传错不会报错，只会得到永远对不上的外参，必须在求解期拦下）
    """
    rows = []
    for A, B in zip(A_list, B_list):
        rows.append(np.kron(np.eye(3), A[:3, :3]) - np.kron(B[:3, :3].T, np.eye(3)))
    M = np.vstack(rows)
    _, sv, Vt = np.linalg.svd(M)
    if sv[0] <= _EPS or sv[-1] / sv[0] > HAND_EYE_MAX_SV_RATIO:
        raise CalibrationError(
            f"运动数据退化或模型不符（奇异值比 {sv[-1] / max(sv[0], _EPS):.3f}）："
            f"请确认机器人给足了姿态差异，且 mode 与实际相机安装方式一致"
            f"（eye-in-hand 相机在法兰上 / eye-to-hand 相机固定在工位）"
        )

    def evaluate(raw: np.ndarray) -> Tuple[float, np.ndarray]:
        """给定近似的旋转矩阵，最小二乘解平移并算出方程残差。"""
        R_X = _orthonormalize(raw)
        A_cols, b_cols = [], []
        for A, B in zip(A_list, B_list):
            A_cols.append(A[:3, :3] - np.eye(3))
            b_cols.append(R_X @ B[:3, 3] - A[:3, 3])
        t_X, *_ = np.linalg.lstsq(np.vstack(A_cols), np.concatenate(b_cols), rcond=None)
        X = _transform(R_X, t_X)
        err = 0.0
        for A, B in zip(A_list, B_list):
            E = np.linalg.inv(X @ B) @ (A @ X)
            err = max(err, float(np.linalg.norm(E - np.eye(4))))
        return err, X

    raw = _unvectorize(Vt[-1])
    results = [evaluate(raw), evaluate(-raw)]
    err, X = min(results, key=lambda r: r[0])

    # 尺度参照：运动本身的平移量。解不自洽到"误差堪比运动幅度"就说明模型选错了
    scale = max(
        1.0,
        max(float(np.linalg.norm(A[:3, 3])) for A in A_list),
        max(float(np.linalg.norm(B[:3, 3])) for B in B_list),
    )
    if err > HAND_EYE_MAX_RESIDUAL_RATIO * scale:
        raise CalibrationError(
            f"手眼方程不自洽（残差 {err:.2f} mm vs 运动幅度 {scale:.1f} mm）："
            f"请检查 mode 是否与实际相机安装方式一致"
            f"（eye-in-hand 相机在法兰上 / eye-to-hand 相机固定在工位）"
        )
    return X


def solve_hand_eye(
    flange_poses: Sequence[Any],
    camera_readings: Sequence[Any],
    mode: str = "eye_in_hand",
    min_rotation_deg: float = DEFAULT_MIN_ROTATION_DEG,
) -> HandEyeResult:
    """手眼标定：由机器人位姿与相机读数解出相机外参。

    两种配置的方程推导（记号见模块头部）：

    **eye-in-hand**（相机在法兰上，标定板固定在工位上）
      相机在基座系的位姿 ``T_b_c = T_b_f·X``（``X = T_f_c`` 待求），
      同时 ``T_b_c = T_b_t·T_c_t⁻¹``（``T_b_t`` 为标定板在基座系，未知但恒定）。
      两式对第 i、j 帧分别成立后相除，得 ``A·X = X·B``，其中
      ``A = T_b_f_j⁻¹·T_b_f_i``、``B = T_c_t_j·T_c_t_i⁻¹``。

    **eye-to-hand**（相机固定，标定板装在夹爪上）
      ``T_b_c = T_b_f_i·X·T_c_m_i⁻¹``（``X = T_f_m`` 待求）恒定，同样相除得
      ``A = T_b_f_j⁻¹·T_b_f_i``、``B = T_c_m_j⁻¹·T_c_m_i``。

    注意两种模式的 ``B`` 是**互逆**的关系——这是手眼标定最容易抄错的一处，
    也是本函数把模式做成显式参数的原因：传错模式不会报错，只会得到一个永远
    对不上的外参。

    Args:
        flange_poses: 各次位姿的法兰位姿（机器人基坐标系）
        camera_readings: 与位姿一一对应的相机读数
        mode: ``"eye_in_hand"`` / ``"eye_to_hand"``
        min_rotation_deg: 有效运动的最小转角（度）。转角过小的相邻位姿解不出
            旋转轴，会被自动剔除

    Returns:
        :class:`HandEyeResult`

    Raises:
        CalibrationError: 模式非法、数据少于 3 组、有效运动不足 2 段，
            或旋转数据退化
    """
    if mode not in ("eye_in_hand", "eye_to_hand"):
        raise CalibrationError(
            f"手眼配置只支持 'eye_in_hand'/'eye_to_hand'，收到 {mode!r}"
        )
    if len(flange_poses) != len(camera_readings):
        raise CalibrationError(
            f"位姿数({len(flange_poses)})与相机读数数({len(camera_readings)})不一致"
        )
    if len(flange_poses) < 3:
        raise CalibrationError(
            f"手眼标定至少需要 3 组数据（≥2 段运动），收到 {len(flange_poses)} 组"
        )

    f_mats = [_as_matrix(f) for f in flange_poses]
    c_mats = [_as_matrix(c) for c in camera_readings]

    A_list: List[np.ndarray] = []
    B_list: List[np.ndarray] = []
    for i in range(len(f_mats) - 1):
        A = np.linalg.inv(f_mats[i + 1]) @ f_mats[i]
        if _rotation_deg(A) < min_rotation_deg:
            continue  # 姿态几乎没变，解不出轴，剔除
        if mode == "eye_in_hand":
            B = c_mats[i + 1] @ np.linalg.inv(c_mats[i])
        else:
            B = np.linalg.inv(c_mats[i + 1]) @ c_mats[i]
        A_list.append(A)
        B_list.append(B)

    if len(A_list) < 2:
        raise CalibrationError(
            f"有效运动不足：需要 ≥2 段转角 ≥{min_rotation_deg:g}° 的运动，"
            f"实际 {len(A_list)} 段（请让机器人多换几个姿态位）"
        )
    _check_axis_diversity([A[:3, :3] for A in A_list])

    X = _solve_ax_xb(A_list, B_list)

    # 相机外参：eye-to-hand 下相机固定在工位，各帧推出的外参应当一致，取均值
    # 抑制单帧视觉噪声；eye-in-hand 下相机**随法兰运动**，各帧外参本来就不同，
    # 取均值毫无意义——只报"首帧位姿下的相机位姿"，其余位形用 camera_pose_at 求
    extrinsics: List[np.ndarray] = []
    for f_mat, c_mat in zip(f_mats, c_mats):
        if mode == "eye_in_hand":
            extrinsics.append(f_mat @ X)
        else:
            extrinsics.append(f_mat @ X @ np.linalg.inv(c_mat))
    camera_in_base = (
        extrinsics[0] if mode == "eye_in_hand" else _average_transforms(extrinsics)
    )

    # 残差：把解代回 A·X = X·B，看两边差多少
    res_mm, res_deg = 0.0, 0.0
    for A, B in zip(A_list, B_list):
        E = np.linalg.inv(X @ B) @ (A @ X)
        res_mm = max(res_mm, float(np.linalg.norm(E[:3, 3])))
        res_deg = max(res_deg, _rotation_deg(E))

    return HandEyeResult(
        mode=mode,
        transform=_pose(X),
        camera_in_base=_pose(camera_in_base),
        n_motions=len(A_list),
        residual_mm=res_mm,
        residual_deg=res_deg,
    )


def generate_hand_eye_dataset(
    transform: Any,
    camera_in_base: Any,
    mode: str = "eye_in_hand",
    flange_poses: Optional[Sequence[Any]] = None,
    n: int = 6,
    nominal: Optional[Any] = None,
    noise_mm: float = 0.0,
    noise_deg: float = 0.0,
    seed: int = 0,
) -> HandEyeDataset:
    """按真值生成一组手眼标定数据（仿真验证 / 单元测试 / 现场预估精度）。

    由真值 ``X`` 与外参反算"相机应该看到什么"，得到的数据拿来跑
    :func:`solve_hand_eye` 就能验证整条标定链路（也用于评估噪声水平下的精度）。

    Args:
        transform: 真值 ``X``（eye-in-hand：``T_flange_camera``；eye-to-hand：
            ``T_flange_marker``）
        camera_in_base: 真值相机外参（基坐标系）
        mode: 配置模式
        flange_poses: 直接给定法兰位姿序列；``None`` 时以 ``nominal`` 为中心
            随机生成 ``n`` 个位姿（绕三轴各随机偏转、位置随机平移）——姿态
            多样性是手眼标定可解的前提
        n: ``flange_poses`` 为 ``None`` 时的位姿数量
        nominal: 随机位姿的中心（``Pose``）；``None`` 时取
            ``(400, 0, 400) mm + RPY(180, 0, 0)``（多数工位相机都是俯视姿态）
        noise_mm: 叠加到相机读数上的位置噪声标准差 (mm)
        noise_deg: 叠加到相机读数上的姿态噪声标准差（度）
        seed: 随机种子（固定默认值，保证结果可复现）

    Returns:
        :class:`HandEyeDataset`（``transform`` / ``camera_in_base`` 为真值）
    """
    if mode not in ("eye_in_hand", "eye_to_hand"):
        raise CalibrationError(
            f"手眼配置只支持 'eye_in_hand'/'eye_to_hand'，收到 {mode!r}"
        )
    X = _as_matrix(transform)
    T_b_c = _as_matrix(camera_in_base)

    if flange_poses is None:
        if nominal is None:
            nominal = Pose.from_xyz_rpy(400.0, 0.0, 400.0, 180.0, 0.0, 0.0)
        base = _as_matrix(nominal)
        rng = np.random.default_rng(seed)
        # 首帧用 nominal 本身（保证数据里有"标准位"），其余为绕随机轴 8°~30° 的
        # 偏转位——姿态多样性是 A·X = X·B 可解的前提
        generated: List[Pose] = [_pose(base)]
        for _ in range(max(int(n), 3)):
            angle = rng.uniform(8.0, 30.0)
            axis = rng.normal(size=3)
            axis = axis / np.linalg.norm(axis)
            R = Rotation.from_rotvec(np.radians(angle) * axis).as_matrix()
            t = base[:3, 3] + rng.uniform(-120.0, 120.0, 3)
            generated.append(_pose(_transform(R, t)))
        flange_poses = generated

    f_mats = [_as_matrix(f) for f in flange_poses]

    rng = np.random.default_rng(seed + 1)
    readings: List[Pose] = []
    for f_mat in f_mats:
        if mode == "eye_in_hand":
            # 标定板固定在工位上：先摆一个"相机前方 750mm"的板位置，再反算读数
            T_b_t = T_b_c @ _pose(_transform(np.eye(3), [0.0, 0.0, 750.0])).matrix
            T_c_t = np.linalg.inv(X) @ np.linalg.inv(f_mat) @ T_b_t
        else:
            T_c_t = np.linalg.inv(T_b_c) @ f_mat @ X
        readings.append(_pose(_perturb(T_c_t, noise_mm, noise_deg, rng)))

    return HandEyeDataset(
        mode=mode,
        flange_poses=[_pose(m) for m in f_mats],
        camera_readings=readings,
        transform=_pose(X),
        camera_in_base=_pose(T_b_c),
    )


def _perturb(
    matrix: np.ndarray, noise_mm: float, noise_deg: float, rng: np.random.Generator
) -> np.ndarray:
    """给一个位姿叠加零均值噪声（模拟视觉测量的抖动）。"""
    out = np.array(matrix, dtype=np.float64)
    if noise_mm > 0:
        out[:3, 3] = out[:3, 3] + rng.normal(0.0, noise_mm, 3)
    if noise_deg > 0:
        d = rng.normal(0.0, noise_deg, 3)
        out[:3, :3] = out[:3, :3] @ Rotation.from_rotvec(np.radians(d)).as_matrix()
    return out


# --------------------------------------------------------------------------- #
# 相机模型与视觉抓取
# --------------------------------------------------------------------------- #


@dataclass
class CameraIntrinsics:
    """针孔相机内参（像素单位）。

    约定与 OpenCV 一致：相机系 **X 向右、Y 向下、Z 向前**，成像为
    ``u = fx·x/z + cx``、``v = fy·y/z + cy``。

    Attributes:
        fx: X 方向焦距（像素）
        fy: Y 方向焦距（像素）
        cx: 主点列坐标（像素）
        cy: 主点行坐标（像素）
        width: 图像宽（像素）
        height: 图像高（像素）
    """

    fx: float
    fy: float
    cx: float
    cy: float
    width: int = 1280
    height: int = 720

    @classmethod
    def from_fov(
        cls,
        width: int,
        height: int,
        hfov_deg: float,
        fy: Optional[float] = None,
    ) -> "CameraIntrinsics":
        """由视场角构造（现场拿卷尺量视野算内参的常用做法）。

        Args:
            width: 图像宽（像素）
            height: 图像高（像素）
            hfov_deg: 水平视场角（度）
            fy: 竖直焦距（像素）；``None`` 时取 ``fx``（方形像素假设）
        """
        if not 0.0 < hfov_deg < 180.0:
            raise CalibrationError(f"水平视场角应在 (0,180) 度内，收到 {hfov_deg}")
        fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        return cls(
            fx=fx,
            fy=fx if fy is None else fy,
            cx=width / 2.0,
            cy=height / 2.0,
            width=int(width),
            height=int(height),
        )

    def project(self, point_cam: Sequence[float]) -> Tuple[float, float]:
        """相机系下的点 → 像素坐标。

        Raises:
            CalibrationError: 点在相机后方（``z <= 0``），投影无意义
        """
        x, y, z = np.asarray(point_cam, dtype=np.float64).ravel()
        if z <= _EPS:
            raise CalibrationError(f"点在相机后方或位于光心平面（z={z}），无法投影")
        return (float(self.fx * x / z + self.cx), float(self.fy * y / z + self.cy))

    def pixel_to_ray(self, pixel: Sequence[float]) -> np.ndarray:
        """像素 → 相机系单位方向向量（从光心指向该像素）。"""
        u, v = np.asarray(pixel, dtype=np.float64).ravel()
        d = np.array([(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0])
        return d / np.linalg.norm(d)

    def unproject(self, pixel: Sequence[float], depth_mm: float) -> np.ndarray:
        """像素 + 深度（沿光轴的 z，mm）→ 相机系下的 3D 点。"""
        if depth_mm <= 0:
            raise CalibrationError(f"深度必须为正，收到 {depth_mm}")
        u, v = np.asarray(pixel, dtype=np.float64).ravel()
        return np.array(
            [
                (u - self.cx) * depth_mm / self.fx,
                (v - self.cy) * depth_mm / self.fy,
                depth_mm,
            ]
        )

    def contains(self, pixel: Sequence[float]) -> bool:
        """像素是否落在图像范围内。"""
        u, v = np.asarray(pixel, dtype=np.float64).ravel()
        return 0.0 <= u < self.width and 0.0 <= v < self.height

    def to_dict(self) -> Dict[str, Any]:
        """序列化为可 JSON 化的字典。"""
        return {
            "fx": float(self.fx),
            "fy": float(self.fy),
            "cx": float(self.cx),
            "cy": float(self.cy),
            "width": int(self.width),
            "height": int(self.height),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CameraIntrinsics":
        """从 :meth:`to_dict` 的结果恢复。"""
        return cls(
            fx=float(data["fx"]),
            fy=float(data["fy"]),
            cx=float(data["cx"]),
            cy=float(data["cy"]),
            width=int(data.get("width", 1280)),
            height=int(data.get("height", 720)),
        )

    def summary(self) -> str:
        """一句话摘要。"""
        return (
            f"相机内参 fx={self.fx:.1f} fy={self.fy:.1f} "
            f"主点=({self.cx:.1f}, {self.cy:.1f}) {self.width}×{self.height}"
        )


def pixel_to_ray_in_base(
    camera: CameraIntrinsics,
    camera_in_base: Any,
    pixel: Sequence[float],
) -> Tuple[np.ndarray, np.ndarray]:
    """像素 → 基坐标系下的射线 ``(起点, 单位方向)``。

    Args:
        camera: 相机内参
        camera_in_base: 相机在机器人基坐标系中的外参（手眼标定产物）
        pixel: 像素坐标 ``(u, v)``

    Returns:
        ``(origin, direction)``——``origin`` 为光心（mm）、``direction`` 为单位向量
    """
    T = _as_matrix(camera_in_base)
    origin = T[:3, 3].copy()
    direction = T[:3, :3] @ camera.pixel_to_ray(pixel)
    return origin, direction / np.linalg.norm(direction)


def intersect_plane(
    origin: Sequence[float],
    direction: Sequence[float],
    plane_point: Sequence[float],
    plane_normal: Sequence[float] = (0.0, 0.0, 1.0),
) -> np.ndarray:
    """射线与平面求交（mm）。

    Args:
        origin: 射线起点 (mm)
        direction: 射线方向（不必单位化）
        plane_point: 平面上一点 (mm)
        plane_normal: 平面法向，默认水平面（Z 向上）

    Returns:
        交点坐标 (mm)

    Raises:
        CalibrationError: 射线与平面平行
    """
    o = np.asarray(origin, dtype=np.float64).ravel()
    d = np.asarray(direction, dtype=np.float64).ravel()
    p = np.asarray(plane_point, dtype=np.float64).ravel()
    n = np.asarray(plane_normal, dtype=np.float64).ravel()
    n = n / np.linalg.norm(n)
    denom = float(d @ n)
    if abs(denom) < 1e-12:
        raise CalibrationError("射线与目标平面平行，无交点（请检查相机朝向与平面法向）")
    t = float((p - o) @ n) / denom
    return o + t * d


def pixel_of_point(
    camera: CameraIntrinsics,
    camera_in_base: Any,
    point_mm: Sequence[float],
) -> Tuple[float, float]:
    """基坐标系下的点 → 像素（"相机应该看到它在哪"，用于仿真检测）。"""
    T_inv = np.linalg.inv(_as_matrix(camera_in_base))
    p_cam = _apply(T_inv, point_mm)
    return camera.project(p_cam)


def grasp_pose_from_pixel(
    camera: CameraIntrinsics,
    camera_in_base: Any,
    pixel: Sequence[float],
    plane_z: float = 0.0,
    grasp_z: Optional[float] = None,
    rpy: Sequence[float] = (180.0, 0.0, 0.0),
    xy_offset: Sequence[float] = (0.0, 0.0),
) -> Pose:
    """视觉抓取的核心一步：**像素 → 机器人抓取位姿**。

    流程：像素反投成基坐标系下的射线 → 与工件支承面（默认 ``z = plane_z`` 的水平面）
    求交得到抓取点的 XY → 抬到 ``grasp_z``（工件顶面）并套上固定抓取姿态。

    前提是相机外参已经标定（:func:`solve_hand_eye`）——外参错 1mm，抓取点就偏
    1mm，这也是"标定与手眼"和"相机触发抓取"必须同批交付的原因。

    Args:
        camera: 相机内参
        camera_in_base: 相机外参（基坐标系）
        pixel: 检测到的目标像素 ``(u, v)``
        plane_z: 工件支承面高度 (mm)，默认 0（地面）
        grasp_z: 抓取时 TCP 停的高度 (mm)；``None`` 时与 ``plane_z`` 相同
        rpy: 抓取姿态（度），默认吸盘/夹爪垂直向下（RPY 180/0/0）
        xy_offset: 在求交点基础上叠加的 XY 偏置 (mm)，用于补偿工具偏心

    Returns:
        :class:`Pose`——可直接作为 ``Robot.inverse_kinematics`` 的目标
    """
    origin, direction = pixel_to_ray_in_base(camera, camera_in_base, pixel)
    hit = intersect_plane(origin, direction, [0.0, 0.0, plane_z])
    dx, dy = np.asarray(xy_offset, dtype=np.float64).ravel()
    z = plane_z if grasp_z is None else float(grasp_z)
    return Pose.from_xyz_rpy(
        float(hit[0]) + dx, float(hit[1]) + dy, z, *(float(v) for v in rpy)
    )


# --------------------------------------------------------------------------- #
# 标定集：打包、落盘、写回仿真与 URDF
# --------------------------------------------------------------------------- #


@dataclass
class CalibrationSet:
    """一串标定结果的集合——"这台工位现在的真实参数"。

    Attributes:
        name: 标定集名称（如工位名），落盘文件名也用它
        tool: TCP 标定结果
        base: **机器人基座**在用户/世界坐标系中的位姿（:attr:`Robot.base` 的同名量）
        frames: 命名用户坐标系（工件坐标系、托盘坐标系……），一律表达在**机器人
            基坐标系**中。与 ``base`` 的区别很重要：``base`` 描述"机器人装在哪"，
            应用它会整体挪动机器人；``frames`` 只是"某个工件摆在哪"的参照系，
            用来算目标点，**绝不该**拿来改 :attr:`Robot.base`——把工件坐标系当
            基座装上去，等于把整台机器人搬走
        hand_eye: 手眼标定结果
        camera: 相机内参
        notes: 备注（标定人、工装、环境温度……）
        created: 生成时间（ISO 8601）
    """

    name: str = "workstation"
    tool: Optional[TcpCalibration] = None
    base: Optional[Pose] = None
    frames: Dict[str, Pose] = field(default_factory=dict)
    hand_eye: Optional[HandEyeResult] = None
    camera: Optional[CameraIntrinsics] = None
    notes: str = ""
    created: str = field(
        default_factory=lambda: datetime.now().isoformat(timespec="seconds")
    )

    # --------------------------- 用户坐标系 --------------------------- #

    def set_frame(self, name: str, pose: Any) -> "CalibrationSet":
        """登记一个命名用户坐标系（工件坐标系/托盘坐标系……，在基坐标系中表达）。

        Returns:
            自身，便于链式书写
        """
        self.frames[name] = _pose(_as_matrix(pose))
        return self

    def get_frame(self, name: str) -> Optional[Pose]:
        """取命名用户坐标系；未登记返回 ``None``（调用方自行决定如何降级）。"""
        return self.frames.get(name)

    # --------------------------- 应用 --------------------------- #

    def apply_to_robot(self, robot: Any, apply_base: bool = True) -> List[str]:
        """把标定结果灌进仿真实例。

        Args:
            robot: :class:`~orolp.robot.Robot`
            apply_base: 是否同时应用基坐标系标定。默认应用；若程序里已经手工
                摆好基座，可传 ``False`` 只更新 TCP

        Returns:
            实际生效项的说明列表（方便日志与通知里如实汇报"改了什么"）

        Raises:
            CalibrationError: 标定集里既没有 TCP 也没有基坐标——没什么可应用时
                报错，而不是安静地什么都不做
        """
        applied: List[str] = []
        if self.tool is not None:
            robot.tool = self.tool.offset
            applied.append(f"TCP 偏移 → {self.tool.offset.to_xyz_rpy()[:3]} mm")
        if apply_base and self.base is not None:
            robot.base = self.base
            applied.append(f"基坐标 → {self.base.to_xyz_rpy()[:3]} mm")
        if not applied:
            raise CalibrationError("标定集里没有 TCP / 基坐标结果，无法应用到机器人")
        return applied

    def summary_lines(self) -> List[str]:
        """若干行人类可读摘要（进日志或程序头注释）。"""
        lines: List[str] = [f"标定集 {self.name}（{self.created}）"]
        if self.tool is not None:
            lines.append(self.tool.summary())
        if self.base is not None:
            x, y, z, rx, ry, rz = self.base.to_xyz_rpy()
            lines.append(
                f"基坐标标定 基座在世界系 = ({x:.2f}, {y:.2f}, {z:.2f}) mm / "
                f"RPY ({rx:.2f}, {ry:.2f}, {rz:.2f})°"
            )
        for fname, fpose in self.frames.items():
            x, y, z, rx, ry, rz = fpose.to_xyz_rpy()
            lines.append(
                f"用户坐标系[{fname}] = ({x:.2f}, {y:.2f}, {z:.2f}) mm / "
                f"RPY ({rx:.2f}, {ry:.2f}, {rz:.2f})°"
            )
        if self.hand_eye is not None:
            lines.append(self.hand_eye.summary())
        if self.camera is not None:
            lines.append(self.camera.summary())
        if self.notes:
            lines.append(f"备注：{self.notes}")
        return lines

    # --------------------------- 持久化 --------------------------- #

    def to_dict(self) -> Dict[str, Any]:
        """序列化为可 JSON 化的字典（``schema`` 供将来兼容升级用）。"""
        return {
            "schema": "orolp.calibration/1",
            "name": self.name,
            "created": self.created,
            "notes": self.notes,
            "tool": self.tool.to_dict() if self.tool else None,
            "base": self.base.matrix.flatten().tolist() if self.base else None,
            "frames": {k: v.matrix.flatten().tolist() for k, v in self.frames.items()},
            "hand_eye": self.hand_eye.to_dict() if self.hand_eye else None,
            "camera": self.camera.to_dict() if self.camera else None,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CalibrationSet":
        """从 :meth:`to_dict` 的结果恢复（缺项容忍，便于分批标定）。"""
        base = data.get("base")
        frames = {
            k: _pose(np.array(v, dtype=np.float64).reshape(4, 4))
            for k, v in (data.get("frames") or {}).items()
        }
        return cls(
            name=data.get("name", "workstation"),
            tool=TcpCalibration.from_dict(data["tool"]) if data.get("tool") else None,
            base=_pose(np.array(base, dtype=np.float64).reshape(4, 4)) if base else None,
            frames=frames,
            hand_eye=(
                HandEyeResult.from_dict(data["hand_eye"]) if data.get("hand_eye") else None
            ),
            camera=(
                CameraIntrinsics.from_dict(data["camera"]) if data.get("camera") else None
            ),
            notes=data.get("notes", ""),
            created=data.get("created", ""),
        )

    def save(self, path: str | Path) -> Path:
        """保存为 JSON。"""
        p = Path(path)
        p.write_text(
            json.dumps(self.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8"
        )
        return p

    @classmethod
    def load(cls, path: str | Path) -> "CalibrationSet":
        """从 JSON 加载。"""
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    # --------------------------- 写回 URDF --------------------------- #

    def write_urdf(
        self,
        urdf_path: str | Path,
        out_path: Optional[str | Path] = None,
        joint_name: str = "tcp_joint",
        link_name: str = "tcp",
    ) -> Path:
        """把标定后的 TCP 写进 URDF 副本——仿真从此用真值。

        做法分两种情形，都落在同一个目标上：**让运动学末端就是标定后的 TCP**。

        * URDF **已带** ``tcp`` link（本仓库程序生成的模型都带：``tool0 → tcp``
          的 fixed 关节既是名义 TCP 也挂着显示几何）：**只更新它的 origin**，
          几何原样保留。这里刻意不删不建——删掉再建会把工具的可视化体一起丢掉。
        * URDF 没有：在运动学解析出的末端 link 下新建 ``fixed`` 关节 + ``tcp``
          link，origin 为标定偏移。

        之所以统一用 **tcp** 这个名字：运动学解析
        （:data:`orolp.kinematics._TIP_CANDIDATES`）优先把名为 ``tcp`` 的末端
        link 当工作末端，于是 URDF 一改，``FK`` 直接返回标定后的 TCP——不需要
        在任何地方额外记得"再乘一个工具变换"。

        Args:
            urdf_path: 源 URDF
            out_path: 输出路径；``None`` 时写同目录的 ``<stem>_calibrated.urdf``
            joint_name: 目标关节名（不存在时新建用）
            link_name: 目标末端 link 名（建议保持默认 ``tcp``）

        Returns:
            实际写出的路径

        Raises:
            CalibrationError: 标定集里没有 TCP 结果，源 URDF 无法解析，
                或末端 link 与目标 link 同名却没有对应关节（无处置）

        Note:
            只改副本，不动原始模型文件——原始 URDF 属于机器人库资产，混进
            某台机器人的标定值后会让别人的仿真一起偏。多次写入是幂等的：
            已存在的关节只更新 origin，不会叠加成两级工具。
        """
        if self.tool is None:
            raise CalibrationError("标定集里没有 TCP 结果，无法写回 URDF")
        src = Path(urdf_path)
        try:
            tree = ET.parse(src)
        except (OSError, ET.ParseError) as exc:
            raise CalibrationError(f"无法解析 URDF {src}: {exc}") from exc
        root = tree.getroot()

        offset = self.tool.offset
        x, y, z, rx, ry, rz = offset.to_xyz_rpy()
        xyz = f"{x / 1000.0:.6f} {y / 1000.0:.6f} {z / 1000.0:.6f}"  # URDF 用米
        rpy = f"{math.radians(rx):.6f} {math.radians(ry):.6f} {math.radians(rz):.6f}"

        # 1) 已有目标关节（按名字或按"子 link 就是目标 link"识别）：原地更新
        target: Optional[ET.Element] = None
        for joint in root.findall("joint"):
            child = joint.find("child")
            if joint.get("name") == joint_name or (
                child is not None and child.get("link") == link_name
            ):
                target = joint
                break

        if target is not None:
            origin = target.find("origin")
            if origin is None:
                origin = ET.SubElement(target, "origin")
            origin.set("xyz", xyz)
            origin.set("rpy", rpy)
            target.set("type", "fixed")  # 工具必须刚性固定
        else:
            # 2) 没有：挂到运动学末端之下新建
            try:
                from .kinematics import RobotKinematics

                tip_link = RobotKinematics(src).tip_link
            except Exception as exc:  # URDF 结构异常时仍给出可理解的报错
                raise CalibrationError(f"无法确定 URDF 末端 link：{exc}") from exc
            if tip_link == link_name:
                raise CalibrationError(
                    f"末端 link 已名为 {link_name!r} 却找不到对应关节，"
                    f"无法在其下挂接标定 TCP；请显式给出 joint_name"
                )
            if not any(el.get("name") == link_name for el in root.findall("link")):
                root.append(ET.Element("link", {"name": link_name}))
            joint = ET.SubElement(root, "joint", {"name": joint_name, "type": "fixed"})
            ET.SubElement(joint, "parent", {"link": tip_link})
            ET.SubElement(joint, "child", {"link": link_name})
            ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": rpy})

        if hasattr(ET, "indent"):  # Python 3.9+
            ET.indent(tree, space="  ")
        dst = Path(out_path) if out_path else src.with_name(f"{src.stem}_calibrated.urdf")
        dst.parent.mkdir(parents=True, exist_ok=True)
        tree.write(dst, encoding="utf-8", xml_declaration=True)
        return dst
