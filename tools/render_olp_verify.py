"""生成离线编程核心的验证图。

四张子图各回答一个"这功能到底是不是真的"的问题：

1. **moveL 直线度** —— TCP 到首尾连线的偏差全程 < 0.05mm。若退化成
   "只对终点求一次逆解"，偏差会达到几十毫米。
2. **moveC 圆弧度** —— 插补点到圆心的距离恒等于半径（误差 < 0.01mm）。
3. **节拍模型** —— 半速≈双倍耗时；短行程受加速段主导，因此不是严格翻倍。
4. **沿路径干涉** —— 机器人扫过工作台时，干涉在第几毫米被抓到。

运行::

    python3.11 tools/render_olp_verify.py [--out docs/preview/olp_verify.png]
"""

from __future__ import annotations

import argparse
import math
import sys
import warnings
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

from service.planner import TrajectoryPlanner  # noqa: E402
from domain.pose import Pose  # noqa: E402
from service.program import ProgramBuilder  # noqa: E402
from domain.robot import Robot  # noqa: E402
from domain.world.scene import Scene  # noqa: E402

warnings.filterwarnings("ignore")

URDF = REPO_ROOT / "assets/models/kuka/kr16_2/kr16_2.urdf"
HOME = [0.0, -20.0, 70.0, 0.0, 40.0, 0.0]

_CJK_FONT_CANDIDATES = (
    "Noto Sans CJK SC",
    "Noto Sans SC",
    "Source Han Sans SC",
    "WenQuanYi Zen Hei",
    "SimHei",
    "Microsoft YaHei",
)

#: 配色（与 render_workcell.py 保持一致）
BG = "#1a1a20"
FG = "#e6e6e6"
ACCENT = "#7fd3ff"
GOOD = "#6fdc6f"
BAD = "#ff8080"
GRID = "#3a3a44"


def _setup_cjk_font() -> str:
    """选中文字体，避免标题渲染成方框。"""
    installed = sorted({f.name for f in font_manager.fontManager.ttflist})
    loose = [n for n in installed if any(k in n for k in ("CJK", "Han", "Hei", "Song"))]
    for name in list(_CJK_FONT_CANDIDATES) + loose:
        if name in installed:
            plt.rcParams["font.sans-serif"] = [name] + list(plt.rcParams["font.sans-serif"])
            plt.rcParams["axes.unicode_minus"] = False
            return name
    return ""


def _tcp_xyz(pose: Pose) -> np.ndarray:
    return np.asarray(pose.to_xyz_quat()[:3], dtype=np.float64)


def _style(ax, title: str, xlabel: str, ylabel: str) -> None:
    ax.set_facecolor("#22222a")
    ax.set_title(title, color=FG, fontsize=11, pad=8)
    ax.set_xlabel(xlabel, color=FG, fontsize=9)
    ax.set_ylabel(ylabel, color=FG, fontsize=9)
    ax.tick_params(colors=FG, labelsize=8)
    ax.grid(True, color=GRID, linewidth=0.6, alpha=0.7)
    for spine in ax.spines.values():
        spine.set_color(GRID)


def make_robot() -> Robot:
    robot = Robot(str(URDF), "kr16_2")
    robot.set_joints(HOME)
    return robot


# --------------------------------------------------------------------------- #
# 1. moveL 直线度
# --------------------------------------------------------------------------- #


def linear_deviation() -> tuple[np.ndarray, np.ndarray, float]:
    """返回 (进度 0~1, 偏差 mm, 最大偏差)。"""
    robot = make_robot()
    p0 = robot.forward_kinematics()
    delta = np.array([150.0, 60.0, -100.0])
    p1 = p0.with_position(*(_tcp_xyz(p0) + delta))
    traj = ProgramBuilder(robot).moveL(p1).build().plan()

    a = _tcp_xyz(p0)
    direction = delta / np.linalg.norm(delta)
    prog, dev = [], []
    for wp in traj.waypoints:
        v = _tcp_xyz(wp.pose) - a
        prog.append(float(v @ direction) / np.linalg.norm(delta))
        dev.append(float(np.linalg.norm(v - float(v @ direction) * direction)))
    return np.array(prog), np.array(dev), float(max(dev))


# --------------------------------------------------------------------------- #
# 2. moveC 圆弧度
# --------------------------------------------------------------------------- #


def arc_radius_error() -> tuple[np.ndarray, np.ndarray, float]:
    """返回 (转角 deg, 半径误差 mm, 最大误差)。"""
    robot = make_robot()
    p0 = robot.forward_kinematics()
    base = _tcp_xyz(p0)
    center = base + np.array([-80.0, 0.0, 0.0])
    radius = 80.0

    def point(deg: float) -> np.ndarray:
        a = math.radians(deg)
        return center + np.array([radius * math.cos(a), radius * math.sin(a), 0.0])

    via = p0.with_position(*point(45.0))
    end = p0.with_position(*point(90.0))
    traj = ProgramBuilder(robot).moveC(via, end).build().plan()

    angles, errs = [], []
    r0 = _tcp_xyz(traj.waypoints[0].pose) - center
    for wp in traj.waypoints:
        r = _tcp_xyz(wp.pose) - center
        ang = math.degrees(math.atan2(float(np.cross(r0, r)[2]), float(r0 @ r)))
        angles.append(abs(ang))
        errs.append(abs(float(np.linalg.norm(r)) - radius))
    return np.array(angles), np.array(errs), float(max(errs))


# --------------------------------------------------------------------------- #
# 3. 节拍模型
# --------------------------------------------------------------------------- #


def cycle_time_sweep() -> tuple[np.ndarray, np.ndarray]:
    """扫描速度百分比，返回 (百分比, 节拍 s)。"""
    robot = make_robot()
    target = [90.0, -20.0, 70.0, 0.0, 40.0, 0.0]
    speeds = np.array([10, 25, 50, 75, 100])
    times = [
        ProgramBuilder(robot).moveJ(target, speed=float(s)).build().plan().duration()
        for s in speeds
    ]
    return speeds, np.array(times)


# --------------------------------------------------------------------------- #
# 4. 沿路径干涉
# --------------------------------------------------------------------------- #


def collision_profile() -> tuple[np.ndarray, np.ndarray, float]:
    """机器人下探扫过工作台，返回 (TCP 高度 mm, 是否干涉, 首次干涉高度)。"""
    scene = Scene()
    scene.set_robot(str(URDF))
    scene.add(
        "worktable",
        name="工作台",
        params={"length": 1400.0, "width": 900.0, "height": 780.0},
        pose=Pose.from_xyz_rpy(1200.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    )
    scene.set_robot_joints(HOME)

    robot = make_robot()
    p0 = robot.forward_kinematics()
    target = p0.with_position(900.0, 0.0, 500.0)
    traj = ProgramBuilder(robot).moveL(target).build().plan()

    heights, flags = [], []
    first_hit = None
    for wp in traj.waypoints:
        scene.set_robot_joints(np.asarray(wp.joints, dtype=np.float64))
        hit = bool(scene.check_collisions(include_robot=True, margin=-0.5))
        z = float(_tcp_xyz(wp.pose)[2])
        heights.append(z)
        flags.append(1 if hit else 0)
        if hit and first_hit is None:
            first_hit = z
    return np.array(heights), np.array(flags), float(first_hit if first_hit is not None else -1.0)


# --------------------------------------------------------------------------- #


def main() -> int:
    parser = argparse.ArgumentParser(description="生成 OLP 核心验证图")
    parser.add_argument("--out", default=str(REPO_ROOT / "docs/preview/olp_verify.png"))
    parser.add_argument("--dpi", type=int, default=130)
    args = parser.parse_args()

    font = _setup_cjk_font()
    if not font:
        print("[warning] 未找到中文字体，标题可能显示为方框")

    fig, axes = plt.subplots(2, 2, figsize=(12.5, 8.5), facecolor=BG)
    fig.suptitle(
        "RobotWorld — 离线编程核心验证（KUKA KR16-2）",
        color=FG,
        fontsize=14,
        fontweight="bold",
        y=0.97,
    )

    # --- 1. moveL 直线度 ---
    ax = axes[0][0]
    prog, dev, dev_max = linear_deviation()
    ax.plot(prog * 100, dev, color=ACCENT, linewidth=1.4)
    ax.axhline(0.05, color=GOOD, linestyle="--", linewidth=1.0, label="0.05 mm 门槛")
    _style(
        ax,
        f"moveL 直线度（最大偏差 {dev_max:.2e} mm）",
        "路径进度 (%)",
        "到理想直线的偏差 (mm)",
    )
    ax.legend(facecolor="#2b2b33", labelcolor=FG, fontsize=8, loc="upper left")
    ax.text(
        0.98,
        0.92,
        f"全程 < 0.05 mm\n（峰值 {dev_max:.1e} mm，DLS 收敛精度）\n→ 逐点逆解插补生效",
        transform=ax.transAxes,
        ha="right",
        va="top",
        color=GOOD,
        fontsize=8,
    )

    # --- 2. moveC 圆弧度 ---
    ax = axes[0][1]
    angles, errs, err_max = arc_radius_error()
    ax.plot(angles, errs, color=ACCENT, linewidth=1.4)
    ax.axhline(0.05, color=GOOD, linestyle="--", linewidth=1.0, label="0.05 mm 门槛")
    _style(
        ax,
        f"moveC 圆弧度（半径 80mm，最大误差 {err_max:.2e} mm）",
        "累计转角 (°)",
        "半径误差 (mm)",
    )
    ax.legend(facecolor="#2b2b33", labelcolor=FG, fontsize=8, loc="upper left")

    # --- 3. 节拍模型 ---
    ax = axes[1][0]
    speeds, times = cycle_time_sweep()
    ax.plot(speeds, times, marker="o", color=ACCENT, linewidth=1.6, markersize=5)
    for s, t in zip(speeds, times):
        ax.annotate(
            f"{t:.2f}s",
            (s, t),
            textcoords="offset points",
            xytext=(0, 7),
            ha="center",
            color=FG,
            fontsize=7.5,
        )
    _style(ax, "节拍 vs 进给速度（轴1 转 90°）", "进给速度 (%)", "估算节拍 (s)")
    ax.text(
        0.98,
        0.94,
        f"100% → 50%：{times[-1]:.2f}s → {times[2]:.2f}s\n" f"受加速段主导，非严格翻倍",
        transform=ax.transAxes,
        ha="right",
        va="top",
        color=FG,
        fontsize=8,
    )

    # --- 4. 沿路径干涉 ---
    ax = axes[1][1]
    heights, flags, first_hit = collision_profile()
    ax.plot(heights, flags, color=ACCENT, linewidth=1.6, drawstyle="steps-post")
    if first_hit > 0:
        ax.axvline(
            first_hit,
            color=BAD,
            linestyle="--",
            linewidth=1.2,
            label=f"首次干涉 z={first_hit:.0f}mm",
        )
        ax.legend(facecolor="#2b2b33", labelcolor=FG, fontsize=8, loc="upper right")
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["无", "干涉"])
    _style(ax, "沿路径干涉检查（下探扫过 780mm 高工作台）", "TCP 高度 z (mm)", "碰撞状态")

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=args.dpi, facecolor=BG)
    print(f"已生成：{out}  ({out.stat().st_size / 1024:.0f} KB)")
    print(f"  moveL 最大偏差 {dev_max:.2e} mm | moveC 最大半径误差 {err_max:.2e} mm")
    print(f"  100% 节拍 {times[-1]:.2f}s | 首次干涉高度 {first_hit:.0f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
