#!/usr/bin/env python3
"""RobotWorld 示例：插件接口全流程（v1.0.4）。

一条命令演示三个扩展点各自"从外部挂进来"的完整用法，全部基于真实 API：

1. **后处理插件** —— 加一个仓库里没有的品牌控制语言（示例是极简"ACME-V2"），
   注册后 ``program.to_post("acme_v2")`` 与内置品牌走完全同一条路；
2. **运动学插件** —— 用另一个运动学实现替换内置数值解。这里刻意实现一个
   **故意带偏差的实现**，用它来演示"换求解器之后轨迹与标定会发生什么变化"
   （真机场景里这就对应"用厂商解析解 / 用标定过的几何模型"）；
3. **渲染插件** —— 把轨迹上"偏差超过容差"的点标出来并连线，纯数据契约
   （``LineSegments`` / ``PointCloud`` / ``FrameMarker``），不依赖 OpenGL，
   所以在无头环境里也能跑、能测。

运行::

    python examples/plugins_demo.py
    # 或未安装 orolp 时：PYTHONPATH=python python examples/plugins_demo.py

以正式包分发插件（推荐做法）见模块末尾 ``ENTRY_POINTS_SNIPPET`` 的说明。
"""

from __future__ import annotations

import sys
import textwrap
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
for p in (str(REPO_ROOT / "python"), str(REPO_ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)

from orolp import Pose, ProgramBuilder, Robot  # noqa: E402
from orolp.kinematics import RobotKinematics  # noqa: E402
from orolp.planner import TrajectoryPlanner  # noqa: E402
from orolp.plugins import (  # noqa: E402
    FrameMarker,
    LineSegments,
    OverlayContext,
    PluginSpec,
    PointCloud,
    collect_overlays,
    default_registry,
    load_entry_points,
    register_kinematics,
    register_post_processor,
    register_renderer,
)

URDF = str(REPO_ROOT / "models/ur/ur5e/ur5e.urdf")
HOME = [0.0, -90.0, 90.0, -90.0, 90.0, 0.0]

TEMPLATE = """{{ config.comment_char }} ACME V2 / {{ meta.program_name|default('OROLP_PROG') }}
{{ config.comment_char }} 由第三方插件生成（RobotWorld 插件接口 v1）
DEF {{ meta.program_name|default('OROLP_PROG') }}()
{% for wp in waypoints %}
{% if wp.cmd in ['SETDO', 'WAITDI', 'WAITTIME', 'ARCSTART', 'ARCEND', 'SPRAYON', 'SPRAYOFF', 'FORCEMODE', 'FORCEMODEEND'] %}
  {{ wp.text }}
{% elif wp.cmd == 'MOVEC' %}
  MOVEC {{ wp.via }} -> {{ wp.target }} @ {{ wp.speed }} ;
{% else %}
  {{ wp.cmd }} {{ wp.target }} @ {{ wp.speed }} ;
{% endif %}
{% endfor %}END
"""


# --------------------------------------------------------------------------- #
# 1) 后处理插件：加一个仓库里没有的品牌
# --------------------------------------------------------------------------- #


def _make_acme_post():
    """构造 ACME-V2 后处理器。

    插件写法与内置品牌**完全一致**——继承 ``BasePostProcessor``、给
    ``brand``/``controller``、实现 ``_convert_motion``/``_get_template``。
    这正是把 :mod:`orolp.post.base` 开放出去的意义：抽象早就在了，只是此前
    外部够不着。
    """
    from orolp.post.base import BasePostProcessor, PostConfig, PostProcessError
    from orolp.program import Motion

    class AcmeV2PostProcessor(BasePostProcessor):
        """第三方品牌 ACME-V2（示例用的极简控制语言）。"""

        brand = "acme"
        controller = "v2"

        def __init__(self):
            super().__init__(
                PostConfig(precision=2, speed_unit="%", comment_char="//")
            )

        @property
        def language_name(self) -> str:
            return "ACME-V2"

        @property
        def file_extension(self) -> str:
            return "acm"

        def _convert_motion(self, motion: Motion) -> dict:
            if self.is_joint(motion):
                return {
                    "cmd": "MOVEJ",
                    "target": self.fmt_joints(motion.target),
                    "speed": f"{motion.speed:.0f}%",
                }
            if self.is_linear(motion):
                v = motion.velocity if motion.velocity is not None else motion.speed
                return {
                    "cmd": "MOVEL",
                    "target": self.fmt_pose(motion.target),
                    "speed": f"{v:.0f}mm/s",
                }
            if self.is_circular(motion):
                if motion.via is None:
                    raise PostProcessError("ACME-V2 圆弧运动需要提供 via 中间点")
                v = motion.velocity if motion.velocity is not None else motion.speed
                return {
                    "cmd": "MOVEC",
                    "via": self.fmt_pose(motion.via),
                    "target": self.fmt_pose(motion.target),
                    "speed": f"{v:.0f}mm/s",
                }
            raise PostProcessError(f"ACME-V2 不支持的运动类型: {motion.type}")

        def _get_template(self) -> str:
            return TEMPLATE

    return AcmeV2PostProcessor()


# --------------------------------------------------------------------------- #
# 2) 运动学插件：替换内置求解器
# --------------------------------------------------------------------------- #


class OffsetKinematics(RobotKinematics):
    """在真机几何**标定值**与 URDF 标称值之间插一层偏置的运动学实现。

    真机现场常遇到这种需求：URDF 是按标称尺寸建的程序生成模型，而实际机器
    连杆长度/零点有偏差（几百微米到几毫米）。做法是取内置解，再在关节零位上
    加一个标定得到的偏置 ``dof`` —— 于是仿真与"真机实际姿态"一致，而轨迹里
    记的仍是控制器的关节读数。

    Args:
        urdf_path: URDF 路径
        joint_offsets_deg: 各关节零位偏置（度）
    """

    def __init__(self, urdf_path: str, joint_offsets_deg=None):
        super().__init__(urdf_path)
        offsets = joint_offsets_deg or [0.0] * self.dof
        arr = np.asarray(offsets, dtype=float).ravel()
        if arr.size == 1:
            arr = np.full(self.dof, float(arr[0]))
        if arr.size != self.dof:
            raise ValueError(f"joint_offsets_deg 长度 {arr.size} 与 dof {self.dof} 不符")
        self._offsets = arr
        self._orolp_signature = f"OffsetKinematics（零位偏置 {np.max(np.abs(arr)):g}°）"

    @property
    def joint_offsets_deg(self) -> np.ndarray:
        """当前零位偏置（度）。"""
        return self._offsets.copy()

    def forward_kinematics(self, q, q_is_radians: bool = True):
        """正解时把"读数"换算成"实际关节角"再交给内置解。"""
        q = np.asarray(q, dtype=float).ravel()
        corrected = q - (np.radians(self._offsets) if q_is_radians else self._offsets)
        return super().forward_kinematics(corrected, q_is_radians=q_is_radians)


# --------------------------------------------------------------------------- #
# 3) 渲染插件：把超差点标出来
# --------------------------------------------------------------------------- #


class ToleranceOverlayRenderer:
    """把"另一套求解器/实测"轨迹中偏离参考轨迹超过容差的点标红并连线。

    覆盖层契约刻意只有**纯数据**（线段/点云/位姿轴），不碰 OpenGL——这样同一份
    插件既能给 GUI 视口用，也能被无头测试断言，还能给网页演示导出。

    参考轨迹从 ``ctx.extra["reference_path"]`` 取（演示里就是"内置求解器算出的
    同一条关节轨迹"）；没提供、或两条轨迹长度不一致时，退回"与首末点连线的
    垂直距离"这个粗代理——它只适合直线运动，真实场景请改用
    :func:`orolp.twinsync.align_traces` + ``TwinReport`` 的实测偏差。
    """

    def __init__(self, tolerance_mm: float = 1.0, max_markers: int = 200):
        self.tolerance_mm = float(tolerance_mm)
        self.max_markers = int(max_markers)
        #: 最近一次 overlays() 的统计（供调用方打印/断言，不进图元）
        self.last_stats: dict = {}

    def overlays(self, ctx: OverlayContext):
        """返回覆盖层图元列表（轨迹不可用时返回空列表，不报错）。"""
        self.last_stats = {}
        if ctx is None or ctx.count < 3:
            return []
        pts = np.asarray(ctx.tcp_path, dtype=float)
        finite = np.isfinite(pts).all(axis=1)
        pts = pts[finite]
        if pts.shape[0] < 3:
            return []
        dev = self._deviation(pts, ctx)
        if dev is None:
            return []
        self.last_stats = {
            "points": int(pts.shape[0]),
            "flagged": int(np.count_nonzero(dev > self.tolerance_mm)),
            "max_dev_mm": float(np.max(dev)),
            "tolerance_mm": self.tolerance_mm,
        }

        out = []
        flag_idx = np.where(dev > self.tolerance_mm)[0]
        if flag_idx.size:
            keep = flag_idx[: self.max_markers]
            out.append(
                PointCloud(positions=pts[keep], color=(1.0, 0.25, 0.25), size=7.0)
            )
        worst = int(np.argmax(dev))
        if dev[worst] > self.tolerance_mm:
            out.append(
                LineSegments(
                    segments=[(pts[worst], pts[worst] + np.array([0, 0, 80.0]))],
                    color=(1.0, 0.55, 0.1),
                    width=3.0,
                )
            )
        out.append(PointCloud(positions=pts, color=(0.35, 0.75, 1.0), size=3.0))
        return out

    def _deviation(self, pts: np.ndarray, ctx: OverlayContext):
        """逐点偏差（mm）；无法判定时返回 ``None``。"""
        ref = (ctx.extra or {}).get("reference_path")
        if ref is not None:
            ref = np.asarray(ref, dtype=float)
            if ref.ndim == 2 and ref.shape == pts.shape:
                return np.linalg.norm(pts - ref, axis=1)
        # 退回：与首末点连线的垂直距离（仅适合近似直线运动）
        p0, p1 = pts[0], pts[-1]
        d = p1 - p0
        norm = float(np.linalg.norm(d))
        if norm < 1e-9:
            return None
        u = d / norm
        rel = pts - p0
        proj = np.outer(rel @ u, u)
        return np.linalg.norm(rel - proj, axis=1)


def _pose_frame_overlay():
    """再给一个位姿轴示例插件（画首点坐标系）。"""

    class HomeFrameRenderer:
        """在轨迹起点画一个位姿三轴。"""

        def overlays(self, ctx: OverlayContext):
            if ctx is None or ctx.count == 0:
                return []
            x, y, z = ctx.tcp_path[0]
            if not np.isfinite([x, y, z]).all():
                return []
            return [FrameMarker(Pose.from_xyz_rpy(x, y, z, 0, 180, 0), scale=80.0)]

    return HomeFrameRenderer()


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #


def main() -> int:
    print("=" * 72)
    print("RobotWorld 插件接口演示（v1.0.4 · 三个扩展点 · 无 GUI）")
    print("=" * 72)

    # ---- 登记三个插件 ----
    print("\n[1] 登记插件")
    register_post_processor(
        "acme_v2",
        _make_acme_post,
        version="1.0.0",
        description="第三方品牌 ACME-V2（示例）",
    )
    register_kinematics(
        "acme.calibrated",
        lambda urdf_path, **kw: OffsetKinematics(urdf_path, **kw),
        version="1.0.0",
        description="带零位标定偏置的 UR5e 运动学（示例）",
        options={"joint_offsets_deg": [0.0, 0.05, -0.04, 0.0, 0.03, 0.0]},
    )
    # 注册一个"预先构造好的实例"（工厂是零参可调用即可）——这样演示代码拿着
    # 同一个对象，就能读回它最近一次计算的统计，方便打印与断言
    tolerance_renderer = ToleranceOverlayRenderer(tolerance_mm=0.3)
    register_renderer(
        "acme.tolerance_overlay",
        lambda: tolerance_renderer,
        version="1.0.0",
        description="把超过容差的轨迹点标红（示例）",
    )
    register_renderer(
        "acme.home_frame",
        _pose_frame_overlay,
        version="1.0.0",
        description="在轨迹起点画坐标系（示例）",
    )
    print(default_registry.summary())

    # ---- 入口点发现（正式分发方式） ----
    print("\n[2] 扫描已安装包的插件入口点（group=orolp.plugins）")
    stats = load_entry_points()
    print(f"    发现 {stats['discovered']} / 登记 {stats['registered']} / "
          f"跳过 {stats['skipped']} / 失败 {stats['failed']}")
    if not stats["discovered"]:
        print("    （当前环境没有以包形式安装的插件——用 register_* 直接登记同样有效）")

    # ---- 后处理插件出码 ----
    robot = Robot(URDF, "UR5e")
    program = (
        ProgramBuilder(robot)
        .moveJ(HOME, speed=60)
        .moveJ([10.0, -70.0, 70.0, -90.0, 90.0, 10.0], speed=60)
        .build()
    )
    from orolp.post import list_post_processors

    print("\n[3] 后处理器清单（内置 7 个 + 插件）")
    print(f"    {', '.join(list_post_processors())}")
    meta = {"program_name": "ACME_DEMO"}
    plugin_code = program.to_post("acme_v2", metadata=meta)
    builtin_code = program.to_post("kuka_krc4", metadata=meta)
    print(f"\n    插件品牌 acme_v2 出码（{len(plugin_code.splitlines())} 行）:")
    for line in plugin_code.splitlines()[:8]:
        print(f"      {line}")
    print(f"    内置品牌 kuka_krc4 同程序出码（{len(builtin_code.splitlines())} 行）——同一条通路")

    # ---- 运动学插件：换求解器带来的差异 ----
    print("\n[4] 运动学插件：同一组关节读数，两个求解器给出的 TCP 不同")
    # 用独立对象做探针，避免把主规划机器人挪离 Home（第 5 节要用同一条轨迹）
    builtin_robot = Robot(URDF, "UR5e")
    probe_robot = Robot(
        URDF, "UR5e", kinematics_plugin="acme.calibrated", kinematics_options={}
    )
    q_test = [10.0, -70.0, 70.0, -90.0, 90.0, 10.0]
    builtin_robot.set_joints(q_test)
    probe_robot.set_joints(q_test)
    p_builtin = builtin_robot.forward_kinematics()
    p_plugin = probe_robot.forward_kinematics()
    xyz_b = [float(v) for v in p_builtin.to_xyz_rpy()[:3]]
    xyz_p = [float(v) for v in p_plugin.to_xyz_rpy()[:3]]
    delta = float(np.linalg.norm(np.asarray(xyz_p) - np.asarray(xyz_b)))
    print(f"    内置求解器        : {tuple(round(v, 3) for v in xyz_b)}")
    print(f"    插件（带标定偏置）: {tuple(round(v, 3) for v in xyz_p)}")
    print(f"    两个求解器的 TCP 差 {delta:.3f} mm —— 这就是 标定过 vs 没标定 的差别")
    print(f"    契约检查: {probe_robot.kinematics_contract}")

    # ---- 渲染插件：纯数据覆盖层 ----
    print("\n[5] 渲染插件：收集覆盖层（纯数据，可在无头环境断言）")
    plugin_robot = Robot(
        URDF, "UR5e", kinematics_plugin="acme.calibrated", kinematics_options={}
    )
    plugin_traj = TrajectoryPlanner(plugin_robot).plan(program)
    # 参考轨迹：同一条程序、同一组关节读数，但用内置（未标定）几何求解
    ref_traj = TrajectoryPlanner(Robot(URDF, "UR5e")).plan(program)
    ref_path = np.asarray(
        [list(wp.pose.to_xyz_rpy()[:3]) for wp in ref_traj.waypoints], dtype=float
    )
    same_len = ref_path.shape[0] == len(plugin_traj.waypoints)
    print(f"    参考轨迹：内置求解器 {ref_path.shape[0]} 点；"
          f"插件求解器 {len(plugin_traj.waypoints)} 点；"
          f"{'逐点可比' if same_len else '长度不一致，退回粗代理'}")

    ctx = OverlayContext.from_trajectory(
        plugin_traj,
        current_index=0,
        tolerance_mm=tolerance_renderer.tolerance_mm,
        robot_name=plugin_robot.name,
        extra={"reference_path": ref_path},
    )
    collected = collect_overlays(ctx)

    print(f"    上下文：{ctx.count} 个轨迹点，当前索引 {ctx.current_index}，"
          f"容差 {ctx.tolerance_mm} mm")
    print(f"    收集结果：{collected.summary()}")
    for name, item in collected.drawables:
        if isinstance(item, PointCloud):
            print(f"      · {name}: PointCloud {item.positions.shape} 色 {item.color}")
        elif isinstance(item, LineSegments):
            print(f"      · {name}: LineSegments {len(item.segments)} 条 线宽 {item.width}")
        elif isinstance(item, FrameMarker):
            xyz = tuple(round(float(v), 1) for v in item.pose.to_xyz_rpy()[:3])
            print(f"      · {name}: FrameMarker @ {xyz}")
    if tolerance_renderer.last_stats:
        s = tolerance_renderer.last_stats
        print(f"    标定偏差（标定 vs 未标定，同一条关节轨迹）："
              f"最大 {s['max_dev_mm']:.3f} mm，"
              f"超 {s['tolerance_mm']} mm 的有 {s['flagged']}/{s['points']} 点")
    if collected.errors:
        for e in collected.errors:
            print(f"      ! {e.name}: {e.message}")

    # ---- 说明正式分发方式 ----
    print("\n[6] 以正式包分发插件（pyproject.toml）")
    print(textwrap.indent(ENTRY_POINTS_SNIPPET.strip(), "    "))

    print("=" * 72)
    print("演示完成：三个扩展点都从外部挂进来了，核心代码一行未改。")
    return 0


#: 第三方包声明插件的标准写法（宿主 load_entry_points() 会自动发现）
ENTRY_POINTS_SNIPPET = """
[project.entry-points."orolp.plugins"]
acme_v2 = "acme_orolp.plugin:post_processor_spec"
acme_calibrated = "acme_orolp.plugin:kinematics_spec"

# acme_orolp/plugin.py 里返回 PluginSpec（或其列表）
from orolp.plugins import PluginSpec

post_processor_spec = PluginSpec(
    name="acme_v2", kind="post",
    factory="acme_orolp.post:AcmeV2PostProcessor",
    version="1.0.0", description="ACME-V2 控制语言",
)
"""


if __name__ == "__main__":
    raise SystemExit(main())
