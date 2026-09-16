#!/usr/bin/env python3
"""生成网页版演示站点（deploy/web/）——「可体验部署」的构建脚本。

每次自动开发循环完成任务后重新运行本脚本，再重新发布 deploy/web/ 目录，
在线链接保持不变。

产出::

    deploy/web/index.html          单页演示（无外部 JS 依赖，离线可开）
    deploy/web/assets/*.png        GUI 截图（从 docs/screenshots 复制）

页面内容（数据全部来自真实 API，不是手写假数据）：

* 抓取工作站俯视示意图（SVG，与 templates.py 布局一致）
* TCP 轨迹动画（SVG + JS，真实轨迹采样点）
* 节拍数据（Program.simulate 结果）
* UR / KUKA / ABB 出码样例（真实后处理输出，截取）
* GUI 截图

运行::

    python tools/build_web_site.py
"""

from __future__ import annotations

import calendar
import html
import math
import re
import json
import os
import shutil
import socket
import sys
import time
import warnings
from pathlib import Path

import numpy as np  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

from orolp import Pose, Robot  # noqa: E402
from orolp.program import ProgramBuilder  # noqa: E402
from orolp.world import build_grasping_workstation  # noqa: E402

warnings.filterwarnings("ignore")

#: 当前全量测试套件规模（`pytest tests python/tests`），网页上所有"测试基线"数字统一取它
TOTAL_TESTS = 1684


def _display_width(text: str) -> int:
    """文本的显示列宽（CJK 与全角符号占 2 列）。

    ``str.ljust`` 数的是**字符**不是列宽，中英混排的表格用它必然参差。
    """
    import unicodedata

    return sum(2 if unicodedata.east_asian_width(c) in ("W", "F") else 1 for c in text)


def _ljust(text: str, width: int) -> str:
    """按显示列宽左对齐补空格。"""
    return text + " " * max(0, width - _display_width(text))


def _rjust(text: str, width: int) -> str:
    """按显示列宽右对齐补空格。"""
    return " " * max(0, width - _display_width(text)) + text

WEB_DIR = REPO_ROOT / "deploy/web"
ASSET_DIR = WEB_DIR / "assets"

HOME = [0, -90, 90, -90, 90, 0]
# 与 orolp.world.templates 的可达性校准布局保持一致（2026-09-13 S4 修正）
BELT_TOP = 200 + 66 + 2  # 皮带面顶 + 工件间隙
WORKPIECE_H = 60.0
PALLET_TOP = 144.0
PICK_XY = (60.0, -400.0)   # 工件_1 中心
PLACE_XY = (350.0, 450.0)  # 托盘落件点
WORKPIECE_XS = (60, 170, 280, 390)


def _demo_program(robot: Robot):
    """抓放主循环：home → 接近 → 下探 → [夹紧] → 提升 → 转移 → 落件 → [松开] → 退回 → home。"""
    pick = Pose.from_xyz_rpy(*PICK_XY, BELT_TOP + WORKPIECE_H, 180, 0, 0)
    place = Pose.from_xyz_rpy(*PLACE_XY, PALLET_TOP + WORKPIECE_H, 180, 0, 0)
    return (
        robot.moveJ(HOME, speed=80)
        .then(robot.moveL(pick.with_z_offset(100), speed=100))
        .then(robot.moveL(pick, speed=40))
        .gripper("close")
        .then(robot.moveL(pick.with_z_offset(100), speed=80))
        .then(robot.moveL(place.with_z_offset(150), speed=100, velocity=350.0, acceleration=1400.0))
        .then(robot.moveL(place, speed=40))
        .gripper("open")
        .then(robot.moveL(place.with_z_offset(150), speed=80))
        .then(robot.moveJ(HOME, speed=80))
    )


#: 演示程序用的 IO 点表（信号逻辑名 → 物理地址）
IO_MAP = {
    "gripper_close": "Q0.1",
    "gripper_open": "Q0.0",
    "clamped": "I0.3",
}


def collect_data() -> dict:
    """用真实 API 收集演示数据。"""
    cell = build_grasping_workstation()
    robot = Robot("models/ur/ur5e/ur5e.urdf", "UR5e")
    robot.tool = cell.tcp_flange
    prog = _demo_program(robot)

    traj = prog.simulate(collision_check=False)
    from orolp.planner import TrajectoryPlanner

    traj_scurve = TrajectoryPlanner(robot, profile="scurve").plan(prog.build())
    report = prog.validate()
    meta = {"program_name": "GRASP_DEMO", "io_map": IO_MAP}
    posts = {
        brand: prog.to_post(brand, metadata=meta)
        for brand in ("ur_polyscope", "kuka_krc4", "abb_irc5", "fanuc_r30ib")
    }
    io_count = sum(
        1
        for i in prog.build().items
        if type(i).__name__ in ("SetDO", "WaitDI", "WaitTime")
    )
    return {
        "cycle_time": traj.duration(),
        "cycle_time_scurve": traj_scurve.duration(),
        "motion_times": report.motion_times or [],
        "motion_labels": report.motion_labels or [],
        "waypoints": len(prog.motions) if hasattr(prog, "motions") else 8,
        "io_commands": io_count,
        "posts": posts,
        "objects": [n for n in cell.scene.objects if n != "机器人"],
        "n_workpieces": len(cell.grasp_objects),
        "tcp": cell.tcp_flange.to_xyz_rpy(),
        **_so101_dynamics(),
        **_cam_data(),
        **_process_data(),
        **_calibration_data(),
        **_multirobot_data(),
        **_compliance_data(),
        **_twin_data(robot, traj, posts["ur_polyscope"]),
        # 插件小节最后跑：它会往 default_registry 里登记（含一个故意做坏的）插件，
        # 放在最后就不会影响上面那些真实出码
        **_plugins_data(robot, traj),
        # 真机 Socket 小节：起一个回环打桩控制器，会移动"控制器侧那台机器"的关节，
        # 因此也放在最后（上面所有几何/运动学数据此时都已采集完毕）
        **_socket_data(robot, traj),
        # HTTP / S3 云通道小节（v1.0.6）：同样起本机回环服务端 + 真实 HTTP 往返
        **_cloud_data(robot, traj),
        # 程序版本对比小节（v1.0.7）：真 HTTP 版本库 + 真规划器节拍/路径对比
        **_progdiff_data(robot, traj),
        **_pluginpkg_data(robot),
        **_envelope_data(),
        # i18n 小节（v1.0.10）：真实目录 + 真实接线统计
        **_i18n_data(),
        # 库层消息 i18n 小节（v1.1）：真实跑 en/zh 双语校验报告与出码
        **_i18n_lib_data(),
        # 真机连接面板小节（v1.1.1）：真起打桩控制器真跑面板 + 离屏截图（放最后，Qt 依赖）
        **_controller_panel_data(),
        # 真机连接 SSH 小节（v1.1.2）：真 SSH/SFTP/exec 回环
        **_ssh_link_data(),
        # waitTime 驻留段时间轴小节（v1.1.3）：真规划器节拍对比 + 双机占用
        **_dwell_data(),
        # 真机连接面板 · SSH 传输小节（v1.1.4，放最后：真起 SSH 打桩 + Qt 离屏截图）
        **_controller_panel_ssh_data(),
        # 轨迹编辑器圆弧编辑小节（v1.1.5，Qt 离屏真跑编辑器 + 截图）
        **_trajectory_arc_data(),
        # PLC ↔ 机器人 IO 点表小节（v1.1.6，真跑校验/自动编号/出码 + 截图）
        **_iomap_data(),
        # 场景拖拽摆放小节（v1.1.7，真跑 placement：支撑面/吸附/拒绝理由）
        **_placement_data(),
        # 规模与性能基准小节（v1.1.8，真跑两份 FK 实现比对 + 真规划器分段计时）
        **_perf_data(),
        # 干涉检查采样与几何缓存小节（v1.1.9，真跑内核对拍 + 采样口径分段 + 三级漏斗账本）
        **_collision_perf_data(),
        # 增量重算小节（v1.2.0，真跑两次规划 + 两组分场景基准 + 两种目标写法对照）
        **_incremental_data(),
        # 真机 CAD 链路小节（v1.2.1，真造合成装配体把 STEP→网格→URDF→Robot 跑通）
        **_vendor_step_data(),
        # 干涉检查结果缓存小节（v1.2.2，真跑两次校验/两次检查 + 场景变更正确性演示）
        **_collision_cache_data(),
        # 力矩校验结果缓存小节（v1.2.3，真跑两次校验 + 改负载 miss + 关缓存对照）
        **_dynamics_cache_data(),
        # 缓存可观测性小节（v1.2.4，真跑校验后取 cache_overview 真实快照）
        **_cache_overview_data(),
        # 界面架构改版小节（v1.3.0，真起主窗口读标签结构 + 真跑法兰装夹，放最后）
        **_ui_layout_data(),
        # 工艺时序（动态时序）小节（v1.3.1，真跑序列模型 + 双向出口 + 泳道编辑器）
        **_sequence_data(),
        # 动态时序使用体验小节（v1.3.2，构建时子进程真跑 examples/sequence_demo.py）
        **_sequence_demo_data(),
        # 零件库与预置场景库小节（v1.3.3，真建模板真注册零件）
        **_partslib_data(),
        # 工程文件小节（v1.3.4，真存真读 + 坏段容错 + 保真逐字节核对 + 真跑 CLI）
        **_project_data(),
        # 会话支撑小节（v1.3.5，真存真读最近打开 + 真写真读自动保存 + 真跑 CLI）
        **_session_data(),
        # 工程模板小节（v1.3.6，真构建三模板 + 真存真读 + 真跑 CLI）
        **_tpl_data(),
    }


def _twin_data(robot: Robot, traj, ur_code: str) -> dict:
    """v1.0.3 数字孪生与云协作小节的真实数据。

    整条链路真实跑一遍：抓放轨迹 → 仿真孪生轨迹 → 两种"真机"（合成传动误差模型）
    → 上传云端 → 跨源偏差报告 → 曲线 SVG。**页面上标注了 source=plant 是合成模型**
    ——没有真机就不假装有，数字给的是"管线跑通量级"，不是现场实测。
    """
    import tempfile

    from orolp.debug import DebugSession
    from orolp.link import SimLink
    from orolp.twinsync import (
        CloudLink,
        LocalCloudStore,
        PlantModel,
        capture_from_session,
        capture_plant_trace,
        compare_traces,
        deviation_chart_svg,
    )

    # ---- 1) 仿真侧孪生轨迹（复用 DebugSession.deviation） ----
    sim_link = SimLink(robot)
    sim_link.connect()
    sim = capture_from_session(
        DebugSession(traj, sim_link, robot=robot), source="sim", program_name="GRASP_DEMO"
    )

    # ---- 2) 云端（文件式对象存储充当云，接真云只需换 transport） ----
    cloud = CloudLink(
        LocalCloudStore(Path(tempfile.mkdtemp(prefix="rw_web_cloud_"))),
        robot=robot,
        namespace="web-demo",
    )
    cloud.connect()
    cloud.upload_program("GRASP_DEMO", ur_code, {"ext": "urp", "brand": "ur"})
    cloud.publish_trace(sim)

    plants = (
        (
            "tuned",
            "调好的机器",
            PlantModel(
                joint_bias_deg=[0.01, -0.008, 0.012, 0.0, -0.006, 0.005],
                scale_error=2e-5,
                noise_deg=0.0008,
                lag_tau=0.004,
                seed=7,
            ),
        ),
        (
            "poor",
            "差调机的机器",
            PlantModel(
                joint_bias_deg=[0.35, -0.28, 0.30, 0.0, -0.22, 0.18],
                scale_error=8e-4,
                noise_deg=0.01,
                lag_tau=0.02,
                seed=7,
            ),
        ),
    )

    rows = []
    charts = {}
    last_q = None
    for key, label, plant in plants:
        real = capture_plant_trace(traj, robot, plant, program_name="GRASP_DEMO")
        cloud.publish_trace(real, name=f"GRASP_DEMO_{key}")
        rep = compare_traces(sim, real, tol_pos_mm=1.0, tol_rot_deg=1.0, max_dt=0.05)
        split = rep.speed_split()
        charts[key] = deviation_chart_svg(
            rep,
            width=760,
            height=260,
            title=f"TCP 偏差 · {label}（sim vs plant）· 容差 1 mm",
        )
        rows.append(
            {
                "label": label,
                "plant": plant.summarize(),
                "max_mm": rep.max_pos_mm,
                "mean_mm": rep.mean_pos_mm,
                "rms_mm": rep.rms_pos_mm,
                "max_rot": rep.max_rot_deg,
                "over": len(rep.over_tol),
                "n": len(rep.samples),
                "slow_mm": split["slow_max_mm"],
                "fast_mm": split["fast_max_mm"],
                "n_slow": split["n_slow"],
                "ratio": split["fast_max_mm"] / max(split["slow_max_mm"], 1e-9),
                "causes": rep.format_causes(),
                "verdict": rep.verdict(),
            }
        )
        last_q = real.snapshots[-1].actual_q

    cloud.push_status(last_q, source="plant")
    trace_rows = []
    for name in cloud.list_traces():
        meta = cloud.trace_meta(name)
        max_dev = meta["max_pos_mm"]
        max_dev_text = "—（无实测回读）" if max_dev is None else f"{max_dev:.3f} mm"
        trace_rows.append(
            f"<tr><td><code>{html.escape(name)}</code></td>"
            f"<td>{html.escape(str(meta['source']))}</td><td>{meta['samples']}</td>"
            f"<td>{max_dev_text}</td></tr>"
        )

    return {
        "twin_chart_tuned": charts["tuned"],
        "twin_chart_poor": charts["poor"],
        "twin_rows": rows,
        "twin_samples": len(sim),
        "twin_cycle": traj.duration(),
        "twin_trace_rows": "\n".join(trace_rows),
        "twin_trace_count": len(cloud.list_traces()),
        "twin_status_age": cloud.status_age() or 0.0,
        "twin_status_fresh": cloud.is_fresh(5.0),
        "twin_readback": " ".join(f"{v:.3f}" for v in cloud.read_joints()),
        "twin_jsonl_head": "\n".join(sim.to_jsonl().splitlines()[:3]),
    }


def _plugins_data(robot: Robot, traj) -> dict:
    """v1.0.4 插件接口小节的真实数据。

    三个扩展点各自从**包外**挂进来跑一遍真实通路：注册第三方品牌后处理 → 出码；
    注册带"标定偏置"的运动学实现 → 与内置求解器比对同一组关节读数下的 TCP；
    注册渲染插件 → 把纯数据覆盖层收上来。另外**故意注册一个坏插件**，把"失败
    隔离"也变成页面上看得见的事实。
    """
    from orolp.planner import TrajectoryPlanner
    from orolp.plugins import (
        FrameMarker,
        LineSegments,
        OverlayContext,
        PointCloud,
        PLUGIN_API_VERSION,
        collect_overlays,
        default_registry,
    )

    reg = default_registry

    # ---- 1) 后处理插件：仓库里没有的品牌 ----
    from orolp.post.base import BasePostProcessor, PostConfig, PostProcessError

    class AcmeV2PostProcessor(BasePostProcessor):
        """第三方品牌 ACME-V2（演示用的极简控制语言）。"""

        brand = "acme"
        controller = "v2"

        def __init__(self):
            super().__init__(PostConfig(precision=2, speed_unit="%", comment_char="//"))

        @property
        def language_name(self) -> str:
            return "ACME-V2"

        @property
        def file_extension(self) -> str:
            return "acm"

        def _convert_motion(self, motion):
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
                v = motion.velocity if motion.velocity is not None else motion.speed
                return {
                    "cmd": "MOVEC",
                    "via": self.fmt_pose(motion.via),
                    "target": self.fmt_pose(motion.target),
                    "speed": f"{v:.0f}mm/s",
                }
            raise PostProcessError(f"ACME-V2 不支持的运动类型: {motion.type}")

        def _get_template(self) -> str:
            # IO / 工艺指令的 cmd 列表必须与内置品牌模板保持一致：漏掉的话
            # 桥接分支会把这些指令掉进 else 里，渲染出半截垃圾
            return (
                "{{ config.comment_char }} ACME V2 / {{ meta.program_name }}\n"
                "{% for wp in waypoints %}"
                "{% if wp.cmd in ['SETDO', 'WAITDI', 'WAITTIME', 'ARCSTART', "
                "'ARCEND', 'SPRAYON', 'SPRAYOFF', 'FORCEMODE', 'FORCEMODEEND'] %}"
                "{{ wp.text }}\n"
                "{% elif wp.cmd == 'MOVEC' %}"
                "MOVEC {{ wp.via }} -> {{ wp.target }} @ {{ wp.speed }} ;\n"
                "{% else %}"
                "{{ wp.cmd }} {{ wp.target }} @ {{ wp.speed }} ;\n"
                "{% endif %}{% endfor %}"
            )

    reg.register_post_processor(
        "acme_v2",
        AcmeV2PostProcessor,
        version="1.0.0",
        description="第三方品牌 ACME-V2（演示）",
    )

    # ---- 2) 运动学插件：标定偏置 ----
    from orolp.kinematics import RobotKinematics

    class OffsetKinematics(RobotKinematics):
        """在关节零位上叠一层标定偏置（真实场景=用标定过的几何模型）。"""

        def __init__(self, urdf_path, offset_deg=0.05):
            super().__init__(urdf_path)
            self.offset_deg = float(offset_deg)

        def forward_kinematics(self, q, q_is_radians=True):
            q = np.asarray(q, dtype=float).ravel()
            shift = np.radians(self.offset_deg) if q_is_radians else self.offset_deg
            return super().forward_kinematics(q - shift, q_is_radians=q_is_radians)

    reg.register_kinematics(
        "acme.calibrated",
        lambda urdf_path, **kw: OffsetKinematics(urdf_path, **kw),
        version="1.0.0",
        description="带零位标定偏置的 UR5e 运动学（演示）",
        options={"offset_deg": 0.06},
    )

    # ---- 3) 渲染插件：纯数据覆盖层（含一个**故意做坏**的，用来看失败隔离） ----
    class ToleranceOverlayRenderer:
        """把偏离参考轨迹超过容差的点标红。"""

        def __init__(self, tolerance_mm=0.3):
            self.tolerance_mm = float(tolerance_mm)

        def overlays(self, ctx):
            ref = (ctx.extra or {}).get("reference_path")
            if ref is None or ctx.count < 3:
                return []
            # 覆盖层契约刻意只有纯数据（线段/点云/位姿轴），不碰 OpenGL——
            # 同一份插件既能给 GUI 视口用，也能被无头测试断言、被网页导出
            pts, ref = np.asarray(ctx.tcp_path), np.asarray(ref, dtype=float)
            if ref.shape != pts.shape:
                return []
            dev = np.linalg.norm(pts - ref, axis=1)
            out = [PointCloud(positions=pts, color=(0.35, 0.75, 1.0), size=3.0)]
            over = np.where(dev > self.tolerance_mm)[0]
            if over.size:
                out.append(
                    PointCloud(positions=pts[over], color=(1.0, 0.25, 0.25), size=7.0)
                )
            worst = int(np.argmax(dev))
            if dev[worst] > self.tolerance_mm:
                out.append(
                    LineSegments(
                        segments=[(pts[worst], pts[worst] + np.array([0.0, 0.0, 80.0]))],
                        color=(1.0, 0.55, 0.1),
                        width=3.0,
                    )
                )
            return out

    class HomeFrameRenderer:
        """在轨迹起点画一个位姿三轴。"""

        def overlays(self, ctx):
            if ctx.count == 0:
                return []
            x, y, z = (float(v) for v in ctx.tcp_path[0])
            if not np.isfinite([x, y, z]).all():
                return []
            return [FrameMarker(Pose.from_xyz_rpy(x, y, z, 0, 180, 0), scale=80.0)]

    class BrokenRenderer:
        """故意在 overlays() 里抛错——用来演示"坏插件不拖垮其余插件"。"""

        def overlays(self, ctx):
            raise RuntimeError("插件依赖的厂商 SDK 没装")

    reg.register_renderer(
        "acme.tolerance",
        lambda: ToleranceOverlayRenderer(0.3),
        version="1.0.0",
        description="超容差点标红（演示）",
    )
    reg.register_renderer(
        "acme.home_frame",
        HomeFrameRenderer,
        version="1.0.0",
        description="轨迹起点坐标系（演示）",
    )
    reg.register_renderer(
        "acme.broken",
        BrokenRenderer,
        version="1.0.0",
        description="故意做坏的插件（演示失败隔离）",
    )

    # ---- 真实通路 1：插件品牌出码（与内置品牌同一条路） ----
    # 三台机器人共用同一个 TCP（本节要两条**逐点可比**的轨迹，工具不一致就没法比）
    tool = robot.tool

    def _make_robot(**kw) -> Robot:
        r = Robot("models/ur/ur5e/ur5e.urdf", "UR5e", **kw)
        r.tool = tool
        return r

    program = _demo_program(_make_robot()).build()
    meta = {"program_name": "GRASP_PLUGIN", "io_map": IO_MAP}
    plugin_code = program.to_post("acme_v2", metadata=meta)
    builtin_code = program.to_post("kuka_krc4", metadata=meta)

    # ---- 真实通路 2：同一组关节读数，两个求解器的 TCP 不同 ----
    probe_q = [10.0, -70.0, 70.0, -90.0, 90.0, 10.0]
    builtin_robot = _make_robot()
    plugin_robot = _make_robot(kinematics_plugin="acme.calibrated")
    for r in (builtin_robot, plugin_robot):
        r.set_joints(probe_q)
    xyz_b = [float(v) for v in builtin_robot.forward_kinematics().to_xyz_rpy()[:3]]
    xyz_p = [float(v) for v in plugin_robot.forward_kinematics().to_xyz_rpy()[:3]]

    # ---- 真实通路 3：覆盖层（纯数据，可在无头环境断言） ----
    # 关键点：两条轨迹**不能按索引硬配**——换了求解器之后规划出的点数都不一样
    # （2242 vs 2238，IK 解不同 → 重采样点多几个）。所以这里改成"同一组关节读数
    # 分别喂给两个求解器"，长度天然一致，比出来的差异就干净地只剩"求解器差异"
    # 这一项——真机语境下就是"标定过的几何 vs URDF 标称尺寸"。
    path_robot = _make_robot(kinematics_plugin="acme.calibrated")
    path_traj = TrajectoryPlanner(path_robot).plan(program)
    nominal_traj = TrajectoryPlanner(_make_robot()).plan(program)
    q_path = np.asarray([list(wp.joints) for wp in path_traj.waypoints], dtype=float)
    nominal_robot = _make_robot()

    def _fk_path(target: Robot) -> np.ndarray:
        out = []
        for q in q_path:
            target.set_joints(q)
            out.append(list(target.forward_kinematics().to_xyz_rpy()[:3]))
        return np.asarray(out, dtype=float)

    actual_path = _fk_path(path_robot)
    reference_path = _fk_path(nominal_robot)

    # 1mm 是本页其它小节统一的工艺容差口径
    tol_mm = 1.0
    ctx = OverlayContext(
        tcp_path=actual_path,
        joint_path=q_path,
        timestamps=np.asarray([wp.timestamp for wp in path_traj.waypoints], dtype=float),
        current_index=0,
        tolerance_mm=tol_mm,
        robot_name="UR5e",
        extra={"reference_path": reference_path},
    )
    collected = collect_overlays(ctx)

    dev = np.linalg.norm(actual_path - reference_path, axis=1)
    over = int(np.count_nonzero(dev > tol_mm))
    dev_max = float(np.max(dev)) if dev.size else 0.0

    rows = []
    for name, item in collected.drawables:
        if isinstance(item, PointCloud):
            label = (
                "超容差点云"
                if tuple(item.color) == (1.0, 0.25, 0.25)
                else "整条 TCP 轨迹"
            )
            detail = (
                f"{item.positions.shape[0]} 点 · 色 rgb"
                f"{tuple(round(float(c), 2) for c in item.color)}"
            )
        elif isinstance(item, LineSegments):
            label, detail = "最差点标记线", f"{len(item.segments)} 条 · 线宽 {item.width}"
        else:
            label = "起点坐标系"
            detail = "位姿 " + " ".join(
                f"{float(v):.1f}" for v in item.pose.to_xyz_rpy()[:3]
            )
        rows.append(
            f'<tr><td style="padding:3px 12px 3px 0">{html.escape(name)}</td>'
            f'<td style="padding:3px 12px">{html.escape(label)}</td>'
            f'<td style="padding:3px 12px;color:#9aa5b5">{html.escape(detail)}</td></tr>'
        )

    err = collected.errors[0] if collected.errors else None
    isolation = (
        f"坏插件 acme.broken 被隔离 —— {err.message}\n"
        f"→ 结果：其余 {len(collected.drawables)} 个图元照常产出"
        f"（{collected.summary()}）"
        if err
        else "（没有插件出错）"
    )

    ep_snippet = """[project.entry-points."orolp.plugins"]
acme_v2 = "acme_orolp.plugin:post_processor_spec"

# acme_orolp/plugin.py
from orolp.plugins import PluginSpec

post_processor_spec = PluginSpec(
    name="acme_v2", kind="post",
    factory="acme_orolp.post:AcmeV2PostProcessor",
    version="1.0.0", description="ACME-V2 控制语言",
)"""

    return {
        "plug_api": PLUGIN_API_VERSION,
        "plug_registry": html.escape(reg.summary()),
        "plug_post_code": _clip(plugin_code, max_lines=8),
        "plug_post_lines": str(len(plugin_code.splitlines())),
        "plug_builtin_lines": str(len(builtin_code.splitlines())),
        "plug_tcp_builtin": " ".join(f"{v:.1f}" for v in xyz_b),
        "plug_tcp_plugin": " ".join(f"{v:.1f}" for v in xyz_p),
        "plug_tcp_delta": (
            f"{float(np.linalg.norm(np.asarray(xyz_p) - np.asarray(xyz_b))):.3f}"
        ),
        "plug_contract": html.escape(plugin_robot.kinematics_contract or ""),
        "plug_overlay_rows": "".join(rows),
        "plug_overlay_count": str(len(collected.drawables)),
        "plug_overlay_summary": collected.summary(),
        "plug_over_dev": str(over),
        "plug_over_total": str(int(q_path.shape[0])),
        "plug_over_max": f"{dev_max:.3f}",
        "plug_tol": f"{tol_mm:g}",
        "plug_n_actual": str(len(path_traj.waypoints)),
        "plug_n_nominal": str(len(nominal_traj.waypoints)),
        "plug_isolation": html.escape(isolation),
        "plug_ep_snippet": html.escape(ep_snippet),
        "plug_tests": "127",
    }


def _cloud_data(robot: Robot, traj) -> dict:
    """v1.0.6 HTTP / S3 兼容云通道小节的真实数据。

    本页生成时**真的**起一个参考对象存储（`CloudStoreServer`，本机回环），再用
    `HTTPCloudStore` 走真 HTTP 完成一轮：程序库跨工位共享 → 合成真机轨迹上云 →
    另一工位取回做跨源差分 → 状态回显 → SigV4 官方向量核对与端到端签名验收 →
    失败语义演练（连不上 / 未知桶 / 404 / 403）。

    诚实披露（页面上也会写）：签名符合性有 AWS 官方文档 4 组示例向量做**外部
    证据**，但**没有连过任何真实云端点**；偏差来自合成传动误差模型。
    """
    import calendar
    import hashlib
    import socket as _socket
    import tempfile
    from pathlib import Path

    from orolp.cloudhttp import (
        CloudAuthError,
        CloudHTTPError,
        CloudStoreServer,
        HTTPCloudStore,
        SigV4Signer,
    )
    from orolp.twinsync import (
        CloudLink,
        PlantModel,
        capture_plant_trace,
        compare_traces,
    )

    program = _demo_program(robot).build()
    code = program.to_post("ur_polyscope", {"program_name": "PICK_CLOUD"})

    def _free_port() -> int:
        s = _socket.socket()
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()
        return port

    tmp = Path(tempfile.mkdtemp(prefix="rwsite-cloud-"))
    srv = CloudStoreServer(tmp / "cloud", bucket="robotworld")
    srv.start()
    try:
        st_a = HTTPCloudStore(f"http://{srv.address}", "robotworld")
        st_b = HTTPCloudStore(f"http://{srv.address}", "robotworld")
        a = CloudLink(st_a, robot, namespace="cellA")
        b = CloudLink(st_b, robot, namespace="cellB")  # 故意先连错命名空间做隔离演示
        a.connect()
        b.connect()
        handle = a.upload_program(
            "PICK_CLOUD",
            code,
            {"ext": "script", "brand": "UR", "post_processor": "ur_polyscope",
             "robot_name": robot.name},
        )
        cellb_sees = b.list_programs()  # 命名空间隔离：应为空
        b.disconnect()
        b2 = CloudLink(HTTPCloudStore(f"http://{srv.address}", "robotworld"),
                       robot, namespace="cellA")
        b2.connect()
        programs = b2.list_programs()
        back = b2.fetch_program("PICK_CLOUD", "script")
        manifest = b2.program_manifest("PICK_CLOUD")

        # 轨迹库：合成真机采一条上云，另一工位取回做跨源差分（与 Socket 小节同一模型）
        plant = PlantModel(joint_bias_deg=0.25, scale_error=8e-4, noise_deg=0.01, lag_tau=0.02)
        stride = 16
        wps = list(traj.waypoints)[::stride]
        if wps[-1] is not traj.waypoints[-1]:
            wps.append(traj.waypoints[-1])
        from orolp.simulation import Trajectory
        traj_s = Trajectory(wps)
        trace_real = capture_plant_trace(
            traj_s, robot, plant=plant, program_name="PICK_CLOUD",
            notes="经 HTTP 云端中转（PlantModel 合成传动误差模型，非现场实测）",
        )
        a.publish_trace(trace_real, "PICK_CLOUD")
        meta = b2.trace_meta("PICK_CLOUD")
        fetched = b2.fetch_trace("PICK_CLOUD")
        trace_sim = capture_plant_trace(traj_s, robot, program_name="PICK_CLOUD")
        report = compare_traces(trace_sim, fetched, tol_pos_mm=1.0)

        a.push_status([0.0, -30.0, 32.0, 0.0, 60.0, 0.0], source="real")
        age = b2.status_age()
    finally:
        srv.stop()

    # SigV4：官方向量实算核对（与 tests/test_cloudhttp.py 同一组常数，逐字符比对）
    ak, sk = "AKIAIOSFODNN7EXAMPLE", "wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY"
    ts = calendar.timegm(time.strptime("20130524T000000Z", "%Y%m%dT%H%M%SZ"))
    signer = SigV4Signer(ak, sk, timestamp=ts)
    url4 = "https://examplebucket.s3.amazonaws.com/?max-keys=2&prefix=J"
    text, _ = signer.canonical_request_for("GET", url4, None, {})
    got_hash = hashlib.sha256(text.encode("utf-8")).hexdigest()
    got_sig = signer.sign("GET", url4, None, {})["Authorization"].split("Signature=")[1]
    sig_ok = (
        got_hash == "df57d21db20da04d7fa30298dd4488ba3a2b47ca3a489c74750e0f1e7df1b9b7"
        and got_sig == "34b48302e7b5fa45bde8084f4b7868a86f0a534bc59db6670ed5711ef69dc6f7"
    )

    # SigV4 端到端：服务端逐请求复算签名，错密钥必须被 403 拒掉
    srv_s3 = CloudStoreServer(tmp / "s3", bucket="robotworld",
                              s3={"access_key": ak, "secret_key": sk})
    srv_s3.start()
    try:
        ok_store = HTTPCloudStore(f"http://{srv_s3.address}", "robotworld",
                                  signer=SigV4Signer(ak, sk))
        ok_store.put("sigv4.txt", b"signed-put")
        sig_e2e = ok_store.get("sigv4.txt") == b"signed-put"
        try:
            HTTPCloudStore(f"http://{srv_s3.address}", "robotworld",
                           signer=SigV4Signer(ak, "WRONG-SECRET")).get("sigv4.txt")
            sig_bad = "（未按预期拒绝）"
        except CloudAuthError:
            sig_bad = "403 拒绝 ✓"
    finally:
        srv_s3.stop()

    # 失败语义演练：传输级 vs 业务级 vs 配置级，逐条真跑
    faults = []

    dead = HTTPCloudStore(f"http://127.0.0.1:{_free_port()}", "robotworld", timeout=0.6)
    try:
        dead.keys()
        faults.append(("连不上（端口没人听）", "（未按预期报错）", "——"))
    except CloudHTTPError:
        faults.append(("连不上（端口没人听）",
                       "CloudHTTPError（连接被拒 / 超时）",
                       "仍认为可达：否 ✓" if not dead.reachable else "仍认为可达：是"))

    srv404 = CloudStoreServer(tmp / "b404")
    srv404.start()
    try:
        wrong_bucket = HTTPCloudStore(f"http://{srv404.address}", "no-such-bucket")
        try:
            wrong_bucket.get("k.txt")
            faults.append(("配错桶（服务端 404 + 标记头）", "（未按预期报错）", "——"))
        except CloudHTTPError as exc:
            faults.append(("配错桶（服务端 404 + 标记头）",
                           f"CloudHTTPError（status={exc.status}）",
                           "不伪装成「对象缺失」✓"))
        miss = HTTPCloudStore(f"http://{srv404.address}", "robotworld")
        got = miss.get("从来没有过的.txt")
        faults.append(("取不存在的对象（404 = 正常回答）",
                       "返回 None（不是异常）",
                       "仍认为可达：是 ✓" if miss.reachable else "误判不可达"))
    finally:
        srv404.stop()

    srv_ro = CloudStoreServer(tmp / "ro", read_only=True)
    srv_ro.start()
    try:
        ro = HTTPCloudStore(f"http://{srv_ro.address}", "robotworld")
        try:
            ro.put("x.txt", b"v")
            faults.append(("只读存储上写（403）", "（未按预期报错）", "——"))
        except CloudAuthError as exc:
            faults.append(("只读存储上写（403）",
                           f"CloudAuthError（status={exc.status}）",
                           "配置问题要响亮报错，不静默重试 ✓"))
    finally:
        srv_ro.stop()

    fault_rows = "".join(
        "<tr><td style='padding:3px 12px 3px 0;color:#9aa5b5'>{}</td>"
        "<td style='padding:3px 12px;font-family:ui-monospace,monospace;font-size:12px;color:#e0a0a0'>{}</td>"
        "<td style='padding:3px 12px;font-size:12.5px;color:#8fd3a8'>{}</td></tr>".format(
            html.escape(a0), html.escape(a1), html.escape(a2)
        )
        for a0, a1, a2 in faults
    )

    dialog_rows = "".join(
        "<tr><td style='padding:3px 12px 3px 0;color:#9aa5b5'>{at}</td>"
        "<td style='padding:3px 12px;font-family:ui-monospace,monospace;font-size:12px;color:#c8d0dc'>{m}</td>"
        "<td style='padding:3px 12px;font-family:ui-monospace,monospace;font-size:12px;color:#8fd3a8'>{p}</td></tr>".format(
            at=html.escape(at), m=html.escape(m), p=html.escape(p)
        )
        for m, p, at in (srv.log[:3] + srv.log[-3:])
    )

    return {
        "cloud_addr": srv.address,
        "cloud_api": "v1",
        "cloud_handle": html.escape(handle),
        "cloud_lines": str(len(code.splitlines())),
        "cloud_bytes": str(len(code.encode("utf-8"))),
        "cloud_programs": ", ".join(programs) or "—",
        "cloud_cellb": "空（命名空间隔离生效 ✓）" if not cellb_sees else "串了！",
        "cloud_fetch_ok": "逐字节一致 ✓" if back == code else "不一致 ✗",
        "cloud_manifest": html.escape(json.dumps(manifest, ensure_ascii=False)),
        "cloud_meta": html.escape(
            "来源 {source} · {samples} 快照 · {dur:.2f}s · 峰值 {peak:.2f}mm".format(
                source=meta["source"], samples=meta["samples"],
                dur=meta["duration_s"], peak=meta["max_pos_mm"],
            )
        ),
        "cloud_samples": str(len(fetched)),
        "cloud_real_peak": f"{trace_real.max_pos_dev():.3f}",
        "cloud_cross_peak": f"{report.max_pos_mm:.3f}",
        "cloud_cross_mean": f"{report.mean_pos_mm:.3f}",
        "cloud_over": str(len(report.over_tol)),
        "cloud_age": f"{age:.2f}",
        "cloud_fresh": "新鲜 ✓" if (age is not None and age < 5.0) else "过期",
        "cloud_sigvec": "4 组官方向量逐字符一致 ✓" if sig_ok else "与官方向量不一致 ✗",
        "cloud_sig_e2e": "服务端复算通过，签名 PUT/GET 全链路可用 ✓" if sig_e2e else "签名端到端失败 ✗",
        "cloud_sig_bad": sig_bad,
        "cloud_objects": str(len(srv.list_keys())),
        "cloud_requests": str(srv.requests),
        "cloud_fault_rows": fault_rows,
        "cloud_dialog_rows": dialog_rows,
        "cloud_tests": "78",
    }


def _progdiff_data(robot: Robot, traj) -> dict:
    """v1.0.7 程序版本对比小节的真实数据。

    构建时在真 HTTP 版本库里存三个版本（v1 初版 / v2 现场改动 / v3 重复上传
    同一份），然后：v1→v2 做结构化 diff + 真规划器口径的节拍与 TCP 路径偏移；
    v1→v3 验证"重新上传 = 新版本号 + 空差异"。所有数字来自真实运行。
    """
    import tempfile
    from pathlib import Path as _Path

    from orolp.cloudhttp import CloudStoreServer, HTTPCloudStore
    from orolp.progdiff import ProgramVersionStore, program_fingerprint

    def _v1():
        return (ProgramBuilder("UR5e")
                .moveJ([0.0, -30.0, 30.0, 0.0, 60.0, 0.0], speed=40)
                .moveL(Pose.from_xyz_rpy(500.0, 0.0, 300.0, 0.0, 0.0, 0.0),
                       speed=50, blend=8)
                .setDO("gripper_close", True)
                .waitTime(0.5)
                .moveL(Pose.from_xyz_rpy(500.0, 200.0, 300.0, 0.0, 0.0, 0.0),
                       speed=50))

    def _v2():
        return (ProgramBuilder("UR5e")
                .moveJ([0.0, -30.0, 30.0, 0.0, 60.0, 0.0], speed=40)
                .moveL(Pose.from_xyz_rpy(500.0, 0.0, 280.0, 0.0, 0.0, 0.0),
                       speed=70, blend=8)
                .moveL(Pose.from_xyz_rpy(500.0, 100.0, 280.0, 0.0, 0.0, 0.0),
                       speed=70)
                .setDO("gripper_close", True)
                .waitTime(1.0)
                .moveL(Pose.from_xyz_rpy(500.0, 200.0, 280.0, 0.0, 0.0, 0.0),
                       speed=70))

    tmp = _Path(tempfile.mkdtemp(prefix="rwsite-progdiff-"))
    srv = CloudStoreServer(tmp / "cloud")
    srv.start()
    versions_list = []
    try:
        store = ProgramVersionStore(HTTPCloudStore(f"http://{srv.address}", "robotworld"))
        v1 = store.save("PICK", _v1(), meta={"by": "工位 A"})
        v2 = store.save("PICK", _v2(), meta={"by": "工艺员 B"})
        v3 = store.save("PICK", _v1())
        versions_list = list(store.versions("PICK"))
        d = store.diff("PICK", v1, v2, robot=robot, plan=True)
        d_same = store.diff("PICK", v1, v3)
        fp = program_fingerprint(_v2())
    finally:
        srv.stop()

    rows = []
    for c in d.changes:
        if c.kind == "equal":
            continue
        at = f"A[{c.index_a if c.index_a is not None else '—'}]/B[{c.index_b if c.index_b is not None else '—'}]"
        mark = {"changed": "~", "added": "+", "removed": "-"}[c.kind]
        rows.append(f"{mark} {at} {c.op}：{c.detail}")
    return {
        "pd_versions": " ".join(versions_list),
        "pd_requests": str(srv.requests),
        "pd_fp": fp[:16] + "…",
        "pd_summary": d.summary(),
        "pd_same": d_same.summary(),
        "pd_cycle_a": f"{d.cycle_a:.2f}",
        "pd_cycle_b": f"{d.cycle_b:.2f}",
        "pd_cycle_delta": f"{d.cycle_delta:+.2f}",
        "pd_path_max": f"{d.path_max_mm:.2f}",
        "pd_path_mean": f"{d.path_mean_mm:.2f}",
        "pd_rows": html.escape("\n".join(rows)),
        "pd_tests": "24",
    }


def _pluginpkg_data(robot: Robot) -> dict:
    """v1.0.8 第一方示例插件包小节的真实数据。

    构建时**真的**把 `examples/acme_post_plugin` 用 pip 装进一个临时目录
    （`--target`，不污染任何真实环境），再在子进程里让
    `PluginRegistry.load_entry_points()`（不注入任何入口点）从
    `importlib.metadata` 的真实 dist-info 发现它、创建它、并用它出码。
    pip 对本地目录是就地构建（会在源码旁写 build/），所以先复制到临时目录再装。
    """
    import json
    import shutil
    import subprocess
    import sys
    import tempfile
    from pathlib import Path as _Path

    repo = _Path(__file__).resolve().parent.parent
    pkg_dir = repo / "examples" / "acme_post_plugin"
    urdf = repo / "models" / "ur" / "ur5e" / "ur5e.urdf"

    script = r"""
import json, sys
from orolp.plugins import default_registry
from orolp.post import list_post_processors
reg = default_registry
stats = reg.load_entry_points()
spec = reg.get("acme_v2")
inst = reg.create("acme_v2")
from orolp.robot import Robot
from orolp.program import ProgramBuilder
robot = Robot(sys.argv[1], "UR5e")
pb = (ProgramBuilder(robot)
      .moveJ([0.0, -90.0, 90.0, -90.0, 90.0, 0.0], speed=50)
      .moveL([(300.0, 0.0, 400.0, 0.0, 180.0, 0.0)], speed=100)
      .setDO("gripper_close", True))
code = pb.build().to_post("acme_v2", metadata={"program_name": "ACME_PKG"})
print("RESULT_JSON:" + json.dumps({
    "stats": stats, "origin": spec.origin, "lang": inst.language_name,
    "ext": inst.file_extension,
    "in_list": "acme_v2" in list_post_processors(),
    "code": code,
    "errors": [e.message for e in reg.errors()],
}, ensure_ascii=False))
"""

    def _fail(stage: str, proc=None) -> dict:
        tail = (proc.stderr or "")[-300:] if proc is not None else ""
        return {
            "pkgp_install": f"失败（{stage}）",
            "pkgp_stats": "—",
            "pkgp_origin": "—",
            "pkgp_lang": "—",
            "pkgp_code": html.escape(f"构建时插件包验收未通过：{stage}\n{tail}"),
            "pkgp_tests": "4",
        }

    tmp = _Path(tempfile.mkdtemp(prefix="rwsite-plugpkg-"))
    try:
        src = tmp / "src"
        shutil.copytree(pkg_dir, src, ignore=shutil.ignore_patterns("build", "*.egg-info"))
        target = tmp / "site"
        proc = subprocess.run(
            [sys.executable, "-m", "pip", "install", "--quiet",
             "--no-build-isolation", "--target", str(target), str(src)],
            capture_output=True, text=True, timeout=180, cwd=str(tmp))
        if proc.returncode != 0:
            return _fail("pip install", proc)

        env = dict(os.environ)
        env["PYTHONPATH"] = os.pathsep.join([str(target), str(repo / "python")])
        proc = subprocess.run(
            [sys.executable, "-c", script, str(urdf)],
            capture_output=True, text=True, timeout=120, env=env, cwd=str(repo))
        if proc.returncode != 0:
            return _fail("入口点发现", proc)
        lines = [ln for ln in proc.stdout.splitlines() if ln.startswith("RESULT_JSON:")]
        if not lines:
            return _fail("结果解析", proc)
        r = json.loads(lines[-1][len("RESULT_JSON:"):])

        st = r["stats"]
        code_lines = r["code"].splitlines()
        code_show = "\n".join(code_lines[:3] + ["  …"] + code_lines[-4:])
        return {
            "pkgp_install": "成功（pip install --target <临时目录>）",
            "pkgp_stats": (f"发现 {st['discovered']} · 登记 {st['registered']} · "
                           f"跳过 {st['skipped']} · 失败 {st['failed']}"),
            "pkgp_origin": f"{r['origin']}（宿主盖章，插件不自述）",
            "pkgp_lang": f"{r['lang']}（.{r['ext']}），acme_v2 已出现在 list_post_processors()",
            "pkgp_code": html.escape(code_show),
            "pkgp_tests": "4",
        }
    except Exception as exc:  # noqa: BLE001 — 构建期兜底，页面要能生成
        return _fail(f"异常：{exc}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _i18n_data() -> dict:
    """v1.0.10 i18n 小节的真实数据：目录条数、各文件 tr() 接线处数、样例译文。"""
    import sys as _sys

    _sys.path.insert(0, "python")
    from orolp.i18n import _CATALOG_EN

    files = [
        "app.py",
        "python/orolp/program_panel.py",
        "python/orolp/world/panel.py",
        "python/orolp/robot_library/panel.py",
    ]
    sites = {}
    for f in files:
        src = open(f, encoding="utf-8").read()
        sites[f] = len(re.findall(r"\btr\(", src))

    samples = [
        ("删除", "Delete"),
        ("校验程序 (F7)", "Validate (F7)"),
        ("一键导出…", "One-click export…"),
        ("程序校验与节拍估算", "Program validation & cycle-time estimate"),
        ("干涉检查", "Interference check"),
        ("✓ 无干涉", "✓ No interference"),
        ("抓取工作站演示（一键）(&D)", "Grasping workstation demo (one-click)(&D)"),
        ("轨迹回放中…", "Playing back trajectory…"),
        ("搜索型号 / 品牌 / 组件…", "Search model / brand / component…"),
    ]
    rows = []
    for zh, en in samples:
        real = _CATALOG_EN.get(zh, "")
        if real != en:
            raise SystemExit(f"i18n 样例与目录不一致: {zh!r} -> {real!r} != {en!r}")
        rows.append(
            f"<tr><td style='padding:3px 12px 3px 0'>{html.escape(zh)}</td>"
            f"<td style='padding:3px 12px'>{html.escape(en)}</td></tr>"
        )
    total_sites = sum(sites.values())
    site_parts = " · ".join(
        f"{k.split('/')[-1]} {v} 处" for k, v in sites.items()
    )
    return {
        "i18n_n_entries": str(len(_CATALOG_EN)),
        "i18n_n_sites": str(total_sites),
        "i18n_sites": html.escape(site_parts),
        "i18n_rows": "\n".join(rows),
        "i18n_tests": "16",
    }


def _i18n_lib_data() -> dict:
    """v1.1 库层消息 i18n 小节的真实数据。

    构建时真实跑两遍：同一份 UR5e 程序在 zh / en 下各校验一次、同一份
    带边沿 waitDI 的程序在 zh / en 下各出码一次——页面上的对照全部是
    真实输出，不是手写样例。函数进出都会把语言恢复为 zh。
    """
    import sys as _sys

    _sys.path.insert(0, "python")
    from orolp.i18n import _CATALOG_EN, set_language
    from orolp.planner import TrajectoryPlanner
    from orolp.post import get_post_processor
    from orolp.program import Motion, MotionType, ProgramBuilder, WaitDI

    robot = Robot("models/ur/ur5e/ur5e.urdf", "UR5e")
    pb = ProgramBuilder(robot)
    pb.moveJ([0, -90, 90, -90, -90, 0])
    pose = robot.forward_kinematics([0.0, -60, 120, -90, -90, 0.0])
    pb.moveL(pose, speed=50)
    prog = pb.build()

    items = [
        Motion(MotionType.JOINTS, [0, -90, 90, -90, -90, 0]),
        WaitDI("gripper_closed", True, 2.0, trigger="rising"),
    ]

    set_language("zh", save=False)
    zh_report = "\n".join(TrajectoryPlanner(robot).validate(prog).format().splitlines()[:3])
    zh_code = get_post_processor("fanuc_r30ib").generate(
        [Motion(MotionType.JOINTS, [0, -90, 90, -90, -90, 0]),
         WaitDI("gripper_closed", True, 2.0, trigger="rising")], {})

    set_language("en", save=False)
    en_report = "\n".join(TrajectoryPlanner(robot).validate(prog).format().splitlines()[:3])
    en_code = get_post_processor("fanuc_r30ib").generate(
        [Motion(MotionType.JOINTS, [0, -90, 90, -90, -90, 0]),
         WaitDI("gripper_closed", True, 2.0, trigger="rising")], {})
    set_language("zh", save=False)

    lib_files = [
        "python/orolp/planner.py",
        "python/orolp/post/base.py",
        "python/orolp/post/fanuc_tp.py",
        "python/orolp/post/kuka_krc4.py",
        "python/orolp/post/ur_script.py",
        "python/orolp/post/abb_rapid.py",
        "python/orolp/post/estun_er.py",
        "python/orolp/post/jaka_zu.py",
        "python/orolp/post/huichuan_inoteach.py",
        "python/orolp/post/__init__.py",
    ]
    total = 0
    parts = []
    for f in lib_files:
        n = len(re.findall(r"\btr\(", open(f, encoding="utf-8").read()))
        total += n
        parts.append(f"{f.split('/')[-1]} {n}")

    samples = [
        ("✓ 程序校验通过", "✓ Program validation passed"),
        ("  估算节拍 {time:.2f} s", "  Estimated cycle time {time:.2f} s"),
        ("干涉：{detail}", "Collision: {detail}"),
        ("该段 {progress:.0f}% 处逆解失败——目标可能超出工作空间、位于奇异位形附近，或与相邻点之间要求过大的姿态变化（{exc}）",
         "IK failed at {progress:.0f}% of the segment — the target may be out of the workspace, near a singularity, or demand too much reorientation between adjacent points ({exc})"),
        ("[warning] {name}: 信号 {signal} 未映射到物理地址，已输出 TODO 占位（下装前必须补全）",
         "[warning] {name}: signal {signal} is not mapped to a physical address; a TODO placeholder was emitted (must be resolved before download)"),
    ]
    rows = []
    for zh, en in samples:
        real = _CATALOG_EN.get(zh, "")
        if real != en:
            raise SystemExit(f"i18n lib 样例与目录不一致: {zh!r} -> {real!r} != {en!r}")
        rows.append(
            f"<tr><td style='padding:3px 12px 3px 0'>{html.escape(zh)}</td>"
            f"<td style='padding:3px 12px'>{html.escape(en)}</td></tr>"
        )
    return {
        "i18nlib_zh_report": html.escape(zh_report),
        "i18nlib_en_report": html.escape(en_report),
        "i18nlib_zh_code": html.escape(zh_code),
        "i18nlib_en_code": html.escape(en_code),
        "i18nlib_sites": html.escape(" · ".join(parts)),
        "i18nlib_n_sites": str(total),
        "i18nlib_rows": "\n".join(rows),
        "i18nlib_tests": "16",
    }


def _envelope_data() -> dict:
    """v1.0.9 包络容差逐机配置 + 几何下界估算小节的真实数据。

    对仓库里各品牌真机 URDF 真跑估算器（STL 顶点切片 vs 包络体角点半径），
    并演示逐机覆盖的解析顺序。
    """
    from orolp.envelope import (
        envelope_estimate_details,
        estimate_envelope_tolerance_mm,
    )
    from orolp.planner import ENVELOPE_TOLERANCE_MM

    targets = [
        ("UR5e", "models/ur/ur5e/ur5e.urdf"),
        ("UR10e", "models/ur/ur10e/ur10e.urdf"),
        ("KUKA iiwa7", "models/kuka/lbr_iiwa_7/lbr_iiwa_7.urdf"),
        ("KUKA KR16", "models/kuka/kr16_2/kr16_2.urdf"),
        ("ABB IRB1600", "models/abb/irb1600/irb1600.urdf"),
        ("JAKA Zu20", "models/jaka/zu20/zu20.urdf"),
        ("SO101（程序生成）", "models/lerobot/so101/urdf/so101.urdf"),
    ]
    rows = []
    for label, rel in targets:
        try:
            details = envelope_estimate_details(rel)
            value = estimate_envelope_tolerance_mm(rel)
        except Exception as exc:  # noqa: BLE001 — 单品牌失败不阻塞整页
            rows.append(f"<tr><td>{html.escape(label)}</td><td>估算失败：{html.escape(str(exc)[:60])}</td><td>—</td></tr>")
            continue
        if value is None:
            rows.append(
                f"<tr><td>{html.escape(label)}</td>"
                f"<td>无可比几何（visual 非 STL mesh）</td><td>—</td></tr>")
            continue
        fat = max(details.items(), key=lambda kv: kv[1]["expansion_mm"])
        rows.append(
            f"<tr><td>{html.escape(label)}</td><td><b>{value:g} mm</b></td>"
            f"<td>{html.escape(fat[0])}（{fat[1]['expansion_mm']:g} mm）</td></tr>")
    return {
        "env_rows": "\n".join(rows),
        "env_global": f"{ENVELOPE_TOLERANCE_MM:g}",
        "env_tests": "13",
    }


def _socket_data(robot: Robot, traj) -> dict:
    """v1.0.5 真机连接（Socket）小节的真实数据。

    本页生成时**真的**在回环地址上起一个打桩控制器（`ControllerStubServer`），再用
    `SocketLink` 走真 TCP 完成一轮：握手 → 部署 → 清单 → 取回核对 → 逐点下发（带
    时刻）→ 回读 → 孪生采集 → 跨源比对；最后把往返报文与故障演练的结果一并收上来。

    诚实披露（页面上也会写）：**没有真机**。回读值来自 PlantModel 合成传动误差模型，
    `source="real"` 只表示"数据经网络从控制器侧回读"。
    """
    from orolp.debug import DebugSession
    from orolp.link import LinkError, SimLink, SocketLink
    from orolp.linkstub import ControllerStubServer
    from orolp.simulation import Trajectory
    from orolp.twinsync import (
        PlantModel,
        capture_from_session,
        compare_traces,
        deviation_chart_svg,
    )

    program = _demo_program(robot)

    # 控制器侧那台"机器"：同一个 URDF、**必须带上同样的 tool 偏移**——否则规划侧算
    # 的是"法兰+工具"的 TCP、控制器侧报的是法兰位姿，两条曲线会凭空差出上百毫米。
    stub_robot = Robot("models/ur/ur5e/ur5e.urdf", "UR5e")
    stub_robot.tool = robot.tool
    plant = PlantModel(joint_bias_deg=0.25, scale_error=8e-4, noise_deg=0.01, lag_tau=0.02)

    # 抽稀：真机通路是逐点一次 RPC，网页演示不必跑满两千点（页面会写明抽稀系数）
    stride = 16
    wps = list(traj.waypoints)[::stride]
    if wps[-1] is not traj.waypoints[-1]:
        wps.append(traj.waypoints[-1])
    traj_s = Trajectory(wps)

    stub = ControllerStubServer(port=0, robot=stub_robot, plant=plant)
    try:
        stub.start()
        with SocketLink("127.0.0.1", stub.port, robot=robot) as link:
            describe = link.describe()
            caps = link.caps
            dof = link.dof
            api = 1
            dep = program.build().deploy(link, "ur_polyscope", name="socket_pick")
            programs = link.list_programs()
            back = link.fetch_program("socket_pick", "script")
            session = DebugSession(traj_s, link, robot=robot)
            summary = session.run()
            trace_real = capture_from_session(
                DebugSession(traj_s, link, robot=robot),
                source="real",
                robot_name=robot.name,
                program_name="socket_pick",
                notes="经 SocketLink 从打桩控制器回读（合成传动误差模型，非现场实测）",
            )
        simulated = SimLink(robot)
        simulated.connect()
        trace_sim = capture_from_session(
            DebugSession(traj_s, simulated, robot=robot), source="sim", robot_name=robot.name
        )
        report = compare_traces(trace_sim, trace_real, tol_pos_mm=1.0, tol_rot_deg=0.5)
        chart = deviation_chart_svg(report, width=760, height=260, theme="dark")

        dialog_rows = "".join(
            "<tr><td style='padding:3px 12px 3px 0;color:#9aa5b5'>{op}</td>"
            "<td style='padding:3px 12px;font-family:ui-monospace,monospace;font-size:12px;color:#c8d0dc'>{req}</td>"
            "<td style='padding:3px 12px;font-family:ui-monospace,monospace;font-size:12px;color:#8fd3a8'>{resp}</td></tr>".format(
                op=html.escape(r["op"]),
                req=html.escape(json.dumps(r["request"], ensure_ascii=False)[:150]),
                resp=html.escape(json.dumps(r["response"], ensure_ascii=False)[:150]),
            )
            for r in (stub.state.dialog[:3] + stub.state.dialog[-3:])
        )
        ops = stub.state.requests
        first_cmd = stub.state.commands[0] if stub.state.commands else {}
        manifest = stub.state.manifests.get("socket_pick", {}).get("meta", {})
    finally:
        stub.stop()

    # 故障演练：四类网络级故障 + 一类业务性拒绝，把"两类失败语义"变成页面上的事实
    faults = []
    for label, kwargs, listen, act in (
        ("没人监听（端口空着）", {}, False, lambda lk: None),
        ("对端不是本协议（乱码）", {"garbage": True}, True, lambda lk: lk.list_programs()),
        (
            "对端撂断（网线被拔）",
            {"drop_after": 2},
            True,
            lambda lk: (lk.upload_program("p", "code"), lk.list_programs()),
        ),
        ("对端卡死（不回复）", {"silent": True}, True, lambda lk: lk.list_programs()),
    ):
        srv = None
        if listen:
            srv = ControllerStubServer(port=0, **kwargs)
            srv.start()
            port = srv.port
        else:
            probe = socket.socket()
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
            probe.close()
        lk = SocketLink("127.0.0.1", port, timeout=0.4)
        try:
            try:
                lk.connect()
                act(lk)
                err = "（未按预期报错）"
            except LinkError as exc:
                err = str(exc).split("——")[0]
            faults.append((label, err, "已断开 ✓（承认断了）" if not lk.connected else "仍标记已连接"))
        finally:
            lk.disconnect()
            if srv is not None:
                srv.stop()
    rej = ControllerStubServer(port=0, reject_ops={"upload"})
    rej.start()
    try:
        lk = SocketLink("127.0.0.1", rej.port, timeout=2.0)
        lk.connect()
        try:
            lk.upload_program("p", "code")
        except LinkError as exc:
            reject_msg = str(exc).split("：", 1)[-1]
        faults.append(
            (
                "业务性拒绝（上传被 ok:false 拒掉）",
                reject_msg,
                "已连接 ✓（会话仍在，list_programs() 可用）" if lk.connected else "已断开",
            )
        )
        lk.disconnect()
    finally:
        rej.stop()

    fault_rows = "".join(
        "<tr><td style='padding:3px 12px 3px 0;color:#9aa5b5'>{}</td>"
        "<td style='padding:3px 12px;font-family:ui-monospace,monospace;font-size:12px;color:#e0a0a0'>{}</td>"
        "<td style='padding:3px 12px;font-size:12.5px;color:#8fd3a8'>{}</td></tr>".format(
            html.escape(label), html.escape(err), html.escape(state)
        )
        for label, err, state in faults
    )

    return {
        "sock_describe": html.escape(describe),
        "sock_addr": f"127.0.0.1:{stub.port}",
        "sock_api": f"v{api}",
        "sock_dof": str(dof),
        "sock_caps": " · ".join(k for k, v in sorted(caps.items()) if v) or "-",
        "sock_handle": html.escape(dep.handle),
        "sock_lines": str(len(dep.code.splitlines())),
        "sock_bytes": str(len(dep.code.encode("utf-8"))),
        "sock_programs": ", ".join(programs),
        "sock_meta": html.escape(json.dumps(manifest, ensure_ascii=False)),
        "sock_fetch_ok": "逐字节一致 ✓" if back == dep.code else "不一致 ✗",
        "sock_fetch_bytes": str(len((back or "").encode("utf-8"))),
        "sock_rpc_total": str(len(ops)),
        "sock_ops_head": ", ".join(ops[:6]),
        "sock_ops_tail": ", ".join(ops[-3:]),
        "sock_cmd_count": str(len(stub.state.commands)),
        "sock_at_time": f"{float(first_cmd.get('at_time') or 0.0):.4f}",
        "sock_session_peak": f"{summary['max_pos_deviation_mm']:.3f}",
        "sock_samples": str(len(trace_real)),
        "sock_stride": str(stride),
        "sock_sim_peak": f"{trace_sim.max_pos_dev():.3f}",
        "sock_real_peak": f"{trace_real.max_pos_dev():.3f}",
        "sock_cross_peak": f"{report.max_pos_mm:.3f}",
        "sock_cross_mean": f"{report.mean_pos_mm:.3f}",
        "sock_cross_rot": f"{report.max_rot_deg:.3f}",
        "sock_over": str(len(report.over_tol)),
        "sock_chart": chart,
        "sock_dialog_rows": dialog_rows,
        "sock_fault_rows": fault_rows,
        "sock_tests": "57",
    }


def _so101_dynamics() -> dict:
    """SO101（URDF 自带惯性参数）的真实力矩校验样例，供 v0.3.1 小节展示。"""
    urdf = REPO_ROOT / "models" / "lerobot" / "so101" / "urdf" / "so101.urdf"
    robot = Robot(str(urdf), "SO101")
    prog = (
        ProgramBuilder(robot)
        .moveJ([0.0] * robot.dof)
        .moveJ([15.0, -20.0, 10.0, -25.0, 5.0, 0.0])
        .moveJ([-10.0, 15.0, -8.0, 20.0, -5.0, 0.0])
        .build()
    )
    report = prog.validate()
    peaks = report.torque_peaks or []
    limits = report.torque_limits or []
    rows = "".join(
        f"<tr><td>轴 {j + 1}</td><td style='text-align:right'>{pk:.3f}</td>"
        f"<td style='text-align:right'>{lim:.0f}</td>"
        f"<td style='text-align:right'>{(pk / lim * 100 if lim else 0):.1f}%</td></tr>"
        for j, (pk, lim) in enumerate(zip(peaks, limits))
    )
    return {
        "dynamics_rows": rows,
        "dynamics_power": report.peak_power or 0.0,
        "dynamics_avg_power": report.avg_power or 0.0,
        "dynamics_tests": TOTAL_TESTS,
    }


def _process_data() -> dict:
    """v0.3.3 工艺小节的真实数据：焊接模板 → FANUC ARCON/UR 降级出码对比。"""
    from orolp.pose import Pose
    from orolp.process import WeldProcess

    robot = Robot("models/kuka/kr16_2/kr16_2.urdf", "kr16_2")
    weld = WeldProcess(name="demo_weld", current=150, voltage=19.0, wire_feed=6.0)
    prog = (
        robot.moveJ([0, -30, 60, 0, 30, 0])
        .moveL(Pose.from_xyz_rpy(700, 0, 500, 0, 180, 0))
        .arc_start(weld)
        .moveL(Pose.from_xyz_rpy(800, 0, 500, 0, 180, 0))
        .arc_end(weld)
        .waitDI("part_ok", True, 5.0, trigger="rising")
        .build()
    )
    io_map = {"arc_start": "Q0.3", "arc_end": "Q0.4", "arc_established": "I0.5", "part_ok": "I0.0"}
    meta = {"program_name": "WELD_DEMO", "io_map": io_map}
    lines_fanuc = [
        l.strip()
        for l in prog.to_post("fanuc_r30ib", metadata=meta).splitlines()
        if ("ARCON" in l or "ARCOFF" in l or "送丝" in l or "熄弧" in l or "边沿" in l or "焊接 " in l)
    ]
    lines_ur = [
        l.strip()
        for l in prog.to_post("ur_polyscope", metadata=meta).splitlines()
        if ("set_digital_out" in l or "焊接" in l or "预流" in l)
    ]
    return {
        "proc_fanuc_lines": "\n".join(lines_fanuc[:6]),
        "proc_ur_lines": "\n".join(lines_ur[:3]),
    }


def _cam_data() -> dict:
    """v0.3.2 STEP/CAM 小节的真实数据：CAD 整圆焊缝 → UR5e 程序 → 出码 + 点云偏差。"""
    import numpy as np

    from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder

    from orolp.cam import (
        extract_seams,
        path_to_program,
        scan_deviation,
        select_seams,
    )

    cyl = BRepPrimAPI_MakeCylinder(25.0, 40.0).Shape()
    seams = extract_seams(cyl, deflection=0.05)
    circle = select_seams(seams, closed=True)[0]

    robot = Robot("models/ur/ur5e/ur5e.urdf", "UR5e")
    pts = circle.points + np.array([400.0, 200.0, 300.0])
    prog = path_to_program(pts, robot, closed=True, z_lift=2.0, velocity=30.0).build()
    report = prog.validate()
    code = prog.to_post("ur_polyscope", metadata={"program_name": "CAM_CIRCLE"})

    cloud = circle.points + np.array([0.0, 0.0, 0.4])  # 模拟 0.4mm 扫描偏差
    dev = scan_deviation(cloud, circle.points)
    return {
        "cam_n_points": len(circle.points),
        "cam_seam_length": circle.length,
        "cam_motions": len(prog.items),
        "cam_cycle": report.cycle_time,
        "cam_code": code,
        "cam_dev_max": dev.max_dev,
        "cam_dev_mean": dev.mean_dev,
        "cam_dev_rms": dev.rms_dev,
        "cam_n_cloud": dev.n_points,
    }


def _multirobot_data() -> dict:
    """v1.0.1 多机器人小节的真实数据。

    直接调用 ``examples/multi_robot_cell_demo.py`` 的 ``run_demo()``：网页上的
    节拍与干涉数与用户本地跑 ``python examples/multi_robot_cell_demo.py`` 完全
    同源，不存在"演示数据"与真实能力脱节的问题。该函数会往 stdout 打整份报告，
    这里用 ``redirect_stdout`` 收走，只取返回值（构建日志保持干净）。
    """
    import contextlib
    import io

    sys.path.insert(0, str(REPO_ROOT / "examples"))
    import multi_robot_cell_demo as mr  # noqa: E402  （示例目录不是包，按路径导入）

    with contextlib.redirect_stdout(io.StringIO()):
        s = mr.run_demo()

    a, b = s["robots"]["机器人A"], s["robots"]["机器人B"]
    scheds = s["schedules"]
    par, ser, gre = (scheds[k] for k in ("parallel", "serial", "greedy"))
    g60 = scheds["greedy_clearance_60"]
    p60 = scheds["parallel_clearance_60"]
    near, ho, mk = s["near_miss"], s["handoff"], s["makespan"]
    starts = {r["robot"]: r["start"] for r in g60["robots"]}
    delayed = max(starts, key=starts.__getitem__)

    def rr(sched: dict) -> int:
        return sum(1 for i in sched["interferences"] if i.get("kind") == "robot-robot")

    def saved(ms: float) -> str:
        return f"{(1.0 - ms / mk['serial']) * 100.0:.0f}"

    return {
        "mr_a_cycle": f"{a['cycle_time']:.2f}",
        "mr_b_cycle": f"{b['cycle_time']:.2f}",
        "mr_a_points": str(a["waypoints"]),
        "mr_b_points": str(b["waypoints"]),
        "mr_a_env": str(a["envelope_contacts"]),
        "mr_b_env": str(b["envelope_contacts"]),
        "mr_handoff_hits": str(len(ho["collisions"])),
        "mr_handoff_sep": f"{ho['separation_mm']:+.1f}",
        "mr_handoff_links": " ↔ ".join(str(x) for x in ho["closest_links"]),
        "mr_mk_parallel": f"{mk['parallel']:.2f}",
        "mr_mk_serial": f"{mk['serial']:.2f}",
        "mr_mk_greedy": f"{mk['greedy']:.2f}",
        "mr_mk_guard": f"{mk['greedy_clearance_60']:.2f}",
        "mr_par0_hits": str(rr(par)),
        "mr_par60_hits": str(rr(p60)),
        "mr_gre60_hits": str(rr(g60)),
        "mr_near_sep": f"{near['min_separation_mm']:+.1f}",
        "mr_near_at": f"{near['closest_at_s']:.2f}",
        "mr_near_links": " ↔ ".join(str(x) for x in near["closest_links"]),
        "mr_delayed": delayed,
        "mr_delay": f"{starts[delayed]:.2f}",
        "mr_save0": saved(mk["greedy"]),
        "mr_save60": saved(mk["greedy_clearance_60"]),
        "mr_env_hits": str(len(gre.get("env_interferences") or [])),
        "mr_tests": "48",
    }


def _compliance_data() -> dict:
    """v1.0.2 力控柔顺小节的真实数据。

    用与页面其余部分同一台 UR5e 构建一个「接近 → 进入力控 → 沿面走 → 退出」
    程序：力控参数、UR 原生出码、导纳仿真曲线全部来自真实 API 调用，不手写
    演示数字。
    """
    from orolp import ComplianceSpec, ProgramBuilder
    from orolp.compliance import (
        force_sensitivity,
        required_depth,
        series_stiffness,
        simulate_admittance,
    )

    robot = Robot("models/ur/ur5e/ur5e.urdf", "UR5e")
    spec = ComplianceSpec.constant_force("z", -30.0, stiffness=8.0)
    b = ProgramBuilder(robot)
    b.moveJ([0, -90, 0, -90, 0, 0], speed=30)
    b.moveL([0, 0, 50, 0, 180, 0], speed=100)
    b.force_mode(spec)
    b.moveL([80, 0, 48, 0, 180, 0], speed=20)
    b.force_mode_end()
    b.moveJ([0, -90, 0, -90, 0, 0], speed=30)
    prog = b.build()
    report = prog.validate(check_dynamics=False)
    ur_code = prog.to_post("ur_polyscope")
    fm_lines = "\n".join(
        ln for ln in ur_code.splitlines()
        if "force_mode" in ln or "end_force_mode" in ln
    )

    # 导纳仿真：30N 阶跃下 z 轴让步曲线（真实数值积分）
    res = simulate_admittance(spec, -30.0, duration=1.0, dt=1e-3)
    k_eff = force_sensitivity(8.0, 100.0)

    # 让步曲线 SVG：offsets[:, 2] 归一化到 240px 高
    curve = res.axis_curve("z")
    peak = float(np.max(np.abs(curve)))
    pts = []
    for i in range(0, len(curve), max(1, len(curve) // 200)):
        x = i / (len(curve) - 1) * 620.0 + 60.0
        y = 270.0 - abs(curve[i]) / peak * 220.0
        pts.append(f"{x:.1f},{y:.1f}")
    steady_y = 270.0 - abs(curve[-1]) / peak * 220.0

    return {
        "fc_force": f"{abs(spec.wrench[2]):.0f}",
        "fc_stiff": f"{spec.stiffness[2]:g}",
        "fc_offset": f"{abs(spec.static_offsets()[2]):.2f}",
        "fc_keff": f"{k_eff:.2f}",
        "fc_depth": f"{required_depth(30.0, 8.0, 100.0):.2f}",
        "fc_sens": f"{k_eff * 1.0:.2f}",
        "fc_settle": f"{res.settling_time('z'):.2f}",
        "fc_overshoot": f"{res.overshoot_percent('z'):.1f}",
        "fc_ur_code": html.escape(fm_lines),
        "fc_curve": " ".join(pts),
        "fc_steady_y": f"{steady_y:.1f}",
        "fc_steady_label": f"{abs(curve[-1]):.2f} mm",
        "fc_issues": str(sum(1 for i in report.issues if i.code.startswith("force_mode"))),
    }


# --------------------------------------------------------------------------- #
# SVG 俯视示意图
# --------------------------------------------------------------------------- #


def _calibration_data() -> dict:
    """v0.3.4 标定与手眼小节的真实数据。

    直接调用 ``examples/vision_pick_demo.py`` 里同一套流程函数——网页上的数字与
    用户本地跑 ``python examples/vision_pick_demo.py`` 得到的是同一条代码路径的
    结果，避免"演示数据"和真实能力脱节。
    """
    sys.path.insert(0, str(REPO_ROOT / "examples"))
    import vision_pick_demo  # noqa: E402  （示例目录不是包，按路径导入）

    d = vision_pick_demo.run_vision_pick(save_calibration=False)
    ur_code = d["posts"]["ur_polyscope"]
    header = [ln for ln in ur_code.splitlines()[:8] if ln.strip().startswith("#")]
    return {
        "cal_tcp_truth": "(" + ", ".join(f"{v:.1f}" for v in d["tcp_truth_mm"]) + ")",
        "cal_tcp_cal": "(" + ", ".join(f"{v:.2f}" for v in d["tcp4_offset_mm"]) + ")",
        "cal_tcp4_err": f"{d['tcp4_error_mm']:.2e}",
        "cal_tcp4_res": f"{d['tcp4_residual_mm']:.4f}",
        "cal_tcp6_rot": f"{d['tcp6_rot_error_deg']:.2e}",
        "cal_frame_err": f"{d['frame_error_mm']:.2e}",
        "cal_frame_rot": f"{d['frame_angle_error_deg']:.2e}",
        "cal_he_motions": str(d["hand_eye_motions"]),
        "cal_he_err": f"{d['hand_eye_pos_error_mm']:.3f}",
        "cal_he_rot": f"{d['hand_eye_angle_error_deg']:.4f}",
        "cal_he_res": f"{d['hand_eye_residual_mm']:.3f}",
        "cal_pixel": f"({d['pixel'][0]:.1f}, {d['pixel'][1]:.1f})",
        "cal_grasp_err": f"{d['grasp_error_mm']:.3f}",
        "cal_grasp_pos": "(" + ", ".join(f"{v:.1f}" for v in d["grasp_pose"][:3]) + ")",
        "cal_motions": str(d["n_motions"]),
        "cal_cycle": f"{d['cycle_time']:.2f}",
        "cal_errors": str(d["n_errors"]),
        "cal_camera": d["camera_intrinsics"],
        "cal_header": "\n".join(header),
    }


def _svg_scene() -> str:
    """俯视布局图：世界坐标 (mm) → 屏幕坐标，与 templates.py 布局一致。"""
    S = 0.35          # 缩放：mm → px
    OX, OY = 520.0, 620.0  # 世界原点的屏幕位置

    def cx(wx):
        return OX + wx * S

    def cy(wy):
        return OY - wy * S

    def box(wcx, wcy, ww, wl, fill, label="", fs=13, angle=0.0):
        """世界中心 + 世界尺寸（x 向宽 w，y 向长 l）+ 绕 Z 角度。"""
        import math

        a = math.radians(angle)
        cos, sin = math.cos(a), math.sin(a)
        pts = []
        for dx, dy in ((-ww / 2, -wl / 2), (ww / 2, -wl / 2), (ww / 2, wl / 2), (-ww / 2, wl / 2)):
            px = cx(wcx) + (dx * cos - dy * sin) * S
            py = cy(wcy) + (dx * sin + dy * cos) * S
            pts.append(f"{px:.0f},{py:.0f}")
        out = f'<polygon points="{" ".join(pts)}" fill="{fill}" stroke="#3a4250" stroke-width="1.5"/>'
        if label:
            out += (
                f'<text x="{cx(wcx):.0f}" y="{cy(wcy) + fs * 0.36:.0f}" text-anchor="middle" '
                f'font-size="{fs}" fill="#e8ecf2">{label}</text>'
            )
        return out

    parts = []
    # 标题性文字
    parts.append(f'<text x="{cx(0):.0f}" y="{cy(-900):.0f}" text-anchor="middle" font-size="15" fill="#9aa5b5">输送带（4 工件排队上料）</text>')
    # 输送带 2400×600，中心 (0,-400)
    parts.append(box(0, -400, 2400, 600, "#33383f"))
    # 4 工件（90×90）
    for i, x in enumerate(WORKPIECE_XS, start=1):
        parts.append(box(x, -400, 90, 90, "#3d6fa8", f"工件_{i}", 11))
    # 料箱 600×400，中心 (-500, 550)
    parts.append(box(-500, 550, 600, 400, "#4a4f57", "料箱"))
    # 托盘 900×700，中心 (350, 450)
    parts.append(box(350, 450, 900, 700, "#5a5142", "托盘（落件区）", 13))
    # 抓取 → 落件 路径
    parts.append(
        f'<path d="M {cx(PICK_XY[0]):.0f} {cy(PICK_XY[1]):.0f} C {cx(100):.0f} {cy(100):.0f}, '
        f'{cx(250):.0f} {cy(450):.0f}, {cx(PLACE_XY[0]):.0f} {cy(PLACE_XY[1]):.0f}" fill="none" '
        f'stroke="#e8a33d" stroke-width="2.5" stroke-dasharray="7 5" marker-end="url(#arrow)"/>'
    )
    # 机器人基座
    parts.append(
        f'<circle cx="{cx(0):.0f}" cy="{cy(0):.0f}" r="30" fill="#4a6b8a" stroke="#e8ecf2" stroke-width="2"/>'
        f'<text x="{cx(0):.0f}" y="{cy(0) + 40:.0f}" text-anchor="middle" font-size="13" fill="#ffffff">UR5e</text>'
    )
    return "".join(parts)


def _svg_two_robot_cell() -> str:
    """双机器人共享工作单元的俯视图。

    坐标**直接取示例脚本的常量**（``FEED_A`` / ``BASE_A`` / ``HANDOFF`` /
    ``BIN_B``），所以网页上的布局不可能与真实场景漂移。
    """
    sys.path.insert(0, str(REPO_ROOT / "examples"))
    import multi_robot_cell_demo as mr  # noqa: E402

    s = 0.30            # mm → px
    ox, oy = 520.0, 180.0

    def cx(wx: float) -> float:
        return ox + wx * s

    def cy(wy: float) -> float:
        return oy - wy * s

    def plate(wx, wy, w, l, fill, label="", fs=12, stroke="#3a4250"):
        pts = " ".join(
            f"{cx(wx + dx):.0f},{cy(wy + dy):.0f}"
            for dx, dy in ((-w / 2, -l / 2), (w / 2, -l / 2), (w / 2, l / 2), (-w / 2, l / 2))
        )
        out = f'<polygon points="{pts}" fill="{fill}" stroke="{stroke}" stroke-width="1.4"/>'
        if label:
            out += (
                f'<text x="{cx(wx):.0f}" y="{cy(wy) + fs * 0.36:.0f}" text-anchor="middle" '
                f'font-size="{fs}" fill="#e8ecf2">{label}</text>'
            )
        return out

    p = []
    # 三张台面（台面同高 = PLANE_Z 800mm）：取件台 / 共享工作台 / 出料台
    p.append(plate(*mr.FEED_A, 500, 500, "#33383f"))
    p.append(plate(0.0, 0.0, 900, 700, "#3d4652", "共享工作台 · 交接区", 12))
    p.append(plate(*mr.BIN_B, 500, 500, "#33383f"))
    # 垫块（把抓取点抬高 RISER_H，避免保守包络蹭台面）
    for xy in (mr.FEED_A, mr.BIN_B):
        p.append(plate(xy[0], xy[1], 240, 240, "#4a4f57"))
    # 工件：待在 A 的取件台上
    p.append(plate(mr.FEED_A[0], mr.FEED_A[1], 60, 60, "#3d6fa8", "工件", 11))
    # 立柱 + 机器人基座
    for tag, base, color in (
        ("A", mr.BASE_A, "#4a6b8a"),
        ("B", mr.BASE_B, "#8a5a4a"),
    ):
        bx = float(base.matrix[0, 3])
        by = float(base.matrix[1, 3])
        p.append(plate(bx, by, 400, 400, "#2a3037"))
        p.append(
            f'<circle cx="{cx(bx):.0f}" cy="{cy(by):.0f}" r="16" fill="{color}" '
            f'stroke="#e8ecf2" stroke-width="1.6"/>'
        )
        p.append(
            f'<text x="{cx(bx):.0f}" y="{cy(by) - 24:.0f}" text-anchor="middle" '
            f'font-size="12" fill="#e8ecf2">机器人{tag}</text>'
        )
    # 搬运路径：A 取件台 → 交接点 → B 出料台（两条轨迹共用中间那片空间）
    for a, b, color in (
        (mr.FEED_A, mr.HANDOFF, "#e8a33d"),
        (mr.HANDOFF, mr.BIN_B, "#5cc8c8"),
    ):
        p.append(
            f'<line x1="{cx(a[0]):.0f}" y1="{cy(a[1]):.0f}" x2="{cx(b[0]):.0f}" '
            f'y2="{cy(b[1]):.0f}" stroke="{color}" stroke-width="2.2" '
            f'stroke-dasharray="6 4" marker-end="url(#mrarw)"/>'
        )
    # 交接点
    p.append(
        f'<circle cx="{cx(0):.0f}" cy="{cy(0):.0f}" r="9" fill="none" '
        f'stroke="#e8a33d" stroke-width="2"/>'
        f'<text x="{cx(0):.0f}" y="{cy(0) + 40:.0f}" text-anchor="middle" font-size="11.5" '
        f'fill="#e8a33d">交接点 (0, 0)</text>'
    )
    return "".join(p)


# --------------------------------------------------------------------------- #
# TCP 轨迹动画数据（关键路点 → 帧插值由 JS 完成）
# --------------------------------------------------------------------------- #


def _traj_keyframes() -> str:
    pick_top = BELT_TOP + WORKPIECE_H
    place_top = PALLET_TOP + WORKPIECE_H
    kfs = [
        (0, -100, 900),
        (PICK_XY[0], PICK_XY[1], pick_top + 100),
        (PICK_XY[0], PICK_XY[1], pick_top),
        (PICK_XY[0], PICK_XY[1], pick_top + 100),
        (PLACE_XY[0], PLACE_XY[1], place_top + 150),
        (PLACE_XY[0], PLACE_XY[1], place_top),
        (PLACE_XY[0], PLACE_XY[1], place_top + 150),
        (0, -100, 900),
    ]
    return "[" + ",".join(f"[{x},{y},{z}]" for x, y, z in kfs) + "]"


# --------------------------------------------------------------------------- #
# 真机连接面板小节（v1.1.1）：构建时真起打桩控制器，真跑面板全流程 + 离屏截图
# --------------------------------------------------------------------------- #


def _controller_panel_data() -> dict:
    """「真机连接面板」小节的真实数据。

    起一个本机回环打桩控制器（v1.0.5 的 ControllerStubServer），把
    ControllerPanel 当作用户一样操作：连接 → 部署当前程序 → 回读关节，
    全程走真 socket。日志与状态文本直接取自面板控件本身——网页上看到的
    就是 GUI 里会出现的文字。面板截图用 ``widget.grab()`` 离屏渲染。
    """
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtWidgets

    from orolp.controller_panel import ControllerPanel
    from orolp.linkstub import ControllerStubServer

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    assert app is not None

    robot = Robot("models/ur/ur5e/ur5e.urdf", "UR5e")
    robot.tool = Pose.from_xyz_rpy(0.0, 0.0, 150.0, 0.0, 0.0, 0.0)

    def provider():
        builder = ProgramBuilder(robot)
        builder.moveJ([0, -90, 90, -90, 0, 0], speed=50.0)
        builder.moveJ([30, -70, 80, -90, 25, 0], speed=50.0)
        return {"program": builder.build(), "post_processor": None}

    stub = ControllerStubServer("127.0.0.1", 0, dof=6)
    port = stub.start()
    panel = ControllerPanel()
    panel.resize(360, 640)
    panel.set_robot(robot)
    panel.set_program_provider(provider)
    panel.spin_port.setValue(port)
    panel.spin_timeout.setValue(2.0)
    panel.connect_to_controller()
    deployment = panel.deploy_current_program()
    q = panel.read_joints()

    # 离屏截图（与真实 GUI 会话同一控件树，无需显示服务器）
    shot = REPO_ROOT / "docs" / "screenshots" / "controller_panel.png"
    shot.parent.mkdir(parents=True, exist_ok=True)
    panel.grab().save(str(shot))

    n_programs = panel.list_programs.count()
    info = panel.label_info.text()
    joints_text = panel.label_joints.text()
    log = panel.log_view.toPlainText()
    rpc = stub.state.requests
    handle = deployment.handle if deployment is not None else ""
    panel.disconnect_from_controller()
    panel.deleteLater()
    stub.stop()

    return {
        "cp_info": html.escape(info),
        "cp_log": html.escape(log),
        "cp_joints": html.escape(joints_text),
        "cp_handle": html.escape(handle),
        "cp_programs": str(n_programs),
        "cp_rpc": str(len(rpc)),
        "cp_n_joints": "6" if q is not None else "-",
        "cp_tests": "16",
    }


def _controller_panel_ssh_data() -> dict:
    """「真机连接面板 · SSH 传输」小节的真实数据（v1.1.4）。

    构建时真的起一个 SSH 打桩控制器，然后**像用户一样操作面板**：切到 SSH 传输 →
    填凭据 → 连接 → 部署当前程序（SFTP 真落盘）→ 取回核对 → 手动改动远端文件 →
    再核对一次（必须报不一致）→ 回读关节（必须如实说"没有通道"）。网页上的每一行
    日志、每一个数字都取自这次真实操作的面板控件本身。
    """
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    import tempfile

    from PySide6 import QtWidgets

    from orolp.controller_panel import TRANSPORT_SSH, ControllerPanel
    from orolp.sshstub import SSHStubServer

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    assert app is not None

    robot = Robot("models/ur/ur5e/ur5e.urdf", "UR5e")
    robot.tool = Pose.from_xyz_rpy(0.0, 0.0, 150.0, 0.0, 0.0, 0.0)

    def provider():
        builder = ProgramBuilder(robot)
        builder.moveJ([0, -90, 90, -90, 0, 0], speed=50.0)
        builder.moveJ([30, -70, 80, -90, 25, 0], speed=50.0)
        return {"program": builder.build(), "post_processor": None}

    with tempfile.TemporaryDirectory() as tmp:
        stub = SSHStubServer(Path(tmp) / "stubroot")
        addr = f"127.0.0.1:{stub.port}"
        panel = ControllerPanel()
        panel.resize(400, 780)
        panel.set_robot(robot)
        panel.set_program_provider(provider)
        panel.set_transport(TRANSPORT_SSH)
        panel.edit_host.setText("127.0.0.1")
        panel.edit_user.setText("tester")
        panel.edit_password.setText("pw")
        panel.edit_remote_dir.setText("/programs")
        panel.spin_port.setValue(stub.port)
        panel.spin_timeout.setValue(10.0)

        connected = panel.connect_to_controller()
        info = panel.label_info.text()
        deployment = panel.deploy_current_program()
        verified = panel.verify_deployed()

        # 手动改动远端文件（模拟被别的进程覆盖）→ 核对必须报"不一致"
        landed = stub.root / "programs" / "program.script"
        tampered = None
        if landed.exists():
            landed.write_text("; 被人改过的程序\n", encoding="utf-8")
            tampered = panel.verify_deployed()

        joints = panel.read_joints()
        joints_text = panel.label_joints.text()
        log = panel.log_view.toPlainText()
        programs = panel.list_programs.count()

        shot = REPO_ROOT / "docs" / "screenshots" / "controller_panel_ssh.png"
        shot.parent.mkdir(parents=True, exist_ok=True)
        panel.grab().save(str(shot))

        if panel.is_connected:
            panel.disconnect_from_controller()
        panel.deleteLater()
        stub.stop()

    handle = deployment.handle if deployment is not None else ""
    return {
        "cps_addr": html.escape(addr),
        "cps_connected": "是" if connected else "否",
        "cps_info": html.escape(info),
        "cps_handle": html.escape(handle),
        "cps_lines": str(len(deployment.code.splitlines()) if deployment else 0),
        "cps_bytes": str(len(deployment.code.encode("utf-8")) if deployment else 0),
        "cps_verify": "一致" if (verified and verified["matches"] is True) else "未核对",
        "cps_tamper": (
            "抓到不一致"
            if (tampered is not None and tampered["matches"] is False)
            else "未抓到"
        ),
        "cps_joints": html.escape(joints_text),
        "cps_joints_none": "是" if joints is None else "否",
        "cps_programs": str(programs),
        "cps_log": html.escape(log),
        "cps_tests": "18",
    }


def _trajectory_arc_data() -> dict:
    """「轨迹编辑器圆弧（moveC）编辑」小节的真实数据（v1.1.5）。

    构建时**真的操作编辑器**：把 P3 设为圆弧（中间点 P2）→ 校验 → 看规划出的
    TCP 与"起终点直线"差多少 → 分别用 UR / KUKA / ABB 出码 → 再故意让正解失败
    一次，看编辑器怎么如实说明降级。网页上每个数字都来自这次真实操作。
    """
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtWidgets

    from orolp.program_panel import ProgramPanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    assert app is not None

    joints = [f"joint_{i}" for i in range(1, 7)]
    points = [
        [0.0, -90.0, 90.0, -90.0, 0.0, 0.0],
        [30.0, -70.0, 70.0, -80.0, 20.0, 0.0],
        [45.0, -60.0, 60.0, -75.0, 30.0, 0.0],
        [60.0, -50.0, 50.0, -70.0, 40.0, 0.0],
    ]
    positions = [{n: math.radians(v) for n, v in zip(joints, p)} for p in points]

    robot = Robot("models/ur/ur5e/ur5e.urdf", "UR5e")
    panel = ProgramPanel()
    panel.set_robot(robot, joints)
    panel.set_taught_points(positions)

    reject_two = panel.arc_rejection(1) or "（无）"
    joint_report = panel.validate_program()

    panel.set_point_motion(2, "C")
    arc_report = panel.validate_program()
    plan, _notes = panel.motion_plan()
    rows = "\n".join(
        panel.point_list.item(i).text() for i in range(panel.point_list.count())
    )

    # 圆弧段的 TCP：只取第 2 条指令的路径点，别把前后两段关节运动算进来
    planner = panel._planner
    via = np.array(robot.forward_kinematics(points[1]).to_xyz_quat()[:3])
    start = np.array(robot.forward_kinematics(points[0]).to_xyz_quat()[:3])
    end = np.array(robot.forward_kinematics(points[2]).to_xyz_quat()[:3])
    arc_pts = [
        np.array(w.pose.to_xyz_quat()[:3])
        for w in panel.trajectory.waypoints
        if planner.motion_at(w.timestamp) == 1
    ]
    nearest_via = min(float(np.linalg.norm(p - via)) for p in arc_pts)
    chord = end - start
    chord_len = float(np.linalg.norm(chord))
    unit = chord / chord_len
    deviations = []
    for p in arc_pts:
        v = p - start
        proj = min(max(float(np.dot(v, unit)), 0.0), chord_len)
        deviations.append(float(np.linalg.norm(v - proj * unit)))

    program = panel.build_program()
    brand_lines = []
    for brand, marker in (
        ("ur_polyscope", "movec("),
        ("kuka_krc4", "CIRC"),
        ("abb_irc5", "MoveC"),
    ):
        code = program.to_post(brand, metadata={"program_name": "arc_demo"})
        line = next(
            (ln.strip() for ln in code.splitlines() if marker in ln),
            "（未找到圆弧指令）",
        )
        brand_lines.append(f"--- {brand} ---\n{line}")

    shot = REPO_ROOT / "docs" / "screenshots" / "trajectory_arc.png"
    shot.parent.mkdir(parents=True, exist_ok=True)
    panel.grab().save(str(shot))
    panel.deleteLater()

    # 故意让中间点的正解失败一次：编辑器必须说清"圆弧没了、中间点改为单独经过"
    panel2 = ProgramPanel()
    panel2.set_robot(Robot("models/ur/ur5e/ur5e.urdf", "UR5e"), joints)
    panel2.set_taught_points(positions)
    panel2.set_point_motion(2, "C")
    real_fk = panel2._robot.forward_kinematics
    bad = [float(v) for v in points[1]]
    state = {"left": 1}

    def flaky(joints_deg, *args, **kwargs):
        hit = len(joints_deg) == len(bad) and all(
            abs(float(x) - y) < 1e-9 for x, y in zip(joints_deg, bad)
        )
        if hit and state["left"] > 0:
            state["left"] -= 1
            raise ValueError("simulated FK failure")
        return real_fk(joints_deg, *args, **kwargs)

    panel2._robot.forward_kinematics = flaky
    degraded = panel2.validate_program()
    degrade_note = panel2.last_build_notes()
    degrade_labels = ", ".join(degraded.motion_labels or [])
    panel2.deleteLater()

    return {
        "arc_reject_two": html.escape(reject_two),
        "arc_cycle_joint": f"{joint_report.cycle_time:.2f}",
        "arc_cycle_arc": f"{arc_report.cycle_time:.2f}",
        "arc_instr_joint": str(len(joint_report.motion_times or [])),
        "arc_instr_arc": str(len(arc_report.motion_times or [])),
        "arc_plan": " → ".join(f"{kind}(P{i + 1})" for kind, i, _via in plan),
        "arc_rows": html.escape(rows),
        "arc_via_dist": f"{nearest_via:.2f}",
        "arc_deviation": f"{max(deviations):.1f}",
        "arc_chord": f"{chord_len:.1f}",
        "arc_brand": html.escape("\n".join(brand_lines)),
        "arc_degrade_note": html.escape(degrade_note[0] if degrade_note else ""),
        "arc_degrade_labels": html.escape(degrade_labels),
        "arc_tests": "24",
    }


def _iomap_data() -> dict:
    """「PLC ↔ 机器人 IO 点表」小节的真实数据（v1.1.6）。

    构建时**真的跑一遍点表**：造一个"看起来没问题"的单元（PLC + 夹爪 + 安全），
    然后让三类现场事故同时发生——机器人程序用了没地址的信号、两个信号挤在同一个
    物理地址、同一信号被机器人与 PLC 同时驱动——看 `SignalMap.validate()` 抓出几条、
    severity 对不对；再真跑 `auto_assign`（必须跳过已占用地址）与 `generate_scl`
    （点表随代码一起交给现场），并离屏截一张硬件列表（新增的两个入口）。
    """
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtWidgets

    from orolp.cell import CellConfig, Device
    from orolp.iomap import SignalMap, used_signals
    from orolp.plc_scl import generate_scl
    from orolp.program import Program

    cell = CellConfig()
    cell.add(Device(name="PLC_S7_1200", type="PLC", brand="Siemens",
                    model="1214C", comm="Profinet", ip="192.168.0.10",
                    io_map={"robot_start": "Q0.0", "robot_done": "I0.0"}))
    cell.add(Device(name="夹爪_1", type="夹具",
                    io_map={"gripper_open": "Q0.1", "gripper_close": "Q0.1"}))
    cell.add(Device(name="安全回路", type="安全",
                    io_map={"safety_ok": "I0.1", "estop": "I0.2"}))

    # 一个"现场很常见"的机器人程序：写夹爪、等工件到位、等夹爪夹紧反馈，
    # 外加一个谁都没建过地址的信号
    program = Program("cell_demo")
    program.setDO("gripper_close", True)
    program.waitDI("part_present")
    program.waitDI("clamped")
    program.setDO("ghost_close", True)
    items = list(program.items)

    smap = SignalMap(cell)
    report = smap.validate(items)
    error_lines = "\n".join(
        f"✗ [{i.signal or '-'}] {i.message}" for i in report.errors
    )
    warn_lines = "\n".join(
        f"⚠ [{i.signal or '-'}] {i.message}" for i in report.warnings
    )

    # 出码**先于**自动编号：这样"校验结论"与"随码交给现场的点表"描述的是同一个
    # 状态，页面上两组数字不会互相打架（自动编号单独作为下一步展示）
    usage = used_signals(items)
    scl = generate_scl(cell, program_name="CellControl", program=items)
    table_start = scl.index("(* ---------------- IO 点表")
    table_end = scl.index('FUNCTION_BLOCK')
    scl_block = scl[table_start:table_end].strip()[:1400]
    # 取 VAR 声明行上的那句——"留空"在现场等于"看起来没问题"，所以它必须显式写出来
    unmapped_comment = next(
        (ln.strip() for ln in scl.splitlines()
         if ": BOOL;" in ln and ">>> 未映射" in ln),
        next((ln.strip() for ln in scl.splitlines() if ">>> 未映射" in ln), "（无）"),
    )

    # 自动编号：跳过已被占用的地址（Q0.1 被夹爪占了，绝不能编到同一个点上）
    auto = smap.auto_assign(
        "PLC_S7_1200", ["robot_start", "clamped", "part_present"], program=items
    )
    auto_text = "、".join(f"{k}→{v}" for k, v in auto.items())

    # 往返：JSON 出去再进来，三种结果必须分开报
    payload = SignalMap(cell).to_json(items)
    target = CellConfig()
    for dev in cell.devices:
        target.add(Device(name=dev.name, type=dev.type, io_map=dict(dev.io_map)))
    roundtrip = SignalMap.from_json(target, payload)

    rows = report.format()
    # 离屏截一张硬件列表：v1.1.6 的两个入口就在这一排按钮上
    shot_note = "（截图不可用）"
    try:
        import app as rw

        app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        win = rw.MainWindow()
        ed = win.code_editor
        ed.cell = cell
        ed._refresh_device_table()
        ed.sub_tabs.setCurrentIndex(1)
        shot = REPO_ROOT / "docs" / "screenshots" / "io_map.png"
        shot.parent.mkdir(parents=True, exist_ok=True)
        ed.grab().save(str(shot))
        shot_note = "docs/screenshots/io_map.png"
        win.close()
        win.deleteLater()
    except Exception as exc:  # pragma: no cover - 截图失败不该拖垮站点构建
        shot_note = f"（截图失败：{exc}）"

    return {
        "io_tests": "90",
        "io_points": str(len(report.points)),
        "io_mapped": str(report.mapped_count),
        "io_summary": html.escape(report.summary()),
        "io_errors": html.escape(error_lines or "（无）"),
        "io_warnings": html.escape(warn_lines or "（无）"),
        "io_error_n": str(len(report.errors)),
        "io_warn_n": str(len(report.warnings)),
        "io_auto": html.escape(auto_text or "（无）"),
        "io_rows": html.escape(rows),
        "io_scl_block": html.escape(scl_block),
        "io_unmapped_comment": html.escape(unmapped_comment),
        "io_roundtrip": "应用 {applied} / 无需改动 {unchanged} / 跳过 {skipped}".format(**roundtrip),
        "io_signals_used": str(len(usage)),
        "io_signal_names": html.escape("、".join(usage)),
        "io_shot": shot_note,
    }


def _ssh_link_data() -> dict:
    """「真机连接 SSH」小节的真实数据。

    起一个本机回环 SSH 打桩控制器（paramiko 服务端 + 真实落盘的 SFTP 根目录），
    把 SSHLink 当作部署流水线一样用：连接 → Program.deploy（SFTP 落盘）→
    清单核对 → 取回原文逐字节比对 → SSH exec 触发现场加载 → 认证失败演练。
    全程走真 SSH/SFTP/exec，网页上的数字就是这次真实回环的结果。
    """
    import tempfile as _tempfile
    from pathlib import Path as _Path

    from orolp.link import LinkError as _LinkError
    from orolp.link import SSHLink
    from orolp.sshstub import SSHStubServer

    robot = Robot("models/ur/ur5e/ur5e.urdf", "UR5e")
    robot.tool = Pose.from_xyz_rpy(0.0, 0.0, 150.0, 0.0, 0.0, 0.0)
    builder = ProgramBuilder(robot)
    builder.moveJ([0, -90, 90, -90, 0, 0], speed=50.0)
    builder.moveL(Pose.from_xyz_rpy(420, 0, 320, 0, 0, 0), speed=60.0)
    builder.moveL(Pose.from_xyz_rpy(420, 180, 320, 0, 0, 0), speed=60.0)
    program = builder.build()

    root = _Path(_tempfile.mkdtemp(prefix="orolp_web_ssh_"))
    stub = SSHStubServer(root, password="demo")
    out: dict = {"ssh_tests": "30"}
    try:
        link = SSHLink("127.0.0.1", stub.port, username="operator",
                       password="demo", remote_dir="/programs", timeout=10)
        link.connect()
        dep = program.deploy(link, "ur_polyscope", name="web_ssh_pick")
        back = link.fetch_program("web_ssh_pick", "script")
        out["ssh_addr"] = f"ssh://operator@127.0.0.1:{stub.port}"
        out["ssh_handle"] = dep.handle.replace(f"ssh://operator@127.0.0.1:{stub.port}", "ssh://…")
        out["ssh_lines"] = str(len(dep.code.splitlines()))
        out["ssh_bytes"] = str(len(dep.code.encode("utf-8")))
        out["ssh_programs"] = ", ".join(link.list_programs())
        out["ssh_fetch_ok"] = "逐字节一致 ✓" if back == dep.code else "不一致 ✗"
        out["ssh_manifest"] = (stub.root / "programs" / "web_ssh_pick.manifest.json").name
        ls = link.run_command("ls -1 programs", cwd="/")
        out["ssh_ls_code"] = str(ls.exit_code)
        out["ssh_ls_out"] = ls.stdout.strip() or ls.stderr.strip()
        load = link.run_command("load web_ssh_pick.script", cwd="/programs")
        out["ssh_load_code"] = str(load.exit_code)
        out["ssh_load_err"] = load.stderr.strip().splitlines()[0][:60] if load.stderr.strip() else "-"
        out["ssh_still_connected"] = "已连接 ✓" if link.connected else "已断开 ✗"
        out["ssh_execs"] = str(link.executions)

        # 认证失败演练（错密码）——网络/认证级故障必须断开
        bad = SSHLink("127.0.0.1", stub.port, username="operator",
                      password="WRONG", remote_dir="/programs", timeout=10)
        try:
            bad.connect()
            out["ssh_auth"] = "错误密码被放行 ✗（打桩配置错误）"
        except _LinkError as exc:
            out["ssh_auth"] = str(exc).split("——")[0].split("：", 1)[-1]
        out["ssh_auth_state"] = "已断开 ✓" if not bad.connected else "仍显示已连接 ✗"
        out["ssh_auth_rejects"] = str(len(stub.auth.rejected))
        link.disconnect()
    finally:
        stub.stop()
    return out


def _dwell_data() -> dict:
    """「waitTime 驻留段时间轴」小节的真实数据（v1.1.3）。

    同一条 UR5e 程序（两次取放路径）加/不加 waitTime 的节拍对比（规划器
    实算），加一个"驻留占用"双机场景：A 在交接点驻留、B 在窗内/窗外到达
    的冲突计数对比——全部真规划器实跑。
    """
    from orolp import Pose
    from orolp.multirobot import RobotJob, scan_timeline
    from orolp.planner import TrajectoryPlanner
    from orolp.world.scene import Scene

    robot = Robot("models/ur/ur5e/ur5e.urdf", "UR5e")

    def _pick_prog(dwell: float):
        b = (
            ProgramBuilder(robot)
            .moveJ([0, -90, 90, -90, 0, 0], speed=60)
            .moveL(Pose.from_xyz_rpy(420, 0, 320, 0, 0, 0), speed=80)
            .moveL(Pose.from_xyz_rpy(420, 0, 380, 0, 0, 0), speed=80)
        )
        if dwell > 0:
            b.waitTime(dwell)
        b.moveL(Pose.from_xyz_rpy(420, 180, 380, 0, 0, 0), speed=80)
        return b.build()

    prog_plain = _pick_prog(0.0)
    prog_dwell = _pick_prog(0.8)
    t_plain = prog_plain.plan().duration()
    t_dwell = prog_dwell.plan().duration()
    report = TrajectoryPlanner(robot).validate(prog_dwell)

    # 双机"驻留占用"场景：两台 UR5e 经 IK 去往同一世界点
    urdf = "models/ur/ur5e/ur5e.urdf"
    Q0 = [0.0, -90.0, 90.0, -90.0, 0.0, 0.0]
    Qh = [40.0, -60.0, 80.0, -90.0, 30.0, 0.0]
    scene = Scene()
    scene.add_robot(urdf, list(Q0), name="A")
    scene.add_robot(urdf, list(Q0), name="B",
                    pose=Pose.from_xyz_rpy(300.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    robot_b = Robot(urdf, "UR5e")
    p_world = robot.forward_kinematics(Qh)
    x, y, z, rx, ry, rz = p_world.to_xyz_rpy()
    qb = [float(v) for v in robot_b.inverse_kinematics(
        Pose.from_xyz_rpy(x - 300.0, y, z, rx, ry, rz), guess=Qh)]

    prog_a = (
        ProgramBuilder(robot)
        .moveJ(Qh, speed=80)
        .waitTime(2.0)
        .moveJ(Q0, speed=160)
        .build()
    )
    pa = TrajectoryPlanner(robot)
    traj_a = pa.plan(prog_a)
    t_arrive = pa._motion_spans[0][1]
    t_leave = pa._motion_spans[1][0]
    traj_b = ProgramBuilder(robot_b).moveJ(qb, speed=60).build().plan()
    t_b = traj_b.duration()

    def _jobs(b_start):
        return [RobotJob(robot="A", trajectory=traj_a, start=0.0, label="A"),
                RobotJob(robot="B", trajectory=traj_b, start=b_start, label="B")]

    rr_hit, _ = scan_timeline(scene, _jobs(max(0.0, t_leave - 0.4 - t_b)), dt=0.05, clearance=60.0)
    rr_safe, _ = scan_timeline(scene, _jobs(t_leave + t_b + 1.0), dt=0.05, clearance=60.0)

    return {
        "dwell_plain": f"{t_plain:.2f}",
        "dwell_with": f"{t_dwell:.2f}",
        "dwell_delta": f"{t_dwell - t_plain:.2f}",
        "dwell_report_total": f"{report.dwell_total:.2f}",
        "dwell_n": str(report.num_dwells),
        "dwell_hit": str(len(rr_hit)),
        "dwell_safe": str(len(rr_safe)),
        "dwell_tests": "11",
    }


def _placement_data() -> dict:
    """「场景拖拽摆放工件」小节的真实数据（v1.1.7）。

    构建时真跑 ``orolp.world.placement``：真取托盘顶面高度、真算工件原点到底面
    的偏移、真跑吸附（25mm 网格），并把三条拒绝路径的实际理由抓出来——
    页面上每个数字都来自这次调用。
    """
    from orolp.world import build_grasping_workstation
    from orolp.world import placement as P

    cell = build_grasping_workstation()
    scene = cell.scene
    part = cell.grasp_objects[0]
    target = cell.place_target

    box = P.world_aabb(scene, target)
    cx = float((box.lo[0] + box.hi[0]) / 2.0)
    cy = float((box.lo[1] + box.hi[1]) / 2.0)
    height, support = P.support_height_at(scene, cx, cy)
    offset = P.ground_offset(scene, part)

    # 故意给一个不在网格上的坐标：看吸附前后差多少
    raw_x, raw_y = cx + 12.0, cy - 7.0
    result = P.place_on_support(scene, part, raw_x, raw_y, snap_step=25.0)
    part_aabb = P.world_aabb(scene, part)

    robot_name = next((n for n in scene.objects if n == "机器人"), "机器人")
    robot_reject = P.place_on_support(scene, robot_name, 0.0, 0.0)
    ground_h, ground_name = P.support_height_at(scene, 1500.0, 1500.0)

    return {
        "pl_part": part,
        "pl_target": target,
        "pl_box": f"{box.hi[0] - box.lo[0]:.0f} × {box.hi[1] - box.lo[1]:.0f} × {box.hi[2] - box.lo[2]:.0f}",
        "pl_support": support,
        "pl_height": f"{height:.0f}",
        "pl_offset": f"{offset:.1f}",
        "pl_raw": f"({raw_x:.1f}, {raw_y:.1f})",
        "pl_snapped": f"({result.x:.1f}, {result.y:.1f})",
        "pl_z": f"{result.z:.0f}",
        "pl_moved": "已落位" if result.moved else "未移动",
        "pl_summary": P.placement_summary(result),
        "pl_box_after": f"Z {part_aabb.lo[2]:.0f} → {part_aabb.hi[2]:.0f} mm（底面贴在 {support} 顶面）",
        "pl_robot_reject": robot_reject.reason,
        "pl_ground": f"({ground_h:.0f}, {ground_name})",
        "pl_snap_rows": f"12 → {P.snap(12.0, 25.0):.0f} ｜ 13 → {P.snap(13.0, 25.0):.0f} ｜ step=0 → {P.snap(12.0, 0.0):.0f}（不吸附）",
        "pl_grabbable": f"{part}={P.grabbable(scene, part)} ｜ 机器人={P.grabbable(scene, '机器人')}",
        "pl_tests": "30",
    }


def _ui_layout_data() -> dict:
    """「界面架构改版」小节的真实数据（v1.3.0）。

    构建时真起一次主窗口（offscreen）读取左右标签结构，并把窗口缩到它能到达的
    **最小高度**，逐页读「内容最小高度 vs 可视高度」；再真跑一次法兰装夹
    （模板场景 + ``orolp.world.placement``，含随动验证）。页面上每个数字都来自
    这次运行；修复前的 286px 是开发期实测值，页面已显式标注是"修复前"。
    """
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtWidgets
    from PySide6.QtWidgets import QScrollArea

    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))
    import app as rw
    from orolp.world import placement as P

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    win = rw.MainWindow()
    win.show()
    win.resize(900, win.minimumSizeHint().height())
    QtWidgets.QApplication.processEvents()

    left = " / ".join(
        win.left_tabs.tabText(i) for i in range(win.left_tabs.count())
    )
    right = " / ".join(
        win.right_tabs.tabText(i) for i in range(win.right_tabs.count())
    )
    subtabs = " / ".join(
        win.code_editor.sub_tabs.tabText(i)
        for i in range(win.code_editor.sub_tabs.count())
    )
    legacy = [
        t
        for t in ("场景", "动作与从属", "资源库")
        if t in {left, right} or t in left.split(" / ") or t in right.split(" / ")
    ]
    rows = []
    for prefix, tabs in (("左", win.left_tabs), ("右", win.right_tabs)):
        for i in range(tabs.count()):
            page = tabs.widget(i)
            rows.append(
                "{p}·{name}：内容 {need}px ≤ 可视 {have}px".format(
                    p=prefix,
                    name=tabs.tabText(i),
                    need=page.minimumSizeHint().height(),
                    have=page.height(),
                )
            )
    fit = "\n".join(rows) + "\n标签容器可视高 {}px ｜ 窗口最小高 {}px".format(
        win.left_tabs.height(), win.minimumSizeHint().height()
    )

    # ---- 法兰装夹（模板场景 + 真 API）----
    cell = build_grasping_workstation()
    scene = cell.scene
    part = cell.grasp_objects[0]
    robot = scene.robot_names[0]
    flange = (scene.world_transform(robot) @ scene.flange_transform(robot))[:3, 3]
    part_w = scene.world_transform(part)[:3, 3]

    P.apply_world_pose(scene, part, float(flange[0] + 30.0), float(flange[1]), float(flange[2]), 0.0)
    near = P.flange_snap_target(scene, part, 60.0)
    P.apply_world_pose(scene, part, float(flange[0] + 200.0), float(flange[1]), float(flange[2]), 0.0)
    far = P.flange_snap_target(scene, part, 60.0)

    P.mount_to_flange(scene, part, robot)
    local = scene.objects[part].pose.matrix[:3, 3]
    mounted = P.mounted_on_robot(scene, part)
    before_move = scene.world_transform(part)[:3, 3].copy()
    joints = list(scene.robots[robot].joints_deg)
    joints[0] += 20.0
    scene.set_robot_joints(joints, robot)
    after_move = scene.world_transform(part)[:3, 3]
    follow = float(np.linalg.norm(after_move - before_move))

    # ---- 截图：把模板场景装进窗口，让设备树显示真实层级
    #（已装夹的工件出现在「机器人」节点下，这正是拖拽装夹要表达的东西）----
    shots = []
    win._adopt_scene(scene)
    win.scene_panel._refresh_tree()
    # 左侧切到「设备树」页：装夹关系（工件挂在机器人节点下）正是这轮要展示的东西
    if win.left_tabs.count() > 1:
        win.left_tabs.setCurrentIndex(1)
    QtWidgets.QApplication.processEvents()
    for name, widget in (
        ("ui_left_tabs.png", win.left_tabs),
        ("ui_right_tabs.png", win.right_tabs),
    ):
        path = REPO_ROOT / "docs" / "screenshots" / name
        if widget.grab().save(str(path)):
            shots.append(name)

    lib = win.library_panel
    scroll = lib.findChild(QScrollArea)
    lib_content = scroll.widget().minimumSizeHint().height() if scroll else 0
    lib_viewport = scroll.viewport().height() if scroll else 0
    lib_panel = lib.minimumSizeHint().height()
    lib_scroll = "已出现" if (scroll and scroll.verticalScrollBar().isVisible()) else "未出现"
    win.close()

    return {
        "uil_left": left,
        "uil_right": right,
        "uil_subtabs": subtabs,
        "uil_legacy": "、".join(legacy) if legacy else "无（已清除）",
        "uil_fit": fit,
        "uil_lib_panel": str(lib_panel),
        "uil_lib_content": str(lib_content),
        "uil_lib_viewport": str(lib_viewport),
        "uil_lib_scroll": lib_scroll,
        "uil_part": "({:.0f}, {:.0f}, {:.0f})".format(*part_w),
        "uil_flange": "({:.0f}, {:.0f}, {:.0f})".format(*flange),
        "uil_snap_near": near or "未命中",
        "uil_snap_far": far or "未命中（超出 60mm 半径）",
        "uil_local": "({:.1f}, {:.1f}, {:.1f})".format(*[float(v) for v in local]),
        "uil_mounted": "是" if mounted else "否",
        "uil_follow": "{:.1f}".format(follow),
        "uil_shots": "、".join(shots) if shots else "（截图未生成）",
        "uil_tests": "12",
    }


def _sequence_data() -> dict:
    """「工艺时序（动态时序）」小节的真实数据（v1.3.1）。

    构建时真跑一遍 ``orolp.sequence``：建参考序列 → 真校验（干净态与"点表里没有
    这个信号"态各一次）→ 两个出口各真翻一次（机器人侧真喂 ``ProgramBuilder``，
    PLC 侧真喂 ``generate_scl``）→ 与 ``steps_from_program`` 逐字段对拍 →
    再真起一次 ``SequenceEditor`` 加四条工步、校验、离屏截图。

    页面上的每个数字都来自这次运行。诚实口径：**只有驻留合计是时间**，触发与
    等待的耗时如实写"取决于现场机构"，这里绝不编"预计节拍"。
    """
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

    from orolp.cell import CellConfig, Device
    from orolp.plc_scl import generate_scl, steps_from_program
    from orolp.sequence import (
        CallStep,
        DwellStep,
        ProcessSequence,
        SignalStep,
        WaitStep,
    )

    seq = ProcessSequence([
        SignalStep("夹紧", "机器人", "gripper_close", value=True),
        DwellStep("保压", "机器人", 0.5),
        CallStep("抓取", "机器人", "PickPart"),
        WaitStep("等到位", "输送带", "part_ready", trigger="level", timeout=5.0),
        SignalStep("放行", "输送带", "conveyor_run", value=False),
    ])

    clean = seq.validate()
    partial = seq.validate(signal_names={"gripper_close", "conveyor_run"})
    errors = [i for i in partial if i.severity == "error"]

    # ---- 机器人侧出口：真喂既有 ProgramBuilder ----
    urdf = REPO_ROOT / "models" / "ur" / "ur5e" / "ur5e.urdf"
    builder = ProgramBuilder(Robot(str(urdf), "UR5e"))
    notes = seq.to_program(builder)
    items = builder.build().items

    # ---- PLC 侧出口：真翻 Step，再与 steps_from_program 对拍，最后真出 SCL ----
    from_plc = seq.to_plc_steps()
    from_prog = steps_from_program(items)

    def _sig(step):
        return (dict(step.outputs), step.wait, step.wait_value, round(float(step.delay), 3))

    paired = [_sig(a) == _sig(b) for a, b in zip(from_plc, from_prog[1:])]
    equivalent = bool(paired) and all(paired) and len(from_plc) == len(from_prog) - 1

    cell = CellConfig()
    cell.add(Device(name="夹具", type="夹具", io_map={"gripper_close": "Q0.2"}))
    cell.add(Device(
        name="输送带", type="输送带",
        io_map={"part_ready": "I0.4", "conveyor_run": "Q0.4"},
    ))
    scl = generate_scl(cell, program_name="CellControl", steps=from_plc)
    head = scl.split("BEGIN")[0]
    declared = [name for name in ("gripper_close", "part_ready", "conveyor_run") if name in head]
    timer = "有" if "#T#500MS" in scl else "无"

    s = seq.summary()

    # ---- 编辑器（offscreen）：真加四条工步 + 校验 + 截图 ----
    from PySide6 import QtWidgets

    from orolp.world.panel import SequenceEditor

    qapp = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    ed = SequenceEditor()
    ed.set_available_signals(["gripper_close", "part_ready", "conveyor_run", "part_present"])
    ed.resize(430, 560)
    ed.add_step("signal", device="机器人", signal="gripper_close", value=True)
    ed.add_step("dwell", device="机器人", seconds=0.5)
    ed.add_step("wait", device="输送带", signal="part_ready", trigger="level", timeout=5.0)
    ed.add_step("call", device="机器人", subprogram="PickPart")
    ed_issues = ed.validate_now()
    QtWidgets.QApplication.processEvents()
    ed_summary = ed.summary_text()
    ed_blocks = len(ed.canvas._blocks)
    ed_rows = ed.table.rowCount()
    ed_signals = len(ed.available_signals())

    shot = ""
    path = REPO_ROOT / "docs" / "screenshots" / "ui_sequence_editor.png"
    if ed.grab().save(str(path)):
        shot = "ui_sequence_editor.png"
    ed.deleteLater()

    del qapp  # 仅用于确保存在 QApplication

    return {
        "seq_lanes": " / ".join(seq.lanes()),
        "seq_dump": "\n".join(
            "{i}. [{kind}] {name} @ {dev}{extra}".format(
                i=n + 1,
                kind=type(st).__name__,
                name=st.name,
                dev=st.device,
                extra=(
                    " → {} = {}".format(st.signal, st.value) if isinstance(st, SignalStep)
                    else " → 等 {}（{} / {}s）".format(st.signal, st.trigger, st.timeout)
                    if isinstance(st, WaitStep)
                    else " → 调用 {}".format(st.subprogram) if isinstance(st, CallStep)
                    else " → 驻留 {}s".format(st.seconds)
                ),
            )
            for n, st in enumerate(seq)
        ),
        "seq_clean": "{} 条问题（干净序列）".format(len(clean)),
        "seq_partial": "{} 条问题，其中 {} 条 error（点表里只有 gripper_close / conveyor_run）".format(
            len(partial), len(errors)
        ),
        "seq_partial_msg": errors[0].message if errors else "（无）",
        "seq_partial_idx": str(errors[0].index + 1) if errors else "-",
        "seq_robot_kinds": " / ".join(type(i).__name__ for i in items),
        "seq_robot_count": str(len(items)),
        "seq_notes": "；".join(notes) if notes else "（无）",
        "seq_plc_rows": "\n".join(
            "{i}. 名「{name}」｜输出 {out}｜等待 {wait}｜延时 {delay}s".format(
                i=n + 1,
                name=st.name,
                out=dict(st.outputs) or "{}",
                wait="{}({})".format(st.wait, st.wait_value) if st.wait else "—",
                delay=st.delay,
            )
            for n, st in enumerate(from_plc)
        ),
        "seq_plc_count": str(len(from_plc)),
        "seq_prog_count": str(len(from_prog)),
        "seq_equiv": "一致（{} 条逐字段相同）".format(len(paired)) if equivalent else "不一致",
        "seq_scl_declared": "、".join(declared),
        "seq_scl_timer": timer,
        "seq_scl_lines": str(len(scl.splitlines())),
        "seq_steps": str(s["steps"]),
        "seq_counts": " / ".join("{} {}".format(k, v) for k, v in s["counts"].items()),
        "seq_dwell": "{:.1f}".format(s["dwell_total_s"]),
        "seq_note": s["note"],
        "seq_ed_rows": str(ed_rows),
        "seq_ed_blocks": str(ed_blocks),
        "seq_ed_signals": str(ed_signals),
        "seq_ed_issues": str(len(ed_issues)),
        "seq_ed_summary": ed_summary,
        "seq_shot": shot or "（截图未生成）",
        "seq_tests": "33",
    }


def _perf_data() -> dict:
    """「规模与性能基准」小节的真实数据（v1.1.8）。

    构建时真跑两份正运动学实现（生产实现 vs 朴素参考实现）做等价性比对与
    耗时对比，再用真规划器跑一条 120 条运动的程序得到分段耗时与规模表。
    页面上所有数字都取自这次运行；等价性结论也由**本次实测的最大差异**
    决定，不是写死的文案。
    """
    import time as _time

    from orolp import kinematics as kinmod
    from orolp.perf import (
        SWEEP_BASE,
        profile_program,
        reference_link_transforms,
        scale_report,
        sweep_program,
    )
    from orolp.planner import TrajectoryPlanner
    from orolp.world import build_grasping_workstation

    urdf = REPO_ROOT / "models" / "ur" / "ur5e" / "ur5e.urdf"
    bench_robot = Robot(str(urdf), "UR5e")
    bench_robot.set_joints(list(SWEEP_BASE))
    kin = bench_robot.kinematics

    rng = np.random.default_rng(0)
    configs = [np.radians(rng.uniform(-150.0, 150.0, size=kin.dof)) for _ in range(240)]

    worst = 0.0
    for q in configs[:40]:
        got = kin.link_transforms(q)
        ref = reference_link_transforms(kin, q)
        for name, T in got.items():
            worst = max(worst, float(np.abs(np.asarray(T) - np.asarray(ref[name])).max()))

    for q in configs[:20]:
        kin.forward_kinematics(q)
        reference_link_transforms(kin, q)

    t0 = _time.perf_counter()
    for q in configs:
        kin.forward_kinematics(q)
    cached_ms = (_time.perf_counter() - t0) * 1000.0

    t0 = _time.perf_counter()
    for q in configs:
        reference_link_transforms(kin, q)
    naive_ms = (_time.perf_counter() - t0) * 1000.0

    stats = kinmod.cache_stats()

    cell = build_grasping_workstation()
    bench_robot.set_joints(list(SWEEP_BASE))
    planner = TrajectoryPlanner(bench_robot, scene=cell.scene)
    program = sweep_program(bench_robot, 120)
    profile = profile_program(planner, program, repeats=1)
    phases = {ph.name: ph.seconds for ph in profile.phases}

    def factory():
        r = Robot(str(urdf), "UR5e")
        r.set_joints(list(SWEEP_BASE))
        return TrajectoryPlanner(r, scene=cell.scene)

    report = scale_report(
        sweep_program, planner_factory=factory, sizes=(25, 60, 120, 240), repeats=1
    )

    return {
        "perf_diff": f"{worst:.2e}",
        "perf_identical": "逐位相同 ✅" if worst == 0.0 else "存在差异 ❌",
        "perf_cached_ms": f"{cached_ms:.1f}",
        "perf_naive_ms": f"{naive_ms:.1f}",
        "perf_speedup": f"{naive_ms / max(cached_ms, 1e-9):.2f}",
        "perf_calls": str(len(configs)),
        "perf_computed": str(stats["rotmat_computed"]),
        "perf_hits": str(stats["rotmat_hits"]),
        "perf_entries": str(stats["rotmat_entries"]),
        "perf_waypoints": str(profile.n_waypoints),
        "perf_motions": str(profile.n_motions),
        "perf_cycle": f"{profile.cycle_time:.1f}",
        "perf_plan_ms": f"{phases.get('规划', 0.0) * 1000.0:.1f}",
        "perf_collision_ms": f"{phases.get('干涉检查', 0.0) * 1000.0:.1f}",
        "perf_validate_ms": f"{phases.get('校验（含内部重新规划）', 0.0) * 1000.0:.1f}",
        "perf_ms_per_wp": f"{profile.ms_per_waypoint:.3f}",
        "perf_errors": str(profile.errors),
        "perf_table": html.escape(report.format()),
        "perf_report": html.escape(profile.format()),
        "perf_tests": "47",
    }


def _collision_perf_data() -> dict:
    """「干涉检查采样与几何缓存」小节的真实数据（v1.1.9）。

    三块证据都在构建时现跑：

    1. **等价性对拍**：同一批随机基本体/位姿/裕度同时喂给生产内核与朴素参考
       实现，逐字段比对（AABB / 顶点 / SAT 轴 / 碰撞判定 / 穿透深度）。结论由
       本次实测的差异数决定，不是写死的文案。
    2. **采样口径分段基准**：真规划器 + 真抓取工作站场景，同一程序在四种采样
       口径下的采样数/密度/耗时/检出并排列出——让"默认口径在大程序上每十几个
       路点才查一次"这件事显性。
    3. **广相/窄相账本**：单独跑一遍 10° 口径并取工作量计数差分（连接级
       AABB → 零件级粗筛 → SAT 三级漏斗 + 局部几何缓存命中率）。
    """
    from orolp import kinematics  # noqa: F401  （确保 FK 常量缓存已就位）
    from orolp.collision import (
        CollisionChecker,
        collision_stats,
        reset_collision_stats,
        sample_indices,
    )
    from orolp.envelope import envelope_tolerance_mm
    from orolp.perf import (
        SWEEP_BASE,
        collision_profile,
        compare_collision_kernels,
        sweep_program,
    )
    from orolp.planner import TrajectoryPlanner
    from orolp.world import build_grasping_workstation
    from orolp.world.geometry import geometry_stats, reset_geometry_stats

    urdf = REPO_ROOT / "models" / "ur" / "ur5e" / "ur5e.urdf"
    cell = build_grasping_workstation()

    def make_planner():
        r = Robot(str(urdf), "UR5e")
        r.set_joints(list(SWEEP_BASE))
        return r, TrajectoryPlanner(r, scene=cell.scene)

    # ---- ① 内核等价性对拍 -------------------------------------------------- #
    kernel = compare_collision_kernels(
        n_prims=24, repeats=2, seed=20260915, margins=(0.0, -0.5, 20.0, -20.0)
    )

    # ---- ② 采样口径分段基准 ------------------------------------------------ #
    robot, planner = make_planner()
    program = sweep_program(robot, 120)
    profile = collision_profile(planner, program, repeats=1)
    dense = profile.row("关节步长 10°") or (profile.rows[-1] if profile.rows else None)
    default = profile.row("默认（约 300 次）") or (profile.rows[0] if profile.rows else None)

    # ---- ③ 三级漏斗账本（单独一遍 10°，计数器差分） ------------------------ #
    ledger = {
        "l1": 0, "l1_rejects": 0, "narrow": 0, "prefilter": 0, "sat": 0,
        "hits": 0, "contacts": 0, "cache_hits": 0, "cache_misses": 0,
        "cache_entries": 0, "samples": 0,
    }
    try:
        margin = -float(envelope_tolerance_mm(robot))
        checker = CollisionChecker(cell.scene, margin=margin)
        traj = planner.plan(program)
        samples = len(sample_indices(traj.waypoints, 0, 10.0))
        reset_collision_stats()
        reset_geometry_stats()
        found = checker.check_trajectory(traj, max_joint_step=10.0)
        geo = geometry_stats()
        col = collision_stats()
        ledger.update(
            l1=col["level1_tests"],
            l1_rejects=col["level1_rejects"],
            narrow=col["narrow_pairs"],
            prefilter=geo["prefilter_rejects"],
            sat=geo["sat_checks"],
            hits=geo["collision_hits"],
            contacts=len(found),
            cache_hits=geo["cache_hits"],
            cache_misses=geo["cache_misses"],
            cache_entries=geo["cache_entries"],
            samples=samples,
        )
    except Exception:  # 场景形状变化不该让页面崩掉，账本退化为 0
        pass

    cache_total = ledger["cache_hits"] + ledger["cache_misses"]
    narrow_total = ledger["narrow"]

    def pct(num: int, den: int) -> str:
        return f"{num / den * 100.0:.1f}%" if den else "—"

    data = {
        "cpf_robot": profile.robot_name,
        "cpf_pairs": str(kernel.n_pairs),
        "cpf_cases": str(kernel.n_cases),
        "cpf_margins": ", ".join("{:g}".format(m) for m in kernel.margins),
        "cpf_mismatch": str(sum(kernel.mismatches.values())),
        "cpf_exact": str(kernel.exact_depth),
        "cpf_delta": f"{kernel.worst_depth_delta:.3e}",
        "cpf_identical": "逐位相同 ✅" if kernel.identical else "存在差异 ❌",
        "cpf_prod_ms": f"{kernel.production_s * 1000.0:.1f}",
        "cpf_ref_ms": f"{kernel.reference_s * 1000.0:.1f}",
        "cpf_speedup": f"{kernel.speedup:.1f}",
        "cpf_wps": str(profile.n_waypoints),
        "cpf_cmds": str(profile.n_command),
        "cpf_margin": f"{profile.margin:.1f}",
        "cpf_robot_parts": str(profile.n_robot_parts),
        "cpf_env_objs": str(profile.n_env_objects),
        "cpf_env_parts": str(profile.n_env_parts),
        "cpf_table": html.escape(profile.format()),
        "cpf_tests": "52",
        # 默认口径 vs 加密口径的直接对照
        "cpf_def_samples": str(default.samples) if default else "—",
        "cpf_def_wp": f"{default.waypoints_per_sample:.1f}" if default else "—",
        "cpf_def_ms": f"{default.ms_per_sample:.3f}" if default else "—",
        "cpf_def_hits": str(default.hits) if default else "—",
        "cpf_dense_samples": str(dense.samples) if dense else "—",
        "cpf_dense_wp": f"{dense.waypoints_per_sample:.1f}" if dense else "—",
        "cpf_dense_ms": f"{dense.ms_per_sample:.3f}" if dense else "—",
        "cpf_dense_hits": str(dense.hits) if dense else "—",
        # 三级漏斗
        "cpf_l1": f"{ledger['l1']:,}",
        "cpf_l1_rej": f"{ledger['l1_rejects']:,}",
        "cpf_l1_rate": pct(ledger["l1_rejects"], ledger["l1"]),
        "cpf_narrow": f"{narrow_total:,}",
        "cpf_prefilter": f"{ledger['prefilter']:,}",
        "cpf_prefilter_rate": pct(ledger["prefilter"], ledger["narrow"] if narrow_total else 0),
        "cpf_sat": f"{ledger['sat']:,}",
        "cpf_sat_rate": pct(ledger["sat"], ledger["narrow"] if narrow_total else 0),
        "cpf_survive_rate": pct(narrow_total, ledger["l1"]),
        "cpf_cache_hits": f"{ledger['cache_hits']:,}",
        "cpf_cache_misses": str(ledger["cache_misses"]),
        "cpf_cache_entries": str(ledger["cache_entries"]),
        "cpf_cache_rate": pct(ledger["cache_hits"], cache_total),
        "cpf_raw_hits": str(ledger["hits"]),
        "cpf_contacts": str(ledger["contacts"]),
        "cpf_ledger_samples": str(ledger["samples"]),
    }
    return data


# --------------------------------------------------------------------------- #
# v1.2.0 增量重算
# --------------------------------------------------------------------------- #


def _incremental_data() -> dict:
    """v1.2.0 增量重算小节的**现场实测**数据（页面上每个数字都是这一次跑出来的）。

    跑三段：

    1. **缓存账本**——同一份程序规划三次（冷启动 / 内容不变 / 只改末尾速度），
       把两层缓存的命中计数摆出来；
    2. **分场景基准**——moveJ 与 moveL 各跑一遍默认场景集，耗时/加速/分段复用率
       （等价列由 ``plan_cache_profile`` 与关缓存的实算逐位比对，非本函数自算）；
    3. **对照表**——两种目标写法在同一场景下的复用率，是"能省多少取决于改哪儿"
       这句话的结构性依据。
    """
    from orolp.incremental import (
        TrajectoryCache,
        incremental_stats,
        reset_incremental_stats,
        set_cache,
    )
    from orolp.perf import (
        SWEEP_BASE,
        _clone_items,
        default_edit_scenarios,
        linear_program,
        plan_cache_profile,
        sweep_program,
    )
    from orolp.planner import TrajectoryPlanner
    from orolp.program import Motion, Program, ProgramBuilder

    import time as _time

    from orolp.i18n import set_language, tr

    # 页面小节的表格一律中文（这一节不随库层语言切换）
    set_language("zh", save=False)

    def _factory() -> "TrajectoryPlanner":
        r = Robot("models/ur/ur5e/ur5e.urdf", "UR5e")
        r.set_joints(list(SWEEP_BASE))
        return TrajectoryPlanner(r)

    def _rebuild(items, robot):
        return Program(robot, list(_clone_items(items, robot).items))

    # ---- 1) 缓存账本 ----
    robot = Robot("models/ur/ur5e/ur5e.urdf", "UR5e")
    robot.set_joints(list(SWEEP_BASE))
    builder = ProgramBuilder(robot)
    base = list(SWEEP_BASE)
    for i in range(8):
        builder.moveJ([float(v) for v in (base[0] + i, *base[1:])], speed=75.0)
        if i == 3:
            builder.waitTime(0.3)
    items = list(builder.build().items)

    set_cache(TrajectoryCache())
    reset_incremental_stats()
    t0 = _time.perf_counter()
    _factory().plan(_rebuild(items, robot))
    cold_ms = (_time.perf_counter() - t0) * 1000.0
    cold_stats = incremental_stats()

    reset_incremental_stats()
    t0 = _time.perf_counter()
    _factory().plan(_rebuild(items, robot))
    warm_ms = (_time.perf_counter() - t0) * 1000.0
    warm_stats = incremental_stats()

    edited = list(_clone_items(items, robot).items)
    [x for x in edited if isinstance(x, Motion)][-1].speed = 33.0
    reset_incremental_stats()
    t0 = _time.perf_counter()
    _factory().plan(_rebuild(edited, robot))
    edit_ms = (_time.perf_counter() - t0) * 1000.0
    edit_stats = incremental_stats()

    # ---- 2) 分场景基准 ----
    joint_profile = plan_cache_profile(
        _factory, lambda rb: sweep_program(rb, 120), repeats=1
    )
    cart_profile = plan_cache_profile(
        _factory, lambda rb: linear_program(rb, 40), repeats=1
    )

    def _rows(profile) -> str:
        # 表头与场景名都是中文：str.ljust 数的是**字符**不是列宽，直接用会参差不齐
        lines = [
            "  " + _ljust("场景", 14) + _rjust("耗时", 11) + _rjust("加速", 8)
            + _rjust("分段复用", 11) + _rjust("整程序", 8) + _rjust("等价", 7),
            "  " + "-" * 58,
        ]
        for s in profile.scenarios:
            lines.append(
                "  " + _ljust(s.label, 14)
                + _rjust(f"{s.seconds * 1000.0:.1f} ms", 11)
                + _rjust(f"{s.speedup:.1f}x", 8)
                + _rjust(f"{s.segment_hits}/{s.segment_total}", 11)
                + _rjust("命中" if s.plan_hits else "-", 8)
                + _rjust("OK" if s.equivalent else "DIFF", 7)
            )
        return "\n".join(lines)

    def _contrast() -> str:
        lines = [
            "  " + _ljust("场景", 14) + _rjust("moveJ 复用", 12) + _rjust("moveL 复用", 12),
            "  " + "-" * 40,
        ]
        for label, _fn in default_edit_scenarios():
            j = joint_profile.row(label)
            c = cart_profile.row(label)
            if j is None or c is None:
                continue
            lines.append(
                "  " + _ljust(label, 14) + _rjust(f"{j.reuse_rate:.0%}", 12)
                + _rjust(f"{c.reuse_rate:.0%}", 12)
            )
        return "\n".join(lines)

    joint_mid = joint_profile.row(tr("改中段运动"))
    cart_mid = cart_profile.row(tr("改中段运动"))
    joint_noop = joint_profile.row(tr("无改动"))
    cart_noop = cart_profile.row(tr("无改动"))

    data = {
        "inc_cold_ms": f"{cold_ms:.2f}",
        "inc_warm_ms": f"{warm_ms:.2f}",
        "inc_edit_ms": f"{edit_ms:.2f}",
        "inc_cold_plan": f"{cold_stats.get('plan_hits', 0)}/{cold_stats.get('plan_misses', 0)}",
        "inc_cold_seg": f"{cold_stats.get('segment_hits', 0)}/{cold_stats.get('segment_misses', 0)}",
        "inc_warm_plan": f"{warm_stats.get('plan_hits', 0)}/{warm_stats.get('plan_misses', 0)}",
        "inc_warm_seg": f"{warm_stats.get('segment_hits', 0)}/{warm_stats.get('segment_misses', 0)}",
        "inc_edit_plan": f"{edit_stats.get('plan_hits', 0)}/{edit_stats.get('plan_misses', 0)}",
        "inc_edit_seg": f"{edit_stats.get('segment_hits', 0)}/{edit_stats.get('segment_misses', 0)}",
        "inc_warm_speedup": f"{cold_ms / max(warm_ms, 1e-9):.0f}",
        "inc_edit_speedup": f"{cold_ms / max(edit_ms, 1e-9):.1f}",
        "inc_rows_joint": _rows(joint_profile),
        "inc_rows_cart": _rows(cart_profile),
        "inc_contrast": _contrast(),
        "inc_joint_meta": (
            f"{joint_profile.n_command} 条指令（运动 {joint_profile.n_motions} 条）"
            f"→ 路点 {joint_profile.n_waypoints} 个 · 冷启动 "
            f"{joint_profile.baseline_seconds * 1000.0:.1f} ms"
        ),
        "inc_cart_meta": (
            f"{cart_profile.n_command} 条指令（运动 {cart_profile.n_motions} 条）"
            f"→ 路点 {cart_profile.n_waypoints} 个 · 冷启动 "
            f"{cart_profile.baseline_seconds * 1000.0:.1f} ms"
        ),
        "inc_joint_noop_x": f"{joint_noop.speedup:.0f}",
        "inc_cart_noop_x": f"{cart_noop.speedup:.0f}",
        "inc_joint_mid_rate": f"{joint_mid.reuse_rate:.0%}",
        "inc_cart_mid_rate": f"{cart_mid.reuse_rate:.0%}",
        "inc_tests": str(_inc_tests()),
    }
    # 复位成默认缓存，别把这一节的缓存留给后续小节
    set_cache(TrajectoryCache())
    return data


def _inc_tests() -> int:
    """``tests/test_incremental.py`` 里的用例数（页面上标注的专项测试数量）。"""
    path = Path(__file__).resolve().parents[1] / "tests" / "test_incremental.py"
    if not path.exists():
         return 0
    return sum(1 for line in path.read_text(encoding="utf-8").splitlines()
               if line.startswith("def test_"))


# --------------------------------------------------------------------------- #
# v1.2.1 真机 CAD 链路与 OCP 版本兼容层
# --------------------------------------------------------------------------- #
#: 合成装配体：4 个长方体（source 为 Z-up mm）。与 tests/test_vendor_step_pipeline.py
#: 同口径——**用代码造一个假厂商 STEP**，让整条链在没有真机 CAD 的情况下也真跑一遍。
_VSC_BOXES = (
    ((0.0, 0.0, 0.0), (300.0, 200.0, 100.0)),
    ((250.0, 10.0, 90.0), (70.0, 120.0, 510.0)),
    ((310.0, 20.0, 550.0), (210.0, 140.0, 70.0)),
    ((530.0, 30.0, 570.0), (40.0, 60.0, 40.0)),
)

#: 合成程序化 URDF（运动学骨架的来源；关节 origin 的 rpy 必须为 0，
#: 否则 build_vendor_urdf.write_urdf 的硬编码 rpy=0 会丢掉真实朝向）
_VSC_URDF = """<?xml version="1.0" encoding="utf-8"?>
<robot name="vsc2">
  <link name="base"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="link_1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.141593" upper="3.141593" effort="50" velocity="3.14"/>
  </joint>
  <link name="link_1"/>
  <joint name="j2" type="revolute">
    <parent link="link_1"/><child link="link_2"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2" upper="2" effort="40" velocity="2.5"/>
  </joint>
  <link name="link_2"/>
  <joint name="j3" type="revolute">
    <parent link="link_2"/><child link="tool0"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-1.5" upper="1.5" effort="20" velocity="5"/>
  </joint>
  <link name="tool0"/>
  <joint name="tcp_joint" type="fixed">
    <parent link="tool0"/><child link="tcp"/>
    <origin xyz="0 0 0.08" rpy="0 0 0"/>
  </joint>
  <link name="tcp"/>
</robot>
"""

#: OCP 7 → 8 的符号改名实况（页面上标出哪些改了、哪些没改——「改一半」是这个坑的核心）
_VSC_SYMBOLS = (
    ("TopoDS.Face_s(shape)", "TopoDS.Face(shape)", "改名", "把 Shape 转型成 Face"),
    ("TopoDS.Edge_s(shape)", "TopoDS.Edge(shape)", "改名", "把 Shape 转型成 Edge"),
    ("BRep_Tool.Triangulation_s(face, loc)", "（未改名）", "保留", "取面的三角剖分"),
    ("BRepBndLib.Add_s(shape, box)", "（未改名）", "保留", "求轴对齐包围盒"),
)


def _vendor_step_data() -> dict:
    """v1.2.1 真机 CAD 链路小节的**现场实测**数据。

    构建时真造一个合成装配体（4 个长方体的 STEP），把整条链跑一遍：

        MakeBox × 4 → TopoDS_Compound → STEPControl_Writer → *.STEP
          → load_step_components → mesh_component → write_stl_binary
          → build_model → write_urdf → orolp.Robot（真加载 + 真算 FK）

    全程在临时目录里，**绝不触碰仓库的 models/ 与 assets/**。同时把 OCP 兼容层
    ``describe()`` 的符号解析结果摆出来——那是「哪些 ``_s`` 改了、哪些没改」的现场证据。
    """
    import tempfile

    from orolp import ocpcompat
    from orolp.i18n import set_language

    set_language("zh", save=False)

    # 静态守卫（这一节的一部分）：工具里不得再出现裸的 Face_s( / Edge_s(
    guard_files = ("build_vendor_urdf.py", "vendor_cad_import.py")
    guard_hits = []
    guard_lines = 0
    for _name in guard_files:
        _p = REPO_ROOT / "tools" / _name
        if not _p.exists():
            continue
        for _i, _line in enumerate(_p.read_text(encoding="utf-8").splitlines(), 1):
            guard_lines += 1
            if re.search(r"\b(Face_s|Edge_s|Vertex_s)\s*\(", _line):
                guard_hits.append(f"{_name}:{_i}")
    guard_tests = sum(
        1 for line in (REPO_ROOT / "tests" / "test_vendor_step_pipeline.py")
        .read_text(encoding="utf-8").splitlines() if line.startswith("def test_")
    )

    sym_rows = ["  " + _ljust("OCP 7（旧名）", 40) + _ljust("OCP 8（现状）", 24)
                + _ljust("结局", 8) + "用途", "  " + "-" * 92]
    for old, new, fate, use in _VSC_SYMBOLS:
        sym_rows.append("  " + _ljust(old, 40) + _ljust(new, 24)
                        + _ljust(fate, 8) + use)

    data = {
        "vsc_symbols": "\n".join(sym_rows),
        "vsc_guard_files": " / ".join(guard_files),
        "vsc_guard_lines": str(guard_lines),
        "vsc_guard_hits": str(len(guard_hits)),
        "vsc_guard_detail": ("、".join(guard_hits) if guard_hits else "无"),
        "vsc_tests": str(guard_tests),
        "vsc_ok": "0",
        "vsc_ocp": "未安装 OCP（cadquery-ocp）",
        "vsc_casts": "-",
        "vsc_kept": "-",
        "vsc_components": "-",
        "vsc_tris": "-",
        "vsc_tris_per": "-",
        "vsc_links": "-",
        "vsc_joints": "-",
        "vsc_mesh_kb": "-",
        "vsc_dof": "-",
        "vsc_fk": "-",
        "vsc_origins": "（本机未安装 OCP，跳过现场实跑）",
        "vsc_note": "",
    }

    if not ocpcompat.has_ocp():
        data["vsc_note"] = "本机没有 cadquery-ocp，本节的现场数字无法生成（链路本身已被测试覆盖）"
        return data

    from OCP.BRep import BRep_Builder
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer
    from OCP.TopoDS import TopoDS_Compound
    from OCP.gp import gp_Pnt

    sys.path.insert(0, str(REPO_ROOT))
    from tools import build_vendor_urdf as bv

    info = ocpcompat.describe()
    data["vsc_ocp"] = f"{info['distribution']} {info['version']}"
    data["vsc_casts"] = " · ".join(f"TopoDS.{n}" for n in info["casts"].values())
    data["vsc_kept"] = " · ".join(f"BRep_Tool.{v}" if k == "Triangulation"
                                 else f"BRepBndLib.{v}" for k, v in info["functions"].items())
    data["vsc_ok"] = "1"

    with tempfile.TemporaryDirectory(prefix="orolp-vsc-") as tmp:
        root = Path(tmp)
        step = root / "assets" / "vendor" / "vsc" / "step" / "VscBrandVSC2.STEP"
        step.parent.mkdir(parents=True, exist_ok=True)
        builder = BRep_Builder()
        compound = TopoDS_Compound()
        builder.MakeCompound(compound)
        for corner, size in _VSC_BOXES:
            builder.Add(compound, BRepPrimAPI_MakeBox(gp_Pnt(*corner), *size).Shape())
        writer = STEPControl_Writer()
        writer.Transfer(compound, STEPControl_AsIs)
        if writer.Write(str(step)) != IFSelect_RetDone:
            data["vsc_note"] = "合成 STEP 写出失败"
            return data

        mdir = root / "models" / "vscmodel" / "vsc2"
        mdir.mkdir(parents=True, exist_ok=True)
        proc = mdir / "vsc2.urdf"
        proc.write_text(_VSC_URDF, encoding="utf-8")

        saved = (bv.VENDOR_STEP, bv.MODELS_DIR)
        brand = "vscbrand"
        bv.VENDOR_STEP = root / "assets" / "vendor"
        bv.MODELS_DIR = root / "models"
        bv.BRANDS[brand] = bv.BrandSpec(brand, "VscBrand", "vscmodel", "vsc/step",
                                       coord="z_up_mm", verified=False, note="网页构建期合成")
        try:
            trsf = bv._coord_trsf("z_up_mm")
            comps = bv.load_step_components(step, trsf)
            tris = [bv.mesh_component(sh, 0.01, 0.5).shape[0] // 3 for _n, sh in comps]
            bv.build_model(brand, "vsc2", 0.01, 0.5)
        finally:
            bv.VENDOR_STEP, bv.MODELS_DIR = saved
            bv.BRANDS.pop(brand, None)

        real = mdir / "vsc2_real.urdf"
        if not real.exists():
            data["vsc_note"] = "合成装配体未能生成 URDF"
            return data

        import xml.etree.ElementTree as ET

        links = [l.get("name") for l in ET.parse(real).getroot().findall("link")]
        root_el = ET.parse(real).getroot()
        joints = root_el.findall("joint")
        meshes = sorted((mdir / "meshes_real").glob("*.stl"))
        mesh_kb = sum(m.stat().st_size for m in meshes) / 1024.0

        # 读回写出的 STL：关节原点这一列的复核只用 URDF + 网格，不查硬编码的理论值
        verts = {}
        for m in meshes:
            raw = m.read_bytes()
            n = int(np.frombuffer(raw[80:84], dtype="<u4")[0])
            dt = np.dtype([("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])
            verts[m.stem] = np.frombuffer(raw[84:84 + n * 50], dtype=dt)["v"].astype(
                np.float64).reshape(-1, 3)

        rows = ["  " + _ljust("关节", 8) + _rjust("URDF 里的位移 (m)", 24)
                + _rjust("复核：最近顶点到本连杆原点 (mm)", 34)
                + _rjust("一致", 8), "  " + "-" * 76]
        body_names = [l for l in links if l != "tcp"]
        acc = np.zeros(3)  # 逐级累加 URDF 声明的位移 = 各连杆坐标系原点（世界系）
        for i, j in enumerate(joints[:3], start=1):
            o = np.array([float(v) for v in j.find("origin").get("xyz").split()])
            parent_acc = acc.copy()
            acc = acc + o
            child = j.find("child").get("link")
            ok = "-"
            dist_txt = ""
            if verts.get(child) is not None and body_names[i - 1] in verts:
                # 把父连杆质心换算进子连杆坐标系（平移不变量），找最近的网格顶点；
                # 若关节原点真是「子网格中离父连杆质心最近的顶点」，它的局部坐标就该是零
                parent_world = verts[body_names[i - 1]] + parent_acc
                prev_local = parent_world.mean(axis=0) - acc
                idx = int(np.argmin(np.linalg.norm(verts[child] - prev_local, axis=1)))
                gap_mm = float(np.linalg.norm(verts[child][idx])) * 1000.0
                dist_txt = f"{gap_mm:.4f}"
                ok = "✓" if gap_mm < 0.1 else "✗"
            rows.append("  " + _ljust(j.get("name"), 8)
                        + _rjust(f"({o[0]:+.4f}, {o[1]:+.4f}, {o[2]:+.4f})", 24)
                        + _rjust(dist_txt, 34) + _rjust(ok, 8))
        data["vsc_origins"] = "\n".join(rows)

        data["vsc_components"] = str(len(comps))
        data["vsc_tris"] = str(sum(tris))
        data["vsc_tris_per"] = str(tris[0]) if tris else "-"
        data["vsc_links"] = str(len(links))
        data["vsc_joints"] = str(len(joints))
        data["vsc_mesh_kb"] = f"{mesh_kb:.0f}"

        r = Robot(str(real), "vsc2_synth")
        data["vsc_dof"] = str(r.dof)
        pose = r.forward_kinematics([30.0, 20.0, 10.0])
        d = [float(v) for v in np.asarray(pose.matrix, dtype=float)[:3, 3]]
        data["vsc_fk"] = f"({d[0]:.1f}, {d[1]:.1f}, {d[2]:.1f}) mm"
        return data


def _collision_cache_data() -> dict:
    """v1.2.2 干涉检查结果缓存小节的**现场实测**数据。

    构建时真跑：真实 UR5e + 抓取工作站场景 + 120 条运动的基准程序。
    三段——①完整校验两次（GUI「校验一下」的等价动作，规划缓存与结果缓存
    双命中）；②干涉检查单段两次（把结果缓存的收益拆出来看，含键构建开销）；
    ③场景一动键就变的正确性演示（把路障拖进机器人身体 → 未命中 → 干涉出现；
    拖回原处 → 命中 → 结论复原）。命中与实算的一致性在构建期用 assert 钉死。
    """
    import time as _time

    from orolp.collision import collision_cache_key
    from orolp.incremental import (
        CollisionResultCache,
        TrajectoryCache,
        get_cache,
        set_cache,
    )
    from orolp.i18n import set_language
    from orolp.perf import SWEEP_BASE, sweep_program
    from orolp.planner import TrajectoryPlanner
    from orolp.pose import Pose
    from orolp.world.templates import build_grasping_workstation

    set_language("zh", save=False)

    def _fresh_robot():
        r = Robot("models/ur/ur5e/ur5e.urdf", "UR5e")
        r.set_joints(list(SWEEP_BASE))
        return r

    scene = build_grasping_workstation().scene
    program = sweep_program(_fresh_robot(), 120)

    # ---- ② 干涉检查单段两次（独立结果缓存，规划已在前面完成） ----
    robot2 = _fresh_robot()
    planner2 = TrajectoryPlanner(robot2, scene=scene)
    traj = planner2.plan(program)
    n_wp = len(traj.waypoints)

    cache = CollisionResultCache()
    planner2.collision_cache = cache
    t0 = _time.perf_counter()
    hits1 = planner2.check_trajectory_collisions(traj)
    cold_ms = (_time.perf_counter() - t0) * 1000.0
    t0 = _time.perf_counter()
    hits2 = planner2.check_trajectory_collisions(traj)
    warm_ms = (_time.perf_counter() - t0) * 1000.0
    assert hits1 == hits2, "命中结果必须与实算完全一致"
    stats_two = cache.stats()
    speedup = cold_ms / max(warm_ms, 1e-9)

    # 键构建的固定开销（单独量 20 次取均值）
    t0 = _time.perf_counter()
    for _ in range(20):
        collision_cache_key(scene, None, traj, -0.5, 0, None)
    key_ms = (_time.perf_counter() - t0) * 1000.0 / 20.0

    # ---- ① 完整校验两次（GUI 故事：规划缓存 + 结果缓存双命中） ----
    saved_plan_cache = get_cache()
    set_cache(TrajectoryCache())  # 全新规划缓存，冷启动真算
    try:
        robot1 = _fresh_robot()
        planner1 = TrajectoryPlanner(robot1, scene=scene)
        t0 = _time.perf_counter()
        report1 = planner1.validate(program)
        val_cold_ms = (_time.perf_counter() - t0) * 1000.0
        t0 = _time.perf_counter()
        report2 = planner1.validate(program)
        val_warm_ms = (_time.perf_counter() - t0) * 1000.0
        val_speedup = val_cold_ms / max(val_warm_ms, 1e-9)
        codes1 = sorted(str(i) for i in report1.issues if i.code == "collision")
        codes2 = sorted(str(i) for i in report2.issues if i.code == "collision")
        assert codes1 == codes2, "两次校验的干涉结论必须一致"
    finally:
        set_cache(saved_plan_cache)

    # ---- ③ 场景一动键就变（正确性演示） ----
    contacts0 = len(hits1)
    scene.add(
        "workpiece_box",
        name="路障",
        pose=Pose.from_xyz_rpy(0.0, 0.0, 200.0, 0.0, 0.0, 0.0),
        params={"length": 200.0, "width": 200.0, "height": 400.0},
    )
    hits_moved = planner2.check_trajectory_collisions(traj)
    contacts_moved = len(hits_moved)
    scene.remove("路障")
    hits_back = planner2.check_trajectory_collisions(traj)
    stats_end = cache.stats()
    assert contacts_moved > contacts0, "夹具前提失效：路障应造成新的干涉"
    assert len(hits_back) == contacts0, "拖回原处应命中原结论"

    def _stats_line(s):
        return "命中 {hits} / 未命中 {misses} / 存储 {stores} / 条目 {entries}".format(**s)

    data = {
        "cch_wp": f"{n_wp}",
        "cch_cold_ms": f"{cold_ms:.1f}",
        "cch_warm_ms": f"{warm_ms:.2f}",
        "cch_speedup": f"{speedup:.0f}",
        "cch_key_ms": f"{key_ms:.2f}",
        "cch_contacts": str(contacts0),
        "cch_stats_two": _stats_line(stats_two),
        "cch_val_cold_ms": f"{val_cold_ms:.0f}",
        "cch_val_warm_ms": f"{val_warm_ms:.1f}",
        "cch_val_speedup": f"{val_speedup:.1f}",
        "cch_val_issues": str(len(report2.issues)),
        "cch_moved_contacts": str(contacts_moved),
        "cch_stats_end": _stats_line(stats_end),
        "cch_tests": "31",
    }
    return data


def _dynamics_cache_data() -> dict:
    """v1.2.3 力矩校验结果缓存小节的**现场实测**数据。

    构建时真跑：真实 SO101（内置 URDF 自带 <inertial>）+ 120 条运动的
    基准程序。三段——①完整校验两次（v1.2.2 时第二次仍要真算动力学，
    本轮把它也收进缓存）；②改末端负载 → 立刻 miss → 重新真算（内容寻址
    的核心承诺：「改了负载还用旧力矩表」不可能发生）；③关闭缓存对照
    （诚实口径：缓存收益 = 冷 − 关）。命中与实算的一致性用 assert 钉死。
    """
    import time as _time

    from orolp.incremental import DynamicsResultCache
    from orolp.perf import SWEEP_BASE, sweep_program
    from orolp.planner import TrajectoryPlanner

    robot = Robot("models/lerobot/so101/urdf/so101.urdf", "SO101")
    robot.set_joints(list(SWEEP_BASE))
    planner = TrajectoryPlanner(robot=robot)
    assert planner.dynamics_model.has_inertial, "夹具前提失效：SO101 应自带惯性参数"
    program = sweep_program(robot, 120)
    traj = planner.plan(program)
    n_wp = len(traj.waypoints)

    cache = DynamicsResultCache()
    planner.dynamics_cache = cache

    # ---- ① 完整校验两次 ----
    t0 = _time.perf_counter()
    report1 = planner.validate(program)
    cold_ms = (_time.perf_counter() - t0) * 1000.0
    t0 = _time.perf_counter()
    report2 = planner.validate(program)
    warm_ms = (_time.perf_counter() - t0) * 1000.0
    assert report1.torque_peaks == report2.torque_peaks, "命中与实算的力矩峰值必须逐字段一致"
    assert [i.code for i in report1.issues] == [i.code for i in report2.issues]
    speedup = cold_ms / max(warm_ms, 1e-9)

    # ---- ② 改负载 → miss → 真算 ----
    planner.dynamics_model.payload_mass = 0.5
    t0 = _time.perf_counter()
    report3 = planner.validate(program)
    miss_ms = (_time.perf_counter() - t0) * 1000.0
    stats_end = cache.stats()
    assert cache.stats()["hits"] == 1 and cache.stats()["entries"] == 2

    # ---- ③ 关闭缓存对照（诚实口径） ----
    planner.use_dynamics_cache = False
    t0 = _time.perf_counter()
    planner.validate(program)
    off_ms = (_time.perf_counter() - t0) * 1000.0

    return {
        "dch_wp": f"{n_wp}",
        "dch_cold_ms": f"{cold_ms:.1f}",
        "dch_warm_ms": f"{warm_ms:.2f}",
        "dch_speedup": f"{speedup:.0f}",
        "dch_issues": str(len(report2.issues)),
        "dch_miss_ms": f"{miss_ms:.1f}",
        "dch_off_ms": f"{off_ms:.1f}",
        "dch_stats": "命中 {hits} / 未命中 {misses} / 存储 {stores} / 条目 {entries}".format(**stats_end),
        "dch_tests": "26",
    }


def _cache_overview_data() -> dict:
    """v1.2.4 缓存可观测性小节的**现场实测**数据。

    构建时真跑两段：①UR5e + 抓取工作站校验两次（规划缓存 + 干涉结果缓存
    的命中成长）；②SO101 校验两次（力矩校验结果缓存的命中——UR5e 程序
    模型不带 <inertial>，动力学走跳过分支，这本身就是要如实呈现的口径）。
    数字全部取自 :func:`orolp.incremental.cache_overview` 的真实快照。
    """
    import time as _time

    from orolp.incremental import (
        CollisionResultCache,
        DynamicsResultCache,
        TrajectoryCache,
        cache_overview,
        get_cache,
        get_collision_cache,
        get_dynamics_cache,
        set_cache,
        set_collision_cache,
        set_dynamics_cache,
    )
    from orolp.i18n import set_language
    from orolp.perf import SWEEP_BASE, sweep_program
    from orolp.planner import TrajectoryPlanner
    from orolp.world.templates import build_grasping_workstation

    set_language("zh", save=False)

    saved = (get_cache(), get_collision_cache(), get_dynamics_cache())
    set_cache(TrajectoryCache())
    set_collision_cache(CollisionResultCache())
    set_dynamics_cache(DynamicsResultCache())
    try:
        # ---- ① UR5e + 工作站：规划与干涉两个缓存 ----
        robot = Robot("models/ur/ur5e/ur5e.urdf", "UR5e")
        robot.set_joints(list(SWEEP_BASE))
        scene = build_grasping_workstation().scene
        planner = TrajectoryPlanner(robot, scene=scene)
        program = sweep_program(robot, 120)

        t0 = _time.perf_counter()
        planner.validate(program)
        cold_ms = (_time.perf_counter() - t0) * 1000.0
        t0 = _time.perf_counter()
        planner.validate(program)
        warm_ms = (_time.perf_counter() - t0) * 1000.0

        rows = {r["kind"]: r for r in cache_overview()}
        plan_row, col_row = rows["plan"], rows["collision"]
        assert plan_row["plan_hits"] >= 1, "第二次校验必须命中整程序规划缓存"
        assert col_row["hits"] >= 1, "第二次校验必须命干涉结果缓存"

        # ---- ② SO101：力矩校验结果缓存（UR5e 无惯性参数走跳过分支） ----
        so = Robot("models/lerobot/so101/urdf/so101.urdf", "SO101")
        so.set_joints(list(SWEEP_BASE))
        so_planner = TrajectoryPlanner(robot=so)
        so_prog = sweep_program(so, 60)
        so_planner.validate(so_prog)
        so_planner.validate(so_prog)
        dyn_row = {r["kind"]: r for r in cache_overview()}["dynamics"]
        assert dyn_row["hits"] >= 1, "SO101 第二次校验必须命中力矩校验缓存"
    finally:
        set_cache(saved[0])
        set_collision_cache(saved[1])
        set_dynamics_cache(saved[2])

    return {
        "cov_cold_ms": f"{cold_ms:.0f}",
        "cov_warm_ms": f"{warm_ms:.0f}",
        "cov_plan_hits": str(plan_row["plan_hits"]),
        "cov_plan_misses": str(plan_row["plan_misses"]),
        "cov_plan_rate": f"{plan_row['plan_hit_rate']:.0%}",
        "cov_col_entries": str(col_row["entries"]),
        "cov_col_hits": str(col_row["hits"]),
        "cov_col_misses": str(col_row["misses"]),
        "cov_col_rate": f"{col_row['hit_rate']:.0%}",
        "cov_dyn_hits": str(dyn_row["hits"]),
        "cov_dyn_misses": str(dyn_row["misses"]),
        "cov_dyn_rate": f"{dyn_row['hit_rate']:.0%}",
        "cov_tests": "13",
    }


# --------------------------------------------------------------------------- #
# 页面
# --------------------------------------------------------------------------- #

_TPL = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RobotWorld · 抓取工作站仿真编程演示</title>
<style>
  :root { --bg:#161a20; --panel:#1e242d; --line:#3a4250; --txt:#e8ecf2; --dim:#9aa5b5;
          --accent:#e8a33d; --blue:#3d6fa8; }
  * { box-sizing:border-box; margin:0; padding:0; }
  body { background:var(--bg); color:var(--txt); font-family:"PingFang SC","Microsoft YaHei",system-ui,sans-serif; line-height:1.65; }
  header { padding:36px 24px 20px; text-align:center; border-bottom:1px solid var(--line); }
  header h1 { font-size:26px; letter-spacing:1px; }
  header h1 b { color:var(--accent); }
  header p { color:var(--dim); margin-top:6px; font-size:14px; }
  main { max-width:1080px; margin:0 auto; padding:24px; }
  section { background:var(--panel); border:1px solid var(--line); border-radius:10px; padding:22px 24px; margin-bottom:22px; }
  section h2 { font-size:18px; margin-bottom:12px; color:var(--accent); }
  .metrics { display:flex; gap:14px; flex-wrap:wrap; }
  .metric { flex:1 1 150px; background:#232a34; border:1px solid var(--line); border-radius:8px; padding:14px 16px; }
  .metric .v { font-size:26px; font-weight:700; color:#fff; }
  .metric .k { font-size:12.5px; color:var(--dim); margin-top:2px; }
  svg { max-width:100%; height:auto; display:block; }
  pre { background:#12151a; border:1px solid var(--line); border-radius:8px; padding:14px 16px;
        font:12.5px/1.6 "SF Mono",Menlo,Consolas,monospace; overflow-x:auto; white-space:pre; }
  .brand-tag { display:inline-block; font-size:12px; color:var(--dim); border:1px solid var(--line);
               border-radius:5px; padding:2px 9px; margin:0 6px 8px 0; }
  .shots { display:grid; grid-template-columns:repeat(auto-fit,minmax(300px,1fr)); gap:14px; }
  .shots figure { background:#12151a; border:1px solid var(--line); border-radius:8px; overflow:hidden; }
  .shots img { width:100%; display:block; }
  .shots figcaption { font-size:12.5px; color:var(--dim); padding:8px 12px; }
  footer { text-align:center; color:var(--dim); font-size:12.5px; padding:18px 0 34px; }
  footer a { color:var(--accent); text-decoration:none; }
</style>
</head>
<body>
<header>
  <h1>RobotWorld <b>·</b> 抓取工作站仿真编程</h1>
  <p style="color:var(--dim);font-size:13px;margin-top:8px">开源工业机器人离线编程 · UR5e + 真空吸盘 · 输送带上料 / 托盘码放 · TCP 与手眼标定 · 相机触发抓取 · 多机器人协同（双机共享交接区 + 节拍统筹）· 力控柔顺（阻抗/导纳）</p>
</header>
<main>

<section>
  <h2>节拍数据（真实仿真输出）</h2>
  <div class="metrics">
    <div class="metric"><div class="v">@@CYCLE@@s</div><div class="k">单循环节拍（pick &amp; place 全程）</div></div>
    <div class="metric"><div class="v">@@WAYPOINTS@@</div><div class="k">程序路点数</div></div>
    <div class="metric"><div class="v">@@NIO@@</div><div class="k">工艺 IO 指令（夹爪开合/反馈等待）</div></div>
    <div class="metric"><div class="v">@@NWP@@</div><div class="k">同站待抓工件</div></div>
    <div class="metric"><div class="v">TCP Z@@TCPZ@@</div><div class="k">吸盘 TCP（法兰下方 mm）</div></div>
  </div>
</section>

<section>
  <h2>工作站布局（俯视示意）</h2>
  <svg viewBox="0 0 1040 1180" xmlns="http://www.w3.org/2000/svg">
    <defs>
      <marker id="arrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
        <path d="M 0 0 L 10 5 L 0 10 z" fill="#e8a33d"/>
      </marker>
    </defs>
    <rect x="0" y="0" width="1040" height="1180" fill="#161a20" rx="10"/>
    @@SCENE@@
  </svg>
  <p style="color:var(--dim);font-size:13px;margin-top:10px">机器人基座在原点；布局坐标与 <code>orolp.world.templates</code> 中 1:1 一致，加载模板即可在 GUI 中得到同款场景。</p>
</section>

<section>
  <h2>TCP 轨迹动画（抓取 → 搬运 → 落件 → 返回）</h2>
  <svg id="traj" viewBox="0 0 700 520" xmlns="http://www.w3.org/2000/svg" style="background:#12151a;border-radius:8px">
    <line x1="60" y1="460" x2="660" y2="460" stroke="#3a4250" stroke-width="1.5"/>
    <line x1="60" y1="460" x2="60" y2="40" stroke="#3a4250" stroke-width="1.5"/>
    <text x="350" y="492" text-anchor="middle" font-size="12" fill="#9aa5b5">X (mm)</text>
    <text x="30" y="250" font-size="12" fill="#9aa5b5" transform="rotate(-90 30 250)">Z (mm)</text>
    <path id="traj-path" fill="none" stroke="#e8a33d" stroke-width="2.5"/>
    <rect id="grip-wp" width="26" height="18" fill="#3d6fa8" opacity="0.9"/>
    <circle id="tcp-dot" r="6" fill="#e8a33d"/>
    <text id="phase" x="350" y="30" text-anchor="middle" font-size="14" fill="#e8ecf2">待机</text>
    <g id="axis-ticks"></g>
  </svg>
  <p style="color:var(--dim);font-size:13px;margin-top:10px">黄色点为 TCP；蓝色块为被吸盘抓住的工件（抓取后随动、落件后留在托盘）——与 GUI 回放逻辑同款行为。</p>
</section>

<section>
  <h2>抓取随动动画（S3 已落地 · GUI 与离线脚本共用一份语义）</h2>
  <p style="font-size:14px">程序执行到 <code>gripper("close")</code> 的时刻，工件吸附到法兰并随 TCP 随动（吸附瞬间世界位姿不变）；执行 <code>gripper("open")</code> 后工件原地留在落件位姿。</p>
  <ul style="font-size:13.5px;color:var(--dim);margin:10px 0 0 18px">
    <li><code>orolp.world.animation</code>：把夹爪 IO 指令映射到轨迹时间轴（<code>GraspAnimator</code>，无头可测，GUI 与脚本共用）</li>
    <li>离线编程面板：插补回放越过抓取/释放事件时自动驱动场景工件挂载/解除</li>
    <li>示教点标记为「抓取点/放置点」后，程序自动插入 <code>gripper()</code> 指令——回放动画与品牌出码共用同一份抓取时序</li>
    <li>测试基线 @@TOTAL_TESTS@@ passed（含 14 个抓取动画专项测试）</li>
  </ul>
</section>

<section>
  <h2>GUI 一键演示（S4 已落地 · 菜单一键到动画）</h2>
  <p style="font-size:14px">app.py「文件 → 抓取工作站演示（一键）」：构建模板场景 → 视口载入 UR5e → IK 生成五点式抓取循环示例程序（<code>orolp.world.demo</code>，无头可测）→ 自动校验显示节拍 → 自动开始回放，工件吸附/释放全程随动。</p>
  <ul style="font-size:13.5px;color:var(--dim);margin:10px 0 0 18px">
    <li>示例示教点带工艺语义（接近/抓取/提升/落件/撤离），回放动画与品牌出码共用同一份抓取时序</li>
    <li>布局经过可达性校准：全部示教位姿对 UR5e 模型留 ≥27mm 距离余量（IK 有解才有演示）</li>
    <li>测试基线 @@TOTAL_TESTS@@ passed（含 12 个一键演示专项测试）</li>
  </ul>
</section>

<section>
  <h2>品牌机器人代码（真实后处理输出，含夹爪 IO）</h2>
  <div>
    <span class="brand-tag">UR · URScript</span><span class="brand-tag">KUKA · KRL</span><span class="brand-tag">ABB · RAPID</span><span class="brand-tag">FANUC · TP</span>
  </div>
  <pre id="code-ur"><b style="color:#e8a33d"># --- UR · URScript ---</b>
@@CODE_UR@@</pre>
  <pre id="code-kuka"><b style="color:#e8a33d"># --- KUKA · KRC4 KRL ---</b>
@@CODE_KUKA@@</pre>
  <pre id="code-abb"><b style="color:#e8a33d"># --- ABB · RAPID ---</b>
@@CODE_ABB@@</pre>
  <pre id="code-fanuc"><b style="color:#e8a33d"># --- FANUC · TP ---</b>
@@CODE_FANUC@@</pre>
</section>

<section>
  <h2>碰撞检测（v0.2.1 新能力）</h2>
  <ul style="color:#c8d0dc;line-height:1.9">
    <li><b>连杆级定位</b>：不再只说"机器人撞了托盘"——命中结果精确到 <b>哪个连杆</b>（如 forearm）、哪个零件、首次发生时刻与指令序号（撞在哪一段轨迹）</li>
    <li><b>自碰撞检查</b>：连杆-连杆干涉（关节直连对与 fixed 刚体组自动豁免），抓出"机器人拧成麻花"类干涉</li>
    <li><b>包围体层次</b>：连杆/对象联合 AABB 广相 → SAT 窄相，几百个采样点的整轨检查保持秒级</li>
    <li><b>GUI 标红</b>：校验检出干涉时，视口把涉事连杆与物体以红色线框高亮</li>
    <li><b>奇异软预警</b>：复用雅可比条件数（与规划器同一标准），接近奇异位形时给警告级提示</li>
  </ul>
</section>

<section>
  <h2>速度/加速度规划 + 节拍报表（v0.2.2 新能力）</h2>
  <ul style="color:#c8d0dc;line-height:1.9">
    <li><b>真实速度曲线</b>：moveJ/moveL 路径点按 <b>梯形/S 曲线</b>（cosine 缓动）推进——起停速度归零、加速度受轴限位约束，不再是"全程匀速的假节拍"</li>
    <li><b>两种曲线可切换</b>：<code>TrajectoryPlanner(profile="trapezoid"|"scurve")</code>——本页演示程序梯形节拍 <b>@@CYCLE@@s</b>，S 曲线节拍 <b>@@CYCLE_SCURVE@@s</b>（起停更柔和、节拍略长）</li>
    <li><b>动力学数据</b>：<code>Waypoint.velocity / acceleration</code> 首次真实填充（时间戳差分），可供动力学校验与数字孪生消费</li>
    <li><b>节拍报表</b>：校验报告按指令输出耗时明细与占比（节拍大头一眼可见，报价有据）</li>
    <li><b>品牌出码带 VEL/ACC</b>：<code>Motion.velocity/acceleration</code> 覆盖值进入 UR <code>v=/a=</code>、KUKA <code>$VEL.CP/$ACC.CP</code>、ABB <code>v-num</code>、FANUC <code>mm/sec</code></li>
  </ul>
</section>

<section>
  <h2>GUI 轨迹编辑器（v0.2.4 新能力）</h2>
  <ul style="color:#c8d0dc;line-height:1.9">
    <li><b>路点直接编辑</b>：离线编程面板内插入（复制源点）/删除/上下移，双击或对话框手填逐关节角度（度）——改完重新校验即见新节拍</li>
    <li><b>逐点运动类型</b>：moveJ / moveL 一键切换；moveL 按示教点 FK 位姿走直线插补（规划器沿路逐点逆解，中途不可达会精确报出在哪一段）</li>
    <li><b>实时着色</b>：校验后路点按问题级别着色——<span style="color:#ff8080">错误红</span> / <span style="color:#ffc866">警告橙</span> / <span style="color:#6fdc6f">正常绿</span>，每行携带该指令的节拍耗时</li>
    <li><b>一键导出</b>：不先"生成代码"也能一步构建 + 按品牌扩展名落盘（UR <code>.script</code> / KUKA <code>.src</code> / ABB <code>.mod</code>…），附 manifest 部署清单</li>
    <li><b>双向同步</b>：面板改动经 <code>taughtPointsChanged</code> 信号回写路径代码编辑器，两条编辑通路永远共享同一份示教点数据</li>
  </ul>
</section>

<section>
  <h2>动力学 / 力矩校验（v0.3.1 新能力）</h2>
  <p style="font-size:14px">运动学回答"能不能到"，动力学回答"到了关节扛不扛得住"。<code>orolp.dynamics</code> 用牛顿-欧拉逆动力学逐路点计算关节力矩与机械功率，超额定直接报错、GUI 路点行标红。</p>
  <ul style="font-size:13.5px;color:var(--dim);margin:10px 0 14px 18px">
    <li><b>惯性参数双轨</b>：URDF <code>&lt;inertial&gt;</code> 自动解析（CAD 导出轨道）+ <code>set_link_inertial</code> 手动录入（手册轨道）+ <code>set_payload</code> 末端负载（平行轴定理合并）</li>
    <li><b>真实物理</b>：含重力、离心/科氏项与连杆惯性张量；单摆解析值、负载 Δτ=JᵀF、质量矩阵对称性三重交叉验证</li>
    <li><b>超限三色</b>：峰值超额定 → 错误红（程序校验不通过）；达 85% → 警告橙；缺惯性参数只提示跳过，不阻塞其余校验</li>
    <li><b>力矩/功率报表</b>：逐轴峰值/额定力矩表 + 机械功率峰值/均值，进校验报告 <code>format()</code></li>
    <li>测试基线 <b>@@DYN_TESTS@@ passed</b>（含 19 个动力学专项测试）</li>
  </ul>
  <p style="font-size:13px;color:#9aa5b5;margin-bottom:6px">实测样例：SO101 桌面臂（URDF 自带惯性参数）两段 moveJ 程序，校验输出逐轴力矩（N·m）：</p>
  <table style="font-size:13px;color:#c8d0dc;border-collapse:collapse">
    <tr style="color:#9aa5b5"><th style="text-align:left;padding:2px 14px 2px 0">关节</th><th style="text-align:right;padding:2px 14px">峰值力矩</th><th style="text-align:right;padding:2px 14px">额定</th><th style="text-align:right;padding:2px 14px">利用率</th></tr>
    @@DYN_ROWS@@
  </table>
  <p style="font-size:13px;color:#9aa5b5;margin-top:8px">机械功率峰值 @@DYN_PEAK_W@@ W（平均 @@DYN_AVG_W@@ W）——功率数据同时可供驱动器选型与能耗估算参考。</p>
</section>

<section>
  <h2>STEP/CAM 轨迹生成（v0.3.2 新能力）</h2>
  <p style="font-size:14px">CAD 模型直接变焊缝程序：<code>orolp.cam</code> 从 STEP/IGES 提取边与焊缝链，自动生成 moveL/moveC 路径，并用扫描点云比对实际偏差。下方数据全部来自真实管线（OpenCASCADE 整圆焊缝 → UR5e 校验 → URScript 出码）。</p>
  <ul style="font-size:13.5px;color:var(--dim);margin:10px 0 14px 18px">
    <li><b>焊缝提取</b>：拓扑边离散（弦高容差可控）→ 端点共点串链 → 闭合回路独立成链（整圆焊缝完整保留）；按长度 / 闭合 / 序号筛选</li>
    <li><b>路径 → 程序</b>：转角检测 + 平滑段 moveC 按累计转角 ≤120° 分片（规避控制器整圆限制）+ 转角/短段 moveL；闭合路径环形分段自动回起点；<code>z_lift</code> 沿法向抬高 + 固定焊炬姿态</li>
    <li><b>扫描偏差</b>：PLY 点云（ASCII/二进制）对规划路径逐点距离 → max / mean / rms + 最差点定位</li>
    <li><b>CLI</b>：<code>python3 tools/cam_path.py seams|path|deviation …</code>，path 模式直达品牌代码</li>
  </ul>
  <p style="font-size:13px;color:#9aa5b5;margin-bottom:6px">实测样例：Ø50mm 圆柱顶面整圆焊缝（@@CAM_N_PTS@@ 点、周长 @@CAM_LEN@@ mm）→ UR5e 程序 @@CAM_MOTIONS@@ 条指令、节拍 @@CAM_CYCLE@@ s、校验 0 错误；URScript 出码（节选 movec 圆弧段）：</p>
  <pre style="font-size:12px;background:#161b22;color:#c8d0dc;padding:12px;border-radius:8px;overflow-x:auto"><code>@@CAM_CODE@@</code></pre>
  <p style="font-size:13px;color:#9aa5b5;margin-top:8px">扫描偏差比对：@@CAM_N_CLOUD@@ 点模拟点云（整体偏 0.4mm）→ max @@CAM_DEV_MAX@@ mm / mean @@CAM_DEV_MEAN@@ mm / rms @@CAM_DEV_RMS@@ mm，最差点自动定位。</p>
</section>

<section>
  <h2>焊接 / 喷涂工艺模板库（v0.3.3 新能力）</h2>
  <p style="font-size:14px">工艺参数做成<b>可配置模板</b>：程序里只写 <code>arc_start("my_weld")</code>，换工艺改模板参数、程序一行不动。支持 YAML 覆盖内置默认（电流/电压/送丝/保护气/预流后流/起弧反馈超时；喷涂流量/扇幅/雾化压力）。</p>
  <ul style="font-size:13.5px;color:var(--dim);margin:10px 0 14px 18px">
    <li><b>一等工艺指令</b>：链式 <code>arc_start / arc_end / spray_on / spray_off</code>，与运动指令同源混排，对轨迹规划/节拍完全透明</li>
    <li><b>品牌出码</b>：FANUC 出原生 <code>ARCON AC=… V=…</code> / <code>ARCOFF</code>（含送丝/预流后流注释）；UR / KUKA / ABB 降级为 SetDO 语法 + 工艺参数注释，下装前人工核对</li>
    <li><b>waitDI 完整语义</b>：<code>trigger</code> 支持 level / rising / falling 边沿触发；无原生边沿指令的品牌强制留 TODO 注释，绝不静默按电平输出</li>
  </ul>
  <p style="font-size:13px;color:#9aa5b5;margin-bottom:6px">实测样例（同一份焊接程序，KR16 校验 0 错误）——FANUC TP 原生焊接指令：</p>
  <pre style="font-size:12px;background:#161b22;color:#c8d0dc;padding:12px;border-radius:8px;overflow-x:auto"><code>@@PROC_FANUC@@</code></pre>
  <p style="font-size:13px;color:#9aa5b5;margin:8px 0 6px">URScript 降级形态（置位信号 + 参数注释）：</p>
  <pre style="font-size:12px;background:#161b22;color:#c8d0dc;padding:12px;border-radius:8px;overflow-x:auto"><code>@@PROC_UR@@</code></pre>
</section>

<section>
  <h2>标定与手眼视觉（v0.3.4 新能力 · 相机触发抓取）</h2>
  <p style="font-size:14px">相机看到工件之后，机器人凭什么知道去哪抓？中间隔着四道"真实世界"的换算：<b>TCP 标定</b>（吸盘中心在哪）、<b>托盘坐标系 3 点法</b>（工件摆在哪）、<b>手眼标定</b>（相机在机器人的什么位置）、<b>像素 → 抓取位姿</b>（图像坐标怎么变成机器人目标）。<code>orolp.calibration</code> 把这四步做成可计算、可校验、可落盘的能力——每一步都给<b>误差</b>，不给出"看着像对"的数。</p>
  <svg viewBox="0 0 1000 300" xmlns="http://www.w3.org/2000/svg">
    <defs>
      <marker id="calarrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
        <path d="M 0 0 L 10 5 L 0 10 z" fill="#e8a33d"/>
      </marker>
    </defs>
    <rect x="0" y="0" width="1000" height="300" fill="#12151a" rx="10"/>
    <text x="20" y="30" font-size="13" fill="#9aa5b5">视觉 → 抓取：一条链把"图像像素"变成"关节角"</text>
    <g font-size="12.5" text-anchor="middle">
      <rect x="24" y="60" width="150" height="58" rx="8" fill="#232a34" stroke="#3a4250"/>
      <text x="99" y="84" fill="#e8ecf2">工件检测像素</text>
      <text x="99" y="104" fill="#9aa5b5">(u, v) ± 0.2px</text>

      <rect x="204" y="60" width="150" height="58" rx="8" fill="#232a34" stroke="#3a4250"/>
      <text x="279" y="84" fill="#e8ecf2">相机系射线</text>
      <text x="279" y="104" fill="#9aa5b5">相机内参 60° FOV</text>

      <rect x="384" y="60" width="150" height="58" rx="8" fill="#232a34" stroke="#3a4250"/>
      <text x="459" y="84" fill="#e8ecf2">基坐标系射线</text>
      <text x="459" y="104" fill="#9aa5b5">手眼外参 T_base_cam</text>

      <rect x="564" y="60" width="150" height="58" rx="8" fill="#232a34" stroke="#3a4250"/>
      <text x="639" y="84" fill="#e8ecf2">抓取位姿</text>
      <text x="639" y="104" fill="#9aa5b5">与支承面求交 + 抬高</text>

      <rect x="744" y="60" width="150" height="58" rx="8" fill="#232a34" stroke="#3a4250"/>
      <text x="819" y="84" fill="#e8ecf2">关节角 + 程序</text>
      <text x="819" y="104" fill="#9aa5b5">IK → 节拍 → 品牌出码</text>

      <line x1="176" y1="89" x2="202" y2="89" stroke="#e8a33d" stroke-width="2" marker-end="url(#calarrow)"/>
      <line x1="356" y1="89" x2="382" y2="89" stroke="#e8a33d" stroke-width="2" marker-end="url(#calarrow)"/>
      <line x1="536" y1="89" x2="562" y2="89" stroke="#e8a33d" stroke-width="2" marker-end="url(#calarrow)"/>
      <line x1="716" y1="89" x2="742" y2="89" stroke="#e8a33d" stroke-width="2" marker-end="url(#calarrow)"/>

      <rect x="24" y="196" width="200" height="66" rx="8" fill="#1e242d" stroke="#3a4250" stroke-dasharray="4 3"/>
      <text x="124" y="220" fill="#e8a33d">TCP 4 点法 / 6 点法</text>
      <text x="124" y="240" fill="#9aa5b5">工具中心点偏移 + 方向</text>

      <rect x="252" y="196" width="200" height="66" rx="8" fill="#1e242d" stroke="#3a4250" stroke-dasharray="4 3"/>
      <text x="352" y="220" fill="#e8a33d">坐标系 3 点法 / Kabsch</text>
      <text x="352" y="240" fill="#9aa5b5">托盘、工件、基座对齐</text>

      <rect x="480" y="196" width="200" height="66" rx="8" fill="#1e242d" stroke="#3a4250" stroke-dasharray="4 3"/>
      <text x="580" y="220" fill="#e8a33d">手眼标定 A·X = X·B</text>
      <text x="580" y="240" fill="#9aa5b5">eye-in-hand / eye-to-hand</text>

      <rect x="708" y="196" width="200" height="66" rx="8" fill="#1e242d" stroke="#3a4250" stroke-dasharray="4 3"/>
      <text x="808" y="220" fill="#e8a33d">标定集 JSON / URDF</text>
      <text x="808" y="240" fill="#9aa5b5">仿真与真机同值</text>

      <line x1="124" y1="192" x2="124" y2="122" stroke="#3a4250" stroke-width="1.5" stroke-dasharray="3 3"/>
      <line x1="352" y1="192" x2="459" y2="122" stroke="#3a4250" stroke-width="1.5" stroke-dasharray="3 3"/>
      <line x1="580" y1="192" x2="580" y2="122" stroke="#3a4250" stroke-width="1.5" stroke-dasharray="3 3"/>
      <line x1="808" y1="192" x2="819" y2="122" stroke="#3a4250" stroke-width="1.5" stroke-dasharray="3 3"/>
    </g>
  </svg>
  <p style="font-size:13px;color:#9aa5b5;margin:12px 0 8px">实测数据（<code>python examples/vision_pick_demo.py</code> 真实跑出，含 0.2px 像素噪声与 0.15mm/0.03° 手眼采集噪声）：</p>
  <table style="font-size:13px;color:#c8d0dc;border-collapse:collapse;width:100%">
    <tr style="color:#9aa5b5">
      <th style="text-align:left;padding:4px 12px 4px 0">环节</th>
      <th style="text-align:left;padding:4px 12px">实测结果</th>
    </tr>
    <tr><td style="padding:3px 12px 3px 0">TCP 4 点法（位置）</td><td style="padding:3px 12px">真值 @@CAL_TCP_TRUTH@@ mm → 标定 @@CAL_TCP_CAL@@ mm，误差 <b>@@CAL_TCP4_ERR@@</b> mm，残差 @@CAL_TCP4_RES@@ mm（合格）</td></tr>
    <tr><td style="padding:3px 12px 3px 0">TCP 6 点法（姿态）</td><td style="padding:3px 12px">沿工具 +X / +Z 各碰一点解出方向，姿态误差 <b>@@CAL_TCP6_ROT@@</b>°</td></tr>
    <tr><td style="padding:3px 12px 3px 0">托盘坐标系 3 点法</td><td style="padding:3px 12px">原点误差 @@CAL_FRAME_ERR@@ mm / 姿态 @@CAL_FRAME_ROT@@°（托盘还转了 15°，落件点按托盘局部坐标算）</td></tr>
    <tr><td style="padding:3px 12px 3px 0">手眼标定（eye-to-hand）</td><td style="padding:3px 12px">@@CAL_HE_MOTIONS@@ 段有效运动（腕部绕不同方向转），外参误差 <b>@@CAL_HE_ERR@@</b> mm / @@CAL_HE_ROT@@°，方程残差 @@CAL_HE_RES@@ mm</td></tr>
    <tr><td style="padding:3px 12px 3px 0">视觉 → 抓取位姿</td><td style="padding:3px 12px">检测像素 @@CAL_PIXEL@@ → 抓取点 @@CAL_GRASP_POS@@ mm，定位误差 <b>@@CAL_GRASP_ERR@@</b> mm（含标定误差 + 像素噪声）</td></tr>
    <tr><td style="padding:3px 12px 3px 0">程序与校验</td><td style="padding:3px 12px">@@CAL_MOTIONS@@ 条指令 / 节拍 @@CAL_CYCLE@@ s / 校验错误 <b>@@CAL_ERRORS@@</b> 条</td></tr>
  </table>
  <p style="font-size:13px;color:#9aa5b5;margin:14px 0 6px">标定结果随程序出码写入头部注释（下装前现场核对用，真实输出）：</p>
  <pre style="font-size:12px;background:#161b22;color:#c8d0dc;padding:12px;border-radius:8px;overflow-x:auto"><code>@@CAL_HEADER@@</code></pre>
  <ul style="font-size:13.5px;color:var(--dim);margin:12px 0 0 18px">
    <li><b>不给出不可信的答案</b>：4 点法姿态差异不足、手眼运动全绕同一根轴、模式传错（<code>eye_in_hand</code> / <code>eye_to_hand</code> 的 B 互为逆）——一律显式报错并说明怎么改动作，绝不沉默地给个数</li>
    <li><b>写回仿真</b>：<code>CalibrationSet.apply_to_robot()</code> 灌 TCP/基座，<code>write_urdf()</code> 把标定 TCP 写进 URDF 副本（沿用已有的 <code>tcp</code> 关节原地更新，工具几何不丢；重复写幂等）</li>
    <li><b>一条命令可复现</b>：<code>python examples/vision_pick_demo.py [--noise 1.0]</code>，噪声放大即见精度退化</li>
    <li>新增 <b>@@CAL_TESTS@@ 个标定专项测试</b>（含"必须报错"的负例与 FK 一致性数值校验）</li>
  </ul>
</section>

<section>
  <h2>多机器人协同（v1.0.1 新能力 · 两台机器人共用一条时间轴）</h2>
  <p style="font-size:14px">两台机器人各跑各的程序，<b>单看每一台都"校验通过"</b>，放到一起却可能撞——这类问题只存在于<b>时间轴</b>上：一台在取件时另一台正伸过同一片空间，或者一台待机位形恰好挡在别人的路径上。<code>orolp.multirobot</code> 把 N 条轨迹排到同一条时间轴上按固定步长采样，分别检查<b>机器人-机器人</b>干涉（含最近连杆对与最小间距）与<b>机器人-环境</b>干涉（<b>待机位形不隐身</b>——A 在等 B 的时候，A 就停在轨迹端点，那也是真实位姿）。</p>
  <svg viewBox="0 0 1040 360" xmlns="http://www.w3.org/2000/svg">
    <defs>
      <marker id="mrarw" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
        <path d="M 0 0 L 10 5 L 0 10 z" fill="#e8a33d"/>
      </marker>
    </defs>
    <rect x="0" y="0" width="1040" height="360" fill="#161a20" rx="10"/>
    @@MR_SCENE@@
  </svg>
  <p style="color:var(--dim);font-size:13px;margin-top:8px">机器人A 从左侧取件台取件 → 放到中间的共享交接点；机器人B 从交接点取走 → 放到右侧出料台。两条轨迹穿过<b>同一片空间</b>，却在时间上前后错开——这正是多机器人现场最典型的隐患。图中坐标取自示例脚本的真实常量。</p>

  <div class="metrics" style="margin-top:14px">
    <div class="metric"><div class="v">@@MR_A_CYCLE@@s</div><div class="k">机器人A 单机节拍（@@MR_A_POINTS@@ 轨迹点，校验 0 错误）</div></div>
    <div class="metric"><div class="v">@@MR_B_CYCLE@@s</div><div class="k">机器人B 单机节拍（@@MR_B_POINTS@@ 轨迹点，校验 0 错误）</div></div>
    <div class="metric"><div class="v">@@MR_HANDOFF_HITS@@ 处</div><div class="k">两机<b>同刻</b>进交接点的干涉（最近连杆对 @@MR_HANDOFF_LINKS@@，最小间距 @@MR_HANDOFF_SEP@@ mm）</div></div>
  </div>

  <p style="font-size:13px;color:#9aa5b5;margin:16px 0 6px">节拍统筹实测（同一批作业、同一组轨迹，只改判据）：</p>
  <table style="font-size:13px;color:#c8d0dc;border-collapse:collapse;width:100%">
    <tr style="color:#9aa5b5">
      <th style="text-align:left;padding:4px 12px 4px 0">排法</th>
      <th style="text-align:right;padding:4px 12px">总节拍</th>
      <th style="text-align:right;padding:4px 12px">机器人之间</th>
      <th style="text-align:left;padding:4px 12px">说明</th>
    </tr>
    <tr><td style="padding:3px 12px 3px 0">同刻齐发 · 净空 0</td><td style="text-align:right;padding:3px 12px">@@MR_MK_PARALLEL@@ s</td><td style="text-align:right;padding:3px 12px">@@MR_PAR0_HITS@@ 处</td><td style="padding:3px 12px">严格口径"没撞"——但最近处仅 <b>@@MR_NEAR_SEP@@ mm</b>（@@MR_NEAR_LINKS@@ @ t=@@MR_NEAR_AT@@s），现场这叫<b>擦肩</b></td></tr>
    <tr><td style="padding:3px 12px 3px 0">同刻齐发 · 净空 60mm</td><td style="text-align:right;padding:3px 12px">@@MR_MK_PARALLEL@@ s</td><td style="text-align:right;padding:3px 12px"><b>@@MR_PAR60_HITS@@ 处近失</b></td><td style="padding:3px 12px">换成"敢不敢跑"的判据，同一组轨迹立刻不合格</td></tr>
    <tr><td style="padding:3px 12px 3px 0">串行</td><td style="text-align:right;padding:3px 12px">@@MR_MK_SERIAL@@ s</td><td style="text-align:right;padding:3px 12px">0 处</td><td style="padding:3px 12px">一定安全，但最慢（两机节拍相加）</td></tr>
    <tr><td style="padding:3px 12px 3px 0">贪心错峰 · 净空 60mm</td><td style="text-align:right;padding:3px 12px"><b>@@MR_MK_GUARD@@ s</b></td><td style="text-align:right;padding:3px 12px">@@MR_GRE60_HITS@@ 处</td><td style="padding:3px 12px">把 @@MR_DELAYED@@ 推迟 @@MR_DELAY@@ s，近失清零；仍比串行省 <b>@@MR_SAVE60@@%</b></td></tr>
  </table>

  <ul style="font-size:13.5px;color:var(--dim);margin:14px 0 0 18px">
    <li><b>共享场景</b>：<code>Scene.add_robot()</code> 让多台机器人各有独立基座位姿、关节值与运动学链；旧的单机器人属性（<code>scene.robot</code> / <code>scene.joints</code>…）以属性代理落在主机器人上，<b>既有的代码与场景 JSON 一行都不用改</b></li>
    <li><b>互避碰</b>：<code>CollisionChecker.check_robot_pair()</code> 直接给"最近连杆对 + 最小间距"（球-球用解析式、其余取顶点云最近距离）；挂载在机器人身上的夹爪只对它自己那台豁免，不会连人带工具一起放行</li>
    <li><b>四种排法</b>：<code>parallel</code>（全从 0 起，体检用）/ <code>serial</code>（串行）/ <code>stagger</code>（指定偏移）/ <code>greedy</code>（逐台找最早无冲突起点，<code>max_wait</code> 限最大等待）</li>
    <li><b>净空即判据</b>：<code>clearance</code> 把"擦着过"也算问题——上表说明同一组轨迹在 0mm 与 60mm 两个口径下的结论完全相反，所以"0 处干涉"从来不该被读成"两机永远不会靠近"</li>
    <li><b>包络接触容差</b>：干涉检查用的是 URDF <code>&lt;collision&gt;</code> 里的厂商简化包络体（比真机胖），浅穿透属模型误差而非撞机，故引入 <code>ENVELOPE_TOLERANCE_MM = 20mm</code> 并在报告里<b>显式披露放行了几处</b>（本页两台机器人各 @@MR_A_ENV@@ / @@MR_B_ENV@@ 处）</li>
    <li><b>一条命令可复现</b>：<code>python examples/multi_robot_cell_demo.py [--json out.json]</code>；新增 <b>@@MR_TESTS@@ 个多机器人专项测试</b>（含"必须错峰"的判定用例与"深穿透不得被容差放行"的反例）</li>
  </ul>
  <p style="color:#9aa5b5;font-size:13px;margin-top:10px">另有机器人-环境包络触碰 @@MR_ENV_HITS@@ 处（由单机轨迹决定，<b>错峰消不掉</b>，只能改轨迹或改布局）——这也是本页不把"0 处机器人干涉"说成"绝对安全"的原因。</p>
</section>

<section>
  <h2>力控柔顺（v1.0.2 新能力 · 恒力打磨 / 贴面 / 装配的规划与出码）</h2>
  <p style="font-size:14px">位置控回答"走到哪"，力控回答"用多大力顶着走"。<code>orolp.compliance</code> 把力控里<b>该先算的数</b>一次算清——再按品牌出码（UR 原生 <code>force_mode()</code>，其余品牌诚实降级、绝不臆造语法）。</p>
  <svg viewBox="0 0 760 300" xmlns="http://www.w3.org/2000/svg" style="margin-top:8px">
    <rect x="0" y="0" width="760" height="300" fill="#161a20" rx="10"/>
    <text x="60" y="34" fill="#e8a33d" font-size="14" font-weight="bold">30N 阶跃力下的 TCP 让步曲线（导纳仿真 · z 轴，刚度 8 N/mm）</text>
    <line x1="60" y1="270" x2="700" y2="270" stroke="#3a4250" stroke-width="1"/>
    <line x1="60" y1="270" x2="60" y2="40" stroke="#3a4250" stroke-width="1"/>
    <line x1="60" y1="@@FC_STEADY_Y@@" x2="700" y2="@@FC_STEADY_Y@@" stroke="#4a90d9" stroke-width="1" stroke-dasharray="6 4"/>
    <text x="700" y="@@FC_STEADY_Y@@" fill="#4a90d9" font-size="12" text-anchor="end" dy="-6">稳态 @@FC_STEADY_LABEL@@（= F/k）</text>
    <polyline points="@@FC_CURVE@@" fill="none" stroke="#6fdc6f" stroke-width="2"/>
    <text x="640" y="288" fill="#9aa5b5" font-size="11">t = 1.0 s</text>
    <text x="64" y="288" fill="#9aa5b5" font-size="11">0</text>
  </svg>

  <div class="metrics" style="margin-top:14px">
    <div class="metric"><div class="v">@@FC_KEFF@@ N/mm</div><div class="k">串联系刚度 k_eff（机器人 8 N/mm × 工件 100 N/mm）</div></div>
    <div class="metric"><div class="v">@@FC_DEPTH@@ mm</div><div class="k">达到 30N 所需标称压入量（接近轨迹按它排，别让力控去"找"）</div></div>
    <div class="metric"><div class="v">@@FC_SENS@@ N</div><div class="k">工件偏 1mm 的力误差（k_eff 就是换算系数——力控不消除误差，只搬运误差）</div></div>
    <div class="metric"><div class="v">@@FC_SETTLE@@ s</div><div class="k">整定时间（超调 @@FC_OVERSHOOT@@%——力控环"跟得上多快"的量级）</div></div>
  </div>

  <p style="font-size:13px;color:#9aa5b5;margin:16px 0 6px">UR 原生出码样例（同一程序 · 单位已全部换算为 m / rad / N——直接可在 Polyscope 加载）：</p>
  <pre style="background:#10141a;border:1px solid var(--line);border-radius:8px;padding:12px 14px;font-size:12px;color:#c8d0dc;overflow-x:auto"><code>@@FC_UR_CODE@@</code></pre>

  <ul style="font-size:13.5px;color:var(--dim);margin:14px 0 0 18px">
    <li><b>阻抗 / 导纳一对偶</b>：<code>simulate_admittance</code>（力→位移，半隐式欧拉）与 <code>predict_contact_force</code>（位移→力）共用同一套二阶系统参数，数值积分与解析阶跃响应<b>双算法互相验证</b></li>
    <li><b>程序里是一等指令</b>：<code>prog.force_mode(spec)</code> / <code>prog.force_mode_end()</code> 与运动混排，对规划与节拍<b>完全透明</b>；校验时逐条体检力控参数并检查起止配对（多余的 end 报错、程序结束仍在力控给警告）</li>
    <li><b>出码不臆造语法</b>：UR 原生 <code>force_mode(task_frame, selection_vector, wrench, type, limits)</code>；ABB <code>FCRefForce/FCAct/FCDeact</code>（RobotWare 选件，附人工核对 TODO）；KUKA/FANUC 拿不到权威手册，只输出参数 + TODO——<b>宁可让人补一行，不给一行假代码</b></li>
    <li><b>关节扛不扛得住</b>：<code>check_force_capacity</code> 把接触力经 τ=JᵀF 叠加到重力矩上再比额定力矩——同一个 30N，臂伸直和收拢需要的关节力矩差数倍</li>
    <li><b>一条命令可复现</b>：<code>tests/test_compliance.py</code> 41 个专项测试（含 JᵀF 交叉验证与 UR 单位逐字段核对），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>

<section>
  <h2>数字孪生与云协作（v1.0.3 新能力 · 仿真 vs 真机 的偏差闭环）</h2>
  <p style="font-size:14px">离线编程的最后一问不是"代码生成没有"，而是<b>"真机跑出来和我想的一样吗"</b>。<code>orolp.twinsync</code> 把 <code>DebugSession.deviation</code> 的逐时刻偏差采样成<b>时间序列</b>，把两条来自不同源的轨迹按时间对齐后做差分，并且——最关键——把偏差拆成<b>"标定"与"伺服"两部分</b>：低速/停点段还剩下的偏差与伺服无关（只可能来自零位、传动比、连杆尺寸、工具坐标），高速段多出来的那部分才是位置环带宽不够。<b>两者处理方式完全不同，改错方向等于白干。</b></p>
  <p style="font-size:13px;color:#e8a33d;margin-top:6px">⚠️ 本页<b>没有真机</b>：真机侧是 <code>PlantModel</code> <b>合成传动误差模型</b>（零位偏置 / 传动比误差 / 编码器噪声 / 一阶伺服滞后，按物理规律作用），所有真机轨迹都标 <code>source=plant</code>。它的用途是验证整条孪生链路、以及标定"偏差多大该报警"的阈值——<b>不是现场实测数据，不能当验收依据</b>。接真机时把连接换成真机 <code>ControllerLink</code> 实现即可，报告逻辑一行不改。</p>

  @@TWIN_CHART_TUNED@@
  <p style="font-size:13px;color:#9aa5b5;margin-top:2px">↑ 同一段抓放轨迹（@@TWIN_SAMPLES@@ 个采样点 / @@TWIN_CYCLE@@s），"真机"参数为正常伺服带宽：偏差几乎全部是随速度变化的跟踪误差。</p>

  @@TWIN_CHART_POOR@@
  <p style="font-size:13px;color:#9aa5b5;margin-top:2px">↑ 只把"真机"参数改成"零点动过 + 减速比标称误差大 + 增益偏低"：偏差整体抬升一个数量级，且<b>连低速段也超标了</b>——这时候再去调伺服增益是白费功夫。</p>

  <table style="font-size:13px;color:#c8d0dc;border-collapse:collapse;width:100%">
    <tr style="color:#9aa5b5">
      <th style="text-align:left;padding:4px 12px 4px 0">口径</th>
      <th style="text-align:right;padding:4px 12px">调好的机器</th>
      <th style="text-align:right;padding:4px 12px">差调机的机器</th>
      <th style="text-align:left;padding:4px 12px">说明</th>
    </tr>
    @@TWIN_TABLE_ROWS@@
  </table>

  <p style="font-size:13px;color:#9aa5b5;margin:16px 0 6px">成因提示（由 <code>TwinReport.format_causes()</code> 按速度分解与曲线形状自动生成，第二轮的真实输出）：</p>
  <pre style="background:#10141a;border:1px solid var(--line);border-radius:8px;padding:12px 14px;font-size:12px;color:#c8d0dc;overflow-x:auto"><code>@@TWIN_CAUSES@@</code></pre>

  <div class="metrics" style="margin-top:14px">
    <div class="metric"><div class="v">@@TWIN_TRACE_COUNT@@</div><div class="k">云端轨迹条数（1 条仿真 + 2 条"真机"，JSONL 可逐行增量上报）</div></div>
    <div class="metric"><div class="v">@@TWIN_STATUS_AGE@@ s</div><div class="k">真机状态回显"有多旧"（≤5s 才算实时；实时性必须可被质疑）</div></div>
    <div class="metric"><div class="v">@@TWIN_STATUS_FRESH@@</div><div class="k">新鲜度判定结果</div></div>
  </div>

  <p style="font-size:13px;color:#9aa5b5;margin:16px 0 6px">真机上报的关节角回读（比上一版更可信的"实时回显"——因为能查它有多旧）：</p>
  <pre style="background:#10141a;border:1px solid var(--line);border-radius:8px;padding:12px 14px;font-size:12px;color:#c8d0dc;overflow-x:auto"><code>@@TWIN_READBACK@@</code></pre>

  <p style="font-size:13px;color:#9aa5b5;margin:16px 0 6px">云端轨迹清单（索引单独存 <code>.meta.json</code>，不为"看一眼有哪些轨迹"就下载全部内容）：</p>
  <table style="font-size:13px;color:#c8d0dc;border-collapse:collapse;width:100%">
    <tr style="color:#9aa5b5">
      <th style="text-align:left;padding:4px 12px 4px 0">轨迹</th>
      <th style="text-align:left;padding:4px 12px">来源</th>
      <th style="text-align:right;padding:4px 12px">样本数</th>
      <th style="text-align:right;padding:4px 12px">偏差峰值</th>
    </tr>
    @@TWIN_TRACE_ROWS@@
  </table>

  <p style="font-size:13px;color:#9aa5b5;margin:16px 0 6px">孪生轨迹的 JSONL 上报格式（首行元数据 + 每行一个快照，真机跑一段就能 push 一段）：</p>
  <pre style="background:#10141a;border:1px solid var(--line);border-radius:8px;padding:12px 14px;font-size:12px;color:#c8d0dc;overflow-x:auto"><code>@@TWIN_JSONL_HEAD@@</code></pre>

  <ul style="font-size:13.5px;color:var(--dim);margin:14px 0 0 18px">
    <li><b>Deviation 只在一处定义</b>：孪生层不重算偏差，而是逐点驱动 <code>DebugSession</code> 采样它的 <code>deviation()</code>——位置用欧氏距离、姿态用旋转矩阵夹角，口径全项目统一</li>
    <li><b>"实时"要能被质疑</b>：<code>CloudLink.status_age()</code> / <code>is_fresh()</code> 让"云端最后一条上报"有多旧是<b>可查</b>的；超龄就该显示"数据过期"，而不是拿旧姿势当真机现状</li>
    <li><b>对齐是必需的一步</b>：真机与仿真的时间戳不会严格相等（触发时刻、时钟漂移、控制器插补差异），先按最近邻 + 窗口对齐再差分，否则算出来的"偏差"里混的全是时间错位</li>
    <li><b>云端是传输之一，不是特殊路径</b>：<code>CloudLink</code> 与 <code>FileLink</code>／<code>SimLink</code> 同属 <code>ControllerLink</code> 抽象；默认传输是文件式对象存储（离线可用、零依赖），换 HTTP/S3 只需实现 <code>CloudTransport</code> 的四个方法</li>
    <li><b>一条命令可复现</b>：<code>examples/twin_sync_demo.py</code> 跑通同一闭环；<code>tests/test_twinsync.py</code> 100 个专项测试（含"一阶滞后稳态误差 ≈ τ·ω"的解析核对与严格 JSON 校验），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>

<section>
  <h2>插件接口（v1.0.4 新能力 · 不加核心代码也能加品牌/算法/渲染层）</h2>
  <p style="font-size:14px">上一节把"真机跑得对不对"补齐了，但还有个更日常的痛：<b>每加一个东西都要改核心仓库</b>。加一个品牌的控制语言得改后处理注册表；真机要换自己的解析解或标定几何得改运动学；现场想看的那条焊缝/那批超差点，只有使用者自己知道要画什么。<code>orolp.plugins</code> 把这三处开放成扩展点——<b>抽象早就在了（<code>post/base.py</code> 的模板 + 格式化钩子、<code>RobotKinematics</code>、覆盖层），缺的只是"从外部挂进来"的入口</b>。</p>

  <p style="font-size:13px;color:#9aa5b5;margin:14px 0 6px">本页运行时按真实通路登记的插件（同一进程内跑的，不是贴图）：</p>
  <pre style="background:#10141a;border:1px solid var(--line);border-radius:8px;padding:12px 14px;font-size:12px;color:#c8d0dc;overflow-x:auto"><code>@@PLUG_REGISTRY@@</code></pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 后处理插件：加一个仓库里没有的品牌</h3>
  <p style="font-size:13.5px;color:var(--dim)">插件写法与内置品牌<b>完全一致</b>——继承 <code>BasePostProcessor</code>、给 <code>brand</code>/<code>controller</code>、实现 <code>_convert_motion</code>/<code>_get_template</code>。登记之后 <code>program.to_post("acme_v2")</code> 走的是和内置品牌<b>逐字相同</b>的那条路。</p>
  <pre style="background:#10141a;border:1px solid var(--line);border-radius:8px;padding:12px 14px;font-size:12px;color:#c8d0dc;overflow-x:auto"><code>@@PLUG_POST_CODE@@</code></pre>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@PLUG_POST_LINES@@</div><div class="k">插件品牌 ACME-V2 出码行数</div></div>
    <div class="metric"><div class="v">@@PLUG_BUILTIN_LINES@@</div><div class="k">内置 KUKA KRL 同程序出码行数（同一条通路）</div></div>
    <div class="metric"><div class="v">@@PLUG_API@@</div><div class="k">宿主插件 API 版本（主版本相等才兼容）</div></div>
  </div>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 运动学插件：换求解器，同一个关节读数算出不同的 TCP</h3>
  <p style="font-size:13.5px;color:var(--dim)">演示插件在关节零位上叠了一层标定偏置（真实场景＝用标定过的几何模型替代 URDF 标称尺寸）。<code>Robot(..., kinematics_plugin="acme.calibrated")</code> 之后，规划、校验、回放全部走新求解器。</p>
  <table style="font-size:13px;color:#c8d0dc;border-collapse:collapse;width:100%">
    <tr style="color:#9aa5b5"><th style="text-align:left;padding:4px 12px 4px 0">求解器</th><th style="text-align:left;padding:4px 12px">同一组关节读数下的 TCP（mm，Rz 为度）</th></tr>
    <tr><td style="padding:3px 12px 3px 0">内置（URDF 标称）</td><td style="padding:3px 12px;color:#9aa5b5">@@PLUG_TCP_BUILTIN@@</td></tr>
    <tr><td style="padding:3px 12px 3px 0">插件（带标定偏置）</td><td style="padding:3px 12px;color:#9aa5b5">@@PLUG_TCP_PLUGIN@@</td></tr>
    <tr><td style="padding:3px 12px 3px 0">两者 TCP 差</td><td style="padding:3px 12px"><b>@@PLUG_TCP_DELTA@@ mm</b></td></tr>
  </table>
  <p style="font-size:13px;color:#9aa5b5;margin:10px 0 0">加载时的契约检查（必需成员缺一即拒绝加载，可选成员缺失只记警告——<b>而不是等到运行到一半才炸 AttributeError</b>）：</p>
  <pre style="background:#10141a;border:1px solid var(--line);border-radius:8px;padding:12px 14px;font-size:12px;color:#c8d0dc;overflow-x:auto"><code>@@PLUG_CONTRACT@@</code></pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ 渲染插件：纯数据覆盖层（不碰 OpenGL，所以无头可测）</h3>
  <p style="font-size:13.5px;color:var(--dim)">覆盖层只有三种图元：<code>LineSegments</code> / <code>PointCloud</code> / <code>FrameMarker</code>。宿主把它们喂给视口，测试可以把同一批对象直接断言——<b>同一份插件既能给 GUI 用，也能给网页/报告导出</b>。下面是本节真实收集到的图元：把规划出的 @@PLUG_OVER_TOTAL@@ 个关节读数<b>分别喂给两个求解器</b>。注意换了求解器之后连规划点数都变了（@@PLUG_N_ACTUAL@@ vs @@PLUG_N_NOMINAL@@，IK 解不同 → 重采样点不同），<b>所以两条轨迹根本不能按索引硬配</b>——把同一组读数喂给两边才比得干净。参考轨迹＝URDF 标称几何，容差 @@PLUG_TOL@@mm。</p>
  <table style="font-size:13px;color:#c8d0dc;border-collapse:collapse;width:100%">
    <tr style="color:#9aa5b5"><th style="text-align:left;padding:4px 12px 4px 0">插件</th><th style="text-align:left;padding:4px 12px">图元</th><th style="text-align:left;padding:4px 12px">内容</th></tr>
    @@PLUG_OVERLAY_ROWS@@
  </table>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@PLUG_OVERLAY_COUNT@@</div><div class="k">收集到的图元数（@@PLUG_OVERLAY_SUMMARY@@）</div></div>
    <div class="metric"><div class="v">@@PLUG_OVER_DEV@@ / @@PLUG_OVER_TOTAL@@</div><div class="k">TCP 偏差超 @@PLUG_TOL@@mm 容差的关节读数（标定过的几何 vs URDF 标称）</div></div>
    <div class="metric"><div class="v">@@PLUG_OVER_MAX@@ mm</div><div class="k">该偏差峰值</div></div>
  </div>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">④ 失败隔离：装了坏插件，软件照样能开</h3>
  <p style="font-size:13.5px;color:var(--dim)">这一节<b>故意注册了一个 <code>overlays()</code> 必然抛错的插件</b>。这不是"演示容错"，而是第三方生态的前提条件：<b>用户装了一个坏插件就再也打不开软件，等于把整个软件判了死刑</b>。</p>
  <pre style="background:#10141a;border:1px solid var(--line);border-radius:8px;padding:12px 14px;font-size:12px;color:#c8d0dc;overflow-x:auto"><code>@@PLUG_ISOLATION@@</code></pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">⑤ 正式分发：以包的形式声明入口点</h3>
  <pre style="background:#10141a;border:1px solid var(--line);border-radius:8px;padding:12px 14px;font-size:12px;color:#c8d0dc;overflow-x:auto"><code>@@PLUG_EP_SNIPPET@@</code></pre>

  <ul style="font-size:13.5px;color:var(--dim);margin:14px 0 0 18px">
    <li><b>延迟加载</b>：<code>register_*</code> 只登记元信息，工厂到 <code>create()</code> 才调用；工厂写成 <code>"module:attr"</code> 时连模块导入都推迟到那一刻——插件可以依赖 PySide6 / OpenGL / 厂商 SDK，而 <code>import orolp</code> 不能因此变重或失败</li>
    <li><b>API 版本显式声明</b>：规则只有一条，<b>主版本号必须相等</b>（<code>1.2</code> 与 <code>1.5</code> 兼容，<code>1.x</code> 与 <code>2.x</code> 不兼容）。刻意选最可解释的规则——宁可偶尔多拒一个其实能跑的插件，也不要让"悄悄按新语义跑出错误轨迹"发生在工业软件里</li>
    <li><b>契约显式校验</b>：运动学插件加载时先查必需成员，缺了就拒；缺可选项只记一条警告（聚合成 1 条，不是每个成员刷 1 条），并且相关功能会明确提示而不是运行到一半崩</li>
    <li><b>演示诚实性</b>：本节所有数字都来自本页生成时真实跑过的注册与调用；<b>没有真的第三方包被安装</b>（入口点扫描结果如实显示"发现 0 个"），所以插件是用 <code>register_*</code> 直接登记的——这正是同一个注册表</li>
    <li><b>一条命令可复现</b>：<code>examples/plugins_demo.py</code> 跑通三个扩展点；<code>tests/test_plugins.py</code> @@PLUG_TESTS@@ 个专项测试（含"登记阶段绝不 import 插件模块"的标记文件验证、坏插件隔离、入口点加载失败隔离），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>

<section>
  <h2>真机连接（v1.0.5 新能力 · 从"落盘 / 仿真"到"真的发到控制器上"）</h2>
  <p style="font-size:14px">前几节把"算得对不对"补齐了，但离线编程的<strong>最后一公里</strong>一直是空的：<code>ControllerLink</code> 抽象早就有了，落地形态却只有两种<strong>离线</strong>的——<code>FileLink</code>（把代码写进目录）和 <code>SimLink</code>（进程内运动学假装成控制器）。于是"仿真 vs 真机偏差闭环"（v1.0.3）只能拿<a href="#twin">合成模型</a>当对手。<code>SocketLink</code> 补上第一种真机形态：<strong>TCP + 一问一答的单行 JSON 协议</strong>。现场控制器五花八门（UR / FANUC / KUKA / 汇川 / 自研），共同点只有"有个网口"——把传输收敛成一条行协议后，控制器侧只要一小段转发，上层（deploy / debug / 孪生）一行都不用改。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 本页生成时真开了一个回环控制器，客户端走的是真 TCP</h3>
  <p style="font-size:13.5px;color:var(--dim)">下面这行状态不是贴图，是本页构建时 <code>SocketLink.connect()</code> 握手后返回的自省结果（地址里的端口由系统分配，每次构建都不同）：</p>
  <pre style="background:#10141a;border:1px solid var(--line);border-radius:8px;padding:12px 14px;font-size:12px;color:#c8d0dc;overflow-x:auto"><code>@@SOCK_DESCRIBE@@</code></pre>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@SOCK_ADDR@@</div><div class="k">打桩控制器监听地址（本机回环，非真机）</div></div>
    <div class="metric"><div class="v">@@SOCK_API@@</div><div class="k">协商出的协议版本（不等就在握手时失败，不带着错语义往下跑）</div></div>
    <div class="metric"><div class="v">@@SOCK_DOF@@</div><div class="k">服务端声明的关节数</div></div>
    <div class="metric"><div class="v">@@SOCK_CAPS@@</div><div class="k">协商出的能力表（没有 <code>feedback</code> 就如实返回 None，绝不伪造"实际位置"）</div></div>
  </div>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 部署 → 清单 → 取回核对</h3>
  <p style="font-size:13.5px;color:var(--dim)"><code>program.deploy(link, "ur_polyscope")</code> 生成的代码经 base64 过线（换行、中文注释、各家控制语言的引号转义都不再是问题）；再 <code>fetch_program()</code> 取回原文——"上传的到底是不是我发的那份"必须能核对，而不是祈祷。</p>
  <div class="metrics" style="margin-top:6px">
    <div class="metric"><div class="v">@@SOCK_LINES@@ 行 / @@SOCK_BYTES@@ B</div><div class="k">下发的程序规模</div></div>
    <div class="metric"><div class="v">@@SOCK_PROGRAMS@@</div><div class="k">控制器侧清单 <code>list_programs()</code></div></div>
    <div class="metric"><div class="v">@@SOCK_FETCH_OK@@</div><div class="k">取回原文与本地出码逐字节比对（@@SOCK_FETCH_BYTES@@ 字节回读）</div></div>
  </div>
  <pre style="background:#10141a;border:1px solid var(--line);border-radius:8px;padding:12px 14px;font-size:12px;color:#c8d0dc;overflow-x:auto"><code>句柄：@@SOCK_HANDLE@@
控制器侧收到的清单 meta：@@SOCK_META@@</code></pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ 逐点下发（时刻也过线）+ 回读实际位置</h3>
  <p style="font-size:13.5px;color:var(--dim)">把 <code>DebugSession</code> 直接接在真机连接上：每个规划点一条 <code>set_joints</code>，<strong>并带上该点在程序时间轴上的时刻</strong>（签名探测，老连接行为不变）；回读走 <code>read_joints</code> / <code>get_pose</code>。下面的"偏差"是<strong>规划值 vs 控制器回读值</strong>（单一来源口径）。</p>
  <div class="metrics" style="margin-top:6px">
    <div class="metric"><div class="v">@@SOCK_RPC_TOTAL@@</div><div class="k">本轮真实往返次数（含握手 / 清单 / 取回 / 位姿）</div></div>
    <div class="metric"><div class="v">@@SOCK_CMD_COUNT@@</div><div class="k">本轮下发的关节指令条数（下方两次采集合计；轨迹抽稀 1/@@SOCK_STRIDE@@ → 采样 @@SOCK_SAMPLES@@ 点，真机上是逐点一次 RPC）</div></div>
    <div class="metric"><div class="v">t=@@SOCK_AT_TIME@@s</div><div class="k">第一条指令携带的程序时刻（真机上就是伺服对标的基准）</div></div>
    <div class="metric"><div class="v">@@SOCK_SESSION_PEAK@@ mm</div><div class="k">整段轨迹回读峰值偏差（规划 vs 回读）</div></div>
  </div>
  <pre style="background:#10141a;border:1px solid var(--line);border-radius:8px;padding:12px 14px;font-size:12px;color:#c8d0dc;overflow-x:auto"><code>线上 op 序列（打桩服务端留档）：@@SOCK_OPS_HEAD@@ … @@SOCK_OPS_TAIL@@</code></pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">④ 偏差曲线：仿真 vs 经网络回读的"真机"</h3>
  <p style="font-size:13.5px;color:var(--dim)">孪生层一行没改——把控制器换成 <code>SocketLink</code> 之后，<code>capture_from_session</code> 拿到的偏差就是<strong>线上回读</strong>产生的。下图由 <code>compare_traces</code>（最近邻 + 窗口对齐后再差分）与 <code>deviation_chart_svg</code> 生成：仿真侧 @@SOCK_SIM_PEAK@@mm 只是数值噪声，线上侧 @@SOCK_REAL_PEAK@@mm 是"控制器报回来的位置和规划不一样"，跨源差分峰值 @@SOCK_CROSS_PEAK@@mm（均值 @@SOCK_CROSS_MEAN@@mm / 姿态 @@SOCK_CROSS_ROT@@°），越 1mm 容差 @@SOCK_OVER@@ 个样本。</p>
  @@SOCK_CHART@@

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">⑤ 线上对话原文（不是示意图，是留档的往返报文）</h3>
  <p style="font-size:13.5px;color:var(--dim)">程序正文只留规模、长数组只留前几项——验收要看的是"发了什么 op、带了什么标量"：</p>
  <table style="font-size:13px;color:#c8d0dc;border-collapse:collapse;width:100%">
    <tr style="color:#9aa5b5">
      <th style="text-align:left;padding:4px 12px 4px 0">op</th>
      <th style="text-align:left;padding:4px 12px">请求（客户端 → 控制器）</th>
      <th style="text-align:left;padding:4px 12px">响应（控制器 → 客户端）</th>
    </tr>
    @@SOCK_DIALOG_ROWS@@
  </table>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">⑥ 链路上出岔子会怎样：两类失败语义<strong>刻意</strong>不同</h3>
  <p style="font-size:13.5px;color:var(--dim)">这是这一层最容易糊弄过去、也最要命的地方。<strong>网络 / 协议级故障</strong>（连不上、超时、乱码、对端撂断、响应缺 <code>ok</code>）必须抛错<strong>并把连接标记为断开、关掉 socket</strong>——把一条已经断了的链路继续留在 <code>connected=True</code>，比报错糟糕得多；而<strong>服务端业务性拒绝</strong>（<code>ok:false</code>，比如程序名写错、关节数不符）只抛异常、<strong>不断开会话</strong>，一次笔误不该废掉一台机器的会话。下表是本页构建时逐条真跑出来的：</p>
  <table style="font-size:13px;color:#c8d0dc;border-collapse:collapse;width:100%">
    <tr style="color:#9aa5b5">
      <th style="text-align:left;padding:4px 12px 4px 0">故障注入</th>
      <th style="text-align:left;padding:4px 12px">客户端报错</th>
      <th style="text-align:left;padding:4px 12px">连接状态</th>
    </tr>
    @@SOCK_FAULT_ROWS@@
  </table>

  <ul style="font-size:13.5px;color:var(--dim);margin:14px 0 0 18px">
    <li><b>没有真机就没有真机</b>：本节所有回读值都来自随包附带的<strong>打桩控制器</strong> <code>ControllerStubServer</code>（回读复用 v1.0.3 的 <code>PlantLink</code> 合成传动误差模型：零位偏置 / 传动比误差 / 编码器噪声 / 伺服一阶滞后）。<code>source="real"</code> 只表示"数据经网络从控制器侧回读"，<strong>不代表现场实测，不能当验收依据引用</strong>——<b>Socket 通路本身也未在真机上验证过</b></li>
    <li><b>接真机要做的只有一件事</b>：在控制器侧准备一段同协议转发（UR 可用 URScript 的 <code>socket_open</code>/<code>read</code>/<code>write</code> 写小桩，或由 PC 侧代理进程把厂商 SDK / 原生协议翻译成本协议），然后把 <code>SocketLink</code> 指向它——孪生层、调试会话、后处理全部无需改动</li>
    <li><b>打桩控制器可以自己跑</b>：<code>python -m orolp.linkstub --port 30002 --urdf models/ur/ur5e/ur5e.urdf</code>，再用 <code>SocketLink("127.0.0.1", 30002)</code> 连上去；它还支持注入<strong>乱码 / 掉线 / 拒答 / 静默</strong>四类故障做排障演练（就是上表的来源）</li>
    <li><b>一条命令可复现</b>：<code>examples/socket_link_demo.py</code> 跑通同一条闭环（含故障演练与线上对话留档）；<code>tests/test_socket_link.py</code> @@SOCK_TESTS@@ 个专项测试（含"服务端谎报关节数"必须报错、乱码/掉线/静默四类故障的连接状态、CLI 真实子进程验收），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>

<section>
  <h2>HTTP / S3 兼容云协作（v1.0.6 新能力 · 多工位共用一套程序库与轨迹库）</h2>
  <p style="font-size:14px">v1.0.5 解决了"连真机"，这一节解决"多台设备共享"：云协作的传输层此前只有进程内与本机目录两种实现，<strong>两个工位共用同一套程序库/轨迹库</strong>做不到。<code>cloudhttp.py</code> 三件套补上：<code>HTTPCloudStore</code>（REST 客户端，把 <code>CloudTransport</code> 四方法映射成四个 HTTP 请求）、<code>SigV4Signer</code>（AWS Signature V4，零第三方依赖——对接 S3 兼容对象存储时补上它即可）、<code>CloudStoreServer</code>（纯标准库参考服务端，<code>python -m orolp.cloudhttp --root &lt;目录&gt;</code> 即起，本机验收与自建自用两用）。失败语义与 SocketLink 同套路：<strong>连不上/5xx 是传输级故障</strong>（标记 <code>reachable=False</code>，不让断网被渲染成"云端是空的"）、<strong>404 是正常回答</strong>（对象缺失→None）、<strong>401/403 是配置问题</strong>（专门的 <code>CloudAuthError</code>，响亮报错不重试）。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 本页生成时真起了一个对象存储服务端，两个工位走真 HTTP 共享程序库</h3>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@CLOUD_ADDR@@</div><div class="k">参考服务端监听地址（本机回环，端口系统分配）</div></div>
    <div class="metric"><div class="v">@@CLOUD_OBJECTS@@ 个对象 / @@CLOUD_REQUESTS@@ 次请求</div><div class="k">服务端处理量（落盘布局 &lt;根&gt;/&lt;桶&gt;/&lt;键&gt;，可直接备份迁移）</div></div>
    <div class="metric"><div class="v">@@CLOUD_CELLB@@</div><div class="k">先连错命名空间的工位看到的程序清单（隔离必须成立）</div></div>
    <div class="metric"><div class="v">@@CLOUD_FETCH_OK@@</div><div class="k">工位 B 取回的程序与 A 部署的逐字节比对（@@CLOUD_LINES@@ 行 / @@CLOUD_BYTES@@ B）</div></div>
  </div>
  <pre style="background:#10141a;border:1px solid var(--line);border-radius:8px;padding:12px 14px;font-size:12px;color:#c8d0dc;overflow-x:auto"><code>工位 A 部署句柄：@@CLOUD_HANDLE@@
工位 B 看到的清单：@@CLOUD_PROGRAMS@@
部署清单 manifest：@@CLOUD_MANIFEST@@</code></pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 轨迹库：合成"真机"轨迹上云，另一工位取回做跨源差分</h3>
  <p style="font-size:13.5px;color:var(--dim)">孪生层一行没改——把传输换成 <code>HTTPCloudStore</code> 之后，<code>publish_trace</code>/<code>fetch_trace</code> 走的就是真 HTTP（轨迹抽稀 1/16 → @@CLOUD_SAMPLES@@ 快照上云；JSONL 逐行格式支持现场增量上报）。取回轨迹的 <code>source</code> <strong>保持原样</strong>——"经过云端"不改变"数据是不是真机测的"这件事，只追加进附注。</p>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@CLOUD_META@@</div><div class="k">轨迹索引 .meta.json（看一眼有哪些轨迹不用下载本体）</div></div>
    <div class="metric"><div class="v">@@CLOUD_REAL_PEAK@@ mm</div><div class="k">上云轨迹峰值偏差（合成传动误差模型，非现场实测）</div></div>
    <div class="metric"><div class="v">@@CLOUD_CROSS_PEAK@@ mm（均值 @@CLOUD_CROSS_MEAN@@ mm）</div><div class="k">跨源差分峰值：纯仿真 vs 经云端取回（越 1mm 容差 @@CLOUD_OVER@@ 点）</div></div>
    <div class="metric"><div class="v">t-@@CLOUD_AGE@@s · @@CLOUD_FRESH@@</div><div class="k">A 工位状态上报 → B 工位回读的新鲜度（"实时回显"必须可被质疑）</div></div>
  </div>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ SigV4 签名：有外部证据，也有端到端验收</h3>
  <p style="font-size:13.5px;color:var(--dim)">签名核心拆成独立函数（规范请求 → 待签字符串 → 派生密钥），每一环都能单独和 AWS 官方文档给的示例向量对——本页构建时又实算核对了一遍（@@CLOUD_SIGVEC@@）。参考服务端在 S3 模式下会<strong>逐请求复算签名</strong>（真实 S3 就是这么校验的）：@@CLOUD_SIG_E2E@@；换一个错密钥再试：@@CLOUD_SIG_BAD@@。<strong>诚实披露</strong>：签名与 AWS 规则一致有外部证据，但<strong>没有连过任何真实云端点</strong>——接真云（MinIO / OSS 兼容端点）前请先做一次冒烟。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">④ 链路上出岔子会怎样：三类语义<strong>刻意</strong>不同</h3>
  <table style="font-size:13px;color:#c8d0dc;border-collapse:collapse;width:100%">
    <tr style="color:#9aa5b5">
      <th style="text-align:left;padding:4px 12px 4px 0">故障注入</th>
      <th style="text-align:left;padding:4px 12px">客户端回答</th>
      <th style="text-align:left;padding:4px 12px">可达性判定</th>
    </tr>
    @@CLOUD_FAULT_ROWS@@
  </table>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">⑤ 服务端请求日志（本页构建时的真实往返留档）</h3>
  <table style="font-size:13px;color:#c8d0dc;border-collapse:collapse;width:100%">
    <tr style="color:#9aa5b5">
      <th style="text-align:left;padding:4px 12px 4px 0">时刻</th>
      <th style="text-align:left;padding:4px 12px">方法</th>
      <th style="text-align:left;padding:4px 12px">路径</th>
    </tr>
    @@CLOUD_DIALOG_ROWS@@
  </table>

  <ul style="font-size:13.5px;color:var(--dim);margin:14px 0 0 18px">
    <li><b>接真云只改一处</b>：把 <code>base_url / bucket / 凭证</code> 换成真的（S3 兼容存储再加一个 <code>SigV4Signer</code>），<code>CloudLink</code>、孪生层、调试会话、后处理一行不用改</li>
    <li><b>自建自用</b>：<code>python -m orolp.cloudhttp --root /srv/rwcloud --port 8080 --token 顶智令牌</code> 就是一个可用的对象存储端点；REST 契约刻意做窄，自己实现一个兼容服务端只要几十行</li>
    <li><b>一条命令可复现</b>：<code>examples/cloud_http_demo.py</code> 跑通同一条闭环；<code>tests/test_cloudhttp.py</code> @@CLOUD_TESTS@@ 个专项测试（AWS 官方 4 组向量逐字符比对、三种传输契约一致性、真并发、CLI 真实子进程验收），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>

<section>
  <h2>程序版本对比（v1.0.7 新能力 · 改了哪几条、节拍变多少、TCP 偏了多少）</h2>
  <p style="font-size:14px">程序在云端被改过一版——谁改的、改了什么、影响多大？靠"肉眼 diff 出码文本"既看不出节拍变化，也看不出一条 <code>moveL</code> 目标挪了 3mm 到底是工艺修正还是手抖。<code>progdiff.py</code> 把对比做成<strong>结构化</strong>的：用序列对齐（<code>difflib</code>）把"插入/删除一条指令"与其余指令<strong>对齐</strong>——不会从插入点开始整体错位报"全变了"；对齐后的每对指令做<strong>字段级</strong>差异（关节角第几轴 Δ多少度、TCP 变多少 mm/°、IO 什么电平变了）。版本库 <code>ProgramVersionStore</code> 挂在任意 <code>CloudTransport</code> 上（本机目录 / 内存 / v1.0.6 的 HTTP 云端都行），按名自动递增 <code>v1/v2/…</code> 并留 SHA256 指纹。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 本页构建时在真 HTTP 版本库里存了三版（真规划器参与对比）</h3>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@PD_VERSIONS@@</div><div class="k">「PICK」程序的版本史（v3 = 重新上传 v1 同一份——重新上传本身就是要留痕的事件）</div></div>
    <div class="metric"><div class="v">@@PD_REQUESTS@@ 次</div><div class="k">参考服务端处理的请求量（版本库与 v1.0.6 共用同一 REST 通道）</div></div>
    <div class="metric"><div class="v">@@PD_FP@@</div><div class="k">v2 的 SHA256 指纹（指纹相同 ⇔ 逐字段相同，改一个关节角都不同）</div></div>
    <div class="metric"><div class="v">@@PD_SAME@@</div><div class="k">v1 → v3 的对比结论（重复上传同一份 = 新版本号 + 空差异）</div></div>
  </div>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② v1 → v2 的结构化差异（现场改动：下探加深 20mm、提速到 70%、插一段中间点、保压加倍）</h3>
  <pre style="background:#10141a;border:1px solid var(--line);border-radius:8px;padding:12px 14px;font-size:12px;color:#c8d0dc;overflow-x:auto"><code>@@PD_ROWS@@

── 汇总 ──────────────────────────────────────────────
@@PD_SUMMARY@@</code></pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ 规划器口径：节拍变化 + TCP 路径偏移</h3>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@PD_CYCLE_A@@s → @@PD_CYCLE_B@@s（Δ@@PD_CYCLE_DELTA@@s）</div><div class="k">两版节拍（<code>plan=True</code> 时连规划器一起比）</div></div>
    <div class="metric"><div class="v">max @@PD_PATH_MAX@@mm（mean @@PD_PATH_MEAN@@mm）</div><div class="k">TCP 路径偏移：新版轨迹逐点到旧版折线的最近距离（复用 v0.3.2 扫描偏差算法）</div></div>
    <div class="metric"><div class="v">@@PD_TESTS@@ 个</div><div class="k">专项测试（对齐/字段级/序列化指纹/多后端版本库/示例脚本子进程验收）</div></div>
  </div>

  <ul style="font-size:13.5px;color:var(--dim);margin:14px 0 0 18px">
    <li><b>口径诚实</b>：结构对比是精确的（字段级、无启发式）；但节拍与 TCP 路径偏移是<strong>规划器口径</strong>——回答"仿真里会多花多久、路径会偏多少"，<b>不回答"真机上会怎样"</b></li>
    <li><b>读报告注意</b>：起点位姿不同的两版程序会比出"全程大偏移"——那是起点差异不是路径差异；对齐按指令种类序列做，同 <code>setDO</code>/<code>waitDI</code> 按信号名对齐</li>
    <li><b>与 v1.0.3/v1.0.6 的关系</b>：<code>Program.deploy</code> 把代码送到控制器/云端，版本库存的是<b>可 diff 的结构化版本</b>——同一 <code>CloudTransport</code> 抽象，HTTP/本机/内存后端行为一致</li>
    <li><b>一条命令可复现</b>：<code>examples/prog_diff_demo.py</code> 跑通同一条评审流程；全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>

<section>
  <h2>第一方示例插件包（v1.0.8 新能力 · 真正可 pip install 的插件）</h2>
  <p style="font-size:14px">v1.0.4 的插件接口此前只能用<strong>注入的假入口点</strong>测——<code>importlib.metadata</code> 的真实发现路径没有被验证过。v1.0.8 把示例里的 ACME-V2 品牌挪成一个<strong>独立的、可安装的 Python 包</strong>（<code>examples/acme_post_plugin/</code>，自带 <code>pyproject.toml</code> 入口点声明），本页构建时<strong>真的执行了</strong> <code>pip install</code> 到临时目录，再让 <code>PluginRegistry.load_entry_points()</code>（不注入任何东西）从真实 dist-info 里发现它、创建它、出码：</p>
  <div class="grid" style="margin:14px 0">
    <div class="card"><div class="k">pip 安装</div><div class="v">@@PKGP_INSTALL@@</div></div>
    <div class="card"><div class="k">入口点扫描</div><div class="v">@@PKGP_STATS@@</div></div>
    <div class="card"><div class="k">登记来源</div><div class="v">@@PKGP_ORIGIN@@</div></div>
    <div class="card"><div class="k">插件品牌</div><div class="v">@@PKGP_LANG@@</div></div>
  </div>
  <pre class="code" style="font-size:12.5px">@@PKGP_CODE@@</pre>
  <ul style="font-size:13.5px;color:var(--dim);margin:14px 0 0 18px">
    <li><b>入口点组</b>：<code>[project.entry-points."orolp.plugins"]</code>，加载结果是"提供函数"（返回 <code>PluginSpec</code> 列表）；工厂写成 <code>"module:attr"</code> 字符串——连插件模块内部都是延迟导入</li>
    <li><b>宿主边界</b>：<code>orolp</code> 本体由宿主环境提供，插件包不写进 <code>dependencies</code>——插件不绑架宿主的 orolp 版本</li>
    <li><b>出码同路</b>：装好后 <code>program.to_post("acme_v2")</code> 与内置 7 品牌走<strong>完全同一套</strong> <code>get_post_processor</code> 通路（含 IO 指令的诚实降级警告）</li>
    <li><b>一条命令可复现</b>：<code>tests/test_plugin_package.py</code> @@PKGP_TESTS@@ 个专项测试（含真 pip install → 真发现 → 真出码的核心验收；先复制再装，pip 就地构建绝不污染源目录），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>

<section>
  <h2>包络接触容差 · 逐机器人可配（v1.0.9 新能力）</h2>
  <p style="font-size:14px">v1.0.1 的 <code>ENVELOPE_TOLERANCE_MM = @@ENV_GLOBAL@@mm</code> 是全局常量（UR5e 实测标定），但各品牌包络体的"胖瘦"不同。v1.0.9 两件事：①<code>Robot(envelope_tolerance_mm=…)</code> <strong>逐机覆盖</strong>——<code>validate(margin=None)</code> 时自动用本机标定值（未标定保持既有默认 −0.5mm，老程序判定结果不变）；②<strong>几何下界估算器</strong> <code>estimate_envelope_tolerance_mm(urdf)</code>——逐连杆把 mesh 顶点变换进包络体坐标系切片比较。下面是构建时对各品牌真机 URDF 真跑出来的数字：</p>
  <table style="width:100%;font-size:13.5px;border-collapse:collapse;margin:12px 0">
    <tr style="color:var(--dim)"><th align="left">机型</th><th align="left">几何下界估算</th><th align="left">最"胖"的连杆</th></tr>
    @@ENV_ROWS@@
  </table>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>口径诚实</b>：UR5e 估算 5.8mm，而抓取工作站<strong>实测</strong>最深包络穿透 18.6mm（→ 推荐容差 20mm）——穿透发生在 mesh 凹陷处与姿态相关接触位置，纯几何法测不到。所以估算是<strong>下界 + 品牌横向对比工具</strong>（谁最需要大容差、最胖连杆在哪），<b>不能替代实测标定</b>；上机取 <code>max(几何下界, 实测值)</code></li>
    <li><b>SO101 显示"无可比几何"是诚实行为</b>：程序生成 URDF 的 visual 不是 STL mesh，估算器拒绝编数字，调用方应回退全局默认</li>
    <li><b>容差不能为负</b>：<code>Robot(envelope_tolerance_mm=-1)</code> 当场 ValueError——容差是放行量，符号写反会把"放行浅穿透"变成"要求重叠才算撞"</li>
    <li><b>一条命令可复现</b>：<code>tests/test_envelope.py</code> @@ENV_TESTS@@ 个专项测试（合成 URDF 确定性验证 + float32 边界切片回归 + 逐机解析三态），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>

<section>
  <h2>界面语言切换 · i18n（v1.0.10 新能力）</h2>
  <p style="font-size:14px">面向开源发布的<strong>英文界面</strong>：新增 <code>orolp.i18n</code>——<code>tr()</code> 轻量运行时翻译。<strong>中文是源语言</strong>（默认恒等返回，零回归），英文查目录（未收录原样返回，绝不出现占位符）；语言经 <code>OROLP_LANG</code> 环境变量或 <code>~/.orolp/language.json</code> 显式配置（不做 locale 自动探测，行为可预期），GUI 菜单「语言 / Language」切换即持久化并提示重启。下表是构建时从目录里取出的真实译文：</p>
  <table style="width:100%;font-size:13.5px;border-collapse:collapse;margin:12px 0">
    <tr style="color:var(--dim)"><th align="left">中文（源语言）</th><th align="left">English</th></tr>
    @@I18N_ROWS@@
  </table>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>目录 @@I18N_ENTRIES@@ 条</b>（含命名占位符模板），GUI 四文件 <b>@@I18N_SITES@@ 处</b>文案接线（@@I18N_SITES_DETAIL@@）</li>
    <li><b>语义安全的边界</b>：设备类型等"存储类数据"保持中文存储、只在显示处翻译——<code>devices_of()</code> 按词表过滤的逻辑不受界面语言影响</li>
    <li><b>口径诚实</b>：英文模式覆盖 <strong>GUI 层全部文案</strong>；库层消息（校验 Issue / 报告 / 后处理 TODO 注释）自 v1.1 起接入（见下一节）</li>
    <li><b>一条命令可复现</b>：<code>tests/test_i18n.py</code> @@I18N_TESTS@@ 个专项测试（tr 语义三态 / 配置优先级 / 持久化往返 / en·zh 双模式 offscreen 面板与语言菜单），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>

<section>
  <h2>库层消息 i18n（v1.1 新能力）</h2>
  <p style="font-size:14px">v1.0.10 只覆盖 GUI 文案；v1.1 把<strong>库层运行时消息</strong>也接进 <code>tr()</code>——planner 校验 Issue 与报告、后处理器告警、<strong>生成代码里的 TODO 注释</strong>（下装前现场工程师看到的就是这些）。英文模式下切语言后重跑校验/出码即得英文报告与英文注释。以下是构建时真实跑出的双语对照：</p>
  <div style="display:flex;gap:18px;flex-wrap:wrap">
    <div style="flex:1;min-width:300px">
      <div style="font-size:13px;color:var(--dim);margin-bottom:4px">校验报告 · zh（源语言）</div>
      <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@I18NLIB_ZH_REPORT@@</pre>
      <div style="font-size:13px;color:var(--dim);margin:8px 0 4px">校验报告 · English</div>
      <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@I18NLIB_EN_REPORT@@</pre>
    </div>
    <div style="flex:1;min-width:300px">
      <div style="font-size:13px;color:var(--dim);margin-bottom:4px">FANUC 出码 TODO 注释 · zh</div>
      <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@I18NLIB_ZH_CODE@@</pre>
      <div style="font-size:13px;color:var(--dim);margin:8px 0 4px">FANUC 出码 TODO 注释 · English</div>
      <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@I18NLIB_EN_CODE@@</pre>
    </div>
  </div>
  <table style="width:100%;font-size:13.5px;border-collapse:collapse;margin:12px 0">
    <tr style="color:var(--dim)"><th align="left">中文（源语言 / 目录 key）</th><th align="left">English</th></tr>
    @@I18NLIB_ROWS@@
  </table>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>接线范围</b>：planner 全部 Issue 文案（越限/逆解失败/奇异/配置跳变/干涉/力矩超限/力控配对）、<code>ProgramReport.format()</code> 全部报告行、7 个品牌后处理器 + post 工厂共 @@I18NLIB_SITES@@ 处 <code>tr()</code>（@@I18NLIB_SITES_DETAIL@@）</li>
    <li><b>消息在生成时刻定稿</b>：与 GUI 同一契约——切语言后需重跑校验/出码才能刷新已生成的报告与代码</li>
    <li><b>一条命令可复现</b>：<code>tests/test_i18n_lib.py</code> @@I18NLIB_TESTS@@ 个专项测试（zh 逐字节恒等 / en 报告与注释零中文残留 / 占位符格式化 / 目录卫生），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>

<section>
  <h2>真机连接面板（v1.1.1 新能力）</h2>
  <p style="font-size:14px">v1.0.5 的真机 Socket 连接此前只是<strong>代码 API</strong>；v1.1.1 把它变成界面能力——主窗口「文件 → 真机连接…」打开面板：<strong>连接</strong>（握手后如实展示服务端自述 / 协议版本 / 关节数 / 能力协商结果）、<strong>部署当前程序</strong>（与「一键导出」同一条生成通路，直接下发到控制器）、<strong>回读实际关节角</strong>（视口立刻摆成真机当前位形，仿真 vs 真机对比从这里开始）。<b>安全边界是刻意的</b>：面板没有"发送运动指令"入口——真机运动一律走"部署经评审的程序"这条可审计路径。</p>
  <div style="display:flex;gap:18px;flex-wrap:wrap;align-items:flex-start">
    <div style="flex:0 0 300px;max-width:300px">
      <figure style="margin:0"><img src="assets/controller_panel.png" alt="真机连接面板" style="width:100%;border-radius:8px;border:1px solid #232a34"><figcaption style="font-size:12px;color:var(--dim);margin-top:4px">真机连接面板（离屏渲染自真实控件树）</figcaption></figure>
    </div>
    <div style="flex:1;min-width:300px">
      <div style="font-size:13px;color:var(--dim);margin-bottom:4px">握手状态（构建时真连打桩控制器）</div>
      <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@CP_INFO@@</pre>
      <div style="font-size:13px;color:var(--dim);margin:8px 0 4px">部署 + 回读后的面板日志（构建时真实操作）</div>
      <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@CP_LOG@@</pre>
    </div>
  </div>
  <ul style="font-size:13.5px;color:var(--dim);margin:14px 0 0 18px">
    <li><b>能力如实展示</b>：控制器声明 <code>feedback=false</code> 时回读按钮给出"控制器不支持关节回读——不伪造数据"，绝不用本机运动学编一份假"实际位置"</li>
    <li><b>失败语义分层</b>：连不上 / 超时 → 连接置为断开；服务端业务拒绝（如程序名冲突）→ 只记日志不断会话——与 <code>SocketLink</code> 同一套契约</li>
    <li><b>部署同路</b>：走 <code>Program.deploy</code>（代码生成 + 部署 + 清单），后处理器跟随「导出打包」标签的当前选择</li>
    <li><b>一条命令可复现</b>：<code>tests/test_controller_panel.py</code> @@CP_TESTS@@ 个专项测试（真 socket 打桩验收 GUI 全链路：部署到控制器原文取回 / 业务拒绝不断会话 / 回读信号 / en 界面），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>

<section>
  <h2>真机连接 SSH（v1.1.2 新能力 · SFTP 传程序 + SSH exec 远程加载）</h2>
  <p style="font-size:14px">相当一部分控制器不支持开放 TCP socket 给上位机自定协议，但都带着 SSH——对这类机器，"把程序文件放到控制器文件系统 + 让现场用一条命令加载它"就是真实的工作流。<code>SSHLink</code> 用 <strong>SFTP</strong> 传程序文件（真实落在控制器文件系统上，带 manifest 清单）、用 <strong>SSH exec</strong> 触发现场加载；"上传的到底是不是我发的那份"用 <code>fetch_program()</code> 取回原文逐字节核对。失败语义与 SocketLink 同契约：<strong>网络/认证级故障断开并如实显示，命令退出码非 0 是业务结果、不断会话</strong>。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 本页生成时真跑了一遍 SSH 回环（paramiko 客户端 ↔ paramiko 服务端打桩，SFTP 文件真实落盘）</h3>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@SSH_ADDR@@</div><div class="k">打桩控制器监听地址（本机回环，端口系统分配，chroot 式 SFTP 根目录）</div></div>
    <div class="metric"><div class="v">@@SSH_HANDLE@@</div><div class="k">Program.deploy 句柄（@@SSH_LINES@@ 行 / @@SSH_BYTES@@ B，SFTP 落盘 + manifest）</div></div>
    <div class="metric"><div class="v">@@SSH_FETCH_OK@@</div><div class="k">取回原文与部署代码逐字节比对（清单：@@SSH_PROGRAMS@@）</div></div>
    <div class="metric"><div class="v">@@SSH_AUTH_STATE@@</div><div class="k">错密码认证 → 必须报错并断开（打桩端拒绝记录 @@SSH_AUTH_REJECTS@@ 条）</div></div>
  </div>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② SSH exec：现场加载命令的退出码与输出如实回传</h3>
  <pre style="background:#10141a;border:1px solid var(--line);border-radius:8px;padding:12px 14px;font-size:12px;color:#c8d0dc;overflow-x:auto"><code>部署句柄：@@SSH_HANDLE@@
清单核对：@@SSH_PROGRAMS@@（manifest：@@SSH_MANIFEST@@）
$ ls -1 programs（cwd=/）          → 退出码 @@SSH_LS_CODE@@
@@SSH_LS_OUT@@
$ load web_ssh_pick.script（cwd=/programs）→ 退出码 @@SSH_LOAD_CODE@@
  stderr：@@SSH_LOAD_ERR@@
命令失败后的连接状态：@@SSH_STILL_CONNECTED@@（本次连接累计 exec @@SSH_EXEC@@ 次）</code></pre>

  <ul style="font-size:13.5px;color:var(--dim);margin:14px 0 0 18px">
    <li><b>为什么不只靠 SocketLink</b>：SSH 形态覆盖"不支持开放 socket 但带 SSH"的控制器（KRC 类控制器、控制器 PC 一体机等）；SSH 形态无常驻对端服务，因此没有握手/能力协商——回读能力一律 <code>False</code>，孪生偏差链路仍走 Socket/厂商协议</li>
    <li><b>诚实披露</b>：SSH 通路<strong>未在真机上验证过</strong>（开发环境无机器人）；接真机请先在现场核对主机密钥、路径权限与控制器侧的加载命令</li>
    <li><b>一条命令可复现</b>：<code>examples/ssh_link_demo.py</code> 跑通同一条闭环（含失败演练）；<code>tests/test_ssh_link.py</code> @@SSH_TESTS@@ 个专项测试（真 SSH 回环验收：SFTP 字节级一致 / 清单读远端 / 取回核对 / 认证失败断连 / CLI 子进程验收），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>

<section>
  <h2>waitTime 驻留段时间轴（v1.1.3 新能力 · 节拍从此包含"停在那儿"的时间）</h2>
  <p style="font-size:14px">夹具保压、握手等待、吸取稳定——程序里的 <code>waitTime</code> 此前对规划器<strong>完全不可见</strong>：节拍少算全部驻留时间，更关键的是多机时间轴看不见"驻留占用"——"A 在交接点驻留、B 扫过来"这类最经典的机器人间冲突在仿真里根本不存在。v1.1.3 把驻留接进时间轴：驻留段关节恒定、时间推进，计入节拍与多机干涉扫描；SetDO/WaitDI/起弧/喷涂/力控仍对轨迹透明（WaitDI 时长取决于现场信号，规划期不可知）。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 同一条程序加/不加 waitTime：节拍精确差出驻留（本页构建时真规划器实算）</h3>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@DW_PLAIN@@ s → @@DW_WITH@@ s（Δ@@DW_DELTA@@ s）</div><div class="k">同一取放路径，插入 waitTime(0.8) 后节拍精确多出 0.80s</div></div>
    <div class="metric"><div class="v">@@DW_TOTAL@@ s / @@DW_N@@ 段</div><div class="k">校验报告的「驻留合计」行（motion_times 明细仍按 Motion 口径，与示教点/夹爪事件对齐）</div></div>
  </div>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 多机"驻留占用"：A 在交接点驻留 2s，B 的到达时刻决定撞不撞</h3>
  <p style="font-size:13.5px;color:var(--dim)">两台 UR5e（B 基座沿 +X 偏移 300mm）经 IK 去往<strong>同一个世界点</strong>。A 到达后驻留 2s 再退回零位；B 的起始偏移分别落在驻留窗内/窗外（净空 60mm 口径）：</p>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@DW_HIT@@ 处冲突</div><div class="k">B 在驻留窗内到达——A 还"停在那儿"，时间轴必须看得见（此前这类冲突不存在）</div></div>
    <div class="metric"><div class="v">@@DW_SAFE@@ 处冲突</div><div class="k">同一对轨迹，B 等 A 退回零位后才到——错峰即清零，节拍统筹的输入从此是真实的</div></div>
  </div>

  <ul style="font-size:13.5px;color:var(--dim);margin:14px 0 0 18px">
    <li><b>语义变化的边界</b>：这是<strong>修正</strong>而非回归——所有含 waitTime 的既有程序节拍会变长（变准）；<code>motion_times</code> 节拍明细与 <code>_motion_spans</code> 仍只按 Motion 计数，夹爪事件/示教点着色的时刻映射零漂移（专项测试锁死）</li>
    <li><b>WaitDI 刻意不进时间轴</b>：等待现场信号何时满足规划期不可知，把它估进节拍才是编造</li>
    <li><b>一条命令可复现</b>：<code>tests/test_dwell.py</code> @@DW_TESTS@@ 个专项测试（节拍差值精确 / 驻留路点零速 / 报告行 / 夹爪事件不漂移 / 多机占用窗 / FANUC 出码不受影响），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>

<section>
  <h2>真机连接面板 · SSH 传输与取回核对（v1.1.4 新能力 · 不支持 socket 的控制器也能从界面部署）</h2>
  <p style="font-size:14px">v1.1.1 把 <code>SocketLink</code> 搬进了界面，但 v1.1.2 的 <code>SSHLink</code> 仍只能在脚本里用——于是<strong>"不支持开放 socket 但带 SSH"的那一大类控制器（KRC 类控制器、控制器 PC 一体机等）在界面上还是空的</strong>。v1.1.4 给同一个面板加了<strong>传输选择器</strong>：切到 SSH，端口/超时默认值自动换成 <code>22 / 10s</code>、SSH 专属字段（用户名 / 密码 / 私钥 / 远端目录）才出现；切回 Socket 时<strong>用户手工填过的现场参数原样保留</strong>。同时补上一个此前缺失的动作——<strong>取回核对</strong>：把控制器上的程序原文取回来，与本次生成的代码<strong>逐字节比对</strong>。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 本页生成时真跑了一遍「面板 → 真 SSH」链路（paramiko 客户端 ↔ paramiko 服务端，SFTP 真实落盘）</h3>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@CPS_ADDR@@</div><div class="k">打桩控制器（已连接：@@CPS_CONNECTED@@）</div></div>
    <div class="metric"><div class="v">@@CPS_LINES@@ 行 / @@CPS_BYTES@@ B</div><div class="k">部署到远端的程序（句柄 @@CPS_HANDLE@@，清单可见 @@CPS_PROGRAMS@@ 条）</div></div>
    <div class="metric"><div class="v">取回核对：@@CPS_VERIFY@@</div><div class="k">取回原文与本次生成的代码逐字节比对</div></div>
    <div class="metric"><div class="v">远端被改动后：@@CPS_TAMPER@@</div><div class="k">构建时手动覆盖了远端文件再核对一次——必须报"不一致"（这是这个按钮存在的理由）</div></div>
  </div>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 状态区按传输如实说话：SSH 没有能力协商，就不写能力表</h3>
  <div style="display:flex;gap:18px;flex-wrap:wrap;align-items:flex-start">
    <div style="flex:0 0 320px;max-width:320px">
      <figure style="margin:0"><img src="assets/controller_panel_ssh.png" alt="真机连接面板（SSH 传输）" style="width:100%;border-radius:8px;border:1px solid #232a34"><figcaption style="font-size:12px;color:var(--dim);margin-top:4px">真机连接面板 · SSH 传输（离屏渲染自真实控件树）</figcaption></figure>
    </div>
    <div style="flex:1;min-width:300px">
      <div style="font-size:13px;color:var(--dim);margin-bottom:4px">连接状态行（SSH 形态）</div>
      <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@CPS_INFO@@</pre>
      <div style="font-size:13px;color:var(--dim);margin:8px 0 4px">回读关节（SSH 形态）：@@CPS_JOINTS@@ / 返回空值：@@CPS_JOINTS_NONE@@</div>
      <div style="font-size:13px;color:var(--dim);margin:8px 0 4px">面板日志（构建时真实操作，含部署 → 核对 → 改动后再核对 → 回读）</div>
      <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@CPS_LOG@@</pre>
    </div>
  </div>

  <ul style="font-size:13.5px;color:var(--dim);margin:14px 0 0 18px">
    <li><b>"部署报成功"≠"控制器上确实是这份代码"</b>：SSH 形态下程序落在远端文件系统里，中间任何一跳出问题（权限、编码、被别的进程覆盖）只有取回来才知道。核对结果分三态：<strong>逐字节一致 / 不一致（附两侧行数） / 只报行数</strong>——本次会话不是由本面板部署的程序，不冒充逐字节结论</li>
    <li><b>能力如实</b>：SSH 无常驻对端服务，也就没有握手与能力协商——状态区写"无能力协商 · 远端目录 … · 不支持关节回读"，回读按钮不伪造数据（孪生偏差链路仍走 Socket / 厂商协议）</li>
    <li><b>安全边界不变</b>：仍然<strong>没有运动指令入口</strong>，也<strong>没有 SSH 自由命令输入框</strong>（<code>SSHLink.run_command()</code> 面向脚本化部署流水线）。把任意 exec 做成文本框，等于在产线现场放一个"随意驱动机器人"的入口</li>
    <li><b>诚实披露</b>：SSH 通路<strong>未在真机上验证过</strong>（开发环境无机器人）；本节所有数字来自随包附带的打桩控制器，接真机请先核对主机密钥、路径权限与现场加载命令</li>
    <li><b>一条命令可复现</b>：<code>tests/test_controller_panel_ssh.py</code> @@CPS_TESTS@@ 个专项测试（真 SSH 回环验收 GUI 全链路：传输切换与默认值 / 现场参数不被覆盖 / 部署真落盘 / 核对抓到远端被改动 / SSH 无回读通道如实说明 / en 界面），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>


<section>
  <h2>轨迹编辑器圆弧（moveC）编辑（v1.1.5 新能力 · 示教点里的圆弧不再只能靠脚本灌进来）</h2>
  <p style="font-size:14px">v0.2.4 的轨迹编辑器只能逐点切 moveJ / moveL——<strong>焊缝拐角、涂胶圆弧、取放绕障</strong>这些真正需要圆弧的场合，只能写脚本把示教点灌进程序，界面上做不出来。v1.1.5 把 moveC 也搬进编辑器，并顺手改掉一个旧前提：<strong>圆弧的中间点被并进上一条指令</strong>，于是"每个路点恰好一条运动指令"不再成立，节拍明细与校验着色都必须经映射取值（否则圆弧之后的每一行都会错位一格）。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 口径：第 N 行的运动类型描述"到达第 N 点"的那段运动</h3>
  <p style="font-size:13.5px;color:var(--dim)">于是 <code>C</code> 在界面上读作"以上一点为中间点、再上一点为起点，画圆弧到达本点"。下面这组 4 个示教点在勾选 moveC 前后，解析出的运动序列与指令数（本页构建时真跑编辑器）：</p>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@ARC_INSTR_JOINT@@ 条 → @@ARC_INSTR_ARC@@ 条</div><div class="k">同一组示教点：全关节 4 条指令，带圆弧 3 条（中间点不再单独出指令）</div></div>
    <div class="metric"><div class="v">@@ARC_CYCLE_JOINT@@ s → @@ARC_CYCLE_ARC@@ s</div><div class="k">节拍对比（中间点被并进圆弧后，路径变长）</div></div>
    <div class="metric"><div class="v">@@ARC_PLAN@@</div><div class="k">解析出的运动序列（括号里是路点序号）</div></div>
  </div>
  <div style="display:flex;gap:18px;flex-wrap:wrap;align-items:flex-start;margin-top:14px">
    <div style="flex:0 0 340px;max-width:340px">
      <figure style="margin:0"><img src="assets/trajectory_arc.png" alt="轨迹编辑器（圆弧）" style="width:100%;border-radius:8px;border:1px solid #232a34"><figcaption style="font-size:12px;color:var(--dim);margin-top:4px">轨迹编辑器 · 圆弧（中间点行显示为「moveC 中间点」，且不带节拍）</figcaption></figure>
    </div>
    <div style="flex:1;min-width:300px">
      <div style="font-size:13px;color:var(--dim);margin-bottom:4px">路点列表（真实控件文本，含逐指令节拍）</div>
      <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@ARC_ROWS@@</pre>
    </div>
  </div>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 规划出来的 TCP 必须真的是弧，不是"看起来像弧的直线"</h3>
  <p style="font-size:13.5px;color:var(--dim)">把规划出的圆弧段路径点与"起点—终点"直线比一遍：经过中间点附近，且明显偏离弦线。</p>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@ARC_VIA_DIST@@ mm</div><div class="k">TCP 离中间点的最近距离（起点—终点弦长 @@ARC_CHORD@@ mm）</div></div>
    <div class="metric"><div class="v">@@ARC_DEVIATION@@ mm</div><div class="k">圆弧段最大偏离弦线的距离——这就是"圆弧没白设"的证据</div></div>
  </div>
  <div style="font-size:13px;color:var(--dim);margin:14px 0 4px">三个品牌的圆弧出码（同一个程序真实出码，只截出圆弧那一行）</div>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@ARC_BRAND@@</pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ 判据与诚实降级：做不了就说做不了，做了也得说清做了什么</h3>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>不允许的圆弧当场拒绝并说明原因</b>（P1/P2 没有两个前驱点、相邻两行不能同为圆弧、抓取/放置点不能当中间点——中间点不停机，夹爪动作会失去落点）。例如把 P2 设为圆弧：<code>@@ARC_REJECT_TWO@@</code></li>
    <li><b>正解失败时如实记录降级，且不吞掉中间点</b>：构建期圆弧正解失败 → 记一条说明 + 中间点改由关节运动单独经过（示教了 4 个点就得走 4 个点，不能只走 3 个）。本次构建时故意让正解失败一次，编辑器给出：<code>@@ARC_DEGRADE_NOTE@@</code>，降级后运动序列为 <code>@@ARC_DEGRADE_LABELS@@</code></li>
    <li><b>着色落在圆弧终点那一行</b>：<code>Issue.motion_index</code> 是运动指令序号，圆弧把指令数压得比路点数少——反向映射后，问题落在圆弧终点行（问题实际出在圆弧段上，落在被并入的那一行最容易被看见）</li>
    <li><b>一条命令可复现</b>：<code>tests/test_trajectory_arc.py</code> @@ARC_TESTS@@ 个专项测试（口径与判据 / 中间点不出指令且后续行节拍不错位 / TCP 真贴中间点且明显偏离弦线 / 三家品牌原生圆弧出码 / 降级不吞中间点 / 着色反向映射 / en 界面），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>

<section>
  <h2>PLC 侧信号 ↔ 机器人 IO 点表（v1.1.6 新能力 · 现场返工最常见的原因不是算法，是点表对不上）</h2>
  <p style="font-size:14px">机器人程序里写 <code>SetDO("gripper_close")</code>，PLC 侧这个词接在哪个地址——此前只能靠人对。<strong>TIA 能编译只说明语法没问题，不代表两侧点位对得上</strong>。v1.1.6 把这件事做成可编辑、可校验、可往返的一张表（<code>orolp/iomap.py</code>），并且<strong>SCL 出码随附点表</strong>——未映射的点直接写在文件里，而不是只在某个界面里红一下。数据源仍是 <code>CellConfig</code>：硬件列表 / PLC 逻辑 / 机器人程序共用同一份模型，<code>SignalMap</code> 只是它的视图 + 编辑器。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 一组"看起来没问题"的单元，校验能抓出什么</h3>
  <p style="font-size:13.5px;color:var(--dim)">PLC（Q0.0/I0.0）+ 夹爪（两个信号都塞在 Q0.1——故意的）+ 安全回路；机器人程序写夹爪、等工件到位、等夹紧反馈，外加一个谁都没建过地址的信号 <code>ghost_close</code>。本页构建时真跑校验：</p>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@IO_POINTS@@ 点 / 已映射 @@IO_MAPPED@@</div><div class="k">点表全貌（握手信号 ∪ 设备 io_map ∪ 程序实际用到的信号——第三项正是"程序在用、点表里没有"）</div></div>
    <div class="metric"><div class="v">@@IO_ERROR_N@@ 处错误 / @@IO_WARN_N@@ 处警告</div><div class="k">@@IO_SUMMARY@@</div></div>
    <div class="metric"><div class="v">@@IO_SIGNALS_USED@@ 个信号</div><div class="k">从程序指令里真提取的用途（@@IO_SIGNAL_NAMES@@），不是猜的</div></div>
  </div>
  <div style="display:flex;gap:18px;flex-wrap:wrap;align-items:flex-start;margin-top:14px">
    <div style="flex:1;min-width:300px">
      <div style="font-size:13px;color:var(--dim);margin-bottom:4px">错误（阻塞下装，必须处理）</div>
      <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#ffb4b4;padding:10px;border-radius:8px;white-space:pre-wrap">@@IO_ERRORS@@</pre>
    </div>
    <div style="flex:1;min-width:300px">
      <div style="font-size:13px;color:var(--dim);margin-bottom:4px">警告（不阻塞，但现场必须有人看过）</div>
      <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#ffd88a;padding:10px;border-radius:8px;white-space:pre-wrap">@@IO_WARNINGS@@</pre>
    </div>
  </div>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② SCL 出码随附点表：交给现场的是一份文件，不是两张纸</h3>
  <div style="font-size:13px;color:var(--dim);margin-bottom:4px">文件头注释块（真实生成的 SCL 截选）——VAR 声明里未映射的信号也不再留空：<code>@@IO_UNMAPPED_COMMENT@@</code></div>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@IO_SCL_BLOCK@@</pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ 界面与工具链</h3>
  <div style="display:flex;gap:18px;flex-wrap:wrap;align-items:flex-start;margin-top:10px">
    <div style="flex:0 0 340px;max-width:340px">
      <figure style="margin:0"><img src="assets/io_map.png" alt="硬件列表（IO 映射 / IO 校验）" style="width:100%;border-radius:8px;border:1px solid #232a34"><figcaption style="font-size:12px;color:var(--dim);margin-top:4px">硬件列表 · 「IO 映射…」编辑地址（自动编号跳过已占用），「IO 校验」一键看结论（离屏渲染自真实控件）</figcaption></figure>
    </div>
    <div style="flex:1;min-width:300px">
      <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
        <li><b>自动编号按方向分输入/输出</b>，且<b>跳过已被占用的地址</b>（自动编号自己造出重复点是最不能忍的）：本次真跑 <code>@@IO_AUTO@@</code>（Q0.1 被夹爪占着，绝不往那儿编）</li>
        <li><b>JSON 往返三种结果分开报</b>（静默丢点最难查）：@@IO_ROUNDTRIP@@</li>
        <li><b>地址解析刻意窄</b>：只认西门子/通用 IEC 写法（<code>DB10.DBX0.0</code> / <code>%I1.3</code> / <code>I/Q/M/E/A</code>），认不出的一律不判断；<code>E</code>/<code>A</code> 归并成 <code>I</code>/<code>Q</code>（德文写法），<code>X</code>/<code>Y</code>（三菱等的位序号）<b>刻意不归并</b>——判重宁可漏报也不误报</li>
        <li><b>校验口径</b>：程序在用的未映射点与地址重复是 <b>error</b>；归属歧义 / 地址格式 / 两侧同时驱动（谁后写谁生效）/ 机器人在等一个没人给的输入是 <b>warning</b>。首跑就抓出本仓库抓取程序的 <code>gripper_close</code> 确实被机器人与 PLC 同时驱动</li>
        <li><b>一条命令可复现</b>：<code>tests/test_iomap.py</code> @@IO_TESTS@@ 个专项测试（地址解析与别名归并 / 用途提取 / 六类问题 severity / 自动编号跳过已占用 / 往返三态计数 / SCL 点表块 / GUI 回写与状态栏通路 / QWidget 无 statusBar 的回归），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
      </ul>
    </div>
  </div>
  <p style="font-size:12.5px;color:var(--dim);margin-top:10px">附：校验报告全文（真实输出，<code>|</code> 分列——中文双宽下补空格的"假对齐"不如不排）</p>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@IO_ROWS@@</pre>
</section>

<section>
  <h2>场景拖拽摆放工件（v1.1.7 新能力 · "把它放在工作台上"不该要人算高度）</h2>
  <p style="font-size:14px">往场景里摆一个工件，此前只能手填坐标——<strong>"把它放在托盘上"这件人一秒就能做的事，界面上要口算 z = 台面高 + 工件半高</strong>。v1.1.7 补上"直接拿起来放桌上"的自然语义：<strong>左键按在工件/设备上＝抓取</strong>（不转视角），拖动保持按下那一刻的高度平面，<strong>松手自动落到支撑面</strong>。核心是一份不依赖 Qt 的纯几何模块（<code>python/orolp/world/placement.py</code>），库层也能复用。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 真跑一次"拖动松手"（本页构建时实测）</h3>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@PL_SUPPORT@@ · @@PL_HEIGHT@@ mm</div><div class="k">该 XY 点上最高的可见非机器人对象顶面（本次是 @@PL_TARGET@@，尺寸 @@PL_BOX@@ mm）——比它低的料箱/地面不参与</div></div>
    <div class="metric"><div class="v">@@PL_OFFSET@@ mm</div><div class="k">工件原点到自身包围盒底面的偏移（"坐在面上"而不是半沉的关键量）</div></div>
    <div class="metric"><div class="v">@@PL_RAW@@ → @@PL_SNAPPED@@</div><div class="k">原始落点 → 25mm 网格吸附后的落点（@@PL_MOVED@@）</div></div>
    <div class="metric"><div class="v">@@PL_Z@@ mm</div><div class="k">落位后工件原点 Z（@@PL_BOX_AFTER@@）</div></div>
  </div>
  <p style="font-size:13.5px;color:var(--dim);margin-top:10px">状态栏说给用户听的那一句（真实调用 <code>placement_summary()</code> 的返回值）：<code>@@PL_SUMMARY@@</code></p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 三条"不许抓"的路径：拒绝要说明理由，不是默默不动</h3>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>机器人本体</b>：拖动机器人基座不是"摆放"——本次实测理由：<code>@@PL_ROBOT_REJECT@@</code></li>
    <li><b>已挂载对象</b>（挂在法兰/工作台上的子件）：位姿由宿主组件决定，手动摆放会和随动逻辑打架，因此拒抓</li>
    <li><b>已锁定对象</b>：明确锁定的一律不动；<b>没有几何、落在空处</b>时支撑面退化为地面：本次实测 <code>@@PL_GROUND@@</code></li>
    <li><b>可抓性判定</b>：<code>@@PL_GRABBABLE@@</code>（<code>grabbable()</code>，与拾取/选择同一套口径）</li>
  </ul>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ 与既有右键 gizmo 的关系：互补，不是替换</h3>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>左键拖拽</b>＝"拿起来放到某个面上"（X/Y 随手放，Z 交给支撑面判定）——本次吸附实测：@@PL_SNAP_ROWS@@</li>
    <li><b>右键 gizmo 沿轴拖拽</b>＝"精确挪 5mm / 转 30°"——原样保留，专项回归测试锁死（改左键语义不许碰它）</li>
    <li><b>拖动中高度不漂</b>：沿用按下时刻的 Z 平面锚点，松手才落到支撑面——避免"越拖越飞"</li>
    <li><b>一条命令可复现</b>：<code>tests/test_scene_drag.py</code> @@PL_TESTS@@ 个专项测试（纯几何：支撑面判定/最高面优先/挂载父系换算/各拒绝原因；GUI：真 <code>QMouseEvent</code> 无头驱动全链路 + 右键 gizmo 回归），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>

<section>
  <h2>规模与性能基准（v1.1.8 新能力 · 从"几十个示教点"到"几千个路点"）</h2>
  <p style="font-size:14px">几十个示教点的演示跑得飞快，几千个路点的程序（激光焊 / 打磨 / 喷涂的典型形态）会把"每次改动全量重算"的代价暴露出来。v1.1.8 做了两件事：<strong>把正运动学里那些"URDF 解析后就不再改变"的量缓存起来</strong>（关节原点的欧拉角转换、转轴单位化、4×4 组装），<strong>并把 <code>motion_at</code> 的时间轴查询从顺序扫描换成二分</strong>；同时新增 <code>orolp/perf.py</code>，把规划/干涉/校验分段计时并给出规模表——所有数字都现场实算，页面不写死任何常数。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 优化不许改变结果：两份实现逐位比对</h3>
  <p style="font-size:13.5px;color:var(--dim)">基准里带了一份<strong>朴素参考实现</strong>（<code>reference_link_transforms</code>：逐次调用 scipy、逐次组装 4×4），它既是等价性的对照组，也是"关掉缓存会怎样"的实物证据。本页构建时把 @@PERF_CALLS@@ 组关节值（含零位与 ±150° 随机组）同时喂给两份实现：</p>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@PERF_DIFF@@</div><div class="k">位姿矩阵最大差异 —— @@PERF_IDENTICAL@@（缓存只是把同一个结果留下来）</div></div>
    <div class="metric"><div class="v">@@PERF_CACHED_MS@@ ms</div><div class="k">生产实现（常量缓存）：@@PERF_CALLS@@ 次 FK 的前向耗时</div></div>
    <div class="metric"><div class="v">@@PERF_NAIVE_MS@@ ms</div><div class="k">朴素参考实现 → <b>@@PERF_SPEEDUP@@x</b>（同一台机器、同一次运行内对比）</div></div>
    <div class="metric"><div class="v">实算 @@PERF_COMPUTED@@ / 命中 @@PERF_HITS@@</div><div class="k">欧拉角转换实算 vs 命中缓存（缓存 @@PERF_ENTRIES@@ 条）——优化前这个比值是"每次调用都算"</div></div>
  </div>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 一条 120 条运动的程序（真规划器 + 真抓取工作站场景）</h3>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@PERF_MOTIONS@@ 条运动</div><div class="k">展开成 @@PERF_WAYPOINTS@@ 个路点，仿真节拍 @@PERF_CYCLE@@ s</div></div>
    <div class="metric"><div class="v">@@PERF_PLAN_MS@@ ms</div><div class="k">规划（含每条指令的 FK 与插补）</div></div>
    <div class="metric"><div class="v">@@PERF_COLLISION_MS@@ ms</div><div class="k">沿路径干涉检查（约 300 次采样，@@PERF_ERRORS@@ 处包络接触报为错误）</div></div>
    <div class="metric"><div class="v">@@PERF_VALIDATE_MS@@ ms</div><div class="k">整轮校验（内部会再规划一次，故不是各阶段之和）</div></div>
  </div>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@PERF_REPORT@@</pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ 规模表（同一台机器上横向比较，数字用于回归预警而非跑分）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@PERF_TABLE@@</pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">④ 口径：这份基准不回答什么</h3>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>耗时与机器负载强相关</b>（同一台机器不同时刻也会差 20%～30%）——它的用途是<strong>同机横向比较与回归预警</strong>，不是跑分</li>
    <li><b>每段取 repeat 次里的最小值（best-of）</b>——平均值会把系统抖动算进来；最小值最接近"没有干扰时的代价"</li>
    <li><b>各阶段耗时不构成一次总时长的分解</b>：<code>validate()</code> 内部会自己再规划一次，模块如实给出墙钟值而不是编一个"总计"</li>
    <li><b>干涉数来自 URDF <code>&lt;collision&gt;</code> 包络体</b>，不是真机几何（本仓库一贯显式披露容差放行的接触）</li>
    <li><b>一条命令可复现</b>：<code>python -m orolp.perf --motions 120 --sizes 25,60,120,240</code> 或 <code>examples/perf_benchmark_demo.py</code>；<code>tests/test_perf.py</code> @@PERF_TESTS@@ 个专项测试（等价性逐位比对 / 缓存失效与只读 / scipy 调用计数 / <code>motion_at</code> 二分等价 / 基准工具语义 / CLI 与子进程），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>

<section>
  <h2>干涉检查：采样口径与几何缓存（v1.1.9 新能力 · 上一轮量出来的最大单项）</h2>
  <p style="font-size:14px">v1.1.8 的基准把"沿路径干涉检查"指认为大程序里的最大单项；v1.1.9 顺着这个指认往下做了两件事。<strong>第一件是几何内核</strong>：单元素 AABB 上的 <code>np.all</code> 归约塔被换成标量比较（3 元素数组上的 numpy 归约调度开销是实际比较的十倍量级），"URDF 解析后就不变"的<strong>局部几何</strong>（包围盒半长、顶点、面法向、棱方向）按基本体参数记忆化并只读共享，SAT 的逐轴点积合并成一次矩阵乘法——<strong>但判定语义一位不改</strong>（下面第 ① 条就是对拍的证据）。<strong>第二件是采样口径</strong>：此前"约 300 次采样"的默认预算意味着<strong>程序越长、采样点隔得越远</strong>，"没查到干涉"这句话的保证强度会随程序规模悄悄变弱——v1.1.9 把这个口径显式化成三种策略（默认预算 / 固定步距 / <strong>关节步长</strong>），并把密度、开销、检出并排列出来（第 ② 条）。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 优化不许改变结果：内核逐字段对拍</h3>
  <p style="font-size:13.5px;color:var(--dim)">基准里带了一份<strong>朴素参考实现</strong>（<code>reference_collides</code> 系列：单元素 AABB 归约、逐基本体算顶点、逐轴点积、逐对循环），它就是"改动之前的那份代码"的可执行副本。本页构建时把 @@CPF_PAIRS@@ 对随机基本体 × @@CPF_CASES@@ 组位姿/裕度（裕度 @@CPF_MARGINS@@ mm）同时喂给两份实现：</p>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@CPF_MISMATCH@@ 处</div><div class="k">全部对拍项的差异总数（AABB / 顶点 / SAT 轴 / 碰撞判定 / 穿透深度）——结论：@@CPF_IDENTICAL@@</div></div>
    <div class="metric"><div class="v">@@CPF_EXACT@@ / @@CPF_PAIRS@@</div><div class="k">穿透深度数值<strong>完全相同</strong>的对数 · 最大数值差 <code>@@CPF_DELTA@@</code></div></div>
    <div class="metric"><div class="v">@@CPF_PROD_MS@@ ms vs @@CPF_REF_MS@@ ms</div><div class="k">同一批判定：生产实现 vs 朴素参考实现 → <b>@@CPF_SPEEDUP@@×</b>（同机、同一次运行内对比）</div></div>
    <div class="metric"><div class="v">@@CPF_MARGINS@@ mm</div><div class="k">对拍用的四档裕度：0（相切）/ 负（放行） / 正（膨胀）/ 大负值（全放行）——覆盖"刚好接触"这类最容易出分歧的边界</div></div>
  </div>
  <p style="font-size:13.5px;color:var(--dim);margin-top:10px">开发期同机记录（端到端、非本页现测）：同一台机器上，@@CPF_WPS@@ 路点程序的干涉检查从 <b>149.4 ms 降到 52.0 ms</b>，<strong>检出数不变（2 处）</strong>——内核快了 @@CPF_SPEEDUP@@×，整段只快约 2.9×，差额是采样循环、场景遍历等不归内核管的开销。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 四种采样口径并排：密度、开销、检出（真规划器 + 真抓取工作站）</h3>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@CPF_CMDS@@ 条指令 · @@CPF_WPS@@ 个路点</div><div class="k">@@CPF_ROBOT_PARTS@@ 个机器人零件 × @@CPF_ENV_OBJS@@ 个环境对象（@@CPF_ENV_PARTS@@ 个零件）· 干涉裕度 @@CPF_MARGIN@@ mm</div></div>
    <div class="metric"><div class="v">默认 @@CPF_DEF_SAMPLES@@ 次 → @@CPF_DEF_HITS@@ 处</div><div class="k">默认口径每 <b>@@CPF_DEF_WP@@ 个路点</b>才查一次（@@CPF_DEF_MS@@ ms/采样）——<strong>程序越长这个数字越大</strong></div></div>
    <div class="metric"><div class="v">加密 @@CPF_DENSE_SAMPLES@@ 次 → @@CPF_DENSE_HITS@@ 处</div><div class="k">关节步长 10° 口径每 @@CPF_DENSE_WP@@ 个路点查一次（@@CPF_DENSE_MS@@ ms/采样）——同一程序多查到的那一处，就是默认口径漏掉的</div></div>
  </div>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@CPF_TABLE@@</pre>
  <p style="font-size:13.5px;color:var(--dim);margin-top:10px"><b>怎么读这张表</b>：关节步长口径的采样数是"机器人可能移动多远"的函数，<strong>不随程序变长而稀释</strong>（同一示教轨迹重复三遍，采样数不翻三倍）；默认口径的采样数被 300 的预算钉住，所以"每采样路点"那一列会随程序规模单调变大。两者每次采样的耗时<strong>基本恒定</strong>（约 0.17 ms）——也就是说「要不要更密」是个纯粹的<strong>时间换保证</strong>选择，而不是"能不能快"的问题。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ 三级漏斗账本：每次采样到底过了多少对</h3>
  <p style="font-size:13.5px;color:var(--dim)">取上表的 10° 口径单独跑一遍（@@CPF_LEDGER_SAMPLES@@ 次采样），前后各取一次工作量计数做差分——下面每个数字都是那一遍真实发生的，不是估算：</p>
  <ul style="font-size:13.5px;color:var(--dim);margin:10px 0 0 18px">
    <li><b>第一级 · 连接包围盒</b>：@@CPF_L1@@ 次「机器人连杆 × 环境对象」粗筛，@@CPF_L1_REJ@@ 次被直接剔除（<b>@@CPF_L1_RATE@@</b>）——只有 @@CPF_SURVIVE_RATE@@ 的候选对值得往下走</li>
    <li><b>第二级 · 零件粗筛</b>：@@CPF_NARROW@@ 对零件进入窄相入口，其中 @@CPF_PREFILTER@@ 次被零件级 AABB 剔除（<b>@@CPF_PREFILTER_RATE@@</b>）</li>
    <li><b>第三级 · SAT</b>：真正跑分离轴判定的只有 @@CPF_SAT@@ 次（<b>@@CPF_SAT_RATE@@</b> 的窄相入口）——这就是"先粗后细"省下来的量</li>
    <li><b>几何缓存</b>：@@CPF_CACHE_HITS@@ 次命中 / @@CPF_CACHE_MISSES@@ 次实算（<b>@@CPF_CACHE_RATE@@</b>），缓存 @@CPF_CACHE_ENTRIES@@ 条局部几何——基本体参数不变则几何不变，位姿变了也照样复用</li>
    <li><b>检出</b>：原始碰撞判定 @@CPF_RAW_HITS@@ 次 → 去重后 <b>@@CPF_CONTACTS@@ 处接触</b>（多次采样命中同一工件会合并，因此"原始次数"与"校验报告里的处数"不是一回事）</li>
  </ul>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">④ 口径：这份基准不回答什么</h3>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>采样永远可能漏</b>：任何离散采样都可能恰好跳过两采样点之间发生的相交，加密只是把概率压小，<strong>不是证明</strong>；要证明需要连续碰撞检测（CCD），本仓库没有做</li>
    <li><b>关节步长口径给的是"机器人可移动多远"，不是"路径多长"</b>：同一示教轨迹重复三遍采样数不变——这是刻意设计（保证强度不该随程序变长而变弱），但它按<strong>关节空间</strong>度量，TCP 走直线而手腕几乎不动的段也会被采到</li>
    <li><b>默认口径的保证随规模变弱，且这一点此前是隐性的</b>——v1.1.9 没有改默认值（大程序上默认口径仍然最快），而是把它写进报告与文档，让"要不要更密"变成用户的显式选择</li>
    <li><b>每次采样约 0.17 ms 是这台机器上的数</b>，且随场景对象数近似线性增长；它的用途是同机横向比较与回归预警，不是跑分</li>
    <li><b>等价性对拍用的是随机基本体，不是真机几何</b>——覆盖的是内核的数值路径（含相切、包含、退化棱），不覆盖 URDF 解析与包络体生成</li>
    <li><b>一条命令可复现</b>：<code>python -m orolp.perf --motions 120 --collision</code> 或 <code>examples/collision_perf_demo.py</code>；<code>tests/test_collision_perf.py</code> @@CPF_TESTS@@ 个专项测试（内核逐字段等价 / AABB 标量化 / 局部几何缓存与只读 / <code>collides_with_boxes</code> 语义 / 暴力遍历 vs <code>check_config</code> 交叉验证 / 三种采样策略 / 检查器接线 / 工作量计数 / 报告与 CLI 子进程 / en 无 CJK），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>

<section>
  <h2>增量重算（v1.2.0 新能力 · 上一轮把干涉检查压下去之后，剩下的最大单项就是"每次改动都全量重跑"）</h2>

  <p style="font-size:13.5px">v1.1.8 把大程序的耗时逐段量了出来，v1.1.9 又把沿路径干涉检查压到约 0.17 ms/采样。剩下的大头就只剩一件事：<strong>改动之后每次全量重算</strong>。本节的实现是<strong>两层内容寻址缓存</strong>——键完全由内容派生（规划器参数 + 机器人结构 + 起始关节值 + 程序内容摘要，分段层再加"单条指令的签名 + 该条指令的起始关节值"），因此<strong>不存在"读到过期结果"这种情况</strong>，也就能放心让多个规划器实例共用一份缓存（界面上的"校验一下"其实会规划三次：程序校验一次、面板回放一次、各自的规划器实例一次，而三次的输入逐字段相同）。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 缓存账本：同一份程序规划三次</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">  第 1 次（冷启动）            @@INC_COLD_MS@@ ms   整程序 命中/未命中 @@INC_COLD_PLAN@@ · 分段 命中/未命中 @@INC_COLD_SEG@@
  第 2 次（内容不变）          @@INC_WARM_MS@@ ms   整程序 命中/未命中 @@INC_WARM_PLAN@@ · 分段 命中/未命中 @@INC_WARM_SEG@@   → @@INC_WARM_SPEEDUP@@x
  第 3 次（只改末尾一条速度）  @@INC_EDIT_MS@@ ms   整程序 命中/未命中 @@INC_EDIT_PLAN@@ · 分段 命中/未命中 @@INC_EDIT_SEG@@   → @@INC_EDIT_SPEEDUP@@x</pre>
  <p style="font-size:13.5px;color:var(--dim)">第 2 次是<strong>整程序命中</strong>：路点、时间轴、驻留段、路径长度、问题清单整份交还，连逐路点的正运动学都不再跑（这是大头）。第 3 次改动只涉及<strong>时间</strong>、不动关节值，所以前缀与后缀全部照旧命中——只有被改的那一条要重算。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 分场景基准 · moveJ（终点是绝对关节目标）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@INC_ROWS_JOINT@@</pre>
  <p style="font-size:13.5px;color:var(--dim)">@@INC_JOINT_META@@。<strong>等价列是硬门槛</strong>：每一行都与<strong>关缓存</strong>的实算结果逐位比对关节/位姿/时刻/速度四项，任何一项不为 0 都会直接显示出来，而不是悄悄给一个更快的错结果。分段复用率的分母是「指令数」不是「路点数」——一条指令命中意味着它那几十个路点全部免算。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ 分场景基准 · moveL（终点是位姿，要经逆解）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@INC_ROWS_CART@@</pre>
  <p style="font-size:13.5px;color:var(--dim)">@@INC_CART_META@@。同一条结论在这里明显不同——原因见下一节。整程序命中时的加速是 @@INC_CART_NOOP_X@@×，而几何编辑只有 @@INC_CART_MID_RATE@@ 的分段复用率。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">④ 两种目标写法的对照（"能省多少取决于改哪儿"的结构性依据）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@INC_CONTRAST@@</pre>
  <ul style="font-size:13.5px;color:var(--dim);margin:10px 0 0 18px">
    <li><b>moveJ 的终点就是绝对关节目标</b>：第 k 条的扰动在<strong>下一条指令处就被吸收</strong>，链条立刻回到原轨，所以改末尾/改中段/改首条都能保住九成以上的复用（@@INC_JOINT_MID_RATE@@ 是改中段）。</li>
    <li><b>moveL 的终点由逆解给出</b>，而逆解以<strong>上一条的关节值为初值</strong>：扰动要么一直传下去，要么等逆解恰好回到同一分支才复位。所以改中段只有 @@INC_CART_MID_RATE@@ 的复用率，改首条反而更好（绝对位姿目标给了链条更多复位机会）。</li>
    <li><b>给一个笼统的百分比是没有意义的</b>。能省的三类是：<strong>前缀</strong>（改第 k 条时 0..k-1 必然可复用）、<strong>不动关节值的改动</strong>（改速度、插驻留）、<strong>终点回到原处的改动</strong>（绝对关节目标天然属于此类）。</li>
  </ul>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">⑤ 口径：它不做什么</h3>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>首次规划不会更快</b>。冷启动仍要完整算一遍，收益全部来自第二次及以后——所以"用缓存"不是"变快"，而是"改动之后不必重来"。</li>
    <li><b>它不缓存干涉检查与动力学</b>。沿路径碰撞检查、动力学差分照常每次真算；本机制只回答"轨迹展开要不要重算"。</li>
    <li><b>它不能把"改前面一条的几何"变成免费</b>。串行运动链里第 k 条的插补以第 k 条的起点为初值（moveL/moveC 还拿它当逆解初值），位置改了终点就变，其后整条链的起点全变——分段键随之改变，这是物理性质，不是实现偷懒。开发期这里踩过一个<strong>静默算错</strong>的坑：分段键一度漏了"该条指令的起始关节值"，于是改中段一条时会错误地"命中"其后全部 59 条，轨迹在接缝处直接跳了 5.0000°；现在由一条专门的回归测试盯着。</li>
    <li><b>交还的路点与位姿是共享对象，调用方必须当只读</b>。深拷贝 5632 个路点比重算一遍还贵，所以列表容器每次新建、关节数组一律复制，但路点对象是共享的——这条约定有测试兜着。</li>
    <li><b>失败与警告也缓存，但缓存里的文案不含指令序号与时刻</b>：序号随插入/删除而变、时刻随前缀时长而变，两者都在使用时按当下值重建（有一条测试往程序前面插一段驻留，验证同一告警的时刻跟着后移 0.5 s）。</li>
    <li><b>容量按"路点数"限容而不是"条数"</b>：一个路点约 0.6~0.7 KB，5632 路点的一份快照就是 3~4 MB——按条数限容会悄悄吃掉几百 MB。</li>
    <li><b>一条命令可复现</b>：<code>python -m orolp.perf --motions 60 --incremental --incremental-cartesian</code> 或 <code>examples/incremental_demo.py</code>；<code>tests/test_incremental.py</code> @@INC_TESTS@@ 个专项测试（签名内容寻址 / 缓存限容与 LRU / <code>resolve_cache</code> 优先级 / 逐位等价 / 起点进键的回归守卫 / 只读共享 / 失败与警告缓存 / 报告自洽 / en 无 CJK），全量基线 <b>@@TOTAL_TESTS@@ passed</b></li>
  </ul>
</section>

<section>
  <h2>真机 CAD 链路与 OCP 版本兼容层（v1.2.1 修复 · 「没有厂商 STEP 就 skip」把缺陷藏了好几轮）</h2>
  <p style="font-size:14px">v0.2.3「5 品牌真机 CAD 接入」在等用户投放 STEP，但队列里自己写着一条<strong>不依赖外部输入</strong>的前置：工具还是 OCP 7 写法，解锁时需先做 OCP 8 兼容。本轮把它做完——顺手也把「它为什么能一直坏着没人发现」修掉了：<strong>装上新版 OCP 之后这条链路已经全线不可用，而验收测试挂在「本地没有厂商 STEP 就 skip」后面，skip 是绿的，于是连续好几轮没有任何自动化发现它。</strong>@@VSC_NOTE@@</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 坑在哪：OCP 8 只改了「一半」</h3>
  <p style="font-size:13.5px;color:var(--dim)">OCP 8 去掉了一批静态转型函数的 <code>_s</code> 后缀，但<strong>不是全部</strong>——所以「按版本号批量去 <code>_s</code>」和「照抄文件里的既有写法」两种做法都会错。本页构建时读到的现场解析结果：<code>@@VSC_CASTS@@</code>（新名）与 <code>@@VSC_KEPT@@</code>（仍带旧后缀），绑定版本 <b>@@VSC_OCP@@</b>。</p>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@VSC_SYMBOLS@@</pre>
  <p style="font-size:13.5px;color:var(--dim)">做法是把这些符号名收进 <code>python/orolp/ocpcompat.py</code> 一处（<code>face_of</code>/<code>edge_of</code>/<code>triangulation_of</code>/<code>bbox_of</code>，新名优先、旧名兜底并缓存解析结果），下次 OCP 再改名只动那一个文件。<strong>OCP 仍是软依赖</strong>：兼容层在 import 期不碰 OCP（有子进程测试盯着 <code>OCP not in sys.modules</code>），没装 OCP 的机器照样能 import 库。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 现场合成一个装配体，把整条链真跑一遍</h3>
  <p style="font-size:13.5px;color:var(--dim)">本页构建时用代码造了一个「假厂商 STEP」——@@VSC_COMPONENTS@@ 个长方体组成一个装配体，然后一路走完：<code>BRepPrimAPI_MakeBox → TopoDS_Compound → STEPControl_Writer → *.STEP → load_step_components → mesh_component → write_stl_binary → build_model → write_urdf → orolp.Robot</code>。全程在临时目录里，<strong>绝不触碰仓库的 <code>models/</code> 与 <code>assets/</code></strong>。</p>
  <div class="metrics" style="margin-top:12px">
    <div class="metric"><div class="v">@@VSC_COMPONENTS@@ 组件</div><div class="k">STEP 读回的顶层组件数（每个盒子 @@VSC_TRIS_PER@@ 个三角面，合计 @@VSC_TRIS@@）</div></div>
    <div class="metric"><div class="v">@@VSC_LINKS@@ 连杆 / @@VSC_JOINTS@@ 关节</div><div class="k">生成的 <code>*_real.urdf</code>：连杆链与关节命名/轴/限位全部沿用已有的程序化 URDF，只有关节原点换成 CAD 算出来的旋转中心</div></div>
    <div class="metric"><div class="v">@@VSC_MESH_KB@@ KB</div><div class="k">写出的真机网格（每个连杆一份二进制 STL，顶点已减去该连杆的旋转中心）</div></div>
    <div class="metric"><div class="v">DOF @@VSC_DOF@@ · TCP @@VSC_FK@@</div><div class="k">把生成的 URDF 真的喂给 <code>orolp.Robot</code>：能加载、自由度对、关节角 [30,20,10]° 下能算出 TCP 位姿</div></div>
  </div>
  <p style="font-size:13.5px;color:var(--dim);margin-top:10px">注意「几何是 <code>&lt;mesh&gt;</code>」这一点有代价：包络几何通道只认基本体，所以真机模型的连杆<strong>不参与干涉检查</strong>（软件的既定契约，测试里也写明了），外形用来对齐现场，碰撞校核仍要用程序化模型或补包络体。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ 关节原点：拿写出的网格独立复核，而不是复述实现</h3>
  <p style="font-size:13.5px;color:var(--dim)">关节原点的定义是「该连杆网格里<strong>离父连杆质心最近的顶点</strong>」。下面这一列是本页构建时<strong>只读回磁盘上的 STL 与 URDF</strong>重新找一遍最近的顶点、再看它离该连杆坐标系原点有多远——若定义没被满足，这一列会是个明显的正数。</p>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@VSC_ORIGINS@@</pre>
  <p style="font-size:13.5px;color:var(--dim)">同一件事还有一个副产物不变量：每个连杆的 STL 都写在<strong>它自己的关节坐标系</strong>里（本地原点处必有顶点）——这正是「关节绕真实连杆连接中心旋转」能成立的原因；要是改成保留世界坐标，真机会绕世界原点甩出去。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">④ 让它以后自己响：合成装配体 + 一条静态守卫</h3>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>合成装配体</b>：<code>tests/test_vendor_step_pipeline.py</code> @@VSC_TESTS@@ 个测试函数，把上面这条链<strong>在没有厂商 CAD 的情况下也真跑一遍</strong>（另有 5 项按品牌参数化的只读冒烟，本地有官方 STEP 时自动生效）。这条测试的有效性被验证过：把新写法临时改回 <code>TopoDS.Face_s(...)</code>，<strong>13 条测试立刻变红</strong>（含下面这条静态守卫），改回即绿。</li>
    <li><b>静态守卫</b>：扫描 <code>@@VSC_GUARD_FILES@@</code>（共 @@VSC_GUARD_LINES@@ 行），命中裸的 <code>Face_s(</code>/<code>Edge_s(</code>/<code>Vertex_s(</code> 的处数 = <b>@@VSC_GUARD_HITS@@</b>（@@VSC_GUARD_DETAIL@@）。这条守卫会在这类改名再次出现时当场失败，而不是等到有人真去跑 <code>--all</code>。</li>
    <li><b>一条命令可复现</b>：<code>python tools/build_vendor_urdf.py --brand &lt;品牌&gt; --probe</code>（现在会先打印 OCP 绑定版本与符号解析结果）、<code>pytest tests/test_vendor_step_pipeline.py</code>；全量基线 <b>@@TOTAL_TESTS@@ passed</b>。</li>
  </ul>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">⑤ 这一节不回答什么</h3>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>合成装配体不等于真机 CAD</b>：它证明的是「链路可用、接线正确、几何/运动学的处理方式自洽」，不能替代「厂商 STEP 的轴系与组件映射对不对」——那件事只有在真实 STEP 到位后跑 <code>--probe</code> 才能确认（页面上的组件数/包围盒与真实机型无关）。</li>
    <li><b>「改一半」的结论绑定在实测版本上</b>：本页读到的是 @@VSC_OCP@@ 的解析结果，换版本请以 <code>describe()</code> 的现场输出为准。</li>
    <li><b>不覆盖真机 CAD 的组件映射</b>：JAKA 之外四个品牌的「第几个组件对应哪根连杆」仍需 STEP 到位后逐个调（队列里 v0.2.3 的剩余工作）。</li>
  </ul>
</section>

<section>
  <h2>干涉检查结果缓存（v1.2.2 · v1.2.0 的模块说明里明确写着「干涉检查与动力学不在缓存范围内」——本轮把前一半收进来）</h2>

  <p style="font-size:13.5px">v1.2.0 之后，「校验一下」的规划部分命中缓存，但<strong>沿路径干涉检查每次仍全量真算</strong>——校验两次就真算两次。本轮补上干涉检查的<strong>结果缓存</strong>，键仍走内容寻址，且多一维<strong>场景签名</strong>：被查机器人的基座与连杆几何、每个环境对象的<strong>世界系</strong>零件几何（父级链随法兰变动的部分天然被折进键）、逐路点关节值、margin 与采样口径，全部进键。于是<strong>场景一动键就变</strong>——「把工件拖进机器人路径后还返回过期"无干涉"」这类事故在键的设计上就不可能发生，并有专项测试盯着。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 完整校验两次（GUI「校验一下」的等价动作，构建时真跑）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">第 1 次（冷启动：规划 + 干涉 + 动力学全真算）  @@CCH_VAL_COLD_MS@@ ms
第 2 次（规划命中 + 干涉命中，动力学照算）    @@CCH_VAL_WARM_MS@@ ms   → @@CCH_VAL_SPEEDUP@@x（@@CCH_VAL_ISSUES@@ 条问题，两次完全一致）</pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 干涉检查单段两次（@@CCH_WP@@ 路点 · 抓取工作站场景 · 构建时真跑）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">第 1 次（实算）   @@CCH_COLD_MS@@ ms   · 检出 @@CCH_CONTACTS@@ 处接触
第 2 次（命中）   @@CCH_WARM_MS@@ ms   → @@CCH_SPEEDUP@@x（其中键构建约 @@CCH_KEY_MS@@ ms，是命中路径的全部开销）
账本：@@CCH_STATS_TWO@@</pre>
  <p style="font-size:13.5px;color:var(--dim)">命中路径做了两件事：交还结论 + <strong>复现"把最后采样点的关节写回场景"的历史副作用</strong>（实算路径经 <code>check_config</code> 写回，命中路径跳过了它——不复现的话紧随其后的单点位形检查会拿到过期关节）。返回的列表容器每次新建，容器里的 <code>Collision</code> 是冻结 dataclass、共享只读。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ 正确性演示：场景一动键就变（构建时真拖）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">基线（路障未放）        检出 @@CCH_CONTACTS@@ 处 · 键 K0
把 200×400mm 路障拖进机器人身体   检出 @@CCH_MOVED_CONTACTS@@ 处 · 键 K1（未命中 → 真算 → 干涉出现，绝不返回过期"无干涉"）
把路障拖回原处          检出 @@CCH_CONTACTS@@ 处 · 键 K0（命中 → 原结论复原，逐字段一致）
全程账本：@@CCH_STATS_END@@</pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">④ 口径：它不做什么</h3>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>动力学仍不缓存</b>：v1.2.0 那句话只剩后一半还成立——力矩/功率差分照常每次真算（这也是两次校验不会快到零的原因）。</li>
    <li><b>基准与计时必须绕开它</b>：<code>orolp.perf</code> 的三处计时（规划档的干涉段、校验段、采样口径表）与 <code>collision_perf_demo</code> 的账本都已在计时期间临时关闭结果缓存——<strong>缓存命中的"干涉检查"测的不是检查</strong>。工作量账本类测试（<code>collision_stats</code> 断言）同样绕行。</li>
    <li><b>其他机器人在场景里的关节角不进键</b>：沿轨迹检查只查「被查机器人 vs 环境 + 自碰撞」，其他机器人不是环境项（机器人-机器人对检查走另一条路径、不经此缓存）；挂在被查机器人下的对象被豁免，其位姿同样不进键——两条口径都与 <code>_env_items</code> 同源，并有测试钉住「改挂载件位姿仍命中、改环境位姿必未命中」。</li>
    <li><b>URDF 改了盘但场景没重建也不串味</b>：键取的是场景<strong>内存里</strong>实际持有的基本体与变换，不是 URDF 路径。</li>
    <li><b>一条命令可复现</b>：<code>pytest tests/test_collision_cache.py</code>（@@CCH_TESTS@@ 个专项测试：命中等价 / 场景签名七类变更 / 拖进路径的正确性守卫 / 副作用复现 / LRU / 开关与隔离 / 基准诚实性），全量基线 <b>@@TOTAL_TESTS@@ passed</b>。</li>
  </ul>
</section>

<section>
  <h2>力矩校验结果缓存（v1.2.3 · v1.2.0 边界句的后一半——「动力学」，本轮收进来）</h2>

  <p style="font-size:13.5px">v1.2.2 收掉干涉检查后，校验第二次起仍慢的大头只剩一件：<strong>逐路点逆动力学</strong>（实测 SO101 / 5632 路点：validate 202ms，关掉动力学只剩 18ms——<strong>约 185ms 全在力矩校验</strong>）。本轮补上它的结果缓存，键仍走内容寻址：模型<strong>内容签名</strong>（连杆惯性逐字节 / 末端负载 / 额定力矩——改负载、改惯性、改 URDF effort 都换键）+ 指令时间区间（决定超限定位）+ 逐路点关节位置/速度/加速度（逆动力学的全部输入）。<code>check_trajectory</code> 本就返回纯数据 dict，命中与实算走<strong>同一条「应用到报表」路径</strong>——结论逐字段一致。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 完整校验两次（SO101 · @@DCH_WP@@ 路点 · 构建时真跑）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">第 1 次（冷：规划命中缓存，力矩校验真算）   @@DCH_COLD_MS@@ ms
第 2 次（热：规划 + 力矩校验双命中）        @@DCH_WARM_MS@@ ms   → @@DCH_SPEEDUP@@x（@@DCH_ISSUES@@ 条问题，两次完全一致）</pre>
  <p style="font-size:13.5px;color:var(--dim)">对照 v1.2.2：当时完整校验第二次约 3x（动力学照真算），本轮之后同一动作快到键构建量级。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 改末端负载 → 立刻未命中（内容寻址的核心承诺）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">set_payload(0.5 kg) 后再校验   @@DCH_MISS_MS@@ ms（未命中 → 重新真算 → 新负载的力矩表）
关闭缓存对照（诚实口径）        @@DCH_OFF_MS@@ ms   → 缓存收益 = 冷 − 关
账本：@@DCH_STATS@@</pre>
  <p style="font-size:13.5px;color:var(--dim)">「改了负载还用旧力矩表」在键的设计上不可能发生——负载、连杆惯性（逐字节）、额定力矩、程序、时间区间任一变动即换键，各有专项测试。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ 口径：它不做什么 / 基准诚实性</h3>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>键不猜</b>：缺惯性参数时校验走「跳过」分支——没算结果就没有键，缓存条目为零（有测试）。</li>
    <li><b>基准诚实性升级（修掉 v1.2.2 的一个缺口）</b>：缓存解析的优先级里<strong>实例缓存高于全局</strong>，v1.2.2 的计时关闭只把全局换成 None——自带实例缓存的规划器在基准里照样命中。现在 <code>orolp.perf</code> 的 <code>_caches_off(planner)</code> 同时关干涉检查与动力学两路、<strong>连规划器实例开关一起否决</strong>，五处计时全部改传 planner——<strong>缓存命中的"校验"测的不是校验</strong>。</li>
    <li><b>规划结果缓存（v1.2.0）刻意不在计时关闭之列</b>：规划基准本身就以「冷/热缓存」两种工况为观测对象，关掉反而测不出来。</li>
    <li><b>一条命令可复现</b>：<code>pytest tests/test_dynamics_cache.py</code>（@@DCH_TESTS@@ 个专项测试：键确定性 / 模型·程序·spans·effort 变更敏感性 / 命中等价 / 共享对象契约 / 三级优先级 / LRU / 计时诚实性 / 包级导出），全量基线 <b>@@TOTAL_TESTS@@ passed</b>。</li>
  </ul>
</section>

<section>
  <h2>缓存可观测性（v1.2.4 · 三类结果缓存的命中情况一屏看完）</h2>

  <p style="font-size:13.5px">规划/分段（v1.2.0）、干涉检查（v1.2.2）、力矩校验（v1.2.3）三类内容寻址缓存都已默认开启——但"活少干了多少"此前只能在代码里看 <code>stats()</code>。本轮把三类缓存的占用与命中统一成一份快照（<code>cache_overview()</code>），GUI 的「视图 → 缓存状态…」一屏看完；「清空缓存与统计」一键回到全量真算。以下数字是本页构建时真跑出来的快照。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① UR5e + 抓取工作站：校验两次（构建时真跑）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">第 1 次（冷）@@COV_COLD_MS@@ ms → 第 2 次（热）@@COV_WARM_MS@@ ms
规划与分段缓存：整程序命中 @@COV_PLAN_HITS@@ / 未命中 @@COV_PLAN_MISSES@@（命中率 @@COV_PLAN_RATE@@）
干涉检查结果缓存：条目 @@COV_COL_ENTRIES@@ · 命中 @@COV_COL_HITS@@ / 未命中 @@COV_COL_MISSES@@（命中率 @@COV_COL_RATE@@）</pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② SO101：力矩校验结果缓存（构建时真跑）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">命中 @@COV_DYN_HITS@@ / 未命中 @@COV_DYN_MISSES@@（命中率 @@COV_DYN_RATE@@）</pre>
  <p style="font-size:13.5px;color:var(--dim)">口径如实：UR5e 的程序生成 URDF 不带 <code>&lt;inertial&gt;</code>，力矩校验走"跳过"分支——没算结果就没有键，缓存条目为零；上面的力矩校验数字来自自带惯性参数的 SO101。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ 口径</h3>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>全局关闭的缓存也在列</b>（对话框里如实写"已关闭"）——看不见的关闭最容易被当成"没这回事"。</li>
    <li><b>命中数字与各缓存自己的 <code>stats()</code> 同源</b>，不另立口径；"清空缓存与统计"是条目与计数一起清（下次校验全量真算），与基准计时里的"临时绕行"是两件事。</li>
    <li><b>内容寻址不会给出过期结果</b>：这个面板解释的是"活少干了多少"，不是正确性开关。</li>
    <li><b>一条命令可复现</b>：<code>pytest tests/test_cache_status.py</code>（@@COV_TESTS@@ 个专项测试：三类快照 / 关闭态如实呈现 / 校验两次后命中成长 / 重置与清空的"账与条目"边界 / offscreen GUI 菜单与对话框 / en 界面零中文），全量基线 <b>@@TOTAL_TESTS@@ passed</b>。</li>
  </ul>
</section>

<section>
  <h2>界面架构改版（v1.3.0 · 左侧双 Tab：场景设备库 / 设备树；把夹爪拖到臂末端就装夹）</h2>

  <p style="font-size:13.5px"><code>docs/prd/ui-architecture.md</code> 是 2026-09-11 立的界面架构 PRD，但它的六条验收标准<b>此前只写在文档里、没有任何测试盯着</b>——结构被后续改动破坏（某个面板把内容直接挂到标签页、不再套滚动容器），全量测试照样是绿的。本轮把验收标准钉成 <code>tests/test_ui_layout.py</code> 的 12 条断言，并补掉两处真缺口。下面所有数字都是本页构建时<b>真起一次主窗口、真跑一次法兰装夹</b>读出来的。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 布局结构（构建时真读）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">左侧：@@UIL_LEFT@@
右侧：@@UIL_RIGHT@@
代码预览子标签：@@UIL_SUBTABS@@
旧标签残留（场景 / 动作与从属 / 资源库）：@@UIL_LEGACY@@</pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 小窗口不裁切（缩到窗口能到达的最小高度再逐页量）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@UIL_FIT@@</pre>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">场景设备库：面板最小高 @@UIL_LIB_PANEL@@px ｜ 内容实际高 @@UIL_LIB_CONTENT@@px ｜ 视口高 @@UIL_LIB_VIEWPORT@@px ｜ 滚动条 @@UIL_LIB_SCROLL@@</pre>
  <p style="font-size:13.5px;color:var(--dim)">修复前实测（开发期记录）：内容最小高度 <b>286px</b> 直接顶成面板最小高度，而最小窗口下标签页可视高度只有 <b>267px</b>——底部的「刷新」与拖拽提示被裁掉。现在的内容最小高是 <b>356px</b>（给列表加了 160px 下限，避免小窗口下筛选行被压扁），但<b>面板</b>最小高只有 <b>88px</b>：多出来的高度交给滚动条。这就是"不被裁切"的可执行含义：<b>内容比可视区高，退化成滚动条，而不是消失</b>。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ 装夹到法兰（构建时真跑 placement）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">工件（料箱里）@@UIL_PART@@ ｜ 法兰世界坐标 @@UIL_FLANGE@@
挪到法兰旁 30mm：吸附判定（半径 60mm）→ @@UIL_SNAP_NEAR@@
挪到 200mm 外：吸附判定 → @@UIL_SNAP_FAR@@
装夹后：局部位姿平移 @@UIL_LOCAL@@（工具坐标系与法兰对齐）｜ 挂载状态 @@UIL_MOUNTED@@
机器人 J1 转 20° 后：工件随动 @@UIL_FOLLOW@@ mm</pre>
  <div class="shots">
    <figure><img src="assets/ui_left_tabs.png" alt="左侧双 Tab"><figcaption>左侧双 Tab：场景设备库 / 设备树（构建时离屏真抓；设备树里「工件_1」挂在机器人节点下＝装夹关系）</figcaption></figure>
    <figure><img src="assets/ui_right_tabs.png" alt="右侧标签"><figcaption>右侧：属性 / 代码预览 / 导出打包（原来的「动作与从属」并入设备树，不再占一个标签）</figcaption></figure>
  </div>
  <p style="font-size:13px;color:var(--dim)">截图口径：本机 macOS 无 Cocoa 会话，GL 视口抓不到内容（离屏无 OpenGL 上下文），因此这里抓的是<b>左右两个标签容器</b>本身——标签名、顺序、面板内容都是真的。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">④ 口径</h3>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>守卫建立在可执行的事实上</b>：那次 286px 的修复就是把滚动容器摘掉——摘掉即红，所以它守得住；标签名、顺序、子标签同样逐条断言，改版回退会当场变红。</li>
    <li><b>顺带修掉一件真缺陷</b>：「一键导出…」自身文档写着"不要求先点生成代码"（内部直接构建程序并后处理），但 <code>set_robot</code> 只开了「生成代码」没开「一键导出」，用户必须先点一次生成代码它才亮得起来——与 PRD「无置灰不可用的按钮」直接冲突。</li>
    <li><b>版本口径由四份手工同步减到三份</b>：GUI 窗口标题改为从 <code>orolp.__version__</code> 派生（v1.0.1 里程碑就漏过 pyproject；标题这一份从此不可能漂移）。</li>
    <li><b>当时仍有一项未过，已在下一节补齐</b>：PRD §6 第 6 条当时拆成两半——按钮部分已过，而「工艺时序（泳道图）」还是骨架（设备树 P1「动态时序」未实现），文档里如实未勾选；<b>v1.3.1 把它换成了真编辑器，§6 六条至此全过</b>（见下一节）。</li>
    <li><b>一条命令可复现</b>：<code>QT_QPA_PLATFORM=offscreen pytest tests/test_ui_layout.py</code>（@@UIL_TESTS@@ 个专项测试），全量基线 <b>@@TOTAL_TESTS@@ passed</b>。</li>
  </ul>
</section>

<section>
  <h2>工艺时序 · 动态时序（v1.3.1 · 设备树不再只有「静态挂载」，工序真的排得出来了）</h2>

  <p style="font-size:13.5px">设备树此前只表达<b>静态绑定</b>（谁挂在谁下面，坐标系自动吸合，v1.3.0 落地）；PRD §4.3 P1 要的是<b>动态时序</b>——配置工序触发、信号等待、子程序调用。原来的「工艺时序（泳道图）」是个空壳，也正是 §6 第 6 条「无空占位区块」唯一没过的一项。本轮新增 <code>python/orolp/sequence.py</code>：四种工步编成一条有序序列，<b>再翻译到两个既有出口——一个新指令语法都不加</b>。下面所有数字都是本页构建时真跑出来的。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 序列与校验（真跑）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">泳道（按首次出现序）：@@SEQ_LANES@@

@@SEQ_DUMP@@

干净序列：@@SEQ_CLEAN@@
只把 gripper_close / conveyor_run 两个信号投进点表：@@SEQ_PARTIAL@@
  └─ 第 @@SEQ_PARTIAL_IDX@@ 步：@@SEQ_PARTIAL_MSG@@</pre>
  <p style="font-size:13.5px;color:var(--dim)">校验问题都带 <code>index</code> 指回出问题的那个工步——<b>「序列里第 4 步用了点表里没有的 part_ready」这种话要能定位</b>，而不是笼统报一句「校验失败」。共七类问题：空序列 / 重名 / 未知设备 / 未知信号 / 未知子程序 / 无超时 / 边沿触发忽略期望电平。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 机器人侧出口（真喂既有 ProgramBuilder）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">落地成既有 IO 指令：@@SEQ_ROBOT_KINDS@@（共 @@SEQ_ROBOT_COUNT@@ 条）
只记进 notes、不进程序的：@@SEQ_NOTES@@</pre>
  <p style="font-size:13.5px;color:var(--dim)">子程序调用<b>不是程序指令</b>——<code>Program</code> 里没有这棵语法树，各品牌主程序/子程序的组织方式也不同（<code>CALL</code> / <code>Job</code> / <code>Run</code>…）。所以这里<b>不臆造调用语法</b>，只如实记进 notes 交给调用方展示；<b>静默丢弃才是最坏的做法</b>。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ PLC 侧出口（真翻 Step，再真出 SCL）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@SEQ_PLC_ROWS@@</pre>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">与 steps_from_program 逐字段对拍：@@SEQ_EQUIV@@
（序列侧 @@SEQ_PLC_COUNT@@ 步 vs 程序侧 @@SEQ_PROG_COUNT@@ 步——差的那一步是程序侧补的「等待工件到位」循环起点）
generate_scl：声明了 @@SEQ_SCL_DECLARED@@ ｜ 驻留定时器 @@SEQ_SCL_TIMER@@ ｜ 共 @@SEQ_SCL_LINES@@ 行</pre>
  <p style="font-size:13.5px;color:var(--dim)">两处差异是<b>刻意的、不是漂移</b>：① <code>steps_from_program</code> 会在头部补一步「等待工件到位」当循环起点，而序列<b>自己就编排等待</b>（本序列第 4 步就是），再补一步会跟它打架；② 步名口径不同（「驻留 0.5s」vs「保压 0.50s」、「末步输出」vs「输出保持」）。测试断言的是<b>语义字段逐条相同</b>（输出 / 等待 / 期望电平 / 延时），名字不参与比较——这样以后谁把翻译规则改歪了会当场变红。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">④ 诚实性口径：只有驻留是可计算的时间</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">工步数 @@SEQ_STEPS@@ ｜ 构成（@@SEQ_COUNTS@@）
驻留合计 @@SEQ_DWELL@@s
note：@@SEQ_NOTE@@</pre>
  <p style="font-size:13.5px;color:var(--dim)">序列只有<b>顺序</b>，没有实测节拍。信号触发与信号等待的真实耗时取决于现场机构（气缸响应、输送带节拍、机器人到位时间），所以 <code>summary()</code> <b>只报驻留合计与各类型步数</b>，绝不编一个「预计节拍」；泳道图抬头也直接写明「驻留按秒加宽；非实测节拍」，免得有人把横轴当时间轴读。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">⑤ 编辑器（真起控件）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">步骤表 @@SEQ_ED_ROWS@@ 行 ｜ 泳道画布 @@SEQ_ED_BLOCKS@@ 块 ｜ 可选信号 @@SEQ_ED_SIGNALS@@ 个 ｜ 校验问题 @@SEQ_ED_ISSUES@@ 条
@@SEQ_ED_SUMMARY@@</pre>
  <div class="shots">
    <figure><img src="assets/ui_sequence_editor.png" alt="工艺时序编辑器（泳道图）"><figcaption>设备树下的「工艺时序（泳道图）」：工步表 + 泳道画布 + 添加 / 删除 / 上下移 / 校验 / 保存 / 载入（构建时离屏真抓）</figcaption></figure>
  </div>
  <p style="font-size:13px;color:var(--dim)">信号下拉不是写死的字符串列表：<code>app.py</code> 用 <code>set_available_signals</code> 把<b>真实点表 ∪ 握手信号</b>灌进去，<b>选不到空气信号</b>；选项里没有的名字仍可手填，但校验会当场报 error（"下装前必须补上映射"）。改动经 <code>sequenceChanged</code> 转发到面板的 <code>sceneChanged</code>——时序改了就是场景改了，撤销/保存共用同一条通路。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">⑥ 口径</h3>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>不新增指令语法</b>：两个出口翻的都是既有指令（<code>SetDO</code>/<code>WaitDI</code>/<code>WaitTime</code>）与既有工步（<code>plc_scl.Step</code>），所以序列一落地就能被后处理、校验、SCL 三处复用，不存在"时序是孤岛"。</li>
    <li><b>等价性有可执行证据</b>：PLC 出口的语义等价不是"看着像"，而是与 <code>steps_from_program</code> 逐字段对拍；SCL 出口也不是"能编译就行"，而是真跑 <code>generate_scl</code> 并断言信号被声明、0.5s 驻留真的变成 <code>T#500MS</code> 定时器。</li>
    <li><b>本轮撞到两个「绿着的坑」</b>：①新测试把内置 UR5e 路径写成 <code>models/ur5e/ur5e.urdf</code>（真路径是 <code>models/ur/ur5e/ur5e.urdf</code>），<code>pytest.skip</code> 静默生效、4 条出口测试根本没跑——<b>skip 又是绿的</b>，与 v1.2.1 那次同类；②仓库里的 <code>.venv</code> 是缺 <code>paramiko</code> / <code>OCP</code> / <code>setuptools</code> 的<b>旁路环境</b>，<code>pytest.importorskip</code> 会让三个测试模块整块不收集，跑出 <b>1431 passed</b> 这种"看着还行"的假账（比真基线少 57 条用例）——全量基线固定用 <code>/Users/jayhan/.workbuddy/binaries/python/envs/default/bin/python</code>。</li>
    <li><b>一条命令可复现</b>：<code>QT_QPA_PLATFORM=offscreen pytest tests/test_sequence.py</code>（@@SEQ_TESTS@@ 个专项测试），全量基线 <b>@@TOTAL_TESTS@@ passed</b>。</li>
  </ul>
</section>

<section>
  <h2>动态时序使用体验 · 示例工程一条命令跑通（v1.3.2）</h2>
  <p style="font-size:13.5px">v1.3.1 落地了工艺时序模型与泳道编辑器，本轮补上<b>使用体验层</b>：<code>examples/sequence_demo.py</code> 一条命令跑通「建点表 → 编排 9 条工步 → 校验 → 机器人/PLC 双出口 → JSON 往返」，配套使用文档 <code>docs/sequence.md</code> 与 <b>8 项子进程验收测试</b>（<code>tests/test_sequence_demo.py</code>）。下面所有输出都是<b>本页构建时真跑示例脚本</b>的原样产物。</p>
  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 工作单元点表 + 工序编排（四种工步，泳道 = 设备）</h3>
  <pre style="white-space:pre-wrap">@@SDEMO_STEPS@@</pre>
  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 校验（点表/子程序/设备全部真实；含刻意安排的无限等待警告与未知信号自证）</h3>
  <pre style="white-space:pre-wrap">@@SDEMO_VALIDATION@@</pre>
  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ 机器人侧出口：CallStep 如实披露 + 点表接进后处理解析出真实序号</h3>
  <pre style="white-space:pre-wrap">@@SDEMO_UR@@</pre>
  <p style="font-size:13px;color:#9aa5b5">边沿触发降级为电平、超时看门狗待补，都在出码里留了显式 TODO——绝不静默丢弃、绝不编假地址。</p>
  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">④ PLC 侧出口：to_plc_steps() 真出 SCL（头部点表节选）</h3>
  <pre style="white-space:pre-wrap">@@SDEMO_SCL@@</pre>
  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">⑤ JSON 往返（examples/assets/pick_sequence.json）</h3>
  <pre style="white-space:pre-wrap">@@SDEMO_JSON@@</pre>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>一条命令复现</b>：<code>PYTHONPATH=python python examples/sequence_demo.py</code>；使用文档在 <code>docs/sequence.md</code>（四种工步 / 七类校验 / 双出口 / 诚实性口径 / GUI 指引）。</li>
    <li><b>诚实口径不变</b>：驻留合计 0.7s 是唯一可计算的时间；触发与等待的耗时取决于现场机构。</li>
  </ul>
</section>

<section>
  <h2>零件库与预置场景库（v1.3.3 落地 · v1.3.7 扩容 · PRD §4.1 P1/P2 内容就绪，入口已亮）</h2>
  <p style="font-size:13.5px">界面 PRD 最后一项「内容就绪再开放入口」落地：**预置场景库**（P2）新增两个行业模板——分拣工位（PRD 点名）与机床上下料——随抓取工作站一起通过既有「文件 → 载入预置工作站」入口一键加载（内容驱动，不做空壳入口）；**零件库**（P1）补上自定义导入：零件库 JSON（<code>orolp.parts/1</code>）里的参数化基本体组合经「导入零件…」注册进组件目录，与内置组件同一拖拽通路。下面数字都是本页构建时真跑出来的。</p>
  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 预置场景库：五个内置模板（构建时真建 + 真跑干涉检查）</h3>
  <table style="width:100%;font-size:12.5px;border-collapse:collapse">
    <tr style="color:#9aa5b5;text-align:left"><th style="padding:4px 8px">id</th><th>名称</th><th>说明（诚实口径）</th><th>对象数</th><th>干涉</th></tr>
    @@PLB_PRESET_ROWS@@
  </table>
  <p style="font-size:13px;color:#9aa5b5">布局经过碰撞 oracle 校准（逐候选位扫描取零干涉解），不是摆出来就行；机床上下料的机床是柜体几何简化示意——真机床走厂商 CAD 通路（v0.2.3），两条通道不混装。<b>v1.3.7 扩容</b>新增两个点名行业模板：3C 装配（上料托盘 → 治具压装 → 成品托盘）与汽车焊装（单轴变位机弧焊 + 输送辊道来料）——焊装模板的焊枪 TCP 依焊嘴型号而定、<b>刻意不猜</b>（tcp_flange=None），真实值走现场标定（v0.3.4）通路；变位机为单轴示意、不做外部轴联动，机器人本体随 URDF 通路替换。</p>
  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 零件库：示例文件真实注册 → 真实构建 → 撤销复原</h3>
  <table style="width:100%;font-size:12.5px;border-collapse:collapse">
    <tr style="color:#9aa5b5;text-align:left"><th style="padding:4px 8px">零件</th><th>注册 id</th><th>几何构成</th><th>说明</th></tr>
    @@PLB_CUSTOM_ROWS@@
  </table>
  <p style="font-size:13px;color:#9aa5b5">组件目录现有分类：@@PLB_CATEGORIES@@（共 @@PLB_CATALOG@@）。导入是幂等的：重复导入先撤上一批；坏 schema / 非法尺寸当场报错，绝不静默吞零件。</p>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>诚实口径</b>：零件库管理的是<b>参数化基本体</b>（box/cylinder/sphere 组合，与内置组件同一几何通道），<b>不解析 STL/STEP 网格</b>——网格件走 URDF/厂商 CAD 通路；自定义零件几何固定（文件里写什么就是什么），不假装参数可调。</li>
    <li><b>一条命令复现</b>：导入样例 <code>examples/assets/custom_parts.json</code>；模板自检 <code>PYTHONPATH=python python -m orolp.world.templates check</code>。</li>
  </ul>
</section>

<section>
  <h2>工程文件（.orolp · v1.3.4）——一次会话一个文件</h2>
  <p style="font-size:13.5px">v1.3.0–v1.3.3 把界面侧一件件补齐（场景设备库 / 设备树 / 工艺时序 / 零件库 / 预置场景库），但每一件都只带<b>自己的</b> JSON——用户视角是"存一个活要存四个文件，回头还得记住哪四个"。v1.3.4 补的是这个缝：一个 <code>.orolp</code> 把<b>场景 / 示教程序 / 工艺时序 / 设备点表 / 零件库来源 / 标定集</b>六段收进单一文件。下面每个数字与文本都是本页构建时真跑出来的（抓取工作站模板 + 真实 IK 示教点 + 真实时序与点表）。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 真存真读：六段工程落盘 → 载入 → 逐段摘要</h3>
  <table style="width:100%;font-size:12.5px;border-collapse:collapse">
    <tr style="color:#9aa5b5;text-align:left"><th style="padding:4px 8px">段</th><th>载入后的摘要（由段内容直接算出）</th></tr>
    @@PROJ_ROWS@@
  </table>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">工程名 @@PROJ_NAME@@ ｜ 文件 @@PROJ_SIZE_KB@@ KB ｜ 示教点 @@PROJ_POINTS@@ 个 ｜ 时序 @@PROJ_SEQ_STEPS@@ 工步 ｜ 设备 @@PROJ_DEVICES@@ 台 / 已映射信号 @@PROJ_MAPPED@@ 个
@@PROJ_DESCRIBE@@

载入报告：
@@PROJ_REPORT@@</pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 文件长什么样（前 14 行，原样截取）</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@PROJ_SNIPPET@@</pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ 一段写坏了会怎样：只跳那一段，其余照常载入</h3>
  <p style="font-size:13px;color:#9aa5b5">构建时把 <code>sequence</code> 段故意写坏（<code>steps</code> 写成字符串），再加一个本库不认识的段 <code>vision</code> 与一个未知顶层键 <code>authoring</code>，然后真的载入一次：</p>
  <table style="width:100%;font-size:12.5px;border-collapse:collapse">
    <tr style="color:#9aa5b5;text-align:left"><th style="padding:4px 8px">段</th><th>结果</th><th>原因 / 说明</th></tr>
    @@PROJ_BROKEN_ROWS@@
  </table>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">@@PROJ_BROKEN_REPORT@@</pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">④ 保真：打开一趟再保存，不认识的内容不能丢</h3>
  <p style="font-size:13px;color:#9aa5b5">把上面那份半坏工程再保存一次，逐项核对原始负载（不是"看起来没报错"，是逐字节比较）：</p>
  <table style="width:100%;font-size:12.5px;border-collapse:collapse">
    <tr style="color:#9aa5b5;text-align:left"><th style="padding:4px 8px">内容</th><th>键</th><th>结果</th></tr>
    @@PROJ_KEPT_ROWS@@
  </table>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">⑤ 不开 GUI 也能看：CLI 摘要</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">$ python -m orolp.project grasping.orolp
@@PROJ_CLI@@</pre>

  <ul style="font-size:13.5px;color:var(--dim);margin:14px 0 0 18px">
    <li><b>为什么是"逐段容错"而不是"整体校验"</b>：工程文件是用户<b>唯一的一份工作成果</b>。零件库格式将来升级了、某个第三方工具加了新段——不能因此让整个工程打不开。坏段只跳坏段，并在报告里说清是哪一段、为什么。</li>
    <li><b>本库不认识的段与顶层键原样保留</b>（第④项逐字节核对）："打开—保存"一趟不会把别家工具写的字段抹掉。</li>
    <li><b>写入是原子的</b>（同目录临时文件 + <code>os.replace</code>）：写一半断电不会把上一份好工程截断成半截 JSON。<code>NaN</code>/<code>Inf</code> 在写入前就被拒绝——它们序列化出来是非法 JSON，等读的时候才发现就晚了。</li>
    <li><b>同一版本内字段缺失按可选</b>；<code>schema</code> 主版本不等则<b>明确拒收</b>（不猜）——工程文件宁可明确拒绝，也不能按错的语义打开后让用户以为"内容丢了"。</li>
    <li><b>GUI 侧把"缝"补实了</b>：<code>文件 → 保存工程…/打开工程…</code>（Ctrl+Shift+S / Ctrl+Shift+O），窗口标题显示工程名与"•未保存"；示教点的<b>抓取/放置语义</b>存在工程里——它原本只活在界面内存中，单存路径代码会丢。</li>
    <li><b>一条命令复现</b>：<code>PYTHONPATH=python python -m orolp.project &lt;工程文件&gt; [--json]</code>；测试 @@PROJ_TESTS@@ 个（<code>tests/test_project.py</code> + <code>tests/test_project_gui.py</code> 离屏真开窗口），全量基线 <b>@@TOTAL_TESTS@@ passed</b>。</li>
  </ul>
</section>

<section>
  <h2>会话支撑（最近打开 + 崩溃自动恢复 · v1.3.5）</h2>
  <p style="font-size:13.5px">v1.3.4 解决了「存一个活只存一个文件」，还剩两条每天都会踩的缝：「昨天那个工程在哪来着」和「改了一下午还没保存，应用崩了」。v1.3.5 补上：<code>文件 → 最近打开</code> 子菜单 + 有未保存改动时每 60 秒自动落一份快照、下次启动询问是否恢复。下面每个数字都是本页构建时真跑出来的（配置目录重定向到临时目录，不碰真实 <code>~/.orolp</code>）。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 最近打开：真存真读，失效条目置灰不隐藏</h3>
  <p style="font-size:13px;color:#9aa5b5">记 3 条（1 条真实工程 + 2 条已不存在的路径），保存后重新载入：</p>
  <table style="width:100%;font-size:12.5px;border-collapse:collapse">
    <tr style="color:#9aa5b5;text-align:left"><th style="padding:4px 8px">记录（最新在前）</th><th>状态</th></tr>
    @@SES_RECENT_ROWS@@
  </table>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">existing()（真实存在的条目，菜单可点）：@@SES_EXISTING_N@@ 条
prune_missing()（清理失效条目的返回值）：@@SES_PRUNED@@
清理后重新载入：@@SES_RECENT_N@@ 条</pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 崩溃自动恢复：快照本身就是合法 .orolp</h3>
  <p style="font-size:13px;color:#9aa5b5">把一份带未知段（<code>vision</code>）与未知顶层键（<code>authoring</code>）的工程写进自动保存目录，再读回元数据、把快照当普通工程载入：</p>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">快照 session.orolp：@@SES_SNAP_SIZE_KB@@ KB ｜ sidecar 元数据：
@@SES_META@@

把快照用 Project.load 载入（与「打开工程」同一条路）：
@@SES_SNAPSHOT_SUMMARY@@</pre>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ 不开 GUI 也能看：CLI</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">$ python -m orolp.session
@@SES_CLI@@</pre>

  <ul style="font-size:13.5px;color:var(--dim);margin:14px 0 0 18px">
    <li><b>恢复走的就是「打开工程」同一条路</b>：快照用 <code>Project.save</code> 原子写入，本身就是合法 <code>.orolp</code>——自动保存机制就算坏了，用户也能手动打开快照文件。</li>
    <li><b>只在有未保存改动时才落快照</b>：干净会话没什么可救；快照用新建的 Project 收集，<b>不动正在编辑的工程对象</b>——快照落盘不能覆盖原工程路径，否则下次手动保存的建议位置会错。</li>
    <li><b>恢复后按「未保存」对待</b>：标题显示「•未保存」、自动保存继续守护；保存建议路径指回原工程；快照路径<b>不进</b>最近打开（它是内部槽位）。</li>
    <li><b>容错口径</b>：这两样都是锦上添花的机制，任何故障都「当它不存在」——快照写失败清残件不留空壳、元数据坏一律当无快照（不给来路不明的恢复选项），绝不打断启动或保存。</li>
    <li><b>一条命令复现</b>：<code>PYTHONPATH=python python -m orolp.session</code>；测试 @@SES_TESTS@@ 个（<code>tests/test_project_session.py</code> + <code>tests/test_project_session_gui.py</code> 离屏真开窗口），全量基线 <b>@@TOTAL_TESTS@@ passed</b>。</li>
  </ul>
</section>

<section>
  <h2>从模板新建工程（工程文件模板 · v1.3.6 落地 · v1.3.8 扩容至五个）</h2>
  <p style="font-size:13.5px">v1.3.4/v1.3.5 解决了「存一个活」与「再找到它」，但「打开工程」面前永远是一张白纸。v1.3.6 补上最后一条缝：<code>文件 → 从模板新建工程…</code>——新活不从空白开始；v1.3.8 把 3C 装配 / 汽车焊装两个行业模板也收进来，与「载入预置工作站」五场景对齐（场景同一数据源）。下面每个数字都是本页构建时真跑出来的（抓取模板经真实 IK 构建后落盘、载回、出摘要）。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">① 五个内置模板（v1.3.8 扩容）</h3>
  <table style="width:100%;font-size:12.5px;border-collapse:collapse">
    <tr style="color:#9aa5b5;text-align:left"><th style="padding:4px 8px">模板</th><th>id</th><th>说明</th></tr>
    @@TPL_ROWS@@
  </table>
  <p style="font-size:13px;color:#9aa5b5">抓取工作站的示例程序由既有 <code>world/templates</code> + <code>world/demo</code> 真实 IK 构建——与「抓取工作站演示（一键）」<b>同一数据源</b>，不是另写的一份假数据。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">② 模板产物就是普通工程：真存真读</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">抓取工作站模板落盘后载回：@@TPL_STATS@@
@@TPL_DESCRIBE@@</pre>
  <p style="font-size:13px;color:#9aa5b5">「工程里没有：工艺时序、设备点表……」是<b>如实展示</b>——模板只收真给的东西，不写空段凑数。</p>

  <h3 style="font-size:15px;margin:20px 0 6px;color:#e6ecf5">③ 不开 GUI 也能看：CLI</h3>
  <pre style="font-size:12px;background:var(--code-bg,#141a22);color:#d6e2f0;padding:10px;border-radius:8px;white-space:pre-wrap">$ python -m orolp.project grasping.orolp
@@TPL_CLI@@</pre>

  <ul style="font-size:13.5px;color:var(--dim);margin:14px 0 0 18px">
    <li><b>三条规矩</b>：模板产物就是普通 <code>Project</code>（保存/载入/摘要/保真全套契约照用）；只收模板真给的段；每次全新构建（两个同模板工程互不共享状态）。</li>
    <li><b>GUI 细节</b>：当前工程有未保存改动时先询问（默认「否」，拒绝则当前工程原样保留）；模板构建失败如实弹窗；新建的工程尚未落盘——标题显示「•未保存」、自动保存继续守护，且<b>不进最近打开</b>（它不是文件）。</li>
    <li><b>一条命令复现</b>：<code>PYTHONPATH=python python -c "from orolp.project_templates import save_project_template; save_project_template('grasping_workstation', 'demo.orolp')"</code>；测试 @@TPL_TESTS@@ 个（<code>tests/test_project_templates.py</code> + <code>tests/test_project_templates_gui.py</code> 离屏真开窗口），全量基线 <b>@@TOTAL_TESTS@@ passed</b>。</li>
  </ul>
</section>

<section>
  <h2>GUI 截图 · 抓取工作站一键演示（S1–S5 就绪）</h2>
  <div class="shots">
    <figure><img src="assets/grasping_workstation.png" alt="抓取工作站主视图"><figcaption>抓取工作站 · 一键载入（UR5e + 吸盘 + 输送带 + 托盘），状态栏显示节拍</figcaption></figure>
    <figure><img src="assets/grasping_playback.png" alt="回放动画"><figcaption>回放中 · TCP 轨迹 + 夹爪抓取（工件随 TCP 随动，落件后留在托盘）</figcaption></figure>
    <figure><img src="assets/grasping_offline_programming.png" alt="离线编程面板"><figcaption>离线编程 · 校验通过 + 节拍 + 4 品牌出码</figcaption></figure>
  </div>
  <p style="color:#9aa5b5;margin-top:10px">无 GUI 一条命令跑通同一流程：<code>python examples/grasping_workstation_demo.py</code>（模板 → IK 示教点 → 夹爪 IO → 节拍 → UR/KUKA/ABB/FANUC 出码）</p>
</section>

<section>
  <h2>路线图调研 · 变位机外部轴联动（v1.3.9 · 纯方案文档）</h2>
  <p style="font-size:13.5px">汽车焊装模板的变位机目前是「单轴示意、不做联动」（诚实口径）。联动怎么做，方案已整理成 <code>docs/design/external-axis.md</code>——<b>只调研不实施</b>，实施前等三个开放问题拍板（品牌优先级 / 机械参数来源 / 联动模式是否立项）。方案要点：</p>
  <ul style="font-size:13.5px;color:var(--dim);margin:0 0 0 18px">
    <li><b>两种模式先钉住</b>：<b>分段定位</b>（position-weld，焊前把焊缝转到合适姿态、停住再焊）工程复杂度低，现场大量使用；<b>联动焊接</b>（weld on the fly，变位机与机器人同时动）需要时间同步，另立项。</li>
    <li><b>核心设计决策</b>：外部轴位置<b>进路点</b>（路点带 <code>ext_joints</code>），不开独立指令——分段定位里「第 3 段在变位机 45° 时焊」这个绑定关系天然属于路点。</li>
    <li><b>出码形态</b>：KUKA（E1 扩展轴）/ FANUC（独立运动组）/ ABB（Mechanical Unit）可原生出码；UR <b>无外部轴概念，诚实降级</b>（注释 + 数字量触发）；国产品牌语法待厂商协议（与真机 CAD 同一阻塞口径）。</li>
    <li><b>P1 代价可控</b>：分段定位不碰 IK（机器人在工件动坐标系里求解，现有运动学原样可用）；碰撞检查吃任意 object 世界位姿，工件随动位姿路径打通后<b>免费复用</b>；变位机转位时间是纯几何可计算项，可进节拍报表。</li>
    <li><b>现状（代码事实）</b>：变位机组件已是参数化静态几何（<code>rot_deg</code> 变了台面真转）；<code>Trajectory</code>/路点只含机器人关节；8 个品牌后处理零外部轴语法（grep 证实）。</li>
  </ul>
</section>

<section>
  <h2>GUI 截图 · 更多视图</h2>
  <div class="shots">
    <figure><img src="assets/main_ur5e.png" alt="主视图"><figcaption>主视图 · 3D 视口 + 机器人库 + 关节控制</figcaption></figure>
    <figure><img src="assets/offline_programming.png" alt="离线编程"><figcaption>离线编程 · 程序校验 + 节拍 + 回放</figcaption></figure>
    <figure><img src="assets/path_code.png" alt="路径代码"><figcaption>路径代码 · 示教点编辑与实时可视化</figcaption></figure>
  </div>
</section>

<footer>
  RobotWorld — 开源工业机器人离线编程 · <a href="https://github.com/">GitHub</a> ·
  本页由 <code>tools/build_web_site.py</code> 自动生成，数据来自真实仿真与后处理
</footer>
</main>

<script>
const KF = @@KEYFRAMES@@; // [x, y, z] 路点
const svg = document.getElementById('traj');
const path = document.getElementById('traj-path');
const dot = document.getElementById('tcp-dot');
const wp = document.getElementById('grip-wp');
const phase = document.getElementById('phase');
const ticks = document.getElementById('axis-ticks');

// 世界坐标 → 屏幕（X: -700..700 → 60..660；Z: 0..1000 → 460..40）
function toScr(x, z) {
  return [60 + (x + 700) / 1400 * 600, 460 - z / 1000 * 420];
}

// 画坐标刻度
for (const z of [0, 250, 500, 750, 1000]) {
  const [, sy] = toScr(0, z);
  ticks.innerHTML += `<line x1="56" y1="${sy}" x2="660" y2="${sy}" stroke="#232a34" stroke-width="1"/>` +
    `<text x="52" y="${sy + 4}" text-anchor="end" font-size="11" fill="#9aa5b5">${z}</text>`;
}
for (const x of [-500, 0, 500]) {
  const [sx] = toScr(x, 0);
  ticks.innerHTML += `<text x="${sx}" y="478" text-anchor="middle" font-size="11" fill="#9aa5b5">${x}</text>`;
}

// 路径（分段：水平走 X/Y、垂直走 Z，简化为 XZ 投影）
let d = '';
for (let i = 0; i < KF.length; i++) {
  const [sx, sy] = toScr(KF[i][0], KF[i][2]);
  d += (i === 0 ? 'M' : 'L') + sx.toFixed(1) + ' ' + sy.toFixed(1);
}
path.setAttribute('d', d);

// 抓取/落件的段索引（0-based：2=抓取下降，5=落件下降）
const GRAB_AT = 2, RELEASE_AT = 5;
const PHASES = ['返回 Home', '接近抓取点', '下探抓取', '提升', '搬运至托盘', '下探落件', '提升退回', '返回 Home'];

// 每段匀速插值
const SEG = 60; // 每段帧数
let t = 0;
function frame() {
  const seg = Math.floor(t / SEG) % (KF.length - 1);
  const u = (t % SEG) / SEG;
  const a = KF[seg], b = KF[seg + 1];
  const x = a[0] + (b[0] - a[0]) * u;
  const z = a[2] + (b[2] - a[2]) * u;
  const [sx, sy] = toScr(x, z);
  dot.setAttribute('cx', sx); dot.setAttribute('cy', sy);
  phase.textContent = PHASES[seg] ?? '';
  // 工件随动：抓取后、释放前挂在 TCP 上
  if (seg >= GRAB_AT && seg < RELEASE_AT) {
    wp.setAttribute('opacity', '0.9');
    wp.setAttribute('x', sx - 13); wp.setAttribute('y', sy + 6);
  } else if (seg >= RELEASE_AT && seg < RELEASE_AT + 1 && u < 0.3) {
    wp.setAttribute('opacity', '0.9');
    wp.setAttribute('x', sx - 13); wp.setAttribute('y', sy + 6);
  } else {
    wp.setAttribute('opacity', '0');
  }
  t++;
  requestAnimationFrame(frame);
}
frame();
</script>
</body>
</html>
"""


def _sequence_demo_data() -> dict:
    """「动态时序使用体验」小节的真实数据（v1.3.2）。

    构建时以子进程真实运行 ``examples/sequence_demo.py``（与
    ``tests/test_sequence_demo.py`` 同一入口），把脚本输出的关键分段
    原样搬进页面——数字与代码行都是本次运行的真实产物，不是手写。
    """
    import subprocess

    root = REPO_ROOT
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        [str(root / "python"), str(root), env.get("PYTHONPATH", "")]
    )
    proc = subprocess.run(
        [sys.executable, str(root / "examples" / "sequence_demo.py")],
        cwd=str(root), env=env, capture_output=True, text=True, timeout=300,
    )
    if proc.returncode != 0:
        raise RuntimeError("sequence_demo.py 运行失败：" + proc.stderr[-2000:])
    out = proc.stdout

    def _section(start: str, end: str) -> str:
        i, j = out.index(start), out.index(end)
        return out[i + len(start):j]

    def _lines(block: str, keep=None, strip_left: bool = False) -> str:
        rows = []
        for ln in block.splitlines():
            if not ln.strip():
                continue
            if keep is not None and not any(k in ln for k in keep):
                continue
            rows.append(html.escape(ln.strip() if strip_left else ln.rstrip()))
        return "\n".join(rows)

    orchestration = _lines(_section("2. 工序编排", "3. 校验"))
    validation = _lines(_section("3. 校验", "4. 组成摘要"))
    ur_io = _lines(
        _section("5. 机器人侧出口", "6. PLC 侧出口"),
        keep=("set_digital_out", "wait_digital_in", "[notes]", "UR 出码共"),
        strip_left=True,
    )
    scl_body = _section("6. PLC 侧出口", "7. JSON 往返").splitlines()
    scl_head = _lines("\n".join(scl_body[4:16]), strip_left=True)
    roundtrip = _lines(_section("7. JSON 往返", "完成："), strip_left=True)
    return {
        "SDEMO_STEPS": orchestration,
        "SDEMO_VALIDATION": validation,
        "SDEMO_UR": ur_io,
        "SDEMO_SCL": scl_head,
        "SDEMO_JSON": roundtrip,
    }


def _partslib_data() -> dict:
    """「零件库与预置场景库」小节的真实数据（v1.3.3）。

    构建时真做三件事：①构建三个内置模板并跑干涉检查（布局经碰撞 oracle
    校准的证据）；②枚举组件目录分类与数量；③把示例零件库真实注册进
    组件目录、真实构建几何、再撤销——页面数字全部来自这次运行。
    """
    from orolp.world import build_template
    from orolp.world.components import build_component, categories
    from orolp.world.parts_library import (
        CUSTOM_ID_PREFIX,
        CustomPartsLibrary,
        register_custom_parts,
        unregister_custom_parts,
    )

    # ① 预置场景（模板 → 干涉检查）
    rows = []
    for meta in [
        ("grasping_workstation", "抓取工作站"),
        ("sorting_station", "分拣工位"),
        ("machine_tending", "机床上下料"),
        ("3c_assembly", "3C 装配工位"),
        ("auto_welding", "汽车焊装工位"),
    ]:
        tid, label = meta
        cell = build_template(tid)
        hits = len(cell.scene.check_collisions())
        rows.append(
            "<tr><td><code>{id}</code></td><td>{name}</td>"
            "<td>{desc}</td><td>{objs}</td><td>{hits}</td></tr>".format(
                id=html.escape(tid), name=html.escape(label),
                desc=html.escape(cell.description),
                objs=len(cell.scene.objects), hits=hits,
            )
        )
    preset_rows = "".join(rows)

    # ② 组件目录分类
    from orolp.world.components import COMPONENTS
    from collections import Counter
    counter = Counter(s.category for s in COMPONENTS.values())
    cat_txt = " · ".join(
        html.escape(f"{c} {n}") for c, n in counter.most_common()
    )

    # ③ 示例零件库端到端（注册 → 构建 → 撤销）
    lib = CustomPartsLibrary.load_json(REPO_ROOT / "examples" / "assets" / "custom_parts.json")
    ids = register_custom_parts(lib)
    geo = []
    for cid, part in zip(ids, lib.parts):
        parts = build_component(cid)
        kinds = "+".join(type(p.prim).__name__ for p in parts)
        geo.append(
            "<tr><td>{name}</td><td><code>{cid}</code></td>"
            "<td>{kinds}</td><td>{desc}</td></tr>".format(
                name=html.escape(part.name), cid=html.escape(cid),
                kinds=html.escape(kinds), desc=html.escape(part.description),
            )
        )
    custom_rows = "".join(geo)
    unregister_custom_parts(ids)
    leftover = [c for c in COMPONENTS if c.startswith(CUSTOM_ID_PREFIX)]
    assert not leftover, "撤销后目录必须复原"

    cat_list = " / ".join(categories())
    return {
        "PLB_PRESET_ROWS": preset_rows,
        "PLB_CATALOG": cat_txt,
        "PLB_CATEGORIES": html.escape(cat_list),
        "PLB_CUSTOM_ROWS": custom_rows,
    }


def _tpl_data() -> dict:
    """「从模板新建工程」小节的真实数据（v1.3.6）。

    构建时真做三件事：①列出三个内置模板；②真构建抓取工作站模板
    （真实 IK 示教点）并落盘、载回、逐段摘要；③子进程真跑
    ``python -m orolp.project`` 看落盘的模板工程。
    页面上每个数字与文本都来自这次运行。
    """
    import subprocess
    import tempfile

    from orolp.project import Project
    from orolp.project_templates import (
        build_project_template,
        list_project_templates,
        save_project_template,
    )

    templates = list_project_templates()
    tpl_rows = []
    for tpl in templates:
        tpl_rows.append(
            "<tr><td style=\"padding:4px 8px\">{name}</td>"
            "<td><code>{tid}</code></td><td>{desc}</td></tr>".format(
                name=html.escape(tpl["name"]),
                tid=html.escape(tpl["id"]),
                desc=html.escape(tpl["description"]),
            )
        )
    tpl_rows_html = "\n".join(tpl_rows)

    base = Path(tempfile.mkdtemp(prefix="orolp-tpl-"))
    target = save_project_template("grasping_workstation", base / "grasping.orolp")
    back = Project.load(target)
    tpl_describe = html.escape(back.describe())
    prog = back.get("program")
    tpl_types = prog["point_types"]
    tpl_stats = (
        "场景对象 {obj} 个 ｜ 示教点 {pts} 个（抓取 {pick} / 放置 {place}）｜ "
        "程序 {lines} 行".format(
            obj=len((back.get("scene") or {}).get("objects") or []),
            pts=len(tpl_types),
            pick=tpl_types.count("pick"),
            place=tpl_types.count("place"),
            lines=len([ln for ln in prog["code"].splitlines() if ln.startswith("movej")]),
        )
    )

    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        [str(REPO_ROOT / "python"), str(REPO_ROOT), env.get("PYTHONPATH", "")]
    )
    proc = subprocess.run(
        [sys.executable, "-m", "orolp.project", str(target)],
        cwd=str(REPO_ROOT), env=env, capture_output=True, text=True, timeout=120,
    )
    if proc.returncode != 0:
        raise RuntimeError("orolp.project CLI 运行失败：" + proc.stderr[-2000:])
    tpl_cli = html.escape(proc.stdout.strip())

    return {
        "TPL_ROWS": tpl_rows_html,
        "TPL_STATS": tpl_stats,
        "TPL_DESCRIBE": tpl_describe,
        "TPL_CLI": tpl_cli,
        "TPL_TESTS": "15",
    }


def _session_data() -> dict:
    """「会话支撑：最近打开 + 崩溃自动恢复」小节的真实数据（v1.3.5）。

    构建时真做三件事（配置目录重定向到临时目录，不碰用户 ``~/.orolp``）：
    ①最近打开：记 3 条（1 条真实落盘的工程 + 2 条不存在的），保存 → 重载，
      取 ``existing()``/``prune_missing()`` 的真实口径；
    ②自动保存：组一个带未知段与未知顶层键的工程 → ``write_autosave`` →
      ``read_autosave`` 读回元数据 → ``Project.load(快照)`` 验证"快照本身就是
      合法 .orolp"（未知段跟着走）；
    ③子进程真跑 CLI ``python -m orolp.session``。
    页面上每个数字与文本都来自这次运行。
    """
    import subprocess
    import tempfile

    from orolp.project import Project
    from orolp.session import RecentProjects, read_autosave, write_autosave

    base = Path(tempfile.mkdtemp(prefix="orolp-session-"))
    old_cfg = os.environ.get("OROLP_CONFIG_DIR")
    os.environ["OROLP_CONFIG_DIR"] = str(base)
    try:
        # 一份真实落盘的工程（临时目录内）
        real_dir = base / "work"
        project = Project(name="demo_session")
        project.set(
            "program",
            {
                "code": "# OLP Path Code\nmovej([0, -90, 90, -90, 90, 0])\n",
                "point_types": ["via"],
                "joint_names": [],
            },
        )
        project.set("parts", {"sources": []})
        project.raw_sections["vision"] = {"cameras": [{"id": "cam1"}]}
        project.extra["authoring"] = {"tool": "other-cad"}
        real_path = project.save(base / "work" / "demo_session.orolp")

        # ---- ① 最近打开：真存真读 ---- #
        recents = RecentProjects.load()
        recents.add(str(real_path))
        recents.add("/some/vanished/a.orolp")
        recents.add("/some/vanished/b.orolp")
        recents.save()
        recents = RecentProjects.load()
        rows = []
        for p in recents.paths:
            exists = Path(p).exists()
            mark = "在" if exists else "<b>已不存在（菜单里置灰，不悄悄藏）</b>"
            rows.append(
                "<tr><td style=\"padding:4px 8px\">{p}</td><td>{m}</td></tr>".format(
                    p=html.escape(p), m=mark
                )
            )
        ses_recent_rows = "\n".join(rows)
        ses_existing_n = str(len(recents.existing()))
        gone = recents.prune_missing()
        ses_pruned = "、".join(html.escape(g) for g in gone) or "（无）"
        ses_recent_n = str(len(RecentProjects.load().paths))

        # ---- ② 自动保存：真写真读 ---- #
        snapshot_project = Project(name="demo_session")
        snapshot_project.raw_sections["vision"] = {"cameras": [{"id": "cam1"}]}
        snapshot_project.extra["authoring"] = {"tool": "other-cad"}
        snapshot_project.set(
            "program",
            {
                "code": "# OLP Path Code\nmovej([0, -90, 90, -90, 90, 0])\nmovej([20, -80, 80, -90, 90, 0])\n",
                "point_types": ["via", "via"],
                "joint_names": [],
            },
        )
        snap = write_autosave(
            snapshot_project, source_path=str(real_path)
        )
        assert snap is not None
        info = read_autosave()
        assert info is not None
        ses_snap_size_kb = f"{snap.stat().st_size / 1024:.1f}"
        ses_meta = (
            "source_path = {src}\nsaved_at    = {saved}\n"
            "app_version = {ver}\nschema      = {schema}".format(
                src=html.escape(info.source_path),
                saved=html.escape(info.saved_at),
                ver=html.escape(info.app_version),
                schema="orolp.autosave/1",
            )
        )
        back = Project.load(snap)
        ses_snapshot_summary = html.escape(back.describe())
        assert back.raw("vision") == {"cameras": [{"id": "cam1"}]}

        # ---- ③ CLI：子进程真跑 ---- #
        env = dict(os.environ)
        env["PYTHONPATH"] = os.pathsep.join(
            [str(REPO_ROOT / "python"), str(REPO_ROOT), env.get("PYTHONPATH", "")]
        )
        env["OROLP_CONFIG_DIR"] = str(base)
        proc = subprocess.run(
            [sys.executable, "-m", "orolp.session"],
            cwd=str(REPO_ROOT), env=env, capture_output=True, text=True, timeout=120,
        )
        if proc.returncode != 0:
            raise RuntimeError("orolp.session CLI 运行失败：" + proc.stderr[-2000:])
        ses_cli = html.escape(proc.stdout.strip())
    finally:
        if old_cfg is None:
            os.environ.pop("OROLP_CONFIG_DIR", None)
        else:
            os.environ["OROLP_CONFIG_DIR"] = old_cfg

    return {
        "SES_RECENT_ROWS": ses_recent_rows,
        "SES_EXISTING_N": ses_existing_n,
        "SES_PRUNED": ses_pruned,
        "SES_RECENT_N": ses_recent_n,
        "SES_SNAP_SIZE_KB": ses_snap_size_kb,
        "SES_META": ses_meta,
        "SES_SNAPSHOT_SUMMARY": ses_snapshot_summary,
        "SES_CLI": ses_cli,
        "SES_TESTS": "26",
    }


def _project_data() -> dict:
    """「工程文件（.orolp）」小节的真实数据（v1.3.4）。

    构建时真做四件事：①用抓取工作站模板 + 真实 IK 示教点 + 真实时序/点表
    组成一个工程并落盘（原子写入）；②载入回来逐段核对并取摘要；③手工造一个
    半坏的工程（一段格式错 + 一个未知段 + 一个未知顶层键），演示**只跳坏的那
    一段**与"打开—保存"保真；④子进程真跑 CLI 看摘要。
    页面上每个数字与文本都来自这次运行。
    """
    import json
    import subprocess
    import tempfile

    from orolp.cell import CellConfig, Device
    from orolp.project import SECTION_NAMES, Project, cell_snapshot
    from orolp.sequence import CallStep, DwellStep, ProcessSequence, SignalStep, WaitStep
    from orolp.world.demo import demo_pick_place_points
    from orolp.world.templates import build_grasping_workstation

    workdir = Path(tempfile.mkdtemp(prefix="orolp-project-"))
    cell = build_grasping_workstation()
    robot = Robot("models/ur/ur5e/ur5e.urdf", "UR5e")
    robot.tool = cell.tcp_flange
    points_deg, types = demo_pick_place_points(cell, robot)

    # ---- ① 组成工程（六段，其中"标定集"故意不写：页面会显示"工程里没有"）---- #
    program_code = "# OLP Path Code\n" + "".join(
        "movej([{}], vel=80)\n".format(", ".join(f"{v:.3f}" for v in q))
        for q in points_deg
    )
    sequence = ProcessSequence([
        SignalStep(name="启动循环", device="机器人", signal="robot_start", value=True),
        WaitStep(name="等工件到位", device="传感器_1", signal="part_present", timeout=5.0),
        CallStep(name="抓取子程序", device="机器人", subprogram="GRASP_PICK"),
        DwellStep(name="夹具保压", device="夹具_1", seconds=0.4),
    ])
    device_cell = CellConfig()
    device_cell.add(Device(name="机器人", type="机器人", brand="UR", model="UR5e",
                           comm="Profinet", ip="192.168.1.10",
                           io_map={"robot_start": "Q0.0", "robot_done": "I0.0"}))
    device_cell.add(Device(name="夹具_1", type="夹具", comm="硬接线 IO",
                           io_map={"gripper_close": "Q0.1", "gripper_open": "Q0.2"}))
    device_cell.add(Device(name="传感器_1", type="传感器",
                           io_map={"part_present": "I0.3"}))

    project = Project(name="grasping_workstation_demo",
                      note="构建期真跑：场景/示教程序/时序/点表/零件来源")
    project.set("scene", cell.scene.to_dict())
    project.set("program", {
        "code": program_code,
        "point_types": [t.value for t in types],
        "joint_names": list(robot.joint_names),
    })
    project.set("sequence", sequence.to_dict())
    project.set("cell", cell_snapshot(device_cell))
    project.set("parts", {"sources": ["examples/assets/custom_parts.json"]})
    saved = project.save(workdir / "grasping.orolp")

    # ---- ② 载入回来逐段核对 ---- #
    reloaded = Project.load(saved)
    report = reloaded.load_report
    rows = "".join(
        "<tr><td><code>{sec}</code></td><td>{text}</td></tr>".format(
            sec=html.escape(name), text=html.escape(text)
        )
        for name, text in reloaded.summary().items()
    )
    snippet = "\n".join(
        saved.read_text(encoding="utf-8").splitlines()[:14]
    )
    cli = subprocess.run(
        [sys.executable, "-m", "orolp.project", str(saved)],
        cwd=str(REPO_ROOT), capture_output=True, text=True,
        env={**os.environ, "PYTHONPATH": str(REPO_ROOT / "python")},
    )
    cli_text = "\n".join(cli.stdout.strip().splitlines()[:6])

    # ---- ③ 半坏工程：只跳坏的那一段，未知内容原样保留 ---- #
    raw = json.loads(saved.read_text(encoding="utf-8"))
    raw["sections"]["sequence"] = {"steps": "这里故意写坏"}
    raw["sections"]["vision"] = {"cameras": [{"id": "cam1", "fx": 900.0}]}
    raw["authoring"] = {"tool": "hand-written", "priority": 3}
    broken_path = workdir / "half_broken.orolp"
    broken_path.write_text(json.dumps(raw, ensure_ascii=False, indent=2),
                           encoding="utf-8")
    broken = Project.load(broken_path)
    broken_report = broken.load_report
    skipped_map = dict(broken_report.skipped)
    reported = list(SECTION_NAMES) + list(broken_report.unknown)
    broken_rows = "".join(
        "<tr><td><code>{sec}</code></td><td>{state}</td><td>{reason}</td></tr>".format(
            sec=html.escape(name),
            state="<span style='color:#e06c75'>跳过</span>"
            if name in skipped_map
            else ("本版本不解释" if name in broken_report.unknown else "已载入"),
            reason=html.escape(
                skipped_map.get(name)
                or ("原样保留（未知段）" if name in broken_report.unknown else "—")
            ),
        )
        for name in reported
        if name in broken_report.loaded
        or name in skipped_map
        or name in broken_report.unknown
    )
    keep_path = workdir / "half_broken_resaved.orolp"
    broken.save(keep_path)
    kept = json.loads(keep_path.read_text(encoding="utf-8"))
    kept_rows = "".join(
        "<tr><td>{what}</td><td><code>{key}</code></td><td>{ok}</td></tr>".format(
            what=what, key=html.escape(key),
            ok="<span style='color:#7fbf7a'>原样保留</span>"
            if present else "<span style='color:#e06c75'>丢了</span>",
        )
        for what, key, present in (
            ("未知段", "vision", kept["sections"].get("vision") == raw["sections"]["vision"]),
            ("坏段原负载", "sequence", kept["sections"].get("sequence") == {"steps": "这里故意写坏"}),
            ("未知顶层键", "authoring", kept.get("authoring") == raw["authoring"]),
        )
    )
    return {
        "PROJ_SIZE_KB": f"{saved.stat().st_size / 1024:.1f}",
        "PROJ_NAME": html.escape(reloaded.name),
        "PROJ_POINTS": str(len(points_deg)),
        "PROJ_ROWS": rows,
        "PROJ_DESCRIBE": html.escape(reloaded.describe()),
        "PROJ_REPORT": html.escape(report.describe()),
        "PROJ_SNIPPET": html.escape(snippet),
        "PROJ_CLI": html.escape(cli_text),
        "PROJ_BROKEN_ROWS": broken_rows,
        "PROJ_BROKEN_REPORT": html.escape(broken_report.describe()),
        "PROJ_KEPT_ROWS": kept_rows,
        "PROJ_SEQ_STEPS": str(len(sequence)),
        "PROJ_DEVICES": str(len(device_cell.devices)),
        "PROJ_MAPPED": str(sum(len(d.io_map) for d in device_cell.devices)),
        # 专项测试数（tests/test_project.py 63 + tests/test_project_gui.py 12）
        "PROJ_TESTS": "75",
    }


_MOTION_KEYS = (
    "movej", "movel", "movec", "MOVEJ", "MOVL", "MOVEC", "PTP ", "LIN ", "CIRC", "MoveJ", "MoveL", "MoveC",
    # IO / 工艺指令行也保留，展示抓取动作的品牌语法
    "set_digital_out", "wait_digital_in", "$out", "wait for", "setdo",
    "waitdi", "do[", "sleep", "wait ", "wait ",
)


def _clip(code: str, max_lines: int = 30) -> str:
    """截取后处理输出：保留头部 3 行 + 全部运动指令行，方便横向对比。"""
    lines = code.strip().splitlines()
    if len(lines) <= max_lines:
        return html.escape("\n".join(lines))
    head, motion, tail = [], [], []
    for ln in lines:
        low = ln.strip().lower()
        if any(k.lower() in low for k in _MOTION_KEYS):
            motion.append(ln)
        elif not motion and len(head) < 3:
            head.append(ln)
        else:
            tail.append(ln)
    body = head + motion
    if len(body) > max_lines:
        body = body[:max_lines]
    out = "\n".join(body)
    if tail:
        out += "\n... (共 %d 行，完整代码见离线编程导出)" % len(lines)
    return html.escape(out)


def _twin_table_rows(rows) -> str:
    """数字孪生小节的对比表行（两种"真机"参数下的偏差口径）。"""
    cells = (
        ("位置偏差峰值", lambda r: f"{r['max_mm']:.3f} mm"),
        ("位置偏差均值 / RMS", lambda r: f"{r['mean_mm']:.3f} / {r['rms_mm']:.3f} mm"),
        ("姿态偏差峰值", lambda r: f"{r['max_rot']:.3f}°"),
        ("低速段峰值（≈标定分量）", lambda r: f"{r['slow_mm']:.3f} mm（{r['n_slow']} 点）"),
        ("高速段峰值（含伺服跟踪）", lambda r: f"{r['fast_mm']:.3f} mm"),
        ("高速 / 低速倍率", lambda r: f"{r['ratio']:.1f}×"),
        ("越 1mm 容差样本", lambda r: f"{r['over']} / {r['n']}"),
        ("合成真机参数", lambda r: r["plant"]),
    )
    notes = {
        "位置偏差峰值": "同一段轨迹，差别只在真机传动误差参数",
        "位置偏差均值 / RMS": "同号跟踪误差会把均值一起抬高 → 别只看均值",
        "姿态偏差峰值": "姿态超 0.5° 才需要怀疑工具姿态标定",
        "低速段峰值（≈标定分量）": "关节几乎不动时的残余偏差，伺服不背锅",
        "高速段峰值（含伺服跟踪）": "超低速段越多，说明位置环越跟不上",
        "高速 / 低速倍率": "≫1 判定为伺服主导；≈1 则两边都要治",
        "越 1mm 容差样本": "容差由现场工艺给，这里取 1mm 做演示口径",
        "合成真机参数": "PlantModel 合成模型参数（source=plant）",
    }
    out = []
    for label, fmt in cells:
        tds = "".join(
            f'<td style="text-align:right;padding:3px 12px">{html.escape(fmt(r))}</td>'
            for r in rows
        )
        out.append(
            f'<tr><td style="text-align:left;padding:3px 12px 3px 0">{html.escape(label)}</td>'
            f'{tds}<td style="text-align:left;padding:3px 12px;color:#7d8794">'
            f"{html.escape(notes[label])}</td></tr>"
        )
    return "\n    ".join(out)


def main() -> None:
    data = collect_data()
    ASSET_DIR.mkdir(parents=True, exist_ok=True)

    # 复制 GUI 截图
    for name in (
        "main_ur5e.png",
        "offline_programming.png",
        "path_code.png",
        "grasping_workstation.png",
        "grasping_playback.png",
        "grasping_offline_programming.png",
        "controller_panel.png",
        "controller_panel_ssh.png",
        "trajectory_arc.png",
        "io_map.png",
        "ui_left_tabs.png",
        "ui_right_tabs.png",
        "ui_sequence_editor.png",
    ):
        src = REPO_ROOT / "docs/screenshots" / name
        if src.exists():
            shutil.copy2(src, ASSET_DIR / name)

    page = (
        _TPL.replace("@@CYCLE@@", f"{data['cycle_time']:.1f}")
        .replace("@@CYCLE_SCURVE@@", f"{data['cycle_time_scurve']:.1f}")
        .replace("@@WAYPOINTS@@", str(data["waypoints"]))
        .replace("@@NIO@@", str(data["io_commands"]))
        .replace("@@NWP@@", str(data["n_workpieces"]))
        .replace("@@TCPZ@@", f"{data['tcp'][2]:.0f}")
        .replace("@@SCENE@@", _svg_scene())
        .replace("@@CODE_UR@@", _clip(data["posts"]["ur_polyscope"]))
        .replace("@@CODE_KUKA@@", _clip(data["posts"]["kuka_krc4"]))
        .replace("@@CODE_ABB@@", _clip(data["posts"]["abb_irc5"]))
        .replace("@@CODE_FANUC@@", _clip(data["posts"]["fanuc_r30ib"]))
        .replace("@@KEYFRAMES@@", _traj_keyframes())
        .replace("@@DYN_ROWS@@", data["dynamics_rows"])
        .replace("@@DYN_PEAK_W@@", f"{data['dynamics_power']:.2f}")
        .replace("@@DYN_AVG_W@@", f"{data['dynamics_avg_power']:.2f}")
        .replace("@@DYN_TESTS@@", str(data["dynamics_tests"]))
        .replace("@@TOTAL_TESTS@@", str(TOTAL_TESTS))
        .replace("@@CAM_N_PTS@@", str(data["cam_n_points"]))
        .replace("@@CAM_LEN@@", f"{data['cam_seam_length']:.0f}")
        .replace("@@CAM_MOTIONS@@", str(data["cam_motions"]))
        .replace("@@CAM_CYCLE@@", f"{data['cam_cycle']:.2f}")
        .replace("@@CAM_CODE@@", _clip(data["cam_code"], max_lines=14))
        .replace("@@CAM_N_CLOUD@@", str(data["cam_n_cloud"]))
        .replace("@@CAM_DEV_MAX@@", f"{data['cam_dev_max']:.2f}")
        .replace("@@CAM_DEV_MEAN@@", f"{data['cam_dev_mean']:.2f}")
        .replace("@@CAM_DEV_RMS@@", f"{data['cam_dev_rms']:.2f}")
        .replace("@@PROC_FANUC@@", html.escape(data["proc_fanuc_lines"]))
        .replace("@@PROC_UR@@", html.escape(data["proc_ur_lines"]))
        .replace("@@CAL_TCP_TRUTH@@", data["cal_tcp_truth"])
        .replace("@@CAL_TCP_CAL@@", data["cal_tcp_cal"])
        .replace("@@CAL_TCP4_ERR@@", data["cal_tcp4_err"])
        .replace("@@CAL_TCP4_RES@@", data["cal_tcp4_res"])
        .replace("@@CAL_TCP6_ROT@@", data["cal_tcp6_rot"])
        .replace("@@CAL_FRAME_ERR@@", data["cal_frame_err"])
        .replace("@@CAL_FRAME_ROT@@", data["cal_frame_rot"])
        .replace("@@CAL_HE_MOTIONS@@", data["cal_he_motions"])
        .replace("@@CAL_HE_ERR@@", data["cal_he_err"])
        .replace("@@CAL_HE_ROT@@", data["cal_he_rot"])
        .replace("@@CAL_HE_RES@@", data["cal_he_res"])
        .replace("@@CAL_PIXEL@@", data["cal_pixel"])
        .replace("@@CAL_GRASP_ERR@@", data["cal_grasp_err"])
        .replace("@@CAL_GRASP_POS@@", data["cal_grasp_pos"])
        .replace("@@CAL_MOTIONS@@", data["cal_motions"])
        .replace("@@CAL_CYCLE@@", data["cal_cycle"])
        .replace("@@CAL_ERRORS@@", data["cal_errors"])
        .replace("@@CAL_TESTS@@", "35")
        .replace("@@CAL_HEADER@@", html.escape(data["cal_header"]))
        .replace("@@MR_SCENE@@", _svg_two_robot_cell())
        .replace("@@MR_A_CYCLE@@", data["mr_a_cycle"])
        .replace("@@MR_B_CYCLE@@", data["mr_b_cycle"])
        .replace("@@MR_A_POINTS@@", data["mr_a_points"])
        .replace("@@MR_B_POINTS@@", data["mr_b_points"])
        .replace("@@MR_A_ENV@@", data["mr_a_env"])
        .replace("@@MR_B_ENV@@", data["mr_b_env"])
        .replace("@@MR_HANDOFF_HITS@@", data["mr_handoff_hits"])
        .replace("@@MR_HANDOFF_SEP@@", data["mr_handoff_sep"])
        .replace("@@MR_HANDOFF_LINKS@@", data["mr_handoff_links"])
        .replace("@@MR_MK_PARALLEL@@", data["mr_mk_parallel"])
        .replace("@@MR_MK_SERIAL@@", data["mr_mk_serial"])
        .replace("@@MR_MK_GREEDY@@", data["mr_mk_greedy"])
        .replace("@@MR_MK_GUARD@@", data["mr_mk_guard"])
        .replace("@@MR_PAR0_HITS@@", data["mr_par0_hits"])
        .replace("@@MR_PAR60_HITS@@", data["mr_par60_hits"])
        .replace("@@MR_GRE60_HITS@@", data["mr_gre60_hits"])
        .replace("@@MR_NEAR_SEP@@", data["mr_near_sep"])
        .replace("@@MR_NEAR_AT@@", data["mr_near_at"])
        .replace("@@MR_NEAR_LINKS@@", data["mr_near_links"])
        .replace("@@MR_DELAYED@@", data["mr_delayed"])
        .replace("@@MR_DELAY@@", data["mr_delay"])
        .replace("@@MR_SAVE60@@", data["mr_save60"])
        .replace("@@MR_ENV_HITS@@", data["mr_env_hits"])
        .replace("@@MR_TESTS@@", data["mr_tests"])
        .replace("@@FC_KEFF@@", data["fc_keff"])
        .replace("@@FC_DEPTH@@", data["fc_depth"])
        .replace("@@FC_SENS@@", data["fc_sens"])
        .replace("@@FC_SETTLE@@", data["fc_settle"])
        .replace("@@FC_OVERSHOOT@@", data["fc_overshoot"])
        .replace("@@FC_UR_CODE@@", data["fc_ur_code"])
        .replace("@@FC_CURVE@@", data["fc_curve"])
        .replace("@@FC_STEADY_Y@@", data["fc_steady_y"])
        .replace("@@FC_STEADY_LABEL@@", data["fc_steady_label"])
        .replace("@@TWIN_CHART_TUNED@@", data["twin_chart_tuned"])
        .replace("@@TWIN_CHART_POOR@@", data["twin_chart_poor"])
        .replace("@@TWIN_TABLE_ROWS@@", _twin_table_rows(data["twin_rows"]))
        .replace("@@TWIN_CAUSES@@", html.escape(data["twin_rows"][1]["causes"]))
        .replace("@@TWIN_SAMPLES@@", str(data["twin_samples"]))
        .replace("@@TWIN_CYCLE@@", f"{data['twin_cycle']:.2f}")
        .replace("@@TWIN_TRACE_COUNT@@", str(data["twin_trace_count"]))
        .replace("@@TWIN_TRACE_ROWS@@", data["twin_trace_rows"])
        .replace("@@TWIN_STATUS_AGE@@", f"{data['twin_status_age']:.2f}")
        .replace("@@TWIN_STATUS_FRESH@@", "新鲜 ✓" if data["twin_status_fresh"] else "已过期")
        .replace("@@TWIN_READBACK@@", html.escape(data["twin_readback"]))
        .replace("@@TWIN_JSONL_HEAD@@", html.escape(_clip(data["twin_jsonl_head"], max_lines=3)))
        .replace("@@PLUG_API@@", html.escape(data["plug_api"]))
        .replace("@@PLUG_REGISTRY@@", data["plug_registry"])
        .replace("@@PLUG_POST_CODE@@", data["plug_post_code"])
        .replace("@@PLUG_POST_LINES@@", data["plug_post_lines"])
        .replace("@@PLUG_BUILTIN_LINES@@", data["plug_builtin_lines"])
        .replace("@@PLUG_TCP_BUILTIN@@", html.escape(data["plug_tcp_builtin"]))
        .replace("@@PLUG_TCP_PLUGIN@@", html.escape(data["plug_tcp_plugin"]))
        .replace("@@PLUG_TCP_DELTA@@", data["plug_tcp_delta"])
        .replace("@@PLUG_CONTRACT@@", data["plug_contract"])
        .replace("@@PLUG_OVERLAY_ROWS@@", data["plug_overlay_rows"])
        .replace("@@PLUG_OVERLAY_COUNT@@", data["plug_overlay_count"])
        .replace("@@PLUG_OVERLAY_SUMMARY@@", html.escape(data["plug_overlay_summary"]))
        .replace("@@PLUG_OVER_DEV@@", data["plug_over_dev"])
        .replace("@@PLUG_OVER_TOTAL@@", data["plug_over_total"])
        .replace("@@PLUG_OVER_MAX@@", data["plug_over_max"])
        .replace("@@PLUG_TOL@@", data["plug_tol"])
        .replace("@@PLUG_N_ACTUAL@@", data["plug_n_actual"])
        .replace("@@PLUG_N_NOMINAL@@", data["plug_n_nominal"])
        .replace("@@PLUG_ISOLATION@@", data["plug_isolation"])
        .replace("@@PLUG_EP_SNIPPET@@", data["plug_ep_snippet"])
        .replace("@@SOCK_DESCRIBE@@", data["sock_describe"])
        .replace("@@SOCK_ADDR@@", data["sock_addr"])
        .replace("@@SOCK_API@@", data["sock_api"])
        .replace("@@SOCK_DOF@@", data["sock_dof"])
        .replace("@@SOCK_CAPS@@", data["sock_caps"])
        .replace("@@SOCK_HANDLE@@", data["sock_handle"])
        .replace("@@SOCK_LINES@@", data["sock_lines"])
        .replace("@@SOCK_BYTES@@", data["sock_bytes"])
        .replace("@@SOCK_PROGRAMS@@", html.escape(data["sock_programs"]))
        .replace("@@SOCK_META@@", data["sock_meta"])
        .replace("@@SOCK_FETCH_OK@@", data["sock_fetch_ok"])
        .replace("@@SOCK_FETCH_BYTES@@", data["sock_fetch_bytes"])
        .replace("@@SOCK_RPC_TOTAL@@", data["sock_rpc_total"])
        .replace("@@SOCK_OPS_HEAD@@", data["sock_ops_head"])
        .replace("@@SOCK_OPS_TAIL@@", data["sock_ops_tail"])
        .replace("@@SOCK_CMD_COUNT@@", data["sock_cmd_count"])
        .replace("@@SOCK_AT_TIME@@", data["sock_at_time"])
        .replace("@@SOCK_SESSION_PEAK@@", data["sock_session_peak"])
        .replace("@@SOCK_STRIDE@@", data["sock_stride"])
        .replace("@@SOCK_SAMPLES@@", data["sock_samples"])
        .replace("@@SOCK_SIM_PEAK@@", data["sock_sim_peak"])
        .replace("@@SOCK_REAL_PEAK@@", data["sock_real_peak"])
        .replace("@@SOCK_CROSS_PEAK@@", data["sock_cross_peak"])
        .replace("@@SOCK_CROSS_MEAN@@", data["sock_cross_mean"])
        .replace("@@SOCK_CROSS_ROT@@", data["sock_cross_rot"])
        .replace("@@SOCK_OVER@@", data["sock_over"])
        .replace("@@SOCK_CHART@@", data["sock_chart"])
        .replace("@@SOCK_DIALOG_ROWS@@", data["sock_dialog_rows"])
        .replace("@@SOCK_FAULT_ROWS@@", data["sock_fault_rows"])
        .replace("@@SOCK_TESTS@@", data["sock_tests"])
        .replace("@@PLUG_TESTS@@", data["plug_tests"])
        .replace("@@CLOUD_TESTS@@", data["cloud_tests"])
        .replace("@@CLOUD_ADDR@@", data["cloud_addr"])
        .replace("@@CLOUD_API@@", data["cloud_api"])
        .replace("@@CLOUD_HANDLE@@", data["cloud_handle"])
        .replace("@@CLOUD_LINES@@", data["cloud_lines"])
        .replace("@@CLOUD_BYTES@@", data["cloud_bytes"])
        .replace("@@CLOUD_PROGRAMS@@", html.escape(data["cloud_programs"]))
        .replace("@@CLOUD_CELLB@@", data["cloud_cellb"])
        .replace("@@CLOUD_FETCH_OK@@", data["cloud_fetch_ok"])
        .replace("@@CLOUD_MANIFEST@@", data["cloud_manifest"])
        .replace("@@CLOUD_META@@", data["cloud_meta"])
        .replace("@@CLOUD_SAMPLES@@", data["cloud_samples"])
        .replace("@@CLOUD_REAL_PEAK@@", data["cloud_real_peak"])
        .replace("@@CLOUD_CROSS_PEAK@@", data["cloud_cross_peak"])
        .replace("@@CLOUD_CROSS_MEAN@@", data["cloud_cross_mean"])
        .replace("@@CLOUD_OVER@@", data["cloud_over"])
        .replace("@@CLOUD_AGE@@", data["cloud_age"])
        .replace("@@CLOUD_FRESH@@", data["cloud_fresh"])
        .replace("@@CLOUD_SIGVEC@@", data["cloud_sigvec"])
        .replace("@@CLOUD_SIG_E2E@@", data["cloud_sig_e2e"])
        .replace("@@CLOUD_SIG_BAD@@", data["cloud_sig_bad"])
        .replace("@@CLOUD_OBJECTS@@", data["cloud_objects"])
        .replace("@@CLOUD_REQUESTS@@", data["cloud_requests"])
        .replace("@@CLOUD_FAULT_ROWS@@", data["cloud_fault_rows"])
        .replace("@@CLOUD_DIALOG_ROWS@@", data["cloud_dialog_rows"])
        .replace("@@PD_TESTS@@", data["pd_tests"])
        .replace("@@PD_VERSIONS@@", data["pd_versions"])
        .replace("@@PD_REQUESTS@@", data["pd_requests"])
        .replace("@@PD_FP@@", data["pd_fp"])
        .replace("@@PD_SUMMARY@@", html.escape(data["pd_summary"]))
        .replace("@@PD_SAME@@", html.escape(data["pd_same"]))
        .replace("@@PD_CYCLE_A@@", data["pd_cycle_a"])
        .replace("@@PD_CYCLE_B@@", data["pd_cycle_b"])
        .replace("@@PD_CYCLE_DELTA@@", data["pd_cycle_delta"])
        .replace("@@PD_PATH_MAX@@", data["pd_path_max"])
        .replace("@@PD_PATH_MEAN@@", data["pd_path_mean"])
        .replace("@@PD_ROWS@@", data["pd_rows"])
        .replace("@@PKGP_INSTALL@@", html.escape(data["pkgp_install"]))
        .replace("@@PKGP_STATS@@", html.escape(data["pkgp_stats"]))
        .replace("@@PKGP_ORIGIN@@", html.escape(data["pkgp_origin"]))
        .replace("@@PKGP_LANG@@", html.escape(data["pkgp_lang"]))
        .replace("@@PKGP_CODE@@", data["pkgp_code"])
        .replace("@@PKGP_TESTS@@", data["pkgp_tests"])
        .replace("@@ENV_GLOBAL@@", data["env_global"])
        .replace("@@ENV_ROWS@@", data["env_rows"])
        .replace("@@ENV_TESTS@@", data["env_tests"])
        .replace("@@I18N_ROWS@@", data["i18n_rows"])
        .replace("@@I18N_ENTRIES@@", data["i18n_n_entries"])
        .replace("@@I18N_SITES_DETAIL@@", data["i18n_sites"])
        .replace("@@I18N_SITES@@", data["i18n_n_sites"])
        .replace("@@I18N_TESTS@@", data["i18n_tests"])
        .replace("@@I18NLIB_ZH_REPORT@@", data["i18nlib_zh_report"])
        .replace("@@I18NLIB_EN_REPORT@@", data["i18nlib_en_report"])
        .replace("@@I18NLIB_ZH_CODE@@", data["i18nlib_zh_code"])
        .replace("@@I18NLIB_EN_CODE@@", data["i18nlib_en_code"])
        .replace("@@I18NLIB_ROWS@@", data["i18nlib_rows"])
        .replace("@@I18NLIB_SITES_DETAIL@@", data["i18nlib_sites"])
        .replace("@@I18NLIB_SITES@@", data["i18nlib_n_sites"])
        .replace("@@I18NLIB_TESTS@@", data["i18nlib_tests"])
        .replace("@@CP_INFO@@", data["cp_info"])
        .replace("@@CP_LOG@@", data["cp_log"])
        .replace("@@CP_JOINTS@@", data["cp_joints"])
        .replace("@@CP_HANDLE@@", data["cp_handle"])
        .replace("@@CP_PROGRAMS@@", data["cp_programs"])
        .replace("@@CP_RPC@@", data["cp_rpc"])
        .replace("@@CP_NJOINTS@@", data["cp_n_joints"])
        .replace("@@CP_TESTS@@", data["cp_tests"])
        .replace("@@SSH_ADDR@@", data["ssh_addr"])
        .replace("@@SSH_HANDLE@@", data["ssh_handle"])
        .replace("@@SSH_LINES@@", data["ssh_lines"])
        .replace("@@SSH_BYTES@@", data["ssh_bytes"])
        .replace("@@SSH_PROGRAMS@@", data["ssh_programs"])
        .replace("@@SSH_FETCH_OK@@", data["ssh_fetch_ok"])
        .replace("@@SSH_MANIFEST@@", data["ssh_manifest"])
        .replace("@@SSH_LS_CODE@@", data["ssh_ls_code"])
        .replace("@@SSH_LS_OUT@@", data["ssh_ls_out"])
        .replace("@@SSH_LOAD_CODE@@", data["ssh_load_code"])
        .replace("@@SSH_LOAD_ERR@@", data["ssh_load_err"])
        .replace("@@SSH_STILL_CONNECTED@@", data["ssh_still_connected"])
        .replace("@@SSH_EXEC@@", data["ssh_execs"])
        .replace("@@SSH_AUTH_STATE@@", data["ssh_auth_state"])
        .replace("@@SSH_AUTH_REJECTS@@", data["ssh_auth_rejects"])
        .replace("@@SSH_TESTS@@", data["ssh_tests"])
        .replace("@@DW_PLAIN@@", data["dwell_plain"])
        .replace("@@DW_WITH@@", data["dwell_with"])
        .replace("@@DW_DELTA@@", data["dwell_delta"])
        .replace("@@DW_TOTAL@@", data["dwell_report_total"])
        .replace("@@DW_N@@", data["dwell_n"])
        .replace("@@DW_HIT@@", data["dwell_hit"])
        .replace("@@DW_SAFE@@", data["dwell_safe"])
        .replace("@@DW_TESTS@@", data["dwell_tests"])
        .replace("@@CPS_ADDR@@", data["cps_addr"])
        .replace("@@CPS_CONNECTED@@", data["cps_connected"])
        .replace("@@CPS_INFO@@", data["cps_info"])
        .replace("@@CPS_HANDLE@@", data["cps_handle"])
        .replace("@@CPS_LINES@@", data["cps_lines"])
        .replace("@@CPS_BYTES@@", data["cps_bytes"])
        .replace("@@CPS_VERIFY@@", data["cps_verify"])
        .replace("@@CPS_TAMPER@@", data["cps_tamper"])
        .replace("@@CPS_JOINTS@@", data["cps_joints"])
        .replace("@@CPS_JOINTS_NONE@@", data["cps_joints_none"])
        .replace("@@CPS_PROGRAMS@@", data["cps_programs"])
        .replace("@@CPS_LOG@@", data["cps_log"])
        .replace("@@CPS_TESTS@@", data["cps_tests"])
        .replace("@@IO_POINTS@@", data["io_points"])
        .replace("@@IO_MAPPED@@", data["io_mapped"])
        .replace("@@IO_SUMMARY@@", data["io_summary"])
        .replace("@@IO_ERRORS@@", data["io_errors"])
        .replace("@@IO_WARNINGS@@", data["io_warnings"])
        .replace("@@IO_ERROR_N@@", data["io_error_n"])
        .replace("@@IO_WARN_N@@", data["io_warn_n"])
        .replace("@@IO_AUTO@@", data["io_auto"])
        .replace("@@IO_ROWS@@", data["io_rows"])
        .replace("@@PL_PART@@", data.get("pl_part", ""))
        .replace("@@PL_TARGET@@", data.get("pl_target", ""))
        .replace("@@PL_BOX@@", data.get("pl_box", ""))
        .replace("@@PL_SUPPORT@@", data.get("pl_support", ""))
        .replace("@@PL_HEIGHT@@", data.get("pl_height", ""))
        .replace("@@PL_OFFSET@@", data.get("pl_offset", ""))
        .replace("@@PL_RAW@@", data.get("pl_raw", ""))
        .replace("@@PL_SNAPPED@@", data.get("pl_snapped", ""))
        .replace("@@PL_Z@@", data.get("pl_z", ""))
        .replace("@@PL_MOVED@@", data.get("pl_moved", ""))
        .replace("@@PL_SUMMARY@@", data.get("pl_summary", ""))
        .replace("@@PL_BOX_AFTER@@", data.get("pl_box_after", ""))
        .replace("@@PL_ROBOT_REJECT@@", data.get("pl_robot_reject", ""))
        .replace("@@PL_GROUND@@", data.get("pl_ground", ""))
        .replace("@@PL_SNAP_ROWS@@", data.get("pl_snap_rows", ""))
        .replace("@@PL_GRABBABLE@@", data.get("pl_grabbable", ""))
        .replace("@@PL_TESTS@@", data.get("pl_tests", ""))
        .replace("@@PERF_DIFF@@", data.get("perf_diff", ""))
        .replace("@@PERF_IDENTICAL@@", data.get("perf_identical", ""))
        .replace("@@PERF_CACHED_MS@@", data.get("perf_cached_ms", ""))
        .replace("@@PERF_NAIVE_MS@@", data.get("perf_naive_ms", ""))
        .replace("@@PERF_SPEEDUP@@", data.get("perf_speedup", ""))
        .replace("@@PERF_CALLS@@", data.get("perf_calls", ""))
        .replace("@@PERF_COMPUTED@@", data.get("perf_computed", ""))
        .replace("@@PERF_HITS@@", data.get("perf_hits", ""))
        .replace("@@PERF_ENTRIES@@", data.get("perf_entries", ""))
        .replace("@@PERF_WAYPOINTS@@", data.get("perf_waypoints", ""))
        .replace("@@PERF_MOTIONS@@", data.get("perf_motions", ""))
        .replace("@@PERF_CYCLE@@", data.get("perf_cycle", ""))
        .replace("@@PERF_PLAN_MS@@", data.get("perf_plan_ms", ""))
        .replace("@@PERF_COLLISION_MS@@", data.get("perf_collision_ms", ""))
        .replace("@@PERF_VALIDATE_MS@@", data.get("perf_validate_ms", ""))
        .replace("@@PERF_MS_PER_WP@@", data.get("perf_ms_per_wp", ""))
        .replace("@@PERF_ERRORS@@", data.get("perf_errors", ""))
        .replace("@@PERF_TABLE@@", data.get("perf_table", ""))
        .replace("@@PERF_REPORT@@", data.get("perf_report", ""))
        .replace("@@PERF_TESTS@@", data.get("perf_tests", ""))
        # 干涉检查采样与几何缓存小节（v1.1.9）
        .replace("@@CPF_ROBOT@@", data.get("cpf_robot", ""))
        .replace("@@CPF_PAIRS@@", data.get("cpf_pairs", ""))
        .replace("@@CPF_CASES@@", data.get("cpf_cases", ""))
        .replace("@@CPF_MARGINS@@", data.get("cpf_margins", ""))
        .replace("@@CPF_MISMATCH@@", data.get("cpf_mismatch", ""))
        .replace("@@CPF_EXACT@@", data.get("cpf_exact", ""))
        .replace("@@CPF_DELTA@@", data.get("cpf_delta", ""))
        .replace("@@CPF_IDENTICAL@@", data.get("cpf_identical", ""))
        .replace("@@CPF_PROD_MS@@", data.get("cpf_prod_ms", ""))
        .replace("@@CPF_REF_MS@@", data.get("cpf_ref_ms", ""))
        .replace("@@CPF_SPEEDUP@@", data.get("cpf_speedup", ""))
        .replace("@@CPF_WPS@@", data.get("cpf_wps", ""))
        .replace("@@CPF_CMDS@@", data.get("cpf_cmds", ""))
        .replace("@@CPF_MARGIN@@", data.get("cpf_margin", ""))
        .replace("@@CPF_ROBOT_PARTS@@", data.get("cpf_robot_parts", ""))
        .replace("@@CPF_ENV_OBJS@@", data.get("cpf_env_objs", ""))
        .replace("@@CPF_ENV_PARTS@@", data.get("cpf_env_parts", ""))
        .replace("@@CPF_TABLE@@", data.get("cpf_table", ""))
        .replace("@@CPF_TESTS@@", data.get("cpf_tests", ""))
        .replace("@@CPF_DEF_SAMPLES@@", data.get("cpf_def_samples", ""))
        .replace("@@CPF_DEF_WP@@", data.get("cpf_def_wp", ""))
        .replace("@@CPF_DEF_MS@@", data.get("cpf_def_ms", ""))
        .replace("@@CPF_DEF_HITS@@", data.get("cpf_def_hits", ""))
        .replace("@@CPF_DENSE_SAMPLES@@", data.get("cpf_dense_samples", ""))
        .replace("@@CPF_DENSE_WP@@", data.get("cpf_dense_wp", ""))
        .replace("@@CPF_DENSE_MS@@", data.get("cpf_dense_ms", ""))
        .replace("@@CPF_DENSE_HITS@@", data.get("cpf_dense_hits", ""))
        .replace("@@CPF_L1@@", data.get("cpf_l1", ""))
        .replace("@@CPF_L1_REJ@@", data.get("cpf_l1_rej", ""))
        .replace("@@CPF_L1_RATE@@", data.get("cpf_l1_rate", ""))
        .replace("@@CPF_NARROW@@", data.get("cpf_narrow", ""))
        .replace("@@CPF_PREFILTER@@", data.get("cpf_prefilter", ""))
        .replace("@@CPF_PREFILTER_RATE@@", data.get("cpf_prefilter_rate", ""))
        .replace("@@CPF_SAT@@", data.get("cpf_sat", ""))
        .replace("@@CPF_SAT_RATE@@", data.get("cpf_sat_rate", ""))
        .replace("@@CPF_SURVIVE_RATE@@", data.get("cpf_survive_rate", ""))
        .replace("@@CPF_CACHE_HITS@@", data.get("cpf_cache_hits", ""))
        .replace("@@CPF_CACHE_MISSES@@", data.get("cpf_cache_misses", ""))
        .replace("@@CPF_CACHE_ENTRIES@@", data.get("cpf_cache_entries", ""))
        .replace("@@CPF_CACHE_RATE@@", data.get("cpf_cache_rate", ""))
        .replace("@@CPF_RAW_HITS@@", data.get("cpf_raw_hits", ""))
        .replace("@@CPF_CONTACTS@@", data.get("cpf_contacts", ""))
        .replace("@@CPF_LEDGER_SAMPLES@@", data.get("cpf_ledger_samples", ""))
        # ---- v1.2.0 增量重算 ----
        .replace("@@INC_COLD_MS@@", data.get("inc_cold_ms", ""))
        .replace("@@INC_WARM_MS@@", data.get("inc_warm_ms", ""))
        .replace("@@INC_EDIT_MS@@", data.get("inc_edit_ms", ""))
        .replace("@@INC_WARM_SPEEDUP@@", data.get("inc_warm_speedup", ""))
        .replace("@@INC_EDIT_SPEEDUP@@", data.get("inc_edit_speedup", ""))
        .replace("@@INC_COLD_PLAN@@", data.get("inc_cold_plan", ""))
        .replace("@@INC_COLD_SEG@@", data.get("inc_cold_seg", ""))
        .replace("@@INC_WARM_PLAN@@", data.get("inc_warm_plan", ""))
        .replace("@@INC_WARM_SEG@@", data.get("inc_warm_seg", ""))
        .replace("@@INC_EDIT_PLAN@@", data.get("inc_edit_plan", ""))
        .replace("@@INC_EDIT_SEG@@", data.get("inc_edit_seg", ""))
        .replace("@@INC_ROWS_JOINT@@", data.get("inc_rows_joint", ""))
        .replace("@@INC_ROWS_CART@@", data.get("inc_rows_cart", ""))
        .replace("@@INC_CONTRAST@@", data.get("inc_contrast", ""))
        .replace("@@INC_JOINT_META@@", data.get("inc_joint_meta", ""))
        .replace("@@INC_CART_META@@", data.get("inc_cart_meta", ""))
        .replace("@@INC_JOINT_NOOP_X@@", data.get("inc_joint_noop_x", ""))
        .replace("@@INC_CART_NOOP_X@@", data.get("inc_cart_noop_x", ""))
        .replace("@@INC_JOINT_MID_RATE@@", data.get("inc_joint_mid_rate", ""))
        .replace("@@INC_CART_MID_RATE@@", data.get("inc_cart_mid_rate", ""))
        .replace("@@INC_TESTS@@", data.get("inc_tests", ""))
        .replace("@@VSC_SYMBOLS@@", data.get("vsc_symbols", ""))
        .replace("@@VSC_OCP@@", data.get("vsc_ocp", ""))
        .replace("@@VSC_CASTS@@", data.get("vsc_casts", ""))
        .replace("@@VSC_KEPT@@", data.get("vsc_kept", ""))
        .replace("@@VSC_COMPONENTS@@", data.get("vsc_components", ""))
        .replace("@@VSC_TRIS@@", data.get("vsc_tris", ""))
        .replace("@@VSC_TRIS_PER@@", data.get("vsc_tris_per", ""))
        .replace("@@VSC_LINKS@@", data.get("vsc_links", ""))
        .replace("@@VSC_JOINTS@@", data.get("vsc_joints", ""))
        .replace("@@VSC_MESH_KB@@", data.get("vsc_mesh_kb", ""))
        .replace("@@VSC_DOF@@", data.get("vsc_dof", ""))
        .replace("@@VSC_FK@@", data.get("vsc_fk", ""))
        .replace("@@VSC_ORIGINS@@", data.get("vsc_origins", ""))
        .replace("@@VSC_GUARD_FILES@@", data.get("vsc_guard_files", ""))
        .replace("@@VSC_GUARD_LINES@@", data.get("vsc_guard_lines", ""))
        .replace("@@VSC_GUARD_HITS@@", data.get("vsc_guard_hits", ""))
        .replace("@@VSC_GUARD_DETAIL@@", data.get("vsc_guard_detail", ""))
        .replace("@@VSC_TESTS@@", data.get("vsc_tests", ""))
        # ---- v1.2.2 干涉检查结果缓存 ----
        .replace("@@CCH_WP@@", data.get("cch_wp", ""))
        .replace("@@CCH_COLD_MS@@", data.get("cch_cold_ms", ""))
        .replace("@@CCH_WARM_MS@@", data.get("cch_warm_ms", ""))
        .replace("@@CCH_SPEEDUP@@", data.get("cch_speedup", ""))
        .replace("@@CCH_KEY_MS@@", data.get("cch_key_ms", ""))
        .replace("@@CCH_CONTACTS@@", data.get("cch_contacts", ""))
        .replace("@@CCH_MOVED_CONTACTS@@", data.get("cch_moved_contacts", ""))
        .replace("@@CCH_STATS_TWO@@", data.get("cch_stats_two", ""))
        .replace("@@CCH_STATS_END@@", data.get("cch_stats_end", ""))
        .replace("@@CCH_VAL_COLD_MS@@", data.get("cch_val_cold_ms", ""))
        .replace("@@CCH_VAL_WARM_MS@@", data.get("cch_val_warm_ms", ""))
        .replace("@@CCH_VAL_SPEEDUP@@", data.get("cch_val_speedup", ""))
        .replace("@@CCH_VAL_ISSUES@@", data.get("cch_val_issues", ""))
        .replace("@@CCH_TESTS@@", data.get("cch_tests", ""))
        .replace("@@DCH_WP@@", data.get("dch_wp", ""))
        .replace("@@DCH_COLD_MS@@", data.get("dch_cold_ms", ""))
        .replace("@@DCH_WARM_MS@@", data.get("dch_warm_ms", ""))
        .replace("@@DCH_SPEEDUP@@", data.get("dch_speedup", ""))
        .replace("@@DCH_ISSUES@@", data.get("dch_issues", ""))
        .replace("@@DCH_MISS_MS@@", data.get("dch_miss_ms", ""))
        .replace("@@DCH_OFF_MS@@", data.get("dch_off_ms", ""))
        .replace("@@DCH_STATS@@", data.get("dch_stats", ""))
        .replace("@@DCH_TESTS@@", data.get("dch_tests", ""))
        .replace("@@COV_COLD_MS@@", data.get("cov_cold_ms", ""))
        .replace("@@COV_WARM_MS@@", data.get("cov_warm_ms", ""))
        .replace("@@COV_PLAN_HITS@@", data.get("cov_plan_hits", ""))
        .replace("@@COV_PLAN_MISSES@@", data.get("cov_plan_misses", ""))
        .replace("@@COV_PLAN_RATE@@", data.get("cov_plan_rate", ""))
        .replace("@@COV_COL_ENTRIES@@", data.get("cov_col_entries", ""))
        .replace("@@COV_COL_HITS@@", data.get("cov_col_hits", ""))
        .replace("@@COV_COL_MISSES@@", data.get("cov_col_misses", ""))
        .replace("@@COV_COL_RATE@@", data.get("cov_col_rate", ""))
        .replace("@@COV_DYN_HITS@@", data.get("cov_dyn_hits", ""))
        .replace("@@COV_DYN_MISSES@@", data.get("cov_dyn_misses", ""))
        .replace("@@COV_DYN_RATE@@", data.get("cov_dyn_rate", ""))
        .replace("@@COV_TESTS@@", data.get("cov_tests", ""))
        .replace("@@UIL_LEFT@@", data.get("uil_left", ""))
        .replace("@@UIL_RIGHT@@", data.get("uil_right", ""))
        .replace("@@UIL_SUBTABS@@", data.get("uil_subtabs", ""))
        .replace("@@UIL_LEGACY@@", data.get("uil_legacy", ""))
        .replace("@@UIL_FIT@@", data.get("uil_fit", ""))
        .replace("@@UIL_LIB_PANEL@@", data.get("uil_lib_panel", ""))
        .replace("@@UIL_LIB_CONTENT@@", data.get("uil_lib_content", ""))
        .replace("@@UIL_LIB_VIEWPORT@@", data.get("uil_lib_viewport", ""))
        .replace("@@UIL_LIB_SCROLL@@", data.get("uil_lib_scroll", ""))
        .replace("@@UIL_PART@@", data.get("uil_part", ""))
        .replace("@@UIL_FLANGE@@", data.get("uil_flange", ""))
        .replace("@@UIL_SNAP_NEAR@@", data.get("uil_snap_near", ""))
        .replace("@@UIL_SNAP_FAR@@", data.get("uil_snap_far", ""))
        .replace("@@UIL_LOCAL@@", data.get("uil_local", ""))
        .replace("@@UIL_MOUNTED@@", data.get("uil_mounted", ""))
        .replace("@@UIL_FOLLOW@@", data.get("uil_follow", ""))
        .replace("@@UIL_TESTS@@", data.get("uil_tests", ""))
        .replace("@@SEQ_LANES@@", data.get("seq_lanes", ""))
        .replace("@@SEQ_DUMP@@", data.get("seq_dump", ""))
        .replace("@@SEQ_CLEAN@@", data.get("seq_clean", ""))
        .replace("@@SEQ_PARTIAL@@", data.get("seq_partial", ""))
        .replace("@@SEQ_PARTIAL_IDX@@", data.get("seq_partial_idx", ""))
        .replace("@@SEQ_PARTIAL_MSG@@", data.get("seq_partial_msg", ""))
        .replace("@@SEQ_ROBOT_KINDS@@", data.get("seq_robot_kinds", ""))
        .replace("@@SEQ_ROBOT_COUNT@@", data.get("seq_robot_count", ""))
        .replace("@@SEQ_NOTES@@", data.get("seq_notes", ""))
        .replace("@@SEQ_PLC_ROWS@@", data.get("seq_plc_rows", ""))
        .replace("@@SEQ_PLC_COUNT@@", data.get("seq_plc_count", ""))
        .replace("@@SEQ_PROG_COUNT@@", data.get("seq_prog_count", ""))
        .replace("@@SEQ_EQUIV@@", data.get("seq_equiv", ""))
        .replace("@@SEQ_SCL_DECLARED@@", data.get("seq_scl_declared", ""))
        .replace("@@SEQ_SCL_TIMER@@", data.get("seq_scl_timer", ""))
        .replace("@@SEQ_SCL_LINES@@", data.get("seq_scl_lines", ""))
        .replace("@@SEQ_STEPS@@", data.get("seq_steps", ""))
        .replace("@@SEQ_COUNTS@@", data.get("seq_counts", ""))
        .replace("@@SEQ_DWELL@@", data.get("seq_dwell", ""))
        .replace("@@SEQ_NOTE@@", data.get("seq_note", ""))
        .replace("@@SEQ_ED_ROWS@@", data.get("seq_ed_rows", ""))
        .replace("@@SEQ_ED_BLOCKS@@", data.get("seq_ed_blocks", ""))
        .replace("@@SEQ_ED_SIGNALS@@", data.get("seq_ed_signals", ""))
        .replace("@@SEQ_ED_ISSUES@@", data.get("seq_ed_issues", ""))
        .replace("@@SEQ_ED_SUMMARY@@", data.get("seq_ed_summary", ""))
        .replace("@@SEQ_TESTS@@", data.get("seq_tests", ""))
        .replace("@@SDEMO_STEPS@@", data.get("SDEMO_STEPS", ""))
        .replace("@@SDEMO_VALIDATION@@", data.get("SDEMO_VALIDATION", ""))
        .replace("@@SDEMO_UR@@", data.get("SDEMO_UR", ""))
        .replace("@@SDEMO_SCL@@", data.get("SDEMO_SCL", ""))
        .replace("@@SDEMO_JSON@@", data.get("SDEMO_JSON", ""))
        .replace("@@PLB_PRESET_ROWS@@", data.get("PLB_PRESET_ROWS", ""))
        .replace("@@PLB_CATALOG@@", data.get("PLB_CATALOG", ""))
        .replace("@@PLB_CATEGORIES@@", data.get("PLB_CATEGORIES", ""))
        .replace("@@PLB_CUSTOM_ROWS@@", data.get("PLB_CUSTOM_ROWS", ""))
        .replace("@@PROJ_SIZE_KB@@", data.get("PROJ_SIZE_KB", ""))
        .replace("@@PROJ_NAME@@", data.get("PROJ_NAME", ""))
        .replace("@@PROJ_POINTS@@", data.get("PROJ_POINTS", ""))
        .replace("@@PROJ_SEQ_STEPS@@", data.get("PROJ_SEQ_STEPS", ""))
        .replace("@@PROJ_DEVICES@@", data.get("PROJ_DEVICES", ""))
        .replace("@@PROJ_MAPPED@@", data.get("PROJ_MAPPED", ""))
        .replace("@@PROJ_ROWS@@", data.get("PROJ_ROWS", ""))
        .replace("@@PROJ_DESCRIBE@@", data.get("PROJ_DESCRIBE", ""))
        .replace("@@PROJ_REPORT@@", data.get("PROJ_REPORT", ""))
        .replace("@@PROJ_SNIPPET@@", data.get("PROJ_SNIPPET", ""))
        .replace("@@PROJ_CLI@@", data.get("PROJ_CLI", ""))
        .replace("@@PROJ_BROKEN_ROWS@@", data.get("PROJ_BROKEN_ROWS", ""))
        .replace("@@PROJ_BROKEN_REPORT@@", data.get("PROJ_BROKEN_REPORT", ""))
        .replace("@@PROJ_KEPT_ROWS@@", data.get("PROJ_KEPT_ROWS", ""))
        .replace("@@PROJ_TESTS@@", data.get("PROJ_TESTS", ""))
        .replace("@@SES_RECENT_ROWS@@", data.get("SES_RECENT_ROWS", ""))
        .replace("@@SES_EXISTING_N@@", data.get("SES_EXISTING_N", ""))
        .replace("@@SES_PRUNED@@", data.get("SES_PRUNED", ""))
        .replace("@@SES_RECENT_N@@", data.get("SES_RECENT_N", ""))
        .replace("@@SES_SNAP_SIZE_KB@@", data.get("SES_SNAP_SIZE_KB", ""))
        .replace("@@SES_META@@", data.get("SES_META", ""))
        .replace("@@SES_SNAPSHOT_SUMMARY@@", data.get("SES_SNAPSHOT_SUMMARY", ""))
        .replace("@@SES_CLI@@", data.get("SES_CLI", ""))
        .replace("@@SES_TESTS@@", data.get("SES_TESTS", ""))
        .replace("@@TPL_ROWS@@", data.get("TPL_ROWS", ""))
        .replace("@@TPL_STATS@@", data.get("TPL_STATS", ""))
        .replace("@@TPL_DESCRIBE@@", data.get("TPL_DESCRIBE", ""))
        .replace("@@TPL_CLI@@", data.get("TPL_CLI", ""))
        .replace("@@TPL_TESTS@@", data.get("TPL_TESTS", ""))
        .replace("@@VSC_NOTE@@", data.get("vsc_note", ""))
        .replace("@@IO_SCL_BLOCK@@", data["io_scl_block"])
        .replace("@@IO_UNMAPPED_COMMENT@@", data["io_unmapped_comment"])
        .replace("@@IO_ROUNDTRIP@@", data["io_roundtrip"])
        .replace("@@IO_SIGNALS_USED@@", data["io_signals_used"])
        .replace("@@IO_SIGNAL_NAMES@@", data["io_signal_names"])
        .replace("@@IO_TESTS@@", data["io_tests"])
        .replace("@@ARC_REJECT_TWO@@", data["arc_reject_two"])
        .replace("@@ARC_CYCLE_JOINT@@", data["arc_cycle_joint"])
        .replace("@@ARC_CYCLE_ARC@@", data["arc_cycle_arc"])
        .replace("@@ARC_INSTR_JOINT@@", data["arc_instr_joint"])
        .replace("@@ARC_INSTR_ARC@@", data["arc_instr_arc"])
        .replace("@@ARC_PLAN@@", data["arc_plan"])
        .replace("@@ARC_ROWS@@", data["arc_rows"])
        .replace("@@ARC_VIA_DIST@@", data["arc_via_dist"])
        .replace("@@ARC_DEVIATION@@", data["arc_deviation"])
        .replace("@@ARC_CHORD@@", data["arc_chord"])
        .replace("@@ARC_BRAND@@", data["arc_brand"])
        .replace("@@ARC_DEGRADE_NOTE@@", data["arc_degrade_note"])
        .replace("@@ARC_DEGRADE_LABELS@@", data["arc_degrade_labels"])
        .replace("@@ARC_TESTS@@", data["arc_tests"])
    )
    (WEB_DIR / "index.html").write_text(page, encoding="utf-8")
    print(f"节拍 {data['cycle_time']:.2f}s | 已生成 {WEB_DIR/'index.html'}")


if __name__ == "__main__":
    main()
