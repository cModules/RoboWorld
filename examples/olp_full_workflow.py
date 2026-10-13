"""离线编程全闭环示例：轨迹生成 → 校验 → 后处理 → 上传 → 在线调试。

一条命令跑通 RoboWorld 的完整链路，不依赖任何真机或 GUI：

1. 加载机器人（URDF），用链式 API 编一条搬运程序（moveJ / moveL / moveSpline）；
2. 仿真展开成密轨迹，并做生成前校验（不可达 / 越限 / 奇异 / 干涉）；
3. 后处理生成目标控制器的源码（如 URScript / KRL / RAPID）；
4. 上传到一台「连接」——这里用 FileLink 离线导出，SimLink 可仿真真机；
5. 用 DebugSession 在连接上播放轨迹，读回实际位姿并与规划值比对偏差。

运行::

    python examples/olp_full_workflow.py [--brand kuka] [--post kuka_krc4]
"""
from __future__ import annotations

import argparse
import sys
import tempfile
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
# 分层包（domain/ service/ adapter/ …）就在仓库根；这里曾指向 core 时代的 python/ 布局，
# 该目录随 ADR-008 一起删除，导致本示例直接运行即 ModuleNotFoundError。
sys.path.insert(0, str(REPO))

from service.program import ProgramBuilder  # noqa: E402
from domain.pose import Pose
from domain.robot import Robot  # noqa: E402
from service.debug import DebugSession  # noqa: E402
from domain.link import FileLink, SimLink  # noqa: E402

# 各品牌代表性模型 + 默认后处理器
BRANDS = {
    "ur": ("assets/models/ur/ur5e/ur5e.urdf", "ur_polyscope"),
    "kuka": ("assets/models/kuka/kr16_2/kr16_2.urdf", "kuka_krc4"),
    "abb": ("assets/models/abb/irb1200/irb1200.urdf", "abb_irc5"),
    "fanuc": ("assets/models/fanuc/m20ia_35m/m20ia_35m.urdf", "fanuc_r30ib"),
    "estun": ("assets/models/estun/er20_1780/er20_1780.urdf", "estun_er"),
    "jaka": ("assets/models/jaka/zu3/zu3.urdf", "jaka_zu"),
}


def build_program(robot: Robot):
    """编一条「抓取 → 搬运 → 放下」式的程序。

    目标位姿直接由真实关节配置的**正解**得到，因此对任意品牌都可达，
    不用为每个品牌手调笛卡尔坐标。
    """
    dof = robot.dof
    a = np.zeros(dof)
    b = np.array([30.0, -30.0, 30.0, -30.0, 30.0, 0.0][:dof], dtype=float)
    c = np.array([-30.0, 30.0, -30.0, 30.0, -30.0, 0.0][:dof], dtype=float)
    pose_b = robot.forward_kinematics(b)
    pose_c = robot.forward_kinematics(c)
    return (
        ProgramBuilder(robot)
        .moveJ(a.tolist(), speed=40)
        .moveL(pose_b, speed=60, blend=10)
        .moveL(pose_c, speed=60, blend=10)
        # 平滑回零：SPLINE 提供比 moveJ 更柔和的加减速
        .moveSpline(a.tolist(), speed=50)
        .build()
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="RoboWorld 离线编程全闭环示例")
    # 默认 KUKA：与 README 的示例保持同源（KUKA / 节卡是首发品牌，机型最全）
    ap.add_argument("--brand", default="kuka", choices=list(BRANDS))
    ap.add_argument("--post", default=None, help="后处理器 full_name（默认按品牌推断）")
    args = ap.parse_args()

    rel, default_post = BRANDS[args.brand]
    post = args.post or default_post
    urdf = REPO / rel
    if not urdf.exists():
        print(f"[跳过] {args.brand} 缺程序化 URDF：{urdf}")
        return 0

    robot = Robot(str(urdf), args.brand)
    print(f"[1] 加载机器人 {args.brand}（DOF={robot.dof}）")

    # --- 轨迹生成 + 校验 ---
    prog = build_program(robot)
    report = prog.validate()
    print(f"[2] 校验：{'通过' if report.ok else '发现问题'}")
    if not report.ok:
        # 校验不通过也继续：后处理按指令生成代码、调试播放已规划出的合法段，
        # 这正体现"离线编程能在交付前暴露问题，但好段仍能上线"。
        print(report.format())
    else:
        print(f"    指令 {report.num_motions} 条 / 路径点 {report.num_waypoints} 个 / "
              f"节拍 {report.cycle_time:.2f}s")

    # --- 后处理（控制程序生成）---
    code = prog.to_post(post)
    print(f"[3] 后处理（{post}）：生成 {len(code.splitlines())} 行代码")
    print("    " + code.strip().splitlines()[0] if code.strip() else "    (空)")

    # --- 上传：离线导出到部署目录 ---
    deploy_dir = Path(tempfile.mkdtemp(prefix=f"olp_{args.brand}_"))
    file_link = FileLink(deploy_dir)
    file_link.connect()
    dep = prog.deploy(file_link, post, name=f"{args.brand}_demo")
    print(f"[4] 上传（FileLink）：{dep.handle}  [{dep.notes}]")

    # --- 在线调试：在仿真连接上播放并比对偏差 ---
    sim_link = SimLink(robot)
    sim_link.connect()
    prog.deploy(sim_link, post, name=f"{args.brand}_demo_sim")
    traj = prog.simulate()
    sess = DebugSession(traj, sim_link, robot=robot)
    summary = sess.run()
    print(f"[5] 在线调试（SimLink）：执行 {summary['executed']} 路点，"
          f"最大位置偏差 {summary['max_pos_deviation_mm']:.4f} mm，"
          f"最大姿态偏差 {summary['max_rot_deviation_deg']:.4f}°")

    print("\n✓ 离线编程全闭环完成。把 FileLink 换成真实控制器连接即可上线运行。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
