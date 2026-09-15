#!/usr/bin/env python3
"""STEP/CAM 轨迹生成命令行工具（v0.3.2）。

从 STEP/IGES 模型提取焊缝/轮廓，生成机器人程序与品牌代码；或把扫描点云
（PLY）与规划路径做偏差比对。

用法::

    # 列出模型里的全部焊缝（索引 / 闭合 / 长度 / 点数）
    python3 tools/cam_path.py seams --input bracket.step

    # 提取第 0 条缝 → 生成 UR 程序代码（抬高 2mm、TCP 30mm/s）
    python3 tools/cam_path.py path --input bracket.step --seam 0 \
        --urdf models/kuka/kr16_2/kr16_2.urdf --robot-name kr16_2 \
        --post ur --velocity 30 --lift 2 --out weld.program

    # 扫描偏差比对：点云 vs 模型焊缝
    python3 tools/cam_path.py deviation --points scan.ply --input bracket.step --seam 0

依赖：STEP/IGES 提取需要 ``cadquery-ocp``（OpenCASCADE 绑定）；
PLY 偏差比对为纯 numpy，无需额外依赖。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "python"))

import numpy as np  # noqa: E402

from orolp.cam import (  # noqa: E402
    extract_seams,
    load_ply,
    path_to_program,
    scan_deviation,
    select_seams,
)


def _pick_seam(args):
    if not Path(args.input).exists():
        sys.exit(f"CAD 文件不存在: {args.input}")
    seams = extract_seams(args.input, deflection=args.deflection)
    if not seams:
        sys.exit("未提取到任何边/焊缝")
    picked = select_seams(seams, min_length=args.min_length, index=args.seam)
    if not picked:
        for i, s in enumerate(seams):
            print(f"  [{i}] closed={s.closed} length={s.length:.1f}mm points={len(s)}")
        sys.exit("筛选条件过严，未命中焊缝（可用列表见上）")
    return seams, picked


def cmd_seams(args) -> None:
    if not Path(args.input).exists():
        sys.exit(f"CAD 文件不存在: {args.input}")
    seams = extract_seams(args.input, deflection=args.deflection)
    print(f"共提取 {len(seams)} 条边链（deflection={args.deflection}mm）：")
    for i, s in enumerate(seams):
        tag = "闭环" if s.closed else "开放"
        print(f"  [{i}] {tag}  长度 {s.length:9.2f} mm  点数 {len(s):4d}")


def cmd_path(args) -> None:
    seams, picked = _pick_seam(args)
    seam = picked[0]
    print(
        f"选用焊缝：closed={seam.closed} 长度={seam.length:.2f}mm 点数={len(seam)}"
    )
    if not args.urdf:
        np.savetxt(args.out if args.out else "cam_path.txt", seam.points, fmt="%.4f")
        print(f"未指定 --urdf，仅导出路径点 → {args.out or 'cam_path.txt'}")
        return

    from orolp.robot import Robot

    robot = Robot(str(Path(args.urdf).resolve()), args.robot_name)
    builder = path_to_program(
        seam.points, robot,
        closed=seam.closed,
        corner_angle_deg=args.corner_angle,
        z_lift=args.lift,
        velocity=args.velocity,
        blend=args.blend,
        normal=tuple(args.normal),
        rpy=tuple(args.rpy),
    )
    if args.weld:
        builder.gripper("close")
    prog = builder.build()
    report = prog.validate()
    errors = [i for i in report.issues if i.severity == "error"]
    print(f"指令数 {len(prog.items)}  节拍 {report.cycle_time:.2f}s  错误 {len(errors)}")
    for e in errors[:5]:
        print(f"  [error] {e.message}")

    if args.post:
        from orolp.post import get_post_processor

        code = get_post_processor(args.post).generate(prog.items, {"name": "cam_seam"})
        out = Path(args.out) if args.out else Path(f"cam_seam.{get_post_processor(args.post).file_extension}")
        out.write_text(code)
        print(f"已生成 {args.post} 代码 → {out}")
    else:
        np.savetxt(args.out or "cam_path.txt", seam.points, fmt="%.4f")
        print(f"未指定 --post，仅导出路径点 → {args.out or 'cam_path.txt'}")


def cmd_deviation(args) -> None:
    if not Path(args.points).exists():
        sys.exit(f"点云文件不存在: {args.points}")
    cloud = load_ply(args.points)
    if args.path:
        path_pts = np.loadtxt(args.path).reshape(-1, 3)
    elif args.input:
        seams, picked = _pick_seam(args)
        path_pts = picked[0].points
    else:
        sys.exit("deviation 模式需要 --path 路径点文件或 --input CAD + --seam")
    report = scan_deviation(cloud, path_pts)
    print(report.format())
    if args.out:
        np.savetxt(args.out, np.column_stack([np.arange(report.n_points), report.distances]),
                   header="index deviation_mm", fmt=["%d", "%.4f"])
        print(f"逐点偏差已导出 → {args.out}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="mode", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--input", help="STEP/IGES 文件")
    common.add_argument("--deflection", type=float, default=0.2, help="边离散弦高容差 mm（默认 0.2）")
    common.add_argument("--min-length", type=float, default=0.0, help="焊缝最小长度 mm")
    common.add_argument("--seam", type=int, default=None, help="直接按序号选缝")

    p_seams = sub.add_parser("seams", parents=[common], help="列出全部焊缝")

    p_path = sub.add_parser("path", parents=[common], help="焊缝 → 机器人程序/代码")
    p_path.add_argument("--urdf", help="机器人 URDF（不指定则仅导出路径点）")
    p_path.add_argument("--robot-name", default="robot")
    p_path.add_argument("--post", help="后处理器名（ur/kuka/abb/fanuc…，不指定仅导路径点）")
    p_path.add_argument("--velocity", type=float, default=50.0, help="TCP 速度 mm/s")
    p_path.add_argument("--blend", type=float, default=1.0, help="融合半径 mm")
    p_path.add_argument("--lift", type=float, default=0.0, help="沿法向抬高 mm")
    p_path.add_argument("--normal", type=float, nargs=3, default=[0, 0, 1], help="表面法向")
    p_path.add_argument("--rpy", type=float, nargs=3, default=[0, 180, 0], help="工具姿态 RPY 度")
    p_path.add_argument("--corner-angle", type=float, default=20.0, help="转角阈值度")
    p_path.add_argument("--weld", action="store_true", help="路径末尾追加夹爪关闭（示意工艺 IO）")
    p_path.add_argument("--out", help="输出文件")

    p_dev = sub.add_parser("deviation", parents=[common], help="点云扫描偏差比对")
    p_dev.add_argument("--points", required=True, help="PLY 点云文件")
    p_dev.add_argument("--path", help="规划路径点 txt（与 --input/--seam 二选一）")
    p_dev.add_argument("--out", help="逐点偏差导出 txt")

    args = parser.parse_args()
    if args.mode == "seams":
        cmd_seams(args)
    elif args.mode == "path":
        cmd_path(args)
    else:
        cmd_deviation(args)


if __name__ == "__main__":
    main()
