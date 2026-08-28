#!/usr/bin/env python3
"""批量生成机器人库 URDF 模型。

扫描 :mod:`orolp.robot_library.specs` 中的全部型号，为每个型号在
``models/<brand>/<model>/<model>.urdf`` 生成一份 URDF，并做数值自检：

* 零位时 TCP 高度应等于各段长度之和
* 水平完全伸展时 TCP 到 J1 轴线的水平距离应等于标称臂展

用法::

    python3 tools/generate_robot_models.py                # 生成全部
    python3 tools/generate_robot_models.py --brand kuka   # 仅生成某个品牌
    python3 tools/generate_robot_models.py --check        # 只校验不写文件
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "python"))

from orolp.robot_library.specs import ROBOTS, RobotSpec  # noqa: E402
from orolp.robot_library.urdf_builder import (  # noqa: E402
    build_urdf_string,
    derive_geometry,
    forward_kinematics,
)


def check_kinematics(spec: RobotSpec) -> tuple[float, float, float]:
    """对一个型号做运动学自检。

    Returns:
        ``(home_height, stretched_horizontal, reach_nominal)``，单位米。
    """
    g = derive_geometry(spec)

    # 零位：全部关节角为 0，TCP 应在正上方
    tcp_home, _ = forward_kinematics(spec, [0.0] * spec.dof)
    home_height = tcp_home[2]
    expected_height = (
        g.base_height
        + g.turret_height
        + g.upper_arm
        + g.forearm
        + g.wrist_1
        + g.wrist_2
        + g.flange_length
    )
    if spec.dof == 7:
        expected_height += 0.08 * spec.reach_mm / 1000.0

    if abs(home_height - expected_height) > 1e-6:
        raise AssertionError(
            f"{spec.key}: home height {home_height:.4f} != expected {expected_height:.4f}"
        )

    # 水平全伸：J2 转 -90°（大臂水平），其余为 0
    q = [0.0] * spec.dof
    q[1] = -90.0
    tcp_stretch, _ = forward_kinematics(spec, q)
    horizontal = math.hypot(tcp_stretch[0], tcp_stretch[1])

    return home_height, horizontal, spec.reach_mm / 1000.0


def main() -> int:
    parser = argparse.ArgumentParser(description="生成机器人库 URDF 模型")
    parser.add_argument("--brand", help="仅生成指定品牌（如 kuka）")
    parser.add_argument("--check", action="store_true", help="只做数值校验，不写文件")
    parser.add_argument("--models-dir", default=str(REPO_ROOT / "models"))
    args = parser.parse_args()

    models_dir = Path(args.models_dir)
    robots = [r for r in ROBOTS if args.brand is None or r.brand_id == args.brand]

    if not robots:
        print(f"No robots matched (brand={args.brand})", file=sys.stderr)
        return 1

    print(f"{'KEY':<28}{'DOF':>4}{'REACH(mm)':>11}{'HOME(m)':>10}{'STRETCH(m)':>12}{'ERR':>9}")
    print("-" * 74)

    failures = 0
    written = 0

    for spec in robots:
        home_h, horizontal, nominal = check_kinematics(spec)
        err = abs(horizontal - nominal) / nominal * 100.0

        if err > 1.0:
            failures += 1
            flag = "  <-- FAIL"
        else:
            flag = ""

        print(
            f"{spec.key:<28}{spec.dof:>4}{spec.reach_mm:>11.0f}"
            f"{home_h:>10.3f}{horizontal:>12.3f}{err:>8.2f}%{flag}"
        )

        if args.check:
            continue

        out_dir = models_dir / spec.brand_id / spec.model_id
        out_dir.mkdir(parents=True, exist_ok=True)
        urdf_path = out_dir / f"{spec.model_id}.urdf"
        urdf_path.write_text(build_urdf_string(spec), encoding="utf-8")
        written += 1

    print("-" * 74)
    print(f"Total: {len(robots)} models, {written} URDF written, {failures} failed.")

    if not args.check:
        _write_brand_readmes(models_dir, robots)

    return 1 if failures else 0


def _write_brand_readmes(models_dir: Path, robots) -> None:
    """为每个品牌生成一份 README，列出型号参数表。"""
    by_brand: dict[str, list[RobotSpec]] = {}
    for spec in robots:
        by_brand.setdefault(spec.brand_id, []).append(spec)

    for brand_id, specs in by_brand.items():
        brand = specs[0].brand
        lines = [
            f"# {brand.name_en}（{brand.name}）",
            "",
            f"- 国别：{brand.country}",
            f"- 典型控制器：{brand.controller}",
            "- 模型来源：由 `orolp.robot_library.urdf_builder` 程序化生成（仅基本几何体，不含厂商 CAD）",
            "",
            "## 型号列表",
            "",
            "| 型号 | 轴数 | 额定负载 (kg) | 臂展 (mm) | 重复定位精度 (mm) | 自重 (kg) | 防护等级 | 目录 |",
            "|---|---:|---:|---:|---:|---:|---|---|",
        ]
        for spec in specs:
            lines.append(
                f"| {spec.model_name} | {spec.dof} | {spec.payload_kg:g} | "
                f"{spec.reach_mm:g} | ±{spec.repeatability_mm:g} | "
                f"{spec.weight_kg if spec.weight_kg else '-'} | "
                f"{spec.ip_rating or '-'} | `{brand_id}/{spec.model_id}` |"
            )
        lines.append("")

        if any(not s.verified for s in specs):
            lines += [
                "> **参数说明**：标注为未核验的型号，其负载/臂展取自厂商公开样本，",
                "> 各轴运动范围与速度为同级别机型典型值，仅用于建模与演示，",
                "> 不可直接用于真实设备的编程与安全性评估。",
                "",
            ]

        (models_dir / brand_id / "README.md").write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    sys.exit(main())
