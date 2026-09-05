#!/usr/bin/env python3
"""机器人库外观渲染验证（无头）。

在 Xvfb 虚拟显示下启动真实的 ``GLWidget``，逐台机器人渲染到离屏缓冲并拼成一张
对比图。用途：

* **回归验证** —— 改动渲染管线后，确认每台机器人都能画出来、没有异常。
* **外观评审** —— 一眼看出造型是否失真、配色是否分层、网格是否加载成功。

与 ``QT_QPA_PLATFORM=offscreen`` 的区别：offscreen 插件**不支持 QOpenGLWidget**，
必须用真实的 X 显示（Xvfb）。

用法::

    xvfb-run -a python3.11 tools/render_robot_gallery.py --out docs/preview/robot_gallery.png

或在本脚本自动拉起 Xvfb 的情况下::

    python3.11 tools/render_robot_gallery.py
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO_ROOT))
sys.path.insert(0, str(_REPO_ROOT / "python"))


def _ensure_display() -> str:
    """确保有可用的 X 显示；必要时拉起 Xvfb。

    Returns:
        ``DISPLAY`` 环境变量的值。

    Raises:
        RuntimeError: 既无现成显示、也无法启动 Xvfb。
    """
    if os.environ.get("DISPLAY"):
        return os.environ["DISPLAY"]
    if shutil.which("Xvfb") is None:
        raise RuntimeError(
            "未检测到 DISPLAY，且系统缺少 Xvfb。"
            "请安装（apt-get install xvfb）后重试，或在已有 X 环境下运行。"
        )
    display = ":99"
    subprocess.Popen(
        ["Xvfb", display, "-screen", "0", "1600x1200x24"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    # 等 X server 建好 unix socket（/tmp/.X11-unix/X99）
    socket_path = Path(f"/tmp/.X11-unix/X{display.lstrip(':')}")
    for _ in range(60):
        if socket_path.exists():
            break
        time.sleep(0.1)
    else:
        raise RuntimeError(f"Xvfb 未能在 6 秒内就绪（等待 {socket_path}）")
    time.sleep(0.3)
    os.environ["DISPLAY"] = display
    return display


def render_robot(gl_widget, urdf_path: Path, width: int, height: int, pose=None):
    """渲染单台机器人，返回 RGB 字节串。

    Args:
        gl_widget: 已初始化的 ``GLWidget``。
        urdf_path: URDF 路径。
        width: 像素宽。
        height: 像素高。
        pose: 可选的关节角字典；为 ``None`` 时使用渲染器的待机位姿。

    Returns:
        ``(rgb_bytes, info_dict)``，RGB 按行优先、自上而下排列。
    """
    ok = gl_widget.load_robot(str(urdf_path))
    if pose:
        gl_widget.renderer.joint_values.update(pose)

    # 按模型尺度自动取景，避免小臂占满画面、大臂只看到一角
    model = gl_widget.renderer.model
    if model is not None:
        gl_widget.frame_robot()
    gl_widget.camera_az = 52.0
    gl_widget.camera_el = 22.0
    gl_widget.resize(width, height)

    # 强制同步渲染一帧
    gl_widget.show()
    gl_widget.repaint()
    from PySide6 import QtWidgets

    QtWidgets.QApplication.processEvents()

    from PySide6 import QtGui

    image = gl_widget.grabFramebuffer().convertToFormat(QtGui.QImage.Format_RGB888)

    # QImage 每行按 4 字节对齐，直接读 bits 会把 padding 混进来，必须按 stride 切行
    raw = np.frombuffer(
        image.constBits(), dtype=np.uint8, count=image.bytesPerLine() * image.height()
    )
    raw = raw.reshape(image.height(), image.bytesPerLine())
    # 必须拷贝：constBits() 给出的是 QImage 内部缓冲的视图，image 被回收后
    # numpy 就指向已释放内存——首次读取可能拿到残留数据，之后直接 SIGSEGV。
    rgb = np.array(raw[:, : image.width() * 3], dtype=np.uint8)
    rgb = rgb.reshape(image.height(), image.width(), 3)
    del image, raw
    return rgb, {
        "ok": ok,
        "name": gl_widget.renderer.robot_name,
        "visuals": model.visual_count if model else 0,
        "triangles": model.triangle_count if model else 0,
        "dof": model.dof if model else 0,
        "links": len(model.links) if model else 0,
    }


@dataclass
class Verdict:
    """单台机器人的渲染结论。"""

    ok: bool
    reason: str = ""
    coverage: float = 0.0     # 非背景像素占比
    hue: float = -1.0         # 画面主色调（度）
    target_hue: float = -1.0  # 期望色调（品牌 primary / accent）
    match: float = 0.0        # 与期望色调接近的像素占比


def _hsv(rgb: np.ndarray):
    """RGB(0..1) → (H 度, S, V)。避免依赖 matplotlib/colorsys 的逐像素开销。"""
    mx = rgb.max(axis=-1)
    mn = rgb.min(axis=-1)
    d = mx - mn
    safe = np.where(d > 1e-9, d, 1.0)
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    is_r = mx == r
    is_g = (mx == g) & ~is_r
    h = np.where(
        is_r, ((g - b) / safe) % 6.0,
        np.where(is_g, ((b - r) / safe) + 2.0, ((r - g) / safe) + 4.0),
    )
    h = (h * 60.0) % 360.0
    h = np.where(d > 1e-9, h, 0.0)
    s = np.where(mx > 1e-9, d / np.where(mx > 1e-9, mx, 1.0), 0.0)
    return h, s, mx


def _dominant_hue(h: np.ndarray, s: np.ndarray, v: np.ndarray, min_s=0.20, min_v=0.12):
    """取画面主色调（度）；没有足够彩色像素时返回 ``-1``。"""
    sel = (s > min_s) & (v > min_v)
    if int(sel.sum()) < 40:
        return -1.0
    hs, ws = h[sel], s[sel]
    bins = np.zeros(72)
    idx = np.minimum((hs / 5.0).astype(int), 71)
    np.add.at(bins, idx, ws)
    return float(np.argmax(bins) * 5.0 + 2.5)


def verify_tile(rgb: np.ndarray, key: str) -> Verdict:
    """判定一格渲染是否"机器人确实画出来了"。

    画面无法靠肉眼过目（CI 环境），所以用两条可量化判据：

    * **覆盖率** —— 非背景像素占比。太低说明什么都没画（相机跑偏、几何出画），
      太高说明贴满全屏（取景距离算错）。
    * **主色调匹配** —— 画面主色调是否接近品牌 primary（或 accent）的色相。
      色相对光照不敏感（光照只缩放明度），能可靠证明"是这台机器人、且着色正常"。
    """
    from orolp.robot_library.specs import BRANDS

    bg = np.array([38, 38, 46], dtype=np.int16)
    diff = np.abs(rgb.astype(np.int16) - bg).sum(axis=2)
    fg = diff > 24
    coverage = float(fg.sum()) / float(diff.size)

    brand = key.split("/")[0]
    spec = BRANDS.get(brand)
    if spec is None:
        return Verdict(coverage >= 0.02, "无品牌配色（仅校验覆盖率）", coverage)

    img01 = rgb.astype(np.float32) / 255.0
    h, s, v = _hsv(img01)
    hue = _dominant_hue(h, s, v)

    # primary 接近灰色时（如 JAKA 的银白）色相无意义，退而用 accent 判定
    primary = np.array(spec.primary, dtype=np.float32)
    ph, ps, _pv = _hsv(primary[None, None, :])
    target = float(ph[0, 0])
    if float(ps[0, 0]) < 0.15:
        accent = np.array(spec.accent, dtype=np.float32)
        ah, _as, _av = _hsv(accent[None, None, :])
        target = float(ah[0, 0])

    if hue < 0:
        return Verdict(
            coverage >= 0.02, "画面几乎没有彩色像素", coverage, hue, target
        )

    dh = abs(hue - target)
    dh = min(dh, 360.0 - dh)
    # 与目标色相 ±25° 内的像素占比
    near = (np.minimum(abs(h - target), 360.0 - abs(h - target)) < 25.0) & (s > 0.15)
    match = float(near.sum()) / float(diff.size)

    ok = coverage >= 0.015 and dh < 25.0
    reason = "" if ok else f"coverage={coverage:.3f} hue={hue:.0f}° target={target:.0f}° Δ={dh:.0f}°"
    return Verdict(ok, reason, coverage, hue, target, match)


def main() -> int:
    parser = argparse.ArgumentParser(description="机器人库外观渲染验证")
    parser.add_argument(
        "--out",
        default=str(_REPO_ROOT / "docs" / "preview" / "robot_gallery.png"),
        help="输出图片路径",
    )
    parser.add_argument("--cols", type=int, default=4, help="每行几台机器人")
    parser.add_argument("--cell", type=int, default=420, help="单格边长（像素）")
    parser.add_argument(
        "--robots",
        nargs="*",
        default=None,
        help="要渲染的机器人 key（如 kuka/kr16_2）；默认渲染全部",
    )
    args = parser.parse_args()

    _ensure_display()

    from PySide6 import QtWidgets
    from orolp.robot_library.catalog import RobotCatalog

    QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)

    # 延迟导入 olp，它会在 import 时构造 Qt 相关常量
    import importlib.util

    spec = importlib.util.spec_from_file_location("olp_main", _REPO_ROOT / "rw.py")
    olp = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(olp)

    catalog = RobotCatalog.from_repo_root(_REPO_ROOT)
    keys = args.robots or [e.key for e in catalog.all()]
    keys = list(keys)
    if "lerobot/so101" not in keys:
        # 带 STL 网格的模型必须纳入验证——它检验的是 <mesh> 渲染通路
        so101 = _REPO_ROOT / "models" / "lerobot" / "so101" / "urdf" / "so101.urdf"
        if so101.is_file():
            keys.append("lerobot/so101")

    widget = olp.GLWidget()
    widget.resize(args.cell, args.cell)

    tiles = []
    infos = []
    for key in keys:
        entry = catalog.get(key)
        if entry is None or entry.urdf_path is None:
            print(f"  ✗ {key}: 目录中不存在该型号，跳过", flush=True)
            continue
        path = Path(entry.urdf_path)
        try:
            rgb, info = render_robot(widget, Path(path), args.cell, args.cell)
        except Exception as exc:  # noqa: BLE001
            print(f"  ✗ {key}: 渲染失败 — {exc}", flush=True)
            continue
        tiles.append((key, np.flipud(rgb)))  # QImage 自下而上，翻回来
        infos.append((key, info))
        verdict = verify_tile(rgb, key)
        info["verdict"] = verdict
        flag = "✓" if verdict.ok else "✗"
        print(
            f"  {flag} {key:22s} visuals={info['visuals']:3d} "
            f"triangles={info['triangles']:7d} dof={info['dof']}"
            + ("" if verdict.ok else f"  ← {verdict.reason}"),
            flush=True,
        )

    if not tiles:
        print("没有任何机器人渲染成功")
        return 1

    # ---- 拼图 ---- #
    from PIL import Image

    cols = max(1, args.cols)
    rows = math.ceil(len(tiles) / cols)
    canvas = Image.new("RGB", (cols * args.cell, rows * args.cell), (26, 28, 34))
    for index, (_key, tile_arr) in enumerate(tiles):
        tile = Image.fromarray(tile_arr)
        canvas.paste(tile, ((index % cols) * args.cell, (index // cols) * args.cell))

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out_path)

    failed = [k for k, i in infos if not i["verdict"].ok]
    cov = [i["verdict"].coverage for _k, i in infos]
    print(f"\n已输出 {len(tiles)} 台机器人 → {out_path}")
    print(
        f"覆盖率 min={min(cov):.1%} 中位={sorted(cov)[len(cov)//2]:.1%} max={max(cov):.1%}"
    )
    if failed:
        print(f"✗ {len(failed)} 台未通过外观校验: {', '.join(failed)}")
        return 1
    print(f"✓ 全部 {len(tiles)} 台通过外观校验（覆盖率 + 品牌色调匹配）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
