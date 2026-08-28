"""渲染工作单元场景为图片。

用途：
    1. 在无 GPU/无显示器的环境下（CI、服务器）产出场景快照
    2. 给文档/宣传材料配图

实现说明：
    直接用 :mod:`mpl_toolkits.mplot3d` 而非 OpenGL。Qt 的 ``QOpenGLWidget``
    在 Xvfb 下 ``grabFramebuffer()`` 只能拿到背景色（实测如此），无法离线
    截图；而本脚本只需要静态图，用 matplotlib 反而更省事、依赖更少。

用法::

    python3 tools/render_workcell.py --out docs/preview/workcell.png
    python3 tools/render_workcell.py --scene scenes/demo.json
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT / "python"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

from orolp.pose import Pose  # noqa: E402
from orolp.world.geometry import Box, Cylinder, Sphere  # noqa: E402
from orolp.world.scene import Scene  # noqa: E402

CYL_SEGMENTS = 24

#: 优先选用的中文字体族名
_CJK_FONT_CANDIDATES = (
    "Noto Sans CJK SC",
    "Noto Sans CJK JP",
    "Noto Sans SC",
    "Source Han Sans SC",
    "WenQuanYi Zen Hei",
)


def _setup_cjk_font() -> str:
    """选中文字体，避免标题/图例渲染成方框。

    Noto Sans CJK 以 ttc 形式发布，matplotlib 往往只登记 ``JP`` 变体，
    但该字体覆盖全部 CJK 统一汉字，因此把含 CJK/Han/Hei 的族名也列为候选。

    Returns:
        实际选中的字体名；未找到时返回空串（调用方仍能继续，只是中文变方框）
    """
    installed = sorted({f.name for f in font_manager.fontManager.ttflist})
    loose = [n for n in installed if any(k in n for k in ("CJK", "Han", "Hei", "Song"))]
    for name in list(_CJK_FONT_CANDIDATES) + loose:
        if name in installed:
            plt.rcParams["font.sans-serif"] = [name] + list(plt.rcParams["font.sans-serif"])
            plt.rcParams["axes.unicode_minus"] = False
            return name
    return ""


def _box_faces(size):
    """长方体的 6 个面（局部坐标）。"""
    sx, sy, sz = (float(v) / 2.0 for v in size)
    v = np.array(
        [
            [-sx, -sy, -sz], [sx, -sy, -sz], [sx, sy, -sz], [-sx, sy, -sz],
            [-sx, -sy, sz], [sx, -sy, sz], [sx, sy, sz], [-sx, sy, sz],
        ]
    )
    return [
        [v[0], v[1], v[2], v[3]],  # -Z
        [v[4], v[5], v[6], v[7]],  # +Z
        [v[0], v[1], v[5], v[4]],  # -Y
        [v[2], v[3], v[7], v[6]],  # +Y
        [v[0], v[3], v[7], v[4]],  # -X
        [v[1], v[2], v[6], v[5]],  # +X
    ]


def _cylinder_mesh(radius: float, length: float):
    """圆柱的侧面 + 上下盖（局部坐标，轴向 +Z）。"""
    n = CYL_SEGMENTS
    h = float(length) / 2.0
    ang = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    lo = np.column_stack([radius * np.cos(ang), radius * np.sin(ang), np.full(n, -h)])
    hi = np.column_stack([radius * np.cos(ang), radius * np.sin(ang), np.full(n, h)])
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris.append([lo[i], lo[j], hi[j]])
        tris.append([lo[i], hi[j], hi[i]])
        tris.append([[0, 0, h], hi[i], hi[j]])
        tris.append([[0, 0, -h], lo[j], lo[i]])
    return np.array(tris, dtype=np.float64)


def _sphere_mesh(radius: float, seg: int = 16, ring: int = 10):
    """球面的三角网格（局部坐标）。"""
    tris = []
    def pt(phi, theta):
        return [
            radius * np.sin(phi) * np.cos(theta),
            radius * np.sin(phi) * np.sin(theta),
            radius * np.cos(phi),
        ]
    for i in range(ring):
        p0, p1 = np.pi * i / ring, np.pi * (i + 1) / ring
        for k in range(seg):
            t0, t1 = 2 * np.pi * k / seg, 2 * np.pi * (k + 1) / seg
            a, b = pt(p0, t0), pt(p0, t1)
            c, d = pt(p1, t0), pt(p1, t1)
            tris.append([a, b, d])
            tris.append([a, d, c])
    return np.array(tris, dtype=np.float64)


def _apply(mat: np.ndarray, pts) -> np.ndarray:
    """把 4x4 变换作用到点集上。"""
    pts = np.asarray(pts, dtype=np.float64)
    shape = pts.shape
    pts = pts.reshape(-1, 3)
    out = (mat[:3, :3] @ pts.T).T + mat[:3, 3]
    return out.reshape(shape)


def draw_scene(ax, scene: Scene, highlight: set | None = None) -> None:
    """把场景绘制到 3D 坐标轴。

    Args:
        ax: matplotlib 3D 坐标轴
        scene: 待绘制场景
        highlight: 需要高亮的对象名集合（干涉对象）
    """
    highlight = highlight or set()

    def emit(parts, name, alpha: float):
        hot = name in highlight
        for part in parts:
            color = (1.0, 0.25, 0.15) if hot else part.color
            prim = part.prim
            if isinstance(prim, Box):
                polys = _box_faces(prim.size)
            elif isinstance(prim, Cylinder):
                polys = _cylinder_mesh(prim.radius, prim.length)
            elif isinstance(prim, Sphere):
                polys = _sphere_mesh(prim.radius)
            else:
                continue
            polys = _apply(part.transform, polys)
            ax.add_collection3d(
                Poly3DCollection(
                    polys,
                    facecolors=color,
                    edgecolors=(0.08, 0.08, 0.10),
                    linewidths=0.35 if not hot else 0.9,
                    alpha=alpha,
                )
            )

    for name in scene.objects:
        if not scene.objects[name].visible:
            continue
        emit(scene.object_parts(name), name, 0.95)

    if scene.has_robot:
        emit(scene.robot_parts(), "", 1.0)


def build_demo_scene(urdf: str) -> Scene:
    """构造一个演示工作单元：机器人 + 工作台 + 工件 + 围栏 + 输送线。

    Args:
        urdf: 机器人 URDF 路径

    Returns:
        场景对象
    """
    scene = Scene()
    scene.set_robot(urdf, [0, 15, 45, 0, 25, 0])
    scene.add("worktable", pose=Pose.from_xyz_rpy(1100, 0, 0, 0, 0, 0),
              params={"length": 1600, "width": 900, "height": 800})
    scene.add("workpiece_box", pose=Pose.from_xyz_rpy(1100, 0, 800, 0, 0, 0),
              params={"length": 500, "width": 350, "height": 220})
    scene.add("vise", pose=Pose.from_xyz_rpy(1100, -220, 800, 0, 0, 0))
    scene.add("conveyor", pose=Pose.from_xyz_rpy(0, -1500, 0, 0, 0, 0),
              params={"length": 3500, "width": 700, "height": 800})
    scene.add("fence", pose=Pose.from_xyz_rpy(0, 1900, 0, 0, 0, 0),
              params={"length": 3000, "height": 2000})
    scene.add("cabinet", pose=Pose.from_xyz_rpy(-1800, 800, 0, 0, 0, 0))
    return scene


def main() -> int:
    parser = argparse.ArgumentParser(description="渲染工作单元场景")
    parser.add_argument("--scene", help="场景 JSON 文件；缺省渲染内置演示场景")
    parser.add_argument("--robot", default=str(_REPO_ROOT / "models/kuka/kr16_2/kr16_2.urdf"),
                        help="演示场景使用的机器人 URDF")
    parser.add_argument("--out", default=str(_REPO_ROOT / "docs/preview/workcell.png"),
                        help="输出图片路径")
    parser.add_argument("--dpi", type=int, default=130)
    args = parser.parse_args()

    if args.scene:
        scene = Scene.load(args.scene)
    else:
        scene = build_demo_scene(args.robot)

    _font = _setup_cjk_font()
    if not _font:
        print("[warning] 未找到中文字体，标题中的汉字可能显示为方框")

    hits = scene.check_collisions(margin=-0.5)
    highlight = {h.obj_b for h in hits}

    fig = plt.figure(figsize=(11, 8), facecolor="#1a1a20")
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor("#1a1a20")

    draw_scene(ax, scene, highlight)

    lo, hi = scene.bounds()
    span = float(np.max(hi - lo)) or 1000.0
    cx, cy, cz = (lo + hi) / 2.0
    r = span * 0.62
    ax.set_xlim(cx - r, cx + r)
    ax.set_ylim(cy - r, cy + r)
    ax.set_zlim(max(0.0, cz - r), cz + r)
    ax.set_box_aspect((1, 1, 0.62))

    ax.set_xlabel("X (mm)", color="#c8cad4", fontsize=9)
    ax.set_ylabel("Y (mm)", color="#c8cad4", fontsize=9)
    ax.set_zlabel("Z (mm)", color="#c8cad4", fontsize=9)
    ax.tick_params(colors="#8a8b98", labelsize=7)
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.set_facecolor("#1a1a20")
        pane.set_edgecolor("#34353f")
    ax.grid(color="#34353f", linewidth=0.4)

    title = f"工作单元 · {len(scene)} 个对象"
    if scene.robot_urdf:
        title += f" · {Path(scene.robot_urdf).stem}"
    if hits:
        title += f" · 检出 {len(hits)} 处干涉"
    ax.set_title(title, color="#e8e9f0", fontsize=12, pad=10)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=args.dpi, facecolor="#1a1a20", bbox_inches="tight")
    plt.close(fig)

    print(f"✓ 已渲染: {out}")
    print(f"  对象 {len(scene)} 个，干涉 {len(hits)} 处")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
