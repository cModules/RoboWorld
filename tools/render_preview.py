#!/usr/bin/env python3
"""渲染机器人库预览图（纯软件光栅化，无需 GPU / 显示器）。

与 GUI 里的 OpenGL 视口不同，本工具自带一套极简的三角面光栅化器：

* 直接解析 ``assets/models/**/*.urdf``（仅 box / cylinder / sphere 三类基本体）；
* 按关节角做正运动学，把每个 visual 的顶点变换到世界坐标；
* 用透视相机 + 画家算法（按面片深度排序）+ 平面着色画出实体感。

这样在 CI / 沙箱 / 无 X server 的机器上也能产出可人工核对的预览图，
用于检查连杆比例、品牌配色、关节朝向是否正确。

用法::

    python3 tools/render_preview.py                  # 生成 docs/preview/ 下的两张图
    python3 tools/render_preview.py --layout hero    # 只生成 6 机型大图
    python3 tools/render_preview.py -o /tmp/all.png --layout all

依赖：numpy、matplotlib（仓库其余部分不依赖它们，仅本工具需要）。
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.collections import LineCollection, PolyCollection

from adapter.robot_library.specs import BRANDS, ROBOTS  # noqa: E402

#: 预览图里有中文（品牌名/型号说明），挑一个系统里存在的中文字体，避免渲染成方框
_CJK_FONT_CANDIDATES = (
    "Noto Sans CJK SC",
    "Noto Sans SC",
    "Source Han Sans SC",
    "WenQuanYi Zen Hei",
    "Microsoft YaHei",
    "SimHei",
    "PingFang SC",
)


def _setup_cjk_font() -> str:
    """选中文字体。Noto Sans CJK 以 ttc 形式发布，matplotlib 往往只登记 ``JP`` 变体，
    但该字体覆盖全部 CJK 统一汉字，因此把含 CJK/Han/Hei 的族名也作为候选。"""
    installed = sorted({f.name for f in font_manager.fontManager.ttflist})
    loose = [n for n in installed if any(k in n for k in ("CJK", "Han", "Hei", "Song"))]
    for name in list(_CJK_FONT_CANDIDATES) + loose:
        if name in installed:
            plt.rcParams["font.sans-serif"] = [name] + list(plt.rcParams["font.sans-serif"])
            plt.rcParams["axes.unicode_minus"] = False
            return name
    return ""


# --------------------------------------------------------------------------- #
# URDF 解析
# --------------------------------------------------------------------------- #


@dataclass
class Visual:
    """一个 <visual>：局部位姿 + 基本体 + 颜色。"""

    origin: np.ndarray  # (3,)
    kind: str  # box | cylinder | sphere
    params: Tuple[float, ...]
    color: Tuple[float, float, float]


@dataclass
class Joint:
    """一个 <joint>：父子关系、原点、转轴。"""

    name: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    movable: bool


@dataclass
class Link:
    """一个 <link>：若干 visual。"""

    name: str
    visuals: List[Visual] = field(default_factory=list)


@dataclass
class UrdfModel:
    """解析后的 URDF。"""

    name: str
    links: Dict[str, Link]
    joints: List[Joint]
    root: str


def _vec3(text: str, default: Sequence[float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    """解析以空白或逗号分隔的浮点串。

    兼容 ``xyz`` (3) 和 ``rgba`` (4) 两种长度——URDF 的 color 节点是 rgba，
    若只接受 3 个值会把所有 visual 错认为缺省色。"""
    parts = (text or "").replace(",", " ").split()
    if len(parts) < 3:
        return np.array(default, dtype=float)
    return np.array([float(p) for p in parts[:3]], dtype=float)


def load_urdf(path: Path) -> UrdfModel:
    """解析 URDF 文件。只支持本仓库生成器的子集：原点无 rpy、基本体几何。"""
    tree = ET.parse(path)
    root = tree.getroot()

    links: Dict[str, Link] = {}
    for link_el in root.findall("link"):
        name = link_el.get("name", "")
        link = Link(name=name)
        for vis in link_el.findall("visual"):
            origin_el = vis.find("origin")
            origin = _vec3(origin_el.get("xyz") if origin_el is not None else None)
            geom = vis.find("geometry")
            if geom is None:
                continue
            color = (0.6, 0.6, 0.65)
            mat = vis.find("material/color")
            if mat is not None:
                rgba = _vec3(mat.get("rgba"), (0.6, 0.6, 0.65))
                color = (float(rgba[0]), float(rgba[1]), float(rgba[2]))
            box = geom.find("box")
            cyl = geom.find("cylinder")
            sph = geom.find("sphere")
            if box is not None:
                size = _vec3(box.get("size"), (0.1, 0.1, 0.1))
                link.visuals.append(Visual(origin, "box", tuple(float(v) for v in size), color))
            elif cyl is not None:
                r = float(cyl.get("radius", 0.05))
                length = float(cyl.get("length", 0.1))
                link.visuals.append(Visual(origin, "cylinder", (r, length), color))
            elif sph is not None:
                r = float(sph.get("radius", 0.02))
                link.visuals.append(Visual(origin, "sphere", (r,), color))
        links[name] = link

    joints: List[Joint] = []
    children = set()
    for joint_el in root.findall("joint"):
        jtype = joint_el.get("type", "fixed")
        parent_el = joint_el.find("parent")
        child_el = joint_el.find("child")
        if parent_el is None or child_el is None:
            continue
        origin_el = joint_el.find("origin")
        axis_el = joint_el.find("axis")
        joints.append(
            Joint(
                name=joint_el.get("name", ""),
                parent=parent_el.get("link", ""),
                child=child_el.get("link", ""),
                origin=_vec3(origin_el.get("xyz") if origin_el is not None else None),
                axis=_vec3(axis_el.get("xyz") if axis_el is not None else "0 0 1"),
                movable=jtype in ("revolute", "continuous", "prismatic"),
            )
        )
        children.add(joints[-1].child)

    roots = [n for n in links if n not in children]
    root_name = "base_link" if "base_link" in roots else (roots[0] if roots else "")
    return UrdfModel(name=root.get("name", path.stem), links=links, joints=joints, root=root_name)


# --------------------------------------------------------------------------- #
# 正运动学
# --------------------------------------------------------------------------- #


def _rot(axis: np.ndarray, angle: float) -> np.ndarray:
    """绕任意轴旋转（Rodrigues），返回 3x3。"""
    n = float(np.linalg.norm(axis))
    if n < 1e-12 or abs(angle) < 1e-12:
        return np.eye(3)
    k = axis / n
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]], dtype=float)
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def _T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = R
    m[:3, 3] = t
    return m


def link_transforms(model: UrdfModel, q_deg: Sequence[float]) -> Dict[str, np.ndarray]:
    """按给定关节角计算每个 link 的世界变换（URDF 约定：T_child = T_parent·T(origin)·R(axis,q)）。"""
    by_parent: Dict[str, List[Joint]] = {}
    for j in model.joints:
        by_parent.setdefault(j.parent, []).append(j)

    transforms: Dict[str, np.ndarray] = {model.root: np.eye(4)}
    qi = 0
    stack = [model.root]
    while stack:
        parent = stack.pop()
        for joint in by_parent.get(parent, []):
            if joint.child in transforms:
                continue
            base = transforms[parent] @ _T(np.eye(3), joint.origin)
            if joint.movable:
                angle = math.radians(q_deg[qi]) if qi < len(q_deg) else 0.0
                qi += 1
                base = base @ _T(_rot(joint.axis, angle), np.zeros(3))
            transforms[joint.child] = base
            stack.append(joint.child)
    return transforms


def tcp_position(model: UrdfModel, q_deg: Sequence[float]) -> np.ndarray:
    """末端 TCP 的世界坐标（用于把机器人挪进相机取景框）。"""
    tr = link_transforms(model, q_deg)
    for name in ("tcp", "tool0"):
        if name in tr:
            return tr[name][:3, 3].copy()
    return np.zeros(3)


# --------------------------------------------------------------------------- #
# 基本体三角化
# --------------------------------------------------------------------------- #

_CYL_SEGMENTS = 28
_SPHERE_SEGMENTS = 16


def _box_tris(sx: float, sy: float, sz: float) -> np.ndarray:
    h = np.array([sx, sy, sz], dtype=float) / 2.0
    c = (
        np.array(
            [
                [-1, -1, -1],
                [1, -1, -1],
                [1, 1, -1],
                [-1, 1, -1],
                [-1, -1, 1],
                [1, -1, 1],
                [1, 1, 1],
                [-1, 1, 1],
            ],
            dtype=float,
        )
        * h
    )
    faces = [
        (0, 1, 2, 3),
        (4, 5, 6, 7),
        (0, 1, 5, 4),
        (1, 2, 6, 5),
        (2, 3, 7, 6),
        (3, 0, 4, 7),
    ]
    tris = []
    for a, b, cc, d in faces:
        tris.append([c[a], c[b], c[cc]])
        tris.append([c[a], c[cc], c[d]])
    return np.array(tris, dtype=float)


def _cylinder_tris(radius: float, length: float) -> np.ndarray:
    n = _CYL_SEGMENTS
    half = length / 2.0
    ang = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    cos, sin = np.cos(ang), np.sin(ang)
    tris = []
    for i in range(n):
        j = (i + 1) % n
        p0 = np.array([radius * cos[i], radius * sin[i], -half])
        p1 = np.array([radius * cos[j], radius * sin[j], -half])
        p2 = np.array([radius * cos[j], radius * sin[j], half])
        p3 = np.array([radius * cos[i], radius * sin[i], half])
        tris.append([p0, p1, p2])
        tris.append([p0, p2, p3])
        # 端盖
        tris.append([np.array([0.0, 0.0, half]), p3, p2])
        tris.append([np.array([0.0, 0.0, -half]), p1, p0])
    return np.array(tris, dtype=float)


def _sphere_tris(radius: float) -> np.ndarray:
    n = _SPHERE_SEGMENTS
    tris = []
    for i in range(n):
        t0 = math.pi * i / n
        t1 = math.pi * (i + 1) / n
        for j in range(n):
            p0 = math.pi * 2.0 * j / n
            p1 = math.pi * 2.0 * (j + 1) / n
            v = []

            def pt(theta: float, phi: float) -> np.ndarray:
                return np.array(
                    [
                        radius * math.sin(theta) * math.cos(phi),
                        radius * math.sin(theta) * math.sin(phi),
                        radius * math.cos(theta),
                    ]
                )

            a, b, c, d = pt(t0, p0), pt(t1, p0), pt(t1, p1), pt(t0, p1)
            if i == 0:
                v.append([a, b, c])
            elif i == n - 1:
                v.append([a, b, d])
            else:
                v.append([a, b, c])
                v.append([a, c, d])
            tris.extend(v)
    return np.array(tris, dtype=float)


def _tris_for(vis: Visual) -> np.ndarray:
    if vis.kind == "box":
        return _box_tris(*vis.params)
    if vis.kind == "cylinder":
        return _cylinder_tris(*vis.params)
    return _sphere_tris(*vis.params)


# --------------------------------------------------------------------------- #
# 相机与光栅化
# --------------------------------------------------------------------------- #


class Camera:
    """简单透视相机（世界为 Z-up）。"""

    def __init__(self, eye: np.ndarray, target: np.ndarray, fov_deg: float = 32.0):
        self.eye = np.asarray(eye, dtype=float)
        self.target = np.asarray(target, dtype=float)
        f = self.target - self.eye
        f = f / np.linalg.norm(f)
        up = np.array([0.0, 0.0, 1.0])
        s = np.cross(f, up)
        if np.linalg.norm(s) < 1e-6:
            s = np.array([1.0, 0.0, 0.0])
        s = s / np.linalg.norm(s)
        u = np.cross(s, f)
        self.basis = np.vstack([s, u, -f])
        self.fov = math.radians(fov_deg)

    def view(self, pts: np.ndarray) -> np.ndarray:
        """世界点 -> 相机坐标（相机朝 -z 看）。"""
        return (self.basis @ (pts - self.eye).T).T

    def project(self, view_pts: np.ndarray, width: int, height: int) -> np.ndarray:
        focal = (height / 2.0) / math.tan(self.fov / 2.0)
        depth = -view_pts[:, 2]
        depth = np.where(depth < 1e-4, 1e-4, depth)
        x = focal * view_pts[:, 0] / depth
        y = focal * view_pts[:, 1] / depth
        return np.column_stack([width / 2.0 + x, height / 2.0 - y, depth])


_LIGHT = np.array([0.45, -0.75, 0.85])
_LIGHT = _LIGHT / np.linalg.norm(_LIGHT)


def _shade(normal: np.ndarray, base: np.ndarray) -> Tuple[float, float, float]:
    """平面着色：环境光 + 漫反射 + 轻微边缘提亮。"""
    ndl = abs(float(np.dot(normal, _LIGHT)))
    ambient = 0.32
    diffuse = 0.68 * ndl
    k = min(1.0, ambient + diffuse)
    gamma = 0.9
    return tuple(float(min(1.0, c * k) ** gamma) for c in base)


def render_triangles(
    model: UrdfModel,
    q_deg: Sequence[float],
    camera: Camera,
    width: int,
    height: int,
) -> Tuple[List[np.ndarray], List[Tuple[float, float, float]]]:
    """返回（屏幕三角形顶点列表, 颜色列表），已按深度从远到近排序。"""
    transforms = link_transforms(model, q_deg)

    tris_world: List[np.ndarray] = []
    tris_color: List[np.ndarray] = []
    for link_name, T in transforms.items():
        link = model.links.get(link_name)
        if link is None:
            continue
        for vis in link.visuals:
            local = _tris_for(vis)
            tv = T @ _T(np.eye(3), vis.origin)
            R = tv[:3, :3]
            t = tv[:3, 3]
            world = (R @ local.reshape(-1, 3).T).T + t
            tris_world.append(world.reshape(-1, 3, 3))
            tris_color.append(np.tile(np.array(vis.color, dtype=float), (len(local), 1)))

    if not tris_world:
        return [], []

    world = np.concatenate(tris_world, axis=0)
    colors = np.concatenate(tris_color, axis=0)

    # 背面剔除（法线背向相机的一律丢弃，可减少一半面片）
    edge1 = world[:, 1] - world[:, 0]
    edge2 = world[:, 2] - world[:, 0]
    normals = np.cross(edge1, edge2)
    lengths = np.linalg.norm(normals, axis=1)
    valid = lengths > 1e-12
    normals = np.where(
        valid[:, None], normals / np.where(lengths[:, None] < 1e-12, 1.0, lengths[:, None]), 0.0
    )
    to_cam = camera.eye - world[:, 0]
    facing = np.einsum("ij,ij->i", normals, to_cam) > 0
    keep = valid & facing
    world = world[keep]
    colors = colors[keep]
    normals = normals[keep]

    if len(world) == 0:
        return [], []

    centroid = world.mean(axis=1)
    view = camera.view(centroid)
    depth = -view[:, 2]

    proj = camera.project(camera.view(world.reshape(-1, 3)), width, height)
    proj = proj.reshape(-1, 3, 3)

    order = np.argsort(-depth)  # 远 -> 近
    out_verts: List[np.ndarray] = []
    out_colors: List[Tuple[float, float, float]] = []
    for idx in order:
        tri = proj[idx][:, :2]
        if not np.isfinite(tri).all():
            continue
        out_verts.append(tri)
        out_colors.append(_shade(normals[idx], colors[idx]))
    return out_verts, out_colors


def ground_lines(radius: float, camera: Camera, width: int, height: int) -> List[np.ndarray]:
    """地面网格（世界 z=0）的屏幕线段，供 LineCollection 使用。"""
    segs: List[np.ndarray] = []
    rings = [r * radius for r in (0.34, 0.67, 1.0)]
    for r in rings:
        ang = np.linspace(0.0, 2.0 * math.pi, 65)
        pts = np.column_stack([r * np.cos(ang), r * np.sin(ang), np.zeros_like(ang)])
        vp = camera.view(pts)
        valid = -vp[:, 2] > 1e-3
        sp = camera.project(vp, width, height)
        for i in range(len(pts) - 1):
            if valid[i] and valid[i + 1]:
                segs.append(np.array([sp[i, :2], sp[i + 1, :2]]))
    for k in range(8):
        a = math.pi * 2.0 * k / 8.0
        p0 = np.array([[0.0, 0.0, 0.0], [radius * math.cos(a), radius * math.sin(a), 0.0]])
        vp = camera.view(p0)
        if (-vp[:, 2] > 1e-3).all():
            sp = camera.project(vp, width, height)
            segs.append(sp[:, :2])
    return segs


# --------------------------------------------------------------------------- #
# 姿态与取景
# --------------------------------------------------------------------------- #


def working_pose(dof: int, seed: int) -> List[float]:
    """一个典型的"作业姿态"，按索引做小幅扰动，便于观察各轴运动方向。"""
    if dof == 7:
        base = [24.0, -34.0, 22.0, -44.0, 26.0, 18.0, 0.0]
    else:
        base = [34.0, -44.0, 58.0, -18.0, 24.0, 0.0]
    wobble = [6.0, 5.0, 8.0, 6.0, 7.0, 10.0, 8.0][:dof]
    sign = [1, -1, 1, -1, 1, 1, -1][:dof]
    phase = (seed % 5) / 4.0 - 0.5
    return [b + w * s * phase for b, w, s in zip(base, wobble, sign)]


def clamp_pose(pose: Sequence[float], limits_deg: Sequence[Tuple[float, float]]) -> List[float]:
    out = []
    for i, v in enumerate(pose):
        if i < len(limits_deg):
            lo, hi = limits_deg[i]
            out.append(min(hi, max(lo, v)))
        else:
            out.append(v)
    return out


def frame_camera(model: UrdfModel, pose: Sequence[float], reach_mm: float) -> Camera:
    """按机型尺寸自动取景：保证整机（含完全伸展范围）落在画面里。"""
    r = max(reach_mm, 200.0) / 1000.0
    tcp = tcp_position(model, pose)
    height = float(np.linalg.norm(tcp)) * 0.0 + r * 0.42
    target = np.array([0.0, 0.0, max(height, r * 0.30)])
    dist = r * 2.35 + 0.45
    az = math.radians(-38.0)
    el = math.radians(19.0)
    eye = target + dist * np.array(
        [math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)]
    )
    return Camera(eye, target, fov_deg=34.0)


# --------------------------------------------------------------------------- #
# 出图
# --------------------------------------------------------------------------- #

BG = "#15161c"


def draw_tile(ax, model: UrdfModel, pose: Sequence[float], spec) -> None:
    """在一个 Axes 里画单个机型。"""
    W, H = 300, 240
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_xticks([])
    ax.set_yticks([])
    for side in ax.spines.values():
        side.set_color("#3a3b46")
        side.set_linewidth(0.8)

    camera = frame_camera(model, pose, spec.reach_mm)
    reach_m = max(spec.reach_mm, 200.0) / 1000.0

    segs = ground_lines(reach_m * 1.08, camera, W, H)
    if segs:
        ax.add_collection(LineCollection(segs, colors="#33343f", linewidths=0.5, zorder=1))

    verts, colors = render_triangles(model, pose, camera, W, H)
    if verts:
        ax.add_collection(
            PolyCollection(
                verts,
                facecolors=colors,
                edgecolors=colors,
                linewidths=0.25,
                zorder=2,
            )
        )

    brand = BRANDS.get(spec.brand_id)
    brand_name = brand.name if brand else spec.brand_id
    accent = brand.primary if brand else (0.54, 0.54, 0.58)
    ax.text(
        6,
        H - 12,
        f"{brand_name} {spec.model_name}",
        color="#f2f3f7",
        fontsize=8.2,
        fontweight="bold",
        va="top",
        zorder=5,
    )
    ax.text(
        6,
        14,
        f"{spec.payload_kg:g} kg · {spec.reach_mm:g} mm · {spec.dof} 轴",
        color=accent,
        fontsize=7.4,
        va="bottom",
        zorder=5,
    )
    if not spec.verified:
        ax.text(
            W - 6,
            H - 12,
            "参数未核验",
            color="#d8a13a",
            fontsize=6.6,
            ha="right",
            va="top",
            zorder=5,
        )


def collect_targets(models_dir: Path) -> List[Tuple[str, Path, object]]:
    """（key, urdf 路径, spec）列表，只包含程序化生成的内建机型。"""
    out = []
    for spec in ROBOTS:
        path = models_dir / spec.brand_id / spec.model_id / f"{spec.model_id}.urdf"
        if not path.exists():
            path = models_dir / spec.brand_id / f"{spec.urdf_name}.urdf"
        if path.exists():
            out.append((spec.key, path, spec))
    return out


def make_figure(
    targets, out_path: Path, cols: int, cell_w: float, cell_h: float, title: str
) -> None:
    rows = math.ceil(len(targets) / cols)
    fig_w = cols * cell_w
    fig_h = rows * cell_h + 0.55
    fig, axes = plt.subplots(rows, cols, figsize=(fig_w, fig_h), dpi=110)
    fig.patch.set_facecolor(BG)
    axes = np.atleast_1d(axes).ravel()

    for i, ax in enumerate(axes):
        ax.set_facecolor("#20212a")
        if i >= len(targets):
            ax.axis("off")
            continue
        key, path, spec = targets[i]
        model = load_urdf(path)
        pose = clamp_pose(working_pose(spec.dof, i), spec.joint_limits_deg)
        draw_tile(ax, model, pose, spec)

    fig.suptitle(title, color="#f2f3f7", fontsize=12, y=0.985)
    fig.tight_layout(rect=(0, 0, 1, 0.965))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, facecolor=BG, dpi=110)
    plt.close(fig)
    print(f"已输出: {out_path}  ({len(targets)} 个机型)")


HERO_KEYS = (
    "kuka/kr16_2",
    "kuka/lbr_iiwa_7",
    "jaka/zu12",
    "abb/irb6700_150_320",
    "fanuc/r2000ic_165f",
    "ur/ur5e",
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--models-dir", default=str(REPO_ROOT / "assets" / "models"), help="模型根目录"
    )
    parser.add_argument("--out-dir", default=str(REPO_ROOT / "docs" / "preview"), help="输出目录")
    parser.add_argument("-o", "--out", default=None, help="单图输出路径（与 --layout 配合）")
    parser.add_argument("--layout", choices=("all", "hero", "both"), default="both")
    args = parser.parse_args()

    font = _setup_cjk_font()
    if not font:
        print("警告：未找到中文字体，预览图中的中文可能显示为方框", file=sys.stderr)

    models_dir = Path(args.models_dir)
    out_dir = Path(args.out_dir)

    targets = collect_targets(models_dir)
    if not targets:
        print("未找到任何内建 URDF，请先运行 tools/generate_robot_models.py", file=sys.stderr)
        return 1

    by_key = {k: (k, p, s) for k, p, s in targets}

    if args.layout in ("all", "both"):
        out = (
            Path(args.out)
            if (args.out and args.layout == "all")
            else out_dir / "robot_library_all.png"
        )
        make_figure(
            targets,
            out,
            cols=5,
            cell_w=3.0,
            cell_h=2.5,
            title=f"OpenRobot 机器人库 · {len(targets)} 个程序化生成机型（几何为示意模型，参数为厂商样本值）",
        )

    if args.layout in ("hero", "both"):
        hero = [by_key[k] for k in HERO_KEYS if k in by_key]
        out = (
            Path(args.out)
            if (args.out and args.layout == "hero")
            else out_dir / "robot_library_hero.png"
        )
        make_figure(
            hero, out, cols=3, cell_w=4.1, cell_h=3.4, title="机器人库代表机型 · 作业姿态预览"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
