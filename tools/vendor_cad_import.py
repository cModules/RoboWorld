#!/usr/bin/env python3
"""从厂商官网抓取官方 CAD 模型，转成可直接挂进 URDF 的 STL 网格。

为什么需要这个工具
-------------------
程序化生成（:mod:`adapter.robot_library.urdf_builder` / ``hifi.py``）产出的
是"像机器人"的曲面，比例对、结构对，但不是真机。要做离线编程，用户往往
需要**与真机一致**的几何——外形尺寸、法兰孔位、干涉轮廓都得准。

本工具走的是另一条路：直接抓厂商公开的 STEP 装配体，转成 STL。

已验证的厂商
------------
**JAKA（节卡）** —— 完全开放，无需登录。

    1. 分类树   ``GET /prod-api/api/download/treeList``
    2. 文件列表 ``GET /prod-api/api/download/selectDownloadFlatByParentId``
                ``?flag=all&parentId=<3D-STEP 下某系列的 id>``
    3. 文件本体 ``GET /prod-api{name}``（``name`` 形如
                ``/profile/upload/2023/11/02/xxx.zip``）

    3D-STEP → JAKA Zu系列 的节点 id 是 **91**，覆盖 Zu 3/5/7/12/18/20/30
    全部七个型号。完整覆盖我们机器人库里的 jaka/* 七个型号。

其他厂商（UR / KUKA / ABB / FANUC / Estun）的下载入口都在登录墙后面，
需要账号；本工具只做公开抓取，不处理登录。

坐标系与单位
------------
厂商 STEP 装配体与 URDF 的约定不同，转换时必须处理，否则模型会躺着或
放大 1000 倍：

=========  ====================  ==================
项目       厂商 STEP               URDF
=========  ====================  ==================
单位       毫米                    米
竖直轴     Y（X 前 / Z 右）      Z（X 前 / Y 左）
原点       装配原点（常在底座）  关节坐标系
=========  ====================  ==================

默认变换 ``(x, y, z)_step → (x, -z, y)_urdf / 1000``，保持右手系
（验证：X'×Y' = (1,0,0)×(0,0,-1) = (0,1,0) = Z'）。

依赖
----
转换需要 OpenCASCADE 的 Python 绑定::

    pip install cadquery-ocp

注意 PyPI 上的 ``OCP`` 是同名但无关的 "Open Collaboration Platform"，
**不是** OpenCASCADE 绑定，装错了 import 会失败。

用法::

    python3 tools/vendor_cad_import.py --list                 # 列出可抓的型号
    python3 tools/vendor_cad_import.py --brand jaka --fetch   # 下载 STEP
    python3 tools/vendor_cad_import.py --brand jaka --convert # STEP → STL
    python3 tools/vendor_cad_import.py --brand jaka --all     # 一条龙

产物落在 ``assets/vendor/<brand>/``（该目录被 .gitignore 忽略，体积大且
可随时重新生成；需要入库的减面结果请另存到 assets/models/）。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.parse
import urllib.request
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

REPO_ROOT = Path(__file__).resolve().parent.parent
#: 抓取产物根目录。体量大且可重新生成，不进版本库。
VENDOR_DIR = REPO_ROOT / "assets" / "vendor"

# 复用库侧的 OCP 版本兼容层（TopoDS.Face_s → Face 这类改名只在那里改一次），
# 见 core/ocpcompat.py 的模块说明。
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from adapter.ocpcompat import bbox_of, face_of, triangulation_of  # noqa: E402

USER_AGENT = "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 Chrome/120 Safari/537.36"


# --------------------------------------------------------------------------- #
# 厂商配置
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class VendorSource:
    """一个可公开抓取的厂商下载源。"""

    brand: str
    base: str
    #: 分类树接口
    tree_url: str
    #: 文件清单接口模板，``{id}`` 会被替换成系列节点 id
    list_url_tpl: str
    #: 我们关心的分类路径，例如 ("模型及图纸", "3D-STEP", "JAKA Zu系列")
    #: 留空表示取该 id 下全部
    category_path: Tuple[str, ...] = ()
    #: 直接指定节点 id（省去按名字查找）
    category_id: Optional[int] = None
    #: 备注
    note: str = ""


#: 已验证可用的厂商源。新增厂商时在这里加一条即可。
VENDORS: Dict[str, VendorSource] = {
    "jaka": VendorSource(
        brand="jaka",
        base="https://www.jaka.com",
        tree_url="https://www.jaka.com/prod-api/api/download/treeList",
        list_url_tpl=(
            "https://www.jaka.com/prod-api/api/download/"
            "selectDownloadFlatByParentId?flag=all&parentId={id}"
        ),
        category_path=("模型及图纸", "3D-STEP", "JAKA Zu系列"),
        category_id=91,
        note="完全开放，无需登录；覆盖 Zu 3/5/7/12/18/20/30 共 7 个型号",
    ),
}


# --------------------------------------------------------------------------- #
# HTTP
# --------------------------------------------------------------------------- #
def _get(url: str, timeout: float = 60.0) -> bytes:
    """GET 一个 URL 并返回原始字节。

    Args:
        url: 目标地址。
        timeout: 超时秒数。

    Returns:
        响应体。
    """
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.read()


def _get_json(url: str, timeout: float = 60.0):
    """GET 并解析 JSON。"""
    return json.loads(_get(url, timeout).decode("utf-8", errors="replace"))


# --------------------------------------------------------------------------- #
# 抓取
# --------------------------------------------------------------------------- #
def find_category_id(src: VendorSource) -> int:
    """按分类路径解析出系列节点 id。

    Args:
        src: 厂商源。

    Returns:
        节点 id。

    Raises:
        SystemExit: 路径未命中。
    """
    if src.category_id is not None:
        return src.category_id

    tree = _get_json(src.tree_url).get("data") or []

    def walk(nodes: Sequence[dict], path: Tuple[str, ...]) -> Optional[int]:
        for n in nodes:
            name = str(n.get("name") or "")
            if path and name == path[0]:
                if len(path) == 1:
                    return n.get("id")
                hit = walk(n.get("children") or [], path[1:])
                if hit is not None:
                    return hit
        return None

    found = walk(tree, src.category_path)
    if found is None:
        raise SystemExit(f"未能在 {src.brand} 分类树中找到 {src.category_path}")
    return found


@dataclass
class Asset:
    """一个可下载的原始文件。"""

    title: str
    path: str  # 形如 /profile/upload/2023/11/02/xxx.zip
    url: str = ""

    def __post_init__(self) -> None:
        if not self.url:
            # path 里含空格等字符，必须逐段 quote（不能 quote 整个 URL）
            self.url = "https://www.jaka.com/prod-api" + urllib.parse.quote(self.path)


def list_assets(src: VendorSource) -> List[Asset]:
    """列出某厂商分类下的全部可下载文件。

    Args:
        src: 厂商源。

    Returns:
        文件条目列表。
    """
    cid = find_category_id(src)
    data = _get_json(src.list_url_tpl.format(id=cid)).get("data") or {}
    out: List[Asset] = []
    for group in data.get("secondLevelDownloadTypeVos") or []:
        for item in group.get("downloadVos") or []:
            files = item.get("fileList") or []
            if not files:
                continue
            out.append(
                Asset(
                    title=str(item.get("title") or ""),
                    path=str(files[0].get("name") or ""),
                )
            )
    return out


def fetch(src: VendorSource, outdir: Path) -> List[Path]:
    """下载原始压缩包并解出 STEP。

    Args:
        src: 厂商源。
        outdir: 输出目录。

    Returns:
        解出来的 STEP 文件路径列表。
    """
    outdir.mkdir(parents=True, exist_ok=True)
    raw = outdir / "_raw"
    step_dir = outdir / "step"
    raw.mkdir(exist_ok=True)
    step_dir.mkdir(exist_ok=True)

    assets = list_assets(src)
    print(f"{src.brand}: 发现 {len(assets)} 个文件")
    steps: List[Path] = []
    for a in assets:
        if not a.path:
            continue
        name = a.title.replace(" ", "_").replace("/", "-") or "asset"
        zpath = raw / f"{name}.zip"
        if not zpath.exists():
            data = _get(a.url, timeout=300)
            zpath.write_bytes(data)
            print(f"  ↓ {zpath.name}  {len(data)/1024:.0f} KB")
        else:
            print(f"  · {zpath.name}  (已存在，跳过)")
        try:
            with zipfile.ZipFile(zpath) as zf:
                for entry in zf.namelist():
                    if entry.lower().endswith((".step", ".stp")):
                        target = step_dir / Path(entry).name.replace(" ", "_")
                        target.write_bytes(zf.read(entry))
                        steps.append(target)
        except zipfile.BadZipFile:
            print(f"  ! {zpath.name} 不是有效 ZIP，跳过", file=sys.stderr)
    return sorted(steps)


# --------------------------------------------------------------------------- #
# STEP → STL
# --------------------------------------------------------------------------- #
def _import_occ() -> None:
    """检查 OpenCASCADE 绑定是否可用，缺失时给出可操作的报错。

    用 :mod:`importlib` 只探测不导入：真正 import ``OCP`` 要花几百毫秒，
    而这里的目的只是尽早给出一个清楚的错误提示。
    """
    import importlib.util

    if importlib.util.find_spec("OCP") is None:
        raise SystemExit(
            "缺少 OpenCASCADE 绑定。请执行：\n"
            "    pip install cadquery-ocp\n"
            "（注意：PyPI 上的 'OCP' 是同名无关项目，装它没有用）"
        )


def convert(
    step_path: Path,
    outdir: Path,
    rel_deflection: float = 0.002,
    angular_deflection: float = 0.5,
    scale: float = 0.001,
    swap_yz: bool = True,
) -> Tuple[int, int]:
    """把一个 STEP 装配体按顶层组件分组转成 STL。

    顶层组件通常就是装配树第一层（底座 / 各连杆），保留这层分组，后面才能
    把网格挂到 URDF 的对应连杆上。

    Args:
        step_path: STEP 文件。
        outdir: 输出目录。
        rel_deflection: 相对弦高，按包围盒对角线比例。0.002 ≈ 对角线的
            0.2%，对 1 m 的臂约 2 mm 误差——屏幕上看不出来，但比固定弦高
            更能自适应大小机型。
        angular_deflection: 角度弦高（弧度）。
        scale: 单位缩放，厂商用毫米而 URDF 用米，故默认 0.001。
        swap_yz: 是否把 Y-up 转成 Z-up。

    Returns:
        ``(组件数, 三角面总数)``。
    """
    from OCP.STEPControl import STEPControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopoDS import TopoDS_Iterator
    from OCP.StlAPI import StlAPI_Writer
    from OCP.TopLoc import TopLoc_Location
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Trsf, gp_Pnt, gp_Dir, gp_Ax3

    reader = STEPControl_Reader()
    if reader.ReadFile(str(step_path)) != IFSelect_RetDone:
        raise SystemExit(f"STEP 读取失败: {step_path}")
    reader.TransferRoots()
    root = reader.OneShape()

    # 先把整个装配体变换到 URDF 约定：Y-up mm → Z-up m。
    # gp_Ax3(原点, 主方向/Z, X 方向)，Y 由右手系导出（Y = Z × X）。
    #   src: 主方向取原 Y（竖直轴），X 不变
    #   dst: 主方向取新 Z（竖直轴），X 不变
    # 实际点映射为 (x,y,z) → (x,z,-y)：源的竖直轴 Y 落到目标的竖直轴 Z 上、方向相反，
    # 右手系保持（det > 0，不是镜像）；轴符号由已验证的渲染结果决定。
    trsf = gp_Trsf()
    if swap_yz:
        src_ax = gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0), gp_Dir(1, 0, 0))
        dst_ax = gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0))
        trsf.SetTransformation(src_ax, dst_ax)
    # 缩放单独一层，避免与换轴互相干扰；A * B 表示"先 B 后 A"
    scale_trsf = gp_Trsf()
    scale_trsf.SetScaleFactor(scale)
    root = BRepBuilderAPI_Transform(root, scale_trsf * trsf, True).Shape()

    it = TopoDS_Iterator(root)
    comps = []
    while it.More():
        comps.append(it.Value())
        it.Next()

    outdir.mkdir(parents=True, exist_ok=True)
    total = 0
    for i, comp in enumerate(comps):
        BRepMesh_IncrementalMesh(comp, rel_deflection, True, angular_deflection, True)
        n = 0
        fx = TopExp_Explorer(comp, TopAbs_FACE)
        while fx.More():
            face = face_of(fx.Current())
            loc = TopLoc_Location()
            tri = triangulation_of(face, loc)
            if tri is not None:
                n += tri.NbTriangles()
            fx.Next()

        bb = bbox_of(comp)
        if bb is None:
            continue
        lo, hi = bb

        out = outdir / f"grp_{i:02d}.stl"
        writer = StlAPI_Writer()
        writer.ASCIIMode = False
        writer.Write(comp, str(out))
        total += n
        print(
            f"    grp_{i:02d}  tris={n:<7} {out.stat().st_size/1024:>7.0f}KB  "
            f"x[{lo[0]:+.3f},{hi[0]:+.3f}] y[{lo[1]:+.3f},{hi[1]:+.3f}] "
            f"z[{lo[2]:+.3f},{hi[2]:+.3f}]"
        )
    return len(comps), total


def convert_all(
    step_dir: Path,
    outdir: Path,
    rel_deflection: float = 0.002,
    angular_deflection: float = 0.5,
) -> None:
    """批量转换一个目录下的全部 STEP。

    Args:
        step_dir: STEP 所在目录。
        outdir: 输出根目录，每个型号一个子目录。
        rel_deflection: 相对弦高。
        angular_deflection: 角度弦高（弧度）。
    """
    steps = sorted(step_dir.glob("*.STEP")) + sorted(step_dir.glob("*.step"))
    if not steps:
        print(f"目录里没有 STEP 文件: {step_dir}", file=sys.stderr)
        return
    for s in steps:
        t0 = time.time()
        print(f"  → {s.name}", flush=True)
        n, tris = convert(
            s,
            outdir / s.stem.replace(" ", "_"),
            rel_deflection=rel_deflection,
            angular_deflection=angular_deflection,
        )
        print(f"     {n} 个组件 / {tris} 三角面 / {time.time()-t0:.1f}s", flush=True)


# --------------------------------------------------------------------------- #
# 入口
# --------------------------------------------------------------------------- #
def main() -> int:
    parser = argparse.ArgumentParser(description="从厂商官网抓取官方 CAD 并转成 STL")
    parser.add_argument(
        "--brand", default="jaka", choices=sorted(VENDORS), help="厂商（目前仅 jaka 完全开放）"
    )
    parser.add_argument("--list", action="store_true", help="列出可抓的型号")
    parser.add_argument("--fetch", action="store_true", help="下载原始 STEP")
    parser.add_argument("--convert", action="store_true", help="STEP → STL")
    parser.add_argument("--all", action="store_true", help="抓取 + 转换")
    parser.add_argument("--out", default=str(VENDOR_DIR), help="产物根目录")
    parser.add_argument(
        "--rel-deflection", type=float, default=0.002, help="相对弦高（默认 0.002，越小越精细）"
    )
    parser.add_argument(
        "--angular-deflection", type=float, default=0.5, help="角度弦高，弧度（默认 0.5）"
    )
    args = parser.parse_args()

    src = VENDORS[args.brand]
    outdir = Path(args.out) / args.brand
    step_dir = outdir / "step"
    stl_dir = outdir / "stl"

    if not (args.list or args.fetch or args.convert or args.all):
        parser.print_help()
        return 0

    if args.list:
        print(f"{src.brand}: {src.note}")
        for a in list_assets(src):
            print(f"  - {a.title}")
            print(f"      {a.url}")
        return 0

    if args.fetch or args.all:
        try:
            steps = fetch(src, outdir)
        except Exception as exc:  # pragma: no cover - 网络问题
            print(f"抓取失败: {exc}", file=sys.stderr)
            return 1
        print(f"解出 {len(steps)} 个 STEP")

    if args.convert or args.all:
        _import_occ()
        convert_all(
            step_dir,
            stl_dir,
            rel_deflection=args.rel_deflection,
            angular_deflection=args.angular_deflection,
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())
