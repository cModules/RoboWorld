"""OCP（OpenCASCADE 的 Python 绑定）版本兼容层。

为什么单独有一层
----------------
OCP 8（对应 OCCT 8）去掉了若干「静态转型函数」的 ``_s`` 后缀::

    OCP 7.x :  TopoDS.Face_s(shape)        OCP 8.x :  TopoDS.Face(shape)
               TopoDS.Edge_s(shape)                   TopoDS.Edge(shape)

但**不是所有** ``_s`` 都改了——``BRep_Tool.Triangulation_s`` 与
``BRepBndLib.Add_s`` 在 8.0.1 里依然存在。这个「改一半」正是坑之所在：只看
版本号去批量替换会把仍然正确的调用改坏，只看某个文件里的既有写法又会漏掉。

历史包袱
--------
仓库里装好 ``cadquery-ocp 8.0.1`` 之后，``tools/build_vendor_urdf.py`` 与
``tools/vendor_cad_import.py`` 的网格化分支会当场 ``AttributeError``；而这条
链路的验收测试此前是「本地没有厂商 STEP 就 skip」，于是缺陷一直躲在 skip 后面，
连续好几轮都没被任何自动化发现（v0.3.2 那轮只给 ``orolp/cam.py`` 做了兼容，
工具侧留了下来）。现在 ``tests/test_vendor_step_pipeline.py`` 用代码**合成**一个
STEP 装配体，让整条链在没有厂商 CAD 的情况下也真跑一遍，这类改名从此会当场被抓住。

设计
----
把「版本相关的符号名」收在这一处：下次 OCP 再改名只动这个文件。解析结果按名字
缓存（一次 resolve，后续直接调）。

OCP 是**软依赖**——本模块在 import 期不加载 OCP，只有真正调用时才 import，
所以没装 OCP 的机器上 ``import orolp.ocpcompat`` 照样成功，也不会拖慢启动。

用法::

    from orolp.ocpcompat import has_ocp, require_ocp, face_of, edge_of, bbox_of

    if has_ocp():
        face = face_of(explorer.Current())
        tri = triangulation_of(face, loc)
"""

from __future__ import annotations

from typing import Any, Callable, Dict, Optional, Tuple

import numpy as np

__all__ = [
    "has_ocp",
    "require_ocp",
    "describe",
    "topods",
    "face_of",
    "edge_of",
    "triangulation_of",
    "bbox_of",
]

_INSTALL_HINT = (
    "缺少 OpenCASCADE 绑定。请执行：\n"
    "    pip install cadquery-ocp\n"
    "（注意：PyPI 上的 'OCP' 是同名无关项目，装它没有用）"
)

# 已解析的转型函数 / 函数名缓存：{逻辑名: (实际属性名, 可调用对象)}
_resolved: Dict[str, Tuple[str, Callable]] = {}


# --------------------------------------------------------------------------- #
# 可用性探测（只探测不导入：真 import OCP 要几百毫秒）
# --------------------------------------------------------------------------- #
def has_ocp() -> bool:
    """当前环境是否装了 OCP 绑定（不触发真正的 import）。"""
    import importlib.util

    try:
        return importlib.util.find_spec("OCP") is not None
    except (ImportError, ValueError):  # pragma: no cover - 解释器环境异常
        return False


def require_ocp() -> None:
    """确认 OCP 可用，否则给出可操作的报错。"""
    if not has_ocp():
        raise ImportError(_INSTALL_HINT)


def _distribution_version() -> Optional[str]:
    try:
        from importlib.metadata import version

        return version("cadquery-ocp")
    except Exception:  # pragma: no cover - 元数据缺失/被裁剪
        return None


def topods():
    """返回 ``OCP.TopoDS.TopoDS`` 模块对象（转型函数的宿主）。"""
    require_ocp()
    from OCP.TopoDS import TopoDS as _TopoDS

    return _TopoDS


def _resolve(module: Any, logical: str, *attrs: str) -> Tuple[str, Callable]:
    """按 ``attrs`` 的顺序在 ``module`` 上找第一个存在的属性。

    顺序即优先级：把**新名放前面**，旧名兜底，这样两代绑定都能跑。
    """
    if logical in _resolved:
        return _resolved[logical]
    for attr in attrs:
        fn = getattr(module, attr, None)
        if fn is not None:
            _resolved[logical] = (attr, fn)
            return _resolved[logical]
    raise AttributeError(
        f"OCP 里找不到 {'/'.join(attrs)}（用于 {logical}）——"
        f"当前绑定版本可能又改了名字，请更新 orolp/ocpcompat.py"
    )


def _cast(logical: str, *attrs: str) -> Callable:
    return _resolve(topods(), logical, *attrs)[1]


# --------------------------------------------------------------------------- #
# 转型：TopoDS_Shape → 具体类型
# --------------------------------------------------------------------------- #
def face_of(shape) -> Any:
    """把 ``TopoDS_Shape`` 转成 ``TopoDS_Face``。

    OCP 8 起是 ``TopoDS.Face``，OCP 7.x 是 ``TopoDS.Face_s``。
    """
    return _cast("Face", "Face", "Face_s")(shape)


def edge_of(shape) -> Any:
    """把 ``TopoDS_Shape`` 转成 ``TopoDS_Edge``。

    OCP 8 起是 ``TopoDS.Edge``，OCP 7.x 是 ``TopoDS.Edge_s``。
    """
    return _cast("Edge", "Edge", "Edge_s")(shape)


def triangulation_of(face, loc) -> Any:
    """取面在给定 ``TopLoc_Location`` 下的三角剖分（尚未网格化时返回 ``None``）。

    ``BRep_Tool.Triangulation_s`` 在 OCP 7/8 都存在，这里同样按「新名优先」
    解析，免得哪天真去掉了后缀再回头改。
    """
    require_ocp()
    from OCP.BRep import BRep_Tool

    return _resolve(BRep_Tool, "Triangulation", "Triangulation", "Triangulation_s")[1](face, loc)


def bbox_of(shape) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """世界系轴对齐包围盒，返回 ``(min_xyz, max_xyz)``；空形状返回 ``None``。

    ``BRepBndLib.Add_s`` 在 OCP 7/8 都存在，解析方式同上。
    """
    require_ocp()
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib

    add = _resolve(BRepBndLib, "Add", "Add", "Add_s")[1]
    box = Bnd_Box()
    add(shape, box)
    if box.IsVoid():
        return None
    mn, mx = box.CornerMin(), box.CornerMax()
    lo = np.array([mn.X(), mn.Y(), mn.Z()], dtype=np.float64)
    hi = np.array([mx.X(), mx.Y(), mx.Z()], dtype=np.float64)
    return lo, hi


# --------------------------------------------------------------------------- #
# 诊断
# --------------------------------------------------------------------------- #
def describe() -> Dict[str, Any]:
    """当前绑定的版本与「每个逻辑符号实际解析成了哪个属性名」。

    给 ``--probe`` 与测试用：换机器/换绑定版本时，一眼能看出解析结果是否符合预期。
    """
    info: Dict[str, Any] = {
        "available": has_ocp(),
        "distribution": "cadquery-ocp",
        "version": _distribution_version(),
        "casts": {},
        "functions": {},
    }
    if not info["available"]:
        return info

    mod = topods()
    info["casts"]["Face"] = _resolve(mod, "Face", "Face", "Face_s")[0]
    info["casts"]["Edge"] = _resolve(mod, "Edge", "Edge", "Edge_s")[0]

    from OCP.BRep import BRep_Tool
    from OCP.BRepBndLib import BRepBndLib

    info["functions"]["Triangulation"] = _resolve(
        BRep_Tool, "Triangulation", "Triangulation", "Triangulation_s"
    )[0]
    info["functions"]["Add"] = _resolve(BRepBndLib, "Add", "Add", "Add_s")[0]
    return info
