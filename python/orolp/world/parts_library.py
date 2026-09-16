"""零件库：自定义参数化零件的导入与管理（PRD §4.1 P1）。

内置组件目录见 :data:`~orolp.world.components.COMPONENTS`（加工工件、夹具、
输送设备等 19 种参数化组件，拖到视口即加入场景）。本模块补上 PRD 里
零件库的另一半——**自定义导入**：用户把常用的加工工件/结构件（原始
基本体的组合）写成一个 JSON 零件库文件，导入后作为常规组件出现在
「场景设备库」的自定义分类里，可拖拽、可入场景、可随场景保存。

诚实性口径（写进文档与页面）：

* 本模块管理的是**参数化基本体**（box / cylinder / sphere 的组合），
  与内置组件同一几何通道；**不解析 STL/STEP 网格**——网格件走
  机器人 URDF / 厂商 CAD 通路（v0.2.3），两条通道不混装；
* 导入的零件几何是**固定的**（文件里写什么就是什么），不像内置组件
  那样带可调参数——参数化定制是后续项，这里绝不假装可调。

JSON schema：``orolp.parts/1``。坏 schema、未知基本体类型、非法尺寸、
重复 id 都当场抛 ``ValueError``，绝不静默丢零件。
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..i18n import tr
from .components import COMPONENTS, ComponentSpec, Part
from .geometry import Box, Cylinder, Primitive, Sphere

__all__ = [
    "PRIMITIVE_KINDS",
    "PrimitiveSpec",
    "CustomPartSpec",
    "CustomPartsLibrary",
    "register_custom_parts",
    "unregister_custom_parts",
    "CUSTOM_CATEGORY",
    "CUSTOM_ID_PREFIX",
]

#: 支持的基本体类型（与内置组件同一词汇表）
PRIMITIVE_KINDS = ("box", "cylinder", "sphere")

#: 自定义零件在组件目录里的分类名
CUSTOM_CATEGORY = "自定义"

#: 自定义零件 id 前缀（注册进 COMPONENTS 时加，避免与内置 id 撞名）
CUSTOM_ID_PREFIX = "custom_"

_VALID_ID = re.compile(r"^[A-Za-z][A-Za-z0-9_\-]{0,63}$")


@dataclass
class PrimitiveSpec:
    """一个基本体的描述（零件局部坐标系）。

    Attributes:
        kind: ``"box"`` / ``"cylinder"`` / ``"sphere"``
        size: box 的 (长, 宽, 高) mm；仅 box 用
        radius: 半径 mm；cylinder / sphere 用
        height: 高 mm；仅 cylinder 用（圆柱轴沿 Z，与内置组件一致）
        offset: 相对零件原点的平移 (x, y, z) mm
        rpy_deg: 相对零件原点的旋转（绕固定轴 XYZ 的欧拉角，度）
        color: RGB 0~1
        name: 该基本体的可选备注名
    """

    kind: str
    size: Optional[Tuple[float, float, float]] = None
    radius: float = 0.0
    height: float = 0.0
    offset: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    color: Tuple[float, float, float] = (0.30, 0.46, 0.62)
    name: str = ""

    def validate(self) -> None:
        if self.kind not in PRIMITIVE_KINDS:
            raise ValueError(
                tr("未知基本体类型 {kind}（支持 {opts}）").format(
                    kind=self.kind, opts="/".join(PRIMITIVE_KINDS)
                )
            )
        if self.kind == "box":
            if self.size is None or len(self.size) != 3:
                raise ValueError(
                    tr("box 基本体需要 size=(长, 宽, 高)，零件 {name}").format(name=self.name)
                )
            if any(not float(s) > 0.0 for s in self.size):
                raise ValueError(
                    tr("box 尺寸必须为正，收到 {size}").format(size=tuple(self.size))
                )
        else:
            if not float(self.radius) > 0.0:
                raise ValueError(
                    tr("{kind} 基本体需要正的 radius，零件 {name}").format(
                        kind=self.kind, name=self.name
                    )
                )
        if self.kind == "cylinder" and not float(self.height) > 0.0:
            raise ValueError(
                tr("cylinder 基本体需要正的 height，零件 {name}").format(name=self.name)
            )
        for c in self.color:
            if not 0.0 <= float(c) <= 1.0:
                raise ValueError(tr("颜色分量必须在 0~1 之间，收到 {c}").format(c=c))

    def to_part(self) -> Part:
        """生成 :class:`~orolp.world.components.Part`。"""
        self.validate()
        if self.kind == "box":
            prim: Primitive = Box(tuple(float(s) for s in self.size))
        elif self.kind == "cylinder":
            prim = Cylinder(float(self.radius), float(self.height))
        else:
            prim = Sphere(float(self.radius))

        m = _rpy_matrix(self.rpy_deg)
        m[:3, 3] = [float(v) for v in self.offset]
        color = tuple(float(c) for c in self.color)
        return Part(prim, m, color)


def _rpy_matrix(rpy_deg: Sequence[float]) -> np.ndarray:
    """固定轴 XYZ 欧拉角（度）→ 4x4 变换（Rz·Ry·Rx，与 Pose 同口径）。"""
    r, p, y = (float(a) for a in rpy_deg)
    cr, sr = np.cos(np.radians(r)), np.sin(np.radians(r))
    cp, sp = np.cos(np.radians(p)), np.sin(np.radians(p))
    cy, sy = np.cos(np.radians(y)), np.sin(np.radians(y))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    m = np.eye(4)
    m[:3, :3] = rz @ ry @ rx
    return m


@dataclass
class CustomPartSpec:
    """一个自定义零件：若干基本体的固定组合。

    ``id`` 必须是字母开头的短标识符（注册进组件目录时会加 ``custom_``
    前缀，因此与内置组件不会撞名）。
    """

    id: str
    name: str
    primitives: List[PrimitiveSpec]
    category: str = CUSTOM_CATEGORY
    description: str = ""

    def validate(self) -> None:
        if not _VALID_ID.match(self.id or ""):
            raise ValueError(
                tr("零件 id 必须是字母开头的短标识符（字母/数字/_/-），收到 {id}").format(id=self.id)
            )
        if not self.name.strip():
            raise ValueError(tr("零件缺少显示名（{id}）").format(id=self.id))
        if not self.primitives:
            raise ValueError(tr("零件 {name} 没有任何几何基本体").format(name=self.name))
        seen_names: Dict[str, int] = {}
        for prim in self.primitives:
            prim.validate()
            if prim.name:
                seen_names[prim.name] = seen_names.get(prim.name, 0) + 1
        dupes = [n for n, c in seen_names.items() if c > 1]
        if dupes:
            raise ValueError(
                tr("零件 {name} 里有重名的基本体：{dupes}").format(
                    name=self.name, dupes="、".join(sorted(dupes))
                )
            )

    def to_part_list(self) -> List[Part]:
        return [p.to_part() for p in self.primitives]


class CustomPartsLibrary:
    """一个零件库文件的内容：若干 :class:`CustomPartSpec`。

    用法（也见 ``examples/assets/custom_parts.json``）::

        lib = CustomPartsLibrary.load_json("my_parts.json")
        ids = register_custom_parts(lib)   # 注册进组件目录，GUI 即可拖拽
        ...
        unregister_custom_parts(ids)       # 测试/换库时清理
    """

    SCHEMA = "orolp.parts/1"

    def __init__(self, parts: Optional[Sequence[CustomPartSpec]] = None) -> None:
        self.parts: List[CustomPartSpec] = list(parts) if parts else []

    def add(self, spec: CustomPartSpec) -> None:
        spec.validate()
        if any(p.id == spec.id for p in self.parts):
            raise ValueError(tr("零件库中已存在同名零件 id：{id}").format(id=spec.id))
        self.parts.append(spec)

    def remove(self, part_id: str) -> CustomPartSpec:
        for i, p in enumerate(self.parts):
            if p.id == part_id:
                return self.parts.pop(i)
        raise ValueError(tr("零件库中没有 id 为 {id} 的零件").format(id=part_id))

    # ---------------- JSON 往返 ---------------- #

    def to_dict(self) -> Dict[str, object]:
        return {
            "schema": self.SCHEMA,
            "parts": [
                {
                    "id": p.id,
                    "name": p.name,
                    "category": p.category,
                    "description": p.description,
                    "primitives": [
                        {
                            "kind": pr.kind,
                            **({"size": list(pr.size)} if pr.kind == "box" else {}),
                            **({"radius": pr.radius} if pr.kind in ("cylinder", "sphere") else {}),
                            **({"height": pr.height} if pr.kind == "cylinder" else {}),
                            "offset": list(pr.offset),
                            "rpy_deg": list(pr.rpy_deg),
                            "color": list(pr.color),
                            **({"name": pr.name} if pr.name else {}),
                        }
                        for pr in p.primitives
                    ],
                }
                for p in self.parts
            ],
        }

    @classmethod
    def from_dict(cls, data: Dict[str, object]) -> "CustomPartsLibrary":
        schema = str(data.get("schema", ""))
        if schema != cls.SCHEMA:
            raise ValueError(
                tr("不支持的零件库 schema：{schema}（期望 {want}）").format(
                    schema=schema or tr("（缺失）"), want=cls.SCHEMA
                )
            )
        lib = cls()
        for raw in data.get("parts", []) or []:
            spec = CustomPartSpec(
                id=str(raw.get("id", "")),
                name=str(raw.get("name", "")),
                category=str(raw.get("category") or CUSTOM_CATEGORY),
                description=str(raw.get("description") or ""),
                primitives=[
                    PrimitiveSpec(
                        kind=str(pr.get("kind", "")),
                        size=tuple(pr["size"]) if pr.get("size") is not None else None,
                        radius=float(pr.get("radius") or 0.0),
                        height=float(pr.get("height") or 0.0),
                        offset=tuple(float(v) for v in pr.get("offset") or (0, 0, 0)),
                        rpy_deg=tuple(float(v) for v in pr.get("rpy_deg") or (0, 0, 0)),
                        color=tuple(float(v) for v in pr.get("color") or (0.30, 0.46, 0.62)),
                        name=str(pr.get("name") or ""),
                    )
                    for pr in raw.get("primitives", []) or []
                ],
            )
            lib.add(spec)
        return lib

    def save_json(self, path: str | Path) -> None:
        Path(path).write_text(
            json.dumps(self.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8"
        )

    @classmethod
    def load_json(cls, path: str | Path) -> "CustomPartsLibrary":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls.from_dict(data)


def register_custom_parts(
    library: CustomPartsLibrary, registry: Optional[Dict[str, ComponentSpec]] = None
) -> List[str]:
    """把零件库注册进组件目录（默认 :data:`COMPONENTS`）。

    Returns:
        注册后的组件 id 列表（带 ``custom_`` 前缀）。调用方应保存它，
        供 :func:`unregister_custom_parts` 清理。

    Raises:
        ValueError: 某个零件不合法，或生成的组件 id 已在目录里。
    """
    reg = COMPONENTS if registry is None else registry
    ids: List[str] = []
    for part in library.parts:
        part.validate()
        cid = CUSTOM_ID_PREFIX + part.id
        if cid in reg:
            raise ValueError(
                tr("组件目录里已存在 {cid}——先 unregister 或换一个零件 id").format(cid=cid)
            )
        reg[cid] = ComponentSpec(
            id=cid,
            name=part.name,
            category=part.category,
            params=(),  # 固定几何：文件里写什么就是什么（诚实口径，见模块 docstring）
            builder=lambda _part=part, **_: _part.to_part_list(),
            description=part.description
            or tr("自定义零件（来自零件库导入，几何固定）"),
        )
        ids.append(cid)
    return ids


def unregister_custom_parts(
    ids: Sequence[str], registry: Optional[Dict[str, ComponentSpec]] = None
) -> None:
    """把 :func:`register_custom_parts` 注册的组件从目录里移除。

    只移除确实存在且带 ``custom_`` 前缀的条目；重复/不存在都安静跳过，
    方便「导入前先清理上一批」的幂等用法。
    """
    reg = COMPONENTS if registry is None else registry
    for cid in ids:
        if cid.startswith(CUSTOM_ID_PREFIX):
            reg.pop(cid, None)
