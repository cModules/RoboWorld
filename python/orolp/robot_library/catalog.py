"""机器人库目录 (Robot catalog)。

提供对标商业 OLP 软件 eCatalog 的检索能力：扫描 ``models/`` 目录建立型号索引，
支持按品牌 / 关键字搜索，按负载、臂展、轴数、机型筛选与排序。

两类条目
--------
* **内置型号**：来自 :mod:`.specs` 规格库，参数完整可信；若对应 URDF 尚未生成，
  首次访问时会调用 :mod:`.urdf_builder` **按需即时生成**（自愈，无需手工跑脚本）。
* **外部模型**：用户自行放置的 URDF。规格参数未知，仅从文件解析出轴数与关节限位，
  负载/臂展等显示为 ``—``。

典型用法::

    catalog = RobotCatalog.from_repo_root(Path("."))
    for e in catalog.search("kuka"):
        print(e.display_name, e.payload_kg, e.urdf_path)
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence

from .specs import BRANDS, ROBOTS, BrandSpec, RobotSpec
from .urdf_builder import build_urdf_string

__all__ = ["CatalogEntry", "RobotCatalog"]


# --------------------------------------------------------------------------- #
# 目录条目
# --------------------------------------------------------------------------- #
@dataclass
class CatalogEntry:
    """库中的一个机器人型号条目。"""

    key: str  # "brand/model"
    brand_id: str
    brand_name: str  # "KUKA"
    brand_name_cn: str  # "库卡"
    model_id: str
    model_name: str  # "KR 16-2"
    urdf_path: Optional[Path]
    dof: int
    source: str  # 'internal' | 'external'
    payload_kg: Optional[float] = None
    reach_mm: Optional[float] = None
    repeatability_mm: Optional[float] = None
    weight_kg: Optional[float] = None
    ip_rating: Optional[str] = None
    robot_type: str = "industrial"
    controller: Optional[str] = None
    verified: bool = False
    note: str = ""
    joint_limits_deg: Optional[Sequence] = None
    joint_speeds_deg_s: Optional[Sequence] = None
    color: tuple = (0.7, 0.7, 0.75)

    # ------------------------------------------------------------------ #
    @property
    def display_name(self) -> str:
        """展示名，如 ``KUKA KR 16-2``。"""
        return f"{self.brand_name} {self.model_name}"

    @property
    def brand_display(self) -> str:
        """品牌展示名，如 ``库卡 KUKA``。"""
        cn = self.brand_name_cn
        return f"{cn} {self.brand_name}" if cn and cn != self.brand_name else self.brand_name

    @property
    def is_internal(self) -> bool:
        """是否为内置（带完整规格）型号。"""
        return self.source == "internal"

    @property
    def hex_color(self) -> str:
        """品牌主色的 CSS 十六进制表示，用于 UI 上色。"""
        r, g, b = (int(round(c * 255)) for c in self.color)
        return f"#{r:02X}{g:02X}{b:02X}"

    def summary_lines(self) -> List[str]:
        """属性面板用的规格摘要（每项 ``"标签: 值"``）。"""
        rows = [
            ("品牌", self.brand_display),
            ("型号", self.model_name),
            ("轴数", f"{self.dof}"),
            ("机型", {"industrial": "工业臂", "cobot": "协作臂", "scara": "SCARA"}.get(
                self.robot_type, self.robot_type)),
        ]
        if self.payload_kg is not None:
            rows.append(("额定负载", f"{self.payload_kg:g} kg"))
        if self.reach_mm is not None:
            rows.append(("最大臂展", f"{self.reach_mm:g} mm"))
        if self.repeatability_mm is not None:
            rows.append(("重复定位精度", f"±{self.repeatability_mm:g} mm"))
        if self.weight_kg is not None:
            rows.append(("本体自重", f"{self.weight_kg:g} kg"))
        if self.ip_rating:
            rows.append(("防护等级", self.ip_rating))
        if self.controller:
            rows.append(("典型控制器", self.controller))
        return [f"{k}: {v}" for k, v in rows]


def _unknown_brand(brand_id: str) -> BrandSpec:
    """为外部/未知品牌构造一个占位 BrandSpec。"""
    return BrandSpec(
        id=brand_id,
        name=brand_id.capitalize(),
        name_en=brand_id.capitalize(),
        country="—",
        primary=(0.62, 0.64, 0.68),
        accent=(0.30, 0.32, 0.36),
        controller="",
    )


# --------------------------------------------------------------------------- #
# 外部 URDF 解析
# --------------------------------------------------------------------------- #
def _parse_urdf_meta(path: Path) -> dict:
    """从任意 URDF 中解析关节数、限位与机器人名。

    解析失败时返回空 dict，不让单个坏文件影响整个库的加载。
    """
    meta: dict = {}
    try:
        root = ET.parse(str(path)).getroot()
    except Exception:
        return meta

    meta["robot_name"] = root.get("name") or path.stem

    joints: List[tuple] = []
    for joint in root.findall("joint"):
        if joint.get("type") != "revolute":
            continue
        limit = joint.find("limit")
        try:
            lo = float(limit.get("lower", "-3.14159")) if limit is not None else -math.pi
            hi = float(limit.get("upper", "3.14159")) if limit is not None else math.pi
        except (TypeError, ValueError):
            lo, hi = -math.pi, math.pi
        joints.append((joint.get("name", f"joint{len(joints)}"), math.degrees(lo), math.degrees(hi)))

    meta["joints"] = joints
    meta["dof"] = len(joints)
    return meta


# --------------------------------------------------------------------------- #
# 目录
# --------------------------------------------------------------------------- #
class RobotCatalog:
    """机器人型号目录。

    Args:
        models_dir: 模型根目录，约定结构为 ``<brand>/<model>/<model>.urdf``
        auto_generate: 内置型号缺 URDF 时是否即时生成
    """

    def __init__(self, models_dir: Path, auto_generate: bool = True):
        self.models_dir = Path(models_dir)
        self.auto_generate = auto_generate
        self._entries: Dict[str, CatalogEntry] = {}
        self.refresh()

    # ---------------- 构造 ---------------- #
    @classmethod
    def from_repo_root(cls, repo_root: Path, auto_generate: bool = True) -> "RobotCatalog":
        """按仓库约定（根目录下的 ``models/``）构造目录。"""
        return cls(Path(repo_root) / "models", auto_generate=auto_generate)

    # ---------------- 扫描 ---------------- #
    def refresh(self) -> None:
        """重新扫描模型目录并重建索引。"""
        self._entries.clear()

        # 1) 内置型号（规格库为准）
        for spec in ROBOTS:
            urdf_path = self._locate_urdf(spec)
            if urdf_path is None and self.auto_generate:
                urdf_path = self._generate_urdf(spec)
            self._entries[spec.key] = self._from_spec(spec, urdf_path)

        # 2) 外部模型（目录里存在但不在规格库中的 URDF）
        if self.models_dir.is_dir():
            for urdf_path in sorted(self.models_dir.rglob("*.urdf")):
                key = self._path_to_key(urdf_path)
                if key in self._entries:
                    continue
                self._entries[key] = self._from_urdf(urdf_path, key)

    def _locate_urdf(self, spec: RobotSpec) -> Optional[Path]:
        """查找内置型号的 URDF（找不到返回 None）。"""
        preferred = self.models_dir / spec.brand_id / spec.model_id / f"{spec.model_id}.urdf"
        if preferred.exists():
            return preferred
        # 兼容：<brand>/<model>/ 下任意 urdf
        folder = self.models_dir / spec.brand_id / spec.model_id
        if folder.is_dir():
            for p in sorted(folder.glob("*.urdf")):
                return p
        return None

    def _generate_urdf(self, spec: RobotSpec) -> Optional[Path]:
        """按需为内置型号生成 URDF 文件。"""
        folder = self.models_dir / spec.brand_id / spec.model_id
        try:
            folder.mkdir(parents=True, exist_ok=True)
            target = folder / f"{spec.model_id}.urdf"
            target.write_text(build_urdf_string(spec), encoding="utf-8")
            return target
        except OSError:
            return None

    def _path_to_key(self, urdf_path: Path) -> str:
        """把 URDF 路径映射为 ``brand/model`` 键。"""
        try:
            rel = urdf_path.relative_to(self.models_dir)
        except ValueError:
            return f"_external/{urdf_path.stem}"
        parts = rel.parts
        if len(parts) >= 2:
            return f"{parts[0]}/{parts[1]}"
        return f"{parts[0]}/{urdf_path.stem}"

    def _from_spec(self, spec: RobotSpec, urdf_path: Optional[Path]) -> CatalogEntry:
        brand = spec.brand
        return CatalogEntry(
            key=spec.key,
            brand_id=spec.brand_id,
            brand_name=brand.name_en,
            brand_name_cn=brand.name,
            model_id=spec.model_id,
            model_name=spec.model_name,
            urdf_path=urdf_path,
            dof=spec.dof,
            source="internal",
            payload_kg=spec.payload_kg,
            reach_mm=spec.reach_mm,
            repeatability_mm=spec.repeatability_mm,
            weight_kg=spec.weight_kg,
            ip_rating=spec.ip_rating,
            robot_type=spec.robot_type,
            controller=spec.controller or brand.controller,
            verified=spec.verified,
            note=spec.note,
            joint_limits_deg=spec.joint_limits_deg,
            joint_speeds_deg_s=spec.joint_speeds_deg_s,
            color=brand.primary,
        )

    def _from_urdf(self, urdf_path: Path, key: str) -> CatalogEntry:
        brand_id, _, model_id = key.partition("/")
        brand = BRANDS.get(brand_id) or _unknown_brand(brand_id)
        meta = _parse_urdf_meta(urdf_path)
        joints = meta.get("joints", [])
        return CatalogEntry(
            key=key,
            brand_id=brand_id,
            brand_name=brand.name_en,
            brand_name_cn=brand.name,
            model_id=model_id or urdf_path.stem,
            model_name=meta.get("robot_name") or urdf_path.stem,
            urdf_path=urdf_path,
            dof=meta.get("dof", 0),
            source="external",
            robot_type="industrial",
            controller=brand.controller,
            verified=False,
            note="外部模型：规格参数未知，仅从 URDF 解析出轴数与关节限位",
            joint_limits_deg=[(lo, hi) for _, lo, hi in joints],
            color=brand.primary,
        )

    # ---------------- 查询 ---------------- #
    def all(self) -> List[CatalogEntry]:
        """全部条目（按品牌、型号排序）。"""
        return sorted(self._entries.values(), key=lambda e: (e.brand_id, e.model_name))

    def get(self, key: str) -> Optional[CatalogEntry]:
        """按键取条目。"""
        return self._entries.get(key)

    def find_by_urdf(self, path) -> Optional[CatalogEntry]:
        """按 URDF 文件路径反查条目（用于加载后回显规格）。"""
        try:
            target = Path(path).resolve()
        except OSError:
            return None
        for entry in self._entries.values():
            if entry.urdf_path and entry.urdf_path.resolve() == target:
                return entry
        return None

    def __len__(self) -> int:
        return len(self._entries)

    def __iter__(self):
        return iter(self.all())

    def brands(self) -> List[tuple]:
        """返回 ``[(brand_id, display_name, count), ...]``，按型号数量降序。"""
        counter: Dict[str, int] = {}
        display: Dict[str, str] = {}
        for entry in self._entries.values():
            counter[entry.brand_id] = counter.get(entry.brand_id, 0) + 1
            display[entry.brand_id] = entry.brand_display
        items = [(bid, display.get(bid, bid), cnt) for bid, cnt in counter.items()]
        items.sort(key=lambda x: (-x[2], x[0]))
        return items

    def search(
        self,
        keyword: str = "",
        brand_id: Optional[str] = None,
        robot_type: Optional[str] = None,
        min_payload: Optional[float] = None,
        max_payload: Optional[float] = None,
        min_reach: Optional[float] = None,
        max_reach: Optional[float] = None,
        dof: Optional[int] = None,
        sort_by: str = "brand",
    ) -> List[CatalogEntry]:
        """按条件检索型号。

        Args:
            keyword: 关键字，匹配型号名/品牌名/型号 id（忽略大小写）
            brand_id: 限定品牌
            robot_type: ``industrial`` / ``cobot``
            min_payload / max_payload: 额定负载区间 (kg)
            min_reach / max_reach: 臂展区间 (mm)
            dof: 精确轴数
            sort_by: ``brand`` | ``payload`` | ``reach`` | ``name``

        Returns:
            命中的条目列表
        """
        kw = keyword.strip().lower()
        results: List[CatalogEntry] = []

        for entry in self._entries.values():
            if brand_id and entry.brand_id != brand_id:
                continue
            if robot_type and entry.robot_type != robot_type:
                continue
            if dof is not None and entry.dof != dof:
                continue
            if min_payload is not None and (
                entry.payload_kg is None or entry.payload_kg < min_payload
            ):
                continue
            if max_payload is not None and (
                entry.payload_kg is None or entry.payload_kg > max_payload
            ):
                continue
            if min_reach is not None and (entry.reach_mm is None or entry.reach_mm < min_reach):
                continue
            if max_reach is not None and (entry.reach_mm is None or entry.reach_mm > max_reach):
                continue

            if kw:
                haystack = " ".join(
                    [entry.model_name, entry.model_id, entry.brand_name, entry.brand_name_cn]
                ).lower()
                if kw not in haystack:
                    continue

            results.append(entry)

        # 未知参数（外部模型）始终排在最后
        if sort_by == "payload":
            results.sort(key=lambda e: (e.payload_kg is None, e.payload_kg or 0.0))
        elif sort_by == "reach":
            results.sort(key=lambda e: (e.reach_mm is None, e.reach_mm or 0.0))
        elif sort_by == "name":
            results.sort(key=lambda e: e.display_name.lower())
        else:
            results.sort(key=lambda e: (e.brand_id, e.model_name))
        return results

    def suggest(self, keyword: str, limit: int = 12) -> List[CatalogEntry]:
        """轻量搜索（用于快速跳转），优先匹配前缀。"""
        kw = keyword.strip().lower()
        if not kw:
            return self.all()[:limit]

        prefix_hits, substr_hits = [], []
        for entry in self.all():
            name = entry.model_name.lower()
            if name.startswith(kw):
                prefix_hits.append(entry)
            elif kw in name or kw in entry.display_name.lower():
                substr_hits.append(entry)
        return (prefix_hits + substr_hits)[:limit]

    def payload_range(self) -> tuple:
        """库内负载的 (min, max)，单位 kg。"""
        vals = [e.payload_kg for e in self._entries.values() if e.payload_kg is not None]
        return (min(vals), max(vals)) if vals else (0.0, 0.0)

    def reach_range(self) -> tuple:
        """库内臂展的 (min, max)，单位 mm。"""
        vals = [e.reach_mm for e in self._entries.values() if e.reach_mm is not None]
        return (min(vals), max(vals)) if vals else (0.0, 0.0)
