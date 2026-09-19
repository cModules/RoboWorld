"""工程文件（``.orolp``）：把一次工作会话收进单一文件。

背景：v1.3.0–v1.3.3 把界面侧的东西一件件补齐（场景设备库 / 设备树 / 工艺
时序 / 零件库 / 预置场景库），但每一件都只带**自己的** JSON——场景是场景
文件、时序是时序文件、示教程序是路径代码文件、点表藏在设备面板里。
用户视角就是"存一个活要存四个文件，回头还得自己记住哪四个"。本模块补
的是这个缝：一个 ``.orolp`` 工程文件把六段一起收下。

段（section）清单——**本模块只做信封，不解释内容**：

===========  ================================================================
``scene``      场景（:meth:`orolp.world.scene.Scene.to_dict` 的原样输出）
``program``    示教程序：``{"code": str, "point_types": [str], "joint_names": [str]}``
               —— ``point_types`` 是**代码文本里没有的**工艺语义（抓取/放置），
               单存路径代码会把它们丢掉，工程文件把它们收进来
``sequence``   工艺时序（:meth:`orolp.sequence.ProcessSequence.to_dict`）
``cell``       设备点表快照：``{"devices": [{name, type, brand, …, io_map}], "steps": [...]}``
``parts``      零件库来源：``{"sources": [文件路径, …]}``
``calibration``标定集（:meth:`orolp.calibration.CalibrationSet.to_dict`，可选）
===========  ================================================================

三条硬规矩（都在测试里钉死）：

1. **逐段容错**：某一段格式不对，只把这**一段**记进载入报告的"跳过"里，
   其余段照常载入——绝不因为零件库格式升级就打不开整个工程
   （GUI 靠 :meth:`LoadReport.describe` 如实告诉用户少了什么）。
2. **不静默丢内容**：跳过的段、未知段名、未知顶层键都**原样保留**在
   :attr:`Project.raw_sections` / :attr:`Project.extra` 里，保存时写回。
   本库不认识的未来字段，往返一趟不会消失。
3. **不猜版本**：``schema`` 主版本不等直接抛 :class:`ProjectError`
   （工程文件是"唯一的一份工作成果"，宁可明确拒绝，也不能按错的语义
   打开后让用户以为"内容丢了"）。同一版本内的字段缺失按可选处理。

写入是**原子**的（同目录临时文件 + ``os.replace``）：写一半断电不会把
上一份好工程截断成半截 JSON。``NaN``/``Inf`` 在 ``set()`` 时就被拒
——它们序列化出来是非法 JSON，等读的时候才发现就晚了。
"""

from __future__ import annotations

import json
import os
import tempfile
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

from .i18n import tr

__all__ = [
    "PROJECT_SCHEMA",
    "PROJECT_EXTENSION",
    "PROJECT_FILTER",
    "SECTION_NAMES",
    "SECTION_TITLES",
    "ProjectError",
    "LoadReport",
    "Project",
    "cell_snapshot",
    "cell_from_snapshot",
    "describe_section",
]

#: 工程文件 schema（主版本不等即拒收）
PROJECT_SCHEMA = "orolp.project/1"

#: 工程文件扩展名
PROJECT_EXTENSION = ".orolp"

#: 文件对话框过滤器
PROJECT_FILTER = tr("RobotWorld 工程 (*.orolp);;所有文件 (*)")

#: 段标题（进 i18n，同时用于报告与 CLI 输出）
SECTION_TITLES: Dict[str, str] = {
    "scene": tr("场景"),
    "program": tr("示教程序"),
    #: 多台机器人**各自**的示教程序。``program`` 只装得下"当前那台"（路径
    #: 编辑器一次只显示一台），多机工程要靠这段才不会丢程序
    "programs": tr("各机示教程序"),
    "sequence": tr("工艺时序"),
    "cell": tr("设备点表"),
    "parts": tr("零件库来源"),
    "calibration": tr("标定集"),
}

#: 段名（顺序即报告与摘要的展示顺序）
SECTION_NAMES: Tuple[str, ...] = tuple(SECTION_TITLES)


class ProjectError(ValueError):
    """工程文件不可用（不是工程文件 / schema 不支持 / 内容无法序列化）。"""


def _now() -> str:
    return datetime.now().isoformat(timespec="seconds")


def _library_version() -> str:
    try:  # 延迟导入：project 可能被 __init__ 在版本号赋值前引入
        from . import __version__

        return str(__version__)
    except Exception:  # pragma: no cover - 正常安装下不会走到
        return "unknown"


# --------------------------------------------------------------------------- #
# 载入报告
# --------------------------------------------------------------------------- #


@dataclass
class LoadReport:
    """一次载入的结果——**如实**列出读了什么、跳过了什么、原样留了什么。"""

    loaded: List[str] = field(default_factory=list)
    #: ``(段名, 原因)``——格式不对被跳过的段（原负载保留在原样段里）
    skipped: List[Tuple[str, str]] = field(default_factory=list)
    #: 本库不认识的段名（原样保留，保存时写回）
    unknown: List[str] = field(default_factory=list)
    #: 本库不认识的顶层键（原样保留，保存时写回）
    extra_keys: List[str] = field(default_factory=list)
    #: 非致命提示（版本差异等）
    notes: List[str] = field(default_factory=list)

    def ok(self) -> bool:
        """没有任何段被跳过。"""
        return not self.skipped

    def has_content(self) -> bool:
        return bool(self.loaded)

    def describe(self) -> str:
        """人话报告（GUI 弹窗 / CLI 直接展示）。"""
        lines: List[str] = []
        if self.loaded:
            lines.append(
                tr("已载入 {n} 段：{names}").format(
                    n=len(self.loaded),
                    names="、".join(_title(n) for n in self.loaded),
                )
            )
        else:
            lines.append(tr("工程里没有任何可识别的段"))
        if self.skipped:
            lines.append(tr("跳过 {n} 段（内容原样保留，保存时写回）：").format(n=len(self.skipped)))
            for name, reason in self.skipped:
                lines.append(tr("  · {name}：{reason}").format(name=_title(name), reason=reason))
        if self.unknown:
            lines.append(
                tr("未知段 {names}——本版本不解释，原样保留").format(
                    names="、".join(self.unknown)
                )
            )
        if self.extra_keys:
            lines.append(
                tr("未知顶层键 {keys}——原样保留").format(keys="、".join(self.extra_keys))
            )
        for note in self.notes:
            lines.append(note)
        return "\n".join(lines)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "loaded": list(self.loaded),
            "skipped": [{"section": n, "reason": r} for n, r in self.skipped],
            "unknown": list(self.unknown),
            "extra_keys": list(self.extra_keys),
            "notes": list(self.notes),
        }


def _title(section: str) -> str:
    return SECTION_TITLES.get(section, section)


# --------------------------------------------------------------------------- #
# 段内容描述（摘要用，不解释语义）
# --------------------------------------------------------------------------- #


def _is_number(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def describe_section(section: str, payload: Any) -> str:
    """给出一段的**可读摘要**；格式不认识的段如实说"格式异常"。"""
    title = _title(section)
    if section == "scene":
        if not isinstance(payload, dict):
            return tr("{title}：格式异常").format(title=title)
        objects = payload.get("objects")
        n_obj = len(objects) if isinstance(objects, list) else 0
        robots = payload.get("robots")
        n_robot = len(robots) if isinstance(robots, list) else 1
        urdf = str(payload.get("robot_urdf") or "")
        name = Path(urdf).name if urdf else tr("（未指定）")
        return tr("{title}：{n} 个对象 ｜ {r} 台机器人（{name}）").format(
            title=title, n=n_obj, r=n_robot, name=name
        )
    if section == "program":
        if not isinstance(payload, dict):
            return tr("{title}：格式异常").format(title=title)
        code = payload.get("code")
        code = code if isinstance(code, str) else ""
        lines = code.count("\n") + 1 if code else 0
        types = payload.get("point_types")
        types = types if isinstance(types, list) else []
        picks = sum(1 for t in types if t == "pick")
        places = sum(1 for t in types if t == "place")
        return tr("{title}：{n} 行 ｜ {p} 个示教点（抓取 {k} / 放置 {v}）").format(
            title=title, n=lines, p=len(types), k=picks, v=places
        )
    if section == "programs":
        if not isinstance(payload, dict):
            return tr("{title}：格式异常").format(title=title)
        total = 0
        picks = places = 0
        for item in payload.values():
            if not isinstance(item, dict):
                continue
            types = item.get("point_types")
            types = types if isinstance(types, list) else []
            total += len(types)
            picks += sum(1 for t in types if t == "pick")
            places += sum(1 for t in types if t == "place")
        return tr("{title}：{r} 台机器人 ｜ 合计 {p} 个示教点（抓取 {k} / 放置 {v}）").format(
            title=title, r=len(payload), p=total, k=picks, v=places
        )
    if section == "sequence":
        if not isinstance(payload, dict):
            return tr("{title}：格式异常").format(title=title)
        steps = payload.get("steps")
        steps = steps if isinstance(steps, list) else []
        dwell = sum(
            float(s.get("seconds") or 0.0)
            for s in steps
            if isinstance(s, dict) and s.get("kind") == "dwell"
        )
        return tr("{title}：{n} 个工步 ｜ 驻留合计 {d}s").format(
            title=title, n=len(steps), d=round(dwell, 3)
        )
    if section == "cell":
        if not isinstance(payload, dict):
            return tr("{title}：格式异常").format(title=title)
        devices = payload.get("devices")
        devices = devices if isinstance(devices, list) else []
        n_sig = sum(
            len(d.get("io_map") or {}) for d in devices if isinstance(d, dict)
        )
        return tr("{title}：{n} 台设备 ｜ {s} 个已映射信号").format(
            title=title, n=len(devices), s=n_sig
        )
    if section == "parts":
        if not isinstance(payload, dict):
            return tr("{title}：格式异常").format(title=title)
        sources = payload.get("sources")
        sources = sources if isinstance(sources, list) else []
        return tr("{title}：{n} 个零件库文件").format(title=title, n=len(sources))
    if section == "calibration":
        if not isinstance(payload, dict):
            return tr("{title}：格式异常").format(title=title)
        present = [
            key
            for key in ("tool", "base", "frames", "hand_eye", "camera")
            if payload.get(key)
        ]
        detail = "、".join(present) if present else tr("（内容为空）")
        return tr("{title}：{detail}").format(title=title, detail=detail)
    return tr("{title}：未知段").format(title=title)


def _section_problem(section: str, payload: Any) -> str:
    """检查一段的**结构**；没问题返回空串，否则返回人话原因。

    只做结构检查（类型/必需键），不解释语义——语义由各段的属主类在
    "应用到界面/模型"时校验（例如 ``Scene.from_dict`` 会自己抱怨坏字段）。
    """
    if section == "scene":
        if not isinstance(payload, dict):
            return tr("场景段必须是对象")
        if payload.get("objects") is not None and not isinstance(payload["objects"], list):
            return tr("场景段的 objects 必须是数组")
        return ""
    if section == "program":
        if not isinstance(payload, dict):
            return tr("示教程序段必须是对象")
        code = payload.get("code")
        if code is None:
            return tr("示教程序段缺 code")
        if not isinstance(code, str):
            return tr("示教程序段的 code 必须是字符串")
        types = payload.get("point_types")
        if types is not None and not isinstance(types, list):
            return tr("示教程序段的 point_types 必须是数组")
        return ""
    if section == "programs":
        if not isinstance(payload, dict):
            return tr("各机示教程序段必须是对象")
        for name, item in payload.items():
            if not isinstance(item, dict):
                return tr("各机示教程序段：{name} 的内容必须是对象").format(name=name)
            positions = item.get("positions")
            if positions is not None and not isinstance(positions, list):
                return tr("各机示教程序段：{name} 的 positions 必须是数组").format(name=name)
        return ""
    if section == "sequence":
        if not isinstance(payload, dict):
            return tr("工艺时序段必须是对象")
        steps = payload.get("steps")
        if steps is not None and not isinstance(steps, list):
            return tr("工艺时序段的 steps 必须是数组")
        return ""
    if section == "cell":
        if not isinstance(payload, dict):
            return tr("设备点表段必须是对象")
        devices = payload.get("devices")
        if devices is None:
            return tr("设备点表段缺 devices")
        if not isinstance(devices, list):
            return tr("设备点表段的 devices 必须是数组")
        for d in devices:
            if not isinstance(d, dict) or not isinstance(d.get("name"), str) or not d.get("name"):
                return tr("设备点表里有一台设备缺 name")
        return ""
    if section == "parts":
        if not isinstance(payload, dict):
            return tr("零件库来源段必须是对象")
        sources = payload.get("sources")
        if sources is None:
            return tr("零件库来源段缺 sources")
        if not isinstance(sources, list) or any(not isinstance(s, str) for s in sources):
            return tr("零件库来源段的 sources 必须是字符串数组")
        return ""
    if section == "calibration":
        if not isinstance(payload, dict):
            return tr("标定集段必须是对象")
        schema = str(payload.get("schema") or "")
        if schema and schema != "orolp.calibration/1":
            return tr("标定集 schema 不支持：{schema}").format(schema=schema)
        return ""
    return tr("未知段")


# --------------------------------------------------------------------------- #
# 设备点表快照（工程段 ⇄ CellConfig）
# --------------------------------------------------------------------------- #

#: ``Device`` 里进快照的字段（顺序即 JSON 里的键序）
_DEVICE_FIELDS: Tuple[str, ...] = ("name", "type", "brand", "model", "comm", "ip")


def cell_snapshot(cell: Any) -> Dict[str, Any]:
    """把 :class:`orolp.cell.CellConfig`（或同形状对象）存成工程段。

    鸭子类型：只要 ``.devices``（每项有 ``name``/``type``/``io_map``）与
    可选 ``.steps`` 即可——面板与测试的替身对象都能用。
    """
    devices: List[Dict[str, Any]] = []
    for dev in getattr(cell, "devices", []) or []:
        row: Dict[str, Any] = {k: getattr(dev, k, "") for k in _DEVICE_FIELDS}
        row["name"] = str(row.get("name") or "")
        row["type"] = str(row.get("type") or "")
        row["io_map"] = {
            str(k): str(v) for k, v in (getattr(dev, "io_map", {}) or {}).items()
        }
        devices.append(row)
    steps = [dict(s) for s in (getattr(cell, "steps", []) or [])]
    return {"devices": devices, "steps": steps}


def cell_from_snapshot(payload: Any) -> Any:
    """把工程段恢复成 :class:`orolp.cell.CellConfig`。

    Raises:
        ValueError: 段结构不对（缺 name 等）——调用方按"这段没载入"处理
    """
    from .cell import CellConfig, Device

    reason = _section_problem("cell", payload)
    if reason:
        raise ValueError(reason)
    cell = CellConfig()
    for row in payload.get("devices") or []:
        cell.add(
            Device(
                name=str(row.get("name")),
                type=str(row.get("type") or "传感器"),
                brand=str(row.get("brand") or ""),
                model=str(row.get("model") or ""),
                comm=str(row.get("comm") or ""),
                ip=str(row.get("ip") or ""),
                io_map={str(k): str(v) for k, v in (row.get("io_map") or {}).items()},
            )
        )
    cell.steps = [dict(s) for s in (payload.get("steps") or [])]
    return cell


# --------------------------------------------------------------------------- #
# 工程
# --------------------------------------------------------------------------- #


class Project:
    """一个工程文件的内容：元信息 + 六段负载。"""

    def __init__(
        self,
        name: str = "",
        note: str = "",
        created: Optional[str] = None,
        modified: Optional[str] = None,
        orolp_version: Optional[str] = None,
    ) -> None:
        self.name = str(name or "")
        self.note = str(note or "")
        self.created = str(created or _now())
        self.modified = str(modified or self.created)
        self.orolp_version = str(orolp_version or _library_version())
        #: 解析成功的段
        self.sections: Dict[str, Any] = {}
        #: 解析失败/未知的段——**原样保留**，保存时写回（不静默丢内容）
        self.raw_sections: Dict[str, Any] = {}
        #: 未知顶层键——原样保留，保存时写回
        self.extra: Dict[str, Any] = {}
        #: 最近一次载入的报告（新建的工程为 None）
        self.load_report: Optional[LoadReport] = None
        #: 最近一次保存/载入的路径（GUI 用来做"另存为/直接保存"）
        self.path: Optional[Path] = None

    # ---------------- 容器语义 ---------------- #

    def __len__(self) -> int:
        return len(self.sections)

    def __contains__(self, section: object) -> bool:
        return section in self.sections

    @property
    def section_names(self) -> List[str]:
        """按固定顺序返回**实际存在**的段名（含原样保留的段）。"""
        return [
            n for n in SECTION_NAMES if n in self.sections
        ] + [n for n in self.raw_sections if n not in self.sections]

    def set(self, section: str, payload: Any) -> None:
        """写入一段。``payload`` 为 ``None`` 时等于删除该段。

        Raises:
            ProjectError: 段名不在 :data:`SECTION_NAMES`，或负载无法
                序列化成合法 JSON（含 ``NaN``/``Inf``——它们出来是非法 JSON，
                在这里拦住比读的时候才发现好）
        """
        if section not in SECTION_NAMES:
            raise ProjectError(
                tr("未知工程段：{name}（可用：{opts}）").format(
                    name=section, opts="/".join(SECTION_NAMES)
                )
            )
        if payload is None:
            self.sections.pop(section, None)
            self.raw_sections.pop(section, None)
            return
        try:
            json.dumps(payload, ensure_ascii=False, allow_nan=False)
        except (TypeError, ValueError) as exc:
            raise ProjectError(
                tr("工程段 {name} 无法序列化：{msg}").format(name=_title(section), msg=exc)
            ) from exc
        self.sections[section] = payload
        self.raw_sections.pop(section, None)

    def get(self, section: str, default: Any = None) -> Any:
        return self.sections.get(section, default)

    def has(self, section: str) -> bool:
        """该段**可用**（解析成功）——被跳过或未知的段返回 ``False``。"""
        return section in self.sections

    def remove(self, section: str) -> None:
        self.sections.pop(section, None)
        self.raw_sections.pop(section, None)

    def raw(self, section: str) -> Any:
        """取被跳过/未知段的**原始负载**（排查与保真用）。"""
        return self.raw_sections.get(section)

    # ---------------- 摘要 ---------------- #

    def missing(self) -> List[str]:
        """本工程**没有**的段（如实报告，不假装齐全）。"""
        return [n for n in SECTION_NAMES if n not in self.sections]

    def summary(self) -> Dict[str, str]:
        out: Dict[str, str] = {}
        for name in SECTION_NAMES:
            if name in self.sections:
                out[name] = describe_section(name, self.sections[name])
        for name in self.raw_sections:
            out[name] = tr("{name}：原样保留（本版本不解释）").format(name=_title(name))
        return out

    def describe(self) -> str:
        """多行摘要（状态栏 / CLI / 弹窗直接展示）。"""
        head = self.name or tr("（未命名工程）")
        lines = [
            tr("工程：{name}").format(name=head),
            tr("保存版本：orolp {v} ｜ 最后修改 {t}").format(
                v=self.orolp_version, t=self.modified
            ),
        ]
        if self.note:
            lines.append(tr("备注：{note}").format(note=self.note))
        summary = self.summary()
        if summary:
            for text in summary.values():
                lines.append("  " + text)
        else:
            lines.append(tr("  （空工程：没有任何段）"))
        missing = self.missing()
        if missing:
            lines.append(
                tr("工程里没有：{names}").format(names="、".join(_title(n) for n in missing))
            )
        return "\n".join(lines)

    # ---------------- 序列化 ---------------- #

    def to_dict(self) -> Dict[str, Any]:
        """序列化为可 JSON 化的字典（未知段与未知顶层键一并写回）。"""
        sections: Dict[str, Any] = dict(self.raw_sections)
        sections.update(self.sections)
        data: Dict[str, Any] = {
            "schema": PROJECT_SCHEMA,
            "meta": {
                "name": self.name,
                "created": self.created,
                "modified": self.modified,
                "orolp_version": self.orolp_version,
                "note": self.note,
            },
            "sections": sections,
        }
        for key, value in self.extra.items():
            data.setdefault(key, value)
        return data

    @classmethod
    def from_dict(cls, data: Any) -> "Project":
        """从字典重建工程，并把结果记进 :attr:`load_report`。

        Raises:
            ProjectError: 不是工程字典，或 ``schema`` 不是 :data:`PROJECT_SCHEMA`
        """
        if not isinstance(data, dict):
            raise ProjectError(tr("工程文件的内容必须是一个对象"))
        schema = str(data.get("schema") or "")
        if schema != PROJECT_SCHEMA:
            raise ProjectError(
                tr("不是 RobotWorld 工程文件（schema={got}，期望 {want}）").format(
                    got=schema or tr("（缺失）"), want=PROJECT_SCHEMA
                )
            )
        meta_raw = data.get("meta")
        meta = meta_raw if isinstance(meta_raw, dict) else {}
        project = cls(
            name=str(meta.get("name") or ""),
            note=str(meta.get("note") or ""),
            created=str(meta.get("created") or "") or None,
            modified=str(meta.get("modified") or "") or None,
            orolp_version=str(meta.get("orolp_version") or "") or None,
        )
        saved_version = str(meta.get("orolp_version") or "")

        report = LoadReport()
        sections_raw = data.get("sections")
        if sections_raw is None:
            sections_raw = {}
        if not isinstance(sections_raw, dict):
            raise ProjectError(tr("工程文件的 sections 必须是一个对象"))
        # 按固定段序处理已知段（报告的"已载入/跳过"顺序因此与文件里的键序无关，
        # 稳定可断言），未知段放在其后。
        ordered: List[str] = [n for n in SECTION_NAMES if n in sections_raw]
        ordered += [n for n in sections_raw if n not in SECTION_NAMES]
        for name in ordered:
            payload = sections_raw[name]
            if name not in SECTION_NAMES:
                project.raw_sections[name] = payload
                report.unknown.append(name)
                continue
            reason = _section_problem(name, payload)
            if reason:
                project.raw_sections[name] = payload
                report.skipped.append((name, reason))
            else:
                project.sections[name] = payload
                report.loaded.append(name)

        for key, value in data.items():
            if key in ("schema", "meta", "sections"):
                continue
            project.extra[key] = value
            report.extra_keys.append(key)

        if saved_version and saved_version != _library_version():
            report.notes.append(
                tr(
                    "工程由 orolp {saved} 保存，当前库版本 {cur}——按现行 schema 读取，"
                    "未做版本迁移"
                ).format(saved=saved_version, cur=_library_version())
            )
        project.load_report = report
        return project

    # ---------------- 磁盘往返 ---------------- #

    def save(self, path: Union[str, Path]) -> Path:
        """原子写入工程文件，返回最终路径。

        * 无扩展名时自动补 :data:`PROJECT_EXTENSION`；
        * 先写同目录临时文件再 ``os.replace``——写一半失败不会破坏上一份；
        * ``meta.modified`` 每次保存刷新（``created`` 只在首次设定）。
        """
        target = Path(path)
        if not target.name:
            raise ProjectError(tr("保存路径为空"))
        if target.suffix == "":
            target = target.with_suffix(PROJECT_EXTENSION)
        self.modified = _now()
        data = self.to_dict()
        try:
            text = json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)
        except (TypeError, ValueError) as exc:
            raise ProjectError(tr("工程无法序列化：{msg}").format(msg=exc)) from exc
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            fd, tmp_name = tempfile.mkstemp(
                dir=str(target.parent), prefix=target.name + ".", suffix=".tmp"
            )
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as fh:
                    fh.write(text)
                    fh.flush()
                    os.fsync(fh.fileno())
                os.replace(tmp_name, target)
            except BaseException:
                try:
                    os.unlink(tmp_name)
                except OSError:
                    pass
                raise
        except OSError as exc:
            raise ProjectError(tr("写入工程文件失败：{msg}").format(msg=exc)) from exc
        if not self.name:
            self.name = target.stem
        self.path = target
        return target

    @classmethod
    def load(cls, path: Union[str, Path]) -> "Project":
        """读取工程文件（逐段容错，报告见 :attr:`load_report`）。

        Raises:
            ProjectError: 文件读不了 / 不是合法 JSON / 不是本库的工程
        """
        source = Path(path)
        try:
            text = source.read_text(encoding="utf-8")
        except OSError as exc:
            raise ProjectError(tr("读取工程文件失败：{msg}").format(msg=exc)) from exc
        try:
            data = json.loads(text)
        except ValueError as exc:
            raise ProjectError(tr("工程文件不是合法 JSON：{msg}").format(msg=exc)) from exc
        project = cls.from_dict(data)
        project.path = source
        if not project.name:
            project.name = source.stem
        return project

    # ---------------- 可选段 ---------------- #

    def calibration(self) -> Any:  # pragma: no cover - 便捷读取，GUI 用
        """把标定段还原成 :class:`orolp.calibration.CalibrationSet`（无则 ``None``）。"""
        payload = self.get("calibration")
        if payload is None:
            return None
        from .calibration import CalibrationSet

        return CalibrationSet.from_dict(payload)


def _main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI：不开 GUI 看一眼工程文件里有什么。"""
    import argparse

    parser = argparse.ArgumentParser(
        prog="python -m orolp.project",
        description=tr("查看 .orolp 工程文件的内容摘要（不加载场景/机器人）"),
    )
    parser.add_argument("path", help=tr("工程文件路径"))
    parser.add_argument("--json", action="store_true", help=tr("以 JSON 输出摘要"))
    args = parser.parse_args(argv)

    try:
        project = Project.load(args.path)
    except ProjectError as exc:
        print(tr("打不开：{msg}").format(msg=exc))
        return 2

    if args.json:
        payload = {
            "name": project.name,
            "modified": project.modified,
            "orolp_version": project.orolp_version,
            "summary": project.summary(),
            "missing": project.missing(),
            "report": project.load_report.to_dict() if project.load_report else None,
        }
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    else:
        print(project.describe())
        if project.load_report is not None:
            print(project.load_report.describe())
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(_main())
