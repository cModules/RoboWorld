"""程序版本对比（v1.0.7）：让"改了哪几条、节拍变多少、TCP 偏了多少"一句话说清。

现场改动评审是离线编程最日常也最容易出事的场景：程序在云端被改过一版，谁改的、
改了什么、影响多大——靠"肉眼 diff 出码文本"既看不出节拍变化，也看不出一条
``moveL`` 目标挪了 3mm 到底是工艺修正还是手抖。这一层把对比做成**结构化**的：

* :func:`diff_programs` / :class:`ProgramDiff` —— 两份程序的逐指令对齐与差异：
  用序列对齐（``difflib``）把"插入/删除一条指令"与其余指令**对齐**，而不是
  从插入点开始全部错位地报"变了"；对齐后的每对指令做**字段级**差异（关节角
  多少度、TCP 多少毫米、速度/混合半径/IO 电平……）；
* :func:`serialize_program` / :func:`deserialize_program` —— 程序 ↔ JSON 往返，
  让差异可以走 v1.0.3/v1.0.6 的云通道被存储、传输、评审；
* :class:`ProgramVersionStore` —— 版本库：挂在任意
  :class:`~orolp.twinsync.CloudTransport` 上（本机目录 / 内存 / HTTP 云端都行），
  按名自动递增 ``v1/v2/...``，任何一版都能取回、能指纹比对、能直接 diff 任意
  两个版本（可选连规划器一起比：**节拍变化 + TCP 路径偏移**）。

诚实说明
--------

* 结构对比是精确的（字段级、无启发式）；但 **plan=True 的节拍与 TCP 路径偏移**
  是规划器口径——它回答"仿真里会多花多久、路径会偏多少"，不回答"真机上会怎样"。
* 节拍变化 = 两条轨迹时长之差；TCP 路径偏移 = 新版轨迹逐点到旧版轨迹折线的
  最近距离（复用 v0.3.2 的 :func:`orolp.cam.scan_deviation`）。起点位姿不同的
  两条程序会比出"全程大偏移"——那是**起点差异**不是路径差异，读报告时要注意。
"""

from __future__ import annotations

import dataclasses
import difflib
import hashlib
import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from .pose import Pose
from .program import Motion, MotionType, Program
from .twinsync import CloudTransport, TwinError, normalize_key

__all__ = [
    "ProgramDiffError",
    "ItemChange",
    "ProgramDiff",
    "diff_programs",
    "ProgramVersionStore",
    "serialize_program",
    "deserialize_program",
    "program_fingerprint",
    "instruction_signature",
]

#: 版本库路径段（<前缀>/versions/<名>/<v>.json + index.json）
VERSIONS_DIR = "versions"

_MOTION_NAMES = {
    MotionType.JOINTS: "moveJ",
    MotionType.LINEAR: "moveL",
    MotionType.CIRCULAR: "moveC",
    MotionType.SPLINE: "moveS",
}


class ProgramDiffError(TwinError):
    """程序对比的输入错误（如版本不存在、无法识别的指令类型）。"""


# --------------------------------------------------------------------------- #
# 序列化：程序 ↔ JSON（差异比对在规范化数据上进行，任何指令类型都覆盖）
# --------------------------------------------------------------------------- #


def _encode_value(value: Any) -> Any:
    if isinstance(value, bool) or value is None or isinstance(value, (int, float, str)):
        return value
    if isinstance(value, Pose):
        return {"__pose__": [float(v) for v in value.to_xyz_rpy()]}
    if isinstance(value, MotionType):
        return value.value
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return {"__class__": type(value).__name__,
                **{f: _encode_value(getattr(value, f))
                   for f in value.__dataclass_fields__}}
    if isinstance(value, (list, tuple)):
        return [_encode_value(v) for v in value]
    if isinstance(value, dict):
        return {str(k): _encode_value(v) for k, v in value.items()}
    return str(value)  # 最后兜底：可复现的字符串，保证指纹稳定


def _encode_item(item: Any) -> Dict[str, Any]:
    if isinstance(item, Motion):
        return {
            "op": "motion",
            "type": item.type.value,
            "target": _encode_value(item.target),
            "via": _encode_value(item.via) if item.via is not None else None,
            "speed": item.speed,
            "blend": item.blend,
            "velocity": item.velocity,
            "acceleration": item.acceleration,
        }
    if dataclasses.is_dataclass(item) and not isinstance(item, type):
        return {
            "op": type(item).__name__,
            **{f: _encode_value(getattr(item, f)) for f in item.__dataclass_fields__},
        }
    raise ProgramDiffError(f"不认识的指令类型：{type(item).__name__}")


def _decode_class(name: str):
    """按类名找指令/参数类（新指令类型在这里登记即可参与往返）。"""
    import orolp.program as P
    import orolp.process as PR

    for mod in (P, PR):
        cls = getattr(mod, name, None)
        if cls is not None and dataclasses.is_dataclass(cls):
            return cls
    raise ProgramDiffError(f"无法反序列化指令类型 {name!r}（请登记到 progdiff）")


def _decode_value(value: Any) -> Any:
    if isinstance(value, dict):
        if "__pose__" in value:
            x, y, z, rx, ry, rz = value["__pose__"]
            return Pose.from_xyz_rpy(x, y, z, rx, ry, rz)
        if "__class__" in value:
            cls = _decode_class(value["__class__"])
            fields = {k: _decode_value(v) for k, v in value.items()
                      if k not in ("__class__",)}
            sig = {f: fields[f] for f in cls.__dataclass_fields__ if f in fields}
            return cls(**sig)
        return {k: _decode_value(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_decode_value(v) for v in value]
    return value


def _decode_item(data: Dict[str, Any]) -> Any:
    if data.get("op") == "motion":
        return Motion(
            MotionType(data["type"]),
            _decode_value(data["target"]),
            via=_decode_value(data["via"]) if data.get("via") is not None else None,
            speed=data.get("speed", 100.0),
            blend=data.get("blend", 0.0),
            velocity=data.get("velocity"),
            acceleration=data.get("acceleration"),
        )
    cls = _decode_class(data["op"])
    fields = {k: _decode_value(v) for k, v in data.items() if k != "op"}
    sig = {f: fields[f] for f in cls.__dataclass_fields__ if f in fields}
    return cls(**sig)


def _program_items(program: Any) -> List[Any]:
    """拿指令序列——Program 用 items、ProgramBuilder 用 motions（历史命名差异）。"""
    if isinstance(getattr(program, "items", None), list):
        return list(program.items)
    if isinstance(getattr(program, "motions", None), list):
        return list(program.motions)
    return list(program)


def _as_program(program: Any) -> Program:
    if isinstance(program, Program):
        return program
    if hasattr(program, "build"):
        return program.build()
    robot = getattr(program, "robot", None)
    return Program(robot, _program_items(program))


def serialize_program(program: Any) -> Dict[str, Any]:
    """把程序序列化成 JSON 安全的 dict（规范化指令序列 + 运动数）。

    ``Program`` / ``ProgramBuilder`` / 指令列表都接受。
    """
    items = _program_items(program)
    return {
        "items": [_encode_item(i) for i in items],
        "n_motions": sum(1 for i in items if isinstance(i, Motion)),
    }


def deserialize_program(data: Dict[str, Any], robot: Any = None) -> Program:
    """从 :func:`serialize_program` 的产物重建 :class:`~orolp.program.Program`。"""
    items = [_decode_item(d) for d in data.get("items", [])]
    return Program(robot, items)


def program_fingerprint(program: Any) -> str:
    """程序指纹：规范化序列的 SHA256。指纹相同 ⇔ 逐字段相同（改一个关节角都不同）。"""
    text = json.dumps(serialize_program(program), ensure_ascii=False,
                      sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def instruction_signature(item: Any) -> str:
    """单条指令的内容签名（规范化 JSON 文本）。

    与 :func:`program_fingerprint` 用的是同一套编码（:func:`_encode_item`），
    区别只是**只编码一条指令**——增量重算（:mod:`orolp.incremental`）拿它当分段
    缓存的键：一条指令的内容没变，它的插补展开就不该重算。

    返回规范化 JSON 串而不是哈希：逐条签名要按顺序拼成整程序摘要，保留原文
    既省一次哈希，也让调试时能直接看出"是哪条变了"。
    """
    return json.dumps(_encode_item(item), ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"))


# --------------------------------------------------------------------------- #
# 差异比对
# --------------------------------------------------------------------------- #


@dataclass
class ItemChange:
    """一条指令的变化。

    Attributes:
        kind: ``equal`` / ``changed`` / ``added`` / ``removed``
        op: 指令简述（``motion`` / ``SetDO`` …）
        index_a / index_b: 在 A/B 两个程序里的下标；单侧不存在的为 ``None``
        detail: 人读的变化描述
        fields: 变化（或新增/删除）的字段路径列表
    """

    kind: str
    op: str
    index_a: Optional[int] = None
    index_b: Optional[int] = None
    detail: str = ""
    fields: List[str] = field(default_factory=list)


def _op_key(data: Dict[str, Any]) -> str:
    op = data.get("op", "?")
    if op == "SetDO":
        return f"SetDO:{data.get('signal', '?')}"
    if op == "WaitDI":
        return f"WaitDI:{data.get('signal', '?')}"
    return op


def _motion_label(data: Dict[str, Any]) -> str:
    return _MOTION_NAMES.get(MotionType(data["type"]), "motion")


def _fmt(v: Any) -> str:
    if isinstance(v, float):
        return f"{v:g}"
    return str(v)


def _scalar_eq(x: Any, y: Any) -> bool:
    if (isinstance(x, (int, float)) and isinstance(y, (int, float))
            and not isinstance(x, bool) and not isinstance(y, bool)):
        return abs(float(x) - float(y)) <= 1e-9
    return x == y


def _flat_leaves(value: Any, prefix: str = "") -> List[Tuple[str, Any]]:
    """把嵌套结构摊平成 (字段路径, 叶子) 列表，供字段级 diff。"""
    out: List[Tuple[str, Any]] = []
    if isinstance(value, dict):
        for k, v in value.items():
            out.extend(_flat_leaves(v, f"{prefix}.{k}" if prefix else str(k)))
    elif isinstance(value, list):
        for i, v in enumerate(value):
            out.extend(_flat_leaves(v, f"{prefix}[{i}]"))
    else:
        out.append((prefix, value))
    return out


def _dict_diff(a: Dict[str, Any], b: Dict[str, Any]) -> List[str]:
    """同 op 指令的字段级差异（返回变化的字段路径）。"""
    da = dict(_flat_leaves(a))
    db = dict(_flat_leaves(b))
    keys_a, keys_b = set(da), set(db)
    out: List[str] = []
    for k in sorted(keys_a & keys_b):
        if not _scalar_eq(da[k], db[k]):
            out.append(k)
    for k in sorted(keys_b - keys_a):
        out.append(k)
    for k in sorted(keys_a - keys_b):
        out.append(k)
    return out


def _item_detail(a: Dict[str, Any], b: Dict[str, Any], fields: List[str]) -> str:
    """给一对"同 op"的指令生成人读差异描述。"""
    if a.get("op") == "motion" and b.get("op") == "motion":
        name_a, name_b = _motion_label(a), _motion_label(b)
        if name_a != name_b:
            return f"运动类型 {name_a} → {name_b}"
        bits = []
        for f in ("speed", "blend", "velocity", "acceleration"):
            if f in fields and not _scalar_eq(a.get(f), b.get(f)):
                bits.append(f"{f} {_fmt(a.get(f))}→{_fmt(b.get(f))}")
        ta, tb = a.get("target"), b.get("target")
        if isinstance(ta, dict) and isinstance(tb, dict):  # 位姿目标
            pa, pb = _decode_value(ta), _decode_value(tb)
            from .twinsync import pose_deviation
            pos, rot = pose_deviation(pa, pb)
            bits.append(f"TCP {pos:.2f}mm / {rot:.2f}°")
        elif isinstance(ta, list) and isinstance(tb, list) and len(ta) == len(tb):
            arr_a = np.asarray([_decode_value(v) for v in ta], dtype=np.float64)
            arr_b = np.asarray([_decode_value(v) for v in tb], dtype=np.float64)
            d = np.abs(arr_b - arr_a)
            j = int(np.argmax(d))
            if d[j] > 1e-9:
                bits.append(f"关节 {j + 1} {arr_a[j]:.3f}°→{arr_b[j]:.3f}°（Δ{d[j]:.3f}°）")
        elif isinstance(ta, dict) or isinstance(tb, dict):
            bits.append("目标在关节/笛卡尔间换了表示")
        return "；".join(bits) if bits else "字段变化：" + "、".join(fields)

    # 非运动指令：报头几个变化字段并带上旧→新值（标量才好读）
    shown = []
    for f in fields[:3]:
        old = a.get(f) if f in a else None
        new = b.get(f) if f in b else None
        if _scalar_eq(old, new):
            shown.append(f)
        else:
            shown.append(f"{f} {_fmt(old)}→{_fmt(new)}")
    more = f" 等 {len(fields)} 处" if len(fields) > 3 else ""
    return "、".join(shown) + more


@dataclass
class ProgramDiff:
    """两份程序的结构化对比结果。

    Attributes:
        changes: 逐指令差异（含 equal 项，保持执行顺序）
        cycle_a / cycle_b: ``plan=True`` 且规划成功时为两版节拍（秒），否则 ``None``
        path_max_mm / path_mean_mm: 新版 TCP 路径逐点对旧版折线的偏移（mm），未规划为 None
        errors: 规划失败等非致命问题（结构对比不受影响）
    """

    changes: List[ItemChange] = field(default_factory=list)
    cycle_a: Optional[float] = None
    cycle_b: Optional[float] = None
    path_max_mm: Optional[float] = None
    path_mean_mm: Optional[float] = None
    errors: List[str] = field(default_factory=list)

    # ------------------------------ 统计 ------------------------------ #

    @property
    def n_equal(self) -> int:
        return sum(1 for c in self.changes if c.kind == "equal")

    @property
    def n_changed(self) -> int:
        return sum(1 for c in self.changes if c.kind == "changed")

    @property
    def n_added(self) -> int:
        return sum(1 for c in self.changes if c.kind == "added")

    @property
    def n_removed(self) -> int:
        return sum(1 for c in self.changes if c.kind == "removed")

    @property
    def has_changes(self) -> bool:
        return bool(self.n_changed or self.n_added or self.n_removed)

    @property
    def cycle_delta(self) -> Optional[float]:
        if self.cycle_a is None or self.cycle_b is None:
            return None
        return self.cycle_b - self.cycle_a

    def summary(self) -> str:
        if not self.has_changes:
            base = "两份程序逐字段一致"
        else:
            base = (f"相同 {self.n_equal} · 修改 {self.n_changed} · "
                    f"新增 {self.n_added} · 删除 {self.n_removed}")
        if self.cycle_delta is not None:
            base += f" · 节拍 {self.cycle_a:.2f}s→{self.cycle_b:.2f}s（Δ{self.cycle_delta:+.2f}s）"
        if self.path_max_mm is not None:
            base += f" · TCP 路径偏移 max {self.path_max_mm:.2f}mm"
        for e in self.errors:
            base += f"（{e}）"
        return base

    def format(self, max_rows: int = 40) -> str:
        lines = [f"程序对比：{self.summary()}"]
        shown = 0
        for c in self.changes:
            if c.kind == "equal":
                continue
            if shown >= max_rows:
                lines.append("  … 其余差异从略")
                break
            at = f"A[{c.index_a if c.index_a is not None else '—'}]/B[{c.index_b if c.index_b is not None else '—'}]"
            mark = {"changed": "~", "added": "+", "removed": "-"}[c.kind]
            lines.append(f"  {mark} {at} {c.op}：{c.detail}")
            shown += 1
        if not shown:
            lines.append("  （无差异）")
        return "\n".join(lines)

    def to_dict(self) -> Dict[str, Any]:
        """JSON 安全的 dict（进版本库 / 云端 / 报表）。"""
        return {
            "summary": self.summary(),
            "n_equal": self.n_equal,
            "n_changed": self.n_changed,
            "n_added": self.n_added,
            "n_removed": self.n_removed,
            "cycle_a": self.cycle_a,
            "cycle_b": self.cycle_b,
            "cycle_delta": self.cycle_delta,
            "path_max_mm": self.path_max_mm,
            "path_mean_mm": self.path_mean_mm,
            "errors": list(self.errors),
            "changes": [
                {
                    "kind": c.kind, "op": c.op, "index_a": c.index_a,
                    "index_b": c.index_b, "detail": c.detail, "fields": c.fields,
                }
                for c in self.changes
                if c.kind != "equal"
            ],
        }


def _trajectory_tcp_points(traj: Any) -> np.ndarray:
    return np.array([[*wp.pose.to_xyz_rpy()[:3]] for wp in traj.waypoints],
                    dtype=np.float64)


def diff_programs(
    program_a: Any,
    program_b: Any,
    *,
    robot: Any = None,
    plan: bool = False,
) -> ProgramDiff:
    """对比两份程序，返回 :class:`ProgramDiff`。

    Args:
        program_a / program_b: ``Program`` / ``ProgramBuilder`` / 指令列表
        robot: 机器人实例；``plan=True`` 时必需（用它的规划器估节拍与 TCP 路径）
        plan: 是否连规划器一起比（节拍变化 + TCP 路径偏移）

    Note:
        对齐用指令**种类序列**（``difflib.SequenceMatcher``）：插入/删除一条
        指令不会让后面的指令整体错位报"全变了"。同一指令对齐后做字段级
        差异——关节角报"第几轴变了多少度"，位姿报"TCP 变了多少 mm/°"。
        ``SetDO``/``WaitDI`` 按信号名对齐（同名信号跨版本对应同一条指令）。
    """
    prog_a, prog_b = _as_program(program_a), _as_program(program_b)
    sa, sb = serialize_program(prog_a), serialize_program(prog_b)
    items_a, items_b = sa["items"], sb["items"]

    diff = ProgramDiff()
    keys_a = [_op_key(d) for d in items_a]
    keys_b = [_op_key(d) for d in items_b]

    matcher = difflib.SequenceMatcher(None, keys_a, keys_b, autojunk=False)
    for tag, i1, i2, j1, j2 in matcher.get_opcodes():
        if tag == "equal":
            for k in range(i2 - i1):
                a, b = items_a[i1 + k], items_b[j1 + k]
                fields = _dict_diff(a, b)
                if not fields:
                    diff.changes.append(
                        ItemChange("equal", _op_key(a), i1 + k, j1 + k))
                else:
                    diff.changes.append(
                        ItemChange("changed", _op_key(a), i1 + k, j1 + k,
                                   _item_detail(a, b, fields), fields))
        elif tag == "replace":
            span_a, span_b = items_a[i1:i2], items_b[j1:j2]
            for k in range(max(len(span_a), len(span_b))):
                a = span_a[k] if k < len(span_a) else None
                b = span_b[k] if k < len(span_b) else None
                if a is not None and b is not None:
                    fields = _dict_diff(a, b)
                    if fields:
                        diff.changes.append(
                            ItemChange("changed", _op_key(a), i1 + k, j1 + k,
                                       _item_detail(a, b, fields), fields))
                    else:
                        diff.changes.append(
                            ItemChange("equal", _op_key(a), i1 + k, j1 + k))
                elif b is not None:
                    diff.changes.append(
                        ItemChange("added", _op_key(b), None, j1 + k, "新增指令"))
                else:
                    diff.changes.append(
                        ItemChange("removed", _op_key(a), i1 + k, None, "删除指令"))
        elif tag == "delete":
            for k in range(i1, i2):
                diff.changes.append(
                    ItemChange("removed", keys_a[k], k, None, "删除指令"))
        elif tag == "insert":
            for k in range(j1, j2):
                diff.changes.append(
                    ItemChange("added", keys_b[k], None, k, "新增指令"))

    if plan:
        if robot is None:
            diff.errors.append("plan=True 需要给 robot 实例（规划节拍与 TCP 路径用）")
            return diff
        try:
            from .planner import TrajectoryPlanner
            planner = TrajectoryPlanner(robot)
            traj_a = planner.plan(prog_a)
            traj_b = planner.plan(prog_b)
            diff.cycle_a = traj_a.cycle_time()
            diff.cycle_b = traj_b.cycle_time()
            pts_a = _trajectory_tcp_points(traj_a)
            pts_b = _trajectory_tcp_points(traj_b)
            if len(pts_a) >= 2 and len(pts_b) >= 1:
                from .cam import scan_deviation
                rep = scan_deviation(pts_b, pts_a)
                diff.path_max_mm = rep.max_dev
                diff.path_mean_mm = rep.mean_dev
        except Exception as exc:  # 规划失败不影响结构对比
            diff.errors.append(f"规划对比未完成：{type(exc).__name__}: {exc}")
    return diff


# --------------------------------------------------------------------------- #
# 版本库：挂在任意 CloudTransport 上（本机目录 / 内存 / HTTP 云端）
# --------------------------------------------------------------------------- #


def _safe_name(name: str) -> str:
    safe = "".join(c if (c.isalnum() or c in "-_.") else "_" for c in str(name))
    while ".." in safe:
        safe = safe.replace("..", "_")
    safe = safe.lstrip(".")
    return safe or "unnamed"


class ProgramVersionStore:
    """程序版本库：任意 :class:`~orolp.twinsync.CloudTransport` 上的版本管理。

    目录布局（键走 :func:`~orolp.twinsync.normalize_key`，天然防路径穿越）::

        <prefix>/versions/<名>/index.json   {"latest": "v3", "count": 3, "history": [...]}
        <prefix>/versions/<名>/v1.json      {"name", "version", "created_at",
                                             "fingerprint", "meta", "items": [...]}
        <prefix>/versions/<名>/v1.code.txt  可选：存版本时一并留档的出码原文

    与 v1.0.3/v1.0.6 的关系：``Program.deploy`` 把代码送到控制器/云端；这个库存的
    是**可 diff 的结构化版本**——两者共用同一抽象（``CloudTransport``），所以
    本机目录、内存和 v1.0.6 的 HTTP 云端都能当版本库后端。
    """

    def __init__(self, transport: CloudTransport, prefix: str = "progdiff"):
        self.transport = transport
        self.prefix = normalize_key(prefix)

    def _dir(self, name: str) -> str:
        return f"{self.prefix}/{VERSIONS_DIR}/{_safe_name(name)}"

    def _index(self, name: str) -> Dict[str, Any]:
        raw = self.transport.get(f"{self._dir(name)}/index.json")
        if raw is None:
            return {"latest": None, "count": 0, "history": []}
        return json.loads(raw.decode("utf-8"))

    def save(
        self,
        name: str,
        program: Any,
        *,
        code: str = "",
        meta: Optional[Dict[str, Any]] = None,
        created_at: Optional[float] = None,
    ) -> str:
        """存一版，自动递增版本号，返回版本名（``"v1"`` / ``"v2"`` …）。

        指纹与上一版相同**仍然照存**——现场"重新上传同一份"本身就是要留痕的
        事件；版本号照常递增，diff 会如实告诉你"这两版逐字段一致"。
        """
        items = serialize_program(program)
        index = self._index(name)
        version = f"v{index['count'] + 1}"
        record = {
            "name": name,
            "version": version,
            "created_at": float(created_at if created_at is not None else time.time()),
            "fingerprint": program_fingerprint(program),
            "n_motions": items["n_motions"],
            "meta": dict(meta or {}),
            "items": items["items"],
        }
        base = self._dir(name)
        self.transport.put(f"{base}/{version}.json",
                           json.dumps(record, ensure_ascii=False).encode("utf-8"))
        if code:
            self.transport.put(f"{base}/{version}.code.txt", code.encode("utf-8"))
        index["latest"] = version
        index["count"] += 1
        index.setdefault("history", []).append(version)
        self.transport.put(f"{base}/index.json",
                           json.dumps(index, ensure_ascii=False).encode("utf-8"))
        return version

    def versions(self, name: str) -> List[str]:
        """列出某程序的全部版本号（按创建顺序）。"""
        return list(self._index(name).get("history", []))

    def latest(self, name: str) -> Optional[str]:
        """最新版本号；从未存过返回 ``None``。"""
        return self._index(name).get("latest")

    def load(self, name: str, version: str) -> Dict[str, Any]:
        """取回一个版本的完整记录（含结构化指令）。"""
        raw = self.transport.get(f"{self._dir(name)}/{_safe_name(version)}.json")
        if raw is None:
            raise ProgramDiffError(
                f"版本库中没有 {name!r} 的 {version!r}"
                f"（可用 versions({name!r}) 查看已有版本）"
            )
        return json.loads(raw.decode("utf-8"))

    def load_code(self, name: str, version: str) -> Optional[str]:
        """取回存版本时一并留档的出码原文；没存过返回 ``None``。"""
        raw = self.transport.get(f"{self._dir(name)}/{_safe_name(version)}.code.txt")
        return None if raw is None else raw.decode("utf-8")

    def diff(
        self,
        name: str,
        version_a: str,
        version_b: str,
        *,
        robot: Any = None,
        plan: bool = False,
    ) -> ProgramDiff:
        """对比同一程序的任意两个版本（可直接带上规划器比节拍与 TCP 路径）。"""
        rec_a = self.load(name, version_a)
        rec_b = self.load(name, version_b)
        prog_a = deserialize_program(rec_a, robot)
        prog_b = deserialize_program(rec_b, robot)
        return diff_programs(prog_a, prog_b, robot=robot, plan=plan)
