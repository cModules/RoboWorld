"""工艺时序（动态时序）：设备树侧的工序编排模型。

PRD（``docs/prd/ui-architecture.md`` §4.3 P1）：设备树 = 静态绑定（拖拽挂载，
v1.3.0 已落地）+ **动态时序**（配置工序触发、信号等待、子程序调用）。本模块
是动态时序的数据模型与校验——它**不新增任何程序指令**，只把"谁触发什么
信号 / 等什么信号 / 调哪个子程序 / 驻留多久"编成一条有序序列，再翻译到两个
既有出口：

* 机器人侧 → :meth:`ProcessSequence.to_program`：翻译成
  :class:`~orolp.program.SetDO` / :class:`~orolp.program.WaitDI` /
  :class:`~orolp.program.WaitTime` 追加进现有 builder；**子程序调用不是
  程序指令**（``Program`` 没有这棵语法树，各品牌主程序/子程序的组织方式
  也不同），它保留在序列里、由 :func:`~orolp.progdiff` 之类工具在
  程序文本层面处理——本模块绝不臆造调用语法，只把它如实记进 notes。
* PLC 侧 → :meth:`ProcessSequence.to_plc_steps`：翻译成
  :class:`~orolp.plc_scl.Step` 工步（语义与 :func:`orolp.plc_scl.steps_from_program`
  完全一致：输出累积、等待收束、驻留转延时），可直接喂给
  :func:`orolp.plc_scl.generate_scl` 出 SCL。子程序调用在 PLC 侧跳过并记
  note——PLC 触发机器人程序走的是既有握手信号（``HANDSHAKE_SIGNALS``），
  不是"调用"。

诚实性口径：序列只有**顺序**，没有实测节拍。驻留（秒）是唯一可计算的时间；
信号触发与等待的真实耗时取决于现场机构——:meth:`ProcessSequence.summary`
只报驻留合计与各类型步数，绝不编一个"预计节拍"。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Sequence, Union

from .i18n import tr
from .planner import SEVERITY_ERROR, SEVERITY_WARNING

__all__ = [
    "CallStep",
    "SignalStep",
    "WaitStep",
    "DwellStep",
    "SeqStep",
    "SeqIssue",
    "ProcessSequence",
]

#: 有效的等待触发方式（与 :class:`orolp.program.WaitDI` 同一词汇表）
TRIGGERS = ("level", "rising", "falling")


@dataclass
class CallStep:
    """子程序调用（机器人侧动作）。

    ``subprogram`` 是子程序**名**——调用语法（``CALL``/``Job``/``Run``…）
    属品牌主程序层，本模块不生成它，只在序列与校验里管理"调谁"。
    """

    name: str
    device: str
    subprogram: str


@dataclass
class SignalStep:
    """触发一个信号（置位/复位数字输出）——工序触发。"""

    name: str
    device: str
    signal: str
    value: bool = True


@dataclass
class WaitStep:
    """等待一个信号达到期望电平/边沿——信号等待。"""

    name: str
    device: str
    signal: str
    trigger: str = "level"
    value: bool = True
    timeout: float = 0.0

    def __post_init__(self) -> None:
        if self.trigger not in TRIGGERS:
            raise ValueError(
                tr("等待触发方式必须是 {opts} 之一，收到 {got}").format(
                    opts="/".join(TRIGGERS), got=self.trigger
                )
            )


@dataclass
class DwellStep:
    """驻留（秒）——夹具保压、机构响应等待。"""

    name: str
    device: str
    seconds: float

    def __post_init__(self) -> None:
        if not float(self.seconds) >= 0.0:
            raise ValueError(tr("驻留时长不能为负，收到 {s}s").format(s=self.seconds))


SeqStep = Union[CallStep, SignalStep, WaitStep, DwellStep]


@dataclass
class SeqIssue:
    """一条时序校验问题。

    ``index`` 指向出问题的步骤下标；结构级问题（如空序列）为 ``None``。
    """

    severity: str  # "error" / "warning"（planner 同款小写口径）
    code: str
    message: str
    index: Optional[int] = None


def _step_kind(step: SeqStep) -> str:
    if isinstance(step, CallStep):
        return "call"
    if isinstance(step, SignalStep):
        return "signal"
    if isinstance(step, WaitStep):
        return "wait"
    return "dwell"


class ProcessSequence:
    """有序工步序列：添加/删除/移动、校验、双出口翻译、JSON 往返。"""

    def __init__(self, steps: Optional[Sequence[SeqStep]] = None) -> None:
        self.steps: List[SeqStep] = list(steps) if steps else []

    # ---------------- 容器语义 ---------------- #

    def __len__(self) -> int:
        return len(self.steps)

    def __iter__(self) -> Iterator[SeqStep]:
        return iter(self.steps)

    def add(self, step: SeqStep) -> None:
        self.steps.append(step)

    def insert(self, index: int, step: SeqStep) -> None:
        self.steps.insert(max(0, min(int(index), len(self.steps))), step)

    def remove(self, index: int) -> SeqStep:
        return self.steps.pop(int(index))

    def move(self, index: int, delta: int) -> bool:
        """把 ``index`` 处的步骤移动 ``delta``（±1）；越界返回 ``False``。"""
        i, j = int(index), int(index) + int(delta)
        if not 0 <= i < len(self.steps) or not 0 <= j < len(self.steps):
            return False
        self.steps[i], self.steps[j] = self.steps[j], self.steps[i]
        return True

    def lanes(self) -> List[str]:
        """按首次出现顺序返回泳道（设备）名。"""
        order: List[str] = []
        for st in self.steps:
            if st.device not in order:
                order.append(st.device)
        return order

    # ---------------- 校验 ---------------- #

    def validate(
        self,
        signal_names: Optional[Sequence[str]] = None,
        subprograms: Optional[Sequence[str]] = None,
        devices: Optional[Sequence[str]] = None,
    ) -> List[SeqIssue]:
        """校验序列，返回问题列表（无问题为空列表，**不是**"通过"布尔值）。

        Args:
            signal_names: 场景里**真实存在**的信号名集合（点表 ∪ 握手信号）；
                给出后"序列用了但点表里没有"报 error——下装后发不出/收不到
            subprograms: 已知子程序名；给出后调用不存在的子程序报 error
            devices: 场景设备名；给出后泳道不在场景里报 error
        """
        issues: List[SeqIssue] = []
        if not self.steps:
            issues.append(SeqIssue(
                SEVERITY_WARNING, "empty",
                tr("时序为空——设备树只编排了静态挂载，还没有任何工序"),
            ))
            return issues

        sig_set = set(signal_names) if signal_names is not None else None
        sub_set = set(subprograms) if subprograms is not None else None
        dev_set = set(devices) if devices is not None else None
        seen_names: Dict[str, int] = {}

        for i, st in enumerate(self.steps):
            where = tr("工步 {i}（{name}）").format(i=i + 1, name=st.name)
            if st.name in seen_names:
                issues.append(SeqIssue(
                    SEVERITY_WARNING, "duplicate_name",
                    tr("{where} 与工步 {j} 重名").format(where=where, j=seen_names[st.name] + 1),
                    index=i,
                ))
            else:
                seen_names[st.name] = i

            if dev_set is not None and st.device not in dev_set:
                issues.append(SeqIssue(
                    SEVERITY_ERROR, "unknown_device",
                    tr("{where} 的泳道 {device} 不在场景设备里").format(where=where, device=st.device),
                    index=i,
                ))

            if isinstance(st, SignalStep):
                if sig_set is not None and st.signal not in sig_set:
                    issues.append(SeqIssue(
                        SEVERITY_ERROR, "unknown_signal",
                        tr("{where} 触发的信号 {signal} 不在点表里——下装前必须补上映射").format(
                            where=where, signal=st.signal
                        ),
                        index=i,
                    ))
            elif isinstance(st, WaitStep):
                if sig_set is not None and st.signal not in sig_set:
                    issues.append(SeqIssue(
                        SEVERITY_ERROR, "unknown_signal",
                        tr("{where} 等待的信号 {signal} 不在点表里——下装前必须补上映射").format(
                            where=where, signal=st.signal
                        ),
                        index=i,
                    ))
                if st.timeout <= 0.0:
                    issues.append(SeqIssue(
                        SEVERITY_WARNING, "no_timeout",
                        tr("{where} 无限等待 {signal}——现场必须配看门狗，否则一步卡死整条线").format(
                            where=where, signal=st.signal
                        ),
                        index=i,
                    ))
                if st.trigger != "level":
                    issues.append(SeqIssue(
                        SEVERITY_WARNING, "edge_ignores_value",
                        tr("{where} 是{trg}触发，期望电平不参与判定").format(
                            where=where,
                            trg={"rising": tr("上升沿"), "falling": tr("下降沿")}.get(st.trigger, st.trigger),
                        ),
                        index=i,
                    ))
            elif isinstance(st, CallStep):
                if sub_set is not None and st.subprogram not in sub_set:
                    issues.append(SeqIssue(
                        SEVERITY_ERROR, "unknown_subprogram",
                        tr("{where} 调用的子程序 {sub} 不存在").format(where=where, sub=st.subprogram),
                        index=i,
                    ))

        return issues

    # ---------------- 出口 ---------------- #

    def to_program(self, builder) -> List[str]:
        """把信号/等待/驻留追加进 :class:`orolp.program.ProgramBuilder`。

        Returns:
            notes：被跳过（子程序调用）与被翻译的说明——调用方应把 notes
            展示出来，而不是静默丢弃
        """
        notes: List[str] = []
        for st in self.steps:
            if isinstance(st, SignalStep):
                builder.setDO(st.signal, st.value)
            elif isinstance(st, WaitStep):
                builder.waitDI(st.signal, st.value, timeout=st.timeout, trigger=st.trigger)
            elif isinstance(st, DwellStep):
                builder.waitTime(float(st.seconds))
            else:
                notes.append(tr("工步 {name}（调用 {sub}）不是程序指令，未写入——调用语法由品牌主程序层生成").format(
                    name=st.name, sub=st.subprogram
                ))
        return notes

    def to_plc_steps(self) -> "List[Any]":
        """翻译成 :class:`orolp.plc_scl.Step` 工步（语义同 ``steps_from_program``）。

        子程序调用在 PLC 侧跳过（见模块 docstring），跳过前先把已累积的
        输出收束成步，保证两侧步序不串味。
        """
        from .plc_scl import Step

        steps: List[Any] = []
        pending: Dict[str, bool] = {}

        def flush(name: str, **kw: Any) -> None:
            steps.append(Step(name=name, outputs=dict(pending), **kw))
            pending.clear()

        for st in self.steps:
            if isinstance(st, SignalStep):
                pending[st.signal] = st.value
            elif isinstance(st, WaitStep):
                flush(
                    tr("等待 {signal}").format(signal=st.signal),
                    wait=st.signal, wait_value=st.value,
                )
            elif isinstance(st, DwellStep):
                flush(tr("驻留 {s}s").format(s=st.seconds), delay=float(st.seconds))
            else:
                if pending:
                    flush(tr("调用 {sub} 前置输出").format(sub=st.subprogram))
                # 调用本身不进 PLC 步序：PLC 触发机器人走握手信号，不臆造"调用"
        if pending:
            flush(tr("末步输出"))
        return steps

    def dwell_total(self) -> float:
        return sum(float(st.seconds) for st in self.steps if isinstance(st, DwellStep))

    def summary(self) -> Dict[str, Any]:
        """组成摘要。**只有驻留是时间**，其余类型的耗时如实标注"取决于现场"。"""
        counts = {"call": 0, "signal": 0, "wait": 0, "dwell": 0}
        for st in self.steps:
            counts[_step_kind(st)] += 1
        return {
            "steps": len(self.steps),
            "lanes": self.lanes(),
            "counts": counts,
            "dwell_total_s": round(self.dwell_total(), 3),
            "note": tr("驻留合计是唯一可计算的时间；触发与等待的耗时取决于现场机构"),
        }

    # ---------------- JSON 往返 ---------------- #

    _FIELDS = {
        "call": ("name", "device", "subprogram"),
        "signal": ("name", "device", "signal", "value"),
        "wait": ("name", "device", "signal", "trigger", "value", "timeout"),
        "dwell": ("name", "device", "seconds"),
    }

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema": "orolp.sequence/1",
            "steps": [{"kind": _step_kind(st), **{
                k: getattr(st, k) for k in self._FIELDS[_step_kind(st)]
            }} for st in self.steps],
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ProcessSequence":
        schema = str(data.get("schema", ""))
        if schema != "orolp.sequence/1":
            raise ValueError(
                tr("不支持的时序 schema：{schema}").format(
                    schema=schema or tr("（缺失）")
                )
            )
        steps: List[SeqStep] = []
        for raw in data.get("steps", []):
            kind = str(raw.get("kind", ""))
            if kind not in cls._FIELDS:
                raise ValueError(tr("未知工步类型：{kind}").format(kind=kind))
            params = {k: raw[k] for k in cls._FIELDS[kind] if k in raw}
            step_cls = {"call": CallStep, "signal": SignalStep,
                        "wait": WaitStep, "dwell": DwellStep}[kind]
            steps.append(step_cls(**params))  # type: ignore[arg-type]
        return cls(steps)

    def save_json(self, path: Union[str, Path]) -> None:
        Path(path).write_text(
            json.dumps(self.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8"
        )

    @classmethod
    def load_json(cls, path: Union[str, Path]) -> "ProcessSequence":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls.from_dict(data)
