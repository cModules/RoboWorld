"""离线编程面板的轨迹编辑器：路点增删移改、逐点运动类型、列表渲染与着色。

圆弧（v1.1.5）口径：**第 N 行的运动类型描述「到达第 N 点」的运动**——
``C`` 表示"以上一点为中间点、再上一点为起点，圆弧到达本点"。中间点被并进
上一条指令、不再单独出运动指令，因此路点行号与指令序号不再一一对应，
节拍明细与问题着色都要经 ``instruction_of_point`` 的映射取值。
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Dict, List, Optional, Sequence, Tuple

from PySide6 import QtGui, QtWidgets

from service.pickplace import gripper_action_for
from support.i18n import tr

if TYPE_CHECKING:  # 仅类型标注
    from service.pickplace import PointType
    from service.planner import ProgramReport

    from app.views.program_panel import ProgramPanel


def set_taught_points(
    panel,
    positions: Sequence[Dict[str, float]],
    types: Optional[Sequence["PointType"]] = None,
    motions: Optional[Sequence[str]] = None,
    robot: Optional[str] = None,
) -> None:
    """导入示教点（``{关节名: 弧度}``）。

    Args:
        positions: 示教点序列
        types: 与示教点对齐的工艺语义（``PointType.PICK/PLACE…``）；
            给出后 ``build_program`` 会在抓取/放置点自动插入
            ``gripper()`` 指令，程序与动画共用同一份抓取定义
        motions: 与示教点对齐的运动类型（``"J"``/``"L"``/``"C"``）。
            为 ``None`` 且数量不变时沿用当前逐点运动类型（轨迹编辑器
            的改动不被外部同步覆盖），否则重置为全 moveJ
        robot: 写到哪台机器人的名下；``None`` 表示当前这台。给非当前机器人
            喂数据时界面不会跟着跳（主窗口批量同步多台时要这个语义）
    """
    # 喂给非当前那台时临时切过去写，写完切换回来——界面不能跟着跳
    hidden = robot is not None and robot in panel._states and robot != panel._active
    prev = panel._active
    if hidden:
        panel._active = robot
    try:
        panel._taught = [dict(p) for p in positions]
        panel._point_types = list(types) if types is not None else None
        # 长度必须对齐：语义少了会漏动作，多了会错位
        if panel._point_types is not None and len(panel._point_types) != len(panel._taught):
            panel._point_types = None
        if motions is not None and len(motions) == len(panel._taught):
            panel._point_motions = [
                k if k in ("L", "C") else "J" for k in (str(m).upper() for m in motions)
            ]
        elif len(panel._point_motions) != len(panel._taught):
            panel._point_motions = ["J"] * len(panel._taught)
        n = len(panel._taught)
        # 点数变了，旧报告的节拍明细/着色不再对得上号——直接作废
        if getattr(panel, "_last_point_count", None) != n:
            panel._report = None
            panel._point_issue_level = {}
            panel._last_point_count = n
    finally:
        panel._active = prev

    if hidden:
        return
    panel.statusMessage.emit(tr("导入 {n} 个示教点").format(n=n))
    panel._refresh_point_list()
    # 有了示教点才有东西可导出：「导出全部」的可用性随点表一起变，
    # 只靠登记机器人时点一次会一直灰着
    panel._refresh_export_all_enabled()


def get_taught_points(panel, robot: Optional[str] = None) -> tuple:
    """读取示教点数据（供外部同步与测试）。

    Args:
        robot: 读哪台的；``None`` 表示当前这台。抓取向导写完点后主窗口要
            按名字把点位回灌视口，故支持指定机器人

    Returns:
        ``(positions, types, motions)``——positions 为
        ``{关节名: 弧度}`` 字典列表的拷贝；types 可为 ``None``；
        motions 为 ``"J"/"L"/"C"`` 列表
    """
    with panel._using_robot(robot) as state:
        positions = [dict(p) for p in state.taught]
        types = list(state.point_types) if state.point_types is not None else None
        return positions, types, list(state.point_motions)


def _selected_index(panel) -> Optional[int]:
    """当前选中行号；未选中返回 ``None``。"""
    row = panel.point_list.currentRow()
    return row if 0 <= row < len(panel._taught) else None


def _motion_kind(panel, index: int) -> str:
    """第 ``index`` 行声明的运动类型（``"J"`` / ``"L"`` / ``"C"``）。"""
    if index < len(panel._point_motions):
        kind = str(panel._point_motions[index]).upper()
        if kind in ("J", "L", "C"):
            return kind
    return "J"


def _gripper_action_at(panel, index: int) -> Optional[str]:
    """第 ``index`` 行是否带夹爪动作（抓取/放置语义）。"""
    if panel._point_types is not None and index < len(panel._point_types):
        return gripper_action_for(panel._point_types[index])
    return None


def arc_rejection(panel, index: int) -> Optional[str]:
    """把第 ``index`` 行设为 ``moveC`` 是否可行；不可行时返回原因（可直接展示）。

    圆弧的约定（轨迹编辑器里的口径）：**第 N 行的运动类型描述"到达第 N 点"
    的运动**——``C`` 表示"以上一点为中间点、再上一点为起点，圆弧到达本点"。
    因此：

    * 第 1、2 行不能是圆弧（没有两个前驱点）；
    * 相邻两行不能同为圆弧（中间点会同时成为上一段的终点，几何上矛盾）；
    * 带工艺语义的点（抓取/放置）不能当中间点——中间点不停机，夹爪动作
      会失去落点。

    Returns:
        拒绝原因（中文，可直接显示）；可行时 ``None``
    """
    if not (0 <= index < len(panel._taught)):
        return tr("路点不存在")
    if index < 2:
        return tr("P{n} 缺少两个前驱点——圆弧需要起点与中间点，P1/P2 不能设为 moveC").format(
            n=index + 1
        )
    if panel._motion_kind(index - 1) == "C":
        return tr("P{n} 已是圆弧终点，不能再作为本段的中间点").format(n=index)
    if index + 1 < len(panel._taught) and panel._motion_kind(index + 1) == "C":
        return tr("P{n} 已把 P{k} 当中间点——本行不能同时作为圆弧终点").format(
            n=index + 2, k=index + 1
        )
    if panel._gripper_action_at(index - 1) is not None:
        return tr("P{n} 是抓取/放置点，不能作为圆弧中间点（中间点不停机）").format(n=index)
    return None


def motion_plan(panel) -> Tuple[List[Tuple[str, int, Optional[int]]], List[str]]:
    """把逐点运动类型解析成可执行的运动序列。

    Returns:
        ``(plan, notes)``：plan 每项为 ``(kind, point_index, via_index)``，
        ``kind`` ∈ ``{"J","L","C"}``，``via_index`` 仅圆弧有值；
        notes 为降级说明（圆弧条件不满足时退化为关节运动，这里如实记下）。
    """
    n = len(panel._taught)
    declared = [panel._motion_kind(i) for i in range(n)]
    accepted: Dict[int, int] = {}  # 圆弧终点行 → 中间点行
    notes: List[str] = []
    for i in range(n):
        if declared[i] != "C":
            continue
        reason = panel.arc_rejection(i)
        if reason is not None:
            notes.append(tr("{reason}——已按关节运动（moveJ）处理").format(reason=reason))
            continue
        accepted[i] = i - 1
    vias = set(accepted.values())
    plan: List[Tuple[str, int, Optional[int]]] = []
    for i in range(n):
        if i in vias:  # 中间点不再单独出运动指令
            continue
        if i in accepted:
            plan.append(("C", i, accepted[i]))
        else:
            kind = declared[i]
            plan.append((("J" if kind == "C" else kind), i, None))
    return plan, notes


def arc_via_indices(panel) -> set:
    """当前被当作圆弧中间点（不再单独出指令）的路点行号集合。"""
    return {via for _kind, _i, via in panel.motion_plan()[0] if via is not None}


def instruction_of_point(panel) -> Dict[int, int]:
    """路点行号 → 运动指令序号（圆弧把中间点并进上一条指令，两者不再一一对应）。"""
    plan, _notes = panel.motion_plan()
    return {index: order for order, (_k, index, _v) in enumerate(plan)}


def last_build_notes(panel) -> List[str]:
    """最近一次 ``build_program`` 的降级说明（空表表示没有降级）。"""
    return list(panel._build_notes)


def _emit_taught_changed(panel) -> None:
    """示教点被编辑后广播，供主窗口回写路径代码编辑器。"""
    positions, types, motions = panel.get_taught_points()
    panel.taughtPointsChanged.emit(positions, types or [], motions)


def _invalidate_results(panel) -> None:
    """示教点变了，旧轨迹/报告/回放全部作废。"""
    panel.stop_playback()
    panel._trajectory = None
    panel._report = None
    panel._planner = None
    panel._gripper_animator = None
    # 「全部回放」跟着失效：轨迹没了就不该再能点——否则按下去播的是
    # 内存里已经被判废的旧轨迹
    if getattr(panel, "btn_play_all", None) is not None:
        panel.btn_play_all.setEnabled(bool(panel._playable()))


def _keep_row_selected(panel, row: int) -> None:
    """把第 ``row`` 行设为选中并滚进视野。

    增删改之后列表是**整体重建**的：重建会把选中清掉，长点表里就再也找不
    到刚动过的那一行。移动类操作一律走这里。注意 ``setCurrentRow`` 只设
    **当前行**、不设**选中**——重建后的行没有选中态，深色主题下只显示
    交替行的淡色而不是蓝色高亮，用户看到的就是"选中的颜色变淡了"
    （2026-09-21 用户反馈）。所以这里显式 ``setSelected(True)``。
    """
    if not (0 <= row < panel.point_list.count()):
        return
    panel.point_list.setCurrentRow(row)
    item = panel.point_list.item(row)
    if item is not None:
        item.setSelected(True)
        panel.point_list.scrollToItem(item, QtWidgets.QAbstractItemView.EnsureVisible)


def insert_point(panel, index: Optional[int] = None) -> Optional[int]:
    """在 ``index`` 之后插入路点，复制其关节值与运动类型。

    Args:
        index: 插入位置（新点落在该序号后一位）；``None`` 取当前
            选中行，无选中时追加到末尾。首点为空时填零位

    Returns:
        新点序号；示教点未初始化时返回 ``None``
    """
    if isinstance(index, bool):  # clicked(bool) 之类把 False 顶了进来
        index = None
    if not panel._taught:
        if panel._joint_names:
            zero = {name: 0.0 for name in panel._joint_names}
            panel._taught = [zero]
            panel._point_types = None
            panel._point_motions = ["J"]
        else:
            panel.statusMessage.emit(tr("请先载入机器人再添加路点"))
            return None
    else:
        if index is None:
            index = panel._selected_index()
        if index is None or not (0 <= index < len(panel._taught)):
            index = len(panel._taught) - 1
        src = dict(panel._taught[index])
        panel._taught.insert(index + 1, src)
        if panel._point_types is not None:
            panel._point_types.insert(index + 1, panel._point_types[index])
        # 复制运动类型：同一位置附近通常保持同一种走法
        panel._point_motions.insert(index + 1, panel._point_motions[index])
        index = index + 1
    panel._invalidate_results()
    panel._refresh_point_list()
    panel._keep_row_selected(index)
    panel._emit_taught_changed()
    panel._keep_row_selected(index)  # 回灌可能重建列表，选中再兜一次
    panel.statusMessage.emit(tr("已插入路点 {n}，请重新校验").format(n=index + 1))
    return index


def delete_point(panel, index: Optional[int] = None) -> bool:
    """删除 ``index`` 处的路点（``None`` 取当前选中行）。

    删完把选中落在**顶上来的那一行**：后续继续删相邻点时不必重新点选。
    """
    if isinstance(index, bool):  # clicked(bool) 之类把 False 顶了进来
        index = None
    if not panel._taught:
        return False
    if index is None:
        index = panel._selected_index()
    if index is None or not (0 <= index < len(panel._taught)):
        panel.statusMessage.emit(tr("请先在列表中选中要删除的路点"))
        return False
    del panel._taught[index]
    if panel._point_types is not None and index < len(panel._point_types):
        del panel._point_types[index]
    if index < len(panel._point_motions):
        del panel._point_motions[index]
    panel._invalidate_results()
    panel._refresh_point_list()
    follow = min(index, panel.point_list.count() - 1)
    panel.point_list.setCurrentRow(follow)
    panel._emit_taught_changed()
    panel._keep_row_selected(follow)
    panel.statusMessage.emit(tr("已删除路点 {n}，请重新校验").format(n=index + 1))
    return True


def move_point(panel, src: int, dst: int) -> bool:
    """把路点从 ``src`` 移动到 ``dst``（``dst`` 为移动后的新序号）。

    移动的**是那一个点本身**：关节值、工艺语义、运动类型一起走，列表里的
    ``P`` 序号按行序重排（第 N 行就是 P{N+1}），因此"被移动的点"到了新位置
    就顶新的号——不会留下"坐标走了、序号还挂旧行"的两张皮。选中也跟着它走。
    """
    n = len(panel._taught)
    if not (0 <= src < n) or not (0 <= dst < n) or src == dst:
        return False
    for seq in (panel._taught, panel._point_motions):
        seq.insert(dst, seq.pop(src))
    if panel._point_types is not None and len(panel._point_types) == n:
        panel._point_types.insert(dst, panel._point_types.pop(src))
    panel._invalidate_results()
    panel._refresh_point_list()
    panel._keep_row_selected(dst)
    panel._emit_taught_changed()
    # 回灌链路（主窗口按机器人回写点表）若顺带重建了列表，选中会被抹掉——
    # 再兜一次，保证"原来选中的还是选中的"
    panel._keep_row_selected(dst)
    panel.statusMessage.emit(tr("P{a} 已移到 P{b}，请重新校验").format(a=src + 1, b=dst + 1))
    return True


def shift_selected_point(panel, delta: int) -> bool:
    """把当前选中路点上移（``delta=-1``）或下移（``delta=1``）。"""
    index = panel._selected_index()
    if index is None:
        panel.statusMessage.emit(tr("请先在列表中选中要移动的路点"))
        return False
    return panel.move_point(index, index + delta)


def set_point_motion(panel, index: int, motion: str) -> bool:
    """设置单个路点的运动类型（``"J"``=moveJ / ``"L"``=moveL / ``"C"``=moveC）。

    ``"C"`` 不满足圆弧条件时**拒绝设置并说明原因**（见 :meth:`arc_rejection`）——
    悄悄改成别的运动类型会让"我以为这是圆弧"变成现场查不动的差异。
    """
    if not (0 <= index < len(panel._taught)):
        return False
    kind = str(motion).upper()
    if kind == "C":
        reason = panel.arc_rejection(index)
        if reason is not None:
            panel.statusMessage.emit(reason)
            return False
    motion = kind if kind in ("L", "C") else "J"
    if 0 <= index < len(panel._point_motions) and panel._point_motions[index] == motion:
        return False
    while index >= len(panel._point_motions):
        panel._point_motions.append("J")
    panel._point_motions[index] = motion
    panel._invalidate_results()
    panel._refresh_point_list()
    panel._emit_taught_changed()
    return True


def update_point(
    panel,
    index: int,
    joints_deg: Sequence[float],
    motion: Optional[str] = None,
) -> bool:
    """写入路点的关节角（**度**）与可选的运动类型。

    Args:
        index: 路点序号
        joints_deg: 各关节角度值，顺序与 ``_joint_names`` 对齐
        motion: ``"J"/"L"/"C"``；``None`` 保持不变
    """
    if not (0 <= index < len(panel._taught)):
        return False
    pos = panel._taught[index]
    for i, name in enumerate(panel._joint_names):
        if i < len(joints_deg):
            pos[name] = math.radians(float(joints_deg[i]))
    if motion is not None:
        panel.set_point_motion(index, motion)
    panel._invalidate_results()
    panel._refresh_point_list()
    panel._emit_taught_changed()
    return True


def edit_selected_point(panel) -> None:
    """弹出对话框编辑当前选中路点的关节角与运动类型。"""
    index = panel._selected_index()
    if index is None or not panel._taught:
        panel.statusMessage.emit(tr("请先在列表中选中要编辑的路点"))
        return
    joints = [math.degrees(float(panel._taught[index].get(n, 0.0))) for n in panel._joint_names]
    motion = panel._point_motions[index] if index < len(panel._point_motions) else "J"

    dialog = QtWidgets.QDialog(panel)
    dialog.setWindowTitle(tr("编辑路点 {n}").format(n=index + 1))
    form = QtWidgets.QFormLayout(dialog)
    spins: List[QtWidgets.QDoubleSpinBox] = []
    for i, name in enumerate(panel._joint_names):
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(-360.0, 360.0)
        spin.setDecimals(2)
        spin.setSuffix(" °")
        if i < len(joints):
            spin.setValue(joints[i])
        spins.append(spin)
        form.addRow(name, spin)
    combo = QtWidgets.QComboBox()
    combo.addItem(tr("moveJ（关节插值）"), "J")
    combo.addItem(tr("moveL（直线插值）"), "L")
    combo.addItem(tr("moveC（圆弧，以上一点为中间点）"), "C")
    combo.setCurrentIndex({"L": 1, "C": 2}.get(str(motion).upper(), 0))
    form.addRow(tr("运动类型"), combo)
    if motion != "C":
        hint = panel.arc_rejection(index)
        hint_label = QtWidgets.QLabel(
            tr("可设为 moveC（圆弧）")
            if hint is None
            else tr("不可设为 moveC：{reason}").format(reason=hint)
        )
        hint_label.setWordWrap(True)
        hint_label.setStyleSheet("color:#90caf9; font-size:11px;")
        form.addRow(hint_label)
    buttons = QtWidgets.QDialogButtonBox(
        QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
    )
    buttons.accepted.connect(dialog.accept)
    buttons.rejected.connect(dialog.reject)
    form.addRow(buttons)
    if dialog.exec() != QtWidgets.QDialog.Accepted:
        return
    panel.update_point(
        index,
        [spin.value() for spin in spins],
        combo.currentData(),
    )
    panel.statusMessage.emit(tr("{n} 已更新，请重新校验").format(n=index + 1))


def _refresh_point_list(panel) -> None:
    """按当前示教点数据重建列表，并套用最近一次校验的着色。

    圆弧（v1.1.5）会打破"每个路点恰好一条运动指令"的旧前提——圆弧的中间点
    被并进上一条指令。因此节拍明细必须经路点→指令序号映射取值，**不能按下标
    硬取**（否则圆弧之后的每一行节拍都会错位一行）。
    """
    report = panel._report
    motion_times = report.motion_times if report is not None else None
    labels = report.motion_labels if report is not None else None
    levels = getattr(panel, "_point_issue_level", {}) if report is not None else {}
    point_to_instruction = panel.instruction_of_point() if report is not None else {}
    vias = panel.arc_via_indices()

    panel.point_list.clear()
    for i, pos in enumerate(panel._taught):
        mtype = panel._motion_kind(i)
        if i in vias:
            label = tr("moveC 中间点")
        elif mtype == "C":
            label = "moveC"
        elif mtype == "L":
            label = "moveL"
        else:
            label = "moveJ"
        joints = ", ".join(
            f"{math.degrees(float(pos.get(n, 0.0))):.1f}" for n in panel._joint_names
        )
        text = f"P{i + 1} · {label} · [{joints}]"
        instruction = point_to_instruction.get(i)
        if (
            motion_times is not None
            and instruction is not None
            and instruction < len(motion_times)
        ):
            kind = labels[instruction] if labels and instruction < len(labels) else label
            text += f" · {kind} {motion_times[instruction]:.2f}s"
        item = QtWidgets.QListWidgetItem(text)
        level = levels.get(i)
        if level == "error":
            item.setForeground(QtGui.QColor("#ff8080"))
            item.setToolTip(tr("该校验错误涉及此路点（见校验结果）"))
        elif level == "warning":
            item.setForeground(QtGui.QColor("#ffc866"))
            item.setToolTip(tr("该校验警告涉及此路点（见校验结果）"))
        elif report is not None:
            item.setForeground(QtGui.QColor("#6fdc6f"))
        panel.point_list.addItem(item)


def _apply_point_colors(panel, report: "ProgramReport") -> None:
    """从校验报告提取每个路点的最严重问题级别并刷新列表着色。

    ``Issue.motion_index`` 是**运动指令**序号，而列表行是**路点**序号。无圆弧时
    两者一一对应，有圆弧时中间点被并进上一条指令（指令数少于路点数），因此这里
    反向映射指令→路点；指向中间点的指令把着色落在**圆弧终点**那一行——问题实际
    出在圆弧段上，落在被并入的那一行最容易被看见。
    """
    panel._point_issue_level = panel._point_levels(report)
    panel._refresh_point_list()
