"""离线编程面板的抓取向导：由抓取点/放置点生成完整抓取循环。

用户只需把机器人摆到"要抓的地方"和"要放的地方"各示教一次，接近 / 提升 /
撤离这些安全高度点由向导按竖直偏移（世界 +Z，毫米）算出来——这是示教里
最费时间、也最容易教歪的一步。逆解失败时**不返回残缺序列**：宁可让用户
调小高度重试，也不要给一串看着完整、实则缺动作的示教点。
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Dict, List, Optional, Tuple

from PySide6 import QtWidgets

from domain.kinematics import KinematicsError
from domain.pose import Pose
from service.pickplace import PointType, default_types_for_count
from support.i18n import tr

if TYPE_CHECKING:  # 仅类型标注
    from app.views.program_panel import ProgramPanel


def grasp_cycle_points(
    panel,
    pick_index: int,
    place_index: Optional[int] = None,
    approach_mm: float = 50.0,
    lift_mm: float = 50.0,
    retract_mm: float = 50.0,
    robot: Optional[str] = None,
) -> Tuple[List[Dict[str, float]], List[PointType], List[str]]:
    """由一个**抓取点**（加可选放置点）生成完整的抓取循环点位。

    用户只需把机器人摆到"要抓的地方"和"要放的地方"各示教一次，接近 / 提升 /
    撤离这些安全高度点由向导按竖直偏移算出来——这是示教里最费时间、也最容
    易教歪的一步（手抖一下，接近点就可能离工件太远或撞上去）。

    偏移沿**世界竖直方向（+Z）**、单位毫米：接近点在抓取点上方
    ``approach_mm``，提升点在抓取点上方 ``lift_mm``，撤离点在放置点上方
    ``retract_mm``。

    Args:
        pick_index: 抓取点在当前示教点里的下标（0 起）
        place_index: 放置点下标；``None`` 表示只生成抓取侧（接近/抓取/提升）
        approach_mm / lift_mm / retract_mm: 竖直偏移，毫米
        robot: 算哪台机器人的；``None`` 表示当前这台

    Returns:
        ``(positions, types, motions)``——positions 为 ``{关节名: 弧度}``，
        types 为对应的 :class:`PointType`，motions 为运动类型
        （竖直升降段用 moveL，转运段用 moveJ）

    Raises:
        KinematicsError: 条件不足，或某个偏移点逆解失败（超出工作空间 /
            奇异位形）。**不返回残缺序列**——少一个点就不是完整的抓取循环，
            宁可让用户把高度调小重试，也不要给一串看着完整、实则缺动作的
            示教点
    """
    with panel._using_robot(robot) as state:
        if state.robot is None or not state.joint_names:
            raise KinematicsError(tr("还没有载入机器人，无法计算抓取点位"))
        taught = state.taught
        names = state.joint_names
        if not taught:
            raise KinematicsError(tr("还没有示教点——请先把机器人摆到抓取位并记录一个点"))
        if not (0 <= pick_index < len(taught)):
            raise KinematicsError(
                tr("抓取点下标 {i} 超出示教点范围（共 {n} 个）").format(
                    i=pick_index, n=len(taught)
                )
            )
        if place_index is not None and not (0 <= place_index < len(taught)):
            raise KinematicsError(
                tr("放置点下标 {i} 超出示教点范围（共 {n} 个）").format(
                    i=place_index, n=len(taught)
                )
            )

        def joints_at(index: int) -> List[float]:
            """第 ``index`` 个示教点的关节值（度）。"""
            return [math.degrees(float(taught[index].get(n, 0.0))) for n in names]

        def lifted(index: int, dz: float, guess: List[float], label: str) -> List[float]:
            """把第 ``index`` 个示教点的 TCP 沿世界 +Z 抬高 ``dz`` 后逆解。

            以 ``guess`` 为初值（这里传抓取/放置位形本身）：目标就在正上方，
            解出来自然与示教位形同构，不会翻腕翻到另一个姿态去。
            """
            pose = state.robot.forward_kinematics(joints_at(index))
            target = pose.matrix.copy()
            target[2, 3] += float(dz)
            try:
                return [
                    float(v) for v in state.robot.inverse_kinematics(Pose(target), guess=guess)
                ]
            except KinematicsError as exc:
                raise KinematicsError(
                    tr("{label}（抬高 {dz:.0f} mm）逆解失败：{msg}").format(
                        label=label, dz=dz, msg=exc
                    )
                ) from exc

        pick_q = joints_at(pick_index)

        def merged(source_index: int, q: List[float]) -> Dict[str, float]:
            """把解出的可动轴覆盖到来源示教点上，其余关节沿用示教值。

            末端工具轴（``tcp_joint``）这类非驱动关节不在 ``names`` 里，只按
            可动轴重建字典就会把它们整条丢掉——生成的点与用户示教的点结构
            对不上，写回场景时工具的关节值被悄悄抹成 0。
            """
            out = {n: float(v) for n, v in taught[source_index].items()}
            out.update({n: math.radians(float(q[i])) for i, n in enumerate(names)})
            return out

        sequence = [
            (
                pick_index,
                lifted(pick_index, approach_mm, pick_q, tr("接近点")),
                PointType.APPROACH,
                "J",
            ),
            (pick_index, pick_q, PointType.PICK, "L"),
            (
                pick_index,
                lifted(pick_index, lift_mm, pick_q, tr("提升点")),
                PointType.LIFT,
                "L",
            ),
        ]
        if place_index is not None:
            place_q = joints_at(place_index)
            sequence += [
                (place_index, place_q, PointType.PLACE, "J"),
                (
                    place_index,
                    lifted(place_index, retract_mm, place_q, tr("撤离点")),
                    PointType.RETRACT,
                    "L",
                ),
            ]

        positions = [merged(src, q) for src, q, _t, _m in sequence]
        return (
            positions,
            [t for _src, _q, t, _m in sequence],
            [m for _src, _q, _t, m in sequence],
        )


def apply_grasp_cycle(
    panel,
    pick_index: int,
    place_index: Optional[int] = None,
    approach_mm: float = 50.0,
    lift_mm: float = 50.0,
    retract_mm: float = 50.0,
    replace: bool = False,
    robot: Optional[str] = None,
) -> int:
    """生成抓取循环并写入 ``robot`` 的示教点。

    Args:
        replace: ``True`` 用生成的序列**替换**原有示教点；``False``（默认）
            **追加**到末尾。默认追加：原有示教点是用户一个个摆出来的，凭
            一次向导就抹掉太可惜——真要替换得由用户明确勾选
        robot: 写到哪台；``None`` 表示当前这台

    Returns:
        写入后的示教点总数

    Raises:
        KinematicsError: 点位算不出来（见 :meth:`grasp_cycle_points`）。
            此时**不改动**任何示教点
    """
    new_pos, new_types, new_motions = panel.grasp_cycle_points(
        pick_index, place_index, approach_mm, lift_mm, retract_mm, robot
    )
    with panel._using_robot(robot) as state:
        cur = [dict(p) for p in state.taught]
        if replace or not cur:
            positions, types, motions = new_pos, new_types, new_motions
        else:
            cur_types = state.point_types
            if cur_types is None or len(cur_types) != len(cur):
                # 语义缺失或长度对不上时按默认补齐——宁可给用户一份能跑的
                # 猜测，也不要让追加后整串语义错位
                cur_types = default_types_for_count(len(cur))
            cur_motions = (
                list(state.point_motions)
                if len(state.point_motions) == len(cur)
                else ["J"] * len(cur)
            )
            positions = cur + new_pos
            types = list(cur_types) + new_types
            motions = cur_motions + new_motions
        panel.set_taught_points(positions, types, motions, robot=robot)
        name = robot or panel._active
    # 面板内部的编辑广播不带机器人名（多机会串台），故走专用信号；当前这台
    # 额外走一次既有广播，让路径代码编辑器同步（与轨迹编辑器改点同一条路）
    panel.graspCycleApplied.emit(name or "")
    if name == panel._active:
        panel._emit_taught_changed()
    panel.statusMessage.emit(
        tr("抓取向导已生成 {n} 个点（{seq}）").format(
            n=len(new_pos),
            seq=" → ".join(t.label for t in new_types),
        )
    )
    return len(positions)


def open_grasp_wizard(panel, robot: Optional[str] = None) -> bool:
    """弹出抓取向导对话框，按用户给的两个示教点生成完整抓取循环。

    Args:
        robot: 对哪台机器人用向导；``None`` 表示当前这台

    Returns:
        确实写入了点位 ``True``；用户取消或算不出来 ``False``
    """
    with panel._using_robot(robot) as state:
        taught = state.taught
        name = robot or panel._active
    if not taught:
        QtWidgets.QMessageBox.information(
            panel,
            tr("抓取向导"),
            tr("请先记录至少一个示教点——把机器人摆到要抓的位置，点「记录」"),
        )
        return False
    if state.robot is None:
        QtWidgets.QMessageBox.information(
            panel, tr("抓取向导"), tr("还没有载入机器人，无法计算抓取点位")
        )
        return False

    dlg = QtWidgets.QDialog(panel)
    dlg.setWindowTitle(tr("抓取向导"))
    form = QtWidgets.QFormLayout(dlg)

    def point_label(i: int) -> str:
        return tr("P{n}").format(n=i + 1)

    combo_pick = QtWidgets.QComboBox()
    combo_place = QtWidgets.QComboBox()
    for i in range(len(taught)):
        combo_pick.addItem(point_label(i), i)
        combo_place.addItem(point_label(i), i)
    combo_place.addItem(tr("（不生成放置侧）"), -1)
    # 默认：抓取点取当前选中行，放置点默认不选（多数人先只想把抓取侧做出来）
    selected = panel._selected_index()
    combo_pick.setCurrentIndex(selected if selected is not None else len(taught) - 1)
    combo_place.setCurrentIndex(len(taught))  # 最后一项 = 不生成

    spin_approach = QtWidgets.QDoubleSpinBox()
    spin_lift = QtWidgets.QDoubleSpinBox()
    spin_retract = QtWidgets.QDoubleSpinBox()
    for spin, value in ((spin_approach, 50.0), (spin_lift, 50.0), (spin_retract, 50.0)):
        spin.setRange(0.0, 2000.0)
        spin.setDecimals(1)
        spin.setSuffix(" mm")
        spin.setValue(value)
    spin_retract.setEnabled(False)

    check_replace = QtWidgets.QCheckBox(tr("替换原有示教点（默认追加到末尾）"))

    preview = QtWidgets.QLabel(tr("接近点 → 抓取点 → 提升点"))
    preview.setWordWrap(True)

    def refresh_preview() -> None:
        spin_retract.setEnabled(combo_place.currentData() != -1)
        seq = [tr("接近点"), tr("抓取点"), tr("提升点")]
        if combo_place.currentData() != -1:
            seq += [tr("放置点"), tr("撤离点")]
        preview.setText(tr("将生成 {n} 个点：{seq}").format(n=len(seq), seq=" → ".join(seq)))

    combo_place.currentIndexChanged.connect(lambda _i: refresh_preview())
    refresh_preview()

    form.addRow(tr("抓取点"), combo_pick)
    form.addRow(tr("放置点"), combo_place)
    form.addRow(tr("接近高度（抓取点上方）"), spin_approach)
    form.addRow(tr("提升高度（抓取点上方）"), spin_lift)
    form.addRow(tr("撤离高度（放置点上方）"), spin_retract)
    form.addRow(check_replace)
    form.addRow(preview)

    buttons = QtWidgets.QDialogButtonBox(
        QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
    )
    buttons.button(QtWidgets.QDialogButtonBox.Ok).setText(tr("生成"))
    buttons.button(QtWidgets.QDialogButtonBox.Cancel).setText(tr("取消"))
    buttons.accepted.connect(dlg.accept)
    buttons.rejected.connect(dlg.reject)
    form.addRow(buttons)

    if not dlg.exec():
        return False

    place = combo_place.currentData()
    try:
        panel.apply_grasp_cycle(
            pick_index=int(combo_pick.currentData()),
            place_index=None if place == -1 else int(place),
            approach_mm=spin_approach.value(),
            lift_mm=spin_lift.value(),
            retract_mm=spin_retract.value(),
            replace=check_replace.isChecked(),
            robot=name,
        )
    except KinematicsError as exc:
        # 逆解失败不能悄无声息：告诉用户是哪个点、抬了多少，让他调小高度重试
        QtWidgets.QMessageBox.warning(
            panel,
            tr("抓取向导"),
            tr("点位算不出来，示教点未改动：\n{msg}").format(msg=exc),
        )
        return False
    return True
