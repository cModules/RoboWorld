"""离线编程面板的机器人管理：登记、切换、注销、场景绑定。

每台机器人按名字各自留存一份 :class:`~app.views.program_state.RobotProgram`
状态槽，多机不再互相顶掉；登记/切换/注销都会同步面板顶部的机器人下拉，
并向主窗口广播 ``activeRobotChanged`` 以同步示教点与视口显示。
"""

from __future__ import annotations

from typing import TYPE_CHECKING, List, Optional, Sequence

from support.i18n import tr

if TYPE_CHECKING:  # 仅类型标注
    from domain.robot import Robot
    from domain.world.scene import Scene

    from app.views.program_panel import ProgramPanel


def set_robot(
    panel,
    robot: Optional["Robot"],
    joint_names: Optional[Sequence[str]] = None,
    name: Optional[str] = None,
) -> None:
    """登记一台机器人：**按名字各自留存**，多机不再互相顶掉。

    Args:
        robot: 机器人实例；``None`` 表示卸载 ``name``（或当前这台）
        joint_names: 示教点字典里用的关节名（视口渲染器的命名，可能比
            可驱动关节多——URDF 里的 ``fixed`` 关节，如 ``tcp_joint``，
            也会被渲染器登记）。为 ``None`` 时取 ``robot.joint_names``
        name: 场景里的机器人名。缺省取 ``robot.name``。登记后自动切到它，
            这样示教/校验/出码/回放都作用在刚加进来的那台上

    Note:
        这里必须把关节名过滤到**可驱动**的那些。渲染器会把 fixed 关节
        一并登记，照单全收的话 ``moveJ`` 会收到多出来的分量，直接被
        自由度校验拒掉——表现为"每条指令都失败"，极难排查。
    """
    if robot is None:
        panel.remove_robot(name or panel._active)
        return

    key = name or robot.name
    # 必须先把「当前」指过去：下面的赋值都经由代理属性落到这一台的槽里
    panel._active = key
    state = panel._ensure_state(key)

    panel.stop_playback()
    state.trajectory = None
    state.report = None
    state.point_issue_level = {}
    state.build_notes = []
    panel.issue_list.clear()

    source = list(joint_names or robot.joint_names)
    actuated = list(robot.joint_names)
    known = set(actuated)
    filtered = [n for n in source if n in known]
    # 命名对不上时宁可退回权威列表，也不要留下长度不匹配的隐患
    state.joint_names = filtered if len(filtered) == len(actuated) else actuated
    panel._robot = robot
    panel.btn_generate.setEnabled(True)
    # 「导出全部」的门槛与「生成代码」完全相同——它内部自己构建程序
    # （见 export_all_programs 的文档），不该逼用户先点一次「生成代码」才亮。
    panel._refresh_export_all_enabled()
    panel.lbl_status.setText(tr("{name} · 就绪").format(name=key))
    panel.lbl_status.setObjectName("ok")
    # 重新套用样式表，objectName 改变后需手动刷新
    panel.lbl_status.style().polish(panel.lbl_status)
    panel._refresh_robot_combo(key)


def active_robot(panel) -> Optional[str]:
    """当前正在编辑/回放的机器人名。"""
    return panel._active


def robot_names(panel) -> List[str]:
    """已登记的机器人名（插入顺序）。"""
    return list(panel._states)


def set_active_robot(panel, name: Optional[str]) -> bool:
    """切换当前编辑的机器人，界面随之回填**它自己**的数据。

    Returns:
        切换成功 ``True``；名字未登记返回 ``False``
    """
    if not name or name not in panel._states:
        return False
    if name != panel._active:
        panel.stop_playback()
    panel._active = name
    panel._refresh_robot_combo(name)
    panel._display_active()
    return True


def remove_robot(panel, name: Optional[str]) -> None:
    """注销一台机器人（场景里删掉它时调用），遗留状态一并清理。"""
    if not name or name not in panel._states:
        return
    if name == panel._active:
        panel.stop_playback()
    panel._states.pop(name, None)
    panel._refresh_robot_combo()
    if name == panel._active:
        panel._active = next(iter(panel._states), None)
        panel._display_active()


def _on_robot_combo_changed(panel, _index: int) -> None:
    """下拉换人：切到对应机器人，并广播给主窗口同步示教点/视口。"""
    name = panel.combo_robot.currentData()
    if not name or name == panel._active:
        return
    panel.set_active_robot(name)
    panel.activeRobotChanged.emit(name)


def set_scene(panel, scene: Optional["Scene"]) -> None:
    """绑定工作单元场景；给出后校验会包含沿路径干涉检查。"""
    panel._scene = scene
