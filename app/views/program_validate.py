"""离线编程面板的程序装配与校验：示教点 → Program → 校验报告/轨迹。

``_build_program_impl`` 按 :meth:`motion_plan` 的逐点运动类型装配程序
（moveJ 直接下发关节值；moveL 先 FK 再直线插补；moveC 以上一点为中间点），
抓取/放置语义点自动追加 ``gripper()`` 指令——回放动画、程序校验与后处理
出码共用同一份抓取时序。校验是只读分析：干涉检查写走的场景关节角在
``_restore_scene_joints`` 里原样还回，不留副作用。
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Dict, List, Optional

from PySide6 import QtCore, QtWidgets

from domain.world.animation import GraspAnimator
from service.pickplace import gripper_action_for
from service.planner import SEVERITY_ERROR, TrajectoryPlanner
from service.program import ProgramBuilder
from support.i18n import tr

from app.logging import logger

if TYPE_CHECKING:  # 仅类型标注
    from service.planner import ProgramReport
    from service.program import Program

    from app.views.program_panel import ProgramPanel


def build_program(panel, robot: Optional[str] = None) -> Optional["Program"]:
    """组装**指定机器人**（默认当前这台）的程序。

    Args:
        robot: 机器人名；``None`` 表示当前正在编辑的那台。批量出全部機器人
            的代码时逐个指名即可，界面不会跟着切走
    """
    with panel._using_robot(robot):
        return panel._build_program_impl()


def _build_program_impl(panel) -> Optional["Program"]:
    # 实际装配：调用方已把当前槽切到目标机器人
    """把示教点组装成程序。

    示教点内部以弧度保存，:class:`Motion` 期望角度。
    带工艺语义的示教点（抓取/放置）会自动追加 ``gripper()`` 指令组，
    于是回放动画、程序校验与后处理出码共用同一份抓取时序。
    运动类型逐点取自轨迹编辑器（:meth:`motion_plan`）：``"L"`` 的点位先对该
    关节配置做 FK 得到 TCP 位姿、再以 ``moveL`` 直线插补过去；``"C"`` 的点位
    以**上一点为中间点**做 ``moveC``（中间点不再单独出指令）。

    圆弧正解失败（位形奇异/越限等）时**退化为关节运动并记入**
    :meth:`last_build_notes`，绝不静默改成另一条几何路径却让界面以为还是圆弧。

    Returns:
        :class:`Program`；无机器人或无示教点时返回 ``None``
    """
    panel._build_notes = []
    if panel._robot is None or not panel._taught:
        return None
    speed = float(panel.spin_speed.value())
    builder = ProgramBuilder(panel._robot)
    plan, notes = panel.motion_plan()
    panel._build_notes.extend(notes)

    def _pose_of(index: int):
        """路点关节配置 → TCP 位姿（度）。"""
        pos = panel._taught[index]
        joints = [math.degrees(float(pos.get(n, 0.0))) for n in panel._joint_names]
        return panel._robot.forward_kinematics(joints), joints

    def _joint_move(index: int) -> None:
        """关节运动到某个示教点——直接下发关节值、不走 FK，任何位形都可用。"""
        joints = [
            math.degrees(float(panel._taught[index].get(n, 0.0))) for n in panel._joint_names
        ]
        builder.moveJ(joints, speed=speed)

    def _gripper_after(index: int) -> None:
        """该点的工艺动作（抓取/放置）——回放与出码共用这份时序。"""
        t = (
            panel._point_types[index]
            if panel._point_types and index < len(panel._point_types)
            else None
        )
        action = gripper_action_for(t)
        if action:
            builder.gripper(action)

    for kind, i, via_index in plan:
        if kind == "C":
            moved = False
            try:
                via_pose, _ = _pose_of(via_index)
                target_pose, _ = _pose_of(i)
            except Exception:
                panel._build_notes.append(
                    tr(
                        "P{n} 的圆弧正解失败——退化为关节运动（moveJ）；中间点 P{k} 改为单独经过"
                    ).format(n=i + 1, k=via_index + 1)
                )
            else:
                try:
                    builder.moveC(via_pose, target_pose, speed=speed)
                    moved = True
                except Exception:
                    panel._build_notes.append(
                        tr(
                            "P{n} 的圆弧无法加入程序——退化为关节运动（moveJ）；中间点 P{k} 改为单独经过"
                        ).format(n=i + 1, k=via_index + 1)
                    )
            if not moved:
                # 圆弧没了，中间点也不能跟着一起消失：示教了 4 个点却只走 3 个，
                # 现场表现就是"这一拐没绕过去"。补一条 moveJ 单独经过中间点——
                # 精度不如圆弧，但经过的示教点一个不少。
                _joint_move(via_index)
                _joint_move(i)
        elif kind == "L":
            moved = False
            try:
                pose, _ = _pose_of(i)
                builder.moveL(pose, speed=speed)
                moved = True
            except Exception:
                # FK 失败（位形奇异/越限等）退回关节运动：宁可走不到
                # 直线，也不能让整个程序构建崩掉
                pass
            if not moved:
                _joint_move(i)
        else:
            _joint_move(i)
        _gripper_after(i)
    return builder.build()


def _scene_joint_snapshot(panel, name: Optional[str]):
    """校验前把 ``name`` 的场景关节角拍个快照（恢复用）；拍不到返回 ``None``。"""
    scene = panel._scene
    if scene is None or not name:
        return None
    state = getattr(scene, "robots", {}).get(name)
    joints = getattr(state, "joints_deg", None)
    if joints is None:
        return None
    return (name, [float(v) for v in joints])


def _restore_scene_joints(panel, snapshot) -> None:
    """把校验期间被干涉检查写走的场景关节角**原样还回去**。

    干涉检查沿轨迹密集采样，且会把采样到的位形写回场景（规划器的
    历史契约，单点位形检查依赖它）。副作用是：校验一跑完，**场景**里
    的机器人停在轨迹末端的位形，而**视口渲染器**还在原地——挂载件
    （夹爪/工件）的位姿按场景关节算，于是"夹爪离开法兰跳到别处"。
    校验是只读的分析，不该留这种副作用，这里统一恢复。
    """
    if not snapshot or panel._scene is None:
        return
    name, joints = snapshot
    try:
        panel._scene.set_robot_joints(joints, name=name)
    except Exception as exc:  # 恢复失败不该盖过校验结果本身
        logger.warning("restore_scene_joints_failed: %s", exc)


def validate_program(panel) -> Optional["ProgramReport"]:
    """校验程序并刷新界面。

    面板自己的槽位是上次同步时的快照，而点表的最新版本在主窗口那边
    （"教点 → 回灌"是两条通路）。点按钮时先请求回灌一次，避免"校的是
    旧数据 / 这一槽压根没灌过点点看起来像没反应"。

    Returns:
        :class:`ProgramReport`；条件不足时返回 ``None``
    """
    panel.syncRequested.emit(panel._active)
    if panel._robot is None:
        panel.statusMessage.emit(tr("请先载入机器人型号，再校验程序"))
        return None

    program = panel.build_program()
    if program is None:
        panel.statusMessage.emit(tr("没有示教点可供校验——请先记录若干关节位形"))
        return None

    QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
    snapshot = panel._scene_joint_snapshot(panel._active)
    try:
        report = program.validate(scene=panel._scene)
        planner = TrajectoryPlanner(panel._robot, scene=panel._scene)
        panel._trajectory = planner.plan(program)
    except Exception as exc:
        # 校验本身崩了也要让用户看见——这类异常几乎总是模型数据的问题
        panel.statusMessage.emit(
            tr("校验异常：{kind}: {msg}").format(kind=type(exc).__name__, msg=exc)
        )
        return None
    finally:
        QtWidgets.QApplication.restoreOverrideCursor()
        panel._restore_scene_joints(snapshot)

    # 规划器要留着：其内部记录了每条指令的时间区间，抓取动画靠它把
    # gripper() 指令映射到轨迹时刻
    panel._planner = planner
    panel._gripper_animator = GraspAnimator(program, planner)

    panel._report = report
    panel._show_report(report)
    # 路点列表着色：错误红/警告橙/正常绿 + 逐指令节拍（v0.2.4）
    panel._apply_point_colors(report)
    panel.reportReady.emit(report)

    # 有轨迹就能回放——即使校验没通过，也允许用户看看走到哪儿崩的
    if getattr(panel, "btn_play_all", None) is not None:
        panel.btn_play_all.setEnabled(bool(panel._playable()))
    message = tr("校验完成：{res}").format(
        res=tr("通过") if report.ok else tr("{n} 处错误").format(n=len(report.errors))
    )
    if panel._build_notes:
        # 圆弧降级这类"几何被换掉了"的事必须说出来，不能只在日志里躺着
        message += tr("；{n} 项运动降级（见日志）").format(n=len(panel._build_notes))
        for note in panel._build_notes:
            panel.statusMessage.emit(tr("⚠ {note}").format(note=note))
    panel.statusMessage.emit(message)
    return report


def validate_all(panel) -> Dict[str, Optional["ProgramReport"]]:
    """逐台校验**所有**已登记的机器人（一键校验全部）。

    装配与规划和 :meth:`validate_program` 完全一致，区别在于**不逐台刷界面**
    ——界面始终显示当前那台。否则问题列表会被一遍遍刷掉，用户只看见最后
    一台的结论，前面几台校出问题等于没看见。每台的结果（轨迹 / 报告 /
    夹爪时序）各存进自己的状态槽，随后「全部回放」直接复用这些轨迹。

    Returns:
        ``{机器人名: ProgramReport}``；没型号、没示教点或校验本身出错的那台
        对应 ``None``——"没结果"和"通过"是两回事，不混为一谈
    """
    reports: Dict[str, Optional["ProgramReport"]] = {}
    for name in list(panel._states):
        state = panel._states.get(name)
        if state is None or state.robot is None:
            continue
        with panel._using_robot(name):
            program = panel._build_program_impl()
            if program is None:
                reports[name] = None  # 这台还没教过程序
                continue
            snapshot = panel._scene_joint_snapshot(name)
            try:
                report = program.validate(scene=panel._scene)
                planner = TrajectoryPlanner(panel._robot, scene=panel._scene)
                state.trajectory = planner.plan(program)
            except Exception as exc:
                # 一台崩了不能带崩全部：如实记下来，继续校其余的
                state.trajectory = None
                state.report = None
                reports[name] = None
                panel.statusMessage.emit(
                    tr("{name} 校验异常：{kind}: {msg}").format(
                        name=name, kind=type(exc).__name__, msg=exc
                    )
                )
                continue
            finally:
                panel._restore_scene_joints(snapshot)
            state.planner = planner
            state.gripper_animator = GraspAnimator(program, planner)
            state.report = report
            state.point_issue_level = panel._point_levels(report)
            reports[name] = report

    # 当前这台若也校过了，顺手把它的结论刷进界面——用户正看着它
    active = panel._active
    if active in reports and reports[active] is not None:
        panel._show_report(reports[active])
        panel._apply_point_colors(reports[active])
    if getattr(panel, "btn_play_all", None) is not None:
        panel.btn_play_all.setEnabled(bool(panel._playable()))

    passed = [n for n, r in reports.items() if r is not None and r.ok]
    failed = [n for n, r in reports.items() if r is not None and not r.ok]
    skipped = [n for n, r in reports.items() if r is None]
    parts = []
    if passed:
        parts.append(tr("{n} 台通过").format(n=len(passed)))
    if failed:
        parts.append(tr("{n} 台有问题").format(n=len(failed)))
    if skipped:
        parts.append(tr("{n} 台没程序").format(n=len(skipped)))
    panel.statusMessage.emit(
        tr("校验全部：{res}").format(res="、".join(parts) or tr("没有可校验的机器人"))
    )
    panel.allValidated.emit(reports)
    return reports


def focus_report(panel, name: Optional[str] = None) -> bool:
    """切到 ``name`` 并把它**已存的**校验报告刷进界面。

    汇总对话框里点「看这台」走这里：报告已经算过了，不必重校一遍。

    Returns:
        有报告并已显示 ``True``；这台还没校过返回 ``False``
    """
    key = name or panel._active
    state = panel._state(key)
    if state is None or state.report is None:
        panel.statusMessage.emit(tr("{name} 还没有校验结果").format(name=key or tr("这台")))
        return False
    panel.set_active_robot(key)
    panel._show_report(state.report)
    panel._apply_point_colors(state.report)
    # 视口与示教点显示也要跟过去——否则"看的是 A 的报告，画的是 B"
    panel.activeRobotChanged.emit(key)
    return True


def _point_levels(panel, report: "ProgramReport") -> Dict[int, str]:
    """报告里每个路点的最严重问题级别——**只算，不刷界面**。

    映射规则同 :meth:`_apply_point_colors`，但不碰列表控件：一键校验全部
    时要逐台算完存进各自的状态槽，界面从头到尾只显示当前那台。
    """
    levels: Dict[int, str] = {}
    plan, _notes = panel.motion_plan()
    instruction_to_point = {order: index for order, (_kind, index, _via) in enumerate(plan)}
    for issue in report.issues:
        point_index = instruction_to_point.get(issue.motion_index, issue.motion_index)
        level = "error" if issue.severity == SEVERITY_ERROR else "warning"
        if levels.get(point_index) != "error":
            levels[point_index] = level
    return levels
