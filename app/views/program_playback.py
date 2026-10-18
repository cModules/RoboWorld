"""离线编程面板的回放控制：定时器推进、逐机发帧、夹爪状态广播。

多台机器人共用同一个 ``QTimer``，各自按自己的时间轴推进（锁步同播）——
多机协同回放能直观看出两台是否会抢同一个工位。面板内部角度一律为度，
发向视口的回放帧换算为弧度（``GLWidget.renderer.joint_values`` 用弧度）。
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, List, Optional, Sequence

from app.contracts.program import PlaybackFrame
from app.views.program_state import PLAYBACK_INTERVAL_MS
from support.i18n import tr

if TYPE_CHECKING:  # 仅类型标注
    from app.views.program_panel import ProgramPanel


def start_playback(panel, robots: Optional[Sequence[str]] = None) -> None:
    """从轨迹起点开始回放。

    Args:
        robots: 要回放的机器人名序列；``None`` 表示当前这台。
            多台时**各自按自己的时间轴**推进但共用同一个定时器——这就是
            多机协同回放：能直观看出两台是否会抢同一个工位
    """
    if isinstance(robots, bool) or isinstance(robots, str):
        # ``clicked(bool)`` 会把 False 灌进来（``list(False)`` 直接崩），
        # 单个名字的字符串也会被拆成一串字符——都不是"机器人名单"
        robots = [robots] if isinstance(robots, str) else None
    wanted = list(robots) if robots is not None else panel._playable()
    names = [n for n in wanted if panel._playable(n)]
    if not names:
        # 不用模态弹窗：无头/自动化场景下会永久阻塞，且"还没校验"只是
        # 普通状态提示，不值得打断用户
        panel.statusMessage.emit(tr("请先校验程序以生成轨迹，再回放"))
        return

    for name in names:
        state = panel._states[name]
        traj = state.trajectory
        state.playhead = float(traj.waypoints[0].timestamp)
        # 回放从头开始：夹爪状态复位为未夹持，主窗口会据此放下工件
        if state.gripper_animator is not None:
            state.gripper_animator.reset()
        panel._set_gripper_for(name, "")

    panel._playing = names
    panel.btn_stop.setEnabled(True)
    panel._timer.start()
    if len(names) > 1:
        panel.statusMessage.emit(tr("多机回放中…（{n} 台）").format(n=len(names)))
    else:
        panel.statusMessage.emit(tr("轨迹回放中…"))


def start_playback_all(panel) -> None:
    """所有已校验过的机器人**锁步同播**（多机回放）。"""
    if not panel._playable():
        panel.statusMessage.emit(tr("没有已生成轨迹的机器人——请先校验程序"))
        return
    panel.start_playback(panel._playable())


def _playable(panel, name: Optional[str] = None) -> List[str]:
    """可回放的机器人名——必须已校验出多于一个路点的轨迹。

    Args:
        name: 只看这台；``None`` 看全部
    """
    keys = [name] if name is not None else list(panel._states)
    out = []
    for key in keys:
        state = panel._states.get(key)
        if state is None:
            continue
        traj = state.trajectory
        if traj is not None and len(traj.waypoints) > 1 and state.joint_names:
            out.append(key)
    return out


def stop_playback(panel) -> None:
    """停止回放。"""
    was_running = panel._timer.isActive()
    panel._timer.stop()
    panel._playing = []
    panel.btn_stop.setEnabled(False)
    if was_running:
        panel.playbackFinished.emit()


def _on_tick(panel) -> None:
    """推进所有在播机器人的播放头并发射关节帧。"""
    if not getattr(panel, "_playing", None):
        panel.stop_playback()
        return
    # 到终点的机器人自己退出；全部到点后整体收尾
    panel._playing = [name for name in panel._playing if panel._tick_one(name)]
    if not panel._playing:
        panel.stop_playback()


def _tick_one(panel, name: str) -> bool:
    """推进 ``name`` 一帧并发给它自己的视口；未到终点返回 ``True``。"""
    state = panel._states.get(name)
    traj = state.trajectory if state is not None else None
    if traj is None or not state.joint_names:
        return False

    # 用定时器标称间隔推进（按速度百分比缩放）。定时器本身有抖动，
    # 但对"看一遍路径怎么走"这个目的足够，不值得引入实时时钟补偿的复杂度
    scale = 100.0 / max(panel.spin_speed.value(), 1)
    state.playhead += (PLAYBACK_INTERVAL_MS / 1000.0) / scale

    t_end = float(traj.waypoints[-1].timestamp)
    done = state.playhead >= t_end
    if done:
        # 终点处的事件（程序以 gripper 收尾）也要应用
        panel._emit_one(name, traj.waypoints[-1].joints)
        if state.gripper_animator is not None:
            panel._set_gripper_for(name, state.gripper_animator.state_at(t_end) or "")
        return False

    waypoint = traj.interpolate(state.playhead)
    panel._emit_one(name, waypoint.joints)
    # 越过夹爪事件时刻时通知主窗口吸附/释放工件
    if state.gripper_animator is not None:
        panel._set_gripper_for(name, state.gripper_animator.state_at(state.playhead) or "")
    return True


def _emit_one(panel, name: str, joints_deg: Sequence[float]) -> None:
    """把 ``name`` 这一帧的关节角（度）换算为弧度发给它自己的机器人。"""
    state = panel._states[name]
    values = {}
    for i, joint in enumerate(state.joint_names):
        if i < len(joints_deg):
            values[joint] = math.radians(float(joints_deg[i]))
    frame = PlaybackFrame(robot_name=name, joints_rad=values)
    panel.playbackFrame.emit(frame.robot_name, dict(frame.joints_rad))
    panel.playbackFrameDTO.emit(frame)


def _emit_frame(panel, joints_deg: Sequence[float]) -> None:
    """兼容旧调用面：给**当前**机器人发一帧。"""
    if panel._active is not None and panel._active in panel._states:
        panel._emit_one(panel._active, joints_deg)


def _set_gripper_for(panel, name: str, action: str) -> None:
    """广播 ``name`` 这台机器人的夹爪状态（去重，避免每帧刷屏）。"""
    state = panel._states.get(name)
    if state is None:
        return
    if action != state.last_gripper_state:
        state.last_gripper_state = action
        panel.gripperStateChanged.emit(name, action)


def _set_gripper_state(panel, action: str) -> None:
    """兼容旧调用面：给**当前**机器人设夹爪状态。"""
    if panel._active is not None:
        panel._set_gripper_for(panel._active, action)
