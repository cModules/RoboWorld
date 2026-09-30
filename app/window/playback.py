"""窗口级「回放与交互」操作（ADR-002 拆解第 6 步）。

从 ``main.py`` 的 ``MainWindow`` 原样搬出（方法体未改一字）。这一组是**把程序/轨迹变成
机器人的运动**，以及随之而来的工件吸附、IO 状态与视口交互：

* 回放：``on_playback_requested`` → ``on_playback_frame`` → ``on_playback_stopped``，
  加上"归属机器人已不在场景"时的 ``_halt_playback``；
* 工艺语义：``_apply_pick_event`` / ``on_gripper_state_changed``（播放到夹爪指令时吸附、
  释放工件）与 ``_update_io_status``（常驻状态栏显示夹爪在干嘛）；
* 关节与视口交互：``on_joint_changed`` / ``on_joint_changed_for_record`` /
  ``_sync_scene_joints`` / ``on_path_point_move_requested`` / ``_highlight_point_line``。

多机的贯穿线索都在这一组：每个入口都必须**指名道姓**说清是哪台机器人，否则会出现
"拖 A 的滑块让 B 转起来"、"两台抢同一件工件"、"给 B 记点把 A 的路径抹掉"这三类串扰
（各自的注释里都写了原因，搬移时一字未动）。

与 ``main.py`` 版本的**唯一**差别：依赖探测在本模块内独立完成、**静默降级**
（main.py 已就同一次缺失打过 ``[warning]``）。``_HAS_WORLD`` / ``_HAS_PICKPLACE``
是本模块自己探测的真标志——它们参与"环境建模/抓取工艺可不可用"的判断。
"""

from __future__ import annotations

import math
from typing import Optional

# 可选依赖由 ``app.window.capabilities`` 单点探测（C2：不再各留一份 try/except）：
# ROBOT_NAME（场景主机器人名，作为兜底目标）与 PointType（示教点语义 PICK / PLACE …）
from app.logging import logger
from app.window.capabilities import (
    PointType,
    ROBOT_NAME,
    _HAS_PICKPLACE,
    _HAS_WORLD,
    tr,
)


class PlaybackMixin:
    """回放、工件吸附、IO 状态与视口交互（宿主：``MainWindow``）。"""

    def _highlight_point_line(self, index: int, robot: Optional[str] = None) -> None:
        """把代码编辑器的光标/选区落到第 ``index`` 个点对应的那一行。

        3D 里点了哪个点、回放走到哪一点，代码就停在那一行——逐点核对时不
        必再数行号。查不到对应行就什么都不做（手打的代码行号本就可能偏）。
        """
        if not hasattr(self, "code_editor"):
            return
        line = self.code_editor.line_of_point(index, robot)
        if line is not None:
            self.code_editor.highlight_line(line)

    def on_path_point_move_requested(self, robot_name: str, index: int) -> None:
        """视口右键「移动到」：把该台机器人驱动到示教点 ``index`` 的位形。

        Args:
            robot_name: 点属于哪台（画出来的那一段说了算，不是"当前那台"）
            index: 那台点表里的下标（0 起，界面上显示为 P{index+1}）
        """
        if not robot_name:
            return
        pts = self.gl_widget.get_recorded_positions(robot_name)
        if not (0 <= index < len(pts)):
            return
        self.gl_widget.set_current_robot(robot_name)
        self.gl_widget.set_joint_values(pts[index], robot_name)
        # 必须同步到场景：只改渲染器的话法兰不动，挂在法兰上的夹爪/工件
        # 会留在原地——看着就是"机器人走了、工具没跟"
        self._sync_scene_joints(robot_name)
        # 面板只跟着"正在被查看的那台"刷新，避免把 A 的帧递给 B 的滑块
        if hasattr(self, "property_panel") and (self.property_panel.current_object == robot_name):
            self.property_panel.update_joint_displays(pts[index])
        self.gl_widget.update()
        self.statusBar().showMessage(
            tr("已把 {robot} 移动到示教点 P{n}").format(robot=robot_name, n=index + 1)
        )
        # 3D 里点了哪个点，代码就停在哪一行——便于对着这一行改数值
        self._highlight_point_line(index, robot_name)

    def on_playback_frame_dto(self, frame) -> None:
        """接收强类型回放 DTO，作为新的窗口内部协议。"""
        self.on_playback_frame(frame.robot_name, frame.joints_rad)

    def on_playback_frame(self, robot_name: str, joint_values: dict) -> None:
        """按插补后的密集轨迹驱动指定机器人，而不是在示教点之间瞬移。"""
        robot = robot_name or getattr(self.gl_widget, "current_robot", None)
        self.gl_widget.set_joint_values(joint_values, robot)
        # 面板只跟着"正在被查看的那台"刷新，避免递给 A 的帧去拉动 B 的滑块
        if hasattr(self, "property_panel") and (self.property_panel.current_object == robot):
            self.property_panel.update_joint_displays(joint_values)
        self._sync_scene_joints(robot)
        self.gl_widget.update()

    def on_joint_changed(self, robot_name, name, value):
        """关节滑块 → 视口：值写进**发信号那台**的渲染器，而不是"当前"那台。

        Args:
            robot_name: 滑块所属机器人名（属性面板的 :attr:`_active_robot`）；
                空串表示老调用方——退回当前机器人
            name: 关节名
            value: 弧度

        Note:
            多机的关键就在这一个参数：不带名字时只能写 ``gl_widget.renderer``，
            而"当前渲染器"未必是"滑块所属那台"（选中与加载顺序会错位），于是
            拖 A 的滑块让 B 转起来——这就是"两台一起动"。
        """
        renderer = self.gl_widget.renderer_of(robot_name) if robot_name else self.gl_widget.renderer
        renderer.joint_values[name] = value
        self._sync_scene_joints(robot_name or None)
        self.gl_widget.update()

    def _sync_scene_joints(self, robot: Optional[str] = None) -> None:
        """把某台机器人的渲染关节同步给场景，让干涉检查/法兰反映实时位形。

        Args:
            robot: 目标机器人名；``None`` 取当前机器人

        Note:
            必须指名道姓：``set_robot_joints`` 默认只作用于主机器人，多机时会把
            "拖 A 的关节"写到 B 身上——表现为两台一起动、或挂在 A 上的工具离开法兰。
        """
        if not (hasattr(self, "scene_panel")):
            return
        scene = self.scene_panel.scene
        if not scene.has_robot:
            return
        target = robot or getattr(self.gl_widget, "current_robot", None) or ROBOT_NAME
        if target not in scene.robots:
            target = scene.robot_names[0]
        renderer = self.gl_widget.renderer_of(target)
        # 必须用 actuated_names() 而非 joints.keys()：后者含 fixed 关节
        # （KR16 是 7 个 joints / 6 个可动），多传一个会让 FK 直接抛错
        names = list(renderer.actuated_names())
        if not names:
            # 这台模型不可用（退化渲染器）：写空列表会把场景里已有的关节角清空，
            # 让法兰/干涉检查掉回零位——宁可不同步，也不要把好数据抹成坏数据。
            return
        self.scene_panel.update_robot_joints(
            [math.degrees(renderer.joint_values.get(n, 0.0)) for n in names],
            name=target,
        )

    def on_joint_changed_for_record(self, robot_name, name, value):
        """同 :meth:`on_joint_changed`，并顺带把位形记成示教点。"""
        renderer = self.gl_widget.renderer_of(robot_name) if robot_name else self.gl_widget.renderer
        renderer.joint_values[name] = value
        self.gl_widget.update()
        if self.code_editor.record_btn.isChecked():
            # 记到**发信号那台**名下：多机时不指定就会把 B 的位形记成 A 的点
            key = robot_name or self.gl_widget.current_robot
            count = self.gl_widget.record_position(key)
            # 同步到 PathCodeEditor：只换**这台**那一段，其余机器人的路径留在
            # 表里。整表替换的话，给 B 记第一个点就把 A 的路径全抹了
            self.code_editor.set_recorded_positions(self.gl_widget.get_recorded_positions(key), key)
            self._sync_program_points(key)

    def _halt_playback(self, robot: Optional[str], index: int) -> None:
        """归属机器人已不在场景里：立刻停表，并把是哪一行说清楚。"""
        self.code_editor._stop_playback()
        self.code_editor.playbackStopped.emit()
        line = self.code_editor.line_of(index)
        self.statusBar().showMessage(
            tr("第 {line} 行：机器人“{robot}”已不在场景中，已停止运行").format(
                line=line, robot=robot or tr("（未指定）")
            ),
            8000,
        )
        self.code_editor.highlight_line(line)

    def on_playback_requested(self, index, robot=None):
        positions = self.code_editor.get_recorded_positions()
        if 0 <= index < len(positions):
            # 播的是**这条指令归属的那台**：全局运行时一行一个主人，只认
            # 整份归属或"当前选中"都会把这台的程序拽到别台身上
            target = (
                robot
                or self.code_editor.owner_robot
                or getattr(self.gl_widget, "current_robot", None)
            )
            # 归属那台已不在场景里（工程载入后被删掉）：停下来报错。硬跑下去只是
            # 画面不动却显示"运行中"，比当场报错难排查得多
            if not self._robot_usable(target):
                self._halt_playback(target, index)
                return
            self.gl_widget.set_joint_values(positions[index], target)
            # 走到哪一点，代码就高亮到哪一行。用**已有**的 _line_map：回放期间
            # 重新解析会改写点表，回放序列与点表错位会让机器人跳点
            if hasattr(self, "code_editor"):
                line = self.code_editor.line_of(index)
                if line:
                    self.code_editor.highlight_line(line)
            # 面板只跟着"正在被查看的那台"刷新，避免递给 A 的帧去拉动 B 的滑块
            if hasattr(self, "property_panel") and (self.property_panel.current_object == target):
                self.property_panel.update_joint_displays(positions[index])
            # 必须同步到场景：视口渲染器与工作单元场景是两套状态。只更新渲染器
            # 的话场景机器人永远停在初始位形，法兰不动，挂在法兰上的夹爪与
            # 被吸附的工件都不会跟着走（干涉检查也会按错误位形算）。
            self._sync_scene_joints(target)
            self._apply_pick_event(index, target)
            self._update_io_status(index, target)
            self.gl_widget.update()

    def _update_io_status(self, index: int, robot: Optional[str] = None) -> None:
        """常驻状态栏显示当前工位的 IO 动作——演示时能一眼看到夹爪在干嘛。

        Args:
            robot: 这条动作属于哪台机器人；``None`` 取当前那台。多机时 A 的夹紧
                不能显示成 B 的工件
        """
        if not hasattr(self, "status_io"):
            return
        action = tr("保持")
        if _HAS_PICKPLACE:
            # 按**总表索引**取语义：播放是按代码行序走的点序列，而
            # get_point_types() 给的是"当前编辑那台"的一段，索引对不上
            t = self.code_editor.point_type_at(index)
            action = {PointType.PICK: tr("夹紧"), PointType.PLACE: tr("松开")}.get(t, tr("保持"))
        wp = self._workpiece_for(robot)
        self.status_io.setText(f"IO {action}" + (f" · {wp}" if wp else ""))

    def _apply_pick_event(self, index: int, robot: Optional[str] = None) -> None:
        """播放到某个示教点时，按工艺语义挂载/释放工件。

        工件被「吸附」的实现就是把它挂到机器人下——复用与夹爪挂载完全相同的
        父子机制，于是它自动跟随法兰运动。释放即解除挂载，并把当前世界位姿
        写回局部 pose，工件原地留下。

        Args:
            robot: 这一帧属于哪台机器人；``None`` 取当前那台。逐点回放时传
                **点表归属**，否则会拿 A 的抓取去挂 B 手里的工件

        Note:
            挂给**谁**必须取当前机器人。写死主机器人 ``ROBOT_NAME`` 时，多机
            场景下 B 的抓取会把工件挂到 A 的法兰上——工件横穿整个工位飞过去。
        """
        if not _HAS_PICKPLACE or not _HAS_WORLD:
            return
        scene = getattr(self, "scene", None) or getattr(
            getattr(self, "scene_panel", None), "scene", None
        )
        if scene is None:
            return

        # 总表索引取语义：点表是多台混排的，按"当前那台的一段"取值会错位
        t = self.code_editor.point_type_at(index)
        if t is None:
            return
        workpiece = self._workpiece_for(robot)
        if not workpiece or workpiece not in scene.objects:
            return
        current = robot or self.gl_widget.current_robot
        holder = (
            current
            if current in scene.robots
            else (scene.robot_names[0] if scene.robot_names else ROBOT_NAME)
        )

        try:
            if t is PointType.PICK:
                scene.attach(workpiece, holder, keep_world=True)
            elif t is PointType.PLACE:
                # 只释放**自己**夹着的：别的机器人手里的工件不能被这台摘掉
                if scene.objects[workpiece].parent != holder:
                    return
                scene.detach(workpiece)
            else:
                return
        except (KeyError, ValueError) as exc:
            logger.warning("workpiece_gripper_action_failed: %s", exc)
            return
        if hasattr(self, "scene_panel"):
            self.scene_panel._refresh_tree()

    def on_gripper_state_changed(self, robot_name: str, action: str) -> None:
        """程序级回放到夹爪指令时，吸附/释放工件（S3 抓取动画）。

        Args:
            robot_name: 这条夹爪指令属于哪台机器人——多机回放时 A 夹紧不能把
                工件挂到 B 的法兰上
            action: ``"close"`` / ``"open"`` / ``""``

        与示教点回放（:meth:`_apply_pick_event`）共用同一机制：把工件
        ``attach`` 到机器人下并保持世界位姿——被夹住的瞬间停在原地，之后
        随法兰/TCP 随动；``detach`` 即原地释放。``action`` 为空串表示回放
        从头开始，此时把残留挂载的工件复位。
        """
        if not _HAS_WORLD:
            return
        scene = getattr(self, "scene", None) or getattr(
            getattr(self, "scene_panel", None), "scene", None
        )
        if scene is None:
            return
        # 抓的是**这台机器人**的工件，而不是"下拉里当前显示的那件"：多机回放
        # 时 A/B 会交替发夹爪事件，共用一份选择就会互相抢工件
        workpiece = self._workpiece_for(robot_name)
        if not workpiece or workpiece not in scene.objects:
            return
        obj = scene.objects.get(workpiece)
        if obj is None:
            return
        # 夹这件东西的那双手：多机时必须是指定的那台，不能再写死主机器人
        holder = robot_name or (scene.robot_names[0] if scene.robot_names else ROBOT_NAME)
        try:
            if action == "close":
                if obj.parent != holder:
                    scene.attach(workpiece, holder, keep_world=True)
            elif obj.parent == holder:  # "open" 或回放复位（""）
                scene.detach(workpiece)
            else:
                return  # 状态未变，不必刷新
        except (KeyError, ValueError) as exc:
            logger.warning("workpiece_gripper_action_failed: %s", exc)
            return
        if hasattr(self, "scene_panel"):
            self.scene_panel._refresh_tree()
        if hasattr(self, "status_io"):
            label = {"close": tr("夹紧"), "open": tr("松开")}.get(action, tr("保持"))
            self.status_io.setText(f"IO {label} · {workpiece}")
        # 视口按场景世界位姿重画——工件随动就体现在这一帧帧的刷新里
        self.gl_widget.update()

    def on_playback_stopped(self):
        self.statusBar().showMessage("Playback finished")
