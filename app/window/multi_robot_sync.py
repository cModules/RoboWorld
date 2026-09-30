"""窗口级多机数据同步（ADR-002 拆解第 7 步）。

本模块承接场景机器人、离线编程面板、路径编辑器与视口之间的同步边界。核心纪律是：
**所有状态都按机器人名归属，禁止用当前选中项覆盖另一台机器人的数据**。

方法体从 ``main.py`` 原样搬出；只把依赖探测复制到本模块，避免依赖入口脚本的模块级状态。
缺少可选模块时维持与宿主一致的安全降级：同步入口直接返回，不阻塞窗口启动。
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

# 可选依赖由 ``app.window.capabilities`` 单点探测（C2：不再各留一份 try/except）
from app.logging import logger
from app.window.capabilities import (
    _HAS_PICKPLACE,
    _HAS_PROGRAM,
    _HAS_WORLD,
    _REPO_ROOT,
    tr,
)


class MultiRobotSyncMixin:
    """多机器人场景、编程面板、代码编辑器与视口的同步行为。"""

    def _register_robot_for_programming(self, name: str, urdf: str) -> bool:
        """把场景里**已有**的 ``name`` 登记进离线编程面板（**不碰场景**）。

        工程载入时机器人已经在场景里了，此时不能走 :meth:`load_robot`——它会调
        ``_register_robot_in_scene`` 再加一台，固定名 ``ROBOT_NAME`` 还会把已有
        那台顶掉。这里只做"登记"，不改动场景。

        Returns:
            登记成功 ``True``；模型不可用或未启用编程模块返回 ``False``
        """
        if not (hasattr(self, "program_panel")):
            return False
        if not urdf or not Path(urdf).exists():
            return False
        try:
            from domain.robot import Robot as CoreRobot

            robot = CoreRobot(str(urdf), Path(urdf).stem)
        except Exception as exc:  # pragma: no cover - 模型损坏时不能带崩载入
            logger.warning("prepare_robot_kinematics_failed: %s: %s", name, exc)
            return False
        # 渲染器按名懒加载：取不到会给它一台专属空渲染器，绝不借用别人的
        renderer = self.gl_widget.renderer_of(name)
        joint_names = (
            list(renderer.joints.keys())
            if renderer is not None and getattr(renderer, "joints", None)
            else list(robot.joint_names)
        )
        self.program_panel.set_robot(robot, joint_names, name=name)
        return True

    def _restore_robot_programs(self, scene) -> None:
        """工程载入：把场景里每台机器人登记进编程面板。

        只登记主机器人是不够的——多机工程里第二台若没登记，它的抓取程序就
        "没有主人"，既不能校验也不能出码。
        """
        if not (hasattr(self, "program_panel")):
            return
        for name in scene.robot_names:
            state = scene.robots.get(name)
            urdf = getattr(state, "urdf", None) if state is not None else None
            if urdf and not Path(urdf).exists():
                alt = _REPO_ROOT / urdf
                if alt.exists():
                    urdf = str(alt)
            self._register_robot_for_programming(name, urdf or "")

    def _sync_program_points(self, robot: Optional[str] = None) -> None:
        """把某台机器人（缺省当前）的示教点同步给离线编程面板。

        工艺语义（抓取/放置点）一并传过去：程序面板据此在对应位置插入
        ``gripper()`` 指令，回放动画与出码共用同一份抓取时序（S3）。

        Args:
            robot: 机器人名；``None`` 取当前机器人。给**当前**这台同步时沿用
                路径代码编辑器的数据（轨迹编辑器改点后先回写到它那里），其余
                机器读视口里各自那一串——这样给 A 示教的点不会算进 B 的程序。
        """
        if not (hasattr(self, "program_panel")):
            return
        current = getattr(self.gl_widget, "current_robot", None)
        key = robot or current
        if key == current:
            # 只给这台自己那一段：点表里可能还装着别的机器人的路径，全拿过去
            # 会让 A 的程序里混进 B 的点
            positions, types = self._editor_points_for(key)
            # 顺手归档本机语义：路径编辑器那份随时会被别的机器人顶掉，只在
            # "切换时"补记的话，用户给 A 示教完就直接「导出全部」，A 名下还是空的
            if types and len(types) == len(positions):
                self._point_types_by_robot[key] = list(types)
        else:
            # 非当前那台：读它自己那份语义（切换时存进 _point_types_by_robot 的）
            positions = self.gl_widget.get_recorded_positions(key)
            types = (
                list(self._point_types_by_robot.get(key) or []) if _HAS_PICKPLACE else None
            ) or None
        if types is not None and len(types) != len(positions):
            types = None
        self.program_panel.set_taught_points(positions, types, robot=key)

    def _editor_points_for(self, key: Optional[str]):
        """路径编辑器里属于 ``key`` 的点与语义。

        旧工程 / 单机器人那份点表没有归属标记（``point_robots`` 全是 ``None``），
        此时整份都算当前这台的——与改归属之前的行为一致。
        """
        ed = self.code_editor
        if key and key in ed.point_robots:
            return ed.points_of(key), (ed.types_of(key) if _HAS_PICKPLACE else None)
        if any(r is not None for r in ed.point_robots):
            return [], ([] if _HAS_PICKPLACE else None)  # 已分段，这台名下没有点
        return list(ed.recorded_positions), (list(ed.point_types) if _HAS_PICKPLACE else None)

    def _sync_viewport_paths_from_editor(self) -> None:
        """把路径编辑器的点表**按归属**回写视口各机器人的路径桶。

        单机时代的写法是"整份灌进当前机器人"（``set_recorded_positions`` 不指名
        就写 ``current_robot``）。多机下它就是一台复制机：加载新机器人时
        ``current_robot`` 已被切成新名字，于是**别人**教好的点被整份写进新机器
        人名下、画在它旁边；等原主人被删掉，那份副本既不在代码里也回放不了，
        变成屏幕上的幽灵路径。

        这里按每条指令的归属（``robot_at``）分桶回写；没有归属标记的旧点表
        整份退回**当前**那台，与旧行为一致。
        """
        positions = self.code_editor.get_recorded_positions()
        if not positions:
            return
        by_robot: dict = {}
        for i, pos in enumerate(positions):
            key = self.code_editor.robot_at(i) or self.gl_widget.current_robot
            by_robot.setdefault(key, []).append(pos)
        for key, pts in by_robot.items():
            self.gl_widget.set_recorded_positions(pts, key)
        # 语义同样按段归档，否则导出程序的夹爪时序会串到别的机器人上
        for key in by_robot:
            self._point_types_by_robot[key] = self.code_editor.types_of(key)
        self.gl_widget.update()

    def _on_robot_removed(self, name: str) -> None:
        """机器人被移出场景：把属于它的**全部**派生状态清掉。

        只从场景里删对象会留下一堆孤儿状态：离线编程面板还挂着这台的程序
        （照样能校验、能出码，却已经没有对应的机器人）、视口还留着它的渲染器
        与示教点、路径可能还在画。这里一并收回。

        Args:
            name: 被移除的机器人名
        """
        if not name:
            return
        if hasattr(self, "program_panel"):
            self.program_panel.remove_robot(name)

        # 视口：渲染器与示教点（路径可视化也会随之消失，见 _path_robots）
        self.gl_widget.renderers.pop(name, None)
        self.gl_widget._empty_renderers.pop(name, None)
        self.gl_widget.clear_recorded_positions(name)
        self._point_types_by_robot.pop(name, None)
        self._workpiece_by_robot.pop(name, None)

        # 路径编辑器：只删**它那一段**，其余机器人的段原样留着。
        #
        # 以前是"被删的那台正好是当前台才整表清空"：既漏掉了它**不是**当前台的
        # 情况（段留在代码里，回放能跑到一台已经不存在的机器人），又会在它是
        # 当前台时把别人教好的路径一起抹掉——视口还画着（分桶没动）、代码与
        # 回放里却没了，正是那条形同幽灵的路径。
        self.code_editor.set_recorded_positions([], name)
        if getattr(self.code_editor, "owner_robot", None) == name:
            # 没有段头的旧点表靠它找归属；指着一个已删的名字会让后续播放直接
            # 判"机器人已不在场景"而停表
            self.code_editor.owner_robot = None

        if self.gl_widget.current_robot == name:
            # 编辑视角还停在被删那台上：挪到剩下的一台
            remaining = []
            if hasattr(self, "scene_panel"):
                remaining = self.scene_panel.scene.robot_names
            if remaining:
                self.gl_widget.set_current_robot(remaining[0])

        self.gl_widget.update()

    def _workpiece_for(self, robot: Optional[str] = None) -> Optional[str]:
        """``robot``（缺省当前）这台机器人抓的工件；没指定过则退回下拉当前值。

        工件下拉是全局一份，两台机器人各抓各的。回放/吸附时**必须**按机器人
        解析，不能直接用"下拉里当前显示的那件"——否则 A 的夹紧事件会把 B 选中
        的工件抓起来，看起来就是"两台抢同一件"。
        """
        key = robot or getattr(self.gl_widget, "current_robot", None)
        if key is not None and key in self._workpiece_by_robot:
            return self._workpiece_by_robot.get(key)
        return self.code_editor.get_workpiece() if _HAS_PICKPLACE else None

    def _on_workpiece_combo_changed(self, _index: int) -> None:
        """工件下拉换了件：记到**当前机器人**名下。

        切换机器人时也会走这里（下拉被回填成那台的工件），此时写回的是同一个
        值——幂等，不会把刚取出来的覆盖成错的。
        """
        if not _HAS_PICKPLACE or not hasattr(self.code_editor, "workpiece_combo"):
            return
        key = getattr(self.gl_widget, "current_robot", None)
        if not key:
            return
        self._workpiece_by_robot[key] = self.code_editor.workpiece_combo.currentData()

    def _on_program_robot_changed(self, name: str) -> None:
        """离线编程面板切换了机器人：视口与示教点显示都跟到那台上去。

        不同步就会出现"在 A 名下编辑、视口却跟着 B 动"——下拉只是换了数据槽，
        视口的当前机器人、路径编辑器里显示的点位都得一起换。
        """
        if not name:
            return
        # 切走之前先把**原来那台**的抓取语义存起来——路径编辑器的语义是按段
        # 存的，不归档就会被新那台的段顶掉
        prev = self.gl_widget.current_robot
        if prev and prev != name:
            self._point_types_by_robot[prev] = list(self.code_editor.get_point_types(prev) or [])
            self._workpiece_by_robot[prev] = self.code_editor.get_workpiece()
        self.gl_widget.set_current_robot(name)
        if hasattr(self, "scene_panel") and name in self.scene_panel.scene.objects:
            self.gl_widget.selected_object = name
            if hasattr(self, "property_panel"):
                self.property_panel.set_object(name)

        # 换成这台自己的示教点**与抓取语义**：只替它那一段，别的机器人的
        # 路径留在表里（都显示在代码预览里，各带自己的 Robot 头）
        positions = self.gl_widget.get_recorded_positions(name)
        self.code_editor.set_recorded_positions(positions, name)
        saved = self._point_types_by_robot.get(name)
        if saved is not None:
            self.code_editor.set_types_for(name, saved)
        # 抓取对象同样换成本台的（存过才动，没存过沿用当前选择）
        if name in self._workpiece_by_robot:
            combo = getattr(self.code_editor, "workpiece_combo", None)
            if combo is not None:
                idx = combo.findData(self._workpiece_by_robot[name])
                if idx >= 0:
                    combo.setCurrentIndex(idx)
        # 长度要与示教点对齐，多退少补（缺的按默认语义补）
        self.code_editor.sync_point_types()
        self.code_editor.pos_count_label.setText(tr("示教点：{n}").format(n=len(positions)))
        self.code_editor._update_code_with_positions()
        self._sync_program_points(name)
        self.gl_widget.update()

    def on_taught_points_changed(self, positions, types, motions) -> None:
        """轨迹编辑器改动示教点后回写路径代码编辑器（v0.2.4）。

        两条编辑通路共用同一份示教点数据：面板里增删改了路点，左侧
        代码预览的 movej 文本与点位计数必须同步，否则下次 Play 会用旧数据。

        **必须按机器人名回灌**：整机替换（``owner=None``）会把点表里的
        ``# Robot:`` 归属标记抹成 ``None``——其余机器人的点当场从表里蒸发，
        代码里再也看不见他们的 movej。这也是"删了/移了别人的点"的源头。
        """
        if not hasattr(self, "code_editor"):
            return
        robot = getattr(self.program_panel, "active_robot", None)
        positions = list(positions)
        self.code_editor.set_recorded_positions(positions, robot)
        if _HAS_PICKPLACE and types is not None:
            # 语义跟着新行序回填：移动过的点带着自己的"抓取/放置"一起走
            for k, i in enumerate(self.code_editor.indices_of(robot)):
                if k < len(types) and i < len(self.code_editor.point_types):
                    self.code_editor.point_types[i] = types[k]
        self.code_editor.pos_count_label.setText(
            tr("示教点：{n}").format(n=len(self.code_editor.recorded_positions))
        )
        self.code_editor._update_code_with_positions()
        # 视口路径同步**这一台**那一段：否则 3D 里画的还是移动前的折线顺序
        self.gl_widget.set_recorded_positions(positions, robot)
        self.gl_widget.update()

    def _on_program_sync_requested(self, name: str) -> None:
        """离线编程面板要动数据了：按名字把最新示教点回灌给它。

        面板只在"记录点 / 切换机器人"这类时刻被同步过，它的槽位可能是旧快照
        （例如先给 A 教完点、再把下拉切到自己还没教点的 C）。此时点「校验程序」
        会用空数据校一遍，看起来就像按钮没反应。这里补一次回灌，让点面板自己
        的按钮和按 F7 走同一份数据。
        """
        if not (hasattr(self, "program_panel")):
            return
        if name and name not in self.program_panel.robot_names:
            return
        if name:
            state = self.program_panel._state(name)
            if state is not None and state.taught:
                # 面板那台已经有内容——**绝不**用别处的版本覆盖它。示教点有两条
                # 通路（面板直接 set_taught_points、以及主窗口回灌），合并规则说
                # 不清时宁可不动：覆盖掉用户刚教的点比"少同步一次"严重得多
                return
        self._sync_program_points(name or None)
