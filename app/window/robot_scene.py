"""窗口级「机器人 / 场景加载」操作（ADR-002 拆解第 5 步）。

从 ``main.py`` 的 ``MainWindow`` 原样搬出（方法体未改一字）。这一组是**把东西装进来**：
载入 URDF / 预置工作站 / 抓取工作站演示、机器人库的展示与重扫、拖拽落点、场景接管、
机器人登记进场景与编程面板。共同点是"动作触发一次、之后由别的组去同步"，所以能整组搬。

与 ``main.py`` 版本的**有意差异（四处，逐条写下来免得被后人当成漏搬）**：

1. 依赖探测在本模块内独立完成、**静默降级**（main.py 已就同一次缺失打过 ``[warning]``）；
2. ``_HAS_PROGRAM`` 的用法一律简化为 ``hasattr(self, "program_panel")``——宿主窗口只在
   编程面板可用时才创建它、从不置 ``None``，故 ``hasattr`` 已蕴含该标志。
   ``_HAS_WORLD`` / ``_HAS_ROBOT_LIBRARY`` / ``_HAS_TEMPLATES`` / ``_HAS_PICKPLACE``
   则是本模块自己探测的真标志（它们参与"多个模块齐不齐"的联合判断）；
3. ``_REPO_ROOT`` 取自 ``domain.world.templates.REPO_ROOT``——**与本组的依赖同源**，
   同值且更稳（那里处理了 PyInstaller 的 ``_MEIPASS``）。main.py 自己那份 ``_REPO_ROOT``
   保留不动：工程/会话组还在用；
4. **``load_preset_cell`` 的预置场景目录必须改**：原来是
   ``Path(__file__).resolve().parent / "assets" / "cells"``，搬进 ``app/window/`` 后
   ``__file__`` 指向本包目录，照抄会**静默地**去 ``app/window/assets/cells`` 找——
   一条预置场景都找不到，界面只会说"未找到预置场景"，不报错。已改为 ``_REPO_ROOT``。
   ``DEFAULT_ROBOT_KEYS`` 也随本组整体搬来（全仓只有本组用它）。
"""

from __future__ import annotations

import math
from pathlib import Path

from PySide6 import QtWidgets

from app.logging import logger
from app.window.capabilities import (
    Pose,
    ROBOT_NAME,
    Scene,
    _HAS_PICKPLACE,
    _HAS_ROBOT_LIBRARY,
    _HAS_TEMPLATES,
    _HAS_WORLD,
    _REPO_ROOT,
    build_grasping_workstation,
    demo_taught_points,
    tr,
)

from support.i18n import tr

# 环境建模（场景 / 位姿）：本组要的 ROBOT_NAME / Scene / Pose
# 机器人库（组件库面板）：本组只用"可不可用"这一个判断
# 工作单元模板 + 一键演示示教点；REPO_ROOT 与它们同源（见模块头差异 ③）
# 抓取工艺：本组只用"可不可用"这一个判断（符号由抓放组使用）
#: 首屏默认载入的示例机器人（按顺序回退）。2026-09-27 随本组从 main.py 整体搬来
#: ——全仓只有 ``_load_default_robot`` 用它。
DEFAULT_ROBOT_KEYS = ("kuka/kr16_2", "kuka/kr6_r900", "ur/ur5e")


class RobotSceneMixin:
    """机器人 / 场景的加载与登记（宿主：``MainWindow``）。"""

    def _load_default_robot(self):
        """未通过命令行指定模型时，载入一个示例机器人，避免首屏空白。"""
        for key in DEFAULT_ROBOT_KEYS:
            entry = self.library_panel.catalog.get(key)
            if entry and entry.urdf_path and entry.urdf_path.exists():
                self.load_robot(str(entry.urdf_path))
                self.library_panel.select_key(key)
                return

    def show_robot_library(self):
        """切换到机器人库标签页。"""
        # 资源库常驻左侧，无需切换标签页
        if hasattr(self, "library_panel"):
            self.library_panel.setFocus()

    def reload_robot_library(self):
        """重新扫描 assets/models/ 目录。"""
        if hasattr(self, "library_panel"):
            self.library_panel.refresh()

    def _add_component_from_library(self, component_id: str) -> None:
        """机器组件库双击/拖拽组件时，加入场景并切到场景标签。"""
        if not (hasattr(self, "scene_panel")):
            return
        self.scene_panel.add_component(component_id)
        self._focus_tab(self.scene_panel)

    def _on_robot_dropped(self, key: str) -> None:
        """机器组件库拖拽机器人型号到视口时，按目录 key 解析 URDF 并载入。"""
        if not (hasattr(self, "library_panel")):
            return
        entry = self.library_panel.catalog.get(key)
        if entry and entry.urdf_path and entry.urdf_path.exists():
            self.load_robot(str(entry.urdf_path))
        else:
            self.statusBar().showMessage(tr("该型号 URDF 不可用"))

    def _adopt_scene(self, scene) -> None:
        """接管一个新场景，把各处引用一次性换掉。

        场景对象是**按引用共享**的：视口、属性面板、工件下拉各存了一份。
        替换场景时三处必须一起换，否则会出现"树里变了、视口没变"这类
        极难排查的不一致。
        """
        self.scene_panel.set_scene(scene)
        self.gl_widget.scene = scene
        self.code_editor.set_scene(scene)
        if hasattr(self, "property_panel"):
            self.property_panel.set_scene(scene)
        self.scene_panel._refresh_tree()
        self.scene_panel.sceneChanged.emit()
        self.gl_widget.update()

    def load_preset_cell(self) -> None:
        """一键载入预置工作单元场景（Demo 起手式）。

        预置场景放在 ``assets/cells/``：载入后机器人、夹爪（已挂法兰）、
        工作台、工件、输送线、围栏、控制柜一次到位，可以直接开始示教。
        """
        if not _HAS_WORLD or not hasattr(self, "scene_panel"):
            QtWidgets.QMessageBox.information(self, tr("预置工作站"), tr("环境建模模块不可用。"))
            return

        # 搬迁注意：原写法是 Path(__file__).resolve().parent —— 搬进 app/window/
        # 后 __file__ 指向本包目录，会静默去 app/window/assets/cells 找预置场景。
        cells_dir = _REPO_ROOT / "assets" / "cells"
        files = sorted(cells_dir.glob("*.json")) if cells_dir.is_dir() else []
        if not files:
            QtWidgets.QMessageBox.information(
                self,
                tr("预置工作站"),
                tr("未找到预置场景（{cells} 下没有 .json）").format(cells=cells_dir),
            )
            return

        if len(files) == 1:
            target = files[0]
        else:
            names = [f.stem for f in files]
            choice, ok = QtWidgets.QInputDialog.getItem(
                self, tr("载入预置工作站"), tr("选择工作站:"), names, 0, False
            )
            if not ok:
                return
            target = files[names.index(choice)]

        try:
            scene = Scene.load(target)
        except (OSError, ValueError) as exc:
            QtWidgets.QMessageBox.critical(self, tr("载入失败"), str(exc))
            return

        # 先接管场景：机器人的名字以**场景**为准（与"抓取工作站演示"同口径），
        # 顺序反过来会让视口按追加分到另一个名字（详见那里的注释）
        self._adopt_scene(scene)

        # 场景里的机器人可能与当前视口的不同——让它加载场景里那台
        if scene.robot_urdf and Path(scene.robot_urdf).exists():
            name = scene.robot_names[0] if scene.robot_names else ROBOT_NAME
            self.gl_widget.load_robot(scene.robot_urdf, name)
            if scene.has_robot:
                self.gl_widget.set_joint_values(
                    {
                        n: math.radians(v)
                        for n, v in zip(
                            self.gl_widget.renderer.actuated_names(), scene._robot_joints_deg
                        )
                    }
                )
        self.statusBar().showMessage(
            tr("已载入 {name}（{n} 个对象）").format(name=target.name, n=len(scene))
        )

    def load_grasping_workstation_demo(self) -> None:
        """一键载入抓取工作站并自动装上示例程序（S4 能力展示入口）。

        与 :meth:`load_preset_cell` 只装场景不同，这里把演示做成**一气呵成**：

        1. 构建模板场景（UR5e + 吸盘 + 输送带 + 4 工件 + 托盘）并载入视口
        2. 用模板 TCP + IK 生成五点式抓取循环示教点（含抓取/放置语义）
        3. 灌进路径编辑器与离线编程面板（与手工示教同一条数据通路）
        4. 自动校验 → 状态栏显示节拍 → 自动开始轨迹回放

        任一步失败都弹窗如实说明，不做静默降级。
        """
        # 原为 ``… and _HAS_PROGRAM and …``：编程面板只在可用时才被创建，故
        # ``hasattr`` 蕴含该标志；而且它**更严**——少了它，缺机器人库时会一路走到
        # ``self.program_panel`` 才炸，而不是这里如实报"所需模块不可用"。
        if not (_HAS_TEMPLATES and hasattr(self, "program_panel") and _HAS_PICKPLACE):
            QtWidgets.QMessageBox.information(
                self,
                tr("抓取工作站演示"),
                tr("所需模块不可用（模板/环境建模/离线编程/抓取工艺）。"),
            )
            return

        try:
            cell = build_grasping_workstation()
        except Exception as exc:
            QtWidgets.QMessageBox.critical(
                self,
                tr("抓取工作站演示"),
                tr("模板构建失败：{kind}: {msg}").format(kind=type(exc).__name__, msg=exc),
            )
            return

        # URDF 路径兼容仓库相对路径（模板 JSON 里存的是相对路径）
        urdf = cell.scene.robot_urdf
        if urdf and not Path(urdf).exists():
            candidate = _REPO_ROOT / urdf
            if candidate.exists():
                urdf = str(candidate)
        if not urdf or not Path(urdf).exists():
            QtWidgets.QMessageBox.critical(
                self, tr("抓取工作站演示"), tr("机器人模型不可用：{urdf}").format(urdf=urdf)
            )
            return

        # 1) 先接管模板场景 → 再**认领**里面的机器人（顺序是不能反的关键）
        #
        #    反过来的话：load_robot 会在**旧场景**里按"追加"分到下一个名字
        #    （机器人_2），随后 _adopt_scene 换上模板场景，名字又从 机器人 开始
        #    编号——视口与离线编程面板手里是旧名字、场景里是新名字，于是：
        #    ① 回放吸附工件时 attach 到一个不存在的父对象（工件不随动）；
        #    ② 存工程时多出一台"幽灵机器人"的程序段，重开后当前那台看不到点。
        #    先换场景再认领，两边同名，这类错位从根上不成立。
        self._adopt_scene(cell.scene)
        self.load_robot(urdf, adopt_existing=True)
        robot = self.olp_robot
        if robot is None:
            QtWidgets.QMessageBox.warning(
                self, tr("抓取工作站演示"), tr("离线编程机器人初始化失败，无法生成示例程序。")
            )
            return

        # 2) 生成示例示教点（TCP 取模板定义的吸盘 TCP）
        try:
            positions, types = demo_taught_points(cell, robot)
        except Exception as exc:
            QtWidgets.QMessageBox.critical(
                self,
                tr("抓取工作站演示"),
                tr("示例程序生成失败：{kind}: {msg}").format(kind=type(exc).__name__, msg=exc),
            )
            return

        # 3) 灌示教点：**走和手工示教同一条通路**（set_recorded_positions 带归属）。
        #    之前这里是直接给 recorded_positions 赋值，点有了但没归属，
        #    _update_code_with_positions 里 segments() 空、代码一个 movej 都不写——
        #    点数看着 8 个，存盘再打开发现一个点都没有（program 段只有两行注释）。
        #    场景已在第 1 步接管。
        self.code_editor.set_recorded_positions(positions, owner=self.gl_widget.current_robot)
        self.code_editor.point_types = list(types)  # set 后会上一套默认值，这里盖回去
        self.code_editor.pos_count_label.setText(tr("示教点：{n}").format(n=len(positions)))
        self.code_editor._update_code_with_positions()
        self.gl_widget.set_recorded_positions(positions)
        self._sync_program_points()

        # 工件下拉默认选第一个可抓工件，回放即可看到吸附/释放随动
        if cell.grasp_objects and hasattr(self.code_editor, "workpiece_combo"):
            idx = self.code_editor.workpiece_combo.findData(cell.grasp_objects[0])
            if idx >= 0:
                self.code_editor.workpiece_combo.setCurrentIndex(idx)

        # 视口关节摆到模板 Home 位（load_robot 用的是通用待机位形，与模板不同）
        names = self.gl_widget.renderer.actuated_names()
        self.gl_widget.set_joint_values(
            {n: math.radians(v) for n, v in zip(names, cell.home_joints)}
        )
        self._sync_scene_joints()

        # 4) 自动校验出节拍 → 自动回放
        report = self.program_panel.validate_program()
        if report is not None:
            self.program_panel.start_playback()
        cycle = getattr(report, "cycle_time", None)
        cycle_txt = tr("，节拍 {t} s").format(t=f"{cycle:.2f}") if cycle is not None else ""
        self.statusBar().showMessage(
            tr(
                "抓取工作站演示已就绪：{n} 个示例示教点{cycle_txt}，" "回放中（可暂停/单步/停止）"
            ).format(n=len(positions), cycle_txt=cycle_txt)
        )

    def open_model(self):
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open Robot Model", "assets/models", "URDF Files (*.urdf);;All Files (*)"
        )
        if file_path:
            self.load_robot(file_path)

    def _register_robot_in_scene(self, path, adopt_existing: bool = False) -> str:
        """把机器人登记进工作单元场景：**已有机器人时追加**，返回其名字。

        回归：此前一律走 ``Scene.set_robot``，名字固定为 ``ROBOT_NAME``，于是
        "再增加一台机器人"实际是把已有的那台顶掉。

        Args:
            path: URDF 路径
            adopt_existing: ``True`` 表示**认领**场景里已有的机器人（工程载入
                用），不再追加。工程载入时机器人已经随场景一起恢复了，再追加
                一台就会凭空多出一个默认名的机器人

        Returns:
            场景中的机器人名
        """
        scene = self.scene_panel.scene
        if scene.robot_names and adopt_existing:
            return scene.robot_names[0]
        if scene.robot_names:
            # 追加并沿 X 错开基座，避免两台叠在一起（随后可自行拖放到位）
            idx = len(scene.robot_names)
            name = self.scene_panel.add_robot(
                str(path),
                pose=Pose.from_xyz_rpy(1500.0 * idx, 0.0, 0.0, 0.0, 0.0, 0.0),
            )
            return name or scene.robot_names[-1]
        self.scene_panel.set_robot(str(path))
        return scene.robot_names[0] if scene.robot_names else ROBOT_NAME

    def load_robot(self, path, adopt_existing: bool = False):
        """加载 URDF 到 3D 视口，并同步属性面板。

        若该文件属于机器人库中的某个型号，属性面板会回显负载、臂展等完整规格。

        Args:
            path: URDF 路径
            adopt_existing: ``True`` 表示机器人**已在场景里**（工程载入），只
                认领不再追加
        """
        # 先决定"追加、认领还是替换"：场景已有机器人时必须用 add_robot（唯一名），
        # 否则 set_robot 的固定名 ROBOT_NAME 会把已有那台顶掉——"再加一台"
        # 就变成了"替换"。名字要先定，渲染器才能登记到对应名下。
        robot_name = ROBOT_NAME
        if hasattr(self, "scene_panel"):
            robot_name = self._register_robot_in_scene(path, adopt_existing)

        if not self.gl_widget.load_robot(path, robot_name):
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to load URDF: {path}")
            return

        # 记录当前模型路径：后处理需要据此推断品牌与关节结构。
        # 历史实现从不更新该字段，导致即使已载入机器人，
        # 依赖 urdf_path 的功能（如生成控制器代码）仍认为"未加载模型"。
        self.urdf_path = str(path)

        entry = None
        if hasattr(self, "library_panel"):
            entry = self.library_panel.catalog.find_by_urdf(path)

        # 规格必须按场景里的机器人名留存：多机时选中谁就回显谁的型号/厂商，
        # 否则永远显示最后加载那台（2026-09-21 用户反馈）
        self.property_panel.update_robot_info(self.gl_widget.renderer, entry, name=robot_name)

        # 设置关节名称顺序，用于解析路径代码
        joint_names = list(self.gl_widget.renderer.joints.keys())
        self.code_editor.set_joint_names(joint_names)

        # 按归属回灌：整份灌进来的话，新加的这台会凭空"继承"别人教好的路径
        self._sync_viewport_paths_from_editor()
        self.statusBar().showMessage(f"Loaded: {Path(path).name}")

        # 同步机器人到工作单元场景，使其参与干涉检查
        if hasattr(self, "scene_panel"):
            self.gl_widget.scene = self.scene_panel.scene
            # 必须同步：渲染器加载时会摆到 HOME 待机位（_apply_home_pose），
            # 而 Scene 里刚建的机器人是**零位**。不同步就出现"画出来的手臂是
            # 弯的、场景法兰却竖直朝天"——挂到法兰上的夹爪跑到手臂外，看着像
            # 装夹失败（位置面板却显示 0/0/0，因为挂载本身成功了）。
            self._sync_scene_joints()
            # 让「代码生成 → 硬件列表」能按场景补齐设备清单
            self.code_editor._scene = self.scene_panel.scene
            self.code_editor.sync_devices(self.scene_panel.scene)
            # 载入后即把机器人作为当前对象：右面板显示关节滑块，视口画黄色高亮
            if robot_name in self.scene_panel.scene.objects:
                self.gl_widget.selected_object = robot_name
                self.gl_widget.update()
                self.property_panel.set_object(robot_name)

        # 离线编程面板需要真正的 core.Robot（含运动学），而不是只用于
        # 绘制的 renderer——校验与节拍估算都建立在 FK/IK 之上
        # 只判 hasattr（原为 ``_HAS_PROGRAM and hasattr(...)``，后者蕴含前者）
        if hasattr(self, "program_panel"):
            try:
                from domain.robot import Robot as CoreRobot

                self.olp_robot = CoreRobot(str(path), Path(path).stem)
                # 必须按**场景里的机器人名**登记：多机同型号时 URDF 文件名相同
                # （都是 ur5e），按它做键会把两台的程序合并成一份。
                self.program_panel.set_robot(self.olp_robot, joint_names, name=robot_name)
                # 「代码生成 → 机器人子程序」需要同样的真实机器人
                self.code_editor.set_robot(self.olp_robot, joint_names)
                self.program_panel.set_scene(
                    self.scene_panel.scene if hasattr(self, "scene_panel") else None
                )
                self._sync_program_points()
            except Exception as exc:
                logger.warning("prepare_program_robot_failed: %s", exc)
                self.olp_robot = None

        self.gl_widget.update()
