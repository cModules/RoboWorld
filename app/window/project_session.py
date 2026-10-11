"""工程文件与会话状态（ADR-002 拆解第 8 步）。

承接 .core 工程的存取与会话周边状态：保存/打开、最近打开、崩溃自动恢复快照、
从模板新建。方法体从 ``main.py`` 原样搬出，可选依赖的降级探测复制到本模块，
不依赖入口脚本的模块级状态。

工程 IO 的一条硬规则：**只收界面里真的有的东西**——没有场景就不写场景段，
而不是写一段空的；载入时同理，缺哪段就如实报"已跳过"，不猜、不静默丢。
"""

from __future__ import annotations

import math
from pathlib import Path

from PySide6 import QtWidgets

from app.window.capabilities import (
    HANDSHAKE_SIGNALS,
    PROJECT_EXTENSION,
    PROJECT_FILTER,
    PointType,
    ProcessSequence,
    Project,
    ProjectError,
    RecentProjects,
    Scene,
    _CORE_VERSION,
    _HAS_CELL,
    _HAS_PICKPLACE,
    _HAS_PROJECT,
    _HAS_PROJECT_TEMPLATES,
    _HAS_ROBOT_LIBRARY,
    _HAS_SEQUENCE,
    _HAS_SESSION_SUPPORT,
    _HAS_WORLD,
    _REPO_ROOT,
    build_project_template,
    cell_from_snapshot,
    cell_snapshot,
    clear_autosave,
    list_project_templates,
    read_autosave,
    tr,
    write_autosave,
)
from PySide6.QtCore import Qt

from domain.world.templates import REPO_ROOT as _REPO_ROOT
from support.i18n import tr


# 版本号：窗口标题从库版本派生，不手写
# 工程文件（.core）：场景 / 示教程序 / 工艺时序 / 设备点表 / 零件来源
# 工艺时序：工程载入时把段还原成 ProcessSequence
# 会话支撑：最近打开列表 + 崩溃自动恢复（依赖工程文件层）
# 工程模板：「从模板新建工程」——新活不从空白开始
# 场景 / 工作单元 / 抓取语义 / 零件库：载入时要还原这些段
class ProjectSessionMixin:
    """工程文件存取与会话周边状态（最近打开 / 自动保存 / 模板）。"""

    def _project_name(self) -> str:
        """当前工程名（工程名 → 文件名 → 空串；显示时才用「未命名工程」兜底）。"""
        if _HAS_PROJECT and self.project is not None:
            if self.project.name:
                return self.project.name
            if self.project.path is not None:
                return self.project.path.stem
        return ""

    def _update_window_title(self) -> None:
        """标题带工程名与未保存标记——"这个界面里装的是哪个工程"一眼可见。"""
        mark = tr(" •未保存") if self._project_dirty else ""
        name = self._project_name() or tr("未命名工程")
        self.setWindowTitle(f"RoboWorld Terminal v{_CORE_VERSION} — {name}{mark}")

    def _set_project_dirty(self, dirty: bool = True) -> None:
        if not _HAS_PROJECT:
            return
        self._project_dirty = bool(dirty)
        self._update_window_title()

    def _collect_into(self, project) -> None:
        """把当前会话的各部分写进工程对象。

        **只收界面里真的有的东西**：没有场景就不写场景段，而不是写一段空的
        ——工程摘要里"工程里没有：场景"比"场景：0 个对象"诚实。
        """
        # 只判 hasattr：宿主窗口仅在环境建模可用时才创建场景面板（从不置 None），
        # 故 hasattr 已蕴含 _HAS_WORLD。
        if hasattr(self, "scene_panel"):
            scene = getattr(self.scene_panel, "scene", None)
            if scene is not None and (len(scene) or getattr(scene, "has_robot", False)):
                project.set("scene", scene.to_dict())

        if hasattr(self, "code_editor"):
            # 存**整份**语义：代码里装着所有机器人的段，只存"当前编辑那台"
            # 一段的话，载入时点数对不上，只能全部按默认语义重建（抓取点丢失）
            types = [getattr(t, "value", str(t)) for t in (self.code_editor.point_types or [])]
            project.set(
                "program",
                {
                    "code": self.code_editor.toPlainText(),
                    "point_types": types,
                    "joint_names": list(self.code_editor.joint_names or []),
                },
            )

        # 多机：每台机器人**各自**的示教点与抓取语义。上面那段 ``program`` 只
        # 装得下当前那台（路径编辑器一次只显示一台），多机工程存盘再打开会丢掉
        # 其余机器人教好的抓取循环——抓取编程必须存得住。
        # 只判 hasattr：同上，编程面板只在可用时才被创建。
        if hasattr(self, "program_panel"):
            snapshot = self.program_panel.robot_snapshot()
            for name, payload in snapshot.items():
                # 工件归属一并存：谁抓哪件是工艺的一部分
                payload["workpiece"] = self._workpiece_by_robot.get(name)
            if snapshot:
                project.set("programs", snapshot)

        if (
            _HAS_WORLD
            and hasattr(self, "scene_panel")
            and hasattr(self.scene_panel, "sequence_editor")
        ):
            project.set("sequence", self.scene_panel.sequence_editor.sequence.to_dict())

        cell = getattr(getattr(self, "code_editor", None), "cell", None)
        # 这里**必须**保留 ``_HAS_CELL``：它保护的不是面板存在与否，而是
        # ``cell_snapshot`` 这个降级时置 None 的函数——去掉会在缺工作单元模块时
        # 直接 TypeError（与上面几处"标志蕴含于 hasattr"的情形不同，别一起简化）。
        if _HAS_CELL and cell is not None:
            project.set("cell", cell_snapshot(cell))

        panel = getattr(self, "library_panel", None)
        sources = list(getattr(panel, "parts_sources", []) or [])
        project.set("parts", {"sources": sources})

    @staticmethod
    def _resolve_project_path(raw: str, base) -> "Path | None":
        """把工程里记的路径解析成真实文件（原样 → 工程目录 → 仓库根）。"""
        candidate = Path(raw)
        if candidate.exists():
            return candidate
        for root in (base, _REPO_ROOT):
            if root is None:
                continue
            alt = Path(root) / raw
            if alt.exists():
                return alt
        return None

    def _apply_project(self, project) -> list:
        """把工程内容应用到界面，返回**应用时发现的问题**（人话字符串）。"""
        issues: list = []

        # ---- 场景 ---- #
        payload = project.get("scene")
        if payload is not None:
            if not hasattr(self, "scene_panel"):
                issues.append(tr("场景段已跳过：环境建模不可用"))
            else:
                scene = None
                try:
                    scene = Scene.from_dict(payload)
                except Exception as exc:
                    issues.append(tr("场景段载入失败：{msg}").format(msg=exc))
                if scene is not None:
                    urdf = scene.robot_urdf
                    if urdf and not Path(urdf).exists():
                        alt = _REPO_ROOT / urdf
                        if alt.exists():
                            urdf = str(alt)
                    # 场景必须先落地：机器人是随场景一起恢复的，认领它要在
                    # adopt 之后——否则 load_robot 操作的是**旧的**场景，会往
                    # 里面再追加一台默认名的机器人，多机工程于是凭空多一台
                    self._adopt_scene(scene)
                    if urdf and Path(urdf).exists():
                        # 只认领，不追加
                        self.load_robot(urdf, adopt_existing=True)
                        names = self.gl_widget.renderer.actuated_names()
                        joints = getattr(scene, "_robot_joints_deg", []) or []
                        if getattr(scene, "has_robot", False) and len(joints) == len(names):
                            self.gl_widget.set_joint_values(
                                {n: math.radians(v) for n, v in zip(names, joints)}
                            )
                    elif urdf:
                        issues.append(
                            tr("场景里的机器人模型找不到：{urdf}（场景仍已载入）").format(urdf=urdf)
                        )
                    if hasattr(self, "program_panel"):
                        self.program_panel.set_scene(scene)
                        # 场景里**每台**机器人都要登记：只登记主机器人的话，
                        # 其余机器人的抓取程序就成了没有主人的孤儿
                        self._restore_robot_programs(scene)

        # ---- 示教程序（含代码文本里没有的工艺语义） ---- #
        payload = project.get("program")
        if payload is not None:
            if not hasattr(self, "code_editor"):
                issues.append(tr("示教程序段已跳过：路径编辑器不可用"))
            else:
                self.code_editor.setPlainText(payload.get("code") or "")
                self.code_editor._parse_code_to_positions()
                n_pts = len(self.code_editor.get_recorded_positions())
                types_raw = [
                    str(t)
                    for t in (payload.get("point_types") or [])
                    if t not in (None, "", "None", "null")
                ]
                if types_raw and _HAS_PICKPLACE:
                    if len(types_raw) != n_pts:
                        issues.append(
                            tr(
                                "示教点语义 {a} 个与解析出的 {b} 个点不一致，"
                                "已按默认语义重建（不猜哪个是抓取点）"
                            ).format(a=len(types_raw), b=n_pts)
                        )
                        self.code_editor.sync_point_types()
                    else:
                        try:
                            self.code_editor.point_types = [PointType(t) for t in types_raw]
                        except ValueError as exc:
                            issues.append(
                                tr("示教点语义含未知值（{msg}），已按默认语义重建").format(msg=exc)
                            )
                            self.code_editor.sync_point_types()
                # 按归属回灌：代码里的 ``# Robot:`` 段头已经写明了每段属于谁
                self._sync_viewport_paths_from_editor()
                self._sync_program_points()

        # ---- 多机：每台机器人各自的示教点与抓取语义 ---- #
        payload = project.get("programs")
        if payload is not None:
            if not hasattr(self, "program_panel"):
                issues.append(tr("多机示教点段已跳过：离线编程模块不可用"))
            else:
                self.program_panel.restore_robots(payload)
                for name, data in payload.items():
                    data = data or {}
                    if data.get("workpiece"):
                        self._workpiece_by_robot[name] = data["workpiece"]
                    positions = data.get("positions") or []
                    if positions:
                        # 视口也要按名回填：路径可视化与各台的点表都从它读
                        self.gl_widget.set_recorded_positions(positions, name)

        # ---- 工艺时序 ---- #
        payload = project.get("sequence")
        if payload is not None:
            editor = getattr(getattr(self, "scene_panel", None), "sequence_editor", None)
            if editor is None or not _HAS_SEQUENCE:
                issues.append(tr("工艺时序段已跳过：时序编辑器不可用"))
            else:
                try:
                    editor.set_sequence(ProcessSequence.from_dict(payload))
                except Exception as exc:
                    issues.append(tr("工艺时序段载入失败：{msg}").format(msg=exc))

        # ---- 设备点表 ---- #
        payload = project.get("cell")
        if payload is not None:
            # 同上：``_HAS_CELL`` 保护 ``cell_from_snapshot`` 不为 None，必须保留。
            if not (_HAS_CELL and hasattr(self, "code_editor")):
                issues.append(tr("设备点表段已跳过：工作单元模块不可用"))
            else:
                try:
                    cell = cell_from_snapshot(payload)
                except ValueError as exc:
                    issues.append(tr("设备点表段载入失败：{msg}").format(msg=exc))
                else:
                    self.code_editor.set_cell(cell)
                    editor = getattr(getattr(self, "scene_panel", None), "sequence_editor", None)
                    if editor is not None:
                        editor.set_available_signals(
                            sorted(
                                {sig for d in cell.devices for sig in d.io_map}
                                | set(HANDSHAKE_SIGNALS)
                            )
                        )

        # ---- 零件库来源 ---- #
        payload = project.get("parts")
        if payload is not None:
            sources = [str(s) for s in (payload.get("sources") or [])]
            panel = getattr(self, "library_panel", None)
            if not (_HAS_ROBOT_LIBRARY and panel is not None):
                if sources:
                    issues.append(tr("零件库来源已跳过：机器组件库不可用"))
            else:
                base = project.path.parent if getattr(project, "path", None) else None
                for raw in sources:
                    resolved = self._resolve_project_path(raw, base)
                    if resolved is None:
                        issues.append(
                            tr("零件库文件找不到：{src}（未导入；工程里仍记录着它）").format(
                                src=raw
                            )
                        )
                        continue
                    try:
                        n = panel.import_parts_file(resolved)
                    except Exception as exc:
                        issues.append(tr("零件库导入失败：{src}（{msg}）").format(src=raw, msg=exc))
                    else:
                        self.statusBar().showMessage(
                            tr("已从工程恢复零件库：{n} 个自定义零件").format(n=n)
                        )
        return issues

    def save_project(self) -> None:
        """「保存工程…」——把场景/示教程序/时序/点表/零件来源收进一个 .core。"""
        if not _HAS_PROJECT:
            QtWidgets.QMessageBox.information(self, tr("保存工程"), tr("工程文件模块不可用。"))
            return
        # 已在某工程上工作时不新建对象：**未知段与未知顶层键要跟着走**
        # （否则"打开—保存"一趟就把别家工具写的字段抹掉了）
        project = self.project if self.project is not None else Project()
        suggested = (
            str(project.path)
            if project.path is not None
            else str(_REPO_ROOT / f"{self._project_name() or 'workcell'}{PROJECT_EXTENSION}")
        )
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, tr("保存工程"), suggested, PROJECT_FILTER
        )
        if not path:
            return
        self._collect_into(project)
        try:
            target = project.save(path)
        except ProjectError as exc:
            QtWidgets.QMessageBox.critical(self, tr("保存工程失败"), str(exc))
            return
        self.project = project
        self._set_project_dirty(False)
        self._record_recent(target)
        self._clear_autosave_snapshot()
        summary = project.summary()
        self.statusBar().showMessage(
            tr("工程已保存：{name}（{n} 段）").format(name=target.name, n=len(summary))
        )

    def open_project(self) -> None:
        """「打开工程…」——选文件后走 :meth:`open_project_path` 同一条路。"""
        if not _HAS_PROJECT:
            QtWidgets.QMessageBox.information(self, tr("打开工程"), tr("工程文件模块不可用。"))
            return
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, tr("打开工程"), "", PROJECT_FILTER)
        if not path:
            return
        self.open_project_path(path)

    def open_project_path(
        self, path: str, record_recent: bool = True, clear_autosave: bool = True
    ) -> bool:
        """载入 .core，逐段应用，并把跳过/缺失如实报出来。

        「打开工程…」对话框与「最近打开」菜单条目走的是**同一条路**，
        返回是否载入成功（最近打开菜单用它判断要不要记这条记录）。
        崩溃恢复也走这条路，但 `record_recent=False`（快照是内部槽位，
        不进最近打开）、`clear_autosave=False`（快照正是刚恢复的内容，
        在用户把它存到真实位置前还得留着）。
        """
        if not _HAS_PROJECT:
            return False
        try:
            project = Project.load(path)
        except ProjectError as exc:
            QtWidgets.QMessageBox.critical(self, tr("打开工程失败"), str(exc))
            return False

        issues = self._apply_project(project)
        self.project = project
        self._set_project_dirty(False)
        if record_recent:
            self._record_recent(project.path or Path(path))
        # 磁盘上已有一份工程了，旧的自动保存快照过时且有误导性 → 清掉
        if clear_autosave:
            self._clear_autosave_snapshot()

        report = project.load_report
        lines = [tr("已打开 {name}").format(name=Path(path).name)]
        if report is not None:
            lines.append(report.describe())
        if issues:
            lines.append(tr("应用时的问题："))
            lines.extend("  · " + s for s in issues)
        text = "\n".join(lines)
        if issues or (report is not None and not report.ok()):
            box = QtWidgets.QMessageBox(self)
            box.setWindowTitle(tr("打开工程"))
            box.setText(text)
            box.exec()
        self.statusBar().showMessage(tr("工程已打开：{name}").format(name=Path(path).name))
        return True

    def _record_recent(self, path) -> None:
        """记一条最近打开（打开成功 / 保存成功后调用；尽力而为）。"""
        if not _HAS_SESSION_SUPPORT or path is None:
            return
        recents = RecentProjects.load()
        recents.add(str(path))
        recents.save()
        self._rebuild_recent_menu()

    def _rebuild_recent_menu(self) -> None:
        """按最近打开记录重建子菜单；失效条目置灰（不悄悄藏）。"""
        if not _HAS_SESSION_SUPPORT:
            return
        menu = getattr(self, "_recent_menu", None)
        if menu is None:
            return
        menu.clear()
        recents = RecentProjects.load()
        entries = recents.paths or []
        if not entries:
            empty = menu.addAction(tr("（暂无记录）"))
            empty.setEnabled(False)
            return
        for index, path in enumerate(entries, start=1):
            action = menu.addAction(f"&{index}  {path}")
            action.setEnabled(Path(path).exists())
            if action.isEnabled():
                action.triggered.connect(lambda _checked=False, p=path: self.open_project_path(p))
        menu.addSeparator()
        clear_action = menu.addAction(tr("清除最近打开记录"))
        clear_action.triggered.connect(self._clear_recent_menu_records)

    def _clear_recent_menu_records(self) -> None:
        if not _HAS_SESSION_SUPPORT:
            return
        RecentProjects().save()
        self._rebuild_recent_menu()
        self.statusBar().showMessage(tr("已清除最近打开记录"))

    def _write_autosave_snapshot(self) -> bool:
        """把当前会话快照写进自动保存目录（不动正在编辑的工程对象）。

        快照走一份**新建的** Project：收集会话内容 + 带上原工程的
        未知段/未知顶层键/元信息——原工程对象的 ``path`` 不能被快照
        覆盖，否则下次手动保存的建议路径会错。
        """
        if not (_HAS_SESSION_SUPPORT and self.project is not None):
            return False
        snapshot = Project(
            name=self.project.name,
            note=self.project.note,
            created=self.project.created,
            modified=self.project.modified,
            core_version=self.project.core_version,
        )
        snapshot.raw_sections = dict(self.project.raw_sections)
        snapshot.extra = dict(self.project.extra)
        self._collect_into(snapshot)
        source_path = str(self.project.path) if self.project.path is not None else None
        return write_autosave(snapshot, source_path=source_path) is not None

    def _autosave_tick(self) -> None:
        """定时器心跳：只在有未保存改动时落快照（干净的会话没什么可救）。"""
        if not _HAS_SESSION_SUPPORT or not self._project_dirty:
            return
        self._write_autosave_snapshot()

    def _clear_autosave_snapshot(self) -> None:
        if _HAS_SESSION_SUPPORT:
            clear_autosave()

    def maybe_prompt_recovery(self) -> bool:
        """启动时若存在自动保存快照，询问是否恢复；恢复成功返回 True。

        只问一次（``showEvent`` 可能触发多次）；没有任何可靠元数据的
        快照不弹询问——快照文件本身仍是合法 .core，用户可以手动打开。
        """
        if not _HAS_SESSION_SUPPORT or self._recovery_prompted:
            return False
        self._recovery_prompted = True
        info = read_autosave()
        if info is None:
            return False
        source = info.source_path or tr("（未命名工程）")
        saved_at = info.saved_at or tr("（未知时间）")
        answer = QtWidgets.QMessageBox.question(
            self,
            tr("恢复上次会话"),
            tr(
                "检测到上次会话的自动保存。\n\n"
                "工程：{source}\n保存时间：{saved}\n\n"
                "是否恢复？（不恢复也可以稍后用「打开工程」手动打开快照：\n{snapshot}）"
            ).format(source=source, saved=saved_at, snapshot=info.snapshot),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.Yes,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return False
        if not self.open_project_path(
            str(info.snapshot), record_recent=False, clear_autosave=False
        ):
            return False
        # 恢复的内容在快照里、不在盘上的"工程位置"——按未保存改动对待，
        # 让标题的"•未保存"和自动保存继续守护它；手动保存的建议路径
        # 指回原工程位置（快照路径只是内部槽位，不该被当成保存目标）
        if self.project is not None:
            self.project.path = Path(info.source_path) if info.source_path else None
            if not info.source_path:
                self.project.name = ""
        self._set_project_dirty(True)
        self.statusBar().showMessage(tr("已从自动保存恢复：{saved}").format(saved=saved_at))
        return True

    def new_project_from_template(self, template_id: str | None = None) -> bool:
        """「从模板新建工程」；传入 ``template_id`` 时跳过选择对话框。

        模板产物走 :meth:`_apply_project` 同一条应用通路（与打开工程
        一致）；新建的工程尚未落盘，按未保存改动对待（标题「•未保存」
        + 自动保存继续守护）。
        """
        if not (_HAS_PROJECT_TEMPLATES and _HAS_PROJECT):
            QtWidgets.QMessageBox.information(
                self, tr("从模板新建工程"), tr("工程模板模块不可用。")
            )
            return False
        if template_id is None:
            template_id = self._choose_project_template()
            if not template_id:
                return False
        if self._project_dirty:
            answer = QtWidgets.QMessageBox.question(
                self,
                tr("从模板新建工程"),
                tr("当前工程有未保存改动，新建模板工程会丢弃这些改动。\n\n是否继续？"),
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if answer != QtWidgets.QMessageBox.Yes:
                return False
        try:
            project = build_project_template(template_id)
        except Exception as exc:
            QtWidgets.QMessageBox.critical(
                self,
                tr("从模板新建工程"),
                tr("模板构建失败：{kind}: {msg}").format(kind=type(exc).__name__, msg=exc),
            )
            return False

        issues = self._apply_project(project)
        self.project = project
        self._set_project_dirty(True)
        if issues:
            box = QtWidgets.QMessageBox(self)
            box.setWindowTitle(tr("从模板新建工程"))
            box.setText(
                tr("已从模板新建：{name}").format(name=project.name or template_id)
                + "\n"
                + tr("应用时的问题：")
                + "\n"
                + "\n".join("  · " + s for s in issues)
            )
            box.exec()
        self.statusBar().showMessage(
            tr("已从模板新建：{name}（尚未保存，可直接修改后「保存工程」）").format(
                name=project.name or template_id
            )
        )
        return True

    def _choose_project_template(self) -> "str | None":
        """模板选择对话框；返回选中的模板 id，取消返回 ``None``。"""
        templates = list_project_templates()
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle(tr("从模板新建工程"))
        layout = QtWidgets.QVBoxLayout(dialog)
        layout.addWidget(QtWidgets.QLabel(tr("选择一个模板作为新工程的起点：")))
        listw = QtWidgets.QListWidget(dialog)
        for tpl in templates:
            item = QtWidgets.QListWidgetItem(tpl["name"])
            item.setData(Qt.UserRole, tpl["id"])
            item.setToolTip(tpl["description"])
            listw.addItem(item)
        desc = QtWidgets.QLabel("")
        desc.setWordWrap(True)
        layout.addWidget(listw)
        layout.addWidget(desc)

        def _sync_desc() -> None:
            item = listw.currentItem()
            tid = item.data(Qt.UserRole) if item is not None else None
            desc.setText(next((t["description"] for t in templates if t["id"] == tid), ""))

        listw.currentItemChanged.connect(lambda *_: _sync_desc())
        listw.itemDoubleClicked.connect(lambda *_: dialog.accept())
        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        listw.setMinimumWidth(420)
        listw.setCurrentRow(0)
        _sync_desc()
        if dialog.exec() != QtWidgets.QDialog.Accepted:
            return None
        item = listw.currentItem()
        return item.data(Qt.UserRole) if item is not None else None
