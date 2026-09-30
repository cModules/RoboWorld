"""窗口级「程序校验 · 控制器代码生成 · 路径代码存取」（ADR-002 拆解第 4 步）。

从 ``main.py`` 的 ``MainWindow`` 原样搬出（方法体未改一字）。这一组是**程序侧动作**：
一键校验（F7 / Shift+F7）、把示教点编译成目标控制器代码（KRL / RAPID / TP / URScript）、
以及路径代码的存/取。它们的共同点是"读面板状态 → 干一件有去无回的事"（弹窗、写文件），
不参与窗口的编辑状态机，所以能整组搬走。

与 ``main.py`` 版本的**唯一**差别：``_HAS_PROGRAM and hasattr(self, "program_panel")``
简化为 ``hasattr(self, "program_panel")``——宿主窗口只在编程面板可用时才创建它
（``setup_ui`` 里 ``if _HAS_PROGRAM:``），从不置 ``None``，故 ``hasattr`` 已蕴含该标志。
"""

from __future__ import annotations

import math
from pathlib import Path

from PySide6 import QtGui, QtWidgets
from PySide6.QtGui import QFont

from support.i18n import tr


class ProgramOpsMixin:
    """程序校验、控制器代码生成与路径代码存取（宿主：``MainWindow``）。"""

    def validate_program(self) -> None:
        """校验程序并给出节拍（F7）。"""
        # 只判 hasattr：宿主窗口仅在编程面板可用时才创建它（从不置 None）。
        if not hasattr(self, "program_panel"):
            QtWidgets.QMessageBox.information(self, tr("程序校验"), tr("离线编程模块不可用。"))
            return
        self._focus_tab(self.program_panel)
        # 校验的是**视口当前正在看的那台**：否则会出现"选了 B、却校了 A"
        current = getattr(self.gl_widget, "current_robot", None)
        if current and current in self.program_panel.robot_names:
            self.program_panel.set_active_robot(current)
        self._sync_program_points()
        self.program_panel.validate_program()

    def validate_all_programs(self) -> None:
        """一键校验**全部**机器人（Shift+F7）。

        多机时一台一台切过去按 F7 太笨——切走就把上一台的报告刷掉了，前面几台
        校出问题等于没看见。这里逐台校完再汇总。
        """
        if not hasattr(self, "program_panel"):
            QtWidgets.QMessageBox.information(self, tr("程序校验"), tr("离线编程模块不可用。"))
            return
        if not getattr(self.program_panel, "validate_all", None):
            QtWidgets.QMessageBox.information(self, tr("程序校验"), tr("当前版本不支持批量校验。"))
            return
        self._focus_tab(self.program_panel)
        self._sync_program_points()
        reports = self.program_panel.validate_all()
        if not reports:
            QtWidgets.QMessageBox.information(
                self,
                tr("校验全部机器人"),
                tr("还没有登记任何机器人——请先载入机器人型号。"),
            )

    def _show_validation_summary(self, reports: dict) -> None:
        """一键校验全部的汇总：每台一行，哪台有问题一眼看见。

        汇总只给结论，详情仍走编程面板——点「看这台」切过去并显示它的报告。
        """
        if not reports:
            return
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle(tr("校验全部机器人"))
        dlg.setMinimumSize(560, 320)
        layout = QtWidgets.QVBoxLayout(dlg)

        table = QtWidgets.QTableWidget(len(reports), 5, dlg)
        table.setHorizontalHeaderLabels(
            [tr("机器人"), tr("结果"), tr("错误"), tr("警示"), tr("节拍 (s)")]
        )
        table.verticalHeader().setVisible(False)
        table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        table.horizontalHeader().setStretchLastSection(True)

        red = QtGui.QColor("#ff6b6b")
        green = QtGui.QColor("#6fdc6f")
        orange = QtGui.QColor("#ffb86b")
        names = list(reports)
        for row, name in enumerate(names):
            report = reports[name]
            cells = [name]
            if report is None:
                cells += [tr("没程序（未示教）"), "—", "—", "—"]
                color = orange
            elif report.ok:
                cells += [
                    tr("通过"),
                    "0",
                    str(len(report.warnings)),
                    f"{report.cycle_time:.2f}" if report.cycle_time is not None else "—",
                ]
                color = green if not report.warnings else orange
            else:
                cells += [
                    tr("不通过"),
                    str(len(report.errors)),
                    str(len(report.warnings)),
                    f"{report.cycle_time:.2f}" if report.cycle_time is not None else "—",
                ]
                color = red
            for col, text in enumerate(cells):
                item = QtWidgets.QTableWidgetItem(text)
                if col == 1:
                    item.setForeground(color)
                table.setItem(row, col, item)
        table.resizeColumnsToContents()
        table.selectRow(0)
        layout.addWidget(table, 1)

        btn_row = QtWidgets.QHBoxLayout()
        btn_row.addStretch(1)
        btn_view = QtWidgets.QPushButton(tr("看这台"))
        btn_close = QtWidgets.QPushButton(tr("关闭"))
        btn_row.addWidget(btn_view)
        btn_row.addWidget(btn_close)
        layout.addLayout(btn_row)

        def goto() -> None:
            row = table.currentRow()
            if row < 0:
                return
            name = names[row]
            # 报告已经算好了，切过去显示即可——不必再校一遍
            self.program_panel.focus_report(name)
            dlg.accept()

        btn_view.clicked.connect(goto)
        btn_close.clicked.connect(dlg.reject)
        table.doubleClicked.connect(lambda _idx: goto())
        dlg.exec()

    def save_path_code(self):
        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save Path Code", "", "Python Files (*.py);;All Files (*)"
        )
        if file_path:
            with open(file_path, "w") as f:
                f.write(self.code_editor.toPlainText())
            self.statusBar().showMessage(f"Saved: {Path(file_path).name}")

    def load_path_code(self):
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load Path Code", "", "Python Files (*.py);;Text Files (*.txt);;All Files (*)"
        )
        if file_path:
            try:
                with open(file_path, "r", encoding="utf-8") as f:
                    code_content = f.read()
                self.code_editor.setPlainText(code_content)
                # 解析代码并更新路径
                self.code_editor._parse_code_to_positions()
                # 更新 GLWidget 的路径可视化：按段头归属分桶，别整份塞给当前那台
                self._sync_viewport_paths_from_editor()
                self.statusBar().showMessage(f"Loaded: {Path(file_path).name}")
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Error", f"Failed to load path code: {e}")

    def generate_robot_code(self):
        """把示教点编译成目标控制器的机器人代码。

        与「保存路径代码」不同，这里走的是完整的后处理链路
        （``service.program.Program.to_post``），输出 KRL / RAPID / TP / URScript
        等**控制器可直接加载**的源码，而不是中间的 Python 形态路径文本。

        出错时弹窗明确提示——不做静默降级。历史上后处理器一旦失败就会输出
        一堆 ``; Motion 1: ...`` 注释占位，界面却显示成功，极易误导。
        """
        positions = self.code_editor.get_recorded_positions()
        if not positions:
            QtWidgets.QMessageBox.information(
                self,
                tr("生成机器人代码"),
                tr("当前没有示教点。\n请先在 3D 视口中调整关节角度并点击「记录」按钮。"),
            )
            return

        urdf_path = getattr(self, "urdf_path", None)
        if not urdf_path or not Path(urdf_path).exists():
            QtWidgets.QMessageBox.warning(
                self,
                tr("生成机器人代码"),
                tr(
                    "未加载机器人模型，无法确定目标品牌与后处理器。\n请先从「机器人库」载入一个型号。"
                ),
            )
            return

        try:
            from domain.robot import Robot
            from service.program import Motion, MotionType, Program
            from adapter.post import list_post_processors
            from adapter.post.base import PostProcessError

            robot = Robot(str(urdf_path), Path(urdf_path).stem)
            joint_names = self.code_editor.joint_names or robot.joint_names

            motions = []
            for pos in positions:
                # 示教点内部以弧度保存，Motion 期望角度
                joints_deg = [math.degrees(float(pos.get(name, 0.0))) for name in joint_names]
                motions.append(Motion(MotionType.JOINTS, joints_deg, speed=50.0, blend=0.0))

            program = Program(robot, motions)
            code = program.to_post(metadata={"program_name": "CORE_PATH"})

        except PostProcessError as exc:
            # 后处理失败必须让用户看见，而不是退回注释占位文本
            QtWidgets.QMessageBox.critical(
                self,
                tr("代码生成失败"),
                str(exc) + tr("\n\n可用后处理器：\n  ") + "\n  ".join(list_post_processors()),
            )
            return
        except Exception as exc:
            QtWidgets.QMessageBox.critical(
                self,
                tr("代码生成失败"),
                f"{type(exc).__name__}: {exc}",
            )
            return

        self._show_generated_code(code, robot)

    def _show_generated_code(self, code: str, robot) -> None:
        """展示生成的控制器代码，并询问是否保存到文件。"""
        from adapter.post import resolve_for_brand

        brand = ""
        urdf_path = getattr(self, "urdf_path", "") or ""
        parts = Path(urdf_path).parts
        if "models" in parts and parts.index("models") + 1 < len(parts):
            brand = parts[parts.index("models") + 1]

        pp = resolve_for_brand(brand) if brand else None
        ext = pp.file_extension if pp else "txt"
        lang = pp.language_name if pp else "unknown"

        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle(tr("生成的机器人代码 — {lang}").format(lang=lang))
        dialog.resize(720, 520)

        layout = QtWidgets.QVBoxLayout(dialog)
        info = QtWidgets.QLabel(
            tr("机器人：{name}    目标语言：{lang}    指令数：{n}").format(
                name=robot.name, lang=lang, n=len(code.splitlines())
            )
        )
        layout.addWidget(info)

        editor = QtWidgets.QPlainTextEdit()
        editor.setPlainText(code)
        editor.setReadOnly(True)
        font = QFont("Monospace", 10)
        font.setStyleHint(QFont.Monospace)
        editor.setFont(font)
        layout.addWidget(editor)

        buttons = QtWidgets.QDialogButtonBox()
        save_btn = buttons.addButton(tr("保存为文件…"), QtWidgets.QDialogButtonBox.AcceptRole)
        buttons.addButton(tr("关闭"), QtWidgets.QDialogButtonBox.RejectRole)
        save_btn.clicked.connect(lambda: self._save_generated_code(code, ext, lang, dialog))
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)

        dialog.exec()

    def _save_generated_code(self, code: str, ext: str, lang: str, parent) -> None:
        """把生成的代码写入文件。"""
        default = f"core_path.{ext}"
        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            parent,
            tr("保存 {lang} 代码").format(lang=lang),
            default,
            f"{lang} Files (*.{ext});;Text Files (*.txt);;All Files (*)",
        )
        if file_path:
            try:
                with open(file_path, "w", encoding="utf-8") as fh:
                    fh.write(code)
                self.statusBar().showMessage(
                    tr("已保存 {lang} 代码: {name}").format(lang=lang, name=Path(file_path).name)
                )
            except OSError as exc:
                QtWidgets.QMessageBox.critical(parent, tr("保存失败"), str(exc))
