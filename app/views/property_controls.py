"""属性面板的动态控件行：参数行 / 机器人关节滑块 / 设备轴滑块。

归属：App / GUI 层 ``app.views``。以 **mixin** 形式供
:class:`~app.views.property_panel.DevicePropertyPanel` 继承——拆出来的目的
只是让面板本体落在 CODING_RULES §3.1 的 GUI 组件阈值（300 行）内，**不是**引入
新抽象：mixin 里的方法只读写面板自身的属性，行为与原先写在类里完全一致
（ADR-002：搬迁只换位置、不改行为）。

覆盖三类动态行：

* 参数行——组件尺寸参数（轴不在这里，见下）；
* 关节滑块——机器人可动轴，按运动学顺序；
* 轴滑块——多轴设备的轴，值写回 ``obj.params`` 走既有几何重建通路。
"""

import math

from PySide6 import QtWidgets
from PySide6.QtCore import Qt

from support.i18n import tr


class ControlRowsMixin:
    """属性面板的「参数 / 关节 / 轴」三块动态行与回调。

    只被 :class:`DevicePropertyPanel` 继承；方法内用到的属性
    （``obj_params_layout`` / ``obj_joint_layout`` / ``sliders`` /
    ``axis_sliders`` …）都在面板的 ``__init__`` 里建好。
    """

    def _clear_layout(self, layout) -> None:
        """清空任意布局内的所有控件与嵌套布局（「参数」与「关节」共用）。"""
        while layout.count():
            item = layout.takeAt(0)
            if item is None:
                continue
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
            else:
                sub = item.layout()
                if sub is not None:
                    self._clear_layout(sub)
                    sub.deleteLater()

    def _clear_params_layout(self) -> None:
        """清空参数面板。"""
        self._clear_layout(self.obj_params_layout)

    def _rebuild_param_rows(self, spec, params: dict) -> None:
        """根据组件 ParamSpec 重建参数编辑行。"""
        self._clear_params_layout()
        self.obj_param_widgets.clear()
        self.sliders.clear()
        self.value_labels.clear()

        if spec is None:
            lbl = QtWidgets.QLabel(tr("（无可调参数）"))
            lbl.setStyleSheet("color:#7f8192; font-size:11px;")
            self.obj_params_layout.addWidget(lbl)
            return

        # 用 plain_params()：轴由「关节」面板负责，这里不再重复出现
        for p in spec.plain_params():
            row = QtWidgets.QHBoxLayout()
            row.setSpacing(8)
            lbl = QtWidgets.QLabel(p.label)
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(p.minimum, p.maximum)
            spin.setDecimals(1)
            spin.setSingleStep(p.step)
            if p.unit:
                spin.setSuffix(f" {p.unit}")
            spin.blockSignals(True)
            spin.setValue(float(params.get(p.name, p.default)))
            spin.blockSignals(False)
            spin.valueChanged.connect(lambda v, n=p.name: self._on_obj_param_edited(n, v))
            self.obj_param_widgets[p.name] = spin
            row.addWidget(lbl)
            row.addWidget(spin, 1)
            self.obj_params_layout.addLayout(row)

    def set_renderer_provider(self, fn) -> None:
        """注入「按机器人名取渲染器」的回调（多机必需）。

        Args:
            fn: ``Callable[[str], RobotRenderer]``；主窗口传
                ``gl_widget.renderer_of``

        Note:
            不注入的话，本面板只能沿用 :attr:`_renderer`（由
            :meth:`update_robot_info` 写入，通常是**最后加载**那台）。于是选中
            A 时滑块仍是 B 的关节组——拖动会把值写进别的机器人，两台一起动。
        """
        self.renderer_provider = fn

    def _renderer_for_robot(self, name: str):
        """按名字取该机器人的渲染器；取不到时退回 :attr:`_renderer`。"""
        if self.renderer_provider is not None:
            try:
                return self.renderer_provider(name)
            except Exception:
                return None
        return self._renderer

    def _setup_joint_controls(self, renderer, robot_name=None) -> None:
        """为机器人在「关节」面板内构建 6 轴滑块。

        Args:
            renderer: 渲染器（取关节清单与初值）
            robot_name: 这台机器人的**名字**。滑块回调要记下来一起发出去——
                否则 MainWindow 无从判断该把关节值写到哪一台
        """
        self._renderer = renderer
        self._active_robot = robot_name
        self._clear_layout(self.obj_joint_layout)
        self.axis_sliders.clear()
        self.axis_value_labels.clear()
        self.sliders.clear()
        self.value_labels.clear()

        # 按运动学顺序取可动关节：不依赖 link 名匹配，顺序也与 FK/IK 一致
        order = list(renderer.actuated_names()) or list(renderer.joints.keys())
        joint_items = [
            (i, n, renderer.joints[n]) for i, n in enumerate(order) if n in renderer.joints
        ]

        for _, name, joint in joint_items:
            lo = math.degrees(joint["limit"]["lower"])
            hi = math.degrees(joint["limit"]["upper"])

            # 紧凑单行「名称 | 滑块 | 当前值」：
            # 原来是「名称/值 + 滑块 + 上下限」三行约 72px，6 个共 472px，
            # 会把「参数」整块顶出可视区；压成一行约 28px，6 个共约 180px。
            row = QtWidgets.QHBoxLayout()
            row.setSpacing(6)
            row.setContentsMargins(2, 1, 2, 1)

            label = QtWidgets.QLabel(name)
            label.setFixedWidth(58)
            label.setStyleSheet("font-size:11px;")

            slider = QtWidgets.QSlider(Qt.Horizontal)
            slider.setMinimum(int(lo * 100))
            slider.setMaximum(int(hi * 100))
            init_deg = math.degrees(renderer.joint_values.get(name, 0.0))
            slider.setValue(int(init_deg * 100))
            slider.setFixedHeight(18)
            slider.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            slider.setToolTip(tr("范围 {lo}° ~ {hi}°").format(lo=f"{lo:.1f}", hi=f"{hi:.1f}"))
            slider.valueChanged.connect(lambda v, n=name: self._on_joint_slider_changed(n, v))
            self.sliders[name] = slider

            val = QtWidgets.QLabel(f"{init_deg:.1f}°")
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val.setFixedWidth(48)
            val.setStyleSheet("font-size:11px;")
            self.value_labels[name] = val

            row.addWidget(label)
            row.addWidget(slider, 1)
            row.addWidget(val)
            self.obj_joint_layout.addLayout(row)

        if not joint_items:
            lbl = QtWidgets.QLabel(tr("（该设备无关节）"))
            lbl.setStyleSheet("color:#7f8192; font-size:11px;")
            self.obj_joint_layout.addWidget(lbl)

    def _rebuild_axis_rows(self, spec, obj) -> None:
        """为多轴设备在「关节」面板构建轴滑块。

        轴值写回 ``obj.params``，由既有的参数驱动几何通路重建几何，
        因此不需要任何渲染侧改动。
        """
        self.sliders.clear()
        self.value_labels.clear()
        self.axis_sliders.clear()
        self.axis_value_labels.clear()
        self._clear_layout(self.obj_joint_layout)

        axes = tuple(getattr(spec, "axes", ()) or ()) if spec is not None else ()
        if not axes:
            lbl = QtWidgets.QLabel(tr("（该设备无关节）"))
            lbl.setStyleSheet("color:#7f8192; font-size:11px;")
            self.obj_joint_layout.addWidget(lbl)
            return

        for axis in axes:
            row = QtWidgets.QHBoxLayout()
            row.setSpacing(6)
            row.setContentsMargins(2, 1, 2, 1)

            label = QtWidgets.QLabel(axis.label)
            label.setFixedWidth(58)
            label.setStyleSheet("font-size:11px;")

            init = float(obj.params.get(axis.param, 0.0))
            slider = QtWidgets.QSlider(Qt.Horizontal)
            slider.setMinimum(int(axis.minimum * 100))
            slider.setMaximum(int(axis.maximum * 100))
            slider.setValue(int(init * 100))
            slider.setFixedHeight(18)
            slider.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
            slider.setToolTip(
                tr("范围 {lo} ~ {hi} {unit}").format(
                    lo=f"{axis.minimum:g}", hi=f"{axis.maximum:g}", unit=axis.unit
                )
            )
            slider.valueChanged.connect(lambda v, a=axis: self._on_axis_slider_changed(a, v))
            self.axis_sliders[axis.name] = slider

            val = QtWidgets.QLabel(f"{init:g}{axis.unit}")
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val.setFixedWidth(48)
            val.setStyleSheet("font-size:11px;")
            self.axis_value_labels[axis.name] = val

            row.addWidget(label)
            row.addWidget(slider, 1)
            row.addWidget(val)
            self.obj_joint_layout.addLayout(row)

    def _on_axis_slider_changed(self, axis, value) -> None:
        """轴值变化：写回 obj.params，触发几何重建。"""
        v = value / 100.0
        if axis.name in self.axis_value_labels:
            self.axis_value_labels[axis.name].setText(f"{v:g}{axis.unit}")
        if not self.current_object or self.scene is None:
            return
        obj = self.scene.objects.get(self.current_object)
        if obj is None:
            return
        obj.params[axis.param] = float(v)
        self.objectChanged.emit()

    def _on_joint_slider_changed(self, name, value):
        angle = value / 100.0
        if name in self.value_labels:
            self.value_labels[name].setText(f"{angle:.1f}°")
        # 带上所属机器人名：多机时 MainWindow 靠它定位该写哪台渲染器
        self.jointChanged.emit(self._active_robot or "", name, math.radians(angle))

    def update_joint_displays(self, joint_values):
        """把关节值回显到滑块与数值标签（拖动之外的值来源，如回放）。"""
        for name, value in joint_values.items():
            if name in self.value_labels:
                self.value_labels[name].setText(f"{math.degrees(value):.1f}°")
            if name in self.sliders:
                self.sliders[name].blockSignals(True)
                self.sliders[name].setValue(int(math.degrees(value) * 100))
                self.sliders[name].blockSignals(False)
