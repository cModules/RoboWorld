"""属性面板的 UI 骨架与规格填充（ADR-002：``DevicePropertyPanel`` 拆出的构建层）。

归属：App / GUI 层 ``app.views``。把 140 行的 ``_setup_ui`` 与规格填充从
面板类里剥离，是为了让 ``property_panel.py`` 落在 CODING_RULES §3.1 的 GUI 组件
阈值（300 行）内——迁进来的是既有代码，不是新功能，拆的目的是**不再制造新的
超限文件**（AGENTS.md #26）。

形式是模块级函数而非类方法：它们只往传入的 ``panel`` 上挂控件，不持有独立
状态，因此**行为与原先的 ``self._setup_ui()`` 完全一致**（ADR-002 只搬不改）。
"""

from PySide6 import QtWidgets
from PySide6.QtGui import QFont

from support.i18n import tr


def build_ui(panel) -> None:
    """构建「属性」面板的全部控件并挂到 ``panel`` 上。

    Args:
        panel: :class:`~app.views.property_panel.DevicePropertyPanel` 实例。
            函数内以 ``panel.xxx`` 取代原先的 ``self.xxx``，控件归属不变。
    """
    layout = QtWidgets.QVBoxLayout(panel)
    layout.setContentsMargins(5, 5, 5, 5)

    # ---- 对象属性（选中场景对象时显示，默认隐藏）----
    # 含：信息（对象名/类型 + 机器人规格）、位置（基座位姿）、参数
    panel.object_group = QtWidgets.QGroupBox(tr("对象属性"))
    obj_layout = QtWidgets.QVBoxLayout(panel.object_group)

    # 信息（对象名/类型 + 机器人规格）
    panel.info_group = QtWidgets.QGroupBox(tr("信息"))
    info_layout = QtWidgets.QFormLayout(panel.info_group)
    panel.obj_name_label = QtWidgets.QLabel(tr("—"))
    panel.obj_type_label = QtWidgets.QLabel(tr("—"))
    info_layout.addRow(tr("名称:"), panel.obj_name_label)
    info_layout.addRow(tr("类型:"), panel.obj_type_label)

    # 挂载（把对象挂到机器人法兰或另一对象下；机器人自身不可挂载）
    panel.obj_mount_widget = QtWidgets.QWidget()
    mount_layout = QtWidgets.QFormLayout(panel.obj_mount_widget)
    mount_layout.setContentsMargins(0, 4, 0, 4)
    panel.obj_parent_combo = QtWidgets.QComboBox()
    panel.obj_parent_combo.setToolTip(
        tr(
            "挂到「机器人」＝ 安装到法兰，工具将跟随机器人运动；\n"
            "挂到其它对象 ＝ 保持当前位置，仅建立随动关系。"
        )
    )
    panel.obj_parent_combo.currentIndexChanged.connect(panel._on_obj_parent_edited)
    mount_layout.addRow(tr("挂载到:"), panel.obj_parent_combo)

    # 装夹 / 解除装夹：与下拉等价，但不用去翻选项——法兰装夹是高频操作
    panel.mount_btn_row = QtWidgets.QWidget()
    btn_layout = QtWidgets.QHBoxLayout(panel.mount_btn_row)
    btn_layout.setContentsMargins(0, 2, 0, 0)
    panel.btn_mount_to_robot = QtWidgets.QPushButton(tr("装夹到末端"))
    panel.btn_unmount = QtWidgets.QPushButton(tr("解除装夹"))
    panel.btn_mount_to_robot.setToolTip(
        tr("吸附到机器人法兰原点，工具坐标系与法兰对齐并随机器人运动")
    )
    panel.btn_unmount.setToolTip(tr("解除挂载并保持当前世界位姿"))
    panel.btn_mount_to_robot.clicked.connect(panel._on_mount_to_robot)
    panel.btn_unmount.clicked.connect(panel._on_unmount)
    btn_layout.addWidget(panel.btn_mount_to_robot)
    btn_layout.addWidget(panel.btn_unmount)
    mount_layout.addRow(panel.mount_btn_row)
    info_layout.addRow(panel.obj_mount_widget)

    # 机器人规格（仅机器人对象可见）
    panel.robot_spec_widget = QtWidgets.QWidget()
    spec_layout = QtWidgets.QFormLayout(panel.robot_spec_widget)
    spec_layout.setContentsMargins(0, 4, 0, 4)
    panel.name_label = QtWidgets.QLabel(tr("—"))
    panel.manufacturer_label = QtWidgets.QLabel(tr("—"))
    panel.type_label = QtWidgets.QLabel(tr("—"))
    panel.payload_label = QtWidgets.QLabel(tr("—"))
    panel.reach_label = QtWidgets.QLabel(tr("—"))
    panel.repeat_label = QtWidgets.QLabel(tr("—"))
    panel.dof_label = QtWidgets.QLabel(tr("—"))
    panel.controller_label = QtWidgets.QLabel(tr("—"))
    spec_layout.addRow(tr("型号:"), panel.name_label)
    spec_layout.addRow(tr("厂商:"), panel.manufacturer_label)
    spec_layout.addRow(tr("类型:"), panel.type_label)
    spec_layout.addRow(tr("额定负载:"), panel.payload_label)
    spec_layout.addRow(tr("最大臂展:"), panel.reach_label)
    spec_layout.addRow(tr("重复定位精度:"), panel.repeat_label)
    spec_layout.addRow(tr("轴数:"), panel.dof_label)
    spec_layout.addRow(tr("控制器:"), panel.controller_label)
    info_layout.addRow(panel.robot_spec_widget)
    obj_layout.addWidget(panel.info_group)

    # 型号备注（数据来源、参数核验状态等）
    panel.note_label = QtWidgets.QLabel("")
    panel.note_label.setWordWrap(True)
    panel.note_label.setStyleSheet("color: #8a6d3b; padding: 2px;")
    note_font = QFont()
    note_font.setPointSize(8)
    panel.note_label.setFont(note_font)
    panel.note_label.hide()
    obj_layout.addWidget(panel.note_label)

    # 位置（虚拟环境坐标，可编辑）—— 6DOF：XYZ + 绕XYZ
    panel.pose_grp = QtWidgets.QGroupBox(tr("位置"))
    pose_layout = QtWidgets.QFormLayout(panel.pose_grp)
    for key, label, lo, hi, step in (
        ("x", tr("X (毫米)"), -20000, 20000, 10),
        ("y", tr("Y (毫米)"), -20000, 20000, 10),
        ("z", tr("Z (毫米)"), -5000, 10000, 10),
        ("rx", tr("绕 X (°)"), -180, 180, 5),
        ("ry", tr("绕 Y (°)"), -180, 180, 5),
        ("rz", tr("绕 Z (°)"), -360, 360, 5),
    ):
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setDecimals(1)
        spin.setSingleStep(step)
        spin.setValue(0.0)
        spin.valueChanged.connect(panel._on_obj_pose_edited)
        panel.obj_pose_widgets[key] = spin
        pose_layout.addRow(label, spin)
    obj_layout.addWidget(panel.pose_grp)

    # 参数（设备模型参数化：只放尺寸参数，轴不在这里）
    panel.obj_params_group = QtWidgets.QGroupBox(tr("参数"))
    panel.obj_params_layout = QtWidgets.QVBoxLayout(panel.obj_params_group)
    panel.obj_params_layout.setContentsMargins(4, 4, 4, 4)
    panel.obj_params_layout.setSpacing(4)
    obj_layout.addWidget(panel.obj_params_group)

    # 关节（设备关节姿态调节：机器人 6 轴 / 多轴设备的轴）
    panel.obj_joint_group = QtWidgets.QGroupBox(tr("关节"))
    panel.obj_joint_layout = QtWidgets.QVBoxLayout(panel.obj_joint_group)
    panel.obj_joint_layout.setContentsMargins(4, 4, 4, 4)
    panel.obj_joint_layout.setSpacing(4)
    obj_layout.addWidget(panel.obj_joint_group)

    # 让上方各分组保持自然高度。滚动区会把多余高度分给 QGroupBox，
    # 导致「信息」这类内容少的分组被拉高、出现大片空白。
    # （注意与外层的区别：外层 layout 加 stretch 会抢滚动区高度，不能加。）
    obj_layout.addStretch(1)

    panel.object_scroll = QtWidgets.QScrollArea()
    panel.object_scroll.setWidgetResizable(True)
    panel.object_scroll.setFrameStyle(QtWidgets.QFrame.NoFrame)
    panel.object_scroll.setWidget(panel.object_group)
    panel.object_scroll.hide()
    # 这里不能加 addStretch：它会与 object_scroll 平分剩余高度（各占一半），
    # 把滚动可视区压到只剩约 350px，导致底部的「参数/关节控制」被挤到
    # 视口以下完全看不见。让 object_scroll 独占全部剩余空间即可把日志顶到底部。
    layout.addWidget(panel.object_scroll, 1)

    log_group = QtWidgets.QGroupBox(tr("日志"))
    log_group.setMaximumHeight(150)  # 限制日志高度，给上方属性区留出可视空间
    log_layout = QtWidgets.QVBoxLayout(log_group)
    panel.log_text = QtWidgets.QTextEdit()
    panel.log_text.setReadOnly(True)
    panel.log_text.setFont(QFont("Consolas", 9))
    panel.log_text.setStyleSheet("background-color: #1a1a1a; color: #ffffff;")
    log_layout.addWidget(panel.log_text)
    layout.addWidget(log_group)


def fill_robot_spec(panel, renderer, entry) -> None:
    """把**指定那台**的规格填进信息区（型号/厂商/负载/臂展…）。

    供两处调用：:meth:`DevicePropertyPanel.update_robot_info`（载入时）与
    :meth:`DevicePropertyPanel.set_object`（选中机器人时按名字回显）——后者没有
    这条，多机时规格块永远是最后加载那台的（2026-09-21 用户反馈）。

    Args:
        panel: 属性面板实例（``self`` 的替代）。
        renderer: 该机器人的渲染器。
        entry: 机器人库条目；``None`` 时只回显 URDF 里的机器人名。
    """
    revolute = [j for j in renderer.joints.values() if j["type"] == "revolute"]
    panel.dof_label.setText(str(len(revolute)))
    if entry is not None:
        panel.name_label.setText(entry.model_name)
        panel.manufacturer_label.setText(entry.brand_display)
        type_map = {"industrial": tr("工业臂"), "cobot": tr("协作臂"), "scara": "SCARA"}
        panel.type_label.setText(type_map.get(entry.robot_type, entry.robot_type))
        panel.payload_label.setText(
            f"{entry.payload_kg:g} kg" if entry.payload_kg is not None else tr("—")
        )
        panel.reach_label.setText(
            f"{entry.reach_mm:g} mm" if entry.reach_mm is not None else tr("—")
        )
        panel.repeat_label.setText(
            f"±{entry.repeatability_mm:g} mm" if entry.repeatability_mm is not None else tr("—")
        )
        panel.controller_label.setText(entry.controller or tr("—"))
        if entry.note:
            prefix = "" if entry.verified else tr("⚠ 参数未核验 — ")
            panel.note_label.setText(prefix + entry.note)
            panel.note_label.show()
        else:
            panel.note_label.hide()
    else:
        panel.name_label.setText(renderer.robot_name)
        panel.manufacturer_label.setText(tr("—"))
        panel.type_label.setText(tr("—"))
        panel.payload_label.setText(tr("—"))
        panel.reach_label.setText(tr("—"))
        panel.repeat_label.setText(tr("—"))
        panel.controller_label.setText(tr("—"))
        panel.note_label.hide()
