from .base import BasePostProcessor, PostConfig, PostProcessError
from support.i18n import tr
from service.program import Motion


class JAKAZuPostProcessor(BasePostProcessor):
    """JAKA（节卡）脚本后处理器。

    目标控制器：JAKA Zu 系列控制器
    线位移单位：**米**；角度单位：**弧度**

    Warning:
        指令语法依据 JAKA 公开脚本资料整理，**未经真实控制器联机验证**。
        JAKA 官方未提供明确的开源许可与完整指令手册，使用前请用实际设备
        核对指令名与参数顺序，尤其是 ``movec`` 的模态参数。

    Note:
        与 URScript 一样，JAKA 的位姿姿态部分为旋转向量（轴角），不是欧拉角。
        因此姿态输出走 :meth:`~adapter.post.base.BasePostProcessor.fmt_pose_rotvec`。

        本项目**不打包 JAKA 的 CAD 网格**（其 GitHub 仓库缺少明确 LICENSE），
        机器人几何为程序化生成的示意模型，此处同理只涉及开放的参数层面。
    """

    brand = "jaka"
    controller = "zu"

    #: 默认加速度 (rad/s²)
    DEFAULT_ACCEL = 1.0

    def __init__(self):
        super().__init__(
            PostConfig(
                precision=4,
                speed_unit="mm/s",
                comment_char="#",
                linear_unit="m",
                angle_unit="rad",
            )
        )

    @property
    def language_name(self) -> str:
        return "JAKA Script"

    @property
    def file_extension(self) -> str:
        return "jks"

    def _convert_motion(self, motion: Motion) -> dict:
        speed_m_s = motion.speed / 1000.0

        if self.is_joint(motion):
            return {
                "cmd": "movej",
                "args": f"[{self.fmt_joints(motion.target)}]",
                "accel": f"{self.DEFAULT_ACCEL:.2f}",
                "speed": f"{speed_m_s:.4f}",
            }
        if self.is_linear(motion):
            return {
                "cmd": "movel",
                "args": f"[{self.fmt_pose_rotvec(motion.target)}]",
                "accel": f"{self.DEFAULT_ACCEL:.2f}",
                "speed": f"{speed_m_s:.4f}",
            }
        if self.is_circular(motion):
            if motion.via is None:
                raise PostProcessError(tr("JAKA 圆弧运动需要提供 via 中间点"))
            return {
                "cmd": "movec",
                "via": f"[{self.fmt_pose_rotvec(motion.via)}]",
                "args": f"[{self.fmt_pose_rotvec(motion.target)}]",
                "accel": f"{self.DEFAULT_ACCEL:.2f}",
                "speed": f"{speed_m_s:.4f}",
            }
        raise PostProcessError(tr("JAKA 不支持的运动类型: {type}").format(type=motion.type))

    def _get_template(self) -> str:
        note = tr("# NOTE: 语法未经真实控制器验证，使用前请核对")
        header = (
            "# RobotWorld v{{ meta.version|default('0.1.0') }} generated JAKA script\n"
            "# Robot: {{ meta.robot_name|default('unknown') }}\n"
            f"# {note}\n"
            "\n"
        )
        return header + """{% for wp in waypoints %}
{% if wp.cmd in ['SETDO', 'WAITDI', 'WAITTIME', 'ARCSTART', 'ARCEND', 'SPRAYON', 'SPRAYOFF', 'FORCEMODE', 'FORCEMODEEND'] %}
{{ wp.text }}
{% elif wp.cmd == 'movec' %}
{{ wp.cmd }}({{ wp.via }}, {{ wp.args }}, {{ wp.speed }}, {{ wp.accel }})
{% else %}
{{ wp.cmd }}({{ wp.args }}, {{ wp.speed }}, {{ wp.accel }})
{% endif %}
{% endfor %}
"""
