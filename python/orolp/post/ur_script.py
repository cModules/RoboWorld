from .base import BasePostProcessor, PostConfig, PostProcessError
from ..program import Motion


class URScriptPostProcessor(BasePostProcessor):
    """Universal Robots URScript（Polyscope）后处理器。

    目标控制器：UR CB3 / e-Series（Polyscope）
    线位移单位：**米**；角度单位：**弧度**

    Note:
        URScript 的位姿姿态部分是**旋转向量**（轴角），不是欧拉角，
        因此这里用 :meth:`~orolp.post.base.BasePostProcessor.fmt_pose_rotvec`
        而非 :meth:`~orolp.post.base.BasePostProcessor.fmt_pose`。
    """

    brand = "ur"
    controller = "polyscope"

    #: 默认加速度 (rad/s²)
    DEFAULT_ACCEL = 1.2

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
        return "URScript"

    @property
    def file_extension(self) -> str:
        return "script"

    def _convert_motion(self, motion: Motion) -> dict:
        # 输入 speed 语义为 mm/s（关节运动时按百分比折算），统一转成 m/s
        speed_m_s = motion.speed / 1000.0
        blend_m = motion.blend / 1000.0

        if self.is_joint(motion):
            return {
                "cmd": "movej",
                "args": f"[{self.fmt_joints(motion.target)}]",
                "accel": f"{self.DEFAULT_ACCEL:.2f}",
                "speed": f"{speed_m_s:.4f}",
                "blend": f"{blend_m:.4f}",
            }
        if self.is_linear(motion):
            return {
                "cmd": "movel",
                "args": f"p[{self.fmt_pose_rotvec(motion.target)}]",
                "accel": f"{self.DEFAULT_ACCEL:.2f}",
                "speed": f"{speed_m_s:.4f}",
                "blend": f"{blend_m:.4f}",
            }
        if self.is_circular(motion):
            if motion.via is None:
                raise PostProcessError("URScript 圆弧运动需要提供 via 中间点")
            return {
                "cmd": "movec",
                "via": f"p[{self.fmt_pose_rotvec(motion.via)}]",
                "args": f"p[{self.fmt_pose_rotvec(motion.target)}]",
                "accel": f"{self.DEFAULT_ACCEL:.2f}",
                "speed": f"{speed_m_s:.4f}",
                "blend": f"{blend_m:.4f}",
            }
        raise PostProcessError(f"URScript 不支持的运动类型: {motion.type}")

    def _get_template(self) -> str:
        return """def {{ meta.program_name|default('orolp_prog') }}():
    # OpenRoboOLP v{{ meta.version|default('0.1.0') }} generated URScript
    # Robot: {{ meta.robot_name|default('unknown') }}
    {% for wp in waypoints %}
    {% if wp.cmd == 'movec' %}
    {{ wp.cmd }}({{ wp.via }}, {{ wp.args }}, a={{ wp.accel }}, v={{ wp.speed }}, r={{ wp.blend }})
    {% else %}
    {{ wp.cmd }}({{ wp.args }}, a={{ wp.accel }}, v={{ wp.speed }}, r={{ wp.blend }})
    {% endif %}
    {% endfor %}
end
"""
