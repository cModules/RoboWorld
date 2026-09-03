from .base import BasePostProcessor, PostConfig, PostProcessError
from ..program import Motion


class HuiChuanInoTeachPostProcessor(BasePostProcessor):
    """汇川 InoTeach 后处理器。

    目标控制器：汇川 InoTeach 系列
    线位移单位：毫米；角度单位：度
    """

    brand = "huichuan"
    controller = "inoteach"

    def __init__(self):
        super().__init__(
            PostConfig(
                precision=3,
                speed_unit="%",
                comment_char="#",
                linear_unit="mm",
                angle_unit="deg",
            )
        )

    @property
    def language_name(self) -> str:
        return "InoTeach"

    @property
    def file_extension(self) -> str:
        return "prg"

    def _convert_motion(self, motion: Motion) -> dict:
        if self.is_joint(motion):
            return {
                "cmd": "MOVJ",
                "target": self.fmt_joints(motion.target),
                "speed": f"{motion.speed:.1f}",
                "zone": f"PL{int(round(motion.blend))}" if motion.blend > 0 else "PL0",
            }
        if self.is_linear(motion):
            return {
                "cmd": "MOVL",
                "target": self.fmt_pose(motion.target, order="zyx"),
                "speed": f"{motion.speed:.1f}",
                "zone": f"PL{int(round(motion.blend))}" if motion.blend > 0 else "PL0",
            }
        if self.is_circular(motion):
            if motion.via is None:
                raise PostProcessError("InoTeach 圆弧运动需要提供 via 中间点")
            return {
                "cmd": "MOVC",
                "via": self.fmt_pose(motion.via, order="zyx"),
                "target": self.fmt_pose(motion.target, order="zyx"),
                "speed": f"{motion.speed:.1f}",
                "zone": f"PL{int(round(motion.blend))}" if motion.blend > 0 else "PL0",
            }
        raise PostProcessError(f"InoTeach 不支持的运动类型: {motion.type}")

    def _get_template(self) -> str:
        return """# RobotWorld generated InoTeach program
# Robot: {{ meta.robot_name|default('Huichuan') }}
# Version: {{ meta.version|default('0.1.0') }}

PROGRAM {{ meta.program_name|default('OROLP_PROG') }}
    {% for wp in waypoints %}
    {% if wp.cmd == 'MOVC' %}
    {{ wp.cmd }} {{ wp.via }}, {{ wp.target }}, Speed={{ wp.speed }}, {{ wp.zone }}
    {% else %}
    {{ wp.cmd }} {{ wp.target }}, Speed={{ wp.speed }}, {{ wp.zone }}
    {% endif %}
    {% endfor %}
END_PROGRAM
"""
