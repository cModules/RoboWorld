from .base import BasePostProcessor, PostConfig
from .. import _core


class HuiChuanInoTeachPostProcessor(BasePostProcessor):
    brand = "huichuan"
    controller = "inoteach"

    def __init__(self):
        super().__init__(PostConfig(precision=3, speed_unit="%", comment_char="#"))

    def _convert_wp(self, wp):
        if wp.type == _core.MoveType.JOINT:
            return {
                "cmd": "MOVJ",
                "target": self.fmt_joints(wp.target),
                "speed": f"{wp.speed:.1f}",
                "zone": f"PL{int(wp.blend)}" if wp.blend > 0 else "PL0",
            }
        elif wp.type == _core.MoveType.LINEAR:
            return {
                "cmd": "MOVL",
                "target": self.fmt_pose(wp.target, order="xyz"),
                "speed": f"{wp.speed:.1f}",
                "zone": f"PL{int(wp.blend)}" if wp.blend > 0 else "PL0",
            }
        return {"cmd": "# UNKNOWN", "target": "", "speed": "", "zone": ""}

    def _get_template(self) -> str:
        return """# OpenRoboOLP generated InoTeach program
# Robot: {{ meta.robot_name|default('Huichuan') }}
# Version: {{ meta.version|default('0.1.0') }}

PROGRAM {{ meta.program_name|default('OROLP_PROG') }}
    {% for wp in waypoints %}
    {{ wp.cmd }} {{ wp.target }}, Speed={{ wp.speed }}, {{ wp.zone }}
    {% endfor %}
END_PROGRAM
"""
