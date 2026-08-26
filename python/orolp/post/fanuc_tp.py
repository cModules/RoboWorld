from .base import BasePostProcessor, PostConfig
from .. import _core


class FANUCTPPostProcessor(BasePostProcessor):
    brand = "fanuc"
    controller = "r30ib"

    def __init__(self):
        super().__init__(PostConfig(precision=2, speed_unit="%", comment_char="/"))

    def _convert_wp(self, wp):
        if wp.type == _core.MoveType.JOINT:
            return {
                "cmd": "J",
                "target": self.fmt_joints(wp.target),
                "speed": f"{wp.speed:.0f}%",
                "zone": f"CNT{int(wp.blend)}" if wp.blend > 0 else "FINE",
            }
        elif wp.type == _core.MoveType.LINEAR:
            return {
                "cmd": "L",
                "target": self.fmt_pose(wp.target, order="xyz"),
                "speed": f"{wp.speed:.0f}mm/sec",
                "zone": f"CNT{int(wp.blend)}" if wp.blend > 0 else "FINE",
            }
        return {"cmd": "/ UNKNOWN", "target": "", "speed": "", "zone": ""}

    def _get_template(self) -> str:
        return """/PROG  {{ meta.program_name|default('OROLP_PROG') }}
/ATTR
COMMENT = "OpenRoboOLP v{{ meta.version|default('0.1.0') }}";
/MN
    {% for wp in waypoints %}
    {{ wp.cmd }} {{ wp.target }} {{ wp.speed }} {{ wp.zone }} ;
    {% endfor %}
/POS
/END
"""
