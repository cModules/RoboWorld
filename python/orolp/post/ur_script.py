from .base import BasePostProcessor, PostConfig
from .. import _core


class URScriptPostProcessor(BasePostProcessor):
    brand = "ur"
    controller = "polyscope"

    def __init__(self):
        super().__init__(PostConfig(precision=4, speed_unit="rad/s"))

    def _convert_wp(self, wp):
        if wp.type == _core.MoveType.JOINT:
            return {
                "cmd": "movej",
                "args": f"[{self.fmt_joints(wp.target)}], a=1.2, v={wp.speed/100.0:.2f}",
                "blend": f"r={wp.blend/1000.0:.4f}" if wp.blend > 0 else "r=0",
            }
        elif wp.type == _core.MoveType.LINEAR:
            return {
                "cmd": "movel",
                "args": f"p[{self.fmt_pose(wp.target, order='zyx')}], a=1.2, v={wp.speed/1000.0:.4f}",
                "blend": f"r={wp.blend/1000.0:.4f}" if wp.blend > 0 else "r=0",
            }
        return {"cmd": "# unknown", "args": "", "blend": ""}

    def _get_template(self) -> str:
        return """def {{ meta.program_name|default('orolp_prog') }}():
    # OpenRoboOLP generated URScript
    {% for wp in waypoints %}
    {{ wp.cmd }}({{ wp.args }}, {{ wp.blend }})
    {% endfor %}
end
"""
