from .base import BasePostProcessor, PostConfig, PostProcessError
from ..program import Motion


class FANUCTPPostProcessor(BasePostProcessor):
    """FANUC TP 程序（R-30iB / R-30iB Plus）后处理器。

    目标控制器：FANUC R-30iB 系列
    线位移单位：毫米；角度单位：度

    Note:
        TP 是 FANUC 示教器上的文本格式。真实 ``.ls`` 文件还包含 ``/POS`` 段的
        点位定义与 ``/APPL`` 应用数据，本后处理器只生成 ``/MN`` 主体段——
        导入示教器时会自动补齐点位表。
    """

    brand = "fanuc"
    controller = "r30ib"

    def __init__(self):
        super().__init__(
            PostConfig(
                precision=2,
                speed_unit="%",
                comment_char="/",
                linear_unit="mm",
                angle_unit="deg",
            )
        )

    @property
    def language_name(self) -> str:
        return "TP"

    @property
    def file_extension(self) -> str:
        return "ls"

    def _convert_motion(self, motion: Motion) -> dict:
        if self.is_joint(motion):
            return {
                "cmd": "J",
                "target": self.fmt_joints(motion.target),
                "speed": f"{motion.speed:.0f}%",
                "zone": f"CNT{int(round(motion.blend))}" if motion.blend > 0 else "FINE",
            }
        if self.is_linear(motion):
            return {
                "cmd": "L",
                "target": self.fmt_pose(motion.target, order="zyx"),
                "speed": f"{motion.speed:.0f}mm/sec",
                "zone": f"CNT{int(round(motion.blend))}" if motion.blend > 0 else "FINE",
            }
        if self.is_circular(motion):
            if motion.via is None:
                raise PostProcessError("FANUC TP 圆弧运动需要提供 via 中间点")
            return {
                "cmd": "C",
                "via": self.fmt_pose(motion.via, order="zyx"),
                "target": self.fmt_pose(motion.target, order="zyx"),
                "speed": f"{motion.speed:.0f}mm/sec",
                "zone": f"CNT{int(round(motion.blend))}" if motion.blend > 0 else "FINE",
            }
        raise PostProcessError(f"FANUC TP 不支持的运动类型: {motion.type}")

    def _get_template(self) -> str:
        return """/PROG  {{ meta.program_name|default('OROLP_PROG') }}
/ATTR
COMMENT = "OpenRoboOLP v{{ meta.version|default('0.1.0') }}";
/MN
    {% for wp in waypoints %}
    {% if wp.cmd == 'C' %}
    {{ wp.cmd }} {{ wp.via }} {{ wp.target }} {{ wp.speed }} {{ wp.zone }} ;
    {% else %}
    {{ wp.cmd }} {{ wp.target }} {{ wp.speed }} {{ wp.zone }} ;
    {% endif %}
    {% endfor %}
/POS
/END
"""
