from .base import BasePostProcessor, PostConfig, PostProcessError
from support.i18n import tr
from service.program import Motion


class ESTUNERPostProcessor(BasePostProcessor):
    """ESTUN（埃斯顿）ER 系列后处理器。

    目标控制器：ESTUN ER 系列控制器
    线位移单位：毫米；角度单位：度

    Warning:
        指令语法依据 ESTUN 公开资料整理，**未经真实控制器联机验证**。
        国产控制器各固件版本间指令细节差异较大，使用前请用实际设备核对。
    """

    brand = "estun"
    controller = "er"

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
        return "ESTUN ER"

    @property
    def file_extension(self) -> str:
        return "prg"

    def _convert_motion(self, motion: Motion) -> dict:
        if self.is_joint(motion):
            # 关节目标写成 J1=.. J2=.. 形式，便于现场人工核对轴序
            joints = [float(v) for v in motion.target]
            parts = ", ".join(
                f"J{i + 1}={v:.{self.config.precision}f}" for i, v in enumerate(joints)
            )
            return {
                "cmd": "MOVJ",
                "target": parts,
                "speed": f"V={motion.speed:.1f}",
                "zone": (
                    f"CNT={motion.blend:.{self.config.precision}f}" if motion.blend > 0 else "CNT=0"
                ),
            }
        if self.is_linear(motion):
            x, y, z, rx, ry, rz = (
                float(v) for v in self.fmt_pose(motion.target, order="zyx").split(", ")
            )
            return {
                "cmd": "MOVL",
                "target": f"X={x:.{self.config.precision}f}, Y={y:.{self.config.precision}f}, "
                f"Z={z:.{self.config.precision}f}, RX={rx:.{self.config.precision}f}, "
                f"RY={ry:.{self.config.precision}f}, RZ={rz:.{self.config.precision}f}",
                "speed": f"V={motion.speed:.1f}",
                "zone": (
                    f"CNT={motion.blend:.{self.config.precision}f}" if motion.blend > 0 else "CNT=0"
                ),
            }
        if self.is_circular(motion):
            if motion.via is None:
                raise PostProcessError(tr("ESTUN 圆弧运动需要提供 via 中间点"))
            v = [float(t) for t in self.fmt_pose(motion.via, order="zyx").split(", ")]
            t = [float(x) for x in self.fmt_pose(motion.target, order="zyx").split(", ")]
            p = self.config.precision
            return {
                "cmd": "MOVC",
                "via": f"X={v[0]:.{p}f}, Y={v[1]:.{p}f}, Z={v[2]:.{p}f}, "
                f"RX={v[3]:.{p}f}, RY={v[4]:.{p}f}, RZ={v[5]:.{p}f}",
                "target": f"X={t[0]:.{p}f}, Y={t[1]:.{p}f}, Z={t[2]:.{p}f}, "
                f"RX={t[3]:.{p}f}, RY={t[4]:.{p}f}, RZ={t[5]:.{p}f}",
                "speed": f"V={motion.speed:.1f}",
                "zone": f"CNT={motion.blend:.{p}f}" if motion.blend > 0 else "CNT=0",
            }
        raise PostProcessError(tr("ESTUN 不支持的运动类型: {type}").format(type=motion.type))

    def _get_template(self) -> str:
        note = tr("# NOTE: 语法未经真实控制器验证，使用前请核对")
        header = (
            "# RoboWorld v{{ meta.version|default('0.1.0') }} generated ESTUN program\n"
            "# Robot: {{ meta.robot_name|default('unknown') }}\n"
            f"# {note}\n"
            "\n"
        )
        return header + """PROGRAM {{ meta.program_name|default('CORE_PROG') }}
    {% for wp in waypoints %}
    {% if wp.cmd in ['SETDO', 'WAITDI', 'WAITTIME', 'ARCSTART', 'ARCEND', 'SPRAYON', 'SPRAYOFF', 'FORCEMODE', 'FORCEMODEEND'] %}
    {{ wp.text }}
    {% elif wp.cmd == 'MOVC' %}
    {{ wp.cmd }} {{ wp.via }} {{ wp.target }} {{ wp.speed }} {{ wp.zone }}
    {% else %}
    {{ wp.cmd }} {{ wp.target }} {{ wp.speed }} {{ wp.zone }}
    {% endif %}
    {% endfor %}
END PROGRAM
"""
