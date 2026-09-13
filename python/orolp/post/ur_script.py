from typing import Dict

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

    # ------------------------- IO 指令（URScript） ------------------------- #

    def format_setdo(self, signal: str, value: bool, io_map: Dict[str, str]) -> str:
        """``set_digital_out(n, True)``。

        URScript 按**数字量序号**寻址，故需把 ``Q0.1`` 换算成序号。
        """
        addr = self.resolve_signal(signal, io_map)
        idx = self.io_index(addr)
        level = "True" if value else "False"
        if idx is None:
            return f"# TODO: 信号 {signal} 地址 {addr} 无法解析为数字量序号，需人工改为 set_digital_out(n, {level})"
        return f"set_digital_out({idx}, {level})"

    def format_waitdi(
        self,
        signal: str,
        value: bool,
        timeout: float,
        io_map: Dict[str, str],
    ) -> str:
        """``wait_digital_in(n, True)``。

        URScript 的 ``wait_digital_in`` **不接受超时**。给了超时就补注释提示
        需要自行加看门狗，不能假装实现了。
        """
        addr = self.resolve_signal(signal, io_map)
        idx = self.io_index(addr)
        if idx is None:
            return f"# TODO: 信号 {signal} 地址 {addr} 无法解析为数字量序号，需人工改为 wait_digital_in(n)"
        level = "True" if value else "False"
        if timeout and timeout > 0:
            return f"wait_digital_in({idx}, {level})  # TODO: 超时 {timeout:.2f}s 需自行实现看门狗"
        return f"wait_digital_in({idx}, {level})"

    def format_waittime(self, seconds: float) -> str:
        """``sleep(n)``。"""
        return f"sleep({seconds:.3f})"

    def _get_template(self) -> str:
        return """def {{ meta.program_name|default('orolp_prog') }}():
    # RobotWorld v{{ meta.version|default('0.1.0') }} generated URScript
    # Robot: {{ meta.robot_name|default('unknown') }}
    {% for wp in waypoints %}
    {% if wp.cmd in ['SETDO', 'WAITDI', 'WAITTIME'] %}
    {{ wp.text }}
    {% elif wp.cmd == 'movec' %}
    {{ wp.cmd }}({{ wp.via }}, {{ wp.args }}, a={{ wp.accel }}, v={{ wp.speed }}, r={{ wp.blend }})
    {% else %}
    {{ wp.cmd }}({{ wp.args }}, a={{ wp.accel }}, v={{ wp.speed }}, r={{ wp.blend }})
    {% endif %}
    {% endfor %}
end
"""
