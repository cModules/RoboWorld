from typing import Dict

from .base import BasePostProcessor, PostConfig, PostProcessError
from support.i18n import tr
from service.program import Motion


class URScriptPostProcessor(BasePostProcessor):
    """Universal Robots URScript（Polyscope）后处理器。

    目标控制器：UR CB3 / e-Series（Polyscope）
    线位移单位：**米**；角度单位：**弧度**

    Note:
        URScript 的位姿姿态部分是**旋转向量**（轴角），不是欧拉角，
        因此这里用 :meth:`~adapter.post.base.BasePostProcessor.fmt_pose_rotvec`
        而非 :meth:`~adapter.post.base.BasePostProcessor.fmt_pose`。
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
        # 输入 speed 语义为 mm/s（关节运动时按百分比折算），统一转成 m/s；
        # Motion.velocity/acceleration 覆盖值优先（直线 mm/s→m/s，关节 deg/s→rad/s）
        blend_m = motion.blend / 1000.0
        if self.is_joint(motion):
            import math as _math

            speed_val = (
                _math.radians(motion.velocity)
                if motion.velocity is not None
                else motion.speed / 1000.0
            )
            accel_val = (
                _math.radians(motion.acceleration)
                if motion.acceleration is not None
                else self.DEFAULT_ACCEL
            )
            return {
                "cmd": "movej",
                "args": f"[{self.fmt_joints(motion.target)}]",
                "accel": f"{accel_val:.2f}",
                "speed": f"{speed_val:.4f}",
                "blend": f"{blend_m:.4f}",
            }
        speed_m_s = (
            motion.velocity / 1000.0 if motion.velocity is not None else motion.speed / 1000.0
        )
        accel_val = (
            motion.acceleration / 1000.0 if motion.acceleration is not None else self.DEFAULT_ACCEL
        )
        if self.is_linear(motion):
            return {
                "cmd": "movel",
                "args": f"p[{self.fmt_pose_rotvec(motion.target)}]",
                "accel": f"{accel_val:.2f}",
                "speed": f"{speed_m_s:.4f}",
                "blend": f"{blend_m:.4f}",
            }
        if self.is_circular(motion):
            if motion.via is None:
                raise PostProcessError(tr("URScript 圆弧运动需要提供 via 中间点"))
            return {
                "cmd": "movec",
                "via": f"p[{self.fmt_pose_rotvec(motion.via)}]",
                "args": f"p[{self.fmt_pose_rotvec(motion.target)}]",
                "accel": f"{accel_val:.2f}",
                "speed": f"{speed_m_s:.4f}",
                "blend": f"{blend_m:.4f}",
            }
        raise PostProcessError(tr("URScript 不支持的运动类型: {type}").format(type=motion.type))

    # ------------------------- IO 指令（URScript） ------------------------- #

    def format_setdo(self, signal: str, value: bool, io_map: Dict[str, str]) -> str:
        """``set_digital_out(n, True)``。

        URScript 按**数字量序号**寻址，故需把 ``Q0.1`` 换算成序号。
        """
        addr = self.resolve_signal(signal, io_map)
        idx = self.io_index(addr)
        level = "True" if value else "False"
        if idx is None:
            return tr(
                "# TODO: 信号 {signal} 地址 {addr} 无法解析为数字量序号，需人工改为 set_digital_out(n, {level})"
            ).format(signal=signal, addr=addr, level=level)
        return f"set_digital_out({idx}, {level})"

    def format_waitdi(
        self,
        signal: str,
        value: bool,
        timeout: float,
        io_map: Dict[str, str],
        trigger: str = "level",
    ) -> str:
        """``wait_digital_in(n, True)``。

        URScript 的 ``wait_digital_in`` **不接受超时**。给了超时就补注释提示
        需要自行加看门狗，不能假装实现了。边沿触发 URScript 需
        ``wait_digital_in`` + 状态轮询循环实现，降级留 TODO。
        """
        addr = self.resolve_signal(signal, io_map)
        idx = self.io_index(addr)
        if idx is None:
            text = tr(
                "# TODO: 信号 {signal} 地址 {addr} 无法解析为数字量序号，需人工改为 wait_digital_in(n)"
            ).format(signal=signal, addr=addr)
            if trigger != "level":
                text += f"  # " + tr("TODO: 原语义为 {trigger} 边沿触发，请人工核对").format(
                    trigger=trigger
                )
            return text
        level = "True" if value else "False"
        text = f"wait_digital_in({idx}, {level})"
        if timeout and timeout > 0:
            text += f"  # " + tr("TODO: 超时 {seconds:.2f}s 需自行实现看门狗").format(
                seconds=timeout
            )
        if trigger != "level":
            text += f"  # " + tr(
                "TODO: 原语义为 {trigger} 边沿触发，需轮询实现，当前按电平输出，请人工核对"
            ).format(trigger=trigger)
        return text

    def comment(self, text: str) -> str:
        """URScript 注释前缀 ``#``。"""
        return f"# {text}"

    def format_waittime(self, seconds: float) -> str:
        """``sleep(n)``。"""
        return f"sleep({seconds:.3f})"

    # ------------------------- 力控指令（URScript 13.1） ------------------------- #

    def format_force_mode(self, spec) -> str:
        """``force_mode(task_frame, selection_vector, wrench, type, limits)``。

        单位由 :meth:`ComplianceSpec.to_ur_force_mode` 统一换算成
        米 / 弧度 / 牛 / 牛米——与 URScript 手册一致，这里只做拼装。
        """
        fm = spec.to_ur_force_mode()

        def _f(vals):
            return "[" + ", ".join(f"{float(v):.5f}" for v in vals) + "]"

        sel = "[" + ", ".join(str(int(v)) for v in fm["selection_vector"]) + "]"
        return (
            f"force_mode({_f(fm['task_frame'])}, {sel}, "
            f"{_f(fm['wrench'])}, {int(fm['type'])}, {_f(fm['limits'])})"
        )

    def format_force_mode_end(self) -> str:
        """``end_force_mode()``。"""
        return "end_force_mode()"

    def _get_template(self) -> str:
        return """def {{ meta.program_name|default('core_prog') }}():
    # RoboWorld v{{ meta.version|default('0.1.0') }} generated URScript
    # Robot: {{ meta.robot_name|default('unknown') }}
    {% for wp in waypoints %}
    {% if wp.cmd in ['SETDO', 'WAITDI', 'WAITTIME', 'ARCSTART', 'ARCEND', 'SPRAYON', 'SPRAYOFF', 'FORCEMODE', 'FORCEMODEEND'] %}
    {{ wp.text }}
    {% elif wp.cmd == 'movec' %}
    {{ wp.cmd }}({{ wp.via }}, {{ wp.args }}, a={{ wp.accel }}, v={{ wp.speed }}, r={{ wp.blend }})
    {% else %}
    {{ wp.cmd }}({{ wp.args }}, a={{ wp.accel }}, v={{ wp.speed }}, r={{ wp.blend }})
    {% endif %}
    {% endfor %}
end
"""
