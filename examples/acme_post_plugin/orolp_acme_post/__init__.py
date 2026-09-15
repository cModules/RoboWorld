"""orolp-plugin-acme-post —— RobotWorld 第一方示例插件包。

把 ``examples/plugins_demo.py`` 里的 ACME-V2 品牌后处理器挪成一个真正可
``pip install`` 的独立包，用于验证 ``orolp.plugins`` 的 **importlib.metadata
真实入口点路径**（此前测试只能注入假入口点对象）。

安装与验证::

    pip install examples/acme_post_plugin          # 或 pip install -e（开发模式）
    python -c "import orolp.plugins as p; \
        reg = p.PluginRegistry(); \
        print(reg.load_entry_points())"            # {'discovered': 1, 'registered': 1, ...}

入口点 ``acme_v2 = "orolp_acme_post:plugin_specs"`` 加载结果是**提供函数**——
返回 ``PluginSpec`` 列表；后处理工厂写成 ``"module:attr"`` 字符串，由宿主在
``create()`` 时才真正 import。
"""

from __future__ import annotations

from typing import Any, Dict, List

TEMPLATE = """{{ config.comment_char }} ACME V2 / {{ meta.program_name|default('OROLP_PROG') }}
{{ config.comment_char }} 由第一方示例插件包生成（orolp-plugin-acme-post 1.0.0）
DEF {{ meta.program_name|default('OROLP_PROG') }}()
{% for wp in waypoints %}
{% if wp.cmd in ['SETDO', 'WAITDI', 'WAITTIME', 'ARCSTART', 'ARCEND', 'SPRAYON', 'SPRAYOFF', 'FORCEMODE', 'FORCEMODEEND'] %}
  {{ wp.text }}
{% elif wp.cmd == 'MOVEC' %}
  MOVEC {{ wp.via }} -> {{ wp.target }} @ {{ wp.speed }} ;
{% else %}
  {{ wp.cmd }} {{ wp.target }} @ {{ wp.speed }} ;
{% endif %}
{% endfor %}END
"""


def _build_post_class():
    """构造 ACME-V2 后处理类（与内置品牌完全同一套基类与契约）。"""
    from orolp.post.base import BasePostProcessor, PostConfig, PostProcessError
    from orolp.program import Motion

    class AcmeV2PostProcessor(BasePostProcessor):
        """第三方品牌 ACME-V2（示例用的极简控制语言）。"""

        brand = "acme"
        controller = "v2"

        def __init__(self) -> None:
            super().__init__(PostConfig(precision=2, speed_unit="%", comment_char="//"))

        @property
        def language_name(self) -> str:
            return "ACME-V2"

        @property
        def file_extension(self) -> str:
            return "acm"

        def _convert_motion(self, motion: Motion) -> Dict[str, Any]:
            if self.is_joint(motion):
                return {
                    "cmd": "MOVEJ",
                    "target": self.fmt_joints(motion.target),
                    "speed": f"{motion.speed:.0f}%",
                }
            if self.is_linear(motion):
                v = motion.velocity if motion.velocity is not None else motion.speed
                return {
                    "cmd": "MOVEL",
                    "target": self.fmt_pose(motion.target),
                    "speed": f"{v:.0f}mm/s",
                }
            if self.is_circular(motion):
                if motion.via is None:
                    raise PostProcessError("ACME-V2 圆弧运动需要提供 via 中间点")
                v = motion.velocity if motion.velocity is not None else motion.speed
                return {
                    "cmd": "MOVEC",
                    "via": self.fmt_pose(motion.via),
                    "target": self.fmt_pose(motion.target),
                    "speed": f"{v:.0f}mm/s",
                }
            raise PostProcessError(f"ACME-V2 不支持的运动类型: {motion.type}")

        def _get_template(self) -> str:
            return TEMPLATE

    return AcmeV2PostProcessor


AcmeV2PostProcessor = _build_post_class()


def plugin_specs() -> List[Any]:
    """入口点提供函数：返回本包登记的 PluginSpec 列表。

    宿主（``PluginRegistry.load_entry_points``）会对每个 spec 打上
    ``origin="entry_point"`` 标记并登记——来源由宿主判定，不由插件自述。
    """
    from orolp.plugins import KIND_POST, PluginSpec

    return [
        PluginSpec(
            name="acme_v2",  # 后处理插件名 = brand_controller，供 get_post_processor 直取
            kind=KIND_POST,
            factory="orolp_acme_post:AcmeV2PostProcessor",  # 延迟导入
            version="1.0.0",
            description="第一方示例插件：ACME-V2 极简品牌控制语言（pip 可装的独立包）",
        ),
    ]
