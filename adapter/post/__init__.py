"""品牌后处理器注册表。

集中登记所有可用的后处理器，避免 :meth:`service.program.Program.to_post`
里写一长串 ``if/elif``（历史上那串分支里的类名全是错的，且改动时极易漏改）。

用法::

    from adapter.post import get_post_processor, list_post_processors

    pp = get_post_processor("kuka_krc4")     # 按 full_name 取
    pp = get_post_processor("kuka")          # 也可按 brand 取
    code = pp.generate(motions, {"program_name": "weld_01"})

内置品牌之外，这里也**认插件登记的后处理器**（v1.0.4 起）：第三方调用
:func:`adapter.plugins.register_post_processor` 之后，新品牌即与内置品牌走同一条路——
``program.to_post("acme_v2")``、品牌清单、扩展名解析都能直接用。
"""

from __future__ import annotations

from typing import Dict, List, Optional, Type

from support.i18n import tr
from .base import BasePostProcessor, PostProcessError

__all__ = [
    "POST_PROCESSORS",
    "get_post_processor",
    "list_post_processors",
    "resolve_for_brand",
    "PostProcessError",
]


def _registry() -> Dict[str, Type[BasePostProcessor]]:
    """延迟构建注册表：逐个导入，任一模块出错都不影响其余品牌可用。"""
    from .abb_rapid import ABBRapidPostProcessor
    from .estun_er import ESTUNERPostProcessor
    from .fanuc_tp import FANUCTPPostProcessor
    from .huichuan_inoteach import HuiChuanInoTeachPostProcessor
    from .jaka_zu import JAKAZuPostProcessor
    from .kuka_krc4 import KUKAKRC4PostProcessor
    from .ur_script import URScriptPostProcessor

    classes = [
        KUKAKRC4PostProcessor,
        JAKAZuPostProcessor,
        ABBRapidPostProcessor,
        FANUCTPPostProcessor,
        URScriptPostProcessor,
        ESTUNERPostProcessor,
        HuiChuanInoTeachPostProcessor,
    ]
    # 注意：``full_name`` 是基类上的 property，类层面访问只会拿到 property 对象，
    # 因此这里用子类上的字符串类属性 brand/controller 自行拼接。
    return {f"{cls.brand}_{cls.controller}": cls for cls in classes}


POST_PROCESSORS: Dict[str, Type[BasePostProcessor]] = _registry()


def _plugin_post_names() -> List[str]:
    """插件登记的后处理器名（不带前缀，就是 ``full_name``）。

    延迟取用：插件层未初始化或取了坏插件都不该影响内置品牌。
    """
    try:
        from ..plugins import KIND_POST, default_registry

        return [s.name for s in default_registry.specs(KIND_POST)]
    except Exception:
        return []


def _create_plugin_post(name: str) -> BasePostProcessor:
    """构造一个插件后处理器实例。"""
    from ..plugins import KIND_POST, default_registry

    pp = default_registry.create(name, kind=KIND_POST)
    if not isinstance(pp, BasePostProcessor):
        raise PostProcessError(
            tr("插件后处理器 {name} 未返回 BasePostProcessor 子类实例，而是 {cls}").format(
                name=repr(name), cls=type(pp).__name__
            )
        )
    return pp


def get_post_processor(name: str) -> BasePostProcessor:
    """按名称取后处理器实例。

    Args:
        name: ``full_name``（如 ``kuka_krc4``）或 ``brand``（如 ``kuka``）；
            也可以是插件登记的名字

    Returns:
        后处理器实例

    Raises:
        PostProcessError: 名称未知，或该品牌有多个后处理器导致 brand 歧义
    """
    key = (name or "").strip().lower()

    if key in POST_PROCESSORS:
        return POST_PROCESSORS[key]()

    plugin_names = _plugin_post_names()

    # 插件名精确命中（插件名就是 full_name）
    for pname in plugin_names:
        if pname.lower() == key:
            return _create_plugin_post(pname)

    # 退化为按品牌匹配（内置 + 插件一起参与）
    matches = [k for k in POST_PROCESSORS if k.split("_", 1)[0] == key]
    matches += [p for p in plugin_names if p.split("_", 1)[0].lower() == key]
    if len(matches) == 1:
        hit = matches[0]
        return _create_plugin_post(hit) if hit in plugin_names else POST_PROCESSORS[hit]()
    if len(matches) > 1:
        raise PostProcessError(
            tr("品牌 {name} 有多个后处理器 {matches}，请指定完整名称").format(
                name=repr(name), matches=matches
            )
        )

    known = sorted(set(POST_PROCESSORS) | set(plugin_names))
    raise PostProcessError(
        tr("未知的后处理器 {name}；可用：{available}").format(
            name=repr(name), available=", ".join(known)
        )
    )


def list_post_processors() -> List[str]:
    """返回所有已注册后处理器的 ``full_name``（内置 + 插件）。"""
    return sorted(set(POST_PROCESSORS) | set(_plugin_post_names()))


def resolve_for_brand(brand_id: str) -> Optional[BasePostProcessor]:
    """按品牌 id 解析后处理器，找不到时返回 ``None``（不抛异常）。

    供上层"根据机器人自动选后处理器"的场景使用——没有对应后处理器是正常
    情况，应由调用方决定如何提示，而不是在这里炸掉。插件品牌同样参与解析。

    Args:
        brand_id: 品牌标识，如 ``kuka``

    Returns:
        后处理器实例，或 ``None``
    """
    try:
        return get_post_processor(brand_id)
    except Exception:
        # 除"没有这个品牌"外，插件（第三方代码）构造失败也可能抛各种异常——
        # 这里一律返回 None 交给调用方提示，避免"自动选后处理器"因一个坏插件整体失败。
        return None
