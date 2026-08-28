"""品牌后处理器注册表。

集中登记所有可用的后处理器，避免 :meth:`orolp.program.Program.to_post`
里写一长串 ``if/elif``（历史上那串分支里的类名全是错的，且改动时极易漏改）。

用法::

    from orolp.post import get_post_processor, list_post_processors

    pp = get_post_processor("kuka_krc4")     # 按 full_name 取
    pp = get_post_processor("kuka")          # 也可按 brand 取
    code = pp.generate(motions, {"program_name": "weld_01"})
"""

from __future__ import annotations

from typing import Dict, List, Optional, Type

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


def get_post_processor(name: str) -> BasePostProcessor:
    """按名称取后处理器实例。

    Args:
        name: ``full_name``（如 ``kuka_krc4``）或 ``brand``（如 ``kuka``）

    Returns:
        后处理器实例

    Raises:
        PostProcessError: 名称未知，或该品牌有多个后处理器导致 brand 歧义
    """
    key = (name or "").strip().lower()

    if key in POST_PROCESSORS:
        return POST_PROCESSORS[key]()

    # 退化为按品牌匹配
    matches = [k for k in POST_PROCESSORS if k.split("_", 1)[0] == key]
    if len(matches) == 1:
        return POST_PROCESSORS[matches[0]]()
    if len(matches) > 1:
        raise PostProcessError(
            f"品牌 {name!r} 有多个后处理器 {matches}，请指定完整名称"
        )

    raise PostProcessError(
        f"未知的后处理器 {name!r}；可用：{', '.join(sorted(POST_PROCESSORS))}"
    )


def list_post_processors() -> List[str]:
    """返回所有已注册后处理器的 ``full_name``。"""
    return sorted(POST_PROCESSORS)


def resolve_for_brand(brand_id: str) -> Optional[BasePostProcessor]:
    """按品牌 id 解析后处理器，找不到时返回 ``None``（不抛异常）。

    供上层"根据机器人自动选后处理器"的场景使用——没有对应后处理器是正常
    情况，应由调用方决定如何提示，而不是在这里炸掉。

    Args:
        brand_id: 品牌标识，如 ``kuka``

    Returns:
        后处理器实例，或 ``None``
    """
    try:
        return get_post_processor(brand_id)
    except PostProcessError:
        return None
