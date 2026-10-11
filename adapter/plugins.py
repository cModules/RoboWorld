"""插件接口（v1.0.4）：让第三方在**不改核心代码**的前提下加品牌、加算法、加渲染层。

为什么需要它
------------

这个仓库里已经有三处"每次加东西都要改核心"的地方：

* **后处理**：新增一个品牌的控制语言，得改 ``post/__init__.py`` 的注册表——而
  :mod:`adapter.post.base` 本身已经把"品牌差异"抽象得很干净了（模板 + 格式化钩子），
  真正缺的只是"从外部挂进来"的入口；
* **运动学**：:class:`domain.kinematics.RobotKinematics` 是通用数值解，但真机常常要有
  自己的解析解/标定过的几何模型（差几丝就能决定标定要不要重做）；
* **渲染**：现场关心的是"把我这条焊缝/这批偏差点画出来"，而这类可视化只有使用者
  自己知道要画什么。

三者的共同点是：**核心已经提供了正确的抽象，只是把它们锁在了包内部**。这一层把
它们开放成插件。

设计上的几个取舍
----------------

**1. 延迟加载。** ``register_*`` 只登记元信息，工厂在 ``create()`` 时才被调用；工厂
还可以写成 ``"module:attr"`` 字符串，连模块导入都推迟到那一刻。原因很实际：插件
可能依赖 PySide6 / OpenGL / 厂商 SDK，而 ``import accel`` 不能因此变重或失败。

**2. 失败隔离。** 一个坏插件不能拖垮整张注册表：入口点加载失败记进
:meth:`PluginRegistry.errors` 继续走；工厂抛错包成 :class:`PluginError` 并保留
``__cause__``；渲染插件返回了不认识的对象只记错误、不影响其余插件出图。这条在
"第三方生态"里不是可选项——用户装了一个坏插件就再也打不开软件，等于把整个
软件判了死刑。

**3. API 版本显式声明。** :data:`PLUGIN_API_VERSION` 是宿主提供的接口版本，插件用
``requires_api`` 声明它需要的版本。规则只有一条：**主版本号必须相等**（``1.2`` 与
``1.5`` 互相兼容，``1.x`` 与 ``2.x`` 不兼容）。刻意选最可解释的规则——宁可偶尔多
拒一个其实能跑的插件，也不要让"悄悄按新语义跑出错误轨迹"这种事发生在工业软件里。

**4. 契约显式校验，而不是等着 AttributeError。** 运动学插件在创建时就会被
:func:`check_kinematics` 检查必需方法是否齐全：缺必需项直接报错（拒绝加载），缺可选项
只记警告（例如只实现 FK/IK 的插件能跑正逆解，但碰撞检测会因为拿不到连杆几何而
明确提示，而不是在运行到一半时炸出 ``AttributeError``）。

用法
----

第三方加一个品牌::

    from adapter.plugins import register_post_processor

    register_post_processor("acme_v2", lambda: MyAcmePostProcessor())

之后 ``program.to_post("acme_v2")`` 就能用——和内置品牌走同一条路。

以正式包分发（推荐）时，在 ``pyproject.toml`` 里声明入口点::

    [project.entry-points."adapter.plugins"]
    acme_v2 = "acme_core.plugin:post_processor_spec"

宿主侧 ``adapter.plugins.load_entry_points()`` 会在**进程启动时**（或用户显式调用时）
把已安装的插件自动登记进来。
"""

from __future__ import annotations

import dataclasses
import importlib
import importlib.metadata as importlib_metadata
import traceback
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

__all__ = [
    "PLUGIN_API_VERSION",
    "PLUGIN_ENTRY_POINT_GROUP",
    "KIND_POST",
    "KIND_KINEMATICS",
    "KIND_RENDERER",
    "PLUGIN_KINDS",
    "PluginError",
    "PluginSpec",
    "PluginLoadError",
    "PluginRegistry",
    "KinematicsReport",
    "check_kinematics",
    "LineSegments",
    "PointCloud",
    "FrameMarker",
    "OVERLAY_TYPES",
    "OverlayContext",
    "OverlayCollection",
    "collect_overlays",
    "api_compatible",
    "default_registry",
    "register_post_processor",
    "register_kinematics",
    "register_renderer",
    "unregister",
    "get_plugin",
    "find_plugin",
    "list_plugins",
    "describe_plugins",
    "plugin_names",
    "create_plugin",
    "load_entry_points",
    "plugin_errors",
    "clear_plugins",
]

#: 宿主提供的插件接口版本。改动**已发布**接口的语义时必须升主版本号。
PLUGIN_API_VERSION = "1.0"

#: 第三方打包插件时使用的入口点组名（``pyproject.toml`` 的
#: ``[project.entry-points."adapter.plugins"]``）。
PLUGIN_ENTRY_POINT_GROUP = "adapter.plugins"  # 契约值：第三方 pyproject 写死，改名会废掉已装插件

#: 扩展点：后处理器
KIND_POST = "post"
#: 扩展点：运动学求解器
KIND_KINEMATICS = "kinematics"
#: 扩展点：渲染覆盖层
KIND_RENDERER = "renderer"

PLUGIN_KINDS: Tuple[str, ...] = (KIND_POST, KIND_KINEMATICS, KIND_RENDERER)

#: 运动学实现的**必需**成员——缺任一项即拒绝加载
KINEMATICS_REQUIRED = ("dof", "joint_names", "forward_kinematics", "inverse_kinematics")
#: 运动学实现的**可选**成员——缺失只记警告（对应功能受限，但基础 FK/IK 可用）
KINEMATICS_OPTIONAL = (
    "jacobian",
    "limits",
    "joint_origins",
    "joint_axes",
    "all_link_transforms",
    "link_transforms",
    "actuated",
    "chain",
)


class PluginError(RuntimeError):
    """插件登记/加载/创建失败。"""


# --------------------------------------------------------------------------- #
# API 版本
# --------------------------------------------------------------------------- #


def _parse_api_version(text: str) -> Tuple[int, int]:
    raw = str(text or "").strip()
    parts = raw.split(".")
    if not parts or not parts[0].isdigit():
        raise PluginError(f"非法的 API 版本号 {text!r}（应形如 '1.0'）")
    major = int(parts[0])
    minor = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 0
    return major, minor


def api_compatible(requires_api: str, host_api: str = PLUGIN_API_VERSION) -> bool:
    """插件声明的 API 需求是否被当前宿主满足。

    规则：**主版本号必须相等**。``1.2`` 与 ``1.5`` 兼容，``1.x`` 与 ``2.x`` 不兼容。
    只比主版本是刻意的保守选择——副版本号变化按约定不破坏语义。
    """
    try:
        need_major, _ = _parse_api_version(requires_api)
        host_major, _ = _parse_api_version(host_api)
    except PluginError:
        return False
    return need_major == host_major


# --------------------------------------------------------------------------- #
# 插件描述
# --------------------------------------------------------------------------- #


@dataclass
class PluginLoadError:
    """一次失败的插件加载/创建记录（失败要被看见，而不是被吞掉）。

    Attributes:
        name: 插件名（能确定时）
        source: 失败发生在哪一步——``entry_point`` / ``register`` / ``create`` / ``render``
        message: 异常文本
        detail: 精简后的 traceback（便于定位，单条不超过 ~500 字符）
    """

    name: str
    source: str
    message: str
    detail: str = ""


def _brief_traceback(limit: int = 500) -> str:
    text = traceback.format_exc().strip()
    return text if len(text) <= limit else "…" + text[-limit:]


def _load_factory(factory: Any, name: str) -> Callable[..., Any]:
    """把工厂解析成可调用对象。

    支持三种形态：可调用对象、``"module:attr"``、``"module.attr"``。
    字符串形式会在**这时**才 import——插件依赖 GUI/SDK 也不会拖累宿主启动。
    """
    if callable(factory):
        return factory
    if not isinstance(factory, str):
        raise PluginError(
            f"插件 {name!r} 的 factory 必须是可调用对象或 'module:attr' 字符串，"
            f"实际是 {type(factory).__name__}"
        )
    ref = factory.strip()
    if ":" in ref:
        mod_name, _, attr = ref.partition(":")
    else:
        mod_name, _, attr = ref.rpartition(".")
    if not mod_name or not attr:
        raise PluginError(f"插件 {name!r} 的 factory 引用 {factory!r} 非法（应为 'module:attr'）")
    try:
        module = importlib.import_module(mod_name)
    except Exception as exc:
        raise PluginError(f"导入插件模块 {mod_name!r} 失败：{exc}") from exc
    try:
        target = getattr(module, attr)
    except AttributeError as exc:
        raise PluginError(f"模块 {mod_name!r} 里没有 {attr!r}") from exc
    if not callable(target):
        raise PluginError(f"{factory!r} 指向的 {attr!r} 不可调用")
    return target


@dataclass
class PluginSpec:
    """一条插件登记项（元信息 + 延迟工厂）。

    Attributes:
        name: 插件唯一名。建议 ``厂商名.功能名`` 形式（如 ``acme.gripper_post``），
            后处理器插件例外——它的名字就是 ``full_name``（``brand_controller``），
            这样才能被 :func:`core.post.get_post_processor` 直接取到。
        kind: 扩展点，取值见 :data:`PLUGIN_KINDS`
        factory: 工厂——可调用对象，或 ``"module:attr"`` 字符串（延迟导入）
        requires_api: 插件要求的宿主 API 版本，默认与本宿主同版本
        version: 插件自身版本号（纯展示）
        description: 一句话说明（会出现在 :func:`describe_plugins` 里）
        origin: 登记来源——``manual`` / ``entry_point``
        options: 默认构造参数，创建时可被 ``overrides`` 覆盖
    """

    name: str
    kind: str
    factory: Any
    requires_api: str = PLUGIN_API_VERSION
    version: str = "0.0.0"
    description: str = ""
    origin: str = "manual"
    options: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.kind not in PLUGIN_KINDS:
            raise PluginError(
                f"插件 {self.name!r} 的 kind={self.kind!r} 未知；可用：{', '.join(PLUGIN_KINDS)}"
            )
        if not str(self.name).strip():
            raise PluginError("插件名不能为空")
        if not api_compatible(self.requires_api):
            raise PluginError(
                f"插件 {self.name!r} 要求 API {self.requires_api}，"
                f"当前宿主提供 {PLUGIN_API_VERSION}——主版本不匹配，拒绝加载"
            )

    @property
    def api_ok(self) -> bool:
        """API 版本是否兼容（登记时已强制，此处供展示）。"""
        return api_compatible(self.requires_api)

    def create(self, **overrides: Any) -> Any:
        """按需构造插件产出的对象。

        Raises:
            PluginError: 工厂解析或调用失败（原异常保留在 ``__cause__``）
        """
        options = dict(self.options)
        options.update(overrides)
        try:
            factory = _load_factory(self.factory, self.name)
            return factory(**options)
        except PluginError:
            raise
        except Exception as exc:
            raise PluginError(f"插件 {self.name!r} 构造失败：{exc}") from exc

    def summary(self) -> str:
        """一行摘要。"""
        bits = [f"{self.name} [{self.kind}] v{self.version}"]
        if self.description:
            bits.append(self.description)
        # 写成 "API 1.x" 而不是 "api>=1.0"：兼容规则是**主版本相等**，
        # 不是"大于等于"，写成 >= 会让人以为 2.0 也能跑。
        major = _parse_api_version(self.requires_api)[0]
        bits.append(f"需 API {major}.x")
        if self.origin != "manual":
            bits.append(f"来源={self.origin}")
        return " · ".join(bits)


# --------------------------------------------------------------------------- #
# 注册表
# --------------------------------------------------------------------------- #


class PluginRegistry:
    """插件注册表（一个进程内通常只有一个，即 :data:`default_registry`）。

    注册表本身不 import 任何插件产物——只登记 :class:`PluginSpec`。真正的加载发生在
    :meth:`create`（以及后处理器的第一次取用）时。
    """

    def __init__(self) -> None:
        self._specs: Dict[str, PluginSpec] = {}
        self._errors: List[PluginLoadError] = []
        self._loaded_groups: set = set()

    # ------------------------------ 登记 ------------------------------ #

    def register(self, spec: PluginSpec, replace: bool = False) -> PluginSpec:
        """登记一条插件项。

        Args:
            spec: 插件描述
            replace: 同名已存在时是否覆盖（默认 ``False``，冲突即报错）

        Raises:
            PluginError: 同名冲突且 ``replace=False``
        """
        if not isinstance(spec, PluginSpec):
            raise PluginError(f"register 需要 PluginSpec，收到 {type(spec).__name__}")
        if spec.name in self._specs and not replace:
            existing = self._specs[spec.name]
            raise PluginError(
                f"插件名 {spec.name!r} 已被占用"
                f"（{existing.summary()}）；要覆盖请传 replace=True"
            )
        self._specs[spec.name] = spec
        return spec

    def _register_factory(
        self,
        name: str,
        kind: str,
        factory: Any,
        *,
        version: str = "0.0.0",
        description: str = "",
        requires_api: str = PLUGIN_API_VERSION,
        options: Optional[Dict[str, Any]] = None,
        replace: bool = False,
    ) -> PluginSpec:
        return self.register(
            PluginSpec(
                name=name,
                kind=kind,
                factory=factory,
                requires_api=requires_api,
                version=version,
                description=description,
                options=dict(options or {}),
            ),
            replace=replace,
        )

    def register_post_processor(
        self,
        name: str,
        factory: Any,
        *,
        version: str = "0.0.0",
        description: str = "",
        requires_api: str = PLUGIN_API_VERSION,
        options: Optional[Dict[str, Any]] = None,
        replace: bool = False,
    ) -> PluginSpec:
        """登记一个后处理器插件。

        ``name`` 必须是 ``brand_controller`` 形式——它就是
        :meth:`core.post.get_post_processor` 的查表键，也决定导出文件扩展名解析。
        """
        if "_" not in str(name):
            raise PluginError(
                f"后处理器插件名 {name!r} 应为 'brand_controller' 形式（如 'acme_v2'）"
            )
        return self._register_factory(
            name,
            KIND_POST,
            factory,
            version=version,
            description=description,
            requires_api=requires_api,
            options=options,
            replace=replace,
        )

    def register_kinematics(
        self,
        name: str,
        factory: Any,
        *,
        version: str = "0.0.0",
        description: str = "",
        requires_api: str = PLUGIN_API_VERSION,
        options: Optional[Dict[str, Any]] = None,
        replace: bool = False,
    ) -> PluginSpec:
        """登记一个运动学求解器插件。"""
        return self._register_factory(
            name,
            KIND_KINEMATICS,
            factory,
            version=version,
            description=description,
            requires_api=requires_api,
            options=options,
            replace=replace,
        )

    def register_renderer(
        self,
        name: str,
        factory: Any,
        *,
        version: str = "0.0.0",
        description: str = "",
        requires_api: str = PLUGIN_API_VERSION,
        options: Optional[Dict[str, Any]] = None,
        replace: bool = False,
    ) -> PluginSpec:
        """登记一个渲染覆盖层插件（工厂需返回带 ``overlays(ctx)`` 的对象）。"""
        return self._register_factory(
            name,
            KIND_RENDERER,
            factory,
            version=version,
            description=description,
            requires_api=requires_api,
            options=options,
            replace=replace,
        )

    def unregister(self, name: str) -> bool:
        """撤销登记；返回是否真的删掉了。"""
        return self._specs.pop(name, None) is not None

    def clear(self) -> None:
        """清空登记与错误记录（测试与"重载插件"用）。"""
        self._specs.clear()
        self._errors.clear()
        self._loaded_groups.clear()

    # ------------------------------ 查询 ------------------------------ #

    def find(self, name: str) -> Optional[PluginSpec]:
        """按名取登记项，找不到返回 ``None``。"""
        return self._specs.get(str(name))

    def get(self, name: str, kind: Optional[str] = None) -> PluginSpec:
        """按名取登记项，必要时校验扩展点。

        Raises:
            PluginError: 不存在，或 kind 不符
        """
        spec = self.find(name)
        if spec is None:
            hint = ""
            for other in self._specs.values():
                if other.name.lower() == str(name).lower():
                    hint = f"（是否想找 {other.name!r}？插件名区分大小写）"
                    break
            raise PluginError(f"未登记的插件 {name!r}{hint}")
        if kind is not None and spec.kind != kind:
            raise PluginError(f"插件 {name!r} 是 {spec.kind!r} 类型，不能当作 {kind!r} 使用")
        return spec

    def specs(self, kind: Optional[str] = None) -> List[PluginSpec]:
        """列出登记项，可按扩展点过滤（按名字排序，结果稳定）。"""
        items = [s for s in self._specs.values() if kind is None or s.kind == kind]
        return sorted(items, key=lambda s: s.name)

    def names(self, kind: Optional[str] = None) -> List[str]:
        """列出插件名。"""
        return [s.name for s in self.specs(kind)]

    def __len__(self) -> int:
        return len(self._specs)

    def __contains__(self, name: object) -> bool:
        return str(name) in self._specs

    # ------------------------------ 创建 ------------------------------ #

    def create(self, name: str, kind: Optional[str] = None, **overrides: Any) -> Any:
        """构造插件产物（后处理器实例 / 运动学实现 / 渲染器）。

        运动学插件在返回前会经过 :func:`check_kinematics`：缺必需成员直接报错，
        缺可选项记入 :meth:`errors` 作为警告。
        """
        spec = self.get(name, kind=kind)
        obj = spec.create(**overrides)
        if spec.kind == KIND_KINEMATICS:
            report = check_kinematics(obj)
            if not report.ok:
                raise PluginError(
                    f"运动学插件 {name!r} 缺少必需成员 {report.missing_required}；"
                    f"必需成员为 {list(KINEMATICS_REQUIRED)}"
                )
            if report.missing_optional:
                # 聚合成一条：一个插件缺 8 个可选成员就是 1 条警告，不是 8 条——
                # 否则 GUI 的错误列表会被同一个插件的同一件事刷屏
                self._errors.append(
                    PluginLoadError(
                        name=name,
                        source="create",
                        message=(
                            f"运动学插件缺少可选成员 {report.missing_optional}——"
                            f"相关功能（碰撞/动力学/限位）将不可用"
                        ),
                    )
                )
        return obj

    def create_renderer(self, name: str, **overrides: Any) -> Any:
        """构造渲染器插件并校验 ``overlays`` 钩子存在。"""
        obj = self.create(name, kind=KIND_RENDERER, **overrides)
        if not callable(getattr(obj, "overlays", None)):
            raise PluginError(f"渲染器插件 {name!r} 必须提供 overlays(ctx) 方法")
        return obj

    # ------------------------------ 入口点发现 ------------------------------ #

    def load_entry_points(
        self,
        group: str = PLUGIN_ENTRY_POINT_GROUP,
        entry_points: Optional[Iterable[Any]] = None,
        replace: bool = False,
    ) -> Dict[str, int]:
        """扫描并登记已安装包的插件入口点。

        Args:
            group: 入口点组名
            entry_points: 显式传入的入口点集合（测试用）；``None`` 时从已安装包读取
            replace: 同名冲突时是否覆盖

        Returns:
            ``{"discovered": n, "registered": n, "skipped": n, "failed": n}``

        Note:
            单个入口点失败**不会**中断整轮扫描：错误记入 :meth:`errors`，继续处理其余
            插件。自带生态的软件里，"装了一个坏插件导致谁都加载不了"是不可接受的。
        """
        stats = {"discovered": 0, "registered": 0, "skipped": 0, "failed": 0}
        if entry_points is None:
            try:
                entry_points = list(importlib_metadata.entry_points(group=group))
            except TypeError:  # Python < 3.10 的旧签名
                entry_points = list(  # pragma: no cover - 兼容分支
                    importlib_metadata.entry_points().get(group, [])
                )
            except Exception as exc:  # pragma: no cover - 环境异常
                self._errors.append(
                    PluginLoadError(
                        name=group, source="entry_point", message=f"读取入口点失败：{exc}"
                    )
                )
                return stats

        for ep in entry_points:
            stats["discovered"] += 1
            ep_name = getattr(ep, "name", "<unnamed>")
            try:
                loaded = ep.load()
            except Exception as exc:
                stats["failed"] += 1
                self._errors.append(
                    PluginLoadError(
                        name=ep_name,
                        source="entry_point",
                        message=f"加载入口点失败：{exc}",
                        detail=_brief_traceback(),
                    )
                )
                continue
            try:
                specs = self._coerce_entry_point(ep_name, loaded)
                for spec in specs:
                    # 来源由**宿主**判定，不由插件自述；用副本而不是改原对象，免得
                    # 同一个 spec 被手工 register 时也顶着 entry_point 的标签
                    spec = dataclasses.replace(spec, origin="entry_point")
                    if spec.name in self._specs and not replace:
                        stats["skipped"] += 1
                        continue
                    self.register(spec, replace=replace)
                    stats["registered"] += 1
            except Exception as exc:
                stats["failed"] += 1
                self._errors.append(
                    PluginLoadError(
                        name=ep_name,
                        source="register",
                        message=f"登记失败：{exc}",
                        detail=_brief_traceback(),
                    )
                )
        self._loaded_groups.add(group)
        return stats

    @staticmethod
    def _coerce_entry_point(ep_name: str, loaded: Any) -> List[PluginSpec]:
        """把入口点加载结果规整成 :class:`PluginSpec` 列表。

        允许三种写法，越简单越好：

        * 单个 :class:`PluginSpec`；
        * :class:`PluginSpec` 的列表/元组（一个包想一次注册多个扩展点）；
        * 一个 ``plugins()`` 函数——调用它拿到上面两种之一（工厂需要读配置/探测
          环境时用）。
        """
        obj = loaded
        if callable(obj) and not isinstance(obj, PluginSpec):
            # 工厂函数形式：约定它返回 spec 或 spec 列表
            obj = _call_if_plugin_provider(obj)
        if isinstance(obj, PluginSpec):
            return [obj]
        if isinstance(obj, (list, tuple)):
            specs: List[PluginSpec] = []
            for item in obj:
                if not isinstance(item, PluginSpec):
                    raise PluginError(
                        f"入口点 {ep_name!r} 返回了 {type(item).__name__}，列表里只能放 PluginSpec"
                    )
                specs.append(item)
            if not specs:
                raise PluginError(f"入口点 {ep_name!r} 返回了空列表")
            return specs
        raise PluginError(
            f"入口点 {ep_name!r} 应返回 PluginSpec 或它的列表，实际是 {type(obj).__name__}"
        )

    # ------------------------------ 错误与总览 ------------------------------ #

    def errors(self, kind: Optional[str] = None) -> List[PluginLoadError]:
        """返回加载/创建过程中的错误与警告记录（``kind`` 这里忽略，仅为签名一致）。"""
        return list(self._errors)

    def add_error(self, name: str, source: str, message: str, detail: str = "") -> PluginLoadError:
        """追加一条错误/警告记录。

        ``errors()`` 返回的是副本（避免调用方无意改到内部状态），所以外部想记一笔
        必须走这里——宿主自己降级时也用它。
        """
        err = PluginLoadError(name=name, source=source, message=message, detail=detail)
        self._errors.append(err)
        return err

    def clear_errors(self) -> None:
        """清空错误记录。"""
        self._errors.clear()

    def summary(self) -> str:
        """人类可读的注册表现状（CLI / GUI 关于框用）。"""
        lines = [f"RoboWorld 插件注册表（宿主 API {PLUGIN_API_VERSION}）"]
        if not self._specs:
            lines.append("  （未登记任何插件）")
        for kind in PLUGIN_KINDS:
            items = self.specs(kind)
            if not items:
                continue
            lines.append(f"  {kind}（{len(items)}）:")
            for spec in items:
                lines.append(f"    · {spec.summary()}")
        if self._errors:
            lines.append(f"  错误/警告（{len(self._errors)}）:")
            for err in self._errors:
                lines.append(f"    ! [{err.source}] {err.name}: {err.message}")
        return "\n".join(lines)


def _call_if_plugin_provider(obj: Callable[..., Any]) -> Any:
    """调用入口点工厂；若签名不接受零参数，则把异常原样抛出（由上层记录）。"""
    return obj()


#: 进程默认注册表。包级函数（:func:`register_post_processor` 等）都转发到这里。
default_registry = PluginRegistry()


# --------------------------------------------------------------------------- #
# 包级便捷函数
# --------------------------------------------------------------------------- #


def register_post_processor(name: str, factory: Any, **kwargs: Any) -> PluginSpec:
    """登记后处理器插件（转发到 :data:`default_registry`）。"""
    return default_registry.register_post_processor(name, factory, **kwargs)


def register_kinematics(name: str, factory: Any, **kwargs: Any) -> PluginSpec:
    """登记运动学插件。"""
    return default_registry.register_kinematics(name, factory, **kwargs)


def register_renderer(name: str, factory: Any, **kwargs: Any) -> PluginSpec:
    """登记渲染覆盖层插件。"""
    return default_registry.register_renderer(name, factory, **kwargs)


def unregister(name: str) -> bool:
    """撤销登记。"""
    return default_registry.unregister(name)


def find_plugin(name: str) -> Optional[PluginSpec]:
    """按名查插件，找不到返回 ``None``。"""
    return default_registry.find(name)


def get_plugin(name: str, kind: Optional[str] = None) -> PluginSpec:
    """按名取插件，找不到报错。"""
    return default_registry.get(name, kind=kind)


def list_plugins(kind: Optional[str] = None) -> List[str]:
    """列出插件名。"""
    return default_registry.names(kind)


def plugin_names(kind: Optional[str] = None) -> List[str]:
    """:func:`list_plugins` 的别名（表达"这是名字清单"时更好读）。"""
    return default_registry.names(kind)


def describe_plugins(kind: Optional[str] = None) -> List[str]:
    """返回每条插件的摘要行。"""
    return [s.summary() for s in default_registry.specs(kind)]


def create_plugin(name: str, kind: Optional[str] = None, **overrides: Any) -> Any:
    """构造插件产物。"""
    return default_registry.create(name, kind=kind, **overrides)


def load_entry_points(group: str = PLUGIN_ENTRY_POINT_GROUP, **kwargs: Any) -> Dict[str, int]:
    """扫描并登记已安装包的插件入口点。"""
    return default_registry.load_entry_points(group, **kwargs)


def plugin_errors() -> List[PluginLoadError]:
    """返回错误/警告记录。"""
    return default_registry.errors()


def clear_plugins() -> None:
    """清空默认注册表（测试用）。"""
    default_registry.clear()


# --------------------------------------------------------------------------- #
# 运动学契约校验
# --------------------------------------------------------------------------- #


@dataclass
class KinematicsReport:
    """运动学插件契约检查结果。

    Attributes:
        missing_required: 缺失的必需成员——非空即拒绝加载
        missing_optional: 缺失的可选成员——只影响对应功能，记警告
        dof: 报告自由度（读不到为 ``None``）
    """

    missing_required: List[str] = field(default_factory=list)
    missing_optional: List[str] = field(default_factory=list)
    dof: Optional[int] = None

    @property
    def ok(self) -> bool:
        """必需成员是否齐全。"""
        return not self.missing_required

    def summary(self) -> str:
        """一行摘要。"""
        if self.ok:
            tail = (
                f"；缺少可选成员 {self.missing_optional}（相关功能受限）"
                if self.missing_optional
                else "；接口完整"
            )
            return f"运动学实现契约通过（dof={self.dof}）{tail}"
        return f"运动学实现缺少必需成员 {self.missing_required}"


def check_kinematics(impl: Any) -> KinematicsReport:
    """检查一个运动学实现是否满足宿主契约。

    Args:
        impl: 运动学实现对象（如 :class:`~domain.kinematics.RobotKinematics` 的同类）

    Returns:
        :class:`KinematicsReport`

    Note:
        只做**存在性**检查，不做数值校验——数值对不对要在真实 URDF 上跑 FK/IK 才能
        判断，那是测试的事，不是加载期该做的事（加载期做重活会拖慢启动，且容易把
        "参数没配好"误报成"插件坏了"）。
    """
    missing_req = [n for n in KINEMATICS_REQUIRED if not hasattr(impl, n)]
    missing_opt = [n for n in KINEMATICS_OPTIONAL if not hasattr(impl, n)]
    dof: Optional[int] = None
    if "dof" in dir(impl) or hasattr(impl, "dof"):
        try:
            dof = int(getattr(impl, "dof"))
        except Exception:
            dof = None
    return KinematicsReport(missing_required=missing_req, missing_optional=missing_opt, dof=dof)


# --------------------------------------------------------------------------- #
# 渲染覆盖层：纯数据契约
# --------------------------------------------------------------------------- #
#
# 覆盖层刻意只描述**几何**，不碰 OpenGL。理由：GL 上下文只能在真正的窗口里创建，
# 一旦插件产出的是 GL 调用，它就只能在 GUI 里被测试（或者根本不能）。而这些数据类
# 是纯 NumPy，可以在无头环境里完整断言"插件到底想画什么"。


@dataclass
class LineSegments:
    """一组线段（世界系，毫米）。

    Attributes:
        segments: ``[(p0, p1), ...]``，每个点是长度 3 的序列
        color: ``(r, g, b)``，0–1
        width: 线宽（px）
    """

    segments: List[Tuple[Sequence[float], Sequence[float]]]
    color: Tuple[float, float, float] = (1.0, 0.8, 0.2)
    width: float = 2.0


@dataclass
class PointCloud:
    """一组点（世界系，毫米）。

    Attributes:
        positions: ``(N, 3)`` 或长度 3 的点列表
        color: ``(r, g, b)``，0–1
        size: 点尺寸（px）
    """

    positions: Any
    color: Tuple[float, float, float] = (0.4, 0.9, 0.4)
    size: float = 5.0

    def __post_init__(self) -> None:
        arr = np.asarray(self.positions, dtype=float)
        if arr.ndim != 2 or arr.shape[1] != 3:
            raise PluginError(f"PointCloud.positions 必须是 (N, 3)，收到形状 {arr.shape}")
        self.positions = arr


@dataclass
class FrameMarker:
    """一个位姿三轴指示（世界系，毫米 + 度）。

    Attributes:
        pose: :class:`~domain.pose.Pose` 或 ``4x4`` 矩阵
        scale: 轴长（mm）
        color: ``(r, g, b)`` 基准色（三轴会各自调亮/调暗以区分）
    """

    pose: Any
    scale: float = 60.0
    color: Tuple[float, float, float] = (0.9, 0.9, 0.9)


#: 渲染插件允许返回的对象类型（其余类型会被拒绝并记错误）
OVERLAY_TYPES = (LineSegments, PointCloud, FrameMarker)


@dataclass
class OverlayContext:
    """交给渲染插件的上下文——**只读**，且不暴露 GL。

    Attributes:
        tcp_path: ``(N, 3)`` 规划 TCP 轨迹（世界系 mm）
        joint_path: ``(N, dof)`` 规划关节角（deg）
        timestamps: ``(N,)`` 各点时刻（s）
        current_index: 当前回放到的路点序号（``-1`` 表示无）
        tolerance_mm: 当前校验容差（mm），画容差带/告警阈值时用
        robot_name: 机器人名
        extra: 插件自定义数据（宿主不认识，原样传下去）
    """

    tcp_path: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    joint_path: np.ndarray = field(default_factory=lambda: np.zeros((0, 0)))
    timestamps: np.ndarray = field(default_factory=lambda: np.zeros(0))
    current_index: int = -1
    tolerance_mm: float = 1.0
    robot_name: str = ""
    extra: Dict[str, Any] = field(default_factory=dict)

    @property
    def count(self) -> int:
        """轨迹点数。"""
        return int(self.tcp_path.shape[0]) if self.tcp_path is not None else 0

    def current_tcp(self) -> Optional[np.ndarray]:
        """当前路点的 TCP 位置；越界或为空时返回 ``None``。"""
        if self.tcp_path is None or self.count == 0:
            return None
        idx = int(np.clip(self.current_index, 0, self.count - 1)) if self.current_index >= 0 else 0
        return np.asarray(self.tcp_path[idx], dtype=float)

    @classmethod
    def from_trajectory(
        cls,
        trajectory: Any,
        current_index: int = -1,
        tolerance_mm: float = 1.0,
        robot_name: str = "",
        extra: Optional[Dict[str, Any]] = None,
    ) -> "OverlayContext":
        """从 :class:`~service.simulation.Trajectory` 构建上下文。

        Args:
            trajectory: 规划结果；``None`` 或空轨迹时返回空上下文（不报错——没轨迹
                也要能画场景装饰，插件自己看 ``count`` 决定画不画）
            current_index: 当前路点序号
            tolerance_mm: 容差
            robot_name: 机器人名
            extra: 额外数据
        """
        waypoints = list(getattr(trajectory, "waypoints", []) or [])
        if not waypoints:
            return cls(
                current_index=current_index,
                tolerance_mm=tolerance_mm,
                robot_name=robot_name,
                extra=dict(extra or {}),
            )
        tcp = []
        joints = []
        times = []
        dof = None
        for wp in waypoints:
            pose = getattr(wp, "pose", None)
            if pose is None:
                tcp.append([np.nan, np.nan, np.nan])
            else:
                tcp.append(list(pose.to_xyz_rpy()[:3]))
            q = np.asarray(getattr(wp, "joints", []), dtype=float).ravel()
            if dof is None:
                dof = q.size
            joints.append(q)
            times.append(float(getattr(wp, "timestamp", 0.0)))
        joint_arr = np.asarray(joints, dtype=float) if dof else np.zeros((len(waypoints), 0))
        return cls(
            tcp_path=np.asarray(tcp, dtype=float),
            joint_path=joint_arr,
            timestamps=np.asarray(times, dtype=float),
            current_index=current_index,
            tolerance_mm=tolerance_mm,
            robot_name=robot_name,
            extra=dict(extra or {}),
        )


@dataclass
class OverlayCollection:
    """所有渲染插件本轮产出的覆盖层（附失败清单）。

    Attributes:
        drawables: ``[(插件名, 图元), ...]``，顺序即绘制顺序
        errors: 本轮失败的插件记录
    """

    drawables: List[Tuple[str, Any]] = field(default_factory=list)
    errors: List[PluginLoadError] = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.drawables)

    @property
    def ok(self) -> bool:
        """是否没有插件出错。"""
        return not self.errors

    def of_type(self, cls: type) -> List[Tuple[str, Any]]:
        """按图元类型过滤。"""
        return [(n, d) for n, d in self.drawables if isinstance(d, cls)]

    def summary(self) -> str:
        """一行摘要。"""
        return f"{len(self.drawables)} 个图元 / {len(self.errors)} 个插件出错"


def collect_overlays(
    context: OverlayContext,
    registry: Optional[PluginRegistry] = None,
    only: Optional[Iterable[str]] = None,
) -> OverlayCollection:
    """逐个调用渲染插件，收集图元；**单个插件出错不影响其余插件**。

    Args:
        context: :class:`OverlayContext`
        registry: 注册表；``None`` 用 :data:`default_registry`
        only: 只跑这些插件名（``None`` 表示全部渲染插件）

    Returns:
        :class:`OverlayCollection`
    """
    reg = registry if registry is not None else default_registry
    wanted = None if only is None else {str(n) for n in only}
    out = OverlayCollection()
    for spec in reg.specs(KIND_RENDERER):
        if wanted is not None and spec.name not in wanted:
            continue
        try:
            renderer = reg.create_renderer(spec.name)
        except Exception as exc:
            out.errors.append(
                PluginLoadError(
                    name=spec.name,
                    source="create",
                    message=f"渲染插件构造失败：{exc}",
                    detail=_brief_traceback(),
                )
            )
            continue
        try:
            produced = renderer.overlays(context)
        except Exception as exc:
            out.errors.append(
                PluginLoadError(
                    name=spec.name,
                    source="render",
                    message=f"overlays() 抛错：{exc}",
                    detail=_brief_traceback(),
                )
            )
            continue
        if produced is None:
            continue
        if isinstance(produced, OVERLAY_TYPES):
            produced = [produced]
        for item in produced:
            if not isinstance(item, OVERLAY_TYPES):
                out.errors.append(
                    PluginLoadError(
                        name=spec.name,
                        source="render",
                        message=(
                            f"返回了不认识的图元 {type(item).__name__}；"
                            f"允许：{[t.__name__ for t in OVERLAY_TYPES]}"
                        ),
                    )
                )
                continue
            out.drawables.append((spec.name, item))
    return out
