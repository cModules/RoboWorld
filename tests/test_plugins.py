"""插件接口（v1.0.4）的回归测试。

四个扩展点面分别守住：

1. **登记与查找**（:class:`PluginSpec` / :class:`PluginRegistry`）——名字冲突、
   替换、按扩展点过滤、kind 校验，以及"来源由宿主判定"；
2. **API 版本与延迟加载**——主版本必须相等；``"module:attr"`` 工厂要到
   ``create()`` 那一刻才 import（否则一个依赖 GUI/SDK 的插件会拖垮宿主启动）；
3. **失败隔离**——坏插件不得拖垮整张注册表，也不得影响其余渲染插件出图；
4. **三处真实集成**——``post.get_post_processor`` 取到插件品牌并出码、
   ``Robot(kinematics_plugin=...)`` 真的换了求解器（且失败时回退内置）、
   覆盖层能挂在真实 UR5e 轨迹上。
"""

import dataclasses
import os
import subprocess
import sys
import textwrap
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from domain.pose import Pose
from domain.robot import Robot
from service.program import ProgramBuilder  # noqa: E402
from adapter import plugins as P  # noqa: E402
from domain.kinematics import RobotKinematics  # noqa: E402
from service.planner import TrajectoryPlanner  # noqa: E402
from adapter.plugins import (
    KIND_KINEMATICS,
    KIND_POST,
    KIND_RENDERER,
    PLUGIN_API_VERSION,
    PLUGIN_ENTRY_POINT_GROUP,
    FrameMarker,
    KinematicsReport,
    LineSegments,
    OverlayCollection,
    OverlayContext,
    PluginError,
    PluginLoadError,
    PluginRegistry,
    PluginSpec,
    PointCloud,
    api_compatible,
    check_kinematics,
    collect_overlays,
)
from service.simulation import Trajectory  # noqa: E402

URDF = str(REPO_ROOT / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf")
HOME = [0.0, -90.0, 90.0, -90.0, 90.0, 0.0]
AWAY = [10.0, -70.0, 70.0, -90.0, 90.0, 10.0]


# --------------------------------------------------------------------------- #
# 夹具与桩件
# --------------------------------------------------------------------------- #


@pytest.fixture
def registry() -> PluginRegistry:
    """每次一个全新注册表——单元测试不该碰进程级的 ``default_registry``。"""
    return PluginRegistry()


@pytest.fixture
def global_registry():
    """用 ``default_registry`` 做集成测试，用完把登记项原样还原。

    必须还原：其它测试（如 ``test_post.py``）会断言后处理器清单，被本文件
    留下的插件污染就会假失败。
    """
    reg = P.default_registry
    saved_specs = dict(reg._specs)
    saved_errors = list(reg._errors)
    saved_groups = set(reg._loaded_groups)
    reg.clear()
    try:
        yield reg
    finally:
        reg._specs.clear()
        reg._specs.update(saved_specs)
        reg._errors[:] = saved_errors
        reg._loaded_groups.clear()
        reg._loaded_groups.update(saved_groups)


class _SpyFactory:
    """记录自己被调用了几次、带了什么参数。"""

    def __init__(self, result=None):
        self.calls = []
        self.result = result

    def __call__(self, **kwargs):
        self.calls.append(kwargs)
        return self.result


class _CompleteKinematics(RobotKinematics):
    """一个契约完整的运动学桩件（真的能算 FK/IK）。"""

    def __init__(self, urdf_path, gain=1.0):
        super().__init__(urdf_path)
        self.gain = gain


class _PartialKinematics:
    """只实现 FK/IK 的"残缺"实现——缺必需项应被拒，缺可选项只记警告。"""

    dof = 6
    joint_names = [f"j{i}" for i in range(6)]

    @staticmethod
    def forward_kinematics(q, q_is_radians=True):  # noqa: D102
        return Pose()

    @staticmethod
    def inverse_kinematics(pose, seed=None):  # noqa: D102
        return [0.0] * 6


class _BareKinematics:
    """连 FK 都没有——必需项缺失。"""

    dof = 3
    joint_names = ["a", "b", "c"]


class _FakeEntryPoint:
    """仿 ``importlib.metadata.EntryPoint``：只要 ``name`` / ``load()``。"""

    def __init__(self, name, payload=None, raises=None):
        self.name = name
        self._payload = payload
        self._raises = raises

    def load(self):
        if self._raises is not None:
            raise self._raises
        return self._payload


@pytest.fixture
def ur5e():
    return Robot(URDF, "UR5e")


def _simple_program(robot=None) -> object:
    robot = robot or Robot(URDF, "UR5e")
    return ProgramBuilder(robot).moveJ(HOME, speed=60).moveJ(AWAY, speed=60).build()


# --------------------------------------------------------------------------- #
# 1) API 版本
# --------------------------------------------------------------------------- #


def test_parse_api_version_basic():
    assert P._parse_api_version("1.0") == (1, 0)
    assert P._parse_api_version("2") == (2, 0)
    assert P._parse_api_version("1.7") == (1, 7)
    assert P._parse_api_version(" 3.4 ") == (3, 4)


def test_parse_api_version_allows_junk_minor():
    # 副版本写错不致命：主版本能读出来就算过，避免"1.x-dev"这类写法直接炸
    assert P._parse_api_version("1.dev") == (1, 0)


@pytest.mark.parametrize("bad", ["", None, "abc", ".1", "-1.0"])
def test_parse_api_version_rejects_garbage(bad):
    with pytest.raises(PluginError):
        P._parse_api_version(bad)


def test_api_compatible_same_major():
    assert api_compatible("1.0", "1.0") is True
    assert api_compatible("1.0", "1.9") is True
    assert api_compatible("1.9", "1.0") is True


def test_api_compatible_major_mismatch():
    assert api_compatible("2.0", "1.0") is False
    assert api_compatible("0.9", "1.0") is False


def test_api_compatible_bad_input_is_false_not_raise():
    # 版本号写错是"不兼容"，不是"崩溃"——加载路径上不该抛
    assert api_compatible("nope") is False
    assert api_compatible("1.0", "nope") is False


def test_host_api_version_is_1_0():
    assert PLUGIN_API_VERSION == "1.0"
    assert PLUGIN_ENTRY_POINT_GROUP == "adapter.plugins"


# --------------------------------------------------------------------------- #
# 2) PluginSpec
# --------------------------------------------------------------------------- #


def test_spec_rejects_unknown_kind():
    with pytest.raises(PluginError, match="kind"):
        PluginSpec(name="x", kind="teleporter", factory=lambda: None)


def test_spec_rejects_blank_name():
    for bad in ("", "   "):
        with pytest.raises(PluginError, match="插件名"):
            PluginSpec(name=bad, kind=KIND_POST, factory=lambda: None)


def test_spec_rejects_incompatible_api():
    with pytest.raises(PluginError, match="主版本不匹配"):
        PluginSpec(name="x", kind=KIND_POST, factory=lambda: None, requires_api="2.0")


def test_spec_accepts_compatible_minor_api():
    spec = PluginSpec(name="x", kind=KIND_POST, factory=lambda: None, requires_api="1.5")
    assert spec.api_ok is True


def test_spec_create_passes_options():
    spy = _SpyFactory(result="made")
    spec = PluginSpec(name="x", kind=KIND_POST, factory=spy, options={"a": 1, "b": 2})
    assert spec.create() == "made"
    assert spy.calls == [{"a": 1, "b": 2}]


def test_spec_create_overrides_options():
    spy = _SpyFactory(result=None)
    spec = PluginSpec(name="x", kind=KIND_POST, factory=spy, options={"a": 1, "b": 2})
    spec.create(b=99, c=3)
    assert spy.calls == [{"a": 1, "b": 99, "c": 3}]


def test_spec_create_does_not_mutate_stored_options():
    spy = _SpyFactory()
    spec = PluginSpec(name="x", kind=KIND_POST, factory=spy, options={"a": 1})
    spec.create(a=2)
    assert spec.options == {"a": 1}


def test_spec_create_wraps_factory_error_with_cause():
    def boom(**kwargs):
        raise ValueError("内部参数不对")

    spec = PluginSpec(name="x", kind=KIND_POST, factory=boom)
    with pytest.raises(PluginError) as exc:
        spec.create()
    assert isinstance(exc.value.__cause__, ValueError)
    assert "内部参数不对" in str(exc.value)


def test_spec_create_keeps_plugin_error_unwrapped():
    def boom(**kwargs):
        raise PluginError("原始插件错误")

    spec = PluginSpec(name="x", kind=KIND_POST, factory=boom)
    with pytest.raises(PluginError, match="原始插件错误"):
        spec.create()


def test_spec_summary_uses_major_x_not_gte():
    spec = PluginSpec(
        name="acme",
        kind=KIND_POST,
        factory=lambda: None,
        version="1.2.3",
        description="一句话",
    )
    text = spec.summary()
    assert "acme" in text and "[post]" in text and "v1.2.3" in text
    assert "一句话" in text
    # "需 API 1.x" 而不是 ">=1.0"：兼容规则是主版本相等，写成 >= 会误导
    assert "需 API 1.x" in text
    assert ">=" not in text


def test_spec_summary_marks_non_manual_origin():
    manual = PluginSpec(name="a", kind=KIND_POST, factory=lambda: None)
    loaded = PluginSpec(name="b", kind=KIND_POST, factory=lambda: None, origin="entry_point")
    assert "来源" not in manual.summary()
    assert "来源=entry_point" in loaded.summary()


# --------------------------------------------------------------------------- #
# 3) 工厂解析与延迟加载
# --------------------------------------------------------------------------- #


def test_load_factory_accepts_callable():
    fn = lambda: 1  # noqa: E731
    assert P._load_factory(fn, "x") is fn


def test_load_factory_accepts_module_colon_attr():
    target = P._load_factory("adapter.plugins:PluginSpec", "x")
    assert target is PluginSpec


def test_load_factory_accepts_module_dot_attr():
    target = P._load_factory("adapter.plugins.PluginSpec", "x")
    assert target is PluginSpec


def test_load_factory_missing_module_raises():
    with pytest.raises(PluginError, match="导入插件模块"):
        P._load_factory("no_such_package_zzz:thing", "x")


def test_load_factory_missing_attr_raises():
    with pytest.raises(PluginError, match="没有"):
        P._load_factory("adapter.plugins:no_such_attr", "x")


def test_load_factory_non_callable_attr_raises():
    with pytest.raises(PluginError, match="不可调用"):
        P._load_factory("adapter.plugins:PLUGIN_API_VERSION", "x")


def test_load_factory_rejects_bad_ref():
    with pytest.raises(PluginError, match="非法"):
        P._load_factory("nocolonandnoattr", "x")


def test_load_factory_rejects_non_str_non_callable():
    with pytest.raises(PluginError, match="factory"):
        P._load_factory(12345, "x")


def test_module_attr_factory_is_imported_lazily(tmp_path, registry):
    """登记时绝不能 import——插件可能依赖 GUI/SDK。

    用临时模块 + 一个"已导入"标记文件来证明：注册完还看不到标记，``create()``
    之后才出现。
    """
    marker = tmp_path / "imported.flag"
    mod = tmp_path / "lazy_plugin_mod.py"
    mod.write_text(
        textwrap.dedent(f"""
            from pathlib import Path as _P

            _P({str(marker)!r}).write_text("imported", encoding="utf-8")


            class Thing:
                def __init__(self, tag="default"):
                    self.tag = tag
            """),
        encoding="utf-8",
    )
    sys.path.insert(0, str(tmp_path))
    sys.modules.pop("lazy_plugin_mod", None)
    try:
        registry.register_post_processor(
            "lazy_brand_1", "lazy_plugin_mod:Thing", options={"tag": "hi"}
        )
        assert not marker.exists(), "登记阶段就 import 了插件模块"
        assert "lazy_plugin_mod" not in sys.modules

        obj = registry.create("lazy_brand_1", kind=KIND_POST)
        assert obj.tag == "hi"
        assert marker.exists(), "create() 之后插件模块仍未 import"
    finally:
        sys.path.remove(str(tmp_path))
        sys.modules.pop("lazy_plugin_mod", None)


# --------------------------------------------------------------------------- #
# 4) 注册表：登记 / 查找 / 过滤
# --------------------------------------------------------------------------- #


def test_register_duplicate_name_raises(registry):
    registry.register_post_processor("dup_1", lambda: None)
    with pytest.raises(PluginError, match="已被占用"):
        registry.register_post_processor("dup_1", lambda: None)


def test_register_replace_overwrites(registry):
    first = registry.register_post_processor("dup_1", lambda: None, version="1.0")
    second = registry.register_post_processor("dup_1", lambda: None, version="2.0", replace=True)
    assert registry.find("dup_1") is second
    assert first is not second
    assert registry.find("dup_1").version == "2.0"
    assert len(registry) == 1


def test_register_requires_pluginspec(registry):
    with pytest.raises(PluginError, match="PluginSpec"):
        registry.register("not a spec")  # type: ignore[arg-type]


def test_post_processor_name_needs_underscore(registry):
    with pytest.raises(PluginError, match="brand_controller"):
        registry.register_post_processor("acme", lambda: None)


def test_register_kinematics_and_renderer_need_no_underscore(registry):
    registry.register_kinematics("acme.calibrated", lambda: None)
    registry.register_renderer("acme.overlay", lambda: None)
    assert set(registry.names()) == {"acme.calibrated", "acme.overlay"}


def test_len_and_contains(registry):
    assert len(registry) == 0 and "x" not in registry
    registry.register_renderer("x", lambda: None)
    assert len(registry) == 1 and "x" in registry


def test_find_returns_none_for_unknown(registry):
    assert registry.find("nope") is None


def test_get_unknown_raises_with_case_hint(registry):
    registry.register_renderer("Acme.Overlay", lambda: None)
    with pytest.raises(PluginError) as exc:
        registry.get("acme.overlay")
    assert "区分大小写" in str(exc.value)
    # 拼对了就能取到
    assert registry.get("Acme.Overlay").name == "Acme.Overlay"


def test_get_kind_mismatch_raises(registry):
    registry.register_renderer("r1", lambda: None)
    with pytest.raises(PluginError, match="不能当作"):
        registry.get("r1", kind=KIND_POST)
    assert registry.get("r1", kind=KIND_RENDERER).kind == KIND_RENDERER


def test_specs_filters_by_kind_and_sorts(registry):
    registry.register_post_processor("zeta_1", lambda: None)
    registry.register_post_processor("alpha_1", lambda: None)
    registry.register_renderer("mid.r", lambda: None)
    assert registry.names(KIND_POST) == ["alpha_1", "zeta_1"]
    assert registry.names(KIND_RENDERER) == ["mid.r"]
    assert len(registry.specs()) == 3


def test_unregister_reports_whether_removed(registry):
    registry.register_renderer("r1", lambda: None)
    assert registry.unregister("r1") is True
    assert registry.unregister("r1") is False
    assert len(registry) == 0


def test_clear_wipes_specs_errors_and_groups(registry):
    registry.register_renderer("r1", lambda: None)
    registry.add_error("r1", "create", "boom")
    registry.load_entry_points(entry_points=[])
    assert registry._loaded_groups
    registry.clear()
    assert len(registry) == 0
    assert registry.errors() == []
    assert not registry._loaded_groups


def test_errors_returns_copy(registry):
    registry.add_error("x", "create", "boom")
    snapshot = registry.errors()
    snapshot.clear()
    assert len(registry.errors()) == 1


def test_clear_errors_keeps_specs(registry):
    registry.register_renderer("r1", lambda: None)
    registry.add_error("r1", "render", "boom")
    registry.clear_errors()
    assert registry.errors() == []
    assert len(registry) == 1


def test_summary_lists_kinds_errors_and_empty_state():
    reg = PluginRegistry()
    assert "未登记任何插件" in reg.summary()
    reg.register_post_processor("acme_v2", lambda: None, description="d")
    reg.add_error("acme_v2", "create", "炸了")
    text = reg.summary()
    assert "[post]" in text and "acme_v2" in text
    assert "错误/警告（1）" in text and "[create]" in text
    assert PLUGIN_API_VERSION in text


# --------------------------------------------------------------------------- #
# 5) 运动学契约校验
# --------------------------------------------------------------------------- #


def test_check_kinematics_ok_on_real_implementation(ur5e):
    report = check_kinematics(ur5e.kinematics)
    assert report.ok
    assert report.missing_required == []
    assert report.dof == 6


def test_check_kinematics_missing_required():
    report = check_kinematics(_BareKinematics())
    assert not report.ok
    assert "forward_kinematics" in report.missing_required
    assert "inverse_kinematics" in report.missing_required
    assert "dof" not in report.missing_required  # 它有 dof
    assert "缺少必需成员" in report.summary()


def test_check_kinematics_missing_optional_is_warning_only():
    report = check_kinematics(_PartialKinematics())
    assert report.ok and report.missing_required == []
    assert "jacobian" in report.missing_optional
    assert "缺少可选成员" in report.summary()


def test_check_kinematics_complete_impl_has_no_optional_gap(ur5e):
    report = check_kinematics(ur5e.kinematics)
    assert report.ok and report.missing_optional == []
    assert report.summary().endswith("接口完整")
    assert "dof=6" in report.summary()


def test_check_kinematics_dof_unreadable_is_none():
    class Weird:
        dof = "六"

        def forward_kinematics(self, *a, **k):
            return None

        def inverse_kinematics(self, *a, **k):
            return None

        joint_names = []

    assert check_kinematics(Weird()).dof is None


def test_kinematics_report_ok_property():
    assert KinematicsReport().ok is True
    assert KinematicsReport(missing_required=["jacobian"]).ok is False


def test_create_kinematics_rejects_missing_required(registry):
    registry.register_kinematics("bad.k", lambda: _BareKinematics())
    with pytest.raises(PluginError, match="缺少必需成员"):
        registry.create("bad.k", kind=KIND_KINEMATICS)


def test_create_kinematics_records_optional_gaps_as_errors(registry):
    registry.register_kinematics("partial.k", lambda: _PartialKinematics())
    impl = registry.create("partial.k", kind=KIND_KINEMATICS)
    assert impl is not None
    # 缺 8 个可选成员只记 **1 条** 警告——否则 GUI 错误列表会被同一个插件刷屏
    errs = registry.errors()
    assert len(errs) == 1
    assert errs[0].name == "partial.k" and errs[0].source == "create"
    assert "jacobian" in errs[0].message
    assert "碰撞/动力学/限位" in errs[0].message


def test_create_kinematics_clean_impl_records_no_errors(registry):
    registry.register_kinematics(
        "good.k",
        lambda urdf_path, **kw: _CompleteKinematics(urdf_path, **kw),
        options={"gain": 2.0},
    )
    impl = registry.create("good.k", kind=KIND_KINEMATICS, urdf_path=URDF)
    assert impl.gain == 2.0
    assert registry.errors() == []


def test_create_kinematics_still_passes_options_through(registry):
    registry.register_kinematics(
        "good.k",
        lambda urdf_path, **kw: _CompleteKinematics(urdf_path, **kw),
        options={"gain": 1.0},
    )
    impl = registry.create("good.k", kind=KIND_KINEMATICS, urdf_path=URDF, gain=7.0)
    assert impl.gain == 7.0


# --------------------------------------------------------------------------- #
# 6) 覆盖层数据类型（纯数据契约）
# --------------------------------------------------------------------------- #


def test_pointcloud_coerces_to_ndarray():
    pc = PointCloud(positions=[[0, 0, 0], [1, 2, 3]])
    assert isinstance(pc.positions, np.ndarray)
    assert pc.positions.shape == (2, 3)


def test_pointcloud_accepts_ndarray():
    arr = np.zeros((4, 3))
    assert PointCloud(positions=arr).positions.shape == (4, 3)


@pytest.mark.parametrize("bad", [[0, 0, 0], np.zeros((3, 2)), np.zeros(3), np.zeros((2, 3, 4))])
def test_pointcloud_rejects_bad_shape(bad):
    with pytest.raises(PluginError, match=r"\(N, 3\)"):
        PointCloud(positions=bad)


def test_linestyles_and_frame_defaults():
    ls = LineSegments(segments=[([0, 0, 0], [1, 1, 1])])
    assert ls.width == 2.0 and len(ls.color) == 3
    fm = FrameMarker(pose=Pose())
    assert fm.scale == 60.0 and len(fm.color) == 3


def test_overlay_types_tuple_contents():
    assert set(P.OVERLAY_TYPES) == {LineSegments, PointCloud, FrameMarker}


# --------------------------------------------------------------------------- #
# 7) OverlayContext
# --------------------------------------------------------------------------- #


def test_overlay_context_defaults_are_empty():
    ctx = OverlayContext()
    assert ctx.count == 0
    assert ctx.current_tcp() is None
    assert ctx.current_index == -1


def test_overlay_context_current_tcp_clamps_index():
    ctx = OverlayContext(tcp_path=np.arange(9, dtype=float).reshape(3, 3))
    assert ctx.count == 3
    np.testing.assert_allclose(ctx.current_tcp(), [0, 1, 2])
    ctx.current_index = 2
    np.testing.assert_allclose(ctx.current_tcp(), [6, 7, 8])
    ctx.current_index = 99
    np.testing.assert_allclose(ctx.current_tcp(), [6, 7, 8])
    ctx.current_index = -5
    np.testing.assert_allclose(ctx.current_tcp(), [0, 1, 2])


def test_overlay_context_from_real_trajectory(ur5e):
    program = _simple_program(ur5e)
    traj = TrajectoryPlanner(ur5e).plan(program)
    ctx = OverlayContext.from_trajectory(
        traj,
        current_index=3,
        tolerance_mm=0.5,
        robot_name="UR5e",
        extra={"reference_path": "ref"},
    )
    n = len(traj.waypoints)
    assert ctx.count == n > 3
    assert ctx.tcp_path.shape == (n, 3)
    assert ctx.joint_path.shape == (n, 6)
    assert ctx.timestamps.shape == (n,)
    assert np.all(np.diff(ctx.timestamps) >= -1e-12)  # 时刻单调不减
    assert ctx.current_index == 3
    assert ctx.tolerance_mm == 0.5
    assert ctx.robot_name == "UR5e"
    assert ctx.extra == {"reference_path": "ref"}
    assert np.isfinite(ctx.tcp_path).all()


def test_overlay_context_from_empty_trajectory_does_not_raise():
    ctx = OverlayContext.from_trajectory(Trajectory([]), robot_name="R")
    assert ctx.count == 0 and ctx.robot_name == "R"
    assert ctx.extra == {}


def test_overlay_context_from_none_trajectory_does_not_raise():
    ctx = OverlayContext.from_trajectory(None)
    assert ctx.count == 0


def test_overlay_context_marks_nan_when_pose_missing():
    class _WP:
        pose = None
        joints = [0.0] * 6
        timestamp = 0.0

    class _Traj:
        waypoints = [_WP(), _WP()]

    ctx = OverlayContext.from_trajectory(_Traj())
    assert ctx.count == 2
    assert np.isnan(ctx.tcp_path).all()


def test_overlay_context_extra_is_copied_not_aliased():
    src = {"a": 1}
    ctx = OverlayContext.from_trajectory(None, extra=src)
    src["a"] = 999
    assert ctx.extra == {"a": 1}


# --------------------------------------------------------------------------- #
# 8) collect_overlays：失败隔离
# --------------------------------------------------------------------------- #


def _ctx(n=5):
    return OverlayContext(tcp_path=np.arange(3 * n, dtype=float).reshape(n, 3))


def test_create_renderer_requires_overlays_hook(registry):
    class NoHook:
        pass

    registry.register_renderer("nohook", lambda: NoHook())
    with pytest.raises(PluginError, match="overlays"):
        registry.create_renderer("nohook")


def test_collect_overlays_single_plugin(registry):
    class One:
        def overlays(self, ctx):
            return PointCloud(positions=ctx.tcp_path)

    registry.register_renderer("one", lambda: One())
    out = collect_overlays(_ctx(4), registry=registry)
    assert isinstance(out, OverlayCollection)
    assert out.ok and len(out) == 1
    assert out.drawables[0][0] == "one"
    assert isinstance(out.drawables[0][1], PointCloud)
    assert "1 个图元 / 0 个插件出错" in out.summary()


def test_collect_overlays_unwraps_bare_drawable(registry):
    """插件只返回一个图元（不是列表）也要收下。"""

    class One:
        def overlays(self, ctx):
            return FrameMarker(pose=Pose())

    registry.register_renderer("one", lambda: One())
    out = collect_overlays(_ctx(4), registry=registry)
    assert len(out) == 1 and isinstance(out.drawables[0][1], FrameMarker)


def test_collect_overlays_none_result_is_skipped(registry):
    class Noneish:
        def overlays(self, ctx):
            return None

    registry.register_renderer("none_ish", lambda: Noneish())
    out = collect_overlays(_ctx(4), registry=registry)
    assert len(out) == 0 and out.ok


def test_collect_overlays_expands_list_result(registry):
    class Many:
        def overlays(self, ctx):
            return [
                PointCloud(positions=ctx.tcp_path),
                LineSegments(segments=[([0, 0, 0], [1, 1, 1])]),
            ]

    registry.register_renderer("many", lambda: Many())
    out = collect_overlays(_ctx(4), registry=registry)
    assert len(out) == 2
    assert names_of(out) == ["many", "many"]


def names_of(collection):
    return [n for n, _ in collection.drawables]


def test_collect_overlays_isolates_runtime_failure(registry):
    """一个插件抛错，其余插件照常出图——这是整个生态能活下去的前提。"""

    class Boom:
        def overlays(self, ctx):
            raise RuntimeError("插件内部炸了")

    class Good:
        def overlays(self, ctx):
            return PointCloud(positions=ctx.tcp_path)

    registry.register_renderer("a_boom", lambda: Boom())
    registry.register_renderer("b_good", lambda: Good())
    out = collect_overlays(_ctx(4), registry=registry)
    assert names_of(out) == ["b_good"]
    assert not out.ok and len(out.errors) == 1
    err = out.errors[0]
    assert err.name == "a_boom" and err.source == "render"
    assert "插件内部炸了" in err.message
    assert err.detail  # 留了 traceback 便于定位


def test_collect_overlays_isolates_construction_failure(registry):
    def boom_factory():
        raise ValueError("构造就炸")

    class Good:
        def overlays(self, ctx):
            return PointCloud(positions=ctx.tcp_path)

    registry.register_renderer("a_bad", boom_factory)
    registry.register_renderer("b_good", lambda: Good())
    out = collect_overlays(_ctx(4), registry=registry)
    assert names_of(out) == ["b_good"]
    assert out.errors[0].source == "create"
    assert "构造就炸" in out.errors[0].message


def test_collect_overlays_rejects_unknown_drawable_type(registry):
    class Wrong:
        def overlays(self, ctx):
            return [object(), PointCloud(positions=ctx.tcp_path)]

    registry.register_renderer("wrong", lambda: Wrong())
    out = collect_overlays(_ctx(4), registry=registry)
    # 不认识的图元被拒绝并记错误，但同一插件里合法的那个仍然收下
    assert len(out) == 1
    assert len(out.errors) == 1 and out.errors[0].source == "render"
    assert "不认识的图元" in out.errors[0].message


def test_collect_overlays_only_filter(registry):
    class One:
        def overlays(self, ctx):
            return PointCloud(positions=ctx.tcp_path)

    registry.register_renderer("keep_me", lambda: One())
    registry.register_renderer("skip_me", lambda: One())
    out = collect_overlays(_ctx(4), registry=registry, only=["keep_me"])
    assert names_of(out) == ["keep_me"]
    out2 = collect_overlays(_ctx(4), registry=registry, only=[])
    assert len(out2) == 0


def test_collect_overlays_draw_order_is_sorted_by_name(registry):
    class One:
        def overlays(self, ctx):
            return PointCloud(positions=ctx.tcp_path)

    for n in ("zzz", "aaa", "mmm"):
        registry.register_renderer(n, lambda: One())
    assert names_of(collect_overlays(_ctx(4), registry=registry)) == ["aaa", "mmm", "zzz"]


def test_collect_overlays_empty_registry(registry):
    out = collect_overlays(_ctx(4), registry=registry)
    assert len(out) == 0 and out.ok and out.of_type(PointCloud) == []


def test_overlay_collection_of_type_filters():
    coll = OverlayCollection(
        drawables=[("a", PointCloud(positions=[[0, 0, 0]])), ("b", FrameMarker(Pose()))]
    )
    assert [n for n, _ in coll.of_type(FrameMarker)] == ["b"]
    assert coll.ok is True


# --------------------------------------------------------------------------- #
# 9) 入口点发现
# --------------------------------------------------------------------------- #


def test_load_entry_points_from_fake_eps(registry):
    eps = [
        _FakeEntryPoint("a", PluginSpec("a_brand_1", KIND_POST, lambda: None)),
        _FakeEntryPoint("b", PluginSpec("b.k", KIND_KINEMATICS, lambda: None)),
    ]
    stats = registry.load_entry_points(entry_points=eps)
    assert stats == {"discovered": 2, "registered": 2, "skipped": 0, "failed": 0}
    assert registry.names() == ["a_brand_1", "b.k"]


def test_load_entry_points_accepts_spec_list(registry):
    payload = [
        PluginSpec("x.post_1", KIND_POST, lambda: None),
        PluginSpec("x.post_2", KIND_POST, lambda: None),
    ]
    stats = registry.load_entry_points(entry_points=[_FakeEntryPoint("x", payload)])
    assert stats["registered"] == 2
    assert stats["discovered"] == 1


def test_load_entry_points_accepts_provider_function(registry):
    def provider():
        return PluginSpec("f.renderer", KIND_RENDERER, lambda: None)

    stats = registry.load_entry_points(entry_points=[_FakeEntryPoint("provider", provider)])
    assert stats["registered"] == 1
    assert "f.renderer" in registry


def test_load_entry_points_stamps_origin_without_touching_original(registry):
    spec = PluginSpec("origin.k", KIND_KINEMATICS, lambda: None)
    registry.load_entry_points(entry_points=[_FakeEntryPoint("origin", spec)])
    assert registry.get("origin.k").origin == "entry_point"
    assert spec.origin == "manual", "不应该改到调用方手上的 spec 对象"


def test_load_entry_points_skips_existing_names(registry):
    registry.register_post_processor("dup_1", lambda: None)
    stats = registry.load_entry_points(
        entry_points=[_FakeEntryPoint("dup", PluginSpec("dup_1", KIND_POST, lambda: None))]
    )
    assert stats == {"discovered": 1, "registered": 0, "skipped": 1, "failed": 0}
    assert registry.get("dup_1").origin == "manual"


def test_load_entry_points_replace_true_overwrites(registry):
    registry.register_post_processor("dup_1", lambda: None, version="0.0.1")
    registry.load_entry_points(
        entry_points=[
            _FakeEntryPoint("dup", PluginSpec("dup_1", KIND_POST, lambda: None, version="9.9"))
        ],
        replace=True,
    )
    assert registry.get("dup_1").version == "9.9"
    assert registry.get("dup_1").origin == "entry_point"


def test_load_entry_points_isolates_load_failure(registry):
    eps = [
        _FakeEntryPoint("bad", raises=ImportError("插件包缺失")),
        _FakeEntryPoint("good", PluginSpec("good_1", KIND_POST, lambda: None)),
    ]
    stats = registry.load_entry_points(entry_points=eps)
    assert stats == {"discovered": 2, "registered": 1, "skipped": 0, "failed": 1}
    assert "good_1" in registry
    err = registry.errors()[0]
    assert err.name == "bad" and err.source == "entry_point"
    assert "插件包缺失" in err.message


def test_load_entry_points_rejects_bad_payload_type(registry):
    stats = registry.load_entry_points(entry_points=[_FakeEntryPoint("bad", 42)])
    assert stats["failed"] == 1 and stats["registered"] == 0
    assert registry.errors()[0].source == "register"
    assert "PluginSpec" in registry.errors()[0].message


def test_load_entry_points_rejects_empty_spec_list(registry):
    stats = registry.load_entry_points(entry_points=[_FakeEntryPoint("bad", [])])
    assert stats["failed"] == 1
    assert "空列表" in registry.errors()[0].message


def test_load_entry_points_rejects_non_spec_in_list(registry):
    payload = [PluginSpec("ok_1", KIND_POST, lambda: None), "not-a-spec"]
    stats = registry.load_entry_points(entry_points=[_FakeEntryPoint("bad", payload)])
    assert stats["failed"] == 1 and stats["registered"] == 0
    assert "只能放 PluginSpec" in registry.errors()[0].message


def test_load_entry_points_incompatible_api_counts_as_failure(registry):
    """API 主版本不匹配的插件要被拒——但只能拒它一个。

    ``PluginSpec`` 在构造时就拦这种版本，所以构造完再改字段来模拟"绕过构造
    校验拿到手上的 spec"；登记入口会在真正收下之前重跑一次校验。
    """
    stale = PluginSpec("old_brand_1", KIND_POST, lambda: None)
    stale.requires_api = "2.0"
    eps = [
        _FakeEntryPoint("old", stale),
        _FakeEntryPoint("new", PluginSpec("new_brand_1", KIND_POST, lambda: None)),
    ]
    stats = registry.load_entry_points(entry_points=eps)
    assert stats["failed"] == 1 and stats["registered"] == 1
    assert "主版本不匹配" in registry.errors()[0].message
    assert "new_brand_1" in registry and "old_brand_1" not in registry


def test_load_entry_points_records_loaded_group(registry):
    registry.load_entry_points(entry_points=[])
    assert PLUGIN_ENTRY_POINT_GROUP in registry._loaded_groups


def test_coerce_entry_point_rejects_plain_object(registry):
    with pytest.raises(PluginError, match="PluginSpec"):
        registry._coerce_entry_point("ep", object())


def test_package_level_helpers_forward_to_default_registry(global_registry):
    P.register_renderer("pkg.r1", lambda: None, version="1.0.0")
    assert "pkg.r1" in P.list_plugins(KIND_RENDERER)
    assert "pkg.r1" in P.plugin_names()
    assert P.find_plugin("pkg.r1") is not None
    assert P.get_plugin("pkg.r1").name == "pkg.r1"
    assert any("pkg.r1" in line for line in P.describe_plugins())
    assert P.find_plugin("nope") is None
    assert P.unregister("pkg.r1") is True
    assert P.list_plugins() == []


def test_package_level_create_and_errors(global_registry):
    P.register_renderer("pkg.r2", lambda: FrameMarkerHolder(), version="1.0.0")
    obj = P.create_plugin("pkg.r2", kind=KIND_RENDERER)
    assert isinstance(obj, FrameMarkerHolder)
    assert P.plugin_errors() == []
    P.clear_plugins()
    assert P.list_plugins() == []


class FrameMarkerHolder:
    def overlays(self, ctx):
        return FrameMarker(pose=Pose())


# --------------------------------------------------------------------------- #
# 10) 集成：后处理器
# --------------------------------------------------------------------------- #


def _acme_post_factory():
    """一个真实可用的第三方品牌后处理器（极简 ACME-V2 语言）。"""
    from adapter.post.base import BasePostProcessor, PostConfig

    class AcmePost(BasePostProcessor):
        brand = "acme"
        controller = "v2"

        def __init__(self):
            super().__init__(PostConfig(precision=2, speed_unit="%", comment_char="#"))

        @property
        def language_name(self) -> str:
            return "ACME-V2"

        @property
        def file_extension(self) -> str:
            return "acm"

        def _convert_motion(self, motion):
            return {
                "cmd": "MOVEJ",
                "target": self.fmt_joints(motion.target),
                "speed": f"{motion.speed:.0f}%",
            }

        def _get_template(self) -> str:
            return (
                "{{ config.comment_char }} {{ meta.program_name }}\n"
                "{% for wp in waypoints %}{{ wp.cmd }} {{ wp.target }} @ {{ wp.speed }}\n"
                "{% endfor %}"
            )

    return AcmePost()


def test_plugin_post_appears_in_list(global_registry):
    from adapter.post import list_post_processors

    before = list_post_processors()
    P.register_post_processor("acme_v2", _acme_post_factory, version="1.0.0")
    after = list_post_processors()
    assert "acme_v2" in after and "acme_v2" not in before
    assert set(after) - set(before) == {"acme_v2"}


def test_get_post_processor_by_exact_plugin_name(global_registry):
    from adapter.post import get_post_processor

    P.register_post_processor("acme_v2", _acme_post_factory)
    pp = get_post_processor("acme_v2")
    assert type(pp).__name__ == "AcmePost"
    assert pp.full_name == "acme_v2"
    # 大小写不敏感
    assert type(get_post_processor("ACME_V2")).__name__ == "AcmePost"


def test_get_post_processor_by_plugin_brand_fallback(global_registry):
    from adapter.post import get_post_processor

    P.register_post_processor("acme_v2", _acme_post_factory)
    pp = get_post_processor("acme")  # 只给品牌，落到那唯一一个 acme_* 上
    assert pp.full_name == "acme_v2"


def test_get_post_processor_ambiguous_brand_raises(global_registry):
    from adapter.post import get_post_processor

    P.register_post_processor("acme_v2", _acme_post_factory)
    P.register_post_processor("acme_v3", _acme_post_factory)
    with pytest.raises(Exception, match="多个后处理器"):
        get_post_processor("acme")


def test_builtin_post_untouched_by_plugin(global_registry):
    from adapter.post import get_post_processor

    P.register_post_processor("acme_v2", _acme_post_factory)
    assert get_post_processor("kuka_krc4").full_name == "kuka_krc4"
    assert get_post_processor("kuka").full_name == "kuka_krc4"


def test_plugin_post_generates_code_end_to_end(global_registry):
    P.register_post_processor("acme_v2", _acme_post_factory, version="1.0.0")
    program = _simple_program()
    code = program.to_post("acme_v2", metadata={"program_name": "PLUGIN_DEMO"})
    assert "PLUGIN_DEMO" in code
    assert "MOVEJ" in code
    assert code.count("MOVEJ") == 2
    assert "60%" in code
    # 注释前缀来自插件自己的 PostConfig，而不是任何内置品牌
    assert code.startswith("# PLUGIN_DEMO")


def test_plugin_post_rejects_foreign_object(global_registry):
    from adapter.post import get_post_processor

    P.register_post_processor("acme_v2", lambda: object())
    with pytest.raises(Exception, match="BasePostProcessor"):
        get_post_processor("acme_v2")


def test_resolve_for_brand_swallows_broken_plugin(global_registry):
    from adapter.post import resolve_for_brand

    def boom():
        raise RuntimeError("第三方代码炸了")

    P.register_post_processor("acme_v2", boom)
    # 自动选后处理器不该因为一个坏插件整体失败
    assert resolve_for_brand("acme") is None
    # 内置品牌不受影响
    assert resolve_for_brand("kuka").full_name == "kuka_krc4"


def test_post_module_survives_plugins_module_failure(global_registry):
    """插件层整体不可用时，内置品牌仍要能取到（延迟导入的容错）。"""
    from adapter.post import get_post_processor, list_post_processors

    assert "kuka_krc4" in list_post_processors()
    assert get_post_processor("fanuc_r30ib").full_name == "fanuc_r30ib"


# --------------------------------------------------------------------------- #
# 11) 集成：Robot 运动学插件
# --------------------------------------------------------------------------- #


class _OffsetKinematics(RobotKinematics):
    """把关节零位偏置一刀切掉——用来证明"真的换了求解器"。"""

    def __init__(self, urdf_path, offset_deg=0.0):
        super().__init__(urdf_path)
        self.offset_deg = float(offset_deg)

    def forward_kinematics(self, q, q_is_radians=True):
        q = np.asarray(q, dtype=float).ravel()
        shift = np.radians(self.offset_deg) if q_is_radians else self.offset_deg
        return super().forward_kinematics(q - shift, q_is_radians=q_is_radians)


def test_robot_uses_kinematics_plugin(global_registry):
    P.register_kinematics(
        "acme.calibrated",
        lambda urdf_path, **kw: _OffsetKinematics(urdf_path, **kw),
        version="1.0.0",
        options={"offset_deg": 0.5},
    )
    robot = Robot(URDF, "UR5e", kinematics_plugin="acme.calibrated")
    assert robot.kinematics_plugin == "acme.calibrated"
    assert isinstance(robot.kinematics, _OffsetKinematics)
    assert robot.kinematics.offset_deg == 0.5
    assert "契约通过" in robot.kinematics_contract
    assert "dof=6" in robot.kinematics_contract


def test_robot_kinematics_plugin_differs_from_builtin(global_registry):
    P.register_kinematics(
        "acme.calibrated",
        lambda urdf_path, **kw: _OffsetKinematics(urdf_path, **kw),
        options={"offset_deg": 1.0},
    )
    builtin = Robot(URDF, "UR5e")
    plugged = Robot(URDF, "UR5e", kinematics_plugin="acme.calibrated")
    for r in (builtin, plugged):
        r.set_joints(AWAY)
    d = np.linalg.norm(
        np.asarray(plugged.forward_kinematics().to_xyz_rpy()[:3])
        - np.asarray(builtin.forward_kinematics().to_xyz_rpy()[:3])
    )
    assert d > 1.0, "插件求解器与内置求解器结果应当不同"


def test_robot_options_override_registry_defaults(global_registry):
    P.register_kinematics(
        "acme.calibrated",
        lambda urdf_path, **kw: _OffsetKinematics(urdf_path, **kw),
        options={"offset_deg": 0.5},
    )
    robot = Robot(
        URDF,
        "UR5e",
        kinematics_plugin="acme.calibrated",
        kinematics_options={"offset_deg": 3.0},
    )
    assert robot.kinematics.offset_deg == 3.0


def test_robot_falls_back_when_plugin_missing(global_registry):
    robot = Robot(URDF, "UR5e", kinematics_plugin="not.registered")
    kin = robot.kinematics
    assert kin is not None
    assert isinstance(kin, RobotKinematics)
    assert not isinstance(kin, _OffsetKinematics)
    # 回退后不再自称用了插件，且原因进了注册表错误记录
    assert robot.kinematics_plugin is None
    errs = [e for e in P.plugin_errors() if e.name == "not.registered"]
    assert errs and "回退内置求解器" in errs[0].message


def test_robot_falls_back_when_plugin_is_broken(global_registry):
    def boom(urdf_path, **kwargs):
        raise RuntimeError("厂商 SDK 没装")

    P.register_kinematics("acme.broken", boom)
    robot = Robot(URDF, "UR5e", kinematics_plugin="acme.broken")
    assert robot.kinematics is not None
    assert robot.kinematics_plugin is None
    assert any("厂商 SDK 没装" in e.message for e in P.plugin_errors())


def test_robot_falls_back_when_plugin_contract_incomplete(global_registry):
    P.register_kinematics("acme.partial", lambda urdf_path, **kw: _BareKinematics())
    robot = Robot(URDF, "UR5e", kinematics_plugin="acme.partial")
    assert robot.kinematics is not None  # 回退内置，不是崩掉
    assert robot.kinematics_plugin is None
    assert any("缺少必需成员" in e.message for e in P.plugin_errors())


def test_robot_without_plugin_has_no_contract(global_registry):
    robot = Robot(URDF, "UR5e")
    assert robot.kinematics_plugin is None
    assert robot.kinematics_contract is None
    assert robot.kinematics is not None


def test_robot_kinematics_lazy_and_cached(global_registry):
    P.register_kinematics(
        "acme.calibrated", lambda urdf_path, **kw: _OffsetKinematics(urdf_path, **kw)
    )
    robot = Robot(URDF, "UR5e", kinematics_plugin="acme.calibrated")
    first = robot.kinematics
    assert robot.kinematics is first  # 惰性构建后缓存住，不重复造


def test_robot_with_plugin_still_plans(global_registry):
    """换了求解器之后，规划器照样能跑（插件不是只用来展示的）。"""
    P.register_kinematics(
        "acme.calibrated",
        lambda urdf_path, **kw: _OffsetKinematics(urdf_path, **kw),
        options={"offset_deg": 0.2},
    )
    robot = Robot(URDF, "UR5e", kinematics_plugin="acme.calibrated")
    traj = TrajectoryPlanner(robot).plan(_simple_program(robot))
    assert traj is not None and len(traj.waypoints) > 3


# --------------------------------------------------------------------------- #
# 12) 集成：渲染插件挂在真实轨迹上
# --------------------------------------------------------------------------- #


def test_renderer_receives_real_trajectory_and_returns_overlays(global_registry):
    seen = {}

    class Recorder:
        def overlays(self, ctx):
            seen["count"] = ctx.count
            seen["robot"] = ctx.robot_name
            seen["tol"] = ctx.tolerance_mm
            return [
                LineSegments(segments=[([0, 0, 0], [1, 1, 1])]),
                PointCloud(positions=ctx.tcp_path),
                FrameMarker(pose=Pose()),
            ]

    P.register_renderer("acme.recorder", lambda: Recorder(), version="1.0.0")
    robot = Robot(URDF, "UR5e")
    traj = TrajectoryPlanner(robot).plan(_simple_program(robot))
    ctx = OverlayContext.from_trajectory(
        traj, current_index=2, tolerance_mm=0.25, robot_name=robot.name
    )
    out = collect_overlays(ctx)
    assert out.ok
    assert seen == {"count": len(traj.waypoints), "robot": "UR5e", "tol": 0.25}
    assert len(out.of_type(PointCloud)) == 1
    assert len(out.of_type(LineSegments)) == 1
    assert len(out.of_type(FrameMarker)) == 1


def test_collect_overlays_uses_default_registry_when_none(global_registry):
    class One:
        def overlays(self, ctx):
            return FrameMarker(pose=Pose())

    P.register_renderer("acme.one", lambda: One())
    out = collect_overlays(_ctx(3))
    assert names_of(out) == ["acme.one"]


def test_renderer_can_be_driven_by_extra_channel(global_registry):
    """``ctx.extra`` 是宿主不认识、原样透传的通道——插件自定义数据走这里。"""

    class RefDiff:
        def overlays(self, ctx):
            ref = np.asarray(ctx.extra["reference_path"], dtype=float)
            dev = np.linalg.norm(ctx.tcp_path - ref, axis=1)
            over = ctx.tcp_path[dev > ctx.tolerance_mm]
            return [] if over.size == 0 else [PointCloud(positions=over)]

    P.register_renderer("acme.refdiff", lambda: RefDiff())
    path = _ctx(4)
    shifted = OverlayContext(
        tcp_path=path.tcp_path + np.array([2.0, 0.0, 0.0]),
        tolerance_mm=1.0,
        extra={"reference_path": path.tcp_path},
    )
    out = collect_overlays(shifted)
    assert out.ok and len(out) == 1
    assert out.drawables[0][1].positions.shape == (4, 3)


# --------------------------------------------------------------------------- #
# 13) 示例脚本：一条命令跑通
# --------------------------------------------------------------------------- #

DEMO = REPO_ROOT / "examples" / "plugins_demo.py"


def _run_demo() -> subprocess.CompletedProcess:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([str(REPO_ROOT), str(REPO_ROOT), env.get("PYTHONPATH", "")])
    return subprocess.run(
        [sys.executable, str(DEMO)],
        cwd=str(REPO_ROOT),
        env=env,
        capture_output=True,
        text=True,
        timeout=600,
    )


def test_plugin_demo_runs_clean():
    proc = _run_demo()
    assert proc.returncode == 0, (
        f"示例脚本退出码 {proc.returncode}\n--- stdout ---\n{proc.stdout[-2000:]}"
        f"\n--- stderr ---\n{proc.stderr[-2000:]}"
    )


def test_plugin_demo_covers_all_three_extension_points():
    out = _run_demo().stdout
    for marker in (
        "[1] 登记插件",
        "[2] 扫描已安装包的插件入口点",
        "[3] 后处理器清单",
        "[4] 运动学插件",
        "[5] 渲染插件",
        "[6] 以正式包分发插件",
    ):
        assert marker in out, f"缺少分段输出: {marker}"
    # 三个扩展点都真的产出了东西
    assert "acme_v2" in out
    assert "两个求解器的 TCP 差" in out
    assert "FrameMarker" in out and "PointCloud" in out
    assert "0 个插件出错" in out
    assert "核心代码一行未改" in out


def test_plugin_demo_reports_plugin_entry_point_snippet():
    out = _run_demo().stdout
    assert 'adapter.plugins"' in out  # 入口点组名是保留的对外契约，见 PLUGIN_ENTRY_POINT_GROUP
    assert "PluginSpec(" in out


def test_plugin_demo_does_not_leak_numpy_repr():
    """打印给用户看的数字不能是 ``np.float64(...)``。"""
    out = _run_demo().stdout
    assert "np.float64" not in out


# --------------------------------------------------------------------------- #
# 14) 模块自洽性
# --------------------------------------------------------------------------- #


def test_all_exports_are_importable():
    for name in P.__all__:
        assert hasattr(P, name), f"__all__ 里的 {name} 不存在"


def test_version_is_0_1_40():
    """版本口径哨兵：升版时这里必须跟上。

    GUI 窗口标题自 v1.3.0 起**从包版本派生**（见 main.py），
    所以真正需要手工同步的只剩三处：``core/__init__.py`` / ``pyproject.toml`` /
    ``app.spec`` 的 ``CFBundleShortVersionString``。

    v0.1.40 = MVP 发版口径（`app/features.py` 隐藏未成熟功能入口）。
    """
    from support import __version__

    assert __version__ == "0.1.40"


def test_plugin_load_error_is_plain_dataclass():
    err = PluginLoadError(name="n", source="s", message="m")
    assert err.detail == ""
    assert dataclasses.asdict(err) == {"name": "n", "source": "s", "message": "m", "detail": ""}
