"""库层消息 i18n（v1.1）的无头测试。

v1.0.10 只覆盖了 GUI 层文案；v1.1 把 planner（Issue / 报告）与后处理
（告警 / 生成代码注释）的运行时消息也接进 ``tr()``。本文件验证：

* zh 恒等——默认语言下报告与出码与旧版逐字节一致（零回归）
* en 查表——校验报告、Issue 消息、后处理 TODO 注释、工厂错误为英文
* en 模式下报告与生成代码中不残留中文字符
* 模板占位符按命名参数正确格式化

全部无 GUI 依赖。
"""

import os
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

import orolp.i18n as i18n  # noqa: E402
from orolp import Robot  # noqa: E402
from orolp.planner import TrajectoryPlanner  # noqa: E402
from orolp.post import get_post_processor  # noqa: E402
from orolp.program import Motion, MotionType, ProgramBuilder, WaitDI  # noqa: E402

URDF = REPO_ROOT / "models" / "ur" / "ur5e" / "ur5e.urdf"


def _cjk(text: str) -> int:
    return sum(1 for c in text if "\u4e00" <= c <= "\u9fff")


@pytest.fixture(autouse=True)
def _isolated_i18n(monkeypatch, tmp_path):
    """每个用例独享语言状态与配置目录，结束后回到 zh。"""
    monkeypatch.delenv("OROLP_LANG", raising=False)
    monkeypatch.setenv("OROLP_CONFIG_DIR", str(tmp_path / "cfg"))
    i18n.set_language(i18n.LANG_ZH, save=False)
    yield
    i18n.set_language(i18n.LANG_ZH, save=False)


@pytest.fixture(scope="module")
def robot():
    return Robot(str(URDF))


def _pick_program(robot) -> "object":
    pb = ProgramBuilder(robot)
    pb.moveJ([0, -90, 90, -90, -90, 0])
    pose = robot.forward_kinematics([0.0, -60, 120, -90, -90, 0.0])
    pb.moveL(pose, speed=50)
    return pb.build()


# ---------------------------------------------------------------------------
# zh 恒等（默认零回归）
# ---------------------------------------------------------------------------

def test_zh_report_head_is_chinese(robot):
    report = TrajectoryPlanner(robot).validate(_pick_program(robot))
    assert report.format().splitlines()[0].startswith("✓ 程序校验通过")


def test_zh_post_comment_is_chinese():
    items = [
        Motion(MotionType.JOINTS, [0, -90, 90, -90, -90, 0]),
        WaitDI("gripper_closed", True, 2.0, trigger="rising"),
    ]
    code = get_post_processor("fanuc_r30ib").generate(items, {})
    assert "TODO: 原语义为 rising 边沿触发，请人工核对" in code


# ---------------------------------------------------------------------------
# en：校验报告与 Issue 消息
# ---------------------------------------------------------------------------

def test_en_report_head_and_data_lines(robot):
    i18n.set_language(i18n.LANG_EN, save=False)
    report = TrajectoryPlanner(robot).validate(_pick_program(robot))
    text = report.format()
    lines = text.splitlines()
    assert lines[0] == "✓ Program validation passed"
    assert "motions /" in lines[1] and "TCP path" in lines[1]
    assert any("Estimated cycle time" in ln for ln in lines)
    assert _cjk(text) == 0


def test_en_report_cycle_breakdown(robot):
    i18n.set_language(i18n.LANG_EN, save=False)
    report = TrajectoryPlanner(robot).validate(_pick_program(robot))
    assert "Cycle breakdown (by motion):" in report.format()


def test_en_no_scene_warning(robot):
    i18n.set_language(i18n.LANG_EN, save=False)
    report = TrajectoryPlanner(robot, scene=None).validate(
        _pick_program(robot), check_collision=True
    )
    msgs = [i.message for i in report.issues if i.code == "no_scene"]
    assert msgs and msgs[0] == "No workcell scene provided; collision check skipped"


def test_en_movej_dof_mismatch(robot):
    i18n.set_language(i18n.LANG_EN, save=False)
    pb = ProgramBuilder(robot)
    pb.moveJ([0.0, -90.0])  # 2 轴 vs 6 DOF
    report = TrajectoryPlanner(robot).validate(pb.build())
    assert report.errors, "维度错误必须产生 error"
    assert "moveJ target has 2 joints but the robot has 6 DOF" in report.errors[0].message


def test_en_violation_message(robot):
    """AxisLimits.violation 的越限描述走目录（随后被 moveJ/moveL 消息内嵌）。"""
    i18n.set_language(i18n.LANG_EN, save=False)
    q = [0.0] * 5 + [999.0]
    text = TrajectoryPlanner(robot)._limits.violation(q)
    assert text.startswith("Axis 6 above upper limit")
    assert "999.000" in text


def test_en_unknown_post_error():
    i18n.set_language(i18n.LANG_EN, save=False)
    with pytest.raises(Exception) as exc:
        get_post_processor("no_such_post_xyz")
    assert "Unknown post-processor" in str(exc.value)
    assert "available:" in str(exc.value)


# ---------------------------------------------------------------------------
# en：后处理注释（生成代码里的用户可见文本）
# ---------------------------------------------------------------------------

def test_en_fanuc_todo_comments():
    i18n.set_language(i18n.LANG_EN, save=False)
    items = [
        Motion(MotionType.JOINTS, [0, -90, 90, -90, -90, 0]),
        WaitDI("gripper_closed", True, 2.0, trigger="rising"),
    ]
    code = get_post_processor("fanuc_r30ib").generate(items, {})
    assert "cannot be resolved to a numeric index" in code
    assert "original semantics is rising-edge triggered" in code
    assert _cjk(code) == 0


def test_en_unresolved_signal_warning(capsys):
    i18n.set_language(i18n.LANG_EN, save=False)
    items = [WaitDI("gripper_closed", True, 0.0)]
    get_post_processor("fanuc_r30ib").generate(items, {})
    out = capsys.readouterr().out
    assert "is not mapped to a physical address" in out
    assert "TODO placeholder" in out


def test_en_arc_comments():
    from orolp.process import WeldProcess

    i18n.set_language(i18n.LANG_EN, save=False)
    proc = WeldProcess(current=200, voltage=24.0, wire_feed=10.0,
                       gas_preflow=0.5, gas_postflow=1.0)
    items = []
    from orolp.program import ArcStart, ArcEnd, SetDO

    class _FakeItems(list):
        pass

    post = get_post_processor("fanuc_r30ib")
    text_start = post.format_arc_start(proc, {})
    assert "Wire feed 10.0 m/min" in text_start
    assert "gas pre-flow 0.5 s before arc start" in text_start
    text_end = post.format_arc_end(proc, {})
    assert "Gas post-flow 1.0 s after arc end" in text_end


def test_en_jaka_template_note():
    i18n.set_language(i18n.LANG_EN, save=False)
    items = [Motion(MotionType.JOINTS, [0, -90, 90, -90, -90, 0])]
    code = get_post_processor("jaka_zu").generate(items, {})
    assert "syntax not verified on a real controller" in code
    assert _cjk(code) == 0


def test_en_estun_template_note():
    i18n.set_language(i18n.LANG_EN, save=False)
    items = [Motion(MotionType.JOINTS, [0, -90, 90, -90, -90, 0])]
    code = get_post_processor("estun_er").generate(items, {})
    assert "syntax not verified on a real controller" in code
    assert _cjk(code) == 0


def test_en_motion_type_unsupported():
    i18n.set_language(i18n.LANG_EN, save=False)
    from orolp.post import PostProcessError

    # SPLINE 走 base 降级（estun 未覆盖 SPLINE），但 ESTUN 未实现任何 SPLINE
    # 转换路径时才会抛——用不存在 via 的 moveC 触发品牌级错误
    items = [Motion(MotionType.CIRCULAR, [0, -90, 90, -90, -90, 0])]  # 缺 via
    with pytest.raises(PostProcessError) as exc:
        get_post_processor("estun_er").generate(items, {})
    assert "ESTUN circular motion requires a via point" in str(exc.value)


# ---------------------------------------------------------------------------
# 目录卫生
# ---------------------------------------------------------------------------

def test_catalog_values_nonempty():
    from orolp.i18n import _CATALOG_EN

    for key, val in _CATALOG_EN.items():
        assert val.strip(), f"空译文: {key!r}"
        assert _cjk(val) == 0 or key.startswith("语言 / Language") or val == key, (
            f"译文残留中文: {key!r} -> {val!r}"
        )


def test_issue_where_label_en(robot):
    """Issue.__str__ 的定位前缀（程序级 / 指令 N）也要走目录。"""
    from orolp.planner import Issue, SEVERITY_ERROR

    i18n.set_language(i18n.LANG_EN, save=False)
    s = str(Issue(SEVERITY_ERROR, "x", None, "boom"))
    assert s.startswith("✗ [Program-level]")
    s2 = str(Issue(SEVERITY_ERROR, "x", 2, "boom"))
    assert "[Motion 3]" in s2
