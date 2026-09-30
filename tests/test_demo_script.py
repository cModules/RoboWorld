"""S5 展示物料：无头示例脚本 ``examples/grasping_workstation_demo.py`` 的验收测试。

以子进程方式真实运行示例脚本（不是 import 后调用函数——示例的价值就在
"一条命令跑通"），断言退出码、全流程分段输出与各品牌出码的抓取 IO 语法。
"""

import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "examples" / "grasping_workstation_demo.py"


def _run_demo() -> subprocess.CompletedProcess:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([str(REPO_ROOT), str(REPO_ROOT), env.get("PYTHONPATH", "")])
    return subprocess.run(
        [sys.executable, str(SCRIPT)],
        cwd=str(REPO_ROOT),
        env=env,
        capture_output=True,
        text=True,
        timeout=600,
    )


def test_demo_script_runs_clean():
    proc = _run_demo()
    assert proc.returncode == 0, (
        f"示例脚本退出码 {proc.returncode}\n--- stdout ---\n"
        f"{proc.stdout[-2000:]}\n--- stderr ---\n{proc.stderr[-2000:]}"
    )


def test_demo_script_full_pipeline_sections():
    out = _run_demo().stdout
    # 全流程 5 个阶段都有分段输出
    for marker in (
        "[1] 工作站模板",
        "[2] 示例示教点",
        "[3] 程序校验",
        "[4] 仿真完成",
        "URScript",
        "KUKA KRL",
        "ABB RAPID",
        "FANUC TP",
    ):
        assert marker in out, f"缺少分段输出: {marker}"
    # 校验通过、节拍非零
    assert "程序校验: 通过" in out
    assert "节拍 0.00 s" not in out


def test_demo_script_discloses_envelope_contacts():
    """干涉按包络容差判定时必须**把放行量说出来**，不能悄悄放过。

    本仓库用 URDF ``<collision>``（厂商保守包络盒）做干涉检查，浅穿透是
    模型误差；演示放行 ≤``ENVELOPE_TOLERANCE_MM`` 的接触，但必须打印
    容差值与放行条数，否则用户会以为"零错误 == 绝对不碰"。
    """
    from service.planner import ENVELOPE_TOLERANCE_MM

    out = _run_demo().stdout
    assert "包络容差" in out
    assert f"{ENVELOPE_TOLERANCE_MM:g}" in out


def test_demo_script_workstation_contents():
    out = _run_demo().stdout
    for name in ("吸盘工具", "输送带", "工件_1", "托盘"):
        assert name in out, f"场景对象缺失: {name}"
    # 五点式抓取循环语义齐全
    for t in ("VIA", "APPROACH", "PICK", "LIFT", "PLACE", "RETRACT"):
        assert t in out, f"工艺语义缺失: {t}"


def test_demo_script_brand_io_syntax():
    out = _run_demo().stdout
    # 抓取时序在各品牌出码中都有真实 IO（不是 TODO 注释代替）
    assert "set_digital_out(1, True)" in out  # UR 夹爪闭合
    assert "WAIT FOR $IN[3]" in out  # KUKA 等待夹紧反馈
    assert "SetDO" in out or "\\SetDO" in out  # ABB 置位
    assert "DO[" in out  # FANUC 直接输出
    # 出码里不允许出现"指令降级为注释"的占位（S2 的验收标准延续到示例：
    # post/base.py 信号未映射时输出 TODO_<信号名> 占位）
    for brand_block in out.split("--- ")[1:]:
        if brand_block.startswith(("URScript", "KUKA", "ABB", "FANUC")):
            assert "TODO_gripper" not in brand_block
