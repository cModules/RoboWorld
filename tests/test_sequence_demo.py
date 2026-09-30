"""v1.3.2 动态时序使用体验：示例脚本 ``examples/sequence_demo.py`` 的验收测试。

以子进程方式真实运行示例脚本（示例的价值就在"一条命令跑通"），
断言退出码、七个分段、双出口的真实内容（UR IO 行 / SCL 握手信号）、
诚实性口径（CallStep 跳过 notes、驻留是唯一可计算时间、边沿/超时 TODO）
与 JSON 往返产物。
"""

import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "examples" / "sequence_demo.py"
JSON_OUT = REPO_ROOT / "examples" / "assets" / "pick_sequence.json"


def _run_demo() -> subprocess.CompletedProcess:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([str(REPO_ROOT), str(REPO_ROOT), env.get("PYTHONPATH", "")])
    return subprocess.run(
        [sys.executable, str(SCRIPT)],
        cwd=str(REPO_ROOT),
        env=env,
        capture_output=True,
        text=True,
        timeout=300,
    )


def test_demo_script_runs_clean():
    proc = _run_demo()
    assert proc.returncode == 0, (
        f"示例脚本退出码 {proc.returncode}\n--- stdout ---\n"
        f"{proc.stdout[-2000:]}\n--- stderr ---\n{proc.stderr[-2000:]}"
    )


def test_demo_script_has_seven_sections():
    out = _run_demo().stdout
    for section in (
        "1. 工作单元点表",
        "2. 工序编排",
        "3. 校验",
        "4. 组成摘要",
        "5. 机器人侧出口",
        "6. PLC 侧出口",
        "7. JSON 往返",
    ):
        assert section in out, f"缺少分段标题：{section}"


def test_demo_script_validation_is_real():
    """校验输出非空（无限等待警告 + 边沿提示），且自证未知信号报 error。"""
    out = _run_demo().stdout
    assert "no_timeout" in out, "无限等待警告必须出现"
    assert "edge_ignores_value" in out, "边沿触发提示必须出现"
    assert "校验真实生效" in out, "未知信号自证必须通过"


def test_demo_script_robot_exit_resolves_real_io():
    """机器人侧出口：UR 出码解析出真实数字量序号，且边沿/超时留 TODO。"""
    out = _run_demo().stdout
    assert "set_digital_out(0, True)" in out, "gripper_close 应解析为 Q0.0 → 序号 0"
    assert "set_digital_out(2, True)" in out, "robot_done 应解析为 Q0.2 → 序号 2"
    assert "wait_digital_in(1, True)" in out, "part_present 应解析为 I0.1 → 序号 1"
    # 诚实性：边沿语义降级必须留 TODO，不许静默当电平
    assert "原语义为 rising 边沿触发" in out


def test_demo_script_call_step_is_disclosed_not_invented():
    """CallStep 不是程序指令——notes 必须如实披露，而不是臆造 CALL 语法。"""
    out = _run_demo().stdout
    assert "调用抓取子程序" in out
    assert "不是程序指令" in out
    assert "[notes]" in out


def test_demo_script_scl_contains_point_table_and_handshake():
    out = _run_demo().stdout
    assert "SeqDemoCell" in out, "SCL FB 名应来自示例参数"
    assert "IO 点表" in out, "SCL 头部应带自动生成的点表注释"
    for sig in ("robot_start", "part_present", "gripper_close", "robot_done"):
        assert sig in out, f"SCL 点表里应出现握手信号 {sig}"


def test_demo_script_honesty_note_and_dwell_total():
    out = _run_demo().stdout
    assert "驻留合计是唯一可计算的时间" in out
    assert "取决于现场机构" in out
    assert "0.7s" in out, "本示例两条驻留 0.5 + 0.2 = 0.7s"


def test_demo_script_json_roundtrip_artifact():
    proc = _run_demo()
    assert proc.returncode == 0
    assert JSON_OUT.exists(), "示例应把序列存成 examples/assets/pick_sequence.json"
    assert "读回：9 条工步" in proc.stdout
    assert "与写入前一致" in proc.stdout
    text = JSON_OUT.read_text(encoding="utf-8")
    assert '"schema": "service.sequence/1"' in text
    assert '"kind": "call"' in text and '"kind": "dwell"' in text
