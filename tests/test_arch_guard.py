"""架构守护脚本自身的回归测试（2026-09-26 补）。

**为什么需要**：`tools/check_arch_rules.py` 的 H3（Domain 禁依赖 Service）与
H4（Adapter 禁依赖 Service）**曾经完全失效**——`SERVICE_FILES` 里写的是文件名
（``planner.py``），而比较用的 ``tail`` 是从 import 语句解析出的裸模块名
（``planner``），于是 ``tail in SERVICE_FILES`` **恒为假**。7 条真实的跨层依赖
因此被静默放过，守护脚本一直报"0 硬违规"，CI（`.github/workflows/ci.yml`）
也一直是绿的。

它能潜伏这么久，根因是**守护脚本自己没有测试**——唯一使用者是 CI，而 CI 只会
看到"绿"。所以这里钉三件事：

1. 口径转换必须真的发生（文件名表 → 裸模块名）；
2. 相对 import 必须解析成 ``core.xxx``（解析错 = 检查又变瞎）；
3. 端到端棘轮：真跑一遍守卫，要求 0 硬违规，且**清单里的每条存量违规都出现在
   WARN 里**（清掉了却忘删清单条目，也会被第 3 条兜住）。
"""

from __future__ import annotations

import importlib.util
import subprocess
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))


def _load_guard():
    """按路径加载守护脚本（它是工具脚本，不在包内）。"""
    spec = importlib.util.spec_from_file_location(
        "_arch_guard_under_test", _ROOT / "tools" / "check_arch_rules.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


guard = _load_guard()


def _run_guard() -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(_ROOT / "tools" / "check_arch_rules.py")],
        cwd=_ROOT,
        capture_output=True,
        text=True,
    )


def test_layers_come_from_directories_not_hand_maintained_tables():
    """回归：分层由**目录**决定，不再有手工文件名表。

    老实现用 ``DOMAIN_FILES`` / ``SERVICE_FILES`` 两张表描述分层，而 H3/H4 又拿
    从 import 解析出的**裸模块名**去比**带 .py 的文件名表** → 恒为假、两条规则
    整体失效（17 条真实跨层依赖被静默放过）。目录即分层后，这类"表与实现不一致"
    在结构上不可能再发生，所以这里连表本身的存在性一起钉死。
    """
    assert not hasattr(guard, "SERVICE_FILES"), "手工文件名表应已删除"
    assert not hasattr(guard, "SERVICE_MODULES"), "手工文件名表应已删除"
    assert not hasattr(guard, "DOMAIN_FILES"), "手工文件名表应已删除"

    assert guard.classify("service/planner.py") == "service"
    assert guard.classify("domain/world/scene.py") == "domain"
    assert guard.classify("adapter/post/base.py") == "adapter"
    assert guard.classify("support/i18n.py") == "support"  # 跨层共享支撑层
    assert guard.classify("accel/__init__.py") == "accel"  # 纯硬件加速层
    assert guard.classify("adapter/robot_library/mesh.py") == "adapter"
    assert guard.classify("app/views/property_panel.py") == "app"
    assert guard.classify("main.py") == "app"
    # GUI 债务优先于目录层：两个存量面板随整包搬进 domain/adapter，仍判 gui
    assert guard.classify("domain/world/panel.py") == "gui"
    assert guard.classify("adapter/robot_library/panel.py") == "gui"


def test_accel_is_acceleration_only_and_support_is_bottom():
    """H7 / H8 反向验证：**塞违规进去必须判硬违规**，删掉必须恢复。

    这两条是 2026-09-26「加速层只放硬件加速代码」的落地口径（目录名 2026-09-26
    由 `core/` 更名为 `accel/`）。写死"当前 0 违规"没有意义（它今天恰好是空的）
    ——真正要钉住的是**规则本身有没有执行力**：上次 H3/H4 失效的教训就是
    "规则在、但从没人验证它会不会响"。
    """
    junk = _ROOT / "accel" / "_probe_junk.py"
    leak = _ROOT / "support" / "_probe_leak.py"
    try:
        junk.write_text("VALUE = 1\n", encoding="utf-8")
        out = _run_guard()
        assert out.returncode != 0 and "H7" in out.stdout, (
            "往 accel/ 塞非加速代码却未被 H7 拦下：\n" + out.stdout
        )

        leak.write_text("from domain.pose import Pose\n", encoding="utf-8")
        out = _run_guard()
        assert out.returncode != 0 and "H8" in out.stdout, (
            "support/ 依赖了层却未被 H8 拦下：\n" + out.stdout
        )
    finally:
        for p in (junk, leak):
            p.unlink(missing_ok=True)

    out = _run_guard()
    assert out.returncode == 0, "清理探测文件后仍未恢复：\n" + out.stdout


def test_relative_import_resolves_to_absolute_module():
    """回归：相对 import 必须解析成绝对模块名，**子包内尤其容易算错一段**。

    ``domain/world/scene.py`` 里的 ``from .geometry import …`` 曾（旧解析器）
    被算成 ``core.world.scene.geometry`` —— 多算了一段、匹配不到任何文件，
    于是 H3/H4 在 ``world/`` · ``post/`` 里整体失明。这里把正确口径钉住。
    """
    path = _ROOT / "domain" / "world" / "scene.py"
    _ext, internal, err = guard.parse_imports(path)
    assert err is None
    assert "domain.world.geometry" in internal, (
        f"`from .geometry import …` 未解析为 domain.world.geometry，" f"实际: {sorted(internal)}"
    )
    # 上跳两级：from ..pose import → domain.pose
    assert "domain.pose" in internal, f"`from ..pose import …` 解析错误: {sorted(internal)}"


def test_guard_reports_no_hard_violations():
    """端到端：当前仓库必须 0 硬违规（CI 与本测试同一口径）。"""
    result = _run_guard()
    assert result.returncode == 0, "守护脚本判定硬违规：\n" + result.stdout + result.stderr
    assert "0 硬违规" in result.stdout


def test_h9_catches_a_key_that_is_missing_from_the_catalog():
    """H9 不是"死检查"：凭空造一个没收录的 key，它必须判红。

    H3/H4 曾长期是**规则在、永远绿**的死检查（比较恒为假），17 条真实违规被
    静默放过。H9 又是"默认全绿"型规则（存量 0），最容易退化成同一种死法——
    所以这里做**负向验证**：往一个真实模块里临时插一个假 key，守护必须报错。
    """
    victim = _ROOT / "app" / "window" / "misc_actions.py"
    original = victim.read_text(encoding="utf-8")
    probe = '\n\ndef _h9_negative_probe():\n    return tr("H9 负向验证用假词条XYZ")\n'
    try:
        victim.write_text(original + probe, encoding="utf-8")
        result = _run_guard()
    finally:
        victim.write_text(original, encoding="utf-8")

    assert result.returncode != 0, "H9 失效：插入未收录的 key 后仍判通过（死检查）"
    assert "H9" in result.stdout and "H9 负向验证用假词条XYZ" in result.stdout


def test_h9_scans_tests_directory_too():
    """H9 必须扫 tests/——漏翻恰恰是"zh 下蒙混过关、切到 en 才红"。

    实测教训：``tests/test_iomap.py`` 曾用一个少了 ``  ✓ `` 前缀的 key，zh 是恒等
    函数所以子串匹配照样通过，只有英文模式会失败。若 H9 沿用分层扫描（不含
    tests/），这类只在英文下暴露的 bug 就永远扫不到。
    """
    guard_mod = _load_guard()
    rels = {guard_mod.rel_id(p) for p in guard_mod.iter_all_py_files()}
    assert any(r.startswith("tests/") for r in rels), "H9 的全仓遍历应包含 tests/"


def test_every_registered_layer_violation_is_reported():
    """棘轮：清单里的每条存量违规都要真的被报出来。

    若某条已经不存在（被修好了却忘删清单），守护脚本会打印"已不再违规"的警告，
    这里就会失败——提醒把条目删掉，让棘轮只减不增。
    """
    result = _run_guard()
    missing = [
        entry
        for entry in sorted(guard.LEGACY_LAYER_VIOLATIONS)
        if f"[{entry[0]}]" not in result.stdout or f"存量跨层依赖 {entry[1]}" not in result.stdout
    ]
    assert not missing, f"清单条目未出现在守卫输出中（可能已消解，请删条目）：{missing}"


def test_flag_layer_violation_separates_legacy_from_new():
    """单元：清单内 → 警告（棘轮），清单外 → 硬违规。"""
    errors: list = []
    warns: list = []
    hit: set = set()
    entry = sorted(guard.LEGACY_LAYER_VIOLATIONS)[0]

    guard.flag_layer_violation("H3", entry[0], entry[1], errors, warns, hit)
    assert not errors and warns and entry in hit

    errors2: list = []
    warns2: list = []
    guard.flag_layer_violation("H3", "brand_new.py", "service.planner", errors2, warns2, set())
    assert errors2 and not warns2
