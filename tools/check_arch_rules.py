#!/usr/bin/env python3
"""RoboWorld 架构守护脚本（对应 docs/develop/CODING_RULES.md v2 / AGENTS.md）

检查项：
  H1  PySide6/OpenGL 只允许出现在 GUI 归宿（app/ + 已登记债务文件）
  H2  C++ 核心（accel_native / accel._native）只允许在单一边界白名单内引用
  H3  Domain 层禁止 import service / app / adapter
  H4  Adapter 层禁止 import service / GUI
  H5  i18n 棘轮：中文串字面量（排除 docstring/注释）超过基线即为硬违规，只许减不许增
  H6  features 一致性：enabled/_feature_enabled 的功能键必须在 ALL_FEATURES 注册
  H7  accel/ 纯净：只允许**硬件加速代码**（C++ 源码 src/ 与 C++ 边界 __init__.py）
  H8  support/ 不得依赖任何层（它是被各层依赖的最底层）
  H9  i18n 词条覆盖率：tr("字面量") 的 key 必须在 _CATALOG_EN 里有英文译文
      （与 H5 互补：H5 管"中文串有没有走 tr()"，H9 管"走了 tr() 有没有译文"。
      漏 H9 的后果是中英混杂在**英文界面**上，而 zh 是恒等函数、永远看不出来）
  L1  行数分档：>600 行须有 TECH_DEBT 标注或在存量债务清单；>1000 行为硬违规

**分层归属 = 目录**（2026-09-26 起，不再有手工文件名表）：

  main.py + app/   → App / GUI（PySide6 唯一归宿；features.py 在此）
  service/         → 业务用例（路径规划 / 工艺 / 抓放 / 标定 / 双机 / 工程）
  domain/          → 领域模型（零 GUI / 零 IO）
  adapter/         → 可替换边界（后处理 / 机器人库 / 真机链路 / 云通道 / 插件注册表；
                     模型资产 mesh / primitives / urdf_model 也在此）
  support/         → 跨层共享支撑（i18n；无业务语义、不依赖任何层）
  accel/           → **纯硬件加速层**：src/（C++ 实现）+ __init__.py（唯一 C++ 边界；
                     兼容门面已于 2026-09-26 下线，见 ADR-008）

依赖方向：``app → service → domain ← adapter``，``support`` 与 ``accel`` 在最底层
被各层依赖（禁止反向、禁止跨层跳跃）。

> 为什么改成目录判定：此前用两张手工文件名表（``DOMAIN_FILES`` / ``SERVICE_FILES``）
> 描述分层，表与目录长期两处维护、必然漂移；更糟的是 H3/H4 拿**裸模块名**去比
> **带 .py 的文件名表**，比较恒为假——两条规则曾经**完全失效**，17 条真实跨层依赖
> 被静默放过（2026-09-26 修复，回归测试见 tests/test_arch_guard.py）。
> 目录即分层后，这类"表与实现不一致"的失效模式在结构上被消除了。

用法：
  python tools/check_arch_rules.py            # 报告模式，硬违规时退出码 1
  python tools/check_arch_rules.py --lines    # 额外打印行数明细
退出码：0=通过；1=存在硬违规
"""

from __future__ import annotations

import argparse
import ast
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# H9 要 import 仓库里的 support.i18n 才能读到词条目录，而脚本被直接运行时
# sys.path[0] 是 tools/ —— 不补这一行，H9 会**静默跳过**（首次启用就踩到了）。
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

#: 启动壳。注意它从 app.py 改名成 main.py，因为同目录下 ``app/`` 包会**遮蔽**
#: 同名模块——``import app`` 会从"启动壳"悄悄变成"GUI 包"，全项目静默换对象。
APP_SHELL = "main.py"

#: 参与检查的顶层目录（= 层）
LAYER_DIRS = ("app", "accel", "domain", "service", "adapter", "support")

# --------------------------------------------------------------------------- #
# 分层判定：目录前缀 → 层。顺序即优先级（先匹配先赢）
# --------------------------------------------------------------------------- #
LAYER_BY_PREFIX = (
    ("app/", "app"),
    ("support/", "support"),  # 跨层共享支撑：被各层依赖，自身不依赖任何层
    ("domain/", "domain"),
    ("service/", "service"),
    ("adapter/", "adapter"),
    ("accel/", "accel"),  # 纯硬件加速层（src/ C++ + 唯一边界）
)

#: PySide6/OpenGL 合法归宿：app/ 整包 + 存量债务文件（ADR-002 拆解后清零）。
#: ``domain/world/panel.py`` 与 ``adapter/robot_library/panel.py`` 是**随整包搬迁
#: 一起过来的**存量 GUI 面板——把包劈成两半会留下两个同名包（core.world 与
#: domain.world），import 语义会烂掉，所以让它们跟着包走，靠这张表冻结，
#: 等 ADR-002 再迁进 app/views。
GUI_DEBT = {
    "adapter/robot_library/panel.py": "迁移 GUI 至 app/views（随整包搬迁的存量面板）",
    "domain/world/panel.py": "迁移 GUI 至 app/views（随整包搬迁的存量面板）",
}

#: C++ 单一边界（AGENTS.md #4；现状边界在 accel/__init__.py，见 CODING_RULES §2 注①）
ACCEL_BOUNDARY = {
    "accel/__init__.py": "C++ 单一边界（唯一 import accel_native 的地方）",
    "domain/collision.py": "C++ 单一边界（碰撞 broadphase，透明降级）",
    "domain/kinematics.py": "C++ 单一边界（FK 热路径，透明降级）",
}
#: 原生符号：出现即需在 ACCEL_BOUNDARY 内。改名后是 accel_native / accel._native。
ACCEL_TOKENS = {"accel_native", "accel._native"}

#: 存量超限债务清单（>600 行，Phase 1 按"一次一个类"消化；新增超限文件不在此列，
#: 必须头部标注 TECH_DEBT 否则判硬违规）
LEGACY_DEBT = {
    "adapter/cloudhttp.py": "Adapter 归位评审（CODING_RULES §1）",
    "adapter/linkstub.py": "Adapter 拆分（Phase 1）",
    "adapter/plugins.py": "插件 API v2（Phase 3）",
    "adapter/post/base.py": "Adapter 拆分（Phase 1）",
    "adapter/robot_library/hifi.py": "Adapter 拆分（Phase 1）",
    "adapter/robot_library/panel.py": "迁移 GUI 至 app/views",
    "adapter/robot_library/specs.py": "Adapter 拆分（Phase 1）",
    "adapter/robot_library/urdf_builder.py": "Adapter 拆分（首轮扫描 1217 行）",
    # 2026-09-27 校正：下面三条**已经在 app/views**，"迁移 GUI 至 app/views" 这条
    # 理由已经不成立（债务描述过期比没有更糟——后人会以为还没迁）。真实债务是
    # **类内继续拆**，拆分方案见 docs/develop/BUILD_QUEUE.md「B 组」。
    "app/views/controller_panel.py": "已在 app/views：真机连接面板 683 行，类内继续拆",
    "app/views/gl_widget.py": "已在 app/views：类内继续拆（1992 行，方案见 BUILD_QUEUE B2）",
    "app/views/path_editor.py": "已在 app/views：类内继续拆（1298 行，方案见 BUILD_QUEUE B3）",
    "app/views/program_panel.py": "已在 app/views：类内继续拆（2193 行，全仓最大单体，方案见 BUILD_QUEUE B1）",
    # 2026-09-27 移除 ``app/window/main_window.py``：C2 把导入桥收敛进
    # capabilities.py 后它降到 596 行（< 软限 600）——债务还完就下调，别留在清单里
    # 误导后人。下面这条仍在软限之上（601 行），保留。
    "app/window/project_session.py": "ADR-002 第 8 步：工程/会话组（601 行；模板与自动保存两块可再分）",
    "support/i18n.py": "词条数据外置（Phase 1）",
    "adapter/robot_library/primitives.py": "View 支撑拆分（Phase 1）",
    "adapter/robot_library/urdf_model.py": "View 支撑拆分（Phase 1）",
    "domain/collision.py": "Domain 拆分（Phase 2）",
    "domain/dynamics.py": "Domain 拆分（Phase 2）",
    "domain/kinematics.py": "双实现合并（Phase 2）",
    "domain/link.py": "Domain 拆分（Phase 2）",
    "domain/world/components.py": "Domain 拆分（Phase 2）",
    "domain/world/geometry.py": "Domain 拆分（Phase 2）",
    "domain/world/panel.py": "迁移 GUI 至 app/views",
    "domain/world/scene.py": "Domain 拆分（Phase 2）",
    "domain/world/templates.py": "Domain 拆分（Phase 2）",
    # 2026-09-27 移除：main.py 已瘦到 160 行（< 600 软限）——ADR-002 启动壳拆解
    # 完成，MainWindow 落地 app/window/main_window.py。留在清单里只会误导后人
    # 以为还有一件没做的事（棘轮的本意就是"还完债就下调"）。
    "service/calibration.py": "Service 拆分（Phase 1）",
    "service/cam.py": "Service 拆分（Phase 1）",
    "service/compliance.py": "Service 拆分（Phase 1）",
    "service/incremental.py": "Service 拆分（Phase 1）",
    "service/iomap.py": "Service 拆分（Phase 1）",
    "service/perf.py": "Service 拆分（Phase 1）",
    "service/planner.py": "Service 拆分（Phase 1）",
    "service/progdiff.py": "Service 拆分（Phase 1）",
    "service/program.py": "Service 拆分（Phase 1）",
    "service/project.py": "Service 拆分（Phase 1）",
    "service/twinsync.py": "Service 拆分（Phase 1）",
}

SOFT_LIMIT = 600
HARD_LIMIT = 1000

# --------------------------------------------------------------------------- #
# H5: i18n 棘轮基线（键＝**仓库相对路径**）
# 规则：文件内"非 docstring 的中文字符串字面量"数量 ≤ 基线；超过即硬违规；
# 减少后把基线同步下调（只许减不许增，逐步收敛到 i18n 键）。
# 搬迁只改路径、不改条数：2026-09-26 三层拆分时键随文件整体重映射，数值守恒。
# **代码搬迁同样守恒**：搬走的方法条数在源文件下调、在目标文件立键，总数不变——
# 例（拆 MainWindow 第 3 步）：main.py 217 → 189，app/window 新立 22 + 6 = 28。
# --------------------------------------------------------------------------- #
I18N_HOME = {"support/i18n.py"}
CJK_RE = re.compile(r"[\u4e00-\u9fff]")
CJK_BASELINE = {
    "adapter/cloudhttp.py": 49,
    "adapter/linkstub.py": 43,
    "adapter/ocpcompat.py": 4,
    "adapter/plugins.py": 70,
    "adapter/post/__init__.py": 3,
    "adapter/post/abb_rapid.py": 4,
    "adapter/post/base.py": 16,
    "adapter/post/estun_er.py": 3,
    "adapter/post/fanuc_tp.py": 10,
    "adapter/post/huichuan_inoteach.py": 2,
    "adapter/post/jaka_zu.py": 3,
    "adapter/post/kuka_krc4.py": 7,
    "adapter/post/ur_script.py": 7,
    "adapter/robot_library/catalog.py": 13,
    "adapter/robot_library/panel.py": 29,
    "adapter/robot_library/specs.py": 38,
    "adapter/robot_library/urdf_builder.py": 2,
    "adapter/sshstub.py": 4,
    "app/cli.py": 36,
    "app/session.py": 8,
    "app/views/controller_panel.py": 76,
    "app/views/gl_widget.py": 6,
    "app/views/path_editor.py": 84,
    "app/views/program_panel.py": 138,
    "app/views/property_controls.py": 5,
    "app/views/property_panel.py": 7,
    "app/views/property_panel_ui.py": 31,
    "app/views/robot_renderer.py": 2,
    "app/window/controller_link.py": 6,
    "app/window/main_window.py": 51,
    "app/window/misc_actions.py": 9,
    "app/window/multi_robot_sync.py": 4,
    "app/window/project_session.py": 50,
    "app/window/playback.py": 12,
    "app/window/plugin_cache.py": 22,
    "app/window/program_ops.py": 33,
    "app/window/robot_scene.py": 23,
    "app/features.py": 5,
    "adapter/robot_library/mesh.py": 11,
    "adapter/robot_library/primitives.py": 6,
    "adapter/robot_library/urdf_model.py": 22,
    "domain/collision.py": 3,
    "domain/kinematics.py": 11,
    "domain/link.py": 59,
    "domain/robot.py": 7,
    "domain/world/animation.py": 1,
    "domain/world/components.py": 115,
    "domain/world/demo.py": 1,
    "domain/world/geometry.py": 4,
    "domain/world/panel.py": 112,
    "domain/world/parts_library.py": 17,
    "domain/world/placement.py": 7,
    "domain/world/scene.py": 11,
    "domain/world/templates.py": 67,
    "main.py": 7,
    "service/calibration.py": 66,
    "service/cam.py": 25,
    "service/cell.py": 6,
    "service/compliance.py": 72,
    "service/debug.py": 5,
    "service/iomap.py": 27,
    "service/multirobot.py": 21,
    "service/perf.py": 78,
    "service/pickplace.py": 10,
    "service/planner.py": 39,
    "service/plc_scl.py": 27,
    "service/process.py": 24,
    "service/progdiff.py": 29,
    "service/program.py": 13,
    "service/project.py": 74,
    "service/project_templates.py": 13,
    "service/sequence.py": 22,
    "service/simulation.py": 2,
    "service/twinsync.py": 92,
}

#: 8 个品牌后处理器 + ``domain/world/animation.py`` 共用的同一条债（根因只有一个：
#: ``Program`` 是程序表示＝领域模型，却定义在 Service）。
_POST_PROGRAM_DEBT = "后处理器/动画都要读程序表示：Program 是领域模型，" "应随 domain 拆分下移"

#: H3/H4 存量跨层依赖（棘轮：**只许减不许增**）。
#:
#: 每条的修法都是"让概念回到它该在的层"：常量回 Domain、协议/纯工具回 Domain 或
#: support、反向工厂调用需单独一 PR 改公开 API。新增条目判硬违规。
LEGACY_LAYER_VIOLATIONS = {
    (
        "adapter/cloudhttp.py",
        "service.twinsync",
    ): "CloudTransport 是协议抽象、normalize_key 是纯工具，应上移到 Domain 或 support，Adapter 只依赖抽象",
    ("adapter/linkstub.py", "service.twinsync"): "同上：Adapter 经协议引用，不直接 import Service",
    ("adapter/post/abb_rapid.py", "service.program"): _POST_PROGRAM_DEBT,
    ("adapter/post/base.py", "service.program"): _POST_PROGRAM_DEBT,
    ("adapter/post/estun_er.py", "service.program"): _POST_PROGRAM_DEBT,
    ("adapter/post/fanuc_tp.py", "service.program"): _POST_PROGRAM_DEBT,
    ("adapter/post/huichuan_inoteach.py", "service.program"): _POST_PROGRAM_DEBT,
    ("adapter/post/jaka_zu.py", "service.program"): _POST_PROGRAM_DEBT,
    ("adapter/post/kuka_krc4.py", "service.program"): _POST_PROGRAM_DEBT,
    ("adapter/post/ur_script.py", "service.program"): _POST_PROGRAM_DEBT,
    (
        "domain/collision.py",
        "service.planner",
    ): "SINGULARITY_COND 是运动学属性常量，应随 domain 拆分上移",
    (
        "domain/collision.py",
        "service.simulation",
    ): "Trajectory 是领域概念（轨迹），应随 domain 拆分上移",
    (
        "domain/dynamics.py",
        "service.simulation",
    ): "Trajectory 是领域概念（轨迹），应随 domain 拆分上移",
    (
        "domain/envelope.py",
        "service.planner",
    ): "ENVELOPE_TOLERANCE_MM 是领域常量，应随 domain 拆分上移",
    (
        "domain/robot.py",
        "service.program",
    ): "Robot.moveJ/moveL… 11 个方法反向构造 ProgramBuilder，需单独一 PR 把流式 API 移到 Service（动公开 API）",
    ("domain/world/animation.py", "service.program"): _POST_PROGRAM_DEBT,
    (
        "domain/world/demo.py",
        "service.pickplace",
    ): "演示场景调用抓放用例：demo 更像样例编排而非 Domain，应移出 domain/ 或改为经 Service 注入",
}

# H6: 功能门控键调用形态（含面板本地包装 _feature_enabled）
FEATURE_CALL_NAMES = {"enabled", "_feature_enabled", "hidden_in_mvp"}


def iter_py_files():
    yield ROOT / APP_SHELL
    for layer in LAYER_DIRS:
        for p in sorted((ROOT / layer).rglob("*.py")):
            if "__pycache__" in p.parts:
                continue
            yield p


def iter_all_py_files():
    """**全仓** .py（含 tests/ 与 tools/）——H9 专用。

    ``iter_py_files()`` 只扫分层目录，而 i18n key 的漏翻同样会发生在测试里
    （实测过：``tests/test_iomap.py`` 用了一个少了 ``  ✓ `` 前缀的 key，zh 下靠
    子串匹配蒙混过关，切到 en 必然红）。故覆盖率必须按全仓算。
    """
    skip = {"__pycache__", ".venv", "build", ".git"}
    for p in sorted(ROOT.rglob("*.py")):
        parts = set(p.relative_to(ROOT).parts)
        if parts & skip:
            continue
        yield p


def check_i18n_coverage(errors: list, warns: list) -> None:
    """H9：``tr("字面量")`` 的 key 必须在 ``_CATALOG_EN`` 里。

    与 H5 互补：H5 管"中文串有没有走 ``tr()``"（按文件记账的棘轮），H9 管
    "走了 ``tr()`` 的 key 有没有英文译文"（按词条记账）。缺 H5 会中英混杂在
    源码里，缺 H9 会中英混杂在**英文界面**上——后者更隐蔽：zh 是恒等函数，
    key 写错也照样显示正确，只有切到 en 才暴露。

    **零容忍硬规则**（不是棘轮）：2026-09-27 首次启用时实测 902 个字面量 key
    只有 1 条未收录，修掉后存量＝0——存量既然为零，就没有理由开基线。

    非字面量（动态拼接的 key）无法静态分析，只提示、不判红。
    """
    try:
        from support.i18n import _CATALOG_EN
    except Exception as _exc:  # pragma: no cover
        warns.append(f"H9 跳过：无法导入 support.i18n._CATALOG_EN（{_exc}）")
        return

    missing: dict = {}
    dynamic: list = []
    for path in iter_all_py_files():
        rel = rel_id(path)
        if rel in I18N_HOME:
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        for n in ast.walk(tree):
            if not (isinstance(n, ast.Call) and isinstance(n.func, ast.Name)):
                continue
            if n.func.id != "tr" or not n.args:
                continue
            arg = n.args[0]
            if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
                if arg.value not in _CATALOG_EN:
                    missing.setdefault(arg.value, []).append(rel)
            else:
                dynamic.append(rel)

    for key in sorted(missing):
        where = ", ".join(sorted(set(missing[key]))[:3])
        shown = key if len(key) <= 48 else key[:45] + "..."
        errors.append(
            f'H9 i18n 词条缺失（英文版会显示中文）："{shown}" —— {where}；'
            f"请在 support/i18n.py 的 _CATALOG_EN 补英文译文"
        )
    if dynamic:
        warns.append(
            f"H9 提示：{len(dynamic)} 处 tr() 用了非字面量 key，无法静态校验（需人工确认）"
        )


def rel_id(path: Path) -> str:
    """统一标识：一律**相对仓库根的 posix 路径**（``service/planner.py``）。"""
    return path.relative_to(ROOT).as_posix()


def module_name(rel: str) -> str:
    return rel[:-3].replace("/", ".") if rel.endswith(".py") else rel


def parse_imports(path: Path):
    """返回 (外部模块名集合, 内部模块名集合)。"""
    try:
        tree = ast.parse(path.read_text(encoding="utf-8", errors="replace"))
    except SyntaxError as e:
        return None, None, f"{path}: 语法解析失败 {e}"
    ext, internal = set(), set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                (internal if a.name.split(".")[0] in ("core", *LAYER_DIRS) else ext).add(a.name)
        elif isinstance(node, ast.ImportFrom):
            if node.level:  # 相对导入 → 绝对化
                # 包路径 = 本模块名去掉最后 node.level 段（模块名自身占最后一段）。
                # 此前这里多加了 ``+ (1 if len>1 else 0)``，于是**子包内的相对导入
                # 全部解析错**（``world/scene.py`` 的 ``from .geometry import``
                # 被算成 ``core.world.scene.geometry``），错名字匹配不到任何文件，
                # H3/H4 的层级判断直接落空。
                mod = module_name(rel_id(path)).split(".")
                # 相对导入的基准包：仓库根下的顶层包名可能多段（如 domain.world.scene）
                parts = mod[: len(mod) - node.level]
                if node.module:
                    parts += node.module.split(".")
                internal.add(".".join(parts) if parts else mod[0])
            elif node.module:
                (internal if node.module.split(".")[0] in ("core", *LAYER_DIRS) else ext).add(
                    node.module
                )
    return ext, internal, None


def layer_of_module(mod: str) -> str:
    """import 名 → 所在层（``service.planner`` → ``service``）。"""
    top = mod.split(".")[0]
    if top in ("app", "domain", "service", "adapter", "support", "accel"):
        return top
    return "external"


def flag_layer_violation(rule: str, rel: str, m: str, errors: list, warns: list, hit: set) -> None:
    """H3/H4 共用出口：存量违规降级为警告（棘轮），新违规判硬违规。

    与 L1 的 LEGACY_DEBT 同一套过渡态思路（CODING_RULES §3.2）：先把已经存在的
    违规**锁住不再增加**，再由后续 PR 逐条清（每次清完把清单条目删掉）。
    """
    if (rel, m) in LEGACY_LAYER_VIOLATIONS:
        hit.add((rel, m))
        warns.append(f"{rule} [{rel}] 存量跨层依赖 {m} —— " f"{LEGACY_LAYER_VIOLATIONS[(rel, m)]}")
        return
    errors.append(f"{rule} [{rel}] 依赖了 {m}")


def has_tech_debt_marker(path: Path) -> bool:
    try:
        head = "".join(
            path.read_text(encoding="utf-8", errors="replace").splitlines(keepends=True)[:15]
        )
    except OSError:
        return False
    return "TECH_DEBT" in head


def classify(rel: str) -> str:
    """仓库相对路径 → 层。**目录即分层**（不再有手工文件名表）。"""
    if rel == APP_SHELL or rel.startswith("app/"):
        return "app"
    if rel in GUI_DEBT:
        return "gui"
    for prefix, layer in LAYER_BY_PREFIX:
        if rel.startswith(prefix):
            return layer
    return "infra"


def _docstring_node_ids(tree) -> set:
    """模块/类/函数首语句的字符串常量视为 docstring，不参与 CJK 计数。"""
    ids = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            if (
                node.body
                and isinstance(node.body[0], ast.Expr)
                and isinstance(node.body[0].value, ast.Constant)
            ):
                ids.add(id(node.body[0].value))
    return ids


def scan_strings(path: Path):
    """单次 ast 遍历，返回 (中文字面量条数, 功能键未注册列表, 错误或 None)。"""
    try:
        tree = ast.parse(path.read_text(encoding="utf-8", errors="replace"))
    except SyntaxError as e:
        return 0, [], f"{path}: 语法解析失败 {e}"
    doc_ids = _docstring_node_ids(tree)
    cjk = 0
    bad_keys: list[str] = []
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Constant)
            and isinstance(node.value, str)
            and CJK_RE.search(node.value)
            and id(node) not in doc_ids
        ):
            cjk += 1
        if isinstance(node, ast.Call):
            fn = node.func
            name = (
                fn.id
                if isinstance(fn, ast.Name)
                else (fn.attr if isinstance(fn, ast.Attribute) else "")
            )
            if name in FEATURE_CALL_NAMES:
                for arg in node.args:
                    if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
                        bad_keys.append(arg.value)
    return cjk, bad_keys, None


def extract_all_features() -> set[str] | None:
    """从 features.py 静态提取 ALL_FEATURES（DEFAULT_HIDDEN_IN_MVP ∪ {multirobot}）。"""
    try:
        tree = ast.parse((ROOT / "app" / "features.py").read_text(encoding="utf-8"))
    except (OSError, SyntaxError):
        return None
    hidden: set[str] = set()
    for node in ast.walk(tree):
        targets = []
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, ast.AnnAssign):
            targets = [node.target]
        for t in targets:
            if isinstance(t, ast.Name) and t.id == "DEFAULT_HIDDEN_IN_MVP":
                value = node.value
                # 兼容两种写法：裸集合字面量，或 frozenset({...}) 调用包裹
                if (
                    isinstance(value, ast.Call)
                    and isinstance(value.func, ast.Name)
                    and value.func.id in {"frozenset", "set", "list", "tuple"}
                    and value.args
                ):
                    value = value.args[0]
                if isinstance(value, (ast.Set, ast.List, ast.Tuple)):
                    hidden = {
                        e.value
                        for e in value.elts
                        if isinstance(e, ast.Constant) and isinstance(e.value, str)
                    }
    return hidden | {"multirobot"}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--lines", action="store_true", help="打印行数明细")
    args = ap.parse_args()

    errors: list[str] = []
    warns: list[str] = []
    #: 本轮真正命中的存量跨层依赖（用于检出"已消解但仍留在清单里"的条目）
    layer_hits: set = set()
    all_features = extract_all_features()
    if all_features is None:
        warns.append("H6 [app/features.py] 无法解析 ALL_FEATURES，跳过功能键校验")

    for path in iter_py_files():
        rel = rel_id(path)
        layer = classify(rel)
        ext, internal, err = parse_imports(path)
        if err:
            errors.append(err)
            continue
        rel_str = " ".join(sorted(ext)) + " " + " ".join(sorted(internal))

        # H1: GUI 依赖归宿
        if ("PySide6" in rel_str or "OpenGL" in rel_str) and layer not in ("gui", "app"):
            errors.append(
                f"H1 [{rel}] ({layer}) 引用了 PySide6/OpenGL，"
                f"GUI 依赖只允许 app/ 或已登记债务文件"
            )
        if rel in GUI_DEBT and not has_tech_debt_marker(path):
            warns.append(f"H1 [{rel}] 存量 GUI 债务未在头部标注 TECH_DEBT（迁移前冻结新功能）")

        # H2: C++ 边界唯一
        if any(t in rel_str for t in ACCEL_TOKENS) and rel not in ACCEL_BOUNDARY:
            errors.append(
                f"H2 [{rel}] 直接引用 C++ 核心（accel_native / accel._native），"
                f"只允许经 {sorted(ACCEL_BOUNDARY)} 的降级边界"
            )

        # H3: Domain 纯净（插件注册表是扩展机制/DI 容器，允许引用；
        # 详见 CODING_RULES §6：扩展点走注册表）
        if layer == "domain":
            for m in internal:
                if m.endswith("plugins"):
                    continue
                if layer_of_module(m) in ("service", "app", "adapter", "gui"):
                    flag_layer_violation("H3", rel, m, errors, warns, layer_hits)

        # H4: Adapter 禁业务禁 GUI
        if layer == "adapter":
            for m in internal:
                if layer_of_module(m) == "service":
                    flag_layer_violation("H4", rel, m, errors, warns, layer_hits)
            if ("PySide6" in rel_str or "OpenGL" in rel_str) and rel not in GUI_DEBT:
                errors.append(f"H4 [{rel}] Adapter 引用 GUI 库")

        # H7: accel/ 只放硬件加速代码（2026-09-26 口径）——白名单制，防杂项回潮。
        # 它拦的正是"顺手往加速层塞点东西"的漂移：i18n / features / render 都曾这么进去。
        if layer == "accel" and rel != "accel/__init__.py" and not rel.startswith("accel/src/"):
            errors.append(
                f"H7 [{rel}] accel/ 只允许硬件加速代码（src/ 的 C++ 与 __init__ 边界）；"
                f"按归属放 support/ · domain/ · service/ · adapter/ · app/"
            )

        # H8: support/ 是被各层依赖的最底层，自身不得依赖任何层
        if layer == "support":
            for m in internal:
                if layer_of_module(m) in ("app", "domain", "service", "adapter", "accel"):
                    errors.append(f"H8 [{rel}] support/ 不得依赖 {m}" f"（它是各层共享的底层）")

        # L1: 行数分档
        n = sum(1 for _ in path.open(encoding="utf-8", errors="replace"))
        if n > HARD_LIMIT and rel not in LEGACY_DEBT:
            errors.append(f"L1 [{rel}] {n} 行 > 硬限 {HARD_LIMIT} 且未登记债务")
        elif n > SOFT_LIMIT and rel not in LEGACY_DEBT:
            warns.append(f"L1 [{rel}] {n} 行 > 软限 {SOFT_LIMIT}，需标注 TECH_DEBT 或登记债务")
        elif rel in LEGACY_DEBT and n <= SOFT_LIMIT:
            # 棘轮收敛：与 LEGACY_LAYER_VIOLATIONS 同一条纪律——债还完了就要把条目
            # 删掉，否则清单只增不减，后人以为还有一件没做的事。
            warns.append(
                f"L1 [{rel}] 已降到 {n} 行（≤ 软限 {SOFT_LIMIT}），"
                f"请从 LEGACY_DEBT 移除（棘轮收敛）"
            )
        if args.lines:
            print(f"  {n:>5}  {layer:<8} {rel}")

        # H5 / H6: 字符串字面量与功能键
        cjk, keys, serr = scan_strings(path)
        if serr:
            errors.append(serr)
        else:
            if rel not in I18N_HOME and cjk:
                base = CJK_BASELINE.get(rel, 0)
                if cjk > base:
                    errors.append(
                        f"H5 [{rel}] 中文字面量 {cjk} 条 > 基线 {base}，"
                        f"新增用户可见串必须走 i18n（AGENTS.md #15）"
                    )
                elif cjk < base:
                    warns.append(
                        f"H5 [{rel}] 中文字面量 {cjk} 条 < 基线 {base}，" f"基线可下调（棘轮收敛）"
                    )
            if all_features is not None:
                for k in keys:
                    if k not in all_features:
                        errors.append(f'H6 [{rel}] 功能键 "{k}" 未在 ALL_FEATURES 注册')

    # H9: i18n 词条覆盖率（全仓，含 tests/——漏翻只在英文模式下暴露）
    check_i18n_coverage(errors, warns)

    for entry in sorted(LEGACY_LAYER_VIOLATIONS):
        if entry not in layer_hits:
            warns.append(
                f"H3/H4 [{entry[0]}] → {entry[1]} 已不再违规，"
                f"请从 LEGACY_LAYER_VIOLATIONS 移除（棘轮收敛）"
            )

    print("=" * 60)
    for w in warns:
        print(f"WARN  {w}")
    for e in errors:
        print(f"ERROR {e}")
    print(f"—— 检查完成：{len(errors)} 硬违规 / {len(warns)} 警告")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
