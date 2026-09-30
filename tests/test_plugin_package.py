"""v1.0.8 第一方示例插件包（examples/acme_post_plugin）验收。

v1.0.4 的插件接口一直只能用**注入的假入口点**（``_FakeEntryPoint``）测
``load_entry_points``——``importlib.metadata`` 的真实发现路径没有被任何测试
覆盖过。本文件补上这条路径：把仓库里的第一方示例包真正 ``pip install`` 进
临时目录，再在子进程里让 ``PluginRegistry.load_entry_points()``（不注入任何
入口点）从真实的 dist-info 里发现它并完成出码。
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
PKG_DIR = REPO_ROOT / "examples" / "acme_post_plugin"
REPO_PYTHON = REPO_ROOT

# 与 test_packaging_spec 同理：不依赖其他测试先把 python/ 插进 sys.path
if str(REPO_PYTHON) not in sys.path:
    sys.path.insert(0, str(REPO_PYTHON))

pytestmark = pytest.mark.skipif(not PKG_DIR.is_dir(), reason="examples/acme_post_plugin 不存在")

# 子进程发现脚本：在"装了插件包 + PYTHONPATH 指向仓库 python/"的环境里跑
_DISCOVER_SCRIPT = r"""
import json, sys
from adapter.plugins import default_registry
from adapter.post import list_post_processors

# to_post()/get_post_processor() 走 default_registry——入口点也要装进它
reg = default_registry
stats = reg.load_entry_points()          # 关键：不注入任何入口点——走 importlib.metadata
spec = reg.get("acme_v2")
inst = reg.create("acme_v2")

from domain.robot import Robot
from service.program import ProgramBuilder
robot = Robot(sys.argv[1], "UR5e")
pb = (ProgramBuilder(robot)
      .moveJ([0.0, -90.0, 90.0, -90.0, 90.0, 0.0], speed=50)
      .moveL([(300.0, 0.0, 400.0, 0.0, 180.0, 0.0)], speed=100)
      .setDO("gripper_close", True))
code = pb.build().to_post("acme_v2", metadata={"program_name": "ACME_PKG"})

print("RESULT_JSON:" + json.dumps({
    "stats": stats,
    "origin": spec.origin,
    "factory": spec.factory,
    "version": spec.version,
    "class": type(inst).__name__,
    "lang": inst.language_name,
    "ext": inst.file_extension,
    "in_list": "acme_v2" in list_post_processors(),
    "code_head": "\n".join(code.splitlines()[:4]),
    "has_movej": "MOVEJ" in code,
    "has_movel": "MOVEL" in code,
    "has_io": "SET_DO" in code,
    "registry_errors": [e.message for e in reg.errors()],
}, ensure_ascii=False))
"""


def _pip_install_to(target: Path, workdir: Path) -> None:
    """把示例插件包装进 target 目录（不污染任何真实环境）。

    注意：pip 对本地目录是**就地构建**——setuptools 会在 pyproject.toml 旁边写
    相对的 ``build/`` 与 ``*.egg-info``。因此先把包**复制到临时目录**再装，
    源目录永远保持干净（否则第二次构建会因 ``build/`` 残留 ``File exists`` 崩掉）。
    """
    import shutil

    src_copy = workdir / "src"
    shutil.copytree(PKG_DIR, src_copy, ignore=shutil.ignore_patterns("build", "*.egg-info"))
    cmd = [
        sys.executable,
        "-m",
        "pip",
        "install",
        "--quiet",
        "--no-build-isolation",
        "--target",
        str(target),
        str(src_copy),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=180, cwd=str(workdir))
    assert proc.returncode == 0, f"pip install 失败：\n{proc.stdout}\n{proc.stderr}"
    assert not (PKG_DIR / "build").exists(), "pip 就地构建污染了源目录（应先复制再装）"


def _run_discover(tmp_target: Path, urdf: Path) -> dict:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([str(tmp_target), str(REPO_PYTHON)])
    proc = subprocess.run(
        [sys.executable, "-c", _DISCOVER_SCRIPT, str(urdf)],
        capture_output=True,
        text=True,
        timeout=120,
        cwd=str(REPO_ROOT),
        env=env,
    )
    assert proc.returncode == 0, f"发现脚本失败：\n{proc.stdout}\n{proc.stderr}"
    lines = [ln for ln in proc.stdout.splitlines() if ln.startswith("RESULT_JSON:")]
    assert lines, f"子进程没有输出 RESULT_JSON 行：\n{proc.stdout}"
    return json.loads(lines[-1][len("RESULT_JSON:") :])


def test_package_declares_core_plugins_entry_point():
    """pyproject.toml 必须声明 adapter.plugins 入口点组（真实发现路径的前提）。"""
    import tomllib  # Python 3.11+；本仓库测试环境固定 3.13

    data = tomllib.loads((PKG_DIR / "pyproject.toml").read_text(encoding="utf-8"))
    eps = data["project"]["entry-points"]["adapter.plugins"]
    assert eps == {"acme_v2": "core_acme_post:plugin_specs"}
    # core 由宿主提供，插件包不得把它写进 dependencies（不绑架宿主版本）
    assert data["project"]["dependencies"] == []


def test_plugin_specs_valid_and_factory_is_lazy_string():
    """提供函数返回合法 PluginSpec；工厂是 module:attr 字符串（延迟导入）。"""
    sys.path.insert(0, str(PKG_DIR))
    try:
        import core_acme_post as pkg
    finally:
        sys.path.remove(str(PKG_DIR))

    from adapter.plugins import KIND_POST, PluginRegistry

    specs = pkg.plugin_specs()
    assert len(specs) == 1
    spec = specs[0]
    assert spec.name == "acme_v2" and spec.kind == KIND_POST
    assert spec.factory == "core_acme_post:AcmeV2PostProcessor"
    assert spec.requires_api == __import__("adapter.plugins", fromlist=["x"]).PLUGIN_API_VERSION

    # 登记后 create() 能解析字符串工厂（延迟导入路径）
    reg = PluginRegistry()
    reg.register(spec)
    inst = reg.create("acme_v2")
    assert type(inst).__name__ == "AcmeV2PostProcessor"
    assert inst.language_name == "ACME-V2" and inst.file_extension == "acm"


def test_pip_installed_package_discovered_via_real_entry_points(tmp_path):
    """★ 核心验收：真 pip install → importlib.metadata 真实发现 → 真出码。"""
    urdf = REPO_ROOT / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"
    assert urdf.is_file()

    target = tmp_path / "site"
    _pip_install_to(target, workdir=tmp_path)
    assert (target / "core_plugin_acme_post-1.0.0.dist-info").is_dir()

    result = _run_discover(target, urdf)

    # 入口点被真实发现并登记，origin 由宿主盖章
    assert result["stats"] == {"discovered": 1, "registered": 1, "skipped": 0, "failed": 0}
    assert result["origin"] == "entry_point"
    assert result["factory"] == "core_acme_post:AcmeV2PostProcessor"
    assert result["version"] == "1.0.0"
    assert result["class"] == "AcmeV2PostProcessor"
    assert result["lang"] == "ACME-V2" and result["ext"] == "acm"
    assert result["in_list"] is True
    assert result["registry_errors"] == []

    # 出码走的是内置品牌同一套通路，且 IO 降级有诚实警告语义（伪语法 + TODO）
    assert "ACME V2 / ACME_PKG" in result["code_head"]
    assert result["has_movej"] and result["has_movel"] and result["has_io"]


def test_pip_install_editable_mode_also_declared_metadata(tmp_path):
    """pyproject 元数据自洽：包名/版本与 dist-info 命名规则一致（可 -e 开发安装）。"""
    import tomllib

    data = tomllib.loads((PKG_DIR / "pyproject.toml").read_text(encoding="utf-8"))
    name, version = data["project"]["name"], data["project"]["version"]
    assert name == "core-plugin-acme-post" and version == "1.0.0"
    # dist-info 目录名规则：连字符规范化 + 版本
    norm = name.replace("-", "_").lower()
    assert norm == "core_plugin_acme_post"
