"""``core`` 命令行入口的回归测试（ADR-003）。

背景：``pyproject.toml`` 的 ``[project.scripts] core = "app.cli:main"`` 长期指向
一个**不存在**的文件——``pip install core`` 之后敲 ``core`` 直接
ModuleNotFoundError，而这件事没有任何测试会变红（没人会去敲它）。

这里守三层：

1. 三个子命令都能跑通并给出人话输出（doctor / post / world）；
2. 诊断类命令**遇到缺失要如实报错**（退出码 1），不能"静默成功"；
3. 入口点真的可被 ``python -m app.cli`` 调用（子进程，绕开测试进程的
   ``sys.path`` 便利，模拟干净安装后的真实调用路径）。
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
for _p in (REPO_ROOT, REPO_ROOT):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from app import cli  # noqa: E402


# --------------------------------------------------------------------------- #
# doctor
# --------------------------------------------------------------------------- #
def test_doctor_reports_ok_in_a_healthy_env(capsys):
    """本仓库开发环境下 doctor 必须判定可用（依赖齐全）。"""
    code = cli.main(["doctor"])
    out = capsys.readouterr().out

    assert code == 0, f"环境自检失败：\n{out}"
    assert "RoboWorld" in out
    assert "Python" in out
    assert "预置工作站" in out or "workcell" in out.lower()


def test_doctor_counts_missing_core_dependency(monkeypatch, capsys):
    """核心依赖缺失必须退出码 1 + 点名——诊断命令不能"报喜不报忧"。"""
    monkeypatch.setattr(cli, "_CORE_DEPS", ("definitely_not_installed_pkg",))

    code = cli.main(["doctor"])
    out = capsys.readouterr().out

    assert code == 1, out
    assert "definitely_not_installed_pkg" in out


def test_repo_root_points_at_source_tree():
    """源码树运行时资源根是仓库根（assets/models/ 与 assets/ 都挂在那里）。"""
    if hasattr(sys, "_MEIPASS"):
        return  # 冻结运行时改用 _MEIPASS，见 cli.repo_root
    assert (cli.repo_root() / "assets" / "models").is_dir()
    assert (cli.repo_root() / "assets" / "cells").is_dir()


# --------------------------------------------------------------------------- #
# post
# --------------------------------------------------------------------------- #
def test_post_list_mentions_builtin_brands(capsys):
    code = cli.main(["post", "--list"])
    out = capsys.readouterr().out

    assert code == 0
    assert "kuka_krc4" in out
    # 注意：模块名是 ur_script.py，但注册名（brand_controller）是 ur_polyscope
    assert "ur_polyscope" in out


def test_post_check_echoes_metadata(capsys):
    code = cli.main(["post", "--check", "kuka"])
    out = capsys.readouterr().out

    assert code == 0, out
    assert "kuka_krc4" in out
    assert ".src" in out  # KUKA KRC4 的扩展名


def test_post_check_unknown_name_fails(capsys):
    """未知品牌要如实报错并给出可用清单线索，而不是静默返回 0。"""
    code = cli.main(["post", "--check", "no_such_brand"])
    out = capsys.readouterr().out

    assert code == 1
    assert "no_such_brand" in out


# --------------------------------------------------------------------------- #
# world
# --------------------------------------------------------------------------- #
def test_world_list_shows_preset_cells(capsys):
    code = cli.main(["world", "--list"])
    out = capsys.readouterr().out

    assert code == 0, out
    cells = sorted(p.stem for p in (REPO_ROOT / "assets" / "cells").glob("*.json"))
    assert cells, "assets/cells 下没有预置场景，测试前提不成立"
    assert cells[0] in out


def test_world_show_loads_the_scene(capsys):
    """--show 要真的把 JSON 载进 Scene 并给出摘要（不是只查文件名）。"""
    cells = sorted((REPO_ROOT / "assets" / "cells").glob("*.json"))
    if not cells:
        return

    code = cli.main(["world", "--show", cells[0].stem])
    out = capsys.readouterr().out

    assert code == 0, out
    assert cells[0].stem in out
    assert "机器人" in out or "robot" in out.lower()


def test_world_show_unknown_name_fails(capsys):
    code = cli.main(["world", "--show", "no_such_cell"])
    out = capsys.readouterr().out

    assert code == 1
    assert "no_such_cell" in out


# --------------------------------------------------------------------------- #
# 入口点
# --------------------------------------------------------------------------- #
def test_no_subcommand_prints_help(capsys):
    """裸敲 ``core`` 打帮助并返回 0——这是用法提示，不是错误。"""
    code = cli.main([])
    out = capsys.readouterr().out

    assert code == 0
    assert "doctor" in out and "post" in out and "world" in out


def test_module_entry_point_runs_in_a_clean_process():
    """``python -m app.cli doctor`` 必须能跑（entry point 的真实调用形态）。"""
    env = dict(os.environ)
    env["PYTHONPATH"] = str(REPO_ROOT)
    proc = subprocess.run(
        [sys.executable, "-m", "app.cli", "doctor"],
        capture_output=True,
        text=True,
        env=env,
        cwd=str(REPO_ROOT),
        timeout=120,
    )
    assert proc.returncode == 0, proc.stderr
    assert "RoboWorld" in proc.stdout
