"""ADR-002 拆解第一步：``RobotRenderer`` 迁入 ``app/views`` 的回归测试。

守四条底线：

1. **搬迁不改行为**：``main.RobotRenderer`` 仍指向同一个类、常量值不变——
   GLWidget 与既有测试都按这个名字取，断了就是静默失效；
2. **是真搬不是复制**：main.py 里不再有类定义（否则两份实现会各自漂移）；
3. **归属合规**：视图渲染器不碰 PySide6/OpenGL，新建子包已同步打包清单；
4. **位姿数学归 Domain**：``domain.pose.matrix_to_rpy`` 与 URDF 的
   ``rpy_to_matrix`` 互逆，万向锁不崩。
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_ROOT))

import numpy as np  # noqa: E402
import pytest  # noqa: E402

from app.views import robot_renderer as views_mod  # noqa: E402
from domain.pose import matrix_to_rpy  # noqa: E402
from adapter.robot_library.urdf_model import rpy_to_matrix  # noqa: E402


def test_app_module_still_exports_the_moved_symbols():
    """main.py 迁出后必须仍导出同名符号：视口与既有测试都按这个名字取。"""
    import main  # noqa: E402

    assert main.RobotRenderer is views_mod.RobotRenderer
    assert main.HOME_POSE_PATTERN_DEG == views_mod.HOME_POSE_PATTERN_DEG
    assert main.HOME_POSE_MARGIN == views_mod.HOME_POSE_MARGIN
    assert main._matrix_to_rpy is matrix_to_rpy


def test_app_py_no_longer_defines_the_class():
    """搬走了就别留副本——两份实现迟早各自漂移（ADR-002：一次搬一个类）。"""
    source = (_ROOT / "main.py").read_text(encoding="utf-8")
    assert "class RobotRenderer" not in source
    assert "def _matrix_to_rpy" not in source


def test_view_module_has_no_gui_dependency():
    """渲染器只做 URDF → 几何字典；Qt/GL 由调用方（GLWidget）负责。"""
    source = Path(views_mod.__file__).read_text(encoding="utf-8")
    for forbidden in ("PySide6", "OpenGL"):
        assert forbidden not in source, f"{forbidden} 不该出现在 {views_mod.__file__}"


def test_subpackages_are_discoverable_by_packaging():
    """六个顶层包都必须真的进 wheel（ADR-003），且不能靠"记得登记"来保证。

    2026-09-26 起是**六个顶层包**：``accel``（纯硬件加速层，当日由 ``core`` 更名收窄）
    ＋ ``domain`` / ``service`` / ``adapter``（由 core 拆出的三层）＋ ``support``
    （跨层共享支撑）＋ 桌面应用 ``app``。此前 pyproject 是手写 packages 列表，漏过
    render/world/robot_library，wheel 装出来是残的；现已改为 ``packages.find``。
    这条测试因此守两件事：

    1. 发现规则本身（``where = ["."]`` 与覆盖六个前缀的 ``include``）；
    2. 每个包树的每个目录都有 ``__init__.py``——**没有它就等于对 find 隐形**，
       而"少一个子包"不会让任何功能测试变红。

    真·安装验证在 CI 的 ``wheel-smoke`` 作业（构建 wheel → 干净 venv 安装 →
    import + ``core doctor``）。
    """
    text = (_ROOT / "pyproject.toml").read_text(encoding="utf-8")
    assert "[tool.setuptools.packages.find]" in text, "packages 未改为自动发现"
    assert 'where = ["."]' in text, "包发现未从仓库根开始（六个顶层包都在根）"
    for prefix in ("accel*", "app*", "domain*", "service*", "adapter*", "support*"):
        assert f'"{prefix}"' in text, f"包发现的 include 漏掉 {prefix}"

    trees = {
        "accel": ("",),  # 硬件加速层：只剩 __init__ 边界与 src/（C++）
        "support": ("",),
        "domain": ("", "world"),
        "service": ("",),
        "adapter": ("", "post", "robot_library"),
        "app": ("", "views", "window"),
    }
    for top, rels in trees.items():
        for rel in rels:
            pkg_dir = _ROOT / top / rel if rel else _ROOT / top
            assert pkg_dir.is_dir(), f"包目录缺失：{pkg_dir}"
            assert (
                pkg_dir / "__init__.py"
            ).is_file(), f"{pkg_dir} 缺 __init__.py——packages.find 会静默漏掉它"


def test_matrix_to_rpy_inverts_rpy_to_matrix():
    rpy = (0.1, -0.4, 0.9)
    assert np.allclose(matrix_to_rpy(rpy_to_matrix(rpy)), rpy, atol=1e-9)


def test_matrix_to_rpy_handles_gimbal_lock():
    """pitch = ±90° 时 roll/yaw 不可分离：约定 yaw=0，且不能崩。"""
    m = rpy_to_matrix((0.3, math.pi / 2, 0.7))
    roll, pitch, yaw = matrix_to_rpy(m)
    assert math.isclose(pitch, math.pi / 2, abs_tol=1e-9)
    assert yaw == 0.0
    assert math.isfinite(roll)


def test_home_pose_constants_kept_their_values():
    """待机位姿是"看得懂机械臂"的关键，搬迁不许顺手改数。"""
    assert len(views_mod.HOME_POSE_PATTERN_DEG) == 7
    assert 0.0 < views_mod.HOME_POSE_MARGIN < 1.0


def test_renderer_rejects_wrong_dof():
    """关节数量不符要抛 ValueError，且走 i18n——不能退化成 KeyError。"""
    renderer = views_mod.RobotRenderer()
    renderer.joints = {"j1": {"type": "revolute"}, "j2": {"type": "revolute"}}
    renderer.joint_values = {"j1": 0.0, "j2": 0.0}

    with pytest.raises(ValueError):
        renderer.set_joints([0.0])

    renderer.set_joints([0.1, 0.2])
    assert renderer.joint_vector() == [0.1, 0.2]
