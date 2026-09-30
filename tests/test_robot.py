"""``core.Robot`` / ``Pose`` 的基础回归（原生在 ``python/tests/``，2026-09-25 归位）。

归位原因：仓库曾有两个测试根（``tests/`` 与 ``python/tests/``），CI 要写
``pytest tests/ python/tests/`` 才能覆盖全；而这个文件不带 ``sys.path`` 前导，
单独跑还得先 ``pip install -e .``。并入 ``tests/`` 后补齐前导，与其余测试同口径。
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
for _p in (REPO_ROOT, REPO_ROOT):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import numpy as np  # noqa: E402

from domain.pose import Pose
from domain.robot import Robot  # noqa: E402  (F401: Robot 供 TestRobotMock 使用)


class TestPose:
    def test_identity(self):
        p = Pose(np.eye(4))
        x, y, z, rx, ry, rz = p.to_xyz_rpy()
        assert np.isclose(x, 0) and np.isclose(y, 0) and np.isclose(z, 0)

    def test_xyz_rpy_roundtrip(self):
        p = Pose.from_xyz_rpy(100, 200, 300, 45, 30, 60)
        x, y, z, rx, ry, rz = p.to_xyz_rpy()
        assert np.isclose(x, 100)
        assert np.isclose(y, 200)
        assert np.isclose(z, 300)
        assert np.isclose(rx, 45, atol=1e-3)
        assert np.isclose(ry, 30, atol=1e-3)
        assert np.isclose(rz, 60, atol=1e-3)

    def test_relative_to(self):
        base = Pose.from_xyz_rpy(500, 0, 0, 0, 0, 0)
        target = Pose.from_xyz_rpy(600, 100, 50, 0, 0, 0)
        rel = target.relative_to(base)
        x, y, z, *_ = rel.to_xyz_rpy()
        assert np.isclose(x, 100)
        assert np.isclose(y, 100)
        assert np.isclose(z, 50)


class TestRobotMock:
    def test_dof(self):
        # This requires a real URDF to run; placeholder for CI
        pass
