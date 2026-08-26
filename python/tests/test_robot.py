import pytest
import numpy as np
from orolp import Robot, Pose


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
