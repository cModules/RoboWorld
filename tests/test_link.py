"""控制器连接层（代码上传）的回归测试。

守住离线编程的最后一公里：FileLink 落盘导出、SimLink 回读实际关节角、
Program.deploy 便捷入口、未连接即报错。
"""
import json
import sys
import tempfile
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

import orolp as orp  # noqa: E402
from orolp import Pose, Robot  # noqa: E402
from orolp.debug import Deviation  # noqa: E402
from orolp.link import Deployment, FileLink, LinkError, SimLink  # noqa: E402

URDF = Path(__file__).resolve().parent.parent / "models/ur/ur5e/ur5e.urdf"
Q0 = [0.0, -30.0, 30.0, 0.0, 60.0, 0.0]


@pytest.fixture
def robot():
    return Robot(str(URDF), "UR5")


@pytest.fixture
def program(robot):
    return (
        orp.ProgramBuilder(robot)
        .moveJ(Q0, speed=40)
        .moveL(Pose.from_xyz_rpy(500, 0, 300, 0, 0, 0), speed=50, blend=8)
        .moveL(Pose.from_xyz_rpy(500, 200, 300, 0, 0, 0), speed=50, blend=8)
        .build()
    )


def test_filelink_writes_code_and_manifest(program):
    d = tempfile.mkdtemp()
    fl = FileLink(d)
    fl.connect()
    dep = program.deploy(fl, "ur_polyscope", name="pick_place")
    assert dep.ok
    assert dep.handle.endswith(".script")  # UR 扩展名
    code_file = Path(dep.handle)
    assert code_file.exists()
    manifest = json.loads((Path(d) / "pick_place.manifest.json").read_text())
    assert manifest["post_processor"] == "ur_polyscope"
    assert manifest["ext"] == "script"
    assert manifest["lines"] > 0
    # 回读能力应为 False（离线导出）
    assert fl.is_actual_feedback is False
    assert fl.read_joints() is None
    assert fl.list_programs() == ["pick_place"]


def test_filelink_default_ext_and_extension_override(program):
    d = tempfile.mkdtemp()
    fl = FileLink(d)
    fl.connect()
    dep = program.deploy(fl, "kuka_krc4", name="weld", metadata={"ext": "src"})
    assert dep.handle.endswith(".src")


def test_simlink_reads_back_joints_and_pose(robot):
    sl = SimLink(robot)
    sl.connect()
    q = np.array([5.0, -30.0, 30.0, 0.0, 60.0, 0.0])
    sl.set_joints(q)
    back = sl.read_joints()
    assert back is not None
    assert np.allclose(back, q, atol=1e-9)
    pose = sl.get_pose()
    assert pose is not None
    # 与机器人正解一致
    assert np.allclose(pose.to_xyz_rpy()[:3], robot.forward_kinematics(q).to_xyz_rpy()[:3], atol=1e-6)
    assert sl.is_actual_feedback is True


def test_simlink_upload_program_handle(robot):
    sl = SimLink(robot)
    sl.connect()
    handle = sl.upload_program("prog1", "PTP ...", {"post_processor": "kuka_krc4"})
    assert handle == "sim://prog1"
    assert sl.list_programs() == ["prog1"]


def test_disconnected_link_raises(robot, program):
    # 未 connect() 直接 deploy 应抛 LinkError
    fl = FileLink(tempfile.mkdtemp())
    with pytest.raises(LinkError):
        program.deploy(fl, "ur_polyscope")
    sl = SimLink(robot)
    with pytest.raises(LinkError):
        sl.read_joints()


def test_deploy_returns_deployment_dataclass(program):
    d = tempfile.mkdtemp()
    fl = FileLink(d)
    fl.connect()
    dep = program.deploy(fl, name="pp")
    assert isinstance(dep, Deployment)
    assert dep.program_name == "pp"
    assert dep.ok is True
    # 品牌应从 URDF 路径推断
    assert dep.brand == "ur"
