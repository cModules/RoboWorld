"""程序版本对比（``progdiff``）与版本库（``ProgramVersionStore``）的回归测试。

要守住的三件事：

* **对齐要真的对**——插入/删除一条指令，其余指令必须保持对齐（不报"全变了"）；
  同 op 指令对齐后要给出字段级差异：关节角报"第几轴 Δ多少度"、位姿报
  "TCP 变了多少 mm/°"、IO 报"哪个信号什么电平变了"；
* **序列化要无损**——``serialize → deserialize`` 之后指纹必须逐字节一致
  （指纹是"改没改"的唯一仲裁，往返有损就等于指纹说谎）；
* **版本库要多后端同行为**——Memory / Local / HTTP 三种 ``CloudTransport``
  挂同一个 ``ProgramVersionStore`` 必须给出同一套结果（复用 v1.0.6 的参考
  服务端走真 HTTP）；``plan=True`` 的节拍/路径对比走真规划器。
"""

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

import orolp as orp  # noqa: E402
from orolp import Pose, ProgramBuilder, Robot  # noqa: E402
from orolp.progdiff import (  # noqa: E402
    ProgramDiffError,
    ProgramVersionStore,
    deserialize_program,
    diff_programs,
    program_fingerprint,
    serialize_program,
)
from orolp.twinsync import LocalCloudStore, MemoryCloudStore  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
URDF = REPO_ROOT / "models/ur/ur5e/ur5e.urdf"
Q0 = [0.0, -30.0, 30.0, 0.0, 60.0, 0.0]


# --------------------------------------------------------------------------- #
# 夹具
# --------------------------------------------------------------------------- #


def make_builder(j3: float = 30.0, z: float = 300.0, speed: float = 50.0,
                 extra_tail: bool = False) -> ProgramBuilder:
    b = (ProgramBuilder("UR5e")
         .moveJ([0.0, -30.0, j3, 0.0, 60.0, 0.0], speed=40)
         .moveL(Pose.from_xyz_rpy(500.0, 0.0, z, 0.0, 0.0, 0.0),
                speed=speed, blend=8)
         .setDO("gripper_close", True)
         .moveL(Pose.from_xyz_rpy(500.0, 200.0, z, 0.0, 0.0, 0.0), speed=speed))
    if extra_tail:
        b.moveJ([10.0, -20.0, j3, 0.0, 45.0, 0.0], speed=40)
    return b


@pytest.fixture
def robot():
    return Robot(str(URDF), "UR5e")


# --------------------------------------------------------------------------- #
# ① 对齐与字段级差异
# --------------------------------------------------------------------------- #


def test_identical_programs_have_no_changes():
    d = diff_programs(make_builder(), make_builder())
    assert not d.has_changes
    assert d.n_equal == 4  # moveJ + moveL + setDO + moveL
    assert "逐字段一致" in d.summary()


def test_joint_target_change_reports_axis_and_degrees():
    d = diff_programs(make_builder(), make_builder(j3=45.0))
    assert d.has_changes and d.n_changed == 1
    ch = d.changes[0]
    assert "关节 3" in ch.detail and "15" in ch.detail
    assert any(f.startswith("target") for f in ch.fields)


def test_pose_target_change_reports_tcp_mm():
    d = diff_programs(make_builder(), make_builder(z=310.0))
    # z 同时出现在两条 moveL 的目标里，两条都要报
    assert d.n_changed == 2
    assert all("TCP 10.00mm" in c.detail for c in d.changes if c.kind == "changed")


def test_speed_change_is_detected():
    d = diff_programs(make_builder(), make_builder(speed=70.0))
    assert d.n_changed == 2  # 两条 moveL 都带 speed
    assert all("speed" in c.detail for c in d.changes if c.kind == "changed")


def test_motion_type_change_detected():
    a = ProgramBuilder("UR5e").moveJ([0.0, -30.0, 30.0, 0.0, 60.0, 0.0]).build()
    b = ProgramBuilder("UR5e").moveL(Pose.from_xyz_rpy(400, 0, 400, 0, 0, 0)).build()
    d = diff_programs(a, b)
    assert d.n_changed == 1
    assert "运动类型" in d.changes[0].detail


def test_inserted_motion_keeps_rest_aligned():
    """中间插一条指令：后面的指令必须保持对齐，而不是整体报"变了"。"""
    a = make_builder().build()
    b = make_builder(extra_tail=True).build()
    d = diff_programs(a, b)
    assert d.n_added == 1 and d.n_changed == 0
    added = next(c for c in d.changes if c.kind == "added")
    assert added.index_a is None and added.index_b == 4


def test_removed_motion_detected():
    a = make_builder(extra_tail=True).build()
    b = make_builder().build()
    d = diff_programs(a, b)
    assert d.n_removed == 1 and d.n_changed == 0


def test_io_signal_rename_is_a_field_change_not_a_shift():
    """setDO 换信号名：按信号名对齐后报"信号字段变了"，其余指令不许错位。"""
    a = make_builder().build()
    b = (ProgramBuilder("UR5e")
         .moveJ([0.0, -30.0, 30.0, 0.0, 60.0, 0.0], speed=40)
         .moveL(Pose.from_xyz_rpy(500, 0, 300, 0, 0, 0), speed=50, blend=8)
         .setDO("gripper_open", True)
         .moveL(Pose.from_xyz_rpy(500, 200, 300, 0, 0, 0), speed=50)
         .build())
    d = diff_programs(a, b)
    assert d.n_changed == 1
    ch = next(c for c in d.changes if c.kind == "changed")
    assert "signal" in ch.fields and "gripper" in ch.detail
    assert ch.index_a == 2 and ch.index_b == 2


def test_do_value_change_is_a_field_change():
    a = ProgramBuilder("UR5e").moveJ(Q0).setDO("gripper_close", True).build()
    b = ProgramBuilder("UR5e").moveJ(Q0).setDO("gripper_close", False).build()
    d = diff_programs(a, b)
    assert d.n_changed == 1
    assert "True→False" in next(c for c in d.changes if c.kind == "changed").detail


def test_wait_time_seconds_delta():
    a = ProgramBuilder("UR5e").moveJ(Q0).waitTime(1.5).build()
    b = ProgramBuilder("UR5e").moveJ(Q0).waitTime(2.0).build()
    d = diff_programs(a, b)
    assert d.n_changed == 1
    assert "seconds 1.5→2" in next(c for c in d.changes if c.kind == "changed").detail


def test_accepts_raw_item_list():
    items = list(make_builder().build().items)
    d = diff_programs(items, items)
    assert not d.has_changes


def test_plan_requires_robot():
    d = diff_programs(make_builder(), make_builder(j3=40.0), plan=True)
    assert d.errors and d.cycle_a is None  # 结构对比仍在
    assert d.has_changes


# --------------------------------------------------------------------------- #
# ② plan=True：真规划器的节拍与 TCP 路径对比
# --------------------------------------------------------------------------- #


def test_plan_true_reports_cycle_and_path_offset(robot):
    prog_a = make_builder().build()
    prog_b = make_builder(z=320.0).build()
    prog_a.robot = robot
    prog_b.robot = robot
    d = diff_programs(prog_a, prog_b, robot=robot, plan=True)
    assert not d.errors, d.errors
    assert d.cycle_a is not None and d.cycle_b is not None
    assert d.cycle_delta is not None
    assert d.path_max_mm is not None and d.path_max_mm > 5.0  # z 抬了 20mm，路径必然偏
    assert "节拍" in d.summary()


def test_plan_true_identical_programs_zero_offset(robot):
    prog = make_builder().build()
    prog.robot = robot
    d = diff_programs(prog, prog, robot=robot, plan=True)
    assert not d.errors
    assert abs(d.cycle_delta) < 1e-9
    assert d.path_max_mm == pytest.approx(0.0, abs=1e-6)


def test_plan_true_with_degenerate_target_still_diffs(robot):
    """对面是条"规划器只能给出空轨迹"的程序时：不崩、错误如实上账、结构差异照出。

    （实测规划器对远超工作空间的直线目标不抛错、返回空轨迹——progdiff 对
    "空轨迹"的口径是 cycle=None 级别的缺失，路径偏移跳过，绝不编数字。）
    """
    prog = make_builder().build()
    prog.robot = robot
    far = (ProgramBuilder("UR5e")
           .moveL(Pose.from_xyz_rpy(5000, 5000, 5000, 0, 0, 0)).build())
    far.robot = robot
    d = diff_programs(prog, far, robot=robot, plan=True)
    assert d.has_changes  # 结构对比照常给出
    # 路径偏移：新版只有 1 个点、旧版正常，能算就给数，算不了就 None，不造假
    assert d.path_max_mm is None or d.path_max_mm >= 0.0


# --------------------------------------------------------------------------- #
# ③ 序列化与指纹
# --------------------------------------------------------------------------- #


def test_serialize_deserialize_roundtrip_keeps_fingerprint():
    prog = make_builder().build()
    data = serialize_program(prog)
    json.dumps(data, ensure_ascii=False)  # 必须是 JSON 安全的
    back = deserialize_program(data)
    assert program_fingerprint(back) == program_fingerprint(prog)


def test_fingerprint_changes_when_one_joint_changes():
    fp_a = program_fingerprint(make_builder(j3=30.0))
    fp_b = program_fingerprint(make_builder(j3=30.5))
    assert fp_a != fp_b


def test_deserialized_program_replannable(robot):
    """往返回来的程序要能直接进规划器——版本库 diff(plan=True) 的前提。"""
    prog = make_builder().build()
    prog.robot = robot
    revived = deserialize_program(serialize_program(prog), robot)
    d = diff_programs(prog, revived, robot=robot, plan=True)
    assert not d.errors, d.errors
    assert not d.has_changes


# --------------------------------------------------------------------------- #
# ④ 版本库：多后端同行为
# --------------------------------------------------------------------------- #


def _exercise_store(store, name="PICK"):
    a, b = make_builder(), make_builder(j3=45.0)
    v1 = store.save(name, a, code="CODE-V1", meta={"station": "A"})
    v2 = store.save(name, b, code="CODE-V2")
    v3 = store.save(name, a)  # 重新上传同一份也要留痕，版本号照常递增
    assert (v1, v2, v3) == ("v1", "v2", "v3")
    assert store.versions(name) == ["v1", "v2", "v3"]
    assert store.latest(name) == "v3"
    rec = store.load(name, v1)
    assert rec["meta"] == {"station": "A"} and rec["n_motions"] == 3  # 4 条指令含 1 条 IO
    assert store.load_code(name, v1) == "CODE-V1"
    assert store.load_code(name, v3) is None  # 没存 code 就如实返回 None
    d = store.diff(name, v1, v2)
    assert d.has_changes and d.n_changed >= 1
    d_same = store.diff(name, v1, v3)
    assert not d_same.has_changes and "逐字段一致" in d_same.summary()
    with pytest.raises(ProgramDiffError):
        store.load(name, "v99")
    assert store.latest("从来没有的程序") is None
    assert store.versions("从来没有的程序") == []


def test_store_on_memory_transport():
    _exercise_store(ProgramVersionStore(MemoryCloudStore()))


def test_store_on_local_transport(tmp_path):
    _exercise_store(ProgramVersionStore(LocalCloudStore(tmp_path / "verlib")))


def test_store_namespaces_do_not_collide():
    store = ProgramVersionStore(MemoryCloudStore())
    store.save("PICK", make_builder())
    store.save("PLACE", make_builder(j3=10.0))
    assert store.versions("PICK") == ["v1"]
    assert store.versions("PLACE") == ["v1"]
    d = store.diff("PICK", "v1", "v1")
    assert not d.has_changes


def test_store_name_sanitized_for_cloud_keys(tmp_path):
    store = ProgramVersionStore(LocalCloudStore(tmp_path / "verlib"))
    store.save("../evil 程序", make_builder())
    assert store.versions("../evil 程序") == ["v1"]  # 调用方怎么写都能取回


# --------------------------------------------------------------------------- #
# ⑤ 导出与示例脚本
# --------------------------------------------------------------------------- #


def test_exports_are_reachable_from_package_root():
    assert orp.diff_programs is diff_programs
    assert orp.ProgramVersionStore is ProgramVersionStore
    assert orp.serialize_program is serialize_program
    assert orp.deserialize_program is deserialize_program
    assert orp.program_fingerprint is program_fingerprint


def test_example_script_runs_end_to_end():
    """示例脚本是文档的一部分——必须真的能跑完（走真 HTTP 版本库）。"""
    script = REPO_ROOT / "examples/prog_diff_demo.py"
    result = subprocess.run(
        [sys.executable, str(script)],
        cwd=str(REPO_ROOT),
        env={"PYTHONPATH": str(REPO_ROOT / "python"), "PATH": "/usr/bin:/bin"},
        capture_output=True,
        text=True,
        timeout=180,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "全部通过" in result.stdout
    assert "规划器口径" in result.stdout  # 诚实披露要真的打在输出里
