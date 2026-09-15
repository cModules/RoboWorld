"""数字孪生与云协作（v1.0.3）的回归测试。

三块内容分别守住：

1. **合成真机模型**（:class:`PlantModel` / :class:`PlantLink`）——误差项按物理规律
   作用（常量偏置、随关节角放大、一阶滞后稳态误差 = τ·ω），且可复现；
2. **偏差时间序列**（:class:`TwinTrace` / :class:`TwinReport`）——采集、跨源对齐、
   差分、速度分解、JSON/JSONL 往返；
3. **云端通道**（:class:`CloudLink`）——程序与轨迹上下行、真机状态回读的
   新鲜度判定，以及键的路径穿越防护。
"""

import json
import math
import sys
import tempfile
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

import orolp as orp  # noqa: E402
from orolp import Pose, Robot  # noqa: E402
from orolp.debug import DebugSession  # noqa: E402
from orolp.link import ControllerLink, FileLink, SimLink  # noqa: E402
from orolp.twinsync import (  # noqa: E402
    TRACE_HEADER_KIND,
    TRACE_SAMPLE_KIND,
    AlignedSample,
    CloudLink,
    LocalCloudStore,
    MemoryCloudStore,
    PlantLink,
    PlantModel,
    TwinError,
    TwinReport,
    TwinSnapshot,
    TwinTrace,
    align_traces,
    capture_from_session,
    capture_plant_trace,
    compare_traces,
    deviation_chart_svg,
    normalize_key,
    pose_deviation,
)

URDF = Path(__file__).resolve().parent.parent / "models/ur/ur5e/ur5e.urdf"
Q0 = [0.0, -30.0, 30.0, 0.0, 60.0, 0.0]


@pytest.fixture
def robot():
    return Robot(str(URDF), "UR5")


@pytest.fixture
def traj(robot):
    prog = (
        orp.ProgramBuilder(robot)
        .moveJ(Q0, speed=40)
        .moveL(Pose.from_xyz_rpy(500, 0, 300, 0, 0, 0), speed=50, blend=8)
        .moveL(Pose.from_xyz_rpy(500, 200, 300, 0, 0, 0), speed=50, blend=8)
        .build()
    )
    return prog.simulate()


@pytest.fixture
def sim_trace(traj, robot):
    link = SimLink(robot)
    link.connect()
    return capture_from_session(
        DebugSession(traj, link, robot=robot), source="sim", program_name="TWIN_DEMO"
    )


def _frame(q, t=0.0, pose=None):
    """构造一个"有实测回读"的快照（默认实测＝规划）。"""
    q = np.asarray(q, dtype=np.float64)
    return TwinSnapshot(
        t=float(t),
        planned_q=q,
        actual_q=q.copy(),
        planned_pose=pose if pose is not None else Pose.from_xyz_rpy(float(q[0]), 0, 0, 0, 0, 0),
        pos_dev_mm=0.0,
        rot_dev_deg=0.0,
        joint_dev_deg=0.0,
    )


def _trace_with_speeds():
    """两条速度截然不同的样本组：低速段偏差小、高速段偏差大。"""
    qs = [np.zeros(6), np.zeros(6) + 0.01, np.zeros(6) + 1.0, np.zeros(6) + 3.0]
    ts = [0.0, 2.0, 3.0, 3.2]
    snaps = [
        _frame(q, t) for q, t in zip(qs, ts)
    ]
    for s, dev in zip(snaps, [0.4, 0.5, 1.0, 2.0]):
        s.pos_dev_mm = dev
        s.rot_dev_deg = dev * 0.1
    return TwinTrace(snapshots=snaps, source="plant")


# =========================================================================== #
# 1. 合成真机模型
# =========================================================================== #


def test_plant_bias_expands_scalar_to_all_axes():
    model = PlantModel(joint_bias_deg=0.1)
    assert np.allclose(model.bias_vector(6), 0.1)


def test_plant_bias_per_axis_sequence():
    model = PlantModel(joint_bias_deg=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert np.allclose(model.bias_vector(6), [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])


def test_plant_param_length_mismatch_raises():
    model = PlantModel(joint_bias_deg=[0.1, 0.2])
    with pytest.raises(TwinError, match="关节数"):
        model.bias_vector(6)


def test_plant_negative_lag_raises():
    with pytest.raises(TwinError, match="lag_tau"):
        PlantModel(lag_tau=-0.01)


def test_plant_negative_noise_raises():
    with pytest.raises(TwinError, match="noise_deg"):
        PlantModel(noise_deg=-1.0)


def test_plant_scale_error_le_minus_one_raises():
    with pytest.raises(TwinError, match="scale_error"):
        PlantModel(scale_error=-1.5)


def test_plant_zero_dof_raises():
    with pytest.raises(TwinError, match="关节数"):
        PlantModel().bias_vector(0)


def test_plant_summarize_marks_synthetic_and_lists_terms():
    text = PlantModel(joint_bias_deg=0.05, scale_error=1e-3, noise_deg=0.002, lag_tau=0.01).summarize()
    assert "合成" in text
    for token in ("零位偏置", "传动比误差", "编码器噪声", "伺服滞后"):
        assert token in text


def test_plant_summarize_no_error_case():
    assert "无误差" in PlantModel().summarize()


def test_plant_link_perfect_model_reads_back_command(robot):
    link = PlantLink(robot, PlantModel())
    link.connect()
    q = np.asarray(Q0, dtype=np.float64)
    link.command(q, at_time=0.0)
    assert np.allclose(link.read_joints(), q, atol=1e-12)


def test_plant_link_constant_bias_shifts_readback(robot):
    link = PlantLink(robot, PlantModel(joint_bias_deg=0.5))
    link.connect()
    q = np.asarray(Q0, dtype=np.float64)
    link.command(q, at_time=0.0)
    assert np.allclose(link.read_joints(), q + 0.5, atol=1e-12)


def test_plant_link_scale_error_is_relative(robot):
    link = PlantLink(robot, PlantModel(scale_error=0.01))
    link.connect()
    q = np.asarray([10.0, -20.0, 30.0, 0.0, 40.0, 0.0])
    link.command(q, at_time=0.0)
    assert np.allclose(link.read_joints(), q * 1.01, atol=1e-12)


def test_plant_link_noise_is_reproducible_with_same_seed(robot):
    model = PlantModel(noise_deg=0.01, seed=42)
    seq = []
    for _ in range(2):
        link = PlantLink(robot, model)
        link.connect()
        seq.append(
            [tuple(link.command(Q0, at_time=float(i)).round(6)) for i in range(4)]
        )
    assert seq[0] == seq[1]


def test_plant_link_different_seed_gives_different_noise(robot):
    outs = []
    for seed in (1, 2):
        link = PlantLink(robot, PlantModel(noise_deg=0.05, seed=seed))
        link.connect()
        outs.append(np.asarray(link.command(Q0, at_time=0.0)))
    assert not np.allclose(outs[0], outs[1])


def test_plant_link_readback_stable_across_repeated_reads(robot):
    link = PlantLink(robot, PlantModel(noise_deg=0.05, seed=3))
    link.connect()
    link.command(Q0, at_time=0.0)
    a = link.read_joints()
    b = link.read_joints()
    assert np.array_equal(a, b)


def test_plant_link_read_joints_and_get_pose_are_consistent(robot):
    link = PlantLink(robot, PlantModel(joint_bias_deg=0.2))
    link.connect()
    link.command(Q0, at_time=0.0)
    q = link.read_joints()
    assert np.allclose(link.get_pose().to_xyz_rpy(), robot.forward_kinematics(q).to_xyz_rpy())


def test_plant_link_first_command_has_no_phantom_lag(robot):
    """首条指令不得从 Home 追过去——否则首个采样点会凭空差出几百毫米。"""
    link = PlantLink(robot, PlantModel(lag_tau=0.05))
    link.connect()
    link.command(Q0, at_time=0.0)
    assert np.allclose(link.read_joints(), Q0, atol=1e-9)


def test_plant_link_replay_from_start_does_not_carry_final_lag(robot):
    """会话回绕重播（时间倒流）时不得沿用上一次终点的滞后状态。

    :meth:`DebugSession.reset` / :func:`capture_from_session` 在采集前都会回到轨迹
    起点，链路复用（或同一台"真机"被复测）时时间戳就往回走。若把 dt 夹到 0，滞后
    状态会停在上一次终点，第一个采样点凭空差出**整条轨迹**的量级——Socket 真机
    通路实测抓出 902mm / 133°，看起来像"机器人坏了"，其实是模型没处理回绕。
    """
    link = PlantLink(robot, PlantModel(joint_bias_deg=0.2, lag_tau=0.05))
    link.connect()
    q = np.zeros(6, dtype=np.float64)
    link.command(q, at_time=0.0)
    for i in range(1, 60):
        q = q + 2.0  # 一路走到 ~118°
        link.command(q, at_time=i * 0.01)
    assert float(np.max(np.abs(link.read_joints() - q))) > 1.0  # 确实走远了
    # 回绕：时间戳回到 0，指令回到起点 —— 只该带传感误差，不该带滞后残留
    link.command(np.zeros(6), at_time=0.0)
    assert np.allclose(link.read_joints(), 0.2, atol=1e-9)


def test_plant_link_lag_steady_state_error_matches_tau_times_omega(robot):
    """匀速运动下一阶滞后的稳态误差应 ≈ τ·ω（物理正确性核对）。"""
    tau, dt, omega = 0.02, 0.002, 100.0  # deg/s
    link = PlantLink(robot, PlantModel(lag_tau=tau))
    link.connect()
    step = omega * dt
    q = np.zeros(6, dtype=np.float64)
    link.command(q, at_time=0.0)
    for i in range(1, 400):
        q = q + step
        link.command(q, at_time=i * dt)
    err = float(np.max(np.abs(link.read_joints() - q)))
    expected = omega * tau
    assert err == pytest.approx(expected, rel=0.05)


def test_plant_link_larger_tau_gives_larger_error(robot):
    errors = []
    for tau in (0.005, 0.05):
        link = PlantLink(robot, PlantModel(lag_tau=tau))
        link.connect()
        q = np.zeros(6, dtype=np.float64)
        link.command(q, at_time=0.0)
        for i in range(1, 150):
            q = q + 0.5
            link.command(q, at_time=i * 0.01)
        errors.append(float(np.max(np.abs(link.read_joints() - q))))
    assert errors[1] > errors[0]


def test_plant_link_without_time_is_immediate(robot):
    """没有时刻信息（手动 jog）时不建模滞后，指令即刻到位。"""
    link = PlantLink(robot, PlantModel(lag_tau=0.5))
    link.connect()
    link.command(Q0, at_time=0.0)
    link.set_joints([5.0, -25.0, 35.0, 5.0, 55.0, 5.0])
    assert np.allclose(link.read_joints(), [5.0, -25.0, 35.0, 5.0, 55.0, 5.0], atol=1e-9)


def test_plant_link_wrong_dof_raises(robot):
    link = PlantLink(robot, PlantModel())
    link.connect()
    with pytest.raises(TwinError, match="关节数"):
        link.command([0.0, 0.0, 0.0], at_time=0.0)


def test_plant_link_requires_connection(robot):
    link = PlantLink(robot, PlantModel())
    with pytest.raises(TwinError, match="未连接"):
        link.command(Q0, at_time=0.0)


def test_plant_link_is_controller_link_subclass(robot):
    link = PlantLink(robot, PlantModel())
    assert isinstance(link, ControllerLink)
    assert link.is_actual_feedback is True
    assert link.protocol == "plant"


def test_plant_link_robot_state_follows_actual_not_commanded(robot):
    """数字孪生视口要显示"真机在哪"，不是"它被要求去哪"。"""
    link = PlantLink(robot, PlantModel(joint_bias_deg=1.0))
    link.connect()
    link.command(Q0, at_time=0.0)
    assert np.allclose(robot.get_joints(), np.asarray(Q0) + 1.0, atol=1e-9)


# =========================================================================== #
# 2. 孪生轨迹：采集
# =========================================================================== #


def test_sim_trace_deviation_is_negligible(sim_trace):
    """理想仿真的实际≈规划：残余量只有 FK/IK 数值精度量级（<0.1mm，与
    ``test_debug`` 同一口径——moveL 的插补点由 IK 反解，FK∘IK 不严格等于插补位姿）。"""
    assert sim_trace.feedback_available
    assert sim_trace.max_pos_dev() < 0.1
    assert sim_trace.max_rot_dev() < 0.1
    assert sim_trace.max_pos_dev() > 0.0


def test_capture_sample_count_matches_waypoints(sim_trace, traj):
    assert len(sim_trace) == len(traj.waypoints)


def test_capture_planned_q_matches_trajectory(sim_trace, traj):
    for snap, wp in zip(sim_trace.snapshots, traj.waypoints):
        assert np.allclose(snap.planned_q, wp.joints)


def test_capture_times_match_trajectory(sim_trace, traj):
    assert np.allclose(sim_trace.times, [w.timestamp for w in traj.waypoints])


def test_capture_empty_trajectory_raises():
    from orolp.simulation import Trajectory

    sess = DebugSession(Trajectory([]), None, robot=None)
    with pytest.raises(TwinError, match="轨迹为空"):
        capture_from_session(sess)


def test_capture_plant_trace_marks_source_and_synthetic_note(traj, robot):
    trace = capture_plant_trace(traj, robot, PlantModel(joint_bias_deg=0.1), program_name="P")
    assert trace.source == "plant"
    assert "合成" in trace.notes
    assert trace.program_name == "P"


def test_capture_plant_trace_has_nonzero_deviation(traj, robot):
    trace = capture_plant_trace(traj, robot, PlantModel(joint_bias_deg=0.3))
    assert trace.max_pos_dev() > 0.5
    assert trace.worst_index() >= 0


def test_capture_with_file_link_has_no_feedback(traj):
    link = FileLink(tempfile.mkdtemp())
    link.connect()
    trace = capture_from_session(DebugSession(traj, link), source="sim")
    assert not trace.feedback_available
    assert math.isnan(trace.max_pos_dev())


def test_trace_stats_keys_and_values(sim_trace):
    st = sim_trace.stats()
    assert st["samples"] == len(sim_trace)
    assert st["feedback_available"] is True
    assert st["duration_s"] > 0
    assert st["max_pos_mm"] < 0.1


def test_trace_worst_index_tracks_largest_deviation():
    trace = _trace_with_speeds()
    assert trace.worst_index() == 3


def test_trace_tolerances_exceeded_lists_over_tol_indices():
    trace = _trace_with_speeds()
    assert trace.tolerances_exceeded(tol_pos_mm=0.9, tol_rot_deg=999) == [2, 3]


def test_trace_joint_speed_series_positive_on_motion():
    trace = _trace_with_speeds()
    speeds = trace.joint_speed_series()
    assert speeds[0] > 0 and speeds[-1] > 0


def test_trace_duration_and_len_on_empty():
    empty = TwinTrace()
    assert len(empty) == 0
    assert empty.duration == 0.0
    assert not empty.feedback_available


# =========================================================================== #
# 3. 序列化往返
# =========================================================================== #


def test_trace_dict_roundtrip(sim_trace):
    back = TwinTrace.from_dict(sim_trace.to_dict())
    assert back.source == sim_trace.source
    assert back.program_name == sim_trace.program_name
    assert len(back) == len(sim_trace)
    assert np.allclose(back.times, sim_trace.times)
    assert np.allclose(back.snapshots[0].planned_q, sim_trace.snapshots[0].planned_q)
    assert np.allclose(
        back.snapshots[-1].planned_pose.to_xyz_rpy(),
        sim_trace.snapshots[-1].planned_pose.to_xyz_rpy(),
    )


def test_trace_jsonl_layout_is_header_plus_samples(sim_trace):
    lines = sim_trace.to_jsonl().strip().splitlines()
    assert len(lines) == len(sim_trace) + 1
    assert json.loads(lines[0])["kind"] == TRACE_HEADER_KIND
    assert all(json.loads(ln)["kind"] == TRACE_SAMPLE_KIND for ln in lines[1:])


def test_trace_jsonl_roundtrip(sim_trace):
    back = TwinTrace.from_jsonl(sim_trace.to_jsonl())
    assert len(back) == len(sim_trace)
    assert back.source == sim_trace.source
    assert np.allclose(back.pos_dev_series(), sim_trace.pos_dev_series(), equal_nan=True)


def test_trace_nan_survives_json_roundtrip():
    """无实测回读时偏差为 nan —— 序列化必须显式转 null 再还原成 nan。"""
    trace = TwinTrace(
        snapshots=[
            TwinSnapshot(
                t=0.0,
                planned_q=np.zeros(6),
                actual_q=None,
                planned_pose=Pose.from_xyz_rpy(0, 0, 0, 0, 0, 0),
                pos_dev_mm=float("nan"),
                rot_dev_deg=float("nan"),
                joint_dev_deg=float("nan"),
            )
        ]
    )
    assert math.isnan(trace.snapshots[0].pos_dev_mm)
    back = TwinTrace.from_jsonl(trace.to_jsonl())
    assert math.isnan(back.snapshots[0].pos_dev_mm)
    assert back.snapshots[0].actual_q is None
    assert not back.feedback_available


def test_trace_jsonl_is_strict_json(sim_trace):
    """逐行必须是严格合法 JSON（NaN/Infinity 不是）。"""
    for line in sim_trace.to_jsonl().splitlines():
        json.loads(line, parse_constant=_reject_constant)


def test_trace_save_load_jsonl(tmp_path, sim_trace):
    p = tmp_path / "t.jsonl"
    sim_trace.save(p)
    back = TwinTrace.load(p)
    assert len(back) == len(sim_trace)


def test_trace_save_load_json(tmp_path, sim_trace):
    p = tmp_path / "t.json"
    sim_trace.save(p)
    back = TwinTrace.load(p)
    assert len(back) == len(sim_trace)


def test_trace_unsupported_format_raises(tmp_path, sim_trace):
    with pytest.raises(TwinError, match="不支持"):
        sim_trace.save(tmp_path / "t.txt", fmt="yaml")


def test_trace_from_jsonl_missing_header_raises():
    with pytest.raises(TwinError, match="元数据"):
        TwinTrace.from_jsonl('{"kind": "sample", "t": 0.0, "planned_q": []}\n')


def test_trace_from_jsonl_bad_kind_raises():
    body = json.dumps({"kind": TRACE_HEADER_KIND}) + "\n" + '{"kind": "nope"}\n'
    with pytest.raises(TwinError, match="kind"):
        TwinTrace.from_jsonl(body)


def test_trace_from_jsonl_broken_json_raises():
    body = json.dumps({"kind": TRACE_HEADER_KIND}) + "\n{not json}\n"
    with pytest.raises(TwinError, match="解析失败"):
        TwinTrace.from_jsonl(body)


def test_trace_from_dict_missing_snapshots_raises():
    with pytest.raises(TwinError, match="snapshots"):
        TwinTrace.from_dict({"source": "sim"})


def test_trace_shifted_moves_times(sim_trace):
    shifted = sim_trace.shifted(1.5)
    assert np.allclose(shifted.times, sim_trace.times + 1.5)
    assert shifted.source == sim_trace.source


# =========================================================================== #
# 4. 对齐与差分
# =========================================================================== #


def test_align_same_timeline_matches_everything(sim_trace):
    pairs = align_traces(sim_trace, sim_trace, max_dt=0.05)
    assert len(pairs) == len(sim_trace)
    assert all(dt == 0.0 for _, _, dt in pairs)


def test_align_outside_window_drops_samples(sim_trace):
    far = sim_trace.shifted(10.0)
    assert align_traces(sim_trace, far, max_dt=0.05) == []


def test_align_within_window_keeps_shifted_samples(sim_trace):
    near = sim_trace.shifted(0.02)
    assert len(align_traces(sim_trace, near, max_dt=0.05)) == len(sim_trace)


def test_align_requires_positive_window(sim_trace):
    with pytest.raises(TwinError, match="max_dt"):
        align_traces(sim_trace, sim_trace, max_dt=0.0)


def test_align_empty_trace_returns_empty(sim_trace):
    assert align_traces(TwinTrace(), sim_trace) == []


def test_compare_identical_traces_passes(sim_trace):
    rep = compare_traces(sim_trace, sim_trace, tol_pos_mm=0.05, tol_rot_deg=0.01)
    assert rep.ok
    assert rep.max_pos_mm < 0.05


def test_compare_plant_trace_shows_deviation(traj, robot):
    sim = capture_from_session(
        DebugSession(traj, _sim_link(robot), robot=robot), source="sim"
    )
    real = capture_plant_trace(traj, robot, PlantModel(joint_bias_deg=0.3), program_name="P")
    rep = compare_traces(sim, real, tol_pos_mm=0.5, tol_rot_deg=1.0)
    assert rep.available
    assert rep.max_pos_mm > 1.0
    assert rep.over_tol and not rep.ok
    assert rep.worst_sample is not None


def test_compare_records_unmatched_count(sim_trace):
    far = sim_trace.shifted(0.03)
    rep = compare_traces(sim_trace, far, max_dt=0.02)
    assert rep.unmatched > 0


def test_compare_report_to_dict_is_strict_json(traj, robot):
    sim = capture_from_session(DebugSession(traj, _sim_link(robot), robot=robot), source="sim")
    real = capture_plant_trace(traj, robot, PlantModel(joint_bias_deg=0.2))
    rep = compare_traces(sim, real, tol_pos_mm=0.5)
    json.loads(json.dumps(rep.to_dict()), parse_constant=_reject_constant)


def test_compare_speed_split_separates_calibration_from_servo():
    """构造：低速段 0.5mm、高速段 2.0mm → 伺服分量占比 ≈ 75%。"""
    ref = _trace_with_speeds()
    meas = TwinTrace(snapshots=[], source="sim")
    rep = TwinReport(
        samples=[
            AlignedSample(
                t=s.t,
                ref_t=s.t,
                meas_t=s.t,
                pos_dev_mm=s.pos_dev_mm,
                rot_dev_deg=s.rot_dev_deg,
                joint_dev_deg=0.0,
                joint_speed_deg_s=sp,
            )
            for s, sp in zip(ref.snapshots, [1.0, 1.0, 200.0, 200.0])
        ],
        ref_source="sim",
        meas_source="plant",
        tol_pos_mm=1.0,
    )
    split = rep.speed_split(slow_below_deg_s=5.0)
    assert split["n_slow"] == 2 and split["n_fast"] == 2
    assert split["slow_max_mm"] == pytest.approx(0.5)
    assert split["fast_max_mm"] == pytest.approx(2.0)
    assert split["servo_share"] == pytest.approx(0.75)


def test_speed_split_on_empty_report_is_nan():
    split = TwinReport().speed_split()
    assert math.isnan(split["slow_max_mm"]) and split["n_slow"] == 0


def test_report_format_contains_key_lines(traj, robot):
    sim = capture_from_session(DebugSession(traj, _sim_link(robot), robot=robot), source="sim")
    real = capture_plant_trace(traj, robot, PlantModel(joint_bias_deg=0.3))
    text = compare_traces(sim, real, tol_pos_mm=1.0).format()
    for token in ("数字孪生偏差报告", "位置偏差", "姿态偏差", "速度分解", "结论"):
        assert token in text


def test_report_format_on_empty_report():
    assert "无可用样本" in TwinReport().format()


def test_format_causes_prefers_servo_when_fast_dominates():
    rep = TwinReport(
        samples=[
            AlignedSample(0.0, 0.0, 0.0, 0.5, 0.1, 0.1, False, 1.0),
            AlignedSample(1.0, 1.0, 1.0, 4.0, 0.3, 0.5, True, 200.0),
        ]
    )
    text = rep.format_causes()
    assert "伺服跟踪误差" in text
    assert "别据此去重新标定" in text


def test_format_causes_flags_calibration_when_slow_already_over():
    rep = TwinReport(
        samples=[
            AlignedSample(0.0, 0.0, 0.0, 1.5, 0.1, 0.2, True, 1.0),
            AlignedSample(1.0, 1.0, 1.0, 1.6, 0.1, 0.2, True, 200.0),
        ]
    )
    assert "标定问题" in rep.format_causes()


def test_format_causes_on_empty_report():
    assert "无可用样本" in TwinReport().format_causes()


def test_report_verdict_text_variants():
    assert "通过" in TwinReport(
        samples=[AlignedSample(0.0, 0.0, 0.0, 0.1, 0.1, 0.0, False, 1.0)]
    ).verdict()
    assert "超差" in TwinReport(
        samples=[AlignedSample(0.0, 0.0, 0.0, 9.9, 0.1, 0.0, True, 1.0)]
    ).verdict()
    assert "无可用" in TwinReport().verdict()


def test_report_stats_on_empty_are_nan():
    rep = TwinReport()
    assert math.isnan(rep.max_pos_mm) and math.isnan(rep.rms_pos_mm)
    assert rep.worst_sample is None


def test_pose_deviation_zero_for_same_pose():
    """同一个位姿偏差必须是零（姿态经 RPY→矩阵→夹角往返，容差按数值精度给）。"""
    p = Pose.from_xyz_rpy(100, 200, 300, 10, 20, 30)
    pos, rot = pose_deviation(p, p)
    assert pos == pytest.approx(0.0, abs=1e-9)
    assert rot < 1e-3


def test_pose_deviation_detects_pure_translation():
    p0 = Pose.from_xyz_rpy(0, 0, 0, 0, 0, 0)
    p1 = Pose.from_xyz_rpy(3, 4, 0, 0, 0, 0)
    pos, rot = pose_deviation(p0, p1)
    assert pos == pytest.approx(5.0)
    assert rot == pytest.approx(0.0, abs=1e-9)


# =========================================================================== #
# 5. SVG 可视化
# =========================================================================== #


def test_chart_svg_wellformed(traj, robot):
    sim = capture_from_session(DebugSession(traj, _sim_link(robot), robot=robot), source="sim")
    real = capture_plant_trace(traj, robot, PlantModel(joint_bias_deg=0.3))
    svg = deviation_chart_svg(compare_traces(sim, real, tol_pos_mm=1.0))
    assert svg.startswith("<svg") and svg.endswith("</svg>")
    assert "<polyline" in svg and "容差" in svg


def test_chart_svg_empty_report_renders_message():
    svg = deviation_chart_svg(TwinReport())
    assert "无可用比对样本" in svg and svg.endswith("</svg>")


def test_chart_svg_unknown_theme_raises():
    with pytest.raises(TwinError, match="主题"):
        deviation_chart_svg(TwinReport(), theme="neon")


def test_chart_svg_too_small_raises():
    with pytest.raises(TwinError, match="画布"):
        deviation_chart_svg(TwinReport(), width=100, height=50)


def test_chart_svg_theme_changes_colors():
    rep = TwinReport(
        samples=[AlignedSample(0.0, 0.0, 0.0, 2.0, 0.1, 0.0, True, 1.0)]
    )
    dark = deviation_chart_svg(rep, theme="dark")
    light = deviation_chart_svg(rep, theme="light")
    assert dark != light
    assert "#161a20" in dark and "#ffffff" in light


def test_chart_svg_marks_over_tolerance_points():
    rep = TwinReport(
        samples=[
            AlignedSample(0.0, 0.0, 0.0, 0.1, 0.0, 0.0, False, 1.0),
            AlignedSample(1.0, 1.0, 1.0, 5.0, 0.0, 0.0, True, 1.0),
        ]
    )
    svg = deviation_chart_svg(rep, theme="dark")
    assert "<circle" in svg
    assert "峰值" in svg


def test_chart_svg_escapes_title():
    rep = TwinReport(samples=[AlignedSample(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, True, 1.0)])
    svg = deviation_chart_svg(rep, title="a <b> & c")
    assert "&lt;b&gt;" in svg and "&amp;" in svg


# =========================================================================== #
# 6. 云协作通道
# =========================================================================== #


def test_normalize_key_strips_slashes_and_rejects_traversal():
    assert normalize_key("/a/b/") == "a/b"
    assert normalize_key("a//b") == "a/b"
    with pytest.raises(TwinError, match="不能为空"):
        normalize_key("   ")
    with pytest.raises(TwinError, match=r"\.\."):
        normalize_key("a/../../etc/passwd")


def test_memory_store_basic_roundtrip():
    store = MemoryCloudStore()
    handle = store.put("x/y.txt", b"hello")
    assert handle == "cloud://x/y.txt"
    assert store.get("x/y.txt") == b"hello"
    assert store.exists("x/y.txt")
    assert store.keys("x/") == ["x/y.txt"]
    assert store.delete("x/y.txt") is True
    assert store.get("x/y.txt") is None
    assert store.delete("x/y.txt") is False


def test_local_store_roundtrip(tmp_path):
    store = LocalCloudStore(tmp_path / "cloud")
    store.put("ns/a.txt", b"1")
    store.put("ns/sub/b.txt", b"2")
    assert store.keys("ns/") == ["ns/a.txt", "ns/sub/b.txt"]
    assert store.get("ns/sub/b.txt") == b"2"
    assert store.get("ns/missing.txt") is None


def test_local_store_blocks_path_traversal(tmp_path):
    root = tmp_path / "cloud"
    store = LocalCloudStore(root)
    with pytest.raises(TwinError, match=r"\.\."):
        store.put("../escape.txt", b"x")
    assert not (tmp_path / "escape.txt").exists()


def test_cloud_link_requires_connection(tmp_path):
    link = CloudLink(LocalCloudStore(tmp_path))
    with pytest.raises(TwinError, match="未连接"):
        link.list_programs()


def test_cloud_link_needs_transport():
    with pytest.raises(TwinError, match="传输实现"):
        CloudLink(None)


def test_cloud_link_is_controller_link_with_cloud_support(tmp_path):
    link = CloudLink(MemoryCloudStore())
    assert isinstance(link, ControllerLink)
    assert link.supports_cloud is True
    assert ControllerLink.supports_cloud is False


def test_controller_link_default_cloud_hooks_return_none():
    link = FileLink(tempfile.mkdtemp())
    link.connect()
    assert link.publish_trace(object()) is None
    assert link.fetch_trace("nope") is None


def test_cloud_link_upload_program_and_manifest(tmp_path):
    link = CloudLink(LocalCloudStore(tmp_path), namespace="cell1")
    link.connect()
    handle = link.upload_program("GRASP", "movej()\n", {"ext": "urp", "brand": "ur"})
    assert handle == "cloud://cell1/programs/GRASP.urp"
    assert link.list_programs() == ["GRASP"]
    assert link.fetch_program("GRASP", "urp") == "movej()\n"
    assert link.program_manifest("GRASP")["bytes"] == len("movej()\n".encode())


def test_cloud_link_fetch_missing_program_returns_none(tmp_path):
    link = CloudLink(MemoryCloudStore())
    link.connect()
    assert link.fetch_program("nope") is None
    assert link.program_manifest("nope") is None


def test_cloud_link_trace_roundtrip(sim_trace):
    link = CloudLink(MemoryCloudStore(), namespace="ns")
    link.connect()
    handle = link.publish_trace(sim_trace)
    assert handle == "cloud://ns/traces/TWIN_DEMO.jsonl"
    assert link.list_traces() == ["TWIN_DEMO"]
    back = link.fetch_trace("TWIN_DEMO")
    assert len(back) == len(sim_trace)
    assert back.source == sim_trace.source
    assert np.allclose(back.pos_dev_series(), sim_trace.pos_dev_series(), equal_nan=True)


def test_cloud_link_trace_meta_is_strict_json(traj, robot):
    link = CloudLink(MemoryCloudStore())
    link.connect()
    real = capture_plant_trace(traj, robot, PlantModel(joint_bias_deg=0.2), program_name="P")
    link.publish_trace(real)
    raw = link.transport.get("robotworld/traces/P.meta.json").decode()
    meta = json.loads(raw, parse_constant=_reject_constant)
    assert meta["samples"] == len(real)
    assert meta["source"] == "plant"


def test_cloud_link_trace_meta_null_max_pos_without_feedback(traj):
    link = CloudLink(MemoryCloudStore())
    link.connect()
    file_link = FileLink(tempfile.mkdtemp())
    file_link.connect()
    trace = capture_from_session(DebugSession(traj, file_link), source="sim")
    link.publish_trace(trace, name="nofb")
    meta = link.trace_meta("nofb")
    assert meta["max_pos_mm"] is None


def test_cloud_link_fetch_missing_trace_raises():
    link = CloudLink(MemoryCloudStore())
    link.connect()
    with pytest.raises(TwinError, match="没有轨迹"):
        link.fetch_trace("ghost")


def test_cloud_link_status_lifecycle(tmp_path):
    link = CloudLink(LocalCloudStore(tmp_path))
    link.connect()
    assert link.status() is None
    assert link.status_age() is None
    assert link.is_fresh() is False
    assert link.is_actual_feedback is False
    assert link.read_joints() is None

    link.push_status([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    assert np.allclose(link.read_joints(), [1, 2, 3, 4, 5, 6])
    assert link.is_actual_feedback is True


def test_cloud_link_status_age_and_freshness_are_checkable(tmp_path):
    link = CloudLink(LocalCloudStore(tmp_path))
    link.connect()
    base = 1_000_000.0
    link.push_status([0] * 6, t=base)
    assert link.status_age(now=base + 2.0) == pytest.approx(2.0)
    assert link.is_fresh(max_age=5.0, now=base + 2.0) is True
    assert link.is_fresh(max_age=5.0, now=base + 30.0) is False


def test_cloud_link_get_pose_requires_robot(tmp_path, robot, traj):
    link = CloudLink(LocalCloudStore(tmp_path))
    link.connect()
    link.push_status(Q0)
    assert link.get_pose() is None
    with_robot = CloudLink(LocalCloudStore(tmp_path), robot=robot)
    with_robot.connect()
    pose = with_robot.get_pose()
    assert pose is not None
    assert np.allclose(
        pose.to_xyz_rpy(), robot.forward_kinematics(np.asarray(Q0)).to_xyz_rpy()
    )


def test_cloud_link_set_joints_is_noop(tmp_path):
    link = CloudLink(LocalCloudStore(tmp_path))
    link.connect()
    assert link.set_joints([1, 2, 3, 4, 5, 6]) is None


def test_cloud_link_summary(tmp_path, sim_trace):
    link = CloudLink(LocalCloudStore(tmp_path), namespace="lineA")
    link.connect()
    link.upload_program("P", "x", {"ext": "src"})
    link.publish_trace(sim_trace)
    s = link.summary()
    assert s["namespace"] == "lineA"
    assert s["programs"] == ["P"] and s["traces"] == ["TWIN_DEMO"]
    assert s["has_status"] is False


def test_cloud_link_sanitizes_unsafe_names(tmp_path):
    link = CloudLink(LocalCloudStore(tmp_path))
    link.connect()
    handle = link.upload_program("../../evil name", "x", {"ext": "src"})
    assert ".." not in handle
    assert len(link.list_programs()) == 1


# =========================================================================== #
# 7. DebugSession 时间透传（回归守卫）
# =========================================================================== #


def test_debug_session_still_works_with_untimed_link(sim_trace):
    """老连接不接受 at_time，行为必须与以前完全一致。"""
    assert sim_trace.feedback_available


def test_debug_session_passes_time_to_timed_link(traj, robot):
    """PlantLink 通过既有 DebugSession 通路就能产出非零偏差。"""
    link = PlantLink(robot, PlantModel(joint_bias_deg=0.5, lag_tau=0.02))
    link.connect()
    sess = DebugSession(traj, link, robot=robot)
    sess.goto_waypoint(1)
    dev = sess.deviation()
    assert dev.available and dev.pos_mm > 0.0


def test_debug_session_jog_works_on_plant_link(traj, robot):
    link = PlantLink(robot, PlantModel(lag_tau=0.1))
    link.connect()
    sess = DebugSession(traj, link, robot=robot)
    sess.goto_waypoint(1)
    before = link.read_joints()
    after = sess.jog(0, 1.0)
    assert after is not None
    assert not np.allclose(before, after)


def _sim_link(robot):
    link = SimLink(robot)
    link.connect()
    return link


def _reject_constant(name):  # pragma: no cover - 仅在违规时触发
    raise AssertionError(f"JSON 里出现非法常量 {name}（NaN/Infinity 不是合法 JSON）")
