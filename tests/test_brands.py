"""多品牌 OLP 覆盖测试（"补充其它品牌"在离线编程层面的落地）。

目标：证明离线编程管线对 7 个已注册品牌都可用——

* 6 个有程序化 URDF 的品牌：加载 → 建程序（moveJ/moveL/moveSpline）→ 展开/校验
  → 后处理出码，全链路跑通；
* 汇川（huichuan）目前只有后处理器、无程序化 URDF：单测其后处理出码与品牌解析，
  作为"该品牌 OLP 出口存在"的回归防线。

这同时也为"把各品牌真机 CAD 接进 URDF"兜底：运动学来源（程序化 URDF）齐了，
STEP 一到位即可用 tools/build_vendor_urdf.py 一键生成真机网格。
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

from service.program import ProgramBuilder, Program  # noqa: E402
from domain.pose import Pose
from domain.robot import Robot  # noqa: E402
from adapter.post import list_post_processors, resolve_for_brand  # noqa: E402

REPO = Path(__file__).resolve().parent.parent

# (品牌, 代表性 URDF 相对 assets/models/ 的路径, 后处理器 full_name)
BRAND_MODELS = [
    ("ur", "ur/ur5e/ur5e.urdf", "ur_polyscope"),
    ("kuka", "kuka/kr16_2/kr16_2.urdf", "kuka_krc4"),
    ("abb", "abb/irb1200/irb1200.urdf", "abb_irc5"),
    ("fanuc", "fanuc/m20ia_35m/m20ia_35m.urdf", "fanuc_r30ib"),
    ("estun", "estun/er20_1780/er20_1780.urdf", "estun_er"),
    ("jaka", "jaka/zu3/zu3.urdf", "jaka_zu"),
]

# 每个品牌一个示教位姿（mm + 度），只用于走通管线，不求物理合理
BRAND_TARGETS = {
    "ur": (400, 0, 300, 0, 0, 0),
    "kuka": (400, 0, 300, 0, 0, 0),
    "abb": (400, 0, 300, 0, 0, 0),
    "fanuc": (400, 0, 300, 0, 0, 0),
    "estun": (400, 0, 300, 0, 0, 0),
    "jaka": (300, 0, 200, 0, 0, 0),
}


@pytest.mark.parametrize("brand,rel,pp", BRAND_MODELS)
def test_brand_full_olp_pipeline(brand, rel, pp):
    urdf = REPO / "assets" / "models" / rel
    assert urdf.exists(), f"{brand} 缺程序化 URDF：{urdf}"
    robot = Robot(str(urdf), brand)
    assert robot.dof >= 4

    q0 = [0.0] * robot.dof
    target = BRAND_TARGETS[brand]
    prog = (
        ProgramBuilder(robot)
        .moveJ(q0, speed=40)
        .moveL(Pose.from_xyz_rpy(*target), speed=50, blend=8)
        .moveSpline([10.0] + [0.0] * (robot.dof - 1), speed=50)  # 只动首轴，确保可达
        .build()
    )
    # 展开 + 校验
    traj = prog.simulate()
    assert traj.num_waypoints() > 2
    rep = prog.validate()
    assert rep.cycle_time is not None and rep.cycle_time > 0
    # 后处理：按 full_name 与按 brand 解析都应出码
    code_by_full = prog.to_post(pp)
    code_by_brand = prog.to_post(brand)
    assert code_by_full.strip() and code_by_brand.strip()
    # 部署到 FileLink（离线导出）
    from domain.link import FileLink
    import tempfile

    fl = FileLink(tempfile.mkdtemp())
    fl.connect()
    dep = prog.deploy(fl, pp, name=f"{brand}_demo")
    assert dep.ok and dep.brand == brand


def test_all_seven_post_processors_registered():
    names = set(list_post_processors())
    expected = {
        "ur_polyscope",
        "kuka_krc4",
        "abb_irc5",
        "fanuc_r30ib",
        "estun_er",
        "jaka_zu",
        "huichuan_inoteach",
    }
    assert expected <= names, f"缺失后处理器：{expected - names}"


def test_huichuan_post_processor_works_without_model():
    """汇川暂无程序化 URDF：至少守住其后处理出口存在且能出码。"""
    assert resolve_for_brand("huichuan") is not None
    pp = ProgramBuilder  # 仅用于触发导入；下面直接构造 Motion 列表
    from service.program import Motion, MotionType

    robot = Robot(str(REPO / "assets" / "models" / "ur" / "ur5e" / "ur5e.urdf"), "proxy")
    motions = [
        Motion(MotionType.JOINTS, [0, -30, 30, 0, 60, 0], speed=40),
        Motion(MotionType.LINEAR, Pose.from_xyz_rpy(400, 0, 300, 0, 0, 0), speed=50),
    ]
    code = Program(motions=motions, robot=robot).to_post("huichuan_inoteach")
    assert code.strip(), "汇川后处理器未产出代码"
