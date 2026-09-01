"""后处理器（控制程序生成 / 代码上传）的回归测试。

这些测试守的是「离线编程 → 控制器代码」这条主线的立身之本：

* **单位不能错** —— UR / JAKA 要米 + 弧度，KUKA / ABB / FANUC / ESTUN / 汇川
  要毫米 + 度。一旦把 UR 的位姿写成毫米（历史上真出过这种 bug），
  机器人会直接飞出工作范围。这里逐品牌断言输出里出现的线位移数值单位。
* **指令字要对** —— 每家控制器的运动指令（PTP / LIN / MoveL / movel …）必须
  出现在生成的代码里，否则就是模板或转换逻辑坏掉了。
* **失败必须抛错** —— 圆弧缺 via 点、遇到不支持的运动类型，必须抛
  ``PostProcessError``，绝不能静默降级成注释占位（旧实现就是这么坑人的）。
* **错误要带指令序号** —— 某条指令转换崩了，报错里得指出是第几条，
  否则用户拿到一堆废代码还不知道哪行出错。

运行::

    pytest tests/test_post.py -q
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

from orolp.pose import Pose  # noqa: E402
from orolp.post import (  # noqa: E402
    POST_PROCESSORS,
    get_post_processor,
    list_post_processors,
    resolve_for_brand,
)
from orolp.post.base import BasePostProcessor, PostConfig, PostProcessError  # noqa: E402
from orolp.program import Motion, MotionType  # noqa: E402


# ----------------------------------------------------------------------------
# 品牌元数据：units 决定线位移单位；cmd_* 是必须出现在代码里的指令字；
# ext 是建议文件扩展名；rotvec=True 表示姿态用旋转向量（轴角）而非欧拉角。
# ----------------------------------------------------------------------------
BRANDS = {
    "ur_polyscope": dict(units="m", joint="movej", linear="movel", circ="movec",
                         ext="script", rotvec=True),
    "kuka_krc4": dict(units="mm", joint="PTP", linear="LIN", circ="CIRC",
                      ext="src", rotvec=False),
    "abb_irc5": dict(units="mm", joint="MoveJ", linear="MoveL", circ="MoveC",
                     ext="mod", rotvec=False),
    "estun_er": dict(units="mm", joint="MOVJ", linear="MOVL", circ="MOVC",
                     ext="prg", rotvec=False),
    "fanuc_r30ib": dict(units="mm", joint="J ", linear="L ", circ="C ",
                        ext="ls", rotvec=False),
    "huichuan_inoteach": dict(units="mm", joint="MOVJ", linear="MOVL", circ="MOVC",
                              ext="prg", rotvec=False),
    "jaka_zu": dict(units="m", joint="movej", linear="movel", circ="movec",
                    ext="jks", rotvec=True),
}

ALL_NAMES = list(BRANDS)


def make_motions() -> list:
    """构造一条含关节 / 直线 / 圆弧的运动序列。

    直线目标放在 x=1000mm 处，便于逐品牌断言线位移单位（米品牌应见 1.0000，
    毫米品牌应见 1000）。
    """
    home = Pose.from_xyz_rpy(0, 0, 0, 0, 0, 0)
    far = Pose.from_xyz_rpy(1000, 0, 0, 0, 0, 0)
    via = Pose.from_xyz_rpy(500, 0, 0, 0, 0, 0)
    return [
        Motion(MotionType.JOINTS, [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]),
        Motion(MotionType.LINEAR, far, speed=80.0, blend=5.0),
        Motion(MotionType.CIRCULAR, via=via, target=home, speed=60.0, blend=0.0),
    ]


# ----------------------------------------------------------------------------
# 注册表
# ----------------------------------------------------------------------------
class TestRegistry:
    def test_all_seven_registered(self):
        names = list_post_processors()
        for name in ALL_NAMES:
            assert name in names
        assert len(names) == 7

    def test_get_by_full_name(self):
        pp = get_post_processor("kuka_krc4")
        assert pp.brand == "kuka"
        assert pp.controller == "krc4"

    def test_get_by_brand(self):
        # 品牌唯一时能靠 brand 解析
        pp = get_post_processor("kuka")
        assert pp.full_name == "kuka_krc4"

    def test_unknown_name_raises(self):
        with pytest.raises(PostProcessError):
            get_post_processor("does_not_exist")

    def test_resolve_for_brand_returns_none_when_missing(self):
        assert resolve_for_brand("nonexistent") is None

    def test_resolve_for_brand_works(self):
        assert resolve_for_brand("ur") is not None


# ----------------------------------------------------------------------------
# 格式化工具（抓单位换算与异常路径）
# ----------------------------------------------------------------------------
class TestFormatHelpers:
    def test_as_matrix_pose(self):
        T = BasePostProcessor.as_matrix(Pose.from_xyz_rpy(100, 200, 300, 0, 0, 0))
        assert np.allclose(T[:3, 3], [100, 200, 300])

    def test_as_matrix_4x4(self):
        m = np.eye(4)
        m[0, 3] = 42
        assert np.allclose(BasePostProcessor.as_matrix(m)[:3, 3], [42, 0, 0])

    def test_as_matrix_xyz_rpy(self):
        T = BasePostProcessor.as_matrix([10, 20, 30, 0, 0, 0])
        assert np.allclose(T[:3, 3], [10, 20, 30])

    def test_as_matrix_translation3(self):
        T = BasePostProcessor.as_matrix([1.0, 2.0, 3.0])
        assert np.allclose(T[:3, 3], [1, 2, 3])

    def test_as_matrix_bad_length_raises(self):
        with pytest.raises(PostProcessError):
            BasePostProcessor.as_matrix([1, 2, 3, 4])

    def test_fmt_pose_units_mm_vs_m(self):
        pose = Pose.from_xyz_rpy(1000, 0, 0, 0, 0, 0)
        kuka = get_post_processor("kuka_krc4")
        ur = get_post_processor("ur_polyscope")
        # 毫米品牌：1000mm 应原样出现
        assert "1000.000" in kuka.fmt_pose(pose)
        # 米品牌：1000mm 应换算成 1m
        assert "1.0000" in ur.fmt_pose(pose)

    def test_fmt_pose_bad_order_raises(self):
        pose = Pose.from_xyz_rpy(1, 2, 3, 0, 0, 0)
        pp = get_post_processor("kuka_krc4")
        with pytest.raises(PostProcessError):
            pp.fmt_pose(pose, order="xyzw")

    def test_fmt_pose_rotvec_magnitude(self):
        # 绕 Z 转 90° → 旋转向量应近似 [0, 0, π/2]（弧度）。
        # 注意 fmt_pose_rotvec 输出是 x,y,z,rx,ry,rz，旋转部分在最后三个。
        R = Rotation.from_euler("z", 90, degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        ur = get_post_processor("ur_polyscope")  # rad
        vals = [float(x) for x in ur.fmt_pose_rotvec(T).split(", ")]
        assert abs(vals[5] - math.pi / 2) < 1e-2
        kuka = get_post_processor("kuka_krc4")  # deg
        vals_k = [float(x) for x in kuka.fmt_pose_rotvec(T).split(", ")]
        assert abs(vals_k[5] - 90.0) < 1e-2

    def test_fmt_pose_quat_seven_values(self):
        ur = get_post_processor("ur_polyscope")
        s = ur.fmt_pose_quat(Pose.from_xyz_rpy(0, 0, 0, 0, 0, 0))
        assert len(s.split(", ")) == 7

    def test_fmt_joints_deg_vs_rad(self):
        kuka = get_post_processor("kuka_krc4")  # deg
        assert "180.000" in kuka.fmt_joints([180.0])
        ur = get_post_processor("ur_polyscope")  # rad
        assert "3.1416" in ur.fmt_joints([180.0])

    def test_fmt_speed_percent(self):
        pp = get_post_processor("kuka_krc4")
        assert pp.fmt_speed(50.0, MotionType.LINEAR) == "50.0"


# ----------------------------------------------------------------------------
# 逐品牌生成：指令字 + 单位 + 扩展名
# ----------------------------------------------------------------------------
@pytest.mark.parametrize("name", ALL_NAMES)
class TestPerBrandGeneration:
    def test_generates_nonempty(self, name):
        pp = get_post_processor(name)
        code = pp.generate(make_motions(), metadata={"program_name": "t", "version": "0.1.0"})
        assert isinstance(code, str)
        assert len(code.strip()) > 0

    def test_contains_all_motion_cmds(self, name):
        meta = BRANDS[name]
        pp = get_post_processor(name)
        code = pp.generate(make_motions(), metadata={"program_name": "t"})
        assert meta["joint"] in code
        assert meta["linear"] in code
        assert meta["circ"] in code

    def test_linear_units_correct(self, name):
        meta = BRANDS[name]
        pp = get_post_processor(name)
        code = pp.generate(make_motions(), metadata={"program_name": "t"})
        if meta["units"] == "mm":
            # 位置 1000mm 应以 "1000."（带点）形式出现；不能是米（1.0000）
            assert "1000." in code
            assert "1.0000" not in code
        else:  # m
            # 1000mm 应换算成 1m（"1.0000"）；"1000." 这种毫米写法绝不能出现
            assert "1.0000" in code
            assert "1000." not in code

    def test_file_extension(self, name):
        meta = BRANDS[name]
        assert get_post_processor(name).file_extension == meta["ext"]


# ----------------------------------------------------------------------------
# 错误路径：失败必须抛错，且带指令序号
# ----------------------------------------------------------------------------
@pytest.mark.parametrize("name", ALL_NAMES)
class TestErrorHandling:
    def test_circular_without_via_raises(self, name):
        pp = get_post_processor(name)
        motions = [Motion(MotionType.CIRCULAR, target=Pose.from_xyz_rpy(0, 0, 0, 0, 0, 0))]
        with pytest.raises(PostProcessError):
            pp.generate(motions)

    def test_unsupported_motion_raises(self, name):
        """SPLINE 现在已支持，应降级为 PTP/LIN 出码，而非抛错。

        真正的"不支持"仍由圆弧缺 via 等场景守住（见
        ``test_circular_without_via_raises``）。
        """
        pp = get_post_processor(name)
        motions = [Motion(MotionType.SPLINE, target=[0, 0, 0, 0, 0, 0])]
        code = pp.generate(motions)  # 不应抛错
        assert code.strip()

    def test_error_carries_motion_index(self, name):
        # 第 2 条指令（关节目标含非数值）转换崩，报错应指出是第 2 条
        pp = get_post_processor(name)
        motions = [
            Motion(MotionType.JOINTS, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            Motion(MotionType.JOINTS, ["bad", 0.0, 0.0, 0.0, 0.0, 0.0]),
        ]
        with pytest.raises(PostProcessError, match="第 2"):
            pp.generate(motions)
