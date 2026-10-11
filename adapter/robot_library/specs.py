"""机器人型号规格数据库 (Robot specification database).

本模块是 RoboWorld 机器人库的数据源：每个型号用一份轻量的参数描述来定义，
再由 :mod:`urdf_builder` 程序化生成 URDF。

设计要点
--------
1. **只描述真实可查的规格**（负载、臂展、重复定位精度、各轴限位与速度），
   几何外形由 :mod:`urdf_builder` 按臂展/负载推导，因此新增一个型号只需十几行。
2. **参数可信度分级**：``verified=True`` 表示数值来自厂商官方数据表或
   ros-industrial 官方 URDF；``False`` 表示限位等取行业典型值，仅用于建模演示。
3. **不内置任何厂商网格文件**：几何全部由基本体（box/cylinder/sphere）程序化生成，
   规避 CAD 模型的再分发许可问题。

单位约定
--------
* 长度：毫米 (mm)，URDF 生成时转换为米
* 角度：度 (°)，URDF 生成时转换为弧度
* 速度：度每秒 (°/s)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

__all__ = [
    "BrandSpec",
    "RobotSpec",
    "BRANDS",
    "ROBOTS",
    "iter_robots",
    "get_robot",
    "get_brand",
    "robot_key",
]


# --------------------------------------------------------------------------- #
# 品牌定义
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class BrandSpec:
    """机器人品牌（厂商）定义。"""

    id: str
    name: str  # 中文名
    name_en: str  # 英文名
    country: str
    primary: Tuple[float, float, float]  # 主体配色 RGB (0~1)
    accent: Tuple[float, float, float]  # 关节/点缀配色 RGB (0~1)
    controller: str  # 典型控制器


BRANDS: Dict[str, BrandSpec] = {
    "kuka": BrandSpec(
        id="kuka",
        name="库卡",
        name_en="KUKA",
        country="德国",
        # KUKA 标志性橙 #FF6A13
        primary=(1.0, 0.416, 0.075),
        accent=(0.16, 0.16, 0.19),
        controller="KRC4 / KRC5",
    ),
    "jaka": BrandSpec(
        id="jaka",
        name="节卡",
        name_en="JAKA",
        country="中国",
        # 节卡白灰配色
        primary=(0.86, 0.87, 0.89),
        accent=(0.20, 0.55, 0.85),
        controller="JAKA Controller",
    ),
    "abb": BrandSpec(
        id="abb",
        name="ABB",
        name_en="ABB",
        country="瑞士/瑞典",
        primary=(0.80, 0.11, 0.14),  # ABB 红
        accent=(0.20, 0.20, 0.24),
        controller="IRC5",
    ),
    "fanuc": BrandSpec(
        id="fanuc",
        name="发那科",
        name_en="FANUC",
        country="日本",
        primary=(0.99, 0.80, 0.05),  # FANUC 黄
        accent=(0.18, 0.18, 0.22),
        controller="R-30iB",
    ),
    "ur": BrandSpec(
        id="ur",
        name="优傲",
        name_en="Universal Robots",
        country="丹麦",
        primary=(0.14, 0.30, 0.49),  # UR 深蓝
        accent=(0.72, 0.74, 0.77),
        controller="PolyScope",
    ),
    "estun": BrandSpec(
        id="estun",
        name="埃斯顿",
        name_en="ESTUN",
        country="中国",
        primary=(0.13, 0.35, 0.62),
        accent=(0.75, 0.77, 0.80),
        controller="ESTUN RC",
    ),
}


# --------------------------------------------------------------------------- #
# 型号定义
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class RobotSpec:
    """单个机器人型号的规格描述。

    Attributes:
        brand_id: 品牌 id，需存在于 :data:`BRANDS`
        model_id: 型号标识（用于目录名/文件名，小写下划线）
        model_name: 展示用型号名
        payload_kg: 额定负载 (kg)
        reach_mm: 最大臂展/工作半径 (mm)
        repeatability_mm: 重复定位精度 (mm)
        weight_kg: 本体自重 (kg)，未知为 None
        ip_rating: 防护等级，未知为 None
        robot_type: ``industrial``（工业臂）/ ``cobot``（协作臂）/ ``scara``
        joint_limits_deg: 各轴运动范围 (min, max)，单位度；**条数即轴数**
        joint_speeds_deg_s: 各轴最大速度，单位 °/s，条数须与限位一致
        verified: 参数是否取自厂商官方数据表
        note: 补充说明（数据来源、注意事项等）

    Note:
        ``dof`` 不是构造参数，而是由 ``joint_limits_deg`` 的条数推导的派生属性，
        这样可以从根本上杜绝"轴数"与"限位表"不一致的错误。
    """

    brand_id: str
    model_id: str
    model_name: str
    payload_kg: float
    reach_mm: float
    repeatability_mm: float
    joint_limits_deg: Tuple[Tuple[float, float], ...]
    joint_speeds_deg_s: Tuple[float, ...]
    robot_type: str = "industrial"
    weight_kg: Optional[float] = None
    ip_rating: Optional[str] = None
    controller: Optional[str] = None
    verified: bool = False
    note: str = ""

    def __post_init__(self) -> None:
        """校验限位表与速度表的一致性（轴数由限位条数决定）。"""
        n = len(self.joint_limits_deg)
        if n != len(self.joint_speeds_deg_s):
            raise ValueError(
                f"{self.brand_id}/{self.model_id}: joint_limits_deg has {n} entries "
                f"but joint_speeds_deg_s has {len(self.joint_speeds_deg_s)}"
            )
        for i, (lo, hi) in enumerate(self.joint_limits_deg):
            if hi <= lo:
                raise ValueError(
                    f"{self.brand_id}/{self.model_id}: joint {i + 1} "
                    f"limit upper ({hi}) must be greater than lower ({lo})"
                )

    # ------------------------- 派生属性 ------------------------- #
    @property
    def dof(self) -> int:
        """自由度（轴数），由关节限位的条数推导，避免与限位表不一致。"""
        return len(self.joint_limits_deg)

    @property
    def brand(self) -> BrandSpec:
        """所属品牌定义。"""
        return BRANDS[self.brand_id]

    @property
    def key(self) -> str:
        """全局唯一键 ``<brand>/<model>``。"""
        return f"{self.brand_id}/{self.model_id}"

    @property
    def urdf_name(self) -> str:
        """URDF 中的 robot name。"""
        return f"{self.brand_id}_{self.model_id}"

    def home_pose_deg(self) -> List[float]:
        """推荐的初始（HOME）姿态，取各轴限位内的安全中值。

        对于限位范围极大的轴（如 ±350°）会收敛到 0°，避免 HOME 姿态过于夸张。
        """
        home: List[float] = []
        for lo, hi in self.joint_limits_deg:
            mid = (lo + hi) / 2.0
            # 范围过大时取 0，避免初始姿态翻转
            if hi - lo > 360.0:
                home.append(0.0)
            else:
                home.append(round(mid, 2))
        return home


# ---- KUKA ----------------------------------------------------------------- #
# 数据来源：KUKA 官方数据表 (my.kuka.com) 与 ros-industrial/kuka_experimental URDF
_KUKA_ROBOTS: List[RobotSpec] = [
    RobotSpec(
        brand_id="kuka",
        model_id="kr6_r900",
        model_name="KR 6 R900",
        robot_type="industrial",
        payload_kg=6.0,
        reach_mm=901.5,
        repeatability_mm=0.03,
        weight_kg=52.0,
        ip_rating="IP54",
        controller="KRC4 compact",
        joint_limits_deg=(
            (-170, 170),  # A1
            (-190, 45),  # A2
            (-120, 156),  # A3
            (-185, 185),  # A4
            (-120, 120),  # A5
            (-350, 350),  # A6
        ),
        joint_speeds_deg_s=(360, 300, 360, 381, 388, 615),
        verified=True,
        note="KR AGILUS 系列，小型高速六轴臂",
    ),
    RobotSpec(
        brand_id="kuka",
        model_id="kr16_2",
        model_name="KR 16-2",
        robot_type="industrial",
        payload_kg=16.0,
        reach_mm=1611.0,
        repeatability_mm=0.05,
        weight_kg=235.0,
        ip_rating="IP54",
        controller="KRC4",
        joint_limits_deg=(
            (-185, 185),
            (-155, 35),
            (-130, 154),
            (-350, 350),
            (-130, 130),
            (-350, 350),
        ),
        joint_speeds_deg_s=(156, 156, 156, 330, 330, 615),
        verified=True,
        note="经典中载机型；几何比例参考 ros-industrial kuka_kr16_support URDF",
    ),
    RobotSpec(
        brand_id="kuka",
        model_id="kr20_r1810",
        model_name="KR 20 R1810-2",
        robot_type="industrial",
        payload_kg=20.0,
        reach_mm=1813.0,
        repeatability_mm=0.04,
        weight_kg=286.0,
        ip_rating="IP54",
        controller="KRC4",
        joint_limits_deg=(
            (-185, 185),
            (-185, 65),
            (-138, 175),
            (-350, 350),
            (-130, 130),
            (-350, 350),
        ),
        joint_speeds_deg_s=(200, 175, 190, 430, 430, 630),
        verified=True,
        note="KR CYBERTECH 系列",
    ),
    RobotSpec(
        brand_id="kuka",
        model_id="kr60_3",
        model_name="KR 60-3",
        robot_type="industrial",
        payload_kg=60.0,
        reach_mm=2033.0,
        repeatability_mm=0.06,
        weight_kg=665.0,
        ip_rating="IP54",
        controller="KRC4",
        joint_limits_deg=(
            (-185, 185),
            (-135, 35),
            (-120, 158),
            (-350, 350),
            (-119, 119),
            (-350, 350),
        ),
        joint_speeds_deg_s=(140, 120, 140, 260, 245, 322),
        verified=True,
        note="中重载机型",
    ),
    RobotSpec(
        brand_id="kuka",
        model_id="kr210_r2700",
        model_name="KR 210 R2700-2",
        robot_type="industrial",
        payload_kg=210.0,
        reach_mm=2701.0,
        repeatability_mm=0.05,
        weight_kg=1077.0,
        ip_rating="IP54",
        controller="KRC4",
        joint_limits_deg=(
            (-185, 185),
            (-140, -5),
            (-120, 168),
            (-350, 350),
            (-125, 125),
            (-350, 350),
        ),
        joint_speeds_deg_s=(120, 115, 112, 179, 172, 220),
        verified=True,
        note="KR QUANTEC 重载系列，底座占地约 830×830 mm",
    ),
    RobotSpec(
        brand_id="kuka",
        model_id="lbr_iiwa_7",
        model_name="LBR iiwa 7 R800",
        robot_type="cobot",
        payload_kg=7.0,
        reach_mm=800.0,
        repeatability_mm=0.1,
        weight_kg=23.9,
        ip_rating="IP54",
        controller="KRC4",
        joint_limits_deg=(
            (-170, 170),
            (-120, 120),
            (-170, 170),
            (-120, 120),
            (-170, 170),
            (-120, 120),
            (-175, 175),
        ),
        joint_speeds_deg_s=(98, 98, 100, 130, 140, 180, 180),
        verified=True,
        note="7 轴轻量协作臂，法兰 DIN ISO 9409-1-A50",
    ),
]

# ---- JAKA ----------------------------------------------------------------- #
# 数据来源：JAKA 2025 官方产品选型手册
# Zu 系列各轴运动范围与速度在家族内通用
_JAKA_LIMITS = (
    (-360, 360),  # J1
    (-85, 265),  # J2
    (-175, 175),  # J3
    (-85, 265),  # J4
    (-360, 360),  # J5
    (-360, 360),  # J6
)
_JAKA_SPEED_ZU3 = (180, 180, 180, 220, 220, 220)
_JAKA_SPEED_MID = (180, 180, 180, 180, 180, 180)
_JAKA_SPEED_LARGE = (120, 120, 180, 180, 180, 180)
_JAKA_SPEED_HEAVY = (120, 120, 120, 220, 220, 220)

_JAKA_ROBOTS: List[RobotSpec] = [
    RobotSpec(
        brand_id="jaka",
        model_id="zu3",
        model_name="Zu 3",
        robot_type="cobot",
        payload_kg=3.0,
        reach_mm=626.0,
        repeatability_mm=0.02,
        weight_kg=12.0,
        ip_rating="IP54",
        joint_limits_deg=_JAKA_LIMITS,
        joint_speeds_deg_s=_JAKA_SPEED_ZU3,
        verified=True,
        note="底座直径 129 mm，TCP 最大速度 1.5 m/s",
    ),
    RobotSpec(
        brand_id="jaka",
        model_id="zu5",
        model_name="Zu 5",
        robot_type="cobot",
        payload_kg=5.0,
        reach_mm=954.0,
        repeatability_mm=0.02,
        weight_kg=23.0,
        ip_rating="IP54",
        joint_limits_deg=_JAKA_LIMITS,
        joint_speeds_deg_s=_JAKA_SPEED_MID,
        verified=True,
        note="底座直径 158 mm，TCP 最大速度 3.0 m/s",
    ),
    RobotSpec(
        brand_id="jaka",
        model_id="zu7",
        model_name="Zu 7",
        robot_type="cobot",
        payload_kg=7.0,
        reach_mm=819.0,
        repeatability_mm=0.02,
        weight_kg=22.0,
        ip_rating="IP54",
        joint_limits_deg=_JAKA_LIMITS,
        joint_speeds_deg_s=_JAKA_SPEED_MID,
        verified=True,
        note="底座直径 158 mm，TCP 最大速度 2.5 m/s",
    ),
    RobotSpec(
        brand_id="jaka",
        model_id="zu12",
        model_name="Zu 12",
        robot_type="cobot",
        payload_kg=12.0,
        reach_mm=1327.0,
        repeatability_mm=0.03,
        weight_kg=41.0,
        ip_rating="IP54",
        joint_limits_deg=_JAKA_LIMITS,
        joint_speeds_deg_s=_JAKA_SPEED_LARGE,
        verified=True,
        note="底座直径 188 mm，TCP 最大速度 3.0 m/s",
    ),
    RobotSpec(
        brand_id="jaka",
        model_id="zu18",
        model_name="Zu 18",
        robot_type="cobot",
        payload_kg=18.0,
        reach_mm=1073.0,
        repeatability_mm=0.03,
        weight_kg=35.0,
        ip_rating="IP54",
        joint_limits_deg=_JAKA_LIMITS,
        joint_speeds_deg_s=_JAKA_SPEED_LARGE,
        verified=True,
        note="底座直径 188 mm，TCP 最大速度 3.5 m/s",
    ),
    RobotSpec(
        brand_id="jaka",
        model_id="zu20",
        model_name="Zu 20",
        robot_type="cobot",
        payload_kg=20.0,
        reach_mm=1780.0,
        repeatability_mm=0.05,
        weight_kg=68.0,
        ip_rating="IP65",
        joint_limits_deg=_JAKA_LIMITS,
        joint_speeds_deg_s=_JAKA_SPEED_HEAVY,
        verified=True,
        note="底座直径 246 mm，TCP 最大速度 5.0 m/s",
    ),
    RobotSpec(
        brand_id="jaka",
        model_id="zu30",
        model_name="Zu 30",
        robot_type="cobot",
        payload_kg=30.0,
        reach_mm=1350.0,
        repeatability_mm=0.05,
        weight_kg=65.0,
        ip_rating="IP65",
        joint_limits_deg=_JAKA_LIMITS,
        joint_speeds_deg_s=_JAKA_SPEED_HEAVY,
        verified=True,
        note="底座直径 246 mm，TCP 最大速度 4.2 m/s",
    ),
]

# ---- ABB ------------------------------------------------------------------ #
# 负载/臂展/精度取自厂商公开样本；各轴限位为行业典型值
_ABB_LIMITS = ((-180, 180), (-110, 110), (-225, 85), (-200, 200), (-120, 120), (-400, 400))
_ABB_SPEEDS = (250, 250, 250, 320, 320, 420)

_ABB_ROBOTS: List[RobotSpec] = [
    RobotSpec(
        brand_id="abb",
        model_id="irb1200",
        model_name="IRB 1200",
        robot_type="industrial",
        payload_kg=7.0,
        reach_mm=1200.0,
        repeatability_mm=0.02,
        weight_kg=54.0,
        ip_rating="IP40",
        joint_limits_deg=_ABB_LIMITS,
        joint_speeds_deg_s=_ABB_SPEEDS,
        verified=False,
        note="负载/臂展取自厂商样本；轴限位为同级别机型典型值",
    ),
    RobotSpec(
        brand_id="abb",
        model_id="irb1600",
        model_name="IRB 1600",
        robot_type="industrial",
        payload_kg=6.0,
        reach_mm=1200.0,
        repeatability_mm=0.02,
        weight_kg=250.0,
        ip_rating="IP54",
        joint_limits_deg=_ABB_LIMITS,
        joint_speeds_deg_s=_ABB_SPEEDS,
        verified=False,
        note="负载/臂展取自厂商样本；轴限位为同级别机型典型值",
    ),
    RobotSpec(
        brand_id="abb",
        model_id="irb4600_20_250",
        model_name="IRB 4600-20/250",
        robot_type="industrial",
        payload_kg=20.0,
        reach_mm=2550.0,
        repeatability_mm=0.06,
        weight_kg=305.0,
        ip_rating="IP54",
        joint_limits_deg=_ABB_LIMITS,
        joint_speeds_deg_s=(175, 175, 175, 250, 250, 360),
        verified=False,
        note="负载/臂展取自厂商样本；轴限位为同级别机型典型值",
    ),
    RobotSpec(
        brand_id="abb",
        model_id="irb6700_150_320",
        model_name="IRB 6700-150/320",
        robot_type="industrial",
        payload_kg=150.0,
        reach_mm=3200.0,
        repeatability_mm=0.1,
        weight_kg=1170.0,
        ip_rating="IP54",
        joint_limits_deg=_ABB_LIMITS,
        joint_speeds_deg_s=(100, 90, 90, 175, 170, 250),
        verified=False,
        note="负载/臂展取自厂商样本；轴限位为同级别机型典型值",
    ),
]

# ---- FANUC ---------------------------------------------------------------- #
_FANUC_LIMITS = ((-185, 185), (-140, 140), (-160, 160), (-200, 200), (-140, 140), (-360, 360))
_FANUC_SPEEDS = (195, 195, 195, 400, 400, 600)

_FANUC_ROBOTS: List[RobotSpec] = [
    RobotSpec(
        brand_id="fanuc",
        model_id="m20ia_35m",
        model_name="M-20iA/35M",
        robot_type="industrial",
        payload_kg=35.0,
        reach_mm=1813.0,
        repeatability_mm=0.08,
        weight_kg=250.0,
        ip_rating="IP54",
        joint_limits_deg=_FANUC_LIMITS,
        joint_speeds_deg_s=_FANUC_SPEEDS,
        verified=False,
        note="负载/臂展取自厂商样本；轴限位为同级别机型典型值",
    ),
    RobotSpec(
        brand_id="fanuc",
        model_id="m710ic_50",
        model_name="M-710iC/50",
        robot_type="industrial",
        payload_kg=50.0,
        reach_mm=2050.0,
        repeatability_mm=0.07,
        weight_kg=560.0,
        ip_rating="IP54",
        joint_limits_deg=_FANUC_LIMITS,
        joint_speeds_deg_s=(175, 175, 175, 300, 300, 450),
        verified=False,
        note="负载/臂展取自厂商样本；轴限位为同级别机型典型值",
    ),
    RobotSpec(
        brand_id="fanuc",
        model_id="r2000ic_165f",
        model_name="R-2000iC/165F",
        robot_type="industrial",
        payload_kg=165.0,
        reach_mm=2655.0,
        repeatability_mm=0.3,
        weight_kg=1090.0,
        ip_rating="IP54",
        joint_limits_deg=_FANUC_LIMITS,
        joint_speeds_deg_s=(120, 110, 110, 190, 190, 260),
        verified=False,
        note="负载/臂展取自厂商样本；轴限位为同级别机型典型值",
    ),
]

# ---- Universal Robots ----------------------------------------------------- #
# UR e 系列：J1~J6 限位为 ±360°，J2/J3/J5 为 ±180°（官方值略有差异，此处取常用值）
_UR_LIMITS = ((-360, 360), (-180, 180), (-180, 180), (-360, 360), (-360, 360), (-360, 360))
_UR_SPEEDS = (180, 180, 180, 360, 360, 360)

_UR_ROBOTS: List[RobotSpec] = [
    RobotSpec(
        brand_id="ur",
        model_id="ur3e",
        model_name="UR3e",
        robot_type="cobot",
        payload_kg=3.0,
        reach_mm=500.0,
        repeatability_mm=0.03,
        weight_kg=11.2,
        ip_rating="IP54",
        joint_limits_deg=_UR_LIMITS,
        joint_speeds_deg_s=_UR_SPEEDS,
        verified=False,
        note="负载/臂展为官方值；轴限位取 UR e 系列常见范围",
    ),
    RobotSpec(
        brand_id="ur",
        model_id="ur5e",
        model_name="UR5e",
        robot_type="cobot",
        payload_kg=5.0,
        reach_mm=850.0,
        repeatability_mm=0.03,
        weight_kg=20.6,
        ip_rating="IP54",
        joint_limits_deg=_UR_LIMITS,
        joint_speeds_deg_s=_UR_SPEEDS,
        verified=False,
        note="负载/臂展为官方值；轴限位取 UR e 系列常见范围",
    ),
    RobotSpec(
        brand_id="ur",
        model_id="ur10e",
        model_name="UR10e",
        robot_type="cobot",
        payload_kg=12.5,
        reach_mm=1300.0,
        repeatability_mm=0.05,
        weight_kg=33.5,
        ip_rating="IP54",
        joint_limits_deg=_UR_LIMITS,
        joint_speeds_deg_s=(120, 120, 180, 180, 180, 180),
        verified=False,
        note="负载/臂展为官方值；轴限位取 UR e 系列常见范围",
    ),
    RobotSpec(
        brand_id="ur",
        model_id="ur16e",
        model_name="UR16e",
        robot_type="cobot",
        payload_kg=16.0,
        reach_mm=900.0,
        repeatability_mm=0.05,
        weight_kg=33.1,
        ip_rating="IP54",
        joint_limits_deg=_UR_LIMITS,
        joint_speeds_deg_s=(120, 120, 180, 180, 180, 180),
        verified=False,
        note="负载/臂展为官方值；轴限位取 UR e 系列常见范围",
    ),
]

# ---- ESTUN ---------------------------------------------------------------- #
_ESTUN_LIMITS = ((-180, 180), (-155, 35), (-130, 154), (-350, 350), (-130, 130), (-350, 350))
_ESTUN_SPEEDS = (185, 185, 185, 330, 330, 615)

_ESTUN_ROBOTS: List[RobotSpec] = [
    RobotSpec(
        brand_id="estun",
        model_id="er6_1450",
        model_name="ER6-1450",
        robot_type="industrial",
        payload_kg=6.0,
        reach_mm=1450.0,
        repeatability_mm=0.05,
        weight_kg=170.0,
        ip_rating="IP54",
        joint_limits_deg=_ESTUN_LIMITS,
        joint_speeds_deg_s=_ESTUN_SPEEDS,
        verified=False,
        note="负载/臂展取自厂商样本；轴限位为同级别机型典型值",
    ),
    RobotSpec(
        brand_id="estun",
        model_id="er20_1780",
        model_name="ER20-1780",
        robot_type="industrial",
        payload_kg=20.0,
        reach_mm=1780.0,
        repeatability_mm=0.05,
        weight_kg=280.0,
        ip_rating="IP54",
        joint_limits_deg=_ESTUN_LIMITS,
        joint_speeds_deg_s=(175, 175, 175, 300, 300, 460),
        verified=False,
        note="负载/臂展取自厂商样本；轴限位为同级别机型典型值",
    ),
    RobotSpec(
        brand_id="estun",
        model_id="er50_2100",
        model_name="ER50-2100",
        robot_type="industrial",
        payload_kg=50.0,
        reach_mm=2100.0,
        repeatability_mm=0.06,
        weight_kg=560.0,
        ip_rating="IP54",
        joint_limits_deg=_ESTUN_LIMITS,
        joint_speeds_deg_s=(150, 150, 150, 260, 260, 400),
        verified=False,
        note="负载/臂展取自厂商样本；轴限位为同级别机型典型值",
    ),
]


# --------------------------------------------------------------------------- #
# 汇总与查询
# --------------------------------------------------------------------------- #
ROBOTS: List[RobotSpec] = (
    _KUKA_ROBOTS + _JAKA_ROBOTS + _ABB_ROBOTS + _FANUC_ROBOTS + _UR_ROBOTS + _ESTUN_ROBOTS
)

_BY_KEY: Dict[str, RobotSpec] = {r.key: r for r in ROBOTS}


def iter_robots():
    """遍历全部型号规格。"""
    return iter(ROBOTS)


def get_robot(key: str) -> Optional[RobotSpec]:
    """按 ``brand/model`` 键取规格，不存在返回 None。"""
    return _BY_KEY.get(key)


def get_brand(brand_id: str) -> Optional[BrandSpec]:
    """按品牌 id 取品牌定义。"""
    return BRANDS.get(brand_id)


def robot_key(brand_id: str, model_id: str) -> str:
    """拼接型号键。"""
    return f"{brand_id}/{model_id}"
