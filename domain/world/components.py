"""参数化工业组件库——用生成器替代模型资产。

为什么不打包模型文件
--------------------
RoboDK 卖的很大一部分价值是它的在线资源库（1200+ 机器人、工具、组件）。
Visual Components 的 eCatalog 同理——本质是**模型资产**。

开源项目在资产数量上永远追不平，也没必要追。这里的思路是给出**生成器**：
用户填几个工程参数（长宽高、工位数、行程），程序实时生成几何。好处是

* 参数天然落在工程语义上（"台面高 800mm" 而不是"顶点坐标 y=0.8"）
* 零下载、零许可问题、零二进制资产
* 尺寸可连续调整，不受预制模型档位限制

每个组件由若干基本体（:mod:`~domain.world.geometry`）拼装，因此可以直接参与
碰撞检测与 OpenGL 渲染，无需额外的网格管线。

单位：毫米。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, List, Tuple

import numpy as np

from .geometry import Box, Cylinder, Primitive, Sphere

__all__ = [
    "Part",
    "ParamSpec",
    "ComponentSpec",
    "COMPONENTS",
    "get_component",
    "categories",
    "build_component",
]


# --------------------------------------------------------------------------- #
# 数据结构
# --------------------------------------------------------------------------- #


@dataclass
class Part:
    """组件的一个几何组成部分。

    Attributes:
        prim: 基本体
        transform: 4x4 变换，位于**组件局部坐标系**
        color: RGB (0~1)
    """

    prim: Primitive
    transform: np.ndarray
    color: Tuple[float, float, float]


@dataclass(frozen=True)
class ParamSpec:
    """一个可调参数的定义，供 GUI 自动生成控件。

    Attributes:
        name: 参数键
        label: 中文显示名
        default: 默认值
        minimum: 最小值
        maximum: 最大值
        unit: 单位字符串
        step: 微调步长
    """

    name: str
    label: str
    default: float
    minimum: float
    maximum: float
    unit: str = "mm"
    step: float = 10.0


@dataclass(frozen=True)
class AxisSpec:
    """设备的一个可动轴（关节）。

    轴与尺寸参数的区别：尺寸参数描述"这个设备长什么样"（进「参数」面板），
    轴描述"这个设备当前摆成什么姿势"（进「关节」面板，供调姿态）。

    Attributes:
        name: 轴标识
        label: 中文显示名（如「倾斜轴」）
        param: 对应的参数键；**必须已被 builder 支持**，否则调了没有几何响应
        minimum: 最小角度/位移
        maximum: 最大角度/位移
        unit: 单位（默认度；直线轴传 "mm"）
    """

    name: str
    label: str
    param: str
    minimum: float
    maximum: float
    unit: str = "°"


@dataclass(frozen=True)
class ComponentSpec:
    """一个参数化组件。

    Attributes:
        id: 唯一标识
        name: 中文名
        category: 分类（用于面板分组）
        params: 参数定义列表
        builder: 构建函数，接收参数字典返回 :class:`Part` 列表
        description: 说明文字
        axes: 可动轴定义；有轴的设备在「关节」面板提供调节滑块
    """

    id: str
    name: str
    category: str
    params: Tuple[ParamSpec, ...]
    builder: Callable[..., List[Part]]
    description: str = ""
    axes: Tuple[AxisSpec, ...] = ()

    def axis_param_keys(self) -> Tuple[str, ...]:
        """所有轴对应的参数键，便于从「参数」面板中排除。"""
        return tuple(a.param for a in self.axes)

    def plain_params(self) -> Tuple[ParamSpec, ...]:
        """排除掉轴之后的普通尺寸参数。"""
        keys = set(self.axis_param_keys())
        return tuple(p for p in self.params if p.name not in keys)


# --------------------------------------------------------------------------- #
# 工具函数
# --------------------------------------------------------------------------- #

# 工业配色（RGB 0~1）
C_STEEL = (0.42, 0.44, 0.48)  # 钢结构
C_DARK = (0.20, 0.21, 0.24)  # 深色机架
C_WORK = (0.30, 0.46, 0.62)  # 工件（蓝灰）
C_FIXTURE = (0.85, 0.45, 0.12)  # 夹具（橙）
C_SAFETY = (0.90, 0.72, 0.10)  # 安全设施（黄）
C_BELT = (0.25, 0.25, 0.27)  # 输送带
C_CABINET = (0.62, 0.64, 0.67)  # 电柜


def _T(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> np.ndarray:
    """平移矩阵。"""
    m = np.eye(4)
    m[:3, 3] = [x, y, z]
    return m


def _TRz(x: float = 0.0, y: float = 0.0, z: float = 0.0, deg: float = 0.0) -> np.ndarray:
    """绕 Z 旋转 + 平移。"""
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    m = np.eye(4)
    m[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    m[:3, 3] = [x, y, z]
    return m


def _TRy(x: float = 0.0, y: float = 0.0, z: float = 0.0, deg: float = 0.0) -> np.ndarray:
    """绕 Y 旋转 + 平移（用于把圆柱轴从 Z 扳到水平）。"""
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    m = np.eye(4)
    m[:3, :3] = [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
    m[:3, 3] = [x, y, z]
    return m


# --------------------------------------------------------------------------- #
# 组件构建器
# --------------------------------------------------------------------------- #


def _build_worktable(
    length: float,
    width: float,
    height: float,
    top_thickness: float,
    **_,
) -> List[Part]:
    """工作台：台面板 + 四条腿。"""
    parts = [
        Part(Box((length, width, top_thickness)), _T(0, 0, height - top_thickness / 2), C_STEEL)
    ]
    leg = min(80.0, min(length, width) * 0.15)
    leg_h = height - top_thickness
    inset_x = length / 2 - leg
    inset_y = width / 2 - leg
    for sx in (-1, 1):
        for sy in (-1, 1):
            parts.append(
                Part(
                    Box((leg, leg, leg_h)),
                    _T(sx * inset_x, sy * inset_y, leg_h / 2),
                    C_DARK,
                )
            )
    return parts


def _build_workpiece_box(length: float, width: float, height: float, **_) -> List[Part]:
    """矩形工件（毛坯）。"""
    return [Part(Box((length, width, height)), _T(0, 0, height / 2), C_WORK)]


def _build_workpiece_plate(length: float, width: float, thickness: float, **_) -> List[Part]:
    """板状工件（焊接件常见）。"""
    return [Part(Box((length, width, thickness)), _T(0, 0, thickness / 2), C_WORK)]


def _build_workpiece_cylinder(radius: float, height: float, **_) -> List[Part]:
    """圆柱工件（轴类、法兰类）。"""
    return [Part(Cylinder(radius, height), _T(0, 0, height / 2), C_WORK)]


def _build_workpiece_sphere(radius: float, **_) -> List[Part]:
    """球体工件/结构件：球类零件、滚轮、料球。"""
    return [Part(Sphere(radius), _T(0, 0, radius), C_WORK)]


def _build_vise(jaw_width: float, jaw_opening: float, body_height: float, **_) -> List[Part]:
    """台钳：底座 + 固定钳口 + 活动钳口。"""
    parts = [
        Part(
            Box((jaw_width * 1.6, jaw_width * 1.1, body_height * 0.45)),
            _T(0, 0, body_height * 0.225),
            C_DARK,
        )
    ]
    jaw_h = body_height * 0.55
    # 固定钳口
    parts.append(
        Part(
            Box((jaw_width * 0.25, jaw_width, jaw_h)),
            _T(-jaw_width * 0.55, 0, body_height * 0.45 + jaw_h / 2),
            C_FIXTURE,
        )
    )
    # 活动钳口（按开口量偏移）
    parts.append(
        Part(
            Box((jaw_width * 0.25, jaw_width, jaw_h)),
            _T(-jaw_width * 0.55 + jaw_opening, 0, body_height * 0.45 + jaw_h / 2),
            C_FIXTURE,
        )
    )
    return parts


def _build_clamp(arm_length: float, clamp_height: float, **_) -> List[Part]:
    """压板夹具：立柱 + 压臂 + 压头。"""
    return [
        Part(Cylinder(18.0, clamp_height), _T(0, 0, clamp_height / 2), C_DARK),
        Part(
            Box((arm_length, 40.0, 22.0)),
            _T(arm_length / 2, 0, clamp_height),
            C_FIXTURE,
        ),
        Part(Cylinder(14.0, 40.0), _T(arm_length, 0, clamp_height - 20.0), C_FIXTURE),
    ]


def _build_pedestal(height: float, top_size: float, **_) -> List[Part]:
    """机器人底座/立柱。"""
    base = top_size * 0.8
    return [
        Part(Box((base, base, 60.0)), _T(0, 0, 30.0), C_DARK),
        Part(
            Cylinder(top_size * 0.38, height - 60.0), _T(0, 0, 60.0 + (height - 60.0) / 2), C_STEEL
        ),
        Part(Box((top_size, top_size, 30.0)), _T(0, 0, height - 15.0), C_STEEL),
    ]


def _build_fence(length: float, height: float, **_) -> List[Part]:
    """安全围栏（单片）：网板 + 立柱 + 踢脚板。"""
    post = 60.0
    parts = [
        Part(Box((length, 30.0, height * 0.9)), _T(0, 0, height * 0.55), C_SAFETY),
        Part(Box((length, 60.0, 120.0)), _T(0, 0, 60.0), C_SAFETY),
    ]
    for sx in (-1, 1):
        parts.append(
            Part(Box((post, post, height)), _T(sx * (length / 2 - post / 2), 0, height / 2), C_DARK)
        )
    return parts


def _build_positioner_1axis(
    table_diameter: float, table_height: float, rot_deg: float = 0.0, **_
) -> List[Part]:
    """单轴回转变位机（转台）。

    立柱固定，**台面与工件接口随回转轴转动**——否则加了 rot_deg 参数
    在几何上也看不出变化。
    """
    rot = _TRz(0, 0, 0, deg=rot_deg)
    return [
        # 立柱（固定）
        Part(
            Cylinder(table_diameter * 0.42, table_height * 0.8),
            _T(0, 0, table_height * 0.4),
            C_STEEL,
        ),
        # 台面（随回转）
        Part(
            Cylinder(table_diameter * 0.5, 30.0), rot @ _T(0, 0, table_height * 0.8 + 15.0), C_DARK
        ),
        # 工件接口（随回转）
        Part(
            Cylinder(table_diameter * 0.28, table_height * 0.2),
            rot @ _T(0, 0, table_height * 0.9),
            C_FIXTURE,
        ),
    ]


def _build_positioner_2axis(
    table_diameter: float, table_height: float, tilt_deg: float, rot_deg: float = 0.0, **_
) -> List[Part]:
    """双轴变位机（L 型，回转 + 倾斜）。

    变换顺序为**先倾斜、再在倾斜坐标系上回转**（``tilt @ rot``），
    与真实 L 型变位机一致：倾斜臂抬起后，回转台绕自身轴转动。
    """
    parts = [
        Part(
            Cylinder(table_diameter * 0.34, table_height * 0.9),
            _T(0, 0, table_height * 0.45),
            C_STEEL,
        ),
        Part(
            Box((table_diameter * 0.9, table_diameter * 0.5, 40.0)),
            _T(0, 0, table_height * 0.9 + 20.0),
            C_DARK,
        ),
    ]
    tilt = _TRy(0, 0, table_height * 0.9 + 40.0, deg=tilt_deg)
    table = tilt @ _TRz(0, 0, 0, deg=rot_deg)
    parts.append(Part(Cylinder(table_diameter * 0.42, 30.0), table @ _T(0, 0, 15.0), C_FIXTURE))
    parts.append(Part(Cylinder(table_diameter * 0.3, 25.0), table @ _T(0, 0, 42.0), C_STEEL))
    return parts


def _build_conveyor(length: float, width: float, height: float, **_) -> List[Part]:
    """皮带输送线：支腿 + 侧梁 + 皮带面 + 端辊。"""
    parts = []
    leg = 50.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            parts.append(
                Part(
                    Box((leg, leg, height)),
                    _T(sx * (length / 2 - leg), sy * (width / 2 - leg / 2), height / 2),
                    C_DARK,
                )
            )
    # 侧梁
    for sy in (-1, 1):
        parts.append(
            Part(Box((length, 30.0, 90.0)), _T(0, sy * (width / 2 - 15.0), height + 45.0), C_STEEL)
        )
    # 皮带面
    parts.append(Part(Box((length, width - 60.0, 12.0)), _T(0, 0, height + 60.0), C_BELT))
    # 端辊
    for sx in (-1, 1):
        parts.append(
            Part(
                Cylinder(45.0, width),
                _TRy(sx * (length / 2 - 20.0), 0, height + 56.0, deg=90.0),
                C_STEEL,
            )
        )
    return parts


def _build_cabinet(width: float, depth: float, height: float, **_) -> List[Part]:
    """控制柜：柜体 + 门缝 + 底座。"""
    return [
        Part(Box((width, depth, 80.0)), _T(0, 0, 40.0), C_DARK),
        Part(Box((width, depth, height - 80.0)), _T(0, 0, 80.0 + (height - 80.0) / 2), C_CABINET),
        Part(
            Box((width * 0.9, 10.0, (height - 80.0) * 0.85)),
            _T(0, depth / 2, 80.0 + (height - 80.0) / 2),
            C_DARK,
        ),
    ]


def _build_tool_gripper(body_length: float, jaw_opening: float, **_) -> List[Part]:
    """两指夹爪：法兰 + 缸体 + 双指。"""
    parts = [
        Part(Cylinder(35.0, 25.0), _T(0, 0, 12.5), C_STEEL),
        Part(Box((70.0, 90.0, body_length)), _T(0, 0, 25.0 + body_length / 2), C_DARK),
    ]
    for sy in (-1, 1):
        parts.append(
            Part(
                Box((50.0, 18.0, body_length * 0.7)),
                _T(0, sy * (jaw_opening / 2 + 9.0), 25.0 + body_length + body_length * 0.35),
                C_FIXTURE,
            )
        )
    return parts


def _build_tool_weldgun(neck_length: float, neck_angle: float, **_) -> List[Part]:
    """焊枪：法兰 + 枪颈 + 导电嘴。"""
    parts = [
        Part(Cylinder(38.0, 30.0), _T(0, 0, 15.0), C_STEEL),
        Part(Cylinder(28.0, neck_length * 0.5), _T(0, 0, 30.0 + neck_length * 0.25), C_DARK),
    ]
    bend = _TRy(0, 0, 30.0 + neck_length * 0.5, deg=neck_angle)
    parts.append(
        Part(Cylinder(20.0, neck_length * 0.5), bend @ _T(0, 0, neck_length * 0.25), C_DARK)
    )
    parts.append(Part(Cylinder(10.0, 30.0), bend @ _T(0, 0, neck_length * 0.5 + 15.0), C_FIXTURE))
    return parts


def _build_tool_suction(cup_count: int = 4, plate_size: float = 200.0, **_) -> List[Part]:
    """真空吸盘工具：连接板 + 若干吸盘。"""
    parts = [
        Part(Box((plate_size, plate_size, 20.0)), _T(0, 0, 10.0), C_STEEL),
        Part(Cylinder(30.0, 40.0), _T(0, 0, 40.0), C_DARK),
    ]
    n = max(1, int(cup_count))
    spacing = plate_size * 0.62
    for sx in (-1, 1):
        for sy in (-1, 1):
            if len(parts) - 2 >= n:
                break
            parts.append(
                Part(Cylinder(28.0, 34.0), _T(sx * spacing / 2, sy * spacing / 2, 0.0), C_FIXTURE)
            )
    return parts


def _build_material_bin(
    length: float, width: float, height: float, wall_thickness: float, **_
) -> List[Part]:
    """料箱（周转箱）：底板 + 四壁，顶部敞开便于取放工件。"""
    wall = max(8.0, wall_thickness)
    inner_h = max(10.0, height - wall)
    parts = [
        Part(Box((length, width, wall)), _T(0, 0, wall / 2), C_DARK),
    ]
    # 前后壁（沿 X 长边）
    for sy in (-1, 1):
        parts.append(
            Part(
                Box((length, wall, inner_h)),
                _T(0, sy * (width / 2 - wall / 2), wall + inner_h / 2),
                C_STEEL,
            )
        )
    # 左右壁（沿 Y 短边）
    for sx in (-1, 1):
        parts.append(
            Part(
                Box((wall, max(10.0, width - 2 * wall), inner_h)),
                _T(sx * (length / 2 - wall / 2), 0, wall + inner_h / 2),
                C_STEEL,
            )
        )
    return parts


def _build_pallet_base(length: float, width: float, height: float, **_) -> List[Part]:
    """空托盘：面板 + 三条纵梁 + 底垫（无料垛，供 pick & place 落件用）。"""
    deck = min(30.0, height * 0.25)
    foot = min(25.0, height * 0.2)
    runner_h = height - deck - foot
    runner = max(80.0, length * 0.08)
    parts = [
        Part(Box((length, width, deck)), _T(0, 0, height - deck / 2), C_DARK),
    ]
    for sx in (-1, 0, 1):
        x = sx * (length / 2 - runner / 2)
        parts.append(Part(Box((runner, width, runner_h)), _T(x, 0, foot + runner_h / 2), C_STEEL))
    for sy in (-1, 1):
        parts.append(
            Part(
                Box((length, width * 0.06, foot)),
                _T(0, sy * (width / 2 - width * 0.03), foot / 2),
                C_DARK,
            )
        )
    return parts


def _build_pallet(length: float, width: float, layers: float, **_) -> List[Part]:
    """码垛托盘 + 料垛（用于码垛应用仿真）。"""
    box_h = 150.0
    parts = [
        Part(Box((length, width, 120.0)), _T(0, 0, 60.0), C_DARK),
    ]
    rows, cols = 2, 2
    bw, bh = length / cols * 0.9, box_h
    for i in range(rows):
        for j in range(cols):
            for k in range(int(layers)):
                parts.append(
                    Part(
                        Box((bw, bw, bh)),
                        _T(
                            (j - (cols - 1) / 2) * (length / cols),
                            (i - (rows - 1) / 2) * (width / rows),
                            120.0 + bh / 2 + k * bh,
                        ),
                        C_WORK,
                    )
                )
    return parts


# --------------------------------------------------------------------------- #
# 组件注册表
# --------------------------------------------------------------------------- #

_P = ParamSpec

COMPONENTS: Dict[str, ComponentSpec] = {
    spec.id: spec
    for spec in (
        ComponentSpec(
            id="worktable",
            name="工作台",
            category="基础设施",
            description="台面板 + 四腿，可调长宽高与台面厚度",
            params=(
                _P("length", "台面长度", 1200, 300, 4000),
                _P("width", "台面宽度", 800, 300, 2500),
                _P("height", "台面高度", 800, 400, 1200),
                _P("top_thickness", "台面厚度", 40, 10, 120, step=5),
            ),
            builder=_build_worktable,
        ),
        ComponentSpec(
            id="pedestal",
            name="机器人底座",
            category="基础设施",
            description="落地底座/立柱，用于把机器人抬到作业高度",
            params=(
                _P("height", "立柱高度", 400, 0, 1500),
                _P("top_size", "顶面尺寸", 400, 150, 900),
            ),
            builder=_build_pedestal,
        ),
        ComponentSpec(
            id="cabinet",
            name="控制柜",
            category="基础设施",
            description="电柜柜体，用于布局与干涉检查",
            params=(
                _P("width", "柜宽", 600, 300, 1200),
                _P("depth", "柜深", 500, 300, 1000),
                _P("height", "柜高", 1800, 800, 2400),
            ),
            builder=_build_cabinet,
        ),
        ComponentSpec(
            id="fence",
            name="安全围栏",
            category="安全设施",
            description="单片围栏（网板 + 立柱 + 踢脚板）",
            params=(
                _P("length", "片宽", 1500, 500, 3000),
                _P("height", "片高", 2000, 1000, 3000),
            ),
            builder=_build_fence,
        ),
        ComponentSpec(
            id="workpiece_box",
            name="矩形工件",
            category="工件",
            description="长方体毛坯",
            params=(
                _P("length", "长度", 300, 20, 2000),
                _P("width", "宽度", 200, 20, 2000),
                _P("height", "高度", 150, 10, 1500),
            ),
            builder=_build_workpiece_box,
        ),
        ComponentSpec(
            id="workpiece_plate",
            name="板状工件",
            category="工件",
            description="薄板件，焊接应用常见",
            params=(
                _P("length", "长度", 500, 50, 3000),
                _P("width", "宽度", 300, 50, 2000),
                _P("thickness", "板厚", 6, 1, 40, step=1),
            ),
            builder=_build_workpiece_plate,
        ),
        ComponentSpec(
            id="workpiece_cylinder",
            name="圆柱工件",
            category="工件",
            description="轴类/法兰类回转体工件",
            params=(
                _P("radius", "半径", 80, 10, 600),
                _P("height", "高度", 250, 20, 1500),
            ),
            builder=_build_workpiece_cylinder,
        ),
        ComponentSpec(
            id="workpiece_sphere",
            name="球体工件",
            category="工件",
            description="球体零件/滚轮/料球，半径可调",
            params=(_P("radius", "半径", 80, 10, 600),),
            builder=_build_workpiece_sphere,
        ),
        ComponentSpec(
            id="material_bin",
            name="料箱",
            category="工件",
            description="顶部敞开的周转料箱，用于毛坯上料/下料",
            params=(
                _P("length", "箱长", 600, 200, 1500),
                _P("width", "箱宽", 400, 150, 1000),
                _P("height", "箱高", 300, 100, 800),
                _P("wall_thickness", "壁厚", 15, 8, 50, step=1),
            ),
            builder=_build_material_bin,
        ),
        ComponentSpec(
            id="pallet_base",
            name="空托盘",
            category="工件",
            description="无料垛托盘（面板 + 纵梁），供码垛/抓放落件",
            params=(
                _P("length", "托盘长", 1200, 400, 2000),
                _P("width", "托盘宽", 800, 300, 1500),
                _P("height", "托盘高", 144, 60, 300),
            ),
            builder=_build_pallet_base,
        ),
        ComponentSpec(
            id="pallet",
            name="码垛料垛",
            category="工件",
            description="托盘 + 多层料箱，用于码垛应用",
            params=(
                _P("length", "托盘长", 1200, 600, 2000),
                _P("width", "托盘宽", 1000, 600, 2000),
                _P("layers", "层数", 3, 1, 8, unit="层", step=1),
            ),
            builder=_build_pallet,
        ),
        ComponentSpec(
            id="vise",
            name="台钳",
            category="夹具",
            description="机加工/去毛刺常用夹紧装置",
            params=(
                _P("jaw_width", "钳口宽", 150, 60, 400),
                _P("jaw_opening", "开口量", 60, 0, 300, step=5),
                _P("body_height", "总高", 160, 80, 400),
            ),
            builder=_build_vise,
            axes=(AxisSpec("jaw", "钳口开合", "jaw_opening", 0, 300, "mm"),),
        ),
        ComponentSpec(
            id="clamp",
            name="压板夹具",
            category="夹具",
            description="立柱 + 压臂 + 压头",
            params=(
                _P("arm_length", "压臂长", 180, 60, 500),
                _P("clamp_height", "立柱高", 200, 80, 600),
            ),
            builder=_build_clamp,
        ),
        ComponentSpec(
            id="positioner_1axis",
            name="单轴变位机",
            category="变位机",
            description="回转转台，单轴",
            params=(
                _P("table_diameter", "台面直径", 600, 200, 2000),
                _P("table_height", "台面高度", 600, 200, 1500),
                # rot_deg 必须在这里声明：build_component 只接受 spec.params
                # 里登记过的键，未声明的话轴滑块的值会被静默丢弃。
                _P("rot_deg", "回转角", 0, -180, 180, unit="°", step=5),
            ),
            builder=_build_positioner_1axis,
            axes=(AxisSpec("rot", "回转轴", "rot_deg", -180, 180),),
        ),
        ComponentSpec(
            id="positioner_2axis",
            name="双轴变位机",
            category="变位机",
            description="L 型回转 + 倾斜双轴变位机",
            params=(
                _P("table_diameter", "台面直径", 800, 300, 2500),
                _P("table_height", "台面高度", 700, 300, 1800),
                _P("tilt_deg", "倾斜角", 0, -90, 90, unit="°", step=5),
                # 同上：未声明的键会被 build_component 过滤掉
                _P("rot_deg", "回转角", 0, -180, 180, unit="°", step=5),
            ),
            builder=_build_positioner_2axis,
            axes=(
                AxisSpec("rot", "回转轴", "rot_deg", -180, 180),
                AxisSpec("tilt", "倾斜轴", "tilt_deg", -90, 90),
            ),
        ),
        ComponentSpec(
            id="conveyor",
            name="皮带输送线",
            category="输送设备",
            description="支腿 + 侧梁 + 皮带 + 端辊",
            params=(
                _P("length", "线体长度", 3000, 800, 12000),
                _P("width", "带宽", 600, 200, 1500),
                _P("height", "带面高度", 800, 300, 1500),
            ),
            builder=_build_conveyor,
        ),
        ComponentSpec(
            id="tool_gripper",
            name="两指夹爪",
            category="末端工具",
            description="气动两指平行夹爪",
            params=(
                _P("body_length", "缸体长", 120, 40, 400),
                _P("jaw_opening", "开口量", 60, 0, 250, step=5),
            ),
            builder=_build_tool_gripper,
            axes=(AxisSpec("jaw", "开合", "jaw_opening", 0, 250, "mm"),),
        ),
        ComponentSpec(
            id="tool_weldgun",
            name="焊枪",
            category="末端工具",
            description="鹅颈焊枪，枪颈角可调",
            params=(
                _P("neck_length", "枪颈长", 200, 80, 500),
                _P("neck_angle", "枪颈角", 22, 0, 60, unit="°", step=1),
            ),
            builder=_build_tool_weldgun,
        ),
        ComponentSpec(
            id="tool_suction",
            name="真空吸盘",
            category="末端工具",
            description="多吸盘真空抓取工具",
            params=(
                _P("plate_size", "连接板尺寸", 200, 80, 600),
                _P("cup_count", "吸盘数", 4, 1, 4, unit="个", step=1),
            ),
            builder=_build_tool_suction,
        ),
    )
}


def get_component(component_id: str) -> ComponentSpec:
    """按 id 取组件定义。

    Args:
        component_id: 组件 id

    Returns:
        组件定义

    Raises:
        KeyError: 组件不存在
    """
    return COMPONENTS[component_id]


def categories() -> List[str]:
    """返回所有分类名（按注册顺序去重）。"""
    seen: List[str] = []
    for spec in COMPONENTS.values():
        if spec.category not in seen:
            seen.append(spec.category)
    return seen


def default_params(component_id: str) -> Dict[str, float]:
    """取组件的默认参数字典。"""
    return {p.name: p.default for p in COMPONENTS[component_id].params}


def build_component(component_id: str, **params) -> List[Part]:
    """按参数生成组件的几何。

    Args:
        component_id: 组件 id
        **params: 参数值；未提供的参数取默认值

    Returns:
        :class:`Part` 列表（位于组件局部坐标系）

    Raises:
        KeyError: 组件不存在
    """
    spec = COMPONENTS[component_id]
    merged = default_params(component_id)
    merged.update({k: float(v) for k, v in params.items() if k in merged})
    return spec.builder(**merged)
