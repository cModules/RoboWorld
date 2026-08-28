"""程序化组件库 —— 工作站的"零件货架"。

与机器人模型采用同一套思路：**不依赖任何外部 CAD 资源**，全部用参数化的
凸体拼装。这样做的理由：

* 开源项目无法随附厂商 CAD（授权风险），程序化生成零授权问题；
* 参数可调 —— 工作台 1200×800 还是 2000×1000，改个数就行；
* 碰撞体与显示体天然一致，不存在"外观与碰撞体对不上"的经典坑。

分类（:attr:`ComponentSpec.category`）沿用产线惯例，便于 UI 分组：

============  ====================================
category      含义
============  ====================================
structure     结构件（工作台、底座、立柱）
safety        安全设施（围栏、安全门）
transport     输送设备（传送带、辊道）
positioner    变位机（回转台、L 型双轴）
workpiece     工件（方块、板料、棒料、管件）
fixture       夹具（底板、定位块、压紧）
storage       储运（料箱、托盘）
tool          末端工具（夹爪、吸盘、焊枪）
============  ====================================
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Sequence, Tuple

import numpy as np

from .geometry import BoxGeometry, ConvexGeometry, CylinderGeometry, SphereGeometry

# --------------------------------------------------------------------------- #
# 配色（线性 RGB，0~1）
# --------------------------------------------------------------------------- #
COLOR_STEEL = (0.42, 0.45, 0.50)      # 钢结构
COLOR_FRAME = (0.95, 0.62, 0.11)      # 安全围栏/警示黄
COLOR_BELT = (0.16, 0.17, 0.19)       # 皮带/橡胶
COLOR_WORK = (0.30, 0.55, 0.78)       # 工件
COLOR_FIXTURE = (0.55, 0.58, 0.62)    # 夹具
COLOR_BIN = (0.36, 0.62, 0.42)        # 料箱
COLOR_TOOL = (0.72, 0.30, 0.32)       # 末端工具
COLOR_DECK = (0.60, 0.63, 0.67)       # 台面


def T_xyz(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> np.ndarray:
    """纯平移变换。"""
    M = np.eye(4)
    M[:3, 3] = (x, y, z)
    return M


def T_rpy(rpy: Sequence[float], xyz: Sequence[float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    """由 rpy（弧度，XYZ 内旋顺序）与平移构造 4x4 变换。

    Args:
        rpy: ``(rx, ry, rz)`` 弧度
        xyz: 平移（米）
    """
    rx, ry, rz = (float(v) for v in rpy)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    M = np.eye(4)
    M[:3, :3] = Rz @ Ry @ Rx
    M[:3, 3] = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
    return M


# --------------------------------------------------------------------------- #
# 数据结构
# --------------------------------------------------------------------------- #
@dataclass
class ComponentPart:
    """组件中的一个凸体零件。

    Attributes:
        name: 零件名，用于场景树与碰撞报告定位
        geometry: 凸体几何（局部坐标，米）
        T_local: 相对组件原点的 4x4 变换（米）
        color: 显示颜色 RGB（0~1）
    """

    name: str
    geometry: ConvexGeometry
    T_local: np.ndarray
    color: Tuple[float, float, float]

    def world_transform(self, T_component: np.ndarray) -> np.ndarray:
        """组件位姿左乘零件局部变换，得到世界变换。"""
        return np.asarray(T_component, dtype=np.float64) @ self.T_local


@dataclass
class ComponentParam:
    """一个可调参数的定义。

    Attributes:
        key: 参数键
        label: UI 显示名
        default: 默认值（**米**）
        minimum / maximum: 取值范围（米）
    """

    key: str
    label: str
    default: float
    minimum: float
    maximum: float


@dataclass
class ComponentSpec:
    """组件定义（模板）。

    Attributes:
        id: 唯一标识，如 ``"workbench"``
        name: 中文显示名
        category: 分类
        description: 用途说明
        params: 可调参数
        factory: 由参数字典构造零件列表的函数
    """

    id: str
    name: str
    category: str
    description: str
    params: Tuple[ComponentParam, ...]
    factory: Callable[[Dict[str, float]], List[ComponentPart]]

    def default_params(self) -> Dict[str, float]:
        """取所有参数的默认值。"""
        return {p.key: p.default for p in self.params}

    def build(self, **overrides: float) -> "Component":
        """用默认参数（可被 ``overrides`` 覆盖）实例化组件。

        Args:
            **overrides: 覆盖默认值的参数

        Returns:
            :class:`Component`

        Raises:
            KeyError: 传入了未定义的参数名
        """
        params = self.default_params()
        unknown = set(overrides) - set(params)
        if unknown:
            raise KeyError(f"{self.id} 没有参数 {sorted(unknown)}，可用：{sorted(params)}")
        params.update({k: float(v) for k, v in overrides.items()})
        parts = self.factory(params)
        return Component(spec=self, params=params, parts=parts)


@dataclass
class Component:
    """实例化后的组件。

    Attributes:
        spec: 所属定义
        params: 实际使用的参数值
        parts: 零件列表
    """

    spec: ComponentSpec
    params: Dict[str, float]
    parts: List[ComponentPart]

    @property
    def id(self) -> str:
        """组件类型标识。"""
        return self.spec.id

    @property
    def name(self) -> str:
        """组件中文名。"""
        return self.spec.name

    @property
    def category(self) -> str:
        """组件分类。"""
        return self.spec.category

    def aabb(self, T_component: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """组件在世界位姿下的包围盒。"""
        lows, highs = [], []
        for part in self.parts:
            lo, hi = part.geometry.aabb(part.world_transform(T_component))
            lows.append(lo)
            highs.append(hi)
        return np.min(lows, axis=0), np.max(highs, axis=0)

    def __repr__(self) -> str:
        return f"Component({self.id}, {len(self.parts)} parts)"


# --------------------------------------------------------------------------- #
# 零件快捷构造
# --------------------------------------------------------------------------- #
def _box(
    name: str, size: Sequence[float], xyz: Sequence[float], color: Tuple[float, float, float]
) -> ComponentPart:
    """构造一个长方体零件，中心位于 ``xyz``。"""
    g = BoxGeometry(tuple(float(v) for v in size))
    return ComponentPart(name, g, T_xyz(*(float(v) for v in xyz)), color)


def _cyl(
    name: str,
    radius: float,
    length: float,
    xyz: Sequence[float],
    color: Tuple[float, float, float],
    rpy: Sequence[float] = (0.0, 0.0, 0.0),
) -> ComponentPart:
    """构造一个圆柱零件；默认轴向 +Z，可用 ``rpy`` 摆正。"""
    g = CylinderGeometry(float(radius), float(length))
    return ComponentPart(name, g, T_rpy(rpy, xyz), color)


# --------------------------------------------------------------------------- #
# 各组件工厂
# --------------------------------------------------------------------------- #
def _workbench(p: Dict[str, float]) -> List[ComponentPart]:
    """工作台：台面 + 四腿 + 横撑。原点在台面中心下方的地面上。"""
    w, d, h, t = p["width"], p["depth"], p["height"], p["top_thickness"]
    leg = 0.06
    parts = [
        _box("台面", (w, d, t), (0, 0, h - t / 2), COLOR_DECK),
    ]
    for sx in (-1, 1):
        for sy in (-1, 1):
            parts.append(
                _box(
                    f"桌腿{'前' if sy < 0 else '后'}{'左' if sx < 0 else '右'}",
                    (leg, leg, h - t),
                    (sx * (w / 2 - leg), sy * (d / 2 - leg), (h - t) / 2),
                    COLOR_STEEL,
                )
            )
    # 横撑只沿 X 方向连左右腿，避免四个面都加导致碰撞体冗余
    for sy in (-1, 1):
        parts.append(
            _box("横撑", (w - 2 * leg, 0.04, 0.04), (0, sy * (d / 2 - leg), 0.12), COLOR_STEEL)
        )
    return parts


def _pedestal(p: Dict[str, float]) -> List[ComponentPart]:
    """机器人底座：底板 + 立柱。原点在地面。"""
    w, d, h = p["width"], p["depth"], p["height"]
    return [
        _box("底板", (w, d, 0.03), (0, 0, 0.015), COLOR_STEEL),
        _box("立柱", (w * 0.6, d * 0.6, h - 0.03), (0, 0, 0.03 + (h - 0.03) / 2), COLOR_STEEL),
        _box("顶板", (w * 0.7, d * 0.7, 0.025), (0, 0, h - 0.0125), COLOR_DECK),
    ]


def _fence(p: Dict[str, float]) -> List[ComponentPart]:
    """安全围栏：两端立柱 + 上下横梁 + 网板。原点在底边中点。"""
    w, h = p["width"], p["height"]
    post, thick = 0.06, 0.03
    parts = [
        _box("立柱左", (post, post, h), (-w / 2 + post / 2, 0, h / 2), COLOR_FRAME),
        _box("立柱右", (post, post, h), (w / 2 - post / 2, 0, h / 2), COLOR_FRAME),
        _box("下横梁", (w - post, thick, 0.05), (0, 0, 0.10), COLOR_FRAME),
        _box("上横梁", (w - post, thick, 0.05), (0, 0, h - 0.10), COLOR_FRAME),
        # 网板用薄板近似：真实网孔对碰撞检测无意义，反而会让 GJK 多算一个凸体
        _box("网板", (w - post * 2, thick * 0.5, h - 0.25), (0, 0, h / 2 + 0.025), COLOR_FRAME),
    ]
    return parts


def _conveyor(p: Dict[str, float]) -> List[ComponentPart]:
    """皮带输送机：带体 + 两侧梁 + 支腿。原点在带面中心的地面投影。"""
    L, w, h = p["length"], p["width"], p["height"]
    parts = [
        _box("带体", (L, w, 0.03), (0, 0, h - 0.015), COLOR_BELT),
        _box("侧梁左", (L, 0.05, 0.10), (0, -w / 2 - 0.025, h - 0.065), COLOR_STEEL),
        _box("侧梁右", (L, 0.05, 0.10), (0, w / 2 + 0.025, h - 0.065), COLOR_STEEL),
    ]
    for sx in (-1, 1):
        for sy in (-1, 1):
            parts.append(
                _box(
                    f"支腿{sx}{sy}",
                    (0.05, 0.05, h - 0.12),
                    (sx * (L / 2 - 0.15), sy * (w / 2), (h - 0.12) / 2),
                    COLOR_STEEL,
                )
            )
    return parts


def _turntable(p: Dict[str, float]) -> List[ComponentPart]:
    """单轴回转变位机：底座 + 转盘。原点在转盘上表面的回转中心。"""
    r, h = p["radius"], p["height"]
    return [
        _cyl("底座", r * 1.1, h * 0.85, (0, 0, -h / 2 + 0.02), COLOR_STEEL),
        _cyl("转盘", r, 0.04, (0, 0, -0.02), COLOR_DECK),
    ]


def _l_positioner(p: Dict[str, float]) -> List[ComponentPart]:
    """L 型双轴变位机：底座 + 立柱 + L 臂工作台。原点在回转中心。"""
    r, h, arm = p["radius"], p["height"], p["arm"]
    return [
        _cyl("底座", r * 1.15, 0.08, (0, 0, -h + 0.04), COLOR_STEEL),
        _box("立柱", (0.16, 0.16, h - 0.08), (-r - 0.08, 0, -h / 2), COLOR_STEEL),
        _box("L 臂", (arm, 0.16, 0.10), (arm / 2 - r, 0, -h + 0.05), COLOR_STEEL),
        _cyl("工作台", r, 0.04, (0, 0, -0.02), COLOR_DECK),
    ]


def _workpiece_block(p: Dict[str, float]) -> List[ComponentPart]:
    """方块工件。原点在几何中心。"""
    return [_box("工件本体", (p["length"], p["width"], p["height"]), (0, 0, 0), COLOR_WORK)]


def _workpiece_plate(p: Dict[str, float]) -> List[ComponentPart]:
    """板料工件。原点在几何中心。"""
    return [_box("板料", (p["length"], p["width"], p["thickness"]), (0, 0, 0), COLOR_WORK)]


def _workpiece_bar(p: Dict[str, float]) -> List[ComponentPart]:
    """棒料工件（轴向 +Z）。原点在几何中心。"""
    return [_cyl("棒料", p["radius"], p["length"], (0, 0, 0), COLOR_WORK)]


def _workpiece_pipe(p: Dict[str, float]) -> List[ComponentPart]:
    """管件工件：用实心圆柱近似（GJK 只能处理凸体，内腔无意义）。

    轴向 +Z，原点在几何中心。
    """
    return [_cyl("管件", p["outer_radius"], p["length"], (0, 0, 0), COLOR_WORK)]


def _fixture_plate(p: Dict[str, float]) -> List[ComponentPart]:
    """夹具底板 + 定位块 + 两个压紧块。原点在底板上表面中心。"""
    w, d, t = p["width"], p["depth"], p["thickness"]
    parts = [_box("底板", (w, d, t), (0, 0, -t / 2), COLOR_FIXTURE)]
    parts.append(
        _box("定位块", (0.05, 0.05, 0.04), (-w / 2 + 0.06, -d / 2 + 0.06, 0.02), COLOR_FIXTURE)
    )
    for sy in (-1, 1):
        parts.append(
            _box(
                f"压紧块{'前' if sy < 0 else '后'}",
                (0.08, 0.04, 0.06),
                (w / 2 - 0.10, sy * (d / 2 - 0.05), 0.03),
                COLOR_FIXTURE,
            )
        )
    return parts


def _storage_bin(p: Dict[str, float]) -> List[ComponentPart]:
    """料箱：底板 + 四面侧板（拆成 4 个凸体，保留内腔的凹形状）。"""
    w, d, h, t = p["width"], p["depth"], p["height"], p["wall"]
    parts = [_box("箱底", (w, d, t), (0, 0, t / 2), COLOR_BIN)]
    half_w, half_d = w / 2, d / 2
    specs = [
        ("侧板前", (w, t, h), (0, -half_d + t / 2, h / 2)),
        ("侧板后", (w, t, h), (0, half_d - t / 2, h / 2)),
        ("侧板左", (t, d - 2 * t, h), (-half_w + t / 2, 0, h / 2)),
        ("侧板右", (t, d - 2 * t, h), (half_w - t / 2, 0, h / 2)),
    ]
    for name, size, xyz in specs:
        parts.append(_box(name, size, xyz, COLOR_BIN))
    return parts


def _pallet(p: Dict[str, float]) -> List[ComponentPart]:
    """托盘：面板 + 三条纵梁。原点在面板上表面中心。"""
    w, d, h = p["width"], p["depth"], p["height"]
    parts = [_box("面板", (w, d, 0.02), (0, 0, -0.01), COLOR_BIN)]
    for sx in (-1, 0, 1):
        parts.append(
            _box(f"纵梁{sx}", (0.10, d, h - 0.02), (sx * (w / 2 - 0.10), 0, -0.02 - (h - 0.02) / 2), COLOR_BIN)
        )
    return parts


def _tool_gripper(p: Dict[str, float]) -> List[ComponentPart]:
    """两指平行夹爪：法兰 + 缸体 + 两指。原点在法兰面（+Z 朝外）。"""
    stroke, finger = p["stroke"], p["finger_length"]
    return [
        _cyl("法兰", 0.04, 0.012, (0, 0, 0.006), COLOR_TOOL),
        _box("缸体", (0.08, 0.05, 0.06), (0, 0, 0.042), COLOR_TOOL),
        _box("指左", (0.02, 0.015, finger), (0, -stroke / 2 - 0.0075, 0.072 + finger / 2), COLOR_TOOL),
        _box("指右", (0.02, 0.015, finger), (0, stroke / 2 + 0.0075, 0.072 + finger / 2), COLOR_TOOL),
    ]


def _tool_suction(p: Dict[str, float]) -> List[ComponentPart]:
    """真空吸盘：法兰 + 连杆 + 吸盘。原点在法兰面。"""
    length = p["length"]
    return [
        _cyl("法兰", 0.04, 0.012, (0, 0, 0.006), COLOR_TOOL),
        _cyl("连杆", 0.012, length, (0, 0, 0.012 + length / 2), COLOR_TOOL),
        _cyl("吸盘", p["cup_radius"], 0.02, (0, 0, 0.012 + length + 0.01), COLOR_TOOL),
    ]


def _tool_torch(p: Dict[str, float]) -> List[ComponentPart]:
    """焊枪：法兰 + 颈部（斜置）+ 导电嘴。原点在法兰面。"""
    length, tilt = p["length"], p["tilt"]
    # 绕 Y 轴倾斜，使焊丝指向斜前下方
    R = T_rpy((0.0, float(tilt), 0.0), (0.0, 0.0, 0.0))
    neck = CylinderGeometry(0.018, float(length))
    neck_mid = np.eye(4)
    neck_mid[:3, 3] = (0.0, 0.0, 0.012 + length / 2)
    tip_pos = R @ np.array([0.0, 0.0, 0.012 + length, 1.0])
    return [
        _cyl("法兰", 0.04, 0.012, (0, 0, 0.006), COLOR_TOOL),
        ComponentPart("枪颈", neck, R @ neck_mid, COLOR_TOOL),
        ComponentPart(
            "导电嘴",
            CylinderGeometry(0.008, 0.03),
            R @ T_xyz(0, 0, 0.012 + length + 0.015),
            COLOR_TOOL,
        ),
        ComponentPart(
            "TCP 标记",
            SphereGeometry(0.006),
            T_xyz(float(tip_pos[0]), float(tip_pos[1]), float(tip_pos[2]) + 0.03),
            (1.0, 0.85, 0.2),
        ),
    ]


# --------------------------------------------------------------------------- #
# 注册表
# --------------------------------------------------------------------------- #
def _specs() -> Tuple[ComponentSpec, ...]:
    """组件定义清单。"""
    return (
        ComponentSpec(
            "workbench", "工作台", "structure", "通用装配/焊接工作台，台面高度可调",
            (
                ComponentParam("width", "台面宽", 1.2, 0.3, 4.0),
                ComponentParam("depth", "台面深", 0.8, 0.3, 3.0),
                ComponentParam("height", "台面高", 0.75, 0.2, 1.5),
                ComponentParam("top_thickness", "台面厚", 0.04, 0.01, 0.2),
            ),
            _workbench,
        ),
        ComponentSpec(
            "pedestal", "机器人底座", "structure", "抬高机器人安装面的钢制底座",
            (
                ComponentParam("width", "底板宽", 0.6, 0.2, 2.0),
                ComponentParam("depth", "底板深", 0.6, 0.2, 2.0),
                ComponentParam("height", "总高", 0.4, 0.05, 2.0),
            ),
            _pedestal,
        ),
        ComponentSpec(
            "fence", "安全围栏", "safety", "隔离机器人作业区的护栏",
            (
                ComponentParam("width", "宽度", 1.5, 0.3, 6.0),
                ComponentParam("height", "高度", 1.8, 0.5, 3.0),
            ),
            _fence,
        ),
        ComponentSpec(
            "conveyor", "皮带输送机", "transport", "上下料用皮带线",
            (
                ComponentParam("length", "长度", 2.0, 0.4, 10.0),
                ComponentParam("width", "带宽", 0.4, 0.1, 2.0),
                ComponentParam("height", "带面高", 0.8, 0.2, 1.5),
            ),
            _conveyor,
        ),
        ComponentSpec(
            "turntable", "单轴回转台", "positioner", "绕竖直轴回转的变位机",
            (
                ComponentParam("radius", "台面半径", 0.4, 0.1, 1.5),
                ComponentParam("height", "台面高", 0.6, 0.2, 1.5),
            ),
            _turntable,
        ),
        ComponentSpec(
            "l_positioner", "L 型双轴变位机", "positioner", "回转 + 翻转双轴，焊接常用",
            (
                ComponentParam("radius", "台面半径", 0.35, 0.1, 1.2),
                ComponentParam("height", "回转中心高", 0.7, 0.2, 1.8),
                ComponentParam("arm", "L 臂长", 0.6, 0.2, 2.0),
            ),
            _l_positioner,
        ),
        ComponentSpec(
            "workpiece_block", "方块工件", "workpiece", "长方体工件",
            (
                ComponentParam("length", "长", 0.2, 0.01, 3.0),
                ComponentParam("width", "宽", 0.15, 0.01, 3.0),
                ComponentParam("height", "高", 0.1, 0.005, 3.0),
            ),
            _workpiece_block,
        ),
        ComponentSpec(
            "workpiece_plate", "板料工件", "workpiece", "薄板工件",
            (
                ComponentParam("length", "长", 0.5, 0.01, 4.0),
                ComponentParam("width", "宽", 0.3, 0.01, 4.0),
                ComponentParam("thickness", "厚", 0.003, 0.0005, 0.05),
            ),
            _workpiece_plate,
        ),
        ComponentSpec(
            "workpiece_bar", "棒料工件", "workpiece", "圆柱棒料",
            (
                ComponentParam("radius", "半径", 0.025, 0.002, 0.5),
                ComponentParam("length", "长度", 0.3, 0.01, 3.0),
            ),
            _workpiece_bar,
        ),
        ComponentSpec(
            "workpiece_pipe", "管件工件", "workpiece", "管件（内腔按实心近似）",
            (
                ComponentParam("outer_radius", "外径", 0.05, 0.005, 0.6),
                ComponentParam("length", "长度", 0.5, 0.01, 6.0),
            ),
            _workpiece_pipe,
        ),
        ComponentSpec(
            "fixture", "夹具", "fixture", "底板 + 定位块 + 压紧块",
            (
                ComponentParam("width", "底板宽", 0.3, 0.05, 2.0),
                ComponentParam("depth", "底板深", 0.25, 0.05, 2.0),
                ComponentParam("thickness", "底板厚", 0.02, 0.005, 0.1),
            ),
            _fixture_plate,
        ),
        ComponentSpec(
            "bin", "料箱", "storage", "四面侧板 + 箱底，用于取放料",
            (
                ComponentParam("width", "内宽", 0.4, 0.1, 2.0),
                ComponentParam("depth", "内深", 0.3, 0.1, 2.0),
                ComponentParam("height", "内高", 0.2, 0.05, 1.0),
                ComponentParam("wall", "壁厚", 0.02, 0.005, 0.1),
            ),
            _storage_bin,
        ),
        ComponentSpec(
            "pallet", "托盘", "storage", "码垛托盘",
            (
                ComponentParam("width", "宽", 1.0, 0.2, 2.0),
                ComponentParam("depth", "深", 1.0, 0.2, 2.0),
                ComponentParam("height", "高", 0.15, 0.05, 0.4),
            ),
            _pallet,
        ),
        ComponentSpec(
            "gripper", "两指夹爪", "tool", "平行气爪，安装在机器人法兰",
            (
                ComponentParam("stroke", "开度", 0.06, 0.01, 0.3),
                ComponentParam("finger_length", "指长", 0.08, 0.02, 0.4),
            ),
            _tool_gripper,
        ),
        ComponentSpec(
            "suction", "真空吸盘", "tool", "单杯真空吸盘",
            (
                ComponentParam("length", "连杆长", 0.1, 0.02, 0.5),
                ComponentParam("cup_radius", "吸盘半径", 0.04, 0.01, 0.2),
            ),
            _tool_suction,
        ),
        ComponentSpec(
            "torch", "焊枪", "tool", "弧焊焊枪，枪颈角度可调",
            (
                ComponentParam("length", "枪颈长", 0.18, 0.05, 0.5),
                ComponentParam("tilt", "倾角(rad)", 0.6, 0.0, 1.4),
            ),
            _tool_torch,
        ),
    )


class ComponentRegistry:
    """组件库注册表。

    用法::

        from orolp.environment.components import get_component
        bench = get_component("workbench", width=1.6, depth=1.0)
    """

    def __init__(self) -> None:
        self._specs: Dict[str, ComponentSpec] = {s.id: s for s in _specs()}

    def specs(self) -> List[ComponentSpec]:
        """按注册顺序返回全部组件定义。"""
        return list(_specs())

    def categories(self) -> List[str]:
        """返回出现过的全部分类名（按注册顺序去重）。"""
        seen: List[str] = []
        for s in _specs():
            if s.category not in seen:
                seen.append(s.category)
        return seen

    def get(self, component_id: str) -> ComponentSpec:
        """按 id 取组件定义。

        Raises:
            KeyError: id 不存在
        """
        try:
            return self._specs[component_id]
        except KeyError:
            raise KeyError(
                f"未知组件 '{component_id}'，可用：{sorted(self._specs)}"
            ) from None

    def build(self, component_id: str, **params: float) -> Component:
        """按 id 与参数实例化组件。"""
        return self.get(component_id).build(**params)

    def __len__(self) -> int:
        return len(self._specs)

    def __repr__(self) -> str:
        return f"ComponentRegistry({len(self._specs)} specs)"


_REGISTRY = ComponentRegistry()

CATEGORY_LABELS: Dict[str, str] = {
    "structure": "结构件",
    "safety": "安全设施",
    "transport": "输送设备",
    "positioner": "变位机",
    "workpiece": "工件",
    "fixture": "夹具",
    "storage": "储运",
    "tool": "末端工具",
}


def list_components(category: str | None = None) -> List[ComponentSpec]:
    """列出组件定义。

    Args:
        category: 仅返回该分类；``None`` 返回全部
    """
    specs = _REGISTRY.specs()
    if category is None:
        return specs
    return [s for s in specs if s.category == category]


def get_component(component_id: str, **params: float) -> Component:
    """按 id 实例化组件（参数缺省用默认值）。

    Args:
        component_id: 组件 id，如 ``"workbench"``
        **params: 覆盖默认尺寸的参数

    Returns:
        :class:`Component`
    """
    return _REGISTRY.build(component_id, **params)
