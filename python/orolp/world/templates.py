"""内置工作单元场景模板——一键搭好一个"拿得出手"的工作站。

为什么要模板
------------
场景（:class:`~orolp.world.scene.Scene`）只负责"有哪些对象、摆在哪"，
但要搭一个能直接演示/验收的工作站，还需要一批**工程语义信息**：
工具挂在哪、TCP 在法兰下方多少毫米、哪些工件可以被抓取、落件区是哪个。

这些信息不属于 Scene 的职责，也不适合塞进 Scene JSON（那是通用交换格式）。
模板把它们作为**附加元数据**与场景一起保存：JSON 里场景字段原样兼容
:meth:`Scene.load`，模板信息放在 ``"template"`` 键下，两边互不干扰。

使用::

    from orolp.world import build_grasping_workstation

    cell = build_grasping_workstation()
    scene = cell.scene          # 直接进 GUI / 仿真
    robot.tool = cell.tcp_flange  # 程序生成用 TCP

单位：毫米（与 Scene / Pose 一致）。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from ..pose import Pose
from .components import build_component
from .scene import ROBOT_NAME, Scene

__all__ = [
    "WorkcellTemplate",
    "GRASPING_WORKSTATION_ID",
    "build_grasping_workstation",
    "list_templates",
    "build_template",
    "load_workcell",
]

#: 仓库根目录（python/orolp/world/templates.py 向上三级）
REPO_ROOT = Path(__file__).resolve().parents[3]

#: 抓取工作站模板 id
GRASPING_WORKSTATION_ID = "grasping_workstation"


# --------------------------------------------------------------------------- #
# 模板数据结构
# --------------------------------------------------------------------------- #


@dataclass
class WorkcellTemplate:
    """一个工作单元模板：场景 + 工程语义元数据。

    Attributes:
        id: 模板唯一 id
        name: 中文名
        description: 一句话说明
        scene: 场景（含机器人）
        tool: 挂在法兰上的工具对象名；``None`` 表示无工具
        tcp_flange: TCP 相对**法兰**的位姿；``None`` 表示未定义
        home_joints: 加载时的机器人关节角（度）
        grasp_objects: 可被抓取的工件对象名列表
        place_target: 落件区对象名（如托盘）；``None`` 表示未定义
    """

    id: str
    name: str
    description: str
    scene: Scene
    tool: Optional[str] = None
    tcp_flange: Optional[Pose] = None
    home_joints: List[float] = field(default_factory=list)
    grasp_objects: List[str] = field(default_factory=list)
    place_target: Optional[str] = None

    # --------------------------- 持久化 --------------------------- #

    def to_dict(self) -> Dict[str, Any]:
        """序列化：场景字段 + ``template`` 元数据键（Scene.load 可直接忽略后者）。"""
        data = self.scene.to_dict()
        data["template"] = {
            "id": self.id,
            "name": self.name,
            "description": self.description,
            "tool": self.tool,
            "tcp_flange": (
                self.tcp_flange.matrix.flatten().tolist() if self.tcp_flange else None
            ),
            "home_joints": list(self.home_joints),
            "grasp_objects": list(self.grasp_objects),
            "place_target": self.place_target,
        }
        return data

    def save(self, path: str | Path) -> None:
        """保存为 JSON 文件。

        Args:
            path: 目标路径

        Raises:
            OSError: 写入失败
        """
        Path(path).write_text(
            json.dumps(self.to_dict(), ensure_ascii=False, indent=2), encoding="utf-8"
        )


# --------------------------------------------------------------------------- #
# 模板注册表
# --------------------------------------------------------------------------- #

def list_templates() -> List[Dict[str, str]]:
    """列出全部内置模板（id / 名称 / 描述）。"""
    out: List[Dict[str, str]] = []
    for tid, builder in _TEMPLATES.items():
        probe = builder()
        out.append({"id": tid, "name": probe.name, "description": probe.description})
    return out


def build_template(template_id: str) -> WorkcellTemplate:
    """按 id 构建一个模板实例（每次返回全新场景）。

    Args:
        template_id: 模板 id

    Returns:
        模板实例

    Raises:
        KeyError: 模板 id 不存在
    """
    return _TEMPLATES[template_id]()


def load_workcell(path: str | Path) -> WorkcellTemplate:
    """从 JSON 文件加载工作单元（场景 + 模板元数据）。

    文件由 :meth:`WorkcellTemplate.save` 生成；缺少 ``template`` 键的纯场景
    文件也能加载，元数据字段退化为空。

    Args:
        path: JSON 路径

    Returns:
        模板实例（``scene`` 已含机器人）

    Raises:
        OSError: 读取失败
        json.JSONDecodeError: 非法 JSON
    """
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    scene = _scene_from_dict(data)
    meta = data.get("template") or {}
    tcp = meta.get("tcp_flange")
    return WorkcellTemplate(
        id=meta.get("id", Path(path).stem),
        name=meta.get("name", Path(path).stem),
        description=meta.get("description", ""),
        scene=scene,
        tool=meta.get("tool"),
        tcp_flange=(
            Pose(np.array(tcp, dtype=np.float64).reshape(4, 4)) if tcp else None
        ),
        home_joints=list(meta.get("home_joints") or []),
        grasp_objects=list(meta.get("grasp_objects") or []),
        place_target=meta.get("place_target"),
    )


def _scene_from_dict(data: Dict[str, Any]) -> Scene:
    """恢复场景，并兼容"JSON 里存的是仓库相对 URDF 路径、CWD 又不在仓库根"的情况。"""
    urdf = data.get("robot_urdf")
    if urdf and not Path(urdf).exists() and (REPO_ROOT / urdf).exists():
        data = dict(data)
        data["robot_urdf"] = str(REPO_ROOT / urdf)
    scene = Scene.from_dict(data)
    # 保存时把 URDF 路径归一化回仓库相对路径，保证 JSON 可移植
    if scene.robot_urdf:
        try:
            scene.robot_urdf = str(
                Path(scene.robot_urdf).resolve().relative_to(REPO_ROOT)
            )
        except ValueError:
            pass  # URDF 不在仓库内（用户自定义），保留绝对路径
    return scene


# --------------------------------------------------------------------------- #
# 抓取工作站
# --------------------------------------------------------------------------- #

#: UR5e 程序生成模型（仓库相对路径，JSON 里也用它保持可移植）
_UR5E_URDF = "models/ur/ur5e/ur5e.urdf"

#: 首载关节角（度）——肩部抬起、肘部立起的常规待机位形
_HOME_JOINTS = [0.0, -90.0, 90.0, -90.0, 90.0, 0.0]

#: 吸盘工具在法兰上的安装高度：缸体贴合法兰面
_TOOL_MOUNT_Z = 60.0

#: TCP 到法兰面的距离 = 安装高度 + 吸盘杯口外伸 17mm
_TCP_FLANGE_Z = _TOOL_MOUNT_Z + 17.0

#: 输送带几何（与工作位姿计算强相关，集中定义）。
#: 高度与距离经过**可达性校准**：仓库内 UR5e 模型（连杆 0.391/0.306，两段
#: 求和 + 51mm 腕部，工具下探时 TCP 可达域满足
#: ``sqrt(r² + (z_TCP-42)²) ≤ 748mm``，r 为水平半径）。早期版本
#: y=-600 / h=700 时工件顶面 828mm、距 j2 953mm，IK 无解（2026-09-13
#: S4 落地时用全局优化证实为几何不可达，并非求解器局部极小），据此把
#: 输送带降高拉近、工件队列内移。
_CONVEYOR_Y = -400.0
_CONVEYOR_HEIGHT = 200.0
_BELT_TOP_Z = _CONVEYOR_HEIGHT + 66.0  # 皮带面顶（组件几何决定）

#: 4 个工件的队列位（沿输送带 X 排开；间距 110mm > 工件长度 90mm）。
#: 最远工件 x=390 时抓取/接近/提升位姿对 j2 的距离余量 ≥27mm。
_WORKPIECE_XS = [60.0, 170.0, 280.0, 390.0]
_WORKPIECE_GAP_Z = 2.0  # 与皮带面留 2mm 间隙，避免"贴面误报干涉"

#: 工件高度（mm）。工件顶面 Z = 皮带面顶 + 间隙 + 本值——抓取 TCP 的
#: 目标高度依赖它，demo 示例程序与模板共用同一份定义，避免两处漂移。
WORKPIECE_HEIGHT_MM = 60.0


def _tool_mount_pose() -> Pose:
    """吸盘工具挂到法兰的局部位姿。

    组件局部坐标里吸盘朝 -Z（缸体在 +Z 侧），而法兰 +Z 是外伸方向，
    所以绕 X 翻转 180° 让杯口朝外，再抬到法兰面。
    """
    m = np.eye(4)
    m[:3, :3] = np.diag([1.0, -1.0, -1.0])  # 绕 X 轴旋转 180°
    m[:3, 3] = [0.0, 0.0, _TOOL_MOUNT_Z]
    return Pose(m)


def build_grasping_workstation() -> WorkcellTemplate:
    """构建标准抓取工作站。

    布局（俯视，机器人基座在原点）::

                  +-----------+
                  |  料箱     |   +-----------+
                  +-----------+   | 托盘(落件) |
        +==输送带（4 工件排队）==+  +-----------+
                  [UR5e 基座]

    包含：UR5e 机器人 + 真空吸盘工具（含 TCP 定义）+ 输送带 + 料箱 +
    空托盘 + 4 个矩形工件（输送带上排队，全部可达）。

    Returns:
        :class:`WorkcellTemplate`——每次调用返回全新实例
    """
    scene = Scene()

    urdf = _UR5E_URDF
    if not Path(urdf).exists():
        urdf = str(REPO_ROOT / _UR5E_URDF)
    scene.set_robot(urdf, _HOME_JOINTS)

    # 工具挂法兰
    scene.add(
        "tool_suction",
        pose=_tool_mount_pose(),
        name="吸盘工具",
        parent=ROBOT_NAME,
    )

    # 输送带（皮带沿 X，位于机器人正前方）
    scene.add(
        "conveyor",
        params={"length": 2400.0, "width": 600.0, "height": _CONVEYOR_HEIGHT},
        pose=Pose.from_xyz_rpy(0.0, _CONVEYOR_Y, 0.0, 0, 0, 0),
        name="输送带",
    )

    # 4 个待抓工件：落在皮带面上方 2mm（90×90×60，间距 110mm 不重叠）
    workpieces: List[str] = []
    for i, x in enumerate(_WORKPIECE_XS, start=1):
        scene.add(
            "workpiece_box",
            params={
                "length": 90.0,
                "width": 90.0,
                "height": WORKPIECE_HEIGHT_MM,
            },
            pose=Pose.from_xyz_rpy(x, _CONVEYOR_Y, _BELT_TOP_Z + _WORKPIECE_GAP_Z, 0, 0, 0),
            name=f"工件_{i}",
        )
        workpieces.append(f"工件_{i}")

    # 料箱（左后方）与空托盘（右后方，落件区）。
    # 托盘位 (350, 450) 同样经过可达性校准：落件及其上方接近位姿的
    # 距离余量 ≥81mm（原 (450, 550) 处 r=711mm，已在可达边界上）。
    scene.add(
        "material_bin",
        params={"length": 600.0, "width": 400.0, "height": 300.0},
        pose=Pose.from_xyz_rpy(-500.0, 550.0, 0.0, 0, 0, 0),
        name="料箱",
    )
    scene.add(
        "pallet_base",
        params={"length": 900.0, "width": 700.0, "height": 144.0},
        pose=Pose.from_xyz_rpy(350.0, 450.0, 0.0, 0, 0, 0),
        name="托盘",
    )

    return WorkcellTemplate(
        id=GRASPING_WORKSTATION_ID,
        name="抓取工作站",
        description="UR5e + 真空吸盘：输送带上料、托盘码放，含 TCP 定义与 4 工件",
        scene=scene,
        tool="吸盘工具",
        tcp_flange=Pose.from_xyz_rpy(0.0, 0.0, _TCP_FLANGE_Z, 0, 0, 0),
        home_joints=list(_HOME_JOINTS),
        grasp_objects=workpieces,
        place_target="托盘",
    )


# --------------------------------------------------------------------------- #
# 模板注册表（在所有 builder 定义之后）
# --------------------------------------------------------------------------- #

_TEMPLATES = {
    GRASPING_WORKSTATION_ID: build_grasping_workstation,
}


def _self_check() -> None:  # pragma: no cover - 开发期快速自检入口
    """CLI 自检：构建模板并打印关键信息。"""
    cell = build_grasping_workstation()
    scene = cell.scene
    print(f"模板: {cell.id} — {cell.name}")
    print(f"机器人: {scene.robot_urdf}")
    print(f"对象: {list(scene.objects)}")
    print(f"TCP(法兰系): {cell.tcp_flange.to_xyz_rpy()}")
    tool_tf = scene.world_transform(cell.tool)
    print(f"工具世界位置: {np.round(tool_tf[:3, 3], 1)}")
    print(f"干涉检查: {len(scene.check_collisions())} 处")


if __name__ == "__main__":  # pragma: no cover
    # 运行方式：python -m orolp.world.templates [输出路径|check]
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "check":
        _self_check()
    else:
        out = sys.argv[1] if len(sys.argv) > 1 else "examples/assets/grasping_workstation.json"
        Path(out).parent.mkdir(parents=True, exist_ok=True)
        cell = build_grasping_workstation()
        cell.save(out)
        print(f"已生成: {out}")
