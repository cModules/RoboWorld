"""工程模板（v1.3.6 首版，v1.3.8 扩容）：「新建一个活」不该从空白开始。

v1.3.4 把一次会话收进 ``.orolp``、v1.3.5 解决了「再找到它」，但
`文件 → 打开工程` 面前永远是一张白纸——每次都要从空白工程开始重走
「载机器人 → 摆工件 → 示教」。本模块补的是这个缝：**从模板新建工程**。

模板 = 一份**现场就能用的**工程起点：

* ``grasping_workstation``（抓取工作站）：场景（UR5e + 吸盘 + 输送带 +
  工件 + 托盘）+ **完整示例示教程序**（8 点五点式抓取循环，含抓取/放置
  语义）——由既有 :func:`orolp.world.templates.build_template` 与
  :func:`orolp.world.demo.demo_pick_place_points` 真实构建，不是另写一份
  假数据；
* ``sorting_station``（分拣工位）/ ``machine_tending``（机床上下料）/
  ``3c_assembly``（3C 装配工位，v1.3.8）/ ``auto_welding``（汽车焊装
  工位，v1.3.8）：场景 + Home 位起步程序（一条 ``movej``，语义 VIA）——
  程序得用户自己示，但机器人、工件、围栏已经摆好了。

三条规矩：

1. **模板产物就是普通工程**：`Project` 对象，`save()` 落盘后与手工保存
   的工程毫无区别（GUI 恢复、CLI 摘要、未知段保真全套契约照用）。
2. **只收模板真给的东西**：没有时序/点表就不写那段——「工程里没有：
   工艺时序」比「工艺时序：0 个工步」诚实。
3. **每次全新构建**：`build_project_template` 每次返回独立对象，两个
   「从同一模板新建」的工程互不共享任何状态。
"""

from __future__ import annotations

from typing import Dict, List, Optional, Union
from pathlib import Path

from .i18n import tr
from .project import Project

__all__ = [
    "list_project_templates",
    "build_project_template",
    "save_project_template",
]


def _repo_root() -> Path:
    from .world.templates import repo_root

    return repo_root()


def _resolve_urdf(urdf: str) -> str:
    """模板场景里的 URDF 可能是仓库相对路径，按仓库根解析。"""
    from pathlib import Path as _P

    if urdf and not _P(urdf).exists():
        candidate = _repo_root() / urdf
        if candidate.exists():
            return str(candidate)
    return urdf


def _starter_program(home_joints: List[float], joint_names: Optional[List[str]]) -> Dict[str, object]:
    """起步程序：注释头 + 一条 Home 位 ``movej``（语义 VIA）。"""
    code = "# OLP Path Code\n"
    types: List[str] = []
    if home_joints:
        code += "movej([{}], vel=80)\n".format(", ".join(f"{v:.3f}" for v in home_joints))
        types = ["via"]
    return {
        "code": code,
        "point_types": types,
        "joint_names": list(joint_names or []),
    }


def _build_grasping_workstation() -> Project:
    """抓取工作站：场景 + 完整示例示教程序（真实 IK，与一键演示同一数据源）。"""
    from . import Robot
    from .world.demo import demo_pick_place_points
    from .world.templates import build_template

    cell = build_template("grasping_workstation")
    urdf = _resolve_urdf(cell.scene.robot_urdf)
    robot = Robot(urdf, "UR5e")
    points_deg, types = demo_pick_place_points(cell, robot)
    code = "# OLP Path Code\n" + "".join(
        "movej([{}], vel=80)\n".format(", ".join(f"{v:.3f}" for v in q))
        for q in points_deg
    )

    project = Project(
        name=cell.name,
        note="由工程模板生成：场景与示教程序可直接使用，按需修改后保存",
    )
    project.set("scene", cell.scene.to_dict())
    project.set(
        "program",
        {
            "code": code,
            "point_types": [getattr(t, "value", str(t)) for t in types],
            "joint_names": list(robot.joint_names),
        },
    )
    return project


def _build_scene_template(template_id: str) -> Project:
    """分拣工位 / 机床上下料：场景 + Home 位起步程序（程序得用户自己示）。"""
    from .world.templates import build_template

    cell = build_template(template_id)
    project = Project(
        name=cell.name,
        note="由工程模板生成：机器人与工件已摆好，示教程序从 Home 位起步",
    )
    project.set("scene", cell.scene.to_dict())
    project.set("program", _starter_program(list(cell.home_joints), None))
    return project


_BUILDERS = {
    "grasping_workstation": _build_grasping_workstation,
    "sorting_station": lambda: _build_scene_template("sorting_station"),
    "machine_tending": lambda: _build_scene_template("machine_tending"),
    "3c_assembly": lambda: _build_scene_template("3c_assembly"),
    "auto_welding": lambda: _build_scene_template("auto_welding"),
}


def list_project_templates() -> List[Dict[str, str]]:
    """列出全部内置工程模板（id / 名称 / 描述），供菜单与对话框展示。"""
    names = {
        "grasping_workstation": tr("抓取工作站"),
        "sorting_station": tr("分拣工位"),
        "machine_tending": tr("机床上下料"),
        "3c_assembly": tr("3C 装配工位"),
        "auto_welding": tr("汽车焊装工位"),
    }
    descriptions = {
        "grasping_workstation": tr(
            "UR5e + 吸盘 + 输送带 + 工件 + 托盘，附 8 点抓取循环示例程序（含抓取/放置语义）"
        ),
        "sorting_station": tr(
            "输送带 + 双料箱分拣 + 围栏，机器人与工件已摆好，示教程序从 Home 位起步"
        ),
        "machine_tending": tr(
            "输送带 + 台钳工位 + 机床柜体 + 成品料箱，机器人与工件已摆好，示教程序从 Home 位起步"
        ),
        "3c_assembly": tr(
            "上料托盘（3 块板件）+ 装配治具 + 成品托盘 + 围栏，机器人与工件已摆好，示教程序从 Home 位起步"
        ),
        "auto_welding": tr(
            "焊接变位机 + 输送辊道 + 控制柜 + 围栏，机器人与工件已摆好，示教程序从 Home 位起步"
        ),
    }
    return [
        {"id": tid, "name": names[tid], "description": descriptions[tid]}
        for tid in _BUILDERS
    ]


def build_project_template(template_id: str) -> Project:
    """按 id 构建一份全新的模板工程（每次调用返回独立对象）。

    Raises:
        KeyError: 模板 id 不存在
        Exception: 模板构建失败（IK 不可达、模型缺失等）——调用方如实
            报错，不做静默降级
    """
    try:
        builder = _BUILDERS[template_id]
    except KeyError:
        raise KeyError(
            tr("未知工程模板：{tid}（可用：{opts}）").format(
                tid=template_id, opts="/".join(_BUILDERS)
            )
        ) from None
    return builder()


def save_project_template(template_id: str, path: Union[str, Path]) -> Path:
    """构建模板工程并落盘（原子写入），返回最终路径。"""
    project = build_project_template(template_id)
    return project.save(path)
