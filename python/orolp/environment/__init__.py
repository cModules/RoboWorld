"""机器人工作站（Robot Cell）环境建模。

对标 RoboDK 的 Station / Visual Components 的 Layout，提供：

* :mod:`orolp.environment.geometry`  —— 凸体几何图元（碰撞与渲染的统一数据源）
* :mod:`orolp.environment.collision` —— GJK/EPA 精确距离与穿透深度
* :mod:`orolp.environment.components`—— 程序化组件库（工作台/围栏/传送带/变位机/工件…）
* :mod:`orolp.environment.robot_body`—— 从 URDF 抽取机器人连杆碰撞体

单位约定：**内部一律使用米与弧度**。仅在 GUI/文件 IO 边界与
:class:`orolp.pose.Pose`（毫米 + 度）互相转换。

设计取舍：
    RoboDK 支持导入任意 STEP/STL 网格。本项目零外部 CAD 依赖，组件库全部
    程序化生成——好处是体积为零、参数可调、开源无授权风险；代价是无法还原
    真实外形。复杂零件可通过 :class:`geometry.MeshGeometry` 传入三角面顶点，
    GJK 的支撑函数会自动取其凸包。
"""

from .geometry import (
    BoxGeometry,
    ConvexGeometry,
    CylinderGeometry,
    MeshGeometry,
    SphereGeometry,
)
from .collision import Contact, collide, gjk, epa
from .components import (
    Component,
    ComponentPart,
    ComponentRegistry,
    get_component,
    list_components,
)
from .robot_body import LinkBody, RobotBody

__all__ = [
    "BoxGeometry",
    "ConvexGeometry",
    "CylinderGeometry",
    "MeshGeometry",
    "SphereGeometry",
    "Contact",
    "collide",
    "gjk",
    "epa",
    "Component",
    "ComponentPart",
    "ComponentRegistry",
    "get_component",
    "list_components",
    "LinkBody",
    "RobotBody",
]
