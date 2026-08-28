"""环境建模：工作单元场景、参数化组件与碰撞检测。

本包提供对标 Visual Components eCatalog / RoboDK 工作单元的能力，但走的是
**参数化生成**路线而非模型资产路线：用户填工程参数实时生成几何，而不是从
上千个预制模型里挑。

主要入口::

    from orolp.world import Scene, COMPONENTS

    scene = Scene()
    scene.set_robot("models/kuka/kr16_2/kr16_2.urdf")
    scene.add("worktable", pose=Pose.from_xyz_rpy(900, 0, 0, 0, 0, 0))
    scene.add("workpiece_box", pose=Pose.from_xyz_rpy(900, 0, 850, 0, 0, 0))
    hits = scene.check_collisions()
"""

from .geometry import (
    AABB,
    Box,
    Cylinder,
    Primitive,
    Sphere,
    collides,
    prim_aabb,
    sat_axes,
    vertices_of,
)
from .components import (
    COMPONENTS,
    ComponentSpec,
    ParamSpec,
    Part,
    build_component,
    categories,
    default_params,
    get_component,
)
from .scene import (
    Collision,
    Scene,
    SceneObject,
    load_robot_visuals,
)

__all__ = [
    # geometry
    "Box",
    "Cylinder",
    "Sphere",
    "Primitive",
    "AABB",
    "collides",
    "prim_aabb",
    "sat_axes",
    "vertices_of",
    # components
    "COMPONENTS",
    "ComponentSpec",
    "ParamSpec",
    "Part",
    "build_component",
    "categories",
    "default_params",
    "get_component",
    # scene
    "Scene",
    "SceneObject",
    "Collision",
    "load_robot_visuals",
]
