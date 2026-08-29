"""渲染层：URDF 解析 + 网格加载 + OpenGL 绘制助手。

本包刻意不依赖 PyOpenGL——:mod:`orolp.render.mesh` 与 :mod:`orolp.render.urdf_model`
只用到 NumPy，可以在无显示环境（CI / 无头服务器）下完整单元测试。
OpenGL 相关的绘制代码保留在 ``olp.py`` 的 ``GLWidget`` 中，消费这里产出的纯数据。
"""

from .mesh import (
    MeshData,
    MeshLoadError,
    compute_normals,
    load_mesh,
    load_obj,
    load_stl,
    weld_vertices,
)
from .urdf_model import (
    Joint,
    JointLimits,
    Link,
    Primitive,
    RobotModel,
    UrdfParseError,
    Visual,
    rpy_to_matrix,
    xyzrpy_to_matrix,
)

__all__ = [
    "MeshData",
    "MeshLoadError",
    "compute_normals",
    "load_mesh",
    "load_obj",
    "load_stl",
    "weld_vertices",
    "Joint",
    "JointLimits",
    "Link",
    "Primitive",
    "RobotModel",
    "UrdfParseError",
    "Visual",
    "rpy_to_matrix",
    "xyzrpy_to_matrix",
]
