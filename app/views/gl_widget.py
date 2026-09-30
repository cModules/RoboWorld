"""三维场景视图（ADR-002 拆解：``GLWidget`` → ``app.views``）。

归属：App / GUI 层 ``app.views``。它是**场景的 OpenGL 视图**——渲染机器人/工件/网格、
相机与投影、鼠标拾取与拖拽、Gizmo 交互、插件覆盖层；几何与 URDF 交给
``app.views.robot_renderer``（在其之上维护旧版字典形态），本类只管画与交互。

依赖：``app → app.views.robot_renderer · domain.world · adapter.plugins · support.i18n``；
**不 import main**（方向单向：入口脚本 import 本视图）。

搬迁纪律：只换位置、不改行为（ADR-002 禁止"顺手重构"）。16 个 ``GL_*`` 常量与
``MIME_TYPE`` 随类一起搬走（AST 核实：只有本类用）。

与 ``main.py`` 的两处**有意差异**（都在降级路径上，方向是安全侧，已记队列）：
① 降级块补齐 ``ROBOT_NAME`` / ``GEOMETRY_COLLISION`` / ``prim_aabb`` 的 ``None`` 兜底
（main.py 漏了，依赖缺失时引用会 NameError）；
② ``import numpy as _np`` 提到模块级（main.py 只在 i18n 兜底分支里定义它）。
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import TYPE_CHECKING, Optional

import numpy as np
import numpy as _np
import OpenGL.GL as gl
from PySide6 import QtCore, QtOpenGLWidgets, QtWidgets
from PySide6.QtCore import Qt

from app.views.robot_renderer import RobotRenderer
from app.logging import logger
from support.i18n import tr

if TYPE_CHECKING:  # 仅供字符串注解（本模块有 from __future__ import annotations）
    from adapter.robot_library.mesh import MeshData

#: 组件库拖拽到视口的 MIME 标识（与 adapter.robot_library.panel.MIME_TYPE 保持一致）
MIME_TYPE = "application/x-machine-item"

# OpenGL 枚举。刻意写死数值：与 PyOpenGL 常量同值，但省掉一次属性查找，
# 且在没有 PyOpenGL 的环境里也能解析（本模块 import gl 仅用于绘制调用）。
GL_COLOR_BUFFER_BIT = 0x00004000
GL_DEPTH_BUFFER_BIT = 0x00000100
GL_TRIANGLES = 0x00000004
GL_QUADS = 0x00000007
GL_QUAD_STRIP = 0x00000008
GL_TRIANGLE_FAN = 0x00000006
GL_LINES = 0x00000001
GL_FRONT_AND_BACK = 0x00000408
GL_LINE = 0x00001B01
GL_FILL = 0x00001B02
GL_LINE_STRIP = 0x00000003
GL_POINTS = 0x00000000
GL_DEPTH_TEST = 0x00000B71
GL_MODELVIEW = 0x00001700
GL_PROJECTION = 0x00001701

#: 曲面细分参数：把圆周按目标弦长切段，再钳到 [16, 72] 段。
#: 视觉上 72 段的圆柱在 4K 屏上已看不出棱角；再往上只是浪费顶点。
#: 目标弦长 6 mm——视口单位是米，因此工业臂最粗的底座（半径约 0.13 m）用满 72 段，
#: 而法兰上的 M6 螺栓（半径约 3 mm）只用 16 段，不会为看不见的细节付费。
_TESSELLATION_CHORD_M = 0.006
_TESSELLATION_MIN_SEGMENTS = 16
_TESSELLATION_MAX_SEGMENTS = 72


def _segments_for_radius(radius: float) -> int:
    """按半径推算圆柱/球需要的周向分段数。

    Args:
        radius: 半径（米）。

    Returns:
        分段数，钳制在 ``[16, 72]`` 区间内。
    """
    if radius <= 0.0:
        return _TESSELLATION_MIN_SEGMENTS
    circumference = 2.0 * math.pi * radius
    segments = int(math.ceil(circumference / _TESSELLATION_CHORD_M))
    return max(_TESSELLATION_MIN_SEGMENTS, min(_TESSELLATION_MAX_SEGMENTS, segments))


# 环境建模（场景 / 组件库 / 干涉检查）——与 main.py 的 _HAS_WORLD 同口径
try:
    from domain.world import placement as _placement
    from domain.world.geometry import prim_aabb
    from domain.world.scene import GEOMETRY_COLLISION, ROBOT_NAME

    _HAS_WORLD = True
except Exception:  # pragma: no cover - 依赖缺失时降级（见模块头"有意差异 ①"）
    _HAS_WORLD = False
    _placement = None
    prim_aabb = None
    ROBOT_NAME = None
    GEOMETRY_COLLISION = None

# 插件接口（渲染覆盖层）——与 main.py 的 _HAS_PLUGINS 同口径
try:
    from adapter.plugins import (
        FrameMarker,
        LineSegments,
        OverlayContext,
        PointCloud,
        collect_overlays,
        default_registry as plugin_registry,
    )

    _HAS_PLUGINS = True
except Exception:  # pragma: no cover - 依赖缺失时降级
    _HAS_PLUGINS = False
    OverlayContext = None
    collect_overlays = None
    plugin_registry = None
    LineSegments = PointCloud = FrameMarker = ()


class GLWidget(QtOpenGLWidgets.QOpenGLWidget):
    objectSelected = QtCore.Signal(str)  # 右键拾取/拖动时发射对象名（未命中不发射）
    componentDropped = QtCore.Signal(str)  # 组件库拖入视口的 component_id
    robotDropped = QtCore.Signal(str)  # 机器库拖入视口的 catalog key
    #: 左键拖拽摆放完成（松手落到支承面），携带状态栏一行小结（已翻译）
    objectPlaced = QtCore.Signal(str)
    #: 视口侧改动了挂载关系（起拖自动解除 / 松手吸附装夹），携带对象名
    sceneChanged = QtCore.Signal(str)
    #: 右键点中**某个已示教路径点** → 把该台机器人驱动到那个示教点。
    #: 参数是（机器人名, 该台点表里的下标）——必须带名字：多机时点属于谁
    #: 由画出来的那一段决定，退回"当前那台"会把 A 的点驱动到 B 身上
    pathPointMoveRequested = QtCore.Signal(str, int)
    #: 右键"算不算拖过"的位移阈值（像素）。触控板两指轻点与鼠标右键都会带
    #: 1~2px 抖动：没有阈值时这些抖动一律被判成"拖过 gizmo/视角"，右键菜单
    #: **永远弹不出来**（2026-09-25 用户反馈：看不到「移动到」）。
    RIGHT_DRAG_SLOP_PX = 4.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self.renderer = RobotRenderer()
        #: 多机器人：每台一个渲染器，键为场景里的机器人名。
        #: ``self.renderer`` 仍是"当前那台"（关节滑杆、记录点位等既有单机器人
        #: 调用面都走它），绘制时按名字从本字典取，互不覆盖。
        self.renderers: dict = {}
        #: 模型不可用的机器人的**专属空渲染器**（键为机器人名）。见
        #: :meth:`renderer_of`——宁可画不出来，也绝不能借用别人的渲染器。
        self._empty_renderers: dict = {}
        #: 当前机器人：关节滑杆与关节同步都作用到它（多机必需）
        self.current_robot: str = ROBOT_NAME
        self.camera_dist = 1.2
        self.camera_az = 45.0
        self.camera_el = 30.0
        self.dragging = False
        self._left_moved = False
        self._left_press_pos = None
        self._right_press_pos = None
        # 工作单元场景（由 MainWindow 注入）；场景内部使用毫米，
        # 而本视口沿用 URDF 的米，绘制时统一乘 MM_TO_M
        self.scene = None
        self.last_pos = None
        #: 多机：示教点按机器人名分桶。``recorded_positions`` 是当前那台的别名，
        #: 形态与从前一致（既有调用面都直接读写它），数据却不再串着用。
        self._recorded_by_robot: dict = {}
        self._recorded_ee_by_robot: dict = {}
        self.show_path = True
        #: 相机注视点（米）。载入机器人后按模型尺度自动对准，否则小协作臂
        #: 只占画面一角、大臂又超出视野。
        self.view_center = (0.0, 0.0, 0.4)

        #: 右键拾取 / 坐标系 gizmo 变换状态
        self.selected_object = None
        self.drag_mode = None  # "xy" / "z" / "rz" / None
        self._drag_anchor = None  # 拖动起始参考量
        self._right_like = False  # Control+左键 视为右键（Mac 触控板兼容）

        #: 左键拖拽摆放（v1.1.7）：按在工件上拖 = 移动，松手自动落到支承面；
        #: 按在空白处拖 = 旋转视角（与既有行为一致）。
        #: ``snap_step_mm > 0`` 时落点 XY 按步长吸附（毫米）。
        self.snap_step_mm = 0.0
        #: 松手时落在机器人法兰此半径内（毫米）即自动装夹到法兰
        self.snap_radius_mm = 60.0
        self._grab_active = False
        self._grab_anchor = None

        #: 插件渲染覆盖层（v1.0.4）：第三方画的东西与它们出的错
        self.show_plugin_overlays = True
        self.plugin_overlay_context = None
        self.plugin_overlays = None
        self.plugin_overlay_errors = []

        # 选中机器人即切换"当前机器人"：关节滑杆与关节同步随之改指向。
        # 挂在内部信号上，左/右键拾取、拖拽起手等所有入口一次覆盖。
        self.objectSelected.connect(self._on_selected_changed)

        self.setAcceptDrops(True)

    def _on_selected_changed(self, name) -> None:
        """选中的是机器人时，把它设为"当前机器人"（多机下的关节归属）。"""
        if name and self.scene is not None and self.scene.is_robot(name):
            self.set_current_robot(name)

    def load_robot(self, path, name: str = ROBOT_NAME):
        """装载渲染模型；``name`` 指定登记在哪台机器人名下（多机）。"""
        renderer = self.renderers.get(name)
        if renderer is None:
            renderer = RobotRenderer()
            self.renderers[name] = renderer
        success = renderer.load_urdf(path)
        if success:
            # 当前机器人切换为新装载这台：关节滑杆、记录点位、关节同步都跟它走
            self.current_robot = name
            self.renderer = renderer
            self.frame_robot(name)
        self.update()
        return success

    def renderer_of(self, name: str = ROBOT_NAME):
        """取某台机器人的渲染器。

        未登记时**按该机器人在场景里的 URDF 懒加载一个**，并把它的关节对齐到
        该机器人的**场景关节**。两条都不能省：

        * 直接回退到"当前渲染器"会让它借用**别的机器人**的模型——于是拖关节
          时两台一起动（共用同一份 joint_values）；
        * 新建的渲染器默认是 HOME 待机位，而场景里那台有自己的关节值——不对齐
          就会"画出来的手臂"与"场景法兰"错位，挂在上面的工具离开显示的法兰。
        """
        renderer = self.renderers.get(name)
        if renderer is not None:
            return renderer
        urdf = None
        if self.scene is not None and name in self.scene.robots:
            urdf = self.scene.robots[name].urdf
        if urdf and Path(urdf).exists():
            renderer = RobotRenderer()
            if renderer.load_urdf(urdf):
                self.renderers[name] = renderer
                state = self.scene.robots.get(name)
                if state is not None:
                    for i, joint_name in enumerate(renderer.actuated_names()):
                        if i < len(state.joints_deg):
                            renderer.joint_values[joint_name] = math.radians(
                                float(state.joints_deg[i])
                            )
                return renderer
        # 走到这里说明这台机器人的模型确实取不到（URDF 丢失、路径相对且已被
        # 移动过、或场景还没登记它）。**绝不退回 ``self.renderer``**：那会让它
        # 借用另一台机器人的模型与 ``joint_values``——于是"拖 A 的关节"连 B 一起
        # 转，而且 B 的位置画出来却是 A 的机身（几何与位形双双张冠李戴）。
        # 画不出来是诚实的，画错了是灾难：给一台专属的空渲染器。
        empty = self._empty_renderers.get(name)
        if empty is None:
            empty = RobotRenderer()
            self._empty_renderers[name] = empty
        return empty

    def set_current_robot(self, name) -> None:
        """切换"当前机器人"：关节滑杆与关节同步都作用到它（多机必需）。"""
        if not name or self.scene is None or name not in self.scene.robots:
            return
        self.current_robot = name
        self.renderer = self.renderer_of(name)

    # ---------------- 拖拽：从机器组件库接收对象 ---------------- #
    def dragEnterEvent(self, event):
        if event.mimeData().hasFormat(MIME_TYPE):
            event.acceptProposedAction()
        else:
            event.ignore()

    def dropEvent(self, event):
        md = event.mimeData()
        if not md.hasFormat(MIME_TYPE):
            event.ignore()
            return
        try:
            data = json.loads(bytes(md.data(MIME_TYPE)).decode("utf-8"))
        except Exception:
            event.ignore()
            return
        kind = data.get("kind")
        if kind == "robot":
            self.robotDropped.emit(data["id"])
        elif kind == "component":
            self.componentDropped.emit(data["id"])
        event.acceptProposedAction()

    def frame_robot(self, robot: str = ROBOT_NAME) -> None:
        """按当前位姿的包围球自动取景（相机距离 + 注视点）。

        机器人尺寸跨度很大：UR3e 臂展 500 mm，IRB 6700 臂展 3200 mm。
        固定相机距离会让小机器人只占画面一角、大机器人直接出画。

        包围球按**每个 visual 的实际几何**算，而不是只取连杆原点——后者会
        低估尺寸（连杆原点只是几个点，几何体本身还有体积），导致大臂贴边出画。
        """
        bounds = self._robot_bounds(robot)
        if bounds is None:
            return
        lo, hi = bounds

        center = np.array(
            [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0],
            dtype=np.float64,
        )
        radius = 0.5 * float(np.linalg.norm(np.asarray(hi) - np.asarray(lo)))
        if radius <= 1e-6:
            radius = 0.1

        # 包围球半径 r 要在 45° 垂直 FOV（半角 22.5°）内完整入画，需
        # dist = r / sin(22.5°)；再留 10% 余量避免贴边。用包围球而非包围盒，
        # 是因为相机会绕模型公转，包围盒在不同方位下投影差异很大。
        self.camera_dist = max(0.3, radius / math.sin(math.radians(22.5)) * 1.10)
        self.view_center = (float(center[0]), float(center[1]), float(center[2]))

    def _robot_base_pose_m(self, robot: str = ROBOT_NAME):
        """机器人在场景中的**世界**基座位姿（视口米制）；无机器人时 ``None``。

        必须用 :meth:`Scene.world_transform` 而不是 ``SceneObject.pose``：机器人
        挂到底座/台面上时世界位姿 = 底座 × 局部位姿，只取局部会让渲染与场景
        数据分家——表现为"画的机器人不跟着底座走、装夹件与手臂对不上"。

        坐标系语义：机器人始终使用**自身坐标系**，基座只是一个把它整体搬到
        某处的父级变换（后续若要支持自定义基座坐标系，改这里即可）。
        """
        if self.scene is None or robot not in self.scene.objects:
            return None
        T = np.asarray(self.scene.world_transform(robot), dtype=np.float64).copy()
        T[:3, 3] *= self.MM_TO_M
        return T

    def _robot_bounds(self, robot: str = ROBOT_NAME):
        """返回**指定机器人**当前位姿下所有 visual 的世界包围盒 ``(lo, hi)``。

        Returns:
            ``(lo, hi)`` 两个长度 3 的数组；无模型时返回 ``None``。
        """
        model = self.renderer_of(robot).model
        poses = self.renderer_of(robot).link_poses()
        if model is None or not poses:
            return None

        lo = np.array([np.inf, np.inf, np.inf], dtype=np.float64)
        hi = np.array([-np.inf, -np.inf, -np.inf], dtype=np.float64)
        found = False

        for link_name in model.link_chain():
            link = model.links.get(link_name)
            pose = poses.get(link_name)
            if link is None or pose is None:
                continue
            for visual in link.visuals:
                origin = (pose @ visual.matrix)[:3, 3]
                radius = self._primitive_radius(visual.primitive)
                lo = np.minimum(lo, origin - radius)
                hi = np.maximum(hi, origin + radius)
                found = True

        if not found:
            return None
        # 基座位姿也要吃进去：否则"重置视图"会把镜头对准原点，而不是对准
        # 挂在底座/台面上的那台机器人
        base = self._robot_base_pose_m(robot)
        if base is not None and not np.allclose(base, np.eye(4)):
            corners = np.array(
                [
                    [x, y, z, 1.0]
                    for x in (lo[0], hi[0])
                    for y in (lo[1], hi[1])
                    for z in (lo[2], hi[2])
                ],
                dtype=np.float64,
            )
            world = (corners @ base.T)[:, :3]
            lo, hi = world.min(axis=0), world.max(axis=0)
        return lo, hi

    @staticmethod
    def _primitive_radius(primitive: object) -> float:
        """求基本体的包围球半径（米），用于快速估算取景范围。"""
        import math as _math

        kind = primitive.kind
        if kind == "box":
            sx, sy, sz = (float(p) for p in primitive.params[:3])
            return 0.5 * _math.sqrt(sx * sx + sy * sy + sz * sz)
        if kind == "cylinder":
            radius, length = (float(p) for p in primitive.params[:2])
            return _math.sqrt(radius * radius + (length / 2.0) ** 2)
        if kind == "sphere":
            return float(primitive.params[0])
        if kind == "mesh":
            mesh = primitive.mesh
            if mesh is not None and not mesh.is_empty:
                extent = mesh.vertices.max(axis=0) - mesh.vertices.min(axis=0)
                return 0.5 * float(np.linalg.norm(extent))
        return 0.0

    # ---------------- 示教点：多机按机器人分桶 ---------------- #
    #
    # 每台机器人各自一串示教点，混在一个列表里就会"给 A 示教的点被算进 B 的
    # 程序"。这里按名字分桶；对外仍保留 ``recorded_positions`` 当前引用形体的
    # plain 属性用法（路径绘制、代码编辑器同步都在读写它），取的是**当前那台**。

    def _recorded_bucket(self, robot: Optional[str] = None) -> list:
        """取 ``robot``（缺省当前机器人）的示教点列表（不存在就建一个）。"""
        key = robot or self.current_robot
        return self._recorded_by_robot.setdefault(key, [])

    def _recorded_ee_bucket(self, robot: Optional[str] = None) -> list:
        """同上，末端点位列表。"""
        key = robot or self.current_robot
        return self._recorded_ee_by_robot.setdefault(key, [])

    @property
    def recorded_positions(self) -> list:
        """当前机器人的示教点（多机分桶后与之等价的旧接口）。"""
        return self._recorded_bucket()

    @recorded_positions.setter
    def recorded_positions(self, value) -> None:
        self._recorded_by_robot[self.current_robot] = list(value)

    @property
    def recorded_ee_positions(self) -> list:
        """当前机器人的末端点位（同上，旧接口兼容）。"""
        return self._recorded_ee_bucket()

    @recorded_ee_positions.setter
    def recorded_ee_positions(self, value) -> None:
        self._recorded_ee_by_robot[self.current_robot] = list(value)

    def set_joints(self, q, robot: Optional[str] = None):
        self.renderer_of(robot or self.current_robot).set_joints(q)
        self.update()

    def get_joint_values(self, robot: Optional[str] = None):
        return dict(self.renderer_of(robot or self.current_robot).joint_values)

    def set_joint_values(self, joint_dict, robot: Optional[str] = None):
        renderer = self.renderer_of(robot or self.current_robot)
        for name, value in joint_dict.items():
            renderer.joint_values[name] = value
        self.update()

    def record_position(self, robot: Optional[str] = None) -> int:
        """记录 ``robot``（缺省当前机器人）当前位形为一个示教点。"""
        key = robot or self.current_robot
        pos = self.get_joint_values(key)
        if pos:
            self._recorded_bucket(key).append(pos)
            self._recorded_ee_bucket(key).append(self._compute_end_effector_position(key))
        return len(self._recorded_bucket(key))

    def clear_recorded_positions(self, robot: Optional[str] = None) -> None:
        """清空 ``robot`` 的示教点；``None`` 表示清空所有机器人。"""
        if robot is None:
            self._recorded_by_robot.clear()
            self._recorded_ee_by_robot.clear()
        else:
            self._recorded_by_robot.pop(robot, None)
            self._recorded_ee_by_robot.pop(robot, None)

    def get_recorded_positions(self, robot: Optional[str] = None) -> list:
        """取 ``robot`` 的示教点；``None`` 取当前机器人。"""
        return self._recorded_bucket(robot)

    def set_recorded_positions(self, positions, robot: Optional[str] = None) -> None:
        """导入 ``robot`` 的示教点并重算末端点（路径可视化要用）。"""
        key = robot or self.current_robot
        bucket = self._recorded_bucket(key)
        bucket[:] = list(positions)
        # 重新计算末端执行器位置
        renderer = self.renderer_of(key)
        original_joint_values = dict(renderer.joint_values)
        ee_bucket = self._recorded_ee_bucket(key)
        ee_bucket.clear()
        for pos in positions:
            self.set_joint_values(pos, key)
            ee_bucket.append(self._compute_end_effector_position(key))
        self.set_joint_values(original_joint_values, key)

    def _compute_end_effector_position(self, robot: Optional[str] = None):
        """计算 ``robot``（缺省当前）末端执行器（TCP）的三维位置，单位米。

        走 :class:`RobotModel` 的正向运动学——它与 Pinocchio 的对拍误差为 0，
        且严格遵循 URDF 的 ``T·R`` 变换语义。旧实现是手写的 R·T 版本，
        一旦关节带 rpy 就会算错，这里直接换成已验证的实现。
        """
        renderer = self.renderer_of(robot or self.current_robot)
        if renderer.root_link is None:
            return (0.0, 0.0, 0.0)

        poses = renderer.link_poses()
        if not poses:
            return (0.0, 0.0, 0.0)

        # 末端 = 运动链上最后一个连杆；优先使用 TCP 标记
        model = renderer.model
        if model is not None:
            chain = model.link_chain()
            target = "tcp" if "tcp" in poses else (chain[-1] if chain else None)
        else:
            target = "tcp" if "tcp" in poses else None

        if target is None:
            target = next(reversed(list(poses)))
        pose = poses.get(target)
        if pose is None:
            return (0.0, 0.0, 0.0)
        return (float(pose[0, 3]), float(pose[1, 3]), float(pose[2, 3]))

    def initializeGL(self):
        gl.glClearColor(0.15, 0.15, 0.18, 1.0)
        gl.glEnable(gl.GL_DEPTH_TEST)
        self._mesh_lists = {}

        # ---- 光照 ---- #
        # 这是"显示失真"的首要成因：没有 GL_LIGHTING 时所有 glNormal3f 全部失效，
        # 箱体的六个面颜色完全一致，三维模型被渲染成二维纯色剪影。
        gl.glEnable(gl.GL_LIGHTING)
        gl.glEnable(gl.GL_LIGHT0)  # 主光：暖白，来自右上前
        gl.glEnable(gl.GL_LIGHT1)  # 补光：冷色，来自左后，避免背光面死黑
        gl.glEnable(gl.GL_LIGHT2)  # 反弹光：自下往上，压掉底部阴影
        gl.glEnable(gl.GL_NORMALIZE)  # 非均匀缩放会破坏法线单位长度
        gl.glShadeModel(gl.GL_SMOOTH)
        # 双面光照：URDF 常见开放曲面与内翻面法线，单面光照会出现黑洞
        gl.glLightModeli(gl.GL_LIGHT_MODEL_TWO_SIDE, 1)

        # 让 glColor3f 直接驱动环境/漫反射，这样既有的 glColor3f 调用无需改动
        gl.glEnable(gl.GL_COLOR_MATERIAL)
        gl.glColorMaterial(gl.GL_FRONT_AND_BACK, gl.GL_AMBIENT_AND_DIFFUSE)
        # 高光用统一材质参数：金属漆感（弱高光 + 中等锐度）
        gl.glMaterialfv(
            gl.GL_FRONT_AND_BACK,
            gl.GL_SPECULAR,
            np.array([0.32, 0.34, 0.38, 1.0], dtype=np.float32),
        )
        gl.glMaterialf(gl.GL_FRONT_AND_BACK, gl.GL_SHININESS, 38.0)

    def _setup_lights(self):
        """在相机矩阵就位后设置光源（方向光，世界坐标系）。

        必须在 ``paintGL`` 里、模型视图矩阵已切换为相机视角之后调用——光源位置会被
        调用时刻的模型视图矩阵变换，放错地方光源就会跟着相机转，明暗不再随模型变化。
        """
        gl.glLightfv(
            gl.GL_LIGHT0,
            gl.GL_POSITION,
            np.array([0.55, -0.75, 1.05, 0.0], dtype=np.float32),
        )
        gl.glLightfv(
            gl.GL_LIGHT0,
            gl.GL_DIFFUSE,
            np.array([0.95, 0.94, 0.90, 1.0], dtype=np.float32),
        )
        gl.glLightfv(
            gl.GL_LIGHT1,
            gl.GL_POSITION,
            np.array([-0.85, -0.35, 0.45, 0.0], dtype=np.float32),
        )
        gl.glLightfv(
            gl.GL_LIGHT1,
            gl.GL_DIFFUSE,
            np.array([0.30, 0.36, 0.48, 1.0], dtype=np.float32),
        )
        gl.glLightfv(
            gl.GL_LIGHT2,
            gl.GL_POSITION,
            np.array([0.10, 0.60, -0.70, 0.0], dtype=np.float32),
        )
        gl.glLightfv(
            gl.GL_LIGHT2,
            gl.GL_DIFFUSE,
            np.array([0.18, 0.18, 0.20, 1.0], dtype=np.float32),
        )
        # 全局环境光：保证最暗处仍有轮廓可辨
        gl.glLightModelfv(
            gl.GL_LIGHT_MODEL_AMBIENT,
            np.array([0.24, 0.25, 0.28, 1.0], dtype=np.float32),
        )

    def paintGL(self):
        gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        w, h = self.width(), self.height()

        gl.glMatrixMode(GL_PROJECTION)
        gl.glLoadIdentity()
        aspect = w / max(1, h)
        fov = 45.0
        near, far = 0.01, 100.0
        top = near * math.tan(math.radians(fov / 2))
        right = top * aspect
        gl.glFrustum(-right, right, -top, top, near, far)

        gl.glMatrixMode(GL_MODELVIEW)
        gl.glLoadIdentity()

        az = math.radians(self.camera_az)
        el = math.radians(self.camera_el)
        # 相机绕注视点公转，而不是绕世界原点——否则 frame_robot() 把
        # view_center 挪到模型中心后，相机仍按原点摆位，机器人直接出画。
        center = np.asarray(self.view_center, dtype=np.float64)
        eye = center + np.array(
            [
                self.camera_dist * math.cos(el) * math.cos(az),
                self.camera_dist * math.cos(el) * math.sin(az),
                self.camera_dist * math.sin(el),
            ],
            dtype=np.float64,
        )
        up_vec = np.array([0.0, 0.0, 1.0], dtype=np.float64)

        f = center - eye
        norm = np.linalg.norm(f)
        if norm > 0:
            f = f / norm
        s = np.cross(f, up_vec)
        norm = np.linalg.norm(s)
        if norm > 0:
            s = s / norm
        u = np.cross(s, f)

        # glLoadMatrixf 按**列主序**读取这 16 个数：flat[col*4 + row]。
        # 若按行写成 [s; u; -f]，GL 拿到的是转置矩阵 Rᵀ = R⁻¹（R 正交），
        # 相机等于装了反向旋转，视线偏到画面外——这是"显示失真"的根因之一。
        rot = [
            float(s[0]),
            float(u[0]),
            float(-f[0]),
            0.0,
            float(s[1]),
            float(u[1]),
            float(-f[1]),
            0.0,
            float(s[2]),
            float(u[2]),
            float(-f[2]),
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        gl.glLoadMatrixf(rot)
        gl.glTranslated(float(-eye[0]), float(-eye[1]), float(-eye[2]))

        # 光源必须在相机矩阵就位后再设，否则会跟着相机一起转
        self._setup_lights()

        # 网格与路径是"示意线"，不该参与光照（否则会被明暗调制成灰暗的虚线）
        gl.glDisable(gl.GL_LIGHTING)
        self._draw_grid()
        gl.glEnable(gl.GL_LIGHTING)

        if self.scene is not None:
            self._draw_scene()
        self._draw_robot()

        if self.show_path and self._has_any_path():
            gl.glDisable(gl.GL_LIGHTING)
            self._draw_recorded_path()
            gl.glEnable(gl.GL_LIGHTING)

        # 选中对象描边高亮 + 坐标系 gizmo（覆盖在最前）
        if self.selected_object is not None and self.scene is not None:
            self._highlight_object(self.selected_object, (1.0, 0.85, 0.0))
        self._draw_gizmo()

        # 插件覆盖层（v1.0.4）：第三方画的东西放在最上层，且不参与光照——
        # 覆盖层的用途是"标注"，被明暗调制反而看不清。
        if self.show_plugin_overlays:
            gl.glDisable(gl.GL_LIGHTING)
            self._draw_overlays()
            gl.glEnable(gl.GL_LIGHTING)

    # ------------------------------------------------------------------ #
    # 工作单元场景绘制
    # ------------------------------------------------------------------ #

    #: 场景毫米 → 视口米
    MM_TO_M = 0.001

    def _draw_scene(self) -> None:
        """绘制场景中的所有可见对象。

        场景用毫米、视口用米，因此这里对平移做整体缩放。旋转部分不受
        各向同性缩放影响，可直接透传。
        """
        if not _HAS_WORLD or self.scene is None:
            return
        try:
            for name in list(self.scene.objects):
                if self.scene.is_robot(name):
                    continue  # 机器人统一由 _draw_robot 负责，避免重复绘制
                obj = self.scene.objects.get(name)
                if obj is None or not obj.visible:
                    continue
                for part in self.scene.object_parts(name):
                    self._draw_part(part)

            # 干涉对用红色线框高亮，让问题一眼可见（v0.2.1 起支持连杆级：
            # 命中带 link_a 时把"撞在哪一段"的连杆也描红）
            for hit in self._last_collisions:
                if not self.scene.is_robot(hit.obj_b):
                    self._highlight_object(hit.obj_b, (1.0, 0.25, 0.15))
                if getattr(hit, "link_a", None):
                    self._highlight_link(hit.link_a, (1.0, 0.25, 0.15))
        except Exception as _exc:  # 渲染异常不应拖垮整个视口
            logger.warning("scene_draw_failed: %s", _exc)

    _last_collisions: list = []

    def set_collision_results(self, hits) -> None:
        """记录最近一次干涉检查结果，供绘制时高亮。

        Args:
            hits: :class:`~domain.world.scene.Collision` 列表
        """
        self._last_collisions = list(hits)
        self.update()

    def _highlight_object(self, name: str, color=None) -> None:
        """用线框高亮指定对象。

        选中对象默认亮黄；干涉命中由调用处传入红色。关闭光照与深度测试，
        使描边始终浮于最前、且颜色不被零件原色覆盖。
        """
        obj = self.scene.objects.get(name)
        if obj is None:
            return
        color = color or (1.0, 0.85, 0.0)
        gl.glPushMatrix()
        gl.glDisable(gl.GL_LIGHTING)
        gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glColor3f(*color)
        gl.glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        gl.glLineWidth(2.5)
        # 机器人走**渲染几何**描边：_picking_parts 给的是碰撞包络（为了点得中），
        # 拿它描边会让选中瞬间轮廓变成简化体，看着像换了一台机器人
        if self.scene.is_robot(name):
            self._draw_robot_wireframe(name)
        else:
            for part in self.scene.object_parts(name):
                self._draw_part(part, force_color=color)
        gl.glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        gl.glLineWidth(1.0)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_LIGHTING)
        gl.glPopMatrix()

    def _draw_robot_wireframe(self, name):
        """把某台机器人**按当前渲染几何**再画一遍（线框，沿用当前颜色）。

        与 :meth:`_draw_one_robot` 同一套几何与基座位姿，只是**不设材质色**——
        描边必须与实体完全重合，否则会出现"描边飘在机身外/形状对不上"。
        """
        renderer = self.renderer_of(name)
        model = renderer.model
        base_tf = self._robot_base_pose_m(name)
        if base_tf is not None:
            gl.glPushMatrix()
            gl.glMultMatrixd(np.ascontiguousarray(base_tf.T))
        try:
            if model is None or model.root is None:
                if renderer.root_link is not None:
                    self._draw_link_recursive(renderer.root_link, renderer=renderer)
                return
            poses = renderer.link_poses()
            for link_name in model.link_chain():
                link = model.links.get(link_name)
                base_pose = poses.get(link_name)
                if link is None or base_pose is None:
                    continue
                for visual in link.visuals:
                    gl.glPushMatrix()
                    combined = np.asarray(base_pose @ visual.matrix, dtype=np.float64)
                    gl.glMultMatrixd(np.ascontiguousarray(combined.T))
                    self._draw_primitive(visual.primitive)
                    gl.glPopMatrix()
        finally:
            if base_tf is not None:
                gl.glPopMatrix()

    def _highlight_link(self, link_name: str, color=None) -> None:
        """用线框高亮机器人的一个连杆（碰撞定位："撞在哪一段"）。

        连杆几何取自 :meth:`scene.robot_link_parts`（世界系、按当前关节
        角摆放——场景同步会把关节摆到回放/检查位形）。连杆不存在时静默
        返回（如该 URDF 无此 link 的视觉体）。
        """
        if not (_HAS_WORLD and self.scene is not None and self.scene.has_robot):
            return
        try:
            parts = self.scene.robot_link_parts().get(link_name)
        except Exception:
            return
        if not parts:
            return
        color = color or (1.0, 0.25, 0.15)
        gl.glPushMatrix()
        gl.glDisable(gl.GL_LIGHTING)
        gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glColor3f(*color)
        gl.glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        gl.glLineWidth(2.5)
        for part in parts:
            self._draw_part(part, force_color=color)
        gl.glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        gl.glLineWidth(1.0)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_LIGHTING)
        gl.glPopMatrix()

    def _draw_part(self, part, tf_parent=None, force_color=None) -> None:
        """绘制单个零件（基本体 + 局部变换）。

        Args:
            part: :class:`~domain.world.components.Part`
            tf_parent: 父级变换（已含世界变换）；``None`` 表示 part 自带世界变换
            force_color: 指定颜色（用于高亮描边），覆盖 part.color
        """
        m = np.array(part.transform, dtype=np.float64)
        if tf_parent is not None:
            m = tf_parent @ m
        m = m.copy()
        m[:3, 3] *= self.MM_TO_M

        gl.glPushMatrix()
        # OpenGL 采用列主序，numpy 为行主序，需转置后再展平
        gl.glMultMatrixd(m.T.flatten())
        if force_color is not None:
            gl.glColor3f(*force_color)
        else:
            gl.glColor3f(*part.color)

        prim = part.prim
        kind = getattr(prim, "kind", "")
        if kind == "box":
            sx, sy, sz = prim.size
            self._draw_box(sx * self.MM_TO_M, sy * self.MM_TO_M, sz * self.MM_TO_M)
        elif kind == "cylinder":
            self._draw_cylinder(prim.radius * self.MM_TO_M, prim.length * self.MM_TO_M)
        elif kind == "sphere":
            self._draw_sphere(prim.radius * self.MM_TO_M)
        elif kind == "mesh":
            mesh = getattr(prim, "mesh", None)
            if mesh is not None and not mesh.is_empty:
                self._draw_mesh(mesh)
        gl.glPopMatrix()

    def _draw_grid(self):
        gl.glBegin(GL_LINES)
        gl.glColor3f(0.3, 0.3, 0.35)
        size = 1.0
        steps = 20
        for i in range(steps + 1):
            x = -size + 2 * size * i / steps
            gl.glVertex3f(x, -size, 0)
            gl.glVertex3f(x, size, 0)
            gl.glVertex3f(-size, x, 0)
            gl.glVertex3f(size, x, 0)
        gl.glEnd()

    def _draw_robot(self):
        """绘制机器人。

        不再递归压栈拼变换，而是先用 :meth:`RobotRenderer.link_poses` 一次性算出
        每个连杆的世界位姿，再逐个 visual 用 ``model · visual.matrix`` 直接定位。
        这样变换只算一遍，且严格遵循 URDF 的 ``T·R`` 语义（旧实现是 ``R·T``，
        带 rpy 的连杆会绕错误的点旋转）。

        若机器人在场景中以真实对象存在，则按基座位姿（SceneObject.pose）整体平移/旋转。
        """
        if self.scene is not None and not self.scene.has_robot:
            return  # 场景已移除机器人，视口不再绘制
        # 多机器人：按场景里的机器人名逐个绘制，各自取渲染器与基座位姿
        names = (
            self.scene.robot_names
            if self.scene is not None and self.scene.robot_names
            else [ROBOT_NAME]
        )
        for name in names:
            self._draw_one_robot(name)

    def _draw_one_robot(self, name):
        """绘制单台机器人：按名字取渲染器，按**世界**基座位姿整体平移/旋转。"""
        renderer = self.renderer_of(name)
        model = renderer.model

        # 机器人基座位姿 → 视口米制（必须是世界位姿：见 _robot_base_pose_m）
        base_tf = self._robot_base_pose_m(name)

        if base_tf is not None:
            gl.glPushMatrix()
            gl.glMultMatrixd(np.ascontiguousarray(base_tf.T))

        try:
            if model is None or model.root is None:
                # 渲染模型模块不可用：回退到旧的单几何递归绘制
                if renderer.root_link is not None:
                    self._draw_link_recursive(renderer.root_link, renderer=renderer)
                return

            poses = renderer.link_poses()
            for link_name in model.link_chain():
                link = model.links.get(link_name)
                base_pose = poses.get(link_name)
                if link is None or base_pose is None:
                    continue
                for visual in link.visuals:
                    self._draw_visual(base_pose, visual)
        finally:
            if base_tf is not None:
                gl.glPopMatrix()

    def _draw_visual(self, base_pose: "np.ndarray", visual: object) -> None:
        """在世界位姿 ``base_pose`` 下绘制一个 visual。"""
        if visual.alpha < 0.999:
            gl.glEnable(gl.GL_BLEND)
            gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
            gl.glDepthMask(gl.GL_FALSE)

        gl.glPushMatrix()
        # OpenGL 要求列主序；numpy 是行主序，故转置后再展平
        combined = np.asarray(base_pose @ visual.matrix, dtype=np.float64)
        gl.glMultMatrixd(np.ascontiguousarray(combined.T))
        gl.glColor3f(*visual.color)
        self._draw_primitive(visual.primitive)
        gl.glPopMatrix()

        if visual.alpha < 0.999:
            gl.glDepthMask(gl.GL_TRUE)
            gl.glDisable(gl.GL_BLEND)

    def _draw_primitive(self, primitive: object) -> None:
        """按几何体类型分派绘制。"""
        kind = primitive.kind
        if kind == "box":
            self._draw_box(*primitive.params)
        elif kind == "cylinder":
            self._draw_cylinder(primitive.params[0], primitive.params[1])
        elif kind == "sphere":
            self._draw_sphere(primitive.params[0])
        elif kind == "mesh":
            mesh = primitive.mesh
            if mesh is not None and not mesh.is_empty:
                self._draw_mesh(mesh)

    # ------------------------------------------------------------------ #
    # 网格（STL / OBJ）
    # ------------------------------------------------------------------ #
    def _draw_mesh(self, mesh: "MeshData") -> None:
        """绘制网格，并把它编译进显示列表缓存。

        一个机器人可能带十几兆、近 20 万个三角形的网格。逐帧走顶点数组尚可接受，
        但编译成显示列表后 GPU 端几乎零开销——首次绘制后同一网格只是 ``glCallList``。

        Args:
            mesh: 已加载并算好法线的网格数据。
        """
        if not hasattr(self, "_mesh_lists"):
            self._mesh_lists = {}
        cached = self._mesh_lists.get(mesh.source)
        if cached is None:
            cached = gl.glGenLists(1)
            if cached == 0:  # pragma: no cover - 极老的驱动可能分配失败
                self._emit_mesh(mesh)
                return
            gl.glNewList(cached, gl.GL_COMPILE)
            self._emit_mesh(mesh)
            gl.glEndList()
            self._mesh_lists[mesh.source] = cached
        gl.glCallList(cached)

    def _emit_mesh(self, mesh: "MeshData") -> None:
        """用顶点数组提交一次三角形绘制。"""
        vertices = np.ascontiguousarray(mesh.vertices, dtype=np.float32)
        normals = np.ascontiguousarray(mesh.normals, dtype=np.float32)
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glEnableClientState(gl.GL_NORMAL_ARRAY)
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, vertices)
        gl.glNormalPointer(gl.GL_FLOAT, 0, normals)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, vertices.shape[0])
        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        gl.glDisableClientState(gl.GL_NORMAL_ARRAY)

    def _draw_link_recursive(self, link_name, parent_joint=None, renderer=None):
        """旧版递归绘制路径，仅在渲染模型模块不可用时使用。

        Args:
            link_name: 本轮要绘制的连杆名。
            parent_joint: 连到父连杆的那个关节（根连杆为 ``None``）。
            renderer: 要绘制的**那台机器人**的渲染器。多机时必须显式传——调用方
                是按机器人名取到的 ``renderer_of(name)``；退回 :attr:`renderer`
                （主/最后加载那台）会把别人的连杆画到这台身上。``None`` 时才
                退回 :attr:`renderer`，保持单机兼容。
        """
        # 回归（2026-09-25）：``renderer`` 形参曾在某次改动中丢失，函数体改读
        # ``self.renderer``，而三处调用点仍按老签名传 ``renderer=renderer``——
        # 于是 ①多机降级绘制时画错机器人；②调用点直接 TypeError。这里按调用
        # 方的意图恢复形参，并把函数体内的取值统一到它身上。
        renderer = renderer if renderer is not None else self.renderer
        link = renderer.links.get(link_name)
        if link is None:
            return

        gl.glPushMatrix()

        if parent_joint is not None:
            jx, jy, jz = parent_joint["origin_xyz"]
            gl.glTranslatef(jx, jy, jz)

            jrx, jry, jrz = parent_joint["origin_rpy"]
            if any(v != 0 for v in [jrx, jry, jrz]):
                gl.glRotatef(math.degrees(jrz), 0, 0, 1)
                gl.glRotatef(math.degrees(jry), 0, 1, 0)
                gl.glRotatef(math.degrees(jrx), 1, 0, 0)

            q = renderer.joint_values.get(parent_joint["name"], 0.0)
            if parent_joint["type"] == "revolute" and q != 0:
                ax, ay, az = parent_joint["axis"]
                gl.glRotatef(math.degrees(q), ax, ay, az)

        ox, oy, oz = link["origin_xyz"]
        orx, ory, orz = link["origin_rpy"]
        if any(v != 0 for v in [orx, ory, orz]):
            gl.glRotatef(math.degrees(orz), 0, 0, 1)
            gl.glRotatef(math.degrees(ory), 0, 1, 0)
            gl.glRotatef(math.degrees(orx), 1, 0, 0)
        gl.glTranslatef(ox, oy, oz)

        gl.glColor3f(*link["color"])
        geo = link["type"]
        if geo == "cylinder":
            radius, length = link["params"]
            self._draw_cylinder(radius, length)
        elif geo == "box":
            w, h, d = link["params"]
            self._draw_box(w, h, d)
        elif geo == "sphere":
            radius = link["params"][0]
            self._draw_sphere(radius)

        for jname, joint in renderer.joints.items():
            if joint["parent"] == link_name:
                self._draw_link_recursive(joint["child"], joint, renderer=renderer)

        gl.glPopMatrix()

    def _draw_cylinder(self, radius, length):
        """绘制沿局部 Z 轴的圆柱（URDF 约定）。

        两处改动直接改善观感：

        * **逐顶点径向法线**（旧代码每个四边形只设一次法线）→ Gouraud 平滑着色，
          圆柱不再是一个个平面拼出的多面体。
        * **按半径自适应分段**（旧代码固定 24 段）→ 粗轴不露棱角，细螺栓不过度细分。
        """
        segments = _segments_for_radius(radius)
        half_len = length / 2.0

        cos_table = [math.cos(2 * math.pi * i / segments) for i in range(segments + 1)]
        sin_table = [math.sin(2 * math.pi * i / segments) for i in range(segments + 1)]

        # 侧面：每个四边形拆两个三角形，绕向 A→B→C→D 保证法线朝外
        gl.glBegin(GL_TRIANGLES)
        for i in range(segments):
            c1, s1 = cos_table[i], sin_table[i]
            c2, s2 = cos_table[i + 1], sin_table[i + 1]
            # A(-h, i) → B(-h, i+1) → C(+h, i+1) → D(+h, i)
            gl.glNormal3f(c1, s1, 0.0)
            gl.glVertex3f(radius * c1, radius * s1, -half_len)
            gl.glNormal3f(c2, s2, 0.0)
            gl.glVertex3f(radius * c2, radius * s2, -half_len)
            gl.glNormal3f(c2, s2, 0.0)
            gl.glVertex3f(radius * c2, radius * s2, half_len)

            gl.glNormal3f(c1, s1, 0.0)
            gl.glVertex3f(radius * c1, radius * s1, -half_len)
            gl.glNormal3f(c2, s2, 0.0)
            gl.glVertex3f(radius * c2, radius * s2, half_len)
            gl.glNormal3f(c1, s1, 0.0)
            gl.glVertex3f(radius * c1, radius * s1, half_len)
        gl.glEnd()

        # 顶盖：从 +Z 看逆时针
        gl.glBegin(GL_TRIANGLE_FAN)
        gl.glNormal3f(0, 0, 1)
        gl.glVertex3f(0, 0, half_len)
        for i in range(segments + 1):
            a = 2 * math.pi * i / segments
            gl.glVertex3f(radius * math.cos(a), radius * math.sin(a), half_len)
        gl.glEnd()
        # 底盖：从 -Z 看逆时针（顶点序反转）
        gl.glBegin(GL_TRIANGLE_FAN)
        gl.glNormal3f(0, 0, -1)
        gl.glVertex3f(0, 0, -half_len)
        for i in range(segments, -1, -1):
            a = 2 * math.pi * i / segments
            gl.glVertex3f(radius * math.cos(a), radius * math.sin(a), -half_len)
        gl.glEnd()

    def _draw_box(self, w, h, d):
        x, y, z = w / 2, h / 2, d / 2
        gl.glBegin(GL_QUADS)
        gl.glNormal3f(0, 0, 1)
        gl.glVertex3f(-x, -y, z)
        gl.glVertex3f(x, -y, z)
        gl.glVertex3f(x, y, z)
        gl.glVertex3f(-x, y, z)
        gl.glNormal3f(0, 0, -1)
        gl.glVertex3f(-x, -y, -z)
        gl.glVertex3f(-x, y, -z)
        gl.glVertex3f(x, y, -z)
        gl.glVertex3f(x, -y, -z)
        gl.glNormal3f(1, 0, 0)
        gl.glVertex3f(x, -y, -z)
        gl.glVertex3f(x, y, -z)
        gl.glVertex3f(x, y, z)
        gl.glVertex3f(x, -y, z)
        gl.glNormal3f(-1, 0, 0)
        gl.glVertex3f(-x, -y, -z)
        gl.glVertex3f(-x, -y, z)
        gl.glVertex3f(-x, y, z)
        gl.glVertex3f(-x, y, -z)
        gl.glNormal3f(0, 1, 0)
        gl.glVertex3f(-x, y, -z)
        gl.glVertex3f(-x, y, z)
        gl.glVertex3f(x, y, z)
        gl.glVertex3f(x, y, -z)
        gl.glNormal3f(0, -1, 0)
        gl.glVertex3f(-x, -y, -z)
        gl.glVertex3f(x, -y, -z)
        gl.glVertex3f(x, -y, z)
        gl.glVertex3f(-x, -y, z)
        gl.glEnd()

    def _draw_sphere(self, radius, stacks=None, slices=None):
        """绘制球（逐顶点径向法线 + 按半径自适应细分）。

        Args:
            radius: 半径。
            stacks: 纬向分段数；``None`` 时按半径自适应。
            slices: 经向分段数；``None`` 时按半径自适应。
        """
        if slices is None:
            segments = _segments_for_radius(radius)
            slices = max(12, segments)
            stacks = max(8, slices // 2)
        for i in range(stacks):
            lat0 = math.pi * (-0.5 + i / stacks)
            lat1 = math.pi * (-0.5 + (i + 1) / stacks)
            z0 = radius * math.sin(lat0)
            zr0 = radius * math.cos(lat0)
            z1 = radius * math.sin(lat1)
            zr1 = radius * math.cos(lat1)
            gl.glBegin(GL_QUAD_STRIP)
            for j in range(slices + 1):
                lng = 2 * math.pi * j / slices
                x = math.cos(lng)
                y = math.sin(lng)
                gl.glNormal3f(x * math.cos(lat0), y * math.cos(lat0), math.sin(lat0))
                gl.glVertex3f(x * zr0, y * zr0, z0)
                gl.glNormal3f(x * math.cos(lat1), y * math.cos(lat1), math.sin(lat1))
                gl.glVertex3f(x * zr1, y * zr1, z1)
            gl.glEnd()

    def _path_robots(self) -> list:
        """有末端路径可画的机器人名（已移除的不算，免得留下幽灵路径）。"""
        names = []
        for name, pts in self._recorded_ee_by_robot.items():
            if len(pts) < 2:
                continue
            # 场景里已经没有它了 → 不该再画（否则原点会堆一条无主的线）
            if self.scene is not None and name not in self.scene.objects:
                continue
            names.append(name)
        return names

    def _has_any_path(self) -> bool:
        """是否任一机器人有可画的路径。"""
        return bool(self._path_robots())

    def _path_points_for(self, robot: Optional[str] = None) -> list:
        """``robot`` 的末端路径点，换算到**世界系米制**。

        记录的末端点是渲染器正向运动学算出来的，处在**机器人自己的基座系**。
        直接拿来画的话，第二台（以及任何被搬动过/挂上底座的机器人）的路径会
        全部叠在原点——必须左乘该机器人的世界基座位姿。
        """
        key = robot or self.current_robot
        pts = self._recorded_ee_by_robot.get(key) or []
        if not pts:
            return []
        base = self._robot_base_pose_m(key)
        if base is None:
            # 没有场景/基座位姿（独立用法）：沿用基座系，与旧行为一致
            return [tuple(float(v) for v in p) for p in pts]
        out = []
        for p in pts:
            v = np.array([float(p[0]), float(p[1]), float(p[2]), 1.0], dtype=np.float64)
            w = base @ v
            out.append((float(w[0]), float(w[1]), float(w[2])))
        return out

    def _draw_recorded_path(self):
        """绘制每台机器人**各自**的末端路径。

        路径属于机器人：既不能只画当前那台（别的机器人示教过的路径会凭空
        消失），也不能都画在原点——末端点是基座系的，得先乘各自的基座位姿。
        """
        for name in self._path_robots():
            pts = self._path_points_for(name)
            if len(pts) < 2:
                continue
            # 当前这台亮、其余压暗：一眼看出"正在编辑谁"
            is_current = name == self.current_robot
            if is_current:
                gl.glLineWidth(2.0)
                gl.glColor3f(0.2, 0.8, 0.2)
            else:
                gl.glLineWidth(1.5)
                gl.glColor3f(0.25, 0.5, 0.25)

            gl.glBegin(GL_LINE_STRIP)
            for x, y, z in pts:
                gl.glVertex3f(x, y, z)
            gl.glEnd()

            if is_current:
                gl.glPointSize(5.0)
                gl.glColor3f(1.0, 0.3, 0.3)
            else:
                gl.glPointSize(3.5)
                gl.glColor3f(0.65, 0.3, 0.3)
            gl.glBegin(GL_POINTS)
            for x, y, z in pts:
                gl.glVertex3f(x, y, z)
            gl.glEnd()

    # ------------------------------------------------------------------ #
    # 插件渲染覆盖层（v1.0.4）
    # ------------------------------------------------------------------ #

    def set_overlay_context(self, context) -> None:
        """设置交给渲染插件的上下文（``None`` 表示暂无轨迹）。"""
        self.plugin_overlay_context = context

    def refresh_plugin_overlays(self):
        """重新收集渲染插件覆盖层。

        Returns:
            :class:`adapter.plugins.OverlayCollection`，或插件层不可用时的 ``None``
        """
        if not _HAS_PLUGINS:
            return None
        try:
            ctx = self.plugin_overlay_context
            if ctx is None:
                ctx = OverlayContext()
            collected = collect_overlays(ctx)
        except Exception as exc:  # 插件层异常不应让视口变黑
            logger.warning("plugin_overlays_failed: %s", exc)
            self.plugin_overlays = None
            self.plugin_overlay_errors = [str(exc)]
            return None
        self.plugin_overlays = collected
        # 顺手把错误回灌到注册表，让"插件状态"对话框能看到真正发生的事
        for err in collected.errors:
            if _HAS_PLUGINS:
                try:
                    # 回归（2026-09-25）：导入时是 ``default_registry as
                    # plugin_registry``，``default_registry`` 这个名字根本没绑定；
                    # NameError 又被下面 ``except Exception: pass`` 吞掉，于是
                    # 插件加载错误一直没能回灌进注册表——"插件状态"里永远查不到。
                    plugin_registry.add_error(err.name, err.source, err.message, err.detail)
                except Exception:
                    pass
        self.plugin_overlay_errors = [f"{e.name}: {e.message}" for e in collected.errors]
        if collected.errors:
            for msg in self.plugin_overlay_errors:
                logger.warning("render_plugin_error: %s", msg)
        self.update()
        return collected

    def _draw_overlays(self) -> None:
        """绘制插件产出的覆盖层（纯数据 → GL 调用，覆盖层坐标是毫米）。"""
        if self.plugin_overlays is None:
            return
        mm = self.MM_TO_M
        try:
            for _name, item in self.plugin_overlays.drawables:
                if isinstance(item, LineSegments):
                    gl.glLineWidth(max(1.0, float(item.width)))
                    gl.glColor3f(*item.color)
                    gl.glBegin(GL_LINES)
                    for p0, p1 in item.segments:
                        gl.glVertex3f(float(p0[0]) * mm, float(p0[1]) * mm, float(p0[2]) * mm)
                        gl.glVertex3f(float(p1[0]) * mm, float(p1[1]) * mm, float(p1[2]) * mm)
                    gl.glEnd()
                elif isinstance(item, PointCloud):
                    gl.glPointSize(max(1.0, float(item.size)))
                    gl.glColor3f(*item.color)
                    gl.glBegin(GL_POINTS)
                    for p in item.positions:
                        gl.glVertex3f(float(p[0]) * mm, float(p[1]) * mm, float(p[2]) * mm)
                    gl.glEnd()
                elif isinstance(item, FrameMarker):
                    self._draw_overlay_frame(item, mm)
        except Exception as exc:  # 画坏一个覆盖层不能毁掉整帧
            logger.warning("overlay_draw_failed: %s", exc)

    def _draw_overlay_frame(self, marker, mm: float) -> None:
        """画位姿三轴（X 偏红轴、Y 偏绿轴、Z 偏蓝轴）。"""
        import numpy as _np

        pose = marker.pose
        try:
            mat = _np.asarray(pose.matrix, dtype=float)
        except Exception:
            return
        origin = mat[:3, 3] * mm
        scale = float(marker.scale) * mm
        base = tuple(float(c) for c in marker.color)
        axes = (
            (mat[:3, 0], (base[0], base[1] * 0.4, base[2] * 0.4)),
            (mat[:3, 1], (base[0] * 0.4, base[1], base[2] * 0.4)),
            (mat[:3, 2], (base[0] * 0.4, base[1] * 0.4, base[2])),
        )
        gl.glLineWidth(2.0)
        gl.glBegin(GL_LINES)
        for axis, color in axes:
            gl.glColor3f(*color)
            gl.glVertex3f(float(origin[0]), float(origin[1]), float(origin[2]))
            tip = origin + _np.asarray(axis, dtype=float) * scale
            gl.glVertex3f(float(tip[0]), float(tip[1]), float(tip[2]))
        gl.glEnd()

    # ------------------------------------------------------------------ #
    # 右键拾取 / 坐标系 gizmo 变换
    # ------------------------------------------------------------------ #

    def _view_proj_matrix(self):
        """返回 (V, P, M)，与 paintGL 完全一致的相机矩阵（米制）。"""
        w, h = self.width(), self.height()
        aspect = w / max(1, h)
        fov = 45.0
        near, far = 0.01, 100.0
        top = near * math.tan(math.radians(fov / 2))
        right = top * aspect

        az = math.radians(self.camera_az)
        el = math.radians(self.camera_el)
        center = np.asarray(self.view_center, dtype=np.float64)
        eye = center + np.array(
            [
                self.camera_dist * math.cos(el) * math.cos(az),
                self.camera_dist * math.cos(el) * math.sin(az),
                self.camera_dist * math.sin(el),
            ],
            dtype=np.float64,
        )
        up_vec = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        f = center - eye
        norm = np.linalg.norm(f)
        if norm > 0:
            f = f / norm
        s = np.cross(f, up_vec)
        norm = np.linalg.norm(s)
        if norm > 0:
            s = s / norm
        u = np.cross(s, f)

        # numpy 行主序：paintGL 用 glLoadMatrixf（列主序）加载的视图矩阵，
        # 其旋转部分的**行**依次为 s, u, -f。这里必须用【行赋值】，否则旋转
        # 被转置（正交矩阵转置=逆），反推射线方向错误，拾取必然失败。
        V = np.eye(4)
        V[0, :3] = s
        V[1, :3] = u
        V[2, :3] = -f
        V[:3, 3] = -(V[:3, :3] @ eye)

        P = np.zeros((4, 4))
        P[0, 0] = near / right
        P[1, 1] = near / top
        P[2, 2] = -(far + near) / (far - near)
        P[2, 3] = -2.0 * far * near / (far - near)
        P[3, 2] = -1.0
        M = P @ V
        return V, P, M

    def _ray_from_mouse(self, x, y):
        """返回 (origin_m, dir_m)：世界系米制射线原点与单位方向。"""
        w, h = self.width(), self.height()
        ndc_x = (2.0 * x) / w - 1.0
        ndc_y = 1.0 - (2.0 * y) / h
        _, _, M = self._view_proj_matrix()
        Minv = np.linalg.inv(M)
        near_h = Minv @ np.array([ndc_x, ndc_y, -1.0, 1.0])
        far_h = Minv @ np.array([ndc_x, ndc_y, 1.0, 1.0])
        near_p = near_h[:3] / near_h[3]
        far_p = far_h[:3] / far_h[3]
        origin = near_p
        direction = far_p - near_p
        n = np.linalg.norm(direction)
        if n > 0:
            direction = direction / n
        return origin, direction

    @staticmethod
    def _ray_aabb(origin, direction, lo, hi):
        """射线-轴对齐包围盒 slab 求交，返回最近 t（>=0），无交返回 None。"""
        tmin = -np.inf
        tmax = np.inf
        for i in range(3):
            if abs(direction[i]) < 1e-12:
                if origin[i] < lo[i] or origin[i] > hi[i]:
                    return None
            else:
                inv = 1.0 / direction[i]
                t1 = (lo[i] - origin[i]) * inv
                t2 = (hi[i] - origin[i]) * inv
                tmin = max(tmin, min(t1, t2))
                tmax = min(tmax, max(t1, t2))
        if tmax < max(tmin, 0.0):
            return None
        return max(tmin, 0.0)

    def _ray_plane_z(self, origin, direction, z_m):
        """射线与水平面 z=z_m 的交点（米）；平行或无交点返回 None。"""
        if abs(direction[2]) < 1e-9:
            return None
        t = (z_m - origin[2]) / direction[2]
        if t < 0:
            return None
        return origin + t * direction

    def _project(self, p_m):
        """世界点（米）投影到视口像素坐标 (sx, sy)，y 轴向下。"""
        w, h = self.width(), self.height()
        _, _, M = self._view_proj_matrix()
        clip = M @ np.array([p_m[0], p_m[1], p_m[2], 1.0])
        if abs(clip[3]) < 1e-9:
            return (w / 2.0, h / 2.0)
        ndc_x = clip[0] / clip[3]
        ndc_y = clip[1] / clip[3]
        sx = (ndc_x + 1.0) / 2.0 * w
        sy = (1.0 - (ndc_y + 1.0) / 2.0) * h
        return float(sx), float(sy)

    def _picking_parts(self, name):
        """对象参与**拾取与选中描边**的几何（世界系）。

        机器人必须改用**碰撞包络**：真实 URDF 的 ``<visual>`` 几乎全是
        ``<mesh>``（场景解析器不解析），用 visual 会让整台机器人只剩一个 TCP
        标记球 —— 射线打空、描边也只有一个小球，表现为"看得见却选不中"。
        干涉检查早就这么处理了，拾取与描边与此保持一致。
        """
        if self.scene.is_robot(name):
            return [
                p
                for group in self.scene.robot_world_link_parts(
                    name, source=GEOMETRY_COLLISION
                ).values()
                for p in group
            ]
        return self.scene.object_parts(name)

    def _pick_object(self, x, y):
        """射线-AABB 求交，返回最近场景对象名；无命中返回 None。"""
        if not _HAS_WORLD or self.scene is None:
            return None
        try:
            origin, direction = self._ray_from_mouse(x, y)
            best = None
            best_t = float("inf")
            for name in self.scene.objects:
                obj = self.scene.objects.get(name)
                if obj is None or not obj.visible:
                    continue
                lo = np.array([np.inf, np.inf, np.inf])
                hi = np.array([-np.inf, -np.inf, -np.inf])
                for part in self._picking_parts(name):
                    try:
                        aabb = prim_aabb(part.prim, part.transform)
                        plo, phi = aabb.lo, aabb.hi
                    except Exception:
                        # mesh 等无解析 AABB 的基本体：直接用顶点包络
                        mesh = getattr(part.prim, "mesh", None)
                        if mesh is None or getattr(mesh, "is_empty", True):
                            continue
                        R = part.transform[:3, :3]
                        t = part.transform[:3, 3]
                        pts = (np.asarray(mesh.vertices, dtype=np.float64) @ R.T) + t
                        plo, phi = pts.min(axis=0), pts.max(axis=0)
                    lo = np.minimum(lo, plo * self.MM_TO_M)
                    hi = np.maximum(hi, phi * self.MM_TO_M)
                if not np.all(np.isfinite(lo)) or not np.all(np.isfinite(hi)):
                    continue
                t = self._ray_aabb(origin, direction, lo, hi)
                if t is not None and t < best_t:
                    best_t = t
                    best = name
            return best
        except Exception as _exc:  # 拾取异常不应拖垮视口
            logger.warning("scene_pick_failed: %s", _exc)
            return None

    def _gizmo_screen_geom(self):
        """返回 gizmo 在屏幕上的几何（像素），供绘制与命中判定共用。"""
        if self.selected_object is None or self.scene is None:
            return None
        obj = self.scene.objects.get(self.selected_object)
        if obj is None:
            return None
        tf = self.scene.world_transform(self.selected_object)  # mm
        center_m = tf[:3, 3] * self.MM_TO_M
        cx, cy = self._project(center_m)
        z_up = self._project(center_m + np.array([0.0, 0.0, 0.1]))  # 上方 0.1m
        px_per_m = math.hypot(z_up[0] - cx, z_up[1] - cy) / 0.1
        return {
            "cx": cx,
            "cy": cy,
            "ring_r": 55.0,  # 旋转环半径(px)
            "axis_len": 75.0,  # Z 轴长度(px)
            "xy_half": 42.0,  # XY 方块半边长(px)
            "hit": 10.0,  # 命中容差(px)
            "z_strip": 16.0,  # Z 轴竖条命中半宽(px)，加宽便于抓取
            "z_tip_r": 18.0,  # Z 轴顶端圆盘抓取半径(px)
            "px_per_m": px_per_m,
        }

    def _gizmo_hit_mode(self, x, y):
        """屏幕空间判定光标命中的手柄，返回 'xy' / 'z' / 'rz' / None。"""
        g = self._gizmo_screen_geom()
        if g is None:
            return None
        dx = x - g["cx"]
        dy = y - g["cy"]
        dist = math.hypot(dx, dy)
        # 1) Z 轴：顶端圆盘抓取点 + 整条加宽竖条，优先于 XY 方块
        if math.hypot(dx, dy + g["axis_len"]) < g["z_tip_r"]:
            return "z"
        if abs(dx) < g["z_strip"] and -g["axis_len"] - g["hit"] <= dy <= g["hit"]:
            return "z"
        # 2) 旋转环
        if abs(dist - g["ring_r"]) < g["hit"]:
            return "rz"
        # 3) XY 方块
        if abs(dx) < g["xy_half"] and abs(dy) < g["xy_half"]:
            return "xy"
        return None

    def _set_object_world_pose(self, name, x_mm, y_mm, z_mm, rz_deg):
        """把对象设到指定的世界位姿（毫米 + 绕Z角度），自动换算回局部 pose。

        世界系 → 局部系的换算只有一处定义：``world.placement.apply_world_pose``。
        """
        if not _HAS_WORLD or self.scene is None:
            return
        _placement.apply_world_pose(self.scene, name, x_mm, y_mm, z_mm, rz_deg)

    def _grabbable(self, name) -> bool:
        """对象是否允许左键拖拽摆放（机器人/挂载对象/锁定对象除外）。"""
        if not _HAS_WORLD or self.scene is None or _placement is None:
            return False
        return _placement.grabbable(self.scene, name)

    def _grab_start(self, name, x, y):
        """左键按在工件上：进入拖拽摆放，锁定"按住时的世界高度平面"。"""
        self.selected_object = name
        self.drag_mode = None
        # 已装夹到法兰的工具/工件：先摘下来再拖（detach 保持世界位姿，不跳变）
        if self._detach_mounted(name):
            self.sceneChanged.emit(name)
        self._grab_active = True
        self._grab_anchor = self._make_anchor("xy", x, y)
        self.objectSelected.emit(name)

    def _grab_move(self, x, y):
        """拖动中：沿按下时的高度平面移动 XY（Z 不变，松手才落面）。"""
        if self._grab_anchor is None or self.selected_object is None:
            return
        self._apply_drag("xy", self._grab_anchor, x, y)

    def _grab_drop(self):
        """松手：落到正下方支承面（几何算出来的，不再手填 Z）。"""
        if not _HAS_WORLD or self.scene is None or self.selected_object is None:
            return
        name = self.selected_object
        # 装夹优先于落面：把夹爪拖到臂末端松手，期望是"装上"而不是"掉地上"
        robot = self._try_snap_mount(name)
        if robot:
            self.update()
            self.objectSelected.emit(name)
            self.sceneChanged.emit(name)
            self.objectPlaced.emit(
                tr("已将 {name} 装夹到 {host} 末端").format(name=name, host=robot)
            )
            return
        tf = self.scene.world_transform(name)
        result = _placement.place_on_support(
            self.scene,
            name,
            float(tf[0, 3]),
            float(tf[1, 3]),
            snap_step=self.snap_step_mm,
        )
        # 吸附只作用于落点 XY：place_on_support 已按 snap_step 写回位姿，
        # 这里只负责把结论说给状态栏听
        self.update()
        self.objectSelected.emit(name)
        self.objectPlaced.emit(_placement.placement_summary(result))

    def _detach_mounted(self, name):
        """起拖前解除法兰挂载（保持世界位姿）。

        不解除的话，写回的局部位姿会变成"工具偏移"：名义上还挂在法兰下、
        位置却被挪开了，用户难以理解。只对**挂在机器人法兰下**的对象生效——
        挂在普通对象下的子对象仍由 placement 层按原规则拒绝拖动。

        Returns:
            是否真的解除过
        """
        if not name or self.scene is None or _placement is None:
            return False
        if not _placement.mounted_on_robot(self.scene, name):
            return False
        try:
            self.scene.detach(name)
        except Exception as exc:  # 只有对象不存在时才会失败
            logger.warning("auto_detach_failed: %s", exc)
            return False
        return True

    def _try_snap_mount(self, name):
        """松手时若停在机器人法兰附近，就装夹到该机器人末端。

        装夹＝局部位姿归零：工具坐标系自动与法兰对齐，而不是保留当前世界位置。

        Returns:
            命中的机器人名；未装夹返回 ``None``
        """
        if not name or self.scene is None or _placement is None:
            return None
        if self.scene.is_robot(name):
            return None  # 机器人本体不能被装夹到别的机器人法兰上
        robot = _placement.flange_snap_target(
            self.scene, name, getattr(self, "snap_radius_mm", 60.0)
        )
        if robot is None or name == robot:
            return None
        try:
            _placement.mount_to_flange(self.scene, name, robot)
        except (KeyError, ValueError) as exc:
            logger.warning("auto_attach_failed: %s", exc)
            return None
        return robot

    def _make_anchor(self, mode, x, y):
        """记录拖动起始参考量（对象世界位姿 + 模式相关的局部参考）。"""
        name = self.selected_object
        tf = self.scene.world_transform(name)  # mm
        start_x, start_y, start_z = tf[:3, 3]
        start_rz = math.degrees(math.atan2(tf[1, 0], tf[0, 0]))
        center_m = tf[:3, 3] * self.MM_TO_M
        anchor = {
            "start_x": float(start_x),
            "start_y": float(start_y),
            "start_z": float(start_z),
            "start_rz": float(start_rz),
            "center_m": center_m,
        }
        if mode == "xy":
            origin, direction = self._ray_from_mouse(x, y)
            p = self._ray_plane_z(origin, direction, center_m[2])
            anchor["plane_start"] = p if p is not None else center_m.copy()
        elif mode == "z":
            anchor["start_mouse_y"] = y
            g = self._gizmo_screen_geom()
            anchor["px_per_m"] = g["px_per_m"] if g else 1000.0
        elif mode == "rz":
            g = self._gizmo_screen_geom()
            anchor["start_cursor_angle"] = math.atan2(y - g["cy"], x - g["cx"])
        return anchor

    def _apply_drag(self, mode, anchor, x, y):
        """按已锁定的模式执行变换，并写回对象位姿。"""
        name = self.selected_object
        if mode == "xy":
            origin, direction = self._ray_from_mouse(x, y)
            p = self._ray_plane_z(origin, direction, anchor["center_m"][2])
            if p is None:
                return
            dx_m = p[0] - anchor["plane_start"][0]
            dy_m = p[1] - anchor["plane_start"][1]
            nx = anchor["start_x"] + dx_m * 1000.0
            ny = anchor["start_y"] + dy_m * 1000.0
            self._set_object_world_pose(name, nx, ny, anchor["start_z"], anchor["start_rz"])
        elif mode == "z":
            dy_px = anchor["start_mouse_y"] - y  # 屏幕向上为正
            mm_per_px = 1000.0 / max(anchor["px_per_m"], 1e-6)
            nz = anchor["start_z"] + dy_px * mm_per_px
            self._set_object_world_pose(
                name, anchor["start_x"], anchor["start_y"], nz, anchor["start_rz"]
            )
        elif mode == "rz":
            g = self._gizmo_screen_geom()
            cur = math.atan2(y - g["cy"], x - g["cx"])
            delta = cur - anchor["start_cursor_angle"]
            new_rz = anchor["start_rz"] - math.degrees(delta)  # 屏幕 y 向下，取负使视觉一致
            self._set_object_world_pose(
                name, anchor["start_x"], anchor["start_y"], anchor["start_z"], new_rz
            )
        self.update()
        self.objectSelected.emit(name)  # 同步刷新左侧数值框

    def _handle_right_drag(self, pos):
        """右键拖动：先探测手柄锁定模式，再执行对应变换。"""
        if not _HAS_WORLD or self.scene is None:
            return
        x, y = pos.x(), pos.y()
        if self.drag_mode is None:
            mode = self._gizmo_hit_mode(x, y)
            if mode is None:
                return
            # 右键 gizmo 与左键一致：拖已装夹件前先摘下来
            if self._detach_mounted(self.selected_object):
                self.sceneChanged.emit(self.selected_object)
            self._drag_anchor = self._make_anchor(mode, x, y)
            self.drag_mode = mode
            self.update()
            return
        self._apply_drag(self.drag_mode, self._drag_anchor, x, y)

    def _draw_gizmo(self):
        """在 selected_object 原点绘制 XY 面 / Z 轴 / 绕Z 环手柄，高亮当前 drag_mode。"""
        if self.selected_object is None or self.scene is None:
            return
        g = self._gizmo_screen_geom()
        if g is None:
            return
        cx, cy = g["cx"], g["cy"]

        gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glDisable(gl.GL_LIGHTING)
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glPushMatrix()
        gl.glLoadIdentity()
        gl.glOrtho(0, self.width(), self.height(), 0, -1, 1)  # 屏幕像素，y 向下
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glPushMatrix()
        gl.glLoadIdentity()

        def line(x1, y1, x2, y2, color, width=2.0):
            gl.glColor3f(*color)
            gl.glLineWidth(width)
            gl.glBegin(gl.GL_LINES)
            gl.glVertex2f(x1, y1)
            gl.glVertex2f(x2, y2)
            gl.glEnd()

        def ring(cx, cy, r, color, width=2.0):
            gl.glColor3f(*color)
            gl.glLineWidth(width)
            gl.glBegin(gl.GL_LINE_LOOP)
            for i in range(48):
                a = 2 * math.pi * i / 48
                gl.glVertex2f(cx + r * math.cos(a), cy + r * math.sin(a))
            gl.glEnd()

        def rect(cx, cy, h, color, width=2.0):
            gl.glColor3f(*color)
            gl.glLineWidth(width)
            gl.glBegin(gl.GL_LINE_LOOP)
            gl.glVertex2f(cx - h, cy - h)
            gl.glVertex2f(cx + h, cy - h)
            gl.glVertex2f(cx + h, cy + h)
            gl.glVertex2f(cx - h, cy + h)
            gl.glEnd()

        def disk(cx, cy, r, color):
            gl.glColor3f(*color)
            gl.glBegin(gl.GL_TRIANGLE_FAN)
            gl.glVertex2f(cx, cy)
            for i in range(25):
                a = 2 * math.pi * i / 24
                gl.glVertex2f(cx + r * math.cos(a), cy + r * math.sin(a))
            gl.glEnd()

        mode = self.drag_mode
        z_col = (1.0, 0.3, 0.25) if mode == "z" else (0.85, 0.45, 0.4)
        line(cx, cy, cx, cy - g["axis_len"], z_col, 3.0 if mode == "z" else 2.0)
        disk(cx, cy - g["axis_len"], g["z_tip_r"] * 0.6, z_col)  # 顶端抓取圆盘
        r_col = (1.0, 0.85, 0.2) if mode == "rz" else (0.85, 0.7, 0.2)
        ring(cx, cy, g["ring_r"], r_col, 3.0 if mode == "rz" else 2.0)
        xy_col = (0.3, 0.7, 1.0) if mode == "xy" else (0.35, 0.55, 0.8)
        rect(cx, cy, g["xy_half"], xy_col, 3.0 if mode == "xy" else 2.0)
        gl.glColor3f(1.0, 1.0, 1.0)
        gl.glPointSize(5.0)
        gl.glBegin(gl.GL_POINTS)
        gl.glVertex2f(cx, cy)
        gl.glEnd()

        gl.glPopMatrix()
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glPopMatrix()
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glEnable(gl.GL_LIGHTING)
        gl.glEnable(gl.GL_DEPTH_TEST)

    def mousePressEvent(self, event):
        # Mac 触控板：两指轻点发出 RightButton；Control+左键（Control+Click）
        # 也是经典右键替代，这里统一当作“右键拾取”处理。
        is_right_like = event.button() == Qt.RightButton or (
            event.button() == Qt.LeftButton and event.modifiers() & Qt.ControlModifier
        )
        if is_right_like:
            self._right_like = True
            #: 本次右键是否已经拖过——拖过就是 gizmo/视角操作，松手不弹菜单
            self._right_moved = False
            #: 按下位置：只有越过 :attr:`RIGHT_DRAG_SLOP_PX` 才算"拖过"
            self._right_press_pos = event.position().toPoint()
            if _HAS_WORLD and self.scene is not None:
                pos = event.position().toPoint()
                name = self._pick_object(pos.x(), pos.y())
                if name is not None:
                    self.selected_object = name
                    self.drag_mode = None
                    self._drag_anchor = None
                    self.objectSelected.emit(name)
                else:
                    self.selected_object = None
                    self.drag_mode = None
                self.update()
        elif event.button() == Qt.LeftButton:
            pos = event.position().toPoint()
            self._left_press_pos = pos
            self._left_moved = False
            # 左键按在可摆放的工件/设备上 = 拖拽摆放（v1.1.7）；
            # 按在机器人或空白处 = 旋转视角（与既有行为一致）
            if _HAS_WORLD and self.scene is not None:
                name = self._pick_object(pos.x(), pos.y())
                if self._grabbable(name):
                    self._grab_start(name, pos.x(), pos.y())
                    self.update()
                    return
            self.dragging = True
            self.last_pos = pos
            # 左键按下即尝试拾取对象，让机器人/组件一点就高亮
            if _HAS_WORLD and self.scene is not None:
                name = self._pick_object(pos.x(), pos.y())
                if name is not None:
                    self.selected_object = name
                    self.objectSelected.emit(name)
                else:
                    self.selected_object = None
                self.update()

    def mouseMoveEvent(self, event):
        # 左键拖拽摆放中：只移动工件，不转视角
        if self._grab_active and (event.buttons() & Qt.LeftButton):
            self._left_moved = True
            self._grab_move(event.position().toPoint().x(), event.position().toPoint().y())
            return

        # 右键（或 Control+左键）按住时做 gizmo 变换，不做视角旋转
        if self._right_like and (event.buttons() & (Qt.RightButton | Qt.LeftButton)):
            # 「这次算不算拖过」带阈值：触控板两指轻点与鼠标右键都会带 1~2px
            # 抖动，一律算拖的话右键菜单永远弹不出来（见 RIGHT_DRAG_SLOP_PX）。
            # 注意阈值**只作用于这个判定**：gizmo 本身照旧响应每一次移动——
            # 手柄模式是靠"第一段小幅移动"锁定的，一并挡掉就再也拖不动了。
            pos = event.position().toPoint()
            start = getattr(self, "_right_press_pos", None)
            if (
                start is None
                or math.hypot(pos.x() - start.x(), pos.y() - start.y()) > self.RIGHT_DRAG_SLOP_PX
            ):
                self._right_moved = True
            if self.selected_object is not None:
                self._handle_right_drag(event.position().toPoint())
            return

        if self.dragging and self.last_pos:
            current_pos = event.position().toPoint()
            if (
                self._left_press_pos is not None
                and (current_pos - self._left_press_pos).manhattanLength() > 4
            ):
                self._left_moved = True
            dx = current_pos.x() - self.last_pos.x()
            dy = current_pos.y() - self.last_pos.y()
            self.camera_az -= dx * 0.5
            self.camera_el += dy * 0.5
            self.camera_el = max(-89, min(89, self.camera_el))
            self.last_pos = current_pos
            self.update()

        if event.buttons() & Qt.RightButton and self.selected_object is not None:
            self._handle_right_drag(event.position().toPoint())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.dragging = False
            self._right_like = False
            if self._grab_active:
                # 拖拽摆放收尾：落到支承面并把结论报给状态栏
                self._grab_drop()
                self._grab_active = False
                self._grab_anchor = None
                self._left_moved = False
                self._left_press_pos = None
                return
            if not self._left_moved and _HAS_WORLD and self.scene is not None:
                pos = event.position().toPoint()
                name = self._pick_object(pos.x(), pos.y())
                if name is not None:
                    self.selected_object = name
                    self.objectSelected.emit(name)
                else:
                    self.selected_object = None
                self.update()
            self._left_moved = False
            self._left_press_pos = None
        elif event.button() == Qt.RightButton:
            self._right_like = False
        # 保留选中与 gizmo 显示；仅清除本次拖动模式，下次右键重新判定
        self.drag_mode = None
        self._drag_anchor = None

    def wheelEvent(self, event):
        delta = event.angleDelta().y() / 120
        self.camera_dist *= 0.9**delta
        self.camera_dist = max(0.3, min(10.0, self.camera_dist))
        self.update()

    # ---------------- 路径点：右键「移动到」 ---------------- #

    def path_point_at(self, x: float, y: float, radius_px: float = 14.0):
        """``(x, y)`` 像素位置附近**最近的已示教路径点**。

        路径点就是 :attr:`_recorded_ee_by_robot` 里的末端位姿，与
        :attr:`_recorded_by_robot` 的关节位形**一一对应**——命中第 i 个点，
        就能拿到该点对应的关节值。

        Args:
            x / y: 视口像素坐标
            radius_px: 命中半径（像素）

        Returns:
            ``(机器人名, 点索引)``；附近没有点返回 ``None``
        """
        best = None
        best_d = float(radius_px)
        for robot, ees in self._recorded_ee_by_robot.items():
            if self.scene is not None and robot not in self.scene.robots:
                continue  # 已被移出场景的机器人，它的点不该还能点
            for i, p in enumerate(ees):
                try:
                    sx, sy = self._project(p)
                except Exception:
                    continue
                d = math.hypot(sx - x, sy - y)
                if d <= best_d:
                    best_d, best = d, (robot, i)
        return best

    def contextMenuEvent(self, event):
        """右键路径点 →「移动到」：点哪个点，机器人就走到那个示教点。

        命中优先级**路径点优先**：红点是刻意去点的小目标（命中半径 14px），
        而机器人的拾取体是整台手臂的一个大包围盒——走近机器人的示教点几乎
        总被它挡住。此前按"对象优先"判断，结果靠近机器人时永远弹不出菜单
        （2026-09-24 用户反馈）。

        拖 gizmo 的手势不会被抢：右键**拖动过**就不弹菜单（见
        :attr:`_right_moved`），菜单只在"点一下就松开"时出现。
        """
        if not self._has_any_path():
            return
        # 右键拖过 = gizmo / 视角操作，不是要点菜单
        if getattr(self, "_right_moved", False):
            return
        pos = event.pos()
        hit = self.path_point_at(pos.x(), pos.y())
        if hit is None:
            return
        robot, index = hit
        menu = QtWidgets.QMenu(self)
        action = menu.addAction(tr("移动到 {robot} 的 P{n}").format(robot=robot, n=index + 1))
        action.setToolTip(tr("把这一台直接走到示教点 P{n} 的关节位形").format(n=index + 1))
        if menu.exec(event.globalPos()) is not None:
            self.pathPointMoveRequested.emit(robot, index)
