#!/usr/bin/env python3
"""
OpenRoboOLP - OLP Terminal
"""

import sys
import math
import struct
import re
import numpy as np
from pathlib import Path

try:
    from PySide6 import QtWidgets, QtCore, QtOpenGLWidgets, QtGui
    from PySide6.QtCore import Qt, Signal
    from PySide6.QtGui import QFont, QAction, QKeySequence
except ImportError:
    print("PySide6 not installed. Install with: pip install PySide6")
    sys.exit(1)

try:
    import OpenGL.GL as gl
except ImportError:
    print("PyOpenGL not installed. Install with: pip install PyOpenGL")
    sys.exit(1)


GL_COLOR_BUFFER_BIT = 0x00004000
GL_DEPTH_BUFFER_BIT = 0x00000100
GL_TRIANGLES = 0x00000004
GL_QUADS = 0x00000007
GL_QUAD_STRIP = 0x00000008
GL_TRIANGLE_FAN = 0x00000006
GL_LINES = 0x00000001
GL_LINE_STRIP = 0x00000003
GL_POINTS = 0x00000000
GL_DEPTH_TEST = 0x00000B71
GL_MODELVIEW = 0x00001700
GL_PROJECTION = 0x00001701


class RobotRenderer:
    def __init__(self):
        self.links = {}
        self.joints = {}
        self.root_link = None
        self.joint_values = {}
        self.urdf_dir = None
        self.robot_name = "Unknown"

    def load_urdf(self, path):
        import xml.etree.ElementTree as ET
        tree = ET.parse(path)
        root = tree.getroot()

        self.links.clear()
        self.joints.clear()
        self.joint_values.clear()
        self.urdf_dir = Path(path).parent
        self.robot_name = root.get("name", "Unknown")

        for link in root.findall("link"):
            name = link.get("name")
            visual = link.find("visual")
            if visual is None:
                continue

            geom = visual.find("geometry")
            material = visual.find("material")
            mat = material.find("color") if material is not None else None
            rgba = [0.7, 0.7, 0.75, 1.0]
            if mat is not None:
                rgba = [float(x) for x in mat.get("rgba", "0.7 0.7 0.75 1").split()]

            origin = visual.find("origin")
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
            if origin is not None:
                xyz = [float(x) for x in origin.get("xyz", "0 0 0").split()]
                rpy = [float(x) for x in origin.get("rpy", "0 0 0").split()]

            geo_type = None
            params = ()
            
            if geom.find("cylinder") is not None:
                cyl = geom.find("cylinder")
                geo_type = "cylinder"
                params = (float(cyl.get("radius", "0.03")), float(cyl.get("length", "0.15")))
            elif geom.find("box") is not None:
                box = geom.find("box")
                geo_type = "box"
                params = tuple(float(x) for x in box.get("size", "0.1 0.1 0.1").split())
            elif geom.find("sphere") is not None:
                sph = geom.find("sphere")
                geo_type = "sphere"
                params = (float(sph.get("radius", "0.02")),)

            self.links[name] = {
                "name": name,
                "type": geo_type,
                "params": params,
                "origin_xyz": xyz,
                "origin_rpy": rpy,
                "color": rgba[:3],
            }

        print(f"Loaded {len(self.links)} links from URDF")

        for joint in root.findall("joint"):
            jname = joint.get("name")
            jtype = joint.get("type", "fixed")
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            origin = joint.find("origin")
            jxyz = [0.0, 0.0, 0.0]
            jrpy = [0.0, 0.0, 0.0]
            if origin is not None:
                jxyz = [float(x) for x in origin.get("xyz", "0 0 0").split()]
                jrpy = [float(x) for x in origin.get("rpy", "0 0 0").split()]
            axis = [0.0, 0.0, 1.0]
            axis_el = joint.find("axis")
            if axis_el is not None:
                axis = [float(x) for x in axis_el.get("xyz", "0 0 1").split()]

            limit_lower = -math.pi
            limit_upper = math.pi
            limit = joint.find("limit")
            if limit is not None:
                limit_lower = float(limit.get("lower", "-3.14159"))
                limit_upper = float(limit.get("upper", "3.14159"))

            self.joints[jname] = {
                "name": jname,
                "type": jtype,
                "parent": parent,
                "child": child,
                "origin_xyz": jxyz,
                "origin_rpy": jrpy,
                "axis": axis,
                "limit": {"lower": limit_lower, "upper": limit_upper}
            }
            self.joint_values[jname] = 0.0

        child_links = set(j["child"] for j in self.joints.values())
        for name in self.links:
            if name not in child_links:
                self.root_link = name
                break

        print(f"Loaded {len(self.joints)} joints, root={self.root_link}")
        return len(self.links) > 0


class GLWidget(QtOpenGLWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.renderer = RobotRenderer()
        self.camera_dist = 1.2
        self.camera_az = 45.0
        self.camera_el = 30.0
        self.dragging = False
        self.last_pos = None
        self.recorded_positions = []
        self.recorded_ee_positions = []
        self.show_path = True

    def load_robot(self, path):
        success = self.renderer.load_urdf(path)
        self.update()
        return success

    def set_joints(self, q):
        self.renderer.set_joints(q)
        self.update()

    def get_joint_values(self):
        return dict(self.renderer.joint_values)

    def set_joint_values(self, joint_dict):
        for name, value in joint_dict.items():
            self.renderer.joint_values[name] = value
        self.update()

    def record_position(self):
        pos = self.get_joint_values()
        if pos:
            self.recorded_positions.append(pos)
            ee_pos = self._compute_end_effector_position()
            self.recorded_ee_positions.append(ee_pos)
        return len(self.recorded_positions)

    def clear_recorded_positions(self):
        self.recorded_positions.clear()
        self.recorded_ee_positions.clear()

    def get_recorded_positions(self):
        return self.recorded_positions

    def set_recorded_positions(self, positions):
        self.recorded_positions = positions
        # 重新计算末端执行器位置
        original_joint_values = dict(self.renderer.joint_values)
        self.recorded_ee_positions.clear()
        for pos in positions:
            self.set_joint_values(pos)
            ee_pos = self._compute_end_effector_position()
            self.recorded_ee_positions.append(ee_pos)
        self.set_joint_values(original_joint_values)

    def _compute_end_effector_position(self):
        """计算末端执行器的3D位置"""
        if self.renderer.root_link is None:
            return (0.0, 0.0, 0.0)
        
        # 找到末端链接（没有子节点的链接）
        def find_end_effector(link_name):
            has_child = False
            for joint in self.renderer.joints.values():
                if joint["parent"] == link_name:
                    has_child = True
                    result = find_end_effector(joint["child"])
                    if result is not None:
                        return result
            if not has_child:
                return link_name
            return None
        
        ee_link_name = find_end_effector(self.renderer.root_link)
        if ee_link_name is None:
            return (0.0, 0.0, 0.0)
        
        # 递归计算变换矩阵
        def compute_transform(link_name, parent_joint=None):
            transform = np.eye(4, dtype=np.float64)
            
            if parent_joint is not None:
                jx, jy, jz = parent_joint["origin_xyz"]
                jrx, jry, jrz = parent_joint["origin_rpy"]
                
                # 应用关节原点平移
                trans = np.eye(4)
                trans[0, 3] = jx
                trans[1, 3] = jy
                trans[2, 3] = jz
                transform = trans @ transform
                
                # 应用关节原点旋转 (Z-Y-X)
                if jrz != 0:
                    c, s = math.cos(jrz), math.sin(jrz)
                    rot = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                    transform = rot @ transform
                if jry != 0:
                    c, s = math.cos(jry), math.sin(jry)
                    rot = np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])
                    transform = rot @ transform
                if jrx != 0:
                    c, s = math.cos(jrx), math.sin(jrx)
                    rot = np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])
                    transform = rot @ transform
                
                # 应用关节旋转
                q = self.renderer.joint_values.get(parent_joint["name"], 0.0)
                if parent_joint["type"] == "revolute" and q != 0:
                    ax, ay, az = parent_joint["axis"]
                    # 绕任意轴旋转
                    angle = q
                    c, s = math.cos(angle), math.sin(angle)
                    t = 1 - c
                    rot = np.array([
                        [t*ax*ax + c, t*ax*ay - s*az, t*ax*az + s*ay, 0],
                        [t*ax*ay + s*az, t*ay*ay + c, t*ay*az - s*ax, 0],
                        [t*ax*az - s*ay, t*ay*az + s*ax, t*az*az + c, 0],
                        [0, 0, 0, 1]
                    ])
                    transform = rot @ transform
            
            link = self.renderer.links.get(link_name)
            if link is None:
                return transform
            
            # 应用链接原点变换
            ox, oy, oz = link["origin_xyz"]
            orx, ory, orz = link["origin_rpy"]
            
            if orx != 0 or ory != 0 or orz != 0:
                if orz != 0:
                    c, s = math.cos(orz), math.sin(orz)
                    rot = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                    transform = rot @ transform
                if ory != 0:
                    c, s = math.cos(ory), math.sin(ory)
                    rot = np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])
                    transform = rot @ transform
                if orx != 0:
                    c, s = math.cos(orx), math.sin(orx)
                    rot = np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])
                    transform = rot @ transform
            
            trans = np.eye(4)
            trans[0, 3] = ox
            trans[1, 3] = oy
            trans[2, 3] = oz
            transform = trans @ transform
            
            return transform
        
        # 从根节点到末端链接计算完整变换
        def get_path_to_ee(link_name, target_name):
            if link_name == target_name:
                return []
            for joint in self.renderer.joints.values():
                if joint["parent"] == link_name:
                    path = get_path_to_ee(joint["child"], target_name)
                    if path is not None:
                        return [joint] + path
            return None
        
        path = get_path_to_ee(self.renderer.root_link, ee_link_name)
        if path is None:
            return (0.0, 0.0, 0.0)
        
        # 计算完整变换矩阵
        transform = np.eye(4)
        current_link = self.renderer.root_link
        for joint in path:
            link = self.renderer.links.get(current_link)
            if link:
                # 应用父链接的变换
                transform = compute_transform(current_link, None) @ transform
            current_link = joint["child"]
        
        # 最后应用末端链接的变换
        final_transform = compute_transform(ee_link_name, None)
        transform = final_transform @ transform
        
        # 原点经过变换后的位置
        origin = np.array([0.0, 0.0, 0.0, 1.0])
        result = transform @ origin
        return (float(result[0]), float(result[1]), float(result[2]))

    def initializeGL(self):
        gl.glClearColor(0.15, 0.15, 0.18, 1.0)
        gl.glEnable(GL_DEPTH_TEST)

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
        cx = self.camera_dist * math.cos(el) * math.cos(az)
        cy = self.camera_dist * math.cos(el) * math.sin(az)
        cz = self.camera_dist * math.sin(el)

        eye = np.array([cx, cy, cz], dtype=np.float64)
        center = np.array([0.0, 0.0, 0.4], dtype=np.float64)
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

        rot = [
            float(s[0]), float(s[1]), float(s[2]), 0.0,
            float(u[0]), float(u[1]), float(u[2]), 0.0,
            float(-f[0]), float(-f[1]), float(-f[2]), 0.0,
            0.0, 0.0, 0.0, 1.0
        ]
        gl.glLoadMatrixf(rot)
        gl.glTranslated(float(-eye[0]), float(-eye[1]), float(-eye[2]))

        self._draw_grid()
        self._draw_robot()
        if self.show_path and len(self.recorded_positions) > 1:
            self._draw_recorded_path()

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
        if self.renderer.root_link is None:
            return
        self._draw_link_recursive(self.renderer.root_link)

    def _draw_link_recursive(self, link_name, parent_joint=None):
        link = self.renderer.links.get(link_name)
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

            q = self.renderer.joint_values.get(parent_joint["name"], 0.0)
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

        for jname, joint in self.renderer.joints.items():
            if joint["parent"] == link_name:
                self._draw_link_recursive(joint["child"], joint)

        gl.glPopMatrix()

    def _draw_cylinder(self, radius, length):
        segments = 24
        half_len = length / 2.0
        gl.glBegin(GL_QUADS)
        for i in range(segments):
            a1 = 2 * math.pi * i / segments
            a2 = 2 * math.pi * (i + 1) / segments
            x1 = radius * math.cos(a1)
            y1 = radius * math.sin(a1)
            x2 = radius * math.cos(a2)
            y2 = radius * math.sin(a2)
            gl.glNormal3f(x1, y1, 0)
            gl.glVertex3f(x1, y1, -half_len)
            gl.glVertex3f(x1, y1, half_len)
            gl.glNormal3f(x2, y2, 0)
            gl.glVertex3f(x2, y2, half_len)
            gl.glVertex3f(x2, y2, -half_len)
        gl.glEnd()
        gl.glBegin(GL_TRIANGLE_FAN)
        gl.glNormal3f(0, 0, 1)
        gl.glVertex3f(0, 0, half_len)
        for i in range(segments + 1):
            a = 2 * math.pi * i / segments
            gl.glVertex3f(radius * math.cos(a), radius * math.sin(a), half_len)
        gl.glEnd()
        gl.glBegin(GL_TRIANGLE_FAN)
        gl.glNormal3f(0, 0, -1)
        gl.glVertex3f(0, 0, -half_len)
        for i in range(segments + 1):
            a = 2 * math.pi * i / segments
            gl.glVertex3f(radius * math.cos(a), radius * math.sin(a), -half_len)
        gl.glEnd()

    def _draw_box(self, w, h, d):
        x, y, z = w/2, h/2, d/2
        gl.glBegin(GL_QUADS)
        gl.glNormal3f(0, 0, 1)
        gl.glVertex3f(-x, -y, z); gl.glVertex3f(x, -y, z)
        gl.glVertex3f(x, y, z); gl.glVertex3f(-x, y, z)
        gl.glNormal3f(0, 0, -1)
        gl.glVertex3f(-x, -y, -z); gl.glVertex3f(-x, y, -z)
        gl.glVertex3f(x, y, -z); gl.glVertex3f(x, -y, -z)
        gl.glNormal3f(1, 0, 0)
        gl.glVertex3f(x, -y, -z); gl.glVertex3f(x, y, -z)
        gl.glVertex3f(x, y, z); gl.glVertex3f(x, -y, z)
        gl.glNormal3f(-1, 0, 0)
        gl.glVertex3f(-x, -y, -z); gl.glVertex3f(-x, -y, z)
        gl.glVertex3f(-x, y, z); gl.glVertex3f(-x, y, -z)
        gl.glNormal3f(0, 1, 0)
        gl.glVertex3f(-x, y, -z); gl.glVertex3f(-x, y, z)
        gl.glVertex3f(x, y, z); gl.glVertex3f(x, y, -z)
        gl.glNormal3f(0, -1, 0)
        gl.glVertex3f(-x, -y, -z); gl.glVertex3f(x, -y, -z)
        gl.glVertex3f(x, -y, z); gl.glVertex3f(-x, -y, z)
        gl.glEnd()

    def _draw_sphere(self, radius):
        stacks = 12
        slices = 16
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

    def _draw_recorded_path(self):
        if not self.recorded_ee_positions or len(self.recorded_ee_positions) < 2:
            return
        # 绘制路径线
        gl.glLineWidth(2.0)
        gl.glColor3f(0.2, 0.8, 0.2)
        gl.glBegin(GL_LINE_STRIP)
        for pos in self.recorded_ee_positions:
            x, y, z = pos
            gl.glVertex3f(x, y, z)
        gl.glEnd()
        
        # 绘制路径点
        gl.glPointSize(5.0)
        gl.glColor3f(1.0, 0.3, 0.3)
        gl.glBegin(GL_POINTS)
        for pos in self.recorded_ee_positions:
            x, y, z = pos
            gl.glVertex3f(x, y, z)
        gl.glEnd()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.dragging = True
            self.last_pos = event.position().toPoint()

    def mouseMoveEvent(self, event):
        if self.dragging and self.last_pos:
            current_pos = event.position().toPoint()
            dx = current_pos.x() - self.last_pos.x()
            dy = current_pos.y() - self.last_pos.y()
            self.camera_az -= dx * 0.5
            self.camera_el += dy * 0.5
            self.camera_el = max(-89, min(89, self.camera_el))
            self.last_pos = current_pos
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.dragging = False

    def wheelEvent(self, event):
        delta = event.angleDelta().y() / 120
        self.camera_dist *= 0.9 ** delta
        self.camera_dist = max(0.3, min(10.0, self.camera_dist))
        self.update()


class PathCodeEditor(QtWidgets.QWidget):
    positionRecorded = Signal(int)
    playbackRequested = Signal(int)
    playbackStopped = Signal()
    clearRequested = Signal()
    positionsParsed = Signal()  # 新增：代码解析完成信号

    def __init__(self, parent=None):
        super().__init__(parent)
        self.recorded_positions = []
        self.joint_names = []  # 存储关节名称顺序
        self.playback_timer = None
        self.current_playback_index = 0
        self.playback_speed = 50
        self._setup_ui()
        self.setPlainText("# OLP Path Code\nmovej([0, -90, 90, -90, 90, 0], vel=0.5)\n")

    def _setup_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        self.code_edit = QtWidgets.QTextEdit()
        self.code_edit.setFont(QFont("Consolas", 10))
        self.code_edit.setStyleSheet("background-color: #1a1a1a; color: #e0e0e0;")
        self.code_edit.setLineWrapMode(QtWidgets.QTextEdit.NoWrap)
        self.code_edit.setTabStopDistance(20)
        layout.addWidget(self.code_edit)

        control_layout = QtWidgets.QHBoxLayout()

        self.record_btn = QtWidgets.QPushButton("● Record")
        self.record_btn.setStyleSheet("background-color: #555555; color: white;")
        self.record_btn.setCheckable(True)
        self.record_btn.clicked.connect(self._on_record_toggled)

        self.play_btn = QtWidgets.QPushButton("▶ Play")
        self.play_btn.setStyleSheet("background-color: #4CAF50; color: white;")
        self.play_btn.clicked.connect(self._on_play_clicked)

        self.stop_btn = QtWidgets.QPushButton("■ Stop")
        self.stop_btn.setStyleSheet("background-color: #f44336; color: white;")
        self.stop_btn.clicked.connect(self._on_stop_clicked)

        self.clear_btn = QtWidgets.QPushButton("Clear")
        self.clear_btn.setStyleSheet("background-color: #777777; color: white;")
        self.clear_btn.clicked.connect(self._on_clear_clicked)

        self.pos_count_label = QtWidgets.QLabel("Positions: 0")
        self.pos_count_label.setFont(QFont("Consolas", 9))

        control_layout.addWidget(self.record_btn)
        control_layout.addWidget(self.play_btn)
        control_layout.addWidget(self.stop_btn)
        control_layout.addWidget(self.clear_btn)
        control_layout.addWidget(self.pos_count_label)
        layout.addLayout(control_layout)

    def _on_record_toggled(self, checked):
        if checked:
            self.record_btn.setText("● Recording")
            self.record_btn.setStyleSheet("background-color: #f44336; color: white;")
            self.record_btn.setIcon(QtGui.QIcon())
        else:
            self.record_btn.setText("● Record")
            self.record_btn.setStyleSheet("background-color: #555555; color: white;")

    def _on_play_clicked(self):
        # 在播放前解析代码编辑器中的路径代码
        self._parse_code_to_positions()
        
        if len(self.recorded_positions) == 0:
            return
        
        # 通知主窗口更新 GLWidget 的路径可视化
        self.positionsParsed.emit()
            
        if self.playback_timer is None:
            self.playback_timer = QtCore.QTimer(self)
            self.playback_timer.timeout.connect(self._on_playback_tick)
        if not self.playback_timer.isActive():
            self.current_playback_index = 0
            self.playback_timer.start(self.playback_speed)
            self.playbackRequested.emit(0)

    def _on_stop_clicked(self):
        self._stop_playback()
        self.playbackStopped.emit()

    def _stop_playback(self):
        if self.playback_timer and self.playback_timer.isActive():
            self.playback_timer.stop()
        self.playback_timer = None
        self.current_playback_index = 0

    def _on_playback_tick(self):
        if self.current_playback_index < len(self.recorded_positions):
            self.playbackRequested.emit(self.current_playback_index)
            self.current_playback_index += 1
        else:
            self._stop_playback()
            self.playbackStopped.emit()

    def _on_clear_clicked(self):
        self._stop_playback()
        self.recorded_positions.clear()
        self.pos_count_label.setText("Positions: 0")
        self.code_edit.clear()
        self.code_edit.append("# OLP Path Code\n")
        self.clearRequested.emit()

    def record_position(self, joint_values):
        if self.record_btn.isChecked():
            self.recorded_positions.append(dict(joint_values))
            self.pos_count_label.setText(f"Positions: {len(self.recorded_positions)}")
            self.positionRecorded.emit(len(self.recorded_positions))
            self._update_code_with_positions()

    def _update_code_with_positions(self):
        self.code_edit.clear()
        self.code_edit.append("# OLP Path Code")
        self.code_edit.append(f"# Recorded positions: {len(self.recorded_positions)}")
        self.code_edit.append("")
        for i, pos in enumerate(self.recorded_positions):
            # 将弧度转换为角度显示
            joints_str = ", ".join([f"{math.degrees(v):.2f}" for v in pos.values()])
            self.code_edit.append(f"movej([{joints_str}], vel=0.5)")

    def get_recorded_positions(self):
        return self.recorded_positions

    def set_recorded_positions(self, positions):
        self.recorded_positions = list(positions)
        self.pos_count_label.setText(f"Positions: {len(self.recorded_positions)}")
        self._update_code_with_positions()
        self.update()

    def toPlainText(self):
        return self.code_edit.toPlainText()

    def setPlainText(self, text):
        self.code_edit.setPlainText(text)

    def set_joint_names(self, names):
        """设置关节名称顺序，用于解析代码"""
        self.joint_names = list(names)

    def _parse_code_to_positions(self):
        """从代码编辑器中解析路径位置数据"""
        code_text = self.code_edit.toPlainText()
        self.recorded_positions.clear()
        
        # 匹配 movej([value1, value2, ...], vel=X.X) 格式
        pattern = r'movej\(\s*\[([^\]]+)\]\s*(?:,\s*vel\s*=\s*[\d.]+)?\s*\)'
        matches = re.findall(pattern, code_text)
        
        for match in matches:
            # 解析数值列表（角度）
            try:
                values = [float(v.strip()) for v in match.split(',')]
                # 将角度转换为弧度
                rad_values = [math.radians(v) for v in values]
                
                if self.joint_names and len(values) == len(self.joint_names):
                    # 如果有关节名称，创建字典
                    pos_dict = {}
                    for i, name in enumerate(self.joint_names):
                        pos_dict[name] = rad_values[i]
                    self.recorded_positions.append(pos_dict)
                elif len(values) > 0:
                    # 如果没有关节名称，使用索引作为键
                    pos_dict = {f"joint_{i+1}": rad_values[i] for i in range(len(rad_values))}
                    self.recorded_positions.append(pos_dict)
            except ValueError:
                continue
        
        # 更新位置计数显示
        self.pos_count_label.setText(f"Positions: {len(self.recorded_positions)}")


class DevicePropertyPanel(QtWidgets.QWidget):
    jointChanged = Signal(str, float)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.sliders = {}
        self.value_labels = {}
        self.joint_layout = None
        self.log_text = None
        self.name_label = None
        self.type_label = None
        self.manufacturer_label = None
        self.dof_label = None
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        info_group = QtWidgets.QGroupBox("Robot Info")
        info_layout = QtWidgets.QFormLayout(info_group)
        self.name_label = QtWidgets.QLabel("N/A")
        self.type_label = QtWidgets.QLabel("Industrial Robot")
        self.manufacturer_label = QtWidgets.QLabel("Unknown")
        self.dof_label = QtWidgets.QLabel("0")
        info_layout.addRow("Name:", self.name_label)
        info_layout.addRow("Type:", self.type_label)
        info_layout.addRow("Manufacturer:", self.manufacturer_label)
        info_layout.addRow("DOF:", self.dof_label)
        layout.addWidget(info_group)
        
        joint_group = QtWidgets.QGroupBox("Joint Controls")
        self.joint_layout = QtWidgets.QVBoxLayout(joint_group)
        layout.addWidget(joint_group)
        
        log_group = QtWidgets.QGroupBox("Log")
        log_layout = QtWidgets.QVBoxLayout(log_group)
        self.log_text = QtWidgets.QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 9))
        self.log_text.setStyleSheet("background-color: #1a1a1a; color: #ffffff;")
        log_layout.addWidget(self.log_text)
        layout.addWidget(log_group)
        
        layout.addStretch()
    
    def setup_joints(self, renderer):
        while self.joint_layout.count() > 0:
            item = self.joint_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        
        self.sliders.clear()
        self.value_labels.clear()
        
        joint_items = []
        for name, joint in renderer.joints.items():
            if joint["type"] == "revolute":
                child = joint["child"]
                if child in renderer.links:
                    try:
                        idx = list(renderer.links.keys()).index(child)
                        joint_items.append((idx, name, joint))
                    except ValueError:
                        pass
        
        joint_items.sort(key=lambda x: x[0])
        
        for _, name, joint in joint_items:
            frame = QtWidgets.QFrame()
            frame.setFrameStyle(QtWidgets.QFrame.StyledPanel)
            flayout = QtWidgets.QVBoxLayout(frame)
            flayout.setContentsMargins(5, 5, 5, 5)
            
            header = QtWidgets.QHBoxLayout()
            label = QtWidgets.QLabel(f"<b>{name}</b>")
            self.value_labels[name] = QtWidgets.QLabel("0.0°")
            header.addWidget(label)
            header.addStretch()
            header.addWidget(self.value_labels[name])
            flayout.addLayout(header)
            
            slider = QtWidgets.QSlider(Qt.Horizontal)
            slider.setMinimum(int(math.degrees(joint["limit"]["lower"]) * 100))
            slider.setMaximum(int(math.degrees(joint["limit"]["upper"]) * 100))
            slider.setValue(0)
            slider.valueChanged.connect(lambda v, n=name: self._on_joint_changed(n, v))
            flayout.addWidget(slider)
            self.sliders[name] = slider
            
            limit_layout = QtWidgets.QHBoxLayout()
            limit_layout.addWidget(QtWidgets.QLabel(f"{math.degrees(joint['limit']['lower']):.1f}°"))
            limit_layout.addStretch()
            limit_layout.addWidget(QtWidgets.QLabel(f"{math.degrees(joint['limit']['upper']):.1f}°"))
            flayout.addLayout(limit_layout)
            
            self.joint_layout.addWidget(frame)
    
    def _on_joint_changed(self, name, value):
        angle = value / 100.0
        if name in self.value_labels:
            self.value_labels[name].setText(f"{angle:.2f}°")
        self.jointChanged.emit(name, math.radians(angle))
    
    def update_robot_info(self, renderer):
        self.name_label.setText(renderer.robot_name)
        self.dof_label.setText(str(len(renderer.joints)))
        self.log_text.append(f"Loaded: {renderer.robot_name}")
        self.log_text.append(f"Joints: {len(renderer.joints)}")

    def update_joint_displays(self, joint_values):
        for name, value in joint_values.items():
            if name in self.value_labels:
                self.value_labels[name].setText(f"{math.degrees(value):.2f}°")
            if name in self.sliders:
                self.sliders[name].blockSignals(True)
                self.sliders[name].setValue(int(math.degrees(value) * 100))
                self.sliders[name].blockSignals(False)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, urdf_path=None):
        super().__init__()
        self.setWindowTitle("OpenRoboOLP Terminal v0.1.0")
        self.resize(1400, 900)
        self._set_window_icon()
        self.setup_menu()
        self.setup_ui()
        self.urdf_path = urdf_path
    
    def showEvent(self, event):
        super().showEvent(event)
        if self.urdf_path and Path(self.urdf_path).exists():
            self.load_robot(self.urdf_path)
        self.gl_widget.update()
    
    def setup_menu(self):
        menubar = self.menuBar()
        
        file_menu = menubar.addMenu("&File")
        open_action = QAction("&Open Model...", self)
        open_action.setShortcut(QKeySequence.Open)
        open_action.triggered.connect(self.open_model)
        file_menu.addAction(open_action)
        
        save_action = QAction("&Save Path Code...", self)
        save_action.setShortcut(QKeySequence.Save)
        save_action.triggered.connect(self.save_path_code)
        file_menu.addAction(save_action)
        
        load_action = QAction("&Load Path Code...", self)
        load_action.setShortcut("Ctrl+O")
        load_action.triggered.connect(self.load_path_code)
        file_menu.addAction(load_action)
        
        file_menu.addSeparator()
        
        exit_action = QAction("E&xit", self)
        exit_action.setShortcut(QKeySequence.Quit)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        view_menu = menubar.addMenu("&View")
        reset_view = QAction("&Reset View", self)
        reset_view.setShortcut("R")
        reset_view.triggered.connect(self.reset_view)
        view_menu.addAction(reset_view)
    
    def _set_window_icon(self):
        import sys
        logo_path = Path(__file__).parent / "logo.png"
        if hasattr(sys, '_MEIPASS'):
            logo_path = Path(sys._MEIPASS) / "logo.png"
        
        if logo_path.exists():
            icon = QtGui.QIcon(str(logo_path))
            self.setWindowIcon(icon)
    
    def setup_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        
        main_layout = QtWidgets.QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(2)
        
        left_panel = QtWidgets.QWidget()
        left_panel.setMaximumWidth(350)
        left_panel.setMinimumWidth(250)
        left_layout = QtWidgets.QVBoxLayout(left_panel)
        left_layout.setContentsMargins(5, 5, 5, 5)
        
        left_header = QtWidgets.QLabel("<h3>Path Code Editor</h3>")
        left_layout.addWidget(left_header)

        self.code_editor = PathCodeEditor()
        self.code_editor.playbackRequested.connect(self.on_playback_requested)
        self.code_editor.playbackStopped.connect(self.on_playback_stopped)
        self.code_editor.clearRequested.connect(self.on_clear_requested)
        self.code_editor.positionsParsed.connect(self.on_positions_parsed)
        left_layout.addWidget(self.code_editor)
        
        main_layout.addWidget(left_panel)
        
        self.gl_widget = GLWidget(self)
        self.gl_widget.setMinimumSize(400, 300)
        main_layout.addWidget(self.gl_widget, 1)
        
        self.property_panel = DevicePropertyPanel()
        self.property_panel.setMaximumWidth(300)
        self.property_panel.setMinimumWidth(250)
        self.property_panel.jointChanged.connect(self.on_joint_changed)
        main_layout.addWidget(self.property_panel)
        
        self.statusBar().showMessage("Ready")
    
    def open_model(self):
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open Robot Model", "models", "URDF Files (*.urdf);;All Files (*)"
        )
        if file_path:
            self.load_robot(file_path)
    
    def load_robot(self, path):
        if not self.gl_widget.load_robot(path):
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to load URDF: {path}")
            return
        self.property_panel.setup_joints(self.gl_widget.renderer)
        self.property_panel.update_robot_info(self.gl_widget.renderer)
        self.property_panel.jointChanged.connect(self.on_joint_changed_for_record)
        
        # 设置关节名称顺序，用于解析路径代码
        joint_names = list(self.gl_widget.renderer.joints.keys())
        self.code_editor.set_joint_names(joint_names)
        
        recorded = self.code_editor.get_recorded_positions()
        if recorded:
            self.gl_widget.set_recorded_positions(recorded)
        self.statusBar().showMessage(f"Loaded: {Path(path).name}")
        self.gl_widget.update()

    def on_joint_changed(self, name, value):
        self.gl_widget.renderer.joint_values[name] = value
        self.gl_widget.update()

    def on_joint_changed_for_record(self, name, value):
        self.gl_widget.renderer.joint_values[name] = value
        self.gl_widget.update()
        if self.code_editor.record_btn.isChecked():
            # 当记录模式开启时，调用 GLWidget 的记录方法
            count = self.gl_widget.record_position()
            # 同步到 PathCodeEditor
            self.code_editor.recorded_positions = self.gl_widget.get_recorded_positions()
            self.code_editor.pos_count_label.setText(f"Positions: {count}")
            self.code_editor._update_code_with_positions()

    def on_playback_requested(self, index):
        positions = self.code_editor.get_recorded_positions()
        if 0 <= index < len(positions):
            self.gl_widget.set_joint_values(positions[index])
            self.property_panel.update_joint_displays(positions[index])
            self.gl_widget.update()

    def on_playback_stopped(self):
        self.statusBar().showMessage("Playback finished")

    def on_positions_parsed(self):
        """当路径代码解析完成后，更新 GLWidget 的路径可视化"""
        positions = self.code_editor.get_recorded_positions()
        if positions:
            self.gl_widget.set_recorded_positions(positions)
            self.gl_widget.update()

    def on_clear_requested(self):
        self.gl_widget.clear_recorded_positions()
        self.statusBar().showMessage("Recorded positions cleared")

    def save_path_code(self):
        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save Path Code", "", "Python Files (*.py);;All Files (*)"
        )
        if file_path:
            with open(file_path, 'w') as f:
                f.write(self.code_editor.toPlainText())
            self.statusBar().showMessage(f"Saved: {Path(file_path).name}")
    
    def load_path_code(self):
        file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load Path Code", "", "Python Files (*.py);;Text Files (*.txt);;All Files (*)"
        )
        if file_path:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    code_content = f.read()
                self.code_editor.setPlainText(code_content)
                # 解析代码并更新路径
                self.code_editor._parse_code_to_positions()
                # 更新 GLWidget 的路径可视化
                positions = self.code_editor.get_recorded_positions()
                if positions:
                    self.gl_widget.set_recorded_positions(positions)
                    self.gl_widget.update()
                self.statusBar().showMessage(f"Loaded: {Path(file_path).name}")
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "Error", f"Failed to load path code: {e}")
    
    def reset_view(self):
        self.gl_widget.camera_dist = 1.2
        self.gl_widget.camera_az = 45.0
        self.gl_widget.camera_el = 30.0
        self.gl_widget.update()


def main():
    fmt = QtGui.QSurfaceFormat()
    fmt.setVersion(2, 1)
    fmt.setProfile(QtGui.QSurfaceFormat.NoProfile)
    QtGui.QSurfaceFormat.setDefaultFormat(fmt)
    
    app = QtWidgets.QApplication(sys.argv)
    
    urdf_path = sys.argv[1] if len(sys.argv) > 1 else None
    window = MainWindow(urdf_path)
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
