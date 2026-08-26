#!/usr/bin/env python3
"""
OpenRoboOLP Python 3D Viewer (Minimal Demo)
A lightweight alternative using PySide6 + OpenGL, no C++ compilation needed.

Usage:
    python examples/gui_demo/python_viewer.py models/ur/test_6dof.urdf
"""

import sys
import math
import struct
import numpy as np
from pathlib import Path

try:
    from PySide6 import QtWidgets, QtCore, QtOpenGLWidgets, QtOpenGL
    from PySide6.QtCore import Qt
except ImportError:
    print("PySide6 not installed. Install with: pip install PySide6")
    sys.exit(1)

try:
    import OpenGL.GL as gl
except ImportError:
    print("PyOpenGL not installed. Install with: pip install PyOpenGL")
    sys.exit(1)


def load_stl(filename):
    """Load an STL file (binary or ASCII)."""
    try:
        with open(filename, 'rb') as f:
            # Check if it's a binary STL
            header = f.read(80)
            num_triangles = struct.unpack('<I', f.read(4))[0]
            
            # Quick check: if num_triangles * 50 bytes + 84 bytes equals file size, it's binary
            f.seek(0, 2)
            file_size = f.tell()
            f.seek(84)
            
            if file_size == 84 + num_triangles * 50:
                # Binary STL
                triangles = []
                for _ in range(num_triangles):
                    normal = struct.unpack('<fff', f.read(12))
                    v1 = struct.unpack('<fff', f.read(12))
                    v2 = struct.unpack('<fff', f.read(12))
                    v3 = struct.unpack('<fff', f.read(12))
                    f.read(2)  # attribute count
                    triangles.append((normal, v1, v2, v3))
                return triangles
    except Exception:
        pass
    
    # Try ASCII STL
    try:
        with open(filename, 'r') as f:
            content = f.read()
        
        lines = content.splitlines()
        triangles = []
        i = 0
        
        while i < len(lines):
            line = lines[i].strip()
            if line.startswith('facet normal'):
                parts = line.split()
                normal = (float(parts[2]), float(parts[3]), float(parts[4]))
                i += 1
                while i < len(lines) and not lines[i].strip().startswith('endfacet'):
                    if lines[i].strip().startswith('outer loop'):
                        i += 1
                        vertices = []
                        while i < len(lines) and not lines[i].strip().startswith('endloop'):
                            line = lines[i].strip()
                            if line.startswith('vertex'):
                                parts = line.split()
                                vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
                            i += 1
                        if len(vertices) == 3:
                            triangles.append((normal, vertices[0], vertices[1], vertices[2]))
                    i += 1
            i += 1
        
        return triangles
    except Exception as e:
        print(f"Error loading STL {filename}: {e}")
        return []
    
    return []

# OpenGL 常量（兼容直接引用）
GL_COLOR_BUFFER_BIT = 0x00004000
GL_DEPTH_BUFFER_BIT = 0x00000100
GL_TRIANGLES        = 0x00000004
GL_QUADS            = 0x00000007
GL_QUAD_STRIP       = 0x00000008
GL_TRIANGLE_FAN     = 0x00000006
GL_LINE_STRIP       = 0x00000003
GL_LINES            = 0x00000001
GL_DEPTH_TEST       = 0x00000B71
GL_MODELVIEW        = 0x00001700
GL_PROJECTION       = 0x00001701
GL_FLOAT            = 0x00001406


def rotation_matrix(axis, angle):
    """Rodrigues' rotation formula."""
    k = np.array(axis) / (np.linalg.norm(axis) or 1.0)
    K = np.array([[0, -k[2], k[1]],
                  [k[2], 0, -k[0]],
                  [-k[1], k[0], 0]])
    R = np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)
    return R


def euler_to_rotation(rpy):
    """Convert roll-pitch-yaw (xyz) to rotation matrix."""
    rx, ry, rz = rpy
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    R = np.array([
        [cy*cz, -cy*sz, sy],
        [cx*sz + sx*sy*cz, cx*cz - sx*sy*sz, -sx*cy],
        [sx*sz - cx*sy*cz, sx*cz + cx*sy*sz, cx*cy]
    ])
    return R


class RobotRenderer:
    def __init__(self):
        self.links = {}          # name -> link data
        self.joints = {}         # name -> joint data
        self.link_joints = {}    # link_name -> incoming joint
        self.root_link = None
        self.joint_values = {}
        self.meshes = {}         # mesh_file -> triangles
        self.mesh_display_lists = {}  # mesh_file -> display_list_id
        self.urdf_dir = None     # Directory of the URDF file

    def load_urdf(self, path):
        """Parse URDF and build kinematic tree."""
        import xml.etree.ElementTree as ET
        tree = ET.parse(path)
        root = tree.getroot()

        # Clear display lists to avoid memory leak
        import OpenGL.GL as gl
        for display_list_id in self.mesh_display_lists.values():
            try:
                gl.glDeleteLists(display_list_id, 1)
            except:
                pass
        
        self.links.clear()
        self.joints.clear()
        self.link_joints.clear()
        self.joint_values.clear()
        self.meshes.clear()
        self.mesh_display_lists.clear()
        self.urdf_dir = Path(path).parent

        # Parse links
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
            mesh_file = None
            
            if geom.find("mesh") is not None:
                mesh = geom.find("mesh")
                geo_type = "mesh"
                mesh_file = mesh.get("filename")
                params = (mesh_file,)
            elif geom.find("cylinder") is not None:
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
                "mesh_file": mesh_file,
                "origin_xyz": xyz,
                "origin_rpy": rpy,
                "color": rgba[:3],
            }

        print(f"Loaded {len(self.links)} links from URDF")

        # Parse joints
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

            self.joints[jname] = {
                "name": jname,
                "type": jtype,
                "parent": parent,
                "child": child,
                "origin_xyz": jxyz,
                "origin_rpy": jrpy,
                "axis": axis,
            }
            self.link_joints[child] = self.joints[jname]
            self.joint_values[jname] = 0.0

        # Find root link (no parent joint, or parent is world)
        all_children = set(self.link_joints.keys())
        for name in self.links:
            if name not in all_children:
                self.root_link = name
                break
        
        # Special handling: if parent is 'world', child is the root
        if self.root_link is None:
            for j in self.joints.values():
                if j["parent"] == "world" and j["child"] in self.links:
                    self.root_link = j["child"]
                    break

        print(f"Loaded {len(self.links)} links, {len(self.joints)} joints from URDF, root={self.root_link}")
        return len(self.links) > 0

    def set_joints(self, q_list):
        """Set joint values by index order."""
        joint_items = []
        for j in self.joints.values():
            child = j["child"]
            if child in self.links:
                try:
                    idx = list(self.links.keys()).index(child)
                    joint_items.append((idx, j))
                except ValueError:
                    pass
        
        joint_items.sort(key=lambda x: x[0])
        joint_names = [j["name"] for _, j in joint_items]
        
        for i, val in enumerate(q_list):
            if i < len(joint_names):
                self.joint_values[joint_names[i]] = val


class GLWidget(QtOpenGLWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.renderer = RobotRenderer()
        self.camera_dist = 1.2
        self.camera_az = 45.0
        self.camera_el = 30.0
        self.dragging = False
        self.last_pos = None

    def load_robot(self, path):
        return self.renderer.load_urdf(path)

    def set_joints(self, q):
        self.renderer.set_joints(q)
        self.update()

    def initializeGL(self):
        gl.glClearColor(0.15, 0.15, 0.18, 1.0)
        gl.glEnable(GL_DEPTH_TEST)
        
        # Enable lighting for better 3D appearance
        gl.glEnable(gl.GL_LIGHTING)
        gl.glEnable(gl.GL_LIGHT0)
        gl.glEnable(gl.GL_LIGHT1)
        
        # Light 0: Directional light from front/top
        gl.glLightfv(gl.GL_LIGHT0, gl.GL_POSITION, np.array([1.0, 1.0, 2.0, 0.0], dtype=np.float32))
        gl.glLightfv(gl.GL_LIGHT0, gl.GL_DIFFUSE, np.array([1.0, 1.0, 1.0, 1.0], dtype=np.float32))
        gl.glLightfv(gl.GL_LIGHT0, gl.GL_SPECULAR, np.array([0.3, 0.3, 0.3, 1.0], dtype=np.float32))
        
        # Light 1: Fill light from opposite side
        gl.glLightfv(gl.GL_LIGHT1, gl.GL_POSITION, np.array([-1.0, -1.0, 1.0, 0.0], dtype=np.float32))
        gl.glLightfv(gl.GL_LIGHT1, gl.GL_DIFFUSE, np.array([0.3, 0.3, 0.3, 1.0], dtype=np.float32))
        
        # Material properties
        gl.glEnable(gl.GL_COLOR_MATERIAL)
        gl.glColorMaterial(gl.GL_FRONT, gl.GL_AMBIENT_AND_DIFFUSE)
        gl.glMaterialfv(gl.GL_FRONT, gl.GL_SPECULAR, np.array([0.8, 0.8, 0.8, 1.0], dtype=np.float32))
        gl.glMaterialf(gl.GL_FRONT, gl.GL_SHININESS, 50.0)
        
        # Enable anti-aliasing
        gl.glEnable(gl.GL_LINE_SMOOTH)
        gl.glEnable(gl.GL_POLYGON_SMOOTH)
        gl.glHint(gl.GL_LINE_SMOOTH_HINT, gl.GL_NICEST)
        gl.glHint(gl.GL_POLYGON_SMOOTH_HINT, gl.GL_NICEST)
        
        # 将调试信息写入文件（避免终端缓冲问题）
        with open("viewer_debug.log", "w", encoding="utf-8") as f:
            fmt = self.context().format()
            f.write(f"initializeGL called\n")
            f.write(f"GL version: {fmt.majorVersion()}.{fmt.minorVersion()}\n")
            f.write(f"Profile: {fmt.profile()}\n")
            f.write(f"Renderer: {str(self.context().functions())[:100]}\n")

    def paintGL(self):
        gl.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        w, h = self.width(), self.height()

        # Projection
        gl.glMatrixMode(GL_PROJECTION)
        gl.glLoadIdentity()
        aspect = w / max(1, h)
        fov = 45.0
        near, far = 0.01, 100.0
        top = near * math.tan(math.radians(fov / 2))
        right = top * aspect
        gl.glFrustum(-right, right, -top, top, near, far)

        # View matrix (exact gluLookAt implementation)
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

        # Debug: draw a visible red triangle at origin
        gl.glDisable(GL_DEPTH_TEST)
        gl.glBegin(GL_TRIANGLES)
        gl.glColor3f(1.0, 0.0, 0.0)
        gl.glVertex3f(0.2, 0, 0)
        gl.glVertex3f(0, 0.2, 0)
        gl.glVertex3f(0, 0, 0.2)
        gl.glEnd()
        gl.glEnable(GL_DEPTH_TEST)

        self._draw_grid(gl)
        self._draw_robot(gl)

        # 调试信息追加到日志
        with open("viewer_debug.log", "a", encoding="utf-8") as f:
            f.write(f"paintGL: size={w}x{h}, links={len(self.renderer.links)}, root={self.renderer.root_link}\n")

    def _draw_grid(self, gl):
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

    def _draw_coordinate_system(self, gl, length=0.1):
        """Draw a coordinate system with X (red), Y (green), Z (blue) axes."""
        gl.glBegin(GL_LINES)
        
        # X axis - red
        gl.glColor3f(1.0, 0.0, 0.0)
        gl.glVertex3f(0, 0, 0)
        gl.glVertex3f(length, 0, 0)
        
        # Y axis - green
        gl.glColor3f(0.0, 1.0, 0.0)
        gl.glVertex3f(0, 0, 0)
        gl.glVertex3f(0, length, 0)
        
        # Z axis - blue
        gl.glColor3f(0.0, 0.0, 1.0)
        gl.glVertex3f(0, 0, 0)
        gl.glVertex3f(0, 0, length)
        
        gl.glEnd()

    def _draw_robot(self, gl):
        """Recursively draw the kinematic tree with proper transforms."""
        if self.renderer.root_link is None:
            return
        self._draw_link_recursive(gl, self.renderer.root_link)

    def _draw_link_recursive(self, gl, link_name, parent_joint=None):
        r = self.renderer
        link = r.links.get(link_name)
        if link is None:
            return

        gl.glPushMatrix()

        # Apply parent joint transform (for non-root links)
        if parent_joint is not None:
            # Apply joint origin translation first
            jx, jy, jz = parent_joint["origin_xyz"]
            gl.glTranslatef(jx, jy, jz)
            
            # Apply joint origin rotation (URDF rpy = Z-Y-X extrinsic order)
            jrx, jry, jrz = parent_joint["origin_rpy"]
            if any(v != 0 for v in [jrx, jry, jrz]):
                gl.glRotatef(math.degrees(jrz), 0, 0, 1)  # yaw (Z)
                gl.glRotatef(math.degrees(jry), 0, 1, 0)  # pitch (Y)
                gl.glRotatef(math.degrees(jrx), 1, 0, 0)  # roll (X)
            
            # Draw coordinate system at joint position (before joint rotation)
            self._draw_coordinate_system(gl, 0.08)
            
            # Draw joint indicator at joint position
            self._draw_joint_indicator(gl, 0.02)
            
            # Apply joint rotation (if revolute)
            q = r.joint_values.get(parent_joint["name"], 0.0)
            if parent_joint["type"] == "revolute" and q != 0:
                ax, ay, az = parent_joint["axis"]
                gl.glRotatef(math.degrees(q), ax, ay, az)
        else:
            # Root link: draw indicator and coordinate system at origin
            self._draw_coordinate_system(gl, 0.1)
            self._draw_joint_indicator(gl, 0.02)

        # Apply link's visual origin (relative to link coordinate system)
        ox, oy, oz = link["origin_xyz"]
        orx, ory, orz = link["origin_rpy"]
        if any(v != 0 for v in [orx, ory, orz]):
            gl.glRotatef(math.degrees(orz), 0, 0, 1)  # yaw (Z)
            gl.glRotatef(math.degrees(ory), 0, 1, 0)  # pitch (Y)
            gl.glRotatef(math.degrees(orx), 1, 0, 0)  # roll (X)
        gl.glTranslatef(ox, oy, oz)

        # Draw geometry
        gl.glColor3f(*link["color"])
        geo = link["type"]
        if geo == "cylinder":
            radius, length = link["params"]
            self._draw_cylinder(gl, radius, length)
            # Add a small marker on +X side to visualize rotation
            gl.glPushMatrix()
            gl.glColor3f(0.95, 0.95, 0.1)  # Bright yellow marker
            marker_size = radius * 0.25
            gl.glTranslatef(radius + marker_size*0.5, 0, 0)
            self._draw_box(gl, marker_size, marker_size, marker_size*2)
            gl.glPopMatrix()
        elif geo == "box":
            w, h, d = link["params"]
            self._draw_box(gl, w, h, d)
            # Add a small marker to visualize rotation
            gl.glPushMatrix()
            gl.glColor3f(0.95, 0.95, 0.1)  # Bright yellow marker
            marker_size = min(w, h, d) * 0.25
            gl.glTranslatef(w*0.5 + marker_size*0.5, 0, 0)
            self._draw_box(gl, marker_size, marker_size, marker_size*2)
            gl.glPopMatrix()
        elif geo == "sphere":
            radius = link["params"][0]
            self._draw_sphere(gl, radius)
            # Add a small marker on +X side to visualize rotation
            gl.glPushMatrix()
            gl.glColor3f(0.95, 0.95, 0.1)  # Bright yellow marker
            marker_size = radius * 0.25
            gl.glTranslatef(radius + marker_size*0.5, 0, 0)
            self._draw_sphere(gl, marker_size*0.8)
            gl.glPopMatrix()
        elif geo == "mesh":
            mesh_file = link["mesh_file"]
            if mesh_file:
                self._draw_mesh(gl, mesh_file, link["color"])

        # Recursively draw child links through joints (transform is accumulated)
        for jname, joint in r.joints.items():
            if joint["parent"] == link_name:
                self._draw_link_recursive(gl, joint["child"], joint)

        # Pop matrix AFTER drawing all child links
        gl.glPopMatrix()

    def _draw_joint_indicator(self, gl, size):
        """Draw a small RGB axis at joint origin, plus a small indicator box on X-axis to show rotation."""
        gl.glDisable(GL_DEPTH_TEST)
        gl.glBegin(GL_LINES)
        # X - red
        gl.glColor3f(1.0, 0.2, 0.2)
        gl.glVertex3f(0, 0, 0); gl.glVertex3f(size, 0, 0)
        # Y - green
        gl.glColor3f(0.2, 1.0, 0.2)
        gl.glVertex3f(0, 0, 0); gl.glVertex3f(0, size, 0)
        # Z - blue
        gl.glColor3f(0.2, 0.4, 1.0)
        gl.glVertex3f(0, 0, 0); gl.glVertex3f(0, 0, size)
        gl.glEnd()
        
        # Draw a small box on X-axis to visualize rotation
        gl.glBegin(GL_QUADS)
        gl.glColor3f(0.9, 0.1, 0.1)
        box_size = size * 0.3
        gl.glVertex3f(size*0.7, -box_size, -box_size)
        gl.glVertex3f(size*1.3, -box_size, -box_size)
        gl.glVertex3f(size*1.3, box_size, -box_size)
        gl.glVertex3f(size*0.7, box_size, -box_size)
        
        gl.glVertex3f(size*0.7, -box_size, box_size)
        gl.glVertex3f(size*1.3, -box_size, box_size)
        gl.glVertex3f(size*1.3, box_size, box_size)
        gl.glVertex3f(size*0.7, box_size, box_size)
        
        gl.glColor3f(0.7, 0.1, 0.1)
        gl.glVertex3f(size*0.7, -box_size, -box_size)
        gl.glVertex3f(size*0.7, -box_size, box_size)
        gl.glVertex3f(size*0.7, box_size, box_size)
        gl.glVertex3f(size*0.7, box_size, -box_size)
        
        gl.glVertex3f(size*1.3, -box_size, -box_size)
        gl.glVertex3f(size*1.3, -box_size, box_size)
        gl.glVertex3f(size*1.3, box_size, box_size)
        gl.glVertex3f(size*1.3, box_size, -box_size)
        
        gl.glColor3f(0.5, 0.1, 0.1)
        gl.glVertex3f(size*0.7, -box_size, -box_size)
        gl.glVertex3f(size*1.3, -box_size, -box_size)
        gl.glVertex3f(size*1.3, -box_size, box_size)
        gl.glVertex3f(size*0.7, -box_size, box_size)
        
        gl.glVertex3f(size*0.7, box_size, -box_size)
        gl.glVertex3f(size*1.3, box_size, -box_size)
        gl.glVertex3f(size*1.3, box_size, box_size)
        gl.glVertex3f(size*0.7, box_size, box_size)
        gl.glEnd()
        gl.glEnable(GL_DEPTH_TEST)

    def _draw_cylinder(self, gl, radius, length):
        segments = 24
        half_len = length / 2.0
        # Body (centered at origin, along Z-axis)
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
        # Top cap
        gl.glBegin(GL_TRIANGLE_FAN)
        gl.glNormal3f(0, 0, 1)
        gl.glVertex3f(0, 0, half_len)
        for i in range(segments + 1):
            a = 2 * math.pi * i / segments
            gl.glVertex3f(radius * math.cos(a), radius * math.sin(a), half_len)
        gl.glEnd()
        # Bottom cap
        gl.glBegin(GL_TRIANGLE_FAN)
        gl.glNormal3f(0, 0, -1)
        gl.glVertex3f(0, 0, -half_len)
        for i in range(segments + 1):
            a = 2 * math.pi * i / segments
            gl.glVertex3f(radius * math.cos(a), radius * math.sin(a), -half_len)
        gl.glEnd()

    def _draw_box(self, gl, w, h, d):
        x, y, z = w/2, h/2, d/2
        gl.glBegin(GL_QUADS)
        # Front (+Z)
        gl.glNormal3f(0, 0, 1)
        gl.glVertex3f(-x, -y, z); gl.glVertex3f(x, -y, z)
        gl.glVertex3f(x, y, z); gl.glVertex3f(-x, y, z)
        # Back (-Z)
        gl.glNormal3f(0, 0, -1)
        gl.glVertex3f(-x, -y, -z); gl.glVertex3f(-x, y, -z)
        gl.glVertex3f(x, y, -z); gl.glVertex3f(x, -y, -z)
        # Right (+X)
        gl.glNormal3f(1, 0, 0)
        gl.glVertex3f(x, -y, -z); gl.glVertex3f(x, y, -z)
        gl.glVertex3f(x, y, z); gl.glVertex3f(x, -y, z)
        # Left (-X)
        gl.glNormal3f(-1, 0, 0)
        gl.glVertex3f(-x, -y, -z); gl.glVertex3f(-x, -y, z)
        gl.glVertex3f(-x, y, z); gl.glVertex3f(-x, y, -z)
        # Top (+Y)
        gl.glNormal3f(0, 1, 0)
        gl.glVertex3f(-x, y, -z); gl.glVertex3f(-x, y, z)
        gl.glVertex3f(x, y, z); gl.glVertex3f(x, y, -z)
        # Bottom (-Y)
        gl.glNormal3f(0, -1, 0)
        gl.glVertex3f(-x, -y, -z); gl.glVertex3f(x, -y, -z)
        gl.glVertex3f(x, -y, z); gl.glVertex3f(-x, -y, z)
        gl.glEnd()

    def _draw_sphere(self, gl, radius):
        # Icosphere-like approximation with quad strips
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

    def _draw_mesh(self, gl, mesh_file, color):
        """Draw an STL mesh using display list caching for performance."""
        r = self.renderer
        
        # Check if mesh is already loaded
        if mesh_file not in r.meshes:
            # Resolve path relative to URDF file
            mesh_path = (r.urdf_dir / mesh_file).resolve()
            print(f"Loading mesh: {mesh_path}")
            triangles = load_stl(str(mesh_path))
            r.meshes[mesh_file] = triangles
            print(f"Loaded {len(triangles)} triangles")
        
        # Check if display list exists, create if not
        if mesh_file not in r.mesh_display_lists:
            triangles = r.meshes.get(mesh_file, [])
            if not triangles:
                return
            
            # Create display list
            display_list_id = gl.glGenLists(1)
            gl.glNewList(display_list_id, gl.GL_COMPILE)
            gl.glBegin(GL_TRIANGLES)
            for normal, v1, v2, v3 in triangles:
                gl.glNormal3f(*normal)
                gl.glVertex3f(*v1)
                gl.glVertex3f(*v2)
                gl.glVertex3f(*v3)
            gl.glEnd()
            gl.glEndList()
            r.mesh_display_lists[mesh_file] = display_list_id
            print(f"Created display list for {mesh_file}")
        
        # Draw using display list
        display_list_id = r.mesh_display_lists.get(mesh_file)
        if display_list_id:
            gl.glColor3f(*color)
            gl.glCallList(display_list_id)

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


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, urdf_path=None):
        super().__init__()
        self.setWindowTitle("OpenRoboOLP Python Viewer v0.1.0")
        self.resize(1200, 800)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        self.gl_widget = GLWidget(self)
        layout.addWidget(self.gl_widget, 1)

        panel = QtWidgets.QWidget(self)
        panel.setMaximumWidth(350)
        panel_layout = QtWidgets.QVBoxLayout(panel)
        panel_layout.addWidget(QtWidgets.QLabel("<h2>Joint Control</h2>"))

        self.sliders = []
        self.value_labels = []

        layout.addWidget(panel)
        self.panel = panel

        if urdf_path and Path(urdf_path).exists():
            self.load_robot(urdf_path)

    def load_robot(self, path):
        if not self.gl_widget.load_robot(path):
            QtWidgets.QMessageBox.critical(self, "Error", f"Failed to load URDF: {path}")
            return

        # Clear old controls
        while self.panel.layout().count() > 1:
            item = self.panel.layout().takeAt(1)
            if item.widget():
                item.widget().deleteLater()

        self.sliders.clear()
        self.value_labels.clear()

        # Create sliders sorted by link order
        renderer = self.gl_widget.renderer
        joint_items = []
        for j in renderer.joints.values():
            child = j["child"]
            if child in renderer.links:
                try:
                    idx = list(renderer.links.keys()).index(child)
                    joint_items.append((idx, j))
                except ValueError:
                    pass
        
        joint_items.sort(key=lambda x: x[0])
        joint_list = [(j["child"], j) for _, j in joint_items]

        for i, (_, joint) in enumerate(joint_list):
            jname = joint["name"]
            axis = joint["axis"]
            axis_str = f"[{axis[0]:.0f},{axis[1]:.0f},{axis[2]:.0f}]"
            row = QtWidgets.QWidget(self.panel)
            row_layout = QtWidgets.QHBoxLayout(row)

            label = QtWidgets.QLabel(f"{jname} {axis_str}:", row)
            row_layout.addWidget(label)

            slider = QtWidgets.QSlider(Qt.Horizontal, row)
            slider.setRange(-1800, 1800)
            slider.setValue(0)
            row_layout.addWidget(slider, 1)

            val_label = QtWidgets.QLabel("0.0°", row)
            val_label.setMinimumWidth(50)
            row_layout.addWidget(val_label)

            self.panel.layout().addWidget(row)
            self.sliders.append(slider)
            self.value_labels.append(val_label)

            slider.valueChanged.connect(
                lambda v, lbl=val_label, jn=jname: self.on_joint_changed(v, lbl, jn)
            )

        btn_layout = QtWidgets.QHBoxLayout()
        zero_btn = QtWidgets.QPushButton("Zero All")
        home_btn = QtWidgets.QPushButton("Home Pose")
        btn_layout.addWidget(zero_btn)
        btn_layout.addWidget(home_btn)
        self.panel.layout().addLayout(btn_layout)

        zero_btn.clicked.connect(self.zero_all)
        home_btn.clicked.connect(self.home_pose)
        self.panel.layout().addStretch()

        # Set default pose
        self.home_pose()

    def on_joint_changed(self, value, label, jname):
        deg = value / 10.0
        label.setText(f"{deg:.1f}°")
        renderer = self.gl_widget.renderer
        renderer.joint_values[jname] = deg * math.pi / 180.0
        self.gl_widget.update()

    def zero_all(self):
        for s in self.sliders:
            s.setValue(0)

    def home_pose(self):
        values = [0, -900, 900, -900, 900, 0]
        for i, s in enumerate(self.sliders):
            if i < len(values):
                s.setValue(values[i])


if __name__ == "__main__":
    # 强制 OpenGL 2.1 兼容模式，支持固定管线 (glBegin/glMatrixMode 等)
    from PySide6.QtGui import QSurfaceFormat
    fmt = QSurfaceFormat()
    fmt.setVersion(2, 1)
    fmt.setProfile(QSurfaceFormat.NoProfile)
    QSurfaceFormat.setDefaultFormat(fmt)

    app = QtWidgets.QApplication(sys.argv)

    urdf_path = sys.argv[1] if len(sys.argv) > 1 else None
    window = MainWindow(urdf_path)
    window.show()

    sys.exit(app.exec())
