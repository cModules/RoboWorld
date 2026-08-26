#!/usr/bin/env python3
"""OpenGL 可用性诊断脚本"""
import sys

try:
    from PySide6 import QtWidgets, QtOpenGLWidgets, QtGui
    from PySide6.QtCore import Qt
except ImportError as e:
    print(f"PySide6 导入失败: {e}")
    sys.exit(1)

try:
    import OpenGL.GL as gl
except ImportError:
    print("错误: PyOpenGL 未安装。请先运行: pip install PyOpenGL")
    sys.exit(1)

# OpenGL 常量
GL_COLOR_BUFFER_BIT = 0x00004000
GL_TRIANGLES        = 0x00000004


class TestGL(QtOpenGLWidgets.QOpenGLWidget):
    def __init__(self):
        super().__init__()
        self.frame_count = 0

    def initializeGL(self):
        gl.glClearColor(0.2, 0.3, 0.4, 1.0)
        fmt = self.context().format()
        self.info = {
            "version": f"{fmt.majorVersion()}.{fmt.minorVersion()}",
            "profile": str(fmt.profile()),
            "renderer": str(self.context().functions())[:80],
        }
        print(f"[GL] 版本: {self.info['version']}, 配置: {self.info['profile']}")

    def paintGL(self):
        gl.glClear(GL_COLOR_BUFFER_BIT)

        # 测试固定管线三角形
        gl.glBegin(GL_TRIANGLES)
        gl.glColor3f(1.0, 0.0, 0.0)
        gl.glVertex2f(-0.5, -0.5)
        gl.glColor3f(0.0, 1.0, 0.0)
        gl.glVertex2f(0.5, -0.5)
        gl.glColor3f(0.0, 0.0, 1.0)
        gl.glVertex2f(0.0, 0.5)
        gl.glEnd()

        self.frame_count += 1
        if self.frame_count <= 3:
            print(f"[GL] paintGL 被调用 #{self.frame_count}, 尺寸: {self.width()}x{self.height()}")

    def resizeGL(self, w, h):
        print(f"[GL] resizeGL: {w}x{h}")


if __name__ == "__main__":
    fmt = QtGui.QSurfaceFormat()
    fmt.setVersion(2, 1)
    fmt.setProfile(QtGui.QSurfaceFormat.NoProfile)
    QtGui.QSurfaceFormat.setDefaultFormat(fmt)

    app = QtWidgets.QApplication(sys.argv)
    w = QtWidgets.QMainWindow()
    w.setWindowTitle("OpenGL Test")
    w.resize(400, 400)

    gl_widget = TestGL()
    w.setCentralWidget(gl_widget)
    w.show()

    print("窗口已显示。如果能看到彩色三角形，说明 OpenGL 固定管线正常工作。")
    print("如果窗口是纯色（蓝灰色）且没有三角形，说明固定管线在当前上下文下不可用。")
    sys.exit(app.exec())
