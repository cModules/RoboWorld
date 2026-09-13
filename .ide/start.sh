#!/usr/bin/env bash
# RobotWorld 云端试运行编排：虚拟显示 → GUI → VNC → noVNC 网页
set -e

# 1) 启动虚拟显示（无物理显示器也能跑 Qt GUI）
Xvfb :99 -screen 0 1280x800x24 >/tmp/xvfb.log 2>&1 &
sleep 2

# 2) 在虚拟显示上启动 RobotWorld 图形界面
DISPLAY=:99 python3 /app/rw.py >/tmp/rw.log 2>&1 &
sleep 3

# 3) VNC 采集虚拟桌面（无密码，仅开发容器内部使用）
x11vnc -display :99 -nopw -forever -rfbport 5900 >/tmp/x11vnc.log 2>&1 &

# 4) 桥接 VNC 到 WebSocket，并挂载 noVNC 网页客户端
#    前台常驻（exec）= 预览服务持续存活
exec websockify --web=/usr/share/novnc 6080 localhost:5900
