#!/usr/bin/env python3
"""
OpenRoboOLP Example: Welding Path with Weave Pattern
Demonstrates Cartesian path generation for continuous processes.
"""

import orolp as orp
import numpy as np

robot = orp.Robot("models/kuka/kr6_r900.urdf", name="KUKA_1")
robot.tool = orp.Pose.from_xyz_rpy(0, 0, 200, 0, 0, 0)  # Welding torch

# Define seam path: straight line with sinusoidal weave
start_pt = np.array([400, 200, 100])
end_pt = np.array([800, 200, 100])
num_points = 20

waypoints = []
for i in range(num_points):
    t = i / (num_points - 1)
    base = start_pt + t * (end_pt - start_pt)
    # Weave amplitude 5mm, frequency 3 cycles over seam
    weave = 5 * np.sin(2 * np.pi * 3 * t)
    pos = base + np.array([0, weave, 0])
    pose = orp.Pose.from_xyz_rpy(pos[0], pos[1], pos[2], 0, 0, 0)
    wp = robot.moveL(pose, speed=30, blend=2)
    waypoints.append(wp)

# Chain all waypoints
program = waypoints[0]
for wp in waypoints[1:]:
    program = program.then(wp)

# Simulate
traj = program.simulate()
print(f"Welding path duration: {traj.duration():.2f} s")

# Export to KUKA
krl = program.to_post("kuka_krc4")
with open("welding_seam.src", "w") as f:
    f.write(krl)
print("Saved to welding_seam.src")
