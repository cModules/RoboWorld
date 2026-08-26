#!/usr/bin/env python3
"""
OpenRoboOLP Example: Pick and Place
Demonstrates joint moves, linear moves, tool frames, and post-processor generation.
"""

import orolp as orp

# Load a test 6-DOF robot model
robot = orp.Robot("models/ur/test_6dof.urdf", name="TestRobot_6DOF")

# Define tool frame: 150mm gripper offset along Z
robot.tool = orp.Pose.from_xyz_rpy(0, 0, 150, 0, 0, 0)

# Define work object (table) frame
wobj = orp.Pose.from_xyz_rpy(500, 200, 0, 0, 0, 90)

# Define pick and place poses relative to work object
pick = orp.Pose.from_xyz_rpy(0, 0, 50, 0, 0, 0).relative_to(wobj)
place = orp.Pose.from_xyz_rpy(100, 100, 50, 0, 0, 0).relative_to(wobj)

# Build motion program (chainable API)
program = (
    robot.moveJ([0, -90, 90, -90, 90, 0], speed=80)   # Home position
    .then(robot.moveL(pick, speed=50, blend=5))        # Approach pick
    .then(robot.moveL(pick.with_z_offset(-30)))         # Descend
    # (gripper close would go here via IO)
    .then(robot.moveL(pick.with_z_offset(50)))          # Lift
    .then(robot.moveL(place, speed=80, blend=5))         # Move to place
    .then(robot.moveL(place.with_z_offset(-30)))         # Descend
    # (gripper open would go here)
    .then(robot.moveL(place.with_z_offset(50)))          # Retreat
)

# Simulate and get cycle time estimate
traj = program.simulate(collision_check=False)
print(f"Estimated cycle time: {traj.duration():.2f} seconds")

# Generate native robot code for different brands
print("\n--- URScript ---")
print(program.to_post("ur_polyscope"))

print("\n--- KUKA KRL ---")
print(program.to_post("kuka_krc4"))

print("\n--- ABB RAPID ---")
print(program.to_post("abb_rapid"))

print("\n--- FANUC TP ---")
print(program.to_post("fanuc_tp"))

print("\n--- HuiChuan InoTeach ---")
print(program.to_post("huichuan_inoteach"))
