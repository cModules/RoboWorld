# Robot Models

This directory contains URDF/SDF robot descriptions.

## Structure
```
assets/models/          # 2026-09-26 由仓库根 models/ 并入 assets/（与 assets/cells 平级）
  abb/        # ABB
  estun/      # 埃斯顿 (Estun)
  fanuc/      # FANUC
  jaka/       # JAKA
  kuka/       # KUKA
  lerobot/    # LeRobot
  ur/         # Universal Robots
```

## Adding a New Robot

1. Download or create URDF/XACRO
2. Place in `assets/models/<brand>/<model_name>/`
3. Ensure mesh files (STL/DAE) are in `meshes/` subdirectory
4. Update `assets/models/<brand>/README.md` with DH parameters

## Recommended Sources

- **UR**: [github.com/ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot)
- **KUKA**: [github.com/ros-industrial/kuka_experimental](https://github.com/ros-industrial/kuka_experimental)
- **ABB**: [github.com/ros-industrial/abb_robot_driver](https://github.com/ros-industrial/abb_robot_driver)

## License Note

Robot CAD models may have separate licenses from the RobotWorld codebase.
Always verify redistribution rights before including proprietary meshes.
