# Robot Models

This directory contains URDF/SDF robot descriptions.

## Structure
```
models/
  ur/         # Universal Robots
  kuka/       # KUKA
  abb/        # ABB
  fanuc/      # FANUC
  huichuan/   # 汇川 (HuiChuan)
```

## Adding a New Robot

1. Download or create URDF/XACRO
2. Place in `models/<brand>/<model_name>/`
3. Ensure mesh files (STL/DAE) are in `meshes/` subdirectory
4. Update `models/<brand>/README.md` with DH parameters

## Recommended Sources

- **UR**: [github.com/ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot)
- **KUKA**: [github.com/ros-industrial/kuka_experimental](https://github.com/ros-industrial/kuka_experimental)
- **ABB**: [github.com/ros-industrial/abb_robot_driver](https://github.com/ros-industrial/abb_robot_driver)

## License Note

Robot CAD models may have separate licenses from the OpenRoboOLP codebase.
Always verify redistribution rights before including proprietary meshes.
