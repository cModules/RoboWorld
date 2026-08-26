# Post-Processor Development Guide

OpenRoboOLP uses a **template-based post-processor engine**. To add support for a new robot brand or controller, you only need to write a single Python file.

## Quick Start (5 minutes)

1. Create `python/orolp/post/my_brand.py`
2. Inherit from `BasePostProcessor`
3. Implement 3 methods: `brand`, `controller`, `_convert_wp`, `_get_template`
4. Restart OpenRoboOLP — your processor is auto-discovered

## Minimal Example

```python
from orolp.post import BasePostProcessor, PostConfig
from orolp import _core

class MyBrandPostProcessor(BasePostProcessor):
    brand = "mybrand"
    controller = "ctrl_v1"

    def __init__(self):
        super().__init__(PostConfig(precision=3))

    def _convert_wp(self, wp):
        if wp.type == _core.MoveType.JOINT:
            return {
                "cmd": "MOVEJ",
                "joints": self.fmt_joints(wp.target),
                "speed": wp.speed,
            }
        return {"cmd": "UNKNOWN"}

    def _get_template(self):
        return """PROGRAM {{ meta.program_name }}
{% for wp in waypoints %}
{{ wp.cmd }} {{ wp.joints }}, Speed={{ wp.speed }}
{% endfor %}
END"
""
```

## Template Context

Your Jinja2 template receives these variables:

| Variable | Type | Description |
|----------|------|-------------|
| `waypoints` | List[dict] | Output of `_convert_wp` for each waypoint |
| `meta` | Dict | Metadata dict passed from `program.to_post()` |
| `config` | PostConfig | Formatter settings |
| `post` | BasePostProcessor | Self reference for utility methods |

## Utility Methods

| Method | Description |
|--------|-------------|
| `fmt_joints(q)` | Format joint list as `j1, j2, ...` |
| `fmt_pose(flat, order)` | Convert flattened 4x4 pose to `x, y, z, rx, ry, rz` |
| `fmt_speed(s, type)` | Format speed according to `config.speed_unit` |

## Coordinate Conventions

Different brands use different rotation orders:
- **KUKA**: ZYX (A, B, C)
- **ABB**: XYZ (quaternion internally, but RAPID uses Euler ZYX)
- **FANUC**: XYZ
- **UR**: Rotation vector (axis-angle) — requires custom formatting

Use `self.fmt_pose(wp.target, order="zyx")` or `"xyz"` accordingly.

## Testing Your Post-Processor

```python
from orolp.post import PostRegistry

# Auto-discover and test
PostRegistry.auto_discover()
post = PostRegistry.get("mybrand_ctrl_v1")
code = post.generate(waypoints, metadata={"program_name": "TEST"})
print(code)
```

## Contributing

1. Place your file in `python/orolp/post/`
2. Add a test in `python/tests/test_post.py`
3. Submit a Pull Request with a sample output file
