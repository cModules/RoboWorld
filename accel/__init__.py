"""硬件加速层（``accel``）：C++ 实现 + 它的**唯一边界**。

口径（ADR-007 / ADR-008）：**只放硬件加速代码**——C++ 源码（``src/``）、后期 GPU 与
下沉的瓶颈计算内核；目录白名单由守护 **H7** 强制。Python 侧只有本文件可以加载
``accel_native``（守护 **H2**）：拿不到就 ``has_accel = False``，上层走纯 Python 降级，
**调用方不需要知道有没有编译 C++**。

本模块**不再**提供公共 API 门面：旧写法 ``core.Robot`` / ``from core import Pose``
（以及 ``core.i18n`` / ``core.features`` / ``core.render``）已于 2026-09-26 下线，
请直接 ``from domain.robot import Robot``，见 ``docs/develop/ARCHITECTURE.md`` ADR-008。
"""

from __future__ import annotations

__all__ = ["has_accel"]

try:
    from . import accel_native as _native

    has_accel = True
except ImportError:  # pragma: no cover - 未编译 C++ 时走纯 Python 降级
    has_accel = False
    _native = None
