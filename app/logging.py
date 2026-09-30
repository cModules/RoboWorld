"""应用层结构化日志入口。"""

from __future__ import annotations

import logging

LOGGER_NAME = "robotworld.app"
logger = logging.getLogger(LOGGER_NAME)

__all__ = ["LOGGER_NAME", "logger"]
