"""工程恢复时点位语义兼容性回归测试。"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from service.pickplace import PointType


def test_restore_robots_ignores_empty_point_type_values():
    from PySide6 import QtWidgets
    from app.views.program_panel import ProgramPanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = ProgramPanel()
    panel._ensure_state("R1")
    panel.restore_robots(
        {
            "R1": {
                "positions": [{"j1": 0.0}, {"j1": 0.1}],
                "point_types": [None, "None"],
            }
        }
    )

    state = panel._state("R1")
    assert state.taught == [{"j1": 0.0}, {"j1": 0.1}]
    assert state.point_types is None
