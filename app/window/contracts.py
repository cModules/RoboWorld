"""窗口 mixin 的显式宿主协议。"""

from __future__ import annotations

from typing import Protocol


class MainWindowHost(Protocol):
    """窗口 mixin 依赖的最小宿主接口，避免隐式依赖任意 ``self`` 属性。"""

    status_bar: object

    def _set_project_dirty(self) -> None: ...

    def load_robot(self, urdf_path: str) -> None: ...

    def _sync_program_points(self) -> None: ...
