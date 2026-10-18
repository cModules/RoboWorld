"""共享 pytest fixture：GUI 测试的用户目录隔离。

背景（BUILD_QUEUE「Day1 冒出来的两个绿着的坑」之一）：此前只有部分测试文件
各自 monkeypatch 了 ``CORE_CONFIG_DIR``，其余 GUI 测试会把自动保存快照写进
**真实的** ``~/.core/autosave``——于是全量跑到后面的用例新建 ``MainWindow``
时，``showEvent → maybe_prompt_recovery`` 弹出恢复询问模态框等人点击：
单文件全绿、全量卡死在 99%（历史上卡过
``test_ui_layout.py::test_no_tab_page_is_clipped_at_minimum_window``）。

这里用 autouse fixture 把 ``CORE_CONFIG_DIR`` 指到每个用例独立的临时目录，
与 ``tools/smoke_gui_demo.py`` 的冒烟口径一致。时序保证：

- ``app/session.py`` / ``support/i18n.py`` 等都在**调用时**读环境变量，
  fixture 在用例前设置即可生效；
- 个别测试自带的 ``monkeypatch.setenv("CORE_CONFIG_DIR", ...)`` 在本 fixture
  之后执行，按需覆盖仍然有效，互不冲突。
"""

import pytest


@pytest.fixture(autouse=True)
def _isolated_config_dir(tmp_path, monkeypatch):
    """每个用例独立的用户配置目录，测试不读写真实的 ``~/.core``。"""
    monkeypatch.setenv("CORE_CONFIG_DIR", str(tmp_path / "cfg"))
