"""主窗口拆解产物（ADR-002 目标态 ``app/window/main_window.py`` 的前身）。

``main.py`` 的 ``MainWindow`` 按**内聚职责**逐组搬到这里：一次搬迁 = 一个独立提交，
纪律是**只换位置、不改行为**——方法体里的 ``self.*`` 一字未改，所以调用点（含既有
测试与工具）全部无需调整。搬完的组由 ``MainWindow`` 以 mixin 方式继承。

纪律：

* 本包**不得 import main**（方向单向：入口脚本 import 本包）；已有
  ``tests/test_app_window_mixins.py`` 钉住这一点；
* 依赖探测各自完成——``main.py`` 的 ``_HAS_*`` 是它自己的导入桥，mixin 伸手去拿
  就形成了对入口脚本的反向依赖。
"""
