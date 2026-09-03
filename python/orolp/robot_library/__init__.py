"""机器人库 (Robot Library)。

对标商业 OLP 软件的 eCatalog / 组件库能力，为 RobotWorld 提供：

* :mod:`.specs` —— 多品牌机器人型号规格数据库（KUKA、JAKA、ABB、FANUC、UR、埃斯顿）
* :mod:`.urdf_builder` —— 由规格程序化生成 URDF（仅使用基本几何体）
* :mod:`.catalog` —— 型号索引、搜索与筛选

典型用法::

    from orolp.robot_library.catalog import RobotCatalog

    catalog = RobotCatalog()               # 扫描 models/ 目录
    entries = catalog.search(keyword="kr")  # 关键字搜索
    entry = catalog.get("kuka/kr16_2")
    print(entry.spec.payload_kg, entry.urdf_path)
"""

from .specs import (
    BRANDS,
    ROBOTS,
    BrandSpec,
    RobotSpec,
    get_brand,
    get_robot,
    iter_robots,
)

__all__ = [
    "BRANDS",
    "ROBOTS",
    "BrandSpec",
    "RobotSpec",
    "get_brand",
    "get_robot",
    "iter_robots",
]
