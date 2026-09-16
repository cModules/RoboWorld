"""零件库自定义导入（world/parts_library.py，v1.3.3）的回归测试。

守的东西：

* **内容校验是真实的**——坏 schema、未知基本体、非法尺寸、重复 id 都当场抛，
  绝不静默吞零件。
* **JSON 往返无损**——schema、id、几何、颜色、备注全部保真。
* **注册进组件目录后与内置组件同一通道**——``build_component`` 出同样形状的
  Part，GUI 拖拽无需任何特判；撤销后目录恢复原样（全局状态用夹具存档还原）。
* **诚实口径钉住**——自定义零件 params=()（几何固定，不假装可调）。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

from orolp.world import build_component  # noqa: E402
from orolp.world.components import (  # noqa: E402
    COMPONENTS,
    Part,
    get_component,
)
from orolp.world.geometry import Box, Cylinder, Sphere  # noqa: E402
from orolp.world.parts_library import (  # noqa: E402
    CUSTOM_CATEGORY,
    CUSTOM_ID_PREFIX,
    CustomPartSpec,
    CustomPartsLibrary,
    PrimitiveSpec,
    register_custom_parts,
    unregister_custom_parts,
)


def _plate(name="平板", pid="plate") -> CustomPartSpec:
    return CustomPartSpec(
        id=pid,
        name=name,
        primitives=[PrimitiveSpec(kind="box", size=(100.0, 50.0, 10.0), offset=(0, 0, 5.0))],
    )


# ---------------- PrimitiveSpec / CustomPartSpec 校验 ---------------- #

def test_unknown_primitive_kind_rejected():
    with pytest.raises(ValueError, match="未知基本体类型"):
        PrimitiveSpec(kind="torus", radius=10.0).validate()


def test_box_requires_size():
    with pytest.raises(ValueError, match="size"):
        PrimitiveSpec(kind="box").validate()


def test_box_dimensions_must_be_positive():
    with pytest.raises(ValueError, match="必须为正"):
        PrimitiveSpec(kind="box", size=(100.0, 0.0, 10.0)).validate()


def test_cylinder_and_sphere_need_radius():
    with pytest.raises(ValueError, match="radius"):
        PrimitiveSpec(kind="cylinder", height=10.0).validate()
    with pytest.raises(ValueError, match="radius"):
        PrimitiveSpec(kind="sphere").validate()


def test_cylinder_needs_height():
    with pytest.raises(ValueError, match="height"):
        PrimitiveSpec(kind="cylinder", radius=10.0).validate()


def test_color_components_in_range():
    with pytest.raises(ValueError, match="0~1"):
        PrimitiveSpec(kind="sphere", radius=5.0, color=(0.2, 1.4, 0.2)).validate()


def test_part_id_must_be_identifier():
    with pytest.raises(ValueError, match="id"):
        CustomPartSpec(id="1bad", name="x", primitives=[PrimitiveSpec(kind="sphere", radius=5.0)]).validate()


def test_part_requires_name_and_primitives():
    with pytest.raises(ValueError, match="显示名"):
        CustomPartSpec(id="ok", name="  ", primitives=[PrimitiveSpec(kind="sphere", radius=5.0)]).validate()
    with pytest.raises(ValueError, match="基本体"):
        CustomPartSpec(id="ok", name="空件", primitives=[]).validate()


def test_duplicate_primitive_names_rejected():
    with pytest.raises(ValueError, match="重名"):
        CustomPartSpec(
            id="ok", name="双板",
            primitives=[
                PrimitiveSpec(kind="box", size=(1, 1, 1), name="板"),
                PrimitiveSpec(kind="box", size=(2, 2, 2), name="板"),
            ],
        ).validate()


def test_rpy_rotation_maps_x_to_y():
    """语义：offset 是平移、rpy 把几何绕零件原点转。

    90° 绕 Z：基本体局部 +X 方向应落到 +Y（与 Pose 同口径），
    而 offset 本身原样生效。
    """
    prim = PrimitiveSpec(
        kind="box", size=(10.0, 10.0, 10.0),
        offset=(100.0, 0.0, 0.0), rpy_deg=(0.0, 0.0, 90.0),
    )
    part = prim.to_part()
    assert np.allclose(part.transform[:3, 3], [100.0, 0.0, 0.0], atol=1e-9)
    assert np.allclose(
        part.transform[:3, :3] @ np.array([1.0, 0.0, 0.0]),
        [0.0, 1.0, 0.0], atol=1e-9,
    )


# ---------------- JSON 往返 ---------------- #

def test_json_roundtrip(tmp_path):
    lib = CustomPartsLibrary([
        _plate(),
        CustomPartSpec(
            id="flange", name="法兰盘", description="示例",
            primitives=[
                PrimitiveSpec(kind="cylinder", radius=80.0, height=16.0, offset=(0, 0, 8.0),
                              color=(0.4, 0.4, 0.4), name="盘体"),
            ],
        ),
    ])
    path = tmp_path / "parts.json"
    lib.save_json(path)
    back = CustomPartsLibrary.load_json(path)
    assert [p.id for p in back.parts] == ["plate", "flange"]
    assert back.parts[1].primitives[0].radius == 80.0
    assert back.parts[1].primitives[0].name == "盘体"
    assert back.parts[0].primitives[0].size == (100.0, 50.0, 10.0)


def test_bad_schema_rejected(tmp_path):
    import json
    path = tmp_path / "bad.json"
    path.write_text(json.dumps({"schema": "nope/9", "parts": []}), encoding="utf-8")
    with pytest.raises(ValueError, match="schema"):
        CustomPartsLibrary.load_json(path)


def test_library_duplicate_id_rejected():
    lib = CustomPartsLibrary([_plate()])
    with pytest.raises(ValueError, match="已存在"):
        lib.add(_plate())


def test_library_remove_unknown_rejected():
    with pytest.raises(ValueError, match="没有"):
        CustomPartsLibrary().remove("ghost")


# ---------------- 注册进组件目录 ---------------- #

@pytest.fixture()
def clean_registry():
    """全局 COMPONENTS 存档/还原：测试注册不污染其他测试。"""
    snapshot = dict(COMPONENTS)
    yield
    COMPONENTS.clear()
    COMPONENTS.update(snapshot)


def test_register_makes_part_buildable(clean_registry):
    ids = register_custom_parts(CustomPartsLibrary([_plate()]))
    assert ids == [CUSTOM_ID_PREFIX + "plate"]
    parts = build_component(ids[0])
    assert len(parts) == 1
    assert isinstance(parts[0].prim, Box)
    assert parts[0].transform[2, 3] == pytest.approx(5.0)


def test_register_is_honest_fixed_geometry(clean_registry):
    """诚实口径：自定义零件 params=()——不假装参数可调。"""
    ids = register_custom_parts(CustomPartsLibrary([_plate()]))
    spec = get_component(ids[0])
    assert spec.params == ()
    assert spec.category == CUSTOM_CATEGORY


def test_register_same_id_twice_rejected(clean_registry):
    """custom_ 前缀保证不撞内置 id；真正的撞名是同一零件注册两次。"""
    register_custom_parts(CustomPartsLibrary([_plate()]))
    with pytest.raises(ValueError, match="已存在"):
        register_custom_parts(CustomPartsLibrary([_plate()]))


def test_unregister_restores_registry(clean_registry):
    before = set(COMPONENTS)
    ids = register_custom_parts(CustomPartsLibrary([_plate(pid="a"), _plate("乙件", "b")]))
    unregister_custom_parts(ids)
    assert set(COMPONENTS) == before


def test_unregister_ignores_builtin_ids():
    """只清理带 custom_ 前缀的条目——内置组件不可能被误删。"""
    before = set(COMPONENTS)
    unregister_custom_parts(["worktable", "conveyor"])
    assert set(COMPONENTS) == before


def test_part_to_part_list_mixed_primitives():
    spec = CustomPartSpec(
        id="mix", name="组合件",
        primitives=[
            PrimitiveSpec(kind="box", size=(10.0, 10.0, 10.0)),
            PrimitiveSpec(kind="cylinder", radius=5.0, height=20.0),
            PrimitiveSpec(kind="sphere", radius=6.0),
        ],
    )
    parts = spec.to_part_list()
    assert [type(p.prim) for p in parts] == [Box, Cylinder, Sphere]
    assert all(isinstance(p, Part) for p in parts)


# ---------------- GUI：场景设备库的「导入零件…」 ---------------- #

@pytest.fixture()
def qapp():
    import os

    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtWidgets

    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _example_parts_path() -> Path:
    return REPO_ROOT / "examples" / "assets" / "custom_parts.json"


def test_panel_import_parts_file(qapp, clean_registry):
    from PySide6.QtCore import Qt

    from orolp.robot_library.panel import RobotLibraryPanel

    panel = RobotLibraryPanel(models_dir=REPO_ROOT / "models")
    n = panel.import_parts_file(_example_parts_path())
    assert n == 2
    listed = [
        panel.list_widget.item(i).data(Qt.UserRole)
        for i in range(panel.list_widget.count())
    ]
    assert {"kind": "component", "id": "custom_flange_plate"} in listed
    assert {"kind": "component", "id": "custom_l_bracket"} in listed


def test_panel_reimport_is_idempotent(qapp, clean_registry):
    """重复导入同一文件不撞名、不残留旧批（先撤上一批再注册）。"""
    from orolp.robot_library.panel import RobotLibraryPanel

    panel = RobotLibraryPanel(models_dir=REPO_ROOT / "models")
    assert panel.import_parts_file(_example_parts_path()) == 2
    assert panel.import_parts_file(_example_parts_path()) == 2
    customs = [cid for cid in COMPONENTS if cid.startswith(CUSTOM_ID_PREFIX)]
    assert sorted(customs) == ["custom_flange_plate", "custom_l_bracket"]


def test_panel_import_bad_file_reports_not_raises(qapp, clean_registry, tmp_path):
    """坏文件在按钮路径上走状态栏反馈，不当场炸 GUI。"""
    from orolp.robot_library.panel import RobotLibraryPanel

    panel = RobotLibraryPanel(models_dir=REPO_ROOT / "models")
    msgs = []
    panel.statusMessage.connect(msgs.append)
    bad = tmp_path / "bad.json"
    bad.write_text('{"schema": "wrong"}', encoding="utf-8")

    class _FakeDialog:
        @staticmethod
        def getOpenFileName(*_a, **_k):
            return str(bad), "选取"

    import PySide6.QtWidgets as QtWidgets

    monkey = pytest.MonkeyPatch()
    monkey.setattr(QtWidgets.QFileDialog, "getOpenFileName", _FakeDialog.getOpenFileName)
    try:
        panel.on_import_parts()
    finally:
        monkey.undo()
    assert msgs and "导入零件失败" in msgs[-1]
