"""离线编程面板的代码生成与导出：后处理出码、逐台导出、工程存盘快照。

「生成代码」与「导出全部…」共用同一套选中口径（视口里选中了哪台设备）；
导出走 ``FileLink`` 离线部署通道，每台机器人各落一份**独立程序文件**
（各自的品牌扩展名、各自的机器人名）外加 ``.manifest.json`` 部署清单。
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Dict, Optional

from PySide6 import QtWidgets

from domain.link import FileLink
from service.pickplace import PointType
from support.i18n import tr

if TYPE_CHECKING:  # 仅类型标注
    from service.program import Program

    from app.views.program_panel import ProgramPanel


def generate_code(panel) -> None:
    """生成控制器代码并预览——口径与「回放」一致。

    * 视口里**选中了某台设备**（机器人/其夹爪/被夹工件都算）→ 只出**那台**的代码；
    * **没选中** → 一次给出**全部**机器人的代码（每台一段，带名字分隔），
      替代已删除的「导出全部…」。

    失败时如实抛错并清空预览——绝不留下"看起来成功"的废代码。
    """
    selected = None
    provider = getattr(panel, "selection_robot_provider", None)
    if callable(provider):
        try:
            selected = provider()
        except Exception:
            selected = None
    if selected and selected in panel._states:
        with panel._using_robot(selected):
            panel._generate_active_code()
        return

    codes = panel.generate_all_code()
    if not codes:
        return
    # 没选中 = 用户要看"整条产线"：逐台拼接，段与段之间用名字隔开
    blocks = [f"# ===== {name} =====\n{code}" for name, code in codes.items()]
    panel.code_preview.setPlainText("\n\n".join(blocks))
    panel._last_code = codes.get(panel._active) or next(iter(codes.values()))
    panel._refresh_export_all_enabled()
    panel.statusMessage.emit(tr("未选中设备——已生成全部 {n} 台的代码").format(n=len(codes)))


def _generate_active_code(panel) -> None:
    """按选择的后处理器生成**当前那台**的控制器代码并预览。"""
    program = panel.build_program()
    if program is None:
        panel.statusMessage.emit(tr("请先加载机器人并示教至少一个点"))
        return
    choice = panel.combo_post.currentData()  # None = 自动（按机器人品牌推断）
    # 程序名用**场景里的机器人名**而不是 URDF 文件名：多机同型号时后者会
    # 重名，落出 second.src / second.src 互相覆盖
    prog_name = panel._active or (panel._robot.name if panel._robot else "program")
    try:
        code = program.to_post(choice, metadata={"program_name": prog_name})
    except Exception as exc:
        panel._last_code = ""
        panel.code_preview.setPlainText(tr("生成失败：\n{exc}").format(exc=exc))
        panel.statusMessage.emit(tr("生成失败: {exc}").format(exc=exc))
        return
    panel._last_code = code
    panel.code_preview.setPlainText(code)
    panel.statusMessage.emit(
        tr("已生成 {choice} 代码，共 {lines} 行").format(
            choice=choice or tr("自动"), lines=len(code.splitlines())
        )
    )


def export_all_programs(panel) -> None:
    """把**每台**机器人各导出一份**独立程序文件**到同一个目录。

    走 ``FileLink`` 离线部署通道，**不要求**先点「生成代码」：内部逐台
    构建自己的程序再后处理，各按自己的品牌落正确扩展名（UR ``.script``
    / KUKA ``.src`` / ABB ``.mod`` / FANUC ``.tp``…），文件名与程序名用
    场景里的**机器人名**（多机同型号也不会互相覆盖），并各写
    ``.manifest.json`` 部署清单。

    选中了设备时只导那一台（与「生成代码」同一套选中口径）；没选就逐台导。
    """
    selected = None
    provider = getattr(panel, "selection_robot_provider", None)
    if callable(provider):
        try:
            selected = provider()
        except Exception:
            selected = None
    targets = [selected] if selected and selected in panel._states else list(panel._states)
    if not targets:
        panel.statusMessage.emit(tr("请先加载机器人并示教至少一个点"))
        return
    dest = QtWidgets.QFileDialog.getExistingDirectory(panel, tr("选择导出目录"), str(Path.cwd()))
    if not dest:
        return
    ok = 0
    for name in targets:
        program = panel.build_program(name)
        if program is None:
            panel.statusMessage.emit(tr("{name}：无示教点，已跳过").format(name=name))
            continue
        if panel.export_program_to(program, dest, robot=name):
            ok += 1
    if ok == 0:
        return
    panel.statusMessage.emit(
        tr("已导出 {n} 台机器人的独立程序到 {dest}").format(n=ok, dest=dest)
    )


def export_program_to(panel, program: "Program", dest: str, robot: Optional[str] = None) -> bool:
    """把 ``program`` 导出到 ``dest`` 目录（无 UI 逻辑，供测试复用）。

    Args:
        program: 已构建好的程序
        dest: 目标目录
        robot: 这台机器人的名字（决定文件名与其槽位写入）；``None`` 取当前

    Returns:
        成功 ``True``；后处理/落盘失败 ``False``（状态栏带原因）
    """
    with panel._using_robot(robot):
        return panel._export_impl(program, dest)


def _export_impl(panel, program: "Program", dest: str) -> bool:
    name = panel._active or (panel._robot.name if panel._robot else "program")
    try:
        link = FileLink(dest)
        link.connect()
        deployment = program.deploy(
            link,
            panel.combo_post.currentData(),
            name=name,
            metadata={"program_name": name},
        )
    except Exception as exc:
        panel.statusMessage.emit(tr("导出失败: {exc}").format(exc=exc))
        return False
    panel._last_code = deployment.code
    panel._refresh_export_all_enabled()
    note = tr("（{note}）").format(note=deployment.notes) if deployment.notes else ""
    panel.statusMessage.emit(
        tr("已导出：{handle}{note}").format(handle=deployment.handle, note=note)
    )
    return True


def robot_snapshot(panel) -> Dict[str, dict]:
    """各台机器人的示教点与抓取语义快照，供工程存盘。

    工程里原先只有一份 ``program`` 段（来自只显示当前那台的路径编辑器），
    多机工程存盘再打开会丢掉其余机器人教好的抓取循环——这段就是为此加的。

    Returns:
        ``{机器人名: {"joint_names": [...], "positions": [{关节名: 弧度}],
        "point_types": [...]}}``
    """
    out: Dict[str, dict] = {}
    for name, st in panel._states.items():
        if st.robot is None:
            continue
        out[name] = {
            "joint_names": list(st.joint_names),
            "positions": [{k: float(v) for k, v in p.items()} for p in st.taught],
            "point_types": [getattr(t, "value", None) for t in (st.point_types or [])],
        }
    return out


def restore_robots(panel, data: Optional[dict]) -> None:
    """把 :meth:`robot_snapshot` 的快照灌回各台机器人。

    场景里已经没有的机器人直接跳过——存档比场景多是正常的（用户删过机器
    人），不该为它凭空造一个条目出来。语义数量对不上时按默认语义重建，
    绝不猜哪个是抓取点。
    """
    for name, payload in (data or {}).items():
        if name not in panel._states:
            continue
        positions = payload.get("positions") or []
        if not positions:
            continue
        raw = [
            t
            for t in (payload.get("point_types") or [])
            if t not in (None, "", "None", "null")
        ]
        types = None
        if raw and len(raw) == len(positions):
            try:
                types = [PointType(t) for t in raw]
            except ValueError:
                types = None
        panel.set_taught_points(positions, types, robot=name)


def generate_all_code(panel) -> Dict[str, str]:
    """给**每一台**机器人各生成一份目标控制器代码。

    后处理器统一用下拉里选的那个（``None`` = 各自按品牌推断），这样一次按钮
    就能拿到"整条产线"的程序，而不是逐台点一遍。

    Returns:
        ``{机器人名: 代码}``；某台生成失败时不进结果，原因已广播到状态栏
    """
    codes: Dict[str, str] = {}
    if not panel._states:
        panel.statusMessage.emit(tr("请先加载机器人并示教至少一个点"))
        return codes
    choice = panel.combo_post.currentData()
    for name in list(panel._states):
        program = panel.build_program(name)
        if program is None:
            panel.statusMessage.emit(tr("{name}：无示教点，已跳过").format(name=name))
            continue
        try:
            codes[name] = program.to_post(choice, metadata={"program_name": name})
        except Exception as exc:
            panel.statusMessage.emit(tr("{name} 生成失败：{exc}").format(name=name, exc=exc))
    # 预览仍保持当前这台：点了"导出全部"就把预览换成别人家的代码，很容易
    # 让人以为当前编辑的那份丢了
    if panel._active in codes:
        panel._last_code = codes[panel._active]
        panel.code_preview.setPlainText(codes[panel._active])
    if codes:
        panel.statusMessage.emit(tr("已生成 {n} 台机器人的代码").format(n=len(codes)))
    return codes
