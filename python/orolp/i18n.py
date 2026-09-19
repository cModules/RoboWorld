"""轻量国际化（v1.0.10）：`tr()` 运行时翻译 + 语言持久化。

设计取舍
========

* **zh 是源语言**：代码里的中文字面量就是 key，``tr()`` 在 zh（默认）下
  恒等返回——既有测试与文档零回归；en 通过目录查表，查不到就原样返回
  （宁可漏翻一句中文，也不让界面出现占位符）。
* **显式配置优先**，不做 locale 自动探测：语言影响的是"用户看到什么"，
  必须可预期。优先级：环境变量 ``OROLP_LANG`` > 配置文件（默认
  ``~/.orolp/language.json``，可用 ``OROLP_CONFIG_DIR`` 重定向）> zh。
* **f-string 一律转成模板**：``tr("已载入 {name}（{n} 个对象）").format(...)``
  ——目录里的 key 是带占位符的模板本身，占位符用命名参数，英文语序可调整。
* **GUI 切换语言需要重启**（面板在构造期写入文案，做全量 retranslate
  的复杂度远大于价值）；库层消息（planner Issue / 报告 / 后处理注释）
  自 v1.1 起同样走 ``tr()``——消息在**生成时刻**按当前语言定稿，与 GUI
  一致（切语言需重启后重建面板/重跑校验）。

用法::

    from orolp.i18n import tr, set_language, current_language

    set_language("en")
    tr("删除")                  # -> "Delete"
    tr("已载入 {name}").format(name="scene1")
"""

from __future__ import annotations

import json
import os
from pathlib import Path

__all__ = [
    "LANG_ZH",
    "LANG_EN",
    "LANGUAGE_NAMES",
    "available_languages",
    "current_language",
    "set_language",
    "tr",
]

LANG_ZH = "zh"
LANG_EN = "en"

#: 语言代码 -> 显示名（菜单里展示用）
LANGUAGE_NAMES = {LANG_ZH: "中文", LANG_EN: "English"}

_LANGUAGE: str | None = None


def _config_path() -> Path:
    """语言配置文件路径。``OROLP_CONFIG_DIR`` 供测试/便携部署重定向。"""
    base = os.environ.get("OROLP_CONFIG_DIR")
    if base:
        return Path(base) / "language.json"
    return Path.home() / ".orolp" / "language.json"


def _detect() -> str:
    """启动时解析语言：环境变量 > 配置文件 > 默认 zh。"""
    env = os.environ.get("OROLP_LANG", "").strip().lower()
    if env in LANGUAGE_NAMES:
        return env
    try:
        data = json.loads(_config_path().read_text(encoding="utf-8"))
        lang = str(data.get("language", "")).strip().lower()
        if lang in LANGUAGE_NAMES:
            return lang
    except Exception:
        pass
    return LANG_ZH


def current_language() -> str:
    """当前语言代码（惰性解析，进程内缓存）。"""
    global _LANGUAGE
    if _LANGUAGE is None:
        _LANGUAGE = _detect()
    return _LANGUAGE


def set_language(lang: str, *, save: bool = True) -> str:
    """设置当前语言；``save=True`` 时写入配置文件供下次启动沿用。"""
    global _LANGUAGE
    if lang not in LANGUAGE_NAMES:
        raise ValueError(
            f"unsupported language {lang!r}; available: {sorted(LANGUAGE_NAMES)}"
        )
    _LANGUAGE = lang
    if save:
        path = _config_path()
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(
                json.dumps({"language": lang}, ensure_ascii=False, indent=1),
                encoding="utf-8",
            )
        except Exception:
            pass  # 配置盘写失败不该挡住切换
    return lang


def available_languages() -> list[tuple[str, str]]:
    """(代码, 显示名) 列表，菜单/设置界面直接用。"""
    return [(code, LANGUAGE_NAMES[code]) for code in (LANG_ZH, LANG_EN)]


def tr(text: str) -> str:
    """翻译一条 UI 文案。zh 恒等返回；en 查目录，未收录原样返回。"""
    if current_language() == LANG_ZH:
        return text
    return _CATALOG_EN.get(text, text)


# ---------------------------------------------------------------------------
# 英文目录（key = 源码中的中文文案原样；模板用命名占位符）
# ---------------------------------------------------------------------------

_CATALOG_EN: dict[str, str] = {
    # --- app.py：菜单 / 全局 ---
    "抓取工作站演示（一键）(&D)": "Grasping workstation demo (one-click)(&D)",
    "载入预置工作站(&W)...": "Load preset workcell(&W)...",
    "预置工作站": "Preset workcells",
    "选择工作站:": "Select workcell:",
    "抓取工作站演示": "Grasping workstation demo",
    "显示&机器组件库": "Show &machine library",
    "插件": "Plugins",
    "插件状态": "Plugin status",
    "插件状态…": "Plugin status…",
    "插件覆盖层": "Plugin overlays",
    "重新扫描已安装插件": "Rescan installed plugins",
    "重新扫描机器组件库": "Rescan machine library",
    "当前环境无插件层（orolp.plugins 不可用）。": "Plugin layer unavailable (orolp.plugins is not installed).",
    "未找到预置场景（{cells} 下没有 .json）": "No preset scenes found (no .json under {cells})",
    "已载入 {name}（{n} 个对象）": "Loaded {name} ({n} objects)",
    "已载入: {name}": "Loaded: {name}",
    "关节数: {n}": "Axes: {n}",
    "模板构建失败：{kind}: {msg}": "Template build failed: {kind}: {msg}",
    "示例程序生成失败：{kind}: {msg}": "Demo program build failed: {kind}: {msg}",
    "离线编程机器人初始化失败，无法生成示例程序。": "Offline-programming robot init failed; cannot build demo program.",
    "所需模块不可用（模板/环境建模/离线编程/抓取工艺）。": "Required modules unavailable (templates / world modeling / OLP / grasping process).",
    "抓取工作站演示已就绪：{n} 个示例示教点{cycle_txt}，回放中（可暂停/单步/停止）": "Grasping workstation demo ready: {n} demo taught points{cycle_txt}, playing back (pause/step/stop available)",
    "，节拍 {t} s": ", cycle {t} s",
    "节拍 {t} s": "Cycle {t} s",
    "节拍 —": "Cycle —",
    "干涉 {n}": "Interference {n}",
    "干涉 —": "Interference —",
    "报错 {n}": "{n} errors",
    "无报错": "No errors",
    "当前没有示教点。\n请先在 3D 视口中调整关节角度并点击「记录」按钮。": "No taught points yet.\nAdjust joint angles in the 3D viewport and click Record first.",
    "重新扫描失败：{exc}": "Rescan failed: {exc}",
    "重新扫描完成：发现 {discovered} 个入口点，登记 {registered} 个，跳过 {skipped} 个，失败 {failed} 个。": "Rescan finished: found {discovered} entry point(s), registered {registered}, skipped {skipped}, failed {failed}.",
    # 缓存可观测性（v1.2.4）
    "缓存状态…": "Cache status…",
    "清空缓存与统计": "Clear caches & statistics",
    "缓存状态": "Cache status",
    "当前环境无缓存层（orolp.incremental 不可用）。": "Cache layer unavailable in this environment (orolp.incremental missing).",
    "内容寻址缓存不会给出过期结果；这里的数字解释\"活少干了多少\"。": "Content-addressed caches can never serve stale results; these numbers show how much work they saved.",
    "已清空全部缓存并把统计归零——下次校验将全量真算。": "All caches cleared and statistics reset — the next validation will recompute everything.",
    "规划与分段缓存（TrajectoryCache）": "Plan & segment cache (TrajectoryCache)",
    "干涉检查结果缓存（CollisionResultCache）": "Collision-check result cache (CollisionResultCache)",
    "力矩校验结果缓存（DynamicsResultCache）": "Torque-check result cache (DynamicsResultCache)",
    "已关闭": "Disabled",
    "整程序 {entries}/{max} 份（{wp} 路点） · 命中 {hits} / 未命中 {misses}（命中率 {rate:.0%}）": "Full-program {entries}/{max} snapshots ({wp} waypoints) · {hits} hits / {misses} misses ({rate:.0%} hit rate)",
    "分段 {entries}/{wp_max} 条（{wp} 路点） · 命中 {hits} / 未命中 {misses}（命中率 {rate:.0%}）": "Per-segment {entries}/{wp_max} entries ({wp} waypoints) · {hits} hits / {misses} misses ({rate:.0%} hit rate)",
    "LRU 淘汰 {n} 次": "{n} LRU eviction(s)",
    "条目 {entries}/{max} · 命中 {hits} / 未命中 {misses}（命中率 {rate:.0%}）· 存储 {stores} · 淘汰 {evictions}": "{entries}/{max} entries · {hits} hits / {misses} misses ({rate:.0%} hit rate) · {stores} stores · {evictions} evictions",
    "本轮还没有缓存活动（计数全为 0）。": "No cache activity yet this session (all counters are zero).",
    "机器人模型不可用：{urdf}": "Robot model unavailable: {urdf}",
    "该型号 URDF 不可用": "URDF unavailable for this model",
    "请先从「机器人库」载入一个型号。": "Load a model from the robot library first.",
    "未加载机器人模型，无法确定目标品牌与后处理器。\n请先从「机器人库」载入一个型号。": "No robot model loaded; cannot determine target brand and post-processor.\nLoad a model from the robot library first.",
    "请先加载机器人": "Load a robot first",
    "载入失败": "Load failed",
    "保存失败": "Save failed",
    "关闭": "Close",
    "（无）": "(none)",
    "（无可调参数）": "(no adjustable parameters)",
    "（该设备无关节）": "(this device has no joints)",
    "⚠ 参数未核验 — ": "⚠ Parameters unverified — ",
    "载入预置工作站": "Load preset workcell",
    "单元模型不可用": "Cell model unavailable",
    "请先在「路径示教」里 Record 至少一个示教点": "Record at least one taught point in Path Teaching first",
    "\n  最近一次渲染覆盖层的问题:\n": "\n  Recent plugin overlay problems:\n",
    "\n\n可用后处理器：\n  ": "\n\nAvailable post-processors:\n  ",
    "机器人：{name}    目标语言：{lang}    指令数：{n}": "Robot: {name}    Target language: {lang}    Motions: {n}",
    "绕 X (°)": "X (°)",
    "绕 Y (°)": "Y (°)",
    "绕 Z (°)": "Z (°)",
    "、": ", ",
    "（{note}）": "({note})",
    # --- app.py：抓取工作站演示 tooltip（多段整组） ---
    "记录时赋予该示教点的工艺语义。\n抓取点：插入夹紧 IO，播放时把工件吸附到法兰；\n放置点：插入松开 IO，播放时把工件释放到世界。": "Process semantics assigned to this taught point.\nPick point: inserts grip IO; on playback the workpiece snaps to the flange;\nPlace point: inserts release IO; on playback the workpiece is left in the world.",
    "挂到「机器人」＝ 安装到法兰，工具将跟随机器人运动；\n挂到其它对象 ＝ 保持当前位置，仅建立随动关系。": "Attach to \"Robot\" = tool install: the object follows the robot flange;\nAttach to any other object = keep current position, follow relationship only.",
    "一键载入抓取工作站（UR5e + 吸盘 + 输送带 + 托盘），\n自动装上示例抓取程序并开始回放，状态栏显示节拍。": "One-click load of the grasping workstation (UR5e + gripper + conveyor + pallet),\nauto-loads the demo pick program and starts playback; the status bar shows cycle time.",
    "暂停 / 继续；暂停保留当前点位，便于逐点核对": "Pause / resume; pausing keeps the current point for step-by-step checks",
    "⏭ 单步": "⏭ Step",
    "⏸ 暂停": "⏸ Pause",
    "▶ 继续": "▶ Resume",
    "走一个示教点——核对抓取/放置时机最有用的操作": "Step one taught point — the most useful way to check grasp/release timing",
    "被抓取的场景对象；留空表示只演示轨迹": "Scene object to grasp; leave empty to demo trajectory only",
    "工件:": "Workpiece:",
    # --- app.py：右侧标签页 / 对象属性 ---
    "属性": "Properties",
    "场景设备库": "Scene Device Library",
    "设备树": "Device Tree",
    "导出打包": "Export & Package",
    "对象属性": "Object properties",
    "位置": "Position",
    "参数": "Parameters",
    "关节": "Joints",
    "信息": "Info",
    "日志": "Log",
    "删除": "Delete",
    "从场景同步": "Sync from scene",
    "自定义": "Custom",
    # --- 组件分类（与设备库分组同源，设备树「类型」列也用它）---
    "基础设施": "Infrastructure",
    "安全设施": "Safety facilities",
    "工件": "Workpiece",
    "夹具": "Fixture",
    "变位机": "Positioner",
    "输送设备": "Conveyor",
    "末端工具": "End effector",
    "机器人": "Robot",
    "范围 {lo} ~ {hi} {unit}": "Range {lo} ~ {hi} {unit}",
    "范围 {lo}° ~ {hi}°": "Range {lo}° ~ {hi}°",
    # --- app.py：代码生成 / PLC / 硬件清单 ---
    "PLC 程序逻辑": "PLC program logic",
    "代码预览": "Code preview",
    "生成": "Generate",
    "代码生成失败": "Code generation failed",
    "保存为文件…": "Save to file…",
    "保存路径代码": "Save path code",
    "保存 {lang} 代码": "Save {lang} code",
    "已保存 {lang} 代码: {name}": "Saved {lang} code: {name}",
    "生成的机器人代码 — {lang}": "Generated robot code — {lang}",
    "生成的 PLC 代码（西门子 SCL）将显示在这里": "Generated PLC code (Siemens SCL) will appear here",
    "生成机器人代码": "Generate robot code",
    "机器人子程序": "Robot subroutine",
    "机器人子程序代码（需先加载机器人并示教）": "Robot subroutine code (load a robot and teach points first)",
    "西门子 SCL": "Siemens SCL",
    "自动（按机器人）": "Auto (by robot)",
    "硬件列表": "Hardware list",
    "添加设备": "Add device",
    "传感器": "Sensor",
    "名称": "Name",
    "类型": "Type",
    "厂商": "Vendor",
    "型号": "Model",
    "通信": "Comm",
    "目标": "Target",
    "品牌": "Brand",
    "名称:": "Name:",
    "厂商:": "Vendor:",
    "型号:": "Model:",
    "控制器:": "Controller:",
    "类型:": "Type:",
    "点位:": "Points:",
    "挂载到:": "Attach to:",
    "轴数:": "Axes:",
    "重复定位精度:": "Repeatability:",
    "额定负载:": "Rated payload:",
    "最大臂展:": "Max reach:",
    "未加载模型": "No model loaded",
    "资源库": "Library",
    "工作单元干涉检查": "Cell interference check",
    "干涉检查": "Interference check",
    "程序校验": "Program validation",
    "程序校验与节拍估算": "Program validation & cycle-time estimate",
    "路径示教": "Path teaching",
    "协作臂": "Collaborative",
    "工业臂": "Industrial",
    "待机位姿": "Home pose",
    "关节值数量 {n} 与自由度 {dof} 不符（关节: {names}）": "Joint value count {n} does not match DOF {dof} (joints: {names})",
    # --- program_panel.py ---
    "程序概览": "Program overview",
    "估算节拍": "Estimate cycle time",
    "校验程序 (F7)": "Validate (F7)",
    "校验结果": "Validation results",
    "后处理与导出": "Post-processing & export",
    "后处理器": "Post-processor",
    "生成代码": "Generate code",
    "一键导出…": "One-click export…",
    "上传真机": "Upload to controller",
    "真机验证": "Controller verification",
    "回放": "Playback",
    "停止": "Stop",
    "从轨迹起点开始回放。": "Play back from the trajectory start.",
    "停止回放。": "Stop playback.",
    "看一遍路径怎么走": "Preview how the path moves",
    "轨迹回放中…": "Playing back trajectory…",
    "TCP 路径": "TCP path",
    "路径点": "Waypoints",
    "运动类型": "Motion type",
    "进给速度": "Feed speed",
    "直线插补": "Linear interpolation",
    "自动": "Auto",
    "自动（按机器人品牌）": "Auto (by robot brand)",
    "未加载机器人": "No robot loaded",
    "还没校验": "Not validated yet",
    "通过": "Pass",
    "不通过": "Failed",
    "结果": "Result",
    "错误": "Errors",
    "警示": "Warnings",
    "节拍 (s)": "Cycle (s)",
    "没程序（未示教）": "No program (nothing taught yet)",
    "看这台": "Show this one",
    "校验全部机器人": "Validate all robots",
    "还没有登记任何机器人——请先载入机器人型号。":
        "No robot registered yet — load a robot model first.",
    "当前版本不支持批量校验。":
        "This version does not support batch validation.",
    "每条指令都失败": "Every motion failed",
    "{n} 处错误": "{n} error(s)",
    "{w} 个 / {m} 条指令": "{w} points / {m} motions",
    "{name} · 就绪": "{name} · Ready",
    "校验完成：{res}": "Validation finished: {res}",
    "校验异常：{kind}: {msg}": "Validation error: {kind}: {msg}",
    "{name} 校验异常：{kind}: {msg}": "Validation error on {name}: {kind}: {msg}",
    "校验全部": "Validate all",
    "逐台校验所有机器人并汇总结果——不必一台一台切过去按 F7":
        "Validate every robot and summarize — no need to switch one by one and press F7",
    "校验全部：{res}": "Validate all: {res}",
    "{n} 台通过": "{n} passed",
    "{n} 台有问题": "{n} have problems",
    "{n} 台没程序": "{n} have no program",
    "没有可校验的机器人": "No robot to validate",
    "{name} 还没有校验结果": "{name} has no validation result yet",
    "这台": "This robot",
    "抓取向导": "Grasp wizard",
    "抓取向导…": "Grasp wizard…",
    "由抓取点/放置点自动生成接近、提升、撤离点——不必逐个手动示教":
        "Generate approach / lift / retract points from the pick and place poses — no need to teach them one by one",
    "当前版本不支持抓取向导。": "This version does not support the grasp wizard.",
    "请先记录至少一个示教点——把机器人摆到要抓的位置，点「记录」":
        "Record at least one taught point first — jog the robot to the grasp pose and press Record",
    "还没有载入机器人，无法计算抓取点位":
        "No robot loaded — cannot compute grasp points",
    "还没有示教点——请先把机器人摆到抓取位并记录一个点":
        "No taught point yet — jog to the grasp pose and record one",
    "抓取点下标 {i} 超出示教点范围（共 {n} 个）":
        "Pick index {i} is out of range ({n} taught points)",
    "放置点下标 {i} 超出示教点范围（共 {n} 个）":
        "Place index {i} is out of range ({n} taught points)",
    "接近点": "Approach",
    "抓取点": "Pick",
    "提升点": "Lift",
    "放置点": "Place",
    "撤离点": "Retract",
    "{label}（抬高 {dz:.0f} mm）逆解失败：{msg}":
        "Inverse kinematics failed for {label} (raised {dz:.0f} mm): {msg}",
    "抓取向导已生成 {n} 个点（{seq}）":
        "Grasp wizard generated {n} points ({seq})",
    "P{n}": "P{n}",
    "（不生成放置侧）": "(no place side)",
    "接近高度（抓取点上方）": "Approach height (above pick)",
    "提升高度（抓取点上方）": "Lift height (above pick)",
    "撤离高度（放置点上方）": "Retract height (above place)",
    "替换原有示教点（默认追加到末尾）": "Replace existing taught points (default: append)",
    "将生成 {n} 个点：{seq}": "Will generate {n} points: {seq}",
    "点位算不出来，示教点未改动：\n{msg}":
        "Could not compute the points; taught points are unchanged:\n{msg}",
    "取消": "Cancel",
    "没有示教点可供校验——请先记录若干关节位形": "No taught points to validate — record some joint poses first",
    "请先加载机器人并示教至少一个点": "Load a robot and teach at least one point first",
    "请先校验程序以生成轨迹，再回放": "Validate the program to generate a trajectory before playback",
    "请先载入机器人再添加路点": "Load a robot before adding waypoints",
    "请先载入机器人型号，再校验程序": "Load a robot model before validating",
    "请先在列表中选中要删除的路点": "Select a waypoint in the list to delete",
    "请先在列表中选中要改运动类型的路点": "Select a waypoint in the list to change its motion type",
    "请先在列表中选中要编辑的路点": "Select a waypoint in the list to edit",
    "插入": "Insert",
    "上移": "Move up",
    "下移": "Move down",
    "编辑位姿…": "Edit pose…",
    "编辑路点 {n}": "Edit waypoint {n}",
    "设为 moveJ": "Set as moveJ",
    "设为 moveL": "Set as moveL",
    "moveJ（关节插值）": "moveJ (joint interpolation)",
    "moveL（直线插值）": "moveL (linear interpolation)",
    "moveJ（关节）": "moveJ (joint)",
    "moveL（直线）": "moveL (linear)",
    "轨迹编辑": "Trajectory editing",
    "操作": "Actions",
    "状态": "Status",
    "选择导出目录": "Select export directory",
    "导入 {n} 个示教点": "Imported {n} taught points",
    "导出失败: {exc}": "Export failed: {exc}",
    "已导出：{handle}{note}": "Deployed: {handle}{note}",
    "已生成 {choice} 代码，共 {lines} 行": "Generated {choice} code, {lines} lines",
    "生成失败: {exc}": "Generation failed: {exc}",
    "生成失败：\n{exc}": "Generation failed:\n{exc}",
    "生成的控制器代码将显示在这里": "Generated controller code will appear here",
    "已插入路点 {n}，请重新校验": "Inserted waypoint {n}; re-validate",
    "已删除路点 {n}，请重新校验": "Deleted waypoint {n}; re-validate",
    "{n} 已更新，请重新校验": "{n} updated; re-validate",
    "{n} 已设为 {label}，请重新校验": "{n} set to {label}; re-validate",
    "修改选中路点的关节角（度）与运动类型（双击同效）": "Edit the selected waypoint's joint angles (deg) and motion type (double-click works too)",
    "在选中点之后插入一个路点（复制其关节值）": "Insert a waypoint after the selection (copies its joint values)",
    "一步构建程序并按品牌导出（UR .script / KUKA .src / ABB .mod / FANUC .tp）": "Build the program and export by brand in one step (UR .script / KUKA .src / ABB .mod / FANUC .tp)",
    "选中点改为关节运动（走平滑关节插值，不保证直线）": "Switch selection to joint motion (smooth joint interpolation; not a straight line)",
    "选中点改为直线运动（TCP 沿直线，走 IK 插补）": "Switch selection to linear motion (TCP follows a straight line via IK interpolation)",
    "该校验警告涉及此路点（见校验结果）": "A validation warning concerns this waypoint (see validation results)",
    "该校验错误涉及此路点（见校验结果）": "A validation error concerns this waypoint (see validation results)",
    "改动后需重新校验：着色与节拍会随校验结果刷新（红=错误 / 橙=警告 / 绿=正常）": "Re-validate after edits: coloring and cycle time refresh with validation results (red=error / orange=warning / green=ok)",
    "预留：接入 ControllerLink 后支持直连控制器上传": "Reserved: direct controller upload once ControllerLink is connected",
    "预留：接入 ControllerLink.read_joints 后做真机回读比对": "Reserved: read-back comparison once ControllerLink.read_joints is available",
    # --- world/panel.py（场景面板） ---
    "场景对象": "Scene objects",
    "对象": "Object",
    "场景为空": "Scene is empty",
    "0 个对象": "0 objects",
    "{n} 个对象": "{n} objects",
    "▢ 机器人": "▢ Robot",
    "保存场景": "Save scene",
    "保存场景…": "Save scene…",
    "加载场景": "Load scene",
    "加载场景…": "Load scene…",
    "加载失败": "Load failed",
    "场景文件 (*.json)": "Scene files (*.json)",
    "场景已保存: {name}": "Scene saved: {name}",
    "场景已加载: {name}（{n} 个对象）": "Scene loaded: {name} ({n} objects)",
    "场景已清空": "Scene cleared",
    "已删除对象": "Object deleted",
    "已添加 {name}": "Added {name}",
    "添加失败: {exc}": "Add failed: {exc}",
    "清空": "Clear",
    "清空场景": "Clear scene",
    "确定要移除所有场景对象吗？（机器人不受影响）": "Remove all scene objects? (robots are not affected)",
    "工艺时序（泳道图）": "Process timeline (swimlanes)",
    "时间 →": "Time →",
    "未检查": "Not checked",
    "检查干涉 (F6)": "Check interference (F6)",
    "✓ 无干涉": "✓ No interference",
    "✗ {n} 处干涉：{shown}{more}": "✗ {n} interference(s): {shown}{more}",
    "发现 {n} 处干涉": "Found {n} interference(s)",
    "干涉检查通过：无干涉": "Interference check passed: no interference",
    "检查失败: {exc}": "Check failed: {exc}",
    "机器人加载失败: {exc}": "Robot load failed: {exc}",
    " …等共 {n} 处": " …{n} in total",
    "拖入对象即可生成泳道，按时间轴编排工艺步骤，后续将驱动“工艺 → 机器动作”的自动映射。": "Drop objects here to create swimlanes and sequence process steps on a timeline; automatic \"process → machine action\" mapping is planned.",
    # --- robot_library/panel.py（机器人库面板） ---
    "全部": "All",
    "全部品牌": "All brands",
    "全部机型": "All models",
    "全部臂展": "All reaches",
    "全部负载": "All payloads",
    "刷新": "Refresh",
    "组件": "Component",
    "参数未核验": "Parameters unverified",
    "固定几何": "Fixed geometry",
    "规格未知": "Specs unknown",
    "载": "Load",
    "选": "Select",
    "搜索型号 / 品牌 / 组件…": "Search model / brand / component…",
    "拖拽到右侧视口加入场景 · 双击载入": "Drag into the viewport to add to the scene · double-click to load",
    "重新扫描 models/ 目录": "Rescan models/ directory",
    "重新扫描模型目录并重建列表。": "Rescan the model directory and rebuild the list.",
    "机器组件库：共 {n} 个型号 / 组件已并入": "Machine library: {n} models / components merged in",
    "{n} 项": "{n} items",
    "{name} — 双击载入": "{name} — double-click to load",
    "{name} 的 URDF 不可用": "URDF unavailable for {name}",
    "{n} 轴": "{n} axes",
    # --- 语言切换（app.py 菜单与对话框） ---
    "语言 / Language": "语言 / Language",
    "切换语言需要重启应用才能完全生效。现在重启吗？": "Restart the app to apply the language change fully. Restart now?",
    "立即重启": "Restart now",
    "稍后": "Later",
    # --- v1.1 库层消息：planner 校验 / 报告 ---
    "程序级": "Program-level",
    "指令 {n}": "Motion {n}",
    "轴 {axis} 低于下限位：{value:.3f} < {limit:.3f}": "Axis {axis} below lower limit: {value:.3f} < {limit:.3f}",
    "轴 {axis} 超出上限位：{value:.3f} > {limit:.3f}": "Axis {axis} above upper limit: {value:.3f} > {limit:.3f}",
    "✓ 程序校验通过": "✓ Program validation passed",
    "✗ 程序校验未通过（{n} 处错误）": "✗ Program validation failed ({n} error(s))",
    "  指令 {motions} 条 / 路径点 {waypoints} 个 / TCP 路径 {length:.1f} mm": "  {motions} motions / {waypoints} waypoints / TCP path {length:.1f} mm",
    "  估算节拍 {time:.2f} s": "  Estimated cycle time {time:.2f} s",
    "  驻留合计 {total:.2f} s（{n} 段 waitTime，已计入节拍）": "  Dwell total {total:.2f} s ({n} waitTime segment(s), included in cycle time)",
    "  节拍明细（按指令）：": "  Cycle breakdown (by motion):",
    "  力矩校验（峰值 / 额定）：": "  Torque check (peak / rated):",
    "  机械功率峰值 {power:.1f} W{avg}": "  Peak mechanical power {power:.1f} W{avg}",
    "（平均 {power:.1f} W）": " (avg {power:.1f} W)",
    "  警告 {n} 处：": "  {n} warning(s):",
    "  错误 {n} 处：": "  {n} error(s):",
    "圆弧的起点、中间点、终点（近）共线，无法确定圆弧；请检查 via 点是否落在起终点的连线上": "Arc start, via and end points are (nearly) collinear; cannot define an arc — check that the via point is not on the start-end line",
    "未知速度曲线风格 {profile}；可用：{available}": "Unknown velocity profile {profile}; available: {available}",
    "未知的指令类型：{type}": "Unknown command type: {type}",
    "轴 {axis} 峰值速度 {speed:.1f} 超出限位 {limit:.1f}（{ratio:.0f}%），该段路径弯折较急，建议放缓速度或增加过渡点": "Axis {axis} peak speed {speed:.1f} exceeds its limit {limit:.1f} ({ratio:.0f}%); this segment bends sharply — slow down or add transition points",
    "未传入工作单元场景，跳过干涉检查": "No workcell scene provided; collision check skipped",
    "干涉：{detail}": "Collision: {detail}",
    "force_mode_end() 没有对应的进入力控指令——真机上 end_force_mode/FCDeact 会直接报错": "force_mode_end() has no matching force-mode start — end_force_mode/FCDeact will fault on a real controller",
    "程序结束时仍处于力控状态（进入力控 N 次但未退出）——确认这是有意为之，否则请补 force_mode_end()": "Program ends while force mode is still active — confirm this is intentional, otherwise add force_mode_end()",
    "力矩校验跳过：机器人没有可用的运动学模型": "Torque check skipped: no kinematics model available for the robot",
    "力矩校验跳过：主链连杆缺少惯性参数（{links}）——可从 CAD 导出带 <inertial> 的 URDF，或用 DynamicsModel.set_link_inertial 手动录入": "Torque check skipped: main-chain links lack inertial parameters ({links}) — export a URDF with <inertial> from CAD, or set them via DynamicsModel.set_link_inertial",
    "关节 {joint} 峰值力矩 {torque:.1f} N·m 超出额定 {limit:.1f} N·m（{ratio:.0f}%）——该段加减速或姿态对关节负载过重，建议放缓速度、增加过渡点或减小负载": "Joint {joint} peak torque {torque:.1f} N·m exceeds its rating {limit:.1f} N·m ({ratio:.0f}%) — this segment overloads the joint; slow down, add transition points or reduce the payload",
    "关节 {joint} 峰值力矩 {torque:.1f} N·m 达到额定 {limit:.1f} N·m 的{ratio:.0f}%，余量偏小，建议留意": "Joint {joint} peak torque {torque:.1f} N·m reaches {ratio:.0f}% of its rating {limit:.1f} N·m — small margin, worth watching",
    "该段 {progress:.0f}% 处逆解失败——目标可能超出工作空间、位于奇异位形附近，或与相邻点之间要求过大的姿态变化（{exc}）": "IK failed at {progress:.0f}% of the segment — the target may be out of the workspace, near a singularity, or demand too much reorientation between adjacent points ({exc})",
    "moveJ 目标关节数 {n} 与机器人自由度 {dof} 不符": "moveJ target has {n} joints but the robot has {dof} DOF",
    "moveJ 目标{violation}": "moveJ target {violation}",
    "moveSpline 目标{violation}": "moveSpline target {violation}",
    "target 既不是关节列表也不是位姿：{exc}": "target is neither a joint list nor a pose: {exc}",
    "目标关节数 {n} 与自由度 {dof} 不符": "target has {n} joints but the robot has {dof} DOF",
    "moveC 缺少 via（中间点）": "moveC is missing a via (intermediate) point",
    "该段 {progress:.0f}% 处{violation}——直线/圆弧插补会把关节推出限位，需调整路径或工件位置": "{violation} at {progress:.0f}% of the segment — linear/circular interpolation would push joints out of limits; adjust the path or workpiece position",
    "该段 {progress:.0f}% 处轴 {axis} 跳变 {step:.1f}°（> {limit}°），可能穿越奇异位形发生翻腕": "Axis {axis} jumps {step:.1f}° at {progress:.0f}% of the segment (> {limit}°) — possible wrist flip through a singularity",
    "接近奇异位形（雅可比条件数 {cond:.2e}），关节速度可能突变，建议调整姿态或路径": "Near a singular configuration (Jacobian condition number {cond:.2e}); joint speeds may jump — adjust the pose or path",
    # --- v1.1 库层消息：后处理告警 / 生成代码注释 ---
    "[warning] {name} 尚未适配 IO 指令，将输出通用伪语法；下装前必须人工核对": "[warning] {name} does not adapt IO commands yet; generic pseudo-syntax will be emitted — verify manually before download",
    "{name}: 第 {index} 条指令转换失败: {exc}": "{name}: failed to convert command {index}: {exc}",
    "{name}: 模板渲染失败: {exc}": "{name}: template rendering failed: {exc}",
    "{name} 尚未实现原生 SPLINE 指令；默认已将其降级为 PTP/LIN（目的地一致）": "{name} has no native SPLINE support; it has been downgraded to PTP/LIN (same destination)",
    "[warning] {name}: 信号 {signal} 未映射到物理地址，已输出 TODO 占位（下装前必须补全）": "[warning] {name}: signal {signal} is not mapped to a physical address; a TODO placeholder was emitted (must be resolved before download)",
    "TODO: 原语义为 {trigger} 边沿触发，需条件监控实现，当前按电平输出，请人工核对": "TODO: original semantics is {trigger}-edge triggered, which requires a condition monitor; currently emitted as a level check — verify manually",
    "起弧前保护气预流 {seconds:.1f}s": "Gas pre-flow {seconds:.1f} s before arc start",
    "熄弧后保护气滞流 {seconds:.1f}s": "Gas post-flow {seconds:.1f} s after arc end",
    "TODO: 该品牌未适配原生力控指令（指令名随选件/版本而变），请按上述参数对照品牌手册人工实现": "TODO: this brand has no adapted native force-control command (the name varies by option/version); implement manually per the brand manual using the parameters above",
    "{name}: 进入力控的 ForceMode 指令缺少 ComplianceSpec": "{name}: the ForceMode command entering force control lacks a ComplianceSpec",
    "{name}: 不支持的指令类型: {type}": "{name}: unsupported command type: {type}",
    "无法识别的目标表示，长度 {size}（期望 3/6/16）": "Unrecognized target representation, size {size} (expected 3/6/16)",
    "不支持的欧拉角顺序: {order}": "Unsupported Euler order: {order}",
    "不支持的线位移单位: {unit}": "Unsupported linear unit: {unit}",
    "不支持的角度单位: {unit}": "Unsupported angle unit: {unit}",
    # --- v1.1 库层消息：post 工厂 / 品牌后处理器 ---
    "插件后处理器 {name} 未返回 BasePostProcessor 子类实例，而是 {cls}": "Plugin post-processor {name} did not return a BasePostProcessor instance but {cls}",
    "品牌 {name} 有多个后处理器 {matches}，请指定完整名称": "Brand {name} has multiple post-processors {matches}; specify the full name",
    "未知的后处理器 {name}；可用：{available}": "Unknown post-processor {name}; available: {available}",
    "! TODO: 信号 {signal} 地址 {addr} 无法解析为数字量序号，需人工改为 DO[n]={level}": "! TODO: signal {signal} address {addr} cannot be resolved to a numeric index; change manually to DO[n]={level}",
    "! TODO: 信号 {signal} 地址 {addr} 无法解析为数字量序号，需人工改为 WAIT DI[n]": "! TODO: signal {signal} address {addr} cannot be resolved to a numeric index; change manually to WAIT DI[n]",
    "TODO: 原语义为 {trigger} 边沿触发，请人工核对": "TODO: original semantics is {trigger}-edge triggered — verify manually",
    "TODO: 超时 {seconds:.2f}s 需用 SKIP 条件实现": "TODO: timeout {seconds:.2f} s requires the SKIP condition — implement manually",
    "送丝 {wire:.1f}m/min": "Wire feed {wire:.1f} m/min",
    "起弧前预流 {seconds:.1f}s": "gas pre-flow {seconds:.1f} s before arc start",
    "FANUC TP 不支持的运动类型: {type}": "FANUC TP does not support motion type: {type}",
    "FANUC TP 圆弧运动需要提供 via 中间点": "FANUC TP circular motion requires a via point",
    "; TODO: 信号 {signal} 地址 {addr} 无法解析为数字量序号，需人工改为 $OUT[n] = {level}": "; TODO: signal {signal} address {addr} cannot be resolved to a numeric index; change manually to $OUT[n] = {level}",
    "; TODO: 信号 {signal} 地址 {addr} 无法解析为数字量序号，需人工改为 WAIT FOR $IN[n]": "; TODO: signal {signal} address {addr} cannot be resolved to a numeric index; change manually to WAIT FOR $IN[n]",
    "TODO: 超时 {seconds:.2f}s 需用 $TIMER 自行实现": "TODO: timeout {seconds:.2f} s requires $TIMER — implement manually",
    "KRL 不支持的运动类型: {type}": "KRL does not support motion type: {type}",
    "KRL 圆弧运动需要提供 via 中间点": "KRL circular motion requires a via point",
    "# TODO: 信号 {signal} 地址 {addr} 无法解析为数字量序号，需人工改为 set_digital_out(n, {level})": "# TODO: signal {signal} address {addr} cannot be resolved to a numeric index; change manually to set_digital_out(n, {level})",
    "# TODO: 信号 {signal} 地址 {addr} 无法解析为数字量序号，需人工改为 wait_digital_in(n)": "# TODO: signal {signal} address {addr} cannot be resolved to a numeric index; change manually to wait_digital_in(n)",
    "TODO: 超时 {seconds:.2f}s 需自行实现看门狗": "TODO: timeout {seconds:.2f} s requires a watchdog — implement manually",
    "TODO: 原语义为 {trigger} 边沿触发，需轮询实现，当前按电平输出，请人工核对": "TODO: original semantics is {trigger}-edge triggered, which requires polling; currently emitted as a level check — verify manually",
    "URScript 不支持的运动类型: {type}": "URScript does not support motion type: {type}",
    "URScript 圆弧运动需要提供 via 中间点": "URScript circular motion requires a via point",
    "TODO: 原语义为 {trigger} 边沿触发，需 WaitUntil 边沿逻辑实现，当前按电平输出，请人工核对": "TODO: original semantics is {trigger}-edge triggered, which requires WaitUntil edge logic; currently emitted as a level check — verify manually",
    "TODO: ABB Force Control 为 RobotWare 选件，FCRefForce/FCAct 的实参语法请对照《Application manual - Force Control》核对": "TODO: ABB Force Control is a RobotWare option; verify the FCRefForce/FCAct argument syntax against the Application manual - Force Control",
    "RAPID 不支持的运动类型: {type}": "RAPID does not support motion type: {type}",
    "RAPID 圆弧运动需要提供 via 中间点": "RAPID circular motion requires a via point",
    "ESTUN 不支持的运动类型: {type}": "ESTUN does not support motion type: {type}",
    "ESTUN 圆弧运动需要提供 via 中间点": "ESTUN circular motion requires a via point",
    "JAKA 不支持的运动类型: {type}": "JAKA does not support motion type: {type}",
    "JAKA 圆弧运动需要提供 via 中间点": "JAKA circular motion requires a via point",
    "InoTeach 不支持的运动类型: {type}": "InoTeach does not support motion type: {type}",
    "InoTeach 圆弧运动需要提供 via 中间点": "InoTeach circular motion requires a via point",
    "# NOTE: 语法未经真实控制器验证，使用前请核对": "# NOTE: syntax not verified on a real controller; check before use",
    # ---- 真机连接面板（v1.1.1）----
    "真机连接": "Real Controller",
    "真机连接…": "Real Controller…",
    "连接": "Connect",
    "断开": "Disconnect",
    "主机": "Host",
    "端口": "Port",
    "超时": "Timeout",
    "未连接": "Not connected",
    "已连接": "Connected",
    "程序部署": "Program Deployment",
    "部署当前程序": "Deploy Current Program",
    "刷新列表": "Refresh List",
    "回读": "Feedback",
    "回读关节": "Read Joints",
    "关节角 —": "Joints —",
    "未知": "unknown",
    "正在连接 {host}:{port} …": "Connecting to {host}:{port} …",
    "连接失败：{exc}": "Connection failed: {exc}",
    "服务端 {server} · 协议 v{api} · 关节数 {dof} · 能力 {caps}": "Server {server} · protocol v{api} · dof {dof} · caps {caps}",
    "已连接：{desc}": "Connected: {desc}",
    "已连接 {name}": "Connected to {name}",
    "已断开 {name}": "Disconnected from {name}",
    "未连接控制器——请先建立连接": "Controller not connected — connect first",
    "未设置程序来源——请从离线编程面板打开本窗口": "No program source set — open this panel from the main window",
    "没有可部署的程序——请先加载机器人并示教至少一个点": "Nothing to deploy — load a robot and teach at least one point first",
    "部署失败：{exc}": "Deployment failed: {exc}",
    "已部署 {name}：{handle}{note}": "Deployed {name}: {handle}{note}",
    "已部署 {name}：{handle}": "Deployed {name}: {handle}",
    "获取程序列表失败：{exc}": "Failed to list controller programs: {exc}",
    "控制器程序（{n}）": "Controller programs ({n})",
    "控制器上还没有程序": "No programs on the controller yet",
    "回读失败：{exc}": "Read failed: {exc}",
    "控制器不支持关节回读（能力协商 feedback=false）——不伪造数据": "Controller does not support joint feedback (negotiated caps.feedback=false) — no fabricated data",
    "控制器不支持回读": "Controller does not support feedback",
    "关节角（度）：{vals}": "Joints (deg): {vals}",
    "回读关节（度）：{vals}": "Read joints (deg): {vals}",
    "回读关节应用失败：{exc}": "Failed to apply read joints: {exc}",
    "关节数不符（真机 {n}，模型 {m}）": "Joint count mismatch (controller {n}, model {m})",
    "已按真机位形摆好视口": "Viewport set to the controller's actual pose",
    "真机连接模块不可用。": "Real controller module unavailable.",
    "连接真机控制器：部署当前程序、回读实际关节角、取回核对。\n传输可选 Socket（orolp-socket 行协议）或 SSH（SFTP 传程序 + exec）。\n无运动指令入口、无自由命令输入框——程序运动一律走部署。": (
        "Connect to a real controller: deploy the current program, read back actual joints,\n"
        "fetch and verify. Transport: Socket (orolp-socket line protocol) or SSH\n"
        "(SFTP upload + exec). No motion-command entry and no free-form command box —\n"
        "motion goes through deployment only."
    ),

    # ---- 真机连接面板：SSH 传输与取回核对（v1.1.4）----
    "传输": "Transport",
    "Socket（orolp-socket 行协议）": "Socket (orolp-socket line protocol)",
    "SSH（SFTP 传程序 + exec）": "SSH (SFTP upload + exec)",
    "用户名": "Username",
    "密码": "Password",
    "与私钥二选一": "or use a private key",
    "私钥": "Private key",
    "私钥文件路径，可留空": "Private key file path (optional)",
    "远端目录": "Remote directory",
    "取回核对": "Fetch & Verify",
    "把控制器上的程序原文取回，与本次生成的代码逐字节比对——\n「部署报成功」与「控制器上确实是这份代码」是两件事": (
        "Fetch the program text back from the controller and compare it byte for byte with the\n"
        "code just generated — \"deploy reported success\" and \"the controller really holds\n"
        "this code\" are two different things"
    ),
    "正在连接 {transport} {host}:{port} …": "Connecting to {transport} {host}:{port} …",
    "SSH 形态无回读通道": "SSH transport has no feedback channel",
    "SSH 形态没有关节回读通道——回读请用 Socket；不伪造数据": (
        "SSH transport has no joint feedback channel — use Socket for feedback; no fabricated data"
    ),
    "SSH 无能力协商 · 远端目录 {dir} · 不支持关节回读": (
        "SSH: no capability negotiation · remote dir {dir} · joint feedback unsupported"
    ),
    "没有可核对的程序——请先部署一次": "Nothing to verify — deploy once first",
    "取回失败：{exc}": "Fetch failed: {exc}",
    "控制器上没有 {name}——部署可能没落盘": (
        "{name} not found on the controller — the deployment may not have landed"
    ),
    "控制器上没有 {name}": "{name} not found on the controller",
    "核对 {name}：与本次生成的代码逐字节一致（{lines} 行）": (
        "Verified {name}: byte-identical to the code just generated ({lines} lines)"
    ),
    "核对 {name}：**与本次生成的代码不一致**（控制器 {lines} 行 / 本地 {local} 行）——先别下线": (
        "Verified {name}: **DIFFERS from the code just generated** "
        "(controller {lines} lines / local {local} lines) — do not go offline yet"
    ),
    "核对 {name}：已取回 {lines} 行（本次会话不是由本面板部署的它，只报了行数，不冒充逐字节结论）": (
        "Fetched {name}: {lines} lines (not deployed by this panel in this session — "
        "line count only, no byte-level claim)"
    ),
    "核对 {name}：{lines} 行": "Verified {name}: {lines} lines",
    "关节角 —": "Joints —",

    # ---- link.describe()（真机连接状态行，随界面语言）----
    "服务端": "server",
    "能力": "caps",
    "请求": "requests",
    "目录": "dir",
    "已传": "uploaded",
    "执行": "exec",

    # ---- Program.deploy 备注（按连接形态如实描述回读能力）----
    "支持实际位置回读": "actual position feedback supported",
    "离线导出，无实际位置回读": "offline export, no actual position feedback",
    "{protocol} 连接，控制器不提供实际位置回读": (
        "{protocol} connection; the controller provides no actual position feedback"
    ),

    # ---- 轨迹编辑器圆弧编辑（v1.1.5）----
    # 口径：第 N 行的运动类型描述"到达第 N 点"的运动，moveC 以上一点为中间点
    "设为 moveC": "Set as moveC",
    "moveC（圆弧）": "moveC (circular)",
    "moveC（圆弧，以上一点为中间点）": "moveC (circular; previous point is the via)",
    "moveC 中间点": "moveC via",
    "可设为 moveC（圆弧）": "Can be set to moveC (circular)",
    "不可设为 moveC：{reason}": "Cannot be set to moveC: {reason}",
    "路点不存在": "No such waypoint",
    "P{n} 缺少两个前驱点——圆弧需要起点与中间点，P1/P2 不能设为 moveC": (
        "P{n} has fewer than two predecessors — an arc needs a start and a via, "
        "so P1/P2 cannot be moveC"
    ),
    "P{n} 已是圆弧终点，不能再作为本段的中间点": (
        "P{n} is already an arc endpoint and cannot serve as this segment's via"
    ),
    "P{n} 已把 P{k} 当中间点——本行不能同时作为圆弧终点": (
        "P{n} already uses P{k} as its via — this row cannot also be an arc endpoint"
    ),
    "P{n} 是抓取/放置点，不能作为圆弧中间点（中间点不停机）": (
        "P{n} is a pick/place point and cannot be an arc via (a via does not stop)"
    ),
    "{reason}——已按关节运动（moveJ）处理": "{reason} — handled as a joint move (moveJ)",
    "P{n} 的圆弧正解失败——退化为关节运动（moveJ）；中间点 P{k} 改为单独经过": (
        "Forward kinematics failed for the arc at P{n} — degraded to joint moves (moveJ); "
        "via point P{k} is now passed through separately"
    ),
    "P{n} 的圆弧无法加入程序——退化为关节运动（moveJ）；中间点 P{k} 改为单独经过": (
        "The arc at P{n} could not be added to the program — degraded to joint moves (moveJ); "
        "via point P{k} is now passed through separately"
    ),
    "；{n} 项运动降级（见日志）": "; {n} motion(s) degraded (see log)",
    "⚠ {note}": "⚠ {note}",
    "✓ 程序校验通过，未发现问题": "✓ Program validated, no issues found",
    "选中点改为圆弧运动：以上一点为中间点、再上一点为起点画圆弧到达本点。\n"
    "中间点不再单独出指令（它只是一段圆弧的途经点），因此 P1/P2 不能设为圆弧，\n"
    "相邻两行也不能同为圆弧，抓取/放置点不能当中间点。": (
        "Turn the selected point into a circular motion: the previous point is the via and "
        "the one before it the start, then arc into this point.\n"
        "The via no longer emits its own instruction (it is merely passed through), so P1/P2 "
        "cannot be arcs, two adjacent rows cannot both be arcs, and pick/place points cannot "
        "be vias."
    ),
    "改动后需重新校验：着色与节拍会随校验结果刷新（红=错误 / 橙=警告 / 绿=正常）\n"
    "圆弧口径：第 N 行的运动类型描述「到达第 N 点」的运动——moveC 以上一点为中间点、"
    "再上一点为起点；中间点行显示为「moveC 中间点」": (
        "Re-validate after edits: colours and cycle times refresh with the validation result "
        "(red = error / orange = warning / green = OK)\n"
        "Arc convention: a row's motion type describes the motion *arriving at* that point — "
        "moveC uses the previous point as via and the one before it as start; via rows are "
        "labelled \"moveC via\""
    ),

    # ---- 补漏：v1.0.10 GUI 接线时遗漏的条目（en 模式不再残留中文）----
    "保持": "Hold",
    "夹紧": "Close",
    "松开": "Open",
    "(场景根)": "(Scene root)",
    "场景根": "Scene root",
    "拖拽节点改写挂载层级：挂到机器人＝安装到法兰，自动对齐坐标系": (
        "Drag a node to re-parent it: dropping onto a robot mounts it on the "
        "flange and auto-aligns the coordinate frame"
    ),
    "{name} 是机器人，不能被挂载": "{name} is a robot and cannot be mounted",
    "{name} 是机器人，不能挂到 {host} 的法兰上": (
        "{name} is a robot and cannot be mounted on {host}'s flange"
    ),
    "挂载失败: {exc}": "Mounting failed: {exc}",
    "已将 {name} 挂到 {host}": "Mounted {name} onto {host}",
    "已将 {name} 装夹到 {host} 末端": "Mounted {name} onto {host}'s flange",
    "装夹到末端": "Mount on flange",
    "解除装夹": "Unmount",
    "吸附到机器人法兰原点，工具坐标系与法兰对齐并随机器人运动": (
        "Snap to the robot flange origin: the tool frame aligns with the flange "
        "and follows the robot"
    ),
    "解除挂载并保持当前世界位姿": "Unmount and keep the current world pose",
    "按场景里的机器人/组件补齐设备清单（已存在的保留不动）": (
        "Fill the device list from the scene's robots/components "
        "(existing entries are kept as-is)"
    ),
    "环境建模模块不可用。": "The environment modelling module is unavailable.",
    "离线编程模块不可用。": "The offline programming module is unavailable.",

    # ---- PLC ↔ 机器人 IO 点表（v1.1.6）----
    # 方向标签是词表数据（DIRECTION_LABELS），只在显示处经 tr() 翻译
    "PLC→机器人": "PLC→robot",
    "机器人→PLC": "robot→PLC",
    "传感器→PLC": "sensor→PLC",
    # 地址解析
    "地址为空": "Address is empty",
    "地址里有空白字符": "Address contains whitespace",
    "地址里有逗号——一行只能写一个地址": "Address contains a comma — one address per row",
    "地址里没有任何数字": "Address contains no digits",
    "点表需要工作单元配置（CellConfig）": (
        "The point table requires a cell configuration (CellConfig)"
    ),
    # 点表行
    "机器人读写": "robot reads and writes",
    "机器人写": "robot writes",
    "机器人读": "robot reads",
    "机器人未用": "not used by the robot",
    "未声明": "undeclared",
    "信号": "Signal",
    "地址": "Address",
    "方向": "Direction",
    "用途": "Usage",
    "归属": "Owner",
    "点表 {total} 点：已映射 {mapped} / 未映射 {unmapped}，{errors} 处错误 {warnings} 处警告": (
        "Point table: {total} point(s), {mapped} mapped / {unmapped} unmapped, "
        "{errors} error(s), {warnings} warning(s)"
    ),
    "  ✓ 点表未发现问题": "  ✓ No point-table issues found",
    # 校验
    "机器人程序在用 {usage}，但没有任何设备给出物理地址——下装前必须补上": (
        "The robot program uses it ({usage}) but no device provides a physical address — "
        "must be filled in before download"
    ),
    "点表里有它但没有地址（下装前补上，或从点表里删掉）": (
        "Listed in the point table but has no address "
        "(fill it in before download, or remove it from the table)"
    ),
    "{addr} 已被 {other} 占用——两个信号不能共用一个物理地址": (
        "{addr} is already taken by {other} — two signals must not share one physical address"
    ),
    "{devices} 都声明了这个信号——归属必须唯一": (
        "{devices} all declare this signal — the owner must be unique"
    ),
    "{addr!r} 不像一个 IO 地址：{why}": "{addr!r} does not look like an IO address: {why}",
    "机器人（SetDO）与 PLC 都在驱动它——必须明确主控方，否则谁后写谁生效": (
        "Both the robot (SetDO) and the PLC drive it — pick one owner, "
        "otherwise the last writer wins"
    ),
    "机器人在等它，但机器人自己不写、PLC 也没驱动它——现场会卡在这一步": (
        "The robot waits on it, but the robot does not write it and the PLC does not drive it — "
        "the cell will stall here"
    ),
    # GUI（IO 映射对话框 + 硬件列表按钮 + 状态栏）
    "IO 映射": "IO map",
    "IO 映射…": "IO map…",
    "IO 映射 · {name}": "IO map · {name}",
    "IO 校验": "Check IO",
    "IO 点表校验": "IO point table check",
    "编辑当前设备的信号地址（点表与 PLC 出码共用这一份数据）": (
        "Edit the selected device's signal addresses "
        "(the point table and the PLC code share this one source)"
    ),
    "检查未映射 / 地址重复 / 两侧同时驱动 / 机器人等一个没人给的信号": (
        "Check for unmapped signals / duplicate addresses / both sides driving / "
        "the robot waiting on a signal nobody gives"
    ),
    "请先在设备列表里选中一行": "Select a row in the device list first",
    "地址（可编辑）": "Address (editable)",
    "添加信号": "Add signal",
    "信号名（如 gripper_close）": "Signal name (e.g. gripper_close)",
    "自动编号": "Auto-number",
    "按方向自动编号，跳过已被占用的地址": (
        "Number by direction, skipping addresses that are already taken"
    ),
    "自动编号：{n} 个信号（跳过已占用地址）": (
        "Auto-numbered {n} signal(s), skipping taken addresses"
    ),
    "IO 映射已更新：{name}（{n} 个信号）": "IO map updated: {name} ({n} signal(s))",
    "有问题的点必须在下载前处理（双击「Details」看逐点清单）": (
        "Problem points must be fixed before download (double-click \"Details\" for the list)"
    ),
    "警告不阻塞下载，但现场必须有人看过": (
        "Warnings do not block download, but someone on site must have looked at them"
    ),
    "PLC 代码已生成，但点表有 {n} 处错误——下装前必须处理": (
        "PLC code generated, but the point table has {n} error(s) — fix before download"
    ),

    # ---- 场景拖拽摆放（v1.1.7）----
    "地面": "ground",
    "{name} → ({x:.1f}, {y:.1f}, {z:.1f}) mm · 落在 {support} 上": (
        "{name} → ({x:.1f}, {y:.1f}, {z:.1f}) mm · placed on {support}"
    ),
    "{name} 位置未变": "{name} not moved",
    "场景里没有 {name}": "{name} is not in the scene",
    "{name} 是机器人——拖拽摆放只用于工件与设备": (
        "{name} is a robot — drag-and-place is for parts and equipment only"
    ),
    "{name} 不能拖动：对象已锁定": "{name} cannot be dragged: the object is locked",
    "{name} 不能拖动：已挂载在 {host} 上，请先解除挂载": (
        "{name} cannot be dragged: mounted on {host}; unmount it first"
    ),

    # ---- 规模与性能基准（v1.1.8）----
    "RobotWorld 规划/校验规模基准": "RobotWorld planning / validation scale benchmark",
    "机器人 URDF 路径": "robot URDF path",
    "单次基准的运动条数": "number of motions in the single-run benchmark",
    "规模表用的运动条数（逗号分隔，如 20,50,100）": (
        "motion counts for the scale table (comma separated, e.g. 20,50,100)"
    ),
    "每段重复次数（取最优）": "repetitions per phase (best-of)",
    "输出 JSON": "emit JSON",
    "找不到机器人模型；用 --robot 指定 URDF 路径": (
        "robot model not found; pass a URDF path with --robot"
    ),
    "规模与性能基准 · {robot}": "Scale and performance benchmark · {robot}",
    "  指令 {commands} 条（其中运动 {motions} 条）→ 路点 {wps} 个": (
        "  {commands} command(s) ({motions} motion(s)) → {wps} waypoint(s)"
    ),
    "  仿真节拍 {cycle:.2f} s · 路径长 {length:.1f} mm": (
        "  simulated cycle time {cycle:.2f} s · TCP travel {length:.1f} mm"
    ),
    "  校验问题：{errors} 错误 / {warnings} 警告": (
        "  validation: {errors} error(s) / {warnings} warning(s)"
    ),
    "  阶段耗时（{repeats} 次取最优）": "  phase timings (best of {repeats})",
    "  单路点规划 {ms:.3f} ms · 整轮墙钟 {wall:.2f} s": (
        "  {ms:.3f} ms per waypoint · {wall:.2f} s wall clock for the whole run"
    ),
    "  （未接场景，干涉检查跳过）": "  (no scene attached — interference check skipped)",
    "  关节原点旋转：实算 {computed} 次 / 命中缓存 {hits} 次": (
        "  Joint-origin rotations: {computed} computed / {hits} cache hit(s)"
    ),
    "规划": "planning",
    "校验（含内部重新规划）": "validation (re-plans internally)",
    "{per:.3f} ms/路点": "{per:.3f} ms/waypoint",
    "{n} 处接触": "{n} contact(s)",
    "{n} 条问题": "{n} issue(s)",
    "未接场景，跳过": "no scene, skipped",
    "运动条数  路点数  节拍(s)  规划(ms)  校验(ms)  干涉(ms)  ms/路点": (
        "motions  waypoints  cycle(s)  plan(ms)  validate(ms)  collision(ms)  ms/wp"
    ),
    "规模表 · {robot}（{repeats} 次取最优）": "Scale table · {robot} (best of {repeats})",

    # ---- 干涉检查分段基准（v1.1.9）----
    "附加干涉检查分段基准（接仓库内置抓取工作站场景）": (
        "also benchmark interference checking (with the built-in grasping workstation)"
    ),
    "干涉基准的关节采样步长（度，逗号分隔）": (
        "joint-space sampling steps for the interference benchmark (degrees, comma separated)"
    ),
    "{n} 处接触 · 采样 {k} 次（每 {m} 路点 1 次）": (
        "{n} contact(s) · {k} sample(s) (1 per {m} waypoints)"
    ),
    "干涉检查分段基准 · {robot}": "Interference-check breakdown · {robot}",
    "  指令 {commands} 条 → 路点 {wps} 个 · 干涉裕度 {margin:.1f} mm": (
        "  {commands} command(s) → {wps} waypoint(s) · interference margin {margin:.1f} mm"
    ),
    "  机器人零件 {parts} 个 · 环境对象 {objs} 个（零件 {eparts} 个）": (
        "  robot parts {parts} · environment objects {objs} (parts {eparts})"
    ),
    "采样口径": "sampling policy",
    "采样数": "samples",
    "每采样路点": "waypoints/sample",
    "总耗时": "total",
    "每次采样": "per sample",
    "接触": "contacts",
    "窄相对": "narrow pairs",
    "广相剔除": "broad-phase rejects",
    "默认": "default",
    "默认（约 {n} 次）": "default (about {n} samples)",
    "关节步长 {deg:g}°": "joint step {deg:g}°",
    "  口径说明：「{default}」按总采样数约 300 的预算取样，程序越长采样点隔得越远；"
    "给定关节步长的口径按「机器人可能移动多远」取样，不随程序变长而稀释。": (
        "  Sampling policy: the 「{default}」 policy spends a budget of ~300 samples, "
        "so the gap between two samples grows with program length; a joint-step policy "
        "samples by how far the robot can actually move and does not thin out as the "
        "program grows."
    ),
    "max_joint_step 必须是非负有限值，收到 {value}——它是采样判据（度），负值没有意义": (
        "max_joint_step must be a finite non-negative number, got {value} — it is a "
        "sampling criterion in degrees, a negative value is meaningless"
    ),

    # ---- 几何内核等价性对拍（v1.1.9）----
    "几何内核等价性对拍（生产实现 vs 朴素参考实现）": (
        "Collision kernel equivalence check (production vs naive reference)"
    ),
    "  {pairs} 对基本体 × {cases} 组位姿/裕度 · 裕度 {margins}": (
        "  {pairs} primitive pair(s) × {cases} pose/margin case(s) · margins {margins}"
    ),
    "  碰撞判定差异 {n} 处 · 穿透深度完全相同 {exact}/{pairs} 次 · 最大数值差 {delta:.3e}": (
        "  collision verdict differences: {n} · penetration depth identical in "
        "{exact}/{pairs} case(s) · worst numeric delta {delta:.3e}"
    ),
    "  AABB {a} · 顶点 {v} · SAT 轴 {s} 差异（均应为 0）": (
        "  AABB {a} · vertices {v} · SAT axes {s} difference(s) (all should be 0)"
    ),
    "  同一批判定：生产 {fast:.3f} s · 朴素 {slow:.3f} s → {x:.1f}× 更快": (
        "  Same batch of verdicts: production {fast:.3f} s · naive {slow:.3f} s "
        "→ {x:.1f}× faster"
    ),

    # ---- 增量重算分场景基准（v1.2.0）----
    "增量重算分场景基准 · {robot}": "Incremental replan per-scenario benchmark · {robot}",
    "  指令 {commands} 条（运动 {motions} 条）→ 路点 {wps} 个 · 冷启动 {base:.1f} ms": (
        "  {commands} command(s) ({motions} motion(s)) → {wps} waypoint(s) · "
        "cold start {base:.1f} ms"
    ),
    "场景": "Scenario",
    "耗时": "Time",
    "加速": "Speedup",
    "分段复用": "Seg reuse",
    "整程序": "Whole",
    "等价": "Equiv",
    "命中": "hit",
    "差 {d:.1e}": "diff {d:.1e}",
    "  分段复用率的分母是「指令数」而不是「路点数」——一条指令命中，意味着它那几十个路点全部免算": (
        "  The segment reuse rate is over commands, not waypoints — one command hit "
        "spares all of its dozens of waypoints"
    ),
    "  等价列＝与关缓存的实算结果逐位比对（关节/位姿/时刻/速度），任何一项不为 0 都会在这里显示出来": (
        "  The Equiv column compares bit-for-bit against a cache-disabled run "
        "(joints/pose/time/velocity); any non-zero difference shows up here"
    ),
    "无改动": "No edit",
    "改末尾运动": "Edit last motion",
    "改中段运动": "Edit middle motion",
    "改首条运动": "Edit first motion",
    "只改速度": "Speed only",
    "插入驻留": "Insert dwell",
    "删除一条": "Delete one",
    "加跑增量重算分场景基准（moveJ 程序）": (
        "Also run the incremental replan benchmark (moveJ program)"
    ),
    "增量基准再加跑 moveL（位姿目标）程序——两者复用率差别是结构性的": (
        "Also benchmark a moveL (pose-target) program — the reuse gap between the "
        "two is structural"
    ),
    # --- 工艺时序（v1.3.1）：序列模型 / 校验 / 双出口 / 设备树编辑器 ---
    "添加工步": "Add step",
    "校验时序": "Validate sequence",
    "保存时序…": "Save sequence…",
    "载入时序…": "Load sequence…",
    "保存工艺时序": "Save process sequence",
    "载入工艺时序": "Load process sequence",
    "未校验": "Not validated",
    "还没有工步——点「添加工步」把工序排进泳道": (
        'No steps yet — click "Add step" to lay operations into the swimlanes'
    ),
    "步骤序号 →（驻留按秒加宽；非实测节拍）": (
        "Step # → (dwell widened by seconds; not a measured cycle)"
    ),
    "序号": "#",
    "泳道": "Lane",
    "内容": "Content",
    "泳道（设备）:": "Lane (device):",
    "工步名:": "Step name:",
    "子程序名:": "Subprogram name:",
    "信号:": "Signal:",
    "动作:": "Action:",
    "置位（勾选＝True，不勾＝False）": "Set (checked = True, unchecked = False)",
    "触发方式:": "Trigger:",
    "电平": "Level",
    "上升沿": "rising edge",
    "下降沿": "falling edge",
    "期望电平为 True": "Expected level is True",
    "期望电平:": "Expected level:",
    "超时 (s):": "Timeout (s):",
    "0＝无限等待。现场必须配看门狗，否则一步卡死整条线": (
        "0 = wait forever. A watchdog is mandatory on site, otherwise one step "
        "stalls the whole line"
    ),
    "驻留 (s):": "Dwell (s):",
    "添加工步：子程序调用": "Add step: subprogram call",
    "添加工步：触发信号": "Add step: set signal",
    "添加工步：等待信号": "Add step: wait for signal",
    "添加工步：驻留": "Add step: dwell",
    "子程序调用": "Subprogram call",
    "触发信号": "Set signal",
    "等待信号": "Wait for signal",
    "调用": "Call",
    "触发": "Set",
    "等待": "Wait",
    "驻留": "Dwell",
    "工步 {n}": "Step {n}",
    "已添加工步 {name}": "Added step {name}",
    "先在步骤表里选中一行": "Select a row in the step table first",
    "已经到头了": "Already at the end",
    "信号名不能为空": "Signal name cannot be empty",
    "子程序名不能为空": "Subprogram name cannot be empty",
    "校验通过：{n} 个工步，驻留合计 {s}s": (
        "Validation passed: {n} steps, dwell total {s}s"
    ),
    "{mark} {e} 处错误 / {w} 处警告": "{mark} {e} errors / {w} warnings",
    "（其余 {n} 条见日志）": "({n} more in the log)",
    "时序已保存：{path}": "Sequence saved: {path}",
    "载入失败: {exc}": "Load failed: {exc}",
    "{n} 个工步 ｜ 触发 {signal} ｜ 等待 {wait} ｜ 调用 {call} ｜ 驻留合计 {dwell}s": (
        "{n} steps | set {signal} | wait {wait} | call {call} | dwell total {dwell}s"
    ),
    "调用 {sub}": "Call {sub}",
    "{signal} = {value}": "{signal} = {value}",
    "等 {signal} = {value}": "Wait {signal} = {value}",
    "等 {signal}（{trg}）": "Wait {signal} ({trg})",
    "等待 {signal}": "Wait {signal}",
    "驻留 {s}s": "Dwell {s}s",
    "调用 {sub} 前置输出": "Outputs before calling {sub}",
    "末步输出": "Final outputs",
    "驻留合计是唯一可计算的时间；触发与等待的耗时取决于现场机构": (
        "Dwell total is the only computable time; trigger and wait durations "
        "depend on the on-site mechanism"
    ),
    "时序为空——设备树只编排了静态挂载，还没有任何工序": (
        "Sequence is empty — the device tree only has static mounts, no operations yet"
    ),
    "工步 {i}（{name}）": "Step {i} ({name})",
    "等待触发方式必须是 {opts} 之一，收到 {got}": (
        "Wait trigger must be one of {opts}, got {got}"
    ),
    "驻留时长不能为负，收到 {s}s": "Dwell duration cannot be negative, got {s}s",
    "不支持的时序 schema：{schema}": "Unsupported sequence schema: {schema}",
    "未知工步类型：{kind}": "Unknown step kind: {kind}",
    "（缺失）": "(missing)",
    "{where} 与工步 {j} 重名": "{where} has the same name as step {j}",
    "{where} 的泳道 {device} 不在场景设备里": (
        "{where} uses lane {device}, which is not a device in the scene"
    ),
    "{where} 触发的信号 {signal} 不在点表里——下装前必须补上映射": (
        "{where} sets signal {signal}, which is not in the point table — map it "
        "before downloading"
    ),
    "{where} 等待的信号 {signal} 不在点表里——下装前必须补上映射": (
        "{where} waits on signal {signal}, which is not in the point table — map "
        "it before downloading"
    ),
    "{where} 无限等待 {signal}——现场必须配看门狗，否则一步卡死整条线": (
        "{where} waits on {signal} forever — a watchdog is mandatory on site, "
        "otherwise one step stalls the whole line"
    ),
    "{where} 是{trg}触发，期望电平不参与判定": (
        "{where} triggers on {trg}, so the expected level is not used in the check"
    ),
    "{where} 调用的子程序 {sub} 不存在": (
        "{where} calls subprogram {sub}, which does not exist"
    ),
    "工步 {name}（调用 {sub}）不是程序指令，未写入——调用语法由品牌主程序层生成": (
        "Step {name} (calls {sub}) is not a program instruction and was not "
        "emitted — call syntax is generated by the brand main-program layer"
    ),
    # ---- 零件库（v1.3.3） ----
    "未知基本体类型 {kind}（支持 {opts}）": (
        "Unknown primitive kind {kind} (supported: {opts})"
    ),
    "box 基本体需要 size=(长, 宽, 高)，零件 {name}": (
        "A box primitive needs size=(L, W, H); part {name}"
    ),
    "box 尺寸必须为正，收到 {size}": (
        "Box dimensions must be positive, got {size}"
    ),
    "{kind} 基本体需要正的 radius，零件 {name}": (
        "A {kind} primitive needs a positive radius; part {name}"
    ),
    "cylinder 基本体需要正的 height，零件 {name}": (
        "A cylinder primitive needs a positive height; part {name}"
    ),
    "颜色分量必须在 0~1 之间，收到 {c}": (
        "Color components must be within 0..1, got {c}"
    ),
    "零件 id 必须是字母开头的短标识符（字母/数字/_/-），收到 {id}": (
        "Part id must be a short identifier starting with a letter "
        "(letters/digits/_/-), got {id}"
    ),
    "零件缺少显示名（{id}）": "Part is missing a display name ({id})",
    "零件 {name} 没有任何几何基本体": (
        "Part {name} has no geometry primitives"
    ),
    "零件 {name} 里有重名的基本体：{dupes}": (
        "Part {name} has duplicate primitive names: {dupes}"
    ),
    "不支持的零件库 schema：{schema}（期望 {want}）": (
        "Unsupported parts-library schema: {schema} (expected {want})"
    ),
    "零件库中已存在同名零件 id：{id}": (
        "A part with id {id} already exists in the library"
    ),
    "零件库中没有 id 为 {id} 的零件": (
        "No part with id {id} in the library"
    ),
    "组件目录里已存在 {cid}——先 unregister 或换一个零件 id": (
        "{cid} already exists in the component catalog — unregister first "
        "or pick another part id"
    ),
    "自定义零件（来自零件库导入，几何固定）": (
        "Custom part (imported from a parts library; fixed geometry)"
    ),
    "导入零件…": "Import Parts…",
    "导入零件库 JSON（orolp.parts/1，参数化基本体的自定义工件/结构件）": (
        "Import a parts-library JSON (orolp.parts/1; custom workpiece / "
        "structural parts made of parametric primitives)"
    ),
    "组件库不可用，无法导入零件": (
        "Component catalog unavailable; cannot import parts"
    ),
    "导入零件库": "Import Parts Library",
    "零件库 JSON (*.json);;所有文件 (*)": "Parts Library JSON (*.json);;All Files (*)",
    "导入零件失败：{kind}: {msg}": "Part import failed: {kind}: {msg}",
    "零件库已导入：{n} 个自定义零件（「自定义」分类下，可拖入视口）": (
        "Parts library imported: {n} custom parts (under the Custom "
        "category; drag into the viewport)"
    ),

    # ---- v1.3.4 工程文件（`.orolp`）与界面接线 ---- #
    " •未保存": " •unsaved",
    "未命名工程": "Untitled project",
    "保存工程": "Save Project",
    "打开工程": "Open Project",
    "工程文件模块不可用。": "Project file support is unavailable.",
    "应用时的问题：": "Issues while applying:",
    "打开工程(&O)…": "&Open Project…",
    (
        "打开 .orolp 工程文件：一次载入场景、示教程序（含抓取/放置语义）、工艺时序、设备点表与零件库来源。\n"
        "缺段/坏段只跳过那一段并在报告里说明——不会因为一段格式问题打不开整个工程。"
    ): (
        "Open a .orolp project: loads the scene, the teach program (including the pick/place semantics that only live in the project file), the process sequence, the device IO table and the parts-library sources in one go.\n"
        "A missing or broken section only skips that section and is reported — a single bad section never blocks the whole project."
    ),
    "保存工程(&S)…": "&Save Project…",
    (
        "把当前工作会话存成一个 .orolp 工程文件（原子写入）。\n"
        "本库不认识的段与字段会原样保留，往返一趟不丢内容。"
    ): (
        "Save the current work session as a single .orolp project file (atomic write).\n"
        "Sections and fields this library does not understand are preserved verbatim, so a round trip never loses content."
    ),
    "场景段已跳过：环境建模不可用": "Scene section skipped: world modeling unavailable",
    "示教程序段已跳过：路径编辑器不可用": "Teach program section skipped: path editor unavailable",
    "工艺时序段已跳过：时序编辑器不可用": "Process sequence section skipped: sequence editor unavailable",
    "设备点表段已跳过：工作单元模块不可用": "Device IO table section skipped: workcell module unavailable",
    "保存工程失败": "Save Project Failed",
    "工程已保存：{name}（{n} 段）": "Project saved: {name} ({n} sections)",
    "打开工程失败": "Open Project Failed",
    "已打开 {name}": "Opened {name}",
    "工程已打开：{name}": "Project opened: {name}",
    "零件库来源已跳过：机器组件库不可用": "Parts sources skipped: robot library unavailable",
    "场景段载入失败：{msg}": "Scene section failed to load: {msg}",
    "示教点语义 {a} 个与解析出的 {b} 个点不一致，已按默认语义重建（不猜哪个是抓取点）": (
        "{a} point semantics do not match the {b} points parsed from the code; "
        "rebuilt with default semantics (we do not guess which point is the pick)"
    ),
    "工艺时序段载入失败：{msg}": "Process sequence section failed to load: {msg}",
    "设备点表段载入失败：{msg}": "Device IO table section failed to load: {msg}",
    "零件库文件找不到：{src}（未导入；工程里仍记录着它）": (
        "Parts-library file not found: {src} (not imported; the project still "
        "records it)"
    ),
    "已从工程恢复零件库：{n} 个自定义零件": "Parts library restored from the project: {n} custom parts",
    "场景里的机器人模型找不到：{urdf}（场景仍已载入）": (
        "Robot model referenced by the scene not found: {urdf} (the scene itself "
        "was still loaded)"
    ),
    "零件库导入失败：{src}（{msg}）": "Parts library import failed: {src} ({msg})",
    "示教点语义含未知值（{msg}），已按默认语义重建": (
        "Point semantics contain an unknown value ({msg}); rebuilt with default "
        "semantics"
    ),
    "RobotWorld 工程 (*.orolp);;所有文件 (*)": "RobotWorld project (*.orolp);;All Files (*)",
    "示教程序": "Teach program",
    "各机示教程序": "Per-robot teach programs",
    "各机示教程序段必须是对象": "Per-robot teach programs section must be an object",
    "各机示教程序段：{name} 的内容必须是对象":
        "Per-robot teach programs: {name} must be an object",
    "各机示教程序段：{name} 的 positions 必须是数组":
        "Per-robot teach programs: positions of {name} must be an array",
    "多机示教点段已跳过：离线编程模块不可用":
        "Per-robot teach points skipped: offline programming module unavailable",
    "工艺时序": "Process sequence",
    "设备点表": "Device IO table",
    "零件库来源": "Parts sources",
    "标定集": "Calibration set",
    "未知段": "Unknown section",
    "（未指定）": "(not specified)",
    "（内容为空）": "(empty)",
    "{title}：未知段": "{title}: unknown section",
    "场景段必须是对象": "The scene section must be an object",
    "场景段的 objects 必须是数组": "The scene section's objects must be an array",
    "示教程序段必须是对象": "The teach program section must be an object",
    "示教程序段缺 code": "The teach program section is missing code",
    "示教程序段的 code 必须是字符串": "The teach program section's code must be a string",
    "示教程序段的 point_types 必须是数组": "The teach program section's point_types must be an array",
    "工艺时序段必须是对象": "The process sequence section must be an object",
    "工艺时序段的 steps 必须是数组": "The process sequence section's steps must be an array",
    "设备点表段必须是对象": "The device IO table section must be an object",
    "设备点表段缺 devices": "The device IO table section is missing devices",
    "设备点表段的 devices 必须是数组": "The device IO table section's devices must be an array",
    "零件库来源段必须是对象": "The parts sources section must be an object",
    "零件库来源段缺 sources": "The parts sources section is missing sources",
    "零件库来源段的 sources 必须是字符串数组": "The parts sources section's sources must be an array of strings",
    "标定集段必须是对象": "The calibration section must be an object",
    "（未命名工程）": "(untitled project)",
    "查看 .orolp 工程文件的内容摘要（不加载场景/机器人）": "Print a content summary of a .orolp project file (no scene/robot loading)",
    "工程文件路径": "path to the project file",
    "以 JSON 输出摘要": "print the summary as JSON",
    "工程里没有任何可识别的段": "No recognizable section in this project",
    "{title}：{n} 个对象 ｜ {r} 台机器人（{name}）": "{title}: {n} object(s) | {r} robot(s) ({name})",
    "{title}：{n} 行 ｜ {p} 个示教点（抓取 {k} / 放置 {v}）": "{title}: {n} line(s) | {p} taught point(s) (pick {k} / place {v})",
    "{title}：{r} 台机器人 ｜ 合计 {p} 个示教点（抓取 {k} / 放置 {v}）": "{title}: {r} robot(s) | {p} taught point(s) in total (pick {k} / place {v})",
    "{title}：{n} 个工步 ｜ 驻留合计 {d}s": "{title}: {n} step(s) | dwell total {d}s",
    "{title}：{n} 台设备 ｜ {s} 个已映射信号": "{title}: {n} device(s) | {s} mapped signal(s)",
    "{title}：{n} 个零件库文件": "{title}: {n} parts-library file(s)",
    "{title}：{detail}": "{title}: {detail}",
    "设备点表里有一台设备缺 name": "A device in the IO table has no name",
    "  （空工程：没有任何段）": "  (empty project: no sections)",
    "工程文件的内容必须是一个对象": "A project file's content must be an object",
    "工程文件的 sections 必须是一个对象": "A project file's sections must be an object",
    "保存路径为空": "Save path is empty",
    "{title}：格式异常": "{title}: malformed",
    "标定集 schema 不支持：{schema}": "Unsupported calibration schema: {schema}",
    "{name}：原样保留（本版本不解释）": "{name}: preserved verbatim (not interpreted here)",
    "工程：{name}": "Project: {name}",
    "保存版本：orolp {v} ｜ 最后修改 {t}": "Saved with: orolp {v} | last modified {t}",
    "已载入 {n} 段：{names}": "Loaded {n} section(s): {names}",
    "跳过 {n} 段（内容原样保留，保存时写回）：": (
        "Skipped {n} section(s) (content preserved verbatim and written back on "
        "save):"
    ),
    "未知段 {names}——本版本不解释，原样保留": (
        "Unknown section(s) {names} — not interpreted by this version, preserved "
        "verbatim"
    ),
    "未知顶层键 {keys}——原样保留": "Unknown top-level key(s) {keys} — preserved verbatim",
    "未知工程段：{name}（可用：{opts}）": "Unknown project section: {name} (available: {opts})",
    "备注：{note}": "Note: {note}",
    "工程里没有：{names}": "Not in this project: {names}",
    "不是 RobotWorld 工程文件（schema={got}，期望 {want}）": "Not a RobotWorld project file (schema={got}, expected {want})",
    "工程由 orolp {saved} 保存，当前库版本 {cur}——按现行 schema 读取，未做版本迁移": (
        "Saved by orolp {saved}; current library version is {cur} — read with the "
        "current schema, no version migration was performed"
    ),
    "打不开：{msg}": "Cannot open: {msg}",
    "  · {name}：{reason}": "  · {name}: {reason}",
    "工程段 {name} 无法序列化：{msg}": "Project section {name} cannot be serialized: {msg}",
    "工程无法序列化：{msg}": "Project cannot be serialized: {msg}",
    "写入工程文件失败：{msg}": "Failed to write the project file: {msg}",
    "读取工程文件失败：{msg}": "Failed to read the project file: {msg}",
    "工程文件不是合法 JSON：{msg}": "Project file is not valid JSON: {msg}",
    # ---- 会话支撑（v1.3.5）：最近打开 + 崩溃自动恢复 ---- #
    "查看最近打开记录与崩溃自动恢复状态": "Show recent projects and crash-autosave status",
    "最近打开（{n} 条）：": "Recent projects ({n}):",
    "  （文件已不存在）": "  (file no longer exists)",
    "自动保存：无": "Autosave: none",
    "自动保存：{saved_at} ｜ 源工程 {src}": "Autosave: {saved_at} | source project {src}",
    "（未知时间）": "(unknown time)",
    "（未命名工程）": "(untitled project)",
    "快照：{path}": "Snapshot: {path}",
    "最近打开(&R)": "Recent &projects",
    "最近打开过的 .orolp 工程文件（最新在前，最多 10 条）。\n文件已不存在的条目会置灰，不会被悄悄藏掉。": (
        "Recently opened .orolp project files (newest first, up to 10).\n"
        "Entries whose file no longer exists are greyed out, not hidden."
    ),
    "（暂无记录）": "(no entries yet)",
    "清除最近打开记录": "Clear recent projects",
    "已清除最近打开记录": "Recent projects cleared",
    "恢复上次会话": "Restore last session",
    "检测到上次会话的自动保存。\n\n工程：{source}\n保存时间：{saved}\n\n是否恢复？（不恢复也可以稍后用「打开工程」手动打开快照：\n{snapshot}）": (
        "An autosaved snapshot from your last session was found.\n\n"
        "Project: {source}\nSaved at: {saved}\n\n"
        "Restore it? (You can also open the snapshot manually later via Open Project:\n{snapshot})"
    ),
    "已从自动保存恢复：{saved}": "Restored from autosave: {saved}",
    # ---- 工程模板（v1.3.6）：从模板新建工程 ---- #
    "从模板新建工程(&N)…": "New project from &template…",
    "从模板新建工程": "New project from template",
    "从内置模板（抓取工作站 / 分拣工位 / 机床上下料 / 3C 装配 / 汽车焊装）新建工程：\n场景与起步程序一步到位，不必从空白开始。\n当前工程有未保存改动时会先询问。": (
        "Start a new project from a built-in template (grasping workstation / "
        "sorting station / machine tending / 3C assembly / automotive welding):\n"
        "scene and starter program in one step — no blank slate.\nAsks first "
        "when the current project has unsaved changes."
    ),
    "工程模板模块不可用。": "Project template module unavailable.",
    "选择一个模板作为新工程的起点：": "Pick a template as the starting point of the new project:",
    "当前工程有未保存改动，新建模板工程会丢弃这些改动。\n\n是否继续？": (
        "The current project has unsaved changes; creating a template project "
        "will discard them.\n\nContinue?"
    ),
    "已从模板新建：{name}": "New project from template: {name}",
    "已从模板新建：{name}（尚未保存，可直接修改后「保存工程」）": (
        "New project from template: {name} (not saved yet — edit freely, then "
        "use Save Project)"
    ),
    "未知工程模板：{tid}（可用：{opts}）": "Unknown project template: {tid} (available: {opts})",
    "抓取工作站": "Grasping workstation",
    "分拣工位": "Sorting station",
    "机床上下料": "Machine tending",
    "UR5e + 吸盘 + 输送带 + 工件 + 托盘，附 8 点抓取循环示例程序（含抓取/放置语义）": (
        "UR5e + suction cup + conveyor + workpieces + pallet, with an 8-point "
        "pick-and-place sample program (pick/place semantics included)"
    ),
    "输送带 + 双料箱分拣 + 围栏，机器人与工件已摆好，示教程序从 Home 位起步": (
        "Conveyor + dual-bin sorting + fence; robot and workpieces laid out, "
        "teach program starts from Home"
    ),
    "输送带 + 台钳工位 + 机床柜体 + 成品料箱，机器人与工件已摆好，示教程序从 Home 位起步": (
        "Conveyor + vise station + machine cabinet + output bin; robot and "
        "workpieces laid out, teach program starts from Home"
    ),
    # ---- 工程模板扩容（v1.3.8）：3C 装配 / 汽车焊装 ---- #
    "3C 装配工位": "3C assembly workcell",
    "汽车焊装工位": "Automotive welding workcell",
    "上料托盘（3 块板件）+ 装配治具 + 成品托盘 + 围栏，机器人与工件已摆好，示教程序从 Home 位起步": (
        "Feed pallet (3 boards) + assembly fixture + finished pallet + fence; "
        "robot and workpieces laid out, teach program starts from Home"
    ),
    "焊接变位机 + 输送辊道 + 控制柜 + 围栏，机器人与工件已摆好，示教程序从 Home 位起步": (
        "Welding positioner + roller conveyor + controller cabinet + fence; "
        "robot and workpieces laid out, teach program starts from Home"
    ),
}
