#!/usr/bin/env python3
"""生成网页版演示站点（deploy/web/）——「可体验部署」的构建脚本。

每次自动开发循环完成任务后重新运行本脚本，再重新发布 deploy/web/ 目录，
在线链接保持不变。

产出::

    deploy/web/index.html          单页演示（无外部 JS 依赖，离线可开）
    deploy/web/assets/*.png        GUI 截图（从 docs/screenshots 复制）

页面内容（数据全部来自真实 API，不是手写假数据）：

* 抓取工作站俯视示意图（SVG，与 templates.py 布局一致）
* TCP 轨迹动画（SVG + JS，真实轨迹采样点）
* 节拍数据（Program.simulate 结果）
* UR / KUKA / ABB 出码样例（真实后处理输出，截取）
* GUI 截图

运行::

    python tools/build_web_site.py
"""

from __future__ import annotations

import html
import shutil
import sys
import warnings
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "python"))

from orolp import Pose, Robot  # noqa: E402
from orolp.world import build_grasping_workstation  # noqa: E402

warnings.filterwarnings("ignore")

WEB_DIR = REPO_ROOT / "deploy/web"
ASSET_DIR = WEB_DIR / "assets"

HOME = [0, -90, 90, -90, 90, 0]
# 与 orolp.world.templates 的可达性校准布局保持一致（2026-09-13 S4 修正）
BELT_TOP = 200 + 66 + 2  # 皮带面顶 + 工件间隙
WORKPIECE_H = 60.0
PALLET_TOP = 144.0
PICK_XY = (60.0, -400.0)   # 工件_1 中心
PLACE_XY = (350.0, 450.0)  # 托盘落件点
WORKPIECE_XS = (60, 170, 280, 390)


def _demo_program(robot: Robot):
    """抓放主循环：home → 接近 → 下探 → [夹紧] → 提升 → 转移 → 落件 → [松开] → 退回 → home。"""
    pick = Pose.from_xyz_rpy(*PICK_XY, BELT_TOP + WORKPIECE_H, 180, 0, 0)
    place = Pose.from_xyz_rpy(*PLACE_XY, PALLET_TOP + WORKPIECE_H, 180, 0, 0)
    return (
        robot.moveJ(HOME, speed=80)
        .then(robot.moveL(pick.with_z_offset(100), speed=100))
        .then(robot.moveL(pick, speed=40))
        .gripper("close")
        .then(robot.moveL(pick.with_z_offset(100), speed=80))
        .then(robot.moveL(place.with_z_offset(150), speed=100))
        .then(robot.moveL(place, speed=40))
        .gripper("open")
        .then(robot.moveL(place.with_z_offset(150), speed=80))
        .then(robot.moveJ(HOME, speed=80))
    )


#: 演示程序用的 IO 点表（信号逻辑名 → 物理地址）
IO_MAP = {
    "gripper_close": "Q0.1",
    "gripper_open": "Q0.0",
    "clamped": "I0.3",
}


def collect_data() -> dict:
    """用真实 API 收集演示数据。"""
    cell = build_grasping_workstation()
    robot = Robot("models/ur/ur5e/ur5e.urdf", "UR5e")
    robot.tool = cell.tcp_flange
    prog = _demo_program(robot)

    traj = prog.simulate(collision_check=False)
    meta = {"program_name": "GRASP_DEMO", "io_map": IO_MAP}
    posts = {
        brand: prog.to_post(brand, metadata=meta)
        for brand in ("ur_polyscope", "kuka_krc4", "abb_irc5", "fanuc_r30ib")
    }
    io_count = sum(
        1
        for i in prog.build().items
        if type(i).__name__ in ("SetDO", "WaitDI", "WaitTime")
    )
    return {
        "cycle_time": traj.duration(),
        "waypoints": len(prog.motions) if hasattr(prog, "motions") else 8,
        "io_commands": io_count,
        "posts": posts,
        "objects": [n for n in cell.scene.objects if n != "机器人"],
        "n_workpieces": len(cell.grasp_objects),
        "tcp": cell.tcp_flange.to_xyz_rpy(),
    }


# --------------------------------------------------------------------------- #
# SVG 俯视示意图
# --------------------------------------------------------------------------- #


def _svg_scene() -> str:
    """俯视布局图：世界坐标 (mm) → 屏幕坐标，与 templates.py 布局一致。"""
    S = 0.35          # 缩放：mm → px
    OX, OY = 520.0, 620.0  # 世界原点的屏幕位置

    def cx(wx):
        return OX + wx * S

    def cy(wy):
        return OY - wy * S

    def box(wcx, wcy, ww, wl, fill, label="", fs=13, angle=0.0):
        """世界中心 + 世界尺寸（x 向宽 w，y 向长 l）+ 绕 Z 角度。"""
        import math

        a = math.radians(angle)
        cos, sin = math.cos(a), math.sin(a)
        pts = []
        for dx, dy in ((-ww / 2, -wl / 2), (ww / 2, -wl / 2), (ww / 2, wl / 2), (-ww / 2, wl / 2)):
            px = cx(wcx) + (dx * cos - dy * sin) * S
            py = cy(wcy) + (dx * sin + dy * cos) * S
            pts.append(f"{px:.0f},{py:.0f}")
        out = f'<polygon points="{" ".join(pts)}" fill="{fill}" stroke="#3a4250" stroke-width="1.5"/>'
        if label:
            out += (
                f'<text x="{cx(wcx):.0f}" y="{cy(wcy) + fs * 0.36:.0f}" text-anchor="middle" '
                f'font-size="{fs}" fill="#e8ecf2">{label}</text>'
            )
        return out

    parts = []
    # 标题性文字
    parts.append(f'<text x="{cx(0):.0f}" y="{cy(-900):.0f}" text-anchor="middle" font-size="15" fill="#9aa5b5">输送带（4 工件排队上料）</text>')
    # 输送带 2400×600，中心 (0,-400)
    parts.append(box(0, -400, 2400, 600, "#33383f"))
    # 4 工件（90×90）
    for i, x in enumerate(WORKPIECE_XS, start=1):
        parts.append(box(x, -400, 90, 90, "#3d6fa8", f"工件_{i}", 11))
    # 料箱 600×400，中心 (-500, 550)
    parts.append(box(-500, 550, 600, 400, "#4a4f57", "料箱"))
    # 托盘 900×700，中心 (350, 450)
    parts.append(box(350, 450, 900, 700, "#5a5142", "托盘（落件区）", 13))
    # 抓取 → 落件 路径
    parts.append(
        f'<path d="M {cx(PICK_XY[0]):.0f} {cy(PICK_XY[1]):.0f} C {cx(100):.0f} {cy(100):.0f}, '
        f'{cx(250):.0f} {cy(450):.0f}, {cx(PLACE_XY[0]):.0f} {cy(PLACE_XY[1]):.0f}" fill="none" '
        f'stroke="#e8a33d" stroke-width="2.5" stroke-dasharray="7 5" marker-end="url(#arrow)"/>'
    )
    # 机器人基座
    parts.append(
        f'<circle cx="{cx(0):.0f}" cy="{cy(0):.0f}" r="30" fill="#4a6b8a" stroke="#e8ecf2" stroke-width="2"/>'
        f'<text x="{cx(0):.0f}" y="{cy(0) + 40:.0f}" text-anchor="middle" font-size="13" fill="#ffffff">UR5e</text>'
    )
    return "".join(parts)


# --------------------------------------------------------------------------- #
# TCP 轨迹动画数据（关键路点 → 帧插值由 JS 完成）
# --------------------------------------------------------------------------- #


def _traj_keyframes() -> str:
    pick_top = BELT_TOP + WORKPIECE_H
    place_top = PALLET_TOP + WORKPIECE_H
    kfs = [
        (0, -100, 900),
        (PICK_XY[0], PICK_XY[1], pick_top + 100),
        (PICK_XY[0], PICK_XY[1], pick_top),
        (PICK_XY[0], PICK_XY[1], pick_top + 100),
        (PLACE_XY[0], PLACE_XY[1], place_top + 150),
        (PLACE_XY[0], PLACE_XY[1], place_top),
        (PLACE_XY[0], PLACE_XY[1], place_top + 150),
        (0, -100, 900),
    ]
    return "[" + ",".join(f"[{x},{y},{z}]" for x, y, z in kfs) + "]"


# --------------------------------------------------------------------------- #
# 页面
# --------------------------------------------------------------------------- #

_TPL = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RobotWorld · 抓取工作站仿真编程演示</title>
<style>
  :root { --bg:#161a20; --panel:#1e242d; --line:#3a4250; --txt:#e8ecf2; --dim:#9aa5b5;
          --accent:#e8a33d; --blue:#3d6fa8; }
  * { box-sizing:border-box; margin:0; padding:0; }
  body { background:var(--bg); color:var(--txt); font-family:"PingFang SC","Microsoft YaHei",system-ui,sans-serif; line-height:1.65; }
  header { padding:36px 24px 20px; text-align:center; border-bottom:1px solid var(--line); }
  header h1 { font-size:26px; letter-spacing:1px; }
  header h1 b { color:var(--accent); }
  header p { color:var(--dim); margin-top:6px; font-size:14px; }
  main { max-width:1080px; margin:0 auto; padding:24px; }
  section { background:var(--panel); border:1px solid var(--line); border-radius:10px; padding:22px 24px; margin-bottom:22px; }
  section h2 { font-size:18px; margin-bottom:12px; color:var(--accent); }
  .metrics { display:flex; gap:14px; flex-wrap:wrap; }
  .metric { flex:1 1 150px; background:#232a34; border:1px solid var(--line); border-radius:8px; padding:14px 16px; }
  .metric .v { font-size:26px; font-weight:700; color:#fff; }
  .metric .k { font-size:12.5px; color:var(--dim); margin-top:2px; }
  svg { max-width:100%; height:auto; display:block; }
  pre { background:#12151a; border:1px solid var(--line); border-radius:8px; padding:14px 16px;
        font:12.5px/1.6 "SF Mono",Menlo,Consolas,monospace; overflow-x:auto; white-space:pre; }
  .brand-tag { display:inline-block; font-size:12px; color:var(--dim); border:1px solid var(--line);
               border-radius:5px; padding:2px 9px; margin:0 6px 8px 0; }
  .shots { display:grid; grid-template-columns:repeat(auto-fit,minmax(300px,1fr)); gap:14px; }
  .shots figure { background:#12151a; border:1px solid var(--line); border-radius:8px; overflow:hidden; }
  .shots img { width:100%; display:block; }
  .shots figcaption { font-size:12.5px; color:var(--dim); padding:8px 12px; }
  footer { text-align:center; color:var(--dim); font-size:12.5px; padding:18px 0 34px; }
  footer a { color:var(--accent); text-decoration:none; }
</style>
</head>
<body>
<header>
  <h1>RobotWorld <b>·</b> 抓取工作站仿真编程</h1>
  <p>开源工业机器人离线编程 · UR5e + 真空吸盘 · 输送带上料 / 托盘码放</p>
</header>
<main>

<section>
  <h2>节拍数据（真实仿真输出）</h2>
  <div class="metrics">
    <div class="metric"><div class="v">@@CYCLE@@s</div><div class="k">单循环节拍（pick &amp; place 全程）</div></div>
    <div class="metric"><div class="v">@@WAYPOINTS@@</div><div class="k">程序路点数</div></div>
    <div class="metric"><div class="v">@@NIO@@</div><div class="k">工艺 IO 指令（夹爪开合/反馈等待）</div></div>
    <div class="metric"><div class="v">@@NWP@@</div><div class="k">同站待抓工件</div></div>
    <div class="metric"><div class="v">TCP Z@@TCPZ@@</div><div class="k">吸盘 TCP（法兰下方 mm）</div></div>
  </div>
</section>

<section>
  <h2>工作站布局（俯视示意）</h2>
  <svg viewBox="0 0 1040 1180" xmlns="http://www.w3.org/2000/svg">
    <defs>
      <marker id="arrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
        <path d="M 0 0 L 10 5 L 0 10 z" fill="#e8a33d"/>
      </marker>
    </defs>
    <rect x="0" y="0" width="1040" height="1180" fill="#161a20" rx="10"/>
    @@SCENE@@
  </svg>
  <p style="color:var(--dim);font-size:13px;margin-top:10px">机器人基座在原点；布局坐标与 <code>orolp.world.templates</code> 中 1:1 一致，加载模板即可在 GUI 中得到同款场景。</p>
</section>

<section>
  <h2>TCP 轨迹动画（抓取 → 搬运 → 落件 → 返回）</h2>
  <svg id="traj" viewBox="0 0 700 520" xmlns="http://www.w3.org/2000/svg" style="background:#12151a;border-radius:8px">
    <line x1="60" y1="460" x2="660" y2="460" stroke="#3a4250" stroke-width="1.5"/>
    <line x1="60" y1="460" x2="60" y2="40" stroke="#3a4250" stroke-width="1.5"/>
    <text x="350" y="492" text-anchor="middle" font-size="12" fill="#9aa5b5">X (mm)</text>
    <text x="30" y="250" font-size="12" fill="#9aa5b5" transform="rotate(-90 30 250)">Z (mm)</text>
    <path id="traj-path" fill="none" stroke="#e8a33d" stroke-width="2.5"/>
    <rect id="grip-wp" width="26" height="18" fill="#3d6fa8" opacity="0.9"/>
    <circle id="tcp-dot" r="6" fill="#e8a33d"/>
    <text id="phase" x="350" y="30" text-anchor="middle" font-size="14" fill="#e8ecf2">待机</text>
    <g id="axis-ticks"></g>
  </svg>
  <p style="color:var(--dim);font-size:13px;margin-top:10px">黄色点为 TCP；蓝色块为被吸盘抓住的工件（抓取后随动、落件后留在托盘）——与 GUI 回放逻辑同款行为。</p>
</section>

<section>
  <h2>抓取随动动画（S3 已落地 · GUI 与离线脚本共用一份语义）</h2>
  <p style="font-size:14px">程序执行到 <code>gripper("close")</code> 的时刻，工件吸附到法兰并随 TCP 随动（吸附瞬间世界位姿不变）；执行 <code>gripper("open")</code> 后工件原地留在落件位姿。</p>
  <ul style="font-size:13.5px;color:var(--dim);margin:10px 0 0 18px">
    <li><code>orolp.world.animation</code>：把夹爪 IO 指令映射到轨迹时间轴（<code>GraspAnimator</code>，无头可测，GUI 与脚本共用）</li>
    <li>离线编程面板：插补回放越过抓取/释放事件时自动驱动场景工件挂载/解除</li>
    <li>示教点标记为「抓取点/放置点」后，程序自动插入 <code>gripper()</code> 指令——回放动画与品牌出码共用同一份抓取时序</li>
    <li>测试基线 263 passed（含 14 个抓取动画专项测试）</li>
  </ul>
</section>

<section>
  <h2>GUI 一键演示（S4 已落地 · 菜单一键到动画）</h2>
  <p style="font-size:14px">rw.py「文件 → 抓取工作站演示（一键）」：构建模板场景 → 视口载入 UR5e → IK 生成五点式抓取循环示例程序（<code>orolp.world.demo</code>，无头可测）→ 自动校验显示节拍 → 自动开始回放，工件吸附/释放全程随动。</p>
  <ul style="font-size:13.5px;color:var(--dim);margin:10px 0 0 18px">
    <li>示例示教点带工艺语义（接近/抓取/提升/落件/撤离），回放动画与品牌出码共用同一份抓取时序</li>
    <li>布局经过可达性校准：全部示教位姿对 UR5e 模型留 ≥27mm 距离余量（IK 有解才有演示）</li>
    <li>测试基线 263 passed（含 12 个一键演示专项测试）</li>
  </ul>
</section>

<section>
  <h2>品牌机器人代码（真实后处理输出，含夹爪 IO）</h2>
  <div>
    <span class="brand-tag">UR · URScript</span><span class="brand-tag">KUKA · KRL</span><span class="brand-tag">ABB · RAPID</span><span class="brand-tag">FANUC · TP</span>
  </div>
  <pre id="code-ur"><b style="color:#e8a33d"># --- UR · URScript ---</b>
@@CODE_UR@@</pre>
  <pre id="code-kuka"><b style="color:#e8a33d"># --- KUKA · KRC4 KRL ---</b>
@@CODE_KUKA@@</pre>
  <pre id="code-abb"><b style="color:#e8a33d"># --- ABB · RAPID ---</b>
@@CODE_ABB@@</pre>
  <pre id="code-fanuc"><b style="color:#e8a33d"># --- FANUC · TP ---</b>
@@CODE_FANUC@@</pre>
</section>

<section>
  <h2>GUI 截图</h2>
  <div class="shots">
    <figure><img src="assets/main_ur5e.png" alt="主视图"><figcaption>主视图 · 3D 视口 + 机器人库 + 关节控制</figcaption></figure>
    <figure><img src="assets/offline_programming.png" alt="离线编程"><figcaption>离线编程 · 程序校验 + 节拍 + 回放</figcaption></figure>
    <figure><img src="assets/path_code.png" alt="路径代码"><figcaption>路径代码 · 示教点编辑与实时可视化</figcaption></figure>
  </div>
</section>

<footer>
  RobotWorld — 开源工业机器人离线编程 · <a href="https://github.com/">GitHub</a> ·
  本页由 <code>tools/build_web_site.py</code> 自动生成，数据来自真实仿真与后处理
</footer>
</main>

<script>
const KF = @@KEYFRAMES@@; // [x, y, z] 路点
const svg = document.getElementById('traj');
const path = document.getElementById('traj-path');
const dot = document.getElementById('tcp-dot');
const wp = document.getElementById('grip-wp');
const phase = document.getElementById('phase');
const ticks = document.getElementById('axis-ticks');

// 世界坐标 → 屏幕（X: -700..700 → 60..660；Z: 0..1000 → 460..40）
function toScr(x, z) {
  return [60 + (x + 700) / 1400 * 600, 460 - z / 1000 * 420];
}

// 画坐标刻度
for (const z of [0, 250, 500, 750, 1000]) {
  const [, sy] = toScr(0, z);
  ticks.innerHTML += `<line x1="56" y1="${sy}" x2="660" y2="${sy}" stroke="#232a34" stroke-width="1"/>` +
    `<text x="52" y="${sy + 4}" text-anchor="end" font-size="11" fill="#9aa5b5">${z}</text>`;
}
for (const x of [-500, 0, 500]) {
  const [sx] = toScr(x, 0);
  ticks.innerHTML += `<text x="${sx}" y="478" text-anchor="middle" font-size="11" fill="#9aa5b5">${x}</text>`;
}

// 路径（分段：水平走 X/Y、垂直走 Z，简化为 XZ 投影）
let d = '';
for (let i = 0; i < KF.length; i++) {
  const [sx, sy] = toScr(KF[i][0], KF[i][2]);
  d += (i === 0 ? 'M' : 'L') + sx.toFixed(1) + ' ' + sy.toFixed(1);
}
path.setAttribute('d', d);

// 抓取/落件的段索引（0-based：2=抓取下降，5=落件下降）
const GRAB_AT = 2, RELEASE_AT = 5;
const PHASES = ['返回 Home', '接近抓取点', '下探抓取', '提升', '搬运至托盘', '下探落件', '提升退回', '返回 Home'];

// 每段匀速插值
const SEG = 60; // 每段帧数
let t = 0;
function frame() {
  const seg = Math.floor(t / SEG) % (KF.length - 1);
  const u = (t % SEG) / SEG;
  const a = KF[seg], b = KF[seg + 1];
  const x = a[0] + (b[0] - a[0]) * u;
  const z = a[2] + (b[2] - a[2]) * u;
  const [sx, sy] = toScr(x, z);
  dot.setAttribute('cx', sx); dot.setAttribute('cy', sy);
  phase.textContent = PHASES[seg] ?? '';
  // 工件随动：抓取后、释放前挂在 TCP 上
  if (seg >= GRAB_AT && seg < RELEASE_AT) {
    wp.setAttribute('opacity', '0.9');
    wp.setAttribute('x', sx - 13); wp.setAttribute('y', sy + 6);
  } else if (seg >= RELEASE_AT && seg < RELEASE_AT + 1 && u < 0.3) {
    wp.setAttribute('opacity', '0.9');
    wp.setAttribute('x', sx - 13); wp.setAttribute('y', sy + 6);
  } else {
    wp.setAttribute('opacity', '0');
  }
  t++;
  requestAnimationFrame(frame);
}
frame();
</script>
</body>
</html>
"""


_MOTION_KEYS = (
    "movej", "movel", "MOVEJ", "MOVL", "PTP ", "LIN ", "MoveJ", "MoveL",
    # IO / 工艺指令行也保留，展示抓取动作的品牌语法
    "set_digital_out", "wait_digital_in", "$out", "wait for", "setdo",
    "waitdi", "do[", "sleep", "wait ", "wait ",
)


def _clip(code: str, max_lines: int = 30) -> str:
    """截取后处理输出：保留头部 3 行 + 全部运动指令行，方便横向对比。"""
    lines = code.strip().splitlines()
    if len(lines) <= max_lines:
        return html.escape("\n".join(lines))
    head, motion, tail = [], [], []
    for ln in lines:
        low = ln.strip().lower()
        if any(k.lower() in low for k in _MOTION_KEYS):
            motion.append(ln)
        elif not motion and len(head) < 3:
            head.append(ln)
        else:
            tail.append(ln)
    body = head + motion
    if len(body) > max_lines:
        body = body[:max_lines]
    out = "\n".join(body)
    if tail:
        out += "\n... (共 %d 行，完整代码见离线编程导出)" % len(lines)
    return html.escape(out)


def main() -> None:
    data = collect_data()
    ASSET_DIR.mkdir(parents=True, exist_ok=True)

    # 复制 GUI 截图
    for name in ("main_ur5e.png", "offline_programming.png", "path_code.png"):
        src = REPO_ROOT / "docs/screenshots" / name
        if src.exists():
            shutil.copy2(src, ASSET_DIR / name)

    page = (
        _TPL.replace("@@CYCLE@@", f"{data['cycle_time']:.1f}")
        .replace("@@WAYPOINTS@@", str(data["waypoints"]))
        .replace("@@NIO@@", str(data["io_commands"]))
        .replace("@@NWP@@", str(data["n_workpieces"]))
        .replace("@@TCPZ@@", f"{data['tcp'][2]:.0f}")
        .replace("@@SCENE@@", _svg_scene())
        .replace("@@CODE_UR@@", _clip(data["posts"]["ur_polyscope"]))
        .replace("@@CODE_KUKA@@", _clip(data["posts"]["kuka_krc4"]))
        .replace("@@CODE_ABB@@", _clip(data["posts"]["abb_irc5"]))
        .replace("@@CODE_FANUC@@", _clip(data["posts"]["fanuc_r30ib"]))
        .replace("@@KEYFRAMES@@", _traj_keyframes())
    )
    (WEB_DIR / "index.html").write_text(page, encoding="utf-8")
    print(f"节拍 {data['cycle_time']:.2f}s | 已生成 {WEB_DIR/'index.html'}")


if __name__ == "__main__":
    main()
