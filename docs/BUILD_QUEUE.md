# RobotWorld 自动开发队列（BUILD_QUEUE）

> 本文件是「自动开发循环」的执行入口。定时任务每次运行时按此文件工作，完成后通过飞书通知用户。
> 手动触发方式：对 WorkBuddy 说"继续 RobotWorld 队列"即可。

## 🎯 短期目标：发布「抓取工作站仿真编程」能力展示

> 2026-09-12 确定。目的：拿得出手的演示——在 GUI 里一键加载抓取工作站，机器人完成 pick & place 循环动画（含抓取/释放、工件随动、节拍显示），并输出真实品牌机器人代码。
> 现状基线：`examples/pick_and_place.py` 已有 API 级演示（工具系/wobj/链式程序/5 品牌出码），缺 IO 指令、工作站场景、抓取动画、GUI 集成。

### 展示任务（优先于其他 Must 执行）

- [x] **S1 抓取工作站场景模板**：基于 `world/scene.py` + `components.py` 搭建标准抓取工作站——机器人（UR5e 程序生成模型）+ 简化吸盘/夹爪工具（含 TCP 定义）+ 输送带/料箱/托盘/4 个工件几何体；存成可加载 JSON（参照 `examples/welding_cell.json` 格式），交付 `examples/assets/grasping_workstation.json` + 加载代码。（2026-09-13 完成：新增 `orolp/world/templates.py` 模板框架 + `material_bin`/`pallet_base` 组件；JSON 内嵌 template 元数据，`Scene.load` 兼容不受影响；测试 219 passed）
- [x] **S2 工艺 IO 指令（v0.3.3 的展示子集）**：Program 加 `setDO` / `waitDI` / `gripper(open|close)` 指令；后处理输出对应品牌 IO 语法（UR `set_digital_out`、KUKA `OUT`、FANUC `DO`，其余品牌可先降级为注释）；更新 pick_and_place.py 去掉两处 IO 占位注释。测试：`tests/test_io_instructions.py`。（2026-09-13 完成：`Program`/`ProgramBuilder` 链式 `setDO`/`waitDI`/`waitTime`/`gripper`（信号互锁 + clamped 反馈等待）；FANUC TP 补齐品牌语法 `DO[n]=ON`/`WAIT DI[n]`/`WAIT n(sec)`（超时留提示注释）；修复 `then(Program)` 合并丢 IO 指令的缺陷；pick_and_place.py 占位注释替换为 `.gripper("close"/"open")` 并修正后处理名 `abb_rapid→abb_irc5`、`fanuc_tp→fanuc_r30ib`；新增 14 测试，233 passed + 2 skipped）
- [x] **S3 抓取动画与工件随动**：仿真回放时，程序执行到抓取指令后，工件挂到 TCP 随动，执行释放指令后留在 place 位姿；碰撞/穿透不做硬校验（后续 v0.2.1 碰撞检测上线后接入）。涉及 `world/panel.py` 回放逻辑 + `rw.py` 视口刷新。（2026-09-13 完成：新增 `orolp/world/animation.py`——`gripper_events()` 把程序里的夹爪指令映射到轨迹时刻（IO 事件=上一条运动的边界），`GraspAnimator.sync()` 按播放头驱动 `Scene.attach/detach`（吸附瞬间保持世界位姿→随 TCP 随动，释放原地留下）；修复 planner 规划失败的指令不占时间区间的缺陷（此前会让事件时刻错位）；ProgramPanel 校验后保留规划器、按示教点 PICK/PLACE 语义自动插入 `gripper()`（回放+出码共用一份时序）、回放越事件发 `gripperStateChanged` 信号；rw.py 连接该信号完成工件吸附/释放+场景树+状态栏+视口刷新；新增 14 测试，251 passed + 2 skipped）
- [x] **S4 GUI 一键展示**：rw.py 加"抓取工作站"一键加载入口（机器人库或 OLP 标签），加载后自动载入示例程序；回放控制（已有）+ 节拍时间显示；运行稳定无崩溃。（2026-09-13 完成：File 菜单新增「抓取工作站演示（一键）」——构建模板场景 → 视口载 UR5e → `orolp.world.demo.demo_taught_points` 用模板 TCP + IK 生成 8 点五点式抓取循环示教点（VIA/APPROACH/PICK/LIFT/VIA/PLACE/RETRACT/VIA，含随机初值重试）→ 灌入路径编辑器与离线编程面板（与手工示教同一条数据通路）→ 自动校验出节拍 → 自动开始回放；**附带修复 S1 遗留几何缺陷**：原布局（输送带 y=-600/皮带面 768mm、托盘 (450,550)）超出 UR5e 模型工作空间，IK 全局无解（scipy 多起点优化证实非求解器问题），重校准为输送带 h=200@y=-400、工件 X∈[60,170,280,390]（长 90mm）、托盘 (350,450)，全部位姿余量 ≥27mm；新增 `tests/test_gui_demo.py` 12 测试，263 passed + 2 skipped）
- [ ] **S5 展示物料**：`examples/grasping_workstation_demo.py`（无 GUI 一键跑通全流程：仿真→节拍→UR/KUKA/ABB 出码打印）+ README「Key Features Demo」增加抓取工作站小节 + 3 张截图（docs/screenshots/）。
- [x] **S6 Web 可体验部署（持续维护）**：建一个网页版演示站点（静态页 + three.js/动画：抓取工作站示意、TCP 轨迹动画、节拍数据、UR/KUKA/ABB 出码样例、GUI 截图），源文件放 `/Users/jayhan/RobotWorld/deploy/web/`。通过 WorkBuddy 的「发布为应用」能力发布为在线链接。**此后每次任务完成后更新该页面内容并重新发布同一目录，保持链接不变**，通知里始终附上最新链接。（2026-09-13 首版上线：`tools/build_web_site.py` 自动生成，数据来自真实仿真（节拍 8.1s）与真实后处理；在线链接 https://f1b8fd6f91ea40d7b56c733d46aea300.app.workbuddy.host ；后续每次任务完成后运行 `python tools/build_web_site.py` 并重新发布同一目录）

### 展示任务与主队列的关系

S2 完成后把 v0.3.3 剩余部分（工艺模板库、waitDI 完整语义）留在原任务继续；S3 的碰撞校验依赖 v0.2.1，上线后回补。S6 在 S1 完成后即可开始搭建，之后作为持续维护项：每次任务完成后同步更新并重新发布，链接保持不变。

## 执行规则

1. **取任务**：优先取「🎯 短期目标」区第一个未完成任务（S1→S5 顺序）；短期目标全部完成后，再按主队列顺序取第一个未完成任务。
2. **实现**：参考 `docs/ROADMAP.md` 对应小节的技术方案；动手前先读相关源码（`python/orolp/`、`rw.py`）。
3. **测试**：为新增功能编写/更新测试（`tests/`、`python/tests/`），运行 `pytest tests python/tests` 全量通过后才算完成（2026-09-13 基线 263 passed + 2 skipped，不得引入回归；旧的 251/219/137 基线已过时）。
4. **记账**：完成后勾选本文件对应项，同步勾选 `docs/ROADMAP.md` 第 3 节，并把能力写入 `README.md` 开发进展表。
5. **通知**：通过飞书给用户发消息，包含：完成内容摘要、测试结果、改动文件清单、**需要用户反馈/决策的问题**、下一个任务预告。
6. **阻塞处理**：遇到无法自行决策的阻塞（如缺厂商 STEP、缺真机参数），在任务后标注 `[阻塞: 原因]`，跳到下一个任务，并在飞书消息中说明。不空转、不瞎猜。

## 任务队列

### Must（决定软件"能用"）

- [ ] **v0.2.1 碰撞检测模块**：`python/orolp/collision.py` + `tests/test_collision.py`。基于 `world/components` 几何体做包围体层次（AABB/OBB/球体树），连杆-环境、连杆-连杆检测；Simulation 每路点查询，碰撞标红并定位"撞在哪一段/哪个物体"；复用 jacobian 做奇异附近软预警。
- [ ] **v0.2.2 速度/加速度 S 曲线规划 + 节拍报表**：Waypoint 已带 velocity/acceleration 字段但规划器未使用；给 moveJ/moveL 加梯形/S 型速度曲线，按轴限幅，输出真实 cycle_time（`_expand` 已留 `path_length`）；后处理同步输出 VEL/ACC/CIRC 指令。
- [ ] **v0.2.3 5 品牌真机 CAD 接入** `[阻塞: 等用户把官方 STEP 放入 assets/vendor/<brand>/step/]`：工具 `tools/build_vendor_urdf.py` 已就绪（--probe 验轴系 → --all 生成 URDF+网格+预览），逐品牌复制 test_vendor_<brand>.py 套件（仿 test_vendor_jaka.py）。汇川仅有后处理无 URDF，CAD 到位后一并补。
- [ ] **v0.2.4 GUI 轨迹编辑器**：rw.py 离线编程标签目前只读；加路点增删拖、目标位姿手填、运动类型切换、实时碰撞/节拍着色、一键导出 .urp/.src。复用 program_panel.py 骨架。

### Should（决定软件"好用"）

- [ ] **v0.3.1 动力学/力矩校验**：用 jacobian 做逆动力学（Newton-Euler 或解析），校验每段力矩是否超额定，输出力矩/功率报表，GUI 超额定标红。质量/惯量参数：真机 CAD 估算质心 + 手册录入双轨。
- [ ] **v0.3.2 STEP/CAM 轨迹生成**：STEP/IGES 导入 → 提取焊缝/边/面 →生成 moveL/moveC 路径（焊/磨/喷/切）；与 build_vendor_urdf.py 的 OCP 通道复用；点云（PLY/PCAP）导入做扫描偏差比对。
- [ ] **v0.3.3 IO 与工艺参数**：Program 加 setDO/waitDI/起弧/熄弧/喷涂开等指令；后处理输出各品牌 IO 语法（UR set_digital_out、KUKA OUT、FANUC DO…）；焊接/喷涂工艺库做成可配模板。
- [ ] **v0.3.4 标定与手眼**：TCP 标定（4 点法）、基坐标/工具坐标标定导入；手眼标定（eye-to-hand / eye-in-hand）求外参；结果写回 URDF/Program。

### Could（差异化）

- [ ] **v1.0.1 多机器人协同**：world/scene.py 扩展多台 Robot 共享场景、互避碰、节拍统筹。
- [ ] **v1.0.2 力控柔顺**：阻抗/导纳规划，输出 UR force_mode、KUKA COMPLIANCE 指令。
- [ ] **v1.0.3 云协作与数字孪生**：ControllerLink 预留接口落地；真机 vs 仿真轨迹偏差可视化（复用 DebugSession.deviation）。
- [ ] **v1.0.4 插件接口**：后处理/运动学/渲染器插件化（post/base.py 为模板）。

## 新增功能想法（待评审，确认后移入上面的队列）

_（用户随时口述补充在这里，下次自动开发前由 AI 整理成正式任务并给出技术方案）_

-

---
*最后更新：2026-09-13（S1 ✅、S2 ✅、S3 ✅、S4 ✅、S6 Web 演示持续维护；下一任务 S5 展示物料）*
