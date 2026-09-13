# RobotWorld 后续软件特性规划（Roadmap）

> 状态：2026-09-02 整理。基于当前 `main`（commit `b1e2fe0`）真实能力梳理。
> 本文是**规划文档**，不是已交付清单。已交付能力见 `README.md` 的开发进展表。

---

## 0. 当前能力基线（已具备，规划从这里长出来）

| 能力 | 状态 | 代码位置 |
|---|---|---|
| 运动学 FK / IK / **Jacobian** | ✅ | `python/orolp/kinematics.py`、`robot.py` |
| 轨迹规划 moveJ / moveL / moveC / moveSpline | ✅ | `python/orolp/planner.py` |
| 7 品牌后处理（UR/KUKA/ABB/FANUC/JAKA/Estun/汇川） | ✅ | `python/orolp/post/` |
| 代码上传 Link（File/Sim/预留 Controller） | ✅ | `python/orolp/link.py` |
| 在线调试 DebugSession（单步/播放/偏差） | ✅ | `python/orolp/debug.py` |
| 场景/世界管理（几何、组件、场景、面板） | ✅（基础） | `python/orolp/world/` |
| 机器人库目录（34 型号） | ✅ | `python/orolp/robot_library/`、`models/` |
| 离线编程 GUI（PySide6 + OpenGL） | ✅ | `rw.py` |
| 测试套件 | ✅ 137 passed | `tests/`、`python/tests/` |

**关键缺口（规划要补的）**：碰撞检测、动力学/力矩、速度/加速度级规划与节拍优化、多品牌真机 CAD 接入、CAD/CAM 轨迹生成、IO/工艺参数、多机器人协同、标定与手眼、插件生态。

---

## 1. 优先级分层（MoSCoW）

- **Must（必须，决定软件"能用"）**：碰撞检测、真机 CAD 接入、速度/节拍规划、GUI 轨迹编辑器
- **Should（应该有，决定"好用"）**：动力学/力矩校验、CAD/CAM 导入、IO 与工艺参数、标定导入
- **Could（可以有，决定"差异化"）**：多机协同、力控柔顺、点云、云协作、插件市场
- **Won't（本阶段不做）**：自研控制器固件、实时 Linux RT 内核、自研 CAD 内核

---

## 2. 路线图

### v0.2 — "可信仿真"（近期，目标 1–2 个月）

让离线程序在"发出去之前"就能发现会撞、会超节拍、会超力矩。

1. **碰撞检测（Collision）** — *Must*
   - 基于 `world/components` 已建几何体，加包围体层次（AABB / OBB / 球体树）做连杆-环境、连杆-连杆检测。
   - `Simulation` 每路点跑一次碰撞查询，碰撞即标红并给出"撞在哪一段/哪个物体"。
   - 复用 `jacobian` 做**自运动奇异附近**的软碰撞预警。
   - 交付：`python/orolp/collision.py` + `tests/test_collision.py`。

2. **速度 / 加速度级规划与节拍优化** — *Must*
   - 当前 `Waypoint` 已带 `velocity/acceleration` 字段但规划器还没用上。
   - 加 S 型速度曲线（梯形/S-curve）到 moveJ/moveL，按各轴最大速度/加速度限幅，输出真实节拍。
   - 后处理同步输出 `VEL`/`ACC`/`CIRC` 工艺指令（已有 base 降级框架可扩展）。
   - 交付：planner 扩展 + 节拍报表（已在 `_expand` 里留 `path_length`，补 `cycle_time`）。

3. **多品牌真机 CAD 接入** — *Must（卡点）*
   - 工具 `tools/build_vendor_urdf.py` 已支持 `--brand/--probe/--all`，等用户把 5 品牌官方 STEP 放进 `assets/vendor/<brand>/step/`。
   - 到位后逐品牌跑 `--probe` 验轴系 → `--all` 生成真机 URDF + 网格 + 预览，并复制 `tests/test_vendor_<brand>.py` 套件（仿 `test_vendor_jaka.py`）。
   - 汇川仅有后处理、无程序化 URDF，真机 CAD 到位后一并补模型。

4. **GUI 轨迹编辑器** — *Must*
   - `rw.py` 离线编程标签当前是只读摘要，加：路点增删拖、目标位姿手填、运动类型切换、实时碰撞/节拍着色、一键导出 `.urp/.src` 等。
   - 复用 `program_panel.py` 现有骨架。

### v0.3 — "工艺级"与"CAD/CAM 集成"（中期）

让软件从"能出码"到"能直接干活"。

5. **动力学 / 力矩校验（Dynamics）** — *Should*
   - 用 `jacobian` 做逆动力学（Newton-Euler 或解析），校验每段力矩是否超额定。
   - 输出力矩/功率报表，超额定段在 GUI 标红。
   - 需要各型号质量/惯量参数（真机 CAD 接入后可从网格估算质心，或手册录入）。

6. **CAD/CAM 轨迹生成** — *Should*
   - STEP/IGES 导入 → 提取焊缝/边/面 → 生成 moveL/moveC 路径（焊接、打磨、喷涂、切割）。
   - 与 `tools/build_vendor_urdf.py` 的 OCP 通道复用（已有 STEP 读取能力）。
   - 点云（PLY/PCAP）导入做扫描与偏差比对（数字孪生基础）。

7. **IO 与工艺参数（Process）** — *Should*
   - 在 Program 里加 `setDO / waitDI / 起弧/熄弧/喷涂开` 等工艺指令。
   - 后处理输出对应品牌 IO 语法（UR `set_digital_out`、KUKA `OUT`、FANUC `DO`、…）。
   - 焊接/喷涂工艺库（电压电流、摆焊参数）做成可配模板。

8. **标定与手眼（Calibration）** — *Should*
   - 导入 TCP 标定结果（4 点法）、基坐标系标定、工具坐标系。
   - 手眼标定（eye-to-hand / eye-in-hand）求外参，对接相机触发抓取。
   - 标定结果写回 URDF / Program，仿真即用真值。

### v1.0 — "协同与生态"（长期）

9. **多机器人协同（Multi-robot）** — *Could*
   - `world/scene.py` 已支持多对象，扩展为多台 Robot 共享场景、互做碰撞避免、节拍统筹。

10. **力控与柔顺（Compliance）** — *Could*
    - 基于动力学做阻抗/导纳规划，输出力控指令（UR `force_mode`、KUKA `COMPLIANCE`）。

11. **云协作与数字孪生** — *Could*
    - 仿真结果上传云端、实时回显真机状态（Link 的 `ControllerLink` 预留接口落地）。
    - 真机轨迹 vs 仿真轨迹偏差可视化（复用 DebugSession.deviation）。

12. **插件生态** — *Could*
    - 后处理、运动学、渲染器做成插件接口（`post/base.py` 已是好模板），第三方可加品牌/算法。

---

## 3. 待办清单（可勾选）

- [x] 抓取工作站场景模板（短期目标 S1：`orolp.world.templates` + UR5e 吸盘工作站 JSON，2026-09-13）
- [x] Web 可体验部署（短期目标 S6：deploy/web 演示站，持续维护）
- [x] 抓取动画与工件随动（短期目标 S3：`orolp.world.animation` 把 `gripper()` 指令映射到轨迹时刻，回放越过事件即吸附/释放工件；吸附瞬间保持世界位姿、随后随 TCP 随动，释放原地留下；程序面板按示教点语义自动插入 gripper 指令，回放动画与出码共用一份时序，2026-09-13）
- [x] GUI 一键展示（短期目标 S4：rw.py「抓取工作站演示」一键入口——构建模板场景载入视口、IK 生成五点式示例示教点（`orolp.world.demo`，无头可测）灌入示教/离线编程面板、自动校验出节拍并自动回放；布局经可达性重校准：输送带 h=200@y=-400、工件 X∈[60,390]、托盘 (350,450)——原布局（y=-600/皮带面 768mm）超出 UR5e 模型工作空间、IK 全局无解，2026-09-13）
- [x] 展示物料（短期目标 S5：`examples/grasping_workstation_demo.py` 无 GUI 一键全流程——模板 → IK 示教点 → 夹爪 IO → 节拍 → UR/KUKA/ABB/FANUC 出码；`tools/capture_grasp_screenshots.py` 抓取 3 张 GUI 截图入 docs/screenshots/；README「Key Features Demo」新增抓取工作站小节，2026-09-13）
- [ ] v0.2.1 碰撞检测模块 `collision.py` + 测试
- [ ] v0.2.2 速度/加速度 S 曲线规划 + 节拍报表
- [ ] v0.2.3 5 品牌真机 CAD（`assets/vendor/<brand>/step/` 待用户投放）
- [ ] v0.2.4 GUI 轨迹编辑器（路点增删拖 / 导出）
- [ ] v0.3.1 动力学力矩校验
- [ ] v0.3.2 STEP/CAM 轨迹生成
- [ ] v0.3.3 IO 与焊接/喷涂工艺参数（2026-09-13 展示子集已完成：`setDO`/`waitDI`/`waitTime`/`gripper` 链式指令 + UR/KUKA/ABB/FANUC 品牌 IO 出码；剩余：焊接/喷涂工艺模板库、waitDI 完整语义）
- [ ] v0.3.4 标定与手眼外参
- [ ] v1.0.1 多机器人协同
- [ ] v1.0.2 力控柔顺
- [ ] v1.0.3 云数字孪生
- [ ] v1.0.4 插件接口

---

## 4. 风险与依赖

| 风险 | 影响 | 缓解 |
|---|---|---|
| 5 品牌官方 STEP 未到位 | 真机 CAD 接入卡住 | 已建接收目录 + 工具就绪，等用户投放即跑 |
| 动力学需质量/惯量参数 | v0.3 力矩校验受限 | 真机 CAD 估算质心 + 手册录入双轨 |
| GUI 渲染依赖 Xvfb（无桌面环境） | 沙箱预览需虚拟显示 | noVNC 已打通；本机直跑无需 |
| 后处理覆盖 7 品牌但语法细节待真机校验 | 出码需现场微调 | 每品牌加真机导出回读测试 |

---

*维护者注：本规划随 `main` 演进更新；新特性落地后从"Should/Could"移入 `README.md` 开发进展表。*
