# 自动开发循环执行记录

## 2026-09-13（本轮）
- 任务：短期目标 S1 抓取工作站场景模板 ✅ + S6 Web 演示站首版上线 ✅
- 产出：`python/orolp/world/templates.py`（模板框架 + build_grasping_workstation）、组件库新增 material_bin/pallet_base、`examples/assets/grasping_workstation.json`、`tests/test_templates.py`（12 测试）、`tools/build_web_site.py` + `deploy/web/` 演示站
- 测试：219 passed, 2 skipped（无回归；基线由 137 更新为 219，队列旧数字过时）
- 发布：https://f1b8fd6f91ea40d7b56c733d46aea300.app.workbuddy.host （deploy/web，重复发布保持链接）
- 记账：BUILD_QUEUE.md（S1、S6 勾选）、ROADMAP.md 第 3 节、README 进展表
- 通知：lark-cli bot 身份 → ou_3df5c3d18dc2ce469b7c79de05d56ff1（用户身份 token 已过期，走 bot 直发 open_id）
- 环境：pytest 依赖装在 /Users/jayhan/.workbuddy/binaries/python/envs/default（pytest/numpy/scipy/jinja2/pyyaml/pydantic；清华源 403 需 -i https://pypi.org/simple）
- 下轮：S2 工艺 IO 指令（setDO/waitDI/gripper + 品牌后处理 + test_io_instructions.py）
- 坑：ABB 后处理名是 abb_irc5 不是 abb_rapid；`python -m orolp.world.templates` 需 PYTHONPATH=python

## 2026-09-13（第二轮）
- 任务：S2 工艺 IO 指令 ✅
- 产出：`program.py` 加 Program/ProgramBuilder 链式 `setDO`/`waitDI`/`waitTime`/`gripper("close"/"open")`（信号互锁 + clamped 反馈）；`post/fanuc_tp.py` 补品牌 IO 语法（DO[n]=ON / WAIT DI[n] / WAIT n(sec)）；修复 `then(Program)` 丢 IO 缺陷；pick_and_place.py 占位注释 → 真实 .gripper()；新增 `tests/test_io_instructions.py`（14 测试）；`tools/build_web_site.py` 演示程序加 IO + FANUC 出码样例 + IO 指令数指标
- 测试：233 passed + 2 skipped（基线 219 → 233，无回归）
- 发布：演示站同链接重新发布 https://f1b8fd6f91ea40d7b56c733d46aea300.app.workbuddy.host
- 记账：BUILD_QUEUE.md（S2 勾选）、ROADMAP.md（v0.3.3 标注展示子集完成）、README（新增 Process IO 行、后处理名修正、233 tests）
- 通知：lark-cli bot → ou_3df5c3d18dc2ce469b7c79de05d56ff1 发送成功（ok:true，message_id om_x100b6556ae6cc4a4c2bc7e995b64dc4）
- 坑：FANUC 后处理名是 fanuc_r30ib 不是 fanuc_tp（示例脚本里也错了，已修）
- 下轮：S3 抓取动画与工件随动（world/panel.py 回放 + rw.py 视口）

## 2026-09-13（第三轮）
- 任务：S3 抓取动画与工件随动 ✅
- 产出：新增 `python/orolp/world/animation.py`（`gripper_events()` 把 Program 里的夹爪 IO 映射到轨迹时刻＝上一条运动边界；`GraspAnimator` 无头驱动 Scene.attach/detach，吸附 keep_world→随 TCP 随动、释放原地留下）；`planner.py` 修复规划失败指令不占 `_motion_spans` 区间的缺陷（索引错位会让事件时刻前移）；`program_panel.py` 校验后保留规划器、`set_taught_points` 接收点类型、build_program 在 PICK/PLACE 自动插 `gripper()`、回放发 `gripperStateChanged(str)` 信号（""/"close"/"open"，去重，终点事件也发）；`pickplace.py` 新增 `gripper_action_for()`；`rw.py` 接信号 → 工件吸附/释放 + 场景树 + 状态栏 IO + 视口刷新
- 测试：251 passed + 2 skipped（新增 tests/test_grasp_animation.py 14 项；基线 233 → 251，无回归）
- 发布：演示站加 S3 能力小节，同链接重新发布 https://f1b8fd6f91ea40d7b56c733d46aea300.app.workbuddy.host
- 记账：BUILD_QUEUE.md（S3 勾选 + 基线更新 251）、ROADMAP.md 第 3 节新增 S3 行、README（Grasp animation 行、251 badge/计数）
- 通知：lark-cli bot → ou_...56ff1 发送成功（ok:true，message_id om_x100b6550a3b330a8c179d0dafbb18fc）
- 坑：①pytest 直接跑会误收集 examples/test_opengl.py（内部 sys.exit），必须 `pytest tests python/tests`；②lark-cli --content 需单行 JSON，多行文本用 `--text` 单引号最稳；③测试里 moveJ 目标=零位与起点相同 → 零长运动 → 事件时刻 t=0，写测试时第一条运动要非零
- 下轮：S4 GUI 一键展示（rw.py「抓取工作站」一键加载入口 + 自动载示例程序 + 节拍显示）

## 2026-09-13（第四轮·里程碑）
- 任务：S5 展示物料 ✅ → 短期目标 S1–S6 全部完成，触发里程碑打包
- 产出：`examples/grasping_workstation_demo.py`（无 GUI 全流程：模板→IK 示教点→自动插夹爪 IO→校验+节拍 6.33s→4 品牌出码，与 GUI 同 `demo_pick_place_points` 通路）；`tests/test_demo_script.py`（4 测试，子进程真实运行示例）；`tools/capture_grasp_screenshots.py` + docs/screenshots/ 3 张新截图（真实 Cocoa 会话 widget.grab()）；README「Grasping workstation」小节 + 徽章 267
- 测试：267 passed + 2 skipped（基线 263 → 267，无回归）
- 发布：演示站同链接重新发布 https://f1b8fd6f91ea40d7b56c733d46aea300.app.workbuddy.host（新增抓取工作站截图小节 + 无头示例命令行）
- 里程碑打包：rw.spec 修 PyInstaller 6 兼容（删 win_no_prefer_redirects/a.zipped_data；onefile 用 EXE(pyz,scripts,binaries,datas)）+ **补回被误排除的 scipy**（kinematics.py 顶层 import，旧 spec 会让打包 app 运动学全挂）+ hiddenimports 加 jinja2；产出 dist/RobotWorld.app（66MB）冒烟 10s 存活；zip=dist/RobotWorld-macOS-20260913.zip（64MB）
- 阻塞：百度网盘上传——连接器本会话未连接（connector-status 只有 agent-mail）且其能力不含本地文件上传，已标注队列「里程碑打包记录」；通知里已给用户替代选项（微信/飞书文件直发）
- 通知：lark-cli bot → ou_...56ff1 ok:true（om_x100b655e4f7b54a0dee90e34c9eede6），已带「抓取工作站能力展示已就绪」标注
- 坑：①PyInstaller `--clean` 会触发 safe-delete 批量删除门禁（删 pyinstaller 缓存 127 文件被拦），去掉 --clean 即可；②venv 缺 PyOpenGL/PyInstaller，清华源 403 用 -i https://pypi.org/simple 装；③lark-cli --markdown 对多行富文本很稳（比 --text 更适合通知）
- 下轮：主队列 v0.2.1 碰撞检测模块（python/orolp/collision.py + tests/test_collision.py，包围体层次 + 连杆-环境/连杆-连杆）
