# 抓取 UR / KUKA / ABB / FANUC / Estun 真机 CAD 的两种方案

JAKA 已用官方公开接口抓到 7 个型号并接进 URDF（commit `0fd574d`）。
其余 5 个品牌的真机 3D 模型社区都**需要登录**才能下载。本文给两条路：

- **方案 A**：本机装 BrowserSkill（`bsk`） + 浏览器扩展，让我去驱动你已登录的浏览器下载。
- **方案 B（推荐）**：你自己登录下载 STEP，丢进约定目录，我跑已验证的流水线生成真机 URDF。

> ⚠️ 先读「重要限制」一节——方案 A 在当前部署下可能连不通，方案 B 永远可用。

---

## 重要限制（先读）

`bsk` 是**本地桥接**：架构是 `Agent → bsk CLI → bsk daemon(127.0.0.1) → 浏览器扩展 → Agent Window`。
daemon 监听本机回环地址，要求 **浏览器与 Agent 运行时在同一台机器上**。

当前对话里我运行在**远程沙箱（Ubuntu 容器）**，与你本机不在同一台机器。
因此即使你本机装好 `bsk`，**我这边也连不到你本机的 daemon**。

要让方案 A 真正可用，需要：
- 你用**本机 WorkBuddy 桌面端**（Agent 在本地运行），并在那条会话里发起抓取任务；或
- 把 Agent 运行时切到与你浏览器同机的环境。

方案 B 不依赖这个，任何部署下都能用，且与 JAKA 已验证的流水线完全复用。

---

## 方案 A：本机安装 BrowserSkill（bsk）+ 扩展

> 来源：<https://github.com/Tencent/BrowserSkill>，官方 README / `AGENT_INSTALL.md`。

### 架构
```
你的 Agent 运行时 ──shell: bsk ...──▶ bsk CLI ──本地 IPC──▶ bsk daemon
                                                     │
                                            (127.0.0.1 WebSocket)
                                                     ▼
                                          浏览器扩展(BrowserSkill)
                                                     ▼
                                            Agent Window(独立窗口)
```
整条链路都在本机，不动你的常规浏览器窗口（除非显式 `bsk tab borrow`）。

### 第 1 步：装 bsk CLI

**macOS / Linux（推荐，装到 `~/.local/bin`）：**
```bash
curl -fsSL https://raw.githubusercontent.com/Tencent/BrowserSkill/main/install.sh | sh
bsk --version          # 应输出类似 bsk 0.1.x
```

**Windows（PowerShell）：**
```powershell
irm https://raw.githubusercontent.com/Tencent/BrowserSkill/main/install.ps1 | iex
bsk --version
```
> 国内 GitHub 不稳时改手动：到 <https://github.com/Tencent/BrowserSkill/releases/latest>
> 下载 `bsk-v*-x86_64-pc-windows-msvc.zip`，解压 `bsk.exe` 到 `C:\Users\<你>\.local\bin\` 并加入 PATH。

### 第 2 步：装浏览器扩展（关键）
装进你**日常使用、且已登录目标站点**的那个浏览器——复用登录态就靠这个前提。

- **Chrome**：Chrome Web Store 搜 `BrowserSkill` 安装。
- **Edge**：Edge 加载项商店搜 `BrowserSkill` 安装（国内可直连，无需翻墙）。
- 商店搜不到时：GitHub 下载扩展 ZIP，开 `chrome://extensions` 开发者模式，解压后拖入。

安装后点扩展图标，确认弹出面板显示**绿色连接状态**。

### 第 3 步：装 skill（教 Agent 怎么用 bsk）
```bash
bsk install-skill        # 空格选中 WorkBuddy，回车自动配置
```
若框架不在列表：手动把仓库 `skill/SKILL.md` 复制到 harness 的 skills 目录 `browser-skill/SKILL.md`。

### 验证
```bash
bsk doctor               # 深度诊断 + 修复提示
bsk status               # 连接健康、已连浏览器、活跃会话
bsk browsers             # 列出已连浏览器实例，拿 instance id
```

### 使用会话生命周期（每次任务必守）
```bash
bsk session start                 # 打印 4 字母 session id
# …… 所有命令都带 --session <id> ……
bsk session stop <id>             # 任务结束必须调用，关闭 Agent Window
```
可选：`bsk session start --no-focus`（后台开窗口，不抢焦点）；
遇登录 / 验证码 / OTP 等人肉步骤：`bsk request-help --session <id> --prompt "..."`。

---

## 方案 B：手动下载（推荐，最稳）

### 流程
1. 你在自己浏览器登录各品牌 3D 模型社区。
2. 找到对应机器人型号的 **STEP**（首选）/ STL / IGES 下载包。
3. 丢进仓库约定目录（见下）。
4. 告诉我「XX 品牌文件已就位」，我跑 STEP→URDF 流水线，复用 JAKA 已验证逻辑。

### 目录约定
```
RoboWorld/assets/vendor/<brand>/step/<model>.STEP
```
例：
```
assets/vendor/ur/step/ur5.STEP
assets/vendor/kuka/step/kr6_r900.STEP
assets/vendor/abb/step/irb120.STEP
assets/vendor/fanuc/step/lr_mate_200id.STEP
assets/vendor/estun/step/er3.STEP
```

### 我需要你提供的文件
| 项目 | 要求 |
|---|---|
| 格式 | **STEP (.STEP/.step) 优先**；STL/IGES 也可 |
| 内容 | 整机装配（含各连杆），不要只给单个零件 |
| 单位 | 原厂单位即可（流水线自动识别 mm→m） |
| 命名 | `<model>.STEP` 即可，型号数字清晰（如 `ur5`、`kr6_r900`） |

> 收到后我会把 `tools/build_vendor_urdf.py` 从 JAKA 硬编码**泛化为多品牌**，
> 复用「顶层组件→连杆局部 STL + 复用程序化 URDF 运动学 + 关节原点按质心最近顶点重算」的整套逻辑。
> JAKA 那套已验证：零位精确还原 CAD 出厂姿态、关节绕真实连接中心旋转。

### 各品牌登录入口（需账号，子路径我可在你登录后进一步核实）
| 品牌 | 社区 / 下载入口 | 备注 |
|---|---|---|
| UR | universal-robots.com → Download / myUR | UR+ 开发者社区有 3D 模型 |
| KUKA | kuka.com → KUKA Xpert / 客户门户 | 3D CAD 经 KUKA Marketplace 或 my.kuka |
| ABB | new.abb.com/products/robotics → myABB | 3D CAD 模型门户 |
| FANUC | fanuc.eu / fanuc.com → CAD 门户 / ROBOGUIDE | 需 FANUC 账号 |
| Estun | estun.com → 合作伙伴门户 | 国内站，中文资料多 |

> 注：以上为各品牌官方主站 / 门户，具体下载子路径需登录后确认；
> 你给我账号入口或登录后截图，我可把精确点击路径补进本文。

---

## 我拿到 STEP 之后做什么
1. 泛化 `build_vendor_urdf.py` 支持该品牌（坐标换轴 + 缩放 + 组件分组）。
2. 若该品牌已有程序化 URDF，直接复用其运动学；没有则新建。
3. 生成 `<model>_real.urdf` + `meshes_real/*.stl`，渲染零位预览图核对外观。
4. 跑 `tools/check_no_secrets.py --staged` 确认无凭据泄漏后提交。

## 建议
- 想要「我自动驱动浏览器」→ 走方案 A，但需确保 Agent 与本机同机（见重要限制）。
- 想要「今天就能推进」→ 走方案 B，你把 STEP 丢进目录即可，零额外环境依赖。
