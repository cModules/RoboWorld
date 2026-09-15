# .ide/Dockerfile
# RobotWorld 云端开发容器（Cloud Studio）—— 编码 + 网页试运行 GUI
# 基础保留 node:20，兼容 code-server / CodeBuddy Code CLI / 插件生态；
# 其上叠加 Python3.11、RobotWorld 运行/测试/GUI 依赖，以及 xvfb + x11vnc + websockify + noVNC，
# 让 app.py 跑在虚拟桌面并经 noVNC 投到浏览器，方便网站浏览测试。

FROM node:20

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONPATH=/app/python \
    LANG=C.UTF-8 \
    LANGUAGE=C.UTF-8

WORKDIR /app

# 系统依赖：Python 运行时 + 编译工具 + Git/SSH + Qt xcb 运行库（PySide6 GUI）
# + 虚拟显示(xvfb) + 远程桌面投屏(x11vnc / websockify / novnc)
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip python3-venv build-essential \
        git openssh-server wget unzip curl \
        libgl1 libegl1 libglib2.0-0 libxkbcommon0 libdbus-1-3 libfontconfig1 \
        libxrender1 libx11-xcb1 libxcb-xinerama0 libxcb-cursor0 libxcb-icccm4 \
        libxcb-keysyms1 libxcb-image0 libxcb-shm0 libxcb-randr0 \
        libxcb-render-util0 libxcb-xkb1 libxkbcommon-x11-0 \
        xvfb x11vnc websockify novnc \
    && rm -rf /var/lib/apt/lists/*

# 安装 code-server（浏览器版 VS Code）及常用插件
# 注：code-server 二进制托管在 GitHub Releases，部分构建环境网络慢/不可达；
# 本步骤设为「尽力而为」——失败不影响 noVNC GUI 试运行栈（IDE 由平台 vscode 服务提供）。
RUN set +e; \
    ARCH=$(dpkg --print-architecture 2>/dev/null || echo amd64); \
    VER=4.137.0; \
    if [ "$ARCH" = "arm64" ]; then DEB=arm64; else DEB=amd64; fi; \
    curl -fsSL --max-time 120 https://code-server.dev/install.sh | sh; \
    if ! command -v code-server >/dev/null 2>&1; then \
      echo "[fallback] install.sh 未就绪，直接拉取 code-server .deb (${DEB})"; \
      curl -fsSL --max-time 180 -o /tmp/code-server.deb \
        "https://github.com/coder/code-server/releases/download/v${VER}/code-server_${VER}_${DEB}.deb" \
        && dpkg -i /tmp/code-server.deb \
        && apt-get install -f -y --no-install-recommends; \
    fi; \
    if command -v code-server >/dev/null 2>&1; then \
      code-server --install-extension cnbcool.cnb-welcome || true; \
      code-server --install-extension redhat.vscode-yaml || true; \
      code-server --install-extension dbaeumer.vscode-eslint || true; \
      code-server --install-extension mhutchie.git-graph || true; \
      echo "code-server 安装完成"; \
    else \
      echo "code-server 未能安装（GitHub Releases 不可达），跳过——不影响 GUI 试运行"; \
    fi; \
    true

# 安装 CodeBuddy Code（提供 codebuddy 命令，用于浏览器中使用 CodeBuddy Web）
# 需确保版本 >= 2.137.0，详见「CodeBuddy Web」文档；npm 不可达时跳过（可选）
RUN npm install -g @tencent-ai/codebuddy-code || echo "codebuddy-code 安装跳过（可选）"

# Debian/bookworm 的 Python 为「外部管理」(PEP 668)，需显式放行系统级 pip 安装
# PIP_NO_BUILD_ISOLATION=1 避免 pip 建临时构建 venv（省内存）；PIP_PREFER_BINARY=1 强制用 wheel
# PIP_INDEX_URL 走国内镜像，规避 pypi.org 在构建机上的 ReadTimeout / 连接中断；
# 超时与重试放大，避免网络抖动触发依赖回溯（ResolutionImpossible）
ENV PIP_BREAK_SYSTEM_PACKAGES=1 \
    PIP_NO_BUILD_ISOLATION=1 \
    PIP_PREFER_BINARY=1 \
    PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple \
    PIP_TIMEOUT=120 \
    PIP_RETRIES=10

# 先装轻量核心依赖（与 GUI 库拆层，降低单步内存峰值，避免一处失败重装全部）
# 版本钉死到 aarch64 wheel 齐全、体积适中的稳定版，避免拉取 numpy2.x / scipy1.17 等大 wheel
# 与依赖回溯；pydantic 钉到 2.10.6（pydantic-core 2.27.2，manylinux_2_17 aarch64 可用）
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir \
        "numpy==1.26.4" "scipy==1.13.1" "jinja2==3.1.4" "pyyaml==6.0.2" \
        "pydantic==2.10.6" "pytest==8.3.4"

# GUI 依赖：用 PySide6-Essentials 替代完整 PySide6
# Essentials 仅含 QtCore/QtGui/QtWidgets/QtOpenGLWidgets，项目未用 3D/Charts 等 Addons，
# 体积与解压内存峰值远小于完整元包，可缓解 Cloud Studio 等内存受限构建机的 OOM
RUN python3 -m pip install --no-cache-dir "PySide6-Essentials>=6.4,<6.9" "PyOpenGL"

# 复制仓库源码（开发容器里直接可编辑 / 试运行）
COPY . .

# 确保启动脚本可执行
RUN chmod +x /app/.ide/start.sh

# 默认启动 noVNC 试运行栈（Xvfb → app.py → x11vnc → websockify/noVNC）
# Cloud Studio 的 code-server IDE 由平台另行拉起；此处 launch 提供 GUI 网页预览
CMD ["bash", "/app/.ide/start.sh"]
