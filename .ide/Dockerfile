# .ide/Dockerfile
# RobotWorld 云端开发容器（Cloud Studio）—— 编码 + 网页试运行 GUI
# 基础保留 node:20，兼容 code-server / CodeBuddy Code CLI / 插件生态；
# 其上叠加 Python3.11、RobotWorld 运行/测试/GUI 依赖，以及 xvfb + x11vnc + websockify + noVNC，
# 让 rw.py 跑在虚拟桌面并经 noVNC 投到浏览器，方便网站浏览测试。

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

# Python 依赖：核心运行时 + 测试 + GUI（PySide6 / PyOpenGL，供 rw.py 试运行）
# Debian/bookworm 的 Python 为「外部管理」(PEP 668)，需显式放行系统级 pip 安装
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir \
        "numpy>=1.21" "scipy>=1.7" "jinja2>=3.0" "pyyaml>=6.0" "pydantic>=2.0" \
        "pytest>=7.0" \
        "PySide6>=6.4" "PyOpenGL"

# 复制仓库源码（开发容器里直接可编辑 / 试运行）
COPY . .

# 确保启动脚本可执行
RUN chmod +x /app/.ide/start.sh

# 默认启动 noVNC 试运行栈（Xvfb → rw.py → x11vnc → websockify/noVNC）
# Cloud Studio 的 code-server IDE 由平台另行拉起；此处 launch 提供 GUI 网页预览
CMD ["bash", "/app/.ide/start.sh"]
