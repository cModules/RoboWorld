# Dockerfile（仓库根目录）
# RobotWorld 交付用制成品镜像（无头 CI 验证用）
# - 真实依赖见 pyproject.toml，仓库无 requirements.txt，直接 pip 装核心依赖
# - C++ 扩展 orolp_core 为可选（__init__ 已 try/except ImportError），跳过编译以加速
# - PySide6/PyOpenGL 仅 GUI 面板用，测试与核心 API 不依赖，无头镜像不装
FROM python:3.11-slim

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONPATH=/app/python \
    PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple \
    PIP_TIMEOUT=120 \
    PIP_RETRIES=10

WORKDIR /app

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential git && \
    rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir \
        "numpy==1.26.4" "scipy==1.13.1" "jinja2==3.1.4" "pyyaml==6.0.2" \
        "pydantic==2.10.6" "pytest==8.3.4"

# .dockerignore 已排除 .git 等；models/ 会一并拷入（URDF 测试需要）
COPY . .

# CMD = 无头自检：跑仓库自带 pytest（退出码 0 即代表镜像可运行、出码链路正常）
# CI smoke test 用 `docker run --rm <image>` 直接验证
CMD ["sh", "-c", "python -m pytest tests/ -q"]
