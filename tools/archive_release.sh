#!/usr/bin/env bash
# 把 CI 产出的发布包归档到 NAS 的 releases 目录。
#
# 背景：GitHub 托管的 runner 看不到本机挂载的网络卷，所以归档这一步只能在本机做
# ——CI 负责构建 + 发 GitHub Release，本脚本负责把产物放进 NAS。
#
# 用法:
#   # 1. 在 Actions 运行页下载 RoboWorld-Windows-Setup artifact，解压到 dist/
#   # 2. 指定归档根目录（必须显式给，不写死任何人的挂载路径）
#   ROBOWORLD_RELEASES_DIR=/Volumes/<共享名>/.../releases ./tools/archive_release.sh
#
#   # 也可显式指定版本与目录
#   ./tools/archive_release.sh --dest /path/to/releases --version 0.1.40
#
# 产物会按 <releases>/v<版本号>/ 归档；版本号默认从 support.__version__ 读。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${ROBOWORLD_RELEASES_DIR:-}"
VERSION=""

usage() {
    sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dest)
            DEST="$2"
            shift 2
            ;;
        --version)
            VERSION="$2"
            shift 2
            ;;
        -h | --help)
            usage
            exit 0
            ;;
        *)
            echo "未知参数：$1（用 --help 看用法）" >&2
            exit 2
            ;;
    esac
done

if [[ -z "$DEST" ]]; then
    echo "错误：未指定归档目录。" >&2
    echo " 请设置 ROBOWORLD_RELEASES_DIR，或传 --dest /path/to/releases" >&2
    exit 2
fi

if [[ -z "$VERSION" ]]; then
    VERSION="$(cd "$REPO_ROOT" && python3 -c 'from support import __version__; print(__version__)')"
fi

if [[ ! -d "$DEST" ]]; then
    echo "错误：归档目录不存在：$DEST" >&2
    echo " NAS 共享挂载了吗？用 mount | grep smbfs 确认。" >&2
    exit 1
fi

if ! touch "$DEST/.rw_probe" 2>/dev/null; then
    echo "错误：归档目录不可写：$DEST" >&2
    exit 1
fi
rm -f "$DEST/.rw_probe"

TARGET="$DEST/v$VERSION"

shopt -s nullglob
FILES=()
for pattern in \
    "$REPO_ROOT/dist/RoboWorld-Setup-$VERSION.exe" \
    "$REPO_ROOT/dist/RoboWorld-source-$VERSION.tar.gz" \
    "$REPO_ROOT/dist/RoboWorld-macOS-"*"$VERSION"*.zip; do
    for path in $pattern; do
        [[ -f "$path" ]] && FILES+=("$path")
    done
done

if [[ ${#FILES[@]} -eq 0 ]]; then
    echo "错误：dist/ 下没有 $VERSION 的发布产物。" >&2
    echo " 先到 Actions 运行页下载 RoboWorld-Windows-Setup artifact，解压到 dist/。" >&2
    exit 1
fi

mkdir -p "$TARGET"
echo "归档 ${#FILES[@]} 个文件到 $TARGET"
rsync -av --progress "${FILES[@]}" "$TARGET/"

echo "完成。校验："
ls -lh "$TARGET"
