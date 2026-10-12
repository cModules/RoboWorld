#!/usr/bin/env bash
# RoboWorld Linux 安装脚本：装可执行文件 + 桌面条目 + 工程文件 MIME 关联
#
# 用法：
#   ./packaging/linux/install.sh                    # 装到 ~/.local（无需 root）
#   sudo ./packaging/linux/install.sh --prefix /usr # 装到系统目录
#   ./packaging/linux/install.sh --uninstall
#
# 为什么不是"拷个 exe 就行"：Linux 没有注册表，文件关联靠 XDG 三处协作——
#   1. share/mime/packages/*.xml  —— 声明"*.rwproj 是 application/x-roboworld-project"；
#   2. share/applications/*.desktop + MimeType —— 声明"本程序能开这种类型"；
#   3. update-mime-database / update-desktop-database —— 让桌面环境重新编译缓存。
# 少任何一步的表现都是"双击没反应"，且不会报错。
#
# 输出保持英文：i18n.py 是 Python 应用内的双语机制，shell 脚本无法复用它；
# 面向用户的桌面条目与 MIME 定义本身已双语（见同目录的 .desktop / .xml）。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # …/packaging/linux
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
LINUX_DIR="$SCRIPT_DIR"

# XDG_DATA_HOME 指向 …/.local/share，而我们要的是它的上一级（bin 与 share 同级）。
# 不能无条件 dirname：未设置该变量时默认值本身已经是 …/.local，再取一次会跑到 $HOME。
if [ -n "${XDG_DATA_HOME:-}" ]; then
    PREFIX="$(dirname "$XDG_DATA_HOME")"
else
    PREFIX="$HOME/.local"
fi
DIST="$REPO_ROOT/dist"
UNINSTALL=0

while [ $# -gt 0 ]; do
    case "$1" in
        --prefix)    PREFIX="$2"; shift 2 ;;
        --dist)      DIST="$2"; shift 2 ;;
        --uninstall) UNINSTALL=1; shift ;;
        -h|--help)   sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

BIN_DIR="$PREFIX/bin"
DATA_DIR="$PREFIX/share"
APP_DIR="$DATA_DIR/applications"
MIME_DIR="$DATA_DIR/mime/packages"
ICON_DIR="$DATA_DIR/icons/hicolor/256x256/apps"

refresh_databases() {
    # 三个工具都只在装了对应桌面栈的机器上存在；不存在就跳过（服务器版 Linux
    # 没有 GTK，硬跑只会报错——那时"文件关联"本来也没有意义）。
    command -v update-desktop-database >/dev/null && update-desktop-database "$APP_DIR" 2>/dev/null || true
    command -v update-mime-database    >/dev/null && update-mime-database "$DATA_DIR/mime" 2>/dev/null || true
    command -v gtk-update-icon-cache   >/dev/null && gtk-update-icon-cache -f -t "$DATA_DIR/icons/hicolor" 2>/dev/null || true
}

if [ "$UNINSTALL" -eq 1 ]; then
    echo "==> Uninstalling from $PREFIX"
    rm -f "$BIN_DIR/roboworld"
    rm -f "$APP_DIR/roboworld.desktop"
    rm -f "$MIME_DIR/roboworld-project.xml"
    rm -f "$ICON_DIR/roboworld.png"
    refresh_databases
    echo "==> Removed. Project files (.rwproj) will fall back to 'no application'."
    exit 0
fi

# --- 前置校验 ---------------------------------------------------------------- #
# PyInstaller 在 Linux 上的产物没有扩展名（spec 里 name='RoboWorld'）
EXE="$DIST/RoboWorld"
if [ ! -f "$EXE" ]; then
    echo "error: $EXE not found." >&2
    echo "build it first:  python -m PyInstaller -y packaging/app.spec" >&2
    exit 1
fi

if ! mkdir -p "$BIN_DIR" 2>/dev/null; then
    echo "error: cannot write to $BIN_DIR — try: sudo $0 --prefix /usr" >&2
    exit 1
fi

echo "==> Installing RoboWorld to $PREFIX"

install -d "$BIN_DIR" "$APP_DIR" "$MIME_DIR" "$ICON_DIR"
install -m 755 "$EXE" "$BIN_DIR/roboworld"
install -m 644 "$LINUX_DIR/roboworld.desktop" "$APP_DIR/roboworld.desktop"
install -m 644 "$LINUX_DIR/roboworld-project.xml" "$MIME_DIR/roboworld-project.xml"
# 图标目录按 256x256 归档：若 packaging/logo.png 换了尺寸，这一行要跟着改，
# 否则部分桌面环境（按尺寸精确匹配的）会找不到图标。
install -m 644 "$REPO_ROOT/packaging/logo.png" "$ICON_DIR/roboworld.png"

refresh_databases

echo ""
echo "==> Installed:"
echo "    executable : $BIN_DIR/roboworld"
echo "    desktop    : $APP_DIR/roboworld.desktop"
echo "    mime       : $MIME_DIR/roboworld-project.xml"

case ":$PATH:" in
    *":$BIN_DIR:"*) ;;
    *) echo ""
       echo "    note: $BIN_DIR is not in PATH — desktop launchers still work,"
       echo "          but 'roboworld' won't be found in a terminal." ;;
esac

echo ""
echo "==> Verify:"
echo "    xdg-mime query filetype /path/to/cell.rwproj"
echo "        expected: application/x-roboworld-project"
echo "    xdg-mime query default application/x-roboworld-project"
echo "        expected: roboworld.desktop"
