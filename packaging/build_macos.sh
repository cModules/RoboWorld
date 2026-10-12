#!/usr/bin/env bash
# RoboWorld macOS 打包脚本：产出 dist/RoboWorld.app 与分发用的 .dmg
#
# 用法（仓库任意位置都行，脚本自己往上找仓库根）：
#   ./packaging/build_macos.sh
#
# 与 build_win64.ps1 的两点不同（都是踩过的坑，别"统一"掉）：
#   1. **不加 --clean**：批量删 PyInstaller 缓存会触发沙箱删除门禁；
#   2. 打完要**签名**：未签名的 .app 在装有 Gatekeeper 的机器上会被判为
#      "已损坏/无法打开"（本机 ad-hoc 签名可解，分发需 Developer ID + notarize）。
#
# 文件关联（双击 .rwproj 打开）由 packaging/app.spec 的 Info.plist 声明，
# 本脚本末尾会校验它确实打进了包里——缺了它应用照常能开，只是双击不关联。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # …/packaging
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DIST="$REPO_ROOT/dist"
APP="$DIST/RoboWorld.app"

# venv 优先：系统 Python 上没装 PyInstaller 时，这里直接失败比打到一半失败好
if [ -x "$REPO_ROOT/.venv/bin/python" ]; then
    PYTHON="$REPO_ROOT/.venv/bin/python"
else
    PYTHON="python3"
fi

VERSION="$("$PYTHON" -c 'from support import __version__; print(__version__)')"
ARCH="$(uname -m)"
DMG="$DIST/RoboWorld-macOS-${VERSION}-${ARCH}.dmg"

echo "========================================"
echo " RoboWorld macOS 打包"
echo "========================================"
echo "版本: $VERSION    架构: $ARCH"
echo ""

# 1. 打包
echo "==> PyInstaller 打包（约几分钟）..."
(cd "$REPO_ROOT" && "$PYTHON" -m PyInstaller -y packaging/app.spec)

if [ ! -d "$APP" ]; then
    echo "错误：未生成 $APP" >&2
    exit 1
fi

# 2. 签名
# ad-hoc（-）只保证"本机不被 Gatekeeper 拦"，不能用于对外分发：
# 别人下载后仍会被拦。正式分发把下一行换成
#   codesign --force --deep --sign "Developer ID Application: <Team> (<ID>)" "$APP"
# 再跑 notarytool submit / xcrun stapler staple。
echo "==> ad-hoc 签名..."
if ! codesign --force --deep --sign - "$APP"; then
    echo "警告：签名失败。本机首次打开可能需在『系统设置 → 隐私与安全性』点『仍要打开』。" >&2
fi

# 3. 校验文件关联确实打进了 Info.plist
#    这一步是必须的：spec 写错时应用照样能启动，只有双击工程文件才发现没关联，
#    而打包机上通常没人去双击。
echo "==> 校验工程文件关联..."
if plutil -extract CFBundleDocumentTypes json -o - "$APP/Contents/Info.plist" >/dev/null 2>&1; then
    echo "    CFBundleDocumentTypes: 已声明"
else
    echo "警告：Info.plist 缺少 CFBundleDocumentTypes —— 双击 .rwproj 不会关联本应用" >&2
fi

# 4. 生成 dmg
#    用 hdiutil 而不是 create-dmg：后者要额外装（brew），且它的排版依赖
#    AppleScript 的 Finder 权限，在 CI/ssh 会话里会失败。这里只做"能装"的最小
#    形态：把 .app 与 /Applications 的软链接放进一个只读镜像，用户拖一下即可。
echo "==> 生成 dmg..."
STAGING="$DIST/dmg"
rm -rf "$STAGING"
mkdir -p "$STAGING"
cp -R "$APP" "$STAGING/"
ln -s /Applications "$STAGING/Applications"
hdiutil create -volname "RoboWorld" -srcfolder "$STAGING" -ov -format UDZO "$DMG" >/dev/null
rm -rf "$STAGING"

echo ""
echo "========================================"
echo " 打包完成"
echo "========================================"
echo "应用: $APP"
echo "镜像: $DMG"
echo ""
echo "验证清单："
echo "  1. 把 RoboWorld 拖进 Applications，启动一次（让 LaunchServices 登记 UTI）；"
echo "  2. 在 Finder 里右键一个 .rwproj → 『打开方式』应出现 RoboWorld；"
echo "  3. 终端跑：mdls -name kMDItemContentType <工程文件>，应含 robotworld.project"
