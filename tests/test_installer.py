"""安装器 / 文件关联配置的静态回归测试。

背景（与 ``test_packaging_spec.py`` 同一条纪律）：**打包与安装配置写错，不会让
任何功能测试变红**——应用照常能启动，只是"双击工程文件没反应""装完关联不上"，
而这类问题只会在用户机器上暴露。所以这里对三端的安装配置做静态约束：

* Windows：``packaging/windows/setup.iss``（Inno Setup）
* macOS：``packaging/app.spec`` 的 Info.plist（文件关联由它声明）
* Linux：``packaging/linux/roboworld.desktop`` + ``roboworld-project.xml``

守的核心是一件事：**工程扩展名、应用身份、类型名在三端各写了一份**，任何一处
与 :mod:`service.project` / :mod:`app.app_identity` 漂移，症状都是同一种——
"菜单里能存能开，双击却没反应"。
"""

from __future__ import annotations

import ast
import configparser
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

PACKAGING = REPO_ROOT / "packaging"
SPEC = PACKAGING / "app.spec"
ISS = PACKAGING / "windows" / "setup.iss"
DESKTOP = PACKAGING / "linux" / "roboworld.desktop"
MIME_XML = PACKAGING / "linux" / "roboworld-project.xml"
INSTALL_SH = PACKAGING / "linux" / "install.sh"
BUILD_MAC_SH = PACKAGING / "build_macos.sh"
BUILD_INSTALLER_PS1 = PACKAGING / "build_installer_win.ps1"

_MIME_NS = "{http://www.freedesktop.org/standards/shared-mime-info}"


def _text(path: Path) -> str:
    assert path.is_file(), f"缺少 {path}"
    return path.read_text(encoding="utf-8")


def _iss_defines(text: str) -> dict:
    """取出 ``#define Name "value"``（ISPP 的宏定义）。"""
    out = {}
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped.startswith("#define"):
            continue
        parts = stripped.split(None, 2)
        if len(parts) != 3:
            continue
        value = parts[2].strip().strip('"')
        out[parts[1]] = value
    return out


def _iss_sections(text: str) -> dict:
    """按 ``[Section]`` 切分 iss（安装器脚本没有 ini 那样的键值规范，按行处理）。"""
    sections: dict = {}
    current = None
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("[") and stripped.endswith("]"):
            current = stripped[1:-1]
            sections.setdefault(current, [])
        elif current is not None:
            sections[current].append(line)
    return sections


# --------------------------------------------------------------------------- #
# Windows（Inno Setup）
# --------------------------------------------------------------------------- #


def test_iss_exists():
    assert ISS.is_file(), "缺 Windows 安装器脚本：Windows 上双击工程文件无法关联"


def test_iss_identity_matches_app_identity():
    """iss 里的 AppId / ProgID / 类型名必须与 app.app_identity 同源。

    三份身份分家的后果：任务栏把窗口分成两组、Jump List 的"最近"挂到别的条目、
    卸载程序认不出旧版本（于是装成两份）。
    """
    from app.app_identity import APPLICATION_ID, PROJECT_PROG_ID, PROJECT_TYPE_NAME

    defines = _iss_defines(_text(ISS))
    assert defines.get("MyAppId") == APPLICATION_ID, "iss 的 MyAppId 与 APPLICATION_ID 不一致"
    assert defines.get("ProjectProgId") == PROJECT_PROG_ID
    assert defines.get("ProjectType") == PROJECT_TYPE_NAME


def test_iss_registers_every_project_extension():
    """三个扩展名（含历史别名 .core）都必须注册，一个都不能漏。"""
    from service.project import PROJECT_EXTENSIONS

    registry = _iss_sections(_text(ISS)).get("Registry", [])
    body = "\n".join(registry)
    for ext in PROJECT_EXTENSIONS:
        assert f"Software\\Classes\\{ext}" in body, f"iss 未注册 {ext}：这种文件双击不会关联"


def test_iss_uses_hka_not_hardcoded_hklm():
    """关联必须写 HKA（有管理员 → HKLM，否则 → HKCU）。

    写死 HKLM 的后果：非管理员安装时写不进去，安装**不报错**，只是装完双击
    依旧弹"你要如何打开这个文件"。
    """
    registry = _iss_sections(_text(ISS)).get("Registry", [])
    assert registry, "iss 没有 [Registry] 段"
    for line in registry:
        if not line.strip().startswith("Root:"):
            continue
        assert "HKA" in line, f"注册表项写死了根键（应为 HKA）：{line.strip()}"


def test_iss_open_command_quotes_path():
    """``%1`` 必须带引号。

    路径含空格（``C:\\Users\\A B\\…``）时不带引号会被截成两段，症状是
    "双击有的工程能开、有的打不开"——只在一个用户的机器上复现。
    """
    registry = _iss_sections(_text(ISS)).get("Registry", [])
    commands = [ln for ln in registry if "shell\\open\\command" in ln]
    assert commands, "iss 没有注册 open\\command"
    for line in commands:
        assert '""%1""' in line, f"open\\command 的 %1 未加引号：{line.strip()}"


def test_iss_declares_association_change():
    """``ChangesAssociations=yes`` 缺失时，Win8+ 不会采用新注册的文件关联。

    系统会沿用它记住的"用户上次为这个类型选了什么"，于是装完再双击仍然是
    旧软件打开——重装也没用，极难排查。
    """
    setup = "\n".join(_iss_sections(_text(ISS)).get("Setup", []))
    assert "ChangesAssociations=yes" in setup


def test_iss_offers_open_with_entry():
    """Applications 键：让本程序出现在"打开方式"列表里。

    缺它时，若 .rwproj 已被别的程序占用，用户在打开方式里根本看不到 RoboWorld。
    """
    registry = "\n".join(_iss_sections(_text(ISS)).get("Registry", []))
    assert "Software\\Classes\\Applications\\" in registry


def test_iss_has_bilingual_wizard():
    """安装界面要中英双语（规则：用户可见字符串双语）。"""
    languages = "\n".join(_iss_sections(_text(ISS)).get("Languages", []))
    assert "chinesesimplified" in languages.lower()
    assert "english" in languages.lower()


def test_installer_script_builds_from_code_version():
    """构建脚本必须从 support.__version__ 取版本号，不能在脚本里手写。"""
    text = _text(BUILD_INSTALLER_PS1)
    assert "support" in text and "__version__" in text, "版本号未从代码读取（第五处手写？）"
    assert "/DMyAppVersion" in text, "未把版本号传给 ISCC"
    assert "setup.iss" in text


def test_windows_exe_icon_is_ico():
    """Windows EXE 图标必须是 .ico，且该文件必须入库。

    PyInstaller 的 CopyIcons 只认 ico/exe；传 PNG 时需要 Pillow 现场转换——
    CI（``pip install -e \".[gui]\" pyinstaller``）并不装 Pillow，转换抛
    ValueError、整个打包以 exit 1 收场（2026-10-20 实锤）。即便装了 Pillow，
    转出的 ico 也只有单一尺寸，Explorer/任务栏缩放难看，因此入库一份多尺寸
    ``logo.ico``，spec 按平台选路径。
    """
    ico = PACKAGING / "logo.ico"
    assert ico.is_file(), "缺 packaging/logo.ico：Windows 打包时 EXE 图标无源"
    # 简单文件头校验（ico = 0x00 0x00 0x01 0x00）
    assert ico.read_bytes()[:4] == b"\x00\x00\x01\x00", "logo.ico 不是合法 ICO 文件"
    # spec 在 win32 路径使用 logo.ico（sys.platform 条件写法或硬编码均可接受）
    spec = _text(SPEC)
    assert "logo.ico" in spec, "spec 的 Windows 图标未引用 logo.ico"


# --------------------------------------------------------------------------- #
# macOS（Info.plist，由 app.spec 生成）
# --------------------------------------------------------------------------- #


def _spec_info_plist_source() -> str:
    """取出 spec 里 ``BUNDLE(info_plist=…)`` 的源码片段。

    用 AST 定位而不是正则扫全文：全文正则会在注释里命中，得到"看起来通过了"的
    假阳性。取不到片段说明 spec 结构变了（例如 BUNDLE 段被删）——那也是要红的。
    """
    text = _text(SPEC)
    tree = ast.parse(text)
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, "id", "") == "BUNDLE":
            for kw in node.keywords:
                if kw.arg == "info_plist":
                    return ast.get_source_segment(text, kw.value) or ""
    pytest.fail("spec 的 BUNDLE 段缺少 info_plist：macOS 文件关联与 Dock 名称都靠它")


def test_spec_declares_project_document_type():
    """Info.plist 必须同时声明 CFBundleDocumentTypes 与 UTI 定义。

    缺前者：LaunchServices 不知道 .rwproj 归谁，双击问"用什么打开"；
    缺后者：系统不认识这个 UTI，Finder 里文件类型显示成"文稿"。
    """
    plist = _spec_info_plist_source()
    assert "CFBundleDocumentTypes" in plist
    assert "UTExportedTypeDeclarations" in plist


def test_spec_document_type_uses_shared_constants():
    """plist 里的 UTI / 类型名 / 扩展名必须来自代码常量，不得手写字面量。"""
    from app.app_identity import PROJECT_MIME_TYPE, PROJECT_TYPE_NAME, PROJECT_UTI

    plist = _spec_info_plist_source()
    assert "PROJECT_UTI" in plist
    assert "_PROJECT_SUFFIXES" in plist, "扩展名应由 service.project 派生，不是手写"
    assert "PROJECT_TYPE_NAME" in plist
    assert "PROJECT_MIME_TYPE" in plist
    del PROJECT_MIME_TYPE, PROJECT_TYPE_NAME, PROJECT_UTI  # 仅为可读性：真源在常量里


def test_spec_has_no_hardcoded_extension_literals():
    """spec 里不许出现 ".rwproj" 这类字面量（注释不算，AST 只看到真字符串）。"""
    tree = ast.parse(_text(SPEC))
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            assert ".rwproj" not in node.value.lower(), (
                f"spec 里硬编码了扩展名字面量 {node.value!r}："
                "与 service.project.PROJECT_EXTENSIONS 漂移后双击会失效"
            )


def test_spec_suffixes_derived_from_project_extensions():
    """``_PROJECT_SUFFIXES`` 必须由 ``PROJECT_EXTENSIONS`` 派生。"""
    text = _text(SPEC)
    assert "_PROJECT_SUFFIXES = [ext.lstrip('.') for ext in PROJECT_EXTENSIONS]" in text


# --------------------------------------------------------------------------- #
# Linux（XDG）
# --------------------------------------------------------------------------- #


def test_desktop_file_declares_project_mime():
    """desktop 文件的 MimeType 与执行方式。

    ``%f`` 缺失时，双击工程文件只会启动一个空窗口——用户以为"软件坏了"。
    """
    parser = configparser.RawConfigParser()  # Raw：%f 会被普通插值器当成占位符而报错
    parser.read_string(_text(DESKTOP))
    entry = parser["Desktop Entry"]
    assert entry["Type"] == "Application"
    assert "%f" in entry["Exec"], "Exec 缺 %f：双击工程文件不会传给程序"

    from app.app_identity import PROJECT_MIME_TYPE

    assert PROJECT_MIME_TYPE in entry["MimeType"], "MimeType 与 MIME 定义对不上，关联不生效"


def test_desktop_file_is_bilingual():
    assert "Name[zh_CN]" in _text(DESKTOP)
    assert "Comment[zh_CN]" in _text(DESKTOP)


def test_mime_xml_covers_every_extension():
    """MIME 定义的 glob 必须覆盖每个工程扩展名。"""
    from app.app_identity import PROJECT_MIME_TYPE
    from service.project import PROJECT_EXTENSIONS

    root = ET.fromstring(_text(MIME_XML))
    mime = root.find(f"{_MIME_NS}mime-type")
    assert mime is not None, "MIME XML 缺 mime-type 节点"
    assert mime.get("type") == PROJECT_MIME_TYPE

    globs = {g.get("pattern") for g in mime.findall(f"{_MIME_NS}glob")}
    for ext in PROJECT_EXTENSIONS:
        assert f"*{ext}" in globs, f"MIME 未声明 {ext}：文件管理器不认这种文件"


def test_mime_xml_is_bilingual():
    assert 'xml:lang="zh_CN"' in _text(MIME_XML)


def test_install_script_wires_all_three_pieces():
    """安装脚本必须同时装 desktop、MIME 并刷新数据库——少一步就没关联。"""
    text = _text(INSTALL_SH)
    assert "roboworld.desktop" in text
    assert "roboworld-project.xml" in text
    assert "update-mime-database" in text
    assert "update-desktop-database" in text


def test_macos_build_script_verifies_association():
    """macOS 打包脚本要在产出后校验关联确实打进了 plist。"""
    text = _text(BUILD_MAC_SH)
    assert "CFBundleDocumentTypes" in text, "打包后不校验关联：spec 写错时无人发现"
    assert "codesign" in text, "未签名：Gatekeeper 会把应用判为已损坏"


# --------------------------------------------------------------------------- #
# 脚本语法（能在本机校验时才校验）
# --------------------------------------------------------------------------- #


def test_shell_scripts_parse():
    """shell 脚本至少要通过 bash 语法检查。

    没有 bash 的机器（Windows CI）跳过该断言——那里的 ps1 由 PowerShell 解析，
    语法错误会在构建步骤直接暴露，不需要这条。
    """
    import shutil

    if shutil.which("bash") is None:
        return
    for script in (INSTALL_SH, BUILD_MAC_SH):
        result = subprocess.run(
            ["bash", "-n", str(script)], capture_output=True, text=True, check=False
        )
        assert result.returncode == 0, f"{script.name} 语法错误：{result.stderr}"
