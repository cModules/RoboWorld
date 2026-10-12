"""Windows 外壳集成：文件关联、任务栏身份、Jump List 的"最近"文档。

只在 ``sys.platform == "win32"`` 生效；其它平台全部 no-op（函数直接返回
``False``）。这样 :mod:`app.launcher` 可以**无条件调用**，不需要在调用处散落
``if platform.system() == "Windows"``——平台分支只留这一处，Linux/macOS 的
调用方代码保持干净。

为什么用 ctypes 直连 shell32，而不是 ``PySide6.QtWinExtras``：

* QtWinExtras 在 PySide6 各版本里是否随包分发并不一致（且它只覆盖 Jump List，
  文件关联终究要写注册表）；
* 这三件事本质是"跟 Windows 外壳打招呼"，走 WinAPI 是标准做法，不引入对某个
  Qt 附加模块的依赖——依赖面小一点，"打包后少了某个 Qt 插件"这类静默失效
  就少一个来源。

失败语义：全部返回 ``bool`` 并吞掉异常。外壳集成是**增强**，不是启动路径的
一部分：写不了注册表（无权限 / 被组策略拦）也要能正常开软件，只是双击不关联。

注册表为什么写 ``HKCU\\Software\\Classes``（每用户）而不是 ``HKCR``：
``HKCR`` 需要管理员权限，而"我自己的文件用我装的软件打开"本就该是每用户
决定；装到 Program Files 再由安装器（Inno）写机器级关联，两者不冲突。
"""

from __future__ import annotations

import ctypes
import sys
from typing import Optional, Tuple

from app.app_identity import APPLICATION_ID, PROJECT_PROG_ID, PROJECT_TYPE_NAME

try:
    from service.project import PROJECT_EXTENSIONS
except Exception:  # pragma: no cover - 工程层不可用时退化成内置一份
    PROJECT_EXTENSIONS = (".rwproj", ".robotproj", ".core")


__all__ = [
    "IS_WINDOWS",
    "HIVE_CLASSES",
    "SHELL_TYPE_NAME",
    "add_recent_document",
    "is_association_registered",
    "notify_shell_associations_changed",
    "register_file_association",
    "set_app_user_model_id",
    "unregister_file_association",
]


#: 是否运行在 Windows 上（本模块所有能力的总开关）
IS_WINDOWS = sys.platform == "win32"

#: 每用户外壳类别根键（``HKCU\\Software\\Classes``，等价于 ``HKCR`` 的每用户视图）
HIVE_CLASSES = r"Software\Classes"

#: 资源管理器里显示的文件类型名（真源在 :data:`app.app_identity.PROJECT_TYPE_NAME`）。
#:
#: 三端（注册表 / macOS plist / Linux MIME）显示的是同一个名字，所以不在本模块
#: 另写一份——否则"Windows 上叫 A、macOS 上叫 B"没人会发现。
#: 为什么它不走 ``tr()``，见 :data:`app.app_identity.PROJECT_TYPE_NAME` 的注释。
SHELL_TYPE_NAME = PROJECT_TYPE_NAME

# --- shell32 常量（只有在这个平台上才用得到，集中放便于核对 MSDN） ---
_SHARD_PATHW = 0x00000005  # SHAddToRecentDocs：pv 是宽字符路径
_SHCNE_ASSOCCHANGED = 0x08000000  # SHChangeNotify：文件关联发生变化
_SHCNF_IDLIST = 0x0000
_SHCNF_FLUSH = 0x1000  # 同步刷新，别让"关联了但图标没变"拖到重启外壳


def _shell32():
    """拿到 ``shell32``；非 Windows 或加载失败返回 ``None``。

    惰性加载：模块级 ``ctypes.windll`` 在 macOS/Linux 上会直接 AttributeError，
    那样 :mod:`app.launcher` 在非 Windows 上连 import 都过不去。
    """
    if not IS_WINDOWS:
        return None
    try:
        return ctypes.windll.shell32  # type: ignore[attr-defined]
    except Exception:  # pragma: no cover - 仅在 Windows 上才可能真的失败
        return None


def set_app_user_model_id(app_id: str = APPLICATION_ID) -> bool:
    """声明本进程的任务栏身份；成功返回 ``True``。

    没有这一步时，Windows 会按"启动它的可执行文件"猜一个身份：PyInstaller
    打出来的 exe 常常被归到别的条目下，Jump List 的"最近"也因此无处可挂——
    表现为"别人装了就有最近文件，我没有"。
    """
    dll = _shell32()
    if dll is None:
        return False
    try:
        fn = dll.SetCurrentProcessExplicitAppUserModelID
        fn.argtypes = [ctypes.c_wchar_p]
        fn.restype = ctypes.HRESULT
        # S_OK == 0；已设置过会返回成功以外的 HRESULT，按"没成功"如实报
        return fn(ctypes.c_wchar_p(str(app_id))) == 0
    except Exception:  # pragma: no cover - 外壳 API 不可用只影响最近文件
        return False


def add_recent_document(path: str) -> bool:
    """把 ``path`` 报给系统"最近使用的文档"（Jump List 的"最近"分类）。

    数据仍以 :class:`app.session.RecentProjects` 为准——系统这份只用于让
    **任务栏/开始菜单**也能看到最近工程，不参与应用内的最近打开菜单。
    两边不一致时以应用内为准（打开成功才报，顺序上天然一致）。
    """
    dll = _shell32()
    if dll is None:
        return False
    try:
        fn = dll.SHAddToRecentDocs
        fn.argtypes = [ctypes.c_uint, ctypes.c_void_p]
        fn.restype = None
        buffer = ctypes.c_wchar_p(str(path))
        fn(_SHARD_PATHW, ctypes.cast(buffer, ctypes.c_void_p))
        return True
    except Exception:  # pragma: no cover
        return False


def notify_shell_associations_changed() -> bool:
    """通知资源管理器"文件关联变了"：不刷新则图标要等重启外壳才更新。"""
    dll = _shell32()
    if dll is None:
        return False
    try:
        fn = dll.SHChangeNotify
        fn.argtypes = [ctypes.c_long, ctypes.c_uint, ctypes.c_void_p, ctypes.c_void_p]
        fn.restype = None
        fn(_SHCNE_ASSOCCHANGED, _SHCNF_IDLIST | _SHCNF_FLUSH, None, None)
        return True
    except Exception:  # pragma: no cover
        return False


def _command_line(exe_path: str) -> str:
    """``shell\\open\\command`` 的默认值。

    ``"%1"`` 必须带引号：路径里有空格（``C:\\Users\\A B\\…``）时，不带引号会让
    Explorer 把路径切成两段，软件收到半个路径——症状是"双击有的文件能开、
    有的打不开"。
    """
    return f'"{exe_path}" "%1"'


def _set_default(key, value: str) -> None:
    import winreg

    winreg.SetValueEx(key, "", 0, winreg.REG_SZ, value)


def register_file_association(
    exe_path: str,
    icon_path: Optional[str] = None,
    prog_id: str = PROJECT_PROG_ID,
    extensions: Tuple[str, ...] = PROJECT_EXTENSIONS,
) -> bool:
    """把工程扩展名关联到本程序（免安装版自注册用）；成功返回 ``True``。

    有安装器时**由安装器负责**（Inno 的 ``[Registry]`` 段），本函数只服务
    "拷个目录就能用"的绿色版；两者写的是同一批键，不会互相打架。

    :param icon_path: 关联图标，缺省用 exe 自带的第一个图标
    :param extensions: 要关联的扩展名，缺省取 :data:`service.project.PROJECT_EXTENSIONS`
    """
    if not IS_WINDOWS:
        return False
    try:
        import winreg
    except Exception:  # pragma: no cover - 非 Windows 走不到这里
        return False

    icon_spec = f'"{icon_path}",0' if icon_path else f'"{exe_path}",0'
    try:
        with winreg.CreateKeyEx(winreg.HKEY_CURRENT_USER, f"{HIVE_CLASSES}\\{prog_id}") as key:
            _set_default(key, SHELL_TYPE_NAME)
        with winreg.CreateKeyEx(
            winreg.HKEY_CURRENT_USER, f"{HIVE_CLASSES}\\{prog_id}\\DefaultIcon"
        ) as key:
            _set_default(key, icon_spec)
        with winreg.CreateKeyEx(
            winreg.HKEY_CURRENT_USER, f"{HIVE_CLASSES}\\{prog_id}\\shell\\open\\command"
        ) as key:
            _set_default(key, _command_line(exe_path))
        for ext in extensions:
            with winreg.CreateKeyEx(winreg.HKEY_CURRENT_USER, f"{HIVE_CLASSES}\\{ext}") as key:
                _set_default(key, prog_id)
    except OSError:  # 权限/策略拦住：增强能力缺失，不向上传
        return False

    notify_shell_associations_changed()
    return True


def is_association_registered(prog_id: str = PROJECT_PROG_ID) -> bool:
    """主扩展名是否已指向 ``prog_id``（用于"已关联就不再写"的判断）。"""
    if not IS_WINDOWS:
        return False
    try:
        import winreg

        key_path = f"{HIVE_CLASSES}\\{PROJECT_EXTENSIONS[0]}"
        with winreg.OpenKey(winreg.HKEY_CURRENT_USER, key_path) as key:
            value, _ = winreg.QueryValueEx(key, "")
        return value == prog_id
    except OSError:
        return False
    except Exception:  # pragma: no cover
        return False


def unregister_file_association(
    prog_id: str = PROJECT_PROG_ID,
    extensions: Tuple[str, ...] = PROJECT_EXTENSIONS,
) -> bool:
    """撤销 :func:`register_file_association` 写入的键（卸载/改关联时用）。

    只删"默认值确实指向本程序"的扩展名键——用户如果已经把 ``.rwproj`` 指给了
    别的软件，不该因为我们卸载而把人家的关联删掉。
    """
    if not IS_WINDOWS:
        return False
    try:
        import winreg
    except Exception:  # pragma: no cover
        return False

    removed = False
    for ext in extensions:
        try:
            with winreg.OpenKeyEx(
                winreg.HKEY_CURRENT_USER, f"{HIVE_CLASSES}\\{ext}", 0, winreg.KEY_ALL_ACCESS
            ) as key:
                value, _ = winreg.QueryValueEx(key, "")
            if value != prog_id:
                continue
            winreg.DeleteKey(winreg.HKEY_CURRENT_USER, f"{HIVE_CLASSES}\\{ext}")
            removed = True
        except OSError:
            continue
    if removed:
        _delete_tree(winreg, prog_id)
        notify_shell_associations_changed()
    return removed


def _delete_tree(winreg, prog_id: str) -> None:
    """删掉 ProgID 及其子键。

    ``DeleteKey`` **不能删带子键的键**（会抛 OSError），所以要自底向上：
    先删 ``shell\\open\\command``，再删 ``shell\\open``，依此类推。
    每一层都吞掉 OSError——卸载路径上"这个键本来就不存在"是常态。
    """
    for sub in _PROG_ID_SUBTREE:
        try:
            winreg.DeleteKey(winreg.HKEY_CURRENT_USER, f"{HIVE_CLASSES}\\{prog_id}\\{sub}")
        except OSError:
            continue
    try:
        winreg.DeleteKey(winreg.HKEY_CURRENT_USER, f"{HIVE_CLASSES}\\{prog_id}")
    except OSError:
        pass


#: ProgID 下的子树，按"从深到浅"排列（:func:`_delete_tree` 依赖这个顺序）
_PROG_ID_SUBTREE = (r"shell\open\command", r"shell\open", "shell", "DefaultIcon")
