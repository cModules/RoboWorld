"""单实例守卫：同一时刻只跑一个 RoboWorld，第二个进程把路径交给第一个就退出。

为什么必须有：注册文件关联之后，**每双击一个工程文件都会启动一个新进程**。
onefile 打包的冷启动要先把几百 MB 解压到临时目录，几秒一个；更糟的是两个进程
会同时写最近打开与自动保存（原子写只保证"不会写坏"，不保证"谁最后写谁赢"——
用户的工程记录会随机丢一条）。

做法：``QLocalServer`` / ``QLocalSocket``（同机命名管道 / Unix 域套接字，Qt 自带，
不需要额外依赖）。首个进程监听；后续进程连上、把路径写进去、然后**自己退出**。

**失败即放行**：任何一步出错（名字被占、权限、socket 目录不可写）都当作
"我是主实例"继续启动。单实例是体验优化，绝不允许它变成"应用起不来"——
那比多开一个窗口严重得多。

为什么不做"第二个进程直接打开文件"：文件归属的是**窗口**（场景、示教程序、
自动保存槽位都在那儿），转交路径让主窗口自己走 ``open_project_path``，
才能复用最近打开、崩溃恢复那些既有语义。
"""

from __future__ import annotations

import getpass
import hashlib
from typing import Optional

from PySide6 import QtCore, QtNetwork

from app.app_identity import APPLICATION_ID

__all__ = ["SingleInstance"]


def _user_slug() -> str:
    """socket 名里的用户区分段。

    同名 socket 在多用户/多会话（远程桌面、fast user switching）里会**串台**：
    A 用户双击文件，路径却发给 B 用户的窗口。用用户名做一段隔离；
    用户名本身可能含不可用作文件名的字符（域用户 ``DOMAIN\\name``），
    所以取摘要而不是原样拼。
    """
    try:
        raw = getpass.getuser()
    except Exception:  # pragma: no cover - 无用户数据库时（容器）退化为固定段
        raw = "unknown"
    return hashlib.sha1(raw.encode("utf-8", "replace")).hexdigest()[:12]


#: 连接/写入超时（毫秒）。单实例握手是启动路径上的一小步，卡住就要放行
_HANDSHAKE_TIMEOUT_MS = 500

#: "没有文件，只是又双击了一次 exe"的载荷替身。
#:
#: 为什么需要它：``QLocalSocket`` **写 0 字节不可靠**（``waitForBytesWritten``
#: 会直接失败），所以"空意图"没法用一个空包表示——用户第二次双击 exe（不带文件）
#: 时，转交会静默失败，表现为"点了没反应"。用一个不可见字符当替身，读回时
#: 还原成空串，协议上始终"恰好一个载荷"。
_FOCUS_PAYLOAD = "\x00"


class SingleInstance(QtCore.QObject):
    """单实例守卫。用法见 :meth:`try_become_primary` 与 :meth:`hand_off`。

    :param app_id: 应用标识，参与 socket 名计算（与 Windows AppUserModelID、
        macOS bundle id 同口径，便于三端互认）
    """

    #: 主实例收到后续进程转交的路径（字符串）。由 :meth:`listen` 触发
    pathReceived = QtCore.Signal(str)

    def __init__(self, app_id: str = APPLICATION_ID, parent: Optional[QtCore.QObject] = None):
        super().__init__(parent)
        self._name = f"{app_id}-{_user_slug()}"
        self._server: Optional[QtNetwork.QLocalServer] = None

    @property
    def server_name(self) -> str:
        """实际监听/连接的 socket 名（测试与排错用）。"""
        return self._name

    def try_become_primary(self) -> bool:
        """尝试成为主实例：成功返回 ``True``；已有一个实例在跑返回 ``False``。

        判定不了时一律返回 ``True``（放行）——宁可退化成"可以多开"，也不能
        让应用起不动。单实例是体验优化，不值得为它赌上"打不开软件"。

        **顺序不能反**：先老实 ``listen``，失败了才去探测；绝不能先
        ``removeServer`` 再 ``listen``。同名 socket 就是同一个文件/管道，先删
        会把正在运行的主实例一起端掉——于是第二次启动也"成功"监听，单实例
        形同虚设。这个错只在第二次启动时暴露，单次冒烟永远发现不了。
        """
        try:
            server = QtNetwork.QLocalServer(self)
            if server.listen(self._name):
                server.newConnection.connect(self._on_new_connection)
                self._server = server
                return True
            server.close()

            # 没监听上：先问一句"是不是有人活着"。只有确认没人，才动清理——
            # 否则可能清掉的是活人的 socket
            if self._primary_is_alive():
                return False

            QtNetwork.QLocalServer.removeServer(self._name)
            retry = QtNetwork.QLocalServer(self)
            if retry.listen(self._name):
                retry.newConnection.connect(self._on_new_connection)
                self._server = retry
                return True
            retry.close()
            return True  # 判定不了：放行
        except Exception:  # pragma: no cover - 单实例设施不可用时放行
            return True

    def _primary_is_alive(self) -> bool:
        """连一次同名 socket：连得上就说明确实有一个活着的实例在监听。

        为什么不靠 ``serverError()`` 判：``QLocalServer`` 的 ServerError 只有
        NoError/PermissionError/ListenError 几种，"地址已被占用"不是它的枚举值
        （那是 ``QAbstractSocket`` 的），用它判会永远判不中。
        """
        socket = QtNetwork.QLocalSocket()
        try:
            socket.connectToServer(self._name)
            return socket.waitForConnected(_HANDSHAKE_TIMEOUT_MS)
        except Exception:  # pragma: no cover
            return False
        finally:
            try:
                socket.disconnectFromServer()
            except Exception:  # pragma: no cover
                pass

    def hand_off(self, path: str) -> bool:
        """把 ``path`` 转交给主实例；返回是否交付成功。

        返回 ``True`` 表示"已经有人在管了，你可以退出了"。连不上/写不进去
        都返回 ``False``，调用方应继续正常启动。

        ``path`` 为空串表示"没有文件"——按 :data:`_FOCUS_PAYLOAD` 的规则编码。
        """
        try:
            socket = QtNetwork.QLocalSocket()
            socket.connectToServer(self._name)
            if not socket.waitForConnected(_HANDSHAKE_TIMEOUT_MS):
                return False
            text = str(path or "").strip()
            payload = (text or _FOCUS_PAYLOAD).encode("utf-8")
            socket.write(payload)
            if not socket.waitForBytesWritten(_HANDSHAKE_TIMEOUT_MS):
                return False
            socket.flush()
            socket.disconnectFromServer()
            return True
        except Exception:  # pragma: no cover - 转交失败就自己起来
            return False

    def _on_new_connection(self) -> None:
        """主实例侧：读后续进程写进来的路径并发信号。"""
        server = self._server
        if server is None:  # pragma: no cover - 信号只可能在监听中触发
            return
        while server.hasPendingConnections():
            socket = server.nextPendingConnection()
            if socket is None:
                break
            try:
                socket.waitForReadyRead(_HANDSHAKE_TIMEOUT_MS)
                payload = bytes(socket.readAll()).decode("utf-8", "replace")
            except Exception:  # pragma: no cover - 读坏了就当没收到
                continue
            finally:
                socket.disconnectFromServer()
            # 空串也是一个合法意图（"把窗口提前台"），所以这里不过滤空值：
            # 过滤掉的话，用户第二次双击 exe 会毫无反馈
            self.pathReceived.emit("" if payload == _FOCUS_PAYLOAD else payload.strip())
