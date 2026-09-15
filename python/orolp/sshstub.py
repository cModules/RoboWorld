"""本机回环打桩 SSH 控制器（给 :class:`~orolp.link.SSHLink` 做无真机验收）。

与 :mod:`orolp.linkstub`（socket 打桩）同一思路：客户端走的是**真 SSH**——
paramiko 客户端连本机回环端口上的 paramiko 服务端，SFTP 子系统与 exec 通道
都是真实协议交互，只是"控制器"这一侧是打桩的。

打桩服务端提供：

* **SFTP 子系统**——根目录限定在给定文件夹内（越界路径按 SFTPNoSuchFile 回答），
  文件读写/列目录真实落盘，所以"上传的代码在不在、内容对不对"可以直接断言；
* **exec 通道**——命令用 ``subprocess`` 在打桩根目录下执行，stdout/stderr/退出码
  原样回传，可注入固定应答（不做真实执行，模拟控制器专属指令）；
* **认证**——默认接受任意密码/公钥（打桩控制器不设防），可配置一个错误密码
  来验收客户端的失败语义。

CLI：``python -m orolp.sshstub --port 2222 --root <目录>``——手动验收用。
"""

from __future__ import annotations

import os
import re
import shlex
import socket
import subprocess
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

__all__ = ["SSHStubServer", "StubAuthLog"]


_CD_RE = re.compile(r"^cd\s+'((?:[^'\\]|\\.)*)'\s*&&\s*(.*)$", re.S)


def _paramiko():
    try:
        import paramiko
    except ImportError as exc:  # pragma: no cover - 环境缺依赖时给可读提示
        raise RuntimeError("SSH 打桩服务端需要 paramiko：pip install paramiko") from exc
    return paramiko


class StubAuthLog:
    """认证记录（测试用）：谁用什么方式登进来、有没有被拒过。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.passwords: List[Tuple[str, str, bool]] = []  # (username, password, ok)

    def add_password(self, username: str, password: str, ok: bool) -> None:
        with self._lock:
            self.passwords.append((username, password, ok))

    @property
    def accepted(self) -> List[str]:
        return [u for u, _p, ok in self.passwords if ok]

    @property
    def rejected(self) -> List[str]:
        return [u for u, _p, ok in self.passwords if not ok]


def _build_classes():
    """构造打桩服务端所需的 paramiko 类（延迟到调用时，模块导入不碰 paramiko）。"""
    paramiko = _paramiko()

    class StubSFTPHandle(paramiko.SFTPHandle):
        """把 SFTP 读写映射到真实文件句柄。

        注意 paramiko 的契约：``write`` 必须返回 ``SFTP_OK``（0）表示成功，
        返回写入字节数会被当成错误码发给客户端（踩过：客户端报 OSError: Unknown）。
        """

        def __init__(self, flags: int, path: Path, mode: str):
            super().__init__(flags)
            self._path = path
            self._mode = mode
            self._f = open(path, mode)

        def read(self, offset: int, length: int):
            self._f.seek(offset)
            return self._f.read(length)

        def write(self, offset: int, data: bytes):
            self._f.seek(offset)
            self._f.write(data)
            return paramiko.sftp.SFTP_OK

        def close(self):
            try:
                if not self._f.closed:
                    self._f.close()
            except Exception:
                pass
            return super().close()

    class StubSFTPServer(paramiko.SFTPServerInterface):
        """根目录受限的最小 SFTP 实现：open/list_folder/stat/lstat/remove。

        paramiko 5 的服务端接口约定：出错时**返回错误码整数**（如
        ``SFTP_NO_SUCH_FILE``）而不是抛异常——抛出去的异常会被顶层 catch
        一律变成 SFTP_FAILURE，客户端就分不清"没有文件"和"坏了"。
        """

        _OK = paramiko.sftp.SFTP_OK
        _NO_SUCH = paramiko.sftp.SFTP_NO_SUCH_FILE
        _FAIL = paramiko.sftp.SFTP_FAILURE

        def __init__(self, server: object, root: str, *largs, **kwargs):
            super().__init__(server, *largs, **kwargs)
            self.root = Path(root).resolve()
            self.root.mkdir(parents=True, exist_ok=True)

        # -- 路径安全：chroot 语义——所有路径（含绝对路径）都映射进根目录，
        #    归一化后仍想逃出去的一律按"不存在"回答（不泄露结构） --------- #
        def _real(self, path):
            text = path.decode("utf-8", "replace") if isinstance(path, bytes) else str(path)
            candidate = self.root / text.lstrip("/")
            resolved = candidate.resolve()
            try:
                resolved.relative_to(self.root)
            except ValueError:
                return None
            return resolved

        def open(self, path, flags, attr):
            real = self._real(path)
            if real is None:
                return self._NO_SUCH
            # paramiko 传来的 flags 与 os 模块同构；带写位就按写打开（覆盖写）
            if flags & (os.O_WRONLY | os.O_RDWR):
                real.parent.mkdir(parents=True, exist_ok=True)
                return StubSFTPHandle(flags, real, "wb")
            if not real.is_file():
                return self._NO_SUCH
            return StubSFTPHandle(flags, real, "rb")

        def list_folder(self, path):
            real = self._real(path)
            if real is None or not real.is_dir():
                return self._NO_SUCH
            out = []
            for child in sorted(real.iterdir()):
                st = child.stat()
                attrs = paramiko.SFTPAttributes.from_stat(st)
                attrs.filename = child.name
                out.append(attrs)
            return out

        def stat(self, path):
            real = self._real(path)
            if real is None or not real.exists():
                return self._NO_SUCH
            return paramiko.SFTPAttributes.from_stat(real.stat())

        def lstat(self, path):
            return self.stat(path)

        def remove(self, path):
            real = self._real(path)
            if real is None or not real.exists():
                return self._NO_SUCH
            if real.is_dir():
                return self._FAIL
            real.unlink()
            return self._OK

    class StubServer(paramiko.ServerInterface):
        """打桩控制器：可配密码（配了才校验），SFTP 子系统 + exec 通道。"""

        def __init__(
            self,
            root: str,
            auth: StubAuthLog,
            password: Optional[str] = None,
            fixed_replies: Optional[Dict[str, str]] = None,
            exec_log: Optional[List[str]] = None,
        ):
            self.root = str(root)
            self.auth = auth
            self.password = password
            self.fixed_replies = dict(fixed_replies or {})
            # exec 记录挂到打桩实例的共享列表（每条连接各建一个接口对象）
            self.exec_log: List[str] = exec_log if exec_log is not None else []

        # --------------------------- 认证 --------------------------- #
        def check_auth_password(self, username: str, password: str):
            ok = self.password is None or password == self.password
            self.auth.add_password(username, password, ok)
            return (
                paramiko.AUTH_SUCCESSFUL
                if ok
                else paramiko.AUTH_FAILED
            )

        def check_auth_publickey(self, username: str, key: object):
            # 打桩：任何公钥都放行
            return paramiko.AUTH_SUCCESSFUL

        def get_allowed_auths(self, username: str) -> str:
            return "password,publickey"

        # --------------------------- 通道 --------------------------- #
        def check_channel_request(self, kind: str, chanid: int):
            return paramiko.OPEN_SUCCEEDED

        def check_channel_exec_request(self, channel, command: bytes) -> bool:
            # paramiko 在传输线程里同步调用本方法、返回后才发 CHANNEL_SUCCESS 应答；
            # 若在这里同步执行命令并 close 通道，客户端会先收到 CHANNEL_CLOSE，
            # exec_command 的 _wait_for_event 直接抛 "Channel closed"。
            # 因此按 paramiko demo_server 的模式：另起线程干活，本方法立即返回。
            cmd = command.decode("utf-8", "replace")
            self.exec_log.append(cmd)
            threading.Thread(
                target=self._exec_thread,
                args=(channel, cmd),
                daemon=True,
                name="orolp-sshstub-exec",
            ).start()
            return True

        def _exec_thread(self, channel, cmd: str) -> None:
            try:
                reply = self.fixed_replies.get(cmd)
                if reply is not None:
                    channel.sendall(reply.encode("utf-8"))
                    channel.send_exit_status(0)
                    return
                try:
                    cmd = self._chroot_cmd(cmd)
                    proc = subprocess.run(
                        cmd,
                        shell=True,
                        cwd=self.root,
                        capture_output=True,
                        timeout=20,
                    )
                    if proc.stdout:
                        channel.sendall(proc.stdout)
                    if proc.stderr:
                        channel.sendall_stderr(proc.stderr)  # 走扩展数据流，别混进 stdout
                    channel.send_exit_status(proc.returncode)
                except subprocess.TimeoutExpired:
                    channel.sendall_stderr(b"stub: command timeout\n")
                    channel.send_exit_status(124)
                except Exception as exc:  # noqa: BLE001 - 打桩端任何异常都回给客户端
                    channel.sendall_stderr(f"stub: {exc}\n".encode("utf-8"))
                    channel.send_exit_status(127)
            finally:
                # 给传输线程一点时间把 CHANNEL_SUCCESS 应答发出去再关通道——
                # 应答在本方法返回后才发（_handle_request 同步调用 handler），
                # 极快路径（如 fixed_replies 秒回）若立即 close，客户端会先收到
                # CHANNEL_CLOSE，exec_command 直接抛 "Channel closed"。
                time.sleep(0.02)
                try:
                    channel.close()
                except Exception:
                    pass

        def _chroot_cmd(self, cmd: str) -> str:
            """把 exec 命令里的 chroot 敏感路径映射进桩根目录。

            覆盖客户端生成的两种形态：``mkdir -p <绝对路径>``（连接期建目录）
            与 ``cd '<绝对路径>' && <命令>``（run_command 带 cwd）。打桩是
            chroot 模型——SFTP 侧的绝对路径都映射进根目录，exec 侧若不同样
            映射，绝对路径会打到打桩宿主机的真文件系统上。
            """
            if cmd.startswith("mkdir"):
                try:
                    parts = shlex.split(cmd)
                except ValueError:
                    return cmd
                mapped = ["mkdir"]
                for part in parts[1:]:
                    if part.startswith("/"):
                        mapped.append(str(Path(self.root) / part.lstrip("/")))
                    else:
                        mapped.append(part)
                return " ".join("'" + p + "'" for p in mapped)
            # cd '<dir>' && rest
            m = _CD_RE.match(cmd)
            if m:
                inner = m.group(1).replace("'\\''", "'")
                if inner.startswith("/"):
                    mapped = str(Path(self.root) / inner.lstrip("/"))
                    return "cd '" + mapped + "' && " + m.group(2)
            return cmd

        def check_channel_shell_request(self, channel) -> bool:
            return False  # 打桩不提供交互 shell

    return paramiko, StubSFTPHandle, StubSFTPServer, StubServer


_HOST_KEY_LOCK = threading.Lock()
_HOST_KEY_CACHE: List[object] = []


def _default_host_key() -> object:
    """模块级缓存的一次性主机密钥：多个打桩实例共享一次 RSA 生成，
    避免逐测试等密钥生成（生成一次约需几百毫秒）。"""
    paramiko = _paramiko()
    with _HOST_KEY_LOCK:
        if not _HOST_KEY_CACHE:
            _HOST_KEY_CACHE.append(paramiko.RSAKey.generate(2048))
        return _HOST_KEY_CACHE[0]


class SSHStubServer:
    """本机回环 SSH 打桩控制器。

    Args:
        root: SFTP 根目录（真实落盘，测试用 tmp_path）
        port: 监听端口；``0`` 表示自动挑一个空闲端口，用 :attr:`port` 读回
        password: 设置后密码必须匹配才放行（默认任意密码放行）
        fixed_replies: ``{命令: 应答文本}``——命中就不再真实执行（模拟控制器
            专属指令，如 ``load pick.src``）
        host_key: 自备主机密钥；缺省用**模块级缓存**的一次性 RSA 密钥——
            多个打桩实例共享一次生成，避免测试逐个等 RSA 生成
    """

    def __init__(
        self,
        root: str | os.PathLike,
        port: int = 0,
        *,
        password: Optional[str] = None,
        fixed_replies: Optional[Dict[str, str]] = None,
        host_key: Optional[object] = None,
    ) -> None:
        paramiko, sftp_handle, sftp_si, server_si = _build_classes()
        self._paramiko = paramiko
        self.root = Path(root).resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.password = password
        self.auth = StubAuthLog()
        self._fixed_replies = dict(fixed_replies or {})
        self._server_si = server_si
        self._sftp_si = sftp_si
        self.exec_log: List[str] = []

        # 一次性主机密钥：打桩每次现场生成，不存在密钥分发问题
        self.host_key = host_key if host_key is not None else _default_host_key()

        self._lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._lsock.bind(("127.0.0.1", port))
        self._lsock.listen(4)
        self._lsock.settimeout(0.2)
        self.port = self._lsock.getsockname()[1]
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True, name="orolp-sshstub")
        self._transports: List[object] = []
        self._lock = threading.Lock()
        self._thread.start()

    # --------------------------- 服务循环 --------------------------- #

    def _serve(self) -> None:  # pragma: no cover - 线程体，靠集成测试覆盖
        paramiko = self._paramiko
        while not self._stop.is_set():
            try:
                sock, _addr = self._lsock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            transport = paramiko.Transport(sock)
            transport.add_server_key(self.host_key)

            interface = self._server_si(
                str(self.root),
                self.auth,
                self.password,
                self._fixed_replies,
                self.exec_log,
            )

            try:
                transport.set_subsystem_handler(
                    "sftp", paramiko.SFTPServer, self._sftp_si, root=str(self.root)
                )
            except Exception:
                pass
            with self._lock:
                self._transports.append(transport)
            try:
                transport.start_server(server=interface)
            except Exception:
                try:
                    transport.close()
                except Exception:
                    pass

    # --------------------------- 生命周期 --------------------------- #

    def stop(self) -> None:
        """停止监听并关闭所有活跃连接（幂等）。"""
        if self._stop.is_set():
            return
        self._stop.set()
        with self._lock:
            transports, self._transports = list(self._transports), []
        for t in transports:
            try:
                t.close()
            except Exception:
                pass
        try:
            self._lsock.close()
        except OSError:
            pass
        self._thread.join(timeout=2.0)

    def __enter__(self) -> "SSHStubServer":
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        self.stop()
        return False


def main() -> None:  # pragma: no cover - 手动验收入口
    import argparse
    import signal
    import time

    parser = argparse.ArgumentParser(description="orolp SSH 打桩控制器（本机回环验收用）")
    parser.add_argument("--port", type=int, default=2222)
    parser.add_argument("--root", required=True, help="SFTP 根目录")
    parser.add_argument("--password", default=None, help="设置后必须匹配（默认任意密码放行）")
    args = parser.parse_args()

    server = SSHStubServer(args.root, args.port, password=args.password)
    print(f"ADDRESS ssh://127.0.0.1:{server.port} root={server.root}")
    print(f"HOSTKEY fingerprint={server.host_key.get_fingerprint().hex()}", flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == "__main__":
    main()
