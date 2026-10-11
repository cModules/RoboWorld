"""本机回环打桩控制器——让"真机部署 + 回读 + 偏差"链路在没有真机时也能整跑一遍。

这不是真机驱动，也不冒充真机：它按 ``core-socket`` v1 协议（客户端见
:class:`domain.link.SocketLink`）应答，用途有三个：

1. **验收**：开发/交付环境通常没有机器人。跑起这个打桩服务端，``SocketLink``
   走的是**真 TCP socket**，于是"下发程序 → 回读实际位置 → 算偏差 → 上传孪生
   轨迹"整条链路可以被真跑一遍，而不是只靠单元测试里的假对象。
2. **联调**：现场调试前先在 PC 上跑通，把"程序对不对"和"网络/协议通不通"两件
   事分开定位。
3. **排障**：可以主动注入**乱码 / 掉线 / 拒答 / 静默**四类故障，用来检查客户端
   在链路异常时到底是报错、还是把断了的连接继续当"已连接"用。

打桩服务端的关节回读可以由两种模型产生：

* 给了 ``robot``（＋可选 ``plant``）→ 复用 :class:`service.twinsync.PlantLink`，
  也就是 v1.0.3 那套"合成真机"（零位偏置 / 传动比误差 / 编码器噪声 / 伺服一阶
  滞后），回读值**带真实形状的偏差**，最适合验收孪生链路；
* 没给 ``robot`` → 退化成"回显 + 线性误差"（``bias`` / ``scale`` / ``noise_deg``），
  足够验证协议本身，且不需要加载 URDF。

命令行::

    PYTHONPATH=python python -m adapter.linkstub --port 30002 \\
        --urdf assets/models/ur/ur5e/ur5e.urdf --bias-all 0.05 --scale 5e-4 --lag-tau 0.02

然后在另一个终端把 :class:`~domain.link.SocketLink` 指向 ``127.0.0.1:30002`` 即可。

真机侧要准备的是**同协议的服务端**：UR 上用 URScript 的 ``socket_open`` /
``read`` / ``write`` 写一小段转发桩，或由 PC 侧代理进程把厂商 SDK / 原生协议
翻译成这个协议。协议本身只有一问一答的单行 JSON，见 ``link.SocketLink`` 的类
文档。
"""

from __future__ import annotations

import argparse
import base64
import binascii
import json
import socket
import socketserver
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from domain.link import (
    DEFAULT_SOCKET_PORT,
    SOCKET_API_VERSION,
    _line_count,
    _safe_name,
)

__all__ = ["StubState", "ControllerStubServer"]


def _abbrev(value: Any, limit: int = 160) -> Any:
    """把一条报文压成"能打印、又不淹没重点"的形态。

    长 base64（程序正文）只留规模、长数组只留前几项——验收时想看的是"客户端发了
    什么 op、带了什么标量"，而不是一屏 base64。
    """
    if isinstance(value, dict):
        out: Dict[str, Any] = {}
        for key, item in value.items():
            if key == "code_b64" and isinstance(item, str):
                out[key] = f"<{len(item)} 字符 base64>"
            else:
                out[key] = _abbrev(item, limit)
        return out
    if isinstance(value, (list, tuple)):
        if len(value) > 4:
            return [_abbrev(v, limit) for v in value[:3]] + [f"…共 {len(value)} 项"]
        return [_abbrev(v, limit) for v in value]
    if isinstance(value, str) and len(value) > limit:
        return value[:limit] + "…"
    return value


class StubState:
    """打桩控制器的内部状态（与网络无关，可被单测直接驱动）。

    把"控制器状态"和"socket 服务器"分开，是为了让 :meth:`handle` 能脱离网络单测
    ——协议语义的正确性不该只能通过真开端口来验证。

    Args:
        robot: 可选机器人实例；给了它会用 :class:`~service.twinsync.PlantLink` 建模
            真机误差（含伺服滞后）
        plant: 传给 ``PlantLink`` 的误差模型（``None`` ＝ 无误差）
        dof: 关节数（未给 ``robot`` 时使用）
        bias: 各关节零位偏置（度；标量=全轴相同，序列=逐轴）
        scale: 减速比相对误差（标量）
        noise_deg: 编码器噪声标准差（度）
        seed: 随机种子（保证可复现）
        caps: 对外声明的能力表，默认 ``programs``/``feedback``/``pose`` 全开
        api: 对外声明的协议版本，默认 :data:`domain.link.SOCKET_API_VERSION`
        server_name: 握手自述名
        address: 对外自述地址（用于生成句柄）；由服务器注入
        dialog_capacity: 保留最近多少条往返报文（0＝不记）；用于示例/网页出示
            "这条数据真的过线了"
    """

    def __init__(
        self,
        robot: "object" = None,
        plant: "object" = None,
        *,
        dof: int = 6,
        bias: Any = 0.0,
        scale: float = 0.0,
        noise_deg: float = 0.0,
        seed: int = 0,
        caps: Optional[Dict[str, Any]] = None,
        api: Optional[int] = None,
        server_name: str = "core.ControllerStubServer",
        address: str = "stub",
        dialog_capacity: int = 32,
    ) -> None:
        self.caps: Dict[str, bool] = {"programs": True, "feedback": True, "pose": True}
        if caps:
            self.caps.update({str(k): bool(v) for k, v in caps.items()})
        self.api = SOCKET_API_VERSION if api is None else int(api)
        self.server_name = str(server_name)
        self.address = str(address)
        self.dialog_capacity = max(0, int(dialog_capacity))

        self.programs: Dict[str, str] = {}
        self.manifests: Dict[str, Dict[str, Any]] = {}
        #: 收到过的 op 序列（验收时断言"客户端确实发了什么"）
        self.requests: List[str] = []
        #: 收到过的 set_joints 请求（含 at_time），用于核对时间戳转发
        self.commands: List[Dict[str, Any]] = []
        #: 最近若干条往返报文（已缩写，不含完整代码正文）
        self.dialog: List[Dict[str, Any]] = []
        #: 累计连上的客户端数 / 已正常结束的客户端数
        self.clients = 0
        self.clients_done = 0

        self.dof = int(dof)
        self.plant_link = None
        if robot is not None:
            from service.twinsync import PlantLink, PlantModel

            self.plant_link = PlantLink(robot, plant or PlantModel())
            self.plant_link.connect()
            self.dof = int(getattr(robot, "dof", self.dof)) or self.dof
            self._q = np.zeros(self.dof, dtype=np.float64)
        else:
            self._q = np.zeros(self.dof, dtype=np.float64)

        # 无 robot 时的线性误差模型（回显 + 偏移 + 缩放 + 噪声）
        self._bias = self._as_vector(bias, self.dof)
        self._scale = float(scale)
        self._noise = float(noise_deg)
        self._rng = np.random.default_rng(seed)

    # ------------------------------ 工具 ------------------------------ #

    @staticmethod
    def _as_vector(value: Any, dof: int) -> np.ndarray:
        if isinstance(value, (int, float)):
            return np.full(dof, float(value), dtype=np.float64)
        vals = [float(v) for v in value]
        if len(vals) == 1:
            return np.full(dof, vals[0], dtype=np.float64)
        if len(vals) != dof:
            raise ValueError(f"bias 长度 {len(vals)} 与关节数 {dof} 不符")
        return np.asarray(vals, dtype=np.float64)

    def joints(self) -> Optional[np.ndarray]:
        """当前"实际"关节角（deg）。"""
        if self.plant_link is not None:
            q = self.plant_link.read_joints()
        else:
            q = self._q
        return None if q is None else np.asarray(q, dtype=np.float64).ravel()

    def pose(self) -> Optional[Tuple[float, ...]]:
        """当前 TCP 位姿 ``(x,y,z,rx,ry,rz)``；没有机器人模型时返回 ``None``。"""
        if self.plant_link is None:
            return None
        p = self.plant_link.get_pose()
        return None if p is None else tuple(float(v) for v in p.to_xyz_rpy())

    # ------------------------------ 协议 ------------------------------ #

    def handle(self, req: Dict[str, Any]) -> Dict[str, Any]:
        """处理一条请求，返回响应体（含 ``ok``）。

        纯函数式、不碰网络；顺带把最近若干条往返记进 :attr:`dialog`（代码正文只留
        规模，见 :func:`_abbrev`），供示例与网页出示"这条数据真的过线了"。
        """
        op = str(req.get("op") or "")
        self.requests.append(op)
        resp = self._dispatch(op, req)
        if self.dialog_capacity > 0:
            self.dialog.append(
                {
                    "op": op,
                    "request": _abbrev(req),
                    "response": _abbrev(resp),
                    "at": time.time(),
                }
            )
            if len(self.dialog) > self.dialog_capacity:
                del self.dialog[: len(self.dialog) - self.dialog_capacity]
        return resp

    def _dispatch(self, op: str, req: Dict[str, Any]) -> Dict[str, Any]:
        if op == "hello":
            return {
                "ok": True,
                "api": self.api,
                "server": self.server_name,
                "dof": self.dof,
                "caps": dict(self.caps),
            }

        if op == "bye":
            return {"ok": True, "closing": True}

        if op == "upload":
            if not self.caps.get("programs"):
                return {"ok": False, "error": "本控制器不支持程序下发（caps.programs=false）"}
            name = str(req.get("name") or "program")
            ext = str(req.get("ext") or "src").lstrip(".") or "src"
            raw = req.get("code_b64")
            if raw is None:
                return {"ok": False, "error": "缺少 code_b64"}
            try:
                code = base64.b64decode(str(raw)).decode("utf-8")
            except (binascii.Error, UnicodeDecodeError) as exc:
                return {"ok": False, "error": f"code_b64 解码失败：{exc}"}
            self.programs[name] = code
            self.manifests[name] = {
                "name": name,
                "ext": ext,
                "lines": _line_count(code),
                "bytes": len(code.encode("utf-8")),
                "meta": dict(req.get("meta") or {}),
                "received_at": time.time(),
            }
            safe = _safe_name(name)
            return {"ok": True, "handle": f"sock://{self.address}/programs/{safe}.{ext}"}

        if op == "list":
            if not self.caps.get("programs"):
                return {"ok": False, "error": "本控制器不支持程序查询（caps.programs=false）"}
            return {"ok": True, "programs": sorted(self.programs)}

        if op == "fetch":
            if not self.caps.get("programs"):
                return {"ok": False, "error": "本控制器不支持程序查询（caps.programs=false）"}
            name = str(req.get("name") or "")
            code = self.programs.get(name)
            if code is None:
                return {"ok": True, "code_b64": None}
            return {
                "ok": True,
                "code_b64": base64.b64encode(code.encode("utf-8")).decode("ascii"),
                "ext": (self.manifests.get(name) or {}).get("ext"),
            }

        if op == "read_joints":
            if not self.caps.get("feedback"):
                return {"ok": True, "joints": None}
            q = self.joints()
            return {"ok": True, "joints": None if q is None else [float(v) for v in q]}

        if op == "set_joints":
            joints = req.get("joints")
            if joints is None:
                return {"ok": False, "error": "缺少 joints"}
            try:
                q = np.asarray([float(v) for v in joints], dtype=np.float64).ravel()
            except (TypeError, ValueError):
                return {"ok": False, "error": "joints 不是数值序列"}
            if q.size != self.dof:
                return {
                    "ok": False,
                    "error": f"关节数不符：收到 {q.size}，控制器有 {self.dof} 轴",
                }
            at_time = req.get("at_time")
            record: Dict[str, Any] = {"joints": [float(v) for v in q], "at_time": at_time}
            self.commands.append(record)
            if self.plant_link is not None:
                self.plant_link.set_joints(q, at_time=None if at_time is None else float(at_time))
            else:
                self._q = q * (1.0 + self._scale) + self._bias
                if self._noise > 0:
                    self._q = self._q + self._rng.normal(0.0, self._noise, size=q.size)
            return {"ok": True}

        if op == "get_pose":
            if not self.caps.get("pose"):
                return {"ok": True, "pose": None}
            return {"ok": True, "pose": self.pose()}

        return {"ok": False, "error": f"不支持的 op：{op or '(空)'}"}


class _StubHandler(socketserver.StreamRequestHandler):
    """单连接处理循环：一问一答，直到对端断开 / ``bye`` / 主动撂断。"""

    def handle(self) -> None:
        srv: "ControllerStubServer" = self.server.stub  # type: ignore[attr-defined]
        srv.state.clients += 1
        served = 0
        try:
            while True:
                try:
                    line = self.rfile.readline()
                except OSError:
                    return
                if not line:
                    return
                text = line.decode("utf-8", "replace").strip()
                if not text:
                    continue
                if srv.garbage:
                    self.wfile.write(b"@@not-json@@\n")
                    continue
                if srv.drop_after is not None and served >= srv.drop_after:
                    return  # 撂断：客户端应看到 EOF 并如实报"连接中断"
                try:
                    req = json.loads(text)
                except ValueError:
                    self._reply({"ok": False, "error": "请求不是合法 JSON"})
                    continue
                if not isinstance(req, dict):
                    req = {}
                served += 1
                op = str(req.get("op") or "")
                if srv.silent:
                    continue  # 收下不答：客户端应超时
                if op in srv.reject_ops:
                    self._reply({"ok": False, "error": f"打桩服务端拒绝 {op}"})
                    continue
                if srv.latency > 0:
                    time.sleep(srv.latency)
                resp = srv.state.handle(req)
                self._reply(resp)
                if resp.get("closing"):
                    return
        finally:
            srv.state.clients_done += 1

    def _reply(self, payload: Dict[str, Any]) -> None:
        self.wfile.write(json.dumps(payload, ensure_ascii=False).encode("utf-8") + b"\n")


class _StubTCPServer(socketserver.ThreadingTCPServer):
    """线程化 TCP 服务端：一个客户端一个线程（打桩场景够用且好懂）。

    额外记住"当前活着的连接"，这样 :meth:`ControllerStubServer.stop` 能真正**断开**
    客户端——只关监听口的话，已连上的会话会被守护线程继续伺候，"停了服务端但客户端
    还在正常收答"会让人误判链路状态。
    """

    allow_reuse_address = True
    daemon_threads = True  # 守护线程：stop() 不会卡在等连接上

    #: 由 :class:`ControllerStubServer` 注入（避免 handler 里再传一遍上下文）
    stub: "ControllerStubServer"

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._conns: set = set()

    def process_request(self, request, client_address) -> None:  # type: ignore[no-untyped-def]
        self._conns.add(request)
        super().process_request(request, client_address)

    def shutdown_request(self, request) -> None:  # type: ignore[no-untyped-def]
        self._conns.discard(request)
        super().shutdown_request(request)

    def close_connections(self) -> None:
        """断开当前所有客户端连接。"""
        for conn in list(self._conns):
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                conn.close()
            except OSError:
                pass
        self._conns.clear()


class ControllerStubServer:
    """把 :class:`StubState` 挂到一个线程化的 TCP 服务端上（打桩）。

    Args:
        host: 监听地址
        port: 监听端口；``0`` 表示由系统分配（测试常用，用 :attr:`port` 读回）
        robot: 可选机器人实例（走 ``PlantLink`` 建模真机误差）
        plant: 误差模型
        dof / bias / scale / noise_deg: 见 :class:`StubState`
        caps: 覆盖对外声明的能力（例如 ``{"feedback": False}``）
        api: 覆盖对外声明的协议版本（用于验收客户端的版本校验）
        server_name: 握手自述名
        latency: 每条响应前的延迟（秒），模拟网络/控制器耗时
        reject_ops: 直接拒绝（``ok:false``）的 op 集合——**业务性拒绝**
        drop_after: 每条连接处理 N 条请求后**撂断**（模拟网络中断）
        garbage: 一律回复非 JSON 报文（模拟接到别的服务/固件乱码）
        silent: 收下请求但从不回复（模拟控制器卡死，客户端应超时）
        dialog: 保留最近多少条往返报文（0＝不记），见 :attr:`StubState.dialog`

    后四个参数是**故意做坏**用的，别在正常验收时打开。
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 0,
        *,
        robot: "object" = None,
        plant: "object" = None,
        dof: int = 6,
        bias: Any = 0.0,
        scale: float = 0.0,
        noise_deg: float = 0.0,
        seed: int = 0,
        caps: Optional[Dict[str, Any]] = None,
        api: Optional[int] = None,
        server_name: str = "core.ControllerStubServer",
        latency: float = 0.0,
        reject_ops: Sequence[str] = (),
        drop_after: Optional[int] = None,
        garbage: bool = False,
        silent: bool = False,
        dialog: int = 32,
    ) -> None:
        self.host = str(host)
        self.port = int(port)
        self.latency = float(latency)
        self.reject_ops = {str(op) for op in reject_ops}
        self.drop_after = None if drop_after is None else int(drop_after)
        self.garbage = bool(garbage)
        self.silent = bool(silent)

        self.state = StubState(
            robot,
            plant,
            dof=dof,
            bias=bias,
            scale=scale,
            noise_deg=noise_deg,
            seed=seed,
            caps=caps,
            api=api,
            server_name=server_name,
            dialog_capacity=dialog,
        )
        self.address = f"{self.host}:{self.port}"
        self._server: Optional[socketserver.TCPServer] = None
        self._thread: Optional[threading.Thread] = None

    # ------------------------------ 生命周期 ------------------------------ #

    def start(self) -> int:
        """启动服务端（后台线程），返回实际监听端口。"""
        if self._server is not None:
            return self.port
        server = _StubTCPServer((self.host, self.port), _StubHandler)
        server.stub = self
        host, port = server.server_address[:2]
        self.host, self.port = str(host), int(port)
        self.address = f"{self.host}:{self.port}"
        self.state.address = self.address  # 句柄里体现真实地址
        self._server = server
        self._thread = threading.Thread(
            target=server.serve_forever,
            # 轮询间隔默认 0.5s，会让每次 stop() 都白等半秒；打桩场景收快一点
            kwargs={"poll_interval": 0.02},
            name="core-linkstub",
            daemon=True,
        )
        self._thread.start()
        return self.port

    def stop(self) -> None:
        """停止服务端（幂等）：先断开已连上的客户端，再关监听口。

        "断开客户端"这一步是刻意的——否则 stop() 之后已连接的客户端还能正常收答，
        人会以为服务端还活着（验收/排障时这种误判最贵）。
        """
        server, self._server = self._server, None
        if server is not None:
            server.shutdown()
            server.close_connections()  # type: ignore[attr-defined]
            server.server_close()
        thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=2.0)

    def __enter__(self) -> "ControllerStubServer":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        self.stop()
        return False

    def describe(self) -> str:
        """一行可读状态，供示例/日志打印。"""
        caps = ",".join(k for k, v in sorted(self.state.caps.items()) if v) or "-"
        return (
            f"ControllerStubServer[{self.address}] 程序={len(self.state.programs)} 条 "
            f"dof={self.state.dof} 能力={caps} 客户端={self.state.clients}"
            f"（已完成 {self.state.clients_done}）"
        )

    def __repr__(self) -> str:  # pragma: no cover - 纯展示
        return f"<ControllerStubServer {self.address}>"


# --------------------------------------------------------------------------- #
# 命令行入口：让用户在自己机器上跑出一个"假控制器"
# --------------------------------------------------------------------------- #


def main(argv: Optional[Sequence[str]] = None) -> int:
    """``python -m adapter.linkstub``：在本地起一个打桩控制器。"""
    parser = argparse.ArgumentParser(
        prog="python -m adapter.linkstub",
        description="本机回环打桩控制器（core-socket v1）：无真机验收 RoboWorld 的部署/回读/孪生链路",
    )
    parser.add_argument("--host", default="127.0.0.1", help="监听地址（默认 127.0.0.1）")
    parser.add_argument(
        "--port", type=int, default=DEFAULT_SOCKET_PORT, help="监听端口（0=自动分配）"
    )
    parser.add_argument("--urdf", help="可选机器人 URDF：给了它就用真实正/逆运动学回答位姿")
    parser.add_argument("--dof", type=int, default=6, help="关节数（未给 URDF 时使用）")
    parser.add_argument("--bias-all", type=float, default=0.0, help="全轴零位偏置（度）")
    parser.add_argument("--scale", type=float, default=0.0, help="减速比相对误差（如 5e-4）")
    parser.add_argument("--noise", type=float, default=0.0, help="编码器噪声标准差（度）")
    parser.add_argument(
        "--lag-tau", type=float, default=0.0, help="伺服一阶滞后时间常数（秒，需 --urdf）"
    )
    parser.add_argument("--no-feedback", action="store_true", help="假装控制器不支持关节回读")
    parser.add_argument(
        "--max-clients", type=int, default=0, help="服务满 N 个客户端后自动退出（0=一直服务）"
    )
    parser.add_argument(
        "--print-address",
        action="store_true",
        help="额外输出一行 `ADDRESS <host>:<port>`，便于脚本解析",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    robot = None
    plant = None
    caps: Optional[Dict[str, Any]] = None
    if args.urdf:
        from domain.robot import Robot
        from service.twinsync import PlantModel

        robot = Robot(args.urdf, "StubRobot")
        plant = PlantModel(
            joint_bias_deg=args.bias_all,
            scale_error=args.scale,
            noise_deg=args.noise,
            lag_tau=args.lag_tau,
        )
    if args.no_feedback:
        caps = {"feedback": False}

    server = ControllerStubServer(
        args.host,
        args.port,
        robot=robot,
        plant=plant,
        dof=args.dof,
        bias=args.bias_all,
        scale=args.scale,
        noise_deg=args.noise,
        caps=caps,
        latency=0.0,
    )
    port = server.start()
    if robot is not None:
        mode = "合成真机模型（PlantLink）"
    else:
        mode = "回显 + 线性误差"
    print(
        f"打桩控制器已启动：{server.address}（core-socket v{server.state.api}，"
        f"dof={server.state.dof}，回读模型={mode}）"
    )
    print("  客户端示例：SocketLink('127.0.0.1', %d)  " "停止：Ctrl-C" % port)
    if args.print_address:
        print(f"ADDRESS {server.address}")
    sys.stdout.flush()

    try:
        while not (args.max_clients and server.state.clients_done >= args.max_clients):
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n收到中断，关闭打桩控制器")
    finally:
        server.stop()
    return 0


if __name__ == "__main__":  # pragma: no cover - 手动运行
    raise SystemExit(main())
