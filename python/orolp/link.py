"""控制器连接层：把离线生成的程序部署到真实的（或仿真的）控制器。

离线编程的最后一公里——生成代码只是中间产物，最终要落到控制器上运行，并且
能在现场读回实际位置做比对调试。这一层把"传输方式"抽象出来，与具体的机器
人品牌、运动学解耦：

* :class:`FileLink` —— 离线导出：把代码写入部署目录并登记清单，无需任何控制器，
  适合交付、存档、离线评审。
* :class:`SimLink` —— 用进程内运动学充当"控制器"，支持回读实际关节角，用于
  联调与在线调试（无需真机即可跑通整条 debug 链路）。
* :class:`SocketLink` —— **真机**：TCP socket + 单行 JSON 协议（``orolp-socket``
  v1），支持程序下发、程序取回、关节回读与下发、位姿回读。它是把"仿真 vs 真机
  偏差闭环"（见 :mod:`orolp.twinsync`）从合成模型变成**现场数据**的那一环。
* :class:`SSHLink` —— **真机**（覆盖不支持 socket 的控制器）：SFTP 传程序文件 +
  SSH exec 远程执行（触发现场加载命令、取状态等），文件真实落在控制器文件系统上，
  "上传的到底是不是我发的那份"用 :meth:`SSHLink.fetch_program` 原文核对。
* 其余真机形态（厂商标定协议）按同一 :class:`ControllerLink` 接口接入
  即可，上层（deploy / debug / 孪生）无需改动。

:class:`~orolp.program.Program.deploy` 是把程序投到某个连接的便捷入口。

配合 :mod:`orolp.linkstub`（本机回环打桩控制器）可以在**没有真机**的情况下把
"部署 → 播放 → 回读 → 偏差"整条链路真跑一遍：客户端走的是真 socket，只是对端
是打桩服务端。
"""

from __future__ import annotations

import base64
import binascii
import json
import socket
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from .i18n import tr
from .pose import Pose

__all__ = [
    "Deployment",
    "ControllerLink",
    "FileLink",
    "SimLink",
    "SocketLink",
    "SSHLink",
    "CommandResult",
    "LinkError",
    "SOCKET_API_VERSION",
    "DEFAULT_SOCKET_PORT",
    "DEFAULT_SOCKET_TIMEOUT",
    "DEFAULT_SSH_PORT",
    "DEFAULT_SSH_TIMEOUT",
]


#: ``orolp-socket`` 协议版本。客户端与打桩/真机服务端必须**相等**才握手成功——
#: 协议一旦不兼容就该在握手时立刻失败，而不是等某条指令发出去才莫名其妙对不上。
SOCKET_API_VERSION = 1

#: 默认端口：UR 的控制端口族（30001 主 / 30002 次 / 30004 RTDE）里最常用于"下发
#: 脚本"的口。连哪台机器由调用方给，这里只是取一个合理的默认值。
DEFAULT_SOCKET_PORT = 30002

#: 单次请求的默认超时（秒）。真机网络抖动时宁可报错，也不要让 GUI 卡死。
DEFAULT_SOCKET_TIMEOUT = 5.0

#: 单行报文的长度上限（字节）。程序代码走 base64，几万行的程序也远在这个上限内；
#: 设上限是为了"对端疯了"时不把内存吃光。
SOCKET_MAX_LINE = 16 * 1024 * 1024

#: SSH 默认端口（IANA 标准端口，现场几乎不会改）。
DEFAULT_SSH_PORT = 22

#: SSH 单次操作（连接/SFTP/exec）的默认超时（秒）。
DEFAULT_SSH_TIMEOUT = 10.0

#: SSH 程序清单文件后缀（与 :class:`FileLink` 同一套 manifest 口径）。
_SSH_MANIFEST_SUFFIX = ".manifest.json"


def _safe_name(name: str) -> str:
    """把程序名净化成可安全用于远端文件名/键的形态。

    与 :class:`FileLink` 同一规则（非字母数字与 ``-_.`` 一律替换成 ``_``），另外
    去掉前导点并兜底成 ``program``——``".."`` 这类名字不该变成 ``...src``。
    """
    safe = "".join(c if (c.isalnum() or c in "-_.") else "_" for c in str(name))
    return safe.lstrip(".") or "program"


def _brief(raw: bytes, limit: int = 80) -> str:
    """把一段（可能非法的）原始报文压成一行，用于报错信息。"""
    text = raw[:limit].decode("utf-8", "replace")
    return text + ("…" if len(raw) > limit else "")


def _json_safe(meta: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """把 metadata 收敛成可 JSON 序列化的标量字典。

    清单里常见 ``pathlib.Path``、numpy 标量、自定义对象；直接 ``json.dumps`` 会在
    **部署那一刻**抛异常（用户视角＝"部署按钮坏了"）。这里统一转成字符串，
    "能发出去"优先于"原样保留"。
    """
    out: Dict[str, Any] = {}
    for key, value in (meta or {}).items():
        if value is None or isinstance(value, (str, int, float, bool)):
            out[str(key)] = value
        else:
            out[str(key)] = str(value)
    return out


def _line_count(code: str) -> int:
    """代码行数（末行无换行也算一行），清单里的口径只此一处。"""
    text = code or ""
    if not text:
        return 0
    return text.count("\n") + (0 if text.endswith("\n") else 1)


def _client_banner() -> str:
    """客户端自述（握手用）。包版本尽量取到，取不到也不影响连接。"""
    try:
        from . import __version__ as _v
    except Exception:  # pragma: no cover - 极端导入顺序下的兜底
        _v = "unknown"
    return f"orolp {_v}"


class LinkError(RuntimeError):
    """控制器连接或部署失败。"""


@dataclass
class Deployment:
    """一次成功（或记录失败）的程序部署结果。

    Attributes:
        program_name: 部署的程序名
        post_processor: 用的后处理器 full_name
        brand: 推断出的品牌 id（可能为 ``None``）
        code: 生成的控制器源码
        handle: 控制器侧句柄（FileLink 为文件路径，SimLink 为 ``sim://<name>``）
        deployed_at: 部署时刻（Unix 时间戳，秒）
        ok: 是否成功
        notes: 附加说明（如降级提示、回读能力）
    """

    program_name: str
    post_processor: Optional[str]
    brand: Optional[str]
    code: str
    handle: str
    deployed_at: float = field(default_factory=time.time)
    ok: bool = True
    notes: str = ""


class ControllerLink(ABC):
    """控制器连接的统一抽象。

    子类只需实现传输细节；"读回实际关节角"与"下关节指令"是可选的——离线导出
    场景（:class:`FileLink`）不需要，在线调试场景（:class:`SimLink`）需要。

    属性约定
    --------
    * ``connected`` —— 是否已建立连接
    * ``is_actual_feedback`` —— 是否支持回读实际关节角（决定在线调试能否用）
    """

    #: 协议/连接类型名，用于日志与清单
    protocol: str = "abstract"

    def __init__(self) -> None:
        self._connected = False

    # --------------------------- 连接管理 --------------------------- #

    @property
    def connected(self) -> bool:
        """是否已建立连接。"""
        return self._connected

    def connect(self) -> None:
        """建立连接。子类实现具体握手逻辑。"""
        self._connected = True

    def disconnect(self) -> None:
        """断开连接（释放资源）。"""
        self._connected = False

    def _require_connected(self) -> None:
        if not self._connected:
            raise LinkError(f"{type(self).__name__} 未连接；请先调用 connect()")

    # --------------------------- 部署接口 --------------------------- #

    @abstractmethod
    def upload_program(
        self, name: str, code: str, metadata: Optional[Dict[str, Any]] = None
    ) -> str:
        """把生成的代码部署到控制器。

        Args:
            name: 程序名
            code: 生成的控制器源码
            metadata: 附加信息（程序名、品牌、扩展名等）

        Returns:
            控制器侧句柄（路径 / id 等），用于后续查询或回读
        """

    @abstractmethod
    def list_programs(self) -> List[str]:
        """列出已部署的程序名。"""

    # --------------------------- 回读 / 指令（可选） --------------------------- #

    @property
    def is_actual_feedback(self) -> bool:
        """是否支持回读实际关节角。默认 ``False``（离线导出场景）。"""
        return False

    def read_joints(self) -> Optional[np.ndarray]:
        """读回控制器中机器人的实际关节角（deg）。

        不支持回读时返回 ``None``。调试会话据此判断是否具备在线比对能力。
        """
        return None

    def set_joints(self, joints: Any) -> None:
        """向控制器下发关节角指令（deg）。

        用于在线调试的播放 / 单步 / 示教。不支持时（离线导出）静默忽略——
        部署本身不需要这一步。
        """
        return None

    def get_pose(self) -> Optional["object"]:  # 返回 orolp.pose.Pose
        """回读当前 TCP 位姿（若有运动学支撑）。默认 ``None``。"""
        return None

    # -------------------- 孪生 / 云协作能力（可选，v1.0.3） -------------------- #

    #: 该连接是否具备云端能力——上层（孪生层）据此决定要不要上传轨迹。
    #: 默认 ``False``：离线落盘（:class:`FileLink`）与进程内仿真（:class:`SimLink`）
    #: 都把数据留在本机。
    supports_cloud: bool = False

    def publish_trace(self, trace: "object") -> Optional[str]:
        """把一条孪生轨迹上传到该连接（默认不支持，返回 ``None``）。

        参数是 :class:`~orolp.twinsync.TwinTrace`，但这里用鸭子类型而不是硬导入——
        ``link`` 层不依赖孪生层的具体类型。
        """
        return None

    def fetch_trace(self, name: str) -> Optional["object"]:
        """从该连接取回一条孪生轨迹（默认不支持，返回 ``None``）。"""
        return None


class FileLink(ControllerLink):
    """离线导出连接：把代码写入部署目录并登记清单。

    不连接任何控制器，因此 ``is_actual_feedback`` 为 ``False``、``read_joints``
    返回 ``None``——它负责把"离线编程产物"落盘交付，在线调试交给 :class:`SimLink`
    或真实控制器。
    """

    protocol = "file"

    def __init__(self, deploy_dir: str | Path):
        """
        Args:
            deploy_dir: 部署目录；不存在时自动创建
        """
        super().__init__()
        self.deploy_dir = Path(deploy_dir)
        self.deploy_dir.mkdir(parents=True, exist_ok=True)
        self._programs: List[str] = []

    @property
    def is_actual_feedback(self) -> bool:
        return False

    def upload_program(
        self, name: str, code: str, metadata: Optional[Dict[str, Any]] = None
    ) -> str:
        self._require_connected()
        meta = dict(metadata or {})
        ext = str(meta.get("ext", "src")).lstrip(".") or "src"
        safe = _safe_name(name)
        code_path = self.deploy_dir / f"{safe}.{ext}"
        code_path.write_text(code, encoding="utf-8")

        manifest = {
            "name": name,
            "ext": ext,
            "post_processor": meta.get("post_processor"),
            "brand": meta.get("brand"),
            "robot_name": meta.get("robot_name"),
            "lines": _line_count(code),
            "bytes": len(code.encode("utf-8")),
            "deployed_at": time.time(),
        }
        (self.deploy_dir / f"{safe}.manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        if name not in self._programs:
            self._programs.append(name)
        return str(code_path)

    def list_programs(self) -> List[str]:
        self._require_connected()
        return list(self._programs)

    def set_joints(self, joints: Any) -> None:
        # 离线导出不需要下发指令，静默忽略
        return None


class SimLink(ControllerLink):
    """仿真连接：用进程内运动学充当"控制器"。

    把给定的 :class:`~orolp.robot.Robot` 当成一台真机——``set_joints`` 改变它的
    关节状态，``read_joints`` / ``get_pose`` 回读实际位置。它让"在线调试"整条
    链路无需真机即可跑通：部署一个程序，再用 :class:`~orolp.debug.DebugSession`
    单步播放、读回偏差、做示教。

    Args:
        robot: 机器人实例（提供正/逆运动学与当前关节状态）
    """

    protocol = "sim"

    def __init__(self, robot: "object"):
        super().__init__()
        self.robot = robot
        self._q: Optional[np.ndarray] = None

    @property
    def is_actual_feedback(self) -> bool:
        return True

    def connect(self) -> None:
        super().connect()
        try:
            self._q = np.asarray(self.robot.get_joints(), dtype=np.float64).ravel().copy()
        except Exception:
            self._q = np.zeros(self.robot.dof)

    def upload_program(
        self, name: str, code: str, metadata: Optional[Dict[str, Any]] = None
    ) -> str:
        self._require_connected()
        # 仿真控制器"烧录"程序：仅登记句柄，不解析代码语义
        if not hasattr(self, "_flashed"):
            self._flashed: Dict[str, str] = {}
        self._flashed[name] = code
        return f"sim://{name}"

    def list_programs(self) -> List[str]:
        self._require_connected()
        return list(getattr(self, "_flashed", {}).keys())

    def set_joints(self, joints: Any) -> None:
        self._require_connected()
        q = np.asarray(joints, dtype=np.float64).ravel()
        self._q = q.copy()
        try:
            self.robot.set_joints(q)
        except Exception:
            pass  # 机器人可能不支持 set_joints；仿真连接以自身 _q 为准

    def read_joints(self) -> Optional[np.ndarray]:
        self._require_connected()
        return None if self._q is None else self._q.copy()

    def get_pose(self):
        self._require_connected()
        if self._q is None:
            return None
        return self.robot.forward_kinematics(self._q)


class SocketLink(ControllerLink):
    """真机控制器连接：TCP socket + 单行 JSON 协议（``orolp-socket`` v1）。

    为什么是"行协议"而不是直接 embed 某个厂商的 SDK：现场要连的控制器五花八门
    （UR / FANUC / KUKA / 汇川 / 自研），共同点只有"有个网口"。把传输收敛成
    **一问一答的单行 JSON**，控制器侧只需要一小段转发（UR 用 URScript 的
    ``socket_open``/``read``/``write``，或 PC 侧代理把厂商原生协议翻译过来），
    上层（deploy / debug / 孪生）就完全不必知道对面是什么牌子。

    协议
    ----
    一问一答，**单行 UTF-8 JSON**、以 ``\\n`` 结尾（与 TCP 流天然对齐）::

        客户端 → {"op":"hello","api":1,"client":"orolp 1.0.5"}
        服务端 → {"ok":true,"api":1,"server":"orolp.ControllerStubServer","dof":6,
                  "caps":{"programs":true,"feedback":true,"pose":true}}
        客户端 → {"op":"upload","name":"pick","ext":"script","code_b64":"…","meta":{…}}
        服务端 → {"ok":true,"handle":"sock://127.0.0.1:30002/programs/pick.script"}
        客户端 → {"op":"list"}            → {"ok":true,"programs":["pick"]}
        客户端 → {"op":"fetch","name":"pick","ext":"script"}
                                        → {"ok":true,"code_b64":"…"}（不存在则 code_b64=null）
        客户端 → {"op":"set_joints","joints":[…],"at_time":0.42} → {"ok":true}
        客户端 → {"op":"read_joints"}    → {"ok":true,"joints":[…]}
        客户端 → {"op":"get_pose"}       → {"ok":true,"pose":[x,y,z,rx,ry,rz]} 或 null
        客户端 → {"op":"bye"}            → {"ok":true}

    几条刻意的设计选择：

    * **程序代码走 base64**：程序里有换行、中文注释、各家控制语言的引号转义规则，
      塞进单行 JSON 字符串只会给自己找麻烦。base64 保证"发出去什么、对面收到就
      是什么"，顺带让报文的编码问题（CRLF / GBK 控制器）不再是问题。
    * **能力要协商**：``caps`` 里没有 ``feedback`` 就是"这台控制器不提供关节回读"，
      此时 :meth:`read_joints` 如实返回 ``None``——**不用本机运动学伪造一份"实际
      位置"**，那会让孪生偏差曲线变成自欺欺人。
    * **两种失败要分开**：网络/协议级故障（连不上、超时、乱码、对端关闭）会**立刻
      把连接标记为未连接**并把底层 socket 关掉——把"其实已经断了"的连接留在
      ``connected=True`` 是更坏的结果；而服务端业务性拒绝（``ok:false``）只抛异常、
      不断连接（一次程序名写错不该废掉一台机器的会话）。
    * **``at_time`` 会被转发**：与合成真机 :class:`~orolp.twinsync.PlantLink` 对齐，
      :meth:`set_joints` 接受 ``at_time=`` 并原样送到对面，服务端可以据此建模伺服
      滞后；不关心它的服务端忽略即可。

    Args:
        host: 控制器（或转发桩）地址
        port: 端口，默认 :data:`DEFAULT_SOCKET_PORT`
        timeout: 单次请求超时（秒）
        robot: 可选机器人实例——服务端不提供位姿回读时，用它由回读的关节角正解 TCP
        name: 连接标识（日志/句柄用），默认 ``host:port``

    注意:
        **本实现未在真机上验证过**（开发环境无机器人）。随包提供的
        :class:`~orolp.linkstub.ControllerStubServer` 是本机回环打桩服务端，
        用来把"部署 → 播放 → 回读 → 偏差"整条链路真跑一遍；接真机前请先用它验收，
        再准备好控制器侧的同协议转发。
    """

    protocol = "socket"

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = DEFAULT_SOCKET_PORT,
        *,
        timeout: float = DEFAULT_SOCKET_TIMEOUT,
        robot: "object" = None,
        name: str = "",
    ) -> None:
        super().__init__()
        self.host = str(host)
        self.port = int(port)
        self.timeout = float(timeout)
        self.robot = robot
        self.name = name or f"{self.host}:{self.port}"
        self._sock: Optional[socket.socket] = None
        self._buf = bytearray()
        self._caps: Dict[str, bool] = {}
        self._info: Dict[str, Any] = {}
        self._dof: Optional[int] = None
        self._uploaded: List[str] = []
        self._requests = 0

    # --------------------------- 连接管理 --------------------------- #

    def connect(self) -> None:
        """建立 TCP 连接并完成握手（版本 + 能力协商）。

        Raises:
            LinkError: 连不上、握手超时/乱码、或协议版本不匹配
        """
        if self._sock is not None or self._connected:
            self._teardown()  # 重复 connect 视为"重连"，不叠第二条连接
        try:
            sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        except OSError as exc:
            raise LinkError(
                f"{self.name}: 连接失败（{type(exc).__name__}: {exc}）——"
                f"确认控制器/打桩服务端已启动，或先在 PC 上跑 "
                f"`python -m orolp.linkstub --port {self.port}`"
            ) from exc
        sock.settimeout(self.timeout)
        try:  # 小报文争这一下延迟不值得，但对"一问一答"的手感影响明显
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:  # pragma: no cover - 平台差异，失败不影响功能
            pass
        self._sock = sock
        self._buf = bytearray()

        try:
            resp = self._exchange(
                {"op": "hello", "api": SOCKET_API_VERSION, "client": _client_banner()}
            )
        except LinkError:
            self._teardown()
            raise

        api = resp.get("api")
        if api != SOCKET_API_VERSION:
            self._teardown()
            raise LinkError(
                f"{self.name}: 协议版本不匹配（客户端 orolp-socket v{SOCKET_API_VERSION}，"
                f"服务端 v{api}）——宁可握手就失败，也不要按错语义跑出错误轨迹"
            )
        caps = resp.get("caps")
        self._caps = (
            {str(k): bool(v) for k, v in caps.items()} if isinstance(caps, dict) else {}
        )
        self._info = {k: v for k, v in resp.items() if k not in ("ok", "api", "caps")}
        try:
            self._dof = None if resp.get("dof") is None else int(resp["dof"])
        except (TypeError, ValueError):
            self._dof = None
        self._connected = True

    def disconnect(self) -> None:
        """发 ``bye``（尽力而为）后关闭连接。对端已经走了也算断开成功。"""
        if self._sock is not None and self._connected:
            try:
                self._exchange({"op": "bye"})
            except LinkError:
                pass
        self._teardown()

    def _teardown(self) -> None:
        """关掉底层 socket 并把状态置为未连接（不抛异常，可重复调用）。"""
        sock, self._sock = self._sock, None
        self._buf = bytearray()
        self._connected = False
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:  # pragma: no cover
                pass

    def __enter__(self) -> "SocketLink":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        self.disconnect()
        return False  # 不吞异常

    # --------------------------- 传输 --------------------------- #

    def _readline(self) -> bytes:
        """从 socket 缓冲里读出一整行（可能跨多次 ``recv``）。"""
        sock = self._sock
        if sock is None:
            raise LinkError("未连接")
        while b"\n" not in self._buf:
            if len(self._buf) > SOCKET_MAX_LINE:
                raise LinkError(f"响应超过 {SOCKET_MAX_LINE} 字节仍未结束")
            try:
                chunk = sock.recv(65536)
            except socket.timeout as exc:
                raise LinkError(f"等待响应超时（{self.timeout}s）") from exc
            except OSError as exc:
                raise LinkError(f"读取响应失败（{type(exc).__name__}: {exc}）") from exc
            if not chunk:
                raise LinkError("连接被对端关闭（读到 EOF）")
            self._buf += chunk
        line, _, rest = bytes(self._buf).partition(b"\n")
        self._buf = bytearray(rest)
        return line

    def _exchange(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """发一条请求并读回一条响应（**不做连接状态检查**，握手期也用）。"""
        if self._sock is None:
            raise LinkError(f"{self.name}: 未连接；请先调用 connect()")
        op = payload.get("op")
        data = (
            json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
            + b"\n"
        )
        if len(data) > SOCKET_MAX_LINE:
            raise LinkError(f"{self.name}: {op} 请求过大（{len(data)} 字节）")
        self._requests += 1
        try:
            self._sock.sendall(data)
            raw = self._readline()
        except LinkError as exc:
            self._teardown()
            raise LinkError(f"{self.name}: {exc}") from exc
        except OSError as exc:
            self._teardown()
            raise LinkError(
                f"{self.name}: 发送/接收 {op} 失败（{type(exc).__name__}: {exc}）"
            ) from exc

        try:
            resp = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, ValueError) as exc:
            self._teardown()
            raise LinkError(f"{self.name}: {op} 的响应不是合法 JSON：{_brief(raw)}") from exc
        if not isinstance(resp, dict) or "ok" not in resp:
            self._teardown()
            raise LinkError(f"{self.name}: {op} 的响应缺少 ok 字段：{_brief(raw)}")
        if not resp.get("ok"):
            # 业务性拒绝：链路本身还活着，不要断连——一次笔误不该废掉整个会话
            raise LinkError(
                f"{self.name}: 服务端拒绝 {op}：{resp.get('error') or '未说明原因'}"
            )
        return resp

    def _op(self, op: str, **fields: Any) -> Dict[str, Any]:
        """带连接状态检查的请求入口。"""
        self._require_connected()
        payload: Dict[str, Any] = {"op": op}
        payload.update(fields)
        return self._exchange(payload)

    # --------------------------- 部署接口 --------------------------- #

    def upload_program(
        self, name: str, code: str, metadata: Optional[Dict[str, Any]] = None
    ) -> str:
        """把生成的控制器代码下发到控制器。

        Returns:
            控制器侧句柄 ``sock://<host:port>/programs/<safe>.<ext>``
        """
        meta = dict(metadata or {})
        ext = str(meta.get("ext", "src")).lstrip(".") or "src"
        safe = _safe_name(name)
        resp = self._op(
            "upload",
            name=str(name),
            ext=ext,
            code_b64=base64.b64encode(str(code).encode("utf-8")).decode("ascii"),
            meta=_json_safe(meta),
        )
        handle = resp.get("handle") or f"sock://{self.name}/programs/{safe}.{ext}"
        if name not in self._uploaded:
            self._uploaded.append(name)
        return str(handle)

    def list_programs(self) -> List[str]:
        """列出控制器上已部署的程序名。"""
        resp = self._op("list")
        programs = resp.get("programs")
        if programs is None:
            return []
        if not isinstance(programs, (list, tuple)):
            raise LinkError(f"{self.name}: list 返回值不是数组：{programs!r}")
        return [str(p) for p in programs]

    # --------------------------- 回读 / 指令 --------------------------- #

    @property
    def is_actual_feedback(self) -> bool:
        """是否有真实回读能力——由**握手协商**决定，而不是类常量。

        控制器的能力是控制器的事：老控制器可能不给关节回读，打桩服务端也可以
        故意关掉它来验收上层行为。未连接时一律 ``False``。
        """
        return bool(self._caps.get("feedback", False))

    def read_joints(self) -> Optional[np.ndarray]:
        """回读控制器里机器人的**实际**关节角（deg）。

        控制器不支持回读时返回 ``None``（不伪造）。返回值要么是数值数组，要么抛错：
        关节数对不上握手声明的 ``dof`` 说明对面给的是一份没法用的数据，
        悄悄返回它只会把问题推到偏差曲线上。
        """
        self._require_connected()
        if not self._caps.get("feedback"):
            return None
        resp = self._op("read_joints")
        joints = resp.get("joints")
        if joints is None:
            return None
        try:
            q = np.asarray(joints, dtype=np.float64).ravel()
        except (TypeError, ValueError) as exc:
            raise LinkError(f"{self.name}: read_joints 返回值不是数值数组") from exc
        if q.size == 0:
            raise LinkError(f"{self.name}: read_joints 返回空数组")
        if self._dof and q.size != self._dof:
            raise LinkError(
                f"{self.name}: 关节数不符（收到 {q.size}，握手声明 {self._dof}）"
            )
        return q

    def set_joints(self, joints: Any, at_time: Optional[float] = None) -> None:
        """向控制器下发关节角指令（deg）。

        Args:
            joints: 关节角（deg）
            at_time: 该指令在程序时间轴上的时刻（秒）。原样转发给服务端——真机可以
                据此做时序对齐/滞后补偿，不关心的服务端忽略即可。
        """
        self._require_connected()
        q = np.asarray(joints, dtype=np.float64).ravel()
        if q.size == 0:
            raise LinkError(f"{self.name}: set_joints 收到空关节数组")
        fields: Dict[str, Any] = {"joints": [float(v) for v in q]}
        if at_time is not None:
            fields["at_time"] = float(at_time)
        self._op("set_joints", **fields)

    def get_pose(self) -> Optional[Pose]:
        """回读当前 TCP 位姿。

        优先问控制器（``caps.pose``）；控制器不提供位姿时，若构造时给了 ``robot``，
        则用回读的关节角在本机正解。两者都不具备时返回 ``None``。
        """
        self._require_connected()
        if self._caps.get("pose"):
            resp = self._op("get_pose")
            pose = resp.get("pose")
            if pose is not None:
                try:
                    vals = [float(v) for v in pose]
                except (TypeError, ValueError) as exc:
                    raise LinkError(f"{self.name}: get_pose 返回值不是数值序列") from exc
                if len(vals) != 6:
                    raise LinkError(
                        f"{self.name}: get_pose 返回 {len(vals)} 个数，期望 6（x,y,z,rx,ry,rz）"
                    )
                return Pose.from_xyz_rpy(*vals)
        if self.robot is not None:
            q = self.read_joints()
            if q is not None:
                return self.robot.forward_kinematics(q)
        return None

    # --------------------------- 程序取回（真机特有） --------------------------- #

    def fetch_program(self, name: str, ext: str = "src") -> Optional[str]:
        """把控制器上的程序原文取回（用于"上传的到底是不是我发的那份"核对）。

        Returns:
            代码文本；控制器上不存在该程序时返回 ``None``
        """
        resp = self._op("fetch", name=str(name), ext=str(ext).lstrip(".") or "src")
        raw = resp.get("code_b64")
        if raw is None:
            return None  # 控制器上没这个程序；注意空程序（""）是合法内容，别一起吞掉
        if raw == "":
            return ""
        try:
            return base64.b64decode(str(raw)).decode("utf-8")
        except (binascii.Error, UnicodeDecodeError) as exc:
            raise LinkError(f"{self.name}: fetch 返回的代码不是合法 base64/UTF-8") from exc

    # --------------------------- 自省 --------------------------- #

    @property
    def dof(self) -> Optional[int]:
        """握手时服务端声明的关节数（未声明为 ``None``）。"""
        return self._dof

    @property
    def caps(self) -> Dict[str, bool]:
        """服务端协商出来的能力表（副本）。"""
        return dict(self._caps)

    @property
    def server_info(self) -> Dict[str, Any]:
        """握手响应里除 ``ok``/``api``/``caps`` 之外的字段（副本）。"""
        return dict(self._info)

    @property
    def requests(self) -> int:
        """本次连接上发出的请求数（含握手与 ``bye``）。"""
        return self._requests

    @property
    def uploaded(self) -> List[str]:
        """本次连接上下发过的程序名（副本）。控制器上的清单以 :meth:`list_programs` 为准。"""
        return list(self._uploaded)

    def describe(self) -> str:
        """一行可读状态，用于日志与 GUI 状态栏（随界面语言）。"""
        state = tr("已连接") if self._connected else tr("未连接")
        caps = ",".join(k for k, v in sorted(self._caps.items()) if v) or "-"
        server = self._info.get("server") or self._info.get("name") or tr("未知")
        dof = "-" if self._dof is None else str(self._dof)
        return (
            f"SocketLink[{self.name}] {state} {tr('服务端')}={server} "
            f"api=v{SOCKET_API_VERSION} dof={dof} {tr('能力')}={caps} {tr('请求')}={self._requests}"
        )

    def __repr__(self) -> str:  # pragma: no cover - 纯展示
        return f"<SocketLink {self.name} {'connected' if self._connected else 'disconnected'}>"


@dataclass
class CommandResult:
    """一次 SSH 远程命令的执行结果。

    Attributes:
        command: 执行的命令文本
        exit_code: 远端退出码（0 = 成功）
        stdout: 标准输出（UTF-8 解码，非法字节替换）
        stderr: 标准错误（同上）
    """

    command: str
    exit_code: int
    stdout: str
    stderr: str

    @property
    def ok(self) -> bool:
        """退出码是否为 0。"""
        return self.exit_code == 0

    def __str__(self) -> str:  # pragma: no cover - 展示便捷
        mark = "ok" if self.ok else f"exit {self.exit_code}"
        out = self.stdout.strip() or self.stderr.strip()
        brief = out[:80] + ("…" if len(out) > 80 else "")
        return f"$ {self.command} [{mark}] {brief}"


class SSHLink(ControllerLink):
    """真机控制器连接（SSH 形态）：SFTP 传程序文件 + SSH exec 远程执行。

    为什么在 :class:`SocketLink` 之外还要一个 SSH 形态：**相当一部分控制器不支持
    开放 TCP socket 给上位机自定协议，但都带着 SSH**——FANUC 的 Karel 辛苦打通
    FTP、KUKA 的 KRC4 本体就是个 Linux 盒子、国产控制器常见的"控制器 PC 一体机"
    也是。对这类机器，"把程序文件放到控制器文件系统 + 让现场用一条命令加载它"
    就是真实的工作流，SFTP + exec 恰好覆盖。

    与 :class:`FileLink` 的关键区别：文件**真实落在远端**，清单（manifest）也是，
    所以 ``list_programs`` 读的是远端清单而不是本机内存；与 :class:`SocketLink` 的
    关键区别：没有常驻对端服务，因此没有握手/能力协商——回读能力一律
    ``False``，回读链路（孪生偏差）仍走 Socket/厂商协议。

    两种失败语义与 :class:`SocketLink` 同契约：

    * **网络/认证级故障**（连不上、认证被拒、超时、连接被对端关闭）→ 抛
      :class:`LinkError` 并把连接标记为**未连接**——"其实已经断了"的连接留在
      ``connected=True`` 是更坏的结果；
    * **业务级结果**（远端命令退出码非 0、程序不存在、清单缺字段）→ 抛异常或如实
      返回，**不断连接**——一条命令写错不该废掉一台机器的会话。

    Args:
        host: 控制器（或跳板机）地址
        port: SSH 端口，默认 :data:`DEFAULT_SSH_PORT`
        username: 登录用户
        password: 密码认证（与 ``key_filename`` 二选一或都给）
        key_filename: 私钥文件路径（``~`` 可用）
        timeout: 单次操作超时（秒）
        remote_dir: 程序存放的远端目录；不存在时 ``connect()`` 会尝试
            ``mkdir -p`` 创建，创建失败视为连接失败（目录都建不出来说明权限配错）
        name: 连接标识（日志/句柄用），默认 ``ssh://user@host:port``

    注意:
        **本实现未在真机上验证过**（开发环境无机器人）。随包提供的
        :class:`~orolp.sshstub.SSHStubServer` 是本机回环打桩服务端（真 SSH：
        paramiko 客户端 ↔ paramiko 服务端），用来把"部署 → 取回核对 → 远程加载"
        整条链路真跑一遍；接真机前请先用它验收，再按现场核对主机密钥/路径/权限。
    """

    protocol = "ssh"

    def __init__(
        self,
        host: str,
        port: int = DEFAULT_SSH_PORT,
        *,
        username: str = "root",
        password: Optional[str] = None,
        key_filename: Optional[str] = None,
        timeout: float = DEFAULT_SSH_TIMEOUT,
        remote_dir: str = "/tmp/orolp_programs",
        name: str = "",
    ) -> None:
        super().__init__()
        self.host = str(host)
        self.port = int(port)
        self.username = str(username)
        self.password = password
        self.key_filename = key_filename
        self.timeout = float(timeout)
        self.remote_dir = str(remote_dir)
        self.name = name or f"ssh://{self.username}@{self.host}:{self.port}"
        self._client: Any = None
        self._sftp: Any = None
        self._uploaded: List[str] = []
        self._executions = 0

    # --------------------------- 连接管理 --------------------------- #

    def connect(self) -> None:
        """建立 SSH 连接 + SFTP 子系统，并确保远端程序目录存在。

        Raises:
            LinkError: 连不上、认证被拒、paramiko 未安装、或远端目录建不出来
        """
        if self._client is not None or self._connected:
            self._teardown()  # 重复 connect 视为"重连"，不叠第二条连接
        try:
            paramiko = self._import_paramiko()
        except ImportError as exc:
            raise LinkError(
                f"{self.name}: SSH 连接需要 paramiko——pip install paramiko"
            ) from exc

        client = paramiko.SSHClient()
        # 现场第一次连接主机密钥必然"未知名"，逐台人工确认不适合批量部署；
        # 打桩/验收场景默认自动添加，接真机时可通过 host_keys 预登记收紧。
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            client.connect(
                self.host,
                port=self.port,
                username=self.username,
                password=self.password,
                key_filename=self.key_filename,
                timeout=self.timeout,
                banner_timeout=self.timeout,
                auth_timeout=self.timeout,
                allow_agent=False,
                look_for_keys=False,
            )
        except Exception as exc:
            try:
                client.close()
            except Exception:  # pragma: no cover
                pass
            raise LinkError(
                f"{self.name}: SSH 连接/认证失败（{type(exc).__name__}: {exc}）——"
                f"确认地址、账号、密码/私钥与 SSH 服务已开启"
            ) from exc

        self._client = client
        try:
            # 远端目录不存在就建：mkdir -p 已存在时成功，权限不够时把输出带回来
            result = self._exec("mkdir -p", ensure_dir=self.remote_dir)
            if not result.ok:
                raise RuntimeError(
                    f"mkdir -p 退出码 {result.exit_code}：{result.stderr.strip()}"
                )
            sftp = client.open_sftp()
            sftp.chdir(self.remote_dir)
        except Exception as exc:
            detail = str(exc)
            self._client = None
            try:
                client.close()
            except Exception:  # pragma: no cover
                pass
            raise LinkError(
                f"{self.name}: 远端程序目录不可用（{self.remote_dir}）：{detail}"
            ) from exc
        self._sftp = sftp
        self._connected = True

    @staticmethod
    def _import_paramiko():
        """paramiko 是 SSHLink 的**软依赖**：不在 orolp 顶层 import，
        只有真正用到 SSH 连接时才需要装。"""
        import paramiko  # noqa: PLC0415 - 刻意延迟导入

        return paramiko

    def disconnect(self) -> None:
        """关闭 SFTP 与 SSH 连接（幂等；对端已走也算断开成功）。"""
        self._teardown()

    def _teardown(self) -> None:
        """关掉底层资源并置为未连接（不抛异常，可重复调用）。"""
        sftp, self._sftp = self._sftp, None
        client, self._client = self._client, None
        self._connected = False
        for resource in (sftp, client):
            if resource is None:
                continue
            try:
                resource.close()
            except Exception:  # pragma: no cover - 关闭失败不影响断开结果
                pass

    def __enter__(self) -> "SSHLink":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        self.disconnect()
        return False  # 不吞异常

    def _require_transport(self) -> Any:
        """返回 SFTP 会话；网络级故障在这里统一翻译成"断开 + LinkError"。"""
        self._require_connected()
        return self._sftp

    def _network_fail(self, op: str, exc: Exception) -> LinkError:
        """把底层异常翻译成 LinkError 并断开（网络/协议级故障语义）。"""
        self._teardown()
        return LinkError(f"{self.name}: {op} 失败（{type(exc).__name__}: {exc}）")

    # --------------------------- 远程执行 --------------------------- #

    def _exec(
        self,
        command: str,
        cwd: Optional[str] = None,
        ensure_dir: Optional[str] = None,
    ) -> CommandResult:
        """exec 通道跑一条命令并收集 stdout/stderr/退出码。

        建目录特例：``ensure_dir`` 给了就执行 ``mkdir -p <dir>``（连接期建目录用）。
        """
        client = self._client
        if client is None:
            raise LinkError(f"{self.name}: 未连接；请先调用 connect()")
        cmd = command
        if ensure_dir is not None:
            cmd = f"mkdir -p {_shell_quote(ensure_dir)}"
        elif cwd:
            cmd = f"cd {_shell_quote(cwd)} && {command}"
        self._executions += 1
        try:
            stdin, stdout, stderr = client.exec_command(cmd, timeout=self.timeout)
            code = stdout.channel.recv_exit_status()
            out = stdout.read().decode("utf-8", "replace")
            err = stderr.read().decode("utf-8", "replace")
        except Exception as exc:
            if ensure_dir is not None:
                # 连接期建目录失败按网络/权限级故障处理（走 connect 的统一翻译）
                raise
            raise self._network_fail(f"执行 {command!r}", exc) from exc
        return CommandResult(command=cmd, exit_code=int(code), stdout=out, stderr=err)

    def run_command(
        self,
        command: str,
        *,
        cwd: Optional[str] = None,
        timeout: Optional[float] = None,
    ) -> CommandResult:
        """在控制器上远程执行一条命令（触发现场加载、取状态、起脚本……）。

        返回退出码与输出；**退出码非 0 是业务结果而不是断连**——命令写错只抛
        :class:`LinkError`（带输出内容），连接本身保持可用。这是"部署经评审的
        程序"工作流的配套能力，GUI 真机连接面板刻意不提供自由命令入口，
        本方法面向脚本化部署流水线。

        Args:
            command: 命令文本（在远端 shell 执行）
            cwd: 工作目录（缺省为登录默认目录）
            timeout: 预留（当前沿连接超时）；远端命令自己卡死由现场超时机制兜底

        Returns:
            :class:`CommandResult`（含 ``exit_code`` / ``stdout`` / ``stderr`` / ``ok``）
        """
        del timeout  # 预留：channel 级单命令超时需独立 watchdog，当前沿用连接超时
        self._require_connected()
        return self._exec(command, cwd=cwd)

    # --------------------------- 部署接口 --------------------------- #

    def upload_program(
        self, name: str, code: str, metadata: Optional[Dict[str, Any]] = None
    ) -> str:
        """通过 SFTP 把生成的控制器代码写到远端程序目录，并写 manifest 清单。

        Returns:
            控制器侧句柄 ``ssh://<user>@<host>:<port><remote_dir>/<safe>.<ext>``
        """
        sftp = self._require_transport()
        meta = dict(metadata or {})
        ext = str(meta.get("ext", "src")).lstrip(".") or "src"
        safe = _safe_name(name)
        code_path = f"{safe}.{ext}"
        try:
            with sftp.open(code_path, "wb") as fh:
                fh.write(str(code).encode("utf-8"))
        except Exception as exc:
            raise self._network_fail(f"上传 {code_path}", exc) from exc

        manifest = {
            "name": name,
            "ext": ext,
            "post_processor": meta.get("post_processor"),
            "brand": meta.get("brand"),
            "robot_name": meta.get("robot_name"),
            "lines": _line_count(code),
            "bytes": len(code.encode("utf-8")),
            "deployed_at": time.time(),
            "meta": _json_safe(meta),  # 完整附加信息（含 Path 之类非标量的字符串化）
        }
        try:
            with sftp.open(f"{safe}{_SSH_MANIFEST_SUFFIX}", "wb") as fh:
                fh.write(
                    json.dumps(manifest, ensure_ascii=False, indent=2).encode("utf-8")
                )
        except Exception as exc:
            raise self._network_fail(f"写清单 {safe}{_SSH_MANIFEST_SUFFIX}", exc) from exc
        if name not in self._uploaded:
            self._uploaded.append(name)
        return f"ssh://{self.username}@{self.host}:{self.port}{self.remote_dir}/{code_path}"

    def list_programs(self) -> List[str]:
        """列出远端程序目录里**真实存在**的程序名（读 manifest 清单）。

        清单缺失的孤儿文件不报错也不上报——``fetch_program`` 才是核对内容的入口；
        但清单 JSON 损坏会抛 :class:`LinkError`（远端文件系统被人动过，该响就响）。
        """
        sftp = self._require_transport()
        try:
            names = sftp.listdir()
        except Exception as exc:
            raise self._network_fail("列目录", exc) from exc
        programs: List[Tuple[float, str]] = []
        for entry in names:
            if not entry.endswith(_SSH_MANIFEST_SUFFIX):
                continue
            try:
                with sftp.open(entry, "rb") as fh:
                    data = json.loads(fh.read().decode("utf-8"))
            except Exception as exc:
                raise self._network_fail(f"读清单 {entry}", exc) from exc
            if not isinstance(data, dict) or "name" not in data:
                raise LinkError(
                    f"{self.name}: 清单 {entry} 缺少 name 字段——远端文件可能被改动"
                )
            try:
                deployed_at = float(data.get("deployed_at") or 0.0)
            except (TypeError, ValueError):
                deployed_at = 0.0
            programs.append((deployed_at, str(data["name"])))
        programs.sort()
        return [name for _ts, name in programs]

    # --------------------------- 程序取回（真机特有） --------------------------- #

    def fetch_program(self, name: str, ext: str = "src") -> Optional[str]:
        """把控制器上的程序原文取回（"上传的到底是不是我发的那份"核对）。

        Returns:
            代码文本；远端不存在该程序时返回 ``None``
        """
        sftp = self._require_transport()
        safe = _safe_name(name)
        path = f"{safe}.{str(ext).lstrip('.') or 'src'}"
        try:
            with sftp.open(path, "rb") as fh:
                raw = fh.read()
        except FileNotFoundError:
            return None
        except IOError as exc:
            # paramiko 对"不存在"的实现间差异：SFTPNoSuchFile 会落到 IOError
            if _is_missing_file(exc):
                return None
            raise self._network_fail(f"取回 {path}", exc) from exc
        except Exception as exc:
            raise self._network_fail(f"取回 {path}", exc) from exc
        try:
            return raw.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise LinkError(
                f"{self.name}: {path} 不是合法 UTF-8——控制器侧可能转存过编码"
            ) from exc

    # --------------------------- 自省 --------------------------- #

    @property
    def uploaded(self) -> List[str]:
        """本次连接上传过的程序名（副本）。远端清单以 :meth:`list_programs` 为准。"""
        return list(self._uploaded)

    @property
    def executions(self) -> int:
        """本次连接执行过的远程命令数（含连接期的 ``mkdir -p``）。"""
        return self._executions

    def describe(self) -> str:
        """一行可读状态，用于日志与 GUI 状态栏（随界面语言）。"""
        state = tr("已连接") if self._connected else tr("未连接")
        return (
            f"SSHLink[{self.name}] {state} {tr('目录')}={self.remote_dir} "
            f"{tr('已传')}={len(self._uploaded)} {tr('执行')}={self._executions}"
        )

    def __repr__(self) -> str:  # pragma: no cover - 纯展示
        return f"<SSHLink {self.name} {'connected' if self._connected else 'disconnected'}>"


def _shell_quote(text: str) -> str:
    """单引号安全引用（远端 shell 用）。比 shlex.quote 更保守：任何特殊字符都包。"""
    return "'" + str(text).replace("'", "'\\''") + "'"


def _is_missing_file(exc: IOError) -> bool:
    """判断 paramiko 的 IOError 是否语义上等于"文件不存在"。

    paramiko 5.x 把 SFTPNoSuchFile 映射成带 errno ENOENT 的 IOError/FabricError
    家族，实现间有差异——用 errno + 文案双保险，避免把"网络断了"误判成"没有文件"
    （那会让 fetch 静默返回 None，"上传的到底是不是我发的那份"就核不成了）。
    """
    if getattr(exc, "errno", None) == 2:  # ENOENT
        return True
    text = str(exc).lower()
    return "no such file" in text or "sftpnosuchfile" in text
