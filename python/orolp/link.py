"""控制器连接层：把离线生成的程序部署到真实的（或仿真的）控制器。

离线编程的最后一公里——生成代码只是中间产物，最终要落到控制器上运行，并且
能在现场读回实际位置做比对调试。这一层把"传输方式"抽象出来，与具体的机器
人品牌、运动学解耦：

* :class:`FileLink` —— 离线导出：把代码写入部署目录并登记清单，无需任何控制器，
  适合交付、存档、离线评审。
* :class:`SimLink` —— 用进程内运动学充当"控制器"，支持回读实际关节角，用于
  联调与在线调试（无需真机即可跑通整条 debug 链路）。
* 真实控制器（SSH/SFTP、Socket、厂商标定协议）日后按同一 :class:`ControllerLink`
  接口接入即可，上层（deploy / debug）无需改动。

:class:`~orolp.program.Program.deploy` 是把程序投到某个连接的便捷入口。
"""

from __future__ import annotations

import json
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

__all__ = [
    "Deployment",
    "ControllerLink",
    "FileLink",
    "SimLink",
    "LinkError",
]


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
        ext = str(meta.get("ext", "src")).lstrip(".")
        safe = "".join(c if (c.isalnum() or c in "-_.") else "_" for c in name)
        code_path = self.deploy_dir / f"{safe}.{ext}"
        code_path.write_text(code, encoding="utf-8")

        manifest = {
            "name": name,
            "ext": ext,
            "post_processor": meta.get("post_processor"),
            "brand": meta.get("brand"),
            "robot_name": meta.get("robot_name"),
            "lines": code.count("\n") + (1 if code and not code.endswith("\n") else 0),
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
