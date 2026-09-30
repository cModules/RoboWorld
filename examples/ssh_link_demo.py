#!/usr/bin/env python3
"""RobotWorld 示例：真机连接（SSH / SFTP）全流程（v1.1.2）。

相当一部分控制器不支持开放 TCP socket 给上位机自定协议，但都带着 SSH——
对这类机器，"把程序文件放到控制器文件系统 + 让现场用一条命令加载它"就是
真实的工作流。本示例走的是**真 SSH**：paramiko 客户端连到跑在本机的
:class:`adapter.sshstub.SSHStubServer`（打桩控制器，paramiko 服务端），
SFTP 文件传输与 exec 远程执行都是真实协议交互：

    起打桩控制器 → SSH 连接（远端程序目录自动 mkdir -p）
      → Program.deploy(...) 经 SFTP 落盘（真实文件 + manifest 清单）
      → 列清单（读的是远端清单，不是本机内存）
      → 取回原文逐字节核对（"上传的到底是不是我发的那份"）
      → SSH exec 触发现场加载（退出码/stdout/stderr 如实回传）
      → 失败演练：密码错 / 程序不存在 / 命令退出码非 0

关于"真机"的诚实说明
--------------------

**本示例没有真机，也没有连过真机**。打桩控制器是 paramiko 服务端 +
真实落盘的 SFTP 根目录，用来把"部署 → 核对 → 远程加载"整条链路在无真机
环境下真跑一遍。接真机时把 :class:`~domain.link.SSHLink` 指向控制器的
SSH 地址、换上现场账号与密钥即可，上层（deploy / 清单 / 核对）一行不改。

运行::

    python examples/ssh_link_demo.py
    # 或未安装 core 时：PYTHONPATH=python python examples/ssh_link_demo.py
"""

from __future__ import annotations

import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
for p in (str(REPO_ROOT), str(REPO_ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)

from domain.pose import Pose
from domain.robot import Robot
from service.program import ProgramBuilder  # noqa: E402
from domain.link import LinkError, SSHLink  # noqa: E402
from adapter.sshstub import SSHStubServer  # noqa: E402

URDF = REPO_ROOT / "assets/models/ur/ur5e/ur5e.urdf"
PROGRAM_NAME = "ssh_pick"
Q0 = [0.0, -30.0, 30.0, 0.0, 60.0, 0.0]


def _hr(title: str) -> None:
    print()
    print("=" * 72)
    print(title)
    print("=" * 72)


def _build_program(robot: Robot):
    return (
        ProgramBuilder(robot)
        .moveJ(Q0, speed=40)
        .moveL(Pose.from_xyz_rpy(500, 0, 300, 0, 0, 0), speed=50, blend=8)
        .moveL(Pose.from_xyz_rpy(500, 200, 300, 0, 0, 0), speed=50, blend=8)
        .build()
    )


def main() -> int:
    robot = Robot(str(URDF), "UR5e")
    program = _build_program(robot)
    root = Path(tempfile.mkdtemp(prefix="core_ssh_"))

    # 打桩控制器：paramiko 服务端 + chroot 式 SFTP 根目录（真实落盘）
    stub = SSHStubServer(root, password="demo")
    print(f"打桩控制器已启动：ssh://127.0.0.1:{stub.port}（SFTP 根 {stub.root}）")

    try:
        # ---------------------------------------------------------------- #
        _hr("[1] SSH 连接：认证 + 远端程序目录自动创建（真 SSH，不是 mock）")
        link = SSHLink(
            "127.0.0.1",
            stub.port,
            username="operator",
            password="demo",
            remote_dir="/programs",
            timeout=10,
        )
        link.connect()
        print(f"  {link.describe()}")
        print("  说明：connect() 会先 mkdir -p 远端程序目录（失败=权限配错，当场报错）")
        print("        再开 SFTP 子系统；SSH 形态无常驻对端服务，没有握手/能力协商——")
        print("        回读能力一律 False，回读链路（孪生偏差）仍走 Socket/厂商协议。")

        # ---------------------------------------------------------------- #
        _hr("[2] 部署程序：Program.deploy(...) → SFTP 落盘 + manifest 清单")
        dep = program.deploy(link, "ur_polyscope", name=PROGRAM_NAME)
        print(f"  句柄      ：{dep.handle}")
        print(f"  后处理器  ：{dep.post_processor}（品牌 {dep.brand}）")
        print(f"  代码规模  ：{len(dep.code.splitlines())} 行 / {len(dep.code.encode('utf-8'))} 字节")
        print(f"  控制器清单：{link.list_programs()}")
        manifest = stub.root / "programs" / f"{PROGRAM_NAME}.manifest.json"
        print(f"  manifest  ：{manifest.name}（真实文件，'name/ext/lines/bytes' 全在内）")

        # ---------------------------------------------------------------- #
        _hr("[3] 线上核对：把程序原文取回来逐字节比对")
        back = link.fetch_program(PROGRAM_NAME, "script")
        print(f"  fetch_program() → {'逐字节一致 ✓' if back == dep.code else '不一致 ✗'}")
        ghost = link.fetch_program("ghost", "script")
        print(f"  取回不存在的程序 → {ghost!r}（None，不伪造、不静默瞎猜）")

        # ---------------------------------------------------------------- #
        _hr("[4] 远程执行：SSH exec 触发现场加载（退出码/输出如实回传）")
        result = link.run_command(f"ls -1 {PROGRAM_NAME}.script {PROGRAM_NAME}.manifest.json",
                                  cwd=link.remote_dir)
        print(f"  $ ls → 退出码 {result.exit_code}")
        for line in result.stdout.strip().splitlines():
            print(f"      {line}")
        load = link.run_command(f"load {PROGRAM_NAME}.script")
        print(f"  $ load {PROGRAM_NAME}.script → 退出码 {load.exit_code}"
              f"（打桩环境没有真加载器，退出码如实非 0：{load.stderr.strip()[:40]}）")
        print(f"  说明：退出码非 0 是**业务结果**，连接保持可用——一条命令写错")
        print(f"        不该废掉一台机器的会话。当前连接状态：{link.connected}")

        # ---------------------------------------------------------------- #
        _hr("[5] 失败演练：认证失败 / 断连语义")
        bad = SSHLink("127.0.0.1", stub.port, username="operator",
                      password="WRONG", remote_dir="/programs", timeout=10)
        try:
            bad.connect()
            print("  · 密码错误：未按预期报错")
        except LinkError as exc:
            print(f"  · 密码错误 → {str(exc).split('——')[0]}")
            print(f"    连接状态：{'已连接' if bad.connected else '已断开 ✓'}")
        finally:
            bad.disconnect()
        print(f"  · 打桩端认证记录：接受={stub.auth.accepted} 拒绝={stub.auth.rejected}")

        # ---------------------------------------------------------------- #
        _hr("[6] 收尾")
        link.disconnect()
        print(f"  {link.describe()}")
        print()
        print("整条链路（部署 → 核对 → 远程加载 → 失败语义）已在真 SSH 上跑通。")
        print("接真机：换 host/账号/密钥，其余一行不改；务必先在现场核对主机密钥、")
        print("路径权限与控制器侧的加载命令。")
        return 0
    finally:
        stub.stop()


if __name__ == "__main__":
    raise SystemExit(main())
