"""SSHLink（SSH/SFTP 真机连接）+ SSHStubServer（本机回环打桩控制器）验收。

与 tests/test_socket_link.py 同一思路：客户端走的是**真 SSH**（paramiko 客户端
连本机回环上的 paramiko 服务端），SFTP 文件传输与 exec 远程执行都是真实协议
交互——没有 mock 出来的"假成功"，上传的代码真实落在打桩根目录里。
"""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import pytest

# core 在 <repo>/python 下；自插路径保证单文件可跑（与 test_packaging_spec 同法）
_SRC = str(Path(__file__).resolve().parents[1] / "python")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from domain.link import (
    DEFAULT_SSH_PORT,
    CommandResult,
    SSHLink,
    LinkError,
    _safe_name,
)
from adapter.sshstub import SSHStubServer

pytest.importorskip("paramiko", reason="SSH 打桩与 SSHLink 都需要 paramiko")


CODE = "DEF main()\n;带中文注释的程序\nMoveAbsJ [[0,0,0,0,0,0]]\nEND\n"


@pytest.fixture()
def stub(tmp_path: Path):
    server = SSHStubServer(tmp_path / "stubroot")
    try:
        yield server
    finally:
        server.stop()


@pytest.fixture()
def link(stub: SSHStubServer) -> SSHLink:
    link = SSHLink(
        "127.0.0.1",
        stub.port,
        username="tester",
        password="pw",
        remote_dir="/programs",
        timeout=10,
    )
    link.connect()
    try:
        yield link
    finally:
        link.disconnect()


# --------------------------- 连接与认证 --------------------------- #


def test_connect_and_disconnect(stub):
    link = SSHLink("127.0.0.1", stub.port, password="pw", remote_dir="/programs")
    assert not link.connected
    assert link.protocol == "ssh"
    link.connect()
    assert link.connected
    link.disconnect()
    assert not link.connected
    # 幂等：再断一次不炸
    link.disconnect()


def test_connect_is_reconnect_not_second_connection(link):
    link.disconnect()
    link.connect()
    assert link.connected
    # 上传记录在重连时清零（连接态是"本次连接"的口径）
    assert link.uploaded == []


def test_context_manager(stub):
    with SSHLink("127.0.0.1", stub.port, password="pw", remote_dir="/programs") as link:
        assert link.connected
    assert not link.connected


def test_wrong_password_is_connect_failure(tmp_path):
    server = SSHStubServer(tmp_path / "root", password="secret")
    try:
        link = SSHLink("127.0.0.1", server.port, password="wrong", remote_dir="/programs")
        with pytest.raises(LinkError, match="SSH 连接/认证失败"):
            link.connect()
        assert not link.connected
        # 打桩端确实记录了拒绝
        assert server.auth.rejected
    finally:
        server.stop()


def test_auth_accepted_recorded(stub, link):
    assert "tester" in stub.auth.accepted


def test_connect_to_dead_port():
    link = SSHLink("127.0.0.1", 1, timeout=2.0)
    with pytest.raises(LinkError, match="SSH 连接/认证失败"):
        link.connect()
    assert not link.connected


def test_operations_require_connect(stub):
    link = SSHLink("127.0.0.1", stub.port, remote_dir="/programs")
    with pytest.raises(LinkError, match="未连接"):
        link.upload_program("p", "x")
    with pytest.raises(LinkError, match="未连接"):
        link.list_programs()
    with pytest.raises(LinkError, match="未连接"):
        link.run_command("ls")
    with pytest.raises(LinkError, match="未连接"):
        link.fetch_program("p")


def test_no_capability_fabrication(link):
    # SSH 形态没有常驻对端服务：不伪造任何回读能力
    assert link.is_actual_feedback is False
    assert link.read_joints() is None


# --------------------------- 程序部署 --------------------------- #


def test_upload_list_fetch_roundtrip(link):
    handle = link.upload_program("pick_place", CODE, {"ext": "src", "brand": "abb"})
    assert handle.startswith("ssh://")
    assert handle.endswith("pick_place.src")
    assert link.list_programs() == ["pick_place"]
    assert link.fetch_program("pick_place", "src") == CODE


def test_upload_writes_real_files_with_manifest(link, stub):
    link.upload_program("pick", CODE, {"ext": "src", "brand": "kuka", "robot_name": "UR5e"})
    code_file = stub.root / "programs" / "pick.src"
    manifest_file = stub.root / "programs" / "pick.manifest.json"
    assert code_file.read_bytes() == CODE.encode("utf-8")  # 字节级一致，含中文
    manifest = json.loads(manifest_file.read_text(encoding="utf-8"))
    assert manifest["name"] == "pick"
    assert manifest["ext"] == "src"
    assert manifest["brand"] == "kuka"
    assert manifest["lines"] == 4
    assert manifest["bytes"] == len(CODE.encode("utf-8"))
    assert manifest["deployed_at"] > 0


def test_list_reads_remote_manifests_not_memory(link, stub):
    link.upload_program("a", "A", {"ext": "txt"})
    # 在打桩目录里手工放一份清单——模拟"远端有别人/上个会话部署的程序"
    (stub.root / "programs" / "legacy.manifest.json").write_text(
        json.dumps({"name": "legacy", "ext": "src", "deployed_at": 1.0}),
        encoding="utf-8",
    )
    programs = link.list_programs()
    assert "legacy" in programs
    assert "a" in programs


def test_list_sorted_by_deploy_time(link):
    link.upload_program("first", "1", {"ext": "txt"})
    time.sleep(0.01)
    link.upload_program("second", "2", {"ext": "txt"})
    assert link.list_programs() == ["first", "second"]


def test_overwrite_same_program(link):
    link.upload_program("pick", "v1", {"ext": "txt"})
    link.upload_program("pick", "v2", {"ext": "txt"})
    assert link.list_programs() == ["pick"]
    assert link.fetch_program("pick", "txt") == "v2"


def test_fetch_missing_program_returns_none(link):
    assert link.fetch_program("ghost", "src") is None


def test_uploaded_tracker(link):
    assert link.uploaded == []
    link.upload_program("x", "X", {"ext": "txt"})
    link.upload_program("x", "X2", {"ext": "txt"})
    assert link.uploaded == ["x"]


def test_program_name_is_sanitized(link, stub):
    handle = link.upload_program("../../evil", "X", {"ext": "txt"})
    # 名字被净化（.._.._evil）：文件只落在程序目录里，不能借名字逃出目录
    safe = _safe_name("../../evil")
    assert safe == "_.._evil"
    assert handle.endswith(f"{safe}.txt")
    assert (stub.root / "programs" / f"{safe}.txt").is_file()


def test_safe_name_basics():
    assert _safe_name("../..") == "_.."  # 点号合法但前导点被剥掉，不可再穿越
    assert _safe_name("a/b") == "a_b"
    assert _safe_name("") == "program"


def test_metadata_non_scalar_is_json_safe(link, stub):
    from pathlib import Path as _P

    link.upload_program("m", "M", {"ext": "txt", "note": _P("/tmp/x")})
    manifest = json.loads((stub.root / "programs" / "m.manifest.json").read_text(encoding="utf-8"))
    assert manifest["meta"]["note"] == "/tmp/x"  # Path 被转成字符串，不炸部署


def test_manifest_missing_name_is_reported_not_silent(link, stub):
    link.upload_program("p", "P", {"ext": "txt"})
    (stub.root / "programs" / "p.manifest.json").write_text("{}", encoding="utf-8")
    with pytest.raises(LinkError, match="缺少 name"):
        link.list_programs()
    assert link.connected  # 业务级问题不断连


# --------------------------- 远程执行 --------------------------- #


def test_run_command_captures_stdout_exit_code(link):
    result = link.run_command("echo hello")
    assert isinstance(result, CommandResult)
    assert result.ok and result.exit_code == 0
    assert result.stdout.strip() == "hello"


def test_run_command_separates_stderr(link):
    result = link.run_command("echo out; echo err >&2")
    assert result.stdout.strip() == "out"
    assert result.stderr.strip() == "err"
    assert result.ok


def test_nonzero_exit_is_business_result_not_disconnect(link):
    result = link.run_command("exit 3")
    assert result.exit_code == 3
    assert not result.ok
    assert link.connected  # 命令失败不断连


def test_run_command_cwd(link):
    # 打桩是 chroot 语义：cwd=/programs 映射进桩根目录（connect 时已建）
    result = link.run_command("pwd", cwd="/programs")
    assert result.ok
    assert "programs" in result.stdout


def test_exec_counter_includes_mkdir(link):
    base = link.executions  # connect() 里的 mkdir -p 已计入
    assert base >= 1
    link.run_command("true")
    assert link.executions == base + 1


def test_fixed_replies_bypass_real_execution(tmp_path):
    server = SSHStubServer(tmp_path / "root", fixed_replies={"load pick.src": "loaded ok\n"})
    try:
        link = SSHLink("127.0.0.1", server.port, password="pw", remote_dir="/programs")
        link.connect()
        result = link.run_command("load pick.src")
        assert result.ok
        assert result.stdout == "loaded ok\n"
        assert "load pick.src" in server.exec_log  # 打桩端看到了这条"控制器指令"
        link.disconnect()
    finally:
        server.stop()


# --------------------------- 失败语义 --------------------------- #


def test_network_failure_disconnects(stub, link):
    link.disconnect()
    # 对已断开的会话继续操作：报未连接（不假装还能用）
    with pytest.raises(LinkError, match="未连接"):
        link.run_command("ls")


def test_remote_dir_unusable_is_connect_failure(tmp_path):
    root = tmp_path / "root"
    root.mkdir()
    # chroot 根里预放一个同名**文件**挡住 mkdir -p 的目标 → 目录建不出来
    (root / "blocked").write_text("x", encoding="utf-8")
    server = SSHStubServer(root)
    try:
        link = SSHLink(
            "127.0.0.1",
            server.port,
            password="pw",
            remote_dir="/blocked/sub",
            timeout=10,
        )
        with pytest.raises(LinkError, match="远端程序目录不可用"):
            link.connect()
        assert not link.connected
    finally:
        server.stop()


def test_describe(link):
    text = link.describe()
    assert "SSHLink" in text
    assert "已连接" in text
    assert "programs" in text


def test_default_port_constant():
    assert DEFAULT_SSH_PORT == 22


# --------------------------- CLI 打桩入口 --------------------------- #


def test_stub_cli_serves_address_line(tmp_path):
    root = tmp_path / "cliroot"
    root.mkdir()
    proc = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "adapter.sshstub",
            "--port",
            "0",
            "--root",
            str(root),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        cwd=Path(__file__).resolve().parents[1] / "python",
        text=True,
    )
    try:
        address = None
        deadline = time.time() + 15
        while time.time() < deadline:
            line = proc.stdout.readline()
            if not line:
                break
            if "ADDRESS" in line:
                address = line.strip()
                break
        assert address is not None, "CLI 未输出 ADDRESS 行"
        assert address.startswith("ADDRESS ssh://127.0.0.1:")
    finally:
        proc.terminate()
        proc.wait(timeout=10)
