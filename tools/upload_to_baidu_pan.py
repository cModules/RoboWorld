#!/usr/bin/env python3
"""把 release 产物（安装包、源代码包）分片上传到百度网盘开放平台。

用法（CI 里调用）::

    export BAIDU_APP_KEY=xxx          # 开放平台应用的 AppKey（client_id）
    export BAIDU_APP_SECRET=xxx       # 开放平台应用的 SecretKey（client_secret）
    export BAIDU_REFRESH_TOKEN=xxx
    python tools/upload_to_baidu_pan.py \\
        --target-dir /apps/RoboWorld/releases \\
        --version 0.1.40 \\
        dist/RoboWorld-Setup-0.1.40.exe \\
        dist/RoboWorld-source-0.1.40.tar.gz

脚本在 ``--target-dir/<version>/`` 下上传所有文件（目录不存在则创建）。

上传三步（官方 xpan 接口，大文件必须这么走）：
1. ``precreate``——提交路径/大小/各 4MB 分片的 MD5，拿 ``uploadid``；
2. ``superfile2``——逐片上传（``partseq`` 递增）；
3. ``create``——提交 uploadid 与 block_list 完成合并。

**refresh_token 是一次性的**：每次刷新都会作废旧值并返回新值。脚本把新值打印成
``NEW_REFRESH_TOKEN=<...>`` 一行，CI 用它回写 GitHub Secret；否则下次发版失败。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

PAN_API = "https://pan.baidu.com"
PCS_API = "https://d.pcs.baidu.com"
OAUTH_API = "https://openapi.baidu.com"
BLOCK_SIZE = 4 * 1024 * 1024  # 百度 superfile2 固定 4MB 分片
USER_AGENT = "pan.baidu.com"  # 缺这个头，xpan 接口常返回 errno -6（身份验证失败）

#: 目录已存在——建目录时视为成功，继续往里传
ERRNO_ALREADY_EXISTS = -8


def _request(
    url: str,
    *,
    data: bytes | None = None,
    headers: dict[str, str] | None = None,
    method: str = "POST",
) -> dict:
    req = urllib.request.Request(url, data=data, method=method)
    req.add_header("User-Agent", USER_AGENT)
    for key, value in (headers or {}).items():
        req.add_header(key, value)
    with urllib.request.urlopen(req, timeout=180) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _check(payload: dict, what: str) -> dict:
    errno = payload.get("errno")
    if errno not in (0, None):
        raise RuntimeError(f"{what}失败：errno={errno} errmsg={payload.get('errmsg', '')}")
    return payload


def refresh_access_token(app_key: str, app_secret: str, refresh_token: str) -> tuple[str, str]:
    """用 refresh_token 换 access_token。返回 (access_token, 新的 refresh_token)。"""
    query = urllib.parse.urlencode(
        {
            "grant_type": "refresh_token",
            "refresh_token": refresh_token,
            "client_id": app_key,
            "client_secret": app_secret,
        }
    )
    payload = _request(f"{OAUTH_API}/oauth/2.0/token?{query}", method="GET")
    if "access_token" not in payload:
        raise RuntimeError(f"刷新 access_token 失败：{payload}")
    return payload["access_token"], payload.get("refresh_token") or refresh_token


def create_folder(access_token: str, path: str) -> None:
    body = urllib.parse.urlencode({"path": path, "isdir": "1", "rtype": "3"}).encode()
    url = f"{PAN_API}/rest/2.0/xpan/file?method=create&access_token={access_token}"
    payload = _request(
        url, data=body, headers={"Content-Type": "application/x-www-form-urlencoded"}
    )
    if payload.get("errno") == ERRNO_ALREADY_EXISTS:
        return  # 目录已存在，直接复用
    _check(payload, f"创建目录 {path} ")


def block_md5_list(file_path: Path) -> list[str]:
    """按 4MB 切分算每片的 MD5（32 位小写）；末片为余量。"""
    digests: list[str] = []
    with file_path.open("rb") as f:
        while True:
            chunk = f.read(BLOCK_SIZE)
            if not chunk:
                break
            digests.append(hashlib.md5(chunk).hexdigest())
    return digests


def upload_file(access_token: str, file_path: Path, remote_dir: str) -> None:
    file_name = file_path.name
    size = file_path.stat().st_size
    if size == 0:
        raise ValueError(f"空文件不能上传：{file_path}")
    remote_path = f"{remote_dir.rstrip('/')}/{file_name}"
    blocks = block_md5_list(file_path)

    # 1. 预上传
    body = urllib.parse.urlencode(
        {
            "path": remote_path,
            "size": str(size),
            "isdir": "0",
            "autoinit": "1",
            "rtype": "3",  # 3 = 覆盖已存在的同名文件
            "block_list": json.dumps(blocks),
        }
    ).encode()
    precreate = _check(
        _request(
            f"{PAN_API}/rest/2.0/xpan/file?method=precreate&access_token={access_token}",
            data=body,
            headers={"Content-Type": "application/x-www-form-urlencoded"},
        ),
        f"预上传 {file_name} ",
    )
    upload_id = precreate["uploadid"]
    # 已存在的分片不用重传（秒传/断点续传）
    uploaded = {int(seq) for seq in precreate.get("block_list", [])}

    # 2. 逐片上传
    with file_path.open("rb") as f:
        for seq in range(len(blocks)):
            chunk = f.read(BLOCK_SIZE)
            if seq in uploaded:
                print(f"  分片 {seq + 1}/{len(blocks)} 已在云端，跳过", flush=True)
                continue
            boundary = "----rwboundary" + os.urandom(8).hex()
            body_parts = [
                f"--{boundary}\r\n".encode(),
                b'Content-Disposition: form-data; name="file"; '
                + f'filename="{file_name}"\r\n'.encode(),
                b"Content-Type: application/octet-stream\r\n\r\n",
                chunk,
                b"\r\n",
                f"--{boundary}--\r\n".encode(),
            ]
            query = urllib.parse.urlencode(
                {
                    "method": "upload",
                    "access_token": access_token,
                    "type": "tmpfile",
                    "path": remote_path,
                    "uploadid": upload_id,
                    "partseq": str(seq),
                }
            )
            part = _request(
                f"{PCS_API}/rest/2.0/pcs/superfile2?{query}",
                data=b"".join(body_parts),
                headers={
                    "Content-Type": f"multipart/form-data; boundary={boundary}"
                },
            )
            if part.get("error_code"):
                raise RuntimeError(
                    f"分片 {seq + 1}/{len(blocks)} 上传失败：{part}"
                )
            print(f"  分片 {seq + 1}/{len(blocks)} 完成", flush=True)

    # 3. 合并
    body = urllib.parse.urlencode(
        {
            "path": remote_path,
            "size": str(size),
            "isdir": "0",
            "rtype": "3",
            "uploadid": upload_id,
            "block_list": json.dumps(blocks),
        }
    ).encode()
    _check(
        _request(
            f"{PAN_API}/rest/2.0/xpan/file?method=create&access_token={access_token}",
            data=body,
            headers={"Content-Type": "application/x-www-form-urlencoded"},
        ),
        f"合并 {file_name} ",
    )
    print(f"  已上传到 {remote_path}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target-dir",
        default="/apps/RoboWorld/releases",
        help="网盘目标父目录（默认 /apps/RoboWorld/releases）",
    )
    parser.add_argument("--version", required=True, help="版本号，用作子文件夹名")
    parser.add_argument("files", nargs="+", help="要上传的文件路径")
    args = parser.parse_args()

    app_key = os.environ.get("BAIDU_APP_KEY")
    app_secret = os.environ.get("BAIDU_APP_SECRET")
    refresh_token = os.environ.get("BAIDU_REFRESH_TOKEN")
    if not (app_key and app_secret and refresh_token):
        print(
            "错误：需要环境变量 BAIDU_APP_KEY / BAIDU_APP_SECRET / BAIDU_REFRESH_TOKEN",
            file=sys.stderr,
        )
        return 1

    files = [Path(p) for p in args.files]
    missing = [str(p) for p in files if not p.is_file()]
    if missing:
        print(f"错误：文件不存在：{missing}", file=sys.stderr)
        return 1

    access_token, new_refresh_token = refresh_access_token(
        app_key, app_secret, refresh_token
    )
    # 一次性 token：无论后续上传成败，都必须把这个新值交出去回写 Secret
    print(f"NEW_REFRESH_TOKEN={new_refresh_token}", flush=True)
    github_output = os.environ.get("GITHUB_OUTPUT")
    if github_output:
        with open(github_output, "a", encoding="utf-8") as f:
            f.write(f"new_refresh_token={new_refresh_token}\n")

    remote_dir = f"{args.target_dir.rstrip('/')}/{args.version}"
    print(f"创建目录 {remote_dir} ...", flush=True)
    create_folder(access_token, remote_dir)

    for fp in files:
        print(
            f"上传 {fp.name}（{fp.stat().st_size / 1024 / 1024:.1f} MB）...",
            flush=True,
        )
        upload_file(access_token, fp, remote_dir)

    print("全部上传完成。", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except urllib.error.HTTPError as e:
        print(
            f"HTTP 错误 {e.code}: {e.read().decode('utf-8', errors='replace')}",
            file=sys.stderr,
        )
        sys.exit(1)
    except Exception as e:  # noqa: BLE001
        print(f"失败：{e}", file=sys.stderr)
        sys.exit(1)
