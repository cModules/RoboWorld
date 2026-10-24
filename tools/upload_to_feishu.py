#!/usr/bin/env python3
"""把 release 产物（安装包、源代码包）分片上传到飞书云空间指定文件夹。

用法（CI 里调用）::

    export FEISHU_APP_ID=xxx
    export FEISHU_APP_SECRET=xxx
    python tools/upload_to_feishu.py \\
        --parent-token <飞书父文件夹 token> \\
        --version 0.1.40 \\
        dist/RoboWorld-Setup-0.1.40.exe \\
        dist/RoboWorld-source-0.1.40.tar.gz

脚本会在父文件夹下创建名为 ``<version>`` 的子文件夹，把所有文件传进去。
分片上传：飞书 ``upload_all`` 限 20MB，安装包必超，走
``upload_prepare → upload_part → upload_finish``。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.error
import urllib.request
from pathlib import Path

FEISHU_BASE = "https://open.feishu.cn"


def _post_json(url: str, payload: dict, token: str | None = None) -> dict:
    body = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=body, method="POST")
    req.add_header("Content-Type", "application/json; charset=utf-8")
    if token:
        req.add_header("Authorization", f"Bearer {token}")
    with urllib.request.urlopen(req, timeout=30) as resp:
        return json.loads(resp.read().decode("utf-8"))


def get_tenant_token(app_id: str, app_secret: str) -> str:
    data = _post_json(
        f"{FEISHU_BASE}/open-apis/auth/v3/tenant_access_token/internal",
        {"app_id": app_id, "app_secret": app_secret},
    )
    if data.get("code") != 0:
        raise RuntimeError(f"获取 tenant_access_token 失败：{data}")
    return data["tenant_access_token"]


def create_folder(token: str, name: str, parent_token: str) -> str:
    data = _post_json(
        f"{FEISHU_BASE}/open-apis/drive/v1/files/create_folder",
        {"name": name, "folder_token": parent_token},
        token=token,
    )
    if data.get("code") != 0:
        raise RuntimeError(f"创建文件夹失败：{data}")
    return data["data"]["token"]


def upload_file(token: str, file_path: Path, folder_token: str) -> str:
    """分片上传单个文件，返回 file_token。"""
    file_name = file_path.name
    size = file_path.stat().st_size
    if size == 0:
        raise ValueError(f"空文件不能上传：{file_path}")

    # 1. 预上传，拿 upload_id 与分片策略
    prepare = _post_json(
        f"{FEISHU_BASE}/open-apis/drive/v1/files/upload_prepare",
        {
            "file_name": file_name,
            "parent_type": "explorer",
            "parent_node": folder_token,
            "size": size,
        },
        token=token,
    )
    if prepare.get("code") != 0:
        raise RuntimeError(f"预上传失败：{prepare}")
    upload_id = prepare["data"]["upload_id"]
    block_size = int(prepare["data"]["block_size"])
    block_num = (size + block_size - 1) // block_size

    # 2. 逐片上传
    with file_path.open("rb") as f:
        for seq in range(block_num):
            chunk = f.read(block_size)
            boundary = "----rwboundary" + os.urandom(8).hex()
            fields = [
                ("upload_id", upload_id),
                ("block_seq", str(seq)),
            ]
            parts = []
            for k, v in fields:
                parts.append(f"--{boundary}\r\n".encode())
                parts.append(
                    f'Content-Disposition: form-data; name="{k}"\r\n\r\n'.encode()
                )
                parts.append(f"{v}\r\n".encode())
            parts.append(f"--{boundary}\r\n".encode())
            parts.append(
                f'Content-Disposition: form-data; name="file"; '
                f'filename="{file_name}"\r\n'.encode()
            )
            parts.append(b"Content-Type: application/octet-stream\r\n\r\n")
            parts.append(chunk)
            parts.append(b"\r\n")
            parts.append(f"--{boundary}--\r\n".encode())
            body = b"".join(parts)

            req = urllib.request.Request(
                f"{FEISHU_BASE}/open-apis/drive/v1/files/upload_part",
                data=body,
                method="POST",
            )
            req.add_header(
                "Content-Type", f"multipart/form-data; boundary={boundary}"
            )
            req.add_header("Authorization", f"Bearer {token}")
            with urllib.request.urlopen(req, timeout=120) as resp:
                part = json.loads(resp.read().decode("utf-8"))
            if part.get("code") != 0:
                raise RuntimeError(
                    f"分片 {seq}/{block_num} 上传失败：{part}"
                )
            print(f"  分片 {seq + 1}/{block_num} 完成", flush=True)

    # 3. 完成上传
    finish = _post_json(
        f"{FEISHU_BASE}/open-apis/drive/v1/files/upload_finish",
        {"upload_id": upload_id, "block_num": block_num},
        token=token,
    )
    if finish.get("code") != 0:
        raise RuntimeError(f"完成上传失败：{finish}")
    return finish["data"]["file_token"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parent-token", required=True, help="飞书父文件夹 token")
    parser.add_argument("--version", required=True, help="版本号，用作子文件夹名")
    parser.add_argument("files", nargs="+", help="要上传的文件路径")
    args = parser.parse_args()

    app_id = os.environ.get("FEISHU_APP_ID")
    app_secret = os.environ.get("FEISHU_APP_SECRET")
    if not app_id or not app_secret:
        print("错误：需要环境变量 FEISHU_APP_ID / FEISHU_APP_SECRET", file=sys.stderr)
        return 1

    files = [Path(p) for p in args.files]
    missing = [str(p) for p in files if not p.is_file()]
    if missing:
        print(f"错误：文件不存在：{missing}", file=sys.stderr)
        return 1

    print(f"获取 tenant_access_token ...", flush=True)
    token = get_tenant_token(app_id, app_secret)

    print(f"创建子文件夹 {args.version} ...", flush=True)
    sub_token = create_folder(token, args.version, args.parent_token)
    print(f"  子文件夹 token: {sub_token}", flush=True)

    for fp in files:
        print(f"上传 {fp.name}（{fp.stat().st_size / 1024 / 1024:.1f} MB）...", flush=True)
        file_token = upload_file(token, fp, sub_token)
        print(f"  完成，file_token: {file_token}", flush=True)

    print("全部上传完成。", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except urllib.error.HTTPError as e:
        print(f"HTTP 错误 {e.code}: {e.read().decode('utf-8', errors='replace')}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:  # noqa: BLE001
        print(f"失败：{e}", file=sys.stderr)
        sys.exit(1)
