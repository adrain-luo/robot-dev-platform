"""HTTP 小工具：写 JSON / 文件 / 字节，读请求体，限定目录内取文件。"""

from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from typing import Any, Dict, Optional


def send_bytes(handler: BaseHTTPRequestHandler, code: int, data: bytes, content_type: str,
               cache: str = "no-cache") -> None:
    handler.send_response(code)
    handler.send_header("Content-Type", content_type)
    handler.send_header("Cache-Control", cache)
    handler.send_header("Content-Length", str(len(data)))
    handler.end_headers()
    handler.wfile.write(data)


def send_json(handler: BaseHTTPRequestHandler, code: int, payload: Dict[str, Any]) -> None:
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    send_bytes(handler, code, body, "application/json; charset=utf-8", cache="no-store")


def send_file(handler: BaseHTTPRequestHandler, path: Path, content_type: str) -> None:
    send_bytes(handler, 200, path.read_bytes(), content_type)


def safe_under(root: Path, name: str) -> Optional[Path]:
    """root 目录下直接的一个文件；名字带路径或 .. 一律拒绝。"""
    if not name or name != Path(name).name or ".." in name:
        return None
    path = (root / name).resolve()
    root_r = root.resolve()
    if path.parent != root_r or not path.is_file():
        return None
    return path


def read_json(handler: BaseHTTPRequestHandler) -> Dict[str, Any]:
    n = int(handler.headers.get("Content-Length") or 0)
    raw = handler.rfile.read(n) if n else b"{}"
    if not raw:
        return {}
    return json.loads(raw.decode("utf-8"))
