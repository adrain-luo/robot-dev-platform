"""末端 · 吸盘：转发到盒子上的 ES80Z 真空单元服务（es80z-api）。

硬件链路（2026-11-13 在 G1D 盒子上查到）：es80z-api.service（~/es80z_api/es80z_api_service.py，
0.0.0.0:18089）→ /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0（CH340，/dev/ttyUSB0）
→ Modbus RTU 9600、从站 1、功能码 0x0F 写线圈：0x01 吸、0x03 正压吹、0x00 全关。
释放 = 先吹 0.25 s 再全关。服务记的是「命令过」的状态，没有真空度反馈。

罗师姐那套（vision_arm_control_refactor）是 SSH 到 123.164 再 curl 那边 18080，那边又转回这个
18089；本服务就跑在盒子上，直接调本机即可。地址可用环境变量 RDP_SUCTION_URL 改。
"""

from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from typing import Any, Dict, Optional

DEFAULT_URL = "http://127.0.0.1:18089/api/v1"
TIMEOUT_S = 6.0


def suction_base_url() -> str:
    return (os.environ.get("RDP_SUCTION_URL") or DEFAULT_URL).rstrip("/")


def _http(method: str, path: str, body: Optional[Dict[str, Any]] = None, timeout: float = TIMEOUT_S) -> Dict[str, Any]:
    url = f"{suction_base_url()}{path}"
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(url, data=data, method=method, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read()
    except urllib.error.HTTPError as exc:
        raw = exc.read()
    except (urllib.error.URLError, OSError) as exc:
        raise RuntimeError(f"连不上吸盘服务 {url}：{getattr(exc, 'reason', exc)}") from exc
    try:
        out = json.loads(raw.decode("utf-8") or "{}")
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"吸盘服务返回的不是 JSON：{raw[:200]!r}") from exc
    return out if isinstance(out, dict) else {"ok": False, "error": "吸盘服务返回格式不对"}


def _summary(st: Dict[str, Any]) -> Dict[str, Any]:
    data = st.get("data") or {}
    dev = st.get("device") or {}
    if data.get("is_blowing"):
        state = "blowing"
    elif data.get("is_sucked"):
        state = "sucking"
    elif data.get("is_released"):
        state = "released"
    else:
        state = "unknown"
    return {
        "available": bool(st.get("ok")),
        "state": state,
        "fault": bool(data.get("fault")),
        "serial_present": data.get("serial_present"),
        "serial_error": data.get("serial_error"),
        "last_ack_age_sec": data.get("last_ack_age_sec"),
        "model": dev.get("model") or "",
        "port": dev.get("port") or "",
        "url": suction_base_url(),
    }


class SuctionMixin:
    def suction_status(self) -> Dict[str, Any]:
        try:
            return _summary(_http("GET", "/status", timeout=2.0))
        except RuntimeError as exc:
            return {"available": False, "state": "unknown", "error": str(exc), "url": suction_base_url()}

    def suction_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        try:
            if path == "/api/suction/status":
                return {"ok": True, "suction": self.suction_status()}
            if path in ("/api/suction/suck", "/api/suction/release"):
                # 吸盘是真执行器：只读模式不让动
                self.require_motion()
                cmd = "suck" if path.endswith("/suck") else "release"
                wait_s = 3.0 if cmd == "suck" else 2.0
                out = _http("POST", "/command", {"command": cmd, "wait": True, "wait_timeout": wait_s})
                st = self.suction_status()
                if not out.get("ok"):
                    return {"ok": False, "error": f"吸盘{'吸' if cmd == 'suck' else '释放'}失败：{out.get('error') or out}",
                            "suction": st}
                msg = "吸盘已吸（真空开）" if cmd == "suck" else "吸盘已释放（吹气后全关）"
                return {"ok": True, "message": msg, "suction": st}
            return {"ok": False, "error": "没有这个吸盘接口"}
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc), "suction": self.suction_status()}
