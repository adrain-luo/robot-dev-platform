"""末端 · 吸盘：/api/suction/* 转发到 es80z-api（这里用一个假的 es80z 服务）。"""

import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest


class _FakeES80Z:
    def __init__(self):
        self.mask = 0
        self.commands = []

    def status(self):
        return {"ok": True, "action": "status", "device": {"model": "ES80Z-FAKE", "port": "/dev/fake"},
                "data": {"fault": 0, "is_sucked": self.mask == 1, "is_released": self.mask == 0,
                         "is_blowing": self.mask == 3, "serial_present": True, "serial_error": None}}


@pytest.fixture(scope="module")
def fake_es80z():
    dev = _FakeES80Z()

    class H(BaseHTTPRequestHandler):
        def _send(self, payload):
            raw = json.dumps(payload).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            self._send(dev.status())

        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers.get("Content-Length") or 0)) or b"{}")
            cmd = body.get("command")
            dev.commands.append(cmd)
            if cmd == "suck":
                dev.mask = 1
            elif cmd == "release":
                dev.mask = 0
            else:
                self._send({"ok": False, "error": "unsupported command"})
                return
            self._send({"ok": True, "action": cmd})

        def log_message(self, *a):
            pass

    srv = ThreadingHTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    old = os.environ.get("RDP_SUCTION_URL")
    # mock_server 子进程起的时候继承这个环境变量
    os.environ["RDP_SUCTION_URL"] = f"http://127.0.0.1:{srv.server_address[1]}/api/v1"
    yield dev
    srv.shutdown()
    if old is None:
        os.environ.pop("RDP_SUCTION_URL", None)
    else:
        os.environ["RDP_SUCTION_URL"] = old


def test_suck_and_release(fake_es80z, mock_server):
    c = mock_server
    code, d = c.post("/api/suction/status")
    assert code == 200 and d["ok"] and d["suction"]["available"], d
    assert d["suction"]["state"] == "released" and d["suction"]["model"] == "ES80Z-FAKE"

    d = c.post("/api/suction/suck")[1]
    assert d["ok"], d
    assert d["suction"]["state"] == "sucking"
    d = c.post("/api/suction/release")[1]
    assert d["ok"] and d["suction"]["state"] == "released", d
    assert fake_es80z.commands[-2:] == ["suck", "release"]
    assert not c.post("/api/suction/nope")[1]["ok"]
