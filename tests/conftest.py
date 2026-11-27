import json
import os
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
SRC = REPO / "src"
if str(SRC) not in sys.path:  # pyproject 里 pytest 已配 pythonpath=src；直接 python 跑单个文件时兜底
    sys.path.insert(0, str(SRC))

from robot_platform.core import instance as instances  # noqa: E402


@pytest.fixture(scope="session", autouse=True)
def temp_instance():
    """进程内的单元测试也用临时实例，不读写任何真实机器的数据。"""
    inst = instances.activate(instances.temporary_instance("g1d"))
    yield inst
    instances.cleanup(inst)


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Client:
    def __init__(self, base: str) -> None:
        self.base = base

    def get(self, path: str):
        with urllib.request.urlopen(self.base + path, timeout=10) as r:
            return json.loads(r.read())

    def post(self, path: str, body=None):
        req = urllib.request.Request(
            self.base + path,
            data=json.dumps(body or {}).encode(),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def confirm_reset(self, kind: str = "estop"):
        """页面上的复位：先要确认令牌，再带令牌复位（web/reset_confirm.py）。返回复位那一步的 (状态码, JSON)。"""
        base = "/api/estop/reset" if kind == "estop" else "/api/guardian/reset"
        code, d = self.post(base + "/request")
        if code != 200:
            return code, d
        return self.post(base, {"token": d["token"]})

    def wait(self, pred, timeout_s: float = 5.0, period_s: float = 0.05):
        deadline = time.monotonic() + timeout_s
        while True:
            scene = self.get("/api/scene")
            if pred(scene):
                return scene
            if time.monotonic() > deadline:
                raise AssertionError("条件未在时限内满足")
            time.sleep(period_s)


@pytest.fixture(scope="module")
def mock_server(tmp_path_factory):
    """起一个 mock 后端的 server 子进程（整套 HTTP 接口都走真实代码）。

    设了 MOCK_SERVER_URL 时改连已在跑的服务（便于调试）。
    """
    url = os.environ.get("MOCK_SERVER_URL")
    if url:
        yield Client(url.rstrip("/"))
        return
    port = _free_port()
    # 输出写文件：server 每个请求都打日志，接管道不读会写满阻塞
    work = tmp_path_factory.mktemp("mock")
    log_path = work / "server.log"
    log = open(log_path, "wb")
    # 临时实例建在 pytest 的目录里：Windows 上 terminate 不走 atexit，靠 pytest 清理
    env = {**os.environ, "TMP": str(work), "TEMP": str(work), "TMPDIR": str(work),
           "PYTHONPATH": os.pathsep.join([str(SRC), os.environ.get("PYTHONPATH", "")]).rstrip(os.pathsep)}
    env.pop("ARM_INSTANCE", None)
    proc = subprocess.Popen(
        [sys.executable, "-m", "robot_platform", "--backend", "mock",
         "--host", "127.0.0.1", "--port", str(port)],
        stdout=log, stderr=subprocess.STDOUT, env=env,
    )
    client = Client(f"http://127.0.0.1:{port}")
    deadline = time.monotonic() + 30
    while True:
        try:
            client.get("/api/scene")
            break
        except Exception:
            if proc.poll() is not None or time.monotonic() > deadline:
                proc.kill()
                log.close()
                raise RuntimeError(log_path.read_text(encoding="utf-8", errors="replace"))
            time.sleep(0.2)
    yield client
    proc.terminate()
    proc.wait(timeout=10)
    log.close()
