"""雷达子进程：DDS 订阅与点云解码放在独立进程里，主服务按需取最新一帧。

为什么单独一个进程：
- 每帧约 450 KB 的 PointCloud2 在 DDS 接收线程里用 Python 反序列化，会把同进程 rt/lowstate
  （~1 kHz）的投递推迟 6–9 ms；放到别的进程，主服务的 DDS 只剩机器人自己的话题。
- 宇树 SDK 建立 / 断开订阅偶发卡死（G1 上遇到过）；卡在子进程里，主服务超时后直接杀掉重开即可。

协议（stdin / stdout，二进制）：
  主 → 子：一行命令 ``get\\n`` 或 ``quit\\n``（stdin 关闭等同 quit）。
  子 → 主：一行 JSON 头 + 紧跟的点数据。头里 ``n`` 为点数，``has_int`` 表示是否带强度；
          数据为 n×3 float32（xyz）+ 可选 n float32（强度），小端。
子进程用 ``python -m robot_platform.perception.lidar_worker`` 启动（全新解释器，不 fork，
避开主进程里已经起来的 DDS 线程）。``--fake`` 不连 DDS、生成假点云，供测试用。
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

REPLY_TIMEOUT_S = 1.5      # 单次取帧等子进程回复的上限；超了就认为子进程卡死，杀掉重开
_SRC_DIR = Path(__file__).resolve().parents[2]


# ====================================================================== 子进程
def _write_frame(out, frame: Optional[Dict[str, Any]], error: str) -> None:
    if frame is None:
        head = {"ok": False, "error": error or "没有雷达数据", "n": 0}
        out.write((json.dumps(head, ensure_ascii=False) + "\n").encode("utf-8"))
        out.flush()
        return
    xyz = np.ascontiguousarray(frame["xyz"], dtype="<f4")
    inten = frame.get("intensity")
    head = {"ok": True, "n": int(len(xyz)), "has_int": inten is not None, "hz": float(frame.get("hz") or 0.0),
            "frame_id": str(frame.get("frame_id") or ""), "stamp": float(frame.get("stamp") or 0.0)}
    out.write((json.dumps(head, ensure_ascii=False) + "\n").encode("utf-8"))
    out.write(xyz.tobytes())
    if inten is not None:
        out.write(np.ascontiguousarray(inten, dtype="<f4").tobytes())
    out.flush()


class _FakeSource:
    """测试用：10 Hz 的假点云，不连 DDS。"""

    def __init__(self, n: int = 2000) -> None:
        self.last_error = ""
        self._n = n
        self._t0 = time.monotonic()

    def latest(self, max_age_s: float = 2.0):
        k = int((time.monotonic() - self._t0) * 10)
        rng = np.random.default_rng(k)
        return {"xyz": rng.normal(size=(self._n, 3)).astype(np.float32),
                "intensity": rng.uniform(0, 255, self._n).astype(np.float32),
                "hz": 10.0, "frame_id": "fake", "stamp": time.time()}

    def close(self) -> None:
        pass


def worker_main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="")
    ap.add_argument("--domain", type=int, default=0)
    ap.add_argument("--iface", default="")
    ap.add_argument("--sdk", default="")
    ap.add_argument("--fake", action="store_true")
    ap.add_argument("--hang-after", type=int, default=0, help="测试用：回复这么多次后不再回复（模拟卡死）")
    a = ap.parse_args(argv)
    out = sys.stdout.buffer
    sys.stdout = sys.stderr  # SDK 的 print 不能混进二进制通道
    if a.fake:
        src = _FakeSource()
    else:
        from robot_platform.perception.lidar import DdsLidar
        src = DdsLidar(a.topic, a.domain, a.iface, Path(a.sdk) if a.sdk else None)
    served = 0
    for line in sys.stdin.buffer:
        cmd = line.strip()
        if cmd == b"quit":
            break
        if cmd != b"get":
            continue
        if a.hang_after and served >= a.hang_after:
            time.sleep(3600)
        frame = src.latest()
        _write_frame(out, frame, src.last_error)
        served += 1
    # 不走 SDK 的析构（偶发卡死），直接退出
    out.flush()
    os._exit(0)


# ====================================================================== 主进程侧
class WorkerLidar:
    """主服务里的雷达对象：接口与 DdsLidar 一致（latest / close / last_error），实际数据来自子进程。

    子进程不回复（卡死）或退出了，就杀掉、下次取帧时重开；主服务这边任何调用都有时间上限。
    """

    kind = "unitree_dds"

    def __init__(self, topic: str, dds_domain: int, network_interface: str, sdk_path: Optional[Path] = None,
                 *, extra_args=(), reply_timeout_s: float = REPLY_TIMEOUT_S) -> None:
        self.topic = topic
        self.last_error = ""
        self.restarts = 0
        self._args = ["--topic", topic, "--domain", str(int(dds_domain)), "--iface", network_interface or ""]
        if sdk_path:
            self._args += ["--sdk", str(sdk_path)]
        self._args += list(extra_args)
        self._timeout = reply_timeout_s
        self._lock = threading.Lock()
        self._proc: Optional[subprocess.Popen] = None
        self._replies: "queue.Queue" = queue.Queue()
        self._closed = False
        self._start()

    # ------------------------------------------------------------ 进程
    def _start(self) -> None:
        env = dict(os.environ)
        env["PYTHONPATH"] = str(_SRC_DIR) + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
        env["PYTHONUNBUFFERED"] = "1"
        self._replies = queue.Queue()
        self._proc = subprocess.Popen(
            [sys.executable, "-m", "robot_platform.perception.lidar_worker", *self._args],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, env=env, bufsize=0)
        threading.Thread(target=self._reader, args=(self._proc, self._replies), name="lidar-worker-rx",
                         daemon=True).start()

    @staticmethod
    def _read_exact(f, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = f.read(n - len(buf))
            if not chunk:
                raise EOFError
            buf += chunk
        return bytes(buf)

    @classmethod
    def _reader(cls, proc: subprocess.Popen, replies: "queue.Queue") -> None:
        """后台线程：阻塞读子进程的回复（读的时候不占 GIL），整帧放进队列。"""
        f = proc.stdout
        try:
            while True:
                line = f.readline()
                if not line:
                    break
                head = json.loads(line.decode("utf-8"))
                n = int(head.get("n") or 0)
                if head.get("ok"):
                    xyz = np.frombuffer(cls._read_exact(f, n * 12), dtype="<f4").reshape(n, 3)
                    inten = np.frombuffer(cls._read_exact(f, n * 4), dtype="<f4") if head.get("has_int") else None
                    head["xyz"], head["intensity"] = xyz, inten
                replies.put(head)
        except (EOFError, OSError, ValueError):
            pass
        replies.put(None)  # 子进程没了

    def _kill(self, proc: Optional[subprocess.Popen]) -> None:
        if proc is None:
            return

        def _do() -> None:
            try:
                try:
                    proc.stdin.write(b"quit\n")
                    proc.stdin.flush()
                except (OSError, ValueError):
                    pass
                try:
                    proc.wait(1.0)
                except subprocess.TimeoutExpired:
                    proc.terminate()
                    try:
                        proc.wait(2.0)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(2.0)
            except Exception:  # noqa: BLE001
                pass

        threading.Thread(target=_do, name="lidar-worker-stop", daemon=True).start()

    # ------------------------------------------------------------ 接口
    def latest(self, max_age_s: float = 2.0):
        if not self._lock.acquire(timeout=self._timeout + 0.5):
            self.last_error = "雷达子进程忙"
            return None
        try:
            if self._closed:
                self.last_error = "雷达已停止"
                return None
            if self._proc is None or self._proc.poll() is not None:
                self.restarts += 1
                self._start()
            # 丢掉上一次超时后迟到的回复
            while not self._replies.empty():
                self._replies.get_nowait()
            try:
                self._proc.stdin.write(b"get\n")
                self._proc.stdin.flush()
                head = self._replies.get(timeout=self._timeout)
            except (OSError, ValueError, queue.Empty):
                head = False
            if not head:
                why = "没有回复" if head is False else "退出了"
                self.last_error = f"雷达子进程{why}，已重启（订阅 {self.topic}）"
                proc, self._proc = self._proc, None
                self._kill(proc)
                return None
            if not head.get("ok"):
                self.last_error = str(head.get("error") or "没有雷达数据")
                return None
            self.last_error = ""
            return {"xyz": head["xyz"], "intensity": head["intensity"], "hz": float(head.get("hz") or 0.0),
                    "frame_id": head.get("frame_id") or "", "stamp": float(head.get("stamp") or 0.0)}
        finally:
            self._lock.release()

    def close(self) -> None:
        self._closed = True
        proc, self._proc = self._proc, None
        self._kill(proc)


if __name__ == "__main__":
    worker_main()
