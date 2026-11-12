"""独立的相机查看服务（与主站点无关，可单独启动）：实时画面、切换左 / 右眼、清晰度掩膜。

    python camera_serve.py                 # 按机型（默认 g1d）声明的双目相机取图，端口 10186
    python camera_serve.py --mock          # 仿真画面（焦点来回变），本地调界面用

只订阅相机（teleimager 是 ZMQ 发布 / 订阅，可以和主站点同时看），不发任何指令、不写任何数据。

接口：
- GET  /                         页面
- GET  /stream.mjpg?eye=left|right&mask=0|1&alpha=0.45   MJPEG 实时流
- GET  /snapshot.jpg?…           同上参数的单帧
- GET  /api/status               取流状态、帧率、分辨率、两眼清晰度（当前 / 峰值）、掩膜刻度
- POST /api/scale  {"eye": "left", "lock": true}   锁定 / 解锁该眼的掩膜刻度；{"reset": true} 重新自动
- POST /api/peak   {"eye": "left"}  清零峰值（不给 eye = 两眼都清）
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional, Tuple
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from robot_platform.perception import sharpness as sh

PAGE = Path(__file__).with_name("page.html")
EYES = ("left", "right")
STREAM_FPS = 20.0
JPEG_QUALITY = 85


class MockStereo:
    """仿真双目：有纹理的画面，焦点（模糊程度）随时间来回变，右眼略有平移。"""

    source = "mock"
    host = ""

    def __init__(self) -> None:
        self.last_error = ""
        self._t0 = time.monotonic()
        h, w = 480, 640
        rng = np.random.default_rng(0)
        img = np.full((h, w, 3), 70, np.uint8)
        for y in range(0, h, 40):
            for x in range(0, w, 40):
                if (x // 40 + y // 40) % 2 == 0:
                    img[y:y + 40, x:x + 40] = 200
        img = cv2.addWeighted(img, 0.7, rng.integers(0, 255, (h, w, 3), dtype=np.uint8), 0.3, 0)
        cv2.putText(img, "MOCK STEREO", (150, 250), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (20, 20, 220), 4)
        self._base = img

    def grab_pair(self, timeout: float = 2.0) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        time.sleep(1 / 30.0)
        t = time.monotonic() - self._t0
        sigma = 0.2 + 3.0 * (0.5 + 0.5 * math.sin(t * 0.6))  # 0.2..3.2 像素来回
        left = cv2.GaussianBlur(self._base, (0, 0), sigma)
        # 右眼：平移一点，且只有左半边对焦不准，方便看掩膜
        right = np.roll(self._base, -12, axis=1)
        right[:, : right.shape[1] // 2] = cv2.GaussianBlur(right[:, : right.shape[1] // 2], (0, 0), sigma + 1.0)
        return left, right

    def close(self) -> None:
        pass


class CameraHub:
    """后台线程不停取最新一帧；两眼清晰度、峰值、掩膜刻度都在这里。"""

    def __init__(self, camera) -> None:
        self.camera = camera
        self._cond = threading.Condition()
        self._frames: Dict[str, np.ndarray] = {}
        self._seq = 0
        self._t_frame = 0.0
        self._fps = 0.0
        self._scores: Dict[str, Dict[str, float]] = {}
        self._peak: Dict[str, Dict[str, float]] = {e: {"whole": 0.0, "center": 0.0} for e in EYES}
        self.scales = {e: sh.Scale() for e in EYES}
        self._render_lock = threading.Lock()
        self._render_cache: Dict[Tuple, Tuple[int, bytes]] = {}
        self._stop = threading.Event()
        self.error = ""
        self._thread = threading.Thread(target=self._run, name="camera-grab", daemon=True)
        self._thread.start()

    # ------------------------------------------------------------ 取图
    def _run(self) -> None:
        last = None
        while not self._stop.is_set():
            try:
                pair = self.camera.grab_pair(2.0)
            except Exception as exc:  # noqa: BLE001
                pair = None
                self.camera.last_error = f"{type(exc).__name__}: {exc}"
            if pair is None:
                self.error = getattr(self.camera, "last_error", "") or "没有画面"
                time.sleep(0.5)
                continue
            now = time.monotonic()
            scores = {}
            for eye, img in zip(EYES, pair):
                s = sh.frame_score(img)
                scores[eye] = s
                pk = self._peak[eye]
                for k in ("whole", "center"):
                    pk[k] = max(pk[k], s[k])
            with self._cond:
                self._frames = dict(zip(EYES, pair))
                self._scores = scores
                self._seq += 1
                if last is not None:
                    dt = now - last
                    self._fps = 1.0 / dt if self._fps == 0 else 0.9 * self._fps + 0.1 / max(dt, 1e-3)
                last = now
                self._t_frame = now
                self.error = ""
                self._cond.notify_all()

    def wait_frame(self, after_seq: int, timeout: float = 3.0) -> Tuple[int, Optional[Dict[str, np.ndarray]]]:
        with self._cond:
            self._cond.wait_for(lambda: self._seq > after_seq or self._stop.is_set(), timeout)
            return self._seq, (self._frames if self._seq > after_seq else None)

    def latest(self, timeout: float = 3.0) -> Tuple[int, Optional[Dict[str, np.ndarray]]]:
        """最新一帧；还没有就等到 timeout。"""
        with self._cond:
            self._cond.wait_for(lambda: self._seq > 0 or self._stop.is_set(), timeout)
            return self._seq, (self._frames or None)

    def close(self) -> None:
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        try:
            self.camera.close()
        except Exception:  # noqa: BLE001
            pass

    # ------------------------------------------------------------ 出图
    def render_jpeg(self, seq: int, frames: Dict[str, np.ndarray], eye: str, mask: bool, alpha: float) -> bytes:
        """同一帧、同样参数只算一次（多个页面同时看也不重复算掩膜、不重复推刻度）。"""
        key = (eye, mask, round(alpha, 2))
        with self._render_lock:
            hit = self._render_cache.get(key)
            if hit is not None and hit[0] == seq:
                return hit[1]
            img = frames[eye]
            if mask:
                img, _ = sh.overlay(img, self.scales[eye], alpha=alpha)
            ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            data = buf.tobytes() if ok else b""
            self._render_cache[key] = (seq, data)
            return data

    def status(self) -> Dict[str, Any]:
        with self._cond:
            frames, scores, fps, t_frame, seq = self._frames, dict(self._scores), self._fps, self._t_frame, self._seq
        size = None
        if frames:
            h, w = frames["left"].shape[:2]
            size = [w, h]
        age = None if not t_frame else time.monotonic() - t_frame
        scales = {}
        for e in EYES:
            st = self.scales[e].state()
            scales[e] = None if st is None else {"lo": st.lo, "hi": st.hi, "locked": st.locked}
        return {
            "ok": True,
            "source": getattr(self.camera, "source", ""),
            "host": getattr(self.camera, "host", ""),
            "seq": seq,
            "fps": round(fps, 1),
            "size": size,
            "age_s": None if age is None else round(age, 2),
            "error": self.error if (age is None or age > 2.0) else "",
            "score": scores,
            "peak": {e: dict(v) for e, v in self._peak.items()},
            "scale": scales,
            "levels": sh.LEVELS,
        }

    def reset_peak(self, eye: Optional[str] = None) -> None:
        for e in ([eye] if eye in EYES else EYES):
            self._peak[e] = {"whole": 0.0, "center": 0.0}


def _parse_view(q: Dict[str, list]) -> Tuple[str, bool, float]:
    eye = (q.get("eye") or ["left"])[0]
    if eye not in EYES:
        raise ValueError("eye 只能是 left 或 right")
    mask = (q.get("mask") or ["0"])[0] in ("1", "true", "on")
    try:
        alpha = float((q.get("alpha") or ["0.45"])[0])
    except ValueError as exc:
        raise ValueError("alpha 要是 0～1 的数") from exc
    return eye, mask, float(np.clip(alpha, 0.0, 1.0))


def make_handler(hub: CameraHub):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt, *args):  # 流请求很多，不逐条打日志
            pass

        def _json(self, code: int, obj: Dict[str, Any]) -> None:
            body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802
            u = urlparse(self.path)
            q = parse_qs(u.query)
            try:
                if u.path in ("/", "/index.html"):
                    body = PAGE.read_bytes()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(body)
                elif u.path == "/api/status":
                    self._json(200, hub.status())
                elif u.path == "/snapshot.jpg":
                    eye, mask, alpha = _parse_view(q)
                    seq, frames = hub.latest(3.0)
                    if not frames:
                        self._json(503, {"ok": False, "error": hub.error or "还没有画面"})
                        return
                    data = hub.render_jpeg(seq, frames, eye, mask, alpha)
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(data)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(data)
                elif u.path == "/stream.mjpg":
                    self._stream(*_parse_view(q))
                else:
                    self._json(404, {"ok": False, "error": "没有这个地址"})
            except ValueError as exc:
                self._json(400, {"ok": False, "error": str(exc)})
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass

        def _stream(self, eye: str, mask: bool, alpha: float) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.close_connection = True
            seq, period, last = 0, 1.0 / STREAM_FPS, 0.0
            while not hub._stop.is_set():
                seq, frames = hub.wait_frame(seq, 3.0)
                if not frames:
                    continue
                wait = period - (time.monotonic() - last)
                if wait > 0:
                    time.sleep(wait)
                last = time.monotonic()
                data = hub.render_jpeg(seq, frames, eye, mask, alpha)
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
                self.wfile.flush()

        def do_POST(self) -> None:  # noqa: N802
            u = urlparse(self.path)
            try:
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(n) or b"{}") if n else {}
                eye = body.get("eye")
                if eye is not None and eye not in EYES:
                    raise ValueError("eye 只能是 left 或 right")
                if u.path == "/api/scale":
                    for e in ([eye] if eye else EYES):
                        if body.get("reset"):
                            hub.scales[e].reset()
                        else:
                            hub.scales[e].set_locked(bool(body.get("lock", True)))
                    self._json(200, hub.status())
                elif u.path == "/api/peak":
                    hub.reset_peak(eye)
                    self._json(200, hub.status())
                else:
                    self._json(404, {"ok": False, "error": "没有这个接口"})
            except (ValueError, json.JSONDecodeError) as exc:
                self._json(400, {"ok": False, "error": str(exc)})

    return Handler


def _camera_from_profile(robot: str, host: str, teleimager_path: str):
    from robot_platform.core.robot_profile import load_profile
    from robot_platform.perception.camera import HeadCamera

    cams = [c for c in load_profile(robot).cameras.values() if c.type == "teleimager"]
    params = dict(cams[0].params) if cams else {}
    return HeadCamera(
        source="teleimager",
        host=host or str(params.get("host") or "192.168.123.164"),
        teleimager_path=teleimager_path or str(params.get("teleimager_path") or ""),
    )


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="独立相机查看服务：实时画面、切换左右眼、清晰度掩膜（只读）")
    ap.add_argument("--port", type=int, default=10186)
    ap.add_argument("--host", default="0.0.0.0", help="网页监听地址")
    ap.add_argument("--robot", default="g1d", help="按哪个机型的双目相机声明取默认地址")
    ap.add_argument("--camera-host", default="", help="teleimager 所在主机（默认取机型声明）")
    ap.add_argument("--teleimager-path", default="", help="teleimager 源码目录（默认取机型声明）")
    ap.add_argument("--mock", action="store_true", help="仿真画面，不连相机")
    a = ap.parse_args(argv)
    camera = MockStereo() if a.mock else _camera_from_profile(a.robot, a.camera_host, a.teleimager_path)
    hub = CameraHub(camera)
    server = ThreadingHTTPServer((a.host, a.port), make_handler(hub))
    server.daemon_threads = True
    src = "仿真" if a.mock else f"teleimager @ {camera.host}"
    print(f"[camera_view] {src}  http://{a.host}:{a.port}/  （只读，不发任何指令）", flush=True)
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        hub.close()
        server.server_close()
    return 0
