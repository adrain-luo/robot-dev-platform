"""独立相机查看服务：清晰度计算、掩膜、HTTP 接口（仿真画面，子进程起服务）。"""

import json
import os
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np
import pytest

from robot_platform.perception import sharpness as sh

ROOT = Path(__file__).resolve().parents[1]


def _texture(seed=0, h=240, w=320):
    rng = np.random.default_rng(seed)
    img = np.zeros((h, w, 3), np.uint8)
    for y in range(0, h, 20):
        for x in range(0, w, 20):
            if (x // 20 + y // 20) % 2 == 0:
                img[y:y + 20, x:x + 20] = 220
    return cv2.addWeighted(img, 0.7, rng.integers(0, 255, (h, w, 3), dtype=np.uint8), 0.3, 0)


def test_blur_lowers_score():
    img = _texture()
    s0 = sh.frame_score(img)
    s1 = sh.frame_score(cv2.GaussianBlur(img, (0, 0), 2.0))
    assert s1["whole"] < 0.5 * s0["whole"] and s1["center"] < 0.5 * s0["center"]


def test_map_finds_sharp_half_and_overlay_shape():
    img = _texture()
    img[:, :160] = cv2.GaussianBlur(img[:, :160], (0, 0), 3.0)  # 左半模糊
    m = sh.sharpness_map(img, 15)
    assert m[:, 200:].mean() > 5 * m[:, :120].mean()
    scale = sh.Scale()
    out, st = sh.overlay(img, scale, alpha=0.5)
    assert out.shape == img.shape and out.dtype == np.uint8 and st.hi > st.lo
    t, _ = sh.level_map(img, scale)
    assert t[:, int(t.shape[1] * 0.7):].mean() > t[:, : int(t.shape[1] * 0.3)].mean() + 0.3


def test_locked_scale_does_not_follow():
    scale = sh.Scale()
    img = _texture()
    sh.overlay(img, scale)
    scale.set_locked(True)
    st0 = scale.state()
    for _ in range(5):
        sh.overlay(cv2.GaussianBlur(img, (0, 0), 3.0), scale)
    st1 = scale.state()
    assert st1.locked and st1.lo == st0.lo and st1.hi == st0.hi
    # 锁定后模糊画面整体偏冷（归一化值变小）
    t_sharp, _ = sh.level_map(img, scale)
    t_blur, _ = sh.level_map(cv2.GaussianBlur(img, (0, 0), 3.0), scale)
    assert t_blur.mean() < t_sharp.mean() - 0.2
    scale.set_locked(False)
    for _ in range(40):
        sh.overlay(cv2.GaussianBlur(img, (0, 0), 3.0), scale)
    assert scale.state().hi < st0.hi  # 解锁后跟着画面走


def test_flat_image_does_not_explode():
    flat = np.full((120, 160, 3), 128, np.uint8)
    out, st = sh.overlay(flat, sh.Scale())
    assert out.shape == flat.shape and st.hi - st.lo >= sh.MIN_SPAN - 1e-6


def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def cam_server(tmp_path_factory):
    port = _free_port()
    log = open(tmp_path_factory.mktemp("cam") / "log.txt", "wb")
    proc = subprocess.Popen([sys.executable, str(ROOT / "camera_serve.py"), "--mock", "--host", "127.0.0.1", "--port", str(port)],
                            stdout=log, stderr=subprocess.STDOUT)
    base = f"http://127.0.0.1:{port}"
    deadline = time.monotonic() + 20
    while True:
        try:
            st = json.loads(urllib.request.urlopen(base + "/api/status", timeout=2).read())
            if st.get("seq", 0) > 2:
                break
        except OSError:
            pass
        if proc.poll() is not None or time.monotonic() > deadline:
            proc.kill()
            raise RuntimeError("相机服务没起来")
        time.sleep(0.2)
    yield base
    proc.terminate()
    proc.wait(timeout=10)
    log.close()


def _post(url, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    return json.loads(urllib.request.urlopen(req, timeout=5).read())


def test_page_and_status(cam_server):
    html = urllib.request.urlopen(cam_server + "/", timeout=5).read().decode("utf-8")
    assert "相机查看" in html and "清晰度掩膜" in html
    st = json.loads(urllib.request.urlopen(cam_server + "/api/status", timeout=5).read())
    assert st["ok"] and st["source"] == "mock" and st["size"] == [640, 480] and not st["error"]
    assert set(st["score"]) == {"left", "right"} and st["score"]["left"]["center"] > 0
    assert st["fps"] > 5


@pytest.mark.parametrize("eye,mask", [("left", 0), ("right", 1)])
def test_snapshot_and_stream(cam_server, eye, mask):
    data = urllib.request.urlopen(f"{cam_server}/snapshot.jpg?eye={eye}&mask={mask}&alpha=0.5", timeout=5).read()
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    assert img.shape == (480, 640, 3)
    r = urllib.request.urlopen(f"{cam_server}/stream.mjpg?eye={eye}&mask={mask}", timeout=5)
    assert r.headers["Content-Type"].startswith("multipart/x-mixed-replace")
    buf = b""
    while buf.count(b"--frame") < 3:
        buf += r.read1(65536)
    r.close()
    assert buf.count(b"\xff\xd8") >= 2  # 至少两帧 JPEG


def test_mask_changes_image(cam_server):
    a = urllib.request.urlopen(f"{cam_server}/snapshot.jpg?eye=left&mask=0", timeout=5).read()
    b = urllib.request.urlopen(f"{cam_server}/snapshot.jpg?eye=left&mask=1&alpha=0.8", timeout=5).read()
    ia = cv2.imdecode(np.frombuffer(a, np.uint8), cv2.IMREAD_COLOR).astype(int)
    ib = cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR).astype(int)
    assert np.abs(ia - ib).mean() > 20  # 掩膜盖上去了


def test_scale_lock_and_peak_api(cam_server):
    urllib.request.urlopen(f"{cam_server}/snapshot.jpg?eye=right&mask=1", timeout=5).read()  # 让刻度先有值
    st = _post(cam_server + "/api/scale", {"eye": "right", "lock": True})
    assert st["scale"]["right"]["locked"] is True
    st = _post(cam_server + "/api/scale", {"eye": "right", "reset": True})
    assert st["scale"]["right"] is None
    time.sleep(3)  # 仿真焦点来回变：让峰值攒到比当前高
    before = json.loads(urllib.request.urlopen(cam_server + "/api/status", timeout=5).read())
    assert before["peak"]["left"]["center"] >= before["score"]["left"]["center"] * 0.99
    st = _post(cam_server + "/api/peak", {"eye": "left"})
    # 清零后从头攒：峰值只来自清零后的一两帧，≈ 当前值
    assert st["peak"]["left"]["center"] <= st["score"]["left"]["center"] * 1.2 + 1e-6
    assert st["peak"]["right"]["center"] >= before["peak"]["right"]["center"]  # 另一眼不受影响
    with pytest.raises(urllib.error.HTTPError):
        _post(cam_server + "/api/scale", {"eye": "middle"})


def test_entry_guarded():
    """camera_serve.py 被当模块导入时不能起服务（与 serve.py 一样有 __main__ 保护）。"""
    src = (ROOT / "camera_serve.py").read_text(encoding="utf-8")
    assert 'if __name__ == "__main__":' in src


def test_hub_skips_repeated_frames():
    """teleimager 会反复返回同一张缓存帧：只处理新帧、不空转（G1D 上曾空转到 205% CPU）。"""
    from robot_platform.tools.camera_view.server import CameraHub

    class Repeating:
        source, host, last_error = "stub", "", ""

        def __init__(self):
            self.calls = 0
            self.img = _texture(1, 120, 160)

        def grab_pair(self, timeout=2.0):
            self.calls += 1
            if self.calls == 30:  # 中途来一张新帧
                self.img = _texture(2, 120, 160)
            return self.img, self.img

        def close(self):
            pass

    cam = Repeating()
    hub = CameraHub(cam)
    try:
        time.sleep(1.0)
        st = hub.status()
    finally:
        hub.close()
    assert st["seq"] == 2  # 只有两张不同的画面
    assert cam.calls < 80  # 重复帧会让出 CPU（约 60 Hz 封顶），不是死循环
