import time

from robot_platform.perception.lidar_worker import WorkerLidar


def _first(dev, wait_s=8.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < wait_s:
        fr = dev.latest()
        if fr is not None:
            return fr
    return None


def test_worker_serves_frames():
    dev = WorkerLidar("t", 0, "lo", extra_args=["--fake"])
    try:
        fr = _first(dev)
        assert fr is not None, dev.last_error
        assert fr["xyz"].shape == (2000, 3) and fr["intensity"].shape == (2000,)
        assert fr["hz"] == 10.0 and fr["frame_id"] == "fake"
        assert dev.latest() is not None
    finally:
        dev.close()
    assert dev.latest() is None and "停止" in dev.last_error


def test_worker_hang_is_bounded_and_restarts():
    """子进程卡住不回复：取帧在超时内返回、报错，下一次自动重开子进程。"""
    dev = WorkerLidar("t", 0, "lo", extra_args=["--fake", "--hang-after", "1"], reply_timeout_s=1.0)
    try:
        assert _first(dev) is not None
        old = dev._proc
        t = time.monotonic()
        assert dev.latest() is None
        assert time.monotonic() - t < 2.0
        assert "没有回复" in dev.last_error
        assert _first(dev) is not None, dev.last_error  # 新子进程又能回复一次
        assert dev.restarts == 1
        old.wait(5.0)  # 卡死的旧子进程被杀掉了
    finally:
        dev.close()


def test_worker_close_stops_process():
    dev = WorkerLidar("t", 0, "lo", extra_args=["--fake"])
    assert _first(dev) is not None
    proc = dev._proc
    dev.close()
    assert proc.wait(5.0) is not None
