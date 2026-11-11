"""指令话题占用检测：判定逻辑（假时钟）+ mock 服务上的接管拒绝与状态显示。"""

import pytest

from robot_platform.hw.backends.base import PermissionErrorMotion
from robot_platform.hw.cmd_monitor import CmdMonitor, scan_processes


class Clock:
    def __init__(self) -> None:
        self.t = 100.0

    def __call__(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        self.t += dt


def _mon(clock):
    return CmdMonitor("rt/arm_sdk", clock=clock)


def test_own_frames_are_not_foreign():
    c = Clock()
    m = _mon(c)
    c.sleep(2.0)
    for k in range(100):           # 本程序 100 Hz，DDS 回环收到自己的帧
        m.note_own(1000 + k)
        m.on_message(1000 + k)
        c.sleep(0.01)
    r = m.report()
    assert not r["foreign"] and r["foreign_hz"] == 0 and r["own_hz"] > 90
    m.check_free(sleep=c.sleep)    # 不抛


def test_foreign_detected_even_while_we_publish():
    c = Clock()
    m = _mon(c)
    c.sleep(2.0)
    for k in range(100):
        m.note_own(k)
        m.on_message(k)
        m.on_message(50000 + k, weight=1.0, kp_max=80.0)   # 别人同时在发
        c.sleep(0.01)
    r = m.report()
    assert r["foreign"] and r["foreign_hz"] > 90 and r["last_kp_max"] == 80.0 and r["last_weight"] == 1.0
    with pytest.raises(PermissionErrorMotion, match="已有别的程序在发 rt/arm_sdk"):
        m.check_free(sleep=c.sleep)


def test_foreign_expires_after_window():
    c = Clock()
    m = _mon(c)
    c.sleep(2.0)
    for k in range(20):
        m.on_message(k)
        c.sleep(0.01)
    assert m.report()["foreign"]
    c.sleep(1.5)                   # 对方停了
    assert not m.report()["foreign"]
    m.check_free(sleep=c.sleep)


def test_single_stray_message_tolerated():
    c = Clock()
    m = _mon(c)
    c.sleep(2.0)
    m.on_message(7)
    assert not m.report()["foreign"]
    m.check_free(sleep=c.sleep)


def test_check_free_waits_until_listened_long_enough():
    c = Clock()
    m = _mon(c)
    c.sleep(0.2)                   # 服务刚起 0.2 s 就接管：先补足 1 s 再判
    m.check_free(sleep=c.sleep)
    assert c.t == pytest.approx(101.0)


def test_check_free_fresh_ignores_earlier_messages():
    """G1D：退出运控之前运控自己可能也在发；fresh 只看从现在起的消息。"""
    c = Clock()
    m = _mon(c)
    for k in range(50):
        m.on_message(k)
        c.sleep(0.01)

    def sleep(dt):                 # 重新听的这段时间里没人发
        c.sleep(dt)

    m.check_free(fresh=True, sleep=sleep)


def test_own_crc_memory_is_bounded():
    c = Clock()
    m = _mon(c)
    for k in range(5000):
        m.note_own(k)
    assert len(m._own_set) <= 512
    m.on_message(4999)             # 最近的仍认得
    assert m.report()["own_hz"] == 1.0


def test_scan_processes_matches_args_and_skips_self(tmp_path, monkeypatch):
    import os
    import sys

    monkeypatch.setattr(sys, "platform", "linux")
    me = os.getpid()

    def proc(pid, *argv):
        d = tmp_path / str(pid)
        d.mkdir()
        (d / "cmdline").write_bytes(b"\0".join(a.encode() for a in argv) + b"\0")

    proc(11, "/opt/envs/teleop/bin/python", "-u", "-", "AUTO", "192.168.123.162")
    proc(12, "/usr/bin/python3", "reach_server.py", "--port", "18001")
    proc(13, "/home/u/envs/abyss/bin/python", "-m", "robot_platform.perception.lidar_worker", "--topic", "rt/x")
    proc(14, "/usr/bin/bash", "-c", "sleep 9")
    proc(me, "/usr/bin/python3", "reach_server.py")
    found = scan_processes(("reach_server", "192.168.123."), proc=tmp_path)
    assert sorted(h["pid"] for h in found) == [11, 12]
    # 只看参数：解释器路径里的 teleop 不算
    assert scan_processes(("teleop",), proc=tmp_path) == []


# ------------------------------------------------------------------ mock 服务


def test_bus_idle_then_engage_shows_own_only(mock_server):
    c = mock_server
    bus = c.get("/api/scene")["ctrl"]["cmd_bus"]
    assert bus["topic"] == "rt/lowcmd" and not bus["foreign"]
    code, d = c.post("/api/engage")
    assert code == 200 and d["ctrl"]["engaged"], d
    s = c.wait(lambda s: s["ctrl"]["cmd_bus"]["own_hz"] > 10)
    assert not s["ctrl"]["cmd_bus"]["foreign"]
    c.post("/api/release")


def test_engage_refused_when_someone_else_publishes(mock_server):
    c = mock_server
    c.post("/api/release")
    assert c.post("/api/mock/foreign", {"hz": 200})[0] == 200
    try:
        c.wait(lambda s: s["ctrl"]["cmd_bus"]["foreign"])
        code, d = c.post("/api/engage")
        assert code == 403 and "已有别的程序在发 rt/lowcmd" in d["error"], d
        assert not c.get("/api/scene")["ctrl"]["engaged"]
    finally:
        c.post("/api/mock/foreign", {"hz": 0})
    c.wait(lambda s: not s["ctrl"]["cmd_bus"]["foreign"], timeout_s=5.0)
    code, d = c.post("/api/engage")
    assert code == 200 and d["ctrl"]["engaged"], d
    c.post("/api/release")


def test_foreign_flagged_while_engaged(mock_server):
    c = mock_server
    code, d = c.post("/api/engage")
    assert code == 200, d
    c.post("/api/mock/foreign", {"hz": 100})
    try:
        s = c.wait(lambda s: s["ctrl"]["cmd_bus"]["foreign"])
        assert s["ctrl"]["engaged"] and s["ctrl"]["cmd_bus"]["own_hz"] > 10
    finally:
        c.post("/api/mock/foreign", {"hz": 0})
        c.post("/api/release")
