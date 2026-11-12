"""底盘遥控：按住才走（心跳超时停）、限速、出错 / 急停立刻停、接口与仿真底盘。"""

import threading
import time

import pytest

from robot_platform.core.robot_profile import load_profile
from robot_platform.hw import chassis_control as cc
from robot_platform.hw.backends.base import ChassisDriver
from robot_platform.hw.chassis_control import ChassisController

SPEC = load_profile("g1d").chassis


class FakeDriver(ChassisDriver):
    def __init__(self, code: int = 0) -> None:
        self.code = code
        self.calls = []
        self.lock = threading.Lock()

    def move(self, vx, vyaw):
        with self.lock:
            self.calls.append((time.monotonic(), vx, vyaw))
        return self.code

    def last(self):
        with self.lock:
            return self.calls[-1]


def _wait(pred, timeout=2.0):
    t = time.monotonic() + timeout
    while time.monotonic() < t:
        if pred():
            return True
        time.sleep(0.01)
    return False


def test_profile_declares_chassis():
    assert SPEC is not None and SPEC.odom_topic == "rt/agv/odom"
    assert SPEC.default_lin_mps == 0.15 and SPEC.max_lin_mps <= 1.5 and SPEC.max_yaw_rad_s <= 0.6
    assert load_profile("g1").chassis is None


def test_heartbeat_scales_by_limit_and_deadman_stops():
    d = FakeDriver()
    c = ChassisController(d, None, SPEC)
    c.command(1.0, -0.5)
    assert _wait(lambda: d.calls and d.last()[1] > 0)
    _, vx, wz = d.last()
    assert vx == pytest.approx(0.15) and wz == pytest.approx(-0.1)  # 默认 0.15 m/s、0.2 rad/s
    assert c.status()["moving"] is True
    # 不再发心跳：DEADMAN_S 后自己发 0
    t0 = time.monotonic()
    assert _wait(lambda: d.last()[1] == 0.0 and d.last()[2] == 0.0, timeout=cc.DEADMAN_S + 1.0)
    assert time.monotonic() - t0 < cc.DEADMAN_S + 0.5
    assert _wait(lambda: c.status()["moving"] is False)
    assert "心跳" in c.status()["stop_reason"]
    # 线程退出后不再发
    assert _wait(lambda: c._thread is None)
    n = len(d.calls)
    time.sleep(0.3)
    assert len(d.calls) == n


def test_lin_and_yaw_limits_are_independent_and_clamped():
    d = FakeDriver()
    c = ChassisController(d, None, SPEC)
    st = c.set_limit(lin_mps=0.3)
    assert st["limit"]["lin_mps"] == pytest.approx(0.3) and st["limit"]["yaw_rad_s"] == pytest.approx(SPEC.default_yaw_rad_s)
    st = c.set_limit(yaw_rad_s=0.4)
    assert st["limit"]["yaw_rad_s"] == pytest.approx(0.4) and st["limit"]["lin_mps"] == pytest.approx(0.3)
    assert c.set_limit(lin_mps=9.0)["limit"]["lin_mps"] == SPEC.max_lin_mps
    assert c.set_limit(yaw_rad_s=9.0)["limit"]["yaw_rad_s"] == SPEC.max_yaw_rad_s
    assert c.set_limit(lin_mps=0.0)["limit"]["lin_mps"] == cc.MIN_LIN_MPS
    assert c.set_limit(yaw_rad_s=0.0)["limit"]["yaw_rad_s"] == cc.MIN_YAW_RAD_S
    with pytest.raises(ValueError):
        c.set_limit(lin_mps=float("nan"))
    with pytest.raises(ValueError):
        c.set_limit()
    # 转向限速直接决定转向速度
    c.set_limit(yaw_rad_s=0.3)
    c.command(0.0, 1.0)
    assert _wait(lambda: d.calls and d.last()[2] > 0)
    assert d.last()[2] == pytest.approx(0.3)
    c.stop()


def test_stop_sends_zero_immediately():
    d = FakeDriver()
    c = ChassisController(d, None, SPEC)
    c.command(1.0, 0.0)
    assert _wait(lambda: d.calls)
    c.stop("急停")
    _, vx, wz = d.last()
    assert vx == 0.0 and wz == 0.0
    assert c.status()["stop_reason"] == "急停"
    assert _wait(lambda: c._thread is None)
    assert all(v == 0.0 for _, v, _ in d.calls[-cc.ZERO_REPEAT:])


def test_error_code_stops():
    d = FakeDriver(code=-1)
    c = ChassisController(d, None, SPEC)
    c.command(1.0, 0.0)
    assert _wait(lambda: "返回" in c.status()["stop_reason"])
    assert _wait(lambda: d.last()[1] == 0.0)


def test_api_drives_mock_base_and_estop_stops_it(mock_server):
    c = mock_server
    caps = c.get("/api/robot")
    assert caps["chassis"]["max_lin_mps"] == SPEC.max_lin_mps
    scene = c.get("/api/scene")
    assert scene["chassis"]["available"] is False and scene["chassis"]["odom"] is not None
    x0 = scene["chassis"]["odom"]["x"]
    code, d = c.post("/api/chassis/limit", {"lin_mps": 0.3})
    assert code == 200 and d["ok"] and d["limit"]["lin_mps"] == pytest.approx(0.3)
    code, d = c.post("/api/chassis/limit", {"yaw_rad_s": 0.25})
    assert d["ok"] and d["limit"]["yaw_rad_s"] == pytest.approx(0.25) and d["limit"]["lin_mps"] == pytest.approx(0.3)
    for _ in range(6):
        code, d = c.post("/api/chassis/move", {"x": 1.0, "yaw": 0.0})
        assert code == 200 and d["ok"], d
        time.sleep(0.1)
    odom = c.get("/api/scene")["chassis"]["odom"]
    assert odom["vx"] == pytest.approx(0.3) and odom["x"] > x0 + 0.05
    # 急停（不接管也能停底盘）
    code, d = c.post("/api/hold")
    assert code == 200
    assert _wait(lambda: c.get("/api/scene")["chassis"]["odom"]["vx"] == 0.0, timeout=1.0)
    assert c.get("/api/scene")["chassis"]["stop_reason"] == "急停"
    assert c.post("/api/chassis/stop")[1]["ok"]
    c.post("/api/chassis/limit", {"lin_mps": SPEC.default_lin_mps, "yaw_rad_s": SPEC.default_yaw_rad_s})
