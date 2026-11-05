import threading
from types import SimpleNamespace

from robot_platform.hw.dds.lowstate import LowStateReader


def _reader_with(msg):
    r = object.__new__(LowStateReader)  # 不连 DDS，只测解析
    r._lock = threading.Lock()
    r._msg = msg
    r._last_t = 1.0
    return r


def test_read_imu_from_lowstate():
    msg = SimpleNamespace(imu_state=SimpleNamespace(rpy=[0.01, 0.05, 1.2], quaternion=[0.9, 0.0, 0.02, 0.4]))
    imu = _reader_with(msg).read_imu()
    assert imu == {"rpy": [0.01, 0.05, 1.2], "quat": [0.9, 0.0, 0.02, 0.4]}


def test_read_imu_missing():
    assert _reader_with(None).read_imu() is None
    assert _reader_with(SimpleNamespace()).read_imu() is None


def test_scene_has_imu_field(mock_server):
    sc = mock_server.get("/api/scene")
    assert "imu" in sc and sc["imu"] is None  # mock 没有 IMU：前端就按旧方式画（骨盆水平）
