"""雷达：机型声明、URDF 安装位姿、PointCloud2 解码、接口（仿真雷达）。"""

from types import SimpleNamespace

import numpy as np
import pytest

from robot_platform.core.robot_profile import activate, active, load_profile
from robot_platform.perception.lidar import MockLidar, decode_pointcloud2, fit_ground, link_transform
from robot_platform.perception.lidar_api import LidarAPI


def test_profile_declares_lidar():
    g1 = load_profile("g1").lidars
    assert list(g1) == ["head"]
    assert g1["head"].type == "unitree_dds" and g1["head"].link == "mid360_link"
    assert g1["head"].params["topic"] == "rt/utlidar/cloud_livox_mid360"
    # G1D 头部与 G1 相同：同一台 MID-360、同一个话题、同样的安装位姿
    g1d = load_profile("g1d").lidars
    assert list(g1d) == ["head"]
    assert g1d["head"].link == "mid360_link" and g1d["head"].params == g1["head"].params | {
        "mount_note": g1d["head"].params["mount_note"]}
    assert np.allclose(link_transform(load_profile("g1d").urdf, "torso_link", "mid360_link"),
                       link_transform(load_profile("g1").urdf, "torso_link", "mid360_link"))


def test_mid360_mount_is_upside_down_on_torso():
    p = load_profile("g1")
    T = link_transform(p.urdf, "torso_link", "mid360_link")
    assert T[:3, 3] == pytest.approx([0.0002835, 0.00003, 0.428434], abs=1e-6)
    # 横滚 180°：雷达 z 朝下；1.2 m 外的「上方」点在躯干系里落到地面附近
    p_ground = T @ np.array([0.0, 0.0, 1.2, 1.0])
    assert p_ground[2] == pytest.approx(0.428 - 1.2 * np.cos(0.0511), abs=2e-3)
    with pytest.raises(ValueError):
        link_transform(p.urdf, "torso_link", "left_wrist_yaw_link")  # 中间有活动关节


def _fake_cloud(points, intensity):
    n = len(points)
    dt = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "time"],
                   "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f4"],
                   "offsets": [0, 4, 8, 12, 16, 18], "itemsize": 22})
    arr = np.zeros(n, dtype=dt)
    arr["x"], arr["y"], arr["z"] = points[:, 0], points[:, 1], points[:, 2]
    arr["intensity"] = intensity
    F = lambda name, off, t: SimpleNamespace(name=name, offset=off, datatype=t, count=1)  # noqa: E731
    return SimpleNamespace(
        width=n, height=1, point_step=22, is_bigendian=False, data=list(arr.tobytes()),
        fields=[F("x", 0, 7), F("y", 4, 7), F("z", 8, 7), F("intensity", 12, 7), F("ring", 16, 4), F("time", 18, 7)],
        header=SimpleNamespace(frame_id="livox_frame", stamp=SimpleNamespace(sec=1, nanosec=0)),
    )


def test_decode_pointcloud2_like_mid360():
    pts = np.array([[1, 2, 3], [-1.5, 0.25, 0.5]], np.float32)
    xyz, inten = decode_pointcloud2(_fake_cloud(pts, [10, 200]))
    assert np.allclose(xyz, pts) and np.allclose(inten, [10, 200])


@pytest.fixture
def g1_lidar_api():
    old = active()
    prof = load_profile("g1")
    activate(prof)
    app = SimpleNamespace(backend=SimpleNamespace(name="mock"),
                          settings=SimpleNamespace(urdf=prof.urdf, base_link="torso_link", dds_domain=0,
                                                   network_interface="lo", unitree_sdk_path=None))
    api = LidarAPI(app)
    yield api
    api.close()
    activate(old)


def test_api_mock_cloud_in_torso_frame(g1_lidar_api):
    api = g1_lidar_api
    st = api.status()
    assert st["lidar"] == "head" and st["mock"] is True and st["running"] is False
    d = api.cloud("max_points=3000&max_range=8&color=height")
    assert d["ok"], d
    assert d["n_sent"] <= 3000 and len(d["xyz"]) == 3 * d["n_sent"] and len(d["rgb"]) == len(d["xyz"])
    xyz = np.asarray(d["xyz"]).reshape(-1, 3)
    # 仿真地面在躯干系 z≈GROUND_Z：换回来的点应有大量落在这附近
    assert np.mean(np.abs(xyz[:, 2] - MockLidar.GROUND_Z) < 0.05) > 0.3
    assert api.status()["running"] is True
    assert api.cloud("color=intensity")["ok"]
    assert api.handle_post("/api/lidar/config", {"lidar": "nope"})["ok"] is False
    assert api.handle_post("/api/lidar/config", {"lidar": "head"})["lidar"] == "head"
    assert api.handle_post("/api/lidar/stop", {})["running"] is False


def test_api_g1d_mock_lidar(mock_server):
    st = mock_server.get("/api/lidar")  # mock 默认机型 g1d
    assert st["ok"] and [l["id"] for l in st["lidars"]] == ["head"] and st["lidar"] == "head"
    d = mock_server.get("/api/lidar/cloud?max_points=500")
    assert d["ok"], d
    assert 0 < d["n_sent"] <= 500


def test_api_without_lidar(g1_lidar_api):
    from dataclasses import replace
    activate(replace(active(), lidars={}))  # 没声明雷达的机型；fixture 收尾时换回原机型
    api = g1_lidar_api
    st = api.status()
    assert st["ok"] and st["lidars"] == [] and st["lidar"] is None
    d = api.cloud("")
    assert d["ok"] is False and "没有在 robot.json 里声明雷达" in d["error"]


def test_stuck_close_does_not_block_lidar_api(g1_lidar_api, monkeypatch):
    """宇树 SDK 的 Close() 偶尔会永远卡住：断开订阅不能堵住后续的雷达请求。"""
    import threading
    import time

    from robot_platform.perception import lidar_api as mod

    api = g1_lidar_api
    assert api.cloud("max_points=500")["ok"]
    never = threading.Event()
    monkeypatch.setattr(api._dev, "close", lambda: never.wait())  # 模拟卡死的断开
    monkeypatch.setattr(mod, "IDLE_CLOSE_S", 0.0)  # 让空闲看门狗马上去断开
    api._last_use = 0.0

    done = {}

    def probe():
        time.sleep(2.5)  # 等看门狗（2 s 一轮）去断开、卡在 close 里
        done["status"] = api.status()
        done["cloud"] = api.cloud("max_points=500")

    t = threading.Thread(target=probe, daemon=True)
    t.start()
    t.join(6.0)
    never.set()
    assert not t.is_alive(), "雷达接口被卡住的断开堵住了"
    assert done["status"]["ok"] and done["cloud"]["ok"]


def test_g1_lidar_mount_override_drops_urdf_pitch(g1_lidar_api):
    """robot.json 的 mount_rpy 覆盖 URDF 的安装姿态：只留倒装，去掉宇树已补偿过的 2.93° 俯仰；位置仍取 URDF。"""
    import math

    api = g1_lidar_api
    assert api.cloud("max_points=500")["ok"]
    T = api._T
    np.testing.assert_allclose(T[:3, :3], np.diag([1.0, -1.0, -1.0]), atol=1e-9)
    np.testing.assert_allclose(T[:3, 3], [0.0002835, 0.00003, 0.428434], atol=1e-9)
    assert abs(math.degrees(active().lidars["head"].params["mount_rpy"][1])) < 1e-9


def test_fit_ground_finds_floor_and_ignores_box_and_wall():
    api_T = np.eye(4)
    xyz = MockLidar(api_T).latest()["xyz"]
    g = fit_ground(xyz)
    assert g is not None
    assert g["height_m"] == pytest.approx(-MockLidar.GROUND_Z, abs=0.01)
    assert np.asarray(g["n"]) @ [0, 0, 1] > 0.999 and g["rms_mm"] < 20
    # 「上」偏了几度也照样找得到；没有地面（只有墙）就返回 None
    assert fit_ground(xyz, up=(0.05, 0.0, 1.0)) is not None
    wall = xyz[np.abs(xyz[:, 0] - 3.0) < 0.05]
    assert fit_ground(wall) is None


def test_api_cloud_reports_ground(g1_lidar_api):
    d = g1_lidar_api.cloud("max_points=3000")
    assert d["ok"] and d["ground"] is not None
    assert d["ground"]["height_m"] == pytest.approx(-MockLidar.GROUND_Z, abs=0.02)
