"""雷达：机型声明、URDF 安装位姿、PointCloud2 解码、接口（仿真雷达）。"""

from types import SimpleNamespace

import numpy as np
import pytest

from robot_platform.core.robot_profile import activate, active, load_profile
from robot_platform.perception.lidar import decode_pointcloud2, link_transform
from robot_platform.perception.lidar_api import LidarAPI


def test_profile_declares_lidar():
    g1 = load_profile("g1").lidars
    assert list(g1) == ["head"]
    assert g1["head"].type == "unitree_dds" and g1["head"].link == "mid360_link"
    assert g1["head"].params["topic"] == "rt/utlidar/cloud_livox_mid360"
    assert load_profile("g1d").lidars == {}


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
    # 仿真地面在躯干系 z≈-0.75：换回来的点应有大量落在这附近
    assert np.mean(np.abs(xyz[:, 2] + 0.75) < 0.05) > 0.3
    assert api.status()["running"] is True
    assert api.cloud("color=intensity")["ok"]
    assert api.handle_post("/api/lidar/config", {"lidar": "nope"})["ok"] is False
    assert api.handle_post("/api/lidar/config", {"lidar": "head"})["lidar"] == "head"
    assert api.handle_post("/api/lidar/stop", {})["running"] is False


def test_api_without_lidar(mock_server):
    st = mock_server.get("/api/lidar")  # mock 默认机型 g1d，没有声明雷达
    assert st["ok"] and st["lidars"] == [] and st["lidar"] is None
    d = mock_server.get("/api/lidar/cloud")
    assert d["ok"] is False and "没有在 robot.json 里声明雷达" in d["error"]
