"""传感器位置标记（/api/sensors）：相机优先用手眼标定结果，没有时用 URDF 标称连杆；雷达用 URDF 连杆。"""

import json

import numpy as np
import pytest

from robot_platform.calib.handeye import results as he_results
from robot_platform.calib.handeye.config import load_config, save_config
from robot_platform.control.urdf_fk import make_transform
from robot_platform.core.robot_profile import load_profile
from robot_platform.perception.sensors import R_BODY_OPTICAL, sensor_list


@pytest.fixture
def no_handeye():
    cfg = load_config()
    old = cfg.get("active_result")
    cfg["active_result"] = ""
    save_config(cfg)
    yield
    cfg["active_result"] = old
    save_config(cfg)


def _by_id(sensors):
    return {s["id"]: s for s in sensors}


def test_g1d_nominal_camera_and_lidar(no_handeye):
    s = _by_id(sensor_list(load_profile("g1d")))
    cam, lid = s["camera:head"], s["lidar:head"]
    assert cam["kind"] == "camera" and cam["parent_link"] == "d435_link" and cam["calibrated"] is False
    assert lid["kind"] == "lidar" and lid["parent_link"] == "mid360_link"
    # URDF 相机连杆是 ROS 机体系：换到光学系后视线（光学 z）= 连杆 x
    T = np.asarray(cam["T"])
    assert np.allclose(T[:3, 2], [1, 0, 0]) and np.allclose(T[:3, :3], R_BODY_OPTICAL)
    assert np.isclose(np.linalg.det(R_BODY_OPTICAL), 1.0)


def test_g1d_nominal_camera_looks_forward_and_down(no_handeye):
    """URDF d435_link 标称位姿：在躯干前上方、视线朝前下约 47.6°，与手眼标定的实测（约 46.5°）相符。"""
    p = load_profile("g1d")
    cam = _by_id(sensor_list(p))["camera:head"]
    import xml.etree.ElementTree as ET

    j = next(j for j in ET.parse(p.urdf).getroot().iter("joint") if j.get("name") == "d435_joint")
    assert j.find("parent").get("link") == p.base_link
    o = j.find("origin")
    T_link = make_transform([float(v) for v in o.get("xyz").split()], [float(v) for v in o.get("rpy").split()])
    T = T_link @ np.asarray(cam["T"])
    z = T[:3, 2]
    assert z[0] > 0.6 and z[2] < -0.6  # 朝前、朝下
    assert np.degrees(np.arctan2(-z[2], z[0])) == pytest.approx(47.6, abs=0.5)
    assert T[2, 3] == pytest.approx(0.43, abs=0.01)


def test_calibrated_stereo_uses_handeye_result():
    """有手眼结果：左右眼各一个标记，挂在躯干上，位姿就是 T_cam2base。"""
    T_left = np.eye(4)
    T_left[:3, 3] = [0.061, 0.028, 0.435]
    T_right = np.eye(4)
    T_right[:3, 3] = [0.060, -0.031, 0.437]
    doc = {"name": "单测手眼", "result": {
        "left": {"T_cam2base": T_left.tolist(), "base_link": "torso_link", "residual_translation_mm": {"mean": 3.4}},
        "right": {"T_cam2base": T_right.tolist(), "base_link": "torso_link"},
    }}
    d = he_results.results_dir()
    d.mkdir(parents=True, exist_ok=True)
    path = d / "单测手眼.json"
    path.write_text(json.dumps(doc, ensure_ascii=False), encoding="utf-8")
    cfg = load_config()
    old = cfg.get("active_result")
    cfg["active_result"] = "单测手眼"
    save_config(cfg)
    try:
        s = _by_id(sensor_list(load_profile("g1d")))
        left, right = s["camera:head:left"], s["camera:head:right"]
        assert "camera:head" not in s and "lidar:head" in s
        assert left["parent_link"] == "torso_link" and left["calibrated"] is True
        assert np.allclose(left["T"], T_left) and np.allclose(right["T"], T_right)
        assert left["label"] == "头部双目 左眼" and right["label"] == "头部双目 右眼"
        assert "单测手眼" in left["source"] and "3.4 mm" in left["source"]
    finally:
        cfg["active_result"] = old
        save_config(cfg)
        path.unlink()


def test_g1_lidar_declared():
    s = _by_id(sensor_list(load_profile("g1")))
    assert s["lidar:head"]["parent_link"] == "mid360_link"


def test_api_sensors(mock_server):
    d = mock_server.get("/api/sensors")
    assert d["ok"] is True
    kinds = sorted(s["kind"] for s in d["sensors"])
    assert "lidar" in kinds and "camera" in kinds
    for s in d["sensors"]:
        assert np.asarray(s["T"]).shape == (4, 4) and s["parent_link"]
