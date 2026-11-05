"""机型声明的相机：默认选第一台、实例旧写法 camera_source 兼容、下拉框选择、mock 自动换仿真相机。"""

import json

import pytest

from robot_platform.calib.handeye import config as he_config
from robot_platform.core import instance as instances
from robot_platform.core.robot_profile import activate, active, load_profile


@pytest.fixture
def profile_g1():
    old = active()
    activate(load_profile("g1"))
    yield
    activate(old)


def _write(cfg):
    p = instances.path("handeye_config")
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(cfg), encoding="utf-8")


def test_profiles_declare_cameras():
    g1 = load_profile("g1").cameras
    g1d = load_profile("g1d").cameras
    assert list(g1) == ["head"] and g1["head"].type == "ros_rgbd"
    assert list(g1d) == ["head"] and g1d["head"].type == "teleimager"
    assert g1d["head"].params["host"] == "192.168.123.164"


def test_default_is_first_camera_and_mock_maps(profile_g1):
    _write({})
    eff = he_config.effective_camera(he_config.load_config())
    assert eff["id"] == "head" and eff["camera_source"] == "ros_rgbd" and eff["camera_kind"] == "rgbd"
    mock = he_config.effective_camera(he_config.load_config(), mock=True)
    assert mock["camera_source"] == "mock_rgbd" and mock["camera_kind"] == "rgbd"


def test_legacy_camera_source_and_overrides(profile_g1):
    # 旧实例：直接写来源 + 主机；对得上机型里的相机就当作那一台
    _write({"camera_source": "ros_rgbd", "camera_host": "1.2.3.4", "ros_topics": {"color": "/x/color"}})
    eff = he_config.effective_camera(he_config.load_config())
    assert eff["id"] == "head" and eff["camera_host"] == "1.2.3.4"
    assert eff["ros_topics"]["color"] == "/x/color" and eff["ros_topics"]["depth"] == "/camera/depth/image_rect_raw"
    # 对不上：照旧按来源打开，id 为空（下拉框显示「实例配置」）
    _write({"camera_source": "mock"})
    eff = he_config.effective_camera(he_config.load_config())
    assert eff["id"] is None and eff["camera_source"] == "mock" and eff["camera_kind"] == "stereo"
    # 选了相机 id 优先于旧来源
    _write({"camera_source": "mock", "camera": "head"})
    assert he_config.effective_camera(he_config.load_config())["camera_source"] == "ros_rgbd"


def test_select_camera_via_api(mock_server):
    c = mock_server
    code, st = c.post("/api/handeye/config", {"camera_source": "mock"})
    assert st["camera_kind"] == "stereo" and st["camera"] is None  # 显式来源对不上机型相机：按「实例配置」
    assert [x["id"] for x in st["cameras"]] == ["head"]
    code, st = c.post("/api/handeye/config", {"camera": "head"})
    assert st["ok"] and st["camera"] == "head" and st["camera_source"] == "mock", "mock 后端下换成仿真相机"
    code, st = c.post("/api/handeye/config", {"camera": "wrist"})
    assert st["ok"] is False and "没有相机" in st["error"]
