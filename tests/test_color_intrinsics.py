"""RGB-D 自标定彩色内参：文件解析、分辨率换算、带畸变取色、上传 / 删除接口、点云匹配选用。"""

import json

import numpy as np
import pytest

from robot_platform.calib.handeye import color_intrinsics as he_color_intr
from robot_platform.perception import rgbd
from robot_platform.perception.camera import MockRgbdCamera

# 2026-11-04 读到的 G1 D435i 出厂彩色内参
F720 = [911.1815185546875, 0, 650.241943359375, 0, 910.4107666015625, 371.0053405761719, 0, 0, 1]
F480 = [607.454345703125, 0, 326.8279724121094, 0, 606.9404907226562, 247.33689880371094, 0, 0, 1]

DOC = {
    "camera_matrix": [[907.17, 0.0, 653.0], [0.0, 906.40, 370.89], [0.0, 0.0, 1.0]],
    "distortion": [0.1349, -0.3172, -0.0017, -0.00056, 0.1064],
    "distortion_model": "opencv_brown_conrady",
    "image_size": [1280, 720],
    "rms_px": 0.145,
    "saved_at": "2026-11-03T17:51:19",
}


def test_adapt_matches_factory_pair():
    """出厂 1280×720 → 640×480 的换算与出厂 640×480 内参一致（按高缩放 + 居中裁切）。"""
    got = rgbd.adapt_intrinsics({"K": F720, "D": [0] * 5, "size": [1280, 720]}, [640, 480])
    assert np.allclose(got["K"], F480, atol=1e-3)
    assert got["converted_from"] == [1280, 720]
    same = rgbd.adapt_intrinsics({"K": F480, "D": [0] * 5, "size": [640, 480]}, [640, 480])
    assert same["converted_from"] is None and np.allclose(same["K"], F480)


def test_parse_formats_and_errors():
    a = rgbd.parse_color_intrinsics(DOC)
    assert a["size"] == [1280, 720] and len(a["K"]) == 9 and a["K"][0] == pytest.approx(907.17)
    b = rgbd.parse_color_intrinsics({"K": F480, "D": [0.0] * 5, "size": [640, 480]})
    assert b["K"] == F480
    with pytest.raises(ValueError):
        rgbd.parse_color_intrinsics({"camera_matrix": [[1, 0], [0, 1]], "image_size": [640, 480]})
    with pytest.raises(ValueError):
        rgbd.parse_color_intrinsics(dict(DOC, image_size=[100, 100]))  # 主点出图
    with pytest.raises(ValueError):
        rgbd.parse_color_intrinsics(dict(DOC, distortion=[0.1, 0.2, 0.3]))


def test_distortion_moves_color_lookup():
    """有畸变时取色位置按 K+D 投影：在彩色图上画一条竖线，点云上染色的点随畸变移动。"""
    f = MockRgbdCamera().grab()
    color = np.zeros_like(f.color)
    color[:, 600:] = (0, 0, 255)  # 右缘一条红带
    pts0, c0 = rgbd.depth_to_color_cloud(f.depth, color, f.info)
    info = json.loads(json.dumps(f.info))
    info["color"]["D"] = [0.2, 0.0, 0.0, 0.0, 0.0]  # k1>0：投影点向外移，边缘一圈出图
    pts1, c1 = rgbd.depth_to_color_cloud(f.depth, color, info)
    red0 = int((c0[:, 2] == 255).sum())
    red1 = int((c1[:, 2] == 255).sum())
    assert pts1.shape[0] < pts0.shape[0]
    assert red1 < red0, (red0, red1)  # 同一条红带对应的三维点变少（被推向图外）
    # 点的三维位置不变（只是取色位置变了，外移的点出图会被丢）
    assert np.allclose(np.median(pts0[:, 2]), np.median(pts1[:, 2]))


def test_apply_to_info_reports_diff():
    he_color_intr.save("t_apply", DOC, force=True)
    f = MockRgbdCamera().grab()
    f.info["color"]["K"] = F480
    out = he_color_intr.apply_to_info(f.info, "t_apply")
    sm = out["summary"]
    assert sm["converted_from"] == [1280, 720] and sm["size"] == [640, 480]
    assert sm["d_fx"] == pytest.approx(-2.67, abs=0.02) and sm["d_cx"] == pytest.approx(1.84, abs=0.02)
    assert out["info"]["color"]["D"] == DOC["distortion"]
    assert f.info["color"]["K"] == F480, "不改原帧"
    he_color_intr.delete("t_apply")


def test_upload_list_delete_api(mock_server):
    c = mock_server
    code, st = c.post("/api/handeye/color-intrinsics/upload", {"name": "head_new", "doc": json.dumps(DOC)})
    assert code == 200 and st["ok"] and st["saved_name"] == "head_new", st
    row = next(f for f in st["color_intrinsics"] if f["name"] == "head_new")
    assert row["size"] == [1280, 720] and row["fx"] == pytest.approx(907.17) and row["rms_px"] == pytest.approx(0.145)
    code, st = c.post("/api/handeye/color-intrinsics/upload", {"name": "head_new", "doc": DOC})
    assert st["ok"] is False and st.get("exists")
    code, st = c.post("/api/handeye/color-intrinsics/upload", {"name": "head_new", "doc": DOC, "force": True})
    assert st["ok"]
    code, st = c.post("/api/handeye/color-intrinsics/upload", {"name": "bad", "doc": "{not json"})
    assert st["ok"] is False and "JSON" in st["error"]
    code, st = c.post("/api/handeye/color-intrinsics/upload", {"name": "../x", "doc": DOC})
    assert st["ok"] is False
    code, st = c.post("/api/handeye/color-intrinsics/delete", {"name": "head_new"})
    assert st["ok"] and all(f["name"] != "head_new" for f in st["color_intrinsics"])


def test_rgbd_cloud_with_selected_intrinsics():
    from robot_platform.calib.handeye.config import load_config
    from robot_platform.calib.handeye.service import HandEyeAPI

    he_color_intr.save("t_cloud", DOC, force=True)
    api = HandEyeAPI(None)
    f = MockRgbdCamera().grab()
    d0 = api._rgbd_cloud(f, np.eye(4))
    assert d0["ok"] and d0["color_intrinsics"] is None and "出厂" in d0["note"]
    d1 = api._rgbd_cloud(f, np.eye(4), "t_cloud")
    assert d1["ok"] and d1["color_intrinsics"]["name"] == "t_cloud"
    assert "1280×720 换算到 640×480" in d1["note"]
    assert not load_config().get("cloud_color_intrinsics"), "生成点云只用不改选择"
    bad = api._rgbd_cloud(f, np.eye(4), "no_such")
    assert bad["ok"] is False and "没有内参文件" in bad["error"]
    he_color_intr.delete("t_cloud")


def test_selected_intrinsics_apply_everywhere_and_lock(mock_server):
    """相机卡片选内参即写入实例；不带参数的匹配用它；配置锁定时不能改、不能删正在用的。"""
    c = mock_server
    c.post("/api/handeye/config", {"camera_source": "mock_rgbd"})
    code, st = c.post("/api/handeye/color-intrinsics/upload", {"name": "sel", "doc": DOC, "force": True})
    assert st["ok"]
    code, st = c.post("/api/handeye/config", {"color_intrinsics": "sel"})
    assert st["ok"] and st["cloud_color_intrinsics"] == "sel"
    code, st = c.post("/api/handeye/config", {"color_intrinsics": "no_such"})
    assert st["ok"] is False and "没有内参文件" in st["error"]

    c.post("/api/runtime-config/lock", {"locked": True})
    try:
        st = c.get("/api/handeye")
        assert st["config_locked"] is True
        code, st = c.post("/api/handeye/config", {"color_intrinsics": ""})
        assert st["ok"] is False and "锁定" in st["error"]
        code, st = c.post("/api/handeye/config", {"camera": "head"})
        assert st["ok"] is False and "锁定" in st["error"]
        code, st = c.post("/api/handeye/color-intrinsics/delete", {"name": "sel"})
        assert st["ok"] is False and "正在用" in st["error"]
        code, st = c.post("/api/handeye/config", {"board_size": "11x8"})
        assert st["ok"], "锁定只管相机与内参，别的手眼表单照常"
    finally:
        c.post("/api/runtime-config/lock", {"locked": False})
    code, st = c.post("/api/handeye/config", {"color_intrinsics": ""})
    assert st["ok"] and st["cloud_color_intrinsics"] == ""
    c.post("/api/handeye/color-intrinsics/delete", {"name": "sel"})
    c.post("/api/handeye/config", {"camera_source": "mock"})


def test_session_intrinsics_from_selected_file(tmp_path):
    """RGB-D 手眼求解写会话内参：选了自标内参就用它（含畸变），并记下来源。"""
    import cv2

    f = MockRgbdCamera().grab()
    f.info["color"]["K"] = F480
    he_color_intr.save("t_solve", DOC, force=True)
    info = he_color_intr.apply_to_info(f.info, "t_solve")["info"]
    fit = rgbd.write_session_intrinsics(tmp_path, info, 15.0, 3, source="t_solve")
    assert fit["source"] == "t_solve" and "t_solve" in fit["note"]
    fs = cv2.FileStorage(str(tmp_path / "stereo_calibration.yaml"), cv2.FILE_STORAGE_READ)
    K = fs.getNode("left_camera_matrix").mat()
    D = fs.getNode("left_dist_coeffs").mat().ravel()
    fs.release()
    assert K[0, 0] == pytest.approx(907.17 * 2 / 3, abs=0.01) and D[0] == pytest.approx(0.1349)
    fit0 = rgbd.write_session_intrinsics(tmp_path, f.info, 15.0, 3)
    assert fit0["source"] == "factory"
    he_color_intr.delete("t_solve")


def test_match_without_param_uses_selected(monkeypatch):
    """渲染卡片 / 相机卡片的「匹配点云」不带参数：用实例里选定的相机与内参。"""
    from robot_platform.calib.handeye import config as he_config
    from robot_platform.calib.handeye.service import HandEyeAPI

    he_color_intr.save("t_match", DOC, force=True)
    cfg = he_config.load_config()
    cfg.pop("camera", None)  # 别的测试可能选过机型相机；这里显式用仿真 RGB-D
    cfg.update(camera_source="mock_rgbd", cloud_color_intrinsics="t_match")
    he_config.save_config(cfg)
    api = HandEyeAPI(None)
    monkeypatch.setattr(api, "_left_cam_to_torso", lambda cfg: np.eye(4))
    try:
        d = api._match({})
        assert d["ok"], d
        assert d["color_intrinsics"]["name"] == "t_match"
        d = api._match({"color_intrinsics": ""})  # 显式给出时按给的
        assert d["ok"] and d["color_intrinsics"] is None
    finally:
        cfg.pop("cloud_color_intrinsics", None)
        cfg.pop("camera_source", None)
        he_config.save_config(cfg)
        he_color_intr.delete("t_match")
        api.close()
