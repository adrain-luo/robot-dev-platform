"""RGB-D 相机（G1 的 D435i）：深度点云换算、手眼求解全流程（合成棋盘图）、mock 相机下的采集接口。"""

import json

import cv2
import numpy as np
import pytest

from robot_platform.perception import rgbd
from robot_platform.perception.camera import MockRgbdCamera

K_COLOR = [607.45, 0, 326.83, 0, 606.94, 247.34, 0, 0, 1]
K_DEPTH = [390.15, 0, 312.64, 0, 390.15, 240.39, 0, 0, 1]


def _info(t=(0.015, 0.0, 0.0)):
    return {
        "color": {"size": [640, 480], "K": K_COLOR, "D": [0.0] * 5},
        "depth": {"size": [640, 480], "K": K_DEPTH, "D": [0.0] * 5},
        "depth_scale_m": 0.001,
        "depth_to_color": {"R": np.eye(3).tolist(), "t": list(t)},
    }


def test_depth_plane_lands_in_color_frame():
    depth = np.full((480, 640), 1200, np.uint16)
    depth[:, :8] = 0
    color = np.zeros((480, 640, 3), np.uint8)
    color[:, :, 2] = 255  # 红
    pts, cols = rgbd.depth_to_color_cloud(depth, color, _info())
    # 深度视场比彩色宽（fx 390 对 607），边缘一圈落在彩色图外，被丢掉
    assert 0.3 * 480 * 640 < pts.shape[0] < 0.6 * 480 * 640
    assert np.allclose(pts[:, 2], 1.2, atol=1e-9), "纯平移外参不改深度"
    # 深度主点处的点：彩色系 x = 0.015 m
    Kd = np.asarray(K_DEPTH).reshape(3, 3)
    u = round(Kd[0, 2])
    x = (u - Kd[0, 2]) * 1.2 / Kd[0, 0] + 0.015
    assert np.min(np.abs(pts[:, 0] - x)) < 1e-3
    assert (cols == [0, 0, 255]).all()


def test_depth_far_and_near_filtered():
    depth = np.zeros((480, 640), np.uint16)
    depth[0:10] = 100     # 0.1 m，太近
    depth[10:20] = 20000  # 20 m，太远
    depth[20:30] = 800
    pts, _ = rgbd.depth_to_color_cloud(depth, np.zeros((480, 640, 3), np.uint8), _info((0, 0, 0)))
    assert np.allclose(pts[:, 2], 0.8)


def _render_board(K, T_board2cam, cols, rows, sq_m, size=(640, 480)):
    """画一张黑白棋盘（内角点 cols×rows，格边 sq_m），带白边，便于 findChessboardCornersSB。"""
    img = np.full((size[1], size[0]), 255, np.uint8)
    R, t = T_board2cam[:3, :3], T_board2cam[:3, 3]
    rvec, _ = cv2.Rodrigues(R)
    for i in range(-1, cols):
        for j in range(-1, rows):
            if (i + j) % 2:
                continue
            quad = np.array([[i, j, 0], [i + 1, j, 0], [i + 1, j + 1, 0], [i, j + 1, 0]], float) * sq_m
            uv, _ = cv2.projectPoints(quad, rvec, t, K, np.zeros(5))
            cv2.fillConvexPoly(img, np.round(uv.reshape(-1, 2) * 16).astype(np.int32), 0, cv2.LINE_AA, shift=4)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def test_rgbd_handeye_solve_recovers_camera_pose(tmp_path):
    """合成：已知相机在躯干的位姿、板在左腕上；按 G1 正运动学渲染各姿态的棋盘，走 RGB-D 求解流程。"""
    from robot_platform.core.robot_profile import load_profile
    from robot_platform.control.urdf_fk import URDFRobotModel
    from robot_platform.calib.handeye.solve_core import SolveOptions, load_intrinsics, make_T, object_points_m, solve_eye

    p = load_profile("g1")
    model = URDFRobotModel(str(p.urdf), "torso_link", "left_wrist_yaw_link")
    K = np.asarray(K_COLOR).reshape(3, 3)
    cols, rows, sq_mm = 6, 5, 30.0
    base_q = np.array([-0.9, 0.3, 0.0, 0.4, 0.0, 0.0, 0.0])
    p0, R0 = model.fk(base_q)

    def look_at(eye, target):
        z = (target - eye) / np.linalg.norm(target - eye)
        x = np.cross(z, [0, 0, 1.0])
        x /= np.linalg.norm(x)
        return np.c_[x, np.cross(z, x), z]  # 光学系：X 右、Y 下、Z 前

    # 相机放在 D435 安装位（URDF d435_joint 附近），光轴对准基准姿态的手腕
    cam_pos = np.array([0.058, 0.018, 0.43])
    T_c2b = make_T(look_at(cam_pos, p0), cam_pos)
    # 板在基准姿态下正对相机、中心在腕前 5 cm；之后钉死在腕上
    board_center = p0 + R0 @ np.array([0.05, 0, 0])
    R_bd = look_at(cam_pos, board_center)  # 板面法向沿视线
    half = np.array([(cols - 1) / 2, (rows - 1) / 2, 0]) * sq_mm / 1000
    T_board_base = make_T(R_bd, board_center - R_bd @ half)
    T_board2wrist = np.linalg.inv(make_T(R0, p0)) @ T_board_base

    data = tmp_path / "s"
    for sub in ("left", "depth", "joints"):
        (data / sub).mkdir(parents=True)
    rng = np.random.default_rng(3)
    saved = 0
    for k in range(80):
        q = base_q + rng.uniform(-1, 1, 7) * np.array([0.15, 0.15, 0.2, 0.15, 0.35, 0.3, 0.3])
        p_w, R_w = model.fk(q)
        T_board2cam = np.linalg.inv(T_c2b) @ make_T(R_w, p_w) @ T_board2wrist
        corners = (T_board2cam @ np.c_[object_points_m((cols, rows), sq_mm), np.ones(cols * rows)].T)[:3]
        if corners[2].min() < 0.25:
            continue
        uv = K @ corners
        uv = uv[:2] / uv[2]
        if uv.min() < 40 or uv[0].max() > 600 or uv[1].max() > 440:
            continue
        img = _render_board(K, T_board2cam, cols, rows, sq_mm / 1000)
        stem = f"{saved:04d}"
        cv2.imwrite(str(data / "left" / f"{stem}.jpg"), img)
        (data / "joints" / f"{stem}.json").write_text(json.dumps({
            "q_rad": q.tolist(), "joint_names": list(model.joint_names)}), encoding="utf-8")
        saved += 1
        if saved >= 14:
            break
    assert saved >= 10, "合成姿态太少"

    fit = rgbd.write_session_intrinsics(data, _info(), sq_mm, saved)
    assert fit["camera"] == "rgbd" and fit["left_fx"] == pytest.approx(607.45)
    Kr, dist = load_intrinsics(data / "stereo_calibration.yaml", "left")
    assert np.allclose(Kr, K) and np.allclose(dist, 0)
    opt = SolveOptions(base_link="torso_link", tip_link="left_wrist_yaw_link", square_size_mm=sq_mm,
                       intrinsics=str(data / "stereo_calibration.yaml"))
    res = solve_eye("left", data, Kr, dist, object_points_m((cols, rows), sq_mm), (cols, rows), model, opt)
    assert res is not None
    T = np.asarray(res["T_cam2base"])
    dt_mm = np.linalg.norm(T[:3, 3] - T_c2b[:3, 3]) * 1000
    dR = T[:3, :3].T @ T_c2b[:3, :3]
    dr_deg = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
    print(f"合成 {saved} 张，内点 {res['num_inliers']}：平移误差 {dt_mm:.3f} mm，旋转误差 {dr_deg:.4f}°")
    assert dt_mm < 2.0 and dr_deg < 0.3


def test_mock_rgbd_frame_shape():
    f = MockRgbdCamera().grab()
    assert f.color.shape == (480, 640, 3) and f.depth.dtype == np.uint16 and f.right is None
    pts, _ = rgbd.depth_to_color_cloud(f.depth, f.color, f.info)
    assert pts.shape[0] > 100000


def test_rgbd_capture_and_cloud_via_api(mock_server):
    """mock_rgbd 相机：补一帧存彩色 + 深度 + 相机参数；没有手眼结果时点云给出明确提示。"""
    c = mock_server
    code, st = c.post("/api/handeye/config", {"camera_source": "mock_rgbd"})
    assert code == 200 and st["camera_kind"] == "rgbd"
    code, st = c.post("/api/handeye/session", {"create": True})
    assert code == 200
    code, st = c.post("/api/handeye/capture", {})
    assert code == 200 and st.get("index") == 0, st
    code, d = c.post("/api/handeye/match", {})
    assert d["ok"] is False and "外参" in d["error"]
    code, d = c.post("/api/handeye/match", {"source": "debug"})
    assert d["ok"] is False and "双目" in d["error"]
    c.post("/api/handeye/config", {"camera_source": "mock"})
