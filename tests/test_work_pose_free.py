"""可平转工作姿态：只锁相对重力的倾斜，绕重力方向（竖直 z）转不约束。"""

import math

import numpy as np
import pytest

from robot_platform.control.fk_scene import RobotKinematics
from robot_platform.control.gravity_dir import GravityDirection
from robot_platform.core.robot_profile import active
from robot_platform.tasks.spatial_points import (
    axis_angle_R,
    nearest_about_axis,
    pose_ori_miss_deg,
    solve_tcp_pose_ik,
    solve_tcp_pose_ik_robust_local,
)
from robot_platform.tasks.work_pose import WorkPoseSession

UP = np.array([0.0, 0.0, 1.0])


def _ang(Ra, Rb) -> float:
    return math.degrees(math.acos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1)))


@pytest.fixture(scope="module")
def arm():
    p = active()
    model = RobotKinematics(p.urdf, p.base_link, list(p.arms)).arms["right"].model
    q0 = np.asarray(p.arm("right").home_q, float)
    tcp = np.array([0.0, 0.0, 0.1])
    t, R = model.fk(q0)
    return model, q0, tcp, t + R @ tcp, R


def test_nearest_about_axis_recovers_rotation():
    R_now = axis_angle_R([0.3, -0.5, 0.8], 50.0)
    u = np.array([0.05, -0.07, 1.0])
    u /= np.linalg.norm(u)
    for deg in (-150.0, -37.0, 0.0, 12.0, 179.0):
        R_saved = axis_angle_R(u, deg) @ R_now
        assert _ang(nearest_about_axis(R_saved, R_now, u), R_now) < 1e-6
        assert pose_ori_miss_deg(R_saved, R_now, u) < 1e-6
        assert pose_ori_miss_deg(R_saved, R_now) == pytest.approx(abs(deg), abs=1e-6)
    # 倾斜不同的姿态：绕 u 怎么转都去不掉，误差就是倾斜差
    tilt = axis_angle_R([1.0, 0.0, 0.0], 20.0) @ R_now
    assert pose_ori_miss_deg(tilt, R_now, UP) == pytest.approx(20.0, abs=0.5)


@pytest.mark.parametrize("ik", ["single", "robust"])
def test_free_axis_ignores_yaw_but_keeps_tilt(arm, ik):
    model, q0, tcp, c, R = arm
    R_lock = axis_angle_R(UP, 40.0) @ R      # 记下的姿态和当前只差绕竖直 40°
    target = c + [0.03, -0.02, 0.02]
    solve = solve_tcp_pose_ik if ik == "single" else solve_tcp_pose_ik_robust_local
    q, miss, ori = solve(model, q0, tcp, target, R_lock, free_axis=UP)
    assert miss < 2.0 and ori < 1.0
    t1, R1 = model.fk(q)
    # 「上」在手腕系里的方向保持不变 = 倾斜锁住
    assert np.degrees(np.arccos(np.clip((R1.T @ UP) @ (R_lock.T @ UP), -1, 1))) < 1.0
    # 不用转那 40°：解的朝向离当前近，离 R_lock 远
    assert _ang(R1, R) < 15.0 and _ang(R1, R_lock) > 25.0


def test_full_lock_still_counts_yaw(arm):
    model, q0, tcp, c, R = arm
    R_lock = axis_angle_R(UP, 40.0) @ R
    q, _, ori = solve_tcp_pose_ik(model, q0, tcp, c, R_lock)
    # 全锁：要把那 40° 真的转过去
    assert ori < 3.0 and _ang(model.fk(q)[1], R_lock) < 3.0


def test_session_saves_and_locks_free_mode():
    wp = WorkPoseSession()
    st = wp.save("可平转测试", "测试TCP", "right", np.eye(3), q=np.zeros(7), free="gravity_z")
    row = next(s for s in st["saved"] if s["name"] == "可平转测试")
    assert row["free"] == "gravity_z"
    wp.save("全锁测试", "测试TCP", "right", np.eye(3))
    assert next(s for s in wp.status("测试TCP")["saved"] if s["name"] == "全锁测试")["free"] == ""
    wp.apply_lock(wp.load_doc("测试TCP", "可平转测试"))
    assert wp.status()["free"] == "gravity_z"
    wp.unlock()
    assert wp.free == "" and wp.status()["free"] == ""
    with pytest.raises(ValueError):
        wp.save("坏", "测试TCP", "right", np.eye(3), free="xyz")


def test_gravity_up_dir_falls_back_to_torso_z():
    assert np.allclose(GravityDirection(None).up_dir(), UP)


def test_jog_and_goto_with_free_work_pose(mock_server):
    """整条链路：记可平转姿态 → 调用锁定 → 绕 z 点动不解锁 → 绕 x 点动解锁。"""
    c = mock_server
    assert c.post("/api/engage")[0] == 200
    c.post("/api/speed", {"max_speed_rad_s": 1.0})
    for path, body in [("/api/tcp/new-flange", {"name": "平转法兰", "arm": "right", "force": True}),
                       ("/api/tcp/load", {"name": "平转法兰"}),
                       ("/api/work-pose/save", {"name": "平转姿态", "arm": "right", "free": "gravity_z"}),
                       ("/api/work-pose/load", {"name": "平转姿态", "tcp_name": "平转法兰", "arm": "right"})]:
        code, d = c.post(path, body)
        assert code == 200 and d.get("ok", True), (path, d)
    assert d["locked"] and d["free"] == "gravity_z"
    assert any(s["name"] == "平转姿态" and s["free"] == "gravity_z" for s in d["saved"])
    code, d = c.post("/api/tcp/jog", {"arm": "right", "mode": "rotate", "axis": "z", "sign": 1, "step_deg": 10})
    assert code == 200 and d["ok"] and d["work_pose_locked"] is True, d
    assert c.post("/api/work-pose/list", {"tcp_name": "平转法兰"})[1]["locked"] is True
    code, d = c.post("/api/tcp/jog", {"arm": "right", "mode": "translate", "axis": "x", "sign": 1, "step_mm": 10})
    assert code == 200 and d["ok"] and d["work_pose_locked"] is True, d
    code, d = c.post("/api/tcp/jog", {"arm": "right", "mode": "rotate", "axis": "x", "sign": 1, "step_deg": 5})
    assert code == 200 and d["ok"] and d["work_pose_locked"] is False, d
    assert c.post("/api/work-pose/list", {"tcp_name": "平转法兰"})[1]["locked"] is False
    c.post("/api/release")
