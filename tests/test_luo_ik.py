"""罗-IK（SLSQP 紧约束 + 偏好挑解）：到位精度、少动、free_axis、策略切换。需要 scipy（本机没有就跳过）。"""

import numpy as np
import pytest

pytest.importorskip("scipy")

from robot_platform.control.fk_scene import RobotKinematics  # noqa: E402
from robot_platform.core.robot_profile import active  # noqa: E402
from robot_platform.tasks.luo_ik import (  # noqa: E402
    ORI_TOL_DEG, POS_TOL_MM, solve_tcp_pose_ik_luo, solve_tcp_pose_ik_luo_local,
)
from robot_platform.tasks.spatial_points import pose_ori_miss_deg, solve_tcp_pose_ik_robust_local  # noqa: E402


@pytest.fixture(scope="module")
def arm():
    p = active()
    model = RobotKinematics(p.urdf, p.base_link, list(p.arms)).arms["right"].model
    q0 = np.asarray(p.arm("right").home_q, float)
    tcp = np.array([0.0, 0.0, 0.1])
    t, R = model.fk(q0)
    return model, q0, tcp, t + R @ tcp, R


def test_tight_constraints_met(arm):
    model, q0, tcp, c, R = arm
    target = c + [0.06, 0.03, 0.04]
    q, m, o = solve_tcp_pose_ik_luo_local(model, q0, tcp, target, R)
    assert m <= POS_TOL_MM * 1.05 and o <= ORI_TOL_DEG * 1.05
    t, R2 = model.fk(q)
    assert np.linalg.norm(t + R2 @ tcp - target) * 1000 == pytest.approx(m, abs=1e-6)
    lim = model.joint_limits()
    assert np.all(q >= lim[:, 0] - 1e-9) and np.all(q <= lim[:, 1] + 1e-9)


def test_moves_little_like_robust(arm):
    """偏好「少动」：离 q0 不比多初值的解远多少（同一支解）。"""
    model, q0, tcp, c, R = arm
    target = c + [0.03, -0.02, 0.02]
    q, m, o = solve_tcp_pose_ik_luo_local(model, q0, tcp, target, R)
    qr, mr, orr = solve_tcp_pose_ik_robust_local(model, q0, tcp, target, R)
    assert m <= POS_TOL_MM * 1.05 and o <= ORI_TOL_DEG * 1.05
    assert np.linalg.norm(q - q0) < np.linalg.norm(qr - q0) + 0.2


def test_free_axis_ignores_spin(arm):
    model, q0, tcp, c, R = arm
    up = np.array([0.0, 0.0, 1.0])
    Rz = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)  # 绕 z 转 90°
    q, m, o = solve_tcp_pose_ik_luo_local(model, q0, tcp, c, Rz @ R, free_axis=up)
    assert m <= POS_TOL_MM * 1.05 and o <= ORI_TOL_DEG * 1.05
    assert np.linalg.norm(q - q0) < 0.05  # 原地就满足：不必转
    assert pose_ori_miss_deg(Rz @ R, model.fk(q)[1]) > 45  # 全姿态看确实差了 90° 左右


def test_runs_in_worker(arm):
    model, q0, tcp, c, R = arm
    target = c + [0.02, 0.0, 0.03]
    got = solve_tcp_pose_ik_luo(model, q0, tcp, target, R)
    ref = solve_tcp_pose_ik_luo_local(model, q0, tcp, target, R)
    assert np.allclose(got[0], ref[0], atol=1e-6)
