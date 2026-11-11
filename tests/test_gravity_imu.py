"""重力前馈按躯干 IMU 的重力方向算：方向换算、平滑、读数不可信时保持、前馈帧、负载辨识。"""

import math
import time

import numpy as np
import pytest

from robot_platform.control.arm_gravity import G_ACCEL, ArmGravity
from robot_platform.control.arm_loop import ArmLoop
from robot_platform.control.fk_scene import RobotKinematics
from robot_platform.control.gravity_dir import TAU_S, GravityDirection, down_in_base, tilt_angles
from robot_platform.calib.payload.identify import solve_payload
from robot_platform.core.robot_profile import active, load_profile
from robot_platform.hw.backends.mock import MockBackend, MockTorsoImu

from test_arm_loop_frame import Recorder, _wait_frames


def _settle(gd, seconds=5.0, dt=0.01):
    for _ in range(int(seconds / dt)):
        gd.update(dt)


def test_g1d_declares_torso_imu():
    spec = load_profile("g1d").torso_imu
    assert spec is not None and spec.topic == "rt/secondary_imu"
    assert spec.gravity_default is False  # 真机验证前默认不开
    assert load_profile("g1").torso_imu is None


@pytest.mark.parametrize("pitch,roll", [(3.4, -0.7), (-5.0, 2.0), (0.0, 0.0)])
def test_mock_quaternion_gives_back_tilt(pitch, roll):
    imu = MockTorsoImu()
    imu.set_tilt(pitch, roll)
    d = down_in_base(imu.latest()[0], np.eye(3))
    t = tilt_angles(d)
    assert t["pitch_deg"] == pytest.approx(pitch, abs=0.05)
    assert t["roll_deg"] == pytest.approx(roll, abs=0.05)
    assert np.linalg.norm(d) == pytest.approx(1.0)


def test_imu_quaternion_like_g1d():
    """G1D 2026-11-11 实测的四元数（宇树 rpy 俯仰 -3.42°、偏航 17°）：躯干后仰约 3.4°，偏航不影响重力方向。
    同时雷达拟合的地面在躯干系里法线朝 +x 倾 3.3°，与此一致。"""
    q = (0.9887269139289856, -0.0021397750824689865, -0.030490079894661903, 0.14657744765281677)
    t = tilt_angles(down_in_base(q, np.eye(3)))
    assert t["pitch_deg"] == pytest.approx(-3.4, abs=0.3)
    assert abs(t["roll_deg"]) < 1.0


def test_direction_off_on_smooth_and_hold():
    spec = load_profile("g1d").torso_imu
    imu = MockTorsoImu()
    imu.set_tilt(4.0, 0.0)
    gd = GravityDirection(imu, spec)
    _settle(gd)
    assert gd.status()["used"]["tilt_deg"] == pytest.approx(0.0, abs=1e-6)  # 默认不开：竖直
    assert gd.status()["imu"]["pitch_deg"] == pytest.approx(4.0, abs=0.05)  # 但一直读着给页面看
    gd.set_enabled(True)
    gd.update(0.01)
    # 平滑：一拍只走 dt/TAU 那么多
    assert gd.status()["used"]["pitch_deg"] == pytest.approx(4.0 * 0.01 / TAU_S, abs=0.02)
    _settle(gd)
    assert gd.status()["used"]["pitch_deg"] == pytest.approx(4.0, abs=0.05)
    assert np.linalg.norm(gd.g_vec()) == pytest.approx(G_ACCEL)
    # 断流：保持
    imu.stale = True
    imu.set_tilt(-8.0, 0.0)
    _settle(gd, 1.0)
    st = gd.status()
    assert st["used"]["pitch_deg"] == pytest.approx(4.0, abs=0.05) and "没更新" in st["error"]
    # 倾角离谱：保持
    imu.stale = False
    imu.set_tilt(30.0, 0.0)
    _settle(gd, 1.0)
    st = gd.status()
    assert st["used"]["pitch_deg"] == pytest.approx(4.0, abs=0.05) and "超过" in st["error"]
    # 恢复正常后跟上；关掉回竖直
    imu.set_tilt(2.0, 0.0)
    _settle(gd)
    assert gd.status()["used"]["pitch_deg"] == pytest.approx(2.0, abs=0.05) and gd.status()["error"] == ""
    gd.set_enabled(False)
    _settle(gd)
    assert gd.status()["used"]["tilt_deg"] == pytest.approx(0.0, abs=1e-3)


def test_no_imu_cannot_enable():
    gd = GravityDirection(None, None)
    assert not gd.available
    with pytest.raises(ValueError):
        gd.set_enabled(True)
    assert np.allclose(gd.g_vec(), [0, 0, -G_ACCEL])


def test_tilted_gravity_sign_and_linearity():
    """默认 g 与显式竖直 g 一致；力矩对 g 线性；仿真 IMU 前倾 θ 给出的重力向量与手算一致（核对符号）。"""
    p = active()
    grav = ArmGravity("right", p.urdf, p.base_link)
    q = np.asarray(p.arm("right").home_q, float)
    assert np.allclose(grav.tau_arm(q), grav.tau_arm(q, [0, 0, -G_ACCEL]))
    th = math.radians(10)
    g_tilt = G_ACCEL * np.array([math.sin(th), 0.0, -math.cos(th)])  # 躯干前倾：重力在躯干系里偏向前（+x）
    t0, t1 = grav.tau_arm(q), grav.tau_arm(q, g_tilt)
    assert not np.allclose(t0, t1)
    # 线性：力矩对 g 线性
    assert np.allclose(grav.tau_arm(q, 2 * g_tilt), 2 * t1)
    imu = MockTorsoImu()
    imu.set_tilt(10.0, 0.0)
    assert np.allclose(G_ACCEL * down_in_base(imu.latest()[0], np.eye(3)), g_tilt, atol=1e-9)


@pytest.fixture
def imu_rig():
    profile = active()
    assert profile.torso_imu is not None
    backend = MockBackend(profile)
    rec = Recorder()
    kin = RobotKinematics(profile.urdf, profile.base_link, list(profile.arms))
    loop = ArmLoop(reader=backend.state(), commander=rec, kin=kin, urdf_path=profile.urdf,
                   base_link=profile.base_link, torso_imu=backend.torso_imu())
    yield profile, backend, rec, loop
    loop.shutdown()
    backend.close()


def test_loop_feedforward_follows_imu(imu_rig):
    profile, backend, rec, loop = imu_rig
    backend.torso_imu().set_tilt(5.0, -1.0)
    loop.engage()
    q0, *_ = _wait_frames(rec)
    arm = profile.arm("right")
    grav = ArmGravity("right", profile.urdf, profile.base_link)

    def tau_of(frame):
        return np.asarray([frame[3][i] for i in arm.motor_indices])

    q_meas = np.asarray([q0[i] for i in arm.motor_indices])
    assert np.allclose(tau_of(_wait_frames(rec)), grav.tau_arm(q_meas), atol=1e-6)  # 没开：竖直
    loop.set_gravity_imu(True)
    deadline = time.monotonic() + 5
    while loop.status()["gravity_dir"]["used"]["pitch_deg"] < 4.99:
        assert time.monotonic() < deadline
        time.sleep(0.05)
    g = loop.gravity.g_vec()
    fr = _wait_frames(rec)
    q_meas = np.asarray([fr[0][i] for i in arm.motor_indices])
    assert np.allclose(tau_of(fr), grav.tau_arm(q_meas, g), atol=1e-3)
    assert not np.allclose(tau_of(fr), grav.tau_arm(q_meas), atol=1e-3)
    s = loop.arm_sample("right")
    assert np.allclose(s["g_torso"], g, atol=1e-3)
    st = loop.status()["gravity_dir"]
    assert st["enabled"] and st["available"] and st["error"] == ""


def test_solve_payload_uses_sample_gravity():
    """样本带着各自的重力向量：躯干倾着测的负载，解出来和真值一致；按竖直硬解会偏。"""
    p = active()
    grav = ArmGravity("right", p.urdf, p.base_link)
    theta = np.array([0.6, 0.6 * 0.01, 0.6 * -0.02, 0.6 * 0.08])  # m, m·com
    rng = np.random.default_rng(1)
    lo, hi = grav.limits[:, 0], grav.limits[:, 1]
    th = math.radians(6)
    g = G_ACCEL * np.array([math.sin(th), 0.0, -math.cos(th)])
    samples = []
    for k in range(10):
        q = lo + (hi - lo) * (0.3 + 0.4 * rng.random(7))
        tau = grav.tau_arm(q, g) + grav.tau_payload(q, theta, g)
        samples.append({"name": f"p{k}", "q_meas": q.tolist(), "tau_pd": tau.tolist(), "g_torso": g.tolist(), "kp": [40.0] * 7})
    got = solve_payload(samples, "right", p.urdf, p.base_link)
    m = got["mass_kg"]
    assert m == pytest.approx(0.6, abs=1e-3)
    for s in samples:
        s.pop("g_torso")
    bad = solve_payload(samples, "right", p.urdf, p.base_link)
    m_bad = bad["mass_kg"]
    assert abs(m_bad - 0.6) > 1e-3


def test_api_gravity_imu(mock_server):
    st = mock_server.get("/api/status")
    gd = st["ctrl"]["gravity_dir"]
    assert gd["available"] is True and gd["enabled"] is False
    code, d = mock_server.post("/api/gravity-imu", {"enabled": True})
    assert code == 200, d
    deadline = time.monotonic() + 3
    while not mock_server.get("/api/status")["ctrl"]["gravity_dir"]["enabled"]:
        assert time.monotonic() < deadline
        time.sleep(0.05)
    code, d = mock_server.post("/api/gravity-imu", {"enabled": False})
    assert code == 200, d
