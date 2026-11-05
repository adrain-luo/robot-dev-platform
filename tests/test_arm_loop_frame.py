"""ArmLoop 发出的指令帧：逐电机核对 q / kp / kd / τ，保证换成后端接口后发令内容不变。"""

import threading
import time

import numpy as np
import pytest

from robot_platform.hw.backends.base import Commander
from robot_platform.hw.backends.mock import MockBackend
from robot_platform.control.arm_loop import TEACH_KD, ArmLoop, PermissionErrorMotion
from robot_platform.control.fk_scene import RobotKinematics
from robot_platform.core.robot_profile import active


class Recorder(Commander):
    def __init__(self):
        self.prepared = 0
        self.frames = []
        self._lock = threading.Lock()

    def prepare(self):
        self.prepared += 1

    def write(self, q, kp, kd, tau, *, mode_machine=None):
        with self._lock:
            self.frames.append((list(q), list(kp), list(kd), list(tau), mode_machine))

    def last(self):
        with self._lock:
            return self.frames[-1]

    def count(self):
        with self._lock:
            return len(self.frames)


@pytest.fixture
def rig():
    profile = active()
    backend = MockBackend(profile)
    rec = Recorder()
    kin = RobotKinematics(profile.urdf, profile.base_link, list(profile.arms))
    loop = ArmLoop(reader=backend.state(), commander=rec, kin=kin, arm="right",
                   urdf_path=profile.urdf, base_link=profile.base_link)
    yield profile, backend, rec, loop
    loop.shutdown()
    backend.close()


def _wait_frames(rec, n=3):
    start = rec.count()
    deadline = time.monotonic() + 2
    while rec.count() < start + n:
        assert time.monotonic() < deadline, "没有新的指令帧"
        time.sleep(0.01)
    return rec.last()


def test_no_frames_before_engage(rig):
    _, _, rec, _ = rig
    time.sleep(0.1)
    assert rec.count() == 0


def test_engaged_frame_layout(rig):
    profile, backend, rec, loop = rig
    loop.engage()
    assert rec.prepared == 1
    q, kp, kd, tau, mode = _wait_frames(rec)
    assert len(q) == profile.num_motors
    assert mode == backend.state().snapshot().mode_machine

    arm = profile.arm("right")
    waist = profile.waist
    status = loop.status()
    meas = backend.state().snapshot().q
    for i in range(profile.num_motors):
        if i in arm.motor_indices:
            j = arm.motor_indices.index(i)
            assert q[i] == pytest.approx(status["cmd_rad"][j])
            assert (kp[i], kd[i]) == (profile.kp[i], profile.kd[i])
        elif i in waist.hold_motors:
            assert q[i] == pytest.approx(status["waist_hold_rad"][i])
            assert (kp[i], kd[i]) == (waist.lock_kp, waist.lock_kd)
            assert tau[i] == 0.0
        else:
            assert q[i] == pytest.approx(meas[i], abs=1e-6)
            assert (kp[i], kd[i]) == (profile.kp[i], profile.kd[i])
            assert tau[i] == 0.0

    # 重力前馈只加在当前臂，且与 ArmGravity 一致
    tau_arm = [tau[i] for i in arm.motor_indices]
    assert any(abs(t) > 1e-3 for t in tau_arm)
    assert tau_arm == pytest.approx(loop._last_tau_ff.tolist())


def test_teach_and_gravity_toggle(rig):
    profile, _, rec, loop = rig
    loop.engage()
    loop.enable_teach()
    q, kp, kd, tau, _ = _wait_frames(rec)
    for i in profile.arm("right").motor_indices:
        assert (kp[i], kd[i]) == (0.0, TEACH_KD)
    loop.disable_teach()
    loop.set_gravity_ff(False)
    q, kp, kd, tau, _ = _wait_frames(rec)
    assert all(tau[i] == 0.0 for i in profile.arm("right").motor_indices)


def test_target_is_rate_limited_and_clamped(rig):
    profile, backend, rec, loop = rig
    loop.engage()
    loop.set_max_speed(0.2)
    q0 = np.asarray(loop.status()["cmd_rad"])
    tgt = q0.copy()
    tgt[3] -= 0.3
    loop.set_target(tgt)
    time.sleep(0.5)
    mid = np.asarray(loop.status()["cmd_rad"])
    # 0.5 s × 0.2 rad/s ≈ 0.1 rad，留出线程调度余量
    assert 0.05 < q0[3] - mid[3] < 0.15
    deadline = time.monotonic() + 4
    while abs(loop.status()["cmd_rad"][3] - tgt[3]) > 1e-9:
        assert time.monotonic() < deadline
        time.sleep(0.05)
    # 超限目标被夹到 URDF 限位
    far = q0.copy()
    far[0] = 100.0
    loop.set_target(far)
    assert loop.status()["desired_rad"][0] == pytest.approx(loop.limits[0, 1])


def test_release_stops_frames_and_blocks_targets(rig):
    _, _, rec, loop = rig
    loop.engage()
    _wait_frames(rec)
    loop.release()
    time.sleep(0.05)
    n = rec.count()
    time.sleep(0.1)
    assert rec.count() == n
    with pytest.raises(PermissionErrorMotion):
        loop.set_target(loop.status()["cmd_rad"])


def test_waist_target_clamped_to_profile_limits(rig):
    profile, _, rec, loop = rig
    loop.engage()
    loop.set_waist(yaw_rad=10.0, pitch_rad=-1.0)
    tgt = loop.status()["waist_target_rad"]
    assert float(tgt[str(profile.waist.joint("yaw").motor)]) == profile.waist.joint("yaw").limit_rad[1]
    assert float(tgt[str(profile.waist.joint("pitch").motor)]) == profile.waist.joint("pitch").limit_rad[0]


def test_idle_motor_guard_blocks_legged_robot():
    """G1D 声明腿部电机空闲；若读到它们在工作（例如站立的 G1），退出运控前必须拒绝。"""
    from robot_platform.hw.backends.base import check_idle_motors

    profile = active()
    idle = profile.control.idle_motors
    assert idle == tuple(range(12))
    backend = MockBackend(profile)
    try:
        check_idle_motors(backend.state(), idle)  # mock 的腿部电机全 0：通过
        with backend.robot._lock:
            backend.robot.q[3] = 0.62  # 模拟站立时的膝关节角
        with pytest.raises(PermissionErrorMotion, match="拒绝接管"):
            check_idle_motors(backend.state(), idle)
    finally:
        backend.close()
