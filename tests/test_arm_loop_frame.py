"""ArmLoop 发出的指令帧：逐电机核对 q / kp / kd / τ；两只手各自独立受控。"""

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
    loop = ArmLoop(reader=backend.state(), commander=rec, kin=kin,
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


def _cmd(loop, arm):
    return np.asarray(loop.status()["arms"][arm]["cmd_rad"])


def _wait_reached(loop, arm, tgt, timeout_s=4.0):
    deadline = time.monotonic() + timeout_s
    while float(np.max(np.abs(_cmd(loop, arm) - tgt))) > 1e-9:
        assert time.monotonic() < deadline, f"{arm} 没走到目标"
        time.sleep(0.02)


def test_no_frames_before_engage(rig):
    _, _, rec, _ = rig
    time.sleep(0.1)
    assert rec.count() == 0


def test_engaged_frame_layout_both_arms(rig):
    """接管后两只手都按各自指令角发、都有重力前馈；腰锁住；其它电机保持实测。"""
    profile, backend, rec, loop = rig
    loop.engage()
    assert rec.prepared == 1
    q, kp, kd, tau, mode = _wait_frames(rec)
    assert len(q) == profile.num_motors
    assert mode == backend.state().snapshot().mode_machine

    waist = profile.waist
    status = loop.status()
    meas = backend.state().snapshot().q
    arm_motors = {}
    for name in profile.arms:
        for j, i in enumerate(profile.arm(name).motor_indices):
            arm_motors[i] = (name, j)
    for i in range(profile.num_motors):
        if i in arm_motors:
            name, j = arm_motors[i]
            assert q[i] == pytest.approx(status["arms"][name]["cmd_rad"][j])
            assert (kp[i], kd[i]) == (profile.kp[i], profile.kd[i])
        elif i in waist.hold_motors:
            assert q[i] == pytest.approx(status["waist_hold_rad"][i])
            assert (kp[i], kd[i]) == (waist.lock_kp, waist.lock_kd)
            assert tau[i] == 0.0
        else:
            assert q[i] == pytest.approx(meas[i], abs=1e-6)
            assert (kp[i], kd[i]) == (profile.kp[i], profile.kd[i])
            assert tau[i] == 0.0

    # 两只手都有重力前馈（以前只有控制臂有，另一只会慢慢往下掉）
    for name in profile.arms:
        tau_arm = [tau[i] for i in profile.arm(name).motor_indices]
        assert any(abs(t) > 1e-3 for t in tau_arm), name
        assert tau_arm == pytest.approx(loop._arms[name].last_tau_ff.tolist())


def test_both_arms_locked_at_engage_angles(rig):
    """接管时两只手都锁在当时的实测角：指令角不再跟着实测走。"""
    profile, backend, rec, loop = rig
    loop.engage()
    locked = {name: _cmd(loop, name).copy() for name in profile.arms}
    with backend.robot._lock:  # 模拟手臂被重力拉低了一点
        for name in profile.arms:
            for i in profile.arm(name).motor_indices:
                backend.robot.q[i] += 0.05
    q, _, _, _, _ = _wait_frames(rec)
    for name in profile.arms:
        sent = [q[i] for i in profile.arm(name).motor_indices]
        assert sent == pytest.approx(locked[name].tolist())


def test_teach_one_arm_other_still_controlled(rig):
    """左手拖拽（刚度 0）时右手照常受控：能设目标、照样按限速走。"""
    profile, _, rec, loop = rig
    loop.engage()
    loop.enable_teach("left")
    q, kp, kd, tau, _ = _wait_frames(rec)
    for i in profile.arm("left").motor_indices:
        assert (kp[i], kd[i]) == (0.0, TEACH_KD)
    for i in profile.arm("right").motor_indices:
        assert (kp[i], kd[i]) == (profile.kp[i], profile.kd[i])
    with pytest.raises(PermissionErrorMotion):
        loop.set_target(_cmd(loop, "left"), arm="left")
    tgt = _cmd(loop, "right").copy()
    tgt[3] -= 0.05
    loop.set_target(tgt, arm="right")
    _wait_reached(loop, "right", tgt)
    assert loop.status()["arms"]["left"]["float"] is True
    loop.disable_teach("left")
    loop.set_gravity_ff(False)
    q, kp, kd, tau, _ = _wait_frames(rec)
    for name in profile.arms:
        assert all(tau[i] == 0.0 for i in profile.arm(name).motor_indices)


def test_targets_are_independent(rig):
    """右手在走的时候给左手发目标：两只手各走各的，互不打断、互不等待。"""
    _, _, _, loop = rig
    loop.engage()
    loop.set_max_speed(0.4)
    r0, l0 = _cmd(loop, "right").copy(), _cmd(loop, "left").copy()
    r_tgt = r0.copy()
    r_tgt[3] -= 0.3      # 右手走得远
    loop.set_target(r_tgt, arm="right")
    time.sleep(0.2)
    l_tgt = l0.copy()
    l_tgt[3] -= 0.04     # 中途给左手一个近的目标
    loop.set_target(l_tgt, arm="left")
    _wait_reached(loop, "left", l_tgt, timeout_s=1.0)
    # 左手先到，右手还在路上、目标没被改
    assert float(np.max(np.abs(_cmd(loop, "right") - r_tgt))) > 0.02
    assert loop.status()["arms"]["right"]["desired_rad"] == pytest.approx(r_tgt.tolist())
    _wait_reached(loop, "right", r_tgt)


def test_target_is_rate_limited_and_clamped(rig):
    _, _, _, loop = rig
    loop.engage()
    loop.set_max_speed(0.2)
    q0 = _cmd(loop, "right").copy()
    tgt = q0.copy()
    tgt[3] -= 0.3
    loop.set_target(tgt, arm="right")
    time.sleep(0.5)
    mid = _cmd(loop, "right")
    # 0.5 s × 0.2 rad/s ≈ 0.1 rad，留出线程调度余量
    assert 0.05 < q0[3] - mid[3] < 0.15
    _wait_reached(loop, "right", tgt)
    # 超限目标被夹到 URDF 限位
    far = q0.copy()
    far[0] = 100.0
    loop.set_target(far, arm="right")
    assert loop.status()["arms"]["right"]["desired_rad"][0] == pytest.approx(loop.limits("right")[0, 1])


def test_claim_blocks_other_sources_on_same_arm_only(rig):
    """任务占用一只手时，别的来源给这只手发目标被拒；另一只手不受影响；任务结束自动归还。"""
    _, _, _, loop = rig
    loop.engage()
    r = _cmd(loop, "right")
    with loop.claim("right", "棋盘触摸"):
        loop.set_target(r, arm="right", owner="棋盘触摸")
        with pytest.raises(PermissionErrorMotion, match="棋盘触摸"):
            loop.set_target(r, arm="right")
        with pytest.raises(PermissionErrorMotion, match="棋盘触摸"):
            with loop.claim("right", "空间点碰点"):
                pass
        with pytest.raises(PermissionErrorMotion):
            loop.enable_teach("right")
        loop.set_target(_cmd(loop, "left"), arm="left")
        assert loop.status()["arms"]["right"]["owner"] == "棋盘触摸"
    assert loop.status()["arms"]["right"]["owner"] is None
    loop.set_target(r, arm="right")


def test_hold_arm_stops_only_that_arm(rig):
    _, _, _, loop = rig
    loop.engage()
    loop.set_max_speed(0.2)
    r_tgt, l_tgt = _cmd(loop, "right").copy(), _cmd(loop, "left").copy()
    r_tgt[3] -= 0.3
    l_tgt[3] -= 0.3
    loop.set_target(r_tgt, arm="right")
    loop.set_target(l_tgt, arm="left")
    time.sleep(0.2)
    loop.hold_arm("left")
    held = loop.status()["arms"]["left"]["desired_rad"]
    assert held != pytest.approx(l_tgt.tolist())
    assert loop.status()["arms"]["right"]["desired_rad"] == pytest.approx(r_tgt.tolist())


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
        loop.set_target(_cmd(loop, "right"), arm="right")


def test_estop_hold_stops_both_arms_and_teach(rig):
    _, _, _, loop = rig
    loop.engage()
    loop.enable_teach("left")
    tgt = _cmd(loop, "right").copy()
    tgt[3] -= 0.3
    loop.set_target(tgt, arm="right")
    time.sleep(0.1)
    loop.estop_hold()
    st = loop.status()["arms"]
    assert st["left"]["float"] is False
    assert st["right"]["desired_rad"] != pytest.approx(tgt.tolist())


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


def test_slew_step_follows_elapsed_time(rig):
    """限速按实际经过的时间算：同一个 dt，步长 = 限速 × dt；循环慢了每拍就走得多些。"""
    _, _, _, loop = rig
    loop.set_max_speed(0.5)
    a = loop._arms["right"]
    with loop._lock:
        a.cmd_q = np.zeros(7)
        a.desired_q = np.full(7, 1.0)  # 远处目标；按最大分量限幅
        a.clamp = lambda q: q          # 不让关节限位干扰这个测试
        loop._slew(a, 0.01)
        assert np.allclose(a.cmd_q, 0.005)
        loop._slew(a, 0.03)            # 循环慢了一倍多：这一拍补上
        assert np.allclose(a.cmd_q, 0.005 + 0.015)


def test_status_reports_loop_rate(rig):
    _, _, _, loop = rig
    time.sleep(0.5)
    hz = loop.status()["loop_hz"]
    assert hz is not None and 50 < hz < 130
