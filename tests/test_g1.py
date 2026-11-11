"""G1：arm_sdk 接管 / 释放过渡、增益混合、机型配置，以及 ArmLoop 在过渡期间的行为。"""

import threading
import time

import numpy as np
import pytest

from robot_platform.hw.backends.base import Commander, PermissionErrorMotion
from robot_platform.hw.backends.mock import MockBackend
from robot_platform.hw.backends.unitree_arm_sdk import ArmSdkSchedule, blend_frame
from robot_platform.control.arm_loop import ArmLoop
from robot_platform.control.fk_scene import RobotKinematics
from robot_platform.core.robot_profile import activate, active, load_profile


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def _run(sched, clock, until, dt=0.01):
    trace = []
    while clock.t < until:
        clock.t += dt
        trace.append((clock.t, sched.state, *sched.step()))
    return trace


def test_schedule_acquire_then_release_never_jumps():
    clock = Clock()
    s = ArmSdkSchedule(acquire_s=8, gain_ramp_s=4, release_gain_s=2, release_s=3, clock=clock)
    assert not s.busy() and not s.ready()
    s.start()
    tr = _run(s, clock, 12.5)
    w = [x[2] for x in tr]
    g = [x[3] for x in tr]
    assert all(b >= a for a, b in zip(w, w[1:])) and all(b >= a for a, b in zip(g, g[1:]))
    # 增益只在权重到 1 之后才开始变
    assert all(gi == 0.0 for wi, gi in zip(w, g) if wi < 1.0)
    assert s.ready() and w[-1] == 1.0 and g[-1] == 1.0
    # 每步变化都很小（无突变）
    assert max(abs(b - a) for a, b in zip(w, w[1:])) < 0.01
    assert max(abs(b - a) for a, b in zip(g, g[1:])) < 0.01

    s.release()
    tr = _run(s, clock, clock.t + 5.5)
    w = [x[2] for x in tr]
    g = [x[3] for x in tr]
    assert all(b <= a for a, b in zip(w, w[1:])) and all(b <= a for a, b in zip(g, g[1:]))
    # 先把增益降回运控，再降权重
    assert all(wi == 1.0 for wi, gi in zip(w, g) if gi > 0.0)
    assert not s.busy() and w[-1] == 0.0 and g[-1] == 0.0


def test_schedule_release_during_acquire_and_restart_guard():
    clock = Clock()
    s = ArmSdkSchedule(8, 4, 2, 3, clock=clock)
    s.start()
    _run(s, clock, 4.0)
    w_mid = s.weight
    assert 0.0 < w_mid < 1.0
    s.release()
    assert s.state == "releasing"  # 增益还没动过，直接降权重
    with pytest.raises(PermissionErrorMotion):
        s.start()  # 释放过渡中不能再接管
    tr = _run(s, clock, clock.t + 3.5)
    assert tr[0][2] <= w_mid and not s.busy()
    s.start()  # 结束后可以再接管
    assert s.state == "acquiring"


def test_blend_frame_native_to_ours():
    p = load_profile("g1")
    opt = p.control.options
    motors = opt["command_motors"]
    q = np.linspace(-1, 1, 29)
    tau = np.full(29, 2.0)
    at0 = blend_frame(q, p.kp, p.kd, tau, motors, opt["native_kp"], opt["native_kd"], 0.0)
    at1 = blend_frame(q, p.kp, p.kd, tau, motors, opt["native_kp"], opt["native_kd"], 1.0)
    assert [m for m, *_ in at0] == list(range(12, 29))
    assert [r[2] for r in at0] == pytest.approx(opt["native_kp"]) and all(r[4] == 0.0 for r in at0)
    assert [r[2] for r in at1] == pytest.approx([p.kp[i] for i in motors])
    assert all(r[4] == 2.0 for r in at1)
    assert [r[1] for r in at1] == pytest.approx([q[i] for i in motors])


@pytest.mark.parametrize("robot", ["g1d", "g1"])
def test_mirror_sign_matches_urdf(robot):
    """左臂 q 的末端位姿关于躯干 y=0 镜像，应等于右臂 mirror(q) 的位姿。"""
    p = load_profile(robot)
    kin = RobotKinematics(p.urdf, p.base_link, list(p.arms))
    sign = np.asarray(p.mirror_q_sign)
    S = np.diag([1.0, -1.0, 1.0])
    rng = np.random.default_rng(1)
    lim = kin.arms["left"].model.joint_limits()
    # 官方 URDF 左右并非严格对称（原点差约 0.01 mm），容差取 0.5 mm / 0.05°
    for _ in range(10):
        q = lim[:, 0] + (lim[:, 1] - lim[:, 0]) * rng.uniform(0.2, 0.8, len(lim))
        tl, Rl = kin.arms["left"].model.fk(q)
        tr, Rr = kin.arms["right"].model.fk(q * sign)
        assert np.abs(S @ tl - tr).max() < 5e-4
        assert np.abs(S @ Rl @ S - Rr).max() < np.radians(0.05)


def test_g1_profile():
    p = load_profile("g1")
    assert p.control.backend == "unitree_arm_sdk" and not p.control.release_motion_mode
    assert p.lift is None and not p.waist.movable
    assert [j.motor for j in p.waist.joints] == [12, 13, 14]
    assert p.urdf.is_file() and len(list(p.meshes_dir.glob("*.STL"))) == 35


class RampCommander(Commander):
    """模拟 arm_sdk 的过渡：prepare 后 busy、ready 由测试控制；release 后 busy 到测试放行。"""

    def __init__(self):
        self.frames = 0
        self.phase_ = "acquiring"
        self._busy = False
        self._ready = False
        self.last = None
        self.lock = threading.Lock()

    def prepare(self):
        self._busy, self._ready = True, False

    def write(self, q, kp, kd, tau, *, mode_machine=None):
        with self.lock:
            self.frames += 1
            self.last = list(q)

    def release(self):
        self._ready = False

    def busy(self):
        return self._busy

    def ready(self):
        return self._ready

    def phase(self):
        return "" if self._ready or not self._busy else self.phase_


@pytest.fixture
def g1_loop():
    prev = active()
    p = activate(load_profile("g1"))
    backend = MockBackend(p)
    cmd = RampCommander()
    kin = RobotKinematics(p.urdf, p.base_link, list(p.arms))
    loop = ArmLoop(reader=backend.state(), commander=cmd, kin=kin, urdf_path=p.urdf)
    yield p, cmd, loop
    cmd._busy = False
    loop.shutdown(timeout_s=1)
    backend.close()
    activate(prev)


def test_g1_motion_blocked_until_transition_done(g1_loop):
    p, cmd, loop = g1_loop
    loop.engage()
    q = loop.status()["arms"]["right"]["cmd_rad"]
    with pytest.raises(PermissionErrorMotion, match="过渡"):
        loop.set_target(q, arm="right")
    with pytest.raises(PermissionErrorMotion, match="过渡"):
        loop.enable_teach()
    assert loop.status()["ready"] is False and loop.status()["transition"]
    cmd._ready = True
    loop.set_target(q, arm="right")
    assert loop.status()["ready"] is True
    with pytest.raises(PermissionErrorMotion, match="只保持"):
        loop.set_waist(yaw_rad=0.1)


def test_g1_release_keeps_sending_until_ramp_done(g1_loop):
    p, cmd, loop = g1_loop
    loop.engage()
    cmd._ready = True
    time.sleep(0.05)
    held = loop.status()["waist_hold_rad"]
    loop.release()
    n = cmd.frames
    time.sleep(0.1)
    assert cmd.frames > n, "释放过渡中必须继续发帧"
    # 过渡中腰仍按接管时的角度保持
    assert loop.status()["waist_hold_rad"] == held
    cmd._busy = False  # 过渡结束
    time.sleep(0.05)
    n = cmd.frames
    time.sleep(0.1)
    assert cmd.frames == n and loop.status()["waist_hold_rad"] is None
