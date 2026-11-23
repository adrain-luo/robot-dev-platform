"""G1D（unitree_lowcmd）释放过渡：lowcmd 一断流手臂就掉，所以释放要先把刚度降到 0 再停发。"""

import threading
import time
from types import SimpleNamespace

import pytest

from robot_platform.control.arm_loop import ArmLoop, PermissionErrorMotion
from robot_platform.control.fk_scene import RobotKinematics
from robot_platform.core.robot_profile import active, activate, load_profile
from robot_platform.hw.backends.base import Commander
from robot_platform.hw.backends.mock import MockBackend
from robot_platform.hw.backends.unitree_lowcmd import (
    DEFAULT_RELEASE_RAMP_S,
    LowCmdCommander,
    LowCmdReleaseRamp,
    scale_frame,
)


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def test_ramp_goes_from_full_to_zero_without_jumps():
    clock = FakeClock()
    r = LowCmdReleaseRamp(release_s=3.0, clock=clock)
    assert r.state == "idle" and not r.busy()
    r.start()
    assert r.step() == 1.0 and r.ready() and not r.busy()
    r.release()
    assert r.busy() and not r.ready()
    seen = []
    for k in range(31):
        clock.t = 100.0 + 0.1 * k
        seen.append(r.step())
    assert seen[0] == pytest.approx(1.0)
    assert seen[15] == pytest.approx(0.5)
    assert all(b <= a + 1e-12 for a, b in zip(seen, seen[1:])), "刚度只能单调下降"
    assert max(a - b for a, b in zip(seen, seen[1:])) < 0.06, "每 0.1 s 降幅要小，不能突变"
    assert seen[-1] == 0.0 and r.state == "idle" and not r.busy()


def test_release_is_noop_unless_active():
    clock = FakeClock()
    r = LowCmdReleaseRamp(release_s=1.0, clock=clock)
    r.release()
    assert r.state == "idle" and not r.busy()
    r.start()
    r.release()
    clock.t += 0.5
    s = r.step()
    r.release()  # 过渡中再按一次释放不能重新计时
    clock.t += 0.6
    assert r.step() == 0.0 < s


def test_scale_frame_scales_stiffness_and_feedforward_only():
    kp, tau = scale_frame([100.0, 40.0], [2.0, -1.0], 0.25)
    assert kp == [25.0, 10.0] and tau == [0.5, -0.25]
    assert scale_frame([10.0], [1.0], 1.7) == ([10.0], [1.0])
    assert scale_frame([10.0], [1.0], -0.3) == ([0.0], [0.0])


def _bare_commander(ramp):
    """不连 DDS 的 LowCmdCommander：只换掉发布器，write() 走真实代码。"""
    c = object.__new__(LowCmdCommander)
    c._cmd = SimpleNamespace(mode_pr=None, mode_machine=None, crc=None,
                             motor_cmd=[SimpleNamespace() for _ in range(4)])
    c._crc = SimpleNamespace(Crc=lambda cmd: 0)
    c._monitor = SimpleNamespace(note_own=lambda crc: None)
    c.sent = []
    c._pub = SimpleNamespace(Write=lambda cmd: c.sent.append([(s.q, s.kp, s.kd, s.tau) for s in cmd.motor_cmd]))
    c._ramp = ramp
    return c


def test_commander_write_scales_only_while_releasing():
    clock = FakeClock()
    ramp = LowCmdReleaseRamp(release_s=2.0, clock=clock)
    c = _bare_commander(ramp)
    q, kp, kd, tau = [0.1, 0.2, 0.3, 0.4], [60.0, 60.0, 40.0, 40.0], [1.0, 1.0, 2.0, 2.0], [0.0, 0.0, 3.0, -3.0]
    ramp.start()
    c.write(q, kp, kd, tau)
    assert [m[1] for m in c.sent[-1]] == kp and [m[3] for m in c.sent[-1]] == tau
    c.release()
    assert c.busy() and not c.ready() and c.phase().startswith("释放")
    clock.t += 1.0
    c.write(q, kp, kd, tau)
    frame = c.sent[-1]
    assert [m[1] for m in frame] == pytest.approx([30.0, 30.0, 20.0, 20.0])
    assert [m[3] for m in frame] == pytest.approx([0.0, 0.0, 1.5, -1.5])
    assert [m[2] for m in frame] == kd, "阻尼保持不变"
    assert [m[0] for m in frame] == q, "目标角不变"
    clock.t += 1.0
    c.write(q, kp, kd, tau)
    assert all(m[1] == 0.0 and m[3] == 0.0 for m in c.sent[-1])
    assert not c.busy() and c.phase() == ""


def test_g1d_profile_sets_release_ramp():
    p = load_profile("g1d")
    assert p.control.backend == "unitree_lowcmd"
    assert float(p.control.options.get("release_ramp_s", DEFAULT_RELEASE_RAMP_S)) > 0.0


class RampRecorder(Commander):
    """用真实的 LowCmdReleaseRamp 驱动 busy/ready，记录每帧 kp。"""

    def __init__(self, release_s):
        self.ramp = LowCmdReleaseRamp(release_s)
        self.frames = []
        self.lock = threading.Lock()

    def prepare(self):
        self.ramp.start()

    def write(self, q, kp, kd, tau, *, mode_machine=None):
        if self.ramp.busy():
            kp, tau = scale_frame(kp, tau, self.ramp.step())
        with self.lock:
            self.frames.append((list(kp), list(kd)))

    def release(self):
        self.ramp.release()

    def busy(self):
        return self.ramp.busy()

    def ready(self):
        return self.ramp.ready()

    def count(self):
        with self.lock:
            return len(self.frames)


@pytest.fixture
def g1d_loop():
    prev = active()
    p = activate(load_profile("g1d"))
    backend = MockBackend(p)
    rec = RampRecorder(release_s=0.4)
    kin = RobotKinematics(p.urdf, p.base_link, list(p.arms))
    loop = ArmLoop(reader=backend.state(), commander=rec, kin=kin, urdf_path=p.urdf, base_link=p.base_link)
    yield p, rec, loop
    loop.shutdown(timeout_s=2)
    backend.close()
    activate(prev)


def test_g1d_release_keeps_sending_while_stiffness_ramps_down(g1d_loop):
    p, rec, loop = g1d_loop
    arm_motors = [i for a in p.arms.values() for i in a.motor_indices]
    loop.engage()
    time.sleep(0.05)
    full_kp = rec.frames[-1][0]
    loop.release()
    t_release = time.monotonic()
    start = rec.count()
    with pytest.raises(PermissionErrorMotion):
        loop.engage()  # 过渡中不能重新接管
    deadline = time.monotonic() + 2.0
    while rec.ramp.busy():
        assert time.monotonic() < deadline, "释放过渡没有结束"
        time.sleep(0.01)
    elapsed = time.monotonic() - t_release
    assert elapsed >= 0.35, "过渡太短：等于直接停发"
    time.sleep(0.05)
    with rec.lock:
        during = rec.frames[start:]
    assert len(during) >= 40, "过渡期间必须持续发帧（200 Hz × 0.4 s）"
    arm_kp = [[f[0][i] for i in arm_motors] for f in during]
    first_motor = [row[0] for row in arm_kp]
    assert all(b <= a + 1e-9 for a, b in zip(first_motor, first_motor[1:])), "刚度单调下降"
    assert all(k == 0.0 for k in arm_kp[-1]), "最后一帧刚度为 0（只剩阻尼）"
    assert during[-1][1] == rec.frames[start - 1][1], "阻尼不变"
    assert full_kp[arm_motors[0]] > 0.0
    n = rec.count()
    time.sleep(0.1)
    assert rec.count() == n, "过渡结束后停发"
