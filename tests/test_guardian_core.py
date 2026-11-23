"""GuardianCore: limits, holds, latches and ramps, driven by a fake clock."""

import math

import pytest

from g1_guardian.config import builtin_config_path, config_from_dict, load_config
from g1_guardian.core import ESTOP_RESTORE_S, GuardianCore, LowStateView, track_joint

G1D = load_config(builtin_config_path("g1d"))
DT = G1D.period_s
RIGHT = G1D.group("right_arm")


def rest_q():
    q = [0.0] * 29
    for m, v in zip(RIGHT.motors, [0.2, -0.25, 0.0, 0.9, 0.0, -0.1, 0.0]):
        q[m] = v
    q[13] = 0.07
    return q


class Rig:
    """Core + fake clock + a robot that follows the commanded q exactly."""

    def __init__(self, cfg=G1D):
        self.cfg = cfg
        self.core = GuardianCore(cfg)
        self.t = 100.0
        self.q = rest_q()
        self.seq = 0
        self.frames = []
        self.lowstate_on = True

    def msg(self, kind, client="box", **fields):
        self.seq += 1
        m = {"v": 1, "type": kind, "client": client, "seq": self.seq}
        m.update(fields)
        return m

    def send(self, kind, client="box", **fields):
        return self.core.handle(self.msg(kind, client, **fields), self.t)

    def lowstate(self):
        return LowStateView(q=list(self.q), stamp=self.t) if self.lowstate_on else None

    def tick(self, n=1, client_alive="box"):
        out = None
        for _ in range(n):
            self.t += DT
            if client_alive:
                self.core.handle(self.msg("heartbeat", client_alive), self.t)
            out = self.core.step(self.t, self.lowstate())
            self.frames.append(out)
            if out is not None:
                for g in self.cfg.groups:
                    for m in g.motors:
                        self.q[m] = out.q[m]
        return out

    def engage(self, client="box"):
        self.tick(1, client_alive=None)  # deliver a lowstate first
        assert self.send("engage", client) is None
        return self.core.epoch

    def setpoint(self, q7, client="box", valid_ms=100, **extra):
        body = {"q": list(q7)}
        body.update(extra)
        return self.send("setpoint", client, epoch=self.core.epoch, valid_for_ms=valid_ms, groups={"right_arm": body})

    def right(self, frame):
        return [frame.q[m] for m in RIGHT.motors]


def test_limiter_respects_velocity_and_acceleration_and_lands_on_target():
    q, v, qs, vs = 0.0, 0.0, [], []
    for _ in range(400):
        q, v = track_joint(q, v, 0.6, 1.0, 10.0, -3.0, 3.0, DT)
        qs.append(q)
        vs.append(v)
    assert q == pytest.approx(0.6, abs=1e-9) and v == pytest.approx(0.0, abs=1e-9)
    assert max(abs(x) for x in vs) <= 1.0 + 1e-9
    acc = [(b - a) / DT for a, b in zip(vs, vs[1:])]
    assert max(abs(a) for a in acc) <= 10.0 + 1e-6
    assert max(qs) <= 0.6 + 1e-9, "no overshoot"


def test_limiter_stop_mode_decelerates_instead_of_stopping_dead():
    q, v = 0.0, 1.0
    vs = []
    for _ in range(40):
        q, v = track_joint(q, v, None, 1.0, 10.0, -3.0, 3.0, DT)
        vs.append(v)
    assert vs[0] == pytest.approx(1.0 - 10.0 * DT)
    assert vs[-1] == 0.0 and q == pytest.approx(0.05, abs=0.01)


def test_idle_until_engaged_then_holds_measured_pose():
    r = Rig()
    assert r.tick(3, client_alive=None) is None, "no frames before engage"
    epoch = r.engage()
    f = r.tick()
    assert r.core.state == "holding" and epoch == r.core.epoch
    assert r.right(f) == pytest.approx([0.2, -0.25, 0.0, 0.9, 0.0, -0.1, 0.0])
    assert f.kp == list(G1D.kp) and f.kd == list(G1D.kd)
    assert f.q[13] == pytest.approx(0.07), "motor 13 held at its engage angle"


def test_tracking_converges_and_speed_is_capped():
    r = Rig()
    r.engage()
    goal = [0.6, -0.25, 0.0, 0.9, 0.0, -0.1, 0.0]
    prev = None
    max_speed = 0.0
    for _ in range(300):
        assert r.setpoint(goal) is None
        f = r.tick()
        x = r.right(f)[0]
        if prev is not None:
            max_speed = max(max_speed, abs(x - prev) / DT)
        prev = x
    assert r.core.state == "tracking"
    assert prev == pytest.approx(0.6, abs=1e-6)
    assert max_speed <= 1.0 + 1e-6


def test_expired_setpoint_decelerates_and_holds():
    r = Rig()
    r.engage()
    goal = [1.1, -0.25, 0.0, 0.9, 0.0, -0.1, 0.0]
    r.setpoint(goal, valid_ms=200)
    r.tick(20)  # moving at speed, setpoint still valid (100 ms)
    r.tick(25)  # setpoint expired at 200 ms
    assert r.core.state == "holding"
    held = r.right(r.tick())[0]
    r.tick(100)
    assert r.right(r.frames[-1])[0] == pytest.approx(held, abs=1e-9), "holding still"
    assert held < 1.1, "did not keep going to the stale target"


def test_link_lost_latches_but_keeps_sending_frames():
    r = Rig()
    r.engage()
    r.setpoint([0.3, -0.25, 0.0, 0.9, 0.0, -0.1, 0.0], valid_ms=1000)
    r.tick(10)
    r.tick(int(0.6 / DT), client_alive=None)  # Box went silent
    assert r.core.state == "fault" and r.core.latched == ["link_lost"]
    assert all(f is not None for f in r.frames[-50:]), "frames continue"
    assert r.setpoint([0.3, -0.25, 0.0, 0.9, 0.0, -0.1, 0.0]) is not None, "setpoints refused while latched"


def test_owner_reengages_after_link_lost_without_moving():
    r = Rig()
    r.engage()
    r.tick(int(0.6 / DT), client_alive=None)
    held = r.right(r.frames[-1])
    old_epoch = r.core.epoch
    assert r.send("engage") is None
    assert r.core.state == "holding" and r.core.epoch > old_epoch
    assert r.right(r.tick()) == pytest.approx(held)


def test_stale_lowstate_latches_fault_and_holds():
    r = Rig()
    r.engage()
    r.tick(5)
    r.lowstate_on = False
    r.tick(int(0.2 / DT))
    assert r.core.state == "fault" and "stale_lowstate" in r.core.latched
    assert r.frames[-1] is not None


def test_estop_latches_until_reset_and_old_epoch_is_rejected():
    r = Rig()
    r.engage()
    r.setpoint([0.5, -0.25, 0.0, 0.9, 0.0, -0.1, 0.0])
    r.tick(10)
    assert r.send("estop", client="web") is None
    r.tick(30)
    assert r.core.state == "estop"
    held = r.right(r.frames[-1])
    assert r.setpoint([0.6, -0.25, 0.0, 0.9, 0.0, -0.1, 0.0]) is not None
    assert r.send("engage") == "estop_latched"
    stale_epoch = r.core.epoch
    assert r.send("reset", client="web") is None
    assert r.core.state == "holding"
    rej = r.send("setpoint", epoch=stale_epoch, valid_for_ms=100,
                 groups={"right_arm": {"q": [0.6, -0.25, 0.0, 0.9, 0.0, -0.1, 0.0]}})
    assert rej == "stale_epoch", "a task that kept streaming must not resume after reset"
    assert r.right(r.tick(10)) == pytest.approx(held, abs=1e-9)
    assert r.send("engage") is None
    assert r.setpoint([0.55, -0.25, 0.0, 0.9, 0.0, -0.1, 0.0]) is None


def test_setpoint_validation():
    r = Rig()
    r.engage()
    base = [0.2, -0.25, 0.0, 0.9, 0.0, -0.1, 0.0]
    assert r.setpoint([0.2 + 0.6] + base[1:]).startswith("target_jump")
    assert r.setpoint([math.nan] + base[1:]).startswith("nonfinite_q")
    assert r.setpoint(base[:6]).startswith("bad_q")
    assert r.send("setpoint", epoch=r.core.epoch, valid_for_ms=100, groups={"tail": {"q": [0]}}).startswith("unknown_group")
    assert r.setpoint(base, client="intruder") == "not_owner"
    assert r.setpoint(base, kp_scale=1.5).startswith("bad_kp_scale")
    m = r.msg("setpoint", epoch=r.core.epoch, valid_for_ms=100, groups={"right_arm": {"q": base}})
    assert r.core.handle(m, r.t) is None
    assert r.core.handle(dict(m), r.t) == "old_seq"
    assert r.core.handle({"v": 2, "type": "estop", "client": "x", "seq": 1}, r.t) == "bad_version"


def test_target_outside_limits_is_clamped_and_feedforward_capped():
    cfg = load_config(builtin_config_path("g1d"))
    r = Rig(cfg)
    r.q[RIGHT.motors[3]] = 2.0  # elbow near its 2.0944 upper limit
    r.engage()
    q = [0.2, -0.25, 0.0, 2.3, 0.0, -0.1, 0.0]
    assert r.setpoint(q, tau_ff=[0, 0, 0, 50.0, 0, 0, 0]) is None
    f = r.tick(400)
    assert r.right(f)[3] == pytest.approx(RIGHT.upper[3], abs=1e-9)
    assert f.tau[RIGHT.motors[3]] == pytest.approx(8.0)
    assert r.core.clamped >= 1


def test_engage_refused_out_of_limits_no_lowstate_or_foreign_writer():
    r = Rig()
    r.q[RIGHT.motors[0]] = 3.0  # beyond 2.6704
    r.tick(1, client_alive=None)
    assert r.send("engage").startswith("out_of_limits:right_arm[0]")
    r2 = Rig()
    r2.lowstate_on = False
    r2.tick(1, client_alive=None)
    assert r2.send("engage") == "no_lowstate"
    r3 = Rig()
    r3.tick(1, client_alive=None)
    r3.core.note_foreign_writer(r3.t)
    assert r3.send("engage") == "foreign_writer"


def test_foreign_writer_while_engaged_latches_fault():
    r = Rig()
    r.engage()
    r.core.note_foreign_writer(r.t)
    r.tick()
    assert r.core.state == "fault" and "foreign_writer" in r.core.latched
    assert r.frames[-1] is not None


def test_second_client_cannot_take_over_a_live_owner_but_can_after_link_lost():
    r = Rig()
    r.engage("box")
    assert r.send("engage", client="other") == "owned_by_other"
    r.tick(int(0.6 / DT), client_alive=None)
    assert r.send("engage", client="other") is None and r.core.owner == "other"


def test_release_ramps_stiffness_then_stops_frames():
    r = Rig()
    r.engage()
    r.tick(5)
    assert r.send("release") is None
    kps = []
    for _ in range(int(G1D.release_ramp_s / DT) + 5):
        f = r.tick()
        if f is None:
            break
        kps.append(f.kp[RIGHT.motors[0]])
    assert kps[0] < G1D.kp[RIGHT.motors[0]]
    assert all(b <= a + 1e-9 for a, b in zip(kps, kps[1:]))
    assert kps[-1] == 0.0
    assert len(kps) == pytest.approx(G1D.release_ramp_s / DT, abs=3)
    assert r.core.state == "idle" and r.tick(5) is None


def test_estop_during_release_restores_stiffness_and_holds():
    r = Rig()
    r.engage()
    r.send("release")
    r.tick(int(1.5 / DT))
    assert 0.4 < r.core.scale < 0.6
    r.send("estop", client="web")
    r.tick(int(ESTOP_RESTORE_S / DT) + 2)
    f = r.frames[-1]
    assert r.core.state == "estop" and r.core.scale == 1.0
    assert f.kp[RIGHT.motors[0]] == G1D.kp[RIGHT.motors[0]]


def test_arm_sdk_weight_ramps_and_gains_stay():
    d = __import__("json").load(open(builtin_config_path("g1"), encoding="utf-8"))
    cfg = config_from_dict(d)
    r = Rig(cfg)
    r.engage()
    f = r.tick()
    assert 0.0 < f.weight < 0.05 and f.kp == list(cfg.kp)
    r.tick(int(cfg.engage_ramp_s / DT) + 2)
    assert r.frames[-1].weight == 1.0
    r.send("release")
    r.tick(int(cfg.release_ramp_s / DT) + 2)
    weights = [f.weight for f in r.frames if f is not None]
    assert weights[-1] == 0.0 and r.core.state == "idle"
