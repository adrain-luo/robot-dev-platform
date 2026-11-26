"""GuardianCore: limits, holds, latches and ramps, driven by a fake clock."""

import dataclasses
import json
import math

import pytest

from g1_guardian.config import AccessConfig, ConfigError, builtin_config_path, config_from_dict, load_config
from g1_guardian.core import (
    ALL_COMMANDS,
    ESTOP_RESTORE_S,
    MAX_REFUSED_SOURCES,
    REFUSED_EVENT_INTERVAL_S,
    GuardianCore,
    LowStateView,
    track_joint,
)
from g1_guardian.protocol import COMMAND_TYPES

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

    def send(self, kind, client="box", source=None, **fields):
        return self.core.handle(self.msg(kind, client, **fields), self.t, source=source)

    def lowstate(self):
        return LowStateView(q=list(self.q), stamp=self.t) if self.lowstate_on else None

    def tick(self, n=1, client_alive="box", alive_source=None):
        out = None
        for _ in range(n):
            self.t += DT
            if client_alive:
                self.core.handle(self.msg("heartbeat", client_alive), self.t, source=alive_source)
            out = self.core.step(self.t, self.lowstate())
            self.frames.append(out)
            if out is not None:
                for g in self.cfg.groups:
                    for m in g.motors:
                        self.q[m] = out.q[m]
        return out

    def engage(self, client="box", source=None):
        self.tick(1, client_alive=None)  # deliver a lowstate first
        assert self.send("engage", client, source=source) is None
        return self.core.epoch

    def setpoint(self, q7, client="box", valid_ms=100, source=None, **extra):
        body = {"q": list(q7)}
        body.update(extra)
        return self.send("setpoint", client, source=source, epoch=self.core.epoch, valid_for_ms=valid_ms,
                         groups={"right_arm": body})

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


# ------------------------------------------------------------- teach / stiffness
def _kp(frame, m=RIGHT.motors[0]):
    return frame.kp[m] / G1D.kp[m]


def _drag(r, joint, total, seconds, tau=None):
    """Move one right-arm joint by hand while the Box streams q = measured, kp 0."""
    n = int(round(seconds / DT))
    m = RIGHT.motors[joint]
    for _ in range(n):
        r.q[m] += total / n
        body = {"kp_scale": 0.0}
        if tau is not None:
            body["tau_ff"] = tau
        assert r.setpoint([r.q[x] for x in RIGHT.motors], **body) is None
        r.tick()


def test_teach_follows_the_hand_and_ramps_stiffness_down():
    r = Rig()
    r.engage()
    r.tick(5)
    assert r.setpoint(r.right(r.frames[-1]), kp_scale=0.0) is None
    r.tick()
    _drag(r, 0, 0.8, 0.5)  # 1.6 rad/s: faster than the tracking limit, must not lag
    last = r.frames[-1]
    assert r.right(last)[0] == pytest.approx(r.q[RIGHT.motors[0]], abs=1e-9)
    assert r.core.q_cmd["right_arm"][0] == pytest.approx(0.2 + 0.8, abs=1e-9)
    assert _kp(last) == 0.0
    assert last.kd[RIGHT.motors[0]] == pytest.approx(G1D.compliant_kd)
    # Ramped, not stepped: stiffness falls by at most dt / stiffness_ramp_s per frame.
    kps = [_kp(f) for f in r.frames if f is not None]
    steps = [a - b for a, b in zip(kps, kps[1:])]
    assert max(steps) <= DT / G1D.stiffness_ramp_s + 1e-9
    st = r.core.status(r.t)["groups"]["right_arm"]
    assert st["compliant"] is True and st["kp_scale"] == 0.0 and st["motors"] == list(RIGHT.motors)
    # The left arm was not in the setpoint: it stays stiff.
    assert last.kp[G1D.group("left_arm").motors[0]] == G1D.kp[G1D.group("left_arm").motors[0]]


def test_leaving_teach_holds_where_the_hand_left_it():
    r = Rig()
    r.engage()
    r.tick(5)
    _drag(r, 3, -0.4, 0.3)
    left_at = [r.q[m] for m in RIGHT.motors]
    for _ in range(int(0.5 / DT)):
        assert r.setpoint(left_at, kp_scale=1.0) is None
        r.tick()
    after = r.frames[-1]
    assert _kp(after) == pytest.approx(1.0)
    assert after.kd[RIGHT.motors[3]] == pytest.approx(G1D.kd[RIGHT.motors[3]])
    # No spring-back towards the pre-teach pose.
    assert r.right(after) == pytest.approx(left_at, abs=1e-9)


@pytest.mark.parametrize("how", ["expire", "link_lost", "estop"])
def test_any_hold_restores_full_stiffness_at_the_measured_pose(how):
    r = Rig()
    r.engage()
    r.tick(5)
    _drag(r, 1, 0.3, 0.2)
    where = [r.q[m] for m in RIGHT.motors]
    if how == "estop":
        assert r.send("estop", "panel") is None
    alive = None if how == "link_lost" else "box"
    r.tick(int(1.2 / DT), client_alive=alive)
    last = r.frames[-1]
    assert _kp(last) == pytest.approx(1.0)
    assert r.right(last) == pytest.approx(where, abs=1e-9)
    assert r.core.status(r.t)["groups"]["right_arm"]["compliant"] is False
    expected = {"expire": "holding", "link_lost": "fault", "estop": "estop"}[how]
    assert r.core.state == expected


def test_waist_is_held_with_the_platform_lock_gains():
    r = Rig()
    r.engage()
    f = r.tick(2)
    assert (f.kp[12], f.kd[12]) == (200.0, 4.0)
    assert (f.kp[13], f.kd[13]) == (200.0, 4.0)
    assert f.q[13] == pytest.approx(0.07)


def test_status_reports_foreign_writer_age():
    r = Rig()
    r.tick()
    assert r.core.status(r.t)["foreign_writer_age_ms"] is None
    r.core.note_foreign_writer(r.t)
    r.tick(10, client_alive=None)
    assert r.core.status(r.t)["foreign_writer_age_ms"] == pytest.approx(10 * DT * 1000.0, abs=0.2)


def test_engage_refused_when_leg_motors_are_not_idle():
    r = Rig()
    r.q[3] = 0.63  # a bent knee: this is a standing biped, not a G1D
    r.tick(1, client_alive=None)
    assert r.send("engage") == "motors_not_idle:3"
    assert r.core.state == "idle"
    r.q[3] = 0.0
    r.tick(1, client_alive=None)
    assert r.send("engage") is None


# ---------------------------------------------------------------------- access
BOX, BOX2, BODY, LOCAL, MON, STRANGER = (
    "192.168.123.5", "192.168.123.6", "192.168.123.164", "127.0.0.1", "192.168.123.9", "192.168.123.77")
REST7 = [0.2, -0.25, 0.0, 0.9, 0.0, -0.1, 0.0]


def access_rig():
    acc = AccessConfig(local=(LOCAL, BODY), control=(BOX, BOX2), monitor=(MON,))
    return Rig(dataclasses.replace(G1D, access=acc))


def test_all_commands_match_the_protocol():
    assert ALL_COMMANDS == set(COMMAND_TYPES)


@pytest.mark.parametrize("name", ["g1d", "g1"])
def test_shipped_configs_give_the_box_control_and_this_computer_local(name):
    acc = load_config(builtin_config_path(name)).access
    assert acc.local == ("127.0.0.1", "192.168.123.164")
    assert acc.control == ("192.168.123.5",)
    assert acc.monitor == ()


@pytest.mark.parametrize("access, error", [
    (None, "missing access section"),
    ({"local": ["127.0.0.1"], "control": ["box"]}, "not an IP address"),
    ({"local": ["127.0.0.1"], "control": ["::1"]}, "IPv4 only"),
    ({"local": ["127.0.0.1"], "control": ["192.168.123.05"]}, "not an IP address"),
    ({"local": ["127.0.0.1"], "control": ["192.168.123.5"], "monitor": ["192.168.123.5"]}, "more than once"),
    ({"local": ["127.0.0.1"], "control": ["127.0.0.2"]}, "belongs in local"),
    ({"local": [], "control": [], "monitor": ["192.168.123.9"]}, "nobody could engage"),
    ({"local": ["127.0.0.1"], "controllers": ["192.168.123.5"]}, "unknown keys"),
    ({"local": "127.0.0.1"}, "expected a list"),
])
def test_access_section_is_required_and_checked(access, error):
    d = json.loads(builtin_config_path("g1d").read_text())
    if access is None:
        del d["access"]
    else:
        d["access"] = access
    with pytest.raises(ConfigError, match=error):
        config_from_dict(d)


def test_unknown_source_is_refused_for_every_command_and_changes_nothing():
    r = access_rig()
    r.tick(1, client_alive=None)
    for kind in ("engage", "estop", "reset", "release", "heartbeat"):
        assert r.send(kind, client="box", source=STRANGER) == "source_not_allowed"
    assert r.setpoint(REST7, source=STRANGER) == "source_not_allowed"
    assert r.core.state == "idle" and r.core.epoch == 0 and r.core.latched == []
    assert r.core.rejects == {"source_not_allowed": 6} and r.core.last_reject is None
    # Engaged by the Box: a stranger can neither stop nor lower the arm.
    r.engage(source=BOX)
    epoch = r.core.epoch
    assert r.send("estop", source=STRANGER) == "source_not_allowed"
    assert r.send("release", source=STRANGER) == "source_not_allowed"
    r.tick(5, alive_source=BOX)
    assert r.core.state == "holding" and r.core.epoch == epoch and r.frames[-1] is not None


def test_refused_sources_are_logged_at_most_once_a_second_and_counted_in_bounds():
    r = access_rig()
    for _ in range(100):
        r.send("estop", source=STRANGER)
    assert [e for _, e in r.core.events] == [f"refused_source:{STRANGER} count=1"]
    r.t += REFUSED_EVENT_INTERVAL_S
    r.send("estop", source=STRANGER)
    assert r.core.events[-1][1] == f"refused_source:{STRANGER} count=101"
    for i in range(2 * MAX_REFUSED_SOURCES):
        r.send("estop", source=f"10.0.{i // 250}.{i % 250 + 1}")
    assert len(r.core.refused_sources) == MAX_REFUSED_SOURCES + 1
    assert r.core.refused_sources["other"] == MAX_REFUSED_SOURCES + 1  # STRANGER took one slot


def test_monitor_may_only_listen_and_estop():
    r = access_rig()
    r.engage(source=BOX)
    assert r.send("heartbeat", client="watch", source=MON) is None
    assert r.send("engage", client="watch", source=MON) == "not_allowed:monitor"
    assert r.setpoint(REST7, client="watch", source=MON) == "not_allowed:monitor"
    assert r.send("release", client="watch", source=MON) == "not_allowed:monitor"
    assert r.core.owner == "box" and r.core.state == "holding"
    assert r.send("estop", client="watch", source=MON) is None
    assert r.core.state == "estop"
    assert r.send("reset", client="watch", source=MON) == "not_allowed:monitor"
    assert r.core.latched == ["estop"]


def test_owner_is_bound_to_the_address_it_engaged_from():
    r = access_rig()
    r.engage("box", source=BOX)
    assert r.core.owner_source == BOX
    goal = [0.3, -0.25, 0.0, 0.9, 0.0, -0.1, 0.0]
    assert r.setpoint(goal, client="box", source=BOX2) == "not_owner", "same id, other address"
    assert r.send("engage", client="box", source=BOX2) == "owned_by_other"
    assert r.setpoint(goal, client="box", source=BOX) is None
    assert r.core.state == "tracking"


def test_a_copied_owner_id_does_not_keep_the_link_alive():
    r = access_rig()
    r.engage("box", source=BOX)
    r.tick(int(0.6 / DT), client_alive="box", alive_source=BOX2)  # only the impostor talks
    assert r.core.state == "fault" and r.core.latched == ["link_lost"]
    assert all(f is not None for f in r.frames[-50:]), "frames continue"


def test_release_only_from_the_owner_or_this_computer():
    r = access_rig()
    r.engage("box", source=BOX)
    assert r.send("release", client="other", source=BOX) == "not_owner"
    assert r.send("release", client="box", source=BOX2) == "not_owner"
    r.tick(5, alive_source=BOX)
    assert r.core.state == "holding" and r.frames[-1].kp == list(G1D.kp)
    assert r.send("release", client="cli", source=BODY) is None, "the operator's CLI on the body"
    assert r.core.state == "releasing"


@pytest.mark.parametrize("client, source", [("box", BOX), ("cli", LOCAL)])
def test_release_is_accepted_while_estop_is_latched(client, source):
    """Owner's decision (2026-11-26): an E-STOP latch does not block release."""
    r = access_rig()
    r.engage("box", source=BOX)
    r.tick(3, alive_source=BOX)
    assert r.send("estop", client="web", source=BOX) is None
    assert r.core.state == "estop"
    assert r.send("release", client="other", source=BOX2) == "not_owner", "still only the owner or local"
    assert r.send("release", client=client, source=source) is None
    assert r.core.state == "releasing"
    for _ in range(int(G1D.release_ramp_s / DT) + 5):
        if r.tick(alive_source=BOX) is None:
            break
    assert r.core.state == "idle" and r.core.owner is None and r.core.owner_source is None
    assert r.frames[-2].kp[RIGHT.motors[0]] == 0.0, "frames stopped only after the ramp"


def test_reset_from_control_or_local_even_when_not_the_owner():
    r = access_rig()
    r.engage("box", source=BOX)
    r.send("estop", client="web", source=BOX)
    assert r.send("reset", client="page", source=BOX2) is None
    assert r.core.latched == [] and r.core.state == "holding"
    r.send("estop", client="web", source=BOX)
    assert r.send("reset", client="cli", source=LOCAL) is None
    assert r.core.latched == []
