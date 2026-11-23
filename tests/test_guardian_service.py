"""GuardianService over real UDP: the loop never stops writing while engaged."""

import threading
import time

import pytest

from g1_guardian.client import GuardianClient, GuardianRejected
from g1_guardian.config import builtin_config_path, load_config
from g1_guardian.core import LowStateView
from g1_guardian.service import GuardianService

CFG = load_config(builtin_config_path("g1d"))
RIGHT = CFG.group("right_arm")
REST = [0.2, -0.25, 0.0, 0.9, 0.0, -0.1, 0.0]
MAX_GAP_S = 0.040


class FakeRobot:
    """Lowstate source + output: joints follow the commanded q exactly."""

    def __init__(self):
        self.q = [0.0] * 29
        for m, v in zip(RIGHT.motors, REST):
            self.q[m] = v
        self.lock = threading.Lock()
        self.writes = []  # (t, right_arm q, kp of first right joint)
        self.prepared = False
        self.prepare_calls = 0
        self.foreign_seen_at = float("-inf")
        self.fail_writes = 0
        self.lowstate_alive = True

    def prepare(self):
        self.prepare_calls += 1
        self.prepared = True

    def write(self, frame):
        if self.fail_writes > 0:
            self.fail_writes -= 1
            raise OSError("injected write failure")
        with self.lock:
            for g in CFG.groups:
                for m in g.motors:
                    self.q[m] = frame.q[m]
            self.writes.append((time.monotonic(), [frame.q[m] for m in RIGHT.motors], frame.kp[RIGHT.motors[0]]))

    def latest(self):
        if not self.lowstate_alive:
            return None
        with self.lock:
            return LowStateView(q=list(self.q), stamp=time.monotonic())

    def snapshot(self):
        with self.lock:
            return list(self.writes)


def max_gap(writes, t_from, t_to=None):
    t_to = time.monotonic() if t_to is None else t_to
    ts = [t_from] + [w[0] for w in writes if t_from <= w[0] <= t_to] + [t_to]
    return max(b - a for a, b in zip(ts, ts[1:]))


@pytest.fixture
def rig(tmp_path):
    robot = FakeRobot()
    svc = GuardianService(CFG, robot, robot, bind=("127.0.0.1", 0), blackbox_dir=tmp_path / "bb", log=lambda s: None)
    t = threading.Thread(target=svc.run, name="guardian_loop", daemon=True)
    t.start()
    clients = []

    def client(cid="box"):
        c = GuardianClient("127.0.0.1", svc.address[1], client_id=cid)
        clients.append(c)
        return c

    yield svc, robot, client, tmp_path / "bb"
    for c in clients:
        c.close()
    svc.stop()
    t.join(2)
    svc.close()


def stream(c, q, seconds, valid_for_ms=100, rate=200):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        c.setpoint({"right_arm": q}, valid_for_ms=valid_for_ms)
        time.sleep(1.0 / rate)


def test_engage_track_and_status(rig):
    svc, robot, client, _ = rig
    c = client()
    assert c.ping()["state"] == "idle"
    assert robot.snapshot() == [], "no frames before engage"
    c.engage()
    assert robot.prepare_calls == 1
    goal = [0.4] + REST[1:]
    stream(c, goal, 0.6)
    st = c.wait_for(lambda s: s["state"] == "tracking", 1.0)
    assert st["groups"]["right_arm"]["q_meas"][0] == pytest.approx(0.4, abs=1e-3)
    assert st["loop"]["write_failures"] == 0


def test_box_dies_mid_motion_guardian_keeps_holding(rig):
    svc, robot, client, bb = rig
    c = client()
    c.engage()
    goal = [0.9] + REST[1:]
    stream(c, goal, 0.2)  # moving towards 0.9 at up to 1 rad/s
    t_dead = time.monotonic()
    c.close()  # the Box process is gone: no setpoints, no heartbeats
    time.sleep(0.9)
    writes = robot.snapshot()
    assert max_gap(writes, t_dead) < MAX_GAP_S, "frames must not stop when the Box dies"
    late = [w[1][0] for w in writes if w[0] > t_dead + 0.4]
    assert late and max(late) - min(late) < 1e-9, "decelerated and holding still"
    assert late[-1] < 0.9, "did not continue to the old target"
    st = client("watcher").ping()
    assert st["state"] == "fault" and st["latched"] == ["link_lost"]
    svc.blackbox.wait()
    assert list(bb.glob("blackbox_*link_lost*.jsonl")), "blackbox dumped on link loss"


def test_operator_estop_from_another_client_freezes_the_task(rig):
    svc, robot, client, _ = rig
    box, web = client("box"), client("web")
    box.engage()
    stop = threading.Event()

    def task():
        q = list(REST)
        while not stop.is_set():
            q[0] = min(1.2, q[0] + 0.002)
            box.setpoint({"right_arm": q})
            time.sleep(0.005)

    th = threading.Thread(target=task, daemon=True)
    th.start()
    time.sleep(0.3)
    t_stop = time.monotonic()
    web.estop()
    time.sleep(0.5)
    writes = robot.snapshot()
    held = [w[1][0] for w in writes if w[0] > t_stop + 0.25]
    assert max(held) - min(held) < 1e-9, "task stream ignored after E-STOP"
    assert max_gap(writes, t_stop) < MAX_GAP_S
    web.reset()
    time.sleep(0.2)
    held2 = [w[1][0] for w in robot.snapshot() if w[0] > time.monotonic() - 0.1]
    assert max(held2) - min(held2) < 1e-9, "old stream (stale epoch) must not resume motion after reset"
    stop.set()
    th.join(1)
    st = box.status()
    assert (st["last_reject"] or {}).get("reason") in ("stale_epoch", "state_estop")
    with pytest.raises(GuardianRejected):
        web.engage()  # the box still owns it and is alive


def test_write_failures_do_not_stop_the_loop(rig):
    svc, robot, client, _ = rig
    c = client()
    c.engage()
    time.sleep(0.1)
    robot.fail_writes = 20
    time.sleep(0.4)
    st = c.wait_for(lambda s: "loop" in s, 1.0)
    assert st["loop"]["write_failures"] == 20
    assert robot.snapshot()[-1][0] > time.monotonic() - 0.05, "writing again"


def test_stale_lowstate_latches_fault_and_frames_continue(rig):
    svc, robot, client, _ = rig
    c = client()
    c.engage()
    time.sleep(0.1)
    t0 = time.monotonic()
    robot.lowstate_alive = False
    time.sleep(0.4)
    assert max_gap(robot.snapshot(), t0) < MAX_GAP_S
    st = c.wait_for(lambda s: s["state"] == "fault", 1.0)
    assert "stale_lowstate" in st["latched"]


def test_release_ramps_then_stops(rig):
    svc, robot, client, _ = rig
    c = client()
    c.engage()
    time.sleep(0.1)
    t0 = time.monotonic()
    c.release()
    time.sleep(CFG.release_ramp_s + 0.4)
    writes = [w for w in robot.snapshot() if w[0] > t0]
    kps = [w[2] for w in writes]
    assert kps[0] > 0 and kps[-1] == 0.0
    assert all(b <= a + 1e-9 for a, b in zip(kps, kps[1:]))
    assert writes[-1][0] - t0 == pytest.approx(CFG.release_ramp_s, abs=0.15)
    n = len(robot.snapshot())
    time.sleep(0.1)
    assert len(robot.snapshot()) == n, "frames stop only after the ramp"


def test_graceful_shutdown_releases_before_exiting(tmp_path):
    robot = FakeRobot()
    svc = GuardianService(CFG, robot, robot, bind=("127.0.0.1", 0), log=lambda s: None)
    t = threading.Thread(target=svc.run, daemon=True)
    t.start()
    c = GuardianClient("127.0.0.1", svc.address[1], client_id="box")
    try:
        c.engage()
        time.sleep(0.1)
        t0 = time.monotonic()
        svc.request_shutdown()  # what SIGTERM does
        t.join(CFG.release_ramp_s + 2.0)
        assert not t.is_alive()
        writes = [w for w in robot.snapshot() if w[0] > t0]
        assert writes[-1][2] == 0.0 and writes[-1][0] - t0 >= CFG.release_ramp_s - 0.1
    finally:
        c.close()
        svc.close()
