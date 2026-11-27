"""运控核心进程（g1_motion_core）：真 Unix 套接字 + 真 ArmLoop / MotionCore + mock 机器人。

核心在本测试进程里跑（帧由 Tee 记下来，能查帧间隔）；客户端是同进程的连接，或被 SIGKILL 的子进程。
要点：客户端断开 / 心跳超时 → 只撤销它自己的租约、停住它手动动着的手，帧不断；
急停走自己的套接字，任务套接字被灌满时照样马上锁存。
"""

import os
import shutil
import socket
import subprocess
import sys
import tempfile
import textwrap
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from g1_contracts import wire
from g1_contracts.motion import CONTRACT_VERSION, LeaseRevoked, MotionRejected, Reason, Resource
from g1_motion_client import CoreConnection, CoreError, MotionClient
from g1_motion_core.server import CoreServer, ServerStartError
from g1_motion_core.service import CoreProcess, jsonable
from robot_platform.control.fk_scene import RobotKinematics
from robot_platform.core.config_loader import load_settings
from robot_platform.hw.backends.base import Commander
from robot_platform.hw.backends.mock import MockBackend

R, L = Resource.RIGHT_ARM, Resource.LEFT_ARM
SRC = Path(__file__).resolve().parents[1] / "src"
MAX_GAP_S = 0.1  # 100 Hz 循环；这台测试机调度抖动大


class Tee(Commander):
    """mock 的指令出口照常驱动仿真机器人，同时记下每一帧的时间和刚度。"""

    def __init__(self, inner):
        self.inner = inner
        self.times = []
        self.kp = []
        self._lock = threading.Lock()

    def prepare(self):
        self.inner.prepare()

    def write(self, q, kp, kd, tau, *, mode_machine=None):
        self.inner.write(q, kp, kd, tau, mode_machine=mode_machine)
        with self._lock:
            self.times.append(time.monotonic())
            self.kp.append(list(kp))

    def max_gap_since(self, t0, t1=None):
        t1 = time.monotonic() if t1 is None else t1
        with self._lock:
            ts = [t0] + [t for t in self.times if t0 <= t <= t1] + [t1]
        return max(b - a for a, b in zip(ts, ts[1:]))


@pytest.fixture
def sockdir():
    d = tempfile.mkdtemp(prefix="g1m")  # AF_UNIX 路径最长 108 字节：别用 pytest 的长 tmp_path
    yield d
    shutil.rmtree(d, ignore_errors=True)


@pytest.fixture
def proc(sockdir):
    settings = load_settings(backend="mock")
    settings.max_speed_rad_s = 0.4
    profile = settings.profile
    backend = MockBackend(profile)
    tee = Tee(backend.commander())
    backend.commander = lambda: tee
    kin = RobotKinematics(settings.urdf, settings.base_link, list(profile.arms))
    p = CoreProcess(settings, kin, backend, log=lambda s: None)
    p.tee = tee
    p.serve(sockdir)
    yield p
    p.close(timeout_s=2.0)


def connect(proc, name="t", **kw):
    c = CoreConnection(proc.server.directory, client=name, **kw)
    c.connect()
    return c


@pytest.fixture
def conns(proc):
    made = []

    def make(name="t", **kw):
        c = connect(proc, name, **kw)
        made.append(c)
        return c
    yield make
    for c in made:
        c.close()


def wait_until(pred, timeout_s=2.0, what="条件"):
    deadline = time.monotonic() + timeout_s
    while not pred():
        assert time.monotonic() < deadline, f"等不到：{what}"
        time.sleep(0.005)


def arm_st(proc, arm="right"):
    return proc.loop.arm_status(arm)


def away(proc, arm="right", d=-0.5):
    q = np.array(arm_st(proc, arm)["cmd_rad"])
    q[3] += d
    return q.tolist()


def held_at_command(proc, arm="right"):
    st = arm_st(proc, arm)
    return np.allclose(st["desired_rad"], st["cmd_rad"])


# ---------------------------------------------------------------- 基本调用
def test_info_status_lowstate_and_a_leased_goal(proc, conns):
    c = conns("web")
    info = c.core_info
    assert info["contract_version"] == CONTRACT_VERSION and info["heartbeat_timeout_ms"] == 250
    assert info["arms"]["right"]["motors"] == list(proc.profile.arms["right"].motor_indices)
    assert len(info["arms"]["right"]["limits"]) == 7
    c.call("engage", timeout_s=30)
    st = c.call("page_status")
    assert st["engaged"] is True and st["motion"]["safety"] == "normal"
    assert st["core_process"]["pid"] == os.getpid() and st["core_process"]["clients"][0]["client"] == "web"
    low = c.call("lowstate")
    assert len(low["q"]) == proc.profile.num_motors and low["age_s"] is not None

    client = MotionClient(c)
    with client.lease([R], "task:demo") as lease:
        assert proc.core.status()["leases"][0]["token"] == lease.owner
        tgt = away(proc)
        assert lease.goal(R, [tgt]).ok
        assert arm_st(proc)["desired_rad"] == pytest.approx(tgt)
        assert client.state(R).lease_owner == "task:demo"
        s = c.call("arm_sample", arm="right", wait=True)
        assert len(s["q_meas"]) == 7 and len(s["g_torso"]) == 3
    assert lease.cancelled.is_set()
    assert proc.core.status()["leases"] == []


def test_errors_keep_their_kind(proc, conns):
    c = conns()
    with pytest.raises(CoreError) as e:
        c.call("manual_target", arm="right", q=[0.0] * 7)
    assert e.value.kind == "permission" and "接管" in str(e.value)
    with pytest.raises(CoreError) as e:
        c.call("hold_arm", arm="tail")
    assert e.value.kind == "value"
    with pytest.raises(CoreError) as e:
        c.call("no_such_op")
    assert e.value.kind == "unsupported"
    with pytest.raises(MotionRejected) as e:
        MotionClient(c).lease([R], "task:t")
    assert e.value.reason is Reason.NOT_ENGAGED
    c.call("engage", timeout_s=30)
    with pytest.raises(MotionRejected) as e:
        MotionClient(c).lease([R], "task:t", heartbeat_ms=100)
    assert e.value.reason is Reason.UNSUPPORTED


def test_another_connections_lease_is_never_honoured(proc, conns):
    a, b = conns("a"), conns("b")
    a.call("engage", timeout_s=30)
    la = MotionClient(a).lease([R], "task:a")
    forged = MotionClient(b)
    from g1_motion_client.client import RemoteLease
    fake = RemoteLease(b, la.lease, "x")
    with pytest.raises(MotionRejected) as e:
        fake.goal(R, [away(proc)])
    assert e.value.reason is Reason.UNKNOWN_LEASE
    assert not la.cancelled.is_set() and la.goal(R, [away(proc, d=-0.1)]).ok
    del forged


# ---------------------------------------------------------------- 断开 / 心跳
def test_disconnect_revokes_only_that_connections_lease_and_frames_keep_going(proc, conns):
    a, b = conns("task-a"), conns("task-b")
    a.call("engage", timeout_s=30)
    la = MotionClient(a).lease([R], "task:a")
    lb = MotionClient(b).lease([L], "task:b")
    la.goal(R, [away(proc, d=-0.6)])
    time.sleep(0.1)
    t0 = time.monotonic()
    a.close()  # 服务端看到 EOF
    wait_until(lambda: [x["owner"] for x in proc.core.status()["leases"]] == ["task:b"], 1.0, "只撤销 a 的租约")
    t_revoked = time.monotonic() - t0
    assert held_at_command(proc, "right"), "被撤销的手停在指令角"
    cmd = np.array(arm_st(proc)["cmd_rad"])
    time.sleep(0.3)
    assert np.array(arm_st(proc)["cmd_rad"]) == pytest.approx(cmd), "之后不再动"
    assert proc.tee.max_gap_since(t0) < MAX_GAP_S, "帧不断"
    assert lb.goal(L, [away(proc, "left", d=-0.1)]).ok, "另一个连接的租约照常"
    assert t_revoked < 0.1, f"断开要马上被核心看到（{t_revoked * 1000:.0f} ms）"


def test_missed_heartbeats_revoke_the_lease_and_hold(proc, conns):
    quiet = conns("stalled-task", heartbeat_ms=60_000)  # 不发心跳：像进程卡死
    quiet.call("engage", timeout_s=30)
    lease = MotionClient(quiet).lease([R], "task:stalled")
    t_sent = time.monotonic()  # 核心最后一次收到消息不早于此刻（goal 本身要过碰撞检查，应答会晚一些）
    lease.goal(R, [away(proc, d=-0.6)])
    t_answered = time.monotonic()
    assert lease.cancelled.wait(2.0), "核心推送了撤销"
    t_revoked = time.monotonic()
    assert t_revoked - t_sent >= 0.25, t_revoked - t_sent
    assert t_revoked - t_answered < 0.6, t_revoked - t_answered
    assert "心跳超时" in lease.revoked
    assert held_at_command(proc)
    with pytest.raises(LeaseRevoked):
        lease.goal(R, [away(proc)])
    assert quiet.call("max_speed") == pytest.approx(0.4), "连接本身还能用（只读、重新申请由任务决定）"


def test_heartbeats_keep_an_idle_lease_alive(proc, conns):
    c = conns("idle-task")
    c.call("engage", timeout_s=30)
    lease = MotionClient(c).lease([R], "task:idle")
    time.sleep(1.0)
    assert not lease.cancelled.is_set()
    lease.check()


def test_manual_moves_are_held_when_the_web_goes_away(proc, conns):
    web, task = conns("web"), conns("task")
    web.call("engage", timeout_s=30)
    lt = MotionClient(task).lease([L], "task:left")
    tgt_l = away(proc, "left", d=-0.4)
    lt.goal(L, [tgt_l])
    web.call("manual_target", arm="right", q=away(proc, d=-0.6))
    time.sleep(0.15)
    assert arm_st(proc)["moving"]
    t0 = time.monotonic()
    web.close()
    wait_until(lambda: held_at_command(proc, "right"), 1.0, "网页断开后右手停住")
    assert arm_st(proc, "left")["desired_rad"] == pytest.approx(tgt_l), "任务租着的左手不受影响"
    assert proc.tee.max_gap_since(t0) < MAX_GAP_S


def test_teach_ends_at_full_stiffness_when_the_web_goes_away(proc, conns):
    web = conns("web")
    web.call("engage", timeout_s=30)
    web.call("enable_teach", arm="right")
    right = proc.profile.arms["right"].motor_indices
    wait_until(lambda: proc.tee.kp and proc.tee.kp[-1][right[0]] == 0.0, 1.0, "拖拽：刚度 0")
    web.close()
    wait_until(lambda: not arm_st(proc)["float"], 1.0, "退出拖拽")
    wait_until(lambda: proc.tee.kp[-1][right[0]] == pytest.approx(proc.profile.kp[right[0]]), 1.0, "全刚度")


def test_killed_client_process_loses_its_lease_at_once(proc, sockdir):
    """真的另一个进程：申请租约、下目标后被 SIGKILL。核心马上撤销、停手，帧不断。"""
    code = textwrap.dedent(f"""
        import sys, time
        sys.path.insert(0, {str(SRC)!r})
        from g1_contracts.motion import Resource
        from g1_motion_client import CoreConnection, MotionClient
        c = CoreConnection({sockdir!r}, client="doomed")
        c.call("engage", timeout_s=30)
        lease = MotionClient(c).lease([Resource.RIGHT_ARM], "task:doomed")
        q = [float(x) for x in sys.argv[1].split(",")]
        lease.goal(Resource.RIGHT_ARM, [q])
        print("ready", flush=True)
        time.sleep(60)
    """)
    tgt = away(proc, d=-0.6)
    child = subprocess.Popen([sys.executable, "-c", code, ",".join(str(v) for v in tgt)],
                             stdout=subprocess.PIPE, text=True)
    try:
        assert child.stdout.readline().strip() == "ready"
        assert proc.core.status()["leases"][0]["owner"] == "task:doomed"
        time.sleep(0.1)
        t0 = time.monotonic()
        child.kill()
        child.wait(5)
        wait_until(lambda: proc.core.status()["leases"] == [], 1.0, "租约撤销")
        assert time.monotonic() - t0 < 0.3
        assert held_at_command(proc)
        assert np.max(np.abs(np.array(arm_st(proc)["cmd_rad"]) - tgt)) > 0.05, "停在半路"
        assert proc.tee.max_gap_since(t0) < MAX_GAP_S
        assert proc.core.status()["last_revoke"]["reason"].startswith("doomed 连接断开")
    finally:
        if child.poll() is None:
            child.kill()


# ---------------------------------------------------------------- 急停
def test_estop_latches_at_once_while_core_sock_is_flooded(proc, conns):
    holder, flood = conns("task"), conns("flood")
    holder.call("engage", timeout_s=30)
    lease = MotionClient(holder).lease([R], "task:t")
    lease.goal(R, [away(proc, d=-0.6)])
    stop = threading.Event()
    replies = {"ok": 0, "busy": 0}

    def spam():
        while not stop.is_set():
            try:
                flood.call("page_status")
                replies["ok"] += 1
            except CoreError as exc:
                if exc.kind == "busy":
                    replies["busy"] += 1

    threads = [threading.Thread(target=spam, daemon=True) for _ in range(16)]
    for t in threads:
        t.start()
    try:
        time.sleep(0.5)
        estop = conns("panel")
        t0 = time.monotonic()
        estop.estop("页面急停")
        latency = time.monotonic() - t0
        assert proc.core.latched and lease.cancelled.wait(1.0)
    finally:
        stop.set()
        for t in threads:
            t.join(2.0)
    print(f"estop latency under flood: {latency * 1000:.1f} ms, flood replies {replies}")
    assert latency < 0.5, latency
    assert replies["busy"] > 0, "灌满时多出来的调用马上回 busy，不排队"
    assert held_at_command(proc)
    with pytest.raises(LeaseRevoked):
        lease.goal(R, [away(proc)])
    holder.call("reset_estop")
    assert not proc.core.latched


def test_reset_estop_carries_the_expected_epoch_over_ipc(proc, conns):
    """网页（ipc）的复位确认把人看到的 epoch 交给核心，在解除的同一把锁里再比一次。"""
    panel = conns("panel")
    panel.estop("页面急停")
    seen = proc.core.epoch
    panel.estop("又按了一次")
    with pytest.raises(CoreError) as e:
        panel.call("reset_estop", expect_epoch=seen)
    assert e.value.kind == "permission" and "锁存变了" in e.value.detail
    assert proc.core.latched
    panel.call("reset_estop", expect_epoch=proc.core.epoch)
    assert not proc.core.latched


# ---------------------------------------------------------------- 协议 / 启动
def test_wrong_version_or_no_hello_is_refused(proc):
    for first in (wire.encode({"v": "1.0", "type": "hello", "seq": 1, "client": "old"}),
                  wire.encode(wire.message("safety", 1))):
        s = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        s.settimeout(2.0)
        s.connect(proc.server.core_path)
        s.send(first)
        reply = wire.decode(s.recv(1 << 20), ("reply",))
        assert reply["ok"] is False and reply["error"]["kind"] in ("version", "unsupported")
        assert s.recv(1 << 20) == b"", "随后断开"
        s.close()


def test_a_second_core_refuses_to_start_and_stale_files_are_replaced(proc, sockdir):
    with pytest.raises(ServerStartError, match="one core per machine"):
        CoreServer(proc.core, sockdir, log=lambda s: None).start()
    d = tempfile.mkdtemp(prefix="g1m")
    try:
        for name in ("core.sock", "estop.sock"):  # 上一个核心崩了留下的文件
            s = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            s.bind(os.path.join(d, name))
            s.close()
        srv = CoreServer(proc.core, d, log=lambda s: None).start()
        assert oct(os.stat(srv.core_path).st_mode & 0o777) == oct(0o660)
        srv.stop()
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_too_many_calls_in_flight_are_answered_busy(sockdir):
    release = threading.Event()
    srv = CoreServer(object(), sockdir, ops={"block": lambda ctx, a: release.wait(5)}, max_in_flight=1,
                     log=lambda s: None).start()
    c = CoreConnection(sockdir, client="t")
    try:
        th = threading.Thread(target=lambda: c.call("block"), daemon=True)
        th.start()
        time.sleep(0.2)
        with pytest.raises(CoreError) as e:
            c.call("block")
        assert e.value.kind == "busy"
        release.set()
        th.join(2.0)
    finally:
        c.close()
        srv.stop()


def test_jsonable_makes_numpy_and_nan_safe():
    out = jsonable({"a": np.float32(1.5), "b": np.array([1, 2]), "c": (np.bool_(True), float("nan")), 3: None})
    assert out == {"a": 1.5, "b": [1, 2], "c": [True, None], "3": None}
    wire.encode(wire.message("reply", 1, result=out))


def test_lease_speed_travels_over_the_socket(proc, conns):
    """D3：refactor 的租约带自己的限速（现场 1.0），页面的全局限速（这里 0.4）不受影响。"""
    c = conns("refactor-node")
    c.call("engage", timeout_s=30)
    with MotionClient(c).lease([R], "refactor-node", max_speed_rad_s=1.0):
        assert arm_st(proc)["speed_rad_s"] == 1.0 and arm_st(proc, "left")["speed_rad_s"] == 0.4
    assert arm_st(proc)["speed_rad_s"] == 0.4
    with pytest.raises(MotionRejected) as e:
        MotionClient(c).lease([R], "refactor-node", max_speed_rad_s=2.0)
    assert e.value.reason is Reason.INVALID
