"""g1_motion_client 对着一个按脚本应答的假核心（真 Unix 套接字）：握手、心跳、租约事件、断线、错误、急停。"""

import json
import os
import socket
import threading
import time

import pytest

from g1_contracts import wire
from g1_contracts.motion import (
    CONTRACT_VERSION, Ack, Lease, LeaseRevoked, MotionRejected, Reason, Resource, Verdict,
)
from g1_motion_client import CoreConnection, CoreError, CoreUnavailable, MotionClient
from g1_motion_client import connection as connmod

R = Resource.RIGHT_ARM


class FakeCore:
    """core.sock + estop.sock；收到的消息都记下来，请求由 handler(msg) 应答（返回字段或抛 wire.WireError）。"""

    def __init__(self, directory, *, version=CONTRACT_VERSION):
        self.dir = str(directory)
        self.version = version
        self.got = []
        self.estops = []
        self.handler = lambda msg: {}
        self.estop_handler = lambda msg: None
        self.conns = []
        self.stop = threading.Event()
        self.lis = self._bind("core.sock")
        self.elis = self._bind("estop.sock")
        threading.Thread(target=self._accept, args=(self.lis, self._serve), daemon=True).start()
        threading.Thread(target=self._accept, args=(self.elis, self._serve_estop), daemon=True).start()

    def _bind(self, name):
        s = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        s.bind(os.path.join(self.dir, name))
        s.listen(8)
        s.settimeout(0.1)
        return s

    def _accept(self, lis, serve):
        while not self.stop.is_set():
            try:
                c, _ = lis.accept()
            except (socket.timeout, OSError):
                continue
            self.conns.append(c)
            threading.Thread(target=serve, args=(c,), daemon=True).start()

    def _send(self, c, kind, seq, **f):
        msg = {"v": self.version, "type": kind, "seq": seq, **f}
        try:
            c.send(json.dumps(msg).encode())
        except OSError:
            pass

    def _serve(self, c):
        while True:
            try:
                data = c.recv(1 << 20)
            except OSError:
                return
            if not data:
                return
            msg = json.loads(data)
            self.got.append(msg)
            if msg["type"] == "heartbeat":
                continue
            if msg["type"] == "hello":
                self._send(c, "reply", msg["seq"], ok=True, core={"name": "fake"})
                continue
            try:
                fields = self.handler(msg)
            except wire.WireError as exc:
                self._send(c, "reply", msg["seq"], ok=False, error=wire.error(exc.kind, exc.detail))
                continue
            if fields is None:
                continue  # never answer
            if "error" in fields:
                self._send(c, "reply", msg["seq"], ok=False, **fields)
            else:
                self._send(c, "reply", msg["seq"], ok=True, **fields)

    def _serve_estop(self, c):
        while True:
            try:
                data = c.recv(1 << 20)
            except OSError:
                return
            if not data:
                return
            msg = json.loads(data)
            self.estops.append(msg)
            err = self.estop_handler(msg)
            if err:
                self._send(c, "reply", msg["seq"], ok=False, error=err)
            else:
                self._send(c, "reply", msg["seq"], ok=True)

    def push(self, kind, **f):
        for c in list(self.conns):
            self._send(c, kind, 999, **f)

    def kick(self):
        """服务端断开所有连接（模拟核心崩溃）。"""
        for c in list(self.conns):
            try:
                c.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            c.close()
        self.conns.clear()

    def types(self):
        return [m["type"] for m in self.got]

    def close(self):
        self.stop.set()
        self.kick()
        self.lis.close()
        self.elis.close()


@pytest.fixture
def fake(tmp_path):
    f = FakeCore(tmp_path)
    yield f
    f.close()


@pytest.fixture
def conn(fake):
    c = CoreConnection(fake.dir, client="test-client", call_timeout_s=2.0)
    yield c
    c.close()


LEASE = Lease(lease_id="L1", epoch=0, resources=(R,), owner="task:t")


def _lease_handler(acks):
    """lease 给 L1；goal / stream 依次用 acks 里的 Ack 应答。"""
    acks = list(acks)

    def handler(msg):
        if msg["type"] == "lease":
            return {"lease": wire.to_wire(LEASE), "token": "task:t#1"}
        if msg["type"] in ("goal", "stream", "hold"):
            return {"ack": wire.to_wire(acks.pop(0))}
        return {}
    return handler


def wait_until(pred, timeout_s=2.0, what="条件"):
    deadline = time.monotonic() + timeout_s
    while not pred():
        assert time.monotonic() < deadline, f"等不到：{what}"
        time.sleep(0.005)


# ---------------------------------------------------------------- 握手 / 心跳
def test_hello_first_then_heartbeats_every_50_ms(fake, conn):
    info = conn.connect()
    assert info == {"name": "fake"} and conn.connected
    hello = fake.got[0]
    assert hello["type"] == "hello" and hello["client"] == "test-client" and hello["pid"] == os.getpid()
    assert hello["v"] == CONTRACT_VERSION
    t0 = time.monotonic()
    time.sleep(0.5)
    n = sum(1 for m in fake.got if m["type"] == "heartbeat")
    # 50 ms 一次：0.5 s 里约 10 次（这台机器调度抖，下限放宽）
    assert n >= 6, (n, time.monotonic() - t0)


def test_a_core_speaking_another_version_is_refused(tmp_path):
    f = FakeCore(tmp_path, version="1.0")
    try:
        c = CoreConnection(f.dir, client="t")
        with pytest.raises(CoreError) as e:
            c.connect()
        assert e.value.kind == "version" and not c.connected
        c.close()
    finally:
        f.close()


def test_no_core_means_unavailable_and_retries_are_rate_limited(tmp_path):
    c = CoreConnection(str(tmp_path), client="t")
    with pytest.raises(CoreUnavailable):
        c.connect()
    with pytest.raises(CoreUnavailable):
        c.request("safety")  # within RETRY_S: the last failure, no new attempt
    c.close()


# ---------------------------------------------------------------- 租约
def test_lease_goal_stream_and_rejections(fake, conn):
    fake.handler = _lease_handler([
        Ack(seq=1, verdict=Verdict.ACCEPTED),
        Ack(seq=2, verdict=Verdict.REJECTED, reason=Reason.BUSY, detail="在拖拽"),
        Ack(seq=3, verdict=Verdict.REJECTED, reason=Reason.STALE_EPOCH, detail="已撤销：急停"),
    ])
    client = MotionClient(conn)
    lease = client.lease([R], "task:t")
    assert lease.lease == LEASE and lease.owner == "task:t#1"
    assert lease.goal(R, [[0.1] * 7]).ok
    goal = fake.got[-1]
    assert goal["type"] == "goal" and goal["request"]["lease_id"] == "L1" and goal["request"]["seq"] == 1
    with pytest.raises(MotionRejected) as e:
        lease.stream(R, [0.1] * 7, valid_for_ms=100)
    assert e.value.reason is Reason.BUSY and not isinstance(e.value, LeaseRevoked)
    assert fake.got[-1]["setpoint"]["valid_for_ms"] == 100
    assert not lease.cancelled.is_set()
    with pytest.raises(LeaseRevoked):
        lease.goal(R, [[0.1] * 7])
    assert lease.cancelled.is_set()
    with pytest.raises(LeaseRevoked):
        lease.check()


def test_invalid_commands_are_refused_locally(fake, conn):
    fake.handler = _lease_handler([])
    lease = MotionClient(conn).lease([R], "task:t")
    n = len(fake.got)
    with pytest.raises(MotionRejected) as e:
        lease.goal(R, [])
    assert e.value.reason is Reason.INVALID
    with pytest.raises(MotionRejected):
        lease.stream(R, [float("nan")])
    assert len([m for m in fake.got[n:] if m["type"] != "heartbeat"]) == 0, "坏指令不发出去"


def test_lease_event_from_the_core_sets_cancelled(fake, conn):
    fake.handler = _lease_handler([])
    lease = MotionClient(conn).lease([R], "task:t")
    fake.push("lease_event", lease_id="L1", event="revoked", reason="急停")
    assert lease.cancelled.wait(1.0)
    assert lease.revoked == "急停"
    with pytest.raises(LeaseRevoked, match="急停"):
        lease.check()


def test_release_and_cancel_tell_the_core(fake, conn):
    fake.handler = _lease_handler([])
    with MotionClient(conn).lease([R], "task:t") as lease:
        lease.cancel()
        assert lease.cancelled.is_set()
        with pytest.raises(MotionRejected):
            lease.check()
    sent = [m for m in fake.types() if m not in ("heartbeat", "hello")]
    assert sent == ["lease", "lease_cancel", "lease_return"]


def test_rejected_lease_raises_the_cores_reason(fake, conn):
    ack = Ack(seq=0, verdict=Verdict.REJECTED, reason=Reason.NOT_ENGAGED, detail="先接管")
    fake.handler = lambda msg: {"error": wire.error("rejected", "先接管", ack)}
    with pytest.raises(MotionRejected) as e:
        MotionClient(conn).lease([R], "task:t")
    assert e.value.reason is Reason.NOT_ENGAGED and str(e.value) == "先接管"


def test_other_errors_raise_core_error_with_the_kind(fake, conn):
    fake.handler = lambda msg: {"error": wire.error("permission", "未接管")}
    with pytest.raises(CoreError) as e:
        conn.call("engage")
    assert e.value.kind == "permission" and str(e.value) == "未接管"
    fake.handler = lambda msg: {"result": {"x": 1}}
    assert conn.call("page_status", arm="right") == {"x": 1}
    assert fake.got[-1]["op"] == "page_status" and fake.got[-1]["args"] == {"arm": "right"}


# ---------------------------------------------------------------- 断线 / 超时
def test_losing_the_core_fails_calls_and_revokes_leases(fake, conn):
    fake.handler = _lease_handler([])
    lease = MotionClient(conn).lease([R], "task:t")
    fake.handler = lambda msg: None  # 下一次调用永远不应答
    errors = []

    def call():
        try:
            conn.call("slow")
        except CoreUnavailable as exc:
            errors.append(exc)

    th = threading.Thread(target=call)
    th.start()
    time.sleep(0.1)
    fake.kick()
    th.join(2.0)
    assert errors, "等着的调用马上失败，不等超时"
    assert lease.cancelled.wait(1.0) and "closed" in (lease.revoked or "")
    with pytest.raises(LeaseRevoked):
        lease.goal(R, [[0.1] * 7])
    ack = lease.hold()
    assert ack.verdict is Verdict.REJECTED, "hold 不抛"
    assert not conn.connected


def test_unanswered_call_times_out_but_the_connection_stays(fake, conn):
    fake.handler = lambda msg: None if msg.get("op") == "slow" else {"result": 1}
    with pytest.raises(CoreUnavailable, match="no answer"):
        conn.call("slow", timeout_s=0.2)
    assert conn.connected and conn.call("fast") == 1


def test_reconnects_on_the_next_call_after_a_restart(tmp_path, monkeypatch):
    monkeypatch.setattr(connmod, "RETRY_S", 0.05)
    f = FakeCore(tmp_path)
    f.handler = lambda msg: {"result": "a"}
    c = CoreConnection(f.dir, client="t")
    try:
        assert c.call("x") == "a"
        f.close()
        wait_until(lambda: not c.connected, 1.0, "看到断线")
        for name in ("core.sock", "estop.sock"):
            os.unlink(os.path.join(f.dir, name))
        f = FakeCore(tmp_path)
        f.handler = lambda msg: {"result": "b"}
        time.sleep(0.1)
        assert c.call("x") == "b"
    finally:
        c.close()
        f.close()


# ---------------------------------------------------------------- 急停
def test_estop_goes_through_its_own_socket(fake, conn):
    fake.handler = lambda msg: None  # core.sock 堵死也不影响急停
    MotionClient(conn).estop("页面急停")
    assert fake.estops[-1]["type"] == "estop" and fake.estops[-1]["reason"] == "页面急停"
    assert fake.estops[-1]["client"] == "test-client"
    assert "estop" not in fake.types()


def test_estop_failure_is_reported(fake, conn):
    fake.estop_handler = lambda msg: wire.error("permission", "守护层没锁上")
    with pytest.raises(CoreError, match="守护层没锁上"):
        conn.estop("x")


def test_estop_without_a_core_raises(tmp_path):
    c = CoreConnection(str(tmp_path), client="t")
    with pytest.raises(CoreUnavailable, match="E-STOP not confirmed"):
        c.estop("x")
    c.close()


def test_estop_reconnects_after_the_core_restarted(tmp_path):
    f = FakeCore(tmp_path)
    c = CoreConnection(f.dir, client="t")
    try:
        c.estop("1")
        f.close()
        for name in ("core.sock", "estop.sock"):
            os.unlink(os.path.join(f.dir, name))
        f = FakeCore(tmp_path)
        c.estop("2")  # 旧连接已死：换新连接重试一次
        assert [m["reason"] for m in f.estops] == ["2"]
    finally:
        c.close()
        f.close()
