"""Socket server of the motion core process.

Threads:
- accept (core.sock), one reader per connection, a bounded worker pool;
- one E-STOP thread for estop.sock (accept and serve), never shared with
  task traffic;
- a monitor (every ``MONITOR_PERIOD_S``): pushes lease events and enforces
  heartbeats.

The reader handles heartbeats itself and hands every other message to the
pool, so a slow call (engage, a detour plan) never delays heartbeats or other
callers. A connection may have at most ``MAX_IN_FLIGHT`` calls running; more
are answered ``busy`` at once.

A connection that closes, or sends nothing for ``HEARTBEAT_TIMEOUT_MS``, loses
its leases (``core.revoke_lease``) and the arms and waist it moved by hand
are held (``core.hold_manual``). The arms stop at their commanded angle and
frames keep going: the control loop is not touched by this module.

A second server refuses to start while another one answers on the same
socket path: one core per machine.

Nothing here writes to an actuator.
"""

from __future__ import annotations

import itertools
import os
import selectors
import socket
import threading
import time
import traceback
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Callable, Dict, List, Optional, Set

from g1_contracts import wire
from g1_contracts.motion import (
    Ack, GoalRequest, LeaseRequest, MotionRejected, Reason, Resource, Stamp, StreamSetpoint, Verdict, rejection,
)

CORE_SOCKET = "core.sock"
ESTOP_SOCKET = "estop.sock"
MONITOR_PERIOD_S = 0.01
HELLO_TIMEOUT_S = 2.0
MAX_IN_FLIGHT = 8
WORKERS = 16
SOCKET_MODE = 0o660


class ServerStartError(RuntimeError):
    pass


class _Conn:
    def __init__(self, sock: socket.socket, cid: int, now: float) -> None:
        self.sock = sock
        self.id = cid
        self.client = "?"
        self.pid: Optional[int] = None
        self.last_rx = now
        self.timed_out = False
        self.closed = False
        self.leases: Dict[str, Any] = {}     # lease_id -> core lease handle
        self.reported: Set[str] = set()
        self.manual: Set[str] = set()        # arms / "waist" this connection moved by hand
        self.in_flight = 0
        self.lock = threading.Lock()
        self.send_lock = threading.Lock()

    @property
    def name(self) -> str:
        return f"{self.client}（连接 {self.id}{'' if self.pid is None else f'，pid {self.pid}'}）"


class CallContext:
    """What a platform call sees about its caller."""

    def __init__(self, conn: _Conn) -> None:
        self._conn = conn

    @property
    def client(self) -> str:
        return self._conn.client

    def touch(self, *what: str) -> None:
        """Mark arms ("right" / "left") or "waist" as moved by hand from this connection (call before moving).

        Raises ``PermissionError`` when the connection is already gone, so nothing moves for a dead client."""
        with self._conn.lock:
            if self._conn.closed or self._conn.timed_out:
                raise PermissionError("连接已断开，不再执行它的指令")
            self._conn.manual.update(what)


def default_classify(exc: BaseException) -> str:
    if isinstance(exc, MotionRejected):
        return "rejected"
    if isinstance(exc, wire.WireError):
        return exc.kind
    if isinstance(exc, PermissionError):
        return "permission"
    if isinstance(exc, (ValueError, KeyError)):
        return "value"
    if isinstance(exc, TimeoutError):
        return "timeout"
    return "internal"


def _reject(seq: int, reason: Reason, detail: str) -> Ack:
    return Ack(seq=seq, verdict=Verdict.REJECTED, reason=reason, detail=detail)


class CoreServer:
    def __init__(
        self,
        core,
        directory: str,
        *,
        ops: Optional[Dict[str, Callable[[CallContext, Dict[str, Any]], Any]]] = None,
        info: Optional[Callable[[], Dict[str, Any]]] = None,
        classify: Optional[Callable[[BaseException], Optional[str]]] = None,
        heartbeat_timeout_ms: int = wire.HEARTBEAT_TIMEOUT_MS,
        max_in_flight: int = MAX_IN_FLIGHT,
        workers: int = WORKERS,
        log: Callable[[str], None] = print,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.core = core
        self.directory = str(directory)
        self.ops = dict(ops or {})
        self._info = info or (lambda: {})
        self._classify = classify
        self._hb_timeout = heartbeat_timeout_ms / 1000.0
        self._max_in_flight = int(max_in_flight)
        self._pool = ThreadPoolExecutor(max_workers=int(workers), thread_name_prefix="motion-core-call")
        self.log = log
        self._clock = clock
        self._ids = itertools.count(1)
        self._event_seq = itertools.count(1)
        self._conns: Dict[int, _Conn] = {}
        self._conns_lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []
        self._listener: Optional[socket.socket] = None
        self._estop_listener: Optional[socket.socket] = None
        self.stats = {"estops": 0, "heartbeat_timeouts": 0, "disconnects": 0, "busy": 0}

    # ------------------------------------------------------------ lifecycle
    @property
    def core_path(self) -> str:
        return os.path.join(self.directory, CORE_SOCKET)

    @property
    def estop_path(self) -> str:
        return os.path.join(self.directory, ESTOP_SOCKET)

    def start(self) -> "CoreServer":
        os.makedirs(self.directory, mode=0o770, exist_ok=True)
        self._listener = self._bind(self.core_path)
        try:
            self._estop_listener = self._bind(self.estop_path)
        except BaseException:
            self._listener.close()
            raise
        for target, name in ((self._accept_loop, "motion-core-accept"), (self._estop_loop, "motion-core-estop"),
                             (self._monitor_loop, "motion-core-monitor")):
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self._threads.append(t)
        self.log(f"[g1_motion_core] listening on {self.core_path} and {self.estop_path}")
        return self

    def _bind(self, path: str) -> socket.socket:
        if os.path.exists(path):
            probe = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            probe.settimeout(0.5)
            try:
                probe.connect(path)
            except OSError:
                os.unlink(path)  # stale socket file from a core that is gone
            else:
                raise ServerStartError(f"another motion core is already serving {path}: one core per machine")
            finally:
                probe.close()
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        sock.bind(path)
        os.chmod(path, SOCKET_MODE)
        sock.listen(16)
        sock.settimeout(0.2)
        return sock

    def stop(self) -> None:
        """Stop serving. Every connection's leases are revoked (arms hold); the core itself keeps running."""
        self._stop.set()
        for s in (self._listener, self._estop_listener):
            if s is not None:
                try:
                    s.close()
                except OSError:
                    pass
        with self._conns_lock:
            conns = list(self._conns.values())
        for c in conns:
            self._drop(c, "运控核心停止服务")
        for t in self._threads:
            t.join(2.0)
        self._pool.shutdown(wait=False, cancel_futures=True)
        for path in (self.core_path, self.estop_path):
            try:
                os.unlink(path)
            except OSError:
                pass

    # ------------------------------------------------------------ core.sock
    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            try:
                sock, _ = self._listener.accept()
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                time.sleep(0.05)
                continue
            conn = _Conn(sock, next(self._ids), self._clock())
            threading.Thread(target=self._serve, args=(conn,), name=f"motion-core-conn-{conn.id}", daemon=True).start()

    def _send(self, conn: _Conn, msg: Dict[str, Any]) -> bool:
        try:
            data = wire.encode(msg)
        except (wire.WireError, TypeError, ValueError) as exc:  # too large, or not JSON (a bug in an op)
            self.log(f"[g1_motion_core] cannot encode {msg.get('type')} for {conn.name}: {exc}")
            data = wire.encode(wire.message("reply", msg.get("seq", 0), ok=False,
                                            error=wire.error("internal", f"reply cannot be encoded: {exc}")))
        try:
            with conn.send_lock:
                conn.sock.send(data)
            return True
        except OSError:
            return False

    def _reply(self, conn: _Conn, seq: int, **fields: Any) -> None:
        self._send(conn, wire.message("reply", seq, ok=True, **fields))

    def _reply_error(self, conn: _Conn, seq: int, kind: str, detail: str, ack: Optional[Ack] = None) -> None:
        self._send(conn, wire.message("reply", seq, ok=False, error=wire.error(kind, detail, ack)))

    def _serve(self, conn: _Conn) -> None:
        sock = conn.sock
        sock.settimeout(HELLO_TIMEOUT_S)
        try:
            data = sock.recv(wire.MAX_MESSAGE_BYTES + 1)
            msg = wire.decode(data, ("hello",))
        except (OSError, wire.WireError) as exc:
            kind = exc.kind if isinstance(exc, wire.WireError) else "protocol"
            self._reply_error(conn, 0, kind, f"expected hello first: {exc}")
            sock.close()
            return
        conn.client = str(msg.get("client") or "?")[:120]
        pid = msg.get("pid")
        conn.pid = pid if isinstance(pid, int) and not isinstance(pid, bool) else None
        conn.last_rx = self._clock()
        with self._conns_lock:
            self._conns[conn.id] = conn
        info = dict(self._info())
        info["contract_version"] = wire.CONTRACT_VERSION
        info["heartbeat_timeout_ms"] = int(self._hb_timeout * 1000)
        self._reply(conn, msg["seq"], core=info)
        self.log(f"[g1_motion_core] connected: {conn.name}")
        sock.settimeout(0.2)
        while not self._stop.is_set() and not conn.closed:
            try:
                data = sock.recv(wire.MAX_MESSAGE_BYTES + 1)
            except socket.timeout:
                continue
            except OSError as exc:
                self._drop(conn, f"连接出错：{exc}")
                return
            if not data:
                self._drop(conn, "连接断开")
                return
            conn.last_rx = self._clock()
            conn.timed_out = False
            try:
                msg = wire.decode(data, wire.CLIENT_TYPES)
            except wire.WireError as exc:
                self._reply_error(conn, 0, exc.kind, exc.detail)
                self._drop(conn, f"消息不合协议：{exc}")
                return
            kind = msg["type"]
            if kind == "heartbeat":
                continue
            if kind == "hello":
                self._reply_error(conn, msg["seq"], "protocol", "hello was already sent")
                continue
            with conn.lock:
                if conn.in_flight >= self._max_in_flight:
                    busy = True
                else:
                    conn.in_flight += 1
                    busy = False
            if busy:
                self.stats["busy"] += 1
                self._reply_error(conn, msg["seq"], "busy", f"too many calls in flight on this connection (max {self._max_in_flight})")
                continue
            try:
                self._pool.submit(self._run, conn, msg)
            except RuntimeError:  # pool shut down
                with conn.lock:
                    conn.in_flight -= 1
                return

    def _run(self, conn: _Conn, msg: Dict[str, Any]) -> None:
        seq = msg["seq"]
        try:
            if conn.closed or conn.timed_out:
                # Queued before the connection died: do not act on it (its leases are already revoked).
                raise wire.WireError("protocol", "connection lost before this call ran")
            result = self._dispatch(conn, msg)
        except BaseException as exc:  # noqa: BLE001  every failure becomes an error reply
            kind = (self._classify(exc) if self._classify else None) or default_classify(exc)
            if kind == "internal":
                self.log(f"[g1_motion_core] internal error in {msg.get('type')}/{msg.get('op', '')}: "
                         f"{traceback.format_exc()}")
            ack = getattr(exc, "ack", None) if kind == "rejected" else None
            detail = str(exc) if not isinstance(exc, KeyError) else f"missing argument {exc}"
            self._reply_error(conn, seq, kind, detail, ack)
        else:
            self._reply(conn, seq, **(result or {}))
        finally:
            with conn.lock:
                conn.in_flight -= 1

    # ------------------------------------------------------------ dispatch
    def _own(self, conn: _Conn, lease_id: str):
        with conn.lock:
            return conn.leases.get(lease_id)

    def _dispatch(self, conn: _Conn, msg: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        kind = msg["type"]
        core = self.core
        if kind == "lease":
            req = wire.from_wire(LeaseRequest, msg.get("request"))
            if req.heartbeat_ms is not None:
                raise rejection(_reject(0, Reason.UNSUPPORTED,
                                        f"心跳按连接统一检查（{int(self._hb_timeout * 1000)} ms），租约里不设 heartbeat_ms"))
            handle = core.acquire(req)
            lid = handle.lease.lease_id
            with conn.lock:
                conn.leases[lid] = handle
                gone = conn.closed or conn.timed_out
            if gone:  # the connection died while the lease was being granted
                core.revoke_lease(lid, "连接在申请租约时断开")
                raise rejection(_reject(0, Reason.STALE_EPOCH, "连接已断开"))
            self.log(f"[g1_motion_core] lease {lid} → {conn.name}: {handle.owner}")
            return {"lease": wire.to_wire(handle.lease), "token": handle.owner}
        if kind in ("lease_return", "lease_cancel"):
            lid = str(msg.get("lease_id") or "")
            with conn.lock:
                handle = conn.leases.get(lid)
                if handle is not None and kind == "lease_return":
                    del conn.leases[lid]
                    conn.reported.discard(lid)
            if handle is not None:
                handle.release() if kind == "lease_return" else handle.cancel()
            return {}
        if kind == "goal":
            req = wire.from_wire(GoalRequest, msg.get("request"))
            if self._own(conn, req.lease_id) is None:
                return {"ack": wire.to_wire(self._not_ours(req.lease_id, req.seq))}
            return {"ack": wire.to_wire(core.submit_goal(req))}
        if kind == "stream":
            sp = wire.from_wire(StreamSetpoint, msg.get("setpoint"))
            if self._own(conn, sp.lease_id) is None:
                return {"ack": wire.to_wire(self._not_ours(sp.lease_id, sp.seq))}
            return {"ack": wire.to_wire(core.submit_stream(sp))}
        if kind == "hold":
            lid = str(msg.get("lease_id") or "")
            hold_seq = msg.get("hold_seq", 0)
            hold_seq = hold_seq if isinstance(hold_seq, int) and not isinstance(hold_seq, bool) and hold_seq >= 0 else 0
            resource = Resource(msg.get("resource"))
            if self._own(conn, lid) is None:
                return {"ack": wire.to_wire(self._not_ours(lid, hold_seq))}
            return {"ack": wire.to_wire(core.hold(lid, resource, hold_seq))}
        if kind == "state":
            return {"state": wire.to_wire(core.state(Resource(msg.get("resource"))))}
        if kind == "state_at":
            stamp = wire.from_wire(Stamp, msg.get("stamp"))
            return {"state": wire.to_wire(core.state_at(Resource(msg.get("resource")), stamp))}
        if kind == "safety":
            return {"safety": core.safety().value}
        if kind == "call":
            op = msg.get("op")
            fn = self.ops.get(op) if isinstance(op, str) else None
            if fn is None:
                raise wire.WireError("unsupported", f"no such call {op!r}")
            args = msg.get("args") or {}
            if not isinstance(args, dict):
                raise wire.WireError("protocol", "call args must be an object")
            return {"result": fn(CallContext(conn), args)}
        raise wire.WireError("unsupported", f"message type {kind!r}")

    def _not_ours(self, lease_id: str, seq: int) -> Ack:
        # A lease this connection does not hold: never touch it (ids are guessable).
        return _reject(seq, Reason.UNKNOWN_LEASE, "这个连接没有这个租约")

    # ------------------------------------------------------------ losing a client
    def _abandon(self, conn: _Conn, reason: str) -> None:
        """Revoke the connection's leases and hold what it moved by hand. Frames keep going."""
        with conn.lock:
            leases = list(conn.leases)
            manual = sorted(conn.manual)
            conn.manual.clear()
        for lid in leases:
            try:
                if self.core.revoke_lease(lid, reason):
                    self.log(f"[g1_motion_core] lease {lid} of {conn.name} revoked: {reason}")
            except Exception:  # noqa: BLE001
                self.log(f"[g1_motion_core] revoking {lid} failed: {traceback.format_exc()}")
        for what in manual:
            try:
                self.core.hold_manual(what)
            except Exception:  # noqa: BLE001
                self.log(f"[g1_motion_core] holding {what} failed: {traceback.format_exc()}")
        if manual:
            self.log(f"[g1_motion_core] {conn.name} gone ({reason}): held {', '.join(manual)}")

    def _drop(self, conn: _Conn, reason: str) -> None:
        with conn.lock:
            if conn.closed:
                return
            conn.closed = True
        with self._conns_lock:
            self._conns.pop(conn.id, None)
        self.stats["disconnects"] += 1
        self._abandon(conn, f"{conn.client} {reason}")
        try:
            conn.sock.shutdown(socket.SHUT_RDWR)  # wakes this connection's reader at once
        except OSError:
            pass
        try:
            conn.sock.close()
        except OSError:
            pass
        self.log(f"[g1_motion_core] disconnected: {conn.name}: {reason}")

    # ------------------------------------------------------------ monitor
    def _monitor_loop(self) -> None:
        while not self._stop.wait(MONITOR_PERIOD_S):
            try:
                self._monitor_once()
            except Exception:  # noqa: BLE001  the monitor must keep running
                self.log(f"[g1_motion_core] monitor: {traceback.format_exc()}")

    def _monitor_once(self) -> None:
        now = self._clock()
        with self._conns_lock:
            conns = list(self._conns.values())
        for conn in conns:
            with conn.lock:
                busy_with = bool(conn.leases or conn.manual)
                late = (not conn.timed_out) and now - conn.last_rx > self._hb_timeout and busy_with
                if late:
                    conn.timed_out = True
            if late:
                self.stats["heartbeat_timeouts"] += 1
                ms = (now - conn.last_rx) * 1000
                self._abandon(conn, f"{conn.client} 心跳超时（{ms:.0f} ms 没有消息，上限 {self._hb_timeout * 1000:.0f} ms）")
            self._push_lease_events(conn)

    def _push_lease_events(self, conn: _Conn) -> None:
        events = []
        with conn.lock:
            for lid, handle in list(conn.leases.items()):
                if lid in conn.reported or not handle.cancelled.is_set():
                    continue
                if handle.revoked:
                    events.append((lid, "revoked", str(handle.revoked)))
                elif handle.returned:
                    events.append((lid, "returned", ""))
                else:
                    continue  # cancelled by its holder: the client already knows
                conn.reported.add(lid)
                del conn.leases[lid]
        for lid, event, reason in events:
            self._send(conn, wire.message("lease_event", next(self._event_seq), lease_id=lid, event=event,
                                          reason=reason))

    # ------------------------------------------------------------ estop.sock
    def _estop_loop(self) -> None:
        sel = selectors.DefaultSelector()
        sel.register(self._estop_listener, selectors.EVENT_READ, None)
        try:
            while not self._stop.is_set():
                for key, _ in sel.select(timeout=0.2):
                    if key.data is None:
                        try:
                            sock, _ = self._estop_listener.accept()
                        except OSError:
                            continue
                        sock.setblocking(True)
                        sock.settimeout(1.0)
                        sel.register(sock, selectors.EVENT_READ, "conn")
                        continue
                    sock = key.fileobj
                    try:
                        data = sock.recv(wire.MAX_MESSAGE_BYTES + 1)
                    except OSError:
                        data = b""
                    if not data:
                        sel.unregister(sock)
                        sock.close()
                        continue
                    self._handle_estop(sock, data)
        except Exception:  # noqa: BLE001
            if not self._stop.is_set():
                self.log(f"[g1_motion_core] estop thread ended: {traceback.format_exc()}")
        finally:
            sel.close()

    def _handle_estop(self, sock: socket.socket, data: bytes) -> None:
        try:
            msg = wire.decode(data, wire.ESTOP_TYPES)
        except wire.WireError as exc:
            self._estop_reply(sock, 0, ok=False, error=wire.error(exc.kind, exc.detail))
            return
        client = str(msg.get("client") or "?")[:120]
        reason = str(msg.get("reason") or "")[:200]
        self.stats["estops"] += 1
        try:
            self.core.estop(reason)
        except BaseException as exc:  # noqa: BLE001  the core latches before anything here can fail
            kind = (self._classify(exc) if self._classify else None) or default_classify(exc)
            self.log(f"[g1_motion_core] E-STOP from {client} latched, but: {exc}")
            self._estop_reply(sock, msg["seq"], ok=False, error=wire.error(kind, str(exc)))
            return
        self.log(f"[g1_motion_core] E-STOP from {client}: {reason}")
        self._estop_reply(sock, msg["seq"], ok=True)

    @staticmethod
    def _estop_reply(sock: socket.socket, seq: int, **fields: Any) -> None:
        try:
            sock.send(wire.encode(wire.message("reply", seq, **fields)))
        except OSError:
            pass

    # ------------------------------------------------------------ status
    def clients(self) -> List[Dict[str, Any]]:
        now = self._clock()
        with self._conns_lock:
            conns = list(self._conns.values())
        out = []
        for c in conns:
            with c.lock:
                out.append({"id": c.id, "client": c.client, "pid": c.pid, "leases": sorted(c.leases),
                            "manual": sorted(c.manual), "silent_ms": round((now - c.last_rx) * 1000),
                            "timed_out": c.timed_out, "in_flight": c.in_flight})
        return out
