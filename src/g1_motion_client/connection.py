"""One client connection to the motion core: request / reply, heartbeats, lease events, E-STOP.

Threads: a reader (replies and lease events) and a heartbeat sender, both
daemons. Calls block the calling thread until the reply arrives, so commands
from one thread reach the core in order; calls from different threads may
run concurrently in the core, as they did in-process.

Nothing here reconnects behind the caller's back while leases are held: a
lost connection fails every pending call, tells every lease listener, and
the next call tries to connect again. Leases are never re-acquired here;
that is the task's decision.
"""

from __future__ import annotations

import itertools
import os
import socket
import threading
import time
from typing import Any, Callable, Dict, List, Optional

from g1_contracts import wire
from g1_contracts.motion import Ack, rejection

CORE_SOCKET = "core.sock"
ESTOP_SOCKET = "estop.sock"
DEFAULT_SOCKET_DIR = "/run/g1-motion"
SOCKET_DIR_ENV = "G1_MOTION_DIR"
CALL_TIMEOUT_S = 10.0
CONNECT_TIMEOUT_S = 2.0
ESTOP_TIMEOUT_S = 3.0      # the core latches at once; with the guardian backend it then waits up to 1 s for the guardian
SEND_TIMEOUT_S = 0.5
RETRY_S = 0.5              # at most one connect attempt per this period


def socket_dir(path: Optional[str] = None) -> str:
    return path or os.environ.get(SOCKET_DIR_ENV) or DEFAULT_SOCKET_DIR


class CoreUnavailable(ConnectionError):
    """The core cannot be reached, the connection was lost, or it did not answer in time."""


class CoreError(RuntimeError):
    """The core refused a request for a reason other than a contract rejection."""

    def __init__(self, kind: str, detail: str) -> None:
        self.kind = kind
        self.detail = detail
        super().__init__(detail or kind)


class _Pending:
    __slots__ = ("event", "reply")

    def __init__(self) -> None:
        self.event = threading.Event()
        self.reply: Optional[Dict[str, Any]] = None


def _close(sock: socket.socket) -> None:
    """Shut down first: a plain close() while the reader thread is blocked in recv() does not reach the
    peer until that recv returns, so the core would see the disconnect up to a timeout late."""
    try:
        sock.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass
    try:
        sock.close()
    except OSError:
        pass


def raise_for_error(err: Dict[str, Any]) -> None:
    kind = str(err.get("kind") or "internal")
    detail = str(err.get("detail") or "")
    if kind == "rejected" and isinstance(err.get("ack"), dict):
        raise rejection(wire.from_wire(Ack, err["ack"]))
    raise CoreError(kind, detail)


class CoreConnection:
    def __init__(self, directory: Optional[str] = None, *, client: str,
                 connect_timeout_s: float = CONNECT_TIMEOUT_S, call_timeout_s: float = CALL_TIMEOUT_S,
                 heartbeat_ms: int = wire.HEARTBEAT_PERIOD_MS) -> None:
        if not isinstance(client, str) or not client.strip():
            raise ValueError("client must be a non-empty string")
        self.directory = socket_dir(directory)
        self.client = client
        self._connect_timeout = float(connect_timeout_s)
        self._call_timeout = float(call_timeout_s)
        self._hb_period = heartbeat_ms / 1000.0
        self._seq = itertools.count(1)
        self._lock = threading.Lock()          # connection state
        self._send_lock = threading.Lock()
        self._sock: Optional[socket.socket] = None
        self._gen = 0                          # bumped per connection, so a stale reader cannot clobber a new one
        self._pending: Dict[int, _Pending] = {}
        self._last_try = -1e9
        self._last_error: Optional[str] = None
        self.core_info: Dict[str, Any] = {}
        self._lease_listeners: Dict[str, Callable[[str, str, str], None]] = {}
        self._disconnect_listeners: List[Callable[[str], None]] = []
        self._estop_sock: Optional[socket.socket] = None
        self._estop_lock = threading.Lock()
        self._closed = threading.Event()
        self._hb = threading.Thread(target=self._hb_loop, name="motion-client-heartbeat", daemon=True)
        self._hb.start()

    # ------------------------------------------------------------ state
    @property
    def connected(self) -> bool:
        return self._sock is not None

    @property
    def last_error(self) -> Optional[str]:
        return self._last_error

    def add_disconnect_listener(self, fn: Callable[[str], None]) -> None:
        self._disconnect_listeners.append(fn)

    def watch_lease(self, lease_id: str, fn: Callable[[str, str, str], None]) -> None:
        with self._lock:
            self._lease_listeners[lease_id] = fn

    def forget_lease(self, lease_id: str) -> None:
        with self._lock:
            self._lease_listeners.pop(lease_id, None)

    # ------------------------------------------------------------ connect
    def connect(self) -> Dict[str, Any]:
        """Connect and say hello (no-op when connected). Returns the core's info."""
        with self._lock:
            if self._sock is not None:
                return self.core_info
            if self._closed.is_set():
                raise CoreUnavailable("connection closed")
            self._last_try = time.monotonic()
            path = os.path.join(self.directory, CORE_SOCKET)
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            sock.settimeout(self._connect_timeout)
            try:
                sock.connect(path)
                sock.send(wire.encode(wire.message("hello", 0, client=self.client, pid=os.getpid())))
                reply = wire.decode(sock.recv(wire.MAX_MESSAGE_BYTES + 1), wire.CORE_TYPES)
            except OSError as exc:
                sock.close()
                self._last_error = f"cannot connect to the motion core at {path}: {exc}"
                raise CoreUnavailable(self._last_error) from None
            except wire.WireError as exc:  # e.g. the core speaks another interface version
                sock.close()
                self._last_error = f"motion core at {path}: {exc.detail}"
                raise CoreError(exc.kind, self._last_error) from None
            if not reply.get("ok"):
                sock.close()
                err = reply.get("error") or {}
                self._last_error = str(err.get("detail") or "hello refused")
                raise CoreError(str(err.get("kind") or "protocol"), self._last_error)
            sock.settimeout(SEND_TIMEOUT_S)
            self._gen += 1
            gen = self._gen
            self._sock = sock
            self.core_info = dict(reply.get("core") or {})
            self._last_error = None
        threading.Thread(target=self._rx_loop, args=(sock, gen), name="motion-client-rx", daemon=True).start()
        return self.core_info

    def ensure(self) -> None:
        """Connect if not connected; at most one attempt per RETRY_S (raises the last failure meanwhile)."""
        if self._sock is not None:
            return
        if time.monotonic() - self._last_try < RETRY_S and self._last_error:
            raise CoreUnavailable(self._last_error)
        self.connect()

    # ------------------------------------------------------------ requests
    def request(self, kind: str, *, timeout_s: Optional[float] = None, **fields: Any) -> Dict[str, Any]:
        """Send a request and wait for its reply. Raises ``CoreUnavailable``, ``CoreError`` or ``MotionRejected``."""
        self.ensure()
        seq = next(self._seq)
        pending = _Pending()
        with self._lock:
            sock = self._sock
            if sock is None:
                raise CoreUnavailable(self._last_error or "not connected")
            self._pending[seq] = pending
        try:
            self._send_on(sock, wire.message(kind, seq, **fields))
            if not pending.event.wait(self._call_timeout if timeout_s is None else timeout_s):
                raise CoreUnavailable(f"no answer to {kind} from the motion core")
        finally:
            with self._lock:
                self._pending.pop(seq, None)
        reply = pending.reply
        if reply is None:
            raise CoreUnavailable(self._last_error or "connection to the motion core lost")
        if not reply.get("ok"):
            raise_for_error(reply.get("error") or {})
        return reply

    def call(self, op: str, *, timeout_s: Optional[float] = None, **args: Any) -> Any:
        """Platform extension call; returns its result."""
        return self.request("call", timeout_s=timeout_s, op=op, args=args).get("result")

    def send(self, kind: str, **fields: Any) -> None:
        with self._lock:
            sock = self._sock
        if sock is None:
            raise CoreUnavailable(self._last_error or "not connected")
        self._send_on(sock, wire.message(kind, next(self._seq), **fields))

    def _send_on(self, sock: socket.socket, msg: Dict[str, Any]) -> None:
        data = wire.encode(msg)
        try:
            with self._send_lock:
                sock.send(data)
        except OSError as exc:
            self._lost(sock, f"send to the motion core failed: {exc}")
            raise CoreUnavailable(self._last_error) from None

    # ------------------------------------------------------------ E-STOP
    def estop(self, reason: str = "") -> None:
        """Latch E-STOP through estop.sock (never queued behind task traffic). Raises if not confirmed."""
        with self._estop_lock:
            for attempt in (1, 2):  # a kept-open socket may have died with an old core: retry once on a fresh one
                sock = self._estop_sock
                fresh = sock is None
                try:
                    if sock is None:
                        sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
                        sock.settimeout(ESTOP_TIMEOUT_S)
                        sock.connect(os.path.join(self.directory, ESTOP_SOCKET))
                        self._estop_sock = sock
                    seq = next(self._seq)
                    sock.send(wire.encode(wire.message("estop", seq, client=self.client, reason=str(reason))))
                    while True:
                        data = sock.recv(wire.MAX_MESSAGE_BYTES + 1)
                        if not data:
                            raise OSError("estop socket closed")
                        reply = wire.decode(data, ("reply",))
                        if reply["seq"] == seq:
                            break
                except (OSError, wire.WireError) as exc:
                    self._drop_estop_socket()
                    if attempt == 1 and not fresh:
                        continue
                    raise CoreUnavailable(f"E-STOP not confirmed by the motion core: {exc}") from None
                if not reply.get("ok"):
                    raise_for_error(reply.get("error") or {})
                return

    def _drop_estop_socket(self) -> None:
        sock, self._estop_sock = self._estop_sock, None
        if sock is not None:
            _close(sock)

    # ------------------------------------------------------------ threads
    def _rx_loop(self, sock: socket.socket, gen: int) -> None:
        while True:
            try:
                data = sock.recv(wire.MAX_MESSAGE_BYTES + 1)
            except socket.timeout:
                if self._gen != gen or self._closed.is_set():
                    return
                continue
            except OSError as exc:
                self._lost(sock, f"connection to the motion core lost: {exc}")
                return
            if not data:
                self._lost(sock, "the motion core closed the connection")
                return
            try:
                msg = wire.decode(data, wire.CORE_TYPES)
            except wire.WireError as exc:
                self._lost(sock, f"bad message from the motion core: {exc}")
                return
            if msg["type"] == "reply":
                with self._lock:
                    pending = self._pending.get(msg["seq"])
                if pending is not None:
                    pending.reply = msg
                    pending.event.set()
            elif msg["type"] == "lease_event":
                lid = str(msg.get("lease_id") or "")
                with self._lock:
                    fn = self._lease_listeners.get(lid)
                if fn is not None:
                    try:
                        fn(lid, str(msg.get("event") or ""), str(msg.get("reason") or ""))
                    except Exception:  # noqa: BLE001  a listener bug must not stop the reader
                        pass

    def _hb_loop(self) -> None:
        while not self._closed.wait(self._hb_period):
            with self._lock:
                sock = self._sock
            if sock is None:
                continue
            try:
                self._send_on(sock, wire.message("heartbeat", 0))
            except CoreUnavailable:
                pass

    def _lost(self, sock: socket.socket, why: str) -> None:
        with self._lock:
            if self._sock is not sock:
                return
            self._sock = None
            self._last_error = why
            self._last_try = time.monotonic()
            pending = list(self._pending.values())
            listeners = list(self._lease_listeners.items())
            self._lease_listeners.clear()
        _close(sock)
        for p in pending:
            p.event.set()  # reply stays None: the caller raises CoreUnavailable
        for lid, fn in listeners:
            try:
                fn(lid, "revoked", why)
            except Exception:  # noqa: BLE001
                pass
        for fn in list(self._disconnect_listeners):
            try:
                fn(why)
            except Exception:  # noqa: BLE001
                pass

    def close(self) -> None:
        self._closed.set()
        with self._lock:
            sock = self._sock
        if sock is not None:
            self._lost(sock, "connection closed")
        with self._estop_lock:
            self._drop_estop_socket()
