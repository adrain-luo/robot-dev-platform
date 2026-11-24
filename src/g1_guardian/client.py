"""Box-side client for the guardian, plus a small operator CLI.

    client = GuardianClient("192.168.123.164", client_id="refactor-node")
    epoch = client.engage()                       # guardian holds the measured pose
    client.setpoint({"right_arm": q7}, valid_for_ms=100)
    client.status()["groups"]["right_arm"]["q_meas"]
    client.estop(); client.reset(); client.engage()
    client.release()                              # ramped, then frames stop

A heartbeat thread keeps the link alive while engaged; if this process
dies, the guardian holds and latches ``link_lost``.

CLI:  python -m g1_guardian.client --host 192.168.123.164 status|engage|estop|reset|release|watch
"""

from __future__ import annotations

import argparse
import itertools
import json
import socket
import threading
import time
from typing import Any, Dict, Optional, Sequence, Union

from g1_guardian.protocol import DEFAULT_PORT, MAX_DATAGRAM, ProtocolError, decode, encode, message

GroupTarget = Union[Sequence[float], Dict[str, Any]]


class GuardianError(RuntimeError):
    pass


class GuardianRejected(GuardianError):
    def __init__(self, kind: str, reason: str) -> None:
        super().__init__(f"guardian rejected {kind}: {reason}")
        self.kind = kind
        self.reason = reason


class GuardianUnavailable(GuardianError):
    pass


class GuardianClient:
    def __init__(self, host: str, port: int = DEFAULT_PORT, *, client_id: str,
                 heartbeat_hz: float = 10.0, bind_host: str = "0.0.0.0") -> None:
        self.addr = (host, int(port))
        self.client_id = client_id
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind_host, 0))
        self.sock.settimeout(0.05)
        self._seq = itertools.count(int(time.time() * 1000) % 1_000_000_000)
        self._send_lock = threading.Lock()
        self._cond = threading.Condition()
        self._status: Optional[Dict[str, Any]] = None
        self._status_rx = 0.0
        self._acks: Dict[int, Dict[str, Any]] = {}
        self.epoch: Optional[int] = None
        self._closed = threading.Event()
        self._heartbeat = threading.Event()
        self._hb_period = 1.0 / heartbeat_hz
        self._rx = threading.Thread(target=self._rx_loop, name="guardian_rx", daemon=True)
        self._rx.start()
        self._hb = threading.Thread(target=self._hb_loop, name="guardian_heartbeat", daemon=True)
        self._hb.start()

    # ------------------------------------------------------------- transport
    def _send(self, kind: str, **fields: Any) -> int:
        seq = next(self._seq)
        data = encode(message(kind, self.client_id, seq, **fields))
        with self._send_lock:
            self.sock.sendto(data, self.addr)
        return seq

    def _rx_loop(self) -> None:
        while not self._closed.is_set():
            try:
                data, _ = self.sock.recvfrom(MAX_DATAGRAM + 1)
            except socket.timeout:
                continue
            except OSError:
                if self._closed.is_set():
                    return
                time.sleep(0.01)
                continue
            try:
                st = decode(data)
            except ProtocolError:
                continue
            if st.get("type") != "status":
                continue
            with self._cond:
                # Keep the newest status as "latest" even if datagrams arrive out of order.
                cur = self._status
                if cur is None or not isinstance(st.get("t"), (int, float)) or st["t"] >= cur.get("t", -1e300):
                    self._status = st
                    self._status_rx = time.monotonic()
                ack = st.get("ack")
                if isinstance(ack, dict) and isinstance(ack.get("seq"), int):
                    self._acks[ack["seq"]] = st
                    if len(self._acks) > 256:
                        for k in sorted(self._acks)[:128]:
                            self._acks.pop(k, None)
                self._cond.notify_all()

    def _hb_loop(self) -> None:
        while not self._closed.wait(self._hb_period):
            if self._heartbeat.is_set():
                try:
                    self._send("heartbeat")
                except OSError:
                    pass

    def _command(self, kind: str, timeout: float) -> Dict[str, Any]:
        seq = self._send(kind)
        deadline = time.monotonic() + timeout
        with self._cond:
            while seq not in self._acks:
                left = deadline - time.monotonic()
                if left <= 0:
                    raise GuardianUnavailable(f"no answer to {kind} from guardian at {self.addr[0]}:{self.addr[1]}")
                self._cond.wait(left)
            st = self._acks.pop(seq)
        ack = st["ack"]
        if not ack.get("ok"):
            raise GuardianRejected(kind, str(ack.get("reason")))
        return st

    # ------------------------------------------------------------------- API
    def engage(self, timeout: float = 5.0) -> int:
        """Take control (or re-read the epoch after a reset). Returns the epoch."""
        st = self._command("engage", timeout)
        self.epoch = int(st["epoch"])
        self._heartbeat.set()
        return self.epoch

    def setpoint(self, groups: Dict[str, GroupTarget], *, valid_for_ms: int = 100) -> int:
        """Send targets; fire-and-forget. Groups not listed hold where they are."""
        if self.epoch is None:
            raise GuardianError("engage() first")
        body: Dict[str, Any] = {}
        for name, value in groups.items():
            if isinstance(value, dict):
                entry = {"q": [float(x) for x in value["q"]]}
                if value.get("tau_ff") is not None:
                    entry["tau_ff"] = [float(x) for x in value["tau_ff"]]
                if value.get("kp_scale") is not None:
                    entry["kp_scale"] = float(value["kp_scale"])
            else:
                entry = {"q": [float(x) for x in value]}
            body[name] = entry
        return self._send("setpoint", epoch=self.epoch, valid_for_ms=int(valid_for_ms), groups=body)

    def estop(self, timeout: float = 1.0) -> Dict[str, Any]:
        return self._command("estop", timeout)

    def reset(self, timeout: float = 2.0) -> Dict[str, Any]:
        return self._command("reset", timeout)

    def release(self, timeout: float = 2.0) -> Dict[str, Any]:
        st = self._command("release", timeout)
        self._heartbeat.clear()
        self.epoch = None
        return st

    def poke(self) -> None:
        """Ask for status updates without waiting for an answer (a plain heartbeat).

        The guardian streams status to addresses heard from in the last few
        seconds; a read-only viewer pokes now and then to keep receiving it."""
        try:
            self._send("heartbeat")
        except OSError:
            pass

    def stop_heartbeat(self) -> None:
        """Stop the keep-alive: if we still own the guardian it latches link_lost and holds."""
        self._heartbeat.clear()
        self.epoch = None

    def status_age(self) -> float:
        with self._cond:
            return float("inf") if self._status is None else time.monotonic() - self._status_rx

    def latest_status(self) -> Optional[Dict[str, Any]]:
        with self._cond:
            return self._status

    def ping(self, timeout: float = 1.0) -> Dict[str, Any]:
        """Ask for a status without changing anything (a heartbeat with an ack)."""
        return self._command("heartbeat", timeout)

    def status(self, max_age: Optional[float] = None) -> Dict[str, Any]:
        with self._cond:
            st, rx = self._status, self._status_rx
        if st is None:
            raise GuardianUnavailable("no status from guardian yet")
        if max_age is not None and time.monotonic() - rx > max_age:
            raise GuardianUnavailable(f"guardian status is {time.monotonic() - rx:.3f}s old")
        return st

    def wait_for(self, predicate, timeout: float) -> Dict[str, Any]:
        deadline = time.monotonic() + timeout
        with self._cond:
            while True:
                if self._status is not None and predicate(self._status):
                    return self._status
                left = deadline - time.monotonic()
                if left <= 0:
                    raise GuardianUnavailable("timed out waiting for guardian state")
                self._cond.wait(left)

    def close(self) -> None:
        self._heartbeat.clear()
        self._closed.set()
        try:
            self.sock.close()
        except OSError:
            pass


def main(argv: Optional[list] = None) -> int:
    p = argparse.ArgumentParser(description="Talk to the g1_guardian service.")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--client-id", default="operator-cli")
    p.add_argument("command", choices=["status", "engage", "estop", "reset", "release", "watch"])
    args = p.parse_args(argv)
    c = GuardianClient(args.host, args.port, client_id=args.client_id)
    try:
        if args.command == "watch":
            c.ping()
            while True:
                st = c.status()
                loop = st.get("loop") or {}
                print(f"{st['state']:>9} latched={st['latched']} epoch={st['epoch']} owner={st['owner']} "
                      f"lowstate={st['lowstate_age_ms']}ms p99={loop.get('p99_ms')}ms max={loop.get('max_ms')}ms "
                      f"last_reject={(st.get('last_reject') or {}).get('reason')}", flush=True)
                c._send("heartbeat")
                time.sleep(0.5)
        st = {"status": c.ping, "engage": lambda: c._command("engage", 5.0), "estop": c.estop,
              "reset": c.reset, "release": c.release}[args.command]()
        print(json.dumps(st, ensure_ascii=False, indent=2))
        return 0
    except GuardianError as exc:
        print(f"error: {exc}")
        return 1
    except KeyboardInterrupt:
        return 0
    finally:
        c.close()


if __name__ == "__main__":
    raise SystemExit(main())
