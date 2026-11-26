"""The guardian service: one thread, one fixed-rate loop.

Each tick: drain UDP messages -> core.step() -> write the frame -> record the
blackbox -> send status when due -> sleep until the next tick. Nothing in
the loop blocks on the network; a failed write is counted and retried on
the next tick, never ends the loop.

Run on the robot's own computer (the G1D body), e.g.

    python -m g1_guardian.service --config g1d --bind 192.168.123.164 \\
        --net eth0 --rt-priority 80 --cpu 3 --blackbox-dir /var/lib/g1-guardian
"""

from __future__ import annotations

import argparse
import gc
import math
import os
import signal
import socket
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Callable, Deque, Dict, Optional, Protocol, Tuple

from g1_guardian import __version__
from g1_guardian.blackbox import Blackbox
from g1_guardian.config import GuardianConfig, builtin_config_path, load_config
from g1_guardian.core import ALL_COMMANDS, Frame, GuardianCore, LowStateView
from g1_guardian.protocol import DEFAULT_PORT, MAX_DATAGRAM, ProtocolError, decode, encode

SUBSCRIBER_TTL_S = 5.0
REJECT_ACK_MIN_INTERVAL_S = 0.05
MAX_MESSAGES_PER_TICK = 256


class LowStateSource(Protocol):
    def latest(self) -> Optional[LowStateView]: ...


class Output(Protocol):
    prepared: bool
    foreign_seen_at: float

    def prepare(self) -> None: ...

    def write(self, frame: Frame) -> None: ...


class LoopStats:
    def __init__(self, period: float, window: int = 2000) -> None:
        self.period = period
        self.intervals: Deque[float] = deque(maxlen=window)
        self.max_interval = 0.0
        self.overruns = 0
        self.last: Optional[float] = None
        self.write_failures = 0
        self.last_write_error = ""
        self._cache: Optional[Dict[str, Any]] = None
        self._cache_t = -math.inf

    def tick(self, now: float) -> None:
        if self.last is not None:
            dt = now - self.last
            self.intervals.append(dt)
            if dt > self.max_interval:
                self.max_interval = dt
            if dt > 2.0 * self.period:
                self.overruns += 1
        self.last = now

    def summary(self, now: Optional[float] = None) -> Dict[str, Any]:
        """Percentiles over the last ticks; recomputed at most every 100 ms."""
        if now is not None and self._cache is not None and now - self._cache_t < 0.1:
            return dict(self._cache, write_failures=self.write_failures, last_write_error=self.last_write_error)
        xs = sorted(self.intervals)
        p = (lambda q: round(xs[min(len(xs) - 1, int(q * len(xs)))] * 1000.0, 3)) if xs else (lambda q: None)
        out = {
            "period_ms": round(self.period * 1000.0, 3),
            "p50_ms": p(0.50),
            "p99_ms": p(0.99),
            "max_ms": round(self.max_interval * 1000.0, 3),
            "overruns": self.overruns,
            "write_failures": self.write_failures,
            "last_write_error": self.last_write_error,
        }
        if now is not None:
            self._cache, self._cache_t = out, now
        return out


class GuardianService:
    def __init__(
        self,
        cfg: GuardianConfig,
        output: Output,
        lowstate: LowStateSource,
        *,
        bind: Tuple[str, int] = ("127.0.0.1", DEFAULT_PORT),
        blackbox_dir: Optional[Path] = None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        log: Callable[[str], None] = lambda s: print(s, flush=True),
    ) -> None:
        self.cfg = cfg
        self.core = GuardianCore(cfg)
        self.output = output
        self.lowstate = lowstate
        self.clock = clock
        self.sleep = sleep
        self.log = log
        self.stats = LoopStats(cfg.period_s)
        self.blackbox = Blackbox(blackbox_dir, seconds=cfg.blackbox_seconds, rate_hz=cfg.rate_hz)
        self.blackbox.fields = ["t", "state", "scale", "lowstate_age_ms"] + [
            f"{kind}.{g.name}" for kind in ("q_cmd", "q_meas") for g in cfg.groups
        ]
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(bind)
        self.sock.setblocking(False)
        self.address = self.sock.getsockname()
        self.subscribers: Dict[Tuple[str, int], float] = {}
        self.stop_event = threading.Event()
        self.shutdown_requested = threading.Event()
        self._last_status = -math.inf
        self._last_reject_ack = -math.inf
        self.ticks = 0

    # ---------------------------------------------------------------- messages
    def _send(self, addr: Tuple[str, int], payload: Dict[str, Any]) -> None:
        try:
            self.sock.sendto(encode(payload), addr)
        except (OSError, ProtocolError):
            pass

    def _drain(self, now: float) -> None:
        for _ in range(MAX_MESSAGES_PER_TICK):
            try:
                data, addr = self.sock.recvfrom(MAX_DATAGRAM + 1)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                return
            source = addr[0]
            if self.cfg.access.role(source) is None:
                # Not in the config's access lists: no answer, no status, not even decoded.
                self.core.refuse_source(now, source)
                continue
            self.subscribers[addr] = now + SUBSCRIBER_TTL_S
            try:
                msg = decode(data)
            except ProtocolError as exc:
                self.core._reject(now, {}, f"bad_message:{exc}"[:80])
                continue
            kind = msg.get("type")
            reason = self.core.permitted(source, kind) if kind in ALL_COMMANDS else None
            if reason is not None:
                # Checked before the engage precheck: Output.prepare() leaves Unitree's motion mode.
                reason = self.core._reject(now, msg, reason)
            elif kind == "engage" and not self.output.prepared:
                # Feed the newest lowstate first: the precheck must see the robot as it is now.
                ls = self.lowstate.latest()
                if ls is not None:
                    self.core.observe(now, ls)
                pre = self.core.precheck_engage(now) if self.core.state == "idle" else None
                if pre:
                    reason = self.core._reject(now, msg, pre)
                else:
                    try:
                        self.output.prepare()
                    except Exception as exc:  # noqa: BLE001
                        reason = self.core._reject(now, msg, f"prepare_failed:{exc}"[:120])
            if reason is None:
                reason = self.core.handle(msg, now, source=source)
            if kind != "setpoint" or (reason is not None and now - self._last_reject_ack >= REJECT_ACK_MIN_INTERVAL_S):
                if kind == "setpoint":
                    self._last_reject_ack = now
                status = self._status(now)
                status["ack"] = {"type": kind, "seq": msg.get("seq"), "ok": reason is None, "reason": reason}
                self._send(addr, status)

    def _status(self, now: float) -> Dict[str, Any]:
        status = self.core.status(now)
        status["loop"] = self.stats.summary(now)
        status["foreign_writer_frames"] = getattr(self.output, "foreign_count", 0)
        return status

    def _broadcast_status(self, now: float) -> None:
        if now - self._last_status < 1.0 / self.cfg.status_hz:
            return
        self._last_status = now
        expired = [a for a, t in self.subscribers.items() if t < now]
        for a in expired:
            del self.subscribers[a]
        if not self.subscribers:
            return
        status = self._status(now)
        for a in list(self.subscribers):
            self._send(a, status)

    # -------------------------------------------------------------------- tick
    def tick(self) -> Optional[Frame]:
        now = self.clock()
        self.stats.tick(now)
        self._drain(now)
        foreign = getattr(self.output, "foreign_seen_at", -math.inf)
        if foreign > -math.inf:
            self.core.note_foreign_writer(foreign)
        ls = self.lowstate.latest()
        frame = self.core.step(now, ls)
        if frame is not None:
            try:
                self.output.write(frame)
            except Exception as exc:  # noqa: BLE001  never end the loop on a write error
                self.stats.write_failures += 1
                self.stats.last_write_error = f"{type(exc).__name__}: {exc}"[:200]
                if self.stats.write_failures in (1, 10, 100) or self.stats.write_failures % 1000 == 0:
                    self.log(f"[g1_guardian][WRITE_FAILED] count={self.stats.write_failures} {self.stats.last_write_error}")
            self._record(now)
        for t, what in self.core.events:
            self.log(f"[g1_guardian][event] {what}")
            if what.startswith("latched:") or what in ("released", "reset"):
                self.blackbox.dump(what, {"t": t, "robot": self.cfg.robot, "status": self.core.status(now)})
        self.core.events.clear()
        self._broadcast_status(now)
        self.ticks += 1
        return frame

    def _record(self, now: float) -> None:
        core = self.core
        age = None if core.lowstate is None else round((now - core.lowstate.stamp) * 1000.0, 2)
        row = [round(now, 4), core.state, round(core.scale, 4), age]
        for g in self.cfg.groups:
            row.append([round(x, 5) for x in core.q_cmd.get(g.name, [])])
        for g in self.cfg.groups:
            row.append([] if core.last_meas is None else [round(float(core.last_meas[m]), 5) for m in g.motors])
        self.blackbox.record(tuple(row))

    def run(self, max_ticks: Optional[int] = None) -> None:
        period = self.cfg.period_s
        next_t = self.clock()
        while not self.stop_event.is_set():
            if self.shutdown_requested.is_set():
                # Never just stop while engaged (G1D arm would drop): release
                # with the ramp first, exit once the guardian is idle.
                if self.core.state == "idle":
                    return
                if self.core.state != "releasing":
                    self.log("[g1_guardian] shutdown requested while engaged: releasing with ramp first")
                    self.core.handle({"v": 1, "type": "release", "client": "guardian-shutdown", "seq": 0}, self.clock())
            self.tick()
            if max_ticks is not None and self.ticks >= max_ticks:
                return
            next_t += period
            delay = next_t - self.clock()
            if delay > 0:
                self.sleep(delay)
            else:
                next_t = self.clock()

    def stop(self) -> None:
        """Stop the loop immediately (tests, or after a ramped shutdown)."""
        self.stop_event.set()

    def request_shutdown(self) -> None:
        """Graceful stop: ramped release if engaged, then exit the loop."""
        self.shutdown_requested.set()

    def close(self) -> None:
        self.sock.close()


def setup_realtime(priority: Optional[int], cpu: Optional[int], log: Callable[[str], None]) -> None:
    if cpu is not None:
        try:
            os.sched_setaffinity(0, {cpu})
            log(f"[g1_guardian] pinned to CPU {cpu}")
        except (AttributeError, OSError) as exc:
            log(f"[g1_guardian][WARN] cannot pin to CPU {cpu}: {exc}")
    if priority is not None:
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(priority))
            log(f"[g1_guardian] SCHED_FIFO priority {priority}")
        except (AttributeError, OSError, PermissionError) as exc:
            log(f"[g1_guardian][WARN] cannot set SCHED_FIFO {priority}: {exc}")


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="G1/G1D body-side execution guardian (sole lowcmd/arm_sdk writer).")
    p.add_argument("--config", default="g1d", help="builtin name (g1d, g1) or path to a JSON config")
    p.add_argument("--allow-draft-config", action="store_true", help="run a config whose status is 'draft'")
    p.add_argument("--bind", default="127.0.0.1", help="UDP address to listen on (the body's LAN address)")
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--net", "--network-interface", dest="net", default="eth0", help="DDS network interface")
    p.add_argument("--dds-domain", type=int, default=0)
    p.add_argument("--sdk-path", help="directory that contains unitree_sdk2py")
    p.add_argument("--rt-priority", type=int, help="SCHED_FIFO priority for the loop thread (needs CAP_SYS_NICE)")
    p.add_argument("--cpu", type=int, help="pin the process to this CPU")
    p.add_argument("--blackbox-dir", default="/var/lib/g1-guardian/blackbox")
    p.add_argument("--wait-lowstate-s", type=float, default=5.0)
    return p


def resolve_config(name: str) -> GuardianConfig:
    path = Path(name)
    if not path.suffix:
        path = builtin_config_path(name)
    return load_config(path)


def main(argv: Optional[list] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    log = lambda s: print(s, flush=True)  # noqa: E731
    cfg = resolve_config(args.config)
    if cfg.status == "draft" and not args.allow_draft_config:
        raise SystemExit(f"config {args.config!r} is a draft ({cfg.extra.get('note', '')}); pass --allow-draft-config")
    from g1_guardian.dds import DdsLowState, DdsOutput, add_sdk_path, init_channel

    add_sdk_path(args.sdk_path)
    init_channel(args.dds_domain, args.net)
    lowstate = DdsLowState(num_motors=cfg.num_motors)
    deadline = time.monotonic() + args.wait_lowstate_s
    while lowstate.latest() is None and time.monotonic() < deadline:
        time.sleep(0.02)
    if lowstate.latest() is None:
        raise SystemExit("no rt/lowstate received")
    output = DdsOutput(cfg)
    svc = GuardianService(cfg, output, lowstate, bind=(args.bind, args.port), blackbox_dir=Path(args.blackbox_dir))
    log(f"[g1_guardian] {__version__} robot={cfg.robot} channel={cfg.channel} status={cfg.status} "
        f"listening udp://{svc.address[0]}:{svc.address[1]} rate={cfg.rate_hz}Hz")
    acc = cfg.access
    log(f"[g1_guardian] access: local={list(acc.local)} control={list(acc.control)} monitor={list(acc.monitor)}; "
        "other addresses are dropped")
    if args.bind not in acc.local:
        log(f"[g1_guardian][WARN] --bind {args.bind} is not in access.local: CLI commands on this computer "
            "will be refused")
    log("[g1_guardian] idle: no frames are sent until a client engages")
    setup_realtime(args.rt_priority, args.cpu, log)
    signal.signal(signal.SIGTERM, lambda *_: svc.request_shutdown())
    signal.signal(signal.SIGINT, lambda *_: svc.request_shutdown())
    gc.collect()
    gc.freeze()
    try:
        svc.run()
    finally:
        if svc.core.state != "idle":
            log("[g1_guardian][WARN] exiting while engaged: frames stop now (G1D arm will drop)")
        svc.close()
        svc.blackbox.wait(2.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
