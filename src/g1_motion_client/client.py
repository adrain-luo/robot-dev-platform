"""``MotionClient`` / ``LeaseHandle`` (g1_contracts.motion) over a ``CoreConnection``.

A ``RemoteLease`` behaves like the in-process lease: rejected goals and
streams raise ``MotionRejected`` (``LeaseRevoked`` once the lease is gone),
``hold`` never raises, and ``cancelled`` is set as soon as the core revokes
the lease (E-STOP, re-engage, release, output not ready, missed heartbeats)
or the connection drops.
"""

from __future__ import annotations

import itertools
import threading
from typing import Any, Optional, Sequence

from g1_contracts import wire
from g1_contracts.motion import (
    REVOKED_REASONS, Ack, GoalRequest, Lease, LeaseRequest, MotionState, Reason, Resource, SafetyState, Stamp,
    StreamSetpoint, Verdict, rejection,
)
from g1_motion_client.connection import CoreConnection, CoreUnavailable

GOAL_TIMEOUT_S = 60.0  # a goal may wait for a detour plan in the core


def _reject(seq: int, reason: Reason, detail: str) -> Ack:
    return Ack(seq=seq, verdict=Verdict.REJECTED, reason=reason, detail=detail)


class RemoteLease:
    def __init__(self, conn: CoreConnection, lease: Lease, token: str) -> None:
        self._conn = conn
        self._lease = lease
        self.token = token
        self._seq = itertools.count(1)
        self._cancelled = threading.Event()
        self.revoked: Optional[str] = None
        self.returned = False
        self.holder_cancelled = False
        conn.watch_lease(lease.lease_id, self._on_event)

    # ---- events from the core
    def _on_event(self, lease_id: str, event: str, reason: str) -> None:
        if event == "returned":
            self.returned = True
        elif self.revoked is None:
            self.revoked = reason or "revoked"
        self._cancelled.set()

    # ---- protocol
    @property
    def lease(self) -> Lease:
        return self._lease

    @property
    def owner(self) -> str:
        return self.token

    @property
    def cancelled(self) -> threading.Event:
        return self._cancelled

    def check(self) -> None:
        if self.revoked:
            raise rejection(_reject(0, Reason.STALE_EPOCH, f"租约已撤销：{self.revoked}"))
        if self.returned:
            raise rejection(_reject(0, Reason.UNKNOWN_LEASE, "租约已归还"))
        if self.holder_cancelled:
            raise rejection(_reject(0, Reason.CANCELLED, "已停止"))

    def cancel(self) -> None:
        self.holder_cancelled = True
        self._cancelled.set()
        try:
            self._conn.request("lease_cancel", lease_id=self._lease.lease_id)
        except Exception:  # noqa: BLE001  the local flag already stops this side; the core rejects with CANCELLED or STALE
            pass

    def _command(self, kind: str, body_key: str, obj: Any, seq: int, timeout_s: Optional[float] = None) -> Ack:
        try:
            reply = self._conn.request(kind, timeout_s=timeout_s, **{body_key: wire.to_wire(obj)})
        except CoreUnavailable as exc:
            self._lost(str(exc))
            raise rejection(_reject(seq, Reason.STALE_EPOCH, f"运控核心连接断开，这一步不执行（{exc}）")) from None
        ack = wire.from_wire(Ack, reply.get("ack"))
        if not ack.ok:
            if ack.reason in REVOKED_REASONS and self.revoked is None:
                self.revoked = ack.detail or ack.reason.value
                self._cancelled.set()
            raise rejection(ack)
        return ack

    def _lost(self, why: str) -> None:
        if self.revoked is None:
            self.revoked = why
        self._cancelled.set()

    def goal(self, resource: Resource, path: Sequence[Sequence[float]], *,
             timeout_ms: Optional[int] = None, allow_detour: bool = False) -> Ack:
        seq = next(self._seq)
        try:
            req = GoalRequest(lease_id=self._lease.lease_id, epoch=self._lease.epoch, seq=seq, resource=resource,
                              path=tuple(tuple(p) for p in path), timeout_ms=timeout_ms, allow_detour=allow_detour)
        except (TypeError, ValueError) as exc:
            raise rejection(_reject(seq, Reason.INVALID, str(exc))) from None
        return self._command("goal", "request", req, seq, timeout_s=GOAL_TIMEOUT_S)

    def stream(self, resource: Resource, q: Sequence[float], *, valid_for_ms: int = 200,
               derived_from: Optional[Stamp] = None, speed_scale: float = 1.0) -> Ack:
        seq = next(self._seq)
        try:
            sp = StreamSetpoint(lease_id=self._lease.lease_id, epoch=self._lease.epoch, seq=seq, resource=resource,
                                q=tuple(q), derived_from=derived_from, valid_for_ms=valid_for_ms,
                                speed_scale=speed_scale)
        except (TypeError, ValueError) as exc:
            raise rejection(_reject(seq, Reason.INVALID, str(exc))) from None
        return self._command("stream", "setpoint", sp, seq)

    def hold(self, resource: Optional[Resource] = None) -> Ack:
        res = self._res(resource)
        seq = next(self._seq)
        try:
            reply = self._conn.request("hold", lease_id=self._lease.lease_id, resource=res.value, hold_seq=seq)
        except CoreUnavailable as exc:
            self._lost(str(exc))
            return _reject(seq, Reason.STALE_EPOCH, "运控核心连接断开（核心已按断线撤销租约并停手）")
        except Exception as exc:  # noqa: BLE001  hold never raises
            return _reject(seq, Reason.INVALID, str(exc))
        return wire.from_wire(Ack, reply.get("ack"))

    def release(self) -> None:
        if self.returned:
            return
        self.returned = True
        self._cancelled.set()
        try:
            self._conn.request("lease_return", lease_id=self._lease.lease_id)
        except Exception:  # noqa: BLE001  gone already (revoked or disconnected): nothing to return
            pass
        finally:
            self._conn.forget_lease(self._lease.lease_id)

    def __enter__(self) -> "RemoteLease":
        return self

    def __exit__(self, *exc) -> None:
        self.release()

    def _res(self, resource: Optional[Resource]) -> Resource:
        if resource is not None:
            return Resource(resource)
        if len(self._lease.resources) != 1:
            raise ValueError("租约含多个资源，要写明是哪一个")
        return self._lease.resources[0]


class MotionClient:
    def __init__(self, conn: CoreConnection) -> None:
        self.conn = conn

    def lease(self, resources: Sequence[Resource], owner: str, *,
              heartbeat_ms: Optional[int] = None, max_speed_rad_s: Optional[float] = None) -> RemoteLease:
        """Acquire now; the returned lease is also a context manager (returned on exit).

        ``max_speed_rad_s``: speed limit for the leased arms (D3); None = the core's global limit."""
        req = LeaseRequest(resources=tuple(resources), owner=owner, heartbeat_ms=heartbeat_ms,
                           max_speed_rad_s=max_speed_rad_s)
        reply = self.conn.request("lease", request=wire.to_wire(req))
        lease = wire.from_wire(Lease, reply.get("lease"))
        return RemoteLease(self.conn, lease, str(reply.get("token") or owner))

    def state(self, resource: Resource) -> MotionState:
        reply = self.conn.request("state", resource=Resource(resource).value)
        return wire.from_wire(MotionState, reply.get("state"))

    def state_at(self, resource: Resource, stamp: Stamp) -> MotionState:
        reply = self.conn.request("state_at", resource=Resource(resource).value, stamp=wire.to_wire(stamp))
        return wire.from_wire(MotionState, reply.get("state"))

    def estop(self, reason: str = "") -> None:
        self.conn.estop(reason)

    def safety(self) -> SafetyState:
        return SafetyState(self.conn.request("safety").get("safety"))

    def call(self, op: str, *, timeout_s: Optional[float] = None, **args: Any) -> Any:
        return self.conn.call(op, timeout_s=timeout_s, **args)
