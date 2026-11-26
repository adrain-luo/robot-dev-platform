"""Motion interface: the only contract between skills / tasks and the motion core.

Standard library only. The types follow design review section 7
(``g1_contracts/motion.py``). Additions, each marked "v1" below:

- ``Reason`` codes and ``SafetyState`` are enums instead of free strings.
- ``GoalRequest`` carries ``seq`` (so its ``Ack`` can be matched) and
  ``allow_detour`` (the core may plan around a collision).
- ``MotionState`` carries ``resource`` and ``epoch``.
- ``LeaseRequest.heartbeat_ms`` is optional; an in-process core does not
  enforce heartbeats. Over IPC (phase 2) the core checks heartbeats per
  connection (``g1_contracts.wire.HEARTBEAT_TIMEOUT_MS``) and refuses a
  lease that sets its own.
- ``MotionRejected`` / ``LeaseRevoked`` exceptions and the ``MotionClient`` /
  ``LeaseHandle`` protocols.

v1 limits: setpoints are joint angles only (IK stays in the skill), a goal
path has exactly one point, and ``speed_scale`` must be 1.0.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from enum import Enum
from typing import ContextManager, Iterable, Optional, Protocol, Sequence, Tuple, Union

CONTRACT_VERSION = "1.1"  # 1.1: wire format (g1_contracts.wire); the types are unchanged


class Resource(str, Enum):
    RIGHT_ARM = "right_arm"
    LEFT_ARM = "left_arm"
    WAIST = "waist"
    CHASSIS = "chassis"
    LIFT = "lift"
    RIGHT_TOOL = "right_tool"


class Verdict(str, Enum):
    ACCEPTED = "accepted"
    CLAMPED = "clamped"      # accepted after clamping to the joint limits
    REJECTED = "rejected"


class Reason(str, Enum):
    NONE = ""
    STALE_EPOCH = "stale_epoch"            # lease was revoked (E-STOP, re-engage, release, fault)
    UNKNOWN_LEASE = "unknown_lease"        # lease id never granted, or already returned
    CANCELLED = "cancelled"                # the holder cancelled its own lease
    ESTOP_LATCHED = "estop_latched"
    NOT_ENGAGED = "not_engaged"
    NOT_READY = "not_ready"                # output not ready: transition, guardian fault, link lost
    BUSY = "busy"                          # resource held by another owner, or in teach mode
    COLLISION = "collision"
    LIMIT = "limit"                        # with CLAMPED: target was outside the joint limits
    STALE_OBSERVATION = "stale_observation"
    UNSUPPORTED = "unsupported"
    INVALID = "invalid"


class SafetyState(str, Enum):
    NORMAL = "normal"
    HOLD = "hold"                          # engaged but not accepting motion (transition / fault)
    ESTOP_LATCHED = "estop_latched"
    STALE_LOWSTATE = "stale_lowstate"
    LINK_LOST = "link_lost"
    RELEASED = "released"                  # not engaged


def _joint_vector(values: Iterable[float], what: str) -> Tuple[float, ...]:
    try:
        out = tuple(float(v) for v in values)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{what}: not a sequence of numbers ({exc})") from None
    if not out:
        raise ValueError(f"{what}: empty")
    if not all(math.isfinite(v) for v in out):
        raise ValueError(f"{what}: contains a non-finite value")
    return out


def _resource(value: Union[Resource, str]) -> Resource:
    try:
        return Resource(value)
    except ValueError:
        raise ValueError(f"unknown resource {value!r}") from None


def _non_negative_int(value: int, what: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{what} must be a non-negative int, got {value!r}")
    return value


def _positive_ms(value: Optional[int], what: str, *, optional: bool) -> Optional[int]:
    if value is None and optional:
        return None
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{what} must be a positive int (ms), got {value!r}")
    return value


@dataclass(frozen=True)
class Stamp:
    t_ns: int                # monotonic clock of the host that produced it
    host: str                # e.g. "body" / "box"; the core only compares stamps of its own host in v1
    uncertainty_ns: int = 0  # uncertainty of the mapping onto the core's clock

    def __post_init__(self) -> None:
        _non_negative_int(self.t_ns, "Stamp.t_ns")
        _non_negative_int(self.uncertainty_ns, "Stamp.uncertainty_ns")
        if not isinstance(self.host, str) or not self.host:
            raise ValueError("Stamp.host must be a non-empty string")


@dataclass(frozen=True)
class LeaseRequest:
    resources: Tuple[Resource, ...]
    owner: str                          # e.g. "skill:visual_align/task-0412"
    heartbeat_ms: Optional[int] = None  # v1: optional; revoke when no heartbeat for this long

    def __post_init__(self) -> None:
        res = tuple(_resource(r) for r in self.resources)
        if not res:
            raise ValueError("LeaseRequest.resources is empty")
        if len(set(res)) != len(res):
            raise ValueError("LeaseRequest.resources has duplicates")
        object.__setattr__(self, "resources", res)
        if not isinstance(self.owner, str) or not self.owner.strip():
            raise ValueError("LeaseRequest.owner must be a non-empty string")
        _positive_ms(self.heartbeat_ms, "LeaseRequest.heartbeat_ms", optional=True)


@dataclass(frozen=True)
class Lease:
    lease_id: str
    epoch: int                    # bumped on E-STOP or revoke; commands with an older epoch are rejected
    resources: Tuple[Resource, ...]
    owner: str


@dataclass(frozen=True)
class StreamSetpoint:
    """Visual servoing: replaces the target, never starts a planned move."""

    lease_id: str
    epoch: int
    seq: int
    resource: Resource
    q: Tuple[float, ...]                 # v1: joint angles only, IK on the skill side
    derived_from: Optional[Stamp] = None  # capture time of the observation this target came from
    valid_for_ms: int = 200              # timed by the core on receipt; expiry = decelerate and hold
    speed_scale: float = 1.0             # can only lower the core's limit; v1 accepts 1.0 only

    def __post_init__(self) -> None:
        object.__setattr__(self, "resource", _resource(self.resource))
        object.__setattr__(self, "q", _joint_vector(self.q, "StreamSetpoint.q"))
        _non_negative_int(self.seq, "StreamSetpoint.seq")
        _positive_ms(self.valid_for_ms, "StreamSetpoint.valid_for_ms", optional=False)
        s = float(self.speed_scale)
        if not (math.isfinite(s) and 0.0 < s <= 1.0):
            raise ValueError(f"StreamSetpoint.speed_scale must be in (0, 1], got {self.speed_scale!r}")


@dataclass(frozen=True)
class GoalRequest:
    """Move to a goal; the core re-checks it (limits, collision gate) before accepting."""

    lease_id: str
    epoch: int
    seq: int                                  # v1 addition
    resource: Resource
    path: Tuple[Tuple[float, ...], ...]       # v1: exactly one point
    timeout_ms: Optional[int] = None          # hold if the goal is still being pursued after this
    allow_detour: bool = False                # v1 addition: plan around a collision if needed

    def __post_init__(self) -> None:
        object.__setattr__(self, "resource", _resource(self.resource))
        _non_negative_int(self.seq, "GoalRequest.seq")
        try:
            pts = tuple(_joint_vector(p, f"GoalRequest.path[{i}]") for i, p in enumerate(self.path))
        except TypeError:
            raise ValueError("GoalRequest.path must be a sequence of joint vectors") from None
        if not pts:
            raise ValueError("GoalRequest.path is empty")
        if len({len(p) for p in pts}) != 1:
            raise ValueError("GoalRequest.path points have different lengths")
        object.__setattr__(self, "path", pts)
        _positive_ms(self.timeout_ms, "GoalRequest.timeout_ms", optional=True)
        object.__setattr__(self, "allow_detour", bool(self.allow_detour))


@dataclass(frozen=True)
class Ack:
    seq: int
    verdict: Verdict
    reason: Reason = Reason.NONE
    detail: str = ""                          # human-readable text for logs and the UI

    @property
    def ok(self) -> bool:
        return self.verdict is not Verdict.REJECTED


@dataclass(frozen=True)
class MotionState:
    """Published by the core at its control rate; about 2 s of history is kept."""

    stamp: Stamp
    resource: Resource
    q_meas: Tuple[float, ...]
    q_cmd: Tuple[float, ...]
    lease_owner: Optional[str]
    safety: SafetyState
    epoch: int


class MotionRejected(PermissionError):
    """A command was rejected; ``ack`` says why."""

    def __init__(self, ack: Ack) -> None:
        self.ack = ack
        super().__init__(ack.detail or ack.reason.value or ack.verdict.value)

    @property
    def reason(self) -> Reason:
        return self.ack.reason


class LeaseRevoked(MotionRejected):
    """The lease is no longer valid (revoked, cancelled or returned). Stop the skill."""


REVOKED_REASONS = frozenset({Reason.STALE_EPOCH, Reason.UNKNOWN_LEASE, Reason.CANCELLED})


def rejection(ack: Ack) -> MotionRejected:
    """The exception that matches a rejected ``Ack``."""
    cls = LeaseRevoked if ack.reason in REVOKED_REASONS else MotionRejected
    return cls(ack)


class LeaseHandle(Protocol):
    """What a skill holds while it owns resources. Rejections raise ``MotionRejected``."""

    @property
    def lease(self) -> Lease: ...

    @property
    def cancelled(self) -> threading.Event:
        """Set when the lease is revoked or cancelled. Skills wait on it to stop promptly."""

    def check(self) -> None:
        """Raise ``LeaseRevoked`` if the lease is no longer valid."""

    def cancel(self) -> None:
        """Stop using the lease (it stays held until returned); later commands are rejected."""

    def goal(self, resource: Resource, path: Sequence[Sequence[float]], *,
             timeout_ms: Optional[int] = None, allow_detour: bool = False) -> Ack: ...

    def stream(self, resource: Resource, q: Sequence[float], *, valid_for_ms: int = 200,
               derived_from: Optional[Stamp] = None, speed_scale: float = 1.0) -> Ack: ...

    def hold(self, resource: Resource) -> Ack:
        """Stop at the current position. A no-op if the lease was already revoked."""


class MotionClient(Protocol):
    def lease(self, resources: Sequence[Resource], owner: str, *,
              heartbeat_ms: Optional[int] = None) -> ContextManager[LeaseHandle]:
        """Acquire resources; raises ``MotionRejected`` if they cannot be granted."""

    def state(self, resource: Resource) -> MotionState: ...

    def state_at(self, resource: Resource, stamp: Stamp) -> MotionState:
        """Measured and commanded joint angles at ``stamp`` (interpolated from history)."""

    def estop(self, reason: str = "") -> None:
        """Latch E-STOP: revoke every lease and hold. Only the core's local reset clears it."""

    def safety(self) -> SafetyState: ...
