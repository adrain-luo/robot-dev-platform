"""Wire format of the motion interface: tasks <-> motion core over a local socket (phase 2).

Standard library only. One message is one JSON object (UTF-8), sent as one
record on a Unix ``SOCK_SEQPACKET`` socket (record boundaries are kept, so no
framing is needed)::

    {"v": "1.1", "type": "<message type>", "seq": <int >= 0>, ...fields}

- ``v`` is ``CONTRACT_VERSION``. A peer that receives any other version
  rejects the message and closes the connection; there is no negotiation.
- ``seq`` is chosen by the sender. A reply carries the ``seq`` of the
  request it answers.
- Contract dataclasses travel as plain objects (``to_wire`` / ``from_wire``):
  enums as their string values, tuples as lists. ``from_wire`` rebuilds the
  dataclass, so a decoded message has passed the same checks as a local one.
  Unknown fields are rejected: both ends run the same version.

Sockets (paths are the core's; tests use a temporary directory):

    core.sock   leases, goal / stream / hold, state, platform calls
    estop.sock  E-STOP only, served by its own thread so it never queues
                behind task traffic

Client -> core on core.sock:

    hello         {client, pid}                 -> reply {core: {...}}; must be the first message
    heartbeat     {}                            no reply; any message also counts as one
    lease         {request: LeaseRequest}       -> reply {lease: Lease, token}
    lease_return  {lease_id}                    -> reply {}
    lease_cancel  {lease_id}                    -> reply {}
    goal          {request: GoalRequest}        -> reply {ack: Ack}
    stream        {setpoint: StreamSetpoint}    -> reply {ack: Ack}
    hold          {lease_id, resource, hold_seq} -> reply {ack: Ack}
    state         {resource}                    -> reply {state: MotionState}
    state_at      {resource, stamp: Stamp}      -> reply {state: MotionState}
    safety        {}                            -> reply {safety}
    call          {op, args: {...}}             -> reply {result}   platform extensions

Core -> client:

    reply         {ok: true, ...} or {ok: false, error: Error}
    lease_event   {lease_id, event: "revoked" | "returned", reason}

Client -> core on estop.sock (no hello):

    estop         {client, reason}              -> reply {}

``Error`` is ``{kind, detail}`` plus ``ack`` (an ``Ack``) when ``kind`` is
``rejected``. Kinds: ``rejected`` (a contract rejection: raise
``rejection(ack)``), ``permission``, ``value``, ``collision``, ``busy`` (too
many requests in flight on this connection), ``unsupported`` (unknown op or
message type), ``version``, ``protocol``, ``timeout``, ``internal``.

Liveness (design decision D5): the client sends a heartbeat every
``HEARTBEAT_PERIOD_MS``. When the core hears nothing on a connection for
``HEARTBEAT_TIMEOUT_MS``, or the connection closes, it revokes that
connection's leases and holds the arms it moved by hand; frames keep going.
"""

from __future__ import annotations

import json
import math
from typing import Any, Dict, Optional, Type, TypeVar

from g1_contracts.motion import (
    CONTRACT_VERSION, Ack, GoalRequest, Lease, LeaseRequest, MotionState, Reason, Resource, SafetyState,
    Stamp, StreamSetpoint, Verdict,
)

MAX_MESSAGE_BYTES = 1 << 20     # one record; collision geometry is the largest reply (tens of kB)
HEARTBEAT_PERIOD_MS = 50        # D5
HEARTBEAT_TIMEOUT_MS = 250      # D5: revoke after this long without any message on the connection

CLIENT_TYPES = ("hello", "heartbeat", "lease", "lease_return", "lease_cancel", "goal", "stream", "hold",
                "state", "state_at", "safety", "call")
CORE_TYPES = ("reply", "lease_event")
ESTOP_TYPES = ("estop",)
ERROR_KINDS = ("rejected", "permission", "value", "collision", "busy", "unsupported", "version", "protocol",
               "timeout", "internal")

T = TypeVar("T")


class WireError(ValueError):
    """A message that cannot be accepted. ``kind`` is the error kind to answer with."""

    def __init__(self, kind: str, detail: str) -> None:
        self.kind = kind
        self.detail = detail
        super().__init__(f"{kind}: {detail}")


# ---------------------------------------------------------------- envelope
def message(kind: str, seq: int, **fields: Any) -> Dict[str, Any]:
    msg = {"v": CONTRACT_VERSION, "type": kind, "seq": int(seq)}
    for key in fields:
        if key in msg:
            raise ValueError(f"field {key!r} is reserved")
    msg.update(fields)
    return msg


def encode(msg: Dict[str, Any]) -> bytes:
    data = json.dumps(msg, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")
    if len(data) > MAX_MESSAGE_BYTES:
        raise WireError("protocol", f"message is {len(data)} bytes, limit {MAX_MESSAGE_BYTES}")
    return data


def decode(data: bytes, allowed_types=None) -> Dict[str, Any]:
    """Parse and check the envelope. Raises ``WireError``."""
    if len(data) > MAX_MESSAGE_BYTES:
        raise WireError("protocol", f"message is {len(data)} bytes, limit {MAX_MESSAGE_BYTES}")
    try:
        msg = json.loads(data.decode("utf-8"), parse_constant=_no_constant)
    except (UnicodeDecodeError, ValueError) as exc:
        raise WireError("protocol", f"not JSON: {exc}") from None
    if not isinstance(msg, dict):
        raise WireError("protocol", "not a JSON object")
    if msg.get("v") != CONTRACT_VERSION:
        raise WireError("version", f"interface version {msg.get('v')!r}, this end speaks {CONTRACT_VERSION}")
    kind = msg.get("type")
    if not isinstance(kind, str) or (allowed_types is not None and kind not in allowed_types):
        raise WireError("unsupported", f"message type {kind!r}")
    seq = msg.get("seq")
    if isinstance(seq, bool) or not isinstance(seq, int) or seq < 0:
        raise WireError("protocol", f"seq must be a non-negative int, got {seq!r}")
    return msg


def _no_constant(name: str):
    raise ValueError(f"{name} is not allowed")


def error(kind: str, detail: str, ack: Optional[Ack] = None) -> Dict[str, Any]:
    if kind not in ERROR_KINDS:
        raise ValueError(f"unknown error kind {kind!r}")
    out: Dict[str, Any] = {"kind": kind, "detail": str(detail)}
    if ack is not None:
        out["ack"] = to_wire(ack)
    return out


# ---------------------------------------------------------------- dataclasses
def _fields(d: Any, cls: type, required, optional=()) -> Dict[str, Any]:
    if not isinstance(d, dict):
        raise WireError("protocol", f"{cls.__name__}: expected an object, got {type(d).__name__}")
    unknown = set(d) - set(required) - set(optional)
    if unknown:
        raise WireError("protocol", f"{cls.__name__}: unknown fields {sorted(unknown)}")
    missing = [k for k in required if k not in d]
    if missing:
        raise WireError("protocol", f"{cls.__name__}: missing fields {missing}")
    return d


def _str(v: Any, what: str) -> str:
    if not isinstance(v, str):
        raise WireError("protocol", f"{what} must be a string")
    return v


def _int(v: Any, what: str) -> int:
    if isinstance(v, bool) or not isinstance(v, int):
        raise WireError("protocol", f"{what} must be an int")
    return v


def _opt_int(v: Any, what: str) -> Optional[int]:
    return None if v is None else _int(v, what)


def _number(v: Any, what: str) -> float:
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(float(v)):
        raise WireError("protocol", f"{what} must be a finite number")
    return float(v)


def _vector(v: Any, what: str):
    if not isinstance(v, list):
        raise WireError("protocol", f"{what} must be a list")
    return tuple(_number(x, what) for x in v)


def _enum(cls, v: Any, what: str):
    try:
        return cls(v)
    except ValueError:
        raise WireError("protocol", f"{what}: unknown value {v!r}") from None


def _stamp_from(d: Any) -> Stamp:
    d = _fields(d, Stamp, ("t_ns", "host"), ("uncertainty_ns",))
    return Stamp(t_ns=_int(d["t_ns"], "Stamp.t_ns"), host=_str(d["host"], "Stamp.host"),
                 uncertainty_ns=_int(d.get("uncertainty_ns", 0), "Stamp.uncertainty_ns"))


def _stamp_to(s: Stamp) -> Dict[str, Any]:
    return {"t_ns": s.t_ns, "host": s.host, "uncertainty_ns": s.uncertainty_ns}


def to_wire(obj: Any) -> Dict[str, Any]:
    """Contract dataclass -> plain object."""
    if isinstance(obj, Stamp):
        return _stamp_to(obj)
    if isinstance(obj, LeaseRequest):
        return {"resources": [r.value for r in obj.resources], "owner": obj.owner, "heartbeat_ms": obj.heartbeat_ms}
    if isinstance(obj, Lease):
        return {"lease_id": obj.lease_id, "epoch": obj.epoch, "resources": [r.value for r in obj.resources],
                "owner": obj.owner}
    if isinstance(obj, StreamSetpoint):
        return {"lease_id": obj.lease_id, "epoch": obj.epoch, "seq": obj.seq, "resource": obj.resource.value,
                "q": list(obj.q), "derived_from": None if obj.derived_from is None else _stamp_to(obj.derived_from),
                "valid_for_ms": obj.valid_for_ms, "speed_scale": float(obj.speed_scale)}
    if isinstance(obj, GoalRequest):
        return {"lease_id": obj.lease_id, "epoch": obj.epoch, "seq": obj.seq, "resource": obj.resource.value,
                "path": [list(p) for p in obj.path], "timeout_ms": obj.timeout_ms, "allow_detour": obj.allow_detour}
    if isinstance(obj, Ack):
        return {"seq": obj.seq, "verdict": obj.verdict.value, "reason": obj.reason.value, "detail": obj.detail}
    if isinstance(obj, MotionState):
        return {"stamp": _stamp_to(obj.stamp), "resource": obj.resource.value, "q_meas": list(obj.q_meas),
                "q_cmd": list(obj.q_cmd), "lease_owner": obj.lease_owner, "safety": obj.safety.value,
                "epoch": obj.epoch}
    raise TypeError(f"no wire form for {type(obj).__name__}")


def from_wire(cls: Type[T], d: Any) -> T:
    """Plain object -> contract dataclass (validated). Raises ``WireError``."""
    try:
        return _from_wire(cls, d)
    except WireError:
        raise
    except (TypeError, ValueError) as exc:  # the dataclass's own validation
        raise WireError("protocol", f"{cls.__name__}: {exc}") from None


def _from_wire(cls, d):
    if cls is Stamp:
        return _stamp_from(d)
    if cls is LeaseRequest:
        d = _fields(d, cls, ("resources", "owner"), ("heartbeat_ms",))
        if not isinstance(d["resources"], list):
            raise WireError("protocol", "LeaseRequest.resources must be a list")
        return LeaseRequest(resources=tuple(_enum(Resource, r, "LeaseRequest.resources") for r in d["resources"]),
                            owner=_str(d["owner"], "LeaseRequest.owner"),
                            heartbeat_ms=_opt_int(d.get("heartbeat_ms"), "LeaseRequest.heartbeat_ms"))
    if cls is Lease:
        d = _fields(d, cls, ("lease_id", "epoch", "resources", "owner"))
        if not isinstance(d["resources"], list):
            raise WireError("protocol", "Lease.resources must be a list")
        return Lease(lease_id=_str(d["lease_id"], "Lease.lease_id"), epoch=_int(d["epoch"], "Lease.epoch"),
                     resources=tuple(_enum(Resource, r, "Lease.resources") for r in d["resources"]),
                     owner=_str(d["owner"], "Lease.owner"))
    if cls is StreamSetpoint:
        d = _fields(d, cls, ("lease_id", "epoch", "seq", "resource", "q"),
                    ("derived_from", "valid_for_ms", "speed_scale"))
        obs = d.get("derived_from")
        return StreamSetpoint(lease_id=_str(d["lease_id"], "StreamSetpoint.lease_id"),
                              epoch=_int(d["epoch"], "StreamSetpoint.epoch"),
                              seq=_int(d["seq"], "StreamSetpoint.seq"),
                              resource=_enum(Resource, d["resource"], "StreamSetpoint.resource"),
                              q=_vector(d["q"], "StreamSetpoint.q"),
                              derived_from=None if obs is None else _stamp_from(obs),
                              valid_for_ms=_int(d.get("valid_for_ms", 200), "StreamSetpoint.valid_for_ms"),
                              speed_scale=_number(d.get("speed_scale", 1.0), "StreamSetpoint.speed_scale"))
    if cls is GoalRequest:
        d = _fields(d, cls, ("lease_id", "epoch", "seq", "resource", "path"), ("timeout_ms", "allow_detour"))
        if not isinstance(d["path"], list):
            raise WireError("protocol", "GoalRequest.path must be a list")
        allow = d.get("allow_detour", False)
        if not isinstance(allow, bool):
            raise WireError("protocol", "GoalRequest.allow_detour must be a bool")
        return GoalRequest(lease_id=_str(d["lease_id"], "GoalRequest.lease_id"),
                           epoch=_int(d["epoch"], "GoalRequest.epoch"), seq=_int(d["seq"], "GoalRequest.seq"),
                           resource=_enum(Resource, d["resource"], "GoalRequest.resource"),
                           path=tuple(_vector(p, "GoalRequest.path") for p in d["path"]),
                           timeout_ms=_opt_int(d.get("timeout_ms"), "GoalRequest.timeout_ms"), allow_detour=allow)
    if cls is Ack:
        d = _fields(d, cls, ("seq", "verdict"), ("reason", "detail"))
        return Ack(seq=_int(d["seq"], "Ack.seq"), verdict=_enum(Verdict, d["verdict"], "Ack.verdict"),
                   reason=_enum(Reason, d.get("reason", ""), "Ack.reason"), detail=_str(d.get("detail", ""), "Ack.detail"))
    if cls is MotionState:
        d = _fields(d, cls, ("stamp", "resource", "q_meas", "q_cmd", "lease_owner", "safety", "epoch"))
        owner = d["lease_owner"]
        return MotionState(stamp=_stamp_from(d["stamp"]), resource=_enum(Resource, d["resource"], "MotionState.resource"),
                           q_meas=_vector(d["q_meas"], "MotionState.q_meas"),
                           q_cmd=_vector(d["q_cmd"], "MotionState.q_cmd"),
                           lease_owner=None if owner is None else _str(owner, "MotionState.lease_owner"),
                           safety=_enum(SafetyState, d["safety"], "MotionState.safety"),
                           epoch=_int(d["epoch"], "MotionState.epoch"))
    raise TypeError(f"no wire form for {cls.__name__}")
