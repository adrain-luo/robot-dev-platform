"""Wire format: one JSON object per UDP datagram, protocol version 1.

Box -> guardian (every message carries "v", "type", "client", "seq"):
  engage     take control (or re-read the epoch); refused while E-STOP latched
  setpoint   {"epoch", "valid_for_ms", "groups": {name: {"q", "tau_ff"?, "kp_scale"?}}}
  heartbeat  keeps the link alive between setpoints
  estop      latch E-STOP: hold, reject setpoints until reset (any allowed source)
  reset      clear E-STOP / faults (local or control); owners must engage again
  release    ramp stiffness / weight to 0, then stop sending frames (the owner, or local)

Who may send what depends on the source address (config "access", see
core.ROLE_COMMANDS); a datagram from any other address gets no answer.

Guardian -> Box: "status" objects, streamed to every address that sent a
message in the last few seconds, plus an immediate status carrying an
"ack" {"type", "seq", "ok", "reason"} after each command (and after a
rejected setpoint, rate-limited).
"""

from __future__ import annotations

import json
from typing import Any, Dict

from g1_guardian import PROTOCOL_VERSION

MAX_DATAGRAM = 16384
DEFAULT_PORT = 18090
COMMAND_TYPES = ("engage", "setpoint", "heartbeat", "estop", "reset", "release")


class ProtocolError(ValueError):
    pass


def encode(msg: Dict[str, Any]) -> bytes:
    data = json.dumps(msg, separators=(",", ":"), allow_nan=False).encode("utf-8")
    if len(data) > MAX_DATAGRAM:
        raise ProtocolError(f"message too large ({len(data)} bytes)")
    return data


def decode(data: bytes) -> Dict[str, Any]:
    if len(data) > MAX_DATAGRAM:
        raise ProtocolError("datagram too large")
    try:
        obj = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"not JSON: {exc}") from exc
    if not isinstance(obj, dict):
        raise ProtocolError("not a JSON object")
    return obj


def message(kind: str, client: str, seq: int, **fields: Any) -> Dict[str, Any]:
    msg = {"v": PROTOCOL_VERSION, "type": kind, "client": client, "seq": int(seq)}
    msg.update(fields)
    return msg
