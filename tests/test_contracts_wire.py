"""g1_contracts.wire：线上格式（阶段 2 任务 ↔ 运控核心）。每个类型往返不变；坏消息都报 WireError。"""

import json

import pytest

from g1_contracts import wire
from g1_contracts.motion import (
    CONTRACT_VERSION, Ack, GoalRequest, Lease, LeaseRequest, MotionState, Reason, Resource, SafetyState, Stamp,
    StreamSetpoint, Verdict,
)

SAMPLES = [
    Stamp(t_ns=123, host="box", uncertainty_ns=5),
    LeaseRequest(resources=(Resource.RIGHT_ARM, Resource.LEFT_ARM), owner="skill:x/1"),
    LeaseRequest(resources=(Resource.RIGHT_ARM,), owner="skill:x/1", heartbeat_ms=250),
    Lease(lease_id="L7", epoch=3, resources=(Resource.RIGHT_ARM,), owner="skill:x/1"),
    StreamSetpoint(lease_id="L7", epoch=3, seq=9, resource=Resource.RIGHT_ARM, q=(0.1, -0.2, 0.0),
                   derived_from=Stamp(t_ns=1, host="box"), valid_for_ms=100, speed_scale=0.5),
    StreamSetpoint(lease_id="L7", epoch=3, seq=10, resource="right_arm", q=[0.1]),
    GoalRequest(lease_id="L7", epoch=3, seq=4, resource=Resource.LEFT_ARM, path=((0.1, 0.2),), timeout_ms=500,
                allow_detour=True),
    Ack(seq=4, verdict=Verdict.CLAMPED, reason=Reason.LIMIT, detail="夹到限位"),
    Ack(seq=5, verdict=Verdict.ACCEPTED),
    MotionState(stamp=Stamp(t_ns=1, host="box"), resource=Resource.RIGHT_ARM, q_meas=(0.1,), q_cmd=(0.2,),
                lease_owner=None, safety=SafetyState.ESTOP_LATCHED, epoch=2),
    LeaseRequest(resources=(Resource.RIGHT_ARM,), owner="refactor-node", max_speed_rad_s=1.0),  # 1.2
]


@pytest.mark.parametrize("obj", SAMPLES, ids=lambda o: type(o).__name__)
def test_every_type_round_trips_through_json(obj):
    d = json.loads(json.dumps(wire.to_wire(obj)))
    assert wire.from_wire(type(obj), d) == obj


def test_envelope_round_trip_and_version():
    msg = wire.message("goal", 7, request=wire.to_wire(SAMPLES[6]))
    assert msg["v"] == CONTRACT_VERSION == "1.2"
    back = wire.decode(wire.encode(msg), wire.CLIENT_TYPES)
    assert back == msg
    assert wire.from_wire(GoalRequest, back["request"]) == SAMPLES[6]


@pytest.mark.parametrize("data, kind", [
    (b"\xff", "protocol"),
    (b"not json", "protocol"),
    (b"[1]", "protocol"),
    (json.dumps({"type": "goal", "seq": 1}).encode(), "version"),
    (json.dumps({"v": "1.0", "type": "goal", "seq": 1}).encode(), "version"),
    (json.dumps({"v": CONTRACT_VERSION, "type": "fly", "seq": 1}).encode(), "unsupported"),
    (json.dumps({"v": CONTRACT_VERSION, "type": "goal", "seq": -1}).encode(), "protocol"),
    (json.dumps({"v": CONTRACT_VERSION, "type": "goal", "seq": True}).encode(), "protocol"),
    (b'{"v": "1.2", "type": "goal", "seq": 1, "x": NaN}', "protocol"),
    (b"x" * (wire.MAX_MESSAGE_BYTES + 1), "protocol"),
])
def test_bad_envelopes_are_refused_with_a_kind(data, kind):
    with pytest.raises(wire.WireError) as e:
        wire.decode(data, wire.CLIENT_TYPES)
    assert e.value.kind == kind


def test_encode_refuses_nan_and_oversized_messages():
    with pytest.raises(ValueError):
        wire.encode(wire.message("reply", 1, x=float("nan")))
    with pytest.raises(wire.WireError):
        wire.encode(wire.message("reply", 1, x="y" * wire.MAX_MESSAGE_BYTES))
    with pytest.raises(ValueError):
        wire.message("reply", 1, v="2")


def _goal(**over):
    d = wire.to_wire(SAMPLES[6])
    d.update(over)
    return d


@pytest.mark.parametrize("cls, d", [
    (GoalRequest, _goal(extra=1)),                          # unknown field
    (GoalRequest, {k: v for k, v in _goal().items() if k != "epoch"}),  # missing field
    (GoalRequest, _goal(resource="tail")),                  # unknown enum value
    (GoalRequest, _goal(path=[[0.1], [0.2, 0.3]])),         # the dataclass's own check
    (GoalRequest, _goal(path=[["a"]])),
    (GoalRequest, _goal(allow_detour="yes")),
    (GoalRequest, _goal(seq=1.5)),
    (StreamSetpoint, {**wire.to_wire(SAMPLES[4]), "speed_scale": 2.0}),
    (StreamSetpoint, {**wire.to_wire(SAMPLES[4]), "q": []}),
    (LeaseRequest, {"resources": "right_arm", "owner": "x"}),
    (LeaseRequest, {"resources": ["right_arm", "right_arm"], "owner": "x"}),
    (LeaseRequest, {"resources": ["right_arm"], "owner": "x", "max_speed_rad_s": 0}),
    (LeaseRequest, {"resources": ["right_arm"], "owner": "x", "max_speed_rad_s": -1.0}),
    (LeaseRequest, {"resources": ["right_arm"], "owner": "x", "max_speed_rad_s": "fast"}),
    (LeaseRequest, {"resources": ["right_arm"], "owner": "x", "max_speed_rad_s": True}),
    (Ack, {"seq": 1, "verdict": "maybe"}),
    (Stamp, {"t_ns": -1, "host": "box"}),
    (MotionState, "nope"),
])
def test_bad_fields_are_refused(cls, d):
    with pytest.raises(wire.WireError) as e:
        wire.from_wire(cls, d)
    assert e.value.kind == "protocol"


def test_error_objects_carry_the_ack_for_rejections():
    ack = Ack(seq=3, verdict=Verdict.REJECTED, reason=Reason.STALE_EPOCH, detail="已撤销")
    err = wire.error("rejected", "已撤销", ack)
    assert wire.from_wire(Ack, err["ack"]) == ack
    with pytest.raises(ValueError):
        wire.error("oops", "x")


def test_heartbeat_numbers_are_the_owners_choice():
    """D5（负责人 2026-11-26 确认）：每 50 ms 一次心跳，250 ms 没消息就撤销。改这两个数要先问负责人。"""
    assert (wire.HEARTBEAT_PERIOD_MS, wire.HEARTBEAT_TIMEOUT_MS) == (50, 250)
