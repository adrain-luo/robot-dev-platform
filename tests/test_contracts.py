"""g1_contracts: validation of the motion interface types and stable wire values.

The stdlib-only rule is checked in test_import_rules.py.
"""

import math

import pytest

import g1_contracts
from g1_contracts.motion import (
    CONTRACT_VERSION, Ack, GoalRequest, LeaseRequest, LeaseRevoked, MotionRejected, Reason, Resource,
    SafetyState, Stamp, StreamSetpoint, Verdict, rejection,
)


def _stream(**kw):
    args = dict(lease_id="L1", epoch=0, seq=0, resource=Resource.RIGHT_ARM, q=[0.0] * 7)
    args.update(kw)
    return StreamSetpoint(**args)


def _goal(**kw):
    args = dict(lease_id="L1", epoch=0, seq=0, resource="right_arm", path=[[0.0] * 7])
    args.update(kw)
    return GoalRequest(**args)


def test_version_is_exported():
    assert g1_contracts.CONTRACT_VERSION == CONTRACT_VERSION == "1.1"


def test_enum_values_are_stable_strings():
    # 这些字符串会出现在日志、页面和以后的 IPC 消息里：改名就是改接口
    assert [r.value for r in Resource] == ["right_arm", "left_arm", "waist", "chassis", "lift", "right_tool"]
    assert [v.value for v in Verdict] == ["accepted", "clamped", "rejected"]
    assert {s.value for s in SafetyState} == {
        "normal", "hold", "estop_latched", "stale_lowstate", "link_lost", "released"}
    assert Reason.STALE_EPOCH == "stale_epoch" and Reason.ESTOP_LATCHED == "estop_latched"


def test_stream_setpoint_normalises_and_validates():
    sp = _stream(resource="left_arm", q=(1, 2, 3))
    assert sp.resource is Resource.LEFT_ARM and sp.q == (1.0, 2.0, 3.0)
    for bad in ([math.nan], [math.inf, 0.0], [], ["x"]):
        with pytest.raises(ValueError):
            _stream(q=bad)
    for bad in (0, -5, 1.5, True):
        with pytest.raises(ValueError):
            _stream(valid_for_ms=bad)
    for bad in (0.0, 1.01, -1.0, math.nan):
        with pytest.raises(ValueError):
            _stream(speed_scale=bad)
    with pytest.raises(ValueError):
        _stream(resource="nose")
    with pytest.raises(ValueError):
        _stream(seq=-1)


def test_goal_request_validates_path_and_timeout():
    g = _goal(path=[(0, 0, 0)], timeout_ms=None, allow_detour=1)
    assert g.path == ((0.0, 0.0, 0.0),) and g.allow_detour is True and g.resource is Resource.RIGHT_ARM
    for bad in ([], [[0.0], [0.0, 1.0]], [[math.nan]], 3):
        with pytest.raises(ValueError):
            _goal(path=bad)
    with pytest.raises(ValueError):
        _goal(timeout_ms=0)


def test_lease_request_and_stamp_validation():
    r = LeaseRequest(resources=["right_arm"], owner="task:x")
    assert r.resources == (Resource.RIGHT_ARM,) and r.heartbeat_ms is None
    with pytest.raises(ValueError):
        LeaseRequest(resources=[], owner="task:x")
    with pytest.raises(ValueError):
        LeaseRequest(resources=["right_arm", Resource.RIGHT_ARM], owner="task:x")
    with pytest.raises(ValueError):
        LeaseRequest(resources=["right_arm"], owner=" ")
    with pytest.raises(ValueError):
        LeaseRequest(resources=["right_arm"], owner="x", heartbeat_ms=0)
    assert Stamp(t_ns=5, host="box").uncertainty_ns == 0
    for kw in (dict(t_ns=-1, host="box"), dict(t_ns=1.5, host="box"), dict(t_ns=1, host=""),
               dict(t_ns=1, host="box", uncertainty_ns=-1)):
        with pytest.raises(ValueError):
            Stamp(**kw)


def test_rejection_maps_revoked_reasons_to_lease_revoked():
    for reason in (Reason.STALE_EPOCH, Reason.UNKNOWN_LEASE, Reason.CANCELLED):
        exc = rejection(Ack(seq=3, verdict=Verdict.REJECTED, reason=reason, detail="撤销"))
        assert isinstance(exc, LeaseRevoked) and exc.ack.seq == 3 and str(exc) == "撤销"
    exc = rejection(Ack(seq=1, verdict=Verdict.REJECTED, reason=Reason.COLLISION))
    assert type(exc) is MotionRejected and isinstance(exc, PermissionError)
    assert exc.reason is Reason.COLLISION and str(exc) == "collision"
    assert Ack(seq=0, verdict=Verdict.CLAMPED, reason=Reason.LIMIT).ok

