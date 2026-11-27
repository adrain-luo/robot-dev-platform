"""网页复位的人工确认（web/reset_confirm.py）：假时钟，不起网页。

真实 HTTP 上的两步复位在 test_mock_server.py（进程内）和 test_web_ipc.py（运控核心进程）。
"""

import pytest

from robot_platform.control.arm_loop import PermissionErrorMotion
from robot_platform.web.reset_confirm import MAX_PENDING, TTL_S, ResetConfirm, estop_fingerprint


class Clock:
    def __init__(self) -> None:
        self.t = 100.0

    def __call__(self) -> float:
        return self.t


@pytest.fixture
def clock():
    return Clock()


@pytest.fixture
def rc(clock):
    return ResetConfirm(clock=clock)


LATCH = (7, "急停已锁定（页面急停）")


def test_a_token_resets_once(rc):
    token, ttl = rc.issue("estop", LATCH)
    assert ttl == TTL_S and rc.pending() == 1
    rc.redeem("estop", token, LATCH)
    assert rc.pending() == 0
    with pytest.raises(PermissionErrorMotion, match="已用过"):
        rc.redeem("estop", token, LATCH)


@pytest.mark.parametrize("token", [None, "", 0])
def test_no_token_is_refused(rc, token):
    rc.issue("estop", LATCH)
    with pytest.raises(PermissionErrorMotion, match="先在页面上确认"):
        rc.redeem("estop", token, LATCH)
    assert rc.pending() == 1, "没带令牌不碰已发出的令牌"


def test_an_unknown_token_is_refused(rc):
    rc.issue("estop", LATCH)
    with pytest.raises(PermissionErrorMotion, match="确认无效"):
        rc.redeem("estop", "made-up", LATCH)


def test_a_token_is_for_one_kind_of_reset(rc):
    token, _ = rc.issue("estop", LATCH)
    with pytest.raises(PermissionErrorMotion, match="不是给这个复位的"):
        rc.redeem("guardian", token, LATCH)
    with pytest.raises(PermissionErrorMotion, match="已用过"):
        rc.redeem("estop", token, LATCH)  # 用错一次就作废


def test_a_token_expires(rc, clock):
    token, ttl = rc.issue("estop", LATCH)
    clock.t += ttl + 0.01
    with pytest.raises(PermissionErrorMotion, match="已失效"):
        rc.redeem("estop", token, LATCH)
    assert rc.pending() == 0


def test_a_token_just_inside_its_lifetime_works(rc, clock):
    token, ttl = rc.issue("estop", LATCH)
    clock.t += ttl - 0.01
    rc.redeem("estop", token, LATCH)


def test_a_changed_latch_refuses_and_burns_the_token(rc):
    """人确认的是发令牌那一刻的锁存：之后又急停 / 接管 / 释放（epoch 变了），确认不算数。"""
    token, _ = rc.issue("estop", LATCH)
    with pytest.raises(PermissionErrorMotion, match="锁存变了"):
        rc.redeem("estop", token, (8, LATCH[1]))
    with pytest.raises(PermissionErrorMotion, match="已用过"):
        rc.redeem("estop", token, LATCH)


def test_tokens_are_bounded_and_the_oldest_go_first(rc):
    tokens = [rc.issue("estop", LATCH)[0] for _ in range(MAX_PENDING + 2)]
    assert rc.pending() == MAX_PENDING
    for old in tokens[:2]:
        with pytest.raises(PermissionErrorMotion, match="确认无效"):
            rc.redeem("estop", old, LATCH)
    rc.redeem("estop", tokens[-1], LATCH)


def test_expired_tokens_are_dropped(rc, clock):
    rc.issue("estop", LATCH)
    clock.t += TTL_S + 1
    assert rc.pending() == 0


def test_tokens_are_unguessable_and_distinct(rc):
    tokens = {rc.issue("estop", LATCH)[0] for _ in range(MAX_PENDING)}
    assert len(tokens) == MAX_PENDING and all(len(t) >= 20 for t in tokens)


def test_only_known_resets_get_tokens(rc):
    with pytest.raises(ValueError):
        rc.issue("release", LATCH)


def test_the_estop_fingerprint_is_the_epoch_and_the_reason():
    st = {"epoch": 3, "latched": True, "latch_reason": "急停已锁定（页面急停）", "leases": []}
    assert estop_fingerprint(st) == (3, "急停已锁定（页面急停）")
