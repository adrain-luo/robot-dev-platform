"""运控核心（control/motion_core.py）：租约、epoch、撤销、goal / stream、设定有效期、state_at、急停锁存。

真的 ArmLoop + mock 机器人 + 记帧的指令出口；需要精确计时的地方用假时钟手动调 _on_cycle。
"""

import threading
import time

import numpy as np
import pytest

from g1_contracts.motion import (
    LeaseRevoked, MotionRejected, Reason, Resource, SafetyState, Stamp, Verdict,
)
from robot_platform.control.arm_loop import ArmLoop, PermissionErrorMotion
from robot_platform.control.fk_scene import RobotKinematics
from robot_platform.control.motion_core import MotionCore, _Sample
from robot_platform.core.robot_profile import active
from robot_platform.hw.backends.mock import MockBackend

from test_arm_loop_frame import Recorder

R, L = Resource.RIGHT_ARM, Resource.LEFT_ARM


class FlagCommander(Recorder):
    """可以切换「就绪」的指令出口（模拟守护层急停 / 失联时 ready() 变假）。"""

    def __init__(self):
        super().__init__()
        self.ok = True
        self.info_state = None

    def ready(self):
        return self.ok

    def phase(self):
        return "" if self.ok else "守护层急停已锁定"

    def info(self):
        return self.info_state


class FakeClock:
    def __init__(self, t_ns=10_000_000_000):
        self.t = t_ns

    def __call__(self):
        return self.t

    def advance_ms(self, ms):
        self.t += int(ms * 1_000_000)


@pytest.fixture
def rig():
    profile = active()
    backend = MockBackend(profile)
    cmd = FlagCommander()
    kin = RobotKinematics(profile.urdf, profile.base_link, list(profile.arms))
    loop = ArmLoop(reader=backend.state(), commander=cmd, kin=kin,
                   urdf_path=profile.urdf, base_link=profile.base_link)
    loop.set_max_speed(0.4)
    yield profile, backend, cmd, loop
    loop.shutdown()
    backend.close()


@pytest.fixture
def core(rig):
    _, _, _, loop = rig
    c = MotionCore(loop)
    c.engage()
    return c


@pytest.fixture
def fake(rig):
    """不装进控制循环的核心 + 假时钟：_on_cycle 由测试手动调。"""
    _, _, _, loop = rig
    clock = FakeClock()
    c = MotionCore(loop, clock_ns=clock, install=False)
    c.engage()
    return c, clock


def _desired(loop, arm="right"):
    return np.asarray(loop.status()["arms"][arm]["desired_rad"])


def _cmd(loop, arm="right"):
    return np.asarray(loop.status()["arms"][arm]["cmd_rad"])


def _away(loop, arm="right", d=-0.3):
    q = _cmd(loop, arm).copy()
    q[3] += d
    return q


# ---------------------------------------------------------------- 租约

def test_lease_is_exclusive_per_arm_and_shows_as_owner(rig, core):
    _, _, _, loop = rig
    with core.lease([R], "task:a") as la:
        assert loop.owner("right") == la.owner and la.owner.startswith("task:a#")
        with pytest.raises(MotionRejected) as e:
            core.lease([R], "task:b")
        assert e.value.reason is Reason.BUSY and "task:a" in str(e.value)
        # 另一只手不受影响；手动来源给被租的手下目标被拒
        with core.lease([L], "task:b"):
            pass
        with pytest.raises(PermissionErrorMotion, match="task:a"):
            loop.set_target(_cmd(loop), arm="right")
        assert [x["owner"] for x in core.status()["leases"]] == ["task:a"]
    assert loop.owner("right") is None and core.status()["leases"] == []
    loop.set_target(_cmd(loop), arm="right")


def test_lease_refused_when_not_engaged_latched_unsupported_or_heartbeat(rig):
    _, _, cmd, loop = rig
    core = MotionCore(loop)
    with pytest.raises(MotionRejected) as e:
        core.lease([R], "t")
    assert e.value.reason is Reason.NOT_ENGAGED
    core.engage()
    for res, why in (([Resource.WAIST], Reason.UNSUPPORTED), ([Resource.CHASSIS], Reason.UNSUPPORTED)):
        with pytest.raises(MotionRejected) as e:
            core.lease(res, "t")
        assert e.value.reason is why
    with pytest.raises(MotionRejected) as e:
        core.lease([R], "t", heartbeat_ms=300)
    assert e.value.reason is Reason.UNSUPPORTED
    cmd.ok = False
    with pytest.raises(MotionRejected) as e:
        core.lease([R], "t")
    assert e.value.reason is Reason.NOT_READY
    cmd.ok = True
    loop.enable_teach("right")
    with pytest.raises(MotionRejected) as e:
        core.lease([R], "t")
    assert e.value.reason is Reason.BUSY
    loop.disable_teach("right")
    core.estop()
    with pytest.raises(MotionRejected) as e:
        core.lease([R], "t")
    assert e.value.reason is Reason.ESTOP_LATCHED


def test_two_arm_lease_is_all_or_nothing(rig, core):
    _, _, _, loop = rig
    with core.lease([L], "holder"):
        with pytest.raises(MotionRejected):
            core.lease([R, L], "both")
        assert loop.owner("right") is None  # 先拿到的右手已退回
    with core.lease([R, L], "both") as lb:
        assert loop.owner("right") == loop.owner("left") == lb.owner
        with pytest.raises(ValueError):
            lb.move_to(_cmd(loop))  # 两只手要写明资源
        lb.goal(R, [_cmd(loop)])


# ---------------------------------------------------------------- goal / 撤销

def test_goal_moves_and_reports_clamping(rig, core):
    _, _, _, loop = rig
    with core.lease([R], "t") as lr:
        tgt = _away(loop, d=-0.05)
        ack = lr.move_to(tgt)
        assert ack.verdict is Verdict.ACCEPTED and ack.seq == 1
        assert _desired(loop) == pytest.approx(tgt)
        far = tgt.copy()
        far[0] = 100.0
        ack = lr.move_to(far)
        assert ack.verdict is Verdict.CLAMPED and ack.reason is Reason.LIMIT
        assert _desired(loop)[0] == pytest.approx(loop.limits("right")[0, 1])
        with pytest.raises(MotionRejected) as e:
            lr.move_to([float("nan")] * 7)
        assert e.value.reason is Reason.INVALID
        with pytest.raises(MotionRejected) as e:
            lr.move_to([0.0] * 3)
        assert e.value.reason is Reason.INVALID
        with pytest.raises(MotionRejected) as e:
            lr.goal(R, [tgt, tgt])
        assert e.value.reason is Reason.UNSUPPORTED
        with pytest.raises(MotionRejected) as e:
            lr.goal(L, [tgt])  # 租约里没有左手
        assert e.value.reason is Reason.INVALID


@pytest.mark.parametrize("how", ["estop", "engage", "release", "revoke"])
def test_revoke_bumps_epoch_and_stale_commands_change_nothing(rig, core, how):
    _, _, _, loop = rig
    lr = core.lease([R], "t")
    lr.move_to(_away(loop))
    time.sleep(0.1)
    e0 = core.epoch
    {"estop": core.estop, "engage": core.engage, "release": core.release,
     "revoke": lambda: core.revoke("测试")}[how]()
    assert core.epoch == e0 + 1
    assert lr.cancelled.is_set() and loop.owner("right") is None
    if how == "release":
        core.engage()
    if how == "estop":
        core.reset_estop()
    held = _desired(loop)
    with pytest.raises(LeaseRevoked) as e:
        lr.move_to(_away(loop, d=-0.2))
    assert e.value.reason is Reason.STALE_EPOCH
    with pytest.raises(LeaseRevoked):
        lr.check()
    assert lr.hold().verdict is Verdict.REJECTED  # 失效租约的 hold 什么都不做
    assert _desired(loop) == pytest.approx(held)
    lr.release()  # 归还失效租约不出错、不影响新租约
    with core.lease([R], "t2"):
        pass


def test_revoked_arm_stops_at_the_commanded_angle(rig, core):
    _, _, _, loop = rig
    lr = core.lease([R], "t")
    lr.move_to(_away(loop))
    time.sleep(0.15)
    core.revoke("测试")
    c = _cmd(loop)
    assert _desired(loop) == pytest.approx(c, abs=0.01)
    time.sleep(0.2)
    assert _cmd(loop) == pytest.approx(c, abs=0.01)


def test_goal_racing_estop_never_lands_after_it(rig, core):
    """任务线程不停下 goal 时急停：急停返回后再没有任务目标生效，任务收到的是撤销 / 锁存。"""
    _, _, _, loop = rig
    lr = core.lease([R], "hammer")
    base = _cmd(loop).copy()
    stop = threading.Event()
    errors = []

    def hammer():
        k = 0
        while not stop.is_set():
            q = base.copy()
            q[3] -= 0.1 if k % 2 else 0.2
            k += 1
            try:
                lr.move_to(q)
            except MotionRejected as exc:
                errors.append(exc.reason)

    th = threading.Thread(target=hammer, daemon=True)
    th.start()
    time.sleep(0.1)
    core.estop("测试")
    held = _desired(loop)
    time.sleep(0.3)
    stop.set()
    th.join(timeout=2)
    assert _desired(loop) == pytest.approx(held)
    assert errors and set(errors) <= {Reason.STALE_EPOCH, Reason.ESTOP_LATCHED}


def test_cancel_stops_own_commands_but_keeps_the_arm(rig, core):
    _, _, _, loop = rig
    lr = core.lease([R], "t")
    lr.cancel()
    assert lr.cancelled.is_set()
    with pytest.raises(LeaseRevoked) as e:
        lr.move_to(_cmd(loop))
    assert e.value.reason is Reason.CANCELLED
    # 还占着这只手（别人拿不到），自己还能停住它
    with pytest.raises(MotionRejected):
        core.lease([R], "other")
    assert lr.hold().ok
    lr.release()
    assert loop.owner("right") is None


# ---------------------------------------------------------------- 急停锁存

def test_estop_latch_blocks_everything_until_reset(rig, core):
    profile, _, rec, loop = rig
    lr = core.lease([R], "t")
    lr.move_to(_away(loop))
    time.sleep(0.1)
    t0 = time.monotonic()
    core.estop("页面急停")
    assert core.latched and "页面急停" in core.latched
    assert core.safety() is SafetyState.ESTOP_LATCHED
    st = core.status()
    assert st["latched"] and st["leases"] == [] and st["last_revoke"]["owners"] == [lr.owner]
    time.sleep(0.3)
    assert rec.max_gap_since(t0) < 0.1, "急停锁存期间不能停发"
    with pytest.raises(MotionRejected) as e:
        core.lease([L], "t2")
    assert e.value.reason is Reason.ESTOP_LATCHED
    with pytest.raises(PermissionErrorMotion, match="急停已锁定"):
        loop.set_target(_cmd(loop, "left"), arm="left")
    # 重新接管不解除锁存
    core.engage()
    assert core.latched and core.safety() is SafetyState.ESTOP_LATCHED
    with pytest.raises(PermissionErrorMotion, match="急停已锁定"):
        loop.set_target(_cmd(loop, "left"), arm="left")
    core.reset_estop()
    assert core.latched is None and core.safety() is SafetyState.NORMAL
    with core.lease([L], "t3") as l3:
        l3.move_to(_cmd(loop, "left"))


# ---------------------------------------------------------------- 指令出口失去就绪（守护层急停 / 失联）

def test_output_losing_ready_revokes_leases_and_drops_their_targets(rig, core):
    _, _, cmd, loop = rig
    lr = core.lease([R], "t")
    lr.move_to(_away(loop))
    time.sleep(0.1)
    cmd.ok = False
    assert lr.cancelled.wait(1.0), "指令出口没就绪后租约应被撤销"
    assert "没就绪" in core.status()["last_revoke"]["reason"]
    cmd.ok = True  # 就绪恢复后不能接着去追被撤销任务的目标
    time.sleep(0.1)
    c = _cmd(loop)
    time.sleep(0.2)
    assert _cmd(loop) == pytest.approx(c, abs=0.01)
    assert _desired(loop) == pytest.approx(c, abs=0.01)


def test_safety_state_from_guardian_info(rig, core):
    _, _, cmd, loop = rig
    assert core.safety() is SafetyState.NORMAL
    cmd.info_state = {"kind": "guardian", "state": "estop", "latched": ["estop"]}
    assert core.safety() is SafetyState.ESTOP_LATCHED
    cmd.info_state = {"kind": "guardian", "state": "fault", "latched": ["link_lost"]}
    assert core.safety() is SafetyState.LINK_LOST
    cmd.info_state = {"kind": "guardian", "state": "fault", "latched": ["stale_lowstate"]}
    assert core.safety() is SafetyState.STALE_LOWSTATE
    cmd.info_state = None
    cmd.ok = False
    assert core.safety() is SafetyState.HOLD
    cmd.ok = True
    core.release()
    assert core.safety() is SafetyState.RELEASED


# ---------------------------------------------------------------- stream / 有效期（假时钟）

def test_stream_expiry_holds_at_command_and_refresh_keeps_going(rig, fake):
    _, _, _, loop = rig
    core, clock = fake
    lr = core.lease([R], "visual")
    lr.stream(R, _away(loop), valid_for_ms=200)
    time.sleep(0.1)
    clock.advance_ms(150)
    core._on_cycle(True, True)
    assert not np.allclose(_desired(loop), _cmd(loop), atol=1e-4), "还没过期，不应停"
    lr.stream(R, _away(loop, d=-0.25), valid_for_ms=200)  # 刷新：从收到时重新计时
    clock.advance_ms(150)
    core._on_cycle(True, True)
    assert not np.allclose(_desired(loop), _cmd(loop), atol=1e-4)
    clock.advance_ms(100)
    core._on_cycle(True, True)
    c = _cmd(loop)
    assert _desired(loop) == pytest.approx(c, abs=0.01), "过期后停在指令角"
    assert lr.stream(R, _away(loop), valid_for_ms=200).ok  # 新的有效目标到来自动继续


def test_goal_timeout_holds_but_goal_without_timeout_does_not(rig, fake):
    _, _, _, loop = rig
    core, clock = fake
    lr = core.lease([R], "t")
    tgt = _away(loop)
    lr.move_to(tgt)
    clock.advance_ms(60_000)
    core._on_cycle(True, True)
    assert _desired(loop) == pytest.approx(tgt)
    lr.move_to(tgt, timeout_ms=500)
    clock.advance_ms(600)
    core._on_cycle(True, True)
    assert _desired(loop) == pytest.approx(_cmd(loop), abs=0.01)


def test_returning_a_lease_stops_an_unexpired_stream(rig, fake):
    _, _, _, loop = rig
    core, _ = fake
    lr = core.lease([R], "visual")
    lr.stream(R, _away(loop), valid_for_ms=5000)
    time.sleep(0.05)
    lr.release()
    assert _desired(loop) == pytest.approx(_cmd(loop), abs=0.01)


def test_stream_rejects_speed_scale_and_foreign_or_old_observations(rig):
    _, _, _, loop = rig
    clock = FakeClock()
    core = MotionCore(loop, clock_ns=clock, install=False, max_observation_age_ms=150)
    core.engage()
    lr = core.lease([R], "visual")
    q = _cmd(loop)
    with pytest.raises(MotionRejected) as e:
        lr.stream(R, q, speed_scale=0.5)
    assert e.value.reason is Reason.UNSUPPORTED
    with pytest.raises(MotionRejected) as e:
        lr.stream(R, q, derived_from=Stamp(t_ns=clock.t, host="body"))
    assert e.value.reason is Reason.STALE_OBSERVATION
    with pytest.raises(MotionRejected) as e:
        lr.stream(R, q, derived_from=Stamp(t_ns=clock.t - 200_000_000, host="local"))
    assert e.value.reason is Reason.STALE_OBSERVATION
    assert lr.stream(R, q, derived_from=Stamp(t_ns=clock.t - 100_000_000, host="local")).ok


# ---------------------------------------------------------------- state_at

def test_state_at_interpolates_history_and_rejects_bad_stamps(rig, fake):
    _, _, _, loop = rig
    core, clock = fake
    t = clock.t
    z = (0.0,) * 7
    core._push(_Sample(t_meas=t - 100_000_000, t_cmd=t - 100_000_000,
                       q_meas={"right": z, "left": z}, q_cmd={"right": z, "left": z}))
    one = (1.0,) * 7
    core._push(_Sample(t_meas=t, t_cmd=t, q_meas={"right": one, "left": z}, q_cmd={"right": (2.0,) * 7, "left": z}))
    st = core.state_at(R, Stamp(t_ns=t - 25_000_000, host="local"))
    assert st.q_meas == pytest.approx((0.75,) * 7) and st.q_cmd == pytest.approx((1.5,) * 7)
    assert st.resource is R and st.epoch == core.epoch
    with pytest.raises(MotionRejected) as e:
        core.state_at(R, Stamp(t_ns=t - 1_000_000_000, host="local"))
    assert e.value.reason is Reason.STALE_OBSERVATION
    with pytest.raises(MotionRejected) as e:
        core.state_at(R, Stamp(t_ns=t + 1_000_000_000, host="local"))
    assert e.value.reason is Reason.INVALID
    with pytest.raises(MotionRejected) as e:
        core.state_at(R, Stamp(t_ns=t, host="body"))
    assert e.value.reason is Reason.INVALID


def test_history_is_recorded_by_the_loop_and_pruned(rig):
    """装进控制循环后每拍记一条；只保留 history_s。"""
    _, _, _, loop = rig
    core = MotionCore(loop, history_s=0.3)
    time.sleep(0.6)
    with core._hist_lock:
        hist = list(core._hist)
    span = (hist[-1].t_cmd - hist[0].t_cmd) / 1e9
    assert len(hist) > 10 and span <= 0.31
    now = time.monotonic_ns()
    st = core.state_at(R, Stamp(t_ns=now - 100_000_000, host="local"))
    meas = loop.arm_sample("right")["q_meas"]
    assert st.q_meas == pytest.approx(meas, abs=1e-3)
    assert core.state(R).q_cmd == pytest.approx(_cmd(loop).tolist())
