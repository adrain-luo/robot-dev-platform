"""lowstate 过期门控（ArmLoop(lowstate_max_age_s=…)，直写后端用）。

已接管时 lowstate 超过上限没更新：不再跟目标、两只手停在当前指令角（拖拽退出、恢复全刚度）、
撤销占用、拒绝新运动，帧一直照发；恢复后照旧保持，不自己接着走。
真的 ArmLoop + mock 机器人 + 记帧的指令出口；lowstate「停更」用一个能冻结年龄的读状态包装。
"""

import threading
import time

import numpy as np
import pytest

from g1_contracts.motion import MotionRejected, Reason, SafetyState
from robot_platform.control.arm_loop import ArmLoop, PermissionErrorMotion
from robot_platform.control.fk_scene import RobotKinematics
from robot_platform.control.motion_core import MotionCore
from robot_platform.core.robot_profile import active
from robot_platform.hw.backends.base import StateSource
from robot_platform.hw.backends.mock import MockBackend
from test_arm_loop_frame import Recorder

LIMIT_S = 0.1


class FreezableState(StateSource):
    """mock 的读状态，可以让 age_s() 从某一刻开始一直涨（模拟 lowstate 停更），也可以让它抛异常。"""

    def __init__(self, inner: StateSource) -> None:
        self.inner = inner
        self._frozen_at = None
        self.raise_on_age = False
        self._lock = threading.Lock()

    def freeze(self) -> None:
        with self._lock:
            self._frozen_at = time.monotonic()

    def thaw(self) -> None:
        with self._lock:
            self._frozen_at = None

    def age_s(self) -> float:
        if self.raise_on_age:
            raise RuntimeError("读不到 lowstate 年龄")
        with self._lock:
            t = self._frozen_at
        return self.inner.age_s() if t is None else time.monotonic() - t

    def wait_for_message(self, timeout_s=None):
        return self.inner.wait_for_message(timeout_s)

    def read_arm(self, arm, *, wait=True):
        return self.inner.read_arm(arm, wait=wait)

    def read_motors(self, motor_indices):
        return self.inner.read_motors(motor_indices)

    def read_motor_states(self, motor_indices):
        return self.inner.read_motor_states(motor_indices)

    def snapshot(self):
        return self.inner.snapshot()


def _make(limit=LIMIT_S):
    profile = active()
    backend = MockBackend(profile)
    reader = FreezableState(backend.state())
    rec = Recorder()
    kin = RobotKinematics(profile.urdf, profile.base_link, list(profile.arms))
    loop = ArmLoop(reader=reader, commander=rec, kin=kin, urdf_path=profile.urdf,
                   base_link=profile.base_link, lowstate_max_age_s=limit)
    return profile, backend, reader, rec, loop


@pytest.fixture
def rig():
    profile, backend, reader, rec, loop = _make()
    yield profile, reader, rec, loop
    loop.shutdown()
    backend.close()


def _cmd(loop, arm):
    return np.asarray(loop.status()["arms"][arm]["cmd_rad"])


def _wait(pred, what, timeout_s=1.0):
    deadline = time.monotonic() + timeout_s
    while not pred():
        assert time.monotonic() < deadline, what
        time.sleep(0.005)


def _wait_stale(loop):
    _wait(lambda: loop.status()["state_stale"] is not None, "lowstate 停更后没有进入过期保持")


def _moving_target(loop, arm="right", dq=-0.3):
    tgt = _cmd(loop, arm).copy()
    tgt[3] += dq
    loop.set_target(tgt, arm=arm)
    return tgt


def test_stale_lowstate_holds_at_command_with_frames_and_full_stiffness(rig):
    profile, reader, rec, loop = rig
    loop.engage()
    loop.set_max_speed(0.4)
    _moving_target(loop)
    time.sleep(0.1)
    t0 = time.monotonic()
    reader.freeze()
    _wait_stale(loop)
    held = _cmd(loop, "right")
    time.sleep(0.4)
    assert rec.max_gap_since(t0) < 0.1, "lowstate 过期后不能停发"
    assert _cmd(loop, "right") == pytest.approx(held), "过期期间指令角不能再往目标走"
    st = loop.status()
    assert st["arms"]["right"]["desired_rad"] == pytest.approx(held.tolist()), "没走完的目标要取消"
    assert st["ready"] is False and "lowstate" in st["state_stale"]
    assert st["lowstate_max_age_ms"] == 100
    _, kp, kd, _, _ = rec.last()
    for name in profile.arms:
        for i in profile.arm(name).motor_indices:
            assert (kp[i], kd[i]) == (profile.kp[i], profile.kd[i]), f"电机 {i} 要全刚度"


def test_stale_lowstate_ends_teach_at_the_last_read_angle(rig):
    profile, reader, rec, loop = rig
    loop.engage()
    loop.enable_teach("left")
    left = profile.arm("left").motor_indices
    _wait(lambda: rec.count() > 0 and rec.last()[1][left[0]] == 0.0, "拖拽时左臂刚度应为 0")
    measured = np.asarray([reader.snapshot().q[i] for i in left])
    reader.freeze()
    _wait_stale(loop)
    assert loop.status()["arms"]["left"]["float"] is False
    assert _cmd(loop, "left") == pytest.approx(measured, abs=1e-6), "锁在最后读到的角度，不跳回拖拽前的指令"
    _wait(lambda: rec.last()[1][left[0]] == profile.kp[left[0]], "退出拖拽后要恢复全刚度")
    q, kp, kd, _, _ = rec.last()
    for k, i in enumerate(left):
        assert (kp[i], kd[i]) == (profile.kp[i], profile.kd[i])
        assert q[i] == pytest.approx(measured[k], abs=1e-6)


def test_stale_lowstate_refuses_new_motion_and_clears_owners(rig):
    _, reader, _, loop = rig
    loop.engage()
    loop.acquire_owner("right", "任务")
    reader.freeze()
    _wait_stale(loop)
    assert loop.owner("right") is None, "过期时撤销占用"
    r = _cmd(loop, "right")
    with pytest.raises(PermissionErrorMotion, match="lowstate"):
        loop.set_target(r, arm="right")
    with pytest.raises(PermissionErrorMotion, match="lowstate"):
        loop.set_target(r, arm="right", owner="任务")
    with pytest.raises(PermissionErrorMotion, match="lowstate"):
        loop.acquire_owner("right", "任务")
    with pytest.raises(PermissionErrorMotion, match="lowstate"):
        loop.enable_teach("left")
    with pytest.raises(PermissionErrorMotion, match="lowstate"):
        loop.set_waist(yaw_rad=0.0)
    # 停下类的操作照常
    loop.hold_arm("right")
    loop.hold_at_command("left")
    loop.hold_waist()
    loop.disable_teach()
    loop.estop_hold()


def test_recovered_lowstate_keeps_holding_until_a_new_target(rig):
    _, reader, _, loop = rig
    loop.engage()
    loop.set_max_speed(0.4)
    _moving_target(loop)
    time.sleep(0.1)
    reader.freeze()
    _wait_stale(loop)
    held = _cmd(loop, "right")
    reader.thaw()
    _wait(lambda: loop.status()["state_stale"] is None, "lowstate 恢复后过期标记要清掉")
    time.sleep(0.3)
    assert _cmd(loop, "right") == pytest.approx(held), "恢复后不能自己接着走"
    assert loop.status()["ready"] is True
    tgt = _moving_target(loop, dq=0.1)
    _wait(lambda: float(np.max(np.abs(_cmd(loop, "right") - tgt))) < 1e-9, "恢复后新目标要能走到", 3.0)


def test_engage_is_refused_while_lowstate_is_stale(rig):
    _, reader, rec, loop = rig
    reader.freeze()
    time.sleep(LIMIT_S + 0.05)
    with pytest.raises(PermissionErrorMotion, match="不能接管"):
        loop.engage()
    assert loop.engaged is False
    assert rec.count() == 0 and rec.prepared == 0
    reader.thaw()
    loop.engage()
    assert loop.engaged is True


def test_unreadable_age_counts_as_stale_and_frames_keep_going(rig):
    _, reader, rec, loop = rig
    loop.engage()
    t0 = time.monotonic()
    reader.raise_on_age = True
    _wait(lambda: loop.status()["state_stale"] is not None, "读不到年龄要按过期处理")
    assert "没有收到 lowstate" in loop.status()["state_stale"]
    time.sleep(0.3)
    assert rec.max_gap_since(t0) < 0.1


def test_without_a_limit_there_is_no_gate():
    """守护层后端、mock 不传上限：行为和以前一样。"""
    profile, backend, reader, rec, loop = _make(limit=None)
    try:
        loop.engage()
        loop.set_max_speed(0.4)
        reader.freeze()
        time.sleep(0.2)
        tgt = _moving_target(loop, dq=-0.1)
        _wait(lambda: float(np.max(np.abs(_cmd(loop, "right") - tgt))) < 1e-9, "不检查时目标照常走", 3.0)
        st = loop.status()
        assert st["state_stale"] is None and st["lowstate_max_age_ms"] is None and st["ready"] is True
    finally:
        loop.shutdown()
        backend.close()


@pytest.mark.parametrize("bad", [0.0, -0.1, float("nan"), float("inf")])
def test_limit_must_be_positive(bad):
    profile = active()
    backend = MockBackend(profile)
    kin = RobotKinematics(profile.urdf, profile.base_link, list(profile.arms))
    try:
        with pytest.raises(ValueError):
            ArmLoop(reader=backend.state(), commander=Recorder(), kin=kin, urdf_path=profile.urdf,
                    base_link=profile.base_link, lowstate_max_age_s=bad)
    finally:
        backend.close()


def test_motion_core_revokes_leases_and_reports_stale_lowstate(rig):
    _, reader, _, loop = rig
    core = MotionCore(loop)
    core.engage()
    lease = core.lease_arm("right", "任务")
    assert core.safety() is SafetyState.NORMAL
    reader.freeze()
    assert lease.cancelled.wait(1.0), "lowstate 过期要撤销租约"
    assert core.safety() is SafetyState.STALE_LOWSTATE
    assert "lowstate" in core.status()["last_revoke"]["reason"]
    with pytest.raises(MotionRejected) as exc:
        core.lease_arm("right", "任务")
    assert exc.value.reason is Reason.NOT_READY and "lowstate" in str(exc.value)
    reader.thaw()
    _wait(lambda: core.safety() is SafetyState.NORMAL, "lowstate 恢复后安全状态回到 normal")
    with core.lease_arm("right", "任务2") as again:
        assert not again.cancelled.is_set()
