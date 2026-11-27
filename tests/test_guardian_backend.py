"""平台经守护层发令：ArmLoop → GuardianCommander → (UDP) → GuardianService → 仿真机器人。

守护层是进程内真的 GuardianService（真 UDP），它的输出接到 mock 后端的电机模型上，
所以这里看到的是整条链路：接管、走点、拖拽、平台卡住 / 失联、守护层急停与复位、释放。
"""

import json
import math
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from g1_guardian.client import GuardianClient
from g1_guardian.config import builtin_config_path, config_from_dict, load_config
from g1_guardian.core import LowStateView
from g1_guardian.service import GuardianService
from robot_platform.control.arm_loop import TAU_FF_LIMIT, TEACH_KD, ArmLoop, PermissionErrorMotion
from robot_platform.control.fk_scene import RobotKinematics
from robot_platform.core.robot_profile import active, load_profile
from robot_platform.hw.backends import create_backend
from robot_platform.hw.backends.guardian import (
    GuardianBackend,
    GuardianCommander,
    GuardianSettings,
    engage_refusal_text,
    guardian_settings,
    lost_text,
)
from robot_platform.hw.backends.mock import MockBackend

RELEASE_S = 0.5
MAX_GAP_S = 0.040  # 守护层 200 Hz；CI 机器上留余量
MAX_GAP_ESTOP_S = 0.100  # 跨守护层锁存（含黑匣子落盘）时；见用到它的测试


def _cfg(name="g1d", **timing):
    d = json.loads(builtin_config_path(name).read_text())
    d["timing"].update({"release_ramp_s": RELEASE_S, **timing})
    return config_from_dict(d)


class Plant:
    """守护层的输出和 lowstate，接到 mock 机器人的电机模型上（kp > 0 跟随指令角，kp = 0 不动）。"""

    def __init__(self, mock: MockBackend):
        self.robot = mock.robot
        self.prepared = False
        self.prepare_calls = 0
        self.lock = threading.Lock()
        self.frames = []  # (t, q, kp, kd, tau)
        self.foreign_seen_at = -math.inf

    def prepare(self):
        self.prepare_calls += 1
        self.prepared = True

    def write(self, frame):
        self.robot.command(frame.q, frame.kp, frame.kd, frame.tau)
        with self.lock:
            self.frames.append((time.monotonic(), list(frame.q), list(frame.kp), list(frame.kd), list(frame.tau)))

    def latest(self):
        with self.robot._lock:
            q = list(self.robot.q)
        return LowStateView(q=q, stamp=time.monotonic())

    def since(self, t0):
        with self.lock:
            return [f for f in self.frames if f[0] >= t0]

    def last(self, timeout_s=1.0):
        """最近一帧；刚接管时守护层可能还没发出第一帧，等一下。"""
        deadline = time.monotonic() + timeout_s
        while True:
            with self.lock:
                if self.frames:
                    return self.frames[-1]
            assert time.monotonic() < deadline, "守护层没有发帧"
            time.sleep(0.005)

    def drag(self, motor, delta, seconds=0.3, steps=30):
        for _ in range(steps):
            with self.robot._lock:
                self.robot.q[motor] += delta / steps
            time.sleep(seconds / steps)


def max_gap(frames, t_from, t_to):
    ts = [t_from] + [f[0] for f in frames if t_from <= f[0] <= t_to] + [t_to]
    return max(b - a for a, b in zip(ts, ts[1:]))


def wait_until(pred, timeout_s=3.0, what="条件"):
    deadline = time.monotonic() + timeout_s
    while not pred():
        assert time.monotonic() < deadline, f"等不到：{what}"
        time.sleep(0.01)


@pytest.fixture
def g(tmp_path):
    profile = active()
    mock = MockBackend(profile)
    mock.commander = lambda: pytest.fail("经守护层时本进程不该创建 lowcmd 写入者")
    plant = Plant(mock)
    svc = GuardianService(_cfg(), plant, plant, bind=("127.0.0.1", 0), blackbox_dir=tmp_path / "bb", log=lambda s: None)
    th = threading.Thread(target=svc.run, name="guardian_loop", daemon=True)
    th.start()
    settings = GuardianSettings(host="127.0.0.1", port=svc.address[1], client_id="robot-platform", release_s=RELEASE_S)
    backend = GuardianBackend(profile, mock, settings)
    kin = RobotKinematics(profile.urdf, profile.base_link, list(profile.arms))
    loop = ArmLoop(reader=backend.state(), commander=backend.commander(), kin=kin,
                   urdf_path=profile.urdf, base_link=profile.base_link, max_speed_rad_s=1.0)
    others = []

    def other(cid):
        c = GuardianClient("127.0.0.1", svc.address[1], client_id=cid)
        others.append(c)
        return c

    yield SimpleNamespace(profile=profile, mock=mock, plant=plant, svc=svc, backend=backend, loop=loop,
                          other=other, right=profile.arms["right"].motor_indices,
                          left=profile.arms["left"].motor_indices)
    try:
        loop.shutdown(timeout_s=RELEASE_S + 3.0)
    except PermissionErrorMotion:
        pass
    for c in others:
        c.close()
    svc.stop()
    th.join(2.0)
    svc.close()
    backend.close()


def _meas(g, motors):
    with g.mock.robot._lock:
        return np.array([g.mock.robot.q[m] for m in motors])


def _gstate(g):
    return g.svc.core.state


# ------------------------------------------------------------------ 整条链路
def test_engage_then_move_through_the_guardian(g):
    q0 = _meas(g, g.right)
    g.loop.engage()
    assert _gstate(g) == "holding" and g.svc.core.owner == "robot-platform"
    assert g.plant.prepare_calls == 1, "退出运控由守护层做"
    st = g.loop.status()
    assert st["ready"] is True and st["transition"] == ""
    assert st["commander"]["kind"] == "guardian" and st["commander"]["ours"] is True
    # 接管不动
    time.sleep(0.2)
    assert np.max(np.abs(_meas(g, g.right) - q0)) < 2e-3

    tgt = q0.copy()
    tgt[3] += 0.25
    g.loop.set_target(tgt.tolist(), arm="right")
    wait_until(lambda: np.max(np.abs(_meas(g, g.right) - tgt)) < 5e-3, 4.0, "右臂走到目标")
    assert g.svc.core.state in ("tracking", "holding")
    last = g.plant.last()
    kp, kd, tau = last[2], last[3], last[4]
    for m in g.right + g.left:
        assert kp[m] == pytest.approx(g.profile.kp[m])
        assert kd[m] == pytest.approx(g.profile.kd[m])
    # 腰按平台的锁腰增益保持
    for m in g.profile.waist.hold_motors:
        assert (kp[m], kd[m]) == (g.profile.waist.lock_kp, g.profile.waist.lock_kd)
    # 重力前馈原样到了守护层的帧里（钳在同一个上限内）
    sent = g.loop.arm_sample("right")["tau_ff_sent"]
    assert np.max(np.abs(sent)) > 0.1
    assert [tau[m] for m in g.right] == pytest.approx(sent, abs=0.3)
    assert max(abs(tau[m]) for m in g.right) <= TAU_FF_LIMIT + 1e-9
    # 页面胶囊：守护层替本程序发帧，没有别人
    bus = g.backend.cmd_monitor().report()
    assert bus["foreign"] is False and bus["guardian_state"] in ("tracking", "holding") and bus["own_hz"] > 100


def test_teach_through_the_guardian_holds_where_the_hand_left_it(g):
    g.loop.engage()
    g.loop.enable_teach("right")
    wait_until(lambda: g.plant.last()[2][g.right[0]] == 0.0, 2.0, "右臂刚度降到 0")
    last = g.plant.last()
    assert last[3][g.right[0]] == pytest.approx(TEACH_KD)
    assert last[2][g.left[0]] == pytest.approx(g.profile.kp[g.left[0]]), "左臂照常受控"
    assert g.loop.status()["commander"]["problem"] is None

    # 手快速推肩 pitch：约 6 rad/s，远超守护层跟踪限速；守护层的指令角必须贴着实测走，不能落后
    g.plant.drag(g.right[0], -0.9, seconds=0.15, steps=15)
    time.sleep(0.03)
    where = _meas(g, g.right)
    assert np.array(g.svc.core.q_cmd["right_arm"]) == pytest.approx(where, abs=2e-3)
    g.loop.disable_teach("right")
    wait_until(lambda: g.plant.last()[2][g.right[0]] == pytest.approx(g.profile.kp[g.right[0]]), 2.0, "刚度回来")
    time.sleep(0.3)
    # 没有弹回拖拽前的姿态
    assert np.max(np.abs(_meas(g, g.right) - where)) < 5e-3
    assert not [r for r in g.svc.core.rejects if r.startswith("target_jump")]


def test_platform_stall_makes_the_guardian_hold_and_it_resumes(g):
    g.loop.engage()
    q0 = _meas(g, g.right)
    tgt = q0.copy()
    tgt[0] -= 0.6
    g.loop.set_target(tgt.tolist(), arm="right")
    time.sleep(0.15)
    t0 = time.monotonic()
    with g.loop._publish_lock:  # 平台循环卡住 0.35 s：设定过期，守护层减速保持
        wait_until(lambda: _gstate(g) == "holding", 0.3, "守护层转为保持")
        time.sleep(0.2)
    t1 = time.monotonic()
    assert max_gap(g.plant.since(t0), t0, t1) <= MAX_GAP_S, "卡住期间守护层照常发帧"
    wait_until(lambda: np.max(np.abs(_meas(g, g.right) - tgt)) < 5e-3, 4.0, "恢复后继续走到目标")
    assert g.loop.status()["ready"] is True


def test_platform_link_lost_holds_and_needs_a_new_engage(g):
    g.loop.engage()
    g.backend.client().stop_heartbeat()
    t0 = time.monotonic()
    with g.loop._publish_lock:  # 进程整个卡死：设定和心跳都没了
        wait_until(lambda: "link_lost" in g.svc.core.latched, 1.5, "守护层锁存 link_lost")
    t1 = time.monotonic()
    assert max_gap(g.plant.since(t0), t0, t1) <= MAX_GAP_S
    wait_until(lambda: g.loop.status()["ready"] is False, 1.0, "平台知道失控")
    st = g.loop.status()
    assert "失联" in st["commander"]["problem"] and "失联" in st["transition"]
    with pytest.raises(PermissionErrorMotion):
        g.loop.set_target(_meas(g, g.right).tolist(), arm="right")
    held = _meas(g, g.right)
    g.loop.engage()  # 重新接管：从实测角开始，不动
    time.sleep(0.2)
    assert g.loop.status()["ready"] is True
    assert np.max(np.abs(_meas(g, g.right) - held)) < 2e-3


def test_guardian_estop_freezes_the_loop_and_reset_reengages_without_motion(g):
    g.loop.engage()
    q0 = _meas(g, g.right)
    tgt = q0.copy()
    tgt[3] += 0.5
    g.loop.set_target(tgt.tolist(), arm="right")
    time.sleep(0.2)
    g.other("panel").estop()
    wait_until(lambda: g.loop.status()["ready"] is False, 1.0, "平台看到守护层急停")
    st = g.loop.status()
    assert "急停" in st["commander"]["problem"]
    # 平台不再往前走指令角（重新接管会从实测角开始）
    cmd1 = np.array(st["arms"]["right"]["cmd_rad"])
    time.sleep(0.2)
    assert np.array(g.loop.status()["arms"]["right"]["cmd_rad"]) == pytest.approx(cmd1)
    assert _gstate(g) == "estop"
    held = _meas(g, g.right)
    assert np.max(np.abs(held - tgt)) > 0.05, "急停时手臂停在半路"

    with pytest.raises(PermissionErrorMotion, match="复位"):
        g.loop.engage()
    g.backend.reset_guardian()
    t_engage = time.monotonic()
    g.loop.engage()
    time.sleep(0.3)
    assert g.loop.status()["ready"] is True
    # 复位、重新接管都不动；急停前的目标已作废
    after = g.plant.since(t_engage)
    assert after and max(np.max(np.abs(np.array([f[1][m] for m in g.right]) - held)) for f in after) < 5e-3
    assert np.max(np.abs(_meas(g, g.right) - held)) < 5e-3


def test_guardian_estop_revokes_motion_core_leases_and_latches_stay_separate(g):
    """守护层急停：运控核心立刻撤销租约（任务马上停，不用等到位超时）。
    两处锁存互不解除：核心复位不清守护层，守护层复位不清核心；页面急停不锁守护层。"""
    from g1_contracts.motion import LeaseRevoked, Resource, SafetyState
    from robot_platform.control.motion_core import MotionCore

    core = MotionCore(g.loop)
    core.engage()
    lease = core.lease([Resource.RIGHT_ARM], "task:test")
    tgt = _meas(g, g.right)
    tgt[3] += 0.5
    lease.move_to(tgt.tolist())
    time.sleep(0.2)
    g.other("panel").estop()
    assert lease.cancelled.wait(1.0), "守护层急停后租约应被撤销"
    assert core.safety() is SafetyState.ESTOP_LATCHED and core.latched is None
    with pytest.raises(LeaseRevoked):
        lease.move_to(tgt.tolist())
    core.reset_estop()
    with pytest.raises(PermissionErrorMotion, match="复位"):
        core.engage()
    g.backend.reset_guardian()
    core.engage()
    wait_until(lambda: g.loop.status()["ready"], 1.0, "重新接管就绪")

    core.estop("页面急停")
    time.sleep(0.1)
    assert _gstate(g) in ("holding", "tracking"), "没装外部急停的核心不锁守护层（App 会装上，见下一个测试）"
    g.backend.reset_guardian()
    core.engage()
    assert core.latched and core.safety() is SafetyState.ESTOP_LATCHED
    with pytest.raises(PermissionErrorMotion, match="急停已锁定"):
        g.loop.set_target(tgt.tolist(), arm="right")
    core.reset_estop()
    with core.lease([Resource.RIGHT_ARM], "task:after") as l2:
        l2.move_to(_meas(g, g.right).tolist())


def test_core_estop_also_latches_the_guardian_and_reset_takes_both_steps(g):
    """页面急停（运控核心急停）同时锁守护层：手臂一直有帧、停在半路；复位要两步。"""
    from g1_contracts.motion import LeaseRevoked, Resource
    from robot_platform.control.motion_core import MotionCore

    core = MotionCore(g.loop, external_estop=g.backend.estop_guardian)
    assert core.status()["external_estop"] is True
    core.engage()
    lease = core.lease([Resource.RIGHT_ARM], "task:test")
    tgt = _meas(g, g.right)
    tgt[3] += 0.5
    lease.move_to(tgt.tolist())
    time.sleep(0.2)
    t0 = time.monotonic()
    core.estop("页面急停")
    assert _gstate(g) == "estop", "核心急停返回时守护层已经锁存"
    assert lease.cancelled.is_set() and core.latched
    time.sleep(0.3)
    t1 = time.monotonic()
    # 守护层锁存时会写黑匣子（同进程的写文件线程要抢 GIL），这台测试机还有虚拟机调度抖动：
    # 这里只证明帧没停（停了会是整段 0.3 s），抖动指标以本体实测为准
    assert max_gap(g.plant.since(t0), t0, t1) <= MAX_GAP_ESTOP_S, "急停前后守护层一直发帧"
    held = _meas(g, g.right)
    assert np.max(np.abs(held - tgt)) > 0.05, "停在半路"
    time.sleep(0.2)
    assert np.max(np.abs(_meas(g, g.right) - held)) < 5e-3, "急停后不动"
    with pytest.raises(LeaseRevoked):
        lease.move_to(tgt.tolist())

    # 第一步：复位核心；守护层仍锁着，接管被拒
    core.reset_estop()
    with pytest.raises(PermissionErrorMotion, match="复位"):
        core.engage()
    # 第二步：复位守护层再接管，从实测角开始、不动
    g.backend.reset_guardian()
    core.engage()
    wait_until(lambda: g.loop.status()["ready"], 1.0, "重新接管就绪")
    time.sleep(0.2)
    assert np.max(np.abs(_meas(g, g.right) - held)) < 5e-3
    with core.lease([Resource.RIGHT_ARM], "task:after") as l2:
        l2.move_to(held.tolist())


def test_core_estop_stops_another_program_driving_through_the_guardian(g):
    """平台没接管、别的程序（refactor）经守护层在动手臂：页面急停照样让它停下。"""
    from robot_platform.control.motion_core import MotionCore

    node = g.other("refactor-node")
    epoch = node.engage()
    wait_until(lambda: _gstate(g) == "holding", 1.0, "refactor 接管")
    q = _meas(g, g.right)
    tgt = q.copy()
    tgt[3] += 0.3  # 守护层单次跳变上限 0.5 rad 以内
    stop = threading.Event()

    def stream():
        while not stop.is_set():
            node.setpoint({"right_arm": {"q": tgt.tolist()}}, valid_for_ms=100)
            time.sleep(0.01)

    th = threading.Thread(target=stream, daemon=True)
    th.start()
    try:
        time.sleep(0.2)
        core = MotionCore(g.loop, external_estop=g.backend.estop_guardian)
        assert g.loop.status()["engaged"] is False
        core.estop("页面急停")
        assert _gstate(g) == "estop" and g.svc.core.owner == "refactor-node"
        time.sleep(0.4)  # 守护层按加速度上限减速到停，仿真手臂再跟上
        held = _meas(g, g.right)
        cmd = np.array(g.svc.core.q_cmd["right_arm"])
        time.sleep(0.3)
        assert np.array(g.svc.core.q_cmd["right_arm"]) == pytest.approx(cmd), "refactor 还在发设定，但守护层不再执行"
        assert np.max(np.abs(_meas(g, g.right) - held)) < 5e-3
        assert np.max(np.abs(held - tgt)) > 0.05
        assert g.svc.core.rejects.get("state_estop", 0) > 0, "守护层拒绝了 refactor 的设定"
    finally:
        stop.set()
        th.join(1.0)
    assert g.svc.core.epoch > epoch, "守护层急停作废了 refactor 的 epoch：复位后它要重新接管"


class _DeadGuardianClient:
    """连不上的守护层：每个命令都等不到应答。"""

    def __init__(self):
        self.estops = 0

    def estop(self, timeout=1.0):
        from g1_guardian.client import GuardianUnavailable

        self.estops += 1
        raise GuardianUnavailable("no answer to estop")

    def close(self):
        pass


def test_core_estop_latches_locally_even_when_the_guardian_does_not_answer():
    from g1_contracts.motion import Resource
    from robot_platform.control.motion_core import MotionCore

    profile = active()
    mock = MockBackend(profile)
    dead = _DeadGuardianClient()
    backend = GuardianBackend(profile, mock, GuardianSettings(host="127.0.0.1", port=9),
                              client_factory=lambda s: dead)
    kin = RobotKinematics(profile.urdf, profile.base_link, list(profile.arms))
    loop = ArmLoop(reader=mock.state(), commander=mock.commander(), kin=kin,
                   urdf_path=profile.urdf, base_link=profile.base_link)
    try:
        core = MotionCore(loop, external_estop=backend.estop_guardian)
        core.engage()
        lease = core.lease([Resource.RIGHT_ARM], "task:test")
        with pytest.raises(PermissionErrorMotion, match="守护层没锁上"):
            core.estop("页面急停")
        assert dead.estops == 1
        assert core.latched and lease.cancelled.is_set() and loop.inhibited
        assert loop.status()["engaged"] is True, "手臂照常保持"
    finally:
        loop.shutdown()
        mock.close()


def test_app_wires_the_backend_estop_into_the_core():
    from robot_platform.core.config_loader import load_settings
    from robot_platform.web.app import App

    settings = load_settings(backend="mock")
    profile = settings.profile
    kin = RobotKinematics(settings.urdf, settings.base_link, list(profile.arms))
    for with_guardian in (False, True):
        backend = MockBackend(profile)
        calls = []
        if with_guardian:
            backend.estop_guardian = lambda reason="": calls.append(reason)
        app = App(settings, kin, allow_real_motion=True, backend=backend)
        try:
            app.reader()
            assert app.motion().has_external_estop is with_guardian
            app.motion().estop("页面急停")
            assert calls == (["页面急停"] if with_guardian else [])
        finally:
            app.motion().reset_estop()
            app.loop.shutdown()
            backend.close()


def test_engage_refused_while_another_program_owns_the_guardian(g):
    node = g.other("refactor-node")
    node.engage()
    with pytest.raises(PermissionErrorMotion, match="别的程序"):
        g.loop.engage()
    assert g.svc.core.owner == "refactor-node"
    assert g.loop.status()["engaged"] is False


def test_release_ramps_down_then_frames_stop(g):
    g.loop.engage()
    time.sleep(0.1)
    g.loop.release()
    st = g.loop.status()
    assert st["engaged"] is False and "释放" in st["transition"]
    kp0 = g.profile.kp[g.right[0]]
    t0 = time.monotonic()
    wait_until(lambda: _gstate(g) == "idle", RELEASE_S + 2.0, "守护层释放完")
    ramp = [f[2][g.right[0]] for f in g.plant.since(t0)]
    assert ramp[0] > 0.5 * kp0 and ramp[-1] == 0.0
    assert all(b <= a + 1e-9 for a, b in zip(ramp, ramp[1:])), "刚度单调下降"
    wait_until(lambda: not g.backend.commander().busy(), 1.0, "过渡结束")
    n = len(g.plant.frames)
    time.sleep(0.15)
    assert len(g.plant.frames) == n, "释放完就停发"
    assert g.loop.status()["transition"] == ""


# ------------------------------------------------------------- 单元：Commander
class FakeClient:
    def __init__(self, status):
        self.st = status
        self.sent = []
        self.heartbeat = True
        self.release_error = None

    def ping(self, timeout=1.0):
        return self.st

    def engage(self, timeout=5.0):
        return self.st["epoch"]

    def latest_status(self):
        return self.st

    def status_age(self):
        return 0.01

    def setpoint(self, groups, valid_for_ms=100):
        self.sent.append(groups)

    def release(self, timeout=2.0):
        if self.release_error:
            raise self.release_error

    def stop_heartbeat(self):
        self.heartbeat = False


def _status(profile, **kw):
    st = {
        "robot": profile.name, "channel": "lowcmd", "state": "holding", "latched": [], "epoch": 3,
        "owner": "robot-platform",
        "groups": {f"{a}_arm": {"motors": list(l.motor_indices)} for a, l in profile.arms.items()},
    }
    st["groups"]["waist"] = {"motors": [12]}
    st.update(kw)
    return st


def _commander(st, profile=None):
    profile = profile or active()
    client = FakeClient(st)
    c = GuardianCommander(profile, client, GuardianSettings())
    return c, client


def _frame(profile, kp=None):
    n = profile.num_motors
    q = [0.01 * i for i in range(n)]
    kp = list(profile.kp) if kp is None else kp
    return q, kp, list(profile.kd), [0.1] * n


def test_commander_sends_groups_with_kp_scale_only_after_activate():
    profile = active()
    c, client = _commander(_status(profile))
    c.prepare()
    c.write(*_frame(profile))
    assert client.sent == [], "ArmLoop 重置完指令角（activate）之前不发"
    c.activate()
    kp = list(profile.kp)
    for m in profile.arms["right"].motor_indices:
        kp[m] = 0.0  # 右臂拖拽
    for m in profile.waist.hold_motors:
        kp[m] = profile.waist.lock_kp
    c.write(*_frame(profile, kp))
    body = client.sent[-1]
    assert set(body) == {"right_arm", "left_arm", "waist"}
    assert body["right_arm"]["kp_scale"] == 0.0
    assert body["left_arm"]["kp_scale"] == 1.0
    assert body["waist"]["kp_scale"] == 1.0, "锁腰增益比机型 kp 大：守护层只认更软，钳到 1"
    assert body["right_arm"]["q"] == pytest.approx([0.01 * m for m in profile.arms["right"].motor_indices])
    assert body["waist"]["q"] == pytest.approx([0.12])
    assert c.ready() and c.phase() == ""


@pytest.mark.parametrize("change, needle", [
    ({"robot": "g1"}, "配置的是 g1"),
    ({"channel": "arm_sdk"}, "arm_sdk"),
    ({"groups": {"right_arm": {"motors": [22, 23]}, "left_arm": {"motors": list(range(15, 22))}}}, "right_arm"),
    ({"groups": {"right_arm": {"motors": list(range(22, 29))}, "left_arm": {"motors": list(range(15, 22))},
                 "waist": {"motors": [12, 14]}}}, "waist"),
])
def test_commander_refuses_a_guardian_configured_for_something_else(change, needle):
    profile = active()
    c, client = _commander(_status(profile, **change))
    with pytest.raises(PermissionErrorMotion, match=needle):
        c.prepare()
    c.activate()
    c.write(*_frame(profile))
    assert client.sent == []


def test_commander_stops_sending_and_reports_when_control_is_lost():
    profile = active()
    st = _status(profile)
    c, client = _commander(st)
    c.prepare()
    c.activate()
    c.write(*_frame(profile))
    st.update(state="estop", latched=["estop"], epoch=4)
    with pytest.raises(PermissionErrorMotion, match="急停"):
        c.write(*_frame(profile))
    assert len(client.sent) == 1
    assert not c.ready() and "急停" in c.phase()


def test_release_without_answer_stops_heartbeat_so_the_guardian_holds():
    from g1_guardian.client import GuardianUnavailable

    profile = active()
    c, client = _commander(_status(profile))
    c.prepare()
    c.activate()
    client.release_error = GuardianUnavailable("no answer")
    with pytest.raises(PermissionErrorMotion, match="原地保持"):
        c.release()
    assert client.heartbeat is False
    assert not c.busy()


def test_lost_text_cases():
    base = dict(state="holding", latched=[], epoch=5, owner="robot-platform")
    assert lost_text(base, "robot-platform", 5) is None
    assert lost_text(dict(base, epoch=4), "robot-platform", 5) is None, "乱序到达的旧状态不算"
    assert "失联" in lost_text(dict(base, state="fault", latched=["link_lost"], epoch=6), "robot-platform", 5)
    assert "lowstate" in lost_text(dict(base, state="fault", latched=["stale_lowstate"], epoch=6), "robot-platform", 5)
    assert "写入者" in lost_text(dict(base, state="fault", latched=["foreign_writer"], epoch=6), "robot-platform", 5)
    assert "释放" in lost_text(dict(base, state="releasing", epoch=6), "robot-platform", 5)
    assert "refactor-node" in lost_text(dict(base, owner="refactor-node", epoch=7), "robot-platform", 5)
    assert "复位" in lost_text(dict(base, epoch=6), "robot-platform", 5)
    assert "复位" in engage_refusal_text("estop_latched", "x")
    assert "13" in engage_refusal_text("motors_not_idle:13", "x")


# -------------------------------------------------------------- 设置与登记
def test_create_backend_guardian_with_settings_from_profile_and_env(monkeypatch):
    profile = active()
    monkeypatch.setenv("ARM_GUARDIAN_HOST", "10.0.0.5")
    monkeypatch.setenv("ARM_GUARDIAN_PORT", "18100")
    monkeypatch.setenv("ARM_GUARDIAN_READ", "mock")
    gs = guardian_settings(profile)
    assert (gs.host, gs.port, gs.read_backend, gs.client_id) == ("10.0.0.5", 18100, "mock", "robot-platform")
    b = create_backend("guardian", SimpleNamespace(profile=profile))
    try:
        assert b.name == "guardian" and b.read_backend.name == "mock"
        assert b._client is None, "建后端时不连守护层"
    finally:
        b.close()
    monkeypatch.setenv("ARM_GUARDIAN_READ", "guardian")
    with pytest.raises(ValueError):
        guardian_settings(profile)


def test_guardian_reset_route_resets_then_engages():
    from robot_platform.web import routes
    from robot_platform.web.reset_confirm import ResetConfirm

    calls = []
    epoch = {"n": 4}
    # 复位和重新接管都经运控核心（守护层的复位由核心转给后端；接管会先撤销全部租约），不直接调后端或 ArmLoop
    core = SimpleNamespace(reset_external_estop=lambda: calls.append("core.reset_external_estop"),
                           engage=lambda: calls.append("core.engage"),
                           has_external_estop=True, status=lambda: {"epoch": epoch["n"]})
    app = SimpleNamespace(backend=SimpleNamespace(reset_guardian=lambda: calls.append("backend.reset")),
                          reset_confirm=ResetConfirm())
    # 复位要人工确认：没带令牌什么都不做
    with pytest.raises(PermissionErrorMotion, match="先在页面上确认"):
        routes._guardian_reset(app, core, {})
    # 确认之后网页又急停 / 接管 / 释放过（核心 epoch 变了）：不复位
    req = routes._guardian_reset_request(app, core, {})
    epoch["n"] += 1
    with pytest.raises(PermissionErrorMotion, match="锁存变了"):
        routes._guardian_reset(app, core, {"token": req["token"]})
    assert calls == []
    req = routes._guardian_reset_request(app, core, {})
    routes._guardian_reset(app, core, {"token": req["token"]})
    assert calls == ["core.reset_external_estop", "core.engage"]


def test_core_forwards_the_guardian_reset_to_the_backend():
    from robot_platform.core.config_loader import load_settings
    from robot_platform.web.app import App

    settings = load_settings(backend="mock")
    profile = settings.profile
    kin = RobotKinematics(settings.urdf, settings.base_link, list(profile.arms))
    backend = MockBackend(profile)
    calls = []
    backend.reset_guardian = lambda: calls.append("reset")
    app = App(settings, kin, allow_real_motion=True, backend=backend)
    try:
        app.reader()
        app.motion().reset_external_estop()
        assert calls == ["reset"]
    finally:
        app.loop.shutdown()
        backend.close()
    plain = MockBackend(profile)
    app = App(settings, kin, allow_real_motion=True, backend=plain)
    try:
        app.reader()
        with pytest.raises(PermissionErrorMotion, match="没有守护层"):
            app.motion().reset_external_estop()
    finally:
        app.loop.shutdown()
        plain.close()


# -------------------------------------------------- 守护层配置与机型是否一致
@pytest.mark.parametrize("robot", ["g1d", "g1"])
def test_guardian_config_matches_the_platform_profile(robot):
    """守护层配置是另写的一份：增益、电机分组、前馈上限、拖拽阻尼都必须和平台一致。"""
    profile = load_profile(robot)
    cfg = load_config(builtin_config_path(robot))
    assert cfg.robot == profile.name and cfg.num_motors == profile.num_motors
    lock = set(profile.waist.hold_motors)
    for m in range(cfg.num_motors):
        if m in lock:
            assert (cfg.kp[m], cfg.kd[m]) == (profile.waist.lock_kp, profile.waist.lock_kd), f"腰电机 {m}"
        else:
            assert (cfg.kp[m], cfg.kd[m]) == (profile.kp[m], profile.kd[m]), f"电机 {m}"
    for arm, layout in profile.arms.items():
        grp = cfg.group(f"{arm}_arm")
        assert grp.motors == tuple(layout.motor_indices)
        assert all(t == TAU_FF_LIMIT for t in grp.tau_ff_limit)
        assert all(v >= 1.0 for v in grp.max_velocity), "不比 ArmLoop 的限速上限更慢"
    waist = set(cfg.group("waist").motors) | set(cfg.hold_at_engage)
    assert waist == lock, "守护层锁住的腰电机就是平台锁的那几个"
    assert cfg.compliant_kd == TEACH_KD
    assert set(cfg.idle_motors) == set(profile.control.idle_motors)
    # 直写后端的 lowstate 过期上限和守护层的一致（负责人定的 100 ms）
    from robot_platform.hw.backends.base import profile_lowstate_max_age_s
    assert profile_lowstate_max_age_s(profile) == cfg.lowstate_stale_s == 0.1
