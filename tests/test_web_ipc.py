"""网页 --motion-core ipc：网页是真的子进程（mock 后端），运控核心在本测试进程里（帧由 Tee 记下）。

- 页面的臂控指令、租约任务（单姿态到位）都落到核心进程里；
- 杀掉网页进程：它手动动着的手停住，核心照常发帧；
- 核心不在：页面报「连不上运控核心」，核心回来后自动重连；
- ipc 时网页不建指令出口、不碰指令话题（守护层后端时也不会多出一个守护层客户端）。
"""

import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import pytest

from g1_contracts.motion import LeaseRevoked, Resource
from robot_platform.control.fk_scene import RobotKinematics
from robot_platform.control.remote_core import RemoteMotionCore, RemoteStateSource
from robot_platform.core import instance as instances
from robot_platform.core.config_loader import load_settings
from robot_platform.hw.backends.base import PermissionErrorMotion
from robot_platform.hw.backends.mock import MockBackend

from conftest import Client, _free_port
from test_motion_core_server import MAX_GAP_S, Tee, arm_st, held_at_command, wait_until
from g1_motion_core.service import CoreProcess

SRC = Path(__file__).resolve().parents[1] / "src"


def _core(sockdir):
    settings = load_settings(backend="mock")
    settings.max_speed_rad_s = 1.0
    backend = MockBackend(settings.profile)
    tee = Tee(backend.commander())
    backend.commander = lambda: tee
    kin = RobotKinematics(settings.urdf, settings.base_link, list(settings.profile.arms))
    p = CoreProcess(settings, kin, backend, log=lambda s: None)
    p.tee = tee
    p.serve(sockdir)
    return p


@pytest.fixture
def sockdir():
    d = tempfile.mkdtemp(prefix="g1m")
    yield d
    shutil.rmtree(d, ignore_errors=True)


@pytest.fixture
def core(sockdir):
    p = _core(sockdir)
    yield p
    p.close(timeout_s=2.0)


@pytest.fixture
def web(core, sockdir, tmp_path):
    """网页子进程：mock 后端、ipc、和核心用同一个实例目录。"""
    port = _free_port()
    log = open(tmp_path / "web.log", "wb")
    env = {**os.environ, "PYTHONPATH": os.pathsep.join([str(SRC), os.environ.get("PYTHONPATH", "")]).rstrip(os.pathsep)}
    env.pop("ARM_INSTANCE", None)
    proc = subprocess.Popen(
        [sys.executable, "-m", "robot_platform", "--backend", "mock", "--motion-core", "ipc", "--core-dir", sockdir,
         "--instance", str(instances.active().root), "--host", "127.0.0.1", "--port", str(port)],
        stdout=log, stderr=subprocess.STDOUT, env=env)
    client = Client(f"http://127.0.0.1:{port}")
    deadline = time.monotonic() + 30
    while True:
        try:
            client.get("/api/scene")
            break
        except Exception:
            if proc.poll() is not None or time.monotonic() > deadline:
                proc.kill()
                log.close()
                raise RuntimeError((tmp_path / "web.log").read_text(encoding="utf-8", errors="replace"))
            time.sleep(0.2)
    client.proc = proc
    yield client
    if proc.poll() is None:
        proc.terminate()
        proc.wait(timeout=10)
    log.close()


def _away(core, arm="right", d=-0.5):
    q = np.array(arm_st(core, arm)["cmd_rad"])
    q[3] += d
    return q.tolist()


def test_page_commands_land_in_the_core_process(core, web):
    assert web.post("/api/engage")[0] == 200
    assert core.loop.status()["engaged"] is True
    scene = web.get("/api/scene")
    ctrl = scene["ctrl"]
    assert ctrl["engaged"] and ctrl["motion"]["safety"] == "normal"
    assert ctrl["core_process"]["pid"] == os.getpid(), "页面状态来自核心进程"
    assert ctrl["cmd_bus"]["foreign"] is False, "指令话题占用由核心报"

    code, d = web.post("/api/speed", {"max_speed_rad_s": 0.5})
    assert code == 200 and core.core.max_speed == 0.5
    tgt = _away(core, d=-0.3)
    code, d = web.post("/api/target", {"q": tgt, "arm": "right"})
    assert code == 200, d
    assert arm_st(core)["desired_rad"] == pytest.approx(tgt)
    web.post("/api/arm/stop", {"arm": "right"})
    code, d = web.post("/api/teach", {"arm": "left", "enabled": True})
    assert code == 200 and arm_st(core, "left")["float"] is True
    web.post("/api/teach", {"arm": "left", "enabled": False})
    assert arm_st(core, "left")["float"] is False

    code, d = web.post("/api/hold")
    assert code == 200 and core.core.latched
    code, d = web.post("/api/target", {"q": tgt, "arm": "right"})
    assert code == 403 and "急停" in d["error"]
    assert web.post("/api/estop/reset")[0] == 200 and not core.core.latched
    code, d = web.post("/api/guardian/reset")
    assert code == 403 and "没有守护层" in d["error"]

    code, d = web.post("/api/collision/status")
    assert code == 200 and "collision" in d
    assert d["collision"].get("gate_attached") in (True, None)


def test_a_task_on_a_lease_runs_from_the_web_process(core, web):
    assert web.post("/api/engage")[0] == 200
    code, d = web.post("/api/single-pose/record", {"name": "ipc到位", "arm": "right", "overwrite": True})
    assert code == 200 and d.get("ok", True), d
    web.post("/api/target", {"q": _away(core, d=-0.25), "arm": "right"})
    wait_until(lambda: not arm_st(core)["moving"], 5.0, "先挪开")
    code, d = web.post("/api/single-pose/goto", {"name": "ipc到位"})
    assert code == 200 and d.get("ok"), d
    assert core.core.status()["leases"] == [], "任务结束归还了租约"
    assert core.core.status()["last_revoke"]["owners"] == [], "没有租约被撤销（只有接管时那一次空撤销）"


def test_killing_the_web_holds_its_manual_move_and_frames_keep_going(core, web):
    assert web.post("/api/engage")[0] == 200
    web.post("/api/speed", {"max_speed_rad_s": 0.3})
    tgt = _away(core, d=-0.8)
    assert web.post("/api/target", {"q": tgt, "arm": "right"})[0] == 200
    wait_until(lambda: arm_st(core)["moving"], 1.0, "在走")
    time.sleep(0.2)
    t0 = time.monotonic()
    web.proc.send_signal(signal.SIGKILL)
    web.proc.wait(5)
    wait_until(lambda: held_at_command(core), 1.0, "网页没了，手停在指令角")
    assert time.monotonic() - t0 < 0.3
    cmd = np.array(arm_st(core)["cmd_rad"])
    assert np.max(np.abs(cmd - np.array(tgt))) > 0.1, "停在半路，没有继续走完"
    time.sleep(0.3)
    assert core.loop.status()["engaged"] is True
    assert core.tee.max_gap_since(t0) < MAX_GAP_S, "核心照常发帧"


def test_the_page_says_when_the_core_is_gone_and_reconnects(core, web, sockdir):
    assert web.post("/api/engage")[0] == 200
    core.server.stop()
    code, d = web.post("/api/target", {"q": _away(core, d=-0.1), "arm": "right"})
    assert code == 403 and "连不上运控核心" in d["error"]
    core.serve(sockdir)
    deadline = time.monotonic() + 3.0
    while True:
        code, d = web.post("/api/target", {"q": _away(core, d=-0.1), "arm": "right"})
        if code == 200 or time.monotonic() > deadline:
            break
        time.sleep(0.2)
    assert code == 200, d


# ---------------------------------------------------------------- 进程内：ipc 时网页不建写入者
def test_ipc_web_never_creates_a_command_writer(core, sockdir):
    from robot_platform.web.app import App

    settings = load_settings(backend="mock")
    profile = settings.profile
    backend = MockBackend(profile)
    backend.commander = lambda: pytest.fail("ipc 时网页不能建指令出口")
    backend.cmd_monitor = lambda: pytest.fail("ipc 时网页不碰指令话题（守护层后端时会多一个同名客户端）")
    backend.state = lambda: pytest.fail("ipc 时实测角从核心读")
    kin = RobotKinematics(settings.urdf, settings.base_link, list(profile.arms))
    app = App(settings, kin, allow_real_motion=True, backend=backend, motion_core="ipc", core_dir=sockdir)
    try:
        assert isinstance(app.reader(), RemoteStateSource)
        motion = app.motion()
        assert isinstance(motion, RemoteMotionCore) and app.loop is None
        motion.engage()
        st = app.ctrl_status()
        assert st["engaged"] and "cmd_bus" in st
        q = app.reader().read_arm(profile.arms["right"]).q
        assert np.allclose(q, core.reader.read_arm(profile.arms["right"]).q)
        with motion.lease_arm("right", "task:inproc") as lease:
            assert lease.move_to(_away(core, d=-0.1)).ok
            assert lease.layout is profile.arms["right"] and lease.limits.shape == (7, 2)
            assert len(lease.arm_sample(wait=True)["q_meas"]) == 7
            assert lease.status()["arm"] == "right"
        motion.estop("测试")
        assert core.core.latched
        motion.reset_estop()
    finally:
        app.shutdown_motion()
        backend.close()


def test_readonly_ipc_web_can_watch_but_not_command(core, sockdir):
    from robot_platform.web.app import App

    settings = load_settings(backend="mock")
    backend = MockBackend(settings.profile)
    kin = RobotKinematics(settings.urdf, settings.base_link, list(settings.profile.arms))
    app = App(settings, kin, allow_real_motion=False, backend=backend, motion_core="ipc", core_dir=sockdir)
    try:
        with pytest.raises(PermissionErrorMotion, match="allow-real-motion"):
            app.motion()
        st = app.ctrl_status()
        assert st["allow_real_motion"] is False and "cmd_bus" in st
        out = app.collision_post("/api/collision/config", {"enabled": False})
        assert out["ok"] is False and "只读" in out["error"]
    finally:
        app.shutdown_motion()
        backend.close()


def test_lease_from_the_web_side_is_revoked_by_a_core_estop(core, sockdir):
    from g1_motion_client import CoreConnection

    remote = RemoteMotionCore(CoreConnection(sockdir, client="web-test"), core.profile)
    try:
        remote.engage()
        lease = remote.lease([Resource.RIGHT_ARM], "task:x")
        core.core.estop("别处急停")
        assert lease.cancelled.wait(1.0)
        with pytest.raises(LeaseRevoked):
            lease.move_to(_away(core, d=-0.1))
        assert remote.latched
        remote.reset_estop()
    finally:
        remote.shutdown()


def test_a_core_for_another_robot_is_refused(core, sockdir):
    from g1_motion_client import CoreConnection
    from robot_platform.core.robot_profile import load_profile

    remote = RemoteMotionCore(CoreConnection(sockdir, client="web-test"), load_profile("g1"), connect=False)
    try:
        with pytest.raises(PermissionErrorMotion, match="同一机型"):
            remote.connect()
    finally:
        remote.shutdown()
