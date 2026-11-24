"""任务改用运控核心的租约：急停撤销租约就能停住任务；「停止」按钮照常；任务结束一定归还租约。"""

import threading
import time

import numpy as np
import pytest

from g1_contracts.motion import MotionRejected, Reason
from robot_platform.calib.payload import identify as iden
from robot_platform.calib.payload import validate as vali
from robot_platform.control.arm_loop import ArmLoop
from robot_platform.control.fk_scene import RobotKinematics
from robot_platform.control.motion_core import MotionCore
from robot_platform.core.robot_profile import active
from robot_platform.hw.backends.mock import MockBackend

from test_arm_loop_frame import Recorder

FAST_STILL = {"still_dq_rad_s": 0.05, "still_drift_rad": 0.01, "still_window_s": 0.2, "settle_timeout_s": 3.0}


@pytest.fixture
def core():
    profile = active()
    backend = MockBackend(profile)
    kin = RobotKinematics(profile.urdf, profile.base_link, list(profile.arms))
    loop = ArmLoop(reader=backend.state(), commander=Recorder(), kin=kin,
                   urdf_path=profile.urdf, base_link=profile.base_link, max_speed_rad_s=0.3)
    c = MotionCore(loop)
    c.engage()
    yield c
    loop.shutdown()
    backend.close()


def _poses(core, arm="right", n=3, step=0.25):
    q0 = np.asarray(core.arm_sample(arm)["q_meas"], float)
    poses = []
    for k in range(n):
        q = q0.copy()
        q[3] -= step * (k + 1)
        poses.append({"name": f"p{k}", "q": q.tolist()})
    return {"name": "lease_test", "arm": arm, "poses": poses}


def _run(task):
    done = threading.Event()
    result = {}

    def on_done(r):
        result.update(r)
        done.set()

    task.on_done = on_done
    th = threading.Thread(target=task.run, daemon=True)
    th.start()
    return th, done, result


def _wait_moving(core, arm, q_from, timeout=5.0):
    deadline = time.monotonic() + timeout
    while float(np.max(np.abs(np.asarray(core.arm_status(arm)["desired_rad"]) - q_from))) < 0.05:
        assert time.monotonic() < deadline, "任务没开始走"
        time.sleep(0.02)


def _measure(core, lease):
    profile = active()
    return iden.MeasureTask(lease, _poses(core), profile.urdf, profile.base_link, lambda s: None, None,
                            still=FAST_STILL, bidirectional=False, dwell_s=0.0, avg_frames=2)


@pytest.mark.parametrize("stopper", ["estop", "engage", "stop_button"])
def test_measure_task_stops_and_returns_its_lease(core, stopper):
    q0 = np.asarray(core.arm_status("right")["desired_rad"])
    lease = core.lease_arm("right", "负载测量")
    task = _measure(core, lease)
    th, done, result = _run(task)
    _wait_moving(core, "right", q0)
    {"estop": lambda: core.estop("测试"), "engage": core.engage, "stop_button": task.request_stop}[stopper]()
    assert done.wait(3.0), "任务没停"
    th.join(2.0)
    assert result["ok"] is False and ("已停止" in result["error"] or "撤销" in result["error"])
    assert core.status()["leases"] == [] and core.loop.owner("right") is None
    if stopper == "estop":
        core.reset_estop()
    # 手空出来了：新任务能拿到
    with core.lease_arm("right", "下一个"):
        pass


def test_finished_task_returns_lease(core):
    lease = core.lease_arm("right", "负载测量")
    task = _measure(core, lease)
    task.poses_cfg["poses"] = task.poses_cfg["poses"][:2]
    for p in task.poses_cfg["poses"]:
        p["q"][3] += 0.2  # 走近一点，测试快
    th, done, result = _run(task)
    assert done.wait(20.0)
    th.join(2.0)
    assert result["ok"] is True, result
    assert core.status()["leases"] == []


def test_compare_task_restores_payload_after_estop(core):
    """到位对比轮换负载方案时被急停：任务停下，原来的负载补偿照样换回去。"""
    profile = active()
    core.set_payload_theta([0.5, 0.0, 0.0, 0.02], "原补偿", arm="right")
    lease = core.lease_arm("right", "到位对比")
    other = {"theta": [1.5, 0.1, 0.0, 0.0], "name": "试的补偿"}
    task = vali.CompareTask(lease, _poses(core), profile.urdf, profile.base_link, lambda s: None, None,
                            still=FAST_STILL, modes=["payload"], payload=other, correction_iters=0)
    q0 = np.asarray(core.arm_status("right")["desired_rad"])
    th, done, result = _run(task)
    _wait_moving(core, "right", q0)
    assert core.arm_status("right")["payload_name"] == "试的补偿"
    core.estop("测试")
    assert done.wait(3.0)
    th.join(2.0)
    assert result["ok"] is False
    assert core.arm_status("right")["payload_name"] == "原补偿"
    assert core.status()["leases"] == []


def test_tasks_cannot_start_while_latched_or_arm_taken(core):
    core.estop("测试")
    with pytest.raises(MotionRejected) as e:
        core.lease_arm("right", "负载测量")
    assert e.value.reason is Reason.ESTOP_LATCHED
    core.reset_estop()
    with core.lease_arm("right", "手眼采集"):
        with pytest.raises(MotionRejected) as e:
            core.lease_arm("right", "负载测量")
        assert e.value.reason is Reason.BUSY and "手眼采集" in str(e.value)


def test_routes_refuse_to_start_tasks_while_latched(mock_server):
    c = mock_server
    assert c.post("/api/engage")[0] == 200
    name = "lease_route_set"
    c.post("/api/posesets/delete", {"name": name})
    assert c.post("/api/posesets/create", {"name": name, "arm": "right"})[0] == 200
    for _ in range(3):
        assert c.post("/api/posesets/record", {"name": name})[0] == 200
    assert c.post("/api/hold")[0] == 200
    for path in ("/api/identify/measure", "/api/validate/start", "/api/posesets/goto"):
        code, d = c.post(path, {"name": name, "index": 0, "modes": ["baseline"]})
        assert code == 403 and "急停已锁定" in d["error"], (path, code, d)
    assert c.get("/api/identify")["identify"]["running"] is False
    assert c.post("/api/estop/reset")[0] == 200
    c.post("/api/posesets/delete", {"name": name})
    c.post("/api/release")


# ---------------------------------------------------------------- 页面上的到位类操作（同步 HTTP，期间租着手）

def _scene_owner(c, arm="right"):
    return c.get("/api/scene")["ctrl"]["arms"][arm]["owner"]


def _in_thread(fn):
    out = {}
    th = threading.Thread(target=lambda: out.update(r=fn()), daemon=True)
    th.start()
    return th, out


def test_hold_stops_a_single_pose_goto(mock_server):
    c = mock_server
    assert c.post("/api/engage")[0] == 200
    assert c.post("/api/speed", {"max_speed_rad_s": 0.2})[0] == 200
    c.post("/api/single-pose/delete", {"name": "lease_sp"})
    q0 = [j["q_rad"] for j in c.get("/api/scene")["arms"]["right"]["joints"]]
    assert c.post("/api/single-pose/record", {"name": "lease_sp", "arm": "right"})[1]["ok"]
    away = list(q0)
    away[3] += 0.6
    c.post("/api/target", {"q": away, "arm": "right"})
    c.wait(lambda s: abs(s["arms"]["right"]["joints"][3]["q_rad"] - away[3]) < 0.01, timeout_s=10)
    th, out = _in_thread(lambda: c.post("/api/single-pose/goto", {"name": "lease_sp"}))
    c.wait(lambda s: (s["ctrl"]["arms"]["right"]["owner"] or "").startswith("单姿态到位"), timeout_s=5)
    time.sleep(0.3)
    assert c.post("/api/hold")[0] == 200
    th.join(5.0)
    code, d = out["r"]
    assert code == 200 and not d["ok"] and ("已停止" in d["error"] or "撤销" in d["error"]), d
    assert _scene_owner(c) is None
    d1 = c.get("/api/scene")["ctrl"]["arms"]["right"]["desired_rad"]
    time.sleep(0.5)
    assert c.get("/api/scene")["ctrl"]["arms"]["right"]["desired_rad"] == d1
    assert c.post("/api/estop/reset")[0] == 200
    c.post("/api/single-pose/delete", {"name": "lease_sp"})
    c.post("/api/release")


def test_work_pose_load_holds_the_arm_while_it_moves(mock_server):
    """调用工作姿态期间租着这只手：手动走点被拒；走完归还。"""
    c = mock_server
    assert c.post("/api/engage")[0] == 200
    assert c.post("/api/speed", {"max_speed_rad_s": 0.2})[0] == 200
    for path, body in [("/api/tcp/new-flange", {"name": "lease法兰", "arm": "right", "force": True}),
                       ("/api/tcp/load", {"name": "lease法兰"}),
                       ("/api/work-pose/save", {"name": "lease姿态", "arm": "right"})]:
        code, d = c.post(path, body)
        assert code == 200 and d.get("ok", True), (path, d)
    q0 = [j["q_rad"] for j in c.get("/api/scene")["arms"]["right"]["joints"]]
    turned = list(q0)
    turned[6] += 0.5  # 转手腕：姿态变了，调用工作姿态要转回来
    c.post("/api/target", {"q": turned, "arm": "right"})
    c.wait(lambda s: abs(s["arms"]["right"]["joints"][6]["q_rad"] - turned[6]) < 0.01, timeout_s=10)
    th, out = _in_thread(lambda: c.post("/api/work-pose/load",
                                        {"name": "lease姿态", "tcp_name": "lease法兰", "arm": "right"}))
    c.wait(lambda s: (s["ctrl"]["arms"]["right"]["owner"] or "").startswith("工作姿态调用"), timeout_s=5)
    code, d = c.post("/api/target", {"q": turned, "arm": "right"})
    assert code == 403 and "工作姿态调用" in d["error"], d
    th.join(20.0)
    code, d = out["r"]
    assert code == 200 and d["ok"], d
    assert _scene_owner(c) is None
    c.post("/api/work-pose/clear", {"tcp_name": "lease法兰"})
    c.post("/api/release")


def test_refused_manual_target_does_not_unlock_the_work_pose():
    """手被任务租着时手动走点被拒：不能顺手把任务依赖的工作姿态锁定解掉（原来靠 _work_pose_motion 标志）。"""
    from types import SimpleNamespace

    from robot_platform.control.arm_loop import PermissionErrorMotion
    from robot_platform.web import routes

    unlocked = []

    def refuse(q, *, arm, detour):
        raise PermissionErrorMotion("右臂正被「空间点碰点#3」使用")

    app = SimpleNamespace(body_arm=lambda body: "right", unlock_work_pose=lambda *a: unlocked.append(a),
                          motion=lambda: SimpleNamespace(manual_target=refuse))
    with pytest.raises(PermissionErrorMotion):
        routes._target(app, None, {"q": [0.0] * 7, "arm": "right"})
    assert unlocked == []
    app.motion = lambda: SimpleNamespace(manual_target=lambda q, *, arm, detour: None)
    assert routes._target(app, None, {"q": [0.0] * 7, "arm": "right"}) is None
    assert unlocked == [("arm_control", "right")]
