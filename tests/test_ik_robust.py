"""多初值姿态 IK：在子进程里算（不占主进程 GIL）、当前角就能解时提前收手、够不着时限时。"""

import os
import runpy
import time
from pathlib import Path

import numpy as np
import pytest

from robot_platform.control.fk_scene import RobotKinematics
from robot_platform.core.robot_profile import active
from robot_platform.tasks import ik_worker
from robot_platform.tasks.spatial_points import (
    solve_tcp_pose_ik,
    solve_tcp_pose_ik_robust,
    solve_tcp_pose_ik_robust_local,
)

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def arm():
    p = active()
    model = RobotKinematics(p.urdf, p.base_link, list(p.arms)).arms["right"].model
    q0 = np.asarray(p.arm("right").home_q, float)
    tcp = np.array([0.0, 0.0, 0.1])
    t, R = model.fk(q0)
    return model, q0, tcp, t + R @ tcp, R


def test_runs_in_worker_process_and_matches_local(arm):
    model, q0, tcp, c, R = arm
    pid = ik_worker.worker_pid()
    assert pid is not None and pid != os.getpid()
    target = c + [0.06, 0.03, 0.04]
    got = solve_tcp_pose_ik_robust(model, q0, tcp, target, R, extra_seeds=[q0])
    ref = solve_tcp_pose_ik_robust_local(model, q0, tcp, target, R, extra_seeds=[q0])
    assert ik_worker.last_error == ""
    assert np.allclose(got[0], ref[0]) and got[1] == pytest.approx(ref[1]) and got[2] == pytest.approx(ref[2])
    assert got[1] < 3.0 and got[2] < 3.0


def test_stops_early_when_current_angles_solve_it(arm):
    """从当前角出发就有合格解：直接用它，不再试随机初值（以前要凑够 5 个合格解）。"""
    model, q0, tcp, c, R = arm
    target = c + [0.02, 0.0, 0.0]
    t0 = time.perf_counter()
    q, m, o = solve_tcp_pose_ik_robust_local(model, q0, tcp, target, R, extra_seeds=[q0])
    dt = time.perf_counter() - t0
    q_single, m_single, _ = solve_tcp_pose_ik(model, q0, tcp, target, R, w_ori=0.35, iters=100)
    assert m < 3.0 and o < 3.0
    assert np.allclose(q, q_single)  # 就是当前角那一档的解
    assert dt < 0.5


def test_unreachable_target_respects_time_budget(arm):
    model, q0, tcp, c, R = arm
    t0 = time.perf_counter()
    q, m, o = solve_tcp_pose_ik_robust_local(model, q0, tcp, c + [0.6, 0, 0], R, time_budget_s=0.5)
    dt = time.perf_counter() - t0
    assert m > 100.0  # 确实够不着，退回残差最好的
    # 预算到了不再开新初值；最后一个初值的 3 档权重最多再多算一会儿
    assert dt < 2.0


def test_inprocess_fallback(arm, monkeypatch):
    model, q0, tcp, c, R = arm
    monkeypatch.setenv("RDP_IK_INPROCESS", "1")
    calls = []

    def fn(x):
        calls.append(os.getpid())
        return x + 1

    assert ik_worker.run(fn, 1) == 2  # 局部函数传不进子进程也无妨：这里直接本进程算
    assert calls == [os.getpid()]


def test_worker_errors_from_fn_propagate():
    with pytest.raises(ValueError):
        ik_worker.run(int, "不是数")


@pytest.mark.parametrize("entry", ["serve.py", "src/robot_platform/__main__.py"])
def test_entry_does_not_start_server_when_reimported(entry, monkeypatch):
    """spawn 子进程以 __mp_main__ 重新导入入口文件：不能在那里再起一个服务。"""
    import robot_platform.web.server as server

    monkeypatch.setattr(server, "main", lambda: pytest.fail("入口在非 __main__ 下启动了服务"))
    runpy.run_path(str(ROOT / entry), run_name="__mp_main__")


def _alive(pid: int) -> bool:
    if os.name == "nt":  # Windows 上 os.kill(pid, 0) 会直接杀进程，不能用来探活
        import subprocess

        out = subprocess.run(["tasklist", "/FI", f"PID eq {pid}", "/NH"], capture_output=True, text=True).stdout
        return str(pid) in out
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


def test_worker_exits_when_server_is_killed():
    """服务被硬杀（来不及收拾进程池）时，IK 子进程也要跟着退出，不留孤儿。"""
    import subprocess
    import sys

    code = (
        "import os, sys; sys.path.insert(0, sys.argv[1])\n"
        "from robot_platform.tasks import ik_worker\n"
        "if __name__ == '__main__':\n"
        "    print(ik_worker.worker_pid(), flush=True)\n"
        "    os._exit(0)\n"
    )
    out = subprocess.run([sys.executable, "-c", code, str(ROOT / "src")], capture_output=True, text=True, timeout=60)
    pid = int(out.stdout.strip().splitlines()[-1])
    deadline = time.monotonic() + 10
    while _alive(pid):
        assert time.monotonic() < deadline, f"IK 子进程 {pid} 在服务退出后还活着"
        time.sleep(0.2)


def test_goto_with_locked_work_pose_uses_robust_ik(mock_server):
    """整条链路：锁定工作姿态（多初值）后走空间点，能到位、姿态保持，期间控制循环频率不掉。"""
    import threading

    c = mock_server
    assert c.post("/api/engage")[0] == 200
    c.post("/api/speed", {"max_speed_rad_s": 1.0})
    for path, body in [("/api/tcp/new-flange", {"name": "e2e法兰", "arm": "right", "force": True}),
                       ("/api/tcp/load", {"name": "e2e法兰"}),
                       ("/api/runtime-config/pose-ik-mode", {"mode": "robust"}),
                       ("/api/work-pose/save", {"name": "e2e姿态", "arm": "right"}),
                       ("/api/work-pose/load", {"name": "e2e姿态", "tcp_name": "e2e法兰", "arm": "right"})]:
        code, d = c.post(path, body)
        assert code == 200 and d.get("ok", True), (path, d)
    T0 = np.asarray(c.get("/api/scene")["arms"]["right"]["scene"]["wrist_T"])
    hz, stop = [], threading.Event()

    def watch():
        while not stop.is_set():
            hz.append(c.get("/api/status")["ctrl"]["loop_hz"])
            time.sleep(0.1)

    th = threading.Thread(target=watch, daemon=True)
    th.start()
    try:
        target = T0[:3, 3] + [0.03, 0.0, 0.02]
        code, d = c.post("/api/points/goto-xyz", {"point_m": target.tolist(), "arm": "right", "refine": False})
    finally:
        stop.set()
        th.join()
    assert code == 200 and d["ok"], d
    assert d["work_pose_locked"] is True and d["plan_ori_deg"] < 3.0
    assert d["measured_miss_mm"] < 3.0
    T1 = np.asarray(c.get("/api/scene")["arms"]["right"]["scene"]["wrist_T"])
    assert np.degrees(np.arccos(np.clip((np.trace(T0[:3, :3].T @ T1[:3, :3]) - 1) / 2, -1, 1))) < 3.0
    assert hz and min(hz) > 80.0, hz
    c.post("/api/work-pose/clear", {"tcp_name": "e2e法兰"})
    c.post("/api/release")
