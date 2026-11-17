"""自碰撞检查：几何距离、路径闸门、接口。"""

from __future__ import annotations

import numpy as np
import pytest

from robot_platform.collision.geometry import segment_distances
from robot_platform.collision.guard import CollisionBlocked, CollisionGuard
from robot_platform.collision.model import try_load
from robot_platform.core.robot_profile import load_profile


def test_segment_distance_matches_brute_force():
    rng = np.random.default_rng(0)
    n = 300
    p1, q1, p2, q2 = (rng.normal(size=(n, 3)) for _ in range(4))
    # 混进退化成点的线段
    q1[:30] = p1[:30]
    q2[20:50] = p2[20:50]
    got = segment_distances(p1, q1, p2, q2)
    t = np.linspace(0, 1, 401)
    for i in range(n):
        a = p1[i] + np.outer(t, q1[i] - p1[i])
        b = p2[i] + np.outer(t, q2[i] - p2[i])
        brute = np.sqrt(((a[:, None] - b[None]) ** 2).sum(-1)).min()
        assert got[i] <= brute + 1e-9
        assert got[i] >= brute - 5e-3 * (np.linalg.norm(q1[i] - p1[i]) + np.linalg.norm(q2[i] - p2[i]) + 1e-9)


@pytest.fixture(scope="module")
def g1d():
    prof = load_profile("g1d")
    model = try_load(prof.urdf)
    assert model is not None, "robots/g1d/collision.json 缺失"
    guard = CollisionGuard(model, {n: lay.joint_names for n, lay in prof.arms.items()}, lambda: {})
    guard.enabled = True
    guard.margin_m = 0.005
    return prof, model, guard


def _colliding_right_target(prof, model, guard):
    """找一个右手会撞上身体 / 左手的目标（从 home 出发）。"""
    from robot_platform.control.urdf_fk import URDFRobotModel

    lay = prof.arms["right"]
    lim = URDFRobotModel(str(prof.urdf), "torso_link", lay.tip_link).joint_limits()
    home = {n: np.asarray(l.home_q, dtype=float) for n, l in prof.arms.items()}
    rng = np.random.default_rng(7)
    idx = guard._arm_pairs["right"]
    for _ in range(2000):
        q = rng.uniform(lim[:, 0], lim[:, 1])
        d = model.evaluate(guard._q({}, {"left": home["left"], "right": q}), 0.08, idx)
        if d.min() < -0.02:
            return home, q
    raise AssertionError("没找到会撞的姿态")


def test_home_pose_is_clear(g1d):
    prof, model, guard = g1d
    home = {n: np.asarray(l.home_q, dtype=float) for n, l in prof.arms.items()}
    d = model.evaluate(guard._q({}, home), 0.08)
    assert d.min() > 0.0, model.pairs[int(np.argmin(d))]


def test_guard_blocks_colliding_target_and_passes_small_move(g1d):
    prof, model, guard = g1d
    home, bad = _colliding_right_target(prof, model, guard)
    with pytest.raises(CollisionBlocked) as ei:
        guard.check_move("right", home["right"], bad, {"left": [home["left"]]})
    assert "碰撞检查" in str(ei.value)
    assert guard.last_block and guard.last_block["arm"] == "right"
    small = home["right"].copy()
    small[3] += 0.05
    out = guard.check_move("right", home["right"], small, {"left": [home["left"]]})
    assert out["checked"] and out["ms"] < 2000
    # 关了就不查
    guard.enabled = False
    try:
        assert guard.check_move("right", home["right"], bad, {"left": [home["left"]]}) == {"checked": False}
    finally:
        guard.enabled = True


def test_guard_allows_moving_out_of_start_overlap(g1d):
    """起点已轻微重叠（只一对）：沿让它变远的方向退放行；反方向（更近 2 mm 以上）拦。"""
    prof, model, guard = g1d
    home = {n: np.asarray(l.home_q, dtype=float) for n, l in prof.arms.items()}
    idx = guard._arm_pairs["right"]
    others = {"left": [home["left"]]}

    def dist(q):
        return model.evaluate(guard._q({}, {"left": home["left"], "right": q}), 0.03, idx)

    rng = np.random.default_rng(5)
    start = None
    for _ in range(5000):
        q = home["right"] + rng.normal(scale=0.6, size=7)
        d = dist(q)
        hit = np.nonzero(d < 0.0)[0]
        if hit.size == 1 and -0.015 < d[hit[0]] < -0.003 and np.sort(d)[1] > 0.012:
            start, k = q, int(hit[0])
            break
    assert start is not None
    grad = np.zeros(7)
    for j in range(7):
        e = np.zeros(7)
        e[j] = 1e-3
        grad[j] = (dist(start + e)[k] - dist(start - e)[k]) / 2e-3
    u = grad / np.linalg.norm(grad)
    d0k = float(dist(start)[k])
    guard.check_move("right", start, start + 0.01 * u, others)
    # 反方向走到这对再近 4 mm 以上，应被拦
    step = 0.02
    while step < 0.5 and dist(start - step * u)[k] > d0k - 0.004:
        step *= 1.5
    with pytest.raises(CollisionBlocked):
        guard.check_move("right", start, start - step * u, others)


def test_api_status_and_block(mock_server):
    c = mock_server
    code, d = c.post("/api/collision/status")
    assert code == 200 and d["collision"]["available"], d
    assert d["collision"]["enabled"]
    code, d = c.post("/api/collision/geometry")
    assert d["ok"] and d["links"]
    code, d = c.post("/api/collision/config", {"margin_mm": 8})
    assert d["ok"] and abs(d["collision"]["margin_mm"] - 8) < 1e-6
    c.post("/api/collision/config", {"margin_mm": 5})

    prof = load_profile("g1d")
    model = try_load(prof.urdf)
    guard = CollisionGuard(model, {n: lay.joint_names for n, lay in prof.arms.items()}, lambda: {})
    st = c.get("/api/scene")
    if st["ctrl"].get("engaged"):
        c.post("/api/release")
        c.wait(lambda s: not s["ctrl"]["engaged"] and not s["ctrl"].get("transition"))
    assert c.post("/api/engage")[0] == 200
    arms = c.get("/api/scene")["ctrl"]["arms"]
    now = {n: np.asarray(a["cmd_rad"], dtype=float) for n, a in arms.items()}
    from robot_platform.control.urdf_fk import URDFRobotModel

    lay = prof.arms["right"]
    lim = URDFRobotModel(str(prof.urdf), "torso_link", lay.tip_link).joint_limits()
    rng = np.random.default_rng(11)
    idx = guard._arm_pairs["right"]
    bad = None
    for _ in range(3000):
        q = rng.uniform(lim[:, 0], lim[:, 1])
        if model.evaluate(guard._q({}, {"left": now["left"], "right": q}), 0.08, idx).min() < -0.02:
            bad = q
            break
    assert bad is not None
    code, d = c.post("/api/target", {"q": bad.tolist(), "arm": "right"})
    assert "碰撞检查" in str(d.get("error")), (code, d)
    code, d = c.post("/api/collision/status")
    assert d["collision"]["last_block"]["arm"] == "right"
    c.post("/api/release")
