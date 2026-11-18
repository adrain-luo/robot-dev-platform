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


def test_prism_distance_matches_box():
    """多边形柱（这里取正方形截面）的精确距离 = 同尺寸长方体的解析有符号距离；胶囊体取线段上的最小值。"""
    from pathlib import Path

    from robot_platform.collision.model import CollisionModel

    prof = load_profile("g1d")
    hx, hy, z0, z1 = 0.05, 0.08, -0.02, 0.10
    prism = {"type": "prism", "z": [z0, z1], "n2": [[1, 0], [0, 1], [-1, 0], [0, -1]], "d2": [hx, hy, hx, hy]}
    spec = {"links": {"torso_link": [prism], "right_wrist_yaw_link": [{"a": [0, 0, 0], "b": [0, 0, 0], "r": 0.01}]},
            "arms": {"right": ["right_wrist_yaw_link"]}, "pairs": []}
    m = CollisionModel(Path(prof.urdf), spec)
    m.world_prims({})
    rng = np.random.default_rng(3)
    x = rng.uniform(-0.2, 0.2, size=(500, 3))
    j = np.full(len(x), int(np.nonzero(m.is_poly)[0][0]))
    got = m._poly_sdf(x, j)
    c = np.array([0, 0, 0.5 * (z0 + z1)])
    h = np.array([hx, hy, 0.5 * (z1 - z0)])
    q = np.abs(x - c) - h
    want = np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)
    assert np.allclose(got, want, atol=1e-9)
    # 胶囊体：线段穿过长方体 → 负；线段在外面平行于一个面 → 等于到该面的距离 − 半径
    a = np.array([[0.0, 0.0, -0.3], [0.08, -0.05, 0.0]])
    b = np.array([[0.0, 0.0, 0.3], [0.08, 0.05, 0.05]])
    d = m._poly_dist(a, b, np.array([0.01, 0.01]), j[:2])
    assert d[0] < 0
    assert abs(d[1] - (0.08 - hx - 0.01)) < 1e-4


def test_tool_capsule_follows_tcp():
    """末端胶囊体跟着生效的 TCP：没 TCP 不加；设了就是法兰 → TCP 一根；换了就重建。"""
    prof = load_profile("g1d")
    model = try_load(prof.urdf)
    if model is None:
        pytest.skip("没有碰撞模型")
    cur = [()]
    guard = CollisionGuard(model, {n: lay.joint_names for n, lay in prof.arms.items()}, lambda: {},
                           tools_fn=lambda: cur[0])
    guard.sync_tools()
    n0 = len(guard.model.R)
    assert guard.tools_text() == []
    cur[0] = (("right_wrist_yaw_link", (0.25, 0.0, -0.05)),)
    guard.sync_tools()
    assert len(guard.model.R) == n0 + 1
    first = guard.geometry()["links"]["right_wrist_yaw_link"][0]
    assert first[:3] == [0.0, 0.0, 0.0] and first[3:6] == [0.25, 0.0, -0.05]
    cur[0] = ()
    guard.sync_tools()
    assert len(guard.model.R) == n0


def test_guard_build_race():
    """两个线程同时要闸门（页面轮询 + 启动时挂控制循环）：后来的要等建完，不能拿到 None。"""
    import threading
    import time

    from robot_platform.web.api.collision import CollisionMixin

    class _App(CollisionMixin):
        def _build_collision_guard(self):
            time.sleep(0.3)
            self._collision_guard = object()

    app = _App()
    got = []
    ts = [threading.Thread(target=lambda: got.append(app.collision_guard())) for _ in range(3)]
    for t in ts:
        t.start()
        time.sleep(0.05)
    for t in ts:
        t.join()
    assert len(got) == 3 and all(g is not None for g in got) and len({id(g) for g in got}) == 1


# ---------------------------------------------------------------- 第二期：绕行
def _detour_case(prof, model, guard, seed=3):
    """从 home 出发：目标本身不碰、但直线会碰（按 home 起点的下限）。"""
    from robot_platform.control.urdf_fk import URDFRobotModel

    lay = prof.arms["right"]
    lim = URDFRobotModel(str(prof.urdf), "torso_link", lay.tip_link).joint_limits()
    home = {n: np.asarray(l.home_q, dtype=float) for n, l in prof.arms.items()}
    others = {"left": [home["left"]]}
    V = guard._variants("right", others)
    T = guard._thresh("right", home["right"], V, {}, guard.margin_m)
    rng = np.random.default_rng(seed)
    for _ in range(400):
        q = rng.uniform(lim[:, 0], lim[:, 1])
        if "block" in guard._sweep("right", q, q, V, T, {}, guard.margin_m):
            continue
        if "block" in guard._sweep("right", home["right"], q, V, T, {}, guard.margin_m):
            return home, q, others, lim
    raise AssertionError("没找到直线会碰的目标")


def test_planner_finds_detour_and_every_segment_is_clear(g1d):
    prof, model, guard = g1d
    home, q1, others, lim = _detour_case(prof, model, guard)
    res = guard.plan_move("right", home["right"], q1, others, lim, budget_s=6.0)
    assert res["path"] is not None, res
    assert np.allclose(res["path"][-1], q1)
    assert guard.last_plan["ok"] and guard.last_plan["waypoints"] >= 1
    pts = [home["right"], *[np.asarray(p) for p in res["path"]]]
    for a, b in zip(pts, pts[1:]):
        guard.check_move("right", a, b, others)  # 每段都按原精度、原起点下限放行（不抛错）


def test_plan_refuses_colliding_target(g1d):
    prof, model, guard = g1d
    home, bad = _colliding_right_target(prof, model, guard)
    res = guard.plan_move("right", home["right"], bad, {"left": [home["left"]]}, np.c_[bad - 3, bad + 3])
    assert res["path"] is None and "目标姿态本身" in res["reason"]


def test_shortcut_keeps_ends_and_drops_detours():
    from robot_platform.collision import planner

    rng = np.random.default_rng(0)
    path = [np.array([0.0, 0.0]), np.array([0.5, 1.0]), np.array([1.0, 1.0]), np.array([1.5, 1.0]),
            np.array([2.0, 0.0])]

    def ok(a, b):  # 只有 y ≥ 0.5 之上能横穿（中间有墙 x∈(0.9,1.1), y<0.8）
        for t in np.linspace(0, 1, 50):
            p = a + t * (b - a)
            if 0.9 < p[0] < 1.1 and p[1] < 0.8:
                return False
        return True

    out = planner.shortcut(ok, path, rng)
    assert np.allclose(out[0], path[0]) and np.allclose(out[-1], path[-1])
    assert len(out) <= len(path)
    for a, b in zip(out, out[1:]):
        assert ok(a, b)


def test_arm_state_follows_waypoints():
    """_slew 走到当前段终点就接下一个中间点；重新设目标清掉剩余中间点。"""
    from types import SimpleNamespace

    from robot_platform.control.arm_loop import ArmLoop, _ArmState

    lay = load_profile("g1d").arms["right"]
    a = _ArmState(lay, np.c_[np.full(7, -3.0), np.full(7, 3.0)], None, np.zeros(7))
    wp = [np.full(7, 0.1), np.full(7, 0.2), np.full(7, 0.05)]
    a.set_path(wp)
    fake = SimpleNamespace(max_speed=1.0, _dt=0.01)
    seen = []
    for _ in range(200):
        ArmLoop._slew(fake, a, 0.01)
        seen.append(a.desired_q.copy())
        if not a.path and np.allclose(a.cmd_q, a.desired_q):
            break
    assert np.allclose(a.cmd_q, wp[-1])
    assert any(np.allclose(s, wp[1]) for s in seen)
    a.set_path(wp)
    a.desired_q = np.zeros(7)
    assert a.path == []


def test_backend_choice_and_unavailable(g1d):
    prof, model, guard = g1d
    ch = guard.choices()
    assert {b["id"] for b in ch["backends"]} >= {"builtin", "coal"}
    assert {p["id"] for p in ch["planners"]} >= {"builtin", "ompl"}
    from robot_platform.collision.model import backend_available

    if not backend_available("coal"):
        with pytest.raises(ValueError):
            guard.configure(backend="coal")
    with pytest.raises(ValueError):
        guard.configure(backend="nope")


def test_coal_backend_is_never_less_conservative():
    """coal 后端（装了才跑）：距离不比内置的大（手臂按凸包，只会更保守），近处差得不多。"""
    pytest.importorskip("coal")
    from robot_platform.collision.coal_model import CoalModel
    from robot_platform.collision.model import default_spec_path, load_spec
    from robot_platform.control.urdf_fk import URDFRobotModel

    prof = load_profile("g1d")
    tools = (("right_wrist_yaw_link", (0.25, 0.05, -0.06)),)
    m = try_load(prof.urdf, tools=tools)
    c = CoalModel(prof.urdf, load_spec(default_spec_path(prof.urdf)), tools=tools)
    g = CollisionGuard(m, {n: lay.joint_names for n, lay in prof.arms.items()}, lambda: {}, load_config=False)
    lay = prof.arms["right"]
    lim = URDFRobotModel(str(prof.urdf), "torso_link", lay.tip_link).joint_limits()
    home = {n: np.asarray(l.home_q, dtype=float) for n, l in prof.arms.items()}
    idx = g._arm_pairs["right"]
    rng = np.random.default_rng(0)
    for _ in range(40):
        q = g._q({}, {"left": home["left"], "right": rng.uniform(lim[:, 0], lim[:, 1])})
        dp, dc = m.evaluate(q, 0.03, idx), c.evaluate(q, 0.03, idx)
        assert np.all(np.minimum(dc, 0.03) <= np.minimum(dp, 0.03) + 1e-4)


def test_ompl_planner_on_toy_problem():
    pytest.importorskip("ompl")
    from robot_platform.collision import planner

    def free(p):
        return not (abs(p[0]) < 0.1 and p[1] > -0.8)

    def edge(a, b):
        return all(free(a + t * (b - a)) for t in np.linspace(0, 1, 40))

    res = planner.plan_ompl(edge, free, np.array([-0.5, 0.5]), np.array([0.5, 0.5]),
                            np.array([[-1.0, 1.0], [-1.0, 1.0]]), budget_s=2.0, seed=0)
    assert res["path"] is not None
    pts = [np.array([-0.5, 0.5]), *res["path"]]
    assert all(edge(a, b) for a, b in zip(pts, pts[1:]))


def test_api_target_detours(mock_server):
    """/api/target 直线会碰：返回绕行摘要，手臂按中间点走（状态里 detour_left > 0），停手清掉剩余中间点。"""
    c = mock_server
    c.post("/api/collision/config", {"enabled": True, "margin_mm": 5, "detour": True})
    st = c.get("/api/scene")
    if st["ctrl"].get("engaged"):
        c.post("/api/release")
        c.wait(lambda s: not s["ctrl"]["engaged"] and not s["ctrl"].get("transition"))
    assert c.post("/api/engage")[0] == 200
    arms = c.get("/api/scene")["ctrl"]["arms"]
    now = {n: np.asarray(a["cmd_rad"], dtype=float) for n, a in arms.items()}
    prof = load_profile("g1d")
    model = try_load(prof.urdf)
    guard = CollisionGuard(model, {n: lay.joint_names for n, lay in prof.arms.items()}, lambda: {},
                           load_config=False)
    from robot_platform.control.urdf_fk import URDFRobotModel

    lim = URDFRobotModel(str(prof.urdf), "torso_link", prof.arms["right"].tip_link).joint_limits()
    others = {"left": [now["left"]]}
    V = guard._variants("right", others)
    T = guard._thresh("right", now["right"], V, {}, 0.005)
    rng = np.random.default_rng(3)
    q1 = None
    for _ in range(400):
        q = rng.uniform(lim[:, 0], lim[:, 1])
        if "block" not in guard._sweep("right", q, q, V, T, {}, 0.005) and \
                "block" in guard._sweep("right", now["right"], q, V, T, {}, 0.005):
            q1 = q
            break
    assert q1 is not None
    code, d = c.post("/api/target", {"q": q1.tolist(), "arm": "right"})
    assert code == 200 and d.get("detour") and d["detour"]["ok"], (code, d)
    r = d["ctrl"]["arms"]["right"]
    assert r["moving"] and r["detour_left"] >= 2
    assert np.allclose(r["final_rad"], q1, atol=1e-6)
    c.post("/api/arm/stop", {"arm": "right"})
    r = c.get("/api/scene")["ctrl"]["arms"]["right"]
    assert r["detour_left"] == 0
    c.post("/api/release")


def test_coal_geometry_draws_hulls():
    """coal 后端的三维显示：手臂各节是凸包（面朝外、包住原来的球），身体仍是柱。"""
    pytest.importorskip("coal")
    prof = load_profile("g1d")
    m = try_load(prof.urdf, backend="coal")
    g = CollisionGuard(m, {n: lay.joint_names for n, lay in prof.arms.items()}, lambda: {}, load_config=False)
    geo = g.geometry()
    assert geo["backend"] == "coal"
    for link in m.arm_links["right"]:
        items = geo["links"][link]
        hulls = [it for it in items if isinstance(it, dict) and "hull" in it]
        assert len(hulls) == 1, link
        V = np.asarray(hulls[0]["hull"]["v"]).reshape(-1, 3)
        F = np.asarray(hulls[0]["hull"]["f"]).reshape(-1, 3)
        c = V.mean(axis=0)
        n = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
        assert np.all(np.einsum("ij,ij->i", n, V[F[:, 0]] - c) > 0)  # 面朝外
        assert not any(isinstance(it, list) and it[0] == it[3] for it in items)  # 球不画了
    assert any(isinstance(it, dict) and "prism" in it for it in geo["links"]["torso_link"])
