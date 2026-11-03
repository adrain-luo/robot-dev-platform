"""整套 HTTP 接口跑在 mock 后端上：接管、走点、急停、拖拽、腰、立柱、换臂、释放。

只调不落盘的接口（不存 TCP / 负载 / 零位），跑完不改仓库里的数据文件。
"""

import math


def test_mock_writes_go_to_temp_instance(mock_server):
    """mock 默认用临时实例：页面上的保存 / 锁定不碰仓库里任何真实机器的数据。"""
    from pathlib import Path
    real = Path(__file__).resolve().parents[1] / "instances"
    before = {p: p.stat().st_mtime_ns for p in real.rglob("*") if p.is_file()}
    assert mock_server.get("/api/scene")["instance"] == "temp"
    code, d = mock_server.post("/api/runtime-config/lock", {"locked": True})
    assert code == 200 and d["locked"]
    code, d = mock_server.post("/api/lift/set-zero")
    assert code == 200 and d["ok"] and d["path"] == "state/lift_zero.json", d
    mock_server.post("/api/runtime-config/lock", {"locked": False})
    after = {p: p.stat().st_mtime_ns for p in real.rglob("*") if p.is_file()}
    assert before == after


def test_scene_shape(mock_server):
    s = mock_server.get("/api/scene")
    assert s["ok"] and s["backend"] == "mock" and s["robot"] == "g1d"
    assert set(s["arms"]) == {"right", "left"}
    assert [j["motor"] for j in s["waist"]["joints"]] == [12, 14]
    assert s["lift"]["available"] is True
    assert len(s["arms"]["right"]["joints"]) == 7


def test_engage_target_hold_release(mock_server):
    c = mock_server
    code, d = c.post("/api/engage")
    assert code == 200 and d["ctrl"]["engaged"]
    q0 = [j["q_rad"] for j in c.get("/api/scene")["arms"]["right"]["joints"]]
    tgt = list(q0)
    tgt[3] -= 0.2
    assert c.post("/api/speed", {"max_speed_rad_s": 0.5})[0] == 200
    assert c.post("/api/target", {"q": tgt})[0] == 200
    c.wait(lambda s: abs(s["arms"]["right"]["joints"][3]["q_rad"] - tgt[3]) < 0.01)

    # 急停：目标锁回实测，后续不再动
    tgt2 = list(tgt)
    tgt2[3] += 0.3
    c.post("/api/target", {"q": tgt2})
    code, d = c.post("/api/hold")
    assert code == 200
    a = c.get("/api/scene")["ctrl"]["desired_rad"][3]
    b = c.get("/api/scene")["ctrl"]["desired_rad"][3]
    assert a == b and abs(a - tgt2[3]) > 0.05

    code, d = c.post("/api/release")
    assert code == 200 and not d["ctrl"]["engaged"]
    code, d = c.post("/api/target", {"q": tgt})
    assert code == 403


def test_teach_mode(mock_server):
    c = mock_server
    c.post("/api/engage")
    code, d = c.post("/api/teach", {"enabled": True})
    assert code == 200 and d["ctrl"]["float"]
    code, d = c.post("/api/target", {"q": d["ctrl"]["cmd_rad"]})
    assert code == 403, "拖拽中不允许走点"
    code, d = c.post("/api/teach", {"enabled": False})
    assert not d["ctrl"]["float"]
    c.post("/api/release")


def test_waist_step_and_limits(mock_server):
    c = mock_server
    c.post("/api/engage")
    c.post("/api/speed", {"max_speed_rad_s": 1.0})
    yaw0 = c.get("/api/scene")["waist"]["yaw_deg"]
    assert c.post("/api/waist/step", {"delta_yaw_deg": 5})[0] == 200
    c.wait(lambda s: abs(s["waist"]["yaw_deg"] - (yaw0 + 5)) < 0.2)
    # 俯仰下限约 -2.5°
    c.post("/api/waist", {"pitch_deg": -30})
    s = c.wait(lambda s: abs(s["waist"]["pitch_deg"] - math.degrees(-0.04363)) < 0.2)
    assert s["waist"]["pitch_deg"] > -3
    assert c.post("/api/waist/stop")[0] == 200
    c.post("/api/waist", {"yaw_deg": yaw0, "pitch_deg": 0})
    c.wait(lambda s: abs(s["waist"]["yaw_deg"] - yaw0) < 0.2)
    c.post("/api/release")


def test_lift_step_and_stop(mock_server):
    c = mock_server
    s0 = c.get("/api/scene")["lift"]["stroke_mm"]
    code, d = c.post("/api/lift/step", {"delta_mm": 15})
    assert code == 200 and d["ok"], d
    s = c.wait(lambda s: not (s["lift"].get("ctrl") or {}).get("busy"), timeout_s=10)
    assert abs(s["lift"]["stroke_mm"] - (s0 + 15)) < 3
    code, d = c.post("/api/lift/step", {"delta_mm": -15})
    code, d = c.post("/api/lift/stop")
    assert d["ok"] and not d["busy"]


def test_switch_arm_requires_release(mock_server):
    c = mock_server
    c.post("/api/engage")
    code, d = c.post("/api/control-arm", {"arm": "left"})
    assert code == 500 and "释放" in d["error"]
    c.post("/api/release")
    code, d = c.post("/api/control-arm", {"arm": "left"})
    assert code == 200 and d["ctrl"]["arm"] == "left"
    c.post("/api/control-arm", {"arm": "right"})


def test_robot_caps(mock_server):
    """前端靠它决定显示哪些控件：G1D 有立柱、腰可转、非腿式。"""
    c = mock_server.get("/api/robot")
    assert c["name"] == "g1d" and c["label"] == "G1D" and c["backend"] == "mock"
    assert c["lift"]["stroke_max_mm"] == 420 and c["waist"]["movable"] is True
    assert [j["motor"] for j in c["waist"]["joints"]] == [12, 14]
    assert c["legged"] is False and c["transitions"] is False
    assert mock_server.get("/api/scene")["body"] == {}
