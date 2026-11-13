"""整套 HTTP 接口跑在 mock 后端上：接管（双臂）、走点、急停、拖拽、腰、立柱、释放。

只调不落盘的接口（不存 TCP / 负载 / 零位），跑完不改仓库里的数据文件。
"""

import time
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
    assert [j["motor"] for j in s["waist"]["joints"]] == [12]  # G1D 没有腰俯仰
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
    assert c.post("/api/target", {"q": tgt, "arm": "right"})[0] == 200
    c.wait(lambda s: abs(s["arms"]["right"]["joints"][3]["q_rad"] - tgt[3]) < 0.01)

    # 急停：目标锁回实测，后续不再动
    tgt2 = list(tgt)
    tgt2[3] += 0.3
    c.post("/api/target", {"q": tgt2, "arm": "right"})
    code, d = c.post("/api/hold")
    assert code == 200
    a = c.get("/api/scene")["ctrl"]["arms"]["right"]["desired_rad"][3]
    b = c.get("/api/scene")["ctrl"]["arms"]["right"]["desired_rad"][3]
    assert a == b and abs(a - tgt2[3]) > 0.05

    code, d = c.post("/api/release")
    assert code == 200 and not d["ctrl"]["engaged"]
    code, d = c.post("/api/target", {"q": tgt, "arm": "right"})
    assert code == 403


def test_target_requires_arm(mock_server):
    """没有「当前控制臂」了：设目标必须说明是哪只手。"""
    c = mock_server
    c.post("/api/engage")
    q = c.get("/api/scene")["ctrl"]["arms"]["right"]["cmd_rad"]
    code, d = c.post("/api/target", {"q": q})
    assert code != 200 and "哪只手" in d["error"]
    c.post("/api/release")


def test_teach_mode_one_arm(mock_server):
    """左手拖拽时右手照常能走点；左手自己不能走点。"""
    c = mock_server
    c.post("/api/engage")
    code, d = c.post("/api/teach", {"enabled": True, "arm": "left"})
    arms = d["ctrl"]["arms"]
    assert code == 200 and arms["left"]["float"] and not arms["right"]["float"]
    code, d = c.post("/api/target", {"q": arms["left"]["cmd_rad"], "arm": "left"})
    assert code == 403, "拖拽中的手不允许走点"
    code, d = c.post("/api/target", {"q": arms["right"]["cmd_rad"], "arm": "right"})
    assert code == 200, "另一只手照常受控"
    code, d = c.post("/api/teach", {"enabled": False, "arm": "left"})
    assert not d["ctrl"]["arms"]["left"]["float"]
    c.post("/api/release")


def test_waist_step_and_limits(mock_server):
    c = mock_server
    c.post("/api/engage")
    c.post("/api/speed", {"max_speed_rad_s": 1.0})
    yaw0 = c.get("/api/scene")["waist"]["yaw_deg"]
    assert c.post("/api/waist/step", {"delta_yaw_deg": 5})[0] == 200
    c.wait(lambda s: abs(s["waist"]["yaw_deg"] - (yaw0 + 5)) < 0.2)
    # G1D 没有腰俯仰：给俯仰报错，三维 / 状态里也没有俯仰
    assert c.post("/api/waist", {"pitch_deg": -30})[0] != 200
    assert c.post("/api/waist/step", {"delta_pitch_deg": 5})[0] != 200
    assert "pitch_deg" not in c.get("/api/scene")["waist"]
    assert [j["role"] for j in c.get("/api/robot")["waist"]["joints"]] == ["yaw"]
    assert c.post("/api/waist/stop")[0] == 200
    c.post("/api/waist", {"yaw_deg": yaw0})
    c.wait(lambda s: abs(s["waist"]["yaw_deg"] - yaw0) < 0.2)
    c.post("/api/release")


def test_lift_jog_deadman(mock_server):
    """键盘长按：按速度升，停止续命 0.5 s 内自动停；stop 立即停。"""
    import time as _t
    c = mock_server
    c.post("/api/lift/stop")
    c.post("/api/lift/goto", {"stroke_mm": 100})
    c.wait(lambda s: not (s["lift"].get("ctrl") or {}).get("busy"), timeout_s=15)
    s0 = c.get("/api/scene")["lift"]["stroke_mm"]
    t0 = _t.monotonic()
    while _t.monotonic() - t0 < 1.0:
        code, d = c.post("/api/lift/jog", {"speed_mm_s": 30})
        assert code == 200 and d["ok"], d
        _t.sleep(0.1)
    # 停止续命：0.5 s 看门狗后停下
    s = c.wait(lambda s: not (s["lift"].get("ctrl") or {}).get("busy"), timeout_s=3)
    moved = s["lift"]["stroke_mm"] - s0
    assert 20 < moved < 60, moved  # ~1 s × 30 mm/s + 看门狗那一截
    _t.sleep(0.4)
    assert abs(c.get("/api/scene")["lift"]["stroke_mm"] - s["lift"]["stroke_mm"]) < 1.0
    # 往下点动再 stop：立即停
    c.post("/api/lift/jog", {"speed_mm_s": -30})
    c.post("/api/lift/stop")
    s = c.wait(lambda s: not (s["lift"].get("ctrl") or {}).get("busy"), timeout_s=3)
    assert not c.post("/api/lift/jog", {})[1]["ok"]


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


def test_engage_takes_both_arms(mock_server):
    """接管就是两只手一起：都锁住、都有指令角；没有切换控制臂的接口了。"""
    c = mock_server
    code, d = c.post("/api/engage")
    assert code == 200 and set(d["ctrl"]["arms"]) == {"left", "right"}
    for arm in ("left", "right"):
        assert len(d["ctrl"]["arms"][arm]["cmd_rad"]) == 7
    s = c.get("/api/scene")
    assert "cmd_links" in s["arms"]["left"]["scene"] and "cmd_links" in s["arms"]["right"]["scene"]
    c.post("/api/release")


def test_robot_caps(mock_server):
    """前端靠它决定显示哪些控件：G1D 有立柱、腰可转、非腿式。"""
    c = mock_server.get("/api/robot")
    assert c["name"] == "g1d" and c["label"] == "G1D" and c["backend"] == "mock"
    assert c["lift"]["stroke_max_mm"] == 420 and c["waist"]["movable"] is True
    assert [j["motor"] for j in c["waist"]["joints"]] == [12]  # G1D 没有腰俯仰
    assert c["legged"] is False and c["transitions"] is False
    # 非腿式：没有腿的显示关节，只有按躯干 IMU 摆的腰俯仰（只给三维看）
    assert set(mock_server.get("/api/scene")["body"]) == {"Yaw_Joint"}


def test_app_js_is_bundle_of_parts(mock_server):
    """/static/app.js 由 web/static/js/ 下各文件按文件名顺序拼成，启动文件在最后。"""
    import urllib.request
    from robot_platform.paths import STATIC_DIR

    with urllib.request.urlopen(mock_server.base + "/static/app.js", timeout=10) as r:
        assert r.status == 200 and "javascript" in r.headers.get("Content-Type")
        text = r.read().decode("utf-8")
    parts = sorted((STATIC_DIR / "js").glob("*.js"))
    assert parts[0].name == "00-util.js" and parts[-1].name == "99-main.js"
    pos = [text.index(f"// {p.name} ——") for p in parts]
    assert pos == sorted(pos), "拼接顺序应与文件名顺序一致"
    assert "try {\n$(\"arm-select\")" not in text, "顶层 try 已去掉"


def test_hold_stops_multi_pose_tasks(mock_server):
    """急停要停掉连续走点的任务（负载测量 / 到位对比 / 手眼采集），不能只停当前这一段。"""
    c = mock_server
    st = c.get("/api/scene")
    if st["ctrl"].get("engaged"):
        c.post("/api/release")
        c.wait(lambda s: not s["ctrl"]["engaged"] and not s["ctrl"].get("transition"))
    assert c.post("/api/engage")[0] == 200
    assert c.post("/api/speed", {"max_speed_rad_s": 0.4})[0] == 200
    name = "estop_test_set"
    c.post("/api/posesets/delete", {"name": name})
    assert c.post("/api/posesets/create", {"name": name, "arm": "right"})[0] == 200
    q0 = [j["q_rad"] for j in c.get("/api/scene")["arms"]["right"]["joints"]]
    for d in (0.0, 0.5, -0.5, 0.5):
        tgt = list(q0)
        tgt[3] += d
        c.post("/api/target", {"q": tgt, "arm": "right"})
        c.wait(lambda s: abs(s["arms"]["right"]["joints"][3]["q_rad"] - tgt[3]) < 0.01, timeout_s=10)
        assert c.post("/api/posesets/record", {"name": name})[0] == 200
    c.post("/api/target", {"q": q0, "arm": "right"})
    c.wait(lambda s: abs(s["arms"]["right"]["joints"][3]["q_rad"] - q0[3]) < 0.01, timeout_s=10)

    code, d = c.post("/api/identify/measure", {"name": name})
    assert code == 200 and d["identify"]["running"], d
    # 等它开始走（desired 离开起点）再急停
    try:
        c.wait(lambda s: abs(s["ctrl"]["arms"]["right"]["desired_rad"][3] - q0[3]) > 0.1, timeout_s=30)
    except AssertionError:
        raise AssertionError(str(c.get("/api/scene")["identify"]))
    time.sleep(0.3)
    assert c.post("/api/hold")[0] == 200
    t0 = time.monotonic()
    while c.get("/api/scene")["identify"]["running"]:
        assert time.monotonic() - t0 < 5, "急停后负载测量还在跑"
        time.sleep(0.1)
    d1 = c.get("/api/scene")["ctrl"]["arms"]["right"]["desired_rad"]
    time.sleep(2.0)
    d2 = c.get("/api/scene")["ctrl"]["arms"]["right"]["desired_rad"]
    assert d1 == d2, "急停后手臂又被发了新目标"
    c.post("/api/posesets/delete", {"name": name})
    c.post("/api/release")
