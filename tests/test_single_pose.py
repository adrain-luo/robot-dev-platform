"""单姿态记录和到位：记录 / 改名 / 删除、按手过滤、按已应用的 IK 策略到位（仿真臂）。"""

import time

import numpy as np
import pytest

from robot_platform.tasks import single_pose as sp


def test_store_record_rename_delete():
    R = np.eye(3)
    sp.record("起手式_t", "right", [0.1] * 7, [0.2, -0.2, 0.1], R, tcp_name="T", tcp_xyz=[0.3, -0.2, 0.1])
    with pytest.raises(ValueError):
        sp.record("起手式_t", "right", [0.1] * 7, [0, 0, 0], R)  # 重名
    with pytest.raises(ValueError):
        sp.record("坏/名字", "right", [0.1] * 7, [0, 0, 0], R)
    with pytest.raises(ValueError):
        sp.record("x_t", "middle", [0.1] * 7, [0, 0, 0], R)
    with pytest.raises(ValueError):
        sp.record("x_t", "right", [0.1] * 6, [0, 0, 0], R)
    sp.record("左一_t", "left", [0.0] * 7, [0, 0.2, 0], R)
    assert [p["name"] for p in sp.list_poses("left")] == ["左一_t"]
    assert "起手式_t" in [p["name"] for p in sp.list_poses("right")]
    d = sp.rename("起手式_t", "起手式2_t")
    assert d["name"] == "起手式2_t" and sp.load("起手式2_t")["q"] == [0.1] * 7
    with pytest.raises(FileNotFoundError):
        sp.load("起手式_t")
    with pytest.raises(ValueError):
        sp.rename("起手式2_t", "左一_t")  # 新名字已占用
    sp.delete("起手式2_t")
    sp.delete("左一_t")
    assert sp.list_poses() == []


def _engage(c):
    st = c.get("/api/scene")
    if not st["ctrl"].get("engaged"):
        assert c.post("/api/engage")[0] == 200
        c.wait(lambda s: s["ctrl"]["engaged"] and not s["ctrl"].get("transition"))


def _q(c, arm="right"):
    return [j["q_rad"] for j in c.get("/api/scene")["arms"][arm]["joints"]]


def test_api_record_and_goto(mock_server):
    c = mock_server
    _engage(c)
    assert c.post("/api/speed", {"max_speed_rad_s": 0.6})[0] == 200
    for n in ("sp_home", "sp_bent", "sp_bent2"):
        c.post("/api/single-pose/delete", {"name": n})
    q0 = _q(c)
    code, d = c.post("/api/single-pose/record", {"name": "sp_home", "arm": "right"})
    assert code == 200 and d["ok"], d
    assert d["pose"]["arm"] == "right"
    code, d = c.post("/api/single-pose/record", {"name": "sp_home", "arm": "right"})
    assert not d["ok"] and "已存在" in d["error"]

    tgt = list(q0)
    tgt[3] += 0.35
    tgt[1] -= 0.15
    c.post("/api/target", {"q": tgt, "arm": "right"})
    c.wait(lambda s: abs(s["arms"]["right"]["joints"][3]["q_rad"] - tgt[3]) < 0.01, timeout_s=10)
    assert c.post("/api/single-pose/record", {"name": "sp_bent", "arm": "right"})[1]["ok"]
    lst = c.post("/api/single-pose/list", {"arm": "right"})[1]
    assert {"sp_home", "sp_bent"} <= {p["name"] for p in lst["list"]}
    assert lst["ik_mode"] in ("single", "robust")

    # 回到 sp_home：末端位姿到位、关节也回到记录的角附近（多初值里带着记录的 q）
    code, d = c.post("/api/single-pose/goto", {"name": "sp_home"})
    assert code == 200 and d["ok"], d
    assert d["measured_miss_mm"] < 3.0 and d["measured_ori_deg"] < 1.0, d
    assert np.max(np.abs(np.array(_q(c)) - np.array(q0))) < 0.05
    # 走完不再占着手
    assert c.get("/api/scene")["ctrl"]["arms"]["right"]["owner"] is None

    assert c.post("/api/single-pose/rename", {"name": "sp_bent", "new_name": "sp_bent2"})[1]["ok"]
    code, d = c.post("/api/single-pose/goto", {"name": "sp_bent2"})
    assert d["ok"], d
    # 冗余的手肘也按记录复现，不是离当前角最近的那组解
    assert np.max(np.abs(np.array(_q(c)) - np.array(tgt))) < 0.05
    assert d["joint_diff_deg"] < 3.0, d

    # 释放后不能到位
    c.post("/api/release")
    c.wait(lambda s: not s["ctrl"]["engaged"] and not s["ctrl"].get("transition"))
    d = c.post("/api/single-pose/goto", {"name": "sp_home"})[1]
    assert not d["ok"] and "接管" in d["error"]
    for n in ("sp_home", "sp_bent2"):
        assert c.post("/api/single-pose/delete", {"name": n})[1]["ok"]
    assert not c.post("/api/single-pose/delete", {"name": "sp_home"})[1]["ok"]
    time.sleep(0.1)
