"""IK 策略加了第三个「罗-IK」：能切过去（有 scipy 时），没有 scipy 时拒绝并说明原因。"""

from robot_platform.tasks.luo_ik import scipy_available


def test_switch_to_luo(mock_server):
    c = mock_server
    d = c.post("/api/runtime-config/pose-ik-mode", {"mode": "luo"})[1]
    if scipy_available():
        assert d["ok"] and d["work_pose"]["ik_mode"] == "luo", d
        assert "罗-IK" in d["message"]
    else:
        assert not d["ok"] and "scipy" in d["error"], d
    d = c.post("/api/runtime-config/pose-ik-mode", {"mode": "robust"})[1]
    assert d["ok"] and d["work_pose"]["ik_mode"] == "robust"
    assert not c.post("/api/runtime-config/pose-ik-mode", {"mode": "xx"})[1]["ok"]
