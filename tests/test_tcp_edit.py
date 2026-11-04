"""TCP 修正：以腕末端中心新建、沿腕三轴修正后覆盖 / 另存、调用中的 TCP 立即生效、配置锁定。"""


def _saved(st, name):
    return next((s for s in st["saved"] if s["name"] == name), None)


def test_new_flange_and_duplicate(mock_server):
    c = mock_server
    code, st = c.post("/api/tcp/new-flange", {"name": "pt_flange", "arm": "left"})
    assert code == 200 and st["ok"], st
    s = _saved(st, "pt_flange")
    assert s["arm"] == "left" and s["tcp_mm"] == [0.0, 0.0, 0.0] and s["source"] == "flange"
    code, st = c.post("/api/tcp/new-flange", {"name": "pt_flange", "arm": "left"})
    assert st["ok"] is False and st.get("exists") and "已存在" in st["error"]
    code, st = c.post("/api/tcp/new-flange", {"name": "pt_flange", "arm": "right", "force": True})
    assert st["ok"] and _saved(st, "pt_flange")["arm"] == "right"


def test_edit_overwrite_saveas_and_history(mock_server):
    c = mock_server
    c.post("/api/tcp/new-flange", {"name": "pt_edit", "arm": "right", "force": True})
    code, st = c.post("/api/tcp/edit-save", {"src": "pt_edit", "tcp_mm": [120, 0, -5], "mode": "overwrite"})
    assert st["ok"], st
    assert _saved(st, "pt_edit")["tcp_mm"] == [120.0, 0.0, -5.0]
    assert "变化 [+120.0, +0.0, -5.0]" in st["message"]

    code, st = c.post("/api/tcp/edit-save", {"src": "pt_edit", "tcp_mm": [125, 2, -5], "mode": "saveas",
                                              "name": "pt_edit_b"})
    assert st["ok"] and st["saved_name"] == "pt_edit_b"
    assert _saved(st, "pt_edit_b")["tcp_mm"] == [125.0, 2.0, -5.0]
    assert _saved(st, "pt_edit")["tcp_mm"] == [120.0, 0.0, -5.0], "另存不改原 TCP"
    assert _saved(st, "pt_edit_b")["source"] == "flange"

    # 另存不能用原名；目标已存在要 force
    code, st = c.post("/api/tcp/edit-save", {"src": "pt_edit", "tcp_mm": [1, 1, 1], "mode": "saveas", "name": "pt_edit"})
    assert st["ok"] is False and "新名字" in st["error"]
    code, st = c.post("/api/tcp/edit-save", {"src": "pt_edit", "tcp_mm": [1, 1, 1], "mode": "saveas", "name": "pt_edit_b"})
    assert st["ok"] is False and st.get("exists")
    # 越界、格式不对
    code, st = c.post("/api/tcp/edit-save", {"src": "pt_edit", "tcp_mm": [600, 0, 0], "mode": "overwrite"})
    assert st["ok"] is False and "500" in st["error"]
    code, st = c.post("/api/tcp/edit-save", {"src": "pt_edit", "tcp_mm": [1, 2], "mode": "overwrite"})
    assert st["ok"] is False


def test_overwrite_active_tcp_takes_effect_and_lock(mock_server):
    c = mock_server
    c.post("/api/tcp/new-flange", {"name": "pt_active", "arm": "right", "force": True})
    code, st = c.post("/api/tcp/load", {"name": "pt_active"})
    assert st["ok"] and st["active_right"] == "pt_active"
    code, st = c.post("/api/tcp/edit-save", {"src": "pt_active", "tcp_mm": [80, 0, 10], "mode": "overwrite"})
    assert st["ok"]
    # 当前调用的就是它：会话里的偏置立即更新（红点、碰点都用它）
    assert st["active_name"] == "pt_active"
    assert [round(v, 6) for v in st["result"]["tcp_m"]] == [0.08, 0.0, 0.01]
    assert st["result"]["fixed_mm"] is None and st["result"]["manual"] is True

    # 配置锁定后：覆盖正在应用的 TCP 被拒绝，另存仍可以
    c.post("/api/runtime-config/lock", {"locked": True})
    try:
        code, st = c.post("/api/tcp/edit-save", {"src": "pt_active", "tcp_mm": [90, 0, 10], "mode": "overwrite"})
        assert st["ok"] is False and "锁定" in st["error"]
        code, st = c.post("/api/tcp/edit-save", {"src": "pt_active", "tcp_mm": [90, 0, 10], "mode": "saveas",
                                                  "name": "pt_active_copy", "force": True})
        assert st["ok"], st
    finally:
        c.post("/api/runtime-config/lock", {"locked": False})
