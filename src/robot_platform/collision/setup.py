"""碰撞闸门的组装与接口逻辑：网页（进程内运控核心）和运控核心进程（阶段 2）共用。

- build_guard：按机型的 collision.json 建闸门；没有模型返回 None。
- active_tools / body_q：闸门要的当前 TCP 和身体关节（腰、立柱）。
- guard_status / guard_request：/api/collision/* 的应答（网页直接调，或运控核心进程里调后经 IPC 回给网页）。
"""

from __future__ import annotations

import traceback
from typing import Any, Callable, Dict, Optional

from robot_platform.calib import lift as lift_calib


def active_tools(profile) -> tuple:
    """各手当前生效的 TCP（腕末端连杆系下的点）；没生效的手不给，末端就只有腕。"""
    from robot_platform.calib import tcp as tcp_calib

    out = []
    for arm, lay in profile.arms.items():
        doc = tcp_calib.load_active_doc(arm) or {}
        tcp = (doc.get("result") or {}).get("tcp_m")
        if str(doc.get("arm") or "") != arm or tcp is None:
            continue
        link = str(doc.get("tip_link") or lay.tip_link)
        out.append((link, tuple(float(v) for v in tcp)))
    return tuple(out)


def body_q(profile, reader, lift_reader) -> Dict[str, float]:
    """身体部分的关节：腰（按实测）、立柱各级（按实测高度）。拿不到就按 0。"""
    q: Dict[str, float] = {}
    try:
        w = profile.waist
        if w is not None and reader is not None:
            snap = reader.snapshot().q
            for j in w.joints:
                q[j.urdf_joint] = float(snap[j.motor])
    except Exception:  # noqa: BLE001
        pass
    try:
        li = profile.lift
        if li is not None and lift_reader is not None:
            st = lift_reader.latest()
            if st is not None:
                q.update(li.height_to_joints(st.height_m, zero_m=lift_calib.load_zero()))
    except Exception:  # noqa: BLE001
        pass
    return q


def build_guard(profile, base_link: str, body_q_fn: Callable[[], Dict[str, float]],
                tools_fn: Callable[[], tuple]):
    """按机型的 collision.json 建闸门；没有模型或模型坏了返回 None（只是不查，不影响别的功能）。"""
    try:
        from robot_platform.collision.guard import CollisionGuard
        from robot_platform.collision.model import try_load

        model = try_load(profile.urdf, base_link)
        if model is None:
            return None
        return CollisionGuard(
            model, {n: lay.joint_names for n, lay in profile.arms.items()}, body_q_fn,
            source=(profile.urdf, base_link), tools_fn=tools_fn,
        )
    except Exception:  # noqa: BLE001
        traceback.print_exc()
        return None


def guard_status(guard, profile_name: str, arms_q: Optional[Dict[str, Any]], *,
                 gate_attached: Optional[bool]) -> Dict[str, Any]:
    """碰撞检查状态。arms_q：两只手的实测角（None = 不算距离）；gate_attached：控制循环上挂的是不是它。"""
    if guard is None:
        return {"available": False,
                "note": f"{profile_name} 没有碰撞模型（robots/<机型>/collision.json，用 tools/fit_collision.py 生成）"}
    try:
        out = guard.status(arms_q)
    except Exception as exc:  # noqa: BLE001
        out = guard.status(None)
        out["error"] = f"{type(exc).__name__}: {exc}"
    out["gate_attached"] = gate_attached
    return out


def guard_request(guard, path: str, body: Dict[str, Any], status: Callable[[bool], Dict[str, Any]]) -> Dict[str, Any]:
    """/api/collision/{status,config,geometry}。status(with_distance) 给出当前状态。"""
    if path == "/api/collision/status":
        return {"ok": True, "collision": status(True)}
    if guard is None:
        return {"ok": False, "error": "本机型没有碰撞模型", "collision": status(False)}
    if path == "/api/collision/config":
        try:
            margin = body.get("margin_mm")
            switch = any(body.get(k) is not None for k in ("backend", "planner", "detour"))
            guard.configure(
                enabled=None if body.get("enabled") is None else bool(body.get("enabled")),
                margin_m=None if margin in (None, "") else float(margin) / 1000.0,
                detour=None if body.get("detour") is None else bool(body.get("detour")),
                backend=None if body.get("backend") in (None, "") else str(body.get("backend")),
                planner=None if body.get("planner") in (None, "") else str(body.get("planner")),
            )
        except (TypeError, ValueError) as exc:
            return {"ok": False, "error": str(exc), "collision": status(True)}
        st = status(True)
        if switch:
            msg = (f"避障：碰撞检测用{'coal' if guard.model.backend == 'coal' else '内置'}，"
                   f"绕行规划用{'OMPL' if guard.planner == 'ompl' else '内置 RRT'}，"
                   f"{'直线会碰时自动绕行' if guard.detour else '只拦不绕'}")
        else:
            msg = (f"碰撞检查已开，余量 {guard.margin_m * 1000:.0f} mm" if guard.enabled
                   else "碰撞检查已关：手臂目标不再检查自碰撞")
        return {"ok": True, "message": msg, "collision": st}
    if path == "/api/collision/geometry":
        return {"ok": True, **guard.geometry()}
    return {"ok": False, "error": "没有这个碰撞检查接口"}
