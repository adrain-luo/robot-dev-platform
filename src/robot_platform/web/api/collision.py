"""碰撞检查：/api/collision/*（状态、开关与余量、三维用的碰撞体）。

闸门本身在 ArmLoop.set_target 里（control/arm_loop.py），所有给手臂下目标的入口都经过它。
"""

from __future__ import annotations

import threading
import traceback
from typing import Any, Dict, Optional

from robot_platform.calib import lift as lift_calib


class CollisionMixin:
    _collision_guard = None
    _collision_tried = False
    # 建闸门要一两秒：页面轮询状态和启动时建控制循环（attach_collision_guard）会同时来要，
    # 不加锁时后来的那个会拿到「已经在建、但还是 None」，控制循环就挂上了空闸门、不再检查
    _collision_build_lock = threading.Lock()

    def collision_guard(self):
        """按机型的 collision.json 建闸门；没有模型返回 None。"""
        if self._collision_tried:
            return self._collision_guard
        with self._collision_build_lock:
            if not self._collision_tried:
                self._build_collision_guard()
                self._collision_tried = True
        return self._collision_guard

    def _build_collision_guard(self) -> None:
        try:
            from robot_platform.collision.guard import CollisionGuard
            from robot_platform.collision.model import try_load

            model = try_load(self.profile.urdf, self.settings.base_link)
            if model is not None:
                self._collision_guard = CollisionGuard(
                    model, {n: lay.joint_names for n, lay in self.profile.arms.items()}, self._collision_body_q,
                    source=(self.profile.urdf, self.settings.base_link), tools_fn=self._collision_tools,
                )
        except Exception:  # noqa: BLE001  模型坏了不影响别的功能，只是不查
            traceback.print_exc()
            self._collision_guard = None

    def _collision_tools(self) -> tuple:
        """各手当前生效的 TCP（腕末端连杆系下的点）；没生效的手不给，末端就只有腕。"""
        from robot_platform.calib import tcp as tcp_calib

        out = []
        for arm, lay in self.profile.arms.items():
            doc = tcp_calib.load_active_doc(arm) or {}
            tcp = (doc.get("result") or {}).get("tcp_m")
            if str(doc.get("arm") or "") != arm or tcp is None:
                continue
            link = str(doc.get("tip_link") or lay.tip_link)
            out.append((link, tuple(float(v) for v in tcp)))
        return tuple(out)

    def _collision_body_q(self) -> Dict[str, float]:
        """身体部分的关节：腰（按实测）、立柱各级（按实测高度）。拿不到就按 0。"""
        q: Dict[str, float] = {}
        try:
            w = self.profile.waist
            if w is not None and self._reader is not None:
                snap = self._reader.snapshot().q
                for j in w.joints:
                    q[j.urdf_joint] = float(snap[j.motor])
        except Exception:  # noqa: BLE001
            pass
        try:
            li = self.profile.lift
            if li is not None and self._hispeed is not None:
                st = self._hispeed.latest()
                if st is not None:
                    q.update(li.height_to_joints(st.height_m, zero_m=lift_calib.load_zero()))
        except Exception:  # noqa: BLE001
            pass
        return q

    def attach_collision_guard(self, loop) -> None:
        loop.collision_guard = self.collision_guard()

    def _collision_arms_q(self) -> Optional[Dict[str, Any]]:
        """两只手的实测关节角。"""
        try:
            reader = self.reader()
            return {n: list(reader.read_arm(lay).q) for n, lay in self.profile.arms.items()}
        except Exception:  # noqa: BLE001
            return None

    def collision_status(self, with_distance: bool = True) -> Dict[str, Any]:
        g = self.collision_guard()
        if g is None:
            return {"available": False, "note": f"{self.profile.name} 没有碰撞模型（robots/<机型>/collision.json，用 tools/fit_collision.py 生成）"}
        try:
            out = g.status(self._collision_arms_q() if with_distance else None)
        except Exception as exc:  # noqa: BLE001
            out = g.status(None)
            out["error"] = f"{type(exc).__name__}: {exc}"
        # 控制循环上挂的是不是这个闸门（没挂上 = 下目标时不检查）；只读模式没有控制循环
        loop = getattr(self, "loop", None)
        out["gate_attached"] = None if loop is None else (getattr(loop, "collision_guard", None) is g)
        return out

    def collision_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        g = self.collision_guard()
        if path == "/api/collision/status":
            return {"ok": True, "collision": self.collision_status()}
        if g is None:
            return {"ok": False, "error": "本机型没有碰撞模型", "collision": self.collision_status(False)}
        if path == "/api/collision/config":
            try:
                margin = body.get("margin_mm")
                switch = any(body.get(k) is not None for k in ("backend", "planner", "detour"))
                g.configure(
                    enabled=None if body.get("enabled") is None else bool(body.get("enabled")),
                    margin_m=None if margin in (None, "") else float(margin) / 1000.0,
                    detour=None if body.get("detour") is None else bool(body.get("detour")),
                    backend=None if body.get("backend") in (None, "") else str(body.get("backend")),
                    planner=None if body.get("planner") in (None, "") else str(body.get("planner")),
                )
            except (TypeError, ValueError) as exc:
                return {"ok": False, "error": str(exc), "collision": self.collision_status()}
            st = self.collision_status()
            if switch:
                msg = (f"避障：碰撞检测用{'coal' if g.model.backend == 'coal' else '内置'}，"
                       f"绕行规划用{'OMPL' if g.planner == 'ompl' else '内置 RRT'}，"
                       f"{'直线会碰时自动绕行' if g.detour else '只拦不绕'}")
            else:
                msg = (f"碰撞检查已开，余量 {g.margin_m * 1000:.0f} mm" if g.enabled
                       else "碰撞检查已关：手臂目标不再检查自碰撞")
            return {"ok": True, "message": msg, "collision": st}
        if path == "/api/collision/geometry":
            return {"ok": True, **g.geometry()}
        return {"ok": False, "error": "没有这个碰撞检查接口"}
