"""立柱：高度读数、Agv 驱动、闭环控制器与 /api/lift/* 接口。"""

from __future__ import annotations

from typing import Any, Dict

from robot_platform.calib import lift as lift_calib
from robot_platform.control.arm_loop import PermissionErrorMotion
from robot_platform.hw.backends.base import LiftDriver, LiftReader
from robot_platform.hw.lift_control import LiftController


class LiftMixin:
    def _require_lift(self):
        if self.profile.lift is None:
            raise PermissionErrorMotion(f"{self.profile.name} 没有立柱")
        return self.profile.lift

    def hispeed(self) -> LiftReader:
        self._require_lift()
        # 先起状态订阅；勿在 _lock 里调 reader，锁不可重入
        self.reader()
        with self._lock:
            if self._hispeed is None:
                self._hispeed = self.backend.lift_reader()
            return self._hispeed

    def agv(self) -> LiftDriver:
        self._require_lift()
        self.reader()
        with self._lock:
            if self._agv is None:
                self._agv = self.backend.lift_driver()
            return self._agv

    def lift_ctrl(self) -> LiftController:
        with self._lock:
            if self._lift_ctrl is None:
                # hispeed/agv 在锁外初始化，避免重入
                pass
        hs = self.hispeed()
        agv = self.agv()
        with self._lock:
            if self._lift_ctrl is None:
                self._lift_ctrl = LiftController(
                    hispeed=hs, agv=agv, stroke_max_m=self.profile.lift.stroke_max_m
                )
            return self._lift_ctrl

    def lift_set_zero_now(self) -> Dict[str, Any]:
        st = self.hispeed().latest()
        if st is None:
            return {"ok": False, "error": f"还没有 {self.profile.lift.state_topic} 数据，稍后再试", "lift": None}
        out = lift_calib.save_zero(st.height_m)
        # 立刻按新零位重算，给前端刷新
        scene = self.both_scenes()
        out["lift"] = scene.get("lift")
        out["message"] = (
            f"已记最低点 {st.height_m:.4f} m（{st.height_m * 1000.0:.1f} mm），三维行程归零"
        )
        return out

    def lift_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        if self.profile.lift is None:
            if path == "/api/lift/stop":
                return {"ok": True, "busy": False, "message": "没有立柱"}
            return {"ok": False, "error": f"{self.profile.name} 没有立柱"}
        if path == "/api/lift/set-zero":
            return self.lift_set_zero_now()
        if path == "/api/lift/stop":
            if self._lift_ctrl is None and self._agv is None:
                return {"ok": True, "busy": False, "message": "立柱本来就没在动"}
            return self.lift_ctrl().stop()
        self.require_motion()
        ctrl = self.lift_ctrl()
        if path == "/api/lift/goto":
            if "stroke_mm" in body and body.get("stroke_mm") not in (None, ""):
                zero = lift_calib.load_zero()
                height = zero + float(body["stroke_mm"]) / 1000.0
            elif "height_m" in body and body.get("height_m") not in (None, ""):
                height = float(body["height_m"])
            else:
                return {"ok": False, "error": "需要 stroke_mm 或 height_m"}
            out = ctrl.goto(height)
            out["message"] = f"立柱目标行程 {(out.get('target_m', height) - lift_calib.load_zero()) * 1000.0:.1f} mm"
            return out
        if path == "/api/lift/step":
            if "delta_mm" in body and body.get("delta_mm") not in (None, ""):
                delta = float(body["delta_mm"]) / 1000.0
            elif "delta_m" in body and body.get("delta_m") not in (None, ""):
                delta = float(body["delta_m"])
            else:
                return {"ok": False, "error": "需要 delta_mm"}
            out = ctrl.step(delta)
            if out.get("ok"):
                sign = "升" if delta > 0 else "降"
                out["message"] = f"立柱{sign}一步 {abs(delta) * 1000.0:.1f} mm"
            return out
        return {"ok": False, "error": "没有这个立柱接口"}
