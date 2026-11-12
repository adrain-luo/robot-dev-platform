"""底盘遥控：/api/chassis/*。按住才走（页面持续发 move 当心跳），急停 / 松手 / 断线即停。"""

from __future__ import annotations

from typing import Any, Dict, Optional

from robot_platform.control.arm_loop import PermissionErrorMotion
from robot_platform.hw.chassis_control import ChassisController


class ChassisMixin:
    _chassis_ctrl: Optional[ChassisController] = None
    _chassis_odom = None
    _chassis_odom_failed = False

    def chassis_ctrl(self) -> ChassisController:
        spec = self.profile.chassis
        if spec is None:
            raise PermissionErrorMotion(f"{self.profile.name} 没有底盘")
        self.reader()  # 后端先连上（DDS 初始化）
        odom = self._chassis_odom_reader()
        with self._lock:
            if self._chassis_ctrl is None:
                self._chassis_ctrl = ChassisController(self.backend.chassis_driver(), odom, spec)
            return self._chassis_ctrl

    def _chassis_odom_reader(self):
        """里程计只读订阅，没遥控过也显示（起不来就 None，不影响遥控）。"""
        if getattr(self, "_chassis_odom", None) is None and not getattr(self, "_chassis_odom_failed", False):
            try:
                self.reader()
                self._chassis_odom = self.backend.chassis_odom()
            except Exception:  # noqa: BLE001
                self._chassis_odom_failed = True
                self._chassis_odom = None
        return getattr(self, "_chassis_odom", None)

    def chassis_stop(self, reason: str = "停止") -> Dict[str, Any]:
        """急停 / 停止按钮用：没建过控制器（从没动过底盘）就什么都不用发。"""
        if self.profile.chassis is None or self._chassis_ctrl is None:
            return {"ok": True, "message": "底盘本来就没在动"}
        return {"ok": True, **self._chassis_ctrl.stop(reason)}

    def chassis_status(self) -> Optional[Dict[str, Any]]:
        if self.profile.chassis is None:
            return None
        if self._chassis_ctrl is not None:
            return self._chassis_ctrl.status()
        spec = self.profile.chassis
        odom = self._chassis_odom_reader()
        return {"available": False, "moving": False,
                "limit": {"lin_mps": spec.default_lin_mps, "yaw_rad_s": spec.default_yaw_rad_s,
                          "max_lin_mps": spec.max_lin_mps, "max_yaw_rad_s": spec.max_yaw_rad_s},
                "odom": odom.latest() if odom is not None else None, "note": "还没遥控过底盘"}

    def chassis_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        if path == "/api/chassis/stop":
            # 页面说明为什么停（松手 / 失焦 / 急停…），只用来显示
            return self.chassis_stop(str(body.get("reason") or "停止按钮")[:16])
        if self.profile.chassis is None:
            return {"ok": False, "error": f"{self.profile.name} 没有底盘"}
        self.require_motion()
        ctrl = self.chassis_ctrl()
        if path == "/api/chassis/move":
            return {"ok": True, **ctrl.command(float(body.get("x") or 0.0), float(body.get("yaw") or 0.0))}
        if path == "/api/chassis/limit":
            lin, yaw = body.get("lin_mps"), body.get("yaw_rad_s")
            return {"ok": True, **ctrl.set_limit(None if lin in (None, "") else float(lin),
                                                 None if yaw in (None, "") else float(yaw))}
        return {"ok": False, "error": "没有这个底盘接口"}
