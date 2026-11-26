"""碰撞检查：/api/collision/*（状态、开关与余量、三维用的碰撞体）。

闸门本身在 ArmLoop.set_target 里（control/arm_loop.py），所有给手臂下目标的入口都经过它。
"""

from __future__ import annotations

import threading
from typing import Any, Dict, Optional

from robot_platform.collision import setup


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
        self._collision_guard = setup.build_guard(self.profile, self.settings.base_link,
                                                  self._collision_body_q, self._collision_tools)

    def _collision_tools(self) -> tuple:
        """各手当前生效的 TCP（腕末端连杆系下的点）；没生效的手不给，末端就只有腕。"""
        return setup.active_tools(self.profile)

    def _collision_body_q(self) -> Dict[str, float]:
        """身体部分的关节：腰（按实测）、立柱各级（按实测高度）。拿不到就按 0。"""
        return setup.body_q(self.profile, self._reader, self._hispeed)

    def attach_collision_guard(self, core) -> None:
        core.set_collision_guard(self.collision_guard())

    def _collision_arms_q(self) -> Optional[Dict[str, Any]]:
        """两只手的实测关节角。"""
        try:
            reader = self.reader()
            return {n: list(reader.read_arm(lay).q) for n, lay in self.profile.arms.items()}
        except Exception:  # noqa: BLE001
            return None

    def _collision_remote(self):
        """ipc：闸门在运控核心进程里，/api/collision/* 转给它。"""
        if getattr(self, "motion_core_mode", "inprocess") != "ipc":
            return None
        return self.remote_core()

    def collision_status(self, with_distance: bool = True) -> Dict[str, Any]:
        remote = self._collision_remote()
        if remote is not None:
            return remote.collision("/api/collision/status", {})["collision"]
        g = self.collision_guard()
        # 控制循环上挂的是不是这个闸门（没挂上 = 下目标时不检查）；只读模式没有运控核心
        core = getattr(self, "motion_core", None)
        attached = None if core is None else (core.collision_guard is g)
        arms_q = self._collision_arms_q() if (with_distance and g is not None) else None
        return setup.guard_status(g, self.profile.name, arms_q, gate_attached=attached)

    def collision_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        remote = self._collision_remote()
        if remote is not None:
            if path == "/api/collision/config" and getattr(self, "motion_core", None) is None:
                return {"ok": False, "error": "只读模式不能改运控核心的碰撞检查设置",
                        "collision": self.collision_status()}
            return remote.collision(path, body)
        return setup.guard_request(self.collision_guard(), path, body, self.collision_status)
