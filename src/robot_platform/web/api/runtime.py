"""运行配置：/api/runtime-config/*（手眼、重力、TCP、工作姿态成套调用与锁定）。"""

from __future__ import annotations

from typing import Any, Dict
import json

from robot_platform.control.arm_loop import PermissionErrorMotion
from robot_platform.core.runtime_config import ConfigLockedError


class RuntimeMixin:
    def runtime_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        try:
            if path == "/api/runtime-config/lock":
                return self.runtime.set_locked(bool(body.get("locked")))
            if path == "/api/runtime-config/stereo":
                return self.runtime.apply_stereo(active_result=body.get("active_result"))
            if path == "/api/runtime-config/gravity":
                return self.runtime.apply_gravity(str(body.get("arm") or "right"), str(body.get("name") or ""))
            if path == "/api/runtime-config/tcp":
                out = self.runtime.apply_tcp(
                    str(body.get("name") or ""),
                    arm=body.get("arm"),
                )
                arm = str(body.get("arm") or self.tcp.arm or "")
                if arm in ("left", "right"):
                    # 只看这只手：锁的是它、TCP 又换了（或不用了）就解锁；另一只手的锁不动
                    nm = str(body.get("name") or "")
                    self.work_pose_lock_for(arm, "" if nm == "__none__" else nm)
                return out
            if path == "/api/runtime-config/work-pose":
                # 不受配置锁定限制：确认后到位并锁定工作姿态
                return self.work_pose_load(
                    str(body.get("tcp_name") or self._default_tcp_name(body)),
                    str(body.get("name") or ""),
                    str(body.get("arm") or ""),
                )
            if path == "/api/runtime-config/pose-ik-mode":
                # 不受配置锁定：随时切换锁定姿态下的 IK 策略
                out = self.work_pose.set_ik_mode(str(body.get("mode") or ""))
                snap = self.runtime.snapshot()
                snap["work_pose"] = out
                snap["ok"] = True
                from robot_platform.tasks.work_pose import IK_LABELS

                snap["message"] = f"已切到{IK_LABELS.get(out.get('ik_mode'), out.get('ik_mode'))} IK"
                return snap
            if path == "/api/runtime-config":
                return self.runtime.snapshot()
            return {"ok": False, "error": "没有这个配置接口"}
        except ConfigLockedError as exc:
            out = self.runtime.snapshot()
            out["ok"] = False
            out["error"] = str(exc)
            return out
        except (ValueError, FileNotFoundError, OSError, KeyError, json.JSONDecodeError, PermissionErrorMotion, RuntimeError) as exc:
            out = self.runtime.snapshot()
            out["ok"] = False
            out["error"] = str(exc)
            return out
