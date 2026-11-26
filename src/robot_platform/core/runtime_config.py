"""运行时生效配置：双目/手眼、左右重力、TCP。可锁定，防止其它板块改动。"""

from __future__ import annotations

import json
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional

from robot_platform.core import instance as instances

# 配置里「不用 TCP（末端为空）」的取值
NO_TCP = "__none__"


def lock_path() -> Path:
    return instances.path("runtime_lock")

# 锁定后禁止从其它入口改这些配置
_GUARD_PATHS = {
    "/api/handeye/load",
    "/api/handeye/solve",
    "/api/identify/apply",
    "/api/identify/payload/apply",
    "/api/tcp/load",
    "/api/runtime-config/stereo",
    "/api/runtime-config/gravity",
    "/api/runtime-config/tcp",
}


class ConfigLockedError(RuntimeError):
    pass


class RuntimeConfig:
    def __init__(self, app) -> None:
        self.app = app
        self._lock = threading.Lock()
        self._locked = self._read_locked()

    @staticmethod
    def _read_locked() -> bool:
        try:
            if lock_path().is_file():
                return bool(json.loads(lock_path().read_text(encoding="utf-8")).get("locked"))
        except (OSError, json.JSONDecodeError, TypeError):
            pass
        return False

    def _write_locked(self, locked: bool) -> None:
        lock_path().parent.mkdir(parents=True, exist_ok=True)
        lock_path().write_text(json.dumps({"locked": bool(locked)}, indent=2), encoding="utf-8")

    @property
    def locked(self) -> bool:
        with self._lock:
            return self._locked

    def set_locked(self, locked: bool) -> Dict[str, Any]:
        with self._lock:
            self._locked = bool(locked)
            self._write_locked(self._locked)
        return self.snapshot()

    def require_unlocked(self) -> None:
        if self.locked:
            raise ConfigLockedError("配置已锁定，请到「控制 → 配置」解锁后再改")

    def guard_path(self, path: str, body: Optional[Dict[str, Any]] = None) -> None:
        """其它板块改配置时调用；已锁定则抛错。双目 yaml 始终禁止单独改。"""
        if path == "/api/handeye/config" and body and "intrinsics" in body:
            raise ConfigLockedError("双目 yaml 已与手眼绑定，请改选手眼结果")
        if not self.locked:
            return
        if path in _GUARD_PATHS:
            raise ConfigLockedError("配置已锁定，请到「控制 → 配置」解锁后再改")
        if path == "/api/handeye/config" and body and "active_result" in body:
            raise ConfigLockedError("配置已锁定，请到「控制 → 配置」解锁后再改")
        if path == "/api/handeye/config" and body and any(k in body for k in ("camera", "camera_source", "color_intrinsics")):
            raise ConfigLockedError("配置已锁定，不能换相机或彩色内参；请到「控制 → 配置」解锁后再改")
        if path == "/api/handeye/color-intrinsics/delete" and body:
            from robot_platform.calib.handeye.config import load_config
            if load_config().get("cloud_color_intrinsics") == str(body.get("name") or ""):
                raise ConfigLockedError("配置已锁定，不能删掉正在用的彩色内参")
        if path == "/api/tcp/edit-save" and body and str(body.get("mode") or "") == "overwrite":
            from robot_platform.calib import tcp as tcp_calib
            name = str(body.get("src") or "")
            if name and name in tcp_calib._read_active_map().values():
                raise ConfigLockedError("配置已锁定，不能覆盖正在应用的 TCP；可以另存，或先解锁")
        if path == "/api/handeye/delete" and body:
            from robot_platform.calib.handeye.config import load_config
            if load_config().get("active_result") == str(body.get("name") or ""):
                raise ConfigLockedError("配置已锁定，不能删掉当前手眼结果")

    @staticmethod
    def list_stereo_yamls() -> List[Dict[str, str]]:
        root = instances.path("handeye_sessions")
        out: List[Dict[str, str]] = []
        if not root.is_dir():
            return out
        for p in sorted(root.glob("*/stereo_calibration.yaml"), key=lambda x: x.stat().st_mtime, reverse=True):
            rel = str(p.resolve())
            out.append({"path": rel, "label": f"{p.parent.name}/stereo_calibration.yaml"})
        return out

    @staticmethod
    def _gravity_side(arm: str) -> Dict[str, Any]:
        from robot_platform.calib.payload import identify as iden
        d = iden.load_payload(arm) or {}
        name = d.get("name") or ""
        mass = d.get("mass_kg")
        com = d.get("com_m")
        path = iden.payload_path(arm)
        return {
            "arm": arm,
            "name": name,
            "mass_kg": mass,
            "com_m": com,
            "file": str(path) if path.exists() else "",
            "updated": d.get("updated") or d.get("applied_at") or "",
        }

    def snapshot(self) -> Dict[str, Any]:
        from robot_platform.calib.handeye.config import load_config
        from robot_platform.calib.payload import identify as iden

        cfg = load_config()
        he = self.app.handeye.status()
        tcp = self.app.tcp.status()
        ctrl = self.app.ctrl_status()
        from robot_platform.calib.handeye import results as he_results

        active = str(cfg.get("active_result") or "")
        intr = str(cfg.get("intrinsics") or "")
        bound_yaml = ""
        if active:
            try:
                doc = he_results.load_saved(active)
                bound_yaml = str(doc.get("intrinsics_yaml") or "")
                cand = he_results.results_dir() / f"{active}.yaml"
                if not bound_yaml and cand.is_file():
                    bound_yaml = str(cand)
            except (OSError, FileNotFoundError, json.JSONDecodeError, ValueError):
                bound_yaml = ""
        if bound_yaml:
            intr = bound_yaml
        saved = []
        for item in (he.get("saved") or []):
            row = dict(item)
            try:
                doc = he_results.load_saved(item["name"])
                yp = str(doc.get("intrinsics_yaml") or "")
                if not yp:
                    cand = he_results.results_dir() / f"{item['name']}.yaml"
                    if cand.is_file():
                        yp = str(cand)
                row["intrinsics_yaml"] = yp
            except (OSError, FileNotFoundError, json.JSONDecodeError, ValueError):
                row["intrinsics_yaml"] = ""
            saved.append(row)
        return {
            "ok": True,
            "locked": self.locked,
            "stereo": {
                "intrinsics": intr,
                "active_result": active,
                "handeye_saved": saved,
                "session": cfg.get("session") or "",
                "bound": True,
            },
            "gravity": {
                "left": self._gravity_side("left"),
                "right": self._gravity_side("right"),
                "profiles": iden.list_profiles(),
                # 两只手各自正在用的前馈（控制循环里实时的那份）
                "live": {
                    name: {"name": a.get("payload_name") or "", "mass_kg": a.get("payload_mass_kg")}
                    for name, a in (ctrl.get("arms") or {}).items()
                },
            },
            "tcp": {
                "active_name": tcp.get("active_name") or "",
                "arm": tcp.get("arm"),
                "active_left": tcp.get("active_left") or "",
                "active_right": tcp.get("active_right") or "",
                "left": self._tcp_side("left", tcp),
                "right": self._tcp_side("right", tcp),
                "saved": tcp.get("saved") or [],
                "has_result": bool(tcp.get("result")),
            },
            "work_pose": self.app.work_pose.status(tcp.get("active_name") or ""),
        }

    @staticmethod
    def _tcp_side(arm: str, tcp_status: Dict[str, Any]) -> Dict[str, Any]:
        name = str(tcp_status.get("active_left" if arm == "left" else "active_right") or "")
        meta = next((s for s in (tcp_status.get("saved") or []) if s.get("name") == name), None) or {}
        return {
            "arm": arm,
            "name": name,
            "tip_link": meta.get("tip_link") or "",
            "residual_max_mm": meta.get("residual_max_mm"),
            "note": meta.get("note") or "",
        }

    def apply_stereo(self, *, active_result: Optional[str] = None, **_ignored) -> Dict[str, Any]:
        """只选手眼结果；自带双目 yaml 一并生效，忽略单独传的 intrinsics。"""
        self.require_unlocked()
        from robot_platform.calib.handeye.config import load_config, save_config
        from robot_platform.calib.handeye import results as he_results

        if active_result is None:
            raise ValueError("先选手眼结果")
        name = str(active_result).strip()
        cfg = load_config()
        if not name:
            cfg["active_result"] = ""
            save_config(cfg)
            self.app.handeye._matcher = None
            self.app.handeye._matcher_key = ""
            return self.snapshot()
        doc = he_results.load_saved(name)
        self.app.handeye._bind_handeye_doc(cfg, doc)
        save_config(cfg)
        self.app.handeye._matcher = None
        self.app.handeye._matcher_key = ""
        return self.snapshot()

    def apply_gravity(self, arm: str, name: str) -> Dict[str, Any]:
        self.require_unlocked()
        from robot_platform.calib.payload import identify as iden

        arm = "left" if arm == "left" else "right"
        name = str(name or "").strip()
        if not name:
            raise ValueError("先选重力补偿文件")
        d = iden.load_profile(name)
        if d.get("arm") not in (None, arm):
            side = "左" if d.get("arm") == "left" else "右"
            want = "左" if arm == "left" else "右"
            raise ValueError(f"「{name}」是{side}臂补偿，不能配到{want}臂")
        saved = dict(d)
        saved["arm"] = arm
        saved["name"] = d.get("name") or name
        iden.save_payload(arm, saved)
        if getattr(self.app, "motion_core", None) is not None:  # 两只手都在控制循环里，补偿立刻进这只手的前馈
            self.app.payload.apply_payload_dict(saved, name=saved["name"])
        return self.snapshot()

    def apply_tcp(self, name: str, arm: Optional[str] = None) -> Dict[str, Any]:
        self.require_unlocked()
        name = str(name or "").strip()
        if name == NO_TCP:
            # 这只手不用 TCP：末端为空（碰撞检查里末端只算腕）
            if arm not in ("left", "right"):
                raise ValueError("先选左臂或右臂")
            self.app.tcp.clear_active(arm)
            return self.snapshot()
        if not name:
            raise ValueError("先选 TCP 结果")
        st = self.app.tcp.load(name)
        got = str(st.get("arm") or "")
        want = "left" if arm == "left" else ("right" if arm == "right" else "")
        if want and got and got != want:
            side = "左" if got == "left" else "右"
            raise ValueError(f"「{name}」是{side}臂 TCP，请选到对应侧")
        return self.snapshot()
