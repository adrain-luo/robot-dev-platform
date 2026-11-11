"""手眼 / 相机接口 HandEyeAPI：由采集、求解、相机点云、调试口几个 mixin 组成，这里只有状态与路由。

走点发令调用 control.arm_loop 的控制循环，关节从已有的 LowStateReader 读。
"""

from __future__ import annotations

import threading
from typing import Any, Dict, Optional

from robot_platform.calib.handeye import color_intrinsics as he_color_intr, results as he_results, session as he_session
from robot_platform.calib.handeye.capture_task import CaptureTask
from robot_platform.calib.handeye.config import camera_choices, effective_camera, load_config
from robot_platform.perception.stereo_match import StereoMatcher
from robot_platform.perception.cloud_api import CameraCloudMixin
from robot_platform.perception.debug_port import DebugPortMixin
from robot_platform.calib.handeye.capture_api import CaptureMixin
from robot_platform.calib.handeye.solve_api import SolveMixin


class HandEyeAPI(CameraCloudMixin, DebugPortMixin, CaptureMixin, SolveMixin):
    def __init__(self, app) -> None:
        self.app = app
        self._lock = threading.Lock()
        self._cam = None  # HeadCamera / RosRgbdCamera / MockRgbdCamera
        self._cam_key = ""
        self._matcher: Optional[StereoMatcher] = None
        self._matcher_key = ""
        self._debug_matcher: Optional[StereoMatcher] = None
        self._debug_key = ""
        self._task: Optional[CaptureTask] = None
        self._running = False

    def status(self) -> Dict[str, Any]:
        cfg = load_config()
        name = str(cfg.get("session") or "")
        count = 0
        if name:
            left = he_session.session_dir(name) / "left"
            if left.is_dir():
                count = len(list(left.glob("*.jpg")))
        cam = self._cam
        eff = effective_camera(cfg, mock=self._is_mock())
        with self._lock:
            task = self._task
            running = self._running
        progress = list(task.progress) if task else [0, 0]
        phase = task.phase if task else ""
        if running and task and task.saved:
            count = task.saved
        return {
            "ok": True,
            "arm": he_session.ARM,
            "tip_link": he_session.tip_link(),
            "base_link": he_session.base_link(),
            "camera": eff["id"],
            "camera_label": eff["label"],
            "cameras": camera_choices(),
            "camera_source": eff["camera_source"],
            "camera_kind": eff["camera_kind"],
            "ros_topics": eff["ros_topics"],
            "camera_host": eff["camera_host"],
            "teleimager_path": eff["teleimager_path"],
            "camera_error": "" if cam is None else cam.last_error,
            "intrinsics": cfg.get("intrinsics") or "",
            "board_size": cfg.get("board_size") or "11x8",
            "square_size_mm": cfg.get("square_size_mm"),
            "eye": cfg.get("eye") or "both",
            "session": (task.session_name if task and task.session_name else name),
            "count": count,
            "sessions": he_session.list_sessions(),
            "saved": he_results.list_saved(),
            "active_result": cfg.get("active_result") or "",
            "color_intrinsics": he_color_intr.list_files(),
            "config_locked": bool(getattr(getattr(self.app, "runtime", None), "locked", False)),
            "cloud_color_intrinsics": cfg.get("cloud_color_intrinsics") or "",
            "running": running,
            "phase": phase,
            "progress": progress,
            "error": None if task is None else task.error,
            "log": [] if task is None else list(task.log_lines[-8:]),
        }

    def handle_get(self, path: str, query: str) -> Optional[Dict[str, Any]]:
        if path == "/api/handeye":
            return self.status()
        return None

    def handle_post(self, path: str, body: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        if path == "/api/handeye/config":
            return self._save_config(body)
        if path == "/api/handeye/session":
            return self._session(body)
        if path == "/api/handeye/capture":
            return self._capture(body)
        if path == "/api/handeye/run":
            return self._run(body)
        if path == "/api/handeye/stop":
            return self._stop()
        if path == "/api/handeye/solve":
            return self._solve(body)
        if path == "/api/handeye/save":
            return self._save_named(body)
        if path == "/api/handeye/load":
            return self._load_named(body)
        if path == "/api/handeye/delete":
            return self._delete_named(body)
        if path == "/api/handeye/match":
            return self._match(body)
        if path == "/api/handeye/cloud-save":
            return self._save_cloud()
        if path == "/api/handeye/color-intrinsics/upload":
            return self._color_intr_upload(body)
        if path == "/api/handeye/color-intrinsics/delete":
            return self._color_intr_delete(body)
        if path == "/api/detect/grab":
            return self._grab_detect(body)
        return None
