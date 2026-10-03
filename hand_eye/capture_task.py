"""按已有姿态集走点，每点停稳后存一对图和该臂关节角。发令仍走 arm_control 的控制循环。"""

from __future__ import annotations

import threading
from typing import Any, Callable, Dict, Optional

import numpy as np

import session as he_session

# 手眼比负载走点更紧：0.003 rad ≈ 0.17°。板子在腕前约 25 cm 时，这项约 0.8 mm。
HAND_EYE_DRIFT_RAD = 0.003


class CaptureTask:
    def __init__(self, loop, camera, poses_cfg: Dict[str, Any], board_size: str,
                 still: Dict[str, float], on_done: Callable[[Dict[str, Any]], None]):
        self.loop = loop
        self.camera = camera
        self.poses_cfg = poses_cfg
        self.board_size = board_size
        self.still = dict(still)
        self.still["still_drift_rad"] = HAND_EYE_DRIFT_RAD
        self.on_done = on_done
        self.stop = threading.Event()
        self.phase = "待命"
        self.progress = [0, 0]
        self.log_lines = []
        self.error: Optional[str] = None
        self.session_name = ""
        self.saved = 0

    def _log(self, msg: str) -> None:
        self.phase = msg
        self.log_lines.append(msg)
        self.log_lines = self.log_lines[-24:]

    def run(self) -> None:
        import identify as iden

        path = None
        try:
            poses = self.poses_cfg.get("poses") or []
            if len(poses) < 3:
                raise ValueError("姿态集至少 3 个点，建议 8 个以上并且姿态拉开")
            arm = self.poses_cfg["arm"]
            if self.loop.arm != arm:
                raise RuntimeError(f"控制臂是{self.loop.arm}，姿态集是{arm}。先释放，把控制臂换成这一侧")
            st = self.loop.status()
            if not st.get("engaged") or st.get("float"):
                raise RuntimeError("先接管，并退出拖拽")
            probe = self.camera.grab_pair(3.0)
            if probe is None:
                raise RuntimeError(self.camera.last_error or "没有画面，还没开始走点")
            path = he_session.create_session(self.board_size, arm=arm, poseset=str(self.poses_cfg.get("name") or ""))
            self.session_name = path.name
            total = len(poses)
            self.progress = [0, total]
            self._log(
                f"采集 {self.poses_cfg.get('name')} · {arm} · {total} 点 → {path.name} · "
                f"漂移阈 {self.still['still_drift_rad']} rad"
            )
            for i, pose in enumerate(poses):
                if self.stop.is_set():
                    raise RuntimeError("已停止")
                name = pose.get("name") or f"p{i:02d}"
                self._log(f"{name} 走点")
                settled = iden.wait_settle(self.loop, pose["q"], self.stop, self.still)
                if self.stop.is_set():
                    raise RuntimeError("已停止")
                pair = self.camera.grab_pair(3.0)
                if pair is None:
                    raise RuntimeError(self.camera.last_error or f"{name} 没有画面")
                samp = self.loop.arm_sample(wait=False)
                q = np.asarray(samp["q_meas"], float).reshape(-1)
                if q.size != 7 or not np.all(np.isfinite(q)):
                    raise RuntimeError(f"{name} 关节角无效")
                he_session.save_sample(path, pair[0], pair[1], q, arm, pose_name=str(name))
                self.saved += 1
                self.progress = [self.saved, total]
                extra = "（超时，未压进漂移阈）" if settled.get("timed_out") else ""
                self._log(f"{name} 已存 {self.saved}/{total}{extra}")
            home = self.poses_cfg.get("home_q")
            if home and not self.stop.is_set():
                self._log("回 home")
                iden.wait_settle(self.loop, home, self.stop, self.still)
            self.phase = "完成"
            self.on_done({"ok": True, "session": self.session_name, "n": self.saved})
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.phase = "失败"
            self.on_done({
                "ok": False,
                "error": self.error,
                "session": self.session_name,
                "n": self.saved,
            })
