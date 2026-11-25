"""手眼采集接口：保存配置、建 / 选会话、补拍一帧、按姿态集自动走点采集、停止。"""

from __future__ import annotations

import threading
from typing import Any, Dict

import numpy as np

from robot_platform.calib.handeye import session as he_session
from robot_platform.calib.handeye.capture_task import CaptureTask
from robot_platform.calib.handeye import color_intrinsics as he_color_intr
from robot_platform.calib.handeye.config import camera_choices, load_config, save_config
from robot_platform.calib.handeye.detection import parse_board_size
from robot_platform.calib.payload import identify as iden, poses as posesets
from robot_platform.control.arm_loop import PermissionErrorMotion
from robot_platform.core import instance as instances


class CaptureMixin:
    def _save_config(self, body: Dict[str, Any]) -> Dict[str, Any]:
        cfg = load_config()
        for key in ("camera_source", "camera_host", "teleimager_path", "intrinsics", "board_size", "eye", "session"):
            if key in body and body[key] is not None:
                cfg[key] = body[key]
        if body.get("camera_source"):
            cfg.pop("camera", None)  # 显式给了来源：不再按下拉框选的相机
        if "color_intrinsics" in body:
            name = str(body.get("color_intrinsics") or "")
            if name:
                he_color_intr.load(name)  # 不存在 / 无效直接报错
            cfg["cloud_color_intrinsics"] = name
        if body.get("camera"):
            cid = str(body["camera"])
            if cid not in {c["id"] for c in camera_choices()}:
                raise ValueError(f"本机型没有相机 {cid!r}")
            cfg["camera"] = cid
            cfg.pop("camera_source", None)  # 下拉框选的相机优先，旧的来源写法作废
        if body.get("intrinsics"):
            inst = instances.active()
            cfg["intrinsics"] = inst.to_stored(inst.from_stored(str(body["intrinsics"])))
        if "square_size_mm" in body:
            raw = body["square_size_mm"]
            cfg["square_size_mm"] = None if raw in ("", None) else float(raw)
        if cfg.get("board_size"):
            parse_board_size(str(cfg["board_size"]))
        if cfg.get("eye") not in ("left", "right", "both"):
            raise ValueError("eye 只能是 left、right 或 both")
        save_config(cfg)
        name = str(cfg.get("session") or "")
        if name:
            path = he_session.session_dir(name)
            if path.is_dir():
                he_session.write_meta(path, str(cfg.get("board_size") or "11x8"))
        return self.status()

    def _session(self, body: Dict[str, Any]) -> Dict[str, Any]:
        self._save_config({k: v for k, v in body.items() if k != "create"})
        cfg = load_config()
        if body.get("create"):
            arm = str(body.get("arm") or he_session.ARM)
            path = he_session.create_session(str(cfg.get("board_size") or "11x8"), arm=arm)
            cfg["session"] = path.name
            save_config(cfg)
        elif body.get("name"):
            path = he_session.session_dir(str(body["name"]))
            if not path.is_dir():
                raise FileNotFoundError(f"没有会话 {body['name']}")
            cfg["session"] = path.name
            save_config(cfg)
        return self.status()

    def _capture(self, body: Dict[str, Any]) -> Dict[str, Any]:
        cfg = load_config()
        if body:
            self._save_config(body)
            cfg = load_config()
        board = parse_board_size(str(cfg.get("board_size") or "11x8"))
        name = str(cfg.get("session") or "")
        if not name or not he_session.session_dir(name).is_dir():
            path = he_session.create_session(f"{board[0]}x{board[1]}")
            cfg["session"] = path.name
            save_config(cfg)
        else:
            path = he_session.session_dir(name)
        meta = he_session.load_meta(path)
        arm = meta.get("arm") or str(body.get("arm") or he_session.ARM)
        cam = self._camera(cfg)
        frame = cam.grab(3.0)
        if frame is None:
            return {"ok": False, "error": cam.last_error or "没有画面", **self._public(cfg)}
        st = self.app.reader().read_arm(arm)
        q = np.asarray(st.q, dtype=float).reshape(-1)
        if q.size != 7 or not np.all(np.isfinite(q)):
            return {"ok": False, "error": f"关节角不是 7 个有限数: {q.tolist()}", **self._public(cfg)}
        try:
            idx = he_session.save_sample(path, frame.color, frame.right, q, arm,
                                         depth=frame.depth, camera_info=frame.info or None)
        except OSError as exc:
            return {"ok": False, "error": str(exc), **self._public(cfg)}
        he_session.write_meta(path, f"{board[0]}x{board[1]}", arm=arm)
        stt = self.status()
        stt["index"] = idx
        stt["q_rad"] = q.tolist()
        return stt

    def _run(self, body: Dict[str, Any]) -> Dict[str, Any]:
        if body:
            self._save_config({k: v for k, v in body.items() if k != "name"})
        cfg = load_config()
        motion = self.app.motion()
        with self.app.payload._id_lock:
            if self.app.payload._id.get("running"):
                raise RuntimeError("负载测量或对比正在走点")
        with self._lock:
            if self._running:
                raise RuntimeError("手眼采集已经在走点")
        name = str(body.get("name") or "").strip()
        if not name:
            raise ValueError("先在姿态集里选一套")
        poses = posesets.load_set(name)
        board = str(cfg.get("board_size") or "11x8")
        parse_board_size(board)
        still = dict(getattr(self.app.payload, "still", None) or iden.DEFAULT_STILL)
        # 租约：占住姿态集那只手直到任务结束（没接管、在拖拽、被占用、急停锁定时这里就拒绝）
        lease = motion.lease_arm(str(poses.get("arm") or he_session.ARM), "手眼采集")

        def done(result: Dict[str, Any]) -> None:
            if result.get("session"):
                cfg2 = load_config()
                cfg2["session"] = result["session"]
                save_config(cfg2)
            with self._lock:
                self._running = False

        try:
            task = CaptureTask(lease, self._camera(cfg), poses, board, still, done)
        except BaseException:
            lease.release()
            raise
        with self._lock:
            self._task = task
            self._running = True
        threading.Thread(target=task.run, name="handeye-capture", daemon=True).start()
        return self.status()

    def stop_capture(self) -> None:
        """「停止」按钮：叫停正在走点的手眼采集。急停不用它：急停撤销租约，任务自己就停了。"""
        task = self._task
        if task is not None:
            task.request_stop()

    def _stop(self) -> Dict[str, Any]:
        self.stop_capture()
        return self.status()

    @staticmethod
    def _public(cfg: Dict[str, Any]) -> Dict[str, Any]:
        return {"session": cfg.get("session") or "", "sessions": he_session.list_sessions()}
