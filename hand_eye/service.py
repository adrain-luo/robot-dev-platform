"""手眼采集与求解。走点发令调用 arm_control 的控制循环，关节从已有的 LowStateReader 读。"""

from __future__ import annotations

import json
import threading
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np

import session as he_session
import results as he_results
from stereo_match import StereoMatcher, colorize_disp, write_ply
from camera import HeadCamera
from capture_task import CaptureTask
from controller.arm_loop import PermissionErrorMotion
from detection import detect_corners, parse_board_size
from model.urdf_fk import URDFRobotModel
from solve_core import SolveOptions, load_intrinsics, object_points_m, solve_eye, stereo_cross_check
import identify as iden
import poses as posesets

ROOT = Path(__file__).resolve().parent
CONFIG_PATH = ROOT / "config.json"

DEFAULTS: Dict[str, Any] = {
    "camera_source": "teleimager",
    "camera_host": "192.168.123.164",
    "teleimager_path": "/home/robot/yx/teleimager_src_copy",
    "intrinsics": "",
    "board_size": "11x8",
    "square_size_mm": None,
    "session": "",
    "eye": "both",
}


def load_config() -> Dict[str, Any]:
    cfg = dict(DEFAULTS)
    if CONFIG_PATH.is_file():
        try:
            raw = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            raw = {}
        if isinstance(raw, dict):
            cfg.update(raw)
    return cfg


def save_config(cfg: Dict[str, Any]) -> None:
    CONFIG_PATH.write_text(json.dumps(cfg, indent=2, ensure_ascii=False), encoding="utf-8")


class HandEyeAPI:
    def __init__(self, app) -> None:
        self.app = app
        self._lock = threading.Lock()
        self._cam: Optional[HeadCamera] = None
        self._cam_key = ""
        self._matcher: Optional[StereoMatcher] = None
        self._matcher_key = ""
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
            "tip_link": he_session.TIP_LINK,
            "base_link": he_session.BASE_LINK,
            "camera_source": cfg.get("camera_source"),
            "camera_host": cfg.get("camera_host"),
            "teleimager_path": cfg.get("teleimager_path") or "",
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
        return None

    def preview_jpeg(self) -> Tuple[Optional[bytes], str]:
        cfg = load_config()
        cam = self._camera(cfg)
        pair = cam.grab_pair(2.0)
        if pair is None:
            return None, cam.last_error or "没有画面"
        left, right = pair
        try:
            board = parse_board_size(str(cfg.get("board_size") or "11x8"))
        except ValueError as exc:
            return None, str(exc)
        _lf, left_vis = detect_corners(left, board, draw=True)
        _rf, right_vis = detect_corners(right, board, draw=True)
        vis = _stack_eyes(left_vis, right_vis)
        ok, buf = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ok:
            return None, "编码预览失败"
        return buf.tobytes(), ""

    def _match(self, body: Dict[str, Any]) -> Dict[str, Any]:
        import base64
        cfg = load_config()
        yaml_path = self._calib_yaml(cfg)
        matcher = self._matcher_for(yaml_path)
        cam = self._camera(cfg)
        pair = cam.grab_pair(2.0)
        if pair is None:
            return {"ok": False, "error": cam.last_error or "没有画面"}
        rect_l, disp, points_m = matcher.match(pair[0], pair[1])
        valid = np.isfinite(points_m).all(axis=2) & (disp > 0.5)
        z = points_m[:, :, 2]
        valid &= (z > 0.25) & (z < 2.0)
        n_valid = int(valid.sum())
        if n_valid < 50:
            return {"ok": False, "error": f"有效点只有 {n_valid}，匹配没有对上"}
        zz = z[valid]
        pts = points_m[valid]
        cols = np.ascontiguousarray(rect_l[valid][:, ::-1])
        T = self._left_cam_to_torso(cfg)
        if T is None:
            return {"ok": False, "error": "没有左目到躯干的外参。先调用已保存的手眼结果，点云才能画到模型上"}
        cloud_dir = he_session.DATA_DIR / "clouds"
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        ply = cloud_dir / f"{stamp}_cam.ply"
        n_ply = write_ply(ply, pts, cols[:, ::-1])
        homo = np.concatenate([pts, np.ones((pts.shape[0], 1))], axis=1)
        pts_t = (T @ homo.T).T[:, :3]
        torso_ply = cloud_dir / f"{stamp}_torso.ply"
        write_ply(torso_ply, pts_t, cols[:, ::-1])
        cloud = {
            "xyz": np.round(pts_t, 4).reshape(-1).tolist(),
            "rgb": cols.reshape(-1).astype(int).tolist(),
        }
        vis = colorize_disp(disp)
        ok, buf = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ok:
            return {"ok": False, "error": "视差图编码失败"}
        return {
            "ok": True,
            "valid": n_valid,
            "saved": n_ply,
            "depth_m": {"median": float(np.median(zz)), "near": float(zz.min()), "far": float(np.percentile(zz, 95))},
            "ply_cam": str(ply),
            "ply_torso": None if torso_ply is None else str(torso_ply),
            "cloud": cloud,
            "disparity_jpeg": base64.b64encode(buf.tobytes()).decode("ascii"),
        }

    def _calib_yaml(self, cfg: Dict[str, Any]) -> Path:
        name = str(cfg.get("active_result") or "")
        if name:
            p = he_results.RESULTS_DIR / f"{name}.yaml"
            if p.is_file():
                return p
        intr = str(cfg.get("intrinsics") or "")
        if intr and Path(intr).is_file():
            return Path(intr)
        raise FileNotFoundError("先在手眼卡片里调用一组已保存的标定")

    def _matcher_for(self, yaml_path: Path) -> StereoMatcher:
        key = str(yaml_path.resolve())
        if self._matcher is None or self._matcher_key != key:
            self._matcher = StereoMatcher(yaml_path)
            self._matcher_key = key
        return self._matcher

    def _left_cam_to_torso(self, cfg: Dict[str, Any]) -> Optional[np.ndarray]:
        name = str(cfg.get("active_result") or "")
        if not name:
            return None
        try:
            doc = he_results.load_saved(name)
        except (OSError, FileNotFoundError, ValueError):
            return None
        left = (doc.get("result") or {}).get("left") or {}
        T = left.get("T_cam2base")
        if not T:
            return None
        return np.asarray(T, dtype=float)

    def close(self) -> None:
        if self._cam is not None:
            self._cam.close()
            self._cam = None

    def _camera(self, cfg: Dict[str, Any]) -> HeadCamera:
        key = f"{cfg.get('camera_source')}|{cfg.get('camera_host')}|{cfg.get('teleimager_path')}"
        with self._lock:
            if self._cam is None or key != self._cam_key:
                if self._cam is not None:
                    self._cam.close()
                self._cam = HeadCamera(
                    source=str(cfg.get("camera_source") or "teleimager"),
                    host=str(cfg.get("camera_host") or "192.168.123.164"),
                    teleimager_path=str(cfg.get("teleimager_path") or ""),
                )
                self._cam_key = key
            return self._cam

    def _save_config(self, body: Dict[str, Any]) -> Dict[str, Any]:
        cfg = load_config()
        for key in ("camera_source", "camera_host", "teleimager_path", "intrinsics", "board_size", "eye", "session"):
            if key in body and body[key] is not None:
                cfg[key] = body[key]
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
            arm = self.app.loop.arm if self.app.loop is not None else he_session.ARM
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
        arm = meta.get("arm") or (self.app.loop.arm if self.app.loop is not None else he_session.ARM)
        if meta.get("arm") and self.app.loop is not None and self.app.loop.arm != meta["arm"]:
            raise PermissionErrorMotion(f"这个会话是{meta['arm']}臂，当前控制臂是{self.app.loop.arm}")
        cam = self._camera(cfg)
        pair = cam.grab_pair(3.0)
        if pair is None:
            return {"ok": False, "error": cam.last_error or "没有画面", **self._public(cfg)}
        st = self.app.reader().read_arm(arm)
        q = np.asarray(st.q, dtype=float).reshape(-1)
        if q.size != 7 or not np.all(np.isfinite(q)):
            return {"ok": False, "error": f"关节角不是 7 个有限数: {q.tolist()}", **self._public(cfg)}
        try:
            idx = he_session.save_sample(path, pair[0], pair[1], q, arm)
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
        loop = self.app.require_loop()
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
        if loop.arm != poses.get("arm"):
            raise PermissionErrorMotion(f"当前控制臂是{loop.arm}，姿态集是{poses.get('arm')}。先释放，把控制臂换成这一侧")
        st = loop.status()
        if not st.get("engaged") or st.get("float"):
            raise RuntimeError("先接管，并退出拖拽")
        board = str(cfg.get("board_size") or "11x8")
        parse_board_size(board)
        still = dict(getattr(self.app.payload, "still", None) or iden.DEFAULT_STILL)

        def done(result: Dict[str, Any]) -> None:
            if result.get("session"):
                cfg2 = load_config()
                cfg2["session"] = result["session"]
                save_config(cfg2)
            with self._lock:
                self._running = False

        task = CaptureTask(loop, self._camera(cfg), poses, board, still, done)
        with self._lock:
            self._task = task
            self._running = True
        threading.Thread(target=task.run, name="handeye-capture", daemon=True).start()
        return self.status()

    def _stop(self) -> Dict[str, Any]:
        task = self._task
        if task is not None:
            task.stop.set()
        return self.status()

    def _solve(self, body: Dict[str, Any]) -> Dict[str, Any]:
        cfg = load_config()
        for key in ("board_size", "eye", "session"):
            if body.get(key):
                cfg[key] = body[key]
        if body.get("square_size_mm") not in (None, ""):
            cfg["square_size_mm"] = float(body["square_size_mm"])
        save_config(cfg)
        name = str(cfg.get("session") or "")
        if not name:
            raise ValueError("先选或新建一个采集会话")
        data_dir = he_session.session_dir(name)
        if not (data_dir / "joints").is_dir():
            raise FileNotFoundError(f"{name} 里没有 joints/")
        square = cfg.get("square_size_mm")
        if square is None or float(square) <= 0:
            raise ValueError("填棋盘格子边长（毫米）")
        board = parse_board_size(str(cfg.get("board_size") or "11x8"))
        eye = str(cfg.get("eye") or "both")
        log: list = []
        from intrinsics import calibrate_session
        fit = calibrate_session(data_dir, board, float(square), log=log)
        yaml_path = Path(fit["yaml"])
        cfg["intrinsics"] = str(yaml_path)
        save_config(cfg)
        urdf = self.app.settings.urdf
        meta = he_session.load_meta(data_dir)
        base = str(meta.get("base_link") or self.app.settings.base_link or he_session.BASE_LINK)
        tip = str(meta.get("tip_link") or he_session.TIP_LINK)
        opt = SolveOptions(
            base_link=base,
            tip_link=tip,
            square_size_mm=float(square),
            intrinsics=str(yaml_path),
        )
        model = URDFRobotModel(str(urdf), opt.base_link, opt.tip_link)
        objp = object_points_m(board, opt.square_size_mm)
        eyes = ["left", "right"] if eye == "both" else [eye]
        results = {}
        for one in eyes:
            K, dist = load_intrinsics(yaml_path, one)
            res = solve_eye(one, data_dir, K, dist, objp, board, model, opt, log=log)
            if res is None:
                continue
            results[one] = res
            (data_dir / f"handeye_result_{one}.json").write_text(
                json.dumps(res, indent=2, ensure_ascii=False), encoding="utf-8"
            )
        if not results:
            raise RuntimeError("没有一眼算出结果。看采集是否够、棋盘是否检出。\n" + "\n".join(log[-12:]))
        cross = stereo_cross_check(results)
        combined = dict(results)
        if cross:
            combined.update(cross)
            (data_dir / "handeye_result.json").write_text(
                json.dumps(combined, indent=2, ensure_ascii=False), encoding="utf-8"
            )
        stt = self.status()
        stt["intrinsics_fit"] = fit
        stt["result"] = combined
        stt["log"] = log[-16:]
        return stt

    def _save_named(self, body: Dict[str, Any]) -> Dict[str, Any]:
        cfg = load_config()
        name = str(body.get("name") or "").strip()
        session = str(body.get("session") or cfg.get("session") or "")
        if not session:
            raise ValueError("先选一次已经求解的采集会话")
        data_dir = he_session.session_dir(session)
        doc = he_results.save_named(name, data_dir)
        cfg["active_result"] = doc["name"]
        cfg["session"] = session
        save_config(cfg)
        stt = self.status()
        stt["intrinsics_fit"] = doc.get("intrinsics_fit")
        stt["result"] = doc.get("result")
        stt["active_result"] = doc["name"]
        return stt

    def _load_named(self, body: Dict[str, Any]) -> Dict[str, Any]:
        doc = he_results.load_saved(str(body.get("name") or ""))
        cfg = load_config()
        cfg["active_result"] = doc["name"]
        if doc.get("session"):
            cfg["session"] = doc["session"]
        save_config(cfg)
        stt = self.status()
        stt["intrinsics_fit"] = doc.get("intrinsics_fit")
        stt["result"] = doc.get("result")
        stt["active_result"] = doc["name"]
        return stt

    def _delete_named(self, body: Dict[str, Any]) -> Dict[str, Any]:
        name = str(body.get("name") or "")
        he_results.delete_saved(name)
        cfg = load_config()
        if cfg.get("active_result") == name:
            cfg["active_result"] = ""
            save_config(cfg)
        return self.status()

    @staticmethod
    def _public(cfg: Dict[str, Any]) -> Dict[str, Any]:
        return {"session": cfg.get("session") or "", "sessions": he_session.list_sessions()}


def _stack_eyes(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    def mark(img: np.ndarray, text: str) -> np.ndarray:
        out = img.copy()
        cv2.putText(out, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
        return out

    a, b = mark(left, "L"), mark(right, "R")
    if b.shape[1] != a.shape[1]:
        b = cv2.resize(b, (a.shape[1], int(b.shape[0] * a.shape[1] / b.shape[1])))
    return np.vstack([a, b])
