"""手眼采集与求解。走点发令调用 arm_control 的控制循环，关节从已有的 LowStateReader 读。"""

from __future__ import annotations

import html
import json
import re
import threading
import urllib.error
import urllib.request
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

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
_DEBUG_HOSTS = (
    "http://192.168.123.164:18081",
)

def _kmat(src: Dict[str, Any]) -> np.ndarray:
    return np.array(
        [[float(src["fx"]), 0.0, float(src["cx"])],
         [0.0, float(src["fy"]), float(src["cy"])],
         [0.0, 0.0, 1.0]],
        dtype=float,
    )


def _optical_from_down_deg(T: np.ndarray) -> float:
    """光轴（相机 +Z）和躯干竖直向下之间的夹角。调试口的 camera_to_vertical_deg 是同一个角。"""
    z = np.asarray(T, dtype=float)[:3, 2]
    z = z / np.linalg.norm(z)
    return float(np.degrees(np.arccos(np.clip(float(np.dot(z, [0.0, 0.0, -1.0])), -1.0, 1.0))))


def _T_from_debug_basis(basis: Dict[str, Any], translation: np.ndarray) -> np.ndarray:
    """调试口的三个轴写在左相机光学系里。躯干是 X 朝前、Y 朝左、Z 朝上。"""
    x_right = np.asarray(basis["x_right_unit_xyz"], dtype=float)
    forward = np.asarray(basis["ground_forward_unit_xyz"], dtype=float)
    down = np.asarray(basis["vertical_down_unit_xyz"], dtype=float)
    cam = np.column_stack([x_right, forward, down])
    torso = np.column_stack([
        np.array([0.0, -1.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 0.0, -1.0]),
    ])
    R = torso @ np.linalg.inv(cam)
    u, _, vt = np.linalg.svd(R)
    R = u @ vt
    if np.linalg.det(R) < 0:
        u = u.copy()
        u[:, -1] *= -1
        R = u @ vt
    out = np.eye(4)
    out[:3, :3] = R
    out[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return out


def _http_body(url: str, timeout: float) -> bytes:
    """调试口在烟盒没认准时用 HTTP 500 退回，正文里仍有标定。"""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            return resp.read()
    except urllib.error.HTTPError as exc:
        return exc.read()


def _http_text(url: str, timeout: float) -> str:
    return _http_body(url, timeout).decode("utf-8", "replace")


def _http_json(url: str, timeout: float) -> Optional[Dict[str, Any]]:
    try:
        data = json.loads(_http_text(url, timeout))
    except json.JSONDecodeError:
        return None
    return data if isinstance(data, dict) else None


def _fill_debug_calib(data: Dict[str, Any], page: str) -> Dict[str, Any]:
    """姿态接口失败时，从调试页正文补上内参、基线和安装角。"""
    out = dict(data)
    basis_m = re.search(
        r'"x_right_unit_xyz":\s*(\[[^\]]+\])\s*,\s*'
        r'"ground_forward_unit_xyz":\s*(\[[^\]]+\])\s*,\s*'
        r'"vertical_down_unit_xyz":\s*(\[[^\]]+\])',
        page,
    )
    cam = dict(((out.get("g1d_visualization") or {}).get("camera") or {}))
    basis = cam.get("basis") if isinstance(cam.get("basis"), dict) else {}
    if basis_m and any(k not in basis for k in ("x_right_unit_xyz", "ground_forward_unit_xyz", "vertical_down_unit_xyz")):
        basis = {
            "x_right_unit_xyz": json.loads(basis_m.group(1)),
            "ground_forward_unit_xyz": json.loads(basis_m.group(2)),
            "vertical_down_unit_xyz": json.loads(basis_m.group(3)),
        }
        cam["basis"] = basis
        vis = dict(out.get("g1d_visualization") or {})
        vis["camera"] = cam
        out["g1d_visualization"] = vis
    stereo = dict(out.get("stereo") or {})
    if not stereo.get("baseline_mm"):
        base_m = re.search(r'"baseline_mm":\s*([0-9.]+)\s*,\s*"stereo_reprojection_error_px"', page)
        if base_m:
            stereo["baseline_mm"] = float(base_m.group(1))
            out["stereo"] = stereo
    return out


def _debug_calib_ready(data: Dict[str, Any]) -> bool:
    left = data.get("intrinsics_assumption") or {}
    right = data.get("intrinsics_assumption_right") or {}
    if any(left.get(k) is None for k in ("fx", "fy", "cx", "cy")) or not left.get("dist_coeffs"):
        return False
    if any(right.get(k) is None for k in ("fx", "fy", "cx", "cy")) or not right.get("dist_coeffs"):
        return False
    try:
        if float((data.get("stereo") or {}).get("baseline_mm") or 0.0) < 10.0:
            return False
    except (TypeError, ValueError):
        return False
    basis = (((data.get("g1d_visualization") or {}).get("camera") or {}).get("basis") or {})
    return all(k in basis for k in ("x_right_unit_xyz", "ground_forward_unit_xyz", "vertical_down_unit_xyz"))


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
        if path == "/api/detect/grab":
            return self._grab_detect(body)
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

    def _debug_base(self, preferred: str = "") -> str:
        """只解析主机，不请求 /debug（空参 /debug 会跑一次自动类别，冲掉 label）。"""
        hosts: List[str] = []
        raw = (preferred or "").strip().rstrip("/")
        if raw:
            hosts.append(raw.split("/debug")[0].split("/pose")[0])
        for host in _DEBUG_HOSTS:
            if host not in hosts:
                hosts.append(host)
        if not hosts:
            raise RuntimeError("没有配置调试口地址")
        return hosts[0]

    def _parse_pose_extra(self, body: Dict[str, Any]) -> Dict[str, str]:
        """从前端 body 解析要传给 /pose 的 query。"""
        from urllib.parse import parse_qsl, urlsplit

        extra: Dict[str, str] = {}
        raw_params = body.get("params")
        if isinstance(raw_params, dict):
            for k, v in raw_params.items():
                if v is None:
                    continue
                extra[str(k)] = str(v)
        qtext = str(body.get("query") or body.get("params_text") or "").strip()
        if qtext.startswith("?"):
            qtext = qtext[1:]
        if qtext.startswith("/pose?"):
            qtext = qtext[len("/pose?") :]
        if qtext.startswith("http://") or qtext.startswith("https://"):
            qtext = urlsplit(qtext).query
        if not qtext:
            return extra
        if qtext.startswith("{"):
            try:
                obj = json.loads(qtext)
            except json.JSONDecodeError as exc:
                raise ValueError(f"参数 JSON 无效：{exc}") from exc
            if not isinstance(obj, dict):
                raise ValueError("params JSON 必须是对象")
            for k, v in obj.items():
                if v is None:
                    continue
                extra[str(k)] = str(v)
            return extra
        for k, v in parse_qsl(qtext, keep_blank_values=True):
            extra[str(k)] = str(v)
        return extra

    def _pose_url_with_params(self, base: str, body: Dict[str, Any]) -> Tuple[str, str]:
        """只拼用户参数 + 新 t，不合并调试页旧 query（避免空参 /debug 的自动结果）。"""
        from urllib.parse import urlencode, urlsplit, urlunsplit

        extra = self._parse_pose_extra(body)
        query = dict(extra)
        query["t"] = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        # 用户若写了 t 则保留用户的
        if "t" in extra:
            query["t"] = extra["t"]
        new_query = urlencode(list(query.items()))
        base_parts = urlsplit(base)
        pose_url = urlunsplit((base_parts.scheme, base_parts.netloc, "/pose", new_query, ""))
        return pose_url, new_query

    def _grab_detect(self, body: Dict[str, Any]) -> Dict[str, Any]:
        """从调试口取左右目原图 + YOLO 掩膜标注图，供前端半透明叠显。"""
        import base64
        import time as _time

        try:
            base = self._debug_base(str(body.get("url") or ""))
        except RuntimeError as exc:
            return {"ok": False, "error": str(exc)}
        overlay = str(body.get("overlay") or "points").strip().lower()
        if overlay not in ("points", "candidates"):
            overlay = "points"
        ov_suffix = "points.jpg" if overlay == "points" else "candidates.jpg"
        stamp = _time.time_ns()
        pose: Dict[str, Any] = {}
        pose_query = ""
        try:
            pose_url, pose_query = self._pose_url_with_params(base, body)
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}
        try:
            # 只打 /pose?label=...，不要先打空参 /debug
            got = _http_json(pose_url, 25)
            if isinstance(got, dict):
                pose = got
        except Exception as exc:
            return {
                "ok": False,
                "error": f"调试口 /pose 失败：{exc.__class__.__name__}: {exc}",
                "pose_url": pose_url,
                "pose_query": pose_query,
            }

        def _explain_non_jpeg(data: bytes, tried: str) -> str:
            text = (data or b"")[:400].decode("utf-8", errors="replace").strip()
            if text.startswith("{"):
                try:
                    obj = json.loads(text)
                    err = obj.get("error") or obj.get("error_code") or text
                    return f"{tried} → {err}"
                except Exception:
                    pass
            if not data:
                return f"{tried} 空响应"
            return f"{tried} 不是 JPEG（{text[:120] or data[:12]!r}）"

        def _jpeg(name: str) -> Tuple[Optional[bytes], str]:
            """叠图优先 /latest；没有则走 /debug/<name>?参数（会按参数重跑一帧）。

            注意：没有 /debug/latest/... 这种路径。
            """
            q = pose_query or f"t={stamp}"
            if "t=" not in q:
                q = f"{q}&t={stamp}"
            urls = [
                f"{base}/latest/{name}?t={stamp}",
                f"{base}/debug/{name}?{q}",
            ]
            errors: List[str] = []
            for url in urls:
                try:
                    data = _http_body(url, 20)
                except Exception as exc:
                    errors.append(f"{url}: {exc.__class__.__name__}: {exc}")
                    continue
                if data and data[:2] == b"\xff\xd8":
                    return data, ""
                errors.append(_explain_non_jpeg(data or b"", url.replace(base, "")))
            return None, "；".join(errors)

        left_raw, err = _jpeg("left_input.jpg")
        if left_raw is None:
            return {"ok": False, "error": f"左目原图失败：{err}", "pose_query": pose_query}
        left_mask, err = _jpeg(f"left_{ov_suffix}")
        if left_mask is None:
            return {"ok": False, "error": f"左目掩膜图失败：{err}", "pose_query": pose_query}
        right_raw, right_raw_err = _jpeg("right_input.jpg")
        right_mask, right_mask_err = _jpeg(f"right_{ov_suffix}")
        if right_raw is None or right_mask is None:
            return {
                "ok": False,
                "error": "右目图失败：" + "；".join(x for x in (right_raw_err, right_mask_err) if x),
                "pose_query": pose_query,
            }

        def _b64(data: bytes) -> str:
            return base64.b64encode(data).decode("ascii")

        yolo = ((pose.get("g1d_visualization") or {}).get("yolo") or {}) if pose else {}
        requested = pose.get("requested_yolo_label")
        label = pose.get("selected_yolo_label") or yolo.get("label")
        name = yolo.get("display_name") or label
        conf = pose.get("selected_yolo_confidence")
        if conf is None:
            conf = yolo.get("confidence")
        note = "左右目各：原图 + YOLO 掩膜，前端 50% 叠显。"
        if requested not in (None, "") and label not in (None, "") and str(requested) != str(label):
            note += f" 请求 label={requested}，实际选中 {label}。"
        if pose.get("ok") is False and pose.get("error"):
            note += f" 姿态接口：{pose.get('error')}。"
        left = {"image_jpeg": _b64(left_raw), "mask_jpeg": _b64(left_mask)}
        right = {"image_jpeg": _b64(right_raw), "mask_jpeg": _b64(right_mask)}
        return {
            "ok": True,
            "host": base,
            "pose_url": pose_url,
            "pose_query": pose_query,
            "overlay": overlay,
            "requested_label": requested,
            "label": label,
            "display_name": name,
            "confidence": None if conf is None else float(conf),
            "class_id": pose.get("selected_yolo_class_id") if pose else yolo.get("class_id"),
            "left": left,
            "right": right,
            # 兼容旧前端字段（等同左目）
            "image_jpeg": left["image_jpeg"],
            "mask_jpeg": left["mask_jpeg"],
            "pose_ok": bool(pose.get("ok")) if pose else None,
            "note": note,
        }

    def _match(self, body: Dict[str, Any]) -> Dict[str, Any]:
        import base64
        cfg = load_config()
        use_debug = str(body.get("source") or "") == "debug"
        cam = self._camera(cfg)
        pair = cam.grab_pair(2.0)
        if pair is None:
            return {"ok": False, "error": cam.last_error or "没有画面"}
        note = ""
        hand_T = self._left_cam_to_torso(cfg)
        if use_debug:
            try:
                debug, host = self._fetch_debug_pose(str(body.get("url") or ""))
                calib = self._debug_calib(debug, hand_T, (pair[0].shape[1], pair[0].shape[0]))
            except (OSError, ValueError, RuntimeError, KeyError) as exc:
                return {"ok": False, "error": str(exc)}
            matcher = calib["matcher"]
            T = calib["T"]
            note = calib["note"] + f" 数据来自 {host}。"
        else:
            matcher = self._matcher_for(self._calib_yaml(cfg))
            T = hand_T
            if T is None:
                return {"ok": False, "error": "没有左目到躯干的外参。先调用已保存的手眼结果，点云才能画到模型上"}
            note = f"手眼外参：光轴离竖直 {_optical_from_down_deg(T):.1f}°。"
        rect_l, disp, points_m = matcher.match(pair[0], pair[1])
        valid = np.isfinite(points_m).all(axis=2) & (disp > 0.5)
        z = points_m[:, :, 2]
        # 只去掉相机背后/贴面噪声与离谱坏点；显示最远距由前端渲染滑条裁切
        valid &= (z > 0.15) & (z < 12.0)
        n_valid = int(valid.sum())
        if n_valid < 50:
            return {"ok": False, "error": f"有效点只有 {n_valid}，匹配没有对上"}
        zz = z[valid]
        pts = points_m[valid]
        cols = np.ascontiguousarray(rect_l[valid][:, ::-1])
        if use_debug:
            tint = np.array([255, 120, 20], dtype=np.float32)
            cols = np.clip(0.35 * cols.astype(np.float32) + 0.65 * tint, 0, 255).astype(np.uint8)
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
            "note": note,
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

    def _fetch_debug_pose(self, preferred: str) -> Tuple[Dict[str, Any], str]:
        hosts: List[str] = []
        raw = preferred.strip().rstrip("/")
        if raw:
            hosts.append(raw.split("/debug")[0].split("/pose")[0])
        for host in _DEBUG_HOSTS:
            if host not in hosts:
                hosts.append(host)
        errors: List[str] = []
        for base in hosts:
            try:
                page = _http_text(base + "/debug", 4)
                found = re.search(r'href="(/pose\?[^"]+)"', page)
                data: Dict[str, Any] = {}
                pose_error = ""
                if found:
                    url = base + html.unescape(found.group(1))
                    pose = _http_json(url, 8)
                    if isinstance(pose, dict):
                        data = pose
                        pose_error = str(pose.get("error") or "")
                data = _fill_debug_calib(data, page)
                if _debug_calib_ready(data):
                    if pose_error:
                        data["_pose_error"] = pose_error
                    return data, base
                errors.append(f"{base} {pose_error or '调试页里没有内参、基线或安装角'}")
            except Exception as exc:
                detail = getattr(exc, "reason", None) or exc
                errors.append(f"{base} {exc.__class__.__name__}: {detail}")
        raise RuntimeError("调试口连不上：" + "；".join(errors))

    def _debug_calib(self, data: Dict[str, Any], hand_T: Optional[np.ndarray], size: Tuple[int, int]) -> Dict[str, Any]:
        if hand_T is None:
            raise ValueError("调试口没有相机在躯干里的位置，仍要先调用已保存的手眼结果，用它的平移")
        left = data.get("intrinsics_assumption") or {}
        right = data.get("intrinsics_assumption_right") or {}
        stereo = data.get("stereo") or {}
        cam = ((data.get("g1d_visualization") or {}).get("camera") or {})
        basis = cam.get("basis") or {}
        for eye, src in (("左", left), ("右", right)):
            if any(src.get(k) is None for k in ("fx", "fy", "cx", "cy")) or not src.get("dist_coeffs"):
                raise ValueError(f"调试口没有{eye}目内参")
        baseline = float(stereo.get("baseline_mm") or 0.0)
        if baseline < 10.0:
            raise ValueError("调试口没有双目基线")
        needed = ("x_right_unit_xyz", "ground_forward_unit_xyz", "vertical_down_unit_xyz")
        if any(k not in basis for k in needed):
            raise ValueError("调试口没有相机相对躯干的朝向")
        K1 = _kmat(left)
        K2 = _kmat(right)
        # 已保存的双目标定里，右相机在左相机的负 X 方向。调试口只给基线长度。
        stereo_T = np.array([-baseline, 0.0, 0.0], dtype=float)
        key = (
            round(float(left["fx"]), 3), round(float(right["fx"]), 3),
            round(baseline, 2), int(size[0]), int(size[1]),
        )
        if self._debug_matcher is None or self._debug_key != key:
            self._debug_matcher = StereoMatcher.from_matrices(
                K1, left["dist_coeffs"], K2, right["dist_coeffs"],
                np.eye(3), stereo_T, size,
            )
            self._debug_key = key
        T = _T_from_debug_basis(basis, np.asarray(hand_T, float)[:3, 3])
        ours = _optical_from_down_deg(hand_T)
        theirs = _optical_from_down_deg(T)
        stated = cam.get("camera_to_vertical_deg")
        stated_txt = "" if stated is None else f"，页面写的是 {float(stated):.1f}°"
        note = (
            f"调试口内参 fx {float(left['fx']):.1f}/{float(right['fx']):.1f}，基线 {baseline:.2f} mm。"
            f"光轴离竖直 {theirs:.1f}°{stated_txt}，手眼是 {ours:.1f}°。"
            "橙色是这套，原色是手眼。平移仍用手眼的，调试口没给相机位置。"
            "它也没给左右相机之间的旋转，右相机按平行、基线沿负 X。"
        )
        pose_error = str(data.get("_pose_error") or data.get("error") or "")
        if pose_error:
            note += f" 烟盒姿态接口返回：{pose_error}。标定仍从调试页读出。"
        return {"matcher": self._debug_matcher, "T": T, "note": note}

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
        self._bind_handeye_doc(cfg, doc)
        save_config(cfg)
        self._matcher = None
        self._matcher_key = ""
        stt = self.status()
        stt["intrinsics_fit"] = doc.get("intrinsics_fit")
        stt["result"] = doc.get("result")
        stt["active_result"] = doc["name"]
        return stt

    @staticmethod
    def _bind_handeye_doc(cfg: Dict[str, Any], doc: Dict[str, Any]) -> None:
        """手眼结果与其自带双目 yaml 成套写入配置，不允许拆开。"""
        cfg["active_result"] = doc["name"]
        if doc.get("session"):
            cfg["session"] = doc["session"]
        yaml_path = str(doc.get("intrinsics_yaml") or "")
        if not yaml_path:
            cand = he_results.RESULTS_DIR / f"{doc['name']}.yaml"
            if cand.is_file():
                yaml_path = str(cand)
        if yaml_path and Path(yaml_path).is_file():
            cfg["intrinsics"] = yaml_path

    def _load_named(self, body: Dict[str, Any]) -> Dict[str, Any]:
        doc = he_results.load_saved(str(body.get("name") or ""))
        cfg = load_config()
        self._bind_handeye_doc(cfg, doc)
        save_config(cfg)
        self._matcher = None
        self._matcher_key = ""
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
