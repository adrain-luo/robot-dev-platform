"""相机与点云接口：取相机、预览图、双目匹配 / RGB-D 深度点云、彩色内参文件上传删除。"""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np

from robot_platform.calib.handeye import color_intrinsics as he_color_intr, results as he_results
from robot_platform.calib.handeye.config import effective_camera, load_config, save_config
from robot_platform.calib.handeye.detection import detect_corners, parse_board_size
from robot_platform.core import instance as instances
from robot_platform.perception import rgbd
from robot_platform.perception.camera import open_camera
from robot_platform.perception.stereo_match import colorize_disp, StereoMatcher, write_ply


def _optical_from_down_deg(T: np.ndarray) -> float:
    """光轴（相机 +Z）和躯干竖直向下之间的夹角。调试口的 camera_to_vertical_deg 是同一个角。"""
    z = np.asarray(T, dtype=float)[:3, 2]
    z = z / np.linalg.norm(z)
    return float(np.degrees(np.arccos(np.clip(float(np.dot(z, [0.0, 0.0, -1.0])), -1.0, 1.0))))


def _stack_eyes(left: np.ndarray, right: np.ndarray, labels=("L", "R")) -> np.ndarray:
    def mark(img: np.ndarray, text: str) -> np.ndarray:
        out = img.copy()
        cv2.putText(out, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
        return out

    a, b = mark(left, labels[0]), mark(right, labels[1])
    if b.shape[1] != a.shape[1]:
        b = cv2.resize(b, (a.shape[1], int(b.shape[0] * a.shape[1] / b.shape[1])))
    return np.vstack([a, b])


class CameraCloudMixin:
    def _is_mock(self) -> bool:
        backend = getattr(getattr(self, "app", None), "backend", None)
        return getattr(backend, "name", "") == "mock"

    def _camera(self, cfg: Dict[str, Any]):
        """按实例配置与机型相机声明打开当前相机（规则见 calib/handeye/config.py）；参数没变就复用。"""
        eff = effective_camera(cfg, mock=self._is_mock())
        key = (f"{eff['camera_source']}|{eff['camera_host']}|{eff['teleimager_path']}|"
               f"{json.dumps(eff['ros_topics'], sort_keys=True)}")
        with self._lock:
            if self._cam is None or key != self._cam_key:
                if self._cam is not None:
                    self._cam.close()
                self._cam = open_camera(eff)
                self._cam_key = key
            return self._cam

    def close(self) -> None:
        if self._cam is not None:
            self._cam.close()
            self._cam = None

    def preview_jpeg(self) -> Tuple[Optional[bytes], str]:
        cfg = load_config()
        cam = self._camera(cfg)
        frame = cam.grab(2.0)
        if frame is None:
            return None, cam.last_error or "没有画面"
        try:
            board = parse_board_size(str(cfg.get("board_size") or "11x8"))
        except ValueError as exc:
            return None, str(exc)
        _lf, left_vis = detect_corners(frame.color, board, draw=True)
        if frame.right is not None:
            _rf, right_vis = detect_corners(frame.right, board, draw=True)
            vis = _stack_eyes(left_vis, right_vis)
        else:
            depth_vis = rgbd.colorize_depth(frame.depth, float(frame.info.get("depth_scale_m", 0.001)))
            vis = _stack_eyes(left_vis, depth_vis, ("RGB", "DEPTH"))
        ok, buf = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ok:
            return None, "编码预览失败"
        return buf.tobytes(), ""

    def _match(self, body: Dict[str, Any]) -> Dict[str, Any]:
        import base64
        cfg = load_config()
        use_debug = str(body.get("source") or "") == "debug"
        cam = self._camera(cfg)
        frame = cam.grab(2.0)
        if frame is None:
            return {"ok": False, "error": cam.last_error or "没有画面"}
        note = ""
        hand_T = self._left_cam_to_torso(cfg)
        if frame.depth is not None:
            if use_debug:
                return {"ok": False, "error": "调试口匹配只适用于 G1D 的双目相机"}
            intr = body["color_intrinsics"] if "color_intrinsics" in body else cfg.get("cloud_color_intrinsics")
            return self._rgbd_cloud(frame, hand_T, str(intr or ""))
        pair = (frame.color, frame.right)
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
        cloud_dir = instances.path("clouds")
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

    def _rgbd_cloud(self, frame, T: Optional[np.ndarray], intr_name: str = "") -> Dict[str, Any]:
        """RGB-D：深度直接给点，不做立体匹配；点在彩色光学系，乘手眼外参到躯干。

        intr_name 非空时，彩色内参换成自标定文件（换算到当前分辨率）；只影响点在彩色图上的取色位置，
        点的三维位置仍由深度相机内参与深度到彩色外参决定。选哪份由相机卡片决定（实例配置
        cloud_color_intrinsics），这里只用不改。
        """
        import base64
        if T is None:
            return {"ok": False, "error": "没有彩色相机到躯干的外参。先调用已保存的手眼结果，点云才能画到模型上"}
        info = frame.info
        intr_note = ""
        intr_summary = None
        if intr_name:
            try:
                applied = he_color_intr.apply_to_info(info, intr_name)
            except (OSError, ValueError, KeyError) as exc:
                return {"ok": False, "error": str(exc)}
            info = applied["info"]
            intr_summary = applied["summary"]
            sm = intr_summary
            conv = ""
            if sm["converted_from"]:
                conv = f"由 {sm['converted_from'][0]}×{sm['converted_from'][1]} 换算到 {sm['size'][0]}×{sm['size'][1]}，"
            intr_note = (f"彩色内参用「{intr_name}」：{conv}fx {sm['fx']:.1f}（{sm['d_fx']:+.1f}）"
                         f" cx {sm['cx']:.1f}（{sm['d_cx']:+.1f}） cy {sm['cy']:.1f}（{sm['d_cy']:+.1f}），含畸变。")
        else:
            intr_note = "彩色内参用出厂 camera_info。"
        pts, cols = rgbd.depth_to_color_cloud(frame.depth, frame.color, info)
        n_valid = int(pts.shape[0])
        if n_valid < 50:
            return {"ok": False, "error": f"有效深度点只有 {n_valid}，看相机前方是否太近（<0.15 m）或被遮挡"}
        zz = pts[:, 2]
        cols = np.ascontiguousarray(cols[:, ::-1])  # BGR → RGB
        cloud_dir = instances.path("clouds")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        ply = cloud_dir / f"{stamp}_cam.ply"
        n_ply = write_ply(ply, pts, cols[:, ::-1])
        homo = np.concatenate([pts, np.ones((pts.shape[0], 1))], axis=1)
        pts_t = (T @ homo.T).T[:, :3]
        torso_ply = cloud_dir / f"{stamp}_torso.ply"
        write_ply(torso_ply, pts_t, cols[:, ::-1])
        vis = rgbd.colorize_depth(frame.depth, float(frame.info.get("depth_scale_m", 0.001)))
        ok, buf = cv2.imencode(".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ok:
            return {"ok": False, "error": "深度图编码失败"}
        ext = frame.info.get("depth_to_color") or {}
        note = f"RGB-D 深度点云：手眼外参光轴离竖直 {_optical_from_down_deg(T):.1f}°。" + intr_note
        if ext.get("assumed"):
            note += "没收到深度到彩色的外参，按同轴零平移处理，点云约有 15 mm 偏差。"
        return {
            "ok": True,
            "valid": n_valid,
            "saved": n_ply,
            "depth_m": {"median": float(np.median(zz)), "near": float(zz.min()), "far": float(np.percentile(zz, 95))},
            "ply_cam": str(ply),
            "ply_torso": str(torso_ply),
            "cloud": {"xyz": np.round(pts_t, 4).reshape(-1).tolist(), "rgb": cols.reshape(-1).astype(int).tolist()},
            "disparity_jpeg": base64.b64encode(buf.tobytes()).decode("ascii"),
            "color_intrinsics": intr_summary,
            "note": note,
        }

    def _color_intr_upload(self, body: Dict[str, Any]) -> Dict[str, Any]:
        name = str(body.get("name") or "").strip()
        try:
            res = he_color_intr.save(name, body.get("doc"), force=bool(body.get("force")))
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}
        if not res.get("ok"):
            return res
        return dict(self.status(), saved_name=res["name"], message=f"已保存内参文件「{res['name']}」")

    def _color_intr_delete(self, body: Dict[str, Any]) -> Dict[str, Any]:
        name = str(body.get("name") or "").strip()
        try:
            he_color_intr.delete(name)
        except (OSError, ValueError) as exc:
            return {"ok": False, "error": str(exc)}
        cfg = load_config()
        if cfg.get("cloud_color_intrinsics") == name:
            cfg["cloud_color_intrinsics"] = ""
            save_config(cfg)
        return dict(self.status(), message=f"已删除内参文件「{name}」")

    def _calib_yaml(self, cfg: Dict[str, Any]) -> Path:
        name = str(cfg.get("active_result") or "")
        if name:
            p = he_results.results_dir() / f"{name}.yaml"
            if p.is_file():
                return p
        intr = instances.active().from_stored(cfg.get("intrinsics"))
        if intr is not None and intr.is_file():
            return intr
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
