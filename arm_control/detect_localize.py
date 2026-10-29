"""识别定位：YOLO 四边形 → 网格点位 → 平面校准 → 左右合成中心。

深度取法对齐棋盘「看一眼」：对调试口最新左右图做 SGBM，在校正左目上采样。
"""

from __future__ import annotations

import threading
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from board_touch import PLANE_MAX_MM, _fit_plane, _sample_point

GRID_N = 10  # 10 行 10 列划分 → (N+1)×(N+1) 交叉点


def _quad_grid_px(quad: Sequence[Sequence[float]], n: int = GRID_N) -> np.ndarray:
    """四边形上均匀行列划分，取交叉点。n 格 → (n+1)×(n+1) 点。顺序行优先。"""
    q = np.asarray(quad, dtype=float).reshape(4, 2)
    # 约定 YOLO 四点：0-1 顶边，2-3 底边（与调试图标注一致）
    top_l, top_r, bot_r, bot_l = q[0], q[1], q[2], q[3]
    pts = []
    for i in range(n + 1):
        v = i / float(n)
        left = top_l * (1 - v) + bot_l * v
        right = top_r * (1 - v) + bot_r * v
        for j in range(n + 1):
            u = j / float(n)
            pts.append(left * (1 - u) + right * u)
    return np.asarray(pts, dtype=float)


def _raw_to_rect_px(
    xy: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    R: np.ndarray,
    P: np.ndarray,
) -> np.ndarray:
    """原图像素 → 校正图像素。"""
    src = np.asarray(xy, dtype=np.float32).reshape(-1, 1, 2)
    dst = cv2.undistortPoints(src, K, dist, R=R, P=P)
    return dst.reshape(-1, 2)


def _decode_jpeg(data: bytes) -> np.ndarray:
    arr = np.frombuffer(data, dtype=np.uint8)
    im = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if im is None:
        raise RuntimeError("JPEG 解码失败")
    return im


def _homography_right_to_left(left_quad: np.ndarray, right_quad: np.ndarray) -> np.ndarray:
    H, mask = cv2.findHomography(
        np.asarray(right_quad, dtype=np.float32),
        np.asarray(left_quad, dtype=np.float32),
        method=0,
    )
    if H is None:
        raise RuntimeError("左右四边形算不出单应，无法把右目网格映到左目")
    return H


def _apply_H(H: np.ndarray, xy: np.ndarray) -> np.ndarray:
    src = np.asarray(xy, dtype=np.float32).reshape(-1, 1, 2)
    dst = cv2.perspectiveTransform(src, H)
    return dst.reshape(-1, 2)


def _grid_ij(indices: np.ndarray, side: int) -> Tuple[np.ndarray, np.ndarray]:
    """原网格线性下标 → (row, col)，行优先。"""
    idx = np.asarray(indices, dtype=int).reshape(-1)
    return idx // side, idx % side


def _center_from_grid_inliers(
    points: np.ndarray,
    indices: np.ndarray,
    grid_n: int,
) -> Tuple[np.ndarray, Dict[str, Any]]:
    """用残缺网格内点恢复矩形面心。

    每个内点带已知格子坐标 (row, col)∈[0,n]²，拟合
        p ≈ origin + col·u + row·v
    再在参数中心 (n/2, n/2) 求值（10×10 格 → (5,5)）。
    ``points`` / ``indices`` 可左右目拼在一起（同一 (row,col) 可有两条观测）。
    """
    side = grid_n + 1
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    idx = np.asarray(indices, dtype=int).reshape(-1)
    if pts.shape[0] != idx.shape[0]:
        raise RuntimeError("内点与下标数量不一致")
    ok = np.isfinite(pts).all(axis=1) & (idx >= 0) & (idx < side * side)
    pts, idx = pts[ok], idx[ok]
    if pts.shape[0] < 3:
        raise RuntimeError("内点少于 3，无法拟合网格面心")
    rows, cols = _grid_ij(idx, side)
    # 行列都要有跨度，否则外推 (n/2,n/2) 不稳定
    if int(np.unique(rows).size) < 2 or int(np.unique(cols).size) < 2:
        raise RuntimeError("内点行列跨度不足，无法外推面心")

    n_obs = pts.shape[0]
    A = np.zeros((3 * n_obs, 9), dtype=float)
    b = np.zeros(3 * n_obs, dtype=float)
    for k in range(n_obs):
        A[3 * k:3 * k + 3, 0:3] = np.eye(3)
        A[3 * k:3 * k + 3, 3:6] = float(cols[k]) * np.eye(3)
        A[3 * k:3 * k + 3, 6:9] = float(rows[k]) * np.eye(3)
        b[3 * k:3 * k + 3] = pts[k]
    sol, residuals, rank, _ = np.linalg.lstsq(A, b, rcond=None)
    if rank < 9:
        raise RuntimeError(f"网格拟合秩不足（rank={rank}）")
    origin, u, v = sol[0:3], sol[3:6], sol[6:9]
    half = 0.5 * float(grid_n)
    center = origin + half * u + half * v
    # 残差（观测点）
    pred = origin + cols[:, None] * u + rows[:, None] * v
    err_mm = np.linalg.norm(pts - pred, axis=1) * 1000.0
    meta = {
        "n_obs": int(n_obs),
        "grid_center_ij": [half, half],
        "fit_rms_mm": float(np.sqrt(np.mean(err_mm ** 2))),
        "fit_max_mm": float(np.max(err_mm)),
        "span_rows": int(np.unique(rows).size),
        "span_cols": int(np.unique(cols).size),
    }
    return center, meta


class DetectLocalize:
    def __init__(self, app) -> None:
        self.app = app
        self._lock = threading.Lock()
        self.phase = ""
        self.error: Optional[str] = None
        self.grid_n = GRID_N
        self.left_quad: Optional[np.ndarray] = None
        self.right_quad: Optional[np.ndarray] = None
        self.left_cam: Optional[np.ndarray] = None   # Nx3 左相机系
        self.right_cam: Optional[np.ndarray] = None
        self.left_torso: Optional[np.ndarray] = None
        self.right_torso: Optional[np.ndarray] = None
        self.left_cal: Optional[np.ndarray] = None
        self.right_cal: Optional[np.ndarray] = None
        self.cal_indices: Optional[np.ndarray] = None  # 校准保留点在原网格中的下标
        self.face_center_torso: Optional[np.ndarray] = None  # 原网格四角贴平面后的面心
        self.center_torso: Optional[np.ndarray] = None
        self.meta: Dict[str, Any] = {}

    def status(self) -> Dict[str, Any]:
        with self._lock:
            def pack(pts: Optional[np.ndarray]) -> Optional[list]:
                if pts is None:
                    return None
                return np.round(pts, 4).reshape(-1).tolist()

            # 校准后优先显示贴平面内点；中心提取后仍保留红蓝
            left_vis = self.left_cal if self.left_cal is not None else self.left_torso
            right_vis = self.right_cal if self.right_cal is not None else self.right_torso
            return {
                "ok": self.error is None,
                "error": self.error,
                "phase": self.phase,
                "grid_n": self.grid_n,
                "meta": dict(self.meta),
                "left_xyz": pack(left_vis),
                "right_xyz": pack(right_vis),
                "center_xyz": pack(
                    None if self.center_torso is None else self.center_torso.reshape(1, 3)
                ),
                "n_left": 0 if left_vis is None else int(left_vis.shape[0]),
                "n_right": 0 if right_vis is None else int(right_vis.shape[0]),
            }

    def clear(self) -> Dict[str, Any]:
        with self._lock:
            self.phase = ""
            self.error = None
            self.left_quad = self.right_quad = None
            self.left_cam = self.right_cam = None
            self.left_torso = self.right_torso = None
            self.left_cal = self.right_cal = None
            self.cal_indices = None
            self.face_center_torso = None
            self.center_torso = None
            self.meta = {}
        return self.status()

    def acquire_points(self, body: Dict[str, Any]) -> Dict[str, Any]:
        """点位获取：YOLO 四边形 → 网格 → SGBM 深度 → 躯干坐标。"""
        from service import _http_body, _http_json, load_config

        self.error = None
        self.center_torso = None
        self.face_center_torso = None
        self.left_cal = self.right_cal = None
        self.cal_indices = None
        try:
            he = self.app.handeye
            cfg = load_config()
            T = he._left_cam_to_torso(cfg)
            if T is None:
                raise ValueError("先调用一组手眼结果，点才能变到躯干上")
            base = he._debug_base(str(body.get("url") or ""))
            pose_url, pose_query = he._pose_url_with_params(base, body)
            pose = _http_json(pose_url, 25) or {}
            if not isinstance(pose, dict):
                raise RuntimeError("调试口 /pose 无有效 JSON")
            pa = pose.get("point_adjustments") or {}
            left_yolo = pa.get("left_yolo") or {}
            right_yolo = pa.get("right_yolo") or {}
            left_quad = left_yolo.get("points_px") or (pose.get("g1d_visualization") or {}).get("yolo", {}).get("points_px")
            right_quad = right_yolo.get("points_px") or (pose.get("right") or {}).get("points_px")
            if left_quad is None or len(left_quad) != 4:
                raise RuntimeError("YOLO 没有左目四点掩膜")
            if right_quad is None or len(right_quad) != 4:
                raise RuntimeError("YOLO 没有右目四点掩膜")
            grid_n = int(body.get("grid_n") or GRID_N)
            grid_n = max(2, min(32, grid_n))
            stamp = pose_query  # cache bust via unique query already used
            import time as _time
            t = _time.time_ns()
            left_jpg = _http_body(f"{base}/latest/left_input.jpg?t={t}", 12)
            right_jpg = _http_body(f"{base}/latest/right_input.jpg?t={t}", 12)
            left_bgr = _decode_jpeg(left_jpg)
            right_bgr = _decode_jpeg(right_jpg)
            matcher = he._matcher_for(he._calib_yaml(cfg))
            rect_l, disp, points_m = matcher.match(left_bgr, right_bgr)

            left_q = np.asarray(left_quad, dtype=float).reshape(4, 2)
            right_q = np.asarray(right_quad, dtype=float).reshape(4, 2)
            left_grid_raw = _quad_grid_px(left_q, grid_n)
            right_grid_raw = _quad_grid_px(right_q, grid_n)
            # 右目网格经四点单应映到左目，再在左目深度图采样（与看一眼同一套 points_m）
            H = _homography_right_to_left(left_q, right_q)
            right_on_left_raw = _apply_H(H, right_grid_raw)

            left_rect = _raw_to_rect_px(left_grid_raw, matcher.K1, matcher.d1, matcher.R1, matcher.P1)
            right_rect = _raw_to_rect_px(right_on_left_raw, matcher.K1, matcher.d1, matcher.R1, matcher.P1)

            left_cam, left_miss = self._sample_grid(points_m, left_rect)
            right_cam, right_miss = self._sample_grid(points_m, right_rect)
            if left_miss:
                raise RuntimeError(f"左目网格有 {left_miss} 个点没有深度")
            if right_miss:
                raise RuntimeError(f"右目网格有 {right_miss} 个点没有深度")

            homo_l = np.concatenate([left_cam, np.ones((left_cam.shape[0], 1))], axis=1)
            homo_r = np.concatenate([right_cam, np.ones((right_cam.shape[0], 1))], axis=1)
            TT = np.asarray(T, dtype=float)
            left_t = (TT @ homo_l.T).T[:, :3]
            right_t = (TT @ homo_r.T).T[:, :3]

            yolo = (pose.get("g1d_visualization") or {}).get("yolo") or {}
            with self._lock:
                self.grid_n = grid_n
                self.phase = "points"
                self.left_quad, self.right_quad = left_q, right_q
                self.left_cam, self.right_cam = left_cam, right_cam
                self.left_torso, self.right_torso = left_t, right_t
                self.meta = {
                    "pose_query": pose_query,
                    "requested_label": pose.get("requested_yolo_label"),
                    "label": pose.get("selected_yolo_label") or yolo.get("label"),
                    "display_name": yolo.get("display_name"),
                    "confidence": pose.get("selected_yolo_confidence") or yolo.get("confidence"),
                    "n_cells": grid_n,
                    "n_points": (grid_n + 1) ** 2,
                }
            out = self.status()
            out["message"] = (
                f"点位获取：{grid_n}×{grid_n} 格 → {(grid_n + 1) ** 2} 交叉点；"
                f"左红 {left_t.shape[0]} · 右蓝 {right_t.shape[0]}"
            )
            return out
        except Exception as exc:
            self.error = str(exc)
            out = self.status()
            out["ok"] = False
            out["error"] = str(exc)
            return out

    def _sample_grid(self, points_m: np.ndarray, rect_xy: np.ndarray) -> Tuple[np.ndarray, int]:
        raw = []
        missed = 0
        for xy in rect_xy:
            p = _sample_point(points_m, float(xy[0]), float(xy[1]))
            if p is None:
                missed += 1
                raw.append(np.full(3, np.nan))
            else:
                raw.append(p)
        return np.asarray(raw, dtype=float), missed

    def calibrate(self) -> Dict[str, Any]:
        """校准：只贴平面；偏离过大的点丢掉，用内点重拟合后再投影。"""
        self.error = None
        self.center_torso = None
        try:
            with self._lock:
                left = None if self.left_cam is None else self.left_cam.copy()
                right = None if self.right_cam is None else self.right_cam.copy()
            if left is None or right is None:
                raise RuntimeError("先做点位获取")
            if left.shape != right.shape:
                raise RuntimeError("左右网格点数不一致")
            finite = np.isfinite(left).all(axis=1) & np.isfinite(right).all(axis=1)
            if int(finite.sum()) < 4:
                raise RuntimeError("有效点太少，拟合不了平面")

            # 左右目一起估平面（同为左相机系），再按残差剔点、重拟合
            seed = np.vstack([left[finite], right[finite]])
            plane0 = _fit_plane(seed)

            def _abs_dist_mm(pts: np.ndarray, plane: Dict[str, Any]) -> np.ndarray:
                return np.abs((pts - plane["center"]) @ plane["normal"]) * 1000.0

            d0 = np.maximum(_abs_dist_mm(left, plane0), _abs_dist_mm(right, plane0))
            keep = finite & (d0 <= PLANE_MAX_MM)
            dropped = int(finite.sum() - keep.sum())
            if int(keep.sum()) < 4:
                raise RuntimeError(
                    f"贴平面后内点不足（丢掉 {dropped} 个，阈值 |d|≤{PLANE_MAX_MM:.0f} mm）"
                )
            plane = _fit_plane(np.vstack([left[keep], right[keep]]))
            d1 = np.maximum(_abs_dist_mm(left, plane), _abs_dist_mm(right, plane))
            keep2 = finite & (d1 <= PLANE_MAX_MM)
            dropped = int(finite.sum() - keep2.sum())
            if int(keep2.sum()) < 4:
                raise RuntimeError(
                    f"二次筛选后内点不足（丢掉 {dropped} 个，阈值 |d|≤{PLANE_MAX_MM:.0f} mm）"
                )
            if int(keep2.sum()) < int(keep.sum()):
                plane = _fit_plane(np.vstack([left[keep2], right[keep2]]))
                keep = keep2
            else:
                keep = keep2

            def project(pts: np.ndarray) -> np.ndarray:
                d = (pts - plane["center"]) @ plane["normal"]
                return pts - d[:, None] * plane["normal"]

            left_flat = project(left[keep])
            right_flat = project(right[keep])
            keep_idx = np.flatnonzero(keep).astype(int)

            from service import load_config
            he = self.app.handeye
            cfg = load_config()
            T = he._left_cam_to_torso(cfg)
            if T is None:
                raise ValueError("没有手眼外参")
            TT = np.asarray(T, dtype=float)
            homo_l = np.concatenate([left_flat, np.ones((left_flat.shape[0], 1))], axis=1)
            homo_r = np.concatenate([right_flat, np.ones((right_flat.shape[0], 1))], axis=1)
            left_t = (TT @ homo_l.T).T[:, :3]
            right_t = (TT @ homo_r.T).T[:, :3]

            # 面心：左右内点都带原网格 (row,col)，拟合格子再求 (n/2,n/2)
            both_pts = np.vstack([left_t, right_t])
            both_idx = np.concatenate([keep_idx, keep_idx])
            face_t, fit_meta = _center_from_grid_inliers(both_pts, both_idx, int(self.grid_n))

            with self._lock:
                self.phase = "calibrate"
                self.left_cal, self.right_cal = left_t, right_t
                self.cal_indices = keep_idx
                self.face_center_torso = face_t
                self.meta = {
                    **self.meta,
                    "plane_rms_mm": plane["rms_mm"],
                    "plane_max_mm": plane["max_mm"],
                    "plane_kept": int(keep.sum()),
                    "plane_dropped": dropped,
                    "plane_reject_mm": PLANE_MAX_MM,
                    "center_fit": fit_meta,
                }
            out = self.status()
            drop_txt = f"，丢掉 {dropped} 个离群点" if dropped else ""
            out["message"] = (
                f"校准完成：贴平面 rms {plane['rms_mm']:.1f} mm / max {plane['max_mm']:.1f} mm"
                f"（保留 {int(keep.sum())} 点{drop_txt}）"
            )
            return out
        except Exception as exc:
            self.error = str(exc)
            out = self.status()
            out["ok"] = False
            out["error"] = str(exc)
            return out

    def extract_center(self) -> Dict[str, Any]:
        """中心点：用带网格坐标的左右内点拟合格子，求 (n/2,n/2)；不清红蓝。"""
        self.error = None
        try:
            with self._lock:
                left = self.left_cal if self.left_cal is not None else self.left_torso
                right = self.right_cal if self.right_cal is not None else self.right_torso
                left = None if left is None else left.copy()
                right = None if right is None else right.copy()
                cal_idx = None if self.cal_indices is None else self.cal_indices.copy()
                grid_n = int(self.grid_n)
                face = None if self.face_center_torso is None else self.face_center_torso.copy()

            if left is None or right is None:
                raise RuntimeError("先做点位获取（建议再校准）")
            if left.shape[0] != right.shape[0]:
                raise RuntimeError("左右点数不一致")

            if cal_idx is not None and cal_idx.size == left.shape[0]:
                both_pts = np.vstack([left, right])
                both_idx = np.concatenate([cal_idx, cal_idx])
                center, fit_meta = _center_from_grid_inliers(both_pts, both_idx, grid_n)
                how = "网格内点外推"
            elif face is not None:
                center, fit_meta = face, {}
                how = "校准缓存面心"
            else:
                # 未剔点：全网格下标 0..side²-1
                side = grid_n + 1
                if side * side != left.shape[0]:
                    raise RuntimeError("无校准内点下标，且网格不是完整方阵")
                full_idx = np.arange(side * side, dtype=int)
                both_pts = np.vstack([left, right])
                both_idx = np.concatenate([full_idx, full_idx])
                center, fit_meta = _center_from_grid_inliers(both_pts, both_idx, grid_n)
                how = "全网格拟合"

            with self._lock:
                self.phase = "center"
                self.center_torso = center
                self.face_center_torso = center
                if fit_meta:
                    self.meta = {**self.meta, "center_fit": fit_meta}
                # 故意保留 left_cal/right_cal / left_torso，红蓝继续显示
            out = self.status()
            ij = fit_meta.get("grid_center_ij", [0.5 * grid_n, 0.5 * grid_n])
            out["message"] = (
                f"中心点（{how} → 格子 ({ij[0]:.1f},{ij[1]:.1f})）："
                f"({center[0]:.4f}, {center[1]:.4f}, {center[2]:.4f}) m"
            )
            out["center_m"] = [float(center[0]), float(center[1]), float(center[2])]
            return out
        except Exception as exc:
            self.error = str(exc)
            out = self.status()
            out["ok"] = False
            out["error"] = str(exc)
            return out
