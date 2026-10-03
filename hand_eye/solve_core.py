"""眼在手外：棋盘在手上，相机固定，求 T_cam2base（相机 → torso_link）。"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np


def make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def invert_T(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def rot_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    """URDF 固定轴 RPY：R = Rz(yaw) Ry(pitch) Rx(roll)。"""
    pitch = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0]))
    if abs(math.cos(pitch)) < 1e-8:
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    else:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def geodesic_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    R = Ra.T @ Rb
    c = (np.trace(R) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    return math.degrees(math.acos(c))


METHOD_MAP = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def load_intrinsics(yaml_path: Path, eye: str) -> Tuple[np.ndarray, np.ndarray]:
    """读 stereo yaml 里的 {eye}_camera_matrix / {eye}_dist_coeffs。路径由调用方传入。"""
    fs = cv2.FileStorage(str(yaml_path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(f"打不开内参: {yaml_path}")
    K = fs.getNode(f"{eye}_camera_matrix").mat()
    dist = fs.getNode(f"{eye}_dist_coeffs").mat()
    fs.release()
    if K is None or dist is None:
        raise ValueError(f"{yaml_path} 缺少 {eye}_camera_matrix / {eye}_dist_coeffs")
    return K.astype(np.float64), dist.astype(np.float64)


def object_points_m(board_size: Tuple[int, int], square_size_mm: float) -> np.ndarray:
    cols, rows = board_size
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= (square_size_mm / 1000.0)
    return objp


def detect_corners_count(img: np.ndarray, board_size: Tuple[int, int]) -> Optional[np.ndarray]:
    """内角数量必须正好是 cols*rows。SB 失败时再试一次亚像素。"""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(gray, board_size)
    need = board_size[0] * board_size[1]
    if found and corners is not None and corners.shape[0] == need:
        return corners
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, board_size, flags=flags)
    if not found or corners is None or corners.shape[0] != need:
        return None
    return cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
    )


def reorder_q(q: np.ndarray, stored_names: Sequence[str], model_names: Sequence[str]) -> np.ndarray:
    if not stored_names:
        if q.size != len(model_names):
            raise ValueError(f"关节数 {q.size} 与模型 {len(model_names)} 不一致，且记录里没有关节名")
        return q
    index = {n: i for i, n in enumerate(stored_names)}
    out = np.empty(len(model_names), dtype=float)
    for i, name in enumerate(model_names):
        if name not in index:
            raise ValueError(f"记录里没有关节 {name}")
        out[i] = q[index[name]]
    return out


@dataclass
class SolveOptions:
    method: str = "park"
    no_reject: bool = False
    max_trans_res_mm: float = 8.0
    max_rot_res_deg: float = 2.5
    reject_iters: int = 5
    base_link: str = "torso_link"
    tip_link: str = "left_wrist_yaw_link"
    square_size_mm: float = 0.0
    intrinsics: str = ""


def solve_eye(
    eye: str,
    data_dir: Path,
    K: np.ndarray,
    dist: np.ndarray,
    objp: np.ndarray,
    board_size: Tuple[int, int],
    model,
    opt: SolveOptions,
    log: Optional[List[str]] = None,
) -> Optional[dict]:
    def say(msg: str) -> None:
        print(msg)
        if log is not None:
            log.append(msg)

    img_dir = data_dir / eye
    joints_dir = data_dir / "joints"
    if not img_dir.exists():
        say(f"[{eye}] 跳过：没有 {img_dir}")
        return None

    R_base2gripper: List[np.ndarray] = []
    t_base2gripper: List[np.ndarray] = []
    R_target2cam: List[np.ndarray] = []
    t_target2cam: List[np.ndarray] = []
    T_g2b_used: List[np.ndarray] = []
    T_t2c_used: List[np.ndarray] = []

    img_files = sorted(img_dir.glob("*.jpg"))
    say(f"[{eye}] {len(img_files)} 张图")

    for lf in img_files:
        stem = lf.stem
        jf = joints_dir / f"{stem}.json"
        if not jf.exists():
            say(f"  [{stem}] 跳过：没有关节")
            continue
        img = cv2.imread(str(lf))
        if img is None:
            say(f"  [{stem}] 跳过：图读不了")
            continue
        corners = detect_corners_count(img, board_size)
        if corners is None:
            say(f"  [{stem}] 跳过：没检出棋盘")
            continue
        ok, rvec, tvec = cv2.solvePnP(objp, corners, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            say(f"  [{stem}] 跳过：solvePnP 失败")
            continue
        R_t2c, _ = cv2.Rodrigues(rvec)
        t_t2c = tvec.reshape(3)

        rec = json.loads(jf.read_text(encoding="utf-8"))
        q = np.asarray(rec.get("q_rad", []), dtype=float).reshape(-1)
        try:
            q_ordered = reorder_q(q, list(rec.get("joint_names") or []), model.joint_names)
        except ValueError as exc:
            say(f"  [{stem}] 跳过：{exc}")
            continue
        p_tip, R_tip = model.fk(q_ordered)
        T_g2b = make_T(R_tip, p_tip)
        T_b2g = invert_T(T_g2b)
        R_base2gripper.append(T_b2g[:3, :3])
        t_base2gripper.append(T_b2g[:3, 3])
        R_target2cam.append(R_t2c)
        t_target2cam.append(t_t2c)
        T_g2b_used.append(T_g2b)
        T_t2c_used.append(make_T(R_t2c, t_t2c))

    n = len(R_target2cam)
    say(f"[{eye}] 可用样本 {n}")
    if n < 3:
        say(f"[{eye}] 至少要 3 个可用样本，建议 8 个以上、姿态拉开")
        return None

    method = METHOD_MAP[opt.method]

    def solve_subset(idx):
        Rc2b, tc2b = cv2.calibrateHandEye(
            [R_base2gripper[i] for i in idx], [t_base2gripper[i] for i in idx],
            [R_target2cam[i] for i in idx], [t_target2cam[i] for i in idx],
            method=method,
        )
        return make_T(Rc2b, tc2b.reshape(3))

    def residuals(T_cb):
        consts_ = [invert_T(T_g2b_used[i]) @ T_cb @ T_t2c_used[i] for i in range(n)]
        trans_ = np.array([c[:3, 3] for c in consts_])
        med_t = np.median(trans_, axis=0)
        tr_mm = np.linalg.norm(trans_ - med_t, axis=1) * 1000.0
        rot_sum = [sum(geodesic_deg(consts_[i][:3, :3], consts_[j][:3, :3]) for j in range(n))
                   for i in range(n)]
        ref_R = consts_[int(np.argmin(rot_sum))][:3, :3]
        rot_deg = np.array([geodesic_deg(ref_R, c[:3, :3]) for c in consts_])
        return tr_mm, rot_deg

    inliers = list(range(n))
    T_cam2base = solve_subset(inliers)
    if not opt.no_reject:
        for it in range(opt.reject_iters):
            tr_mm, rot_deg = residuals(T_cam2base)
            keep = [i for i in range(n)
                    if tr_mm[i] <= opt.max_trans_res_mm and rot_deg[i] <= opt.max_rot_res_deg]
            if len(keep) < 6:
                say(f"[{eye}] 剔除第 {it} 轮只剩 {len(keep)} 个内点，停止")
                break
            if keep == inliers:
                say(f"[{eye}] 剔除收敛，{len(keep)}/{n}")
                break
            say(f"[{eye}] 剔除第 {it} 轮 {len(keep)}/{n}")
            inliers = keep
            T_cam2base = solve_subset(inliers)

    tr_mm_all, rot_deg_all = residuals(T_cam2base)
    trans_res_mm = np.array([tr_mm_all[i] for i in inliers])
    rot_res_deg = np.array([rot_deg_all[i] for i in inliers])
    rpy = rot_to_rpy(T_cam2base[:3, :3])
    t = T_cam2base[:3, 3].reshape(3)
    say(
        f"[{eye}] xyz(m)=[{t[0]:+.5f}, {t[1]:+.5f}, {t[2]:+.5f}]  "
        f"平移残差 mean={trans_res_mm.mean():.2f}mm max={trans_res_mm.max():.2f}mm  "
        f"内点 {len(inliers)}/{n}"
    )
    return {
        "eye": eye,
        "base_link": opt.base_link,
        "tip_link": opt.tip_link,
        "method": opt.method,
        "num_samples": n,
        "num_inliers": len(inliers),
        "inlier_indices": inliers,
        "board_size": list(board_size),
        "square_size_mm": opt.square_size_mm,
        "intrinsics_yaml": str(Path(opt.intrinsics).resolve()) if opt.intrinsics else "",
        "T_cam2base": T_cam2base.tolist(),
        "R_cam2base": T_cam2base[:3, :3].tolist(),
        "t_cam2base_m": t.tolist(),
        "rpy_rad": list(rpy),
        "rpy_deg": [math.degrees(x) for x in rpy],
        "residual_translation_mm": {"mean": float(trans_res_mm.mean()), "max": float(trans_res_mm.max())},
        "residual_rotation_deg": {"mean": float(rot_res_deg.mean()), "max": float(rot_res_deg.max())},
    }


def stereo_cross_check(results: dict) -> Optional[dict]:
    if "left" not in results or "right" not in results:
        return None
    T_l = np.array(results["left"]["T_cam2base"])
    T_r = np.array(results["right"]["T_cam2base"])
    T_r2l = invert_T(T_l) @ T_r
    return {
        "T_right2left": T_r2l.tolist(),
        "stereo_baseline_mm": float(np.linalg.norm(T_r2l[:3, 3]) * 1000.0),
    }
