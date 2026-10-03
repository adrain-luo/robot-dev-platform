"""用同一会话的左右目棋盘图估计内参，写成 stereo_calibration.yaml。

物体点用毫米，和旧双目标定一致。焦距在像素里，不随格子边长缩放；
手眼的 solvePnP 另用米，和正运动学对齐。
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

from solve_core import detect_corners_count


def _object_points_mm(board_size: Tuple[int, int], square_size_mm: float) -> np.ndarray:
    cols, rows = board_size
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square_size_mm)
    return objp


def _align_right(left_corners: np.ndarray, right_corners: np.ndarray) -> np.ndarray:
    """左右目角点起点可能相反。基线很小时，反序更接近就翻过来。"""
    left = left_corners.reshape(-1, 2)
    right = right_corners.reshape(-1, 2)
    same = float(np.mean(np.linalg.norm(left - right, axis=1)))
    rev = float(np.mean(np.linalg.norm(left - right[::-1], axis=1)))
    if rev < same:
        return right_corners[::-1].copy()
    return right_corners


def calibrate_session(
    data_dir: Path,
    board_size: Tuple[int, int],
    square_size_mm: float,
    log: Optional[List[str]] = None,
) -> dict:
    def say(msg: str) -> None:
        print(msg)
        if log is not None:
            log.append(msg)

    left_dir = data_dir / "left"
    right_dir = data_dir / "right"
    if not left_dir.is_dir() or not right_dir.is_dir():
        raise FileNotFoundError(f"{data_dir} 需要同时有 left/ 和 right/")

    objp = _object_points_mm(board_size, square_size_mm)
    obj_points: List[np.ndarray] = []
    left_points: List[np.ndarray] = []
    right_points: List[np.ndarray] = []
    image_size = None

    for left_file in sorted(left_dir.glob("*.jpg")):
        right_file = right_dir / left_file.name
        if not right_file.is_file():
            say(f"  [{left_file.stem}] 跳过：没有右目")
            continue
        left = cv2.imread(str(left_file))
        right = cv2.imread(str(right_file))
        if left is None or right is None:
            say(f"  [{left_file.stem}] 跳过：图读不了")
            continue
        lc = detect_corners_count(left, board_size)
        rc = detect_corners_count(right, board_size)
        if lc is None or rc is None:
            say(f"  [{left_file.stem}] 跳过：棋盘没配对")
            continue
        rc = _align_right(lc, rc)
        obj_points.append(objp)
        left_points.append(lc)
        right_points.append(rc)
        if image_size is None:
            h, w = left.shape[:2]
            image_size = (w, h)
        say(f"  [{left_file.stem}] 内参样本")

    n = len(obj_points)
    say(f"[内参] 可用左右对 {n}")
    if n < 3:
        raise RuntimeError("估内参至少要 3 对同时检出棋盘的左右图")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    left_rms, K1, d1, _r1, _t1 = cv2.calibrateCamera(obj_points, left_points, image_size, None, None)
    right_rms, K2, d2, _r2, _t2 = cv2.calibrateCamera(obj_points, right_points, image_size, None, None)
    stereo_rms, K1, d1, K2, d2, R, T, E, F = cv2.stereoCalibrate(
        obj_points, left_points, right_points, K1, d1, K2, d2, image_size,
        criteria=criteria,
        flags=cv2.CALIB_FIX_INTRINSIC,
    )
    baseline_mm = float(np.linalg.norm(T))
    say(
        f"[内参] 左 fx={K1[0, 0]:.1f} RMS {left_rms:.3f}px · "
        f"右 fx={K2[0, 0]:.1f} RMS {right_rms:.3f}px · "
        f"双目 RMS {stereo_rms:.3f}px · 基线 {baseline_mm:.1f} mm"
    )

    yaml_path = data_dir / "stereo_calibration.yaml"
    fs = cv2.FileStorage(str(yaml_path), cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", int(image_size[0]))
    fs.write("image_height", int(image_size[1]))
    fs.write("left_camera_matrix", K1)
    fs.write("left_dist_coeffs", d1)
    fs.write("right_camera_matrix", K2)
    fs.write("right_dist_coeffs", d2)
    fs.write("R", R)
    fs.write("T", T)
    fs.write("E", E)
    fs.write("F", F)
    fs.release()
    say(f"[内参] 已写入 {yaml_path}")
    return {
        "yaml": str(yaml_path),
        "num_pairs": n,
        "image_size": [int(image_size[0]), int(image_size[1])],
        "square_size_mm": float(square_size_mm),
        "left_rms_px": float(left_rms),
        "right_rms_px": float(right_rms),
        "stereo_rms_px": float(stereo_rms),
        "left_fx": float(K1[0, 0]),
        "left_fy": float(K1[1, 1]),
        "right_fx": float(K2[0, 0]),
        "right_fy": float(K2[1, 1]),
        "baseline_mm": baseline_mm,
    }
