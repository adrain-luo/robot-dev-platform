"""用已保存的双目标定做校正和 SGBM 匹配，得到左相机坐标系下的点。"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np


class StereoMatcher:
    def __init__(self, yaml_path: Path):
        fs = cv2.FileStorage(str(yaml_path), cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise FileNotFoundError(f"打不开标定: {yaml_path}")
        try:
            width = int(fs.getNode("image_width").real())
            height = int(fs.getNode("image_height").real())
            self.K1 = fs.getNode("left_camera_matrix").mat()
            self.d1 = fs.getNode("left_dist_coeffs").mat()
            self.K2 = fs.getNode("right_camera_matrix").mat()
            self.d2 = fs.getNode("right_dist_coeffs").mat()
            self.R = fs.getNode("R").mat()
            self.T = fs.getNode("T").mat()
        finally:
            fs.release()
        if self.K1 is None or self.K2 is None or self.R is None or self.T is None:
            raise ValueError(f"{yaml_path} 缺少双目外参 R/T 或内参")
        size = (width, height)
        self.R1, self.R2, self.P1, self.P2, self.Q, _, _ = cv2.stereoRectify(
            self.K1, self.d1, self.K2, self.d2, size, self.R, self.T,
            alpha=0, flags=cv2.CALIB_ZERO_DISPARITY,
        )
        self.map1_l, self.map2_l = cv2.initUndistortRectifyMap(
            self.K1, self.d1, self.R1, self.P1, size, cv2.CV_16SC2
        )
        self.map1_r, self.map2_r = cv2.initUndistortRectifyMap(
            self.K2, self.d2, self.R2, self.P2, size, cv2.CV_16SC2
        )
        block = 5
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=64,
            blockSize=block,
            P1=8 * 3 * block * block,
            P2=32 * 3 * block * block,
            disp12MaxDiff=1,
            uniquenessRatio=12,
            speckleWindowSize=80,
            speckleRange=2,
            preFilterCap=31,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

    def match(self, left: np.ndarray, right: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """返回校正后的左图、视差（像素）、以及原左相机坐标系下的点（米）。"""
        if left.shape[1] != self.map1_l.shape[1] or left.shape[0] != self.map1_l.shape[0]:
            raise ValueError(f"画面是 {left.shape[1]}x{left.shape[0]}，标定是 {self.map1_l.shape[1]}x{self.map1_l.shape[0]}")
        rect_l = cv2.remap(left, self.map1_l, self.map2_l, cv2.INTER_LINEAR)
        rect_r = cv2.remap(right, self.map1_r, self.map2_r, cv2.INTER_LINEAR)
        gray_l = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(rect_r, cv2.COLOR_BGR2GRAY)
        disp = self.sgbm.compute(gray_l, gray_r).astype(np.float32) / 16.0
        points_rect = cv2.reprojectImageTo3D(disp, self.Q)
        bad = ~np.isfinite(points_rect).all(axis=2)
        points_rect = points_rect.copy()
        points_rect[bad] = 0
        # stereoRectify 的 T 用毫米，三维点先是毫米。R1 把原左相机转到校正相机。
        points_m = (self.R1.T @ (points_rect.reshape(-1, 3) / 1000.0).T).T.reshape(points_rect.shape)
        points_m[bad] = np.nan
        return rect_l, disp, points_m


def colorize_disp(disp: np.ndarray) -> np.ndarray:
    valid = disp > 0
    vis = np.zeros_like(disp, dtype=np.uint8)
    if np.any(valid):
        lo, hi = np.percentile(disp[valid], [2, 98])
        if hi <= lo:
            hi = lo + 1
        vis = np.clip((disp - lo) / (hi - lo) * 255, 0, 255).astype(np.uint8)
        vis[~valid] = 0
    return cv2.applyColorMap(vis, cv2.COLORMAP_TURBO)


def write_ply(path: Path, points: np.ndarray, colors: np.ndarray) -> int:
    """points、colors 已是有效点，Nx3。颜色是 BGR uint8。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    n = int(points.shape[0])
    rgb = colors[:, ::-1]
    with path.open("w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for p, c in zip(points, rgb):
            f.write(f"{p[0]:.5f} {p[1]:.5f} {p[2]:.5f} {int(c[0])} {int(c[1])} {int(c[2])}\n")
    return n
