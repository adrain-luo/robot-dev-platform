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
        self._prepare((width, height))

    @classmethod
    def from_matrices(cls, K1, d1, K2, d2, R, T, size: Tuple[int, int]) -> "StereoMatcher":
        obj = cls.__new__(cls)
        obj.K1 = np.asarray(K1, dtype=float)
        obj.d1 = np.asarray(d1, dtype=float).reshape(-1, 1)
        obj.K2 = np.asarray(K2, dtype=float)
        obj.d2 = np.asarray(d2, dtype=float).reshape(-1, 1)
        obj.R = np.asarray(R, dtype=float)
        obj.T = np.asarray(T, dtype=float).reshape(3, 1)
        obj._prepare((int(size[0]), int(size[1])))
        return obj

    def _prepare(self, size: Tuple[int, int]) -> None:
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
        disp = smooth_disp(disp)
        points_rect = cv2.reprojectImageTo3D(disp, self.Q)
        bad = ~np.isfinite(points_rect).all(axis=2)
        points_rect = points_rect.copy()
        points_rect[bad] = 0
        # stereoRectify 的 T 用毫米，三维点先是毫米。R1 把原左相机转到校正相机。
        points_m = (self.R1.T @ (points_rect.reshape(-1, 3) / 1000.0).T).T.reshape(points_rect.shape)
        points_m[bad] = np.nan
        return rect_l, disp, points_m


def smooth_disp(disp: np.ndarray) -> np.ndarray:
    """只抹视差里不到 1 个像素的颗粒。更大的深度差，包括桌面和地面，保持原样。"""
    invalid = disp <= 0
    filled = disp.copy()
    filled[invalid] = 0
    smooth = cv2.bilateralFilter(filled, d=5, sigmaColor=0.6, sigmaSpace=3)
    smooth[invalid] = 0
    return smooth


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
    # 二进制小端（每点 15 字节），numpy 一次写出：12 万点约几十毫秒；
    # 原来逐点写 ASCII 在服务进程里要 1–3 s。CloudCompare / Open3D / MeshLab 都能直接打开。
    path.parent.mkdir(parents=True, exist_ok=True)
    n = int(points.shape[0])
    rec = np.empty(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                             ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    p = np.asarray(points, dtype=np.float32).reshape(n, 3)
    c = np.asarray(colors).reshape(n, 3)[:, ::-1].astype(np.uint8)  # BGR → RGB
    rec["x"], rec["y"], rec["z"] = p[:, 0], p[:, 1], p[:, 2]
    rec["red"], rec["green"], rec["blue"] = c[:, 0], c[:, 1], c[:, 2]
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"element vertex {n}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property uchar red\nproperty uchar green\nproperty uchar blue\n"
              "end_header\n")
    with path.open("wb") as f:
        f.write(header.encode("ascii"))
        f.write(rec.tobytes())
    return n
