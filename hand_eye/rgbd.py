"""RGB-D 相机（D435i）：深度 → 彩色光学系下的点云，以及给手眼求解用的内参文件。

点云坐标与手眼结果一致，都在彩色相机光学系（X 右、Y 下、Z 前），单位米。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Tuple

import cv2
import numpy as np


def _K(src: Dict[str, Any]) -> np.ndarray:
    return np.asarray(src["K"], float).reshape(3, 3)


def depth_to_color_cloud(depth: np.ndarray, color: np.ndarray, info: Dict[str, Any], *,
                         near_m: float = 0.15, far_m: float = 12.0, stride: int = 1
                         ) -> Tuple[np.ndarray, np.ndarray]:
    """深度图反投影到深度相机系，经外参换到彩色系，再投回彩色图取颜色。

    返回 (points_m[N,3] 彩色光学系, colors_bgr[N,3])；只保留落在彩色图内、深度在 [near, far] 的点。
    """
    scale = float(info.get("depth_scale_m", 0.001))
    Kd, Kc = _K(info["depth"]), _K(info["color"])
    ext = info.get("depth_to_color") or {}
    R = np.asarray(ext.get("R", np.eye(3)), float)
    t = np.asarray(ext.get("t", [0.0, 0.0, 0.0]), float)

    d = depth[::stride, ::stride].astype(np.float64) * scale
    vs, us = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    ok = (d > near_m) & (d < far_m)
    z = d[ok]
    x = (us[ok] - Kd[0, 2]) * z / Kd[0, 0]
    y = (vs[ok] - Kd[1, 2]) * z / Kd[1, 1]
    p_depth = np.stack([x, y, z], axis=1)
    p_color = p_depth @ R.T + t

    zc = p_color[:, 2]
    front = zc > 1e-6
    u = np.full(zc.shape, -1.0)
    v = np.full(zc.shape, -1.0)
    u[front] = Kc[0, 0] * p_color[front, 0] / zc[front] + Kc[0, 2]
    v[front] = Kc[1, 1] * p_color[front, 1] / zc[front] + Kc[1, 2]
    h, w = color.shape[:2]
    ui, vi = np.round(u).astype(int), np.round(v).astype(int)
    inside = front & (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
    return p_color[inside], color[vi[inside], ui[inside]]


def colorize_depth(depth: np.ndarray, scale_m: float = 0.001, far_m: float = 4.0) -> np.ndarray:
    """深度可视化：近红远蓝，无效为黑。"""
    m = depth.astype(np.float32) * scale_m
    valid = m > 0
    norm = np.clip(m / far_m, 0, 1)
    img = cv2.applyColorMap((255 * (1 - norm)).astype(np.uint8), cv2.COLORMAP_JET)
    img[~valid] = 0
    return img


def write_session_intrinsics(data_dir: Path, info: Dict[str, Any], square_size_mm: float, n_images: int
                             ) -> Dict[str, Any]:
    """RGB-D 会话：用出厂内参写成与双目标定同格式的 yaml（只有 left = 彩色），供求解与保存沿用。"""
    c = info["color"]
    K = _K(c)
    D = np.asarray(c.get("D") or [0.0] * 5, float).reshape(1, -1)
    yaml_path = data_dir / "stereo_calibration.yaml"
    fs = cv2.FileStorage(str(yaml_path), cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", int(c["size"][0]))
    fs.write("image_height", int(c["size"][1]))
    fs.write("left_camera_matrix", K)
    fs.write("left_dist_coeffs", D)
    fs.write("source", "factory camera_info (RGB-D)")
    fs.release()
    fit = {
        "yaml": str(yaml_path),
        "source": "factory",
        "camera": "rgbd",
        "num_pairs": int(n_images),
        "image_size": [int(c["size"][0]), int(c["size"][1])],
        "square_size_mm": float(square_size_mm),
        "left_fx": float(K[0, 0]),
        "left_fy": float(K[1, 1]),
        "left_rms_px": None,
        "note": "RGB-D 相机：内参用出厂 camera_info，未从图像重新标定",
    }
    (data_dir / "intrinsics_fit.json").write_text(json.dumps(fit, indent=2, ensure_ascii=False), encoding="utf-8")
    return fit
