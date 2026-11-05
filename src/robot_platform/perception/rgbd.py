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
    Dc = np.asarray(info["color"].get("D") or [], float).ravel()
    if Dc.size and np.any(np.abs(Dc) > 1e-12) and front.any():
        # 彩色图本身带畸变（自标内参才有；出厂 camera_info 为 0）：按 K+D 投影取色
        proj, _ = cv2.projectPoints(p_color[front].reshape(-1, 1, 3), np.zeros(3), np.zeros(3), Kc, Dc)
        u[front], v[front] = proj[:, 0, 0], proj[:, 0, 1]
    h, w = color.shape[:2]
    ui, vi = np.round(u).astype(int), np.round(v).astype(int)
    inside = front & (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
    return p_color[inside], color[vi[inside], ui[inside]]


def parse_color_intrinsics(doc: Dict[str, Any]) -> Dict[str, Any]:
    """读自标定的彩色内参 JSON。认两种写法：

    - ``camera_matrix`` 3×3 + ``distortion`` + ``image_size`` [w, h]（相机内参标定工具导出的格式）
    - ``K`` 9 个数 + ``D`` + ``size`` [w, h]（与 camera_info 同格式）
    """
    if not isinstance(doc, dict):
        raise ValueError("内参文件应是 JSON 对象")
    if "camera_matrix" in doc:
        K = np.asarray(doc["camera_matrix"], float)
    elif "K" in doc:
        K = np.asarray(doc["K"], float)
    else:
        raise ValueError("内参文件里没有 camera_matrix 或 K")
    if K.size != 9:
        raise ValueError("相机矩阵应是 3×3")
    K = K.reshape(3, 3)
    D = np.asarray(doc.get("distortion", doc.get("D")) or [0.0] * 5, float).ravel()
    if D.size not in (4, 5, 8, 12, 14):
        raise ValueError(f"畸变系数个数 {D.size} 不对（应为 4/5/8/12/14 个）")
    size = doc.get("image_size", doc.get("size"))
    if not (isinstance(size, (list, tuple)) and len(size) == 2):
        raise ValueError("缺 image_size [宽, 高]")
    w, h = int(size[0]), int(size[1])
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    if not (w > 0 and h > 0 and fx > 0 and fy > 0 and 0 < cx < w and 0 < cy < h):
        raise ValueError("内参数值不合理（焦距应为正，主点应在图内）")
    if not np.all(np.isfinite(K)) or not np.all(np.isfinite(D)):
        raise ValueError("内参里有非数值")
    rms = doc.get("rms_px")
    return {"K": K.reshape(-1).tolist(), "D": D.tolist(), "size": [w, h],
            "rms_px": None if rms is None else float(rms), "saved_at": doc.get("saved_at")}


def adapt_intrinsics(intr: Dict[str, Any], size_to) -> Dict[str, Any]:
    """把一种分辨率下标的内参换到相机当前出图的分辨率。

    D435 彩色各分辨率都由同一传感器缩放后居中裁切得到（出厂内参 1280×720 与 640×480 之间
    正好满足：按高缩放 2/3、左右各裁 106.7 px），所以换算为：缩放 s = max(w2/w1, h2/h1)，
    主点再平移居中裁切量。畸变系数作用于归一化坐标，不随之改变。
    """
    w1, h1 = intr["size"]
    w2, h2 = int(size_to[0]), int(size_to[1])
    K = np.asarray(intr["K"], float).reshape(3, 3).copy()
    if (w1, h1) == (w2, h2):
        return dict(intr, K=K.reshape(-1).tolist(), converted_from=None)
    s = max(w2 / w1, h2 / h1)
    ox, oy = (w2 - s * w1) / 2.0, (h2 - s * h1) / 2.0
    K[0, 0] *= s
    K[1, 1] *= s
    K[0, 2] = K[0, 2] * s + ox
    K[1, 2] = K[1, 2] * s + oy
    return dict(intr, K=K.reshape(-1).tolist(), size=[w2, h2], converted_from=[w1, h1])


def colorize_depth(depth: np.ndarray, scale_m: float = 0.001, far_m: float = 4.0) -> np.ndarray:
    """深度可视化：近红远蓝，无效为黑。"""
    m = depth.astype(np.float32) * scale_m
    valid = m > 0
    norm = np.clip(m / far_m, 0, 1)
    img = cv2.applyColorMap((255 * (1 - norm)).astype(np.uint8), cv2.COLORMAP_JET)
    img[~valid] = 0
    return img


def write_session_intrinsics(data_dir: Path, info: Dict[str, Any], square_size_mm: float, n_images: int,
                             source: str = "") -> Dict[str, Any]:
    """RGB-D 会话：把彩色内参写成与双目标定同格式的 yaml（只有 left = 彩色），供求解与保存沿用。

    source 为空表示 info 里是出厂 camera_info；否则是所用自标内参文件名（info 已替换好）。
    """
    c = info["color"]
    K = _K(c)
    D = np.asarray(c.get("D") or [0.0] * 5, float).reshape(1, -1)
    yaml_path = data_dir / "stereo_calibration.yaml"
    fs = cv2.FileStorage(str(yaml_path), cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", int(c["size"][0]))
    fs.write("image_height", int(c["size"][1]))
    fs.write("left_camera_matrix", K)
    fs.write("left_dist_coeffs", D)
    fs.write("source", f"color intrinsics file {source} (RGB-D)" if source else "factory camera_info (RGB-D)")
    fs.release()
    fit = {
        "yaml": str(yaml_path),
        "source": source or "factory",
        "camera": "rgbd",
        "num_pairs": int(n_images),
        "image_size": [int(c["size"][0]), int(c["size"][1])],
        "square_size_mm": float(square_size_mm),
        "left_fx": float(K[0, 0]),
        "left_fy": float(K[1, 1]),
        "left_rms_px": None,
        "note": (f"RGB-D 相机：内参用自标定文件「{source}」（含畸变）" if source
                 else "RGB-D 相机：内参用出厂 camera_info，未从图像重新标定"),
    }
    (data_dir / "intrinsics_fit.json").write_text(json.dumps(fit, indent=2, ensure_ascii=False), encoding="utf-8")
    return fit
