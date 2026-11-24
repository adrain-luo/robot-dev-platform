"""感知与任务共用的点云几何（纯计算，不依赖任务、运控或硬件）。

从 tasks/board_touch.py 搬过来（原 _sample_point / _fit_plane），算法不变：
感知（perception/detect_localize.py）原来要从任务模块导入它们，依赖方向反了。
"""

from __future__ import annotations

from typing import Any, Dict, Optional

import numpy as np


def sample_point(points: np.ndarray, x: float, y: float) -> Optional[np.ndarray]:
    """在 HxWx3 点云（米）的像素 (x, y) 取一个三维点。

    该像素有效（有限、深度 0.2–2 m）就直接用；否则取周围 9x9 内有效点的中位数，少于 5 个返回 None。
    """
    h, w = points.shape[:2]
    xi = int(np.clip(round(x), 0, w - 1))
    yi = int(np.clip(round(y), 0, h - 1))
    p = points[yi, xi]
    if np.isfinite(p).all() and 0.2 < float(p[2]) < 2.0:
        return p.astype(float)
    r = 4
    patch = points[max(0, yi - r):yi + r + 1, max(0, xi - r):xi + r + 1].reshape(-1, 3)
    ok = np.isfinite(patch).all(axis=1) & (patch[:, 2] > 0.2) & (patch[:, 2] < 2.0)
    if int(ok.sum()) < 5:
        return None
    return np.median(patch[ok], axis=0)


def fit_plane(pts: np.ndarray) -> Dict[str, Any]:
    """最小二乘平面：中心、单位法向、点到平面距离的均方根 / 最大值（mm）、投到平面上的点。"""
    center = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - center, full_matrices=False)
    normal = vt[-1]
    dist_mm = (pts - center) @ normal * 1000.0
    flat = pts - dist_mm[:, None] / 1000.0 * normal
    return {
        "center": center,
        "normal": normal,
        "rms_mm": float(np.sqrt(np.mean(dist_mm ** 2))),
        "max_mm": float(np.max(np.abs(dist_mm))),
        "flat": flat,
    }
