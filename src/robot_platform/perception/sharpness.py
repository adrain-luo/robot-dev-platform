"""画面清晰度：调焦用。

清晰度 = 灰度图 Sobel 梯度能量（Tenengrad，gx² + gy²）的局部均值。对焦越准，边缘越陡，数值越大；
它也和纹理多少有关，所以看的是「同一处画面随焦距环转动怎么变」，不同地方之间不好直接比。

- :func:`sharpness_map`：逐像素的局部清晰度（窗口内均值）；
- :func:`frame_score`：整幅 / 中心区域一个数（×1000，方便读）；
- :class:`Scale`：对数刻度的上下限。默认跟着画面自动走（指数平滑）；锁定后不再变，转焦距环时颜色才可比；
- :func:`overlay`：分档伪彩掩膜（带透明度）+ 档间等高线。
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

LEVELS = 8  # 掩膜分几档
MIN_SPAN = 0.6  # 刻度最小跨度（log10，约 4 倍）
EPS = 1e-7
COLORMAP = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)


def _gray(img: np.ndarray) -> np.ndarray:
    g = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return g.astype(np.float32) / 255.0


def gradient_energy(img: np.ndarray) -> np.ndarray:
    g = _gray(img)
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
    return gx * gx + gy * gy


def sharpness_map(img: np.ndarray, win: int = 15) -> np.ndarray:
    """局部清晰度（窗口 win×win 内梯度能量的均值），与输入同尺寸。"""
    win = max(3, int(win) | 1)
    return cv2.boxFilter(gradient_energy(img), -1, (win, win), borderType=cv2.BORDER_REFLECT)


def center_roi(shape: Tuple[int, ...], frac: float = 1 / 3) -> Tuple[slice, slice]:
    h, w = shape[:2]
    rh, rw = int(h * frac), int(w * frac)
    y0, x0 = (h - rh) // 2, (w - rw) // 2
    return slice(y0, y0 + rh), slice(x0, x0 + rw)


def frame_score(img: np.ndarray) -> Dict[str, float]:
    """整幅与中心 1/3 区域的清晰度（梯度能量均值 ×1000）。"""
    e = gradient_energy(img)
    ys, xs = center_roi(e.shape)
    return {"whole": float(e.mean() * 1000.0), "center": float(e[ys, xs].mean() * 1000.0)}


@dataclass
class ScaleState:
    lo: float
    hi: float
    locked: bool


class Scale:
    """掩膜的对数刻度 [lo, hi]（log10 清晰度）。未锁定时按每帧的 2% / 99.5% 分位指数平滑跟随。"""

    def __init__(self, smooth: float = 0.15) -> None:
        self._lock = threading.Lock()
        self._lo: Optional[float] = None
        self._hi: Optional[float] = None
        self._smooth = float(smooth)
        self.locked = False

    def update(self, logmap: np.ndarray) -> ScaleState:
        lo, hi = np.percentile(logmap, [2.0, 99.5])
        with self._lock:
            if self._lo is None:
                self._lo, self._hi = float(lo), float(hi)
            elif not self.locked:
                a = self._smooth
                self._lo += a * (float(lo) - self._lo)
                self._hi += a * (float(hi) - self._hi)
            if self._hi - self._lo < MIN_SPAN:  # 至少约 4 倍：别把纹理的小起伏 / 噪声放大成满屏等高线
                mid = 0.5 * (self._hi + self._lo)
                self._lo, self._hi = mid - MIN_SPAN / 2, mid + MIN_SPAN / 2
            return ScaleState(self._lo, self._hi, self.locked)

    def set_locked(self, on: bool) -> None:
        with self._lock:
            self.locked = bool(on)

    def reset(self) -> None:
        with self._lock:
            self._lo = self._hi = None
            self.locked = False

    def state(self) -> Optional[ScaleState]:
        with self._lock:
            return None if self._lo is None else ScaleState(self._lo, self._hi, self.locked)


def level_map(img: np.ndarray, scale: Scale, work_width: int = 320) -> Tuple[np.ndarray, ScaleState]:
    """在缩小的图上算局部清晰度 → 0..1（按刻度），返回 (归一化图（缩小尺寸）, 刻度)。"""
    h, w = img.shape[:2]
    s = min(1.0, work_width / float(w))
    small = cv2.resize(img, (max(8, int(w * s)), max(8, int(h * s))), interpolation=cv2.INTER_AREA) if s < 1 else img
    win = max(5, int(small.shape[1] / 8) | 1)  # 约画面宽 1/8：比纹理大，等高线才不碎
    logm = np.log10(sharpness_map(small, win) + EPS)
    st = scale.update(logm)
    t = np.clip((logm - st.lo) / max(st.hi - st.lo, 1e-6), 0.0, 1.0)
    return t, st


def overlay(img: np.ndarray, scale: Scale, alpha: float = 0.45, levels: int = LEVELS,
            work_width: int = 320) -> Tuple[np.ndarray, ScaleState]:
    """原图 + 分档伪彩掩膜（透明度 alpha）+ 档间等高线。蓝 = 模糊，红 = 清晰（相对刻度）。"""
    h, w = img.shape[:2]
    t, st = level_map(img, scale, work_width)
    t = cv2.GaussianBlur(t, (0, 0), 3.0)  # 等高线别太碎
    q = np.minimum(np.floor(t * levels), levels - 1)  # 0..levels-1
    band = ((q + 0.5) / levels * 255.0).astype(np.uint8)
    color = cv2.applyColorMap(band, COLORMAP)
    color = cv2.resize(color, (w, h), interpolation=cv2.INTER_NEAREST)
    a = float(np.clip(alpha, 0.0, 1.0))
    base = img if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    out = cv2.addWeighted(base, 1.0 - a, color, a, 0.0)
    sx, sy = w / float(t.shape[1]), h / float(t.shape[0])
    for k in range(1, levels):
        m = (q >= k).astype(np.uint8)
        cs, _ = cv2.findContours(m, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cs = [np.round(c.astype(np.float32) * [sx, sy]).astype(np.int32) for c in cs if len(c) >= 3]
        if cs:
            cv2.drawContours(out, cs, -1, (255, 255, 255), 1, cv2.LINE_AA)
    return out, st
