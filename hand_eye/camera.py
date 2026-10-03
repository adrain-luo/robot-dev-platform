"""头目左右拼图拆成左、右。相机进程是 teleimager，本模块不另起相机服务。"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np


class HeadCamera:
    def __init__(self, source: str = "teleimager", host: str = "192.168.123.164", teleimager_path: str = ""):
        if source not in ("teleimager", "mock"):
            raise ValueError(f"未知相机来源: {source!r}（teleimager 或 mock）")
        self.source = source
        self.host = host
        self.teleimager_path = teleimager_path
        self.last_error = ""
        self._client = None
        self._lock = threading.Lock()
        self._mock_t0 = time.monotonic()

    def grab_pair(self, timeout: float = 2.0) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if self.source == "mock":
            self.last_error = ""
            return self._mock_pair()
        with self._lock:
            if not self._ensure_client():
                return None
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                try:
                    frame, _fps = self._client.get_head_frame()
                except Exception as exc:
                    self.last_error = f"读头目失败: {exc}"
                    self._drop_client()
                    time.sleep(0.1)
                    if not self._ensure_client():
                        return None
                    continue
                if frame is not None:
                    h, w = frame.shape[:2]
                    if w % 2 != 0:
                        self.last_error = f"头目宽度不是偶数，无法左右拆分: {w}"
                        return None
                    left = frame[:, : w // 2].copy()
                    right = frame[:, w // 2 :].copy()
                    self.last_error = ""
                    return left, right
                time.sleep(0.05)
            self.last_error = f"{self.host} 在 {timeout:.0f}s 内没有头目画面"
            return None

    def close(self) -> None:
        with self._lock:
            self._drop_client()

    def _ensure_client(self) -> bool:
        if self._client is not None:
            return True
        path = (self.teleimager_path or "").strip()
        if path and path not in sys.path:
            if not Path(path).is_dir():
                self.last_error = f"teleimager 目录不存在: {path}"
                return False
            sys.path.insert(0, path)
        try:
            from teleimager.image_client import ImageClient
        except Exception as exc:
            self.last_error = f"无法导入 teleimager: {exc}"
            return False
        try:
            self._client = ImageClient(host=self.host)
        except Exception as exc:
            self.last_error = f"连接 {self.host} 的相机进程失败: {exc}"
            self._client = None
            return False
        self.last_error = ""
        return True

    def _drop_client(self) -> None:
        if self._client is None:
            return
        try:
            self._client.close()
        except Exception:
            pass
        self._client = None

    def _mock_pair(self) -> Tuple[np.ndarray, np.ndarray]:
        h, w = 480, 640
        img = np.full((h, w, 3), 30, dtype=np.uint8)
        t = time.monotonic() - self._mock_t0
        cx = int(w / 2 + 120 * np.cos(t))
        cy = int(h / 2 + 80 * np.sin(t * 0.7))
        cv2.rectangle(img, (cx - 60, cy - 40), (cx + 60, cy + 40), (200, 200, 200), -1)
        cv2.putText(img, "MOCK LEFT", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 212, 255), 2)
        right = img.copy()
        cv2.putText(right, "MOCK RIGHT", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 212, 255), 2)
        return img, right
