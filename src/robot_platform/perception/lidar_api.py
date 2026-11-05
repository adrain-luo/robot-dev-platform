"""雷达接口：/api/lidar（状态）、/api/lidar/cloud（一帧点云，已换到躯干系并降采样）、
/api/lidar/config（选雷达）、/api/lidar/stop（停止订阅）。

用哪台雷达：实例 state/lidar.json 的 ``lidar``，没有就用机型 robot.json ``lidars`` 的第一台。
订阅在第一次取用时才建立，20 s 没人取就自动断开，不占 DDS 带宽。
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Dict, Optional
from urllib.parse import parse_qs

import numpy as np

from robot_platform.core import instance as instances
from robot_platform.core.robot_profile import active as active_profile
from robot_platform.perception.lidar import DdsLidar, MockLidar, link_transform

IDLE_CLOSE_S = 20.0


def _jet(t: np.ndarray) -> np.ndarray:
    """0..1 → RGB 0..255（蓝 → 青 → 黄 → 红）。"""
    t = np.clip(t, 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4 * t - 3), 0, 1)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0, 1)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0, 1)
    return (np.stack([r, g, b], axis=1) * 255).astype(np.uint8)


class LidarAPI:
    def __init__(self, app) -> None:
        self.app = app
        self._lock = threading.Lock()
        self._dev = None
        self._dev_key = ""
        self._T: Optional[np.ndarray] = None
        self._last_use = 0.0
        self._watch = threading.Thread(target=self._idle_loop, name="lidar-idle", daemon=True)
        self._watch.start()

    # ------------------------------------------------------------ 配置
    @staticmethod
    def _config() -> Dict[str, Any]:
        p = instances.path("lidar_config")
        if p.is_file():
            try:
                d = json.loads(p.read_text(encoding="utf-8"))
                return d if isinstance(d, dict) else {}
            except (OSError, json.JSONDecodeError):
                return {}
        return {}

    @staticmethod
    def _save_config(cfg: Dict[str, Any]) -> None:
        p = instances.path("lidar_config")
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(cfg, indent=2, ensure_ascii=False), encoding="utf-8")

    def _selected(self):
        lidars = active_profile().lidars
        if not lidars:
            return None
        lid = self._config().get("lidar")
        return lidars.get(lid) or next(iter(lidars.values()))

    def _is_mock(self) -> bool:
        backend = getattr(self.app, "backend", None)
        return getattr(backend, "name", "") == "mock"

    # ------------------------------------------------------------ 设备
    def _device(self):
        spec = self._selected()
        if spec is None:
            raise LookupError(f"机型 {active_profile().name} 没有在 robot.json 里声明雷达")
        mock = self._is_mock() or spec.type == "mock"
        key = f"{spec.id}|{spec.type}|{json.dumps(spec.params, sort_keys=True)}|{mock}"
        with self._lock:
            self._last_use = time.monotonic()
            if self._dev is not None and key == self._dev_key:
                return spec, self._dev
            if self._dev is not None:
                self._dev.close()
                self._dev = None
            settings = self.app.settings
            self._T = link_transform(settings.urdf, settings.base_link, spec.link)
            if mock:
                self._dev = MockLidar(self._T)
            else:
                self._dev = DdsLidar(str(spec.params.get("topic") or ""), settings.dds_domain,
                                     settings.network_interface, settings.unitree_sdk_path)
            self._dev_key = key
            return spec, self._dev

    def _idle_loop(self) -> None:
        while True:
            time.sleep(2.0)
            with self._lock:
                if self._dev is not None and time.monotonic() - self._last_use > IDLE_CLOSE_S:
                    self._dev.close()
                    self._dev = None
                    self._dev_key = ""

    def close(self) -> None:
        with self._lock:
            if self._dev is not None:
                self._dev.close()
            self._dev = None
            self._dev_key = ""

    # ------------------------------------------------------------ 接口
    def status(self) -> Dict[str, Any]:
        prof = active_profile()
        spec = self._selected()
        with self._lock:
            dev = self._dev
        return {
            "ok": True,
            "lidars": [{"id": l.id, "label": l.label, "type": l.type} for l in prof.lidars.values()],
            "lidar": spec.id if spec else None,
            "label": spec.label if spec else "",
            "type": spec.type if spec else "",
            "topic": (spec.params.get("topic") or "") if spec else "",
            "link": spec.link if spec else "",
            "mock": self._is_mock(),
            "running": dev is not None,
            "error": dev.last_error if dev is not None else "",
        }

    def cloud(self, query: str) -> Dict[str, Any]:
        q = {k: v[0] for k, v in parse_qs(query).items()}
        max_points = int(min(30000, max(500, float(q.get("max_points") or 8000))))
        max_range = float(min(40.0, max(0.5, float(q.get("max_range") or 8.0))))
        color = q.get("color") or "height"
        try:
            spec, dev = self._device()
        except (LookupError, ValueError, OSError) as exc:
            return {"ok": False, "error": str(exc)}
        frame = dev.latest()
        if frame is None:
            return {"ok": False, "error": dev.last_error or "没有雷达数据"}
        xyz = frame["xyz"]
        inten = frame["intensity"]
        r = np.linalg.norm(xyz, axis=1)
        ok = np.isfinite(r) & (r > 0.15) & (r < max_range)
        n_raw, n_valid = int(len(xyz)), int(ok.sum())
        xyz = xyz[ok]
        inten = inten[ok] if inten is not None else None
        if len(xyz) > max_points:
            idx = np.linspace(0, len(xyz) - 1, max_points).astype(int)
            xyz = xyz[idx]
            inten = inten[idx] if inten is not None else None
        T = self._T
        pts = (T[:3, :3] @ xyz.T).T + T[:3, 3]
        if color == "intensity" and inten is not None and len(inten):
            hi = float(np.percentile(inten, 98)) or 1.0
            rgb = _jet(inten / hi)
        else:
            z = pts[:, 2]
            lo, hi = (float(np.percentile(z, 2)), float(np.percentile(z, 98))) if len(z) else (0.0, 1.0)
            rgb = _jet((z - lo) / max(hi - lo, 1e-3))
        rr = r[ok]
        return {
            "ok": True,
            "lidar": spec.id,
            "label": spec.label,
            "hz": round(float(frame["hz"]), 1),
            "frame_id": frame["frame_id"],
            "n_raw": n_raw,
            "n_valid": n_valid,
            "n_sent": int(len(pts)),
            "range_m": ({"near": float(rr.min()), "median": float(np.median(rr)), "far": float(rr.max())}
                        if len(rr) else None),
            "xyz": np.round(pts, 3).reshape(-1).tolist(),
            "rgb": rgb.reshape(-1).astype(int).tolist(),
        }

    def handle_get(self, path: str, query: str) -> Optional[Dict[str, Any]]:
        if path == "/api/lidar":
            return self.status()
        if path == "/api/lidar/cloud":
            return self.cloud(query)
        return None

    def handle_post(self, path: str, body: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        if path == "/api/lidar/config":
            lid = str(body.get("lidar") or "")
            if lid not in active_profile().lidars:
                return {"ok": False, "error": f"本机型没有雷达 {lid!r}"}
            cfg = self._config()
            cfg["lidar"] = lid
            self._save_config(cfg)
            return self.status()
        if path == "/api/lidar/stop":
            self.close()
            return self.status()
        return None
