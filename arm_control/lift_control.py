"""立柱高度闭环：读 hispeed.y，发 AgvClient.HeightAdjust(vz)。

PD 参数对齐官方 g1d_height_control.cpp。
"""

from __future__ import annotations

import threading
import time
from typing import Any, Dict, Optional

from dds.agv_client import AgvClient
from dds.hispeed import HispeedStateReader
from model.layout import LIFT_STROKE_MAX_M
import lift_calib

_KP = 20.0
_KD = 0.3
_MAX_VZ = 0.8
_ARRIVE_TOL_M = 0.0015
_ARRIVE_STABLE = 12
_DT_S = 0.05
_HEIGHT_TIMEOUT_S = 0.6


class LiftController:
    def __init__(
        self,
        *,
        hispeed: HispeedStateReader,
        agv: AgvClient,
    ) -> None:
        self._hispeed = hispeed
        self._agv = agv
        self._lock = threading.Lock()
        self._target_m: Optional[float] = None
        self._busy = False
        self._last_error: Optional[str] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def status(self) -> Dict[str, Any]:
        st = self._hispeed.latest()
        zero = lift_calib.load_zero()
        with self._lock:
            target = self._target_m
            busy = self._busy
            err = self._last_error
        height = None if st is None else float(st.height_m)
        stroke = None if height is None else max(0.0, min(LIFT_STROKE_MAX_M, height - zero))
        return {
            "busy": busy,
            "target_m": target,
            "target_mm": None if target is None else target * 1000.0,
            "height_m": height,
            "height_mm": None if height is None else height * 1000.0,
            "stroke_m": stroke,
            "stroke_mm": None if stroke is None else stroke * 1000.0,
            "zero_m": zero,
            "stroke_max_m": LIFT_STROKE_MAX_M,
            "stroke_max_mm": LIFT_STROKE_MAX_M * 1000.0,
            "error": err,
        }

    def current_height(self) -> Optional[float]:
        st = self._hispeed.latest()
        return None if st is None else float(st.height_m)

    def clamp_height(self, height_m: float) -> float:
        zero = lift_calib.load_zero()
        lo = zero
        hi = zero + LIFT_STROKE_MAX_M
        return max(lo, min(hi, float(height_m)))

    def goto(self, height_m: float) -> Dict[str, Any]:
        target = self.clamp_height(height_m)
        with self._lock:
            self._target_m = target
            self._last_error = None
        self._ensure_thread()
        return {"ok": True, **self.status()}

    def step(self, delta_m: float) -> Dict[str, Any]:
        cur = self.current_height()
        if cur is None:
            return {"ok": False, "error": "还没有立柱高度反馈", **self.status()}
        return self.goto(cur + float(delta_m))

    def stop(self) -> Dict[str, Any]:
        self._stop.set()
        with self._lock:
            self._target_m = None
            self._busy = False
            self._last_error = None
        try:
            code = self._agv.stop_height()
        except Exception as exc:
            return {"ok": False, "error": f"{type(exc).__name__}: {exc}", **self.status()}
        if code != 0:
            return {"ok": False, "error": f"HeightAdjust(0) 返回 {code}", **self.status()}
        return {"ok": True, **self.status()}

    def _ensure_thread(self) -> None:
        self._stop.clear()
        t = self._thread
        if t is not None and t.is_alive():
            return
        self._thread = threading.Thread(target=self._loop, name="lift-pd", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        prev_err = None
        prev_t = time.monotonic()
        arrive = 0
        with self._lock:
            self._busy = True
        try:
            while not self._stop.is_set():
                with self._lock:
                    target = self._target_m
                if target is None:
                    break
                st = self._hispeed.latest()
                age = self._hispeed.age_s()
                if st is None or age > _HEIGHT_TIMEOUT_S:
                    with self._lock:
                        self._last_error = "立柱高度超时"
                    self._agv.stop_height()
                    break
                height = float(st.height_m)
                now = time.monotonic()
                dt = max(1e-3, now - prev_t)
                prev_t = now
                err = target - height
                d_err = 0.0 if prev_err is None else (err - prev_err) / dt
                prev_err = err
                vz = _KP * err + _KD * d_err
                vz = max(-_MAX_VZ, min(_MAX_VZ, vz))
                if abs(err) <= _ARRIVE_TOL_M:
                    vz = 0.0
                    arrive += 1
                else:
                    arrive = 0
                code = self._agv.height_adjust(vz)
                if code != 0:
                    with self._lock:
                        self._last_error = f"HeightAdjust 返回 {code}"
                    self._agv.stop_height()
                    break
                if arrive >= _ARRIVE_STABLE:
                    self._agv.stop_height()
                    with self._lock:
                        self._target_m = None
                    break
                time.sleep(_DT_S)
        except Exception as exc:
            with self._lock:
                self._last_error = f"{type(exc).__name__}: {exc}"
            try:
                self._agv.stop_height()
            except Exception:
                pass
        finally:
            with self._lock:
                self._busy = False
                if self._target_m is not None and self._last_error is None:
                    # 被 stop 清掉目标时保持
                    pass
            try:
                self._agv.stop_height()
            except Exception:
                pass
