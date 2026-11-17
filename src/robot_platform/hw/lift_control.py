"""立柱高度闭环：读后端立柱高度（G1D 为 hispeed.y），发 HeightAdjust(vz)。

PD 参数对齐官方 g1d_height_control.cpp。
"""

from __future__ import annotations

import threading
import time
from typing import Any, Dict, Optional, Tuple

from robot_platform.hw.backends.base import LiftDriver, LiftReader
from robot_platform.calib import lift as lift_calib

_KP = 20.0
_KD = 0.3
_MAX_VZ = 0.8
_ARRIVE_TOL_M = 0.0015
_ARRIVE_STABLE = 12
_DT_S = 0.05
_HEIGHT_TIMEOUT_S = 0.6
# HeightAdjust vz=1 时的官方满速（mock 也按它积分）
FULL_SPEED_MM_S = 76.5
# 点动（键盘长按）：多久没收到续命就停；离行程两端多近就不再往那边走
JOG_DEADMAN_S = 0.5
_JOG_END_MARGIN_M = 0.002


class LiftController:
    def __init__(
        self,
        *,
        hispeed: LiftReader,
        agv: LiftDriver,
        stroke_max_m: float,
        default_zero_m: Optional[float] = None,
    ) -> None:
        self._hispeed = hispeed
        self._agv = agv
        self._stroke_max_m = float(stroke_max_m)
        # 出厂配置里的最低读数：运动范围按它算，不跟「记为最低点」走（记高了也还能降下去重记）
        self._default_zero_m = None if default_zero_m is None else float(default_zero_m)
        self._lock = threading.Lock()
        self._target_m: Optional[float] = None
        self._busy = False
        self._last_error: Optional[str] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._jog_vz: Optional[float] = None  # 点动中：归一化速度
        self._jog_until = 0.0

    def status(self) -> Dict[str, Any]:
        st = self._hispeed.latest()
        zero = lift_calib.load_zero()
        with self._lock:
            target = self._target_m
            busy = self._busy
            err = self._last_error
        height = None if st is None else float(st.height_m)
        stroke = None if height is None else max(0.0, min(self._stroke_max_m, height - zero))
        with self._lock:
            jog = self._jog_vz
        return {
            "busy": busy,
            "jog_mm_s": None if jog is None else jog * FULL_SPEED_MM_S,
            "target_m": target,
            "target_mm": None if target is None else target * 1000.0,
            "height_m": height,
            "height_mm": None if height is None else height * 1000.0,
            "stroke_m": stroke,
            "stroke_mm": None if stroke is None else stroke * 1000.0,
            "zero_m": zero,
            "stroke_max_m": self._stroke_max_m,
            "stroke_max_mm": self._stroke_max_m * 1000.0,
            "move_lo_m": self.move_limits()[0],
            "move_hi_m": self.move_limits()[1],
            "error": err,
        }

    def current_height(self) -> Optional[float]:
        st = self._hispeed.latest()
        return None if st is None else float(st.height_m)

    def move_limits(self) -> Tuple[float, float]:
        """允许运动的高度范围 (lo, hi)：出厂最低读数往上一个行程，再放宽到包住记下的最低点。

        「记为最低点」只管三维从哪算起；记错了（比真最低高）也不会把立柱锁在那个点上。
        没有出厂值时退回按记下的最低点算。
        """
        zero = lift_calib.load_zero()
        if self._default_zero_m is None:
            return zero, zero + self._stroke_max_m
        lo = self._default_zero_m
        hi = self._default_zero_m + self._stroke_max_m
        return min(lo, zero), max(hi, zero)

    def clamp_height(self, height_m: float) -> float:
        lo, hi = self.move_limits()
        return max(lo, min(hi, float(height_m)))

    def goto(self, height_m: float) -> Dict[str, Any]:
        target = self.clamp_height(height_m)
        with self._lock:
            self._target_m = target
            self._jog_vz = None
            self._last_error = None
        self._ensure_thread()
        return {"ok": True, **self.status()}

    def jog(self, speed_mm_s: float, hold_s: float = JOG_DEADMAN_S) -> Dict[str, Any]:
        """按速度升降（正为升），hold_s 内没有再调就自动停；到行程两端自动停。"""
        if self.current_height() is None:
            return {"ok": False, "error": "还没有立柱高度反馈", **self.status()}
        vz = max(-1.0, min(1.0, float(speed_mm_s) / FULL_SPEED_MM_S))
        with self._lock:
            self._target_m = None
            self._jog_vz = vz
            self._jog_until = time.monotonic() + max(0.05, min(2.0, float(hold_s)))
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
            self._jog_vz = None
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
        t = self._thread
        if t is not None and t.is_alive() and self._stop.is_set():
            # 刚 stop 过、旧线程还在退出：等它退完再起新的，免得新目标被正在退出的线程吞掉
            t.join(timeout=0.5)
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
                    jog = self._jog_vz
                    jog_until = self._jog_until
                if target is None and jog is None:
                    break
                st = self._hispeed.latest()
                age = self._hispeed.age_s()
                if st is None or age > _HEIGHT_TIMEOUT_S:
                    with self._lock:
                        self._last_error = "立柱高度超时"
                        self._jog_vz = None
                    self._agv.stop_height()
                    break
                height = float(st.height_m)
                if jog is not None:
                    lo, hi = self.move_limits()
                    at_top = jog > 0 and height >= hi - _JOG_END_MARGIN_M
                    at_bottom = jog < 0 and height <= lo + _JOG_END_MARGIN_M
                    if time.monotonic() > jog_until or at_top or at_bottom:
                        # 没续命（松键 / 断线）或到头：停
                        with self._lock:
                            if at_top or at_bottom or self._jog_until == jog_until:  # 到头，或期间没人续命
                                self._jog_vz = None
                        self._agv.stop_height()
                        break
                    code = self._agv.height_adjust(jog)
                    if code != 0:
                        with self._lock:
                            self._last_error = f"HeightAdjust 返回 {code}"
                            self._jog_vz = None
                        self._agv.stop_height()
                        break
                    prev_err = None
                    time.sleep(_DT_S)
                    continue
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
                # 线程正要退出时又来了新目标 / 点动续命（没被 stop、没出错）：接着跑
                again = (not self._stop.is_set() and self._last_error is None
                         and (self._target_m is not None or self._jog_vz is not None))
            if again:
                self._thread = threading.Thread(target=self._loop, name="lift-pd", daemon=True)
                self._thread.start()
                return
            try:
                self._agv.stop_height()
            except Exception:
                pass
