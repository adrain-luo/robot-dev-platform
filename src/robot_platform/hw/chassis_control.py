"""底盘遥控：按住才走（心跳），限速，松手 / 断线 / 急停立刻停。

页面按住摇杆或方向键时每 ~100 ms 发一次 move（归一化的 x、yaw ∈ [-1, 1]），这里乘上当前限速后
由后台线程每 DT_S 重发一次 Move。超过 DEADMAN_S 没收到新的 move 就当松手，发 0 停车。
底盘停发后会不会自己停没有确认过，所以不依赖它：停车时连发几次 0，任何出错也先发 0。
"""

from __future__ import annotations

import threading
import time
from typing import Any, Dict, Optional

from robot_platform.hw.backends.base import ChassisDriver, ChassisOdom

DT_S = 0.1            # 重发 Move 的周期
DEADMAN_S = 0.5       # 这么久没收到页面的 move 就停
ZERO_REPEAT = 3       # 停车时连发几次 0
MIN_LIN_MPS = 0.02


class ChassisController:
    def __init__(self, driver: ChassisDriver, odom: Optional[ChassisOdom], spec) -> None:
        self._driver = driver
        self._odom = odom
        self._spec = spec
        self._lock = threading.Lock()
        self._x = 0.0         # 归一化指令
        self._yaw = 0.0
        self._cmd_t = 0.0
        self._lin = float(spec.default_lin_mps)
        self._last_code: Optional[int] = None
        self._last_error = ""
        self._stop_reason = ""
        self._moving = False
        self._thread: Optional[threading.Thread] = None
        self._wake = threading.Event()

    # ------------------------------------------------------------ 限速
    def yaw_limit(self, lin: Optional[float] = None) -> float:
        """转向限速跟直线限速按机型默认的比例走，不超过机型上限。"""
        s = self._spec
        lin = self._lin if lin is None else lin
        return min(float(s.max_yaw_rad_s), lin * float(s.default_yaw_rad_s) / float(s.default_lin_mps))

    def set_limit(self, lin_mps: float) -> Dict[str, Any]:
        v = float(lin_mps)
        if v != v:
            raise ValueError("限速不是数")
        with self._lock:
            self._lin = max(MIN_LIN_MPS, min(float(self._spec.max_lin_mps), v))
        return self.status()

    # ------------------------------------------------------------ 指令
    def command(self, x: float, yaw: float) -> Dict[str, Any]:
        """一次心跳：x 前后、yaw 转向，归一化到 [-1, 1]（×当前限速）。"""
        x, yaw = float(x), float(yaw)
        if x != x or yaw != yaw:
            raise ValueError("指令不是数")
        with self._lock:
            self._x = max(-1.0, min(1.0, x))
            self._yaw = max(-1.0, min(1.0, yaw))
            self._cmd_t = time.monotonic()
            self._last_error = ""
            self._stop_reason = ""
        self._ensure_thread()
        self._wake.set()
        return self.status()

    def stop(self, reason: str = "停止") -> Dict[str, Any]:
        """立刻停：清指令并在调用线程里直接发 0（不等后台线程）。"""
        with self._lock:
            self._x = self._yaw = 0.0
            self._cmd_t = 0.0
            self._stop_reason = reason
        self._wake.set()
        try:
            self._last_code = int(self._driver.stop())
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self._last_error = f"停车指令出错：{type(exc).__name__}: {exc}"
        return self.status()

    def close(self) -> None:
        self.stop("服务退出")

    # ------------------------------------------------------------ 后台
    def _ensure_thread(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._thread = threading.Thread(target=self._loop, name="chassis-teleop", daemon=True)
            self._thread.start()

    def _velocity(self):
        lin = self._lin
        return self._x * lin, self._yaw * self.yaw_limit(lin)

    def _loop(self) -> None:
        zeros = 0
        clean = False
        try:
            while True:
                with self._lock:
                    if self._cmd_t and time.monotonic() - self._cmd_t > DEADMAN_S:
                        self._x = self._yaw = 0.0
                        self._cmd_t = 0.0
                        self._stop_reason = "松手 / 心跳超时"
                    vx, wz = self._velocity()
                    active = abs(vx) > 1e-6 or abs(wz) > 1e-6
                    if not active and zeros >= ZERO_REPEAT:
                        # 在锁里退出：之后来的 command 会看到没有线程、自己起一个
                        self._moving = False
                        self._thread = None
                        clean = True
                        return
                try:
                    code = int(self._driver.move(vx, wz))
                except Exception as exc:  # noqa: BLE001
                    code = -999
                    with self._lock:
                        self._last_error = f"Move 出错：{type(exc).__name__}: {exc}"
                with self._lock:
                    self._last_code = code
                    self._moving = active
                    if code != 0 and active:
                        self._x = self._yaw = 0.0
                        self._cmd_t = 0.0
                        self._stop_reason = f"Move 返回 {code}，已停"
                        if not self._last_error:
                            self._last_error = f"Move 返回 {code}"
                zeros = 0 if active else zeros + 1
                self._wake.wait(DT_S)
                self._wake.clear()
        finally:
            if not clean:
                # 出异常退出：先补一个 0，再让下一次 command 重起线程
                try:
                    self._driver.stop()
                except Exception:  # noqa: BLE001
                    pass
                with self._lock:
                    self._moving = False
                    self._thread = None

    # ------------------------------------------------------------ 状态
    def status(self) -> Dict[str, Any]:
        with self._lock:
            vx, wz = self._velocity()
            out: Dict[str, Any] = {
                "available": True,
                "moving": self._moving,
                "cmd": {"vx": vx, "vyaw": wz},
                "limit": {"lin_mps": self._lin, "yaw_rad_s": self.yaw_limit(self._lin),
                          "max_lin_mps": float(self._spec.max_lin_mps),
                          "max_yaw_rad_s": float(self._spec.max_yaw_rad_s)},
                "last_code": self._last_code,
                "error": self._last_error,
                "stop_reason": self._stop_reason,
                "deadman_s": DEADMAN_S,
            }
        out["odom"] = None
        if self._odom is not None:
            try:
                out["odom"] = self._odom.latest()
            except Exception:  # noqa: BLE001
                pass
        return out
