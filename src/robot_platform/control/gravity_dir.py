"""重力前馈用的重力方向（base_link 系）。

不开 IMU：恒为正下方（假设躯干竖直，以前一直这样）。
开 IMU：按躯干 IMU 的四元数算重力在躯干系里的方向，不依赖腰关节读数（G1D 腰俯仰电机坏了也不影响），
底盘停在坡上也一并算进去。

安全：
- 每拍按时间常数 TAU_S 平滑逼近目标方向，开 / 关、IMU 跳变都不会让前馈力矩突变；
- IMU 断流（> STALE_S）或倾角超过机型的 max_tilt_deg：保持上一次的方向不动，并在 error 里说明。
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import numpy as np

from robot_platform.control.arm_gravity import G_ACCEL
from robot_platform.hw.backends.base import TorsoImu

TAU_S = 0.5
STALE_S = 0.1
DOWN = np.array([0.0, 0.0, -1.0])


def quat_to_R(q) -> np.ndarray:
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if not n > 1e-6:
        raise ValueError("四元数为零")
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rpy_to_R(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def down_in_base(quat, R_mount: np.ndarray) -> np.ndarray:
    """IMU 四元数（IMU 系 → 世界系）→ 重力方向在 base_link 系里的单位向量。R_mount：IMU 系 → base_link 系。"""
    R = quat_to_R(quat)
    return R_mount @ (R.T @ DOWN)


def tilt_angles(down: np.ndarray) -> dict:
    """躯干相对重力的倾角，度。pitch：绕 y，上端往前（+x）倒为正，即躯干系 R = Ry(pitch)；
    roll：绕 x，R = Rx(roll)（上端往右 -y 倒为正）；tilt：总倾角。"""
    up = -np.asarray(down, float)
    return {
        "pitch_deg": math.degrees(math.atan2(-up[0], up[2])),
        "roll_deg": math.degrees(math.atan2(up[1], up[2])),
        "tilt_deg": math.degrees(math.acos(float(np.clip(up[2] / max(np.linalg.norm(up), 1e-9), -1, 1)))),
    }


class GravityDirection:
    def __init__(self, imu: Optional[TorsoImu], spec=None) -> None:
        self._imu = imu if spec is not None else None
        self._spec = spec
        self._R_mount = rpy_to_R(*spec.mount_rpy) if spec is not None else np.eye(3)
        self._max_tilt = float(spec.max_tilt_deg) if spec is not None else 0.0
        self._lock = threading.Lock()
        self._enabled = bool(spec.gravity_default) if self._imu is not None else False
        self._dir = DOWN.copy()
        self._imu_dir: Optional[np.ndarray] = None
        self._imu_age = float("inf")
        self._err = ""

    @property
    def available(self) -> bool:
        return self._imu is not None

    def set_enabled(self, on: bool) -> None:
        if on and self._imu is None:
            raise ValueError("这个机型没有声明躯干 IMU（robot.json 的 torso_imu）")
        with self._lock:
            self._enabled = bool(on)

    def _read_imu(self) -> Optional[np.ndarray]:
        got = self._imu.latest() if self._imu is not None else None
        if got is None:
            self._imu_age = float("inf")
            self._err = "躯干 IMU 还没有数据"
            return None
        quat, t = got
        self._imu_age = time.monotonic() - t
        try:
            d = down_in_base(quat, self._R_mount)
        except ValueError as e:
            self._err = f"躯干 IMU：{e}"
            return None
        self._imu_dir = d
        if self._imu_age > STALE_S:
            self._err = f"躯干 IMU {self._imu_age * 1000:.0f} ms 没更新，重力方向保持不动"
            return None
        if tilt_angles(d)["tilt_deg"] > self._max_tilt:
            self._err = f"躯干 IMU 倾角 {tilt_angles(d)['tilt_deg']:.1f}° 超过 {self._max_tilt:.0f}°，重力方向保持不动"
            return None
        self._err = ""
        return d

    def update(self, dt: float) -> None:
        """控制循环每拍调一次（接不接管都调，方向一直是最新的）。"""
        with self._lock:
            enabled = self._enabled
            meas = self._read_imu() if self._imu is not None else None
            if not enabled:
                target = DOWN
            elif meas is None:
                return  # 读数不可信：保持
            else:
                target = meas
            a = min(1.0, max(0.0, float(dt)) / TAU_S)
            d = self._dir + a * (target - self._dir)
            self._dir = d / np.linalg.norm(d)

    def up_dir(self) -> np.ndarray:
        """实测「竖直向上」在 base_link 系里的单位向量（不管前馈开没开）；IMU 不可用 / 过期 / 倾角超限时
        退回躯干 +z。供「可平转」工作姿态取绕重力的转轴。"""
        with self._lock:
            d = self._imu_dir
            fresh = (d is not None and self._imu_age <= STALE_S
                     and tilt_angles(d)["tilt_deg"] <= self._max_tilt)
            return (-d / np.linalg.norm(d)) if fresh else -DOWN.copy()

    def g_vec(self) -> np.ndarray:
        """当前用于前馈的重力向量（m/s²，base_link 系）。"""
        with self._lock:
            return G_ACCEL * self._dir

    def status(self) -> dict:
        with self._lock:
            used = tilt_angles(self._dir)
            imu = tilt_angles(self._imu_dir) if self._imu_dir is not None else None
            return {
                "available": self._imu is not None,
                "enabled": self._enabled,
                "topic": self._spec.topic if self._spec is not None else "",
                "used": {k: round(v, 2) for k, v in used.items()},
                "imu": None if imu is None else {k: round(v, 2) for k, v in imu.items()},
                "imu_age_ms": None if not math.isfinite(self._imu_age) else round(self._imu_age * 1000, 1),
                "error": self._err if self._enabled else "",
            }
