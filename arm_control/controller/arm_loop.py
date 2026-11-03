"""50–100 Hz hold + rate-limited set_target. 接管后经后端 Commander 周期发指令。"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

# PermissionErrorMotion 定义在后端接口里（后端安全检查也抛它），其它模块仍从这里导入
from backends.base import Commander, PermissionErrorMotion, StateSource  # noqa: F401
from model.arm_gravity import ArmGravity
from model.fk_scene import RobotKinematics
from model.layout import ArmLayout, get_arm
from model.robot_profile import active as active_profile

TAU_FF_LIMIT = 8.0
TEACH_KD = 2.0


def _clamp_span(value: float, span: tuple) -> float:
    lo, hi = span
    if not np.isfinite(value):
        raise ValueError("腰部角度必须是有限数")
    return float(min(hi, max(lo, value)))


class ArmLoop:
    def __init__(
        self,
        *,
        reader: StateSource,
        commander: Commander,
        kin: RobotKinematics,
        arm: str = "right",
        max_speed_rad_s: float = 0.2,
        rate_hz: float = 100.0,
        urdf_path: Optional[Path] = None,
        base_link: str = "torso_link",
    ) -> None:
        self.reader = reader
        self._commander = commander
        self.kin = kin
        self.profile = active_profile()
        self.layout: ArmLayout = get_arm(arm)
        self.max_speed = float(np.clip(max_speed_rad_s, 0.02, 1.0))
        self._dt = 1.0 / float(rate_hz)
        self.limits = self.kin.arms[self.layout.name].model.joint_limits()
        self.kp_body = list(self.profile.kp)
        self.kd_body = list(self.profile.kd)
        self._waist = self.profile.waist
        self._waist_motors = self._waist.hold_motors if self._waist else ()
        urdf = Path(urdf_path) if urdf_path is not None else self.kin.urdf_path
        self._grav = ArmGravity(self.layout.name, urdf, base_link)
        self._grav_on = True
        self._float = False
        self._waist_cmd: Optional[dict] = None
        self._waist_target: Optional[dict] = None

        n = self.layout.n
        q0 = np.asarray(reader.read_arm(self.layout).q, dtype=float)
        self._lock = threading.Lock()
        self._cmd_q = q0.copy()
        self._desired_q = q0.copy()
        self._engaged = False
        self._stop = threading.Event()
        self._last_err: Optional[str] = None
        self._payload_theta: Optional[np.ndarray] = None
        self._payload_name: Optional[str] = None
        self._last_tau_ff = np.zeros(n)
        self._thread = threading.Thread(target=self._loop, name="arm-loop", daemon=True)
        self._thread.start()

    @property
    def arm(self) -> str:
        return self.layout.name

    def set_arm(self, name: str) -> None:
        if self._engaged:
            raise RuntimeError("释放后再切换控制臂")
        layout = get_arm(name)
        q0 = np.asarray(self.reader.read_arm(layout).q, dtype=float)
        with self._lock:
            self.layout = layout
            self.limits = self.kin.arms[layout.name].model.joint_limits()
            self._grav = ArmGravity(layout.name, self._grav.model.urdf_path, self._grav.base_link)
            self._float = False
            self._payload_theta = None
            self._payload_name = None
            self._cmd_q = q0.copy()
            self._desired_q = q0.copy()

    def set_max_speed(self, v: float) -> None:
        self.max_speed = float(np.clip(v, 0.02, 1.0))

    def _clamp(self, q: np.ndarray) -> np.ndarray:
        lo, hi = self.limits[:, 0], self.limits[:, 1]
        return np.minimum(np.maximum(q, lo), hi)

    def _measured(self) -> np.ndarray:
        return np.asarray(self.reader.read_arm(self.layout).q, dtype=float)

    def _read_waist(self) -> dict:
        q = self.reader.snapshot().q
        return {idx: float(q[idx]) for idx in self._waist_motors}

    def _slew_waist(self) -> None:
        if not self._waist_cmd or not self._waist_target:
            return
        step = self.max_speed * self._dt
        for idx in self._waist_motors:
            cur = float(self._waist_cmd[idx])
            tgt = float(self._waist_target[idx])
            d = tgt - cur
            if abs(d) <= step:
                self._waist_cmd[idx] = tgt
            else:
                self._waist_cmd[idx] = cur + (step if d > 0 else -step)

    def engage(self) -> None:
        if self._commander.busy() and not self._engaged:
            raise PermissionErrorMotion("上次释放还在过渡，稍等再接管")
        self._commander.prepare()
        q = self._clamp(self._measured())
        waist_now = self._read_waist()
        with self._lock:
            self._cmd_q = q.copy()
            self._desired_q = q.copy()
            self._float = False
            self._waist_cmd = waist_now
            self._waist_target = dict(waist_now)
            self._engaged = True

    def release(self) -> None:
        # 拖拽中释放：先把指令角对齐实测，过渡期间按它保持，避免跳回拖拽前的旧指令
        q = self._clamp(self._measured()) if self._float else None
        with self._lock:
            if q is not None:
                self._cmd_q = q.copy()
                self._desired_q = q.copy()
            self._float = False
            self._engaged = False
        self._commander.release()
        # 需要过渡的后端（arm_sdk）在过渡期间继续按最后的腰 / 臂角保持，结束后再清
        if not self._commander.busy():
            self._clear_waist()

    def _clear_waist(self) -> None:
        with self._lock:
            self._waist_cmd = None
            self._waist_target = None

    def _require_ready(self, what: str) -> None:
        if not self._commander.ready():
            raise PermissionErrorMotion(f"接管过渡中（{self._commander.phase()}），等完成再{what}")

    def estop_hold(self) -> None:
        q = self._clamp(self._measured())
        waist_now = self._read_waist()
        with self._lock:
            self._float = False
            self._cmd_q = q.copy()
            self._desired_q = q.copy()
            if self._waist_cmd is not None:
                self._waist_cmd = waist_now
                self._waist_target = dict(waist_now)

    def set_waist(self, yaw_rad: Optional[float] = None, pitch_rad: Optional[float] = None) -> None:
        if self._waist is None:
            raise PermissionErrorMotion(f"{self.profile.name} 没有可控的腰")
        if not self._waist.movable:
            raise PermissionErrorMotion(f"{self.profile.name} 的腰只保持不转（站立时转腰会影响平衡）")
        self._require_ready("动腰")
        with self._lock:
            if not self._engaged or self._waist_target is None:
                raise PermissionErrorMotion("先接管再动腰")
            if self._float:
                raise PermissionErrorMotion("先急停退出拖拽，再动腰")
            for role, value in (("yaw", yaw_rad), ("pitch", pitch_rad)):
                if value is None:
                    continue
                j = self._waist.joint(role)
                self._waist_target[j.motor] = _clamp_span(float(value), j.limit_rad)

    def hold_waist(self) -> None:
        """停在当前实测角（取消未走完的腰目标）。"""
        waist_now = self._read_waist()
        with self._lock:
            if not self._engaged or self._waist_target is None:
                raise PermissionErrorMotion("先接管再停腰")
            self._waist_cmd = waist_now
            self._waist_target = dict(waist_now)

    def enable_teach(self) -> None:
        self._require_ready("拖拽")
        with self._lock:
            if not self._engaged:
                raise PermissionErrorMotion("先接管再拖拽")
            self._float = True

    def disable_teach(self) -> None:
        q = self._clamp(self._measured())
        with self._lock:
            self._float = False
            self._cmd_q = q.copy()
            self._desired_q = q.copy()

    def set_gravity_ff(self, on: bool) -> None:
        self._grav_on = bool(on)

    def arm_gains(self) -> tuple:
        idx = self.layout.motor_indices
        return [float(self.kp_body[i]) for i in idx], [float(self.kd_body[i]) for i in idx]

    def set_payload_theta(self, theta, name: Optional[str] = None) -> None:
        with self._lock:
            if theta is None:
                self._payload_theta = None
                self._payload_name = None
            else:
                self._payload_theta = np.asarray(theta, float).reshape(4)
                self._payload_name = name

    def payload_theta(self) -> Optional[list]:
        if self._payload_theta is None:
            return None
        return self._payload_theta.tolist()

    def arm_sample(self, wait: bool = False) -> dict:
        st = self.reader.read_arm(self.layout, wait=wait)
        kp, kd = self.arm_gains()
        with self._lock:
            cmd = self._cmd_q.copy()
            tau_ff = self._last_tau_ff.copy()
        tau_est = [None if t is None else float(t) for t in st.tau_est]
        return {
            "q_meas": [float(v) for v in st.q],
            "dq": [float(v) for v in st.dq],
            "tau_est": tau_est,
            "q_cmd": cmd.tolist(),
            "tau_ff_sent": tau_ff.tolist(),
            "kp": kp,
            "kd": kd,
        }

    def set_target(self, q: Sequence[float]) -> None:
        arr = np.asarray(q, dtype=float).reshape(-1)
        if arr.size != self.layout.n or not np.all(np.isfinite(arr)):
            raise ValueError(f"需要 {self.layout.n} 个有限关节角")
        if self._engaged:
            self._require_ready("走点")
        with self._lock:
            if not self._engaged:
                raise PermissionErrorMotion("先接管再设目标")
            if self._float:
                raise PermissionErrorMotion("先急停退出拖拽，再走点")
            self._desired_q = self._clamp(arr)

    def status(self) -> dict:
        with self._lock:
            engaged = self._engaged
            floating = self._float
            grav_on = self._grav_on
            waist_cmd = dict(self._waist_cmd) if self._waist_cmd else None
            waist_target = {str(i): float(v) for i, v in self._waist_target.items()} if self._waist_target else None
            cmd = self._cmd_q.tolist()
            desired = self._desired_q.tolist()
            theta = None if self._payload_theta is None else self._payload_theta.tolist()
            pname = self._payload_name
        return {
            "engaged": engaged,
            "float": floating,
            "gravity_ff": grav_on,
            "gravity_mass_kg": self._grav.moving_mass_kg,
            "payload_theta": theta,
            "payload_name": pname,
            "kp_vec": [float(self.kp_body[i]) for i in self.layout.motor_indices],
            "waist_locked": waist_cmd is not None,
            "waist_hold_rad": waist_cmd,
            "waist_target_rad": waist_target,
            "arm": self.layout.name,
            "max_speed_rad_s": self.max_speed,
            "cmd_rad": cmd,
            "desired_rad": desired,
            "last_error": self._last_err,
            "joint_names": list(self.layout.joint_names),
            # 接管 / 释放过渡（arm_sdk 后端有，其它为空）
            "ready": engaged and self._commander.ready(),
            "transition": self._commander.phase(),
        }

    def _slew(self) -> None:
        step = self.max_speed * self._dt
        delta = self._desired_q - self._cmd_q
        n = float(np.max(np.abs(delta)))
        if n > step and n > 1e-12:
            delta = delta * (step / n)
        self._cmd_q = self._clamp(self._cmd_q + delta)

    def _publish(self) -> None:
        snap = self.reader.snapshot()
        st = self.reader.read_arm(self.layout)
        q_meas = np.asarray(st.q, dtype=float)
        with self._lock:
            floating = self._float
            q_arm = q_meas.copy() if self._float else self._cmd_q.copy()
            grav_on = self._grav_on
            waist_hold = dict(self._waist_cmd) if self._waist_cmd else None
        tau_arm = np.zeros(self.layout.n)
        if grav_on:
            tau_arm = self._grav.tau_arm(q_meas)
            th = None
            with self._lock:
                th = None if self._payload_theta is None else self._payload_theta.copy()
            if th is not None:
                tau_arm = tau_arm + self._grav.tau_payload(q_meas, th)
            tau_arm = np.clip(tau_arm, -TAU_FF_LIMIT, TAU_FF_LIMIT)
        self._last_tau_ff = np.asarray(tau_arm, float)

        # 未接管的电机：保持实测角 + 默认 PD；腰按保持角锁住；当前臂按指令角
        n = min(self.profile.num_motors, len(snap.q))
        q = list(snap.q[:n])
        kp = list(self.kp_body[:n])
        kd = list(self.kd_body[:n])
        tau = [0.0] * n
        if waist_hold:
            for idx, q_w in waist_hold.items():
                i = int(idx)
                q[i] = float(q_w)
                kp[i] = float(self._waist.lock_kp)
                kd[i] = float(self._waist.lock_kd)
        for name, qv, t in zip(self.layout.joint_names, q_arm, tau_arm):
            i = self.layout.motor_index(name)
            q[i] = float(qv)
            tau[i] = float(t)
            if floating:
                kp[i] = 0.0
                kd[i] = TEACH_KD
        self._commander.write(q, kp, kd, tau, mode_machine=snap.mode_machine)

    def _loop(self) -> None:
        next_t = time.perf_counter()
        while not self._stop.is_set():
            try:
                engaged = self._engaged
                # 释放过渡中（arm_sdk）仍要发帧，按最后的指令角保持
                if engaged or self._commander.busy():
                    if engaged:
                        with self._lock:
                            if not self._float:
                                self._slew()
                            self._slew_waist()
                    self._publish()
                    self._last_err = None
                elif self._waist_cmd is not None:
                    self._clear_waist()
            except Exception as e:
                self._last_err = f"{type(e).__name__}: {e}"
            next_t += self._dt
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.perf_counter()

    def shutdown(self, timeout_s: float = 30.0) -> None:
        """释放并等过渡结束再停线程（arm_sdk 要先把权重降到 0）。"""
        self.release()
        deadline = time.monotonic() + timeout_s
        while self._commander.busy() and time.monotonic() < deadline:
            time.sleep(0.05)
        self._stop.set()
