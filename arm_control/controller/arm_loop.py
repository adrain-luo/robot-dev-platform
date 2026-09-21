"""50–100 Hz hold + rate-limited set_target. Publishes rt/lowcmd only while engaged."""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from controller.gains import G1_NUM_MOTOR, KD_BODY, KP_BODY
from dds.lowstate import LowStateReader
from dds.sdk import init_channel
from model.fk_scene import RobotKinematics
from model.layout import ArmLayout, get_arm


class PermissionErrorMotion(PermissionError):
    pass


class ArmLoop:
    def __init__(
        self,
        *,
        reader: LowStateReader,
        kin: RobotKinematics,
        arm: str = "right",
        command_topic: str = "rt/lowcmd",
        max_speed_rad_s: float = 0.15,
        rate_hz: float = 100.0,
        sdk_path: Optional[Path] = None,
        dds_domain: int = 0,
        network_interface: str = "enp86s0",
    ) -> None:
        if command_topic != "rt/lowcmd":
            raise ValueError(f"Only rt/lowcmd is implemented, got {command_topic}")
        init_channel(dds_domain, network_interface, sdk_path)
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.utils.crc import CRC

        self.reader = reader
        self.kin = kin
        self.layout: ArmLayout = get_arm(arm)
        self.max_speed = float(np.clip(max_speed_rad_s, 0.02, 1.0))
        self._dt = 1.0 / float(rate_hz)
        self.limits = self.kin.arms[self.layout.name].model.joint_limits()
        self.kp_body = list(KP_BODY)
        self.kd_body = list(KD_BODY)

        self._crc = CRC()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._pub = ChannelPublisher(command_topic, LowCmd_)
        self._pub.Init()

        n = self.layout.n
        q0 = np.asarray(reader.read_arm(self.layout).q, dtype=float)
        self._lock = threading.Lock()
        self._cmd_q = q0.copy()
        self._desired_q = q0.copy()
        self._engaged = False
        self._stop = threading.Event()
        self._last_err: Optional[str] = None
        self._released_mode = False
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
            self._cmd_q = q0.copy()
            self._desired_q = q0.copy()

    def set_max_speed(self, v: float) -> None:
        self.max_speed = float(np.clip(v, 0.02, 1.0))

    def _clamp(self, q: np.ndarray) -> np.ndarray:
        lo, hi = self.limits[:, 0], self.limits[:, 1]
        return np.minimum(np.maximum(q, lo), hi)

    def _measured(self) -> np.ndarray:
        return np.asarray(self.reader.read_arm(self.layout).q, dtype=float)

    def _release_motion_mode(self) -> None:
        if self._released_mode:
            return
        try:
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
                MotionSwitcherClient,
            )
        except ImportError:
            print("[arm_loop] MotionSwitcherClient missing; skip ReleaseMode")
            self._released_mode = True
            return
        switcher = MotionSwitcherClient()
        switcher.SetTimeout(5.0)
        switcher.Init()
        _, result = switcher.CheckMode()
        while isinstance(result, dict) and result.get("name"):
            print(f"[arm_loop] releasing motion mode: {result}")
            switcher.ReleaseMode()
            time.sleep(1.0)
            _, result = switcher.CheckMode()
        self._released_mode = True

    def engage(self) -> None:
        self._release_motion_mode()
        q = self._clamp(self._measured())
        with self._lock:
            self._cmd_q = q.copy()
            self._desired_q = q.copy()
            self._engaged = True

    def release(self) -> None:
        with self._lock:
            self._engaged = False

    def estop_hold(self) -> None:
        q = self._clamp(self._measured())
        with self._lock:
            self._cmd_q = q.copy()
            self._desired_q = q.copy()

    def set_target(self, q: Sequence[float]) -> None:
        arr = np.asarray(q, dtype=float).reshape(-1)
        if arr.size != self.layout.n or not np.all(np.isfinite(arr)):
            raise ValueError(f"需要 {self.layout.n} 个有限关节角")
        with self._lock:
            if not self._engaged:
                raise PermissionErrorMotion("先接管再设目标")
            self._desired_q = self._clamp(arr)

    def status(self) -> dict:
        with self._lock:
            engaged = self._engaged
            cmd = self._cmd_q.tolist()
            desired = self._desired_q.tolist()
        return {
            "engaged": engaged,
            "arm": self.layout.name,
            "max_speed_rad_s": self.max_speed,
            "cmd_rad": cmd,
            "desired_rad": desired,
            "last_error": self._last_err,
            "joint_names": list(self.layout.joint_names),
        }

    def _slew(self) -> None:
        step = self.max_speed * self._dt
        delta = self._desired_q - self._cmd_q
        n = float(np.max(np.abs(delta)))
        if n > step and n > 1e-12:
            delta = delta * (step / n)
        self._cmd_q = self._clamp(self._cmd_q + delta)

    def _publish(self) -> None:
        msg, _ = self.reader.latest_msg()
        if msg is None:
            raise RuntimeError("no lowstate")
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = int(getattr(msg, "mode_machine", 0))
        n_state = len(msg.motor_state)
        n_cmd = len(self._cmd.motor_cmd)
        n = min(G1_NUM_MOTOR, n_state, n_cmd)
        for i in range(n):
            slot = self._cmd.motor_cmd[i]
            ms = msg.motor_state[i]
            slot.mode = 1
            slot.q = float(ms.q)
            slot.dq = 0.0
            slot.kp = float(self.kp_body[i])
            slot.kd = float(self.kd_body[i])
            slot.tau = 0.0
        with self._lock:
            q_arm = self._cmd_q.copy()
        for name, qv in zip(self.layout.joint_names, q_arm):
            idx = self.layout.motor_index(name)
            slot = self._cmd.motor_cmd[idx]
            slot.q = float(qv)
            slot.dq = 0.0
            slot.kp = float(self.kp_body[idx])
            slot.kd = float(self.kd_body[idx])
            slot.tau = 0.0
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    def _loop(self) -> None:
        next_t = time.perf_counter()
        while not self._stop.is_set():
            try:
                if self._engaged:
                    with self._lock:
                        self._slew()
                    self._publish()
                    self._last_err = None
            except Exception as e:
                self._last_err = f"{type(e).__name__}: {e}"
            next_t += self._dt
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.perf_counter()

    def shutdown(self) -> None:
        self.release()
        self._stop.set()
