"""Read-only rt/lowstate subscriber. Never creates a command publisher."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Sequence, Union

from model.layout import ArmLayout, get_arm
from dds.sdk import init_channel


@dataclass(frozen=True)
class ArmState:
    timestamp_monotonic: float
    mode_machine: Optional[int]
    q: tuple  # rad, length = arm.n
    dq: tuple
    tau_est: tuple  # may contain None


class LowStateReader:
    """Subscribe to rt/lowstate only (safe alongside publishers from other processes)."""

    def __init__(
        self,
        *,
        network_interface: str = "enp86s0",
        state_topic: str = "rt/lowstate",
        dds_domain: int = 0,
        timeout_s: float = 5.0,
        sdk_path: Optional[Path] = None,
    ) -> None:
        init_channel(dds_domain, network_interface, sdk_path)
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        self._timeout_s = float(timeout_s)
        self._lock = threading.Lock()
        self._msg = None
        self._last_t = 0.0
        self.state_topic = state_topic
        self.network_interface = network_interface

        self._sub = ChannelSubscriber(state_topic, LowState_)
        self._sub.Init(self._on_msg, 10)
        self.wait_for_message()

    def _on_msg(self, msg) -> None:
        with self._lock:
            self._msg = msg
            self._last_t = time.monotonic()

    def wait_for_message(self, timeout_s: Optional[float] = None) -> None:
        deadline = time.monotonic() + float(
            timeout_s if timeout_s is not None else self._timeout_s
        )
        while time.monotonic() < deadline:
            with self._lock:
                if self._msg is not None:
                    return
            time.sleep(0.02)
        raise TimeoutError(
            f"No message on {self.state_topic} within timeout "
            f"(iface={self.network_interface!r}). Check DDS/robot power/peer."
        )

    def latest_msg(self):
        with self._lock:
            return self._msg, self._last_t

    def age_s(self) -> float:
        with self._lock:
            if self._last_t <= 0:
                return float("inf")
            return time.monotonic() - self._last_t

    def read_arm(self, arm: Union[str, ArmLayout], *, wait: bool = True) -> ArmState:
        layout = arm if isinstance(arm, ArmLayout) else get_arm(arm)
        if wait:
            self.wait_for_message(timeout_s=self._timeout_s)
        with self._lock:
            msg = self._msg
            ts = self._last_t
        if msg is None:
            raise RuntimeError("lowstate message missing")
        q, dq, tau = [], [], []
        for idx in layout.motor_indices:
            ms = msg.motor_state[idx]
            q.append(float(ms.q))
            dq.append(float(ms.dq))
            t = getattr(ms, "tau_est", None)
            tau.append(None if t is None else float(t))
        mode = getattr(msg, "mode_machine", None)
        return ArmState(
            timestamp_monotonic=ts,
            mode_machine=None if mode is None else int(mode),
            q=tuple(q),
            dq=tuple(dq),
            tau_est=tuple(tau),
        )

    def read_motors(self, motor_indices: Sequence[int]) -> Dict[int, float]:
        self.wait_for_message(timeout_s=self._timeout_s)
        with self._lock:
            msg = self._msg
        if msg is None:
            raise RuntimeError("lowstate message missing after wait")
        return {int(i): float(msg.motor_state[int(i)].q) for i in motor_indices}
