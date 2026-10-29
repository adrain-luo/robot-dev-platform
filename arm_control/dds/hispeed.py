"""Read-only rt/hispeed_state subscriber (G1D 立柱高度).

官方文档：Point32_.y → 实时高度，Point32_.z → 实时速度。
见 https://support.unitree.com/home/zh/G1D_Developer/move
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from dds.sdk import init_channel


@dataclass(frozen=True)
class LiftState:
    timestamp_monotonic: float
    x: float
    height_m: float
    velocity_mps: float


class HispeedStateReader:
    """Subscribe to rt/hispeed_state only. Does not publish commands."""

    def __init__(
        self,
        *,
        network_interface: str = "enp86s0",
        state_topic: str = "rt/hispeed_state",
        dds_domain: int = 0,
        timeout_s: float = 2.0,
        sdk_path: Optional[Path] = None,
        wait: bool = False,
    ) -> None:
        init_channel(dds_domain, network_interface, sdk_path)
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_

        self._timeout_s = float(timeout_s)
        self._lock = threading.Lock()
        self._msg = None
        self._last_t = 0.0
        self.state_topic = state_topic
        self.network_interface = network_interface

        self._sub = ChannelSubscriber(state_topic, Point32_)
        self._sub.Init(self._on_msg, 10)
        if wait:
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
            f"(iface={self.network_interface!r})."
        )

    def age_s(self) -> float:
        with self._lock:
            if self._last_t <= 0:
                return float("inf")
            return time.monotonic() - self._last_t

    def latest(self) -> Optional[LiftState]:
        with self._lock:
            msg = self._msg
            ts = self._last_t
        if msg is None:
            return None
        return LiftState(
            timestamp_monotonic=ts,
            x=float(msg.x),
            height_m=float(msg.y),
            velocity_mps=float(msg.z),
        )
