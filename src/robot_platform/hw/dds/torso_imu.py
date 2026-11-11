"""只读订阅躯干 IMU（宇树 unitree_hg IMUState，G1D 上是 rt/secondary_imu，约 1 kHz）。

只取融合后的四元数：原始加速度计在 G1D 上有约 2° 零偏，四元数与雷达拟合的地面只差约 0.15°（2026-11-11 实测）。
回调里只存最新一帧，主进程多订这一路对 lowstate 间隔影响很小（p99 1.68 → 2.04 ms，同日实测）。
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Optional, Tuple

from robot_platform.hw.backends.base import TorsoImu
from robot_platform.hw.dds.sdk import init_channel


class TorsoImuReader(TorsoImu):
    def __init__(self, *, topic: str, network_interface: str, dds_domain: int = 0,
                 sdk_path: Optional[Path] = None) -> None:
        init_channel(dds_domain, network_interface, sdk_path)
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import IMUState_

        self.topic = topic
        self._lock = threading.Lock()
        self._latest: Optional[Tuple[Tuple[float, float, float, float], float]] = None
        self._sub = ChannelSubscriber(topic, IMUState_)
        self._sub.Init(self._on_msg, 10)

    def _on_msg(self, msg) -> None:
        q = msg.quaternion
        item = ((float(q[0]), float(q[1]), float(q[2]), float(q[3])), time.monotonic())
        with self._lock:
            self._latest = item

    def latest(self):
        with self._lock:
            return self._latest
