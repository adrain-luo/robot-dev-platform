"""只读订阅 G1D 底盘里程计 rt/agv/odom（nav_msgs Odometry，约 18 Hz）。"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Optional

from robot_platform.hw.backends.base import ChassisOdom
from robot_platform.hw.dds.sdk import init_channel


class AgvOdomReader(ChassisOdom):
    def __init__(self, *, topic: str, network_interface: str, dds_domain: int = 0,
                 sdk_path: Optional[Path] = None) -> None:
        init_channel(dds_domain, network_interface, sdk_path)
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_

        self.topic = topic
        self._lock = threading.Lock()
        self._msg = None
        self._t = 0.0
        self._sub = ChannelSubscriber(topic, Odometry_)
        self._sub.Init(self._on_msg, 10)

    def _on_msg(self, msg) -> None:
        with self._lock:
            self._msg = msg
            self._t = time.monotonic()

    def latest(self) -> Optional[dict]:
        with self._lock:
            msg, t = self._msg, self._t
        if msg is None:
            return None
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        tw = msg.twist.twist
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return {"x": float(p.x), "y": float(p.y), "yaw": yaw, "vx": float(tw.linear.x),
                "vyaw": float(tw.angular.z), "age_s": time.monotonic() - t}
