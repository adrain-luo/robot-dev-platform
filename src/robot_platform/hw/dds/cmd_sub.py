"""只读订阅指令话题（rt/lowcmd 或 rt/arm_sdk），把每条消息交给 CmdMonitor 判定是谁发的。"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Sequence

from robot_platform.hw.cmd_monitor import CmdMonitor
from robot_platform.hw.dds.sdk import init_channel


class CmdSubscriber:
    def __init__(self, monitor: CmdMonitor, *, watch_motors: Sequence[int], weight_motor: Optional[int],
                 dds_domain: int, network_interface: str, sdk_path: Optional[Path]) -> None:
        init_channel(dds_domain, network_interface, sdk_path)
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

        self.monitor = monitor
        self._watch = tuple(int(i) for i in watch_motors)
        self._weight = None if weight_motor is None else int(weight_motor)
        self._sub = ChannelSubscriber(monitor.topic, LowCmd_)
        self._sub.Init(self._on_msg, 10)

    def _on_msg(self, msg) -> None:
        try:
            mc = msg.motor_cmd
            kp = max((float(mc[i].kp) for i in self._watch), default=None)
            w = None if self._weight is None else float(mc[self._weight].q)
            self.monitor.on_message(int(msg.crc), weight=w, kp_max=kp)
        except Exception:  # noqa: BLE001  回调里不能抛
            pass

    def close(self) -> None:
        try:
            self._sub.Close()
        except Exception:  # noqa: BLE001
            pass
