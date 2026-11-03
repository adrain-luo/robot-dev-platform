"""unitree_lowcmd：退出高层运动后，由本程序发全身 rt/lowcmd（G1D）。

状态订阅 rt/lowstate；立柱读 rt/hispeed_state、经 AgvClient.HeightAdjust 升降。
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Optional, Sequence

from backends.base import Backend, Commander, LiftDriver, LiftReader, StateSource, check_idle_motors
from dds.sdk import init_channel
from model.robot_profile import RobotProfile


class LowCmdCommander(Commander):
    def __init__(self, *, reader: StateSource, idle_motors: Sequence[int], command_topic: str, release_motion_mode: bool,
                 dds_domain: int, network_interface: str, sdk_path: Optional[Path]) -> None:
        if command_topic != "rt/lowcmd":
            raise ValueError(f"Only rt/lowcmd is implemented, got {command_topic}")
        init_channel(dds_domain, network_interface, sdk_path)
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.utils.crc import CRC

        self._reader = reader
        self._idle_motors = tuple(idle_motors)
        self._release = bool(release_motion_mode)
        self._released_mode = False
        self._crc = CRC()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._pub = ChannelPublisher(command_topic, LowCmd_)
        self._pub.Init()

    def prepare(self) -> None:
        # 每次接管都查：机器人可能在两次接管之间换过状态
        check_idle_motors(self._reader, self._idle_motors)
        if self._released_mode:
            return
        if not self._release:
            self._released_mode = True
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

    def write(self, q: Sequence[float], kp: Sequence[float], kd: Sequence[float], tau: Sequence[float],
              *, mode_machine: Optional[int] = None) -> None:
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = int(mode_machine or 0)
        n = min(len(q), len(self._cmd.motor_cmd))
        for i in range(n):
            slot = self._cmd.motor_cmd[i]
            slot.mode = 1
            slot.q = float(q[i])
            slot.dq = 0.0
            slot.kp = float(kp[i])
            slot.kd = float(kd[i])
            slot.tau = float(tau[i])
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)


class UnitreeLowCmdBackend(Backend):
    name = "unitree_lowcmd"

    def __init__(self, profile: RobotProfile, *, network_interface: str, dds_domain: int = 0,
                 sdk_path: Optional[Path] = None, timeout_s: float = 5.0) -> None:
        self.profile = profile
        self._iface = network_interface
        self._domain = dds_domain
        self._sdk = sdk_path
        self._timeout_s = timeout_s
        self._lock = threading.Lock()
        self._state = None
        self._commander = None
        self._lift_reader = None
        self._lift_driver = None

    def state(self) -> StateSource:
        with self._lock:
            if self._state is None:
                from dds.lowstate import LowStateReader

                self._state = LowStateReader(
                    network_interface=self._iface,
                    state_topic=self.profile.control.state_topic,
                    dds_domain=self._domain,
                    timeout_s=self._timeout_s,
                    sdk_path=self._sdk,
                )
            return self._state

    def commander(self) -> Commander:
        reader = self.state()
        with self._lock:
            if self._commander is None:
                self._commander = LowCmdCommander(
                    reader=reader,
                    idle_motors=self.profile.control.idle_motors,
                    command_topic=self.profile.control.command_topic,
                    release_motion_mode=self.profile.control.release_motion_mode,
                    dds_domain=self._domain,
                    network_interface=self._iface,
                    sdk_path=self._sdk,
                )
            return self._commander

    def lift_reader(self) -> LiftReader:
        if self.profile.lift is None:
            return super().lift_reader()
        # 先起 lowstate（ChannelFactory）
        self.state()
        with self._lock:
            if self._lift_reader is None:
                from dds.hispeed import HispeedStateReader

                self._lift_reader = HispeedStateReader(
                    network_interface=self._iface,
                    state_topic=self.profile.lift.state_topic,
                    dds_domain=self._domain,
                    timeout_s=self._timeout_s,
                    sdk_path=self._sdk,
                    wait=False,
                )
            return self._lift_reader

    def lift_driver(self) -> LiftDriver:
        if self.profile.lift is None:
            return super().lift_driver()
        self.state()
        with self._lock:
            if self._lift_driver is None:
                from dds.agv_client import AgvClient

                self._lift_driver = AgvClient(
                    network_interface=self._iface,
                    dds_domain=self._domain,
                    sdk_path=self._sdk,
                    timeout_s=3.0,
                )
            return self._lift_driver
