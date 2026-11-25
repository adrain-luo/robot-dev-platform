"""unitree_lowcmd：退出高层运动后，由本程序发全身 rt/lowcmd（G1D）。

状态订阅 rt/lowstate；立柱读 rt/hispeed_state、经 AgvClient.HeightAdjust 升降。
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Optional, Sequence

from robot_platform.hw.backends.base import (
    Backend, Commander, LiftDriver, LiftReader, StateSource, check_idle_motors, profile_lowstate_max_age_s,
)
from robot_platform.hw.cmd_monitor import CmdMonitor, hints_for
from robot_platform.hw.dds.sdk import init_channel
from robot_platform.core.robot_profile import RobotProfile


class LowCmdCommander(Commander):
    def __init__(self, *, reader: StateSource, monitor: CmdMonitor, idle_motors: Sequence[int], command_topic: str, release_motion_mode: bool,
                 dds_domain: int, network_interface: str, sdk_path: Optional[Path]) -> None:
        if command_topic != "rt/lowcmd":
            raise ValueError(f"Only rt/lowcmd is implemented, got {command_topic}")
        init_channel(dds_domain, network_interface, sdk_path)
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.utils.crc import CRC

        self._reader = reader
        self._monitor = monitor
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
        if self._released_mode or not self._release:
            # 运控已退出（或本机型不需要退出）：此时 rt/lowcmd 上不该有任何别人
            self._monitor.check_free()
            self._released_mode = True
            return
        try:
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import (
                MotionSwitcherClient,
            )
        except ImportError:
            print("[arm_loop] MotionSwitcherClient missing; skip ReleaseMode")
            self._monitor.check_free()
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
        # 运控在时它自己可能也发 rt/lowcmd，分不清；退出之后重新听，还有人发就是别的程序
        self._monitor.check_free(fresh=True)

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
        self._monitor.note_own(self._cmd.crc)
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
        self._monitor = None
        self._cmd_sub = None
        self._lift_reader = None
        self._lift_driver = None

    def state(self) -> StateSource:
        with self._lock:
            if self._state is None:
                from robot_platform.hw.dds.lowstate import LowStateReader

                self._state = LowStateReader(
                    network_interface=self._iface,
                    state_topic=self.profile.control.state_topic,
                    dds_domain=self._domain,
                    timeout_s=self._timeout_s,
                    sdk_path=self._sdk,
                )
            return self._state

    def lowstate_max_age_s(self) -> float:
        # 本程序直接写 rt/lowcmd：lowstate 过期就由 ArmLoop 停在指令角保持
        return profile_lowstate_max_age_s(self.profile)

    def cmd_monitor(self) -> CmdMonitor:
        self.state()
        with self._lock:
            if self._monitor is None:
                from robot_platform.hw.dds.cmd_sub import CmdSubscriber

                arm_motors = [i for a in self.profile.arms.values() for i in a.motor_indices]
                self._monitor = CmdMonitor(self.profile.control.command_topic, hints=hints_for(self.profile))
                self._cmd_sub = CmdSubscriber(
                    self._monitor, watch_motors=arm_motors, weight_motor=None,
                    dds_domain=self._domain, network_interface=self._iface, sdk_path=self._sdk)
            return self._monitor

    def commander(self) -> Commander:
        reader = self.state()
        monitor = self.cmd_monitor()
        with self._lock:
            if self._commander is None:
                self._commander = LowCmdCommander(
                    reader=reader,
                    monitor=monitor,
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
                from robot_platform.hw.dds.hispeed import HispeedStateReader

                self._lift_reader = HispeedStateReader(
                    network_interface=self._iface,
                    state_topic=self.profile.lift.state_topic,
                    dds_domain=self._domain,
                    timeout_s=self._timeout_s,
                    sdk_path=self._sdk,
                    wait=False,
                )
            return self._lift_reader

    def torso_imu(self):
        if self.profile.torso_imu is None:
            return None
        self.state()  # 先起 lowstate（ChannelFactory）
        with self._lock:
            if getattr(self, "_torso_imu", None) is None:
                from robot_platform.hw.dds.torso_imu import TorsoImuReader

                self._torso_imu = TorsoImuReader(
                    topic=self.profile.torso_imu.topic,
                    network_interface=self._iface,
                    dds_domain=self._domain,
                    sdk_path=self._sdk,
                )
            return self._torso_imu

    def chassis_driver(self):
        if self.profile.chassis is None:
            return super().chassis_driver()
        self.state()
        with self._lock:
            if getattr(self, "_chassis_driver", None) is None:
                from robot_platform.hw.dds.agv_client import AgvChassisDriver

                self._chassis_driver = AgvChassisDriver(
                    network_interface=self._iface, dds_domain=self._domain, sdk_path=self._sdk)
            return self._chassis_driver

    def chassis_odom(self):
        spec = self.profile.chassis
        if spec is None or not spec.odom_topic:
            return None
        self.state()
        with self._lock:
            if getattr(self, "_chassis_odom", None) is None:
                from robot_platform.hw.dds.agv_odom import AgvOdomReader

                self._chassis_odom = AgvOdomReader(
                    topic=spec.odom_topic, network_interface=self._iface, dds_domain=self._domain,
                    sdk_path=self._sdk)
            return self._chassis_odom

    def lift_driver(self) -> LiftDriver:
        if self.profile.lift is None:
            return super().lift_driver()
        self.state()
        with self._lock:
            if self._lift_driver is None:
                from robot_platform.hw.dds.agv_client import AgvClient

                self._lift_driver = AgvClient(
                    network_interface=self._iface,
                    dds_domain=self._domain,
                    sdk_path=self._sdk,
                    timeout_s=3.0,
                )
            return self._lift_driver
