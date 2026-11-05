"""unitree_arm_sdk：运控保持开启（机器人自己站稳），本程序只经 rt/arm_sdk 接管腰与双臂（G1）。

与 unitree_lowcmd 的区别：
- 绝不调用 ReleaseMode；接管前要求运控正在运行，且没有别的程序在发 rt/arm_sdk。
- 第 29 号槽位的 q 是“权重”：0 = 运控完全控制，1 = 完全听本程序。
- 接管：权重 0→1（此时增益与运控一致），再把增益从运控原值过渡到本程序的值；
  释放：先把增益过渡回运控原值，再把权重降到 0，之后才停发。任何一刻都不突变。
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Callable, List, Optional, Sequence, Tuple

from robot_platform.hw.backends.base import Backend, Commander, PermissionErrorMotion, StateSource
from robot_platform.hw.dds.sdk import init_channel
from robot_platform.core.robot_profile import RobotProfile


def _smooth(u: float) -> float:
    u = min(1.0, max(0.0, u))
    return u * u * (3.0 - 2.0 * u)


class ArmSdkSchedule:
    """接管 / 释放的权重与增益过渡，纯计时逻辑（不碰 SDK，便于测试）。

    阶段：idle → acquiring（权重升）→ gaining（增益升）→ active
          active / gaining / acquiring → releasing_gain（增益降）→ releasing（权重降）→ idle
    step() 返回 (权重, 增益比例)：增益比例 0 = 运控原增益，1 = 本程序增益。
    """

    def __init__(self, acquire_s: float, gain_ramp_s: float, release_gain_s: float, release_s: float,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.acquire_s = max(1e-3, float(acquire_s))
        self.gain_ramp_s = max(1e-3, float(gain_ramp_s))
        self.release_gain_s = max(1e-3, float(release_gain_s))
        self.release_s = max(1e-3, float(release_s))
        self._clock = clock
        self._lock = threading.Lock()
        self.state = "idle"
        self._t0 = 0.0
        self._from = 0.0
        self.weight = 0.0
        self.gain = 0.0

    def start(self) -> None:
        with self._lock:
            if self.state != "idle":
                raise PermissionErrorMotion(f"arm_sdk 正在{self.state}，等过渡结束再接管")
            self.state, self._t0 = "acquiring", self._clock()
            self.weight = self.gain = 0.0

    def release(self) -> None:
        with self._lock:
            if self.state in ("idle", "releasing_gain", "releasing"):
                return
            now = self._clock()
            if self.gain > 0.0:
                self.state, self._t0, self._from = "releasing_gain", now, self.gain
            else:
                self.state, self._t0, self._from = "releasing", now, self.weight

    def step(self) -> Tuple[float, float]:
        with self._lock:
            now = self._clock()
            u = now - self._t0
            if self.state == "acquiring":
                self.weight = _smooth(u / self.acquire_s)
                if u >= self.acquire_s:
                    self.weight, self.state, self._t0 = 1.0, "gaining", now
            elif self.state == "gaining":
                self.gain = _smooth(u / self.gain_ramp_s)
                if u >= self.gain_ramp_s:
                    self.gain, self.state = 1.0, "active"
            elif self.state == "releasing_gain":
                self.gain = self._from * (1.0 - _smooth(u / self.release_gain_s))
                if u >= self.release_gain_s:
                    self.gain, self.state, self._t0, self._from = 0.0, "releasing", now, self.weight
            elif self.state == "releasing":
                self.weight = self._from * (1.0 - _smooth(u / self.release_s))
                if u >= self.release_s:
                    self.weight, self.state = 0.0, "idle"
            return self.weight, self.gain

    def busy(self) -> bool:
        return self.state != "idle"

    def ready(self) -> bool:
        return self.state == "active"


def blend_frame(q: Sequence[float], kp: Sequence[float], kd: Sequence[float], tau: Sequence[float],
                motors: Sequence[int], native_kp: Sequence[float], native_kd: Sequence[float],
                gain: float) -> List[Tuple[int, float, float, float, float]]:
    """按增益比例混合运控原增益与本程序增益；前馈力矩随增益一起加入。返回 [(电机, q, kp, kd, τ)]。"""
    out = []
    for j, i in enumerate(motors):
        out.append((
            int(i),
            float(q[i]),
            float(native_kp[j] + gain * (kp[i] - native_kp[j])),
            float(native_kd[j] + gain * (kd[i] - native_kd[j])),
            float(gain * tau[i]),
        ))
    return out


class ArmSdkCommander(Commander):
    def __init__(self, *, profile: RobotProfile, dds_domain: int, network_interface: str,
                 sdk_path: Optional[Path]) -> None:
        opt = profile.control.options
        self.motors = tuple(int(i) for i in opt["command_motors"])
        self.weight_motor = int(opt.get("weight_motor", 29))
        self.native_kp = tuple(float(v) for v in opt["native_kp"])
        self.native_kd = tuple(float(v) for v in opt["native_kd"])
        if not (len(self.native_kp) == len(self.native_kd) == len(self.motors)):
            raise ValueError("native_kp / native_kd 要与 command_motors 一一对应")
        self.schedule = ArmSdkSchedule(opt.get("acquire_s", 8.0), opt.get("gain_ramp_s", 4.0),
                                       opt.get("release_gain_s", 2.0), opt.get("release_s", 3.0))
        topic = profile.control.command_topic
        if topic != "rt/arm_sdk":
            raise ValueError(f"unitree_arm_sdk 只发 rt/arm_sdk，配置里是 {topic}")

        init_channel(dds_domain, network_interface, sdk_path)
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.utils.crc import CRC

        self._crc = CRC()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        # 先订阅再发布：接管前用它确认没有别的程序在发 rt/arm_sdk
        self._others_t = 0.0
        self._sending = False
        self._sub = ChannelSubscriber(topic, LowCmd_)
        self._sub.Init(self._on_other, 10)
        self._pub = ChannelPublisher(topic, LowCmd_)
        self._pub.Init()

    def _on_other(self, _msg) -> None:
        if not self._sending:
            self._others_t = time.monotonic()

    def _motion_mode(self) -> str:
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

        ms = MotionSwitcherClient()
        ms.SetTimeout(3.0)
        ms.Init()
        code, result = ms.CheckMode()
        if code != 0:
            raise PermissionErrorMotion(f"查询运动模式失败（{code}），不接管")
        return str((result or {}).get("name") or "")

    def prepare(self) -> None:
        if self.schedule.state == "active":
            return
        mode = self._motion_mode()
        if not mode:
            raise PermissionErrorMotion("运控没有在运行：arm_sdk 依赖运控保持站立，不接管")
        time.sleep(0.3)
        if time.monotonic() - self._others_t < 0.5:
            raise PermissionErrorMotion("已有别的程序在发 rt/arm_sdk（例如另一套臂控），不接管")
        self.schedule.start()
        self._sending = True

    def write(self, q, kp, kd, tau, *, mode_machine: Optional[int] = None) -> None:
        if not self.schedule.busy():
            self._sending = False
            return
        weight, gain = self.schedule.step()
        self._cmd.mode_pr = 0
        self._cmd.mode_machine = int(mode_machine or 0)
        for i, qi, kpi, kdi, ti in blend_frame(q, kp, kd, tau, self.motors, self.native_kp, self.native_kd, gain):
            slot = self._cmd.motor_cmd[i]
            slot.mode = 1
            slot.q = qi
            slot.dq = 0.0
            slot.kp = kpi
            slot.kd = kdi
            slot.tau = ti
        self._cmd.motor_cmd[self.weight_motor].q = float(weight)
        self._cmd.crc = self._crc.Crc(self._cmd)
        self._pub.Write(self._cmd)

    def release(self) -> None:
        self.schedule.release()

    def busy(self) -> bool:
        return self.schedule.busy()

    def ready(self) -> bool:
        return self.schedule.ready()

    def phase(self) -> str:
        s = self.schedule
        if s.state in ("idle", "active"):
            return ""
        names = {"acquiring": "接管：权重上升", "gaining": "接管：增益过渡",
                 "releasing_gain": "释放：增益回到运控", "releasing": "释放：权重下降"}
        return f"{names[s.state]} · 权重 {s.weight:.2f} · 增益 {s.gain:.2f}"


class UnitreeArmSdkBackend(Backend):
    name = "unitree_arm_sdk"

    def __init__(self, profile: RobotProfile, *, network_interface: str, dds_domain: int = 0,
                 sdk_path: Optional[Path] = None, timeout_s: float = 5.0) -> None:
        if profile.control.release_motion_mode:
            raise ValueError("unitree_arm_sdk 不退出运控，robot.json 的 release_motion_mode 必须为 false")
        self.profile = profile
        self._iface = network_interface
        self._domain = dds_domain
        self._sdk = sdk_path
        self._timeout_s = timeout_s
        self._lock = threading.Lock()
        self._state = None
        self._commander = None

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

    def commander(self) -> Commander:
        self.state()
        with self._lock:
            if self._commander is None:
                self._commander = ArmSdkCommander(
                    profile=self.profile,
                    dds_domain=self._domain,
                    network_interface=self._iface,
                    sdk_path=self._sdk,
                )
            return self._commander
