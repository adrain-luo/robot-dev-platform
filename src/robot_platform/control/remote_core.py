"""网页这一侧的运控核心（--motion-core ipc）：和进程内 MotionCore 同样的方法，经 g1_motion_client 调运控核心进程。

- 页面、任务、标定代码拿到的对象和进程内时用法一样：拒绝照样抛 PermissionErrorMotion / ValueError /
  MotionRejected（租约失效是 LeaseRevoked），租约的 cancelled 在核心撤销时置位。
- 连不上 / 断线：调用抛 PermissionErrorMotion（"连不上运控核心"），持有的租约全部置为已撤销；
  核心那边已经按断线撤销了租约、停住了这边动过的手，帧照发。下一次调用自动重连，租约不会自动重新申请。
- RemoteStateSource：网页读的实测角也来自核心（和核心用的是同一份 lowstate；仿真时两个进程各有一个
  仿真机器人，只有核心那个在动）。
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional, Sequence, Union

import numpy as np

from g1_contracts.motion import MotionRejected, MotionState, Resource, SafetyState, Stamp
from g1_motion_client import CoreConnection, CoreError, CoreUnavailable, MotionClient, RemoteLease
from robot_platform.control.motion_core import ARM_RESOURCES, resource_for_arm
from robot_platform.core.layout import ArmLayout
from robot_platform.hw.backends.base import ArmState, MotorSnapshot, PermissionErrorMotion, StateSource

LONG_CALL_S = 60.0  # 接管（守护层退出运控要几秒）、释放、带绕行规划的走点


def _translate(exc: Exception) -> Exception:
    """核心那边的错误 → 和进程内一样的异常类型。"""
    if isinstance(exc, CoreUnavailable):
        return PermissionErrorMotion(f"连不上运控核心（{exc}）：手臂由核心保持；核心起来后再试")
    if isinstance(exc, CoreError):
        if exc.kind == "permission":
            return PermissionErrorMotion(exc.detail)
        if exc.kind == "collision":
            from robot_platform.collision.guard import CollisionBlocked
            return CollisionBlocked(exc.detail)
        if exc.kind == "value":
            return ValueError(exc.detail)
        if exc.kind == "busy":
            return PermissionErrorMotion(f"运控核心忙（{exc.detail}），稍后再试")
        if exc.kind == "version":
            return PermissionErrorMotion(f"运控核心的接口版本不一致（{exc.detail}）：网页和核心要用同一版本")
        return RuntimeError(f"运控核心：{exc.detail or exc.kind}")
    return exc


class RemoteMotionCore:
    """方法与 control/motion_core.py 的 MotionCore 一一对应（页面、任务、标定用到的那些）。"""

    def __init__(self, conn: CoreConnection, profile, *, connect: bool = True) -> None:
        self._conn = conn
        self._client = MotionClient(conn)
        self._profile = profile
        self._info: Dict[str, Any] = {}
        if connect:
            self.connect()

    # ------------------------------------------------------------ 连接
    def connect(self) -> Dict[str, Any]:
        """连上（已连上时不重连）并核对机型；连不上抛 PermissionErrorMotion。重试有限频（0.5 s 一次）。"""
        try:
            self._conn.ensure()
        except (CoreUnavailable, CoreError) as exc:
            raise _translate(exc) from None
        info = self._conn.core_info
        if info is not self._info:
            self._check_layout(info)
            self._info = info
        return info

    def _check_layout(self, info: Dict[str, Any]) -> None:
        """核心和网页必须是同一机型、每只手同一组电机，否则不用它。"""
        robot = info.get("robot")
        if robot != self._profile.name:
            raise PermissionErrorMotion(f"运控核心是 {robot}，网页是 {self._profile.name}：两边要用同一机型")
        arms = info.get("arms") or {}
        for name, lay in self._profile.arms.items():
            motors = list((arms.get(name) or {}).get("motors") or [])
            if motors != list(lay.motor_indices):
                raise PermissionErrorMotion(f"运控核心的{name}臂是电机 {motors}，网页是 {list(lay.motor_indices)}")

    def _call(self, op: str, *, timeout_s: Optional[float] = None, **args: Any) -> Any:
        self.connect()
        try:
            return self._conn.call(op, timeout_s=timeout_s, **args)
        except (CoreUnavailable, CoreError) as exc:
            raise _translate(exc) from None

    @property
    def connection(self) -> CoreConnection:
        return self._conn

    # ------------------------------------------------------------ 只读：手臂信息
    @property
    def arm_names(self) -> List[str]:
        return list((self._info.get("arms") or {}).keys())

    @property
    def max_speed(self) -> float:
        return float(self._call("max_speed"))

    @property
    def host(self) -> str:
        return str(self._info.get("host") or "local")

    def layout(self, arm: str) -> ArmLayout:
        return self._profile.arms[arm]

    def limits(self, arm: str) -> np.ndarray:
        return np.asarray(self._info["arms"][arm]["limits"], dtype=float)

    def arm_status(self, arm: str) -> dict:
        return self.page_status()["arms"][arm]

    def arm_sample(self, arm: str, wait: bool = False) -> dict:
        s = self._call("arm_sample", arm=arm, wait=bool(wait))
        s["tau_est"] = [None if t is None else float(t) for t in s.get("tau_est") or []]
        return s

    def arm_gains(self, arm: str) -> tuple:
        kp, kd = self._call("arm_gains", arm=arm)
        return list(kp), list(kd)

    def payload_theta(self, arm: str) -> Optional[list]:
        return self._call("payload_theta", arm=arm)

    def set_payload_theta(self, theta, name: Optional[str] = None, *, arm: str) -> None:
        self._call("set_payload_theta", arm=arm, name=name,
                   theta=None if theta is None else [float(v) for v in np.asarray(theta, float).reshape(-1)])

    def manual_target(self, q: Sequence[float], *, arm: str, detour: bool = False) -> Optional[dict]:
        return self._call("manual_target", timeout_s=LONG_CALL_S, arm=arm, q=[float(v) for v in q],
                          detour=bool(detour))

    def arm_view_status(self, arm: str) -> dict:
        st = self._call("arm_view_status", arm=arm)
        return st

    # ------------------------------------------------------------ 页面
    def page_status(self) -> dict:
        return self._call("page_status")

    def hold_arm(self, arm: str) -> None:
        self._call("hold_arm", arm=arm)

    def set_waist(self, yaw_rad: Optional[float] = None, pitch_rad: Optional[float] = None) -> None:
        self._call("set_waist", yaw_rad=None if yaw_rad is None else float(yaw_rad),
                   pitch_rad=None if pitch_rad is None else float(pitch_rad))

    def hold_waist(self) -> None:
        self._call("hold_waist")

    def enable_teach(self, arm: Optional[str] = None) -> None:
        self._call("enable_teach", arm=arm)

    def disable_teach(self, arm: Optional[str] = None) -> None:
        self._call("disable_teach", arm=arm)

    def set_gravity_ff(self, on: bool) -> None:
        self._call("set_gravity_ff", on=bool(on))

    def set_gravity_imu(self, on: bool) -> None:
        self._call("set_gravity_imu", on=bool(on))

    def gravity_up(self) -> List[float]:
        return [float(v) for v in self._call("gravity_up")]

    def set_max_speed(self, v: float) -> float:
        return float(self._call("set_max_speed", v=float(v)))

    @property
    def collision_guard(self):
        return None  # 闸门在核心进程里；状态经 collision() 取

    def collision(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        return self._call("collision", timeout_s=LONG_CALL_S, path=path, body=body)

    def lowstate(self) -> Dict[str, Any]:
        return self._call("lowstate")

    def shutdown(self, timeout_s: float = 30.0) -> None:
        """网页退出：只断开连接。核心照常保持手臂（这边的租约由核心按断线撤销）。"""
        self._conn.close()

    # ------------------------------------------------------------ 整机
    def engage(self) -> None:
        self._call("engage", timeout_s=LONG_CALL_S)

    def release(self) -> None:
        self._call("release", timeout_s=LONG_CALL_S)

    def estop(self, reason: str = "") -> None:
        """经 estop.sock（不排在任务消息后面）。核心先锁存，守护层后端时再锁守护层。"""
        try:
            self._conn.estop(reason)
        except (CoreUnavailable, CoreError) as exc:
            if isinstance(exc, CoreUnavailable):
                raise PermissionErrorMotion(
                    f"急停没送到运控核心（{exc}）：核心若还在，手臂按它自己的状态保持；请用现场断电或遥控") from None
            raise _translate(exc) from None

    def reset_estop(self) -> None:
        self._call("reset_estop")

    def reset_external_estop(self) -> None:
        self._call("reset_external_estop", timeout_s=LONG_CALL_S)

    @property
    def has_external_estop(self) -> bool:
        return bool(self._info.get("external_estop"))

    @property
    def latched(self) -> Optional[str]:
        return (self.page_status().get("motion") or {}).get("latch_reason")

    def status(self, loop_status: Optional[dict] = None) -> Dict[str, Any]:
        return (loop_status or self.page_status()).get("motion") or {}

    def safety(self, loop_status: Optional[dict] = None) -> SafetyState:
        if loop_status is not None:
            return SafetyState((loop_status.get("motion") or {}).get("safety") or "hold")
        try:
            return self._client.safety()
        except (CoreUnavailable, CoreError) as exc:
            raise _translate(exc) from None

    def state(self, resource: Resource) -> MotionState:
        try:
            return self._client.state(resource)
        except (CoreUnavailable, CoreError) as exc:
            raise _translate(exc) from None

    def state_at(self, resource: Resource, stamp: Stamp) -> MotionState:
        try:
            return self._client.state_at(resource, stamp)
        except (CoreUnavailable, CoreError) as exc:
            raise _translate(exc) from None

    # ------------------------------------------------------------ 租约
    def lease(self, resources: Sequence[Resource], owner: str, *,
              heartbeat_ms: Optional[int] = None, max_speed_rad_s: Optional[float] = None) -> "RemotePlatformLease":
        self.connect()
        try:
            inner = self._client.lease(resources, owner, heartbeat_ms=heartbeat_ms, max_speed_rad_s=max_speed_rad_s)
        except MotionRejected:
            raise
        except (CoreUnavailable, CoreError) as exc:
            raise _translate(exc) from None
        return RemotePlatformLease(self, inner)

    def lease_arm(self, arm: str, owner: str) -> "RemotePlatformLease":
        return self.lease([resource_for_arm(arm)], owner)


class RemotePlatformLease:
    """RemoteLease 加上进程内 CoreLease 的单臂便利方法（move_to / status / arm_sample / layout / limits ...）。"""

    def __init__(self, core: RemoteMotionCore, inner: RemoteLease) -> None:
        self._core = core
        self._inner = inner

    # ---- 协议（照转）
    @property
    def lease(self):
        return self._inner.lease

    @property
    def owner(self) -> str:
        return self._inner.owner

    @property
    def cancelled(self):
        return self._inner.cancelled

    @property
    def revoked(self) -> Optional[str]:
        return self._inner.revoked

    def check(self) -> None:
        self._inner.check()

    def cancel(self) -> None:
        self._inner.cancel()

    def goal(self, resource, path, *, timeout_ms=None, allow_detour=False):
        return self._wrap(self._inner.goal, resource, path, timeout_ms=timeout_ms, allow_detour=allow_detour)

    def stream(self, resource, q, *, valid_for_ms=200, derived_from=None, speed_scale=1.0):
        return self._wrap(self._inner.stream, resource, q, valid_for_ms=valid_for_ms, derived_from=derived_from,
                          speed_scale=speed_scale)

    def hold(self, resource=None):
        return self._inner.hold(resource)

    def release(self) -> None:
        self._inner.release()

    def __enter__(self) -> "RemotePlatformLease":
        return self

    def __exit__(self, *exc) -> None:
        self.release()

    @staticmethod
    def _wrap(fn, *a, **kw):
        try:
            return fn(*a, **kw)
        except CoreError as exc:
            raise _translate(exc) from None

    # ---- 单臂便利方法
    def _res(self, resource: Optional[Resource]) -> Resource:
        return self._inner._res(resource)

    def arm_name(self, resource: Optional[Resource] = None) -> str:
        return ARM_RESOURCES[self._res(resource)]

    def move_to(self, q: Sequence[float], *, timeout_ms: Optional[int] = None, allow_detour: bool = False,
                resource: Optional[Resource] = None):
        return self.goal(self._res(resource), [[float(v) for v in q]], timeout_ms=timeout_ms,
                         allow_detour=allow_detour)

    def status(self, resource: Optional[Resource] = None) -> dict:
        return self._core.arm_view_status(self.arm_name(resource))

    def arm_sample(self, wait: bool = False, resource: Optional[Resource] = None) -> dict:
        return self._core.arm_sample(self.arm_name(resource), wait=wait)

    def arm_gains(self, resource: Optional[Resource] = None) -> tuple:
        return self._core.arm_gains(self.arm_name(resource))

    @property
    def layout(self) -> ArmLayout:
        return self._core.layout(self.arm_name())

    @property
    def limits(self) -> np.ndarray:
        return self._core.limits(self.arm_name())

    @property
    def max_speed(self) -> float:
        return self._core.max_speed

    def payload_theta(self, resource: Optional[Resource] = None) -> Optional[list]:
        return self._core.payload_theta(self.arm_name(resource))

    def set_payload_theta(self, theta, name: Optional[str] = None, resource: Optional[Resource] = None) -> None:
        self._core.set_payload_theta(theta, name, arm=self.arm_name(resource))


class RemoteStateSource(StateSource):
    """网页的 lowstate 读数来自运控核心（一次调用取全部电机）。"""

    def __init__(self, core_call, profile, *, clock=time.monotonic) -> None:
        self._call = core_call  # () -> lowstate dict
        self._profile = profile
        self._clock = clock

    def _get(self) -> Dict[str, Any]:
        return self._call()

    def wait_for_message(self, timeout_s: Optional[float] = None) -> None:
        deadline = self._clock() + float(timeout_s if timeout_s is not None else 5.0)
        while True:
            if self._get().get("age_s") is not None:
                return
            if self._clock() >= deadline:
                raise RuntimeError("运控核心还没收到 lowstate")
            time.sleep(0.05)

    def age_s(self) -> float:
        age = self._get().get("age_s")
        return float("inf") if age is None else float(age)

    def _layout(self, arm: Union[str, ArmLayout]) -> ArmLayout:
        return arm if isinstance(arm, ArmLayout) else self._profile.arms[arm]

    def read_arm(self, arm: Union[str, ArmLayout], *, wait: bool = True) -> ArmState:
        layout = self._layout(arm)
        d = self._get()
        if d.get("age_s") is None and wait:
            self.wait_for_message()
            d = self._get()
        idx = layout.motor_indices
        age = d.get("age_s")
        return ArmState(
            timestamp_monotonic=self._clock() - (float(age) if age is not None else math.inf),
            mode_machine=d.get("mode_machine"),
            q=tuple(float(d["q"][i]) for i in idx),
            dq=tuple(float(d["dq"][i]) for i in idx),
            tau_est=tuple(None if d["tau_est"][i] is None else float(d["tau_est"][i]) for i in idx),
        )

    def read_motors(self, motor_indices: Sequence[int]) -> Dict[int, float]:
        d = self._get()
        return {int(i): float(d["q"][i]) for i in motor_indices}

    def read_motor_states(self, motor_indices: Sequence[int]) -> Dict[int, Dict[str, float]]:
        d = self._get()
        nan = float("nan")
        return {int(i): {"q": float(d["q"][i]), "dq": float(d["dq"][i]),
                         "tau_est": nan if d["tau_est"][i] is None else float(d["tau_est"][i])}
                for i in motor_indices}

    def snapshot(self) -> MotorSnapshot:
        d = self._get()
        return MotorSnapshot(q=tuple(float(v) for v in d["q"]), mode_machine=d.get("mode_machine"))

    def read_imu(self) -> Optional[Dict[str, Any]]:
        return self._get().get("imu")
