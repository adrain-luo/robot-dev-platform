"""guardian：手臂指令经本体守护层 g1_guardian 发出，本程序不写 rt/lowcmd。

ArmLoop 照旧每拍拼一整帧（保持角、限速、拖拽、重力前馈、腰锁），
GuardianCommander 把帧里各组（right_arm / left_arm / waist）的角度、前馈和
刚度比例转成守护层设定（UDP），守护层用自己的 200 Hz 循环发帧。

所以平台进程卡住、崩溃、网络断开，手臂都只会原地保持（守护层的保证），
不会像直接发 lowcmd 那样一断流就掉。

状态、立柱、底盘、躯干 IMU 照旧由"读后端"（G1D 为 unitree_lowcmd）直接读 DDS，
读不占指令话题；读后端的 commander() 从不创建，本进程没有 lowcmd 的写入者。

设置（robot.json 的 control.guardian，环境变量可覆盖）：
    host        守护层地址          ARM_GUARDIAN_HOST   默认 192.168.123.164（本体内网地址：守护层按 README 绑在这个地址上，
                                                       平台在本体上或在 Box 上都能连到；守护层绑 127.0.0.1 时才改成 127.0.0.1）
    port        守护层 UDP 端口     ARM_GUARDIAN_PORT   默认 18090
    client_id   本程序在守护层的名字 ARM_GUARDIAN_CLIENT 默认 robot-platform
    read_backend 读状态用的后端      ARM_GUARDIAN_READ   默认按指令话题选 unitree_lowcmd / unitree_arm_sdk
"""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Mapping, Optional, Sequence, Tuple

from robot_platform.core.robot_profile import RobotProfile
from robot_platform.hw.backends.base import Backend, Commander, PermissionErrorMotion, StateSource, check_idle_motors

LIVE = ("holding", "tracking")
DEFAULT_PORT = 18090
# 设定有效期：平台循环卡顿超过它，守护层就减速保持（新设定来了自动继续）
SETPOINT_VALID_MS = 100
# 守护层状态这么久没更新，就当链路有问题（守护层自己会保持，这里只是停止报"正常"）
STATUS_MAX_AGE_S = 0.5
# 第一次接管时守护层要退出运控（ReleaseMode），可能要几秒
ENGAGE_TIMEOUT_S = 20.0
POKE_PERIOD_S = 1.0


@dataclass(frozen=True)
class GuardianSettings:
    host: str = "192.168.123.164"
    port: int = DEFAULT_PORT
    client_id: str = "robot-platform"
    read_backend: str = "unitree_lowcmd"
    release_s: float = 3.0  # 只用于提示和等待释放的上限；真正的时长在守护层配置里

    @property
    def address(self) -> str:
        return f"{self.host}:{self.port}"


def guardian_settings(profile: RobotProfile, env: Mapping[str, str] = os.environ) -> GuardianSettings:
    o = dict(profile.control.options.get("guardian") or {})
    default_read = "unitree_arm_sdk" if profile.control.command_topic == "rt/arm_sdk" else "unitree_lowcmd"
    read = str(env.get("ARM_GUARDIAN_READ") or o.get("read_backend") or default_read)
    if read == "guardian":
        raise ValueError("control.guardian.read_backend 不能是 guardian 自己")
    return GuardianSettings(
        host=str(env.get("ARM_GUARDIAN_HOST") or o.get("host") or "192.168.123.164"),
        port=int(env.get("ARM_GUARDIAN_PORT") or o.get("port") or DEFAULT_PORT),
        client_id=str(env.get("ARM_GUARDIAN_CLIENT") or o.get("client_id") or "robot-platform"),
        read_backend=read,
        release_s=float(o.get("release_s") or 3.0),
    )


# ---------------------------------------------------------------------- 文案
def engage_refusal_text(reason: str, where: str) -> str:
    """守护层拒绝 engage 的原因 → 给页面看的话。"""
    kind, _, detail = str(reason).partition(":")
    texts = {
        "estop_latched": "守护层急停已锁定：确认现场安全后先复位守护层，再接管",
        "owned_by_other": "守护层正被别的程序控制：等它释放，或它失联后再接管",
        "no_lowstate": "守护层收不到 lowstate（本体 DDS 有问题），不接管",
        "foreign_writer": "守护层发现 rt/lowcmd 上有别的程序在发令，不接管：先停掉它",
        "releasing": "守护层还在释放过渡，稍等再接管",
        "out_of_limits": f"有关节在限位外（{detail}），守护层不接管：先手动把它挪回限位内",
        "motors_not_idle": f"电机 {detail} 有读数，与机型声明不符（可能是站立中的双足机器人），守护层不接管",
        "prepare_failed": f"守护层退出运控失败：{detail}",
    }
    return texts.get(kind, f"守护层 {where} 拒绝接管：{reason}")


def lost_text(st: Dict[str, Any], client_id: str, epoch: int) -> Optional[str]:
    """已接管时，守护层状态说明我们已不再控制手臂 → 原因；仍在控制 → None。"""
    state = st.get("state")
    latched = list(st.get("latched") or [])
    owner = st.get("owner")
    st_epoch = st.get("epoch")
    if isinstance(st_epoch, int) and st_epoch < epoch:
        return None  # 比接管应答还旧的状态报文（乱序），不算
    if state == "estop":
        return "守护层已急停（锁存，手臂原地保持）：确认安全后复位，再重新接管"
    if state == "fault":
        if "link_lost" in latched:
            return "守护层判定本程序失联，已原地保持：重新接管即可继续"
        if "stale_lowstate" in latched:
            return "守护层收不到 lowstate，已原地保持：查本体 DDS，复位后再接管"
        if "foreign_writer" in latched:
            return "rt/lowcmd 上出现别的写入者，守护层已原地保持：停掉它，复位后再接管"
        return f"守护层故障（{', '.join(latched) or '未知'}），已原地保持：复位后再接管"
    if state in ("releasing", "idle"):
        return "守护层已释放手臂：要继续请重新接管"
    if owner != client_id:
        return f"守护层被「{owner}」接管了：等它释放后再接管"
    if st_epoch != epoch:
        return "守护层已复位或换了 epoch：重新接管即可继续"
    if state not in LIVE:
        return f"守护层状态 {state}：重新接管"
    return None


# ---------------------------------------------------------------- Commander
class GuardianCommander(Commander):
    """把 ArmLoop 的整机帧转成守护层设定。

    - 只发守护层配置里的组；组内角度、前馈取帧里对应电机的值。
    - 刚度比例 = 帧里的 kp / 机型 kp（拖拽时为 0），钳在 [0, 1]：守护层只认更软，不认更硬。
    - 守护层急停 / 故障 / 被别人接管 / 状态断流时 ready() 为假、phase() 给出原因，
      write() 不再发设定而是抛 PermissionErrorMotion（ArmLoop 记到 last_error）；
      要继续只能重新 engage（ArmLoop.engage 会从实测角重新开始）。
    """

    kind = "guardian"

    def __init__(self, profile: RobotProfile, client, settings: GuardianSettings, *,
                 reader: Optional[StateSource] = None, valid_for_ms: int = SETPOINT_VALID_MS,
                 status_max_age_s: float = STATUS_MAX_AGE_S, engage_timeout_s: float = ENGAGE_TIMEOUT_S,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._profile = profile
        self._client = client
        self._settings = settings
        self._reader = reader
        self._valid_ms = int(valid_for_ms)
        self._status_max_age = float(status_max_age_s)
        self._engage_timeout = float(engage_timeout_s)
        self._clock = clock
        self._kp_ref = tuple(float(k) for k in profile.kp)
        self._lock = threading.Lock()
        self._epoch: Optional[int] = None  # 正在用的接管（write 用它）
        self._pending: Optional[int] = None  # prepare 拿到、等 ArmLoop 重置完指令后 activate
        self._groups: Dict[str, Tuple[int, ...]] = {}
        self._release_until: Optional[float] = None
        self._sent = 0

    # ------------------------------------------------------------ 接管前
    def _check_layout(self, st: Dict[str, Any]) -> Dict[str, Tuple[int, ...]]:
        """守护层配置必须和本机型一致：同一机型，每只手是同一组电机。"""
        p = self._profile
        where = self._settings.address
        if st.get("robot") != p.name:
            raise PermissionErrorMotion(f"守护层 {where} 配置的是 {st.get('robot')}，本平台是 {p.name}，不接管")
        want_channel = "arm_sdk" if p.control.command_topic == "rt/arm_sdk" else "lowcmd"
        if st.get("channel") != want_channel:
            raise PermissionErrorMotion(f"守护层发 {st.get('channel')}，本机型要 {want_channel}，不接管")
        groups_st = st.get("groups") or {}
        groups: Dict[str, Tuple[int, ...]] = {}
        for arm, layout in p.arms.items():
            name = f"{arm}_arm"
            motors = tuple(int(m) for m in (groups_st.get(name) or {}).get("motors") or ())
            if motors != tuple(layout.motor_indices):
                raise PermissionErrorMotion(
                    f"守护层的 {name} 是电机 {list(motors)}，机型是 {list(layout.motor_indices)}，不接管")
            groups[name] = motors
        waist = groups_st.get("waist")
        if waist is not None:
            motors = tuple(int(m) for m in waist.get("motors") or ())
            hold = tuple(p.waist.hold_motors) if p.waist else ()
            if not motors or not set(motors) <= set(hold):
                raise PermissionErrorMotion(f"守护层的 waist 是电机 {list(motors)}，机型锁腰电机是 {list(hold)}，不接管")
            groups["waist"] = motors
        return groups

    def prepare(self) -> None:
        from g1_guardian.client import GuardianError, GuardianRejected

        where = self._settings.address
        if self._reader is not None:
            check_idle_motors(self._reader, self._profile.control.idle_motors)
        try:
            groups = self._check_layout(self._client.ping(timeout=1.0))
            epoch = self._client.engage(timeout=self._engage_timeout)
        except GuardianRejected as exc:
            raise PermissionErrorMotion(engage_refusal_text(exc.reason, where)) from None
        except GuardianError as exc:
            raise PermissionErrorMotion(f"连不上守护层 {where}（没启动、地址不对或网络不通）：{exc}") from None
        with self._lock:
            self._groups = groups
            self._pending = epoch
            self._release_until = None

    def activate(self) -> None:
        # ArmLoop 已把指令角重置为实测角：从这一拍起才用新 epoch 发设定，
        # 免得把重新接管前的旧指令角发给守护层
        with self._lock:
            if self._pending is not None:
                self._epoch, self._pending = self._pending, None

    # ------------------------------------------------------------ 每拍
    def _problem(self, epoch: int) -> Optional[str]:
        age = self._client.status_age()
        if age > self._status_max_age:
            return f"守护层状态 {age:.1f} s 没更新（网络或守护层有问题；守护层会自己原地保持）"
        st = self._client.latest_status()
        return None if st is None else lost_text(st, self._settings.client_id, epoch)

    def _kp_scale(self, kp: Sequence[float], motors: Sequence[int]) -> float:
        scale = 1.0
        for m in motors:
            ref = self._kp_ref[m] if m < len(self._kp_ref) else 0.0
            if ref > 0.0:
                scale = min(scale, float(kp[m]) / ref)
        return max(0.0, min(1.0, scale))

    def write(self, q: Sequence[float], kp: Sequence[float], kd: Sequence[float], tau: Sequence[float],
              *, mode_machine: Optional[int] = None) -> None:
        with self._lock:
            epoch, groups = self._epoch, self._groups
        if epoch is None:
            return  # 没接管（释放过渡中 ArmLoop 也会调用）：守护层自己在发帧
        problem = self._problem(epoch)
        if problem:
            raise PermissionErrorMotion(problem)
        body = {}
        for name, motors in groups.items():
            body[name] = {
                "q": [float(q[m]) for m in motors],
                "tau_ff": [float(tau[m]) for m in motors],
                "kp_scale": self._kp_scale(kp, motors),
            }
        self._client.setpoint(body, valid_for_ms=self._valid_ms)
        self._sent += 1

    # ------------------------------------------------------------ 释放
    def release(self) -> None:
        from g1_guardian.client import GuardianError

        with self._lock:
            epoch, self._epoch = self._epoch, None
            pending, self._pending = self._pending, None
        if epoch is None and pending is None:
            return
        try:
            self._client.release(timeout=2.0)
        except GuardianError as exc:
            # 守护层没回应：停掉心跳，它会判失联并原地保持（不会掉），也让别的程序能接管
            self._client.stop_heartbeat()
            raise PermissionErrorMotion(f"守护层没确认释放（{exc}）：它会原地保持手臂") from None
        self._release_until = self._clock() + self._settings.release_s + 2.0

    def busy(self) -> bool:
        until = self._release_until
        if until is None:
            return False
        st = self._client.latest_status()
        if self._clock() < until and st is not None and st.get("state") == "releasing":
            return True
        self._release_until = None
        return False

    def ready(self) -> bool:
        with self._lock:
            epoch = self._epoch
        return epoch is not None and self._problem(epoch) is None

    def phase(self) -> str:
        if self.busy():
            return "守护层释放中（刚度渐降到 0）"
        with self._lock:
            epoch = self._epoch
        if epoch is None:
            return ""
        return self._problem(epoch) or ""

    # ------------------------------------------------------------ 守护层操作
    def reset(self) -> None:
        """清守护层的急停 / 故障锁存（任何客户端都可以）。之后要重新 engage。"""
        from g1_guardian.client import GuardianError, GuardianRejected

        try:
            self._client.reset(timeout=2.0)
        except GuardianRejected as exc:
            raise PermissionErrorMotion(f"守护层拒绝复位：{exc.reason}") from None
        except GuardianError as exc:
            raise PermissionErrorMotion(f"连不上守护层 {self._settings.address}：{exc}") from None

    def info(self) -> Dict[str, Any]:
        st = self._client.latest_status() or {}
        age = self._client.status_age()
        with self._lock:
            epoch = self._epoch
        loop = st.get("loop") or {}
        return {
            "kind": self.kind,
            "address": self._settings.address,
            "client_id": self._settings.client_id,
            "reachable": age <= self._status_max_age,
            "status_age_s": None if age == float("inf") else round(age, 2),
            "state": st.get("state"),
            "latched": st.get("latched") or [],
            "owner": st.get("owner"),
            "ours": epoch is not None,
            "problem": None if epoch is None else self._problem(epoch),
            "setpoint_age_ms": st.get("setpoint_age_ms"),
            "lowstate_age_ms": st.get("lowstate_age_ms"),
            "loop_max_ms": loop.get("max_ms"),
            "loop_p99_ms": loop.get("p99_ms"),
            "overruns": loop.get("overruns"),
            "rejects": st.get("rejects") or {},
            "setpoints_sent": self._sent,
        }


# ------------------------------------------------------- 指令话题占用（页面胶囊）
class GuardianBus:
    """代替 CmdMonitor 给页面看：守护层是 lowcmd 唯一的写入者，"别人"由守护层自己检测。

    只读模式下也每秒戳一下守护层，让它继续推状态。"""

    def __init__(self, client, settings: GuardianSettings, *, clock: Callable[[], float] = time.monotonic) -> None:
        self._client = client
        self._settings = settings
        self._clock = clock
        self._last_poke = -1e9
        self.topic = f"守护层 {settings.address}"

    def report(self) -> Dict[str, Any]:
        now = self._clock()
        if now - self._last_poke >= POKE_PERIOD_S:
            self._last_poke = now
            self._client.poke()
        st = self._client.latest_status() or {}
        age = self._client.status_age()
        alive = age <= 2.0
        fw = st.get("foreign_writer_age_ms")
        foreign = bool(alive and fw is not None and fw < 1000.0)
        rate = 0.0
        if alive and st.get("state") not in (None, "idle"):
            period = (st.get("loop") or {}).get("p50_ms")
            rate = round(1000.0 / period, 1) if period else 0.0
        return {
            "topic": self.topic,
            "own_hz": rate,  # 守护层的发帧频率（它替本程序发）
            "foreign_hz": 0.0,
            "foreign": foreign,
            "listening_s": None,
            "last_foreign_age_s": None if fw is None else round(fw / 1000.0, 1),
            "last_weight": None,
            "last_kp_max": None,
            "guardian_state": st.get("state") if alive else "unreachable",
            "guardian_owner": st.get("owner") if alive else None,
        }


# ------------------------------------------------------------------ Backend
class GuardianBackend(Backend):
    name = "guardian"

    def __init__(self, profile: RobotProfile, read_backend: Backend, settings: GuardianSettings, *,
                 client_factory: Optional[Callable[[GuardianSettings], Any]] = None) -> None:
        self.profile = profile
        self.settings = settings
        self._read = read_backend
        self._client_factory = client_factory or _default_client
        self._lock = threading.Lock()
        self._client = None
        self._commander: Optional[GuardianCommander] = None
        self._bus: Optional[GuardianBus] = None

    @property
    def read_backend(self) -> Backend:
        return self._read

    def client(self):
        with self._lock:
            if self._client is None:
                self._client = self._client_factory(self.settings)
            return self._client

    def state(self) -> StateSource:
        return self._read.state()

    def lowstate_max_age_s(self) -> None:
        # 不沿用读状态后端的上限：lowstate 新鲜度由本体上的守护层自己查（lowstate_stale_ms），
        # 平台这边守护层状态太久没更新时 commander 会报没就绪
        return None

    def commander(self) -> Commander:
        reader = self.state()
        client = self.client()
        with self._lock:
            if self._commander is None:
                self._commander = GuardianCommander(self.profile, client, self.settings, reader=reader)
            return self._commander

    def cmd_monitor(self):
        client = self.client()
        with self._lock:
            if self._bus is None:
                self._bus = GuardianBus(client, self.settings)
            return self._bus

    def reset_guardian(self) -> None:
        self.commander()
        assert self._commander is not None
        self._commander.reset()

    # 立柱、底盘、IMU 都不经守护层
    def lift_reader(self):
        return self._read.lift_reader()

    def lift_driver(self):
        return self._read.lift_driver()

    def torso_imu(self):
        return self._read.torso_imu()

    def chassis_driver(self):
        return self._read.chassis_driver()

    def chassis_odom(self):
        return self._read.chassis_odom()

    def close(self) -> None:
        with self._lock:
            client, self._client = self._client, None
        if client is not None:
            client.close()
        self._read.close()


def _default_client(settings: GuardianSettings):
    from g1_guardian.client import GuardianClient

    return GuardianClient(settings.host, settings.port, client_id=settings.client_id)
