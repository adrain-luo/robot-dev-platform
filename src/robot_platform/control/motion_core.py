"""运控核心（阶段 1，进程内）：在 ArmLoop 外包一层 g1_contracts 接口。

任务 / 技能只通过租约动手臂，不再直接拿 ArmLoop：

- 租约（owner + epoch）：每只手同一时刻只有一个租约。租约的占用记号（owner#编号）写进
  ArmLoop 的占用者，所以页面照样显示谁在用这只手，手动来源照样被拒。
- 撤销：急停、重新接管、释放、指令出口失去就绪（守护层急停 / 故障 / 失联）都会让
  epoch 加一、撤销全部租约、置位各租约的 cancelled 事件；旧租约之后的指令一律拒绝
  （ArmLoop 里带 owner 的目标也会被拒，见 ArmLoop.set_target）。被撤销的手停在指令角。
- goal：到位类目标（可规划绕行）；stream：视觉闭环类目标，带有效期，过期就停在指令角。
- state_at：按时间戳查当时的实测角 / 指令角（约 2 s 历史，线性插值，只认本机时间戳）。
- 急停锁存：estop() 撤销租约、两手锁在实测角、禁止一切新运动（包括页面手动），
  只有 reset_estop() 能解除；接管、释放、保持照常可用。
- 外部急停（external_estop，守护层后端时是守护层的急停）：核心锁存之后再锁它，
  这样经守护层发令的别的程序也被拦住，平台进程崩了锁存也还在本体上。
  它的复位另走（守护层复位），reset_estop() 不碰它。

不改发帧：限速、限位、增益、前馈、频率都还在 ArmLoop（和守护层）里，这里只做仲裁。
"""

from __future__ import annotations

import bisect
import itertools
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np

from g1_contracts.motion import (
    Ack, GoalRequest, Lease, LeaseRequest, MotionState, Reason, Resource, SafetyState,
    Stamp, StreamSetpoint, Verdict, rejection,
)
from robot_platform.control.arm_loop import ArmLoop, PermissionErrorMotion, side_label

ARM_RESOURCES: Dict[Resource, str] = {Resource.RIGHT_ARM: "right", Resource.LEFT_ARM: "left"}
HISTORY_S = 2.0          # state_at 能查多久以前
STAMP_TOLERANCE_MS = 50  # 时间戳比最新一条历史新这么多以内，按最新一条算
_GONE_KEEP = 64          # 记住最近撤销 / 归还的租约，拒绝时能说清原因


def resource_for_arm(arm: str) -> Resource:
    for res, name in ARM_RESOURCES.items():
        if name == arm:
            return res
    raise ValueError(f"没有 {arm!r} 臂对应的资源")


@dataclass
class _LeaseRec:
    lease: Lease
    token: str                       # 写进 ArmLoop 的占用者
    arms: Tuple[str, ...]
    cancelled: threading.Event = field(default_factory=threading.Event)
    revoked: Optional[str] = None    # 撤销原因
    returned: bool = False
    holder_cancelled: bool = False
    max_speed: Optional[float] = None  # 租约自带的限速（None = 全局限速）


@dataclass(frozen=True)
class _Sample:
    t_meas: int                          # 实测角对应的时刻（收到 lowstate 的时刻）
    t_cmd: int                           # 指令角对应的时刻（这一拍）
    q_meas: Dict[str, Tuple[float, ...]]
    q_cmd: Dict[str, Tuple[float, ...]]


def _reject(seq: int, reason: Reason, detail: str) -> Ack:
    return Ack(seq=seq, verdict=Verdict.REJECTED, reason=reason, detail=detail)


class MotionCore:
    """一台机器一个；包住唯一的 ArmLoop。线程安全。"""

    def __init__(
        self,
        loop: ArmLoop,
        *,
        host: str = "local",
        clock_ns: Callable[[], int] = time.monotonic_ns,
        history_s: float = HISTORY_S,
        max_observation_age_ms: Optional[int] = None,
        install: bool = True,
        external_estop: Optional[Callable[[str], None]] = None,
        external_reset: Optional[Callable[[], None]] = None,
    ) -> None:
        self._loop = loop
        # 核心锁存之后再锁的外部急停（守护层）；它出错时核心照样已锁存，错误抛给调用方
        self._external_estop = external_estop
        # 复位外部急停 / 故障锁存（守护层复位）；不解除核心自己的锁存
        self._external_reset = external_reset
        self.host = str(host)
        self._clock = clock_ns
        self._history_ns = int(float(history_s) * 1e9)
        # 观测太旧就拒绝 stream（None = 不查；阈值由负责人定，阶段 1 没有调用方用 stream）
        self._max_obs_ns = None if max_observation_age_ms is None else int(max_observation_age_ms) * 1_000_000
        self._lock = threading.Lock()
        self._epoch = 0
        self._ids = itertools.count(1)
        self._leases: Dict[str, _LeaseRec] = {}
        self._by_arm: Dict[str, str] = {}
        self._gone: Dict[str, str] = {}
        # 手 → (截止时刻 ns, "stream" / "goal", 租约 id)
        self._deadlines: Dict[str, Tuple[int, str, str]] = {}
        self._latched: Optional[str] = None
        self._transition = False
        self._last_revoke: Optional[Dict[str, Any]] = None
        self._hist: Deque[_Sample] = deque()
        self._hist_lock = threading.Lock()
        if install:
            loop.on_cycle = self._on_cycle

    # ------------------------------------------------------------ 只读：手臂信息
    @property
    def loop(self) -> ArmLoop:
        return self._loop

    @property
    def epoch(self) -> int:
        return self._epoch

    @property
    def arm_names(self) -> List[str]:
        return self._loop.arm_names

    @property
    def max_speed(self) -> float:
        return self._loop.max_speed

    def layout(self, arm: str):
        return self._loop.layout(arm)

    def limits(self, arm: str) -> np.ndarray:
        return self._loop.limits(arm)

    def arm_status(self, arm: str) -> dict:
        return self._loop.arm_status(arm)

    def arm_sample(self, arm: str, wait: bool = False) -> dict:
        return self._loop.arm_sample(arm, wait=wait)

    def arm_gains(self, arm: str) -> tuple:
        return self._loop.arm_gains(arm)

    def payload_theta(self, arm: str) -> Optional[list]:
        return self._loop.payload_theta(arm)

    def set_payload_theta(self, theta, name: Optional[str] = None, *, arm: str) -> None:
        """应用负载补偿（配置，不是运动目标；任务内改用租约上的同名方法）。"""
        self._loop.set_payload_theta(theta, name, arm=arm)

    def manual_target(self, q: Sequence[float], *, arm: str, detour: bool = False) -> Optional[dict]:
        """手动来源（页面走点、姿态集「到这一点」）：不占用这只手。这只手被租约占着、
        在拖拽、急停锁存时 ArmLoop 拒绝（PermissionErrorMotion）。返回绕行规划摘要（没绕行为 None）。"""
        return self._loop.set_target(q, arm=arm, detour=detour)

    def arm_view_status(self, arm: str) -> dict:
        """整机状态加这只手的字段（和旧 ArmView.status 一样的形状）。"""
        st = self._loop.status()
        st.update(st["arms"][arm])
        return st

    # ------------------------------------------------------------ 页面：手动来源与设置（不经租约）
    # 网页只经这些方法碰控制循环（阶段 2 起网页可能在另一个进程，见 g1_motion_core）。
    # 拒绝时 ArmLoop 抛 PermissionErrorMotion / ValueError，和以前直接调 ArmLoop 一样。
    def page_status(self) -> dict:
        """控制循环状态加运控核心状态（"motion"）：页面轮询的 ctrl。"""
        st = self._loop.status()
        st["motion"] = self.status(st)
        return st

    def hold_arm(self, arm: str) -> None:
        """只停这一只手、锁在实测角（占用它的任务不受影响，要停任务用急停）。"""
        self._loop.hold_arm(arm)

    def set_waist(self, yaw_rad: Optional[float] = None, pitch_rad: Optional[float] = None) -> None:
        self._loop.set_waist(yaw_rad, pitch_rad)

    def hold_waist(self) -> None:
        self._loop.hold_waist()

    def enable_teach(self, arm: Optional[str] = None) -> None:
        self._loop.enable_teach(arm)

    def disable_teach(self, arm: Optional[str] = None) -> None:
        self._loop.disable_teach(arm)

    def set_gravity_ff(self, on: bool) -> None:
        self._loop.set_gravity_ff(on)

    def set_gravity_imu(self, on: bool) -> None:
        self._loop.set_gravity_imu(on)

    def gravity_up(self) -> List[float]:
        """躯干系里实测的「竖直向上」（躯干 IMU 不可用时为躯干 +z）：「可平转」工作姿态的转轴。"""
        return [float(v) for v in self._loop.gravity.up_dir()]

    def set_max_speed(self, v: float) -> float:
        """改限速（ArmLoop 钳在 0.02–1.0 rad/s），返回生效的值。"""
        self._loop.set_max_speed(v)
        return self._loop.max_speed

    @property
    def collision_guard(self):
        return self._loop.collision_guard

    def set_collision_guard(self, guard) -> None:
        """装上自碰撞闸门（None = 不查）。"""
        self._loop.collision_guard = guard

    def shutdown(self, timeout_s: float = 30.0) -> None:
        """进程退出时：释放（需要过渡的后端等过渡结束）并停掉控制循环。"""
        self._loop.shutdown(timeout_s)

    # ------------------------------------------------------------ 租约
    def lease(self, resources: Sequence[Resource], owner: str, *,
              heartbeat_ms: Optional[int] = None, max_speed_rad_s: Optional[float] = None) -> "CoreLease":
        """申请资源。返回的租约本身是上下文管理器（with 结束时归还）；不能给时抛 MotionRejected。

        max_speed_rad_s：租约期间这几只手用的限速（D3：各程序用自己配置的限速）；None = 全局限速（页面滑条）。"""
        req = LeaseRequest(resources=tuple(resources), owner=owner, heartbeat_ms=heartbeat_ms,
                           max_speed_rad_s=max_speed_rad_s)
        return self.acquire(req)

    def lease_arm(self, arm: str, owner: str) -> "CoreLease":
        return self.lease([resource_for_arm(arm)], owner)

    def acquire(self, req: LeaseRequest) -> "CoreLease":
        if req.heartbeat_ms is not None:
            raise rejection(_reject(0, Reason.UNSUPPORTED, "进程内运控核心不检查心跳（阶段 2 才有）"))
        arms = []
        for res in req.resources:
            arm = ARM_RESOURCES.get(res)
            if arm is None or arm not in self._loop.arm_names:
                raise rejection(_reject(0, Reason.UNSUPPORTED, f"运控核心 v1 不管 {res.value}"))
            arms.append(arm)
        st = self._loop.status()
        with self._lock:
            if self._latched:
                raise rejection(_reject(0, Reason.ESTOP_LATCHED, self._latched))
            if self._transition:
                raise rejection(_reject(0, Reason.NOT_READY, "正在接管 / 释放，稍等"))
            if not st["engaged"]:
                raise rejection(_reject(0, Reason.NOT_ENGAGED, "先接管"))
            if not st["ready"]:
                why = st.get("state_stale") or f"指令出口没就绪：{st.get('transition') or '过渡中'}"
                raise rejection(_reject(0, Reason.NOT_READY, why))
            n = next(self._ids)
            lease = Lease(lease_id=f"L{n}", epoch=self._epoch, resources=tuple(req.resources), owner=req.owner)
            rec = _LeaseRec(lease=lease, token=f"{req.owner}#{n}", arms=tuple(arms), max_speed=req.max_speed_rad_s)
            taken: List[str] = []
            try:
                for arm in arms:
                    if arm in self._by_arm:
                        holder = self._leases[self._by_arm[arm]].token
                        raise PermissionErrorMotion(f"{side_label(arm)}臂正被「{holder}」使用，等它结束或先急停")
                    self._loop.acquire_owner(arm, rec.token, max_speed=req.max_speed_rad_s)
                    taken.append(arm)
            except (PermissionErrorMotion, ValueError) as exc:
                for arm in taken:
                    self._loop.release_owner(arm, rec.token)
                if isinstance(exc, ValueError):  # 租约带的限速超出核心的钳位
                    raise rejection(_reject(0, Reason.INVALID, str(exc))) from None
                reason = Reason.ESTOP_LATCHED if self._loop.inhibited else Reason.BUSY
                raise rejection(_reject(0, reason, str(exc))) from None
            self._leases[lease.lease_id] = rec
            for arm in arms:
                self._by_arm[arm] = lease.lease_id
        return CoreLease(self, rec)

    def _return(self, rec: _LeaseRec) -> None:
        """归还租约（with 结束 / release()）。stream 目标不能比租约活得久：还在跟的就地停住。"""
        hold = []
        with self._lock:
            if rec.returned:
                return
            rec.returned = True
            if self._leases.pop(rec.lease.lease_id, None) is not None:
                for arm in rec.arms:
                    if self._by_arm.get(arm) == rec.lease.lease_id:
                        del self._by_arm[arm]
                    dl = self._deadlines.get(arm)
                    if dl is not None and dl[2] == rec.lease.lease_id:
                        del self._deadlines[arm]
                        if dl[1] == "stream":
                            hold.append(arm)
                    self._loop.release_owner(arm, rec.token)
                self._remember_gone(rec.lease.lease_id, "已归还")
        rec.cancelled.set()
        for arm in hold:
            self._loop.hold_at_command(arm)

    def _remember_gone(self, lease_id: str, why: str) -> None:
        self._gone[lease_id] = why
        while len(self._gone) > _GONE_KEEP:
            self._gone.pop(next(iter(self._gone)))

    def _revoke_locked(self, reason: str) -> List[_LeaseRec]:
        """调用方持有 _lock。epoch 加一、清空租约表；返回被撤销的租约（事件由调用方在放锁后置位）。"""
        self._epoch += 1
        recs = list(self._leases.values())
        for rec in recs:
            rec.revoked = reason
            self._remember_gone(rec.lease.lease_id, f"已撤销：{reason}")
        self._leases.clear()
        self._by_arm.clear()
        self._deadlines.clear()
        self._last_revoke = {
            "reason": reason,
            "epoch": self._epoch,
            "owners": [r.token for r in recs],
            "t_ns": self._clock(),
        }
        return recs

    def revoke_lease(self, lease_id: str, reason: str) -> bool:
        """只撤销这一个租约（持有它的连接断了 / 心跳超时）：它的手停在指令角，不动别的租约、不改 epoch。

        返回 False = 没有这个租约（已归还或已撤销）。"""
        with self._lock:
            rec = self._leases.pop(lease_id, None)
            if rec is None:
                return False
            rec.revoked = reason
            self._remember_gone(lease_id, f"已撤销：{reason}")
            for arm in rec.arms:
                if self._by_arm.get(arm) == lease_id:
                    del self._by_arm[arm]
                dl = self._deadlines.get(arm)
                if dl is not None and dl[2] == lease_id:
                    del self._deadlines[arm]
                self._loop.release_owner(arm, rec.token)
            self._last_revoke = {"reason": reason, "epoch": self._epoch, "owners": [rec.token], "t_ns": self._clock()}
        for arm in rec.arms:
            self._loop.hold_at_command(arm)
        rec.cancelled.set()
        return True

    def hold_manual(self, what: str) -> None:
        """停住手动来源动着的手或腰（发令的那个连接没了）。被租约占着的手不动（它归任务管）；
        拖拽中的手退出拖拽、全刚度锁在实测角；其它停在指令角。腰：锁在实测角（同页面「停腰」）。"""
        if what == "waist":
            try:
                self._loop.hold_waist()
            except PermissionErrorMotion:
                pass  # 没接管：没有要停的腰
            return
        with self._lock:
            if what in self._by_arm:
                return
        if self._loop.arm_status(what)["float"]:
            self._loop.disable_teach(what)
        else:
            self._loop.hold_at_command(what)

    def revoke(self, reason: str) -> int:
        """撤销全部租约（不锁存）；被撤销的手停在指令角。返回撤销了几个。"""
        with self._lock:
            recs = self._revoke_locked(reason)
            self._loop.revoke_owners()
        for rec in recs:
            for arm in rec.arms:
                self._loop.hold_at_command(arm)
            rec.cancelled.set()
        return len(recs)

    # ------------------------------------------------------------ 整机：接管 / 释放 / 急停
    def engage(self) -> None:
        """重新接管会把两手重置到实测角：先撤销全部租约，接管过程中不发新租约。"""
        with self._lock:
            recs = self._revoke_locked("重新接管")
            self._transition = True
        for rec in recs:
            rec.cancelled.set()
        try:
            self._loop.engage()
        finally:
            with self._lock:
                self._transition = False

    def release(self) -> None:
        with self._lock:
            recs = self._revoke_locked("释放")
            self._transition = True
        for rec in recs:
            rec.cancelled.set()
        try:
            self._loop.release()
        finally:
            with self._lock:
                self._transition = False

    def estop(self, reason: str = "") -> None:
        """急停锁存：撤销全部租约、两手锁在实测角（退出拖拽）、禁止一切新运动，直到 reset_estop()。

        有外部急停（守护层）时随后也锁它；它失败（连不上、没应答）时抛出，但核心这边已经锁存。"""
        text = "急停已锁定" + (f"（{reason}）" if reason else "") + "：确认现场安全后点「复位急停」"
        with self._lock:
            self._latched = text
            recs = self._revoke_locked("急停")
            # 撤销占用、禁止新运动、锁角度在 ArmLoop 的同一把锁里完成
            self._loop.estop_hold(latch=text)
        for rec in recs:
            rec.cancelled.set()
        if self._external_estop is not None:
            self._external_estop(reason)

    @property
    def has_external_estop(self) -> bool:
        return self._external_estop is not None

    def reset_external_estop(self) -> None:
        """复位守护层的急停 / 故障锁存（现场确认安全后）；之后要重新接管。不解除核心的锁存。"""
        if self._external_reset is None:
            raise PermissionErrorMotion("当前后端不经守护层，没有守护层可复位")
        self._external_reset()

    def reset_estop(self) -> None:
        """现场确认安全后解除急停锁存。不碰守护层（守护层的锁存另有复位）。"""
        with self._lock:
            self._latched = None
            self._loop.clear_inhibit()

    @property
    def latched(self) -> Optional[str]:
        return self._latched

    # ------------------------------------------------------------ 指令
    def _lookup(self, lease_id: str, epoch: int, seq: int) -> Tuple[Optional[_LeaseRec], Optional[Ack]]:
        """调用方持有 _lock。"""
        rec = self._leases.get(lease_id)
        if rec is None:
            gone = self._gone.get(lease_id)
            if gone is not None and gone.startswith("已撤销"):
                return None, _reject(seq, Reason.STALE_EPOCH, f"租约{gone}，这一步不执行")
            if gone is not None:
                return None, _reject(seq, Reason.UNKNOWN_LEASE, f"租约{gone}")
            return None, _reject(seq, Reason.UNKNOWN_LEASE, "没有这个租约")
        if epoch != self._epoch or epoch != rec.lease.epoch:
            return None, _reject(seq, Reason.STALE_EPOCH, "租约的 epoch 已过期，这一步不执行")
        if rec.holder_cancelled:
            return None, _reject(seq, Reason.CANCELLED, "已停止")
        return rec, None

    def _arm_of(self, rec: _LeaseRec, resource: Resource, seq: int) -> Tuple[Optional[str], Optional[Ack]]:
        arm = ARM_RESOURCES.get(resource)
        if arm is None or arm not in rec.arms:
            return None, _reject(seq, Reason.INVALID, f"这个租约不含 {resource.value}")
        return arm, None

    def _refusal(self, rec: _LeaseRec, arm: str, seq: int, exc: Exception) -> Ack:
        """ArmLoop 拒绝了目标：按当时的状态归类原因。"""
        from robot_platform.collision.guard import CollisionBlocked

        if isinstance(exc, CollisionBlocked):
            return _reject(seq, Reason.COLLISION, str(exc))
        if isinstance(exc, ValueError):
            return _reject(seq, Reason.INVALID, str(exc))
        if rec.revoked or rec.lease.epoch != self._epoch:
            return _reject(seq, Reason.STALE_EPOCH, str(exc))
        if self._loop.inhibited:
            return _reject(seq, Reason.ESTOP_LATCHED, str(exc))
        st = self._loop.status()
        if not st["engaged"]:
            return _reject(seq, Reason.NOT_ENGAGED, str(exc))
        if st["arms"][arm]["float"]:
            return _reject(seq, Reason.BUSY, str(exc))
        if not st["ready"]:
            return _reject(seq, Reason.NOT_READY, str(exc))
        return _reject(seq, Reason.BUSY, str(exc))

    def _set_target(self, rec: _LeaseRec, arm: str, seq: int, q: Sequence[float], *, detour: bool,
                    deadline: Optional[Tuple[int, str]]) -> Ack:
        arr = np.asarray(q, dtype=float)
        lim = self._loop.limits(arm)
        clamped = arr.shape == (lim.shape[0],) and bool(np.any(arr < lim[:, 0]) or np.any(arr > lim[:, 1]))
        try:
            self._loop.set_target(arr, arm=arm, owner=rec.token, detour=detour)
        except (PermissionErrorMotion, ValueError) as exc:
            return self._refusal(rec, arm, seq, exc)
        with self._lock:
            if self._leases.get(rec.lease.lease_id) is rec:
                if deadline is None:
                    self._deadlines.pop(arm, None)
                else:
                    self._deadlines[arm] = (deadline[0], deadline[1], rec.lease.lease_id)
        if clamped:
            return Ack(seq=seq, verdict=Verdict.CLAMPED, reason=Reason.LIMIT, detail="目标超出关节限位，已夹到限位")
        return Ack(seq=seq, verdict=Verdict.ACCEPTED)

    def submit_goal(self, req: GoalRequest) -> Ack:
        now = self._clock()
        with self._lock:
            rec, ack = self._lookup(req.lease_id, req.epoch, req.seq)
            if ack is not None:
                return ack
            arm, ack = self._arm_of(rec, req.resource, req.seq)
            if ack is not None:
                return ack
        if len(req.path) != 1:
            return _reject(req.seq, Reason.UNSUPPORTED, "v1 的到位目标只收一个点（绕行由核心规划）")
        deadline = None if req.timeout_ms is None else (now + req.timeout_ms * 1_000_000, "goal")
        return self._set_target(rec, arm, req.seq, req.path[0], detour=req.allow_detour, deadline=deadline)

    def submit_stream(self, sp: StreamSetpoint) -> Ack:
        now = self._clock()  # 有效期从核心收到时算
        with self._lock:
            rec, ack = self._lookup(sp.lease_id, sp.epoch, sp.seq)
            if ack is not None:
                return ack
            arm, ack = self._arm_of(rec, sp.resource, sp.seq)
            if ack is not None:
                return ack
        if sp.speed_scale != 1.0:
            return _reject(sp.seq, Reason.UNSUPPORTED, "v1 不支持按目标降速（speed_scale 只能是 1）")
        obs = sp.derived_from
        if obs is not None:
            if obs.host != self.host:
                return _reject(sp.seq, Reason.STALE_OBSERVATION, f"观测时间戳来自 {obs.host}，v1 只认本机（{self.host}）")
            if self._max_obs_ns is not None and now - obs.t_ns - obs.uncertainty_ns > self._max_obs_ns:
                return _reject(sp.seq, Reason.STALE_OBSERVATION, "观测太旧")
        return self._set_target(rec, arm, sp.seq, sp.q, detour=False,
                                deadline=(now + sp.valid_for_ms * 1_000_000, "stream"))

    def hold(self, lease_id: str, resource: Resource, seq: int = 0) -> Ack:
        """停在当前实测角（任务自己中止时用）。租约已撤销 / 归还时什么都不做（核心撤销时已经停过）。"""
        with self._lock:
            rec = self._leases.get(lease_id)
            if rec is None:
                return _reject(seq, Reason.STALE_EPOCH, "租约已失效，不再动这只手")
            arm, ack = self._arm_of(rec, resource, seq)
            if ack is not None:
                return ack
            self._deadlines.pop(arm, None)
        self._loop.hold_arm(arm)
        return Ack(seq=seq, verdict=Verdict.ACCEPTED)

    # ------------------------------------------------------------ 每拍（ArmLoop 线程里调用）
    def _on_cycle(self, engaged: bool, ready: bool) -> None:
        now = self._clock()
        self._record(now)
        hold: List[str] = []
        recs: List[_LeaseRec] = []
        with self._lock:
            if engaged and not ready and self._leases:
                st = self._loop.status()
                why = st.get("state_stale") or f"指令出口没就绪：{st.get('transition') or '未知原因'}"
                recs = self._revoke_locked(why)
                self._loop.revoke_owners()
                hold = [arm for rec in recs for arm in rec.arms]
            for arm, (t_end, _kind, _lid) in list(self._deadlines.items()):
                if now >= t_end:
                    del self._deadlines[arm]
                    hold.append(arm)
        for arm in hold:
            self._loop.hold_at_command(arm)
        for rec in recs:
            rec.cancelled.set()

    def _record(self, now: int) -> None:
        reader = self._loop.reader
        age = float(reader.age_s())
        if not np.isfinite(age) or age < 0:
            return
        snap = reader.snapshot()
        cmds = self._loop.command_snapshot()
        q_meas = {}
        for arm in self._loop.arm_names:
            idx = self._loop.layout(arm).motor_indices
            q_meas[arm] = tuple(float(snap.q[i]) for i in idx)
        sample = _Sample(t_meas=now - int(age * 1e9), t_cmd=now, q_meas=q_meas,
                         q_cmd={arm: tuple(float(v) for v in q) for arm, q in cmds.items()})
        self._push(sample)

    def _push(self, sample: _Sample) -> None:
        with self._hist_lock:
            self._hist.append(sample)
            cutoff = sample.t_cmd - self._history_ns
            while self._hist and self._hist[0].t_cmd < cutoff:
                self._hist.popleft()

    # ------------------------------------------------------------ 状态
    def safety(self, loop_status: Optional[dict] = None) -> SafetyState:
        """loop_status：调用方刚取过的 ArmLoop.status()（省一次）。"""
        if self._latched:
            return SafetyState.ESTOP_LATCHED
        st = loop_status if loop_status is not None else self._loop.status()
        if not st["engaged"]:
            return SafetyState.RELEASED
        info = st.get("commander") or {}
        latched = list(info.get("latched") or [])
        if "link_lost" in latched:
            return SafetyState.LINK_LOST
        if "stale_lowstate" in latched or st.get("state_stale"):
            return SafetyState.STALE_LOWSTATE
        if info.get("state") == "estop":
            return SafetyState.ESTOP_LATCHED
        if not st["ready"]:
            return SafetyState.HOLD
        return SafetyState.NORMAL

    def _owner_of(self, arm: str) -> Optional[str]:
        lid = self._by_arm.get(arm)
        return None if lid is None else self._leases[lid].lease.owner

    def state(self, resource: Resource) -> MotionState:
        arm = self._arm_for(resource)
        now = self._clock()
        q_meas = tuple(float(v) for v in self._loop.arm_sample(arm)["q_meas"])
        q_cmd = tuple(float(v) for v in self._loop.command_snapshot()[arm])
        with self._lock:
            owner, epoch = self._owner_of(arm), self._epoch
        return MotionState(stamp=Stamp(t_ns=now, host=self.host), resource=Resource(resource), q_meas=q_meas,
                           q_cmd=q_cmd, lease_owner=owner, safety=self.safety(), epoch=epoch)

    def state_at(self, resource: Resource, stamp: Stamp) -> MotionState:
        """stamp 那一刻的实测角 / 指令角（按历史线性插值）。"""
        arm = self._arm_for(resource)
        if stamp.host != self.host:
            raise rejection(_reject(0, Reason.INVALID, f"时间戳来自 {stamp.host}，v1 只认本机（{self.host}）"))
        with self._hist_lock:
            hist = list(self._hist)
        if not hist:
            raise rejection(_reject(0, Reason.NOT_READY, "还没有状态历史"))
        tol = STAMP_TOLERANCE_MS * 1_000_000
        t = int(stamp.t_ns)
        if t < hist[0].t_meas - tol:
            raise rejection(_reject(0, Reason.STALE_OBSERVATION,
                                    f"时间戳早于状态历史（只保留约 {self._history_ns / 1e9:.1f} s）"))
        if t > hist[-1].t_cmd + tol:
            raise rejection(_reject(0, Reason.INVALID, "时间戳比最新状态还新"))
        q_meas = _interp([s.t_meas for s in hist], [s.q_meas[arm] for s in hist], t)
        q_cmd = _interp([s.t_cmd for s in hist], [s.q_cmd[arm] for s in hist], t)
        with self._lock:
            owner, epoch = self._owner_of(arm), self._epoch
        return MotionState(stamp=stamp, resource=Resource(resource), q_meas=q_meas, q_cmd=q_cmd,
                           lease_owner=owner, safety=self.safety(), epoch=epoch)

    def _arm_for(self, resource: Resource) -> str:
        arm = ARM_RESOURCES.get(Resource(resource))
        if arm is None or arm not in self._loop.arm_names:
            raise rejection(_reject(0, Reason.UNSUPPORTED, f"运控核心 v1 不管 {Resource(resource).value}"))
        return arm

    def status(self, loop_status: Optional[dict] = None) -> Dict[str, Any]:
        """给页面看的运控核心状态。loop_status 同 safety()。"""
        with self._lock:
            leases = [
                {"lease_id": r.lease.lease_id, "owner": r.lease.owner, "token": r.token,
                 "resources": [x.value for x in r.lease.resources], "epoch": r.lease.epoch,
                 "cancelled": r.holder_cancelled, "max_speed_rad_s": r.max_speed}
                for r in self._leases.values()
            ]
            last = dict(self._last_revoke) if self._last_revoke else None
            epoch, latched = self._epoch, self._latched
        if last is not None:
            last["age_s"] = round((self._clock() - last.pop("t_ns")) / 1e9, 1)
        return {
            "epoch": epoch,
            "latched": latched is not None,
            "latch_reason": latched,
            "safety": self.safety(loop_status).value,
            "leases": leases,
            "last_revoke": last,
            # 急停同时锁守护层（复位要两步：守护层、核心各一次）
            "external_estop": self._external_estop is not None,
        }


def _interp(ts: List[int], values: List[Tuple[float, ...]], t: int) -> Tuple[float, ...]:
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return tuple(values[0])
    if i >= len(ts):
        return tuple(values[-1])
    t0, t1 = ts[i - 1], ts[i]
    if t1 <= t0:
        return tuple(values[i])
    w = (t - t0) / (t1 - t0)
    a, b = np.asarray(values[i - 1], float), np.asarray(values[i], float)
    return tuple(float(v) for v in a + w * (b - a))


class CoreLease:
    """进程内的租约句柄（实现 g1_contracts.motion.LeaseHandle），外加单臂便利方法。

    被拒绝的 goal / stream 抛 MotionRejected（租约失效时是 LeaseRevoked）；hold 不抛。
    只含一只手的租约可以省略 resource，用 move_to / status / arm_sample / layout / limits。
    """

    def __init__(self, core: MotionCore, rec: _LeaseRec) -> None:
        self._core = core
        self._rec = rec
        self._seq = itertools.count(1)

    # ---- 协议
    @property
    def lease(self) -> Lease:
        return self._rec.lease

    @property
    def owner(self) -> str:
        return self._rec.token

    @property
    def cancelled(self) -> threading.Event:
        return self._rec.cancelled

    @property
    def revoked(self) -> Optional[str]:
        """撤销原因（没撤销为 None）。"""
        return self._rec.revoked

    @property
    def returned(self) -> bool:
        return self._rec.returned

    @property
    def holder_cancelled(self) -> bool:
        return self._rec.holder_cancelled

    def check(self) -> None:
        rec = self._rec
        if rec.revoked:
            raise rejection(_reject(0, Reason.STALE_EPOCH, f"租约已撤销：{rec.revoked}"))
        if rec.returned:
            raise rejection(_reject(0, Reason.UNKNOWN_LEASE, "租约已归还"))
        if rec.holder_cancelled:
            raise rejection(_reject(0, Reason.CANCELLED, "已停止"))

    def cancel(self) -> None:
        self._rec.holder_cancelled = True
        self._rec.cancelled.set()

    def goal(self, resource: Resource, path: Sequence[Sequence[float]], *,
             timeout_ms: Optional[int] = None, allow_detour: bool = False) -> Ack:
        seq = next(self._seq)
        try:
            req = GoalRequest(lease_id=self.lease.lease_id, epoch=self.lease.epoch, seq=seq, resource=resource,
                              path=tuple(tuple(p) for p in path), timeout_ms=timeout_ms, allow_detour=allow_detour)
        except (TypeError, ValueError) as exc:
            raise rejection(_reject(seq, Reason.INVALID, str(exc))) from None
        return self._raise_if_rejected(self._core.submit_goal(req))

    def stream(self, resource: Resource, q: Sequence[float], *, valid_for_ms: int = 200,
               derived_from: Optional[Stamp] = None, speed_scale: float = 1.0) -> Ack:
        seq = next(self._seq)
        try:
            sp = StreamSetpoint(lease_id=self.lease.lease_id, epoch=self.lease.epoch, seq=seq, resource=resource,
                                q=tuple(q), derived_from=derived_from, valid_for_ms=valid_for_ms,
                                speed_scale=speed_scale)
        except (TypeError, ValueError) as exc:
            raise rejection(_reject(seq, Reason.INVALID, str(exc))) from None
        return self._raise_if_rejected(self._core.submit_stream(sp))

    def hold(self, resource: Optional[Resource] = None) -> Ack:
        return self._core.hold(self.lease.lease_id, self._res(resource), next(self._seq))

    def release(self) -> None:
        self._core._return(self._rec)

    def __enter__(self) -> "CoreLease":
        return self

    def __exit__(self, *exc) -> None:
        self.release()

    @staticmethod
    def _raise_if_rejected(ack: Ack) -> Ack:
        if not ack.ok:
            raise rejection(ack)
        return ack

    # ---- 单臂便利方法（平台扩展，不在协议里）
    def _res(self, resource: Optional[Resource]) -> Resource:
        if resource is not None:
            return Resource(resource)
        if len(self.lease.resources) != 1:
            raise ValueError("租约含多个资源，要写明是哪一个")
        return self.lease.resources[0]

    def arm_name(self, resource: Optional[Resource] = None) -> str:
        return ARM_RESOURCES[self._res(resource)]

    def move_to(self, q: Sequence[float], *, timeout_ms: Optional[int] = None, allow_detour: bool = False,
                resource: Optional[Resource] = None) -> Ack:
        return self.goal(self._res(resource), [q], timeout_ms=timeout_ms, allow_detour=allow_detour)

    def status(self, resource: Optional[Resource] = None) -> dict:
        return self._core.arm_view_status(self.arm_name(resource))

    def arm_sample(self, wait: bool = False, resource: Optional[Resource] = None) -> dict:
        return self._core.arm_sample(self.arm_name(resource), wait=wait)

    def arm_gains(self, resource: Optional[Resource] = None) -> tuple:
        return self._core.arm_gains(self.arm_name(resource))

    @property
    def layout(self):
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
        """换负载补偿（到位对比轮换方案用）。这是前馈配置而不是运动目标：租约失效后也允许，
        这样任务被急停打断时还能在 finally 里把原来的补偿换回去（和以前一样）。"""
        self._core.set_payload_theta(theta, name, arm=self.arm_name(resource))
