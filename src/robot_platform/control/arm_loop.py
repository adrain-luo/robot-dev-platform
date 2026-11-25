"""双臂控制循环：50–100 Hz 保持 + 限速逼近目标，接管后经后端 Commander 周期发整机指令。

两只手各有一套状态（目标角、指令角、拖拽、负载补偿、占用者），互不等待：
一只手拖拽时另一只照常受控，一只手在走时可以给另一只发新目标。
接管 / 释放 / 急停 / 腰仍是整机的（arm_sdk 的权重、lowcmd 的整帧都是整机共用）。
"""

from __future__ import annotations

import threading
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Callable, Dict, Iterator, List, Optional, Sequence

import numpy as np

# PermissionErrorMotion 定义在后端接口里（后端安全检查也抛它），其它模块仍从这里导入
from robot_platform.hw.backends.base import Commander, PermissionErrorMotion, StateSource  # noqa: F401
from robot_platform.control.arm_gravity import ArmGravity
from robot_platform.control.gravity_dir import GravityDirection
from robot_platform.control.fk_scene import RobotKinematics
from robot_platform.core.layout import ArmLayout
from robot_platform.core.robot_profile import active as active_profile

TAU_FF_LIMIT = 8.0
TEACH_KD = 2.0


def _clamp_span(value: float, span: tuple) -> float:
    lo, hi = span
    if not np.isfinite(value):
        raise ValueError("腰部角度必须是有限数")
    return float(min(hi, max(lo, value)))


def side_label(arm: str) -> str:
    return "左" if arm == "left" else "右"


def _stale_age_text(age_s: float, limit_s: float) -> str:
    if not np.isfinite(age_s):
        return "没有收到 lowstate"
    return f"lowstate 已 {age_s * 1000:.0f} ms 没更新（上限 {limit_s * 1000:.0f} ms）"


class _ArmState:
    """一只手的控制状态（读写都在 ArmLoop._lock 里）。"""

    def __init__(self, layout: ArmLayout, limits: np.ndarray, grav: ArmGravity, q0: np.ndarray) -> None:
        self.layout = layout
        self.limits = limits
        self.grav = grav
        self.cmd_q = q0.copy()
        self._desired = q0.copy()
        # 绕行的剩余中间点（desired_q 是当前这一段的终点，走到就接下一个）；任何地方改 desired_q 都会清掉
        self.path: List[np.ndarray] = []
        self.float = False
        self.payload_theta: Optional[np.ndarray] = None
        self.payload_name: Optional[str] = None
        self.last_tau_ff = np.zeros(layout.n)
        self.owner: Optional[str] = None

    @property
    def desired_q(self) -> np.ndarray:
        return self._desired

    @desired_q.setter
    def desired_q(self, q: np.ndarray) -> None:
        self._desired = q
        self.path = []

    def set_path(self, waypoints: Sequence[np.ndarray]) -> None:
        """按折线走：先去第一个点，到了再接下一个。"""
        pts = [self.clamp(np.asarray(p, dtype=float)) for p in waypoints]
        self._desired = pts[0]
        self.path = pts[1:]

    def next_waypoint(self) -> None:
        """当前段走完了就切到下一个中间点。"""
        if self.path and float(np.max(np.abs(self._desired - self.cmd_q))) <= 1e-9:
            self._desired = self.path.pop(0)

    def clamp(self, q: np.ndarray) -> np.ndarray:
        lo, hi = self.limits[:, 0], self.limits[:, 1]
        return np.minimum(np.maximum(q, lo), hi)


class ArmLoop:
    def __init__(
        self,
        *,
        reader: StateSource,
        commander: Commander,
        kin: RobotKinematics,
        max_speed_rad_s: float = 0.2,
        rate_hz: float = 100.0,
        urdf_path: Optional[Path] = None,
        base_link: str = "torso_link",
        torso_imu=None,
        lowstate_max_age_s: Optional[float] = None,
    ) -> None:
        if lowstate_max_age_s is not None and not (np.isfinite(lowstate_max_age_s) and lowstate_max_age_s > 0):
            raise ValueError("lowstate_max_age_s 必须是正数（或 None = 不检查）")
        self.reader = reader
        self._commander = commander
        self.kin = kin
        self.profile = active_profile()
        self.max_speed = float(np.clip(max_speed_rad_s, 0.02, 1.0))
        self._dt = 1.0 / float(rate_hz)
        self._loop_period = self._dt  # 实测循环周期（指数平均），给页面看实际频率
        self.kp_body = list(self.profile.kp)
        self.kd_body = list(self.profile.kd)
        self._waist = self.profile.waist
        self._waist_motors = self._waist.hold_motors if self._waist else ()
        urdf = Path(urdf_path) if urdf_path is not None else self.kin.urdf_path
        self._grav_on = True
        # 重力方向：默认躯干竖直；机型有躯干 IMU 时可按实测（见 control/gravity_dir.py）
        self.gravity = GravityDirection(torso_imu, self.profile.torso_imu)
        self._waist_cmd: Optional[dict] = None
        self._waist_target: Optional[dict] = None
        self._lock = threading.Lock()
        # 发帧与接管互斥：接管时先重置指令角、再让指令出口启用新一轮接管，中间不会夹进一帧旧指令
        self._publish_lock = threading.Lock()
        self._arms: Dict[str, _ArmState] = {}
        for layout in self.profile.arms.values():
            q0 = np.asarray(reader.read_arm(layout).q, dtype=float)
            limits = self.kin.arms[layout.name].model.joint_limits()
            self._arms[layout.name] = _ArmState(layout, limits, ArmGravity(layout.name, urdf, base_link), q0)
        self._engaged = False
        # 运动禁止（急停锁存）：非空时拒绝新目标、动腰、拖拽、占用；保持、接管、释放照常。
        # 由运控核心（control/motion_core.py）置位 / 清除；检查都在 _lock 里，和设目标不会交错
        self._inhibit: Optional[str] = None
        # lowstate 过期门控（直写后端用；守护层后端由本体上的守护层自己查，传 None）：
        # 已接管时 lowstate 超过这么久没更新，就不再跟目标，两只手停在当前指令角
        # （拖拽中的手退出拖拽、恢复全刚度），占用全部撤销、新运动一律拒绝，帧照发不停。
        # lowstate 恢复后照旧保持，不会自己接着走，要重新下目标。
        self._state_max_age = None if lowstate_max_age_s is None else float(lowstate_max_age_s)
        self._stale: Optional[str] = None  # 过期原因（None = 没过期）；读写在 _lock 里
        # 每拍结束后调用一次 on_cycle(engaged, ready)（运控核心记状态历史、查设定有效期）。
        # 它出错只记 last_error，不影响发帧
        self.on_cycle: Optional[Callable[[bool, bool], None]] = None
        # 自碰撞闸门（collision/guard.py）：由 app 装上；None 时不查
        self.collision_guard = None
        self._stop = threading.Event()
        self._last_err: Optional[str] = None
        self._thread = threading.Thread(target=self._loop, name="arm-loop", daemon=True)
        self._thread.start()

    # ------------------------------------------------------------ 手臂
    @property
    def arm_names(self) -> List[str]:
        return list(self._arms)

    def _arm(self, arm: str) -> _ArmState:
        a = self._arms.get(str(arm))
        if a is None:
            raise ValueError(f"没有 {arm!r} 臂（可选 {', '.join(self._arms)}）")
        return a

    def layout(self, arm: str) -> ArmLayout:
        return self._arm(arm).layout

    def limits(self, arm: str) -> np.ndarray:
        return self._arm(arm).limits

    def _measured(self, a: _ArmState) -> np.ndarray:
        return np.asarray(self.reader.read_arm(a.layout).q, dtype=float)

    def set_max_speed(self, v: float) -> None:
        self.max_speed = float(np.clip(v, 0.02, 1.0))

    # ------------------------------------------------------------ 占用
    def owner(self, arm: str) -> Optional[str]:
        with self._lock:
            return self._arm(arm).owner

    def acquire_owner(self, arm: str, owner: str) -> None:
        """占用一只手：之后只有带同一个 owner 的目标能下到这只手。"""
        a = self._arm(arm)
        with self._lock:
            self._refuse_if_blocked_locked()
            if a.owner is not None and a.owner != owner:
                raise PermissionErrorMotion(f"{side_label(arm)}臂正被「{a.owner}」使用，等它结束或先急停")
            if a.float:
                raise PermissionErrorMotion(f"{side_label(arm)}臂在拖拽，先退出拖拽")
            a.owner = owner

    def release_owner(self, arm: str, owner: str) -> None:
        """归还占用（已经被撤销或换了占用者时什么都不做）。"""
        a = self._arm(arm)
        with self._lock:
            if a.owner == owner:
                a.owner = None

    def revoke_owners(self) -> Dict[str, str]:
        """撤销所有占用；返回 {手: 原占用者}。之后旧占用者再下目标会被拒绝。"""
        with self._lock:
            out = {name: a.owner for name, a in self._arms.items() if a.owner is not None}
            for a in self._arms.values():
                a.owner = None
        return out

    @contextmanager
    def claim(self, arm: str, owner: str) -> Iterator[None]:
        """任务独占一只手：期间别的来源给这只手设目标会被拒绝；退出时自动归还。"""
        self.acquire_owner(arm, owner)
        try:
            yield
        finally:
            self.release_owner(arm, owner)

    # ------------------------------------------------------------ 运动禁止（急停锁存）
    @property
    def inhibited(self) -> Optional[str]:
        return self._inhibit

    def clear_inhibit(self) -> None:
        with self._lock:
            self._inhibit = None

    @property
    def state_stale(self) -> Optional[str]:
        """lowstate 过期的原因（None = 没过期或不检查）。"""
        return self._stale

    def _refuse_if_blocked_locked(self) -> None:
        """急停锁存或 lowstate 过期时拒绝新运动（调用方持有 _lock）。"""
        if self._inhibit:
            raise PermissionErrorMotion(self._inhibit)
        if self._stale:
            raise PermissionErrorMotion(self._stale)

    def _state_age_s(self) -> float:
        try:
            age = float(self.reader.age_s())
        except Exception:  # noqa: BLE001  读不到年龄按过期处理
            return float("inf")
        return age if np.isfinite(age) and age >= 0 else float("inf")

    @property
    def engaged(self) -> bool:
        return self._engaged

    # ------------------------------------------------------------ 腰
    def _read_waist(self) -> dict:
        q = self.reader.snapshot().q
        return {idx: float(q[idx]) for idx in self._waist_motors}

    def _slew_waist(self, dt: Optional[float] = None) -> None:
        if not self._waist_cmd or not self._waist_target:
            return
        step = self.max_speed * (self._dt if dt is None else dt)
        for idx in self._waist_motors:
            cur = float(self._waist_cmd[idx])
            tgt = float(self._waist_target[idx])
            d = tgt - cur
            if abs(d) <= step:
                self._waist_cmd[idx] = tgt
            else:
                self._waist_cmd[idx] = cur + (step if d > 0 else -step)

    # ------------------------------------------------------------ 整机：接管 / 释放 / 急停
    def engage(self) -> None:
        """两只手一起接管：都锁在当前实测角（叠重力前馈），腰锁在当前角。"""
        if self._commander.busy() and not self._engaged:
            raise PermissionErrorMotion("上次释放还在过渡，稍等再接管")
        if self._state_max_age is not None:
            age = self._state_age_s()
            if age > self._state_max_age:
                raise PermissionErrorMotion(f"{_stale_age_text(age, self._state_max_age)}，不能接管")
        self._commander.prepare()
        with self._publish_lock:
            qs = {name: a.clamp(self._measured(a)) for name, a in self._arms.items()}
            waist_now = self._read_waist()
            with self._lock:
                for name, a in self._arms.items():
                    a.cmd_q = qs[name].copy()
                    a.desired_q = qs[name].copy()
                    a.float = False
                    a.owner = None
                self._waist_cmd = waist_now
                self._waist_target = dict(waist_now)
                self._stale = None
                self._engaged = True
            self._commander.activate()

    def release(self) -> None:
        # 拖拽中的手：先把指令角对齐实测，过渡期间按它保持，避免跳回拖拽前的旧指令
        qs = {name: a.clamp(self._measured(a)) for name, a in self._arms.items() if a.float}
        with self._lock:
            for name, a in self._arms.items():
                if name in qs:
                    a.cmd_q = qs[name].copy()
                    a.desired_q = qs[name].copy()
                a.float = False
                a.owner = None
            self._engaged = False
        self._commander.release()
        # 需要过渡的后端（arm_sdk）在过渡期间继续按最后的腰 / 臂角保持，结束后再清
        if not self._commander.busy():
            self._clear_waist()

    def _clear_waist(self) -> None:
        with self._lock:
            self._waist_cmd = None
            self._waist_target = None

    def _require_ready(self, what: str) -> None:
        if not self._commander.ready():
            raise PermissionErrorMotion(f"接管过渡中（{self._commander.phase()}），等完成再{what}")

    def estop_hold(self, *, latch: Optional[str] = None) -> None:
        """两只手都退出拖拽、锁在当前实测角；腰也锁住。

        latch 非空（急停锁存）：同一把锁里撤销所有占用、禁止新运动（原因 = latch），
        这样任何和急停交错的 set_target 要么被这里的保持覆盖，要么被拒绝。"""
        qs = {name: a.clamp(self._measured(a)) for name, a in self._arms.items()}
        waist_now = self._read_waist()
        with self._lock:
            if latch:
                self._inhibit = str(latch)
                for a in self._arms.values():
                    a.owner = None
            for name, a in self._arms.items():
                a.float = False
                a.cmd_q = qs[name].copy()
                a.desired_q = qs[name].copy()
            if self._waist_cmd is not None:
                self._waist_cmd = waist_now
                self._waist_target = dict(waist_now)

    def hold_arm(self, arm: str) -> None:
        """只停这一只手：锁在当前实测角（另一只手、腰、拖拽状态都不动）。任务自己中止时用。"""
        a = self._arm(arm)
        q = a.clamp(self._measured(a))
        with self._lock:
            if a.float:
                return
            a.cmd_q = q.copy()
            a.desired_q = q.copy()

    def hold_at_command(self, arm: str) -> None:
        """只停这一只手、停在当前指令角（不回跳到实测角）：取消没走完的目标和绕行点。

        设定过期、租约撤销时用：指令角就此不再前进，刚度照旧。"""
        a = self._arm(arm)
        with self._lock:
            if a.float:
                return
            a.desired_q = a.cmd_q.copy()

    def command_snapshot(self) -> Dict[str, np.ndarray]:
        """各手当前指令角（拷贝）。"""
        with self._lock:
            return {name: a.cmd_q.copy() for name, a in self._arms.items()}

    def set_waist(self, yaw_rad: Optional[float] = None, pitch_rad: Optional[float] = None) -> None:
        if self._waist is None:
            raise PermissionErrorMotion(f"{self.profile.name} 没有可控的腰")
        if not self._waist.movable:
            raise PermissionErrorMotion(f"{self.profile.name} 的腰只保持不转（站立时转腰会影响平衡）")
        self._require_ready("动腰")
        with self._lock:
            self._refuse_if_blocked_locked()
            if not self._engaged or self._waist_target is None:
                raise PermissionErrorMotion("先接管再动腰")
            if any(a.float for a in self._arms.values()):
                raise PermissionErrorMotion("先退出拖拽，再动腰")
            for role, value in (("yaw", yaw_rad), ("pitch", pitch_rad)):
                if value is None:
                    continue
                if not self._waist.has(role):
                    raise ValueError(f"{self.profile.name} 的腰没有{'俯仰' if role == 'pitch' else role}轴")
                j = self._waist.joint(role)
                self._waist_target[j.motor] = _clamp_span(float(value), j.limit_rad)

    def hold_waist(self) -> None:
        """停在当前实测角（取消未走完的腰目标）。"""
        waist_now = self._read_waist()
        with self._lock:
            if not self._engaged or self._waist_target is None:
                raise PermissionErrorMotion("先接管再停腰")
            self._waist_cmd = waist_now
            self._waist_target = dict(waist_now)

    # ------------------------------------------------------------ 单臂：拖拽 / 前馈 / 目标
    def _pick(self, arm: Optional[str]) -> List[_ArmState]:
        return list(self._arms.values()) if arm in (None, "", "both") else [self._arm(arm)]

    def enable_teach(self, arm: Optional[str] = None) -> None:
        """拖拽这只手（None = 两只都拖）：刚度 0 + 重力前馈；另一只手不受影响。"""
        self._require_ready("拖拽")
        picked = self._pick(arm)
        with self._lock:
            self._refuse_if_blocked_locked()
            if not self._engaged:
                raise PermissionErrorMotion("先接管再拖拽")
            for a in picked:
                if a.owner is not None:
                    raise PermissionErrorMotion(f"{side_label(a.layout.name)}臂正被「{a.owner}」使用，先急停再拖拽")
            for a in picked:
                a.float = True

    def disable_teach(self, arm: Optional[str] = None) -> None:
        """退出拖拽：锁在当前实测角。"""
        picked = self._pick(arm)
        qs = {a.layout.name: a.clamp(self._measured(a)) for a in picked}
        with self._lock:
            for a in picked:
                if not a.float:
                    continue
                a.float = False
                a.cmd_q = qs[a.layout.name].copy()
                a.desired_q = qs[a.layout.name].copy()

    def set_gravity_ff(self, on: bool) -> None:
        self._grav_on = bool(on)

    def set_gravity_imu(self, on: bool) -> None:
        """重力前馈按躯干 IMU 的重力方向算（开）或按躯干竖直算（关）；切换在约 0.5 s 内平滑过渡。"""
        self.gravity.set_enabled(on)

    def arm_gains(self, arm: str) -> tuple:
        idx = self._arm(arm).layout.motor_indices
        return [float(self.kp_body[i]) for i in idx], [float(self.kd_body[i]) for i in idx]

    def set_payload_theta(self, theta, name: Optional[str] = None, *, arm: str) -> None:
        a = self._arm(arm)
        with self._lock:
            if theta is None:
                a.payload_theta = None
                a.payload_name = None
            else:
                a.payload_theta = np.asarray(theta, float).reshape(4)
                a.payload_name = name

    def payload_theta(self, arm: str) -> Optional[list]:
        a = self._arm(arm)
        with self._lock:
            return None if a.payload_theta is None else a.payload_theta.tolist()

    def arm_sample(self, arm: str, wait: bool = False) -> dict:
        a = self._arm(arm)
        st = self.reader.read_arm(a.layout, wait=wait)
        kp, kd = self.arm_gains(arm)
        with self._lock:
            cmd = a.cmd_q.copy()
            tau_ff = a.last_tau_ff.copy()
        tau_est = [None if t is None else float(t) for t in st.tau_est]
        g = self.gravity.g_vec()
        return {
            "arm": a.layout.name,
            "q_meas": [float(v) for v in st.q],
            "dq": [float(v) for v in st.dq],
            "tau_est": tau_est,
            "q_cmd": cmd.tolist(),
            "tau_ff_sent": tau_ff.tolist(),
            "kp": kp,
            "kd": kd,
            # 这一刻前馈用的重力向量（base_link 系）；负载辨识按它解，换了重力方向也对得上
            "g_torso": [float(v) for v in g],
        }

    def set_target(self, q: Sequence[float], *, arm: str, owner: Optional[str] = None,
                   detour: bool = False) -> Optional[dict]:
        """给一只手设目标，另一只手不受影响。owner 是占用这只手的任务名（手动来源为 None）。

        带 owner 的目标只有在这只手当前正被同一个 owner 占用时才接受：占用被撤销
        （急停、重新接管、释放）之后，旧任务的下一步会被拒绝，而不是在手空着时照样下发。

        detour=True（「到位」类单次目标）：直线会碰时规划一条绕行折线，按中间点依次走；
        返回规划摘要（没绕行返回 None）。连续发的小步目标（点动、跟踪）不要开，规划要零点几秒到几秒。"""
        a = self._arm(arm)
        arr = np.asarray(q, dtype=float).reshape(-1)
        if arr.size != a.layout.n or not np.all(np.isfinite(arr)):
            raise ValueError(f"需要 {a.layout.n} 个有限关节角")
        if self._engaged:
            self._require_ready("走点")

        def _gate() -> None:
            self._refuse_if_blocked_locked()
            if not self._engaged:
                raise PermissionErrorMotion("先接管再设目标")
            if a.float:
                raise PermissionErrorMotion(f"{side_label(arm)}臂在拖拽，先退出拖拽再走点")
            if a.owner is not None and a.owner != owner:
                raise PermissionErrorMotion(f"{side_label(arm)}臂正被「{a.owner}」使用，等它结束或先急停")
            if owner is not None and a.owner != owner:
                raise PermissionErrorMotion(
                    f"「{owner}」对{side_label(arm)}臂的占用已撤销（急停、重新接管或释放），这一步不执行")

        target = a.clamp(arr)
        guard = self.collision_guard
        if guard is not None and guard.enabled:
            from robot_platform.collision.guard import CollisionBlocked

            # 自碰撞检查：从当前指令角沿关节直线走到目标；另一只手按它的指令角、目标角、剩余中间点各查一遍
            with self._lock:
                _gate()
                q_from = a.cmd_q.copy()
                others = {n: [o.cmd_q.copy(), o.desired_q.copy(), *[p.copy() for p in o.path]]
                          for n, o in self._arms.items() if n != arm}
            try:
                guard.check_move(arm, q_from, target, others)
            except CollisionBlocked as exc:
                if not (detour and guard.detour):
                    raise
                res = guard.plan_move(arm, q_from, target, others, a.limits)
                path = res.get("path")
                if not path:
                    raise CollisionBlocked(f"{exc}\n绕行也没找到：{res.get('reason') or '没有路线'}") from None
                with self._lock:
                    _gate()
                    if float(np.max(np.abs(a.cmd_q - q_from))) > 1e-3:
                        raise PermissionErrorMotion(f"规划绕行期间{side_label(arm)}臂动了，路线作废，请重新下目标")
                    a.set_path(path)
                return guard.last_plan
        with self._lock:
            _gate()
            a.desired_q = target
        return None

    # ------------------------------------------------------------ 状态
    def arm_status(self, arm: str) -> dict:
        a = self._arm(arm)
        with self._lock:
            return self._arm_status_locked(a)

    def _arm_status_locked(self, a: _ArmState) -> dict:
        cmd = a.cmd_q
        moving = bool(a.path) or bool(cmd.size and float(np.max(np.abs(a.desired_q - cmd))) > 1e-4)
        return {
            "arm": a.layout.name,
            "float": a.float,
            "owner": a.owner,
            "moving": moving,
            "cmd_rad": cmd.tolist(),
            "desired_rad": a.desired_q.tolist(),
            # 绕行：还剩几个点（含当前这段的终点）、最终目标
            "detour_left": len(a.path) + 1 if a.path else 0,
            "final_rad": (a.path[-1] if a.path else a.desired_q).tolist(),
            "payload_theta": None if a.payload_theta is None else a.payload_theta.tolist(),
            "payload_name": a.payload_name,
            "gravity_mass_kg": a.grav.moving_mass_kg,
            "kp_vec": [float(self.kp_body[i]) for i in a.layout.motor_indices],
            "joint_names": list(a.layout.joint_names),
        }

    def status(self) -> dict:
        with self._lock:
            engaged = self._engaged
            grav_on = self._grav_on
            waist_cmd = dict(self._waist_cmd) if self._waist_cmd else None
            waist_target = {str(i): float(v) for i, v in self._waist_target.items()} if self._waist_target else None
            arms = {name: self._arm_status_locked(a) for name, a in self._arms.items()}
            stale = self._stale
        return {
            "engaged": engaged,
            "float": any(v["float"] for v in arms.values()),
            "gravity_ff": grav_on,
            "gravity_dir": self.gravity.status(),
            "arms": arms,
            "waist_locked": waist_cmd is not None,
            "waist_hold_rad": waist_cmd,
            "waist_target_rad": waist_target,
            "max_speed_rad_s": self.max_speed,
            "loop_hz": round(1.0 / self._loop_period, 1) if self._loop_period > 0 else None,
            "last_error": self._last_err,
            # 急停锁存等运动禁止的原因（None = 没禁止）
            "inhibited": self._inhibit,
            # lowstate 过期的原因（None = 没过期或不检查）；过期时 ready 为 False
            "state_stale": stale,
            "lowstate_max_age_ms": None if self._state_max_age is None else round(self._state_max_age * 1000),
            # 接管 / 释放过渡（arm_sdk 后端有，其它为空）
            "ready": engaged and stale is None and self._commander.ready(),
            "transition": self._commander.phase(),
            # 指令出口详情（守护层后端有：守护层状态、锁存、循环抖动），其它为 None
            "commander": self._commander.info(),
        }

    # ------------------------------------------------------------ 循环
    def _check_state_age(self, engaged: bool) -> bool:
        """每拍查一次 lowstate 年龄；返回 True = 新鲜（或不检查）。不抛异常：它出错不能让本拍少发一帧。

        刚过期的那一拍，在同一把锁里：两只手停在当前指令角（拖拽中的手退出拖拽、锁在最后读到的
        角度、恢复全刚度）、腰停在当前指令角、撤销所有占用。过期期间新运动一律拒绝；恢复后照旧
        保持，不自动接着走。"""
        limit = self._state_max_age
        if limit is None or not engaged:
            if self._stale is not None:
                with self._lock:
                    self._stale = None
            return True
        age = self._state_age_s()
        if age <= limit:
            if self._stale is not None:
                with self._lock:
                    self._stale = None
            return True
        text = f"{_stale_age_text(age, limit)}：两只手停在当前指令角保持（帧照发），恢复后要重新下目标"
        try:
            try:
                snap_q = self.reader.snapshot().q
            except Exception:  # noqa: BLE001  读不到实测时拖拽中的手维持原样（没有角度可锁，不能跳回旧指令角）
                snap_q = None
            with self._lock:
                entering = self._stale is None
                self._stale = text
                if entering:
                    for a in self._arms.values():
                        if a.float and snap_q is not None:
                            # 拖拽时指令角不跟实测：先对齐到最后读到的角度，再恢复全刚度
                            a.cmd_q = a.clamp(np.asarray([snap_q[i] for i in a.layout.motor_indices], dtype=float))
                            a.float = False
                        if not a.float:
                            a.desired_q = a.cmd_q.copy()
                        a.owner = None
                    if self._waist_cmd is not None:
                        self._waist_target = dict(self._waist_cmd)
        except Exception as e:  # noqa: BLE001  保持没设成功也要照发本拍
            self._last_err = f"lowstate 过期处理：{type(e).__name__}: {e}"
            if self._stale is None:
                self._stale = text
        return False

    def _slew(self, a: _ArmState, dt: Optional[float] = None) -> None:
        # 每只手各按限速走、互不等待；每拍走的量按实际经过的时间算
        step = self.max_speed * (self._dt if dt is None else dt)
        delta = a.desired_q - a.cmd_q
        n = float(np.max(np.abs(delta))) if delta.size else 0.0
        if n > step and n > 1e-12:
            delta = delta * (step / n)
            a.cmd_q = a.clamp(a.cmd_q + delta)
        else:
            a.cmd_q = a.clamp(a.desired_q.copy())
        a.next_waypoint()

    def _publish(self) -> None:
        snap = self.reader.snapshot()
        n = min(self.profile.num_motors, len(snap.q))
        q = list(snap.q[:n])
        kp = list(self.kp_body[:n])
        kd = list(self.kd_body[:n])
        tau = [0.0] * n
        with self._lock:
            grav_on = self._grav_on
            waist_hold = dict(self._waist_cmd) if self._waist_cmd else None
            plan = [(a, a.float, a.cmd_q.copy(),
                     None if a.payload_theta is None else a.payload_theta.copy())
                    for a in self._arms.values()]
        g = self.gravity.g_vec()
        # 腰按保持角锁住；两只手都按各自指令角（拖拽中的手跟实测、刚度 0），都叠重力前馈
        if waist_hold:
            for idx, q_w in waist_hold.items():
                i = int(idx)
                q[i] = float(q_w)
                kp[i] = float(self._waist.lock_kp)
                kd[i] = float(self._waist.lock_kd)
        for a, floating, cmd_q, th in plan:
            q_meas = np.asarray([snap.q[i] for i in a.layout.motor_indices], dtype=float)
            q_arm = q_meas if floating else cmd_q
            tau_arm = np.zeros(a.layout.n)
            if grav_on:
                tau_arm = a.grav.tau_arm(q_meas, g)
                if th is not None:
                    tau_arm = tau_arm + a.grav.tau_payload(q_meas, th, g)
                tau_arm = np.clip(tau_arm, -TAU_FF_LIMIT, TAU_FF_LIMIT)
            a.last_tau_ff = np.asarray(tau_arm, float)
            for name, qv, t in zip(a.layout.joint_names, q_arm, tau_arm):
                i = a.layout.motor_index(name)
                q[i] = float(qv)
                tau[i] = float(t)
                if floating:
                    kp[i] = 0.0
                    kd[i] = TEACH_KD
        self._commander.write(q, kp, kd, tau, mode_machine=snap.mode_machine)

    def _loop(self) -> None:
        next_t = time.perf_counter()
        last = next_t
        while not self._stop.is_set():
            now = time.perf_counter()
            # 本拍实际间隔；卡顿后最多按 3 拍补，避免一下子跳一大步
            dt = min(max(now - last, 0.0), 3.0 * self._dt)
            last = now
            if dt > 0:
                self._loop_period = 0.95 * self._loop_period + 0.05 * dt
            try:
                self.gravity.update(dt)
            except Exception as e:  # noqa: BLE001  IMU 出错只影响重力方向（保持上一次的），不能停掉控制循环
                self._last_err = f"重力方向：{type(e).__name__}: {e}"
            engaged = ready = False
            try:
                engaged = self._engaged
                # lowstate 过期：本拍不往前走，按指令角照发（见 _check_state_age）
                fresh = self._check_state_age(engaged)
                # 指令出口没就绪（过渡中、守护层急停 / 失联）时不往前走：重新接管会从实测角开始
                ready = engaged and fresh and self._commander.ready()
                # 释放过渡中（arm_sdk）仍要发帧，按最后的指令角保持
                if engaged or self._commander.busy():
                    if ready:
                        with self._lock:
                            for a in self._arms.values():
                                if not a.float:
                                    self._slew(a, dt)
                            self._slew_waist(dt)
                    with self._publish_lock:
                        self._publish()
                    self._last_err = None
                elif self._waist_cmd is not None:
                    self._clear_waist()
            except Exception as e:
                self._last_err = f"{type(e).__name__}: {e}"
            hook = self.on_cycle
            if hook is not None:
                try:
                    hook(engaged, ready)
                except Exception as e:  # noqa: BLE001  钩子出错不能停掉控制循环
                    self._last_err = f"on_cycle: {type(e).__name__}: {e}"
            next_t += self._dt
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.perf_counter()

    def shutdown(self, timeout_s: float = 30.0) -> None:
        """释放并等过渡结束再停线程（arm_sdk 要先把权重降到 0）。"""
        self.release()
        deadline = time.monotonic() + timeout_s
        while self._commander.busy() and time.monotonic() < deadline:
            time.sleep(0.05)
        self._stop.set()

