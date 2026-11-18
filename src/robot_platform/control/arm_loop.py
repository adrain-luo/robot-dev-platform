"""双臂控制循环：50–100 Hz 保持 + 限速逼近目标，接管后经后端 Commander 周期发整机指令。

两只手各有一套状态（目标角、指令角、拖拽、负载补偿、占用者），互不等待：
一只手拖拽时另一只照常受控，一只手在走时可以给另一只发新目标。
接管 / 释放 / 急停 / 腰仍是整机的（arm_sdk 的权重、lowcmd 的整帧都是整机共用）。
"""

from __future__ import annotations

import threading
import time
from contextlib import contextmanager, nullcontext
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Sequence

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
    ) -> None:
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
        self._arms: Dict[str, _ArmState] = {}
        for layout in self.profile.arms.values():
            q0 = np.asarray(reader.read_arm(layout).q, dtype=float)
            limits = self.kin.arms[layout.name].model.joint_limits()
            self._arms[layout.name] = _ArmState(layout, limits, ArmGravity(layout.name, urdf, base_link), q0)
        self._engaged = False
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

    @contextmanager
    def claim(self, arm: str, owner: str) -> Iterator[None]:
        """任务独占一只手：期间别的来源给这只手设目标会被拒绝；退出时自动归还。"""
        a = self._arm(arm)
        with self._lock:
            if a.owner is not None and a.owner != owner:
                raise PermissionErrorMotion(f"{side_label(arm)}臂正被「{a.owner}」使用，等它结束或先急停")
            if a.float:
                raise PermissionErrorMotion(f"{side_label(arm)}臂在拖拽，先退出拖拽")
            a.owner = owner
        try:
            yield
        finally:
            with self._lock:
                if a.owner == owner:
                    a.owner = None

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
        self._commander.prepare()
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
            self._engaged = True

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

    def estop_hold(self) -> None:
        """两只手都退出拖拽、锁在当前实测角；腰也锁住。"""
        qs = {name: a.clamp(self._measured(a)) for name, a in self._arms.items()}
        waist_now = self._read_waist()
        with self._lock:
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

    def set_waist(self, yaw_rad: Optional[float] = None, pitch_rad: Optional[float] = None) -> None:
        if self._waist is None:
            raise PermissionErrorMotion(f"{self.profile.name} 没有可控的腰")
        if not self._waist.movable:
            raise PermissionErrorMotion(f"{self.profile.name} 的腰只保持不转（站立时转腰会影响平衡）")
        self._require_ready("动腰")
        with self._lock:
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

        detour=True（「到位」类单次目标）：直线会碰时规划一条绕行折线，按中间点依次走；
        返回规划摘要（没绕行返回 None）。连续发的小步目标（点动、跟踪）不要开，规划要零点几秒到几秒。"""
        a = self._arm(arm)
        arr = np.asarray(q, dtype=float).reshape(-1)
        if arr.size != a.layout.n or not np.all(np.isfinite(arr)):
            raise ValueError(f"需要 {a.layout.n} 个有限关节角")
        if self._engaged:
            self._require_ready("走点")

        def _gate() -> None:
            if not self._engaged:
                raise PermissionErrorMotion("先接管再设目标")
            if a.float:
                raise PermissionErrorMotion(f"{side_label(arm)}臂在拖拽，先退出拖拽再走点")
            if a.owner is not None and a.owner != owner:
                raise PermissionErrorMotion(f"{side_label(arm)}臂正被「{a.owner}」使用，等它结束或先急停")

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

    def view(self, arm: str, owner: Optional[str] = None) -> "ArmView":
        """给只管一只手的流程（负载测量、到位对比、手眼采集、棋盘触摸……）用的单臂接口。"""
        self._arm(arm)
        return ArmView(self, arm, owner)

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
            # 接管 / 释放过渡（arm_sdk 后端有，其它为空）
            "ready": engaged and self._commander.ready(),
            "transition": self._commander.phase(),
        }

    # ------------------------------------------------------------ 循环
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
            try:
                engaged = self._engaged
                # 释放过渡中（arm_sdk）仍要发帧，按最后的指令角保持
                if engaged or self._commander.busy():
                    if engaged:
                        with self._lock:
                            for a in self._arms.values():
                                if not a.float:
                                    self._slew(a, dt)
                            self._slew_waist(dt)
                    self._publish()
                    self._last_err = None
                elif self._waist_cmd is not None:
                    self._clear_waist()
            except Exception as e:
                self._last_err = f"{type(e).__name__}: {e}"
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


class ArmView:
    """ArmLoop 的单臂视图：接口与旧的单臂 ArmLoop 一致，发目标时带上占用者。

    status() 是整机状态加上这只手的字段（float / cmd_rad / desired_rad / payload_* / kp_vec），
    所以按旧接口写的流程（只认一只手）不用改。
    """

    def __init__(self, loop: ArmLoop, arm: str, owner: Optional[str] = None) -> None:
        self.loop = loop
        self.arm = arm
        self.owner = owner

    @property
    def layout(self) -> ArmLayout:
        return self.loop.layout(self.arm)

    @property
    def limits(self) -> np.ndarray:
        return self.loop.limits(self.arm)

    @property
    def max_speed(self) -> float:
        return self.loop.max_speed

    @property
    def kin(self) -> RobotKinematics:
        return self.loop.kin

    @property
    def reader(self) -> StateSource:
        return self.loop.reader

    def claim(self):
        """占用这只手（owner 为 None 时不占用）。用法：with view.claim(): ..."""
        if self.owner is None:
            return nullcontext()
        return self.loop.claim(self.arm, self.owner)

    def set_target(self, q: Sequence[float], detour: bool = False) -> Optional[dict]:
        return self.loop.set_target(q, arm=self.arm, owner=self.owner, detour=detour)

    def arm_sample(self, wait: bool = False) -> dict:
        return self.loop.arm_sample(self.arm, wait=wait)

    def arm_gains(self) -> tuple:
        return self.loop.arm_gains(self.arm)

    def set_payload_theta(self, theta, name: Optional[str] = None) -> None:
        self.loop.set_payload_theta(theta, name, arm=self.arm)

    def payload_theta(self) -> Optional[list]:
        return self.loop.payload_theta(self.arm)

    def estop_hold(self) -> None:
        """单臂流程自己中止时只停这只手；整机急停走 ArmLoop.estop_hold。"""
        self.loop.hold_arm(self.arm)

    def status(self) -> dict:
        st = self.loop.status()
        st.update(st["arms"][self.arm])
        return st
