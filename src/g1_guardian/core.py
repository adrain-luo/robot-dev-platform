"""GuardianCore: the decision logic of the body-side guardian, with no I/O.

The service calls ``handle(msg, now)`` for every incoming message and
``step(now, lowstate)`` once per control period; ``step`` returns the frame
to write (or ``None`` when the guardian is not driving the robot).

States
------
idle       not driving the robot, no frames (before engage, after release)
holding    engaged, no valid setpoint: decelerate to a stop and hold there
tracking   engaged, following the owner's latest valid setpoint
estop      latched: hold; setpoints rejected until ``reset``
fault      latched: hold; reasons link_lost / stale_lowstate / foreign_writer
releasing  explicit release: stiffness (lowcmd) or weight (arm_sdk) ramps to
           0, then frames stop -- the only way frames ever stop

Holding always means: keep writing frames at the last commanded position,
reached by decelerating within the acceleration limit, never by stopping.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

from g1_guardian import PROTOCOL_VERSION
from g1_guardian.config import GroupConfig, GuardianConfig

ENGAGED = ("holding", "tracking", "estop", "fault")
LIVE = ("holding", "tracking")
# How far outside the configured limits a joint may sit at engage. Further
# out means the config or the robot is wrong: refuse instead of moving it.
ENGAGE_LIMIT_TOLERANCE_RAD = 0.05
# Stiffness comes back this fast when E-STOP interrupts a release.
ESTOP_RESTORE_S = 0.3
FOREIGN_WRITER_WINDOW_S = 1.0


@dataclass
class LowStateView:
    q: Sequence[float]
    stamp: float  # monotonic time the message was received
    mode_machine: int = 0


@dataclass
class Frame:
    q: List[float]
    kp: List[float]
    kd: List[float]
    tau: List[float]
    mode_machine: int = 0
    weight: Optional[float] = None  # arm_sdk only
    final: bool = False  # last frame of a release


def track_joint(q: float, v: float, target: Optional[float], vmax: float, amax: float,
                lo: float, hi: float, dt: float) -> Tuple[float, float]:
    """One step of an acceleration- and velocity-limited follower.

    ``target is None`` means stop: decelerate at ``amax`` and stay where the
    joint comes to rest. Otherwise move towards ``target`` so that it can
    always still brake in time and never overshoots in one step
    (v <= |e| / dt). The braking speed is the discrete-time one: braking from
    v in steps of a*dt covers about v^2/(2a) + v*dt/2, so
    v <= a * (sqrt(dt^2/4 + 2|e|/a) - dt/2).
    """
    if dt <= 0.0:
        return q, v
    dv_max = amax * dt
    if target is None:
        if abs(v) <= dv_max:
            v = 0.0
        else:
            v -= math.copysign(dv_max, v)
    else:
        e = target - q
        if e == 0.0 and abs(v) <= dv_max:
            return min(hi, max(lo, q)), 0.0
        v_brake = amax * (math.sqrt(0.25 * dt * dt + 2.0 * abs(e) / amax) - 0.5 * dt)
        mag = min(vmax, v_brake, abs(e) / dt)
        v_des = math.copysign(mag, e)
        v += max(-dv_max, min(dv_max, v_des - v))
    q += v * dt
    if q < lo:
        q, v = lo, max(0.0, v)
    elif q > hi:
        q, v = hi, min(0.0, v)
    return q, v


class GuardianCore:
    def __init__(self, cfg: GuardianConfig) -> None:
        self.cfg = cfg
        self.state = "idle"
        self.latched: List[str] = []
        self.epoch = 0
        self.owner: Optional[str] = None
        self.owner_last_msg = -math.inf
        self.q_cmd: Dict[str, List[float]] = {}
        self.v_cmd: Dict[str, List[float]] = {}
        self.tau_ff: Dict[str, List[float]] = {}
        self.kp_scale: Dict[str, float] = {}
        self.hold_q: Dict[int, float] = {}
        self.target: Optional[Dict[str, List[float]]] = None
        self.target_expire = -math.inf
        self.target_received = -math.inf
        self.last_seq: Dict[str, int] = {}
        self.scale = 0.0
        self.scale_goal = 0.0
        self.scale_rate = 0.0
        self.last_step: Optional[float] = None
        self.lowstate: Optional[LowStateView] = None
        self.last_meas: Optional[List[float]] = None
        self.mode_machine = 0
        self.foreign_writer_at = -math.inf
        self.frames = 0
        self.rejects: Dict[str, int] = {}
        self.clamped = 0
        self.last_reject: Optional[Dict[str, Any]] = None
        self.events: List[Tuple[float, str]] = []  # drained by the service

    # ------------------------------------------------------------------ helpers
    def _event(self, now: float, what: str) -> None:
        self.events.append((now, what))

    def _reject(self, now: float, msg: Dict[str, Any], reason: str) -> str:
        self.rejects[reason] = self.rejects.get(reason, 0) + 1
        self.last_reject = {
            "t": now,
            "type": msg.get("type"),
            "client": msg.get("client"),
            "seq": msg.get("seq"),
            "reason": reason,
        }
        return reason

    def _fresh(self, now: float) -> bool:
        return self.lowstate is not None and now - self.lowstate.stamp <= self.cfg.lowstate_stale_s

    def _measured(self, g: GroupConfig) -> List[float]:
        assert self.last_meas is not None
        return [float(self.last_meas[m]) for m in g.motors]

    def _latch(self, now: float, reason: str) -> None:
        if reason not in self.latched:
            self.latched.append(reason)
        self.state = "estop" if "estop" in self.latched else "fault"
        self.target = None
        self.epoch += 1
        self._event(now, f"latched:{reason}")

    def note_foreign_writer(self, now: float) -> None:
        """Called by the output when frames that are not ours appear on the topic."""
        self.foreign_writer_at = now

    # ----------------------------------------------------------------- messages
    def handle(self, msg: Any, now: float) -> Optional[str]:
        """Apply one decoded message. Returns a reject reason, or None if accepted."""
        if not isinstance(msg, dict):
            return self._reject(now, {}, "not_an_object")
        if msg.get("v") != PROTOCOL_VERSION:
            return self._reject(now, msg, "bad_version")
        kind = msg.get("type")
        client = msg.get("client")
        if not isinstance(client, str) or not client:
            return self._reject(now, msg, "no_client")
        if self.owner is not None and client == self.owner:
            self.owner_last_msg = now
        if kind == "heartbeat":
            return None
        if kind == "estop":
            return self._on_estop(now, msg)
        if kind == "engage":
            return self._on_engage(now, msg, client)
        if kind == "setpoint":
            return self._on_setpoint(now, msg, client)
        if kind == "reset":
            return self._on_reset(now, msg)
        if kind == "release":
            return self._on_release(now, msg)
        return self._reject(now, msg, "unknown_type")

    def _on_estop(self, now: float, msg: Dict[str, Any]) -> Optional[str]:
        if self.state == "releasing":
            # Stop lowering the arm: hold where it is, bring stiffness back.
            meas_ok = self._fresh(now) and self.last_meas is not None
            for g in self.cfg.groups:
                if meas_ok:
                    self.q_cmd[g.name] = [min(hi, max(lo, q)) for q, lo, hi in zip(self._measured(g), g.lower, g.upper)]
                self.v_cmd[g.name] = [0.0] * g.size
            self.scale_goal, self.scale_rate = 1.0, 1.0 / ESTOP_RESTORE_S
        self._latch(now, "estop")
        if self.q_cmd == {}:
            # Not engaged: nothing to hold, but a later engage needs a reset.
            self.state = "idle"
        return None

    def _check_engage_limits(self) -> Optional[str]:
        for g in self.cfg.groups:
            for i, (q, lo, hi) in enumerate(zip(self._measured(g), g.lower, g.upper)):
                if q < lo - ENGAGE_LIMIT_TOLERANCE_RAD or q > hi + ENGAGE_LIMIT_TOLERANCE_RAD:
                    return f"out_of_limits:{g.name}[{i}]={q:.3f}"
        return None

    def _on_engage(self, now: float, msg: Dict[str, Any], client: str) -> Optional[str]:
        if self.state == "releasing":
            return self._reject(now, msg, "releasing")
        if "estop" in self.latched:
            return self._reject(now, msg, "estop_latched")
        if self.state in LIVE:
            if self.owner == client:
                return None  # idempotent: the owner asks for the current epoch
            if now - self.owner_last_msg <= self.cfg.link_timeout_s:
                return self._reject(now, msg, "owned_by_other")
        if not self._fresh(now) or self.last_meas is None:
            return self._reject(now, msg, "no_lowstate")
        if now - self.foreign_writer_at <= FOREIGN_WRITER_WINDOW_S:
            return self._reject(now, msg, "foreign_writer")
        bad = self._check_engage_limits()
        if bad:
            return self._reject(now, msg, bad)

        was_idle = self.state == "idle"
        if was_idle:
            # Fresh takeover: start from the measured pose.
            for g in self.cfg.groups:
                self.q_cmd[g.name] = [min(hi, max(lo, q)) for q, lo, hi in zip(self._measured(g), g.lower, g.upper)]
                self.v_cmd[g.name] = [0.0] * g.size
                self.tau_ff[g.name] = [0.0] * g.size
                self.kp_scale[g.name] = 1.0
            self.hold_q = {m: float(self.last_meas[m]) for m in self.cfg.hold_at_engage}
            if self.cfg.channel == "arm_sdk" and self.cfg.engage_ramp_s > 0:
                self.scale, self.scale_goal, self.scale_rate = 0.0, 1.0, 1.0 / self.cfg.engage_ramp_s
            else:
                self.scale, self.scale_goal, self.scale_rate = 1.0, 1.0, 0.0
        # Taking over from a lost owner (link_lost / stale_lowstate) keeps the
        # current hold; only the latch is cleared and a new epoch issued.
        self.latched = [r for r in self.latched if r == "estop"]
        self.owner = client
        self.owner_last_msg = now
        self.target = None
        self.state = "holding"
        self.epoch += 1
        self.last_seq.pop(client, None)
        self._event(now, f"engaged:{client}")
        return None

    def _on_setpoint(self, now: float, msg: Dict[str, Any], client: str) -> Optional[str]:
        if client != self.owner:
            return self._reject(now, msg, "not_owner")
        if self.state not in LIVE:
            return self._reject(now, msg, f"state_{self.state}")
        if msg.get("epoch") != self.epoch:
            return self._reject(now, msg, "stale_epoch")
        seq = msg.get("seq")
        if not isinstance(seq, int):
            return self._reject(now, msg, "no_seq")
        if seq <= self.last_seq.get(client, -1):
            return self._reject(now, msg, "old_seq")
        try:
            valid_for = float(msg.get("valid_for_ms", 0)) / 1000.0
        except (TypeError, ValueError):
            return self._reject(now, msg, "bad_valid_for")
        if not math.isfinite(valid_for):
            return self._reject(now, msg, "bad_valid_for")
        valid_for = min(self.cfg.max_valid_for_s, max(self.cfg.min_valid_for_s, valid_for))
        groups = msg.get("groups")
        if not isinstance(groups, dict) or not groups:
            return self._reject(now, msg, "no_groups")

        new_target: Dict[str, List[float]] = {}
        new_tau: Dict[str, List[float]] = {}
        new_kp: Dict[str, float] = {}
        clamped = 0
        for name, body in groups.items():
            try:
                g = self.cfg.group(name)
            except KeyError:
                return self._reject(now, msg, f"unknown_group:{name}")
            if not isinstance(body, dict):
                return self._reject(now, msg, f"bad_group:{name}")
            q = body.get("q")
            if not isinstance(q, list) or len(q) != g.size:
                return self._reject(now, msg, f"bad_q:{name}")
            try:
                q = [float(x) for x in q]
            except (TypeError, ValueError):
                return self._reject(now, msg, f"bad_q:{name}")
            if not all(math.isfinite(x) for x in q):
                return self._reject(now, msg, f"nonfinite_q:{name}")
            for i, (x, cur) in enumerate(zip(q, self.q_cmd[name])):
                if abs(x - cur) > self.cfg.max_target_jump_rad:
                    return self._reject(now, msg, f"target_jump:{name}[{i}]")
            q_cl = [min(hi, max(lo, x)) for x, lo, hi in zip(q, g.lower, g.upper)]
            clamped += sum(1 for a, b in zip(q, q_cl) if a != b)
            new_target[name] = q_cl
            tau = body.get("tau_ff")
            if tau is not None:
                if not isinstance(tau, list) or len(tau) != g.size:
                    return self._reject(now, msg, f"bad_tau_ff:{name}")
                try:
                    tau = [float(x) for x in tau]
                except (TypeError, ValueError):
                    return self._reject(now, msg, f"bad_tau_ff:{name}")
                if not all(math.isfinite(x) for x in tau):
                    return self._reject(now, msg, f"nonfinite_tau_ff:{name}")
                new_tau[name] = [max(-lim, min(lim, x)) for x, lim in zip(tau, g.tau_ff_limit)]
            ks = body.get("kp_scale")
            if ks is not None:
                try:
                    ks = float(ks)
                except (TypeError, ValueError):
                    return self._reject(now, msg, f"bad_kp_scale:{name}")
                if not 0.0 <= ks <= 1.0:
                    return self._reject(now, msg, f"bad_kp_scale:{name}")
                new_kp[name] = ks

        self.last_seq[client] = seq
        self.clamped += clamped
        self.target = new_target
        self.tau_ff.update(new_tau)
        self.kp_scale.update(new_kp)
        self.target_received = now
        self.target_expire = now + valid_for
        self.state = "tracking"
        return None

    def _on_reset(self, now: float, msg: Dict[str, Any]) -> Optional[str]:
        if not self.latched:
            return None
        if self.state != "idle" and not self._fresh(now):
            return self._reject(now, msg, "no_lowstate")
        self.latched = []
        self.epoch += 1
        self.target = None
        if self.state in ("estop", "fault"):
            self.state = "holding"
            # Whoever owned the arm must engage again to learn the new epoch.
            self.owner_last_msg = now
        self._event(now, "reset")
        return None

    def _on_release(self, now: float, msg: Dict[str, Any]) -> Optional[str]:
        if self.state in ("idle", "releasing"):
            return None
        self.state = "releasing"
        self.target = None
        self.scale_goal = 0.0
        self.scale_rate = math.inf if self.cfg.release_ramp_s <= 0 else 1.0 / self.cfg.release_ramp_s
        self.epoch += 1
        self._event(now, "release_started")
        return None

    # --------------------------------------------------------------------- step
    def step(self, now: float, lowstate: Optional[LowStateView]) -> Optional[Frame]:
        dt = 0.0 if self.last_step is None else min(max(now - self.last_step, 0.0), 3.0 * self.cfg.period_s)
        self.last_step = now
        if lowstate is not None and (self.lowstate is None or lowstate.stamp >= self.lowstate.stamp):
            self.lowstate = lowstate
            if now - lowstate.stamp <= self.cfg.lowstate_stale_s:
                self.last_meas = list(lowstate.q)
                self.mode_machine = int(lowstate.mode_machine)
        if self.state == "idle":
            return None

        if self.state in LIVE:
            if not self._fresh(now):
                self._latch(now, "stale_lowstate")
            elif now - self.owner_last_msg > self.cfg.link_timeout_s:
                self._latch(now, "link_lost")
        if self.state in ENGAGED and now - self.foreign_writer_at <= 0.2 and "foreign_writer" not in self.latched:
            self._latch(now, "foreign_writer")
        if self.state == "tracking" and now > self.target_expire:
            self.state = "holding"
            self.target = None
            self._event(now, "setpoint_expired")

        for g in self.cfg.groups:
            tgt = self.target.get(g.name) if (self.state == "tracking" and self.target) else None
            qs, vs = self.q_cmd[g.name], self.v_cmd[g.name]
            for i in range(g.size):
                qs[i], vs[i] = track_joint(
                    qs[i], vs[i], None if tgt is None else tgt[i],
                    g.max_velocity[i], g.max_acceleration[i], g.lower[i], g.upper[i], dt,
                )

        if self.scale != self.scale_goal:
            if math.isinf(self.scale_rate):
                self.scale = self.scale_goal
            elif self.scale < self.scale_goal:
                self.scale = min(self.scale_goal, self.scale + self.scale_rate * dt)
            else:
                self.scale = max(self.scale_goal, self.scale - self.scale_rate * dt)

        frame = self._build_frame()
        if self.state == "releasing" and self.scale <= 0.0:
            frame.final = True
            self.state = "idle"
            self.owner = None
            self.target = None
            self.q_cmd, self.v_cmd, self.tau_ff, self.kp_scale = {}, {}, {}, {}
            self._event(now, "released")
        self.frames += 1
        return frame

    def _build_frame(self) -> Frame:
        cfg = self.cfg
        n = cfg.num_motors
        meas = self.last_meas if self.last_meas is not None else [0.0] * n
        q = [float(meas[i]) if i < len(meas) else 0.0 for i in range(n)]
        kp = list(cfg.kp)
        kd = list(cfg.kd)
        tau = [0.0] * n
        for m, v in self.hold_q.items():
            q[m] = v
        for g in cfg.groups:
            ks = self.kp_scale.get(g.name, 1.0)
            for i, m in enumerate(g.motors):
                q[m] = self.q_cmd[g.name][i]
                kp[m] = cfg.kp[m] * ks
                tau[m] = self.tau_ff[g.name][i]
        weight = None
        if cfg.channel == "lowcmd":
            if self.scale < 1.0:
                kp = [k * self.scale for k in kp]
                tau = [t * self.scale for t in tau]
        else:
            weight = self.scale
        return Frame(q=q, kp=kp, kd=kd, tau=tau, mode_machine=self.mode_machine, weight=weight)

    # ------------------------------------------------------------------- status
    def status(self, now: float) -> Dict[str, Any]:
        groups = {}
        for g in self.cfg.groups:
            entry: Dict[str, Any] = {}
            if g.name in self.q_cmd:
                entry["q_cmd"] = [round(x, 6) for x in self.q_cmd[g.name]]
            if self.last_meas is not None:
                entry["q_meas"] = [round(float(self.last_meas[m]), 6) for m in g.motors]
            groups[g.name] = entry
        age = None if self.lowstate is None else round((now - self.lowstate.stamp) * 1000.0, 1)
        sp_age = None if self.target_received == -math.inf else round((now - self.target_received) * 1000.0, 1)
        return {
            "v": PROTOCOL_VERSION,
            "type": "status",
            "t": now,
            "robot": self.cfg.robot,
            "channel": self.cfg.channel,
            "state": self.state,
            "latched": list(self.latched),
            "epoch": self.epoch,
            "owner": self.owner,
            "scale": round(self.scale, 4),
            "lowstate_age_ms": age,
            "setpoint_age_ms": sp_age,
            "frames": self.frames,
            "clamped": self.clamped,
            "rejects": dict(self.rejects),
            "last_reject": self.last_reject,
            "groups": groups,
        }
