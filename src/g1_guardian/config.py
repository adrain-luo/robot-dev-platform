"""Guardian configuration: motors, groups, gains, limits, timeouts.

A config is a JSON file (see configs/g1d.json). Everything that decides how
hard and how fast the arm may move lives here, per robot type; Box-side
programs can only ask for less (a kp scale <= 1), never for more.
"""

from __future__ import annotations

import ipaddress
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

CHANNELS = ("lowcmd", "arm_sdk")


class ConfigError(ValueError):
    pass


@dataclass(frozen=True)
class GroupConfig:
    """A set of motors the guardian actively holds and tracks."""

    name: str
    motors: Tuple[int, ...]
    lower: Tuple[float, ...]
    upper: Tuple[float, ...]
    max_velocity: Tuple[float, ...]
    max_acceleration: Tuple[float, ...]
    tau_ff_limit: Tuple[float, ...]

    @property
    def size(self) -> int:
        return len(self.motors)


@dataclass(frozen=True)
class AccessConfig:
    """Which source addresses may talk to the guardian, by role.

    local    this computer (loopback, the bound address): every command, and
             ``release`` even when it is not the owner (the operator's CLI)
    control  the Box running the motion core: every command; ``release`` only
             from the current owner
    monitor  status and E-STOP only
    Any other address is dropped without an answer. What each role may send
    is decided in ``core.ROLE_COMMANDS``.
    """

    local: Tuple[str, ...]
    control: Tuple[str, ...]
    monitor: Tuple[str, ...] = ()

    def role(self, source: Optional[str]) -> Optional[str]:
        if source is None:
            return None
        if source in self.local:
            return "local"
        if source in self.control:
            return "control"
        if source in self.monitor:
            return "monitor"
        return None

    @property
    def remote(self) -> Tuple[str, ...]:
        """Addresses that reach the guardian over the network (what the firewall must let in)."""
        return self.control + self.monitor


@dataclass(frozen=True)
class GuardianConfig:
    robot: str
    channel: str  # "lowcmd" (G1D) or "arm_sdk" (G1)
    num_motors: int
    kp: Tuple[float, ...]
    kd: Tuple[float, ...]
    groups: Tuple[GroupConfig, ...]
    hold_at_engage: Tuple[int, ...] = ()
    # Motors that must read exactly idle before the guardian takes over (G1D
    # has no legs: a leg motor with an angle means this is a standing biped,
    # and releasing its motion mode would drop it).
    idle_motors: Tuple[int, ...] = ()
    status: str = "candidate"
    rate_hz: float = 200.0
    lowstate_stale_s: float = 0.1
    link_timeout_s: float = 0.5
    min_valid_for_s: float = 0.01
    max_valid_for_s: float = 1.0
    max_target_jump_rad: float = 0.5
    engage_ramp_s: float = 0.0
    release_ramp_s: float = 3.0
    release_motion_mode: bool = True
    arm_sdk_weight_motor: Optional[int] = None
    status_hz: float = 50.0
    blackbox_seconds: float = 10.0
    # Time for a group's stiffness scale to go all the way between 0 and 1.
    stiffness_ramp_s: float = 0.25
    # Damping used while a group is fully compliant (kp scale 0, e.g. teach
    # mode); None keeps the body kd. Interpolated with the stiffness scale.
    compliant_kd: Optional[float] = None
    # Who may send what (``access`` in the JSON). Required: a config without it does not load.
    access: Optional[AccessConfig] = None
    extra: Dict[str, Any] = field(default_factory=dict)

    @property
    def period_s(self) -> float:
        return 1.0 / self.rate_hz

    def group(self, name: str) -> GroupConfig:
        for g in self.groups:
            if g.name == name:
                return g
        raise KeyError(name)

    @property
    def group_names(self) -> Tuple[str, ...]:
        return tuple(g.name for g in self.groups)

    @property
    def active_motors(self) -> Tuple[int, ...]:
        return tuple(m for g in self.groups for m in g.motors)


def _floats(values: Any, n: int, what: str) -> Tuple[float, ...]:
    if isinstance(values, (int, float)):
        values = [values] * n
    if not isinstance(values, list) or len(values) != n:
        raise ConfigError(f"{what}: expected {n} numbers, got {values!r}")
    out = tuple(float(v) for v in values)
    if not all(math.isfinite(v) for v in out):
        raise ConfigError(f"{what}: values must be finite")
    return out


def _addresses(values: Any, what: str) -> Tuple[str, ...]:
    if not isinstance(values, list):
        raise ConfigError(f"access.{what}: expected a list of IPv4 addresses, got {values!r}")
    out = []
    for v in values:
        try:
            ip = ipaddress.ip_address(str(v))
        except ValueError as exc:
            raise ConfigError(f"access.{what}: {v!r} is not an IP address") from exc
        if ip.version != 4:
            raise ConfigError(f"access.{what}: {v!r}: the guardian listens on IPv4 only")
        if str(ip) != v:
            raise ConfigError(f"access.{what}: write {v!r} as {str(ip)!r}")
        out.append(str(ip))
    return tuple(out)


def access_from_dict(d: Any) -> AccessConfig:
    if not isinstance(d, dict):
        raise ConfigError("missing access section: list the local, control and monitor addresses")
    unknown = sorted(k for k in d if not k.startswith("_") and k not in ("local", "control", "monitor"))
    if unknown:
        raise ConfigError(f"access: unknown keys {unknown}")
    local = _addresses(d.get("local", []), "local")
    control = _addresses(d.get("control", []), "control")
    monitor = _addresses(d.get("monitor", []), "monitor")
    every = local + control + monitor
    dup = sorted({a for a in every if every.count(a) > 1})
    if dup:
        raise ConfigError(f"access: {dup} listed more than once (each address has one role)")
    if not local and not control:
        raise ConfigError("access: no local or control address, nobody could engage")
    for a in control + monitor:
        if ipaddress.ip_address(a).is_loopback:
            raise ConfigError(f"access: loopback {a} belongs in local")
    return AccessConfig(local=local, control=control, monitor=monitor)


def config_from_dict(d: Dict[str, Any]) -> GuardianConfig:
    try:
        robot = str(d["robot"])
        channel = str(d["channel"])
        n = int(d["num_motors"])
        gains = d["gains"]
        groups_raw = d["groups"]
    except KeyError as exc:
        raise ConfigError(f"missing key {exc}") from exc
    if channel not in CHANNELS:
        raise ConfigError(f"channel must be one of {CHANNELS}, got {channel!r}")
    kp = _floats(gains.get("kp"), n, "gains.kp")
    kd = _floats(gains.get("kd"), n, "gains.kd")
    if any(v < 0 for v in kp + kd):
        raise ConfigError("gains must be >= 0")

    limits = d.get("limits", {})
    default_v = limits.get("max_velocity_rad_s", 1.0)
    default_a = limits.get("max_acceleration_rad_s2", 10.0)
    default_tau = limits.get("tau_ff_limit_nm", 8.0)
    margin = float(limits.get("position_margin_rad", 0.02))

    groups: List[GroupConfig] = []
    seen = set()
    for name, g in groups_raw.items():
        motors = tuple(int(m) for m in g["motors"])
        k = len(motors)
        if k == 0:
            raise ConfigError(f"group {name}: no motors")
        for m in motors:
            if not 0 <= m < n:
                raise ConfigError(f"group {name}: motor {m} out of range 0..{n - 1}")
            if m in seen:
                raise ConfigError(f"motor {m} is in two groups")
            seen.add(m)
        lower = _floats(g["lower"], k, f"{name}.lower")
        upper = _floats(g["upper"], k, f"{name}.upper")
        lower = tuple(lo + margin for lo in lower)
        upper = tuple(hi - margin for hi in upper)
        if any(lo >= hi for lo, hi in zip(lower, upper)):
            raise ConfigError(f"group {name}: lower must be below upper (after margin)")
        vmax = _floats(g.get("max_velocity_rad_s", default_v), k, f"{name}.max_velocity_rad_s")
        amax = _floats(g.get("max_acceleration_rad_s2", default_a), k, f"{name}.max_acceleration_rad_s2")
        tau = _floats(g.get("tau_ff_limit_nm", default_tau), k, f"{name}.tau_ff_limit_nm")
        if any(v <= 0 for v in vmax + amax) or any(t < 0 for t in tau):
            raise ConfigError(f"group {name}: velocity/acceleration must be > 0, tau limit >= 0")
        groups.append(GroupConfig(name, motors, lower, upper, vmax, amax, tau))

    hold_at_engage = tuple(int(m) for m in d.get("hold_at_engage", []))
    for m in hold_at_engage:
        if not 0 <= m < n:
            raise ConfigError(f"hold_at_engage: motor {m} out of range")
        if m in seen:
            raise ConfigError(f"hold_at_engage: motor {m} is also in a group")
    idle_motors = tuple(int(m) for m in d.get("idle_motors", []))
    for m in idle_motors:
        if not 0 <= m < n:
            raise ConfigError(f"idle_motors: motor {m} out of range")
        if m in seen or m in hold_at_engage:
            raise ConfigError(f"idle_motors: motor {m} is also driven")
    status = str(d.get("status", "candidate"))
    if status not in ("candidate", "draft", "validated"):
        raise ConfigError("status must be candidate, draft or validated")

    timing = d.get("timing", {})
    weight_motor = d.get("arm_sdk_weight_motor")
    if channel == "arm_sdk" and weight_motor is None:
        raise ConfigError("arm_sdk channel needs arm_sdk_weight_motor")
    cfg = GuardianConfig(
        robot=robot,
        channel=channel,
        num_motors=n,
        kp=kp,
        kd=kd,
        groups=tuple(groups),
        hold_at_engage=hold_at_engage,
        idle_motors=idle_motors,
        status=status,
        rate_hz=float(timing.get("rate_hz", 200.0)),
        lowstate_stale_s=float(timing.get("lowstate_stale_ms", 100)) / 1000.0,
        link_timeout_s=float(timing.get("link_timeout_ms", 500)) / 1000.0,
        min_valid_for_s=float(timing.get("min_valid_for_ms", 10)) / 1000.0,
        max_valid_for_s=float(timing.get("max_valid_for_ms", 1000)) / 1000.0,
        max_target_jump_rad=float(limits.get("max_target_jump_rad", 0.5)),
        engage_ramp_s=float(timing.get("engage_ramp_s", 0.0)),
        release_ramp_s=float(timing.get("release_ramp_s", 3.0)),
        release_motion_mode=bool(d.get("release_motion_mode", channel == "lowcmd")),
        arm_sdk_weight_motor=None if weight_motor is None else int(weight_motor),
        status_hz=float(timing.get("status_hz", 50.0)),
        blackbox_seconds=float(timing.get("blackbox_seconds", 10.0)),
        stiffness_ramp_s=float(timing.get("stiffness_ramp_s", 0.25)),
        compliant_kd=None if limits.get("compliant_kd") is None else float(limits["compliant_kd"]),
        access=access_from_dict(d.get("access")),
        extra={k: v for k, v in d.items() if k.startswith("_") or k in ("note", "joints")},
    )
    if cfg.rate_hz <= 0 or cfg.status_hz <= 0:
        raise ConfigError("rates must be > 0")
    if not 0 < cfg.min_valid_for_s <= cfg.max_valid_for_s:
        raise ConfigError("need 0 < min_valid_for_ms <= max_valid_for_ms")
    if cfg.link_timeout_s <= 0 or cfg.lowstate_stale_s <= 0:
        raise ConfigError("timeouts must be > 0")
    if cfg.release_ramp_s < 0 or cfg.engage_ramp_s < 0 or cfg.stiffness_ramp_s < 0:
        raise ConfigError("ramps must be >= 0")
    if cfg.compliant_kd is not None and not (math.isfinite(cfg.compliant_kd) and cfg.compliant_kd >= 0):
        raise ConfigError("limits.compliant_kd must be a finite number >= 0")
    return cfg


def load_config(path: str | Path) -> GuardianConfig:
    p = Path(path)
    with p.open(encoding="utf-8") as f:
        return config_from_dict(json.load(f))


def builtin_config_path(name: str) -> Path:
    return Path(__file__).resolve().parent / "configs" / f"{name}.json"


def group_slices(cfg: GuardianConfig) -> Dict[str, Sequence[int]]:
    return {g.name: g.motors for g in cfg.groups}
