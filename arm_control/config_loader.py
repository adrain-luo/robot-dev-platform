"""Load arm_control runtime settings (instance.json + env overrides).

instances/<实例>/instance.json 放这台机器的运行参数（网卡、DDS 域、默认限速）和机型名 `robot`；
关节表、话题、URDF 等机型数据在 robots/<robot>/robot.json。
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

import instance as instances
from model.robot_profile import RobotProfile, load_profile

HERE = Path(__file__).resolve().parent


@dataclass
class Settings:
    robot: str
    profile: RobotProfile
    backend: str
    network_interface: str
    unitree_sdk_path: Optional[Path]
    command_topic: str
    state_topic: str
    urdf: Path
    base_link: str
    dds_domain: int = 0
    lowstate_timeout_s: float = 5.0
    allow_real_motion: bool = False
    max_speed_rad_s: float = 0.2


def _as_path(value: Any, relative_to: Optional[Path] = None) -> Optional[Path]:
    if value is None or value == "" or value == "null":
        return None
    p = Path(str(value)).expanduser()
    if not p.is_absolute() and relative_to is not None:
        p = (relative_to / p).resolve()
    return p


def load_settings(
    inst: Optional["instances.Instance"] = None, robot: Optional[str] = None, backend: Optional[str] = None
) -> Settings:
    inst = inst or instances.active()
    raw: Dict[str, Any] = dict(inst.settings)

    defaults = raw.get("defaults") or {}
    robot_name = robot or os.environ.get("ARM_ROBOT") or raw.get("robot") or "g1d"
    profile = load_profile(robot_name)
    iface = os.environ.get("ARM_DDS_INTERFACE") or raw.get("network_interface") or "enp86s0"
    sdk = os.environ.get("UNITREE_SDK2PY_PATH") or raw.get("unitree_sdk_path")

    return Settings(
        robot=profile.name,
        profile=profile,
        backend=str(backend or os.environ.get("ARM_BACKEND") or profile.control.backend),
        network_interface=str(iface),
        unitree_sdk_path=_as_path(sdk),
        command_topic=profile.control.command_topic,
        state_topic=profile.control.state_topic,
        urdf=profile.urdf,
        base_link=profile.base_link,
        dds_domain=int(raw.get("dds_domain") or 0),
        lowstate_timeout_s=float(raw.get("lowstate_timeout_s") or 5.0),
        allow_real_motion=bool(defaults.get("allow_real_motion", False)),
        max_speed_rad_s=float(defaults.get("max_speed_rad_s") or 0.2),
    )
