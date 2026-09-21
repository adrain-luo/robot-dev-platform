"""Load arm_control runtime settings (JSON + env overrides)."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE / "config" / "g1d.json"


@dataclass
class Settings:
    robot: str
    network_interface: str
    unitree_sdk_path: Optional[Path]
    command_topic: str
    state_topic: str
    urdf: Optional[Path]
    base_link: str
    dds_domain: int = 0
    lowstate_timeout_s: float = 5.0
    allow_real_motion: bool = False
    max_speed_rad_s: float = 0.2

    @property
    def config_dir(self) -> Path:
        return HERE / "config"


def _as_path(value: Any) -> Optional[Path]:
    if value is None or value == "" or value == "null":
        return None
    return Path(str(value)).expanduser()


def load_settings(path: Optional[Path] = None) -> Settings:
    cfg_path = Path(path) if path else DEFAULT_CONFIG
    raw: Dict[str, Any] = {}
    if cfg_path.exists():
        raw = json.loads(cfg_path.read_text(encoding="utf-8"))

    defaults = raw.get("defaults") or {}
    iface = os.environ.get("ARM_DDS_INTERFACE") or raw.get("network_interface") or "enp86s0"
    sdk = os.environ.get("UNITREE_SDK2PY_PATH") or raw.get("unitree_sdk_path")

    return Settings(
        robot=str(raw.get("robot") or "g1d"),
        network_interface=str(iface),
        unitree_sdk_path=_as_path(sdk),
        command_topic=str(raw.get("command_topic") or "rt/lowcmd"),
        state_topic=str(raw.get("state_topic") or "rt/lowstate"),
        urdf=_as_path(raw.get("urdf")),
        base_link=str(raw.get("base_link") or "torso_link"),
        dds_domain=int(raw.get("dds_domain") or 0),
        lowstate_timeout_s=float(raw.get("lowstate_timeout_s") or 5.0),
        allow_real_motion=bool(defaults.get("allow_real_motion", False)),
        max_speed_rad_s=float(defaults.get("max_speed_rad_s") or 0.2),
    )
