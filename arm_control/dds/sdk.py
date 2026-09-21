"""Resolve and prepend Unitree SDK2 Python to sys.path."""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Optional


def ensure_sdk(sdk_path: Optional[Path] = None) -> Path:
    candidates = []
    if sdk_path is not None:
        candidates.append(Path(sdk_path).expanduser())
    env = os.environ.get("UNITREE_SDK2PY_PATH")
    if env:
        candidates.append(Path(env).expanduser())
    candidates.extend(
        [
            Path("/home/robot/vision_arm_control_coupled_ik_fast_v6_2_20260911/unitree_sdk2_python"),
            Path("/home/robot/vision_arm_control_coupled_ik_dev_current/unitree_sdk2_python"),
            Path("/home/robot/vision_arm_control_refactor/unitree_sdk2_python"),
            Path("/home/robot/unitree_sdk2_python"),
        ]
    )
    for cand in candidates:
        if (cand / "unitree_sdk2py" / "__init__.py").is_file():
            resolved = cand.resolve()
            path = str(resolved)
            if path not in sys.path:
                sys.path.insert(0, path)
            return resolved
    tried = ", ".join(str(c) for c in candidates)
    raise FileNotFoundError(
        "unitree_sdk2_python not found. Set UNITREE_SDK2PY_PATH or config "
        f"unitree_sdk_path. Tried: {tried}"
    )


_DDS_READY = False


def init_channel(dds_domain: int = 0, network_interface: Optional[str] = None, sdk_path: Optional[Path] = None) -> None:
    """ChannelFactoryInitialize once per process."""
    global _DDS_READY
    ensure_sdk(sdk_path)
    if _DDS_READY:
        return
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    if network_interface:
        ChannelFactoryInitialize(int(dds_domain), network_interface)
    else:
        ChannelFactoryInitialize(int(dds_domain))
    _DDS_READY = True
