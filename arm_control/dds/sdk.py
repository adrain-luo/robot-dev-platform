"""Load unitree_sdk2py from vendored tree, then the active env."""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Optional

_VENDOR = Path(__file__).resolve().parents[2] / "third_party" / "unitree_sdk2_python"


def _root_from_import() -> Optional[Path]:
    try:
        import unitree_sdk2py
    except ImportError:
        return None
    return Path(unitree_sdk2py.__file__).resolve().parent.parent


def _use_tree(root: Path) -> Path:
    resolved = root.expanduser().resolve()
    path = str(resolved)
    if path not in sys.path:
        sys.path.insert(0, path)
    return resolved


def ensure_sdk(sdk_path: Optional[Path] = None) -> Path:
    """Prefer 本仓库 third_party；仅在显式给出路径时覆盖。"""
    trees = []
    if sdk_path is not None:
        trees.append(Path(sdk_path))
    env = os.environ.get("UNITREE_SDK2PY_PATH")
    if env:
        trees.append(Path(env))
    trees.append(_VENDOR)
    for cand in trees:
        if (cand / "unitree_sdk2py" / "__init__.py").is_file():
            return _use_tree(cand)

    found = _root_from_import()
    if found is not None:
        return found

    raise FileNotFoundError(
        "未找到 unitree_sdk2py。仓库应自带 third_party/unitree_sdk2_python；"
        "也可设置 UNITREE_SDK2PY_PATH。"
    )


_DDS_READY = False


def init_channel(
    dds_domain: int = 0,
    network_interface: Optional[str] = None,
    sdk_path: Optional[Path] = None,
) -> None:
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
