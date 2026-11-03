"""立柱最低点标定：把当前 hispeed.y 记为零位行程。

结果写在实例的 state/lift_zero.json，重启后仍生效。
"""

from __future__ import annotations

import json
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

import instance as instances
from model.robot_profile import active as active_profile


def zero_path() -> Path:
    return instances.path("lift_zero")


_lock = threading.Lock()
_zero_m: Optional[float] = None
_loaded = False


def _default_zero() -> float:
    lift = active_profile().lift
    return 0.0 if lift is None else float(lift.height_zero_m)


def load_zero() -> float:
    global _zero_m, _loaded
    with _lock:
        if _loaded and _zero_m is not None:
            return float(_zero_m)
        zero = _default_zero()
        source = "default"
        if zero_path().is_file():
            try:
                doc = json.loads(zero_path().read_text(encoding="utf-8"))
                zero = float(doc["height_zero_m"])
                source = "file"
            except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError):
                zero = _default_zero()
                source = "default"
        _zero_m = zero
        _loaded = True
        return float(_zero_m)


def save_zero(height_m: float) -> Dict[str, Any]:
    global _zero_m, _loaded
    zero = float(height_m)
    if not (abs(zero) < 10.0):
        raise ValueError(f"高度异常: {zero}")
    doc = {
        "height_zero_m": zero,
        "saved_at": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
        "source": "frontend",
    }
    zero_path().parent.mkdir(parents=True, exist_ok=True)
    zero_path().write_text(json.dumps(doc, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    with _lock:
        _zero_m = zero
        _loaded = True
    return {"ok": True, **doc, "path": instances.active().to_stored(zero_path())}


def status() -> Dict[str, Any]:
    zero = load_zero()
    from_file = zero_path().is_file()
    saved_at = None
    if from_file:
        try:
            saved_at = json.loads(zero_path().read_text(encoding="utf-8")).get("saved_at")
        except (OSError, json.JSONDecodeError, TypeError):
            saved_at = None
    return {
        "height_zero_m": zero,
        "height_zero_mm": zero * 1000.0,
        "from_file": from_file,
        "saved_at": saved_at,
        "default_m": _default_zero(),
    }
