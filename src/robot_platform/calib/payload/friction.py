"""摩擦停驻带：用双向逼近测量里 +/- 落点差的一半，作为到位修正的逐关节死区。"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from robot_platform.calib.payload.identify import measure_dir

N_JOINT = 7
MAX_SESSIONS = 12
FALLBACK_BAND_DEG = 0.30


class BandTable:
    def __init__(self, per_pose: Dict[str, List[float]], default: List[float], meta: Dict):
        self.per_pose = per_pose
        self.default = np.asarray(default, float)
        self.meta = meta

    def for_pose(self, name: Optional[str] = None) -> np.ndarray:
        v = self.per_pose.get(name or "")
        return np.asarray(v, float) if v else self.default

    def to_dict(self) -> Dict:
        return {
            **self.meta,
            "default_deg": self.default.round(3).tolist(),
            "per_pose_deg": {k: np.round(v, 3).tolist() for k, v in self.per_pose.items()},
        }


def _bidirectional(meta: Dict) -> bool:
    if meta.get("bidirectional"):
        return True
    return bool((meta.get("opts") or {}).get("bidirectional"))


def _sessions(arm: str, poseset: Optional[str]) -> List[Path]:
    if not measure_dir().exists():
        return []
    out = []
    for d in sorted(measure_dir().iterdir(), reverse=True):
        meta_p, samp_p = d / "meta.json", d / "samples.json"
        if not (meta_p.exists() and samp_p.exists()):
            continue
        try:
            meta = json.loads(meta_p.read_text(encoding="utf-8"))
        except Exception:
            continue
        if meta.get("arm") != arm or not _bidirectional(meta):
            continue
        if poseset and meta.get("poseset") != poseset:
            continue
        out.append(d)
        if len(out) >= MAX_SESSIONS:
            break
    return out


def _session_bands(d: Path, kp_now: Optional[np.ndarray]) -> Dict[str, np.ndarray]:
    meta = json.loads((d / "meta.json").read_text(encoding="utf-8"))
    samples = json.loads((d / "samples.json").read_text(encoding="utf-8"))
    kp_meas = np.asarray((meta.get("gains") or {}).get("kp") or [], float)
    if kp_meas.size != N_JOINT and samples:
        kp_meas = np.asarray(samples[0].get("kp") or [], float)
    scale = np.ones(N_JOINT)
    if kp_now is not None and kp_meas.size == N_JOINT and np.all(kp_meas > 0) and np.all(kp_now > 0):
        scale = kp_meas / kp_now
    by_pose: Dict[str, Dict[str, np.ndarray]] = {}
    for s in samples:
        q = s.get("q_meas")
        if q and len(q) == N_JOINT:
            by_pose.setdefault(s.get("name", ""), {})[s.get("approach", "")] = np.asarray(q, float)
    out = {}
    for name, v in by_pose.items():
        if "+" in v and "-" in v:
            out[name] = np.abs(v["+"] - v["-"]) / 2 * 180 / math.pi * scale
    return out


def estimate(arm: str, poseset: Optional[str] = None, kp=None) -> Optional[BandTable]:
    kp_now = np.asarray(kp, float) if kp is not None and len(kp) == N_JOINT else None
    acc: Dict[str, List[np.ndarray]] = {}
    used = []
    for d in _sessions(arm, poseset):
        try:
            bands = _session_bands(d, kp_now)
        except Exception:
            continue
        if not bands:
            continue
        used.append(d.name)
        for name, b in bands.items():
            acc.setdefault(name, []).append(b)
    if not acc:
        return None
    per_pose = {name: np.median(np.stack(v), axis=0) for name, v in acc.items()}
    default = np.median(np.stack(list(per_pose.values())), axis=0)
    meta = {
        "arm": arm, "poseset": poseset,
        "kp": None if kp_now is None else kp_now.tolist(),
        "n_sessions": len(used), "n_poses": len(per_pose), "sessions": used,
        "fallback": False,
    }
    return BandTable({k: v.tolist() for k, v in per_pose.items()}, default.tolist(), meta)


def fallback_table(kp=None) -> BandTable:
    kp_now = np.asarray(kp, float) if kp is not None and len(kp) == N_JOINT else None
    b = np.full(N_JOINT, FALLBACK_BAND_DEG)
    if kp_now is not None and np.all(kp_now > 0):
        b = b * (40.0 / kp_now)
    return BandTable({}, b.tolist(), {
        "fallback": True,
        "kp": None if kp_now is None else kp_now.tolist(),
        "n_sessions": 0, "n_poses": 0, "sessions": [],
    })


def table_for(arm: str, poseset: Optional[str] = None, kp=None) -> BandTable:
    for ps in ([poseset, None] if poseset else [None]):
        t = estimate(arm, ps, kp)
        if t is not None:
            if ps is None and poseset:
                t.meta["poseset_fallback"] = poseset
            return t
    return fallback_table(kp)
