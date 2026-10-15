"""躯干坐标系里的命名空间点，以及让 TCP 碰到该点的关节解。"""

from __future__ import annotations

import json
import re
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parent
POINTS_DIR = ROOT / "config" / "spatial_points"
_ACTIVE_PATH = POINTS_DIR / "_active.json"
_NAME_RE = re.compile(r"^[\w\-\u4e00-\u9fff.]{1,48}$")


def _check_name(name: str) -> str:
    name = (name or "").strip()
    if not _NAME_RE.match(name):
        raise ValueError("名字只能用中英文、数字、_ - .，1～48 个字符")
    return name


def _path(name: str) -> Path:
    return POINTS_DIR / f"{_check_name(name)}.json"


def list_saved() -> List[Dict[str, Any]]:
    if not POINTS_DIR.is_dir():
        return []
    out = []
    for path in sorted(POINTS_DIR.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True):
        if path.name.startswith("_"):
            continue
        try:
            doc = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        out.append({
            "name": doc.get("name") or path.stem,
            "arm": doc.get("arm") or "",
            "tcp_name": doc.get("tcp_name") or "",
            "point_mm": doc.get("point_mm") or [],
            "note": doc.get("note") or "",
            "saved_at": doc.get("saved_at") or "",
        })
    return out


def _read_active() -> str:
    if not _ACTIVE_PATH.is_file():
        return ""
    try:
        return str(json.loads(_ACTIVE_PATH.read_text(encoding="utf-8")).get("name") or "")
    except (OSError, json.JSONDecodeError):
        return ""


def _write_active(name: str) -> None:
    POINTS_DIR.mkdir(parents=True, exist_ok=True)
    if not name:
        if _ACTIVE_PATH.is_file():
            _ACTIVE_PATH.unlink()
        return
    _ACTIVE_PATH.write_text(json.dumps({"name": name}, ensure_ascii=False, indent=2), encoding="utf-8")


def solve_tcp_ik(model, q0, tcp_m, target_m) -> Tuple[np.ndarray, float]:
    """求一组靠近 q0 的关节角，使腕位姿下的 TCP 落到 target。返回 (q, 终点误差 mm)。"""
    q = np.asarray(q0, dtype=float).reshape(-1).copy()
    tcp = np.asarray(tcp_m, dtype=float).reshape(3)
    target = np.asarray(target_m, dtype=float).reshape(3)
    limits = model.joint_limits()
    eye = np.eye(3)
    for _ in range(80):
        pos, jac = _tcp_pos_jac(model, q, tcp)
        err = target - pos
        if float(np.linalg.norm(err)) < 2e-4:
            break
        jjt = jac @ jac.T + 1e-3 * eye
        dq = jac.T @ np.linalg.solve(jjt, err)
        null = np.eye(model.n) - jac.T @ np.linalg.solve(jjt, jac)
        dq = dq + null @ (0.15 * (np.asarray(q0, dtype=float).reshape(-1) - q))
        step = float(np.linalg.norm(dq))
        if step > 0.12:
            dq *= 0.12 / step
        q = np.clip(q + dq, limits[:, 0], limits[:, 1])
    pos, _ = _tcp_pos_jac(model, q, tcp)
    miss_mm = float(np.linalg.norm(pos - target) * 1000.0)
    return q, miss_mm


def _tcp_pos_jac(model, q, tcp):
    t, R = model.fk(q)
    pos = t + R @ tcp
    jac = np.zeros((3, model.n))
    eps = 1e-5
    for i in range(model.n):
        dq = np.zeros(model.n)
        dq[i] = eps
        t2, R2 = model.fk(q + dq)
        jac[:, i] = ((t2 + R2 @ tcp) - pos) / eps
    return pos, jac


class SpatialPoints:
    def __init__(self) -> None:
        self.active_name = ""
        self.doc: Optional[Dict[str, Any]] = None

    def status(self) -> Dict[str, Any]:
        doc = self.doc or {}
        return {
            "ok": True,
            "active_name": self.active_name,
            "arm": doc.get("arm") or "",
            "tcp_name": doc.get("tcp_name") or "",
            "point_m": doc.get("point_m"),
            "point_mm": doc.get("point_mm"),
            "note": doc.get("note") or "",
            "saved": list_saved(),
        }

    def restore_active(self) -> None:
        name = _read_active()
        if name and _path(name).is_file():
            self._apply(json.loads(_path(name).read_text(encoding="utf-8")))

    def save(self, name: str, arm: str, tcp_name: str, tcp_m, point_m, note: str = "") -> Dict[str, Any]:
        name = _check_name(name)
        point = np.asarray(point_m, dtype=float).reshape(3)
        tcp = np.asarray(tcp_m, dtype=float).reshape(3)
        if not np.all(np.isfinite(point)) or not np.all(np.isfinite(tcp)):
            raise ValueError("空间点或 TCP 偏置不是有限数")
        POINTS_DIR.mkdir(parents=True, exist_ok=True)
        doc = {
            "name": name,
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "arm": arm,
            "tcp_name": tcp_name,
            "tcp_m": tcp.tolist(),
            "point_m": point.tolist(),
            "point_mm": (point * 1000.0).tolist(),
            "note": (note or "").strip(),
            "base_link": "torso_link",
        }
        _path(name).write_text(json.dumps(doc, indent=2, ensure_ascii=False), encoding="utf-8")
        self._apply(doc)
        _write_active(name)
        return self.status()

    def load(self, name: str) -> Dict[str, Any]:
        path = _path(name)
        if not path.is_file():
            raise FileNotFoundError(f"没有空间点 {name}")
        self._apply(json.loads(path.read_text(encoding="utf-8")))
        _write_active(self.active_name)
        return self.status()

    def delete(self, name: str) -> Dict[str, Any]:
        path = _path(name)
        if not path.is_file():
            raise FileNotFoundError(f"没有空间点 {name}")
        path.unlink()
        if self.active_name == name:
            self.doc = None
            self.active_name = ""
            _write_active("")
        return self.status()

    def _apply(self, doc: Dict[str, Any]) -> None:
        self.doc = doc
        self.active_name = str(doc.get("name") or "")
