"""单姿态：一个名字对应一只手的一个姿态，存在实例的 calib/single_poses/<名字>.json。

记录时存实测关节角 q 和末端（法兰）在躯干系的位姿 p、R；到位时按「已应用的 IK 策略」
（工作姿态那边的 single / robust）解法兰位姿，记下的 q 作为多初值之一。
"""

from __future__ import annotations

import json
import re
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from robot_platform.core import instance as instances

_NAME_RE = re.compile(r"^[\w\-一-鿿.]{1,48}$")
ARMS = ("left", "right")


def poses_dir() -> Path:
    return instances.path("single_poses")


def check_name(name: str) -> str:
    name = (name or "").strip()
    if not _NAME_RE.match(name):
        raise ValueError("姿态名字只能用中英文、数字、_ - .，1～48 个字符")
    return name


def check_arm(arm: str) -> str:
    arm = str(arm or "").strip()
    if arm not in ARMS:
        raise ValueError("arm 只能是 left 或 right")
    return arm


def pose_path(name: str) -> Path:
    return poses_dir() / f"{check_name(name)}.json"


def _summary(d: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "name": d.get("name"),
        "arm": d.get("arm"),
        "note": d.get("note", ""),
        "p": d.get("p"),
        "tcp_name": d.get("tcp_name", ""),
        "tcp_xyz": d.get("tcp_xyz"),
        "created": d.get("created"),
        "updated": d.get("updated"),
    }


def list_poses(arm: Optional[str] = None) -> List[Dict[str, Any]]:
    d = poses_dir()
    if not d.is_dir():
        return []
    out = []
    for p in sorted(d.glob("*.json"), key=lambda x: x.stat().st_mtime):
        try:
            doc = json.loads(p.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if arm and doc.get("arm") != arm:
            continue
        out.append(_summary(doc))
    return out


def load(name: str) -> Dict[str, Any]:
    p = pose_path(name)
    if not p.is_file():
        raise FileNotFoundError(f"没有姿态「{name}」")
    return json.loads(p.read_text(encoding="utf-8"))


def _write(doc: Dict[str, Any]) -> None:
    poses_dir().mkdir(parents=True, exist_ok=True)
    doc["updated"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    pose_path(doc["name"]).write_text(json.dumps(doc, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def record(name: str, arm: str, q, p, R, *, tcp_name: str = "", tcp_xyz=None, note: str = "",
           overwrite: bool = False) -> Dict[str, Any]:
    """记一个姿态：q 实测关节角，p / R 法兰在躯干系的位姿。"""
    name = check_name(name)
    arm = check_arm(arm)
    q = np.asarray(q, dtype=float).reshape(-1)
    if q.size != 7 or not np.all(np.isfinite(q)):
        raise ValueError("需要 7 个有限的关节角")
    if pose_path(name).exists() and not overwrite:
        raise ValueError(f"姿态「{name}」已存在，换个名字或先删掉")
    now = time.strftime("%Y-%m-%dT%H:%M:%S")
    doc = {
        "name": name,
        "arm": arm,
        "note": str(note or "")[:80],
        "q": [round(float(v), 6) for v in q],
        "p": [round(float(v), 6) for v in np.asarray(p, dtype=float).reshape(3)],
        "R": [round(float(v), 8) for v in np.asarray(R, dtype=float).reshape(9)],
        "tcp_name": str(tcp_name or ""),
        "tcp_xyz": None if tcp_xyz is None else [round(float(v), 6) for v in np.asarray(tcp_xyz, dtype=float).reshape(3)],
        "created": now,
    }
    _write(doc)
    return doc


def rename(old: str, new: str) -> Dict[str, Any]:
    doc = load(old)
    new = check_name(new)
    if new == doc["name"]:
        return doc
    if pose_path(new).exists():
        raise ValueError(f"姿态「{new}」已存在")
    old_path = pose_path(doc["name"])
    doc["name"] = new
    _write(doc)
    old_path.unlink()
    return doc


def delete(name: str) -> None:
    p = pose_path(name)
    if not p.is_file():
        raise FileNotFoundError(f"没有姿态「{name}」")
    p.unlink()
