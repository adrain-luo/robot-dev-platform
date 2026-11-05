"""姿态集：实例的 calib/posesets/<名字>.json。示教记录实测角；左右关于躯干矢状面（y=0）镜像。

关节顺序：肩 pitch/roll/yaw、肘、腕 roll/pitch/yaw。
G1D URDF 左右轴定义相同（pitch∥Y、roll∥X、yaw∥Z），限位关于 0 对调。
镜像：绕 Y 的角不变，绕 X/Z 的角取反 → 符号 [1,-1,-1, 1,-1, 1,-1]（机型 robot.json 的 mirror_q_sign）。
"""

from __future__ import annotations

import json
import re
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from robot_platform.control.arm_gravity import ArmGravity
from robot_platform.core.layout import get_arm
from robot_platform.core.robot_profile import active as active_profile

from robot_platform.core import instance as instances


def sets_dir() -> Path:
    return instances.path("posesets")
_NAME_RE = re.compile(r"^[\w\-\u4e00-\u9fff.]{1,48}$")

def _mirror_sign() -> np.ndarray:
    """机型 robot.json 的 mirror_q_sign（G1D：pitch 同号，roll/yaw 反号）。"""
    sign = active_profile().mirror_q_sign
    if not sign:
        raise ValueError(f"{active_profile().name} 没有配置 mirror_q_sign，不能镜像")
    return np.asarray(sign, float)


def _home_q(arm: str) -> List[float]:
    layout = get_arm(arm)
    return list(layout.home_q) if layout.home_q else [0.0] * layout.n


def _check_name(name: str) -> str:
    name = (name or "").strip()
    if not _NAME_RE.match(name):
        raise ValueError("姿态集名字只能用中英文、数字、_ - .，1～48 个字符")
    return name


def set_path(name: str) -> Path:
    return sets_dir() / f"{_check_name(name)}.json"


def mirror_q(q) -> List[float]:
    sign = _mirror_sign()
    return [round(float(v), 5) + 0.0 for v in np.asarray(q, float).reshape(sign.size) * sign]


def condition_number(arm: str, qs: List, urdf_path: Path, base_link: str) -> Optional[float]:
    if len(qs) < 2:
        return None
    grav = ArmGravity(arm, urdf_path, base_link)
    Y = np.vstack([grav.payload_regressor(np.asarray(q, float)) for q in qs])
    scale = np.linalg.norm(Y, axis=0) + 1e-9
    return float(np.linalg.cond(Y / scale))


def list_sets(arm: Optional[str] = None) -> List[Dict]:
    sets_dir().mkdir(parents=True, exist_ok=True)
    out = []
    for p in sorted(sets_dir().glob("*.json"), key=lambda x: x.stat().st_mtime, reverse=True):
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
        except Exception:
            continue
        if arm and d.get("arm") != arm:
            continue
        out.append({
            "name": d.get("name", p.stem),
            "arm": d.get("arm"),
            "n": len(d.get("poses", [])),
            "note": d.get("note", ""),
            "cond": d.get("regressor_cond"),
            "mirrored_from": d.get("mirrored_from"),
        })
    return out


def load_set(name: str) -> Dict:
    p = set_path(name)
    if not p.exists():
        raise FileNotFoundError(f"没有姿态集 {name}")
    return json.loads(p.read_text(encoding="utf-8"))


def save_set(d: Dict, urdf_path: Optional[Path] = None, base_link: str = "torso_link") -> Path:
    sets_dir().mkdir(parents=True, exist_ok=True)
    qs = [p["q"] for p in d.get("poses", [])]
    if urdf_path is not None and len(qs) >= 2:
        d["regressor_cond"] = round(condition_number(d["arm"], qs, urdf_path, base_link), 2)
    elif len(qs) < 2:
        d["regressor_cond"] = None
    d["updated"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    p = set_path(d["name"])
    p.write_text(json.dumps(d, indent=2, ensure_ascii=False), encoding="utf-8")
    return p


def create_set(name: str, arm: str, note: str = "") -> Dict:
    name = _check_name(name)
    if set_path(name).exists():
        raise ValueError(f"姿态集 {name} 已存在")
    get_arm(arm)
    d = {
        "name": name,
        "arm": arm,
        "note": note,
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "home_q": _home_q(arm),
        "poses": [],
    }
    save_set(d)
    return d


def delete_set(name: str) -> None:
    p = set_path(name)
    if not p.exists():
        raise FileNotFoundError(f"没有姿态集 {name}")
    p.unlink()


def add_pose(name: str, q, urdf_path: Path, base_link: str, pose_name: Optional[str] = None,
             source: str = "teach") -> Dict:
    d = load_set(name)
    q = [round(float(v), 5) for v in np.asarray(q, float).reshape(-1)]
    if len(q) != 7:
        raise ValueError("需要 7 个关节角")
    grav = ArmGravity(d["arm"], urdf_path, base_link)
    pose = {
        "name": pose_name or f"p{len(d['poses']):02d}",
        "q": q,
        "tip_xyz": [round(float(v), 4) for v in grav.tip_T(q)[:3, 3]],
        "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "source": source,
    }
    d["poses"].append(pose)
    save_set(d, urdf_path, base_link)
    return {"pose": pose, "set": d}


def delete_pose(name: str, index: int, urdf_path: Optional[Path] = None, base_link: str = "torso_link") -> Dict:
    d = load_set(name)
    if not (0 <= index < len(d["poses"])):
        raise IndexError("姿态下标越界")
    d["poses"].pop(index)
    save_set(d, urdf_path, base_link)
    return d


def mirror_set(src: str, urdf_path: Path, base_link: str, name: Optional[str] = None,
               note: str = "", force: bool = False) -> Dict:
    d = load_set(src)
    src_arm = d.get("arm")
    dst_arm = "left" if src_arm == "right" else "right"
    grav = ArmGravity(dst_arm, urdf_path, base_link)
    lo, hi = grav.limits[:, 0], grav.limits[:, 1]
    now = time.strftime("%Y-%m-%dT%H:%M:%S")
    poses = []
    for p in d.get("poses", []):
        q = np.asarray(mirror_q(p["q"]), float)
        bad = np.where((q < lo - 1e-6) | (q > hi + 1e-6))[0]
        if len(bad):
            raise ValueError(f"姿态 {p.get('name')} 镜像后超出 {dst_arm} 臂限位（关节 {bad.tolist()}）")
        poses.append({
            "name": p.get("name", f"p{len(poses):02d}"),
            "q": q.round(5).tolist(),
            "tip_xyz": [round(float(v), 4) for v in grav.tip_T(q)[:3, 3]],
            "recorded_at": now,
            "source": f"mirror:{p.get('source', '?')}",
        })
    new_name = _check_name(name or f"{d.get('name', 'set')}_{'左' if dst_arm == 'left' else '右'}")
    out_d = {
        "name": new_name,
        "arm": dst_arm,
        "note": note or f"由{src_arm}臂「{d.get('name')}」关于 y=0 镜像。",
        "created": now,
        "mirrored_from": {"name": d.get("name"), "arm": src_arm},
        "home_q": mirror_q(d.get("home_q", _home_q(src_arm))),
        "poses": poses,
    }
    path = set_path(new_name)
    if path.exists() and not force:
        raise FileExistsError(f"姿态集 {new_name} 已存在")
    save_set(out_d, urdf_path, base_link)
    return out_d
