"""采集目录：left/NNNN.jpg、right/NNNN.jpg、joints/NNNN.json、session_meta.json。"""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

ROOT = Path(__file__).resolve().parent
DATA_DIR = ROOT / "data"

ARM = "left"
TIP_LINK = "left_wrist_yaw_link"
BASE_LINK = "torso_link"
JOINT_NAMES = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
)


def _layout(arm: str):
    from model.layout import get_arm
    return get_arm(arm)


def list_sessions() -> List[Dict[str, Any]]:
    if not DATA_DIR.is_dir():
        return []
    out = []
    for d in sorted(DATA_DIR.iterdir(), reverse=True):
        if not d.is_dir() or not (d / "session_meta.json").is_file():
            continue
        n = len(list((d / "left").glob("*.jpg"))) if (d / "left").is_dir() else 0
        meta = {}
        try:
            meta = json.loads((d / "session_meta.json").read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            meta = {}
        out.append({
            "name": d.name,
            "n": n,
            "arm": meta.get("arm") or ARM,
            "tip_link": meta.get("tip_link") or TIP_LINK,
            "board_size": meta.get("board_size") or "11x8",
            "poseset": meta.get("poseset") or "",
        })
    return out


def session_dir(name: str) -> Path:
    name = Path(str(name)).name
    if not name or name in (".", ".."):
        raise ValueError("会话名不对")
    return DATA_DIR / name


def create_session(board_size: str, arm: str = ARM, poseset: str = "") -> Path:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    name = datetime.now().strftime("%Y%m%d_%H%M%S") + f"_{arm}"
    path = DATA_DIR / name
    path.mkdir(parents=True, exist_ok=False)
    for sub in ("left", "right", "joints"):
        (path / sub).mkdir()
    write_meta(path, board_size, arm=arm, poseset=poseset)
    return path


def write_meta(path: Path, board_size: str, arm: Optional[str] = None, poseset: Optional[str] = None) -> None:
    old = load_meta(path)
    arm = arm or old.get("arm") or ARM
    layout = _layout(arm)
    if poseset is None:
        poseset = old.get("poseset") or ""
    meta = {
        "created": old.get("created") or datetime.now().isoformat(timespec="seconds"),
        "board_size": board_size,
        "arm": layout.name,
        "tip_link": layout.tip_link,
        "base_link": BASE_LINK,
        "poseset": poseset,
        "joint_source": "lowstate",
        "joint_names": list(layout.joint_names),
        "note": "left/NNNN.jpg 与 joints/NNNN.json 同一序号。末端是该臂的腕 yaw。对侧用姿态集镜像后再采集。",
    }
    (path / "session_meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")


def save_sample(path: Path, left_img, right_img, q, arm: str, pose_name: str = "") -> int:
    import cv2
    layout = _layout(arm)
    idx = next_index(path)
    stem = f"{idx:04d}"
    left_file = path / "left" / f"{stem}.jpg"
    right_file = path / "right" / f"{stem}.jpg"
    joints_file = path / "joints" / f"{stem}.json"
    if not cv2.imwrite(str(left_file), left_img) or not cv2.imwrite(str(right_file), right_img):
        left_file.unlink(missing_ok=True)
        right_file.unlink(missing_ok=True)
        raise OSError("写图片失败")
    record = {
        "index": idx,
        "timestamp": datetime.now().timestamp(),
        "datetime": datetime.now().isoformat(timespec="seconds"),
        "joint_source": "lowstate",
        "arm": layout.name,
        "tip_link": layout.tip_link,
        "pose": pose_name,
        "joint_names": list(layout.joint_names),
        "q_rad": [float(v) for v in q],
        "image": f"left/{stem}.jpg",
        "image_right": f"right/{stem}.jpg",
    }
    try:
        joints_file.write_text(json.dumps(record, indent=2, ensure_ascii=False), encoding="utf-8")
    except OSError:
        left_file.unlink(missing_ok=True)
        right_file.unlink(missing_ok=True)
        raise
    return idx


def next_index(path: Path) -> int:
    used = [int(p.stem) for p in (path / "left").glob("*.jpg") if p.stem.isdigit()]
    return (max(used) + 1) if used else 0


def load_meta(path: Path) -> Dict[str, Any]:
    f = path / "session_meta.json"
    if not f.is_file():
        return {}
    return json.loads(f.read_text(encoding="utf-8"))


def ensure_session(name: Optional[str], board_size: str) -> Path:
    if name:
        path = session_dir(name)
        if not path.is_dir():
            raise FileNotFoundError(f"没有会话 {name}")
        return path
    return create_session(board_size)
