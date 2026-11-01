"""工作姿态：与 TCP 绑定的腕部方向（躯干系 R），可锁定供碰点/棋盘使用。"""

from __future__ import annotations

import json
import re
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

ROOT = Path(__file__).resolve().parent
POSES_DIR = ROOT / "config" / "work_poses"
IK_MODE_PATH = ROOT / "config" / "pose_ik_mode.json"
# single=旧：当前角一枪局部 IK；robust=新：多初值全姿态 IK
IK_MODES = ("single", "robust")
_NAME_RE = re.compile(r"^[\w\-\u4e00-\u9fff.]{1,48}$")
_TCP_DIR_RE = re.compile(r"^[\w\-\u4e00-\u9fff.]{1,48}$")


def _read_ik_mode() -> str:
    try:
        if IK_MODE_PATH.is_file():
            mode = str(json.loads(IK_MODE_PATH.read_text(encoding="utf-8")).get("mode") or "")
            if mode in IK_MODES:
                return mode
    except (OSError, json.JSONDecodeError, TypeError):
        pass
    return "robust"


def _write_ik_mode(mode: str) -> None:
    IK_MODE_PATH.parent.mkdir(parents=True, exist_ok=True)
    IK_MODE_PATH.write_text(
        json.dumps({"mode": mode}, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def _check_name(name: str) -> str:
    name = (name or "").strip()
    if not _NAME_RE.match(name):
        raise ValueError("名字只能用中英文、数字、_ - .，1～48 个字符")
    return name


def _check_tcp_dir(tcp_name: str) -> str:
    tcp_name = (tcp_name or "").strip()
    if not _TCP_DIR_RE.match(tcp_name):
        raise ValueError("TCP 名无效，无法作为工作姿态目录")
    return tcp_name


def _dir(tcp_name: str) -> Path:
    return POSES_DIR / _check_tcp_dir(tcp_name)


def _path(tcp_name: str, name: str) -> Path:
    return _dir(tcp_name) / f"{_check_name(name)}.json"


def list_for_tcp(tcp_name: str) -> List[Dict[str, Any]]:
    if not tcp_name:
        return []
    d = _dir(tcp_name)
    if not d.is_dir():
        return []
    out = []
    for path in sorted(d.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True):
        if path.name.startswith("_"):
            continue
        try:
            doc = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        out.append({
            "name": doc.get("name") or path.stem,
            "tcp_name": doc.get("tcp_name") or tcp_name,
            "arm": doc.get("arm") or "",
            "note": doc.get("note") or "",
            "saved_at": doc.get("saved_at") or "",
        })
    return out


def list_all() -> List[Dict[str, Any]]:
    if not POSES_DIR.is_dir():
        return []
    out: List[Dict[str, Any]] = []
    for d in sorted(POSES_DIR.iterdir()):
        if not d.is_dir():
            continue
        out.extend(list_for_tcp(d.name))
    return out


def delete_all_for_tcp(tcp_name: str) -> int:
    """删除某 TCP 目录下全部工作姿态，返回删除的文件数。"""
    if not tcp_name:
        return 0
    try:
        d = _dir(tcp_name)
    except ValueError:
        return 0
    if not d.is_dir():
        return 0
    n = 0
    for path in list(d.glob("*.json")):
        try:
            path.unlink()
            n += 1
        except OSError:
            continue
    try:
        d.rmdir()
    except OSError:
        pass
    return n


class WorkPoseSession:
    def __init__(self) -> None:
        self.locked = False
        self.active_name = ""
        self.tcp_name = ""
        self.arm = ""
        self.R_lock: Optional[np.ndarray] = None
        self.q_seed: Optional[np.ndarray] = None  # 记下/调用时的关节角，供多初值 IK
        self.note = ""
        self.ik_mode = _read_ik_mode()

    def set_ik_mode(self, mode: str) -> Dict[str, Any]:
        mode = str(mode or "").strip()
        if mode not in IK_MODES:
            raise ValueError("姿态 IK 只能是 single（旧·单次）或 robust（新·多初值）")
        self.ik_mode = mode
        _write_ik_mode(mode)
        return self.status()

    def status(self, tcp_name: str = "") -> Dict[str, Any]:
        tcp = tcp_name or self.tcp_name
        return {
            "ok": True,
            "locked": bool(self.locked),
            # 仅锁定时有名字；未锁定固定为「无」语义（空字符串）
            "active_name": self.active_name if self.locked else "",
            "tcp_name": self.tcp_name if self.locked else "",
            "arm": self.arm if self.locked else "",
            "note": self.note if self.locked else "",
            "R": self.R_lock.tolist() if (self.locked and self.R_lock is not None) else None,
            "ik_mode": self.ik_mode,
            "saved": list_for_tcp(tcp) if tcp else [],
            "saved_all": list_all(),
        }

    def unlock(self) -> Dict[str, Any]:
        self.locked = False
        self.active_name = ""
        self.tcp_name = ""
        self.arm = ""
        self.R_lock = None
        self.q_seed = None
        self.note = ""
        return self.status()

    def unlock_if_tcp_mismatch(self, tcp_name: str) -> bool:
        """当前锁定姿态不属于该 TCP 时解锁。返回是否发生了解锁。"""
        if not self.locked:
            return False
        if (tcp_name or "") != (self.tcp_name or ""):
            self.unlock()
            return True
        return False

    def save(
        self,
        name: str,
        tcp_name: str,
        arm: str,
        R,
        q=None,
        note: str = "",
    ) -> Dict[str, Any]:
        name = _check_name(name)
        tcp_name = _check_tcp_dir(tcp_name)
        arm = "left" if arm == "left" else "right"
        Rm = np.asarray(R, dtype=float).reshape(3, 3)
        if not np.all(np.isfinite(Rm)):
            raise ValueError("姿态旋转不是有限数")
        # 轻微正交化
        u, _, vt = np.linalg.svd(Rm)
        Rm = u @ vt
        if np.linalg.det(Rm) < 0:
            u[:, -1] *= -1
            Rm = u @ vt
        d = _dir(tcp_name)
        d.mkdir(parents=True, exist_ok=True)
        doc: Dict[str, Any] = {
            "name": name,
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "tcp_name": tcp_name,
            "arm": arm,
            "base_link": "torso_link",
            "tip_link": f"{arm}_wrist_yaw_link",
            "R": Rm.tolist(),
            "note": (note or "").strip(),
        }
        if q is not None:
            qa = np.asarray(q, dtype=float).reshape(-1)
            if np.all(np.isfinite(qa)):
                doc["q_rad"] = qa.tolist()
        _path(tcp_name, name).write_text(json.dumps(doc, indent=2, ensure_ascii=False), encoding="utf-8")
        return self.status(tcp_name)

    def load_doc(self, tcp_name: str, name: str) -> Dict[str, Any]:
        path = _path(tcp_name, name)
        if not path.is_file():
            raise FileNotFoundError(f"没有工作姿态 {tcp_name}/{name}")
        return json.loads(path.read_text(encoding="utf-8"))

    def delete(self, tcp_name: str, name: str) -> Dict[str, Any]:
        path = _path(tcp_name, name)
        if not path.is_file():
            raise FileNotFoundError(f"没有工作姿态 {tcp_name}/{name}")
        path.unlink()
        if self.locked and self.active_name == name and self.tcp_name == tcp_name:
            self.unlock()
        return self.status(tcp_name)

    def apply_lock(self, doc: Dict[str, Any], q_now=None) -> None:
        Rm = np.asarray(doc["R"], dtype=float).reshape(3, 3)
        if not np.all(np.isfinite(Rm)):
            raise ValueError("姿态文件里的 R 无效")
        self.locked = True
        self.active_name = str(doc.get("name") or "")
        self.tcp_name = str(doc.get("tcp_name") or "")
        self.arm = str(doc.get("arm") or "")
        self.R_lock = Rm
        self.note = str(doc.get("note") or "")
        self.q_seed = None
        for cand in (q_now, doc.get("q_rad")):
            if cand is None:
                continue
            qa = np.asarray(cand, dtype=float).reshape(-1)
            if np.all(np.isfinite(qa)):
                self.q_seed = qa
                break
