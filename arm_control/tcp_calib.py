"""固定点 TCP：腕部转开时工作点不动，求工作点在腕坐标系里的位置。

命名结果写在 config/tcp_results/{名字}.json，可再调用。"""

from __future__ import annotations

import json
import re
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

ROOT = Path(__file__).resolve().parent
RESULTS_DIR = ROOT / "config" / "tcp_results"
_ACTIVE_PATH = RESULTS_DIR / "_active.json"
_NAME_RE = re.compile(r"^[\w\-\u4e00-\u9fff.]{1,48}$")
# 躯干 y=0 镜像：位置/固定点反射；腕系偏置 y 取反；关节同姿态集符号
_MIRROR_S = np.diag([1.0, -1.0, 1.0])
_MIRROR_Q_SIGN = np.array([1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0])


def _check_name(name: str) -> str:
    name = (name or "").strip()
    if not _NAME_RE.match(name):
        raise ValueError("名字只能用中英文、数字、_ - .，1～48 个字符")
    return name


def _result_path(name: str) -> Path:
    return RESULTS_DIR / f"{_check_name(name)}.json"


def _mirror_dst_name(src_name: str, dst_arm: str) -> str:
    suffix = "左" if dst_arm == "left" else "右"
    return _check_name(f"{src_name}_{suffix}")


def _mirror_vec3(v) -> List[float]:
    a = np.asarray(v, dtype=float).reshape(3)
    return (_MIRROR_S @ a).tolist()


def _mirror_R(R) -> List[List[float]]:
    Rm = _MIRROR_S @ np.asarray(R, dtype=float).reshape(3, 3) @ _MIRROR_S
    return Rm.tolist()


def _mirror_q(q) -> List[float]:
    qa = np.asarray(q, dtype=float).reshape(-1)
    if qa.size != 7:
        raise ValueError("镜像需要 7 轴关节角样本")
    return (qa * _MIRROR_Q_SIGN).tolist()


def list_saved() -> List[Dict[str, Any]]:
    if not RESULTS_DIR.is_dir():
        return []
    out = []
    for path in sorted(RESULTS_DIR.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True):
        if path.name.startswith("_"):
            continue
        try:
            doc = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        residual = (doc.get("result") or {}).get("residual_mm") or {}
        out.append({
            "name": doc.get("name") or path.stem,
            "arm": doc.get("arm") or "",
            "tip_link": doc.get("tip_link") or "",
            "n": doc.get("n") or len(doc.get("samples") or []),
            "residual_max_mm": residual.get("max"),
            "note": doc.get("note") or "",
            "saved_at": doc.get("saved_at") or "",
        })
    return out


def _read_active_map() -> Dict[str, str]:
    """左右臂各自生效的 TCP 名。兼容旧格式 {"name": "..."}。"""
    out = {"left": "", "right": ""}
    if not _ACTIVE_PATH.is_file():
        return out
    try:
        doc = json.loads(_ACTIVE_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return out
    if not isinstance(doc, dict):
        return out
    if "left" in doc or "right" in doc:
        out["left"] = str(doc.get("left") or "")
        out["right"] = str(doc.get("right") or "")
        return out
    # 旧版只有一个 name：按结果文件里的 arm 归位
    name = str(doc.get("name") or "")
    if not name:
        return out
    path = RESULTS_DIR / f"{name}.json"
    arm = ""
    if path.is_file():
        try:
            arm = str(json.loads(path.read_text(encoding="utf-8")).get("arm") or "")
        except (OSError, json.JSONDecodeError):
            arm = ""
    if arm in ("left", "right"):
        out[arm] = name
    else:
        out["right"] = name
    return out


def _write_active_map(active: Dict[str, str]) -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    left = str(active.get("left") or "")
    right = str(active.get("right") or "")
    if not left and not right:
        if _ACTIVE_PATH.is_file():
            _ACTIVE_PATH.unlink()
        return
    _ACTIVE_PATH.write_text(
        json.dumps({"left": left, "right": right}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def _set_active_for_arm(arm: str, name: str) -> None:
    arm = "left" if arm == "left" else "right"
    m = _read_active_map()
    m[arm] = str(name or "")
    _write_active_map(m)


def _clear_active_name(name: str) -> None:
    m = _read_active_map()
    changed = False
    for arm in ("left", "right"):
        if m.get(arm) == name:
            m[arm] = ""
            changed = True
    if changed:
        _write_active_map(m)


def solve_pivot(samples: List[Dict[str, Any]]) -> Dict[str, Any]:
    """每个样本带腕在躯干下的 R、t。返回工作点（腕系）和固定点（躯干系）。"""
    n = len(samples)
    if n < 3:
        raise ValueError("至少记 3 个姿态，而且每次要把腕转开")
    A = np.zeros((3 * n, 6))
    b = np.zeros(3 * n)
    rots = []
    for i, sample in enumerate(samples):
        R = np.asarray(sample["R"], dtype=float).reshape(3, 3)
        t = np.asarray(sample["t_m"], dtype=float).reshape(3)
        rots.append(R)
        A[3 * i : 3 * i + 3, 0:3] = R
        A[3 * i : 3 * i + 3, 3:6] = -np.eye(3)
        b[3 * i : 3 * i + 3] = -t
    sol, _, _, svals = np.linalg.lstsq(A, b, rcond=None)
    tcp = sol[:3]
    fixed = sol[3:]
    err_mm = []
    for sample in samples:
        R = np.asarray(sample["R"], dtype=float).reshape(3, 3)
        t = np.asarray(sample["t_m"], dtype=float).reshape(3)
        err_mm.append(float(np.linalg.norm(R @ tcp + t - fixed) * 1000.0))
    err = np.asarray(err_mm)
    spread = 0.0
    for i in range(n):
        for j in range(i + 1, n):
            R = rots[i] @ rots[j].T
            c = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
            spread = max(spread, float(np.degrees(np.arccos(c))))
    warning = ""
    if spread < 25.0:
        warning = f"腕部最大只转开 {spread:.0f}°，沿转轴方向的工作点会不准。"
    if float(svals[-1]) < 1e-3:
        warning = (warning + " " if warning else "") + "这组姿态解不出完整的工作点，再换几个方向。"
    return {
        "tcp_m": tcp.tolist(),
        "tcp_mm": (tcp * 1000.0).tolist(),
        "fixed_point_m": fixed.tolist(),
        "fixed_mm": (fixed * 1000.0).tolist(),
        "residual_mm": {"mean": float(err.mean()), "max": float(err.max()), "each": err_mm},
        "wrist_spread_deg": spread,
        "warning": warning,
    }


class TcpSession:
    def __init__(self) -> None:
        self.arm = ""
        self.tip_link = ""
        self.samples: List[Dict[str, Any]] = []
        self.result: Optional[Dict[str, Any]] = None
        self.active_name = ""
        self.note = ""

    def status(self) -> Dict[str, Any]:
        active = _read_active_map()
        return {
            "ok": True,
            "arm": self.arm,
            "tip_link": self.tip_link,
            "n": len(self.samples),
            "result": self.result,
            "active_name": self.active_name,
            "active_left": active.get("left") or "",
            "active_right": active.get("right") or "",
            "note": self.note,
            "saved": list_saved(),
        }

    def restore_active(self, arm: Optional[str] = None) -> None:
        """恢复某臂生效的 TCP；未指定臂时优先右、再左。"""
        active = _read_active_map()
        if arm in ("left", "right"):
            name = active.get(arm) or ""
        else:
            name = active.get("right") or active.get("left") or ""
        if not name or not _result_path(name).is_file():
            return
        self._apply_doc(json.loads(_result_path(name).read_text(encoding="utf-8")))

    def restore_for_arm(self, arm: str) -> None:
        self.restore_active(arm)

    def save(self, name: str, note: str = "") -> Dict[str, Any]:
        if self.result is None:
            raise ValueError("先计算，再命名保存")
        if self.arm not in ("left", "right"):
            raise ValueError("先记录某一侧手臂的姿态")
        name = _check_name(name)
        RESULTS_DIR.mkdir(parents=True, exist_ok=True)
        doc = {
            "name": name,
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "arm": self.arm,
            "tip_link": self.tip_link,
            "base_link": "torso_link",
            "note": (note or "").strip(),
            "n": len(self.samples),
            "samples": self.samples,
            "result": self.result,
        }
        _result_path(name).write_text(json.dumps(doc, indent=2, ensure_ascii=False), encoding="utf-8")
        self.active_name = name
        self.note = doc["note"]
        _set_active_for_arm(self.arm, name)
        return self.status()

    def load(self, name: str) -> Dict[str, Any]:
        path = _result_path(name)
        if not path.is_file():
            raise FileNotFoundError(f"没有 TCP 结果 {name}")
        self._apply_doc(json.loads(path.read_text(encoding="utf-8")))
        if self.arm not in ("left", "right"):
            raise ValueError(f"「{name}」没有左右臂标记，不能调用")
        _set_active_for_arm(self.arm, self.active_name)
        return self.status()

    def delete(self, name: str) -> Dict[str, Any]:
        path = _result_path(name)
        if not path.is_file():
            raise FileNotFoundError(f"没有 TCP 结果 {name}")
        path.unlink()
        _clear_active_name(name)
        if self.active_name == name:
            self.samples.clear()
            self.arm = ""
            self.tip_link = ""
            self.result = None
            self.active_name = ""
            self.note = ""
        return self.status()

    def mirror(self, name: str, force: bool = False) -> Dict[str, Any]:
        """关于躯干 y=0 镜像到另一只手，生成新名「原名_左/右」。不改当前应用。"""
        src_name = _check_name(name)
        path = _result_path(src_name)
        if not path.is_file():
            raise FileNotFoundError(f"没有 TCP 结果 {src_name}")
        doc = json.loads(path.read_text(encoding="utf-8"))
        src_arm = str(doc.get("arm") or "")
        if src_arm not in ("left", "right"):
            raise ValueError(f"「{src_name}」没有左右臂标记，不能镜像")
        if not doc.get("result") or doc["result"].get("tcp_m") is None:
            raise ValueError(f"「{src_name}」没有可用的 TCP 结果")
        dst_arm = "left" if src_arm == "right" else "right"
        dst_name = _mirror_dst_name(src_name, dst_arm)
        dst_path = _result_path(dst_name)
        if dst_path.is_file() and not force:
            raise FileExistsError(f"TCP「{dst_name}」已存在")
        tip = f"{dst_arm}_wrist_yaw_link"
        samples_out: List[Dict[str, Any]] = []
        for sample in doc.get("samples") or []:
            samples_out.append({
                "q_rad": _mirror_q(sample.get("q_rad")),
                "R": _mirror_R(sample.get("R")),
                "t_m": _mirror_vec3(sample.get("t_m")),
            })
        res = dict(doc["result"])
        res["tcp_m"] = _mirror_vec3(res.get("tcp_m"))
        res["tcp_mm"] = [float(v) * 1000.0 for v in res["tcp_m"]]
        if res.get("fixed_point_m") is not None:
            res["fixed_point_m"] = _mirror_vec3(res.get("fixed_point_m"))
            res["fixed_mm"] = [float(v) * 1000.0 for v in res["fixed_point_m"]]
        src_side = "左" if src_arm == "left" else "右"
        dst_side = "左" if dst_arm == "left" else "右"
        note_src = str(doc.get("note") or "").strip()
        note = f"由{src_side}臂「{src_name}」关于 y=0 镜像。"
        if note_src:
            note = f"{note} 原备注：{note_src}"
        out_doc = {
            "name": dst_name,
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "arm": dst_arm,
            "tip_link": tip,
            "base_link": "torso_link",
            "note": note[:80],
            "n": len(samples_out),
            "samples": samples_out,
            "result": res,
            "mirrored_from": {"name": src_name, "arm": src_arm},
        }
        RESULTS_DIR.mkdir(parents=True, exist_ok=True)
        dst_path.write_text(json.dumps(out_doc, indent=2, ensure_ascii=False), encoding="utf-8")
        status = self.status()
        status["mirrored_name"] = dst_name
        status["mirrored_arm"] = dst_arm
        status["message"] = f"已镜像为「{dst_name}」（{dst_side}臂）"
        return status

    def _apply_doc(self, doc: Dict[str, Any]) -> None:
        self.arm = str(doc.get("arm") or "")
        self.tip_link = str(doc.get("tip_link") or "")
        self.samples = list(doc.get("samples") or [])
        self.result = doc.get("result")
        self.active_name = str(doc.get("name") or "")
        self.note = str(doc.get("note") or "")

    def record(self, arm: str, tip_link: str, q, R, t) -> Dict[str, Any]:
        q = np.asarray(q, dtype=float).reshape(-1)
        if self.arm and arm != self.arm:
            side = "左" if self.arm == "left" else "右"
            raise ValueError(f"这组是{side}臂的，先清空再记另一只")
        for sample in self.samples:
            if float(np.max(np.abs(np.asarray(sample["q_rad"]) - q))) < 0.02:
                raise ValueError("这个姿态和已记的太近，先把腕转开")
        self.arm = arm
        self.tip_link = tip_link
        self.result = None
        self.active_name = ""
        self.samples.append({
            "q_rad": q.tolist(),
            "R": np.asarray(R, dtype=float).reshape(3, 3).tolist(),
            "t_m": np.asarray(t, dtype=float).reshape(3).tolist(),
        })
        return self.status()

    def undo(self) -> Dict[str, Any]:
        if self.samples:
            self.samples.pop()
        self.result = None
        self.active_name = ""
        if not self.samples:
            self.arm = ""
            self.tip_link = ""
        return self.status()

    def clear(self) -> Dict[str, Any]:
        # 只清空当前标定会话，不改左右各自已生效的命名结果
        self.samples.clear()
        self.arm = ""
        self.tip_link = ""
        self.result = None
        self.active_name = ""
        self.note = ""
        return self.status()

    def solve(self) -> Dict[str, Any]:
        self.result = solve_pivot(self.samples)
        self.active_name = ""
        return self.status()
