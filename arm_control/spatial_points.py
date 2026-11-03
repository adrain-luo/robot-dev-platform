"""躯干坐标系里的命名空间点，以及让 TCP 碰到该点的关节解。"""

from __future__ import annotations

import json
import math
import re
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

import instance as instances


def points_dir() -> Path:
    return instances.path("spatial_points")


def _active_path() -> Path:
    return points_dir() / "_active.json"
_NAME_RE = re.compile(r"^[\w\-\u4e00-\u9fff.]{1,48}$")


def _check_name(name: str) -> str:
    name = (name or "").strip()
    if not _NAME_RE.match(name):
        raise ValueError("名字只能用中英文、数字、_ - .，1～48 个字符")
    return name


def _path(name: str) -> Path:
    return points_dir() / f"{_check_name(name)}.json"


def list_saved() -> List[Dict[str, Any]]:
    if not points_dir().is_dir():
        return []
    out = []
    for path in sorted(points_dir().glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True):
        if path.name.startswith("_"):
            continue
        try:
            doc = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        out.append({
            "name": doc.get("name") or path.stem,
            "source": doc.get("source") or "",
            "point_mm": doc.get("point_mm") or [],
            "note": doc.get("note") or "",
            "saved_at": doc.get("saved_at") or "",
            # 旧文件可能还有 arm/tcp_name，仅作显示兼容，碰点不再使用
            "arm": doc.get("arm") or "",
            "tcp_name": doc.get("tcp_name") or "",
        })
    return out


def _read_active() -> str:
    if not _active_path().is_file():
        return ""
    try:
        return str(json.loads(_active_path().read_text(encoding="utf-8")).get("name") or "")
    except (OSError, json.JSONDecodeError):
        return ""


def _write_active(name: str) -> None:
    points_dir().mkdir(parents=True, exist_ok=True)
    if not name:
        if _active_path().is_file():
            _active_path().unlink()
        return
    _active_path().write_text(json.dumps({"name": name}, ensure_ascii=False, indent=2), encoding="utf-8")


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


def axis_angle_R(axis, deg: float) -> np.ndarray:
    """躯干系单位轴 × 角度(°) → 旋转矩阵。"""
    a = np.asarray(axis, dtype=float).reshape(3)
    n = float(np.linalg.norm(a))
    if n < 1e-12:
        return np.eye(3)
    k = a / n
    th = float(np.deg2rad(deg))
    K = np.array([
        [0.0, -k[2], k[1]],
        [k[2], 0.0, -k[0]],
        [-k[1], k[0], 0.0],
    ], dtype=float)
    return np.eye(3) + math.sin(th) * K + (1.0 - math.cos(th)) * (K @ K)


def _rotvec_from_R(R_err: np.ndarray) -> np.ndarray:
    """SO(3) 相对旋转 → 旋转向量（轴角）。"""
    cos_th = float(np.clip((np.trace(R_err) - 1.0) * 0.5, -1.0, 1.0))
    th = float(np.arccos(cos_th))
    if th < 1e-8:
        return np.zeros(3)
    w = np.array([
        R_err[2, 1] - R_err[1, 2],
        R_err[0, 2] - R_err[2, 0],
        R_err[1, 0] - R_err[0, 1],
    ], dtype=float)
    if th > np.pi - 1e-4:
        # 近 180°：从对角取轴
        d = np.clip(np.diag(R_err) + 1.0, 0.0, None)
        axis = np.sqrt(d)
        if axis[0] >= axis[1] and axis[0] >= axis[2]:
            axis = np.array([axis[0], R_err[0, 1] / max(axis[0], 1e-9), R_err[0, 2] / max(axis[0], 1e-9)])
        elif axis[1] >= axis[2]:
            axis = np.array([R_err[0, 1] / max(axis[1], 1e-9), axis[1], R_err[1, 2] / max(axis[1], 1e-9)])
        else:
            axis = np.array([R_err[0, 2] / max(axis[2], 1e-9), R_err[1, 2] / max(axis[2], 1e-9), axis[2]])
        n = float(np.linalg.norm(axis))
        return (axis / n) * th if n > 1e-12 else np.zeros(3)
    return w * (th / (2.0 * np.sin(th)))


def solve_tcp_pose_ik(
    model,
    q0,
    tcp_m,
    target_m,
    R_target,
    *,
    w_ori: float = 0.5,
    iters: int = 120,
) -> Tuple[np.ndarray, float, float]:
    """TCP 到 target，且腕部姿态对齐 R_target（躯干系）。返回 (q, 位置误差 mm, 姿态误差 deg)。"""
    q_seed = np.asarray(q0, dtype=float).reshape(-1)
    q = q_seed.copy()
    tcp = np.asarray(tcp_m, dtype=float).reshape(3)
    target = np.asarray(target_m, dtype=float).reshape(3)
    Rd = np.asarray(R_target, dtype=float).reshape(3, 3)
    limits = model.joint_limits()
    eye6 = np.eye(6)
    for _ in range(int(iters)):
        pos, R, jac = _tcp_pose_jac(model, q, tcp, w_ori=w_ori)
        err_p = target - pos
        err_r = _rotvec_from_R(Rd @ R.T)
        err = np.concatenate([err_p, w_ori * err_r])
        if float(np.linalg.norm(err_p)) < 2e-4 and float(np.linalg.norm(err_r)) < np.deg2rad(0.3):
            break
        jjt = jac @ jac.T + 1e-3 * eye6
        dq = jac.T @ np.linalg.solve(jjt, err)
        null = np.eye(model.n) - jac.T @ np.linalg.solve(jjt, jac)
        dq = dq + null @ (0.08 * (q_seed - q))
        step = float(np.linalg.norm(dq))
        if step > 0.15:
            dq *= 0.15 / step
        q = np.clip(q + dq, limits[:, 0], limits[:, 1])
    pos, R, _ = _tcp_pose_jac(model, q, tcp, w_ori=w_ori)
    miss_mm = float(np.linalg.norm(pos - target) * 1000.0)
    ori_miss_deg = float(np.degrees(np.linalg.norm(_rotvec_from_R(Rd @ R.T))))
    return q, miss_mm, ori_miss_deg


def solve_tcp_pose_ik_robust(
    model,
    q0,
    tcp_m,
    target_m,
    R_target,
    *,
    extra_seeds=None,
    n_random: int = 16,
    n_ok_stop: int = 5,
    pos_ok_mm: float = 3.0,
    ori_ok_deg: float = 3.0,
) -> Tuple[np.ndarray, float, float]:
    """多初值求位置+全姿态解，避免从当前角一枪局部 IK 陷住。

    策略：按需依次试种子（当前角 → 纯位置解 → 中位 → 额外种子 → 随机 /
    随机上的位置解）；每个种子几档姿态权重。收集进门解，够 ``n_ok_stop``
    个就停，在合格解里取距 ``q0`` 最近的一组；若始终不够门，退回残差最好的。
    """
    q0 = np.asarray(q0, dtype=float).reshape(-1)
    limits = model.joint_limits()
    rng = np.random.default_rng(0)
    n_rand = max(0, int(n_random))
    n_stop = max(1, int(n_ok_stop))

    def _seed_iter():
        yield q0.copy()
        qp, mp = solve_tcp_ik(model, q0, tcp_m, target_m)
        if mp < 25.0:
            yield qp
        yield 0.5 * (limits[:, 0] + limits[:, 1])
        if extra_seeds:
            for s in extra_seeds:
                sa = np.asarray(s, dtype=float).reshape(-1)
                if sa.size == model.n and np.all(np.isfinite(sa)):
                    yield np.clip(sa, limits[:, 0], limits[:, 1])
        for _ in range(n_rand):
            yield rng.uniform(limits[:, 0], limits[:, 1])
        for _ in range(n_rand // 2):
            qr = rng.uniform(limits[:, 0], limits[:, 1])
            qp2, mp2 = solve_tcp_ik(model, qr, tcp_m, target_m)
            if mp2 < 15.0:
                yield qp2

    ok_list: List[Tuple[np.ndarray, float, float, float]] = []
    best_any: Optional[Tuple[np.ndarray, float, float]] = None

    for seed in _seed_iter():
        for w in (0.35, 0.7, 1.2):
            q, m, o = solve_tcp_pose_ik(model, seed, tcp_m, target_m, R_target, w_ori=w, iters=100)
            if best_any is None or (m + 2.0 * o) < (best_any[1] + 2.0 * best_any[2]):
                best_any = (q, m, o)
            if m <= pos_ok_mm and o <= ori_ok_deg:
                ok_list.append((q, m, o, float(np.linalg.norm(q - q0))))
                if len(ok_list) >= n_stop:
                    ok_list.sort(key=lambda x: x[3])
                    qb, mb, ob, _ = ok_list[0]
                    return qb, mb, ob
    if ok_list:
        ok_list.sort(key=lambda x: x[3])
        qb, mb, ob, _ = ok_list[0]
        return qb, mb, ob
    assert best_any is not None
    return best_any


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


def _tcp_pose_jac(model, q, tcp, w_ori: float = 0.5):
    t, R = model.fk(q)
    pos = t + R @ tcp
    jac = np.zeros((6, model.n))
    eps = 1e-5
    for i in range(model.n):
        dq = np.zeros(model.n)
        dq[i] = eps
        t2, R2 = model.fk(q + dq)
        pos2 = t2 + R2 @ tcp
        jac[0:3, i] = (pos2 - pos) / eps
        # d(rotvec)/dq ≈ rotvec(R2 R^T) / eps
        jac[3:6, i] = w_ori * _rotvec_from_R(R2 @ R.T) / eps
    return pos, R, jac


class SpatialPoints:
    def __init__(self) -> None:
        self.active_name = ""
        self.doc: Optional[Dict[str, Any]] = None

    def status(self) -> Dict[str, Any]:
        doc = self.doc or {}
        return {
            "ok": True,
            "active_name": self.active_name,
            "source": doc.get("source") or "",
            "point_m": doc.get("point_m"),
            "point_mm": doc.get("point_mm"),
            "note": doc.get("note") or "",
            "saved": list_saved(),
        }

    def restore_active(self) -> None:
        name = _read_active()
        if name and _path(name).is_file():
            self._apply(json.loads(_path(name).read_text(encoding="utf-8")))

    def save(
        self,
        name: str,
        point_m,
        note: str = "",
        *,
        source: str = "",
    ) -> Dict[str, Any]:
        """只存躯干系位置。碰点时再用当前控制臂 + 当前 TCP。"""
        name = _check_name(name)
        point = np.asarray(point_m, dtype=float).reshape(3)
        if not np.all(np.isfinite(point)):
            raise ValueError("空间点不是有限数")
        points_dir().mkdir(parents=True, exist_ok=True)
        doc = {
            "name": name,
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "source": (source or "").strip(),
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
