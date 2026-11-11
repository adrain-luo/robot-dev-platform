"""末端负载辨识：姿态集静止采样 → 最小二乘 θ=[m, m·cx, m·cy, m·cz]，α 固定为 1。"""

from __future__ import annotations

import json
import math
import re
import threading
import time
from pathlib import Path
from typing import Callable, Dict, List, Optional

import numpy as np

from robot_platform.control.arm_gravity import ArmGravity

from robot_platform.core import instance as instances


def measure_dir() -> Path:
    return instances.path("measure")


def payload_dir() -> Path:
    return instances.path("payloads")


def theta_to_payload(theta) -> tuple:
    th = np.asarray(theta, float).reshape(4)
    m = float(th[0])
    if abs(m) < 1e-9:
        return 0.0, [0.0, 0.0, 0.0]
    return m, (th[1:4] / m).tolist()


def payload_to_theta(mass: float, com) -> List[float]:
    c = np.asarray(com, float).reshape(3)
    return [float(mass), float(mass * c[0]), float(mass * c[1]), float(mass * c[2])]


_NAME_RE = re.compile(r"^[\w\-\u4e00-\u9fff.]{1,48}$")


def payload_path(arm: str) -> Path:
    """最后一次应用到该臂的快照（拆换后启动仍可自动加载）。"""
    last = payload_dir() / f"_last_{arm}.json"
    old = payload_dir() / f"{arm}.json"
    return last if last.exists() else old


def save_payload(arm: str, d: Dict) -> Path:
    payload_dir().mkdir(parents=True, exist_ok=True)
    p = payload_dir() / f"_last_{arm}.json"
    p.write_text(json.dumps(d, indent=2, ensure_ascii=False), encoding="utf-8")
    return p


def load_payload(arm: str) -> Optional[Dict]:
    p = payload_path(arm)
    if not p.exists():
        return None
    return json.loads(p.read_text(encoding="utf-8"))


def clear_payload(arm: str) -> None:
    """清掉该臂「最后应用」快照（不影响命名补偿文件）。"""
    arm = "left" if arm == "left" else "right"
    p = payload_dir() / f"_last_{arm}.json"
    if p.is_file():
        p.unlink()


def applied_name_for_arm(arm: str) -> str:
    d = load_payload(arm) or {}
    return str(d.get("name") or "")


def _check_profile_name(name: str) -> str:
    name = (name or "").strip()
    if name.startswith("_last_"):
        raise ValueError("名字不能以 _last_ 开头")
    if not _NAME_RE.match(name):
        raise ValueError("补偿文件名只能用中英文、数字、_ - .，1～48 个字符")
    return name


def profile_path(name: str) -> Path:
    return payload_dir() / f"{_check_profile_name(name)}.json"


def list_profiles(arm: Optional[str] = None) -> List[Dict]:
    payload_dir().mkdir(parents=True, exist_ok=True)
    out = []
    for p in sorted(payload_dir().glob("*.json"), key=lambda x: x.stat().st_mtime, reverse=True):
        if p.stem.startswith("_last_"):
            continue
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
        except Exception:
            continue
        d["name"] = d.get("name") or p.stem
        if arm and d.get("arm") not in (None, arm):
            continue
        out.append({
            "name": d["name"],
            "arm": d.get("arm"),
            "mass_kg": d.get("mass_kg"),
            "com_m": d.get("com_m"),
            "session": d.get("session"),
            "poseset": d.get("poseset"),
            "note": d.get("note", ""),
            "updated": d.get("updated") or d.get("applied_at"),
        })
    return out


def list_sessions(arm: Optional[str] = None, poseset: Optional[str] = None) -> List[Dict]:
    if not measure_dir().exists():
        return []
    out = []
    for d in sorted(measure_dir().iterdir(), reverse=True):
        meta_p, samp_p = d / "meta.json", d / "samples.json"
        if not meta_p.exists() or not samp_p.exists():
            continue
        try:
            meta = json.loads(meta_p.read_text(encoding="utf-8"))
        except Exception:
            continue
        if arm and meta.get("arm") != arm:
            continue
        if poseset and meta.get("poseset") != poseset:
            continue
        n = 0
        try:
            n = len(json.loads(samp_p.read_text(encoding="utf-8")))
        except Exception:
            pass
        out.append({
            "name": d.name,
            "arm": meta.get("arm"),
            "poseset": meta.get("poseset"),
            "n": n,
            "created": meta.get("created"),
            "bidirectional": bool(meta.get("bidirectional") or (meta.get("opts") or {}).get("bidirectional")),
        })
    return out


def load_profile(name: str) -> Dict:
    p = profile_path(name)
    if not p.exists():
        raise FileNotFoundError(f"没有补偿文件 {name}")
    d = json.loads(p.read_text(encoding="utf-8"))
    d["name"] = p.stem
    if not d.get("theta") and d.get("mass_kg") is not None:
        d["theta"] = payload_to_theta(d["mass_kg"], d.get("com_m") or [0, 0, 0])
    return d


def save_profile(name: str, d: Dict, note: str = "") -> Dict:
    name = _check_profile_name(name)
    payload_dir().mkdir(parents=True, exist_ok=True)
    out = dict(d)
    out["name"] = name
    if note:
        out["note"] = note
    out["updated"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    if not out.get("theta") and out.get("mass_kg") is not None:
        out["theta"] = payload_to_theta(out["mass_kg"], out.get("com_m") or [0, 0, 0])
    profile_path(name).write_text(json.dumps(out, indent=2, ensure_ascii=False), encoding="utf-8")
    return out


def delete_profile(name: str) -> None:
    p = profile_path(name)
    if not p.exists():
        raise FileNotFoundError(f"没有补偿文件 {name}")
    p.unlink()


def tau_from_pd(s: Dict) -> Optional[List[float]]:
    if s.get("tau_ff_sent") is None or s.get("q_cmd") is None:
        return None
    kp, kd = np.asarray(s["kp"], float), np.asarray(s["kd"], float)
    dq = np.asarray(s["dq"], float) if s.get("dq") is not None else np.zeros_like(kp)
    tau = kp * (np.asarray(s["q_cmd"]) - np.asarray(s["q_meas"])) - kd * dq + np.asarray(s["tau_ff_sent"])
    return [float(v) for v in tau]


def average_samples(loop, n: int, stop: threading.Event) -> Dict:
    acc: Optional[Dict[str, np.ndarray]] = None
    keys = ("q_meas", "dq", "q_cmd", "tau_ff_sent", "tau_est")
    got = 0
    for _ in range(max(n, 1) * 4):
        if stop.is_set():
            raise RuntimeError("已停止")
        s = loop.arm_sample()
        if any(v is None for v in s["tau_est"]):
            time.sleep(0.02)
            continue
        if acc is None:
            acc = {k: np.asarray(s[k], float) for k in keys}
        else:
            for k in keys:
                acc[k] += np.asarray(s[k], float)
        got += 1
        if got >= n:
            break
        time.sleep(0.02)
    if acc is None or got == 0:
        raise RuntimeError("采不到 tau_est")
    out = {k: (acc[k] / got).tolist() for k in keys}
    out["kp"], out["kd"] = s["kp"], s["kd"]
    if s.get("g_torso") is not None:
        out["g_torso"] = s["g_torso"]  # 采样时前馈用的重力向量（平滑过，静止时一拍就够）
    out["n_avg"] = got
    return out


# 保持态编码器噪声约 0.02 rad/s；0.01 真机常等不到。略宽于 H2 的 0.03。
DEFAULT_STILL = {
    "still_dq_rad_s": 0.05,
    "still_drift_rad": 0.01,
    "still_window_s": 0.5,
    "settle_timeout_s": 8.0,
}


def wait_cmd_caught(loop, stop: threading.Event, q_eps: float = 0.02, timeout_s: float = 90.0) -> None:
    """限速层 cmd 追上 desired。不看实测是否到目标。"""
    t0 = time.monotonic()
    while True:
        if stop.is_set():
            raise RuntimeError("已停止")
        st = loop.status()
        cmd = np.asarray(st["cmd_rad"], float)
        des = np.asarray(st["desired_rad"], float)
        if float(np.max(np.abs(des - cmd))) < q_eps:
            return
        if time.monotonic() - t0 > timeout_s:
            return
        time.sleep(0.05)


def wait_still(loop, stop: threading.Event, params: Optional[Dict] = None) -> Dict:
    """窗口内 max|dq| < 速度阈 且 各关节位置峰峰值 < 漂移阈。超时仍按当前角采样。"""
    p = {**DEFAULT_STILL, **(params or {})}
    dq_th = float(p["still_dq_rad_s"])
    drift_th = float(p["still_drift_rad"])
    win = float(p["still_window_s"])
    timeout = float(p["settle_timeout_s"])
    hist = []
    t0 = time.monotonic()
    drift, max_dq = float("nan"), float("nan")
    while True:
        if stop.is_set():
            raise RuntimeError("已停止")
        now = time.monotonic()
        try:
            samp = loop.arm_sample(wait=False)
            q = np.asarray(samp["q_meas"], float)
            dq = np.asarray(samp["dq"], float)
            hist.append((now, q, dq))
        except RuntimeError:
            pass
        hist = [h for h in hist if now - h[0] <= win]
        if hist:
            max_dq = float(max(float(np.max(np.abs(h[2]))) for h in hist))
            if len(hist) >= 2:
                qs = np.stack([h[1] for h in hist])
                drift = float(np.max(np.max(qs, axis=0) - np.min(qs, axis=0)))
            else:
                drift = 0.0
            span = now - hist[0][0]
            if span >= win * 0.95 and len(hist) >= 3 and drift < drift_th and max_dq < dq_th:
                return {"settle_s": now - t0, "timed_out": False, "drift_rad": drift, "max_dq_rad_s": max_dq}
        if now - t0 > timeout:
            return {"settle_s": now - t0, "timed_out": True, "drift_rad": drift, "max_dq_rad_s": max_dq}
        time.sleep(0.02)


def wait_settle(loop, q, stop: threading.Event, still: Optional[Dict] = None) -> Dict:
    loop.set_target(q)
    t0 = time.monotonic()
    wait_cmd_caught(loop, stop)
    r = wait_still(loop, stop, still)
    r["settle_s"] = time.monotonic() - t0
    return r


def solve_payload(samples: List[Dict], arm: str, urdf_path: Path, base_link: str,
                  source: str = "pd") -> Dict:
    grav = ArmGravity(arm, urdf_path, base_link)
    rows_A, rows_b, rows_arm, tags = [], [], [], []
    for s in samples:
        tau_true = s.get("tau_pd") if source == "pd" else s.get("tau_est")
        if tau_true is None:
            continue
        if any(v is None for v in tau_true):
            continue
        q = np.asarray(s["q_meas"], float)
        g = s.get("g_torso")  # 老样本没有：当时就是按躯干竖直算的
        Y = grav.payload_regressor(q, g)
        ta = grav.tau_arm(q, g)
        rows_A.append(Y)
        rows_b.append(np.asarray(tau_true, float))
        rows_arm.append(ta)
        tags.append(f"{s.get('name', '?')}{s.get('approach', '')}")
    used = len(rows_A)
    if used < 4:
        raise ValueError(f"可用样本只有 {used} 个（需要 ≥4；建议双向、点散开）")
    A = np.vstack(rows_A)
    b = np.concatenate(rows_b)
    ta = np.concatenate(rows_arm)
    b_fit = b - ta
    joint_idx = np.tile(np.arange(7), used)

    def wlsq(w):
        x, *_ = np.linalg.lstsq(A * w[:, None], b_fit * w, rcond=None)
        return x

    w = np.ones_like(b_fit)
    x = wlsq(w)
    res = b_fit - A @ x
    sig = np.array([max(float(np.std(res[joint_idx == j])), 0.02) if np.any(joint_idx == j) else 1.0 for j in range(7)])
    w = 1.0 / sig[joint_idx]
    theta = wlsq(w)
    res_before = b - ta
    res_after = b_fit - A @ theta
    mass, com = theta_to_payload(theta)
    scale = np.linalg.norm(A, axis=0) + 1e-9
    kp = np.asarray(samples[0]["kp"], float)
    sag_before = np.abs(res_before) / kp[joint_idx]
    sag_after = np.abs(res_after) / kp[joint_idx]

    def per_joint(r):
        return [float(np.sqrt(np.mean(r[joint_idx == j] ** 2))) for j in range(7)]

    return {
        "arm": arm,
        "source": source,
        "fit_alpha": False,
        "alpha": 1.0,
        "n_samples": used,
        "mass_kg": mass,
        "com_m": com,
        "theta": [float(v) for v in theta],
        "regressor_cond": float(np.linalg.cond(A / scale)),
        "residual_rms_before_nm": float(np.sqrt(np.mean(res_before ** 2))),
        "residual_rms_after_nm": float(np.sqrt(np.mean(res_after ** 2))),
        "residual_per_joint_before_nm": per_joint(res_before),
        "residual_per_joint_after_nm": per_joint(res_after),
        "sag_est_before_deg": float(np.mean(sag_before) * 180 / math.pi),
        "sag_est_after_deg": float(np.mean(sag_after) * 180 / math.pi),
        "solved_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "sample_tags": tags,
    }


class MeasureTask:
    def __init__(self, loop, poses_cfg: Dict, urdf_path: Path, base_link: str,
                 log: Callable[[str], None], on_done: Callable[[Dict], None],
                 still: Optional[Dict] = None, bidirectional: bool = True,
                 approach_delta: float = 0.06, dwell_s: float = 0.5, avg_frames: int = 20):
        self.loop = loop
        self.poses_cfg = poses_cfg
        self.urdf_path = urdf_path
        self.base_link = base_link
        self.log = log
        self.on_done = on_done
        self.still = {**DEFAULT_STILL, **(still or {})}
        self.bidirectional = bidirectional
        self.approach_delta = approach_delta
        self.dwell_s = dwell_s
        self.avg_frames = avg_frames
        self.stop = threading.Event()
        self.samples: List[Dict] = []
        self.session_dir = measure_dir() / f"{time.strftime('%Y%m%d_%H%M%S')}_{poses_cfg['arm']}"
        self.phase = "待命"
        self.progress = [0, 0]

    def _save(self) -> None:
        self.session_dir.mkdir(parents=True, exist_ok=True)
        (self.session_dir / "samples.json").write_text(
            json.dumps(self.samples, indent=1, ensure_ascii=False), encoding="utf-8")
        (self.session_dir / "poseset.json").write_text(
            json.dumps(self.poses_cfg, indent=2, ensure_ascii=False), encoding="utf-8")
        meta = {
            "arm": self.poses_cfg["arm"],
            "poseset": self.poses_cfg.get("name"),
            "poseset_file": "poseset.json",
            "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "bidirectional": self.bidirectional,
            "still": self.still,
        }
        (self.session_dir / "meta.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")

    def _one(self, pose: Dict, approach: str) -> Dict:
        q_t = np.asarray(pose["q"], float)
        r = wait_settle(self.loop, q_t, self.stop, self.still)
        time.sleep(self.dwell_s)
        avg = average_samples(self.loop, self.avg_frames, self.stop)
        s = {
            "name": pose.get("name"),
            "approach": approach,
            "q_target": q_t.tolist(),
            **avg,
            "settle_s": r["settle_s"],
            "timed_out": r["timed_out"],
            "drift_rad": r.get("drift_rad"),
            "max_dq_rad_s": r.get("max_dq_rad_s"),
        }
        s["tau_pd"] = tau_from_pd(s)
        return s

    def run(self) -> None:
        try:
            poses = self.poses_cfg.get("poses") or []
            if len(poses) < 2:
                raise ValueError("姿态集至少 2 个点")
            with self.loop.claim():
                self._run(poses)
        except Exception as e:
            self.phase = "失败"
            self.on_done({"ok": False, "error": f"{type(e).__name__}: {e}", "session": self.session_dir.name,
                          "n_samples": len(self.samples)})

    def _run(self, poses) -> None:
        st = self.loop.status()
        if not st.get("engaged") or st.get("float"):
            raise RuntimeError("先接管并退出拖拽再测量")
        lo, hi = self.loop.limits[:, 0], self.loop.limits[:, 1]
        n_app = 2 if self.bidirectional else 1
        total = len(poses) * n_app
        self.progress = [0, total]
        self.log(
            f"测量 {len(poses)} 点 × {n_app}，会话 {self.session_dir.name}，限速 {self.loop.max_speed:.3f} rad/s，"
            f"静止 |dq|<{self.still['still_dq_rad_s']} 漂移<{self.still['still_drift_rad']} "
            f"窗 {self.still['still_window_s']}s 超时 {self.still['settle_timeout_s']}s"
        )
        for pose in poses:
            q_t = np.asarray(pose["q"], float)
            approaches = [("+", 1.0), ("-", -1.0)] if self.bidirectional else [("", 0.0)]
            for tag, sgn in approaches:
                if self.stop.is_set():
                    raise RuntimeError("已停止")
                self.phase = f"{pose.get('name')}{tag}"
                if sgn != 0.0:
                    pre = np.clip(q_t + sgn * self.approach_delta, lo, hi)
                    self.log(f"{self.phase} 预位")
                    wait_settle(self.loop, pre, self.stop, {**self.still, "still_window_s": min(0.2, self.still["still_window_s"])})
                self.log(f"{self.phase} 到位采样")
                s = self._one(pose, tag)
                self.samples.append(s)
                self._save()
                self.progress = [len(self.samples), total]
                extra = ""
                if s["timed_out"]:
                    extra = f"（超时仍采样 dq={s.get('max_dq_rad_s')} 漂移={s.get('drift_rad')}）"
                self.log(f"{self.phase} 完成{extra}")
        home = self.poses_cfg.get("home_q")
        if home:
            self.phase = "回 home"
            wait_settle(self.loop, home, self.stop, self.still)
        self.phase = "完成"
        self.on_done({"ok": True, "session": self.session_dir.name, "n_samples": len(self.samples)})
