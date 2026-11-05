"""到位对比：同一姿态集，基线 / 负载补偿 / 补偿+摩擦带修正。"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import Counter
from pathlib import Path
from typing import Callable, Dict, List, Optional

import numpy as np

from robot_platform.calib.payload import friction
from robot_platform.calib.payload import identify as iden
from robot_platform.control.arm_gravity import ArmGravity

from robot_platform.core import instance as instances


def validate_dir() -> Path:
    return instances.path("validate")


def list_results(arm: Optional[str] = None, poseset: Optional[str] = None) -> List[Dict]:
    if not validate_dir().is_dir():
        return []
    out = []
    for p in sorted(validate_dir().glob("*.json"), key=lambda x: x.stat().st_mtime, reverse=True):
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
        except Exception:
            continue
        if arm and d.get("arm") != arm:
            continue
        if poseset and d.get("poseset") != poseset:
            continue
        out.append({
            "name": d.get("session") or p.stem,
            "arm": d.get("arm"),
            "poseset": d.get("poseset"),
            "n": len(d.get("rows") or []),
            "ok": bool(d.get("ok")),
            "finished": d.get("finished"),
            "file": str(p),
        })
    return out


def load_result(name: str) -> Dict:
    stem = Path(str(name)).name
    if stem.endswith(".json"):
        stem = stem[:-5]
    if not stem or "/" in stem or stem.startswith("."):
        raise ValueError("非法对比记录名")
    p = (validate_dir() / f"{stem}.json").resolve()
    if p.parent != validate_dir().resolve() or not p.is_file():
        raise FileNotFoundError(f"没有对比记录 {stem}")
    return json.loads(p.read_text(encoding="utf-8"))


JOINT_SHORT = ("sh_pitch", "sh_roll", "sh_yaw", "elbow", "wr_roll", "wr_pitch", "wr_yaw")

CORRECTION_DEADBAND = {
    "err_max_deg": 0.5,
    "ee_mm": 2.0,
    "use_band": True,
    "band_scale": 1.0,
    "band_min_deg": 0.05,
    "soft_ratio": 2.0,
    "soft_gain": 0.5,
    "step_max_deg": 2.0,
    "revert_to_best": True,
    "revert_margin_deg": 0.05,
}


def max_joint_fields(err_rad) -> Dict:
    e = np.asarray(err_rad, float).reshape(-1)
    i = int(np.argmax(np.abs(e))) if e.size else 0
    name = JOINT_SHORT[i] if 0 <= i < len(JOINT_SHORT) else f"J{i + 1}"
    signed = float(e[i] * 180.0 / math.pi) if e.size else 0.0
    return {
        "err_max_deg": float(np.max(np.abs(e)) * 180 / math.pi) if e.size else 0.0,
        "err_max_joint": i,
        "err_max_joint_name": name,
        "err_max_joint_signed_deg": round(signed, 3),
    }


def fmt_max_joint(d: Dict) -> str:
    return f"{d.get('err_max_deg', 0):.2f}° @{d.get('err_max_joint_name', '?')}"


def ee_error(grav: ArmGravity, q_final, q_target) -> Dict:
    Tf, Tt = grav.tip_T(q_final), grav.tip_T(q_target)
    dp = Tf[:3, 3] - Tt[:3, 3]
    dR = Tt[:3, :3].T @ Tf[:3, :3]
    ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1) / 2))))
    return {"pos_mm": float(np.linalg.norm(dp) * 1000), "rot_deg": ang}


def _metrics(grav: ArmGravity, q, target) -> Dict:
    e = np.asarray(q, float) - np.asarray(target, float)
    out = {"err_rms_deg": float(np.sqrt(np.mean(e ** 2)) * 180 / math.pi),
           "ee_mm": ee_error(grav, q, target)["pos_mm"]}
    out.update(max_joint_fields(e))
    return out


def correction_thresholds(db: Dict, band_deg=None, n: int = 7) -> np.ndarray:
    if db.get("use_band") and band_deg is not None:
        b = np.asarray(band_deg, float) * float(db.get("band_scale", 1.0))
        return np.maximum(b, float(db.get("band_min_deg", 0.05)))
    return np.full(n, float(db["err_max_deg"]))


def snapshot_payload(loop) -> Optional[Dict]:
    th = loop.payload_theta()
    st = loop.status()
    if not th:
        return None
    m, c = iden.theta_to_payload(th)
    return {"theta": th, "mass_kg": m, "com_m": c, "name": st.get("payload_name")}


def apply_payload(loop, p: Optional[Dict]) -> None:
    if not p or not p.get("theta"):
        loop.set_payload_theta(None)
        return
    loop.set_payload_theta(p["theta"], p.get("name"))


def goto_with_correction(loop, grav, target, still, iters, stop, deadband, band_deg, on_phase=None, on_log=None) -> Dict:
    db = {**CORRECTION_DEADBAND, **(deadband or {})}
    target = np.asarray(target, float)
    n = target.size
    thr = correction_thresholds(db, band_deg, n)
    t0 = time.monotonic()
    r0 = iden.wait_settle(loop, target, stop, still)
    samp = loop.arm_sample(wait=False)
    q_final = np.asarray(samp["q_meas"], float)
    m = _metrics(grav, q_final, target)
    steps = [{"iter": 0, **m, "t_s": r0["settle_s"], "timed_out": r0["timed_out"]}]
    lo, hi = loop.limits[:, 0], loop.limits[:, 1]
    cmd = np.asarray(loop.status()["cmd_rad"], float)
    best = {"iter": 0, "cmd": cmd.copy(), **m}
    applied, skipped_at, stop_reason, last_step = 0, None, None, False
    still_short = {**still, "still_window_s": min(0.3, float(still.get("still_window_s", 0.5)))}
    for k in range(1, int(iters) + 1):
        e_deg = (q_final - target) * 180 / math.pi
        ratio = np.abs(e_deg) / thr
        if m["err_max_deg"] < db["err_max_deg"] and m["ee_mm"] < db["ee_mm"]:
            stop_reason = f"误差 {fmt_max_joint(m)} / {m['ee_mm']:.1f} mm 已达标"
        elif not np.any(ratio > 1.0):
            worst = int(np.argmax(ratio))
            stop_reason = (f"各关节都在摩擦带内（{JOINT_SHORT[worst]} "
                           f"{abs(e_deg[worst]):.2f}° vs {thr[worst]:.2f}°）")
        if stop_reason:
            skipped_at = k
            steps.append({"iter": k, "skipped": True, "reason": stop_reason, **m, "t_s": time.monotonic() - t0})
            if on_log:
                on_log(f"{stop_reason}，跳过第 {k}/{iters} 次修正")
            break
        if on_phase:
            on_phase(f"修正 {k}/{iters}")
        full = ratio >= float(db.get("soft_ratio", 2.0))
        g = np.where(full, 1.0, np.where(ratio > 1.0, float(db.get("soft_gain", 0.5)), 0.0))
        step_deg = np.clip(g * e_deg, -float(db["step_max_deg"]), float(db["step_max_deg"]))
        cmd = np.asarray(loop.status()["cmd_rad"], float)
        new_cmd = np.clip(cmd - step_deg * math.pi / 180, lo, hi)
        loop.set_target(new_cmd.tolist())
        last_step = not np.any(full)
        time.sleep(0.1)
        iden.wait_cmd_caught(loop, stop)
        still_r = iden.wait_still(loop, stop, still_short)
        samp = loop.arm_sample(wait=False)
        q_final = np.asarray(samp["q_meas"], float)
        m = _metrics(grav, q_final, target)
        applied += 1
        steps.append({"iter": k, **m, "t_s": time.monotonic() - t0, "timed_out": still_r["timed_out"],
                      "n_active": int(np.count_nonzero(g)), "step_deg": step_deg.round(3).tolist()})
        if m["err_max_deg"] < best["err_max_deg"]:
            best = {"iter": k, "cmd": np.asarray(new_cmd, float).copy(), **m}
        if last_step:
            stop_reason = "误差已接近摩擦带，走完这一小步收手"
            if on_log:
                on_log(stop_reason)
            break
    reverted = None
    if (db.get("revert_to_best") and applied and best["iter"] != steps[-1]["iter"]
            and m["err_max_deg"] > best["err_max_deg"] + float(db.get("revert_margin_deg", 0.05))):
        if on_log:
            on_log(f"回退到第 {best['iter']} 步指令重测")
        loop.set_target(best["cmd"].tolist())
        time.sleep(0.1)
        iden.wait_cmd_caught(loop, stop)
        still_r = iden.wait_still(loop, stop, still_short)
        samp = loop.arm_sample(wait=False)
        q_final = np.asarray(samp["q_meas"], float)
        m = _metrics(grav, q_final, target)
        reverted = {"to_iter": best["iter"], **m}
        steps.append({"iter": "revert", "to_iter": best["iter"], **m,
                      "t_s": time.monotonic() - t0, "timed_out": still_r["timed_out"]})
    e = q_final - target
    ee = ee_error(grav, q_final, target)
    return {
        "q_final": q_final.tolist(), "err_rad": e.tolist(),
        "err_max_deg": m["err_max_deg"], "err_rms_deg": m["err_rms_deg"],
        "err_max_joint": m.get("err_max_joint"), "err_max_joint_name": m.get("err_max_joint_name"),
        "err_max_joint_signed_deg": m.get("err_max_joint_signed_deg"),
        "settle_s": r0["settle_s"], "total_s": time.monotonic() - t0, "timed_out": r0["timed_out"],
        "ee": ee, "corrections": steps,
        "corr_requested": int(iters), "corr_applied": applied, "corr_skipped_at": skipped_at,
        "corr_stop_reason": stop_reason, "corr_reverted": reverted,
        "corr_thr_deg": thr.round(3).tolist(),
        "corr_band_deg": None if band_deg is None else np.round(band_deg, 3).tolist(),
        "corr_timed_out": int(sum(1 for s in steps if s["iter"] != 0 and s.get("timed_out"))),
    }


def summarize(rows: List[Dict], order: Optional[List[str]] = None) -> Dict:
    out = {}
    labels = order or list(dict.fromkeys(r["mode"] for r in rows))
    for m in labels:
        rs = [r for r in rows if r["mode"] == m]
        if not rs:
            continue
        worst = max(rs, key=lambda r: r["err_max_deg"])
        counts = Counter(r.get("err_max_joint_name") for r in rs if r.get("err_max_joint_name"))
        out[m] = {
            "n": len(rs),
            "err_max_deg_mean": float(np.mean([r["err_max_deg"] for r in rs])),
            "err_max_deg_worst": float(np.max([r["err_max_deg"] for r in rs])),
            "err_max_deg_worst_pose": worst.get("pose"),
            "err_rms_deg_mean": float(np.mean([r["err_rms_deg"] for r in rs])),
            "ee_mm_mean": float(np.mean([r["ee_mm"] for r in rs])),
            "ee_mm_worst": float(np.max([r["ee_mm"] for r in rs])),
            "total_s_mean": float(np.mean([r["total_s"] for r in rs])),
            "n_timed_out": int(sum(1 for r in rs if r.get("timed_out"))),
            "corr_applied": int(sum(r.get("corr_applied") or 0 for r in rs)),
            "err_max_joint_counts": dict(counts),
        }
    return out


class CompareTask:
    def __init__(self, loop, poses_cfg: Dict, urdf_path: Path, base_link: str,
                 log: Callable[[str], None], on_done: Callable[[Dict], None],
                 still: Optional[Dict] = None, modes: Optional[List[str]] = None,
                 payload: Optional[Dict] = None, correction_iters: int = 2,
                 on_row: Optional[Callable[[Dict, Dict], None]] = None):
        self.loop = loop
        self.poses_cfg = poses_cfg
        self.urdf_path = urdf_path
        self.base_link = base_link
        self.log = log
        self.on_done = on_done
        self.on_row = on_row
        self.still = {**iden.DEFAULT_STILL, **(still or {})}
        self.modes = modes or ["baseline", "payload"]
        self.payload = payload
        self.correction_iters = int(correction_iters)
        self.stop = threading.Event()
        self.phase = "待命"
        self.progress = [0, 0]
        self.rows: List[Dict] = []
        self.summary: Dict = {}
        self.band_info: Dict = {}
        self.session = f"{time.strftime('%Y%m%d_%H%M%S')}_{poses_cfg['arm']}"

    def run(self) -> None:
        original = snapshot_payload(self.loop)
        try:
            poses = self.poses_cfg.get("poses") or []
            if len(poses) < 1:
                raise ValueError("姿态集是空的")
            arm = self.poses_cfg["arm"]
            if self.loop.arm != arm:
                raise RuntimeError(f"控制臂是 {self.loop.arm}，姿态集是 {arm}")
            st = self.loop.status()
            if not st.get("engaged") or st.get("float"):
                raise RuntimeError("先接管并退出拖拽再对比")
            grav = ArmGravity(arm, self.urdf_path, self.base_link)
            kp = st.get("kp_vec")
            band = friction.table_for(arm, self.poses_cfg.get("name"), kp)
            self.band_info = band.to_dict()
            src = ("无双向数据，常数兜底 0.30°" if band.meta.get("fallback")
                   else f"{band.meta['n_sessions']} 次双向测量 / {band.meta['n_poses']} 个姿态")
            self.log(f"摩擦带：{src}，默认 {[round(float(v), 2) for v in band.default.tolist()]}°")

            variants = []
            if "baseline" in self.modes:
                variants.append({"label": "基线（无补偿）", "kind": "baseline", "payload": None, "iters": 0})
            if "payload" in self.modes:
                if not self.payload or not self.payload.get("theta"):
                    raise ValueError("补偿方案需要先选补偿文件")
                variants.append({"label": f"补偿「{self.payload.get('name') or '?'}」", "kind": "payload",
                                 "payload": self.payload, "iters": 0})
            if "payload_correct" in self.modes:
                if not self.payload or not self.payload.get("theta"):
                    raise ValueError("补偿+修正需要先选补偿文件")
                variants.append({"label": f"补偿+修正×{self.correction_iters}", "kind": "payload_correct",
                                 "payload": self.payload, "iters": self.correction_iters})
            if not variants:
                raise ValueError("勾选至少一种方案")

            labels = [v["label"] for v in variants]
            total = len(variants) * len(poses)
            self.progress = [0, total]
            self.log(f"对比 {len(poses)} 点 × {len(variants)} 方案，会话 {self.session}，限速 {self.loop.max_speed:.3f}")

            for v in variants:
                apply_payload(self.loop, v["payload"])
                self.log(f"方案「{v['label']}」")
                home = self.poses_cfg.get("home_q")
                if home:
                    self.phase = f"{v['label']} 回 home"
                    iden.wait_settle(self.loop, home, self.stop, self.still)
                for i, pose in enumerate(poses):
                    if self.stop.is_set():
                        raise RuntimeError("已停止")
                    self.phase = f"{v['label']} {pose.get('name')}"
                    r = goto_with_correction(
                        self.loop, grav, pose["q"], self.still, v["iters"], self.stop,
                        CORRECTION_DEADBAND, band.for_pose(pose.get("name")),
                        on_phase=lambda s, lab=v["label"], pn=pose.get("name"): setattr(self, "phase", f"{lab} {pn} {s}"),
                        on_log=lambda s, lab=v["label"], pn=pose.get("name"): self.log(f"{lab} {pn}: {s}"),
                    )
                    row = {
                        "mode": v["label"], "kind": v["kind"], "pose": pose.get("name"), "index": i,
                        "err_max_deg": r["err_max_deg"], "err_max_joint_name": r.get("err_max_joint_name"),
                        "err_rms_deg": r["err_rms_deg"], "ee_mm": r["ee"]["pos_mm"],
                        "ee_rot_deg": r["ee"]["rot_deg"], "settle_s": r["settle_s"], "total_s": r["total_s"],
                        "timed_out": r["timed_out"], "corr_applied": r.get("corr_applied"),
                        "corr_stop_reason": r.get("corr_stop_reason"), "corr_reverted": bool(r.get("corr_reverted")),
                    }
                    self.rows.append(row)
                    self.summary = summarize(self.rows, labels)
                    self.progress = [len(self.rows), total]
                    if self.on_row:
                        self.on_row(row, self.summary)
                    self.log(f"{v['label']} {pose.get('name')}: {fmt_max_joint(r)}  末端 {r['ee']['pos_mm']:.1f} mm")
                if home:
                    iden.wait_settle(self.loop, home, self.stop, self.still)

            result = {
                "ok": True, "session": self.session, "arm": arm,
                "poseset": self.poses_cfg.get("name"),
                "friction_band": self.band_info, "rows": self.rows, "summary": self.summary,
                "finished": time.strftime("%Y-%m-%dT%H:%M:%S"),
            }
            validate_dir().mkdir(parents=True, exist_ok=True)
            (validate_dir() / f"{self.session}.json").write_text(
                json.dumps(result, indent=1, ensure_ascii=False), encoding="utf-8")
            self.phase = "完成"
            self.on_done(result)
        except Exception as e:
            self.phase = "失败"
            self.on_done({"ok": False, "error": f"{type(e).__name__}: {e}", "session": self.session,
                          "rows": self.rows, "summary": self.summary})
        finally:
            try:
                apply_payload(self.loop, original)
            except Exception:
                pass
