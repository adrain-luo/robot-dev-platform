"""空间点：/api/points/*、按 TCP 位姿限速到位并测量到位误差。"""

from __future__ import annotations

from typing import Any, Dict
import threading

import numpy as np

from robot_platform.calib.payload import identify as iden
from robot_platform.control.arm_loop import ArmLoop, PermissionErrorMotion
from robot_platform.core.layout import get_arm


class PointsMixin:
    def point_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        try:
            name = str(body.get("name") or "")
            note = str(body.get("note") or "")
            if path == "/api/points/save-fixed":
                # 固定点本身在躯干系；取点仍需当前 TCP 会话里的标定结果
                _, _, _ = self._active_tcp()
                fixed = (self.tcp.result or {}).get("fixed_point_m")
                if fixed is None:
                    raise ValueError("这个 TCP 没有标定固定点")
                status = self.points.save(name, fixed, note, source="tcp_fixed")
            elif path == "/api/points/save-now":
                # 用当前控制臂（或请求里的臂）+ 该臂 TCP 算爪尖，但文件只存坐标
                arm = str(body.get("arm") or (self.loop.arm if self.loop else "") or "right")
                if arm not in ("left", "right"):
                    arm = "right"
                _, tcp = self._tcp_for_arm(arm)
                st = self.reader().read_arm(get_arm(arm))
                t, R = self.kin.arms[arm].model.fk(st.q)
                status = self.points.save(name, t + R @ tcp, note, source="tcp_now")
            elif path == "/api/points/save-center":
                det = self.detect
                center = det.center_torso
                if center is None:
                    center = det.face_center_torso
                if center is None:
                    raise ValueError("还没有中心点：先在识别定位里做校准/中心点提取")
                note_out = (note or "").strip() or "识别定位中心点（绿）"
                status = self.points.save(name, center, note_out, source="detect_center")
                status["message"] = (
                    f"已记下中心点 [{float(center[0])*1000:.1f}, "
                    f"{float(center[1])*1000:.1f}, {float(center[2])*1000:.1f}] mm"
                )
            elif path == "/api/points/load":
                status = self.points.load(name)
            elif path == "/api/points/delete":
                status = self.points.delete(name)
            else:
                return {"ok": False, "error": "没有这个空间点接口"}
            return status
        except (ValueError, KeyError, FileNotFoundError, OSError) as exc:
            out = self.points.status()
            out["ok"] = False
            out["error"] = str(exc)
            return out

    def _tcp_pose_from_q(self, model, q, tcp_m) -> np.ndarray:
        t, R = model.fk(np.asarray(q, dtype=float).reshape(-1))
        tcp_vec = np.asarray(tcp_m, dtype=float).reshape(3)
        return np.asarray(t, dtype=float).reshape(3) + np.asarray(R, dtype=float).reshape(3, 3) @ tcp_vec

    def _goto_settle_measure(
        self,
        loop: ArmLoop,
        model,
        q_cmd,
        tcp_m,
        point_arr: np.ndarray,
        stop: threading.Event,
    ) -> Dict[str, Any]:
        """限速走到 q_cmd，停稳后用电机角算 TCP 与目标差。"""
        self._work_pose_motion = True
        try:
            settle = iden.wait_settle(
                loop,
                q_cmd,
                stop,
                {
                    "still_dq_rad_s": 0.05,
                    "still_drift_rad": 0.015,
                    "still_window_s": 0.5,
                    "settle_timeout_s": 12.0,
                },
            )
        finally:
            self._work_pose_motion = False
        st = loop.status()
        if not st.get("engaged") or st.get("float"):
            raise PermissionErrorMotion("到位前已释放或进入拖拽")
        samp = loop.arm_sample(wait=True)
        q_meas = np.asarray(samp["q_meas"], dtype=float)
        if q_meas.size != model.n or not np.all(np.isfinite(q_meas)):
            raise ValueError(f"电机角无效：{q_meas.tolist()}")
        tcp_now = self._tcp_pose_from_q(model, q_meas, tcp_m)
        if not np.all(np.isfinite(tcp_now)):
            raise ValueError("FK 得到的 TCP 含非有限数")
        measured_miss_mm = float(np.linalg.norm(tcp_now - point_arr) * 1000.0)
        if not np.isfinite(measured_miss_mm):
            raise ValueError("实测误差不是有限数")
        return {
            "q_meas": q_meas,
            "tcp_now": tcp_now,
            "measured_miss_mm": measured_miss_mm,
            "settle_s": float(settle.get("settle_s") or 0.0),
            "settle_timed_out": bool(settle.get("timed_out")),
        }

    @staticmethod
    def _parse_offset_mm(body: Dict[str, Any]) -> np.ndarray:
        off = body.get("offset_mm") or body.get("offset") or [0, 0, 0]
        try:
            if isinstance(off, dict):
                off_mm = np.array(
                    [float(off.get("x", 0)), float(off.get("y", 0)), float(off.get("z", 0))],
                    dtype=float,
                )
            else:
                off_mm = np.asarray(off, dtype=float).reshape(3)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"偏置无效：{exc}") from exc
        if not np.all(np.isfinite(off_mm)):
            raise ValueError("偏置含非有限数")
        return off_mm

    def _tcp_goto_target(
        self,
        loop: ArmLoop,
        base_arr: np.ndarray,
        off_mm: np.ndarray,
        *,
        refine_on: bool = True,
        refine_thresh_mm: float = 1.5,
        refine_passes: int = 2,
        refine_gain: float = 1.0,
    ) -> Dict[str, Any]:
        """躯干系 base + 偏置 mm，可选误差补偿。不依赖已保存空间点。"""
        tcp_name, tcp = self._tcp_for_arm(loop.arm)
        model = self.kin.arms[loop.arm].model
        base_arr = np.asarray(base_arr, dtype=float).reshape(3)
        off_mm = np.asarray(off_mm, dtype=float).reshape(3)
        point_arr = base_arr + off_mm / 1000.0
        self.work_pose.unlock_if_tcp_mismatch(self.tcp.active_name or "")
        R_lock = self.work_pose.R_lock if self.work_pose.locked else None
        self.motion_stop.clear()
        stop = self.motion_stop
        q0 = np.asarray(self.reader().read_arm(loop.layout).q, dtype=float)
        q, plan_miss_mm, plan_ori = self._solve_tcp_motion(model, q0, tcp, point_arr, R_lock)
        ori_lim = 3.0 if R_lock is not None else 999.0
        if plan_miss_mm > 3.0 or (plan_ori is not None and plan_ori > ori_lim):
            msg = f"从这个姿态到不了该点，规划 TCP 还差 {plan_miss_mm:.1f} mm"
            if plan_ori is not None:
                msg += f"，姿态还差 {plan_ori:.1f}°"
            return {
                "ok": False,
                "error": msg,
                "miss_mm": plan_miss_mm,
                "plan_miss_mm": plan_miss_mm,
                "plan_ori_deg": plan_ori,
            }

        first = self._goto_settle_measure(loop, model, q, tcp, point_arr, stop)
        measured_before_mm = float(first["measured_miss_mm"])
        refine_passes_n = 0
        max_refine = int(refine_passes) if refine_on else 0
        max_refine = max(0, min(max_refine, 3))
        gain = max(0.2, min(float(refine_gain), 1.5))
        thresh = float(refine_thresh_mm)
        cur = first
        while cur["measured_miss_mm"] > thresh and refine_passes_n < max_refine:
            if stop.is_set():
                raise RuntimeError("已停止")
            err = point_arr - cur["tcp_now"]
            aim = point_arr + gain * err
            q2, plan2, _ori2 = self._solve_tcp_motion(model, cur["q_meas"], tcp, aim, R_lock)
            if plan2 > 5.0:
                break
            if float(np.max(np.abs(q2 - cur["q_meas"]))) < 5e-4:
                break
            cur = self._goto_settle_measure(loop, model, q2, tcp, point_arr, stop)
            refine_passes_n += 1
            if cur["measured_miss_mm"] > measured_before_mm * 1.15 + 1.0:
                break

        tcp_now = cur["tcp_now"]
        return {
            "ok": True,
            "miss_mm": float(plan_miss_mm),
            "plan_miss_mm": float(plan_miss_mm),
            "plan_ori_deg": None if plan_ori is None else float(plan_ori),
            "measured_miss_mm": float(cur["measured_miss_mm"]),
            "measured_before_mm": measured_before_mm,
            "refine_passes": refine_passes_n,
            "refine": bool(refine_on),
            "refine_mode": "error_compensate" if refine_on else "off",
            "work_pose_locked": bool(R_lock is not None),
            "work_pose_name": self.work_pose.active_name if R_lock is not None else "",
            "measured_tcp_m": [float(v) for v in tcp_now.tolist()],
            "measured_tcp_mm": [float(v) * 1000.0 for v in tcp_now.tolist()],
            "settle_s": float(cur["settle_s"]),
            "settle_timed_out": bool(cur["settle_timed_out"]),
            "goto_arm": loop.arm,
            "goto_tcp": tcp_name,
            "offset_mm": [float(v) for v in off_mm.tolist()],
            "target_m": [float(v) for v in point_arr.tolist()],
            "target_mm": [float(v) * 1000.0 for v in point_arr.tolist()],
            "base_mm": [float(v) * 1000.0 for v in base_arr.tolist()],
        }

    def point_goto(self, body: Dict[str, Any], loop: ArmLoop) -> Dict[str, Any]:
        try:
            name = str(body.get("name") or self.points.active_name or "")
            if not name:
                return {"ok": False, "error": "先选一个空间点"}
            if not self.points.doc or self.points.active_name != name:
                self.points.load(name)
            doc = self.points.doc or {}
            point = doc.get("point_m")
            if point is None:
                return {"ok": False, "error": "这个空间点没有坐标"}
            off_mm = self._parse_offset_mm(body)
            refine_on = body.get("refine")
            refine_on = True if refine_on is None else bool(refine_on)
            out = self._tcp_goto_target(
                loop,
                np.asarray(point, dtype=float).reshape(3),
                off_mm,
                refine_on=refine_on,
                refine_thresh_mm=float(body.get("refine_thresh_mm") or 1.5),
                refine_passes=int(body.get("refine_passes") or 2),
                refine_gain=float(body.get("refine_gain") or 1.0),
            )
            status = self.points.status()
            status.update(out)
            return status
        except (ValueError, KeyError, FileNotFoundError, OSError, PermissionErrorMotion) as exc:
            out = self.points.status()
            out["ok"] = False
            out["error"] = str(exc)
            return out
        except RuntimeError as exc:
            out = self.points.status()
            out["ok"] = False
            out["error"] = str(exc)
            return out

    def point_goto_xyz(self, body: Dict[str, Any], loop: ArmLoop) -> Dict[str, Any]:
        """工作流等：直接给躯干系目标点（米）+ 偏置 mm，不落盘空间点。"""
        try:
            raw = body.get("point_m") or body.get("target_m")
            if raw is None and body.get("point_mm") is not None:
                raw = np.asarray(body.get("point_mm"), dtype=float).reshape(3) / 1000.0
            if raw is None:
                return {"ok": False, "error": "缺少 point_m / point_mm"}
            base = np.asarray(raw, dtype=float).reshape(3)
            if not np.all(np.isfinite(base)):
                return {"ok": False, "error": "目标点含非有限数"}
            off_mm = self._parse_offset_mm(body)
            refine_on = body.get("refine")
            refine_on = True if refine_on is None else bool(refine_on)
            return self._tcp_goto_target(
                loop,
                base,
                off_mm,
                refine_on=refine_on,
                refine_thresh_mm=float(body.get("refine_thresh_mm") or 1.5),
                refine_passes=int(body.get("refine_passes") or 2),
                refine_gain=float(body.get("refine_gain") or 1.0),
            )
        except (ValueError, KeyError, FileNotFoundError, OSError, PermissionErrorMotion) as exc:
            return {"ok": False, "error": str(exc)}
        except RuntimeError as exc:
            return {"ok": False, "error": str(exc)}
