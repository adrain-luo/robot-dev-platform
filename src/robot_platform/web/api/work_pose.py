"""工作姿态：/api/work-pose/*、绑定 TCP 的姿态限速到位与锁定。"""

from __future__ import annotations

from typing import Any, Dict

import numpy as np

from robot_platform.calib.payload import identify as iden
from robot_platform.control.arm_loop import PermissionErrorMotion
from robot_platform.core.layout import get_arm
from robot_platform.tasks.spatial_points import solve_tcp_ik, solve_tcp_pose_ik, solve_tcp_pose_ik_robust


class WorkPoseMixin:
    def unlock_work_pose(self, reason: str = "") -> None:
        if self.work_pose.locked:
            self.work_pose.unlock()

    def _solve_tcp_motion(self, model, q0, tcp, target, R_lock):
        """有锁定姿态则按配置做单次/多初值 6D IK，否则纯位置。返回 (q, miss_mm, ori_miss_deg|None)。"""
        if R_lock is not None:
            if self.work_pose.ik_mode == "single":
                q, miss_mm, ori_deg = solve_tcp_pose_ik(model, q0, tcp, target, R_lock)
            else:
                extra = []
                if self.work_pose.q_seed is not None:
                    extra.append(self.work_pose.q_seed)
                q, miss_mm, ori_deg = solve_tcp_pose_ik_robust(
                    model,
                    q0,
                    tcp,
                    target,
                    R_lock,
                    extra_seeds=extra,
                    n_random=16,
                    n_ok_stop=5,
                )
            return q, miss_mm, ori_deg
        q, miss_mm = solve_tcp_ik(model, q0, tcp, target)
        return q, miss_mm, None

    def work_pose_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        try:
            tcp_name = str(body.get("tcp_name") or self.tcp.active_name or "")
            name = str(body.get("name") or "")
            note = str(body.get("note") or "")
            if path == "/api/work-pose/list":
                out = self.work_pose.status(tcp_name)
                return out
            if path == "/api/work-pose/save":
                if not self.tcp.active_name or self.tcp.result is None:
                    raise ValueError("先调用一套 TCP，再记工作姿态")
                arm = self.loop.arm if self.loop is not None else (self.tcp.arm or "right")
                if self.tcp.arm and arm != self.tcp.arm:
                    side = "左" if self.tcp.arm == "left" else "右"
                    raise ValueError(f"当前 TCP 是{side}臂的，先把控制臂换成同一侧")
                layout = get_arm(arm)
                st = self.reader().read_arm(layout)
                scene = self.kin.arm_scene(arm, st.q)
                T = np.asarray(scene["wrist_T"], dtype=float)
                return self.work_pose.save(
                    name,
                    self.tcp.active_name,
                    arm,
                    T[:3, :3],
                    q=st.q,
                    note=note,
                )
            if path == "/api/work-pose/delete":
                return self.work_pose.delete(tcp_name, name)
            if path == "/api/work-pose/clear":
                return self.work_pose.unlock()
            if path == "/api/work-pose/load":
                return self.work_pose_load(tcp_name, name)
            return {"ok": False, "error": "没有这个工作姿态接口"}
        except (ValueError, KeyError, FileNotFoundError, OSError, PermissionErrorMotion) as exc:
            out = self.work_pose.status(str(body.get("tcp_name") or self.tcp.active_name or ""))
            out["ok"] = False
            out["error"] = str(exc)
            return out
        except RuntimeError as exc:
            out = self.work_pose.status(str(body.get("tcp_name") or self.tcp.active_name or ""))
            out["ok"] = False
            out["error"] = str(exc)
            return out

    def work_pose_load(self, tcp_name: str, name: str) -> Dict[str, Any]:
        """转到该工作姿态（保 TCP 位置）并锁定。"""
        loop = self.require_loop()
        st = loop.status()
        if st.get("float"):
            raise PermissionErrorMotion("先点急停退出拖拽，再调用工作姿态")
        if not st.get("engaged"):
            raise PermissionErrorMotion("先接管再调用工作姿态")
        if not self.tcp.active_name or self.tcp.result is None or self.tcp.result.get("tcp_m") is None:
            raise ValueError("先调用一套 TCP")
        tcp_name = tcp_name or self.tcp.active_name
        if tcp_name != self.tcp.active_name:
            raise ValueError(f"该姿态属于 TCP「{tcp_name}」，请先调用该 TCP")
        doc = self.work_pose.load_doc(tcp_name, name)
        if doc.get("arm") and doc.get("arm") != loop.arm:
            side = "左" if doc.get("arm") == "left" else "右"
            raise ValueError(f"这个工作姿态是{side}臂的，先换控制臂")
        tcp = np.asarray(self.tcp.result["tcp_m"], dtype=float).reshape(3)
        R_target = np.asarray(doc["R"], dtype=float).reshape(3, 3)
        model = self.kin.arms[loop.arm].model
        q0 = np.asarray(self.reader().read_arm(loop.layout).q, dtype=float)
        t, R0 = model.fk(q0)
        hold = np.asarray(t, dtype=float).reshape(3) + R0 @ tcp
        self._work_pose_motion = True
        try:
            if self.work_pose.ik_mode == "single":
                q, miss_mm, ori_deg = solve_tcp_pose_ik(model, q0, tcp, hold, R_target)
            else:
                extra = []
                if doc.get("q_rad") is not None:
                    extra.append(doc["q_rad"])
                q, miss_mm, ori_deg = solve_tcp_pose_ik_robust(
                    model, q0, tcp, hold, R_target,
                    extra_seeds=extra, n_random=12, n_ok_stop=5,
                )
            if miss_mm > 8.0 or ori_deg > 5.0:
                raise ValueError(
                    f"到不了该工作姿态：位置还差 {miss_mm:.1f} mm，姿态还差 {ori_deg:.1f}°"
                )
            self.motion_stop.clear()
            settle = iden.wait_settle(
                loop,
                q,
                self.motion_stop,
                {
                    "still_dq_rad_s": 0.05,
                    "still_drift_rad": 0.015,
                    "still_window_s": 0.5,
                    "settle_timeout_s": 12.0,
                },
            )
            if self.motion_stop.is_set():
                raise RuntimeError("已停止")
            if not loop.status().get("engaged") or loop.status().get("float"):
                raise PermissionErrorMotion("到位前已释放或进入拖拽")
            # 用电机角复核
            samp = loop.arm_sample(wait=True)
            q_meas = np.asarray(samp["q_meas"], dtype=float)
            t2, R2 = model.fk(q_meas)
            tcp_now = t2 + R2 @ tcp
            pos_miss = float(np.linalg.norm(tcp_now - hold) * 1000.0)
            # rotvec err
            from robot_platform.tasks.spatial_points import _rotvec_from_R
            ori_miss = float(np.degrees(np.linalg.norm(_rotvec_from_R(R_target @ R2.T))))
            self.work_pose.apply_lock(doc, q_now=q_meas)
            out = self.work_pose.status(tcp_name)
            out["plan_miss_mm"] = float(miss_mm)
            out["plan_ori_deg"] = float(ori_deg)
            out["measured_miss_mm"] = pos_miss
            out["measured_ori_deg"] = ori_miss
            out["settle_s"] = float(settle.get("settle_s") or 0.0)
            out["settle_timed_out"] = bool(settle.get("timed_out"))
            out["message"] = f"已锁定工作姿态「{doc.get('name')}」"
            return out
        finally:
            self._work_pose_motion = False
