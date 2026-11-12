"""工作姿态：/api/work-pose/*、绑定 TCP 的姿态限速到位与锁定。"""

from __future__ import annotations

from typing import Any, Dict

import numpy as np

from robot_platform.calib.payload import identify as iden
from robot_platform.control.arm_loop import PermissionErrorMotion
from robot_platform.core.layout import get_arm
from robot_platform.tasks.spatial_points import (
    pose_ori_miss_deg, solve_tcp_ik, solve_tcp_pose_ik, solve_tcp_pose_ik_robust,
)


class WorkPoseMixin:
    def unlock_work_pose(self, reason: str = "", arm: str = "") -> None:
        """解锁工作姿态；给了 arm 时只在锁的是这只手时解锁（另一只手的锁不受影响）。"""
        if self.work_pose.locked and (not arm or self.work_pose.arm in ("", arm)):
            self.work_pose.unlock()

    def work_pose_lock_for(self, arm: str, tcp_name: str):
        """这只手、这个 TCP 下锁定的姿态 (R_lock, q_seed)；锁的是另一只手时返回 (None, None)。

        锁定的是这只手但 TCP 已换掉时顺手解锁。
        """
        wp = self.work_pose
        if not wp.locked or (wp.arm and wp.arm != arm):
            return None, None
        if (wp.tcp_name or "") != (tcp_name or ""):
            wp.unlock()
            return None, None
        return wp.R_lock, wp.q_seed

    def gravity_up(self) -> np.ndarray:
        """躯干系里的「竖直向上」：有躯干 IMU 用实测，否则躯干 +z。"""
        loop = getattr(self, "loop", None)
        grav = getattr(loop, "gravity", None) if loop is not None else None
        if grav is not None:
            try:
                return np.asarray(grav.up_dir(), dtype=float)
            except Exception:  # noqa: BLE001
                pass
        return np.array([0.0, 0.0, 1.0])

    def free_axis_for(self, free: str):
        """姿态自由度 → IK 的不约束转轴（躯干系）；全锁返回 None。"""
        return self.gravity_up() if free == "gravity_z" else None

    def work_pose_free_axis(self):
        """当前锁定姿态的不约束转轴（可平转 → 重力方向）；没锁或全锁时 None。"""
        wp = self.work_pose
        return self.free_axis_for(wp.free) if wp.locked else None

    def _solve_tcp_motion(self, model, q0, tcp, target, R_lock):
        """有锁定姿态则按配置做单次/多初值 6D IK（可平转时绕重力不约束），否则纯位置。
        返回 (q, miss_mm, ori_miss_deg|None)。"""
        if R_lock is not None:
            free_axis = self.work_pose_free_axis()
            if self.work_pose.ik_mode == "single":
                q, miss_mm, ori_deg = solve_tcp_pose_ik(model, q0, tcp, target, R_lock, free_axis=free_axis)
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
                    free_axis=free_axis,
                )
            return q, miss_mm, ori_deg
        q, miss_mm = solve_tcp_ik(model, q0, tcp, target)
        return q, miss_mm, None

    def work_pose_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        try:
            tcp_name = str(body.get("tcp_name") or self._default_tcp_name(body))
            name = str(body.get("name") or "")
            note = str(body.get("note") or "")
            if path == "/api/work-pose/list":
                out = self.work_pose.status(tcp_name)
                return out
            if path == "/api/work-pose/save":
                arm = self.body_arm(body)
                tcp_name, _ = self._tcp_for_arm(arm)
                layout = get_arm(arm)
                st = self.reader().read_arm(layout)
                scene = self.kin.arm_scene(arm, st.q)
                T = np.asarray(scene["wrist_T"], dtype=float)
                return self.work_pose.save(
                    name,
                    tcp_name,
                    arm,
                    T[:3, :3],
                    q=st.q,
                    note=note,
                    free=str(body.get("free") or ""),
                )
            if path == "/api/work-pose/delete":
                return self.work_pose.delete(tcp_name, name)
            if path == "/api/work-pose/clear":
                return self.work_pose.unlock()
            if path == "/api/work-pose/load":
                return self.work_pose_load(tcp_name, name, str(body.get("arm") or ""))
            return {"ok": False, "error": "没有这个工作姿态接口"}
        except (ValueError, KeyError, FileNotFoundError, OSError, PermissionErrorMotion) as exc:
            out = self.work_pose.status(str(body.get("tcp_name") or self._default_tcp_name(body)))
            out["ok"] = False
            out["error"] = str(exc)
            return out
        except RuntimeError as exc:
            out = self.work_pose.status(str(body.get("tcp_name") or self._default_tcp_name(body)))
            out["ok"] = False
            out["error"] = str(exc)
            return out

    def _default_tcp_name(self, body: Dict[str, Any]) -> str:
        """没给 tcp_name 时：给了 arm 就取那只手生效的 TCP，否则取 TCP 卡片当前的。"""
        arm = str(body.get("arm") or "")
        if arm in ("left", "right"):
            try:
                return self._tcp_for_arm(arm)[0]
            except ValueError:
                return ""
        return self.tcp.active_name or ""

    def work_pose_load(self, tcp_name: str, name: str, arm: str = "") -> Dict[str, Any]:
        """转到该工作姿态（保 TCP 位置）并锁定。姿态记着是哪只手的，只动那只手。"""
        loop = self.require_loop()
        if not tcp_name:
            raise ValueError("先调用一套 TCP")
        doc = self.work_pose.load_doc(tcp_name, name)
        arm = str(doc.get("arm") or arm or "")
        if arm not in loop.arm_names:
            raise ValueError("这个工作姿态没记是哪只手的")
        side = "左" if arm == "left" else "右"
        st = loop.status()
        if not st.get("engaged"):
            raise PermissionErrorMotion("先接管再调用工作姿态")
        if st["arms"][arm].get("float"):
            raise PermissionErrorMotion(f"{side}臂在拖拽，先退出拖拽再调用工作姿态")
        active_name, tcp = self._tcp_for_arm(arm)
        if tcp_name != active_name:
            raise ValueError(f"该姿态属于 TCP「{tcp_name}」，请先为{side}臂调用该 TCP")
        tcp = np.asarray(tcp, dtype=float).reshape(3)
        R_target = np.asarray(doc["R"], dtype=float).reshape(3, 3)
        free_axis = self.free_axis_for(str(doc.get("free") or ""))
        model = self.kin.arms[arm].model
        q0 = np.asarray(self.reader().read_arm(loop.layout(arm)).q, dtype=float)
        view = loop.view(arm)
        t, R0 = model.fk(q0)
        hold = np.asarray(t, dtype=float).reshape(3) + R0 @ tcp
        self._work_pose_motion = True
        try:
            if self.work_pose.ik_mode == "single":
                q, miss_mm, ori_deg = solve_tcp_pose_ik(model, q0, tcp, hold, R_target, free_axis=free_axis)
            else:
                extra = []
                if doc.get("q_rad") is not None:
                    extra.append(doc["q_rad"])
                q, miss_mm, ori_deg = solve_tcp_pose_ik_robust(
                    model, q0, tcp, hold, R_target,
                    extra_seeds=extra, n_random=12, n_ok_stop=5, free_axis=free_axis,
                )
            if miss_mm > 8.0 or ori_deg > 5.0:
                raise ValueError(
                    f"到不了该工作姿态：位置还差 {miss_mm:.1f} mm，姿态还差 {ori_deg:.1f}°"
                )
            self.motion_stop.clear()
            settle = iden.wait_settle(
                view,
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
            st2 = view.status()
            if not st2.get("engaged") or st2.get("float"):
                raise PermissionErrorMotion("到位前已释放或进入拖拽")
            # 用电机角复核
            samp = view.arm_sample(wait=True)
            q_meas = np.asarray(samp["q_meas"], dtype=float)
            t2, R2 = model.fk(q_meas)
            tcp_now = t2 + R2 @ tcp
            pos_miss = float(np.linalg.norm(tcp_now - hold) * 1000.0)
            ori_miss = pose_ori_miss_deg(R_target, R2, free_axis)
            self.work_pose.apply_lock(doc, q_now=q_meas)
            out = self.work_pose.status(tcp_name)
            out["plan_miss_mm"] = float(miss_mm)
            out["plan_ori_deg"] = float(ori_deg)
            out["measured_miss_mm"] = pos_miss
            out["measured_ori_deg"] = ori_miss
            out["settle_s"] = float(settle.get("settle_s") or 0.0)
            out["settle_timed_out"] = bool(settle.get("timed_out"))
            out["arm"] = arm
            out["message"] = f"已锁定{side}臂工作姿态「{doc.get('name')}」"
            return out
        finally:
            self._work_pose_motion = False
