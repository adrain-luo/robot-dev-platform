"""TCP：标定会话接口 /api/tcp/*、TCP 点动（躯干系平移 / 绕红点旋转）、按臂取当前 TCP。"""

from __future__ import annotations

from typing import Any, Dict

import numpy as np

from g1_contracts.motion import MotionRejected
from robot_platform.calib import tcp as tcp_calib
from robot_platform.control.arm_loop import PermissionErrorMotion
from robot_platform.core.layout import get_arm
from robot_platform.tasks.spatial_points import axis_angle_R, solve_tcp_pose_ik
from robot_platform.tasks.work_pose import delete_all_for_tcp as delete_work_poses_for_tcp


class TcpMixin:
    def tcp_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        try:
            if path == "/api/tcp/record":
                arm = self.body_arm(body, self.tcp.arm or None)
                layout = get_arm(arm)
                st = self.reader().read_arm(layout)
                scene = self.kin.arm_scene(arm, st.q)
                T = np.asarray(scene["wrist_T"], dtype=float)
                status = self.tcp.record(arm, layout.tip_link, st.q, T[:3, :3], T[:3, 3])
            elif path == "/api/tcp/undo":
                status = self.tcp.undo()
            elif path == "/api/tcp/clear":
                status = self.tcp.clear()
            elif path == "/api/tcp/solve":
                status = self.tcp.solve()
            elif path == "/api/tcp/save":
                status = self.tcp.save(str(body.get("name") or ""), str(body.get("note") or ""))
            elif path == "/api/tcp/load":
                status = self.tcp.load(str(body.get("name") or ""))
                if self.tcp.arm in ("left", "right"):
                    self.work_pose_lock_for(self.tcp.arm, self.tcp.active_name or "")
            elif path == "/api/tcp/mirror":
                status = self.tcp.mirror(
                    str(body.get("name") or ""),
                    force=bool(body.get("force")),
                )
            elif path == "/api/tcp/delete":
                name = str(body.get("name") or "")
                n_wp = delete_work_poses_for_tcp(name)
                if self.work_pose.locked and self.work_pose.tcp_name == name:
                    self.work_pose.unlock()
                status = self.tcp.delete(name)
                status["deleted_work_poses"] = int(n_wp)
                status["message"] = (
                    f"已删除「{name}」"
                    + (f"及 {n_wp} 个工作姿态" if n_wp else "")
                )
            elif path == "/api/tcp/new-flange":
                arm = self.body_arm(body)
                layout = get_arm(arm)
                status = self.tcp.new_flange(str(body.get("name") or ""), layout.name, layout.tip_link,
                                             str(body.get("note") or ""), force=bool(body.get("force")))
            elif path == "/api/tcp/edit-save":
                status = self.tcp.save_edit(
                    str(body.get("src") or ""), body.get("tcp_mm"), str(body.get("mode") or ""),
                    name=str(body.get("name") or ""),
                    note=None if body.get("note") is None else str(body.get("note")),
                    force=bool(body.get("force")),
                )
            elif path == "/api/tcp/jog":
                return self.tcp_jog(body)
            else:
                return {"ok": False, "error": "没有这个 TCP 接口"}
            return status
        except FileExistsError as exc:
            out = self.tcp.status()
            out["ok"] = False
            out["error"] = str(exc)
            out["exists"] = True
            return out
        except (ValueError, KeyError, FileNotFoundError, OSError, PermissionErrorMotion, MotionRejected) as exc:
            out = self.tcp.status()
            out["ok"] = False
            out["error"] = str(exc)
            return out
        except RuntimeError as exc:
            out = self.tcp.status()
            out["ok"] = False
            out["error"] = str(exc)
            return out

    def tcp_jog(self, body: Dict[str, Any]) -> Dict[str, Any]:
        """TCP 点动（body.arm 指定哪只手）：躯干系平移（保姿态）或绕 TCP 点旋转（轴沿躯干）。

        手动来源：不租这只手；这只手被任务租着、急停锁存时目标被拒。"""
        motion = self.motion()
        arm = self.body_arm(body)
        side = "左" if arm == "left" else "右"
        st = motion.page_status()
        if not st.get("engaged"):
            raise PermissionErrorMotion("先接管再点动 TCP")
        if st["arms"][arm].get("float"):
            raise PermissionErrorMotion(f"{side}臂在拖拽，先退出拖拽再点动 TCP")
        tcp_name, tcp = self._tcp_for_arm(arm)
        model = self.kin.arms[arm].model
        mode = str(body.get("mode") or "").strip().lower()
        axis = str(body.get("axis") or "").strip().lower()
        if axis not in ("x", "y", "z"):
            raise ValueError("轴只能是 x / y / z（躯干系）")
        try:
            sign = float(body.get("sign") if body.get("sign") is not None else 1.0)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"方向无效：{exc}") from exc
        sign = 1.0 if sign >= 0.0 else -1.0
        ax_i = {"x": 0, "y": 1, "z": 2}[axis]
        axis_vec = np.zeros(3, dtype=float)
        axis_vec[ax_i] = 1.0

        q0 = np.asarray(self.reader().read_arm(motion.layout(arm)).q, dtype=float)
        t, R = model.fk(q0)
        tcp = np.asarray(tcp, dtype=float).reshape(3)
        c = np.asarray(t, dtype=float).reshape(3) + R @ tcp

        R_lock, _ = self.work_pose_lock_for(arm, tcp_name)

        if mode == "translate":
            try:
                step_mm = float(body.get("step_mm") if body.get("step_mm") is not None else 5.0)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"平移步长无效：{exc}") from exc
            if not (0.2 <= step_mm <= 50.0):
                raise ValueError("平移步长请在 0.2～50 mm")
            delta = axis_vec * (sign * step_mm / 1000.0)
            c_tgt = c + delta
            if R_lock is not None:
                R_tgt = np.asarray(R_lock, dtype=float).reshape(3, 3)
            else:
                R_tgt = R.copy()
            keep_lock = R_lock is not None
            free_axis = self.work_pose_free_axis() if keep_lock else None
            q, miss_mm, ori_deg = solve_tcp_pose_ik(model, q0, tcp, c_tgt, R_tgt, free_axis=free_axis)
            if miss_mm > 3.0 or ori_deg > 3.0:
                raise ValueError(
                    f"平移不到：位置还差 {miss_mm:.1f} mm，姿态还差 {ori_deg:.1f}°"
                )
            motion.manual_target(q, arm=arm)
            label = {"x": ("前", "后"), "y": ("左", "右"), "z": ("上", "下")}[axis]
            dir_name = label[0] if sign > 0 else label[1]
            out = self.tcp.status()
            out["ok"] = True
            out["mode"] = "translate"
            out["axis"] = axis
            out["sign"] = sign
            out["step_mm"] = step_mm
            out["dir"] = dir_name
            out["plan_miss_mm"] = float(miss_mm)
            out["plan_ori_deg"] = float(ori_deg)
            out["work_pose_locked"] = keep_lock
            out["arm"] = arm
            out["message"] = f"{side}臂 TCP {dir_name} {step_mm:g} mm（躯干 {axis.upper()}）"
            return out

        if mode == "rotate":
            try:
                step_deg = float(body.get("step_deg") if body.get("step_deg") is not None else 5.0)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"旋转步长无效：{exc}") from exc
            if not (0.2 <= step_deg <= 30.0):
                raise ValueError("旋转步长请在 0.2～30°")
            # 可平转姿态绕 z 转：绕重力方向转、锁不解（绕重力本来就不约束）；其余旋转会改姿态，解锁
            free_axis = self.work_pose_free_axis() if R_lock is not None else None
            yaw_keep = free_axis is not None and axis == "z"
            if R_lock is not None and not yaw_keep:
                self.unlock_work_pose("tcp_jog_rotate")
                free_axis = None
            R_delta = axis_angle_R(free_axis if yaw_keep else axis_vec, sign * step_deg)
            R_tgt = R_delta @ R
            c_tgt = c.copy()
            q, miss_mm, ori_deg = solve_tcp_pose_ik(model, q0, tcp, c_tgt, R_tgt)
            if miss_mm > 3.0 or ori_deg > 3.0:
                raise ValueError(
                    f"旋转不到：位置还差 {miss_mm:.1f} mm，姿态还差 {ori_deg:.1f}°"
                )
            motion.manual_target(q, arm=arm)
            out = self.tcp.status()
            out["ok"] = True
            out["mode"] = "rotate"
            out["axis"] = axis
            out["sign"] = sign
            out["step_deg"] = step_deg
            out["plan_miss_mm"] = float(miss_mm)
            out["plan_ori_deg"] = float(ori_deg)
            out["work_pose_locked"] = yaw_keep
            out["arm"] = arm
            around = "重力（竖直）" if yaw_keep else f"躯干 {axis.upper()}"
            out["message"] = (
                f"{side}臂 TCP 绕{around} {'+' if sign > 0 else '−'}{step_deg:g}°（红点不动"
                + ("，可平转姿态保持锁定）" if yaw_keep else "）")
            )
            return out

        raise ValueError("mode 只能是 translate 或 rotate")

    def _active_tcp(self) -> tuple:
        result = self.tcp.result or {}
        tcp = result.get("tcp_m")
        if not self.tcp.arm or tcp is None:
            raise ValueError("先在 TCP 卡片里调用一个结果")
        return self.tcp.arm, self.tcp.active_name or self.tcp.tip_link, np.asarray(tcp, dtype=float)

    def _tcp_for_arm(self, arm: str) -> tuple:
        """该臂当前生效的 TCP（只读文件，不动 TCP 标定会话），返回 (tcp_name, tcp_m)。"""
        arm = "left" if arm == "left" else "right"
        doc = tcp_calib.load_active_doc(arm) or {}
        tcp = (doc.get("result") or {}).get("tcp_m")
        if str(doc.get("arm") or "") != arm or tcp is None:
            side = "左" if arm == "left" else "右"
            raise ValueError(f"先为{side}臂调用一套 TCP")
        return str(doc.get("name") or doc.get("tip_link") or ""), np.asarray(tcp, dtype=float)
