"""TCP：标定会话接口 /api/tcp/*、TCP 点动（躯干系平移 / 绕红点旋转）、按臂取当前 TCP。"""

from __future__ import annotations

from typing import Any, Dict

import numpy as np

from robot_platform.control.arm_loop import PermissionErrorMotion
from robot_platform.core.layout import get_arm
from robot_platform.tasks.spatial_points import axis_angle_R, solve_tcp_pose_ik
from robot_platform.tasks.work_pose import delete_all_for_tcp as delete_work_poses_for_tcp


class TcpMixin:
    def tcp_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        try:
            if path == "/api/tcp/record":
                arm = self.loop.arm if self.loop is not None else "right"
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
                self.work_pose.unlock_if_tcp_mismatch(self.tcp.active_name or "")
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
                arm = str(body.get("arm") or (self.loop.arm if self.loop is not None else "right"))
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
        except (ValueError, KeyError, FileNotFoundError, OSError, PermissionErrorMotion) as exc:
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
        """TCP 点动：躯干系平移（保姿态）或绕 TCP 点旋转（轴沿躯干）。"""
        loop = self.require_loop()
        st = loop.status()
        if st.get("float"):
            raise PermissionErrorMotion("先点急停退出拖拽，再点动 TCP")
        if not st.get("engaged"):
            raise PermissionErrorMotion("先接管再点动 TCP")
        tcp_name, tcp = self._tcp_for_arm(loop.arm)
        model = self.kin.arms[loop.arm].model
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

        q0 = np.asarray(self.reader().read_arm(loop.layout).q, dtype=float)
        t, R = model.fk(q0)
        tcp = np.asarray(tcp, dtype=float).reshape(3)
        c = np.asarray(t, dtype=float).reshape(3) + R @ tcp

        self.work_pose.unlock_if_tcp_mismatch(self.tcp.active_name or "")

        if mode == "translate":
            try:
                step_mm = float(body.get("step_mm") if body.get("step_mm") is not None else 5.0)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"平移步长无效：{exc}") from exc
            if not (0.2 <= step_mm <= 50.0):
                raise ValueError("平移步长请在 0.2～50 mm")
            delta = axis_vec * (sign * step_mm / 1000.0)
            c_tgt = c + delta
            if self.work_pose.locked and self.work_pose.R_lock is not None:
                R_tgt = np.asarray(self.work_pose.R_lock, dtype=float).reshape(3, 3)
            else:
                R_tgt = R.copy()
            keep_lock = bool(self.work_pose.locked)
            self._work_pose_motion = True
            try:
                q, miss_mm, ori_deg = solve_tcp_pose_ik(model, q0, tcp, c_tgt, R_tgt)
                if miss_mm > 3.0 or ori_deg > 3.0:
                    raise ValueError(
                        f"平移不到：位置还差 {miss_mm:.1f} mm，姿态还差 {ori_deg:.1f}°"
                    )
                loop.set_target(q)
            finally:
                self._work_pose_motion = False
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
            out["message"] = f"TCP {dir_name} {step_mm:g} mm（躯干 {axis.upper()}）"
            return out

        if mode == "rotate":
            try:
                step_deg = float(body.get("step_deg") if body.get("step_deg") is not None else 5.0)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"旋转步长无效：{exc}") from exc
            if not (0.2 <= step_deg <= 30.0):
                raise ValueError("旋转步长请在 0.2～30°")
            # 旋转会改姿态，解锁工作姿态
            self.unlock_work_pose("tcp_jog_rotate")
            R_delta = axis_angle_R(axis_vec, sign * step_deg)
            R_tgt = R_delta @ R
            c_tgt = c.copy()
            q, miss_mm, ori_deg = solve_tcp_pose_ik(model, q0, tcp, c_tgt, R_tgt)
            if miss_mm > 3.0 or ori_deg > 3.0:
                raise ValueError(
                    f"旋转不到：位置还差 {miss_mm:.1f} mm，姿态还差 {ori_deg:.1f}°"
                )
            loop.set_target(q)
            out = self.tcp.status()
            out["ok"] = True
            out["mode"] = "rotate"
            out["axis"] = axis
            out["sign"] = sign
            out["step_deg"] = step_deg
            out["plan_miss_mm"] = float(miss_mm)
            out["plan_ori_deg"] = float(ori_deg)
            out["work_pose_locked"] = False
            out["message"] = (
                f"TCP 绕躯干 {axis.upper()} {'+' if sign > 0 else '−'}{step_deg:g}°（红点不动）"
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
        """加载该臂当前生效的 TCP，返回 (tcp_name, tcp_m)。"""
        arm = "left" if arm == "left" else "right"
        self.tcp.restore_for_arm(arm)
        result = self.tcp.result or {}
        tcp = result.get("tcp_m")
        if self.tcp.arm != arm or tcp is None:
            side = "左" if arm == "left" else "右"
            raise ValueError(f"先为{side}臂调用一套 TCP")
        return self.tcp.active_name or self.tcp.tip_link, np.asarray(tcp, dtype=float)
