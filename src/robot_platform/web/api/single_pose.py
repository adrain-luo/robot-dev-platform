"""单姿态记录和到位：/api/single-pose/*（list / record / rename / delete / goto）。

到位用「已应用的 IK 策略」（工作姿态那边的 single / robust / luo）解记下的法兰位姿（以记录时的关节角
为起点和参照，冗余的手肘也按记录复现），限速走过去、
等停稳、用实测角复核。走的期间租着这只手（急停撤销租约就停）；到位后若这只手锁着工作姿态就解锁（姿态已变）。
"""

from __future__ import annotations

from typing import Any, Dict

import numpy as np

from g1_contracts.motion import MotionRejected
from robot_platform.calib.payload import identify as iden
from robot_platform.control.arm_loop import PermissionErrorMotion
from robot_platform.tasks import single_pose as sp
from robot_platform.tasks.spatial_points import (
    pose_ori_miss_deg, solve_pose_ik_by_mode,
)
from robot_platform.tasks.work_pose import IK_LABELS

OWNER = "单姿态到位"
PLAN_MAX_MM = 8.0
PLAN_MAX_DEG = 5.0
SETTLE = {"still_dq_rad_s": 0.05, "still_drift_rad": 0.015, "still_window_s": 0.5, "settle_timeout_s": 12.0}


class SinglePoseMixin:
    def single_pose_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        try:
            if path == "/api/single-pose/list":
                arm = str(body.get("arm") or "")
                return {"ok": True, "list": sp.list_poses(sp.check_arm(arm) if arm else None),
                        "ik_mode": self.work_pose.ik_mode}
            if path == "/api/single-pose/record":
                return self._single_pose_record(body)
            if path == "/api/single-pose/rename":
                d = sp.rename(str(body.get("name") or ""), str(body.get("new_name") or ""))
                return {"ok": True, "pose": sp._summary(d), "list": sp.list_poses(),
                        "message": f"已改名为「{d['name']}」"}
            if path == "/api/single-pose/delete":
                name = str(body.get("name") or "")
                sp.delete(name)
                return {"ok": True, "list": sp.list_poses(), "message": f"已删除「{name}」"}
            if path == "/api/single-pose/goto":
                return self._single_pose_goto(str(body.get("name") or ""))
            return {"ok": False, "error": "没有这个单姿态接口"}
        except (ValueError, KeyError, FileNotFoundError, OSError, PermissionErrorMotion, MotionRejected,
                RuntimeError) as exc:
            return {"ok": False, "error": str(exc)}

    def _single_pose_record(self, body: Dict[str, Any]) -> Dict[str, Any]:
        arm = sp.check_arm(str(body.get("arm") or ""))
        core = self.motion_core
        layout = core.layout(arm) if core is not None else None
        if layout is None:
            from robot_platform.core.layout import get_arm
            layout = get_arm(arm)
        q = np.asarray(self.reader().read_arm(layout).q, dtype=float)
        model = self.kin.arms[arm].model
        p, R = model.fk(q)
        tcp_name, tcp_xyz = "", None
        try:
            tcp_name, tcp = self._tcp_for_arm(arm)
            tcp_xyz = np.asarray(p, dtype=float) + R @ np.asarray(tcp, dtype=float).reshape(3)
        except ValueError:
            pass  # 没调用 TCP 也能记，只是不显示 TCP 位置
        d = sp.record(str(body.get("name") or ""), arm, q, p, R, tcp_name=tcp_name, tcp_xyz=tcp_xyz,
                      note=str(body.get("note") or ""), overwrite=bool(body.get("overwrite")))
        side = "左" if arm == "left" else "右"
        return {"ok": True, "pose": sp._summary(d), "list": sp.list_poses(),
                "message": f"已记录{side}手姿态「{d['name']}」"}

    def _single_pose_goto(self, name: str) -> Dict[str, Any]:
        motion = self.motion()
        doc = sp.load(name)
        arm = sp.check_arm(str(doc.get("arm") or ""))
        side = "左" if arm == "left" else "右"
        st = motion.arm_view_status(arm)
        if not st.get("engaged"):
            raise PermissionErrorMotion("先接管再到位")
        if st.get("float"):
            raise PermissionErrorMotion(f"{side}臂在拖拽，先退出拖拽再到位")
        model = self.kin.arms[arm].model
        p_t = np.asarray(doc["p"], dtype=float).reshape(3)
        R_t = np.asarray(doc["R"], dtype=float).reshape(3, 3)
        q_rec = np.asarray(doc["q"], dtype=float).reshape(-1)
        zero = np.zeros(3)
        with motion.lease_arm(arm, OWNER) as lease:
            q_now = np.asarray(self.reader().read_arm(lease.layout).q, dtype=float)
            mode = self.work_pose.ik_mode
            # 7 轴有冗余：同一末端位姿有一族关节解。以记录时的关节角为起点 / 参照，
            # 求解器取最贴近记录姿态的那组（手肘也复现），而不是离当前角最近的那组
            q, miss_mm, ori_deg = solve_pose_ik_by_mode(
                mode, model, q_rec, zero, p_t, R_t, extra_seeds=[q_now], n_random=12, n_ok_stop=5,
            )
            if miss_mm > PLAN_MAX_MM or ori_deg > PLAN_MAX_DEG:
                raise ValueError(f"到不了「{name}」：位置还差 {miss_mm:.1f} mm，姿态还差 {ori_deg:.1f}°")
            settle = iden.wait_settle(lease, q, lease.cancelled, SETTLE)
            if lease.cancelled.is_set():
                raise RuntimeError("已停止")
            st2 = lease.status()
            if not st2.get("engaged") or st2.get("float"):
                raise PermissionErrorMotion("到位前已释放或进入拖拽")
            q_meas = np.asarray(lease.arm_sample(wait=True)["q_meas"], dtype=float)
        p2, R2 = model.fk(q_meas)
        pos_miss = float(np.linalg.norm(p2 - p_t) * 1000.0)
        ori_miss = float(pose_ori_miss_deg(R_t, R2))
        joint_diff = float(np.degrees(np.max(np.abs(q_meas - q_rec))))
        # 姿态变了：这只手锁着的工作姿态不再成立
        self.unlock_work_pose("single_pose", arm)
        mode_label = IK_LABELS.get(mode, mode)
        return {
            "ok": True,
            "arm": arm,
            "name": name,
            "ik_mode": mode,
            "plan_miss_mm": float(miss_mm),
            "plan_ori_deg": float(ori_deg),
            "measured_miss_mm": pos_miss,
            "measured_ori_deg": ori_miss,
            "joint_diff_deg": joint_diff,
            "settle_s": float(settle.get("settle_s") or 0.0),
            "settle_timed_out": bool(settle.get("timed_out")),
            "message": (f"{side}手已到「{name}」（{mode_label} IK）：末端差 {pos_miss:.1f} mm、{ori_miss:.1f}°，"
                        f"关节与记录时最大差 {joint_diff:.1f}°"),
        }
