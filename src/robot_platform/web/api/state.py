"""状态与场景：控制状态、单臂快照、机型能力、整机场景（前端轮询的 /api/scene）。"""

from __future__ import annotations

from typing import Any, Dict
import math
import time

from robot_platform.calib import lift as lift_calib
from robot_platform.calib.payload import identify as iden
from robot_platform.core import instance as instances
from robot_platform.control.gravity_dir import down_in_base, rpy_to_R, tilt_angles
from robot_platform.core.layout import get_arm, short_joint_label


class StateMixin:
    def body_arm(self, body: Dict[str, Any], fallback: Any = None) -> str:
        """请求里说明是哪只手（arm=left/right）；两只手都在控制循环里，没有「当前控制臂」可兜底。"""
        arm = str((body or {}).get("arm") or fallback or "")
        if arm not in ("left", "right"):
            raise ValueError("说明是哪只手（arm = left / right）")
        return arm

    def ctrl_status(self) -> Dict[str, Any]:
        base = {
            "allow_real_motion": self.allow_real_motion,
            "engaged": False,
            "float": False,
            "gravity_ff": True,
            "max_speed_rad_s": self.settings.max_speed_rad_s,
        }
        mon = getattr(self, "cmd_monitor", None)
        if mon is not None:
            base["cmd_bus"] = mon.report()
        if self.loop is None:
            return base
        base.update(self.loop.status())
        for a in (base.get("arms") or {}).values():
            th = a.get("payload_theta")
            if th:
                m, com = iden.theta_to_payload(th)
                a["payload_mass_kg"] = m
                a["payload_com_m"] = com
        return base

    def _urdf(self):
        return self.settings.urdf, self.settings.base_link

    def snapshot(self, arm: str) -> Dict[str, Any]:
        layout = get_arm(arm)
        r = self.reader()
        st = r.read_arm(layout)
        age = r.age_s()
        scene = self.kin.arm_scene(arm, st.q)
        ctrl_all = self.ctrl_status()
        ctrl = (ctrl_all.get("arms") or {}).get(arm) or {}
        if ctrl and ctrl_all.get("engaged"):
            scene["cmd_links"] = self.kin.arm_scene(arm, ctrl["cmd_rad"])["links"]
            scene["target_links"] = self.kin.arm_scene(arm, ctrl["desired_rad"])["links"]
        if ctrl.get("payload_theta"):
            com = ctrl.get("payload_com_m")
            mass = ctrl.get("payload_mass_kg")
            if com is None:
                mass, com = iden.theta_to_payload(ctrl["payload_theta"])
            T = scene["wrist_T"]
            o = [T[i][3] for i in range(3)]
            scene["payload_point"] = [
                o[0] + T[0][0] * com[0] + T[0][1] * com[1] + T[0][2] * com[2],
                o[1] + T[1][0] * com[0] + T[1][1] * com[1] + T[1][2] * com[2],
                o[2] + T[2][0] * com[0] + T[2][1] * com[1] + T[2][2] * com[2],
            ]
            scene["payload_mass"] = float(mass or 0.0)
            scene["payload_name"] = ctrl.get("payload_name")
            scene["payload_com_m"] = [float(v) for v in com]
        try:
            lim = self.kin.arms[layout.name].model.joint_limits()
        except Exception:
            lim = None
        joints = []
        for i, name in enumerate(layout.joint_names):
            tau = st.tau_est[i]
            row = {
                "name": name,
                "label": short_joint_label(name),
                "motor": layout.motor_indices[i],
                "q_rad": st.q[i],
                "q_deg": math.degrees(st.q[i]),
                "dq": st.dq[i],
                "tau_est": None if tau is None else float(tau),
            }
            if lim is not None:
                row["limit_rad"] = [float(lim[i, 0]), float(lim[i, 1])]
                row["limit_deg"] = [math.degrees(float(lim[i, 0])), math.degrees(float(lim[i, 1]))]
            joints.append(row)
        return {
            "ok": True,
            "mode": "real" if self.allow_real_motion else "readonly",
            "robot": self.settings.robot,
            "backend": self.backend.name,
            "instance": instances.active().name,
            "arm": arm,
            "iface": self.settings.network_interface,
            "topic": self.settings.state_topic,
            "command_topic": self.settings.command_topic,
            "mode_machine": st.mode_machine,
            "age_s": age,
            "fresh": age < 0.5,
            "joints": joints,
            "scene": scene,
            "ctrl": ctrl_all,
        }

    def _transition_s(self):
        """arm_sdk 接管 / 释放的总时长（秒），给页面提示用；其它后端为 None。"""
        if self.backend.name != "unitree_arm_sdk":
            return None
        o = self.profile.control.options
        return {"engage": float(o.get("acquire_s", 8.0)) + float(o.get("gain_ramp_s", 4.0)),
                "release": float(o.get("release_gain_s", 2.0)) + float(o.get("release_s", 3.0))}

    def robot_caps(self) -> Dict[str, Any]:
        """给前端的机型能力：决定显示哪些控件。"""
        p = self.profile
        w = p.waist
        return {
            "ok": True,
            "name": p.name,
            "label": p.name.upper(),
            "display_name": p.display_name,
            "backend": self.backend.name,
            "instance": instances.active().name,
            "arms": list(p.arms),
            "waist": None if w is None else {
                "movable": w.movable,
                "joints": [
                    {"role": j.role, "name": j.urdf_joint, "motor": j.motor,
                     "limit_deg": [math.degrees(j.limit_rad[0]), math.degrees(j.limit_rad[1])]}
                    for j in w.joints
                ],
            },
            "lift": None if p.lift is None else {"stroke_max_mm": p.lift.stroke_max_m * 1000.0},
            # 接管 / 释放有过渡（arm_sdk）：页面显示进度，过渡中不能走点
            "transitions": self.backend.name == "unitree_arm_sdk",
            "transition_s": self._transition_s(),
            # 腿式机型：三维里根在骨盆，要按腿部实测贴地
            "legged": bool(p.display_joints),
        }

    def body_joints(self) -> Dict[str, float]:
        """只用于三维显示的其它关节（如 G1 的腿、G1D 按躯干 IMU 摆的腰俯仰）。"""
        out = self._torso_tilt_joint()
        disp = self.profile.display_joints
        if not disp:
            return out
        try:
            st = self.reader().read_motor_states(list(disp))
        except Exception:
            return out
        out.update({name: st[i]["q"] for i, name in disp.items()})
        return out

    def _torso_tilt_joint(self) -> Dict[str, float]:
        """机型声明了 torso_imu.display_pitch_joint：按躯干 IMU 测到的俯仰摆这个 URDF 关节（只给三维用）。

        G1D 没有腰俯仰电机，躯干实际后仰约 3～4°，不这样三维里躯干永远是竖直的。
        IMU 没数据、断流或倾角离谱时不给（三维按 0 画）。只读模式也读（不发任何指令）。
        """
        spec = self.profile.torso_imu
        if spec is None or not spec.display_pitch_joint:
            return {}
        now = time.monotonic()
        if now < getattr(self, "_torso_view_retry_at", 0.0):
            return {}
        try:
            self.reader()
            imu = self.backend.torso_imu()
            got = imu.latest() if imu is not None else None
        except Exception:  # noqa: BLE001  IMU 起不来：三维照旧按 0 画，过一会儿再试
            self._torso_view_retry_at = now + 5.0
            return {}
        if got is None or now - got[1] > 0.5:
            return {}
        try:
            t = tilt_angles(down_in_base(got[0], rpy_to_R(*spec.mount_rpy)))
        except ValueError:
            return {}
        if t["tilt_deg"] > spec.max_tilt_deg:
            return {}
        return {spec.display_pitch_joint: math.radians(t["pitch_deg"])}

    def _imu(self):
        """机身 IMU 姿态，给三维把骨盆按重力摆正（只用横滚 / 俯仰）。没有就 None。"""
        try:
            return self.reader().read_imu()
        except Exception:
            return None

    def both_scenes(self) -> Dict[str, Any]:
        self.reader()
        arms = {}
        err = None
        for name in self.profile.arms:
            try:
                arms[name] = self.snapshot(name)
            except Exception as e:
                err = f"{type(e).__name__}: {e}"
                arms[name] = None
        waist = {}
        lift = {}
        spec = self.profile.waist
        try:
            if spec is None:
                raise LookupError("no waist")
            st_w = self.reader().read_motor_states(spec.read_motors)
            def _tau(v: float):
                return None if v != v else float(v)  # NaN → None
            waist = {"limits_deg": {}, "joints": []}
            for j in spec.joints:
                s = st_w[j.motor]
                waist[j.urdf_joint] = s["q"]
                waist[f"{j.role}_deg"] = math.degrees(s["q"])
                waist["limits_deg"][j.role] = [math.degrees(j.limit_rad[0]), math.degrees(j.limit_rad[1])]
                waist["joints"].append({
                    "name": j.urdf_joint,
                    "label": f"waist_{j.role}",
                    "motor": j.motor,
                    "q_rad": s["q"],
                    "q_deg": math.degrees(s["q"]),
                    "dq": s["dq"],
                    "tau_est": _tau(s["tau_est"]),
                })
        except Exception:
            waist = {}
        lift_spec = self.profile.lift
        # 官方：rt/hispeed_state Point32.y=高度 .z=速度（不在 lowstate）
        try:
            if lift_spec is None:
                raise LookupError("no lift")
            topic = lift_spec.state_topic
            hs = self.hispeed()
            st = hs.latest()
            age = hs.age_s()
            calib = lift_calib.status()
            zero_m = float(calib["height_zero_m"])
            if st is None:
                lift = {
                    "available": False,
                    "topic": topic,
                    "height_m": None,
                    "velocity_mps": None,
                    "age_s": None,
                    "joints": {n: 0.0 for n in lift_spec.urdf_joints},
                    "joints_urdf": list(lift_spec.urdf_joints),
                    "calib": calib,
                    "note": f"等待 {topic}（Point32.y 高度 / .z 速度）。升降不在 rt/lowstate。",
                }
            else:
                joints = lift_spec.height_to_joints(st.height_m, zero_m=zero_m)
                stroke = sum(joints.values())
                lift = {
                    "available": True,
                    "topic": topic,
                    "height_m": st.height_m,
                    "height_mm": st.height_m * 1000.0,
                    "stroke_m": stroke,
                    "stroke_mm": stroke * 1000.0,
                    "velocity_mps": st.velocity_mps,
                    "velocity_mms": st.velocity_mps * 1000.0,
                    "x": st.x,
                    "age_s": age,
                    "joints": joints,
                    "joints_urdf": list(lift_spec.urdf_joints),
                    "calib": calib,
                    "note": f"来自 {topic}；已映射到 {' / '.join(n.removesuffix('_Joint') for n in lift_spec.urdf_joints)} 驱动三维",
                }
        except Exception as e:
            if lift_spec is None:
                lift = {}
            else:
                lift = {
                    "available": False,
                    "topic": lift_spec.state_topic,
                    "error": f"{type(e).__name__}: {e}",
                    "calib": lift_calib.status(),
                    "note": f"订阅 {lift_spec.state_topic} 失败",
                }
        if self._lift_ctrl is not None:
            try:
                lift = {**lift, "ctrl": self._lift_ctrl.status()}
            except Exception:
                pass
        return {
            "ok": err is None,
            "error": err,
            "mode": "real" if self.allow_real_motion else "readonly",
            "robot": self.settings.robot,
            "backend": self.backend.name,
            "instance": instances.active().name,
            "iface": self.settings.network_interface,
            "topic": self.settings.state_topic,
            "command_topic": self.settings.command_topic,
            "ctrl": self.ctrl_status(),
            "identify": self.payload.identify_status(),
            "handeye": self.handeye.status(),
            "waist": waist,
            "body": self.body_joints(),
            "imu": self._imu(),
            "lift": lift,
            "tcp": self.tcp.status(),
            "spatial": self.points.status(),
            "work_pose": self.work_pose.status(self.tcp.active_name or ""),
            "board": self.board.status(),
            "detect": self.detect.status(),
            "runtime": self.runtime.snapshot(),
            "arms": arms,
        }
