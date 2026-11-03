#!/usr/bin/env python3
"""G1D arm viewer / optional motion.

  python server.py                         # 只读（先 conda activate abyss）
  python server.py --allow-real-motion --max-speed 0.2
"""

from __future__ import annotations

import argparse
import atexit
import json
import math
import os
import re
import signal
import numpy as np
import sys
import threading
import time
import traceback
from html import escape
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent
PAYLOAD_ROOT = HERE.parent / "arm_payload_gravity"
HAND_EYE_ROOT = HERE.parent / "hand_eye"
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
if str(PAYLOAD_ROOT) not in sys.path:
    sys.path.insert(0, str(PAYLOAD_ROOT))
if str(HAND_EYE_ROOT) not in sys.path:
    sys.path.insert(0, str(HAND_EYE_ROOT))

from backends import BACKENDS, create_backend
from backends.base import Backend, LiftDriver, LiftReader, StateSource
from config_loader import load_settings
import instance as instances
from controller.arm_loop import ArmLoop, PermissionErrorMotion
import lift_calib
from lift_control import LiftController
from model.fk_scene import RobotKinematics
from model.layout import get_arm, short_joint_label
from model.robot_profile import activate as activate_profile
import identify as iden
import poses as posesets
import validate as vali
from api import PayloadAPI
from board_touch import BoardTouch
from detect_localize import DetectLocalize
from runtime_config import ConfigLockedError, RuntimeConfig
from spatial_points import (
    SpatialPoints,
    axis_angle_R,
    list_saved as list_spatial_saved,
    solve_tcp_ik,
    solve_tcp_pose_ik,
    solve_tcp_pose_ik_robust,
)
from tcp_calib import TcpSession
from work_pose import WorkPoseSession, delete_all_for_tcp as delete_work_poses_for_tcp
import session as he_session
from service import HandEyeAPI

def _opt(value: str, label: str) -> str:
    return f'<option value="{escape(value, quote=True)}">{escape(label)}</option>'


def _inject_options(html: str, select_id: str, options: list) -> str:
    pattern = rf'(<select\b[^>]*\bid="{re.escape(select_id)}"[^>]*>)\s*(</select>)'
    inner = "".join(options)
    new, _ = re.subn(pattern, rf"\1{inner}\2", html, count=1)
    return new


def _page_html() -> str:
    html = (FRONTEND / "index.html").read_text(encoding="utf-8")
    try:
        pose_opts = [_opt("", "选择姿态集…")]
        for item in posesets.list_sets():
            side = "左" if item.get("arm") == "left" else "右"
            cond = f" · cond {item['cond']}" if item.get("cond") is not None else ""
            pose_opts.append(_opt(item["name"], f"{item['name']} · {side} · {item['n']}点{cond}"))
        html = _inject_options(html, "poseset-select", pose_opts)
        sess_opts = [_opt("", "选择测量会话再求解…")]
        sessions = iden.list_sessions()
        if not sessions:
            sess_opts = [_opt("", "还没有测量会话（先点测量当前集）")]
        for item in sessions:
            side = "左" if item.get("arm") == "left" else "右"
            bi = "双向" if item.get("bidirectional") else "单向"
            sess_opts.append(_opt(item["name"], f"{item['name']} · {side} {item.get('poseset') or ''} {item.get('n')}样 {bi}"))
        html = _inject_options(html, "id-session", sess_opts)
        vals = vali.list_results()
        val_opts = [_opt("", "打开已保存的对比…" if vals else "还没有对比记录")]
        for item in vals:
            side = "左" if item.get("arm") == "left" else "右"
            val_opts.append(_opt(item["name"], f"data/validate/{item['name']}.json · {side} {item.get('poseset') or ''}"))
        html = _inject_options(html, "val-session", val_opts)
        he_opts = [_opt("", "选择采集会话…")]
        sessions = he_session.list_sessions()
        if not sessions:
            he_opts = [_opt("", "还没有采集会话")]
        for item in sessions:
            side = "左" if item.get("arm") == "left" else "右"
            ps = f" · {item['poseset']}" if item.get("poseset") else ""
            he_opts.append(_opt(item["name"], f"{item['name']} · {side} {item.get('n', 0)}张{ps}"))
        html = _inject_options(html, "he-session", he_opts)
        pts = list_spatial_saved()
        pt_opts = [_opt("", "选择空间点…" if pts else "还没有空间点")]
        for item in pts:
            side = "左" if item.get("arm") == "left" else "右" if item.get("arm") == "right" else ""
            pt_opts.append(_opt(item["name"], f"{item['name']} · {side} · {item.get('tcp_name') or ''}".rstrip(" ·")))
        html = _inject_options(html, "pt-select", pt_opts)
    except Exception:
        traceback.print_exc()
    return html


FRONTEND = HERE / "frontend"


class App:
    def __init__(self, settings, kin: RobotKinematics, allow_real_motion: bool, backend: Backend):
        self.settings = settings
        self.profile = settings.profile
        self.kin = kin
        self.allow_real_motion = bool(allow_real_motion)
        self.backend = backend
        self._reader: Optional[StateSource] = None
        self._hispeed: Optional[LiftReader] = None
        self._agv: Optional[LiftDriver] = None
        self._lift_ctrl: Optional[LiftController] = None
        self.loop: Optional[ArmLoop] = None
        self._lock = threading.Lock()
        self.payload = PayloadAPI(self)
        self.tcp = TcpSession()
        self.points = SpatialPoints()
        self.work_pose = WorkPoseSession()
        self.board = BoardTouch(self)
        self.detect = DetectLocalize(self)
        self.runtime = RuntimeConfig(self)
        # 工作姿态到位 / 已锁定下的碰点与棋盘：set_target 时不自动解锁
        self._work_pose_motion = False
        # 限速到位共用：急停 / 工作流「停止」置位，打断 wait_settle
        self.motion_stop = threading.Event()
        try:
            self.tcp.restore_active()
            self.points.restore_active()
        except (OSError, json.JSONDecodeError, ValueError):
            traceback.print_exc()
        self.handeye = HandEyeAPI(self)

    def reader(self) -> StateSource:
        with self._lock:
            if self._reader is None:
                self._reader = self.backend.state()
                if self.allow_real_motion:
                    self.loop = ArmLoop(
                        reader=self._reader,
                        commander=self.backend.commander(),
                        kin=self.kin,
                        arm="right",
                        max_speed_rad_s=self.settings.max_speed_rad_s,
                        urdf_path=self.settings.urdf,
                        base_link=self.settings.base_link,
                    )
                    self.payload.restore(self.loop)
            return self._reader

    def _require_lift(self):
        if self.profile.lift is None:
            raise PermissionErrorMotion(f"{self.profile.name} 没有立柱")
        return self.profile.lift

    def hispeed(self) -> LiftReader:
        self._require_lift()
        # 先起状态订阅；勿在 _lock 里调 reader，锁不可重入
        self.reader()
        with self._lock:
            if self._hispeed is None:
                self._hispeed = self.backend.lift_reader()
            return self._hispeed

    def agv(self) -> LiftDriver:
        self._require_lift()
        self.reader()
        with self._lock:
            if self._agv is None:
                self._agv = self.backend.lift_driver()
            return self._agv

    def lift_ctrl(self) -> LiftController:
        with self._lock:
            if self._lift_ctrl is None:
                # hispeed/agv 在锁外初始化，避免重入
                pass
        hs = self.hispeed()
        agv = self.agv()
        with self._lock:
            if self._lift_ctrl is None:
                self._lift_ctrl = LiftController(
                    hispeed=hs, agv=agv, stroke_max_m=self.profile.lift.stroke_max_m
                )
            return self._lift_ctrl

    def require_motion(self) -> None:
        if not self.allow_real_motion:
            raise PermissionErrorMotion("只读模式，不能发运动指令（需 --allow-real-motion）")

    def lift_set_zero_now(self) -> Dict[str, Any]:
        st = self.hispeed().latest()
        if st is None:
            return {"ok": False, "error": f"还没有 {self.profile.lift.state_topic} 数据，稍后再试", "lift": None}
        out = lift_calib.save_zero(st.height_m)
        # 立刻按新零位重算，给前端刷新
        scene = self.both_scenes()
        out["lift"] = scene.get("lift")
        out["message"] = (
            f"已记最低点 {st.height_m:.4f} m（{st.height_m * 1000.0:.1f} mm），三维行程归零"
        )
        return out

    def lift_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        if self.profile.lift is None:
            if path == "/api/lift/stop":
                return {"ok": True, "busy": False, "message": "没有立柱"}
            return {"ok": False, "error": f"{self.profile.name} 没有立柱"}
        if path == "/api/lift/set-zero":
            return self.lift_set_zero_now()
        if path == "/api/lift/stop":
            if self._lift_ctrl is None and self._agv is None:
                return {"ok": True, "busy": False, "message": "立柱本来就没在动"}
            return self.lift_ctrl().stop()
        self.require_motion()
        ctrl = self.lift_ctrl()
        if path == "/api/lift/goto":
            if "stroke_mm" in body and body.get("stroke_mm") not in (None, ""):
                zero = lift_calib.load_zero()
                height = zero + float(body["stroke_mm"]) / 1000.0
            elif "height_m" in body and body.get("height_m") not in (None, ""):
                height = float(body["height_m"])
            else:
                return {"ok": False, "error": "需要 stroke_mm 或 height_m"}
            out = ctrl.goto(height)
            out["message"] = f"立柱目标行程 {(out.get('target_m', height) - lift_calib.load_zero()) * 1000.0:.1f} mm"
            return out
        if path == "/api/lift/step":
            if "delta_mm" in body and body.get("delta_mm") not in (None, ""):
                delta = float(body["delta_mm"]) / 1000.0
            elif "delta_m" in body and body.get("delta_m") not in (None, ""):
                delta = float(body["delta_m"])
            else:
                return {"ok": False, "error": "需要 delta_mm"}
            out = ctrl.step(delta)
            if out.get("ok"):
                sign = "升" if delta > 0 else "降"
                out["message"] = f"立柱{sign}一步 {abs(delta) * 1000.0:.1f} mm"
            return out
        return {"ok": False, "error": "没有这个立柱接口"}

    def ctrl_status(self) -> Dict[str, Any]:
        base = {
            "allow_real_motion": self.allow_real_motion,
            "engaged": False,
            "float": False,
            "gravity_ff": True,
            "arm": self.loop.arm if self.loop else "right",
            "max_speed_rad_s": self.settings.max_speed_rad_s,
        }
        if self.loop is None:
            return base
        base.update(self.loop.status())
        th = base.get("payload_theta")
        if th:
            m, com = iden.theta_to_payload(th)
            base["payload_mass_kg"] = m
            base["payload_com_m"] = com
        return base

    def _urdf(self):
        return self.settings.urdf, self.settings.base_link


    def snapshot(self, arm: str) -> Dict[str, Any]:
        layout = get_arm(arm)
        r = self.reader()
        st = r.read_arm(layout)
        age = r.age_s()
        scene = self.kin.arm_scene(arm, st.q)
        ctrl = self.ctrl_status()
        if self.loop is not None and self.loop.arm == arm and ctrl.get("engaged"):
            scene["cmd_links"] = self.kin.arm_scene(arm, ctrl["cmd_rad"])["links"]
            scene["target_links"] = self.kin.arm_scene(arm, ctrl["desired_rad"])["links"]
        if self.loop is not None and self.loop.arm == arm and ctrl.get("payload_theta"):
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
            "ctrl": ctrl,
        }

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
            "lift": lift,
            "tcp": self.tcp.status(),
            "spatial": self.points.status(),
            "work_pose": self.work_pose.status(self.tcp.active_name or ""),
            "board": self.board.status(),
            "detect": self.detect.status(),
            "runtime": self.runtime.snapshot(),
            "arms": arms,
        }

    def runtime_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        try:
            if path == "/api/runtime-config/lock":
                return self.runtime.set_locked(bool(body.get("locked")))
            if path == "/api/runtime-config/stereo":
                return self.runtime.apply_stereo(active_result=body.get("active_result"))
            if path == "/api/runtime-config/gravity":
                return self.runtime.apply_gravity(str(body.get("arm") or "right"), str(body.get("name") or ""))
            if path == "/api/runtime-config/tcp":
                out = self.runtime.apply_tcp(
                    str(body.get("name") or ""),
                    arm=body.get("arm"),
                )
                self.work_pose.unlock_if_tcp_mismatch(self.tcp.active_name or "")
                return out
            if path == "/api/runtime-config/work-pose":
                # 不受配置锁定限制：确认后到位并锁定工作姿态
                return self.work_pose_load(
                    str(body.get("tcp_name") or self.tcp.active_name or ""),
                    str(body.get("name") or ""),
                )
            if path == "/api/runtime-config/pose-ik-mode":
                # 不受配置锁定：随时切换锁定姿态下的 IK 策略
                out = self.work_pose.set_ik_mode(str(body.get("mode") or ""))
                snap = self.runtime.snapshot()
                snap["work_pose"] = out
                snap["ok"] = True
                snap["message"] = (
                    "已切到多初值（新）" if out.get("ik_mode") == "robust" else "已切到单次局部 IK（旧）"
                )
                return snap
            if path == "/api/runtime-config":
                return self.runtime.snapshot()
            return {"ok": False, "error": "没有这个配置接口"}
        except ConfigLockedError as exc:
            out = self.runtime.snapshot()
            out["ok"] = False
            out["error"] = str(exc)
            return out
        except (ValueError, FileNotFoundError, OSError, KeyError, json.JSONDecodeError, PermissionErrorMotion, RuntimeError) as exc:
            out = self.runtime.snapshot()
            out["ok"] = False
            out["error"] = str(exc)
            return out

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
            from spatial_points import _rotvec_from_R
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

    def require_loop(self) -> ArmLoop:
        self.reader()
        if self.loop is None:
            raise PermissionErrorMotion("未开 --allow-real-motion，不能发令")
        return self.loop


APP: Optional[App] = None


def _json(handler: BaseHTTPRequestHandler, code: int, payload: Dict[str, Any]) -> None:
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    handler.send_response(code)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Cache-Control", "no-store")
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def _safe_under(root: Path, name: str) -> Optional[Path]:
    if not name or name != Path(name).name or ".." in name:
        return None
    path = (root / name).resolve()
    root_r = root.resolve()
    if path.parent != root_r or not path.is_file():
        return None
    return path


def _file(handler: BaseHTTPRequestHandler, path: Path, content_type: str) -> None:
    data = path.read_bytes()
    handler.send_response(200)
    handler.send_header("Content-Type", content_type)
    handler.send_header("Cache-Control", "no-cache")
    handler.send_header("Content-Length", str(len(data)))
    handler.end_headers()
    handler.wfile.write(data)


def _read_json(handler: BaseHTTPRequestHandler) -> Dict[str, Any]:
    n = int(handler.headers.get("Content-Length") or 0)
    raw = handler.rfile.read(n) if n else b"{}"
    if not raw:
        return {}
    return json.loads(raw.decode("utf-8"))


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args) -> None:
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))

    def do_GET(self) -> None:  # noqa: N802
        assert APP is not None
        parsed = urlparse(self.path)
        path = parsed.path
        try:
            if path in ("/", "/index.html"):
                data = _page_html().encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            if path in ("/rail-preview", "/rail-preview.html"):
                _file(self, FRONTEND / "rail_preview.html", "text/html; charset=utf-8")
                return
            if path == "/static/style.css":
                _file(self, FRONTEND / "style.css", "text/css; charset=utf-8")
                return
            if path == "/static/app.js":
                _file(self, FRONTEND / "app.js", "application/javascript; charset=utf-8")
                return
            if path == "/static/viewer.js":
                _file(self, FRONTEND / "viewer.js", "text/javascript; charset=utf-8")
                return
            if path.startswith("/static/vendor/"):
                name = path.split("/")[-1]
                vendor = _safe_under(FRONTEND / "vendor", name)
                if vendor is None:
                    self.send_error(404)
                    return
                _file(self, vendor, "text/javascript; charset=utf-8")
                return
            if path in ("/urdf/robot.urdf", f"/urdf/{APP.profile.urdf.name}"):
                _file(self, APP.profile.urdf, "application/xml")
                return
            if path.startswith("/urdf/meshes/"):
                mesh = _safe_under(APP.profile.meshes_dir, path.split("/")[-1])
                if mesh is None:
                    self.send_error(404)
                    return
                _file(self, mesh, "application/octet-stream")
                return
            if path == "/api/runtime-config":
                _json(self, 200, APP.runtime.snapshot())
                return
            if path == "/api/status":
                q = parse_qs(parsed.query)
                arm = (q.get("arm") or ["right"])[0]
                _json(self, 200, APP.snapshot(arm))
                return
            if path == "/api/scene":
                _json(self, 200, APP.both_scenes())
                return
            if path == "/api/handeye/preview.jpg":
                jpeg, err = APP.handeye.preview_jpeg()
                if not jpeg:
                    _json(self, 503, {"ok": False, "error": err or "没有画面"})
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Content-Length", str(len(jpeg)))
                self.end_headers()
                self.wfile.write(jpeg)
                return
            got = APP.handeye.handle_get(path, parsed.query)
            if got is not None:
                _json(self, 200, got)
                return
            got = APP.payload.handle_get(path, parsed.query)
            if got is not None:
                _json(self, 200, got)
                return
            self.send_error(404)
        except Exception as e:
            traceback.print_exc()
            _json(self, 500, {"ok": False, "error": f"{type(e).__name__}: {e}"})

    def do_POST(self) -> None:  # noqa: N802
        assert APP is not None
        path = urlparse(self.path).path
        try:
            body = _read_json(self)
            if path.startswith("/api/runtime-config"):
                _json(self, 200, APP.runtime_post(path, body))
                return
            try:
                APP.runtime.guard_path(path, body)
            except ConfigLockedError as exc:
                _json(self, 200, {"ok": False, "error": str(exc), "runtime": APP.runtime.snapshot()})
                return
            posted = APP.handeye.handle_post(path, body)
            if posted is None:
                posted = APP.payload.handle_post(path, body)
            if posted is not None:
                if isinstance(posted, dict):
                    posted.setdefault("runtime", APP.runtime.snapshot())
                _json(self, 200, posted)
                return
            if path.startswith("/api/tcp/"):
                out = APP.tcp_post(path, body)
                if isinstance(out, dict):
                    out.setdefault("runtime", APP.runtime.snapshot())
                    out.setdefault("work_pose", APP.work_pose.status(APP.tcp.active_name or ""))
                _json(self, 200, out)
                return
            if path.startswith("/api/work-pose/"):
                out = APP.work_pose_post(path, body)
                if isinstance(out, dict):
                    out.setdefault("runtime", APP.runtime.snapshot())
                _json(self, 200, out)
                return
            if path.startswith("/api/board/"):
                if path == "/api/board/look":
                    _json(self, 200, APP.board.look(
                        str(body.get("board") or "9x6"),
                        bool(body.get("ideal")),
                    ))
                elif path == "/api/board/start":
                    _json(self, 200, APP.board.start(
                        float(body.get("above_cm") or 5.0) / 100.0,
                        float(body.get("down_cm") or 4.0) / 100.0,
                        float(body.get("up_cm") or 4.0) / 100.0,
                    ))
                elif path == "/api/board/stop":
                    APP.board.request_stop()
                    if APP.loop is not None and APP.loop.status().get("engaged"):
                        APP.loop.estop_hold()
                    _json(self, 200, APP.board.status())
                else:
                    _json(self, 200, {"ok": False, "error": "没有这个棋盘接口"})
                return
            if path.startswith("/api/detect/") and path != "/api/detect/grab":
                if path == "/api/detect/points":
                    _json(self, 200, APP.detect.acquire_points(body))
                elif path == "/api/detect/calibrate":
                    _json(self, 200, APP.detect.calibrate())
                elif path == "/api/detect/center":
                    _json(self, 200, APP.detect.extract_center())
                elif path == "/api/detect/clear":
                    _json(self, 200, APP.detect.clear())
                else:
                    _json(self, 200, {"ok": False, "error": "没有这个识别定位接口"})
                return
            if path.startswith("/api/points/"):
                if path == "/api/points/goto":
                    _json(self, 200, APP.point_goto(body, APP.require_loop()))
                elif path == "/api/points/goto-xyz":
                    _json(self, 200, APP.point_goto_xyz(body, APP.require_loop()))
                else:
                    _json(self, 200, APP.point_post(path, body))
                return
            if path.startswith("/api/lift/"):
                try:
                    _json(self, 200, APP.lift_post(path, body))
                except PermissionErrorMotion as e:
                    _json(self, 403, {"ok": False, "error": str(e)})
                return
            loop = APP.require_loop()
            if path == "/api/engage":
                loop.engage()
            elif path == "/api/release":
                loop.release()
            elif path == "/api/hold":
                APP.motion_stop.set()
                APP.board.request_stop()
                try:
                    APP.lift_post("/api/lift/stop", {})
                except Exception:
                    traceback.print_exc()
                loop.estop_hold()
            elif path == "/api/teach":
                if bool(body.get("enabled", True)):
                    APP.unlock_work_pose("teach")
                    loop.enable_teach()
                else:
                    loop.disable_teach()
            elif path == "/api/gravity":
                loop.set_gravity_ff(bool(body.get("enabled", True)))
            elif path == "/api/speed":
                loop.set_max_speed(float(body["max_speed_rad_s"]))
                APP.settings.max_speed_rad_s = loop.max_speed
            elif path == "/api/target":
                if not APP._work_pose_motion:
                    APP.unlock_work_pose("arm_control")
                loop.set_target(body["q"])
            elif path == "/api/waist":
                yaw = body.get("yaw_deg")
                pitch = body.get("pitch_deg")
                loop.set_waist(
                    None if yaw in (None, "") else math.radians(float(yaw)),
                    None if pitch in (None, "") else math.radians(float(pitch)),
                )
            elif path == "/api/waist/step":
                # 相对当前目标（或实测）步进；与立柱 step 同思路
                spec = APP.profile.waist
                if spec is None:
                    raise PermissionErrorMotion(f"{APP.profile.name} 没有可控的腰")
                m_yaw = spec.joint("yaw").motor
                m_pitch = spec.joint("pitch").motor
                st_w = APP.reader().read_motor_states(spec.read_motors)
                tgt = (APP.ctrl_status() or {}).get("waist_target_rad") or {}
                base_yaw = float(tgt.get(str(m_yaw), st_w[m_yaw]["q"]))
                base_pitch = float(tgt.get(str(m_pitch), st_w[m_pitch]["q"]))
                dy = body.get("delta_yaw_deg")
                dp = body.get("delta_pitch_deg")
                new_yaw = base_yaw if dy in (None, "") else base_yaw + math.radians(float(dy))
                new_pitch = base_pitch if dp in (None, "") else base_pitch + math.radians(float(dp))
                loop.set_waist(new_yaw, new_pitch)
            elif path == "/api/waist/stop":
                loop.hold_waist()
            elif path == "/api/control-arm":
                loop.set_arm(str(body["arm"]))
                APP.payload.restore(loop)
                APP.tcp.restore_for_arm(loop.arm)
                APP.work_pose.unlock_if_tcp_mismatch(APP.tcp.active_name or "")
            else:
                self.send_error(404)
                return
            _json(self, 200, {
                "ok": True,
                "ctrl": APP.ctrl_status(),
                "work_pose": APP.work_pose.status(APP.tcp.active_name or ""),
            })
        except PermissionErrorMotion as e:
            _json(self, 403, {"ok": False, "error": str(e)})
        except Exception as e:
            traceback.print_exc()
            _json(self, 500, {"ok": False, "error": f"{type(e).__name__}: {e}"})


def main() -> int:
    global APP
    p = argparse.ArgumentParser(description="Arm viewer / optional motion")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=10184)
    p.add_argument("--instance", default=None,
                   help="机器实例（instances/<名字>），默认按 ARM_INSTANCE / instances/default；mock 默认用临时实例")
    p.add_argument("--robot", default=None, help="机型名（robots/<名字>/robot.json），默认取实例里的 robot")
    p.add_argument("--backend", default=None, choices=BACKENDS, help="硬件后端，默认取机型 robot.json；mock 为仿真")
    p.add_argument("--iface", default=None)
    p.add_argument("--allow-real-motion", action="store_true")
    p.add_argument("--max-speed", type=float, default=None, help="rad/s, default 0.2, clamp 0.02–1.0")
    args = p.parse_args()

    backend_name = args.backend or os.environ.get("ARM_BACKEND")
    explicit = args.instance or os.environ.get("ARM_INSTANCE")
    if backend_name == "mock" and not explicit:
        # 仿真默认不碰任何真实机器的标定数据
        inst = instances.temporary_instance(args.robot or "g1d")
        atexit.register(instances.cleanup, inst)
    else:
        inst = instances.resolve(args.instance)
    instances.activate(inst)
    settings = load_settings(inst, robot=args.robot, backend=args.backend)
    activate_profile(settings.profile)
    if args.iface:
        settings.network_interface = args.iface
    if args.max_speed is not None:
        settings.max_speed_rad_s = float(max(0.02, min(1.0, args.max_speed)))
    if settings.urdf is None or not settings.urdf.is_file():
        raise SystemExit(f"URDF not found: {settings.urdf}")

    kin = RobotKinematics(settings.urdf, settings.base_link, list(settings.profile.arms))
    backend = create_backend(settings.backend, settings)
    # 仿真后端不接硬件，运动按钮默认可用
    allow_motion = args.allow_real_motion or backend.name == "mock"
    APP = App(settings, kin, allow_real_motion=allow_motion, backend=backend)
    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    mode = "REAL MOTION" if args.allow_real_motion else "readonly"
    if backend.name == "mock":
        mode = "MOCK"
    print(
        f"[arm_control] {settings.robot} {mode}  instance={inst.name}  http://{args.host}:{args.port}/  "
        f"iface={settings.network_interface}  max_speed={settings.max_speed_rad_s}",
        flush=True,
    )
    threading.Thread(target=lambda: APP.reader(), name="dds-warmup", daemon=True).start()

    def _on_term(*_):
        raise KeyboardInterrupt

    # kill / systemctl stop 也走下面的收尾：先释放（arm_sdk 降权重），再退出
    signal.signal(signal.SIGTERM, _on_term)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[arm_control] stopped")
        if APP.handeye:
            APP.handeye.close()
        if APP.loop:
            APP.loop.shutdown()
        backend.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
