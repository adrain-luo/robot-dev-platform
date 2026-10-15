#!/usr/bin/env python3
"""G1D arm viewer / optional motion.

  python server.py                         # 只读（先 conda activate abyss）
  python server.py --allow-real-motion --max-speed 0.2
"""

from __future__ import annotations

import argparse
import json
import math
import re
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

from config_loader import load_settings
from controller.arm_loop import ArmLoop, PermissionErrorMotion
from dds.lowstate import LowStateReader
from model.fk_scene import RobotKinematics
from model.layout import (
    ARMS,
    WAIST_PITCH_JOINT,
    WAIST_PITCH_MOTOR_INDEX,
    WAIST_YAW_JOINT,
    WAIST_YAW_MOTOR_INDEX,
    get_arm,
    short_joint_label,
)
import identify as iden
import poses as posesets
import validate as vali
from api import PayloadAPI
from spatial_points import SpatialPoints, list_saved as list_spatial_saved, solve_tcp_ik
from tcp_calib import TcpSession, list_saved as list_tcp_saved
import session as he_session
import results as he_results
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
        profiles = iden.list_profiles()
        pay_opts = [_opt("", "选择补偿文件…" if profiles else "还没有补偿文件")]
        for item in profiles:
            side = "左" if item.get("arm") == "left" else "右"
            mass = item.get("mass_kg")
            m = f" {float(mass):.2f}kg" if mass is not None else ""
            pay_opts.append(_opt(item["name"], f"{item['name']} · {side}{m}"))
        html = _inject_options(html, "payload-select", pay_opts)
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
        saved = he_results.list_saved()
        cal_opts = [_opt("", "调用已保存的标定…" if saved else "还没有保存的标定")]
        for item in saved:
            base = f"{item['baseline_mm']:.1f} mm" if item.get("baseline_mm") is not None else ""
            cal_opts.append(_opt(item["name"], f"{item['name']} · {base}".rstrip(" ·")))
        html = _inject_options(html, "he-calib", cal_opts)
        tcp_saved = list_tcp_saved()
        tcp_opts = [_opt("", "调用已保存的 TCP…" if tcp_saved else "还没有保存的 TCP")]
        for item in tcp_saved:
            side = "左" if item.get("arm") == "left" else "右" if item.get("arm") == "right" else ""
            res = item.get("residual_max_mm")
            tail = f" · {side} {item.get('n') or 0}姿态" if side else ""
            if res is not None:
                tail += f" · 残差 {float(res):.1f} mm"
            tcp_opts.append(_opt(item["name"], f"{item['name']}{tail}"))
        html = _inject_options(html, "tcp-select", tcp_opts)
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
    def __init__(self, settings, kin: RobotKinematics, allow_real_motion: bool):
        self.settings = settings
        self.kin = kin
        self.allow_real_motion = bool(allow_real_motion)
        self._reader: Optional[LowStateReader] = None
        self.loop: Optional[ArmLoop] = None
        self._lock = threading.Lock()
        self.payload = PayloadAPI(self)
        self.tcp = TcpSession()
        self.points = SpatialPoints()
        try:
            self.tcp.restore_active()
            self.points.restore_active()
        except (OSError, json.JSONDecodeError, ValueError):
            traceback.print_exc()
        self.handeye = HandEyeAPI(self)

    def reader(self) -> LowStateReader:
        with self._lock:
            if self._reader is None:
                self._reader = LowStateReader(
                    network_interface=self.settings.network_interface,
                    state_topic=self.settings.state_topic,
                    dds_domain=self.settings.dds_domain,
                    timeout_s=self.settings.lowstate_timeout_s,
                    sdk_path=self.settings.unitree_sdk_path,
                )
                if self.allow_real_motion:
                    self.loop = ArmLoop(
                        reader=self._reader,
                        kin=self.kin,
                        arm="right",
                        command_topic=self.settings.command_topic,
                        max_speed_rad_s=self.settings.max_speed_rad_s,
                        sdk_path=self.settings.unitree_sdk_path,
                        dds_domain=self.settings.dds_domain,
                        network_interface=self.settings.network_interface,
                        urdf_path=self.settings.urdf,
                        base_link=self.settings.base_link,
                    )
                    self.payload.restore(self.loop)
            return self._reader

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
        joints = []
        for i, name in enumerate(layout.joint_names):
            tau = st.tau_est[i]
            joints.append(
                {
                    "name": name,
                    "label": short_joint_label(name),
                    "motor": layout.motor_indices[i],
                    "q_rad": st.q[i],
                    "q_deg": math.degrees(st.q[i]),
                    "dq": st.dq[i],
                    "tau_est": None if tau is None else float(tau),
                }
            )
        return {
            "ok": True,
            "mode": "real" if self.allow_real_motion else "readonly",
            "robot": self.settings.robot,
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
        for name in ARMS:
            try:
                arms[name] = self.snapshot(name)
            except Exception as e:
                err = f"{type(e).__name__}: {e}"
                arms[name] = None
        waist = {}
        try:
            q = self.reader().read_motors((WAIST_YAW_MOTOR_INDEX, WAIST_PITCH_MOTOR_INDEX))
            waist = {
                WAIST_YAW_JOINT: q[WAIST_YAW_MOTOR_INDEX],
                WAIST_PITCH_JOINT: q[WAIST_PITCH_MOTOR_INDEX],
            }
        except Exception:
            waist = {}
        return {
            "ok": err is None,
            "error": err,
            "mode": "real" if self.allow_real_motion else "readonly",
            "robot": self.settings.robot,
            "iface": self.settings.network_interface,
            "topic": self.settings.state_topic,
            "command_topic": self.settings.command_topic,
            "ctrl": self.ctrl_status(),
            "identify": self.payload.identify_status(),
            "handeye": self.handeye.status(),
            "waist": waist,
            "tcp": self.tcp.status(),
            "spatial": self.points.status(),
            "arms": arms,
        }

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
            elif path == "/api/tcp/delete":
                status = self.tcp.delete(str(body.get("name") or ""))
            else:
                return {"ok": False, "error": "没有这个 TCP 接口"}
            return status
        except (ValueError, KeyError, FileNotFoundError, OSError) as exc:
            out = self.tcp.status()
            out["ok"] = False
            out["error"] = str(exc)
            return out

    def _active_tcp(self) -> tuple:
        result = self.tcp.result or {}
        tcp = result.get("tcp_m")
        if not self.tcp.arm or tcp is None:
            raise ValueError("先在 TCP 卡片里调用一个结果")
        return self.tcp.arm, self.tcp.active_name or self.tcp.tip_link, np.asarray(tcp, dtype=float)

    def point_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        try:
            name = str(body.get("name") or "")
            note = str(body.get("note") or "")
            if path == "/api/points/save-fixed":
                arm, tcp_name, tcp = self._active_tcp()
                fixed = (self.tcp.result or {}).get("fixed_point_m")
                if fixed is None:
                    raise ValueError("这个 TCP 没有标定固定点")
                status = self.points.save(name, arm, tcp_name, tcp, fixed, note)
            elif path == "/api/points/save-now":
                arm, tcp_name, tcp = self._active_tcp()
                st = self.reader().read_arm(get_arm(arm))
                t, R = self.kin.arms[arm].model.fk(st.q)
                status = self.points.save(name, arm, tcp_name, tcp, t + R @ tcp, note)
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

    def point_goto(self, body: Dict[str, Any], loop: ArmLoop) -> Dict[str, Any]:
        try:
            name = str(body.get("name") or self.points.active_name or "")
            if not name:
                return {"ok": False, "error": "先选一个空间点"}
            if not self.points.doc or self.points.active_name != name:
                self.points.load(name)
            doc = self.points.doc or {}
            if loop.arm != doc.get("arm"):
                side = "左" if doc.get("arm") == "left" else "右"
                return {"ok": False, "error": f"先把控制臂换成{side}臂"}
            q0 = np.asarray(self.reader().read_arm(loop.layout).q, dtype=float)
            q, miss_mm = solve_tcp_ik(self.kin.arms[loop.arm].model, q0, doc["tcp_m"], doc["point_m"])
            if miss_mm > 3.0:
                return {"ok": False, "error": f"从这个姿态到不了该点，TCP 还差 {miss_mm:.1f} mm", "miss_mm": miss_mm}
            loop.set_target(q)
            out = self.points.status()
            out["miss_mm"] = miss_mm
            return out
        except (ValueError, KeyError, FileNotFoundError, OSError) as exc:
            out = self.points.status()
            out["ok"] = False
            out["error"] = str(exc)
            return out

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
            if path == "/urdf/g1_d.urdf":
                _file(self, HERE / "config" / "g1_d.urdf", "application/xml")
                return
            if path.startswith("/urdf/meshes/"):
                mesh = _safe_under(HERE / "config" / "meshes", path.split("/")[-1])
                if mesh is None:
                    self.send_error(404)
                    return
                _file(self, mesh, "application/octet-stream")
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
            posted = APP.handeye.handle_post(path, body)
            if posted is None:
                posted = APP.payload.handle_post(path, body)
            if posted is not None:
                _json(self, 200, posted)
                return
            if path.startswith("/api/tcp/"):
                _json(self, 200, APP.tcp_post(path, body))
                return
            if path.startswith("/api/points/"):
                if path == "/api/points/goto":
                    _json(self, 200, APP.point_goto(body, APP.require_loop()))
                else:
                    _json(self, 200, APP.point_post(path, body))
                return
            loop = APP.require_loop()
            if path == "/api/engage":
                loop.engage()
            elif path == "/api/release":
                loop.release()
            elif path == "/api/hold":
                loop.estop_hold()
            elif path == "/api/teach":
                if bool(body.get("enabled", True)):
                    loop.enable_teach()
                else:
                    loop.disable_teach()
            elif path == "/api/gravity":
                loop.set_gravity_ff(bool(body.get("enabled", True)))
            elif path == "/api/speed":
                loop.set_max_speed(float(body["max_speed_rad_s"]))
                APP.settings.max_speed_rad_s = loop.max_speed
            elif path == "/api/target":
                loop.set_target(body["q"])
            elif path == "/api/waist":
                yaw = body.get("yaw_deg")
                pitch = body.get("pitch_deg")
                loop.set_waist(
                    None if yaw in (None, "") else math.radians(float(yaw)),
                    None if pitch in (None, "") else math.radians(float(pitch)),
                )
            elif path == "/api/control-arm":
                loop.set_arm(str(body["arm"]))
                APP.payload.restore(loop)
            else:
                self.send_error(404)
                return
            _json(self, 200, {"ok": True, "ctrl": APP.ctrl_status()})
        except PermissionErrorMotion as e:
            _json(self, 403, {"ok": False, "error": str(e)})
        except Exception as e:
            traceback.print_exc()
            _json(self, 500, {"ok": False, "error": f"{type(e).__name__}: {e}"})


def main() -> int:
    global APP
    p = argparse.ArgumentParser(description="G1D arm viewer / optional motion")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=10184)
    p.add_argument("--config", type=Path, default=None)
    p.add_argument("--iface", default=None)
    p.add_argument("--allow-real-motion", action="store_true")
    p.add_argument("--max-speed", type=float, default=None, help="rad/s, default 0.2, clamp 0.02–1.0")
    args = p.parse_args()

    settings = load_settings(args.config)
    if args.iface:
        settings.network_interface = args.iface
    if args.max_speed is not None:
        settings.max_speed_rad_s = float(max(0.02, min(1.0, args.max_speed)))
    if settings.urdf is None or not settings.urdf.is_file():
        raise SystemExit(f"URDF not found: {settings.urdf}")

    kin = RobotKinematics(settings.urdf, settings.base_link)
    APP = App(settings, kin, allow_real_motion=args.allow_real_motion)
    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    mode = "REAL MOTION" if args.allow_real_motion else "readonly"
    print(
        f"[arm_control] {mode}  http://{args.host}:{args.port}/  "
        f"iface={settings.network_interface}  max_speed={settings.max_speed_rad_s}",
        flush=True,
    )
    threading.Thread(target=lambda: APP.reader(), name="dds-warmup", daemon=True).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[arm_control] stopped")
        if APP.handeye:
            APP.handeye.close()
        if APP.loop:
            APP.loop.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
