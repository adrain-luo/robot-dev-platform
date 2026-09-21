#!/usr/bin/env python3
"""G1D arm viewer / optional motion.

  /usr/bin/python3 server.py                         # 只读
  /usr/bin/python3 server.py --allow-real-motion --max-speed 0.2
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from config_loader import load_settings
from controller.arm_loop import ArmLoop, PermissionErrorMotion
from dds.lowstate import LowStateReader
from model.fk_scene import RobotKinematics
from model.layout import ARMS, get_arm, short_joint_label
import poses as posesets
import identify as iden
import friction
import validate as vali

FRONTEND = HERE / "frontend"


class App:
    def __init__(self, settings, kin: RobotKinematics, allow_real_motion: bool):
        self.settings = settings
        self.kin = kin
        self.allow_real_motion = bool(allow_real_motion)
        self._reader: Optional[LowStateReader] = None
        self.loop: Optional[ArmLoop] = None
        self._lock = threading.Lock()
        self._id_lock = threading.Lock()
        self._id_task = None
        self._id: Dict[str, Any] = {
            "running": False, "phase": "", "log": [], "session": None,
            "solution": None, "error": None, "progress": [0, 0],
            "kind": None, "rows": [], "summary": {}, "band": {},
        }
        self.still = dict(iden.DEFAULT_STILL)

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
                    self._restore_payload(self.loop)
            return self._reader

    def _restore_payload(self, loop: ArmLoop) -> None:
        d = iden.load_payload(loop.arm)
        if d and d.get("theta"):
            loop.set_payload_theta(d["theta"], d.get("name"))

    def _apply_payload_dict(self, d: Dict[str, Any], *, name: Optional[str] = None) -> Dict[str, Any]:
        loop = self.require_loop()
        if d.get("arm") and loop.arm != d["arm"]:
            raise PermissionErrorMotion(f"文件是{d.get('arm')}臂，当前控制臂是{loop.arm}")
        theta = d.get("theta")
        if not theta:
            theta = iden.payload_to_theta(d["mass_kg"], d.get("com_m") or [0, 0, 0])
        name = name or d.get("name")
        loop.set_payload_theta(theta, name)
        saved = {
            "arm": loop.arm, "alpha": 1.0, "theta": list(theta),
            "mass_kg": d.get("mass_kg"), "com_m": d.get("com_m"),
            "session": d.get("session"), "poseset": d.get("poseset"),
            "name": name, "note": d.get("note", ""),
            "applied_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }
        if saved["mass_kg"] is None:
            saved["mass_kg"], saved["com_m"] = iden.theta_to_payload(theta)
        iden.save_payload(loop.arm, saved)
        return saved

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

    def identify_status(self) -> Dict[str, Any]:
        with self._id_lock:
            st = {
                "running": self._id["running"],
                "phase": self._id["phase"],
                "log": list(self._id["log"][-16:]),
                "session": self._id["session"],
                "solution": self._id["solution"],
                "error": self._id["error"],
                "progress": list(self._id["progress"]),
                "still": dict(self.still),
            }
            sess = self._id["session"]
            task = self._id_task
        if sess:
            sdir = iden.MEASURE_DIR / str(sess)
            st["session_dir"] = str(sdir)
            arm = (self.loop.arm if self.loop else None) or (st.get("solution") or {}).get("arm")
            if arm:
                pp = iden.payload_path(arm)
                st["payload_file"] = str(pp) if pp.exists() else None
            else:
                st["payload_file"] = None
        else:
            st["session_dir"] = None
            st["payload_file"] = None
        st["profiles"] = iden.list_profiles()
        st["sessions"] = iden.list_sessions()
        st["applied_name"] = None
        if self.loop is not None:
            st["applied_name"] = self.loop.status().get("payload_name")
        st["kind"] = self._id.get("kind")
        st["rows"] = list(self._id.get("rows") or [])
        st["summary"] = dict(self._id.get("summary") or {})
        st["band"] = dict(self._id.get("band") or {})
        if task is not None:
            st["phase"] = task.phase
            st["progress"] = list(task.progress)
            if hasattr(task, "rows"):
                st["rows"] = list(task.rows)
                st["summary"] = dict(getattr(task, "summary") or {})
                st["band"] = dict(getattr(task, "band_info") or {})
        return st

    def _id_log(self, msg: str) -> None:
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        with self._id_lock:
            self._id["log"].append(line)
            self._id["phase"] = msg

    def handle_identify_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        urdf, base = self._urdf()
        if path == "/api/identify/still":
            for k in iden.DEFAULT_STILL:
                if body.get(k) is not None:
                    self.still[k] = float(body[k])
            return {"ok": True, "identify": self.identify_status()}
        if path == "/api/identify/stop":
            with self._id_lock:
                task = self._id_task
            if task:
                task.stop.set()
            return {"ok": True, "identify": self.identify_status()}
        if path == "/api/identify/measure":
            loop = self.require_loop()
            with self._id_lock:
                if self._id["running"]:
                    raise RuntimeError("已有任务在跑")
            d = posesets.load_set(str(body["name"]))
            def done(r: Dict[str, Any]) -> None:
                with self._id_lock:
                    self._id["running"] = False
                    self._id["session"] = r.get("session")
                    self._id["error"] = None if r.get("ok") else r.get("error")
                    self._id["log"].append(
                        f"{time.strftime('%H:%M:%S')} "
                        + ("测量完成 " + str(r.get("session")) if r.get("ok") else str(r.get("error")))
                    )
            task = iden.MeasureTask(loop, d, urdf, base, self._id_log, done, still=self.still)
            with self._id_lock:
                self._id.update(running=True, error=None, solution=None, session=task.session_dir.name,
                                kind="measure", rows=[], summary={})
                self._id_task = task
            threading.Thread(target=task.run, name="identify-measure", daemon=True).start()
            return {"ok": True, "identify": self.identify_status()}
        if path == "/api/identify/solve":
            session = body.get("session") or self.identify_status().get("session")
            if not session:
                raise ValueError("没有测量会话")
            sdir = iden.MEASURE_DIR / str(session)
            samples = json.loads((sdir / "samples.json").read_text(encoding="utf-8"))
            meta = json.loads((sdir / "meta.json").read_text(encoding="utf-8"))
            source = str(body.get("source") or "pd")
            sol = iden.solve_payload(samples, meta["arm"], urdf, base, source=source)
            sol["session"] = str(session)
            sol["poseset"] = meta.get("poseset")
            (sdir / f"payload_{source}.json").write_text(json.dumps(sol, indent=2, ensure_ascii=False), encoding="utf-8")
            with self._id_lock:
                self._id["solution"] = sol
                self._id["session"] = str(session)
            return {"ok": True, "solution": sol, "identify": self.identify_status()}
        if path == "/api/identify/apply":
            with self._id_lock:
                cur = self._id.get("solution")
            if not cur:
                raise ValueError("先求解")
            name = str(body.get("name") or "").strip() or (cur.get("poseset") or "payload")
            saved = self._apply_payload_dict(cur, name=name)
            iden.save_profile(name, saved)
            return {"ok": True, "payload": saved, "identify": self.identify_status(), "ctrl": self.ctrl_status()}
        if path == "/api/identify/clear":
            loop = self.require_loop()
            loop.set_payload_theta(None)
            return {"ok": True, "ctrl": self.ctrl_status()}
        if path == "/api/identify/payload/apply":
            d = iden.load_profile(str(body["name"]))
            saved = self._apply_payload_dict(d, name=d["name"])
            return {"ok": True, "payload": saved, "identify": self.identify_status(), "ctrl": self.ctrl_status()}
        if path == "/api/identify/payload/save":
            with self._id_lock:
                cur = self._id.get("solution")
            if not cur:
                raise ValueError("先求解")
            name = str(body.get("name") or "").strip() or (cur.get("poseset") or "payload")
            saved = iden.save_profile(name, {
                **cur, "arm": cur.get("arm"), "alpha": 1.0,
                "applied_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            }, note=str(body.get("note") or ""))
            return {"ok": True, "payload": saved, "identify": self.identify_status()}
        if path == "/api/identify/payload/delete":
            iden.delete_profile(str(body["name"]))
            return {"ok": True, "identify": self.identify_status()}
        raise FileNotFoundError(path)

    def handle_validate_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        urdf, base = self._urdf()
        if path == "/api/validate/start":
            loop = self.require_loop()
            with self._id_lock:
                if self._id["running"]:
                    raise RuntimeError("已有任务在跑")
            d = posesets.load_set(str(body["name"]))
            modes = list(body.get("modes") or ["baseline", "payload"])
            pname = body.get("payload")
            payload = None
            if pname:
                payload = iden.load_profile(str(pname))
            elif any(m != "baseline" for m in modes):
                payload = vali.snapshot_payload(loop)
            def done(r: Dict[str, Any]) -> None:
                with self._id_lock:
                    self._id["running"] = False
                    self._id["session"] = r.get("session")
                    self._id["error"] = None if r.get("ok") else r.get("error")
                    self._id["rows"] = r.get("rows") or []
                    self._id["summary"] = r.get("summary") or {}
                    self._id["band"] = r.get("friction_band") or {}
                    self._id["log"].append(
                        f"{time.strftime('%H:%M:%S')} "
                        + ("对比完成 " + str(r.get("session")) if r.get("ok") else str(r.get("error")))
                    )
            task = vali.CompareTask(
                loop, d, urdf, base, self._id_log, done, still=self.still,
                modes=modes, payload=payload,
                correction_iters=int(body.get("correction_iters") or 2),
            )
            with self._id_lock:
                self._id.update(running=True, error=None, session=task.session, kind="validate",
                                rows=[], summary={}, band={})
                self._id_task = task
            threading.Thread(target=task.run, name="validate", daemon=True).start()
            return {"ok": True, "identify": self.identify_status()}
        raise FileNotFoundError(path)

    def _urdf(self):
        return self.settings.urdf, self.settings.base_link

    def handle_poseset_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        urdf, base = self._urdf()
        if path == "/api/posesets/create":
            d = posesets.create_set(str(body["name"]), str(body.get("arm") or "right"), str(body.get("note") or ""))
            return {"ok": True, "set": d, "list": posesets.list_sets()}
        if path == "/api/posesets/delete":
            posesets.delete_set(str(body["name"]))
            return {"ok": True, "list": posesets.list_sets()}
        if path == "/api/posesets/record":
            name = str(body["name"])
            d = posesets.load_set(name)
            st = self.reader().read_arm(d["arm"])
            r = posesets.add_pose(name, st.q, urdf, base, body.get("pose_name"), "teach")
            return {"ok": True, "set": r["set"], "pose": r["pose"]}
        if path == "/api/posesets/delete-pose":
            d = posesets.delete_pose(str(body["name"]), int(body["index"]), urdf, base)
            return {"ok": True, "set": d}
        if path == "/api/posesets/mirror":
            d = posesets.mirror_set(
                str(body["src"]),
                urdf,
                base,
                name=body.get("name"),
                force=bool(body.get("force", False)),
            )
            return {"ok": True, "set": d, "list": posesets.list_sets()}
        if path == "/api/posesets/goto":
            loop = self.require_loop()
            d = posesets.load_set(str(body["name"]))
            i = int(body["index"])
            if not (0 <= i < len(d["poses"])):
                raise IndexError("姿态下标越界")
            if loop.arm != d["arm"]:
                raise PermissionErrorMotion(f"当前控制臂是{loop.arm}，姿态集是{d['arm']}，请先释放并切换")
            loop.set_target(d["poses"][i]["q"])
            return {"ok": True, "ctrl": self.ctrl_status()}
        raise FileNotFoundError(path)

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
        return {
            "ok": err is None,
            "error": err,
            "mode": "real" if self.allow_real_motion else "readonly",
            "robot": self.settings.robot,
            "iface": self.settings.network_interface,
            "topic": self.settings.state_topic,
            "command_topic": self.settings.command_topic,
            "ctrl": self.ctrl_status(),
            "identify": self.identify_status(),
            "arms": arms,
        }

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
                _file(self, FRONTEND / "index.html", "text/html; charset=utf-8")
                return
            if path == "/static/style.css":
                _file(self, FRONTEND / "style.css", "text/css; charset=utf-8")
                return
            if path == "/static/app.js":
                _file(self, FRONTEND / "app.js", "application/javascript; charset=utf-8")
                return
            if path == "/api/status":
                q = parse_qs(parsed.query)
                arm = (q.get("arm") or ["right"])[0]
                _json(self, 200, APP.snapshot(arm))
                return
            if path == "/api/scene":
                _json(self, 200, APP.both_scenes())
                return
            if path == "/api/posesets":
                q = parse_qs(parsed.query)
                name = (q.get("name") or [None])[0]
                if name:
                    _json(self, 200, {"ok": True, "set": posesets.load_set(name)})
                else:
                    arm = (q.get("arm") or [None])[0]
                    _json(self, 200, {"ok": True, "list": posesets.list_sets(arm)})
                return
            if path == "/api/identify":
                _json(self, 200, {"ok": True, "identify": APP.identify_status()})
                return
            if path == "/api/validate/band":
                q = parse_qs(parsed.query)
                arm = (q.get("arm") or ["right"])[0]
                ps = (q.get("poseset") or [None])[0]
                kp = None
                if APP.loop is not None:
                    kp = APP.loop.status().get("kp_vec")
                band = friction.table_for(arm, ps, kp)
                _json(self, 200, {"ok": True, "band": band.to_dict()})
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
            if path.startswith("/api/posesets"):
                _json(self, 200, APP.handle_poseset_post(path, body))
                return
            if path.startswith("/api/identify"):
                _json(self, 200, APP.handle_identify_post(path, body))
                return
            if path.startswith("/api/validate"):
                _json(self, 200, APP.handle_validate_post(path, body))
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
            elif path == "/api/control-arm":
                loop.set_arm(str(body["arm"]))
                APP._restore_payload(loop)
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
        if APP.loop:
            APP.loop.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
