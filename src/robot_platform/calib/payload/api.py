"""末端负载、姿态集、到位对比的接口。走点任务经运控核心的租约发令（control/motion_core.py）。"""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Dict, Optional
from urllib.parse import parse_qs

from robot_platform.calib.payload import friction
from robot_platform.calib.payload import identify as iden
from robot_platform.calib.payload import poses as posesets
from robot_platform.calib.payload import validate as vali


class PayloadAPI:
    def __init__(self, app) -> None:
        self.app = app
        self._id_lock = threading.Lock()
        self._id_task = None
        self._id: Dict[str, Any] = {
            "running": False, "phase": "", "log": [], "session": None,
            "solution": None, "error": None, "progress": [0, 0],
            "kind": None, "rows": [], "summary": {}, "band": {},
        }
        self.still = dict(iden.DEFAULT_STILL)

    def _core(self):
        """运控核心；只读模式（没开 --allow-real-motion）为 None。"""
        return getattr(self.app, "motion_core", None)

    def restore(self, motion) -> None:
        """两只手各自带上最后应用的负载补偿（motion：运控核心）。"""
        iden.restore_payloads(motion)

    def apply_payload_dict(self, d: Dict[str, Any], *, name: Optional[str] = None,
                           arm: Optional[str] = None) -> Dict[str, Any]:
        """写入该臂的「最后应用」快照，并立刻进这只手的前馈（两只手都在控制循环里）。"""
        motion = self.app.motion()
        arm = str(d.get("arm") or arm or "")
        if arm not in motion.arm_names:
            raise ValueError("这套补偿没有记是哪只手的")
        theta = d.get("theta")
        if not theta:
            theta = iden.payload_to_theta(d["mass_kg"], d.get("com_m") or [0, 0, 0])
        name = name or d.get("name")
        saved = {
            "arm": arm, "alpha": 1.0, "theta": list(theta),
            "mass_kg": d.get("mass_kg"), "com_m": d.get("com_m"),
            "session": d.get("session"), "poseset": d.get("poseset"),
            "name": name, "note": d.get("note", ""),
            "applied_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }
        if saved["mass_kg"] is None:
            saved["mass_kg"], saved["com_m"] = iden.theta_to_payload(theta)
        iden.save_payload(arm, saved)
        motion.set_payload_theta(theta, name, arm=arm)
        saved["live"] = True
        return saved

    def stop_tasks(self) -> None:
        """「停止」按钮：叫停正在跑的负载测量 / 到位对比（下一次检查时抛「已停止」，不再发新目标）。
        急停不用它：急停撤销租约，任务自己就停了。"""
        with self._id_lock:
            task = self._id_task
        if task is not None:
            task.request_stop()

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
        kind = self._id.get("kind")
        if sess and kind != "validate":
            sdir = iden.measure_dir() / str(sess)
            st["session_dir"] = str(sdir) if sdir.is_dir() else None
        else:
            st["session_dir"] = None
        arm = (st.get("solution") or {}).get("arm")
        if arm:
            pp = iden.payload_path(arm)
            st["payload_file"] = str(pp) if pp.exists() else None
        else:
            st["payload_file"] = None
        st["profiles"] = iden.list_profiles()
        st["sessions"] = iden.list_sessions()
        st["applied_left"] = iden.applied_name_for_arm("left")
        st["applied_right"] = iden.applied_name_for_arm("right")
        # 两只手各有一套补偿：页面按姿态集的手取 applied_left / applied_right
        core = self._core()
        if core is not None:
            for arm_name in core.arm_names:
                live = core.arm_status(arm_name).get("payload_name")
                if live:
                    st[f"applied_{arm_name}"] = live
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

    def _log(self, msg: str) -> None:
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        with self._id_lock:
            self._id["log"].append(line)
            self._id["phase"] = msg

    def handle_get(self, path: str, query: str) -> Optional[Dict[str, Any]]:
        q = parse_qs(query)
        if path == "/api/posesets":
            name = (q.get("name") or [None])[0]
            if name:
                return {"ok": True, "set": posesets.load_set(name)}
            arm = (q.get("arm") or [None])[0]
            return {"ok": True, "list": posesets.list_sets(arm)}
        if path == "/api/identify":
            return {"ok": True, "identify": self.identify_status()}
        if path == "/api/validate/band":
            arm = (q.get("arm") or ["right"])[0]
            ps = (q.get("poseset") or [None])[0]
            kp = None
            core = self._core()
            if core is not None and arm in core.arm_names:
                kp = core.arm_status(arm).get("kp_vec")
            band = friction.table_for(arm, ps, kp)
            return {"ok": True, "band": band.to_dict()}
        if path == "/api/validate/list":
            arm = (q.get("arm") or [None])[0]
            ps = (q.get("poseset") or [None])[0]
            return {"ok": True, "list": vali.list_results(arm, ps)}
        if path == "/api/validate/result":
            name = (q.get("name") or [""])[0]
            return {"ok": True, "result": vali.load_result(name)}
        return None

    def handle_post(self, path: str, body: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        if path.startswith("/api/posesets"):
            return self._poseset_post(path, body)
        if path.startswith("/api/identify"):
            return self._identify_post(path, body)
        if path.startswith("/api/validate"):
            return self._validate_post(path, body)
        return None

    def _poseset_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        urdf, base = self.app._urdf()
        if path == "/api/posesets/create":
            d = posesets.create_set(str(body["name"]), str(body.get("arm") or "right"), str(body.get("note") or ""))
            return {"ok": True, "set": d, "list": posesets.list_sets()}
        if path == "/api/posesets/delete":
            posesets.delete_set(str(body["name"]))
            return {"ok": True, "list": posesets.list_sets()}
        if path == "/api/posesets/record":
            name = str(body["name"])
            d = posesets.load_set(name)
            st = self.app.reader().read_arm(d["arm"])
            r = posesets.add_pose(name, st.q, urdf, base, body.get("pose_name"), "teach")
            return {"ok": True, "set": r["set"], "pose": r["pose"]}
        if path == "/api/posesets/delete-pose":
            d = posesets.delete_pose(str(body["name"]), int(body["index"]), urdf, base)
            return {"ok": True, "set": d}
        if path == "/api/posesets/mirror":
            d = posesets.mirror_set(
                str(body["src"]), urdf, base,
                name=body.get("name"), force=bool(body.get("force", False)),
            )
            return {"ok": True, "set": d, "list": posesets.list_sets()}
        if path == "/api/posesets/goto":
            motion = self.app.motion()
            d = posesets.load_set(str(body["name"]))
            i = int(body["index"])
            if not (0 <= i < len(d["poses"])):
                raise IndexError("姿态下标越界")
            plan = motion.manual_target(d["poses"][i]["q"], arm=d["arm"], detour=True)
            out = {"ok": True, "ctrl": self.app.ctrl_status()}
            if plan:
                out["detour"] = plan
                out["message"] = (f"直线会碰，已绕行：{plan.get('waypoints', 0)} 个中间点"
                                  f"（规划 {plan.get('ms', 0) / 1000:.1f} s）")
            return out
        raise FileNotFoundError(path)

    def _identify_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        urdf, base = self.app._urdf()
        if path == "/api/identify/still":
            for k in iden.DEFAULT_STILL:
                if body.get(k) is not None:
                    self.still[k] = float(body[k])
            return {"ok": True, "identify": self.identify_status()}
        if path == "/api/identify/stop":
            self.stop_tasks()
            return {"ok": True, "identify": self.identify_status()}
        if path == "/api/identify/measure":
            motion = self.app.motion()
            with self._id_lock:
                if self._id["running"]:
                    raise RuntimeError("已有任务在跑")
            d = posesets.load_set(str(body["name"]))
            he = getattr(self.app, "handeye", None)
            if he is not None and he._running:
                raise RuntimeError("手眼采集正在走点")

            def done(r: Dict[str, Any]) -> None:
                with self._id_lock:
                    self._id["running"] = False
                    self._id["session"] = r.get("session")
                    self._id["error"] = None if r.get("ok") else r.get("error")
                    self._id["log"].append(
                        f"{time.strftime('%H:%M:%S')} "
                        + ("测量完成 " + str(r.get("session")) if r.get("ok") else str(r.get("error")))
                    )

            # 租约：占住姿态集那只手直到任务结束（没接管、在拖拽、被占用、急停锁定时这里就拒绝）
            lease = motion.lease_arm(d["arm"], "负载测量")
            try:
                task = iden.MeasureTask(lease, d, urdf, base, self._log, done, still=self.still)
            except BaseException:
                lease.release()
                raise
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
            sdir = iden.measure_dir() / str(session)
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
            saved = self.apply_payload_dict(cur, name=name)
            iden.save_profile(name, saved)
            return {"ok": True, "payload": saved, "identify": self.identify_status(), "ctrl": self.app.ctrl_status()}
        if path == "/api/identify/clear":
            motion = self.app.motion()
            arm = str(body.get("arm") or "")
            if arm not in motion.arm_names:
                raise ValueError("说明去掉哪只手的负载项（先选姿态集）")
            motion.set_payload_theta(None, arm=arm)
            iden.clear_payload(arm)
            return {"ok": True, "identify": self.identify_status(), "ctrl": self.app.ctrl_status()}
        if path == "/api/identify/payload/apply":
            d = iden.load_profile(str(body["name"]))
            saved = self.apply_payload_dict(d, name=d["name"])
            side = "左" if saved.get("arm") == "left" else "右"
            msg = f"已应用「{saved.get('name')}」为{side}臂补偿"
            return {
                "ok": True,
                "payload": saved,
                "identify": self.identify_status(),
                "ctrl": self.app.ctrl_status(),
                "message": msg,
            }
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
            name = str(body["name"])
            core = self._core()
            for arm in ("left", "right"):
                if iden.applied_name_for_arm(arm) == name:
                    iden.clear_payload(arm)
                    if core is not None and arm in core.arm_names:
                        core.set_payload_theta(None, arm=arm)
            iden.delete_profile(name)
            return {
                "ok": True,
                "identify": self.identify_status(),
                "ctrl": self.app.ctrl_status() if core is not None else None,
                "message": f"已删除「{name}」",
            }
        raise FileNotFoundError(path)

    def _validate_post(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        urdf, base = self.app._urdf()
        if path == "/api/validate/start":
            motion = self.app.motion()
            with self._id_lock:
                if self._id["running"]:
                    raise RuntimeError("已有任务在跑")
            d = posesets.load_set(str(body["name"]))
            he = getattr(self.app, "handeye", None)
            if he is not None and he._running:
                raise RuntimeError("手眼采集正在走点")
            modes = list(body.get("modes") or ["baseline", "payload"])
            pname = body.get("payload")
            payload = iden.load_profile(str(pname)) if pname else None

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

            # 租约：占住姿态集那只手直到任务结束（没接管、在拖拽、被占用、急停锁定时这里就拒绝）
            lease = motion.lease_arm(d["arm"], "到位对比")
            try:
                if payload is None and any(m != "baseline" for m in modes):
                    payload = vali.snapshot_payload(lease)
                task = vali.CompareTask(
                    lease, d, urdf, base, self._log, done, still=self.still,
                    modes=modes, payload=payload,
                    correction_iters=int(body.get("correction_iters") or 2),
                )
            except BaseException:
                lease.release()
                raise
            with self._id_lock:
                self._id.update(running=True, error=None, session=task.session, kind="validate",
                                rows=[], summary={}, band={})
                self._id_task = task
            threading.Thread(target=task.run, name="validate", daemon=True).start()
            return {"ok": True, "identify": self.identify_status()}
        raise FileNotFoundError(path)
