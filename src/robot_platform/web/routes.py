"""HTTP 路由：GET（页面、静态文件、URDF、状态）与 POST（按功能分组，按顺序尝试）。

POST 的顺序有讲究，与拆分前一致：
1. /api/runtime-config/* 先走（它自己管锁定，不受锁定拦截）；
2. 配置锁定检查 runtime.guard_path；
3. 手眼、负载、雷达三组（它们自带路由表）；
4. TCP、工作姿态、棋盘、识别定位、空间点、立柱；
5. 其余都是臂控指令，要求开了 --allow-real-motion。

每个分组函数返回 (状态码, JSON) 表示已处理，返回 None 表示不归它管。
"""

from __future__ import annotations

import math
import traceback
from typing import Any, Callable, Dict, List, Optional, Tuple
from urllib.parse import ParseResult, parse_qs

from robot_platform.control.arm_loop import PermissionErrorMotion
from robot_platform.core.runtime_config import ConfigLockedError
from robot_platform.paths import STATIC_DIR
from robot_platform.web.http_util import safe_under, send_bytes, send_file, send_json
from robot_platform.web.page import page_html

Reply = Tuple[int, Dict[str, Any]]
NOT_FOUND: Reply = (404, {})  # 交给 send_error(404)

STATIC_FILES = {
    "/static/style.css": ("style.css", "text/css; charset=utf-8"),
    "/static/viewer.js": ("viewer.js", "text/javascript; charset=utf-8"),
}
APP_JS_DIR = STATIC_DIR / "js"


def app_js_bundle() -> bytes:
    """前端主脚本：static/js/ 下各文件按文件名顺序拼成一个脚本（共用一个作用域，函数互相可见）。"""
    parts = sorted(APP_JS_DIR.glob("*.js"))
    return "\n".join(f.read_text(encoding="utf-8") for f in parts).encode("utf-8")


# ---------------------------------------------------------------- GET

def handle_get(app, handler, parsed: ParseResult) -> None:
    path = parsed.path
    if path in ("/", "/index.html"):
        send_bytes(handler, 200, page_html().encode("utf-8"), "text/html; charset=utf-8")
        return
    if path in ("/rail-preview", "/rail-preview.html"):
        send_file(handler, STATIC_DIR / "rail_preview.html", "text/html; charset=utf-8")
        return
    if path == "/static/app.js":
        send_bytes(handler, 200, app_js_bundle(), "application/javascript; charset=utf-8")
        return
    if path in STATIC_FILES:
        name, ctype = STATIC_FILES[path]
        send_file(handler, STATIC_DIR / name, ctype)
        return
    if path.startswith("/static/vendor/"):
        vendor = safe_under(STATIC_DIR / "vendor", path.split("/")[-1])
        if vendor is None:
            handler.send_error(404)
            return
        send_file(handler, vendor, "text/javascript; charset=utf-8")
        return
    if path in ("/urdf/robot.urdf", f"/urdf/{app.profile.urdf.name}"):
        send_file(handler, app.profile.urdf, "application/xml")
        return
    if path.startswith("/urdf/meshes/"):
        mesh = safe_under(app.profile.meshes_dir, path.split("/")[-1])
        if mesh is None:
            handler.send_error(404)
            return
        send_file(handler, mesh, "application/octet-stream")
        return
    if path == "/api/robot":
        send_json(handler, 200, app.robot_caps())
        return
    if path == "/api/sensors":
        from robot_platform.perception.sensors import sensor_list

        send_json(handler, 200, {"ok": True, "sensors": sensor_list(app.profile)})
        return
    if path == "/api/runtime-config":
        send_json(handler, 200, app.runtime.snapshot())
        return
    if path == "/api/status":
        q = parse_qs(parsed.query)
        arm = (q.get("arm") or ["right"])[0]
        send_json(handler, 200, app.snapshot(arm))
        return
    if path == "/api/scene":
        send_json(handler, 200, app.both_scenes())
        return
    if path == "/api/handeye/preview.jpg":
        jpeg, err = app.handeye.preview_jpeg()
        if not jpeg:
            send_json(handler, 503, {"ok": False, "error": err or "没有画面"})
            return
        send_bytes(handler, 200, jpeg, "image/jpeg")
        return
    for sub in (app.handeye, app.payload, app.lidar):
        got = sub.handle_get(path, parsed.query)
        if got is not None:
            send_json(handler, 200, got)
            return
    handler.send_error(404)


# ---------------------------------------------------------------- POST 分组

def _runtime(app, path: str, body: Dict[str, Any]) -> Optional[Reply]:
    if path.startswith("/api/runtime-config"):
        return 200, app.runtime_post(path, body)
    return None


def _locked(app, path: str, body: Dict[str, Any]) -> Optional[Reply]:
    try:
        app.runtime.guard_path(path, body)
    except ConfigLockedError as exc:
        return 200, {"ok": False, "error": str(exc), "runtime": app.runtime.snapshot()}
    return None


def _handeye_payload(app, path: str, body: Dict[str, Any]) -> Optional[Reply]:
    posted = app.handeye.handle_post(path, body)
    if posted is None:
        posted = app.payload.handle_post(path, body)
    if posted is None:
        posted = app.lidar.handle_post(path, body)
    if posted is None:
        return None
    if isinstance(posted, dict):
        posted.setdefault("runtime", app.runtime.snapshot())
    return 200, posted


def _tcp(app, path: str, body: Dict[str, Any]) -> Optional[Reply]:
    if not path.startswith("/api/tcp/"):
        return None
    out = app.tcp_post(path, body)
    if isinstance(out, dict):
        out.setdefault("runtime", app.runtime.snapshot())
        out.setdefault("work_pose", app.work_pose.status(app.tcp.active_name or ""))
    return 200, out


def _work_pose(app, path: str, body: Dict[str, Any]) -> Optional[Reply]:
    if not path.startswith("/api/work-pose/"):
        return None
    out = app.work_pose_post(path, body)
    if isinstance(out, dict):
        out.setdefault("runtime", app.runtime.snapshot())
    return 200, out


def _board(app, path: str, body: Dict[str, Any]) -> Optional[Reply]:
    if not path.startswith("/api/board/"):
        return None
    if path == "/api/board/look":
        return 200, app.board.look(str(body.get("board") or "9x6"), bool(body.get("ideal")))
    if path == "/api/board/start":
        return 200, app.board.start(
            float(body.get("above_cm") or 5.0) / 100.0,
            float(body.get("down_cm") or 4.0) / 100.0,
            float(body.get("up_cm") or 4.0) / 100.0,
            app.body_arm(body),
        )
    if path == "/api/board/stop":
        app.board.request_stop()
        # 只停摸棋盘的那只手，另一只手不受影响
        if app.loop is not None and app.loop.status().get("engaged") and app.board.arm:
            app.loop.hold_arm(app.board.arm)
        return 200, app.board.status()
    return 200, {"ok": False, "error": "没有这个棋盘接口"}


def _detect(app, path: str, body: Dict[str, Any]) -> Optional[Reply]:
    # /api/detect/grab 归手眼（取图），已在前面处理
    if not path.startswith("/api/detect/") or path == "/api/detect/grab":
        return None
    actions = {
        "/api/detect/points": lambda: app.detect.acquire_points(body),
        "/api/detect/calibrate": app.detect.calibrate,
        "/api/detect/center": app.detect.extract_center,
        "/api/detect/clear": app.detect.clear,
    }
    fn = actions.get(path)
    return 200, (fn() if fn else {"ok": False, "error": "没有这个识别定位接口"})


def _points(app, path: str, body: Dict[str, Any]) -> Optional[Reply]:
    if not path.startswith("/api/points/"):
        return None
    if path == "/api/points/goto":
        return 200, app.point_goto(body, app.require_loop())
    if path == "/api/points/goto-xyz":
        return 200, app.point_goto_xyz(body, app.require_loop())
    return 200, app.point_post(path, body)


def _lift(app, path: str, body: Dict[str, Any]) -> Optional[Reply]:
    if not path.startswith("/api/lift/"):
        return None
    return 200, app.lift_post(path, body)


def _chassis(app, path: str, body: Dict[str, Any]) -> Optional[Reply]:
    if not path.startswith("/api/chassis/"):
        return None
    return 200, app.chassis_post(path, body)


# ---------------------------------------------------------------- 臂控指令

def _waist_step(app, loop, body: Dict[str, Any]) -> None:
    """相对当前目标（或实测）步进；与立柱 step 同思路。"""
    spec = app.profile.waist
    if spec is None:
        raise PermissionErrorMotion(f"{app.profile.name} 没有可控的腰")
    st_w = app.reader().read_motor_states(spec.read_motors)
    tgt = (app.ctrl_status() or {}).get("waist_target_rad") or {}
    new = {}
    for role in ("yaw", "pitch"):
        d = body.get(f"delta_{role}_deg")
        if d in (None, ""):
            continue  # 只动给了步长的轴（G1D 没有俯仰轴）
        if not spec.has(role):
            raise ValueError(f"{app.profile.name} 的腰没有{'俯仰' if role == 'pitch' else role}轴")
        m = spec.joint(role).motor
        new[role] = float(tgt.get(str(m), st_w[m]["q"])) + math.radians(float(d))
    loop.set_waist(new.get("yaw"), new.get("pitch"))


def _hold(app, loop, body: Dict[str, Any]) -> None:
    """急停：先叫停所有会连续发目标的任务，再把手臂 / 腰锁在当前实测角。

    任务只是被通知停止，线程要等下一次检查才退出；所以先置停止标志、最后才锁目标，
    避免任务在锁目标之后又发出下一个点。各项互不影响：某项出错也继续往下停。
    """
    app.motion_stop.set()  # 空间点碰点、工作姿态调用
    stoppers = (
        lambda: app.chassis_stop("急停"),  # 底盘最先停（它不受手臂接管影响）
        app.board.request_stop,          # 棋盘触摸
        app.payload.stop_tasks,          # 负载测量、到位对比
        app.handeye.stop_capture,        # 手眼采集
        lambda: app.lift_post("/api/lift/stop", {}),
    )
    for stop in stoppers:
        try:
            stop()
        except Exception:
            traceback.print_exc()
    loop.estop_hold()


def _teach(app, loop, body: Dict[str, Any]) -> None:
    """拖拽：body.arm 指定哪只手（不给 = 两只手）；另一只手照常受控。"""
    arm = str(body.get("arm") or "")
    if bool(body.get("enabled", True)):
        app.unlock_work_pose("teach", arm)
        loop.enable_teach(arm or None)
    else:
        loop.disable_teach(arm or None)


def _speed(app, loop, body: Dict[str, Any]) -> None:
    loop.set_max_speed(float(body["max_speed_rad_s"]))
    app.settings.max_speed_rad_s = loop.max_speed


def _target(app, loop, body: Dict[str, Any]) -> None:
    arm = app.body_arm(body)
    if not app._work_pose_motion:
        app.unlock_work_pose("arm_control", arm)
    loop.set_target(body["q"], arm=arm)


def _arm_stop(app, loop, body: Dict[str, Any]) -> None:
    """只停一只手（取消它没走完的目标）；占用它的任务不受影响，要停任务用急停。"""
    loop.hold_arm(app.body_arm(body))


def _waist(app, loop, body: Dict[str, Any]) -> None:
    yaw = body.get("yaw_deg")
    pitch = body.get("pitch_deg")
    loop.set_waist(
        None if yaw in (None, "") else math.radians(float(yaw)),
        None if pitch in (None, "") else math.radians(float(pitch)),
    )


ARM_COMMANDS: Dict[str, Callable[[Any, Any, Dict[str, Any]], None]] = {
    "/api/engage": lambda app, loop, body: loop.engage(),
    "/api/release": lambda app, loop, body: loop.release(),
    "/api/hold": _hold,
    "/api/teach": _teach,
    "/api/gravity": lambda app, loop, body: loop.set_gravity_ff(bool(body.get("enabled", True))),
    "/api/gravity-imu": lambda app, loop, body: loop.set_gravity_imu(bool(body.get("enabled", True))),
    "/api/speed": _speed,
    "/api/target": _target,
    "/api/waist": _waist,
    "/api/waist/step": _waist_step,
    "/api/waist/stop": lambda app, loop, body: loop.hold_waist(),
    "/api/arm/stop": _arm_stop,
}


def _arm(app, path: str, body: Dict[str, Any]) -> Reply:
    loop = app.require_loop()
    fn = ARM_COMMANDS.get(path)
    if fn is None:
        return NOT_FOUND
    fn(app, loop, body)
    return 200, {
        "ok": True,
        "ctrl": app.ctrl_status(),
        "work_pose": app.work_pose.status(app.tcp.active_name or ""),
    }


def _mock(app, path: str, body: Dict[str, Any]) -> Optional[Reply]:
    """仿真专用：模拟另一个程序往指令话题发令（测试占用检测）。真机后端没有这个接口。"""
    if path != "/api/mock/foreign":
        return None
    sim = getattr(app.backend, "simulate_foreign", None)
    if sim is None:
        return NOT_FOUND
    hz = float(body.get("hz") or 0.0)
    sim(hz)
    return 200, {"ok": True, "hz": hz}


POST_GROUPS: List[Callable[[Any, str, Dict[str, Any]], Optional[Reply]]] = [
    _mock, _runtime, _locked, _handeye_payload, _tcp, _work_pose, _board, _detect, _points, _lift, _chassis,
]


def handle_post(app, path: str, body: Dict[str, Any]) -> Reply:
    for group in POST_GROUPS:
        reply = group(app, path, body)
        if reply is not None:
            return reply
    return _arm(app, path, body)
