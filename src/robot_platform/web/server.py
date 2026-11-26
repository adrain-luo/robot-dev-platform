#!/usr/bin/env python3
"""网页服务入口：解析参数、选实例与后端、组装 App、起 HTTP 服务；收尾时先释放手臂再退出。

    python serve.py                                     # 只读
    python serve.py --allow-real-motion --max-speed 0.2  # 真机
    python serve.py --backend mock                      # 仿真

路由见 routes.py，各功能接口见 api/，App 本体见 app.py。
"""

from __future__ import annotations

import argparse
import atexit
import os
import signal
import sys
import threading
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional
from urllib.parse import urlparse

from g1_contracts.motion import MotionRejected
from robot_platform.control.arm_loop import PermissionErrorMotion
from robot_platform.control.fk_scene import RobotKinematics
from robot_platform.core import instance as instances
from robot_platform.core.config_loader import load_settings
from robot_platform.core.robot_profile import activate as activate_profile
from robot_platform.hw.backends import BACKENDS, create_backend
from robot_platform.web import routes
from robot_platform.web.app import MOTION_CORE_MODES, App
from robot_platform.web.http_util import read_json, send_json

APP: Optional[App] = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args) -> None:
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))

    def do_GET(self) -> None:  # noqa: N802
        assert APP is not None
        try:
            routes.handle_get(APP, self, urlparse(self.path))
        except Exception as e:
            traceback.print_exc()
            send_json(self, 500, {"ok": False, "error": f"{type(e).__name__}: {e}"})

    def do_POST(self) -> None:  # noqa: N802
        assert APP is not None
        path = urlparse(self.path).path
        try:
            body = read_json(self)
            reply = routes.handle_post(APP, path, body)
            if reply is routes.NOT_FOUND:
                self.send_error(404)
                return
            send_json(self, *reply)
        except (PermissionErrorMotion, MotionRejected) as e:
            send_json(self, 403, {"ok": False, "error": str(e)})
        except Exception as e:
            traceback.print_exc()
            send_json(self, 500, {"ok": False, "error": f"{type(e).__name__}: {e}"})


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
    p.add_argument("--motion-core", default="inprocess", choices=MOTION_CORE_MODES,
                   help="inprocess（默认）：本进程建控制循环、是指令写入者；ipc：连单独的运控核心进程"
                        "（python -m g1_motion_core.service），本进程不写指令，崩溃时手臂由核心保持")
    p.add_argument("--core-dir", default=None, help="ipc 时运控核心的套接字目录，默认 $G1_MOTION_DIR 或 /run/g1-motion")
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
    APP = App(settings, kin, allow_real_motion=allow_motion, backend=backend,
              motion_core=args.motion_core, core_dir=args.core_dir)
    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    mode = "REAL MOTION" if args.allow_real_motion else "readonly"
    if backend.name == "mock":
        mode = "MOCK"
    print(
        f"[arm_control] {settings.robot} {mode}  instance={inst.name}  http://{args.host}:{args.port}/  "
        f"iface={settings.network_interface}  max_speed={settings.max_speed_rad_s}  motion_core={args.motion_core}",
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
        try:
            APP.chassis_stop("服务退出")  # 底盘先停：停发后它会不会自己停没确认
        except Exception:  # noqa: BLE001
            pass
        if APP.handeye:
            APP.handeye.close()
        if APP.lidar:
            APP.lidar.close()
        APP.shutdown_motion()
        backend.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
