"""运控核心进程入口（阶段 2a）。

    python -m g1_motion_core.service --backend mock --socket-dir /tmp/g1-motion   # 仿真
    python -m g1_motion_core.service --allow-real-motion                          # 真机

组装的东西和网页 ``--motion-core inprocess`` 时完全一样（robot_platform/control/assembly.py）：
后端、ArmLoop、MotionCore（守护层后端时急停联动守护层）、碰撞闸门、最后应用的负载补偿。
再把网页要的平台调用（页面状态、手动走点、拖拽、腰、限速、负载、碰撞检查、lowstate）注册给 CoreServer。

- 直写后端（unitree_lowcmd / unitree_arm_sdk）时本进程就是 rt/lowcmd（rt/arm_sdk）的唯一写入者：
  启动前要停掉别的写入者，``--allow-real-motion`` 必须显式给出（同 serve.py）。
- 守护层后端时本进程是守护层的唯一客户端，不写 lowcmd。
- 网页、任务进程崩溃或心跳超时：它们的租约撤销、它们手动动着的手停住，本进程照常发帧。
- 本进程退出（SIGTERM / Ctrl-C）：先停服务（各连接的租约撤销），再像原来网页退出那样释放手臂、停控制循环。
  直写后端时释放就是停发：G1D 手臂会掉，退出前手臂要有支撑。

上机前看 docs/阶段2a_运控核心进程.md 的操作卡。
"""

from __future__ import annotations

import argparse
import atexit
import math
import os
import signal
import sys
import threading
import time
import traceback
from typing import Any, Callable, Dict, Optional

from g1_motion_core.server import CallContext, CoreServer, ServerStartError

DIRECT_BACKENDS = ("unitree_lowcmd", "unitree_arm_sdk")


def jsonable(x: Any) -> Any:
    """转成线上能发的 JSON：numpy 标量 / 数组、元组、非有限浮点（→ None）。"""
    if x is None or isinstance(x, (bool, str)):
        return x
    if isinstance(x, int):
        return int(x)
    if isinstance(x, float):
        return x if math.isfinite(x) else None
    if isinstance(x, dict):
        return {str(k): jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [jsonable(v) for v in x]
    if hasattr(x, "tolist"):  # numpy 数组和标量
        return jsonable(x.tolist())
    if hasattr(x, "item"):
        return jsonable(x.item())
    return str(x)


class CoreProcess:
    """一台机器的运控核心：后端 + 控制循环 + 运控核心 + 碰撞闸门，加给网页的平台调用。"""

    def __init__(self, settings, kin, backend, *, log: Callable[[str], None] = print) -> None:
        from robot_platform.calib.payload import identify as iden
        from robot_platform.collision import setup as collision_setup
        from robot_platform.control.assembly import build_motion_core

        self.settings = settings
        self.profile = settings.profile
        self.backend = backend
        self.log = log
        self.started = time.monotonic()
        self.reader = backend.state()
        self.cmd_monitor = None
        try:
            self.cmd_monitor = backend.cmd_monitor()
        except Exception:  # noqa: BLE001  检测起不来不影响发令；页面就不显示指令话题占用
            traceback.print_exc()
        self.loop, self.core = build_motion_core(settings, kin, backend, self.reader)
        try:
            iden.restore_payloads(self.core)
        except Exception:  # noqa: BLE001  补偿文件坏了：不带补偿运行，页面能看到
            traceback.print_exc()
        self._lift_reader = None
        try:
            self._lift_reader = backend.lift_reader()
        except Exception:  # noqa: BLE001  没有立柱 / 读不到：碰撞检查按立柱在 0 位算
            pass
        self._collision = collision_setup
        self.guard = collision_setup.build_guard(
            self.profile, settings.base_link,
            lambda: collision_setup.body_q(self.profile, self.reader, self._lift_reader),
            lambda: collision_setup.active_tools(self.profile))
        self.core.set_collision_guard(self.guard)
        self.server: Optional[CoreServer] = None

    # ------------------------------------------------------------ 平台调用
    def _arm(self, args: Dict[str, Any]) -> str:
        arm = str(args.get("arm") or "")
        if arm not in self.core.arm_names:
            raise ValueError(f"没有 {arm!r} 臂（{' / '.join(self.core.arm_names)}）")
        return arm

    def _arm_or_both(self, args: Dict[str, Any]) -> Optional[str]:
        return None if args.get("arm") in (None, "", "both") else self._arm(args)

    def info(self) -> Dict[str, Any]:
        arms = {}
        for name in self.core.arm_names:
            lay = self.core.layout(name)
            arms[name] = {"motors": list(lay.motor_indices), "joints": list(lay.joint_names),
                          "limits": jsonable(self.core.limits(name))}
        return {"robot": self.profile.name, "backend": self.backend.name, "host": self.core.host,
                "arms": arms, "external_estop": self.core.has_external_estop, "pid": os.getpid(),
                "collision": self.guard is not None}

    def page_status(self) -> Dict[str, Any]:
        st = self.core.page_status()
        if self.cmd_monitor is not None:
            try:
                st["cmd_bus"] = self.cmd_monitor.report()
            except Exception as exc:  # noqa: BLE001
                st["cmd_bus_error"] = f"{type(exc).__name__}: {exc}"
        st["core_process"] = {"pid": os.getpid(), "uptime_s": round(time.monotonic() - self.started, 1),
                              "backend": self.backend.name,
                              "clients": self.server.clients() if self.server else [],
                              "stats": dict(self.server.stats) if self.server else {}}
        return st

    def lowstate(self) -> Dict[str, Any]:
        r = self.reader
        age = float(r.age_s())
        snap = r.snapshot()
        n = len(snap.q)
        st = r.read_motor_states(list(range(n)))
        return {"age_s": age, "mode_machine": snap.mode_machine,
                "q": [st[i]["q"] for i in range(n)], "dq": [st[i]["dq"] for i in range(n)],
                "tau_est": [st[i]["tau_est"] for i in range(n)], "imu": r.read_imu()}

    def collision(self, path: str, body: Dict[str, Any]) -> Dict[str, Any]:
        g = self.guard

        def status(with_distance: bool) -> Dict[str, Any]:
            arms_q = None
            if with_distance and g is not None:
                try:
                    arms_q = {n: list(self.reader.read_arm(lay).q) for n, lay in self.profile.arms.items()}
                except Exception:  # noqa: BLE001
                    arms_q = None
            return self._collision.guard_status(g, self.profile.name, arms_q,
                                                gate_attached=self.core.collision_guard is g)

        return self._collision.guard_request(g, str(path), dict(body or {}), status)

    def ops(self) -> Dict[str, Callable[[CallContext, Dict[str, Any]], Any]]:
        core = self.core

        def manual_target(ctx: CallContext, a: Dict[str, Any]):
            arm = self._arm(a)
            ctx.touch(arm)
            return core.manual_target(a["q"], arm=arm, detour=bool(a.get("detour")))

        def enable_teach(ctx: CallContext, a: Dict[str, Any]):
            arm = self._arm_or_both(a)
            ctx.touch(*(core.arm_names if arm is None else [arm]))
            core.enable_teach(arm)

        def set_waist(ctx: CallContext, a: Dict[str, Any]):
            ctx.touch("waist")
            core.set_waist(a.get("yaw_rad"), a.get("pitch_rad"))

        table: Dict[str, Callable[[CallContext, Dict[str, Any]], Any]] = {
            # 只读
            "info": lambda ctx, a: self.info(),
            "page_status": lambda ctx, a: self.page_status(),
            "arm_view_status": lambda ctx, a: core.arm_view_status(self._arm(a)),
            "arm_sample": lambda ctx, a: core.arm_sample(self._arm(a), wait=bool(a.get("wait"))),
            "arm_gains": lambda ctx, a: list(core.arm_gains(self._arm(a))),
            "payload_theta": lambda ctx, a: core.payload_theta(self._arm(a)),
            "max_speed": lambda ctx, a: core.max_speed,
            "gravity_up": lambda ctx, a: core.gravity_up(),
            "lowstate": lambda ctx, a: self.lowstate(),
            "clients": lambda ctx, a: self.server.clients() if self.server else [],
            # 设置（不是运动目标）
            "set_payload_theta": lambda ctx, a: core.set_payload_theta(a.get("theta"), a.get("name"), arm=self._arm(a)),
            "set_gravity_ff": lambda ctx, a: core.set_gravity_ff(bool(a.get("on"))),
            "set_gravity_imu": lambda ctx, a: core.set_gravity_imu(bool(a.get("on"))),
            "set_max_speed": lambda ctx, a: core.set_max_speed(float(a["v"])),
            "collision": lambda ctx, a: self.collision(a.get("path") or "", a.get("body") or {}),
            # 手动来源（连接断了就停住它动过的手 / 腰）
            "manual_target": manual_target,
            "enable_teach": enable_teach,
            "disable_teach": lambda ctx, a: core.disable_teach(self._arm_or_both(a)),
            "set_waist": set_waist,
            "hold_waist": lambda ctx, a: core.hold_waist(),
            "hold_arm": lambda ctx, a: core.hold_arm(self._arm(a)),
            # 整机
            "engage": lambda ctx, a: core.engage(),
            "release": lambda ctx, a: core.release(),
            # expect_epoch：网页的复位确认带上（见 robot_platform/web/reset_confirm.py）；客户端不带
            "reset_estop": lambda ctx, a: core.reset_estop(
                None if a.get("expect_epoch") is None else int(a["expect_epoch"])),
            "reset_external_estop": lambda ctx, a: core.reset_external_estop(),
        }
        return {name: _jsonable_result(fn) for name, fn in table.items()}

    # ------------------------------------------------------------ 服务
    def serve(self, directory: str, **kw) -> CoreServer:
        from robot_platform.collision.guard import CollisionBlocked

        def classify(exc: BaseException) -> Optional[str]:
            return "collision" if isinstance(exc, CollisionBlocked) else None

        self.server = CoreServer(self.core, directory, ops=self.ops(), info=lambda: jsonable(self.info()),
                                 classify=classify, log=self.log, **kw)
        return self.server.start()

    def close(self, timeout_s: float = 30.0) -> None:
        """先停服务（各连接租约撤销），再释放手臂、停控制循环（同原来网页退出）。"""
        if self.server is not None:
            self.server.stop()
        try:
            self.core.shutdown(timeout_s)
        finally:
            self.backend.close()


def _jsonable_result(fn):
    def call(ctx, args):
        return jsonable(fn(ctx, args))
    return call


def main(argv: Optional[list] = None) -> int:
    from g1_motion_client.connection import socket_dir
    from robot_platform.control.fk_scene import RobotKinematics
    from robot_platform.core import instance as instances
    from robot_platform.core.config_loader import load_settings
    from robot_platform.core.robot_profile import activate as activate_profile
    from robot_platform.hw.backends import BACKENDS, create_backend

    p = argparse.ArgumentParser(description="G1 / G1D motion core process (phase 2a)")
    p.add_argument("--socket-dir", default=None, help="套接字目录，默认 $G1_MOTION_DIR 或 /run/g1-motion")
    p.add_argument("--instance", default=None, help="机器实例（同 serve.py；网页和本进程必须用同一个）")
    p.add_argument("--robot", default=None)
    p.add_argument("--backend", default=None, choices=BACKENDS)
    p.add_argument("--iface", default=None)
    p.add_argument("--allow-real-motion", action="store_true", help="真机后端必须给：本进程会接管并发令")
    p.add_argument("--max-speed", type=float, default=None, help="rad/s，默认取实例设置（0.2），钳在 0.02–1.0")
    args = p.parse_args(argv)

    backend_name = args.backend or os.environ.get("ARM_BACKEND")
    explicit = args.instance or os.environ.get("ARM_INSTANCE")
    if backend_name == "mock" and not explicit:
        inst = instances.temporary_instance(args.robot or "g1d")
        atexit.register(instances.cleanup, inst)
    else:
        inst = instances.resolve(args.instance)
    instances.activate(inst)
    settings = load_settings(inst, robot=args.robot, backend=args.backend)
    activate_profile(settings.profile)
    if settings.backend != "mock" and not args.allow_real_motion:
        print(f"[g1_motion_core] 后端 {settings.backend} 会接管手臂并发令：要真机运行必须加 --allow-real-motion",
              file=sys.stderr)
        return 2
    if args.iface:
        settings.network_interface = args.iface
    if args.max_speed is not None:
        settings.max_speed_rad_s = float(max(0.02, min(1.0, args.max_speed)))
    if settings.urdf is None or not settings.urdf.is_file():
        print(f"[g1_motion_core] URDF not found: {settings.urdf}", file=sys.stderr)
        return 2
    directory = socket_dir(args.socket_dir)
    kin = RobotKinematics(settings.urdf, settings.base_link, list(settings.profile.arms))
    backend = create_backend(settings.backend, settings)
    proc = CoreProcess(settings, kin, backend)
    try:
        proc.serve(directory)
    except ServerStartError as exc:
        print(f"[g1_motion_core] {exc}", file=sys.stderr)
        proc.core.shutdown()
        backend.close()
        return 3
    print(f"[g1_motion_core] {settings.robot} backend={settings.backend} instance={inst.name} "
          f"socket_dir={directory} max_speed={proc.core.max_speed} pid={os.getpid()}", flush=True)

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    while not stop.wait(0.5):
        pass
    print("[g1_motion_core] stopping: revoking leases, then releasing the arm", flush=True)
    proc.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
