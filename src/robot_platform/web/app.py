"""App：持有设置、后端、运控核心和各功能会话；具体接口方法按功能放在 web/api/ 下。

运控核心两种跑法（--motion-core）：
- inprocess（默认，即阶段 1 的现状）：本进程建控制循环和运控核心，本进程就是指令写入者；
- ipc（阶段 2a）：运控核心在自己的进程里（python -m g1_motion_core.service），本进程不建控制循环、
  不建指令出口、不碰指令话题，只经套接字调它；网页崩溃时手臂由核心保持。
"""

from __future__ import annotations

from typing import Optional, Union
import json
import os
import threading
import traceback

from robot_platform.calib.handeye.service import HandEyeAPI
from robot_platform.calib.payload.api import PayloadAPI
from robot_platform.calib.tcp import TcpSession
from robot_platform.control.arm_loop import ArmLoop, PermissionErrorMotion
from robot_platform.control.assembly import build_motion_core
from robot_platform.control.fk_scene import RobotKinematics
from robot_platform.control.motion_core import MotionCore
from robot_platform.control.remote_core import RemoteMotionCore, RemoteStateSource
from robot_platform.core.runtime_config import RuntimeConfig
from robot_platform.hw.backends.base import Backend, LiftDriver, LiftReader, StateSource
from robot_platform.hw.lift_control import LiftController
from robot_platform.perception.detect_localize import DetectLocalize
from robot_platform.perception.lidar_api import LidarAPI
from robot_platform.tasks import ik_worker
from robot_platform.tasks.board_touch import BoardTouch, BoardTouchEnv
from robot_platform.tasks.spatial_points import SpatialPoints
from robot_platform.tasks.work_pose import WorkPoseSession
from robot_platform.web.api.state import StateMixin
from robot_platform.web.api.lift import LiftMixin
from robot_platform.web.api.chassis import ChassisMixin
from robot_platform.web.api.tcp import TcpMixin
from robot_platform.web.api.work_pose import WorkPoseMixin
from robot_platform.web.api.points import PointsMixin
from robot_platform.web.api.runtime import RuntimeMixin
from robot_platform.web.reset_confirm import ResetConfirm
from robot_platform.web.api.single_pose import SinglePoseMixin
from robot_platform.web.api.suction import SuctionMixin
from robot_platform.web.api.collision import CollisionMixin


MOTION_CORE_MODES = ("inprocess", "ipc")


class App(StateMixin, LiftMixin, ChassisMixin, TcpMixin, WorkPoseMixin, PointsMixin, RuntimeMixin,
          SinglePoseMixin, SuctionMixin, CollisionMixin):
    def __init__(self, settings, kin: RobotKinematics, allow_real_motion: bool, backend: Backend, *,
                 motion_core: str = "inprocess", core_dir: Optional[str] = None):
        if motion_core not in MOTION_CORE_MODES:
            raise ValueError(f"--motion-core 只能是 {' / '.join(MOTION_CORE_MODES)}")
        self.motion_core_mode = motion_core
        self._core_dir = core_dir
        self._remote: Optional[RemoteMotionCore] = None  # ipc：到运控核心进程的连接
        self.settings = settings
        self.profile = settings.profile
        self.kin = kin
        self.allow_real_motion = bool(allow_real_motion)
        self.backend = backend
        self._reader: Optional[StateSource] = None
        self._hispeed: Optional[LiftReader] = None
        self._agv: Optional[LiftDriver] = None
        self._lift_ctrl: Optional[LiftController] = None
        self._chassis_ctrl = None  # 底盘遥控（web/api/chassis.py），第一次遥控时才建
        self.loop: Optional[ArmLoop] = None
        # 运控核心：任务只经它的租约动手臂；接管 / 释放 / 急停也走它（control/motion_core.py；
        # ipc 时是 control/remote_core.py 的 RemoteMotionCore，方法一样）
        self.motion_core: Optional[Union[MotionCore, RemoteMotionCore]] = None
        self.cmd_monitor = None
        # 页面复位（急停 / 守护层）的一次性确认令牌：复位要人在弹窗里确认（web/reset_confirm.py）
        self.reset_confirm = ResetConfirm()
        self._lock = threading.Lock()
        self.payload = PayloadAPI(self)
        self.tcp = TcpSession()
        self.points = SpatialPoints()
        self.work_pose = WorkPoseSession()
        # 任务不拿 App：注入运控客户端、只读运动学和它要的几样能力
        self.board = BoardTouch(BoardTouchEnv(
            motion=self.motion,
            kin=kin,
            stereo=self._board_stereo,
            tcp_for_arm=self._tcp_for_arm,
            work_pose_lock=self.work_pose_lock_for,
            work_pose_free_axis=self.work_pose_free_axis,
            work_pose_info=lambda: (self.work_pose.active_name or "", getattr(self.work_pose, "ik_mode", "robust")),
        ))
        self.detect = DetectLocalize(self)
        self.runtime = RuntimeConfig(self)
        try:
            self.tcp.restore_active()
            self.points.restore_active()
        except (OSError, json.JSONDecodeError, ValueError):
            traceback.print_exc()
        self.handeye = HandEyeAPI(self)
        self.lidar = LidarAPI(self)
        # 多初值 IK 在子进程里算（不占主进程 GIL，控制循环不卡），启动时先把子进程起好
        ik_worker.warm()

    def reader(self) -> StateSource:
        with self._lock:
            if self._reader is None and self.motion_core_mode == "ipc":
                self._connect_core_locked()
            elif self._reader is None:
                self._reader = self.backend.state()
                # 指令话题占用检测：只读模式也开始听，页面一直显示有没有别的程序在发令
                try:
                    self.cmd_monitor = self.backend.cmd_monitor()
                except Exception:  # noqa: BLE001  检测起不来不影响读状态；接管时 commander 还会再建
                    traceback.print_exc()
                if self.allow_real_motion:
                    # 和运控核心进程同一份组装（control/assembly.py）：直写后端时 lowstate 过期就停在指令角保持；
                    # 守护层后端时急停同时锁守护层
                    self.loop, self.motion_core = build_motion_core(self.settings, self.kin, self.backend, self._reader)
                    self.payload.restore(self.motion_core)
                    self.attach_collision_guard(self.motion_core)
            return self._reader

    def _connect_core_locked(self) -> None:
        """ipc：连运控核心进程（连不上抛 PermissionErrorMotion，下次调用再试）。实测角也从核心读。"""
        if self._remote is None:
            from g1_motion_client import CoreConnection

            conn = CoreConnection(self._core_dir, client=f"web:{self.profile.name}:{os.getpid()}")
            self._remote = RemoteMotionCore(conn, self.profile, connect=False)
        self._remote.connect()
        self._reader = RemoteStateSource(self._remote.lowstate, self.profile)
        if self.allow_real_motion:
            self.motion_core = self._remote

    def remote_core(self) -> Optional[RemoteMotionCore]:
        """ipc 时到运控核心进程的连接（只读模式也有，用于状态和碰撞检查显示）；inprocess 为 None。"""
        if self.motion_core_mode != "ipc":
            return None
        self.reader()
        return self._remote

    def shutdown_motion(self) -> None:
        """进程退出时：inprocess 释放手臂并停控制循环（同以前）；ipc 只断开连接，手臂由核心保持。"""
        if self._remote is not None:
            self._remote.shutdown()
        elif self.motion_core is not None:
            self.motion_core.shutdown()

    def require_motion(self) -> None:
        if not self.allow_real_motion:
            raise PermissionErrorMotion("只读模式，不能发运动指令（需 --allow-real-motion）")

    def _board_stereo(self):
        """棋盘触摸「看一眼」用：(左目→躯干变换, 双目相机, 匹配器工厂)。"""
        from robot_platform.calib.handeye.config import load_config

        cfg = load_config()
        he = self.handeye
        T = he._left_cam_to_torso(cfg)
        if T is None:
            raise ValueError("先调用一组手眼结果，角点才能变到躯干上")
        cam = he._camera(cfg)
        if getattr(cam, "kind", "stereo") != "stereo":
            raise ValueError("棋盘触摸目前只支持双目相机（G1D）；RGB-D 相机的深度取点还没接入")
        return T, cam, lambda: he._matcher_for(he._calib_yaml(cfg))

    def motion(self) -> MotionCore:
        """运控核心：页面臂控指令和任务都只经它动手臂（进程内就是运控核心本身）；只读模式下抛 PermissionErrorMotion。"""
        self.reader()
        if self.motion_core is None:
            raise PermissionErrorMotion("未开 --allow-real-motion，不能发令")
        return self.motion_core
