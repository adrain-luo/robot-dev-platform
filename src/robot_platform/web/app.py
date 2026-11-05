"""App：持有设置、后端、控制循环和各功能会话；具体接口方法按功能放在 web/api/ 下。"""

from __future__ import annotations

from typing import Optional
import json
import threading
import traceback

from robot_platform.calib.handeye.service import HandEyeAPI
from robot_platform.calib.payload.api import PayloadAPI
from robot_platform.calib.tcp import TcpSession
from robot_platform.control.arm_loop import ArmLoop, PermissionErrorMotion
from robot_platform.control.fk_scene import RobotKinematics
from robot_platform.core.runtime_config import RuntimeConfig
from robot_platform.hw.backends.base import Backend, LiftDriver, LiftReader, StateSource
from robot_platform.hw.lift_control import LiftController
from robot_platform.perception.detect_localize import DetectLocalize
from robot_platform.perception.lidar_api import LidarAPI
from robot_platform.tasks.board_touch import BoardTouch
from robot_platform.tasks.spatial_points import SpatialPoints
from robot_platform.tasks.work_pose import WorkPoseSession
from robot_platform.web.api.state import StateMixin
from robot_platform.web.api.lift import LiftMixin
from robot_platform.web.api.tcp import TcpMixin
from robot_platform.web.api.work_pose import WorkPoseMixin
from robot_platform.web.api.points import PointsMixin
from robot_platform.web.api.runtime import RuntimeMixin


class App(StateMixin, LiftMixin, TcpMixin, WorkPoseMixin, PointsMixin, RuntimeMixin):
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
        self.lidar = LidarAPI(self)

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

    def require_motion(self) -> None:
        if not self.allow_real_motion:
            raise PermissionErrorMotion("只读模式，不能发运动指令（需 --allow-real-motion）")

    def require_loop(self) -> ArmLoop:
        self.reader()
        if self.loop is None:
            raise PermissionErrorMotion("未开 --allow-real-motion，不能发令")
        return self.loop
