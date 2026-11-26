"""组装一台机器的控制循环和运控核心：网页进程（--motion-core inprocess）和运控核心进程
（g1_motion_core.service）用同一份，免得两处的限速、lowstate 上限、守护层急停联动走样。"""

from __future__ import annotations

import traceback
from typing import Tuple

from robot_platform.control.arm_loop import ArmLoop
from robot_platform.control.motion_core import MotionCore


def build_motion_core(settings, kin, backend, reader) -> Tuple[ArmLoop, MotionCore]:
    """建控制循环（会创建后端的指令出口，即写入者）和包住它的运控核心。

    - lowstate 过期上限来自后端（直写后端 = 机型 robot.json 的值；守护层后端 / mock 为 None）；
    - 守护层后端：急停同时锁守护层，守护层复位也经运控核心。"""
    torso_imu = None
    try:
        torso_imu = backend.torso_imu()
    except Exception:  # noqa: BLE001  IMU 起不来：前馈照旧按躯干竖直算
        traceback.print_exc()
    loop = ArmLoop(
        reader=reader,
        commander=backend.commander(),
        kin=kin,
        max_speed_rad_s=settings.max_speed_rad_s,
        urdf_path=settings.urdf,
        base_link=settings.base_link,
        torso_imu=torso_imu,
        lowstate_max_age_s=backend.lowstate_max_age_s(),
    )
    core = MotionCore(loop,
                      external_estop=getattr(backend, "estop_guardian", None),
                      external_reset=getattr(backend, "reset_guardian", None))
    return loop, core

