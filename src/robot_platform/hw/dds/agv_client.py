"""G1D AgvClient（Python）：底盘 Move + 立柱 HeightAdjust。

对应 C++ unitree::robot::g1::AgvClient：
  服务名 agv，HeightAdjust API 1002，参数 JSON {\"data\": vz}，vz∈[-1,1]。
"""

from __future__ import annotations

import json
import threading
from pathlib import Path
from typing import Optional

from robot_platform.hw.backends.base import ChassisDriver, LiftDriver
from robot_platform.hw.dds.sdk import init_channel

AGV_SERVICE_NAME = "agv"
AGV_API_VERSION = "1.0.0.1"
ROBOT_API_ID_AGV_MOVE = 1001
ROBOT_API_ID_AGV_HEIGHT_ADJUST = 1002

# 官方：vz=±1 → ±76.5 mm/s
LIFT_MAX_SPEED_MPS = 0.0765


class AgvClient(LiftDriver):
    def __init__(
        self,
        *,
        network_interface: str,
        dds_domain: int = 0,
        sdk_path: Optional[Path] = None,
        timeout_s: float = 3.0,
    ) -> None:
        init_channel(dds_domain, network_interface, sdk_path)
        from unitree_sdk2py.rpc.client import Client

        self._cli = Client(AGV_SERVICE_NAME, False)
        self._cli.SetTimeout(float(timeout_s))
        self._cli._SetApiVerson(AGV_API_VERSION)
        self._cli._RegistApi(ROBOT_API_ID_AGV_MOVE, 0)
        self._cli._RegistApi(ROBOT_API_ID_AGV_HEIGHT_ADJUST, 0)
        self._lock = threading.Lock()

    def height_adjust(self, vz: float) -> int:
        """立柱速度指令。vz 归一化 [-1, 1]，正为升高。"""
        v = max(-1.0, min(1.0, float(vz)))
        parameter = json.dumps({"data": v})
        with self._lock:
            code, _ = self._cli._Call(ROBOT_API_ID_AGV_HEIGHT_ADJUST, parameter)
        return int(code)

    def stop_height(self) -> int:
        return self.height_adjust(0.0)

    def move(self, vx: float, vy: float, vyaw: float) -> int:
        parameter = json.dumps({"vx": float(vx), "vy": float(vy), "vyaw": float(vyaw)})
        with self._lock:
            code, _ = self._cli._Call(ROBOT_API_ID_AGV_MOVE, parameter)
        return int(code)


class AgvChassisDriver(ChassisDriver):
    """底盘 Move。单独一个 RPC 客户端、超时短（0.5 s）：立柱闭环的 3 s 超时不能卡住底盘停车。"""

    def __init__(self, *, network_interface: str, dds_domain: int = 0, sdk_path: Optional[Path] = None,
                 timeout_s: float = 0.5) -> None:
        self._agv = AgvClient(network_interface=network_interface, dds_domain=dds_domain,
                              sdk_path=sdk_path, timeout_s=timeout_s)

    def move(self, vx: float, vyaw: float) -> int:
        return self._agv.move(float(vx), 0.0, float(vyaw))
