"""硬件后端接口。

上层（ArmLoop、server、立柱闭环）只通过这里的接口读状态、发指令；
具体怎么接 DDS / 仿真由各后端实现，见 backends/README.md。
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Dict, Optional, Sequence, Tuple, Union

from robot_platform.core.robot_profile import ArmLayout


class PermissionErrorMotion(PermissionError):
    """拒绝发令（只读模式、未接管、安全检查不过等），接口返回 403。"""


def check_idle_motors(state: "StateSource", motors: Sequence[int]) -> None:
    """退出运控前确认这些电机没在工作（如 G1D 没有腿）。

    读到任何一个有角度或力矩，就说明连着的不是声明的机型（例如双足 G1 正站着），
    此时退出运控会撤掉平衡控制，必须拒绝。
    """
    if not motors:
        return
    st = state.read_motor_states(motors)
    active = [i for i in motors if abs(st[i]["q"]) > 1e-4 or abs(st[i]["tau_est"]) > 1e-3]
    if active:
        raise PermissionErrorMotion(
            f"电机 {active} 正在工作，与机型声明不符（可能是站立中的双足机器人）。"
            "为防止退出运控导致摔倒，拒绝接管。"
        )


@dataclass(frozen=True)
class ArmState:
    timestamp_monotonic: float
    mode_machine: Optional[int]
    q: tuple  # rad, length = arm.n
    dq: tuple
    tau_est: tuple  # may contain None


@dataclass(frozen=True)
class MotorSnapshot:
    """一帧全部电机的实测角（按电机号），供指令帧填未接管的电机。"""

    q: Tuple[float, ...]
    mode_machine: Optional[int]


@dataclass(frozen=True)
class LiftState:
    timestamp_monotonic: float
    x: float
    height_m: float
    velocity_mps: float


class StateSource(ABC):
    """只读关节状态。"""

    @abstractmethod
    def wait_for_message(self, timeout_s: Optional[float] = None) -> None: ...

    @abstractmethod
    def age_s(self) -> float: ...

    @abstractmethod
    def read_arm(self, arm: Union[str, ArmLayout], *, wait: bool = True) -> ArmState: ...

    @abstractmethod
    def read_motors(self, motor_indices: Sequence[int]) -> Dict[int, float]: ...

    @abstractmethod
    def read_motor_states(self, motor_indices: Sequence[int]) -> Dict[int, Dict[str, float]]:
        """返回 {idx: {q, dq, tau_est}}；tau_est 缺失时为 nan。"""

    @abstractmethod
    def snapshot(self) -> MotorSnapshot: ...

    def read_imu(self) -> Optional[Dict[str, Any]]:
        """机身 IMU（腿式机型在骨盆）：{"rpy": [r, p, y] rad, "quat": [w, x, y, z]}；没有就 None。"""
        return None


class Commander(ABC):
    """电机指令出口。只在接管后由 ArmLoop 周期调用 write。"""

    @abstractmethod
    def prepare(self) -> None:
        """接管前调用一次（例如退出高层运动模式）。"""

    @abstractmethod
    def write(
        self,
        q: Sequence[float],
        kp: Sequence[float],
        kd: Sequence[float],
        tau: Sequence[float],
        *,
        mode_machine: Optional[int] = None,
    ) -> None:
        """发一帧：下标即电机号，长度为本帧要写的电机数；dq 目标恒为 0。"""

    def release(self) -> None:
        """释放。需要平滑交还控制权的后端在这里开始过渡，过渡期间 busy() 为真、ArmLoop 继续发帧。"""

    def busy(self) -> bool:
        """接管或释放过渡中，需要继续发帧。"""
        return False

    def ready(self) -> bool:
        """过渡完成，可以接受运动目标。"""
        return True

    def phase(self) -> str:
        """给页面看的过渡阶段（空串表示无过渡）。"""
        return ""


class TorsoImu(ABC):
    """躯干 IMU（只读）。latest() 返回 (四元数 [w, x, y, z]：IMU 系 → 世界系, 收到时刻 monotonic)。"""

    @abstractmethod
    def latest(self) -> Optional[Tuple[Tuple[float, float, float, float], float]]: ...

    def age_s(self) -> float:
        got = self.latest()
        return float("inf") if got is None else time.monotonic() - got[1]

    def close(self) -> None:
        pass


class LiftReader(ABC):
    @abstractmethod
    def latest(self) -> Optional[LiftState]: ...

    @abstractmethod
    def age_s(self) -> float: ...


class LiftDriver(ABC):
    @abstractmethod
    def height_adjust(self, vz: float) -> int:
        """立柱速度指令，vz 归一化 [-1, 1]，正为升高；返回 0 为成功。"""

    def stop_height(self) -> int:
        return self.height_adjust(0.0)


class Backend(ABC):
    """一台机器人的全部硬件出口。各部件按需懒创建，同一后端只建一份。"""

    name: str = ""

    @abstractmethod
    def state(self) -> StateSource: ...

    @abstractmethod
    def commander(self) -> Commander: ...

    def cmd_monitor(self):
        """指令话题占用检测（hw/cmd_monitor.CmdMonitor）；不支持的后端返回 None。"""
        return None

    def lift_reader(self) -> LiftReader:
        raise NotImplementedError(f"{self.name} 后端没有立柱")

    def torso_imu(self) -> Optional[TorsoImu]:
        """机型声明了躯干 IMU 才有；没有返回 None（重力前馈就按躯干竖直算）。"""
        return None

    def lift_driver(self) -> LiftDriver:
        raise NotImplementedError(f"{self.name} 后端没有立柱")

    def close(self) -> None:
        pass
