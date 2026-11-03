"""硬件后端接口。

上层（ArmLoop、server、立柱闭环）只通过这里的接口读状态、发指令；
具体怎么接 DDS / 仿真由各后端实现，见 backends/README.md。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple, Union

from model.robot_profile import ArmLayout


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

    def lift_reader(self) -> LiftReader:
        raise NotImplementedError(f"{self.name} 后端没有立柱")

    def lift_driver(self) -> LiftDriver:
        raise NotImplementedError(f"{self.name} 后端没有立柱")

    def close(self) -> None:
        pass
