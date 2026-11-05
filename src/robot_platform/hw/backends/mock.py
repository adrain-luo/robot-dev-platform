"""mock：不接硬件的仿真机器人，用于本地开发与自动测试。

- 电机：有指令且刚度 > 0 时，实测角按一阶惯性跟随指令角（时间常数约 40 ms）；
  刚度为 0（拖拽）时停在原处；指令超过 0.5 s 不来就保持当前角。
- 力矩估计：kp·(q_cmd − q) − kd·dq + τ_ff，只是给页面一个合理的数。
- 立柱：HeightAdjust 的 vz 乘官方满速 76.5 mm/s 积分，夹在零位到最大行程之间。
- 初始姿态：两臂取 robot.json 的 home_q，腰 0，立柱在行程中段；有腿的机型摆成屈膝站姿。
"""

from __future__ import annotations

import math
import threading
import time
from typing import Dict, List, Optional, Sequence, Union

from robot_platform.hw.backends.base import (
    ArmState,
    Backend,
    Commander,
    LiftDriver,
    LiftReader,
    LiftState,
    MotorSnapshot,
    StateSource,
)
from robot_platform.core.robot_profile import ArmLayout, RobotProfile

SIM_DT_S = 0.005
TRACK_TAU_S = 0.04
CMD_TIMEOUT_S = 0.5
LIFT_MAX_SPEED_MPS = 0.0765
LIFT_CMD_TIMEOUT_S = 0.5
MOCK_MODE_MACHINE = 5
# G1 站立时实测的腿部角度（rad）
MOCK_STAND = {"hip_pitch": -0.37, "knee": 0.63, "ankle_pitch": -0.30}


class MockRobot:
    """仿真状态与积分线程。所有读写都过同一把锁。"""

    def __init__(self, profile: RobotProfile) -> None:
        self.profile = profile
        n = profile.num_motors
        self._lock = threading.Lock()
        self.q: List[float] = [0.0] * n
        self.dq: List[float] = [0.0] * n
        self.tau: List[float] = [0.0] * n
        for arm in profile.arms.values():
            for i, qi in zip(arm.motor_indices, arm.home_q or [0.0] * arm.n):
                self.q[i] = float(qi)
        # 有腿的机型（只显示用的关节）摆成 G1 实测的屈膝站姿
        for i, name in profile.display_joints.items():
            for key, q in MOCK_STAND.items():
                if key in name and i < n:
                    self.q[i] = q
        self._cmd: Optional[Dict[str, List[float]]] = None
        self._cmd_t = 0.0
        self.last_t = time.monotonic()
        lift = profile.lift
        self.lift_height = 0.0 if lift is None else lift.height_zero_m + 0.5 * lift.stroke_max_m
        self.lift_v = 0.0
        self._lift_vz = 0.0
        self._lift_cmd_t = 0.0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="mock-robot", daemon=True)
        self._thread.start()

    def command(self, q, kp, kd, tau) -> None:
        with self._lock:
            self._cmd = {"q": list(q), "kp": list(kp), "kd": list(kd), "tau": list(tau)}
            self._cmd_t = time.monotonic()

    def lift_command(self, vz: float) -> None:
        with self._lock:
            self._lift_vz = max(-1.0, min(1.0, float(vz)))
            self._lift_cmd_t = time.monotonic()

    def _step(self, dt: float) -> None:
        now = time.monotonic()
        cmd = self._cmd if self._cmd is not None and now - self._cmd_t < CMD_TIMEOUT_S else None
        alpha = 1.0 - math.exp(-dt / TRACK_TAU_S)
        for i in range(len(self.q)):
            q_old = self.q[i]
            if cmd is not None and i < len(cmd["q"]) and cmd["kp"][i] > 0.0:
                self.q[i] = q_old + alpha * (cmd["q"][i] - q_old)
            self.dq[i] = (self.q[i] - q_old) / dt
            if cmd is not None and i < len(cmd["q"]):
                self.tau[i] = (cmd["kp"][i] * (cmd["q"][i] - self.q[i])
                               - cmd["kd"][i] * self.dq[i] + cmd["tau"][i])
            else:
                self.tau[i] = 0.0
        lift = self.profile.lift
        if lift is not None:
            vz = self._lift_vz if now - self._lift_cmd_t < LIFT_CMD_TIMEOUT_S else 0.0
            lo, hi = lift.height_zero_m, lift.height_zero_m + lift.stroke_max_m
            h = min(hi, max(lo, self.lift_height + vz * LIFT_MAX_SPEED_MPS * dt))
            self.lift_v = (h - self.lift_height) / dt
            self.lift_height = h
        self.last_t = now

    def _run(self) -> None:
        prev = time.monotonic()
        while not self._stop.is_set():
            time.sleep(SIM_DT_S)
            now = time.monotonic()
            with self._lock:
                self._step(max(1e-4, now - prev))
            prev = now

    def read(self, indices: Sequence[int]) -> Dict[int, Dict[str, float]]:
        with self._lock:
            return {int(i): {"q": self.q[i], "dq": self.dq[i], "tau_est": self.tau[i]} for i in indices}

    def snapshot(self) -> MotorSnapshot:
        with self._lock:
            return MotorSnapshot(q=tuple(self.q), mode_machine=MOCK_MODE_MACHINE)

    def lift_state(self) -> LiftState:
        with self._lock:
            return LiftState(timestamp_monotonic=self.last_t, x=0.0,
                             height_m=self.lift_height, velocity_mps=self.lift_v)

    def age_s(self) -> float:
        with self._lock:
            return time.monotonic() - self.last_t

    def close(self) -> None:
        self._stop.set()


class MockState(StateSource):
    def __init__(self, robot: MockRobot) -> None:
        self._robot = robot

    def wait_for_message(self, timeout_s: Optional[float] = None) -> None:
        return None

    def age_s(self) -> float:
        return self._robot.age_s()

    def read_arm(self, arm: Union[str, ArmLayout], *, wait: bool = True) -> ArmState:
        layout = arm if isinstance(arm, ArmLayout) else self._robot.profile.arm(arm)
        st = self._robot.read(layout.motor_indices)
        rows = [st[i] for i in layout.motor_indices]
        return ArmState(
            timestamp_monotonic=self._robot.last_t,
            mode_machine=MOCK_MODE_MACHINE,
            q=tuple(r["q"] for r in rows),
            dq=tuple(r["dq"] for r in rows),
            tau_est=tuple(r["tau_est"] for r in rows),
        )

    def read_motors(self, motor_indices: Sequence[int]) -> Dict[int, float]:
        return {i: s["q"] for i, s in self._robot.read(motor_indices).items()}

    def read_motor_states(self, motor_indices: Sequence[int]) -> Dict[int, Dict[str, float]]:
        return self._robot.read(motor_indices)

    def snapshot(self) -> MotorSnapshot:
        return self._robot.snapshot()


class MockCommander(Commander):
    def __init__(self, robot: MockRobot) -> None:
        self._robot = robot

    def prepare(self) -> None:
        return None

    def write(self, q, kp, kd, tau, *, mode_machine: Optional[int] = None) -> None:
        self._robot.command(q, kp, kd, tau)


class MockLiftReader(LiftReader):
    def __init__(self, robot: MockRobot) -> None:
        self._robot = robot

    def latest(self) -> Optional[LiftState]:
        return self._robot.lift_state()

    def age_s(self) -> float:
        return self._robot.age_s()


class MockLiftDriver(LiftDriver):
    def __init__(self, robot: MockRobot) -> None:
        self._robot = robot

    def height_adjust(self, vz: float) -> int:
        self._robot.lift_command(vz)
        return 0


class MockBackend(Backend):
    name = "mock"

    def __init__(self, profile: RobotProfile) -> None:
        self.profile = profile
        self.robot = MockRobot(profile)
        self._state = MockState(self.robot)
        self._commander = MockCommander(self.robot)

    def state(self) -> StateSource:
        return self._state

    def commander(self) -> Commander:
        return self._commander

    def lift_reader(self) -> LiftReader:
        if self.profile.lift is None:
            return super().lift_reader()
        return MockLiftReader(self.robot)

    def lift_driver(self) -> LiftDriver:
        if self.profile.lift is None:
            return super().lift_driver()
        return MockLiftDriver(self.robot)

    def close(self) -> None:
        self.robot.close()
