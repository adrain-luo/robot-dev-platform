"""G1D arm joint layout (motor indices + URDF joint names).

Source of truth for code. Keep in sync with config/g1d.example.yaml.
Validated against vision_arm_control …/g1d_joint_layout.py and
unitree-joint-control-ab/config/robot.yaml.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple


@dataclass(frozen=True)
class ArmLayout:
    name: str
    tip_link: str
    joint_names: Tuple[str, ...]
    motor_indices: Tuple[int, ...]

    def __post_init__(self) -> None:
        if len(self.joint_names) != len(self.motor_indices):
            raise ValueError(f"{self.name}: joint_names/motor_indices length mismatch")
        if len(self.joint_names) != 7:
            raise ValueError(f"{self.name}: expected 7-DoF arm, got {len(self.joint_names)}")

    @property
    def n(self) -> int:
        return len(self.joint_names)

    def motor_index(self, joint_name: str) -> int:
        try:
            i = self.joint_names.index(joint_name)
        except ValueError as exc:
            raise KeyError(f"{joint_name!r} not in {self.name} arm") from exc
        return self.motor_indices[i]


RIGHT = ArmLayout(
    name="right",
    tip_link="right_wrist_yaw_link",
    joint_names=(
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ),
    motor_indices=(22, 23, 24, 25, 26, 27, 28),
)

LEFT = ArmLayout(
    name="left",
    tip_link="left_wrist_yaw_link",
    joint_names=(
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
    ),
    motor_indices=(15, 16, 17, 18, 19, 20, 21),
)

ARMS: Dict[str, ArmLayout] = {"right": RIGHT, "left": LEFT}

WAIST_YAW_JOINT = "torso_Joint"
WAIST_YAW_MOTOR_INDEX = 12
# URDF 里名叫 Yaw_Joint，轴是 Y，实际是前后倾。电机 14 没有母线电压和温度，角度恒为 0。
# 手眼光轴离躯干竖直 34.3°，调试口相对地面是 47.6°。差 13.3°，绕 +Y 为负，只用于画面。
WAIST_PITCH_JOINT = "Yaw_Joint"
WAIST_PITCH_MOTOR_INDEX = 14
WAIST_PITCH_OFFSET_RAD = -0.2321
# G1 29-DoF 腰组：生产 HAL 在纯臂模式会一起写 12/13/14。13 是腰 roll，这份 URDF 没有对应活动关节。
WAIST_MOTOR_INDICES = (12, 13, 14)
BASE_LINK = "torso_link"


def get_arm(name: str) -> ArmLayout:
    key = str(name).strip().lower()
    if key not in ARMS:
        raise KeyError(f"Unknown arm {name!r}; expected one of {sorted(ARMS)}")
    return ARMS[key]


def short_joint_label(joint_name: str) -> str:
    """right_shoulder_pitch_joint -> shoulder_pitch"""
    name = joint_name
    for prefix in ("right_", "left_"):
        if name.startswith(prefix):
            name = name[len(prefix) :]
            break
    if name.endswith("_joint"):
        name = name[: -len("_joint")]
    return name


def format_q_table(arm: ArmLayout, q: Sequence[float], *, degrees: bool = True) -> List[str]:
    import math

    lines = []
    for name, qi in zip(arm.joint_names, q):
        if degrees:
            lines.append(f"  {short_joint_label(name):16s}  {math.degrees(qi):8.2f} deg")
        else:
            lines.append(f"  {short_joint_label(name):16s}  {qi:9.4f} rad")
    return lines
