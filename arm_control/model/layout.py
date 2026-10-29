"""G1D arm joint layout (motor indices + URDF joint names).

Source of truth for code. Keep in sync with config/g1d.example.yaml.
Validated against vision_arm_control …/g1d_joint_layout.py and
unitree-joint-control-ab/config/robot.yaml.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple


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
WAIST_YAW_LIMIT_RAD = (-2.7053, 2.7053)
# URDF 里名叫 Yaw_Joint，轴是 Y，实际是前后倾。电机 14 没有母线电压和温度，角度恒为 0。
WAIST_PITCH_JOINT = "Yaw_Joint"
WAIST_PITCH_MOTOR_INDEX = 14
WAIST_PITCH_LIMIT_RAD = (-0.04363, 2.3562)
# G1 29-DoF 腰组：生产 HAL 在纯臂模式会一起写 12/13/14。13 在标准 G1 是腰 roll；
# 这份 G1D URDF 没有 roll 活动关节，但有 LZ_* 升降棱柱。高度不在 rt/lowstate，
# 官方走 AgvClient.HeightAdjust + 订阅 rt/hispeed_state（Point32.y 高度 / .z 速度）。
WAIST_MOTOR_INDICES = (12, 13, 14)
LIFT_TOPIC = "rt/hispeed_state"
LIFT_JOINT_CANDIDATES = ("LZ_mt_Joint", "LZ_it_Joint")  # URDF 两级棱柱，各约 0～0.21 m
LIFT_STAGE_LIMIT_M = 0.21
LIFT_STROKE_MAX_M = 0.42  # 两级合计；与整机最低/最高差约 420 mm 一致
# 真机收到最低位时 rt/hispeed_state Point32.y（2026-03-29 实测约 -0.1897）
LIFT_HEIGHT_ZERO_M = -0.1897
BASE_LINK = "torso_link"


def lift_height_to_joints(
    height_m: float, *, zero_m: Optional[float] = None
) -> Dict[str, float]:
    """把 hispeed 高度映到 URDF 两级棱柱（先伸 LZ_mt，再伸 LZ_it）。"""
    zero = float(LIFT_HEIGHT_ZERO_M if zero_m is None else zero_m)
    stroke = float(height_m) - zero
    stroke = max(0.0, min(LIFT_STROKE_MAX_M, stroke))
    q_mt = max(0.0, min(LIFT_STAGE_LIMIT_M, stroke))
    q_it = max(0.0, min(LIFT_STAGE_LIMIT_M, stroke - LIFT_STAGE_LIMIT_M))
    return {"LZ_mt_Joint": q_mt, "LZ_it_Joint": q_it}


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
