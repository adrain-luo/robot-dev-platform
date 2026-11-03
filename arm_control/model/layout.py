"""手臂关节布局：按当前机型（robots/<名字>/robot.json）取。"""

from __future__ import annotations

from typing import List, Sequence

from model.robot_profile import ArmLayout, active

__all__ = ["ArmLayout", "arm_names", "get_arm", "short_joint_label", "format_q_table"]


def get_arm(name: str) -> ArmLayout:
    return active().arm(name)


def arm_names() -> List[str]:
    return list(active().arms)


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
