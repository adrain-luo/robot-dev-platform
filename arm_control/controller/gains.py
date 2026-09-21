"""Default G1 29-DoF PD gains (from vision_arm_control HAL)."""

from __future__ import annotations

from typing import List

# Index = motor index 0..28
KP_BODY: List[float] = [
    60, 60, 60, 100, 40, 40,
    60, 60, 60, 100, 40, 40,
    60, 40, 40,
    40, 40, 40, 40, 40, 40, 40,
    200, 40, 180, 180, 40, 40, 40,
]
KD_BODY: List[float] = [
    1, 1, 1, 2, 1, 1,
    1, 1, 1, 2, 1, 1,
    1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1,
]

G1_NUM_MOTOR = 29

# 接管锁腰：生产 HAL 腰 yaw kp=60，人手能掰动。锁零时用更高刚度（仍是 PD，不是机械刹车）。
WAIST_LOCK_KP = 200.0
WAIST_LOCK_KD = 4.0
