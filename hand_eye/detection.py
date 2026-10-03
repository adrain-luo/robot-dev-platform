"""棋盘内角检测。网格严格按给定的内角数，不放大。"""

from __future__ import annotations

from typing import Tuple

import cv2
import numpy as np


def parse_board_size(size_str: str) -> Tuple[int, int]:
    """'11x8' -> (cols, rows)，内角个数。"""
    parts = str(size_str).lower().split("x")
    if len(parts) != 2:
        raise ValueError(f"棋盘尺寸格式不对: {size_str}，应为 '列x行'")
    return int(parts[0]), int(parts[1])


def detect_corners(
    image: np.ndarray,
    board_size: Tuple[int, int] = (11, 8),
    draw: bool = False,
) -> Tuple[bool, np.ndarray]:
    """返回 (是否找到, 原图或画了角点的副本)。"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 不用 CALIB_CB_LARGER：它会让 SB 检出比 board_size 更大的网格。
    found, corners = cv2.findChessboardCornersSB(gray, board_size)
    result = image.copy()
    if found and draw:
        cv2.drawChessboardCorners(result, board_size, corners, found)
    return found, result
