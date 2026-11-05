"""手眼 / 相机配置：实例的 state/handeye.json（相机来源、内参、棋盘、会话、当前手眼结果）。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict

from robot_platform.core import instance as instances


def config_path() -> Path:
    return instances.path("handeye_config")


DEFAULTS: Dict[str, Any] = {
    "camera_source": "teleimager",
    "camera_host": "192.168.123.164",
    "teleimager_path": "/home/robot/yx/teleimager_src_copy",
    "intrinsics": "",
    "board_size": "11x8",
    "square_size_mm": None,
    "session": "",
    "eye": "both",
}


def load_config() -> Dict[str, Any]:
    cfg = dict(DEFAULTS)
    if config_path().is_file():
        try:
            raw = json.loads(config_path().read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            raw = {}
        if isinstance(raw, dict):
            cfg.update(raw)
    return cfg


def save_config(cfg: Dict[str, Any]) -> None:
    config_path().parent.mkdir(parents=True, exist_ok=True)
    config_path().write_text(json.dumps(cfg, indent=2, ensure_ascii=False), encoding="utf-8")
