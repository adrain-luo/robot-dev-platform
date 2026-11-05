"""手眼 / 相机配置：实例的 state/handeye.json（选用的相机、内参、棋盘、会话、当前手眼结果）。

用哪台相机：
1. 实例配置里 ``camera`` 写了机型上某台相机的 id（相机卡片的下拉框选的就是它）→ 用它；
2. 否则若实例配置里直接写了 ``camera_source``（旧写法，或接口显式指定）→ 按这个来源；
3. 否则用机型 robot.json ``cameras`` 里的第一台。
实例里的 ``camera_host`` / ``teleimager_path`` / ``ros_topics`` 覆盖机型里的同名默认参数。
mock 后端下真实相机自动换成对应的仿真相机（双目 → mock，RGB-D → mock_rgbd）。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List

from robot_platform.core import instance as instances
from robot_platform.core.robot_profile import active as active_profile
from robot_platform.perception.camera import ROS_TOPICS, camera_kind


def config_path() -> Path:
    return instances.path("handeye_config")


DEFAULTS: Dict[str, Any] = {
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


MOCK_OF = {"teleimager": "mock", "ros_rgbd": "mock_rgbd"}
FALLBACK_CAMERA = {"type": "teleimager", "label": "头部双目（teleimager）",
                   "host": "192.168.123.164", "teleimager_path": ""}


def camera_choices() -> List[Dict[str, Any]]:
    """机型上声明的相机，给相机卡片的下拉框。"""
    return [{"id": c.id, "label": c.label, "type": c.type, "kind": camera_kind(c.type)}
            for c in active_profile().cameras.values()]


def effective_camera(cfg: Dict[str, Any], mock: bool = False) -> Dict[str, Any]:
    """按上面的规则算出当前要打开的相机：id、标签、来源 camera_source 及其参数。"""
    cams = active_profile().cameras
    cid = cfg.get("camera")
    if cid in cams:
        spec = cams[cid]
        ctype, label, params = spec.type, spec.label, dict(spec.params)
    elif cfg.get("camera_source"):
        ctype = str(cfg["camera_source"])
        match = next((c for c in cams.values() if c.type == ctype), None)
        cid = match.id if match else None
        label = match.label if match else ctype
        params = dict(match.params) if match else {}
    elif cams:
        spec = next(iter(cams.values()))
        cid, ctype, label, params = spec.id, spec.type, spec.label, dict(spec.params)
    else:
        cid, ctype, label = None, FALLBACK_CAMERA["type"], FALLBACK_CAMERA["label"]
        params = {k: v for k, v in FALLBACK_CAMERA.items() if k not in ("type", "label")}
    source = MOCK_OF.get(ctype, ctype) if mock else ctype
    return {
        "id": cid,
        "label": label,
        "type": ctype,
        "camera_source": source,
        "camera_kind": camera_kind(source),
        "camera_host": str(cfg.get("camera_host") or params.get("host") or FALLBACK_CAMERA["host"]),
        "teleimager_path": str(cfg.get("teleimager_path") or params.get("teleimager_path") or ""),
        "ros_topics": dict(ROS_TOPICS, **(params.get("ros_topics") or {}), **(cfg.get("ros_topics") or {})),
    }
