"""自标定的 RGB-D 彩色内参文件：存在实例 calib/color_intrinsics/<名字>.json，点云匹配时可选用。

文件原样保存（便于追溯标定来源），读取时经 rgbd.parse_color_intrinsics 校验。
"""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List

import instance as instances
import rgbd
from results import _check_name


def intr_dir() -> Path:
    return instances.path("color_intrinsics")


def _path(name: str) -> Path:
    return intr_dir() / f"{_check_name(name)}.json"


def list_files() -> List[Dict[str, Any]]:
    d = intr_dir()
    if not d.is_dir():
        return []
    out: List[Dict[str, Any]] = []
    for p in sorted(d.glob("*.json"), key=lambda q: q.stat().st_mtime, reverse=True):
        row: Dict[str, Any] = {"name": p.stem}
        try:
            intr = rgbd.parse_color_intrinsics(json.loads(p.read_text(encoding="utf-8")))
            K = intr["K"]
            row.update(size=intr["size"], fx=K[0], fy=K[4], cx=K[2], cy=K[5],
                       dist=intr["D"], rms_px=intr["rms_px"], saved_at=intr["saved_at"])
        except (OSError, ValueError, KeyError, TypeError) as exc:
            row["error"] = str(exc)
        out.append(row)
    return out


def load(name: str) -> Dict[str, Any]:
    p = _path(name)
    if not p.is_file():
        raise FileNotFoundError(f"没有内参文件「{name}」")
    return rgbd.parse_color_intrinsics(json.loads(p.read_text(encoding="utf-8")))


def save(name: str, doc: Any, force: bool = False) -> Dict[str, Any]:
    """校验后保存；同名已存在且没 force 时返回 exists。"""
    if isinstance(doc, str):
        try:
            doc = json.loads(doc)
        except json.JSONDecodeError as exc:
            raise ValueError(f"不是有效的 JSON：{exc}") from exc
    rgbd.parse_color_intrinsics(doc)
    p = _path(name)
    if p.exists() and not force:
        return {"ok": False, "exists": True, "error": f"内参文件「{name}」已存在"}
    doc = dict(doc)
    doc.setdefault("uploaded_at", datetime.now().isoformat(timespec="seconds"))
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(doc, indent=2, ensure_ascii=False), encoding="utf-8")
    return {"ok": True, "name": p.stem}


def delete(name: str) -> None:
    p = _path(name)
    if not p.is_file():
        raise FileNotFoundError(f"没有内参文件「{name}」")
    p.unlink()


def apply_to_info(info: Dict[str, Any], name: str) -> Dict[str, Any]:
    """用自标内参替换一帧的彩色 camera_info（换算到当前分辨率）；返回新 info 和说明。"""
    intr = load(name)
    color = dict(info["color"])
    adapted = rgbd.adapt_intrinsics(intr, color["size"])
    factory_K = list(color["K"])
    color.update(K=adapted["K"], D=adapted["D"], model="opencv")
    new_info = dict(info, color=color)
    K = adapted["K"]
    summary = {
        "name": name,
        "size": adapted["size"],
        "converted_from": adapted.get("converted_from"),
        "fx": K[0], "fy": K[4], "cx": K[2], "cy": K[5],
        "d_fx": K[0] - factory_K[0], "d_fy": K[4] - factory_K[4],
        "d_cx": K[2] - factory_K[2], "d_cy": K[5] - factory_K[5],
        "dist": adapted["D"],
    }
    return {"info": new_info, "summary": summary}
