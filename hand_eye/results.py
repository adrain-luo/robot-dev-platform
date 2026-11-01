"""命名保存的手眼结果。数字拷在 config/results/，不依赖采集目录。"""

from __future__ import annotations

import json
import re
import shutil
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
RESULTS_DIR = ROOT / "config" / "results"
_NAME_RE = re.compile(r"^[\w\-\u4e00-\u9fff.]{1,48}$")


def _check_name(name: str) -> str:
    name = (name or "").strip()
    if not _NAME_RE.match(name):
        raise ValueError("名字只能用中英文、数字、_ - .，1～48 个字符")
    return name


def _path(name: str) -> Path:
    return RESULTS_DIR / f"{_check_name(name)}.json"


def list_saved() -> List[Dict[str, Any]]:
    if not RESULTS_DIR.is_dir():
        return []
    out = []
    for p in sorted(RESULTS_DIR.glob("*.json"), key=lambda x: x.stat().st_mtime, reverse=True):
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        fit = d.get("intrinsics_fit") or {}
        out.append({
            "name": d.get("name") or p.stem,
            "session": d.get("session") or "",
            "arm": d.get("arm") or "",
            "poseset": d.get("poseset") or "",
            "baseline_mm": fit.get("baseline_mm"),
            "saved_at": d.get("saved_at") or "",
        })
    return out


def load_saved(name: str) -> Dict[str, Any]:
    p = _path(name)
    if not p.is_file():
        raise FileNotFoundError(f"没有标定结果 {name}")
    return json.loads(p.read_text(encoding="utf-8"))


def delete_saved(name: str) -> None:
    p = _path(name)
    if not p.is_file():
        raise FileNotFoundError(f"没有标定结果 {name}")
    p.unlink()
    yaml = p.with_suffix(".yaml")
    if yaml.is_file():
        yaml.unlink()


def _fit_from_yaml(yaml_path: Path) -> Dict[str, Any]:
    fs = cv2.FileStorage(str(yaml_path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(f"打不开内参: {yaml_path}")
    try:
        Kl = fs.getNode("left_camera_matrix").mat()
        Kr = fs.getNode("right_camera_matrix").mat()
        T = fs.getNode("T").mat()
    finally:
        fs.release()
    if Kl is None or Kr is None:
        raise ValueError(f"{yaml_path} 缺少左右相机矩阵")
    baseline = float(np.linalg.norm(T)) if T is not None else None
    return {
        "left_fx": float(Kl[0, 0]),
        "left_fy": float(Kl[1, 1]),
        "right_fx": float(Kr[0, 0]),
        "right_fy": float(Kr[1, 1]),
        "baseline_mm": baseline,
    }


def _read_result(data_dir: Path) -> Dict[str, Any]:
    combined = data_dir / "handeye_result.json"
    if combined.is_file():
        return json.loads(combined.read_text(encoding="utf-8"))
    result = {}
    for eye in ("left", "right"):
        f = data_dir / f"handeye_result_{eye}.json"
        if f.is_file():
            result[eye] = json.loads(f.read_text(encoding="utf-8"))
    if not result:
        raise FileNotFoundError("这次会话还没有手眼结果，先求解")
    return result


def save_named(name: str, data_dir: Path) -> Dict[str, Any]:
    name = _check_name(name)
    yaml_src = data_dir / "stereo_calibration.yaml"
    if not yaml_src.is_file():
        raise FileNotFoundError("这次会话还没有内参，先求解")
    result = _read_result(data_dir)
    fit_path = data_dir / "intrinsics_fit.json"
    if fit_path.is_file():
        fit = json.loads(fit_path.read_text(encoding="utf-8"))
    else:
        fit = _fit_from_yaml(yaml_src)
    meta_path = data_dir / "session_meta.json"
    meta = json.loads(meta_path.read_text(encoding="utf-8")) if meta_path.is_file() else {}
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    yaml_dst = RESULTS_DIR / f"{name}.yaml"
    shutil.copyfile(yaml_src, yaml_dst)
    doc = {
        "name": name,
        "saved_at": datetime.now().isoformat(timespec="seconds"),
        "session": data_dir.name,
        "arm": meta.get("arm") or "",
        "tip_link": meta.get("tip_link") or "",
        "base_link": meta.get("base_link") or "torso_link",
        "poseset": meta.get("poseset") or "",
        "board_size": meta.get("board_size") or "",
        "square_size_mm": fit.get("square_size_mm"),
        "intrinsics_yaml": str(yaml_dst),
        "intrinsics_fit": fit,
        "result": result,
    }
    _path(name).write_text(json.dumps(doc, indent=2, ensure_ascii=False), encoding="utf-8")
    return doc
