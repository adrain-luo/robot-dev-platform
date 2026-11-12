"""机器人上的传感器装在哪：给三维画位置标记用（/api/sensors）。

- 双目相机：优先用当前生效的手眼标定结果（左 / 右眼各自的 T_cam2base，躯干系），这是实测的准确位姿，
  点云也用同一个结果放进躯干系；没有标定结果时退回机型声明的 URDF 连杆（标称位置）。
- 雷达：机型 lidars 声明的 URDF 连杆（G1 / G1D：mid360_link，照宇树 URDF）。

每项给出挂在哪个 URDF 连杆下（parent_link）和相对它的 4×4 位姿 T，三维把标记挂在那个连杆上，
躯干倾斜（G1D 按 IMU 摆）时跟着一起动。相机坐标系是 OpenCV 光学系：z 朝前（视线）、x 朝右、y 朝下。
"""

from __future__ import annotations

from typing import Any, Dict, List

import numpy as np

from robot_platform.core.robot_profile import RobotProfile

EYE_LABEL = {"left": "左眼", "right": "右眼"}


def _short(label: str) -> str:
    """三维标签用短名：去掉括号里的说明（「头部双目（teleimager）」→「头部双目」）。"""
    return label.split("（")[0].split("(")[0].strip() or label


def _handeye_eyes(base_link: str) -> Dict[str, Any]:
    """当前生效的手眼结果里各眼的位姿（躯干系）。没有 / 读不了返回 {}。"""
    try:
        from robot_platform.calib.handeye import results as he_results
        from robot_platform.calib.handeye.config import load_config

        name = str(load_config().get("active_result") or "")
        if not name:
            return {}
        doc = he_results.load_saved(name)
    except Exception:  # noqa: BLE001  标定文件缺失 / 损坏：退回标称位置
        return {}
    out = {}
    for eye, res in ((doc.get("result") or {}).items()):
        T = res.get("T_cam2base") if isinstance(res, dict) else None
        if eye not in EYE_LABEL or T is None or str(res.get("base_link") or base_link) != base_link:
            continue
        T = np.asarray(T, dtype=float)
        if T.shape != (4, 4) or not np.all(np.isfinite(T)):
            continue
        resid = (res.get("residual_translation_mm") or {}).get("mean")
        out[eye] = {"T": T, "result": name, "residual_mm": None if resid is None else float(resid)}
    return out


# ROS 机体系（x 前、y 左、z 上）→ OpenCV 光学系（z 前、x 右、y 下）：R = rpy(-90°, 0, -90°)
R_BODY_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def sensor_list(profile: RobotProfile) -> List[Dict[str, Any]]:
    sensors: List[Dict[str, Any]] = []
    for key, cam in profile.cameras.items():
        eyes = _handeye_eyes(profile.base_link)
        if eyes:
            for eye, e in eyes.items():
                note = f"手眼标定「{e['result']}」"
                if e["residual_mm"] is not None:
                    note += f"，残差 {e['residual_mm']:.1f} mm"
                sensors.append({
                    "id": f"camera:{key}:{eye}", "kind": "camera", "label": f"{_short(cam.label)} {EYE_LABEL[eye]}" if len(eyes) > 1 else _short(cam.label), "eye": eye,
                    "parent_link": profile.base_link, "T": e["T"].tolist(), "source": note, "calibrated": True,
                })
            break  # 手眼结果只对应当前用的那台相机
        link = str(cam.params.get("link") or "")
        if link:
            T = np.eye(4)
            T[:3, :3] = R_BODY_OPTICAL  # URDF 相机连杆按 ROS 机体系
            sensors.append({
                "id": f"camera:{key}", "kind": "camera", "label": _short(cam.label),
                "parent_link": link, "T": T.tolist(),
                "source": f"URDF {link}（标称位置，未做手眼标定）", "calibrated": False,
            })
    for key, lid in profile.lidars.items():
        sensors.append({
            "id": f"lidar:{key}", "kind": "lidar", "label": _short(lid.label or key),
            "parent_link": lid.link, "T": np.eye(4).tolist(),
            "source": f"URDF {lid.link}", "calibrated": False,
        })
    return sensors
