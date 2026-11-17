"""机器实例：一台具体机器的设置与标定数据，放在 instances/<名字>/。

机型（robots/<机型>/robot.json）描述“这一类机器长什么样”；实例描述“这一台”：
网卡、相机地址，以及它自己的 TCP、手眼、负载、空间点等标定结果。两台同型号的机器各有一个实例，
数据互不覆盖。布局见 instances/README.md。

选哪个实例：--instance / 环境变量 ARM_INSTANCE / instances/default 文件（只在本机，不进 git）。
"""

from __future__ import annotations

import json
import os
import shutil
import tempfile
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Union

from robot_platform.paths import INSTANCES_DIR, REPO_ROOT as REPO
DEFAULT_FILE = INSTANCES_DIR / "default"
INSTANCE_FILE = "instance.json"

# 实例内各类数据的位置（相对实例根目录）
LAYOUT: Dict[str, str] = {
    "tcp_results": "calib/tcp",
    "work_poses": "calib/work_poses",
    "spatial_points": "calib/spatial_points",
    "handeye_results": "calib/handeye",
    "payloads": "calib/payloads",
    "posesets": "calib/posesets",
    "single_poses": "calib/single_poses",
    "color_intrinsics": "calib/color_intrinsics",
    "lift_zero": "state/lift_zero.json",
    "runtime_lock": "state/runtime_lock.json",
    "pose_ik_mode": "state/pose_ik_mode.json",
    "collision_config": "state/collision.json",
    "handeye_config": "state/handeye.json",
    "lidar_config": "state/lidar.json",
    "handeye_sessions": "data/handeye",
    "clouds": "data/clouds",
    "board_touch": "data/board_touch",
    "measure": "data/measure",
    "validate": "data/validate",
}

# 改成实例之前，这些数据散在仓库各模块下的位置（相对仓库根目录）。
# 迁移脚本按它搬家；旧文件里存的绝对路径也按它映射到实例里。
LEGACY_LAYOUT: Dict[str, str] = {
    "tcp_results": "arm_control/config/tcp_results",
    "work_poses": "arm_control/config/work_poses",
    "spatial_points": "arm_control/config/spatial_points",
    "handeye_results": "hand_eye/config/results",
    "payloads": "arm_payload_gravity/config/payloads",
    "posesets": "arm_payload_gravity/config/posesets",
    "lift_zero": "arm_control/config/lift_zero.json",
    "runtime_lock": "arm_control/config/runtime_lock.json",
    "pose_ik_mode": "arm_control/config/pose_ik_mode.json",
    "handeye_config": "hand_eye/config.json",
    "clouds": "hand_eye/data/clouds",
    "board_touch": "hand_eye/data/board_touch",
    "handeye_sessions": "hand_eye/data",
    "measure": "arm_payload_gravity/data/measure",
    "validate": "arm_payload_gravity/data/validate",
}
LEGACY_SETTINGS = "arm_control/config/g1d.json"


@dataclass
class Instance:
    name: str
    root: Path
    settings: Dict[str, Any] = field(default_factory=dict)
    temporary: bool = False

    @property
    def robot(self) -> str:
        return str(self.settings.get("robot") or "g1d")

    def path(self, key: str) -> Path:
        return self.root / LAYOUT[key]

    def to_stored(self, p: Union[str, Path]) -> str:
        """写进数据文件的路径：在实例里就存相对路径，换机器 / 换目录也能用。"""
        p = Path(p).resolve()
        try:
            return p.relative_to(self.root.resolve()).as_posix()
        except ValueError:
            return str(p)

    def from_stored(self, s: Optional[str]) -> Optional[Path]:
        """读数据文件里的路径：相对路径按实例根解析；旧版绝对路径若已不存在，按旧布局映射进实例。"""
        if not s:
            return None
        p = Path(str(s))
        # 以 / 开头的按绝对路径处理（Windows 上读设备写下的 /home/... 也一样）
        if not (p.is_absolute() or str(s).startswith("/")):
            return self.root / p
        posix = p.as_posix()
        # 别的仓库副本里同名实例写下的绝对路径（例如搬到新仓库前的旧目录）：本实例有同一个文件就用本实例的
        marker = f"/instances/{self.name}/"
        i = posix.find(marker)
        if i >= 0:
            mine = self.root / posix[i + len(marker):]
            if mine.exists():
                return mine
        if p.exists():
            return p
        for key, legacy in sorted(LEGACY_LAYOUT.items(), key=lambda kv: -len(kv[1])):
            marker = "/" + legacy.rstrip("/") + "/"
            i = posix.find(marker)
            if i >= 0:
                return self.path(key) / posix[i + len(marker):]
        return p


def available() -> List[str]:
    if not INSTANCES_DIR.is_dir():
        return []
    return sorted(d.name for d in INSTANCES_DIR.iterdir() if (d / INSTANCE_FILE).is_file())


def load_instance(name_or_path: Union[str, Path]) -> Instance:
    p = Path(name_or_path)
    root = p if (p / INSTANCE_FILE).is_file() else INSTANCES_DIR / str(name_or_path)
    f = root / INSTANCE_FILE
    if not f.is_file():
        raise FileNotFoundError(f"没有实例 {name_or_path!r}（已有：{', '.join(available()) or '无'}）")
    return Instance(name=root.name, root=root.resolve(), settings=json.loads(f.read_text(encoding="utf-8")))


def temporary_instance(robot: str = "g1d") -> Instance:
    """空的临时实例（mock 默认用它），进程退出后由调用方 cleanup。"""
    root = Path(tempfile.mkdtemp(prefix="arm_instance_"))
    settings = {"robot": robot, "note": "临时实例，退出即删"}
    (root / INSTANCE_FILE).write_text(json.dumps(settings, ensure_ascii=False, indent=2), encoding="utf-8")
    return Instance(name="temp", root=root, settings=settings, temporary=True)


def cleanup(inst: Instance) -> None:
    if inst.temporary:
        shutil.rmtree(inst.root, ignore_errors=True)


def default_name() -> Optional[str]:
    env = os.environ.get("ARM_INSTANCE")
    if env:
        return env
    if DEFAULT_FILE.is_file():
        name = DEFAULT_FILE.read_text(encoding="utf-8").strip()
        return name or None
    return None


def resolve(name: Optional[str] = None) -> Instance:
    name = name or default_name()
    if not name:
        raise SystemExit(
            "没有指定机器实例：用 --instance <名字>、环境变量 ARM_INSTANCE，"
            f"或把名字写进 {DEFAULT_FILE}（已有：{', '.join(available()) or '无'}）"
        )
    return load_instance(name)


_active: Optional[Instance] = None
_lock = threading.Lock()


def activate(inst: Instance) -> Instance:
    global _active
    with _lock:
        _active = inst
    return inst


def active() -> Instance:
    """当前实例。服务启动时指定；离线脚本直接调用时按默认规则选。"""
    with _lock:
        if _active is not None:
            return _active
    return activate(resolve())


def path(key: str) -> Path:
    return active().path(key)
