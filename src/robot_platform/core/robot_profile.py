"""机型描述：robots/<名字>/robot.json → RobotProfile。

机型相关的数（关节表、电机号、增益、腰、立柱、URDF）只在 robot.json 里写一次，
代码通过 active() 取当前机型。字段说明见 robots/README.md。
"""

from __future__ import annotations

import json
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

from robot_platform.paths import ROBOTS_DIR


@dataclass(frozen=True)
class ArmLayout:
    name: str
    tip_link: str
    joint_names: Tuple[str, ...]
    motor_indices: Tuple[int, ...]
    home_q: Tuple[float, ...] = ()

    def __post_init__(self) -> None:
        if len(self.joint_names) != len(self.motor_indices):
            raise ValueError(f"{self.name}: joint_names/motor_indices length mismatch")
        if self.home_q and len(self.home_q) != len(self.joint_names):
            raise ValueError(f"{self.name}: home_q length mismatch")

    @property
    def n(self) -> int:
        return len(self.joint_names)

    def motor_index(self, joint_name: str) -> int:
        try:
            i = self.joint_names.index(joint_name)
        except ValueError as exc:
            raise KeyError(f"{joint_name!r} not in {self.name} arm") from exc
        return self.motor_indices[i]


@dataclass(frozen=True)
class WaistJoint:
    role: str  # yaw / pitch / roll
    urdf_joint: str
    motor: int
    limit_rad: Tuple[float, float]


@dataclass(frozen=True)
class WaistSpec:
    joints: Tuple[WaistJoint, ...]
    # 接管时一并保持的电机（可含 URDF 里没有的轴，如 G1D 的 13）
    hold_motors: Tuple[int, ...]
    lock_kp: float
    lock_kd: float
    # False：接管时只保持当前角，不接受腰部目标（如站立的 G1，转腰会影响平衡）
    movable: bool = True

    def has(self, role: str) -> bool:
        return any(j.role == role for j in self.joints)

    def joint(self, role: str) -> WaistJoint:
        for j in self.joints:
            if j.role == role:
                return j
        raise KeyError(f"腰没有 {role} 轴")

    @property
    def read_motors(self) -> Tuple[int, ...]:
        return tuple(j.motor for j in self.joints)


@dataclass(frozen=True)
class LiftSpec:
    state_topic: str
    urdf_joints: Tuple[str, ...]  # 多级棱柱，按伸出顺序
    stage_limit_m: float
    stroke_max_m: float
    height_zero_m: float  # 未标定时的默认最低位读数

    def height_to_joints(self, height_m: float, *, zero_m: Optional[float] = None) -> Dict[str, float]:
        """把实测高度映到 URDF 各级棱柱（逐级伸出）。"""
        zero = float(self.height_zero_m if zero_m is None else zero_m)
        rest = max(0.0, min(self.stroke_max_m, float(height_m) - zero))
        out: Dict[str, float] = {}
        for name in self.urdf_joints:
            q = max(0.0, min(self.stage_limit_m, rest))
            out[name] = q
            rest -= q
        return out


@dataclass(frozen=True)
class ControlSpec:
    backend: str
    state_topic: str
    command_topic: str
    release_motion_mode: bool
    # 退出运控前必须空闲的电机；读到在工作就拒绝接管
    idle_motors: Tuple[int, ...] = ()
    # 后端专用参数（如 arm_sdk 的权重过渡时长、运控原增益），由对应后端解释
    options: Dict[str, Any] = field(default_factory=dict)


CAMERA_TYPES = ("teleimager", "ros_rgbd", "mock", "mock_rgbd")


@dataclass(frozen=True)
class CameraSpec:
    """机型上的一台相机。type 决定怎么取图；其余字段是该类型的默认参数（实例配置可覆盖）。"""

    id: str
    label: str
    type: str  # teleimager（双目）/ ros_rgbd（RGB-D 经 ROS 话题）/ mock / mock_rgbd
    params: Dict[str, Any] = field(default_factory=dict)  # host、teleimager_path、ros_topics…


LIDAR_TYPES = ("unitree_dds", "mock")


@dataclass(frozen=True)
class LidarSpec:
    """机型上的一台雷达。type：unitree_dds（读宇树转发到 DDS 的 PointCloud2）/ mock；
    link：URDF 里雷达坐标系的连杆名，点云据此换到 base_link。"""

    id: str
    label: str
    type: str
    link: str
    params: Dict[str, Any] = field(default_factory=dict)  # topic …


@dataclass(frozen=True)
class TorsoImuSpec:
    """装在躯干上的 IMU（G1D：宇树 rt/secondary_imu）。重力前馈可以按它测到的重力方向算，
    不依赖腰关节读数。mount_rpy：IMU 坐标系相对 base_link 的姿态（rad）。"""

    topic: str
    mount_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    max_tilt_deg: float = 20.0  # 躯干倾角超过这个就当读数不可信，保持上一次的重力方向
    gravity_default: bool = False  # 服务启动时重力前馈是否就按 IMU 算
    note: str = ""


@dataclass(frozen=True)
class RobotProfile:
    name: str
    display_name: str
    root: Path
    urdf: Path
    base_link: str
    num_motors: int
    control: ControlSpec
    kp: Tuple[float, ...]
    kd: Tuple[float, ...]
    arms: Dict[str, ArmLayout]
    mirror_q_sign: Tuple[float, ...]
    waist: Optional[WaistSpec]
    lift: Optional[LiftSpec]
    # 只用于三维显示的其它关节（电机号 → URDF 关节名），如 G1 的腿
    display_joints: Dict[int, str] = field(default_factory=dict)
    # 相机，按 robot.json 里的顺序；第一台是默认
    cameras: Dict[str, CameraSpec] = field(default_factory=dict)
    # 雷达，按 robot.json 里的顺序；第一台是默认
    lidars: Dict[str, LidarSpec] = field(default_factory=dict)
    torso_imu: Optional[TorsoImuSpec] = None

    @property
    def meshes_dir(self) -> Path:
        return self.urdf.parent / "meshes"

    def arm(self, name: str) -> ArmLayout:
        key = str(name).strip().lower()
        if key not in self.arms:
            raise KeyError(f"Unknown arm {name!r}; expected one of {sorted(self.arms)}")
        return self.arms[key]


_CONTROL_KEYS = {"backend", "state_topic", "command_topic", "release_motion_mode", "idle_motors"}


def _pair(v: Any) -> Tuple[float, float]:
    lo, hi = (float(x) for x in v)
    return lo, hi


def _parse(raw: Dict[str, Any], root: Path) -> RobotProfile:
    n_motor = int(raw["num_motors"])
    gains = raw.get("gains") or {}
    kp = tuple(float(v) for v in gains.get("kp") or ())
    kd = tuple(float(v) for v in gains.get("kd") or ())
    if len(kp) != n_motor or len(kd) != n_motor:
        raise ValueError(f"gains.kp/kd 需要 {n_motor} 项")

    arms = {
        name: ArmLayout(
            name=name,
            tip_link=str(a["tip_link"]),
            joint_names=tuple(a["joint_names"]),
            motor_indices=tuple(int(i) for i in a["motor_indices"]),
            home_q=tuple(float(v) for v in a.get("home_q") or ()),
        )
        for name, a in (raw.get("arms") or {}).items()
    }
    if not arms:
        raise ValueError("arms 不能为空")
    mirror = tuple(float(v) for v in raw.get("mirror_q_sign") or ())
    for arm in arms.values():
        if mirror and len(mirror) != arm.n:
            raise ValueError(f"mirror_q_sign 长度 {len(mirror)} 与 {arm.name} 臂 {arm.n} 轴不符")
        if any(not 0 <= i < n_motor for i in arm.motor_indices):
            raise ValueError(f"{arm.name} 臂电机号超出 0..{n_motor - 1}")

    waist = None
    w = raw.get("waist")
    if w:
        waist = WaistSpec(
            joints=tuple(
                WaistJoint(
                    role=str(j["role"]),
                    urdf_joint=str(j["urdf_joint"]),
                    motor=int(j["motor"]),
                    limit_rad=_pair(j["limit_rad"]),
                )
                for j in w["joints"]
            ),
            hold_motors=tuple(int(i) for i in w.get("hold_motors") or (j["motor"] for j in w["joints"])),
            lock_kp=float(w["lock_kp"]),
            lock_kd=float(w["lock_kd"]),
            movable=bool(w.get("movable", True)),
        )

    lift = None
    li = raw.get("lift")
    if li:
        lift = LiftSpec(
            state_topic=str(li["state_topic"]),
            urdf_joints=tuple(li["urdf_joints"]),
            stage_limit_m=float(li["stage_limit_m"]),
            stroke_max_m=float(li["stroke_max_m"]),
            height_zero_m=float(li["height_zero_m"]),
        )

    c = raw.get("control") or {}
    urdf = (root / str(raw["urdf"])).resolve()
    return RobotProfile(
        name=str(raw["name"]),
        display_name=str(raw.get("display_name") or raw["name"]),
        root=root,
        urdf=urdf,
        base_link=str(raw.get("base_link") or "torso_link"),
        num_motors=n_motor,
        control=ControlSpec(
            backend=str(c.get("backend") or "unitree_lowcmd"),
            state_topic=str(c.get("state_topic") or "rt/lowstate"),
            command_topic=str(c.get("command_topic") or "rt/lowcmd"),
            release_motion_mode=bool(c.get("release_motion_mode", True)),
            idle_motors=tuple(int(i) for i in c.get("idle_motors") or ()),
            options={k: v for k, v in c.items() if k not in _CONTROL_KEYS},
        ),
        kp=kp,
        kd=kd,
        arms=arms,
        mirror_q_sign=mirror,
        waist=waist,
        lift=lift,
        display_joints={int(k): str(v) for k, v in (raw.get("display_joints") or {}).items()},
        cameras=_parse_cameras(raw.get("cameras") or {}),
        lidars=_parse_lidars(raw.get("lidars") or {}),
        torso_imu=_parse_torso_imu(raw.get("torso_imu")),
    )


def _parse_torso_imu(raw: Optional[Dict[str, Any]]) -> Optional[TorsoImuSpec]:
    if not raw:
        return None
    rpy = tuple(float(v) for v in raw.get("mount_rpy") or (0.0, 0.0, 0.0))
    if len(rpy) != 3:
        raise ValueError("torso_imu.mount_rpy 需要 3 个数")
    return TorsoImuSpec(
        topic=str(raw["topic"]),
        mount_rpy=rpy,
        max_tilt_deg=float(raw.get("max_tilt_deg", 20.0)),
        gravity_default=bool(raw.get("gravity_default", False)),
        note=str(raw.get("note") or ""),
    )


def _parse_lidars(raw: Dict[str, Any]) -> Dict[str, LidarSpec]:
    out: Dict[str, LidarSpec] = {}
    for lid, spec in raw.items():
        ltype = str(spec.get("type") or "")
        if ltype not in LIDAR_TYPES:
            raise ValueError(f"雷达 {lid!r} 的 type {ltype!r} 不认识（可用：{', '.join(LIDAR_TYPES)}）")
        if not spec.get("link"):
            raise ValueError(f"雷达 {lid!r} 缺 link（URDF 里雷达坐标系的连杆名）")
        params = {k: v for k, v in spec.items() if k not in ("type", "label", "link")}
        out[str(lid)] = LidarSpec(id=str(lid), label=str(spec.get("label") or lid), type=ltype,
                                  link=str(spec["link"]), params=params)
    return out


def _parse_cameras(raw: Dict[str, Any]) -> Dict[str, CameraSpec]:
    out: Dict[str, CameraSpec] = {}
    for cid, spec in raw.items():
        ctype = str(spec.get("type") or "")
        if ctype not in CAMERA_TYPES:
            raise ValueError(f"相机 {cid!r} 的 type {ctype!r} 不认识（可用：{', '.join(CAMERA_TYPES)}）")
        params = {k: v for k, v in spec.items() if k not in ("type", "label")}
        out[str(cid)] = CameraSpec(id=str(cid), label=str(spec.get("label") or cid), type=ctype, params=params)
    return out


def load_profile(name_or_path: str | Path) -> RobotProfile:
    """按名字（robots/<名字>/robot.json）或 robot.json 路径加载。"""
    p = Path(name_or_path)
    if p.suffix != ".json":
        p = ROBOTS_DIR / str(name_or_path) / "robot.json"
    if not p.is_file():
        known = sorted(d.name for d in ROBOTS_DIR.iterdir() if (d / "robot.json").is_file())
        raise FileNotFoundError(f"没有机型 {name_or_path!r}（已有：{', '.join(known)}）")
    raw = json.loads(p.read_text(encoding="utf-8"))
    return _parse(raw, p.parent.resolve())


_active: Optional[RobotProfile] = None
_lock = threading.Lock()


def activate(profile: RobotProfile) -> RobotProfile:
    global _active
    with _lock:
        _active = profile
    return profile


def active() -> RobotProfile:
    """当前机型。服务启动时由 config 指定；离线脚本直接调用时按默认 config 加载。"""
    with _lock:
        if _active is not None:
            return _active
    from robot_platform.core.config_loader import load_settings

    return activate(load_settings().profile)
