"""头部相机。本模块只取图，不另起相机服务。

两类相机：
- 双目（stereo）：G1D 的 teleimager 左右拼图，拆成左、右两张；深度靠 SGBM 匹配。
- RGB-D（rgbd）：G1 的 RealSense D435i，经 ROS 话题取彩色 + 深度（毫米）与出厂内参。

上层统一用 grab() 取一帧 Frame；color 即“左目”，手眼与点云都以它的光学系为准。
"""

from __future__ import annotations

import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np

# D435i 在 G1 上由 realsense2_camera（ROS1 noetic）发布
ROS_TOPICS = {
    "color": "/camera/color/image_raw",
    "depth": "/camera/depth/image_rect_raw",
    "color_info": "/camera/color/camera_info",
    "depth_info": "/camera/depth/camera_info",
    "extrinsics": "/camera/extrinsics/depth_to_color",
}
ROS_PY_PATHS = ("/opt/ros/noetic/lib/python3/dist-packages", "/usr/lib/python3/dist-packages")
EXTRINSICS_WAIT_S = 1.5


@dataclass
class Frame:
    color: np.ndarray  # BGR；双目时是左目
    right: Optional[np.ndarray] = None  # 双目右目
    depth: Optional[np.ndarray] = None  # uint16，深度相机像素坐标，单位见 info["depth_scale_m"]
    info: Dict[str, Any] = field(default_factory=dict)  # RGB-D 的内参 / 外参


def camera_kind(source: str) -> str:
    return "rgbd" if source in ("ros_rgbd", "mock_rgbd") else "stereo"


def open_camera(cfg: Dict[str, Any]):
    source = str(cfg.get("camera_source") or "teleimager")
    if source == "ros_rgbd":
        return RosRgbdCamera(dict(ROS_TOPICS, **(cfg.get("ros_topics") or {})))
    if source == "mock_rgbd":
        return MockRgbdCamera()
    return HeadCamera(
        source=source,
        host=str(cfg.get("camera_host") or "192.168.123.164"),
        teleimager_path=str(cfg.get("teleimager_path") or ""),
    )


class HeadCamera:
    kind = "stereo"

    def __init__(self, source: str = "teleimager", host: str = "192.168.123.164", teleimager_path: str = ""):
        if source not in ("teleimager", "mock"):
            raise ValueError(f"未知相机来源: {source!r}（teleimager、mock、ros_rgbd 或 mock_rgbd）")
        self.source = source
        self.host = host
        self.teleimager_path = teleimager_path
        self.last_error = ""
        self._client = None
        self._lock = threading.Lock()
        self._mock_t0 = time.monotonic()

    def grab(self, timeout: float = 2.0) -> Optional[Frame]:
        pair = self.grab_pair(timeout)
        return None if pair is None else Frame(color=pair[0], right=pair[1])

    def grab_pair(self, timeout: float = 2.0) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if self.source == "mock":
            self.last_error = ""
            return self._mock_pair()
        with self._lock:
            if not self._ensure_client():
                return None
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                try:
                    frame, _fps = self._client.get_head_frame()
                except Exception as exc:
                    self.last_error = f"读头目失败: {exc}"
                    self._drop_client()
                    time.sleep(0.1)
                    if not self._ensure_client():
                        return None
                    continue
                if frame is not None:
                    h, w = frame.shape[:2]
                    if w % 2 != 0:
                        self.last_error = f"头目宽度不是偶数，无法左右拆分: {w}"
                        return None
                    left = frame[:, : w // 2].copy()
                    right = frame[:, w // 2 :].copy()
                    self.last_error = ""
                    return left, right
                time.sleep(0.05)
            self.last_error = f"{self.host} 在 {timeout:.0f}s 内没有头目画面"
            return None

    def close(self) -> None:
        with self._lock:
            self._drop_client()

    def _ensure_client(self) -> bool:
        if self._client is not None:
            return True
        path = (self.teleimager_path or "").strip()
        if path and path not in sys.path:
            if not Path(path).is_dir():
                self.last_error = f"teleimager 目录不存在: {path}"
                return False
            sys.path.insert(0, path)
        try:
            from teleimager.image_client import ImageClient
        except Exception as exc:
            self.last_error = f"无法导入 teleimager: {exc}"
            return False
        try:
            self._client = ImageClient(host=self.host)
        except Exception as exc:
            self.last_error = f"连接 {self.host} 的相机进程失败: {exc}"
            self._client = None
            return False
        self.last_error = ""
        return True

    def _drop_client(self) -> None:
        if self._client is None:
            return
        try:
            self._client.close()
        except Exception:
            pass
        self._client = None

    def _mock_pair(self) -> Tuple[np.ndarray, np.ndarray]:
        h, w = 480, 640
        img = np.full((h, w, 3), 30, dtype=np.uint8)
        t = time.monotonic() - self._mock_t0
        cx = int(w / 2 + 120 * np.cos(t))
        cy = int(h / 2 + 80 * np.sin(t * 0.7))
        cv2.rectangle(img, (cx - 60, cy - 40), (cx + 60, cy + 40), (200, 200, 200), -1)
        cv2.putText(img, "MOCK LEFT", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 212, 255), 2)
        right = img.copy()
        cv2.putText(right, "MOCK RIGHT", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 212, 255), 2)
        return img, right


def _decode_color(msg) -> np.ndarray:
    ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(msg.encoding)
    if ch is None:
        raise ValueError(f"不支持的彩色编码 {msg.encoding}")
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    img = rows[:, : msg.width * ch].reshape(msg.height, msg.width, ch)
    conv = {"rgb8": cv2.COLOR_RGB2BGR, "rgba8": cv2.COLOR_RGBA2BGR, "bgra8": cv2.COLOR_BGRA2BGR,
            "mono8": cv2.COLOR_GRAY2BGR}.get(msg.encoding)
    return cv2.cvtColor(img, conv) if conv is not None else img.copy()


def _decode_depth(msg) -> np.ndarray:
    if msg.encoding not in ("16UC1", "mono16"):
        raise ValueError(f"不支持的深度编码 {msg.encoding}")
    rows = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.step // 2)
    return rows[:, : msg.width].copy()


def _info_dict(msg) -> Dict[str, Any]:
    return {"size": [int(msg.width), int(msg.height)], "K": [float(v) for v in msg.K],
            "D": [float(v) for v in msg.D], "model": msg.distortion_model, "frame_id": msg.header.frame_id}


class RosRgbdCamera:
    """经 ROS1 话题取 D435i。借系统 noetic 的纯 Python 包（rospy、消息定义），不用 cv_bridge。"""

    kind = "rgbd"
    source = "ros_rgbd"

    def __init__(self, topics: Dict[str, str]):
        self.topics = topics
        self.last_error = ""
        self._lock = threading.Lock()
        self._subs: list = []
        self._latest: Dict[str, Any] = {}
        self._started = False
        self._started_at = 0.0
        self._wants_extrinsics = False

    def _start(self) -> bool:
        if self._started:
            return True
        for p in ROS_PY_PATHS:
            if Path(p).is_dir() and p not in sys.path:
                sys.path.append(p)
        try:
            import rospy
            from sensor_msgs.msg import CameraInfo, Image
        except Exception as exc:
            self.last_error = f"无法导入 rospy（需要 ROS noetic）: {exc}"
            return False
        try:
            if not rospy.core.is_initialized():
                rospy.init_node("arm_control_camera", anonymous=True, disable_signals=True)
            t = self.topics
            self._subs = [
                rospy.Subscriber(t["color"], Image, self._keep("color"), queue_size=1, buff_size=8 << 20),
                rospy.Subscriber(t["depth"], Image, self._keep("depth"), queue_size=1, buff_size=8 << 20),
                rospy.Subscriber(t["color_info"], CameraInfo, self._keep("color_info"), queue_size=1),
                rospy.Subscriber(t["depth_info"], CameraInfo, self._keep("depth_info"), queue_size=1),
            ]
            try:
                from realsense2_camera.msg import Extrinsics
                self._subs.append(rospy.Subscriber(t["extrinsics"], Extrinsics, self._keep("extrinsics"), queue_size=1))
                self._wants_extrinsics = True
            except Exception:
                pass  # 没有外参消息类型时按深度与彩色同轴、零平移处理，点云会差约 15 mm
        except Exception as exc:
            self.last_error = f"订阅 ROS 相机话题失败: {exc}"
            return False
        self._started = True
        self._started_at = time.monotonic()
        return True

    def _keep(self, name: str):
        def cb(msg) -> None:
            with self._lock:
                self._latest[name] = (msg, time.monotonic())
        return cb

    def grab(self, timeout: float = 2.0) -> Optional[Frame]:
        if not self._start():
            return None
        deadline = time.monotonic() + timeout
        start = time.monotonic()
        while time.monotonic() < deadline:
            with self._lock:
                got = dict(self._latest)
            c, d = got.get("color"), got.get("depth")
            # 取在调用之后到达的一对，彩色与深度时间戳相差不超过 1 帧
            # 外参是一次性（latched）消息，刚订阅时可能晚于图像到；首次取帧最多多等 EXTRINSICS_WAIT_S
            ext_ready = (not self._wants_extrinsics or "extrinsics" in got
                         or time.monotonic() - self._started_at > EXTRINSICS_WAIT_S)
            if (c and d and c[1] >= start and d[1] >= start and got.get("color_info") and got.get("depth_info")
                    and ext_ready):
                dt = abs(c[0].header.stamp.to_sec() - d[0].header.stamp.to_sec())
                if dt <= 0.04:
                    try:
                        return Frame(color=_decode_color(c[0]), depth=_decode_depth(d[0]),
                                     info=self._info(got, dt))
                    except ValueError as exc:
                        self.last_error = str(exc)
                        return None
            time.sleep(0.01)
        missing = [k for k in ("color", "depth", "color_info", "depth_info") if k not in self._latest]
        self.last_error = (f"{timeout:.0f}s 内没有收到 {', '.join(self.topics[k] for k in missing)}"
                           if missing else f"{timeout:.0f}s 内彩色与深度没有对上同一时刻的帧")
        return None

    def _info(self, got: Dict[str, Any], dt: float) -> Dict[str, Any]:
        info = {
            "source": "ros_rgbd",
            "color": _info_dict(got["color_info"][0]),
            "depth": _info_dict(got["depth_info"][0]),
            "depth_scale_m": 0.001,  # realsense2_camera 的 16UC1 深度单位是毫米
            "stamp_diff_s": float(dt),
        }
        ext = got.get("extrinsics")
        if ext:
            # librealsense 的旋转按列优先存放：p_color = R · p_depth + t
            info["depth_to_color"] = {
                "R": np.asarray(ext[0].rotation, float).reshape(3, 3).T.tolist(),
                "t": [float(v) for v in ext[0].translation],
            }
        else:
            info["depth_to_color"] = {"R": np.eye(3).tolist(), "t": [0.0, 0.0, 0.0], "assumed": True}
        return info

    def close(self) -> None:
        for s in self._subs:
            try:
                s.unregister()
            except Exception:
                pass
        self._subs = []
        self._started = False


class MockRgbdCamera:
    """本地测试用：彩色画一张亮块，深度是 1.2 m 处的平面；内参取 G1 实测值。"""

    kind = "rgbd"
    source = "mock_rgbd"

    def __init__(self):
        self.last_error = ""

    def grab(self, timeout: float = 2.0) -> Optional[Frame]:
        h, w = 480, 640
        color = np.full((h, w, 3), 40, np.uint8)
        cv2.rectangle(color, (220, 160), (420, 320), (200, 200, 200), -1)
        cv2.putText(color, "MOCK RGB-D", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 212, 255), 2)
        depth = np.full((h, w), 1200, np.uint16)
        depth[:, :8] = 0  # 左缘无效，同真实 D435 的深度盲边
        info = {
            "source": "mock_rgbd",
            "color": {"size": [w, h], "K": [607.45, 0, 326.83, 0, 606.94, 247.34, 0, 0, 1], "D": [0.0] * 5,
                      "model": "plumb_bob", "frame_id": "camera_color_optical_frame"},
            "depth": {"size": [w, h], "K": [390.15, 0, 312.64, 0, 390.15, 240.39, 0, 0, 1], "D": [0.0] * 5,
                      "model": "plumb_bob", "frame_id": "camera_depth_optical_frame"},
            "depth_scale_m": 0.001,
            "stamp_diff_s": 0.0,
            "depth_to_color": {"R": np.eye(3).tolist(), "t": [0.015, 0.0, 0.0]},
        }
        return Frame(color=color, depth=depth, info=info)

    def close(self) -> None:
        pass
