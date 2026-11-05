"""雷达：取点云（宇树转发到 DDS 的 PointCloud2 / 仿真），并按 URDF 把点换到 base_link（躯干）。

G1 的 Livox MID-360 把数据发给宇树运控机 192.168.123.161，由它转发到 DDS 话题
rt/utlidar/cloud_livox_mid360。这里只订阅这个话题，不连雷达、不改雷达的目的地址
（用 Livox SDK 直连会把目的地址改到本机，宇树那边就收不到了）。
"""

from __future__ import annotations

import math
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path
from typing import Deque, Optional, Tuple

import numpy as np

# PointField.datatype → numpy
_PF_DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def decode_pointcloud2(msg) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """PointCloud2 → (xyz[N,3] float32, intensity[N] 或 None)。只认 x/y/z/intensity 字段。"""
    n = int(msg.width) * int(msg.height)
    names, formats, offsets = [], [], []
    for f in msg.fields:
        dt = _PF_DTYPES.get(int(f.datatype))
        if dt is None or int(f.count) != 1:
            continue
        names.append(f.name)
        formats.append((">" if msg.is_bigendian else "<") + dt)
        offsets.append(int(f.offset))
    for k in ("x", "y", "z"):
        if k not in names:
            raise ValueError(f"点云缺 {k} 字段")
    dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": int(msg.point_step)})
    raw = msg.data if isinstance(msg.data, (bytes, bytearray)) else bytes(msg.data)
    arr = np.frombuffer(raw, dtype=dtype, count=n)
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float32)
    inten = arr["intensity"].astype(np.float32) if "intensity" in names else None
    return xyz, inten


def _rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def link_transform(urdf: Path, base_link: str, link: str) -> np.ndarray:
    """URDF 里 base_link → link 的 4×4 位姿（link 系的点乘它得到 base_link 系）。
    沿 link 往上找父连杆，链上必须都是 fixed 关节（雷达装在躯干上就是这样）。"""
    root = ET.parse(str(urdf)).getroot()
    parent_of = {}
    for j in root.findall("joint"):
        child = j.find("child").get("link")
        parent_of[child] = j
    T = np.eye(4)
    cur = link
    while cur != base_link:
        j = parent_of.get(cur)
        if j is None:
            raise ValueError(f"URDF 里从 {link} 走不到 {base_link}")
        if j.get("type") != "fixed":
            raise ValueError(f"{link} 到 {base_link} 之间有活动关节 {j.get('name')}，雷达位姿需要实时关节角，暂不支持")
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        Tj = np.eye(4)
        Tj[:3, :3] = _rpy_matrix(*rpy)
        Tj[:3, 3] = xyz
        T = Tj @ T
        cur = j.find("parent").get("link")
    return T


class _RateMeter:
    def __init__(self, window_s: float = 3.0) -> None:
        self.window_s = window_s
        self.t: Deque[float] = deque()

    def tick(self) -> None:
        now = time.monotonic()
        self.t.append(now)
        while self.t and now - self.t[0] > self.window_s:
            self.t.popleft()

    def hz(self) -> float:
        if len(self.t) < 2:
            return 0.0
        span = self.t[-1] - self.t[0]
        return (len(self.t) - 1) / span if span > 0 else 0.0


class DdsLidar:
    """订阅宇树 DDS 上的 PointCloud2（只读）。消息在回调里只存引用，取用时再解码。"""

    kind = "unitree_dds"

    def __init__(self, topic: str, dds_domain: int, network_interface: str, sdk_path: Optional[Path] = None):
        self.topic = topic
        self.last_error = ""
        self._lock = threading.Lock()
        self._msg = None
        self._t = 0.0
        self._rate = _RateMeter()
        self._sub = None
        self._closed = False
        try:
            from robot_platform.hw.dds.sdk import init_channel
            init_channel(dds_domain, network_interface, sdk_path)
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
            self._sub = ChannelSubscriber(topic, PointCloud2_)
            # queueLen=0：回调直接在 DDS 监听线程里跑（只存引用，很快）。
            # 不用 SDK 的队列线程：它的 Close() 会 join 那个线程，而 SDK 的退出通知有竞态，
            # 偶尔丢唤醒，join 就永远等下去（G1 上实测卡死过整个雷达接口）。
            self._sub.Init(self._on_msg, 0)
        except Exception as exc:  # noqa: BLE001  DDS / SDK 不可用时只报错，不影响别的功能
            self.last_error = f"订阅 {topic} 失败：{exc}"

    def _on_msg(self, msg) -> None:
        if self._closed:
            return
        with self._lock:
            self._msg = msg
            self._t = time.monotonic()
            self._rate.tick()

    def latest(self, max_age_s: float = 2.0):
        with self._lock:
            msg, t, hz = self._msg, self._t, self._rate.hz()
        if msg is None:
            if not self.last_error:
                self.last_error = f"还没收到 {self.topic}"
            return None
        if time.monotonic() - t > max_age_s:
            self.last_error = f"{self.topic} 已 {time.monotonic() - t:.1f} s 没有新数据"
            return None
        xyz, inten = decode_pointcloud2(msg)
        self.last_error = ""
        return {"xyz": xyz, "intensity": inten, "hz": hz, "frame_id": msg.header.frame_id,
                "stamp": float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9}

    def close(self) -> None:
        """断开订阅。不阻塞调用方：SDK 的 Close() 放到后台线程里做，万一卡住也只卡那个线程。"""
        self._closed = True
        sub, self._sub = self._sub, None
        with self._lock:
            self._msg = None
        if sub is None:
            return

        def _close() -> None:
            try:
                sub.Close()
            except Exception:  # noqa: BLE001
                pass

        threading.Thread(target=_close, name="lidar-close", daemon=True).start()


class MockLidar:
    """仿真：地面 + 前方一堵墙 + 一个箱子，按雷达安装位姿换到雷达系（看起来和真雷达一样要再换回来）。"""

    kind = "mock"

    def __init__(self, T_base_lidar: np.ndarray):
        self.last_error = ""
        self._T_lidar_base = np.linalg.inv(T_base_lidar)
        self._rng = np.random.default_rng(0)
        self._t0 = time.monotonic()

    def latest(self, max_age_s: float = 2.0):
        r = self._rng
        g = r.uniform(-6, 6, size=(9000, 2))
        ground = np.c_[g, np.full(len(g), -0.75)]  # 躯干系里地面大约在 0.75 m 以下
        wall = np.c_[np.full(3000, 3.0), r.uniform(-3, 3, 3000), r.uniform(-0.75, 1.5, 3000)]
        box = np.c_[r.uniform(1.0, 1.4, 1500), r.uniform(-0.3, 0.3, 1500), r.uniform(-0.75, -0.35, 1500)]
        pts = np.vstack([ground, wall, box]) + r.normal(0, 0.01, (13500, 3))
        homo = np.c_[pts, np.ones(len(pts))]
        xyz = (self._T_lidar_base @ homo.T).T[:, :3].astype(np.float32)
        inten = r.uniform(0, 255, len(xyz)).astype(np.float32)
        return {"xyz": xyz, "intensity": inten, "hz": 10.0, "frame_id": "mock", "stamp": time.time()}

    def close(self) -> None:
        pass
