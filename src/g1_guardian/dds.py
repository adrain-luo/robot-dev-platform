"""Unitree DDS adapters: lowstate in, rt/lowcmd or rt/arm_sdk out.

The SDK (unitree_sdk2py) is imported lazily so the rest of the package and
the tests run without it.
"""

from __future__ import annotations

import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Deque, Optional, Set

from g1_guardian.config import GuardianConfig
from g1_guardian.core import Frame, LowStateView

_initialized = False


def add_sdk_path(explicit: Optional[str]) -> None:
    candidates = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    here = Path(__file__).resolve()
    candidates += [
        here.parents[2] / "third_party" / "unitree_sdk2_python",
        Path("/home/unitree/unitree_sdk2_python"),
    ]
    for c in candidates:
        if (c / "unitree_sdk2py").exists():
            if str(c) not in sys.path:
                sys.path.insert(0, str(c))
            return
    try:
        import unitree_sdk2py  # noqa: F401
    except ImportError as exc:
        raise RuntimeError("Cannot find unitree_sdk2py; pass --sdk-path") from exc


def init_channel(domain: int, interface: Optional[str]) -> None:
    global _initialized
    if _initialized:
        return
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    if interface:
        ChannelFactoryInitialize(domain, interface)
    else:
        ChannelFactoryInitialize(domain)
    _initialized = True


class DdsLowState:
    """Keeps the latest rt/lowstate with its receive time (monotonic)."""

    def __init__(self, topic: str = "rt/lowstate", num_motors: int = 29) -> None:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        self.num_motors = num_motors
        self._latest: Optional[LowStateView] = None
        self.count = 0
        self._sub = ChannelSubscriber(topic, LowState_)
        self._sub.Init(self._on_msg, 10)

    def _on_msg(self, msg) -> None:
        q = [float(msg.motor_state[i].q) for i in range(self.num_motors)]
        # one reference assignment: atomic for the reader in the loop thread
        self._latest = LowStateView(q=q, stamp=time.monotonic(), mode_machine=int(getattr(msg, "mode_machine", 0)))
        self.count += 1

    def latest(self) -> Optional[LowStateView]:
        return self._latest


class DdsOutput:
    """Writes frames to rt/lowcmd (G1D) or rt/arm_sdk (G1) and watches the
    same topic for frames that are not ours (another writer)."""

    def __init__(self, cfg: GuardianConfig) -> None:
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.utils.crc import CRC

        self.cfg = cfg
        self.topic = "rt/lowcmd" if cfg.channel == "lowcmd" else "rt/arm_sdk"
        self._crc = CRC()
        self._cmd = unitree_hg_msg_dds__LowCmd_()
        self._pub = ChannelPublisher(self.topic, LowCmd_)
        self._pub.Init()
        self._own: Deque[int] = deque(maxlen=512)
        self._own_set: Set[int] = set()
        self._lock = threading.Lock()
        self.foreign_seen_at = float("-inf")
        self.foreign_count = 0
        self.prepared = False
        self._mon = ChannelSubscriber(self.topic, LowCmd_)
        self._mon.Init(self._on_topic, 10)
        if cfg.channel == "arm_sdk":
            self._motors = sorted(set(cfg.active_motors) | set(cfg.hold_at_engage))
        else:
            self._motors = list(range(cfg.num_motors))

    def _on_topic(self, msg) -> None:
        crc = int(msg.crc)
        with self._lock:
            ours = crc in self._own_set
        if not ours:
            self.foreign_seen_at = time.monotonic()
            self.foreign_count += 1

    def prepare(self) -> None:
        """Before the first frame: on G1D release Unitree's motion mode."""
        if self.prepared:
            return
        if self.cfg.channel == "lowcmd" and self.cfg.release_motion_mode:
            from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

            msc = MotionSwitcherClient()
            msc.SetTimeout(5.0)
            msc.Init()
            _, result = msc.CheckMode()
            tries = 0
            while isinstance(result, dict) and result.get("name"):
                tries += 1
                if tries > 10:
                    raise RuntimeError(f"motion mode still active after ReleaseMode: {result}")
                print(f"[g1_guardian] releasing motion mode: {result}", flush=True)
                msc.ReleaseMode()
                time.sleep(1.0)
                _, result = msc.CheckMode()
        self.prepared = True

    def write(self, frame: Frame) -> None:
        cmd = self._cmd
        cmd.mode_pr = 0
        cmd.mode_machine = int(frame.mode_machine)
        for i in self._motors:
            slot = cmd.motor_cmd[i]
            slot.mode = 1
            slot.q = float(frame.q[i])
            slot.dq = 0.0
            slot.kp = float(frame.kp[i])
            slot.kd = float(frame.kd[i])
            slot.tau = float(frame.tau[i])
        if self.cfg.channel == "arm_sdk":
            cmd.motor_cmd[int(self.cfg.arm_sdk_weight_motor)].q = float(frame.weight or 0.0)
        cmd.crc = self._crc.Crc(cmd)
        crc = int(cmd.crc)
        with self._lock:
            if len(self._own) == self._own.maxlen:
                self._own_set.discard(self._own[0])
            self._own.append(crc)
            self._own_set.add(crc)
        self._pub.Write(cmd)
