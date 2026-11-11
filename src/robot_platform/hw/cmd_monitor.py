"""指令话题占用检测：有没有别的程序在往本机型的指令话题（rt/lowcmd 或 rt/arm_sdk）发令。

DDS 是多对多的：任何进程都能往同一个话题发，机器人会同时收到。两个程序一起发的表现是
"接管看似成功，但手臂发抖 / 卸力后被对方的 kp 顶住不动"；G1D 的 rt/lowcmd 没有权重混合，
两边直接打架。所以：
- 服务启动就开始只读订阅指令话题，页面一直显示占用情况（只读模式也显示）；
- 接管前听一段时间，听到别人的指令就拒绝接管；
- 接管期间仍有别人的指令，页面标红。

怎么分辨"别人"：本程序每发一帧都把它的 CRC 记下来，收到的消息 CRC 在记录里就是自己的，
不在就是别人的。所以接管期间也能准确数出第三方，不用"总频率减本程序频率"去估。

本模块只放判定逻辑，不碰 SDK（便于测试）；DDS 订阅见 hw/dds/cmd_sub.py，mock 后端直接喂消息。
"""

from __future__ import annotations

import collections
import os
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Deque, Dict, List, Optional, Sequence

from robot_platform.hw.backends.base import PermissionErrorMotion

WINDOW_S = 1.0          # 统计频率的窗口
LISTEN_S = 1.0          # 接管前至少听这么久
FOREIGN_MIN_MSGS = 2    # 窗口里别人的消息达到这么多条才算占用（容忍偶发的一条）
OWN_CRC_KEEP = 512      # 记住最近这么多帧自己的 CRC（100 Hz 下约 5 s，远大于 DDS 回环延迟）
HINT_CACHE_S = 3.0
# 本机疑似在发令的进程关键词（只看命令行参数）；机型可在 robot.json 的 control.cmd_hints 覆盖
DEFAULT_HINTS = ("arm_sdk", "lowcmd", "teleop", "reach_server")


def hints_for(profile) -> tuple:
    return tuple(profile.control.options.get("cmd_hints") or DEFAULT_HINTS)


class CmdMonitor:
    def __init__(self, topic: str, *, hints: Sequence[str] = (), clock: Callable[[], float] = time.monotonic) -> None:
        self.topic = topic
        self.hints = tuple(h for h in hints if h)
        self._clock = clock
        self._lock = threading.Lock()
        self._t_start = clock()
        self._own: Deque[int] = collections.deque(maxlen=OWN_CRC_KEEP)
        self._own_set: Dict[int, int] = {}
        self._own_times: Deque[float] = collections.deque()
        self._foreign_times: Deque[float] = collections.deque()
        self._last_foreign: Optional[Dict] = None
        self._hint_cache: tuple = (0.0, [])

    # ------------------------------------------------------------ 输入
    def note_own(self, crc: int) -> None:
        """本程序发出一帧（在 Write 之前调用）。"""
        crc = int(crc)
        with self._lock:
            if len(self._own) == self._own.maxlen:
                old = self._own[0]
                n = self._own_set.get(old, 0) - 1
                if n <= 0:
                    self._own_set.pop(old, None)
                else:
                    self._own_set[old] = n
            self._own.append(crc)
            self._own_set[crc] = self._own_set.get(crc, 0) + 1

    def on_message(self, crc: int, *, weight: Optional[float] = None, kp_max: Optional[float] = None) -> None:
        """订阅到一条指令消息（自己的或别人的）。"""
        now = self._clock()
        crc = int(crc)
        with self._lock:
            if crc in self._own_set:
                self._own_times.append(now)
                self._trim(now)
                return
            self._foreign_times.append(now)
            self._last_foreign = {"t": now, "weight": weight, "kp_max": kp_max}
            self._trim(now)

    def _trim(self, now: float) -> None:
        for dq in (self._own_times, self._foreign_times):
            while dq and now - dq[0] > WINDOW_S:
                dq.popleft()

    # ------------------------------------------------------------ 判定
    def _foreign_since(self, t0: float) -> int:
        return sum(1 for t in self._foreign_times if t >= t0)

    def report(self) -> Dict:
        now = self._clock()
        with self._lock:
            self._trim(now)
            own_hz = len(self._own_times) / WINDOW_S
            foreign_n = len(self._foreign_times)
            last = dict(self._last_foreign) if self._last_foreign else None
        busy = foreign_n >= FOREIGN_MIN_MSGS
        out = {
            "topic": self.topic,
            "own_hz": round(own_hz, 1),
            "foreign_hz": round(foreign_n / WINDOW_S, 1),
            "foreign": busy,
            "listening_s": round(now - self._t_start, 1),
            "last_foreign_age_s": None if last is None else round(now - last["t"], 1),
            "last_weight": None if last is None else last["weight"],
            "last_kp_max": None if last is None else last["kp_max"],
        }
        if busy:
            out["local_hints"] = self.local_hints()
        return out

    def check_free(self, *, fresh: bool = False, listen_s: float = LISTEN_S,
                   sleep: Callable[[float], None] = time.sleep) -> None:
        """接管前调用：听到别人的指令就抛 PermissionErrorMotion。

        fresh=True：从现在起重新听 listen_s（例如刚退出运控，要看退出之后还有没有人发）。
        否则用已经听到的最近 listen_s；服务刚启动、还没听够就先补足。
        """
        now = self._clock()
        if fresh:
            t0 = now
            sleep(listen_s)
        else:
            t0 = now - listen_s
            short = self._t_start + listen_s - now
            if short > 0:
                sleep(short)
        with self._lock:
            self._trim(self._clock())
            n = self._foreign_since(t0)
            last = dict(self._last_foreign) if self._last_foreign else None
        if n < FOREIGN_MIN_MSGS:
            return
        dur = max(1e-3, self._clock() - t0)
        parts = [f"已有别的程序在发 {self.topic}（约 {n / dur:.0f} Hz"]
        if last and last.get("kp_max") is not None:
            parts.append(f"，kp 最大 {last['kp_max']:.0f}")
        if last and last.get("weight") is not None:
            parts.append(f"，权重 {last['weight']:.2f}")
        parts.append("），两个程序同时发令会让手臂发抖或卸不了力，不接管。先停掉它")
        hints = self.local_hints()
        if hints:
            parts.append("。本机疑似：" + "；".join(f"{h['pid']} {h['cmd']}" for h in hints[:3]))
        else:
            parts.append("（本机没找到疑似进程，可能在别的电脑上）")
        raise PermissionErrorMotion("".join(parts))

    # ------------------------------------------------------------ 本机疑似进程
    def local_hints(self) -> List[Dict]:
        now = self._clock()
        t, cached = self._hint_cache
        if now - t < HINT_CACHE_S:
            return cached
        found = scan_processes(self.hints)
        self._hint_cache = (now, found)
        return found


def _own_pids() -> set:
    pids = {os.getpid()}
    try:
        pids.add(os.getppid())
    except OSError:
        pass
    return pids


def scan_processes(patterns: Sequence[str], proc: Path = Path("/proc")) -> List[Dict]:
    """本机上命令行含任一关键词的进程（只是提示：发布者也可能在别的电脑上）。

    只看参数部分（不含解释器路径，否则 .../envs/teleop/bin/python 会让所有进程都命中），
    并跳过本服务自己（含雷达子进程）。非 Linux 返回空。
    """
    if not patterns or not sys.platform.startswith("linux") or not proc.is_dir():
        return []
    mine = _own_pids()
    out = []
    for d in proc.iterdir():
        if not d.name.isdigit() or int(d.name) in mine:
            continue
        try:
            argv = (d / "cmdline").read_bytes().split(b"\0")
        except OSError:
            continue
        args = " ".join(a.decode(errors="replace") for a in argv[1:] if a)
        if not args or "robot_platform" in args or "serve.py" in args:
            continue
        if any(p in args for p in patterns):
            exe = argv[0].decode(errors="replace").rsplit("/", 1)[-1]
            out.append({"pid": int(d.name), "cmd": (exe + " " + args)[:160]})
    return out[:10]
