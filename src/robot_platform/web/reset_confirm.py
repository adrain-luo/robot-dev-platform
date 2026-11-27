"""网页上的复位要人工确认（负责人 2026-11-26：执行重置操作需人工确认）。

页面的两个复位都分两步：

1. ``POST /api/estop/reset/request``（守护层是 ``/api/guardian/reset/request``）：网页进程发一个
   一次性令牌，``TTL_S`` 秒内有效，记下发令牌那一刻的锁存（运控核心的 epoch 和锁存原因），
   并把锁存原因交给页面，弹窗里给人看；
2. 人在弹窗里确认后，页面带着令牌 ``POST /api/estop/reset``（``/api/guardian/reset``）。

一律拒绝、要重新确认的情况：没带令牌；令牌不认识、用过了、过期了、是另一个复位的；发令牌之后
锁存变了（又按了急停、接管、释放，运控核心的 epoch 都会变）。令牌只能用一次，复位失败也作废。

挡得住：一个 POST 就复位（脚本、浏览器重试、开着的旧页面）；人确认的不是眼前这一次锁存。
挡不住：有人故意写脚本先要令牌再复位（网页没有鉴权）。
不管：上位机经 ROS 发的 RESUME_HOME / RECOVER 走运控核心的客户端接口，负责人认定那就算人工确认
（2026-11-26），不经这里。
"""

from __future__ import annotations

import secrets
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass
from typing import Any, Callable, Dict, Hashable, Optional, Tuple

from robot_platform.control.arm_loop import PermissionErrorMotion

TTL_S = 30.0       # 令牌有效期：弹窗开着时人要看一圈现场再点确认；锁存变了另有拒绝，这里只防令牌放久了
MAX_PENDING = 8    # 同时有效的令牌上限（多开几个页面也够），超了丢最早的
KINDS = ("estop", "guardian")


@dataclass(frozen=True)
class _Pending:
    kind: str
    fingerprint: Hashable
    expires: float


class ResetConfirm:
    """发令牌、收令牌。线程安全（网页每个请求一个线程）。clock 可换成假时钟测试。"""

    def __init__(self, *, clock: Callable[[], float] = time.monotonic,
                 ttl_s: float = TTL_S, max_pending: int = MAX_PENDING) -> None:
        self._clock = clock
        self.ttl_s = float(ttl_s)
        self._max = int(max_pending)
        self._lock = threading.Lock()
        self._pending: "OrderedDict[str, _Pending]" = OrderedDict()

    def issue(self, kind: str, fingerprint: Hashable) -> Tuple[str, float]:
        """发一个令牌，记下这个复位对应的锁存。返回 (令牌, 有效秒数)。"""
        if kind not in KINDS:
            raise ValueError(f"没有 {kind!r} 这种复位")
        now = self._clock()
        token = secrets.token_urlsafe(16)
        with self._lock:
            self._drop_expired(now)
            while len(self._pending) >= self._max:
                self._pending.popitem(last=False)
            self._pending[token] = _Pending(kind, fingerprint, now + self.ttl_s)
        return token, self.ttl_s

    def redeem(self, kind: str, token: Any, fingerprint: Hashable) -> None:
        """核对令牌，对上了才返回；对不上抛 PermissionErrorMotion（网页回 403）。令牌无论对错都作废。"""
        if not token:
            raise PermissionErrorMotion("复位要先在页面上确认：请点「复位急停」或「接管」，在弹窗里确认")
        now = self._clock()
        with self._lock:
            rec = self._pending.pop(str(token), None)
            self._drop_expired(now)
        if rec is None:
            raise PermissionErrorMotion("复位确认无效（已用过或不是这台页面发的）：请重新点复位并确认")
        if rec.kind != kind:
            raise PermissionErrorMotion("这个确认不是给这个复位的：请重新点复位并确认")
        if now > rec.expires:
            raise PermissionErrorMotion(f"复位确认超过 {self.ttl_s:.0f} 秒已失效：请重新点复位并确认")
        if rec.fingerprint != fingerprint:
            raise PermissionErrorMotion("确认之后锁存变了（又按了急停、接管或释放）：请看清现在的状态，重新点复位并确认")

    def pending(self) -> int:
        with self._lock:
            self._drop_expired(self._clock())
            return len(self._pending)

    def _drop_expired(self, now: float) -> None:
        """调用方持有 _lock。"""
        for tok in [t for t, p in self._pending.items() if now > p.expires]:
            del self._pending[tok]


def estop_fingerprint(status: Dict[str, Any]) -> Tuple[Optional[int], Optional[str]]:
    """运控核心的急停锁存：epoch（急停、接管、释放都会加一）加锁存原因。status 是 core.status()。"""
    return status.get("epoch"), status.get("latch_reason")
