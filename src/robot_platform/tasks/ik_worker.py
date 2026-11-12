"""在独立子进程里跑耗时的 IK（多初值姿态 IK）。

为什么：多初值 IK 是纯 Python + 小 numpy 运算，一次几百 ms 到几秒，在服务主进程里算会一直占着 GIL，
同进程 100 Hz 控制循环（两只手共用）发不出帧，手臂会颤（2026-11-11 实测：G1D 上够不着的目标
算 8.7 s，期间 10 ms 周期线程最长 1.9 s 才轮到一次；放进子进程后最长 10.6 ms）。

- 用 spawn 起子进程：主进程里有 DDS 线程，fork 出来的子进程可能卡在它们留下的锁上；
- 只开一个常驻子进程，第一次用时起（或服务启动时 :func:`warm` 预热），之后复用；
- 子进程起不来 / 崩了：退回本进程算（会颤，但功能不丢），并记下原因；下次再试着重起。
- 环境变量 RDP_IK_INPROCESS=1：一律本进程算（调试用）。
"""

from __future__ import annotations

import multiprocessing as mp
import os
import pickle
import threading
from concurrent.futures import ProcessPoolExecutor
from concurrent.futures import TimeoutError as FuturesTimeout
from concurrent.futures.process import BrokenProcessPool
from typing import Any, Callable, Optional

# 结果最多等这么久（预算 + 子进程冷启动）；超时则本进程算
RESULT_TIMEOUT_S = 30.0

_lock = threading.Lock()
_pool: Optional[ProcessPoolExecutor] = None
last_error = ""


def _ping() -> int:
    return os.getpid()


def _exit_with_parent() -> None:
    """子进程初始化：父进程（服务）一退出就跟着退出。

    服务被 SIGTERM / 硬杀时来不及收拾进程池，不这样子进程会成为孤儿一直留着（每重启一次多一个）。
    """
    parent = mp.parent_process()
    if parent is None:
        return

    def _watch():
        parent.join()  # 等父进程的 sentinel：父进程没了就返回
        os._exit(0)

    threading.Thread(target=_watch, name="ik-worker-parent-watch", daemon=True).start()


def _get_pool() -> ProcessPoolExecutor:
    global _pool
    with _lock:
        if _pool is None:
            _pool = ProcessPoolExecutor(max_workers=1, mp_context=mp.get_context("spawn"),
                                        initializer=_exit_with_parent)
        return _pool


def _reset_pool() -> None:
    global _pool
    with _lock:
        pool, _pool = _pool, None
    if pool is not None:
        pool.shutdown(wait=False, cancel_futures=True)


def warm() -> None:
    """服务启动时调：后台把子进程先起好（numpy 等导入要 1～2 s），第一次走点不用等。"""
    if os.environ.get("RDP_IK_INPROCESS") == "1":
        return

    def _go():
        global last_error
        try:
            _get_pool().submit(_ping).result(timeout=RESULT_TIMEOUT_S)
        except Exception as e:  # noqa: BLE001
            last_error = f"IK 子进程预热失败：{type(e).__name__}: {e}"

    threading.Thread(target=_go, name="ik-worker-warm", daemon=True).start()


def worker_pid() -> Optional[int]:
    """子进程 pid（测试 / 排查用）；起不来返回 None。"""
    try:
        return int(_get_pool().submit(_ping).result(timeout=RESULT_TIMEOUT_S))
    except Exception:  # noqa: BLE001
        return None


def run(fn: Callable[..., Any], *args, **kwargs) -> Any:
    """在子进程里调 fn(*args, **kwargs)（fn 须是模块级函数、参数可 pickle）。

    子进程起不来 / 崩了 / 超时 / 参数传不过去：本进程调。fn 自己抛的错照常往上抛。
    """
    global last_error
    if os.environ.get("RDP_IK_INPROCESS") != "1":
        try:
            out = _get_pool().submit(fn, *args, **kwargs).result(timeout=RESULT_TIMEOUT_S)
            last_error = ""
            return out
        except (BrokenProcessPool, FuturesTimeout, pickle.PicklingError, OSError) as e:
            last_error = f"IK 子进程不可用，本次在主进程算：{type(e).__name__}: {e}"
            _reset_pool()
    return fn(*args, **kwargs)
