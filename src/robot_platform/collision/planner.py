"""绕行规划：直线会碰时，在这只手的关节空间里找一条不碰的折线路径（RRT-Connect + 抄近路）。

- 每段折线都是关节空间直线，用闸门同一套保守推进检查（CollisionGuard._sweep），
  下限也和闸门一样按「整次规划的起点」算一次，路上不会一段段放宽；
- 只动这一只手，另一只手按它的指令角 / 目标角 / 剩余中间点各查一遍（和闸门一致）；
- 找到后反复随机抄近路（两点直连不碰就删掉中间的点），最后通常只剩一两个中间点；
- 有时间上限，超时就放弃（按原样拦下）。
"""

from __future__ import annotations

import math
import time
from typing import List, Optional

import numpy as np

STEP_RAD = 0.35  # 树每次往外长的最大关节步（L2，rad）
LIMIT_PAD_RAD = 0.02  # 采样离关节限位留一点
DEFAULT_BUDGET_S = 4.0
SHORTCUT_ROUNDS = 20

PLANNERS = {"builtin": "内置 RRT-Connect（Python）", "ompl": "OMPL RRTConnect（C++ 搜索树）"}


def planner_available(name: str) -> bool:
    if name == "builtin":
        return True
    if name == "ompl":
        try:
            import ompl.base  # noqa: F401
            import ompl.geometric  # noqa: F401
        except Exception:  # noqa: BLE001
            return False
        return True
    return False


class _Tree:
    def __init__(self, root: np.ndarray) -> None:
        self.q = [root]
        self.parent = [-1]

    def nearest(self, x: np.ndarray) -> int:
        d = np.linalg.norm(np.asarray(self.q) - x, axis=1)
        return int(np.argmin(d))

    def add(self, q: np.ndarray, parent: int) -> int:
        self.q.append(q)
        self.parent.append(parent)
        return len(self.q) - 1

    def path_to_root(self, i: int) -> List[np.ndarray]:
        out = []
        while i >= 0:
            out.append(self.q[i])
            i = self.parent[i]
        return out


def plan(edge_ok, q0: np.ndarray, q1: np.ndarray, limits: np.ndarray,
         budget_s: float = DEFAULT_BUDGET_S, seed: Optional[int] = None, direct_known_bad: bool = False) -> dict:
    """edge_ok(a, b) → 关节直线 a→b 不碰。返回 {"path": [中间点..., q1] 或 None, "nodes", "ms", "reason"}。

    path 不含起点；None 表示时间内没找到。direct_known_bad：已知直线不通（闸门刚查过），不再查。
    """
    t0 = time.perf_counter()
    rng = np.random.default_rng(seed)
    lo = limits[:, 0] + LIMIT_PAD_RAD
    hi = limits[:, 1] - LIMIT_PAD_RAD
    lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    n_edge = [0]

    def ok(a, b):
        n_edge[0] += 1
        return edge_ok(a, b)

    def done(path, reason=""):
        return {"path": path, "nodes": len(ta.q) + len(tb.q), "edges": n_edge[0],
                "ms": (time.perf_counter() - t0) * 1000.0, "reason": reason}

    ta, tb = _Tree(q0), _Tree(q1)
    if not direct_known_bad and ok(q0, q1):
        return done([q1])

    def extend(tree: _Tree, x: np.ndarray):
        """往 x 长一步；返回 (新节点下标或 None, 是否到了 x)。"""
        i = tree.nearest(x)
        base = tree.q[i]
        d = x - base
        n = float(np.linalg.norm(d))
        reached = n <= STEP_RAD
        y = x if reached else base + d * (STEP_RAD / n)
        if not ok(base, y):
            return None, False
        return tree.add(y, i), reached

    def connect(tree: _Tree, x: np.ndarray):
        """一直往 x 长，直到到达或被挡。"""
        while True:
            j, reached = extend(tree, x)
            if j is None:
                return None
            if reached:
                return j

    found = None
    while time.perf_counter() - t0 < budget_s:
        x = rng.uniform(lo, hi)
        i, _ = extend(ta, x)
        if i is not None:
            j = connect(tb, ta.q[i])
            if j is not None:
                found = (ta, i, tb, j)
                break
        ta, tb = tb, ta
    if found is None:
        return done(None, f"{budget_s:.0f} s 内没找到不碰的绕行路线")
    t_a, i, t_b, j = found
    pa = t_a.path_to_root(i)[::-1]
    pb = t_b.path_to_root(j)[1:]  # 去掉重复的连接点
    path = pa + pb
    if not np.allclose(path[0], q0):
        path = path[::-1]
    path = shortcut(ok, path, rng, deadline=t0 + budget_s + 2.0)
    return done(path[1:])


def shortcut(ok, path: List[np.ndarray], rng, rounds: int = SHORTCUT_ROUNDS,
             deadline: float = math.inf) -> List[np.ndarray]:
    """抄近路，首尾不动。先贪心（从当前点直连能到的最远点，一般收成一两个中间点）；
    还剩不止一个中间点时，再随机挑两点试直连。每次试都要查一整段长线（贵），所以次数要少。"""
    path = list(path)
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and (time.perf_counter() > deadline or not ok(path[i], path[j])):
            j -= 1
        out.append(path[j])
        i = j
    path = out
    for _ in range(rounds):
        if len(path) <= 3 or time.perf_counter() > deadline:
            break
        i, j = sorted(rng.choice(len(path), size=2, replace=False))
        if j - i < 2:
            continue
        if ok(path[i], path[j]):
            path = path[: i + 1] + path[j:]
    return path


def plan_ompl(edge_ok, point_ok, q0: np.ndarray, q1: np.ndarray, limits: np.ndarray,
              budget_s: float = DEFAULT_BUDGET_S, seed: Optional[int] = None) -> dict:
    """同 plan()，用 OMPL 的 RRTConnect：采样 / 树 / 最近邻在 C++ 里，碰撞仍按 edge_ok（保守推进整段查）。
    找到后按 shortcut() 抄近路。"""
    from ompl import base as ob, geometric as og, util as ou

    try:
        ou.setLogLevel(ou.LogLevel.LOG_WARN)
    except Exception:  # noqa: BLE001
        pass
    t0 = time.perf_counter()
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    n = q0.size
    lo = limits[:, 0] + LIMIT_PAD_RAD
    hi = limits[:, 1] - LIMIT_PAD_RAD
    lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)
    lo, hi = np.minimum(lo, np.minimum(q0, q1)), np.maximum(hi, np.maximum(q0, q1))
    sp = ob.RealVectorStateSpace(n)
    b = ob.RealVectorBounds(n)
    for i in range(n):
        b.setLow(i, float(lo[i]))
        b.setHigh(i, float(hi[i]))
    sp.setBounds(b)
    ss = og.SimpleSetup(sp)
    n_edge = [0]

    def vec(s):
        return np.array([s[i] for i in range(n)], dtype=float)

    def edge(a, c):
        n_edge[0] += 1
        return bool(edge_ok(a, c))

    ss.setStateValidityChecker(lambda s: bool(point_ok(vec(s))))
    si = ss.getSpaceInformation()

    class _Motion(ob.MotionValidator):
        def checkMotion(self, s1, s2, *rest):
            return edge(vec(s1), vec(s2))

    mv = _Motion(si)
    si.setMotionValidator(mv)
    start, goal = sp.allocState(), sp.allocState()
    for i in range(n):
        start[i] = float(q0[i])
        goal[i] = float(q1[i])
    ss.setStartAndGoalStates(start, goal)
    pl = og.RRTConnect(si)
    pl.setRange(STEP_RAD)
    ss.setPlanner(pl)
    solved = ss.solve(float(budget_s))
    out = {"nodes": None, "reason": "", "path": None}
    if solved and ss.haveExactSolutionPath():
        # 不用 OMPL 自带的 simplifySolution：它还做 B 样条平滑，会加很多点、每点都要查，慢；用同一套抄近路
        p = ss.getSolutionPath()
        path = [vec(p.getState(i)) for i in range(p.getStateCount())]
        path[0], path[-1] = q0, q1
        path = shortcut(edge, path, np.random.default_rng(seed), deadline=t0 + budget_s + 2.0)
        out["path"] = path[1:]
    else:
        out["reason"] = f"{budget_s:.0f} s 内没找到不碰的绕行路线"
    out["edges"] = n_edge[0]
    out["ms"] = (time.perf_counter() - t0) * 1000.0
    return out
