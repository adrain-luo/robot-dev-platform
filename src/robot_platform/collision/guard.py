"""手臂运动前的自碰撞检查（闸门）：给一只手下新目标时，沿关节空间直线检查整段路径。

- 只查「这只手」相关的连杆对：它对身体、对另一只手、它自己隔得远的几节；
- 另一只手按它的当前指令角和目标角各查一遍（它可能正在走）；
- 起点就已经在余量以内的连杆对（模型偏保守、或本来就贴得近）不拦，只要路上不比起点更近 2 mm
  （统一写成：每对的下限 = min(余量, 起点距离 − 2 mm)）；
- 步长用保守推进：离最近障碍还剩 d，手臂上任一点的位移 ≤ 力臂 × Σ|Δq|，所以按 d 能安全走多远就跳多远，
  不会穿过薄的东西，离得远时只查几次。
"""

from __future__ import annotations

import itertools
import json
import math
import threading
import time
from typing import Callable, Dict, List, Mapping, Optional, Sequence

import numpy as np

from robot_platform.collision.model import CollisionModel
from robot_platform.core import instance as instances
from robot_platform.hw.backends.base import PermissionErrorMotion

DEFAULT_MARGIN_M = 0.005
DEEPER_TOL_M = 0.002
MIN_STEP_M = 0.002  # 保守推进最小步（末端位移），免得贴着走时步子无限小
NEAR_M = 0.03  # 闸门细算的范围：之外按包围球下界

_PART = {
    "shoulder_pitch": "肩（俯仰）", "shoulder_roll": "肩（横滚）", "shoulder_yaw": "上臂",
    "elbow": "肘 / 前臂", "wrist_roll": "腕（横滚）", "wrist_pitch": "腕（俯仰）", "wrist_yaw": "手腕 / 末端",
}
_BODY = {
    "torso_link": "躯干", "head_link": "头", "logo_link": "胸前", "AGV_link": "底盘",
    "Left_Wheel_Link": "左轮", "RIght_Wheel_Link": "右轮", "Right_Wheel_Link": "右轮",
    "LZ_ot_Link": "立柱", "LZ_mt_Link": "立柱", "LZ_it_Link": "立柱", "Pitching_Link": "腰", "Yaw_Link": "腰",
}


def link_label(link: str) -> str:
    if link in _BODY:
        return _BODY[link]
    side = "左" if link.startswith("left_") else "右" if link.startswith("right_") else ""
    for k, v in _PART.items():
        if k in link:
            return f"{side}{v}"
    return link


class CollisionBlocked(PermissionErrorMotion):
    """路径上会撞：不下发这个目标。"""


class CollisionGuard:
    def __init__(
        self,
        model: CollisionModel,
        arm_joints: Mapping[str, Sequence[str]],
        body_q: Callable[[], Dict[str, float]],
        source: Optional[tuple] = None,
    ) -> None:
        """source=(urdf 路径, base_link)：给了就在 IK 子进程里查（见 check_move），不占主进程 GIL。"""
        self.model = model
        self._source = source
        self.arm_joints = {k: list(v) for k, v in arm_joints.items()}
        self._body_q = body_q
        self._lock = threading.Lock()
        self.enabled = True
        self.margin_m = DEFAULT_MARGIN_M
        self.last_block: Optional[dict] = None
        self.last_check: Optional[dict] = None
        # 每只手相关的连杆对（下标）
        arm_of = {l: k for k, ls in model.arm_links.items() for l in ls}
        self._arm_pairs = {
            arm: np.array([i for i, (a, b) in enumerate(model.pairs) if arm_of.get(a) == arm or arm_of.get(b) == arm],
                          dtype=int)
            for arm in self.arm_joints
        }
        # 力臂表：lever[arm][j, k] = 关节 j 转 1 rad 时，这只手第 k 节连杆表面最多移动多少（m）
        self._lever = {arm: self._lever_table(arm) for arm in self.arm_joints}
        # 每只手的每个连杆对：两端各是这只手的第几节（不是这只手的记为 n，对应全 0 列）
        self._pair_k = {}
        for arm, idx in self._arm_pairs.items():
            links = self.model.arm_links.get(arm, [])
            n = len(links)
            ka = np.array([links.index(model.pairs[i][0]) if model.pairs[i][0] in links else n for i in idx], dtype=int)
            kb = np.array([links.index(model.pairs[i][1]) if model.pairs[i][1] in links else n for i in idx], dtype=int)
            self._pair_k[arm] = (ka, kb)
        self._load_config()

    # ------------------------------------------------------------ 配置
    def _load_config(self) -> None:
        try:
            p = instances.path("collision_config")
            if p.is_file():
                d = json.loads(p.read_text(encoding="utf-8"))
                self.enabled = bool(d.get("enabled", True))
                self.margin_m = float(np.clip(float(d.get("margin_m", DEFAULT_MARGIN_M)), 0.0, 0.1))
        except (Exception, SystemExit):  # noqa: BLE001  没有实例（单测 / 工具里）就用默认值
            pass

    def configure(self, enabled: Optional[bool] = None, margin_m: Optional[float] = None) -> dict:
        with self._lock:
            if enabled is not None:
                self.enabled = bool(enabled)
            if margin_m is not None:
                v = float(margin_m)
                if not math.isfinite(v):
                    raise ValueError("余量要是有限数")
                self.margin_m = float(np.clip(v, 0.0, 0.1))
        try:
            p = instances.path("collision_config")
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(json.dumps({"enabled": self.enabled, "margin_m": self.margin_m}, indent=2) + "\n",
                         encoding="utf-8")
        except (OSError, SystemExit):
            pass
        return self.config()

    def config(self) -> dict:
        return {"enabled": self.enabled, "margin_m": self.margin_m, "margin_mm": self.margin_m * 1000.0}

    # ------------------------------------------------------------ 工具
    def _lever_table(self, arm: str, samples: int = 200) -> np.ndarray:
        """(关节数, 连杆数 + 1) 的力臂表：随机采样姿态，取关节 j 的转轴原点到第 k 节（k ≥ j）图元表面的最远距离，
        再放大 10%。最后一列全 0（给「不是这只手的连杆」用）。保守推进按它算步长：靠近肩的连杆力臂小，
        那里离得近（如肩 ↔ 头）也不会把整段的步子拖小。"""
        joints = self.arm_joints[arm]
        links = self.model.arm_links.get(arm, [])
        n_j, n_l = len(joints), len(links)
        lev = np.zeros((n_j, n_l + 1))
        rng = np.random.default_rng(0)
        jobj = [self.model.tree.joints[j] for j in joints]
        prim_ids = [np.nonzero(self.model.owner == self.model.prim_links.index(l))[0] if l in self.model.prim_links
                    else np.zeros(0, dtype=int) for l in links]
        for s_i in range(samples):
            q = {j: float(rng.uniform(-math.pi, math.pi)) for j in joints} if s_i else {}
            T = self.model.tree.transforms(q)
            origins = [(T[jo.parent] @ jo.T0)[:3, 3] for jo in jobj]
            for k, l in enumerate(links):
                ids = prim_ids[k]
                if ids.size == 0 or l not in T:
                    continue
                R, t = T[l][:3, :3], T[l][:3, 3]
                P = np.r_[self.model.A[ids] @ R.T + t, self.model.B[ids] @ R.T + t]
                rr = np.r_[self.model.R[ids], self.model.R[ids]]
                for j in range(min(k + 1, n_j)):
                    lev[j, k] = max(lev[j, k], float((np.linalg.norm(P - origins[j], axis=1) + rr).max()))
        return lev * 1.1

    def _q(self, base: Dict[str, float], arms: Mapping[str, np.ndarray]) -> Dict[str, float]:
        q = dict(base)
        for arm, v in arms.items():
            q.update(zip(self.arm_joints[arm], [float(x) for x in np.asarray(v).reshape(-1)]))
        return q

    def _pair_text(self, k: int) -> str:
        a, b = self.model.pairs[k]
        return f"{link_label(a)} ↔ {link_label(b)}"

    # ------------------------------------------------------------ 闸门
    def check_move(
        self,
        arm: str,
        q_from: Sequence[float],
        q_to: Sequence[float],
        others: Mapping[str, Sequence[Sequence[float]]],
    ) -> dict:
        """查 arm 从 q_from 直线走到 q_to 的路径；会撞就抛 CollisionBlocked。返回检查摘要。

        纯 Python 循环，一次几十到几百 ms：有 source 时放进 IK 子进程算（tasks/ik_worker.py），
        否则同进程里 100 Hz 控制循环会被 GIL 卡住、手臂发颤。
        """
        if not self.enabled or arm not in self._arm_pairs:
            return {"checked": False}
        t0 = time.perf_counter()
        args = (arm, np.asarray(q_from, dtype=float).reshape(-1), np.asarray(q_to, dtype=float).reshape(-1),
                {k: [np.asarray(v, dtype=float) for v in vs] for k, vs in others.items()},
                self._body_q(), float(self.margin_m))
        if self._source is not None:
            from robot_platform.tasks import ik_worker

            res = ik_worker.run(_remote_check, self._source, self.arm_joints, *args)
        else:
            res = self._check_core(*args)
        res["ms"] = (time.perf_counter() - t0) * 1000.0
        blk = res.pop("block", None)
        if blk is not None:
            blk["time"] = time.strftime("%H:%M:%S")
            with self._lock:
                self.last_block = blk
                self.last_check = res
            dk = blk["distance_mm"] / 1000.0
            side = "左" if arm == "left" else "右"
            t = blk["at"]
            where = "目标姿态" if t >= 1.0 else f"走到 {t * 100:.0f}% 处"
            raise CollisionBlocked(
                f"碰撞检查拦下：{side}手{where}会{'撞上' if dk < 0 else '贴近'}"
                f"（{blk['pair_text']}，{'穿透' if dk < 0 else '间距'} {abs(dk) * 1000:.1f} mm，"
                f"余量 {blk['margin_mm']:.0f} mm）。换个姿态 / 路径，或在「碰撞检查」里调余量。"
            )
        with self._lock:
            self.last_check = res
        return res

    def _check_core(self, arm, q0, q1, others, base, margin) -> dict:
        """路径检查本体（可在子进程里跑）：不改状态、不抛错，结果里带 block 表示会撞。"""
        idx = self._arm_pairs[arm]
        D = q1 - q0
        l1 = float(np.abs(D).sum())
        names = [k for k in others if k != arm and k in self.arm_joints]
        variants = []

        def _uniq(vs):
            out = []
            for v in vs:
                v = np.asarray(v, dtype=float)
                if not any(np.allclose(v, u, atol=1e-4) for u in out):
                    out.append(v)
            return out

        for combo in itertools.product(*[_uniq(others[k]) or [None] for k in names]):
            variants.append({k: v for k, v in zip(names, combo) if v is not None})
        if not variants:
            variants = [{}]
        # 每对的下限：平时是余量；起点就已比「余量 + 2 mm」近的，只要求不比起点更近 2 mm
        thresh = [np.minimum(margin, self.model.evaluate(self._q(base, {**o, arm: q0}), NEAR_M, idx) - DEEPER_TOL_M)
                  for o in variants]
        lev = self._lever[arm]
        ka, kb = self._pair_k[arm]
        absD = np.abs(D)
        # 每对的「表面最大移动速度」（m / 单位路径参数）：两端各自按力臂表算，都在动就相加
        rate = absD @ lev[:, ka] + absD @ lev[:, kb]
        rate = np.maximum(rate, 1e-9)
        t = 0.0
        n_eval = 0
        closest = math.inf
        while True:
            q = q0 + t * D
            free = math.inf
            for vi, o in enumerate(variants):
                d = self.model.evaluate(self._q(base, {**o, arm: q}), NEAR_M, idx)
                n_eval += 1
                bad = np.nonzero(d < thresh[vi])[0]
                if bad.size:
                    k = int(idx[bad[np.argmin(d[bad])]])
                    dk = float(d[bad].min())
                    return {
                        "checked": True, "arm": arm, "evals": n_eval,
                        "block": {"arm": arm, "pair": list(self.model.pairs[k]), "pair_text": self._pair_text(k),
                                  "distance_mm": dk * 1000.0, "at": t, "margin_mm": margin * 1000.0},
                    }
                if d.size:
                    # 每对能安全走多远（路径参数）取最小
                    free = min(free, float((np.maximum(d - thresh[vi], MIN_STEP_M) / rate).min()))
                    closest = min(closest, float(d.min()))
            if t >= 1.0 or l1 < 1e-9:
                break
            t = min(1.0, t + free)
        return {"checked": True, "arm": arm, "evals": n_eval,
                "closest_mm": None if not math.isfinite(closest) else closest * 1000.0}

    # ------------------------------------------------------------ 状态
    def status(self, arms_q: Optional[Mapping[str, Sequence[float]]] = None) -> dict:
        out = {**self.config(), "available": True, "last_block": self.last_block, "last_check": self.last_check,
               "pairs": len(self.model.pairs), "prims": int(len(self.model.R))}
        if arms_q:
            q = self._q(self._body_q(), {k: np.asarray(v, dtype=float) for k, v in arms_q.items() if k in self.arm_joints})
            if self._source is not None:
                from robot_platform.tasks import ik_worker

                d = ik_worker.run(_remote_eval, self._source, self.arm_joints, q, 0.15)
            else:
                d = self.model.evaluate(q, 0.15)
            if d.size:
                k = int(np.argmin(d))
                out["nearest"] = {"pair": list(self.model.pairs[k]), "pair_text": self._pair_text(k),
                                  "distance_mm": float(d[k]) * 1000.0, "capped": bool(d[k] >= 0.15 - 1e-9)}
                per_arm = {}
                for arm, idx in self._arm_pairs.items():
                    if idx.size:
                        j = int(idx[np.argmin(d[idx])])
                        per_arm[arm] = {"pair_text": self._pair_text(j), "distance_mm": float(d[j]) * 1000.0}
                out["per_arm"] = per_arm
                # 比余量近的全部连杆对（三维里标红）
                close = np.nonzero(d < self.margin_m)[0]
                out["close"] = [{"pair": list(self.model.pairs[j]), "pair_text": self._pair_text(int(j)),
                                 "distance_mm": float(d[j]) * 1000.0} for j in close[np.argsort(d[close])]]
        return out

    def geometry(self) -> dict:
        """给三维显示：各连杆系下的图元。"""
        links: Dict[str, List[list]] = {}
        for i, li in enumerate(self.model.owner):
            links.setdefault(self.model.prim_links[li], []).append(
                [*np.round(self.model.A[i], 4).tolist(), *np.round(self.model.B[i], 4).tolist(), round(float(self.model.R[i]), 4)]
            )
        return {"links": links}


# ---------------------------------------------------------------- 子进程
_REMOTE: Dict[tuple, CollisionGuard] = {}


def _remote_guard(source, arm_joints) -> Optional[CollisionGuard]:
    """按 (urdf, base_link, collision.json 修改时间) 缓存一个闸门（子进程里用）。"""
    from robot_platform.collision.model import default_spec_path, try_load

    urdf, base_link = source
    spec = default_spec_path(urdf)
    key = (str(urdf), base_link, spec.stat().st_mtime if spec.is_file() else 0.0)
    g = _REMOTE.get(key)
    if g is None:
        model = try_load(urdf, base_link)
        if model is None:
            return None
        _REMOTE.clear()
        g = _REMOTE[key] = CollisionGuard(model, arm_joints, lambda: {})
    return g


def _remote_check(source, arm_joints, arm, q0, q1, others, base, margin) -> dict:
    g = _remote_guard(source, arm_joints)
    return {"checked": False} if g is None else g._check_core(arm, q0, q1, others, base, margin)


def _remote_eval(source, arm_joints, q, near) -> np.ndarray:
    g = _remote_guard(source, arm_joints)
    return np.zeros(0) if g is None else g.model.evaluate(q, near)
