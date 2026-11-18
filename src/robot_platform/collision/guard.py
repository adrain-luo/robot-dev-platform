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

from robot_platform.collision.model import BACKENDS, CollisionModel, backend_available, build_model
from robot_platform.core import instance as instances
from robot_platform.hw.backends.base import PermissionErrorMotion

DEFAULT_MARGIN_M = 0.005
DEEPER_TOL_M = 0.002
MIN_STEP_M = 0.002  # 保守推进最小步（末端位移），免得贴着走时步子无限小
NEAR_M = 0.03  # 闸门细算的范围：之外按包围球下界
COARSE_M = 0.01  # 绕行搜索时下限加厚 / 最小步（见 _plan_core）

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
        tools_fn: Optional[Callable[[], tuple]] = None,
        load_config: bool = True,
    ) -> None:
        """source=(urdf 路径, base_link)：给了就在 IK 子进程里查（见 check_move），不占主进程 GIL。
        tools_fn() → ((腕末端连杆, TCP 点), ...)：当前生效的 TCP，末端胶囊体跟着它走（变了就重建模型）。"""
        self._source = source
        self.arm_joints = {k: list(v) for k, v in arm_joints.items()}
        self._body_q = body_q
        self._tools_fn = tools_fn
        self._lock = threading.Lock()
        self._model_lock = threading.Lock()
        self.enabled = True
        self.margin_m = DEFAULT_MARGIN_M
        self.detour = True  # 直线会碰时自动规划绕行（只对「到位」类目标，见 ArmLoop.set_target(detour=True)）
        # 碰撞距离用哪个后端（见 model.BACKENDS）：装了 coal 默认用它（快约 10 倍），没装用内置
        self.backend = "coal" if backend_available("coal") else "builtin"
        self.planner = "builtin"  # 绕行用哪个规划器（见 planner.PLANNERS）
        self.last_plan: Optional[dict] = None
        self.last_block: Optional[dict] = None
        self.last_check: Optional[dict] = None
        self._set_model(model)
        if load_config:  # 子进程里不读实例配置：后端由主进程在 source 里给定，余量等按参数传
            self._load_config()
            self._apply_backend()

    def _set_model(self, model: CollisionModel) -> None:
        self.model = model
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

    def sync_tools(self) -> None:
        """当前生效的 TCP 变了（换了 / 清掉 / 新标定）就按新末端重建模型。"""
        if self._tools_fn is None:
            return
        try:
            tools = tuple((str(l), tuple(round(float(v), 5) for v in p)) for l, p in self._tools_fn())
        except Exception:  # noqa: BLE001  读不到 TCP 就当没有末端
            tools = ()
        if tools == self.model.tools:
            return
        with self._model_lock:
            if tools == self.model.tools:
                return
            m = self.model
            self._set_model(build_model(m.urdf_path, m.spec, m.base_link, tools, m.backend))

    def _apply_backend(self) -> None:
        """模型换成当前选的后端（选的用不了就留在内置）。"""
        want = self.backend if backend_available(self.backend) else "builtin"
        if want == self.model.backend:
            return
        with self._model_lock:
            m = self.model
            if want != m.backend:
                self._set_model(build_model(m.urdf_path, m.spec, m.base_link, m.tools, want))

    def _remote_source(self) -> tuple:
        return (*self._source[:2], self.model.tools, self.model.backend)

    def choices(self) -> dict:
        from robot_platform.collision import planner

        return {
            "backends": [{"id": k, "label": v, "available": backend_available(k)} for k, v in BACKENDS.items()],
            "planners": [{"id": k, "label": v, "available": planner.planner_available(k)}
                         for k, v in planner.PLANNERS.items()],
            "backend_active": self.model.backend,
        }

    def tools_text(self) -> List[dict]:
        return [{"link": l, "side": "left" if l.startswith("left_") else "right", "tcp_mm": [v * 1000.0 for v in p]}
                for l, p in self.model.tools]

    # ------------------------------------------------------------ 配置
    def _load_config(self) -> None:
        try:
            p = instances.path("collision_config")
            if p.is_file():
                d = json.loads(p.read_text(encoding="utf-8"))
                self.enabled = bool(d.get("enabled", True))
                self.margin_m = float(np.clip(float(d.get("margin_m", DEFAULT_MARGIN_M)), 0.0, 0.1))
                self.detour = bool(d.get("detour", True))
                if str(d.get("backend") or "") in BACKENDS:
                    self.backend = str(d["backend"])
                if str(d.get("planner") or "") in ("builtin", "ompl"):
                    self.planner = str(d["planner"])
        except (Exception, SystemExit):  # noqa: BLE001  没有实例（单测 / 工具里）就用默认值
            pass

    def configure(self, enabled: Optional[bool] = None, margin_m: Optional[float] = None,
                  detour: Optional[bool] = None, backend: Optional[str] = None,
                  planner: Optional[str] = None) -> dict:
        from robot_platform.collision import planner as planner_mod

        if backend is not None:
            if backend not in BACKENDS:
                raise ValueError(f"没有这个碰撞后端：{backend}")
            if not backend_available(backend):
                raise ValueError(f"本机没装 {backend}，用不了这个后端")
        if planner is not None:
            if planner not in planner_mod.PLANNERS:
                raise ValueError(f"没有这个规划器：{planner}")
            if not planner_mod.planner_available(planner):
                raise ValueError(f"本机没装 {planner}，用不了这个规划器")
        with self._lock:
            if backend is not None:
                self.backend = backend
            if planner is not None:
                self.planner = planner
            if enabled is not None:
                self.enabled = bool(enabled)
            if detour is not None:
                self.detour = bool(detour)
            if margin_m is not None:
                v = float(margin_m)
                if not math.isfinite(v):
                    raise ValueError("余量要是有限数")
                self.margin_m = float(np.clip(v, 0.0, 0.1))
        try:
            p = instances.path("collision_config")
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(json.dumps({"enabled": self.enabled, "margin_m": self.margin_m, "detour": self.detour,
                                     "backend": self.backend, "planner": self.planner}, indent=2) + "\n",
                         encoding="utf-8")
        except (OSError, SystemExit):
            pass
        self._apply_backend()
        return self.config()

    def config(self) -> dict:
        return {"enabled": self.enabled, "margin_m": self.margin_m, "margin_mm": self.margin_m * 1000.0,
                "detour": self.detour, "backend": self.backend, "planner": self.planner}

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
        self.sync_tools()
        args = (arm, np.asarray(q_from, dtype=float).reshape(-1), np.asarray(q_to, dtype=float).reshape(-1),
                {k: [np.asarray(v, dtype=float) for v in vs] for k, vs in others.items()},
                self._body_q(), float(self.margin_m))
        if self._source is not None:
            from robot_platform.tasks import ik_worker

            res = ik_worker.run(_remote_check, self._remote_source(), self.arm_joints, *args)
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

    def plan_move(
        self,
        arm: str,
        q_from: Sequence[float],
        q_to: Sequence[float],
        others: Mapping[str, Sequence[Sequence[float]]],
        limits: np.ndarray,
        budget_s: float = 4.0,
    ) -> dict:
        """直线会碰时找绕行路线。返回 {"path": [中间点..., 目标] 或 None, "reason", "ms", ...}，同时记到 last_plan。"""
        t0 = time.perf_counter()
        self.sync_tools()
        args = (arm, np.asarray(q_from, dtype=float).reshape(-1), np.asarray(q_to, dtype=float).reshape(-1),
                {k: [np.asarray(v, dtype=float) for v in vs] for k, vs in others.items()},
                self._body_q(), float(self.margin_m), np.asarray(limits, dtype=float), float(budget_s), self.planner)
        if self._source is not None:
            from robot_platform.tasks import ik_worker

            res = ik_worker.run(_remote_plan, self._remote_source(), self.arm_joints, *args)
        else:
            res = self._plan_core(*args)
        res["ms"] = (time.perf_counter() - t0) * 1000.0
        path = res.get("path")
        info = {"arm": arm, "time": time.strftime("%H:%M:%S"), "ok": path is not None,
                "planner": res.get("planner") or self.planner, "backend": self.model.backend, "mode": res.get("mode"),
                "waypoints": 0 if path is None else len(path) - 1, "ms": res["ms"],
                "reason": res.get("reason") or "", "nodes": res.get("nodes"), "edges": res.get("edges")}
        if path is not None:
            q = [np.asarray(q_from, dtype=float), *[np.asarray(p) for p in path]]
            info["length_rad"] = float(sum(np.linalg.norm(b - a) for a, b in zip(q, q[1:])))
            info["direct_rad"] = float(np.linalg.norm(q[-1] - q[0]))
        with self._lock:
            self.last_plan = info
        return res

    def _check_core(self, arm, q0, q1, others, base, margin) -> dict:
        """路径检查本体（可在子进程里跑）：不改状态、不抛错，结果里带 block 表示会撞。"""
        variants = self._variants(arm, others)
        thresh = self._thresh(arm, q0, variants, base, margin)
        return self._sweep(arm, q0, q1, variants, thresh, base, margin)

    def _variants(self, arm, others) -> List[dict]:
        """另一只手要查的各个姿态组合（指令角 / 目标角 / 剩余中间点，去重）。"""
        names = [k for k in others if k != arm and k in self.arm_joints]

        def _uniq(vs):
            out = []
            for v in vs:
                v = np.asarray(v, dtype=float)
                if not any(np.allclose(v, u, atol=1e-4) for u in out):
                    out.append(v)
            return out

        variants = [{k: v for k, v in zip(names, combo) if v is not None}
                    for combo in itertools.product(*[_uniq(others[k]) or [None] for k in names])]
        return variants or [{}]

    def _thresh(self, arm, q0, variants, base, margin) -> List[np.ndarray]:
        """每对的下限：平时是余量；起点就已比「余量 + 2 mm」近的，只要求不比起点更近 2 mm。"""
        idx = self._arm_pairs[arm]
        return [np.minimum(margin, self.model.evaluate(self._q(base, {**o, arm: q0}), NEAR_M, idx) - DEEPER_TOL_M)
                for o in variants]

    def _sweep(self, arm, q0, q1, variants, thresh, base, margin, min_step: float = MIN_STEP_M) -> dict:
        """按给定下限查 q0 → q1 这段直线（保守推进；min_step = 贴着走时每步至少走多远）。"""
        idx = self._arm_pairs[arm]
        D = q1 - q0
        l1 = float(np.abs(D).sum())
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
                    free = min(free, float((np.maximum(d - thresh[vi], min_step) / rate).min()))
                    closest = min(closest, float(d.min()))
            if t >= 1.0 or l1 < 1e-9:
                break
            t = min(1.0, t + free)
        return {"checked": True, "arm": arm, "evals": n_eval,
                "closest_mm": None if not math.isfinite(closest) else closest * 1000.0}

    def _plan_core(self, arm, q0, q1, others, base, margin, limits, budget_s, planner_name="builtin") -> dict:
        """绕行规划本体（可在子进程里跑）：目标本身会碰就不规划；否则 RRT-Connect 找折线。"""
        from robot_platform.collision import planner

        variants = self._variants(arm, others)
        thresh = self._thresh(arm, q0, variants, base, margin)
        end = self._sweep(arm, q1, q1, variants, thresh, base, margin)
        if "block" in end:
            b = end["block"]
            return {"path": None, "reason": f"目标姿态本身就{'撞上' if b['distance_mm'] < 0 else '贴近'}"
                                            f"（{b['pair_text']}，{abs(b['distance_mm']):.1f} mm），没法绕"}

        def exact_ok(a, b):
            return "block" not in self._sweep(arm, a, b, variants, thresh, base, margin)

        # 搜索时下限加厚 COARSE_M、贴着走的最小步也取 COARSE_M：离障碍 1 cm 内就算不通，算的次数少得多。
        # 这样搜出的每段对原下限仍是安全的（每步走的距离不超过多出来的那 1 cm）；
        # 起点 / 目标本来就贴近的连杆对下限加不上去，所以最后再按原精度把整条复查一遍，不过就退回精确搜索。
        idx = self._arm_pairs[arm]

        def _d(q):
            return [self.model.evaluate(self._q(base, {**o, arm: q}), NEAR_M, idx) for o in variants]

        coarse = [np.minimum(t + COARSE_M, np.maximum(t, np.minimum(a, b) - DEEPER_TOL_M))
                  for t, a, b in zip(thresh, _d(q0), _d(q1))]

        def coarse_ok(a, b):
            return "block" not in self._sweep(arm, a, b, variants, coarse, base, margin, min_step=COARSE_M)

        def coarse_point(q):
            return "block" not in self._sweep(arm, q, q, variants, coarse, base, margin)

        def exact_point(q):
            return "block" not in self._sweep(arm, q, q, variants, thresh, base, margin)

        use_ompl = planner_name == "ompl" and planner.planner_available("ompl")

        def run(edge, point, budget, seed):
            if use_ompl:
                return planner.plan_ompl(edge, point, q0, q1, lim, budget_s=budget, seed=seed)
            return planner.plan(edge, q0, q1, lim, budget_s=budget, seed=seed, direct_known_bad=True)

        t0 = time.perf_counter()
        lim = np.asarray(limits, dtype=float)
        res = run(coarse_ok, coarse_point, budget_s, 0)
        res["mode"] = "coarse"
        if res["path"] is not None:
            pts = [np.asarray(q0, dtype=float), *res["path"]]
            if not all(exact_ok(a, b) for a, b in zip(pts, pts[1:])):
                res = {"path": None}
        if res["path"] is None:
            left = budget_s - (time.perf_counter() - t0)
            if left > 0.5:
                res = run(exact_ok, exact_point, left, 1)
                res["mode"] = "exact"
        if res["path"] is not None:
            res["path"] = [np.asarray(p, dtype=float).tolist() for p in res["path"]]
        res["planner"] = "ompl" if use_ompl else "builtin"
        return res

    # ------------------------------------------------------------ 状态
    def status(self, arms_q: Optional[Mapping[str, Sequence[float]]] = None) -> dict:
        self.sync_tools()
        out = {**self.config(), "available": True, "last_block": self.last_block, "last_check": self.last_check,
               "last_plan": self.last_plan, **self.choices(),
               "pairs": len(self.model.pairs), "prims": int(len(self.model.R)), "tools": self.tools_text()}
        if arms_q:
            q = self._q(self._body_q(), {k: np.asarray(v, dtype=float) for k, v in arms_q.items() if k in self.arm_joints})
            if self._source is not None:
                from robot_platform.tasks import ik_worker

                d = ik_worker.run(_remote_eval, self._remote_source(), self.arm_joints, q, 0.15)
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
        self.sync_tools()
        links: Dict[str, List] = {}
        m = self.model
        # coal 后端：手臂各节按实际检查用的凸包画（{"hull": {"v", "f"}}），对应的球 / 胶囊不再画
        skip = getattr(m, "hull_prims", set())
        for link, items in getattr(m, "hull_display", {}).items():
            links.setdefault(link, []).extend(items)
        for i, li in enumerate(m.owner):
            if i in skip:
                continue
            if m.is_poly[i]:
                # 多边形柱：{"prism": {"poly": [[u,v],...], "z": [w0, w1], "F": 按行 9 个}}
                # （柱自身 uvw 坐标；F 的三列是 u、v、w 在连杆系里的方向）
                from robot_platform.collision.geometry import prism_frame, prism_poly

                pr = m.prism_src[i]
                item = {"prism": {"poly": np.round(prism_poly(pr), 4).tolist(),
                                  "z": [round(float(v), 4) for v in pr["z"]],
                                  "F": np.round(prism_frame(pr).reshape(-1), 6).tolist()}}
            else:
                item = [*np.round(m.A[i], 4).tolist(), *np.round(m.B[i], 4).tolist(), round(float(m.R[i]), 4)]
            links.setdefault(m.prim_links[li], []).append(item)
        return {"links": links, "tools": self.tools_text(), "backend": m.backend}


# ---------------------------------------------------------------- 子进程
_REMOTE: Dict[tuple, CollisionGuard] = {}


def _remote_guard(source, arm_joints) -> Optional[CollisionGuard]:
    """按 (urdf, base_link, 末端, collision.json 修改时间) 缓存一个闸门（子进程里用）。"""
    from robot_platform.collision.model import default_spec_path, try_load

    urdf, base_link = source[:2]
    tools = tuple(source[2]) if len(source) > 2 else ()
    backend = str(source[3]) if len(source) > 3 else "builtin"
    spec = default_spec_path(urdf)
    key = (str(urdf), base_link, tools, backend, spec.stat().st_mtime if spec.is_file() else 0.0)
    g = _REMOTE.get(key)
    if g is None:
        model = try_load(urdf, base_link, tools, backend)
        if model is None:
            return None
        _REMOTE.clear()
        g = _REMOTE[key] = CollisionGuard(model, arm_joints, lambda: {}, load_config=False)
    return g


def _remote_check(source, arm_joints, arm, q0, q1, others, base, margin) -> dict:
    g = _remote_guard(source, arm_joints)
    return {"checked": False} if g is None else g._check_core(arm, q0, q1, others, base, margin)


def _remote_plan(source, arm_joints, arm, q0, q1, others, base, margin, limits, budget_s, planner_name) -> dict:
    g = _remote_guard(source, arm_joints)
    if g is None:
        return {"path": None, "reason": "子进程里没有碰撞模型"}
    return g._plan_core(arm, q0, q1, others, base, margin, limits, budget_s, planner_name)


def _remote_eval(source, arm_joints, q, near) -> np.ndarray:
    g = _remote_guard(source, arm_joints)
    return np.zeros(0) if g is None else g.model.evaluate(q, near)
