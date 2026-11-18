"""coal（原 hpp-fcl）碰撞后端：和 CollisionModel 同一份 collision.json、同一套连杆对，距离用 coal 的 GJK / EPA 算。

几何的组织（每个连杆若干「块」，每块一个 coal 凸体）：
- 手臂各节：这节所有球 / 胶囊的凸包（一块）——比几百个小球少算得多；凸包把凹处也包进去，所以偏保守
  （实测和内置模型比中位差 0.3 mm、最大 3 cm 左右，只会多拦不会少拦）；
- 末端胶囊（法兰 → TCP）：单独一块 coal.Capsule，不并进腕的凸包（不然整个腕到 TCP 都被包成一大块）；
- 身体：每根凸多边形柱一块。
粗筛：每块一个包围球，连杆对的所有块对一次用 numpy 算间隙；只有间隙 < near 的块对才调 coal.distance。
返回值语义和 CollisionModel.evaluate 一致：≥ near 的给包围球间隙（真实距离的下界），细算的结果封顶 near。
"""

from __future__ import annotations

from pathlib import Path
from typing import Mapping, Optional, Tuple

import numpy as np

from robot_platform.collision.model import CollisionModel

try:  # 没装 coal（如本地 Windows）时这个后端不可用，见 available()
    import coal  # type: ignore
except Exception:  # noqa: BLE001
    coal = None


def available() -> bool:
    return coal is not None


def _fib_dirs(n: int = 64, scale: float = 1.065) -> np.ndarray:
    """球面上 n 个方向，放大 scale 倍：64 个点的凸包内切球半径约 0.943，放大 1.065 后凸包把整个单位球包住（偏保守约 6%）。"""
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    th = np.pi * (1 + 5 ** 0.5) * i
    return scale * np.c_[np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)]


def _rot_z_to(d: np.ndarray) -> np.ndarray:
    z = d / np.linalg.norm(d)
    a = np.array([0.0, 0.0, 1.0])
    v = np.cross(a, z)
    c = float(a @ z)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def _convex(pts: np.ndarray):
    """凸包；保留三角面（keepTriangles=True），三维显示要用。"""
    vv = coal.StdVec_Vec3s()
    for p in np.asarray(pts, dtype=float):
        vv.append(p)
    return coal.Convex.convexHull(vv, True, "Qt")  # 保留三角面时 qhull 要 Qt（三角化输出）


def hull_mesh(g) -> dict:
    """coal 凸包 → {"v": 顶点按行展平, "f": 三角形下标展平}（面朝外）。"""
    P = np.asarray(g.points(), dtype=float).reshape(-1, 3)
    F = [[int(g.polygons(i)[k]) for k in range(3)] for i in range(int(g.num_polygons))]
    return {"v": np.round(P, 4).reshape(-1).tolist(), "f": [k for t in F for k in t]}


class CoalModel(CollisionModel):
    backend = "coal"

    def __init__(self, urdf_path: Path, spec: dict, base_link: str = "torso_link", tools: Tuple = ()) -> None:
        if coal is None:
            raise RuntimeError("没装 coal（pip install coal）")
        super().__init__(urdf_path, spec, base_link, tools)
        from robot_platform.collision.geometry import prism_frame, prism_poly

        arm_set = {l for v in self.arm_links.values() for l in v}
        dirs = _fib_dirs()
        tool_ids = set(self._tool_prim_ids())
        geoms, plink, pc, pr, pT = [], [], [], [], []
        # 三维显示：手臂各节画凸包（不再画那几百个球），这些图元下标在 geometry() 里跳过
        self.hull_prims: set = set()
        self.hull_display: dict = {}

        def add(g, li, P_local, T_local):
            c = 0.5 * (P_local.min(axis=0) + P_local.max(axis=0))
            geoms.append(g)
            plink.append(li)
            pc.append(c)
            pr.append(float(np.linalg.norm(P_local - c, axis=1).max()))
            pT.append(T_local)

        for li, link in enumerate(self.prim_links):
            ids = np.nonzero(self.owner == li)[0]
            hull_ids = [i for i in ids if not self.is_poly[i] and i not in tool_ids]
            if link in arm_set and hull_ids:
                P = np.concatenate([np.r_[self.A[i] + self.R[i] * dirs, self.B[i] + self.R[i] * dirs]
                                    for i in hull_ids])
                g = _convex(P)
                add(g, li, P, np.eye(4))
                self.hull_prims.update(int(i) for i in hull_ids)
                self.hull_display.setdefault(link, []).append({"hull": hull_mesh(g)})
            elif hull_ids:  # 不是手臂上的球 / 胶囊（一般没有）：逐个
                for i in hull_ids:
                    P = np.r_[self.A[i] + self.R[i] * dirs, self.B[i] + self.R[i] * dirs]
                    add(_convex(P), li, P, np.eye(4))
            for i in ids:
                if self.is_poly[i]:
                    p = self.prism_src[i]
                    V, F = prism_poly(p), prism_frame(p)
                    z0, z1 = (float(v) for v in p["z"])
                    P = np.array([F @ [u, v, w] for (u, v) in V for w in (z0, z1)], dtype=float)
                    add(_convex(P), li, P, np.eye(4))
                elif i in tool_ids:
                    d = self.B[i] - self.A[i]
                    L = float(np.linalg.norm(d))
                    T = np.eye(4)
                    T[:3, 3] = 0.5 * (self.A[i] + self.B[i])
                    if L < 1e-9:
                        g = coal.Sphere(float(self.R[i]))
                    else:
                        g = coal.Capsule(float(self.R[i]), L)
                        T[:3, :3] = _rot_z_to(d)
                    P = np.r_[self.A[i] + self.R[i] * dirs, self.B[i] + self.R[i] * dirs]
                    add(g, li, P, T)
        self._geoms = geoms
        self._plink = np.asarray(plink, dtype=int)
        self._pc = np.asarray(pc, dtype=float).reshape(-1, 3)
        self._pr = np.asarray(pr, dtype=float)
        self._pT = np.asarray(pT, dtype=float).reshape(-1, 4, 4)
        self._req = coal.DistanceRequest()
        self._req.enable_signed_distance = True
        self._res = coal.DistanceResult()
        self._index_pairs()

    def _tool_prim_ids(self):
        """末端胶囊（tool_prims 加在各连杆列表最前面）在图元数组里的下标。"""
        from robot_platform.collision.model import tool_prims

        tp = tool_prims(self.spec, self.tools)
        out = []
        for link, lst in tp.items():
            if link not in self.prim_links:
                continue
            ids = np.nonzero(self.owner == self.prim_links.index(link))[0]
            out.extend(int(i) for i in ids[:len(lst)])
        return out

    def set_pairs(self, pairs) -> None:
        super().set_pairs(pairs)
        if hasattr(self, "_plink"):
            self._index_pairs()

    def _index_pairs(self) -> None:
        """所有连杆对展开成块对：(块 i, 块 j, 属于第几个连杆对)。"""
        by_link = [np.nonzero(self._plink == li)[0] for li in range(len(self.prim_links))]
        I, J, K = [], [], []
        for k, (la, lb) in enumerate(zip(self._pa, self._pb)):
            a, b = by_link[la], by_link[lb]
            if a.size == 0 or b.size == 0:
                continue
            I.append(np.repeat(a, b.size))
            J.append(np.tile(b, a.size))
            K.append(np.full(a.size * b.size, k))
        self._bi = np.concatenate(I) if I else np.zeros(0, dtype=int)
        self._bj = np.concatenate(J) if J else np.zeros(0, dtype=int)
        self._bk = np.concatenate(K) if K else np.zeros(0, dtype=int)
        self._bk_order = np.argsort(self._bk, kind="stable")

    def evaluate(self, q: Mapping[str, float], near: float = 0.08, idx: Optional[np.ndarray] = None) -> np.ndarray:
        if not self.pairs:
            return np.zeros(0)
        T = self.tree.transforms(q, self.prim_links)
        Rl = np.stack([T[l][:3, :3] for l in self.prim_links])
        tl = np.stack([T[l][:3, 3] for l in self.prim_links])
        R, t = Rl[self._plink], tl[self._plink]
        cw = np.einsum("nij,nj->ni", R, self._pc) + t
        npair = len(self.pairs)
        if idx is None:
            sel = np.arange(self._bk.size)
            want = np.arange(npair)
        else:
            want = np.asarray(idx, dtype=int)
            mask = np.zeros(npair, dtype=bool)
            mask[want] = True
            sel = np.nonzero(mask[self._bk])[0]
        bi, bj, bk = self._bi[sel], self._bj[sel], self._bk[sel]
        gap = np.linalg.norm(cw[bi] - cw[bj], axis=1) - self._pr[bi] - self._pr[bj]
        lower = np.full(npair, np.inf)
        np.minimum.at(lower, bk, gap)
        out = np.where(lower >= near, lower, near)
        cand = np.nonzero(gap < near)[0]
        if cand.size:
            cand = cand[np.lexsort((gap[cand], bk[cand]))]
            tf_cache = {}

            def tf(p):
                X = tf_cache.get(p)
                if X is None:
                    W = np.eye(4)
                    W[:3, :3] = R[p]
                    W[:3, 3] = t[p]
                    W = W @ self._pT[p]
                    X = tf_cache[p] = coal.Transform3s(np.ascontiguousarray(W[:3, :3]),
                                                       np.ascontiguousarray(W[:3, 3]))
                return X

            req, res = self._req, self._res
            for c in cand:
                k = bk[c]
                if gap[c] >= out[k]:
                    continue  # 已有更近的（同一对按间隙从小到大排，这块不可能更近）
                i, j = bi[c], bj[c]
                res.clear()
                d = float(coal.distance(self._geoms[i], tf(i), self._geoms[j], tf(j), req, res))
                if d < out[k]:
                    out[k] = d
        return out[want]
