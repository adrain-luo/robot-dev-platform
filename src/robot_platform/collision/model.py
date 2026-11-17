"""碰撞模型（运行时）：整棵 URDF 树的正运动学 + 各连杆的胶囊体 / 球，算各连杆对的最近距离。

几何来自 robots/<机型>/collision.json（tools/fit_collision.py 由 STL 拟合生成），坐标在各连杆自身坐标系里。
所有距离都在躯干系（torso_link）下算：两只手、头、躯干在躯干系里只随手臂关节动；立柱 / 底盘随立柱高度、
腰转角动。
"""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from robot_platform.collision.geometry import segment_distances
from robot_platform.control.urdf_fk import axis_angle_to_matrix, make_transform


@dataclass
class _Joint:
    name: str
    type: str
    parent: str
    child: str
    T0: np.ndarray
    axis: np.ndarray


class UrdfTree:
    """整棵树的正运动学，结果都换到 base_link（默认躯干）坐标系下。"""

    def __init__(self, urdf_path: Path, base_link: str = "torso_link") -> None:
        root = ET.parse(str(urdf_path)).getroot()
        self.links = [l.get("name") for l in root.findall("link")]
        self.joints: Dict[str, _Joint] = {}
        self.child_joint: Dict[str, _Joint] = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz") if o is not None and o.get("xyz") else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy") if o is not None and o.get("rpy") else "0 0 0").split()]
            ax = j.find("axis")
            axis = np.array([float(v) for v in (ax.get("xyz") if ax is not None else "1 0 0").split()], dtype=float)
            jj = _Joint(
                name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
                child=j.find("child").get("link"), T0=make_transform(xyz, rpy), axis=axis,
            )
            # 同名关节（URDF 里偶有重复）按子连杆区分
            self.joints[jj.name if jj.name not in self.joints else f"{jj.name}#{jj.child}"] = jj
            self.child_joint[jj.child] = jj
        self.base_link = base_link
        # 拓扑顺序（父在前）
        children: Dict[str, List[_Joint]] = {}
        for jj in self.child_joint.values():
            children.setdefault(jj.parent, []).append(jj)
        roots = [l for l in self.links if l not in self.child_joint]
        self.order: List[_Joint] = []
        stack = list(roots)
        while stack:
            l = stack.pop()
            for jj in children.get(l, []):
                self.order.append(jj)
                stack.append(jj.child)
        self.roots = roots

    def parent_of(self, link: str) -> Optional[str]:
        jj = self.child_joint.get(link)
        return None if jj is None else jj.parent

    def transforms(self, q: Mapping[str, float], links: Optional[Sequence[str]] = None) -> Dict[str, np.ndarray]:
        """各连杆在 base_link 系下的 4x4。q 里没有的关节按 0。"""
        T: Dict[str, np.ndarray] = {r: np.eye(4) for r in self.roots}
        for jj in self.order:
            Tp = T[jj.parent]
            if jj.type in ("revolute", "continuous"):
                M = np.eye(4)
                M[:3, :3] = axis_angle_to_matrix(jj.axis, float(q.get(jj.name, 0.0)))
                T[jj.child] = Tp @ jj.T0 @ M
            elif jj.type == "prismatic":
                M = np.eye(4)
                M[:3, 3] = jj.axis * float(q.get(jj.name, 0.0))
                T[jj.child] = Tp @ jj.T0 @ M
            else:
                T[jj.child] = Tp @ jj.T0
        inv = np.linalg.inv(T[self.base_link])
        keys = links if links is not None else list(T.keys())
        return {k: inv @ T[k] for k in keys if k in T}


# 每组大约多少个图元（连杆内再分组的包围球筛选）
GROUP = 12
# 组对一批细算多少对
BATCH = 32


def load_spec(path: Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


class CollisionModel:
    """几何 + 要查的连杆对。evaluate(q) → 各连杆对的最近表面距离（m，负数=穿透）。"""

    def __init__(self, urdf_path: Path, spec: dict, base_link: str = "torso_link") -> None:
        self.tree = UrdfTree(urdf_path, base_link)
        self.spec = spec
        prims: Dict[str, list] = {}
        for src in ("links", "manual"):
            for link, lst in (spec.get(src) or {}).items():
                prims.setdefault(link, []).extend(lst)
        self.prim_links: List[str] = sorted(prims)
        from robot_platform.collision.geometry import prism_frame, prism_planes, prism_poly

        A, B, R, owner = [], [], [], []
        polys: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}
        self.prism_src: Dict[int, dict] = {}
        for li, link in enumerate(self.prim_links):
            for p in prims[link]:
                if p.get("type") == "prism":
                    # 凸多边形柱（沿连杆 z）：半空间 n·x ≤ d；A=B=中心，R=包围半径（粗筛用）
                    N, D = prism_planes(p)
                    V = prism_poly(p)
                    z0, z1 = (float(v) for v in p["z"])
                    c2 = V.mean(axis=0)
                    c = prism_frame(p) @ np.array([c2[0], c2[1], 0.5 * (z0 + z1)])
                    rb = float(np.sqrt((np.linalg.norm(V - c2, axis=1).max()) ** 2 + (0.5 * (z1 - z0)) ** 2))
                    polys[len(A)] = (N, D)
                    self.prism_src[len(A)] = p
                    A.append(c)
                    B.append(c)
                    R.append(rb)
                else:
                    A.append(p["a"])
                    B.append(p["b"])
                    R.append(float(p["r"]))
                owner.append(li)
        self.A = np.asarray(A, dtype=float).reshape(-1, 3)
        self.B = np.asarray(B, dtype=float).reshape(-1, 3)
        self.R = np.asarray(R, dtype=float)
        self.owner = np.asarray(owner, dtype=int)
        n = len(self.R)
        self.is_poly = np.zeros(n, dtype=bool)
        K = max([len(prism_poly(pr)) for pr in self.prism_src.values()], default=3)
        # 各柱：横截面顶点 V、侧面外法向 n2 / 偏移 d2、上下 z（连杆系）；边数不同的补最后一个顶点（退化边不影响）
        self.polyV = np.zeros((n, K, 2))
        self.polyN2 = np.zeros((n, K, 2))
        self.polyD2 = np.full((n, K), 1e9)  # 补的行：n=0、d 很大 → n·p − d 很负，取 max 时不起作用
        self.polyZ = np.zeros((n, 2))
        self.polyF = np.tile(np.eye(3), (n, 1, 1))
        self.polyN = np.zeros((n, K + 2, 3))
        self.polyD = np.full((n, K + 2), 1e9)
        for i, (N, D) in polys.items():
            self.polyN[i, :len(D)] = N
            self.polyD[i, :len(D)] = D
        for i, pr in self.prism_src.items():
            self.is_poly[i] = True
            V = prism_poly(pr)
            n2 = np.asarray(pr["n2"], dtype=float)
            d2 = np.asarray(pr["d2"], dtype=float)
            k = len(V)
            self.polyV[i, :k] = V
            self.polyV[i, k:] = V[-1]
            self.polyN2[i, :len(n2)] = n2
            self.polyD2[i, :len(d2)] = d2
            self.polyZ[i] = [float(v) for v in pr["z"]]
            self.polyF[i] = prism_frame(pr)
        self.is_sphere = np.all(np.abs(self.A - self.B) < 1e-12, axis=1) & ~self.is_poly
        self.arm_links: Dict[str, List[str]] = {k: list(v) for k, v in (spec.get("arms") or {}).items()}
        self._build_bounds()
        self.set_pairs([tuple(p[:2]) for p in spec.get("pairs") or []])

    # ------------------------------------------------------------ 连杆对
    def set_pairs(self, pairs: Sequence[Tuple[str, str]]) -> None:
        idx = {l: i for i, l in enumerate(self.prim_links)}
        keep = [(a, b) for a, b in pairs if a in idx and b in idx and a != b]
        self.pairs: List[Tuple[str, str]] = keep
        self._pa = np.array([idx[a] for a, _ in keep], dtype=int)
        self._pb = np.array([idx[b] for _, b in keep], dtype=int)

    def _build_bounds(self) -> None:
        """每个连杆一个包围球（连杆系），粗筛用。"""
        self._link_prims = [np.nonzero(self.owner == i)[0] for i in range(len(self.prim_links))]
        C, Rb = [], []
        for ids in self._link_prims:
            pts = np.r_[self.A[ids], self.B[ids]]
            c = 0.5 * (pts.min(axis=0) + pts.max(axis=0))
            r = float(np.max(np.r_[np.linalg.norm(self.A[ids] - c, axis=1), np.linalg.norm(self.B[ids] - c, axis=1)]
                             + np.r_[self.R[ids], self.R[ids]]))
            C.append(c)
            Rb.append(r)
        self._bc = np.asarray(C).reshape(-1, 3)
        self._br = np.asarray(Rb)
        # 第二层：每个连杆的图元按位置分成约 GROUP 个一组，各组一个包围球（连杆系），细算前先按组筛
        self._groups: List[List[np.ndarray]] = []
        self._gc: List[np.ndarray] = []
        self._gr: List[np.ndarray] = []
        rng = np.random.default_rng(0)
        for ids in self._link_prims:
            mid = 0.5 * (self.A[ids] + self.B[ids])
            k = max(1, int(np.ceil(len(ids) / GROUP)))
            if k == 1:
                lab = np.zeros(len(ids), dtype=int)
            else:
                cen = mid[rng.choice(len(ids), k, replace=False)]
                for _ in range(15):
                    lab = np.argmin(((mid[:, None, :] - cen[None, :, :]) ** 2).sum(-1), axis=1)
                    cen = np.array([mid[lab == i].mean(axis=0) if np.any(lab == i) else cen[i] for i in range(k)])
            groups, gc, gr = [], [], []
            for i in range(k):
                sel = ids[lab == i]
                if sel.size == 0:
                    continue
                pts = np.r_[self.A[sel], self.B[sel]]
                c = 0.5 * (pts.min(axis=0) + pts.max(axis=0))
                r = float(np.max(np.r_[np.linalg.norm(self.A[sel] - c, axis=1) + self.R[sel],
                                       np.linalg.norm(self.B[sel] - c, axis=1) + self.R[sel]]))
                groups.append(sel)
                gc.append(c)
                gr.append(r)
            self._groups.append(groups)
            self._gc.append(np.asarray(gc).reshape(-1, 3))
            self._gr.append(np.asarray(gr))

    def candidate_pairs(self, min_chain_gap: int = 2, same_arm_gap: int = 3) -> List[Tuple[str, str]]:
        """所有至少一端在手臂上的连杆对，去掉树上相隔太近的（相邻连杆本来就贴着）：
        手臂对身体 / 另一只手隔 ≥ min_chain_gap 个活动关节；同一只手隔 ≥ same_arm_gap（腕三节、肘与腕俯仰
        这类隔一个关节的，机械上就挨着转，不会互撞）。"""
        arm_of = {l: k for k, ls in self.arm_links.items() for l in ls}
        out = []
        for i, a in enumerate(self.prim_links):
            for b in self.prim_links[i + 1:]:
                if a not in arm_of and b not in arm_of:
                    continue
                need = same_arm_gap if arm_of.get(a) is not None and arm_of.get(a) == arm_of.get(b) else min_chain_gap
                if self.chain_gap(a, b) < need:
                    continue
                out.append((a, b))
        return out

    def chain_gap(self, a: str, b: str) -> int:
        """两连杆在树上隔了几个活动关节（fixed 不算）。"""
        def path_up(l):
            out = [l]
            while True:
                p = self.tree.parent_of(out[-1])
                if p is None:
                    return out
                out.append(p)
        pa, pb = path_up(a), path_up(b)
        sb = set(pb)
        common = next(l for l in pa if l in sb)
        n = 0
        for path in (pa, pb):
            for l in path:
                if l == common:
                    break
                if self.tree.child_joint[l].type != "fixed":
                    n += 1
        return n

    # ------------------------------------------------------------ 求值
    def world_prims(self, q: Mapping[str, float]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """全部图元端点（躯干系）+ 各连杆包围球心（各连杆的旋转 / 平移留在 self._last_T，分组筛选用）。"""
        T = self.tree.transforms(q, self.prim_links)
        Rl = np.stack([T[l][:3, :3] for l in self.prim_links])
        tl = np.stack([T[l][:3, 3] for l in self.prim_links])
        Rs, ts = Rl[self.owner], tl[self.owner]
        A = np.einsum("nij,nj->ni", Rs, self.A) + ts
        B = np.einsum("nij,nj->ni", Rs, self.B) + ts
        bc = np.einsum("nij,nj->ni", Rl, self._bc) + tl
        self._last_T = (Rl, tl)
        # 多边形柱：精确距离时把对方点换到柱所在连杆系；平面（快速下界用）换到躯干系
        self._Rs, self._ts = Rs, ts
        self._RF = np.einsum("nij,njk->nik", Rs, self.polyF) if self.is_poly.any() else Rs
        if self.is_poly.any():
            pi = np.nonzero(self.is_poly)[0]
            Nw = np.einsum("nij,nkj->nki", Rs[pi], self.polyN[pi])
            self._polyNw = np.zeros_like(self.polyN)
            self._polyDw = np.full_like(self.polyD, 1e9)
            self._polyNw[pi] = Nw
            self._polyDw[pi] = self.polyD[pi] + np.einsum("nki,ni->nk", Nw, ts[pi])
        return A, B, bc

    def evaluate(self, q: Mapping[str, float], near: float = 0.08, idx: Optional[np.ndarray] = None) -> np.ndarray:
        """各连杆对（self.pairs 的顺序；给了 idx 就只算这些下标）的最近表面距离（m，负=穿透）。

        粗筛：包围球间隙 ≥ near 的连杆对不细算，返回包围球间隙（真实距离的下界）；
        细算时也只取离对方包围球 < near 的图元。
        """
        if not self.pairs:
            return np.zeros(0)
        pa = self._pa if idx is None else self._pa[idx]
        pb = self._pb if idx is None else self._pb[idx]
        A, B, bc = self.world_prims(q)
        gap = np.linalg.norm(bc[pa] - bc[pb], axis=1) - self._br[pa] - self._br[pb]
        out = gap.copy()
        Rl, tl = self._last_T
        for g in np.nonzero(gap < near)[0]:
            la, lb = pa[g], pb[g]
            ca = self._gc[la] @ Rl[la].T + tl[la]
            cb = self._gc[lb] @ Rl[lb].T + tl[lb]
            gg = (np.linalg.norm(ca[:, None, :] - cb[None, :, :], axis=2)
                  - self._gr[la][:, None] - self._gr[lb][None, :])
            ii, jj = np.nonzero(gg < near)
            # 筛掉的组离对方至少 near：结果取 min(细算, near)，仍是真实距离的下界
            if ii.size == 0:
                out[g] = near
                continue
            # 按组间隙从小到大一批批细算；下一批的组间隙已不小于当前最近距离就停（不可能更近）
            order = np.argsort(gg[ii, jj])
            ii, jj, gs = ii[order], jj[order], gg[ii, jj][order]
            best = near
            for s0 in range(0, ii.size, BATCH):
                if gs[s0] >= best:
                    break
                sl = slice(s0, s0 + BATCH)
                ga, gb = self._groups[la], self._groups[lb]
                I = np.concatenate([np.repeat(ga[i], gb[j].size) for i, j in zip(ii[sl], jj[sl])])
                J = np.concatenate([np.tile(gb[j], ga[i].size) for i, j in zip(ii[sl], jj[sl])])
                best = min(best, self._pair_min(A, B, I, J, best))
            out[g] = best
        return out

    def _pair_min(self, A, B, I, J, cutoff: float = np.inf) -> float:
        """成对图元 (I[k], J[k]) 的最近表面距离的最小值。两端都是球的直接用球心距；有多边形柱的单独算。"""
        px = self.is_poly[I] | self.is_poly[J]
        best = np.inf
        if np.any(px):
            i, j = I[px], J[px]
            sw = self.is_poly[i] & ~self.is_poly[j]  # 让柱在 j 一侧
            i, j = np.where(sw, j, i), np.where(sw, i, j)
            both = self.is_poly[i]
            if np.any(both):  # 柱对柱（身体对身体，一般不查）：按包围球，偏保守
                bi, bj = i[both], j[both]
                best = float((np.linalg.norm(A[bi] - A[bj], axis=1) - self.R[bi] - self.R[bj]).min())
            i, j = i[~both], j[~both]
            if i.size:
                # 包围球初筛（柱的包围半径大，这一步最便宜）：线段按 a 端点 + 线段长算
                pre = (np.linalg.norm(A[i] - A[j], axis=1) - self.R[i] - self.R[j]
                       - np.linalg.norm(B[i] - A[i], axis=1)) < min(cutoff, best)
                i, j = i[pre], j[pre]
            if i.size:
                # 快速下界：每个平面在线段两端取小的，再对平面取大（不大于真实距离）；够远的不细算
                Nw, Dw = self._polyNw[j], self._polyDw[j]
                la = np.einsum("nkj,nj->nk", Nw, A[i]) - Dw
                lb = np.einsum("nkj,nj->nk", Nw, B[i]) - Dw
                lower = np.minimum(la, lb).max(axis=1) - self.R[i]
                keep = lower < min(cutoff, best)
                if np.any(keep):
                    i, j = i[keep], j[keep]
                    # 先细算下界最小的一批，用它收紧门限，再算剩下的
                    order = np.argsort(lower[keep])
                    i, j, lw = i[order], j[order], lower[keep][order]
                    for s0 in range(0, i.size, 256):
                        if lw[s0] >= best:
                            break
                        sl = slice(s0, s0 + 256)
                        best = min(best, float(self._poly_dist(A[i[sl]], B[i[sl]], self.R[i[sl]], j[sl]).min()))
            I, J = I[~px], J[~px]
            if I.size == 0:
                return best
        ss = self.is_sphere[I] & self.is_sphere[J]
        if np.any(ss):
            i, j = I[ss], J[ss]
            best = float((np.linalg.norm(A[i] - A[j], axis=1) - self.R[i] - self.R[j]).min())
        if not np.all(ss):
            i, j = I[~ss], J[~ss]
            best = min(best, float((segment_distances(A[i], B[i], A[j], B[j]) - self.R[i] - self.R[j]).min()))
        return best

    def _poly_sdf(self, x: np.ndarray, j: np.ndarray) -> np.ndarray:
        """点 x（躯干系）到柱 j 的精确有符号距离（外正内负）。拉伸体：2D 多边形距离与 z 区间距离合成。"""
        R, t = self._RF[j], self._ts[j]
        p = np.einsum("nij,ni->nj", R, x - t)  # (R F)ᵀ(x − t)：换到柱自身 (u, v, w)
        xy = p[:, :2]
        lin = np.einsum("nkj,nj->nk", self.polyN2[j], xy) - self.polyD2[j]
        inside = lin.max(axis=1)
        V = self.polyV[j]
        W = np.roll(V, -1, axis=1)
        e = W - V
        L = np.maximum(np.einsum("nkj,nkj->nk", e, e), 1e-18)
        tt = np.clip(np.einsum("nkj,nkj->nk", xy[:, None, :] - V, e) / L, 0.0, 1.0)
        dseg = np.linalg.norm(xy[:, None, :] - (V + e * tt[..., None]), axis=2).min(axis=1)
        s2 = np.where(inside <= 0.0, inside, dseg)
        z0, z1 = self.polyZ[j, 0], self.polyZ[j, 1]
        sz = np.maximum(z0 - p[:, 2], p[:, 2] - z1)
        return np.minimum(np.maximum(s2, sz), 0.0) + np.hypot(np.maximum(s2, 0.0), np.maximum(sz, 0.0))

    def _poly_dist(self, a, b, r, j, rounds: int = 3, n: int = 9) -> np.ndarray:
        """球 / 胶囊体（a-b, r）到柱 j 的表面距离，成批。球直接算；胶囊体沿线段逐级加密取样：
        到凸体的有符号距离沿线段是凸函数，每轮在 n 个等分点里取最小的，把区间缩到它左右各一格，
        3 轮后区间只剩原长的 (2/(n−1))³（9 点时 1/64）。一次算一整批，比黄金分割的几十次调用快。"""
        out = self._poly_sdf(a, j) - r
        seg = np.any(np.abs(b - a) > 1e-12, axis=1)
        if np.any(seg):
            a2, d2, j2 = a[seg], (b - a)[seg], j[seg]
            m = len(a2)
            lo = np.zeros(m)
            hi = np.ones(m)
            u = np.linspace(0.0, 1.0, n)
            jj = np.repeat(j2, n)
            best = np.full(m, np.inf)
            for _ in range(rounds):
                t = lo[:, None] + (hi - lo)[:, None] * u[None, :]
                x = (a2[:, None, :] + d2[:, None, :] * t[..., None]).reshape(-1, 3)
                f = self._poly_sdf(x, jj).reshape(m, n)
                k = np.argmin(f, axis=1)
                best = np.minimum(best, f[np.arange(m), k])
                step = (hi - lo) / (n - 1)
                tk = t[np.arange(m), k]
                lo = np.maximum(tk - step, 0.0)
                hi = np.minimum(tk + step, 1.0)
            out[seg] = best - r[seg]
        return out

    @staticmethod
    def _near_ball(A, B, R, c, rc, near) -> np.ndarray:
        ab = B - A
        L = np.einsum("ij,ij->i", ab, ab)
        t = np.clip(np.einsum("ij,ij->i", c - A, ab) / np.where(L > 1e-12, L, 1.0), 0.0, 1.0)
        d = np.linalg.norm(A + ab * t[:, None] - c, axis=1) - R - rc
        return d < near


def default_spec_path(urdf_path: Path) -> Path:
    return Path(urdf_path).parent / "collision.json"


def try_load(urdf_path: Path, base_link: str = "torso_link") -> Optional[CollisionModel]:
    p = default_spec_path(urdf_path)
    if not p.is_file():
        return None
    return CollisionModel(urdf_path, load_spec(p), base_link)


def deg(x: float) -> float:
    return x * 180.0 / math.pi
