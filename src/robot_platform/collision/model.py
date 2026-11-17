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
        A, B, R, owner = [], [], [], []
        for li, link in enumerate(self.prim_links):
            for p in prims[link]:
                A.append(p["a"])
                B.append(p["b"])
                R.append(float(p["r"]))
                owner.append(li)
        self.A = np.asarray(A, dtype=float).reshape(-1, 3)
        self.B = np.asarray(B, dtype=float).reshape(-1, 3)
        self.R = np.asarray(R, dtype=float)
        self.owner = np.asarray(owner, dtype=int)
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
        """全部图元端点（躯干系）+ 各连杆包围球心。"""
        T = self.tree.transforms(q, self.prim_links)
        Rl = np.stack([T[l][:3, :3] for l in self.prim_links])
        tl = np.stack([T[l][:3, 3] for l in self.prim_links])
        Rs, ts = Rl[self.owner], tl[self.owner]
        A = np.einsum("nij,nj->ni", Rs, self.A) + ts
        B = np.einsum("nij,nj->ni", Rs, self.B) + ts
        bc = np.einsum("nij,nj->ni", Rl, self._bc) + tl
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
        for g in np.nonzero(gap < near)[0]:
            la, lb = pa[g], pb[g]
            ia, ib = self._link_prims[la], self._link_prims[lb]
            ia = ia[self._near_ball(A[ia], B[ia], self.R[ia], bc[lb], self._br[lb], near)]
            ib = ib[self._near_ball(A[ib], B[ib], self.R[ib], bc[la], self._br[la], near)]
            # 筛掉的图元离对方至少 near：结果取 min(细算, near)，仍是真实距离的下界
            if ia.size == 0 or ib.size == 0:
                out[g] = near
                continue
            I = np.repeat(ia, ib.size)
            J = np.tile(ib, ia.size)
            d = segment_distances(A[I], B[I], A[J], B[J]) - self.R[I] - self.R[J]
            out[g] = min(float(d.min()), near)
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
