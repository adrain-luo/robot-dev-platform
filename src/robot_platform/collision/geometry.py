"""碰撞几何：读 STL、把网格拟合成胶囊体 / 一组球、胶囊体之间的距离。

统一用胶囊体表示（线段 a-b + 半径 r；球就是 a == b 的胶囊体）：
两个胶囊体的距离 = 两线段最近距离 − 两半径，一次能对几千对向量化算完，且对关节角平滑。
"""

from __future__ import annotations

import struct
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


# ---------------------------------------------------------------- STL
def read_stl(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    """返回 (三角形顶点 (n,3,3), 面积 (n,))。支持二进制和 ASCII STL。"""
    raw = Path(path).read_bytes()
    tris: np.ndarray
    if len(raw) >= 84:
        n = struct.unpack("<I", raw[80:84])[0]
        if 84 + 50 * n == len(raw):
            rec = np.frombuffer(raw[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
            tris = rec["v"].reshape(-1, 3, 3).astype(float)
            return tris, _areas(tris)
    verts = []
    for line in raw.decode("utf-8", errors="ignore").splitlines():
        s = line.strip().split()
        if len(s) == 4 and s[0] == "vertex":
            verts.append([float(s[1]), float(s[2]), float(s[3])])
    tris = np.asarray(verts, dtype=float).reshape(-1, 3, 3)
    return tris, _areas(tris)


def _areas(tris: np.ndarray) -> np.ndarray:
    return 0.5 * np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1)


def sample_surface(tris: np.ndarray, areas: np.ndarray, n: int, rng: np.random.Generator) -> np.ndarray:
    """按面积均匀采样表面点，再并上全部顶点（保证尖角也被包住）。"""
    pts = [tris.reshape(-1, 3)]
    total = float(areas.sum())
    if total > 0 and n > 0:
        idx = rng.choice(len(tris), size=n, p=areas / total)
        u = rng.random((n, 1))
        v = rng.random((n, 1))
        flip = (u + v) > 1.0
        u = np.where(flip, 1.0 - u, u)
        v = np.where(flip, 1.0 - v, v)
        t = tris[idx]
        pts.append(t[:, 0] + u * (t[:, 1] - t[:, 0]) + v * (t[:, 2] - t[:, 0]))
    out = np.concatenate(pts, axis=0)
    # 顶点很多时去重压一压
    return np.unique(np.round(out, 4), axis=0)


def sample_surface_normals(tris: np.ndarray, areas: np.ndarray, n: int,
                           rng: np.random.Generator) -> Tuple[np.ndarray, np.ndarray]:
    """同 sample_surface，但带每个点所在三角形的法向（按 STL 顶点顺序，一般朝外）。"""
    nrm_t = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(nrm_t, axis=1, keepdims=True)
    good = ln[:, 0] > 1e-12
    tris, areas, nrm_t = tris[good], areas[good], nrm_t[good] / ln[good]
    pts = [tris.reshape(-1, 3)]
    nrm = [np.repeat(nrm_t, 3, axis=0)]
    total = float(areas.sum())
    if total > 0 and n > 0:
        idx = rng.choice(len(tris), size=n, p=areas / total)
        u = rng.random((n, 1))
        v = rng.random((n, 1))
        flip = (u + v) > 1.0
        u = np.where(flip, 1.0 - u, u)
        v = np.where(flip, 1.0 - v, v)
        t = tris[idx]
        pts.append(t[:, 0] + u * (t[:, 1] - t[:, 0]) + v * (t[:, 2] - t[:, 0]))
        nrm.append(nrm_t[idx])
    P = np.concatenate(pts)
    N = np.concatenate(nrm)
    _, keep = np.unique(np.round(P, 4), axis=0, return_index=True)
    return P[keep], N[keep]


# ---------------------------------------------------------------- 拟合
def point_segment_dist(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ab = b - a
    L = float(ab @ ab)
    if L < 1e-12:
        return np.linalg.norm(p - a, axis=1)
    t = np.clip(((p - a) @ ab) / L, 0.0, 1.0)
    return np.linalg.norm(p - (a + t[:, None] * ab), axis=1)


def fit_capsule(pts: np.ndarray) -> Dict[str, object]:
    """主轴（PCA）方向上的一根胶囊体，包住全部点。"""
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c, full_matrices=False)
    axis = vt[0]
    proj = (pts - c) @ axis
    radial = np.linalg.norm((pts - c) - np.outer(proj, axis), axis=1)
    r0 = float(radial.max())
    lo, hi = float(proj.min()) + r0, float(proj.max()) - r0
    if lo > hi:
        lo = hi = 0.5 * (float(proj.min()) + float(proj.max()))
    a = c + lo * axis
    b = c + hi * axis
    r = float(point_segment_dist(pts, a, b).max())
    return {"a": a, "b": b, "r": r}


def _farthest_seeds(pts: np.ndarray, k: int, rng: np.random.Generator) -> np.ndarray:
    seeds = [pts[rng.integers(len(pts))]]
    d = np.linalg.norm(pts - seeds[0], axis=1)
    for _ in range(1, k):
        seeds.append(pts[int(np.argmax(d))])
        d = np.minimum(d, np.linalg.norm(pts - seeds[-1], axis=1))
    return np.asarray(seeds)


def fit_spheres(pts: np.ndarray, r_target: float, rng: np.random.Generator, k_max: int = 48) -> List[Dict[str, object]]:
    """k-means 分簇，每簇一个球（球心=簇心，半径=簇内最远点）；k 从小往上加，直到最大半径 ≤ r_target。"""
    best = None
    for k in range(1, k_max + 1):
        cen = _farthest_seeds(pts, k, rng)
        for _ in range(25):
            lab = np.argmin(((pts[:, None, :] - cen[None, :, :]) ** 2).sum(-1), axis=1)
            new = np.array([pts[lab == i].mean(axis=0) if np.any(lab == i) else cen[i] for i in range(k)])
            if np.allclose(new, cen, atol=1e-5):
                break
            cen = new
        lab = np.argmin(((pts[:, None, :] - cen[None, :, :]) ** 2).sum(-1), axis=1)
        spheres = []
        for i in range(k):
            m = lab == i
            if not np.any(m):
                continue
            r = float(np.linalg.norm(pts[m] - cen[i], axis=1).max())
            spheres.append({"a": cen[i], "b": cen[i], "r": r})
        best = spheres
        if max(s["r"] for s in spheres) <= r_target:
            break
    assert best is not None
    return best


def _kmeans(pts: np.ndarray, k: int, rng: np.random.Generator, iters: int = 30) -> np.ndarray:
    cen = _farthest_seeds(pts, k, rng)
    lab = np.zeros(len(pts), dtype=int)
    for _ in range(iters):
        lab = np.argmin(((pts[:, None, :] - cen[None, :, :]) ** 2).sum(-1), axis=1)
        new = np.array([pts[lab == i].mean(axis=0) if np.any(lab == i) else cen[i] for i in range(k)])
        if np.allclose(new, cen, atol=1e-6):
            break
        cen = new
    return lab


def fit_capsules(pts: np.ndarray, max_parts: int = 3, gain: float = 0.85,
                 rng: Optional[np.random.Generator] = None) -> List[Dict[str, object]]:
    """一到 max_parts 根胶囊体包住连杆：点先 k-means 分成 k 块，每块一根；多一根体积要小于 gain 倍才加。"""
    rng = rng or np.random.default_rng(0)
    best = [fit_capsule(pts)]
    best_v = capsule_volume(best[0])
    sub = pts if len(pts) <= 6000 else pts[rng.choice(len(pts), 6000, replace=False)]
    for k in range(2, max_parts + 1):
        lab_sub = _kmeans(sub, k, rng)
        # 分块中心 → 全部点按最近中心归块
        cen = np.array([sub[lab_sub == i].mean(axis=0) for i in range(k) if np.any(lab_sub == i)])
        lab = np.argmin(((pts[:, None, :] - cen[None, :, :]) ** 2).sum(-1), axis=1)
        parts = [fit_capsule(pts[lab == i]) for i in range(len(cen)) if np.sum(lab == i) >= 4]
        v = sum(capsule_volume(x) for x in parts)
        if v < gain * best_v:
            best, best_v = parts, v
    return best


def fit_spheres_inset(pts: np.ndarray, nrm: np.ndarray, cell: float) -> List[Dict[str, object]]:
    """表面按（体素格 × 法向朝向）分块，每块一个球，球心沿法向往里挪，减少在平面上往外鼓。

    往里挪 h 后，覆盖横向半宽 ρ 的块需要半径 √(ρ²+h²)，外鼓约 √(ρ²+h²) − h；h 取 ρ 与「到对面表面
    距离 × 0.45」中较小的（别从薄壁的另一面鼓出去）。
    """
    c_all = pts.mean(axis=0)
    if float(np.mean(np.einsum("ij,ij->i", pts - c_all, nrm))) < 0:
        nrm = -nrm
    vox = np.floor(pts / cell).astype(np.int64)
    dirs = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]], dtype=float)
    bucket = np.argmax(nrm @ dirs.T, axis=1)
    keys = np.c_[vox, bucket]
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    out: List[Dict[str, object]] = []
    for g in range(int(inv.max()) + 1):
        m = inv == g
        P = pts[m]
        nb = nrm[m].sum(axis=0)
        nl = float(np.linalg.norm(nb))
        c0 = P.mean(axis=0)
        if nl < 1e-9:
            out.append({"a": c0, "b": c0, "r": float(np.linalg.norm(P - c0, axis=1).max())})
            continue
        n = nb / nl
        rel = P - c0
        lat = rel - np.outer(rel @ n, n)
        rho = float(np.linalg.norm(lat, axis=1).max()) if len(P) > 1 else 0.0
        # 对面表面：在 c0 后方、横向距离 < ρ+cell、法向大致反向的点
        q = pts - c0
        depth = -(q @ n)
        latq = np.linalg.norm(q + np.outer(depth, n), axis=1)
        opp = (depth > 1e-4) & (latq < rho + 0.5 * cell) & ((nrm @ n) < -0.3)
        thick = float(depth[opp].min()) if np.any(opp) else 3.0 * max(rho, cell)
        h = max(0.0, min(rho, 0.45 * thick))
        c = c0 - h * n
        r = float(np.linalg.norm(P - c, axis=1).max())
        out.append({"a": c, "b": c, "r": r})
    return out


def capsule_volume(c: Dict[str, object]) -> float:
    r = float(c["r"])
    L = float(np.linalg.norm(np.asarray(c["b"]) - np.asarray(c["a"])))
    return np.pi * r * r * L + 4.0 / 3.0 * np.pi * r ** 3


# ---------------------------------------------------------------- 距离
def segment_distances(p1: np.ndarray, q1: np.ndarray, p2: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """成批算线段 p1-q1 与 p2-q2 的最近距离（形状 (n,3) → (n,)）。Ericson《Real-Time Collision Detection》5.1.9。"""
    eps = 1e-12
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = np.einsum("ij,ij->i", d1, d1)
    e = np.einsum("ij,ij->i", d2, d2)
    f = np.einsum("ij,ij->i", d2, r)
    c = np.einsum("ij,ij->i", d1, r)
    b = np.einsum("ij,ij->i", d1, d2)
    denom = a * e - b * b
    a_s = np.where(a > eps, a, 1.0)
    e_s = np.where(e > eps, e, 1.0)
    # 一般情形
    s = np.where(denom > eps, np.clip((b * f - c * e) / np.where(denom > eps, denom, 1.0), 0.0, 1.0), 0.0)
    t = (b * s + f) / e_s
    s = np.where(t < 0.0, np.clip(-c / a_s, 0.0, 1.0), np.where(t > 1.0, np.clip((b - c) / a_s, 0.0, 1.0), s))
    t = np.clip(t, 0.0, 1.0)
    # 退化：第一段是点 / 第二段是点
    p1pt = a <= eps
    p2pt = e <= eps
    s = np.where(p1pt, 0.0, s)
    t = np.where(p1pt, np.clip(f / e_s, 0.0, 1.0), t)
    s = np.where(p2pt & ~p1pt, np.clip(-c / a_s, 0.0, 1.0), s)
    t = np.where(p2pt, 0.0, t)
    c1 = p1 + d1 * s[:, None]
    c2 = p2 + d2 * t[:, None]
    return np.linalg.norm(c1 - c2, axis=1)
