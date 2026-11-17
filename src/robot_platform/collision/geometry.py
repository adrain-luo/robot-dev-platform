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


_DIRS2 = np.c_[np.cos(np.linspace(0, 2 * np.pi, 180, endpoint=False)),
               np.sin(np.linspace(0, 2 * np.pi, 180, endpoint=False))]


def _extreme2d(p: np.ndarray) -> np.ndarray:
    """二维点集在 180 个方向上的最远点（凸包顶点的子集，最小包围圆只取决于它们，向量化很快）。"""
    idx = np.unique(np.argmax(p @ _DIRS2.T, axis=0))
    return p[idx]


def min_enclosing_circle(p: np.ndarray, rng: Optional[np.random.Generator] = None,
                         iters: int = 300) -> Tuple[np.ndarray, float]:
    """二维近似最小包围圆（Bădoiu–Clarkson：圆心每步往最远点挪 1/(k+1)），先取凸包减点。
    半径取圆心到最远点，所以一定包住全部点；比最优大不到 1%。"""
    p = np.asarray(p, dtype=float)
    h = _extreme2d(p)
    c = h.mean(axis=0)
    for k in range(1, iters + 1):
        far = h[int(np.argmax(((h - c) ** 2).sum(axis=1)))]
        c = c + (far - c) / (k + 1)
    # 半径按全部点算：方向采样之间漏掉的顶点也包住
    return c, float(np.sqrt(((p - c) ** 2).sum(axis=1)).max())


def _capsule_on_axis(pts: np.ndarray, axis: np.ndarray, rng) -> Dict[str, object]:
    """给定轴向：横截面用最小包围圆定轴线位置和半径，再按两端收线段，最后按点到线段距离定半径。"""
    axis = axis / np.linalg.norm(axis)
    tmp = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(axis, tmp)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(axis, e1)
    uv = np.c_[pts @ e1, pts @ e2]
    c2, r0 = min_enclosing_circle(uv, rng)
    base = c2[0] * e1 + c2[1] * e2
    proj = pts @ axis
    rho = np.linalg.norm(uv - c2, axis=1)
    pmin, pmax = float(proj.min()), float(proj.max())
    # 两端各往里收 s：收得多线段短，但端面边缘那圈点离线段端点远（平端面收 r0 时半径会涨到 1.41 倍）。
    # 每端在 0..r0 里试几档，取体积最小的组合
    fr = np.linspace(0.0, 1.0, 9)
    need_lo, need_hi = [], []
    for f in fr:
        e = pmin + f * r0
        dz = np.maximum(e - proj, 0.0)
        need_lo.append(float(np.sqrt(dz * dz + rho * rho)[proj < e].max()) if np.any(proj < e) else 0.0)
        e2 = pmax - f * r0
        dz2 = np.maximum(proj - e2, 0.0)
        need_hi.append(float(np.sqrt(dz2 * dz2 + rho * rho)[proj > e2].max()) if np.any(proj > e2) else 0.0)
    best = None
    for i, fl in enumerate(fr):
        for j, fh in enumerate(fr):
            lo, hi = pmin + fl * r0, pmax - fh * r0
            if lo > hi:
                continue
            r = max(r0, need_lo[i], need_hi[j])
            v = np.pi * r * r * (hi - lo) + 4.0 / 3.0 * np.pi * r ** 3
            if best is None or v < best[0]:
                best = (v, lo, hi)
    if best is None:
        lo = hi = 0.5 * (pmin + pmax)
    else:
        _, lo, hi = best
    a = base + lo * axis
    b = base + hi * axis
    r = float(point_segment_dist(pts, a, b).max())
    return {"a": a, "b": b, "r": r}


def _best_axis(pts: np.ndarray, rng, n_dirs: int = 120) -> np.ndarray:
    """PCA 主轴附近（≤ 35°）搜索，单根胶囊体体积最小的轴向。"""
    sub = pts if len(pts) <= 2000 else pts[rng.choice(len(pts), 2000, replace=False)]
    c = sub.mean(axis=0)
    _, _, vt = np.linalg.svd(sub - c, full_matrices=False)
    dirs = [vt[0], vt[1], vt[2]]
    for _ in range(n_dirs):
        v = vt[0] + np.tan(np.radians(rng.uniform(0, 35))) * (rng.normal() * vt[1] + rng.normal() * vt[2]) / 1.41
        dirs.append(v / np.linalg.norm(v))
    vols = [capsule_volume(_capsule_on_axis(sub, d, rng)) for d in dirs]
    return dirs[int(np.argmin(vols))]


def fit_capsule(pts: np.ndarray, rng: Optional[np.random.Generator] = None) -> Dict[str, object]:
    """一根胶囊体包住全部点：轴向在 PCA 主轴附近搜索，横截面取最小包围圆，取体积最小的。"""
    rng = rng or np.random.default_rng(0)
    return _capsule_on_axis(pts, _best_axis(pts, rng), rng)


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


def union_volume(caps: List[Dict[str, object]], step: float = 0.004) -> float:
    """一组胶囊体并集的体积（格点数）：并排的细胶囊体互相重叠，不能直接把体积相加。"""
    A = np.array([np.asarray(c["a"], dtype=float) for c in caps])
    B = np.array([np.asarray(c["b"], dtype=float) for c in caps])
    R = np.array([float(c["r"]) for c in caps])
    lo = np.minimum(A, B).min(axis=0) - R.max()
    hi = np.maximum(A, B).max(axis=0) + R.max()
    axes = [np.arange(lo[i], hi[i] + step, step) for i in range(3)]
    g = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
    inside = np.zeros(len(g), dtype=bool)
    for a, b, r in zip(A, B, R):
        inside |= point_segment_dist(g, a, b) <= r
    return float(inside.sum()) * step ** 3


# 多一根胶囊体，比较用的分数加罚这么多（m；检查时每根都要算距离）
PART_PENALTY = 0.0005

_FIB = None


def _fib_dirs(n: int = 48) -> np.ndarray:
    global _FIB
    if _FIB is None or len(_FIB) != n:
        i = np.arange(n) + 0.5
        phi = np.arccos(1 - 2 * i / n)
        th = np.pi * (1 + 5 ** 0.5) * i
        _FIB = np.c_[np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)]
    return _FIB


def bulge(parts: List[Dict[str, object]], surf: np.ndarray, q: float = 95.0) -> float:
    """碰撞体比实物往外鼓多少（m）：在各胶囊体表面取点，去掉落在别的胶囊体里面的（只留并集外表面），
    算到实物表面点的最近距离，取 q 分位。比并集体积更贴「看起来像不像」：空心、内部重叠都不算。"""
    D = _fib_dirs()
    A = np.array([np.asarray(c["a"], dtype=float) for c in parts])
    B = np.array([np.asarray(c["b"], dtype=float) for c in parts])
    R = np.array([float(c["r"]) for c in parts])
    pts = []
    for a, b, r in zip(A, B, R):
        L = float(np.linalg.norm(b - a))
        if L < 1e-9:
            pts.append(a + r * D)
            continue
        ax = (b - a) / L
        e1, e2 = _perp_basis(ax)
        # 端帽（各取朝外那半）+ 侧面一圈圈
        pts.append(a + r * D[(D @ ax) <= 0])
        pts.append(b + r * D[(D @ ax) >= 0])
        n_ring = max(2, int(np.ceil(L / max(r, 0.004))) + 1)
        th = np.linspace(0, 2 * np.pi, 16, endpoint=False)
        ring = np.cos(th)[:, None] * e1 + np.sin(th)[:, None] * e2
        for t in np.linspace(0, 1, n_ring):
            pts.append(a + t * (b - a) + r * ring)
    P = np.concatenate(pts)
    # 只留并集外表面上的点
    inside = np.zeros(len(P), dtype=bool)
    for a, b, r in zip(A, B, R):
        inside |= point_segment_dist(P, a, b) < r - 1e-6
    P = P[~inside]
    if len(P) == 0:
        return 0.0
    S = surf if len(surf) <= 4000 else surf[np.random.default_rng(0).choice(len(surf), 4000, replace=False)]
    _, d = _nearest(P, S)
    return float(np.percentile(d, q))


def _nearest(q: np.ndarray, pts: np.ndarray, chunk: int = 2048) -> Tuple[np.ndarray, np.ndarray]:
    """每个 q 最近的表面点下标和距离（分块暴力，float32）。"""
    P = pts.astype(np.float32)
    pn = (P * P).sum(axis=1)
    idx = np.empty(len(q), dtype=np.int64)
    dist = np.empty(len(q))
    for i in range(0, len(q), chunk):
        Q = q[i:i + chunk].astype(np.float32)
        d2 = (Q * Q).sum(axis=1)[:, None] + pn[None, :] - 2.0 * Q @ P.T
        j = np.argmin(d2, axis=1)
        idx[i:i + chunk] = j
        dist[i:i + chunk] = np.sqrt(np.maximum(d2[np.arange(len(Q)), j], 0.0))
    return idx, dist


def _perp_basis(axis: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    tmp = np.array([1.0, 0.0, 0.0]) if abs(axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = np.cross(axis, tmp)
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(axis, e1)


def _bundle(seg: np.ndarray, rng, max_k: int, gain: float) -> List[Dict[str, object]]:
    """一段点用 1..max_k 根平行的胶囊体包住：横截面 k-means 分块，每块一根。
    轴向试「单根最优轴」、三个主轴和连杆的三个坐标轴，取外鼓（bulge，每多一根加罚 PART_PENALTY）最小的。
    短粗的圆柱（肘部电机）单根胶囊体两端各多鼓一个半径；拆成几根细的，端部只鼓细的那个半径。"""
    sub = seg if len(seg) <= 4000 else seg[rng.choice(len(seg), 4000, replace=False)]
    c = sub.mean(axis=0)
    _, _, vt = np.linalg.svd(sub - c, full_matrices=False)
    # URDF 网格一般和连杆坐标轴对齐（电机轴就是关节轴），三个坐标轴也试
    axes = [_best_axis(seg, rng), vt[0], vt[1], vt[2], *np.eye(3)]
    best, best_v = None, np.inf
    for axis in axes:
        e1, e2 = _perp_basis(axis)
        uv_sub = np.c_[sub @ e1, sub @ e2]
        uv = np.c_[seg @ e1, seg @ e2]
        cur, cur_v = None, np.inf
        for k in range(1, max_k + 1):
            if k == 1:
                parts = [_capsule_on_axis(seg, axis, rng)]
            else:
                lab_s = _kmeans(uv_sub, k, rng)
                cen = np.array([uv_sub[lab_s == i].mean(axis=0) for i in range(k) if np.any(lab_s == i)])
                lab = np.argmin(((uv[:, None, :] - cen[None, :, :]) ** 2).sum(-1), axis=1)
                parts = [_capsule_on_axis(seg[lab == i], axis, rng) for i in range(len(cen)) if np.sum(lab == i) >= 4]
            v = bulge(parts, seg) + PART_PENALTY * (len(parts) - 1)
            if v < cur_v:
                cur, cur_v = parts, v
        if cur_v < best_v:
            best, best_v = cur, cur_v
    return best


def fit_capsules(pts: np.ndarray, max_parts: int = 3, gain: float = 0.93,
                 rng: Optional[np.random.Generator] = None, max_bundle: int = 9) -> List[Dict[str, object]]:
    """胶囊体包住一节连杆，两级：先沿长度 k-means 分 1..max_parts 段（弯的连杆），每段再在横截面上拆成
    1..max_bundle 根平行的细胶囊体（方截面、短粗圆柱）。都按外鼓（bulge）比，每多一根加罚 PART_PENALTY。"""
    rng = rng or np.random.default_rng(0)
    sub = pts if len(pts) <= 6000 else pts[rng.choice(len(pts), 6000, replace=False)]
    best, best_v = None, np.inf
    for k in range(1, max_parts + 1):
        if k == 1:
            segs = [pts]
        else:
            lab_s = _kmeans(sub, k, rng)
            cen = np.array([sub[lab_s == i].mean(axis=0) for i in range(k) if np.any(lab_s == i)])
            lab = np.argmin(((pts[:, None, :] - cen[None, :, :]) ** 2).sum(-1), axis=1)
            segs = [pts[lab == i] for i in range(len(cen)) if np.sum(lab == i) >= 4]
        parts = []
        for seg in segs:
            parts += _bundle(seg, rng, max_bundle, gain)
        v = bulge(parts, pts) + PART_PENALTY * (len(parts) - 1)
        if v < best_v:
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
