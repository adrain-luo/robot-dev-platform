"""罗-IK：带约束的 SLSQP 姿态 IK（紧约束到位 + 代价挑解）。

来源：罗师姐的吸盘取放项目（github adrain-luo/suction-free-yaw-ik-pick-place，G1D 盒子
~/vision_arm_control_refactor/ik_5d_suction_slsqp_optimizer.py）。思路沿用它：把 IK 写成
「最小化代价，受约束」交给 scipy SLSQP，在 7 轴冗余的一族解里按偏好挑一个。改动：

- 到位是**紧约束**：TCP 位置 ≤ POS_TOL_MM、姿态 ≤ ORI_TOL_DEG（原版是 10 mm / 30°，且位置/方向
  只是代价里的软项，常停在几 mm、几度）；
- 代价里**只有偏好**：离参照角 q0 近（少动）+ 关节靠限位中间；离限位留 LIMIT_MARGIN；
- 用解析梯度（我们的 TCP 位姿雅可比），不用 scipy 有限差分（原版每步十几次正解，慢）；
- 6D 全姿态（与单次局部 / 多初值同一套目标），free_axis 给了就不计绕该轴的转角（「可平转」）；
- 每个初值先用 DLS 拉到解附近再交给 SLSQP（SLSQP 从远处的不可行点出发常失败），
  SLSQP 之后再从它的解做几步 DLS 精修（SLSQP 会把容差用满，停在 1 mm / 1° 边界上）。

和多初值的区别：多初值在几个合格解里取「离 q0 最近」的离散一个；罗-IK 在解的流形上连续地
挑代价最小的点（少动 + 居中 + 离限位远），所以同样离 q0 近，但关节更居中、不贴限位。
需要 scipy（G1D 上有；没有时切换策略会被拒）。在 IK 子进程里算（见 tasks/ik_worker.py）。
"""

from __future__ import annotations

import time
from typing import List, Optional, Tuple

import numpy as np

from robot_platform.tasks.spatial_points import (
    _rotvec_from_R, _tcp_pose_jac, _unit_axis, nearest_about_axis, pose_ori_miss_deg, solve_tcp_pose_ik,
)

POS_TOL_MM = 1.0
ORI_TOL_DEG = 1.0
LIMIT_MARGIN = 0.05  # rad，离限位至少留这么多
W_DELTA = 1.0  # |q - q0|²
W_CENTER = 0.3  # Σ((q - 中点) / 半量程)²
LUO_BUDGET_S = 1.5
# DLS 预拉后离目标还这么远就不交给 SLSQP（不在解附近，白算）
WARM_MAX_MM = 20.0
WARM_MAX_DEG = 20.0


def scipy_available() -> bool:
    try:
        import scipy.optimize  # noqa: F401
    except Exception:  # noqa: BLE001
        return False
    return True


def solve_tcp_pose_ik_luo(
    model,
    q0,
    tcp_m,
    target_m,
    R_target,
    *,
    extra_seeds=None,
    n_random: int = 8,
    n_ok_stop: int = 2,
    time_budget_s: float = LUO_BUDGET_S,
    free_axis=None,
) -> Tuple[np.ndarray, float, float]:
    """在 IK 子进程里算罗-IK。参数与返回同 :func:`solve_tcp_pose_ik_luo_local`。"""
    from robot_platform.tasks import ik_worker

    return ik_worker.run(
        solve_tcp_pose_ik_luo_local, model, q0, tcp_m, target_m, R_target,
        extra_seeds=None if extra_seeds is None else [np.asarray(s, dtype=float) for s in extra_seeds],
        n_random=n_random, n_ok_stop=n_ok_stop, time_budget_s=time_budget_s,
        free_axis=None if free_axis is None else np.asarray(free_axis, dtype=float),
    )


def solve_tcp_pose_ik_luo_local(
    model,
    q0,
    tcp_m,
    target_m,
    R_target,
    *,
    extra_seeds=None,
    n_random: int = 8,
    n_ok_stop: int = 2,
    time_budget_s: float = LUO_BUDGET_S,
    free_axis=None,
) -> Tuple[np.ndarray, float, float]:
    """TCP 到 target、腕部姿态对齐 R_target，在满足紧约束的解里取代价最小的。返回 (q, 位置误差 mm, 姿态误差 deg)。

    初值依次：q0 → 额外种子 → 限位中点 → 随机；够 ``n_ok_stop`` 个合格解或用完 ``time_budget_s`` 就停，
    取代价最小的合格解；一个都不合格时返回残差最好的（由调用方按门限决定报不报错）。
    """
    try:
        from scipy.optimize import minimize
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError("罗-IK 需要 scipy，本机没有；请换回多初值") from exc

    deadline = time.monotonic() + max(0.0, float(time_budget_s))
    q_ref = np.asarray(q0, dtype=float).reshape(-1)
    tcp = np.asarray(tcp_m, dtype=float).reshape(3)
    target = np.asarray(target_m, dtype=float).reshape(3)
    Rd0 = np.asarray(R_target, dtype=float).reshape(3, 3)
    u = _unit_axis(free_axis)
    P = None if u is None else np.eye(3) - np.outer(u, u)
    n = model.n
    limits = np.asarray(model.joint_limits(), dtype=float)
    lo = limits[:, 0] + LIMIT_MARGIN
    hi = limits[:, 1] - LIMIT_MARGIN
    narrow = lo > hi
    lo[narrow] = limits[narrow, 0]
    hi[narrow] = limits[narrow, 1]
    center = 0.5 * (limits[:, 0] + limits[:, 1])
    half = np.where(np.abs(0.5 * (limits[:, 1] - limits[:, 0])) > 1e-9, 0.5 * (limits[:, 1] - limits[:, 0]), 1.0)
    tol_p = POS_TOL_MM / 1000.0
    tol_r = float(np.deg2rad(ORI_TOL_DEG))
    bounds = list(zip(lo, hi))

    cache: dict = {}

    def _res(q: np.ndarray):
        key = q.tobytes()
        hit = cache.get(key)
        if hit is not None:
            return hit
        pos, R, jac = _tcp_pose_jac(model, q, tcp, w_ori=1.0)
        Rd = Rd0 if u is None else nearest_about_axis(Rd0, R, u)
        ep = target - pos
        er = _rotvec_from_R(Rd @ R.T)
        jw = jac[3:6]
        if P is not None:
            er = P @ er
            jw = P @ jw
        out = (ep, er, jac[0:3], jw)
        if len(cache) > 64:
            cache.clear()
        cache[key] = out
        return out

    def _cost(q):
        d = q - q_ref
        c = (q - center) / half
        return float(W_DELTA * d @ d + W_CENTER * c @ c)

    def _cost_grad(q):
        return 2.0 * W_DELTA * (q - q_ref) + 2.0 * W_CENTER * (q - center) / (half * half)

    # 约束按容差归一化：1 - |e|²/tol² ≥ 0。d(ep)/dq = -Jp，d(er)/dq ≈ -Jw
    def _c_pos(q):
        ep = _res(q)[0]
        return float(1.0 - (ep @ ep) / (tol_p * tol_p))

    def _c_pos_grad(q):
        ep, _, jp, _ = _res(q)
        return 2.0 * (ep @ jp) / (tol_p * tol_p)

    def _c_ori(q):
        er = _res(q)[1]
        return float(1.0 - (er @ er) / (tol_r * tol_r))

    def _c_ori_grad(q):
        _, er, _, jw = _res(q)
        return 2.0 * (er @ jw) / (tol_r * tol_r)

    constraints = [
        {"type": "ineq", "fun": _c_pos, "jac": _c_pos_grad},
        {"type": "ineq", "fun": _c_ori, "jac": _c_ori_grad},
    ]

    rng = np.random.default_rng(0)

    def _seed_iter():
        yield q_ref.copy()
        for s in extra_seeds or []:
            sa = np.asarray(s, dtype=float).reshape(-1)
            if sa.size == n and np.all(np.isfinite(sa)):
                yield np.clip(sa, limits[:, 0], limits[:, 1])
        yield center.copy()
        for _ in range(max(0, int(n_random))):
            yield rng.uniform(limits[:, 0], limits[:, 1])

    ok: List[Tuple[float, np.ndarray, float, float]] = []
    best_any: Optional[Tuple[np.ndarray, float, float]] = None
    seen: set = set()
    n_stop = max(1, int(n_ok_stop))

    def _consider(q, m, o):
        nonlocal best_any
        if best_any is None or (m + 2.0 * o) < (best_any[1] + 2.0 * best_any[2]):
            best_any = (q, m, o)

    for k, seed in enumerate(_seed_iter()):
        if k > 0 and time.monotonic() > deadline:
            break
        key = tuple(np.round(seed, 4))
        if key in seen:
            continue
        seen.add(key)
        # DLS 预拉：零空间往 seed 拉，落在 seed 那一支解上
        qw, mw, ow = solve_tcp_pose_ik(model, seed, tcp, target, Rd0, w_ori=0.7, iters=100, free_axis=u)
        _consider(qw, mw, ow)
        if mw > WARM_MAX_MM or ow > WARM_MAX_DEG:
            continue
        x0 = np.clip(qw, lo, hi)
        try:
            opt = minimize(
                _cost, x0, jac=_cost_grad, method="SLSQP", bounds=bounds, constraints=constraints,
                options={"maxiter": 60, "ftol": 1e-8},
            )
            qs = np.clip(np.asarray(opt.x, dtype=float).reshape(-1), limits[:, 0], limits[:, 1])
        except Exception:  # noqa: BLE001
            qs = x0
        if not np.all(np.isfinite(qs)):
            continue
        # SLSQP 会把容差用满（停在 1 mm / 1° 的边界上）：从它的解再做几步 DLS 精修，零空间往它拉，
        # 留在它挑的那一支、那个位置，误差压到 ~0
        qp, mp, op = solve_tcp_pose_ik(model, qs, tcp, target, Rd0, w_ori=0.7, iters=40, free_axis=u)
        qp = np.clip(qp, limits[:, 0], limits[:, 1])
        for q in (qp, qs, x0):
            pos, R, _ = _tcp_pose_jac(model, q, tcp, w_ori=1.0)
            m = float(np.linalg.norm(pos - target) * 1000.0)
            o = pose_ori_miss_deg(Rd0, R, u)
            _consider(q, m, o)
            # 留 5% 余量：SLSQP 停在约束边界上
            if m <= POS_TOL_MM * 1.05 and o <= ORI_TOL_DEG * 1.05:
                ok.append((_cost(q), q, m, o))
                break
        if len(ok) >= n_stop:
            break
    if ok:
        ok.sort(key=lambda x: x[0])
        _, q, m, o = ok[0]
        return q, m, o
    assert best_any is not None
    return best_any
