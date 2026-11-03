"""看一眼棋盘，把角点变成躯干里的点，再让 TCP 逐个抬到上方、下去摸、再抬起。"""

from __future__ import annotations

import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np

import instance as instances


def log_dir() -> Path:
    return instances.path("board_touch")
# 明显不平整：角点离开拟合平面超过这些值就停，不往下摸。
PLANE_RMS_MM = 10.0
PLANE_MAX_MM = 20.0
# 理想网格把角点挪得太远，说明横竖方向和实测对不上。
IDEAL_MAX_MM = 20.0


def _sample_point(points: np.ndarray, x: float, y: float) -> Optional[np.ndarray]:
    h, w = points.shape[:2]
    xi = int(np.clip(round(x), 0, w - 1))
    yi = int(np.clip(round(y), 0, h - 1))
    p = points[yi, xi]
    if np.isfinite(p).all() and 0.2 < float(p[2]) < 2.0:
        return p.astype(float)
    r = 4
    patch = points[max(0, yi - r):yi + r + 1, max(0, xi - r):xi + r + 1].reshape(-1, 3)
    ok = np.isfinite(patch).all(axis=1) & (patch[:, 2] > 0.2) & (patch[:, 2] < 2.0)
    if int(ok.sum()) < 5:
        return None
    return np.median(patch[ok], axis=0)


def _fit_plane(pts: np.ndarray) -> Dict[str, Any]:
    center = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - center, full_matrices=False)
    normal = vt[-1]
    dist_mm = (pts - center) @ normal * 1000.0
    flat = pts - dist_mm[:, None] / 1000.0 * normal
    return {
        "center": center,
        "normal": normal,
        "rms_mm": float(np.sqrt(np.mean(dist_mm ** 2))),
        "max_mm": float(np.max(np.abs(dist_mm))),
        "flat": flat,
    }


def _ideal_grid(flat: np.ndarray, cols: int, rows: int, normal: np.ndarray):
    """把平面上的角点收成等距、互相垂直的网格。OpenCV 角点按行优先。"""
    n = rows * cols
    A = np.zeros((3 * n, 9))
    b = np.zeros(3 * n)
    k = 0
    grid = flat.reshape(rows, cols, 3)
    for r in range(rows):
        for c in range(cols):
            A[3 * k:3 * k + 3, 0:3] = np.eye(3)
            A[3 * k:3 * k + 3, 3:6] = float(c) * np.eye(3)
            A[3 * k:3 * k + 3, 6:9] = float(r) * np.eye(3)
            b[3 * k:3 * k + 3] = grid[r, c]
            k += 1
    sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    origin, u, v = sol[0:3], sol[3:6], sol[6:9]
    su = float(np.linalg.norm(u))
    u_hat = u / su
    v_side = v - float(np.dot(v, u_hat)) * u_hat
    sv = float(np.linalg.norm(v_side))
    if su < 1e-4 or sv < 1e-4:
        raise ValueError("角点挤在一起，排不出横竖方向")
    v_hat = v_side / sv
    # 网格要落在拟合平面上。
    u_hat = u_hat - float(np.dot(u_hat, normal)) * normal
    v_hat = v_hat - float(np.dot(v_hat, normal)) * normal
    u_hat = u_hat / np.linalg.norm(u_hat)
    v_hat = v_hat - float(np.dot(v_hat, u_hat)) * u_hat
    v_hat = v_hat / np.linalg.norm(v_hat)
    spacing = 0.5 * (su + sv)
    ideal = np.zeros_like(grid)
    for r in range(rows):
        for c in range(cols):
            ideal[r, c] = origin + c * spacing * u_hat + r * spacing * v_hat
    return ideal.reshape(-1, 3), spacing


class BoardTouch:
    def __init__(self, app) -> None:
        self.app = app
        self.stop = threading.Event()
        self._lock = threading.Lock()
        self.running = False
        self.phase = ""
        self.error: Optional[str] = None
        self.logs: List[str] = []
        self.log_path = ""
        self.points: Optional[np.ndarray] = None
        self.point_xyz: Optional[list] = None
        self._fh = None
        self._thread: Optional[threading.Thread] = None

    def status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "ok": True,
                "running": self.running,
                "phase": self.phase,
                "error": self.error,
                "n": 0 if self.points is None else int(self.points.shape[0]),
                "point_xyz": self.point_xyz,
                "log": list(self.logs[-12:]),
                "log_path": self.log_path,
            }

    def request_stop(self) -> None:
        self.stop.set()
        if self.running:
            self._log("收到停止")

    def _log(self, msg: str) -> None:
        line = datetime.now().strftime("%H:%M:%S ") + msg
        with self._lock:
            self.phase = msg
            self.logs.append(line)
            self.logs = self.logs[-80:]
            fh = self._fh
        if fh is not None:
            fh.write(line + "\n")
            fh.flush()

    def _open_log(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None
        log_dir().mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = log_dir() / f"{stamp}.log"
        self._fh = path.open("a", encoding="utf-8")
        self.log_path = str(path)
        self._log(f"日志 {path}")

    def look(self, board: str, ideal: bool) -> Dict[str, Any]:
        from service import load_config
        from detection import parse_board_size

        if self.running:
            return {"ok": False, "error": "正在触摸，先停掉"}
        self.error = None
        self.points = None
        self.point_xyz = None
        self._open_log()
        try:
            cols, rows = parse_board_size(board)
            if cols < 3 or rows < 3:
                raise ValueError("棋盘至少 3x3 个内角点")
            cfg = load_config()
            he = self.app.handeye
            T = he._left_cam_to_torso(cfg)
            if T is None:
                raise ValueError("先调用一组手眼结果，角点才能变到躯干上")
            cam = he._camera(cfg)
            pair = cam.grab_pair(3.0)
            if pair is None:
                raise RuntimeError(cam.last_error or "没有画面")
            matcher = he._matcher_for(he._calib_yaml(cfg))
            rect_l, _disp, points_m = matcher.match(pair[0], pair[1])
            gray = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCornersSB(gray, (cols, rows))
            if not found or corners is None:
                raise RuntimeError(f"校正后的左目没有找到 {cols}x{rows} 棋盘")
            vis = rect_l.copy()
            cv2.drawChessboardCorners(vis, (cols, rows), corners, True)
            img_path = Path(self.log_path).with_suffix(".jpg")
            cv2.imwrite(str(img_path), vis)
            self._log(f"棋盘 {cols}x{rows}，角点图 {img_path.name}")
            raw = []
            missed = 0
            for i, xy in enumerate(corners.reshape(-1, 2)):
                p = _sample_point(points_m, float(xy[0]), float(xy[1]))
                if p is None:
                    missed += 1
                    raw.append(np.full(3, np.nan))
                else:
                    raw.append(p)
            pts = np.asarray(raw, dtype=float)
            self._log(f"有深度的角点 {pts.shape[0] - missed}/{pts.shape[0]}")
            if missed:
                raise RuntimeError(f"{missed} 个角点没有深度，棋盘可能出了匹配范围")
            plane = _fit_plane(pts)
            self._log(f"平面残差 平均 {plane['rms_mm']:.1f} mm，最大 {plane['max_mm']:.1f} mm")
            if plane["rms_mm"] > PLANE_RMS_MM or plane["max_mm"] > PLANE_MAX_MM:
                raise RuntimeError(
                    f"棋盘明显不平整：平均 {plane['rms_mm']:.1f} mm，最大 {plane['max_mm']:.1f} mm"
                )
            used = plane["flat"]
            self._log("角点已投到拟合平面上")
            if ideal:
                ideal_pts, spacing = _ideal_grid(used, cols, rows, plane["normal"])
                shift = np.linalg.norm(ideal_pts - used, axis=1) * 1000.0
                self._log(
                    f"理想网格间距 {spacing * 1000.0:.1f} mm，角点最大挪动 {float(shift.max()):.1f} mm"
                )
                if float(shift.max()) > IDEAL_MAX_MM:
                    raise RuntimeError(
                        f"理想网格和实测差 {float(shift.max()):.1f} mm，横竖对不齐。关掉理想对齐再看一眼"
                    )
                used = ideal_pts
            else:
                self._log("未做横竖理想对齐，用平面上的实测角点")
            homo = np.concatenate([used, np.ones((used.shape[0], 1))], axis=1)
            torso = (np.asarray(T, dtype=float) @ homo.T).T[:, :3]
            self.points = torso
            self.point_xyz = np.round(torso, 4).reshape(-1).tolist()
            self._log(f"得到 {torso.shape[0]} 个躯干坐标，已画到中央画面，准备按这个顺序触摸")
            return self.status()
        except Exception as exc:
            self.error = str(exc)
            self._log(f"失败：{exc}")
            out = self.status()
            out["ok"] = False
            out["error"] = str(exc)
            return out

    def start(self, above_m: float, down_m: float, up_m: float) -> Dict[str, Any]:
        if self.running:
            return {"ok": False, "error": "已经在触摸"}
        if self.points is None or self.points.shape[0] < 1:
            return {"ok": False, "error": "先看一眼棋盘"}
        tcp = self.app.tcp
        if not tcp.arm or not tcp.result or tcp.result.get("tcp_m") is None:
            return {"ok": False, "error": "先调用一个 TCP 结果"}
        loop = self.app.loop
        if loop is None:
            return {"ok": False, "error": "未开 --allow-real-motion，不能走点"}
        if loop.arm != tcp.arm:
            side = "左" if tcp.arm == "left" else "右"
            return {"ok": False, "error": f"TCP 是{side}臂的，先把控制臂换成{side}臂"}
        st = loop.status()
        if not st.get("engaged") or st.get("float"):
            return {"ok": False, "error": "先接管，并退出拖拽"}
        if above_m <= 0 or down_m <= 0 or up_m <= 0:
            return {"ok": False, "error": "上方、下降、抬升都要大于 0"}
        if down_m > above_m + 1e-6:
            return {"ok": False, "error": "下降超过了上方高度，爪尖会穿过棋盘"}
        self.stop.clear()
        self.error = None
        if self._fh is None:
            self._open_log()
        self.running = True
        self._thread = threading.Thread(
            target=self._run,
            args=(float(above_m), float(down_m), float(up_m)),
            name="board-touch",
            daemon=True,
        )
        self._thread.start()
        return self.status()

    def _run(self, above_m: float, down_m: float, up_m: float) -> None:
        try:
            loop = self.app.loop
            tcp = np.asarray(self.app.tcp.result["tcp_m"], dtype=float)
            model = self.app.kin.arms[loop.arm].model
            pts = self.points
            total = int(pts.shape[0])
            up = np.array([0.0, 0.0, 1.0])
            wp = getattr(self.app, "work_pose", None)
            pose_note = ""
            if wp is not None and wp.locked:
                pose_note = f"，工作姿态锁定「{wp.active_name}」"
            self._log(
                f"开始触摸 {total} 点：上方 {above_m * 1000:.0f} mm，下降 {down_m * 1000:.0f} mm，"
                f"再抬 {up_m * 1000:.0f} mm。TCP {self.app.tcp.active_name or loop.arm}{pose_note}"
            )
            q = np.asarray(loop.arm_sample(wait=False)["q_meas"], dtype=float)
            for i, p in enumerate(pts):
                if self.stop.is_set():
                    raise RuntimeError("已停止")
                high = p + up * above_m
                low = high - up * down_m
                back = low + up * up_m
                self._log(f"第 {i + 1}/{total} 到上方 {above_m * 1000:.0f} mm")
                q = self._line(loop, model, tcp, q, high, f"第 {i + 1} 点上方")
                self._log(f"第 {i + 1}/{total} 下降 {down_m * 1000:.0f} mm")
                q = self._line(loop, model, tcp, q, low, f"第 {i + 1} 点下降")
                self._log(f"第 {i + 1}/{total} 抬升 {up_m * 1000:.0f} mm")
                q = self._line(loop, model, tcp, q, back, f"第 {i + 1} 点抬升")
            self._log("全部角点走完")
        except Exception as exc:
            self.error = str(exc)
            self._log(f"中止：{exc}")
        finally:
            with self._lock:
                self.running = False
            if self._fh is not None:
                self._fh.close()
                self._fh = None

    def _line(self, loop, model, tcp: np.ndarray, q: np.ndarray, dest: np.ndarray, label: str) -> np.ndarray:
        """沿直线走，大约每 1 cm 一个点，避免关节空间把爪尖甩出竖线。"""
        from spatial_points import solve_tcp_ik, solve_tcp_pose_ik, solve_tcp_pose_ik_robust

        wp = getattr(self.app, "work_pose", None)
        if wp is not None:
            wp.unlock_if_tcp_mismatch(getattr(self.app.tcp, "active_name", "") or "")
        R_lock = wp.R_lock if (wp is not None and wp.locked) else None
        extra = []
        if wp is not None and wp.q_seed is not None:
            extra.append(wp.q_seed)
        t, R = model.fk(q)
        start = t + R @ tcp
        dist = float(np.linalg.norm(dest - start))
        steps = max(1, int(np.ceil(dist / 0.01)))
        self.app._work_pose_motion = True
        try:
            for s in range(1, steps + 1):
                if self.stop.is_set():
                    loop.estop_hold()
                    raise RuntimeError("急停，停在当前姿态")
                target = start + (dest - start) * (s / steps)
                if R_lock is not None:
                    mode = getattr(wp, "ik_mode", "robust") if wp is not None else "robust"
                    if mode == "single":
                        q_next, miss, ori = solve_tcp_pose_ik(model, q, tcp, target, R_lock)
                    else:
                        q_next, miss, ori = solve_tcp_pose_ik(model, q, tcp, target, R_lock)
                        if miss > 5.0 or ori > 8.0:
                            q_next, miss, ori = solve_tcp_pose_ik_robust(
                                model, q, tcp, target, R_lock,
                                extra_seeds=extra, n_random=8, n_ok_stop=5,
                            )
                    if miss > 5.0 or ori > 8.0:
                        raise RuntimeError(
                            f"{label}到不了，还差 {miss:.1f} mm / {ori:.1f}°（工作姿态锁定）"
                        )
                else:
                    q_next, miss = solve_tcp_ik(model, q, tcp, target)
                    if miss > 5.0:
                        raise RuntimeError(f"{label}到不了，还差 {miss:.1f} mm")
                jump = np.abs(q_next - q)
                ji = int(np.argmax(jump))
                if float(jump[ji]) > 0.5:
                    self._log(
                        f"{label} 这一厘米要转 {loop.layout.joint_names[ji]} {np.degrees(float(jump[ji])):.0f}°"
                    )
                q = q_next
                loop.set_target(q)
                if not self._wait(loop, q):
                    loop.estop_hold()
                    if self.stop.is_set():
                        raise RuntimeError("急停，停在当前姿态")
                    raise RuntimeError(f"{label}到位超时")
        finally:
            self.app._work_pose_motion = False
        return q

    def _wait(self, loop, q: np.ndarray) -> bool:
        """关节停住就算这一小段结束。差在约 8° 以内继续，更大就说明没走到规划姿态。"""
        names = list(loop.layout.joint_names)
        still_since = None
        while True:
            if self.stop.is_set():
                return False
            st = loop.status()
            if not st.get("engaged") or st.get("float"):
                return False
            samp = loop.arm_sample(wait=False)
            meas = np.asarray(samp["q_meas"], dtype=float)
            dq = np.asarray(samp["dq"], dtype=float)
            err = np.abs(meas - q)
            gap = float(np.max(err))
            idx = int(np.argmax(err))
            speed_now = float(np.max(np.abs(dq)))
            now = time.monotonic()
            if speed_now < 0.05:
                if still_since is None:
                    still_since = now
                elif now - still_since >= 0.4:
                    if gap < 0.15:
                        if gap >= 0.03:
                            self._log(f"已停住，{names[idx]} 和规划还差 {np.degrees(gap):.1f}°，继续")
                        return True
                    self._log(f"已停住但差太多：{names[idx]} 还差 {np.degrees(gap):.1f}°")
                    return False
            else:
                still_since = None
            time.sleep(0.05)
