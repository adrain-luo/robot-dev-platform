"""看一眼棋盘，把角点变成躯干里的点，再让 TCP 逐个抬到上方、下去摸、再抬起。

任务不拿 App：运控客户端、只读运动学和其余几样外部能力都经 BoardTouchEnv 注入。
走点期间租着这只手；急停撤销租约，任务在下一步停下。
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import cv2
import numpy as np

from g1_contracts.motion import MotionRejected
from robot_platform.core import instance as instances
from robot_platform.geometry import fit_plane, sample_point


def log_dir() -> Path:
    return instances.path("board_touch")
# 明显不平整：角点离开拟合平面超过这些值就停，不往下摸。
PLANE_RMS_MM = 10.0
PLANE_MAX_MM = 20.0
# 理想网格把角点挪得太远，说明横竖方向和实测对不上。
IDEAL_MAX_MM = 20.0


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


@dataclass(frozen=True)
class BoardTouchEnv:
    """棋盘触摸用到的外部能力（由 App 注入）。"""

    motion: Callable[[], Any]
    """运控客户端（进程内是 MotionCore）；只读模式下抛 PermissionErrorMotion。"""
    kin: Any
    """只读运动学（RobotKinematics）：kin.arms[arm].model 做 FK / IK。"""
    stereo: Callable[[], Tuple[Any, Any, Callable[[], Any]]]
    """看棋盘用：(左目→躯干 4x4 变换, 双目相机, 立体匹配器工厂)；没有手眼结果 / 不是双目时抛 ValueError。"""
    tcp_for_arm: Callable[[str], Tuple[str, Any]]
    """这只手当前生效的 TCP：(名字, 法兰系偏移 m)。"""
    work_pose_lock: Callable[[str, str], Tuple[Optional[np.ndarray], Optional[Any]]]
    """这只手、这个 TCP 下锁定的工作姿态 (R_lock, q_seed)；没锁为 (None, None)。"""
    work_pose_free_axis: Callable[[], Optional[np.ndarray]]
    """锁定姿态的不约束转轴；没有为 None。"""
    work_pose_info: Callable[[], Tuple[str, str]]
    """(当前工作姿态名, 位姿 IK 策略 single / robust / luo)。"""


class BoardTouch:
    def __init__(self, env: BoardTouchEnv) -> None:
        self.env = env
        self._lease = None     # 走点期间这只手的租约
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
        self.arm = ""          # 正在摸的那只手
        self._tcp_name = ""
        self._tcp_m: Optional[np.ndarray] = None

    def status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "ok": True,
                "running": self.running,
                "arm": self.arm,
                "phase": self.phase,
                "error": self.error,
                "n": 0 if self.points is None else int(self.points.shape[0]),
                "point_xyz": self.point_xyz,
                "log": list(self.logs[-12:]),
                "log_path": self.log_path,
            }

    def request_stop(self) -> None:
        """停止按钮：不再下新目标（手由调用方锁住）。急停不用它：急停撤销租约，任务自己就停。"""
        lease = self._lease
        if lease is not None:
            lease.cancel()
        if self.running:
            self._log("收到停止")

    def _stopped(self) -> bool:
        lease = self._lease
        return lease is None or lease.cancelled.is_set()

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
        from robot_platform.calib.handeye.detection import parse_board_size

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
            T, cam, matcher_for = self.env.stereo()
            pair = cam.grab_pair(3.0)
            if pair is None:
                raise RuntimeError(cam.last_error or "没有画面")
            matcher = matcher_for()
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
                p = sample_point(points_m, float(xy[0]), float(xy[1]))
                if p is None:
                    missed += 1
                    raw.append(np.full(3, np.nan))
                else:
                    raw.append(p)
            pts = np.asarray(raw, dtype=float)
            self._log(f"有深度的角点 {pts.shape[0] - missed}/{pts.shape[0]}")
            if missed:
                raise RuntimeError(f"{missed} 个角点没有深度，棋盘可能出了匹配范围")
            plane = fit_plane(pts)
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

    def start(self, above_m: float, down_m: float, up_m: float, arm: str = "right") -> Dict[str, Any]:
        if self.running:
            return {"ok": False, "error": "已经在触摸"}
        if self.points is None or self.points.shape[0] < 1:
            return {"ok": False, "error": "先看一眼棋盘"}
        try:
            motion = self.env.motion()
        except PermissionError:
            return {"ok": False, "error": "未开 --allow-real-motion，不能走点"}
        if arm not in motion.arm_names:
            return {"ok": False, "error": "说明用哪只手摸（arm = left / right）"}
        try:
            tcp_name, tcp_m = self.env.tcp_for_arm(arm)
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}
        st = motion.arm_view_status(arm)
        if not st.get("engaged") or st.get("float"):
            return {"ok": False, "error": "先接管，并让这只手退出拖拽"}
        if above_m <= 0 or down_m <= 0 or up_m <= 0:
            return {"ok": False, "error": "上方、下降、抬升都要大于 0"}
        if down_m > above_m + 1e-6:
            return {"ok": False, "error": "下降超过了上方高度，爪尖会穿过棋盘"}
        self.error = None
        self.arm = arm
        self._tcp_name = tcp_name
        self._tcp_m = np.asarray(tcp_m, dtype=float).reshape(3)
        if self._fh is None:
            self._open_log()
        # 租约：占住这只手直到摸完（被占用、急停锁定时这里就拒绝）；线程起不来就马上还
        try:
            lease = motion.lease_arm(arm, "棋盘触摸")
        except MotionRejected as exc:
            return {"ok": False, "error": str(exc)}
        self._lease = lease
        self.running = True
        self._thread = threading.Thread(
            target=self._run,
            args=(float(above_m), float(down_m), float(up_m)),
            name="board-touch",
            daemon=True,
        )
        try:
            self._thread.start()
        except BaseException:
            self.running = False
            lease.release()
            raise
        return self.status()

    def _run(self, above_m: float, down_m: float, up_m: float) -> None:
        lease = self._lease
        try:
            tcp = self._tcp_m
            model = self.env.kin.arms[self.arm].model
            pts = self.points
            total = int(pts.shape[0])
            up = np.array([0.0, 0.0, 1.0])
            R_lock, _ = self.env.work_pose_lock(self.arm, self._tcp_name)
            pose_note = f"，工作姿态锁定「{self.env.work_pose_info()[0]}」" if R_lock is not None else ""
            side = "左" if self.arm == "left" else "右"
            self._log(
                f"开始触摸 {total} 点：上方 {above_m * 1000:.0f} mm，下降 {down_m * 1000:.0f} mm，"
                f"再抬 {up_m * 1000:.0f} mm。{side}臂 TCP {self._tcp_name}{pose_note}"
            )
            q = np.asarray(lease.arm_sample(wait=False)["q_meas"], dtype=float)
            for i, p in enumerate(pts):
                if self._stopped():
                    raise RuntimeError("已停止")
                high = p + up * above_m
                low = high - up * down_m
                back = low + up * up_m
                self._log(f"第 {i + 1}/{total} 到上方 {above_m * 1000:.0f} mm")
                q = self._line(lease, model, tcp, q, high, f"第 {i + 1} 点上方")
                self._log(f"第 {i + 1}/{total} 下降 {down_m * 1000:.0f} mm")
                q = self._line(lease, model, tcp, q, low, f"第 {i + 1} 点下降")
                self._log(f"第 {i + 1}/{total} 抬升 {up_m * 1000:.0f} mm")
                q = self._line(lease, model, tcp, q, back, f"第 {i + 1} 点抬升")
            self._log("全部角点走完")
        except Exception as exc:
            self.error = str(exc)
            self._log(f"中止：{exc}")
        finally:
            lease.release()
            with self._lock:
                self.running = False
            if self._fh is not None:
                self._fh.close()
                self._fh = None

    def _line(self, lease, model, tcp: np.ndarray, q: np.ndarray, dest: np.ndarray, label: str) -> np.ndarray:
        """沿直线走，大约每 1 cm 一个点，避免关节空间把爪尖甩出竖线。"""
        from robot_platform.tasks.spatial_points import (
            solve_pose_ik_by_mode, solve_tcp_ik, solve_tcp_pose_ik, solve_tcp_pose_ik_robust,
        )

        R_lock, q_seed = self.env.work_pose_lock(self.arm, self._tcp_name)
        free_axis = self.env.work_pose_free_axis() if R_lock is not None else None
        extra = [] if q_seed is None else [q_seed]
        t, R = model.fk(q)
        start = t + R @ tcp
        dist = float(np.linalg.norm(dest - start))
        steps = max(1, int(np.ceil(dist / 0.01)))
        for s in range(1, steps + 1):
            if self._stopped():
                lease.hold()
                raise RuntimeError("急停，停在当前姿态")
            target = start + (dest - start) * (s / steps)
            if R_lock is not None:
                mode = self.env.work_pose_info()[1] or "robust"
                if mode == "single":
                    q_next, miss, ori = solve_tcp_pose_ik(model, q, tcp, target, R_lock, free_axis=free_axis)
                elif mode == "luo":
                    # 每厘米一步：只从上一步出发（连续），不行再加种子
                    q_next, miss, ori = solve_pose_ik_by_mode(
                        "luo", model, q, tcp, target, R_lock, n_random=0, n_ok_stop=1, free_axis=free_axis,
                    )
                    if miss > 5.0 or ori > 8.0:
                        q_next, miss, ori = solve_pose_ik_by_mode(
                            "luo", model, q, tcp, target, R_lock,
                            extra_seeds=extra, n_random=8, n_ok_stop=3, free_axis=free_axis,
                        )
                else:
                    q_next, miss, ori = solve_tcp_pose_ik(model, q, tcp, target, R_lock, free_axis=free_axis)
                    if miss > 5.0 or ori > 8.0:
                        q_next, miss, ori = solve_tcp_pose_ik_robust(
                            model, q, tcp, target, R_lock,
                            extra_seeds=extra, n_random=8, n_ok_stop=5, free_axis=free_axis,
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
                    f"{label} 这一厘米要转 {lease.layout.joint_names[ji]} {np.degrees(float(jump[ji])):.0f}°"
                )
            q = q_next
            lease.move_to(q)
            if not self._wait(lease, q):
                lease.hold()
                if self._stopped():
                    raise RuntimeError("急停，停在当前姿态")
                raise RuntimeError(f"{label}到位超时")
        return q

    def _wait(self, lease, q: np.ndarray) -> bool:
        """关节停住就算这一小段结束。差在约 8° 以内继续，更大就说明没走到规划姿态。"""
        names = list(lease.layout.joint_names)
        still_since = None
        while True:
            if self._stopped():
                return False
            st = lease.status()
            if not st.get("engaged") or st.get("float"):
                return False
            samp = lease.arm_sample(wait=False)
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
