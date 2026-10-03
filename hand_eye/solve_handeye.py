#!/usr/bin/env python3
"""离线求 T_cam2base。末端默认左手腕 yaw，内参 yaml 必须传入。

    python solve_handeye.py --data data/20261003_120000 \\
        --intrinsics /path/to/stereo_calibration.yaml --square-size 25
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ARM_CONTROL = HERE.parent / "arm_control"
for _p in (str(HERE), str(ARM_CONTROL)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from detection import parse_board_size  # noqa: E402
from model.urdf_fk import URDFRobotModel  # noqa: E402
from session import load_meta  # noqa: E402
from solve_core import (  # noqa: E402
    SolveOptions,
    load_intrinsics,
    object_points_m,
    solve_eye,
    stereo_cross_check,
)

DEFAULT_URDF = ARM_CONTROL / "config" / "g1_d.urdf"
DEFAULT_TIP = "left_wrist_yaw_link"
DEFAULT_BASE = "torso_link"


def solve_dir(data_dir: Path, urdf: Path, opt: SolveOptions, board_size, eyes) -> dict:
    if not (data_dir / "joints").is_dir():
        raise FileNotFoundError(f"{data_dir} 里没有 joints/")
    model = URDFRobotModel(str(urdf), opt.base_link, opt.tip_link)
    print(f"[fk] base={opt.base_link} tip={opt.tip_link} joints={model.joint_names}")
    objp = object_points_m(board_size, opt.square_size_mm)
    yaml_path = Path(opt.intrinsics)
    results = {}
    for eye in eyes:
        K, dist = load_intrinsics(yaml_path, eye)
        res = solve_eye(eye, data_dir, K, dist, objp, board_size, model, opt)
        if res is None:
            continue
        results[eye] = res
        out = data_dir / f"handeye_result_{eye}.json"
        out.write_text(json.dumps(res, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"[{eye}] 已写入 {out}")
    if not results:
        raise RuntimeError("没有一眼算出结果")
    cross = stereo_cross_check(results)
    combined = dict(results)
    if cross:
        combined.update(cross)
        print(f"[双目] 右相对左基线 {cross['stereo_baseline_mm']:.2f} mm")
        path = data_dir / "handeye_result.json"
        path.write_text(json.dumps(combined, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"[OK] {path}")
    return combined


def main() -> int:
    p = argparse.ArgumentParser(description="眼在手外求解（默认左手腕 yaw）")
    p.add_argument("--data", required=True, help="采集会话目录（left/ right/ joints/）")
    p.add_argument("--intrinsics", default="", help="已有 stereo_calibration.yaml；留空则用本次左右图估计")
    p.add_argument("--square-size", type=float, required=True, help="格子边长，毫米")
    p.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    p.add_argument("--eye", default="both", choices=["left", "right", "both"])
    p.add_argument("--base-link", default=None)
    p.add_argument("--tip-link", default=None)
    p.add_argument("--board-size", default="11x8")
    p.add_argument("--method", default="park", choices=["tsai", "park", "horaud", "andreff", "daniilidis"])
    p.add_argument("--no-reject", action="store_true")
    p.add_argument("--max-trans-res-mm", type=float, default=8.0)
    p.add_argument("--max-rot-res-deg", type=float, default=2.5)
    p.add_argument("--reject-iters", type=int, default=5)
    args = p.parse_args()
    data_dir = Path(args.data)
    meta = load_meta(data_dir)
    board_size = parse_board_size(args.board_size)
    intr = str(args.intrinsics or "").strip()
    if not intr:
        from intrinsics import calibrate_session
        fit = calibrate_session(data_dir, board_size, args.square_size)
        intr = fit["yaml"]
    opt = SolveOptions(
        method=args.method,
        no_reject=args.no_reject,
        max_trans_res_mm=args.max_trans_res_mm,
        max_rot_res_deg=args.max_rot_res_deg,
        reject_iters=args.reject_iters,
        base_link=args.base_link or meta.get("base_link") or DEFAULT_BASE,
        tip_link=args.tip_link or meta.get("tip_link") or DEFAULT_TIP,
        square_size_mm=args.square_size,
        intrinsics=intr,
    )
    eyes = ["left", "right"] if args.eye == "both" else [args.eye]
    solve_dir(data_dir, args.urdf, opt, board_size, eyes)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
