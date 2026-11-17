#!/usr/bin/env python3
"""由 URDF 里的 STL 拟合碰撞体，写 robots/<机型>/collision.json。

- 身体：表面按格子 × 法向朝向分块，每块一个球，球心往里挪（少往外鼓）；躯干、头格子细，立柱中，底盘粗；
- 手臂：胶囊体（沿长度分段 × 横截面并排）和一组球各拟一遍，取「外鼓」小的（碰撞体外表面到实物表面的
  距离，95 分位；每个图元扣 --prim-cost）；
- 末端（夹爪 / 吸盘，URDF 里没有网格）：按 MANUAL 手写，已有 collision.json 里的 manual 段原样保留；
- 要查的连杆对：至少一端在手臂上、树上隔 ≥ 2 个活动关节（同一只手 ≥ 3）；再随机采样两臂关节角，
  几乎总是重叠（≥ --always）的连杆对视为结构性贴合，去掉。

用法：python tools/fit_collision.py g1d
"""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from robot_platform.collision.geometry import (  # noqa: E402
    bulge, fit_capsules, fit_spheres_inset, read_stl, sample_surface_normals,
)
from robot_platform.collision.model import CollisionModel  # noqa: E402
from robot_platform.control.urdf_fk import make_transform  # noqa: E402
from robot_platform.core.robot_profile import load_profile  # noqa: E402

# 末端执行器（URDF 无网格）：连杆坐标系里的胶囊体，可手改 collision.json 的 manual 段
MANUAL = {
    "g1d": {
        # 右手：夹爪本体 + 吸盘杆（right_dex1_tool_link 在腕 yaw 系约 (0.233, 0, -0.037)）
        "right_wrist_yaw_link": [
            {"a": [0.06, 0.0, 0.0], "b": [0.19, 0.0, 0.0], "r": 0.045, "note": "末端本体（估计）"},
            {"a": [0.19, 0.0, 0.0], "b": [0.233, 0.0, -0.037], "r": 0.02, "note": "吸盘杆（估计）"},
        ],
        "left_wrist_yaw_link": [
            {"a": [0.06, 0.0, 0.0], "b": [0.18, 0.0, 0.0], "r": 0.045, "note": "夹爪（估计）"},
        ],
    },
}
# 离手臂近、要拟合得细一点的连杆
NEAR = ("torso", "head", "logo", "yaw_link", "pitching")
# 手臂垂下时手能伸到旁边的：立柱
MID = ("lz_",)
SKIP = ("dex1", "hand_", "d435", "mid360", "camera", "imu")


def _geom_points(link_el, kind, mesh_dir: Path, rng, n_samples: int):
    """连杆的某类几何（collision / visual）→ 连杆系下的 (表面点, 法向)；没有可用几何返回 None。"""
    P_all, N_all = [], []
    for g in link_el.findall(kind):
        geo = g.find("geometry")
        if geo is None or len(geo) == 0:
            continue
        o = g.find("origin")
        xyz = [float(v) for v in (o.get("xyz") if o is not None and o.get("xyz") else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy") if o is not None and o.get("rpy") else "0 0 0").split()]
        T = make_transform(xyz, rpy)
        shape = geo[0]
        if shape.tag == "mesh":
            f = mesh_dir.parent / shape.get("filename")
            if not f.is_file():
                f = mesh_dir / Path(shape.get("filename")).name
            if not f.is_file():
                continue
            tris, areas = read_stl(f)
            sc = [float(v) for v in (shape.get("scale") or "1 1 1").split()]
            pts, nrm = sample_surface_normals(tris * np.asarray(sc), areas, n_samples, rng)
        elif shape.tag == "cylinder":
            r, L = float(shape.get("radius")), float(shape.get("length"))
            th = rng.uniform(0, 2 * np.pi, n_samples)
            z = rng.uniform(-L / 2, L / 2, n_samples)
            side = np.c_[np.cos(th), np.sin(th), np.zeros(n_samples)]
            rr = r * np.sqrt(rng.random(400))
            tc = rng.uniform(0, 2 * np.pi, 400)
            caps = np.c_[rr * np.cos(tc), rr * np.sin(tc), np.where(np.arange(400) < 200, L / 2, -L / 2)]
            pts = np.r_[r * side + np.c_[np.zeros(n_samples), np.zeros(n_samples), z], caps]
            nrm = np.r_[side, np.c_[np.zeros(400), np.zeros(400), np.sign(caps[:, 2])]]
        elif shape.tag == "box":
            sz = np.asarray([float(v) for v in shape.get("size").split()])
            pts = (rng.random((n_samples, 3)) - 0.5) * sz
            ax = rng.integers(0, 3, n_samples)
            sg = np.sign(pts[np.arange(n_samples), ax])
            pts[np.arange(n_samples), ax] = sg * sz[ax] / 2
            nrm = np.zeros_like(pts)
            nrm[np.arange(n_samples), ax] = sg
        elif shape.tag == "sphere":
            r = float(shape.get("radius"))
            v = rng.normal(size=(n_samples, 3))
            nrm = v / np.linalg.norm(v, axis=1, keepdims=True)
            pts = r * nrm
        else:
            continue
        P_all.append(pts @ T[:3, :3].T + T[:3, 3])
        N_all.append(nrm @ T[:3, :3].T)
    if not P_all:
        return None
    return np.concatenate(P_all), np.concatenate(N_all)


def _has_mesh(link_el, kind, mesh_dir: Path) -> bool:
    for g in link_el.findall(kind):
        m = g.find("geometry/mesh")
        if m is not None:
            f = mesh_dir.parent / m.get("filename")
            if f.is_file() or (mesh_dir / Path(m.get("filename")).name).is_file():
                return True
    return False


def _jsonable(prims, src):
    out = []
    for p in prims:
        out.append({"a": [round(float(x), 4) for x in p["a"]], "b": [round(float(x), 4) for x in p["b"]],
                    "r": round(float(p["r"]), 4), "src": src})
    return out


def dump_spec(spec: dict) -> str:
    """每个图元 / 连杆对占一行，文件小、diff 好看。"""
    lines = ["{"]
    keys = list(spec)
    for ki, k in enumerate(keys):
        v = spec[k]
        tail = "," if ki < len(keys) - 1 else ""
        if k in ("links", "manual") and isinstance(v, dict):
            lines.append(f' "{k}": {{')
            names = list(v)
            for ni, name in enumerate(names):
                lines.append(f'  "{name}": [')
                prims = v[name]
                for pi, p in enumerate(prims):
                    lines.append("   " + json.dumps(p, ensure_ascii=False) + ("," if pi < len(prims) - 1 else ""))
                lines.append("  ]" + ("," if ni < len(names) - 1 else ""))
            lines.append(" }" + tail)
        elif k in ("pairs", "ignored_pairs") and isinstance(v, list):
            lines.append(f' "{k}": [')
            for pi, p in enumerate(v):
                lines.append("  " + json.dumps(p, ensure_ascii=False) + ("," if pi < len(v) - 1 else ""))
            lines.append(" ]" + tail)
        else:
            lines.append(f" {json.dumps(k)}: {json.dumps(v, ensure_ascii=False)}{tail}")
    lines.append("}")
    return "\n".join(lines) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("robot", nargs="?", default="g1d")
    ap.add_argument("--cell-near", type=float, default=0.015, help="躯干 / 头这类靠近手臂的连杆，分块格子边长（m）")
    ap.add_argument("--cell-arm", type=float, default=0.016, help="手臂连杆用球时的分块格子边长（m）")
    ap.add_argument("--prim-cost", type=float, default=0.00001, help="手臂选拟合方式时每个图元扣的分（m）")
    ap.add_argument("--arm-parts", type=int, default=3, help="每节手臂沿长度最多分几段（每段再横截面拆成几根平行细胶囊体）")
    ap.add_argument("--arm-gain", type=float, default=0.93, help="多一根胶囊体，总体积要小于这个倍数才加")
    ap.add_argument("--cell-mid", type=float, default=0.025, help="立柱，分块格子边长（m）")
    ap.add_argument("--cell-far", type=float, default=0.06, help="底盘 / 轮子，分块格子边长（m）")
    ap.add_argument("--samples", type=int, default=3000, help="每个网格的表面采样点数")
    ap.add_argument("--acm-samples", type=int, default=3000, help="随机关节角采样数（找结构性贴合的连杆对）")
    ap.add_argument("--always", type=float, default=0.95, help="重叠比例 ≥ 它的连杆对视为结构性贴合、不查")
    args = ap.parse_args()

    prof = load_profile(args.robot)
    urdf = prof.urdf
    out_path = urdf.parent / "collision.json"
    old = json.loads(out_path.read_text(encoding="utf-8")) if out_path.is_file() else {}
    rng = np.random.default_rng(0)
    root = ET.parse(str(urdf)).getroot()

    arm_links = {}
    child_of = {j.find("child").get("link"): j.get("name") for j in root.findall("joint")}
    joint_child = {j.get("name"): j.find("child").get("link") for j in root.findall("joint")}
    for name, lay in prof.arms.items():
        arm_links[name] = [joint_child[jn] for jn in lay.joint_names]
    arm_set = {l for v in arm_links.values() for l in v}

    links = {}
    print(f"{'连杆':28s} {'形状':6s} {'个数':>4s} {'最大半径':>8s} 来源")
    for link_el in root.findall("link"):
        name = link_el.get("name")
        if any(s in name.lower() for s in SKIP):
            continue
        # 优先用网格：碰撞网格 → 外观网格 → 碰撞里的简单形状（有的 URDF 碰撞体只是个小圆柱，比实物小）
        got, src = None, ""
        for kind in ("collision", "visual"):
            if _has_mesh(link_el, kind, prof.meshes_dir):
                got, src = _geom_points(link_el, kind, prof.meshes_dir, rng, args.samples), kind
                break
        if got is None:
            got, src = _geom_points(link_el, "collision", prof.meshes_dir, rng, args.samples), "collision"
        if got is None or len(got[0]) < 4:
            continue
        pts, nrm = got
        if name in arm_set:
            # 胶囊体拆分和一组球都拟一遍，取外鼓（bulge 95 分位）小的；每个图元扣 args.prim_cost 分（检查要算）
            caps = fit_capsules(pts, args.arm_parts, gain=args.arm_gain, rng=rng)
            sph = fit_spheres_inset(pts, nrm, args.cell_arm)
            bc, bs = bulge(caps, pts), bulge(sph, pts)
            if bs + args.prim_cost * len(sph) < bc + args.prim_cost * len(caps):
                prims, shape = sph, f"球（外鼓 {bs * 1000:.0f} mm，胶囊 {bc * 1000:.0f}）"
            else:
                prims, shape = caps, f"胶囊（外鼓 {bc * 1000:.0f} mm，球 {bs * 1000:.0f}）"
        else:
            low = name.lower()
            cell = (args.cell_near if any(k in low for k in NEAR)
                    else args.cell_mid if any(k in low for k in MID) else args.cell_far)
            prims, shape = fit_spheres_inset(pts, nrm, cell), "球"
        links[name] = _jsonable(prims, src)
        print(f"{name:28s} {shape:6s} {len(prims):4d} {max(p['r'] for p in prims) * 1000:7.1f}mm {src}")

    manual = old.get("manual") or MANUAL.get(args.robot, {})
    spec = {
        "robot": args.robot,
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "generator": "tools/fit_collision.py",
        "note": "连杆系下的胶囊体（a-b 线段 + 半径 r，a==b 即球）。manual 段是手写的末端，重跑工具会保留。",
        "links": links,
        "manual": manual,
        "arms": arm_links,
        "pairs": [],
        "ignored_pairs": [],
    }
    model = CollisionModel(urdf, spec)
    cand = model.candidate_pairs()
    model.set_pairs(cand)

    # 随机两臂关节角（立柱中位、腰 0）统计每对的重叠比例
    base_q = {}
    if prof.lift is not None:
        for jn in prof.lift.urdf_joints:
            base_q[jn] = 0.5 * prof.lift.stage_limit_m
    from robot_platform.control.urdf_fk import URDFRobotModel
    arm_limits = {name: URDFRobotModel(str(urdf), "torso_link", lay.tip_link).joint_limits()
                  for name, lay in prof.arms.items()}
    hits = np.zeros(len(cand))
    for _ in range(args.acm_samples):
        q = dict(base_q)
        for name, lay in prof.arms.items():
            lo, hi = arm_limits[name][:, 0], arm_limits[name][:, 1]
            q.update(zip(lay.joint_names, rng.uniform(lo, hi)))
        hits += model.evaluate(q) < 0.0
    rate = hits / args.acm_samples
    home = {**base_q}
    for name, lay in prof.arms.items():
        if lay.home_q:
            home.update(zip(lay.joint_names, lay.home_q))
    d_home = model.evaluate(home)

    keep, ignored = [], []
    for (a, b), r, dh in zip(cand, rate, d_home):
        if r >= args.always:
            ignored.append([a, b, f"随机姿态 {r * 100:.0f}% 重叠，视为结构性贴合"])
        else:
            keep.append([a, b])
        if dh < 0 and r < args.always:
            print(f"  注意：home 姿态下 {a} ↔ {b} 已重叠 {dh * 1000:.1f} mm（重叠比例 {r * 100:.0f}%）")
    spec["pairs"] = keep
    spec["ignored_pairs"] = ignored
    print(f"\n候选连杆对 {len(cand)}，查 {len(keep)}，忽略 {len(ignored)}")
    for a, b, why in ignored:
        print(f"  忽略 {a} ↔ {b}：{why}")
    busy = sorted(zip(rate, cand), reverse=True)[:12]
    print("重叠比例最高的连杆对：")
    for r, (a, b) in busy:
        print(f"  {r * 100:5.1f}%  {a} ↔ {b}")
    out_path.write_text(dump_spec(spec), encoding="utf-8")
    print(f"\n已写 {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
