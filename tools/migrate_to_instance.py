#!/usr/bin/env python3
"""把旧布局（数据散在 arm_control/config、hand_eye/config、*/data 等处）搬进 instances/<名字>/。

    python tools/migrate_to_instance.py bwt-g1d1 --dry-run
    python tools/migrate_to_instance.py bwt-g1d1 --from /path/to/旧仓库或备份 --move-data --set-default

- 标定与状态文件（calib/、state/）按文件复制；实例里已有的同名文件会被旧位置的覆盖。
- 采集与测量（data/）默认复制；加 --move-data 则直接改名搬过去（同一磁盘上不占额外空间）。
- 数据文件里存的旧绝对路径改写成相对实例的路径。
- 不删除旧位置的任何文件；确认无误后再自己清理。
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path
from typing import Any, Dict, List

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "arm_control"))

import instance as instances  # noqa: E402
from instance import INSTANCE_FILE, LAYOUT, LEGACY_LAYOUT, LEGACY_SETTINGS  # noqa: E402

DATA_KEYS = ("clouds", "board_touch", "handeye_sessions", "measure", "validate")
SETTINGS_KEYS = ("network_interface", "unitree_sdk_path", "dds_domain", "lowstate_timeout_s", "defaults")


def _copy_tree(src: Path, dst: Path, dry: bool, skip=()) -> int:
    n = 0
    for f in sorted(src.rglob("*")):
        if not f.is_file() or any(part in skip for part in f.relative_to(src).parts[:1]):
            continue
        t = dst / f.relative_to(src)
        n += 1
        if not dry:
            t.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(f, t)
    return n


def _move_children(src: Path, dst: Path, dry: bool, skip=()) -> int:
    n = 0
    for child in sorted(src.iterdir()):
        if child.name in skip:
            continue
        t = dst / child.name
        if t.exists():
            raise SystemExit(f"目标已存在，不覆盖：{t}")
        n += 1
        if not dry:
            dst.mkdir(parents=True, exist_ok=True)
            shutil.move(str(child), str(t))
    return n


def _rewrite_paths(inst: "instances.Instance", dry: bool) -> List[str]:
    """把 JSON 里指向旧布局的绝对路径改成相对实例的路径。"""
    changed = []

    def fix(v: Any) -> Any:
        if isinstance(v, dict):
            return {k: fix(x) for k, x in v.items()}
        if isinstance(v, list):
            return [fix(x) for x in v]
        if isinstance(v, str) and (v.startswith("/") or (len(v) > 2 and v[1] == ":")):
            p = inst.from_stored(v)
            stored = inst.to_stored(p) if p is not None else v
            if not Path(stored).is_absolute():
                return stored
        return v

    for f in sorted(inst.root.rglob("*.json")):
        text = f.read_text(encoding="utf-8")
        if not any("/" + legacy + "/" in text for legacy in LEGACY_LAYOUT.values()):
            continue
        doc = json.loads(text)
        new = fix(doc)
        if new != doc:
            changed.append(str(f.relative_to(inst.root)))
            if not dry:
                f.write_text(json.dumps(new, indent=2, ensure_ascii=False), encoding="utf-8")
    return changed


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("name", help="实例名，例如 bwt-g1d1")
    p.add_argument("--from", dest="src", type=Path, default=REPO, help="旧布局所在目录（默认本仓库）")
    p.add_argument("--robot", default=None, help="机型名，默认取旧 config/g1d.json 的 robot，否则 g1d")
    p.add_argument("--move-data", action="store_true", help="data/ 直接搬走而不是复制")
    p.add_argument("--set-default", action="store_true", help="写 instances/default，之后不加 --instance 就用它")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()

    src: Path = args.src.resolve()
    root = instances.INSTANCES_DIR / args.name
    dry = args.dry_run
    print(f"从 {src}\n到 {root}{'（试运行，不写任何文件）' if dry else ''}")

    settings: Dict[str, Any] = {}
    if (root / INSTANCE_FILE).is_file():
        settings = json.loads((root / INSTANCE_FILE).read_text(encoding="utf-8"))
    legacy_settings = src / LEGACY_SETTINGS
    if legacy_settings.is_file():
        raw = json.loads(legacy_settings.read_text(encoding="utf-8"))
        settings.update({k: raw[k] for k in SETTINGS_KEYS if k in raw})
        settings.setdefault("robot", raw.get("robot"))
    robot = args.robot or settings.pop("robot", None) or "g1d"
    settings = {"robot": robot, **{k: v for k, v in settings.items() if k != "robot"}}
    if not dry:
        root.mkdir(parents=True, exist_ok=True)
        (root / INSTANCE_FILE).write_text(json.dumps(settings, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"  {INSTANCE_FILE}: {json.dumps(settings, ensure_ascii=False)}")

    inst = instances.Instance(name=args.name, root=root.resolve(), settings=settings)
    for key, legacy in LEGACY_LAYOUT.items():
        s = src / legacy
        d = inst.path(key)
        if not s.exists():
            continue
        # hand_eye/data 下还套着 clouds/、board_touch/，它们各有去处
        skip = ("clouds", "board_touch") if key == "handeye_sessions" else ()
        if s.is_file():
            n = 1
            if not dry:
                d.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(s, d)
        elif key in DATA_KEYS and args.move_data:
            n = _move_children(s, d, dry, skip)
        else:
            n = _copy_tree(s, d, dry, skip)
        verb = "搬" if key in DATA_KEYS and args.move_data else "复制"
        print(f"  {legacy} → {LAYOUT[key]}：{verb} {n} 项")

    if not dry:
        changed = _rewrite_paths(inst, dry)
        print(f"  改写绝对路径的文件：{len(changed)} 个")
        for c in changed:
            print(f"    {c}")
    if args.set_default and not dry:
        instances.DEFAULT_FILE.write_text(args.name + "\n", encoding="utf-8")
        print(f"  默认实例 → {args.name}（{instances.DEFAULT_FILE}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
