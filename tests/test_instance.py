"""实例路径：新旧路径的读写换算，以及迁移脚本在一棵假旧布局上的搬家结果。"""

import json
import subprocess
import sys
from pathlib import Path

from robot_platform.core import instance as instances

REPO = Path(__file__).resolve().parents[1]


def _inst(tmp_path):
    root = tmp_path / "inst"
    root.mkdir()
    return instances.Instance(name="t", root=root.resolve(), settings={"robot": "g1d"})


def test_stored_paths_roundtrip(tmp_path):
    inst = _inst(tmp_path)
    f = inst.path("handeye_results") / "a.yaml"
    assert inst.to_stored(f) == "calib/handeye/a.yaml"
    assert inst.from_stored("calib/handeye/a.yaml") == inst.root / "calib/handeye/a.yaml"
    assert inst.from_stored("") is None
    outside = tmp_path / "elsewhere.yaml"
    outside.write_text("x")
    assert inst.from_stored(str(outside)) == outside
    assert inst.to_stored(outside) == str(outside.resolve())


def test_legacy_absolute_paths_map_into_instance(tmp_path):
    inst = _inst(tmp_path)
    old = "/home/robot/abyss/project/g1d"
    assert inst.from_stored(f"{old}/hand_eye/config/results/1029手眼.yaml") == inst.path("handeye_results") / "1029手眼.yaml"
    assert inst.from_stored(f"{old}/hand_eye/data/20261029_100052_left/stereo_calibration.yaml") == (
        inst.path("handeye_sessions") / "20261029_100052_left" / "stereo_calibration.yaml")
    assert inst.from_stored(f"{old}/hand_eye/data/clouds/x.ply") == inst.path("clouds") / "x.ply"
    # 与实例无关的绝对路径原样保留
    assert inst.from_stored("/home/robot/yx/teleimager_src_copy").as_posix() == "/home/robot/yx/teleimager_src_copy"


def test_other_checkout_instance_paths_prefer_own_copy(tmp_path):
    """搬仓库后，数据里写着旧目录下同名实例的绝对路径：本实例有这个文件就用本实例的。"""
    inst = _inst(tmp_path)
    old_root = tmp_path / "old_repo" / "instances" / "t"
    old_file = old_root / "data/handeye/s1/stereo_calibration.yaml"
    old_file.parent.mkdir(parents=True)
    old_file.write_text("old")
    stored = old_file.as_posix()
    assert inst.from_stored(stored) == old_file, "本实例没有这份时仍用原路径"
    mine = inst.root / "data/handeye/s1/stereo_calibration.yaml"
    mine.parent.mkdir(parents=True)
    mine.write_text("new")
    assert inst.from_stored(stored) == mine


def test_migration_script(tmp_path):
    legacy = tmp_path / "old"
    old_abs = "/home/robot/abyss/project/g1d"
    files = {
        "arm_control/config/g1d.json": {"robot": "g1d", "network_interface": "eth9", "dds_domain": 0},
        "arm_control/config/tcp_results/t.json": {"name": "t"},
        "arm_control/config/runtime_lock.json": {"locked": True},
        "hand_eye/config.json": {"intrinsics": f"{old_abs}/hand_eye/config/results/r.yaml", "camera_host": "1.2.3.4"},
        "hand_eye/config/results/r.json": {"intrinsics_yaml": f"{old_abs}/hand_eye/config/results/r.yaml"},
        "hand_eye/data/s1/session_meta.json": {"arm": "left"},
        "hand_eye/data/s1/handeye_result.json": {"intrinsics_yaml": f"{old_abs}/hand_eye/data/s1/stereo_calibration.yaml"},
        "hand_eye/data/board_touch/log.jsonl": None,
        "arm_payload_gravity/data/measure/m1/meta.json": {"arm": "right"},
    }
    for rel, doc in files.items():
        p = legacy / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("{}\n" if doc is None else json.dumps(doc, ensure_ascii=False), encoding="utf-8")
    (legacy / "hand_eye/config/results/r.yaml").write_text("yaml", encoding="utf-8")
    import os
    old_mtime = 1_700_000_000
    os.utime(legacy / "hand_eye/config/results/r.json", (old_mtime, old_mtime))

    name = "pytest_migrate_tmp"
    dest = instances.INSTANCES_DIR / name
    try:
        subprocess.run([sys.executable, str(REPO / "tools" / "migrate_to_instance.py"), name,
                        "--from", str(legacy), "--move-data"], check=True, capture_output=True)
        inst = instances.load_instance(name)
        assert inst.settings["robot"] == "g1d" and inst.settings["network_interface"] == "eth9"
        assert json.loads(inst.path("runtime_lock").read_text(encoding="utf-8")) == {"locked": True}
        assert (inst.path("tcp_results") / "t.json").is_file()
        assert (inst.path("handeye_results") / "r.yaml").read_text(encoding="utf-8") == "yaml"
        cfg = json.loads(inst.path("handeye_config").read_text(encoding="utf-8"))
        assert cfg == {"intrinsics": "calib/handeye/r.yaml", "camera_host": "1.2.3.4"}
        # 改写路径不改修改时间（手眼结果列表按它排序）
        r = inst.path("handeye_results") / "r.json"
        assert json.loads(r.read_text(encoding="utf-8"))["intrinsics_yaml"] == "calib/handeye/r.yaml"
        assert int(r.stat().st_mtime) == old_mtime
        doc = json.loads((inst.path("handeye_sessions") / "s1" / "handeye_result.json").read_text(encoding="utf-8"))
        assert doc["intrinsics_yaml"] == "data/handeye/s1/stereo_calibration.yaml"
        # data 被搬走；board_touch 单独归位，不混进手眼会话
        assert not (legacy / "hand_eye/data/s1").exists()
        assert (inst.path("board_touch") / "log.jsonl").is_file()
        assert not (inst.path("handeye_sessions") / "board_touch").exists()
        assert (inst.path("measure") / "m1" / "meta.json").is_file()
        # 标定文件是复制，旧位置保留
        assert (legacy / "arm_control/config/tcp_results/t.json").is_file()
    finally:
        import shutil
        shutil.rmtree(dest, ignore_errors=True)
