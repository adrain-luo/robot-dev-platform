import numpy as np

from robot_platform.perception.stereo_match import write_ply


def _read_binary_ply(path):
    raw = path.read_bytes()
    end = raw.index(b"end_header\n") + len(b"end_header\n")
    header = raw[:end].decode("ascii")
    n = int(header.split("element vertex ")[1].split("\n")[0])
    dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    rec = np.frombuffer(raw[end:], dtype=dt)
    return header, n, rec


def test_write_ply_binary_roundtrip(tmp_path):
    rng = np.random.default_rng(0)
    pts = rng.normal(size=(1000, 3))
    bgr = rng.integers(0, 256, size=(1000, 3), dtype=np.uint8)
    path = tmp_path / "sub" / "c.ply"
    assert write_ply(path, pts, bgr) == 1000
    header, n, rec = _read_binary_ply(path)
    assert "format binary_little_endian 1.0" in header and n == 1000 and len(rec) == 1000
    np.testing.assert_allclose(np.stack([rec["x"], rec["y"], rec["z"]], 1), pts.astype(np.float32))
    # 入参是 BGR，文件里是 RGB
    assert (rec["red"] == bgr[:, 2]).all() and (rec["green"] == bgr[:, 1]).all() and (rec["blue"] == bgr[:, 0]).all()


def test_write_ply_empty(tmp_path):
    path = tmp_path / "e.ply"
    assert write_ply(path, np.zeros((0, 3)), np.zeros((0, 3), np.uint8)) == 0
    assert _read_binary_ply(path)[1] == 0
