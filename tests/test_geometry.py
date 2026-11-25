"""公共点云几何（robot_platform/geometry.py）：从棋盘触摸搬出来后行为不变。

感知不导入任务的规则在 test_import_rules.py。
"""

import numpy as np

from robot_platform.geometry import fit_plane, sample_point


def test_fit_plane_recovers_tilted_plane():
    rng = np.random.default_rng(0)
    xy = rng.uniform(-0.1, 0.1, size=(50, 2))
    n = np.array([0.0, 0.6, 0.8])  # 单位法向
    z = (0.5 - n[0] * xy[:, 0] - n[1] * xy[:, 1]) / n[2]
    pts = np.column_stack([xy, z])
    bump = np.zeros(50)
    bump[7] = 0.004  # 一个点离开平面 4 mm
    pts = pts + bump[:, None] * n
    plane = fit_plane(pts)
    assert abs(abs(float(plane["normal"] @ n)) - 1.0) < 1e-3
    assert 3.5 < plane["max_mm"] < 4.0
    assert plane["rms_mm"] < plane["max_mm"]
    # 投影后的点都在拟合平面上
    d = (plane["flat"] - plane["center"]) @ plane["normal"]
    assert float(np.max(np.abs(d))) < 1e-9


def test_sample_point_direct_hole_and_rejects():
    cloud = np.full((20, 30, 3), np.nan)
    cloud[..., 0] = 0.1
    cloud[..., 1] = 0.2
    cloud[..., 2] = 1.0
    # 像素本身有效：原样返回（含四舍五入、越界裁到边上）
    assert np.allclose(sample_point(cloud, 5.4, 6.6), [0.1, 0.2, 1.0])
    assert np.allclose(sample_point(cloud, -3, 99), [0.1, 0.2, 1.0])
    # 像素是洞：取 9x9 邻域有效点的中位数
    cloud[10, 10] = np.nan
    cloud[9:12, 9:12, 2] = 1.2
    cloud[10, 10] = np.nan
    assert np.allclose(sample_point(cloud, 10, 10), [0.1, 0.2, 1.0])
    # 深度超出 0.2–2 m 不算有效
    far = cloud.copy()
    far[..., 2] = 2.5
    assert sample_point(far, 10, 10) is None
    # 邻域有效点少于 5 个
    sparse = np.full((20, 30, 3), np.nan)
    sparse[0, 0] = [0.0, 0.0, 1.0]
    assert sample_point(sparse, 10, 10) is None

