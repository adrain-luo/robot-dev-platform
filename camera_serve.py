#!/usr/bin/env python3
"""独立相机查看服务（与主站点 serve.py 无关，可单独启动），只读：

    python camera_serve.py                 # 按 g1d 声明的双目相机取图，http://<本机>:10186/
    python camera_serve.py --mock          # 仿真画面
    python camera_serve.py --port 10190 --camera-host 192.168.123.164

功能：实时画面、切换左 / 右眼、清晰度掩膜（分档伪彩 + 等高线，可开关、可调透明度、可锁定刻度）。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))

from robot_platform.tools.camera_view.server import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
