#!/usr/bin/env python3
"""启动网页服务，不需要先 pip install。参数与 `python -m robot_platform` 相同，例如：

    python serve.py                                     # 只读
    python serve.py --allow-real-motion --max-speed 0.2  # 真机
    python serve.py --backend mock                      # 仿真
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))

from robot_platform.web.server import main  # noqa: E402

raise SystemExit(main())
