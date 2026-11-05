"""包与仓库的位置。其它模块要找仓库里的文件（实例、第三方库、前端）都从这里取。"""

from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent          # src/robot_platform
REPO_ROOT = PACKAGE_DIR.parents[1]                      # 仓库根目录
ROBOTS_DIR = PACKAGE_DIR / "robots"                     # 机型档案（robot.json + URDF + 网格）
STATIC_DIR = PACKAGE_DIR / "web" / "static"             # 网页前端
INSTANCES_DIR = REPO_ROOT / "instances"                 # 各台机器的标定 / 运行数据
THIRD_PARTY_DIR = REPO_ROOT / "third_party"
