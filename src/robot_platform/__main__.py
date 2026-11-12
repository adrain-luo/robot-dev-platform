"""python -m robot_platform —— 启动网页服务（参数同 robot_platform.web.server）。"""

from robot_platform.web.server import main

# IK 子进程用 spawn 起，会以 __mp_main__ 重新导入这个文件：不能在那里再起一个服务
if __name__ == "__main__":
    raise SystemExit(main())
