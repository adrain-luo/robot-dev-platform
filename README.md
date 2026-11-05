# Robot Development Platform（机器人开发平台）

多机器人（G1D 轮式 / G1 双足）的臂控、标定、感知与工作流，都挂在**同一个网页进程**里。

仓库：<https://github.com/Abyss-hhy/robot-dev-platform>（源自 `g1d_eye_to_hand` v1.0，历史一并保留）

## 怎么跑

```bash
conda activate abyss          # 不要用 base / 系统 python3
cd ~/abyss/project/robot-dev-platform
python serve.py               # 只读：http://<本机>:10184/
# 真机发令（先停掉占用发令话题的任务，扶臂、守急停）：
python serve.py --allow-real-motion --max-speed 0.2
python serve.py --backend mock   # 不接机器人，仿真
```

`serve.py` 不用安装即可运行。也可以 `pip install -e . --no-deps` 装进当前环境，之后在任何目录用 `python -m robot_platform` 或 `robot-platform` 启动，参数相同。

首次若没有 `abyss` 环境：`conda env create -f environment.yml`，或跑 `./setup_env.sh`。

启动时用 `instances/default` 里写的机器实例，也可 `--instance <名字>` 指定；新机器先建实例，见 [`instances/README.md`](instances/README.md)。

## 目录

代码都在 `src/robot_platform/` 这一个包里，按功能分区，和网页左右两侧的分区对应：

| 目录 | 职责 |
|------|------|
| `core/` | 机器实例（数据放哪）、运行设置、机型档案加载、关节布局、可锁定的成套配置 |
| `robots/<机型>/` | 机型描述：`robot.json` + URDF + 网格，见 [`robots/README.md`](src/robot_platform/robots/README.md) |
| `hw/` | 硬件：`backends/`（G1D `unitree_lowcmd`、G1 `unitree_arm_sdk`、`mock`）、`dds/`、立柱闭环 |
| `control/` | 臂控循环 `ArmLoop`（限速、接管、拖拽、腰保持）、URDF 正运动学、重力前馈 |
| `calib/` | 标定：`tcp.py`、`lift.py`、`handeye/`（采集、求解、结果、内参）、`payload/`（负载辨识、姿态集、摩擦、到位对比） |
| `perception/` | 相机（双目 / RGB-D / mock）、立体匹配、深度点云、相机与点云接口 `cloud_api.py`、G1D 调试口 `debug_port.py`、识别定位 |
| `tasks/` | 面向操作的功能：工作姿态、空间点、棋盘触摸 |
| `web/` | HTTP 服务 `server.py` 与前端 `static/` |
| `paths.py` | 仓库内各目录的位置 |

仓库根目录的其它内容：

| 目录 | 职责 |
|------|------|
| `instances/` | 机器实例：每台机器的设置与标定数据，见 [`instances/README.md`](instances/README.md) |
| `tools/` | 零散脚本：只读看 lowstate、旧数据迁移 |
| `tests/` | `python -m pytest`（`pyproject.toml` 已配好搜索路径） |
| `third_party/unitree_sdk2_python/` | Unitree SDK |
| `功能说明.md` | 按模块写的详细说明（算法、坐标、文件路径） |

**边界：** 只有 `hw/` 的后端订阅 / 发布机器人话题（G1D 发 `rt/lowcmd`，G1 发 `rt/arm_sdk`；立柱另用 Agv + `rt/hispeed_state`）。其它模块经 `control/arm_loop.py` 发令，不另起服务、不另发 DDS。页面只有 `web/static/` 一份。

导入一律写完整包名，例如 `from robot_platform.calib import tcp`，不再改 `sys.path`（只有 `serve.py`、`tools/` 脚本和测试夹具为了免安装，把 `src` 加进搜索路径）。

## 文档去哪看

- 启动、网页结构与安全操作 → [`web/README.md`](src/robot_platform/web/README.md)
- 功能原理与数据路径 → [`功能说明.md`](功能说明.md)
- 硬件后端 → [`hw/backends/README.md`](src/robot_platform/hw/backends/README.md)
- 手眼 → [`calib/handeye/README.md`](src/robot_platform/calib/handeye/README.md)
