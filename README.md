# g1d（眼在手外 · 臂控与标定）

G1D 侧的臂控底座、负载辨识、手眼、TCP、空间点与识别定位，都挂在**同一个网页进程**里。

仓库：<https://github.com/Abyss-hhy/g1d_eye_to_hand>

## 怎么跑

```bash
conda activate abyss          # 不要用 base / 系统 python3
cd ~/abyss/project/g1d/arm_control
python server.py              # 只读：http://<本机>:10184/
# 真机发令（先停掉占用 rt/lowcmd 的任务，扶臂、守急停）：
python server.py --allow-real-motion --max-speed 0.2
```

首次若没有 `abyss` 环境：`conda env create -f environment.yml`，或跑 `./setup_env.sh`。

## 目录分工

| 目录 | 职责 |
|------|------|
| `arm_control/` | **唯一发令处**：DDS、限速循环、URDF、前端、腰/立柱、TCP、空间点、棋盘触摸、识别定位 |
| `arm_payload_gravity/` | 姿态集、末端负载辨识、摩擦带、到位对比（接口挂在臂控网页上） |
| `hand_eye/` | 头相机取流、棋盘检测、内参、手眼外参、SGBM 点云 |
| `tcp_calib/` | 早期占位；现行 TCP 在 `arm_control/tcp_calib.py` |
| `third_party/unitree_sdk2_python/` | Unitree SDK |
| `功能说明.md` | 按模块写的详细说明（算法、坐标、文件路径） |

**边界：** 只有 `arm_control` 订阅/发布 `rt/lowstate`、`rt/lowcmd`（立柱另用 Agv + `rt/hispeed_state`）。其它目录不另起服务、不另发 DDS。页面只有 `arm_control/frontend/` 一份。

## 文档去哪看

- 启动与安全操作 → [`arm_control/README.md`](arm_control/README.md)
- 功能原理与数据路径 → [`功能说明.md`](功能说明.md)
- 手眼目录说明 → [`hand_eye/README.md`](hand_eye/README.md)
