# g1d

G1D 侧功能区。控制底座与各标定/辨识项目分目录存放；H2 参考项目在 `../arm_payload_gravity`，本树不改它。

Python 用 conda 环境 `abyss`（`conda activate abyss`）。不要用 `base`、裸 `python3` 或 `/usr/bin/python3`。Unitree SDK 与 G1D URDF 在本树 `third_party/`、`arm_control/config/g1_d.urdf`。

| 目录 | 用途 |
|------|------|
| `arm_control/` | 共用臂控：DDS、限速、接管、3D。只有这里发令 |
| `arm_payload_gravity/` | 末端负载辨识、姿态集、到位对比。挂在臂控同一个网页上 |
| `tcp_calib/` | TCP 标定 |
| `hand_eye/` | 手眼标定 |

## 边界

只有 `arm_control` 发 DDS（`rt/lowstate` / `rt/lowcmd`）。功能目录不另起进程，由 `arm_control/server.py` 在启动时挂上它们的接口和页面。真机发令需 `--allow-real-motion` 并在网页上接管。

重力前馈的**施加**留在 `arm_control` 的控制循环里，因为那是发令的一部分。质量、质心怎么辨识出来，以及姿态集、补偿文件、摩擦带、到位对比，都在 `arm_payload_gravity`。它走点、改前馈时调用臂控的控制循环，不自己发令。

| 内容 | 位置 |
|------|------|
| 启动 | `cd arm_control && python server.py` |
| 姿态集、补偿文件 | `arm_payload_gravity/config/` |
| 测量会话、对比记录 | `arm_payload_gravity/data/` |
| 负载功能的页面 | `arm_payload_gravity/frontend/`，由臂控网页拼进去 |

`tcp_calib`、`hand_eye` 仍是占位，还没有实现。
