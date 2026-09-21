# g1d

G1D 侧功能区。控制底座与各标定/辨识项目分目录存放；H2 参考项目在 `../arm_payload_gravity`，本树不改它。

| 目录 | 用途 |
|------|------|
| `arm_control/` | 共用臂控（只读 3D；可选 `--allow-real-motion` 限速走点） |
| `arm_payload_gravity/` | 末端负载辨识与重力前馈 |
| `tcp_calib/` | TCP 标定 |
| `hand_eye/` | 手眼标定 |

约定：只有 `arm_control` 发 DDS 指令；功能项目只调用其接口。真机发令需显式允许（如 `--allow-real-motion`）并 engage。
