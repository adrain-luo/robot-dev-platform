# arm_control

G1D 独立臂控底座。供 `arm_payload_gravity` / `tcp_calib` / `hand_eye` 复用。

## 目录

- `dds/` — lowstate 订阅；可运动时 `ArmLoop` 发 `rt/lowcmd`
- `controller/` — hold、矢量同步限速 `set_target`
- `model/` — 关节布局 + URDF FK
- `frontend/` — 3D 骨架 + 接管/走点
- `config/` — `g1d.json`

需 `/usr/bin/python3`（conda `base` 无 cyclonedds）。

## 只读

```bash
cd ~/abyss/project/g1d/arm_control
/usr/bin/python3 server.py
# http://<本机IP>:10184/
```

不创建发令循环。网页按钮为灰。

## 可运动（⚠ 真机）

先停掉占用 `rt/lowcmd` 的任务（如 vision-arm-control service）。有人扶臂、守急停。

```bash
cd ~/abyss/project/g1d/arm_control
/usr/bin/python3 server.py --allow-real-motion --max-speed 0.15
```

1. 徽标「可运动」→ 点「接管」：在当前角保持（会 `ReleaseMode` 高层模式）。
2. 「填入当前角」改几个数 →「出发」：限速逼近。
3. 「急停(保持)」冻结目标；「释放」停止发令（请扶住）。

灰虚线=指令，蓝虚线=目标，实线=实测。网页限速可在 0.02–1.0 rad/s 内改（点「设」）。尚无重力前馈 / 示教卸力。
