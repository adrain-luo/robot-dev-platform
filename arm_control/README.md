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
/usr/bin/python3 server.py --allow-real-motion --max-speed 0.2
```

1. 徽标「可运动」→ 点「接管」：在当前角保持（会 `ReleaseMode` 高层模式）。
2. 「填入当前角」改几个数 →「出发」：限速逼近。
3. 「急停(保持)」冻结目标；「释放」停止发令（请扶住）。

灰虚线=指令，蓝虚线=目标，实线=实测。网页限速（点「设」，0.02–1.0 rad/s）是循环里同一把尺子：单点「出发」、点姿态芯片走点、接管后腰回 0 都走它。改完必须点「设」。拖拽卸力不限速。

接管后：**拖拽** = 当前臂 kp=0 + URDF 重力前馈（α=1）；**急停(保持)** = 退出拖拽并锁在当前角。可勾选「重力前馈」。接管后腰电机 12/13/14 按限速回到 0，用锁腰增益 kp=200 / kd=4 钉住（比默认腰 kp=60 硬；仍是 PD，大力仍能掰，并可能微振）。释放后停发令即一起放开。

## 重力前馈示教姿态集

网页「姿态集」：选控制臂 → 新建名字 → 拖拽到位 → 空格/「记录」。JSON 在 `config/posesets/`。

「镜像到对侧」按躯干矢状面（y=0）映射，不是抄同一组关节角。G1D 左右轴定义相同，符号为 `[1,-1,-1, 1,-1, 1,-1]`（pitch 同号，roll/yaw 反号），末端 xyz 会变成 `(x,-y,z)`。点芯片可填入该角；已接管且非拖拽则按上面同一限速走过去。`regressor_cond` 是末端负载回归条件数，点太少或太挤时会很大。

## 末端负载辨识

网页「末端负载辨识」：选姿态集 → 接管 →「测量当前集」（双向逼近，走点也用同一限速）→「求解」（α=1）→「应用到前馈」。补偿文件 `config/payloads/<left|right>.json`。力矩默认用 PD 重构 `kp(q_cmd−q)−kd·dq+τ_ff`（与测量时已有的前馈无关）；也可用 `tau_est`。尚无到位对比/迭代修正。
