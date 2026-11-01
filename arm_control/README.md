# arm_control

G1D 控制与标定的**唯一网页入口**。其它模块（负载、手眼、TCP…）都挂在这里，不另开页面、不另发 DDS。

## 启动

```bash
conda activate abyss
cd ~/abyss/project/g1d/arm_control

python server.py                              # 只读，按钮灰色
python server.py --allow-real-motion --max-speed 0.2   # 真机
# 打开 http://<本机IP>:10184/
```

常用参数：`--host` / `--port`（默认 `0.0.0.0:10184`）、`--iface`、`--max-speed`（0.02–1.0 rad/s）。

真机前：停掉占用 `rt/lowcmd` 的其它任务；有人扶臂、守急停。

## 网页结构（书签）

左侧偏视觉，右侧偏标定/控制。卡片可多开，内容区可滚动。

| 区域 | 做什么 |
|------|--------|
| **渲染 / 双目** | 头相机或调试口匹配点云；固定场景累加 |
| **识别定位** | YOLO 取图 → 点位获取（网格投影）→ 校准（贴平面剔点）→ 中心点提取 |
| **运行配置** | 手眼结果、重力补偿、TCP 成套调用；可锁定防误改 |
| **姿态集 / 负载 / 到位对比** | 记点、辨识、摩擦带对比（数据在 `arm_payload_gravity/`） |
| **手眼标定** | 按姿态集采集 → 内参 → 外参 → 命名保存/调用 |
| **TCP / 工作姿态** | 爪尖偏置标定；与 TCP 绑定的腕部方向，可锁定 |
| **TCP 控制**（控制） | 躯干系平移（保姿态）/ 绕红点旋转；可设步长 |
| **空间点 / 棋盘触摸**（控制） | 躯干系定点碰 TCP；「看一眼」角点后逐点摸（验手眼+TCP） |
| **关节** | 当前臂 7 轴 + 腰 12/14 读数；立柱高度与「记为最低点」 |
| **躯体控制** | 腰（滑条/步进/到目标/停止）+ 立柱行程（Agv HeightAdjust） |
| **手臂控制 / 接管** | 滑条单关节限速；拖拽卸力、急停保持、释放 |

更细的算法与文件路径见仓库根目录 [`功能说明.md`](../功能说明.md)。

## 发令要点

1. 徽标「可运动」→ **接管**：手臂与腰停在当前角（退出高层运动模式）。
2. 网页限速（点「设」）是同一把尺子：单点出发、姿态芯片走点、腰目标逼近都用它。拖拽卸力不限速。
3. **拖拽** = 当前臂 kp=0 + URDF 重力前馈；**急停(保持)** = 退出拖拽并锁当前角（棋盘触摸若在跑也会停）。
4. **释放** = 停发 `rt/lowcmd`（请扶住）。

腰电机按官方表：**12 = 旋转（yaw）**，**14 = 俯仰（pitch）**；13 在 G1D 无效，跟写即可。立柱高度在 `rt/hispeed_state`，不在 lowstate。

画面：灰虚线=指令，蓝虚线=目标，实线=实测。红点=当前 TCP 爪尖，黄点=选中空间点。

## 本目录代码

| 路径 | 作用 |
|------|------|
| `server.py` | HTTP + 汇总各模块 API |
| `controller/` | `ArmLoop` 限速、接管、腰保持 |
| `dds/` | `lowstate`、立柱 `hispeed`、Agv |
| `model/` | 关节布局、URDF FK、重力 |
| `frontend/` | 唯一前端 |
| `detect_localize.py` | 识别定位流水线 |
| `board_touch.py` | 棋盘看一眼 + 触摸 |
| `tcp_calib.py` / `spatial_points.py` / `work_pose.py` | TCP、空间点、工作姿态 |
| `lift_*.py` | 立柱零位与闭环 |
| `runtime_config.py` | 可锁定的成套配置 |
| `config/` | `g1d.json`、URDF、TCP/工作姿态/空间点/立柱零位等 |

## 数据落在哪

| 内容 | 位置 |
|------|------|
| 姿态集、补偿文件 | `../arm_payload_gravity/config/` |
| 负载测量、到位对比 | `../arm_payload_gravity/data/`（一般不进 git） |
| 手眼会话与结果 | `../hand_eye/data/`、`../hand_eye/config/results/` |
| TCP / 工作姿态 / 空间点 / 立柱零位 | `config/tcp_results/`、`config/work_poses/`、`config/spatial_points/`、`config/lift_zero.json` |
