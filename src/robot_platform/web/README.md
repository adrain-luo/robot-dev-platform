# web（网页服务）

G1D 控制与标定的**唯一网页入口**。其它模块（负载、手眼、TCP…）都挂在这里，不另开页面、不另发 DDS。

## 启动

```bash
conda activate abyss
cd ~/abyss/project/robot-dev-platform

python serve.py                               # 只读，按钮灰色
python serve.py --allow-real-motion --max-speed 0.2    # 真机
# 打开 http://<本机IP>:10184/
```

常用参数：`--host` / `--port`（默认 `0.0.0.0:10184`）、`--iface`、`--max-speed`（0.02–1.0 rad/s）、`--robot`、`--backend`。

不接机器人时可用仿真：`python serve.py --backend mock`（见 [`hw/backends/README.md`](../hw/backends/README.md)）。

真机前：停掉占用 `rt/lowcmd` 的其它任务；有人扶臂、守急停。

## 网页结构（书签）

左侧偏感知，右侧偏标定/控制。卡片可多开，内容区可滚动。

| 区域 | 做什么 |
|------|--------|
| **渲染 / 双目** | 头相机或调试口匹配点云；固定场景累加 |
| **识别定位** | 各方案各占一张小卡；现有「烟盒 · YOLO（调试口）」：取图 → 点位获取（网格投影）→ 校准（贴平面剔点）→ 中心点提取 |
| **运行配置** | 手眼结果、重力补偿、TCP 成套调用；可锁定防误改 |
| **姿态集 / 负载 / 到位对比** | 记点、辨识、摩擦带对比（代码在 `calib/payload/`） |
| **手眼标定** | 按姿态集采集 → 内参 → 外参 → 命名保存/调用 |
| **TCP / 工作姿态** | TCP 管理表（应用/镜像/删除）与录入分区；工作姿态绑定 TCP 可锁定 |
| **TCP 控制**（控制） | 躯干系平移（保姿态）/ 绕红点旋转；可设步长 |
| **空间点 / 棋盘触摸**（控制） | 躯干系定点碰 TCP；「看一眼」角点后逐点摸（验手眼+TCP） |
| **关节** | 当前臂 7 轴 + 腰 12/14 读数；立柱高度与「记为最低点」 |
| **躯体控制** | 腰（滑条/步进/到目标/停止）+ 立柱行程（Agv HeightAdjust） |
| **手臂控制 / 接管** | 滑条单关节限速；拖拽卸力、急停保持、释放 |

更细的算法与文件路径见仓库根目录 [`功能说明.md`](../../../功能说明.md)。

## 发令要点

1. 徽标「可运动」→ **接管**：手臂与腰停在当前角（退出高层运动模式）。
2. 网页限速（点「设」）是同一把尺子：单点出发、姿态芯片走点、腰目标逼近都用它。拖拽卸力不限速。
3. **拖拽** = 当前臂 kp=0 + URDF 重力前馈；**急停(保持)** = 退出拖拽并锁当前角（棋盘触摸若在跑也会停）。
4. **释放** = 停发 `rt/lowcmd`（请扶住）。

腰电机按官方表：**12 = 旋转（yaw）**，**14 = 俯仰（pitch）**；13 在 G1D 无效，跟写即可。立柱高度在 `rt/hispeed_state`，不在 lowstate。

画面：灰虚线=指令，蓝虚线=目标，实线=实测。红点=当前 TCP 爪尖，黄点=选中空间点。

## 相关代码（`src/robot_platform/` 下）

| 路径 | 作用 |
|------|------|
| `web/server.py` | 入口：参数、实例 / 后端选择、起 HTTP 服务、收尾释放 |
| `web/routes.py` | 路由：GET（页面、静态、URDF、状态）与按功能分组的 POST，组的先后顺序见文件开头 |
| `web/app.py` | `App` 本体：设置、后端、控制循环、各功能会话 |
| `web/api/` | `App` 按功能拆开的接口方法：`state`（状态 / 场景）、`lift`、`tcp`、`work_pose`、`points`、`runtime` |
| `web/page.py` / `web/http_util.py` | 首页下拉框预填；HTTP 收发小工具 |
| `web/static/` | 唯一前端：`index.html`、`style.css`、三维 `viewer.js`（ES 模块） |
| `web/static/js/` | 页面脚本按功能分文件：`00-util`、`01-robot`、`02-arm`、`03-body`、`04-ui`、`05-payload`、`06-runtime`、`07-tcp`、`08-spatial`、`09-camera-handeye`、`10-render`、`11-detect`、`12-workflow`、`99-main`（启动，放最后）。服务端按文件名顺序拼成 `/static/app.js`，共用一个脚本作用域；新文件按编号插进去即可 |
| `control/arm_loop.py` | `ArmLoop` 限速、接管、腰保持、拼指令帧 |
| `hw/backends/` | 硬件后端：`unitree_lowcmd`（G1D）、`unitree_arm_sdk`（G1）、`mock`（仿真） |
| `hw/dds/` | Unitree DDS：`lowstate`、立柱 `hispeed`、Agv |
| `core/robot_profile.py` / `core/layout.py` | 机型加载、关节布局 |
| `control/urdf_fk.py` / `fk_scene.py` / `arm_gravity.py` | URDF 正运动学、重力 |
| `robots/<机型>/` | `robot.json` + URDF + 网格；字段说明见 [`robots/README.md`](../robots/README.md) |
| `perception/detect_localize.py` | 识别定位流水线 |
| `tasks/board_touch.py` | 棋盘看一眼 + 触摸 |
| `calib/tcp.py` / `tasks/spatial_points.py` / `tasks/work_pose.py` | TCP、空间点、工作姿态 |
| `calib/lift.py` / `hw/lift_control.py` | 立柱零位与闭环 |
| `core/runtime_config.py` | 可锁定的成套配置 |
| `core/instance.py` | 机器实例：本机设置与标定数据的位置，见 [`instances/README.md`](../../../instances/README.md) |

## 数据落在哪

| 内容 | 位置 |
|------|------|
| 姿态集、补偿文件 | 实例的 `calib/posesets/`、`calib/payloads/` |
| 负载测量、到位对比 | 实例的 `data/measure/`、`data/validate/`（不进 git） |
| 手眼会话与结果 | 实例的 `data/handeye/`、`calib/handeye/` |
| TCP / 工作姿态 / 空间点 / 立柱零位 | 实例的 `calib/tcp/`、`calib/work_poses/`、`calib/spatial_points/`、`state/lift_zero.json` |
