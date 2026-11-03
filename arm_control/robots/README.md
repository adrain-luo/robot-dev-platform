# 机型描述

每个机型一个目录：`robots/<名字>/robot.json`，同目录放 URDF 和 `meshes/`。
代码里与机型有关的数（关节表、电机号、增益、腰、立柱、URDF）都从这里读，
由 `model/robot_profile.py` 加载，各模块用 `active()` 取当前机型。

选哪个机型：机器实例 `instances/<实例>/instance.json` 的 `robot` 字段；也可以用环境变量 `ARM_ROBOT` 或 `server.py --robot <名字>` 覆盖。

## robot.json 字段

| 字段 | 说明 |
| --- | --- |
| `name` / `display_name` | 机型名（与目录同名）/ 显示名 |
| `urdf` | URDF 文件名，相对本目录；网格放在同目录 `meshes/` |
| `base_link` | 正运动学与手眼的基准连杆 |
| `num_motors` | LowCmd/LowState 里使用的电机数 |
| `control.backend` | 默认硬件后端，见 [`backends/README.md`](../backends/README.md)；G1D 为 `unitree_lowcmd` |
| `control.state_topic` / `command_topic` | 状态与指令话题 |
| `control.release_motion_mode` | 接管前是否 `ReleaseMode` 退出高层运动 |
| `gains.kp` / `gains.kd` | 每个电机的默认 PD，长度等于 `num_motors` |
| `arms.<臂>` | `tip_link`、`joint_names`（URDF 关节名，按链顺序）、`motor_indices`、`home_q`（新建姿态集的初始角） |
| `mirror_q_sign` | 左右镜像时每个关节角的符号；不配则不能镜像姿态集 / TCP |
| `waist`（可选） | `joints`：每个可控轴的 `role`（yaw/pitch/roll）、`urdf_joint`、`motor`、`limit_rad`；`hold_motors`：接管时一并锁住的电机；`lock_kp` / `lock_kd` |
| `lift`（可选） | 立柱：`state_topic`、`urdf_joints`（逐级伸出的棱柱）、`stage_limit_m`、`stroke_max_m`、`height_zero_m`（未标定时的最低位读数） |

没有 `waist` 或 `lift` 的机型，对应接口会返回“没有腰 / 没有立柱”，`/api/scene` 里对应字段为空。

## 新增机型

1. 建 `robots/<名字>/`，放 URDF 和 `meshes/`。
2. 照 `g1d/robot.json` 写一份，核对关节名与 URDF 一致（启动时会校验臂链）。
3. 给这台机器建实例（见 [`instances/README.md`](../../instances/README.md)），`robot` 填新机型名，先只读跑一遍。
