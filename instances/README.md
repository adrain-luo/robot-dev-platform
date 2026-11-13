# 机器实例

每台机器一个目录 `instances/<名字>/`，放这台机器自己的设置和标定数据。
机型（关节表、URDF 等）在 `src/robot_platform/robots/<机型>/`，两台同型号机器共用机型、各有实例。

## 布局

```
instances/<名字>/
├── instance.json        机型名 robot、网卡、DDS 域、默认限速
├── calib/               标定结果（进 git）
│   ├── tcp/             TCP，_active.json 记各臂当前调用
│   ├── work_poses/      工作姿态，按 TCP 分目录
│   ├── spatial_points/  空间点，_active.json 记当前选中
│   ├── handeye/         命名的手眼结果（json + 配套双目 yaml）
│   ├── payloads/        末端负载补偿，_last_left/right.json 记各臂当前应用
│   ├── posesets/        姿态集
│   └── single_poses/    单姿态（一个名字一只手一个姿态，控制页「单姿态记录和到位」）
├── state/               当前状态（进 git）
│   ├── handeye.json     相机地址、当前手眼 / 双目 yaml、采集参数
│   ├── lift_zero.json   立柱最低点
│   ├── runtime_lock.json
│   └── pose_ik_mode.json
└── data/                采集与测量（不进 git）
    ├── handeye/         手眼采集会话
    ├── clouds/          导出的点云
    ├── board_touch/     棋盘触摸日志
    ├── measure/         负载测量会话
    └── validate/        到位对比
```

数据文件里记的路径（如手眼结果绑定的双目 yaml）存成相对实例目录的形式，整个目录挪到别处也能用。

## 选哪个实例

依次看：`serve.py --instance <名字>`、环境变量 `ARM_INSTANCE`、`instances/default` 文件（一行名字，只在本机，不进 git）。
都没有时启动会报错并列出已有实例。

`--backend mock` 不指定实例时用一个空的临时实例，退出即删，不碰任何真实机器的数据。

## 新机器

```bash
mkdir -p instances/<名字>
echo '{"robot": "g1d", "network_interface": "enp86s0"}' > instances/<名字>/instance.json
echo <名字> > instances/default        # 在那台机器上
```

## 从旧布局迁移

改成实例之前，数据散在 `arm_control/config/`、`hand_eye/config*`、`arm_payload_gravity/config/` 和各模块的 `data/` 里。
`tools/migrate_to_instance.py` 按旧位置搬进实例，并把数据文件里的旧绝对路径改成相对路径：

```bash
python tools/migrate_to_instance.py <名字> --from <旧布局目录> --move-data --set-default --dry-run   # 先看
python tools/migrate_to_instance.py <名字> --from <旧布局目录> --move-data --set-default
```

脚本只复制标定 / 状态文件、搬走 data，不删旧位置的东西；核对无误后再自己清理。
