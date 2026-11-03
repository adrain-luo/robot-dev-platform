# 硬件后端

`ArmLoop`、`server.py`、立柱闭环只通过 `base.py` 里的接口碰硬件，具体接法由后端决定。

| 接口 | 做什么 | 谁在用 |
| --- | --- | --- |
| `StateSource` | 读手臂 / 指定电机的 q、dq、τ，全电机快照，数据时效 | 页面读数、`ArmLoop`、负载与手眼采集 |
| `Commander` | `prepare()` 接管前准备；`write()` 发一帧全电机指令（下标即电机号） | `ArmLoop` |
| `LiftReader` / `LiftDriver` | 立柱高度读数 / 速度指令 `height_adjust(vz)` | 立柱闭环 `lift_control.py` |

指令帧怎么拼（哪些电机保持实测角、腰锁住、当前臂跟指令、拖拽卸力、重力前馈）在 `ArmLoop._publish` 里，与后端无关；后端只负责把这一帧发出去。

## 现有后端

| 名字 | 说明 |
| --- | --- |
| `unitree_lowcmd` | G1D：接管前 `ReleaseMode` 退出高层运动，之后由本程序发全身 `rt/lowcmd`；状态 `rt/lowstate`；立柱 `rt/hispeed_state` + `AgvClient.HeightAdjust` |
| `mock` | 仿真机器人：电机一阶跟随指令角，拖拽时停住；立柱按速度积分。不依赖 SDK / DDS，Windows 上也能跑 |

机型默认后端写在 `robots/<机型>/robot.json` 的 `control.backend`；`server.py --backend mock` 或环境变量 `ARM_BACKEND` 可覆盖。mock 后端下运动按钮默认可用。

## 本地开发

```bash
conda env create -f environment.yml   # cyclonedds 在 Windows 装不上可以跳过，mock 用不到
python arm_control/server.py --backend mock --port 10184
python -m pytest tests                 # 指令帧逐电机核对 + 整套 HTTP 接口
```

注意：mock 下页面里的「保存 / 应用」类操作（TCP、负载、空间点、立柱零位等）仍写仓库里的 config 文件，测试只调不落盘的接口。

## 新增后端

1. 在本目录实现 `Backend`（至少 `state()`、`commander()`；有立柱再实现 `lift_reader()` / `lift_driver()`）。
2. 在 `__init__.py` 的 `create_backend` 里登记名字。
3. 在 `tests/` 里补对应的指令帧测试。
