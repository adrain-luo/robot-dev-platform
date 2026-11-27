# 硬件后端

`control/arm_loop.py`、`web/server.py`、立柱闭环只通过 `base.py` 里的接口碰硬件，具体接法由后端决定。

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
| `guardian` | 手臂指令经本体守护层 `g1_guardian` 发出（UDP 设定），**本程序不写 `rt/lowcmd`**；状态、立柱、底盘、IMU 照旧由读后端（G1D 为 `unitree_lowcmd`）直接读 DDS。见下文 |
| `mock` | 仿真机器人：电机一阶跟随指令角，拖拽时停住；立柱按速度积分。不依赖 SDK / DDS，Windows 上也能跑 |

机型默认后端写在 `robots/<机型>/robot.json` 的 `control.backend`；`serve.py --backend mock` 或环境变量 `ARM_BACKEND` 可覆盖。mock 后端下运动按钮默认可用。

## guardian：经本体守护层发令

```bash
# 本体上先起守护层（见 src/g1_guardian/README.md）
python3 -m g1_guardian.service --config g1d --bind 192.168.123.164 --net eth0 --sdk-path ~/unitree_sdk2_python
# 平台改走守护层
ARM_BACKEND=guardian python serve.py --allow-real-motion      # 或 --backend guardian
```

ArmLoop 一行没改发帧逻辑：它照旧每拍拼一整帧，`GuardianCommander` 只是把帧里
`right_arm` / `left_arm` / `waist` 各组的角度、前馈和刚度比例（帧里的 kp ÷ 机型 kp，拖拽时为 0，钳在 [0, 1]）
转成守护层设定，有效期 100 ms。于是：

| 情况 | 结果 |
| --- | --- |
| 平台循环卡顿 > 100 ms | 守护层减速、原地保持；卡顿过去自动继续 |
| 平台进程卡死 / 崩溃 / 网络断 | 守护层锁存 `link_lost`、原地保持（不会掉）；页面显示原因，**重新接管**后从实测角继续 |
| 守护层急停（任何客户端、按钮） | 平台停止往前走指令角、不能走点；接管时提示复位，人在弹窗里确认后带一次性确认令牌调 `/api/guardian/reset`（先要令牌：`/api/guardian/reset/request`，见 `web/reset_confirm.py`），复位并从实测角重新接管 |
| 别的程序已占用守护层 | 拒绝接管，提示等它释放 |
| 守护层的配置与本机型不符（机型、通道、每只手的电机、锁腰电机） | 拒绝接管 |
| 拖拽 | 该组 `kp_scale = 0`：守护层的指令角贴着实测走、阻尼换成 `TEACH_KD`，松手后停在松手处 |
| 释放 / 服务退出 | 守护层约 3 s 把刚度降到 0 再停发 |

页面上「指令话题」胶囊改为显示守护层（发帧频率、有没有别的写入者由守护层自己检测）。
设置在 `robot.json` 的 `control.guardian`，环境变量 `ARM_GUARDIAN_HOST / ARM_GUARDIAN_PORT / ARM_GUARDIAN_CLIENT / ARM_GUARDIAN_READ` 可覆盖。
平台的「急停」按钮仍是平台级的保持（不锁存守护层）；要锁存的急停用 `python -m g1_guardian.client estop` 或以后的实体按钮。

测试：`tests/test_guardian_backend.py` 用进程内真的守护层（真 UDP）驱动 mock 机器人，跑通整条链路；
另有一条测试逐电机核对守护层配置与机型（增益、分组、前馈上限、拖拽阻尼、腿部空闲电机）。

## 本地开发

```bash
conda env create -f environment.yml   # cyclonedds 在 Windows 装不上可以跳过，mock 用不到
python serve.py --backend mock --port 10184
python -m pytest tests                 # 指令帧逐电机核对 + 整套 HTTP 接口
```

注意：mock 下页面里的「保存 / 应用」类操作（TCP、负载、空间点、立柱零位等）仍写仓库里的 config 文件，测试只调不落盘的接口。

## 新增后端

1. 在本目录实现 `Backend`（至少 `state()`、`commander()`；有立柱再实现 `lift_reader()` / `lift_driver()`）。
2. 在 `__init__.py` 的 `create_backend` 里登记名字。
3. 在 `tests/` 里补对应的指令帧测试。
