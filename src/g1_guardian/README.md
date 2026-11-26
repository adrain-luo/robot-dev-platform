# g1_guardian：本体执行守护层

运行在机器人本体（G1D 的 192.168.123.164）上的一个小进程，是 `rt/lowcmd`（G1D）或 `rt/arm_sdk`（G1）**唯一的写入者**。Box 上的程序（refactor 取放节点、平台）只通过 UDP 给它发关节设定，它用固定 200 Hz 的单线程循环发帧。

为什么要它：G1D 上 lowcmd 一断流手臂就掉；运控在 Box 上时，网线、交换机、Box 进程任何一处中断都等于断流。守护层放在本体，**任何上游问题都只会让手臂停住保持，不会掉**。

## 它保证什么

| 情况 | 守护层的反应 |
| --- | --- |
| 设定过期（`valid_for_ms` 内没有新设定） | 按加速度上限减速到停，原地保持；新设定到来自动继续 |
| Box 失联（`link_timeout_ms` 内没有任何消息） | 减速保持，锁存 `link_lost`；Box 重新 `engage` 后继续 |
| lowstate 过期 | 保持最后指令，锁存 `stale_lowstate` |
| 话题上出现别人的帧 | 拒绝接管；已接管时锁存 `foreign_writer` |
| 接管时 `idle_motors`（G1D：0–11 号腿部电机）有读数 | 拒绝接管（`motors_not_idle`），**在退出运控之前**就拒绝：连着的可能是站着的双足 G1，退出运控它会摔 |
| E-STOP（本机、Box 或监视地址发来，见“访问控制”） | 减速保持并锁存；设定一律拒绝，直到 `reset`；`reset` 后旧 epoch 的设定仍被拒绝，任务必须重新 `engage` |
| 设定越限 | 位置钳到限位内；与当前指令相差超过 `max_target_jump_rad` 直接拒绝；前馈钳到 `tau_ff_limit_nm` |
| 写帧异常 | 计数、下一拍重试，循环不退出 |
| `release` 或 `systemctl stop` | G1D：3 s 内把刚度和前馈降到 0（阻尼不变）再停发；G1：权重降到 0 再停发 |

**只有 `release` 会停发。** 速度上限 1.0 rad/s（与平台 ArmLoop 的硬上限一致）、加速度上限 10 rad/s²，都在 `configs/g1d.json` 里，Box 只能请求更小的刚度（`kp_scale ≤ 1`），不能请求更大的。

**刚度与拖拽。** 设定里每组可带 `kp_scale`（0–1），只在这条设定有效期内生效；守护层一旦因任何原因保持（设定过期、失联、急停、释放），各组都回到满刚度。刚度变化按 `stiffness_ramp_s`（0.25 s 走完 0↔1）渐变，不突变。`kp_scale = 0` 就是拖拽：该组的指令角跟着实测角走，阻尼换成 `compliant_kd`（2.0，同平台 `TEACH_KD`），所以手松开、刚度回来时手臂停在手放下的位置，不会弹回拖拽前的目标。

**腰按平台的锁腰增益保持**：G1D 电机 12、13 用 kp 200 / kd 4（`robot.json` 的 `waist.lock_kp / lock_kd`，平台 ArmLoop 接管时也是这么锁的），不是全身默认增益。

## 运行

```bash
# 本体上（需要 unitree_sdk2py）
python3 -m g1_guardian.service --config g1d --bind 192.168.123.164 --net eth0 \
    --sdk-path /home/unitree/unitree_sdk2_python --blackbox-dir /var/lib/g1-guardian/blackbox

# 本机或 Box 上看状态 / 操作（其他地址发的命令守护层不回应，见“访问控制”）
python3 -m g1_guardian.client --host 192.168.123.164 watch
python3 -m g1_guardian.client --host 192.168.123.164 estop
python3 -m g1_guardian.client --host 192.168.123.164 reset
python3 -m g1_guardian.client --host 192.168.123.164 release
```

systemd 单元见 `deploy/g1-guardian.service`。启动后守护层是 `idle`，**不发任何帧**，直到第一个客户端 `engage`；接管时才退出宇树运控（`ReleaseMode`），并把当前实测位姿作为保持目标。

单元的 `ExecStartPre` 跑 `python3 -m g1_guardian.writer_check`：本机 `/etc/g1/actuator-writer-units`（从 `deploy/actuator-writer-units.example` 复制后按现场改）里列出的其他写入者单元有在运行的，守护层就不启动；文件不存在也不启动。它只读单元状态，**从不停掉别的单元**（G1D 上停掉正在保持手臂的写入者就是掉臂），所以没有用 systemd 的 `Conflicts=`。它只管本机的 systemd 单元；别的主机上的写入者、手动起的程序，由“接管前听到第三方帧就拒绝接管”兜住。

`configs/g1.json`（G1，arm_sdk）是草稿：只有权重渐变，没有平台 `ArmSdkSchedule` 那样的增益过渡，从未上机。服务默认拒绝草稿配置，要加 `--allow-draft-config`。

## 协议（v1，UDP，每个数据报一个 JSON 对象）

```json
{"v":1,"type":"engage","client":"refactor-node","seq":1}
{"v":1,"type":"setpoint","client":"refactor-node","seq":2,"epoch":7,"valid_for_ms":100,
 "groups":{"right_arm":{"q":[0.2,-0.25,0.0,0.9,0.0,-0.1,0.0],"tau_ff":[0,0,0,0,0,0,0],"kp_scale":1.0}}}
{"v":1,"type":"heartbeat","client":"refactor-node","seq":3}
{"v":1,"type":"estop","client":"web","seq":1}
{"v":1,"type":"reset","client":"web","seq":2}
{"v":1,"type":"release","client":"refactor-node","seq":9}
```

守护层向最近 5 s 内发过消息的地址推送 `status`（G1D 配置 100 Hz），每条命令另有一条带 `ack` 的即时 `status`。字段见 `core.GuardianCore.status()`；`loop` 里有发帧周期的 p50 / p99 / 最大值和超时次数，可直接用作“控制周期抖动”的验收数据。

组（group）：`right_arm`（电机 22–28）、`left_arm`（15–21）、`waist`（12）。G1D 的 13 号电机没有文档说明的关节，接管时锁在当时的角度，不跟踪设定。

## 访问控制（谁能发什么）

两层，都只管 UDP 18090，报文格式不变（协议仍是 v1）：

1. **防火墙**：`deploy/g1-guardian-firewall.nft`，由 `deploy/g1-guardian-firewall.service` 开机加载。只放行本机（`lo`）和 Box（`eth0` 上的 192.168.123.5），其余丢弃。Wi-Fi 那一侧、机器人网段上的其他设备都到不了守护层。
2. **配置里的 `access`**：守护层自己按来源地址分角色判断，不依赖防火墙。配置里没有 `access` 段就不加载。

| 角色 | 地址（`configs/g1d.json`） | 能发 |
| --- | --- | --- |
| `local`（本机） | 127.0.0.1、192.168.123.164（`--bind`） | 全部；`release` 不必是拥有者（本体上的命令行） |
| `control`（Box） | 192.168.123.5 | 全部；`release` 只收当前拥有者 |
| `monitor` | 默认没有 | 收状态、`estop` |
| 其他地址 | — | 不回应，也不推状态；日志每秒最多一行 `refused_source:<地址>` |

- **拥有者 = `engage` 时的 client 名加上发 `engage` 的地址。** 别的地址抄同一个 client 名发设定、心跳或 `release` 都无效：心跳也不算拥有者还活着，真正的拥有者失联照样锁存 `link_lost`。
- 急停锁存时仍然收 `release`（负责人 2026-11-26 定），同样只收拥有者或本机。
- `monitor` 发的 `engage` 在退出宇树运控（`ReleaseMode`）之前就被拒绝。
- 改地址时三处一起改：`configs/<机型>.json` 的 `access`、`deploy/g1-guardian-firewall.nft`、`deploy/g1-guardian.service` 的 `--bind`。`tests/test_guardian_access.py` 检查它们一致。
- 挡不住的：同一网段里伪造源地址（要第 3 层报文签名，没做）；Box 上的任何程序都算 Box。

### 操作卡：装防火墙（由负责人执行）

前提：守护层是 `idle`（没接管）。装防火墙不碰发帧；但地址写错、把 Box 挡在外面时，正接管着的守护层会因失联锁存 `link_lost` 并原地保持（不掉），所以在 `idle` 时装。

1. 本体上 `nft --version`（要 0.9 以上）。没有 `nft` 就先停在这一步，把输出发回。
2. `ip -brief addr`：本体有线口是 `eth0`、地址 192.168.123.164；Box 是 192.168.123.5。不一致的话先按上面“三处一起改”。
3. 按 `deploy/g1-guardian-firewall.service` 文件头的三步安装、启用。
4. `sudo nft list table inet g1_guardian`：应有三条规则。
5. Box 上 `python3 -m g1_guardian.client --host 192.168.123.164 status` 应正常返回；从 Wi-Fi 上的另一台机器发同样的命令应超时，第 4 步里 `drop` 的计数增加。
6. 守护层启动日志里有 `access: local=[...] control=[...]`。

回滚：`sudo systemctl disable --now g1-guardian-firewall`（删掉这张表，守护层不受影响；第 2 层照样生效）。

## 上机前必须实测

在手臂有机械支撑、旁边有人能断电的前提下：

1. 启动服务，`watch` 看 `state=idle`、`lowstate_age_ms` 正常，且没有 `foreign_writer_frames`；先按上面的操作卡装好防火墙。
2. `engage`：手臂应原地不动。
3. 拔掉 Box 网线（或停 Box 上的节点）：手臂原地保持，状态变 `fault [link_lost]`。
4. `estop` → `reset` → 客户端重新 `engage`：期间手臂不动。
5. 平台经守护层接管后拖拽一只手，松手后手臂应停在松手处；拖拽中拔掉 Box 网线，手臂应在 0.25 s 内变硬、停在当时位置。
6. `release`：手臂在约 3 s 内慢慢落下，而不是瞬间掉落；检查落下速度是否可接受，必要时调 `release_ramp_s`。
7. 满负载（视觉、可视化、网页都在跑）下运行数小时，看 `loop.max_ms` 和 `overruns`。

## 测试

```bash
python -m pytest tests/test_guardian_core.py tests/test_guardian_service.py
```

只依赖标准库（核心、客户端、测试都不需要 SDK）。
