# arm_payload_gravity（G1D）

末端负载辨识 → 重力前馈。发令走 `../arm_control` 网页，本目录不单独起服务。

当前入口：`arm_control` 的「末端负载辨识」面板（α=1，不拟合臂自重比例）。测量会话在 `../arm_control/data/measure/`。

对齐参考 `../../arm_payload_gravity`（H2）。尚未做到位对比与迭代辨识。
