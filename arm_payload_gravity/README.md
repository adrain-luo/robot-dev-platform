# arm_payload_gravity（G1D）

末端负载辨识 → 重力前馈。发令走 `../arm_control` 网页，本目录不单独起服务。

当前入口：`arm_control` 网页。辨识、补偿文件库、到位对比（基线/补偿/摩擦带修正）都在同一服务。测量会话 `../arm_control/data/measure/`，对比 `../arm_control/data/validate/`。
