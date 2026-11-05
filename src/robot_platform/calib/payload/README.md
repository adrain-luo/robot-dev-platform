# calib/payload（负载与姿态集）

末端负载辨识、示教姿态集、到位对比。不单独起服务，由 `web/server.py` 挂上本目录的接口。

只有 `hw/` 的后端发 DDS。本目录通过它的控制循环走点、采样、改前馈。

- `identify.py` / `poses.py` / `friction.py` / `validate.py` — 辨识、姿态集、摩擦带、对比
- `api.py` — 这些功能的 HTTP 接口
- 实例的 `calib/posesets/`、`calib/payloads/` — 姿态集和补偿文件
- 实例的 `data/measure/`、`data/validate/` — 测量会话和对比记录

页面在 `web/static/`，不在本目录。
