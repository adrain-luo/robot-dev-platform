# hand_eye（G1D）

眼在手外：头相机固定，标定板装在手上。输出是相机到 `torso_link` 的 `T_cam2base`。

点从姿态集来。记录一次，下次「采集当前集」会自己走完。对侧用姿态集的镜像，末端是该臂的腕 yaw（`left_wrist_yaw_link` / `right_wrist_yaw_link`），没有工具连杆。关节从 `arm_control` 已有的低层状态读，这里不发 DDS，也不另开网页。

## 网页

和臂控同一页。姿态集在「姿态集」卡片里记录；「镜像到对侧」得到另一侧的同一套空间姿态。手眼卡片点「采集当前集」后，按限速走完该集，每个点停稳存图。控制臂要和姿态集是同一侧，并且已经接管、不在拖拽。

求解先用该会话的左右图估计内参，写成会话目录里的 `stereo_calibration.yaml`，再用它做手眼。格子边长仍然要填。不必再贴 7 月那份旧 yaml。

相机主机默认 `192.168.123.164`（请求口 60000 有响应）。本机 `127.0.0.1:60000` 没有相机进程。teleimager 不在 conda `abyss` 里，`config.json` 的 `teleimager_path` 指向本机源码副本，可改。

## 命令行

在 `conda activate abyss` 之后：

```bash
python solve_handeye.py --data data/会话名 --square-size 25
```

不传 `--intrinsics` 时，先用该目录的左右图估计内参。`--urdf` 默认用 `arm_control/config/g1_d.urdf`。末端默认读会话里的 `tip_link`。

不要把旧工程里右手、`right_dex1_tool_link` 的 `T_cam2base` 数字抄过来。
