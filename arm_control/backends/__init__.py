"""硬件后端：按名字创建。

    unitree_lowcmd  退出高层运动后发全身 rt/lowcmd（G1D）
    unitree_arm_sdk 运控保持站立，只经 rt/arm_sdk 接管腰与双臂（G1）
    mock            仿真机器人，不接硬件
"""

from __future__ import annotations

from backends.base import Backend

BACKENDS = ("unitree_lowcmd", "unitree_arm_sdk", "mock")


def create_backend(name: str, settings) -> Backend:
    profile = settings.profile
    if name == "unitree_lowcmd":
        from backends.unitree_lowcmd import UnitreeLowCmdBackend

        return UnitreeLowCmdBackend(
            profile,
            network_interface=settings.network_interface,
            dds_domain=settings.dds_domain,
            sdk_path=settings.unitree_sdk_path,
            timeout_s=settings.lowstate_timeout_s,
        )
    if name == "unitree_arm_sdk":
        from backends.unitree_arm_sdk import UnitreeArmSdkBackend

        return UnitreeArmSdkBackend(
            profile,
            network_interface=settings.network_interface,
            dds_domain=settings.dds_domain,
            sdk_path=settings.unitree_sdk_path,
            timeout_s=settings.lowstate_timeout_s,
        )
    if name == "mock":
        from backends.mock import MockBackend

        return MockBackend(profile)
    raise ValueError(f"没有后端 {name!r}（可选：{', '.join(BACKENDS)}）")
