"""Build a 3D skeleton from URDF FK + measured joint angles."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Sequence

import numpy as np

from robot_platform.core.layout import ArmLayout, get_arm, short_joint_label
from robot_platform.control.urdf_fk import URDFRobotModel


class ArmKinematics:
    def __init__(self, urdf_path: Path, base_link: str, layout: ArmLayout):
        self.layout = layout
        self.model = URDFRobotModel(str(urdf_path), base_link, layout.tip_link)
        if list(self.model.joint_names) != list(layout.joint_names):
            raise ValueError(
                f"{layout.name} URDF chain joints {self.model.joint_names} "
                f"!= layout {list(layout.joint_names)}"
            )

    def scene(self, q: Sequence[float]) -> Dict:
        q = np.asarray(q, dtype=float).reshape(-1)
        chain = self.model.fk_chain(q)
        links = [[0.0, 0.0, 0.0]]
        names = [self.model.base_link]
        last_T = np.eye(4)
        for link_name, T in chain:
            p = T[:3, 3]
            links.append([float(p[0]), float(p[1]), float(p[2])])
            names.append(link_name)
            last_T = T
        return {
            "arm": self.layout.name,
            "tip_link": self.layout.tip_link,
            "links": links,
            "link_names": names,
            "wrist_T": last_T.tolist(),
            "q_rad": [float(x) for x in q],
            "q_deg": [float(np.degrees(x)) for x in q],
            "labels": [short_joint_label(n) for n in self.layout.joint_names],
        }


class RobotKinematics:
    def __init__(self, urdf_path: Path, base_link: str, arm_names: Sequence[str] = ("right", "left")):
        self.urdf_path = Path(urdf_path)
        self.base_link = base_link
        self.arms: Dict[str, ArmKinematics] = {
            name: ArmKinematics(self.urdf_path, base_link, get_arm(name)) for name in arm_names
        }

    def arm_scene(self, name: str, q: Sequence[float]) -> Dict:
        return self.arms[name].scene(q)
