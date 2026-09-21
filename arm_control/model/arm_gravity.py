"""URDF arm gravity torque (alpha=1). Numpy only.

tau_i = -sum_j m_j * g · (z_i × (p_cj - p_i)) in torso_link.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

from model.layout import BASE_LINK, get_arm
from model.urdf_fk import URDFRobotModel, axis_angle_to_matrix, make_transform

G_ACCEL = 9.81


@dataclass
class _Joint:
    name: str
    jtype: str
    parent: str
    child: str
    T_origin: np.ndarray
    axis: np.ndarray


def _parse_urdf(urdf_path: Path):
    root = ET.parse(str(urdf_path)).getroot()
    joints: Dict[str, _Joint] = {}
    children: Dict[str, List[_Joint]] = {}
    for el in root.findall("joint"):
        o = el.find("origin")
        xyz = [float(v) for v in (o.attrib.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.attrib.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        a = el.find("axis")
        axis = np.array([float(v) for v in a.attrib.get("xyz", "1 0 0").split()]) if a is not None else np.array([1.0, 0, 0])
        n = float(np.linalg.norm(axis))
        axis = axis / n if n > 1e-12 else np.array([0.0, 0.0, 1.0])
        parent_el, child_el = el.find("parent"), el.find("child")
        if parent_el is None or child_el is None:
            continue
        j = _Joint(
            el.attrib["name"],
            el.attrib.get("type", "fixed"),
            parent_el.attrib["link"],
            child_el.attrib["link"],
            make_transform(xyz, rpy),
            axis,
        )
        joints[j.name] = j
        children.setdefault(j.parent, []).append(j)
    inertials: Dict[str, Tuple[float, np.ndarray]] = {}
    for el in root.findall("link"):
        ine = el.find("inertial")
        if ine is None:
            continue
        m_el = ine.find("mass")
        mass = float(m_el.attrib.get("value", 0.0)) if m_el is not None else 0.0
        if mass <= 0:
            continue
        o = ine.find("origin")
        com = np.array([float(v) for v in (o.attrib.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
        inertials[el.attrib["name"]] = (mass, com)
    return joints, children, inertials


class ArmGravity:
    def __init__(self, arm: str, urdf_path: Path, base_link: str = BASE_LINK):
        layout = get_arm(arm)
        self.arm = layout.name
        self.joint_names = list(layout.joint_names)
        self.tip_link = layout.tip_link
        self.n = layout.n
        self.base_link = base_link
        joints, children, inertials = _parse_urdf(Path(urdf_path))
        self._joints = joints
        self._children = children
        missing = [n for n in self.joint_names if n not in joints]
        if missing:
            raise ValueError(f"URDF missing joints {missing}")
        self._chain = [joints[n] for n in self.joint_names]
        self._T_base_firstparent = self._static_T(base_link, self._chain[0].parent)
        self._downstream: List[List[str]] = []
        for j in self._chain:
            self._downstream.append([ln for ln in self._subtree(j.child) if ln in inertials])
        self.bodies = {ln: inertials[ln] for sub in self._downstream for ln in sub}
        self._fk_root = self._chain[0]
        self.model = URDFRobotModel(str(urdf_path), base_link, self.tip_link)
        self.limits = self.model.joint_limits()
        self.moving_mass_kg = float(sum(m for m, _ in self.bodies.values()))

    def _subtree(self, link: str) -> List[str]:
        out, stack = [], [link]
        while stack:
            ln = stack.pop()
            out.append(ln)
            stack.extend(j.child for j in self._children.get(ln, []))
        return out

    def _static_T(self, from_link: str, to_link: str) -> np.ndarray:
        if from_link == to_link:
            return np.eye(4)
        stack = [(from_link, np.eye(4))]
        seen = set()
        while stack:
            ln, T = stack.pop()
            if ln == to_link:
                return T
            if ln in seen:
                continue
            seen.add(ln)
            for j in self._children.get(ln, []):
                stack.append((j.child, T @ j.T_origin))
        raise ValueError(f"URDF 中找不到 {from_link}→{to_link}")

    def _fk(self, q: np.ndarray) -> Dict[str, np.ndarray]:
        qmap = dict(zip(self.joint_names, [float(v) for v in q]))
        T0 = self._T_base_firstparent
        out: Dict[str, np.ndarray] = {self._fk_root.parent: T0}
        stack = [self._fk_root.parent]
        while stack:
            ln = stack.pop()
            Tp = out[ln]
            for j in self._children.get(ln, []):
                if ln == self._fk_root.parent and j is not self._fk_root:
                    continue
                T = Tp @ j.T_origin
                if j.jtype in ("revolute", "continuous"):
                    v = qmap.get(j.name, 0.0)
                    if v != 0.0:
                        R = axis_angle_to_matrix(j.axis, v)
                        M = np.eye(4)
                        M[:3, :3] = R
                        T = T @ M
                elif j.jtype == "prismatic":
                    T = T.copy()
                    T[:3, 3] += T[:3, :3] @ (j.axis * qmap.get(j.name, 0.0))
                out[j.child] = T
                stack.append(j.child)
        return out

    def tau_arm(self, q) -> np.ndarray:
        q = np.asarray(q, float).reshape(-1)
        if q.size != self.n:
            raise ValueError(f"需要 {self.n} 个关节角")
        g = np.array([0.0, 0.0, -G_ACCEL])
        fk = self._fk(q)
        coms = {ln: fk[ln][:3, :3] @ com + fk[ln][:3, 3] for ln, (m, com) in self.bodies.items()}
        zs, ps = [], []
        for j in self._chain:
            T = fk[j.child]
            zs.append(T[:3, :3] @ j.axis)
            ps.append(T[:3, 3])
        tau = np.zeros(self.n)
        for i, j in enumerate(self._chain):
            tot = 0.0
            for ln in self._downstream[i]:
                tot += self.bodies[ln][0] * float(g @ np.cross(zs[i], coms[ln] - ps[i]))
            tau[i] = -tot
        return tau

    def payload_regressor(self, q) -> np.ndarray:
        q = np.asarray(q, float).reshape(-1)
        g = np.array([0.0, 0.0, -G_ACCEL])
        fk = self._fk(q)
        zs, ps = [], []
        for j in self._chain:
            T = fk[j.child]
            zs.append(T[:3, :3] @ j.axis)
            ps.append(T[:3, 3])
        TL = fk[self.tip_link]
        pL, RL = TL[:3, 3], TL[:3, :3]
        Y = np.zeros((self.n, 4))
        for i in range(self.n):
            Y[i, 0] = -float(g @ np.cross(zs[i], pL - ps[i]))
            for k in range(3):
                Y[i, 1 + k] = -float(g @ np.cross(zs[i], RL[:, k]))
        return Y

    def tip_T(self, q) -> np.ndarray:
        return self.model.fk_T(q)
