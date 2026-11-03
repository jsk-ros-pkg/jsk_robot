#!/usr/bin/env python3
"""Close kinematic loops at runtime by numerical IK (pure-numpy URDF FK).

A URDF is a tree, so a closed linkage (four-bar / parallel mechanism) has its
loops cut and robot_state_publisher honours the cut joints' <mimic> only
linearly -- which a real loop is not.  This node parses the URDF and, on every
joint-state, sets the DRIVEN (independent) joints, then solves (damped
least-squares, sub-stepped to stay on the assembly branch) the DEPENDENT joints
so each cut loop closes (the two link points the dropped hinge joined coincide),
and republishes the corrected joint_states.  Exact and
general: any single-DOF-per-loop linkage, planar or spatial.  Needs only rclpy,
sensor_msgs, numpy and yaml -- no extra install.

Config (config/loop_closures.yaml): {independent:[...], dependent:[...],
closures:[{link_a, link_b, point:[xyz in base_link], axis:[xyz]}]}.
"""
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _rpy_matrix(rpy):
    rx, ry, rz = rpy
    cx, cy, cz = np.cos(rx), np.cos(ry), np.cos(rz)
    sx, sy, sz = np.sin(rx), np.sin(ry), np.sin(rz)
    rxm = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    rym = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rzm = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    m = np.eye(4)
    m[:3, :3] = rzm @ rym @ rxm
    return m


class _FK:
    """Minimal URDF forward kinematics (revolute/continuous/prismatic/fixed)."""

    def __init__(self, urdf_path):
        root = ET.parse(urdf_path).getroot()
        self.j = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = ([float(x) for x in o.get("xyz", "0 0 0").split()]
                   if o is not None else [0, 0, 0])
            rpy = ([float(x) for x in o.get("rpy", "0 0 0").split()]
                   if o is not None else [0, 0, 0])
            ax = j.find("axis")
            axis = (np.array([float(x) for x in ax.get("xyz").split()])
                    if ax is not None else np.array([0.0, 0.0, 1.0]))
            t = _rpy_matrix(rpy)
            t[:3, 3] = xyz
            self.j[j.get("name")] = (j.get("type"),
                                     j.find("child").get("link"),
                                     j.find("parent").get("link"), t, axis)
        self.cj = {v[1]: n for n, v in self.j.items()}

    @staticmethod
    def _rot(a, q):
        a = a / (np.linalg.norm(a) or 1.0)
        x, y, z = a
        c, s = np.cos(q), np.sin(q)
        cc = 1.0 - c
        return np.array([[c + x * x * cc, x * y * cc - z * s, x * z * cc + y * s, 0],
                         [y * x * cc + z * s, c + y * y * cc, y * z * cc - x * s, 0],
                         [z * x * cc - y * s, z * y * cc + x * s, c + z * z * cc, 0],
                         [0, 0, 0, 1.0]])

    def _tj(self, n, q):
        typ, _c, _p, t, axis = self.j[n]
        if typ in ("revolute", "continuous"):
            return t @ self._rot(axis, q)
        if typ == "prismatic":
            m = np.eye(4)
            m[:3, 3] = axis / (np.linalg.norm(axis) or 1.0) * q
            return t @ m
        return t

    def world(self, link, q):
        chain, ln = [], link
        while ln in self.cj:
            n = self.cj[ln]
            chain.append(n)
            ln = self.j[n][2]
        m = np.eye(4)
        for n in reversed(chain):
            m = m @ self._tj(n, q.get(n, 0.0))
        return m


class LoopClosureRelay(Node):
    def __init__(self):
        super().__init__("loop_closure_relay")
        self.declare_parameter("urdf_file", "")
        self.declare_parameter("config_file", "")
        self.fk = _FK(self.get_parameter("urdf_file").value)
        cfg = {}
        path = self.get_parameter("config_file").value
        if path:
            with open(path) as f:
                cfg = yaml.safe_load(f) or {}
        self.deps = list(cfg.get("dependent", []))
        self.indep = set(cfg.get("independent", []))
        self.wit = []
        for c in cfg.get("closures", []):
            la, lb = c["link_a"], c["link_b"]
            t0a, t0b = self.fk.world(la, {}), self.fk.world(lb, {})
            p = np.asarray(c["point"], float)
            a = np.asarray(c["axis"], float)
            a = a / (np.linalg.norm(a) or 1.0)
            for q in (p, p + a * 0.03):
                la_loc = (np.linalg.inv(t0a) @ np.append(q, 1.0))[:3]
                lb_loc = (np.linalg.inv(t0b) @ np.append(q, 1.0))[:3]
                self.wit.append((la, la_loc, lb, lb_loc))
        self._x = np.zeros(len(self.deps))        # warm start (tracks branch)
        self._indep_prev = None                   # previous driven-joint values
        if not self.deps or not self.wit:
            self.get_logger().warn("no loop closures loaded; relaying unchanged")
        self._pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(JointState, "joint_states_source",
                                 self._cb, 10)

    def _resid(self, q):
        out = []
        for la, lal, lb, lbl in self.wit:
            wa = (self.fk.world(la, q) @ np.append(lal, 1.0))[:3]
            wb = (self.fk.world(lb, q) @ np.append(lbl, 1.0))[:3]
            out.append(wa - wb)
        return np.concatenate(out) if out else np.zeros(0)

    def _solve(self, q):
        x0 = self._x.copy()
        x = x0.copy()
        n = len(self.deps)
        for _ in range(50):
            for i, dn in enumerate(self.deps):
                q[dn] = x[i]
            r = self._resid(q)
            if r.size == 0 or np.linalg.norm(r) < 1e-10:
                break
            jac = np.zeros((r.size, n))
            for i, dn in enumerate(self.deps):
                xb = x[i]
                q[dn] = xb + 1e-6
                jac[:, i] = (self._resid(q) - r) / 1e-6
                q[dn] = xb
            # damped least squares (Levenberg-Marquardt) + a step clamp keep the
            # solver on the CURRENT assembly branch: near a singularity (a
            # four-bar toggle) plain Gauss-Newton jumps/spins to the other mode
            dx = np.linalg.solve(jac.T @ jac + 1e-6 * np.eye(n), -jac.T @ r)
            s = float(np.linalg.norm(dx))
            if s > 0.15:
                dx *= 0.15 / s
            x = x + dx
        # at / past a toggle the loop has NO solution; rather than accept a
        # diverged (wrapped) config that warm-start would then lock in forever,
        # hold the last valid pose -- the linkage just stops at its limit
        for i, dn in enumerate(self.deps):
            q[dn] = x[i]
        if np.linalg.norm(self._resid(q)) > 1e-5:
            x = x0
        self._x = x
        return x

    def _cb(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if not self.deps or not self.wit:
            self._pub.publish(msg)
            return
        target = {n: pos[n] for n in self.indep if n in pos}
        # SUB-STEP the driven joints from their previous values: a big jump (a
        # fast slider drag, or clicking a far value) solved in one shot walks to
        # the WRONG assembly branch and sticks; stepping it in small increments
        # keeps the warm-started solver on the current branch.
        prev = self._indep_prev if self._indep_prev is not None else target
        maxd = max((abs(target[k] - prev.get(k, target[k])) for k in target),
                   default=0.0)
        steps = max(1, int(maxd / 0.05))          # ~3 deg per sub-step
        x = self._x
        for s in range(1, steps + 1):
            f = s / steps
            q = {k: prev.get(k, target[k])
                 + (target[k] - prev.get(k, target[k])) * f for k in target}
            x = self._solve(q)
        self._indep_prev = dict(target)
        for i, dn in enumerate(self.deps):
            pos[dn] = float(x[i])
        out = JointState()
        out.header = msg.header
        out.name = list(pos.keys())
        out.position = [float(pos[n]) for n in out.name]
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LoopClosureRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
