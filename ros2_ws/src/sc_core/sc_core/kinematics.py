"""Serial-chain kinematics read from a URDF string (numpy + xml only).

Used to place the FR3 gripper on the handle at start-up (step 3) and as a
reference for the Cartesian controller (step 4). Revolute, continuous,
prismatic and fixed joints are supported; mimic joints are ignored.

    chain = Chain.from_urdf(urdf_xml, 'fr3_link0', 'fr3_hand_tcp')
    T = chain.fk(q)                  # 4x4 pose of the tip in the base frame
    J = chain.jacobian(q)            # 6xn, rows = (linear, angular), base frame
    q, ok = chain.ik(T_target, q_seed)
"""

import xml.etree.ElementTree as ET

import numpy as np


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle_matrix(axis, angle):
    k = np.asarray(axis, dtype=float)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


def transform(R=None, p=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = p
    return T


def rotation_error(R_target, R):
    """Rotation vector w (base frame) with R_target = exp([w]) R."""
    R_err = R_target @ R.T
    cos = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos)
    v = np.array([R_err[2, 1] - R_err[1, 2], R_err[0, 2] - R_err[2, 0], R_err[1, 0] - R_err[0, 1]])
    if angle < 1e-9:
        return 0.5 * v
    if np.pi - angle < 1e-6:                      # 180 deg: axis from the symmetric part
        A = (R_err + np.eye(3)) / 2.0
        axis = A[:, np.argmax(np.diag(A))]
        return angle * axis / np.linalg.norm(axis)
    return angle / (2.0 * np.sin(angle)) * v


class Joint:

    def __init__(self, name, jtype, T_origin, axis, lower, upper):
        self.name, self.type = name, jtype
        self.T_origin = T_origin
        self.axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
        self.lower, self.upper = lower, upper

    @property
    def movable(self):
        return self.type != 'fixed'

    def motion(self, q):
        if self.type in ('revolute', 'continuous'):
            return transform(R=axis_angle_matrix(self.axis, q))
        if self.type == 'prismatic':
            return transform(p=self.axis * q)
        return np.eye(4)


class Chain:

    def __init__(self, joints):
        self.joints = joints
        self.movable = [j for j in joints if j.movable]
        self.names = [j.name for j in self.movable]
        self.lower = np.array([j.lower for j in self.movable])
        self.upper = np.array([j.upper for j in self.movable])

    @classmethod
    def from_urdf(cls, urdf_xml, base, tip):
        root = ET.fromstring(urdf_xml)
        by_child = {}
        for j in root.findall('joint'):
            by_child[j.find('child').get('link')] = j
        chain, link = [], tip
        while link != base:
            if link not in by_child:
                raise ValueError('no path from %s to %s' % (base, tip))
            j = by_child[link]
            chain.append(j)
            link = j.find('parent').get('link')
        joints = []
        for j in reversed(chain):
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            a = j.find('axis')
            axis = [float(v) for v in a.get('xyz').split()] if a is not None else [1.0, 0.0, 0.0]
            lim = j.find('limit')
            lower = float(lim.get('lower', -np.inf)) if lim is not None else -np.inf
            upper = float(lim.get('upper', np.inf)) if lim is not None else np.inf
            if j.get('type') == 'continuous':
                lower, upper = -np.inf, np.inf
            joints.append(Joint(j.get('name'), j.get('type'), transform(rpy_matrix(*rpy), xyz),
                                axis, lower, upper))
        return cls(joints)

    @property
    def dof(self):
        return len(self.movable)

    def _frames(self, q):
        """Pose of every movable joint frame (after origin) and of the tip."""
        T, k, frames = np.eye(4), 0, []
        for j in self.joints:
            T = T @ j.T_origin
            if j.movable:
                frames.append((j, T.copy()))
                T = T @ j.motion(q[k])
                k += 1
        return frames, T

    def fk(self, q):
        return self._frames(np.asarray(q, dtype=float))[1]

    def jacobian(self, q):
        frames, T_tip = self._frames(np.asarray(q, dtype=float))
        p_tip = T_tip[:3, 3]
        J = np.zeros((6, self.dof))
        for k, (j, T) in enumerate(frames):
            a = T[:3, :3] @ j.axis
            if j.type == 'prismatic':
                J[:3, k] = a
            else:
                J[:3, k] = np.cross(a, p_tip - T[:3, 3])
                J[3:, k] = a
        return J

    def ik(self, T_target, q_seed, max_iter=300, tol_pos=1e-4, tol_rot=1e-3,
           damping=1e-2, step=0.5, posture_gain=0.05):
        """Damped least-squares pose IK with joint limits and a weak pull
        towards q_seed in the null space. Returns (q, converged)."""
        q = np.clip(np.asarray(q_seed, dtype=float), self.lower, self.upper)
        q_seed = q.copy()
        for _ in range(max_iter):
            T = self.fk(q)
            e = np.concatenate([T_target[:3, 3] - T[:3, 3],
                                rotation_error(T_target[:3, :3], T[:3, :3])])
            if np.linalg.norm(e[:3]) < tol_pos and np.linalg.norm(e[3:]) < tol_rot:
                return q, True
            J = self.jacobian(q)
            JJt = J @ J.T + damping ** 2 * np.eye(6)
            J_pinv = J.T @ np.linalg.solve(JJt, np.eye(6))
            dq = J_pinv @ e + (np.eye(self.dof) - J_pinv @ J) @ (posture_gain * (q_seed - q))
            q = np.clip(q + step * dq, self.lower, self.upper)
        return q, False

    def ik_restarts(self, T_target, q_seed, restarts=30, seed=0, margin=0.05, **kw):
        """ik() from q_seed, then from random configurations (deterministic
        rng) until one converges. Among converged solutions the one with the
        largest distance to the joint limits wins; returns (q, converged)."""
        rng = np.random.default_rng(seed)
        lo, hi = self.lower + margin, self.upper - margin
        best, best_margin = None, -np.inf
        q_fallback, _ = self.ik(T_target, q_seed, **kw)
        for k in range(restarts + 1):
            start = q_seed if k == 0 else rng.uniform(lo, hi)
            q, ok = self.ik(T_target, start, **kw)
            if ok:
                m = self.limit_margin(q)
                if m > best_margin:
                    best, best_margin = q, m
                if k == 0 and m > margin:        # seed solution is fine as is
                    break
        return (best, True) if best is not None else (q_fallback, False)

    def limit_margin(self, q):
        """Smallest distance [rad] of any joint to its limits."""
        return float(np.min(np.minimum(q - self.lower, self.upper - q)))
