"""Grasp pose of the handle along the mechanism's path.

Matches sc_description/urdf/mechanism.urdf.xacro: the mechanism frame M sits
at the grasp point on the closed handle (q = 0), +x_M is the opening
direction; a door's hinge is at (-h, -r, 0) in M with axis -z_M.

Grasp convention (Franka hand TCP frame): the approach axis z_tcp points into
the mechanism (-x_M) and the fingers close vertically (y_tcp = world z), so a
horizontal bar handle sits between the finger pads.
"""

import numpy as np

HANDLE_STANDOFF = 0.05   # h in mechanism.urdf.xacro


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def mechanism_frame(x, y, z, yaw):
    T = np.eye(4)
    T[:3, :3] = rot_z(yaw)
    T[:3, 3] = [x, y, z]
    return T


def grasp_rotation(R_M):
    """TCP orientation for a mechanism frame with rotation R_M."""
    z_tcp = -R_M[:, 0]
    y_tcp = np.array([0.0, 0.0, 1.0])
    x_tcp = np.cross(y_tcp, z_tcp)
    return np.column_stack([x_tcp, y_tcp, z_tcp])


def handle_pose(kind, x, y, z, yaw, q=0.0, radius=0.6, h=HANDLE_STANDOFF):
    """World pose (4x4) of the grasp point after the mechanism opened by q
    (metres for the drawer, radians for the door), with the grasp orientation."""
    T_M = mechanism_frame(x, y, z, yaw)
    T_local = np.eye(4)
    if kind == 'drawer':
        T_local[:3, 3] = [q, 0.0, 0.0]
    elif kind == 'door':
        hinge = np.array([-h, -radius, 0.0])
        R = rot_z(-q)                          # hinge axis is -z_M
        T_local[:3, :3] = R
        T_local[:3, 3] = hinge + R @ (-hinge)
    else:
        raise ValueError('kind must be drawer or door')
    T = T_M @ T_local
    T[:3, :3] = grasp_rotation(T[:3, :3])
    return T


def handle_tangent(kind, yaw, q=0.0, radius=0.6, h=HANDLE_STANDOFF):
    """Unit opening direction d* of the handle in the world frame."""
    if kind == 'drawer':
        return rot_z(yaw) @ np.array([1.0, 0.0, 0.0])
    # handle at (h, r) from the hinge; d/dq of Rz(-q) (h, r) is Rz(-q) (r, -h)
    d = rot_z(yaw) @ rot_z(-q) @ np.array([radius, -h, 0.0])
    return d / np.linalg.norm(d)


# The hand is symmetric about its approach axis: rotating the TCP by pi about
# z_tcp gives the same grasp but a very different joint-7 angle.
FLIP_Z = np.diag([-1.0, -1.0, 1.0, 1.0])


def solve_grasp(chain, T_handle, q_seed, path=None, restarts=20, seed=0, min_margin=0.1):
    """Joint configuration placing the TCP on the handle.

    Collects IK solutions for both symmetric hand orientations (from q_seed
    and from random starts). With `path`, a list of handle poses along the
    opening the task needs (each with the grasp orientation), solutions are
    ranked by how far they can follow it continuously; ties go to the
    solution closest to q_seed (a natural posture, so the arm does not swing
    round through the mechanism), and solutions within min_margin rad of a
    joint limit rank last. Returns (q, flipped, reach) where reach is the
    number of path poses followed; q is None if no solution converged."""
    rng = np.random.default_rng(seed)
    q_seed = np.asarray(q_seed, dtype=float)
    best, best_key = None, None
    for flipped in (False, True):
        F = FLIP_Z if flipped else np.eye(4)
        for k in range(restarts + 1):
            start = q_seed if k == 0 else rng.uniform(chain.lower + 0.05, chain.upper - 0.05)
            q, ok = chain.ik(T_handle @ F, start)
            if not ok:
                continue
            reach = len(follow_path(chain, [T @ F for T in path], q)) if path else 0
            key = (reach, chain.limit_margin(q) >= min_margin, -np.linalg.norm(q - q_seed))
            if best_key is None or key > best_key:
                best, best_key = (q, flipped, reach), key
    return best if best is not None else (None, False, 0)


def follow_path(chain, poses, q_start, max_step=0.3):
    """IK along a list of TCP poses, each solved from the previous solution
    (continuous motion). Stops early if IK fails or a joint would jump by
    more than max_step rad. Returns the list of configurations reached."""
    qs, q = [], np.asarray(q_start, dtype=float)
    for T in poses:
        q_new, ok = chain.ik(T, q)
        if not ok or np.abs(q_new - q).max() > max_step:
            break
        qs.append(q_new)
        q = q_new
    return qs
